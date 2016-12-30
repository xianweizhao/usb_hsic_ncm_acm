/* Copyright(C) 2005-2013,CYIT. Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/cdev.h>
#include <linux/syscalls.h>
#include <linux/errno.h>
#include <linux/debugfs.h>
#include <linux/regulator/driver.h>
#include <linux/vmalloc.h>
#include <linux/p632x_cp.h>

#include <asm/uaccess.h>
#include <asm/bug.h>

#include <mach/hardware.h>

#include <linux/gpio.h>
#include <mach/gpio-exynos.h>
#include <plat/gpio-cfg.h>

#include <mach/regs-mailbox.h>
#include <mach/regs-pmu.h>

#include "c6320_cp.h"
#include "../smd/smd_public.h"

#define TAG "[CP_DRIVER] "

#define WLOCK_TIME				(10)

#define print_log(fmt, args...)\
	do{\
		if(debug_on)\
			printk(KERN_INFO TAG fmt, ##args);\
	}while(0)


typedef enum {
	CP_RESET = 0,
	CP_POWER_ON,
	CP_POWER_OFF,
	CP_RELEASE,
} cp_ctrl;

struct cp_data {
	struct cdev cdev;
	dev_t devno;

	struct class *cp_class;
	struct device *cp_dev;
	struct wake_lock cp_wakelock;
	//spinlock_t lock;

	enum c2a_status cp_status;
	struct work_struct notify_wq;
	struct timer_list c2a_status_timer;
	struct cp_platform_data *cp_pdata;

	int wd_happen;
};

static struct cp_data * cpdata;

struct work_struct pmic_work;

int panic_denote = 0;
static int debug_on  = 1;
static int cp_sleepctrl = 0;

static unsigned reg;
static unsigned val;
static struct T_C2A_STATUS_NOTIFY status_notify;

extern struct resource cp_mem_resource;
extern struct resource ipc_mem_resource;

static DEFINE_MUTEX(cp_mutex);

/* cp status change notify for other kernel module*/
static BLOCKING_NOTIFIER_HEAD(cp_chain_head);

int register_cp_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&cp_chain_head, nb);
}

int unregister_cp_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&cp_chain_head, nb);
}

static int cp_notifier_call_chain(unsigned long val)
{
	int ret = blocking_notifier_call_chain(&cp_chain_head, val, NULL);

	return notifier_to_errno(ret);
}

static inline void set_id(void)
{
	*machine_id_addr = get_boardid_for_cp();
	*board_id_addr = get_hardwareid();

	printk("modem_id-->0x%x, board_id-->%d\n",
		*machine_id_addr, *board_id_addr);
}

static int cp_sleep_ctrl(int ctrl)
{
	struct T_A2C_DEVCTL cmd;
	int ret = 0;

	cmd.type = 7;
	cmd.len = 4;
	cmd.data = ctrl;

	if (ipc_write_avail(IPC_CHANNEL_DEVCTL_ID) >= sizeof(cmd)) {
		ipc_write(IPC_CHANNEL_DEVCTL_ID, &cmd, sizeof(cmd));
    } else {
		ret = -1;
        printk(TAG "%s: IPC_CHANNEL_DEVCTL unwriteable.\n", __func__);
    }
	print_log("%s: Set CP %s\n", __func__, ctrl ? "can not sleep" : "can sleep");

	return ret;
}

static int sysc_cp_blk_ctrl(const cp_ctrl cc)
{
	int ret = 0;
	struct cp_platform_data *platdata = cpdata->cp_pdata;

	if(platdata == NULL){
		printk("[%s] platdata is NULL\n", __func__);
		return 0;
	}

	switch(cc)
	{
		case CP_POWER_ON:
			if(platdata->cp_power_on)
				ret = platdata->cp_power_on();
			break;

		case CP_POWER_OFF:
			if(platdata->cp_power_off)
				ret = platdata->cp_power_off();
			break;

		case CP_RESET:
			if(platdata->cp_reset)
				ret = platdata->cp_reset();
			break;

		case CP_RELEASE:
			if(platdata->cp_release)
				ret = platdata->cp_release();
			break;
		default:
			return 0;
	}

	if(ret < 0)
		return 0;
	else
		return 1;
}

static void report_uevent(struct work_struct *work)
{
	unsigned long cp_status = cpdata->cp_status;

	/* notify for user space */
	kobject_uevent(&cpdata->cp_dev->kobj, KOBJ_CHANGE);
	
	/* notify for kernel module */
	cp_notifier_call_chain(cp_status);
}

static void c2a_status_timer_func(unsigned long arg)
{
	printk("%s\n", __func__);

	memcpy(&status_notify.data[0], abnormal_cause, 200);

	printk("Abort reason is: [%s]\n", status_notify.data);

	schedule_work(&cpdata->notify_wq);
}

static ssize_t cp_cause_show(struct device *pdev, struct device_attribute *attr,
			char *buf)
{
	printk("cp_status is %d\n", cpdata->cp_status);

	return sprintf(buf, status_notify.data);
}
static DEVICE_ATTR(cause, S_IRUGO, cp_cause_show, NULL);

static ssize_t cp_command_store(struct device *pdev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    unsigned long command = simple_strtoul(buf, NULL, 10);

	print_log("set command to: %ld\n", command);

	switch (command) {
	case RESET_CP:
		cpdata->cp_status = CP_BOOTING;
		sysc_cp_blk_ctrl(CP_RESET);
		break;

	case LINUX_PANIC:
		printk(KERN_ERR TAG"[++++++++++AP set CP to Abort status++++++++++++++++]\n");
		panic_denote = 1; /* force Linux to be freezed, not to restart */
//		BUG();
		break;

	case AT_TIMEOUT:
		if(cpdata->cp_status != CP_ABORT)
			cpdata->cp_status = CP_AT_TIMEOUT;
		wake_lock_timeout(&cpdata->cp_wakelock, WLOCK_TIME * HZ);
		schedule_work(&cpdata->notify_wq);
		break;

	default:
		break;
	}

    return count;
}
static DEVICE_ATTR(command, S_IWUSR|S_IWGRP|S_IWOTH, NULL, cp_command_store);

static ssize_t cp_status_show(struct device *pdev, struct device_attribute *attr,
			char *buf)
{
	int cnt = 0;

	switch (cpdata->cp_status) {
	case CP_ABORT:
		cnt = sprintf(buf, "abort");
		break;
	case CP_ASSERT:
		cnt = sprintf(buf, "assert");
		break;
	case CP_SWD:
		cnt = sprintf(buf, "sw_watchdog");
		break;
	case CP_BOOTING:
		cnt = sprintf(buf, "rebooting");
		break;
	case CP_AT_TIMEOUT:
		cnt = sprintf(buf, "at_timeout");
		break;
	case CP_HWD:
		cnt = sprintf(buf, "hw_watchdog");
		break;
	case CP_BOOT_COMPLETE:
		cnt = sprintf(buf, "working");
		break;
	case CP_HALT:
		cnt = sprintf(buf, "halt");
		break;
	default:
		cnt = sprintf(buf, "error");
		break;
	}

	print_log("cp_status is status_code=%d: %s\n", cpdata->cp_status, buf);

	return cnt;
}

static ssize_t cp_status_store(struct device *pdev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	print_log("set cp_status to:%s\n", buf);

	if (!strncmp(buf, "abort", count))
		cpdata->cp_status = CP_ABORT;
	else if (!strncmp(buf, "rebooting", count))
		cpdata->cp_status = CP_BOOTING;
	else if (!strncmp(buf, "at timeout", count))
		cpdata->cp_status = CP_AT_TIMEOUT;
	else if (!strncmp(buf, "working", count))
		cpdata->cp_status = CP_BOOT_COMPLETE;

	return count;
}
static DEVICE_ATTR(status, S_IRWXUGO, cp_status_show, cp_status_store);

static ssize_t debug_show(struct device *pdev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d", debug_on);
}

static ssize_t debug_store(struct device *pdev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	debug_on = (int)simple_strtoul(buf, NULL, 10);

	return count;
}
static DEVICE_ATTR(debug, S_IRWXUGO, debug_show, debug_store);

static ssize_t cp_sleep_ctrl_show(struct device *pdev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "CP %s\n", cp_sleepctrl ? "can not sleep" : "can sleep");
}

static ssize_t cp_sleep_ctrl_store(struct device *pdev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	cp_sleepctrl = simple_strtoul(buf, NULL, 10);

	printk(TAG "%s: Set CP %s\n", __func__, cp_sleepctrl ? "can not sleep" : "can sleep");

	cp_sleep_ctrl(cp_sleepctrl);

	return count;
}
static DEVICE_ATTR(cp_sleep_ctrl, S_IRWXUGO, cp_sleep_ctrl_show, cp_sleep_ctrl_store);

static ssize_t dump_mem(unsigned long mem_paddr, unsigned long mem_size,
								char * filename)
{
	struct file *fp = NULL;
	mm_segment_t fs;
	unsigned long len = 0;
	unsigned long saved_len = 0;
	void __iomem *vaddr = NULL;
	unsigned long m_addr = mem_paddr;
	unsigned long unmap_size = mem_size;
	unsigned long map_block_size = mem_size;

	if(mem_paddr == 0 || mem_size == 0)
		return 0;

	/* find enough vmalloc address space */
	do{
		vaddr = ioremap(m_addr, map_block_size);
		if(vaddr == NULL){
			map_block_size = map_block_size >> 1;
		}else{
			break;
		}

	}while(map_block_size > 0);

	if(vaddr == NULL){
		printk(KERN_ERR TAG "Dump mem: %s Fail ,not enough vmalloc address space!\n", filename);
		return 0;
	}else if(map_block_size != mem_size){
		printk(KERN_INFO TAG "Dump mem: %s, map_block_size=%lu, mem_size=%lu\n", filename, map_block_size, mem_size);
	}

    fs = get_fs();
    set_fs(KERNEL_DS);

	/* create file and seek to 0 */
	fp = filp_open(filename, O_RDWR|O_CREAT|O_TRUNC, 777);
	if (IS_ERR(fp)) {
		pr_err(TAG "Dump mem, unable to open file: %s\n", filename);
		goto err1;
	}

	filp_close(fp, NULL);

	/* append file data */
	fp = filp_open(filename, O_RDWR|O_CREAT|O_APPEND, 777);
	if (IS_ERR(fp)) {
		pr_err(TAG "Dump mem, unable to open file: %s\n", filename);
		goto err1;
	}


	do{
		len = vfs_write(fp, (const char __user *)vaddr, map_block_size, &fp->f_pos);
		saved_len += len;
		if(len != map_block_size){
			printk(KERN_ERR TAG "Dump mem, write to file error : %s\n", filename);
			break;
		}

		iounmap(vaddr);
		vaddr = NULL;

		unmap_size = unmap_size - map_block_size;
		if(unmap_size == 0)
			break;

		m_addr += map_block_size;
		map_block_size = (unmap_size > map_block_size)? map_block_size:unmap_size;

		vaddr = ioremap(m_addr, map_block_size);
		if(vaddr == NULL){
			printk(KERN_ERR TAG "Dump mem, ioremap error : %s\n", filename);
			break;
		}

	}while(unmap_size > 0);

	filp_close(fp, NULL);
err1:
	set_fs(fs);

	if(vaddr != NULL)
		iounmap(vaddr);

	return saved_len;
}

static long cp_unlocked_ioctl (struct file *file, unsigned int cmd_in,
								unsigned long arg)
{
    long ret = 0;

    mutex_lock(&cp_mutex);

    switch (cmd_in) {
	case IOC_CP_STOP:
		break;

	case IOC_CP_START:
		if (cpdata->wd_happen == 1) {
			enable_irq(EXYNOS3_IRQ_CP2AP_RESET_REQ);
			cpdata->wd_happen = 0;
		}
		ipc_reset();
		set_id();
		sysc_cp_blk_ctrl(CP_RELEASE);
		print_log("cp start\n");
		break;

    case IOC_CP_POWON:
		cpdata->cp_status = CP_BOOTING;
		sysc_cp_blk_ctrl(CP_POWER_ON);
		sysc_cp_blk_ctrl(CP_RESET); //Evan Tan
		print_log("cp power on\n");
		break;

    case IOC_CP_POWOFF:
		cpdata->cp_status = CP_HALT;
		sysc_cp_blk_ctrl(CP_POWER_OFF);
		print_log("cp power off\n");
        break;

    case IOC_CP_LOOPBACK:
	{
		struct T_A2C_DEVCTL loopback_cmd;
		loopback_cmd.type = 0xFF;
        loopback_cmd.len = 4;
        loopback_cmd.data = arg;

		if (ipc_write_avail(IPC_CHANNEL_DEVCTL_ID) >= sizeof(loopback_cmd)) {
			ipc_write(IPC_CHANNEL_DEVCTL_ID, &loopback_cmd, sizeof(loopback_cmd));
        } else {
			ret = -1;
            printk(KERN_ERR TAG"IPC_CHANNEL_DEVCTL unwriteable.\n");
        }
		print_log("set ipc to loopback mode\n");
    }
		break;

	case IOC_CP_DHI_SM:
		ret = dump_mem(CP_DHI_SM_PADDR, CP_DHI_SM_SIZE, DHI_SM_FILE);
		if (ret > 0)
			printk(KERN_INFO TAG"Dump DHI mem success !\n");
		else
			printk(KERN_ERR TAG"Dump DHI mem fail !\n");
		break;

    case IOC_CP_ZSP_IMEM:
		ret = dump_mem(CP_ZSP_IMEM_PADDR, CP_ZSP_IMEM_SIZE, ZSP_IMEM_FILE);
		if (ret > 0)
			printk(KERN_INFO TAG"Dump ZSP IMEM success !\n");
		else
			printk(KERN_ERR TAG"Dump ZSP IMEM fail !\n");
		break;

    case IOC_CP_ZSP_DMEM:
		ret = dump_mem(CP_ZSP_DMEM_PADDR, CP_ZSP_DMEM_SIZE , ZSP_DMEM_FILE);
		if (ret > 0)
			printk(KERN_INFO TAG"Dump ZSP DMEM success !\n");
		else
			printk(KERN_ERR TAG"Dump ZSP DMEM fail !\n");
		break;

	case IOC_CP_CEVA_SM:
		ret = dump_mem(CP_CEVA_SM_PADDR, CP_CEVA_SM_SIZE, CEVA_SM_FILE);
		if (ret > 0)
			printk(KERN_INFO TAG"Dump CEVA SM success !\n");
		else
			printk(KERN_ERR TAG"Dump CEVA SM fail !\n");
		break;

	case IOC_CP_INITRAM:
		ret = dump_mem(CP_INITRAM_PADDR, CP_INITRAM_SIZE, INITRAM_FILE);
		if (ret > 0)
			printk(KERN_INFO TAG"Dump INITRAM success !\n");
		else
			printk(KERN_ERR TAG"Dump INITRAM fail !\n");
		break;

	case IOC_AP_DDR_MEM:
		/*
		ret = dump_mem(AP_DDR_PADDR, AP_DDR_SIZE, AP_DDR_FILE);
		if(ret > 0)
			printk(KERN_INFO TAG"Dump AP DDR MEM success !\n");
		else
			printk(KERN_ERR TAG"Dump AP DDR MEM fail !\n");
		*/
		break;

	case IOC_CP_DDR_MEM:
		ret = dump_mem(cp_mem_resource.start, resource_size(&cp_mem_resource), CP_DDR_FILE);
		if(ret > 0)
			printk(KERN_INFO TAG"Dump CP DDR MEM success !\n");
		else
			printk(KERN_ERR TAG"Dump CP DDR MEM fail !\n");
		break;

	case IOC_IPC_MEM:
		ret = dump_mem(ipc_mem_resource.start, resource_size(&ipc_mem_resource), IPC_MEM_FILE);
		if(ret > 0)
			printk(KERN_INFO TAG"Dump IPC MEM success !\n");
		else
			printk(KERN_ERR TAG"Dump IPC MEM fail !\n");
		break;

	default:
		ret = -1;
		print_log("unknown IOC command\n");
		break;
    }

	mutex_unlock(&cp_mutex);

	return ret;
}


#ifdef CONFIG_COMPAT
static long cp_compat_ioctl (struct file * file, unsigned int cmd_in, unsigned long arg)
{
	return cp_unlocked_ioctl(file , cmd_in, arg);
}
#endif

static int cp_mmap(struct file *filp, struct vm_area_struct *vma)
{
    unsigned long pfn, psize;
	unsigned long vmsize;
	vmsize = vma->vm_end - vma->vm_start;

	pfn = cp_mem_resource.start >> PAGE_SHIFT;
	psize = ipc_mem_resource.start - cp_mem_resource.start;
    if (vmsize > psize) {
       	pr_err(TAG "%s size err\n", __func__);
        return -ENXIO;
    }

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

   	if (remap_pfn_range(vma, vma->vm_start, pfn, vmsize, vma->vm_page_prot)) {
	 	pr_err(TAG "%s mmap failed\n", __func__);
	 	return -EAGAIN;
    }

    return 0;
}

static const struct file_operations cp_fops = {
	.owner	= THIS_MODULE,
	.mmap	= cp_mmap,
	.unlocked_ioctl = cp_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= cp_compat_ioctl,
#endif
};

static irqreturn_t c2a_abnormal_irq_handle(int irq, void *pdev)
{
    pr_err(TAG "==========Baseband abnormal HW IRQ=========\n");

	set_bits(EXYNOS3_CP_CTRL,6,1,1);

	if (cpdata->cp_status == CP_HALT)
		return IRQ_HANDLED;

	cpdata->cp_status = CP_ABORT;

	wake_lock_timeout(&cpdata->cp_wakelock, WLOCK_TIME * HZ);
	printk("Start c2a status timer...\n");
	mod_timer(&cpdata->c2a_status_timer, jiffies + 3 * HZ);

	return IRQ_HANDLED;
}

static irqreturn_t c2a_watchdog_irq_handle(int irq, void *pdev)
{
    pr_err(TAG "==========Baseband HW WD IRQ=========\n");

	disable_irq_nosync(irq);
	set_bits(EXYNOS3_CP_CTRL, 8,1,1);
	cpdata->wd_happen = 1;

	if (cpdata->cp_status == CP_HALT)
		return IRQ_HANDLED;

	cpdata->cp_status = CP_HWD;
	wake_lock_timeout(&cpdata->cp_wakelock, WLOCK_TIME * HZ);
	memset(&status_notify.data[0], 0, sizeof(status_notify.data));
	sprintf(&status_notify.data[0], "===Baseband HW WD IRQ===");
	schedule_work(&cpdata->notify_wq);

	return IRQ_HANDLED;
}

static long devctl_ipc_rx_irq_handle(int id, void *priv, int event)
{
	struct T_C2A_STATUS_NOTIFY status_notify;

	if (event & R_AVAIL) {
		if (ipc_read_avail(IPC_CHANNEL_DEVCTL_ID)) {
			if (ipc_read(IPC_CHANNEL_DEVCTL_ID, &status_notify, sizeof(struct T_C2A_STATUS_NOTIFY)) < 0)
				return IRQ_HANDLED;
		} else {
			pr_err(TAG "devctl channel unreadable!\n");
			BUG();
		}
	}
	else
		return IRQ_HANDLED;

	switch (status_notify.type) {
	case CP_BOOT_COMPLETE:
		print_log("==========Baseband Boot completed=========\n");

		cpdata->cp_status =  CP_BOOT_COMPLETE;//working
		wake_lock_timeout(&cpdata->cp_wakelock, WLOCK_TIME * HZ);
		schedule_work(&cpdata->notify_wq);
		break;

	default:
		pr_info(TAG "Devctl unknow notify type!\n");
		break;
	}

	return IRQ_HANDLED;
}


static long devctl_event_rx_irq_handler(int id, void *priv, int event)
{
	unsigned int type = 0;
	int ret;

	if (event & R_AVAIL) {
		if ((ret = ipc_read_avail(IPC_CHANNEL_DEVCTL_EVENT_ID)) >= 4) {
			ret = ipc_read(IPC_CHANNEL_DEVCTL_EVENT_ID, &type, sizeof(unsigned int));
			if (ret <= 0) {
				printk(TAG "Read channel devctl event failed!\n");
				return IRQ_HANDLED;
			}
		}
	}else
		return IRQ_HANDLED;

	switch (type) {
	case CP_ASSERT:
		pr_err(TAG "==========Baseband assert=========\n");
		cpdata->cp_status = CP_ASSERT;
		break;

	case CP_ABORT:
		pr_err(TAG "==========Baseband abort=========\n");
		cpdata->cp_status = CP_ABORT;
		break;

	case CP_SWD:
		pr_err(TAG "==========Baseband SWD=========\n");
		cpdata->cp_status = CP_SWD;
		break;

	default:
		pr_err(TAG "Devctl event unknow notify type!\n");
		break;
	}

	return IRQ_HANDLED;
}

static void pmic_ipc_work(struct work_struct *work)
{
	unsigned int pmic_req = 0, pmic_res = 0;
	int ret = 0, vol_mV = 0;
	struct cp_platform_data *platdata = cpdata->cp_pdata;

	if (ipc_read_avail(IPC_CHANNEL_PMIC_REQ_ID) >= 4) {
		ret = ipc_read(IPC_CHANNEL_PMIC_REQ_ID, &pmic_req, sizeof(unsigned int));
		if (ret < 0) {
			pr_err(TAG "ipc_read error, ret = %d!\n", ret);
		}
	} else {
		pr_err(TAG "PMIC req channel unreadable!\n");
		BUG();
	}

	vol_mV = pmic_req & 0x0000ffff;

	/* config sim card voltage */
	switch (pmic_req >> 16) {
	case pmic_sim1_vol_ctl_req:
		printk("==============================================\n");
		pr_info(TAG "SIM1_VOL_CTL_REQ, vol = %d.\n", vol_mV);
		if(platdata && platdata->sim_power_config){
			ret = platdata->sim_power_config(SIM0, vol_mV);
		}
		//ret = ctrl_sim_power("vdd_sim0_cp", vol_mV);
		break;

	case pmic_sim2_vol_ctl_req:
		pr_info(TAG "SIM2_VOL_CTL_REQ, vol = %d.\n", vol_mV);
		if(platdata && platdata->sim_power_config){
			ret = platdata->sim_power_config(SIM1, vol_mV);
		}
		//ret = ctrl_sim_power("vdd_sim1_cp", vol_mV);
		break;

	case pmic_battery_vol_get_req:
		pr_info(TAG "BATTERY_VOL_GET_REQ\n");
		break;

	default:
		pr_info(TAG "unknow pmic req\n");
		break;
	}

	/* write config result */
	if (ret)
		pmic_res = 0x0000ffff;
	else
		pmic_res = 0;
	pmic_res |= pmic_req & 0xffff0000;
	pr_info(TAG "%s: Write set result to CP [0x%x]\n", __func__, pmic_res);

	if (ipc_write_avail(IPC_CHANNEL_PMIC_CNF_ID) >= 4) {
		ipc_write(IPC_CHANNEL_PMIC_CNF_ID, &pmic_res, sizeof(unsigned int));
	} else {
		pr_err(TAG "pmic cnf channel unwriteable!\n");
		BUG();
	}
}

static long pmic_ipc_rx_irq_handle(int id, void *priv, int event)
{
	if (event & R_AVAIL) {
		printk("--------------------------------------------\n");
		schedule_work(&pmic_work);
	}
	return IRQ_HANDLED;
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t debug_reg_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	char tmp_buf[10] = {0};
	char *append = "\n";

	sprintf(tmp_buf, "%x", reg);
	strcat(tmp_buf, append);

	return simple_read_from_buffer(buf, count, ppos, tmp_buf, strlen(tmp_buf) + 1);
}

static ssize_t debug_reg_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	reg = simple_strtoul(buf, NULL, 0);
	printk("%s: phy addr 0x%x\n", __func__, reg);
	return count;
}

static const struct file_operations reg_fops = {
	.read = debug_reg_read,
	.write = debug_reg_write,
	.open = debug_open,
};

static ssize_t debug_val_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	char tmp_buf[10] = {0};
	char *append = "\n";
	void __iomem *vaddr;
	int ret = 0;

	vaddr = ioremap(reg, sizeof(reg));
	val = readl(vaddr);
	iounmap(vaddr);

	ret = sprintf(tmp_buf, "%x", val);
	strcat(tmp_buf, append);

	//printk("%s: val = 0x%x, buf = %s, ret = %d, len = %d\n",
	//	    __func__, val, buf, ret, strlen(tmp_buf) + 1);

	return simple_read_from_buffer(buf, count, ppos, tmp_buf, strlen(tmp_buf) + 1);
}

static ssize_t debug_val_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	void __iomem *vaddr;

	val = simple_strtoul(buf, NULL, 0);
	printk("%s: 0x%x\n", __func__, val);

	vaddr = ioremap(reg, sizeof(reg));
	writel(val, vaddr);
	iounmap(vaddr);

	return count;
}

loff_t debug_val_lseek(struct file *file, loff_t offset, int origin)
{
	file->f_pos = offset;
	return file->f_pos;
}

static const struct file_operations val_fops = {
	.read = debug_val_read,
	.write = debug_val_write,
	.llseek = debug_val_lseek,
	.open = debug_open,
};

static ssize_t debug_set_vol(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	int val, vol_mV, ret;
	struct cp_platform_data *platdata;

	platdata = cpdata->cp_pdata;
	if(platdata == NULL){
		printk("[%s] platdata is NULL\n", __func__);
		return 0;
	}

	val = simple_strtoul(buf, NULL, 0);
	printk("debug sim voltage, val = 0x%x.\n", val);

	vol_mV = val & 0x0000ffff;

	switch (val >> 16) {
	case pmic_sim1_vol_ctl_req:
		pr_info(TAG "set sim1 voltage, vol = %d.\n", vol_mV);

		if(platdata->sim_power_config){
			ret = platdata->sim_power_config(SIM0, vol_mV);
			if (ret < 0) {
				pr_err(TAG "set sim1 voltage failed.\n");
			}
		}
		break;

	case pmic_sim2_vol_ctl_req:
		pr_info(TAG "set sim2 voltage, vol = %d.\n", vol_mV);

		if(platdata->sim_power_config){
			ret = platdata->sim_power_config(SIM1, vol_mV);
			if (ret < 0) {
				pr_err(TAG "set sim2 voltage failed.\n");
			}
		}
		break;
	}

	return count;
}

static const struct file_operations vol_fops = {
	.write = debug_set_vol,
	.open = debug_open,
};

static long debugfs_init(void)
{
	struct dentry *dent;

	printk("%s: start\n", __func__);

	dent = debugfs_create_dir("cpctrl", NULL);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("reg", 0666, dent, NULL, &reg_fops);
	debugfs_create_file("val", 0666, dent, NULL, &val_fops);
	debugfs_create_file("vol", 0666, dent, NULL, &vol_fops);

	printk("%s: end\n", __func__);

	return 0;
}

static int __init cp_probe(struct platform_device *pdev)
{
	int err = -1;
	struct cp_platform_data *platdata;

	cpdata = kzalloc(sizeof(struct cp_data), GFP_KERNEL);
	if (IS_ERR(cpdata)) {
		pr_err("kzalloc cp_data failed.\n");
		goto err;
	}

	err = alloc_chrdev_region(&cpdata->devno, 0, 1, "c6320_cp");
	if (err < 0) {
		pr_err("alloc c6320 devno failed!\n");
		goto err;
	}

	cdev_init(&cpdata->cdev, &cp_fops);
	err = cdev_add(&cpdata->cdev, cpdata->devno, 1);
	if (err < 0) {
		pr_err("add cdev failed!\n");
		goto err;
	}

	cpdata->cp_class = class_create(THIS_MODULE, "c6320_cp");
	if (IS_ERR(cpdata->cp_class)) {
		err = PTR_ERR(cpdata->cp_class);
		BUG();
	}

	cpdata->cp_dev = device_create(cpdata->cp_class, NULL, cpdata->devno, NULL, "c6320_cp");
	if (IS_ERR(cpdata->cp_dev)) {
		err = PTR_ERR(cpdata->cp_dev);
		BUG();
	}

	platdata = (struct cp_platform_data *)(pdev->dev.platform_data);
	cpdata->cp_pdata = platdata;

	err = device_create_file(&pdev->dev, &dev_attr_command);
	err = device_create_file(&pdev->dev, &dev_attr_status);
	err = device_create_file(&pdev->dev, &dev_attr_debug);
	err = device_create_file(&pdev->dev, &dev_attr_cause);
	err = device_create_file(&pdev->dev, &dev_attr_cp_sleep_ctrl);

    err = ipc_create_ch(&ipc_channel_devctl);
    if (err < 0) {
		printk(KERN_ERR TAG"Create devctl channel failed.\n");
	   	goto err;
	}

	err = ipc_create_ch(&ipc_channel_devctl_event);
	if (err < 0) {
		printk(KERN_ERR TAG"Create devctl event channel failed.\n");
		goto err;
	}

	err = ipc_create_ch(&ipc_channel_pmic_req);
   	if (err < 0) {
		printk(KERN_ERR TAG"Create pmic req channel failed.\n");
	   	goto err;
	}

	err = ipc_create_ch(&ipc_channel_pmic_cnf);
    if (err < 0) {
		printk(KERN_ERR TAG"Create pmic cnf channel failed.\n");
	   	goto err;
	 }

	ipc_register_inthandle(IPC_CHANNEL_DEVCTL_ID, NULL, devctl_ipc_rx_irq_handle);
    ipc_enable_interrupt(IPC_CHANNEL_DEVCTL_ID, R_AVAIL_INT);

	ipc_register_inthandle(IPC_CHANNEL_DEVCTL_EVENT_ID, NULL, devctl_event_rx_irq_handler);
	ipc_enable_interrupt(IPC_CHANNEL_DEVCTL_EVENT_ID, R_AVAIL_INT);

	ipc_register_inthandle(IPC_CHANNEL_PMIC_REQ_ID, NULL, pmic_ipc_rx_irq_handle);
	ipc_enable_interrupt(IPC_CHANNEL_PMIC_REQ_ID, R_AVAIL_INT);

	ipc_set_localcpu_status(CPU_WORK_STAT);
	cpdata->cp_status = CP_HALT;
	wake_lock_init(&cpdata->cp_wakelock, WAKE_LOCK_SUSPEND, "cp_wakelock");
	//spin_lock_init(&cpdata->lock);
	INIT_WORK(&cpdata->notify_wq, report_uevent);
	INIT_WORK(&pmic_work, pmic_ipc_work);
	setup_timer(&cpdata->c2a_status_timer, c2a_status_timer_func, (unsigned long)pdev);

	/* Baseband Abort IRQ */
	err = request_irq(EXYNOS3_IRQ_CP_ACTIVE, c2a_abnormal_irq_handle, 0, "CP_ACTIVE_IRQ", &pdev);
	if (err < 0) {
		pr_err(TAG"failed to request cp to ap abort irq: %d\n", err);
		goto err;
	}
	enable_irq_wake(EXYNOS3_IRQ_CP_ACTIVE);

	/* Baseband WD IRQ*/
	err = request_irq(EXYNOS3_IRQ_CP2AP_RESET_REQ, c2a_watchdog_irq_handle, 0,"CP2AP_RESET_IRQ", &pdev);
	if (err < 0) {
		pr_err(TAG"failed to request cp to ap watchdog irq: %d\n", err);
		goto err;
	}
	enable_irq_wake(EXYNOS3_IRQ_CP2AP_RESET_REQ);

	if(platdata && platdata->cp_power_config){
		platdata->cp_power_config(NULL, force_disable);
	}

	debugfs_init();

	if(platdata && platdata->setup_access_area){
		platdata->setup_access_area();
	}

	pr_info(TAG "C6320 CP Driver initial success !\n");
	return 0;

err:
	pr_err(TAG "C6320 CP Driver initial failed !\n");
	return -1;
}

static int __exit cp_remove(struct platform_device *pdev)
{
	if (cpdata == NULL)
		return 0;

	free_irq(EXYNOS3_IRQ_CP2AP_RESET_REQ, pdev);
	free_irq(EXYNOS3_IRQ_CP_ACTIVE, pdev);

	device_remove_file(&pdev->dev, &dev_attr_command);
	device_remove_file(&pdev->dev, &dev_attr_status);
	device_remove_file(&pdev->dev, &dev_attr_debug);

	device_destroy(cpdata->cp_class, cpdata->devno);
	class_destroy(cpdata->cp_class);

	cdev_del(&cpdata->cdev);
	unregister_chrdev_region(cpdata->devno,1);

	kfree(cpdata);
	cpdata = NULL;

	return 0;
}

static int cp_suspend(struct platform_device *pdev, pm_message_t state)
{
	ipc_set_localcpu_status(CPU_SLEEP_STAT);
	return 0;
}

static int cp_resume(struct platform_device * pdev)
{
    ipc_set_localcpu_status(CPU_WORK_STAT);
	return 0;
}

static struct platform_driver c63xx_driver = {
	.driver = {
		.name = "c6320_cp",
		.owner = THIS_MODULE,
	},
	.suspend = cp_suspend,
	.resume = cp_resume,
	.probe = cp_probe,
	.remove = __exit_p(cp_remove),
};

static int __init cp_init(void)
{
	return platform_driver_register(&c63xx_driver);
}

static void __exit cp_exit(void)
{
	platform_driver_unregister(&c63xx_driver);
}

late_initcall_sync(cp_init);
module_exit(cp_exit);

EXPORT_SYMBOL_GPL(register_cp_notifier);
EXPORT_SYMBOL_GPL(unregister_cp_notifier);

MODULE_DESCRIPTION("Driver for CYIT CP");
MODULE_LICENSE("GPL");
