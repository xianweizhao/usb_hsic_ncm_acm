/*
 * Copyright (C), 2010-2020,  CYIT. Co., LTD.
 *
 * File: drivers/ipc/c83xx_cp.c
 *
 * Description:
 *
 *
 * C6310plus Platform + CYIT C8320 modern -- 4 lines scheme
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/tty.h>
#include <linux/c8320_modem.h>
#include <linux/err.h>

#include <plat/gpio-cfg.h>

#include <asm/uaccess.h>
#ifdef CONFIG_C8320_HSIC_NEW
#include "../modem_v2/c8320_modem_ctl.h"
#endif

#define A2C_STATUS_DELAY ((1/2)*HZ)
#define N_C83XX 26

#define debug_mask 1
#define CP_DEBUG(fmt, args...) do{ \
	if(debug_mask) \
		printk(KERN_INFO "[cp_c83xx]: " fmt, ##args); \
	}while(0)

static BLOCKING_NOTIFIER_HEAD(cp_c83xx_notifiers);
static int g_modem_abort_times = 0;
static int g_modem_reset_times = 0;
struct cp_c83xx_info
{
	struct class *cp_class;
	struct device *dev;
	struct device *chdev;
	struct cdev cdev;
	dev_t devno;
	atomic_t cdev_count;
	struct spinlock cdev_lock;

	atomic_t ldisc_count;
	unsigned long pmic_sim_res;

	struct wake_lock abort_wakelock;
	struct wake_lock reset_wakelock;
	struct wake_lock wakeup_uart_wakelock;
	struct wake_lock wakeup_hsic_wakelock;

	struct work_struct  notify_wq;
	struct work_struct rest_wq;
	struct delayed_work abort_wq;

	struct timer_list a2c_status_timer;
	struct timer_list a2c_reset_timer;
	int cp_status;
	struct spinlock slock;

	unsigned int c2a_abort_irq;
	unsigned int c2a_wake_uart_irq;
	unsigned int c2a_wake_hsic_irq;

	struct c83xx_platform_data *cpdata;
	struct c83xx_platform_pm *cp_pm_uart;
	struct c83xx_platform_pm *cp_pm_hsic;

	struct blocking_notifier_head *notifiers;
};

static struct cp_c83xx_info *c83xx_modem_info;

/*panic_denote, use for Linux restart control */
int c8320_panic_denote = 0;

int cp_c83xx_add_notifier(struct notifier_block *notifier)
{
	int ret;
	struct blocking_notifier_head * nh;

	if(c83xx_modem_info)
		nh = c83xx_modem_info->notifiers;
	else
		nh = &cp_c83xx_notifiers;

	ret = blocking_notifier_chain_register(nh, notifier);

	return ret;
}
EXPORT_SYMBOL(cp_c83xx_add_notifier);

int cp_c83xx_remove_notifier(struct notifier_block *notifier)
{
	int ret;
	struct blocking_notifier_head * nh;

	if(c83xx_modem_info)
		nh = c83xx_modem_info->notifiers;
	else
		nh = &cp_c83xx_notifiers;

	ret = blocking_notifier_chain_unregister(nh, notifier);

	return ret;
}
EXPORT_SYMBOL(cp_c83xx_remove_notifier);

static void set_cp_status(struct cp_c83xx_info *cpinfo, int status)
{
	spin_lock(&cpinfo->slock);
	if(!cpinfo){
		pr_err("[%s] Cannot find the device of c83xx\n", __func__);
		spin_unlock(&cpinfo->slock);
	}
	else{
		cpinfo->cp_status = status;
		spin_unlock(&cpinfo->slock);
	}
}

int get_cp_status(void)
{
	int cp_status;

	spin_lock(&c83xx_modem_info->slock);
	if(!c83xx_modem_info){
		pr_err("[%s] c83xx_modem_info has not been inited\n", __func__);
		spin_unlock(&c83xx_modem_info->slock);
		return POWEROFF;
	}

	cp_status = c83xx_modem_info->cp_status;
	spin_unlock(&c83xx_modem_info->slock);

	return cp_status;
}
EXPORT_SYMBOL(get_cp_status);

static void cp_rest_wq_handler(struct work_struct *work)
{
	struct cp_c83xx_info *cpinfo;

	cpinfo = container_of(work, struct cp_c83xx_info, rest_wq);

	printk("==========c83xx boot complete==========\n");
	set_cp_status(cpinfo, WORKING);
	kobject_uevent(&cpinfo->chdev->kobj, KOBJ_CHANGE);
}

static void cp_notify_wq_handler(struct work_struct *work)
{
	struct cp_c83xx_info *cpinfo;

	CP_DEBUG("[%s] uevent changed\n", __func__);
	cpinfo = container_of(work, struct cp_c83xx_info, notify_wq);
	kobject_uevent(&cpinfo->chdev->kobj, KOBJ_CHANGE);
}

static void cp_abort_wq_handler(struct work_struct *work)
{
	int ret;
	struct delayed_work *dwork;
	struct cp_c83xx_info *cpinfo;
	struct c83xx_platform_data *cpdata;

	CP_DEBUG("%s\n", __func__);
	dwork = to_delayed_work(work);
	cpinfo = container_of(dwork, struct cp_c83xx_info, abort_wq);
	cpdata = cpinfo->cpdata;
	ret = cpdata->cp_check_abort();
	if(!ret){  // 1: cp normal, 0: cp abort
		if (cpinfo->cp_status != REBOOTING) {
			cpdata->cp_usb_power_config(1);
			set_cp_status(cpinfo, ABORT);
			wake_lock_timeout(&cpinfo->abort_wakelock, 120 * HZ);
			blocking_notifier_call_chain(cpinfo->notifiers, cpinfo->cp_status, NULL);
			kobject_uevent(&cpinfo->chdev->kobj, KOBJ_CHANGE);
			g_modem_reset_times++;
		}
	}
}

static void a2c_status_timer_func(unsigned long data)
{
	struct cp_c83xx_info *cpinfo = (struct cp_c83xx_info *)data;
	struct c83xx_platform_pm *cp_pm_uart = cpinfo->cp_pm_uart;
	struct c83xx_platform_pm *cp_pm_hsic = cpinfo->cp_pm_uart;

	if(cp_pm_uart && cp_pm_uart->a2c_status_update)
		cp_pm_uart->a2c_status_update(0);
	if(cp_pm_hsic && cp_pm_hsic->a2c_status_update)
		cp_pm_hsic->a2c_status_update(0);
}

static void a2c_reset_timer_func(unsigned long data)
{
	struct cp_c83xx_info *cpinfo = (struct cp_c83xx_info *)data;
	struct c83xx_platform_data *cpdata = cpinfo->cpdata;
	int cpstatus = 1;

	if(cpdata && cpdata->cp_check_boot)
		cpstatus = cpdata->cp_check_boot();

	if(!cpstatus){
		schedule_work(&cpinfo->rest_wq);
		if(wake_lock_active(&cpinfo->reset_wakelock))
			wake_unlock(&cpinfo->reset_wakelock);
	} else {
		mod_timer(&cpinfo->a2c_reset_timer, jiffies + (HZ/20));
	}
}

static ssize_t cp_c83xx_cmd_show(struct device *devp, struct device_attribute *attr,
			char *buf)
{
	struct platform_device *pdev = to_platform_device(devp);
	struct cp_c83xx_info *cpinfo = platform_get_drvdata(pdev);

	return sprintf(buf, "pdata->c2a_boot_gpio is: %d\n", cpinfo->cp_status);
}

static ssize_t cp_c83xx_cmd_store(struct device *devp, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	unsigned long command;
	struct platform_device *pdev = to_platform_device(devp);
	struct cp_c83xx_info *cpinfo;
	struct c83xx_platform_data * cpdata;
	struct c83xx_platform_pm *cp_pm_uart;
	struct c83xx_platform_pm *cp_pm_hsic;
	int ret;

	cpinfo = platform_get_drvdata(pdev);
	cpdata = cpinfo->cpdata;
	cp_pm_uart = cpinfo->cp_pm_uart;
	cp_pm_hsic = cpinfo->cp_pm_hsic;

	command = simple_strtoul(buf, NULL, 10);
	switch (command) {
		case CP_RESET:  // reset cp
			CP_DEBUG("Reset c83xx modem\n");
			set_cp_status(cpinfo, REBOOTING);
			if(cpdata && cpdata->cp_reset)
				cpdata->cp_reset();
			//mod_timer(&cpinfo->a2c_reset_timer, jiffies + 3 * HZ);
			//wake_lock(&cpinfo->reset_wakelock);
			break;
		case CP_POWER_ON:
			pr_info("C83xx power on\n");
			set_cp_status(cpinfo, BOOTING);
			if(cpdata && cpdata->cp_power_on)
				ret = cpdata->cp_power_on();
			break;
		case CP_POWER_OFF:
			pr_info("C83xx power off\n");
			set_cp_status(cpinfo, POWEROFF);
			if(cpdata && cpdata->cp_power_off)
				ret = cpdata->cp_power_off();
			break;
		case CP_ABORT:  //if cp abort then force linux to be panic, only in debug version
			c8320_panic_denote = 1;  // force Linux to be freezed, not to restart
			printk("+++++++++C83xx Abort+++++++++\n");
			blocking_notifier_call_chain(cpinfo->notifiers, cpinfo->cp_status, NULL);
			BUG();
			break;
		case CP_AT_TIMEOUT:  // triger cp abort UEvent notify
			printk("+++++++++C83xx AT timeout+++++++++\n");
			if(cpinfo->cp_status != ABORT)
				set_cp_status(cpinfo, AT_TIMEOUT);
			schedule_work(&cpinfo->notify_wq);
			break;

		/* for debug by using uart*/
		case CP_HW_RESET:
			set_cp_status(cpinfo, REBOOTING);
			if(cpdata && cpdata->cp_hw_reset)
				cpdata->cp_hw_reset();
			break;
		case CP_A2C_WAKEUP_SET_UART:
			if(cp_pm_uart && cp_pm_uart->cp_activate_nosync)
				cp_pm_uart->cp_activate_nosync(1);
			break;
		case CP_A2C_WAKEUP_CLR_UART:
			if(cp_pm_uart && cp_pm_uart->cp_activate_nosync)
				cp_pm_uart->cp_activate_nosync(0);
			break;
		case CP_A2C_WAKEUP_FAL_UART:
			if(cp_pm_uart && cp_pm_uart->cp_activate_nosync){
				cp_pm_uart->cp_activate_nosync(1);
				mdelay(100);
				cp_pm_uart->cp_activate_nosync(0);
			}
			break;
		case CP_ACTIVATE_SYNC_UART:
			if(cp_pm_uart && cp_pm_uart->cp_activate_sync)
				cp_pm_uart->cp_activate_sync(1);
			break;

		/* for debug by using hsic*/
		case CP_A2C_WAKEUP_SET_HSIC:
			if(cp_pm_hsic && cp_pm_hsic->cp_activate_sync)
				cp_pm_hsic->cp_activate_nosync(1);
			break;
		case CP_A2C_WAKEUP_CLR_HSIC:
			if(cp_pm_hsic && cp_pm_hsic->cp_activate_sync)
				cp_pm_hsic->cp_activate_nosync(0);
			break;
		case CP_A2C_WAKEUP_FAL_HSIC:
			if(cp_pm_hsic && cp_pm_hsic->cp_activate_sync){
				cp_pm_hsic->cp_activate_nosync(1);
				mdelay(100);
				cp_pm_hsic->cp_activate_nosync(0);
			}
			break;
		case CP_ACTIVATE_SYNC_HSIC:
			if(cp_pm_hsic && cp_pm_hsic->cp_activate_sync)
				cp_pm_hsic->cp_activate_sync(1);
			break;
		default:
			break;
	}

	return count;
}

static DEVICE_ATTR(command, 0660, cp_c83xx_cmd_show, cp_c83xx_cmd_store);

static ssize_t cp_c83xx_status_show(struct device *devp, struct device_attribute *attr,
			char *buf)
{
	struct platform_device *pdev = to_platform_device(devp);
	struct cp_c83xx_info *cpinfo;
	int cp_status;

	cpinfo = platform_get_drvdata(pdev);
	spin_lock(&cpinfo->slock);
	cp_status = cpinfo->cp_status;
	spin_unlock(&cpinfo->slock);

	CP_DEBUG("cp_status is %d\n", cp_status);

	switch (cp_status) {
	case ABORT:
		return sprintf(buf, "abort");
	case REBOOTING:
		return sprintf(buf, "rebooting");
	case AT_TIMEOUT:
		return sprintf(buf, "at timeout");
	case WORKING:
		return sprintf(buf, "working");
	case POWEROFF:
		return sprintf(buf, "poweroff");
	case BOOTING:
		return sprintf(buf, "booting");
	default:
		return sprintf(buf, "Error!");
	}
}

static ssize_t cp_c83xx_status_store(struct device *devp, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(devp);
	struct cp_c83xx_info *cpinfo;
	int cp_status;

	cpinfo = platform_get_drvdata(pdev);
	spin_lock(&cpinfo->slock);
	cp_status = cpinfo->cp_status;
	printk(" count : %d --%s\n",count, buf);
	if (!strncmp(buf, "abort", count-1)){
		cp_status = ABORT;
		printk("set abort\n");
		cpinfo->cp_status = cp_status;
		c8320_modem_ctl_ex.phone_state = STATE_CRASH;
		wake_lock_timeout(&cpinfo->abort_wakelock, 120 * HZ);
		blocking_notifier_call_chain(cpinfo->notifiers, cpinfo->cp_status, NULL);
		kobject_uevent(&cpinfo->chdev->kobj, KOBJ_CHANGE);
	}
	else if (!strncmp(buf, "rebooting", count-1)){
		cp_status = REBOOTING;
	}
	else if (!strncmp(buf, "at timeout", count-1)){
		cp_status = AT_TIMEOUT;
	}
	else if (!strncmp(buf, "working", count-1)){
		cp_status = WORKING;
	}

	printk("change c83xx status %d -> %d\n", cpinfo->cp_status, cp_status);
	cpinfo->cp_status = cp_status;
	spin_unlock(&cpinfo->slock);

	return count;
}
static DEVICE_ATTR(status, 0660, cp_c83xx_status_show, cp_c83xx_status_store);

static irqreturn_t cp2ap_abort_handler(int irq, void *devid)
{
	struct cp_c83xx_info *cpinfo;
	if(c8320_modem_ctl_ex.phone_state < STATE_ONLINE){
		
		CP_DEBUG("cp2ap_abort_irq, do nothing before boot CP\n");
		return IRQ_HANDLED;
	} 

	CP_DEBUG("cp2ap_abort_irq\n");
#ifdef CONFIG_C8320_HSIC_NEW
	c8320_modem_ctl_ex.phone_state = STATE_CRASH;
#endif
	cpinfo = (struct cp_c83xx_info *)devid;
	schedule_delayed_work(&cpinfo->abort_wq, msecs_to_jiffies(3));
	g_modem_abort_times++;
	return IRQ_HANDLED;
}
	//if((c8320_modem_ctl_ex.phone_state == STATE_RESET_CP)||(c8320_modem_ctl_ex.phone_state == STATE_OFFLINE)){
static ssize_t cp_c83xx_manual_store(struct device *devp, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(devp);
	struct cp_c83xx_info *cpinfo;
	struct c83xx_platform_data * cpdata;
	int ret;

	cpinfo = platform_get_drvdata(pdev);
	cpdata = cpinfo->cpdata;

	c8320_modem_ctl_ex.phone_state = STATE_RESET_CP;
	printk("Karl:manual reset modem\n");
	g_modem_reset_times++;
	cpdata->cp_usb_power_config(1);
	set_cp_status(cpinfo, ABORT);
	wake_lock_timeout(&cpinfo->abort_wakelock, 100 * HZ);
	blocking_notifier_call_chain(cpinfo->notifiers, cpinfo->cp_status, NULL);
	kobject_uevent(&cpinfo->chdev->kobj, KOBJ_CHANGE);

	return count;
}

static ssize_t cp_c83xx_manual_show(struct device *devp, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "abort_times=%d, reset_times=%d\n", 
			g_modem_abort_times, g_modem_reset_times);
}
static DEVICE_ATTR(manual_reset, 0777, cp_c83xx_manual_show, cp_c83xx_manual_store);

static irqreturn_t cp2ap_wake_uart_handler(int irq, void *devid)
{
	struct cp_c83xx_info *cpinfo;
	struct c83xx_platform_pm *cp_pm_uart;
	int value = 0;

	cpinfo = (struct cp_c83xx_info *)devid;
/*
	if(wake_lock_active(&cpinfo->wakeup_uart_wakelock))
		return IRQ_HANDLED;
*/

	cp_pm_uart = cpinfo->cp_pm_uart;
	if(cp_pm_uart && cp_pm_uart->cp_check_wake)
		value = cp_pm_uart->cp_check_wake();

	if (value)
		wake_lock(&cpinfo->wakeup_uart_wakelock);
	else
		wake_lock_timeout(&cpinfo->wakeup_uart_wakelock, 2 * HZ);

	return IRQ_HANDLED;
}

#if 0
static irqreturn_t cp2ap_wake_hsic_handler(int irq, void *devid)
{
	struct cp_c83xx_info *cpinfo;
	struct c83xx_platform_pm *cp_pm_hsic;
	int value = 0;

	cpinfo = (struct cp_c83xx_info *)devid;

	if(wake_lock_active(&cpinfo->wakeup_hsic_wakelock))
		return IRQ_HANDLED;

	cp_pm_hsic = cpinfo->cp_pm_uart;
	if(cp_pm_hsic && cp_pm_hsic->cp_check_wake)
		value = cp_pm_hsic->cp_check_wake();
	if (value)
		wake_lock(&cpinfo->wakeup_hsic_wakelock);
	else
		wake_lock_timeout(&cpinfo->wakeup_hsic_wakelock, 2 * HZ);

	return IRQ_HANDLED;
}
#endif

/*type : 1--active
*           0--inactive
* If activate cp, wait untile cp works up.
*/
int ap2cp_wake_sync(int ch, int type)
{
	struct c83xx_platform_pm *cp_pm;
	int ret = 0;

	if(!c83xx_modem_info){
		pr_err("[%s] c83xx_modem_info has not been inited\n", __func__);
		return -1;
	}

	if(ch == CH_UART)
		cp_pm = c83xx_modem_info->cp_pm_uart;
	else
		cp_pm = c83xx_modem_info->cp_pm_hsic;

	if(cp_pm && cp_pm->cp_activate_sync)
		ret = cp_pm->cp_activate_sync(type);

	return ret;
}
EXPORT_SYMBOL(ap2cp_wake_sync);

/*type : 1--active
*        0--inactive
* If activate cp, pull up io and return right now.
*/
void ap2cp_wake_nosync(int ch, int type)
{
	struct c83xx_platform_pm *cp_pm;

	if(!c83xx_modem_info){
		pr_err("[%s] c83xx_modem_info has not been inited\n", __func__);
		return;
	}

	if(ch == CH_UART)
		cp_pm = c83xx_modem_info->cp_pm_uart;
	else
		cp_pm = c83xx_modem_info->cp_pm_hsic;

	if(cp_pm && cp_pm->cp_activate_nosync)
		cp_pm->cp_activate_nosync(type);
}
EXPORT_SYMBOL(ap2cp_wake_nosync);

/*
*  return cp pm requirement for ap
*  1---require AP working
*  0---no care about AP status
*/
int cp2ap_wake_check(int ch)
{
	int cp_pm_status = 0;
	struct c83xx_platform_pm *cp_pm;

	if(!c83xx_modem_info){
		pr_err("[%s] c83xx_modem_info has not been inited\n", __func__);
		return -1;
	}

	if(ch == CH_UART)
		cp_pm = c83xx_modem_info->cp_pm_uart;
	else
		cp_pm = c83xx_modem_info->cp_pm_hsic;

	if(cp_pm && cp_pm->cp_check_wake)
		cp_pm_status = cp_pm->cp_check_wake();

	return cp_pm_status;
}
EXPORT_SYMBOL(cp2ap_wake_check);

void ap2cp_status_update(int ch, int status)
{
	struct c83xx_platform_pm *cp_pm;

	if(!c83xx_modem_info){
		pr_err("[%s] c83xx_modem_info has not been inited\n", __func__);
		return;
	}

	printk("[%s] ch: %d, status: %d\n", __func__, ch, status);
	if(ch == CH_UART)
		cp_pm = c83xx_modem_info->cp_pm_uart;
	else
		cp_pm = c83xx_modem_info->cp_pm_hsic;

	if(cp_pm && cp_pm->a2c_status_update)
		cp_pm->a2c_status_update(status);
}
EXPORT_SYMBOL(ap2cp_status_update);

#if 0
static int cp_c83xx_ldisc_open(struct tty_struct *tty)
{
	tty->disc_data = c83xx_modem_info;
	return 0;
}

static void cp_c83xx_ldisc_close(struct tty_struct *tty)
{

}

static ssize_t cp_c83xx_ldisc_write(struct tty_struct *tty, struct file *file,
			const unsigned char *buf, size_t nr)
{
	unsigned long pmic_req, pmic_res;
	int vol_mV, ret = 0;
	struct cp_c83xx_info *cpinfo = tty->disc_data;
	struct c83xx_platform_data *cpdata = cpinfo->cpdata;

	pmic_req = simple_strtoul(buf, NULL, 0);
	vol_mV = pmic_req & 0x0000ffff;

	/* config sim card voltage */
	switch (pmic_req >> 16) {
	case PMIC_SIM0_VOL_CTL_REQ:
		pr_info("SIM1_VOL_CTL_REQ, vol = %d.\n", vol_mV);
		if(cpdata && cpdata->sim_power_config){
			ret = cpdata->sim_power_config(SIM0, vol_mV);
		}
		break;

	case PMIC_SIM1_VOL_CTL_REQ:
		pr_info("SIM2_VOL_CTL_REQ, vol = %d.\n", vol_mV);
		if(cpdata && cpdata->sim_power_config){
			ret = cpdata->sim_power_config(SIM1, vol_mV);
		}
		break;

	case PMIC_BAT_VOL_GET_REQ:
		pr_info("BATTERY_VOL_GET_REQ\n");
		break;

	default:
		pr_info("unknow pmic req\n");
		break;
	}

	/* write config result */
	if (ret)
		pmic_res = 0x0000ffff;
	else
		pmic_res = 0;
	pmic_res |= pmic_req & 0xffff0000;
	cpinfo->pmic_sim_res = pmic_res;

	return nr;
}

static ssize_t	cp_c83xx_ldisc_read(struct tty_struct * tty, struct file * file,
			unsigned char __user * buf, size_t nr)
{
	struct cp_c83xx_info *cpinfo = tty->disc_data;
	unsigned long pmic_sim_res = cpinfo->pmic_sim_res;

	pr_info("%s: Write set result to CP [0x%lx]\n", __func__, pmic_sim_res);

	sprintf(buf, "%lx", pmic_sim_res);

	return nr;
}

static struct tty_ldisc_ops rmnet_ldisc = {
  .owner = THIS_MODULE,
  .magic = TTY_LDISC_MAGIC,
  .name = "cp_ldisc",
  .open = cp_c83xx_ldisc_open,
  .close = cp_c83xx_ldisc_close,
  .read = cp_c83xx_ldisc_read,
  .write = cp_c83xx_ldisc_write,
};

static int cp_c83xx_ldisc_init(void)
{
	int ret;

	ret = tty_register_ldisc(N_C83XX, &rmnet_ldisc);
	if (ret != 0)
		pr_err("Failed to register ldisc for c83xx (%d)\n", ret);

    return ret;
}
static int cp_c83xx_ldisc_exit(void)
{
	int ret;

	ret = tty_unregister_ldisc(N_C83XX);
	if (ret != 0)
		pr_err("Failed to unregister ldisc for c83xx (%d)\n", ret);

    return ret;
}
#endif

static int cp_c83xx_cdev_open(struct inode *inodep, struct file *filep)
{
	if(!c83xx_modem_info){
		pr_err("[%s] c83xx_modem_info has not been inited\n", __func__);
		return -1;
	}

	filep->private_data = c83xx_modem_info;
	if(atomic_read(&c83xx_modem_info->cdev_count) > 0){
		pr_err("[%s] This devices has been opened\n", __func__);
		return -EBUSY;
	}
	atomic_inc(&c83xx_modem_info->cdev_count);

	return 0;
}

static ssize_t cp_c83xx_cdev_read(struct file *filep, char __user *buf,
			size_t size, loff_t *posp)
{
	struct cp_c83xx_info *cpinfo = filep->private_data;
	unsigned long pmic_sim_res = cpinfo->pmic_sim_res;
	char resbuf[10];
	int ret;

	if(atomic_read(&cpinfo->cdev_count) == 0){
		pr_err("[%s] The device has not been opened\n", __func__);
		return -1;
	}

	pr_info("Write result to CP [0x%lx]\n", pmic_sim_res);
	sprintf(resbuf, "%lx", pmic_sim_res);
	ret = copy_to_user(buf, resbuf, size);
	if(ret){
		pr_err("[%s] Cannot copy data(%d) to user\n", __func__, ret);
		return -EFAULT;
	}
	return size;
}

static ssize_t cp_c83xx_cdev_write(struct file *filep, const char __user *buf,
			size_t size, loff_t *posp)
{
	char reqbuf[10];
	unsigned long pmic_req, pmic_res;
	int vol_mV, ret = 0;
	struct cp_c83xx_info *cpinfo = filep->private_data;
	struct c83xx_platform_data *cpdata = cpinfo->cpdata;

	if(atomic_read(&cpinfo->cdev_count) == 0){
		pr_err("[%s] The device has not been opened\n", __func__);
		return -1;
	}

	ret = copy_from_user(reqbuf, buf, size);
	if(ret){
		pr_err("[%s] Cannot copy data(%d) from user\n", __func__, ret);
		return -EFAULT;
	}
	reqbuf[size] = '\0';
	pmic_req = simple_strtoul(reqbuf, NULL, 16);
	vol_mV = pmic_req & 0x0000ffff;

	switch (pmic_req >> 16) {
	case PMIC_SIM0_VOL_CTL_REQ:  // config sim0 card voltage
		CP_DEBUG("SIM1_VOL_CTL_REQ, vol = %d.\n", vol_mV);
		if(cpdata && cpdata->sim_power_config)
			ret = cpdata->sim_power_config(SIM0, vol_mV);

		if (ret)
			pmic_res = 0x0000ffff;
		else
			pmic_res = 0;

		break;

	case PMIC_SIM1_VOL_CTL_REQ:  // config sim2 card voltage
		CP_DEBUG("SIM2_VOL_CTL_REQ, vol = %d.\n", vol_mV);
		if(cpdata && cpdata->sim_power_config)
			ret = cpdata->sim_power_config(SIM1, vol_mV);

		if (ret)
			pmic_res = 0x0000ffff;
		else
			pmic_res = 0;
		break;

	case PMIC_BAT_VOL_GET_REQ:
		CP_DEBUG("BATTERY_VOL_GET_REQ\n");
		pmic_res = 0;
		break;

	case PMIC_USB_VOL_CTL_REQ:  // config usb voltage of c8320
		CP_DEBUG("USB_VOL_CTL_REQ,vol_mV is %d\n",vol_mV);
		if(cpdata && cpdata->cp_usb_power_config)
			ret = cpdata->cp_usb_power_config(vol_mV);

		if (ret)
			pmic_res = 0x0000ffff;
		else
			pmic_res = 0;
		break;

	case PMIC_CORE_VOL_CTL_REQ:  //config core voltage of c8320
		CP_DEBUG("CORE_VOL_CTL_REQ\n");
		pmic_res = 0;
		break;

	default:
		pr_info("Invalid pmic req(0x%lx)\n", pmic_req);
		pmic_res = 0x0000ffff;
		break;
	}

	/* write config result */
	pmic_res |= pmic_req & 0xffff0000;
	cpinfo->pmic_sim_res = pmic_res;

	return size;
}

static long cp_c83xx_cdev_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	struct cp_c83xx_info *cpinfo;
	struct c83xx_platform_data *cpdata;
	int ret = 0;

	cpinfo = filep->private_data;
	cpdata = cpinfo->cpdata;

	if(atomic_read(&cpinfo->cdev_count) == 0){
		pr_err("[%s] The device has not been opened\n", __func__);
		return -1;
	}

	switch(cmd){
		case IOC_CP_RESET:
			pr_info("c83xx reset\n");
			if(cpdata && cpdata->cp_reset)
				cpdata->cp_reset();
			set_cp_status(cpinfo, REBOOTING);
			break;
		case IOC_CP_POWON:
			pr_info("c83xx power on\n");
			if(cpdata && cpdata->cp_power_on)
				ret = cpdata->cp_power_on();
			if(cpinfo->cp_status == POWEROFF)
				set_cp_status(cpinfo, BOOTING);
			break;
		case IOC_CP_POWOFF:
			pr_info("c83xx power off\n");
			if(cpdata && cpdata->cp_power_off)
				ret = cpdata->cp_power_off();
			set_cp_status(cpinfo, POWEROFF);
			break;
		case IOC_CP_RELEASE:
			pr_info("c83xx release\n");
			break;
		case IOC_CP_BOOT:
			pr_info("c83xx boot\n");
			mod_timer(&cpinfo->a2c_reset_timer, jiffies + (HZ/10));
			wake_lock(&cpinfo->reset_wakelock);
			break;
		case IOC_CP_UART_START:
			printk("write interrupt value is 1\n");
			c8320_modem_ctl_ex.phone_state = STATE_BOOTING;
			break;

		default:
			pr_info("[%s] unknown command[%d]\n", __func__,cmd);
			ret = -1;
			break;
	}

	return ret;
}

static int cp_c83xx_cdev_release(struct inode *inodep, struct file *filep)
{
	struct cp_c83xx_info *cpinfo = filep->private_data;

	if(atomic_read(&cpinfo->cdev_count) > 0)
		atomic_dec(&cpinfo->cdev_count);

	return 0;
}

static const struct file_operations cp_c83xx_ops = {
	.open = cp_c83xx_cdev_open,
	.write = cp_c83xx_cdev_write,
	.read = cp_c83xx_cdev_read,
	.unlocked_ioctl = cp_c83xx_cdev_ioctl,
	.release = cp_c83xx_cdev_release,
};

static int cp_c83xx_cdev_init(struct cdev *cdevp)
{
	int ret;
	struct cp_c83xx_info *cpinfo;

	cpinfo = container_of(cdevp, struct cp_c83xx_info, cdev);

	ret = alloc_chrdev_region(&cpinfo->devno, 0, 1, "cp_c83xx");
	if(ret < 0){
		pr_err("Cannot alloc devno for c83xx\n");
		return ret;
	}
	printk("[%s] major: %d\n", __func__, MAJOR(cpinfo->devno));
	cdev_init(cdevp, &cp_c83xx_ops);
	cdevp->owner = THIS_MODULE;
	ret = cdev_add(cdevp, cpinfo->devno, 1);
	if(ret){
		pr_err("Failed to add cdev of c83xx\n");
		unregister_chrdev_region(cpinfo->devno, 1);
		return ret;
	}
	printk("[%s] init finished\n", __func__);

	return 0;
}

static int cp_c83xx_cdev_exit(struct cdev *cdevp)
{
	struct cp_c83xx_info *cpinfo;

	cpinfo = container_of(cdevp, struct cp_c83xx_info, cdev);
	cdev_del(cdevp);
	unregister_chrdev_region(cpinfo->devno, 1);

	return 0;
}

static int __devinit cp_c83xx_probe(struct platform_device *pdev)
{
	int err = 0;
	struct cp_c83xx_info *cpinfo;
	struct c83xx_platform_data *cpdata;
	struct resource *c2a_abort_res, *c2a_wake_uart_res;  //*c2a_wake_hsic_res;
	struct device *dev;

	cpinfo = kzalloc(sizeof(struct cp_c83xx_info), GFP_KERNEL);
	if(cpinfo == NULL){
		pr_err("[%s] Failed to alloc c83xx_modem\n", __func__);
		return -ENOMEM;
	}

	err = cp_c83xx_cdev_init(&cpinfo->cdev);
	if(err){
		kfree(cpinfo);
		return err;
	}

#if 0
	err = cp_c83xx_ldisc_init();
	if(err)
		goto cp_cdev_error;
#endif

	cpinfo->dev = &pdev->dev;
	cpinfo->cpdata = pdev->dev.platform_data;
	cpinfo->cp_pm_uart = cpinfo->cpdata->cp_pm_uart;
	cpinfo->cp_pm_hsic = cpinfo->cpdata->cp_pm_hsic;
	cpinfo->notifiers = &cp_c83xx_notifiers;

	atomic_set(&cpinfo->cdev_count, 0);
	wake_lock_init(&cpinfo->wakeup_uart_wakelock, WAKE_LOCK_SUSPEND,
				"cp_wakeup_uart_wakelock");
	wake_lock_init(&cpinfo->wakeup_hsic_wakelock, WAKE_LOCK_SUSPEND,
				"cp_wakeup_hsic_wakelock");
	wake_lock_init(&cpinfo->abort_wakelock, WAKE_LOCK_SUSPEND,
				"cp_abort_wakelock");
	wake_lock_init(&cpinfo->reset_wakelock, WAKE_LOCK_SUSPEND,
				"cp_reset_wakelock");
	spin_lock_init(&cpinfo->slock);

	INIT_WORK(&cpinfo->notify_wq, cp_notify_wq_handler);
	INIT_WORK(&cpinfo->rest_wq, cp_rest_wq_handler);
	INIT_DELAYED_WORK(&cpinfo->abort_wq, cp_abort_wq_handler);

	setup_timer(&cpinfo->a2c_status_timer, a2c_status_timer_func, (unsigned long)cpinfo);
	setup_timer(&cpinfo->a2c_reset_timer, a2c_reset_timer_func, (unsigned long)cpinfo);

	platform_set_drvdata(pdev, cpinfo);

	cpdata = cpinfo->cpdata;
	err = cpdata->cp_init();
	if(err){
		pr_err("[%s] cp init error\n", __func__);
		goto cp_probe_error;
	}

	cpinfo->cp_status = POWEROFF;

	c83xx_modem_info = cpinfo;

	c2a_abort_res = platform_get_resource(pdev,IORESOURCE_IRQ, 0);
	if(!c2a_abort_res){
		pr_err("[%s] Failed to get c2a abort resource\n", __func__);
		goto cp_probe_error;
	}
	cpinfo->c2a_abort_irq = c2a_abort_res->start;
	err = request_threaded_irq(cpinfo->c2a_abort_irq,
				NULL,
				cp2ap_abort_handler,
				IRQF_TRIGGER_FALLING,
				"c2a_abort",
				(void *)cpinfo);
	if (err != 0){
		pr_err("[%s] failed to request c2a abort irq: %d\n", __func__, err);
		goto cp_probe_error;
	}

	c2a_wake_uart_res = platform_get_resource(pdev,IORESOURCE_IRQ, 1);
	if(!c2a_wake_uart_res){
		pr_err("[%s] Failed to get c2a wakeup resource when using uart\n", __func__);
		goto cp_probe_error;
	}
	cpinfo->c2a_wake_uart_irq = c2a_wake_uart_res->start;
	err = request_threaded_irq(cpinfo->c2a_wake_uart_irq,
				NULL,
				cp2ap_wake_uart_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"c2a_wakeup_uart",
				(void *)cpinfo);
	if (err != 0){
		pr_err("[%s] failed to request c2a wakeup irq when using uart: %d\n", __func__, err);
		goto cp_probe_error;
	}
#if 0
	c2a_wake_hsic_res = platform_get_resource(pdev,IORESOURCE_IRQ, 2);
	if(!c2a_wake_hsic_res){
		pr_err("[%s] Failed to get c2a wakeup resource when using hsic\n", __func__);
		goto cp_probe_error;
	}
	cpinfo->c2a_wake_hsic_irq = c2a_wake_hsic_res->start;
	err = request_threaded_irq(cpinfo->c2a_wake_hsic_irq,
				NULL,
				cp2ap_wake_hsic_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"c2a_wakeup_hsic",
				(void *)cpinfo);
	if (err != 0){
		pr_err("[%s] failed to request c2a wakeup irq when using hsic: %d\n", __func__, err);
		goto cp_probe_error;
	}
#endif
	enable_irq_wake(cpinfo->c2a_abort_irq);
	enable_irq_wake(cpinfo->c2a_wake_uart_irq);
	//enable_irq_wake(cpinfo->c2a_wake_hsic_irq);

	cpinfo->cp_class = class_create(THIS_MODULE, "cp_class");
	if (IS_ERR(cpinfo->cp_class)) {
		pr_err("[%s] Cannot creat class for cp\n", __func__);
		err = PTR_ERR(cpinfo->cp_class);
		goto cp_probe_error;
	}

	dev = device_create(cpinfo->cp_class, cpinfo->dev, cpinfo->devno, NULL, "cp_c83xx");
	if (IS_ERR(dev)) {
		pr_err("[%s] Cannot creat device for cp\n", __func__);
		err = PTR_ERR(dev);
		goto cp_probe_error;
	}
	cpinfo->chdev = dev;

	device_create_file(cpinfo->dev, &dev_attr_command);
	device_create_file(cpinfo->dev, &dev_attr_status);
	device_create_file(cpinfo->dev, &dev_attr_manual_reset);

	printk("[%s] init finished\n", __func__);
	return 0;

cp_probe_error:
	//cp_c83xx_ldisc_exit();
//cp_cdev_error:
	cp_c83xx_cdev_exit(&cpinfo->cdev);
	kfree(cpinfo);

	return err;
}

//extern int cp_activate_type;
static int cp_c83xx_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct cp_c83xx_info *cpinfo;
	struct c83xx_platform_pm *cp_pm_uart;
	struct c83xx_platform_pm *cp_pm_hsic;

	cpinfo = platform_get_drvdata(pdev);
	cp_pm_uart = cpinfo->cp_pm_uart;
	cp_pm_hsic = cpinfo->cp_pm_hsic;

	if(cp_pm_uart && cp_pm_uart->a2c_status_update)
		cp_pm_uart->a2c_status_update(1);

	if(cp_pm_hsic&& cp_pm_hsic->a2c_status_update)
		cp_pm_hsic->a2c_status_update(1);

	return 0;
}

static int cp_c83xx_resume(struct platform_device * pdev)
{
	struct cp_c83xx_info *cpinfo;

	cpinfo = platform_get_drvdata(pdev);
	mod_timer(&cpinfo->a2c_status_timer, jiffies + A2C_STATUS_DELAY);

	return 0;
}

static void cp_c83xx_shutdown(struct platform_device *pdev)
{
	struct cp_c83xx_info *cpinfo;

	cpinfo = platform_get_drvdata(pdev);
	free_irq(cpinfo->c2a_abort_irq, (void *)cpinfo);
	free_irq(cpinfo->c2a_wake_uart_irq, (void *)cpinfo);
	//free_irq(cpinfo->c2a_wake_hsic_irq, (void *)cpinfo);
}

static int __exit cp_c83xx_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_command);
	device_remove_file(&pdev->dev, &dev_attr_status);
	device_remove_file(&pdev->dev, &dev_attr_manual_reset);

	return 0;
}

static struct platform_driver cp_c83xx_driver = {
	.driver = {
		.name = "cp_c83xx",
		.owner = THIS_MODULE,
	},
	.probe = cp_c83xx_probe,
	.suspend = cp_c83xx_suspend,
	.resume = cp_c83xx_resume,
	.shutdown = cp_c83xx_shutdown,
	.remove = __exit_p(cp_c83xx_remove),
};

static int __init cp_c83xx_init(void)
{
	return platform_driver_register(&cp_c83xx_driver);
}
late_initcall_sync(cp_c83xx_init);

static void __exit cp_c83xx_exit(void)
{
	platform_driver_unregister(&cp_c83xx_driver);
}
module_exit(cp_c83xx_exit);

MODULE_DESCRIPTION("Driver for c83xx modem");
MODULE_LICENSE("GPL");

