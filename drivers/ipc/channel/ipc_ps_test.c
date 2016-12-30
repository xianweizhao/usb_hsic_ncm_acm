/*
 *  linux/kernel/drivers/misc/ipc/ipc_ps_test.c
 *
 *  Copyright (C) 2010-2020 Chongqing CYIT Co., Ltd.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/tty.h>
#include <linux/netdevice.h>
#include <linux/poll.h>
#include <linux/crc-ccitt.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/types.h>
#include <linux/compiler.h>
#include <linux/signal.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/etherdevice.h>
#include <linux/ipv6.h>
#include <linux/ip.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <asm/uaccess.h>
#include <asm/string.h>
#include <asm/byteorder.h>
#include <asm/atomic.h>

#include <mach/ipc.h>
#include <mach/hardware.h>

#include "../../usb/gadget/f_common.h"

static int ps_mask = 1;
static int dbg_open = 1;
module_param_named(dbg_mask, ps_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);

#define TAG	"[IPC_PS_TEST] "
#define PDBG(fmt, args...) do {\
								if ((ps_mask && dbg_open)) \
									printk(TAG fmt, ##args);\
				      		} while(0)

/* test mode defines */
#define MAX_NET_IF          	6   
#define FIFO_SIZE				(128*1024)
#define TEMPBUF_LEN				(2*1024)
#define IP_PACKET_LEN			1500
#define HEAD_LEN				4
#define TAIL_LEN				2
#define ONE_PACKET_LEN			(HEAD_LEN + IP_PACKET_LEN + TAIL_LEN)

struct ipc_ps_usb_dev {
	int channel;
	
	struct mutex lock;
	struct common_dev *ps_dev;
	struct work_struct ps_work;

	struct task_struct *ps_i2ud;
	struct task_struct *ps_u2id;

	struct kfifo fifo;
	char *buf;
	char *buf_w;

	wait_queue_head_t w_wait;
	wait_queue_head_t r_wait;

	int commom_dev_num;
	bool is_connect;
};

struct ipc_ps_usb_dev *ps_usb[MAX_NET_IF] = {NULL,};
static int ps_tcount = 0;

int is_cp_alone_test_mode(void)
{
	char *buf = saved_command_line;
	char *ret = NULL;
	int len = strlen(buf);
	char tmp[1024];

	if (len > (sizeof(tmp) - 1)) {
		return -EINVAL;
	}

	strcpy(tmp, buf);
	ret = strstr(tmp, "androidboot.mode=cptestmode");
	if (ret) {
		pr_info("ipc ps into cp alone test mode\n");
		return true;
	}
	return false;
}

static long ps_usb_int_handler(int id, void *priv, int event)
{
	struct ipc_ps_usb_dev *ptx = (struct ipc_ps_usb_dev *)priv;
	int ch_id = IPC_CHANNEL_PS1_ID + ptx->channel;

	if (id != ch_id) {
		pr_err("%s: ID error, id = %d, ch_id = %d\n", __func__, id, ch_id);
		return IRQ_HANDLED;
	}

	if (event & R_AVAIL) {
		ipc_disable_interrupt(ch_id, R_AVAIL_INT);
		wake_up_interruptible(&ptx->r_wait);
	}

	if (event & W_AVAIL) {
		ipc_disable_interrupt(ch_id, W_AVAIL_INT);
		wake_up_interruptible(&ptx->w_wait);
	}

	return IRQ_HANDLED;
}

/*
 * CP test mode operations
 */
static int ps_i2u_thread(void *arg)
{
	struct ipc_ps_usb_dev *dev = arg;
	int len = 0, ret = 0;
	int timeout = (30*HZ)/1000;
	int ch_id = IPC_CHANNEL_PS1_ID + dev->channel;

	ret = usb_common_dev_open(ps_usb[dev->channel]->ps_dev);
	if (ret < 0) {
		pr_err("%s: Open USB failed\n", __func__);
	}

	do {	
		len = ipc_read_avail(ch_id);
		if (len > 0) {
			if(!dev->is_connect){
				break;
			}
			ret = usb_common_dev_write(ps_usb[dev->channel]->ps_dev, NULL, len);
			if (ret < 0) {
				PDBG("%s%d: write usb error.\n", __func__, dev->channel);
			}
		} else {
			ipc_enable_interrupt(ch_id, R_AVAIL_INT);
			wait_event_interruptible_timeout(ps_usb[dev->channel]->r_wait,
				((ipc_read_avail(ch_id) > 0)|| (!dev->is_connect)), timeout);
		}
	} while ((dev->is_connect)&&(!kthread_should_stop()));
	
	pr_err("===%s===exit===\n", __func__);
	return 0;
}

static int ps_u2i_thread(void *arg)
{
	struct ipc_ps_usb_dev *dev = arg;
	struct __kfifo *fifo = &dev->fifo.kfifo;
	struct common_dev *ps_dev = dev->ps_dev;
	int ch_id = IPC_CHANNEL_PS1_ID + dev->channel;
	int ret;
	unsigned int fifo_len, len1, len2;
	char head_t[2], tail_t[2];
	
	ret = usb_common_dev_open(ps_dev);
	if (ret < 0) {
		PDBG("%s: Open USB failed\n", __func__);
		return -1;
	}

	do {
		ret = ipc_write_avail(ch_id);
		if (ret < IP_PACKET_LEN) {
			ipc_enable_interrupt(ch_id, W_AVAIL_INT);
			printk("wait PS%d writeable.\n", dev->channel);
			wait_event_interruptible(dev->w_wait, (ipc_write_avail(ch_id) > IP_PACKET_LEN)||(!dev->is_connect));
		}
		if(!dev->is_connect){
			break;
		}

		/* read data to fifo */
		fifo_len = kfifo_len(&dev->fifo);
		if (fifo_len < ONE_PACKET_LEN) {
			unsigned int len = kfifo_avail(&dev->fifo);
			unsigned int off = fifo->in & fifo->mask;
			unsigned int l = min(len, fifo->mask + 1 - off);

			len1 = usb_common_dev_read(ps_dev, dev->buf + off, l);
			if (len1 < l) {
				smp_wmb();
				fifo->in += len1;
			} else {
				len2 = usb_common_dev_read(ps_dev, dev->buf, len - l);
				smp_wmb();
				fifo->in += len1 + len2;
			}
		}

		/* get head */
		head_t[0] = dev->buf[fifo->out & fifo->mask];
		head_t[1] = dev->buf[(fifo->out + 1) & fifo->mask];
		
		/* get len */
		len1 = dev->buf[(fifo->out + 2) & fifo->mask] << 8 | // PC Tool passed 
               dev->buf[(fifo->out + 3) & fifo->mask];
		len2 = dev->buf[(fifo->out + 6) & fifo->mask] << 8 | // IP Head passed
               dev->buf[(fifo->out + 7) & fifo->mask];
		
		/* get tail */
		tail_t[0] = dev->buf[(fifo->out + HEAD_LEN + len1) & fifo->mask];
		tail_t[1] = dev->buf[(fifo->out + HEAD_LEN + len1 + 1) & fifo->mask];

		if (head_t[0] == 0xaa && head_t[1] == 0xaa && 
			len1 == len2 && 
			tail_t[0] == 0xbb && tail_t[1] == 0xbb) 
		{
			int i;
			
			/* skip head */
			for (i = 0; i < HEAD_LEN; i++)
				kfifo_skip(&dev->fifo);

			/* get ip packet */
			ret = kfifo_out(&dev->fifo, dev->buf_w, len1);
			WARN_ON(ret != len1);

			/* write to smd */
			ret = ipc_write(ch_id, dev->buf_w, len1);
			if (ret != len1) {
				PDBG("%s: ipc_write failed, %d != %d\n", __func__, ret, len1);
				return ret;
			}

			/* skip tail */
			for (i = 0; i < TAIL_LEN; i++)
				kfifo_skip(&dev->fifo);
		} 
		else
		{
			PDBG("Packet is broken: \n");
			PDBG("head is 0x%x 0x%x\n", head_t[0], head_t[1]);
			PDBG("len1 = %d, len = %d\n", len1, len2);
			PDBG("tail is 0x%x 0x%x\n", tail_t[0], tail_t[1]);
		}
	} while ((dev->is_connect)&&(!kthread_should_stop()));

	return 0;
}

static void ps_test_work(struct work_struct *work)
{
	struct ipc_ps_usb_dev *dev = container_of(work, struct ipc_ps_usb_dev, ps_work);
	int unit = dev->ps_dev->port_num;
	char *name = NULL;
	
	name = kasprintf(GFP_KERNEL, "%s_%d", "ps_i2ud", unit);
	ps_usb[unit]->ps_i2ud = kthread_run(ps_i2u_thread, ps_usb[unit], name);
	if (IS_ERR(ps_usb[unit]->ps_i2ud)) {
		 PDBG("%s: kthread_run %s failed\n", __func__, name);
	}

	name = kasprintf(GFP_KERNEL, "%s_%d", "ps_u2id", unit);
	ps_usb[unit]->ps_u2id = kthread_run(ps_u2i_thread, ps_usb[unit], name);
	if (IS_ERR(ps_usb[unit]->ps_u2id)) {
		 PDBG("%s: kthread_run %s failed\n", __func__, name);
	}
}

static int ps_test_probe(struct common_dev *com)
{
	int unit, ch_id, ret;

	unit = com->port_num;
	ch_id = IPC_CHANNEL_PS1_ID + unit;
	
	ps_usb[unit] = kzalloc(sizeof(struct ipc_ps_usb_dev), GFP_KERNEL);
	ps_usb[unit]->channel = com->port_num;
	ps_usb[unit]->ps_dev = com;
	ps_usb[unit]->ps_dev->channel_id = ch_id;

	ps_usb[unit]->buf = kzalloc(FIFO_SIZE, GFP_KERNEL);
	if (!ps_usb[unit]->buf) {
		PDBG("%s: kzalloc fifo buf failed.\n", __func__);
		return -ENOMEM;
	}

	ret = kfifo_init(&ps_usb[unit]->fifo, ps_usb[unit]->buf, FIFO_SIZE);
	if (ret < 0) {
		PDBG("%s: init kfifo failed!\n", __func__);
		kfree(ps_usb[unit]->buf);
		return -1;
	}

	ps_usb[unit]->buf_w = kzalloc(TEMPBUF_LEN, GFP_KERNEL);
	if (!ps_usb[unit]->buf_w) {
		PDBG("%s: kzalloc temp buf failed.\n", __func__);
		return -ENOMEM;
	}

	INIT_WORK(&ps_usb[unit]->ps_work, ps_test_work);
	init_waitqueue_head(&ps_usb[unit]->w_wait);
	init_waitqueue_head(&ps_usb[unit]->r_wait);

	ret = ipc_register_inthandle(ch_id, ps_usb[unit], ps_usb_int_handler);
	if (ret < 0) {
		PDBG("%s: register inthandle PS%d failed\n", __func__, unit);
		return ret;
	}
	ipc_disable_interrupt(ch_id, RW_AVAIL_INT);

	return 0;
}

static int ps_test_remove(struct common_dev *com)
{
	int unit;
	
	PDBG("%s()\n", __func__);

	unit = com->port_num;
	ps_usb[unit]->channel = 0;
	ps_usb[unit]->ps_dev = NULL;

	kfree(ps_usb[unit]->buf);
	kfree(ps_usb[unit]->buf_w);
	//kfree(ps_usb[unit]);

	return 0;
}

static int ps_test_connect(struct common_dev *com)
{
	int unit;

	PDBG("=== %s: com->port_num -> %d ===\n", __func__, com->port_num);
	unit = com->port_num;
	ps_usb[unit]->is_connect = true;
	schedule_work(&ps_usb[unit]->ps_work);
	return 0;
}

static int ps_test_disconnect(struct common_dev *com)
{
	int unit;
	
	PDBG("common_dev->port_num = %d", com->port_num);
	unit = com->port_num;
	ps_usb[unit]->is_connect = false;
	wake_up_interruptible(&ps_usb[unit]->r_wait);
	wake_up_interruptible(&ps_usb[unit]->w_wait);

	return 0;
}

static struct usb_common_driver ps_test_driver = {
	.name 		= "ipc_ps",
	.probe 		= ps_test_probe,
	.remove 	= ps_test_remove,
	.connect 	= ps_test_connect,
	.disconnect = ps_test_disconnect,
};

static int create_ps_test_channel(void)
{
	int ret = 0;
	unsigned int n = 0;
	struct ipc_channel ch;

	for (n = 0; n < ps_tcount; n++) {
		sprintf(ch.name, "IPC_PS%d", n + 1);
		ch.type = IPC_TYPE_PACKET;
		ch.c2a_buf_len = CHANNEL_BUF_128KB;
		ch.a2c_buf_len = CHANNEL_BUF_128KB;
		ch.id = IPC_CHANNEL_PS1_ID + n;
		ch.wake_flag = CH_WAKEUP_ENABLE;

		ret = ipc_create_ch(&ch);
		if (ret < 0) {
			pr_err("ipc_create_ch ps%d fialed.\n", n);
			return ret;
		} else {
			pr_info("ipc_create_ch ps%d successful.\n", n);
		}
	}

	return ret;
}

static int __init ipc_ps_test_init(void) 
{
	int ret = 0;
	int cpid = 0, sim_flag = 0;
	
	if (is_cp_alone_test_mode()) {
		PDBG("Into CP test mode.\n");

		cpid = get_boardid_for_cp();
		sim_flag = (cpid & 0xF0000) >> 16;
		if (sim_flag == SINGLESIM) {
			ps_tcount = 3; 
		} else if (sim_flag == DUALSIM) {
			ps_tcount = 6;
		}
		PDBG("cpid->0x%x, sim_flag->0x%x, ps_tcount = %d\n", 
			cpid, sim_flag, ps_tcount);
		
		ret = create_ps_test_channel();
		if (ret < 0) {
			PDBG("creat_ps_test_channel failed\n");
			return ret;
		}

		ret = register_common_driver(&ps_test_driver);
		if (ret < 0) {
			PDBG("register ps_test_driver failed\n");
			return ret;
		}
	}

	return 0;
}
late_initcall_sync(ipc_ps_test_init);
