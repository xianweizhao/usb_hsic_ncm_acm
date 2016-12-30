/*
 *  linux/drivers/misc/ipc/ipc_cs.c
 *
 *  Copyright (C) 2010-2020 Chongqing CYIT Co., Ltd.
 *
 */
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

#include <linux/scatterlist.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#include <linux/export.h>

#include <mach/ipc.h>
#include "../../usb/gadget/f_common.h"

/*debug open: mask = 1 means open debug info*/
static int cs_mask = 0;
static int dbg_open = 1;
module_param_named(dbg_mask, cs_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define CS_DBG(fmt, args...) do{\
								if ((cs_mask && dbg_open)) \
									printk(KERN_INFO fmt, ##args);\
				      			}while(0)

struct ipc_cs_dev{
	int ch_stats;
	struct mutex lock;
	wait_queue_head_t w_wait;
	wait_queue_head_t r_wait;

	struct common_dev *cs_dev;
	struct task_struct *cs_ipc_to_usb_thread;
	struct task_struct *cs_usb_to_ipc_thread;
	struct work_struct ipc_cs_usb_work;

	char *buf_r; /* tmp buffer read from ipc */
	char *buf_w; /* tmp buffer to write to ipc */
};

static struct ipc_cs_dev *cs;
static int bufsize = 1024;

static int is_test_mode(void)
{
	char *buf = saved_command_line;
	int len = strlen(buf);
	char tmp[1024];
	char *ret = NULL;

	if (len > bufsize)
		return -EINVAL;

	strcpy(tmp, buf);
	ret = strstr(tmp, "androidboot.mode=cptestmode");
	if (ret) {
		printk("ipc cs into cptestmode\n");
		return true;
	}

	return false;
}

/*
 * Test mode operations
 */

//????? fix me , because this is packet channel.
static int cs_ipc_to_usb_thread(void *arg)
{
    int r_len, w_len;
	int ret;

	do {
		ret = cs->cs_dev->ops->open(cs->cs_dev);
		if (ret)
			CS_DBG("%s(): Open USB failed\n", __func__);

		msleep(500);
	} while (ret != 0);

	do {
		if ((r_len = ipc_read_avail(IPC_CHANNEL_CS_ID)) <= 0) {
			ipc_enable_interrupt(IPC_CHANNEL_CS_ID, R_AVAIL_INT);
			wait_event_interruptible(cs->r_wait, (r_len = ipc_read_avail(IPC_CHANNEL_CS_ID)) > 0);
		}

		r_len = ipc_read(IPC_CHANNEL_CS_ID, cs->buf_r, r_len);

		while (r_len > 0) {
			w_len = cs->cs_dev->ops->write(cs->cs_dev, cs->buf_r, r_len);
			r_len = r_len - w_len;
		}
	} while (!kthread_should_stop());

	return ret;
}



//????? fix me , because this is packet channel.

static int cs_usb_to_ipc_thread(void *arg)
{

	int r_len, w_len;
	int ret;

	do {
		ret = cs->cs_dev->ops->open(cs->cs_dev);
		if (ret)
			CS_DBG("%s(): Open USB failed\n", __func__);

		msleep(500);
	} while (ret != 0);

	do {
        if ((w_len = ipc_write_avail(IPC_CHANNEL_CS_ID)) <= 0) {
			ipc_enable_interrupt(IPC_CHANNEL_CS_ID, W_AVAIL_INT);
			wait_event_interruptible(cs->w_wait, (w_len = ipc_write_avail(IPC_CHANNEL_CS_ID)) > 0);
		}

		while (w_len > 0) {
			r_len = cs->cs_dev->ops->read(cs->cs_dev, cs->buf_w, w_len);
			ret = ipc_write(IPC_CHANNEL_CS_ID, cs->buf_w, r_len);
			w_len = w_len - r_len;
		}
	} while (!kthread_should_stop());

	return ret;
}

static void ipc_cs_work(struct work_struct *work)
{
	cs->cs_ipc_to_usb_thread = kthread_run(cs_ipc_to_usb_thread, (void *)NULL,
												"cs-ipc-to-usb");
	if (IS_ERR(cs->cs_ipc_to_usb_thread)) {
	     pr_err("Run cs_ipc_to_usb_thread failed\n");
	}

	cs->cs_usb_to_ipc_thread = kthread_run(cs_usb_to_ipc_thread, (void *)NULL,
												"cs-usb-to-ipc");
	if (IS_ERR(cs->cs_usb_to_ipc_thread)) {
	     pr_err("Run cs_usb_to_ipc_thread failed\n");
	}
}

static int ipc_cs_probe(struct common_dev *com)
{
	cs->cs_dev = com;

	cs->buf_r = kzalloc(4096, GFP_KERNEL);
	if (cs->buf_r == NULL)
		return -ENOMEM;

	cs->buf_w = kzalloc(4096, GFP_KERNEL);
	if (cs->buf_w == NULL)
		return -ENOMEM;

	INIT_WORK(&cs->ipc_cs_usb_work, ipc_cs_work);

	return 0;
}

static int ipc_cs_remove(struct common_dev *com)
{
	kfree(cs->buf_r);
	kfree(cs->buf_w);

	return 0;
}

/*
 * this function works in interupt context, so we use
 * workqueue here.
 */
static int ipc_cs_connect(struct common_dev *com)
{
	//schedule_work(&cs->ipc_cs_usb_work);
	return 0;
}

static int ipc_cs_disconnect(struct common_dev *com)
{
	//kthread_stop(cs->cs_ipc_to_usb_thread);
	//kthread_stop(cs->cs_usb_to_ipc_thread);

	return 0;
}

static struct usb_common_driver ipc_cs_func_driver = {
	.name = "ipc_cs",
	.probe = ipc_cs_probe,
	.remove = ipc_cs_remove,
	.connect = ipc_cs_connect,
	.disconnect = ipc_cs_disconnect,
};


/*
 * Normal mode operations
 */
static int ipc_cs_open(struct inode *inode, struct file *filp)
{
	printk("%s()\n", __func__);

	if (cs->ch_stats == 0)
		return -ENODEV;

	filp->private_data = cs;

	return 0;
}

static int ipc_cs_close(struct inode *inode, struct file *filp)
{
	printk("%s()\n", __func__);

	return 0;
}

static ssize_t ipc_cs_read(struct file *filp, char __user *buf,
								size_t count, loff_t *f_pos)
{
	int len = 0;
	int ret = 0;
	struct ipc_cs_dev *dev = filp->private_data;

	//printk("%s()\n", __func__);

	if ((len = ipc_read_avail(IPC_CHANNEL_CS_ID)) <= 0)
	{
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ipc_enable_interrupt(IPC_CHANNEL_CS_ID, R_AVAIL_INT);
		wait_event_interruptible(dev->r_wait, (len = ipc_read_avail(IPC_CHANNEL_CS_ID)) > 0);
	}
    if(len > count)
		len = count;

	ret = ipc_read_to_user(IPC_CHANNEL_CS_ID, buf, len);

	return ret;
}

static ssize_t ipc_cs_write(struct file *filp, const char __user *buf,
								size_t count, loff_t *f_pos)
{
	int len = 0;
	int ret = 0;
	struct ipc_cs_dev *dev = filp->private_data;

	//printk("%s()\n", __func__);

	if ((len = ipc_write_avail(IPC_CHANNEL_CS_ID)) <= 0)
	{
	    if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ipc_enable_interrupt(IPC_CHANNEL_CS_ID, W_AVAIL_INT);
		wait_event_interruptible(dev->w_wait, (len = ipc_write_avail(IPC_CHANNEL_CS_ID)) > 0);
	}
    if(len > count)
		len = count;

	ret = ipc_write_from_user(IPC_CHANNEL_CS_ID, buf, len);

	return ret;
}

static struct file_operations cs_ops = {
	.owner 		= THIS_MODULE,
	.open  		= ipc_cs_open,
	.release 	= ipc_cs_close,
	.read  		= ipc_cs_read,
	.write 		= ipc_cs_write,
};

static struct miscdevice cs_miscdev = {
	.minor = 14,
	.name = "ipc_cs",
	.fops = &cs_ops,
};


static long cs_int_handler(int id, void *priv, int event)
{
	struct ipc_cs_dev *ctx = (struct ipc_cs_dev *)priv;

	if (id != IPC_CHANNEL_CS_ID)
		return IRQ_HANDLED;

	if (event & R_AVAIL) {
		ipc_disable_interrupt(IPC_CHANNEL_CS_ID, R_AVAIL_INT);
		wake_up_interruptible(&ctx->r_wait);
	}

	if(event & W_AVAIL){
		ipc_disable_interrupt(IPC_CHANNEL_CS_ID, W_AVAIL_INT);
		wake_up_interruptible(&ctx->w_wait);
	}

	return IRQ_HANDLED;
}

static int create_cs_channel(struct ipc_cs_dev *dev)
{
	int ret = 0;
	struct ipc_channel ch;

	sprintf(ch.name, IPC_CHANNEL_CS_NAME);
	ch.type = IPC_TYPE_STREAM;
	ch.c2a_buf_len = CHANNEL_BUF_128KB;
	ch.a2c_buf_len = CHANNEL_BUF_128KB;
	ch.id = IPC_CHANNEL_CS_ID;
	ch.wake_flag = CH_WAKEUP_ENABLE;

	ret = ipc_create_ch(&ch);
	if (ret < 0) {
		dev->ch_stats = 0;
		return ret;
	} else {
		dev->ch_stats = 1;
	}

	return ret;
}


static int __init ipc_cs_init(void)
{
	int ret = 0;

	printk("%s()\n", __func__);

	cs = kzalloc(sizeof(struct ipc_cs_dev), GFP_KERNEL);
	if (!cs) {
		return -ENOMEM;
	}

	ret = create_cs_channel(cs);
	if (ret < 0) {
		printk(KERN_ERR "Create cs channel failed\n");
		goto fail_create_cs_channel;
	}

	/* test mode */
	if (is_test_mode()) {
		printk("ipc_cs_init into test mode.\n");

        ret = register_common_driver(&ipc_cs_func_driver);
		if (ret < 0) {
		printk(KERN_ERR "ipc_cs: register_common_driver failed\n");
		goto fail_register;
		}
	}
	else {
		printk("ipc_cs_init into normal mode.\n");

		mutex_init(&cs->lock);
		init_waitqueue_head(&cs->w_wait);
		init_waitqueue_head(&cs->r_wait);

		ret = misc_register(&cs_miscdev);
		if (ret < 0) {
			printk(KERN_ERR "ipc_cs: misc_register failed\n");
			goto fail_register;
		}
	}

	ipc_disable_interrupt(IPC_CHANNEL_CS_ID, RW_AVAIL_INT);
	ipc_register_inthandle(IPC_CHANNEL_CS_ID, cs, cs_int_handler);

	return ret;

fail_register:
//	ipc_destroy_ch(IPC_CHANNEL_CS_ID);

fail_create_cs_channel:
	kfree(cs);
	return ret;
}
late_initcall_sync(ipc_cs_init);
