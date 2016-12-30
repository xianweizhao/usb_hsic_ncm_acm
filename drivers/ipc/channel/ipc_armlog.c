/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/export.h>
#include <linux/kfifo.h>
#include <mach/ipc.h>

#include "../../usb/gadget/f_common.h"

#define ATAG		"ipc_armlog"

enum trace_mode {
	NORMAL_MODE = 0,
	TEST_MODE
};

struct ipc_armlog_dev {
	int usb_trace_working;
	atomic_t trace_mode;
	atomic_t stop_thread;
	
	struct device *dev;
	struct cdev *cdev;
	struct class *armlog_class;
	
	struct ipc_channel ch;
	
	struct workqueue_struct *armlog_wq;
	struct work_struct armlog_work;
	struct task_struct *armlog_thread;
	
	wait_queue_head_t read_wait;
	
    struct common_dev *com_dev;
	struct mutex read_lock;
};

struct ipc_armlog_dev *armlog_dev;

static ssize_t tracemode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int temp = atomic_read(&armlog_dev->trace_mode);
	return sprintf(buf, "%d\n", temp);
}
static ssize_t tracemode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	
	sscanf(buf, "%d", &value);	
	atomic_set(&armlog_dev->trace_mode, value);

	return size;
}
static DEVICE_ATTR(tracemode, S_IRUGO | S_IWUGO, tracemode_show, tracemode_store);

static int ipc_armlog_open(struct inode *inode, struct file *file)
{
	printk("%s()\n", __func__);
	file->private_data = armlog_dev;
	return 0;
}

static int ipc_armlog_close(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long ipc_armlog_int_handle(int id, void *priv, int event)
{
	if (id != IPC_CHANNEL_ARMLOG_ID)
		return IRQ_HANDLED;

	if (event & R_AVAIL) {
		ipc_disable_interrupt(IPC_CHANNEL_ARMLOG_ID, R_AVAIL_INT);
		wake_up_interruptible(&armlog_dev->read_wait);
	}

	return IRQ_HANDLED;
}

static ssize_t ipc_armlog_read(struct file *file, char __user *buf, size_t count,
			  loff_t *ppos)
{
	int len = 0, ret = 0;

	if (unlikely(armlog_dev->usb_trace_working)) {
		//printk("%s: ret = %d\n", __func__, ret);
		return 0;
	} else {
		if ((len = ipc_read_avail(IPC_CHANNEL_ARMLOG_ID)) <= 0) {
			if (file->f_flags & O_NONBLOCK)
				return -EAGAIN;

			ipc_enable_interrupt(IPC_CHANNEL_ARMLOG_ID, R_AVAIL_INT);
			wait_event_interruptible(armlog_dev->read_wait,
						(len = ipc_read_avail(IPC_CHANNEL_ARMLOG_ID)) > 0);
		}
		if (len == 0)
			return 0;
		if (count > len)
			count = len;

		ret = ipc_read_to_user(IPC_CHANNEL_ARMLOG_ID, buf, count);
		//printk("%s: ret = %d\n", __func__, ret);

		return ret;
	}
}

static const struct file_operations armlog_fops = {
	.owner 		= THIS_MODULE,
	.read 		= ipc_armlog_read,
	.open 		= ipc_armlog_open,
	.release 	= ipc_armlog_close
};

static int armlog_i2u_thread(void *arg)
{
	int ret, len;
	int timeout = (30*HZ)/1000;

	do {
		if (atomic_read(&armlog_dev->trace_mode) == TEST_MODE) {
			ret = usb_common_dev_open(armlog_dev->com_dev);
			if (ret < 0) {
				printk(ATAG "%s: open usb failed\n", __func__);
			} else {
				armlog_dev->usb_trace_working = 1;
				break;
			}
		}
		msleep(500);
	}while (!kthread_should_stop());
	
	do {
		if (!atomic_read(&armlog_dev->stop_thread)) {
			len = ipc_read_avail(IPC_CHANNEL_ARMLOG_ID);
			if (len > 0) {
				ret = usb_common_dev_write(armlog_dev->com_dev, NULL, len);
				if (ret != len) {
					printk(ATAG "USB buffer is not enough, discard %d Byte\n", len-ret);
				}
			} else {
				ipc_enable_interrupt(IPC_CHANNEL_ARMLOG_ID, R_AVAIL_INT);
				wait_event_interruptible_timeout(armlog_dev->read_wait, 
							ipc_read_avail(IPC_CHANNEL_ARMLOG_ID) > 0, timeout);
			}
		} else {
			armlog_dev->usb_trace_working = 0;
			break;
		}
	} while (!kthread_should_stop());
	
	usb_common_dev_close(armlog_dev->com_dev);
	printk(ATAG "=====%s exit=====\n", __func__);
	
	return 0;
}

static void armlog_work_func(struct work_struct *work)
{
	printk(ATAG "%s\n", __func__);
	
	armlog_dev->armlog_thread = kthread_run(armlog_i2u_thread, 0, "armlog_u2id");
}

static int ipc_armlog_usb_probe(struct common_dev *com)
{
	printk(ATAG "%s\n", __func__);
	armlog_dev->com_dev = com;
	com->channel_id = IPC_CHANNEL_ARMLOG_ID;
	return 0;
}

static int ipc_armlog_usb_remove(struct common_dev *com)
{
	printk(ATAG "%s\n", __func__);
	return 0;
}

static int ipc_armlog_usb_connect(struct common_dev *com)
{
	printk(ATAG "%s\n", __func__);
	
	atomic_set(&armlog_dev->stop_thread, false);
	if (atomic_read(&armlog_dev->trace_mode) == TEST_MODE) {
		queue_work(armlog_dev->armlog_wq, &armlog_dev->armlog_work);
	}
	
	return 0;
}

static int ipc_armlog_usb_disconnect(struct common_dev *com)
{
	printk(ATAG "%s\n", __func__);
	atomic_set(&armlog_dev->stop_thread, true);
	return 0;
}

static struct usb_common_driver ipc_armlog_usb_driver={
	.name 		= "ipc_armlog",
	.probe 		= ipc_armlog_usb_probe,
	.remove 	= ipc_armlog_usb_remove,
	.connect 	= ipc_armlog_usb_connect,
	.disconnect = ipc_armlog_usb_disconnect,
};

static int ipc_armlog_setup_cdev(void)
{
	int ret;
	dev_t devno;

	ret = alloc_chrdev_region(&devno, 0, 1, "ipc_armlog");
	if (ret < 0) {
		pr_err("armlog_dev allocate Major number failed\n");
		return ret;
	}

	armlog_dev->cdev = cdev_alloc();
	armlog_dev->cdev->owner = THIS_MODULE;
	cdev_init(armlog_dev->cdev, &armlog_fops);

	ret = cdev_add(armlog_dev->cdev, devno, 1);
	if (ret < 0) {
		pr_info("armlog cdev register failed !\n\n");
		kfree(armlog_dev->cdev);
		return ret;
	}

	armlog_dev->armlog_class = class_create(THIS_MODULE, "ipc_armlog");
	if (IS_ERR(armlog_dev->armlog_class)) {
		pr_err("create ipc_armlog class failed.\n");
		return -1;
	}

	armlog_dev->dev = device_create(armlog_dev->armlog_class, NULL, devno,
						(void *)armlog_dev, "ipc_armlog");
	if (IS_ERR(armlog_dev->dev)) {
		pr_err("device_create ipc_armlog failed\n");
		return -1;
	}

	return 0;
}

static int ipc_armlog_setup_usb(void)
{
	int ret;

	ret = register_common_driver(&ipc_armlog_usb_driver);
	if (ret < 0) {
		pr_err("register ipc_armlog_usb_driver failed\n");
		return ret;
	}

	return 0;
}

static int create_armlog_channel(void)
{
	int ret = 0;

	printk("create_armlog_channel\n");

	sprintf(armlog_dev->ch.name, IPC_CHANNEL_ARMLOG_NAME);
	armlog_dev->ch.type = IPC_TYPE_STREAM;
	armlog_dev->ch.id = IPC_CHANNEL_ARMLOG_ID;
	armlog_dev->ch.wake_flag = CH_WAKEUP_DISENABLE;
	armlog_dev->ch.c2a_buf_len = CHANNEL_BUF_128KB;

	ret = ipc_create_ch(&armlog_dev->ch);
	if (ret < 0) {
		pr_err("create_armlog_channel failed\n");
		return ret;
	}

	return ret;
}

static int __init ipc_armlog_init(void)
{
	int ret;

	pr_info("%s\n", __func__);

	armlog_dev = kzalloc(sizeof(struct ipc_armlog_dev), GFP_KERNEL);
	if (IS_ERR(armlog_dev)) {
		pr_info(ATAG "kzalloc armlog dev failed\n");
		return -ENOMEM;
	}

	ret = create_armlog_channel();
	if (ret < 0) 
		goto err1;

	ret = ipc_armlog_setup_cdev();
	if (ret < 0) 
		goto err1;

	ret = ipc_armlog_setup_usb();
	if (ret < 0) 
		goto err1;

	device_create_file(armlog_dev->dev, &dev_attr_tracemode);

	mutex_init(&armlog_dev->read_lock);
	init_waitqueue_head(&armlog_dev->read_wait);

	ipc_register_inthandle(IPC_CHANNEL_ARMLOG_ID, NULL, ipc_armlog_int_handle);
	ipc_disable_interrupt(IPC_CHANNEL_ARMLOG_ID, R_AVAIL_INT);
	
	armlog_dev->armlog_wq = create_singlethread_workqueue("armlog");
	INIT_WORK(&armlog_dev->armlog_work, armlog_work_func);
	
	pr_info(ATAG "ipc_armlog driver initialized\n");

	return 0;

err1:
	kfree(armlog_dev);
	return -1;
}

static void __exit ipc_armlog_exit(void)
{
	class_destroy(armlog_dev->armlog_class);
	cdev_del(armlog_dev->cdev);
	kfree(armlog_dev);
}

late_initcall_sync(ipc_armlog_init);
module_exit(ipc_armlog_exit);
MODULE_DESCRIPTION("CP TRACE Driver");
