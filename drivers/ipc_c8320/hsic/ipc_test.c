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
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/sched.h> 
#include <linux/fs.h>
#include <linux/slab.h>
//#include <mach/ipc.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#include "ipc.h"


#define TAG "[SMD_TEST_MODULE] "

#define SMD_IOC_MAGIC 'S'
#define SMD_FIND_CH _IOWR(SMD_IOC_MAGIC, 1, struct ipc_channel)
#define SMD_CREAE_CH _IOWR(SMD_IOC_MAGIC, 2, struct ipc_channel)
#define SMD_RESET_CH _IOWR(SMD_IOC_MAGIC, 3, struct ipc_channel)

struct ch_ctx{
	struct list_head list;
	struct ipc_channel ch; 
	wait_queue_head_t read_wait; 
	wait_queue_head_t write_wait;
};

struct smd_dev {
	dev_t devno;
	struct class *smd_class;
	struct cdev cdev; 
	//struct mutex smd_mutex;
}; 

static struct smd_dev *smddev;


//static LIST_HEAD(chs);

static int smd_test_open(struct inode *inode, struct file *file)
{
	struct ch_ctx *ctx;

	ctx = kzalloc(sizeof(struct ch_ctx), GFP_KERNEL);
	if (!ctx) {
		printk(KERN_ERR TAG"not enough memory for ctx\n");
		return -ENOMEM;
	}
	
	file->private_data = (void *)ctx;
	
	//list_add_tail(&ctx->list, &chs);
	init_waitqueue_head(&ctx->read_wait);
	init_waitqueue_head(&ctx->write_wait);
	
	return 0; 	
}

static int smd_test_close(struct inode *inode, struct file *file)
{
	struct ch_ctx *ctx = (struct ch_ctx *)file->private_data;
	
	if(ctx != NULL){
		kfree(ctx);
	}
		
	return 0;
}

static  long smd_test_int_handle(int id, void *priv, int event)
 {
	struct ch_ctx *ctx = (struct ch_ctx *)priv;
 	
	//list_for_each_entry(ctx, &chs, list) {
	//	if(id == ctx->ch.id)
	//	{
			if((event&R_AVAIL) != 0)
			{
				//smd_disable_interrupt(id, R_AVAIL_INT);
				wake_up_interruptible(&ctx->read_wait);
			}else if((event&W_AVAIL) != 0)
			{
				//smd_disable_interrupt(id,W_AVAIL_INT);
				wake_up_interruptible(&ctx->write_wait);
			}
			
	//	}
	//}

	return 0;
 }

static ssize_t smd_test_write (struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	int cnt = 0;
	int len = 0;
	int ret = 0;
	//int left = count;
	struct ch_ctx *ctx;

	ctx = filp->private_data;


	if(ctx->ch.type == IPC_TYPE_PACKET)
	{
		while(ipc_write_avail(ctx->ch.id) < count)
		{
			ipc_enable_interrupt(ctx->ch.id, W_AVAIL_INT);
			ret = wait_event_interruptible(ctx->write_wait, ipc_write_avail(ctx->ch.id) >= count);
			if(ret == -ERESTARTSYS)
				return 0;
			ipc_disable_interrupt(ctx->ch.id, W_AVAIL_INT);
		}

		cnt = ipc_write_from_user(ctx->ch.id, buf, count);
		if(cnt != count)
			printk(TAG"packet data wirte error ! \n");
	}

	if(ctx->ch.type == IPC_TYPE_STREAM)
	{
#if 0
		while(left > 0)
		{
			len = ipc_write_avail(ctx->ch.id);

			if(len > 0){
				len = left>len ? len:left;
				left -= ipc_write_from_user(ctx->ch.id, buf, len);
			}else{
				ipc_enable_interrupt(ctx->ch.id, W_AVAIL_INT);
				wait_event(smddev->write_wait, smd_write_avail(ctx->ch.id)>0);
				ipc_disable_interrupt(ctx->ch.id, W_AVAIL_INT);
			}	
			
		} // end while

#endif

		while((len = ipc_write_avail(ctx->ch.id)) <= 0)
		{
			ipc_enable_interrupt(ctx->ch.id, W_AVAIL_INT);
			ret = wait_event_interruptible(ctx->write_wait, ipc_write_avail(ctx->ch.id) >= count);
			if(ret == -ERESTARTSYS)
				return 0;
			ipc_disable_interrupt(ctx->ch.id, W_AVAIL_INT);
		}

		len = len>count ? count:len;

		cnt = ipc_write_from_user(ctx->ch.id, buf, len);
		if(cnt != len)
			printk(TAG"stream data wirte error ! \n");
	}

	return cnt;
}

static int smd_test_read(struct file *file, char __user *buf, size_t count,
			  loff_t *ppos)
{
	int cnt = 0;
	int len = 0;
	int ret = 0;
	struct ch_ctx *ctx;

	ctx = file->private_data;

	//while( (len = smd_read_avail(ctx->ch.id)) <= 0)
	//{
	//	smd_enable_interrupt(ctx->ch.id, R_AVAIL_INT);
	//	wait_event_interruptible(ctx->read_wait, smd_read_avail(ctx->ch.id) > 0);
	//	if(ret == -ERESTARTSYS)
	//			return 0;
	//}

	//smd_disable_interrupt(ctx->ch.id, R_AVAIL_INT);


	if((len = ipc_read_avail(ctx->ch.id)) <= 0)
		return 0;

	if(ctx->ch.type == IPC_TYPE_PACKET)
	{
		if(len > count){
			printk(TAG"buffer not enough to hold packet data !\n");
			return 0;
		}

		cnt = ipc_read_to_user(ctx->ch.id, buf, len);
		if(cnt != len)
			printk(TAG"read packet data error , packet not integrity !\n");
	}

	if(ctx->ch.type == IPC_TYPE_STREAM)
	{
		len = len>count ? count:len;
		cnt = ipc_read_to_user(ctx->ch.id, buf, len);
	}

	if(ctx->ch.type == IPC_TYPE_EVENT)
	{
		cnt = ipc_read_to_user(ctx->ch.id, buf, len);
	}


	return cnt;
}

long smd_test_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct ch_ctx *ctx;
	int size = sizeof(struct ipc_channel);
	
	ctx = filp->private_data;

	copy_from_user(&ctx->ch, (void __user *)arg, size);

	switch (cmd) {
		case SMD_FIND_CH:
			 if(ipc_find_ch(ctx->ch.name) > 0){
			 	printk(KERN_ERR TAG"ch [%s] alreadly created before !\n", ctx->ch.name);
			 }else
			 {
			 	err =  -EFAULT;
				printk(KERN_ERR TAG"ch [%s] NOT creat before !\n", ctx->ch.name);
			 }

			break;
   		case SMD_CREAE_CH:
			 if (ipc_create_ch(&ctx->ch) == -1){
			 	printk(KERN_ERR TAG"ch [%s] create ch error !\n", ctx->ch.name);
				err =  -EFAULT;
			 }else{
			 	ipc_register_inthandle(ctx->ch.id, ctx, smd_test_int_handle);
				printk(KERN_ERR TAG"ch [%s] create success !\n", ctx->ch.name);
			 }

			 break;
		case SMD_RESET_CH:		 
			 if(ipc_reset_ch(ctx->ch.id) >= 0){
			 	ipc_register_inthandle(ctx->ch.id, ctx, smd_test_int_handle);
			 	printk(KERN_ERR TAG"ch [%s] reset success!\n", ctx->ch.name);
			 }else{
				printk(KERN_ERR TAG"ch [%s] reset error!\n", ctx->ch.name);
				err =  -EFAULT;
			 }

			break;		
    	default:
        	printk(KERN_ERR TAG"unknown ioctl cmd value!\n");
        	err =  -EFAULT;
    }
	
	return err;
}


static struct file_operations smddevfops = {
	.owner = THIS_MODULE,
	.read = smd_test_read,
	.open = smd_test_open,
	.write = smd_test_write,
	.release = smd_test_close,
	.unlocked_ioctl = smd_test_ioctl,
};

static int smdtest_setup_cdev(void)
{
	int err;

    cdev_init(&smddev->cdev, &smddevfops);
	
	smddev->cdev.owner = THIS_MODULE;
	
	err = cdev_add(&smddev->cdev, smddev->devno, 1);
	printk(TAG" device MAJOR : %d \n", MAJOR(smddev->devno));
	if (err) {
		printk(KERN_INFO TAG"cdev registration failed !\n\n");
		return -1;
	}

	return 0;
}

static int __init smdtest_init(void)
{
	int err;
	pr_debug(TAG"smdtest driver initializing ...\n");

	smddev = kzalloc(sizeof(struct smd_dev), GFP_KERNEL);
	if (!smddev)
	{
		printk(KERN_INFO TAG"smd driver alloc fail !\n");
		return -1;
	}

	//smddev->devno = MKDEV(250, 2);

	err = alloc_chrdev_region(&smddev->devno,0,1, "smdtest");
	if (err) {
		printk(KERN_INFO TAG"Major number not allocated\n");
		goto alloc_chrdev_fail;
	} 
	
	if(smdtest_setup_cdev() != 0)
		goto setup_cdev_fail;

	smddev->smd_class = class_create(THIS_MODULE, "smdtestclass");
      device_create(smddev->smd_class, NULL, smddev->devno,NULL, "smdtest") ;	

	printk(KERN_INFO TAG"smd driver initialized\n");
	return 0;

setup_cdev_fail:
	unregister_chrdev_region(smddev->devno,1);

alloc_chrdev_fail:
	kfree(smddev);
	printk(KERN_INFO TAG"smd driver initialized faild!\n");

	return -1;

}

static void __exit smdtest_exit(void)
{
	printk(KERN_INFO TAG"smdtest exiting ..\n");

	device_destroy(smddev->smd_class, smddev->cdev.dev);
	class_destroy(smddev->smd_class);
	
	cdev_del(&smddev->cdev);
	unregister_chrdev_region(smddev->devno,1);
	kfree(smddev);
}

module_init(smdtest_init);
module_exit(smdtest_exit);

MODULE_AUTHOR("xuxiaoqing@cqcyit.com");
MODULE_DESCRIPTION("smdtest Char Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
