/* c8320_voicememo.c
 *
 * Voice Memo device
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/export.h>
#include <asm/atomic.h>
#include <asm/ioctls.h>
#include "../hsic/ipc.h"

static struct ipc_vm_dev{
	int ch_stats;
	struct mutex lock;
	wait_queue_head_t r_wait;

	struct common_dev *vm_dev;
	struct work_struct vm_rwork;
	struct workqueue_struct *vm_data_rwq;

	char *buf_r;
}ipc_vm;

static struct ipc_vm_dev *vm = &ipc_vm;



static int audio_vm_open(struct inode *inode, struct file *file)
{
	printk("%s\n",__func__);
	return 0;
}

static int audio_vm_release(struct inode *inode, struct file *file)
{
	printk("%s\n",__func__);
	return 0;
}

static ssize_t audio_vm_read(struct file *file,
				char __user *buf,
				size_t count, loff_t *pos)
{
	int len = 0;
	int ret = 0;

	if ((len = ipc_read_avail(IPC_CHANNEL_VM_ID)) <= 0)
	{
		ipc_enable_interrupt(IPC_CHANNEL_VM_ID, R_AVAIL_INT);
		ret = wait_event_timeout(vm->r_wait, ipc_read_avail(IPC_CHANNEL_VM_ID) > 0, msecs_to_jiffies(20));
		if (ret == 0) {
			printk("audio_voicememo_read read timeout!\n");
			return 0;
		} else if (ret < 0){
			printk("audio_voicememo_read read error!\n");
			return -EIO;
		}else {
			ret = ipc_read_to_user(IPC_CHANNEL_VM_ID, buf, len);
			return ret;
		}

	}else {
		ret = ipc_read_to_user(IPC_CHANNEL_VM_ID, buf, len);
		return ret;
	}

}


static long vm_int_handler(int id, void *priv, int event)
{
	struct ipc_vm_dev *ctx = (struct ipc_vm_dev *)priv;

	if (id != IPC_CHANNEL_VM_ID)
		return IRQ_HANDLED;

	if (event & R_AVAIL) {
		ipc_disable_interrupt(IPC_CHANNEL_VM_ID, R_AVAIL_INT);
		wake_up(&ctx->r_wait);
	}

	return IRQ_HANDLED;
}


static struct file_operations vm_fops = {
	.owner 		= THIS_MODULE,
	.open  		= audio_vm_open,
	.release 	= audio_vm_release,
	.read  		= audio_vm_read,
};


static struct miscdevice vm_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "c8320_voicememo",
	.fops	= &vm_fops,
};

static int __init audio_voicememo_init(void)
{
	int ret = 0;
	struct ipc_channel ch;

	printk("%s\n", __func__);

	sprintf(ch.name, IPC_CHANNEL_VM_NAME);
	//ch.name = IPC_CHANNEL_VM_NAME;
	ch.type = IPC_TYPE_STREAM;
	ch.id = IPC_CHANNEL_VM_ID;
	ch.wake_flag = CH_WAKEUP_ENABLE;
	ch.c2a_buf_len = CHANNEL_BUF_128KB;
	ch.a2c_buf_len = CHANNEL_BUF_128KB;

	ret = ipc_create_ch(&ch);
	if (ret < 0) {
		printk("smd_create_ch vm fialed\n");
		return 0;
	} else {
		printk("smd_create_ch vm successful\n");
	}

	mutex_init(&vm->lock);
	//init_waitqueue_head(&vm->w_wait);
	init_waitqueue_head(&vm->r_wait);

	ipc_register_inthandle(IPC_CHANNEL_VM_ID, &ipc_vm, vm_int_handler);
	ipc_disable_interrupt(IPC_CHANNEL_VM_ID, R_AVAIL_INT);

	ret = misc_register(&vm_dev);

	if (ret) {
			printk(KERN_ERR "ipc_vm: can't misc_register\n");
			return ret;
	}

  return ret;
}

late_initcall_sync(audio_voicememo_init);

MODULE_DESCRIPTION("C6320 Voice Memo driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("CYIT");


