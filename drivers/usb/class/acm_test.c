#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/workqueue.h>
#include <linux/vmalloc.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include "../../ipc_c8320/hsic/ipc_acm.h"

struct smd_channel {
	unsigned n;

	struct list_head ch_list;

	void *priv;
	void (*notify_low_layer)(struct acm_ch *ch);
	struct work_struct read_work;	

	
	unsigned type;

	char name[32];
	struct acm_ch *ch;
};
int static opend = 0;
static char* read_buf = NULL;
static struct timer_list ap_timer;

static void ap_work(struct work_struct *work);
static struct smd_channel *smd_ch = NULL;

static int smd_rx_data_cb (void *priv, struct acm_rb *buf)
{
	struct smd_channel *ch = (struct smd_channel*)priv;
	printk("in function: %s\n",__func__);
	printk("recieve is %s\n",buf->base);
	ch->ch->read_done(ch->ch,buf);

	return 0;
}
static void smd_write_complete_cb(void *priv)
{
	struct smd_channel *ch = (struct smd_channel*)priv;

	printk("in function: %s\n",__func__);
}
static int notify_callback(void *priv, enum acm_state type)
{
	struct smd_channel *ch = (struct smd_channel*)priv;
	switch(type){
	case ACM_PROBE:
		ap_timer.expires = jiffies + msecs_to_jiffies(2 * HZ);
		add_timer(&ap_timer);
		break;
	default:
		break;
	}
}
static int smd_alloc_channel(uint32_t cid)
{
	struct smd_channel *ch;
	
	ch = kzalloc(sizeof(struct smd_channel), GFP_KERNEL);
	if (ch == 0) {
		pr_err("smd_alloc_channel() out of memory\n");
		return -1;
	}
	smd_ch = ch;
	ch->n = cid;

	ch->ch = acm_register(cid, &ch->notify_low_layer, smd_rx_data_cb, smd_write_complete_cb, notify_callback, ch);
	if(!ch->ch){
		kfree(ch);
		return -1;
	}

	INIT_WORK(&ch->read_work, ap_work);
	return 0;
}
static void ap_work(struct work_struct *work)
{
	int ret = 0;
	if (!opend){	
		ret = smd_ch->ch->open(smd_ch->ch);
	}
	if (ret){	
		ap_timer.expires = jiffies + 5 * HZ;
		mod_timer(&ap_timer, msecs_to_jiffies(jiffies + 5 * HZ));
		printk("open failed------\n");
	}
	else{
		if (!opend){	
			printk("open ok------\n");
		}
		printk("in function: %s\n",__func__);
		smd_ch->ch->write(smd_ch->ch,"AT",2);
		opend = 1;
		mod_timer(&ap_timer, jiffies + msecs_to_jiffies(1 * HZ));
	}
}

static void
ap_timeout(unsigned long ptr)
{
	printk("= %s =\n",__func__);
	schedule_work(&smd_ch->read_work);
}

static __init int main(void)
{
	int ret = 0;
	printk("-----failed\n");
	read_buf = kzalloc(1024,GFP_KERNEL);
	if (read_buf = NULL){
		printk("malloc is failed\n");
	}
	ret = smd_alloc_channel(0);
	if (ret){
		printk("-----failed\n");
	}
	init_timer(&ap_timer);
	ap_timer.function = ap_timeout;
	ap_timer.data = NULL;
	return ret;
}
late_initcall(main);


