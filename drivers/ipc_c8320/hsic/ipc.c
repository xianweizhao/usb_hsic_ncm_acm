
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <asm/mach/irq.h>
#include <mach/map.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <mach/regs-pmu.h>
#include <mach/regs-mailbox.h>
#include <linux/kfifo.h>
#include "ipc.h"
#include "ipc_acm.h"

#define TAG "[ACM2SMD] "

#define CH_NAME_LEN_MAX (64)
#define CH_NUM_MAX (20)


#define TX_TIMER_MSEC (200)
#define RX_TIMER_MSEC (200)

#define RX_WAKELOCK_TIMEOUT (2)
#define TX_WAKELOCK_TIMEOUT (2)

/* MAX value must >= 1*/
#define RX_TIMER_LOOPCNT_MAX  (5)
#define TX_TIMER_LOOPCNT_MAX  (5)

static int rx_timer_loopcnt ;
static struct timer_list rx_timer;
static void rx_timer_fun(unsigned long param);
static struct wake_lock rx_wakelock;
static struct wake_lock event_wakelock;
//for test
static struct wake_lock g_wakelock;
static int hacm_debug = 1;


typedef enum
{
	TX = 0,
	RX = 1
}xfer_t;


#define FIFO_SIZE (1024)
struct smd_ch
{
	char name[CH_NAME_LEN_MAX];
	unsigned id;
	unsigned type;
	unsigned wake_flag;
	
	void *priv;

	STRUCT_KFIFO_REC_1(FIFO_SIZE) rx_info_fifo;
	
	struct acm_ch *acm_ch;
	void (*notify_acm_driver)(struct acm_ch *ch);

	STRUCT_KFIFO_REC_1(64) acm_event_fifo;
	struct work_struct acm_event_work;
	struct mutex acm_event_mutex;

	
	long (*notify)(int id, void *priv, int event);
	void (*notify_other_cpu)(struct smd_ch *ch,  xfer_t direction);
	int (*read)(struct smd_ch *ch, void *data, unsigned len);
	int (*write)(struct smd_ch *ch, const void *data, unsigned len);
	int (*read_avail)(struct smd_ch *ch);
	int (*write_avail)(struct smd_ch *ch);
	int (*read_to_user)(struct smd_ch *ch, void __user *data, unsigned len);
	int (*write_from_user)(struct smd_ch *ch, const void __user *data, unsigned len);
};



static struct smd_ch * channels[IPC_CHANNEL_ID_MAX] = {NULL};	
static struct smd_ch * ch_priority[IPC_CHANNEL_ID_MAX] = {NULL}; 

static int ch_num = 0;

// sort channel from high priority to low priority
static void pri_ch(struct smd_ch * ch)
{
	int i = 0;
	int count = 0;
		
	if(ch_num == 0)
		ch_priority[0] = ch;
	
	for(i=0; i<ch_num; i++)
	{
		if(ch_priority[i]->id > ch->id)
			break;
	}

	if(i == ch_num)
	{	
		ch_priority[i] = ch;
	}
	else
	{
		count = ch_num;
		for(; count >= i; count --)
		{
			ch_priority[count] = ch_priority[count-1];
		}

		ch_priority[i] = ch;
	}
	
}

static inline void notify_other_cpu(struct smd_ch *ch,  xfer_t direction)
{
		if(ch->notify_acm_driver != NULL)
			ch->notify_acm_driver(ch->acm_ch);
}

static int ch_stream_write(struct smd_ch *ch, const void *data, unsigned len)
{
	int ret = 0;
	
	if (data == NULL || len == 0)
		return 0;

	if(ch->acm_ch->write_avail(ch->acm_ch) < len )
		BUG();

	ret = ch->acm_ch->write(ch->acm_ch, data, len);

	return ret;
}


static int ch_stream_write_from_user(struct smd_ch *ch, const void *data, unsigned len)
{
	int ret = 0;
	char *buf = NULL;
	
	if (data == NULL || len == 0)
		return 0;

	if(ch->acm_ch->write_avail(ch->acm_ch) < len )
		BUG();

	buf = kmalloc(len, GFP_KERNEL);
	if(buf == NULL){
		printk(KERN_ERR TAG " kmalloc error !!!, %s !\n", __func__);
		return 0;
	}
	
	if(copy_from_user(buf, data, len))
		printk(KERN_ERR TAG " copy error !!!, %s !\n", __func__);
	
	ret = ch->acm_ch->write(ch->acm_ch, buf, len);
	kfree(buf);
	
	return ret;
}

static inline int ch_read(struct smd_ch *ch, void *data, unsigned len, int flag)
{
	int count = 0;
	int ret = 0;
	struct acm_rb *rb = NULL;

	
	if (data == NULL || len == 0)
		goto err;

	if (kfifo_is_empty(&ch->rx_info_fifo)){
		printk(KERN_INFO TAG "no data read avail !\n");
		goto err;
	}

	ret = kfifo_out(&ch->rx_info_fifo, &rb, sizeof(rb));
	if(ret != sizeof(rb) || rb == NULL){
		BUG();
	}

	
	if(rb->size <= len){
		count = rb->size;
	}else{
		printk(KERN_ERR TAG "user rx buffer is small, skip some data !\n");
		count = len;
	}

	/*if acm device is removed ,data may be invalid becasue buf kfree already*/
	if(!ch->acm_ch->is_opened(ch->acm_ch)){
		count = 0;
		goto err;
	}

	if(flag == 0){
		memcpy(data, rb->base, count);
	}else if(flag == 1)
	{
		if (copy_to_user(data, rb->base, count))
			printk(KERN_ERR TAG "rx copy to user error !\n");
		
	}

	if(ch->acm_ch->read_done != NULL && rb != NULL)
		ret = ch->acm_ch->read_done(ch->acm_ch, rb);

err:
	ch->notify_other_cpu(ch, TX);

	rx_timer_loopcnt = 1;
	mod_timer(&rx_timer, jiffies + msecs_to_jiffies(RX_TIMER_MSEC));
	wake_lock_timeout(&rx_wakelock, RX_WAKELOCK_TIMEOUT * HZ);

	return count;

}

static int ch_stream_read(struct smd_ch *ch, void *data, unsigned len)
{
	return ch_read(ch, data, len, 0);
}


static int ch_stream_read_to_user(struct smd_ch *ch, void __user *data, unsigned len)
{
	return ch_read(ch, data, len, 1);
}


static int ch_stream_read_avail(struct smd_ch *ch)
{
	int ret = 0;
	struct acm_rb *rb = NULL;

	if(!ch->acm_ch->is_opened(ch->acm_ch))
		return 0;

/*	
	if (kfifo_is_empty(&ch->rx_info_fifo)){
		printk("%s, empty kfifo\n", __func__);
		return 0;
	}
*/

	ret = kfifo_out_peek(&ch->rx_info_fifo, &rb, sizeof(rb));
	if(ret != sizeof(rb) || rb == NULL){
		return 0;
	}

	return rb->size;
}

static int ch_stream_write_avail(struct smd_ch *ch)
{
	if(!ch->acm_ch->is_opened(ch->acm_ch))
		return 0;
	
	return ch->acm_ch->write_avail(ch->acm_ch);
}


static int ch_packet_write(struct smd_ch *ch, const void *data, unsigned len)
{
	return ch_stream_write(ch, data, len);
}


static int ch_packet_write_from_user(struct smd_ch *ch, const void __user *data, unsigned len)
{
	return ch_stream_write_from_user(ch, data, len);
}


static int ch_packet_read(struct smd_ch *ch, void *data, unsigned len)
{
	return ch_stream_read(ch, data, len);
}


static int ch_packet_read_to_user(struct smd_ch *ch, void  __user *data, unsigned len)
{
	return ch_stream_read_to_user(ch, data, len);
}



static int ch_packet_read_avail(struct smd_ch *ch)
{
	return ch_stream_read_avail(ch);
}

static int ch_packet_write_avail(struct smd_ch *ch)
{
	return ch_stream_write_avail(ch);
}


static int acm_rx_cb (void *priv, struct acm_rb *buf)
{

	struct smd_ch *chan = (struct smd_ch *)priv;

	kfifo_in(&chan->rx_info_fifo, &buf, sizeof(buf));

	if(chan->notify != NULL)
		chan->notify(chan->id, chan->priv, R_AVAIL);

	return 0;
}

static void acm_tx_cb(void *priv)
{
	struct smd_ch *chan = (struct smd_ch *)priv;

	
	if(chan->notify != NULL)
		chan->notify(chan->id, chan->priv, W_AVAIL);

}

static int acm_report_stat(void * priv, enum acm_state stat)
{
	struct smd_ch *chan = (struct smd_ch *)priv;

	printk(KERN_ERR TAG "acm report state=%d !\n", stat);

	kfifo_in(&chan->acm_event_fifo, &stat, sizeof(stat));
	
	schedule_work(&chan->acm_event_work);

	return 0;
}

static void handle_acm_event(struct work_struct *work)
{
	int ret = 0;
	enum acm_state stat = ACM_NONE;
	struct smd_ch *chan = container_of(work, struct smd_ch, acm_event_work);

	mutex_lock(&chan->acm_event_mutex);

	while(!kfifo_is_empty(&chan->acm_event_fifo))
	{
		ret = kfifo_out(&chan->acm_event_fifo, &stat, sizeof(stat));
		if(ret != sizeof(stat)){
			BUG();
		}

		switch (stat){
			case ACM_PROBE:

				kfifo_reset_out(&chan->rx_info_fifo);
				
				ret = chan->acm_ch->open(chan->acm_ch);
				if(ret != 0){
					printk(KERN_ERR TAG "open acm error !, %s \n", __func__);
					break ;
				}

				printk(KERN_INFO TAG "hardware connected !\n");
	
				if(chan->notify != NULL)
					chan->notify(chan->id, chan->priv, RW_AVIL);
			
				break;
			
			case ACM_REMOVE:
				printk(KERN_INFO TAG "hardware disconnected ! \n");
				break;
			default:
				printk(KERN_ERR TAG "unknown acm hardware state ! \n");
		}

	}
	
	mutex_unlock(&chan->acm_event_mutex);

}

static int change_id(int id)
{
	switch(id){
		case IPC_CHANNEL_NVM_ID:
				return 0;
		case IPC_CHANNEL_ARMLOG_ID:
		case IPC_CHANNEL_ZSPLOG_ID:
				return 2;
		case IPC_CHANNEL_VM_ID:
			return 1;
		//case IPC_CHANNEL_AT1_ID:
		//	return 0;

		default:
			printk(KERN_ERR TAG "not support this channel ID ,id=%d! \n", id);
			BUG();
	}

	return -1;

}

int hacm_create_ch(struct ipc_channel *ch)
{	
	int ret = -1;
	struct smd_ch *chan;
	
	int name_len = strlen(ch->name) ;

	if(!ch || (ch->type >= IPC_TYPE_MAX))
		return -1;

	if(hacm_find_ch(ch->name) >= 0)
	{
		printk(KERN_ERR TAG"Ch %s create error, alread created !\n", ch->name);
		return -1;
	}

	if(ch_num == CH_NUM_MAX)
		return -1;
		
	if(name_len > CH_NAME_LEN_MAX-1)
		name_len = CH_NAME_LEN_MAX-1;

	chan = kzalloc(sizeof(struct smd_ch), GFP_KERNEL);
	if(chan == NULL)
		return -1;


	chan->acm_ch = acm_register(change_id(ch->id), &chan->notify_acm_driver, acm_rx_cb, acm_tx_cb, acm_report_stat, chan);

	chan->notify_other_cpu = notify_other_cpu;
	
	INIT_WORK(&chan->acm_event_work, handle_acm_event);
	INIT_KFIFO(chan->acm_event_fifo);
	mutex_init(&chan->acm_event_mutex);

	INIT_KFIFO(chan->rx_info_fifo);

	memcpy((void *)chan->name, ch->name, name_len);
	chan->name[name_len] = '\0';
	chan->id = ch->id;
	chan->type = ch->type;
	pri_ch(chan);
	channels[ch->id] = chan;
	ch_num += 1;

	switch(ch->type)
	{
		case IPC_TYPE_STREAM:

			chan->read = ch_stream_read;
			chan->write = ch_stream_write;
			chan->read_avail = ch_stream_read_avail;
			chan->write_avail = ch_stream_write_avail;
			chan->read_to_user = ch_stream_read_to_user;
			chan->write_from_user = ch_stream_write_from_user;
			ret = ch->id;
			break;
			
		case IPC_TYPE_PACKET:	
			chan->read = ch_packet_read;
			chan->write = ch_packet_write;
			chan->read_avail = ch_packet_read_avail;
			chan->write_avail = ch_packet_write_avail;
			chan->read_to_user = ch_packet_read_to_user;
			chan->write_from_user = ch_packet_write_from_user;
			
			ret = ch->id;
			break;
			
		default:
			break;

	}

	if (chan->acm_ch->is_connected(chan->acm_ch)){
		chan->acm_ch->open(chan->acm_ch);
	}	
	
	return ret;
}



int hacm_read_avail(int id)
{
	struct smd_ch *ch = channels[id];

	return ch->read_avail(ch);
}

int hacm_write_avail(int id)
{
	struct smd_ch *ch = channels[id];

	return ch->write_avail(ch);
}

int hacm_read(int id, void *data, unsigned len)
{

	struct smd_ch *ch = channels[id];

	return ch->read(ch, data, len);
}

int hacm_write(int id, const void *data, unsigned len)
{
	struct smd_ch *ch = channels[id];

	return ch->write(ch, data, len);
}

int hacm_read_to_user(int id, void *data, unsigned len)
{
	struct smd_ch *ch = channels[id];

	return ch->read_to_user(ch, data, len);
}

int hacm_write_from_user(int id, const void *data, unsigned len)
{
	struct smd_ch *ch = channels[id];

	return ch->write_from_user(ch, data, len);
}

int hacm_register_inthandle(int id, void *priv, long (*handle)(int id, void *priv, int event))
{
	int ret = -1;
	
	struct smd_ch *ch = channels[id];
	if(ch)
	{	
		ch->priv = priv;
		ch->notify = handle;
		ret = 0;
	}
	
	return ret;
}

int hacm_enable_interrupt(int id, unsigned flag)
{
	return 0;
}


int hacm_disable_interrupt(int id, unsigned flag)
{
       return 0;
}

int hacm_find_ch(char *name)
{
	struct smd_ch * ch;
	int ret = -1;
	int i = 0;
	
	for(i=0; i < ch_num; i++)
	{
		if(ch_priority[i] == NULL)
		{
			continue;
		}

		ch = ch_priority[i];
		if(!strcmp((const char *)ch->name, name))
		{
			ret = ch->id;
			break;
		}
	}

	return ret;
}



int hacm_reset_ch(int id)
{
	struct smd_ch *ch = channels[id];

	if(ch == NULL){
		printk(KERN_ERR TAG "ch reset error!  id=%d\n", id);
		return -1;
	}

	kfifo_reset(&ch->rx_info_fifo);
	
	return 0;

}

int hacm_reset(void)
{
	int i;

	for(i=0; i < ch_num; i++)
	{	
		hacm_reset_ch(i);
	}

	return 0;
}

int __init hacm_init(void)
{
	wake_lock_init(&rx_wakelock, WAKE_LOCK_SUSPEND, "hacm_rx_wakelock");
	wake_lock_init(&event_wakelock, WAKE_LOCK_SUSPEND, "hacm_event_wakelock");

	//for development test , only. fixme, remove it when need not
	wake_lock_init(&g_wakelock, WAKE_LOCK_SUSPEND, "hacm_hold_wakelock");
	wake_lock(&g_wakelock);

	setup_timer(&rx_timer, rx_timer_fun, 0);

	printk(KERN_INFO TAG"initial hacm driver success !\n");

	return 0;
}



static void rx_timer_fun(unsigned long param)
{
	struct smd_ch *ch;
	int i = 0;

	//if(smd_debug == 1)
	//	printk(KERN_INFO TAG" %s\n", __func__);
	
	for(i=0; i < ch_num; i++)
	{	
		ch = ch_priority[i];

		if(kfifo_len(&ch->rx_info_fifo) > 0)
		{
			if(ch->notify != NULL)
				ch->notify(ch->id, ch->priv, R_AVAIL);

			if(rx_timer_loopcnt < RX_TIMER_LOOPCNT_MAX)
			{
				rx_timer_loopcnt++;
				mod_timer(&rx_timer, jiffies + msecs_to_jiffies(RX_TIMER_MSEC));
			}

			if(hacm_debug == 1)
				printk(KERN_INFO TAG"%s rx data timeout, rx_timer_loopcnt=%d \n", ch->name, rx_timer_loopcnt);
		}
	}
}


static struct platform_device hacm_device = {
	.name		= "hacm",
	.id		= 0,
};

static int __init hacm_device_init(void)
{
	platform_device_register(&hacm_device);
	return 0;
}

device_initcall(hacm_device_init);


static ssize_t hacm_debug_show(struct device *pdev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d", hacm_debug);
}

static ssize_t hacm_debug_store(struct device *pdev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%d", &hacm_debug);
	return count;
}

static DEVICE_ATTR(debug, S_IRWXUGO, hacm_debug_show, hacm_debug_store);


static int __init hacm_probe(struct platform_device *pdev)
{

	device_create_file(&pdev->dev, &dev_attr_debug);
	return hacm_init();
}

static int __exit hacm_remove(struct platform_device *pdev)
{
	return 0;
}

static int hacm_suspend(struct platform_device *pdev, pm_message_t state)
{
	if(hacm_debug == 1)
	{
		struct smd_ch *ch;
		int i = 0;
	
		for(i=0; i < ch_num; i++)
		{	
			ch = ch_priority[i];
			if(kfifo_len(&ch->rx_info_fifo) > 0)
			{
				printk(KERN_ERR TAG"smd suspend error!, ch %s is read avail\n", ch->name);
				BUG();
				return -1;
			}			
		}
	}

	return 0;
}

static int hacm_resume(struct platform_device * pdev)
{
	return 0;
}


static struct platform_driver hacm_driver = {
	.driver = {
		.name = "hacm",
		.owner = THIS_MODULE,
	},
	.suspend = hacm_suspend,
	.resume = hacm_resume,
	.probe = hacm_probe,
	.remove = __exit_p(hacm_remove),
};

static int __init hacm_init_module(void)
{

	return platform_driver_register(&hacm_driver);
}


static void __exit hacm_cleanup_module(void)
{
	platform_driver_unregister(&hacm_driver);
}



module_init(hacm_init_module);
module_exit( hacm_cleanup_module);
EXPORT_SYMBOL(hacm_create_ch);
EXPORT_SYMBOL(hacm_read_avail);
EXPORT_SYMBOL(hacm_write_avail);
EXPORT_SYMBOL(hacm_read);
EXPORT_SYMBOL(hacm_write);
EXPORT_SYMBOL(hacm_read_to_user);
EXPORT_SYMBOL(hacm_write_from_user);
EXPORT_SYMBOL(hacm_register_inthandle);
EXPORT_SYMBOL(hacm_enable_interrupt);
EXPORT_SYMBOL(hacm_disable_interrupt);
EXPORT_SYMBOL(hacm_reset);
EXPORT_SYMBOL(hacm_reset_ch);


MODULE_DESCRIPTION("CYIT smd  Core");
MODULE_AUTHOR("xu xiaoqing <xuxiaoqing@cqcyit.com>");
MODULE_LICENSE("GPL");

