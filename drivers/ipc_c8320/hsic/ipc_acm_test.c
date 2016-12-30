

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
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#include <linux/export.h>
#include <linux/kthread.h>
#include "ipc.h"


#define TAG "[IPC_ACM2SMD_TEST] "
#define LOG

#ifdef LOG
	#define print_log(fmt, args...) do{\
				printk(KERN_INFO TAG fmt, ##args);\
			}while(0)
#else
	#define print_log(fmt, args...) 
#endif

#define RX_BUF_LEN (512)
#define TX_BUF_LEN (512)

struct ch_dev {
	char rx_buf[RX_BUF_LEN];
	char tx_buf[TX_BUF_LEN];
	
	struct ipc_channel ch;

	wait_queue_head_t w_wait;
	wait_queue_head_t r_wait;

	struct task_struct	* rx_task;
	struct task_struct	* tx_task;
};

static struct ch_dev *devch = NULL;
static int g_tx_len = 0;

#define SEND_STRING "AT"



static int rx_task_fun(void *arg)
{
	int len;
	
	do{
		if ((len = ipc_read_avail(devch->ch.id)) <= 0) {

			ipc_enable_interrupt(devch->ch.id, R_AVAIL_INT);
			//print_log("wait for rx!\n");
			wait_event_interruptible(devch->r_wait, (len = ipc_read_avail(devch->ch.id)) > 0);
		}

		if(len <= 0){
			print_log("rx len = 0,exit !\n");
			return -1;
		}
	
		if(len > RX_BUF_LEN){
			print_log("warning, rx len out of buf len !\n");
			len = RX_BUF_LEN;
		}

		ipc_read(devch->ch.id, devch->rx_buf, len);
		devch->rx_buf[len] = '\0';
	
		print_log("rx: %s, len = %d\n", devch->rx_buf, len);

	} while (!kthread_should_stop());

	return 0;
}


static int tx_task_fun(void * arg)
{
	int len = 0;

	do{
		if ((len = ipc_write_avail(devch->ch.id)) <= 0)
		{
			ipc_enable_interrupt(devch->ch.id, W_AVAIL_INT);
			//print_log("wait for tx!\n");
			wait_event_interruptible(devch->w_wait, (len = ipc_write_avail(devch->ch.id)) > 0);
		}

		if(len <= 0){
			print_log("tx len = 0,exit !\n");
			return -1;
		}

		ipc_write(devch->ch.id, devch->tx_buf, g_tx_len);

		print_log("tx: %s, len = %d\n", devch->tx_buf, g_tx_len);

		msleep(1);
		
	} while (!kthread_should_stop());

	return 0;
}

static long devch_int_handler(int id, void *priv, int event)
{
	struct ch_dev *devch = (struct ch_dev *)priv;

	if (event & R_AVAIL) {
		ipc_disable_interrupt(devch->ch.id, R_AVAIL_INT);
		wake_up_interruptible(&devch->r_wait);
	}

	if (event & W_AVAIL) {
		ipc_disable_interrupt(devch->ch.id, W_AVAIL_INT);
		wake_up_interruptible(&devch->w_wait);
	}

	return IRQ_HANDLED;
}


static int __init ipc_test_init(void)
{
	devch = kzalloc(sizeof(struct ch_dev), GFP_KERNEL);

	init_waitqueue_head(&devch->w_wait);
	init_waitqueue_head(&devch->r_wait);

	memcpy(devch->ch.name, IPC_CHANNEL_AT1_NAME, sizeof(IPC_CHANNEL_AT1_NAME));
	devch->ch.id = IPC_CHANNEL_AT1_ID;
	devch->ch.type = IPC_TYPE_STREAM;
	devch->ch.c2a_buf_len = CHANNEL_BUF_128KB;
	devch->ch.a2c_buf_len = CHANNEL_BUF_128KB;
	devch->ch.wake_flag = CH_WAKEUP_ENABLE;

	ipc_create_ch(&devch->ch);

	ipc_register_inthandle(devch->ch.id, devch, devch_int_handler);

	g_tx_len = strlen(SEND_STRING);
	memcpy(devch->tx_buf, SEND_STRING, g_tx_len);


	devch->rx_task = kthread_run(rx_task_fun, devch, "kacm_rx_task");
	devch->tx_task = kthread_run(tx_task_fun, devch, "kacm_tx_task");

	return 0;
}

static void __exit ipc_test_exit( void )
{
	return ;
}

late_initcall_sync(ipc_test_init);



