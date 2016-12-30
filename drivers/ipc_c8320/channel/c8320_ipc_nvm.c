/* Copyright (c) 2013-2014, CYIT. All rights reserved.
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
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/export.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include "../hsic/ipc.h"

#undef TAG
#define TAG	"[C8320_NVM] "

#define NVM_DBG(fmt, args...)\
		do{\
			if (!log_mask) \
				printk(KERN_INFO TAG fmt, ##args);\
		}while(0)

static int log_mask = 0;
module_param_named(dbg_mask, log_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);

#define RX_BUF_LEN (512*1024)
//#define TEST_DATA_SAVE

enum c8320_nvm_type {
	NVM_STATIC = 1,
	NVM_DYNAMIC = 2,
	TD_TABLE = 3,
	GSM_TABLE = 4,
	IMEI = 5,
	GSM_PARAMETERS = 6,
	NVM_AUPAR = 7,
	PROJECTPAR = 8,
	TD_RFTEST_TABLE = 9,
	GSM_RFTEST_TABLE = 10,
	TDD_RFTEST_TABLE = 11,
	FDD_RFTEST_TABLE = 12,
	LTE_TABLE = 13,
	APT_NVM = 14,
	NVM_TYPE_MAX, //add type before this item
};

static char *c8320_nvm_file[NVM_TYPE_MAX]={
	NULL,
	"/custom/c8320/nvm_static.bin",
	"/custom/c8320/nvm_dynamic.bin",
	"/persist/c8320/TDtable.bin",
	"/persist/c8320/GSMtable.bin",
	"/persist/c8320/IMEI.bin",
	"/persist/c8320/gsmparameters.bin",
	"/custom/c8320/nvm_audpara.bin",
	"/custom/c8320/projectpar.bin",
	"/persist/c8320/tdtesttab.bin",
	"/persist/c8320/gsmtesttab.bin",
	"/persist/c8320/tddtesttab.bin",
	"/persist/c8320/fddtesttab.bin",
	"/persist/c8320/ltetable.bin",
	"/persist/c8320/nvm_apt.bin",
};

enum c8320_ack_result {
	ACK_OK = 1,
	ACK_FAIL,
};

enum c8320_pkt_state {
	PKT_ERROR, //pkt data error
	PKT_NEXT, //we have a next pkt
	PKT_END, //this is the end pkt
};

struct c8320_nvm_data_pkt {
	u8 head;   //nvm packet head
	u8 type;   //nvm type
	u16 cur_id;
	u16 total_id;
	u16 datalen;
};

struct c8320_nvm_ack_pkt {
	u8 head;
	u8 type;
	u8 result;
	u8 tail;
};

struct c8320_nvm_dev {
	unsigned char *rx_buf;
	int w_ptr;

	u8 nvm_type;
	u16 pkt_want_id;
	u16 pkt_max_id;
	u16 datalen;

	wait_queue_head_t read_wait;
	wait_queue_head_t write_wait;

	struct task_struct *task;
};


static struct c8320_nvm_dev *c8320_nvm = NULL;

static struct ipc_channel c8320_nvm_ch = {
	.id = IPC_CHANNEL_NVM_ID,
	.type = IPC_TYPE_PACKET,
	.c2a_buf_len = CHANNEL_BUF_128KB,
	.a2c_buf_len = CHANNEL_BUF_1KB,
	.wake_flag = CH_WAKEUP_ENABLE
};

/* return current get packet pointer */
static struct c8320_nvm_data_pkt* c8320_get_nvm_data(void)
{
	int ret = 0, len = 0, rlen = 0;
	int time_out = 150 * HZ/1000;
	struct c8320_nvm_data_pkt *pkt = NULL;

	printk(TAG "%s: start\n", __func__);

	if ((len = ipc_read_avail(IPC_CHANNEL_NVM_ID)) <= 0) {
		ipc_enable_interrupt(IPC_CHANNEL_NVM_ID, R_AVAIL_INT);
		pr_info(TAG "%s: wait nvm channel readable.\n", __func__);
		wait_event_interruptible(c8320_nvm->read_wait,
			(len = ipc_read_avail(IPC_CHANNEL_NVM_ID)) > 0);
	}

	/* if suspend wake up, do not read */
	if (len == 0) {
		printk(TAG "len = %d\n", len);
		return NULL;
	}

	if (len + c8320_nvm->w_ptr > RX_BUF_LEN) {
		pr_err(TAG "rx_buf is not enough now!");
		BUG();
		return NULL;
	}

	printk(TAG "Start read...\n");

	ret = ipc_read(IPC_CHANNEL_NVM_ID, c8320_nvm->rx_buf + c8320_nvm->w_ptr, len);
	if (ret != len) {
		pr_err(TAG "read nvm data failed !");
		BUG();
		return NULL;
	} else {
		pkt = (struct c8320_nvm_data_pkt *)c8320_nvm->rx_buf;
		#ifdef TEST_DATA_SAVE
		int i;
		for (i = 0; i < 13; i++)
			printk("0x%.2x ", (c8320_nvm->rx_buf + c8320_nvm->w_ptr)[i]);
		printk("\n");

		pr_info(TAG "Packet data info, pkt->head=0x%x, pkt->type=%d, "
			   " pkt->cur_id=%d, pkt->total_id=%d, pkt->datalen=%d\n",
				pkt->head, pkt->type, pkt->cur_id, pkt->total_id, pkt->datalen);
		#endif

		c8320_nvm->datalen = pkt->datalen;
		c8320_nvm->w_ptr += len;
	}

	printk(TAG "read remain data...\n");

	/* get remain data */
	rlen = c8320_nvm->datalen - len + sizeof(* pkt) + 1;
	if (c8320_nvm->w_ptr + rlen > RX_BUF_LEN) {
		pr_err(TAG "rx_buf is not enough now!");
		BUG();
		return NULL;
	}
	printk(TAG "remain len = %d\n", rlen);

	while (rlen > 0) {
		if (ipc_read_avail(IPC_CHANNEL_NVM_ID) <= 0) {
			ipc_enable_interrupt(IPC_CHANNEL_NVM_ID, R_AVAIL_INT);
			pr_info(TAG "%s: reveive remain data, wait channel readable.\n", __func__);
			wait_event_interruptible_timeout(c8320_nvm->read_wait,
							ipc_read_avail(IPC_CHANNEL_NVM_ID) > 0, time_out);
		}

		ret = ipc_read(IPC_CHANNEL_NVM_ID, c8320_nvm->rx_buf + c8320_nvm->w_ptr, rlen);
		if (ret < 0) {
			NVM_DBG("%s: ipc read err, ret = %d\n", __func__, ret);
			BUG();
		}
		if (ret == rlen) {
			c8320_nvm->w_ptr += rlen;
			break;
		}
		c8320_nvm->w_ptr += ret;
		rlen -= ret;
		NVM_DBG("%s: get len = %d, remain = %d\n", __func__, ret, rlen);
	}

	return (struct c8320_nvm_data_pkt *)c8320_nvm->rx_buf;
}

static int c8320_send_ack(enum c8320_nvm_type type, enum c8320_ack_result result)
{
	int ret = -1;
	struct c8320_nvm_ack_pkt ack;

	ack.head = 0x7f;
	ack.type = type;
	ack.result = result;
	ack.tail = 0xff;

	printk(TAG "%s \n", __func__);

	if (ipc_write_avail(IPC_CHANNEL_NVM_ID) < sizeof(struct c8320_nvm_ack_pkt))
	{
		ipc_enable_interrupt(IPC_CHANNEL_NVM_ID, W_AVAIL_INT);

		printk(TAG "%s: wait nvm channel writeable\n", __func__);
		wait_event_interruptible(c8320_nvm->write_wait,
			ipc_write_avail(IPC_CHANNEL_NVM_ID) >= sizeof(struct c8320_nvm_ack_pkt));
	}

	ret = ipc_write(IPC_CHANNEL_NVM_ID, &ack, sizeof(struct c8320_nvm_ack_pkt));
	if (ret == sizeof(struct c8320_nvm_ack_pkt)) {
		if (result == ACK_OK) {
			pr_info(TAG "Send ACK_OK!\n");
		} else if (result == ACK_FAIL) {
			pr_info(TAG "Send ACK_FAIL!\n");
		}
	} else {
		pr_err(TAG "Send ack failed!\n");
		BUG();
		return -1;
	}

	c8320_nvm->w_ptr = 0;
	c8320_nvm->pkt_want_id = 1;
	c8320_nvm->pkt_max_id = 0;
	c8320_nvm->nvm_type = 0;
	c8320_nvm->datalen = 0;

	return 0;
}

static enum c8320_pkt_state c8320_parse_data(struct c8320_nvm_data_pkt* pkt)
{
	// 1. check pkt self error
	if (pkt->head != 0x7F ||
		pkt->type == 0 ||
		pkt->type >= NVM_TYPE_MAX ||
		pkt->cur_id > pkt->total_id ||
		pkt->total_id == 0 ||
		*((char *)pkt + sizeof(struct c8320_nvm_data_pkt) + pkt->datalen) != 0xFF)
	{
		pr_err(TAG " pkt data error, pkt->head=0x%x, pkt->type=%d, pkt->cur_id=%d, "
				   " pkt->total_id=%d, pkt->datalen=%d, tail = 0x%x\n",
				pkt->head, pkt->type, pkt->cur_id, pkt->total_id, pkt->datalen,
				*((char *)pkt + sizeof(struct c8320_nvm_data_pkt) + pkt->datalen));
		goto pkt_err;
	}
	pr_info(TAG "Packet data info, pkt->head=0x%x, pkt->type=%d, pkt->cur_id=%d, "
			   " pkt->total_id=%d, pkt->datalen=%d, tail = 0x%x\n",
				pkt->head, pkt->type, pkt->cur_id, pkt->total_id, pkt->datalen,
				*((char *)pkt + sizeof(struct c8320_nvm_data_pkt) + pkt->datalen));

	// 2. save total_id and pkt type in fisrt pkt
	if (c8320_nvm->pkt_want_id == 1)
	{
		c8320_nvm->pkt_max_id = pkt->total_id;
		c8320_nvm->nvm_type = pkt->type;
	}

	// 3. check pkt sequence
	if (pkt->cur_id != c8320_nvm->pkt_want_id)
	{
		pr_err(TAG "pkt sequence error, we want id=%d, but cur_id=%d!",
				c8320_nvm->pkt_want_id, pkt->cur_id);
		goto pkt_err;
	}

	// 4. check nvm type
	if (pkt->type != c8320_nvm->nvm_type)
	{
		pr_err(TAG "nvm type error, current hande nvm type:%d, but get type:%d!",
				c8320_nvm->nvm_type, pkt->type);
		goto pkt_err;
	}

	/* 5. check total id */
	if (pkt->total_id != c8320_nvm->pkt_max_id)
	{
		pr_err(TAG "pkt->total_id=%d error, not eque before value:%d!",
				pkt->total_id, c8320_nvm->pkt_max_id);
		goto pkt_err;
	}

	/* 6. if it is last pkt return END, else pkt_want_id++ */
	if (pkt->cur_id == c8320_nvm->pkt_max_id)
	{
		return PKT_END;
	} else {
		c8320_nvm->pkt_want_id++;
		return PKT_NEXT;
	}

pkt_err:
	c8320_send_ack(pkt->type, ACK_FAIL);
	BUG();

	return PKT_ERROR;
}

static int c8320_save_nvm_data(void)
{
	int i, len;
	int ret = 0;
	char * buf = c8320_nvm->rx_buf;
	struct c8320_nvm_data_pkt * pkt = (struct c8320_nvm_data_pkt *)buf;
	struct file *fp = NULL;
	mm_segment_t fs;

    fs = get_fs();
    set_fs(KERNEL_DS);

	fp = filp_open(c8320_nvm_file[pkt->type], O_RDWR|O_CREAT|O_TRUNC, 777);
	if (IS_ERR(fp))
	{
		pr_err(TAG "%s, open %s failed!\n", __func__, c8320_nvm_file[pkt->type]);
		ret = -1;
		goto err1;
	}

	for (i = 0; i < c8320_nvm->pkt_max_id; i++)
	{
		pkt = (struct c8320_nvm_data_pkt *)buf;

		len = vfs_write(fp, buf+sizeof(struct c8320_nvm_data_pkt), pkt->datalen, &fp->f_pos);
		if (len != pkt->datalen)
		{
			pr_err(TAG "save nv, wirte %s failed!\n", c8320_nvm_file[pkt->type]);
			ret = -1;
			BUG();
		}

		buf = buf + sizeof(struct c8320_nvm_data_pkt) + pkt->datalen + 1;
	}

	ret = vfs_fsync(fp, 0);
	if (ret < 0) {
		printk(TAG "vfs_fsync %s ERR!\n", c8320_nvm_file[pkt->type]);
	} else {
		printk(TAG "Save %s OK!\n", c8320_nvm_file[pkt->type]);
	}

	filp_close(fp, NULL);

err1:
	set_fs(fs);

	return ret;
}

static int c8320_knvmd(void * p)
{
	struct c8320_nvm_data_pkt* pkt = NULL;
	enum c8320_pkt_state stat;

	while (!kthread_should_stop())
	{
		 pkt = c8320_get_nvm_data();
		 if (pkt != NULL)
		 {
			stat = c8320_parse_data(pkt);
			if (stat == PKT_END)
			{
				c8320_save_nvm_data();
				c8320_send_ack(pkt->type, ACK_OK);
			}
		 }
		 printk(TAG "while end\n");
	}

	return 0;
}

static long nvm_ipc_int_handle(int id, void *priv, int event)
{
	if (id != IPC_CHANNEL_NVM_ID)
		return -ENODEV;

	if (event & R_AVAIL) {
		ipc_disable_interrupt(IPC_CHANNEL_NVM_ID, R_AVAIL_INT);
		wake_up_interruptible(&c8320_nvm->read_wait);
	}

	if (event & W_AVAIL) {
		ipc_disable_interrupt(IPC_CHANNEL_NVM_ID, W_AVAIL_INT);
		wake_up_interruptible(&c8320_nvm->write_wait);
	}

	return IRQ_HANDLED;
}

static int __init c8320_nvm_init(void)
{
	int ret;

	pr_info(TAG "nvm driver initializing ...\n");

	c8320_nvm = kzalloc(sizeof(struct c8320_nvm_dev), GFP_KERNEL);
	if (IS_ERR(c8320_nvm)) {
		printk("%s: kzalloc nvm_dev failed.\n", __func__);
        return -ENOMEM;
	}

	c8320_nvm->rx_buf = kzalloc(RX_BUF_LEN, GFP_KERNEL);
	if (IS_ERR(c8320_nvm->rx_buf)) {
		printk("%s: kzalloc rx buf failed.\n", __func__);
		ret = PTR_ERR(c8320_nvm->rx_buf);
        goto err1;
	}

	sprintf(c8320_nvm_ch.name, IPC_CHANNEL_NVM_NAME);
	ret = ipc_create_ch(&c8320_nvm_ch);
	if (ret < 0) {
		printk("ipc_create_ch nvm fialed.\n");
		goto err2;
	} else {
		printk("ipc_create_ch nvm successed.\n");
	}
	ipc_register_inthandle(IPC_CHANNEL_NVM_ID, NULL, nvm_ipc_int_handle);

	init_waitqueue_head(&c8320_nvm->read_wait);
	init_waitqueue_head(&c8320_nvm->write_wait);

	c8320_nvm->w_ptr = 0;
	c8320_nvm->pkt_want_id = 1;
	c8320_nvm->pkt_max_id = 0;
	c8320_nvm->nvm_type = 0;
	c8320_nvm->datalen = 0;

	c8320_nvm->task = kthread_run(c8320_knvmd, 0, "c8320_knvmd");
	if (IS_ERR(c8320_nvm->task)) {
		printk("run c8320_knvmd failed\n");
		ret = PTR_ERR(c8320_nvm->task);
		goto err2;
	}
	pr_info(TAG "nvm driver initialized.\n");
	return 0;

err2:
	kfree(c8320_nvm->rx_buf);
err1:
    kfree(c8320_nvm);
	return ret;
}

static void __exit c8320_nvm_exit(void)
{
	kfree(c8320_nvm->rx_buf);
	kfree(c8320_nvm);
}

late_initcall_sync(c8320_nvm_init);
module_exit(c8320_nvm_exit);
MODULE_DESCRIPTION("C8320 NVM Driver");
