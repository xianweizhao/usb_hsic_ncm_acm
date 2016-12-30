/* /linux/drivers/new_modem_if/link_dev_usb.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <linux/pm_runtime.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>
#include <linux/version.h>
#include <linux/rtc.h>
#include <linux/tcp.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_link_device_hsic_ncm.h"

#include "link_usb_cdc_ncm.h"

/* /sys/module/modem_link_device_hsic_ncm/parameters/... */
static int tx_qlen = 10;
module_param(tx_qlen, int, S_IRUGO);
MODULE_PARM_DESC(tx_qlen, "tx qlen");

enum bit_debug_flags {
	LINK_DEBUG_LOG_IPC_TX,
	LINK_DEBUG_LOG_IPC_RX,
	LINK_DEBUG_LOG_RFS_TX,
	LINK_DEBUG_LOG_RFS_RX,
	LINK_DEBUG_LOG_RAW_TX,
	LINK_DEBUG_LOG_RAW_RX,
	LINK_DEBUG_LOG_NCM_TX,
	LINK_DEBUG_LOG_NCM_RX,

	LINK_DEBUG_RECOVER_CPDUMP,
	LINK_DEBUG_RECOVER_RESET,
	LINK_DEBUG_RECOVER_PANIC,
};
#define MAX_TX_ERROR 30
#define MAX_RX_ERROR 30
static char last_link_err[64];
static unsigned long dflags = (
	1 << LINK_DEBUG_LOG_IPC_TX | 1 << LINK_DEBUG_LOG_IPC_RX |
	1 << LINK_DEBUG_RECOVER_RESET);
module_param(dflags, ulong, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(dflags, "link device debug flags");

#define get_hostwake(p) \
	(gpio_get_value((p)->gpio_link_hostwake) == HOSTWAKE_TRIGLEVEL)

#define get_hostactive(p)	(gpio_get_value((p)->gpio_link_active))

static struct modem_ctl *if_usb_get_modemctl(struct link_pm_data *pm_data);
#ifdef CONFIG_PM_RUNTIME
static int link_pm_runtime_get_active_async(struct link_pm_data *pm_data);
static int usb_get_rpm_status(struct device *dev)
{
	return dev->power.runtime_status;
}
#else
static inline int link_pm_runtime_get_active_async(struct link_pm_data *pm_data)
								{ return 0; }
static inline int usb_get_rpm_status(struct device *dev) { return 0; }
#endif
static void usb_rx_complete(struct urb *urb);
static void link_pm_change_modem_state(struct usb_link_device *usb_ld,
						enum modem_state state);
static int usb_rx_submit(struct usb_link_device *usb_ld,
		struct if_usb_devdata *pipe_data, gfp_t gfp_flags);

struct timer_list s3c_dump_timer;
static void sc3_dump_func(unsigned long data)
{
	debug_ehci_reg_dump();
	s3c_dump_timer.expires = jiffies +HZ/100;
	if (!timer_pending(&s3c_dump_timer))
		add_timer(&s3c_dump_timer);
	else
		mod_timer(&s3c_dump_timer, jiffies +HZ/100);
	
}

static void mif_net_suspend(struct if_usb_devdata *pipe_data, int flag)
{
	if (!pipe_data->ndev || !flag)
		return;
	pipe_data->net_suspend |= flag;
	netif_stop_queue(pipe_data->ndev);
	mif_debug("stop 0x%04x, mask=0x%04x\n", flag, pipe_data->net_suspend);
	return;
}

static void mif_net_resume(struct if_usb_devdata *pipe_data, int flag)
{
	if (!pipe_data->ndev || !flag)
		return;
	pipe_data->net_suspend &= ~(flag);
	if (!pipe_data->net_suspend) {
		netif_wake_queue(pipe_data->ndev);
		mif_debug("wake 0x%04x\n", flag);
	} else {
		mif_debug("net 0x%04x, mask=0x%04x\n", flag,
							pipe_data->net_suspend);
	}
	return;
}

static struct if_usb_devdata *get_devdata_with_iod(
	struct usb_link_device *usb_ld, struct io_device *iod)
{

	return &usb_ld->devdata[iod->id];
	
	
}
/* this function do what???  cool_zhao*/
static void enable_c8320_dm(struct link_device *ld, bool enable)
{
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct if_usb_devdata *pipe_data = &usb_ld->devdata[IPC_RAW];

	pipe_data->iod = (enable) ? NULL
			: NULL;

	mif_info("enable DM iod->name(%s)\n", pipe_data->iod->name);
}
/* how to use this fmt iodevs */

static void usb_async_cmd_callback(struct urb *urb)
{
        struct usb_ctrlrequest *req = (struct usb_ctrlrequest *)urb->context;
        int status = urb->status;

        if (status < 0){
                mif_err("async callback failed with %d\n", status);
        }else{
                printk("---------xmit ok\n");
        }

        kfree(req);
        usb_free_urb(urb);
}
/* usb1 net start need send class request for MCCI DRIVER cool_zhao */
static void usbnet_set_multicast(struct link_device *ld, struct io_device *iod, int can_sleep)
{
        struct usb_ctrlrequest *req;
        int status;
	int rpm_state = 0;
	struct device *dev = NULL;
        struct urb *urb;       
	struct if_usb_devdata *pipe_data = NULL;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct net_device *netdev = iod->ndev;
	int count = 0;
	
        u16 filter = 0x1d;

	pipe_data = get_devdata_with_iod(usb_ld, iod);;
		/* get active async */
	dev = &usb_ld->usbdev->dev;
	if(can_sleep){
		rpm_state = link_pm_runtime_get_active_async(usb_ld->link_pm_data);
		if (!rpm_state)
			pm_runtime_get_noresume(dev);
	}else{
		mif_info("-----\n");
	}

	urb = usb_alloc_urb(0, GFP_ATOMIC); 
	if (!urb) { 
		mif_info("Error allocating URB\n");
                return;
        }

        req = kmalloc(sizeof(*req), GFP_ATOMIC);
        if (!req) {
                mif_info("Error allocating control msg\n");
                goto out;
        }

        req->bRequestType = USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE;
        req->bRequest = 0x43;
        req->wValue = cpu_to_le16(filter);
        req->wIndex = 9;
        req->wLength = 0;

        usb_fill_control_urb(urb, usb_ld->usbdev, usb_sndctrlpipe(usb_ld->usbdev, 0),
                (void *)req, NULL, 0,
                usb_async_cmd_callback,
                (void *)req);
	if(can_sleep){
retry:
		if (pipe_data->state == STATE_SUSPENDED
				|| usb_get_rpm_status(dev) == RPM_SUSPENDING) {
			/*
			   usb_anchor_urb(urb, &pipe_data->tx_deferd_urbs);
			   if (pipe_data->ndev)
			   mif_net_suspend(pipe_data, MIF_NET_SUSPEND_LINK_WAKE);
			 */
			mif_info("need wait\n");	
			count++;
			msleep(500);
			if(count>10)
				return;
			goto retry;
		}
	}

        status = usb_submit_urb(urb, GFP_ATOMIC);
        if (status < 0) {
                mif_err("Error submitting control msg, sts=%d\n",
                            status);
                goto out1;
        }
	if(can_sleep){
		if (!rpm_state) {
			usb_mark_last_busy(usb_ld->usbdev);
			pm_runtime_put(&usb_ld->usbdev->dev);
		}
	}
	c8320_modem_ctl_ex.cmd_send = false;
        return;
out1:
        kfree(req);
out:
        usb_free_urb(urb);
	mif_info("go to out\n");
}

static int usb_init_rx_skb_pool(struct if_usb_devdata *pipe_data)
{
	struct sk_buff *free_skb;	

	while (pipe_data->free_rx_q.qlen < RX_POOL_SIZE) {
		free_skb = alloc_skb((pipe_data->rx_buf_size + NET_IP_ALIGN),
			GFP_KERNEL | GFP_DMA);
		if (!free_skb) {
			mif_err("alloc free skb fail\n");
			return -ENOMEM;
		}
		skb_queue_tail(&pipe_data->free_rx_q, free_skb);
	}
	return 0;
}

static int usb_init_communication(struct link_device *ld, struct io_device *iod)
{
	struct task_struct *task = get_current();
	char str[TASK_COMM_LEN];
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct if_usb_devdata *pipe_data;

	mif_info("%d:%s\n", task->pid, get_task_comm(str, task));

	if (!usb_ld->if_usb_connected) {
		mif_err("HSIC not connected, open fail\n");
		return -ENODEV;
	}

	pipe_data = get_devdata_with_iod(usb_ld, iod);
		
	switch (iod->format) {
	case IPC_RAW_NCM:
		pipe_data->ndev = iod->ndev;
		if(!strcmp(iod->ndev->name,"usb1")){
			printk(" -------------------------------------------------------------set multicast \n");
			usbnet_set_multicast(ld,iod,1);
		}
		break;
	default:
		break;
	}	
	return 0;
}

static void usb_terminate_communication(struct link_device *ld,
			struct io_device *iod)
{
	return;
}

/* CDC class interrupt in endpoint control*/
static int init_status(struct if_usb_devdata *pipe_data,
	struct usb_interface *intf)
{
	char		*buf = NULL;
	unsigned	pipe = 0;
	unsigned	maxp;
	unsigned	period;

	if (!pipe_data->info->intr_complete)
		return 0;

	pipe = usb_rcvintpipe(pipe_data->usbdev,
			pipe_data->status->desc.bEndpointAddress
				& USB_ENDPOINT_NUMBER_MASK);
	maxp = usb_maxpacket(pipe_data->usbdev, pipe, 0);

	/* avoid 1 msec chatter:  min 8 msec poll rate*/
	period = max((int)pipe_data->status->desc.bInterval,
		(pipe_data->usbdev->speed == USB_SPEED_HIGH) ? 7 : 3);
	buf = kmalloc(maxp, GFP_KERNEL);
	if (buf) {
		pipe_data->intr_urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!pipe_data->intr_urb) {
			kfree(buf);
			return -ENOMEM;
		} else {
			usb_fill_int_urb(pipe_data->intr_urb, pipe_data->usbdev,
				pipe, buf, maxp, pipe_data->info->intr_complete,
				pipe_data, period);
			pipe_data->intr_urb->transfer_flags |= URB_FREE_BUFFER;
			mif_debug("status ep%din, %d bytes period %d\n",
				usb_pipeendpoint(pipe), maxp, period);
		}
	}
	return 0;
}

static int usb_rx_submit(struct usb_link_device *usb_ld,
		struct if_usb_devdata *pipe_data, gfp_t gfp_flags)
{
	int ret = 0;
	struct urb *urb;
	struct sk_buff *skb;
	int delay = 0;

	ehci_vendor_txfilltuning();

	skb = skb_dequeue(&pipe_data->free_rx_q);	/* free queue first */
	if (!skb) {
		skb = alloc_skb((pipe_data->rx_buf_size + NET_IP_ALIGN),
				GFP_ATOMIC); /* alloc new skb with GFP_ATOMIC */
		if (!skb) {
			mif_debug("alloc skb fail\n");
			pipe_data->defered_rx = true;
			delay = msecs_to_jiffies(20);
			goto defered_submit;
		}
	}

	skbpriv(skb)->context = pipe_data;
	urb = pipe_data->urb;
	urb->transfer_flags = 0;
	usb_fill_bulk_urb(urb, pipe_data->usbdev, pipe_data->rx_pipe,
		(void *)skb->data, pipe_data->rx_buf_size, usb_rx_complete,
		(void *)skb);

	if (pipe_data->disconnected)
		return -ENOENT;

	usb_mark_last_busy(usb_ld->usbdev);
	ret = usb_submit_urb(urb, gfp_flags);
	if (ret)
		mif_err("submit urb fail with ret (%d)\n", ret);

defered_submit:
	/* Hold L0 until rx sumit complete */
	usb_mark_last_busy(usb_ld->usbdev);
	schedule_delayed_work(&pipe_data->rx_defered_work, delay);
	return ret;
}

static void usb_defered_work(struct work_struct *work)
{
	struct if_usb_devdata *pipe_data = container_of(work,
		struct if_usb_devdata, rx_defered_work.work);

	if (pipe_data->disconnected)
		return;

	usb_init_rx_skb_pool(pipe_data);
	if (pipe_data->defered_rx) {
		pipe_data->defered_rx = false;
		mif_debug("defered rx submit\n");
		usb_mark_last_busy(pipe_data->usbdev);
		usb_rx_submit(pipe_data->usb_ld, pipe_data, GFP_ATOMIC);
	}
}

static void usb_rx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct if_usb_devdata *pipe_data = skbpriv(skb)->context;
	struct usb_link_device *usb_ld = pipe_data->usb_ld;
	struct io_device *iod = pipe_data->iod;
	int ret;

	if (usb_ld->usbdev)
		usb_mark_last_busy(usb_ld->usbdev);
	
	switch (urb->status) {
	case -ENOENT:
	case 0:
	//	printk("%s: urb->status is %d, actual len is %d\n",__func__,urb->status,urb->actual_length);
		usb_ld->rx_err = 0;
		if (!urb->actual_length) {
			mif_debug("urb has zero length!\n");
			/* reuse RX skb*/
			skb_queue_tail(&pipe_data->free_rx_q, skb);
			goto rx_submit;
		}
		/* call iod recv */
		skb_put(skb, urb->actual_length);
		
		if (pipe_data->info->rx_fixup) {			
			pipe_data->info->rx_fixup(pipe_data, skb);
			/* cdc_ncm_rx_fixup will be free the skb */
			goto rx_submit;
		}
		/* flow control CMD by CP, not use io device */
		usb_ld->rx_cnt++;
		ret = iod->recv_skb(iod, &usb_ld->ld, skb);
		if (ret < 0) {
			mif_err("io device recv error (%d)\n", ret);
			pr_urb("HDLC error", urb);
			dev_kfree_skb_any(skb);
			return; /* skip rx submit */
		}
		dev_kfree_skb_any(skb);
rx_submit:
		if (urb->status == 0) {
			if (usb_ld->usbdev)
				usb_mark_last_busy(usb_ld->usbdev);
			usb_rx_submit(usb_ld, pipe_data, GFP_ATOMIC);
		}
		break;
	default:
		mif_err("urb error status = %d\n", urb->status);
		if (usb_ld->rx_err++ > MAX_RX_ERROR
					&& c8320_modem_ctl_ex.com_state == COM_ONLINE) {
			usb_ld->rx_err = 0;
			sprintf(last_link_err, "RX error %d times\n",
								MAX_RX_ERROR);
			mif_err("%s\n", last_link_err);
			set_bit(LINK_EVENT_RECOVERY, &usb_ld->events);

		}
		dev_kfree_skb_any(skb);
		break;
	}
}

static int usb_send(struct link_device *ld, struct io_device *iod,
				struct sk_buff *skb)
{
	int ret;
	size_t tx_size;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct if_usb_devdata *pipe_data = get_devdata_with_iod(usb_ld, iod);	
	
	/* store the tx size before run the tx_delayed_work*/
	tx_size = skb->len;

	/* drop packet, when link is not online */
	if (c8320_modem_ctl_ex.com_state != COM_ONLINE ) {
		mif_err("%s: drop packet, size=%d, com_state=%d\n",
				iod->name, skb->len, c8320_modem_ctl_ex.com_state);
		dev_kfree_skb_any(skb);
		return 0;
	}

	/* Synchronous tx */
	ret = usb_tx_skb(pipe_data, skb);
	if (ret < 0) {
		/* TODO: */
		mif_err("usb_tx_skb fail(%d)\n", ret);
	}

	return tx_size;
}

static void usb_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct io_device *iod = skbpriv(skb)->iod;
	struct link_device *ld = skbpriv(skb)->ld;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct if_usb_devdata *pipe_data = skbpriv(skb)->context;
	unsigned long flag;
	bool retry_tx_urb = false;

	switch (urb->status) {
	case 0:
		if (urb->actual_length != urb->transfer_buffer_length)
			mif_err("TX len=%d, Complete len=%d\n",
				urb->transfer_buffer_length, urb->actual_length);
/*
		printk("%s:TX len=%d, Complete len=%d\n",__func__,
				urb->transfer_buffer_length, urb->actual_length);		
*/
		if (pipe_data->txpend_ts.tv_sec) {
			/* TX flowctl was resumed */
			mif_info("flowctl %s CH%d(%d) (%02d:%02d:%02d.%09lu)\n",
				"resume", pipe_data->idx, pipe_data->tx_pend,
				pipe_data->txpend_tm.tm_hour,
				pipe_data->txpend_tm.tm_min,
				pipe_data->txpend_tm.tm_sec,
				pipe_data->txpend_ts.tv_nsec);
			pipe_data->txpend_ts.tv_sec = 0;
		}
		break;
	case -ECONNRESET:
		/* This urb was returned by suspend unlink urb and it will send
		 * again at next resume time for XMM6360  CDC-NCM flowctl*/
		if (urb->actual_length)
			mif_err("ECONNRESET: TX len=%d, Complete len=%d\n",
				urb->transfer_buffer_length, urb->actual_length);
		if (pipe_data->format == IPC_RAW_NCM
						&& !pipe_data->net_connected) {
			retry_tx_urb = false;
			pipe_data->txpend_ts.tv_sec = 0;
			mif_info("NCM net stop(EP:%d), free remain NTB\n",
						usb_pipeendpoint(urb->pipe));
		} else {
			retry_tx_urb = true;
		}
		break;
	case -EPROTO:
		/* Keep the IPC/RFS packet if status -EPROTO, it will be send
		 * next time.
		 * If real USB protocol error, runtime resume will be failed
		 * and the RECOVERY EVENT will rise */
		if (iod->format == IPC_FMT ) {
			retry_tx_urb = true;
			queue_delayed_work(usb_ld->link_pm_data->wq,
					&usb_ld->link_pm_data->link_pm_work,
					msecs_to_jiffies(20));
			mif_info("WARN: TX retry IPC/RFS packet, rpm work\n");
		}
	case -ENOENT:
	case -ESHUTDOWN:
	default:
		if (iod->format != IPC_BOOT) {
			printk("TX error (%d), EP(%d)\n",
				urb->status, usb_pipeendpoint(urb->pipe));
			mif_info("TX error (%d), EP(%d)\n", urb->status,
						usb_pipeendpoint(urb->pipe));
		}
	}

	spin_lock_irqsave(&pipe_data->sk_tx_q.lock, flag);
	__skb_unlink(skb, &pipe_data->sk_tx_q);
	if (pipe_data->ndev && pipe_data->sk_tx_q.qlen < tx_qlen
						&& !pipe_data->txpend_ts.tv_sec)
		netif_wake_queue(pipe_data->ndev);
	spin_unlock_irqrestore(&pipe_data->sk_tx_q.lock, flag);

	if (retry_tx_urb) {
		/* If tx error case, retry_tx_urb flags will retry tx submit
		 * next runtime resume, it keep the urb to tx_deferd_urb and
		 * interface resume(if_usb_resume) will be re-submit these urbs.
		 */
		usb_anchor_urb(urb, &pipe_data->tx_deferd_urbs);
		mif_info("TX deferd urbs (%d) EP(%d)\n", urb->status,
						usb_pipeendpoint(urb->pipe));
	} else {
		dev_kfree_skb_any(skb);
		usb_free_urb(urb);
	}

	if (urb->dev && usb_ld->if_usb_connected)
		usb_mark_last_busy(urb->dev);
}

int usb_tx_skb(struct if_usb_devdata *pipe_data, struct sk_buff *skb)
{
	int ret, rpm_state;
	struct usb_link_device *usb_ld = NULL;
	struct device *dev = NULL;
	struct urb *urb;
	gfp_t mem_flags = in_interrupt() ? GFP_ATOMIC : GFP_KERNEL;
	unsigned long flag;	
	usb_ld = pipe_data->usb_ld;
	
	dev = &usb_ld->usbdev->dev;

	if (!usb_ld->usbdev) {
		mif_info("usbdev is invalid\n");
		return -EINVAL;
	}

	usb_ld->tx_cnt++;
	/* get active async */
	rpm_state = link_pm_runtime_get_active_async(usb_ld->link_pm_data);
	if (!rpm_state)
		pm_runtime_get_noresume(dev);
	
	if (pipe_data->info->tx_fixup) {
		/* check the modem status before sending IP packet */
		#if 0
		if (c8320_modem_ctl_ex.phone_state != STATE_ONLINE
						&& pipe_data->iod->ndev) {
			mif_info("not STATE_ONLINE, netif_carrier_off\n");
			netif_carrier_off(pipe_data->iod->ndev);
			ret = -EINVAL;
			goto done;
		}
		#endif

		//pr_tx_skb_with_format(pipe_data->iod->format, skb);
		skb = pipe_data->info->tx_fixup(pipe_data, skb, mem_flags);
		if (!skb) {
			//mif_debug("CDC-NCM gether skbs to NTB\n");
			ret = 0;
			goto done;
		}
	}
	if (!skb) {
		mif_err("invalid skb\n");
		ret = -EINVAL;
		goto done;
	}
	urb = usb_alloc_urb(0, mem_flags);
	if (!urb) {
		mif_err("alloc urb error\n");
		ret = -ENOMEM;
		goto done;
	}
	if (!skbpriv(skb)->nzlp)
		urb->transfer_flags = URB_ZERO_PACKET;

	usb_fill_bulk_urb(urb, pipe_data->usbdev, pipe_data->tx_pipe, skb->data,
			skb->len, usb_tx_complete, (void *)skb);

	skbpriv(skb)->context = pipe_data;
	skbpriv(skb)->urb = urb; /* kill urb when suspend if tx not complete*/

	/* Check usb link_pm status and if suspend,  transmission will be done
	 * in resume.
	 * If TX packet was sent from rild/network while interface suspending,
	 * It will fail because link pm will be changed to L2 as soon as tx
	 * submit. It should keep the TX packet and send next time when
	 * RPM_SUSPENDING status.
	 * In RPM_RESUMMING status, pipe_data->state flag can help the submit
	 * timming after sending previous kept anchor urb submit.
	 */
	if (pipe_data->state == STATE_SUSPENDED
				|| usb_get_rpm_status(dev) == RPM_SUSPENDING) {
		usb_anchor_urb(urb, &pipe_data->tx_deferd_urbs);
		if (pipe_data->ndev)
			mif_net_suspend(pipe_data, MIF_NET_SUSPEND_LINK_WAKE);
		ret = 0;
		goto done;
	}

	spin_lock_irqsave(&pipe_data->sk_tx_q.lock, flag);
	__skb_queue_tail(&pipe_data->sk_tx_q, skb);
	if (pipe_data->ndev && pipe_data->sk_tx_q.qlen > tx_qlen)
		netif_stop_queue(pipe_data->ndev);
	spin_unlock_irqrestore(&pipe_data->sk_tx_q.lock, flag);

	ret = usb_submit_urb(urb, mem_flags);
	if (ret < 0) {
		mif_err("usb_submit_urb with ret(%d)\n", ret);
		spin_lock_irqsave(&pipe_data->sk_tx_q.lock, flag);
		__skb_unlink(skb, &pipe_data->sk_tx_q);
		spin_unlock_irqrestore(&pipe_data->sk_tx_q.lock, flag);
		usb_anchor_urb(urb, &pipe_data->tx_deferd_urbs);
		goto done;
	}

done:
	if (!rpm_state) {
		usb_mark_last_busy(usb_ld->usbdev);
		pm_runtime_put(&usb_ld->usbdev->dev);
	}
	return  ret;
}


static void link_pm_change_modem_state(struct usb_link_device *usb_ld,
						enum modem_state state)
{
	struct modem_ctl *mc = usb_ld->ld.mc;
	struct modem_shared *msd = mc->msd;
	struct io_device *io_dev = NULL;
/*
	if (c8320_modem_ctl_ex.com_state == COM_ONLINE)
		return;
*/
	mif_info("set modem_state is %d\n", state);

	list_for_each_entry(io_dev,&msd->iodevs_list,list){
		io_dev->modem_state_changed(io_dev, state);
	}
}

/*
#ifdef CONFIG_LINK_PM
*/

#ifdef CONFIG_PM_RUNTIME
static int link_pm_runtime_get_active_async(struct link_pm_data *pm_data)
{
	struct usb_link_device *usb_ld = pm_data->usb_ld;
	struct device *dev = &usb_ld->usbdev->dev;

	if (!usb_ld->if_usb_connected || c8320_modem_ctl_ex.com_state != COM_ONLINE)
		return -ENODEV;

	if (usb_get_rpm_status(dev) == RPM_ACTIVE) {
		pm_data->resume_retry_cnt = 0;
		return 0;
	}

	if (!pm_data->resume_requested) {
		mif_debug("QW PM\n");
		pm_data->apinit_l0_req = true;
		queue_delayed_work(pm_data->wq, &pm_data->link_pm_work, 0);
	}

	return 0;
}

static void link_pm_runtime_start(struct work_struct *work)
{
	struct link_pm_data *pm_data =
		container_of(work, struct link_pm_data, link_pm_start.work);
	struct device *dev;

	if (!pm_data->usb_ld->if_usb_connected
		|| c8320_modem_ctl_ex.com_state != COM_ONLINE) {
		mif_info("CP not in normal mode, ignore\n");
		return;
	}

	dev = &pm_data->usb_ld->usbdev->dev;

	/* wait interface driver resumming */
	if (usb_get_rpm_status(dev) == RPM_SUSPENDED) {
		mif_info("suspended yet, delayed work\n");
		queue_delayed_work(pm_data->wq, &pm_data->link_pm_start,
			msecs_to_jiffies(20));
		return;
	}

	if (pm_data->usb_ld->usbdev && dev->parent) {
		struct device *ppdev;
		
		mif_info("rpm_status: %d\n", usb_get_rpm_status(dev));
		pm_runtime_set_autosuspend_delay(dev, 500);
		ppdev = dev->parent->parent;
		pm_runtime_allow(dev);
		pm_runtime_allow(ppdev);/*ehci*/
		pm_data->link_pm_active = true;
		pm_data->resume_requested = false;
		pm_data->link_reconnect_cnt = 5;
		pm_data->resume_retry_cnt = 0;
	}
}

static void link_pm_reconnect_work(struct work_struct *work)
{
	struct link_pm_data *pm_data =
		container_of(work, struct link_pm_data,
					link_reconnect_work.work);
	struct modem_ctl *mc = if_usb_get_modemctl(pm_data);
	if (!mc || pm_data->usb_ld->if_usb_connected)
		return;

	if (c8320_modem_ctl_ex.com_state != COM_ONLINE)
		return;

	if (pm_data->link_reconnect_cnt--) {
		if (c8320_modem_ctl_ex.phone_state == STATE_ONLINE &&
						!pm_data->link_reconnect())
			/* try reconnect and check */
			schedule_delayed_work(&pm_data->link_reconnect_work,
							msecs_to_jiffies(500));
		else	/* under cp crash or reset, just return */
			return;
	} else {
		/* try to recover cp */
		if (!pm_data->disable_recovery) {
			mif_err("recover connection: silent reset\n");
			link_pm_change_modem_state(pm_data->usb_ld,
							STATE_CRASH);
		}
	}
}

static void link_pm_runtime_work(struct work_struct *work)
{
	int ret;
	struct link_pm_data *pm_data =
		container_of(work, struct link_pm_data, link_pm_work.work);
	struct device *dev = &pm_data->usb_ld->usbdev->dev;

	if (!pm_data->usb_ld->if_usb_connected || pm_data->dpm_suspending)
		return;

	if (c8320_modem_ctl_ex.phone_state != STATE_ONLINE){
		return;
	}
	if (c8320_modem_ctl_ex.com_state == COM_NONE)
		return;

	mif_info("for dev 0x%p : current %d \n(0:act;1:resuming;2:suspended;3:suspending)\n", dev, usb_get_rpm_status(dev));

	switch (usb_get_rpm_status(dev)) {
	case RPM_ACTIVE:
		pm_data->resume_retry_cnt = 0;
		pm_data->resume_requested = false;
		pm_data->apinit_l0_req = false;
		pm_data->rpm_suspending_cnt = 0;
		return;
	case RPM_SUSPENDED:
		if (pm_data->resume_requested)
			break;
		pm_data->resume_requested = true;
		if (!pm_data->usb_ld->if_usb_connected) {
			mif_info("not conneted\n");
			return;
		}		
		ret = pm_runtime_resume(dev);
		if (ret < 0) {
			mif_err("resume error(%d)\n", ret);
			if (!pm_data->usb_ld->if_usb_connected) {
				return;
			}
			/* force to go runtime idle before retry resume */
			if (dev->power.timer_expires == 0 &&
						!dev->power.request_pending) {
				mif_debug("run time idle\n");
				pm_runtime_idle(dev);
			}
		}
		pm_data->rpm_suspending_cnt = 0;
		break;
	case RPM_SUSPENDING:
		/* Checking the usb_runtime_suspend running time.*/
		mif_info("rpm_states=%d", usb_get_rpm_status(dev));
		pm_data->rpm_suspending_cnt++;
		if (pm_data->rpm_suspending_cnt < 10)
			msleep(20);
		else if (pm_data->rpm_suspending_cnt < 30)
			msleep(50);
		else
			msleep(100);

		break;
	default:
		pm_data->rpm_suspending_cnt = 0;
		break;
	}
	pm_data->resume_requested = false;

	/* check until runtime_status goes to active */
	/* attemp 10 times, or re-establish modem-link */
	/* if pm_runtime_resume run properly, rpm status must be in ACTIVE */
	if (usb_get_rpm_status(dev) == RPM_ACTIVE) {
		pm_data->resume_retry_cnt = 0;
		pm_data->rpm_suspending_cnt = 0;
	} else if (pm_data->resume_retry_cnt++ > 80
						&& !pm_data->disable_recovery) {
		mif_err("runtime_status(%d), retry_cnt(%d)\n",
			usb_get_rpm_status(dev), pm_data->resume_retry_cnt);
		link_pm_change_modem_state(pm_data->usb_ld, STATE_CRASH);
	} else{

		queue_delayed_work(pm_data->wq, &pm_data->link_pm_work,
							msecs_to_jiffies(20));
	}
}

#ifdef CONFIG_LINK_DEVICE_POWER_MANGER 
static irqreturn_t link_pm_irq_handler(int irq, void *data)
{
	int value;
	struct link_pm_data *pm_data = data;

	if (!pm_data->link_pm_active){
		return IRQ_HANDLED;
	}
	if (c8320_modem_ctl_ex.phone_state != STATE_ONLINE){
		return IRQ_HANDLED;
	}

	value = gpio_get_value(pm_data->gpio_link_hostwake);
	mif_info("CP>>AP: set hostwakeup  %d host trig LOW level\n", value);

	if (value != HOSTWAKE_TRIGLEVEL) {
		mif_info("AP>>CP set slavewakeup  0 trigel is HIGH level\n");
		gpio_set_value(pm_data->gpio_link_slavewake, 0);	
		return IRQ_HANDLED;
	}
	/*
	* igonore host wakeup interrupt at suspending kernel
	*/
	if (pm_data->dpm_suspending) {
		mif_info("ignore request by suspending\n");
		/* Ignore HWK but AP got to L2 by suspending fail */
		wake_lock(&pm_data->l2_wake);
		return IRQ_HANDLED;
	}

	/* debounce the slave wakeup after usb resume */
	wake_lock(&pm_data->l2_wake);
	queue_delayed_work(pm_data->wq, &pm_data->link_pm_work, 0);
	return IRQ_HANDLED;
}
extern int c83xx_usb_power_config_extern(int on);
static irqreturn_t link_pm_suspend_irq_handler(int irq, void *data)
{
	int value;
	struct link_pm_data *pm_data = data;
	if(c8320_modem_ctl_ex.phone_state != STATE_ONLINE){
		return IRQ_HANDLED;
	}
	if(c8320_modem_ctl_ex.com_state != COM_ONLINE){
		mif_info("do nothing\n");
		return IRQ_HANDLED;
	}
	if(gpio_get_value(pm_data->gpio_link_suspend_req) == 0){
		mif_info("close cp usb power\n");
		c83xx_usb_power_config_extern(0);
		wake_unlock(&pm_data->rpm_wake);
	}
	return IRQ_HANDLED;
}
#endif
#endif

static long link_pm_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	int value;
	struct link_pm_data *pm_data = file->private_data;
	struct modem_ctl *mc = if_usb_get_modemctl(pm_data);

	mif_info("%x\n", cmd);

	switch (cmd) {
	case IOCTL_LINK_CONTROL_ENABLE:
		if (copy_from_user(&value, (const void __user *)arg,
							sizeof(int)))
			return -EFAULT;
		if (pm_data->link_ldo_enable)
			pm_data->link_ldo_enable(!!value);
		if (pm_data->gpio_link_enable)
			gpio_set_value(pm_data->gpio_link_enable, value);
		break;
	case IOCTL_LINK_CONTROL_ACTIVE:
		if (copy_from_user(&value, (const void __user *)arg,
							sizeof(int)))
			return -EFAULT;
		gpio_set_value(pm_data->gpio_link_active, value);
		break;
	case IOCTL_LINK_GET_HOSTWAKE:
		return !gpio_get_value(pm_data->gpio_link_hostwake);
	case IOCTL_LINK_CONNECTED:
		return pm_data->usb_ld->if_usb_connected;
	case IOCTL_LINK_SET_BIAS_CLEAR:
		if (copy_from_user(&value, (const void __user *)arg,
							sizeof(int)))
			return -EFAULT;
		if (value) {
			gpio_direction_output(pm_data->gpio_link_slavewake, 0);
			gpio_direction_output(pm_data->gpio_link_hostwake, 0);
		} else {
			gpio_direction_output(pm_data->gpio_link_slavewake, 0);
			gpio_direction_input(pm_data->gpio_link_hostwake);
			irq_set_irq_type(pm_data->irq_link_hostwake,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
		}
		break;
	case IOCTL_LINK_GET_PHONEACTIVE:
		return gpio_get_value(mc->gpio_phone_active);
	case IOCTL_LINK_DISABLE_RECOVERY:
		if (copy_from_user(&value, (const void __user *)arg,
							sizeof(int)))
			return -EFAULT;
		pm_data->disable_recovery = value ? true : false;
		mif_info("change disable recovery (%d)\n",
						pm_data->disable_recovery);
		break;
	default:
		break;
	}

	return 0;
}

static int link_pm_open(struct inode *inode, struct file *file)
{
	struct link_pm_data *pm_data =
		(struct link_pm_data *)file->private_data;
	file->private_data = (void *)pm_data;
	return 0;
}

static int link_pm_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static const struct file_operations link_pm_fops = {
	.owner = THIS_MODULE,
	.open = link_pm_open,
	.release = link_pm_release,
	.unlocked_ioctl = link_pm_ioctl,
};

enum known_device_type {
	MIF_UNKNOWN_DEVICE,
	MIF_MAIN_DEVICE,
	MIF_BOOT_DEVICE,
};

struct  link_usb_id {
	int vid;
	int pid;
	enum known_device_type type;
};

/*TODO: get pid, vid from platform data */
static struct link_usb_id link_device_ids[] = {
	{0x231e, 0x6304, MIF_MAIN_DEVICE}, /* XMM6360 */
};

/* hooking from generic_suspend and generic_resume */
static int (*_usb_suspend) (struct usb_device *, pm_message_t);
static int (*_usb_resume) (struct usb_device *, pm_message_t);

struct linkpm_devices {
	struct list_head pm_data;
	spinlock_t lock;
} c8320_devices;

static int linkpm_known_device(const struct usb_device_descriptor *desc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(link_device_ids); i++) {
		if (link_device_ids[i].vid == desc->idVendor &&
				link_device_ids[i].pid == desc->idProduct)
			return link_device_ids[i].type;
	}
	return MIF_UNKNOWN_DEVICE;
}

static struct link_pm_data *linkdata_from_udev(struct usb_device *udev)
{
	struct link_pm_data *pm_data = NULL;

	spin_lock_bh(&c8320_devices.lock);
	list_for_each_entry(pm_data, &c8320_devices.pm_data, link) {
		if (pm_data
			&& (udev == pm_data->udev || udev == pm_data->hdev)) {
			spin_unlock_bh(&c8320_devices.lock);
			return pm_data;
		}
		mif_debug("udev=%p, %s\n", udev, dev_name(&udev->dev));
	}
	spin_unlock_bh(&c8320_devices.lock);
	return NULL;
}

static void set_slavewake(struct link_pm_data *pm_data, int val)
{
	
	if (!val) {
		mif_info("AP>>CP :slavewakeup 0\n");
		gpio_set_value(pm_data->gpio_link_slavewake, 0);
	} else {
		if (gpio_get_value(pm_data->gpio_link_slavewake)) {
			mif_info("warn.. slavewake toggle,fist AP>>CP :slavewakeup 0\n");			
			gpio_set_value(pm_data->gpio_link_slavewake, 0);
			msleep(20);
		}
		mif_info("AP>>CP :slavewakeup 1\n");
		gpio_set_value(pm_data->gpio_link_slavewake, 1);
	}
	printk(KERN_DEBUG "mif: slave wake(%d)\n",
			gpio_get_value(pm_data->gpio_link_slavewake));
}

/* c8320 GPIO L2->L0 sequence */
static int c8320_gpio_usb_resume(struct link_pm_data *pm_data)
{
	int spin = 20;

	if (get_hostwake(pm_data)){ /* CP inititated L2->L0 */
		mif_info("hostwakeup is 0, not need set salvewakeup\n");
		goto exit;
	}

	/* AP initiated L2->L0 */
	set_slavewake(pm_data, 1);
	/* 
	 * here why need wait hostwake =1 
	 * sepc is: AP>>CP slavewake 1,
	 * then CP>>AP hosthost 0, cool_zhao
	 * 
	 * because get_hostwake is means hostwakeup gpio is 0 not 1
	 */
	while (spin-- && !get_hostwake(pm_data))
		mdelay(5);

	if (!get_hostwake(pm_data)) /* Hostwakeup timeout */
		return -ETIMEDOUT;
exit:
	return 0;
}

#define CP_PORT 2
static int c8320_gpio_usb_resume_cp_l3(struct link_pm_data *pm_data)
{
	int spin = 20;

	if (get_hostactive(pm_data)){
		mif_info("hostactive is 1 do nothing\n");
		goto exit;
	}

	/* AP initiated L2->L0 */
	set_slavewake(pm_data, 1);

	while (spin-- && !get_hostwake(pm_data))
		mdelay(5);

	if (!get_hostwake(pm_data)){ /* Hostwakeup timeout */
		mif_info("host wakeup not set active\n");
		return -ETIMEDOUT;
	}
	mif_info("set host_active 1\n");
	gpio_set_value(pm_data->gpio_link_active, 1);
	if (pm_data->wait_cp_connect)
		pm_data->wait_cp_connect(CP_PORT);
exit:
	return 0;
}

/* c8320 GPIO L3->L0 sequence */
static int c8320_gpio_l3_resume(struct link_pm_data *pm_data)
{
	int retry = 50;
	pm_data->roothub_resume_req = false;
	/* this point PDA Active should high */
	if (get_hostactive(pm_data)){
		mif_info("hostactive is 1 not need set host activ\n");
		return 0;
	}
	if (!get_hostwake(pm_data)) {
		set_slavewake(pm_data, 1);
		do{
			msleep(20);
		}while(!get_hostwake(pm_data)&&(retry--));
		printk("%s:retry 50 left times %d\n",__func__,retry);
		if(retry< 0){
			return -1;
		}
	}
	printk("AP>>CP:set host active 1\n");
	gpio_set_value(pm_data->gpio_link_active, 1);
	//printk("mif: host active(%d)\n", get_hostactive(pm_data));
	if (pm_data->wait_cp_connect)
		pm_data->wait_cp_connect(CP_PORT);

	return 0;
}

static int c8320_linkpm_usb_resume(struct usb_device *udev, pm_message_t msg)
{
	struct link_pm_data *pm_data = linkdata_from_udev(udev);
	int ret = 0;
	int cnt = 10;
	mif_info("start\n");
	wake_lock(&pm_data->l2_wake);
#if 0
	if(pm_data->c8320_runtime_resume){
		mif_info("c8320 is in runtime resume\n");
		msleep(500);
		return 0 ;
	}
#endif
	pm_data->c8320_runtime_resume = true;
	if (!pm_data){ /* unknown devices */
		mif_info("BUG!!! unknown devices\n");
		goto generic_resume;
	}
/* 
 * here udev never eq hdev but i think it means when AP is in L3 
 * we need set phone active and resume cool_zhao 
 */
	if (udev == pm_data->hdev) {
	
		pm_runtime_mark_last_busy(&pm_data->hdev->dev);
		ret = c8320_gpio_l3_resume(pm_data);  /* when use l3 to resume */		
		if(ret){
				pm_data->c8320_runtime_resume = false;
				return 0;
		}
		goto generic_resume;
	}
	
	/* Because HSIC modem skip the hub dpm_resume by quirk, if root hub
	  dpm_suspend was called at runtmie active status, hub resume was not
	  call by port runtime resume. So, it check the L3 status and root hub
	  resume before port resume */
	if (!get_hostactive(pm_data)) {
		mif_info("AP is l2  and cp is l3 and hostactive is 0\n");
		c8320_gpio_usb_resume_cp_l3(pm_data);
		goto generic_resume;
	}

	if (pm_data->roothub_resume_req) {
		mif_err("ehci root hub resume first\n");
		pm_runtime_mark_last_busy(&pm_data->hdev->dev);
		if(usb_get_rpm_status(&pm_data->hdev->dev)!=RPM_ACTIVE){
			ret = usb_resume(&pm_data->hdev->dev, PMSG_RESUME);
			if (ret)
				mif_err("hub resume fail\n");
		}
	}
 retry:
	/* Sometimes IMC modem send remote wakeup with gpio, we should check
	  the runtime status and if aleady resumed, */
	if (usb_get_rpm_status(&udev->dev) == RPM_ACTIVE) {
		mif_info("aleady resume, skip gpio resume\n");
		goto generic_resume;
	}
	mif_info("next set slave wake up\n");
	ret = c8320_gpio_usb_resume(pm_data);
	if (ret < 0) {
		if (cnt--) {
			mif_info("%s: fail(%d)\n", __func__,ret);
			goto retry;
		} else {
			mif_err("many time (10) retry set slavewakeup ,but not waited host wakeup. failed\n");
			/* TODO: exception handing if hoswakeup timeout */
			return -EIO;
		}
	}

generic_resume:
	ret = _usb_resume(udev, msg);
	mif_info("generic resume complete ret is %d\n",ret);
	pm_data->c8320_runtime_resume = false;
	return ret;
}

static int c8320_linkpm_usb_suspend(struct usb_device *udev, pm_message_t msg)
{
	struct link_pm_data *pm_data = linkdata_from_udev(udev);
	int ret = 0;


	if (!pm_data) /* unknown devices */
		goto generic_suspend;

	if (!pm_data->usb_ld->if_usb_connected)
		goto generic_suspend;

	if (msg.event == PM_EVENT_SUSPEND) {
		mif_info("hub suspend with rpm active\n");
		pm_data->roothub_resume_req = true;
	}
	/* DO nothing yet */
generic_suspend:
	ret = _usb_suspend(udev, msg);
	if(pm_data->hdev == udev){
		wake_lock_timeout(&pm_data->l2_wake, msecs_to_jiffies(50));
	}
	return ret;	
}
/* 
 * this function is add usb_dev(ipc),add GPIO control when usb_driver suspend resume 
 * ?????? cool_zhao 
 */
static int link_usb_notifier_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	struct link_pm_data *pm_data =
			container_of(this, struct link_pm_data,	usb_notifier);
	struct usb_device *udev = ptr;
	const struct usb_device_descriptor *desc = &udev->descriptor;
	struct usb_device_driver *udriver =
					to_usb_device_driver(udev->dev.driver);
	unsigned long flags;
	
	switch (event) {
	case USB_DEVICE_ADD:
	
		if (linkpm_known_device(desc) == MIF_MAIN_DEVICE) {
			if (pm_data->udev) {
				mif_err("pmdata was assigned for udev=%p\n",
								pm_data->udev);
				return NOTIFY_DONE;
			}
			pm_data->udev = udev;
			pm_data->hdev = udev->bus->root_hub;
			mif_info("udev=%p, hdev=%p\n", udev, pm_data->hdev);

			spin_lock_irqsave(&pm_data->lock, flags);
			if (!_usb_resume && udriver->resume) {				
				_usb_resume = udriver->resume;
				udriver->resume = c8320_linkpm_usb_resume;
			}
			if (!_usb_suspend && udriver->suspend) {
				_usb_suspend = udriver->suspend;
				udriver->suspend = c8320_linkpm_usb_suspend;
			}
			spin_unlock_irqrestore(&pm_data->lock, flags);
			mif_info("hook: (%pf, %pf), (%pf, %pf)\n",
					_usb_resume, udriver->resume,
					_usb_suspend,	udriver->suspend);
			pm_data->c8320_runtime_resume = false;
		}
		break;
	case USB_DEVICE_REMOVE:
		if (linkpm_known_device(desc) == MIF_MAIN_DEVICE) {
			pm_data->hdev = NULL;
			pm_data->udev = NULL;

			mif_info("unhook: (%pf, %pf), (%pf, %pf)\n",
					_usb_resume, udriver->resume,
					_usb_suspend,	udriver->suspend);
			spin_lock_irqsave(&pm_data->lock, flags);
			if (_usb_resume) {
				udriver->resume = _usb_resume;
				_usb_resume = NULL;
			}
			if (_usb_suspend) {
				udriver->suspend = _usb_suspend;
				_usb_suspend = NULL;
			}
			spin_unlock_irqrestore(&pm_data->lock, flags);
			pm_data->c8320_runtime_resume = false;
		}
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}
/*
 * linux system suspend or resume notifier event
 */
static int link_pm_notifier_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	struct link_pm_data *pm_data =
			container_of(this, struct link_pm_data,	pm_notifier);
	struct modem_ctl *mc = if_usb_get_modemctl(pm_data);
	int value = 0;

	if (!pm_data->usb_ld->if_usb_connected
					|| c8320_modem_ctl_ex.com_state!= COM_ONLINE) {
		mif_info("HISC not connected, skip!\n");
		return NOTIFY_DONE;
	}

	switch (event) {
	case PM_SUSPEND_PREPARE:
#ifdef CONFIG_HIBERNATION
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
#endif
		
		pm_data->dpm_suspending = true;
		if(wake_lock_active(&pm_data->l2_wake)){
			return NOTIFY_OK;
		}
		if (get_hostactive(pm_data)){
			if((gpio_get_value(pm_data->gpio_link_slavewake)) || get_hostwake(pm_data)){
				mif_info("has resume progress,not set active 0\n");
			}else{
				mif_debug("set hostactive 0\n");
				gpio_set_value(pm_data->gpio_link_active, 0);
				wake_lock(&pm_data->rpm_wake);
			}

		}
		
		c8320_modem_ctl_ex.cmd_send = true;
		c8320_modem_ctl_ex.debugmask = c8320_modem_ctl_ex.debugmask| FORBID_SWITCH_USBDEV;

		return NOTIFY_OK;
	case PM_POST_SUSPEND:
#ifdef CONFIG_HIBERNATION
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
#endif
		pm_data->dpm_suspending = false;
		c8320_modem_ctl_ex.debugmask = c8320_modem_ctl_ex.debugmask| FORBID_SWITCH_USBDEV;
		/* LPA to Kernel suspend and User Freezing task fail resume,
		restore to LPA GPIO states. */
		pm_data->usb_ld->debug_pending = 0;
		value = gpio_get_value(pm_data->gpio_link_hostwake);
		if (value == HOSTWAKE_TRIGLEVEL) {
			queue_delayed_work(pm_data->wq, &pm_data->link_pm_work,	msecs_to_jiffies(100));
		}
		mif_debug("dpm suspending set to false\n");
		debug_ehci_reg_dump();
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct modem_ctl *if_usb_get_modemctl(struct link_pm_data *pm_data)
{
	struct io_device *iod;

	iod = link_get_iod_with_idex(&pm_data->usb_ld->ld, 1);
	if (!iod) {
		mif_err("no iodevice for modem control\n");
		return NULL;
	}

	return iod->mc;
}

static int mif_unlink_urbs(struct if_usb_devdata *pipe_data,
							struct sk_buff_head *q)
{
	unsigned long flags;
	int count = 0;

	spin_lock_irqsave(&q->lock, flags);
	if (!skb_queue_empty(q)) {
		struct sk_buff *skb;
		struct urb *urb;
		int ret = 0;
		skb_queue_walk(q, skb) {
			urb = skbpriv(skb)->urb;
			usb_get_urb(urb);
			ret = usb_unlink_urb(urb);
			if (ret != -EINPROGRESS && ret != 0)
				mif_info("unlink urb err = %d\n", ret);
			else
				count++;
			usb_put_urb(urb);
		}
	}
	spin_unlock_irqrestore(&q->lock, flags);
	return count;
}

static int if_usb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct if_usb_devdata *pipe_data = usb_get_intfdata(intf);
	struct usb_link_device *usb_ld = pipe_data->usb_ld;
	struct link_pm_data *pm_data = pipe_data->usb_ld->link_pm_data;

	if (!pipe_data->disconnected && pipe_data->state == STATE_RESUMED) {
		usb_kill_urb(pipe_data->urb);
		if (pipe_data->info->intr_complete && pipe_data->status)
			usb_kill_urb(pipe_data->intr_urb);
		/* release TX urbs */
		pipe_data->tx_pend = mif_unlink_urbs(pipe_data,
							&pipe_data->sk_tx_q);
		if (pipe_data->tx_pend) {
			if (pipe_data->ndev)
				netif_stop_queue(pipe_data->ndev);
			/* Mark the last flowctl start time */
			if (!pipe_data->txpend_ts.tv_sec) {
				getnstimeofday(&pipe_data->txpend_ts);
				rtc_time_to_tm(pipe_data->txpend_ts.tv_sec,
							&pipe_data->txpend_tm);
			}
			mif_info("flowctl %s CH%d(%d) (%02d:%02d:%02d.%09lu)\n",
				"suspend", pipe_data->idx, pipe_data->tx_pend,
				pipe_data->txpend_tm.tm_hour,
				pipe_data->txpend_tm.tm_min,
				pipe_data->txpend_tm.tm_sec,
				pipe_data->txpend_ts.tv_nsec);
		}
		pipe_data->state = STATE_SUSPENDED;
	}
	usb_ld->suspended++;

	if (usb_ld->suspended == /*LINKPM_DEV_NUM test*/ 5) {/* 3ACM +2NCM CYIT */

		/* Check HSIC interface error for debugging */
		if (!usb_ld->rx_cnt && !usb_ld->tx_cnt) {
			if (usb_ld->debug_pending++ > 20) {
				usb_ld->debug_pending = 0;
				sprintf(last_link_err, "No TX/RX after resume");
				mif_err("%s\n", last_link_err);
				set_bit(LINK_EVENT_RECOVERY, &usb_ld->events);
			
			}
		} else {
			usb_ld->debug_pending = 0;
			usb_ld->tx_cnt = 0;
			usb_ld->rx_cnt = 0;
		}
	}
	return 0;
}

/* usb_anchor_urb_head - add urb to anchor head
 * @urb: pointer to the urb to anchor
 * @anchor: pointer to the anchor
 *
 * If tx send fail, this add the urb to head of anchor, then it can
 * retry the tx urb after link recovery.
 */

static void usb_anchor_urb_head(struct urb *urb, struct usb_anchor *anchor)
{
	unsigned long flags;

	spin_lock_irqsave(&anchor->lock, flags);
	usb_get_urb(urb);
	list_add(&urb->anchor_list, &anchor->urb_list);
	urb->anchor = anchor;

	if (unlikely(anchor->poisoned)) {
		atomic_inc(&urb->reject);
	}
	spin_unlock_irqrestore(&anchor->lock, flags);
}

static int usb_defered_tx_purge_anchor(struct if_usb_devdata *pipe_data)
{
	struct urb *urb;
	struct sk_buff *skb;
	int cnt = 0;

	mif_info("Previous TX packet purge\n");
	while ((urb = usb_get_from_anchor(&pipe_data->tx_deferd_urbs))) {
		usb_put_urb(urb);
		skb = (struct sk_buff *)urb->context;
		//pr_tx_skb_with_format(pipe_data->iod->format, skb);
		dev_kfree_skb_any(skb);
		usb_free_urb(urb);
		cnt++;
	}
	if (cnt)
		mif_info("purge tx urb=%d(CH%d)\n", cnt, pipe_data->idx);
	return 0;
}

static int usb_defered_tx_from_anchor(struct if_usb_devdata *pipe_data)
{
	struct urb *urb;
	struct sk_buff *skb;
	int cnt = 0;
	int ret = 0;
	unsigned long flag;
	bool refrash = true;

	while ((urb = usb_get_from_anchor(&pipe_data->tx_deferd_urbs))) {
		usb_put_urb(urb);
		if (!pipe_data->usb_ld->if_usb_connected) {
			usb_anchor_urb_head(urb, &pipe_data->tx_deferd_urbs);
			ret = -ENODEV;
			goto exit;
		}
		skb = (struct sk_buff *)urb->context;
		skb_queue_tail(&pipe_data->sk_tx_q, skb);
		if (refrash) {
			usb_fill_bulk_urb(urb, pipe_data->usbdev,
				pipe_data->tx_pipe, skb->data, skb->len,
				usb_tx_complete, (void *)skb);
		}

		ret = usb_submit_urb(urb, GFP_ATOMIC);
		if (ret < 0) {
			/* TODO: deferd TX again */
			mif_err("resume deferd TX fail(%d)\n", ret);
			spin_lock_irqsave(&pipe_data->sk_tx_q.lock, flag);
			__skb_unlink(skb, &pipe_data->sk_tx_q);
			spin_unlock_irqrestore(&pipe_data->sk_tx_q.lock, flag);
			usb_anchor_urb_head(urb, &pipe_data->tx_deferd_urbs);
			goto exit;
		}
		cnt++;
	}
exit:
	if (cnt)
		mif_info("deferd tx urb=%d(CH%d)\n", cnt, pipe_data->idx);
	return ret;
}
static int usb_resume_count = 0;
#define EXYNOS_USB_CFG_1		(S3C_VA_SYS + 0x21C)
static int if_usb_resume(struct usb_interface *intf)
{
	int ret;
	struct if_usb_devdata *pipe_data = usb_get_intfdata(intf);
	struct link_pm_data *pm_data = pipe_data->usb_ld->link_pm_data;
	struct usb_device *udev = interface_to_usbdev(intf);
	int dev_index = 0;
	

	if (pipe_data->state != STATE_SUSPENDED) {
		mif_debug("aleady resume!\n");
		goto done;
	}
	dev_index = intf->altsetting->desc.bInterfaceNumber / 2;
	/* Submit interrupt_in control RX urbs */
	if (pipe_data->info->intr_complete && pipe_data->status) {
		ret = usb_submit_urb(pipe_data->intr_urb, GFP_NOIO);
		if (ret < 0) {
			mif_err("control rx submit failed(%d)\n", ret);
			return ret;
		}
	}
	pipe_data->state = STATE_RESUMED;
	/* Send defferd TX urbs */
	ret = usb_defered_tx_from_anchor(pipe_data);
	if (ret < 0)
		goto resume_exit;

	/* Submit bulk-in data RX urbs */
	ret = usb_rx_submit(pipe_data->usb_ld, pipe_data, GFP_ATOMIC);
	if (ret < 0) {
		mif_err("usb_rx_submit error with (%d)\n", ret);
		return ret;
	}
	mif_net_resume(pipe_data, MIF_NET_SUSPEND_LINK_WAKE);
	
	if((dev_index == pipe_data->usb_ld->max_link_ch -1)&&(c8320_modem_ctl_ex.cmd_send == true)){
		
		usbnet_set_multicast(&pipe_data->usb_ld->ld, pipe_data->iod, 0);
	}
done:
	pm_data->resume_retry_cnt = 0;

	pipe_data->usb_ld->suspended--;
	if (!pipe_data->usb_ld->suspended) {
		wake_lock(&pm_data->l2_wake);
	}

	if(++usb_resume_count == 10){
		debug_ehci_reg_dump();
		usb_resume_count = 0;
	}
	c8320_modem_ctl_ex.debugmask = c8320_modem_ctl_ex.debugmask & ~FORBID_SWITCH_USBDEV;
	sc3_set_phy2dev();
	return 0;

resume_exit:
	return ret;
}

static int if_usb_reset_resume(struct usb_interface *intf)
{
	int ret;
	struct if_usb_devdata *pipe_data = usb_get_intfdata(intf);
	struct link_pm_data *pm_data = pipe_data->usb_ld->link_pm_data;

	ret = if_usb_resume(intf);
	pipe_data->usb_ld->debug_pending = 0;
	/* for runtime suspend, kick runtime pm at L3 -> L0 reset resume */
	if (!pipe_data->usb_ld->suspended)
		queue_delayed_work(pm_data->wq, &pm_data->link_pm_start, 0);

	return ret;
}

static void if_usb_disconnect(struct usb_interface *intf)
{
	struct if_usb_devdata *pipe_data = usb_get_intfdata(intf);
	struct link_pm_data *pm_data;
	struct usb_device *udev = interface_to_usbdev(intf);

	if (!pipe_data || pipe_data->disconnected)
		return;

	printk("Called %s func\n", __func__);

	pm_data = pipe_data->usb_ld->link_pm_data;
	pipe_data->usb_ld->if_usb_connected = 0;

	cancel_delayed_work_sync(&pipe_data->rx_defered_work);

	if (pipe_data->info->unbind) {
		mif_info("unbind(%pf)\n", pipe_data->info->unbind);
		pipe_data->info->unbind(pipe_data, intf);
	}
	usb_kill_urb(pipe_data->urb);
	/* TODO: kill interrupt_in urb */
	if (pipe_data->info->intr_complete && pipe_data->status) {
		usb_kill_urb(pipe_data->intr_urb);
		usb_free_urb(pipe_data->intr_urb);
	}

	mif_debug("put dev 0x%p\n", udev);
	usb_put_dev(udev);

	pipe_data->usb_ld->suspended = 0;

	/* cancel runtime start delayed works */
	cancel_delayed_work_sync(&pm_data->link_pm_start);

	/* if reconnect function exist , try reconnect without reset modem
	 * reconnect function checks modem is under crash or not, so we don't
	 * need check crash state here. reconnect work checks and determine
	 * further works
	 */
	if (!pm_data->link_reconnect)
		return;

	if (c8320_modem_ctl_ex.com_state != COM_ONLINE) {
		cancel_delayed_work(&pm_data->link_reconnect_work);
		return;
	} else
		schedule_delayed_work(&pm_data->link_reconnect_work,
							msecs_to_jiffies(500));
	return;
}

static int c8320_acm_bind(struct if_usb_devdata *pipe_data,
		struct usb_interface *intf, struct usb_link_device *usb_ld)
{
	int ret;
	const struct usb_cdc_union_desc *union_hdr = NULL;
	const struct usb_host_interface *data_desc;
	unsigned char *buf = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *data_intf = NULL;
	struct usb_interface *control_intf;
	struct usb_device *usbdev = interface_to_usbdev(intf);
	struct usb_driver *usbdrv = to_usb_driver(intf->dev.driver);
	
	if (!buflen) {
		if (intf->cur_altsetting->endpoint->extralen &&
				    intf->cur_altsetting->endpoint->extra) {
			buflen = intf->cur_altsetting->endpoint->extralen;
			buf = intf->cur_altsetting->endpoint->extra;
		} else {
			data_desc = intf->cur_altsetting;
			if (!data_desc) {
				mif_err("data_desc is NULL\n");
				return -EINVAL;
			}
			mif_err("cdc-data desc - XMM6360 bootrom\n");
			goto found_data_desc;
		}
	}

	while (buflen > 0) {
		if (buf[1] == USB_DT_CS_INTERFACE) {
			switch (buf[2]) {
			case USB_CDC_UNION_TYPE:
				if (union_hdr)
					break;
				union_hdr = (struct usb_cdc_union_desc *)buf;
				break;
			default:
				break;
			}
		}
		buf += buf[0];
		buflen -= buf[0];
	}

	if (!union_hdr) {
		mif_err("USB CDC is not union type\n");
		return -EINVAL;
	}

	control_intf = usb_ifnum_to_if(usbdev, union_hdr->bMasterInterface0);
	if (!control_intf) {
		mif_err("control_inferface is NULL\n");
		return -ENODEV;
	}

	pipe_data->status = control_intf->altsetting->endpoint;
	if (!usb_endpoint_dir_in(&pipe_data->status->desc)) {
		mif_err("not initerrupt_in ep\n");
		pipe_data->status = NULL;
	}

	data_intf = usb_ifnum_to_if(usbdev, union_hdr->bSlaveInterface0);
	if (!data_intf) {
		mif_err("data_inferface is NULL\n");
		return -ENODEV;
	}

	data_desc = data_intf->altsetting;
	if (!data_desc) {
		mif_err("data_desc is NULL\n");
		return -ENODEV;
	}

found_data_desc:
	/* if_usb_set_pipe */
	if ((usb_pipein(data_desc->endpoint[0].desc.bEndpointAddress)) &&
	    (usb_pipeout(data_desc->endpoint[1].desc.bEndpointAddress))) {
		pipe_data->rx_pipe = usb_rcvbulkpipe(usbdev,
				data_desc->endpoint[0].desc.bEndpointAddress);
		pipe_data->tx_pipe = usb_sndbulkpipe(usbdev,
				data_desc->endpoint[1].desc.bEndpointAddress);
	} else if ((usb_pipeout(data_desc->endpoint[0].desc.bEndpointAddress))
		&& (usb_pipein(data_desc->endpoint[1].desc.bEndpointAddress))) {
		pipe_data->rx_pipe = usb_rcvbulkpipe(usbdev,
				data_desc->endpoint[1].desc.bEndpointAddress);
		pipe_data->tx_pipe = usb_sndbulkpipe(usbdev,
				data_desc->endpoint[0].desc.bEndpointAddress);
	} else {
		mif_err("undefined endpoint\n");
		return -EINVAL;
	}

	mif_debug("EP tx:%x, rx:%x\n",
		data_desc->endpoint[0].desc.bEndpointAddress,
		data_desc->endpoint[1].desc.bEndpointAddress);

	pipe_data->usbdev = usb_get_dev(usbdev);
	pipe_data->usb_ld = usb_ld;
	pipe_data->data_intf = data_intf;
	pipe_data->disconnected = 0;
	pipe_data->state = STATE_RESUMED;

	if (data_intf) {
		ret = usb_driver_claim_interface(usbdrv, data_intf,
							(void *)pipe_data);
		if (ret < 0) {
			mif_err("usb_driver_claim() failed\n");
			return ret;
		}
	}
	usb_set_intfdata(intf, (void *)pipe_data);

	return 0;
}

static void c8320_acm_unbind(struct if_usb_devdata *pipe_data,
	struct usb_interface *intf)
{
	usb_driver_release_interface(to_usb_driver(intf->dev.driver), intf);
	pipe_data->data_intf = NULL;
	pipe_data->usbdev = NULL;
	pipe_data->disconnected = 1;
	pipe_data->state = STATE_SUSPENDED;

	usb_set_intfdata(intf, NULL);
	return;
}

void cdc_acm_intr_complete(struct urb *urb)
{
	int ret;

	mif_debug("status = %d\n", urb->status);

	switch (urb->status) {
	/* success */
	case -ENOENT:		/* urb killed by L2 suspend */
	case 0:
		break;
	case -ESHUTDOWN:	/* hardware gone */
		mif_err("intr shutdown, code %d, ep = %d\n", urb->status,
						usb_pipeendpoint(urb->pipe));
		return;

	/* NOTE:  not throttling like RX/TX, since this endpoint
	 * already polls infrequently
	 */
	default:
		mif_err("intr status %d, ep = %d\n", urb->status,
						usb_pipeendpoint(urb->pipe));
		break;
	}

	if (!urb->status) { /*skip -ENOENT L2 enter status */
		memset(urb->transfer_buffer, 0, urb->transfer_buffer_length);
		ret = usb_submit_urb(urb, GFP_ATOMIC);
		mif_debug("status: usb_submit_urb ret=%d\n", ret);
		if (ret != 0)
			mif_err("intr resubmit --> %d\n", ret);
	}
}

static int __devinit if_usb_probe(struct usb_interface *intf,
					const struct usb_device_id *id)
{
	int ret = 0;
	int dev_index = 0;
	struct if_usb_devdata *pipe_data;
	int subclass = intf->altsetting->desc.bInterfaceSubClass;
	struct usb_device *usbdev = interface_to_usbdev(intf);
	struct usb_id_info *info = (struct usb_id_info *)id->driver_info;
	struct usb_link_device *usb_ld = info->usb_ld;
	const char			*name;
	
	pr_debug("%s: Class=%d, SubClass=%d, Protocol=%d\n", __func__,
		intf->altsetting->desc.bInterfaceClass,
		intf->altsetting->desc.bInterfaceSubClass,
		intf->altsetting->desc.bInterfaceProtocol);

	/* if usb disconnected, AP try to reconnect 5 times.
	 * but because if_sub_connected is configured
	 * at the end of if_usb_probe, there was a chance
	 * that swk will be called again during enumeration.
	 * so.. cancel reconnect work_queue in this case. */
	if (c8320_modem_ctl_ex.com_state == COM_ONLINE)
		cancel_delayed_work(&usb_ld->link_pm_data->link_reconnect_work);
	name = intf->dev.driver->name;
	usb_ld->usbdev = usbdev;
	pm_runtime_forbid(&usbdev->dev);
	usb_ld->link_pm_data->link_pm_active = false;
	usb_ld->link_pm_data->dpm_suspending = false;

	switch (subclass) {
	case USB_CDC_SUBCLASS_NCM:
		dev_index = intf->altsetting->desc.bInterfaceNumber / 2;
		if (dev_index >= usb_ld->max_link_ch) {
			mif_err("Over the NCM device Max number(%d/%d)\n",
				dev_index, usb_ld->max_link_ch);
			return -EINVAL;
		}
		pipe_data = &usb_ld->devdata[dev_index];
		pipe_data->format = IPC_RAW_NCM;
		pipe_data->idx = dev_index;
		pipe_data->net_suspend = 0;
		pipe_data->net_connected = false;
		/* cool_zhao cannot understand red-back trees can we need modify it?*/

		
		pipe_data->iod = link_get_iod_with_idex(&usb_ld->ld,dev_index);
		if(pipe_data->iod == NULL){
			mif_info("%s:err iod is NULL\n",__func__);
		}
		break;

	case USB_CDC_SUBCLASS_ACM:
	default:
		/* when use project sc3 not go here */
		if (info->intf_id == BOOT_DOWN) {
			
			dev_index = 0;
			pipe_data = &usb_ld->devdata[dev_index];
			pipe_data->idx = dev_index;
				/* cool_zhao cannot understand red-back trees can we need modify it?*/
			pipe_data->iod =
				link_get_iod_with_format(&usb_ld->ld, IPC_BOOT);
			pipe_data->rx_buf_size = (16 * 1024);
		} else {
				
		/* sc3 go here cool_zhao */
			dev_index = intf->altsetting->desc.bInterfaceNumber / 2;
			pipe_data = &usb_ld->devdata[dev_index];
			pipe_data->idx = dev_index;
			pipe_data->format = dev_index;
			/* cool_zhao cannot understand red-back trees can we need modify it?*/

			pipe_data->iod = link_get_iod_with_idex(&usb_ld->ld,dev_index);
			if(pipe_data->iod == NULL){
				mif_err("ACM interferce not has iodevs to mapped\n");
				goto	error_exit;
			}
			/* here we define buffer_size here according as ioddev */
			pipe_data->rx_buf_size = (0xE00); /* 3.5KB */
		}
		if (dev_index >= usb_ld->max_acm_ch) {
			mif_err("Over the data ch Max number(%d/%d)\n",
						dev_index, usb_ld->max_acm_ch);
			return -EINVAL;
		}
		break;
	}
	/* pipe_data is not exit before switch */	
	pipe_data->info = info;
	pipe_data->driver_name = name;
	if (info->bind) {	
	
		ret = info->bind(pipe_data, intf, usb_ld);
		if (ret < 0) {
			mif_err("SubClass bind(%pf) err=%d\n", info->bind, ret);
			goto error_exit;
		}
		

		skb_queue_purge(&pipe_data->free_rx_q);
		ret = usb_init_rx_skb_pool(pipe_data);
		if (ret < 0)
			goto error_exit;

		ret = usb_rx_submit(usb_ld, pipe_data, GFP_ATOMIC);
		if (ret < 0)
			goto error_exit;
	} else {
		mif_err("SubClass bind func was not defined\n");
		ret = -EINVAL;
		goto error_exit;
	}

	/* For CDC control interface interrupt in endpoint RX */
	if (info->intr_complete && pipe_data->status) {
		ret = init_status(pipe_data, intf);
		if (ret < 0) {
			mif_err("Control interface EP setup fail=(%d)\n", ret);
			goto error_exit;
		}
		ret = usb_submit_urb(pipe_data->intr_urb, GFP_KERNEL);
		if (ret < 0)
			goto error_exit;
	}

	usb_ld->suspended = 0;
	pm_suspend_ignore_children(&usbdev->dev, true);

	switch (info->intf_id) {
	case BOOT_DOWN:
		usb_defered_tx_purge_anchor(pipe_data);
		usb_ld->if_usb_connected = 1;
		usb_ld->debug_pending = 0;
		usb_ld->rx_err = 0;
		usb_ld->tx_err = 0;
		break;

	case IPC_CHANNEL:
	
		/* HSIC main comm channel has been established */

		if (dev_index == usb_ld->max_link_ch - 1) 
		{
			if (!work_pending(&usb_ld->link_pm_data->link_pm_start.work)) {
#ifdef CONFIG_LINK_DEVICE_POWER_MANGER
				queue_delayed_work(usb_ld->link_pm_data->wq,
							&usb_ld->link_pm_data->link_pm_start,
							msecs_to_jiffies(2000));
#endif
				wake_lock(&usb_ld->link_pm_data->l2_wake);
			}
			c8320_modem_ctl_ex.com_state = COM_ONLINE;
			link_pm_change_modem_state(usb_ld, STATE_ONLINE);
			usb_ld->if_usb_connected = 1;
		}
		/* send defered IPC after reconnect */
		usb_defered_tx_from_anchor(pipe_data);
		break;
	default:
		mif_err("undefined interface value(0x%x)\n", info->intf_id);
		break;
	}
	if(subclass == USB_CDC_SUBCLASS_NCM){
		netif_device_attach (pipe_data->iod->ndev);
	}
	mif_info("successfully done\n");

	return 0;

error_exit:
	return ret;
}

/* Vendor specific wrapper functions */
static int c8320_cdc_ncm_bind(struct if_usb_devdata *pipe_data,
		struct usb_interface *intf, struct usb_link_device *usb_ld)
{
	mif_info("%s:start\n",__func__);
	pipe_data->iod->ipc_version = NO_SIPC_VER;	
	return cdc_ncm_bind(pipe_data, intf, usb_ld);
}

static struct usb_id_info c8320_cdc_acm_info = {/*Current only use xmm626x function interface*/
	.description = "IMC IPC CDC-ACM",
	.intf_id = IPC_CHANNEL,
	.bind = c8320_acm_bind,
	.unbind = c8320_acm_unbind,
	.intr_complete = cdc_acm_intr_complete,
};

static struct usb_id_info c8320_cdc_ncm_info = {/*Current only use xmm626x function interface*/
	.description = "IMC IPC CDC-NCM",
	.intf_id = IPC_CHANNEL,
	.bind = c8320_cdc_ncm_bind,
	.unbind = cdc_ncm_unbind,
	.tx_fixup = cdc_ncm_tx_fixup,
	.rx_fixup = cdc_ncm_rx_fixup,
	.intr_complete = cdc_ncm_intr_complete,
};

static struct usb_device_id if_usb_ids[] = {
	{ USB_DEVICE_AND_INTERFACE_INFO(0x231E, 0x6304, USB_CLASS_COMM,
		USB_CDC_SUBCLASS_ACM, USB_CDC_ACM_PROTO_AT_V25TER),
	.driver_info = (unsigned long)&c8320_cdc_acm_info,
	}, /* CYIT C8320 ACM MAIN */
	
	{ USB_DEVICE_AND_INTERFACE_INFO(0x231E, 0x6304,	USB_CLASS_COMM,
		USB_CDC_SUBCLASS_NCM, USB_CDC_PROTO_NONE),
	.driver_info = (unsigned long)&c8320_cdc_ncm_info,
	}, /* CYIT C8320 MAIN NCM */

	{}
};
MODULE_DEVICE_TABLE(usb, if_usb_ids);

static struct usb_driver if_usb_driver = {
	.name =		"cdc_modem",
	.probe =	if_usb_probe,
	.disconnect =	if_usb_disconnect,
	.id_table =	if_usb_ids,
	.suspend =	if_usb_suspend,
	.resume =	if_usb_resume,
	.reset_resume =	if_usb_reset_resume,
	.supports_autosuspend = 1,
};

static int if_usb_init(struct link_device *ld)
{
	int ret;
	int i;
	struct usb_link_device *usb_ld = to_usb_link_device(ld);
	struct if_usb_devdata *pipe_data;
	struct usb_id_info *id_info;

	/* to connect usb link device with usb interface driver */
	for (i = 0; i < ARRAY_SIZE(if_usb_ids); i++) {
		id_info = (struct usb_id_info *)if_usb_ids[i].driver_info;
		if (id_info)
			id_info->usb_ld = usb_ld;
	}

	ret = usb_register(&if_usb_driver);
	if (ret) {
		mif_err("usb_register_driver() fail : %d\n", ret);
		return ret;
	}

	/* common devdata initialize */
	for (i = 0; i < usb_ld->max_link_ch; i++) {
		pipe_data = &usb_ld->devdata[i];
		pipe_data->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!pipe_data->urb) {
			mif_err("alloc urb fail\n");
			return -ENOMEM;
		}
		pipe_data->usb_ld = usb_ld;
		skb_queue_head_init(&pipe_data->free_rx_q);
		skb_queue_head_init(&pipe_data->sk_tx_q);
		init_usb_anchor(&pipe_data->tx_deferd_urbs);
		INIT_DELAYED_WORK(&pipe_data->rx_defered_work,
			usb_defered_work);
	}
	ld->enable_dm = enable_c8320_dm;


	mif_info("if_usb_init() done : %d, usb_ld (0x%p)\n", ret, usb_ld);
	return 0;
}

static int usb_link_pm_init(struct usb_link_device *usb_ld, void *data)
{
	int ret;
	struct platform_device *pdev = (struct platform_device *)data;
	struct modem_data *pdata =
			(struct modem_data *)pdev->dev.platform_data;
	struct modemlink_pm_data *pm_pdata = pdata->link_pm_data;
	struct link_pm_data *pm_data =
			kzalloc(sizeof(struct link_pm_data), GFP_KERNEL);
	if (!pm_data) {
		mif_err("link_pm_data is NULL\n");
		return -ENOMEM;
	}
	/* get link pm data from modemcontrol's platform data */
	pm_data->gpio_link_active = pm_pdata->gpio_link_active;
	pm_data->gpio_link_enable = pm_pdata->gpio_link_enable;
	pm_data->gpio_link_hostwake = pm_pdata->gpio_link_hostwake;
	pm_data->gpio_link_slavewake = pm_pdata->gpio_link_slavewake;
	pm_data->gpio_link_suspend_req = pm_pdata->gpio_link_suspend_req;
	pm_data->irq_link_hostwake = gpio_to_irq(pm_data->gpio_link_hostwake);
	pm_data->irq_link_suspend_req = gpio_to_irq(pm_data->gpio_link_suspend_req);
	pm_data->link_ldo_enable = pm_pdata->link_ldo_enable;
	pm_data->link_reconnect = pm_pdata->link_reconnect;
	pm_data->wait_cp_connect = pm_pdata->wait_cp_resume;

	pm_data->usb_ld = usb_ld;
	pm_data->link_pm_active = false;
	pm_data->disable_recovery = false;
	pm_data->roothub_resume_req = false;
	pm_data->apinit_l0_req = false;
	/* cool_zhao pm_data do it slonely, it connection whit usb_ld is only pointer */
	usb_ld->link_pm_data = pm_data;

	pm_data->miscdev.minor = MISC_DYNAMIC_MINOR;
	pm_data->miscdev.name = "link_pm";
	pm_data->miscdev.fops = &link_pm_fops;
	pm_data->rpm_suspending_cnt = 0;

	ret = misc_register(&pm_data->miscdev);
	if (ret < 0) {
		mif_err("fail to register pm device(%d)\n", ret);
		goto err_misc_register;
	}
#ifdef CONFIG_PM_RUNTIME
/* mask irq for proble success */
#ifdef CONFIG_LINK_DEVICE_POWER_MANGER 
	ret = request_irq(pm_data->irq_link_hostwake, link_pm_irq_handler,
		IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		"hostwake", (void *)pm_data);
	if (ret) {
		mif_err("fail to request irq(%d)\n", ret);
		goto err_request_irq;
	}

	ret = enable_irq_wake(pm_data->irq_link_hostwake);
	if (ret) {
		mif_err("failed to enable_irq_wake:%d\n", ret);
		goto err_set_wake_irq;
	}



	ret = enable_irq_wake(pm_data->irq_link_hostwake);
	if (ret) {
		mif_err("failed to enable_irq_wake:%d\n", ret);
		goto err_set_wake_irq;
	}

	ret = request_irq(pm_data->irq_link_suspend_req, link_pm_suspend_irq_handler,
		IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		"hostwake", (void *)pm_data);
	if (ret) {
		mif_err("fail to suspend request irq(%d)\n", ret);
		goto err_request_irq;
	}

	ret = enable_irq_wake(pm_data->irq_link_suspend_req);
	if (ret) {
		mif_err("failed to enable_irq_wake:%d\n", ret);
		goto err_set_wake_irq;
	}
#endif 
	/* create work queue & init work for runtime pm */
	pm_data->wq = create_singlethread_workqueue("linkpmd");
	if (!pm_data->wq) {
		mif_err("fail to create wq\n");
		goto err_create_wq;
	}

	INIT_DELAYED_WORK(&pm_data->link_pm_work, link_pm_runtime_work);
	INIT_DELAYED_WORK(&pm_data->link_pm_start, link_pm_runtime_start);
	INIT_DELAYED_WORK(&pm_data->link_reconnect_work,
						link_pm_reconnect_work);
#endif
	pm_data->usb_notifier.notifier_call = link_usb_notifier_event;
	usb_register_notify(&pm_data->usb_notifier);

	pm_data->pm_notifier.notifier_call = link_pm_notifier_event;
	register_pm_notifier(&pm_data->pm_notifier);

	spin_lock_init(&pm_data->lock);
	INIT_LIST_HEAD(&c8320_devices.pm_data);
	spin_lock_init(&c8320_devices.lock);
	spin_lock_bh(&c8320_devices.lock);
	list_add(&pm_data->link, &c8320_devices.pm_data);
	spin_unlock_bh(&c8320_devices.lock);

	wake_lock_init(&pm_data->l2_wake, WAKE_LOCK_SUSPEND, "l2_hsic");
	wake_lock_init(&pm_data->rpm_wake, WAKE_LOCK_SUSPEND, "rpm_hsic");
	/* here is debug for evt  not need*/
	wake_lock(&pm_data->l2_wake);

#if defined(CONFIG_SLP)
	device_init_wakeup(pm_data->miscdev.this_device, true);
#endif

	return 0;

err_create_wq:
	disable_irq_wake(pm_data->irq_link_hostwake);
err_set_wake_irq:
	free_irq(pm_data->irq_link_hostwake, (void *)pm_data);
err_request_irq:
	misc_deregister(&pm_data->miscdev);
err_misc_register:
	kfree(pm_data);
	return ret;
}

/* Export private data symbol for ramdump debugging */
static struct usb_link_device *g_mif_usbld;

struct link_device *hsic_create_link_device(void *data)
{
	int ret;
	struct modem_data *pdata;
	struct platform_device *pdev = (struct platform_device *)data;
	struct usb_link_device *usb_ld;
	struct link_device *ld;
	
	pdata = pdev->dev.platform_data;

	usb_ld = kzalloc(sizeof(struct usb_link_device), GFP_KERNEL);
	if (!usb_ld) {
		mif_err("get usb_ld fail -ENOMEM\n");
		return NULL;
	}
	g_mif_usbld = usb_ld;

#define C8320_ACM_NUM 3
#define C8320_NCM_NUM 2
	usb_ld->max_acm_ch = pdata->max_acm_channel ?: C8320_ACM_NUM;
	usb_ld->max_link_ch = pdata->max_link_channel ?: (C8320_ACM_NUM +C8320_NCM_NUM);
	usb_ld->devdata = kzalloc(
		usb_ld->max_link_ch * sizeof(struct if_usb_devdata),
		GFP_KERNEL);
	if (!usb_ld->devdata) {
		mif_err("get devdata fail -ENOMEM\n");
		goto error;
	}

	INIT_LIST_HEAD(&usb_ld->ld.list);

	ld = &usb_ld->ld;

	ld->name = "usb";
	ld->init_comm = usb_init_communication;
	ld->terminate_comm = usb_terminate_communication;
	ld->send = usb_send;
	c8320_modem_ctl_ex.com_state = COM_NONE;
	ld->raw_tx_suspended = false;
	init_completion(&ld->raw_tx_resumed_by_cp);

	ld->tx_wq = create_singlethread_workqueue("usb_tx_wq");
	if (!ld->tx_wq) {
		mif_err("fail to create work Q.\n");
		goto error;
	}
	ld->rx_wq = create_singlethread_workqueue("usb_rx_wq");
	if (!ld->rx_wq) {
		mif_err("fail to create work Q.\n");
		goto error;
	}

	/* create link pm device */
	ret = usb_link_pm_init(usb_ld, data);
	if (ret)
		goto error;

	ret = if_usb_init(ld);
	if (ret)
		goto error;

	mif_info("%s : create_link_device DONE\n", usb_ld->ld.name);
	return (void *)ld;
error:
	kfree(usb_ld->devdata);
	kfree(usb_ld);
	return NULL;
}

static void __exit if_usb_exit(void)
{
	usb_deregister(&if_usb_driver);
}
