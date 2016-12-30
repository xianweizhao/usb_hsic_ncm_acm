/* /linux/drivers/misc/modem_if/sipc5_io_device.c
 *
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
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rtc.h>

#include <linux/platform_data/modem.h>
#ifdef CONFIG_LINK_DEVICE_C2C
#include <linux/platform_data/c2c.h>
#endif
#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_link_device_hsic_ncm.h"

enum iod_debug_flag_bit {
	IOD_DEBUG_IPC_LOOPBACK,
};

static unsigned long dbg_flags = 0;
module_param(dbg_flags, ulong, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(dbg_flags, "sipc iodevice debug flags\n");

static int fd_waketime = (6 * HZ);
module_param(fd_waketime, int, S_IRUGO);
MODULE_PARM_DESC(fd_waketime, "fd wake lock timeout");

static ssize_t show_waketime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int msec;
	char *p = buf;

	msec = jiffies_to_msecs(fd_waketime);

	p += sprintf(buf, "raw waketime : %ums\n", msec);

	return p - buf;
}

static ssize_t store_waketime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long msec;
	int ret;
	struct miscdevice *miscdev = dev_get_drvdata(dev);
	struct io_device *iod = container_of(miscdev, struct io_device,
			miscdev);

	ret = kstrtoul(buf, 10, &msec);
	if (ret)
		return count;

	iod->waketime = msecs_to_jiffies(msec);
	fd_waketime = msecs_to_jiffies(msec);

	return count;
}

static struct device_attribute attr_waketime =
	__ATTR(waketime, S_IRUGO | S_IWUSR, show_waketime, store_waketime);

static void iodev_showtxlink(struct io_device *iod, void *args)
{
	char **p = (char **)args;
	struct link_device *ld = get_current_link(iod);

	if (iod->io_typ == IODEV_NET && IS_CONNECTED(iod, ld))
		*p += sprintf(*p, "%s: %s\n", iod->name, ld->name);
}

static ssize_t show_txlink(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct miscdevice *miscdev = dev_get_drvdata(dev);
	struct modem_shared *msd =
		container_of(miscdev, struct io_device, miscdev)->msd;
	char *p = buf;

	iodevs_for_each(msd, iodev_showtxlink, &p);

	return p - buf;
}

static ssize_t store_txlink(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	/* don't change without gpio dynamic switching */
	return -EINVAL;
}

static struct device_attribute attr_txlink =
	__ATTR(txlink, S_IRUGO | S_IWUSR, show_txlink, store_txlink);

static int rx_multi_pdp(struct sk_buff *skb)
{
	struct io_device *iod = skbpriv(skb)->iod; /* same with real_iod */
	struct net_device *ndev;
	struct iphdr *iphdr;
	struct ethhdr *ehdr;
	int ret;
	const char source[ETH_ALEN] = SOURCE_MAC_ADDR;

	ndev = iod->ndev;
	if (!ndev) {
		mif_info("%s: ERR! no iod->ndev\n", iod->name);
		return -ENODEV;
	}
/*  CONFIG_LINK_ETHERNET */
	skb->protocol = eth_type_trans(skb, ndev);
/*********************************/

	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += skb->len;


	if (in_interrupt())
		ret = netif_rx(skb);
	else
		ret = netif_rx_ni(skb);

	if (ret != NET_RX_SUCCESS)
		mif_info("%s: ERR! netif_rx fail (err %d)\n", iod->name, ret);

	return ret;
}

/* called from link device when a packet arrives for this io device */
static int io_dev_recv_skb_from_link_dev(struct io_device *iod,
		struct link_device *ld, struct sk_buff *skb_in)
{
	struct sk_buff *skb = NULL;
	int err = 0;
	
	if (!skb_in) {
		mif_info("%s: ERR! !data\n", ld->name);
		return -EINVAL;
	}

	if (skb_in->len <= 0) {
		mif_info("%s: ERR! len %d <= 0\n", ld->name, skb_in->len);
		return -EINVAL;
	}
	
	switch (iod->format) {
	case IPC_FMT:
	case IPC_RAW:
		if (iod->waketime)
			wake_lock_timeout(&iod->wakelock, iod->waketime);
		
		skb = skb_clone(skb_in, GFP_ATOMIC);
		if (unlikely(!skb)) {
			mif_err("boot skb clone fail\n");
			return -ENOMEM;
		}
		skbpriv(skb)->ld = ld;
		skb_queue_tail(&iod->sk_rx_q, skb);
		wake_up(&iod->wq);
		return err;
	case IPC_RAW_NCM:
		printk("%s: recive net data\n",__func__);
		if (fd_waketime)
			wake_lock_timeout(&iod->wakelock, fd_waketime);
		skbpriv(skb_in)->ld = ld;
		skbpriv(skb_in)->iod = iod;
		skbpriv(skb_in)->real_iod = iod;
		return rx_multi_pdp(skb_in);
	default:
		mif_info("%s: ERR! unknown format %d\n", ld->name, iod->format);
		return -EINVAL;
	}
}

/* inform the IO device that the modem is now online or offline or
 * crashing or whatever...
 */
static void io_dev_modem_state_changed(struct io_device *iod,
			enum modem_state state)
{
	mif_info("%s: %s state changed (state %d)\n",
		iod->name, iod->mc->name, state);

	c8320_modem_ctl_ex.phone_state = state;
	if(iod->format != IPC_RAW){
		return;
	}

	if (state == STATE_CRASH) {
		wake_lock_timeout(&iod->wakelock, msecs_to_jiffies(2000));
		wake_up(&iod->wq);
	}
}

static void iodev_dump_status(struct io_device *iod, void *args)
{
	if (iod->format == IPC_RAW && iod->io_typ == IODEV_NET) {
		struct link_device *ld = get_current_link(iod);
		printk("%s: %s\n", iod->name, ld->name);
	}
}

static int misc_open(struct inode *inode, struct file *filp)
{
	struct io_device *iod = to_io_device(filp->private_data);
	struct modem_shared *msd = iod->msd;
	struct link_device *ld;
//	struct if_usb_devdata *pipe_data;
	
	int ret;
//	pipe_data = container_of(iod,struct if_usb_devdata,iod);
	filp->private_data = (void *)iod;

	atomic_inc(&iod->opened);
#if 0
// for IPC_NVM test and becase raw init_comm do nothing 

	list_for_each_entry(ld, &msd->link_dev_list, list) {
		if (IS_CONNECTED(iod, ld) && ld->init_comm) {
			ret = ld->init_comm(ld, iod);
			if (ret < 0) {
				mif_info("%s: init_comm fail(%d)\n",
					ld->name, ret);
				return ret;
			}
		}
	}
#endif

	mif_info("%s (opened %d)\n", iod->name, atomic_read(&iod->opened));

	return 0;
}

static int misc_release(struct inode *inode, struct file *filp)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct modem_shared *msd = iod->msd;
	struct link_device *ld;

	atomic_dec(&iod->opened);
	skb_queue_purge(&iod->sk_rx_q);

	list_for_each_entry(ld, &msd->link_dev_list, list) {
		if (IS_CONNECTED(iod, ld)) {
			if (ld->terminate_comm)
				ld->terminate_comm(ld, iod);
		}
	}

	mif_err("%s (opened %d)\n", iod->name, atomic_read(&iod->opened));

	return 0;
}

static unsigned int misc_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	if (c8320_modem_ctl_ex.phone_state == STATE_CRASH){
		//return POLLERR|POLLHUP;
		msleep(10000);
		return 0;
	}

	if (skb_queue_empty(&iod->sk_rx_q))
		poll_wait(filp, &iod->wq, wait);

	if (!skb_queue_empty(&iod->sk_rx_q)) {

		return POLLIN | POLLRDNORM;
	} else if ((c8320_modem_ctl_ex.phone_state == STATE_CRASH)){
		mif_info("io dev will hungup\n");
		//return POLLERR|POLLHUP;
		msleep(10000);
		return 0;
	} else {
		return 0;
	}
}

static long misc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int p_state;
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct link_device *ld = get_current_link(iod);
	char cpinfo_buf[530] = "CP Crash ";
	unsigned long size;
	int ret;

	switch (cmd) {

	default:
		 /* If you need to handle the ioctl for specific link device,
		  * then assign the link ioctl handler to ld->ioctl
		  * It will be call for specific link ioctl */
		if (ld->ioctl)
			return ld->ioctl(ld, iod, cmd, arg);

		mif_info("%s: ERR! cmd 0x%X not defined.\n", iod->name, cmd);
		return -EINVAL;
	}
	return 0;
}

static unsigned get_fmt_multiframe_id(void)
{
	/*TODO: get unique fmt multiframe id*/
	return 0;
}

#define ALIGN_BYTE 4
#define SIPC_ALIGN(x) ALIGN(x, ALIGN_BYTE)
#define SIPC_PADLEN(x) (ALIGN(x, ALIGN_BYTE) - x)
static ssize_t misc_write(struct file *filp, const char __user *data,
			size_t count, loff_t *fpos)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct link_device *ld = get_current_link(iod);
	struct sk_buff *skb;
	int ret;
	unsigned len, tx_size;

	/* cool_zhao format is RAW (ACM STREAMS) */
	if (iod->format != IPC_RAW )
		return -EINVAL;

	len = count;
	skb = alloc_skb(SIPC_ALIGN(len), GFP_KERNEL);
	if (!skb) {
		
		mif_err("alloc_skb fail (%s, size:%d)\n", iod->name,
							SIPC_ALIGN(len));
		return -ENOMEM;
	}

	if (copy_from_user(skb_put(skb, len), data , len) != 0) {
		if (skb)
			dev_kfree_skb_any(skb);
		return -EFAULT;
	}

	/* acm can need  aligned ? because of acm is streams  cool_zhao*/

	skbpriv(skb)->iod = iod;
	skbpriv(skb)->ld = ld;

	tx_size = skb->len;
	ret = ld->send(ld, iod, skb);
	if (ret < 0) {
		mif_err("ld->send fail (%s, err %d)\n", iod->name, ret);
		return ret;
	}

	if (ret != tx_size)
		mif_info("wrong tx size (%s, count:%d copied:%d ret:%d)\n",
			iod->name, count, tx_size, ret);
	return count;
}

static ssize_t misc_read(struct file *filp, char *buf, size_t count,
			loff_t *fpos)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct sk_buff_head *rxq = &iod->sk_rx_q;
	struct sk_buff *skb;
	unsigned  len = 0;
	unsigned  copyed = 0;
stream_char:
	skb = skb_dequeue(rxq);
	if (!skb) {
		mif_info("%s: no data in rxq\n", iod->name);
		goto exit;
	}

	len = min(skb->len, count);
	if (copy_to_user((buf+copyed), skb->data, len)) {
		mif_info("%s: ERR! copy_to_user fail\n", iod->name);
		skb_queue_head(rxq, skb);
		return -EFAULT;
	}
	copyed = copyed + len;
	count = count - len;

	skb_pull_inline(skb, len);

	mif_debug("%s: data:%d count:%d  qlen:%d\n",
		iod->name, len, count, rxq->qlen);

	if (skb->len)
		skb_queue_head(rxq, skb);
	else
		dev_kfree_skb_any(skb);

	if(iod->id == 1 || iod->id ==2){
		if(count > 0){
			goto stream_char;
		}
	}
	
exit:
	return copyed;
}

static const struct file_operations misc_io_fops = {
	.owner = THIS_MODULE,
	.open = misc_open,
	.release = misc_release,
	.poll = misc_poll,
	.unlocked_ioctl = misc_ioctl,
	.write = misc_write,
	.read = misc_read,

};

static int vnet_open(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = vnet->iod;
	struct modem_shared *msd = iod->msd;
	struct link_device *ld;
	int ret = 0;
//	struct if_usb_devdata *pipe_data;

//	pipe_data = container_of(vnet->iod,struct if_usb_devdata,iod);
	list_for_each_entry(ld, &msd->link_dev_list, list) {
		if (IS_CONNECTED(iod, ld) && ld->init_comm) {
			ret = ld->init_comm(ld, iod);
			if (ret < 0) {
				mif_info("%s: init_comm fail(%d)\n",
					ld->name, ret);
				return ret;
			}
		}
	}
	netif_start_queue(ndev);
	
	atomic_inc(&vnet->iod->opened);
	return 0;
}

static int vnet_stop(struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	mif_err("%s\n", vnet->iod->name);

	atomic_dec(&vnet->iod->opened);
	netif_stop_queue(ndev);
	skb_queue_purge(&vnet->iod->sk_rx_q);
	dump_stack();
	return 0;
}

static int vnet_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct vnet *vnet = netdev_priv(ndev);
	struct io_device *iod = vnet->iod;
	struct link_device *ld = get_current_link(iod);
	struct sk_buff *skb_new;
	int ret;
	unsigned long tx_bytes = skb->len;


	skb_new = skb;

	skbpriv(skb_new)->iod = iod;
	skbpriv(skb_new)->ld = ld;
	printk("%s:send net data\n",__func__);

	ret = ld->send(ld, iod, skb_new);
	if (ret < 0) {
		netif_stop_queue(ndev);
		mif_info("%s: ERR! ld->send fail (err %d)\n", iod->name, ret);
		return NETDEV_TX_BUSY;
	}
	
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += tx_bytes;

	return NETDEV_TX_OK;
}


static struct net_device_ops vnet_ops = {
	.ndo_open = vnet_open,
	.ndo_stop = vnet_stop,
	.ndo_start_xmit = vnet_xmit,
};

/* ehter ops for CDC-NCM */
/* CONFIG_LINK_ETHERNET */
static struct net_device_ops vnet_ether_ops = {
	.ndo_open = vnet_open,
	.ndo_stop = vnet_stop,
	.ndo_start_xmit = vnet_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
};
/****************************/


int sipc5_init_io_device(struct io_device *iod)
{
	int ret = 0;
	struct vnet *vnet;

	/* Get modem state from modem control device */
	iod->modem_state_changed = io_dev_modem_state_changed;

	/* Get data from link device */
	mif_debug("%s: SIPC version = %d\n", iod->name, iod->ipc_version);
	iod->recv_skb = io_dev_recv_skb_from_link_dev;

	/* Register misc or net device */
	switch (iod->io_typ) {
	case IODEV_MISC:
		init_waitqueue_head(&iod->wq);
		skb_queue_head_init(&iod->sk_rx_q);

		iod->miscdev.minor = MISC_DYNAMIC_MINOR;
		iod->miscdev.name = iod->name;
		iod->miscdev.fops = &misc_io_fops;

		ret = misc_register(&iod->miscdev);
		if (ret)
			mif_info("%s: ERR! misc_register failed\n", iod->name);


		break;

	case IODEV_NET:
		skb_queue_head_init(&iod->sk_rx_q);
/* CONFIG_LINK_ETHERNET	*/	
		iod->ndev = alloc_etherdev(sizeof(*vnet));
		if (!iod->ndev) {
			mif_err("failed to alloc netdev\n");
			return -ENOMEM;
		}
		iod->ndev->netdev_ops = &vnet_ether_ops;
		iod->ndev->watchdog_timeo = 5 * HZ;
		iod->ndev->tx_queue_len = 0;
/**************************************************/

		/*register_netdev parsing % */
		strcpy(iod->ndev->name, "usb%d");
/* here is for send clour message cool_zhao */
		iod->ndev->flags = IFF_NOARP|IFF_POINTOPOINT|IFF_MULTICAST;
#if 1
		mif_debug("iod 0x%p\n", iod);
		vnet = netdev_priv(iod->ndev);
		mif_debug("vnet 0x%p\n", vnet);
		vnet->iod = iod;
		ret = register_netdev(iod->ndev);
		if (ret) {
			mif_info("%s: ERR! register_netdev fail\n", iod->name);
			free_netdev(iod->ndev);
		}
#endif


		break;

	case IODEV_DUMMY:
		skb_queue_head_init(&iod->sk_rx_q);

		iod->miscdev.minor = MISC_DYNAMIC_MINOR;
		iod->miscdev.name = iod->name;
		iod->miscdev.fops = &misc_io_fops;

		ret = misc_register(&iod->miscdev);
		if (ret)
			mif_info("%s: ERR! misc_register fail\n", iod->name);
		ret = device_create_file(iod->miscdev.this_device,
					&attr_waketime);
		if (ret)
			mif_info("%s: ERR! device_create_file fail\n",
				iod->name);

		ret = device_create_file(iod->miscdev.this_device,
				&attr_txlink);
		if (ret)
			mif_err("failed to create `txlink file' : %s\n",
					iod->name);
		break;

	default:
		mif_info("%s: ERR! wrong io_type %d\n", iod->name, iod->io_typ);
		return -EINVAL;
	}

	return ret;
}

