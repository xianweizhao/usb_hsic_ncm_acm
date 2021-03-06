/* linux/drivers/modem/modem.c
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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/if_arp.h>

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/string.h>

#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include "modem_variation.h"
#include "modem_utils.h"

#define FMT_WAKE_TIME   (20*HZ)  /* CP MC iod waketime */
#define RAW_WAKE_TIME   (HZ*6)
struct c8320_modem_ctl_extern c8320_modem_ctl_ex;
/* add sys GPIO control */
static ssize_t show_active(struct device *d,
		struct device_attribute *attr, char *buf)
{
	int value = 0;
	char *p = buf;
	struct modem_data *pdata = d->platform_data;
	struct modemlink_pm_data *pmdata = pdata->link_pm_data;
	value = gpio_get_value(pmdata->gpio_link_active);
	if(value){
		p += sprintf(p, "high\n");
	}else{
		p += sprintf(p, "low\n");
	}
	return p - buf;
}

static ssize_t store_active(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct modem_data *pdata = d->platform_data;
	struct modemlink_pm_data *pmdata = pdata->link_pm_data;
	if (!strncmp(buf, "0", 1)) {
		printk("set AP>>CP:  HOSTACTIVE 0\n");
		gpio_direction_output(pmdata->gpio_link_active, 0);
		return count;	
	}
	if (!strncmp(buf, "1", 1)) {
		printk("set AP>>CP:  HOSTACTIVE 1\n");
		gpio_direction_output(pmdata->gpio_link_active, 1);
		return count;	
	}
}
static ssize_t show_test(struct device *d,
		struct device_attribute *attr, char *buf)
{	
	debug_ehci_reg_dump();
	return sprintf(buf, "%d\n", c8320_modem_ctl_ex.debugmask);	
}

static ssize_t store_test(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	sscanf(buf, "%d", &value);	
	c8320_modem_ctl_ex.debugmask = value;
	mif_info("set control debugmask %d\n",c8320_modem_ctl_ex.debugmask);
	return count;	
}

static ssize_t show_slavewakeup(struct device *d,
		struct device_attribute *attr, char *buf)
{
	int value = 0;
	char *p = buf;
	struct modem_data *pdata = d->platform_data;
	struct modemlink_pm_data *pmdata = pdata->link_pm_data;
	value = gpio_get_value(pmdata->gpio_link_slavewake);
	if(value){
		p += sprintf(p, "high\n");
	}else{
		p += sprintf(p, "low\n");
	}
	return p - buf;
}

static ssize_t store_slavewakeup(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct modem_data *pdata = d->platform_data;
	struct modemlink_pm_data *pmdata = pdata->link_pm_data;
	if (!strncmp(buf, "0", 1)) {
		printk("AP>>CP:  SLAVEWAKEUP 0\n");
		gpio_direction_output(pmdata->gpio_link_slavewake, 0);
		return count;	
	}
	if (!strncmp(buf, "1", 1)) {		
		printk("AP>>CP:  SLAVEWAKEUP 1\n");
		gpio_direction_output(pmdata->gpio_link_slavewake, 1);
		return count;	
	}
}
static ssize_t show_cpswitch(struct device *d,
		struct device_attribute *attr, char *buf)
{
	int value = 0;
	char *p = buf;
	struct modem_data *pdata = d->platform_data;
	struct modemlink_pm_data *pmdata = pdata->link_pm_data;
	value = gpio_get_value(pdata->gpio_cp_switch);
	if(value){
		p += sprintf(p, "high\n");
	}else{
		p += sprintf(p, "low\n");
	}
	return p - buf;
}
static ssize_t show_hostwakeup(struct device *d,
		struct device_attribute *attr, char *buf)
{
	int value = 0;
	char *p = buf;
	struct modem_data *pdata = d->platform_data;
	struct modemlink_pm_data *pmdata = pdata->link_pm_data;
	value = gpio_get_value(pmdata->gpio_link_hostwake);
	if(value){
		p += sprintf(p, "high\n");
	}else{
		p += sprintf(p, "low\n");
	}
	return p - buf;
}
static DEVICE_ATTR(test, 0664, show_test, store_test);
static DEVICE_ATTR(hostactive, 0664, show_active, store_active);
static DEVICE_ATTR(slavewakeup, 0664, show_slavewakeup, store_slavewakeup);
static DEVICE_ATTR(cpswitch, 0444, show_cpswitch, NULL);
static DEVICE_ATTR(hostwakeup, 0444, show_hostwakeup, NULL);
static struct attribute *modem_attributes[] = {
	&dev_attr_hostactive.attr,
	&dev_attr_slavewakeup.attr,
	&dev_attr_cpswitch.attr,
	&dev_attr_hostwakeup.attr,
	&dev_attr_test.attr,
	NULL
};
static const struct attribute_group modem_group = {
	.name = NULL,
	.attrs = modem_attributes,
};
static struct modem_shared *create_modem_shared_data(void)
{
	struct modem_shared *msd;
	int size = MAX_MIF_BUFF_SIZE;

	msd = kzalloc(sizeof(struct modem_shared), GFP_KERNEL);
	if (!msd)
		return NULL;

	/* initialize link device list */
	INIT_LIST_HEAD(&msd->link_dev_list);
	
	/* initialize iodevs head  cool_zhao*/
	INIT_LIST_HEAD(&msd->iodevs_list);
	
	/* initialize tree of io devices */

	msd->storage.cnt = 0;
	msd->storage.addr = kzalloc(MAX_MIF_BUFF_SIZE +
		(MAX_MIF_SEPA_SIZE * 2), GFP_KERNEL);
	if (!msd->storage.addr) {
		mif_err("IPC logger buff alloc failed!!\n");
		return NULL;
	}
	memset(msd->storage.addr, 0, size + (MAX_MIF_SEPA_SIZE * 2));
	memcpy(msd->storage.addr, MIF_SEPARATOR, strlen(MIF_SEPARATOR));
	msd->storage.addr += MAX_MIF_SEPA_SIZE;
	memcpy(msd->storage.addr, &size, MAX_MIF_SEPA_SIZE);
	msd->storage.addr += MAX_MIF_SEPA_SIZE;
	spin_lock_init(&msd->lock);

	return msd;
}

static struct modem_ctl *create_modemctl_device(struct platform_device *pdev,
		struct modem_shared *msd)
{
	int ret = 0;
	struct modem_data *pdata;
	struct modem_ctl *modemctl;
	struct device *dev = &pdev->dev;

	/* create modem control device */
	modemctl = kzalloc(sizeof(struct modem_ctl), GFP_KERNEL);
	if (!modemctl)
		return NULL;

	modemctl->msd = msd;
	modemctl->dev = dev;
	c8320_modem_ctl_ex.phone_state = STATE_OFFLINE;

	pdata = pdev->dev.platform_data;
	modemctl->mdm_data = pdata;
	modemctl->name = pdata->name;

	/* init modemctl device for getting modemctl operations */
	ret = call_modem_init_func(modemctl, pdata);
	if (ret) {
		kfree(modemctl);
		return NULL;
	}

	mif_info("%s is created!!!\n", pdata->name);

	return modemctl;
}

static struct io_device *create_io_device(struct modem_io_t *io_t,
		struct modem_shared *msd, struct modem_ctl *modemctl,
		struct modem_data *pdata)
{
	int ret = 0;
	struct io_device *iod = NULL;

	iod = kzalloc(sizeof(struct io_device), GFP_KERNEL);
	if (!iod) {
		mif_err("iod == NULL\n");
		return NULL;
	}

	iod->name = io_t->name;
	iod->id = io_t->id;
	iod->format = io_t->format;
	iod->io_typ = io_t->io_type;
	iod->link_types = io_t->links;
	iod->use_handover = pdata->use_handover;
	iod->ipc_version = pdata->ipc_version;
	atomic_set(&iod->opened, 0);

	/* link between io device and modem control */
	iod->mc = modemctl;
	/*modem control iod dev format is IPC_FMT cool_zhao */
	if (iod->format == IPC_FMT)
		modemctl->iod = iod;
	if (iod->format == IPC_BOOT && iod->id == 0xff) {
		modemctl->bootd = iod;
		mif_info("Bood device = %s\n", iod->name);
	}

	/* link between io device and modem shared */
	iod->msd = msd;

	/* add iod to rb_tree */
	/* cool_zhao how to use red_back trees,can we need modify manners */

	INIT_LIST_HEAD(&iod->list);
	list_add(&iod->list, &msd->iodevs_list);
	/* register misc device or net device */
	ret = sipc5_init_io_device(iod);
	if (ret) {
		kfree(iod);
		mif_err("sipc5_init_io_device fail (%d)\n", ret);
		return NULL;
	}

	mif_debug("%s is created!!!\n", iod->name);
	return iod;
}

static int attach_devices(struct io_device *iod, enum modem_link tx_link)
{
	struct modem_shared *msd = iod->msd;
	struct link_device *ld;

	/* find link type for this io device */
	list_for_each_entry(ld, &msd->link_dev_list, list) {
		if (IS_CONNECTED(iod, ld)) {
			/* The count 1 bits of iod->link_types is count
			 * of link devices of this iod.
			 * If use one link device,
			 * or, 2+ link devices and this link is tx_link,
			 * set iod's link device with ld
			 */
			if ((countbits(iod->link_types) <= 1) ||
					(tx_link == ld->link_type)) {
				mif_debug("set %s->%s\n", iod->name, ld->name);
				set_current_link(iod, ld);
			}
		}
	}

	/* if use rx dynamic switch, set tx_link at modem_io_t of
	 * board-*-modems.c
	 */
	if (!get_current_link(iod)) {
		mif_err("%s->link == NULL\n", iod->name);
		BUG();
	}

	switch (iod->format) {
	case IPC_FMT:
		wake_lock_init(&iod->wakelock, WAKE_LOCK_SUSPEND, iod->name);
		iod->waketime = FMT_WAKE_TIME;
		break;

	case IPC_RAW:
		wake_lock_init(&iod->wakelock, WAKE_LOCK_SUSPEND, iod->name);
		iod->waketime = RAW_WAKE_TIME;
		break;

	case IPC_BOOT:
		wake_lock_init(&iod->wakelock, WAKE_LOCK_SUSPEND, iod->name);
		iod->waketime = RAW_WAKE_TIME;
		break;

	case IPC_RAW_NCM:
		wake_lock_init(&iod->wakelock, WAKE_LOCK_SUSPEND, iod->name);
		break;

	default:
		break;
	}

	return 0;
}

static int __devinit modem_probe(struct platform_device *pdev)
{
	int i;
	struct modem_data *pdata = pdev->dev.platform_data;
	struct modem_shared *msd = NULL;
	struct modem_ctl *modemctl = NULL;
	struct io_device *iod[pdata->num_iodevs];
	struct link_device *ld;
	int error = 0;

	mif_info("%s\n", pdev->name);
	memset(iod, 0, sizeof(iod));
	c8320_modem_ctl_ex.debugmask= 0;

	msd = create_modem_shared_data();
	if (!msd) {
		mif_err("msd == NULL\n");
		goto err_free_modemctl;
	}

	modemctl = create_modemctl_device(pdev, msd);
	if (!modemctl) {
		mif_err("modemctl == NULL\n");
		goto err_free_modemctl;
	}

	/* create link device */
	/* support multi-link device */
	for (i = 0; i < LINKDEV_MAX ; i++) {
		/* find matching link type */
		if (pdata->link_types & LINKTYPE(i)) {
			ld = call_link_init_func(pdev, i);
			if (!ld)
				goto err_free_modemctl;

			mif_err("link created: %s\n", ld->name);
			ld->link_type = i;
			ld->mc = modemctl;
			ld->msd = msd;
			list_add(&ld->list, &msd->link_dev_list);
		}
	}

	/* create io deivces and connect to modemctl device */
	for (i = 0; i < pdata->num_iodevs; i++) {
		iod[i] = create_io_device(&pdata->iodevs[i], msd, modemctl,
				pdata);
		if (!iod[i]) {
			mif_err("iod[%d] == NULL\n", i);
			goto err_free_modemctl;
		}

		attach_devices(iod[i], pdata->iodevs[i].tx_link);
	}

	platform_set_drvdata(pdev, modemctl);
	error = sysfs_create_group(&pdev->dev.kobj, &modem_group);
	if (error) {
		dev_err(&pdev->dev, "Failed to create sysfs files\n");
		goto err_free_modemctl;
	}

	mif_info("Complete!!!\n");

	return 0;

err_free_modemctl:
	for (i = 0; i < pdata->num_iodevs; i++)
		if (iod[i] != NULL)
			kfree(iod[i]);

	if (modemctl != NULL)
		kfree(modemctl);

	if (msd != NULL)
		kfree(msd);

	return -ENOMEM;
}

static void modem_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct modem_ctl *mc = dev_get_drvdata(dev);
	if(mc->ops.modem_off)
		mc->ops.modem_off(mc);
	c8320_modem_ctl_ex.phone_state = STATE_OFFLINE;
}

static int modem_suspend(struct device *pdev)
{
#ifndef CONFIG_LINK_DEVICE_HSIC
	struct modem_ctl *mc = dev_get_drvdata(pdev);
#endif
	return 0;
}

static int modem_resume(struct device *pdev)
{
#ifndef CONFIG_LINK_DEVICE_HSIC
	struct modem_ctl *mc = dev_get_drvdata(pdev);
#endif
	return 0;
}

static int modem_suspend_noirq(struct device *pdev)
{
#ifdef CONFIG_LINK_DEVICE_HSIC
	struct modem_ctl *mc = dev_get_drvdata(pdev);
#endif
	return 0;
}

static int modem_resume_noirq(struct device *pdev)
{
#ifdef CONFIG_LINK_DEVICE_HSIC
	struct modem_ctl *mc = dev_get_drvdata(pdev);
#endif
	return 0;
}

static const struct dev_pm_ops modem_pm_ops = {
	.suspend    = modem_suspend,
	.resume     = modem_resume,
	.suspend_noirq = modem_suspend_noirq,
	.resume_noirq = modem_resume_noirq,
};

static struct platform_driver modem_driver = {
	.probe = modem_probe,
	.shutdown = modem_shutdown,
	.driver = {
		.name = "mif_sipc5",
		.pm   = &modem_pm_ops,
	},
};

static int __init modem_init(void)
{
	return platform_driver_register(&modem_driver);
}

module_init(modem_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung Modem Interface Driver");
