/*
 * Modem control driver
 *
 * Copyright (C) 2010 CYIT Co.Ltd
 * Author: 
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <mach/regs-gpio.h>
#include <plat/devs.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-exynos.h>
#include <mach/gpio.h>

#include <mach/modemctl.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/io.h>

#include <linux/clk.h>

#include <linux/pm_runtime.h>

#include <plat/cpu.h>
#include <plat/ehci.h>
#include <plat/usb-phy.h>

#include <mach/regs-pmu.h>
#include <mach/regs-usb-host.h>
static struct s5p_ehci_platdata ehci_pdata __initdata;
#define CONFIG_C8320_NREADY 	0
#define DEV_WAKEUP_ENABLE      1
#define SC3_TEMP_HSIC

struct platform_device c8320_modem = {
	.name = "c8320_modem_hsic",
	.id = -1,
};

enum {
	HOST_WAKEUP_LOW = 1,
	HOST_WAKEUP_WAIT_RESET,
} HOST_WAKEUP_STATE;
struct hsic_modem_ctl * global_mc = NULL;

extern int acm_request_resume(void);

int mc_is_normal(void)
{
	struct hsic_modem_ctl *mc = global_mc;
#ifdef CONFIG_C8320_NREADY
	return 1;
#endif
	if (!mc)
		return 0;
	return (mc->cp_status == CP_NORMAL);
	
}
EXPORT_SYMBOL_GPL(mc_is_normal);

int modem8320_is_cp_wakeup_ap(void)
{
	struct hsic_modem_ctl *mc = global_mc;
	if(!mc)
		return 0;
	return ((mc->c8320_platform_pm->cp_get_gpio(mc->gpio_ipc_host_wakeup)
		     == HOST_WUP_LEVEL)? 1 : 0);
}
EXPORT_SYMBOL_GPL(modem8320_is_cp_wakeup_ap);

int mc_interface_count(int used){
	struct hsic_modem_ctl *mc = global_mc;
	if (!mc){
		printk("mc is not ready\n");
		return -ENODEV;
	}
	if(used){
		mc->interface_used++;
	}else{
		mc->interface_used = 0;
	}
	if(mc->interface_used == C8320_INF_NUM){
		schedule_delayed_work(&mc->pm_runtime_work, msecs_to_jiffies(500));
	}
	
	return mc->interface_used;		
}
EXPORT_SYMBOL_GPL(mc_interface_count);

int hsic_modem_on(struct hsic_modem_ctl *mc)
{
	return 0;
}
int hsic_modem_off(struct hsic_modem_ctl *mc)
{
	return 0;
}
int hsic_modem_reset(struct hsic_modem_ctl *mc)
{
	return 0;
}
int mc_prepare_resume(int ms_time)
{
	int val;
	struct completion done;
	struct hsic_modem_ctl *mc = global_mc;

	if (!mc)
		return -EFAULT;
	/*before do ipc, we should check if modem is on or not.*/
	if(!mc_is_normal())
		return -ENODEV;
	val = mc->c8320_platform_pm->cp_get_gpio(mc->gpio_ipc_slave_wakeup);
	if (val) {		
		mc->c8320_platform_pm->cp_set_gpio(mc->gpio_ipc_slave_wakeup, 0);
		dev_info(mc->dev, "svn SLAV_WUP:reset\n");
	}
	val = mc->c8320_platform_pm->cp_get_gpio(mc->gpio_ipc_host_wakeup);
	if (val == HOST_WUP_LEVEL) {
		dev_info(mc->dev, "svn HOST_WUP:high!\n");
		return MC_HOST_HIGH;
	}

	init_completion(&done);
	mc->l2_done = &done;
	
	mc->c8320_platform_pm->cp_set_gpio(mc->gpio_ipc_slave_wakeup, 1);
	dev_dbg(mc->dev, "AP>>CP:  SLAV_WUP:1,%d\n",
		mc->c8320_platform_pm->cp_get_gpio(mc->gpio_ipc_slave_wakeup));

	if (!wait_for_completion_timeout(&done, ms_time)) {
			/*before do ipc, we should check if modem is on or not.*/
			if(!mc_is_normal())
				return -ENODEV;
		val =mc->c8320_platform_pm->cp_get_gpio(mc->gpio_ipc_host_wakeup);
		if (val == HOST_WUP_LEVEL) {
			dev_err(mc->dev, "maybe complete late.. %d\n", ms_time);
			mc->l2_done = NULL;
			return MC_SUCCESS;
		}
		dev_err(mc->dev, "Modem wakeup timeout %d\n", ms_time);
	
		mc->c8320_platform_pm->cp_set_gpio(mc->gpio_ipc_slave_wakeup, 0);
		dev_dbg(mc->dev, "AP>>CP:  SLAV_WUP:0,%d\n",
			mc->c8320_platform_pm->cp_get_gpio(mc->gpio_ipc_slave_wakeup));
		mc->l2_done = NULL;
		return MC_HOST_TIMEOUT;
	}
	return MC_SUCCESS;
}
EXPORT_SYMBOL_GPL(mc_prepare_resume);

static ssize_t show_control(struct device *d,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;
	struct hsic_modem_ctl *mc = dev_get_drvdata(d);
	
	if (mc->op_state == HSIC_OP_ON)
		p += sprintf(p, "on ");
	if (mc->op_state == HSIC_OP_OFF)
		p += sprintf(p, "off ");
	if (mc->op_state == HSIC_OP_RESET)
		p += sprintf(p, "reset ");
	if (mc->op_state == HSIC_OP_NONE)
		p += sprintf(p, "(No ops)");	

	p += sprintf(p, "\n");
	return p - buf;
}

static ssize_t store_control(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct hsic_modem_ctl *mc = dev_get_drvdata(d);

	if (!strncmp(buf, "on", 2)) {
		hsic_modem_on(mc);
		return count;	
	}
	if (!strncmp(buf, "off", 3)) {
		hsic_modem_off(mc);
		return count;
	}
	if (!strncmp(buf, "reset", 5)) {
		hsic_modem_reset(mc);
		return count;
	}
	if (!strncmp(buf, "gsw=0", 5)) {
		mc->c8320_platform_pm->cp_set_gpio(mc->gpio_ipc_slave_wakeup, 0);
		return count;
	}
	if (!strncmp(buf, "gsw=1", 5)) {
		mc->c8320_platform_pm->cp_set_gpio(mc->gpio_ipc_slave_wakeup, 1);
		return count;
	}
	if (!strncmp(buf, "gat=0", 5)) {
		mc->c8320_platform_pm->cp_set_gpio(mc->gpio_active_state, 0);
		return count;
	}
	if (!strncmp(buf, "gat=1", 5)) {
		mc->c8320_platform_pm->cp_set_gpio(mc->gpio_active_state, 1);
		return count;
	}	
	return count;
}

static ssize_t show_status(struct device *d,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;
	struct hsic_modem_ctl *mc = dev_get_drvdata(d);

	p += sprintf(p, "%d\n", mc_is_normal());

	return p - buf;
}

static ssize_t show_wakeup(struct device *d,
		struct device_attribute *attr, char *buf)
{
	struct hsic_modem_ctl *mc = dev_get_drvdata(d);
	int count = 0;

	if (!mc->gpio_ipc_host_wakeup)
		return -ENXIO;

	count += sprintf(buf + count, "%d\n",
			mc->wakeup_flag);

	return count;
}

static ssize_t store_wakeup(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct hsic_modem_ctl *mc = dev_get_drvdata(d);

	if (!strncmp(buf, "reset", 5)) {
		mc->wakeup_flag = HOST_WAKEUP_WAIT_RESET;
		dev_info(mc->dev, "%s: wakup_flag %d\n",
			__func__, mc->wakeup_flag);
		return count;
	}
	return 0;

}

static ssize_t show_debug(struct device *d,
		struct device_attribute *attr, char *buf)
{
	char *p = buf;
	int i;
	struct hsic_modem_ctl *mc = dev_get_drvdata(d);

	if (mc->irq)
		p += sprintf(p, "Irq : %d\n",  mc->irq);

	p += sprintf(p, "GPIO ----\n");

	if (mc->gpio_ipc_host_wakeup)
		p += sprintf(p, "\t%3d %d : hw sel\n", mc->gpio_ipc_host_wakeup,
				gpio_get_value(mc->gpio_ipc_host_wakeup));
	if (mc->gpio_ipc_slave_wakeup)
		p += sprintf(p, "\t%3d %d : sw sel\n", mc->gpio_ipc_slave_wakeup,
				gpio_get_value(mc->gpio_ipc_slave_wakeup));
	p += sprintf(p, "Support types ---\n");

	return p - buf;
}
static ssize_t show_active(struct device *d,
		struct device_attribute *attr, char *buf)
{
	int value = 0;
	char *p = buf;
	struct hsic_modem_ctl *mc = global_mc;
	value = gpio_get_value(mc->gpio_active_state);
	if(value){
		p += sprintf(p, "high\n");
	}else{
		p = sprintf(p, "low\n");
	}
	return p - buf;
}

static ssize_t store_active(struct device *d,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct hsic_modem_ctl *mc = global_mc;
	if (!strncmp(buf, "0", 1)) {
		printk("ACTIVE is high need set AP>>CP:  HOSTACTIVE 0\n");
		gpio_direction_output(mc->gpio_active_state, 0);
		return count;	
	}
	if (!strncmp(buf, "1", 1)) {
		printk("ACTIVE is high need set AP>>CP:  HOSTACTIVE 1\n");
		gpio_direction_output(mc->gpio_active_state, 1);
		return count;	
	}
}
static DEVICE_ATTR(control, 0664, show_control, store_control);
static DEVICE_ATTR(status, S_IRUGO, show_status, NULL);
static DEVICE_ATTR(wakeup, 0664, show_wakeup, store_wakeup);
static DEVICE_ATTR(debug, S_IRUGO, show_debug, NULL);
static DEVICE_ATTR(hostactive, 0777, show_active,store_active);
static struct attribute *modem_attributes[] = {
	&dev_attr_control.attr,
	&dev_attr_status.attr,
	&dev_attr_wakeup.attr,
	&dev_attr_debug.attr,
	&dev_attr_hostactive.attr,
	NULL
};

static const struct attribute_group modem_group = {
	.name = NULL,
	.attrs = modem_attributes,
};

static irqreturn_t modem_resume_thread(int irq, void *dev_id)
{
	struct hsic_modem_ctl *mc = (struct hsic_modem_ctl *)dev_id;
	int val = mc->c8320_platform_pm->cp_get_gpio(mc->gpio_ipc_host_wakeup);
	int err;

	dev_err(mc->dev, "CP>>AP:  HOST_WUP:%d\n", val);
	if (val != HOST_WUP_LEVEL) {
		mc->c8320_platform_pm->cp_set_gpio(mc->gpio_ipc_slave_wakeup, 0);
		//set_modem_slave_wakeup(0);
		mc->debug_cnt = 0;
		return IRQ_HANDLED;
	}

	if (val == HOST_WUP_LEVEL) {
		if (mc->l2_done) {
			complete(mc->l2_done);
			mc->l2_done = NULL;
		}
		err = acm_request_resume(); //mask by cool_zhao for on times
		if (err < 0)
			dev_err(mc->dev, "request resume failed: %d\n", err);
		mc->debug_cnt++;
	}
	if (mc->debug_cnt > 30) {
		dev_err(mc->dev, "Abnormal Host wakeup -- over 30times");
		//disable_irq(irq);
		mc->debug_cnt = 0;
		if(mc_is_normal())
		{
			// here we need to do reset cp, but who do it? and how to know?
		}		
	}
	
	if (!val && mc->wakeup_flag == HOST_WAKEUP_WAIT_RESET) {
		mc->wakeup_flag = HOST_WAKEUP_LOW;
		dev_err(mc->dev, "%s: wakeup flag (%d)\n",
			__func__, mc->wakeup_flag);
	}
	
	return mc->irq_callback(irq, mc->c8320_platform_data);
	
}

static void ehci_runtime_start(struct work_struct *work)
{

	struct hsic_modem_ctl *mc = global_mc;
	
	if (mc) {
		dev_info(&mc->peripheral_platform_device_ehci->dev, "ACM Runtime PM Start!!\n");		
		
		//pm_runtime_allow(&mc->peripheral_platform_device_ehci->dev); /*ehci*/
		//should try usb_enable_autosuspend(acm->dev); in next week
#if 0
		if (usb_autopm_get_interface() >= 0)
			acm->control->needs_remote_wakeup = 0;
		usb_autopm_put_interface(acm->control);
#endif
	}
}

static void free_all(struct hsic_modem_ctl *mc)
{
	if (mc) {
		if (mc->c8320_platform_pm){
			mc->c8320_platform_pm = NULL;
		}
		if(mc->c8320_platform_data){
			mc->c8320_platform_data = NULL;
		}
		if (mc->group){
			sysfs_remove_group(&mc->dev->kobj, mc->group);
		}
		if (mc->irq){
			free_irq(mc->irq, mc);	
		}
		kfree(mc);
	}
}
#ifdef SC3_TEMP_HSIC
/* for test and host active swap to suspend request */
#define HSIC_HOST_AVCTIVE_TEMP EXYNOS3_GPX2(5) //GPX1_1 //EXYNOS3_GPX3(5) //GPX1_1
#define HSIC_HOST_WAKEUP_TEMP  EXYNOS3_GPX2(3) //GPX2_2 //EXYNOS3_GPX2(2)  //GPX2_2
#define HSIC_CP_WAKEUP_TEMP	   EXYNOS3_GPX0(5) //GPX0_5
#define HSIC_CP_SUSPEND_REQUEST_TEMP EXYNOS3_GPX3(1) // GPX3_1

static struct work_struct reset_work;

extern int s5p_ehci_power(struct platform_device *pdev, int value);
static void reset_usb_work(struct work_struct *data)
{
	struct hsic_modem_ctl *mc = global_mc;
	int value = 0;
	s5p_ehci_power(mc->peripheral_platform_device_ehci, 0);
	msleep(1);
	s5p_ehci_power(mc->peripheral_platform_device_ehci, 1);
	
	value = gpio_get_value(mc->gpio_active_state);
	if(value == 1)
	{
		printk("ACTIVE is high need set AP>>CP:  HOSTACTIVE 0\n");
		gpio_direction_output(mc->gpio_active_state, 0);
		msleep(1);
	}
	printk("AP>>CP:  HOSTACTIVE 1\n");
	gpio_direction_output(mc->gpio_active_state, 1);
}
static irqreturn_t modem_resume_thread_temp (int irq, void *dev_id)
{
	struct hsic_modem_ctl *mc = (struct hsic_modem_ctl *)dev_id;
	int val = gpio_get_value(mc->gpio_ipc_host_wakeup);
	int err;

	
	printk("%s :CP>>AP:  HOST_WUP:%d\n", __func__,val);
        extern int cp2ap_enable_irq;
        if(cp2ap_enable_irq != 1){
                printk("do no thing\n");
                return IRQ_HANDLED;
        }
	if (val != HOST_WUP_LEVEL) {
		printk("AP>>CP:  HOSTACTIVE: 0\n");
		gpio_direction_output(mc->gpio_active_state, 0);		
		mc->debug_cnt = 0;
		return IRQ_HANDLED;
	}
	/* in test mode only use switch usb device ,here need reset host  and set host_active to CP */
	if (val == HOST_WUP_LEVEL) {
		schedule_work(&reset_work);
		printk("%s:reset host hcd\n",__func__);		
	}
	return IRQ_HANDLED;
	
}

static int c83xx_hsic_gpio_init(void)
{
	int err;

	/* AP to CP status indication for uart */
	err = gpio_request(HSIC_HOST_AVCTIVE_TEMP, "AP2CP_HSIC_ACTIVE"); 
	if(err < 0){
		pr_err("[%s] Failed to request gpio of AP2CP_HSIC_ACTIVE\n", __func__);
		goto hsic_pm_init_err;
	}
	samsung_gpio_cfgpin(HSIC_HOST_AVCTIVE_TEMP, SAMSUNG_GPIO_OUTPUT);
	samsung_gpio_setpull(HSIC_HOST_AVCTIVE_TEMP, SAMSUNG_GPIO_PULL_NONE);
	gpio_set_value(HSIC_HOST_AVCTIVE_TEMP, 0);

	/* AP to CP WAKE for uart */
	err = gpio_request(HSIC_CP_WAKEUP_TEMP, "AP2CP_HSIC_WAKE");
	if(err < 0){
		pr_err("[%s] Failed to request gpio of AP2CP_HSIC_WAKE\n", __func__);
		goto hsic_pm_init_err;
	}
	samsung_gpio_cfgpin(HSIC_CP_WAKEUP_TEMP, SAMSUNG_GPIO_OUTPUT);
	samsung_gpio_setpull(HSIC_CP_WAKEUP_TEMP, SAMSUNG_GPIO_PULL_NONE);
	gpio_set_value(HSIC_CP_WAKEUP_TEMP, 0);

	/* CP to AP status indication for uart */
	err = gpio_request(HSIC_HOST_WAKEUP_TEMP, "CP2AP_HSIC_WAKEUP");
	if(err < 0){
		pr_err("[%s] Failed to request gpio of CP2AP_HSIC_WAKEUP\n", __func__);
		goto hsic_pm_init_err;
	}
	samsung_gpio_cfgpin(HSIC_HOST_WAKEUP_TEMP, SAMSUNG_GPIO_INPUT);
	samsung_gpio_setpull(HSIC_HOST_WAKEUP_TEMP, SAMSUNG_GPIO_PULL_NONE);

	/* CP to AP WAKE for uart */
	err = gpio_request( HSIC_CP_SUSPEND_REQUEST_TEMP, "CP2AP_HSIC_SUSPEND_REQUEST");
	if(err < 0){
		pr_err("[%s] Failed to request gpio of CP2AP_HSIC_SUSPEND_REQUEST\n", __func__);
		goto hsic_pm_init_err;
	}
	samsung_gpio_cfgpin( HSIC_CP_SUSPEND_REQUEST_TEMP, SAMSUNG_GPIO_INPUT);
	samsung_gpio_setpull( HSIC_CP_SUSPEND_REQUEST_TEMP, SAMSUNG_GPIO_PULL_NONE);
	printk("%s: end ok\n",__func__);
hsic_pm_init_err:
	return err;

}
static void mc_gpio_init(void){
	int ret = 0;
	struct hsic_modem_ctl *mc = global_mc;
	mc->gpio_active_state = HSIC_HOST_AVCTIVE_TEMP;
	mc->gpio_ipc_host_wakeup = HSIC_HOST_WAKEUP_TEMP;
	mc->gpio_ipc_slave_wakeup = HSIC_CP_WAKEUP_TEMP;
	mc->gpio_suspend_request = HSIC_CP_SUSPEND_REQUEST_TEMP;
	ret = c83xx_hsic_gpio_init();
	if(ret){
		printk("hsic gpio init failed\n");
		return ret;
	}
	mc->irq = gpio_to_irq(mc->gpio_ipc_host_wakeup);
	
	ret = request_threaded_irq(mc->irq, NULL, modem_resume_thread_temp,
                        IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                        "IPC_HOST_WAKEUP", mc);
	if(ret){
		printk("register irq failed\n");
		return ret;
	}
	INIT_WORK(&reset_work, reset_usb_work);
	printk("%s:complete ok\n",__func__);
}
#endif

static int __devinit modem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hsic_modem_ctl *mc;
	int error;

	mc = kzalloc(sizeof(struct hsic_modem_ctl), GFP_KERNEL);
	if (!mc) {
		dev_err(dev, "Failed to allocate device\n");
		return -ENOMEM;
	}	
	mc->dev = dev;
	dev_set_drvdata(mc->dev, mc);
	
	INIT_DELAYED_WORK(&mc->pm_runtime_work, ehci_runtime_start);
	
	error = sysfs_create_group(&mc->dev->kobj, &modem_group);
	if (error) {
		dev_err(dev, "Failed to create sysfs files\n");
		goto fail;
	}
	mc->group = &modem_group; 
	mc->debug_cnt = 0;
	mc->interface_used = 0;
	mc->peripheral_platform_device_ehci = &s5p_device_ehci;
	
	spin_lock_init(&mc->hsic_mc_lock);
	mutex_init(&mc->mutex);
	
	device_init_wakeup(&pdev->dev, DEV_WAKEUP_ENABLE);
	platform_set_drvdata(pdev, mc);
	global_mc = mc;
	#ifdef SC3_TEMP_HSIC
	mc_gpio_init();
	#endif
	
	return 0;

fail:
	free_all(mc);
	return error;
}

static int __devexit modem_remove(struct platform_device *pdev)
{
	struct hsic_modem_ctl *mc = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
#if CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&mc->reset_lock);
#endif
	free_all(mc);
	return 0;
}


#ifdef CONFIG_PM
static int modem_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct hsic_modem_ctl *mc = platform_get_drvdata(pdev);

	
	if (device_may_wakeup(dev) && mc_is_normal())
		enable_irq_wake(mc->irq);

	return 0;
}

static int modem_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct hsic_modem_ctl *mc = platform_get_drvdata(pdev);

	if (device_may_wakeup(dev) && mc_is_normal())
		disable_irq_wake(mc->irq);
	return 0;
}

static const struct dev_pm_ops modem_pm_ops = {
	.suspend	= modem_suspend,
	.resume		= modem_resume,
};
#endif

static struct platform_driver modem_driver = {
	.probe		= modem_probe,
	.remove		= __devexit_p(modem_remove),
	.driver		= {
		.name	= "c8320_modem_hsic",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &modem_pm_ops,
#endif
	},
};
int register_modem_ctl(int irq, irqreturn_t (*irq_callback)(int irq, void *dev_id), struct c83xx_platform_data *pdata)
{
	struct hsic_modem_ctl *mc = global_mc;
	int ret = 0;
	if(mc == NULL){
		printk("hsic modem ctl is not ready\n");
		return -ENODEV;
	}
	mc->c8320_platform_data = pdata;
	mc->c8320_platform_pm = pdata->cp_pm_hsic;
	mc->gpio_active_state = mc->c8320_platform_pm->a2c_status_gpio;
	mc->gpio_ipc_host_wakeup = mc->c8320_platform_pm->c2a_wake_gpio;
	mc->gpio_ipc_slave_wakeup = mc->c8320_platform_pm->a2c_wake_gpio;
	mc->gpio_suspend_request = mc->c8320_platform_pm->c2a_status_gpio;
	mc->irq = irq;
	ret = request_threaded_irq(irq, NULL, modem_resume_thread,
                        IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                        "IPC_HOST_WAKEUP", mc);
	if(ret){
		printk("register irq failed\n");
		return ret;
	}
	pdata->mc = mc;
	return 0;
	
}
EXPORT_SYMBOL_GPL(register_modem_ctl);

static int __init modem_init(void)
{
	platform_driver_register(&modem_driver);
	platform_device_register(&c8320_modem);
	return 0; 
}

static void __exit modem_exit(void)
{
	platform_driver_unregister(&modem_driver);
}
module_init(modem_init);
module_exit(modem_exit);

