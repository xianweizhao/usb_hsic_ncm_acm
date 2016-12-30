/*
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Author: Yulgon Kim <yulgon.kim@samsung.com>
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <mach/regs-pmu.h>
#include <mach/regs-usb-phy.h>
#include <plat/cpu.h>
#include <plat/usb-phy.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio-exynos3472-smdk-board.h>
#include "../../../drivers/ipc_c8320/modem_v2/c8320_modem_ctl.h"
//#include <mach/sec_modem.h>

#define ETC6PUD		(S5P_VA_GPIO2 + 0x228)
#define EXYNOS_USB_CFG		(S3C_VA_SYS + 0x21C)

#define PHY_ENABLE	(1 << 0)
#define PHY_DISABLE	(0)

extern struct timer_list sc3_usbdev_timer;
extern int timer_sc3_init;
enum usb_host_type {
	HOST_PHY_EHCI	,//= (0x1 << 0),
	HOST_PHY_OHCI	,//= (0x1 << 1),
	HOST_PHY_DEV	,//= (0x1 << 2),
};

enum usb_phy_type {
	USB_PHY		= (0x1 << 0),
	USB_PHY0	= (0x1 << 1),/*phy0,phy1 share the same phy,but the control register is diff*/
	USB_PHY1	= (0x1 << 2),
	USB_PHY_HSIC0	= (0x1 << 3),
	USB_PHY_HSIC1	= (0x1 << 4),
};
enum usb_phy_status{
	USB_PHY_L3	= 0,
	USB_PHY_L2,
	USB_PHY_L1,
	USB_PHY_L0,
};
struct exynos_usb_phy {
	struct clk *dev_phy_clk;
	struct clk *host_phy_clk;
	struct mutex phy_lock;
	atomic_t phyclkused;
	atomic_t sharedphyused;
	unsigned long phystatus;
};

static struct exynos_usb_phy usb_phy_control = {
	.phy_lock = __MUTEX_INITIALIZER(usb_phy_control.phy_lock)
};
static struct regulator *usb_analog_ap, *usb_core_ap ,*usb_hsic_ap;
void usb_phy_power_on(void)
{
	int retval;

	if(usb_phy_control.phystatus){
		printk("not need enable power has done\n");
		return;
	}
	if (!usb_analog_ap) {
		usb_analog_ap = regulator_get(NULL, "vdd33_utog_ap");
		if (IS_ERR(usb_analog_ap)) {
			retval = PTR_ERR(usb_analog_ap);
			printk( "No VDD_USB_3.3V regualtor: %d\n",retval);
			usb_analog_ap = NULL;
		}
	}
	if (!usb_core_ap) {
		usb_core_ap = regulator_get(NULL, "vdd10_usb_ap");
		if (IS_ERR(usb_core_ap)) {
			retval = PTR_ERR(usb_core_ap);
			printk("No VDD_USB_1.0V regualtor: %d\n",retval);
			usb_core_ap = NULL;
		}
	}
	if (!usb_hsic_ap) {
		usb_hsic_ap = regulator_get(NULL, "vdd18_hsic_ap");
		if (IS_ERR(usb_hsic_ap)) {
			retval = PTR_ERR(usb_hsic_ap);
			printk("No VDD_USB_1.8V regualtor: %d\n",retval);
			usb_hsic_ap = NULL;
		}
	}
	if (usb_analog_ap)
		regulator_enable(usb_analog_ap);
	if (usb_core_ap)
		regulator_enable(usb_core_ap);
	if (usb_hsic_ap)
		regulator_enable(usb_hsic_ap);
}
void usb_phy_power_off(void)
{
	if(usb_phy_control.phystatus){
		printk("power off undo phystatus is %ld ,not zero\n",usb_phy_control.phystatus);
		return;
	}
	if (usb_analog_ap)
		regulator_disable(usb_analog_ap);
	if (usb_core_ap)
		regulator_disable(usb_core_ap);	
	if (usb_hsic_ap)
		regulator_disable(usb_hsic_ap);
}
static int exynos_usb_dev_phy_clock_enable(struct platform_device *pdev)
{
	struct clk *clk;

	if (!usb_phy_control.dev_phy_clk) {
		/*
		 * PHY clock domain is 'usbhost' on exynos5250.
		 * But, PHY clock domain is 'otg' on others.
		 */
		clk = clk_get(&pdev->dev, "usbotg");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "Failed to get phy clock\n");
			return PTR_ERR(clk);
		} else
			usb_phy_control.dev_phy_clk = clk;
	}
	return clk_enable(usb_phy_control.dev_phy_clk);
}



static int exynos_usb_phy_clock_enable(struct platform_device *pdev)
{

	/* only dev clk to operate USB  host regs  so here need enable cool_zhao */	

	return exynos_usb_dev_phy_clock_enable(pdev);

}

static int exynos_usb_dev_phy_clock_disable(struct platform_device *pdev)
{
	struct clk *clk;

	if (!usb_phy_control.dev_phy_clk) {
		clk = clk_get(&pdev->dev, "usbotg");
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "Failed to get phy clock\n");
			return PTR_ERR(clk);
		} else
			usb_phy_control.dev_phy_clk = clk;
	}
	clk_disable(usb_phy_control.dev_phy_clk);
	return 0;
}
static int exynos_usb_phy_clock_disable(struct platform_device *pdev)
{

	/* only dev clk to operate USB  host regs  so here need disable cool_zhao */	
	return exynos_usb_dev_phy_clock_disable(pdev);
	
}

static void exynos_usb_mux_change(struct platform_device *pdev, int val)
{
	u32 is_host;

	is_host = readl(EXYNOS_USB_CFG);
	writel(val, EXYNOS_USB_CFG);
	if (is_host != val)
		dev_dbg(&pdev->dev, "Change USB MUX from %s to %s",
			is_host ? "Host" : "Device",
			val ? "Host" : "Device");
	printk("%s:is host %d\n",__func__,val);
}
#if 0
static int exynos3_usb_phy0_is_on(void)
{
	return (readl(EXYNOS3_PHYPWR) & PHY0_ANALOG_POWERDOWN) ? 0 : 1;
}
static int exynos3_usb_phy1_is_on(void)
{
	return (readl(EXYNOS3_PHYPWR) & PHY1_STD_ANALOG_POWERDOWN) ? 0 : 1;
}
static int exynos3_usb_hsic_phy_is_on(void)
{
	return (readl(EXYNOS3_PHYPWR) & PHY1_HSIC0_ANALOG_DOWN) ? 0 : 1;
}
static int exynos3_usb_device_phy0_is_on()
{
	if(exynos3_usb_phy0_is_on())
		return readl(EXYNOS_USB_CFG) ? 0 : 1;
	return -ENXIO;
}
static int exynos3_usb_host_phy1_is_on()
{
	if(exynos3_usb_phy1_is_on())
		return readl(EXYNOS_USB_CFG) ? 1 : 0;
	return -ENXIO;
}
#endif
static u32 exynos_usb_phy_set_clock(struct platform_device *pdev, int on)
{
	struct clk *ref_clk;
	u32 refclk_freq = 0;

	ref_clk = clk_get(&pdev->dev, "sclk_usbphy");
	if (IS_ERR(ref_clk)) {
		dev_err(&pdev->dev, "Failed to get reference clock\n");
		return PTR_ERR(ref_clk);
	}
	if(on){
		switch (clk_get_rate(ref_clk)) {
		case 96 * 100000:
			refclk_freq = EXYNOS3472_CLKSEL_9600K;
			break;
		case 10 * MHZ:
			refclk_freq = EXYNOS3472_CLKSEL_10M;
			break;
		case 12 * MHZ:
			refclk_freq = EXYNOS3472_CLKSEL_12M;
			break;
		case 192 * 100000:
			refclk_freq = EXYNOS3472_CLKSEL_19200K;
			break;
		case 20 * MHZ:
			refclk_freq = EXYNOS3472_CLKSEL_20M;
			break;
		case 50 * MHZ:
			refclk_freq = EXYNOS3472_CLKSEL_50M;
			break;
		case 24 * MHZ:
		default:
			/* default reference clock */
			refclk_freq = EXYNOS3472_CLKSEL_24M;
			break;
		}
		clk_enable(ref_clk);
	}
	else{
		clk_disable(ref_clk);
	}		
	clk_put(ref_clk);

	return refclk_freq;
}

static void exynos_usb_phy_control(enum usb_phy_type phy_type , int on)
{
	if (phy_type & (USB_PHY|USB_PHY0|USB_PHY1))
		writel(on, EXYNOS3_USB_PHY_CONTROL);
	if (phy_type & USB_PHY_HSIC0)
		writel(on, EXYNOS3_HSIC1_PHY_CONTROL);
}
static int exynos3_usb_shared_phy_power_init(struct platform_device *pdev)
{
	int sharedphyused =  atomic_read(&usb_phy_control.sharedphyused);

	atomic_inc(&usb_phy_control.sharedphyused);
	exynos_usb_phy_control(USB_PHY, PHY_ENABLE);
	printk("%s : phy enableed\n", __func__);
	if (c8320_modem_ctl_ex.force_hsic_poweroff){
		c8320_modem_ctl_ex.force_hsic_poweroff = false;
		if(HOST_PHY_EHCI&usb_phy_control.phystatus){
			exynos_usb_phy_control(USB_PHY, PHY_ENABLE);
		}
	}else{
		if(!sharedphyused){
			/*first init, enable */
			exynos_usb_phy_control(USB_PHY, PHY_ENABLE);
		}
	}
	return 0;
}
static int exynos3_usb_shared_phy_power_exit(struct platform_device *pdev)
{

	if (c8320_modem_ctl_ex.force_hsic_poweroff){
		c8320_modem_ctl_ex.force_hsic_poweroff = false;
		if(HOST_PHY_EHCI&usb_phy_control.phystatus){
			return -EBUSY;
		}
	}else{
		if (atomic_dec_return(&usb_phy_control.sharedphyused) > 0) {
			dev_info(&pdev->dev, "shared phy still being used,exit failed.\n");
			return -EBUSY;
		}
	}
	exynos_usb_phy_control(USB_PHY, PHY_DISABLE);
	return 0;
}

static int exynos3_usb_phy_clock_init(struct platform_device *pdev)
{
	u32 phyclk;
//	dump_stack();
	int phyclkused = atomic_read(&usb_phy_control.phyclkused);
	printk("-----------------------%s-------------------pdev=%p, pdev->name=%s, used count=%d\n", __func__, pdev, pdev->name?pdev->name:"null", phyclkused);
	
	atomic_inc(&usb_phy_control.phyclkused);
	if(!phyclkused){
		phyclk = exynos_usb_phy_set_clock(pdev,1);
		phyclk |= EXYNOS3_REF_CLKSEL;
		writel(phyclk, EXYNOS3_PHYCLK);		
	}
	return 0;
}
static int exynos3_usb_phy_clock_exit(struct platform_device *pdev)
{
	int phyclkused = 0;
	phyclkused = atomic_read(&usb_phy_control.phyclkused);
	if(phyclkused == 0){
		printk("%s: pdev=%p, pdev->name=%s, used count=%d  not need disable clk\n", __func__, pdev, pdev->name?pdev->name:"null", phyclkused);
		return 0;
	}
	if (atomic_dec_return(&usb_phy_control.phyclkused) > 0) {	
		dev_info(&pdev->dev, "phy clock still being used,exit failed.\n");	

		phyclkused = atomic_read(&usb_phy_control.phyclkused);
		printk("-----------------------%s-------------------pdev=%p, pdev->name=%s, used count=%d\n", __func__, pdev, pdev->name?pdev->name:"null", phyclkused);
		return -EBUSY;	
	}
	phyclkused = atomic_read(&usb_phy_control.phyclkused);
	printk("-----------------------%s-------------------pdev=%p, pdev->name=%s, used count=%d\n", __func__, pdev, pdev->name?pdev->name:"null", phyclkused);
	exynos_usb_phy_set_clock(pdev,0);
	return 0;
}

static int exynos3_usb_phy0_init(struct platform_device *pdev)
{
	u32 phypwr;
	u32 rstcon;

	exynos3_usb_shared_phy_power_init(pdev);
	exynos3_usb_phy_clock_init(pdev);
	/* reset all ports of both PHY and Link */
	/* set to normal of PHY0 */
	phypwr = readl(EXYNOS3_PHYPWR) & ~PHY0_NORMAL_MASK;
	writel(phypwr, EXYNOS3_PHYPWR);

	rstcon = readl(EXYNOS3_RSTCON) | PHY0_SWRST_MASK;
	writel(rstcon, EXYNOS3_RSTCON);
	udelay(10);
	rstcon &= ~PHY0_SWRST_MASK;
	writel(rstcon, EXYNOS3_RSTCON);
	udelay(80);	
	return 0;
}

static int exynos3_usb_phy0_exit(struct platform_device *pdev)
{

	/* unset to normal of PHY0 */
	writel((readl(EXYNOS3_PHYPWR) | PHY0_NORMAL_MASK),
			EXYNOS3_PHYPWR);	
	exynos3_usb_shared_phy_power_exit(pdev);
	exynos3_usb_phy_clock_exit(pdev);
	
	return 0;
}
static int exynos3_usb_phy1_init(struct platform_device *pdev)
{
	u32 phypwr;
	u32 rstcon;
	
	exynos3_usb_shared_phy_power_init(pdev);
	exynos3_usb_phy_clock_init(pdev);
	/* reset all ports of both PHY and Link */
	/* set to normal of PHY1 */
	phypwr = readl(EXYNOS3_PHYPWR) & ~PHY1_STD_NORMAL_MASK;
	writel(phypwr, EXYNOS3_PHYPWR);

	rstcon = readl(EXYNOS3_RSTCON) | PHY1_SWRST_MASK;
	writel(rstcon, EXYNOS3_RSTCON);
	udelay(10);
	rstcon &= ~PHY1_SWRST_MASK;
	writel(rstcon, EXYNOS3_RSTCON);
	udelay(80);
	return 0;
}
static int exynos3_usb_phy1_exit(struct platform_device *pdev)
{
	/* unset to normal of PHY1 */
	writel((readl(EXYNOS3_PHYPWR) | PHY1_STD_NORMAL_MASK),
			EXYNOS3_PHYPWR);
	exynos3_usb_shared_phy_power_exit(pdev);
	exynos3_usb_phy_clock_exit(pdev);;
	return 0;
}

static int exynos3_usb_phy1_suspend(struct platform_device *pdev)
{	
	if (atomic_dec_return(&usb_phy_control.sharedphyused) > 0) {	
		dev_info(&pdev->dev, "phy0 still being used,suspend failed.\n");	
		return -EBUSY;	
	}
	/* set to suspend phy0 */
	writel(readl(EXYNOS3_PHYPWR)|PHY1_STD_FORCE_SUSPEND
		, EXYNOS3_PHYPWR);	
	return 0;
}
static int exynos3_usb_phy1_resume(struct platform_device *pdev)
{

	int sharedphyused =  atomic_read(&usb_phy_control.sharedphyused);

	atomic_inc(&usb_phy_control.sharedphyused);	
	if(!sharedphyused){
		writel(readl(EXYNOS3_PHYPWR)&~PHY1_STD_FORCE_SUSPEND
			, EXYNOS3_PHYPWR);
	}
	return 0;
}

static int exynos3_usb_hsic_suspend(struct platform_device *pdev)
{
	/* set to normal of Host */	
	writel(readl(EXYNOS3_PHYPWR)|PHY1_HSIC0_FORCE_SUSPEND
		, EXYNOS3_PHYPWR);
	printk("%s:complete\n",__func__);
	return 0;
}
static int exynos3_usb_hsic_resume(struct platform_device *pdev)
{
	/* set to normal of Host */	
	writel(readl(EXYNOS3_PHYPWR)&~PHY1_HSIC0_FORCE_SUSPEND
		, EXYNOS3_PHYPWR);
	msleep(30);
	printk("%s:complete\n",__func__);
	
	return 0;
}
#if 0
static int exynos3_usb_dump_register(void)
{
	printk("power=%x\n",readl(EXYNOS3_PHYPWR));	
	printk("clk=%x\n",readl(EXYNOS3_PHYCLK));
	printk("rescon=%x\n",readl(EXYNOS3_RSTCON));
	printk("phy0=%x,phy1=%x\n",readl(EXYNOS3_USB_PHY_CONTROL),
		readl(EXYNOS3_HSIC1_PHY_CONTROL));
	return 0;
	
}
#endif
static int exynos3_usb_hsic_init(struct platform_device *pdev)
{
	u32 rstcon, phypwr;

	exynos_usb_phy_control(USB_PHY_HSIC0,
		PHY_ENABLE);
	exynos3_usb_phy_clock_init(pdev);
	/* set to normal of Host */	
	 /* only enale hsic0 power cool_zhao*/
	phypwr = readl(EXYNOS3_PHYPWR);	
	phypwr &= ~(/*PHY1_STD_NORMAL_MASK	   
		| */EXYNOS3472_HSIC0_NORMAL_MASK);	
	writel(phypwr, EXYNOS3_PHYPWR);

	/* reset both PHY and Link of Host */
	rstcon = readl(EXYNOS3_RSTCON)
		| EXYNOS3472_PHY1_HSIC0_SWRST;
	writel(rstcon, EXYNOS3_RSTCON);
	udelay(10);

	rstcon &= ~(EXYNOS3472_PHY1_HSIC0_SWRST);
	writel(rstcon, EXYNOS3_RSTCON);
	mdelay(10);
	return 0;
}

static int exynos3_usb_hsic_exit(struct platform_device *pdev)
{
	/* unset to normal of PHY0 */
	writel((readl(EXYNOS3_PHYPWR) | EXYNOS3472_HSIC0_NORMAL_MASK),
		EXYNOS3_PHYPWR);
	exynos_usb_phy_control(USB_PHY_HSIC0,
		PHY_DISABLE);
	exynos3_usb_phy_clock_exit(pdev);
	return 0;
}


static int exynos_usb_dev_init(struct platform_device *pdev)
{
	exynos3_usb_phy0_init(pdev);
	exynos_usb_mux_change(pdev, 0);
	return 0;
}


static int exynos_usb_dev_exit(struct platform_device *pdev)
{
	exynos3_usb_phy0_exit(pdev);
	exynos_usb_mux_change(pdev, 1);
	return 0;
}

int exynos3_usb_phy_host_suspend(struct platform_device *pdev)
{
	exynos3_usb_phy1_suspend(pdev);
	if (!strcmp(pdev->name, "s5p-ehci"))
		exynos3_usb_hsic_suspend(pdev);
	return 0;
}
int exynos3_usb_phy_host_resume(struct platform_device *pdev)
{
	if (!strcmp(pdev->name, "s5p-ehci"))
		exynos3_usb_hsic_resume(pdev);
	exynos3_usb_phy1_resume(pdev);
	return 0;
}


int exynos3_usb_phy_host_init(struct platform_device *pdev)
{
#ifndef CONFIG_USB_EHCI_DISABLE_STANDARD_PORT
	exynos_usb_mux_change(pdev, 1);
	exynos3_usb_phy1_init(pdev);
#endif
	if (!strcmp(pdev->name, "s5p-ehci"))
		exynos3_usb_hsic_init(pdev);
	if (timer_sc3_init == 1){
	sc3_usbdev_timer.expires = jiffies +1*HZ;
	if (!timer_pending(&sc3_usbdev_timer))
		add_timer(&sc3_usbdev_timer);
	else
		mod_timer(&sc3_usbdev_timer, jiffies +1*HZ);
	}
	return 0;
}
int exynos3_usb_phy_host_exit(struct platform_device *pdev)
{
	if (!strcmp(pdev->name, "s5p-ehci"))
		exynos3_usb_hsic_exit(pdev);
#ifndef CONFIG_USB_EHCI_DISABLE_STANDARD_PORT
	exynos3_usb_phy1_exit(pdev);
#endif
	return 0;
}

int s5p_usb_phy_suspend(struct platform_device *pdev, int type)
{
	int ret = 0;

	mutex_lock(&usb_phy_control.phy_lock);
	exynos_usb_phy_clock_enable(pdev);
	if (type == S5P_USB_PHY_HOST) {
		if (soc_is_exynos3472())
			ret = exynos3_usb_phy_host_suspend(pdev);
	}
	exynos_usb_phy_clock_disable(pdev);
	mutex_unlock(&usb_phy_control.phy_lock);
	return ret;
}

int s5p_usb_phy_resume(struct platform_device *pdev, int type)
{
	int ret = 0;

	mutex_lock(&usb_phy_control.phy_lock);
	exynos_usb_phy_clock_enable(pdev);
	if (type == S5P_USB_PHY_HOST) {
		if (soc_is_exynos3472())
			ret = exynos3_usb_phy_host_resume(pdev);
	}
	exynos_usb_phy_clock_disable(pdev);
	mutex_unlock(&usb_phy_control.phy_lock);

	return ret;
}
static int ehci_inited_flag = 0;
int s5p_usb_phy_init(struct platform_device *pdev, int type)
{
	int ret = -EINVAL;

	mutex_lock(&usb_phy_control.phy_lock);
 
	/*******if use hub 3503 need reset hub here cool_zhao*****/

	usb_phy_power_on();
	exynos_usb_phy_clock_enable(pdev);
	if (type == S5P_USB_PHY_HOST) {
		if(ehci_inited_flag == 0){
			if (soc_is_exynos3472()){
				set_bit(HOST_PHY_EHCI, &usb_phy_control.phystatus);
				ret = exynos3_usb_phy_host_init(pdev);
			}
			ehci_inited_flag = 1;
		}
	} else if (type == S5P_USB_PHY_DEVICE) {
		if (soc_is_exynos3472()){
			set_bit(HOST_PHY_DEV, &usb_phy_control.phystatus);
			ret = exynos_usb_dev_init(pdev);
		}
	}
	exynos_usb_phy_clock_disable(pdev);
	mutex_unlock(&usb_phy_control.phy_lock);

	return ret;
}

int s5p_usb_phy_exit(struct platform_device *pdev, int type)
{
	int ret = -EINVAL;
	
	mutex_lock(&usb_phy_control.phy_lock);
	exynos_usb_phy_clock_enable(pdev);
	if (type == S5P_USB_PHY_HOST) {
		if(ehci_inited_flag == 1){
			if ( soc_is_exynos3472()){
				ret = exynos3_usb_phy_host_exit(pdev);
				clear_bit(HOST_PHY_EHCI, &usb_phy_control.phystatus);
			}
			ehci_inited_flag = 0;
		}
	} else if (type == S5P_USB_PHY_DEVICE) {
		if (soc_is_exynos3472()){
			clear_bit(HOST_PHY_DEV, &usb_phy_control.phystatus);
			ret = exynos_usb_dev_exit(pdev);
		}
	}
	exynos_usb_phy_clock_disable(pdev);
	usb_phy_power_off();
	mutex_unlock(&usb_phy_control.phy_lock);
	
	return ret;
}
