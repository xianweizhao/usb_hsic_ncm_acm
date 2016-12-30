/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <mach/usb-switch.h>
#include <plat/gpio-cfg.h>
#include <plat/ehci.h>
#include <plat/devs.h>
#include <plat/udc-hs.h>


static struct s5p_ohci_platdata smdk3472_ohci_pdata __initdata;
static struct s5p_ehci_platdata smdk3472_ehci_pdata __initdata;
static struct s3c_hsotg_plat smdk3472_hsotg_pdata __initdata;
static struct s5p_usbswitch_platdata smdk3472_usbswitch_pdata;

static void __init smdk3472_ohci_init(void)
{
	s5p_ohci_set_platdata(&smdk3472_ohci_pdata);
}

static void __init smdk3472_ehci_init(void)
{
	s5p_ehci_set_platdata(&smdk3472_ehci_pdata);
}

static void __init smdk3472_gadget_init(void)
{
	s3c_hsotg_set_platdata(&smdk3472_hsotg_pdata);
}
static void __init smdk3472_usbswitch_init(void)
{
	struct s5p_usbswitch_platdata *pdata = &smdk3472_usbswitch_pdata;
	int err;
#if defined(CONFIG_USB_EHCI_S5P)||defined(CONFIG_USB_OHCI_EXYNOS)
	pdata->gpio_host_detect = GPIO_USBHOST_DETECTED;
	err = gpio_request_one(pdata->gpio_host_detect, GPIOF_IN,
		"HOST_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request host gpio\n");
		return;
	}
	samsung_gpio_cfgpin(pdata->gpio_host_detect, SAMSUNG_GPIO_SFN(0xF));
	samsung_gpio_setpull(pdata->gpio_host_detect, SAMSUNG_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_host_detect);
	pdata->gpio_host_vbus = GPIO_USB_VUBS;
	err = gpio_request_one(pdata->gpio_host_vbus,
		GPIOF_OUT_INIT_LOW,
		"HOST_VBUS_CONTROL");
	if (err) {
		printk(KERN_ERR "failed to request host_vbus gpio\n");
		return;
	}

	samsung_gpio_setpull(pdata->gpio_host_vbus, SAMSUNG_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_host_vbus);
#endif
#ifdef CONFIG_USB_GADGET
	pdata->gpio_device_detect = GPIO_USBDEVICE_DETECTED;
	err = gpio_request_one(pdata->gpio_device_detect, GPIOF_IN,
		"DEVICE_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request device gpio\n");
		return;
	}

	samsung_gpio_cfgpin(pdata->gpio_device_detect, SAMSUNG_GPIO_SFN(0xF));
	samsung_gpio_setpull(pdata->gpio_device_detect, SAMSUNG_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_device_detect);
	pdata->s3c_udc_dev=&s3c_device_usb_hsotg.dev;
#endif
#ifdef CONFIG_USB_EHCI_S5P
	pdata->ehci_dev = &s5p_device_ehci.dev;
#endif
#ifdef CONFIG_USB_OHCI_EXYNOS
	pdata->ohci_dev = &s5p_device_ohci.dev;
#endif
	s5p_usbswitch_set_platdata(pdata);
}
static struct platform_device *smdk3472_usb_devices[] __initdata = {
#ifdef CONFIG_USB_EHCI_S5P
	&s5p_device_ehci,
#endif
#ifdef CONFIG_USB_OHCI_EXYNOS
	&s5p_device_ohci,
#endif
#ifdef CONFIG_USB_GADGET
	&s3c_device_usb_hsotg,
#endif
};

void __init exynos_smdk3472_usb_init(void)
{
#ifdef CONFIG_USB_OHCI_EXYNOS
	smdk3472_ohci_init();
#endif
#ifdef CONFIG_USB_EHCI_S5P
	smdk3472_ehci_init();
#endif
#ifdef CONFIG_USB_GADGET
	smdk3472_gadget_init();
#endif
#ifdef CONFIG_USB_EXYNOS_SWITCH
	smdk3472_usbswitch_init();
#endif
	platform_add_devices(smdk3472_usb_devices,
			ARRAY_SIZE(smdk3472_usb_devices));
}
