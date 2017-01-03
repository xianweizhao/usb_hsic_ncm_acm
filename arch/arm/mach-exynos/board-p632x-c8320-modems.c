/* linux/arch/arm/mach-xxxx/board-p632x-c8320-modems.c
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* Modem configuraiton for CM1 (p632x)*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/ehci_def.h>

#include <linux/platform_data/modem.h>
#include <mach/sec_modem.h>
#include <linux/io.h>
#include <mach/map.h>
#include <mach/regs-pmu.h>
#include <mach/regs-usb-phy.h>


#define EHCI_REG_DUMP
#define BS02_Z

/* 4 liner power control */

#ifdef BS02_Z
#ifdef CONFIG_LINK_DEVICE_POWER_MANGER
#define GPIO_IPC_SLAVE_WAKEUP EXYNOS3_GPX0(5)
#define GPIO_HOST_ACTIVE EXYNOS3_GPX3(5)
#define GPIO_IPC_HOST_WAKEUP EXYNOS3_GPX2(2)
#define GPIO_SUSPEND_REQUEST EXYNOS3_GPX3(1)
#else

#define GPIO_IPC_SLAVE_WAKEUP 0
#define GPIO_HOST_ACTIVE 0
#define GPIO_IPC_HOST_WAKEUP 0
#define GPIO_SUSPEND_REQUEST 0

#endif

#define	GPIO_CP_SWITCH          EXYNOS3_GPX2(3)
#define GPIO_PHONE_ACTIVE	EXYNOS3_GPX2(5)
#endif

/* umts target platform data */
static struct modem_io_t umts_io_devices[] = {
	[0] = {
		.name = "usb_nvm",
		.id = 0,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_HSIC),
	},
	[1] = {
		.name = "usb_vm",
		.id = 1,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_HSIC),
	},
	[2] = {
		.name = "usb_trace",
		.id = 2,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_HSIC),
	},

	[3] = {
		.name = "usb0",
		.id = 3,
		.format = IPC_RAW_NCM,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_HSIC),
	},
	[4] = {
		.name = "usb1",
		.id = 4,
		.format = IPC_RAW_NCM,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_HSIC),
	},
#if 0

	[5] = {
		.name = "bootcp",
		.id = 0xff,
		.format = IPC_BOOT,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_HSIC),
	},
	[6] = {
		.name = "mc_dump",
		.id = 0xff,
		.format = IPC_FMT,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_HSIC),
	},
#endif
};

static int umts_link_ldo_enble(bool enable)
{
	/* Exynos HSIC V1.2 LDO was controlled by kernel */
	return 0;
}

struct mif_ehci_regs {
	unsigned caps_hc_capbase;
	unsigned caps_hcs_params;
	unsigned caps_hcc_params;
	unsigned reserved0;
	struct ehci_regs regs;
	unsigned port_usb;  /*0x54*/
	unsigned port_hsic0;
	unsigned port_hsic1;
	unsigned reserved[12];
	unsigned insnreg00;	/*0x90*/
	unsigned insnreg01;
	unsigned insnreg02;
	unsigned insnreg03;
	unsigned insnreg04;
	unsigned insnreg05;
	unsigned insnreg06;
	unsigned insnreg07;
};
static struct mif_ehci_regs __iomem *ehci_reg;

#ifdef EHCI_REG_DUMP
#define pr_reg(s, r) printk("reg(%s):\t 0x%08x\n", s, r)
static void print_ehci_regs(struct mif_ehci_regs *base)
{
	pr_info("------- EHCI reg dump -------\n");
	pr_reg("HCCPBASE", base->caps_hc_capbase);
	pr_reg("HCSPARAMS", base->caps_hcs_params);
	pr_reg("HCCPARAMS", base->caps_hcc_params);
	pr_reg("USBCMD", base->regs.command);
	pr_reg("USBSTS", base->regs.status);
	pr_reg("USBINTR", base->regs.intr_enable);
	pr_reg("FRINDEX", base->regs.frame_index);
	pr_reg("CTRLDSSEGMENT", base->regs.segment);
	pr_reg("PERIODICLISTBASE", base->regs.frame_list);
	pr_reg("ASYNCLISTADDR", base->regs.async_next);
	pr_reg("CONFIGFLAG", base->regs.configured_flag);
	pr_reg("PORT0 Status/Control", base->port_usb);
	pr_reg("PORT1 Status/Control", base->port_hsic0);
	pr_reg("PORT2 Status/Control", base->port_hsic1);
	pr_reg("INSNREG00", base->insnreg00);
	pr_reg("INSNREG01", base->insnreg01);
	pr_reg("INSNREG02", base->insnreg02);
	pr_reg("INSNREG03", base->insnreg03);
	pr_reg("INSNREG04", base->insnreg04);
	pr_reg("INSNREG05", base->insnreg05);
	pr_reg("INSNREG06", base->insnreg06);
	pr_reg("INSNREG07", base->insnreg07);
	pr_info("-----------------------------\n");
}
#define EXYNOS_USB_CFG		(S3C_VA_SYS + 0x21C)
static void print_phy_regs(void)
{
	pr_info("----- EHCI PHY REG DUMP -----\n");
#if 1
	pr_reg("EXYNOS3_HSIC1_PHY_CONTROL", readl(EXYNOS3_HSIC1_PHY_CONTROL));
	pr_reg("EXYNOS3_USB_PHY_CONTROL", readl(EXYNOS3_USB_PHY_CONTROL));
	pr_reg("EXYNOS3_PHYPWR", readl(EXYNOS3_PHYPWR));
	pr_reg("EXYNOS3_PHYCLK", readl(EXYNOS3_PHYCLK));
	pr_reg("EXYNOS3_RSTCON", readl(EXYNOS3_RSTCON));
//	pr_reg("UPHYTUNE0", readl(UPHYTUNE0));
//	pr_reg("UPHYTUNE0", readl(UPHYTUNE1));	
	pr_reg("EXYNOS_USB_CFG", readl(EXYNOS_USB_CFG));
#endif
	pr_info("-----------------------------\n");
}

void debug_ehci_reg_dump(void)
{
	print_phy_regs();
	print_ehci_regs(ehci_reg);
}
#else
#define debug_ehci_reg_dump() do {} while(0);
#endif

#define WAIT_CONNECT_CHECK_CNT 30
static int s5p_ehci_port_reg_init(void)
{
	if (ehci_reg) {
		mif_info("port reg aleady initialized\n");
		return -EBUSY;
	}

	ehci_reg = ioremap((S5P_PA_EHCI), SZ_256);
	if (!ehci_reg) {
		mif_err("fail to get port reg address\n");
		return -EINVAL;
	}
	mif_info("port reg get success (%p)\n", ehci_reg);

	return 0;
}

static void s5p_ehci_wait_cp_resume(int port)
{
	u32 __iomem *portsc;
	int cnt = WAIT_CONNECT_CHECK_CNT;
	u32 val;

	if (!ehci_reg) {
		mif_err("port reg addr invalid\n");
		return;
	}
	portsc = &ehci_reg->port_usb + (port - 1);

	do {
		msleep(20);
		val = readl(portsc);
		mif_info("port(%d), reg(0x%x)\n", port, val);
	} while (cnt-- && !(val & PORT_CONNECT));
#ifdef EHCI_REG_DUMP
	if (!(val & PORT_CONNECT))
		debug_ehci_reg_dump();
#endif
}

static int umts_link_reconnect(void);
static struct modemlink_pm_data modem_link_pm_data = {
	.name = "link_pm",
	.link_ldo_enable = umts_link_ldo_enble,
	.gpio_link_enable = 0,
	.gpio_link_active = GPIO_HOST_ACTIVE,
	.gpio_link_hostwake = GPIO_IPC_HOST_WAKEUP,
	.gpio_link_slavewake = GPIO_IPC_SLAVE_WAKEUP,
	.gpio_link_suspend_req = GPIO_SUSPEND_REQUEST,
	.link_reconnect = umts_link_reconnect,
	.wait_cp_resume = s5p_ehci_wait_cp_resume,
};

static struct modemlink_pm_link_activectl active_ctl;

static void c8320_gpio_revers_bias_clear(void);
static void c8320_gpio_revers_bias_restore(void);

#define MAX_CDC_ACM_CH 3
#define MAX_CDC_NCM_CH 2
static struct modem_data umts_modem_data = {
	.name = "c8320",

	.gpio_cp_switch = GPIO_CP_SWITCH,  		/* cp device switch CP2AP by cool_zhao*/
	.gpio_phone_active = GPIO_PHONE_ACTIVE,	/* phone active is the same is reset cp usb phy by cool_zhao */

	.modem_type = CYIT_C8320,
	.link_types = LINKTYPE(LINKDEV_HSIC),
	.use_handover = false,

	.num_iodevs = ARRAY_SIZE(umts_io_devices),
	.iodevs = umts_io_devices,

	.link_pm_data = &modem_link_pm_data,
	.gpio_revers_bias_clear = c8320_gpio_revers_bias_clear,
	.gpio_revers_bias_restore = c8320_gpio_revers_bias_restore,
	.max_link_channel = MAX_CDC_ACM_CH + MAX_CDC_NCM_CH,
	.max_acm_channel = MAX_CDC_ACM_CH,
	.ipc_version = SIPC_VER_10,
};
void set_host_states(int type)	
{
	if (active_ctl.gpio_initialized) {
		mif_info("Active States =%d\n", type);
#ifdef CONFIG_LINK_DEVICE_POWER_MANGER
		mif_info("AP>>CP: hostactive =%d\n", type);
		gpio_direction_output(modem_link_pm_data.gpio_link_active,
			type);
#endif
	}
}


/* this function is normal mode, usb link reconnet, how to do it ? by cool_zhao */
static int umts_link_reconnect(void)
{
	mif_info("\n");
	if (gpio_get_value(umts_modem_data.gpio_phone_active) &&
		gpio_get_value(umts_modem_data.gpio_cp_switch)) {
		mif_info("trying reconnect link\n");
/* when reset resume is failed, this function is called by disconnect function driver and reconnect CP  by cool_zhao*/
#ifdef CONFIG_LINK_DEVICE_POWER_MANGER
		gpio_set_value(modem_link_pm_data.gpio_link_active, 0);
		mdelay(10);
		gpio_set_value(modem_link_pm_data.gpio_link_active, 1);
#endif
	} else
		return -ENODEV;

	return 0;
}

/* if use more than one modem device, then set id num */
static struct platform_device umts_modem = {
	.name = "mif_sipc5",
	.id = -1,
	.dev = {
		.platform_data = &umts_modem_data,
	},
};

static void umts_modem_cfg_gpio(void)
{
	int ret = 0;

	
	unsigned gpio_cp_switch = umts_modem_data.gpio_cp_switch;
	unsigned gpio_phone_active = umts_modem_data.gpio_phone_active;

	if (gpio_cp_switch && gpio_cp_switch != GPIO_SUSPEND_REQUEST) {
		ret = gpio_request(gpio_cp_switch, "CP_D_SW");
		if (ret)
			mif_err("fail to request gpio %s:%d\n", "CP_D_SW", ret);
		gpio_direction_input(gpio_cp_switch);
	}

	if (gpio_phone_active && gpio_phone_active != GPIO_HOST_ACTIVE) {
		ret = gpio_request(gpio_phone_active, "PHONE_ACTIVE");
		if (ret)
			mif_err("fail to request gpio %s:%d\n", "PHONE_ACTIVE",
				ret);
		
		gpio_direction_output(gpio_phone_active, 0);
		samsung_gpio_setpull(gpio_phone_active,SAMSUNG_GPIO_PULL_NONE);
	}


	/* set low unused gpios between AP and CP */
	
	mif_info("c8320_modem_cfg_gpio done\n");
}

static void c8320_gpio_revers_bias_clear(void)
{
	gpio_direction_output(umts_modem_data.gpio_phone_active, 0);

#ifdef CONFIG_LINK_DEVICE_POWER_MANGER
	gpio_direction_output(modem_link_pm_data.gpio_link_active, 1);
	gpio_direction_output(modem_link_pm_data.gpio_link_hostwake, 0);
	gpio_direction_output(modem_link_pm_data.gpio_link_slavewake, 0);
#endif
	msleep(20);
}

static void c8320_gpio_revers_bias_restore(void)
{
	samsung_gpio_cfgpin(umts_modem_data.gpio_phone_active, SAMSUNG_GPIO_SFN(0xF));
#ifdef CONFIG_LINK_DEVICE_POWER_MANGER
	samsung_gpio_cfgpin(modem_link_pm_data.gpio_link_hostwake,
		SAMSUNG_GPIO_SFN(0xF));
#endif
}

static void modem_link_pm_config_gpio(void)
{
	int ret = 0;

	unsigned gpio_link_enable = modem_link_pm_data.gpio_link_enable;
	unsigned gpio_link_active = modem_link_pm_data.gpio_link_active;
	unsigned gpio_link_hostwake = modem_link_pm_data.gpio_link_hostwake;
	unsigned gpio_link_slavewake = modem_link_pm_data.gpio_link_slavewake;
	unsigned gpio_link_suspend_req = modem_link_pm_data.gpio_link_suspend_req;
	
	if (gpio_link_enable) {
		ret = gpio_request(gpio_link_enable, "LINK_EN");
		if (ret) {
			mif_err("fail to request gpio %s:%d\n", "LINK_EN",
				ret);
		}
		gpio_direction_output(gpio_link_enable, 0);
	}

	if (gpio_link_active) {
		ret = gpio_request(gpio_link_active, "LINK_ACTIVE");
		if (ret) {
			mif_err("fail to request gpio %s:%d\n", "LINK_ACTIVE",
				ret);
		}
		samsung_gpio_cfgpin(gpio_link_active, SAMSUNG_GPIO_OUTPUT);
		samsung_gpio_setpull(gpio_link_active, SAMSUNG_GPIO_PULL_NONE);
		gpio_set_value(gpio_link_active, 1);	
	}

	if (gpio_link_hostwake) {
		ret = gpio_request(gpio_link_hostwake, "HOSTWAKE");
		if (ret) {
			mif_err("fail to request gpio %s:%d\n", "HOSTWAKE",
				ret);
		}
		samsung_gpio_cfgpin(gpio_link_hostwake, SAMSUNG_GPIO_INPUT);
		samsung_gpio_setpull(gpio_link_hostwake, SAMSUNG_GPIO_PULL_NONE);
	}

	if (gpio_link_slavewake) {
		ret = gpio_request(gpio_link_slavewake, "SLAVEWAKE");
		if (ret) {
			mif_err("fail to request gpio %s:%d\n", "SLAVEWAKE",
				ret);
		}
		samsung_gpio_cfgpin(gpio_link_slavewake, SAMSUNG_GPIO_OUTPUT);
		samsung_gpio_setpull(gpio_link_slavewake, SAMSUNG_GPIO_PULL_NONE);
		gpio_set_value(gpio_link_slavewake, 0);
	}
	
	if (gpio_link_suspend_req) {
		ret = gpio_request(gpio_link_suspend_req, "SUSPENDREQ");
		if (ret) {
			mif_err("fail to request gpio %s:%d\n", "SUSPENDREQ",
				ret);
		}
		samsung_gpio_cfgpin(gpio_link_suspend_req, SAMSUNG_GPIO_INPUT);
		samsung_gpio_setpull(gpio_link_suspend_req, SAMSUNG_GPIO_PULL_NONE);
	}

	active_ctl.gpio_initialized = 1;

	mif_info("modem_link_pm_config_gpio done\n");
}


static int __init init_modem(void)
{
	int ret;

	mif_info("init_modem\n");

	/* umts gpios configuration */
	umts_modem_cfg_gpio();
	modem_link_pm_config_gpio();
	s5p_ehci_port_reg_init();
	ret = platform_device_register(&umts_modem);
	if (ret < 0){	
		return ret;
	}
	return ret;
}
late_initcall(init_modem);
