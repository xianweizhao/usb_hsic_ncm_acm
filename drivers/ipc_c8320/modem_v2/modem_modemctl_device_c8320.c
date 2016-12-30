/* /linux/drivers/misc/modem_if/modem_modemctl_device_c8320.c
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

#define DEBUG

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <plat/devs.h>
#include <linux/platform_data/modem.h>
#include "modem_prj.h"
#include <linux/if_arp.h>
/* here we can use this irq to switch CP STATE BOOT NORMAL or BLUESCREEN cool_zhao */
static irqreturn_t phone_switch_irq_handler(int irq, void *_mc)
{

	struct modem_ctl *mc = (struct modem_ctl *)_mc;
	int val = 0;
	int err;
	
	if((c8320_modem_ctl_ex.phone_state == STATE_RESET_CP)||(c8320_modem_ctl_ex.phone_state == STATE_OFFLINE)){
		mif_info("cp crash or cp offine is do nothing\n");
		return IRQ_HANDLED;
	}

	val = gpio_get_value(mc->gpio_cp_switch);
	mif_info("CP>>AP GPIO_CP_SWITCH:%d\n", val);

	if (val != CP_DEV_SW_LEVEL) {

		printk("AP>>CP  PHONE_ACTIVE: 0\n");
		gpio_direction_output(mc->gpio_phone_active, 0);	
		return IRQ_HANDLED;
	}
	/* in test mode only use switch usb device ,here need reset host  and set host_active to CP */
	if (val == CP_DEV_SW_LEVEL) {
		if(c8320_modem_ctl_ex.phone_state == STATE_CRASH){
			c8320_modem_ctl_ex.debugmask = c8320_modem_ctl_ex.debugmask| FORBID_SWITCH_USBDEV;
		}
		schedule_work(&mc->work);
		printk("%s:reset host hcd\n",__func__);		
	}
	return IRQ_HANDLED;
}


void c8320_start_loopback(struct io_device *iod, struct modem_shared *msd)
{
	struct link_device *ld = get_current_link(iod);
	struct sk_buff *skb = alloc_skb(16, GFP_ATOMIC);

	if (unlikely(!skb))
		return;
	mif_info("Send loopback key '%s'\n",
					(msd->loopback_ipaddr) ? "s" : "x");

}

extern int s5p_ehci_power(struct platform_device *pdev, int value);
extern int c83xx_usb_power_on(void);
int exynos_ss_udc_session(int onoff);
static void c8320_usb_switch_work(struct work_struct *data)
{
	struct modem_ctl *mc = container_of(data, struct modem_ctl, work);
	int value = 0;

	c83xx_usb_power_on();
	//exynos_ss_udc_session(0);
	s5p_ehci_power(mc->peripheral_platform_device_ehci, 0);
	msleep(1);
	s5p_ehci_power(mc->peripheral_platform_device_ehci, 1);
	//exynos_ss_udc_session(1);

	value = gpio_get_value(mc->gpio_phone_active);
	if(value == 1)
	{
		mif_info("ACTIVE is high need set AP>>CP:  GPIO_PHONE_ACTIVE 0\n");
		gpio_direction_output(mc->gpio_phone_active, 0);
		msleep(1);
	}
	mif_info("AP>>CP:  GPIO_PHONE_ACTIVE 1\n");
	gpio_direction_output(mc->gpio_phone_active, 1);
}
int c8320_init_modemctl_device(struct modem_ctl *mc,
			struct modem_data *pdata)
{
	int ret = 0;
	struct platform_device *pdev;

	mc->gpio_cp_on = pdata->gpio_cp_on;
	mc->gpio_cp_switch = pdata->gpio_cp_switch;
	mc->gpio_phone_active = pdata->gpio_phone_active;	
	mc->gpio_revers_bias_clear = pdata->gpio_revers_bias_clear;
	mc->gpio_revers_bias_restore = pdata->gpio_revers_bias_restore;
	INIT_WORK(&mc->work, c8320_usb_switch_work);
	mc->peripheral_platform_device_ehci = &s5p_device_ehci;
	pdev = to_platform_device(mc->dev);
	
	mc->irq_cp_switch = gpio_to_irq(mc->gpio_cp_switch);	

//	mc->msd->loopback_start = c8320_start_loopback;

	mif_info(" request cp_switch irq\n");
	ret = request_irq(mc->irq_cp_switch, phone_switch_irq_handler,
				IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"cp_switch", mc);
	if (ret) {
		mif_err("failed to request_irq:%d\n", ret);
		goto err_phone_active_request_irq;
	}


	ret = enable_irq_wake(mc->irq_cp_switch);
	if (ret) {
		mif_err("failed to irq_cp_switch:%d\n", ret);
		goto err_phone_active_set_wake_irq;
	}
	/* 
	 * initialize sim_state if gpio_sim_detect exists 
	 * if we need sim detect, TODO init cool_zhao
	 */

	return ret;
err_phone_active_set_wake_irq:
	free_irq(mc->irq_cp_switch, mc);
err_phone_active_request_irq:
	return ret;
}
