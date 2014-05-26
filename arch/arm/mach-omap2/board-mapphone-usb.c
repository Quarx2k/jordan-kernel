/*
 * kernel/arch/arm/mach-omap2/board-mapphone-usb.c
 *
 * Copyright (C) 2010-2012 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <asm/mach-types.h>
#include <linux/of.h>
#include "dt_path.h"
#include <mach/board-mapphone.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <linux/gpio_mapping.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/composite.h>
#include <plat/omap-pm.h>

#if defined(CONFIG_USB_MUSB_OTG)
#include <linux/spi/cpcap.h>
#include <linux/usb/musb.h>
#include <linux/usb/composite.h>
#endif

#include <plat/common.h>
#include "cm-regbits-34xx.h"
#include "clock.h"
#include "dvfs.h"

#define MAPPHONE_BP_READY2_AP_GPIO      59
#define MAPPHONE_IPC_USB_SUSP_GPIO	142 //95
#define DIE_ID_REG_BASE			(L4_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET		0x218

void cpcap_musb_notifier_call(unsigned long event);

static struct platform_device android_usb_platform_device = {
	.name	= "android_gadget",
	.id	= -1,
	.dev	= {
	},
};

static int cpcap_usb_connected_probe(struct platform_device *pdev)
{
	printk("USB Connected!\n");
	cpcap_musb_notifier_call(USB_EVENT_VBUS);
	return 0;
}

static int cpcap_usb_connected_remove(struct platform_device *pdev)
{
	printk("USB Disconnected!\n");
	cpcap_musb_notifier_call(USB_EVENT_NONE);
	return 0;
}

static struct platform_driver cpcap_usb_connected_driver = {
	.probe		= cpcap_usb_connected_probe,
	.remove		= cpcap_usb_connected_remove,
	.driver		= {
		.name	= "cpcap_usb_connected",
		.owner	= THIS_MODULE,
	},
};

void mapphone_gadget_init(void)
{
	platform_driver_register(&cpcap_usb_connected_driver);
	platform_device_register(&android_usb_platform_device);

}

static struct usbhs_omap_board_data usbhs_bdata  = {
	.phy_reset  = false,
	.ehci_phy_vbus_not_used = false,
	.es2_compatibility = false,
	.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_EHCI_PORT_MODE_TLL,
};


void __init mapphone_usbhost_init(void)
{
	usbhs_init(&usbhs_bdata);
}
