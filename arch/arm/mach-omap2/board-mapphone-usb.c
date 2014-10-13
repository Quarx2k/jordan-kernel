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
#include <mach/hardware.h>
#include <linux/usb/otg.h>
#include <linux/usb/composite.h>
#include <linux/usb/musb.h>
#include <linux/usb/phy.h>
#include <linux/usb/nop-usb-xceiv.h>
#include "usb.h"

#if defined(CONFIG_USB_MUSB_OTG)
#include <linux/spi/cpcap.h>
#include <linux/usb/musb.h>
#include <linux/usb/composite.h>
#endif

static struct usbhs_omap_platform_data usbhs_bdata  = {
	.phy_reset  = false,
	.es2_compatibility = false,
	.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_EHCI_PORT_MODE_TLL,
};

void mapphone_gadget_init(void)
{
	usb_bind_phy("musb-hdrc.1.auto", 0, "cpcap_usb");
	usb_musb_init(NULL);
	usbhs_init(&usbhs_bdata);
}

