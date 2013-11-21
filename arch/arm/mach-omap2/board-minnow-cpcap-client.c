/*
 * arch/arm/mach-omap2/board-minnow-cpcap-client.c
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spi/cpcap.h>
#include "omap34xx.h"

/*
 * CPCAP devcies are common for different HW Rev.
 *
 */


static struct platform_device cpcap_usb_device = {
	.name           = "cpcap_usb",
	.id             = -1,
	.dev.platform_data = NULL,
};
static struct platform_device cpcap_usb_det_device = {
	.name   = "cpcap_usb_det",
	.id     = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};

static struct platform_device *cpcap_devices[] = {
	&cpcap_usb_device,
	&cpcap_usb_det_device,
};


void __init minnow_cpcap_client_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cpcap_devices); i++)
		cpcap_device_register(cpcap_devices[i]);

}
