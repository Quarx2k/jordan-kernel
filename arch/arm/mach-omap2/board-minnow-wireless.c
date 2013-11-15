/*
 * linux/arch/arm/mach-omap2/board-minnow-wireless.c
 *
 * Copyright (C) 2013 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/wl127x-rfkill.h>
#include "common.h"



void change_vio_mode(int source, int value)
{
	printk(KERN_DEBUG "Entering change_vio_mode\n");
#if 0
	static int bt_request;
    static int wifi_request;

    /*If have DC-DC converter, skip it*/
    if (misc_cpcap == NULL)
		return;

    mutex_lock(&vio_access);
    if (source == 0)
		bt_request = value;
    else if (source == 1)
		wifi_request = value;
    else {
		printk(KERN_ERR "unknown source to vio ");
		mutex_unlock(&vio_access);
		return;
    }

    if (bt_request | wifi_request)
		cpcap_regacc_write(misc_cpcap, CPCAP_REG_S3C,
			0, CPCAP_BIT_SW3STBY);
    else
		cpcap_regacc_write(misc_cpcap, CPCAP_REG_S3C,
			CPCAP_BIT_SW3STBY, CPCAP_BIT_SW3STBY);

    mutex_unlock(&vio_access);
#endif
}


static int minnow_wl1271_init(void);
static int minnow_wl1271_release(void);
static int minnow_wl1271_enable(void);
static int minnow_wl1271_disable(void);

static struct wl127x_rfkill_platform_data minnow_wl1271_pdata = {
    .bt_nshutdown_gpio = -1,
    .fm_enable_gpio = -1,
    .bt_hw_init = minnow_wl1271_init,
    .bt_hw_release = minnow_wl1271_release,
    .bt_hw_enable = minnow_wl1271_enable,
    .bt_hw_disable = minnow_wl1271_disable,
};

static int minnow_wl1271_init(void)
{
    return 0;
}

static int minnow_wl1271_release(void)
{
    return 0;
}

static int minnow_wl1271_enable(void)
{
    /* FIXME
     * Change vio mode dynamically if necessary
     */
    change_vio_mode(0, 1);
    return 0;
}

static int minnow_wl1271_disable(void)
{
    /* FIXME
     * Change vio mode dynamically if necessary
     */
    change_vio_mode(0, 0);
    return 0;
}

static struct platform_device minnow_wl1271_device = {
    .name = "wl127x-rfkill",
    .id = 0,
    .dev.platform_data = &minnow_wl1271_pdata,
};

void __init minnow_bt_init(void)
{
	/* TODO use device tree once ported */
    minnow_wl1271_pdata.bt_nshutdown_gpio = 83;

	platform_device_register(&minnow_wl1271_device);
}
