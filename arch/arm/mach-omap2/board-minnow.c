/*
 * linux/arch/arm/mach-omap2/board-minnow.c
 *
 * Copyright (C) 2013 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/input/touch_platform.h>
#include <linux/usb/musb.h>
#include <linux/usb/phy.h>
#include <linux/usb/nop-usb-xceiv.h>
#include "board-minnow.h"
#include "mux.h"
#include "common.h"
#include "dss-common.h"
#include "board-minnow.h"

#include "sdram-toshiba-hynix-numonyx.h"

static struct of_device_id omap_dt_match_table[] __initdata = {
	{ .compatible = "simple-bus", },
	{ .compatible = "ti,omap-infra", },
	{ }
};

static const char *omap3_gp_boards_compat[] __initdata = {
	"mot,omap3-minnow",
	NULL,
};
static void __init minnow_musb_init(void)
{
	usb_bind_phy("musb-hdrc.1.auto", 0, "cpcap_usb");
	usb_musb_init(NULL);
}

static void __init minnow_init(void)
{
	of_platform_populate(NULL, omap_dt_match_table, NULL, NULL);

	omap_sdrc_init(JEDEC_JESD209A_sdrc_params, JEDEC_JESD209A_sdrc_params);
	omap_minnow_display_init();
	minnow_spi_init();
	minnow_bt_init();
	minnow_sensors_init();
	minnow_cpcap_client_init();
	minnow_musb_init();
}

MACHINE_START(MINNOW, "minnow")
	.atag_offset    = 0x100,
	.reserve        = omap_reserve,
	.map_io         = omap3_map_io,
	.init_early     = omap3630_init_early,
	.init_irq       = omap_intc_of_init,
	.handle_irq     = omap3_intc_handle_irq,
	.init_machine   = minnow_init,
	.init_late      = omap3630_init_late,
	.init_time      = omap3_sync32k_timer_init,
	.dt_compat	    = omap3_gp_boards_compat,
	.restart        = omap3xxx_restart,
MACHINE_END
