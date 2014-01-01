/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/mtd/nand.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "common.h"
#include <plat/board.h>
#include <plat/usb.h>

#include <mach/board-zoom.h>

#include "board-flash.h"
#include "mux.h"
#include "sdram-toshiba-hynix-numonyx.h"

static void __init omap_mapphone_init_early(void)
{
	omap3630_init_early();
	omap_sdrc_init(JEDEC_JESD209A_sdrc_params,
				   JEDEC_JESD209A_sdrc_params);
}

static void __init omap_mapphone_init(void)
{

}

MACHINE_START(MAPPHONE, "mapphone_")
	.atag_offset	= 0x80C00100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap_mapphone_init_early,
	.init_irq	= omap3_init_irq,
	.init_machine	= omap_mapphone_init,
	.timer		= &omap3_timer,
	.restart	= omap_prcm_restart,
MACHINE_END

