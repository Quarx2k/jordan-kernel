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
#include <linux/mtd/nand.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "common.h"
#include "gpmc-smc91x.h"

#include "board-zoom.h"

#include "board-flash.h"
#include "mux.h"
#include "sdram-toshiba-hynix-numonyx.h"
static void __init omap_sdp_init(void)
{
	omap2_sdrc_init(JEDEC_JESD209A_sdrc_params,
				   JEDEC_JESD209A_sdrc_params);
}

MACHINE_START(MAPPHONE, "mapphone_")
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3630_init_early,
	.init_irq	= omap3_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= omap_sdp_init,
	.init_late	= omap3630_init_late,
	.init_time	= omap3_sync32k_timer_init,
	.restart	= omap3xxx_restart,
MACHINE_END
