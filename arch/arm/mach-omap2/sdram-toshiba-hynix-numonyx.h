/*
 * SDRC register values for the Toshiba, Hynxi and Numonyx.
 * These are common SDRC parameters for all vendors since we use the JEDEC JESD209A parameters.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARCH_ARM_MACH_OMAP2_SDRAM_TOSHIBA_HYNIX_NUMONYX

#define ARCH_ARM_MACH_OMAP2_SDRAM_TOSHIBA_HYNIX_NUMONYX

#include <plat/sdrc.h>

static struct omap_sdrc_params JEDEC_JESD209A_sdrc_params[] = {
	[0] = {
		.rate        = 200000000,
		.actim_ctrla = 0x92E1C4C6,
		.actim_ctrlb = 0x0002431C,
		.rfr_ctrl    = 0x0005E602,
		.mr          = 0x00000032,
	},
	[1] = {
		.rate        = 100000000,
		.actim_ctrla = 0x49912283,
		.actim_ctrlb = 0x0002430E,
		.rfr_ctrl    = 0x0002DA02,
		.mr          = 0x00000032,
	},
	[2] = {
		.rate        = 160000000,
		.actim_ctrla = 0xBA9DB4C6,
		.actim_ctrlb = 0x00022220,
		.rfr_ctrl    = 0x0004AE02,
		.mr	     = 0x00000032,
	},
	[3] = {
		.rate        = 80000000,
		.actim_ctrla = 0x49512284,
		.actim_ctrlb = 0x0001120C,
		.rfr_ctrl    = 0x23E02,
		.mr	     = 0x00000032,
	},
	[4] = {
		.rate        = 0
	},
};

#endif

