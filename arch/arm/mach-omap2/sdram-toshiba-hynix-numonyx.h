/*
 * SDRC register values for the Toshiba, Hynxi and Numonyx.
 * These are common SDRC parameters for all vendors since we
 * use the JEDEC JESD209A parameters.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef ARCH_ARM_MACH_OMAP2_SDRAM_TOSHIBA_HYNIX_NUMONYX

#define ARCH_ARM_MACH_OMAP2_SDRAM_TOSHIBA_HYNIX_NUMONYX

#include "sdrc.h"

static struct omap_sdrc_params JEDEC_JESD209A_sdrc_params[] = {
	[0] = {
		.rate        = 200000000,
		.actim_ctrla = 0xE2E1B4C6,
		.actim_ctrlb = 0x00022228,
		.rfr_ctrl    = 0x0005E602,
		.mr          = 0x00000032,
	},
	[1] = {
		.rate        = 100000000,
		.actim_ctrla = 0x7211B485,
		.actim_ctrlb = 0x00022214,
		.rfr_ctrl    = 0x0002DA02,
		.mr          = 0x00000032,
	},
	[2] = {
		.rate        = 166000000,
		.actim_ctrla = 0xE2E1B4C6,
		.actim_ctrlb = 0x00022228,
		.rfr_ctrl    = 0x0004DD02,
		.mr          = 0x00000032,
	},
	[3] = {
		.rate        = 83000000,
		.actim_ctrla = 0x7215B485,
		.actim_ctrlb = 0x00022214,
		.rfr_ctrl    = 0x00025602,
		.mr          = 0x00000032,
	},
	[4] = {
		.rate        = 160000000,
		.actim_ctrla = 0xBA9DB4C6,
		.actim_ctrlb = 0x00022220,
		.rfr_ctrl    = 0x0004AE02,
		.mr	     = 0x00000032,
	},
	[5] = {
		.rate        = 80000000,
		.actim_ctrla = 0x49512284,
		.actim_ctrlb = 0x0001120C,
		.rfr_ctrl    = 0x23E02,
		.mr	     = 0x00000032,
	},
	[6] = {
		.rate        = 0
	},
};

#endif

