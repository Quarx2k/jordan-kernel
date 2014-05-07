/*
 * drivers/media/video/hp3a/hp3a_hardpipe.c
 *
 * HP Imaging/3A Driver : ISP Hardpipe specific function implementation.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
 *
 * Contributors:
 *		Tanvir Islam <tanvir.islam@hp.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include "hp3a_common.h"
#include "hp3a_ispreg.h"

/**
 * hp3a_update_hardpipe - Updates ISP hardpipe parameters.
 *
 * No return value.
 **/
void hp3a_update_hardpipe(void)
{
	if (g_tc.hw_initialized == 1 && g_tc.update_hardpipe == 1 &&
		!(omap_readl(ISPPRV_PCR) & ISPPRV_PCR_BUSY)) {
		omap_writel(g_tc.hpipe_param.dgain, ISPPRV_WB_DGAIN);

		omap_writel(g_tc.hpipe_param.r_gain |
			g_tc.hpipe_param.gb_gain << ISPPRV_WBGAIN_COEF1_SHIFT |
			g_tc.hpipe_param.gr_gain << ISPPRV_WBGAIN_COEF2_SHIFT |
			g_tc.hpipe_param.b_gain << ISPPRV_WBGAIN_COEF3_SHIFT,
			ISPPRV_WBGAIN);

		/* Keeping the HW default value as such */
		omap_writel(ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N0_0_SHIFT
			| ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N0_1_SHIFT
			| ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N0_2_SHIFT
			| ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N0_3_SHIFT
			| ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N1_0_SHIFT
			| ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N1_1_SHIFT
			| ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N1_2_SHIFT
			| ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N1_3_SHIFT
			| ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N2_0_SHIFT
			| ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N2_1_SHIFT
			| ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N2_2_SHIFT
			| ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N2_3_SHIFT
			| ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N3_0_SHIFT
			| ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N3_1_SHIFT
			| ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N3_2_SHIFT
			| ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N3_3_SHIFT,
			ISPPRV_WBSEL);

		omap_writel((g_tc.hpipe_param.rgb2rgb.matrix[0][0] << \
			ISPPRV_RGB_MAT1_MTX_RR_SHIFT) | \
			(g_tc.hpipe_param.rgb2rgb.matrix[0][1] << \
			ISPPRV_RGB_MAT1_MTX_GR_SHIFT), ISPPRV_RGB_MAT1);

		omap_writel((g_tc.hpipe_param.rgb2rgb.matrix[0][2] << \
			ISPPRV_RGB_MAT2_MTX_BR_SHIFT) | \
			(g_tc.hpipe_param.rgb2rgb.matrix[1][0] << \
			ISPPRV_RGB_MAT2_MTX_RG_SHIFT), ISPPRV_RGB_MAT2);

		omap_writel((g_tc.hpipe_param.rgb2rgb.matrix[1][1] << \
			ISPPRV_RGB_MAT3_MTX_GG_SHIFT) | \
			(g_tc.hpipe_param.rgb2rgb.matrix[1][2] << \
			ISPPRV_RGB_MAT3_MTX_BG_SHIFT), ISPPRV_RGB_MAT3);

		omap_writel((g_tc.hpipe_param.rgb2rgb.matrix[2][0] << \
			ISPPRV_RGB_MAT4_MTX_RB_SHIFT) | \
			(g_tc.hpipe_param.rgb2rgb.matrix[2][1] << \
			ISPPRV_RGB_MAT4_MTX_GB_SHIFT), ISPPRV_RGB_MAT4);

		omap_writel((g_tc.hpipe_param.rgb2rgb.matrix[2][2] << \
			ISPPRV_RGB_MAT5_MTX_BB_SHIFT), ISPPRV_RGB_MAT5);

		omap_writel((g_tc.hpipe_param.rgb2rgb.offset[0] << \
			ISPPRV_RGB_OFF1_MTX_OFFG_SHIFT) | \
			(g_tc.hpipe_param.rgb2rgb.offset[1] << \
		ISPPRV_RGB_OFF1_MTX_OFFR_SHIFT), ISPPRV_RGB_OFF1);

		omap_writel(g_tc.hpipe_param.rgb2rgb.offset[2] << \
			ISPPRV_RGB_OFF2_MTX_OFFB_SHIFT, ISPPRV_RGB_OFF2);

		g_tc.update_hardpipe = 0;
	}
}
