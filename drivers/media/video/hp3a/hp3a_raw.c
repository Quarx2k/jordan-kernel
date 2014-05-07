/*
 * drivers/media/video/hp3a/hp3a_raw.c
 *
 * HP Imaging/3A Driver : Raw stat specific function implementation.
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
#include "hp3a_user.h"
#include "hp3a_ispreg.h"

static struct hp3a_reg isp_raw_regs[] = {
	{HP3A_REG_32BIT, ISPCCDC_SYN_MODE, 0},
	{HP3A_REG_32BIT, ISPCCDC_HSIZE_OFF, 0},
	{HP3A_REG_32BIT, ISPCCDC_SDR_ADDR, 0},
	{HP3A_REG_32BIT, ISPCCDC_CFG, 0},
	{HP3A_REG_TOK_TERM, 0, 0}
	};

/**
 * hp3a_enable_raw - Enables raw statistics collection.
 *
 * No return value.
 **/
void hp3a_enable_raw(unsigned long buffer_addr)
{
	if (likely(buffer_addr != 0)) {
		if (g_tc.isp_ctx_saved == 0) {
			/* Save ISP registers. */
			hp3a_read_ispregs(isp_raw_regs);
			g_tc.isp_ctx_saved = 1;
		}

		/* Set ccdc config register. */
		omap_writel((omap_readl(ISPCCDC_CFG)) | ISPCCDC_CFG_WENLOG,
							ISPCCDC_CFG);

		/* Set oputput memory address. */
		omap_writel(buffer_addr, ISPCCDC_SDR_ADDR);

		/* Set register for line memory offset*/
		omap_writel(ALIGN_TO((g_tc.raw_width << 1), 32),
					ISPCCDC_HSIZE_OFF);

		/*
			The following registers must be set during CCDC config.
			CCDC_HORZ_INFO
			CCDC_VERT_START
			CCDC_VERT_LINES
			CCDC_SDOFST
		*/

		/* ISPCCDC_SYN_MODE must be set last. */
		omap_writel((omap_readl(ISPCCDC_SYN_MODE) |
				ISPCCDC_SYN_MODE_WEN |
				ISPCCDC_SYN_MODE_VP2SDR),
				ISPCCDC_SYN_MODE);
	}
}

/**
 * hp3a_disable_raw - Disables raw statistics collection.
 *
 * No return value.
 **/
void hp3a_disable_raw(void)
{
	if (g_tc.isp_ctx_saved == 1) {
		/* Restore ISP registers. */
		hp3a_write_ispregs(isp_raw_regs);
		g_tc.isp_ctx_saved = 0;
	}
}

/**
 * hp3a_configure_raw - Coinfigures hp3a framework for raw capture.
 *
 * No return value.
 **/
int hp3a_configure_raw(struct hp3a_raw_config *raw)
{
	unsigned long irqflags = 0;

	if (g_tc.hw_initialized == 0)
		return -EINVAL;

	/* Synchronize with stats collection. */
	spin_lock_irqsave(&g_tc.stats_lock, irqflags);

	if (raw->enable) {
		/* Configure raw frame size based on ccdc configuration. */
		g_tc.raw_width = (omap_readl(ISPCCDC_VP_OUT) >> \
					ISPCCDC_VP_OUT_HORZ_NUM_SHIFT)&0xFFF;
		g_tc.raw_height = (omap_readl(ISPCCDC_VP_OUT) >> \
					ISPCCDC_VP_OUT_VERT_NUM_SHIFT)&0xFFF;

		raw->width = g_tc.raw_width;
		raw->height = g_tc.raw_height;
		if (unlikely(raw->frequency < MIN_RAW_CAPTURE_INTERVAL)) {
			raw->frequency = MIN_RAW_CAPTURE_INTERVAL;
			g_tc.raw_frequency = raw->frequency;
		} else {
			g_tc.raw_frequency = raw->frequency;
		}

		/* Only configure raw capture if width and height is valid. */
		if (likely(g_tc.raw_width != 0 && g_tc.raw_height != 0)) {
			g_tc.raw_hw_configured = 1;
			g_tc.req_raw_buffer_size = (u32)ALIGN_TO(((ALIGN_TO( \
				g_tc.raw_width, 16)*g_tc.raw_height)<<1), \
				0x1000);
		} else {
			g_tc.raw_hw_configured = 0;
			g_tc.req_raw_buffer_size = -1;

			/* Give up synchronize lock. */
			spin_unlock_irqrestore(&g_tc.stats_lock, irqflags);

			return  -1;
		}
	} else {
		g_tc.raw_hw_configured = 0;
	}

	/* Give up synchronize lock. */
	spin_unlock_irqrestore(&g_tc.stats_lock, irqflags);

	return 0;
}
