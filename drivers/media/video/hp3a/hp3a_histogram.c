/*
 * drivers/media/video/hp3a/hp3a_histogram.c
 *
 * HP Imaging/3A Driver : Histogram specific function implementation.
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

#include <linux/interrupt.h>
#include <linux/device.h>

#include "hp3a.h"
#include "hp3a_common.h"
#include "hp3a_user.h"
#include "isp.h"
#include "hp3a_histogram.h"
#include "hp3a_ispreg.h"

static struct hp3a_reg isp_hist_regs[] = {
	{HP3A_REG_32BIT, ISPHIST_PCR, 0},	/* 0 */
	{HP3A_REG_32BIT, ISPHIST_CNT, 0},	/* 1 - CNT */
	{HP3A_REG_32BIT, ISPHIST_WB_GAIN, 0},	/* 2 */
	{HP3A_REG_32BIT, ISPHIST_R0_HORZ, 0},	/* 3 */
	{HP3A_REG_32BIT, ISPHIST_R0_VERT, 0},	/* 4 */
	{HP3A_REG_32BIT, ISPHIST_R1_HORZ, 0},	/* 5 */
	{HP3A_REG_32BIT, ISPHIST_R1_VERT, 0},	/* 6 */
	{HP3A_REG_32BIT, ISPHIST_R2_HORZ, 0},	/* 7 */
	{HP3A_REG_32BIT, ISPHIST_R2_VERT, 0},	/* 8 */
	{HP3A_REG_32BIT, ISPHIST_R3_HORZ, 0},	/* 9 */
	{HP3A_REG_32BIT, ISPHIST_R3_VERT, 0},	/* 10 */
	{HP3A_REG_32BIT, ISPHIST_ADDR, 0},	/* 11 */
	{HP3A_REG_32BIT, ISPHIST_RADD, 0},	/* 12 - RADD */
	{HP3A_REG_32BIT, ISPHIST_RADD_OFF, 0},	/* 13 - RADD_OFF */
	{HP3A_REG_32BIT, ISPHIST_H_V_INFO, 0},	/* 14 */
	{HP3A_REG_TOK_TERM, 0, 0}
	};

/**
 * hp3a_enable_histogram - Enables histogram harware.
 *
 * No return value.
 **/
void hp3a_enable_histogram(void)
{
	struct hp3a_internal_buffer *ibuffer;

	if (unlikely(g_tc.hist_hw_configured == 0))
		return;

	if (hp3a_dequeue(&g_tc.hist_stat_queue, &ibuffer) == 0) {
		if (g_tc.hist_hw_enable == 0) {
			/* Write histogram hardware registers. */
			hp3a_write_ispregs(isp_hist_regs);
			omap_writel(IRQ0STATUS_HIST_DONE_IRQ,
				ISP_IRQ0STATUS);
			/* Enable histogram hardware. */
			omap_writel(omap_readl(ISPHIST_PCR) | \
				(ISPHIST_PCR_EN), ISPHIST_PCR);

			g_tc.hist_hw_enable = 1;
			g_tc.hist_done = 0;
		}

		ibuffer->type = HISTOGRAM;
		hp3a_enqueue(&g_tc.hist_hw_queue, &ibuffer);
		hp3a_enqueue(&g_tc.ready_stats_queue, &ibuffer);
	} else if (g_tc.hist_hw_enable == 1) {
		g_tc.hist_hw_enable = 0;
		omap_writel(omap_readl(ISPHIST_PCR) & \
			~(ISPHIST_PCR_EN), ISPHIST_PCR);
	}
}

/**
 * hp3a_disable_histogram - Disables histogram harware.
 *
 * No return value.
 **/
void hp3a_disable_histogram(void)
{
	g_tc.hist_hw_enable = 0;
	omap_writel(omap_readl(ISPHIST_PCR) & ~(ISPHIST_PCR_EN), ISPHIST_PCR);
}

/**
 * hp3a_histogram_isr - ISR for the histogram done interrupt.
 *
 * No return value.
 **/
static void hp3a_histogram_isr(unsigned long status, isp_vbq_callback_ptr arg1,
	void *arg2)
{
	u32 *hist_buffer;
	u32 i;
	struct hp3a_internal_buffer *ibuffer = NULL;

	if (unlikely((HIST_DONE & status) != HIST_DONE))
		return;

	omap_writel(omap_readl(ISPHIST_PCR) & ~(ISPHIST_PCR_EN), ISPHIST_PCR);

	if (unlikely(g_tc.v4l2_streaming == 0))
		return;

	if (hp3a_dequeue_irqsave(&g_tc.hist_hw_queue, &ibuffer) == 0) {
		/* If there is a buffer available then fill it. */
		hist_buffer = (u32 *)phys_to_virt( \
			page_to_phys(ibuffer->pages[0]));
		if (hist_buffer) {
			omap_writel((omap_readl(ISPHIST_CNT)) | \
				ISPHIST_CNT_CLR_EN, ISPHIST_CNT);
			for (i = g_tc.hist_bin_size; i--;) {
				*hist_buffer = omap_readl(ISPHIST_DATA);
				++hist_buffer;
			}
			omap_writel((omap_readl(ISPHIST_CNT)) \
				& ~ISPHIST_CNT_CLR_EN, ISPHIST_CNT);
		}
	} else {
		/* There are no buffers availavle so just */
		/* clear internal histogram memory. */
		omap_writel((omap_readl(ISPHIST_CNT)) | \
			ISPHIST_CNT_CLR_EN, ISPHIST_CNT);
		for (i = g_tc.hist_bin_size; i--;)
			omap_writel(0, ISPHIST_DATA);
		omap_writel((omap_readl(ISPHIST_CNT)) & ~ISPHIST_CNT_CLR_EN,
			ISPHIST_CNT);
	}

	/* Set memory HW memory address and enable. */
	omap_writel(0, ISPHIST_ADDR);

	if (g_tc.hist_hw_enable == 1) {
		/* Enable histogram. */
		omap_writel(omap_readl(ISPHIST_PCR) | (ISPHIST_PCR_EN),
			ISPHIST_PCR);
	}

	g_tc.hist_done = 1;

	/* Release the tasks waiting for stats. */
	wake_up(&g_tc.stats_done);
}

/**
 * hp3a_config_histogram - Configures OMAP ISP Histogram module.
 * @config: Pointer to a structure with histogram configurations.
 * @fh: Pointer to a hp3a_fh stucture..
 *
 * Return 0 on success, -1 otherwise.
 **/
int hp3a_config_histogram(struct hp3a_histogram_config *config,
				struct hp3a_fh *fh)
{
	int ret = 0;
	unsigned long irqflags = 0;
	struct hp3a_dev *device = fh->device;
	int bit_shift = 0;

	if (g_tc.hw_initialized == 0)
		return -EINVAL;

	if (config->enable) {
		/* Install HIST_IRQ callback. */
		ret = isp_set_callback(CBK_HIST_DONE, hp3a_histogram_isr,
					(void *)NULL, (void *)NULL);
		if (ret)
			return ret;

		if (hp3a_hist_busy()) {
			dev_info(device->dev, \
				"Error: Histogram engine is busy\n");
			return -EINVAL;
		}

		spin_lock_irqsave(&g_tc.hist_lock, irqflags);

		/* Set return value to error, it is set to 0 on success. */
		ret = -EINVAL;

		/* clear histogram registers. */
		hp3a_clear_regs(isp_hist_regs);

		WRITE_SOURCE(isp_hist_regs[1].val, config->hist_source);

		if (config->hist_source) {
			/* source is memory. */
			WRITE_HV_INFO(isp_hist_regs[14].val,
				config->hist_h_v_info);

			if ((config->hist_radd & ISP_32B_BOUNDARY_BUF) ==
				config->hist_radd) {
				WRITE_RADD(isp_hist_regs[12].val,
					config->hist_radd);
			} else {
				dev_info(device->dev,
					"Err: Addr needs 32 byte boundary\n");
				goto func_exit;
			}

			if ((config->hist_radd_off &
				ISP_32B_BOUNDARY_OFFSET) ==
				config->hist_radd_off) {
				WRITE_RADD_OFF(isp_hist_regs[13].val, \
					config->hist_radd_off);
			} else {
				dev_info(device->dev,
					"Err: Offset needs 32 byte boundary\n");
				goto func_exit;
			}
		}

		/* set data size bit if the pixel data is 8-bit wide and */
		/* 2 pixels are packed in to 16-bits. */
		if (config->hist_packed_pxl)
			WRITE_DATA_SIZE(isp_hist_regs[1].val, 1);

		/* White Balance Field-to-Pattern Assignments */
		if (unlikely((config->wb_gain_R > MAX_WB_GAIN)
		|| (config->wb_gain_RG > MAX_WB_GAIN)
		|| (config->wb_gain_B > MAX_WB_GAIN)
		|| (config->wb_gain_BG > MAX_WB_GAIN))) {
			dev_info(device->dev, "Error: Invalid WB gain\n");
			goto func_exit;
		} else {
			WRITE_WB_R(isp_hist_regs[2].val, config->wb_gain_R);
			WRITE_WB_RG(isp_hist_regs[2].val,  config->wb_gain_RG);
			WRITE_WB_B(isp_hist_regs[2].val, config->wb_gain_B);
			WRITE_WB_BG(isp_hist_regs[2].val, config->wb_gain_BG);
		}

		/* Regions size and position */
		if (config->num_regions > MAX_REGIONS)
			goto func_exit;

		/* Region 0. */
		WRITE_REG_HORIZ(isp_hist_regs[3].val, config->reg0_hor);
		WRITE_REG_VERT(isp_hist_regs[4].val, config->reg0_ver);

		if (config->num_regions >= 1) {
			WRITE_REG_HORIZ(isp_hist_regs[5].val, config->reg1_hor);
			WRITE_REG_VERT(isp_hist_regs[6].val, config->reg1_ver);
		}
		if (config->num_regions >= 2) {
			WRITE_REG_HORIZ(isp_hist_regs[7].val, config->reg2_hor);
			WRITE_REG_VERT(isp_hist_regs[8].val, config->reg2_ver);
		}
		if (config->num_regions >= 3) {
			WRITE_REG_HORIZ(isp_hist_regs[9].val, config->reg3_hor);
			WRITE_REG_VERT(isp_hist_regs[10].val, config->reg3_ver);
		}

		/* Number of Bins. */
		if (unlikely(((config->hist_bins > BINS_256) &&
			(config->hist_bins != BINS_32)) ||
			((config->hist_bins == BINS_256) &&
			config->num_regions != 0) ||
			((config->hist_bins == BINS_128) &&
			config->num_regions >= 2))) {
			dev_info(device->dev,
				"Error: Invalid Bins Number: %d\n",
				config->hist_bins);
			goto func_exit;
		} else {
			WRITE_NUM_BINS(isp_hist_regs[1].val, config->hist_bins);
		}

		if ((config->input_bit_width > MAX_BIT_WIDTH) ||
			(config->input_bit_width < MIN_BIT_WIDTH)) {
			dev_info(device->dev,
				"Error: Invalid Bit Width: %d\n",
				config->input_bit_width);
			goto func_exit;
		} else {
			if (config->hist_bins == BINS_256) {
				bit_shift = config->input_bit_width - 8;
				g_tc.hist_bin_size = HIST_MEM_SIZE;
			} else if (config->hist_bins == BINS_128) {
				bit_shift = config->input_bit_width - 7;
				g_tc.hist_bin_size = (HIST_MEM_SIZE>>
					(1-config->num_regions));
			} else if (config->hist_bins == BINS_64) {
				bit_shift = config->input_bit_width - 6;
				g_tc.hist_bin_size = (HIST_MEM_SIZE>>
					(2-config->num_regions));
			} else if (config->hist_bins == BINS_32) {
				bit_shift = config->input_bit_width - 5;
				g_tc.hist_bin_size = (HIST_MEM_SIZE>>
					(3-config->num_regions));
			} else {
				goto func_exit;
			}

			WRITE_BIT_SHIFT(isp_hist_regs[1].val, bit_shift);
		}

		g_tc.hist_hw_configured = 1;
		ret = 0;
	} else {
		spin_lock_irqsave(&g_tc.hist_lock, irqflags);
		isp_unset_callback(CBK_HIST_DONE);
		g_tc.hist_hw_configured = 0;

		if (g_tc.hist_hw_enable == 1) {
			/* Disabling Histogram hardware. */
			g_tc.hist_hw_enable = 0;
			omap_writel(omap_readl(ISPHIST_PCR) & ~(ISPHIST_PCR_EN),
				ISPHIST_PCR);
		}
	}

func_exit:
	/* Give up synchronize lock. */
	spin_unlock_irqrestore(&g_tc.hist_lock, irqflags);

	return ret;
}

/**
 * hp3a_hist_busy - Verify if histogram hardware is busy.
 *
 * Return 1 if busy, 0 otherwise.
 **/
int hp3a_hist_busy(void)
{
	return omap_readl(ISPHIST_PCR) &
		ISPHIST_PCR_BUSY_MASK;

}
EXPORT_SYMBOL(hp3a_hist_busy);
