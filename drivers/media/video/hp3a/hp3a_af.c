/*
 * drivers/media/video/hp3a/hp3a_af.c
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
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <plat/isp_user.h>

#include "hp3a.h"
#include "hp3a_common.h"
#include "hp3a_user.h"
#include "hp3a_af.h"
#include "isp.h"
#include "hp3a_ispreg.h"

#define  AF_MEMORY_ALIGNMENT      64

struct hp3a_reg isp_af_regs[] = {
	{HP3A_REG_32BIT, ISPH3A_PCR, 0}, /* 0 */
	{HP3A_REG_32BIT, ISPH3A_AFPAX1, 0},
	{HP3A_REG_32BIT, ISPH3A_AFPAX2, 0},
	{HP3A_REG_32BIT, ISPH3A_AFPAXSTART, 0},
	{HP3A_REG_32BIT, ISPH3A_AFIIRSH, 0},
	{HP3A_REG_32BIT, ISPH3A_AFBUFST, 0}, /* 5 */
	{HP3A_REG_32BIT, ISPH3A_AFCOEF010, 0},
	{HP3A_REG_32BIT, ISPH3A_AFCOEF032, 0},
	{HP3A_REG_32BIT, ISPH3A_AFCOEF054, 0},
	{HP3A_REG_32BIT, ISPH3A_AFCOEF076, 0},
	{HP3A_REG_32BIT, ISPH3A_AFCOEF098, 0}, /* 10 */
	{HP3A_REG_32BIT, ISPH3A_AFCOEF0010, 0},
	{HP3A_REG_32BIT, ISPH3A_AFCOEF110, 0},
	{HP3A_REG_32BIT, ISPH3A_AFCOEF132, 0},
	{HP3A_REG_32BIT, ISPH3A_AFCOEF154, 0},
	{HP3A_REG_32BIT, ISPH3A_AFCOEF176, 0}, /* 15 */
	{HP3A_REG_32BIT, ISPH3A_AFCOEF198, 0},
	{HP3A_REG_32BIT, ISPH3A_AFCOEF1010, 0},
	{HP3A_REG_32BIT, ISPH3A_AEWWIN1, 0},
	{HP3A_REG_32BIT, ISPH3A_AEWINSTART, 0},
	{HP3A_REG_32BIT, ISPH3A_AEWINBLK, 0},
	{HP3A_REG_32BIT, ISPH3A_AEWSUBWIN, 0}, /* 20 */
	{HP3A_REG_32BIT, ISPH3A_AEWBUFST, 0},
	{HP3A_REG_TOK_TERM, 0, 0}
	};

/**
 * hp3a_enable_af - Enables af harware.
 *
 * No return value.
 **/
void hp3a_enable_af(void)
{
	struct hp3a_internal_buffer *ibuffer = NULL;

	if (unlikely(g_tc.af_hw_configured == 0))
		return;

	if (omap_readl(ISPH3A_PCR) & ISPH3A_PCR_AF_BUSY_MASK) {
		g_tc.af_hw_enable = 0;
		return;
	}

	if (likely(hp3a_dequeue(&g_tc.af_stat_queue, &ibuffer) == 0)) {
		if (g_tc.af_hw_enable == 0) {
			/* Write H3A hardware registers. */
			hp3a_write_ispregs(isp_af_regs);
			omap_writel(IRQ0STATUS_H3A_AF_DONE_IRQ,
				ISP_IRQ0STATUS);

			/* Enable H3A AF hardware.  */
			omap_writel(omap_readl(ISPH3A_PCR) | \
				ISPH3A_PCR_AF_EN_MASK, ISPH3A_PCR);
			g_tc.af_hw_enable = 1;
		}

		/* Set memory address to store paxels. */
		omap_writel(ALIGN_TO(ibuffer->isp_addr,
			AF_MEMORY_ALIGNMENT), ISPH3A_AFBUFST);
		omap_readl(ISPH3A_AFBUFST);

		ibuffer->type = PAXEL;
		hp3a_enqueue(&g_tc.ready_stats_queue, &ibuffer);
	} else if (g_tc.af_hw_enable == 1) {
		g_tc.af_hw_enable = 0;
		omap_writel(omap_readl(ISPH3A_PCR) & \
			~(ISPH3A_PCR_AF_EN_MASK), ISPH3A_PCR);
	}
}

/**
 * hp3a_disable_af - Disables af harware.
 *
 * No return value.
 **/
void hp3a_disable_af(void)
{
	g_tc.af_hw_enable = 0;
	omap_writel(omap_readl(ISPH3A_PCR) & \
		~(ISPH3A_PCR_AF_EN_MASK), ISPH3A_PCR);
}

/**
 * hp3a_af_isr - ISR for the af done interrupt.
 *
 * No return value.
 **/
static void hp3a_af_isr(unsigned long status, isp_vbq_callback_ptr arg1,
			void *arg2)
{
	if (unlikely((H3A_AF_DONE & status) != H3A_AF_DONE))
		return;

	/* clear IRQ status bit.*/
	/*
	omap_writel(IRQ0STATUS_H3A_AF_DONE_IRQ,
					ISP_IRQ0STATUS);
	*/
	if (g_tc.af_hw_enable == 0) {
		omap_writel(omap_readl(ISPH3A_PCR) & \
				~(ISPH3A_PCR_AF_EN_MASK), ISPH3A_PCR);
	}
}

/**
 * hp3a_config_af - Configures OMAP ISP Histogram module.
 * @config: Pointer to a structure with af configurations.
 * @fh: Pointer to a hp3a_fh stucture..
 *
 * Return 0 on success, -1 otherwise.
 **/
int hp3a_config_af(struct hp3a_af_config *config, struct hp3a_fh *fh)
{
	int ret = 0;
	unsigned long irqflags = 0;
	struct hp3a_dev *device = fh->device;
	int index;
	int coeff0_index = 6; /* index into isp_af_regs array. */
	int coeff1_index = 12; /* index into isp_af_regs array. */

	if (g_tc.hw_initialized == 0)
		return -EINVAL;

	if (config->enable) {
		/* Install AF callback. */
		ret = isp_set_callback(CBK_H3A_AF_DONE, hp3a_af_isr,
					(void *)NULL, (void *)NULL);
		if (ret)
			return ret;

		if (hp3a_af_busy()) {
			dev_info(device->dev, "Error: AF engine is busy!\n");
			return -EINVAL;
		}

		spin_lock_irqsave(&g_tc.af_lock, irqflags);

		/* Set return value to error, it is set to 0 on success. */
		ret = -EINVAL;

		/* clear af registers. */
		hp3a_clear_regs(isp_af_regs);

		/* Enable af engine. */
		WRITE_REG(isp_af_regs[0].val, AF_ENABLE);

		if (config->alaw_enable) {
			/* Enable alaw. */
			OR_REG(isp_af_regs[0].val, AF_ALAW_ENABLE);
		}

		if (config->hmf.enable) {
			/* Enable HMF */
			OR_REG(isp_af_regs[0].val, (config->hmf.threshold << \
				AF_MED_TH_SHIFT) | AF_HMF_ENABLE);
		}

		/* Validate RGB pixel position. */
		if (config->rgbpos < 0 || config->rgbpos > 5) {
			dev_info(device->dev,
				"Error: Invalid RGB pixel position\n");
			goto func_exit;
		}
		/* Setup RGB pixel position. */
		OR_REG(isp_af_regs[0].val, config->rgbpos << AF_RGBPOS_SHIFT);

		/* Setup accumulations mode. */
		if (config->mode == ACCUMULATOR_PEAK) {
			/* Setup accumulations mode. */
			OR_REG(isp_af_regs[0].val, AF_FVMODE_PEAK);
		}

		/* Validate paxel dimessions. */
		if (config->paxel.width < 16 || config->paxel.width > 256 ||
			config->paxel.height < 2 ||
			config->paxel.height > 256) {
			dev_info(device->dev,
				"Error: Invalid paxel dimention %d-%d\n",
				config->paxel.width, config->paxel.height);
			goto func_exit;
		}

		/* Validate IIR filter hz start position. */
		if (config->iir.hz_start_pos > 4095) {
			dev_info(device->dev,
				"Error : Invalid IIR hz start position.");
			goto func_exit;
		}
		/* IIR filter hz start setup. */
		WRITE_REG(isp_af_regs[4].val, config->iir.hz_start_pos);

		/* Setup paxel start position after */
		/* IIR filter hz start setup. */
		if (config->paxel.hz_start < (config->iir.hz_start_pos + 1) ||
			config->paxel.hz_start > 4095 ||
			config->paxel.vt_start > 4095) {
			dev_info(device->dev,
				"Error : Invalid paxel start. (hz=%d vt=%d)\n",
				config->paxel.hz_start, config->paxel.vt_start);
			goto func_exit;
		}
		/* Setp pxel start positions. */
		WRITE_REG(isp_af_regs[3].val,
			(config->paxel.hz_start << AF_HZ_START_SHIFT) | \
			config->paxel.vt_start);

		/* Paxel dimension setup. */
		WRITE_REG(isp_af_regs[1].val, (config->paxel.height>>1) - 1);
		OR_REG(isp_af_regs[1].val,
			((config->paxel.width>>1) - 1) << AF_PAXW_SHIFT);

		/* Validate paxel count. */
		if (config->paxel.vt_cnt > 128 || config->paxel.hz_cnt < 1 ||
			config->paxel.hz_cnt > 36 || config->paxel.vt_cnt < 1) {
			dev_info(device->dev,
				"Error: Invalid paxel count (hz)%d-(vt)%d\n",
				config->paxel.hz_cnt, config->paxel.vt_cnt);
			goto func_exit;
		}

		if (config->paxel.line_incr > 8 ||
			config->paxel.line_incr < 2) {
			dev_info(device->dev,
				"Error: Invalid paxel line increment %d\n",
				config->paxel.line_incr);
			goto func_exit;
		}

		/* Paxel count setup. */
		WRITE_REG(isp_af_regs[2].val, config->paxel.hz_cnt - 1);
		OR_REG(isp_af_regs[2].val,
			(config->paxel.vt_cnt - 1) << AF_VT_COUNT_SHIFT);
		OR_REG(isp_af_regs[2].val,
			((config->paxel.line_incr >> 1) - 1) <<
			AF_LINE_INCR_SHIFT);

		/* Validate IIR filter coefficients. */
		for (index = 0; index < AF_NUMBER_OF_COEF; ++index) {
			if ((config->iir.coeff_set0[index]) > AF_COEF_MAX) {
				dev_info(device->dev,
					"Err : Coefficient %d for set 0 "
					" is incorrect\n",
					index);
				goto func_exit;
			}
			if ((config->iir.coeff_set1[index]) > AF_COEF_MAX) {
				dev_info(device->dev,
					"Err : Coefficient %d for set 1 "
					"wrong\n",
					index);
				goto func_exit;
			}
		}

		for (index = 0; index < (AF_NUMBER_OF_COEF - 1); index += 2) {
			WRITE_REG(isp_af_regs[coeff0_index].val,
				config->iir.coeff_set0[(index)]);
			OR_REG(isp_af_regs[coeff0_index].val,
				config->iir.coeff_set0[index+1] << \
				AF_COEF_SHIFT);

			WRITE_REG(isp_af_regs[coeff1_index].val,
				config->iir.coeff_set1[index]);
			OR_REG(isp_af_regs[coeff1_index].val,
				config->iir.coeff_set1[index+1] << \
				AF_COEF_SHIFT);

			++coeff0_index;
			++coeff1_index;
		}

		/* Writing last coefficient. */
		WRITE_REG(isp_af_regs[coeff0_index].val,
			config->iir.coeff_set0[AF_NUMBER_OF_COEF-1]);
		WRITE_REG(isp_af_regs[coeff1_index].val,
			config->iir.coeff_set1[AF_NUMBER_OF_COEF-1]);

		g_tc.req_af_buffer_size = (config->paxel.hz_cnt+1) * \
				(config->paxel.vt_cnt+1) * AF_PAXEL_SIZE;
		g_tc.af_hw_configured = 1;
		ret = 0;
	} else {
		spin_lock_irqsave(&g_tc.af_lock, irqflags);
		isp_unset_callback(CBK_H3A_AF_DONE);
		g_tc.af_hw_configured = 0;

		if (g_tc.af_hw_enable == 1) {
			/* Disabling AF hardware. */
			g_tc.af_hw_enable = 0;
			omap_writel(omap_readl(ISPH3A_PCR) & \
				~(ISPH3A_PCR_AF_EN_MASK),   ISPH3A_PCR);
		}
	}

func_exit:
	/* Give up synchronize lock. */
	spin_unlock_irqrestore(&g_tc.af_lock, irqflags);

	return ret;
}

/**
 * hp3a_af_busy - Verify if AF hardware is busy.
 *
 * Return 1 if busy, 0 otherwise.
 **/
int hp3a_af_busy(void)
{
	return omap_readl(ISPH3A_PCR)
		& ISPH3A_PCR_AF_BUSY_MASK;
}
EXPORT_SYMBOL(hp3a_af_busy);
