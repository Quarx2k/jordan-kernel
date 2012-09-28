/*
 * isphist.c
 *
 * HISTOGRAM module for TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 *	Sergio Aguirre <saaguirre@ti.com>
 *	Troy Laramy
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <asm/cacheflush.h>

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>

#include "isp.h"
#include "ispreg.h"
#include "isphist.h"
#include "ispmmu.h"

/**
 * struct isp_hist_status - Histogram status.
 * @hist_enable: Enables the histogram module.
 * @initialized: Flag to indicate that the module is correctly initializated.
 * @frame_cnt: Actual frame count.
 * @frame_req: Frame requested by user.
 * @completed: Flag to indicate if a frame request is completed.
 */
struct isp_hist_status {
	u8 hist_enable;
	u8 pm_state;
	u8 initialized;
	u8 frame_cnt;
	u8 frame_req;
	u8 completed;
} histstat;

/**
 * struct isp_hist_buffer - Frame histogram buffer.
 * @virt_addr: Virtual address to mmap the buffer.
 * @phy_addr: Physical address of the buffer.
 * @addr_align: Virtual Address 32 bytes aligned.
 * @ispmmu_addr: Address of the buffer mapped by the ISPMMU.
 * @mmap_addr: Mapped memory area of buffer. For userspace access.
 */
struct isp_hist_buffer {
	unsigned long virt_addr;
	unsigned long phy_addr;
	unsigned long addr_align;
	unsigned long ispmmu_addr;
	unsigned long mmap_addr;
} hist_buff;

/**
 * struct isp_hist_regs - Current value of Histogram configuration registers.
 * @reg_pcr: Peripheral control register.
 * @reg_cnt: Histogram control register.
 * @reg_wb_gain: Histogram white balance gain register.
 * @reg_r0_h: Region 0 horizontal register.
 * @reg_r0_v: Region 0 vertical register.
 * @reg_r1_h: Region 1 horizontal register.
 * @reg_r1_v: Region 1 vertical register.
 * @reg_r2_h: Region 2 horizontal register.
 * @reg_r2_v: Region 2 vertical register.
 * @reg_r3_h: Region 3 horizontal register.
 * @reg_r3_v: Region 3 vertical register.
 * @reg_hist_addr: Histogram address register.
 * @reg_hist_data: Histogram data.
 * @reg_hist_radd: Address register. When input data comes from mem.
 * @reg_hist_radd_off: Address offset register. When input data comes from mem.
 * @reg_h_v_info: Image size register. When input data comes from mem.
 */
static struct isp_hist_regs {
	u32 reg_pcr;
	u32 reg_cnt;
	u32 reg_wb_gain;
	u32 reg_r0_h;
	u32 reg_r0_v;
	u32 reg_r1_h;
	u32 reg_r1_v;
	u32 reg_r2_h;
	u32 reg_r2_v;
	u32 reg_r3_h;
	u32 reg_r3_v;
	u32 reg_hist_addr;
	u32 reg_hist_data;
	u32 reg_hist_radd;
	u32 reg_hist_radd_off;
	u32 reg_h_v_info;
} hist_regs;

/* Structure for saving/restoring histogram module registers */
struct isp_reg isphist_reg_list[] = {
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_WB_GAIN, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R0_HORZ, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R0_VERT, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R1_HORZ, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R1_VERT, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R2_HORZ, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R2_VERT, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R3_HORZ, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_R3_VERT, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_ADDR, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_RADD, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_RADD_OFF, 0},
	{OMAP3_ISP_IOMEM_HIST, ISPHIST_H_V_INFO, 0},
	{0, ISP_TOK_TERM, 0}
};

static void isp_hist_print_status(void);

void __isp_hist_enable(u8 enable)
{
	if (enable)
		DPRINTK_ISPHIST("   histogram enabled \n");
	else
		DPRINTK_ISPHIST("   histogram disabled \n");

	isp_reg_and_or(OMAP3_ISP_IOMEM_HIST, ISPHIST_PCR, ~ISPHIST_PCR_EN,
						(enable ? ISPHIST_PCR_EN : 0));
	histstat.hist_enable = enable;
}

/**
 * isp_hist_enable - Enables ISP Histogram submodule operation.
 * @enable: 1 - Enables the histogram submodule.
 *
 * Client should configure all the Histogram registers before calling this
 * function.
 **/
void isp_hist_enable(u8 enable)
{
	__isp_hist_enable(enable);
	histstat.pm_state = enable;
}

/**
 * isp_hist_suspend - Suspend ISP Histogram submodule.
 **/
void isp_hist_suspend(void)
{
	if (histstat.pm_state)
		__isp_hist_enable(0);
}

/**
 * isp_hist_resume - Resume ISP Histogram submodule.
 **/
void isp_hist_resume(void)
{
	if (histstat.pm_state)
		__isp_hist_enable(1);
}

int isp_hist_busy(void)
{
	return isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_PCR) &
		ISPHIST_PCR_BUSY;
}


/**
 * isp_hist_update_regs - Helper function to update Histogram registers.
 **/
static void isp_hist_update_regs(void)
{
	isp_reg_writel(hist_regs.reg_pcr, OMAP3_ISP_IOMEM_HIST, ISPHIST_PCR);
	isp_reg_writel(hist_regs.reg_cnt, OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT);
	isp_reg_writel(hist_regs.reg_wb_gain, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_WB_GAIN);
	isp_reg_writel(hist_regs.reg_r0_h, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R0_HORZ);
	isp_reg_writel(hist_regs.reg_r0_v, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R0_VERT);
	isp_reg_writel(hist_regs.reg_r1_h, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R1_HORZ);
	isp_reg_writel(hist_regs.reg_r1_v, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R1_VERT);
	isp_reg_writel(hist_regs.reg_r2_h, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R2_HORZ);
	isp_reg_writel(hist_regs.reg_r2_v, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R2_VERT);
	isp_reg_writel(hist_regs.reg_r3_h, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R3_HORZ);
	isp_reg_writel(hist_regs.reg_r3_v, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_R3_VERT);
	isp_reg_writel(hist_regs.reg_hist_addr, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_ADDR);
	isp_reg_writel(hist_regs.reg_hist_data, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_DATA);
	isp_reg_writel(hist_regs.reg_hist_radd, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_RADD);
	isp_reg_writel(hist_regs.reg_hist_radd_off, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_RADD_OFF);
	isp_reg_writel(hist_regs.reg_h_v_info, OMAP3_ISP_IOMEM_HIST,
		       ISPHIST_H_V_INFO);
}

/**
 * isp_hist_isr - Callback from ISP driver for HIST interrupt.
 * @status: IRQ0STATUS in case of MMU error, 0 for hist interrupt.
 *          arg1 and arg2 Not used as of now.
 **/
static void isp_hist_isr(unsigned long status, isp_vbq_callback_ptr arg1,
			 void *arg2)
{
	isp_hist_enable(0);

	if (!(status & HIST_DONE))
		return;

	if (!histstat.completed) {
		if (histstat.frame_req == histstat.frame_cnt) {
			histstat.frame_cnt = 0;
			histstat.frame_req = 0;
			histstat.completed = 1;
		} else {
			isp_hist_enable(1);
			histstat.frame_cnt++;
		}
	}
}

/**
 * isp_hist_reset_mem - clear Histogram memory before start stats engine.
 *
 * Returns 0 after histogram memory was cleared.
 **/
static int isp_hist_reset_mem(void)
{
	int i;

	isp_reg_or(OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT, ISPHIST_CNT_CLR_EN);

	for (i = 0; i < HIST_MEM_SIZE; i++)
		isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_DATA);

	isp_reg_and(OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT, ~ISPHIST_CNT_CLR_EN);

	return 0;
}

/**
 * isp_hist_set_params - Helper function to check and store user given params.
 * @user_cfg: Pointer to user configuration structure.
 *
 * Returns 0 on success configuration.
 **/
static int isp_hist_set_params(struct isp_hist_config *user_cfg)
{

	int reg_num = 0;
	int bit_shift = 0;


	if (isp_hist_busy())
		return -EINVAL;

	if (user_cfg->input_bit_width > MIN_BIT_WIDTH)
		WRITE_DATA_SIZE(hist_regs.reg_cnt, 0);
	else
		WRITE_DATA_SIZE(hist_regs.reg_cnt, 1);

	WRITE_SOURCE(hist_regs.reg_cnt, user_cfg->hist_source);

	if (user_cfg->hist_source) {
		WRITE_HV_INFO(hist_regs.reg_h_v_info, user_cfg->hist_h_v_info);

		if ((user_cfg->hist_radd & ISP_32B_BOUNDARY_BUF) ==
		    user_cfg->hist_radd) {
			WRITE_RADD(hist_regs.reg_hist_radd,
				   user_cfg->hist_radd);
		} else {
			printk(KERN_ERR "Address should be in 32 byte boundary"
			       "\n");
			return -EINVAL;
		}

		if ((user_cfg->hist_radd_off & ISP_32B_BOUNDARY_OFFSET) ==
		    user_cfg->hist_radd_off) {
			WRITE_RADD_OFF(hist_regs.reg_hist_radd_off,
				       user_cfg->hist_radd_off);
		} else {
			printk(KERN_ERR "Offset should be in 32 byte boundary"
			       "\n");
			return -EINVAL;
		}

	}

	isp_hist_reset_mem();
	DPRINTK_ISPHIST("ISPHIST: Memory Cleared\n");
	histstat.frame_req = user_cfg->hist_frames;

	if (unlikely(user_cfg->wb_gain_R > MAX_WB_GAIN ||
		     user_cfg->wb_gain_RG > MAX_WB_GAIN ||
		     user_cfg->wb_gain_B > MAX_WB_GAIN ||
		     user_cfg->wb_gain_BG > MAX_WB_GAIN)) {
		printk(KERN_ERR "Invalid WB gain\n");
		return -EINVAL;
	} else {
		WRITE_WB_R(hist_regs.reg_wb_gain, user_cfg->wb_gain_R);
		WRITE_WB_RG(hist_regs.reg_wb_gain, user_cfg->wb_gain_RG);
		WRITE_WB_B(hist_regs.reg_wb_gain, user_cfg->wb_gain_B);
		WRITE_WB_BG(hist_regs.reg_wb_gain, user_cfg->wb_gain_BG);
	}

	/* Regions size and position */

	if (user_cfg->num_regions > MAX_REGIONS)
		return -EINVAL;

	if (likely((user_cfg->reg0_hor & ISPHIST_REGHORIZ_HEND_MASK) -
		   ((user_cfg->reg0_hor & ISPHIST_REGHORIZ_HSTART_MASK) >>
		    ISPHIST_REGHORIZ_HSTART_SHIFT))) {
		WRITE_REG_HORIZ(hist_regs.reg_r0_h, user_cfg->reg0_hor);
		reg_num++;
	} else {
		printk(KERN_ERR "Invalid Region parameters\n");
		return -EINVAL;
	}

	if (likely((user_cfg->reg0_ver & ISPHIST_REGVERT_VEND_MASK) -
		   ((user_cfg->reg0_ver & ISPHIST_REGVERT_VSTART_MASK) >>
		    ISPHIST_REGVERT_VSTART_SHIFT))) {
		WRITE_REG_VERT(hist_regs.reg_r0_v, user_cfg->reg0_ver);
	} else {
		printk(KERN_ERR "Invalid Region parameters\n");
		return -EINVAL;
	}

	if (user_cfg->num_regions >= 1) {
		if (likely((user_cfg->reg1_hor & ISPHIST_REGHORIZ_HEND_MASK) -
			   ((user_cfg->reg1_hor &
			     ISPHIST_REGHORIZ_HSTART_MASK) >>
			    ISPHIST_REGHORIZ_HSTART_SHIFT))) {
			WRITE_REG_HORIZ(hist_regs.reg_r1_h, user_cfg->reg1_hor);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}

		if (likely((user_cfg->reg1_ver & ISPHIST_REGVERT_VEND_MASK) -
			   ((user_cfg->reg1_ver &
			     ISPHIST_REGVERT_VSTART_MASK) >>
			    ISPHIST_REGVERT_VSTART_SHIFT))) {
			WRITE_REG_VERT(hist_regs.reg_r1_v, user_cfg->reg1_ver);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}
	}

	if (user_cfg->num_regions >= 2) {
		if (likely((user_cfg->reg2_hor & ISPHIST_REGHORIZ_HEND_MASK) -
			   ((user_cfg->reg2_hor &
			     ISPHIST_REGHORIZ_HSTART_MASK) >>
			    ISPHIST_REGHORIZ_HSTART_SHIFT))) {
			WRITE_REG_HORIZ(hist_regs.reg_r2_h, user_cfg->reg2_hor);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}

		if (likely((user_cfg->reg2_ver & ISPHIST_REGVERT_VEND_MASK) -
			   ((user_cfg->reg2_ver &
			     ISPHIST_REGVERT_VSTART_MASK) >>
			    ISPHIST_REGVERT_VSTART_SHIFT))) {
			WRITE_REG_VERT(hist_regs.reg_r2_v, user_cfg->reg2_ver);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}
	}

	if (user_cfg->num_regions >= 3) {
		if (likely((user_cfg->reg3_hor & ISPHIST_REGHORIZ_HEND_MASK) -
			   ((user_cfg->reg3_hor &
			     ISPHIST_REGHORIZ_HSTART_MASK) >>
			    ISPHIST_REGHORIZ_HSTART_SHIFT))) {
			WRITE_REG_HORIZ(hist_regs.reg_r3_h, user_cfg->reg3_hor);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}

		if (likely((user_cfg->reg3_ver & ISPHIST_REGVERT_VEND_MASK) -
			   ((user_cfg->reg3_ver &
			     ISPHIST_REGVERT_VSTART_MASK) >>
			    ISPHIST_REGVERT_VSTART_SHIFT))) {
			WRITE_REG_VERT(hist_regs.reg_r3_v, user_cfg->reg3_ver);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}
	}
	reg_num = user_cfg->num_regions;
	if (unlikely(((user_cfg->hist_bins > BINS_256) &&
		      (user_cfg->hist_bins != BINS_32)) ||
		     ((user_cfg->hist_bins == BINS_256) &&
		      reg_num != 0) || ((user_cfg->hist_bins ==
					 BINS_128) && reg_num >= 2))) {
		printk(KERN_ERR "Invalid Bins Number: %d\n",
		       user_cfg->hist_bins);
		return -EINVAL;
	} else {
		WRITE_NUM_BINS(hist_regs.reg_cnt, user_cfg->hist_bins);
	}

	if (user_cfg->input_bit_width > MAX_BIT_WIDTH ||
	    user_cfg->input_bit_width < MIN_BIT_WIDTH) {
		printk(KERN_ERR "Invalid Bit Width: %d\n",
		       user_cfg->input_bit_width);
		return -EINVAL;
	} else {
		switch (user_cfg->hist_bins) {
		case BINS_256:
			bit_shift = user_cfg->input_bit_width - 8;
			break;
		case BINS_128:
			bit_shift = user_cfg->input_bit_width - 7;
			break;
		case BINS_64:
			bit_shift = user_cfg->input_bit_width - 6;
			break;
		case BINS_32:
			bit_shift = user_cfg->input_bit_width - 5;
			break;
		default:
			return -EINVAL;
		}
		WRITE_BIT_SHIFT(hist_regs.reg_cnt, bit_shift);
	}

	isp_hist_update_regs();
	histstat.initialized = 1;

	return 0;
}

/**
 * isp_hist_configure - API to configure HIST registers.
 * @histcfg: Pointer to user configuration structure.
 *
 * Returns 0 on success configuration.
 **/
int isp_hist_configure(struct isp_hist_config *histcfg)
{

	int ret = 0;

	if (NULL == histcfg) {
		printk(KERN_ERR "Null argument in configuration. \n");
		return -EINVAL;
	}

	if (!histstat.initialized) {
		DPRINTK_ISPHIST("Setting callback for HISTOGRAM\n");
		ret = isp_set_callback(CBK_HIST_DONE, isp_hist_isr,
				       (void *)NULL, (void *)NULL);
		if (ret) {
			printk(KERN_ERR "No callback for HIST\n");
			return ret;
		}
	}

	ret = isp_hist_set_params(histcfg);
	if (ret) {
		printk(KERN_ERR "Invalid parameters! \n");
		return ret;
	}

	histstat.frame_cnt = 0;
	histstat.completed = 0;
	isp_hist_enable(1);
	isp_hist_print_status();

	return 0;
}
EXPORT_SYMBOL(isp_hist_configure);

/**
 * isp_hist_request_statistics - Request statistics in Histogram.
 * @histdata: Pointer to data structure.
 *
 * This API allows the user to request for histogram statistics.
 *
 * Returns 0 on successful request.
 **/
int isp_hist_request_statistics(struct isp_hist_data *histdata)
{
	int i, ret;
	u32 curr;

	if (isp_hist_busy())
		return -EBUSY;

	if (!histstat.completed && histstat.initialized)
		return -EINVAL;

	isp_reg_or(OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT, ISPHIST_CNT_CLR_EN);

	for (i = 0; i < HIST_MEM_SIZE; i++) {
		curr = isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_DATA);
		ret = put_user(curr, histdata->hist_statistics_buf + i);
		if (ret) {
			printk(KERN_ERR "Failed copy_to_user for "
			       "HIST stats buff, %d\n", ret);
		}
	}

	isp_reg_and(OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT,
		    ~ISPHIST_CNT_CLR_EN);
	histstat.completed = 0;
	return 0;
}
EXPORT_SYMBOL(isp_hist_request_statistics);

/**
 * isp_hist_init - Module Initialization.
 *
 * Returns 0 if successful.
 **/
int __init isp_hist_init(void)
{
	memset(&histstat, 0, sizeof(histstat));
	memset(&hist_regs, 0, sizeof(hist_regs));

	return 0;
}

/**
 * isp_hist_cleanup - Module cleanup.
 **/
void isp_hist_cleanup(void)
{
	memset(&histstat, 0, sizeof(histstat));
	memset(&hist_regs, 0, sizeof(hist_regs));
}

/**
 * isphist_save_context - Saves the values of the histogram module registers.
 **/
void isphist_save_context(void)
{
	DPRINTK_ISPHIST(" Saving context\n");
	isp_save_context(isphist_reg_list);
}
EXPORT_SYMBOL(isphist_save_context);

/**
 * isphist_restore_context - Restores the values of the histogram module regs.
 **/
void isphist_restore_context(void)
{
	DPRINTK_ISPHIST(" Restoring context\n");
	isp_restore_context(isphist_reg_list);
}
EXPORT_SYMBOL(isphist_restore_context);

/**
 * isp_hist_print_status - Debug print
 **/
static void isp_hist_print_status(void)
{
	DPRINTK_ISPHIST("ISPHIST_PCR = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_PCR));
	DPRINTK_ISPHIST("ISPHIST_CNT = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_CNT));
	DPRINTK_ISPHIST("ISPHIST_WB_GAIN = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_WB_GAIN));
	DPRINTK_ISPHIST("ISPHIST_R0_HORZ = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_R0_HORZ));
	DPRINTK_ISPHIST("ISPHIST_R0_VERT = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_R0_VERT));
	DPRINTK_ISPHIST("ISPHIST_R1_HORZ = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_R1_HORZ));
	DPRINTK_ISPHIST("ISPHIST_R1_VERT = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_R1_VERT));
	DPRINTK_ISPHIST("ISPHIST_R2_HORZ = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_R2_HORZ));
	DPRINTK_ISPHIST("ISPHIST_R2_VERT = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_R2_VERT));
	DPRINTK_ISPHIST("ISPHIST_R3_HORZ = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_R3_HORZ));
	DPRINTK_ISPHIST("ISPHIST_R3_VERT = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_R3_VERT));
	DPRINTK_ISPHIST("ISPHIST_ADDR = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_ADDR));
	DPRINTK_ISPHIST("ISPHIST_RADD = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_RADD));
	DPRINTK_ISPHIST("ISPHIST_RADD_OFF = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_RADD_OFF));
	DPRINTK_ISPHIST("ISPHIST_H_V_INFO = 0x%08x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_HIST, ISPHIST_H_V_INFO));
}
