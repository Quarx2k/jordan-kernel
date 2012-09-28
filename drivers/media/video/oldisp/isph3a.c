/*
 * drivers/media/video/isp/isph3a.c
 *
 * H3A module for TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Contributors:
 *	Sergio Aguirre <saaguirre@ti.com>
 *	Troy Laramy <t-laramy@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/syscalls.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>

#include "isp.h"
#include "ispreg.h"
#include "isph3a.h"
#include "ispmmu.h"
#include "isppreview.h"

static DECLARE_MUTEX(isph3a_mutex);

/**
 * struct isph3a_aewb_buffer - AE, AWB frame stats buffer.
 * @virt_addr: Virtual address to mmap the buffer.
 * @phy_addr: Physical address of the buffer.
 * @addr_align: Virtual Address 32 bytes aligned.
 * @ispmmu_addr: Address of the buffer mapped by the ISPMMU.
 * @mmap_addr: Mapped memory area of buffer. For userspace access.
 * @locked: 1 - Buffer locked from write. 0 - Buffer can be overwritten.
 * @frame_num: Frame number from which the statistics are taken.
 * @next: Pointer to link next buffer.
 */
struct isph3a_aewb_buffer {
	unsigned long virt_addr;
	unsigned long phy_addr;
	unsigned long addr_align;
	unsigned long ispmmu_addr;
	unsigned long mmap_addr;	/* For userspace */
	struct timeval ts;

	u8 locked;
	u16 frame_num;
	struct isph3a_aewb_buffer *next;
};

/**
 * struct isph3a_aewb_status - AE, AWB status.
 * @initialized: 1 - Buffers initialized.
 * @update: 1 - Update registers.
 * @stats_req: 1 - Future stats requested.
 * @stats_done: 1 - Stats ready for user.
 * @frame_req: Number of frame requested for statistics.
 * @h3a_buff: Array of statistics buffers to access.
 * @stats_buf_size: Statistics buffer size.
 * @min_buf_size: Minimum statisitics buffer size.
 * @win_count: Window Count.
 * @frame_count: Frame Count.
 * @stats_wait: Wait primitive for locking/unlocking the stats request.
 * @buffer_lock: Spinlock for statistics buffers access.
 */
static struct isph3a_aewb_status {
	u8 initialized;
	u8 update;
	u8 stats_req;
	u8 stats_done;
	u16 frame_req;

	struct isph3a_aewb_buffer h3a_buff[H3A_MAX_BUFF];
	unsigned int stats_buf_size;
	unsigned int min_buf_size;
	unsigned int curr_cfg_buf_size;

	u16 win_count;
	u32 frame_count;
	wait_queue_head_t stats_wait;
	spinlock_t buffer_lock;		/* For stats buffers read/write sync */
} aewbstat;

/**
 * struct isph3a_aewb_regs - Current value of AE, AWB configuration registers.
 * reg_pcr: Peripheral control register.
 * reg_win1: Control register.
 * reg_start: Start position register.
 * reg_blk: Black line register.
 * reg_subwin: Configuration register.
 */
static struct isph3a_aewb_regs {
	u32 reg_pcr;
	u32 reg_win1;
	u32 reg_start;
	u32 reg_blk;
	u32 reg_subwin;
} aewb_regs;

static struct isph3a_aewb_config aewb_config_local = {
	.saturation_limit = 0x3FF,
	.win_height = 0,
	.win_width = 0,
	.ver_win_count = 0,
	.hor_win_count = 0,
	.ver_win_start = 0,
	.hor_win_start = 0,
	.blk_ver_win_start = 0,
	.blk_win_height = 0,
	.subsample_ver_inc = 0,
	.subsample_hor_inc = 0,
	.alaw_enable = 0,
	.aewb_enable = 0,
};

/* Structure for saving/restoring h3a module registers */
static struct isp_reg isph3a_reg_list[] = {
	{ISPH3A_AEWWIN1, 0},
	{ISPH3A_AEWINSTART, 0},
	{ISPH3A_AEWINBLK, 0},
	{ISPH3A_AEWSUBWIN, 0},
	{ISPH3A_AEWBUFST, 0},
	{ISPH3A_AFPAX1, 0},
	{ISPH3A_AFPAX2, 0},
	{ISPH3A_AFPAXSTART, 0},
	{ISPH3A_AFIIRSH, 0},
	{ISPH3A_AFBUFST, 0},
	{ISPH3A_AFCOEF010, 0},
	{ISPH3A_AFCOEF032, 0},
	{ISPH3A_AFCOEF054, 0},
	{ISPH3A_AFCOEF076, 0},
	{ISPH3A_AFCOEF098, 0},
	{ISPH3A_AFCOEF0010, 0},
	{ISPH3A_AFCOEF110, 0},
	{ISPH3A_AFCOEF132, 0},
	{ISPH3A_AFCOEF154, 0},
	{ISPH3A_AFCOEF176, 0},
	{ISPH3A_AFCOEF198, 0},
	{ISPH3A_AFCOEF1010, 0},
	{ISP_TOK_TERM, 0}
};

static struct ispprev_wbal h3awb_update;
static struct isph3a_aewb_buffer *active_buff;
static struct isph3a_aewb_xtrastats h3a_xtrastats[H3A_MAX_BUFF];
static int camnotify;
static int wb_update;
static void isph3a_print_status(void);

/**
 * isph3a_aewb_setxtrastats - Receives extra statistics from prior frames.
 * @xtrastats: Pointer to structure containing extra statistics fields like
 *             field count and timestamp of frame.
 *
 * Called from update_vbq in camera driver
 **/
void isph3a_aewb_setxtrastats(struct isph3a_aewb_xtrastats *xtrastats)
{
	int i;
	if (active_buff == NULL)
		return;

	for (i = 0; i < H3A_MAX_BUFF; i++) {
		if (aewbstat.h3a_buff[i].frame_num == active_buff->frame_num) {
			if (i == 0) {
				if (aewbstat.h3a_buff[H3A_MAX_BUFF - 1].
								locked == 0)
					h3a_xtrastats[H3A_MAX_BUFF - 1] =
								*xtrastats;
				else
					h3a_xtrastats[H3A_MAX_BUFF - 2] =
								*xtrastats;
			} else if (i == 1) {
				if (aewbstat.h3a_buff[0].locked == 0)
					h3a_xtrastats[0] = *xtrastats;
				else
					h3a_xtrastats[H3A_MAX_BUFF - 1] =
								*xtrastats;
			} else {
				if (aewbstat.h3a_buff[i - 1].locked == 0)
					h3a_xtrastats[i - 1] = *xtrastats;
				else
					h3a_xtrastats[i - 2] = *xtrastats;
			}
			return;
		}
	}
}
EXPORT_SYMBOL(isph3a_aewb_setxtrastats);

/**
 * isph3a_aewb_enable - Enables AE, AWB engine in the H3A module.
 * @enable: 1 - Enables the AE & AWB engine.
 *
 * Client should configure all the AE & AWB registers in H3A before this.
 **/
static void isph3a_aewb_enable(u8 enable)
{
	omap_writel(IRQ0STATUS_H3A_AWB_DONE_IRQ, ISP_IRQ0STATUS);

	if (enable) {
		aewb_regs.reg_pcr |= ISPH3A_PCR_AEW_EN;
		omap_writel(omap_readl(ISPH3A_PCR) | ISPH3A_PCR_AEW_EN,
								ISPH3A_PCR);
		DPRINTK_ISPH3A("    H3A enabled \n");
	} else {
		int timeout = 20;
		aewb_regs.reg_pcr &= ~ISPH3A_PCR_AEW_EN;
		omap_writel(omap_readl(ISPH3A_PCR) & ~ISPH3A_PCR_AEW_EN,
								ISPH3A_PCR);
		while ((omap_readl(ISPH3A_PCR) & ISPH3A_PCR_AEW_BUSY) &&
								timeout) {
			mdelay(10);
			timeout--;
		}
		if (timeout == 0) {
			printk(KERN_DEBUG "%s - can't disable AEWB H3A\n",
								__func__);
		}

		DPRINTK_ISPH3A("    H3A disabled \n");
	}
	aewb_config_local.aewb_enable = enable;
}

/**
 * isph3a_update_wb - Updates WB parameters.
 *
 * Needs to be called when no ISP Preview processing is taking place.
 **/
void isph3a_update_wb(void)
{
	if (wb_update) {
		isppreview_config_whitebalance(h3awb_update);
		wb_update = 0;
	}
	return;
}
EXPORT_SYMBOL(isph3a_update_wb);

/**
 * isph3a_aewb_update_regs - Helper function to update h3a registers.
 **/
static void isph3a_aewb_update_regs(void)
{
	omap_writel(aewb_regs.reg_pcr, ISPH3A_PCR);
	omap_writel(aewb_regs.reg_win1, ISPH3A_AEWWIN1);
	omap_writel(aewb_regs.reg_start, ISPH3A_AEWINSTART);
	omap_writel(aewb_regs.reg_blk, ISPH3A_AEWINBLK);
	omap_writel(aewb_regs.reg_subwin, ISPH3A_AEWSUBWIN);

	aewbstat.update = 0;
	aewbstat.frame_count = 0;
}

/**
 * isph3a_aewb_update_req_buffer - Helper function to update buffer cache pages
 * @buffer: Pointer to structure
 **/
static void isph3a_aewb_update_req_buffer(struct isph3a_aewb_buffer *buffer)
{
	int size = aewbstat.stats_buf_size;

	size = PAGE_ALIGN(size);
	dmac_inv_range((void *)buffer->addr_align,
		(void *)buffer->addr_align + size);
}

/**
 * isph3a_aewb_stats_available - Check for stats available of specified frame.
 * @aewbdata: Pointer to return AE AWB statistics data
 *
 * Returns 0 if successful, or -1 if statistics are unavailable.
 **/
static int isph3a_aewb_stats_available(struct isph3a_aewb_data *aewbdata)
{
	int i, ret;
	unsigned long irqflags;

	spin_lock_irqsave(&aewbstat.buffer_lock, irqflags);
	for (i = 0; i < H3A_MAX_BUFF; i++) {
		if ((aewbdata->frame_number == aewbstat.h3a_buff[i].frame_num)
					&& (aewbstat.h3a_buff[i].frame_num !=
					active_buff->frame_num)) {
			aewbstat.h3a_buff[i].locked = 1;
			spin_unlock_irqrestore(&aewbstat.buffer_lock, irqflags);
			isph3a_aewb_update_req_buffer(&aewbstat.h3a_buff[i]);
			aewbstat.h3a_buff[i].frame_num = 0;
			ret = copy_to_user(
				(void *)aewbdata->h3a_aewb_statistics_buf,
				(void *)aewbstat.h3a_buff[i].virt_addr,
				aewbstat.curr_cfg_buf_size);
			if (ret) {
				printk(KERN_ERR
					"Failed copy_to_user for "
					"H3A stats buff, %d\n", ret);
			}
			aewbdata->ts = aewbstat.h3a_buff[i].ts;
			aewbdata->field_count = h3a_xtrastats[i].field_count;
			return 0;
		}
	}
	spin_unlock_irqrestore(&aewbstat.buffer_lock, irqflags);

	return -1;
}

/**
 * isph3a_aewb_link_buffers - Helper function to link allocated buffers.
 **/
static void isph3a_aewb_link_buffers(void)
{
	int i;

	for (i = 0; i < H3A_MAX_BUFF; i++) {
		if ((i + 1) < H3A_MAX_BUFF) {
			aewbstat.h3a_buff[i].next = &aewbstat.h3a_buff[i + 1];
			h3a_xtrastats[i].next = &h3a_xtrastats[i + 1];
		} else {
			aewbstat.h3a_buff[i].next = &aewbstat.h3a_buff[0];
			h3a_xtrastats[i].next = &h3a_xtrastats[0];
		}
	}
}

/**
 * isph3a_aewb_unlock_buffers - Helper function to unlock all buffers.
 **/
static void isph3a_aewb_unlock_buffers(void)
{
	int i;
	unsigned long irqflags;

	spin_lock_irqsave(&aewbstat.buffer_lock, irqflags);
	for (i = 0; i < H3A_MAX_BUFF; i++)
		aewbstat.h3a_buff[i].locked = 0;

	spin_unlock_irqrestore(&aewbstat.buffer_lock, irqflags);
}

/**
 * isph3a_aewb_isr - Callback from ISP driver for H3A AEWB interrupt.
 * @status: IRQ0STATUS in case of MMU error, 0 for H3A interrupt.
 * @arg1: Not used as of now.
 * @arg2: Not used as of now.
 */
static void isph3a_aewb_isr(unsigned long status, isp_vbq_callback_ptr arg1,
								void *arg2)
{
	u16 frame_align;

	if ((H3A_AWB_DONE & status) != H3A_AWB_DONE)
		return;

	do_gettimeofday(&active_buff->ts);
	active_buff = active_buff->next;
	if (active_buff->locked == 1)
		active_buff = active_buff->next;
	omap_writel(active_buff->ispmmu_addr, ISPH3A_AEWBUFST);

	aewbstat.frame_count++;
	frame_align = aewbstat.frame_count;
	if (aewbstat.frame_count > MAX_FRAME_COUNT) {
		aewbstat.frame_count = 1;
		frame_align++;
	}
	active_buff->frame_num = aewbstat.frame_count;

	if (aewbstat.stats_req) {
		DPRINTK_ISPH3A("waiting for frame %d\n", aewbstat.frame_req);
		if (frame_align >= (aewbstat.frame_req + 1)) {
			aewbstat.stats_req = 0;
			aewbstat.stats_done = 1;
			wake_up_interruptible(&aewbstat.stats_wait);
		}
	}

	if (aewbstat.update)
		isph3a_aewb_update_regs();

}

/**
 * isph3a_aewb_set_params - Helper function to check & store user given params.
 * @user_cfg: Pointer to AE and AWB parameters struct.
 *
 * As most of them are busy-lock registers, need to wait until AEW_BUSY = 0 to
 * program them during ISR.
 *
 * Returns 0 if successful, or -EINVAL if any of the parameters are invalid.
 **/
static int isph3a_aewb_set_params(struct isph3a_aewb_config *user_cfg)
{
	if (unlikely(user_cfg->saturation_limit > MAX_SATURATION_LIM)) {
		printk(KERN_ERR "Invalid Saturation_limit: %d\n",
			user_cfg->saturation_limit);
		return -EINVAL;
	} else if (aewb_config_local.saturation_limit !=
						user_cfg->saturation_limit) {
		WRITE_SAT_LIM(aewb_regs.reg_pcr, user_cfg->saturation_limit);
		aewb_config_local.saturation_limit =
						user_cfg->saturation_limit;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	if (aewb_config_local.alaw_enable != user_cfg->alaw_enable) {
		WRITE_ALAW(aewb_regs.reg_pcr, user_cfg->alaw_enable);
		aewb_config_local.alaw_enable = user_cfg->alaw_enable;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	if (unlikely((user_cfg->win_height < MIN_WIN_H) ||
					(user_cfg->win_height > MAX_WIN_H) ||
					(user_cfg->win_height & 0x01))) {
		printk(KERN_ERR "Invalid window height: %d\n",
							user_cfg->win_height);
		return -EINVAL;
	} else if (aewb_config_local.win_height != user_cfg->win_height) {
		WRITE_WIN_H(aewb_regs.reg_win1, user_cfg->win_height);
		aewb_config_local.win_height = user_cfg->win_height;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	if (unlikely((user_cfg->win_width < MIN_WIN_W) ||
					(user_cfg->win_width > MAX_WIN_W) ||
					(user_cfg->win_width & 0x01))) {
		printk(KERN_ERR "Invalid window width: %d\n",
							user_cfg->win_width);
		return -EINVAL;
	} else if (aewb_config_local.win_width != user_cfg->win_width) {
		WRITE_WIN_W(aewb_regs.reg_win1, user_cfg->win_width);
		aewb_config_local.win_width = user_cfg->win_width;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	if (unlikely((user_cfg->ver_win_count < 1) ||
				(user_cfg->ver_win_count > MAX_WINVC))) {
		printk(KERN_ERR "Invalid vertical window count: %d\n",
						user_cfg->ver_win_count);
		return -EINVAL;
	} else if (aewb_config_local.ver_win_count
						!= user_cfg->ver_win_count) {
		WRITE_VER_C(aewb_regs.reg_win1, user_cfg->ver_win_count);
		aewb_config_local.ver_win_count = user_cfg->ver_win_count;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	if (unlikely((user_cfg->hor_win_count < 1) ||
				(user_cfg->hor_win_count > MAX_WINHC))) {
		printk(KERN_ERR "Invalid horizontal window count: %d\n",
						user_cfg->hor_win_count);
		return -EINVAL;
	} else if (aewb_config_local.hor_win_count
						!= user_cfg->hor_win_count) {
		WRITE_HOR_C(aewb_regs.reg_win1,
					user_cfg->hor_win_count);
		aewb_config_local.hor_win_count	=
					user_cfg->hor_win_count;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	if (unlikely(user_cfg->ver_win_start > MAX_WINSTART)) {
		printk(KERN_ERR "Invalid vertical window start: %d\n",
			user_cfg->ver_win_start);
		return -EINVAL;
	} else if (aewb_config_local.ver_win_start
						!= user_cfg->ver_win_start) {
		WRITE_VER_WIN_ST(aewb_regs.reg_start, user_cfg->ver_win_start);
		aewb_config_local.ver_win_start = user_cfg->ver_win_start;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	if (unlikely(user_cfg->hor_win_start > MAX_WINSTART)) {
		printk(KERN_ERR "Invalid horizontal window start: %d\n",
			user_cfg->hor_win_start);
		return -EINVAL;
	} else if (aewb_config_local.hor_win_start
				!= user_cfg->hor_win_start){
		WRITE_HOR_WIN_ST(aewb_regs.reg_start,
					 user_cfg->hor_win_start);
		aewb_config_local.hor_win_start	=
					user_cfg->hor_win_start;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	if (unlikely(user_cfg->blk_ver_win_start > MAX_WINSTART)) {
		printk(KERN_ERR "Invalid black vertical window start: %d\n",
			user_cfg->blk_ver_win_start);
		return -EINVAL;
	} else if (aewb_config_local.blk_ver_win_start
				!= user_cfg->blk_ver_win_start){
		WRITE_BLK_VER_WIN_ST(aewb_regs.reg_blk,
					user_cfg->blk_ver_win_start);
		aewb_config_local.blk_ver_win_start =
					user_cfg->blk_ver_win_start;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	if (unlikely((user_cfg->blk_win_height < MIN_WIN_H)
			|| (user_cfg->blk_win_height > MAX_WIN_H)
			|| (user_cfg->blk_win_height & 0x01))) {
		printk(KERN_ERR "Invalid black window height: %d\n",
			user_cfg->blk_win_height);
		return -EINVAL;
	} else if (aewb_config_local.blk_win_height
				!= user_cfg->blk_win_height) {
		WRITE_BLK_WIN_H(aewb_regs.reg_blk,
				user_cfg->blk_win_height);
		aewb_config_local.blk_win_height
				= user_cfg->blk_win_height;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	if (unlikely((user_cfg->subsample_ver_inc < MIN_SUB_INC)
			|| (user_cfg->subsample_ver_inc > MAX_SUB_INC)
			|| (user_cfg->subsample_ver_inc & 0x01))) {
		printk(KERN_ERR "Invalid vertical subsample increment: %d\n",
			user_cfg->subsample_ver_inc);
		return -EINVAL;
	} else if (aewb_config_local.subsample_ver_inc
				!= user_cfg->subsample_ver_inc) {
		WRITE_SUB_VER_INC(aewb_regs.reg_subwin,
						user_cfg->subsample_ver_inc);
		aewb_config_local.subsample_ver_inc
					= user_cfg->subsample_ver_inc;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	if (unlikely((user_cfg->subsample_hor_inc < MIN_SUB_INC)
			|| (user_cfg->subsample_hor_inc > MAX_SUB_INC)
			|| (user_cfg->subsample_hor_inc & 0x01))) {
		printk(KERN_ERR "Invalid horizontal subsample increment: %d\n",
			user_cfg->subsample_hor_inc);
		return -EINVAL;
	} else if (aewb_config_local.subsample_hor_inc
				!= user_cfg->subsample_hor_inc) {
		WRITE_SUB_HOR_INC(aewb_regs.reg_subwin,
						user_cfg->subsample_hor_inc);
		aewb_config_local.subsample_hor_inc
					= user_cfg->subsample_hor_inc;
		spin_lock(&aewbstat.buffer_lock);
		aewbstat.update = 1;
		spin_unlock(&aewbstat.buffer_lock);
	}

	spin_lock(&aewbstat.buffer_lock);
	if ((!aewbstat.initialized) || (0 == aewb_config_local.aewb_enable)) {
		isph3a_aewb_update_regs();
		aewbstat.initialized = 1;
	}
	spin_unlock(&aewbstat.buffer_lock);
	return 0;
}

/**
 * isph3a_aewb_configure - Configure AEWB regs, enable/disable H3A engine.
 * @aewbcfg: Pointer to AEWB config structure.
 *
 * Returns 0 if successful, -EINVAL if aewbcfg pointer is NULL, -ENOMEM if
 * was unable to allocate memory for the buffer, of other errors if H3A
 * callback is not set or the parameters for AEWB are invalid.
 **/
int isph3a_aewb_configure(struct isph3a_aewb_config *aewbcfg)
{
	int ret = 0;
	int i;
	int win_count = 0;

	if (NULL == aewbcfg) {
		printk(KERN_ERR "Null argument in configuration. \n");
		return -EINVAL;
	}

	if (!aewbstat.initialized) {
		DPRINTK_ISPH3A("Setting callback for H3A\n");
		ret = isp_set_callback(CBK_H3A_AWB_DONE, isph3a_aewb_isr,
					(void *)NULL, (void *)NULL);
		if (ret) {
			printk(KERN_ERR "No callback for H3A\n");
			return ret;
		}
	}

	ret = isph3a_aewb_set_params(aewbcfg);
	if (ret) {
		printk(KERN_ERR "Invalid parameters! \n");
		return ret;
	}

	win_count = (aewbcfg->ver_win_count * aewbcfg->hor_win_count);
	win_count += aewbcfg->hor_win_count;
	ret = (win_count / 8);
	win_count += (win_count % 8) ? 1 : 0;
	win_count += ret;

	down(&isph3a_mutex);
	aewbstat.win_count = win_count;
	aewbstat.curr_cfg_buf_size = win_count * AEWB_PACKET_SIZE;

	if (aewbstat.stats_buf_size && ((win_count * AEWB_PACKET_SIZE) >
						aewbstat.stats_buf_size)) {
		DPRINTK_ISPH3A("There was a previous buffer... "
			"Freeing/unmapping current stat busffs\n");
		isph3a_aewb_enable(0);
		for (i = 0; i < H3A_MAX_BUFF; i++) {
			ispmmu_unmap(aewbstat.h3a_buff[i].ispmmu_addr);
			dma_free_coherent(NULL,
				aewbstat.min_buf_size,
				(void *)aewbstat.h3a_buff[i].virt_addr,
				(dma_addr_t)aewbstat.h3a_buff[i].phy_addr);
			aewbstat.h3a_buff[i].virt_addr = 0;
		}
		aewbstat.stats_buf_size = 0;
	}

	if (!aewbstat.h3a_buff[0].virt_addr) {
		aewbstat.stats_buf_size = win_count * AEWB_PACKET_SIZE;
		aewbstat.min_buf_size = PAGE_ALIGN(aewbstat.stats_buf_size);

		DPRINTK_ISPH3A("Allocating/mapping new stat buffs\n");
		for (i = 0; i < H3A_MAX_BUFF; i++) {
			aewbstat.h3a_buff[i].virt_addr =
					(unsigned long)dma_alloc_coherent(NULL,
						aewbstat.min_buf_size,
						(dma_addr_t *)
						&aewbstat.h3a_buff[i].
						phy_addr, GFP_KERNEL |
						GFP_DMA);
			if (aewbstat.h3a_buff[i].virt_addr == 0) {
				printk(KERN_ERR "Can't acquire memory for "
					"buffer[%d]\n", i);
				up(&isph3a_mutex);
				return -ENOMEM;
			}
			aewbstat.h3a_buff[i].addr_align =
					aewbstat.h3a_buff[i].virt_addr;
			while ((aewbstat.h3a_buff[i].addr_align &
							0xFFFFFFC0) !=
							aewbstat.h3a_buff[i].
							addr_align)
				aewbstat.h3a_buff[i].addr_align++;
			aewbstat.h3a_buff[i].ispmmu_addr =
							ispmmu_map(aewbstat.
							h3a_buff[i].phy_addr,
							aewbstat.min_buf_size);
		}
		isph3a_aewb_unlock_buffers();
		isph3a_aewb_link_buffers();

		if (active_buff == NULL)
			active_buff = &aewbstat.h3a_buff[0];
		omap_writel(active_buff->ispmmu_addr, ISPH3A_AEWBUFST);
	}
	for (i = 0; i < H3A_MAX_BUFF; i++) {
		DPRINTK_ISPH3A("buff[%d] addr is:\n    virt    0x%lX\n"
					"    aligned 0x%lX\n"
					"    phys    0x%lX\n"
					"    ispmmu  0x%08lX\n"
					"    mmapped 0x%lX\n", i,
					aewbstat.h3a_buff[i].virt_addr,
					aewbstat.h3a_buff[i].addr_align,
					aewbstat.h3a_buff[i].phy_addr,
					aewbstat.h3a_buff[i].ispmmu_addr,
					aewbstat.h3a_buff[i].mmap_addr);
	}
	isph3a_aewb_enable(aewbcfg->aewb_enable);
	isph3a_print_status();
	up(&isph3a_mutex);
	return 0;
}
EXPORT_SYMBOL(isph3a_aewb_configure);

/**
 * isph3a_aewb_request_statistics - REquest statistics and update gains in AEWB
 * @aewbdata: Pointer to return AE AWB statistics data.
 *
 * This API allows the user to update White Balance gains, as well as
 * exposure time and analog gain. It is also used to request frame
 * statistics.
 *
 * Returns 0 if successful, -EINVAL when H3A engine is not enabled, or other
 * errors when setting gains.
 **/
int isph3a_aewb_request_statistics(struct isph3a_aewb_data *aewbdata)
{
	int ret = 0;
	u16 frame_diff = 0;
	u16 frame_cnt = aewbstat.frame_count;
	wait_queue_t wqt;

	if (!aewb_config_local.aewb_enable) {
		printk(KERN_ERR "H3A engine not enabled\n");
		return -EINVAL;
	}

	DPRINTK_ISPH3A("isph3a_aewb_request_statistics: Enter "
		"(frame req. => %d, current frame => %d, update => %d)\n",
		aewbdata->frame_number, frame_cnt, aewbdata->update);
	DPRINTK_ISPH3A("User data received: \n");
	DPRINTK_ISPH3A("Digital gain = 0x%04x\n", aewbdata->dgain);
	DPRINTK_ISPH3A("WB gain b *=   0x%04x\n", aewbdata->wb_gain_b);
	DPRINTK_ISPH3A("WB gain r *=   0x%04x\n", aewbdata->wb_gain_r);
	DPRINTK_ISPH3A("WB gain gb =   0x%04x\n", aewbdata->wb_gain_gb);
	DPRINTK_ISPH3A("WB gain gr =   0x%04x\n", aewbdata->wb_gain_gr);
	DPRINTK_ISPH3A("ISP AEWB request status wait for interrupt\n");

	if (aewbdata->update != 0) {
		if (aewbdata->update & SET_DIGITAL_GAIN)
			h3awb_update.dgain = (u16)aewbdata->dgain;
		if (aewbdata->update & SET_COLOR_GAINS) {
			h3awb_update.coef3 = (u8)aewbdata->wb_gain_b;
			h3awb_update.coef2 = (u8)aewbdata->wb_gain_gr;
			h3awb_update.coef1 = (u8)aewbdata->wb_gain_gb;
			h3awb_update.coef0 = (u8)aewbdata->wb_gain_r;
		}
		if (aewbdata->update & (SET_COLOR_GAINS | SET_DIGITAL_GAIN))
			wb_update = 1;

		if (aewbdata->update & REQUEST_STATISTICS) {
			isph3a_aewb_unlock_buffers();

			DPRINTK_ISPH3A("Stats available?\n");
			ret = isph3a_aewb_stats_available(aewbdata);
			if (!ret)
				goto out;

			DPRINTK_ISPH3A("Stats in near future?\n");
			if (aewbdata->frame_number > frame_cnt) {
				frame_diff = aewbdata->frame_number - frame_cnt;
			} else if (aewbdata->frame_number < frame_cnt) {
				if ((frame_cnt > (MAX_FRAME_COUNT -
							MAX_FUTURE_FRAMES))
							&& (aewbdata->
							frame_number
							< MAX_FRAME_COUNT))
					frame_diff = aewbdata->frame_number
							+ MAX_FRAME_COUNT
							- frame_cnt;
				else {
					frame_diff = MAX_FUTURE_FRAMES + 1;
				}
			}

			if (frame_diff > MAX_FUTURE_FRAMES) {
				printk(KERN_ERR "Invalid frame requested"
					", returning current frame stats\n");
				aewbdata->frame_number = frame_cnt;
			}
			if (!camnotify) {
				DPRINTK_ISPH3A("Waiting on stats IRQ "
					"for frame %d\n",
					aewbdata->frame_number);
				down(&isph3a_mutex);
				aewbstat.frame_req = aewbdata->frame_number;
				aewbstat.stats_req = 1;
				aewbstat.stats_done = 0;
				up(&isph3a_mutex);
				init_waitqueue_entry(&wqt, current);
				ret = wait_event_interruptible
						(aewbstat.stats_wait,
						aewbstat.stats_done == 1);
				if (ret < 0) {
					printk(KERN_ERR
						"isph3a_aewb_request_statistics"
						" Error on wait event %d\n",
						ret);
					aewbdata->h3a_aewb_statistics_buf =
									NULL;
					return ret;
				}

				DPRINTK_ISPH3A("ISP AEWB request status"
						" interrupt raised\n");
				ret = isph3a_aewb_stats_available(aewbdata);
				if (ret) {
					DPRINTK_ISPH3A
						("After waiting for stats,"
						" stats not available!!\n");
					aewbdata->h3a_aewb_statistics_buf =
									NULL;
				}
			} else {
				DPRINTK_ISPH3A("NOT Waiting on stats IRQ "
					"for frame %d "
					"because camnotify set\n",
					aewbdata->frame_number);
				aewbdata->h3a_aewb_statistics_buf = NULL;
			}
		} else {
			aewbdata->h3a_aewb_statistics_buf = NULL;
		}
	} else {
		aewbdata->h3a_aewb_statistics_buf = NULL;
	}
out:
	DPRINTK_ISPH3A("isph3a_aewb_request_statistics: "
		"aewbdata->h3a_aewb_statistics_buf => %p\n",
		aewbdata->h3a_aewb_statistics_buf);
	aewbdata->curr_frame = aewbstat.frame_count;

	return 0;
}
EXPORT_SYMBOL(isph3a_aewb_request_statistics);

/**
 * isph3a_aewb_init - Module Initialisation.
 *
 * Always returns 0.
 **/
int __init isph3a_aewb_init(void)
{
	memset(&aewbstat, 0, sizeof(aewbstat));
	memset(&aewb_regs, 0, sizeof(aewb_regs));

	init_waitqueue_head(&aewbstat.stats_wait);
	spin_lock_init(&aewbstat.buffer_lock);
	return 0;
}

/**
 * isph3a_aewb_cleanup - Module exit.
 **/
void __exit isph3a_aewb_cleanup(void)
{
	int i;
	isph3a_aewb_enable(0);
	isp_unset_callback(CBK_H3A_AWB_DONE);

	if (aewbstat.h3a_buff) {
		for (i = 0; i < H3A_MAX_BUFF; i++) {
			ispmmu_unmap(aewbstat.h3a_buff[i].ispmmu_addr);
			dma_free_coherent(NULL,
				aewbstat.min_buf_size,
				(void *)aewbstat.h3a_buff[i].virt_addr,
				(dma_addr_t)aewbstat.h3a_buff[i].phy_addr);
		}
	}
	memset(&aewbstat, 0, sizeof(aewbstat));
	memset(&aewb_regs, 0, sizeof(aewb_regs));
}

/**
 * isph3a_print_status - Debug print. Values of H3A related registers.
 **/
static void isph3a_print_status(void)
{
	DPRINTK_ISPH3A("ISPH3A_PCR = 0x%08x\n", omap_readl(ISPH3A_PCR));
	DPRINTK_ISPH3A("ISPH3A_AEWWIN1 = 0x%08x\n",
						omap_readl(ISPH3A_AEWWIN1));
	DPRINTK_ISPH3A("ISPH3A_AEWINSTART = 0x%08x\n",
						omap_readl(ISPH3A_AEWINSTART));
	DPRINTK_ISPH3A("ISPH3A_AEWINBLK = 0x%08x\n",
						omap_readl(ISPH3A_AEWINBLK));
	DPRINTK_ISPH3A("ISPH3A_AEWSUBWIN = 0x%08x\n",
						omap_readl(ISPH3A_AEWSUBWIN));
	DPRINTK_ISPH3A("ISPH3A_AEWBUFST = 0x%08x\n",
						omap_readl(ISPH3A_AEWBUFST));
	DPRINTK_ISPH3A("stats windows = %d\n", aewbstat.win_count);
	DPRINTK_ISPH3A("stats buff size = %d\n", aewbstat.stats_buf_size);
	DPRINTK_ISPH3A("currently configured stats buff size = %d\n",
						aewbstat.curr_cfg_buf_size);
}

/**
 * isph3a_notify - Unblocks user request for statistics when camera is off
 * @notify: 1 - Camera is turned off
 *
 * Used when the user has requested statistics about a future frame, but the
 * camera is turned off before it happens, and this function unblocks the
 * request so the user can continue in its program.
 **/
void isph3a_notify(int notify)
{
	camnotify = notify;
	if (camnotify && aewbstat.initialized) {
		printk(KERN_DEBUG "Warning Camera Off \n");
		aewbstat.stats_req = 0;
		aewbstat.stats_done = 1;
		wake_up_interruptible(&aewbstat.stats_wait);
	}
}
EXPORT_SYMBOL(isph3a_notify);

/**
 * isph3a_save_context - Saves the values of the h3a module registers.
 **/
void isph3a_save_context(void)
{
	DPRINTK_ISPH3A(" Saving context\n");
	isp_save_context(isph3a_reg_list);
}
EXPORT_SYMBOL(isph3a_save_context);

/**
 * isph3a_restore_context - Restores the values of the h3a module registers.
 **/
void isph3a_restore_context(void)
{
	DPRINTK_ISPH3A(" Restoring context\n");
	isp_restore_context(isph3a_reg_list);
}
EXPORT_SYMBOL(isph3a_restore_context);
