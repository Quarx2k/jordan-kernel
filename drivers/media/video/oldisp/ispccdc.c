/*
 * drivers/media/video/isp/ispccdc.c
 *
 * Driver Library for CCDC module in TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 * Copyright (C) 2009 Motorola.
 *
 * Contributors:
 *	Senthilvadivu Guruswamy <svadivu@ti.com>
 *	Pallavi Kulkarni <p-kulkarni@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <asm/mach-types.h>
#include <linux/io.h>
#include <linux/scatterlist.h>
#include <linux/uaccess.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccdc.h"
#include "ispmmu.h"

#define LSC_TABLE_INIT_SIZE	50052

static u32 *fpc_table_add;
static unsigned long fpc_table_add_m;

/**
 * struct isp_ccdc - Structure for the CCDC module to store its own information
 * @ccdc_inuse: Flag to determine if CCDC has been reserved or not (0 or 1).
 * @ccdcout_w: CCDC output width.
 * @ccdcout_h: CCDC output height.
 * @ccdcin_w: CCDC input width.
 * @ccdcin_h: CCDC input height.
 * @ccdcin_woffset: CCDC input horizontal offset.
 * @ccdcin_hoffset: CCDC input vertical offset.
 * @crop_w: Crop width.
 * @crop_h: Crop weight.
 * @ccdc_inpfmt: CCDC input format.
 * @ccdc_outfmt: CCDC output format.
 * @vpout_en: Video port output enable.
 * @wen: Data write enable.
 * @exwen: External data write enable.
 * @refmt_en: Reformatter enable.
 * @ccdcslave: CCDC slave mode enable.
 * @syncif_ipmod: Image
 * @obclamp_en: Data input format.
 * @mutexlock: Mutex used to get access to the CCDC.
 */
static struct isp_ccdc {
	u8 ccdc_inuse;
	u32 ccdcout_w;
	u32 ccdcout_h;
	u32 ccdcin_w;
	u32 ccdcin_h;
	u32 ccdcin_woffset;
	u32 ccdcin_hoffset;
	u32 crop_w;
	u32 crop_h;
	u8 ccdc_inpfmt;
	u8 ccdc_outfmt;
	u8 vpout_en;
	u8 wen;
	u8 exwen;
	u8 refmt_en;
	u8 ccdcslave;
	u8 syncif_ipmod;
	u8 obclamp_en;
	u8 lsc_en;
	struct mutex ispccdc_mutex; /* For checking/modifying ccdc_inuse */
	spinlock_t ispccdc_lock; /* spinlock to protect for pre-emption*/
	u32 wenlog;
	u32 dcsub;
} ispccdc_obj;

static struct ispccdc_lsc_config lsc_config;
static u8 *lsc_gain_table;
static unsigned long lsc_ispmmu_addr;
static int lsc_initialized;
static int size_mismatch;
static u8 ccdc_use_lsc;
static u8 ispccdc_lsc_tbl[LSC_TABLE_INIT_SIZE];

/* Structure for saving/restoring CCDC module registers*/
static struct isp_reg ispccdc_reg_list[] = {
	{ISPCCDC_SYN_MODE, 0},
	{ISPCCDC_HD_VD_WID, 0},
	{ISPCCDC_PIX_LINES, 0},
	{ISPCCDC_HORZ_INFO, 0},
	{ISPCCDC_VERT_START, 0},
	{ISPCCDC_VERT_LINES, 0},
	{ISPCCDC_CULLING, 0},
	{ISPCCDC_HSIZE_OFF, 0},
	{ISPCCDC_SDOFST, 0},
	{ISPCCDC_SDR_ADDR, 0},
	{ISPCCDC_CLAMP, 0},
	{ISPCCDC_DCSUB, 0},
	{ISPCCDC_COLPTN, 0},
	{ISPCCDC_BLKCMP, 0},
	{ISPCCDC_FPC, 0},
	{ISPCCDC_FPC_ADDR, 0},
	{ISPCCDC_VDINT, 0},
	{ISPCCDC_ALAW, 0},
	{ISPCCDC_REC656IF, 0},
	{ISPCCDC_CFG, 0},
	{ISPCCDC_FMTCFG, 0},
	{ISPCCDC_FMT_HORZ, 0},
	{ISPCCDC_FMT_VERT, 0},
	{ISPCCDC_FMT_ADDR0, 0},
	{ISPCCDC_FMT_ADDR1, 0},
	{ISPCCDC_FMT_ADDR2, 0},
	{ISPCCDC_FMT_ADDR3, 0},
	{ISPCCDC_FMT_ADDR4, 0},
	{ISPCCDC_FMT_ADDR5, 0},
	{ISPCCDC_FMT_ADDR6, 0},
	{ISPCCDC_FMT_ADDR7, 0},
	{ISPCCDC_PRGEVEN0, 0},
	{ISPCCDC_PRGEVEN1, 0},
	{ISPCCDC_PRGODD0, 0},
	{ISPCCDC_PRGODD1, 0},
	{ISPCCDC_VP_OUT, 0},
	{ISPCCDC_LSC_CONFIG, 0},
	{ISPCCDC_LSC_INITIAL, 0},
	{ISPCCDC_LSC_TABLE_BASE, 0},
	{ISPCCDC_LSC_TABLE_OFFSET, 0},
	{ISP_TOK_TERM, 0}
};

/**
 * omap34xx_isp_ccdc_config - Sets CCDC configuration from userspace
 * @userspace_add: Structure containing CCDC configuration sent from userspace.
 *
 * Returns 0 if successful, -EINVAL if the pointer to the configuration
 * structure is null, or the copy_from_user function fails to copy user space
 * memory to kernel space memory.
 **/
int omap34xx_isp_ccdc_config(void *userspace_add)
{
	struct ispccdc_bclamp bclamp_t;
	struct ispccdc_blcomp blcomp_t;
	struct ispccdc_fpc fpc_t;
	struct ispccdc_culling cull_t;
	struct ispccdc_update_config *ccdc_struct;
	u32 old_size;

	if (userspace_add == NULL)
		return -EINVAL;

	ccdc_struct = (struct ispccdc_update_config *) userspace_add;

	if ((ISP_ABS_CCDC_ALAW & ccdc_struct->flag) == ISP_ABS_CCDC_ALAW) {
		if ((ISP_ABS_CCDC_ALAW & ccdc_struct->update) ==
							ISP_ABS_CCDC_ALAW)
			ispccdc_config_alaw(ccdc_struct->alawip);
		ispccdc_enable_alaw(1);
	} else if ((ISP_ABS_CCDC_ALAW & ccdc_struct->update) ==
							ISP_ABS_CCDC_ALAW)
		ispccdc_enable_alaw(0);

	if ((ISP_ABS_CCDC_LPF & ccdc_struct->flag) == ISP_ABS_CCDC_LPF)
		ispccdc_enable_lpf(1);
	else
		ispccdc_enable_lpf(0);

	if ((ISP_ABS_CCDC_BLCLAMP & ccdc_struct->flag) ==
		ISP_ABS_CCDC_BLCLAMP) {
		if ((ISP_ABS_CCDC_BLCLAMP & ccdc_struct->update) ==
			ISP_ABS_CCDC_BLCLAMP) {
			if (copy_from_user(&bclamp_t, (struct ispccdc_bclamp *)
						(ccdc_struct->bclamp),
						sizeof(struct ispccdc_bclamp)))
				goto copy_from_user_err;

			ispccdc_enable_black_clamp(1);
			ispccdc_config_black_clamp(bclamp_t);
		} else
			ispccdc_enable_black_clamp(1);
	} else {
		if ((ISP_ABS_CCDC_BLCLAMP & ccdc_struct->update) ==
					ISP_ABS_CCDC_BLCLAMP) {
			if (copy_from_user(&bclamp_t,
				(struct ispccdc_bclamp *)(ccdc_struct->bclamp),
				sizeof(struct ispccdc_bclamp)))
				goto copy_from_user_err;

			ispccdc_enable_black_clamp(0);
			ispccdc_config_black_clamp(bclamp_t);
		}
	}

	if ((ISP_ABS_CCDC_BCOMP & ccdc_struct->update) == ISP_ABS_CCDC_BCOMP) {
		if (copy_from_user(&blcomp_t, (struct ispccdc_blcomp *)
							(ccdc_struct->blcomp),
							sizeof(blcomp_t)))
			goto copy_from_user_err;

		ispccdc_config_black_comp(blcomp_t);
	}

	if ((ISP_ABS_CCDC_FPC & ccdc_struct->flag) == ISP_ABS_CCDC_FPC) {
		if ((ISP_ABS_CCDC_FPC & ccdc_struct->update) ==
							ISP_ABS_CCDC_FPC) {
			if (copy_from_user(&fpc_t, (struct ispccdc_fpc *)
							(ccdc_struct->fpc),
							sizeof(fpc_t)))
				goto copy_from_user_err;
			mutex_lock(&ispccdc_obj.ispccdc_mutex);
			fpc_table_add = kmalloc((64 + (fpc_t.fpnum * 4)),
							GFP_KERNEL | GFP_DMA);
			if (!fpc_table_add) {
				printk(KERN_ERR "Cannot allocate memory for"
								" FPC table");
				mutex_unlock(&ispccdc_obj.ispccdc_mutex);
				return -ENOMEM;
			}
			while (((int)fpc_table_add & 0xFFFFFFC0) !=
							(int)fpc_table_add)
				fpc_table_add++;

			fpc_table_add_m = ispmmu_map(virt_to_phys
							(fpc_table_add),
							(fpc_t.fpnum) * 4);
			mutex_unlock(&ispccdc_obj.ispccdc_mutex);
			if (copy_from_user(fpc_table_add, (u32 *)fpc_t.fpcaddr,
							fpc_t.fpnum * 4))
				goto copy_from_user_err;

			fpc_t.fpcaddr = fpc_table_add_m;
			ispccdc_config_fpc(fpc_t);
		}
		ispccdc_enable_fpc(1);
	} else if ((ISP_ABS_CCDC_FPC & ccdc_struct->update) ==
							ISP_ABS_CCDC_FPC)
			ispccdc_enable_fpc(0);

	if ((ISP_ABS_CCDC_CULL & ccdc_struct->update) == ISP_ABS_CCDC_CULL) {
		if (copy_from_user(&cull_t, (struct ispccdc_culling *)
							(ccdc_struct->cull),
							sizeof(cull_t)))
			goto copy_from_user_err;
		ispccdc_config_culling(cull_t);
	}

	if (is_isplsc_activated()) {
		if ((ISP_ABS_CCDC_CONFIG_LSC & ccdc_struct->flag) ==
						ISP_ABS_CCDC_CONFIG_LSC) {
			if ((ISP_ABS_CCDC_CONFIG_LSC & ccdc_struct->update) ==
						ISP_ABS_CCDC_CONFIG_LSC) {
				old_size = lsc_config.size;
				if (copy_from_user(&lsc_config,
						(struct ispccdc_lsc_config *)
						(ccdc_struct->lsc_cfg),
						sizeof(struct
						ispccdc_lsc_config)))
					goto copy_from_user_err;
				mutex_lock(&ispccdc_obj.ispccdc_mutex);
				if (lsc_config.size <= old_size)
					size_mismatch = 0;
				else {
					size_mismatch = 1;
					lsc_initialized = 0;
				}
				mutex_unlock(&ispccdc_obj.ispccdc_mutex);
				ispccdc_config_lsc(&lsc_config);
			}
			ccdc_use_lsc = 1;
			ispccdc_enable_lsc(1);
		} else if ((ISP_ABS_CCDC_CONFIG_LSC & ccdc_struct->update) ==
						ISP_ABS_CCDC_CONFIG_LSC) {
				ispccdc_enable_lsc(0);
				ccdc_use_lsc = 0;
		}
		mutex_lock(&ispccdc_obj.ispccdc_mutex);
		if ((ISP_ABS_TBL_LSC & ccdc_struct->update)
			== ISP_ABS_TBL_LSC) {
			if (size_mismatch) {
				ispmmu_unmap(lsc_ispmmu_addr);
				kfree(lsc_gain_table);
				lsc_gain_table = kmalloc(
					(lsc_config.size + 0x1000),
					GFP_KERNEL | GFP_DMA);
				if (!lsc_gain_table) {
					printk(KERN_ERR
						"Cannot allocate\
						memory for \
						gain tables \n");
					mutex_unlock(&ispccdc_obj.ispccdc_mutex);
					return -ENOMEM;
				}
				lsc_ispmmu_addr = ispmmu_map(
					virt_to_phys((u8 *)ALIGN_TO(lsc_gain_table, 0x1000)),
					lsc_config.size);
				omap_writel(lsc_ispmmu_addr,
					ISPCCDC_LSC_TABLE_BASE);
				lsc_initialized = 1;
				size_mismatch = 0;
			}
			if (copy_from_user((u8 *)ALIGN_TO(lsc_gain_table, 0x1000),
				(ccdc_struct->lsc), lsc_config.size)) {
				mutex_unlock(&ispccdc_obj.ispccdc_mutex);
				goto copy_from_user_err;
			}
		}
		mutex_unlock(&ispccdc_obj.ispccdc_mutex);
	}

	if ((ISP_ABS_CCDC_COLPTN & ccdc_struct->update) == ISP_ABS_CCDC_COLPTN)
		ispccdc_config_imgattr(ccdc_struct->colptn);

	return 0;

copy_from_user_err:
	printk(KERN_ERR "CCDC Config:Copy From User Error");
	return -EINVAL ;
}
EXPORT_SYMBOL(omap34xx_isp_ccdc_config);

/**
 * Set the value to be used for CCDC_CFG.WENLOG.
 *  w - Value of wenlog.
 */
void ispccdc_set_wenlog(u32 wenlog)
{
	ispccdc_obj.wenlog = wenlog;
}
EXPORT_SYMBOL(ispccdc_set_wenlog);

/**
 * Set the value to be used for ISPCCDC_DCSUB.
 *  dcsub - Value of black level.
 */
void ispccdc_set_dcsub(u32 dcsub)
{
	ispccdc_obj.dcsub = dcsub;
}
EXPORT_SYMBOL(ispccdc_set_dcsub);

/**
 * ispccdc_request - Reserves the CCDC module.
 *
 * Reserves the CCDC module and assures that is used only once at a time.
 *
 * Returns 0 if successful, or -EBUSY if CCDC module is busy.
 **/
int ispccdc_request(void)
{
	spin_lock(&ispccdc_obj.ispccdc_lock);
	if (ispccdc_obj.ccdc_inuse) {
		spin_unlock(&ispccdc_obj.ispccdc_lock);
		DPRINTK_ISPCCDC("ISP_ERR : CCDC Module Busy");
		return -EBUSY;
	}

	ispccdc_obj.ccdc_inuse = 1;
	omap_writel((omap_readl(ISP_CTRL)) | ISPCTRL_CCDC_RAM_EN |
							ISPCTRL_CCDC_CLK_EN |
							ISPCTRL_SBL_WR1_RAM_EN,
							ISP_CTRL);
	omap_writel((omap_readl(ISPCCDC_CFG)) | ISPCCDC_CFG_VDLC, ISPCCDC_CFG);
	spin_unlock(&ispccdc_obj.ispccdc_lock);
	return 0;
}
EXPORT_SYMBOL(ispccdc_request);

/**
 * ispccdc_free - Frees the CCDC module.
 *
 * Frees the CCDC module so it can be used by another process.
 *
 * Returns 0 if successful, or -EINVAL if module has been already freed.
 **/
int ispccdc_free(void)
{
	spin_lock(&ispccdc_obj.ispccdc_lock);
	if (!ispccdc_obj.ccdc_inuse) {
		spin_unlock(&ispccdc_obj.ispccdc_lock);
		DPRINTK_ISPCCDC("ISP_ERR: CCDC Module already freed\n");
		return -EINVAL;
	}

	ispccdc_obj.ccdc_inuse = 0;
	omap_writel((omap_readl(ISP_CTRL)) & ~(ISPCTRL_CCDC_CLK_EN |
						ISPCTRL_CCDC_RAM_EN |
						ISPCTRL_SBL_WR1_RAM_EN),
						ISP_CTRL);
	spin_unlock(&ispccdc_obj.ispccdc_lock);
	return 0;
}
EXPORT_SYMBOL(ispccdc_free);

/**
 * ispccdc_load_lsc - Load Lens Shading Compensation table.
 * @table_size: LSC gain table size.
 *
 * Returns 0 if successful, or -ENOMEM of its no memory available.
 **/
int ispccdc_load_lsc(u32 table_size)
{
	if (!is_isplsc_activated())
		return 0;

	if (table_size == 0)
		return -EINVAL;

	if (lsc_initialized)
		return 0;

	ispccdc_enable_lsc(0);
	lsc_gain_table = kmalloc(table_size, GFP_KERNEL | GFP_DMA);

	if (!lsc_gain_table) {
		printk(KERN_ERR "Cannot allocate memory for gain tables \n");
		return -ENOMEM;
	}

	memcpy(lsc_gain_table, ispccdc_lsc_tbl, table_size);
	lsc_ispmmu_addr = ispmmu_map(virt_to_phys(lsc_gain_table), table_size);
	spin_lock(&ispccdc_obj.ispccdc_lock);
	omap_writel(lsc_ispmmu_addr, ISPCCDC_LSC_TABLE_BASE);
	lsc_initialized = 1;
	spin_unlock(&ispccdc_obj.ispccdc_lock);
	return 0;
}
EXPORT_SYMBOL(ispccdc_load_lsc);

/**
 * ispccdc_config_lsc - Configures the lens shading compensation module
 * @lsc_cfg: LSC configuration structure
 **/
void ispccdc_config_lsc(struct ispccdc_lsc_config *lsc_cfg)
{
	int reg;

	if (!is_isplsc_activated())
		return;

	ispccdc_enable_lsc(0);
	spin_lock(&ispccdc_obj.ispccdc_lock);
	omap_writel(lsc_cfg->offset, ISPCCDC_LSC_TABLE_OFFSET);

	reg = 0;
	reg |= (lsc_cfg->gain_mode_n << ISPCCDC_LSC_GAIN_MODE_N_SHIFT);
	reg |= (lsc_cfg->gain_mode_m << ISPCCDC_LSC_GAIN_MODE_M_SHIFT);
	reg |= (lsc_cfg->gain_format << ISPCCDC_LSC_GAIN_FORMAT_SHIFT);
	omap_writel(reg, ISPCCDC_LSC_CONFIG);

	reg = 0;
	reg &= ~ISPCCDC_LSC_INITIAL_X_MASK;
	reg |= (lsc_cfg->initial_x << ISPCCDC_LSC_INITIAL_X_SHIFT);
	reg &= ~ISPCCDC_LSC_INITIAL_Y_MASK;
	reg |= (lsc_cfg->initial_y << ISPCCDC_LSC_INITIAL_Y_SHIFT);
	omap_writel(reg, ISPCCDC_LSC_INITIAL);
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_config_lsc);

/**
 * ispccdc_enable_lsc - Enables/Disables the Lens Shading Compensation module.
 * @enable: 0 Disables LSC, 1 Enables LSC.
 **/
void ispccdc_enable_lsc(u8 enable)
{
	if (!is_isplsc_activated())
		return;

	if (enable) {
		omap_writel(omap_readl(ISP_CTRL) | ISPCTRL_SBL_SHARED_RPORTB |
					ISPCTRL_SBL_RD_RAM_EN, ISP_CTRL);
		omap_writel(omap_readl(ISPCCDC_LSC_CONFIG) | 0x1,
							ISPCCDC_LSC_CONFIG);
		ispccdc_obj.lsc_en = 1;
	} else {
		omap_writel(omap_readl(ISPCCDC_LSC_CONFIG) & 0xFFFE,
							ISPCCDC_LSC_CONFIG);
		ispccdc_obj.lsc_en = 0;
	}
}
EXPORT_SYMBOL(ispccdc_enable_lsc);

/**
 * ispccdc_set_crop_offset - Store the component order as component offset.
 * @raw_fmt: Input data component order.
 *
 * Turns the component order into a horizontal & vertical offset and store
 * offsets to be used later.
 **/
void ispccdc_set_crop_offset(enum ispccdc_raw_fmt raw_fmt)
{
	switch (raw_fmt) {
	case ISPCCDC_INPUT_FMT_GR_BG:
		ispccdc_obj.ccdcin_woffset = 1;
		ispccdc_obj.ccdcin_hoffset = 0;
		break;
	case ISPCCDC_INPUT_FMT_BG_GR:
		ispccdc_obj.ccdcin_woffset = 1;
		ispccdc_obj.ccdcin_hoffset = 1;
		break;
	case ISPCCDC_INPUT_FMT_RG_GB:
		ispccdc_obj.ccdcin_woffset = 0;
		ispccdc_obj.ccdcin_hoffset = 0;
		break;
	case ISPCCDC_INPUT_FMT_GB_RG:
		ispccdc_obj.ccdcin_woffset = 0;
		ispccdc_obj.ccdcin_hoffset = 1;
		break;
	}
}
EXPORT_SYMBOL(ispccdc_set_crop_offset);

/**
 * ispccdc_config_crop - Configures crop parameters for the ISP CCDC.
 * @left: Left offset of the crop area.
 * @top: Top offset of the crop area.
 * @height: Height of the crop area.
 * @width: Width of the crop area.
 *
 * The following restrictions are applied for the crop settings. If incoming
 * values do not follow these restrictions then we map the settings to the
 * closest acceptable crop value.
 * 1) Left offset is always odd. This can be avoided if we enable byte swap
 *    option for incoming data into CCDC.
 * 2) Top offset is always even.
 * 3) Crop height is always even.
 * 4) Crop width is always a multiple of 16 pixels
 **/
void ispccdc_config_crop(u32 left, u32 top, u32 height, u32 width)
{
	ispccdc_obj.ccdcin_woffset = left + ((left + 1) % 2);
	ispccdc_obj.ccdcin_hoffset = top + (top % 2);

	ispccdc_obj.crop_w = width - (width % 16);
	ispccdc_obj.crop_h = height + (height % 2);

	DPRINTK_ISPCCDC("\n\tOffsets L %d T %d W %d H %d\n",
						ispccdc_obj.ccdcin_woffset,
						ispccdc_obj.ccdcin_hoffset,
						ispccdc_obj.crop_w,
						ispccdc_obj.crop_h);
}

/**
 * ispccdc_config_datapath - Specifies the input and output modules for CCDC.
 * @input: Indicates the module that inputs the image to the CCDC.
 * @output: Indicates the module to which the CCDC outputs the image.
 *
 * Configures the default configuration for the CCDC to work with.
 *
 * The valid values for the input are CCDC_RAW (0), CCDC_YUV_SYNC (1),
 * CCDC_YUV_BT (2), and CCDC_OTHERS (3).
 *
 * The valid values for the output are CCDC_YUV_RSZ (0), CCDC_YUV_MEM_RSZ (1),
 * CCDC_OTHERS_VP (2), CCDC_OTHERS_MEM (3), CCDC_OTHERS_VP_MEM (4).
 *
 * Returns 0 if successful, or -EINVAL if wrong I/O combination or wrong input
 * or output values.
 **/
int ispccdc_config_datapath(enum ccdc_input input, enum ccdc_output output)
{
	u32 syn_mode = 0;
	struct ispccdc_vp vpcfg;
	struct ispccdc_syncif syncif;
	struct ispccdc_bclamp blkcfg;

	u32 colptn = (ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC0_SHIFT) |
		(ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP0PLC1_SHIFT) |
		(ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC2_SHIFT) |
		(ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP0PLC3_SHIFT) |
		(ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP1PLC0_SHIFT) |
		(ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP1PLC1_SHIFT) |
		(ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP1PLC2_SHIFT) |
		(ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP1PLC3_SHIFT) |
		(ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC0_SHIFT) |
		(ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP2PLC1_SHIFT) |
		(ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC2_SHIFT) |
		(ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP2PLC3_SHIFT) |
		(ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP3PLC0_SHIFT) |
		(ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP3PLC1_SHIFT) |
		(ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP3PLC2_SHIFT) |
		(ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP3PLC3_SHIFT);

	/* CCDC does not convert the image format */
	if (((input == CCDC_RAW) || (input == CCDC_OTHERS)) &&
						(output == CCDC_YUV_RSZ)) {
		DPRINTK_ISPCCDC("ISP_ERR: Wrong CCDC I/O Combination\n");
		return -EINVAL;
	}

	syn_mode = omap_readl(ISPCCDC_SYN_MODE);

	switch (output) {
	case CCDC_YUV_RSZ:
		syn_mode |= ISPCCDC_SYN_MODE_SDR2RSZ;
		syn_mode &= ~ISPCCDC_SYN_MODE_WEN;
		break;

	case CCDC_YUV_MEM_RSZ:
		syn_mode |= ISPCCDC_SYN_MODE_SDR2RSZ;
		ispccdc_obj.wen = 1;
		syn_mode |= ISPCCDC_SYN_MODE_WEN;
		break;

	case CCDC_OTHERS_VP:
		syn_mode &= ~ISPCCDC_SYN_MODE_VP2SDR;
		syn_mode &= ~ISPCCDC_SYN_MODE_SDR2RSZ;
		syn_mode &= ~ISPCCDC_SYN_MODE_WEN;
		vpcfg.bitshift_sel = BIT9_0;
		vpcfg.freq_sel = PIXCLKBY2;
		ispccdc_config_vp(vpcfg);
		ispccdc_enable_vp(1);
		break;

	case CCDC_OTHERS_MEM:
		syn_mode &= ~ISPCCDC_SYN_MODE_VP2SDR;
		syn_mode &= ~ISPCCDC_SYN_MODE_SDR2RSZ;
		syn_mode |= ISPCCDC_SYN_MODE_WEN;
		syn_mode &= ~ISPCCDC_SYN_MODE_EXWEN;
		spin_lock(&ispccdc_obj.ispccdc_lock);
		omap_writel((omap_readl(ISPCCDC_CFG)) & ~ISPCCDC_CFG_WENLOG,
								ISPCCDC_CFG);
		spin_unlock(&ispccdc_obj.ispccdc_lock);
		vpcfg.bitshift_sel = BIT11_2;
		vpcfg.freq_sel = PIXCLKBY2;
		ispccdc_config_vp(vpcfg);
		ispccdc_enable_vp(0);
		break;

	case CCDC_OTHERS_VP_MEM:
		syn_mode &= ~ISPCCDC_SYN_MODE_VP2SDR;
		syn_mode &= ~ISPCCDC_SYN_MODE_SDR2RSZ;
		syn_mode |= ISPCCDC_SYN_MODE_WEN;
		syn_mode |= ISPCCDC_SYN_MODE_EXWEN;
		spin_lock(&ispccdc_obj.ispccdc_lock);
		omap_writel((omap_readl(ISPCCDC_CFG) & ~ISPCCDC_CFG_WENLOG) |
					ispccdc_obj.wenlog, ISPCCDC_CFG);
		spin_unlock(&ispccdc_obj.ispccdc_lock);
		vpcfg.bitshift_sel = BIT9_0;
		vpcfg.freq_sel = PIXCLKBY2;
		ispccdc_config_vp(vpcfg);
		ispccdc_enable_vp(1);
		break;
	case CCDC_OTHERS_LSC_MEM: /* Added by MMS */
		syn_mode |= ISPCCDC_SYN_MODE_VP2SDR;
		syn_mode |= ISPCCDC_SYN_MODE_WEN;
		/* Generally cam_wen is used with cam_hs, vs signals */
		syn_mode |= ISPCCDC_SYN_MODE_EXWEN;
		omap_writel((omap_readl(ISPCCDC_CFG))
			| ISPCCDC_CFG_WENLOG, ISPCCDC_CFG);
		/* Video Port Configuration */
		vpcfg.bitshift_sel = BIT9_0;
		vpcfg.freq_sel = PIXCLKBY2;
		ispccdc_config_vp(vpcfg);
		ispccdc_enable_vp(1);
		break;
	default:
		DPRINTK_ISPCCDC("ISP_ERR: Wrong CCDC Output");
		return -EINVAL;
	};

	spin_lock(&ispccdc_obj.ispccdc_lock);
	omap_writel(syn_mode, ISPCCDC_SYN_MODE);
	spin_unlock(&ispccdc_obj.ispccdc_lock);

	switch (input) {
	case CCDC_RAW:
		syncif.ccdc_mastermode = 0;
		syncif.datapol = 0;
		syncif.datsz = DAT10;
		syncif.fldmode = 0;
		syncif.fldout = 0;
		syncif.fldpol = 0;
		syncif.fldstat = 0;
		syncif.hdpol = 0;
		syncif.ipmod = RAW;
		syncif.vdpol = 0;
		ispccdc_config_sync_if(syncif);
		ispccdc_config_imgattr(colptn);
		blkcfg.dcsubval = ispccdc_obj.dcsub;
		ispccdc_config_black_clamp(blkcfg);
		if (is_isplsc_activated()) {
			ispccdc_config_lsc(&lsc_config);
			ispccdc_load_lsc((u32)sizeof(ispccdc_lsc_tbl));
		}

		break;
	case CCDC_YUV_SYNC:
		syncif.ccdc_mastermode = 0;
		syncif.datapol = 0;
		syncif.datsz = DAT8;
		syncif.fldmode = 0;
		syncif.fldout = 0;
		syncif.fldpol = 0;
		syncif.fldstat = 0;
		syncif.hdpol = 0;
		syncif.ipmod = YUV16;
		syncif.vdpol = 0;
		ispccdc_config_imgattr(0);
		ispccdc_config_sync_if(syncif);
		blkcfg.dcsubval = 0;
		ispccdc_config_black_clamp(blkcfg);
		break;
	/* Added by MMS */
	case CCDC_RAW_PATTERN:
		/* Slave mode */
		syncif.ccdc_mastermode = 0;
		/* Normal */
		syncif.datapol = 0;
		syncif.datsz = DAT8;
		/* Progressive Mode */
		syncif.fldmode = 0;
		/* Input */
		syncif.fldout = 0;
		/* Positive */
		syncif.fldpol = 0;
		/* Odd Field */
		syncif.fldstat = 0;
		/* Positive */
		syncif.hdpol = 0;
		syncif.ipmod = RAW;
		/* Positive */
		syncif.vdpol = 0;
		ispccdc_config_sync_if(syncif);
		ispccdc_config_imgattr(colptn);
		/* Config DC sub */
		blkcfg.dcsubval = ispccdc_obj.dcsub;
		ispccdc_config_black_clamp(blkcfg);
		break;
	case CCDC_RAW_10_BIT_PATTERN:
		/* Slave mode */
		syncif.ccdc_mastermode = 0;
		/* Normal */
		syncif.datapol = 0;
		syncif.datsz = DAT10;
		/* Progressive Mode */
		syncif.fldmode = 0;
		/* Input */
		syncif.fldout = 0;
		/* Positive */
		syncif.fldpol = 0;
		/* Odd Field */
		syncif.fldstat = 0;
		/* Positive */
		syncif.hdpol = 0;
		syncif.ipmod = RAW;
		/* Positive */
		syncif.vdpol = 0;
		ispccdc_config_sync_if(syncif);
		ispccdc_config_imgattr(colptn);
		/* Zero out DC sub */
		blkcfg.dcsubval = 0;
		ispccdc_config_black_clamp(blkcfg);
		break;
	case CCDC_YUV_BT:
		break;
	case CCDC_OTHERS:
		break;
	default:
		DPRINTK_ISPCCDC("ISP_ERR: Wrong CCDC Input");
		return -EINVAL;
	}

	ispccdc_obj.ccdc_inpfmt = input;
	ispccdc_obj.ccdc_outfmt = output;
		ispccdc_print_status();
		isp_print_status();
	return 0;
}
EXPORT_SYMBOL(ispccdc_config_datapath);

/**
 * ispccdc_config_sync_if - Sets the sync i/f params between sensor and CCDC.
 * @syncif: Structure containing the sync parameters like field state, CCDC in
 *          master/slave mode, raw/yuv data, polarity of data, field, hs, vs
 *          signals.
 **/
void ispccdc_config_sync_if(struct ispccdc_syncif syncif)
{
	u32 syn_mode = omap_readl(ISPCCDC_SYN_MODE);

	syn_mode |= ISPCCDC_SYN_MODE_VDHDEN;
	syn_mode &= ~ISPCCDC_SYN_MODE_PACK8; /* Added by MMS */

	if (syncif.fldstat)
		syn_mode |= ISPCCDC_SYN_MODE_FLDSTAT;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDSTAT;

	syn_mode &= ISPCCDC_SYN_MODE_INPMOD_MASK;
	ispccdc_obj.syncif_ipmod = syncif.ipmod;

	switch (syncif.ipmod) {
	case RAW:
		break;
	case YUV16:
		syn_mode |= ISPCCDC_SYN_MODE_INPMOD_YCBCR16;
		break;
	case YUV8:
		syn_mode |= ISPCCDC_SYN_MODE_INPMOD_YCBCR8;
		break;
	};

	syn_mode &= ISPCCDC_SYN_MODE_DATSIZ_MASK;
	switch (syncif.datsz) {
	case DAT8:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_8;
		syn_mode |= ISPCCDC_SYN_MODE_PACK8; /* Added by MMS */
		break;
	case DAT10:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_10;
		break;
	case DAT11:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_11;
		break;
	case DAT12:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_12;
		break;
	};

	if (syncif.fldmode)
		syn_mode |= ISPCCDC_SYN_MODE_FLDMODE;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDMODE;

	if (syncif.datapol)
		syn_mode |= ISPCCDC_SYN_MODE_DATAPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_DATAPOL;

	if (syncif.fldpol)
		syn_mode |= ISPCCDC_SYN_MODE_FLDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDPOL;

	if (syncif.hdpol)
		syn_mode |= ISPCCDC_SYN_MODE_HDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_HDPOL;

	if (syncif.vdpol)
		syn_mode |= ISPCCDC_SYN_MODE_VDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_VDPOL;

	if (syncif.ccdc_mastermode) {
		syn_mode |= ISPCCDC_SYN_MODE_FLDOUT | ISPCCDC_SYN_MODE_VDHDOUT;
		omap_writel((syncif.hs_width << ISPCCDC_HD_VD_WID_HDW_SHIFT)
						| (syncif.vs_width <<
						ISPCCDC_HD_VD_WID_VDW_SHIFT),
						ISPCCDC_HD_VD_WID);

		omap_writel(syncif.ppln << ISPCCDC_PIX_LINES_PPLN_SHIFT
			| syncif.hlprf << ISPCCDC_PIX_LINES_HLPRF_SHIFT,
			ISPCCDC_PIX_LINES);
	} else
		syn_mode &= ~(ISPCCDC_SYN_MODE_FLDOUT |
						ISPCCDC_SYN_MODE_VDHDOUT);

	omap_writel(syn_mode, ISPCCDC_SYN_MODE);

	if (!(syncif.bt_r656_en)) {
		omap_writel((omap_readl(ISPCCDC_REC656IF)) &
						~ISPCCDC_REC656IF_R656ON,
						ISPCCDC_REC656IF);
	}
}
EXPORT_SYMBOL(ispccdc_config_sync_if);

/**
 * ispccdc_config_black_clamp - Configures the clamp parameters in CCDC.
 * @bclamp: Structure containing the optical black average gain, optical black
 *          sample length, sample lines, and the start pixel position of the
 *          samples w.r.t the HS pulse.
 * Configures the clamp parameters in CCDC. Either if its being used the
 * optical black clamp, or the digital clamp. If its a digital clamp, then
 * assures to put a valid DC substraction level.
 *
 * Returns always 0 when completed.
 **/
int ispccdc_config_black_clamp(struct ispccdc_bclamp bclamp)
{
	u32 bclamp_val = 0;

	spin_lock(&ispccdc_obj.ispccdc_lock);
	if (ispccdc_obj.obclamp_en) {
		bclamp_val |= bclamp.obgain << ISPCCDC_CLAMP_OBGAIN_SHIFT;
		bclamp_val |= bclamp.oblen << ISPCCDC_CLAMP_OBSLEN_SHIFT;
		bclamp_val |= bclamp.oblines << ISPCCDC_CLAMP_OBSLN_SHIFT;
		bclamp_val |= bclamp.obstpixel << ISPCCDC_CLAMP_OBST_SHIFT;
		omap_writel(bclamp_val, ISPCCDC_CLAMP);
	} else {
		if (system_rev < OMAP3430_REV_ES2_0)
			if ((ispccdc_obj.syncif_ipmod == YUV16) ||
					(ispccdc_obj.syncif_ipmod == YUV8) ||
					((omap_readl(ISPCCDC_REC656IF) &
					ISPCCDC_REC656IF_R656ON) ==
					ISPCCDC_REC656IF_R656ON))
				bclamp.dcsubval = 0;
		omap_writel(bclamp.dcsubval, ISPCCDC_DCSUB);
	}
	spin_unlock(&ispccdc_obj.ispccdc_lock);
	return 0;
}
EXPORT_SYMBOL(ispccdc_config_black_clamp);

/**
 * ispccdc_enable_black_clamp - Enables/Disables the optical black clamp.
 * @enable: 0 Disables optical black clamp, 1 Enables optical black clamp.
 *
 * Enables or disables the optical black clamp. When disabled, the digital
 * clamp operates.
 **/
void ispccdc_enable_black_clamp(u8 enable)
{
	spin_lock(&ispccdc_obj.ispccdc_lock);
	if (enable) {
		omap_writel((omap_readl(ISPCCDC_CLAMP))|ISPCCDC_CLAMP_CLAMPEN,
								ISPCCDC_CLAMP);
		ispccdc_obj.obclamp_en = 1;
	} else {
		omap_writel((omap_readl(ISPCCDC_CLAMP)) &
					~ISPCCDC_CLAMP_CLAMPEN, ISPCCDC_CLAMP);
		ispccdc_obj.obclamp_en = 0;
	}
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_enable_black_clamp);

/**
 * ispccdc_config_fpc - Configures the Faulty Pixel Correction parameters.
 * @fpc: Structure containing the number of faulty pixels corrected in the
 *       frame, address of the FPC table.
 *
 * Returns 0 if successful, or -EINVAL if FPC Address is not on the 64 byte
 * boundary.
 **/
int ispccdc_config_fpc(struct ispccdc_fpc fpc)
{
	u32 fpc_val = 0;

	spin_lock(&ispccdc_obj.ispccdc_lock);
	fpc_val = omap_readl(ISPCCDC_FPC);

	if ((fpc.fpcaddr & 0xFFFFFFC0) == fpc.fpcaddr) {
		omap_writel(fpc_val&(~ISPCCDC_FPC_FPCEN), ISPCCDC_FPC);
		omap_writel(fpc.fpcaddr, ISPCCDC_FPC_ADDR);
	} else {
		DPRINTK_ISPCCDC("FPC Address should be on 64byte boundary\n");
		spin_unlock(&ispccdc_obj.ispccdc_lock);
		return -EINVAL;
	}
	omap_writel(fpc_val | (fpc.fpnum << ISPCCDC_FPC_FPNUM_SHIFT),
								ISPCCDC_FPC);
	spin_unlock(&ispccdc_obj.ispccdc_lock);
	return 0;
}
EXPORT_SYMBOL(ispccdc_config_fpc);

/**
 * ispccdc_enable_fpc - Enables the Faulty Pixel Correction.
 * @enable: 0 Disables FPC, 1 Enables FPC.
 **/
void ispccdc_enable_fpc(u8 enable)
{
	spin_lock(&ispccdc_obj.ispccdc_lock);
	if (enable) {
		omap_writel(omap_readl(ISP_CTRL) | ISPCTRL_SBL_SHARED_RPORTB |
					ISPCTRL_SBL_RD_RAM_EN, ISP_CTRL);
		omap_writel(omap_readl(ISPCCDC_FPC) | ISPCCDC_FPC_FPCEN,
								ISPCCDC_FPC);
	} else {
		omap_writel(omap_readl(ISPCCDC_FPC) & ~ISPCCDC_FPC_FPCEN,
								ISPCCDC_FPC);
	}
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_enable_fpc);

/**
 * ispccdc_config_black_comp - Configures Black Level Compensation parameters.
 * @blcomp: Structure containing the black level compensation value for RGrGbB
 *          pixels. in 2's complement.
 **/
void ispccdc_config_black_comp(struct ispccdc_blcomp blcomp)
{
	u32 blcomp_val = 0;

	blcomp_val |= (((u32)blcomp.b_mg & 0xFF) << ISPCCDC_BLKCMP_B_MG_SHIFT);
	blcomp_val |= (((u32)blcomp.gb_g & 0xFF) << ISPCCDC_BLKCMP_GB_G_SHIFT);
	blcomp_val |= (((u32)blcomp.gr_cy & 0xFF) <<
						ISPCCDC_BLKCMP_GR_CY_SHIFT);
	blcomp_val |= (((u32)blcomp.r_ye & 0xFF) << ISPCCDC_BLKCMP_R_YE_SHIFT);

	spin_lock(&ispccdc_obj.ispccdc_lock);
	omap_writel(blcomp_val, ISPCCDC_BLKCMP);
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_config_black_comp);

/**
 * ispccdc_config_vp - Configures the Video Port Configuration parameters.
 * @vpcfg: Structure containing the Video Port input frequency, and the 10 bit
 *         format.
 **/
void ispccdc_config_vp(struct ispccdc_vp vpcfg)
{
	u32 fmtcfg_vp = omap_readl(ISPCCDC_FMTCFG);

	fmtcfg_vp &= ISPCCDC_FMTCFG_VPIN_MASK & ISPCCDC_FMTCF_VPIF_FRQ_MASK;

	switch (vpcfg.bitshift_sel) {
	case BIT9_0:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_9_0;
		break;
	case BIT10_1:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_10_1;
		break;
	case BIT11_2:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_11_2;
		break;
	case BIT12_3:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_12_3;
		break;
	};
	switch (vpcfg.freq_sel) {
	case PIXCLKBY2:
		fmtcfg_vp |= ISPCCDC_FMTCF_VPIF_FRQ_BY2;
		break;
	case PIXCLKBY3_5:
		fmtcfg_vp |= ISPCCDC_FMTCF_VPIF_FRQ_BY3;
		break;
	case PIXCLKBY4_5:
		fmtcfg_vp |= ISPCCDC_FMTCF_VPIF_FRQ_BY4;
		break;
	case PIXCLKBY5_5:
		fmtcfg_vp |= ISPCCDC_FMTCF_VPIF_FRQ_BY5;
		break;
	case PIXCLKBY6_5:
		fmtcfg_vp |= ISPCCDC_FMTCF_VPIF_FRQ_BY6;
		break;
	};
	spin_lock(&ispccdc_obj.ispccdc_lock);
	omap_writel(fmtcfg_vp, ISPCCDC_FMTCFG);
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_config_vp);

/**
 * ispccdc_enable_vp - Enables the Video Port.
 * @enable: 0 Disables VP, 1 Enables VP
 **/
void ispccdc_enable_vp(u8 enable)
{
	spin_lock(&ispccdc_obj.ispccdc_lock);
	if (enable) {
		omap_writel((omap_readl(ISPCCDC_FMTCFG)) |
					ISPCCDC_FMTCFG_VPEN, ISPCCDC_FMTCFG);
	} else {
		omap_writel(omap_readl(ISPCCDC_FMTCFG) &
					~ISPCCDC_FMTCFG_VPEN, ISPCCDC_FMTCFG);
	}
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_enable_vp);

/**
 * ispccdc_config_reformatter - Configures the Reformatter.
 * @refmt: Structure containing the memory address to format and the bit fields
 *         for the reformatter registers.
 *
 * Configures the Reformatter register values if line alternating is disabled.
 * Else, just enabling line alternating is enough.
 **/
void ispccdc_config_reformatter(struct ispccdc_refmt refmt)
{
	u32 fmtcfg_val = 0;

	fmtcfg_val = omap_readl(ISPCCDC_FMTCFG);

	if (refmt.lnalt)
		fmtcfg_val |= ISPCCDC_FMTCFG_LNALT;
	else {
		fmtcfg_val &= ~ISPCCDC_FMTCFG_LNALT;
		fmtcfg_val &= 0xFFFFF003;
		fmtcfg_val |= refmt.lnum << ISPCCDC_FMTCFG_LNUM_SHIFT;
		fmtcfg_val |= refmt.plen_even <<
						ISPCCDC_FMTCFG_PLEN_EVEN_SHIFT;
		fmtcfg_val |= refmt.plen_odd << ISPCCDC_FMTCFG_PLEN_ODD_SHIFT;

		omap_writel(refmt.prgeven0, ISPCCDC_PRGEVEN0);
		omap_writel(refmt.prgeven1, ISPCCDC_PRGEVEN1);
		omap_writel(refmt.prgodd0, ISPCCDC_PRGODD0);
		omap_writel(refmt.prgodd1, ISPCCDC_PRGODD1);
		omap_writel(refmt.fmtaddr0, ISPCCDC_FMT_ADDR0);
		omap_writel(refmt.fmtaddr1, ISPCCDC_FMT_ADDR1);
		omap_writel(refmt.fmtaddr2, ISPCCDC_FMT_ADDR2);
		omap_writel(refmt.fmtaddr3, ISPCCDC_FMT_ADDR3);
		omap_writel(refmt.fmtaddr4, ISPCCDC_FMT_ADDR4);
		omap_writel(refmt.fmtaddr5, ISPCCDC_FMT_ADDR5);
		omap_writel(refmt.fmtaddr6, ISPCCDC_FMT_ADDR6);
		omap_writel(refmt.fmtaddr7, ISPCCDC_FMT_ADDR7);
	}
	spin_lock(&ispccdc_obj.ispccdc_lock);
	omap_writel(fmtcfg_val, ISPCCDC_FMTCFG);
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_config_reformatter);

/**
 * ispccdc_enable_reformatter - Enables the Reformatter.
 * @enable: 0 Disables Reformatter, 1- Enables Data Reformatter
 **/
void ispccdc_enable_reformatter(u8 enable)
{
	spin_lock(&ispccdc_obj.ispccdc_lock);
	if (enable) {
		omap_writel((omap_readl(ISPCCDC_FMTCFG)) |
							ISPCCDC_FMTCFG_FMTEN,
							ISPCCDC_FMTCFG);
		ispccdc_obj.refmt_en = 1;
	} else {
		omap_writel((omap_readl(ISPCCDC_FMTCFG)) &
							~ISPCCDC_FMTCFG_FMTEN,
							ISPCCDC_FMTCFG);
		ispccdc_obj.refmt_en = 0;
	}
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_enable_reformatter);

/**
 * ispccdc_config_culling - Configures the culling parameters.
 * @cull: Structure containing the vertical culling pattern, and horizontal
 *        culling pattern for odd and even lines.
 **/
void ispccdc_config_culling(struct ispccdc_culling cull)
{
	u32 culling_val = 0;

	culling_val |= cull.v_pattern << ISPCCDC_CULLING_CULV_SHIFT;
	culling_val |= cull.h_even << ISPCCDC_CULLING_CULHEVN_SHIFT;
	culling_val |= cull.h_odd << ISPCCDC_CULLING_CULHODD_SHIFT;

	spin_lock(&ispccdc_obj.ispccdc_lock);
	omap_writel(culling_val, ISPCCDC_CULLING);
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_config_culling);

/**
 * ispccdc_enable_lpf - Enables the Low-Pass Filter (LPF).
 * @enable: 0 Disables LPF, 1 Enables LPF
 **/
void ispccdc_enable_lpf(u8 enable)
{
	spin_lock(&ispccdc_obj.ispccdc_lock);
	if (enable) {
		omap_writel(omap_readl(ISPCCDC_SYN_MODE) |
							ISPCCDC_SYN_MODE_LPF,
							ISPCCDC_SYN_MODE);
	} else {
		omap_writel(omap_readl(ISPCCDC_SYN_MODE) &
							~ISPCCDC_SYN_MODE_LPF,
							ISPCCDC_SYN_MODE);
	}
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_enable_lpf);

/**
 * ispccdc_config_alaw - Configures the input width for A-law.
 * @ipwidth: Input width for A-law
 **/
void ispccdc_config_alaw(enum alaw_ipwidth ipwidth)
{
	mutex_lock(&ispccdc_obj.ispccdc_mutex);
	omap_writel(ipwidth << ISPCCDC_ALAW_GWDI_SHIFT, ISPCCDC_ALAW);
	mutex_lock(&ispccdc_obj.ispccdc_mutex);
}
EXPORT_SYMBOL(ispccdc_config_alaw);

/**
 * ispccdc_enable_alaw - Enables the A-law compression.
 * @enable: 0 - Disables A-law, 1 - Enables A-law
 **/
void ispccdc_enable_alaw(u8 enable)
{
	spin_lock(&ispccdc_obj.ispccdc_lock);
	if (enable) {
		omap_writel((omap_readl(ISPCCDC_ALAW)) | ISPCCDC_ALAW_CCDTBL,
								ISPCCDC_ALAW);
	} else {
		omap_writel((omap_readl(ISPCCDC_ALAW)) & ~ISPCCDC_ALAW_CCDTBL,
								ISPCCDC_ALAW);
	}
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_enable_alaw);

/**
 * ispccdc_config_imgattr - Configures the sensor image specific attributes.
 * @colptn: Color pattern of the sensor.
 **/
void ispccdc_config_imgattr(u32 colptn)
{
	spin_lock(&ispccdc_obj.ispccdc_lock);
	omap_writel(colptn, ISPCCDC_COLPTN);
	spin_unlock(&ispccdc_obj.ispccdc_lock);
}
EXPORT_SYMBOL(ispccdc_config_imgattr);

/**
 * ispccdc_config_shadow_registers - Programs the shadow registers for CCDC.
 * Currently nothing to program in shadow, but kept for future use.
 **/
void ispccdc_config_shadow_registers(void)
{
	return;
}
EXPORT_SYMBOL(ispccdc_config_shadow_registers);

/**
 * ispccdc_try_size - Checks if requested Input/output dimensions are valid
 * @input_w: input width for the CCDC in number of pixels per line
 * @input_h: input height for the CCDC in number of lines
 * @output_w: output width from the CCDC in number of pixels per line
 * @output_h: output height for the CCDC in number of lines
 *
 * Calculates the number of pixels cropped if the reformater is disabled,
 * Fills up the output width and height variables in the isp_ccdc structure.
 *
 * Returns 0 if successful, or -EINVAL if the input width is less than 2 pixels
 **/
int ispccdc_try_size(u32 input_w, u32 input_h, u32 *output_w, u32 *output_h)
{
	if (input_w < 32 || input_h < 32) {
		DPRINTK_ISPCCDC("ISP_ERR: CCDC cannot handle input width less"
				" than 32 pixels or height less than 32\n");
		return -EINVAL;
	}

	if (ispccdc_obj.crop_w)
		*output_w = ispccdc_obj.crop_w;
	else
		*output_w = input_w;

	if (ispccdc_obj.crop_h)
		*output_h = ispccdc_obj.crop_h;
	else
		*output_h = input_h;

	if ((!ispccdc_obj.refmt_en) && ((ispccdc_obj.ccdc_outfmt !=
		CCDC_OTHERS_MEM) && ispccdc_obj.ccdc_outfmt !=
					CCDC_OTHERS_VP_MEM))
		*output_h -= 1;

	if (ispccdc_obj.ccdc_outfmt == CCDC_OTHERS_VP) {
		*output_h -= ispccdc_obj.ccdcin_hoffset;
		*output_w -= ispccdc_obj.ccdcin_woffset;
		*output_h &= 0xFFFFFFFE;
		*output_w &= 0xFFFFFFFE;
	}

	if ((ispccdc_obj.ccdc_outfmt == CCDC_OTHERS_MEM) ||
						(ispccdc_obj.ccdc_outfmt ==
						CCDC_OTHERS_VP_MEM) ||
						(ispccdc_obj.ccdc_outfmt ==
						CCDC_OTHERS_LSC_MEM)) {
		if (*output_w % 16) {
			*output_w -= (*output_w % 16);
			*output_w += 16;
		}
	}

	ispccdc_obj.ccdcout_w = *output_w;
	ispccdc_obj.ccdcout_h = *output_h;
	ispccdc_obj.ccdcin_w = input_w;
	ispccdc_obj.ccdcin_h = input_h;

	DPRINTK_ISPCCDC("try size: ccdcin_w=%u,ccdcin_h=%u,ccdcout_w=%u,"
							" ccdcout_h=%u\n",
							ispccdc_obj.ccdcin_w,
							ispccdc_obj.ccdcin_h,
							ispccdc_obj.ccdcout_w,
							ispccdc_obj.ccdcout_h);

	return 0;
}
EXPORT_SYMBOL(ispccdc_try_size);

/**
 * ispccdc_config_size - Configure the dimensions of the CCDC input/output
 * @input_w: input width for the CCDC in number of pixels per line
 * @input_h: input height for the CCDC in number of lines
 * @output_w: output width from the CCDC in number of pixels per line
 * @output_h: output height for the CCDC in number of lines
 *
 * Configures the appropriate values stored in the isp_ccdc structure to
 * HORZ/VERT_INFO registers and the VP_OUT depending on whether the image
 * is stored in memory or given to the another module in the ISP pipeline.
 *
 * Returns 0 if successful, or -EINVAL if try_size was not called before to
 * validate the requested dimensions.
 **/
int ispccdc_config_size(u32 input_w, u32 input_h, u32 output_w, u32 output_h)
{
	DPRINTK_ISPCCDC("config size: input_w=%u, input_h=%u, output_w=%u,"
							" output_h=%u\n",
							input_w, input_h,
							output_w, output_h);
	if ((output_w != ispccdc_obj.ccdcout_w) || (output_h !=
						ispccdc_obj.ccdcout_h)) {
		DPRINTK_ISPCCDC("ISP_ERR : ispccdc_try_size should"
					" be called before config size\n");
		return -EINVAL;
	}

	if (ispccdc_obj.ccdc_outfmt == CCDC_OTHERS_VP) {
		omap_writel((ispccdc_obj.ccdcin_woffset <<
					ISPCCDC_FMT_HORZ_FMTSPH_SHIFT) |
			((ispccdc_obj.ccdcin_w-ispccdc_obj.ccdcin_woffset) <<
					ISPCCDC_FMT_HORZ_FMTLNH_SHIFT),
					ISPCCDC_FMT_HORZ);
		omap_writel((ispccdc_obj.ccdcin_hoffset <<
					ISPCCDC_FMT_VERT_FMTSLV_SHIFT) |
			((ispccdc_obj.ccdcin_h-ispccdc_obj.ccdcin_hoffset) <<
					ISPCCDC_FMT_VERT_FMTLNV_SHIFT),
					ISPCCDC_FMT_VERT);
		omap_writel((ispccdc_obj.ccdcout_w <<
					ISPCCDC_VP_OUT_HORZ_NUM_SHIFT) |
					(ispccdc_obj.ccdcout_h <<
					ISPCCDC_VP_OUT_VERT_NUM_SHIFT),
					ISPCCDC_VP_OUT);
		omap_writel((((ispccdc_obj.ccdcout_h - 25) &
					ISPCCDC_VDINT_0_MASK) <<
					ISPCCDC_VDINT_0_SHIFT) |
					((50 & ISPCCDC_VDINT_1_MASK) <<
					ISPCCDC_VDINT_1_SHIFT), ISPCCDC_VDINT);

	} else if (ispccdc_obj.ccdc_outfmt == CCDC_OTHERS_MEM) {
		omap_writel(0, ISPCCDC_VP_OUT);
		if (cpu_is_omap3410()) {
			omap_writel(0 << ISPCCDC_HORZ_INFO_SPH_SHIFT |
						((ispccdc_obj.ccdcout_w - 1) <<
						ISPCCDC_HORZ_INFO_NPH_SHIFT),
						ISPCCDC_HORZ_INFO);
		} else {
			if (ispccdc_obj.ccdc_inpfmt == CCDC_RAW) {
				omap_writel(ispccdc_obj.ccdcin_woffset
						<< ISPCCDC_HORZ_INFO_SPH_SHIFT
						| ((ispccdc_obj.ccdcout_w - 1)
						<< ISPCCDC_HORZ_INFO_NPH_SHIFT),
						ISPCCDC_HORZ_INFO);
			} else {
				omap_writel(0 << ISPCCDC_HORZ_INFO_SPH_SHIFT
						| ((ispccdc_obj.ccdcout_w - 1)
						<< ISPCCDC_HORZ_INFO_NPH_SHIFT),
						ISPCCDC_HORZ_INFO);
			}
		}
		omap_writel(ispccdc_obj.ccdcin_hoffset
				<< ISPCCDC_VERT_START_SLV0_SHIFT,
				ISPCCDC_VERT_START);
		omap_writel((ispccdc_obj.ccdcout_h - 1) <<
						ISPCCDC_VERT_LINES_NLV_SHIFT,
						ISPCCDC_VERT_LINES);

		ispccdc_config_outlineoffset(ispccdc_obj.ccdcout_w * 2, 0, 0);
		omap_writel((((ispccdc_obj.ccdcout_h - 2) &
					ISPCCDC_VDINT_0_MASK) <<
					ISPCCDC_VDINT_0_SHIFT) |
					((50 & ISPCCDC_VDINT_1_MASK) <<
					ISPCCDC_VDINT_1_SHIFT), ISPCCDC_VDINT);
	} else if (ispccdc_obj.ccdc_outfmt == CCDC_OTHERS_VP_MEM) {
		omap_writel((1 << ISPCCDC_FMT_HORZ_FMTSPH_SHIFT) |
					(ispccdc_obj.ccdcin_w <<
					ISPCCDC_FMT_HORZ_FMTLNH_SHIFT),
					ISPCCDC_FMT_HORZ);
		omap_writel((0 << ISPCCDC_FMT_VERT_FMTSLV_SHIFT) |
					((ispccdc_obj.ccdcin_h) <<
					ISPCCDC_FMT_VERT_FMTLNV_SHIFT),
					ISPCCDC_FMT_VERT);
		omap_writel((ispccdc_obj.ccdcout_w
					<< ISPCCDC_VP_OUT_HORZ_NUM_SHIFT) |
					(ispccdc_obj.ccdcout_h <<
					ISPCCDC_VP_OUT_VERT_NUM_SHIFT),
					ISPCCDC_VP_OUT);
/* MMS: fix wrong pattern */
/*
		omap_writel(0 << ISPCCDC_HORZ_INFO_SPH_SHIFT |
					((ispccdc_obj.ccdcout_w - 1) <<
					ISPCCDC_HORZ_INFO_NPH_SHIFT),
					ISPCCDC_HORZ_INFO);
*/
		omap_writel(1 << ISPCCDC_HORZ_INFO_SPH_SHIFT |
					((ispccdc_obj.ccdcout_w - 1) <<
					ISPCCDC_HORZ_INFO_NPH_SHIFT),
					ISPCCDC_HORZ_INFO);
		omap_writel(0 << ISPCCDC_VERT_START_SLV0_SHIFT,
					ISPCCDC_VERT_START);
		omap_writel((ispccdc_obj.ccdcout_h - 1) <<
					ISPCCDC_VERT_LINES_NLV_SHIFT,
					ISPCCDC_VERT_LINES);
		ispccdc_config_outlineoffset(ispccdc_obj.ccdcout_w * 2, 0, 0);
		omap_writel((((ispccdc_obj.ccdcout_h - 25) &
					ISPCCDC_VDINT_0_MASK) <<
					ISPCCDC_VDINT_0_SHIFT) |
					((50 & ISPCCDC_VDINT_1_MASK) <<
					ISPCCDC_VDINT_1_SHIFT), ISPCCDC_VDINT);
	} else if (ispccdc_obj.ccdc_outfmt == CCDC_OTHERS_LSC_MEM) {
		/* Added by MMS */
		/* Start with 1 pixel apart */
		omap_writel((1 << ISPCCDC_FMT_HORZ_FMTSPH_SHIFT)
				| (ispccdc_obj.ccdcin_w
				<< ISPCCDC_FMT_HORZ_FMTLNH_SHIFT),
				ISPCCDC_FMT_HORZ);

		omap_writel((0 << ISPCCDC_FMT_VERT_FMTSLV_SHIFT)
				| ((ispccdc_obj.ccdcin_h)
				<< ISPCCDC_FMT_VERT_FMTLNV_SHIFT),
				ISPCCDC_FMT_VERT);

		omap_writel((ispccdc_obj.ccdcout_w
				<< ISPCCDC_VP_OUT_HORZ_NUM_SHIFT)
				| (ispccdc_obj.ccdcout_h
				<< ISPCCDC_VP_OUT_VERT_NUM_SHIFT),
				ISPCCDC_VP_OUT);
		omap_writel(0 << ISPCCDC_HORZ_INFO_SPH_SHIFT
				| ((ispccdc_obj.ccdcout_w - 1)
				<< ISPCCDC_HORZ_INFO_NPH_SHIFT),
				ISPCCDC_HORZ_INFO);
		omap_writel(0 << ISPCCDC_VERT_START_SLV0_SHIFT,
				ISPCCDC_VERT_START);
		omap_writel((ispccdc_obj.ccdcout_h - 1)
				<< ISPCCDC_VERT_LINES_NLV_SHIFT,
				ISPCCDC_VERT_LINES);
		/*Configure the HSIZE_OFF with output buffer width*/

		ispccdc_config_outlineoffset((ispccdc_obj.ccdcout_w * 2), 0, 0);
		omap_writel((((ispccdc_obj.ccdcout_h - 25)
				& ISPCCDC_VDINT_0_MASK)
				<< ISPCCDC_VDINT_0_SHIFT)
				| (((50) &  ISPCCDC_VDINT_1_MASK)
				<< ISPCCDC_VDINT_1_SHIFT),
				ISPCCDC_VDINT);
	}

	if (is_isplsc_activated()) {
		if ((ispccdc_obj.ccdc_inpfmt == CCDC_RAW) ||
			(ispccdc_obj.ccdc_inpfmt == CCDC_RAW_PATTERN)) {
			ispccdc_config_lsc(&lsc_config);
			ispccdc_load_lsc(lsc_config.size);
		}
	}

	return 0;
}
EXPORT_SYMBOL(ispccdc_config_size);

/**
 * ispccdc_config_outlineoffset - Configures the output line offset
 * @offset: Must be twice the Output width and aligned on 32 byte boundary
 * @oddeven: Specifies the odd/even line pattern to be chosen to store the
 *           output.
 * @numlines: Set the value 0-3 for +1-4lines, 4-7 for -1-4lines.
 *
 * - Configures the output line offset when stored in memory
 * - Sets the odd/even line pattern to store the output
 *    (EVENEVEN (1), ODDEVEN (2), EVENODD (3), ODDODD (4))
 * - Configures the number of even and odd line fields in case of rearranging
 * the lines.
 *
 * Returns 0 if successful, or -EINVAL if the offset is not in 32 byte
 * boundary.
 **/
int ispccdc_config_outlineoffset(u32 offset, u8 oddeven, u8 numlines)
{
	if ((offset & ISP_32B_BOUNDARY_OFFSET) == offset)
		omap_writel((offset & 0xFFFF), ISPCCDC_HSIZE_OFF);
	else {
		DPRINTK_ISPCCDC("ISP_ERR : Offset should be in 32 byte"
								" boundary");
		return -EINVAL;
	}

	omap_writel((omap_readl(ISPCCDC_SDOFST) & (~ISPCCDC_SDOFST_FINV)),
							ISPCCDC_SDOFST);

	omap_writel(omap_readl(ISPCCDC_SDOFST) & ISPCCDC_SDOFST_FOFST_1L,
							ISPCCDC_SDOFST);

	switch (oddeven) {
	case EVENEVEN:
		omap_writel((omap_readl(ISPCCDC_SDOFST)) | ((numlines & 0x7) <<
						ISPCCDC_SDOFST_LOFST0_SHIFT),
						ISPCCDC_SDOFST);
		break;
	case ODDEVEN:
		omap_writel((omap_readl(ISPCCDC_SDOFST)) | ((numlines & 0x7) <<
						ISPCCDC_SDOFST_LOFST1_SHIFT),
						ISPCCDC_SDOFST);
		break;
	case EVENODD:
		omap_writel((omap_readl(ISPCCDC_SDOFST)) | ((numlines & 0x7) <<
						ISPCCDC_SDOFST_LOFST2_SHIFT),
						ISPCCDC_SDOFST);
		break;
	case ODDODD:
		omap_writel((omap_readl(ISPCCDC_SDOFST)) | ((numlines & 0x7) <<
						ISPCCDC_SDOFST_LOFST3_SHIFT),
						ISPCCDC_SDOFST);
		break;
	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL(ispccdc_config_outlineoffset);

/**
 * ispccdc_set_outaddr - Sets the memory address where the output will be saved
 * @addr: 32-bit memory address aligned on 32 byte boundary.
 *
 * Sets the memory address where the output will be saved.
 *
 * Returns 0 if successful, or -EINVAL if the address is not in the 32 byte
 * boundary.
 **/
int ispccdc_set_outaddr(u32 addr)
{
	if ((addr & ISP_32B_BOUNDARY_BUF) == addr) {
		omap_writel(addr, ISPCCDC_SDR_ADDR);
		return 0;
	} else {
		DPRINTK_ISPCCDC("ISP_ERR : Address should be in 32 byte"
								" boundary");
		return -EINVAL;
	}

}
EXPORT_SYMBOL(ispccdc_set_outaddr);

/**
 * ispccdc_enable - Enables the CCDC module.
 * @enable: 0 Disables CCDC, 1 Enables CCDC
 *
 * Client should configure all the sub modules in CCDC before this.
 **/
void ispccdc_enable(u8 enable)
{
	if (enable) {
		if (ccdc_use_lsc && !ispccdc_obj.lsc_en &&
			((ispccdc_obj.ccdc_inpfmt == CCDC_RAW) ||
			(ispccdc_obj.ccdc_inpfmt == CCDC_RAW_PATTERN)))
			ispccdc_enable_lsc(1);
			omap_writel(omap_readl(ISPCCDC_PCR) | (ISPCCDC_PCR_EN),
								ISPCCDC_PCR);
	} else {
		omap_writel(omap_readl(ISPCCDC_PCR) & ~(ISPCCDC_PCR_EN),
								ISPCCDC_PCR);
	}

}
EXPORT_SYMBOL(ispccdc_enable);

/**
 * ispccdc_busy - Gets busy state of the CCDC.
 **/
int ispccdc_busy(void)
{
	return omap_readl(ISPCCDC_PCR) & ISPCCDC_PCR_BUSY;
}
EXPORT_SYMBOL(ispccdc_busy);

/**
 * ispccdc_save_context - Saves the values of the CCDC module registers
 **/
void ispccdc_save_context(void)
{
	DPRINTK_ISPCCDC("Saving context\n");
	isp_save_context(ispccdc_reg_list);

}
EXPORT_SYMBOL(ispccdc_save_context);

/**
 * ispccdc_restore_context - Restores the values of the CCDC module registers
 **/
void ispccdc_restore_context(void)
{
	DPRINTK_ISPCCDC("Restoring context\n");
	isp_restore_context(ispccdc_reg_list);
}
EXPORT_SYMBOL(ispccdc_restore_context);

/**
 * ispccdc_print_status - Prints the values of the CCDC Module registers
 *
 * Also prints other debug information stored in the CCDC module.
 **/
void ispccdc_print_status(void)
{
	if (!is_ispccdc_debug_enabled())
		return;

	DPRINTK_ISPCCDC("Module in use =%d\n", ispccdc_obj.ccdc_inuse);
	DPRINTK_ISPCCDC("Accepted CCDC Input (width = %d,Height = %d)\n",
							ispccdc_obj.ccdcin_w,
							ispccdc_obj.ccdcin_h);
	DPRINTK_ISPCCDC("Accepted CCDC Output (width = %d,Height = %d)\n",
							ispccdc_obj.ccdcout_w,
							ispccdc_obj.ccdcout_h);
	DPRINTK_ISPCCDC("###CCDC PCR=0x%x\n", omap_readl(ISPCCDC_PCR));
	DPRINTK_ISPCCDC("ISP_CTRL =0x%x\n", omap_readl(ISP_CTRL));
	switch (ispccdc_obj.ccdc_inpfmt) {
	case CCDC_RAW:
		DPRINTK_ISPCCDC("ccdc input format is CCDC_RAW\n");
		break;
	case CCDC_YUV_SYNC:
		DPRINTK_ISPCCDC("ccdc input format is CCDC_YUV_SYNC\n");
		break;
	case CCDC_YUV_BT:
		DPRINTK_ISPCCDC("ccdc input format is CCDC_YUV_BT\n");
		break;
	}

	switch (ispccdc_obj.ccdc_outfmt) {
	case CCDC_OTHERS_VP:
		DPRINTK_ISPCCDC("ccdc output format is CCDC_OTHERS_VP\n");
		break;
	case CCDC_OTHERS_MEM:
		DPRINTK_ISPCCDC("ccdc output format is CCDC_OTHERS_MEM\n");
		break;
	case CCDC_YUV_RSZ:
		DPRINTK_ISPCCDC("ccdc output format is CCDC_YUV_RSZ\n");
		break;
	}

	DPRINTK_ISPCCDC("###ISP_CTRL in ccdc =0x%x\n", omap_readl(ISP_CTRL));
	DPRINTK_ISPCCDC("###ISP_IRQ0ENABLE in ccdc =0x%x\n",
						omap_readl(ISP_IRQ0ENABLE));
	DPRINTK_ISPCCDC("###ISP_IRQ0STATUS in ccdc =0x%x\n",
						omap_readl(ISP_IRQ0STATUS));
	DPRINTK_ISPCCDC("###CCDC SYN_MODE=0x%x\n",
						omap_readl(ISPCCDC_SYN_MODE));
	DPRINTK_ISPCCDC("###CCDC HORZ_INFO=0x%x\n",
						omap_readl(ISPCCDC_HORZ_INFO));
	DPRINTK_ISPCCDC("###CCDC VERT_START=0x%x\n",
					omap_readl(ISPCCDC_VERT_START));
	DPRINTK_ISPCCDC("###CCDC VERT_LINES=0x%x\n",
					omap_readl(ISPCCDC_VERT_LINES));
	DPRINTK_ISPCCDC("###CCDC CULLING=0x%x\n", omap_readl(ISPCCDC_CULLING));
	DPRINTK_ISPCCDC("###CCDC HSIZE_OFF=0x%x\n",
						omap_readl(ISPCCDC_HSIZE_OFF));
	DPRINTK_ISPCCDC("###CCDC SDOFST=0x%x\n", omap_readl(ISPCCDC_SDOFST));
	DPRINTK_ISPCCDC("###CCDC SDR_ADDR=0x%x\n",
						omap_readl(ISPCCDC_SDR_ADDR));
	DPRINTK_ISPCCDC("###CCDC CLAMP=0x%x\n", omap_readl(ISPCCDC_CLAMP));
	DPRINTK_ISPCCDC("###CCDC COLPTN=0x%x\n", omap_readl(ISPCCDC_COLPTN));
	DPRINTK_ISPCCDC("###CCDC CFG=0x%x\n", omap_readl(ISPCCDC_CFG));
	DPRINTK_ISPCCDC("###CCDC VP_OUT=0x%x\n", omap_readl(ISPCCDC_VP_OUT));
	DPRINTK_ISPCCDC("###CCDC_SDR_ADDR= 0x%x\n",
						omap_readl(ISPCCDC_SDR_ADDR));
	DPRINTK_ISPCCDC("###CCDC FMTCFG=0x%x\n", omap_readl(ISPCCDC_FMTCFG));
	DPRINTK_ISPCCDC("###CCDC FMT_HORZ=0x%x\n",
						omap_readl(ISPCCDC_FMT_HORZ));
	DPRINTK_ISPCCDC("###CCDC FMT_VERT=0x%x\n",
						omap_readl(ISPCCDC_FMT_VERT));
	DPRINTK_ISPCCDC("###CCDC LSC_CONFIG=0x%x\n",
					omap_readl(ISPCCDC_LSC_CONFIG));
	DPRINTK_ISPCCDC("###CCDC LSC_INIT=0x%x\n",
					omap_readl(ISPCCDC_LSC_INITIAL));
	DPRINTK_ISPCCDC("###CCDC LSC_TABLE BASE=0x%x\n",
					omap_readl(ISPCCDC_LSC_TABLE_BASE));
	DPRINTK_ISPCCDC("###CCDC LSC TABLE OFFSET=0x%x\n",
					omap_readl(ISPCCDC_LSC_TABLE_OFFSET));
	/* Added by MMS */
	DPRINTK_ISPCCDC("###CCDC ISPCCDC_BLKCMP=0x%x\n",
						omap_readl(ISPCCDC_BLKCMP));
	DPRINTK_ISPCCDC("###CCDC ISPCCDC_DCSUB=0x%x\n",
						omap_readl(ISPCCDC_DCSUB));
	DPRINTK_ISPCCDC("###CCDC ISPCCDC_FPC=0x%x\n", omap_readl(ISPCCDC_FPC));

}
EXPORT_SYMBOL(ispccdc_print_status);

/**
 * isp_ccdc_init - CCDC module initialization.
 *
 * Always returns 0
 **/
int __init isp_ccdc_init(void)
{
	ispccdc_obj.ccdc_inuse = 0;
	ispccdc_obj.dcsub = 0;
	ispccdc_config_crop(0, 0, 0, 0);
	mutex_init(&ispccdc_obj.ispccdc_mutex);
	spin_lock_init(&ispccdc_obj.ispccdc_lock);

	if (is_isplsc_activated()) {
		memset(ispccdc_lsc_tbl, 0x40, LSC_TABLE_INIT_SIZE);
		lsc_config.initial_x = 0;
		lsc_config.initial_y = 0;
		lsc_config.gain_mode_n = 0x6;
		lsc_config.gain_mode_m = 0x6;
		lsc_config.gain_format = 0x4;
		lsc_config.offset = 0x60;
		lsc_config.size = LSC_TABLE_INIT_SIZE;
		ccdc_use_lsc = 1;
	}

	return 0;
}

/**
 * isp_ccdc_cleanup - CCDC module cleanup.
 **/
void isp_ccdc_cleanup(void)
{
	if (is_isplsc_activated()) {
		if (lsc_initialized) {
			ispmmu_unmap(lsc_ispmmu_addr);
			kfree(lsc_gain_table);
			lsc_initialized = 0;
		}
	}

	if (fpc_table_add_m != 0) {
		ispmmu_unmap(fpc_table_add_m);
		kfree(fpc_table_add);
	}
}
