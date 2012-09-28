/*
 * drivers/media/video/isp/ispccdc.h
 *
 * Driver header file for CCDC module in TI's OMAP3430 Camera ISP
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

#ifndef OMAP_ISP_CCDC_H
#define OMAP_ISP_CCDC_H

#include <mach/oldisp_user.h>

#ifndef CONFIG_ARCH_OMAP3410
# define cpu_is_omap3410()		0
# define is_isplsc_activated()		1
#else
# define cpu_is_omap3410()		1
# define is_isplsc_activated()		0
#endif

#ifdef OMAP_ISPCCDC_DEBUG
# define is_ispccdc_debug_enabled()     1
#else
# define is_ispccdc_debug_enabled()     0
#endif

/* Enumeration constants for CCDC input output format */
enum ccdc_input {
	CCDC_RAW,
	CCDC_YUV_SYNC,
	CCDC_YUV_BT,
	CCDC_RAW_PATTERN,
	CCDC_RAW_10_BIT_PATTERN,
	CCDC_OTHERS
};

enum ccdc_output {
	CCDC_YUV_RSZ,
	CCDC_YUV_MEM_RSZ,
	CCDC_OTHERS_VP,
	CCDC_OTHERS_MEM,
	CCDC_OTHERS_VP_MEM,
	CCDC_OTHERS_LSC_MEM
};

/* Enumeration constants for the sync interface parameters */
enum inpmode {
	RAW,
	YUV16,
	YUV8
};
enum datasize {
	DAT8,
	DAT10,
	DAT11,
	DAT12
};


/**
 * struct ispccdc_syncif - Structure for Sync Interface between sensor and CCDC
 * @ccdc_mastermode: Master mode. 1 - Master, 0 - Slave.
 * @fldstat: Field state. 0 - Odd Field, 1 - Even Field.
 * @ipmod: Input mode.
 * @datsz: Data size.
 * @fldmode: 0 - Progressive, 1 - Interlaced.
 * @datapol: 0 - Positive, 1 - Negative.
 * @fldpol: 0 - Positive, 1 - Negative.
 * @hdpol: 0 - Positive, 1 - Negative.
 * @vdpol: 0 - Positive, 1 - Negative.
 * @fldout: 0 - Input, 1 - Output.
 * @hs_width: Width of the Horizontal Sync pulse, used for HS/VS Output.
 * @vs_width: Width of the Vertical Sync pulse, used for HS/VS Output.
 * @ppln: Number of pixels per line, used for HS/VS Output.
 * @hlprf: Number of half lines per frame, used for HS/VS Output.
 * @bt_r656_en: 1 - Enable ITU-R BT656 mode, 0 - Sync mode.
 */
struct ispccdc_syncif {
	u8 ccdc_mastermode;
	u8 fldstat;
	enum inpmode ipmod;
	enum datasize datsz;
	u8 fldmode;
	u8 datapol;
	u8 fldpol;
	u8 hdpol;
	u8 vdpol;
	u8 fldout;
	u8 hs_width;
	u8 vs_width;
	u8 ppln;
	u8 hlprf;
	u8 bt_r656_en;
};

/**
 * ispccdc_refmt - Structure for Reformatter parameters
 * @lnalt: Line alternating mode enable. 0 - Enable, 1 - Disable.
 * @lnum: Number of output lines from 1 input line. 1 to 4 lines.
 * @plen_even: Number of program entries in even line minus 1.
 * @plen_odd: Number of program entries in odd line minus 1.
 * @prgeven0: Program entries 0-7 for even lines register
 * @prgeven1: Program entries 8-15 for even lines register
 * @prgodd0: Program entries 0-7 for odd lines register
 * @prgodd1: Program entries 8-15 for odd lines register
 * @fmtaddr0: Output line in which the original pixel is to be placed
 * @fmtaddr1: Output line in which the original pixel is to be placed
 * @fmtaddr2: Output line in which the original pixel is to be placed
 * @fmtaddr3: Output line in which the original pixel is to be placed
 * @fmtaddr4: Output line in which the original pixel is to be placed
 * @fmtaddr5: Output line in which the original pixel is to be placed
 * @fmtaddr6: Output line in which the original pixel is to be placed
 * @fmtaddr7: Output line in which the original pixel is to be placed
 */
struct ispccdc_refmt {
	u8 lnalt;
	u8 lnum;
	u8 plen_even;
	u8 plen_odd;
	u32 prgeven0;
	u32 prgeven1;
	u32 prgodd0;
	u32 prgodd1;
	u32 fmtaddr0;
	u32 fmtaddr1;
	u32 fmtaddr2;
	u32 fmtaddr3;
	u32 fmtaddr4;
	u32 fmtaddr5;
	u32 fmtaddr6;
	u32 fmtaddr7;
};

int ispccdc_request(void);

int ispccdc_free(void);

int ispccdc_config_datapath(enum ccdc_input input, enum ccdc_output output);

void ispccdc_config_crop(u32 left, u32 top, u32 height, u32 width);

void ispccdc_config_sync_if(struct ispccdc_syncif syncif);

int ispccdc_config_black_clamp(struct ispccdc_bclamp bclamp);

void ispccdc_enable_black_clamp(u8 enable);

int ispccdc_config_fpc(struct ispccdc_fpc fpc);

void ispccdc_enable_fpc(u8 enable);

void ispccdc_config_black_comp(struct ispccdc_blcomp blcomp);

void ispccdc_config_vp(struct ispccdc_vp vp);

void ispccdc_enable_vp(u8 enable);

void ispccdc_config_reformatter(struct ispccdc_refmt refmt);

void ispccdc_enable_reformatter(u8 enable);

void ispccdc_config_culling(struct ispccdc_culling culling);

void ispccdc_enable_lpf(u8 enable);

void ispccdc_config_alaw(enum alaw_ipwidth ipwidth);

void ispccdc_enable_alaw(u8 enable);

int ispccdc_load_lsc(u32 table_size);

void ispccdc_config_lsc(struct ispccdc_lsc_config *lsc_cfg);

void ispccdc_enable_lsc(u8 enable);

void ispccdc_config_imgattr(u32 colptn);

void ispccdc_config_shadow_registers(void);

int ispccdc_try_size(u32 input_w, u32 input_h, u32 *output_w, u32 *output_h);

int ispccdc_config_size(u32 input_w, u32 input_h, u32 output_w, u32 output_h);

int ispccdc_config_outlineoffset(u32 offset, u8 oddeven, u8 numlines);

int ispccdc_set_outaddr(u32 addr);

void ispccdc_enable(u8 enable);

int ispccdc_busy(void);

void ispccdc_save_context(void);

void ispccdc_restore_context(void);

void ispccdc_print_status(void);

int omap34xx_isp_ccdc_config(void *userspace_add);

void ispccdc_set_wenlog(u32 wenlog);

void ispccdc_set_dcsub(u32 dcsub);

void ispccdc_set_crop_offset(enum ispccdc_raw_fmt);

#endif		/* OMAP_ISP_CCDC_H */
