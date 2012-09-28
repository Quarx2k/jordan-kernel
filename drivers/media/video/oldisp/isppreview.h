/*
 * drivers/media/video/isp/isppreview.h
 *
 * Driver header file for Preview module in TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
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

#ifndef OMAP_ISP_PREVIEW_H
#define OMAP_ISP_PREVIEW_H

#include <mach/oldisp_user.h>
/* Isp query control structure */

#define ISPPRV_BRIGHT_STEP		0x1
#define ISPPRV_BRIGHT_DEF		0x0
#define ISPPRV_BRIGHT_LOW		0x0
#define ISPPRV_BRIGHT_HIGH		0xF
#define ISPPRV_BRIGHT_UNITS      	0x7

#define ISPPRV_CONTRAST_STEP	0x1
#define ISPPRV_CONTRAST_DEF	0x10
#define ISPPRV_CONTRAST_LOW	0x0
#define ISPPRV_CONTRAST_HIGH	0x10
#define ISPPRV_CONTRAST_UNITS	0x1



#define NO_AVE				0x0
#define AVE_2_PIX			0x1
#define AVE_4_PIX			0x2
#define AVE_8_PIX			0x3
#define AVE_ODD_PIXEL_DIST		(1 << 4) /* For Bayer Sensors */
#define AVE_EVEN_PIXEL_DIST		(1 << 2)

#define WB_GAIN_MAX			4

/* Features list */
#define PREV_AVERAGER			(1 << 0)
#define PREV_INVERSE_ALAW 		(1 << 1)
#define PREV_HORZ_MEDIAN_FILTER		(1 << 2)
#define PREV_NOISE_FILTER 		(1 << 3)
#define PREV_CFA			(1 << 4)
#define PREV_GAMMA_BYPASS		(1 << 5)
#define PREV_LUMA_ENHANCE		(1 << 6)
#define PREV_CHROMA_SUPPRESS		(1 << 7)
#define PREV_DARK_FRAME_SUBTRACT	(1 << 8)
#define PREV_LENS_SHADING		(1 << 9)
#define PREV_DARK_FRAME_CAPTURE		(1 << 10)
#define PREV_DEFECT_COR			(1 << 11)


#define ISP_NF_TABLE_SIZE 		(1 << 10)

#define ISP_GAMMA_TABLE_SIZE 		(1 << 10)

/* Table addresses */
#define ISPPRV_TBL_ADDR_RED_G_START  0x00
#define ISPPRV_TBL_ADDR_BLUE_G_START  0x800
#define ISPPRV_TBL_ADDR_GREEN_G_START  0x400

/*
 *Enumeration Constants for input and output format
 */
enum preview_input {
	PRV_RAW_CCDC,
	PRV_RAW_MEM,
	PRV_RGBBAYERCFA,
	PRV_COMPCFA,
	PRV_CCDC_DRKF,
	PRV_OTHERS
};
enum preview_output {
	PREVIEW_RSZ,
	PREVIEW_MEM
};
/*
 * Configure byte layout of YUV image
 */
enum preview_ycpos_mode {
	YCPOS_YCrYCb = 0,
	YCPOS_YCbYCr = 1,
	YCPOS_CbYCrY = 2,
	YCPOS_CrYCbY = 3
};

enum preview_color_effect {
	PREV_DEFAULT_COLOR = 0,
	PREV_SEPIA_COLOR = 1,
	PREV_BW_COLOR = 2
};


/**
 * struct ispprev_gtable - Structure for Gamma Correction.
 * @redtable: Pointer to the red gamma table.
 * @greentable: Pointer to the green gamma table.
 * @bluetable: Pointer to the blue gamma table.
 */
struct ispprev_gtable {
	u32 *redtable;
	u32 *greentable;
	u32 *bluetable;
};

/**
 * struct prev_white_balance - Structure for White Balance 2.
 * @wb_dgain: White balance common gain.
 * @wb_gain: Individual color gains.
 * @wb_coefmatrix: Coefficient matrix
 */
struct prev_white_balance {
	u16 wb_dgain; /* white balance common gain */
	u8 wb_gain[WB_GAIN_MAX]; /* individual color gains */
	u8 wb_coefmatrix[WB_GAIN_MAX][WB_GAIN_MAX];
};

/**
 * struct prev_size_params - Structure for size parameters.
 * @hstart: Starting pixel.
 * @vstart: Starting line.
 * @hsize: Width of input image.
 * @vsize: Height of input image.
 * @pixsize: Pixel size of the image in terms of bits.
 * @in_pitch: Line offset of input image.
 * @out_pitch: Line offset of output image.
 */
struct prev_size_params {
	unsigned int hstart;
	unsigned int vstart;
	unsigned int hsize;
	unsigned int vsize;
	unsigned char pixsize;
	unsigned short in_pitch;
	unsigned short out_pitch;
};

/**
 * struct prev_rgb2ycbcr_coeffs - Structure RGB2YCbCr parameters.
 * @coeff: Color conversion gains in 3x3 matrix.
 * @offset: Color conversion offsets.
 */
struct prev_rgb2ycbcr_coeffs {
	short coeff[RGB_MAX][RGB_MAX];
	short offset[RGB_MAX];
};

/**
 * struct prev_darkfrm_params - Structure for Dark frame suppression.
 * @addr: Memory start address.
 * @offset: Line offset.
 */
struct prev_darkfrm_params {
	u32 addr;
	u32 offset;
};

/**
 * struct prev_params - Structure for all configuration
 * @features: Set of features enabled.
 * @pix_fmt: Output pixel format.
 * @cfa: CFA coefficients.
 * @csup: Chroma suppression coefficients.
 * @ytable: Pointer to Luma enhancement coefficients.
 * @nf: Noise filter coefficients.
 * @dcor: Noise filter coefficients.
 * @gtable: Gamma coefficients.
 * @wbal: White Balance parameters.
 * @blk_adj: Black adjustment parameters.
 * @rgb2rgb: RGB blending parameters.
 * @rgb2ycbcr: RGB to ycbcr parameters.
 * @hmf_params: Horizontal median filter.
 * @size_params: Size parameters.
 * @drkf_params: Darkframe parameters.
 * @lens_shading_shift:
 * @average: Downsampling rate for averager.
 * @contrast: Contrast.
 * @brightness: Brightness.
 */
struct prev_params {
	u16 features;
	enum preview_ycpos_mode pix_fmt;
	struct ispprev_cfa cfa;
	struct ispprev_csup csup;
	u32 *ytable;
	struct ispprev_nf nf;
	struct ispprev_dcor dcor;
	struct ispprev_gtable gtable;
	struct ispprev_wbal wbal;
	struct ispprev_blkadj blk_adj;
	struct ispprev_rgbtorgb rgb2rgb;
	struct ispprev_csc rgb2ycbcr;
	struct ispprev_hmed hmf_params;
	struct prev_size_params size_params;
	struct prev_darkfrm_params drkf_params;
	u8 lens_shading_shift;
	u8 average;
	u8 contrast;
	u8 brightness;
};

/**
 * struct isptables_update - Structure for Table Configuration.
 * @update: Specifies which tables should be updated.
 * @flag: Specifies which tables should be enabled.
 * @prev_nf: Pointer to structure for Noise Filter
 * @lsc: Pointer to LSC gain table. (currently not used)
 * @red_gamma: Pointer to red gamma correction table.
 * @green_gamma: Pointer to green gamma correction table.
 * @blue_gamma: Pointer to blue gamma correction table.
 */
struct isptables_update {
	u16 update;
	u16 flag;
	struct ispprev_nf *prev_nf;
	u32 *lsc;
	u32 *red_gamma;
	u32 *green_gamma;
	u32 *blue_gamma;
};

void isppreview_config_shadow_registers(void);

int isppreview_request(void);

int isppreview_free(void);

int isppreview_config_datapath(enum preview_input input,
					enum preview_output output);

void isppreview_config_ycpos(enum preview_ycpos_mode mode);

void isppreview_config_averager(u8 average);

void isppreview_enable_invalaw(u8 enable);

void isppreview_enable_drkframe(u8 enable);

void isppreview_enable_shadcomp(u8 enable);

void isppreview_config_drkf_shadcomp(u8 scomp_shtval);

void isppreview_enable_gammabypass(u8 enable);

void isppreview_enable_hmed(u8 enable);

void isppreview_config_hmed(struct ispprev_hmed);

void isppreview_enable_noisefilter(u8 enable);

void isppreview_config_noisefilter(struct ispprev_nf prev_nf);

void isppreview_enable_dcor(u8 enable);

void isppreview_config_dcor(struct ispprev_dcor prev_dcor);


void isppreview_config_cfa(struct ispprev_cfa);

void isppreview_config_gammacorrn(struct ispprev_gtable);

void isppreview_config_chroma_suppression(struct ispprev_csup csup);

void isppreview_enable_cfa(u8 enable);

void isppreview_config_luma_enhancement(u32 *ytable);

void isppreview_enable_luma_enhancement(u8 enable);

void isppreview_enable_chroma_suppression(u8 enable);

void isppreview_config_whitebalance(struct ispprev_wbal);

void isppreview_config_blkadj(struct ispprev_blkadj);

void isppreview_config_rgb_blending(struct ispprev_rgbtorgb);

void isppreview_config_rgb_to_ycbcr(struct ispprev_csc);

void isppreview_update_contrast(u8 *contrast);

void isppreview_query_contrast(u8 *contrast);

void isppreview_config_contrast(u8 contrast);

void isppreview_get_contrast_range(u8 *min_contrast, u8 *max_contrast);

void isppreview_update_brightness(u8 *brightness);

void isppreview_config_brightness(u8 brightness);

void isppreview_get_brightness_range(u8 *min_brightness, u8 *max_brightness);

void isppreview_set_color(u8 *mode);

void isppreview_get_color(u8 *mode);

void isppreview_query_brightness(u8 *brightness);

void isppreview_config_yc_range(struct ispprev_yclimit yclimit);

int isppreview_try_size(u32 input_w, u32 input_h, u32 *output_w,
				u32 *output_h);

int isppreview_config_size(u32 input_w, u32 input_h, u32 output_w,
			u32 output_h);

int isppreview_config_inlineoffset(u32 offset);

int isppreview_set_inaddr(u32 addr);

int isppreview_config_outlineoffset(u32 offset);

int isppreview_set_outaddr(u32 addr);

int isppreview_config_darklineoffset(u32 offset);

int isppreview_set_darkaddr(u32 addr);

void isppreview_enable(u8 enable);

int isppreview_busy(void);

struct prev_params *isppreview_get_config(void);

void isppreview_print_status(void);

#ifndef CONFIG_ARCH_OMAP3410
void isppreview_save_context(void);
#else
static inline void isppreview_save_context(void) {}
#endif

#ifndef CONFIG_ARCH_OMAP3410
void isppreview_restore_context(void);
#else
static inline void isppreview_restore_context(void) {}
#endif

int omap34xx_isp_preview_config(void *userspace_add);

int omap34xx_isp_tables_update(struct isptables_update *isptables_struct);

void isppreview_set_skip(u32 h, u32 v);

void isppreview_set_whitebalance(struct ispprev_wbal *wbal);

#endif/* OMAP_ISP_PREVIEW_H */
