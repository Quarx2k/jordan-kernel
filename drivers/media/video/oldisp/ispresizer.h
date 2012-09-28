/*
 * drivers/media/video/isp/ispresizer.h
 *
 * Driver header file for Resizer module in TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Contributors:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Mohit Jalori <mjalori@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_RESIZER_H
#define OMAP_ISP_RESIZER_H

/*
 * Resizer Constants
 */
#define MAX_IN_WIDTH_MEMORY_MODE	4095

#define MAX_IN_WIDTH_ONTHEFLY_MODE	1280
#define MAX_IN_WIDTH_ONTHEFLY_MODE_ES2	4095
#define MAX_IN_HEIGHT			4095
#define MINIMUM_RESIZE_VALUE		64
#define MAXIMUM_RESIZE_VALUE		1024
#define MID_RESIZE_VALUE		512

#define MAX_7TAP_HRSZ_OUTWIDTH		1280
#define MAX_7TAP_VRSZ_OUTWIDTH		640

#define MAX_7TAP_HRSZ_OUTWIDTH_ES2	3300
#define MAX_7TAP_VRSZ_OUTWIDTH_ES2	1650

#define DEFAULTSTPIXEL			0
#define DEFAULTSTPHASE			0
#define DEFAULTHSTPIXEL4TAPMODE		3
#define FOURPHASE			4
#define EIGHTPHASE			8
#define RESIZECONSTANT			256
#define SHIFTER4TAPMODE			0
#define SHIFTER7TAPMODE			1
#define DEFAULTOFFSET			7
#define OFFSETVERT4TAPMODE		4
#define OPWDALIGNCONSTANT		0xFFFFFFF0

/*
 * The client is supposed to call resizer API in the following sequence:
 * 	- request()
 * 	- config_datatpath()
 * 	- optionally config/enable sub modules
 * 	- try/config size
 * 	- setup callback
 * 	- setup in/out memory offsets and ptrs
 * 	- enable()
 * 	...
 * 	- disable()
 * 	- free()
 */

enum ispresizer_input {
	RSZ_OTFLY_YUV,
	RSZ_MEM_YUV,
	RSZ_MEM_COL8
};

/**
 * struct isprsz_coef - Structure for resizer filter coefficients.
 * @h_filter_coef_4tap: Horizontal filter coefficients for 8-phase/4-tap
 *			mode (.5x-4x)
 * @v_filter_coef_4tap: Vertical filter coefficients for 8-phase/4-tap
 *			mode (.5x-4x)
 * @h_filter_coef_7tap: Horizontal filter coefficients for 4-phase/7-tap
 *			mode (.25x-.5x)
 * @v_filter_coef_7tap: Vertical filter coefficients for 4-phase/7-tap
 *			mode (.25x-.5x)
 */
struct isprsz_coef {
	u16 h_filter_coef_4tap[32];
	u16 v_filter_coef_4tap[32];
	u16 h_filter_coef_7tap[28];
	u16 v_filter_coef_7tap[28];
};

/**
 * struct isprsz_yenh - Structure for resizer luminance enhancer parameters.
 * @algo: Algorithm select.
 * @gain: Maximum gain.
 * @slope: Slope.
 * @coreoffset: Coring offset.
 */
struct isprsz_yenh {
	u8 algo;
	u8 gain;
	u8 slope;
	u8 coreoffset;
};

void ispresizer_config_shadow_registers(void);

int ispresizer_request(void);

int ispresizer_free(void);

int ispresizer_config_datapath(enum ispresizer_input input);

void ispresizer_enable_cbilin(u8 enable);

void ispresizer_config_ycpos(u8 yc);

void ispresizer_config_startphase(u8 hstartphase, u8 vstartphase);

void ispresizer_config_filter_coef(struct isprsz_coef *coef);

void ispresizer_get_filter_coef(struct isprsz_coef *coef);

void ispresizer_write_filter_coef(void);

void ispresizer_config_luma_enhance(struct isprsz_yenh *yenh);

int ispresizer_try_size(u32 *input_w, u32 *input_h, u32 *output_w,
								u32 *output_h);

void ispresizer_applycrop(void);

void ispresizer_trycrop(u32 left, u32 top, u32 width, u32 height, u32 ow,
								u32 oh);

int ispresizer_config_size(u32 input_w, u32 input_h, u32 output_w,
								u32 output_h);

int ispresizer_config_inlineoffset(u32 offset);

int ispresizer_set_inaddr(u32 addr);

int ispresizer_config_outlineoffset(u32 offset);

int ispresizer_set_outaddr(u32 addr);

void ispresizer_enable(u8 enable);

int ispresizer_busy(void);

void ispresizer_save_context(void);

void ispresizer_restore_context(void);

void ispresizer_print_status(void);

#endif		/* OMAP_ISP_RESIZER_H */
