/*
 * drivers/media/video/isp/ispresizer.c
 *
 * Driver Library for Resizer module in TI's OMAP3430 Camera ISP
 *
 * Copyright (C)2008 Texas Instruments, Inc.
 *
 * Contributors:
 *	Sameer Venkatraman <sameerv@ti.com>
 *	Mohit Jalori <mjalori@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/io.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/semaphore.h>

#include "isp.h"
#include "ispreg.h"
#include "ispresizer.h"

#if ISP_WORKAROUND
dma_addr_t buff_addr_lsc_wa;
#endif

static u8 need_to_write_filter_coefs = {0};

/* Default configuration of resizer,filter coefficients,yenh for camera isp */
static struct isprsz_yenh ispreszdefaultyenh = {0, 0, 0, 0};
static struct isprsz_coef ispreszdefcoef = {
		{
			0,    256,  0,    0,
			-12, 247, 23,   -2,
			-18, 222, 58,   -6,
			-19, 186, 100, -11,
			-16, 144, 144, -16,
			-11, 100, 186, -19,
			-6,   58,   222, -18,
			-2,   23,   247, -12,
			},
			{
			0,    256,  0,    0,
			-12, 247, 23,   -2,
			-18, 222, 58,   -6,
			-19, 186, 100, -11,
			-16, 144, 144, -16,
			-11, 100, 186, -19,
			-6,   58,   222, -18,
			-2,   23,   247, -12,
		},
		{
			0,	64, 128, 64, 0, 0, 0,
			-9,	39, 124, 92, 13, -3, 0,
			-8,	24, 112, 112, 24, -8, 0,
			-3, 13, 92, 124, 39, -9, 0,
		},
		{
			0,	64, 128, 64, 0, 0, 0,
			-9,	39, 124, 92, 13, -3, 0,
			-8,	24, 112, 112, 24, -8, 0,
			-3, 13, 92, 124, 39, -9, 0,
		}
	};

/**
 * struct isp_res - Structure for the resizer module to store its information.
 * @res_inuse: Indicates if resizer module has been reserved. 1 - Reserved,
 *             0 - Freed.
 * @h_startphase: Horizontal starting phase.
 * @v_startphase: Vertical starting phase.
 * @h_resz: Horizontal resizing value.
 * @v_resz: Vertical resizing value.
 * @outputwidth: Output Image Width in pixels.
 * @outputheight: Output Image Height in pixels.
 * @inputwidth: Input Image Width in pixels.
 * @inputheight: Input Image Height in pixels.
 * @algo: Algorithm select. 0 - Disable, 1 - [-1 2 -1]/2 high-pass filter,
 *        2 - [-1 -2 6 -2 -1]/4 high-pass filter.
 * @ipht_crop: Vertical start line for cropping.
 * @ipwd_crop: Horizontal start pixel for cropping.
 * @cropwidth: Crop Width.
 * @cropheight: Crop Height.
 * @resinput: Resizer input.
 * @coeflist: Register configuration for Resizer.
 * @ispres_mutex: Mutex for isp resizer.
 */
static struct isp_res {
	u8 res_inuse;
	u8 h_startphase;
	u8 v_startphase;
	u16 h_resz;
	u16 v_resz;
	u32 outputwidth;
	u32 outputheight;
	u32 inputwidth;
	u32 inputheight;
	u8 algo;
	u32 ipht_crop;
	u32 ipwd_crop;
	u32 cropwidth;
	u32 cropheight;
	enum ispresizer_input resinput;
	struct isprsz_coef coeflist;
	struct mutex ispres_mutex; /* For checking/modifying res_inuse */
} ispres_obj;

/* Structure for saving/restoring resizer module registers */
static struct isp_reg isprsz_reg_list[] = {
	{ISPRSZ_CNT, 0x0000},
	{ISPRSZ_OUT_SIZE, 0x0000},
	{ISPRSZ_IN_START, 0x0000},
	{ISPRSZ_IN_SIZE, 0x0000},
	{ISPRSZ_SDR_INADD, 0x0000},
	{ISPRSZ_SDR_INOFF, 0x0000},
	{ISPRSZ_SDR_OUTADD, 0x0000},
	{ISPRSZ_SDR_OUTOFF, 0x0000},
	{ISPRSZ_HFILT10, 0x0000},
	{ISPRSZ_HFILT32, 0x0000},
	{ISPRSZ_HFILT54, 0x0000},
	{ISPRSZ_HFILT76, 0x0000},
	{ISPRSZ_HFILT98, 0x0000},
	{ISPRSZ_HFILT1110, 0x0000},
	{ISPRSZ_HFILT1312, 0x0000},
	{ISPRSZ_HFILT1514, 0x0000},
	{ISPRSZ_HFILT1716, 0x0000},
	{ISPRSZ_HFILT1918, 0x0000},
	{ISPRSZ_HFILT2120, 0x0000},
	{ISPRSZ_HFILT2322, 0x0000},
	{ISPRSZ_HFILT2524, 0x0000},
	{ISPRSZ_HFILT2726, 0x0000},
	{ISPRSZ_HFILT2928, 0x0000},
	{ISPRSZ_HFILT3130, 0x0000},
	{ISPRSZ_VFILT10, 0x0000},
	{ISPRSZ_VFILT32, 0x0000},
	{ISPRSZ_VFILT54, 0x0000},
	{ISPRSZ_VFILT76, 0x0000},
	{ISPRSZ_VFILT98, 0x0000},
	{ISPRSZ_VFILT1110, 0x0000},
	{ISPRSZ_VFILT1312, 0x0000},
	{ISPRSZ_VFILT1514, 0x0000},
	{ISPRSZ_VFILT1716, 0x0000},
	{ISPRSZ_VFILT1918, 0x0000},
	{ISPRSZ_VFILT2120, 0x0000},
	{ISPRSZ_VFILT2322, 0x0000},
	{ISPRSZ_VFILT2524, 0x0000},
	{ISPRSZ_VFILT2726, 0x0000},
	{ISPRSZ_VFILT2928, 0x0000},
	{ISPRSZ_VFILT3130, 0x0000},
	{ISPRSZ_YENH, 0x0000},
	{ISP_TOK_TERM, 0x0000}
};

/**
 * ispresizer_config_shadow_registers - Configure shadow registers.
 **/
void ispresizer_config_shadow_registers()
{
	if (need_to_write_filter_coefs)
		ispresizer_write_filter_coef();
}
EXPORT_SYMBOL(ispresizer_config_shadow_registers);

/**
 * ispresizer_trycrop - Validate crop dimensions.
 * @left: Left distance to start position of crop.
 * @top: Top distance to start position of crop.
 * @width: Width of input image.
 * @height: Height of input image.
 * @ow: Width of output image.
 * @oh: Height of output image.
 **/
void ispresizer_trycrop(u32 left, u32 top, u32 width, u32 height, u32 ow,
									u32 oh)
{
	ispres_obj.cropwidth = width + 6;
	ispres_obj.cropheight = height + 6;
	ispresizer_try_size(&ispres_obj.cropwidth, &ispres_obj.cropheight, &ow,
									&oh);
	ispres_obj.ipht_crop = top;
	ispres_obj.ipwd_crop = left;
}
EXPORT_SYMBOL(ispresizer_trycrop);

/**
 * ispresizer_applycrop - Apply crop to input image.
 **/
void ispresizer_applycrop()
{
	ispresizer_config_size(ispres_obj.cropwidth, ispres_obj.cropheight,
						ispres_obj.outputwidth,
						ispres_obj.outputheight);
	return;
}
EXPORT_SYMBOL(ispresizer_applycrop);

/**
 * ispresizer_request - Reserves the Resizer module.
 *
 * Allows only one user at a time.
 *
 * Returns 0 if successful, or -EBUSY if resizer module was already requested.
 **/
int ispresizer_request()
{
	mutex_lock(&ispres_obj.ispres_mutex);
	if (!ispres_obj.res_inuse) {
		ispres_obj.res_inuse = 1;
		omap_writel(omap_readl(ISP_CTRL) | ISPCTRL_SBL_WR0_RAM_EN |
						ISPCTRL_RSZ_CLK_EN, ISP_CTRL);
		mutex_unlock(&ispres_obj.ispres_mutex);
		return 0;
	} else {
		mutex_unlock(&ispres_obj.ispres_mutex);
		printk(KERN_ERR "ISP_ERR : Resizer Module Busy\n");
		return -EBUSY;
	}
}
EXPORT_SYMBOL(ispresizer_request);

/**
 * ispresizer_free - Makes Resizer module free.
 *
 * Returns 0 if successful, or -EINVAL if resizer module was already freed.
 **/
int ispresizer_free()
{
	mutex_lock(&ispres_obj.ispres_mutex);
	if (ispres_obj.res_inuse) {
		ispres_obj.res_inuse = 0;
		omap_writel(omap_readl(ISP_CTRL) & ~(ISPCTRL_RSZ_CLK_EN |
					ISPCTRL_SBL_WR0_RAM_EN), ISP_CTRL);
		mutex_unlock(&ispres_obj.ispres_mutex);
		return 0;
	} else {
		mutex_unlock(&ispres_obj.ispres_mutex);
		DPRINTK_ISPRESZ("ISP_ERR : Resizer Module already freed\n");
		return -EINVAL;
	}
}
EXPORT_SYMBOL(ispresizer_free);

/**
 * ispresizer_config_datapath - Specifies which input to use in resizer module
 * @input: Indicates the module that gives the image to resizer.
 *
 * Sets up the default resizer configuration according to the arguments.
 *
 * Returns 0 if successful, or -EINVAL if an unsupported input was requested.
 **/
int ispresizer_config_datapath(enum ispresizer_input input)
{
	u32 cnt = 0;
	DPRINTK_ISPRESZ("ispresizer_config_datapath()+\n");
	ispres_obj.resinput = input;
	switch (input) {
	case RSZ_OTFLY_YUV:
		cnt &= ~ISPRSZ_CNT_INPTYP;
		cnt &= ~ISPRSZ_CNT_INPSRC;
		ispresizer_set_inaddr(0);
		ispresizer_config_inlineoffset(0);
		break;
	case RSZ_MEM_YUV:
		cnt |= ISPRSZ_CNT_INPSRC;
		cnt &= ~ISPRSZ_CNT_INPTYP;
		break;
	case RSZ_MEM_COL8:
		cnt |= ISPRSZ_CNT_INPSRC;
		cnt |= ISPRSZ_CNT_INPTYP;
		break;
	default:
		printk(KERN_ERR "ISP_ERR : Wrong Input\n");
		return -EINVAL;
	}

	mutex_lock(&ispres_obj.ispres_mutex);
	omap_writel(omap_readl(ISPRSZ_CNT) | cnt, ISPRSZ_CNT);
	mutex_unlock(&ispres_obj.ispres_mutex);

	ispresizer_config_ycpos(0);
	ispresizer_config_filter_coef(&ispreszdefcoef);
	ispresizer_enable_cbilin(0);
	ispresizer_config_luma_enhance(&ispreszdefaultyenh);
	DPRINTK_ISPRESZ("ispresizer_config_datapath()-\n");
	return 0;
}
EXPORT_SYMBOL(ispresizer_config_datapath);

/**
 * ispresizer_try_size - Validates input and output images size.
 * @input_w: input width for the resizer in number of pixels per line
 * @input_h: input height for the resizer in number of lines
 * @output_w: output width from the resizer in number of pixels per line
 *            resizer when writing to memory needs this to be multiple of 16.
 * @output_h: output height for the resizer in number of lines, must be even.
 *
 * Calculates the horizontal and vertical resize ratio, number of pixels to
 * be cropped in the resizer module and checks the validity of various
 * parameters. Formula used for calculation is:-
 *
 * 8-phase 4-tap mode :-
 * inputwidth = (32 * sph + (ow - 1) * hrsz + 16) >> 8 + 7
 * inputheight = (32 * spv + (oh - 1) * vrsz + 16) >> 8 + 4
 * endpahse for width = ((32 * sph + (ow - 1) * hrsz + 16) >> 5) % 8
 * endphase for height = ((32 * sph + (oh - 1) * hrsz + 16) >> 5) % 8
 *
 * 4-phase 7-tap mode :-
 * inputwidth = (64 * sph + (ow - 1) * hrsz + 32) >> 8 + 7
 * inputheight = (64 * spv + (oh - 1) * vrsz + 32) >> 8 + 7
 * endpahse for width = ((64 * sph + (ow - 1) * hrsz + 32) >> 6) % 4
 * endphase for height = ((64 * sph + (oh - 1) * hrsz + 32) >> 6) % 4
 *
 * Where:
 * sph = Start phase horizontal
 * spv = Start phase vertical
 * ow = Output width
 * oh = Output height
 * hrsz = Horizontal resize value
 * vrsz = Vertical resize value
 *
 * Fills up the output/input widht/height, horizontal/vertical resize ratio,
 * horizontal/vertical crop variables in the isp_res structure.
 **/
int ispresizer_try_size(u32 *input_width, u32 *input_height, u32 *output_w,
								u32 *output_h)
{
	u32 rsz, rsz_7, rsz_4;
	u32 sph;
	u32 input_w, input_h;
	int max_in_otf, max_out_7tap;
	input_w = *input_width;
	input_h = *input_height;

	if (input_w < 32 || input_h < 32) {
		DPRINTK_ISPCCDC("ISP_ERR: RESIZER cannot handle input width"
				" less than 32 pixels or height less than"
				" 32\n");
		return -EINVAL;
	}
	input_w -= 6;
	input_h -= 6;

	if (input_h > MAX_IN_HEIGHT)
		return -EINVAL;

	if (*output_w < 16)
		*output_w = 16;

	if (*output_h < 2)
		*output_h = 2;

	if (system_rev == OMAP3430_REV_ES1_0) {
		max_in_otf = MAX_IN_WIDTH_ONTHEFLY_MODE;
		max_out_7tap = MAX_7TAP_VRSZ_OUTWIDTH;
	} else {
		max_in_otf = MAX_IN_WIDTH_ONTHEFLY_MODE_ES2;
		max_out_7tap = MAX_7TAP_VRSZ_OUTWIDTH_ES2;
	}

	if (ispres_obj.resinput == RSZ_OTFLY_YUV) {
		if (input_w > max_in_otf)
			return -EINVAL;
	} else {
		if (input_w > MAX_IN_WIDTH_MEMORY_MODE)
			return -EINVAL;
	}

	*output_h &= 0xfffffffe;
	sph = DEFAULTSTPHASE;

	rsz_7 = ((input_h - 7) * 256) / (*output_h - 1);
	rsz_4 = ((input_h - 4) * 256) / (*output_h - 1);

	rsz = (input_h * 256) / *output_h;

	if (rsz <= MID_RESIZE_VALUE) {
		rsz = rsz_4;
		if (rsz < MINIMUM_RESIZE_VALUE) {
			rsz = MINIMUM_RESIZE_VALUE;
			*output_h = (((input_h - 4) * 256) / rsz) + 1;
			printk(KERN_INFO "%s: using output_h %d instead\n",
			       __func__, *output_h);
		}
	} else {
		rsz = rsz_7;
		if (*output_w > max_out_7tap)
			*output_w = max_out_7tap;
		if (rsz > MAXIMUM_RESIZE_VALUE) {
			rsz = MAXIMUM_RESIZE_VALUE;
			*output_h = (((input_h - 7) * 256) / rsz) + 1;
			printk(KERN_INFO "%s: using output_h %d instead\n",
			       __func__, *output_h);
		}
	}

	if (rsz > MID_RESIZE_VALUE)
		input_h =
			(((64 * sph) + ((*output_h - 1) * rsz) + 32) / 256) + 7;
	else
		input_h =
			(((32 * sph) + ((*output_h - 1) * rsz) + 16) / 256) + 4;

	ispres_obj.outputheight = *output_h;
	ispres_obj.v_resz = rsz;
	ispres_obj.inputheight = input_h;
	ispres_obj.ipht_crop = DEFAULTSTPIXEL;
	ispres_obj.v_startphase = sph;

	*output_w &= 0xfffffff0;
	sph = DEFAULTSTPHASE;

	rsz_7 = ((input_w - 7) * 256) / (*output_w - 1);
	rsz_4 = ((input_w - 4) * 256) / (*output_w - 1);

	rsz = (input_w * 256) / *output_w;
	if (rsz > MID_RESIZE_VALUE) {
		rsz = rsz_7;
		if (rsz > MAXIMUM_RESIZE_VALUE) {
			rsz = MAXIMUM_RESIZE_VALUE;
			*output_w = (((input_w - 7) * 256) / rsz) + 1;
			*output_w = (*output_w + 0xf) & 0xfffffff0;
			printk(KERN_INFO "%s: using output_w %d instead\n",
			       __func__, *output_w);
		}
	} else {
		rsz = rsz_4;
		if (rsz < MINIMUM_RESIZE_VALUE) {
			rsz = MINIMUM_RESIZE_VALUE;
			*output_w = (((input_w - 4) * 256) / rsz) + 1;
			*output_w = (*output_w + 0xf) & 0xfffffff0;
			printk(KERN_INFO "%s: using output_w %d instead\n",
			       __func__, *output_w);
		}
	}

	/* Recalculate input based on TRM equations */
	if (rsz > MID_RESIZE_VALUE)
		input_w =
			(((64 * sph) + ((*output_w - 1) * rsz) + 32) / 256) + 7;
	else
		input_w =
			(((32 * sph) + ((*output_w - 1) * rsz) + 16) / 256) + 7;

	ispres_obj.outputwidth = *output_w;
	ispres_obj.h_resz = rsz;
	ispres_obj.inputwidth = input_w;
	ispres_obj.ipwd_crop = DEFAULTSTPIXEL;
	ispres_obj.h_startphase = sph;

	*input_height = input_h;
	*input_width = input_w;
	return 0;
}
EXPORT_SYMBOL(ispresizer_try_size);

/**
 * ispresizer_config_size - Configures input and output image size.
 * @input_w: input width for the resizer in number of pixels per line.
 * @input_h: input height for the resizer in number of lines.
 * @output_w: output width from the resizer in number of pixels per line.
 * @output_h: output height for the resizer in number of lines.
 *
 * Configures the appropriate values stored in the isp_res structure in the
 * resizer registers.
 *
 * Returns 0 if successful, or -EINVAL if passed values haven't been verified
 * with ispresizer_try_size() previously.
 **/
int ispresizer_config_size(u32 input_w, u32 input_h, u32 output_w,
								u32 output_h)
{
	u32 res;

	DPRINTK_ISPRESZ("ispresizer_config_size()+, input_w = %d,input_h ="
						" %d, output_w = %d, output_h"
						" = %d,hresz = %d,vresz = %d,"
						" hcrop = %d, vcrop = %d,"
						" hstph = %d, vstph = %d\n",
						ispres_obj.inputwidth,
						ispres_obj.inputheight,
						ispres_obj.outputwidth,
						ispres_obj.outputheight,
						ispres_obj.h_resz,
						ispres_obj.v_resz,
						ispres_obj.ipwd_crop,
						ispres_obj.ipht_crop,
						ispres_obj.h_startphase,
						ispres_obj.v_startphase);
	if ((output_w != ispres_obj.outputwidth)
			|| (output_h != ispres_obj.outputheight)) {
		printk(KERN_ERR "Output parameters passed do not match the"
						" values calculated by the"
						" trysize passed w %d, h %d"
						" \n", output_w , output_h);
		return -EINVAL;
	}

#if ISP_WORKAROUND
	buff_addr_lsc_wa = isp_buf_get();
	if (buff_addr_lsc_wa) {
		/* Set Resizer input address and offset adderss */
		ispresizer_set_inaddr(buff_addr_lsc_wa);
		ispresizer_config_inlineoffset(omap_readl(ISPPRV_WADD_OFFSET));
	}
#endif

	/* mutex_lock(&ispres_obj.ispres_mutex); */

	res = omap_readl(ISPRSZ_CNT) & (~(ISPRSZ_CNT_HSTPH_MASK |
					ISPRSZ_CNT_VSTPH_MASK));
	omap_writel(res | (ispres_obj.h_startphase << ISPRSZ_CNT_HSTPH_SHIFT) |
						(ispres_obj.v_startphase <<
						ISPRSZ_CNT_VSTPH_SHIFT),
						ISPRSZ_CNT);

#if ISP_WORKAROUND
	omap_writel((0x00 <<	ISPRSZ_IN_START_HORZ_ST_SHIFT) |
					(0x00 <<
					ISPRSZ_IN_START_VERT_ST_SHIFT),
					ISPRSZ_IN_START);

#else
	omap_writel(((ispres_obj.ipwd_crop * 2) <<
					ISPRSZ_IN_START_HORZ_ST_SHIFT) |
					(ispres_obj.ipht_crop <<
					ISPRSZ_IN_START_VERT_ST_SHIFT),
					ISPRSZ_IN_START);
#endif

	omap_writel((ispres_obj.inputwidth << ISPRSZ_IN_SIZE_HORZ_SHIFT) |
						(ispres_obj.inputheight <<
						ISPRSZ_IN_SIZE_VERT_SHIFT),
						ISPRSZ_IN_SIZE);
	if (!ispres_obj.algo) {
		omap_writel((output_w << ISPRSZ_OUT_SIZE_HORZ_SHIFT) |
						(output_h <<
						ISPRSZ_OUT_SIZE_VERT_SHIFT),
						ISPRSZ_OUT_SIZE);
	} else {
		omap_writel(((output_w - 4) << ISPRSZ_OUT_SIZE_HORZ_SHIFT) |
						(output_h <<
						ISPRSZ_OUT_SIZE_VERT_SHIFT),
						ISPRSZ_OUT_SIZE);
	}

	res = omap_readl(ISPRSZ_CNT) & (~(ISPRSZ_CNT_HRSZ_MASK |
						ISPRSZ_CNT_VRSZ_MASK));
	omap_writel(res | ((ispres_obj.h_resz - 1) << ISPRSZ_CNT_HRSZ_SHIFT) |
						((ispres_obj.v_resz - 1) <<
						ISPRSZ_CNT_VRSZ_SHIFT),
						ISPRSZ_CNT);

	/* mutex_unlock(&ispres_obj.ispres_mutex); */

    ispresizer_write_filter_coef();

    ispresizer_config_outlineoffset(output_w*2);
    DPRINTK_ISPRESZ("ispresizer_config_size()-\n");
    return 0;
}
EXPORT_SYMBOL(ispresizer_config_size);

/**
 * ispresizer_enable - Enables the resizer module.
 * @enable: 1 - Enable, 0 - Disable
 *
 * Client should configure all the sub modules in resizer before this.
 **/
void ispresizer_enable(u8 enable)
{
	int val;
	DPRINTK_ISPRESZ("+ispresizer_enable()+\n");
	if (enable)
		val = (((omap_readl(ISPRSZ_PCR)) & 0x2) | ISPRSZ_PCR_ENABLE);
	else
		val = (omap_readl(ISPRSZ_PCR) & ~ISPRSZ_PCR_ENABLE);
	omap_writel(val, ISPRSZ_PCR);
	DPRINTK_ISPRESZ("+ispresizer_enable()-\n");
}
EXPORT_SYMBOL(ispresizer_enable);

/**
 * ispresizer_busy - Checks if ISP resizer is busy.
 *
 * Returns busy field from ISPRSZ_PCR register.
 **/
int ispresizer_busy(void)
{
	return omap_readl(ISPRSZ_PCR) & ISPPRV_PCR_BUSY;
}
EXPORT_SYMBOL(ispresizer_busy);

/**
 * ispresizer_config_startphase - Sets the horizontal and vertical start phase.
 * @hstartphase: horizontal start phase (0 - 7).
 * @vstartphase: vertical startphase (0 - 7).
 *
 * This API just updates the isp_res struct. Actual register write happens in
 * ispresizer_config_size.
 **/
void ispresizer_config_startphase(u8 hstartphase, u8 vstartphase)
{
	DPRINTK_ISPRESZ("ispresizer_config_startphase()+\n");
	ispres_obj.h_startphase = hstartphase;
	ispres_obj.v_startphase = vstartphase;
	DPRINTK_ISPRESZ("ispresizer_config_startphase()-\n");
}
EXPORT_SYMBOL(ispresizer_config_startphase);

/**
 * ispresizer_config_ycpos - Specifies if output should be in YC or CY format.
 * @yc: 0 - YC format, 1 - CY format
 **/
void ispresizer_config_ycpos(u8 yc)
{
	DPRINTK_ISPRESZ("ispresizer_config_ycpos()+\n");
	if (yc)
		omap_writel((omap_readl(ISPRSZ_CNT)) |
			(ISPRSZ_CNT_YCPOS), ISPRSZ_CNT);
	else
		omap_writel((omap_readl(ISPRSZ_CNT)) &
			(~ISPRSZ_CNT_YCPOS), ISPRSZ_CNT);
	DPRINTK_ISPRESZ("ispresizer_config_ycpos()-\n");
}
EXPORT_SYMBOL(ispresizer_config_ycpos);

/**
 * Sets the chrominance algorithm
 * @cbilin: 0 - chrominance uses same processing as luminance,
 *          1 - bilinear interpolation processing
 **/
void ispresizer_enable_cbilin(u8 enable)
{
	mutex_lock(&ispres_obj.ispres_mutex);
	DPRINTK_ISPRESZ("ispresizer_enable_cbilin()+\n");
	if (enable) {
		omap_writel(omap_readl(ISPRSZ_CNT) | ISPRSZ_CNT_CBILIN,
								ISPRSZ_CNT);
	} else {
		omap_writel(omap_readl(ISPRSZ_CNT) & ~ISPRSZ_CNT_CBILIN,
								ISPRSZ_CNT);
	}
	DPRINTK_ISPRESZ("ispresizer_enable_cbilin()-\n");
	mutex_unlock(&ispres_obj.ispres_mutex);
}
EXPORT_SYMBOL(ispresizer_enable_cbilin);

/**
 * ispresizer_config_luma_enhance - Configures luminance enhancer parameters.
 * @yenh: Pointer to structure containing desired values for core, slope, gain
 *        and algo parameters.
 **/
void ispresizer_config_luma_enhance(struct isprsz_yenh *yenh)
{
	DPRINTK_ISPRESZ("ispresizer_config_luma_enhance()+\n");
	ispres_obj.algo = yenh->algo;
	omap_writel((yenh->algo << ISPRSZ_YENH_ALGO_SHIFT) |
			(yenh->gain << ISPRSZ_YENH_GAIN_SHIFT) |
			(yenh->slope << ISPRSZ_YENH_SLOP_SHIFT) |
			(yenh->coreoffset << ISPRSZ_YENH_CORE_SHIFT),
			ISPRSZ_YENH);
	DPRINTK_ISPRESZ("ispresizer_config_luma_enhance()-\n");
}
EXPORT_SYMBOL(ispresizer_config_luma_enhance);

/**
 * ispresizer_config_filter_coef - Sets filter coefficients for 4 & 7-tap mode.
 * This API just updates the isp_res struct.Actual register write happens in
 * ispresizer_config_size.
 * @coef: Structure containing horizontal and vertical filter coefficients for
 *        both 4-tap and 7-tap mode.
 **/
void ispresizer_config_filter_coef(struct isprsz_coef *coef)
{
	int i;
	DPRINTK_ISPRESZ("ispresizer_config_filter_coef()+\n");

	need_to_write_filter_coefs = 0;

	for (i = 0; i < 32; i++) {
		ispres_obj.coeflist.h_filter_coef_4tap[i] =
						coef->h_filter_coef_4tap[i];
		ispres_obj.coeflist.v_filter_coef_4tap[i] =
						coef->v_filter_coef_4tap[i];
	}
	for (i = 0; i < 28; i++) {
		ispres_obj.coeflist.h_filter_coef_7tap[i] =
						coef->h_filter_coef_7tap[i];
		ispres_obj.coeflist.v_filter_coef_7tap[i] =
						coef->v_filter_coef_7tap[i];
	}

	need_to_write_filter_coefs = 1;

	DPRINTK_ISPRESZ("ispresizer_config_filter_coef()-\n");
}
EXPORT_SYMBOL(ispresizer_config_filter_coef);

/**
* ispresizer_get_filter_coef - Get the resizer filter coefficients.
* @coef: Structure containing horizontal and vertical filter coefficients for
*        both 4-tap and 7-tap mode.
**/
void ispresizer_get_filter_coef(struct isprsz_coef *coef)
{
	mutex_lock(&ispres_obj.ispres_mutex);
	memcpy(coef, &ispres_obj.coeflist, sizeof(struct isprsz_coef));
	mutex_unlock(&ispres_obj.ispres_mutex);
}
EXPORT_SYMBOL(ispresizer_get_filter_coef);

/**
* ispresizer_set_filter_coef - Writes the resizer filter coefficients to the
* resizer registers.
**/
void ispresizer_write_filter_coef(void)
{
	int i, j;
	DPRINTK_ISPRESZ("ispresizer_write_filter_coef\n");

	if (ispresizer_busy() == 0) {
		if (ispres_obj.h_resz <= MID_RESIZE_VALUE) {
			j = 0;
			for (i = 0; i < 16; i++) {
				omap_writel((ispres_obj.coeflist.
						h_filter_coef_4tap[j] <<
						ISPRSZ_HFILT10_COEF0_SHIFT) |
						(ispres_obj.coeflist.
						h_filter_coef_4tap[j + 1] <<
						ISPRSZ_HFILT10_COEF1_SHIFT),
						ISPRSZ_HFILT10 + (i * 0x04));
				j += 2;
			}
		} else {
			j = 0;
			for (i = 0; i < 16; i++) {
				if ((i + 1) % 4 == 0) {
					omap_writel((ispres_obj.coeflist.
							h_filter_coef_7tap[j] <<
							ISPRSZ_HFILT10_COEF0_SHIFT),
							ISPRSZ_HFILT10 + (i * 0x04));
					j += 1;
				} else {
					omap_writel((ispres_obj.coeflist.
							h_filter_coef_7tap[j] <<
							ISPRSZ_HFILT10_COEF0_SHIFT) |
							(ispres_obj.coeflist.
							h_filter_coef_7tap[j+1] <<
							ISPRSZ_HFILT10_COEF1_SHIFT),
							ISPRSZ_HFILT10 + (i * 0x04));
					j += 2;
				}
			}
		}
		if (ispres_obj.v_resz <= MID_RESIZE_VALUE) {
			j = 0;
			for (i = 0; i < 16; i++) {
				omap_writel((ispres_obj.coeflist.
						v_filter_coef_4tap[j] <<
						ISPRSZ_VFILT10_COEF0_SHIFT) |
						(ispres_obj.coeflist.
						v_filter_coef_4tap[j + 1] <<
						ISPRSZ_VFILT10_COEF1_SHIFT),
						ISPRSZ_VFILT10 + (i * 0x04));
				j += 2;
			}
		} else {
			j = 0;
			for (i = 0; i < 16; i++) {
				if ((i + 1) % 4 == 0) {
					omap_writel((ispres_obj.coeflist.
							v_filter_coef_7tap[j] <<
							ISPRSZ_VFILT10_COEF0_SHIFT),
							ISPRSZ_VFILT10 + (i * 0x04));
					j += 1;
				} else {
					omap_writel((ispres_obj.coeflist.
							v_filter_coef_7tap[j] <<
							ISPRSZ_VFILT10_COEF0_SHIFT) |
							(ispres_obj.coeflist.
							v_filter_coef_7tap[j+1] <<
							ISPRSZ_VFILT10_COEF1_SHIFT),
							ISPRSZ_VFILT10 + (i * 0x04));
					j += 2;
				}
			}
		}

		need_to_write_filter_coefs = 0;
	} else {
		DPRINTK_ISPRESZ("Resizer busy when programming filter coefs\n");
	}
}
EXPORT_SYMBOL(ispresizer_write_filter_coef);

/**
 * ispresizer_config_inlineoffset - Configures the read address line offset.
 * @offset: Line Offset for the input image.
 *
 * Returns 0 if successful, or -EINVAL if offset is not 32 bits aligned.
 **/
int ispresizer_config_inlineoffset(u32 offset)
{
	DPRINTK_ISPRESZ("ispresizer_config_inlineoffset()+\n");
	if (offset%32)
		return -EINVAL;
	omap_writel(offset << ISPRSZ_SDR_INOFF_OFFSET_SHIFT, ISPRSZ_SDR_INOFF);
	DPRINTK_ISPRESZ("ispresizer_config_inlineoffset()-\n");
	return 0;
}
EXPORT_SYMBOL(ispresizer_config_inlineoffset);

/**
 * ispresizer_set_inaddr - Sets the memory address of the input frame.
 * @addr: 32bit memory address aligned on 32byte boundary.
 *
 * Returns 0 if successful, or -EINVAL if address is not 32 bits aligned.
 **/
int ispresizer_set_inaddr(u32 addr)
{
	DPRINTK_ISPRESZ("ispresizer_set_inaddr()+\n");
	if (addr%32)
		return -EINVAL;
	omap_writel(addr << ISPRSZ_SDR_INADD_ADDR_SHIFT, ISPRSZ_SDR_INADD);
	DPRINTK_ISPRESZ("ispresizer_set_inaddr()-\n");
	return 0;
}
EXPORT_SYMBOL(ispresizer_set_inaddr);

/**
 * ispresizer_config_outlineoffset - Configures the write address line offset.
 * @offset: Line offset for the preview output.
 *
 * Returns 0 if successful, or -EINVAL if address is not 32 bits aligned.
 **/
int ispresizer_config_outlineoffset(u32 offset)
{
	DPRINTK_ISPRESZ("ispresizer_config_outlineoffset()+\n");
	if (offset%32)
		return -EINVAL;
	omap_writel(offset << ISPRSZ_SDR_OUTOFF_OFFSET_SHIFT,
							ISPRSZ_SDR_OUTOFF);
	DPRINTK_ISPRESZ("ispresizer_config_outlineoffset()-\n");
	return 0;
}
EXPORT_SYMBOL(ispresizer_config_outlineoffset);

/**
 * Configures the memory address to which the output frame is written.
 * @addr: 32bit memory address aligned on 32byte boundary.
 **/
int ispresizer_set_outaddr(u32 addr)
{
	DPRINTK_ISPRESZ("ispresizer_set_outaddr()+\n");
	if (addr%32)
		return -EINVAL;
	omap_writel(addr << ISPRSZ_SDR_OUTADD_ADDR_SHIFT, ISPRSZ_SDR_OUTADD);

	DPRINTK_ISPRESZ("ispresizer_set_outaddr()-\n");
	return 0;
}
EXPORT_SYMBOL(ispresizer_set_outaddr);

/**
 * ispresizer_save_context - Saves the values of the resizer module registers.
 **/
void ispresizer_save_context(void)
{
	DPRINTK_ISPRESZ("Saving context\n");
	isp_save_context(isprsz_reg_list);
}
EXPORT_SYMBOL(ispresizer_save_context);

/**
 * ispresizer_restore_context - Restores resizer module register values.
 **/
void ispresizer_restore_context(void)
{
	DPRINTK_ISPRESZ("Restoring context\n");
	isp_restore_context(isprsz_reg_list);
}
EXPORT_SYMBOL(ispresizer_restore_context);

/**
 * ispresizer_print_status - Prints the values of the resizer module registers.
 **/
void ispresizer_print_status()
{
	if (!is_ispresz_debug_enabled())
		return;
	DPRINTK_ISPRESZ("###ISP_CTRL inresizer =0x%x\n", omap_readl(ISP_CTRL));

	DPRINTK_ISPRESZ("###ISP_IRQ0ENABLE in resizer =0x%x\n",
						omap_readl(ISP_IRQ0ENABLE));
	DPRINTK_ISPRESZ("###ISP_IRQ0STATUS in resizer =0x%x\n",
						omap_readl(ISP_IRQ0STATUS));
	DPRINTK_ISPRESZ("###RSZ PCR =0x%x\n", omap_readl(ISPRSZ_PCR));
	DPRINTK_ISPRESZ("###RSZ CNT =0x%x\n", omap_readl(ISPRSZ_CNT));
	DPRINTK_ISPRESZ("###RSZ OUT SIZE =0x%x\n",
						omap_readl(ISPRSZ_OUT_SIZE));
	DPRINTK_ISPRESZ("###RSZ IN START =0x%x\n",
						omap_readl(ISPRSZ_IN_START));
	DPRINTK_ISPRESZ("###RSZ IN SIZE =0x%x\n", omap_readl(ISPRSZ_IN_SIZE));
	DPRINTK_ISPRESZ("###RSZ SDR INADD =0x%x\n",
						omap_readl(ISPRSZ_SDR_INADD));
	DPRINTK_ISPRESZ("###RSZ SDR INOFF =0x%x\n",
						omap_readl(ISPRSZ_SDR_INOFF));
	DPRINTK_ISPRESZ("###RSZ SDR OUTADD =0x%x\n",
						omap_readl(ISPRSZ_SDR_OUTADD));
	DPRINTK_ISPRESZ("###RSZ SDR OTOFF =0x%x\n",
						omap_readl(ISPRSZ_SDR_OUTOFF));
	DPRINTK_ISPRESZ("###RSZ YENH =0x%x\n", omap_readl(ISPRSZ_YENH));
}
EXPORT_SYMBOL(ispresizer_print_status);

/**
 * isp_resizer_init - Module Initialisation.
 *
 * Always returns 0.
 **/
int __init isp_resizer_init(void)
{
	mutex_init(&ispres_obj.ispres_mutex);

	return 0;
}

/**
 * isp_resizer_cleanup - Module Cleanup.
 **/
void __exit isp_resizer_cleanup(void)
{
}
