/*
 * drivers/media/video/isp/omap_resizer.c
 *
 * Wrapper for Resizer module in TI's OMAP3430 ISP
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Contributors:
 * 	Sergio Aguirre <saaguirre@ti.com>
 * 	Troy Laramy <t-laramy@ti.com>
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
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <media/v4l2-dev.h>
#include <asm/cacheflush.h>

#include "isp.h"
#include "ispmmu.h"
#include "ispreg.h"
#include "ispresizer.h"
#include <linux/omap_resizer.h>

#define OMAP_REZR_NAME		"omap-resizer"

/* Defines and Constants*/
#define MAX_CHANNELS		16
#define MAX_IMAGE_WIDTH		2047
#define MAX_IMAGE_WIDTH_HIGH	2047
#define ALIGNMENT		16
#define CHANNEL_BUSY		1
#define CHANNEL_FREE		0
#define PIXEL_EVEN		2
#define RATIO_MULTIPLIER	256
/* Bit position Macro */
/* macro for bit set and clear */
#define BITSET(variable, bit)	((variable) | (1 << bit))
#define BITRESET(variable, bit)	((variable) & ~(0x00000001 << (bit)))
#define SET_BIT_INPUTRAM	28
#define SET_BIT_CBLIN		29
#define SET_BIT_INPTYP		27
#define SET_BIT_YCPOS		26
#define INPUT_RAM		1
#define UP_RSZ_RATIO		64
#define DOWN_RSZ_RATIO		512
#define UP_RSZ_RATIO1		513
#define DOWN_RSZ_RATIO1		1024
#define RSZ_IN_SIZE_VERT_SHIFT	16
#define MAX_HORZ_PIXEL_8BIT	31
#define MAX_HORZ_PIXEL_16BIT	15
#define NUM_PHASES		8
#define NUM_TAPS		4
#define NUM_D2PH		4	/* for downsampling * 2+x ~ 4x,
					 * number of phases
					 */
#define NUM_D2TAPS		7 	/* for downsampling * 2+x ~ 4x,
					 * number of taps
					 */
#define ALIGN32			32
#define MAX_COEF_COUNTER	16
#define COEFF_ADDRESS_OFFSET	0x04

static DECLARE_MUTEX(resz_wrapper_mutex);

/* Global structure which contains information about number of channels
   and protection variables */
struct device_params {

	unsigned char opened;			/* state of the device */
	struct completion compl_isr;		/* Completion for interrupt */
	struct mutex reszwrap_mutex;		/* Semaphore for array */

	struct videobuf_queue_ops vbq_ops;	/* videobuf queue operations */
};

/* Register mapped structure which contains the every register
   information */
struct resizer_config {
	u32 rsz_pcr;				/* pcr register mapping
						 * variable.
						 */
	u32 rsz_in_start;			/* in_start register mapping
						 * variable.
						 */
	u32 rsz_in_size;			/* in_size register mapping
						 * variable.
						 */
	u32 rsz_out_size;			/* out_size register mapping
						 * variable.
						 */
	u32 rsz_cnt;				/* rsz_cnt register mapping
						 * variable.
						 */
	u32 rsz_sdr_inadd;			/* sdr_inadd register mapping
						 * variable.
						 */
	u32 rsz_sdr_inoff;			/* sdr_inoff register mapping
						 * variable.
						 */
	u32 rsz_sdr_outadd;			/* sdr_outadd register mapping
						 * variable.
						 */
	u32 rsz_sdr_outoff;			/* sdr_outbuff register
						 * mapping variable.
						 */
	u32 rsz_coeff_horz[16];			/* horizontal coefficients
						 * mapping array.
						 */
	u32 rsz_coeff_vert[16];			/* vertical coefficients
						 * mapping array.
						 */
	u32 rsz_yehn;				/* yehn(luma)register mapping
						 * variable.
						 */
};

struct rsz_mult {
	int in_hsize;				/* input frame horizontal
						 * size.
						 */
	int in_vsize;				/* input frame vertical size.
						 */
	int out_hsize;				/* output frame horizontal
						 * size.
						 */
	int out_vsize;				/* output frame vertical
						 * size.
						 */
	int in_pitch;				/* offset between two rows of
						 * input frame.
						 */
	int out_pitch;				/* offset between two rows of
						 * output frame.
						 */
	int end_hsize;
	int end_vsize;
	int num_htap;				/* 0 = 7tap; 1 = 4tap */
	int num_vtap;				/* 0 = 7tap; 1 = 4tap */
	int active;
	int inptyp;
	int vrsz;
	int hrsz;
	int hstph;				/* for specifying horizontal
						 * starting phase.
						 */
	int vstph;
	int pix_fmt;				/* # defined, UYVY or YUYV. */
	int cbilin;				/* # defined, filter with luma
						 * or bi-linear.
						 */
	u16 tap4filt_coeffs[32];		/* horizontal filter
						 * coefficients.
						 */
	u16 tap7filt_coeffs[32];		/* vertical filter
						 * coefficients.
						 */
};
/* Channel specific structure contains information regarding
   the every channel */
struct channel_config {
	struct resizer_config register_config;	/* Instance of register set
						 * mapping structure
						 */
	int status;				/* Specifies whether the
						 * channel is busy or not
						 */
	struct mutex chanprotection_mutex;
	enum config_done config_state;
	u8 input_buf_index;
	u8 output_buf_index;

};

/* per-filehandle data structure */
struct rsz_fh {
	struct rsz_params *params;
	struct channel_config *config;
	struct rsz_mult *multipass;		/* Multipass to support
						 * resizing ration outside
						 * of 0.25x to 4x
						 */
	spinlock_t vbq_lock;			/* spinlock for videobuf
						 * queues.
						 */
	enum v4l2_buf_type type;
	struct videobuf_queue vbq;
	struct device_params *device;

	dma_addr_t isp_addr_read;		/* Input/Output address */
	dma_addr_t isp_addr_write;		/* Input/Output address */
	u32 rsz_bufsize;			/* channel specific buffersize
						 */
};

static struct device_params *device_config;
static struct device *rsz_device;
static int rsz_major = -1;
/* functions declaration */
static void rsz_hardware_setup(struct channel_config *rsz_conf_chan);
static int rsz_set_params(struct rsz_mult *multipass, struct rsz_params *,
						struct channel_config *);
static int rsz_get_params(struct rsz_params *, struct channel_config *);
static void rsz_copy_data(struct rsz_mult *multipass,
						struct rsz_params *params);
static void rsz_isr(unsigned long status, isp_vbq_callback_ptr arg1,
						void *arg2);
static void rsz_calculate_crop(struct channel_config *rsz_conf_chan,
					struct rsz_cropsize *cropsize);
static int rsz_set_multipass(struct rsz_mult *multipass,
					struct channel_config *rsz_conf_chan);
static int rsz_set_ratio(struct rsz_mult *multipass,
					struct channel_config *rsz_conf_chan);
static void rsz_config_ratio(struct rsz_mult *multipass,
					struct channel_config *rsz_conf_chan);

/**
 * rsz_hardware_setup - Sets hardware configuration registers
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Set hardware configuration registers
 **/
static void rsz_hardware_setup(struct channel_config *rsz_conf_chan)
{
	int coeffcounter;
	int coeffoffset = 0;

	down(&resz_wrapper_mutex);
	omap_writel(rsz_conf_chan->register_config.rsz_cnt, ISPRSZ_CNT);

	omap_writel(rsz_conf_chan->register_config.rsz_in_start,
							ISPRSZ_IN_START);
	omap_writel(rsz_conf_chan->register_config.rsz_in_size,
							ISPRSZ_IN_SIZE);

	omap_writel(rsz_conf_chan->register_config.rsz_out_size,
							ISPRSZ_OUT_SIZE);
	omap_writel(rsz_conf_chan->register_config.rsz_sdr_inadd,
							ISPRSZ_SDR_INADD);
	omap_writel(rsz_conf_chan->register_config.rsz_sdr_inoff,
							ISPRSZ_SDR_INOFF);
	omap_writel(rsz_conf_chan->register_config.rsz_sdr_outadd,
							ISPRSZ_SDR_OUTADD);
	omap_writel(rsz_conf_chan->register_config.rsz_sdr_outoff,
							ISPRSZ_SDR_OUTOFF);
	omap_writel(rsz_conf_chan->register_config.rsz_yehn, ISPRSZ_YENH);

	for (coeffcounter = 0; coeffcounter < MAX_COEF_COUNTER;
							coeffcounter++) {
		omap_writel(rsz_conf_chan->register_config.
						rsz_coeff_horz[coeffcounter],
						ISPRSZ_HFILT10 + coeffoffset);

		omap_writel(rsz_conf_chan->register_config.
						rsz_coeff_vert[coeffcounter],
						ISPRSZ_VFILT10 + coeffoffset);
		coeffoffset = coeffoffset + COEFF_ADDRESS_OFFSET;
	}
	up(&resz_wrapper_mutex);
}

/**
 * rsz_start - Enables Resizer Wrapper
 * @arg: Currently not used.
 * @device: Structure containing ISP resizer wrapper global information
 *
 * Submits a resizing task specified by the rsz_resize structure. The call can
 * either be blocked until the task is completed or returned immediately based
 * on the value of the blocking argument in the rsz_resize structure. If it is
 * blocking, the status of the task can be checked by calling ioctl
 * RSZ_G_STATUS. Only one task can be outstanding for each logical channel.
 *
 * Returns 0 if successful, or -EINVAL if could not set callback for RSZR IRQ
 * event or the state of the channel is not configured.
 **/
int rsz_start(int *arg, struct rsz_fh *fh)
{
	struct channel_config *rsz_conf_chan = fh->config;
	struct rsz_mult *multipass = fh->multipass;
	struct videobuf_queue *q = &fh->vbq;
	int ret;

	if (rsz_conf_chan->config_state) {
		dev_err(rsz_device, "State not configured \n");
		goto err_einval;
	}

	rsz_conf_chan->status = CHANNEL_BUSY;

	rsz_hardware_setup(rsz_conf_chan);

	if (isp_set_callback(CBK_RESZ_DONE, rsz_isr, (void *) NULL,
							(void *)NULL)) {
		dev_err(rsz_device, "No callback for RSZR\n");
		goto err_einval;
	}
mult:
	device_config->compl_isr.done = 0;

	ispresizer_enable(1);

	ret = wait_for_completion_interruptible(&device_config->compl_isr);
	if (ret != 0) {
		dev_dbg(rsz_device, "Unexpected exit from "
				"wait_for_completion_interruptible\n");
		wait_for_completion(&device_config->compl_isr);
	}

	if (multipass->active) {
		rsz_set_multipass(multipass, rsz_conf_chan);
		goto mult;
	}

	if (fh->isp_addr_read) {
		ispmmu_unmap(fh->isp_addr_read);
		fh->isp_addr_read = 0;
	}
	if (fh->isp_addr_write) {
		ispmmu_unmap(fh->isp_addr_write);
		fh->isp_addr_write = 0;
	}

	rsz_conf_chan->status = CHANNEL_FREE;
	q->bufs[rsz_conf_chan->input_buf_index]->state = VIDEOBUF_NEEDS_INIT;
	q->bufs[rsz_conf_chan->output_buf_index]->state = VIDEOBUF_NEEDS_INIT;
	rsz_conf_chan->register_config.rsz_sdr_outadd = 0;
	rsz_conf_chan->register_config.rsz_sdr_inadd = 0;

	/* Unmap and free the DMA memory allocated for buffers */
	videobuf_dma_unmap(q, videobuf_to_dma(
				q->bufs[rsz_conf_chan->input_buf_index]));
	videobuf_dma_unmap(q, videobuf_to_dma(
				q->bufs[rsz_conf_chan->output_buf_index]));
	videobuf_dma_free(videobuf_to_dma(
				q->bufs[rsz_conf_chan->input_buf_index]));
	videobuf_dma_free(videobuf_to_dma(
				q->bufs[rsz_conf_chan->output_buf_index]));

	isp_unset_callback(CBK_RESZ_DONE);

	return 0;
err_einval:
	return -EINVAL;
}

/**
 * rsz_set_multipass - Set resizer multipass
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Returns always 0
 **/
static int rsz_set_multipass(struct rsz_mult *multipass,
			struct channel_config *rsz_conf_chan)
{
	multipass->in_hsize = multipass->out_hsize;
	multipass->in_vsize = multipass->out_vsize;
	multipass->out_hsize = multipass->end_hsize;
	multipass->out_vsize = multipass->end_vsize;

	multipass->out_pitch = (multipass->inptyp ? multipass->out_hsize
						: (multipass->out_hsize * 2));
	multipass->in_pitch = (multipass->inptyp ? multipass->in_hsize
						: (multipass->in_hsize * 2));

	rsz_set_ratio(multipass, rsz_conf_chan);
	rsz_config_ratio(multipass, rsz_conf_chan);
	rsz_hardware_setup(rsz_conf_chan);
	return 0;
}

/**
 * rsz_copy_data - Copy data
 * @params: Structure containing the Resizer Wrapper parameters
 *
 * Copy data
 **/
static void rsz_copy_data(struct rsz_mult *multipass, struct rsz_params *params)
{
	int i;
	multipass->in_hsize = params->in_hsize;
	multipass->in_vsize = params->in_vsize;
	multipass->out_hsize = params->out_hsize;
	multipass->out_vsize = params->out_vsize;
	multipass->end_hsize = params->out_hsize;
	multipass->end_vsize = params->out_vsize;
	multipass->in_pitch = params->in_pitch;
	multipass->out_pitch = params->out_pitch;
	multipass->hstph = params->hstph;
	multipass->vstph = params->vstph;
	multipass->inptyp = params->inptyp;
	multipass->pix_fmt = params->pix_fmt;
	multipass->cbilin = params->cbilin;

	for (i = 0; i < 32; i++) {
		multipass->tap4filt_coeffs[i] = params->tap4filt_coeffs[i];
		multipass->tap7filt_coeffs[i] = params->tap7filt_coeffs[i];
	}
}

/**
 * rsz_set_params - Set parameters for resizer wrapper
 * @params: Structure containing the Resizer Wrapper parameters
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Used to set the parameters of the Resizer hardware, including input and
 * output image size, horizontal and vertical poly-phase filter coefficients,
 * luma enchancement filter coefficients, etc.
 **/
static int rsz_set_params(struct rsz_mult *multipass, struct rsz_params *params,
					struct channel_config *rsz_conf_chan)
{
	int mul = 1;
	if ((params->yenh_params.type < 0) || (params->yenh_params.type > 2)) {
		dev_err(rsz_device, "rsz_set_params: Wrong yenh type\n");
		return -EINVAL;
	}
	if ((params->in_vsize <= 0) || (params->in_hsize <= 0) ||
			(params->out_vsize <= 0) || (params->out_hsize <= 0) ||
			(params->in_pitch <= 0) || (params->out_pitch <= 0)) {
		dev_err(rsz_device, "rsz_set_params: Invalid size params\n");
		return -EINVAL;
	}
	if ((params->inptyp != RSZ_INTYPE_YCBCR422_16BIT) &&
			(params->inptyp != RSZ_INTYPE_PLANAR_8BIT)) {
		dev_err(rsz_device, "rsz_set_params: Invalid input type\n");
		return -EINVAL;
	}
	if ((params->pix_fmt != RSZ_PIX_FMT_UYVY) &&
			(params->pix_fmt != RSZ_PIX_FMT_YUYV)) {
		dev_err(rsz_device, "rsz_set_params: Invalid pixel format\n");
		return -EINVAL;
	}
	if (params->inptyp == RSZ_INTYPE_YCBCR422_16BIT)
		mul = 2;
	else
		mul = 1;
	if (params->in_pitch < (params->in_hsize * mul)) {
		dev_err(rsz_device, "rsz_set_params: Pitch is incorrect\n");
		return -EINVAL;
	}
	if (params->out_pitch < (params->out_hsize * mul)) {
		dev_err(rsz_device, "rsz_set_params: Out pitch cannot be less"
					" than out hsize\n");
		return -EINVAL;
	}
	/* Output H size should be even */
	if ((params->out_hsize % PIXEL_EVEN) != 0) {
		dev_err(rsz_device, "rsz_set_params: Output H size should"
					" be even\n");
		return -EINVAL;
	}
	if (params->horz_starting_pixel < 0) {
		dev_err(rsz_device, "rsz_set_params: Horz start pixel cannot"
					" be less than zero\n");
		return -EINVAL;
	}

	rsz_copy_data(multipass, params);
	if (0 != rsz_set_ratio(multipass, rsz_conf_chan))
		goto err_einval;

	if (params->yenh_params.type) {
		if ((multipass->num_htap && multipass->out_hsize >
				1280) ||
				(!multipass->num_htap && multipass->out_hsize >
				640))
			goto err_einval;
	}

	if (INPUT_RAM)
		params->vert_starting_pixel = 0;

	rsz_conf_chan->register_config.rsz_in_start =
						(params->vert_starting_pixel
						<< ISPRSZ_IN_SIZE_VERT_SHIFT)
						& ISPRSZ_IN_SIZE_VERT_MASK;

	if (params->inptyp == RSZ_INTYPE_PLANAR_8BIT) {
		if (params->horz_starting_pixel > MAX_HORZ_PIXEL_8BIT)
			goto err_einval;
	}
	if (params->inptyp == RSZ_INTYPE_YCBCR422_16BIT) {
		if (params->horz_starting_pixel > MAX_HORZ_PIXEL_16BIT)
			goto err_einval;
	}

	rsz_conf_chan->register_config.rsz_in_start |=
						params->horz_starting_pixel
						& ISPRSZ_IN_START_HORZ_ST_MASK;

	rsz_conf_chan->register_config.rsz_yehn =
						(params->yenh_params.type
						<< ISPRSZ_YENH_ALGO_SHIFT)
						& ISPRSZ_YENH_ALGO_MASK;

	if (params->yenh_params.type) {
		rsz_conf_chan->register_config.rsz_yehn |=
						params->yenh_params.core
						& ISPRSZ_YENH_CORE_MASK;

		rsz_conf_chan->register_config.rsz_yehn |=
						(params->yenh_params.gain
						<< ISPRSZ_YENH_GAIN_SHIFT)
						& ISPRSZ_YENH_GAIN_MASK;

		rsz_conf_chan->register_config.rsz_yehn |=
						(params->yenh_params.slop
						<< ISPRSZ_YENH_SLOP_SHIFT)
						& ISPRSZ_YENH_SLOP_MASK;
	}

	rsz_config_ratio(multipass, rsz_conf_chan);

	rsz_conf_chan->config_state = STATE_CONFIGURED;

	return 0;
err_einval:
	return -EINVAL;
}

/**
 * rsz_set_ratio - Set ratio
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Returns 0 if successful, -EINVAL if invalid output size, upscaling ratio is
 * being requested, or other ratio configuration value is out of bounds
 **/
static int rsz_set_ratio(struct rsz_mult *multipass,
				struct channel_config *rsz_conf_chan)
{
	int alignment = 0;

	rsz_conf_chan->register_config.rsz_cnt = 0;

	if ((multipass->out_hsize > MAX_IMAGE_WIDTH) ||
			(multipass->out_vsize > MAX_IMAGE_WIDTH)) {
		dev_err(rsz_device, "Invalid output size!");
		goto err_einval;
	}
	if (multipass->cbilin) {
		rsz_conf_chan->register_config.rsz_cnt =
				BITSET(rsz_conf_chan->register_config.rsz_cnt,
				SET_BIT_CBLIN);
	}
	if (INPUT_RAM) {
		rsz_conf_chan->register_config.rsz_cnt =
				BITSET(rsz_conf_chan->register_config.rsz_cnt,
				SET_BIT_INPUTRAM);
	}
	if (multipass->inptyp == RSZ_INTYPE_PLANAR_8BIT) {
		rsz_conf_chan->register_config.rsz_cnt =
				BITSET(rsz_conf_chan->register_config.rsz_cnt,
				SET_BIT_INPTYP);
	} else {
		rsz_conf_chan->register_config.rsz_cnt =
				BITRESET(rsz_conf_chan->register_config.
				rsz_cnt, SET_BIT_INPTYP);

		if (multipass->pix_fmt == RSZ_PIX_FMT_UYVY) {
			rsz_conf_chan->register_config.rsz_cnt =
				BITRESET(rsz_conf_chan->register_config.
				rsz_cnt, SET_BIT_YCPOS);
		} else if (multipass->pix_fmt == RSZ_PIX_FMT_YUYV) {
			rsz_conf_chan->register_config.rsz_cnt =
					BITSET(rsz_conf_chan->register_config.
					rsz_cnt, SET_BIT_YCPOS);
		}

	}
	multipass->vrsz =
		(multipass->in_vsize * RATIO_MULTIPLIER) / multipass->out_vsize;
	multipass->hrsz =
		(multipass->in_hsize * RATIO_MULTIPLIER) / multipass->out_hsize;
	if (UP_RSZ_RATIO > multipass->vrsz || UP_RSZ_RATIO > multipass->hrsz) {
		dev_err(rsz_device, "Upscaling ratio not supported!");
		goto err_einval;
	}
	multipass->vrsz = (multipass->in_vsize - NUM_D2TAPS) * RATIO_MULTIPLIER
						/ (multipass->out_vsize - 1);
	multipass->hrsz = ((multipass->in_hsize - NUM_D2TAPS)
						* RATIO_MULTIPLIER) /
						(multipass->out_hsize - 1);

	if (multipass->hrsz <= 512) {
		multipass->hrsz = (multipass->in_hsize - NUM_TAPS)
						* RATIO_MULTIPLIER
						/ (multipass->out_hsize - 1);
		if (multipass->hrsz < 64)
			multipass->hrsz = 64;
		if (multipass->hrsz > 512)
			multipass->hrsz = 512;
		if (multipass->hstph > NUM_PHASES)
			goto err_einval;
		multipass->num_htap = 1;
	} else if (multipass->hrsz >= 513 && multipass->hrsz <= 1024) {
		if (multipass->hstph > NUM_D2PH)
			goto err_einval;
		multipass->num_htap = 0;
	}

	if (multipass->vrsz <= 512) {
		multipass->vrsz = (multipass->in_vsize - NUM_TAPS)
						* RATIO_MULTIPLIER
						/ (multipass->out_vsize - 1);
		if (multipass->vrsz < 64)
			multipass->vrsz = 64;
		if (multipass->vrsz > 512)
			multipass->vrsz = 512;
		if (multipass->vstph > NUM_PHASES)
			goto err_einval;
		multipass->num_vtap = 1;
	} else if (multipass->vrsz >= 513 && multipass->vrsz <= 1024) {
		if (multipass->vstph > NUM_D2PH)
			goto err_einval;
		multipass->num_vtap = 0;
	}

	if ((multipass->in_pitch) % ALIGN32) {
		dev_err(rsz_device, "Invalid input pitch: %d \n",
							multipass->in_pitch);
		goto err_einval;
	}
	if ((multipass->out_pitch) % ALIGN32) {
		dev_err(rsz_device, "Invalid output pitch %d \n",
							multipass->out_pitch);
		goto err_einval;
	}

	if (multipass->vrsz < 256 &&
			(multipass->in_vsize < multipass->out_vsize)) {
		if (multipass->inptyp == RSZ_INTYPE_PLANAR_8BIT)
			alignment = ALIGNMENT;
		else if (multipass->inptyp == RSZ_INTYPE_YCBCR422_16BIT)
			alignment = (ALIGNMENT / 2);
		else
			dev_err(rsz_device, "Invalid input type\n");

		if (!(((multipass->out_hsize % PIXEL_EVEN) == 0)
				&& (multipass->out_hsize % alignment) == 0)) {
			dev_err(rsz_device, "wrong hsize\n");
			goto err_einval;
		}
	}
	if (multipass->hrsz >= 64 && multipass->hrsz <= 1024) {
		if (multipass->out_hsize > MAX_IMAGE_WIDTH) {
			dev_err(rsz_device, "wrong width\n");
			goto err_einval;
		}
		multipass->active = 0;

	} else if (multipass->hrsz > 1024) {
		if (multipass->out_hsize > MAX_IMAGE_WIDTH) {
			dev_err(rsz_device, "wrong width\n");
			goto err_einval;
		}
		if (multipass->hstph > NUM_D2PH)
			goto err_einval;
		multipass->num_htap = 0;
		multipass->out_hsize = multipass->in_hsize * 256 / 1024;
		if (multipass->out_hsize % ALIGN32) {
			multipass->out_hsize +=
				abs((multipass->out_hsize % ALIGN32) - ALIGN32);
		}
		multipass->out_pitch = ((multipass->inptyp) ?
						multipass->out_hsize :
						(multipass->out_hsize * 2));
		multipass->hrsz = ((multipass->in_hsize - NUM_D2TAPS)
						* RATIO_MULTIPLIER)
						/ (multipass->out_hsize - 1);
		multipass->active = 1;

	}

	if (multipass->vrsz > 1024) {
		if (multipass->out_vsize > MAX_IMAGE_WIDTH_HIGH) {
			dev_err(rsz_device, "wrong width\n");
			goto err_einval;
		}

		multipass->out_vsize = multipass->in_vsize * 256 / 1024;
		multipass->vrsz = ((multipass->in_vsize - NUM_D2TAPS)
						* RATIO_MULTIPLIER)
						/ (multipass->out_vsize - 1);
		multipass->active = 1;
		multipass->num_vtap = 0;

	}
	rsz_conf_chan->register_config.rsz_out_size =
						multipass->out_hsize
						& ISPRSZ_OUT_SIZE_HORZ_MASK;

	rsz_conf_chan->register_config.rsz_out_size |=
						(multipass->out_vsize
						<< ISPRSZ_OUT_SIZE_VERT_SHIFT)
						& ISPRSZ_OUT_SIZE_VERT_MASK;

	rsz_conf_chan->register_config.rsz_sdr_inoff =
						multipass->in_pitch
						& ISPRSZ_SDR_INOFF_OFFSET_MASK;

	rsz_conf_chan->register_config.rsz_sdr_outoff =
					multipass->out_pitch
					& ISPRSZ_SDR_OUTOFF_OFFSET_MASK;

	if (multipass->hrsz >= 64 && multipass->hrsz <= 512) {
		if (multipass->hstph > NUM_PHASES)
			goto err_einval;
	} else if (multipass->hrsz >= 64 && multipass->hrsz <= 512) {
		if (multipass->hstph > NUM_D2PH)
			goto err_einval;
	}

	rsz_conf_chan->register_config.rsz_cnt |=
						(multipass->hstph
						<< ISPRSZ_CNT_HSTPH_SHIFT)
						& ISPRSZ_CNT_HSTPH_MASK;

	if (multipass->vrsz >= 64 && multipass->hrsz <= 512) {
		if (multipass->vstph > NUM_PHASES)
			goto err_einval;
	} else if (multipass->vrsz >= 64 && multipass->vrsz <= 512) {
		if (multipass->vstph > NUM_D2PH)
			goto err_einval;
	}

	rsz_conf_chan->register_config.rsz_cnt |=
						(multipass->vstph
						<< ISPRSZ_CNT_VSTPH_SHIFT)
						& ISPRSZ_CNT_VSTPH_MASK;

	rsz_conf_chan->register_config.rsz_cnt |=
						(multipass->hrsz - 1)
						& ISPRSZ_CNT_HRSZ_MASK;

	rsz_conf_chan->register_config.rsz_cnt |=
						((multipass->vrsz - 1)
						<< ISPRSZ_CNT_VRSZ_SHIFT)
						& ISPRSZ_CNT_VRSZ_MASK;

	return 0;
err_einval:
	return -EINVAL;
}

/**
 * rsz_config_ratio - Configure ratio
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Configure ratio
 **/
static void rsz_config_ratio(struct rsz_mult *multipass,
				struct channel_config *rsz_conf_chan)
{
	int hsize;
	int vsize;
	int coeffcounter;

	if (multipass->hrsz <= 512) {
		hsize = ((32 * multipass->hstph + (multipass->out_hsize - 1)
					* multipass->hrsz + 16) >> 8) + 7;
	} else {
		hsize = ((64 * multipass->hstph + (multipass->out_hsize - 1)
					* multipass->hrsz + 32) >> 8) + 7;
	}
	if (multipass->vrsz <= 512) {
		vsize = ((32 * multipass->vstph + (multipass->out_vsize - 1)
					* multipass->vrsz + 16) >> 8) + 4;
	} else {
		vsize = ((64 * multipass->vstph + (multipass->out_vsize - 1)
					* multipass->vrsz + 32) >> 8) + 7;
	}
	rsz_conf_chan->register_config.rsz_in_size = hsize;

	rsz_conf_chan->register_config.rsz_in_size |=
					((vsize << ISPRSZ_IN_SIZE_VERT_SHIFT)
					& ISPRSZ_IN_SIZE_VERT_MASK);

	for (coeffcounter = 0; coeffcounter < MAX_COEF_COUNTER;
							coeffcounter++) {
		if (multipass->num_htap) {
			rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter] =
					(multipass->tap4filt_coeffs[2
					* coeffcounter]
					& ISPRSZ_HFILT10_COEF0_MASK);
			rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter] |=
					((multipass->tap4filt_coeffs[2
					* coeffcounter + 1]
					<< ISPRSZ_HFILT10_COEF1_SHIFT)
					& ISPRSZ_HFILT10_COEF1_MASK);
		} else {
			rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter] =
					(multipass->tap7filt_coeffs[2
					* coeffcounter]
					& ISPRSZ_HFILT10_COEF0_MASK);

			rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter] |=
					((multipass->tap7filt_coeffs[2
					* coeffcounter + 1]
					<< ISPRSZ_HFILT10_COEF1_SHIFT)
					& ISPRSZ_HFILT10_COEF1_MASK);
		}

		if (multipass->num_vtap) {
			rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter] =
					(multipass->tap4filt_coeffs[2
					* coeffcounter]
					& ISPRSZ_VFILT10_COEF0_MASK);

			rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter] |=
					((multipass->tap4filt_coeffs[2
					* coeffcounter + 1]
					<< ISPRSZ_VFILT10_COEF1_SHIFT) &
					ISPRSZ_VFILT10_COEF1_MASK);
		} else {
			rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter] =
					(multipass->tap7filt_coeffs[2
					* coeffcounter]
					& ISPRSZ_VFILT10_COEF0_MASK);
			rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter] |=
					((multipass->tap7filt_coeffs[2
					* coeffcounter + 1]
					<< ISPRSZ_VFILT10_COEF1_SHIFT)
					& ISPRSZ_VFILT10_COEF1_MASK);
		}
	}
}

/**
 * rsz_get_params - Gets the parameter values
 * @params: Structure containing the Resizer Wrapper parameters
 * @rsz_conf_chan: Structure containing channel configuration
 *
 * Used to get the Resizer hardware settings associated with the
 * current logical channel represented by fd.
 **/
static int rsz_get_params(struct rsz_params *params,
					struct channel_config *rsz_conf_chan)
{
	int coeffcounter;

	if (rsz_conf_chan->config_state) {
		dev_err(rsz_device, "state not configured\n");
		return -EINVAL;
	}

	params->in_hsize = rsz_conf_chan->register_config.rsz_in_size
					& ISPRSZ_IN_SIZE_HORZ_MASK;
	params->in_vsize = (rsz_conf_chan->register_config.rsz_in_size
					& ISPRSZ_IN_SIZE_VERT_MASK)
					>> ISPRSZ_IN_SIZE_VERT_SHIFT;

	params->in_pitch = rsz_conf_chan->register_config.rsz_sdr_inoff
					& ISPRSZ_SDR_INOFF_OFFSET_MASK;

	params->out_hsize = rsz_conf_chan->register_config.rsz_out_size
					& ISPRSZ_OUT_SIZE_HORZ_MASK;

	params->out_vsize = (rsz_conf_chan->register_config.rsz_out_size
					& ISPRSZ_OUT_SIZE_VERT_MASK)
					>> ISPRSZ_OUT_SIZE_VERT_SHIFT;

	params->out_pitch = rsz_conf_chan->register_config.rsz_sdr_outoff
					& ISPRSZ_SDR_OUTOFF_OFFSET_MASK;

	params->cbilin = (rsz_conf_chan->register_config.rsz_cnt
					& SET_BIT_CBLIN) >> SET_BIT_CBLIN;

	params->inptyp = (rsz_conf_chan->register_config.rsz_cnt
					& ISPRSZ_CNT_INPTYP_MASK)
					>> SET_BIT_INPTYP;
	params->horz_starting_pixel = ((rsz_conf_chan->register_config.
					rsz_in_start
					& ISPRSZ_IN_START_HORZ_ST_MASK));
	params->vert_starting_pixel = ((rsz_conf_chan->register_config.
					rsz_in_start
					& ISPRSZ_IN_START_VERT_ST_MASK)
					>> ISPRSZ_IN_START_VERT_ST_SHIFT);

	params->hstph = ((rsz_conf_chan->register_config.rsz_cnt
					& ISPRSZ_CNT_HSTPH_MASK
					>> ISPRSZ_CNT_HSTPH_SHIFT));
	params->vstph = ((rsz_conf_chan->register_config.rsz_cnt
					& ISPRSZ_CNT_VSTPH_MASK
					>> ISPRSZ_CNT_VSTPH_SHIFT));

	for (coeffcounter = 0; coeffcounter < MAX_COEF_COUNTER;
							coeffcounter++) {
		params->tap4filt_coeffs[2 * coeffcounter] =
					rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter]
					& ISPRSZ_HFILT10_COEF0_MASK;

		params->tap4filt_coeffs[2 * coeffcounter + 1] =
					(rsz_conf_chan->register_config.
					rsz_coeff_horz[coeffcounter]
					& ISPRSZ_HFILT10_COEF1_MASK)
					>> ISPRSZ_HFILT10_COEF1_SHIFT;

		params->tap7filt_coeffs[2 * coeffcounter] =
					rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter]
					& ISPRSZ_VFILT10_COEF0_MASK;

		params->tap7filt_coeffs[2 * coeffcounter + 1] =
					(rsz_conf_chan->register_config.
					rsz_coeff_vert[coeffcounter]
					& ISPRSZ_VFILT10_COEF1_MASK)
					>> ISPRSZ_VFILT10_COEF1_SHIFT;

	}

	params->yenh_params.type = (rsz_conf_chan->register_config.rsz_yehn
					& ISPRSZ_YENH_ALGO_MASK)
					>> ISPRSZ_YENH_ALGO_SHIFT;

	params->yenh_params.core = rsz_conf_chan->register_config.rsz_yehn
					& ISPRSZ_YENH_CORE_MASK;

	params->yenh_params.gain = (rsz_conf_chan->register_config.rsz_yehn
					& ISPRSZ_YENH_GAIN_MASK)
					>> ISPRSZ_YENH_GAIN_SHIFT;

	params->yenh_params.slop = (rsz_conf_chan->register_config.rsz_yehn
					& ISPRSZ_YENH_SLOP_MASK)
					>> ISPRSZ_YENH_SLOP_SHIFT;

	params->pix_fmt = ((rsz_conf_chan->register_config.rsz_cnt
					& ISPRSZ_CNT_PIXFMT_MASK)
					>> SET_BIT_YCPOS);

	if (params->pix_fmt)
		params->pix_fmt = RSZ_PIX_FMT_UYVY;
	else
		params->pix_fmt = RSZ_PIX_FMT_YUYV;

	return 0;
}

/**
 * rsz_calculate_crop - Calculate Crop values
 * @rsz_conf_chan: Structure containing channel configuration
 * @cropsize: Structure containing crop parameters
 *
 * Calculate Crop values
 **/
static void rsz_calculate_crop(struct channel_config *rsz_conf_chan,
						struct rsz_cropsize *cropsize)
{
	int luma_enable;

	cropsize->hcrop = 0;
	cropsize->vcrop = 0;

	luma_enable = (rsz_conf_chan->register_config.rsz_yehn
						& ISPRSZ_YENH_ALGO_MASK)
						>> ISPRSZ_YENH_ALGO_SHIFT;

	if (luma_enable)
		cropsize->hcrop += 2;
}

/**
 * rsz_vbq_release - Videobuffer queue release
 * @q: Structure containing the videobuffer queue file handle, and device
 *     structure which contains the actual configuration.
 * @vb: Structure containing the videobuffer used for resizer processing.
 **/
static void rsz_vbq_release(struct videobuf_queue *q,
						struct videobuf_buffer *vb)
{
	int i;
	struct rsz_fh *fh = q->priv_data;

	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		struct videobuf_dmabuf *dma = NULL;
		if (!q->bufs[i])
			continue;
		if (q->bufs[i]->memory != V4L2_MEMORY_MMAP)
			continue;
		dma = videobuf_to_dma(q->bufs[i]);
		videobuf_dma_unmap(q, dma);
		videobuf_dma_free(dma);
	}

	ispmmu_unmap(fh->isp_addr_read);
	ispmmu_unmap(fh->isp_addr_write);
	fh->isp_addr_read = 0;
	fh->isp_addr_write = 0;
	spin_lock(&fh->vbq_lock);
	vb->state = VIDEOBUF_NEEDS_INIT;
	spin_unlock(&fh->vbq_lock);

}

/**
 * rsz_vbq_setup - Sets up the videobuffer size and validates count.
 * @q: Structure containing the videobuffer queue file handle, and device
 *     structure which contains the actual configuration.
 * @cnt: Number of buffers requested
 * @size: Size in bytes of the buffer used for previewing
 *
 * Always returns 0.
 **/
static int rsz_vbq_setup(struct videobuf_queue *q, unsigned int *cnt,
							unsigned int *size)
{
	struct rsz_fh *fh = q->priv_data;
	struct rsz_mult *multipass = fh->multipass;
	u32 insize, outsize;

	spin_lock(&fh->vbq_lock);
	if (*cnt <= 0)
		*cnt = VIDEO_MAX_FRAME;

	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;

	outsize = multipass->out_pitch * multipass->out_vsize;
	insize = multipass->in_pitch * multipass->in_vsize;
	if (*cnt == 1 && (outsize > insize)) {
		dev_err(rsz_device, "2 buffers are required for Upscaling "
								"mode\n");
		goto err_einval;
	}
	if (!fh->params->in_hsize || !fh->params->in_vsize) {
		dev_err(rsz_device, "Can't setup buffer size\n");
		goto err_einval;
	} else {
		if (outsize > insize)
			*size = outsize;
		else
			*size = insize;

		fh->rsz_bufsize = *size;
	}
	spin_unlock(&fh->vbq_lock);

	return 0;
err_einval:
	spin_unlock(&fh->vbq_lock);
	return -EINVAL;
}

/**
 * rsz_vbq_prepare - Videobuffer is prepared and mmapped.
 * @q: Structure containing the videobuffer queue file handle, and device
 *     structure which contains the actual configuration.
 * @vb: Structure containing the videobuffer used for resizer processing.
 * @field: Type of field to set in videobuffer device.
 *
 * Returns 0 if successful, or -EINVAL if buffer couldn't get allocated, or
 * -EIO if the ISP MMU mapping fails
 **/
static int rsz_vbq_prepare(struct videobuf_queue *q,
						struct videobuf_buffer *vb,
						enum v4l2_field field)
{
	struct rsz_fh *fh = q->priv_data;
	struct channel_config *rsz_conf_chan = fh->config;
	struct rsz_mult *multipass = fh->multipass;
	int err = 0;
	unsigned int isp_addr, insize, outsize;
	struct videobuf_dmabuf *dma = videobuf_to_dma(vb);

	spin_lock(&fh->vbq_lock);
	if (vb->baddr) {
		vb->size = fh->rsz_bufsize;
		vb->bsize = fh->rsz_bufsize;
	} else {
		spin_unlock(&fh->vbq_lock);
		dev_err(rsz_device, "No user buffer allocated\n");
		goto out;
	}
	if (vb->i) {
		vb->width = fh->params->out_hsize;
		vb->height = fh->params->out_vsize;
	} else {
		vb->width = fh->params->in_hsize;
		vb->height = fh->params->in_vsize;
	}

	vb->field = field;
	spin_unlock(&fh->vbq_lock);

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		err = videobuf_iolock(q, vb, NULL);
		if (!err) {
			isp_addr = ispmmu_map_sg(dma->sglist, dma->sglen);
			if (!isp_addr)
				err = -EIO;
			else {
				if (vb->i) {
					rsz_conf_chan->register_config.
							rsz_sdr_outadd
							= isp_addr;
					fh->isp_addr_write = isp_addr;
					rsz_conf_chan->output_buf_index = vb->i;
				} else {
					rsz_conf_chan->register_config.
							rsz_sdr_inadd
							= isp_addr;
					rsz_conf_chan->input_buf_index = vb->i;
					outsize = multipass->out_pitch *
							multipass->out_vsize;
					insize = multipass->in_pitch *
							multipass->in_vsize;
					if (outsize < insize) {
						rsz_conf_chan->register_config.
								rsz_sdr_outadd
								= isp_addr;
						rsz_conf_chan->
							output_buf_index =
							vb->i;
					}

					fh->isp_addr_read = isp_addr;
				}
			}
		}

	}

	if (!err) {
		spin_lock(&fh->vbq_lock);
		vb->state = VIDEOBUF_PREPARED;
		spin_unlock(&fh->vbq_lock);
		flush_cache_user_range(NULL, vb->baddr, (vb->baddr
								+ vb->bsize));
	} else
		rsz_vbq_release(q, vb);

out:
	return err;
}

static void rsz_vbq_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	return;
}

/**
 * rsz_open - Initializes and opens the Resizer Wrapper
 * @inode: Inode structure associated with the Resizer Wrapper
 * @filp: File structure associated with the Resizer Wrapper
 *
 * Returns 0 if successful, -EBUSY if its already opened or the ISP module is
 * not available, or -ENOMEM if its unable to allocate the device in kernel
 * space memory.
 **/
static int rsz_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct channel_config *rsz_conf_chan;
	struct rsz_fh *fh;
	struct device_params *device = device_config;
	struct rsz_params *params;
	struct rsz_mult *multipass;

	if ((filp->f_flags & O_NONBLOCK) == O_NONBLOCK) {
		printk(KERN_DEBUG "omap-resizer: Device is opened in "
					"non blocking mode\n");
	} else {
		printk(KERN_DEBUG "omap-resizer: Device is opened in blocking "
					"mode\n");
	}
	fh = kzalloc(sizeof(struct rsz_fh), GFP_KERNEL);
	if (NULL == fh)
		return -ENOMEM;

	isp_get();

	rsz_conf_chan = kzalloc(sizeof(struct channel_config), GFP_KERNEL);
	if (rsz_conf_chan == NULL) {
		dev_err(rsz_device, "\n cannot allocate memory to config");
		ret = -ENOMEM;
		goto err_enomem0;
	}
	params = kzalloc(sizeof(struct rsz_params), GFP_KERNEL);
	if (params == NULL) {
		dev_err(rsz_device, "\n cannot allocate memory to params");
		ret = -ENOMEM;
		goto err_enomem1;
	}
	multipass = kzalloc(sizeof(struct rsz_mult), GFP_KERNEL);
	if (multipass == NULL) {
		dev_err(rsz_device, "\n cannot allocate memory to multipass");
		ret = -ENOMEM;
		goto err_enomem2;
	}

	fh->multipass = multipass;
	fh->params = params;
	fh->config = rsz_conf_chan;

	if (mutex_lock_interruptible(&device->reszwrap_mutex)) {
		ret = -EINTR;
		goto err_enomem2;
	}
	device->opened++;
	mutex_unlock(&device->reszwrap_mutex);

	rsz_conf_chan->config_state = STATE_NOT_CONFIGURED;
	rsz_conf_chan->status = CHANNEL_FREE;

	filp->private_data = fh;
	fh->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fh->device = device;

	videobuf_queue_sg_init(&fh->vbq, &device->vbq_ops, NULL,
					&fh->vbq_lock, fh->type,
					V4L2_FIELD_NONE,
					sizeof(struct videobuf_buffer), fh);

	spin_lock_init(&fh->vbq_lock);
	mutex_init(&rsz_conf_chan->chanprotection_mutex);

	return 0;
err_enomem2:
	kfree(params);
err_enomem1:
	kfree(rsz_conf_chan);
err_enomem0:
	kfree(fh);
	return ret;
}

/**
 * rsz_release - Releases Resizer Wrapper and frees up allocated memory
 * @inode: Inode structure associated with the Resizer Wrapper
 * @filp: File structure associated with the Resizer Wrapper
 *
 * Returns 0 if successful, or -EBUSY if channel is being used.
 **/
static int rsz_release(struct inode *inode, struct file *filp)
{
	u32 timeout = 0;
	struct rsz_fh *fh = filp->private_data;
	struct channel_config *rsz_conf_chan = fh->config;
	struct rsz_params *params = fh->params;
	struct rsz_mult *multipass = fh->multipass;
	struct videobuf_queue *q = &fh->vbq;

	while ((rsz_conf_chan->status != CHANNEL_FREE) && (timeout < 20)) {
		timeout++;
		schedule();
	}
	if (mutex_lock_interruptible(&device_config->reszwrap_mutex))
		return -EINTR;
	device_config->opened--;
	mutex_unlock(&device_config->reszwrap_mutex);
	/* This will Free memory allocated to the buffers,
	 * and flushes the queue
	 */
	videobuf_queue_cancel(q);
	fh->params = NULL;
	fh->config = NULL;

	fh->rsz_bufsize = 0;
	filp->private_data = NULL;

	kfree(rsz_conf_chan);
	kfree(params);
	kfree(multipass);
	kfree(fh);

	isp_put();

	return 0;
}

/**
 * rsz_mmap - Memory maps the Resizer Wrapper module.
 * @file: File structure associated with the Resizer Wrapper
 * @vma: Virtual memory area structure.
 *
 * Returns 0 if successful, or returned value by the videobuf_mmap_mapper()
 * function.
 **/
static int rsz_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct rsz_fh *fh = file->private_data;

	return videobuf_mmap_mapper(&fh->vbq, vma);
}

/**
 * rsz_ioctl - I/O control function for Resizer Wrapper
 * @inode: Inode structure associated with the Resizer Wrapper.
 * @file: File structure associated with the Resizer Wrapper.
 * @cmd: Type of command to execute.
 * @arg: Argument to send to requested command.
 *
 * Returns 0 if successful, -EBUSY if channel is being used, -1 if bad command
 * passed or access is denied, -EFAULT if copy_from_user() or copy_to_user()
 * fails, -EINVAL if parameter validation fails or parameter structure is not
 * present.
 **/
static long rsz_unlocked_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	int ret = 0;
	struct rsz_fh *fh = file->private_data;
	struct device_params *device = fh->device;
	struct channel_config *rsz_conf_chan = fh->config;

	if ((_IOC_TYPE(cmd) != RSZ_IOC_BASE)
					|| (_IOC_NR(cmd) > RSZ_IOC_MAXNR)) {
		dev_err(rsz_device, "Bad command value \n");
		goto err_minusone;
	}

	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));

	if (ret) {
		dev_err(rsz_device, "Access denied\n");
		goto err_minusone;
	}

	switch (cmd) {
	case RSZ_REQBUF:
	{
		struct v4l2_requestbuffers req_buf;
		if (copy_from_user(&req_buf, (struct v4l2_requestbuffers *)arg,
					sizeof(struct v4l2_requestbuffers))) {
			goto err_efault;
		}
		if (mutex_lock_interruptible(&rsz_conf_chan->
							chanprotection_mutex))
			goto err_eintr;
		ret = videobuf_reqbufs(&fh->vbq, (void *)&req_buf);
		mutex_unlock(&rsz_conf_chan->chanprotection_mutex);
		break;
	}
	case RSZ_QUERYBUF:
	{
		struct v4l2_buffer buf;
		if (copy_from_user(&buf, (struct v4l2_buffer *)arg,
						sizeof(struct v4l2_buffer))) {
			goto err_efault;
		}
		if (mutex_lock_interruptible(&rsz_conf_chan->
							chanprotection_mutex))
			goto err_eintr;
		ret = videobuf_querybuf(&fh->vbq, (void *)&buf);
		mutex_unlock(&rsz_conf_chan->chanprotection_mutex);
		if (copy_to_user((struct v4l2_buffer *)arg, &buf,
						sizeof(struct v4l2_buffer)))
			ret = -EFAULT;
		break;
	}
	case RSZ_QUEUEBUF:
	{
		struct v4l2_buffer buf;
		if (copy_from_user(&buf, (struct v4l2_buffer *)arg,
						sizeof(struct v4l2_buffer))) {
			goto err_efault;
		}
		if (mutex_lock_interruptible(&rsz_conf_chan->
							chanprotection_mutex))
			goto err_eintr;
		ret = videobuf_qbuf(&fh->vbq, (void *)&buf);
		mutex_unlock(&rsz_conf_chan->chanprotection_mutex);
		break;
	}
	case RSZ_S_PARAM:
	{
		struct rsz_params *params = fh->params;
		if (copy_from_user(params, (struct rsz_params *)arg,
						sizeof(struct rsz_params))) {
			goto err_efault;
		}
		if (mutex_lock_interruptible(&rsz_conf_chan->
							chanprotection_mutex))
			goto err_eintr;
		ret = rsz_set_params(fh->multipass, params, rsz_conf_chan);
		mutex_unlock(&rsz_conf_chan->chanprotection_mutex);
		break;
	}
	case RSZ_G_PARAM:
		ret = rsz_get_params((struct rsz_params *)arg, rsz_conf_chan);
		break;

	case RSZ_G_STATUS:
	{
		struct rsz_status *status;
		status = (struct rsz_status *)arg;
		status->chan_busy = rsz_conf_chan->status;
		status->hw_busy = ispresizer_busy();
		status->src = INPUT_RAM;
		break;
	}
	case RSZ_RESIZE:
		if (file->f_flags & O_NONBLOCK) {
			if (ispresizer_busy())
				return -EBUSY;
			else {
				if (!mutex_trylock(&device->reszwrap_mutex))
					return -EBUSY;
			}
		} else {
			if (mutex_lock_interruptible(&device->reszwrap_mutex))
				goto err_eintr;
		}
		ret = rsz_start((int *)arg, fh);
		mutex_unlock(&device->reszwrap_mutex);
		break;
	case RSZ_GET_CROPSIZE:
		rsz_calculate_crop(rsz_conf_chan, (struct rsz_cropsize *)arg);
		break;

	default:
		dev_err(rsz_device, "resizer_ioctl: Invalid Command Value");
		ret = -EINVAL;
	}

out:
	return (long)ret;
err_minusone:
	ret = -1;
	goto out;
err_eintr:
	ret = -EINTR;
	goto out;
err_efault:
	ret = -EFAULT;
	goto out;
}

static struct file_operations rsz_fops = {
	.owner = THIS_MODULE,
	.open = rsz_open,
	.release = rsz_release,
	.mmap = rsz_mmap,
	.unlocked_ioctl = rsz_unlocked_ioctl,
};

/**
 * rsz_isr - Interrupt Service Routine for Resizer wrapper
 * @status: ISP IRQ0STATUS register value
 * @arg1: Currently not used
 * @arg2: Currently not used
 *
 * Interrupt Service Routine for Resizer wrapper
 **/
static void rsz_isr(unsigned long status, isp_vbq_callback_ptr arg1, void *arg2)
{

	if ((status & RESZ_DONE) != RESZ_DONE)
		return;

	complete(&(device_config->compl_isr));

}

/**
 * resizer_platform_release - Acts when Reference count is zero
 * @device: Structure containing ISP resizer wrapper global information
 *
 * This is called when the reference count goes to zero.
 **/
static void resizer_platform_release(struct device *device)
{
}

/**
 * resizer_probe - Checks for device presence
 * @device: Structure containing details of the current device.
 *
 * Always returns 0.
 **/
static int __init resizer_probe(struct platform_device *device)
{
	return 0;
}

/**
 * resizer_remove - Handles the removal of the driver
 * @omap_resizer_device: Structure containing details of the current device.
 *
 * Always returns 0.
 **/
static int resizer_remove(struct platform_device *omap_resizer_device)
{
	return 0;
}

static struct class *rsz_class;
static struct cdev c_dev;
static dev_t dev;
static struct platform_device omap_resizer_device = {
	.name = OMAP_REZR_NAME,
	.id = 2,
	.dev = {
		.release = resizer_platform_release,}
};

static struct platform_driver omap_resizer_driver = {
	.probe = resizer_probe,
	.remove = resizer_remove,
	.driver = {
			.bus = &platform_bus_type,
			.name = OMAP_REZR_NAME,
	},
};

/**
 * omap_rsz_init - Initialization of Resizer Wrapper
 *
 * Returns 0 if successful, -ENOMEM if could not allocate memory, -ENODEV if
 * could not register the wrapper as a character device, or other errors if the
 * device or driver can't register.
 **/
static int __init omap_rsz_init(void)
{
	int ret = 0;
	struct device_params *device;
	device = kzalloc(sizeof(struct device_params), GFP_KERNEL);
	if (!device) {
		dev_err(rsz_device, OMAP_REZR_NAME ": could not allocate "
								"memory\n");
		return -ENOMEM;
	}

	ret = alloc_chrdev_region(&dev, 0, 1, OMAP_REZR_NAME);
	if (ret < 0) {
		dev_err(rsz_device, OMAP_REZR_NAME ": intialization failed. "
			"Could not allocate region "
			"for character device\n");
		kfree(device);
		return -ENODEV;
	}

	/* Register the driver in the kernel */
	/* Initialize of character device */
	cdev_init(&c_dev, &rsz_fops);
	c_dev.owner = THIS_MODULE;
	c_dev.ops = &rsz_fops;

	/* Addding character device */
	ret = cdev_add(&c_dev, dev, 1);
	if (ret) {
		dev_err(rsz_device, OMAP_REZR_NAME ": Error adding "
			"device - %d\n", ret);
		goto fail2;
	}
	rsz_major = MAJOR(dev);

	/* register driver as a platform driver */
	ret = platform_driver_register(&omap_resizer_driver);
	if (ret) {
		dev_err(rsz_device, OMAP_REZR_NAME
		       ": Failed to register platform driver!\n");
		goto fail3;
	}

	/* Register the drive as a platform device */
	ret = platform_device_register(&omap_resizer_device);
	if (ret) {
		dev_err(rsz_device, OMAP_REZR_NAME
		       ": Failed to register platform device!\n");
		goto fail4;
	}

	rsz_class = class_create(THIS_MODULE, OMAP_REZR_NAME);
	if (!rsz_class) {
		dev_err(rsz_device, OMAP_REZR_NAME
			": Failed to create class!\n");
		goto fail5;
	}

	/* make entry in the devfs */
	rsz_device = device_create(rsz_class, rsz_device,
						MKDEV(rsz_major, 0), NULL,
						OMAP_REZR_NAME);
	dev_dbg(rsz_device, OMAP_REZR_NAME ": Registered Resizer Wrapper\n");
	device->opened = 0;

	device->vbq_ops.buf_setup = rsz_vbq_setup;
	device->vbq_ops.buf_prepare = rsz_vbq_prepare;
	device->vbq_ops.buf_release = rsz_vbq_release;
	device->vbq_ops.buf_queue = rsz_vbq_queue;
	init_completion(&device->compl_isr);
	mutex_init(&device->reszwrap_mutex);

	device_config = device;
	return 0;

fail5:
	platform_device_unregister(&omap_resizer_device);
fail4:
	platform_driver_unregister(&omap_resizer_driver);
fail3:
	cdev_del(&c_dev);
fail2:
	unregister_chrdev_region(dev, 1);
	kfree(device);
	return ret;
}

/**
 * omap_rsz_exit - Close of Resizer Wrapper
 **/
void __exit omap_rsz_exit(void)
{
	device_destroy(rsz_class, dev);
	class_destroy(rsz_class);
	platform_device_unregister(&omap_resizer_device);
	platform_driver_unregister(&omap_resizer_driver);
	cdev_del(&c_dev);
	unregister_chrdev_region(dev, 1);
	kfree(device_config);
}

module_init(omap_rsz_init)
module_exit(omap_rsz_exit)

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP ISP Resizer");
MODULE_LICENSE("GPL");
