/*
 * drivers/media/video/isp/omap_previewer.h
 *
 * Header file for Preview module wrapper in TI's OMAP3430 ISP
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Contributors:
 * 	Leonides Martinez <leonides.martinez@ti.com>
 * 	Sergio Aguirre <saaguirre@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "isppreview.h"

#ifndef OMAP_ISP_PREVIEW_WRAP_H
#define OMAP_ISP_PREVIEW_WRAP_H

#define PREV_IOC_BASE			'P'
#define PREV_REQBUF			_IOWR(PREV_IOC_BASE, 1,\
						struct v4l2_requestbuffers)
#define PREV_QUERYBUF			_IOWR(PREV_IOC_BASE, 2,\
							struct v4l2_buffer)
#define PREV_SET_PARAM			_IOW(PREV_IOC_BASE, 3,\
							struct prev_params)
#define PREV_GET_PARAM			_IOWR(PREV_IOC_BASE, 4,\
							struct prev_params)
#define PREV_PREVIEW			_IOR(PREV_IOC_BASE, 5, int)
#define PREV_GET_STATUS			_IOR(PREV_IOC_BASE, 6, char)
#define PREV_GET_CROPSIZE		_IOR(PREV_IOC_BASE, 7,\
							struct prev_cropsize)
#define PREV_QUEUEBUF			_IOWR(PREV_IOC_BASE, 8,\
							struct v4l2_buffer)
#define PREV_IOC_MAXNR			8

#define LUMA_TABLE_SIZE			128
#define GAMMA_TABLE_SIZE		1024
#define CFA_COEFF_TABLE_SIZE		576
#define NOISE_FILTER_TABLE_SIZE		256

#define MAX_IMAGE_WIDTH			3300

#define PREV_INWIDTH_8BIT		0	/* pixel width of 8 bits */
#define PREV_INWIDTH_10BIT		1	/* pixel width of 10 bits */

#define PREV_32BYTES_ALIGN_MASK		0xFFFFFFE0
#define PREV_16PIX_ALIGN_MASK		0xFFFFFFF0

/**
 * struct prev_rgbblending - Structure for RGB2RGB blending parameters
 * @blending: Color correlation 3x3 matrix.
 * @offset: Color correlation offsets.
 */
struct prev_rgbblending {
	short blending[RGB_MAX][RGB_MAX];	/* color correlation 3x3
						 * matrix.
						 */
	short offset[RGB_MAX];			/* color correlation offsets */
};

/**
 * struct prev_cfa_coeffs - Structure for CFA coefficients
 * @hthreshold: Horizontal threshold.
 * @vthreshold: Vertical threshold.
 * @coeffs: CFA coefficients
 */
struct prev_cfa_coeffs {
	char hthreshold, vthreshold;
	int coeffs[CFA_COEFF_TABLE_SIZE];
};

/**
 * struct prev_gamma_coeffs - Structure for Gamma Coefficients
 * @red: Table of gamma correction values for red color.
 * @green: Table of gamma correction values for green color.
 * @blue: Table of gamma correction values for blue color.
 */
struct prev_gamma_coeffs {
	unsigned char red[GAMMA_TABLE_SIZE];
	unsigned char green[GAMMA_TABLE_SIZE];
	unsigned char blue[GAMMA_TABLE_SIZE];
};

/**
 * struct prev_noiseflt_coeffs - Structure for Noise Filter Coefficients.
 * @noise: Noise filter table.
 * @strength: Used to find out weighted average.
 */
struct prev_noiseflt_coeffs {
	unsigned char noise[NOISE_FILTER_TABLE_SIZE];
	unsigned char strength;
};

/**
 * struct prev_chroma_spr - Structure for Chroma Suppression.
 * @hpfy: High passed version of Y or normal Y.
 * @threshold: Threshold for chroma suppress.
 * @gain: Chroma suppression gain
 */
struct prev_chroma_spr {
	unsigned char hpfy;
	char threshold;
	unsigned char gain;
};

/**
 * struct prev_status - Structure to know status of the hardware
 * @hw_busy: Flag to indicate if Hardware is Busy.
 */
struct prev_status {
	char hw_busy;
};

/**
 * struct prev_cropsize - Structure to know crop size.
 * @hcrop: Horizontal size of crop window.
 * @vcrop: Vertical size of crop window.
 */
struct prev_cropsize {
	int hcrop;
	int vcrop;
};

/**
 * struct prev_device - Global device information structure.
 * @params: Pointer to structure containing preview parameters.
 * @opened: State of the device.
 * @wfc: Wait for completion. Used for locking operations.
 * @prevwrap_mutex: Mutex for preview wrapper use.
 * @inout_vbq_lock: Spinlock for in/out videobuf queues.
 * @lsc_vbq_lock: Spinlock for LSC videobuf queues.
 * @vbq_ops: Videobuf queue operations
 * @isp_addr_read: Input/Output address
 * @isp_addr_read: LSC address
 */
struct prev_device {
	struct prev_params *params;
	unsigned char opened;
	struct completion wfc;
	struct mutex prevwrap_mutex;
	spinlock_t inout_vbq_lock; /* Spinlock for in/out videobuf queues. */
	spinlock_t lsc_vbq_lock; /* Spinlock for LSC videobuf queues. */
	struct videobuf_queue_ops vbq_ops;
	dma_addr_t isp_addr_read;
	dma_addr_t isp_addr_lsc;
};

/**
 * struct prev_fh - Per-filehandle data structure
 * @inout_type: Used buffer type for I/O.
 * @inout_vbq: I/O Videobuffer queue.
 * @lsc_type: Used buffer type for LSC.
 * @lsc_vbq: LSC Videobuffer queue.
 * @device: Pointer to device information structure.
 */
struct prev_fh {
	enum v4l2_buf_type inout_type;
	struct videobuf_queue inout_vbq;
	enum v4l2_buf_type lsc_type;
	struct videobuf_queue lsc_vbq;
	struct prev_device *device;
};
#endif
