/*
 * drivers/media/video/isp/omap_previewer.c
 *
 * Wrapper for Preview module in TI's OMAP3430 ISP
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

#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <media/v4l2-dev.h>
#include <asm/cacheflush.h>

#include "isp.h"
#include "ispmmu.h"
#include "ispreg.h"
#include "omap_previewer.h"

#define OMAP_PREV_NAME		"omap-previewer"

#define BIT_SET(var, shift, mask, val)		\
	do {					\
		var = (var & ~(mask << shift))	\
			| (val << shift);	\
	} while (0)

#define ISP_CTRL_SBL_SHARED_RPORTB	(1 << 28)
#define ISP_CTRL_SBL_SHARED_RPORTA	(1 << 27)
#define SBL_RD_RAM_EN				18

static u32 isp_ctrl;
static u32 prv_wsdr_addr;
static int prev_major = -1;
static struct device *prev_dev;
static struct class *prev_class;
static struct prev_device *prevdevice;
static struct platform_driver omap_previewer_driver;
static u32 prev_bufsize;
static u32 lsc_bufsize;
static struct prev_params isppreview_tmp;

/**
 * prev_calculate_crop - Calculate crop size according to device parameters
 * @device: Structure containing ISP preview wrapper global information
 * @crop: Structure containing crop size
 *
 * This function is used to calculate frame size reduction depending on
 * the features enabled by the application.
 **/
static int prev_calculate_crop(struct prev_device *device,
						struct prev_cropsize *crop)
{
	int ret = 0;

	if (!device || !crop) {
		dev_err(prev_dev, "%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	crop->hcrop = device->out_hsize;
	crop->vcrop = device->out_vsize;

	dev_dbg(prev_dev, "%s: Exit (%dx%d -> %dx%d)\n", __func__,
		device->params->size_params.hsize,
		device->params->size_params.vsize,
		crop->hcrop, crop->vcrop);

	return ret;
}

/**
 * prev_get_status - Get status of ISP preview module
 * @status: Structure containing the busy state.
 *
 * Checks if the ISP preview module is busy.
 *
 * Returns 0 if successful, or -EINVAL if the status parameter is invalid.
 **/
static int prev_get_status(struct prev_status *status)
{
	if (!status) {
		dev_err(prev_dev, "%s: invalid argument\n", __func__);
		return -EINVAL;
	}
	status->hw_busy = (char)isppreview_busy();
	return 0;
}

/**
 * prev_hw_setup - Stores the desired configuration in the proper HW registers
 * @config: Structure containing the desired configuration for ISP preview
 *          module.
 *
 * Reads the structure sent, and modifies the desired registers.
 *
 * Always returns 0.
 **/
static int prev_hw_setup(struct prev_params *config)
{
	dev_dbg(prev_dev, "%s: Enter\n", __func__);

	if (config->features & PREV_AVERAGER)
		isppreview_config_averager(config->average);
	else
		isppreview_config_averager(0);

	if (config->features & PREV_INVERSE_ALAW)
		isppreview_enable_invalaw(1);
	else
		isppreview_enable_invalaw(0);

	if (config->features & PREV_HORZ_MEDIAN_FILTER) {
		isppreview_config_hmed(config->hmf_params);
		isppreview_enable_hmed(1);
	} else
		isppreview_enable_hmed(0);

	if (config->features & PREV_DARK_FRAME_SUBTRACT) {
		dev_dbg(prev_dev, "%s: darkaddr %08x, darklineoffset %d\n",
			__func__,
			config->drkf_params.addr,
			config->drkf_params.offset);
		isppreview_set_darkaddr(config->drkf_params.addr);
		isppreview_config_darklineoffset(config->drkf_params.offset);
		isppreview_enable_drkframe(1);
	} else
		isppreview_enable_drkframe(0);

	if (config->features & PREV_LENS_SHADING) {
		isppreview_config_drkf_shadcomp(config->lens_shading_shift);
		isppreview_enable_shadcomp(1);
	} else
		isppreview_enable_shadcomp(0);

	dev_dbg(prev_dev, "%s: Exit\n", __func__);
	return 0;
}

/**
 * prev_validate_params - Validate configuration parameters for Preview Wrapper
 * @params: Structure containing configuration parameters
 *
 * Validate configuration parameters for Preview Wrapper
 *
 * Returns 0 if successful, or -EINVAL if a parameter value is invalid.
 **/
static int prev_validate_params(struct prev_params *params)
{
	if (!params) {
		dev_err(prev_dev, "%s: invalid argument\n", __func__);
		goto err_einval;
	}

	if ((params->features & PREV_AVERAGER) == PREV_AVERAGER) {
		if ((params->average != NO_AVE)
				&& (params->average != AVE_2_PIX)
				&& (params->average != AVE_4_PIX)
				&& (params->average != AVE_8_PIX)) {
			dev_err(prev_dev, "%s: wrong pix average\n", __func__);
			goto err_einval;
		} else if (((params->average == AVE_2_PIX)
				&& (params->size_params.hsize % 2))
				|| ((params->average == AVE_4_PIX)
				&& (params->size_params.hsize % 4))
				|| ((params->average == AVE_8_PIX)
				&& (params->size_params.hsize % 8))) {
			dev_err(prev_dev, "%s: wrong pix average for input"
				" size\n", __func__);
			goto err_einval;
		}
	}

	if ((params->size_params.pixsize != PREV_INWIDTH_8BIT)
					&& (params->size_params.pixsize
					!= PREV_INWIDTH_10BIT)) {
		dev_err(prev_dev, "%s: wrong pixsize\n", __func__);
		goto err_einval;
	}

	if (params->size_params.hsize > MAX_IMAGE_WIDTH
					|| params->size_params.hsize < 0) {
		dev_err(prev_dev, "%s: wrong hsize\n", __func__);
		goto err_einval;
	}

	if (params->size_params.hsize % 32) {
		dev_err(prev_dev, "%s: width must be multiple of"
			" 64 bytes\n", __func__);
		goto err_einval;
	}

	if ((params->pix_fmt != YCPOS_YCrYCb)
			&& (YCPOS_YCbYCr != params->pix_fmt)
			&& (YCPOS_CbYCrY != params->pix_fmt)
			&& (YCPOS_CrYCbY != params->pix_fmt)) {
		dev_err(prev_dev, "%s: wrong pix_fmt\n", __func__);
		goto err_einval;
	}

	if ((params->features & PREV_DARK_FRAME_SUBTRACT)
			&& (params->features & PREV_DARK_FRAME_CAPTURE)) {
		dev_err(prev_dev, "%s: DARK FRAME CAPTURE and SUBSTRACT "
			"cannot be enabled at same time\n", __func__);
		goto err_einval;
	}

	if ((params->size_params.in_pitch <= 0)
			|| (params->size_params.in_pitch % 32)) {
		dev_err(prev_dev, "%s: invalid input pitch\n", __func__);
		goto err_einval;
	}

	if ((params->size_params.out_pitch <= 0)
			|| (params->size_params.out_pitch % 32)) {
		dev_err(prev_dev, "%s: invalid output pitch\n", __func__);
		goto err_einval;
	}

	return 0;
err_einval:
	return -EINVAL;
}

/**
 * preview_isr - Callback from ISP driver for ISP Preview Interrupt
 * @status: ISP IRQ0STATUS register value
 * @arg1: Structure containing ISP preview wrapper global information
 * @arg2: Currently not used
 **/
static void prev_isr(unsigned long status, isp_vbq_callback_ptr arg1,
								void *arg2)
{
	struct prev_device *device = (struct prev_device *)arg1;

	if ((status & PREV_DONE) != PREV_DONE)
		return;

	if (device)
		complete(&device->wfc);
}

/*
 * Set shared ports for using dark frame (lens shading)
 */
static void prev_set_isp_ctrl(u16 mode)
{
	u32 val;

	val = isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);

	isp_ctrl = val;

	/* Read port used by preview module data read */
	val &= ~ISP_CTRL_SBL_SHARED_RPORTA;

	/* Read port used by preview module dark frame read */
	if (mode & (PREV_DARK_FRAME_SUBTRACT | PREV_LENS_SHADING))
		val &= ~ISP_CTRL_SBL_SHARED_RPORTB;

	BIT_SET(val, SBL_RD_RAM_EN, 0x1, 0x1);

	/* write ISP CTRL register */
	isp_reg_writel(val, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);

	prv_wsdr_addr = isp_reg_readl(OMAP3_ISP_IOMEM_PREV, ISPPRV_WSDR_ADDR);
}

/*
 * Set old isp shared port configuration
 */
static void prev_unset_isp_ctrl(void)
{
	u32 val;

	val = isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);

	if (isp_ctrl & ISP_CTRL_SBL_SHARED_RPORTB)
		val |= ISP_CTRL_SBL_SHARED_RPORTB;

	if (isp_ctrl & ISP_CTRL_SBL_SHARED_RPORTA)
		val |= ISP_CTRL_SBL_SHARED_RPORTA;

	if (isp_ctrl & (1 << SBL_RD_RAM_EN))
		val &= ~(1 << SBL_RD_RAM_EN);

	/* write ISP CTRL register */
	isp_reg_writel(val, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);

	/* disable dark frame and shading compensation */
	isppreview_enable_drkframe(0);
	isppreview_enable_shadcomp(0);

	/* Set output and input adresses to 0 */
	isppreview_set_outaddr(prv_wsdr_addr);
}

/**
 * prev_config_size - Set input width and height in previewer registers
 * @input_w: input width
 * @inout_h: input height
 *
 * Returns 0 if successful, or -EINVAL if the sent parameters are invalid.
 **/
static void prev_config_size(u32 input_w, u32 input_h)
{
	isp_reg_writel((0 << ISPPRV_HORZ_INFO_SPH_SHIFT) | (input_w - 1),
		OMAP3_ISP_IOMEM_PREV, ISPPRV_HORZ_INFO);

	isp_reg_writel((0 << ISPPRV_VERT_INFO_SLV_SHIFT) | (input_h - 1),
		OMAP3_ISP_IOMEM_PREV, ISPPRV_VERT_INFO);

	isp_reg_writel((ISPPRV_AVE_EVENDIST_2 << ISPPRV_AVE_EVENDIST_SHIFT) |
		(ISPPRV_AVE_ODDDIST_2 << ISPPRV_AVE_ODDDIST_SHIFT),
		OMAP3_ISP_IOMEM_PREV, ISPPRV_AVE);
}

/**
 * prev_negotiate_output_size - Calculate previewer engine output size
 * @device: Structure containing ISP preview wrapper global information
 * @out_hsize: Return horizontal size
 * @out_vsize: Return vertical size
 *
 * Returns 0 if successful, or -EINVAL if the sent parameters are invalid.
 **/
static int prev_negotiate_output_size(struct prev_device *prvdev,
				u32 *out_hsize, u32 *out_vsize)
{
	int bpp, ret, outh, outv;

	if (prvdev->params->size_params.pixsize == PREV_INWIDTH_8BIT)
		bpp = 1;
	else
		bpp = 2;

	ret = isppreview_config_datapath(PRV_RAW_MEM, PREVIEW_MEM);
	if (ret) {
		dev_err(prev_dev, "%s: ERROR while configure isp "
			"preview datapath!\n", __func__);
		return ret;
	}

	ret = isppreview_try_size(prvdev->params->size_params.hsize,
			prvdev->params->size_params.vsize, &outh, &outv);
	if (ret) {
		dev_err(prev_dev, "%s: ERROR while try isp preview size!\n",
			__func__);
		return ret;
	}

	dev_dbg(prev_dev, "%s: try size %dx%d -> %dx%d\n", __func__,
		prvdev->params->size_params.hsize,
		prvdev->params->size_params.vsize, outh, outv);

	dev_dbg(prev_dev, "%s: out_pitch %d, output width %d, out_hsize %d\n",
		__func__, prvdev->params->size_params.out_pitch,
		prvdev->params->size_params.out_pitch / bpp, outh);

	if (outh > (prvdev->params->size_params.out_pitch / bpp))
		outh = prvdev->params->size_params.out_pitch / bpp;

	*out_hsize = outh;
	*out_vsize = outv;
	return 0;
}

/**
 * prev_do_preview - Performs the Preview process
 * @device: Structure containing ISP preview wrapper global information
 *
 * Returns 0 if successful, or -EINVAL if the sent parameters are invalid.
 **/
static int prev_do_preview(struct prev_device *device)
{
	u32 out_hsize, out_vsize, out_line_offset, in_line_offset;
	int ret = 0, bpp;

	dev_dbg(prev_dev, "%s: Enter\n", __func__);

	if (!device) {
		dev_err(prev_dev, "%s: invalid argument\n", __func__);
		return -EINVAL;
	}

	prev_set_isp_ctrl(device->params->features);

	if (device->params->size_params.pixsize == PREV_INWIDTH_8BIT)
		bpp = 1;
	else
		bpp = 2;

	out_hsize = device->out_hsize;
	out_vsize = device->out_vsize;

	in_line_offset = device->params->size_params.hsize * bpp;

	ret = isppreview_config_inlineoffset(in_line_offset);
	if (ret)
		goto out;

	dev_dbg(prev_dev, "%s: out_pitch %d, output width %d, out_hsize %d, "
						"out_vsize %d\n", __func__,
		device->params->size_params.out_pitch,
		device->params->size_params.out_pitch / bpp,
		out_hsize, out_vsize);

	out_line_offset = (out_hsize * bpp) & PREV_32BYTES_ALIGN_MASK;

	ret = isppreview_config_outlineoffset(out_line_offset);
	if (ret)
		goto out;

	prev_config_size(device->params->size_params.hsize,
					device->params->size_params.vsize);

	device->params->drkf_params.addr = device->isp_addr_lsc;

	prev_hw_setup(device->params);

	ret = isppreview_set_inaddr(device->isp_addr_read);
	if (ret)
		goto out;

	ret = isppreview_set_outaddr(device->isp_addr_read);
	if (ret)
		goto out;

	ret = isp_set_callback(CBK_PREV_DONE, prev_isr, (void *) device,
							(void *) NULL);
	if (ret) {
		dev_err(prev_dev,
			"%s: setting previewer callback failed\n", __func__);
		goto out;
	}

	/* Make sure we don't wait for any yuv frames */
	isp_set_wait_yuv(0);

	isppreview_enable(1);

	wait_for_completion_interruptible(&device->wfc);

	isppreview_enable(0);

	ret = isp_unset_callback(CBK_PREV_DONE);

	prev_unset_isp_ctrl();

	dev_dbg(prev_dev, "%s: Exit\n", __func__);
out:
	return ret;
}

/**
 * previewer_vbq_release - Videobuffer queue release
 * @q: Structure containing the videobuffer queue.
 * @vb: Structure containing the videobuffer used for previewer processing.
 **/
static void previewer_vbq_release(struct videobuf_queue *q,
						struct videobuf_buffer *vb)
{
	struct prev_fh *fh = q->priv_data;
	struct prev_device *device = fh->device;

	dev_dbg(prev_dev, "%s: Enter\n", __func__);

	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		ispmmu_vunmap(device->isp_addr_read);
		device->isp_addr_read = 0;
		spin_lock(&device->inout_vbq_lock);
		vb->state = VIDEOBUF_NEEDS_INIT;
		spin_unlock(&device->inout_vbq_lock);
	} else if (q->type == V4L2_BUF_TYPE_PRIVATE) {
		ispmmu_vunmap(device->isp_addr_lsc);
		device->isp_addr_lsc = 0;
		spin_lock(&device->lsc_vbq_lock);
		vb->state = VIDEOBUF_NEEDS_INIT;
		spin_unlock(&device->lsc_vbq_lock);
	}

	if (vb->memory != V4L2_MEMORY_MMAP) {
		videobuf_dma_unmap(q, videobuf_to_dma(vb));
		videobuf_dma_free(videobuf_to_dma(vb));
	}

	dev_dbg(prev_dev, "%s: Exit\n", __func__);
}

/**
 * previewer_vbq_setup - Sets up the videobuffer size and validates count.
 * @q: Structure containing the videobuffer queue.
 * @cnt: Number of buffers requested
 * @size: Size in bytes of the buffer used for previewing
 *
 * Always returns 0.
 **/
static int previewer_vbq_setup(struct videobuf_queue *q,
							unsigned int *cnt,
							unsigned int *size)
{
	struct prev_fh *fh = q->priv_data;
	struct prev_device *device = fh->device;
	u32 bpp = 1;

	dev_dbg(prev_dev, "%s: Enter\n", __func__);

	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		spin_lock(&device->inout_vbq_lock);

		if (*cnt <= 0)
			*cnt = 1;

		if (*cnt > VIDEO_MAX_FRAME)
			*cnt = VIDEO_MAX_FRAME;

		if (!device->params->size_params.hsize ||
			!device->params->size_params.vsize) {
			dev_err(prev_dev, "%s: Can't setup input/output "
				"buffer sizes\n", __func__);
			spin_unlock(&device->inout_vbq_lock);
			return -EINVAL;
		}

		if (device->params->size_params.pixsize == PREV_INWIDTH_10BIT)
			bpp = 2;

		*size = prev_bufsize = bpp * device->params->size_params.hsize *
					device->params->size_params.vsize;
		spin_unlock(&device->inout_vbq_lock);
	} else if (q->type == V4L2_BUF_TYPE_PRIVATE) {
		spin_lock(&device->lsc_vbq_lock);
		if (*cnt <= 0)
			*cnt = 1;

		if (*cnt > 1)
			*cnt = 1;

		if (!device->params->size_params.hsize ||
			!device->params->size_params.vsize) {
			dev_err(prev_dev, "%s: Can't setup lsc buffer size\n",
				 __func__);
			spin_unlock(&device->lsc_vbq_lock);
			return -EINVAL;
		}

		/* upsampled lsc table size - for now bpp = 2 */
		bpp = 2;
		*size = lsc_bufsize = bpp * device->params->size_params.hsize *
					device->params->size_params.vsize;

		spin_unlock(&device->lsc_vbq_lock);
	} else {
		return -EINVAL;
	}

	dev_dbg(prev_dev, "%s: Exit\n", __func__);
	return 0;
}

/**
 * previewer_vbq_prepare - Videobuffer is prepared and mmapped.
 * @q: Structure containing the videobuffer queue.
 * @vb: Structure containing the videobuffer used for previewer processing.
 * @field: Type of field to set in videobuffer device.
 *
 * Returns 0 if successful, or -EINVAL if buffer couldn't get allocated, or
 * -EIO if the ISP MMU mapping fails
 **/
static int previewer_vbq_prepare(struct videobuf_queue *q,
						struct videobuf_buffer *vb,
						enum v4l2_field field)
{
	struct prev_fh *fh = q->priv_data;
	struct prev_device *device = fh->device;
	int err = -EINVAL;
	unsigned int isp_addr;
	struct videobuf_dmabuf *dma = videobuf_to_dma(vb);

	dev_dbg(prev_dev, "%s: Enter\n", __func__);

	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		spin_lock(&device->inout_vbq_lock);

		if (vb->baddr) {
			vb->size = prev_bufsize;
			vb->bsize = prev_bufsize;
			dev_dbg(prev_dev, "%s: bsize = %d\n",
				__func__, vb->bsize);
		} else {
			spin_unlock(&device->inout_vbq_lock);
			dev_err(prev_dev, "%s: No user buffer allocated\n",
				__func__);
			goto out;
		}

		vb->width = device->params->size_params.hsize;
		vb->height = device->params->size_params.vsize;
		vb->field = field;
		spin_unlock(&device->inout_vbq_lock);

		if (vb->state == VIDEOBUF_NEEDS_INIT) {
			dev_dbg(prev_dev, "%s: baddr = %08x\n",
				__func__, (int)vb->baddr);
			err = videobuf_iolock(q, vb, NULL);
			if (!err) {
				isp_addr = ispmmu_vmap(dma->sglist,
						dma->sglen);

				if (!isp_addr) {
					err = -EIO;
				} else {
					device->isp_addr_read = isp_addr;
					dev_dbg(prev_dev, "%s: isp_addr_read "
						"= %08x\n",
						__func__, isp_addr);
				}
			}
		}

		if (!err) {
			vb->state = VIDEOBUF_PREPARED;
			flush_cache_user_range(NULL, vb->baddr, (vb->baddr +
								vb->bsize));
		} else {
			previewer_vbq_release(q, vb);
		}

	} else if (q->type == V4L2_BUF_TYPE_PRIVATE) {

		spin_lock(&device->lsc_vbq_lock);

		if (vb->baddr) {
			vb->size = lsc_bufsize;
			vb->bsize = lsc_bufsize;
			dev_dbg(prev_dev, "%s: bsize = %d\n",
				__func__, vb->bsize);
		} else {
			spin_unlock(&device->lsc_vbq_lock);
			dev_err(prev_dev, "%s: No user buffer allocated\n",
				__func__);
			goto out;
		}

		vb->width = device->params->size_params.hsize;
		vb->height = device->params->size_params.vsize;
		vb->field = field;
		spin_unlock(&device->lsc_vbq_lock);

		if (vb->state == VIDEOBUF_NEEDS_INIT) {
			dev_dbg(prev_dev, "%s: baddr = %08x\n",
				__func__, (int)vb->baddr);
			err = videobuf_iolock(q, vb, NULL);
			if (!err) {
				isp_addr = ispmmu_vmap(dma->sglist,
								dma->sglen);
				if (!isp_addr) {
					err = -EIO;
				} else {
					device->isp_addr_lsc = isp_addr;
					dev_dbg(prev_dev, "%s: isp_addr_lsc"
						"= %08x\n",
						__func__, isp_addr);
				}
			}
		}

		if (!err) {
			vb->state = VIDEOBUF_PREPARED;
			flush_cache_user_range(NULL, vb->baddr, (vb->baddr +
								vb->bsize));
		} else {
			previewer_vbq_release(q, vb);
		}

	} else {
		return -EINVAL;
	}

	dev_dbg(prev_dev, "%s: Exit\n", __func__);
out:
	return err;
}

static void previewer_vbq_queue(struct videobuf_queue *q,
						struct videobuf_buffer *vb)
{
	return;
}

/**
 * previewer_open - Initializes and opens the Preview Wrapper
 * @inode: Inode structure associated with the Preview Wrapper
 * @filp: File structure associated with the Preview Wrapper
 *
 * Returns 0 if successful, -EACCES if its unable to initialize default config,
 * -EBUSY if its already opened or the ISP module is not available, or -ENOMEM
 * if its unable to allocate the device in kernel space memory.
 **/
static int previewer_open(struct inode *inode, struct file *filp)
{
	struct prev_device *device = prevdevice;
	struct prev_params *config = isppreview_get_config();
	struct prev_fh *fh;
	int ret = 0;

	if (config == NULL) {
		dev_err(prev_dev, "%s: Unable to get default config "
			"from isp preview\n", __func__);
		return -EACCES;
	}

	if (mutex_lock_interruptible(&device->prevwrap_mutex))
		return -EINTR;

	if (device->opened || (filp->f_flags & O_NONBLOCK)) {
		dev_err(prev_dev, "%s: Device is already opened\n", __func__);
		ret = -EBUSY;
		goto err_open_fh;
	}

	fh = kzalloc(sizeof(struct prev_fh), GFP_KERNEL);
	if (NULL == fh) {
		ret = -ENOMEM;
		goto err_open_fh;
	}

	ret = isp_get();
	if (ret < 0) {
		dev_err(prev_dev, "%s: Can't acquire isp core\n", __func__);
		goto err_isp;
	}

	ret = isppreview_request();
	if (ret < 0) {
		dev_err(prev_dev, "%s: Can't acquire isp preview\n", __func__);
		goto err_prev;
	}

	device->params = config;
	device->opened = true;

	filp->private_data = fh;
	fh->inout_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fh->lsc_type = V4L2_BUF_TYPE_PRIVATE;
	fh->device = device;

	videobuf_queue_sg_init(&fh->inout_vbq, &device->vbq_ops, NULL,
			&device->inout_vbq_lock, fh->inout_type,
			V4L2_FIELD_NONE,
			sizeof(struct videobuf_buffer), fh);

	videobuf_queue_sg_init(&fh->lsc_vbq, &device->vbq_ops, NULL,
			&device->lsc_vbq_lock, fh->lsc_type,
			V4L2_FIELD_NONE,
			sizeof(struct videobuf_buffer), fh);

	init_completion(&device->wfc);
	device->configured = false;
	mutex_unlock(&device->prevwrap_mutex);
	return 0;

err_prev:
	isp_put();
err_isp:
	kfree(fh);
err_open_fh:
	mutex_unlock(&device->prevwrap_mutex);
	return ret;
}

/**
 * previewer_release - Releases Preview Wrapper and frees up allocated memory
 * @inode: Inode structure associated with the Preview Wrapper
 * @filp: File structure associated with the Preview Wrapper
 *
 * Always returns 0.
 **/
static int previewer_release(struct inode *inode, struct file *filp)
{
	struct prev_fh *fh = filp->private_data;
	struct prev_device *device = fh->device;
	struct videobuf_queue *q1 = &fh->inout_vbq;
	struct videobuf_queue *q2 = &fh->lsc_vbq;

	if (mutex_lock_interruptible(&device->prevwrap_mutex))
		return -EINTR;
	device->opened = false;
	device->params = NULL;
	videobuf_mmap_free(q1);
	videobuf_mmap_free(q2);
	videobuf_queue_cancel(q1);
	videobuf_queue_cancel(q2);
	isppreview_free();
	isp_put();
	prev_bufsize = 0;
	lsc_bufsize = 0;
	filp->private_data = NULL;
	mutex_unlock(&device->prevwrap_mutex);
	kfree(fh);
	dev_dbg(prev_dev, "%s: Exit\n", __func__);
	return 0;
}

/**
 * previewer_mmap - Memory maps the Preview Wrapper module.
 * @file: File structure associated with the Preview Wrapper
 * @vma: Virtual memory area structure.
 *
 * Returns 0 if successful, or returned value by the videobuf_mmap_mapper()
 * function.
 **/
static int previewer_mmap(struct file *file, struct vm_area_struct *vma)
{
	return -EINVAL;
}

/**
 * prev_copy_params - Copy usermode params to kernel.
 * @usr_params: Pointer to usermode structure
 * @isp_params: Pointer to kernel structure
 *
 * Returns 0 if successful, or negative on fail
 **/
static int prev_copy_params(struct prev_params *usr_params,
				struct prev_params *isp_params)
{
	isp_params->features = usr_params->features;
	isp_params->pix_fmt = usr_params->pix_fmt;
	isp_params->cfa.cfafmt = usr_params->cfa.cfafmt;
	isp_params->cfa.cfa_gradthrs_vert = usr_params->cfa.cfa_gradthrs_vert;
	isp_params->cfa.cfa_gradthrs_horz = usr_params->cfa.cfa_gradthrs_horz;

	if (usr_params->cfa.cfa_table && isp_params->cfa.cfa_table) {
		if (copy_from_user(isp_params->cfa.cfa_table,
				usr_params->cfa.cfa_table, ISPPRV_CFA_TBL_SIZE))
			return -EFAULT;
	} else {
		dev_warn(prev_dev,
			"%s: invalid cfa table pointer (usr: %08x,"
			" krn: %08x)\n", __func__,
			(int)usr_params->cfa.cfa_table,
			(int)isp_params->cfa.cfa_table);
	}

	isp_params->csup = usr_params->csup;

	if (usr_params->ytable && isp_params->ytable) {
		if (copy_from_user(isp_params->ytable,
				usr_params->ytable, ISPPRV_YENH_TBL_SIZE))
			return -EFAULT;
	} else {
		dev_warn(prev_dev,
			"%s: invalid ytable pointer (usr: %08x, krn: %08x)\n",
			__func__,
			(int)usr_params->ytable, (int)isp_params->ytable);
	}

	isp_params->nf = usr_params->nf;
	isp_params->dcor = usr_params->dcor;

	if (usr_params->gtable.redtable && isp_params->gtable.redtable) {
		if (copy_from_user(isp_params->gtable.redtable,
				usr_params->gtable.redtable,
				ISPPRV_GAMMA_TBL_SIZE))
			return -EFAULT;
	} else {
		dev_warn(prev_dev,
			"%s: invalid gtable red pointer (usr: %08x, "
			"krn: %08x)\n", __func__,
			(int)usr_params->gtable.redtable,
			(int)isp_params->gtable.redtable);
	}

	if (usr_params->gtable.greentable && isp_params->gtable.greentable) {
		if (copy_from_user(isp_params->gtable.greentable,
				usr_params->gtable.greentable,
				ISPPRV_GAMMA_TBL_SIZE))
			return -EFAULT;
	} else {
		dev_warn(prev_dev,
			"%s: invalid gtable green pointer (usr: %08x,"
			"krn: %08x)\n", __func__,
			(int)usr_params->gtable.greentable,
			(int)isp_params->gtable.greentable);
	}

	if (usr_params->gtable.bluetable && isp_params->gtable.bluetable) {
		if (copy_from_user(isp_params->gtable.bluetable,
				usr_params->gtable.bluetable,
				ISPPRV_GAMMA_TBL_SIZE))
			return -EFAULT;
	} else {
		dev_warn(prev_dev,
			"%s: invalid gtable blue pointer (usr: %08x,"
			" krn: %08x)\n", __func__,
			(int)usr_params->gtable.bluetable,
			(int)isp_params->gtable.bluetable);
	}

	isp_params->wbal = usr_params->wbal;
	isp_params->blk_adj = usr_params->blk_adj;
	isp_params->rgb2rgb = usr_params->rgb2rgb;
	isp_params->rgb2ycbcr = usr_params->rgb2ycbcr;
	isp_params->hmf_params = usr_params->hmf_params;
	isp_params->size_params = usr_params->size_params;
	isp_params->drkf_params = usr_params->drkf_params;
	isp_params->lens_shading_shift = usr_params->lens_shading_shift;
	isp_params->average = usr_params->average;
	isp_params->contrast = usr_params->contrast;
	isp_params->brightness = usr_params->brightness;

	return 0;
}

static int prev_get_params(struct prev_params __user *usr_params,
			   struct prev_device *device)
{
	struct prev_params usr_params_new;
	u32 *cfa_table; /* params.cfa.cfa_table */
	u32 *ytable; /* params.ytable */
	u32 *redtable; /* params.gtable.redtable */
	u32 *greentable; /* params.gtable.greentable */
	u32 *bluetable; /* params.gtable.bluetable */

	if (copy_from_user(&usr_params_new, usr_params,
			   sizeof(struct prev_params)))
		return -EFAULT;

	/* Backup user pointers */
	cfa_table = usr_params_new.cfa.cfa_table;
	ytable = usr_params_new.ytable;
	redtable = usr_params_new.gtable.redtable;
	greentable = usr_params_new.gtable.greentable;
	bluetable = usr_params_new.gtable.bluetable;

	memcpy(&usr_params_new, device->params, sizeof(struct prev_params));

	/* Restore user pointers */
	usr_params_new.cfa.cfa_table = cfa_table;
	usr_params_new.ytable = ytable;
	usr_params_new.gtable.redtable = redtable;
	usr_params_new.gtable.greentable = greentable;
	usr_params_new.gtable.bluetable = bluetable;

	if (copy_to_user(usr_params, &usr_params_new,
			 sizeof(struct prev_params)))
		return -EFAULT;
	return 0;
}

/**
 * previewer_ioctl - I/O control function for Preview Wrapper
 * @inode: Inode structure associated with the Preview Wrapper.
 * @file: File structure associated with the Preview Wrapper.
 * @cmd: Type of command to execute.
 * @arg: Argument to send to requested command.
 *
 * Returns 0 if successful, -1 if bad command passed or access is denied,
 * -EFAULT if copy_from_user() or copy_to_user() fails, -EINVAL if parameter
 * validation fails or parameter structure is not present
 **/
static int previewer_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct prev_params params;
	struct prev_fh *fh = file->private_data;
	struct prev_device *device = fh->device;
	struct v4l2_buffer b;
	struct v4l2_requestbuffers req;

	dev_dbg(prev_dev, "%s: Enter\n", __func__);

	if ((_IOC_TYPE(cmd) != PREV_IOC_BASE)
					|| (_IOC_NR(cmd) > PREV_IOC_MAXNR)) {
		dev_err(prev_dev, "%s: bad command value\n", __func__);
		goto err_minusone;
	}

	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (ret) {
		dev_err(prev_dev, "%s: access denied\n", __func__);
		goto err_minusone;
	}

	switch (cmd) {
	case PREV_REQBUF:
		if (copy_from_user(&req, (struct v4l2_requestbuffers *)arg,
					sizeof(struct v4l2_requestbuffers)))
			return -EFAULT;

		if (mutex_lock_interruptible(&device->prevwrap_mutex))
			goto err_eintr;

		if (req.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			ret = videobuf_reqbufs(&fh->inout_vbq, &req);
		else if (req.type == V4L2_BUF_TYPE_PRIVATE)
			ret = videobuf_reqbufs(&fh->lsc_vbq, &req);
		else
			ret = -EINVAL;

		if (!ret && copy_to_user((struct v4l2_requestbuffers *)arg,
				&req, sizeof(struct v4l2_requestbuffers)))
			ret = -EFAULT;

		mutex_unlock(&device->prevwrap_mutex);
		break;

	case PREV_QUERYBUF:
		if (copy_from_user(&b, (struct v4l2_buffer *)arg,
					sizeof(struct v4l2_buffer)))
			return -EFAULT;

		if (mutex_lock_interruptible(&device->prevwrap_mutex))
			goto err_eintr;

		if (b.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			ret = videobuf_querybuf(&fh->inout_vbq, &b);
		else if (b.type == V4L2_BUF_TYPE_PRIVATE)
			ret = videobuf_querybuf(&fh->lsc_vbq, &b);
		else
			ret = -EINVAL;

		if (!ret && copy_to_user((struct v4l2_buffer *)arg, &b,
					sizeof(struct v4l2_buffer)))
			ret = -EFAULT;

		mutex_unlock(&device->prevwrap_mutex);
		break;

	case PREV_QUEUEBUF:
		if (copy_from_user(&b, (struct v4l2_buffer *)arg,
					sizeof(struct v4l2_buffer)))
			return -EFAULT;

		if (mutex_lock_interruptible(&device->prevwrap_mutex))
			goto err_eintr;

		if (b.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			ret = videobuf_qbuf(&fh->inout_vbq, &b);
		else if (b.type == V4L2_BUF_TYPE_PRIVATE)
			ret = videobuf_qbuf(&fh->lsc_vbq, &b);
		else
			ret = -EINVAL;

		mutex_unlock(&device->prevwrap_mutex);
		break;

	case PREV_SET_PARAM:
		if (mutex_lock_interruptible(&device->prevwrap_mutex))
			goto err_eintr;

		if (copy_from_user(&params, (struct prev_params *)arg,
						sizeof(struct prev_params))) {
			mutex_unlock(&device->prevwrap_mutex);
			return -EFAULT;
		}

		memcpy(&isppreview_tmp, device->params,
			sizeof(isppreview_tmp));

		ret = prev_copy_params(&params, device->params);
		if (ret) {
			memcpy(device->params, &isppreview_tmp,
				sizeof(isppreview_tmp));
			dev_err(prev_dev, "%s: copy parameters fail\n",
				__func__);
			goto out;
		}

		ret = prev_validate_params(device->params);
		if (ret < 0) {
			memcpy(device->params, &isppreview_tmp,
				sizeof(isppreview_tmp));
			dev_err(prev_dev, "%s: validating parameters fail!\n",
				 __func__);
			mutex_unlock(&device->prevwrap_mutex);
			goto out;
		}

		device->configured = true;

		ret = prev_negotiate_output_size(device, &device->out_hsize,
					&device->out_vsize);
		if (ret) {
			dev_err(prev_dev, "%s: negotiate output size fail\n",
				 __func__);
			ret = -EINVAL;
			goto out;
		}
		dev_dbg(prev_dev, "%s: out_hsize %d, out_vsize %d\n", __func__,
			device->out_hsize, device->out_vsize);

		mutex_unlock(&device->prevwrap_mutex);
		break;

	case PREV_GET_PARAM:
		prev_get_params((struct prev_params *)arg, device);
		break;

	case PREV_GET_STATUS:
	{
		struct prev_status status;

		ret = prev_get_status(&status);
		if (ret)
			break;

		if (copy_to_user((struct prev_status *)arg, &status,
						sizeof(struct prev_status)))
			ret = -EFAULT;
		break;
	}

	case PREV_PREVIEW:
		if (mutex_lock_interruptible(&device->prevwrap_mutex))
			goto err_eintr;
		ret = prev_do_preview(device);
		mutex_unlock(&device->prevwrap_mutex);
		break;

	case PREV_GET_CROPSIZE:
	{
		struct prev_cropsize outputsize;

		if (device->configured == false) {
			ret = -EPERM;
			dev_err(prev_dev, "%s: not configured yet\n",
				__func__);
			break;
		}

		ret = prev_calculate_crop(device, &outputsize);
		if (ret)
			break;

		if (copy_to_user((struct prev_cropsize *)arg, &outputsize,
					sizeof(struct prev_cropsize)))
			ret = -EFAULT;
		break;
	}

	default:
		dev_err(prev_dev, "%s: invalid command value\n", __func__);
		ret = -EINVAL;
	}
out:
	return ret;
err_minusone:
	return -1;
err_eintr:
	return -EINTR;
}

/**
 * previewer_platform_release - Acts when Reference count is zero
 * @device: Structure containing ISP preview wrapper global information
 *
 * This is called when the reference count goes to zero
 **/
static void previewer_platform_release(struct device *device)
{
	dev_dbg(prev_dev, "%s: Enter\n", __func__);
}

static const struct file_operations prev_fops = {
	.owner = THIS_MODULE,
	.open = previewer_open,
	.release = previewer_release,
	.mmap = previewer_mmap,
	.ioctl = previewer_ioctl,
};

static struct platform_device omap_previewer_device = {
	.name = OMAP_PREV_NAME,
	.id = -1,
	.dev = {
		.release = previewer_platform_release,
	}
};

/**
 * previewer_probe - Checks for device presence
 * @pdev: Structure containing details of the current device.
 *
 * Always returns 0
 **/
static int previewer_probe(struct platform_device *pdev)
{
	return 0;
}

/**
 * previewer_remove - Handles the removal of the driver
 * @pdev: Structure containing details of the current device.
 *
 * Always returns 0.
 **/
static int previewer_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver omap_previewer_driver = {
	.probe = previewer_probe,
	.remove = previewer_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = OMAP_PREV_NAME,
	},
};

/**
 * omap_previewer_init - Initialization of Preview Wrapper
 *
 * Returns 0 if successful, -ENOMEM if could not allocate memory, -ENODEV if
 * could not register the wrapper as a character device, or other errors if the
 * device or driver can't register.
 **/
static int __init omap_previewer_init(void)
{
	int ret;
	struct prev_device *device;

	device = kzalloc(sizeof(struct prev_device), GFP_KERNEL);
	if (!device) {
		dev_err(prev_dev, OMAP_PREV_NAME ": could not allocate"
								" memory\n");
		return -ENOMEM;
	}
	prev_major = register_chrdev(0, OMAP_PREV_NAME, &prev_fops);

	if (prev_major < 0) {
		dev_err(prev_dev, OMAP_PREV_NAME ": Initialization "
			"failed. Could not register character device\n");
		ret = -ENODEV;
		goto fail1;
	}

	ret = platform_driver_register(&omap_previewer_driver);
	if (ret) {
		dev_err(prev_dev, OMAP_PREV_NAME
			": failed to register platform driver!\n");
		goto fail2;
	}
	ret = platform_device_register(&omap_previewer_device);
	if (ret) {
		dev_err(prev_dev, OMAP_PREV_NAME
			": failed to register platform device!\n");
		goto fail3;
	}

	prev_class = class_create(THIS_MODULE, OMAP_PREV_NAME);
	if (!prev_class)
		goto fail4;

	prev_dev = device_create(prev_class, prev_dev,
					MKDEV(prev_major, 0), NULL,
					OMAP_PREV_NAME);
	dev_info(prev_dev, OMAP_PREV_NAME ": Registered preview wrapper\n");
	device->opened = false;

	device->vbq_ops.buf_setup = previewer_vbq_setup;
	device->vbq_ops.buf_prepare = previewer_vbq_prepare;
	device->vbq_ops.buf_release = previewer_vbq_release;
	device->vbq_ops.buf_queue = previewer_vbq_queue;
	mutex_init(&device->prevwrap_mutex);
	spin_lock_init(&device->inout_vbq_lock);
	spin_lock_init(&device->lsc_vbq_lock);
	prevdevice = device;
	return 0;

fail4:
	platform_device_unregister(&omap_previewer_device);
fail3:
	platform_driver_unregister(&omap_previewer_driver);
fail2:
	unregister_chrdev(prev_major, OMAP_PREV_NAME);
fail1:
	kfree(device);
	return ret;
}

/**
 * omap_previewer_exit - Close of Preview Wrapper
 **/
static void __exit omap_previewer_exit(void)
{
	device_destroy(prev_class, MKDEV(prev_major, 0));
	class_destroy(prev_class);
	platform_device_unregister(&omap_previewer_device);
	platform_driver_unregister(&omap_previewer_driver);
	unregister_chrdev(prev_major, OMAP_PREV_NAME);

	kfree(prevdevice);
	prev_major = -1;
}

module_init(omap_previewer_init);
module_exit(omap_previewer_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP ISP Previewer");
MODULE_LICENSE("GPL");
