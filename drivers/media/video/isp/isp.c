/*
 * isp.c
 *
 * Driver Library for ISP Control module in TI's OMAP3 Camera ISP
 * ISP interface and IRQ related APIs are defined here.
 *
 * Copyright (C) 2009 Texas Instruments.
 * Copyright (C) 2009 Nokia.
 *
 * Contributors:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Mohit Jalori <mjalori@ti.com>
 * 	Sergio Aguirre <saaguirre@ti.com>
 * 	Sakari Ailus <sakari.ailus@nokia.com>
 * 	Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 *	Toni Leinonen <toni.leinonen@nokia.com>
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
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>

#include "isp.h"
#include "ispmmu.h"
#include "ispreg.h"
#include "ispccdc.h"
#include "isppreview.h"
#include "ispresizer.h"
#include "ispcsi2.h"
#include "isp_mem_process.h"
#if defined(CONFIG_VIDEO_OMAP3_HP3A)
 #include "hp3a.h"
#else
 #include "isph3a.h"
 #include "isphist.h"
 #include "isp_af.h"
#endif

#define USE_LSC_WORKAROUND

static struct isp_device *omap3isp;

static int isp_try_size(struct v4l2_pix_format *pix_input,
			struct v4l2_pix_format *pix_output);

static void isp_save_ctx(void);
static void isp_restore_ctx(void);
static void isp_buf_init(void);

/* List of image formats supported via OMAP ISP */
const static struct v4l2_fmtdesc isp_formats[] = {
	{
		.description = "UYVY, packed",
		.pixelformat = V4L2_PIX_FMT_UYVY,
	},
	{
		.description = "YUYV (YUV 4:2:2), packed",
		.pixelformat = V4L2_PIX_FMT_YUYV,
	},
	{
		.description = "Bayer10 (GrR/BGb)",
		.pixelformat = V4L2_PIX_FMT_SGRBG10,
	},
};

/* ISP Crop capabilities */
static struct v4l2_rect ispcroprect;
static struct v4l2_rect cur_rect;

/**
 * struct vcontrol - Video control structure.
 * @qc: V4L2 Query control structure.
 * @current_value: Current value of the control.
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
	{
		{
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Brightness",
			.minimum = ISPPRV_BRIGHT_LOW,
			.maximum = ISPPRV_BRIGHT_HIGH,
			.step = ISPPRV_BRIGHT_STEP,
			.default_value = ISPPRV_BRIGHT_DEF,
		},
		.current_value = ISPPRV_BRIGHT_DEF,
	},
	{
		{
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = ISPPRV_CONTRAST_LOW,
			.maximum = ISPPRV_CONTRAST_HIGH,
			.step = ISPPRV_CONTRAST_STEP,
			.default_value = ISPPRV_CONTRAST_DEF,
		},
		.current_value = ISPPRV_CONTRAST_DEF,
	},
	{
		{
			.id = V4L2_CID_COLORFX,
			.type = V4L2_CTRL_TYPE_MENU,
			.name = "Color Effects",
			.minimum = V4L2_COLORFX_NONE,
			.maximum = V4L2_COLORFX_SEPIA,
			.step = 1,
			.default_value = V4L2_COLORFX_NONE,
		},
		.current_value = V4L2_COLORFX_NONE,
	}
};

static struct v4l2_querymenu video_menu[] = {
	{
		.id = V4L2_CID_COLORFX,
		.index = 0,
		.name = "None",
	},
	{
		.id = V4L2_CID_COLORFX,
		.index = 1,
		.name = "B&W",
	},
	{
		.id = V4L2_CID_COLORFX,
		.index = 2,
		.name = "Sepia",
	},
};

struct isp_buf {
	dma_addr_t isp_addr;
	void (*complete)(struct videobuf_buffer *vb, void *priv);
	struct videobuf_buffer *vb;
	void *priv;
	u32 vb_state;
};

#define ISP_BUFS_IS_FULL(bufs)					\
	(((bufs)->queue + 1) % NUM_BUFS == (bufs)->done)
#define ISP_BUFS_IS_EMPTY(bufs)		((bufs)->queue == (bufs)->done)
#define ISP_BUFS_IS_LAST(bufs)					\
	((bufs)->queue == ((bufs)->done + 1) % NUM_BUFS)
#define ISP_BUFS_QUEUED(bufs)						\
	((((bufs)->done - (bufs)->queue + NUM_BUFS)) % NUM_BUFS)
#define ISP_BUF_DONE(bufs)		((bufs)->buf + (bufs)->done)
#define ISP_BUF_NEXT_DONE(bufs)				\
	((bufs)->buf + ((bufs)->done + 1) % NUM_BUFS)
#define ISP_BUF_QUEUE(bufs)		((bufs)->buf + (bufs)->queue)
#define ISP_BUF_MARK_DONE(bufs)				\
	(bufs)->done = ((bufs)->done + 1) % NUM_BUFS;
#define ISP_BUF_MARK_QUEUED(bufs)			\
	(bufs)->queue = ((bufs)->queue + 1) % NUM_BUFS;

struct isp_bufs {
	dma_addr_t isp_addr_capture[VIDEO_MAX_FRAME];
	spinlock_t lock;	/* For handling current buffer */
	/* queue full: (ispsg.queue + 1) % NUM_BUFS == ispsg.done
	   queue empty: ispsg.queue == ispsg.done */
	struct isp_buf buf[NUM_BUFS];
	/* Next slot to queue a buffer. */
	int queue;
	/* Buffer that is being processed. */
	int done;
	/* Skip this many frames before starting bayer capture */
	int wait_bayer_frame;
	/* Skip this many frames before starting yuv capture */
	int wait_yuv_frame;
};

/**
 * struct ispirq - Structure for containing callbacks to be called in ISP ISR.
 * @isp_callbk: Array which stores callback functions, indexed by the type of
 *              callback (8 possible types).
 * @isp_callbk_arg1: Pointer to array containing pointers to the first argument
 *                   to be passed to the requested callback function.
 * @isp_callbk_arg2: Pointer to array containing pointers to the second
 *                   argument to be passed to the requested callback function.
 *
 * This structure is used to contain all the callback functions related for
 * each callback type (CBK_CCDC_VD0, CBK_CCDC_VD1, CBK_PREV_DONE,
 * CBK_RESZ_DONE, CBK_MMU_ERR, CBK_H3A_AWB_DONE, CBK_HIST_DONE, CBK_HS_VS,
 * CBK_LSC_ISR).
 */
struct isp_irq {
	isp_callback_t isp_callbk[CBK_END];
	isp_vbq_callback_ptr isp_callbk_arg1[CBK_END];
	void *isp_callbk_arg2[CBK_END];
};

/**
 * struct ispmodule - Structure for storing ISP sub-module information.
 * @isp_pipeline: Bit mask for submodules enabled within the ISP.
 * @applyCrop: Flag to do a crop operation when video buffer queue ISR is done
 * @pix: Structure containing the format and layout of the output image.
 * @ccdc_input_width: ISP CCDC module input image width.
 * @ccdc_input_height: ISP CCDC module input image height.
 * @ccdc_output_width: ISP CCDC module output image width.
 * @ccdc_output_height: ISP CCDC module output image height.
 * @preview_input_width: ISP Preview module input image width.
 * @preview_input_height: ISP Preview module input image height.
 * @preview_output_width: ISP Preview module output image width.
 * @preview_output_height: ISP Preview module output image height.
 * @resizer_input_width: ISP Resizer module input image width.
 * @resizer_input_height: ISP Resizer module input image height.
 * @resizer_output_width: ISP Resizer module output image width.
 * @resizer_output_height: ISP Resizer module output image height.
 */
struct isp_module {
	unsigned int isp_pipeline;
	int applyCrop;
	struct v4l2_pix_format pix;
	unsigned int ccdc_input_width;
	unsigned int ccdc_input_height;
	unsigned int ccdc_output_width;
	unsigned int ccdc_output_height;
	unsigned int preview_input_width;
	unsigned int preview_input_height;
	unsigned int preview_output_width;
	unsigned int preview_output_height;
	unsigned int resizer_input_width;
	unsigned int resizer_input_height;
	unsigned int resizer_output_width;
	unsigned int resizer_output_height;
};

#define CCDC_CAPTURE(isp)					\
	((isp)->module.isp_pipeline == OMAP_ISP_CCDC)

#define CCDC_PREV_CAPTURE(isp)					\
	((isp)->module.isp_pipeline == (OMAP_ISP_CCDC | OMAP_ISP_PREVIEW))

#define CCDC_PREV_RESZ_CAPTURE(isp)					\
	((isp)->module.isp_pipeline == (OMAP_ISP_CCDC | \
					OMAP_ISP_PREVIEW | \
					OMAP_ISP_RESIZER))

/**
 * struct isp - Structure for storing ISP Control module information
 * @lock: Spinlock to sync between isr and processes.
 * @isp_mutex: Semaphore used to get access to the ISP.
 * @ref_count: Reference counter.
 * @cam_ick: Pointer to ISP Interface clock.
 * @cam_fck: Pointer to ISP Functional clock.
 *
 * This structure is used to store the OMAP ISP Control Information.
 */
static struct isp {
	spinlock_t lock;	/* For handling registered ISP callbacks */
	struct mutex isp_mutex;	/* For handling ref_count field */
	int ref_count;
	struct clk *cam_ick;
	struct clk *cam_mclk;
	struct clk *csi2_fck;
	u32 mclk_hz;
	u32 mclk_src_div;
	struct isp_interface_config *config;
	dma_addr_t tmp_buf;
	size_t tmp_buf_size;
	unsigned long tmp_buf_offset;
	struct isp_bufs bufs;
	struct isp_irq irq;
	struct isp_module module;
	enum isp_running running;
	int isp_lsc_workaround;
} isp_obj;

/* Structure for saving/restoring ISP module registers */
static struct isp_reg isp_reg_list[] = {
	{OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_GRESET_LENGTH, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_PSTRB_REPLAY, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_CTRL, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_CTRL, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_FRAME, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_PSTRB_DELAY, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_STRB_DELAY, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_SHUT_DELAY, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_PSTRB_LENGTH, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_STRB_LENGTH, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_SHUT_LENGTH, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF_SYSCONFIG, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF_IRQENABLE, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF0_CTRL, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF1_CTRL, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF0_START, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF1_START, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF0_END, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF1_END, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF0_WINDOWSIZE, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF1_WINDOWSIZE, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF0_THRESHOLD, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF1_THRESHOLD, 0},
	{0, ISP_TOK_TERM, 0}
};

u32 isp_reg_readl(enum isp_mem_resources isp_mmio_range, u32 reg_offset)
{
	return __raw_readl(omap3isp->mmio_base[isp_mmio_range] + reg_offset);
}
EXPORT_SYMBOL(isp_reg_readl);

void isp_reg_writel(u32 reg_value, enum isp_mem_resources isp_mmio_range,
		    u32 reg_offset)
{
	__raw_writel(reg_value,
		     omap3isp->mmio_base[isp_mmio_range] + reg_offset);
}
EXPORT_SYMBOL(isp_reg_writel);

void isp_flush(void)
{
	isp_reg_writel(0, OMAP3_ISP_IOMEM_MAIN, ISP_REVISION);
	isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_REVISION);
}

enum isp_running isp_state(void)
{
	return isp_obj.running;
}

int isp_lsc_workaround_enabled(void)
{
	return isp_obj.isp_lsc_workaround;
}
EXPORT_SYMBOL(isp_lsc_workaround_enabled);

/*
 *
 * V4L2 Handling
 *
 */

/**
 * find_vctrl - Returns the index of the ctrl array of the requested ctrl ID.
 * @id: Requested control ID.
 *
 * Returns 0 if successful, -EINVAL if not found, or -EDOM if its out of
 * domain.
 **/
static int find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;

	if (i < 0)
		i = -EINVAL;

	return i;
}

static int find_next_vctrl(int id)
{
	int i;
	u32 best = (u32)-1;

	for (i = 0; i < ARRAY_SIZE(video_control); i++) {
		if (video_control[i].qc.id > id &&
		    (best == (u32)-1 ||
		     video_control[i].qc.id <
		     video_control[best].qc.id)) {
			best = i;
		}
	}

	if (best == (u32)-1)
		return -EINVAL;

	return best;
}

/**
 * find_vmenu - Returns index of the menu array of the requested ctrl option.
 * @id: Requested control ID.
 * @index: Requested menu option index.
 *
 * Returns 0 if successful, -EINVAL if not found, or -EDOM if its out of
 * domain.
 **/
static int find_vmenu(int id, int index)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_menu) - 1); i >= 0; i--) {
		if (video_menu[i].id != id || video_menu[i].index != index)
			continue;
		return i;
	}

	return -EINVAL;
}

/**
 * isp_release_resources - Free ISP submodules
 **/
static void isp_release_resources(void)
{
	if (isp_obj.module.isp_pipeline & OMAP_ISP_CCDC)
		ispccdc_free();

	if (isp_obj.module.isp_pipeline & OMAP_ISP_PREVIEW)
		isppreview_free();

	if (isp_obj.module.isp_pipeline & OMAP_ISP_RESIZER)
		ispresizer_free();
	return;
}

static int isp_wait(int (*busy)(void), int wait_for_busy, int max_wait)
{
	int wait = 0;

	if (max_wait == 0)
		max_wait = 10000; /* 10 ms */

	while ((wait_for_busy && !busy())
	       || (!wait_for_busy && busy())) {
		rmb();
		udelay(1);
		wait++;
		if (wait > max_wait) {
			printk(KERN_ALERT "%s: wait is too much\n", __func__);
			return -EBUSY;
		}
	}
	DPRINTK_ISPCTRL(KERN_ALERT "%s: wait %d\n", __func__, wait);

	return 0;
}

static int ispccdc_sbl_wait_idle(int max_wait)
{
	return isp_wait(ispccdc_sbl_busy, 0, max_wait);
}

static void isp_enable_interrupts(void)
{
	isp_reg_writel(-1, OMAP3_ISP_IOMEM_MAIN,
			ISP_IRQ0STATUS);

	isp_reg_or(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
		   IRQ0ENABLE_HS_VS_IRQ |
		   IRQ0ENABLE_CCDC_VD0_IRQ);

	if (CCDC_PREV_CAPTURE(&isp_obj))
		isp_reg_or(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			   IRQ0ENABLE_PRV_DONE_IRQ);

	if (CCDC_PREV_RESZ_CAPTURE(&isp_obj))
		isp_reg_or(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			IRQ0ENABLE_PRV_DONE_IRQ |
			IRQ0ENABLE_RSZ_DONE_IRQ);

	return;
}

static void isp_disable_interrupts(void)
{
	isp_reg_and(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
		~(IRQ0ENABLE_HS_VS_IRQ |
		IRQ0ENABLE_CCDC_VD0_IRQ));

	if (CCDC_PREV_CAPTURE(&isp_obj))
		isp_reg_and(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
		~IRQ0ENABLE_PRV_DONE_IRQ);

	if (CCDC_PREV_RESZ_CAPTURE(&isp_obj))
		isp_reg_and(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
		~(IRQ0ENABLE_PRV_DONE_IRQ|IRQ0ENABLE_RSZ_DONE_IRQ));

	return;
}

/**
 * isp_set_callback - Sets the callback for the ISP module done events.
 * @type: Type of the event for which callback is requested.
 * @callback: Method to be called as callback in the ISR context.
 * @arg1: First argument to be passed when callback is called in ISR.
 * @arg2: Second argument to be passed when callback is called in ISR.
 *
 * This function sets a callback function for a done event in the ISP
 * module, and enables the corresponding interrupt.
 **/
int isp_set_callback(enum isp_callback_type type, isp_callback_t callback,
		     isp_vbq_callback_ptr arg1,
		     void *arg2)
{
	unsigned long irqflags = 0;

	if (callback == NULL) {
		DPRINTK_ISPCTRL("ISP_ERR : Null Callback\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_obj.lock, irqflags);
	isp_obj.irq.isp_callbk[type] = callback;
	isp_obj.irq.isp_callbk_arg1[type] = arg1;
	isp_obj.irq.isp_callbk_arg2[type] = arg2;
	spin_unlock_irqrestore(&isp_obj.lock, irqflags);

	switch (type) {
	case CBK_H3A_AWB_DONE:
		isp_reg_writel(IRQ0ENABLE_H3A_AWB_DONE_IRQ,
			       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
		isp_reg_or(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			   IRQ0ENABLE_H3A_AWB_DONE_IRQ);
		break;
	case CBK_H3A_AF_DONE:
		isp_reg_writel(IRQ0ENABLE_H3A_AF_DONE_IRQ,
			       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
		isp_reg_or(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			   IRQ0ENABLE_H3A_AF_DONE_IRQ);
		break;
	case CBK_HIST_DONE:
		isp_reg_writel(IRQ0ENABLE_HIST_DONE_IRQ,
			       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
		isp_reg_or(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			   IRQ0ENABLE_HIST_DONE_IRQ);
		break;
	case CBK_PREV_DONE:
		isp_reg_writel(IRQ0ENABLE_PRV_DONE_IRQ,
			       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
		isp_reg_or(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			   IRQ0ENABLE_PRV_DONE_IRQ);
		break;
	case CBK_RESZ_DONE:
		isp_reg_writel(IRQ0ENABLE_RSZ_DONE_IRQ,
			       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
		isp_reg_or(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			   IRQ0ENABLE_RSZ_DONE_IRQ);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(isp_set_callback);

/**
 * isp_unset_callback - Clears the callback for the ISP module done events.
 * @type: Type of the event for which callback to be cleared.
 *
 * This function clears a callback function for a done event in the ISP
 * module, and disables the corresponding interrupt.
 **/
int isp_unset_callback(enum isp_callback_type type)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&isp_obj.lock, irqflags);
	isp_obj.irq.isp_callbk[type] = NULL;
	isp_obj.irq.isp_callbk_arg1[type] = NULL;
	isp_obj.irq.isp_callbk_arg2[type] = NULL;
	spin_unlock_irqrestore(&isp_obj.lock, irqflags);

	switch (type) {
	case CBK_H3A_AWB_DONE:
		isp_reg_and(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			    ~IRQ0ENABLE_H3A_AWB_DONE_IRQ);
		break;
	case CBK_H3A_AF_DONE:
		isp_reg_and(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			    ~IRQ0ENABLE_H3A_AF_DONE_IRQ);
		break;
	case CBK_HIST_DONE:
		isp_reg_and(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			    ~IRQ0ENABLE_HIST_DONE_IRQ);
		break;
	case CBK_CSIA:
		isp_csi2_irq_set(0);
		break;
	case CBK_CSIB:
		isp_reg_writel(IRQ0ENABLE_CSIB_IRQ, OMAP3_ISP_IOMEM_MAIN,
			       ISP_IRQ0STATUS);
		isp_reg_or(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			   IRQ0ENABLE_CSIB_IRQ);
		break;
	case CBK_PREV_DONE:
		isp_reg_and(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			    ~IRQ0ENABLE_PRV_DONE_IRQ);
		break;
	case CBK_RESZ_DONE:
		isp_reg_and(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			    ~IRQ0ENABLE_RSZ_DONE_IRQ);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(isp_unset_callback);

/**
 * isp_set_xclk - Configures the specified cam_xclk to the desired frequency.
 * @xclk: Desired frequency of the clock in Hz.
 * @xclksel: XCLK to configure (0 = A, 1 = B).
 *
 * Configures the specified MCLK divisor in the ISP timing control register
 * (TCTRL_CTRL) to generate the desired xclk clock value.
 *
 * Divisor = isp_obj.mclk_hz / xclk
 *
 * Returns the final frequency that is actually being generated
 **/
u32 isp_set_xclk(u32 xclk, u8 xclksel)
{
	u32 divisor;
	u32 currentxclk;

	if (xclk >= isp_obj.mclk_hz) {
		divisor = ISPTCTRL_CTRL_DIV_BYPASS;
		currentxclk = isp_obj.mclk_hz;
	} else if (xclk >= 2) {
		divisor = isp_obj.mclk_hz / xclk;
		if (divisor >= ISPTCTRL_CTRL_DIV_BYPASS)
			divisor = ISPTCTRL_CTRL_DIV_BYPASS - 1;
		currentxclk = isp_obj.mclk_hz / divisor;
	} else {
		divisor = xclk;
		currentxclk = 0;
	}

	switch (xclksel) {
	case 0:
		isp_reg_and_or(OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_CTRL,
			       ~ISPTCTRL_CTRL_DIVA_MASK,
			       divisor << ISPTCTRL_CTRL_DIVA_SHIFT);
		DPRINTK_ISPCTRL("isp_set_xclk(): cam_xclka set to %d Hz\n",
				currentxclk);
		break;
	case 1:
		isp_reg_and_or(OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_CTRL,
			       ~ISPTCTRL_CTRL_DIVB_MASK,
			       divisor << ISPTCTRL_CTRL_DIVB_SHIFT);
		DPRINTK_ISPCTRL("isp_set_xclk(): cam_xclkb set to %d Hz\n",
				currentxclk);
		break;
	default:
		DPRINTK_ISPCTRL("ISP_ERR: isp_set_xclk(): Invalid requested "
				"xclk. Must be 0 (A) or 1 (B)."
				"\n");
		return -EINVAL;
	}

	return currentxclk;
}
EXPORT_SYMBOL(isp_set_xclk);

/**
 * isp_power_settings - Sysconfig settings, for Power Management.
 * @isp_sysconfig: Structure containing the power settings for ISP to configure
 *
 * Sets the power settings for the ISP, and SBL bus.
 **/
void isp_power_settings(int idle)
{
	if (idle) {
		isp_reg_writel(ISP_SYSCONFIG_AUTOIDLE |
			       (ISP_SYSCONFIG_MIDLEMODE_NOSTANBY <<
				ISP_SYSCONFIG_MIDLEMODE_SHIFT),
			       OMAP3_ISP_IOMEM_MAIN,
			       ISP_SYSCONFIG);
		if (omap_rev() == OMAP3430_REV_ES1_0) {
			isp_reg_writel(ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_SMARTSTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CSI2A,
				       ISP_CSIA_SYSCONFIG);
			isp_reg_writel(ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_SMARTSTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CCP2,
				       ISP_CSIB_SYSCONFIG);
		}
		isp_reg_writel(ISPCTRL_SBL_AUTOIDLE, OMAP3_ISP_IOMEM_MAIN,
			       ISP_CTRL);

	} else {
		isp_reg_writel(ISP_SYSCONFIG_AUTOIDLE |
			       (ISP_SYSCONFIG_MIDLEMODE_FORCESTANDBY <<
				ISP_SYSCONFIG_MIDLEMODE_SHIFT),
			       OMAP3_ISP_IOMEM_MAIN,
			       ISP_SYSCONFIG);
		if (omap_rev() == OMAP3430_REV_ES1_0) {
			isp_reg_writel(ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_FORCESTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CSI2A,
				       ISP_CSIA_SYSCONFIG);

			isp_reg_writel(ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_FORCESTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CCP2,
				       ISP_CSIB_SYSCONFIG);
		}

		isp_reg_writel(ISPCTRL_SBL_AUTOIDLE, OMAP3_ISP_IOMEM_MAIN,
			       ISP_CTRL);
	}
}
EXPORT_SYMBOL(isp_power_settings);

#define BIT_SET(var, shift, mask, val)		\
	do {					\
		var = (var & ~(mask << shift))	\
			| (val << shift);	\
	} while (0)

static int isp_init_csi(struct isp_interface_config *config)
{
	u32 i = 0, val, reg;
	int format;

	switch (config->u.csi.format) {
	case V4L2_PIX_FMT_SGRBG10:
		format = 0x16;		/* RAW10+VP */
		break;
	case V4L2_PIX_FMT_SGRBG10DPCM8:
		format = 0x12;		/* RAW8+DPCM10+VP */
		break;
	default:
		printk(KERN_ERR "isp_init_csi: bad csi format\n");
		return -EINVAL;
	}

	/* Reset the CSI and wait for reset to complete */
	isp_reg_writel(isp_reg_readl(OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSCONFIG) |
		       BIT(1),
		       OMAP3_ISP_IOMEM_CCP2,
		       ISPCSI1_SYSCONFIG);
	while (!(isp_reg_readl(OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSSTATUS) &
		 BIT(0))) {
		udelay(10);
		if (i++ > 10)
			break;
	}
	if (!(isp_reg_readl(OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSSTATUS) &
	      BIT(0))) {
		printk(KERN_WARNING
		       "omap3_isp: timeout waiting for csi reset\n");
	}

	/* ISPCSI1_CTRL */
	val = isp_reg_readl(OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);
	val &= ~BIT(11);	/* Enable VP only off ->
				   extract embedded data to interconnect */
	BIT_SET(val, 8, 0x3, config->u.csi.vpclk);	/* Video port clock */
/*	val |= BIT(3);	*/	/* Wait for FEC before disabling interface */
	val |= BIT(2);		/* I/O cell output is parallel
				   (no effect, but errata says should be enabled
				   for class 1/2) */
	val |= BIT(12);		/* VP clock polarity to falling edge
				   (needed or bad picture!) */

	/* Data/strobe physical layer */
	BIT_SET(val, 1, 1, config->u.csi.signalling);
	BIT_SET(val, 10, 1, config->u.csi.strobe_clock_inv);
	val |= BIT(4);		/* Magic bit to enable CSI1 and strobe mode */
	isp_reg_writel(val, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);

	/* ISPCSI1_LCx_CTRL logical channel #0 */
	reg = ISPCSI1_LCx_CTRL(0);	/* reg = ISPCSI1_CTRL1; */
	val = isp_reg_readl(OMAP3_ISP_IOMEM_CCP2, reg);
	/* Format = RAW10+VP or RAW8+DPCM10+VP*/
	BIT_SET(val, 3, 0x1f, format);
	/* Enable setting of frame regions of interest */
	BIT_SET(val, 1, 1, 1);
	BIT_SET(val, 2, 1, config->u.csi.crc);
	isp_reg_writel(val, OMAP3_ISP_IOMEM_CCP2, reg);

	/* ISPCSI1_DAT_START for logical channel #0 */
	reg = ISPCSI1_LCx_DAT_START(0);		/* reg = ISPCSI1_DAT_START; */
	val = isp_reg_readl(OMAP3_ISP_IOMEM_CCP2, reg);
	BIT_SET(val, 16, 0xfff, config->u.csi.data_start);
	isp_reg_writel(val, OMAP3_ISP_IOMEM_CCP2, reg);

	/* ISPCSI1_DAT_SIZE for logical channel #0 */
	reg = ISPCSI1_LCx_DAT_SIZE(0);		/* reg = ISPCSI1_DAT_SIZE; */
	val = isp_reg_readl(OMAP3_ISP_IOMEM_CCP2, reg);
	BIT_SET(val, 16, 0xfff, config->u.csi.data_size);
	isp_reg_writel(val, OMAP3_ISP_IOMEM_CCP2, reg);

	/* Clear status bits for logical channel #0 */
	isp_reg_writel(0xFFF & ~BIT(6), OMAP3_ISP_IOMEM_CCP2,
		       ISPCSI1_LC01_IRQSTATUS);

	/* Enable CSI1 */
	val = isp_reg_readl(OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);
	val |=  BIT(0) | BIT(4);
	isp_reg_writel(val, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);

	if (!(isp_reg_readl(OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL) & BIT(4))) {
		printk(KERN_WARNING "OMAP3 CSI1 bus not available\n");
		if (config->u.csi.signalling)	/* Strobe mode requires CSI1 */
			return -EIO;
	}

	return 0;
}

/**
 * isp_configure_interface - Configures ISP Control I/F related parameters.
 * @config: Pointer to structure containing the desired configuration for the
 * 	ISP.
 *
 * Configures ISP control register (ISP_CTRL) with the values specified inside
 * the config structure. Controls:
 * - Selection of parallel or serial input to the preview hardware.
 * - Data lane shifter.
 * - Pixel clock polarity.
 * - 8 to 16-bit bridge at the input of CCDC module.
 * - HS or VS synchronization signal detection
 **/
int isp_configure_interface(struct isp_interface_config *config)
{
	u32 ispctrl_val = isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);
	int r;

	isp_obj.config = config;

	ispctrl_val &= ISPCTRL_SHIFT_MASK;
	ispctrl_val |= config->dataline_shift << ISPCTRL_SHIFT_SHIFT;
	ispctrl_val &= ~ISPCTRL_PAR_CLK_POL_INV;

	ispctrl_val &= ISPCTRL_PAR_SER_CLK_SEL_MASK;

	isp_buf_init();

	switch (config->ccdc_par_ser) {
	case ISP_PARLL:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_PARALLEL;
		ispctrl_val |= config->u.par.par_clk_pol
			<< ISPCTRL_PAR_CLK_POL_SHIFT;
		ispctrl_val &= ~ISPCTRL_PAR_BRIDGE_BENDIAN;
		ispctrl_val |= config->u.par.par_bridge
			<< ISPCTRL_PAR_BRIDGE_SHIFT;
		break;
	case ISP_CSIA:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_CSIA;
		ispctrl_val &= ~ISPCTRL_PAR_BRIDGE_BENDIAN;

		if (config->u.csi.crc)
			isp_csi2_ctrl_config_ecc_enable(true);

		isp_csi2_ctrl_config_vp_out_ctrl(config->u.csi.vpclk);
		isp_csi2_ctrl_config_vp_only_enable(true);
		isp_csi2_ctrl_config_vp_clk_enable(true);
		isp_csi2_ctrl_update(false);

		isp_csi2_ctx_config_format(0, config->u.csi.format);
		isp_csi2_ctx_update(0, false);

		isp_csi2_irq_complexio1_set(1);
		isp_csi2_irq_status_set(1);
		isp_csi2_irq_set(1);

		isp_csi2_enable(1);
		mdelay(3);
		break;
	case ISP_CSIB:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_CSIB;
		r = isp_init_csi(config);
		if (r)
			return r;
		break;
	case ISP_NONE:
		return 0;
	default:
		return -EINVAL;
	}

	ispctrl_val &= ~ISPCTRL_SYNC_DETECT_VSRISE;
	ispctrl_val |= config->hsvs_syncdetect;

	isp_reg_writel(ispctrl_val, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);

	/* Set sensor specific fields in CCDC and Previewer module.*/
	ispccdc_set_wenlog(config->wenlog);
	ispccdc_set_dcsub(config->dcsub);
	ispccdc_set_raw_offset(config->raw_fmt_in);

	isp_obj.mclk_hz = config->cam_mclk;
	isp_obj.mclk_src_div = config->cam_mclk_src_div;

	/* MAKE THIS BETTER */
	omap_writel(isp_obj.mclk_src_div, OMAP3_CM_CLKSEL_CAM);

	return 0;
}
EXPORT_SYMBOL(isp_configure_interface);

/**
 * isp_configure_interface_bridge - Configure CCDC i/f bridge.
 *
 * Sets the bit field that controls the 8 to 16-bit bridge at
 * the input to CCDC.
 **/
int isp_configure_interface_bridge(u32 par_bridge)
{
	u32 ispctrl_val = isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);

	ispctrl_val &= ~ISPCTRL_PAR_BRIDGE_BENDIAN;
	ispctrl_val |= (par_bridge << ISPCTRL_PAR_BRIDGE_SHIFT);
	isp_reg_writel(ispctrl_val, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);

	return 0;
}
EXPORT_SYMBOL(isp_configure_interface_bridge);

static int isp_buf_process(struct isp_bufs *bufs);
static void isp_buf_complete(struct isp_bufs *bufs);

/**
 * omap34xx_isp_isr - Interrupt Service Routine for Camera ISP module.
 * @irq: Not used currently.
 * @ispirq_disp: Pointer to the object that is passed while request_irq is
 *               called. This is the isp_obj.irq object containing info on the
 *               callback.
 *
 * Handles the corresponding callback if plugged in.
 *
 * Returns IRQ_HANDLED when IRQ was correctly handled, or IRQ_NONE when the
 * IRQ wasn't handled.
 **/
static irqreturn_t omap34xx_isp_isr(int irq, void *_isp)
{
	u32 irqstatus = 0;
	struct isp *isp = _isp;
	struct isp_irq *irqdis = &isp->irq;
	struct isp_bufs *bufs = &isp->bufs;
	unsigned long irqflags = 0;

	spin_lock_irqsave(&isp_obj.lock, irqflags);
	irqstatus = isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
	isp_reg_writel(irqstatus, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);

	if (isp->running == ISP_RUNNING) {
		if ((irqstatus & CCDC_VD0) &&
				CCDC_CAPTURE(&isp_obj)) {
			if (bufs->wait_bayer_frame) {
				bufs->wait_bayer_frame--;
				irqstatus &= ~CCDC_VD0;
			}
		} else if ((irqstatus & RESZ_DONE) &&
				CCDC_PREV_RESZ_CAPTURE(&isp_obj)) {
			if (bufs->wait_yuv_frame) {
				bufs->wait_yuv_frame--;
				irqstatus &= ~RESZ_DONE;
			}
		} else if ((irqstatus & PREV_DONE) &&
				CCDC_PREV_CAPTURE(&isp_obj)) {
			if (bufs->wait_yuv_frame) {
				bufs->wait_yuv_frame--;
				irqstatus &= ~PREV_DONE;
			}
		}
	}

	if (irqstatus & CSIA) {
		struct isp_buf *buf = ISP_BUF_DONE(bufs);
		isp_csi2_isr();
		buf->vb_state = VIDEOBUF_ERROR;
	}

	if (irqstatus & IRQ0STATUS_CSIB_IRQ) {
		u32 ispcsi1_irqstatus;
		ispcsi1_irqstatus = isp_reg_readl(OMAP3_ISP_IOMEM_CCP2,
						  ISPCSI1_LC01_IRQSTATUS);
		DPRINTK_ISPCTRL("%x\n", ispcsi1_irqstatus);
	}

	if (irqstatus & HS_VS) {
		if (isp_obj.isp_lsc_workaround == 0 &&
			CCDC_PREV_RESZ_CAPTURE(&isp_obj) &&
			!ispresizer_busy()) {
			if (isp_obj.module.applyCrop == 0 &&
				isp_obj.running == ISP_RUNNING)
				ispresizer_enable(1);
			else {
				ispresizer_applycrop();
				if (!ispresizer_busy())
					isp_obj.module.applyCrop = 0;
			}
		}
#if defined(CONFIG_VIDEO_OMAP3_HP3A)
		hp3a_ccdc_start();
#endif
	}

	if (irqstatus & CCDC_VD0) {
#if defined(CONFIG_VIDEO_OMAP3_HP3A)
		struct isp_buf *buf = ISP_BUF_DONE(bufs);
		if (isp_obj.running == ISP_RUNNING)
			hp3a_ccdc_done();
		else if (isp_obj.running == ISP_STOPPING)
			ispccdc_enable(0);
		if (buf && buf->vb)
			ktime_get_ts((struct timespec *)&(buf->vb->ts));
#endif
		if (CCDC_CAPTURE(&isp_obj))
			isp_buf_process(bufs);
	}

	/*
	if (irqstatus & CCDC_VD1) {
		if (CCDC_CAPTURE(&isp_obj))
			ispccdc_enable(0);
	}
	*/

	if (irqstatus & LSC_PRE_ERR) {
		if (isp_obj.running == ISP_RUNNING) {
			if (!CCDC_CAPTURE(&isp_obj)) {
				struct isp_buf *buf = ISP_BUF_DONE(bufs);
				/* Mark buffer faulty. */
				buf->vb_state = VIDEOBUF_ERROR;
			}
			ispccdc_lsc_state_handler(LSC_PRE_ERR);
			printk(KERN_ERR "isp: lsc prefetch error\n");
		}
	}

	if (irqstatus & PREV_DONE) {
#ifdef CONFIG_VIDEO_OMAP3_HP3A
		hp3a_frame_done();
#endif
		if (irqdis->isp_callbk[CBK_PREV_DONE]) {
			irqdis->isp_callbk[CBK_PREV_DONE](
				PREV_DONE,
				irqdis->isp_callbk_arg1[CBK_PREV_DONE],
				irqdis->isp_callbk_arg2[CBK_PREV_DONE]);
		} else {
			if (CCDC_PREV_RESZ_CAPTURE(&isp_obj) &&
				isp_obj.isp_lsc_workaround == 1) {
				if (!ispresizer_busy()) {
					if (isp_obj.module.applyCrop) {
						ispresizer_applycrop();
						if (!ispresizer_busy())
							isp_obj \
							.module.applyCrop = 0;
					}
					if (!isppreview_busy()) {
						if (isp_obj.running ==
							ISP_RUNNING)
							ispresizer_enable(1);
						if (isppreview_busy()) {
							/* FIXME: locking! */
							ISP_BUF_DONE \
							(bufs)->vb_state =
								VIDEOBUF_ERROR;
							printk(KERN_ERR \
								"%s: can't stop"
								" preview\n",
								__func__);
						}
					}
				}
			}

			if (isp_obj.running == ISP_STOPPING) {
				isppreview_enable(0);
			} else {
				if (!isppreview_busy())
					isppreview_config_shadow_registers();
#if defined(CONFIG_VIDEO_OMAP3_HP3A)
				hp3a_update_wb();
#else
				if (!isppreview_busy())
					isph3a_update_wb();
#endif
				if (CCDC_PREV_CAPTURE(&isp_obj))
					isp_buf_process(bufs);
			}
		}
	}

	if (irqstatus & RESZ_DONE) {
		if (irqdis->isp_callbk[CBK_RESZ_DONE])
			irqdis->isp_callbk[CBK_RESZ_DONE](
				RESZ_DONE,
				irqdis->isp_callbk_arg1[CBK_RESZ_DONE],
				irqdis->isp_callbk_arg2[CBK_RESZ_DONE]);
		else if (CCDC_PREV_RESZ_CAPTURE(&isp_obj)) {
			isp_buf_process(bufs);
			ispresizer_config_shadow_registers();
		}
	}

	if (irqstatus & H3A_AWB_DONE) {
		if (irqdis->isp_callbk[CBK_H3A_AWB_DONE])
			irqdis->isp_callbk[CBK_H3A_AWB_DONE](
				H3A_AWB_DONE,
				irqdis->isp_callbk_arg1[CBK_H3A_AWB_DONE],
				irqdis->isp_callbk_arg2[CBK_H3A_AWB_DONE]);
	}

	if (irqstatus & HIST_DONE) {
		if (irqdis->isp_callbk[CBK_HIST_DONE])
			irqdis->isp_callbk[CBK_HIST_DONE](
				HIST_DONE,
				irqdis->isp_callbk_arg1[CBK_HIST_DONE],
				irqdis->isp_callbk_arg2[CBK_HIST_DONE]);
	}

	if (irqstatus & H3A_AF_DONE) {
		if (irqdis->isp_callbk[CBK_H3A_AF_DONE])
			irqdis->isp_callbk[CBK_H3A_AF_DONE](
				H3A_AF_DONE,
				irqdis->isp_callbk_arg1[CBK_H3A_AF_DONE],
				irqdis->isp_callbk_arg2[CBK_H3A_AF_DONE]);
	}

	if (irqdis->isp_callbk[CBK_CATCHALL]) {
		irqdis->isp_callbk[CBK_CATCHALL](
			irqstatus,
			irqdis->isp_callbk_arg1[CBK_CATCHALL],
			irqdis->isp_callbk_arg2[CBK_CATCHALL]);
	}

	if (irqstatus & LSC_DONE)
		ispccdc_lsc_state_handler(LSC_DONE);

	spin_unlock_irqrestore(&isp_obj.lock, irqflags);

	if (irqstatus & LSC_PRE_COMP)
		ispccdc_lsc_state_handler(LSC_PRE_COMP);

	isp_flush();

#if 0
	{
		static const struct {
			int num;
			char *name;
		} bits[] = {
			{ 31, "HS_VS_IRQ" },
			{ 30, "SEC_ERR_IRQ" },
			{ 29, "OCP_ERR_IRQ" },
			{ 28, "MMU_ERR_IRQ" },
			{ 27, "res27" },
			{ 26, "res26" },
			{ 25, "OVF_IRQ" },
			{ 24, "RSZ_DONE_IRQ" },
			{ 23, "res23" },
			{ 22, "res22" },
			{ 21, "CBUFF_IRQ" },
			{ 20, "PRV_DONE_IRQ" },
			{ 19, "CCDC_LSC_PREFETCH_ERROR" },
			{ 18, "CCDC_LSC_PREFETCH_COMPLETED" },
			{ 17, "CCDC_LSC_DONE" },
			{ 16, "HIST_DONE_IRQ" },
			{ 15, "res15" },
			{ 14, "res14" },
			{ 13, "H3A_AWB_DONE_IRQ" },
			{ 12, "H3A_AF_DONE_IRQ" },
			{ 11, "CCDC_ERR_IRQ" },
			{ 10, "CCDC_VD2_IRQ" },
			{  9, "CCDC_VD1_IRQ" },
			{  8, "CCDC_VD0_IRQ" },
			{  7, "res7" },
			{  6, "res6" },
			{  5, "res5" },
			{  4, "CSIB_IRQ" },
			{  3, "CSIB_LCM_IRQ" },
			{  2, "res2" },
			{  1, "res1" },
			{  0, "CSIA_IRQ" },
		};
		int i;
		for (i = 0; i < ARRAY_SIZE(bits); i++) {
			if ((1 << bits[i].num) & irqstatus)
				DPRINTK_ISPCTRL("%s ", bits[i].name);
		}
		DPRINTK_ISPCTRL("\n");
	}
#endif

	return IRQ_HANDLED;
}

/* Device name, needed for resource tracking layer */
struct device_driver camera_drv = {
	.name = "camera"
};

struct device camera_dev = {
	.driver = &camera_drv,
};

/**
 *  isp_tmp_buf_free - To free allocated 10MB memory
 *
 **/
static void isp_tmp_buf_free(void)
{
	if (isp_obj.tmp_buf) {
		ispmmu_vfree(isp_obj.tmp_buf);
		isp_obj.tmp_buf = 0;
		isp_obj.tmp_buf_size = 0;
	}
}

/**
 *  isp_tmp_buf_alloc - To allocate LSC memory
 *
 **/
static u32 isp_tmp_buf_alloc(size_t size)
{
	isp_tmp_buf_free();

	printk(KERN_INFO "%s: allocating %d bytes\n", __func__, size);

	isp_obj.tmp_buf = ispmmu_vmalloc(size);
	if (IS_ERR((void *)isp_obj.tmp_buf)) {
		isp_obj.tmp_buf = 0;
		printk(KERN_ERR "ispmmu_vmap mapping failed ");
		return -ENOMEM;
	}
	isp_obj.tmp_buf_size = size;

	return 0;
}

/**
 *  isp_tmp_buf_addr - Returns ISP tmp buffer address
 *
 **/
dma_addr_t isp_tmp_buf_addr(void)
{
	return isp_obj.tmp_buf;
}

/**
 * isp_start - Starts ISP submodule
 *
 * Start the needed isp components assuming these components
 * are configured correctly.
 **/
void isp_start(void)
{
	if (isp_obj.module.isp_pipeline & OMAP_ISP_PREVIEW)
		isppreview_enable(1);

#if !defined(CONFIG_VIDEO_OMAP3_HP3A)
	isph3a_notify(0);
	isp_af_notify(0);
#endif

	isp_obj.running = ISP_RUNNING;
	return;
}
EXPORT_SYMBOL(isp_start);

#define ISP_STATISTICS_BUSY			\
	()
#define ISP_STOP_TIMEOUT	msecs_to_jiffies(256)
static int __isp_disable_modules(int suspend)
{
	unsigned long timeout;
	int reset = 0;

	/*
	 * We need to stop all the modules after CCDC or they'll
	 * never stop since they may not get a full frame from CCDC.
	 */
	if (suspend) {
#if !defined(CONFIG_VIDEO_OMAP3_HP3A)
		isp_af_suspend();
		isph3a_aewb_suspend();
		isp_hist_suspend();
#endif
		if (isp_obj.module.isp_pipeline & OMAP_ISP_PREVIEW)
			isppreview_suspend();
		if (isp_obj.module.isp_pipeline & OMAP_ISP_RESIZER)
			ispresizer_suspend();
	} else {
#if !defined(CONFIG_VIDEO_OMAP3_HP3A)
		isp_af_enable(0);
		isph3a_aewb_enable(0);
		isp_hist_enable(0);
#endif
		if (isp_obj.module.isp_pipeline & OMAP_ISP_PREVIEW)
			isppreview_enable(0);
		if (isp_obj.module.isp_pipeline & OMAP_ISP_RESIZER)
			ispresizer_enable(0);
	}

	timeout = jiffies + ISP_STOP_TIMEOUT;
#if defined(CONFIG_VIDEO_OMAP3_HP3A)
	while (hp3a_af_busy()
	       || hp3a_hist_busy()
	       || isppreview_busy()
	       || ispresizer_busy()) {
#else
	while (isp_af_busy()
	       || isph3a_aewb_busy()
	       || isp_hist_busy()
	       || isppreview_busy()
	       || ispresizer_busy()) {
#endif
		if (time_after(jiffies, timeout)) {
			printk(KERN_ERR "%s: can't stop non-ccdc modules\n",
			       __func__);
			reset = 1;
			break;
		}
		msleep(1);
	}

	/* Let's stop CCDC now. */
	if (suspend)
		ispccdc_suspend();/* This function supends lsc too */


	timeout = jiffies + ISP_STOP_TIMEOUT;
	while (ispccdc_busy()) {
		if (time_after(jiffies, timeout)) {
			printk(KERN_ERR "%s: can't stop ccdc\n", __func__);
			reset = 1;
			break;
		}
		msleep(1);
	}

	/* Trigger isp reset if lsc is still busy */
	if (ispccdc_lsc_busy() || !cpu_is_omap3630())
		reset = 1;

	/* disable lsc now */
	ispccdc_enable_lsc(0);

	if (!suspend) {
		isp_csi2_irq_complexio1_set(0);
		isp_csi2_ctrl_phy_if_enable(0);
	}

	if (!reset)
		DPRINTK_ISPCTRL(KERN_INFO
			"(%s) isp_complete_reset \n", __func__);
	return reset;
}

static int isp_stop_modules(void)
{
	return __isp_disable_modules(0);
}

static int isp_suspend_modules(void)
{
	return __isp_disable_modules(1);
}

static void isp_resume_modules(void)
{
	if (isp_obj.module.isp_pipeline & OMAP_ISP_RESIZER)
		ispresizer_resume();
	if (isp_obj.module.isp_pipeline & OMAP_ISP_PREVIEW)
		isppreview_resume();
#if !defined(CONFIG_VIDEO_OMAP3_HP3A)
	isp_hist_resume();
	isph3a_aewb_resume();
	isp_af_resume();
#endif
	ispccdc_resume();
}

static void isp_reset(void)
{
	unsigned long timeout = 0;

	isp_reg_writel(isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG)
		       | ISP_SYSCONFIG_SOFTRESET,
		       OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG);
	while (!(isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_SYSSTATUS) & 0x1)) {
		if (timeout++ > 1000) {
			printk(KERN_ALERT "%s: cannot reset ISP\n", __func__);
			break;
		}
		udelay(10);
	}
}

/**
 * isp_stop - Stops isp submodules
 **/
void isp_stop()
{
	unsigned long irqflags = 0;
	int reset;

	spin_lock_irqsave(&isp_obj.lock, irqflags);
	isp_obj.running = ISP_STOPPING;
	ispccdc_request_lsc_enable(0);
	ispccdc_enable(0);
	spin_unlock_irqrestore(&isp_obj.lock, irqflags);
#if !defined(CONFIG_VIDEO_OMAP3_HP3A)
	isph3a_notify(1);
	isp_af_notify(1);
#endif
	reset = isp_stop_modules();
	isp_disable_interrupts();
	isp_obj.running = ISP_STOPPED;
	isp_buf_complete(&isp_obj.bufs);
	isp_buf_init();
	if (!reset)
		return;

	isp_save_ctx();
	isp_reset();
	isp_restore_ctx();
}
EXPORT_SYMBOL(isp_stop);

static void isp_set_buf(struct isp_buf *buf)
{
	if (CCDC_PREV_RESZ_CAPTURE(&isp_obj))
		ispresizer_set_outaddr(buf->isp_addr);
	else if (CCDC_PREV_CAPTURE(&isp_obj))
		isppreview_set_outaddr(buf->isp_addr);
	else if (CCDC_CAPTURE(&isp_obj))
		ispccdc_set_outaddr(buf->isp_addr);
}

/**
 * isp_calc_pipeline - Sets pipeline depending of input and output pixel format
 * @pix_input: Pointer to V4L2 pixel format structure for input image.
 * @pix_output: Pointer to V4L2 pixel format structure for output image.
 **/
static u32 isp_calc_pipeline(struct v4l2_pix_format *pix_input,
			     struct v4l2_pix_format *pix_output)
{
	isp_release_resources();
	if ((pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10
	     || pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10DPCM8)
	    && pix_output->pixelformat != V4L2_PIX_FMT_SGRBG10) {

		isp_obj.module.isp_pipeline = OMAP_ISP_PREVIEW |
						      OMAP_ISP_RESIZER |
						      OMAP_ISP_CCDC;

		ispccdc_request();
		isppreview_request();
		ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_VP);

		if (isp_obj.isp_lsc_workaround == 1) {
			isppreview_config_datapath(PRV_RAW_CCDC, PREVIEW_MEM);
			if (isp_obj.module.isp_pipeline & OMAP_ISP_RESIZER) {
				ispresizer_request();
				ispresizer_config_datapath(RSZ_MEM_YUV, 1);
			}
		} else {
			/* ispccdc_enable_lsc(0); */
			if (isp_obj.module.isp_pipeline & OMAP_ISP_RESIZER) {
				isppreview_config_datapath(PRV_RAW_CCDC,
					PREVIEW_RSZ);
				ispresizer_request();
				ispresizer_config_datapath(RSZ_OTFLY_YUV, 1);
			} else {
				isppreview_config_datapath(PRV_RAW_CCDC,
					PREVIEW_MEM);
			}
		}
	} else {
		isp_obj.module.isp_pipeline = OMAP_ISP_CCDC;
		ispccdc_request();
		if (pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10
		    || pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10DPCM8)
			ispccdc_config_datapath(CCDC_RAW,
				CCDC_OTHERS_VP_MEM_LSC /*CCDC_OTHERS_VP_MEM*/);
		else
			ispccdc_config_datapath(CCDC_YUV_SYNC,
						CCDC_OTHERS_MEM);
	}
	return 0;
}

/**
 * isp_config_pipeline - Configures the image size and ycpos for ISP submodules
 * @pix_input: Pointer to V4L2 pixel format structure for input image.
 * @pix_output: Pointer to V4L2 pixel format structure for output image.
 *
 * The configuration of ycpos depends on the output pixel format for both the
 * Preview and Resizer submodules.
 **/
static void isp_config_pipeline(struct v4l2_pix_format *pix_input,
				struct v4l2_pix_format *pix_output)
{
	ispccdc_config_size(isp_obj.module.ccdc_input_width,
			    isp_obj.module.ccdc_input_height,
			    isp_obj.module.ccdc_output_width,
			    isp_obj.module.ccdc_output_height);

	if (isp_obj.module.isp_pipeline & OMAP_ISP_PREVIEW) {
		isppreview_config_size(isp_obj.module.preview_input_width,
				       isp_obj.module.preview_input_height,
				       isp_obj.module.preview_output_width,
				       isp_obj.module.preview_output_height);
	}

	if (isp_obj.module.isp_pipeline & OMAP_ISP_RESIZER) {
		ispresizer_config_size(isp_obj.module.resizer_input_width,
				       isp_obj.module.resizer_input_height,
				       isp_obj.module.resizer_output_width,
				       isp_obj.module.resizer_output_height);
	}

	if (CCDC_PREV_RESZ_CAPTURE(&isp_obj)) {
		if (pix_output->pixelformat == V4L2_PIX_FMT_UYVY) {
			isppreview_config_ycpos(YCPOS_YCrYCb);
			ispresizer_config_ycpos(0);
		} else {
			isppreview_config_ycpos(YCPOS_CrYCbY);
			ispresizer_config_ycpos(1);
		}
	} else if (CCDC_PREV_CAPTURE(&isp_obj)) {
		if (pix_output->pixelformat == V4L2_PIX_FMT_UYVY)
			isppreview_config_ycpos(YCPOS_YCrYCb);
		else
			isppreview_config_ycpos(YCPOS_CrYCbY);
	}

	return;
}

void isp_set_wait_yuv(int wait_yuv)
{
	struct isp_bufs *bufs = &isp_obj.bufs;

	bufs->wait_yuv_frame = wait_yuv;
	return;
}
EXPORT_SYMBOL(isp_set_wait_yuv);

static void isp_buf_init(void)
{
	struct isp_bufs *bufs = &isp_obj.bufs;
	int sg;

	bufs->queue = 0;
	bufs->done = 0;
	bufs->wait_bayer_frame = isp_obj.config->wait_bayer_frame;
	bufs->wait_yuv_frame = isp_obj.config->wait_yuv_frame;
	for (sg = 0; sg < NUM_BUFS; sg++) {
		bufs->buf[sg].complete = NULL;
		bufs->buf[sg].vb = NULL;
		bufs->buf[sg].priv = NULL;
	}
}

/**
 * isp_vbq_sync - Walks the pages table and flushes the cache for
 *                each page.
 **/
static int isp_vbq_sync(struct videobuf_buffer *vb, int when)
{
	if (when == DMA_TO_DEVICE)
		flush_cache_all();

	return 0;
}

static void isp_buf_complete(struct isp_bufs *bufs)
{
	struct isp_buf *buf = NULL;

	if (!ISP_BUFS_IS_EMPTY(bufs)) {
		buf = ISP_BUF_DONE(bufs);
		if (buf != NULL) {
			ISP_BUF_MARK_DONE(bufs);
			/*
			 * Mark a buffer done to be qequeued
			 */
			buf->vb->state = VIDEOBUF_DONE;
			buf->complete(buf->vb, buf->priv);
		}
	}
}

static int isp_buf_process(struct isp_bufs *bufs)
{
	struct isp_buf *buf = NULL;
	unsigned long flags;
	int last;

	spin_lock_irqsave(&bufs->lock, flags);

	if (ISP_BUFS_IS_EMPTY(bufs))
		goto out;

	if (CCDC_CAPTURE(&isp_obj)) {
		if (ispccdc_sbl_wait_idle(1000)) {
			ispccdc_enable(1);
			printk(KERN_ERR "ccdc %d won't become idle!\n",
			       CCDC_CAPTURE(&isp_obj));
			goto out;
		}
	}

	/* We had at least one buffer in queue. */
	buf = ISP_BUF_DONE(bufs);
	last = ISP_BUFS_IS_LAST(bufs);

	if (!last) {
		/* Set new buffer address. */
		isp_set_buf(ISP_BUF_NEXT_DONE(bufs));
	} else {
		/* Tell ISP not to write any of our buffers. */
		isp_disable_interrupts();
		/*
		 * We must wait for one HS_VS since before that the
		 * CCDC may trigger interrupts even if it's not
		 * receiving a frame.
		 */
		if (CCDC_CAPTURE(&isp_obj)) {
			/*
			 * Disabling CCDC out to memory to avoid frame data
			 * being over written.
			 */
			ispccdc_enable(0);
			ispccdc_set_outaddr(isp_tmp_buf_addr());
		} else if (CCDC_PREV_CAPTURE(&isp_obj))
			isppreview_enable(0);
		else if (CCDC_PREV_RESZ_CAPTURE(&isp_obj))
			ispresizer_enable(0);
	}
	if ((CCDC_CAPTURE(&isp_obj) && ispccdc_busy()) ||
		(CCDC_PREV_RESZ_CAPTURE(&isp_obj) && ispresizer_busy()) ||
		(CCDC_PREV_CAPTURE(&isp_obj) && isppreview_busy())) {
		/*
		 * Next buffer available: for the transfer to succeed, the
		 * CCDC (RAW capture) or resizer (YUV capture) must be idle
		 * for the duration of transfer setup. Bad things happen
		 * otherwise!
		 *
		 * Next buffer not available: if we fail to stop the
		 * ISP the buffer is probably going to be bad.
		 */
		/* Mark this buffer faulty. */
		buf->vb_state = VIDEOBUF_ERROR;
		/* Mark next faulty, too, in case we have one. */
		if (!last) {
			ISP_BUF_NEXT_DONE(bufs)->vb_state =
				VIDEOBUF_ERROR;
			printk(KERN_ALERT "OUCH!!!\n");
		} else {
			printk(KERN_ALERT "Ouch!\n");
		}
	}

	/* Mark the current buffer as done. */
	ISP_BUF_MARK_DONE(bufs);

	DPRINTK_ISPCTRL(KERN_ALERT "%s: finish %d mmu %p\n", __func__,
			(bufs->done - 1 + NUM_BUFS) % NUM_BUFS,
			(bufs->buf+((bufs->done - 1 + NUM_BUFS)
				    % NUM_BUFS))->isp_addr);

out:
	spin_unlock_irqrestore(&bufs->lock, flags);

	if (buf != NULL) {
		/*
		 * We want to dequeue a buffer from the video buffer
		 * queue. Let's do it!
		 */
		buf->vb->state = buf->vb_state;
		buf->complete(buf->vb, buf->priv);
	}

	return 0;
}

int isp_buf_queue(struct videobuf_buffer *vb,
		  void (*complete)(struct videobuf_buffer *vb, void *priv),
		  void *priv)
{
	unsigned long flags;
	struct isp_buf *buf;
	struct videobuf_dmabuf *dma = videobuf_to_dma(vb);
	const struct scatterlist *sglist = dma->sglist;
	struct isp_bufs *bufs = &isp_obj.bufs;
	int sglen = dma->sglen;

	BUG_ON(sglen < 0 || !sglist);

	isp_vbq_sync(vb, DMA_TO_DEVICE);

	spin_lock_irqsave(&bufs->lock, flags);

	BUG_ON(ISP_BUFS_IS_FULL(bufs));

	buf = ISP_BUF_QUEUE(bufs);

	buf->isp_addr = bufs->isp_addr_capture[vb->i];
	buf->complete = complete;
	buf->vb = vb;
	buf->priv = priv;
	buf->vb_state = VIDEOBUF_DONE;

	/* If ISP is in the process of stopping then don't re-enable ISP */
	if (ISP_BUFS_IS_EMPTY(bufs) &&
		isp_obj.running != ISP_STOPPING) {
		isp_enable_interrupts();
		isp_set_buf(buf);
		ispccdc_enable(1);
		isp_start();
	}

	ISP_BUF_MARK_QUEUED(bufs);

	spin_unlock_irqrestore(&bufs->lock, flags);

	DPRINTK_ISPCTRL(KERN_ALERT "%s: queue %d vb %d, mmu %p\n", __func__,
			(bufs->queue - 1 + NUM_BUFS) % NUM_BUFS, vb->i,
			buf->isp_addr);

	return 0;
}
EXPORT_SYMBOL(isp_buf_queue);

int isp_vbq_setup(struct videobuf_queue *vbq, unsigned int *cnt,
		  unsigned int *size)
{
	int rval = 0;
	size_t tmp_size = PAGE_ALIGN(isp_obj.module.preview_output_width
				     * isp_obj.module.preview_output_height
				     * ISP_BYTES_PER_PIXEL);

	if (CCDC_PREV_RESZ_CAPTURE(&isp_obj)) {
		if (isp_obj.isp_lsc_workaround == 1) {
			if (isp_obj.tmp_buf_size < tmp_size)
				rval = isp_tmp_buf_alloc(tmp_size);
			if (!rval) {
				isppreview_set_outaddr(isp_obj.tmp_buf);
				ispresizer_set_inaddr(isp_obj.tmp_buf);
			}
		}
	}

	return rval;
}
EXPORT_SYMBOL(isp_vbq_setup);

/**
 * isp_vbq_prepare - Videobuffer queue prepare.
 * @vbq: Pointer to videobuf_queue structure.
 * @vb: Pointer to videobuf_buffer structure.
 * @field: Requested Field order for the videobuffer.
 *
 * Returns 0 if successful, or -EIO if the ispmmu was unable to map a
 * scatter-gather linked list data space.
 **/
int isp_vbq_prepare(struct videobuf_queue *vbq, struct videobuf_buffer *vb,
		    enum v4l2_field field)
{
	unsigned int isp_addr;
	struct videobuf_dmabuf *vdma;
	struct isp_bufs *bufs = &isp_obj.bufs;

	int err = 0;

	vdma = videobuf_to_dma(vb);

	isp_addr = ispmmu_vmap(vdma->sglist, vdma->sglen);

	if (IS_ERR_VALUE(isp_addr))
		err = -EIO;
	else
		bufs->isp_addr_capture[vb->i] = isp_addr;

	return err;
}
EXPORT_SYMBOL(isp_vbq_prepare);

/**
 * isp_vbq_release - Videobuffer queue release.
 * @vbq: Pointer to videobuf_queue structure.
 * @vb: Pointer to videobuf_buffer structure.
 **/
void isp_vbq_release(struct videobuf_queue *vbq, struct videobuf_buffer *vb)
{
	struct isp_bufs *bufs = &isp_obj.bufs;

	if (bufs->isp_addr_capture[vb->i]) {
		ispmmu_vunmap(bufs->isp_addr_capture[vb->i]);
		bufs->isp_addr_capture[vb->i] = (dma_addr_t)NULL;
	}
	return;
}
EXPORT_SYMBOL(isp_vbq_release);

/**
 * isp_queryctrl - Query V4L2 control from existing controls in ISP.
 * @a: Pointer to v4l2_queryctrl structure. It only needs the id field filled.
 *
 * Returns 0 if successful, or -EINVAL if not found in ISP.
 **/
int isp_queryctrl(struct v4l2_queryctrl *a)
{
	int i;

	if (a->id & V4L2_CTRL_FLAG_NEXT_CTRL) {
		a->id &= ~V4L2_CTRL_FLAG_NEXT_CTRL;
		i = find_next_vctrl(a->id);
	} else {
		i = find_vctrl(a->id);
	}

	if (i < 0)
		return -EINVAL;

	*a = video_control[i].qc;
	return 0;
}
EXPORT_SYMBOL(isp_queryctrl);

/**
 * isp_queryctrl - Query V4L2 control from existing controls in ISP.
 * @a: Pointer to v4l2_queryctrl structure. It only needs the id field filled.
 *
 * Returns 0 if successful, or -EINVAL if not found in ISP.
 **/
int isp_querymenu(struct v4l2_querymenu *a)
{
	int i;

	i = find_vmenu(a->id, a->index);

	if (i < 0)
		return -EINVAL;

	*a = video_menu[i];
	return 0;
}
EXPORT_SYMBOL(isp_querymenu);

/**
 * isp_g_ctrl - Gets value of the desired V4L2 control.
 * @a: V4L2 control to read actual value from.
 *
 * Return 0 if successful, or -EINVAL if chosen control is not found.
 **/
int isp_g_ctrl(struct v4l2_control *a)
{
	u8 current_value;
	int rval = 0;

	if (!isp_obj.ref_count)
		return -EINVAL;

	switch (a->id) {
	case V4L2_CID_BRIGHTNESS:
		isppreview_query_brightness(&current_value);
		a->value = current_value / ISPPRV_BRIGHT_UNITS;
		break;
	case V4L2_CID_CONTRAST:
		isppreview_query_contrast(&current_value);
		a->value = current_value / ISPPRV_CONTRAST_UNITS;
		break;
	case V4L2_CID_COLORFX:
		isppreview_get_color(&current_value);
		a->value = current_value;
		break;
	default:
		rval = -EINVAL;
		break;
	}

	return rval;
}
EXPORT_SYMBOL(isp_g_ctrl);

/**
 * isp_s_ctrl - Sets value of the desired V4L2 control.
 * @a: V4L2 control to read actual value from.
 *
 * Return 0 if successful, -EINVAL if chosen control is not found or value
 * is out of bounds, -EFAULT if copy_from_user or copy_to_user operation fails
 * from camera abstraction layer related controls or the transfered user space
 * pointer via the value field is not set properly.
 **/
int isp_s_ctrl(struct v4l2_control *a)
{
	int rval = 0;
	u8 new_value = a->value;

	if (!isp_obj.ref_count)
		return -EINVAL;

	switch (a->id) {
	case V4L2_CID_BRIGHTNESS:
		if (new_value > ISPPRV_BRIGHT_HIGH)
			rval = -EINVAL;
		else
			isppreview_update_brightness(&new_value);
		break;
	case V4L2_CID_CONTRAST:
		if (new_value > ISPPRV_CONTRAST_HIGH)
			rval = -EINVAL;
		else
			isppreview_update_contrast(&new_value);
		break;
	case V4L2_CID_COLORFX:
		if (new_value > V4L2_COLORFX_SEPIA)
			rval = -EINVAL;
		else
			isppreview_set_color(&new_value);
		break;
	default:
		rval = -EINVAL;
		break;
	}

	return rval;
}
EXPORT_SYMBOL(isp_s_ctrl);

/**
 * isp_handle_private - Handle all private ioctls for isp module.
 * @cmd: ioctl cmd value
 * @arg: ioctl arg value
 *
 * Return 0 if successful, -EINVAL if chosen cmd value is not handled or value
 * is out of bounds, -EFAULT if ioctl arg value is not valid.
 * Function simply routes the input ioctl cmd id to the appropriate handler in
 * the isp module.
 **/
int isp_handle_private(struct mutex *vdev_mutex, int cmd, void *arg)
{
	int rval = 0;

	if (!isp_obj.ref_count ||
		 (isp_obj.running == ISP_STOPPING))
		return -EINVAL;

	switch (cmd) {
	case VIDIOC_PRIVATE_ISP_CCDC_CFG:
		mutex_lock(vdev_mutex);
		rval = omap34xx_isp_ccdc_config(arg);
		mutex_unlock(vdev_mutex);
		break;
	case VIDIOC_PRIVATE_ISP_PRV_CFG:
		mutex_lock(vdev_mutex);
		rval = omap34xx_isp_preview_config(arg);
		mutex_unlock(vdev_mutex);
		break;
#if !defined(CONFIG_VIDEO_OMAP3_HP3A)
	case VIDIOC_PRIVATE_ISP_AEWB_CFG:
	{
		struct isph3a_aewb_config *params;
		params = (struct isph3a_aewb_config *)arg;
		mutex_lock(vdev_mutex);
		rval = isph3a_aewb_configure(params);
		mutex_unlock(vdev_mutex);
	}
	break;
	case VIDIOC_PRIVATE_ISP_AEWB_REQ:
	{
		struct isph3a_aewb_data *data;
		data = (struct isph3a_aewb_data *)arg;
		rval = isph3a_aewb_request_statistics(data);
	}
	break;
	case VIDIOC_PRIVATE_ISP_HIST_CFG:
	{
		struct isp_hist_config *params;
		params = (struct isp_hist_config *)arg;
		mutex_lock(vdev_mutex);
		rval = isp_hist_configure(params);
		mutex_unlock(vdev_mutex);
	}
	break;
	case VIDIOC_PRIVATE_ISP_HIST_REQ:
	{
		struct isp_hist_data *data;
		data = (struct isp_hist_data *)arg;
		mutex_lock(vdev_mutex);
		rval = isp_hist_request_statistics(data);
		mutex_unlock(vdev_mutex);
	}
	break;
	case VIDIOC_PRIVATE_ISP_AF_CFG:
	{
		struct af_configuration *params;
		params = (struct af_configuration *)arg;
		mutex_lock(vdev_mutex);
		rval = isp_af_configure(params);
		mutex_unlock(vdev_mutex);
	}
	break;
	case VIDIOC_PRIVATE_ISP_AF_REQ:
	{
		struct isp_af_data *data;
		data = (struct isp_af_data *)arg;
		rval = isp_af_request_statistics(data);
	}
	break;
#endif
	case VIDIOC_PRIVATE_ISP_RESIZE_DATA:
	{
		enum isp_running isp_state;
		mutex_lock(vdev_mutex);
		isp_state = isp_obj.running;
		if (isp_state == ISP_STOPPED) {
			isp_obj.running = ISP_FREERUNNING;
			rval = isp_resize_mem_data(arg);
			isp_obj.running = isp_state;
		} else {
			rval = -EFAULT;
			DPRINTK_ISPCTRL("isp state = %d\n", isp_state);
		}
		mutex_unlock(vdev_mutex);
	}
	break;
	case VIDIOC_PRIVATE_ISP_PROCESS_DATA:
	{
		enum isp_running isp_state;
		mutex_lock(vdev_mutex);
		isp_state = isp_obj.running;
		if (isp_state == ISP_STOPPED) {
			isp_obj.running = ISP_FREERUNNING;
			rval = isp_process_mem_data(arg);
			isp_obj.running = isp_state;
		} else {
			rval = -EFAULT;
			DPRINTK_ISPCTRL("isp state = %d\n", isp_state);
		}
		mutex_unlock(vdev_mutex);
	}
	break;
	case VIDIOC_PRIVATE_ISP_RSZ_REQ:
	{
		struct isprsz_coef *resizer_coef;
		resizer_coef = (struct isprsz_coef *) arg;
		mutex_lock(vdev_mutex);
		ispresizer_get_filter_coef(resizer_coef);
		mutex_unlock(vdev_mutex);
	}
	break;
	case VIDIOC_PRIVATE_ISP_RSZ_CFG:
	{
		struct isprsz_coef *resizer_coef;
		resizer_coef = (struct isprsz_coef *) arg;
		mutex_lock(vdev_mutex);
		ispresizer_config_filter_coef(resizer_coef);
		mutex_unlock(vdev_mutex);
	}
	break;
	case VIDIOC_PRIVATE_ISP_CCDC_BAYER_CFG:
	{
		enum ispccdc_raw_fmt raw_fmt;
		raw_fmt = (enum ispccdc_raw_fmt)(
			(struct ispccdc_color_offset *)arg)->offsetcode;
		mutex_lock(vdev_mutex);
		ispccdc_set_raw_offset(raw_fmt);
		mutex_unlock(vdev_mutex);
	}
	break;
	case VIDIOC_PRIVATE_ISP_LSC_WORKAROUND_CFG:
	{
		mutex_lock(vdev_mutex);
#ifdef USE_LSC_WORKAROUND
		if (isp_obj.running != ISP_RUNNING)
#else
		if (isp_obj.running != ISP_RUNNING && !cpu_is_omap3630())
#endif
			isp_obj.isp_lsc_workaround = (*(int *)arg ? 1 : 0);
		else
			rval = -EFAULT;
		mutex_unlock(vdev_mutex);
	}
	break;
	default:
		rval = -EINVAL;
		break;
	}

	return rval;
}
EXPORT_SYMBOL(isp_handle_private);

/**
 * isp_enum_fmt_cap - Gets more information of chosen format index and type
 * @f: Pointer to structure containing index and type of format to read from.
 *
 * Returns 0 if successful, or -EINVAL if format index or format type is
 * invalid.
 **/
int isp_enum_fmt_cap(struct v4l2_fmtdesc *f)
{
	int index = f->index;
	enum v4l2_buf_type type = f->type;
	int rval = -EINVAL;

	if (index >= NUM_ISP_CAPTURE_FORMATS)
		goto err;

	memset(f, 0, sizeof(*f));
	f->index = index;
	f->type = type;

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		rval = 0;
		break;
	default:
		goto err;
	}

	f->flags = isp_formats[index].flags;
	strncpy(f->description, isp_formats[index].description,
		sizeof(f->description));
	f->pixelformat = isp_formats[index].pixelformat;
err:
	return rval;
}
EXPORT_SYMBOL(isp_enum_fmt_cap);

/**
 * isp_g_fmt_cap - Gets current output image format.
 * @f: Pointer to V4L2 format structure to be filled with current output format
 **/
void isp_g_fmt_cap(struct v4l2_pix_format *pix)
{
	*pix = isp_obj.module.pix;
	return;
}
EXPORT_SYMBOL(isp_g_fmt_cap);

/**
 * isp_s_fmt_cap - Sets I/O formats and crop and configures pipeline in ISP
 * @f: Pointer to V4L2 format structure to be filled with current output format
 *
 * Returns 0 if successful, or return value of either isp_try_size or
 * isp_try_fmt if there is an error.
 **/
int isp_s_fmt_cap(struct v4l2_pix_format *pix_input,
		  struct v4l2_pix_format *pix_output)
{
	int crop_scaling_w = 0, crop_scaling_h = 0;
	int rval = 0;

	if (!isp_obj.ref_count)
		return -EINVAL;

	rval = isp_calc_pipeline(pix_input, pix_output);
	if (rval)
		goto out;

	rval = isp_try_size(pix_input, pix_output);
	if (rval)
		goto out;

	rval = isp_try_fmt(pix_input, pix_output);
	if (rval)
		goto out;

	if (ispcroprect.width != pix_output->width) {
		crop_scaling_w = 1;
		ispcroprect.left = 0;
		ispcroprect.width = pix_output->width;
	}

	if (ispcroprect.height != pix_output->height) {
		crop_scaling_h = 1;
		ispcroprect.top = 0;
		ispcroprect.height = pix_output->height;
	}

	isp_config_pipeline(pix_input, pix_output);
	if (isp_obj.module.isp_pipeline & OMAP_ISP_RESIZER
	    && (crop_scaling_h || crop_scaling_w))
		isp_config_crop(pix_output);

out:
	return rval;
}
EXPORT_SYMBOL(isp_s_fmt_cap);

/**
 * isp_config_crop - Configures crop parameters in isp resizer.
 * @croppix: Pointer to V4L2 pixel format structure containing crop parameters
 **/
void isp_config_crop(struct v4l2_pix_format *croppix)
{
	unsigned long org_left, num_pix, new_top;

	struct v4l2_pix_format *pix = croppix;

	cur_rect.left =  (u32)(ispcroprect.left *
		isp_obj.module.preview_output_width)   / (u32)pix->width;
	cur_rect.top =  (u32)(ispcroprect.top *
		isp_obj.module.preview_output_height) / (u32)pix->height;
	cur_rect.width =  (u32)(ispcroprect.width *
		isp_obj.module.preview_output_width) / (u32)pix->width;
	cur_rect.height =  (u32)(ispcroprect.height *
		isp_obj.module.preview_output_height) / (u32)pix->height;

	org_left = cur_rect.left;
	while (((int)cur_rect.left & 0xFFFFFFF0) != (int)cur_rect.left)
		(int)cur_rect.left--;

	num_pix = org_left - cur_rect.left;
	new_top = (int)(num_pix * 3) / 4;
	if (new_top > cur_rect.top)
		new_top = cur_rect.top;
	cur_rect.top = cur_rect.top - new_top;
	cur_rect.height = (2 * new_top) + cur_rect.height;

	cur_rect.width = cur_rect.width + (2 * num_pix);
	while (((int)cur_rect.width & 0xFFFFFFF0) != (int)cur_rect.width)
		(int)cur_rect.width--;

	isp_obj.tmp_buf_offset =
		cur_rect.left * 2 +
		isp_obj.module.preview_output_width * 2 * cur_rect.top;

	ispresizer_trycrop(cur_rect.left, cur_rect.top, cur_rect.width,
			   cur_rect.height,
			   isp_obj.module.resizer_output_width,
			   isp_obj.module.resizer_output_height);

	return;
}
EXPORT_SYMBOL(isp_config_crop);

/**
 * isp_get_buf_offset - Gets offset of start of crop.
 *
 * Returns the offset (in bytes) of the start of the crop rectangle.
 **/
unsigned long isp_get_buf_offset()
{
	return isp_obj.tmp_buf_offset;
}
EXPORT_SYMBOL(isp_get_buf_offset);

/**
 * isp_g_crop - Gets crop rectangle size and position.
 * @a: Pointer to V4L2 crop structure to be filled.
 *
 * Always returns 0.
 **/
int isp_g_crop(struct v4l2_crop *a)
{
	struct v4l2_crop *crop = a;

	crop->c = ispcroprect;

	return 0;
}
EXPORT_SYMBOL(isp_g_crop);

/**
 * isp_s_crop - Sets crop rectangle size and position and queues crop operation
 * @a: Pointer to V4L2 crop structure with desired parameters.
 * @pix: Pointer to V4L2 pixel format structure with desired parameters.
 *
 * Returns 0 if successful, or -EINVAL if crop parameters are out of bounds.
 **/
int isp_s_crop(struct v4l2_crop *a, struct v4l2_pix_format *pix)
{
	struct v4l2_crop *crop = a;
	int rval = 0;

	if (!isp_obj.ref_count)
		return -EINVAL;

	if (crop->c.left < 0)
		crop->c.left = 0;
	if (crop->c.width < 0)
		crop->c.width = 0;
	if (crop->c.top < 0)
		crop->c.top = 0;
	if (crop->c.height < 0)
		crop->c.height = 0;

	if (crop->c.left >= pix->width)
		crop->c.left = pix->width - 1;
	if (crop->c.top >= pix->height)
		crop->c.top = pix->height - 1;

	if (crop->c.left + crop->c.width > pix->width)
		crop->c.width = pix->width - crop->c.left;
	if (crop->c.top + crop->c.height > pix->height)
		crop->c.height = pix->height - crop->c.top;

	ispcroprect.left = crop->c.left;
	ispcroprect.top = crop->c.top;
	ispcroprect.width = crop->c.width;
	ispcroprect.height = crop->c.height;

	isp_config_crop(pix);

	isp_obj.module.applyCrop = 1;

	return rval;
}
EXPORT_SYMBOL(isp_s_crop);

/**
 * isp_try_fmt_cap - Tries desired input/output image formats
 * @pix_input: Pointer to V4L2 pixel format structure for input image.
 * @pix_output: Pointer to V4L2 pixel format structure for output image.
 *
 * Returns 0 if successful, or return value of either isp_try_size or
 * isp_try_fmt if there is an error.
 **/
int isp_try_fmt_cap(struct v4l2_pix_format *pix_input,
		    struct v4l2_pix_format *pix_output)
{
	int rval = 0;

	rval = isp_calc_pipeline(pix_input, pix_output);
	if (rval)
		goto out;

	rval = isp_try_size(pix_input, pix_output);
	if (rval)
		goto out;

	rval = isp_try_fmt(pix_input, pix_output);
	if (rval)
		goto out;

out:
	return rval;
}
EXPORT_SYMBOL(isp_try_fmt_cap);

/**
 * isp_try_size - Tries size configuration for I/O images of each ISP submodule
 * @pix_input: Pointer to V4L2 pixel format structure for input image.
 * @pix_output: Pointer to V4L2 pixel format structure for output image.
 *
 * Returns 0 if successful, or return value of ispccdc_try_size,
 * isppreview_try_size, or ispresizer_try_size (depending on the pipeline
 * configuration) if there is an error.
 **/
static int isp_try_size(struct v4l2_pix_format *pix_input,
			struct v4l2_pix_format *pix_output)
{
	int rval = 0;

	if (pix_output->width <= ISPRSZ_MIN_OUTPUT
	    || pix_output->height <= ISPRSZ_MIN_OUTPUT)
		return -EINVAL;
	if (pix_output->width >= ISPRSZ_MAX_OUTPUT
	    || pix_output->height > ISPRSZ_MAX_OUTPUT)
		return -EINVAL;

	isp_obj.module.ccdc_input_width = pix_input->width;
	isp_obj.module.ccdc_input_height = pix_input->height;

	if (isp_obj.module.isp_pipeline & OMAP_ISP_RESIZER) {
		isp_obj.module.resizer_output_width = pix_output->width;
		isp_obj.module.resizer_output_height = pix_output->height;
	}

	if (isp_obj.module.isp_pipeline & OMAP_ISP_PREVIEW) {
		isp_obj.module.preview_output_width = pix_output->width;
		isp_obj.module.preview_output_height = pix_output->height;
	}

	if (isp_obj.module.isp_pipeline & OMAP_ISP_CCDC) {
		u32 in_aspect_ratio = 0;
		u32 out_aspect_ratio = 0;
		u32 adjusted_height = 0;
		u32 adjusted_width = 0;

		if (pix_output->width > pix_output->height) {
			in_aspect_ratio = (pix_input->width * 256)/
				pix_input->height;
			out_aspect_ratio = (pix_output->width * 256)/
				(pix_output->height);

			if ((out_aspect_ratio - in_aspect_ratio) > 25 &&
				(out_aspect_ratio - in_aspect_ratio) < 180) {
				/* Adjusted for output aspect ratio. */
				adjusted_height = ALIGN_TO( \
					((pix_input->width*256)/
						out_aspect_ratio), 2);

				ispccdc_config_crop(0,
					(pix_input->height-adjusted_height)/2,
					adjusted_height +
					(pix_input->height-adjusted_height)/2,
					pix_input->width);
			} else {
				ispccdc_config_crop(0, 0, 0, 0);
			}
		} else {
			/* height > width */
			in_aspect_ratio = (pix_input->height * 256)/
				pix_input->width;
			out_aspect_ratio = (pix_output->height * 256)/
				(pix_output->width);

			if ((out_aspect_ratio - in_aspect_ratio) > 50) {
				/* Adjusted for output aspect ratio. */
				adjusted_width = ALIGN_TO( \
					((pix_input->height*256)/
						out_aspect_ratio), 2);

				ispccdc_config_crop(
					(pix_input->width-adjusted_width)/2,
					0,
					pix_input->height,
					adjusted_width +
					(pix_input->width-adjusted_width)/2);
			} else {
				ispccdc_config_crop(0, 0, 0, 0);
			}
		}

		rval = ispccdc_try_size(isp_obj.module.ccdc_input_width,
					isp_obj.module.ccdc_input_height,
					&isp_obj.module.ccdc_output_width,
					&isp_obj.module.ccdc_output_height);
		if (rval) {
			printk(KERN_ERR "ISP_ERR: The dimensions %dx%d are not"
			       " supported\n", pix_input->width,
			       pix_input->height);
			return rval;
		}
		pix_output->width = isp_obj.module.ccdc_output_width;
		pix_output->height = isp_obj.module.ccdc_output_height;
	}

	if (isp_obj.module.isp_pipeline & OMAP_ISP_PREVIEW) {
		isp_obj.module.preview_input_width =
			isp_obj.module.ccdc_output_width;
		isp_obj.module.preview_input_height =
			isp_obj.module.ccdc_output_height;
		rval = isppreview_try_size(
			isp_obj.module.preview_input_width,
			isp_obj.module.preview_input_height,
			&isp_obj.module.preview_output_width,
			&isp_obj.module.preview_output_height);
		if (rval) {
			printk(KERN_ERR "ISP_ERR: The dimensions %dx%d are not"
			       " supported\n", pix_input->width,
			       pix_input->height);
			return rval;
		}
		pix_output->width = isp_obj.module.preview_output_width;
		pix_output->height = isp_obj.module.preview_output_height;
	}

	if (isp_obj.module.isp_pipeline & OMAP_ISP_RESIZER) {
		isp_obj.module.resizer_input_width =
			isp_obj.module.preview_output_width;
		isp_obj.module.resizer_input_height =
			isp_obj.module.preview_output_height;
		rval = ispresizer_try_size(
			&isp_obj.module.resizer_input_width,
			&isp_obj.module.resizer_input_height,
			&isp_obj.module.resizer_output_width,
			&isp_obj.module.resizer_output_height);
		if (rval) {
			printk(KERN_ERR "ISP_ERR: The dimensions %dx%d are not"
			       " supported\n", pix_input->width,
			       pix_input->height);
			return rval;
		}
		pix_output->width = isp_obj.module.resizer_output_width;
		pix_output->height = isp_obj.module.resizer_output_height;
	}

	return rval;
}

/**
 * isp_try_fmt - Validates input/output format parameters.
 * @pix_input: Pointer to V4L2 pixel format structure for input image.
 * @pix_output: Pointer to V4L2 pixel format structure for output image.
 *
 * Always returns 0.
 **/
int isp_try_fmt(struct v4l2_pix_format *pix_input,
		struct v4l2_pix_format *pix_output)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_ISP_CAPTURE_FORMATS; ifmt++) {
		if (pix_output->pixelformat == isp_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_ISP_CAPTURE_FORMATS)
		ifmt = 1;
	pix_output->pixelformat = isp_formats[ifmt].pixelformat;
	pix_output->field = V4L2_FIELD_NONE;
	pix_output->bytesperline = pix_output->width * ISP_BYTES_PER_PIXEL;
	pix_output->sizeimage =
		PAGE_ALIGN(pix_output->bytesperline * pix_output->height);
	pix_output->priv = 0;
	switch (pix_output->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		pix_output->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	default:
		pix_output->colorspace = V4L2_COLORSPACE_SRGB;
	}

	isp_obj.module.pix.pixelformat = pix_output->pixelformat;
	isp_obj.module.pix.width = pix_output->width;
	isp_obj.module.pix.height = pix_output->height;
	isp_obj.module.pix.field = pix_output->field;
	isp_obj.module.pix.bytesperline = pix_output->bytesperline;
	isp_obj.module.pix.sizeimage = pix_output->sizeimage;
	isp_obj.module.pix.priv = pix_output->priv;
	isp_obj.module.pix.colorspace = pix_output->colorspace;

	return 0;
}
EXPORT_SYMBOL(isp_try_fmt);

/**
 * isp_save_ctx - Saves ISP, CCDC, HIST, H3A, PREV, RESZ & MMU context.
 *
 * Routine for saving the context of each module in the ISP.
 * CCDC, HIST, H3A, PREV, RESZ and MMU.
 **/
static void isp_save_ctx(void)
{
	isp_save_context(isp_reg_list);
	ispccdc_save_context();
	ispmmu_save_context();
#if !defined(CONFIG_VIDEO_OMAP3_HP3A)
	isphist_save_context();
	isph3a_save_context();
#endif
	isppreview_save_context();
	ispresizer_save_context();
	ispcsi2_save_context();
}

/**
 * isp_restore_ctx - Restores ISP, CCDC, HIST, H3A, PREV, RESZ & MMU context.
 *
 * Routine for restoring the context of each module in the ISP.
 * CCDC, HIST, H3A, PREV, RESZ and MMU.
 **/
static void isp_restore_ctx(void)
{
	isp_restore_context(isp_reg_list);
	ispccdc_restore_context();
	ispmmu_restore_context();
#if !defined(CONFIG_VIDEO_OMAP3_HP3A)
	isphist_restore_context();
	isph3a_restore_context();
#endif
	isppreview_restore_context();
	ispresizer_restore_context();
	ispcsi2_restore_context();
}

static int isp_enable_clocks(void)
{
	int r;

	r = clk_enable(isp_obj.cam_ick);
	if (r) {
		DPRINTK_ISPCTRL("ISP_ERR: clk_en for ick failed\n");
		goto out_clk_enable_ick;
	}
	r = clk_enable(isp_obj.cam_mclk);
	if (r) {
		DPRINTK_ISPCTRL("ISP_ERR: clk_en for mclk failed\n");
		goto out_clk_enable_mclk;
	}
	r = clk_enable(isp_obj.csi2_fck);
	if (r) {
		DPRINTK_ISPCTRL("ISP_ERR: clk_en for csi2_fclk"
				" failed\n");
		goto out_clk_enable_csi2_fclk;
	}
	return 0;

out_clk_enable_csi2_fclk:
	clk_disable(isp_obj.cam_mclk);
out_clk_enable_mclk:
	clk_disable(isp_obj.cam_ick);
out_clk_enable_ick:
	return r;
}

static void isp_disable_clocks(void)
{
	clk_disable(isp_obj.cam_ick);
	clk_disable(isp_obj.cam_mclk);
	clk_disable(isp_obj.csi2_fck);
}

/**
 * isp_get - Adquires the ISP resource.
 *
 * Initializes the clocks for the first acquire.
 **/
int isp_get(void)
{
	static int has_context;
	int ret_err = 0;

	if (omap3isp == NULL)
		return -EBUSY;

	DPRINTK_ISPCTRL("isp_get: old %d\n", isp_obj.ref_count);
	mutex_lock(&(isp_obj.isp_mutex));
	if ((isp_obj.ref_count++) == 0) {
		ret_err = isp_enable_clocks();
		if (ret_err)
			goto out_err;
		/* We don't want to restore context before saving it! */
		if (has_context)
			isp_restore_ctx();
		else
			has_context = 1;
		/* No standy */
		isp_power_settings(1);
		enable_irq(omap3isp->irq);
#if defined(CONFIG_VIDEO_OMAP3_HP3A)
			hp3a_hw_enabled(1);
#endif
	}
	mutex_unlock(&(isp_obj.isp_mutex));

	DPRINTK_ISPCTRL("isp_get: new %d\n", isp_obj.ref_count);
	return isp_obj.ref_count;

out_err:
	--isp_obj.ref_count;
	mutex_unlock(&(isp_obj.isp_mutex));
	return ret_err;
}
EXPORT_SYMBOL(isp_get);

/**
 * isp_put - Releases the ISP resource.
 *
 * Releases the clocks also for the last release.
 **/
int isp_put(void)
{
	if (omap3isp == NULL)
		return -EBUSY;

	DPRINTK_ISPCTRL("isp_put: old %d\n", isp_obj.ref_count);
	mutex_lock(&(isp_obj.isp_mutex));
	if (isp_obj.ref_count > 0 &&
		(--isp_obj.ref_count == 0)) {
#if defined(CONFIG_VIDEO_OMAP3_HP3A)
		hp3a_hw_enabled(0);
#endif
		disable_irq_nosync(omap3isp->irq);
		isp_save_ctx();
		isp_release_resources();
		isp_obj.module.isp_pipeline = 0;
		/*force ISP standby, smart standby disabled*/
		isp_power_settings(0);
		isp_disable_clocks();
		memset(&ispcroprect, 0, sizeof(ispcroprect));
		memset(&cur_rect, 0, sizeof(cur_rect));
	}
	mutex_unlock(&(isp_obj.isp_mutex));
	DPRINTK_ISPCTRL("isp_put: new %d\n", isp_obj.ref_count);
	return isp_obj.ref_count;
}
EXPORT_SYMBOL(isp_put);

/**
 * isp_save_context - Saves the values of the ISP module registers.
 * @reg_list: Structure containing pairs of register address and value to
 *            modify on OMAP.
 **/
void isp_save_context(struct isp_reg *reg_list)
{
	struct isp_reg *next = reg_list;

	for (; next->reg != ISP_TOK_TERM; next++)
		next->val = isp_reg_readl(next->mmio_range, next->reg);
}
EXPORT_SYMBOL(isp_save_context);

/**
 * isp_restore_context - Restores the values of the ISP module registers.
 * @reg_list: Structure containing pairs of register address and value to
 *            modify on OMAP.
 **/
void isp_restore_context(struct isp_reg *reg_list)
{
	struct isp_reg *next = reg_list;

	for (; next->reg != ISP_TOK_TERM; next++)
		isp_reg_writel(next->val, next->mmio_range, next->reg);
}
EXPORT_SYMBOL(isp_restore_context);

static int isp_remove(struct platform_device *pdev)
{
	struct isp_device *isp = platform_get_drvdata(pdev);
	int i;

	isp_tmp_buf_free();

#ifdef CONFIG_VIDEO_OMAP3_HP3A
	isp_csi2_cleanup();
	isp_resizer_cleanup();
	isp_preview_cleanup();
	ispmmu_cleanup();
	isp_ccdc_cleanup();
#else
	isp_csi2_cleanup();
	isp_af_exit();
	isp_resizer_cleanup();
	isp_preview_cleanup();
	ispmmu_cleanup();
	isph3a_aewb_cleanup();
	isp_hist_cleanup();
	isp_ccdc_cleanup();
#endif

	if (!isp)
		return 0;

	clk_put(isp_obj.cam_ick);
	clk_put(isp_obj.cam_mclk);
	clk_put(isp_obj.csi2_fck);

	free_irq(isp->irq, &isp_obj);

	for (i = 0; i <= OMAP3_ISP_IOMEM_CSI2PHY; i++) {
		if (isp->mmio_base[i]) {
			iounmap((void *)isp->mmio_base[i]);
			isp->mmio_base[i] = 0;
		}

		if (isp->mmio_base_phys[i]) {
			release_mem_region(isp->mmio_base_phys[i],
					   isp->mmio_size[i]);
			isp->mmio_base_phys[i] = 0;
		}
	}

	omap3isp = NULL;

	kfree(isp);

	return 0;
}

#ifdef CONFIG_PM

static int isp_suspend(struct platform_device *pdev, pm_message_t state)
{
	int reset;

	mutex_lock(&(isp_obj.isp_mutex));
	DPRINTK_ISPCTRL("isp_suspend: starting\n");
	if (isp_obj.ref_count == 0)
		goto out;

	isp_disable_interrupts();
	reset = isp_suspend_modules();
	isp_save_ctx();
	if (reset)
		isp_reset();

	isp_disable_clocks();

out:
	DPRINTK_ISPCTRL("isp_suspend: done\n");
	mutex_unlock(&(isp_obj.isp_mutex));
	return 0;
}

static int isp_resume(struct platform_device *pdev)
{
	int ret_err = 0;

	DPRINTK_ISPCTRL("isp_resume: starting\n");

	if (omap3isp == NULL)
		goto out;

	if (isp_obj.ref_count == 0)
		goto out;

	ret_err = isp_enable_clocks();
	if (ret_err)
		goto out;
	isp_restore_ctx();
	isp_resume_modules();
	isp_enable_interrupts();
	isp_start();

out:
	DPRINTK_ISPCTRL("isp_resume: done \n");
	return ret_err;
}

#else

#define isp_suspend	NULL
#define isp_resume	NULL

#endif /* CONFIG_PM */


static int isp_probe(struct platform_device *pdev)
{
	struct isp_device *isp;
	int ret_err = 0;
	int i;

	isp = kzalloc(sizeof(*isp), GFP_KERNEL);
	if (!isp) {
		dev_err(&pdev->dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, isp);

	isp->dev = &pdev->dev;

	for (i = 0; i <= OMAP3_ISP_IOMEM_CSI2PHY; i++) {
		struct resource *mem;
		/* request the mem region for the camera registers */
		mem = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!mem) {
			dev_err(isp->dev, "no mem resource?\n");
			return -ENODEV;
		}

		if (!request_mem_region(mem->start, mem->end - mem->start + 1,
					pdev->name)) {
			dev_err(isp->dev,
				"cannot reserve camera register I/O region\n");
			return -ENODEV;

		}
		isp->mmio_base_phys[i] = mem->start;
		isp->mmio_size[i] = mem->end - mem->start + 1;

		/* map the region */
		isp->mmio_base[i] = (unsigned long)
			ioremap_nocache(isp->mmio_base_phys[i],
					isp->mmio_size[i]);
		if (!isp->mmio_base[i]) {
			dev_err(isp->dev,
				"cannot map camera register I/O region\n");
			return -ENODEV;
		}
	}

	isp->irq = platform_get_irq(pdev, 0);
	if (isp->irq <= 0) {
		dev_err(isp->dev, "no irq for camera?\n");
		return -ENODEV;
	}

	isp_obj.mclk_hz = CM_CAM_MCLK_HZ;
	isp_obj.mclk_src_div = 4;

	isp_obj.cam_ick = clk_get(&camera_dev, "cam_ick");
	if (IS_ERR(isp_obj.cam_ick)) {
		DPRINTK_ISPCTRL("ISP_ERR: clk_get for "
				"cam_ick failed\n");
		return PTR_ERR(isp_obj.cam_ick);
	}
	isp_obj.cam_mclk = clk_get(&camera_dev, "cam_mclk");
	if (IS_ERR(isp_obj.cam_mclk)) {
		DPRINTK_ISPCTRL("ISP_ERR: clk_get for "
				"cam_mclk failed\n");
		ret_err = PTR_ERR(isp_obj.cam_mclk);
		goto out_clk_get_mclk;
	}
	isp_obj.csi2_fck = clk_get(&camera_dev, "csi2_96m_fck");
	if (IS_ERR(isp_obj.csi2_fck)) {
		DPRINTK_ISPCTRL("ISP_ERR: clk_get for csi2_fclk"
				" failed\n");
		ret_err = PTR_ERR(isp_obj.csi2_fck);
		goto out_clk_get_csi2_fclk;
	}

	if (request_irq(isp->irq, omap34xx_isp_isr, IRQF_SHARED,
			"Omap 3 Camera ISP", &isp_obj)) {
		DPRINTK_ISPCTRL("Could not install ISR\n");
		ret_err = -EINVAL;
		goto out_request_irq;
	}

	disable_irq(isp->irq);

	isp_obj.ref_count = 0;
	isp_obj.tmp_buf = 0;
	isp_obj.tmp_buf_size = 0;
	isp_obj.running = ISP_STOPPED;
#ifdef USE_LSC_WORKAROUND
	isp_obj.isp_lsc_workaround = 1;
#else
	if (cpu_is_omap3630())
		isp_obj.isp_lsc_workaround = 0;
	else
		isp_obj.isp_lsc_workaround = 1;
#endif

	mutex_init(&(isp_obj.isp_mutex));
	spin_lock_init(&isp_obj.lock);
	spin_lock_init(&isp_obj.bufs.lock);

	omap3isp = isp;

	ret_err = ispmmu_init();
	if (ret_err)
		goto out_ispmmu_init;

	ret_err = isp_tmp_buf_alloc(ISP_LSC_MEMORY);
	if (ret_err) {
		dev_err(isp->dev, "Couldn't allocate lsc"
				  " workaround memory\n");
		goto out_tmp_buf_alloc;
	}

#if defined(CONFIG_VIDEO_OMAP3_HP3A)
	isp_ccdc_init();
	isp_preview_init();
	isp_resizer_init();
	isp_csi2_init();
	isp_mem_process_init();
#else
	isp_ccdc_init();
	isp_hist_init();
	isph3a_aewb_init();
	isp_preview_init();
	isp_resizer_init();
	isp_af_init();
	isp_csi2_init();
#endif

	isp_get();
	/* Get ISP revision */
	isp->revision = isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_REVISION);
	dev_info(isp->dev, "Revision %d.%d found\n",
		 (isp->revision & 0xF0) >> 4, isp->revision & 0xF);
	isp_power_settings(1);
	isp_put();

#if !defined(CONFIG_VIDEO_OMAP3_HP3A)
	isph3a_notify(1);
	isp_af_notify(1);
#endif
	return 0;

out_tmp_buf_alloc:
	ispmmu_cleanup();
out_ispmmu_init:
	omap3isp = NULL;
	free_irq(isp->irq, &isp_obj);
out_request_irq:
	clk_put(isp_obj.csi2_fck);
out_clk_get_csi2_fclk:
	clk_put(isp_obj.cam_mclk);
out_clk_get_mclk:
	clk_put(isp_obj.cam_ick);

	return ret_err;
}

static struct platform_driver omap3isp_driver = {
	.probe = isp_probe,
	.remove = isp_remove,
	.suspend = isp_suspend,
	.resume = isp_resume,
	.driver = {
		.name = "omap3isp",
	},
};

/**
 * isp_init - ISP module initialization.
 **/
static int __init isp_init(void)
{
	return platform_driver_register(&omap3isp_driver);
}

/**
 * isp_cleanup - ISP module cleanup.
 **/
static void __exit isp_cleanup(void)
{
	platform_driver_unregister(&omap3isp_driver);
}

/**
 * isp_print_status - Prints the values of the ISP Control Module registers
 *
 * Also prints other debug information stored in the ISP module structure.
 **/
void isp_print_status(void)
{
	if (!is_ispctrl_debug_enabled())
		return;

	DPRINTK_ISPCTRL("###ISP_CTRL=0x%x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_CTRL));
	DPRINTK_ISPCTRL("###ISP_TCTRL_CTRL=0x%x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_CTRL));
	DPRINTK_ISPCTRL("###ISP_SYSCONFIG=0x%x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG));
	DPRINTK_ISPCTRL("###ISP_SYSSTATUS=0x%x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_SYSSTATUS));
	DPRINTK_ISPCTRL("###ISP_IRQ0ENABLE=0x%x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE));
	DPRINTK_ISPCTRL("###ISP_IRQ0STATUS=0x%x\n",
			isp_reg_readl(OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS));
}
EXPORT_SYMBOL(isp_print_status);

module_init(isp_init);
module_exit(isp_cleanup);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("ISP Control Module Library");
MODULE_LICENSE("GPL");
