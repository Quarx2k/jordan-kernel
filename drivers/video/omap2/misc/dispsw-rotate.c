/*
 * drivers/video/omap2/misc/dispsw-rotate.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/mm.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/gfp.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <plat/display.h>
#include <plat/dma.h>
#include <plat/vrfb.h>
#include <plat/dispsw.h>
#include <asm/processor.h>
#include <asm/cacheflush.h>
#include <asm/page.h>

#include "dispsw-rotate.h"

#define VRFB_TX_TIMEOUT		1000

/*=== Local Functions ==================================================*/

int dispsw_rotate_alloc(unsigned long *paddr, void **vaddr, u32 size)
{
	void *page_addr;

	size = PAGE_ALIGN(size);

	page_addr = alloc_pages_exact(size, GFP_KERNEL);
	if (!page_addr) {
		printk(KERN_ERR "Failed to allocate pages!\n");
		return -ENOMEM;
	}

	*paddr = virt_to_phys(page_addr);
	*vaddr = ioremap_cached(*paddr, size);

	DBG("alloc: page/%08x; phy/%08lx; virt/%08x; size/%x\n",
		(unsigned int) page_addr, *paddr,
		(unsigned int) *vaddr, size);

	return 0;
}

void dispsw_rotate_free(unsigned long paddr, void *vaddr, u32 size)
{
	void *page_addr;

	size = PAGE_ALIGN(size);
	page_addr = __va((void *)paddr);

	free_pages_exact(page_addr, size);

	iounmap(vaddr);
}

static int dispsw_rotate_format_bytespp(enum omap_color_mode fmt)
{
	int bpp = 4;

	switch (fmt) {
	case OMAP_DSS_COLOR_RGB24U:
	case OMAP_DSS_COLOR_ARGB32:
	case OMAP_DSS_COLOR_RGBA32:
	case OMAP_DSS_COLOR_RGBX32:
		bpp = 4;
		break;
	case OMAP_DSS_COLOR_RGB16:
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
		bpp = 2;
		break;
	default:
		printk(KERN_ERR "Unknown Format (%d)\n", fmt);
		break;
	}

	return bpp;
}

static void dispsw_rotate_calc_offset(struct dispsw_rotate_data *rot,
				int bpp, int ow, int oh)
{
	/* It is assumed that the caller has locked the vout mutex */

	switch (rot->dss_rot)	{
	case 1: /* 90 degrees */
		rot->buf_offset = oh * bpp;
		break;
	case 2: /* 180 degrees */
		rot->buf_offset = (oh * OMAP_VRFB_LINE_LEN * bpp)
				+ (ow * bpp);
		break;
	case 3: /* 270 degrees */
		rot->buf_offset = (ow * OMAP_VRFB_LINE_LEN * bpp);
		break;
	default:
	case 0: /* 0 degrees */
		rot->buf_offset = 0;
		break;
	}
}

/* This functions wakes up the application once the DMA transfer to
 * VRFB space is completed.
 */
static void dispsw_vrfb_dma_cb(int lch, u16 ch_status, void *data)
{
	struct dispsw_vrfb *vrfb;

	vrfb = (struct dispsw_vrfb *) data;

	vrfb->dma_complete = true;
	wake_up_interruptible(&vrfb->wait);
}

static int dispsw_rotate_acquire_vrfb(struct dispsw_rotate_data *rot)
{
	int rc = 0;
	int size;
	int max_pixels;
	u16 w, h;
	struct dispsw_vrfb *vrfb;

	/* It is assumed that the caller has locked the vout mutex */

	vrfb = &rot->vrfb;
	vrfb->dma_id = OMAP_DMA_NO_DEVICE;
	vrfb->dma_ch = -1;
	vrfb->next = 0;

	rc = omap_vrfb_request_ctx(&vrfb->ctx[0]);
	if (rc != 0) {
		printk(KERN_ERR "VRFB context allocation 0 failed %d\n", rc);
		goto failed_ctx0;
	}

	rc = omap_vrfb_request_ctx(&vrfb->ctx[1]);
	if (rc != 0) {
		printk(KERN_ERR "VRFB context allocation 1 failed %d\n", rc);
		goto failed_ctx1;
	}

	w = rot->max_w;
	h = rot->max_h;
	max_pixels = w * h;
	w += 32; /* Oversize as typical for VRFB */
	h += 32;
	size = PAGE_ALIGN(w * h * (rot->max_buf_size / max_pixels));
	vrfb->size = size;

	rc = dispsw_rotate_alloc(&vrfb->paddr[0], &vrfb->vaddr[0], size);
	if (rc != 0) {
		printk(KERN_ERR "VRFB buffer alloc 0 failed %d\n", rc);
		goto failed_mem0;
	}

	rc = dispsw_rotate_alloc(&vrfb->paddr[1], &vrfb->vaddr[1], size);
	if (rc != 0) {
		printk(KERN_ERR "VRFB buffer alloc 1 failed %d\n", rc);
		goto failed_mem1;
	}

	rc = omap_request_dma(vrfb->dma_id, "VRFB DMA",
				dispsw_vrfb_dma_cb,
				(void *)vrfb,
				&vrfb->dma_ch);
	if (rc != 0) {
		printk(KERN_ERR "No VRFB DMA channel for rotation\n");
		goto failed_dma;
	}

	init_waitqueue_head(&vrfb->wait);

	return 0;

failed_dma:
	dispsw_rotate_free(vrfb->paddr[1], vrfb->vaddr[1], size);
failed_mem1:
	dispsw_rotate_free(vrfb->paddr[0], vrfb->vaddr[0], size);
failed_mem0:
	omap_vrfb_release_ctx(&vrfb->ctx[1]);
failed_ctx1:
	omap_vrfb_release_ctx(&vrfb->ctx[0]);
failed_ctx0:
	return rc;
}

static void dispsw_rotate_release_vrfb(struct dispsw_rotate_data *rot)
{
	int size;
	struct dispsw_vrfb *vrfb;

	/* It is assumed that the caller has locked the vout mutex */

	vrfb = &rot->vrfb;
	omap_free_dma(vrfb->dma_ch);
	/* FIXME: de-init the wait queue? */

	size = vrfb->size;
	dispsw_rotate_free(vrfb->paddr[0], vrfb->vaddr[0], size);
	dispsw_rotate_free(vrfb->paddr[1], vrfb->vaddr[1], size);

	omap_vrfb_release_ctx(&vrfb->ctx[0]);
	omap_vrfb_release_ctx(&vrfb->ctx[1]);
}

static unsigned long dispsw_rotate_perform_vrfb_dma(
			struct dispsw_rotate_data *rot,	unsigned long paddr)
{
	struct dispsw_vrfb *vrfb;
	unsigned long src_paddr;
	unsigned long dst_paddr;
	unsigned long raddr = 0;

	/* It is assumed that the caller has locked the vout mutex */

	vrfb = &rot->vrfb;

	if (vrfb->need_cfg) {
		enum omap_color_mode fmt;
		bool yuv_mode;
		int bytespp;
		int w, h;

		w = rot->buf_w;
		h = rot->buf_h;
		fmt = rot->buf_fmt;
		yuv_mode = fmt == OMAP_DSS_COLOR_YUV2 ||
				fmt == OMAP_DSS_COLOR_UYVY;
		bytespp = dispsw_rotate_format_bytespp(fmt);
		omap_vrfb_setup(&vrfb->ctx[0], vrfb->paddr[0], w, h, bytespp,
							yuv_mode, rot->dss_rot);
		omap_vrfb_setup(&vrfb->ctx[1], vrfb->paddr[1], w, h, bytespp,
							yuv_mode, rot->dss_rot);

		dispsw_rotate_calc_offset(rot, vrfb->ctx[0].bytespp,
				vrfb->ctx[0].xoffset, vrfb->ctx[0].yoffset);

		vrfb->en = (w * bytespp) / 4; /* 32 bit ES */
		vrfb->fn = h;
		vrfb->dst_ei = 1;
		if (fmt == OMAP_DSS_COLOR_YUV2 || fmt == OMAP_DSS_COLOR_UYVY) {
			vrfb->dst_fi = (OMAP_VRFB_LINE_LEN * bytespp * 2)
							- (vrfb->en * 4) + 1;
		} else {
			vrfb->dst_fi = (OMAP_VRFB_LINE_LEN * bytespp)
							- (vrfb->en * 4) + 1;
		}
	}

	src_paddr = paddr;
	dst_paddr = vrfb->ctx[vrfb->next].paddr[rot->dss_rot]
						+ rot->buf_offset;

	omap_set_dma_transfer_params(vrfb->dma_ch, OMAP_DMA_DATA_TYPE_S32,
				vrfb->en, vrfb->fn, OMAP_DMA_SYNC_ELEMENT,
				vrfb->dma_id, 0x0);
	omap_set_dma_src_params(vrfb->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
				src_paddr, 0, 0);
	omap_set_dma_src_burst_mode(vrfb->dma_ch, OMAP_DMA_DATA_BURST_16);
	omap_set_dma_dest_params(vrfb->dma_ch, 0, OMAP_DMA_AMODE_DOUBLE_IDX,
				dst_paddr, vrfb->dst_ei, vrfb->dst_fi);
	omap_set_dma_dest_burst_mode(vrfb->dma_ch, OMAP_DMA_DATA_BURST_16);
	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0x20, 0);

	vrfb->dma_complete = false;
	omap_start_dma(vrfb->dma_ch);
	wait_event_interruptible_timeout(vrfb->wait, vrfb->dma_complete,
							VRFB_TX_TIMEOUT);

	if (!vrfb->dma_complete) {
		printk(KERN_ERR "VRFB DMA timeout\n");
		omap_stop_dma(vrfb->dma_ch);
		return 0;
	}

	raddr = vrfb->ctx[vrfb->next].paddr[0] + rot->buf_offset;

	vrfb->next = (vrfb->next) ? 0 : 1;

	return raddr;
}

/*=== Public Functions =================================================*/

int dispsw_rotate_init(struct dispsw_rotate_data *rot, int max_w, int max_h,
							int max_buf_size)
{
	if (!rot)
		return -EINVAL;

	if (max_w > OMAP_VRFB_LINE_LEN || max_h > OMAP_VRFB_LINE_LEN)
		return -EINVAL;

	memset(rot, 0, sizeof(struct dispsw_rotate_data));

	mutex_init(&rot->mtx);

	rot->max_w = max_w;
	rot->max_h = max_h;
	rot->max_buf_size = max_buf_size;

	return dispsw_rotate_acquire_vrfb(rot);
}

void dispsw_rotate_remove(struct dispsw_rotate_data *rot)
{
	mutex_lock(&rot->mtx);

	dispsw_rotate_release_vrfb(rot);

	mutex_unlock(&rot->mtx);
}

int dispsw_rotate_set_params(struct dispsw_rotate_data *rot, int width,
			int height, int stride, int rotate,
			enum omap_color_mode format)
{
	int rc = 0;
	int size;

	mutex_lock(&rot->mtx);

	size = width * height * dispsw_rotate_format_bytespp(format);

	if (size > rot->max_buf_size ||
	    width > stride ||
	    stride > OMAP_VRFB_LINE_LEN) {
		rc = -EINVAL;
		goto exit;
	}

	if (rotate < 1 || rotate > 3) { /* Only 90, 180, 270 supported */
		rc = -EINVAL;
		goto exit;
	}

	if (rot->buf_w != width ||
	    rot->buf_h != height ||
	    rot->buf_fmt != format ||
	    rot->buf_rot != rotate)
		rot->vrfb.need_cfg = true;

	rot->buf_w = width;
	rot->buf_h = height;
	rot->buf_stride = stride;
	rot->buf_rot = rotate;
	rot->buf_fmt = format;

	switch (rot->buf_rot) {
	case 1:
		rot->dss_rot = 3;
		break;
	case 3:
		rot->dss_rot = 1;
		break;
	default:
		rot->dss_rot = rot->buf_rot;
		break;
	}

exit:
	mutex_unlock(&rot->mtx);

	return rc;
}

unsigned long dispsw_rotate_perform_rotation(struct dispsw_rotate_data *rot,
							unsigned long paddr)
{
	unsigned long raddr = 0;

	mutex_lock(&rot->mtx);

	if (!rot->buf_w || !rot->buf_h)
		goto exit;

	raddr = dispsw_rotate_perform_vrfb_dma(rot, paddr);

	mutex_unlock(&rot->mtx);

	return raddr;

exit:
	mutex_unlock(&rot->mtx);

	return 0;
}

