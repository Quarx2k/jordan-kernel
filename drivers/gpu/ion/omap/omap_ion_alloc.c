/*
 * drivers/gpu/ion/omap_ion_alloc.c
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/ion.h>
#include <linux/omap_ion.h>
#include <mach/tiler.h>
#include <asm/mach/map.h>
#include <asm/page.h>

#ifdef CONFIG_OMAP2_VRFB

#include <plat/vrfb.h>
#include "../ion_priv.h"

#define VRFB_NUM_SLOTS	12
#define VRFB_LINE_LENGTH 2048

/*
 * Display rotation support
 */
#define VRFB_ROTATE_UR      0
#define VRFB_ROTATE_CW      1
#define VRFB_ROTATE_UD      2
#define VRFB_ROTATE_CCW     3

static int have_vrfb_ctx;

struct ion_vrfb {
	struct vrfb vrfb_context;
	__u32 ba;
	__u8 ismapped;
} ion_vrfb_t[VRFB_NUM_SLOTS];

#endif

#define BITS_PER_PIXEL  8

int omap_ion_mem_alloc(struct ion_client *client,
		struct omap_ion_tiler_alloc_data *sAllocData)
{
	int ret = 0;
	struct ion_allocation_data data;

	data.len = (sAllocData->w * sAllocData->h * sAllocData->fmt) /
					BITS_PER_PIXEL;
	data.align = 0;
	data.flags = 1 << ION_HEAP_TYPE_CARVEOUT;

	sAllocData->handle = ion_alloc(client,
			PAGE_ALIGN(data.len), data.align, data.flags);
	if (!sAllocData->handle) {
		pr_err("%s: Failed to allocate via ion_alloc\n", __func__);
		ret = -ENOMEM;
	}

	return ret;
}

int omap_ion_get_pages(struct ion_client *client, struct ion_handle *handle,
		int *n, unsigned long *ion_addr,
		struct omap_ion_tiler_alloc_data *sAllocData)
{
	size_t len;

	/* validate that the handle exists in this client */
	ion_phys(client, handle, ion_addr, &len);
	*n = len / PAGE_SIZE; /* Number of pages */

	return 0;
}

#ifdef CONFIG_OMAP2_VRFB

void omap_free_vrfb_buffer(__u32 paddr)
{
	int j = 0;

	if (!have_vrfb_ctx)
		return;

	for (j = 0; j < VRFB_NUM_SLOTS; j++) {
		if (ion_vrfb_t[j].ba == paddr) {
			omap_vrfb_release_ctx(&ion_vrfb_t[j].vrfb_context);
			ion_vrfb_t[j].ba = 0;
			ion_vrfb_t[j].ismapped = 0;
			have_vrfb_ctx--;
			break;
		}
	}

}

void omap_get_vrfb_buffer(__u32 paddr)
{
	int i = 0;
	int j = 0;

	if (have_vrfb_ctx >= VRFB_NUM_SLOTS)
		return;

	for (i = 0; i < VRFB_NUM_SLOTS; i++) {
		if (ion_vrfb_t[i].ba == 0) {
			if (omap_vrfb_request_ctx
				(&ion_vrfb_t[i].vrfb_context)) {
				pr_err("%s:VRFB allocation failed\n", __func__);
				for (j = 0; j < VRFB_NUM_SLOTS; j++)
					if (ion_vrfb_t[j].ba) {
						omap_vrfb_release_ctx(
						&ion_vrfb_t[j].vrfb_context);
						ion_vrfb_t[j].ba = 0;
						ion_vrfb_t[j].ismapped = 0;
						have_vrfb_ctx--;
				}
			return ;
			}
			ion_vrfb_t[i].ba = paddr;
			ion_vrfb_t[i].ismapped = 0;
			have_vrfb_ctx++;
			break;
		}
	}

	return;
}

static int get_bpp(int color_mode)
{
	int bytes_per_pixel = 0;
	int bpp = 0;

	switch (color_mode) {
	case OMAP_DSS_COLOR_NV12:
		bpp = 8;
		break;

	case OMAP_DSS_COLOR_CLUT1:
	case OMAP_DSS_COLOR_CLUT2:
	case OMAP_DSS_COLOR_CLUT4:
	case OMAP_DSS_COLOR_CLUT8:
	case OMAP_DSS_COLOR_RGB16:
	case OMAP_DSS_COLOR_ARGB16:
	case OMAP_DSS_COLOR_YUV2:
	case OMAP_DSS_COLOR_UYVY:
	case OMAP_DSS_COLOR_RGBA16:
	case OMAP_DSS_COLOR_RGB12U:
	case OMAP_DSS_COLOR_RGBX16:
	case OMAP_DSS_COLOR_ARGB16_1555:
	case OMAP_DSS_COLOR_XRGB16_1555:
		bpp = 16;
		break;

	case OMAP_DSS_COLOR_RGB24P:
		bpp = 24;
		break;

	case OMAP_DSS_COLOR_RGB24U:
	case OMAP_DSS_COLOR_ARGB32:
	case OMAP_DSS_COLOR_RGBA32:
	case OMAP_DSS_COLOR_RGBX32:
		bpp = 32;
		break;

	default:
		BUG();
	}
	bytes_per_pixel = bpp >> 3;

	return bytes_per_pixel;
}

static int is_yuv_mode(int mode)
{
	int ret  = 0;

	if (mode == OMAP_DSS_COLOR_YUV2 || mode == OMAP_DSS_COLOR_UYVY)
		ret = 1;
	else
		ret = 0;

	return ret;
}

static int check_vrfb_params(struct vrfb *vrfb_context,
				struct dss2_ovl_info *oi)
{
	int ret = 0;

	if (vrfb_context->yuv_mode != is_yuv_mode(oi->cfg.color_mode))
		ret = 1;
	else if (((vrfb_context->xres * 2) !=  oi->cfg.crop.w) ||
			(vrfb_context->yres != oi->cfg.crop.h))
		ret = 1;

	return ret;
}

static unsigned omap_get_vrfb_offset(struct vrfb *vrfb, int rot)
{
	unsigned offset;

	switch (rot) {
	case VRFB_ROTATE_UR:
		offset = 0;
		break;
	case VRFB_ROTATE_CW:
		offset = vrfb->yoffset;
		break;
	case VRFB_ROTATE_UD:
		offset = vrfb->yoffset * VRFB_LINE_LENGTH + vrfb->xoffset;
		break;
	case VRFB_ROTATE_CCW:
		offset = vrfb->xoffset * VRFB_LINE_LENGTH;
		break;
	default:
		BUG();
	}

	offset *= vrfb->bytespp;

	return offset;
}


int omap_setup_vrfb_buffer(struct dss2_ovl_info *ovl_info)
{
	int i = 0;
	int mode = ovl_info->cfg.color_mode;
	int got_vrfb_mapped_buf = 0;

	if (!have_vrfb_ctx) {
		pr_err("%s: No VRFB context available for setup\n", __func__);
		return 1;
	}

	for (i = 0; i < VRFB_NUM_SLOTS; i++) {
		if (ion_vrfb_t[i].ba == ovl_info->ba) {
			if (ion_vrfb_t[i].ismapped) {
				if (check_vrfb_params(
					&ion_vrfb_t[i].vrfb_context,
					ovl_info)) {
					got_vrfb_mapped_buf = 0;
					ion_vrfb_t[i].ismapped = 0;
				} else {
					ovl_info->ba = ion_vrfb_t[i].
						vrfb_context.
						paddr[ovl_info->cfg.rotation]
						+ omap_get_vrfb_offset(
						&ion_vrfb_t[i].vrfb_context,
						ovl_info->cfg.rotation);
				got_vrfb_mapped_buf = 1;
				}
			}
			break;
		}
	}

	if (!got_vrfb_mapped_buf) {
		omap_vrfb_setup(&ion_vrfb_t[i].vrfb_context,
				ovl_info->ba, ovl_info->cfg.crop.w,
				ovl_info->cfg.crop.h, get_bpp(mode),
				is_yuv_mode(mode));

		ovl_info->ba = ion_vrfb_t[i].
				vrfb_context.paddr[ovl_info->cfg.rotation]
				+ omap_get_vrfb_offset(
				&ion_vrfb_t[i].vrfb_context,
				ovl_info->cfg.rotation);
		ion_vrfb_t[i].ismapped = 1;
	}

	if (ovl_info->cfg.rotation & 1)	{
		ovl_info->cfg.win.w =
			(ovl_info->cfg.win.h * ovl_info->cfg.win.h) /
			ovl_info->cfg.win.w;
	}

	ovl_info->cfg.stride =  VRFB_LINE_LENGTH * 2;

	return 0;
}

#endif
