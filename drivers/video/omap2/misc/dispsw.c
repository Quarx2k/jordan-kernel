/*
 * drivers/video/omap2/misc/dispsw.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pagemap.h>
#include <linux/platform_device.h>

#include <plat/display.h>
#include <plat/dispsw.h>
#include <plat/vout.h>

#include <linux/omap-dispsw.h>
#include "dispsw-mr.h"
#include "dispsw-rotate.h"

/*=========================================================================*/
/* These are platform dependent and should be provided in another manner.  */

/* By default the max is a 720P RGB16 frame */
#define MAX_ROTATION_WIDTH		(1280)
#define MAX_ROTATION_HEIGHT		(720)
#define MAX_ROTATION_BUFFER_SIZE	(MAX_ROTATION_WIDTH * \
					 MAX_ROTATION_HEIGHT * 2)
/*=========================================================================*/

#define MAX_MANAGERS  (10)	/* A little bit of future proofing */
#define MAX_OVERLAYS  (10)	/* A little bit of future proofing */

#define MAX_UPSCALE   (8)	/* 8X */
#define MAX_DOWNSCALE (4)	/* 1/4X */

typedef int (*ovl_set_info)(struct omap_overlay *ovl,
				struct omap_overlay_info *info);
typedef	void (*ovl_get_info)(struct omap_overlay *ovl,
				struct omap_overlay_info *info);

struct dispsw_osi {
	int id;
	int idx;

	ovl_set_info set_func;
	ovl_get_info get_func;
	struct omap_overlay *ovl;

	struct omap_overlay_info last_info;

	bool override;
	bool persist;

	struct omap_dss_device *dssdev;
	int disp_w;
	int disp_h;

	bool force_disabled;
	bool stored_enable;
	int  force_cnt;

	bool lock_aspect_ratio;
	enum dispsw_rotate rotate;
	enum dispsw_scale scale;
	int v_scale_percent;
	int h_scale_percent;
	enum dispsw_align align;
	int v_align_percent;
	int h_align_percent;
};

struct dispsw_device {
	struct mutex  mtx; /* Lock for all device accesses */

	int major;
	struct class *cls;
	struct device *dev;

	int opened;

	struct dispsw_rotate_data rot;
	struct dispsw_mr_data mr;

	int num_ovls;

	/*
	 * Performance hack to turn off GFX plane for playback
	 * played to an overridden device
	 */
	int videoOverrideEnabled;

	/* Data, per overlay, for overriding the overlay set info call */
	struct dispsw_osi osi[MAX_OVERLAYS];

	bool no_update;
};

#define DEVICE_NAME  "dispsw"

static struct dispsw_device *g_dev;

/* A couple of prototypes */
static void dispsw_override_ovl(struct dispsw_osi *osi,
				struct omap_overlay_info *info);

/*=== Local Functions ==================================================*/

/*
 *  This function contains all of the code related to a performance hack where,
 *  for HDMI, we believe the GFX plane is fully transparent most of the time,
 *  thus does not need to be part of the hardware composition.  If we turn off
 *  the GFX plane, we save a lot of L3 bus traffic since we don't need to read
 *  the entire GFX plane 60 times per second for display controller HW
 *  composition.
 */
static void dispsw_handle_gfx_disable(int ovl_id,
					struct omap_overlay_info *info)
{
	struct dispsw_osi *osi = NULL;
	struct omap_overlay_info gfx_info;
	int gfx = 0;
	int prev;
	int i;

	/* If the GFX plane changed, just chk if it should be enabled */
	if (ovl_id == OMAP_DSS_GFX) {
		info->enabled = (g_dev->videoOverrideEnabled) ? false : true;
	} else {
		prev = g_dev->videoOverrideEnabled;
		g_dev->videoOverrideEnabled = 0;
		for (i = 0; i < MAX_OVERLAYS; i++) {
			osi = &g_dev->osi[i];
			if (osi->ovl == NULL)
				continue;
			/* Save the GFX plane idx for below*/
			else if (osi->id == OMAP_DSS_GFX)
				gfx = i;
			/*
			 *  For non-GFX planes, if they are enabled,
			 *  overriden (meaning HDMI), disable the GFX plane.
			 */
			else if (osi->last_info.enabled == true	&&
								osi->override)
				g_dev->videoOverrideEnabled = 1;
		}
		if (g_dev->videoOverrideEnabled != prev) {
			osi = &g_dev->osi[gfx];
			memcpy(&gfx_info, &osi->last_info, sizeof(gfx_info));
			if (gfx_info.enabled && osi->override)
				dispsw_override_ovl(osi, &gfx_info);
			if (gfx_info.enabled && g_dev->videoOverrideEnabled)
				gfx_info.enabled = false;
			osi->set_func(osi->ovl, &gfx_info);
		}
	}
}

static int dispsw_convert_to_dss_rotation(enum dispsw_rotate rotate)
{
	int rot;

	switch (rotate) {
	case DISPSW_ROTATE_90:
		rot = 1;
		break;
	case DISPSW_ROTATE_180:
		rot = 2;
		break;
	case DISPSW_ROTATE_270:
		rot = 3;
		break;
	default:
		rot = 0;
		break;
	}

	return rot;
}

static int dispsw_convert_to_bytespp(enum omap_color_mode fmt)
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

static int dispsw_center_align(int side, int dside, unsigned long *pa, int inc)
{
	int offset = 0;

	if (side <= dside)
		offset = ((dside - side) / 2);
	else
		*pa += ((((side - dside) / 2) + 1) & ~1) * inc;

	return offset;
}

static int dispsw_far_side_align(int side, int dside, unsigned long *pa,
								    int inc)
{
	int offset = 0;

	if (side <= dside)
		offset = (dside - side);
	else
		*pa += (((side - dside) + 1) & ~1) * inc;

	return offset;
}

static bool dispsw_crop_side(int max, int offset, int *side)
{
	if (offset + *side < max)
		return false;
	if (offset > max)
		return true;
	*side = max - offset;
	return false;
}

static bool dispsw_crop(struct dispsw_osi *osi,
				struct omap_overlay_info *info)
{
	bool disable = false;
	int w, h;
	int ratio;

	w = info->out_width;
	h = info->out_height;
	disable = dispsw_crop_side(osi->disp_w, info->pos_x, &w);
	if (disable)
		goto exit;

	disable = dispsw_crop_side(osi->disp_h, info->pos_y, &h);
	if (disable)
		goto exit;

	/* Don't attempt adjusting the input frame if no change occurred */
	if (w == info->out_width && h == info->out_height)
		goto exit;

	if (osi->id == OMAP_DSS_GFX || (info->width == info->out_width &&
					info->height == info->out_height)) {
		info->width = w;
		info->height = h;
		info->out_width = w;
		info->out_height = h;
	} else {
		/* If scaling is involved, crop the input frame
		 *
		 * Assuming the width's and height's are small enough to not
		 * cause any issues when performing this fixed point math.
		 */
		ratio = (info->out_width << 8) * (w << 8);
		info->width = ((info->width << 8) * ratio) >> 8;
		ratio = (info->out_height << 8) * (h << 8);
		info->height = ((info->height << 8) * ratio) >> 8;

		info->out_width = w;
		info->out_height = h;
	}

exit:
	return disable;
}

static void dispsw_scale(struct dispsw_osi *osi,
				struct omap_overlay_info *info)
{
	int w, h;
	int tw, th;
	int ratio;
	const int shift = 10; /* Fixed point shift factor */

	w = info->width;
	h = info->height;

	switch (osi->scale) {
	case DISPSW_SCALE_FIT_TO_SCREEN:
		if (osi->lock_aspect_ratio) {
			tw = (osi->disp_w << shift) / w;
			th = (osi->disp_h << shift) / h;
			ratio = (tw < th) ? tw : th;
			w = (w * ratio) >> shift;
			h = (h * ratio) >> shift;
			w = (w + 1) & ~1;
			h = (h + 1) & ~1;
		} else {
			w = osi->disp_w;
			h = osi->disp_h;
		}
		break;
	case DISPSW_SCALE_PERCENT:
		w += (((w << shift) / 100) * osi->v_scale_percent) >> shift;
		h += (((h << shift) / 100) * osi->h_scale_percent) >> shift;
		break;
	case DISPSW_SCALE_IGNORE:
	default:
		break;
	}

	/* Enforce HW scaling limits */
	if (w > (info->width * MAX_UPSCALE))
		w = (info->width * MAX_UPSCALE);
	else if (w < (info->width / MAX_DOWNSCALE))
		w = (info->width / MAX_DOWNSCALE);

	if (h > (info->height * MAX_UPSCALE))
		h = (info->height * MAX_UPSCALE);
	else if (h < (info->height / MAX_DOWNSCALE))
		h = (info->height / MAX_DOWNSCALE);

	info->out_width = w;
	info->out_height = h;
}

static void dispsw_rotate(struct dispsw_osi *osi,
				struct omap_overlay_info *info)
{
	int rc = 0;
	int rotate;
	int t;
	unsigned long raddr;

	if (osi->rotate == DISPSW_ROTATE_IGNORE)
		return;

	rotate = dispsw_convert_to_dss_rotation(osi->rotate);
	if (rotate == info->rotation)
		return;

	/* Video plane rotation is handled else where */
	if (osi->id != OMAP_DSS_GFX)
		return;

	/* It is assumed that the GFX plane is not currently rotated */
	if (info->rotation != 0)
		return;

	rc = dispsw_rotate_set_params(&g_dev->rot, info->width, info->height,
			info->screen_width, rotate, info->color_mode);
	if (rc != 0)
		return;

	raddr = dispsw_rotate_perform_rotation(&g_dev->rot,
						(unsigned long) info->paddr);

	if (!raddr)
		return;

	info->paddr = raddr;
	info->vaddr = NULL;
	info->screen_width = OMAP_VRFB_LINE_LEN;
	info->rotation_type = OMAP_DSS_ROT_VRFB;
	info->rotation = rotate;
	if (rotate == 1 || rotate == 3) {
		t = info->width;
		info->width = info->height;
		info->height = t;
		t = info->out_width;
		info->out_width = info->out_height;
		info->out_height = t;
	}
}

static void dispsw_align(struct dispsw_osi *osi,
				struct omap_overlay_info *info)
{
	int x, y, w, h;
	int dw, dh;
	int bpp, stride;
	unsigned long paddr;

	x = info->pos_x;
	y = info->pos_y;
	w = info->out_width;
	h = info->out_height;
	dw = osi->disp_w;
	dh = osi->disp_h;

	bpp = dispsw_convert_to_bytespp(info->color_mode);
	stride = info->screen_width * bpp;
	paddr = info->paddr;

	/* Handle the case of VRFB and YUV */
	if (info->screen_width == 2048 &&
	    (info->color_mode == OMAP_DSS_COLOR_YUV2 ||
	     info->color_mode == OMAP_DSS_COLOR_UYVY))
		stride *= 2;

	switch (osi->align) {
	case DISPSW_ALIGN_IGNORE:
		break;
	case DISPSW_ALIGN_CENTER:
		x = dispsw_center_align(w, dw, &paddr, bpp);
		y = dispsw_center_align(h, dh, &paddr, stride);
		break;
	case DISPSW_ALIGN_TOP:
		y = 0;
		break;
	case DISPSW_ALIGN_TOP_LEFT:
		x = 0;
		y = 0;
		break;
	case DISPSW_ALIGN_TOP_CENTER:
		x = dispsw_center_align(w, dw, &paddr, bpp);
		y = 0;
		break;
	case DISPSW_ALIGN_TOP_RIGHT:
		x = dispsw_far_side_align(w, dw, &paddr, bpp);
		y = 0;
		break;
	case DISPSW_ALIGN_BOTTOM:
		y = dispsw_far_side_align(h, dh, &paddr, stride);
		break;
	case DISPSW_ALIGN_BOTTOM_LEFT:
		x = 0;
		y = dispsw_far_side_align(h, dh, &paddr, stride);
		break;
	case DISPSW_ALIGN_BOTTOM_CENTER:
		x = dispsw_center_align(w, dw, &paddr, bpp);
		y = dispsw_far_side_align(h, dh, &paddr, stride);
		break;
	case DISPSW_ALIGN_BOTTOM_RIGHT:
		x = dispsw_far_side_align(w, dw, &paddr, bpp);
		y = dispsw_far_side_align(h, dh, &paddr, stride);
		break;
	case DISPSW_ALIGN_LEFT:
		x = 0;
		break;
	case DISPSW_ALIGN_LEFT_CENTER:
		x = 0;
		y = dispsw_center_align(h, dh, &paddr, stride);
		break;
	case DISPSW_ALIGN_RIGHT:
		x = dispsw_far_side_align(w, dw, &paddr, bpp);
		break;
	case DISPSW_ALIGN_RIGHT_CENTER:
		x = dispsw_far_side_align(w, dw, &paddr, bpp);
		y = dispsw_center_align(h, dh, &paddr, stride);
		break;
	case DISPSW_ALIGN_PERCENT:
		x = (dw * osi->v_align_percent) / 100;
		y = (dh * osi->h_align_percent) / 100;
		break;
	}

	info->pos_x = ((x + 1) & ~1);
	info->pos_y = ((y + 1) & ~1);
	info->paddr = paddr;
}

/* This function will adjust the overlay info of any plane based on
 * the client provided, and stored, override information
 */
static void dispsw_override_ovl(struct dispsw_osi *osi,
				struct omap_overlay_info *info)
{
	bool disable = false;

	dispsw_rotate(osi, info);
	dispsw_scale(osi, info);
	dispsw_align(osi, info);

	/* As a last resort, crop any frame area beyond the display area */
	disable = dispsw_crop(osi, info);

	if (disable)
		info->enabled = false;
}

/* This function performs the substitute "set_overlay_info" operation
 */
static int dispsw_ovl_set_info(struct omap_overlay *ovl,
				struct omap_overlay_info *info)
{
	struct dispsw_osi *osi = NULL;
	int rc = 0;
	int i;

	/* It is assumed the caller has the mutex locked */

	for (i = 0; i < MAX_OVERLAYS; i++) {
		if (g_dev->osi[i].id == ovl->id) {
			osi = &g_dev->osi[i];
			break;
		}
	}

	if (!osi) {
		rc = -EINVAL;
	} else {
		if (!g_dev->no_update)
			memcpy(&osi->last_info, info, sizeof(*info));

		if (osi->force_disabled) {
			osi->stored_enable = info->enabled;
			info->enabled = false;
		} else if (osi->id == OMAP_DSS_GFX && osi->force_cnt > 0) {
			/* HACK: The FB (frame buffer), which feeds the GFX
			 * plane, has an assumption that the enable flag stays
			 * the same.  When display's are switched, a GFX plane
			 * under-run can happen which disables the plane and,
			 * do to the FB assumption, causes a continuation of
			 * the state, which is not good.  This hack attempts
			 * to work-around this issue, until a better fix can
			 * be found, likely in the FB code.
			 */
			osi->force_cnt--;
			info->enabled = osi->stored_enable;
		}

		if (info->enabled && osi->override)
			dispsw_override_ovl(osi, info);

		dispsw_handle_gfx_disable(osi->id, info);

		rc = osi->set_func(ovl, info);
	}

	return rc;
}

/* This is the substitute "set_overlay_info" function.
 * All calls to the "set_overlay_info" function of any overlay will
 * come here, so we overlay info can be adjusted as requested by
 * the client
 */
static int dispsw_ovl_set_info_lock(struct omap_overlay *ovl,
				struct omap_overlay_info *info)
{
	int rc;

	mutex_lock(&g_dev->mtx);

	rc = dispsw_ovl_set_info(ovl, info);

	mutex_unlock(&g_dev->mtx);

	return rc;
}

/* This is the substitute "get_overlay_info" function.
 * All calls to the "set_overlay_info" function of any overlay will
 * come here, so we overlay info can be adjusted as requested by
 * the client
 */
static void dispsw_ovl_get_info_lock(struct omap_overlay *ovl,
				struct omap_overlay_info *info)
{
	struct dispsw_osi *osi = NULL;
	int i;

	mutex_lock(&g_dev->mtx);

	for (i = 0; i < MAX_OVERLAYS; i++) {
		if (g_dev->osi[i].id == ovl->id) {
			osi = &g_dev->osi[i];
			break;
		}
	}

	if (osi)
		memcpy(info, &osi->last_info, sizeof(*info));

	mutex_unlock(&g_dev->mtx);
}

/* When switching back to the original display, the original overlay
 * info is restored
 */
static void dispsw_restore_overlay_info(struct dispsw_osi *osi)
{
	struct omap_overlay_info info;

	osi->get_func(osi->ovl, &info);
	memcpy(&info, &osi->last_info, sizeof(info));
	osi->set_func(osi->ovl, &info);

	/* Make sure the rotation gets reset for video planes */
	if (osi->id != OMAP_DSS_GFX)
		omapvout_force_rotation(osi->id, 0, 0);

	osi->override = false;
	osi->persist = false;
}

/* Update an overlay plane based on the the default plane data
 * and the stored override plane data
 */
static void dispsw_force_ovl_update(struct dispsw_osi *osi)
{
	struct omap_overlay_info info;
	int rot;

	memcpy(&info, &osi->last_info, sizeof(info));

	DBG("Override/in%d/%dx%d(%d)-> %dx%d @ %d,%d\n", osi->id,
		info.width, info.height, info.screen_width,
		info.out_width, info.out_height, info.pos_x, info.pos_y);

	if (osi->id != OMAP_DSS_GFX && osi->rotate != DISPSW_ROTATE_IGNORE) {
		rot = dispsw_convert_to_dss_rotation(osi->rotate);
		omapvout_force_rotation(osi->id, 1, rot);
	}

	if (info.enabled) {
		dispsw_override_ovl(osi, &info);
		osi->set_func(osi->ovl, &info);
	}

	DBG("Override/out/%dx%d(%d)-> %dx%d @ %d,%d\n",
		info.width, info.height, info.screen_width,
		info.out_width, info.out_height, info.pos_x, info.pos_y);
}

/* Given plane override data for a single plane, update the stored
 * override data for that plane
 */
static int dispsw_store_ovl_data(struct dispsw_plane *plane,
					struct omap_dss_device *dssdev)
{
	struct dispsw_osi *osi = NULL;
	int rc = 0;
	int id = plane->plane - 1;
	int i;
	u16 w, h;

	for (i = 0; i < g_dev->num_ovls; i++) {
		if (id == g_dev->osi[i].id) {
			osi = &g_dev->osi[i];
			break;
		}
	}

	if (!osi) {
		printk(KERN_ERR "invalid plane number\n");
		return -EINVAL;
	}

	if (plane->override == 0) {
		dispsw_restore_overlay_info(osi);
		return 0;
	}

	if (plane->rotate < DISPSW_ROTATE_IGNORE ||
			plane->rotate > DISPSW_ROTATE_270) {
		printk(KERN_ERR "invalid rotation\n");
		return -EINVAL;
	}

	if (plane->scale < DISPSW_SCALE_IGNORE ||
	    plane->scale > DISPSW_SCALE_PERCENT ||
	    (plane->scale == DISPSW_SCALE_PERCENT &&
	     (plane->v_scale_percent > 800 ||
	      plane->v_scale_percent < 25 ||
	      plane->h_scale_percent > 800 ||
	      plane->h_scale_percent < 25))) {
		printk(KERN_ERR "invalid scale\n");
		return -EINVAL;
	}

	if (plane->scale != DISPSW_SCALE_IGNORE && osi->id == OMAP_DSS_GFX) {
		printk(KERN_ERR "invalid scale (GFX)\n");
		return -EINVAL;
	}

	if (plane->align < DISPSW_ALIGN_IGNORE ||
	    plane->align > DISPSW_ALIGN_PERCENT ||
	    (plane->align == DISPSW_ALIGN_PERCENT &&
	     (plane->v_align_percent > 100 ||
	      plane->v_align_percent < 0 ||
	      plane->h_align_percent > 100 ||
	      plane->h_align_percent < 0))) {
		printk(KERN_ERR "invalid align\n");
		return -EINVAL;
	}

	osi->override = true;
	osi->persist = (plane->persist) ? true : false;
	osi->dssdev = dssdev;
	dssdev->get_resolution(dssdev, &w, &h);
	osi->disp_w = w;
	osi->disp_h = h;
	osi->lock_aspect_ratio = (plane->lock_aspect_ratio) ? true : false;
	osi->rotate = plane->rotate;
	osi->scale = plane->scale;
	osi->v_scale_percent = plane->v_scale_percent;
	osi->h_scale_percent = plane->h_scale_percent;
	osi->align = plane->align;
	osi->v_align_percent = plane->v_align_percent;
	osi->h_align_percent = plane->h_align_percent;

	dispsw_force_ovl_update(osi);

	return rc;
}

/* Given a list of plane override data, update the stored overrode plane
 * data and perform the requested overrides
 */
static int dispsw_process_planes(struct dispsw_cmd *cmd,
					struct omap_dss_device *dssdev)
{
	int rc = 0;
	int i;

	/* Update the plane data */
	for (i = 0; i < g_dev->num_ovls; i++) {
		if (!cmd->planes[i].plane)
			continue;

		rc = dispsw_store_ovl_data(&cmd->planes[i], dssdev);
		if (rc != 0)
			break;
	}

	return rc;
}

static void dispsw_get_planes(struct dispsw_cmd *cmd)
{
	int i;

	/* Get the plane data */
	memset(cmd->planes, 0, sizeof(cmd->planes));
	for (i = 0; i < g_dev->num_ovls; i++) {
		cmd->planes[i].plane 		= g_dev->osi[i].id + 1;
		cmd->planes[i].override 	= g_dev->osi[i].override;
		cmd->planes[i].persist 		= g_dev->osi[i].persist;
		cmd->planes[i].lock_aspect_ratio =
						g_dev->osi[i].lock_aspect_ratio;
		cmd->planes[i].rotate 		= g_dev->osi[i].rotate;
		cmd->planes[i].scale 		= g_dev->osi[i].scale;
		cmd->planes[i].v_scale_percent 	= g_dev->osi[i].v_scale_percent;
		cmd->planes[i].h_scale_percent 	= g_dev->osi[i].h_scale_percent;
		cmd->planes[i].align 		= g_dev->osi[i].align;
		cmd->planes[i].v_align_percent 	= g_dev->osi[i].v_align_percent;
		cmd->planes[i].h_align_percent 	= g_dev->osi[i].h_align_percent;
	}
}

static void dispsw_unset_display(struct omap_dss_device *dssdev,
				struct omap_overlay_manager *dssmgr)
{
	struct omap_overlay_info info;
	struct omap_overlay *ovl;
	int i;
	int rc;

	DBG("Disconnecting %s from %s\n", dssdev->name, dssmgr->name);

	rc = dssdev->wait_vsync(dssdev);
	if (rc != 0)
		DBG("Wait VSync Failed - ignoring\n");

	/* Ignore the errors as we are unsetting */
	dssdev->disable(dssdev);

	for (i = 0; i < g_dev->num_ovls; i++) {
		ovl = g_dev->osi[i].ovl;
		if (ovl->manager != dssmgr)
			continue;

		g_dev->osi[i].get_func(ovl, &info);
		g_dev->osi[i].force_disabled = true;
		g_dev->osi[i].stored_enable = info.enabled;
		info.enabled = false;
		g_dev->osi[i].set_func(ovl, &info);
	}
	rc = dssmgr->apply(dssmgr);
	if (rc != 0)
		DBG("Unset Apply Failed - ignoring\n");

	/* Remove any plane overrides related this display */
	for (i = 0; i < g_dev->num_ovls; i++) {
		if (g_dev->osi[i].dssdev == dssdev)
			dispsw_restore_overlay_info(&g_dev->osi[i]);
	}

	rc = dssmgr->unset_device(dssmgr);
	if (rc != 0)
		DBG("Unset Failed - ignoring\n");
}

static int dispsw_set_display(struct dispsw_cmd *cmd,
			      struct omap_dss_device *dssdev,
			      struct omap_overlay_manager *dssmgr)
{
	struct omap_overlay_info info;
	struct omap_overlay *ovl;
	int rc;
	int i;

	DBG("Connecting %s to %s\n", dssdev->name, dssmgr->name);

	rc = dispsw_process_planes(cmd, dssdev);
	if (rc != 0)
		goto exit;

	if (dssdev->manager && dssdev->manager != dssmgr)
		dispsw_unset_display(dssdev, dssdev->manager);

	if (!dssdev->manager) {
		rc = dssmgr->set_device(dssmgr, dssdev);
		if (rc != 0)
			goto exit;
	}

	for (i = 0; i < g_dev->num_ovls; i++) {
		ovl = g_dev->osi[i].ovl;
		if (ovl->manager != dssmgr)
			continue;

		g_dev->osi[i].get_func(ovl, &info);
		g_dev->osi[i].force_disabled = false;
		info.enabled = g_dev->osi[i].stored_enable;
		g_dev->osi[i].force_cnt = 5;
		g_dev->no_update = true;
		dispsw_ovl_set_info(ovl, &info);
		g_dev->no_update = false;
	}

	rc = dssmgr->apply(dssmgr);
	if (rc != 0)
		goto exit;

	rc = dssdev->enable(dssdev);

exit:
	return rc;
}

static int dispsw_switch_to_display(struct dispsw_cmd *cmd,
				    struct omap_dss_device *dssdev,
				    struct omap_overlay_manager *dssmgr)
{
	struct omap_dss_device *olddev = NULL;
	int rc;

	if (dssmgr->device) {
		olddev = dssmgr->device;
		dispsw_unset_display(dssmgr->device, dssmgr);
	}

	rc = dispsw_set_display(cmd, dssdev, dssmgr);
	if (rc != 0 && olddev) {
		if (olddev->manager && olddev->manager != dssmgr)
			dssmgr->unset_device(dssmgr);

		if (!olddev->manager) {
			dssmgr->set_device(dssmgr, olddev);
			dssmgr->apply(dssmgr);
		}
	}

	return rc;
}

static struct omap_dss_device *dispsw_get_dssdev(char *name)
{
	struct omap_dss_device *dssdev = NULL;

	int match(struct omap_dss_device *dssdev, void *data)
	{
		const char *str = data;
		return sysfs_streq(dssdev->name, str);
	}

	if (!name || strlen(name) == 0)
		return NULL;

	dssdev = omap_dss_find_device((void *)name, match);

	return dssdev;
}

static struct omap_overlay_manager *dispsw_get_dssmgr(
					struct omap_dss_device *dssdev)
{
	struct omap_overlay_manager *dssmgr = NULL;
	int i;

	/* There is an assumption here that each device can only have
	 * one manager.
	 */
	i = omap_dss_get_num_overlay_managers() - 1;
	for (; i >= 0; i--) {
		dssmgr = omap_dss_get_overlay_manager(i);
		if (dssmgr && (dssmgr->supported_displays & dssdev->type))
			break;
	}

	if (i < 0)
		dssmgr = NULL;

	return dssmgr;
}

static int dispsw_g_cmd(struct dispsw_cmd *disp_cmd)
{
	struct dispsw_cmd cmd;
	int rc = 0;

	rc = copy_from_user(&cmd, disp_cmd, sizeof(cmd));
	if (rc != 0) {
		printk(KERN_ERR "G_CMD copy from user failed\n");
		return rc;
	}

	mutex_lock(&g_dev->mtx);

	switch (cmd.type) {
	case DISPSW_CMD_CONFIG:
		dispsw_get_planes(&cmd);
		rc = copy_to_user(disp_cmd, &cmd, sizeof(cmd));
		break;
	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&g_dev->mtx);

	return rc;
}

static int dispsw_s_cmd(struct dispsw_cmd *disp_cmd)
{
	struct dispsw_cmd cmd;
	struct omap_dss_device *dssdev;
	struct omap_overlay_manager *dssmgr;
	int rc = 0;

	rc = copy_from_user(&cmd, disp_cmd, sizeof(cmd));
	if (rc != 0) {
		printk(KERN_ERR "S_CMD copy from user failed\n");
		return rc;
	}

	dssdev = dispsw_get_dssdev(cmd.name);
	if (!dssdev) {
		printk(KERN_ERR "invalid device name\n");
		return -EINVAL;
	}

	dssmgr = dispsw_get_dssmgr(dssdev);
	if (!dssmgr) {
		printk(KERN_ERR "no manager for device\n");
		return -EINVAL;
	}

	mutex_lock(&g_dev->mtx);

	switch (cmd.type) {
	case DISPSW_CMD_SET:
		rc = dispsw_set_display(&cmd, dssdev, dssmgr);
		break;
	case DISPSW_CMD_UNSET:
		dispsw_unset_display(dssdev, dssmgr);
		break;
	case DISPSW_CMD_SWITCH_TO:
		rc = dispsw_switch_to_display(&cmd, dssdev, dssmgr);
		break;
	case DISPSW_CMD_CONFIG:
		rc = dispsw_process_planes(&cmd, dssdev);
		if (rc == 0)
			rc = dssmgr->apply(dssmgr);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	if (rc == 0 && dssdev->update) {
		u16 w, h;
		dssdev->get_resolution(dssdev, &w, &h);
		dssdev->update(dssdev, 0, 0, w, h);
	}

	omap_dss_put_device(dssdev);

	mutex_unlock(&g_dev->mtx);

	return rc;
}

static int dispsw_s_res(struct dispsw_res *disp_res)
{
	struct dispsw_res res;
	struct omap_dss_device *dssdev;
	int rc = 0;

	rc = copy_from_user(&res, disp_res, sizeof(res));
	if (rc != 0) {
		printk(KERN_ERR "S_RES copy from user failed\n");
		return rc;
	}

	dssdev = dispsw_get_dssdev(res.name);
	if (!dssdev) {
		printk(KERN_ERR "invalid device name\n");
		return -EINVAL;
	}

	mutex_lock(&g_dev->mtx);

	rc = dispsw_mr_set_res(&g_dev->mr, dssdev, res.resolution_name);

	omap_dss_put_device(dssdev);

	mutex_unlock(&g_dev->mtx);

    return rc;
}

static int dispsw_queryres(struct dispsw_res_info *res_info)
{
	struct dispsw_res_info info;
	struct omap_dss_device *dssdev = NULL;
	int rc = 0;
	char *res;

	rc = copy_from_user(&info, res_info, sizeof(info));
	if (rc != 0) {
		printk(KERN_ERR "QUERYRES copy from user failed\n");
		goto failed;
	}

	dssdev = dispsw_get_dssdev(info.name);
	if (!dssdev) {
		printk(KERN_ERR "invalid device name\n");
		rc = -EINVAL;
		goto failed;
	}

	res = dispsw_mr_get_idx_res(&g_dev->mr, dssdev, info.idx);
	if (res) {
		strncpy(info.resolution_name, res, DISPSW_MAX_NAME_SIZE);
		rc = copy_to_user(res_info, &info, sizeof(info));
	} else {
		rc = -EINVAL;
	}

failed:
	return rc;
}

static int dispsw_querydisp(struct dispsw_info *disp_info)
{
	struct dispsw_info info;
	struct omap_dss_device *dssdev = NULL;
	int rc = 0;
	int i = 0;
	int found = 0;

	rc = copy_from_user(&info, disp_info, sizeof(info));
	if (rc != 0) {
		printk(KERN_ERR "QUERYDISP copy from user failed\n");
		goto failed;
	}

	for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);

		if (i != info.idx) {
			i++;
			continue;
		}

		found = 1;
		memset(info.name, 0, DISPSW_MAX_NAME_SIZE + 1);
		strncpy(info.name, dssdev->name, DISPSW_MAX_NAME_SIZE);
		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
			info.active = 1;
		else
			info.active = 0;

		info.multi_resolutions = dispsw_mr_is_multi_res(&g_dev->mr,
								dssdev);
		if (info.multi_resolutions && info.active) {
			char *res;

			res = dispsw_mr_get_active_res(&g_dev->mr, dssdev);
			if (res != NULL)
				strncpy(info.resolution_name, res,
							DISPSW_MAX_NAME_SIZE);
		}

		break;
	}

	if (found)
		rc = copy_to_user(disp_info, &info, sizeof(info));
	else
		rc = -EINVAL;

failed:
	return rc;
}

/*=== Driver Interface Functions =======================================*/

static int dispsw_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	DBG("dispsw_open\n");

	if (g_dev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&g_dev->mtx);

	/* We only support single open */
	if (g_dev->opened) {
		DBG("Device already opened\n");
		rc = -EBUSY;
		goto failed;
	}

	g_dev->opened = 1;

	g_dev->videoOverrideEnabled = 0;

failed:
	mutex_unlock(&g_dev->mtx);
	return rc;
}

static int dispsw_release(struct inode *inode, struct file *file)
{
	int rc = 0;
	int i;

	DBG("dispsw_release\n");

	if (g_dev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&g_dev->mtx);

	g_dev->opened = 0;

	/* Make sure to disable all non-persistant overlay overrides. */
	for (i = 0; i < g_dev->num_ovls; i++)
		if (!g_dev->osi[i].persist)
			dispsw_restore_overlay_info(&g_dev->osi[i]);

	mutex_unlock(&g_dev->mtx);

	return rc;
}

static int dispsw_ioctl(struct inode *inode, struct file *file,
							u_int cmd, u_long arg)
{
	int rc = 0;

	if (unlikely(_IOC_TYPE(cmd) != DISPSW_IOCTL_MAGIC)) {
		printk(KERN_ERR "Bad command value (%d)\n", cmd);
		return -EINVAL;
	}

	switch (cmd) {
	case DISPSW_QUERYDISP:
		rc = dispsw_querydisp((struct dispsw_info *) arg);
		break;
	case DISPSW_QUERYRES:
		rc = dispsw_queryres((struct dispsw_res_info *) arg);
		break;
	case DISPSW_G_CMD:
		rc = dispsw_g_cmd((struct dispsw_cmd *) arg);
		break;
	case DISPSW_S_CMD:
		rc = dispsw_s_cmd((struct dispsw_cmd *) arg);
		break;
	case DISPSW_S_RES:
		rc = dispsw_s_res((struct dispsw_res *) arg);
		break;
	default:
		printk(KERN_ERR "Invalid ioctl (%x)\n", cmd);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static const struct file_operations dispsw_fops = {
	.owner = THIS_MODULE,
	.open = dispsw_open,
	.release = dispsw_release,
	.ioctl = dispsw_ioctl,
};

static int __init dispsw_probe(struct platform_device *pdev)
{
	struct omap_overlay *ovl;
	struct dispsw_board_info *info;
	int rc = 0;
	int num_ovls;
	int i;

	DBG("dispsw_probe\n");

	g_dev = kzalloc(sizeof(struct dispsw_device), GFP_KERNEL);
	if (g_dev == NULL)
		return -ENOMEM;

	memset(g_dev, 0, sizeof(g_dev));

	mutex_init(&g_dev->mtx);

	g_dev->opened = 0;

	g_dev->major = register_chrdev(0, DEVICE_NAME, &dispsw_fops);
	if (g_dev->major < 0) {
		printk(KERN_ERR "failed chrdev register\n");
		rc = -ENODEV;
		goto failed_chrdev;
	}

	g_dev->cls = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(g_dev->cls)) {
		printk(KERN_ERR "failed class creation\n");
		rc = PTR_ERR(g_dev->cls);
		goto failed_class;
	}

	rc = dispsw_mr_init(&g_dev->mr);
	if (rc != 0) {
		printk(KERN_ERR "failed multi-resolution init\n");
		goto failed_mr;
	}

	if (pdev->dev.platform_data) {
		info = (struct dispsw_board_info *) pdev->dev.platform_data;
		dispsw_mr_set_board_info(&g_dev->mr, info);
	}

	rc = dispsw_rotate_init(&g_dev->rot, MAX_ROTATION_WIDTH,
				MAX_ROTATION_HEIGHT, MAX_ROTATION_BUFFER_SIZE);
	if (rc != 0) {
		printk(KERN_ERR "failed rotate init\n");
		goto failed_rotate;
	}

	/* Hook into the set_overlay_info call path */
	num_ovls = omap_dss_get_num_overlays();
	if (!num_ovls) {
		printk(KERN_ERR "no dss overlays\n");
		rc = -ENODEV;
		goto failed;
	} else if (num_ovls > MAX_OVERLAYS) {
		printk(KERN_ERR "too many dss overlays\n");
		rc = -ENODEV;
		goto failed;
	}

	for (i = 0; i < num_ovls; i++) {
		ovl = omap_dss_get_overlay(i);
		g_dev->osi[i].id = ovl->id;
		g_dev->osi[i].idx = i;
		g_dev->osi[i].set_func = ovl->set_overlay_info;
		g_dev->osi[i].get_func = ovl->get_overlay_info;
		g_dev->osi[i].ovl = ovl;
		ovl->set_overlay_info = dispsw_ovl_set_info_lock;
		ovl->get_overlay_info = dispsw_ovl_get_info_lock;
		g_dev->osi[i].get_func(ovl, &g_dev->osi[i].last_info);

		if (g_dev->osi[i].set_func == NULL ||
				g_dev->osi[i].get_func == NULL) {
			printk(KERN_ERR "No set/get overlay info functions\n");
			rc = -ENODEV;
			goto failed;
		}
	}
	g_dev->num_ovls = num_ovls;

	g_dev->dev = device_create(g_dev->cls, g_dev->dev,
				MKDEV(g_dev->major, 0), NULL, DEVICE_NAME);

	return 0;

failed:
	dispsw_rotate_remove(&g_dev->rot);
failed_rotate:
	dispsw_mr_remove(&g_dev->mr);
failed_mr:
	class_destroy(g_dev->cls);
failed_class:
	unregister_chrdev(g_dev->major, DEVICE_NAME);
failed_chrdev:
	kfree(g_dev);
	g_dev = NULL;
	return rc;
}

static int dispsw_remove(struct platform_device *pdev)
{
	struct dispsw_device *dsw = platform_get_drvdata(pdev);

	DBG("dispsw_remove\n");

	if (dsw) {
		dispsw_rotate_remove(&dsw->rot);
		dispsw_mr_remove(&dsw->mr);
		class_destroy(dsw->cls);
		unregister_chrdev(dsw->major, DEVICE_NAME);
		kfree(dsw);
	}

	return 0;
}

static struct platform_driver dispsw_driver = {
	.remove		= dispsw_remove,
	.driver		= {
		.name   = DEVICE_NAME,
	},
};

static int __init dispsw_init(void)
{
	int rc;

	DBG("dispsw_init\n");

	rc = platform_driver_probe(&dispsw_driver, dispsw_probe);
	if (rc != 0) {
		printk(KERN_ERR "failed dispsw register/probe %d\n", rc);
		return -ENODEV;
	}

	return 0;
}

static void __exit dispsw_exit(void)
{
	DBG("dispsw_exit\n");

	platform_driver_unregister(&dispsw_driver);
}

device_initcall_sync(dispsw_init);
module_exit(dispsw_exit);

MODULE_DESCRIPTION("DSS2 Display Switcher");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

