/*
 * drivers/media/video/omap/omapvout-dss.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * Based on drivers/media/video/omap24xx/omap24xxvout.c&h
 *
 * Copyright (C) 2005-2006 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/mm.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <plat/display.h>
#include <plat/dma.h>
#include <plat/vrfb.h>

#include "omapvout.h"
#include "omapvout-dss.h"
#include "omapvout-mem.h"
#include "omapvout-vbq.h"

#define DMA_CHAN_ALLOTED	1
#define DMA_CHAN_NOT_ALLOTED	0

#define VRFB_TX_TIMEOUT		1000

/*=== Local Functions ==================================================*/

static int omapvout_dss_format_bytespp(u32 pixelformat)
{
	int bpp = 2;

	switch (pixelformat) {
	case V4L2_PIX_FMT_RGB32:
		bpp = 4;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		bpp = 2;
		break;
	}

	return bpp;
}

static enum omap_color_mode omapvout_dss_color_mode(u32 pixelformat)
{
	enum omap_color_mode mode = OMAP_DSS_COLOR_RGB16;

	switch (pixelformat) {
	case V4L2_PIX_FMT_RGB32:
		mode = OMAP_DSS_COLOR_RGB24U;
		break;
	case V4L2_PIX_FMT_RGB565:
		mode = OMAP_DSS_COLOR_RGB16;
		break;
	case V4L2_PIX_FMT_YUYV:
		mode = OMAP_DSS_COLOR_YUV2;
		break;
	case V4L2_PIX_FMT_UYVY:
		mode = OMAP_DSS_COLOR_UYVY;
		break;
	}

	return mode;
}

static void omapvout_dss_calc_offset(struct omapvout_device *vout,
				int bpp, int ow, int oh)
{
	struct omapvout_dss *dss;
	int iw, ih;
	int cx, cy, cw, ch;

	/* It is assumed that the caller has locked the vout mutex */

	dss = vout->dss;

	if (dss->rotation == 1 || dss->rotation == 3) {
		iw = vout->pix.height;
		ih = vout->pix.width;
		cw = vout->crop.height;
		ch = vout->crop.width;
	} else {
		iw = vout->pix.width;
		ih = vout->pix.height;
		cw = vout->crop.width;
		ch = vout->crop.height;
	}
	cx = vout->crop.left;
	cy = vout->crop.top;

	switch (dss->rotation)	{
	case 1: /* 90 degrees */
		dss->foffset = (cx * OMAP_VRFB_LINE_LEN * bpp)
				+ ((oh + (ih - cy - ch)) * bpp);
		break;
	case 2: /* 180 degrees */
		dss->foffset = ((oh + (ih - cy - ch)) * OMAP_VRFB_LINE_LEN
							* bpp)
				+ ((ow + (iw - cx - cw)) * bpp);
		break;
	case 3: /* 270 degrees */
		dss->foffset = ((ow + (iw - cx - cw)) * OMAP_VRFB_LINE_LEN
							* bpp)
				+ (cy * bpp);
		break;
	default:
	case 0: /* 0 degrees */
		dss->foffset = ((cy * iw) + (cx)) * bpp;
		break;
	}
}

static int omapvout_dss_get_overlays(struct omap_overlay **gfx,
			struct omap_overlay **vid1, struct omap_overlay **vid2)
{
	struct omap_overlay *t;
	int num_ovlys;
	int i;

	*gfx = NULL;
	*vid1 = NULL;
	*vid2 = NULL;
	num_ovlys = omap_dss_get_num_overlays();
	for (i = 0; i < num_ovlys; i++) {
		t = omap_dss_get_overlay(i);

		switch (t->id) {
		case OMAP_DSS_GFX:
			*gfx = t;
			break;
		case OMAP_DSS_VIDEO1:
			*vid1 = t;
			break;
		case OMAP_DSS_VIDEO2:
			*vid2 = t;
			break;
		}
	}

	if (*gfx && *vid1 && *vid2)
		return 0;

	return -EINVAL;
}

/* The algorithm below was cooked up to provide a reasonable global alpha
 * configuration based on the typical use case, which is a single video plane
 * blended with the graphics plane.  This algorithm will handle multiple
 * video planes as well, but the thought is that typically the user is not
 * going to want to be forced to set the global alpha value via the graphics
 * plane (the frame buffer), but instead via the video plane.
 *
 * Algorithm:
 * 1) For video1, if video2 is enabled, place the global alpha value set to
 *    video2, else set to the graphics plane.
 * 2) For video2, always place the global alpha value set to the graphics
 *   plane, but need to be careful to move the video1 alpha value if its
 *   already enabled.
 */
static void omapvout_dss_set_global_alpha(struct omapvout_device *vout)
{
	struct omap_overlay *gfx, *vid1, *vid2;
	struct omap_overlay_info info;
	u8 alpha;
	u8 t;

	/* Invert the requested alpha value since this alpha value is how
	 * transparent the video plane is supposed to be, but it is being
	 * applied to the plane above.
	 */
	alpha = 255 - vout->win.global_alpha;

	if (omapvout_dss_get_overlays(&gfx, &vid1, &vid2)) {
		printk(KERN_ERR "Not all planes available\n");
		return;
	}

	if (vout->dss->overlay->id == OMAP_DSS_VIDEO1) {
		vid2->get_overlay_info(vid2, &info);
		if (info.enabled) {
			info.global_alpha = alpha;
			vid2->set_overlay_info(vid2, &info);
		} else {
			gfx->get_overlay_info(gfx, &info);
			info.global_alpha = alpha;
			gfx->set_overlay_info(gfx, &info);
		}
	} else if (vout->dss->overlay->id == OMAP_DSS_VIDEO2) {
		vid1->get_overlay_info(vid1, &info);
		if (info.enabled) {
			gfx->get_overlay_info(gfx, &info);
			t = info.global_alpha;
			info.global_alpha = alpha;
			gfx->set_overlay_info(gfx, &info);
			vid2->get_overlay_info(vid2, &info);
			info.global_alpha = t;
			vid2->set_overlay_info(vid2, &info);
		} else {
			gfx->get_overlay_info(gfx, &info);
			info.global_alpha = alpha;
			gfx->set_overlay_info(gfx, &info);
			vid2->get_overlay_info(vid2, &info);
			info.global_alpha = 255;
			vid2->set_overlay_info(vid2, &info);
		}
	}

#ifdef DEBUG
	gfx->get_overlay_info(gfx, &info);
	DBG("GFX Alpha = %d\n", info.global_alpha);
	vid2->get_overlay_info(vid2, &info);
	DBG("VID2 Alpha = %d\n", info.global_alpha);
	vid1->get_overlay_info(vid1, &info);
	DBG("VID1 Alpha = %d\n", info.global_alpha);
#endif
}

/* This algorithm reverses the changes made in the "_set_" function and
 * returns a flag denoting if alpha should still be enabled.
 *
 * The non-trivial part is to handle the case where both video planes are
 * enabled.
 */
static bool omapvout_dss_clr_global_alpha(struct omapvout_device *vout)
{
	struct omap_overlay *gfx, *vid1, *vid2;
	struct omap_overlay_info info;
	bool alpha_en = false;
	u8 t;

	if (omapvout_dss_get_overlays(&gfx, &vid1, &vid2)) {
		printk(KERN_ERR "Not all planes available\n");
		return false;
	}

	if (vout->dss->overlay->id == OMAP_DSS_VIDEO1) {
		vid2->get_overlay_info(vid2, &info);
		if (info.enabled)
			alpha_en = true;
	} else if (vout->dss->overlay->id == OMAP_DSS_VIDEO2) {
		vid1->get_overlay_info(vid1, &info);
		if (info.enabled) {
			alpha_en = true;

			/* Set gfx alpha to vid1's alpha */
			vid2->get_overlay_info(vid2, &info);
			t = info.global_alpha;
			gfx->get_overlay_info(gfx, &info);
			info.global_alpha = t;
			gfx->set_overlay_info(gfx, &info);
		}
	}

	return alpha_en;
}

static int omapvout_dss_enable_transparency(struct omapvout_device *vout)
{
	struct omap_overlay_manager *mgr;
	struct omap_overlay_manager_info m_info;

	mgr = vout->dss->overlay->manager;
	if (mgr == NULL)
		return -EINVAL;

	if (mgr->set_manager_info == NULL || mgr->get_manager_info == NULL)
		return -EINVAL;

	mgr->get_manager_info(mgr, &m_info);

	m_info.default_color = vout->bg_color;

	if (vout->fbuf.flags & V4L2_FBUF_FLAG_CHROMAKEY) {
		m_info.trans_key_type = OMAP_DSS_COLOR_KEY_GFX_DST;
		m_info.trans_key = vout->win.chromakey;
		m_info.trans_enabled = true;
	} else if (vout->fbuf.flags & V4L2_FBUF_FLAG_SRC_CHROMAKEY) {
		m_info.trans_key_type = OMAP_DSS_COLOR_KEY_VID_SRC;
		m_info.trans_key = vout->win.chromakey;
		m_info.trans_enabled = true;
	} else {
		m_info.trans_enabled = false;
	}

	if (vout->fbuf.flags & V4L2_FBUF_FLAG_LOCAL_ALPHA) {
		m_info.alpha_enabled = true;
	} else if (vout->fbuf.flags & V4L2_FBUF_FLAG_GLOBAL_ALPHA) {
		omapvout_dss_set_global_alpha(vout);
		m_info.alpha_enabled = true;
	} else {
		m_info.alpha_enabled = false;
	}

	DBG("Trans Enable = %d\n", m_info.trans_enabled);
	DBG("Trans Mode = %d\n", m_info.trans_key_type);
	DBG("Alpha Enable = %d\n", m_info.alpha_enabled);

	mgr->set_manager_info(mgr, &m_info);

	return 0;
}

static int omapvout_dss_disable_transparency(struct omapvout_device *vout)
{
	struct omap_overlay_manager *mgr;
	struct omap_overlay_manager_info m_info;

	mgr = vout->dss->overlay->manager;
	if (mgr == NULL)
		return -EINVAL;

	if (mgr->set_manager_info == NULL || mgr->get_manager_info == NULL)
		return -EINVAL;

	mgr->get_manager_info(mgr, &m_info);

	m_info.alpha_enabled = omapvout_dss_clr_global_alpha(vout);
	m_info.trans_enabled = false;

	mgr->set_manager_info(mgr, &m_info);

	return 0;
}

/* This functions wakes up the application once the DMA transfer to
 * VRFB space is completed.
 */
static void omapvout_dss_vrfb_dma_cb(int lch, u16 ch_status, void *data)
{
	struct omapvout_dss_vrfb *vrfb;

	vrfb = (struct omapvout_dss_vrfb *) data;

	vrfb->dma_complete = true;
	wake_up_interruptible(&vrfb->wait);
}

static int omapvout_dss_acquire_vrfb(struct omapvout_device *vout)
{
	int rc = 0;
	int size;
	int w, h;
	int max_pixels;
	struct omapvout_dss_vrfb *vrfb;

	/* It is assumed that the caller has locked the vout mutex */

	vrfb = &vout->dss->vrfb;
	vrfb->dma_id = OMAP_DMA_NO_DEVICE;
	vrfb->dma_ch = -1;
	vrfb->req_status = DMA_CHAN_NOT_ALLOTED;
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

	/* Determine the VFRB buffer size by oversizing for the VRFB */
	w = vout->max_video_width;
	h = vout->max_video_height;
	max_pixels = w * h;
	w += 32; /* Oversize as typical for VRFB */
	h += 32;
	size = PAGE_ALIGN(w * h * (vout->max_video_buffer_size / max_pixels));
	vrfb->size = size;

	rc = omapvout_mem_alloc(size, &vrfb->phy_addr[0], &vrfb->virt_addr[0]);
	if (rc != 0) {
		printk(KERN_ERR "VRFB buffer alloc 0 failed %d\n", rc);
		goto failed_mem0;
	}

	rc = omapvout_mem_alloc(size, &vrfb->phy_addr[1], &vrfb->virt_addr[1]);
	if (rc != 0) {
		printk(KERN_ERR "VRFB buffer alloc 1 failed %d\n", rc);
		goto failed_mem1;
	}

	rc = omap_request_dma(vrfb->dma_id, "VRFB DMA",
				omapvout_dss_vrfb_dma_cb,
				(void *)vrfb,
				&vrfb->dma_ch);
	if (rc != 0) {
		printk(KERN_ERR "No VRFB DMA channel for %d\n", vout->id);
		goto failed_dma;
	}

	vrfb->req_status = DMA_CHAN_ALLOTED;
	init_waitqueue_head(&vrfb->wait);

	return rc;

failed_dma:
	omapvout_mem_free(vrfb->phy_addr[1], vrfb->virt_addr[1], size);
failed_mem1:
	omapvout_mem_free(vrfb->phy_addr[0], vrfb->virt_addr[0], size);
failed_mem0:
	omap_vrfb_release_ctx(&vrfb->ctx[1]);
failed_ctx1:
	omap_vrfb_release_ctx(&vrfb->ctx[0]);
failed_ctx0:
	return rc;
}

static int omapvout_dss_release_vrfb(struct omapvout_device *vout)
{
	int rc = 0;
	int size;
	struct omapvout_dss_vrfb *vrfb;

	/* It is assumed that the caller has locked the vout mutex */

	vrfb = &vout->dss->vrfb;
	if (vrfb->req_status == DMA_CHAN_ALLOTED) {
		vrfb->req_status = DMA_CHAN_NOT_ALLOTED;
		omap_free_dma(vrfb->dma_ch);
		/* FIXME: de-init the wait queue? */

		size = vrfb->size;
		omapvout_mem_free(vrfb->phy_addr[0], vrfb->virt_addr[0], size);
		omapvout_mem_free(vrfb->phy_addr[1], vrfb->virt_addr[1], size);

		omap_vrfb_release_ctx(&vrfb->ctx[0]);
		omap_vrfb_release_ctx(&vrfb->ctx[1]);
	}

	return rc;
}

static int omapvout_dss_perform_vrfb_dma(struct omapvout_device *vout,
					int buf_idx, bool vrfb_cfg)
{
	int rc = 0;
	int rot = vout->dss->rotation;
	struct omapvout_dss_vrfb *vrfb;
	u32 src_paddr;
	u32 dst_paddr;

	/* It is assumed that the caller has locked the vout mutex */

	if (vout->dss->vrfb.req_status != DMA_CHAN_ALLOTED)
		return -EINVAL;

	vrfb = &vout->dss->vrfb;

	if (vrfb_cfg) {
		enum omap_color_mode dss_fmt;
		int bytespp;
		int w, h;
		u32 fmt = vout->pix.pixelformat;
		bool is_yuv;

		w = vout->crop.width;
		h = vout->crop.height;

		is_yuv = (fmt == V4L2_PIX_FMT_YUYV || fmt == V4L2_PIX_FMT_UYVY);
		bytespp = omapvout_dss_format_bytespp(vout->pix.pixelformat);
		dss_fmt = omapvout_dss_color_mode(vout->pix.pixelformat);
		omap_vrfb_setup(&vrfb->ctx[0], vrfb->phy_addr[0],
				w, h, bytespp, is_yuv, rot);
		omap_vrfb_setup(&vrfb->ctx[1], vrfb->phy_addr[1],
				w, h, bytespp, is_yuv, rot);

		omapvout_dss_calc_offset(vout, vrfb->ctx[0].bytespp,
				vrfb->ctx[0].xoffset, vrfb->ctx[0].yoffset);

		vrfb->en = (w * bytespp) / 4; /* 32 bit ES */
		vrfb->fn = h;
		vrfb->dst_ei = 1;
		if (fmt == V4L2_PIX_FMT_YUYV || fmt == V4L2_PIX_FMT_UYVY) {
			vrfb->dst_fi = (OMAP_VRFB_LINE_LEN * bytespp * 2)
							- (vrfb->en * 4) + 1;
		} else {
			vrfb->dst_fi = (OMAP_VRFB_LINE_LEN * bytespp)
							- (vrfb->en * 4) + 1;
		}
	}

	src_paddr = vout->queue.bufs[buf_idx]->baddr;
	dst_paddr = vrfb->ctx[vrfb->next].paddr[rot] + vout->dss->foffset;

	omap_set_dma_transfer_params(vrfb->dma_ch, OMAP_DMA_DATA_TYPE_S32,
				vrfb->en, vrfb->fn, OMAP_DMA_SYNC_ELEMENT,
				vrfb->dma_id, 0x0);
	omap_set_dma_src_params(vrfb->dma_ch, 0, OMAP_DMA_AMODE_POST_INC,
				src_paddr, 0, 0);
	omap_set_dma_src_burst_mode(vrfb->dma_ch, 1);
	omap_set_dma_dest_params(vrfb->dma_ch, 0, OMAP_DMA_AMODE_DOUBLE_IDX,
				dst_paddr, vrfb->dst_ei, vrfb->dst_fi);
	omap_set_dma_dest_burst_mode(vrfb->dma_ch, 1);
	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0x20, 0);

	vrfb->dma_complete = false;
	omap_start_dma(vrfb->dma_ch);
	wait_event_interruptible_timeout(vrfb->wait, vrfb->dma_complete,
							VRFB_TX_TIMEOUT);

	if (!vrfb->dma_complete) {
		DBG("VRFB DMA timeout\n");
		omap_stop_dma(vrfb->dma_ch);
		return -EINVAL;
	}

	return rc;
}

static int omapvout_dss_update_overlay(struct omapvout_device *vout,
							int buf_idx)
{
	struct omap_overlay_info o_info;
	struct omap_overlay *ovly;
	struct omapvout_dss_vrfb *vrfb;
	int rc = 0;
	int rot = vout->dss->rotation;

	/* It is assumed that the caller has locked the vout mutex */


	/* Populate the overlay info struct and set it */
	ovly = vout->dss->overlay;
	ovly->get_overlay_info(ovly, &o_info);
	o_info.enabled = true;
	vrfb = &vout->dss->vrfb;
	o_info.paddr = vrfb->ctx[vrfb->next].paddr[0];
	o_info.vaddr = NULL;
	o_info.screen_width = OMAP_VRFB_LINE_LEN;
	vrfb->next = (vrfb->next) ? 0 : 1;

	if (rot == 1 || rot == 3) { /* 90 or 270 degree rotation */
		o_info.width = vout->crop.height;
		o_info.height = vout->crop.width;
	} else {
		o_info.width = vout->crop.width;
		o_info.height = vout->crop.height;
	}

	o_info.pos_x = vout->win.w.left & ~1;
	o_info.pos_y = vout->win.w.top & ~1;
	o_info.out_width = vout->win.w.width;
	o_info.out_height = vout->win.w.height;
	o_info.color_mode = omapvout_dss_color_mode(vout->pix.pixelformat);
	o_info.rotation_type = OMAP_DSS_ROT_VRFB;
	o_info.rotation = vout->rotation; // Rotation value, not buffer index
	o_info.mirror = false;

	rc = ovly->set_overlay_info(ovly, &o_info);
	if (rc) {
		DBG("Failed setting the overlay info %d\n", rc);
		return rc;
	}

	rc = ovly->manager->apply(ovly->manager);
	if (rc) {
		DBG("Failed apply to overlay manager %d\n", rc);
		return rc;
	}

	return rc;
}

static void omapvout_dss_mark_buf_done(struct omapvout_device *vout, int idx)
{
	if ((vout->queue.bufs[idx]->state == VIDEOBUF_QUEUED) ||
		(vout->queue.bufs[idx]->state == VIDEOBUF_ACTIVE))
		vout->queue.bufs[idx]->state = VIDEOBUF_DONE;
	wake_up_interruptible(&vout->queue.bufs[idx]->done);
}

static void omapvout_dss_perform_update(struct work_struct *work)
{
	struct omapvout_device *vout;
	struct omapvout_dss *dss;
	struct omap_dss_device *dev;
	struct videobuf_buffer *buf;
	int rc;
	int idx = 0;

	dss = container_of(work, struct omapvout_dss, work);
	vout = dss->vout;

	if (!dss->enabled)
		return;

	mutex_lock(&vout->mtx);

	dss->working = true;

	init_completion(&vout->working_completion);

	while (!list_empty(&vout->q_list)) {

		buf = list_entry(vout->q_list.next,
				struct videobuf_buffer, queue);
		list_del(&buf->queue);
		buf->state = VIDEOBUF_ACTIVE;
		idx = buf->i;

		/*DBG("Processing frame %d\n", idx);*/

		if (dss->need_cfg) {
			rc = omapvout_dss_enable_transparency(vout);
			if (rc != 0) {
				DBG("Alpha config failed %d\n", rc);
				goto failed_need_done;
			}

			switch (vout->rotation) {
			case 1:
				dss->rotation = 3;
				break;
			case 3:
				dss->rotation = 1;
				break;
			default:
				dss->rotation = vout->rotation;
				break;
			}
		}

		rc = omapvout_dss_perform_vrfb_dma(vout, idx, dss->need_cfg);
		if (rc != 0) {
			DBG("VRFB rotation failed %d\n", rc);
			goto failed_need_done;
		}

                omapvout_dss_mark_buf_done(vout, idx);

		rc = omapvout_dss_update_overlay(vout, idx);
		if (rc != 0) {
			DBG("DSS update failed %d\n", rc);
			goto failed;
		}

		dss->need_cfg = false;

		mutex_unlock(&vout->mtx);

		/* Wait until the new frame is being used.  There is no problem
		 * doing this here since we are in a worker thread.  The mutex
		 * is unlocked since the sync may take some time.
		 */
		dev = dss->overlay->manager->device;
		if (dev && dev->sync)
			dev->sync(dev);

		/* Since the mutex was unlocked, it is possible that the DSS
		 * may be disabled when we return, so check for this and exit
		 * if so.
		 */
		if (!dss->enabled) {
			/* Since the DSS is disabled, this isn't a problem */
			dss->working = false;
			complete(&vout->working_completion);

			return;
		}

		mutex_lock(&vout->mtx);

		if (dev->update) {
			rc = dev->update(dev, 0, 0,
				vout->disp_width, vout->disp_height);
			if (rc)
				DBG("Overlay update failed %d\n", rc);
		}

	}

	dss->working = false;
	complete(&vout->working_completion);
	mutex_unlock(&vout->mtx);

	return;

failed_need_done:
	/* Set the done flag on failures to be sure the buffer can be DQ'd */
	omapvout_dss_mark_buf_done(vout, idx);
failed:
	dss->working = false;
	complete(&vout->working_completion);
	mutex_unlock(&vout->mtx);
}

/*=== Public Functions =================================================*/

int  omapvout_dss_init(struct omapvout_device *vout, enum omap_plane plane)
{
	struct omap_overlay *ovly;
	int rc = 0;
	int i;
	int cnt;

	if (vout->dss) {
		rc = -EINVAL;
		goto failed;
	}

	vout->dss = kzalloc(sizeof(struct omapvout_dss), GFP_KERNEL);
	if (vout->dss == NULL) {
		rc = -ENOMEM;
		goto failed;
	}

	/* Retrieve the desired DSS overlay object */
	vout->dss->overlay = NULL;
	cnt = omap_dss_get_num_overlays();
	printk("plane %d, # of overlays: %d\n", plane, cnt);
	for (i = 0; i < cnt; i++) {
		ovly = omap_dss_get_overlay(i);
		if (ovly->id == plane) {
			vout->dss->overlay = ovly;
			break;
		}
	}

	if (vout->dss->overlay == NULL) {
		printk(KERN_ERR "No overlay %d found\n", plane);
		rc = -ENODEV;
		goto failed_mem;
	}

	rc = omapvout_dss_acquire_vrfb(vout);
	if (rc != 0) {
		printk(KERN_ERR "VRFB allocation failed\n");
		goto failed_mem;
	}

	vout->dss->vout = vout;

	return rc;

failed_mem:
	kfree(vout->dss);
failed:
	return rc;
}

void omapvout_dss_remove(struct omapvout_device *vout)
{
	if (vout->dss != NULL) {
		omapvout_dss_release_vrfb(vout);
		kfree(vout->dss);
	}
}

int omapvout_dss_open(struct omapvout_device *vout, u16 *disp_w, u16 *disp_h)
{
	struct omap_dss_device *dev;
	int rc = 0;

	/* It is assumed that the caller has locked the vout mutex */

	if (vout->dss->overlay->manager == NULL) {
		printk(KERN_ERR "No manager found\n");
		rc = -ENODEV;
		goto failed;
	}

	if (vout->dss->overlay->manager->device == NULL) {
		printk(KERN_ERR "No device found\n");
		rc = -ENODEV;
		goto failed;
	}

	dev = vout->dss->overlay->manager->device;

	/* TODO: Do we need to deal with rotation? */
	dev->get_resolution(dev, disp_w, disp_h);

	vout->dss->workqueue = create_singlethread_workqueue("OMAPVOUT-DSS");
	if (vout->dss->workqueue == NULL) {
		rc = -ENOMEM;
		goto failed;
	}

	INIT_WORK(&vout->dss->work, omapvout_dss_perform_update);

	vout->dss->enabled = false;

failed:
	return rc;
}

void omapvout_dss_release(struct omapvout_device *vout)
{
	/* It is assumed that the caller has locked the vout mutex */

	if (vout->dss->enabled)
		omapvout_dss_disable(vout);

	flush_workqueue(vout->dss->workqueue);
	destroy_workqueue(vout->dss->workqueue);
}

bool omapvout_dss_is_rotation_supported(struct omapvout_device *vout)
{
	return vout->dss->vrfb.req_status == DMA_CHAN_ALLOTED;
}

int omapvout_dss_enable(struct omapvout_device *vout)
{
	/* It is assumed that the caller has locked the vout mutex */

	/* Reset the current frame idx */
	vout->dss->cur_q_idx = -1;

	/* Force a reconfiguration */
	vout->dss->need_cfg = true;

	vout->dss->enabled = true;

	return 0;
}

void omapvout_dss_disable(struct omapvout_device *vout)
{
	int rc = 0;
	struct omap_overlay_info o_info;
	struct omap_overlay *ovly;
	struct omap_dss_device *dev;
	struct videobuf_buffer *buf, *tmp;

	/* It is assumed that the caller has locked the vout mutex */

	memset(&o_info, 0, sizeof(o_info));
	o_info.enabled = false;

	vout->dss->enabled = false;

	dev = vout->dss->overlay->manager->device;
	if (vout->dss->working) {
		mutex_unlock(&vout->mtx);
		rc = wait_for_completion_timeout(
			&vout->working_completion, msecs_to_jiffies(50));
		if (!rc)
			printk(KERN_ERR "Timeout waiting for working flag \
			to be set false %d\n", rc);
		mutex_lock(&vout->mtx);
	}

	list_for_each_entry_safe(buf, tmp, &vout->q_list, queue) {
		list_del(&buf->queue);
                omapvout_dss_mark_buf_done(vout, buf->i);
        }

	rc = omapvout_dss_disable_transparency(vout);
	if (rc)
		printk(KERN_ERR "Disabling transparency failed %d\n", rc);

	ovly = vout->dss->overlay;
	rc = ovly->set_overlay_info(ovly, &o_info);
	if (rc)
		printk(KERN_ERR "Setting overlay info failed %d\n", rc);

	rc = ovly->manager->apply(ovly->manager);
	if (rc)
		printk(KERN_ERR "Overlay manager apply failed %d\n", rc);

	if (ovly->manager->device->update) {
		rc = ovly->manager->device->update(ovly->manager->device,
				0, 0, vout->disp_width, vout->disp_height);
		if (rc)
			printk(KERN_ERR "Display update failed %d\n", rc);
	}
}

int omapvout_dss_update(struct omapvout_device *vout)
{
	/* It is assumed that the caller has locked the vout mutex */

	if (!vout->dss->enabled) {
		DBG("DSS overlay is not enabled\n");
		return -EINVAL;
	}

	if (vout->dss->working) {
		/* Exit quitely, since still working on previous frame */
		/*DBG("DSS busy, handle shortly\n");*/
		return 0;
	}

	if (queue_work(vout->dss->workqueue, &vout->dss->work) == 0) {
		DBG("Queuing DSS work failed\n");
		return -EINVAL;
	}

	return 0;
}

