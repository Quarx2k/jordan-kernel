/*
 * drivers/media/video/omap/omapvout-dss.h
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

#ifndef __OMAPVOUT_DSS_H__
#define __OMAPVOUT_DSS_H__

struct omapvout_dss_vrfb {
	/* Ping-pong buffers and VRFB contexts */
	struct vrfb ctx[2];
	unsigned long phy_addr[2];
	void *virt_addr[2];
	int size;
	int next;

	/* VRFB dma config data */
	u32 en;
	u32 fn;
	u32 dst_ei;
	u32 dst_fi;

	/* VRFB dma channel data */
	int dma_id;
	int dma_ch;
	int req_status;
	bool dma_complete;
	wait_queue_head_t wait;
};

struct omapvout_dss {
	struct omap_overlay *overlay;

	/* FIXME: This is a minor hack to allow the work callback to
	 * gain access to the vout pointer.
	 */
	struct omapvout_device *vout;

	bool enabled;
	bool need_cfg;

	int rotation;

	struct omapvout_dss_vrfb vrfb;
	int foffset; /* per frame address offset */

	struct work_struct work;
	struct workqueue_struct *workqueue;
	bool working;

	int cur_q_idx; /* The current Q frame used by DSS */
};

/* Driver init/remove time calls */
extern int  omapvout_dss_init(struct omapvout_device *vout,
						enum omap_plane plane);
extern void omapvout_dss_remove(struct omapvout_device *vout);

/* Driver open/release time calls */
extern int  omapvout_dss_open(struct omapvout_device *vout,
						u16 *disp_w, u16 *disp_h);
extern void omapvout_dss_release(struct omapvout_device *vout);

/* Driver operation calls */
extern bool omapvout_dss_is_rotation_supported(struct omapvout_device *vout);
extern int  omapvout_dss_enable(struct omapvout_device *vout);
extern void omapvout_dss_disable(struct omapvout_device *vout);
extern int  omapvout_dss_update(struct omapvout_device *vout);

#endif /* __OMAPVOUT_DSS_H__ */

