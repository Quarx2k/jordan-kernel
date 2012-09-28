/*
 * drivers/video/omap2/misc/dispsw-rotate.h
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * -----------------------------------------------------------------------
 *
 * This interface is intended to perform rotation on a single plane only,
 * typically the graphics plane.  It is assumed that the video planes
 * will already support rotation, thus this interface would not be needed
 * for them.
 */

#include <plat/display.h>
#include <plat/vrfb.h>
#include <plat/dispsw.h>

#ifndef __DISPSW_ROTATE_H__
#define __DISPSW_ROTATE_H__

struct dispsw_vrfb {
	bool need_cfg;

	/* Ping-pong buffers and VRFB contexts */
	struct vrfb ctx[2];
	unsigned long paddr[2];
	void *vaddr[2];
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
	bool dma_complete;
	wait_queue_head_t wait;
};

struct dispsw_rotate_data {
	struct mutex mtx; /* Lock for all device accesses */

	struct dispsw_vrfb vrfb;

	int max_w;
	int max_h;
	int max_buf_size;

	int buf_w;
	int buf_h;
	int buf_stride;
	int buf_rot;	/* 1 = 90; 2 = 180; 3 = 270; */
	enum omap_color_mode buf_fmt;
	int buf_offset;

	int dss_rot;	/* 1 = 90; 2 = 180; 3 = 270; */
};

/* Intended to be called at driver init and removal times */
int  dispsw_rotate_init(struct dispsw_rotate_data *rot, int max_w, int max_h,
							int max_buf_size);
void dispsw_rotate_remove(struct dispsw_rotate_data *rot);

int  dispsw_rotate_set_params(struct dispsw_rotate_data *rot, int width,
					int height, int stride, int rotate,
					enum omap_color_mode format);
unsigned long dispsw_rotate_perform_rotation(struct dispsw_rotate_data *rot,
							unsigned long paddr);

#endif /* __DISPSW_ROTATE_H__ */

