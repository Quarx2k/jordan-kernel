/*
 * drivers/media/video/omap/omapvout-vbq.h
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
 *
 * This file contains the code necessary to interface with the
 * vbq (video buffer queue).
 */

#ifndef __OMAPVOUT_VBQ_H__
#define __OMAPVOUT_VBQ_H__

#include <linux/videodev2.h>

#include "omapvout.h"

struct omapvout_buf {
	u32 size;
	unsigned long phy_addr;
	void *virt_addr;
	bool released;
};

struct omapvout_vbq {
	spinlock_t lock;
	int cnt; /* number of frames in the queue */
	int min_size; /* smallest frame in the queue */
	struct omapvout_buf buf[VIDEO_MAX_FRAME];
};

extern int  omapvout_vbq_init(struct omapvout_device *vout);
extern void omapvout_vbq_destroy(struct omapvout_device *vout);
extern int  omapvout_vbq_buf_cnt(struct omapvout_device *vout);

#endif /* __OMAPVOUT_VBQ_H__ */

