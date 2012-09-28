/*
 * drivers/media/video/omap/omapvout.h
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

#ifndef __OMAPVOUT_H__
#define __OMAPVOUT_H__

#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include <plat/display.h>
#include <plat/vrfb.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-core.h>

#ifdef DEBUG
#define DBG(format, ...) \
	printk(KERN_DEBUG "OMAPVOUT: " format, ## __VA_ARGS__)
#else
#define DBG(format, ...)
#endif

/* The device structure */

struct omapvout_device {
	struct video_device vdev;
	struct mutex  mtx; /* Lock for all device accesses */
	struct video_device *vfd;
	struct completion working_completion;

	int opened;
	int id;

	int disp_width;
	int disp_height;

	int max_video_width;
	int max_video_height;
	int max_video_buffer_size;

	/* Buffer pool */
	struct omapvout_bp *bp;

	/* DSS data */
	struct omapvout_dss *dss;

	/* V4L2 data */
	int rotation;
	int bg_color;
	struct v4l2_pix_format pix;
	struct v4l2_window win;
	struct v4l2_rect crop;
	struct v4l2_framebuffer fbuf;

	/* Frame Q */
	struct videobuf_queue queue;
	struct omapvout_vbq *vbq;
	struct list_head q_list;

	/* Don't allow new buffers when some are still mapped */
	int mmap_cnt;
};

#define vdev_to_omapvout(d) container_of(d, struct omapvout_device, vdev)

#endif /* __OMAPVOUT_H__ */
