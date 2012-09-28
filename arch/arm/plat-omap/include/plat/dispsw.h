/*
 * arch/arm/plat-omap/include/mach/dispsw.h
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/ioctl.h>
#include <plat/display.h>

#ifndef __DISPSW_H__
#define __DISPSW_H__

/*#define DEBUG*/
#ifdef DEBUG
#define DBG(format, ...) \
	printk(KERN_DEBUG "DISPSW: " format, ## __VA_ARGS__)
#else
#define DBG(format, ...)
#endif

#define DISPSW_MAX_NAME_SIZE	(32)

struct dispsw_mr_support {
	char dev_name[DISPSW_MAX_NAME_SIZE + 1];
	char res_name[DISPSW_MAX_NAME_SIZE + 1];
	struct omap_video_timings dev_timing;
	enum omap_panel_config panel_config;
};

/* Board specific data */
struct dispsw_board_info {
	int num_resolutions;
	struct dispsw_mr_support **resolutions;
};

#endif /* __DISPSW_H__ */

