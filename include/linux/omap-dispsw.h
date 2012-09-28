/*
 * Copyright (C) 2009 Motorola Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/ioctl.h>

#ifndef __OMAP_DISPSW_H__
#define __OMAP_DISPSW_H__

#define DISPSW_MAX_NAME_SIZE	(32)
#define DISPSW_MAX_PLANES	(10)

struct dispsw_info {
	/* Requested display idx (zero based) */
	int idx;
	/* Returned display name */
	char name[DISPSW_MAX_NAME_SIZE + 1];
	/* Returned flag denoting if display is enabled */
	int active;
	/* Returned flag denoting if display supports multiple resolutions */
	int multi_resolutions;
	/* Returned active resolution name.
	 * 'empty' string if not active or if not supporting multiple
	 * resolutions
	 */
	char resolution_name[DISPSW_MAX_NAME_SIZE + 1];
};

struct dispsw_res_info {
	/* Requested display name for resolution info */
	char name[DISPSW_MAX_NAME_SIZE + 1];
	/* Requested resolution idx (zero based) */
	int idx;
	/* Returned resolution name.
	 * This name is highly display device specific and potentially
	 * platform specific
	 */
	char resolution_name[DISPSW_MAX_NAME_SIZE + 1];
};

enum dispsw_cmd_type {
	DISPSW_CMD_SET = 1,	/* S_CMD only */
	DISPSW_CMD_UNSET,	/* S_CMD only */
	DISPSW_CMD_SWITCH_TO,	/* S_CMD only */
	DISPSW_CMD_CONFIG,	/* G_CMD & S_CMD */
};

enum dispsw_rotate {
	DISPSW_ROTATE_IGNORE,
	DISPSW_ROTATE_0,
	DISPSW_ROTATE_90,
	DISPSW_ROTATE_180,
	DISPSW_ROTATE_270,
};

enum dispsw_scale {
	DISPSW_SCALE_IGNORE,
	DISPSW_SCALE_FIT_TO_SCREEN,
	DISPSW_SCALE_PERCENT, /* Percent of increase or decrease of plane */
};

enum dispsw_align {
	DISPSW_ALIGN_IGNORE,
	DISPSW_ALIGN_CENTER,
	DISPSW_ALIGN_TOP,
	DISPSW_ALIGN_TOP_LEFT,
	DISPSW_ALIGN_TOP_CENTER,
	DISPSW_ALIGN_TOP_RIGHT,
	DISPSW_ALIGN_BOTTOM,
	DISPSW_ALIGN_BOTTOM_LEFT,
	DISPSW_ALIGN_BOTTOM_CENTER,
	DISPSW_ALIGN_BOTTOM_RIGHT,
	DISPSW_ALIGN_LEFT,
	DISPSW_ALIGN_LEFT_CENTER,
	DISPSW_ALIGN_RIGHT,
	DISPSW_ALIGN_RIGHT_CENTER,
	DISPSW_ALIGN_PERCENT,  /* Percent of display size to align to.
				* Top-left of display to top-left of plane
				*/
};

/* Meant for hacking, but may be useful in some real cases */
struct dispsw_plane {
	int plane;	/* From 1 to # of planes available; 0 = ignore;
			 * - For omap3: 1 = gfx; 2 = vid1; 3 = vid2; */
	int override;	/* Override the input plane frames */
	int persist;	/* Persist after client closes, not suggested */

	int lock_aspect_ratio;
	enum dispsw_rotate rotate;
	enum dispsw_scale scale;
	int v_scale_percent;	/* From 800% to 25% */
	int h_scale_percent;	/* From 800% to 25% */
	enum dispsw_align align;
	int v_align_percent;	/* From 0% (left) to 100% (right) */
	int h_align_percent;	/* From 0% (top) to 100% (bottom) */
};

struct dispsw_cmd {
	enum dispsw_cmd_type type;
	char name[DISPSW_MAX_NAME_SIZE];

	/* Only used if the named display is active when the operation is
	 * complete, include SET, UNSET, SWITCH_TO, and CONFIG
	 */
	struct dispsw_plane planes[DISPSW_MAX_PLANES];
};

struct dispsw_res {
	char name[DISPSW_MAX_NAME_SIZE];
	char resolution_name[DISPSW_MAX_NAME_SIZE + 1];
};


#define DISPSW_IOCTL_MAGIC	'g'
#define DISPSW_IOCTL_BASE	0x20
#define DISPSW_QUERYDISP	_IOWR(DISPSW_IOCTL_MAGIC, \
				    DISPSW_IOCTL_BASE+0, struct dispsw_info)
#define DISPSW_QUERYRES		_IOWR(DISPSW_IOCTL_MAGIC, \
				    DISPSW_IOCTL_BASE+1, struct dispsw_res_info)
#define DISPSW_G_CMD		_IOWR(DISPSW_IOCTL_MAGIC, \
				    DISPSW_IOCTL_BASE+2, struct dispsw_cmd)
#define DISPSW_S_CMD		_IOW(DISPSW_IOCTL_MAGIC, \
				    DISPSW_IOCTL_BASE+3, struct dispsw_cmd)
#define DISPSW_S_RES		_IOW(DISPSW_IOCTL_MAGIC, \
				    DISPSW_IOCTL_BASE+4, struct dispsw_res)

#endif /* __OMAP_DISPSW_H__ */

