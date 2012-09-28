/*
 * Copyright (C) 2009 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __OMAP_PANEL_H__
#define __OMAP_PANEL_H__

#define OMAP_PANEL_MAX_NAME_SIZE	(32)

struct omap_panel_info {
	/* Requested panel idx (zero based) */
	int idx;
	/* Returned panel name */
	char name[OMAP_PANEL_MAX_NAME_SIZE + 1];
	/* Returned active flag */
	int active;
};

struct omap_panel_fod {
	/* Requested panel name */
	char name[OMAP_PANEL_MAX_NAME_SIZE + 1];
	/* Returned (G_FOD) or requested (S_FOD) state */
	int  enable;
};

#define OMAP_PANEL_IOCTL_MAGIC	'g'
#define OMAP_PANEL_IOCTL_BASE	0x60

#define OMAP_PANEL_QUERYPANEL	_IOWR(OMAP_PANEL_IOCTL_MAGIC, \
				    OMAP_PANEL_IOCTL_BASE+0, \
				    struct omap_panel_info)
/* Freeze On Disable */
#define OMAP_PANEL_G_FOD  	_IOWR(OMAP_PANEL_IOCTL_MAGIC, \
				    OMAP_PANEL_IOCTL_BASE+1, \
				    struct omap_panel_fod)
#define OMAP_PANEL_S_FOD	_IOW(OMAP_PANEL_IOCTL_MAGIC, \
				    OMAP_PANEL_IOCTL_BASE+2, \
				    struct omap_panel_fod)

#endif /* __OMAP_PANEL_H__ */

