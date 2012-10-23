/*
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef _LINUX_PM_DBG_H
#define _LINUX_PM_DBG_H

#include <linux/ioctl.h>

struct pm_dbg_currmeas_req {
	unsigned long time_start;
	unsigned long time_stop;
	int curr_drain;
};

struct pm_dbg_currmeas_suspend_sts_req {
	unsigned long time_suspend;
	unsigned long time_resume;
	int avg_curr_drain;
	unsigned char retention_failure;
	unsigned long time_retention;
	int min_curr_drain;
};

struct pm_dbg_drvdata {
	unsigned short pm_cd_factor;
};

#define PM_DBG_IOCTL_CURRMEAS_START _IOWR(0, 0, \
		struct pm_dbg_currmeas_req*)
#define PM_DBG_IOCTL_CURRMEAS_STOP _IOWR(0, 1, \
		struct pm_dbg_currmeas_req*)
#define PM_DBG_IOCTL_CURRMEAS_SUSPEND_STS _IOWR(0, 2, \
				struct pm_dbg_currmeas_suspend_sts_req*)
#define PM_DBG_IOCTL_CURRMEAS_CMD _IOWR(0, 3, unsigned char *)

#endif	/* _LINUX_PM_DBG_H */
