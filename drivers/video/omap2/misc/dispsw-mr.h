/*
 * drivers/video/omap2/misc/dispsw-mr.h
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * -----------------------------------------------------------------------
 *
 * This interface is intended to handle all aspects of multiple resolution
 * support for the platform displays.
 */

#include <plat/dispsw.h>

#ifndef __DISPSW_MR_H__
#define __DISPSW_MR_H__

#define DISPSW_MR_MAX_RES_SUPPORTED	(20)

struct dispsw_mr_data {
	struct mutex mtx; /* Lock for all device accesses */

	int num_entries;
	struct dispsw_mr_support *res[DISPSW_MR_MAX_RES_SUPPORTED];
};

/* Intended to be called at driver init and removal times */
int  dispsw_mr_init(struct dispsw_mr_data *mr);
void dispsw_mr_remove(struct dispsw_mr_data *mr);

void dispsw_mr_set_board_info(struct dispsw_mr_data *mr,
				struct dispsw_board_info *info);

bool  dispsw_mr_is_multi_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev);
char *dispsw_mr_get_active_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev);
char *dispsw_mr_get_idx_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev, int idx);
int   dispsw_mr_set_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev, char *name);

#endif /* __DISPSW_MR_H__ */

