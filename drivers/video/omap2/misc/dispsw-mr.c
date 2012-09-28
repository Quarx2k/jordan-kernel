/*
 * drivers/video/omap2/misc/dispsw-mr.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <plat/dispsw.h>

#include "dispsw-mr.h"

/*=== Local Functions ==================================================*/

/* None */

/*=== Public Functions =================================================*/

int dispsw_mr_init(struct dispsw_mr_data *mr)
{
	if (!mr)
		return -EINVAL;

	memset(mr, 0, sizeof(struct dispsw_mr_data));

	mutex_init(&mr->mtx);

	return 0;
}

void dispsw_mr_remove(struct dispsw_mr_data *mr)
{
	/* Nothing to do */
}

void dispsw_mr_set_board_info(struct dispsw_mr_data *mr,
				struct dispsw_board_info *info)
{
	int i;
	int cnt;

	if (!mr || !info)
		return;

	mutex_lock(&mr->mtx);

	cnt = info->num_resolutions;
	if (cnt > DISPSW_MR_MAX_RES_SUPPORTED)
		cnt = DISPSW_MR_MAX_RES_SUPPORTED;

	for (i = 0; i < cnt; i++) {
		mr->res[i] = info->resolutions[i];
		mr->num_entries++;
	}

	mutex_unlock(&mr->mtx);
}

bool dispsw_mr_is_multi_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev)
{
	int i;
	bool multi_res = false;

	mutex_lock(&mr->mtx);

	for (i = 0; i < mr->num_entries; i++) {
		if (mr->res[i] == NULL)
			continue;

		if (strcmp(dssdev->name, mr->res[i]->dev_name) == 0) {
			multi_res = true;
			break;
		}
	}

	mutex_unlock(&mr->mtx);

	return multi_res;
}

char *dispsw_mr_get_active_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev)
{
	int i;
	int cnt;
	char *name = NULL;
	struct omap_video_timings timings;

	mutex_lock(&mr->mtx);

	dssdev->get_timings(dssdev, &timings);

	cnt = 0;
	for (i = 0; i < mr->num_entries; i++) {
		if (mr->res[i] == NULL)
			continue;

		if (strcmp(dssdev->name, mr->res[i]->dev_name) != 0)
			continue;

		if (timings.x_res == mr->res[i]->dev_timing.x_res &&
		    timings.y_res == mr->res[i]->dev_timing.y_res &&
		    timings.hsw == mr->res[i]->dev_timing.hsw &&
		    timings.hfp == mr->res[i]->dev_timing.hfp &&
		    timings.hbp == mr->res[i]->dev_timing.hbp &&
		    timings.vsw == mr->res[i]->dev_timing.vsw &&
		    timings.vfp == mr->res[i]->dev_timing.vfp &&
		    timings.vbp == mr->res[i]->dev_timing.vbp) {
			name = mr->res[i]->res_name;
			break;
		}
		cnt++;
	}

	mutex_unlock(&mr->mtx);

	return name;
}

char *dispsw_mr_get_idx_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev, int idx)
{
	int i;
	int cnt;
	char *name = NULL;

	mutex_lock(&mr->mtx);

	cnt = 0;
	for (i = 0; i < mr->num_entries; i++) {
		if (mr->res[i] == NULL)
			continue;

		if (strcmp(dssdev->name, mr->res[i]->dev_name) != 0)
			continue;

		if (cnt == idx) {
			name = mr->res[i]->res_name;
			break;
		}
		cnt++;
	}

	mutex_unlock(&mr->mtx);

	return name;
}

int dispsw_mr_set_res(struct dispsw_mr_data *mr,
				struct omap_dss_device *dssdev, char *name)
{
	int rc = -EINVAL;
	int i;

	mutex_lock(&mr->mtx);

	for (i = 0; i < mr->num_entries; i++) {
		if (mr->res[i] == NULL)
			continue;

		if (strcmp(dssdev->name, mr->res[i]->dev_name) != 0)
			continue;

		if (strcmp(name, mr->res[i]->res_name) != 0)
			continue;

		dssdev->panel.config = mr->res[i]->panel_config;
		dssdev->set_timings(dssdev, &(mr->res[i]->dev_timing));
		rc = 0;
	}

	mutex_unlock(&mr->mtx);

	return rc;
}

