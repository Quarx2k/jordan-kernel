/*
 * Copyright (C) 2012 Texas Instruments, Inc
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/thermal_framework.h>

static int cool_device(struct thermal_dev *dev, int cooling_level);

static struct cool_data {
	int freq_cnt;
	unsigned long *freq_list;
} cd;

static struct thermal_dev_ops cool_dev_ops = {
	.cool_device = cool_device,
};

static struct thermal_dev cool_dev = {
	.name = "gpu_cooling.0",
	.domain_name = "gpu",
	.dev_ops = &cool_dev_ops,
};

static struct thermal_dev case_cool_dev = {
	.name = "gpu_cooling.1",
	.domain_name = "case",
	.dev_ops = &cool_dev_ops,
};

static unsigned int gpu_cooling_level;
#if defined(CONFIG_CASE_TEMP_GOVERNOR)
static unsigned int case_cooling_level;
#endif

int cool_init(void)
{
	int ret;
	cd.freq_cnt = sgxfreq_get_freq_list(&cd.freq_list);
	if (!cd.freq_cnt || !cd.freq_list)
		return -EINVAL;

	ret = thermal_cooling_dev_register(&cool_dev);
	if (ret)
		return ret;

	return thermal_cooling_dev_register(&case_cool_dev);
}

void cool_deinit(void)
{
	thermal_cooling_dev_unregister(&cool_dev);
	thermal_cooling_dev_unregister(&case_cool_dev);
}

static int cool_device(struct thermal_dev *dev, int cooling_level)
{
	int freq_max_index, freq_limit_index;

#if defined(CONFIG_CASE_TEMP_GOVERNOR)
	if (!strcmp(dev->domain_name, "case"))
	{
		int tmp = 0;
		tmp = cooling_level - case_subzone_number;
		if (tmp < 0)
			tmp = 0;
		case_cooling_level = tmp;
	}
	else
#endif
	{
               gpu_cooling_level = cooling_level;
	}

	freq_max_index = cd.freq_cnt - 1;
#if defined(CONFIG_CASE_TEMP_GOVERNOR)
	if (case_cooling_level > gpu_cooling_level)
	{
		freq_limit_index = freq_max_index - case_cooling_level;
	}
	else
#endif
	{
		freq_limit_index = freq_max_index - gpu_cooling_level;
	}

	if (freq_limit_index < 0)
		freq_limit_index = 0;

	sgxfreq_set_freq_limit(cd.freq_list[freq_limit_index]);

	return 0;
}
