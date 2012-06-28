/*
 * hpd_omap3630.c
 *
 * HPD library support functions for TI OMAP3 processors.
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <video/omapdss.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/delay.h>

#include "dss.h"

static struct {
	struct mutex hpd_lock;
	struct switch_dev hpd_switch;
} hpd;

enum {
	DISPLAY_LCD_STATE_ON,
	DISPLAY_HDMI_STATE_ON,
	DISPLAY_TV_STATE_ON,
	INVALID_DISPLAY_STATE,
};


static struct hpd_worker_data {
	struct delayed_work dwork;
	atomic_t state;
} hpd_work;
static struct workqueue_struct *my_workq;

static void hotplug_detect_worker(struct work_struct *work)
{
	struct hpd_worker_data *d = container_of(work, typeof(*d), dwork.work);
	struct omap_dss_device *dssdev = NULL;
	int state = atomic_read(&d->state);

	int match(struct omap_dss_device *dssdev, void *arg)
	{
		return sysfs_streq(dssdev->name , "hdmi");
	}
	dssdev = omap_dss_find_device(NULL, match);

	pr_err("in hpd work %d, state=%d\n", state, dssdev->state);
	if (dssdev == NULL)
		return;

	mutex_lock(&hpd.hpd_lock);
	switch_set_state(&hpd.hpd_switch, state);
	mutex_unlock(&hpd.hpd_lock);
}

int hpd_panel_handler(int hpd)
{
	__cancel_delayed_work(&hpd_work.dwork);

	atomic_set(&hpd_work.state, hpd);
	queue_delayed_work(my_workq, &hpd_work.dwork,
					msecs_to_jiffies(hpd ? 30 : 40));
	return 0;
}

static int lcd_enable(void *data, u64 val)
{
	hpd_panel_handler(DISPLAY_LCD_STATE_ON);
	return 0;
}

static int hdmi_enable(void *data, u64 val)
{
	hpd_panel_handler(DISPLAY_HDMI_STATE_ON);
	return 0;
}

static int tv_out_enable(void *data, u64 val)
{
	hpd_panel_handler(DISPLAY_TV_STATE_ON);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_lcd_fops, NULL, lcd_enable, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(pm_hdmi_fops, NULL, hdmi_enable, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(pm_tv_fops, NULL, tv_out_enable, "%llu\n");

int hpd_panel_init(void)
{
	struct dentry *dbg_dir_hpd;

	mutex_init(&hpd.hpd_lock);
	hpd.hpd_switch.name = "display_support";
	switch_dev_register(&hpd.hpd_switch);
	my_workq = create_singlethread_workqueue("display_hotplug");
	INIT_DELAYED_WORK(&hpd_work.dwork, hotplug_detect_worker);

	dbg_dir_hpd = debugfs_create_dir("hpd_support", NULL);
	(void) debugfs_create_file("switch_to_hdmi", S_IRUGO | S_IWUSR,
					dbg_dir_hpd, NULL, &pm_hdmi_fops);
	(void) debugfs_create_file("switch_to_lcd", S_IRUGO | S_IWUSR,
					dbg_dir_hpd, NULL, &pm_lcd_fops);
	(void) debugfs_create_file("switch_to_tv", S_IRUGO | S_IWUSR,
					dbg_dir_hpd, NULL, &pm_tv_fops);
	return 0;
}

int hpd_panel_exit(void)
{
	destroy_workqueue(my_workq);
	switch_dev_unregister(&hpd.hpd_switch);
	return 0;
}
