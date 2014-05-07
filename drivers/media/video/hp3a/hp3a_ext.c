/*
 * drivers/media/video/hp3a/hp3a_ext.c
 *
 * HP Imaging/3A Driver : Exported function implementation.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
 *
 * Contributors:
 *		Tanvir Islam <tanvir.islam@hp.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>

#include "hp3a.h"
#include "hp3a_common.h"

/**
 * hp3a_hw_enabled - Notify HW is enabled.
 *
 * No return value.
 **/
void hp3a_hw_enabled(u8 enable)
{
	if (enable)
		g_tc.hw_initialized = 1;
	else
		g_tc.hw_initialized = 0;
}

/**
 * hp3a_ccdc_done - End of ccdc readout specific 3A tasks.
 *
 * No return value.
 **/
void hp3a_ccdc_done(void)
{
	hp3a_update_stats_readout_done();
}
EXPORT_SYMBOL(hp3a_ccdc_done);

/**
 * hp3a_ccdc_start - Start of ccdc readout specific 3A tasks.
 *
 * No return value.
 **/
void hp3a_ccdc_start(void)
{
	++g_tc.frame_count;
	hp3a_schedule_task();
}
EXPORT_SYMBOL(hp3a_ccdc_start);

/**
 * hp3a_frame_done - End of frame 3A task dispatcher .
 *
 * No return value.
 **/
void hp3a_frame_done(void)
{
	hp3a_update_stats_pipe_done();
}
EXPORT_SYMBOL(hp3a_frame_done);

/**
 * hp3a_update_wb - Update WB related hw settings.
 *
 * No return value.
 **/
void hp3a_update_wb(void)
{
	hp3a_update_hardpipe();
}
EXPORT_SYMBOL(hp3a_update_wb);

/**
 * hp3a_stream_on - Perform stream on specific tasks.
 *
 * No return value.
 **/
void hp3a_stream_on(void)
{
	g_tc.frame_count = 0;
	memset(&g_tc.sensor_current, 0, sizeof(struct hp3a_sensor_param));
	memset(&g_tc.sensor_requested, 0, sizeof(struct hp3a_sensor_param));
	memset(&g_tc.sensor_stats, 0, sizeof(struct hp3a_sensor_param));
	hp3a_flush_queue_irqsave(&g_tc.ready_stats_queue);
	hp3a_flush_queue_irqsave(&g_tc.hist_hw_queue);
	g_tc.raw_cap_sched_count = 0;
	g_tc.v4l2_streaming = 1;

	hp3a_enable_histogram();
	hp3a_update_hardpipe();
}
EXPORT_SYMBOL(hp3a_stream_on);

/**
 * hp3a_stream_off - Perform stream off specific tasks.
 *
 * No return value.
 **/
void hp3a_stream_off(void)
{
	g_tc.v4l2_streaming = 0;
	g_tc.raw_cap_sched_count = 0;
	g_tc.update_hardpipe = 0;
	hp3a_flush_queue_irqsave(&g_tc.sensor_write_queue);
}
EXPORT_SYMBOL(hp3a_stream_off);

/**
 * hp3a_set_sensor_sync - Set sync delay for exposure and gain
 *			setting updata.
 *
 * Return 1 if busy, 0 otherwise.
 **/
void hp3a_set_sensor_sync(unsigned char exposure, unsigned char gain)
{
	g_tc.exposure_sync = exposure;
	g_tc.gain_sync = gain;
}
EXPORT_SYMBOL(hp3a_set_sensor_sync);
