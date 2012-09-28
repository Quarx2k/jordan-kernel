/*
 * drivers/media/video/hp3a/hp3a_common.h
 *
 * HP Imaging/3A library common functions and definitions.
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
 */

#ifndef	__HP3A_COMMON_H_INCLUDED
#define __HP3A_COMMON_H_INCLUDED

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>

#include "hp3a_queue.h"
#include "hp3a_af.h"

#define  SUCCEEDED(x)		(x == 0)
#define  FAILED(x)		(x < 0)
#define  ALIGN_TO(x, b)		(((unsigned long)x + (b - 1)) & ~(b - 1))
#define  NR_PAGES(x, y)		((((y + x - 1) & PAGE_MASK) >> PAGE_SHIFT) - \
					((x & PAGE_MASK) >> PAGE_SHIFT) + 1)
#define  WRITE_REG(x, v)	(x = (u32)(v))
#define  OR_REG(x, v)		(x |= (u32)(v))
#define  AND_REG(x, v)		(x &= (u32)v)

/* hp3a specific default values. */
#define  MIN_RAW_CAPTURE_INTERVAL	1
#define  MAX_STAT_BUFFERS_PER_FRAME	3

enum {
	HISTOGRAM = 1,
	PAXEL = 2,
	BAYER = 3,
};

/**
 * struct hp3a_reg: Register reag/write data structure.
 * @len: Length of register data.
 * @reg: Register address.
 * @val: Register value.
 **/
struct hp3a_reg {
	u16 len;
	u32 reg;
	u32 val;
};

/**
 * hp3a_reg_page: Page of register page reag/write data structure.
 * @len: Length of register page array.
 * @regs: Pointer to an array of registers.
 **/
struct hp3a_reg_page {
	u16 len;
	struct hp3a_reg *regs;
};

/**
 * struct hp3a_RequestBufffers - Structure for requesting kernel buffer space.
 * @count: number of buffer spaces requested.
 * @type: Type of buffer - currently unused.
 **/
struct hp3a_request_bufffers{
	int count;
	int type;
};

/**
 * struct hp3a_Buffer - Structure for user to kernel buffer transaction..
 * @index: index of the buffer.
 * @buffer_size: Size of the buffer.
 * @addr: address of the buffer.
 **/
struct hp3a_buffer{
	int index;
	u32 buffer_size;
	unsigned long addr;
};

/**
 * struct hp3a_InternalBuffer - Internal data structure for buffer management.
 **/
struct hp3a_internal_buffer{
	int index;
	int type;
	u32 buffer_size;
	struct page **pages;
	unsigned long user_addr;
	unsigned long isp_addr;
};

/**
 * struct hp3a_3x3woff - Structure for 3x3 matrix with offset.
 * @matrix: Two dimentional array for storing 3x3 matrix.
 * @offset: One dimentional array for storing 3 offsets.
 **/
struct hp3a_3x3with_offset {
	u16 matrix[3][3];
	u16 offset[3];
};

/**
 * struct hp3a_SensorParam - Structure for setting sensor specific parameters.
 * @fame_number: V4L2 device index.
 * @exposure: Exposure value is micro seconds.
 * @gain: Linear analog gain value Q8 format.
 **/
struct hp3a_sensor_param {
	u32 exposure;
	u16 gain;
	u16 fps;
};

/**
 * struct hp3a_SensorParam - Structure for setting sensor specific parameters.
 * @fame_number: V4L2 device index.
 * @exposure: Exposure value is micro seconds.
 * @gain: Linear analog gain value Q8 format.
 **/
struct hp3a_sensor_param_internal {
	int v4l2_dev;
	u32 frame_id;
	u32 exposure;
	u16 gain;
	u16 fps;
};

/**
 * struct hp3a_HardPipeParam - Structure for setting hard pipe specific parameters.
 * @dgain: Digital gain.
 * @r_gain: Red channel gain.
 * @gb_gain: Green-blue channel gain.
 * @gr_gain: Green-reg channel gain.
 * @b_gain: Blue channel gain.
 * @rgb2rgb: Rgb to rgb blending matrix.
 **/
struct hp3a_hardpipe_param {
	u16 dgain;
	u8 r_gain;
	u8 gb_gain;
	u8 gr_gain;
	u8 b_gain;
	struct hp3a_3x3with_offset rgb2rgb;
};

/**
 * struct hp3a_histogram_config - Structure for configuring histogram hw
 **/
struct hp3a_histogram_config {
   u32 enable;
	u8 hist_source;		/* CCDC or Memory */
	u8 input_bit_width;	/* Needed o know the size per pixel */
	u8 hist_frames;	/* Num frames to be processed, accumulated */
	u8 hist_h_v_info;	/* input width and height if source is memory */
	u8 hist_packed_pxl;	/* If data is packed packed 8 into 16 bits */
	u16 hist_radd;		/* frame-input address in memory */
	u16 hist_radd_off;	/* line-offset for frame-input */
	u16 hist_bins;		/* number of bins: 32, 64, 128, or 256 */
	u16 wb_gain_R;		/* White Balance Field-to-Pattern Assignments */
	u16 wb_gain_RG;		/* White Balance Field-to-Pattern Assignments */
	u16 wb_gain_B;		/* White Balance Field-to-Pattern Assignments */
	u16 wb_gain_BG;		/* White Balance Field-to-Pattern Assignments */
	u8 num_regions;		/* number of regions to be configured */
	u32 reg0_hor;		/* Region 0 size and position */
	u32 reg0_ver;		/* Region 0 size and position */
	u32 reg1_hor;		/* Region 1 size and position */
	u32 reg1_ver;		/* Region 1 size and position */
	u32 reg2_hor;		/* Region 2 size and position */
	u32 reg2_ver;		/* Region 2 size and position */
	u32 reg3_hor;		/* Region 3 size and position */
	u32 reg3_ver;		/* Region 3 size and position */
};

/**
 * struct hp3a_af_config - AF stats configuration data structure.
 * @enable: Flag to enable/disable AF engine
 * @alaw_enable: Flag to enable/disable alaw
 * @mode: Accumulation mode.
 * @rgbpos: RGB pattern position.
 **/
struct hp3a_af_config {
   u32 enable;
   int alaw_enable;
   int mode;
   int rgbpos;
   struct hp3a_af_hmf  hmf;
   struct hp3a_af_iir iir;
   struct hp3a_af_paxel paxel;
};

/**
 * struct hp3a_raw_config - - RAW stats configuration data structure.
 * @enable: Flag to enable/disable AF engine
 * @frequency: frame frequebcy of raw data collection
* @width: Raw frame width.
 * @height Raw frame height.
 **/
struct hp3a_raw_config {
   u32 enable;
   int frequency;
	u16 width;
	u16 height;
};

/**
 * struct hp3a_Statistics - Data structure for collecting all statistics.
 * @frame_id: frame index.
 * @exposure: Exposue time for current frame.
 * @gain: Gain value for current frame.
 * @hist_stat_index: Index to the hist stats buffer.
 * @af_stat_index: Index to the af stats buffer.
 * @raw_stat_index: Index to the raw stats buffer.
 **/
struct hp3a_statistics {
	u32 frame_id;
	u32 exposure;
	u16 gain;
	u16 fps;
	u16 raw_width;
	u16 raw_height;
	int hist_stat_index;
	int af_stat_index;
	int raw_frame_index;
};

/**
 * struct hp3a_context - Structure for storing hp3a framework
 * context specific data.
 **/
struct hp3a_context {
	u8 initialized;
	u8 hw_initialized;
	int default_v4l2_dev;
	int v4l2_streaming;
	int update_hardpipe;
	int hist_done;
	int hist_hw_configured;
	int hist_hw_enable;
	int af_hw_configured;
	int af_hw_enable;
	int raw_hw_configured;
	int raw_frequency;
	int raw_cap_sched_count;
	int isp_ctx_saved;
	u8 exposure_sync;
	u8 gain_sync;
	u32 hist_bin_size;
	struct hp3a_sensor_param sensor_current;
	struct hp3a_sensor_param sensor_requested;
	struct hp3a_sensor_param sensor_stats;
	u32 frame_count;
	u32 req_af_buffer_size;
	u32 req_raw_buffer_size;
	u16 raw_width;
	u16 raw_height;
	spinlock_t stats_lock;
	spinlock_t hist_lock;
	spinlock_t af_lock;
	spinlock_t hardpipe_lock;
	struct completion frame_done;
	wait_queue_head_t stats_done;
	struct hp3a_sensor_param sensor_param;
	struct hp3a_hardpipe_param hpipe_param;
	struct hp3a_queue hist_stat_queue;
	struct hp3a_queue hist_hw_queue;
	struct hp3a_queue af_stat_queue;
	struct hp3a_queue raw_frame_queue;
	struct hp3a_queue sensor_write_queue;
	struct hp3a_queue sensor_read_queue;
	struct hp3a_queue ready_stats_queue;
	struct hp3a_internal_buffer *histogram_buffer;
	struct hp3a_internal_buffer *af_buffer;
	struct hp3a_internal_buffer *raw_buffer;
};

/**
 * struct device_params - Global device information structure.
 * @v4l2_dev: V4L2 device index.
 * @opened: Device state, open reference count.
 **/
struct hp3a_dev {
	struct device *dev;
};

/**
 * struct hp3a_fh - Per-filehandle data structure
 * @v4l2_dev: V4L2 device index.
 * @device: hp3a device reference.
 * @buffer_count: Buffer count for user mapped buffers.
 * @buffers: List of user mapped buufers.
 **/
struct hp3a_fh {
	int v4l2_dev;
	struct hp3a_dev *device;
	int buffer_count;
	struct hp3a_internal_buffer *buffers;
};

/**
 * Driver function declarations
 **/
long hp3a_unlocked_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg);

/**
 * Framework function declarations
 **/
void initialize_hp3a_framework(struct hp3a_dev *device);
void deinitialize_hp3a_framework(void);
void hp3a_framework_start(struct hp3a_fh *fh);
void hp3a_framework_stop(struct hp3a_fh *fh);
int hp3a_set_sensor_param(struct hp3a_sensor_param *param,
	struct hp3a_fh *fh);
int hp3a_set_hardpipe_param(struct hp3a_hardpipe_param *param,
	struct hp3a_fh *fh);
int hp3a_collect_statistics(struct hp3a_statistics *stat);
void hp3a_update_stats_readout_done(void);
void hp3a_update_stats_pipe_done(void);
void hp3a_schedule_task(void);

/**
 * Histogram function declarations
 **/
void hp3a_enable_histogram(void);
void hp3a_disable_histogram(void);
int hp3a_config_histogram(struct hp3a_histogram_config *config,
	struct hp3a_fh *fh);

/**
 * AF paxel function declarations
 **/
void hp3a_enable_af(void);
void hp3a_disable_af(void);
int hp3a_config_af(struct hp3a_af_config *config, struct hp3a_fh *fh);

/**
 * Raw stats function declarations
 **/
void hp3a_enable_raw(unsigned long buffer_addr);
void hp3a_disable_raw(void);
int hp3a_configure_raw(struct hp3a_raw_config *raw);

/**
 * Hardpipe function declarations
 **/
void hp3a_update_hardpipe(void);

/**
 * Library function declarations
 **/
struct page **map_user_memory(unsigned long addr, u32 size);
void unmap_user_memory(struct page **pages, int nr_pages);
int map_user_to_kernel(struct hp3a_buffer *src,
	struct hp3a_internal_buffer *dest);
void unmap_buffer_from_kernel(struct hp3a_internal_buffer *buffers);
void flush_dcache_ibuffer(struct hp3a_internal_buffer  *ibuffer);
void hp3a_clear_regs(struct hp3a_reg *regs);
void hp3a_read_ispregs(struct hp3a_reg *regs);
void hp3a_write_ispregs(struct hp3a_reg *regs);
int hp3a_read_ispregs_to_user(struct hp3a_reg_page *user_page);
int hp3a_read_ispreg_to_user(struct hp3a_reg *user_reg);

/**
 * External global task data.
 **/
extern struct hp3a_context g_tc;

#endif	/* __HP3A_COMMON_H_INCLUDED */
