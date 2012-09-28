/*
 * drivers/media/video/omap34xxcam.h
 *
 * Video-for-Linux (Version 2) Camera capture driver for OMAP34xx ISP.
 *
 * Copyright (C) 2008 Texas Instruments.
 * Copyright (C) 2008 Nokia.
 *
 * Contributors:
 *	Sameer Venkatraman <sameerv@ti.com>
 *	Mohit Jalori <mjalori@ti.com>
 *	Sakari Ailus <sakari.ailus@nokia.com>
 *	Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
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

#ifndef OMAP34XXCAM_H
#define OMAP34XXCAM_H

#include <media/v4l2-int-device.h>
#include "oldisp/isp.h"

#define CAM_NAME "omap34xxcam"

#define OMAP_ISP_AF		(1 << 4)
#define OMAP_ISP_HIST		(1 << 5)
#define OMAP34XXCAM_XCLK_NONE	-1
#define OMAP34XXCAM_XCLK_A	0
#define OMAP34XXCAM_XCLK_B	1

#define OMAP34XXCAM_SLAVE_SENSOR	0
#define OMAP34XXCAM_SLAVE_LENS		1
#define OMAP34XXCAM_SLAVE_FLASH		2 /* This is the last slave! */

#define OMAP34XXCAM_VIDEODEVS		4

struct omap34xxcam_device;
struct omap34xxcam_videodev;

struct omap34xxcam_hw_csi2_lanes_data {
	unsigned polarity:1;
	unsigned position:3;
};

struct omap34xxcam_hw_csi2_lanes {
	struct omap34xxcam_hw_csi2_lanes_data data[4];
	struct omap34xxcam_hw_csi2_lanes_data clock;
};

struct omap34xxcam_hw_csi2_phy {
	u8 ths_term;
	u8 ths_settle;
	u8 tclk_term;
	unsigned tclk_miss:1;
	u8 tclk_settle;
};

struct omap34xxcam_hw_csi2 {
	struct omap34xxcam_hw_csi2_lanes lanes;
	struct omap34xxcam_hw_csi2_phy phy;
};

struct omap34xxcam_sensor_config {
	int xclk;
	int sensor_isp;
	u32 capture_mem;
};

struct omap34xxcam_lens_config {
};

struct omap34xxcam_flash_config {
};

/**
 * struct omap34xxcam_hw_config - struct for vidioc_int_g_priv ioctl
 * @xclk: OMAP34XXCAM_XCLK_A or OMAP34XXCAM_XCLK_B
 * @sensor_isp: Is sensor smart/SOC or raw
 * @s_pix_sparm: Access function to set pix and sparm.
 * Pix will override sparm
 */
struct omap34xxcam_hw_config {
	int dev_index; /* Index in omap34xxcam_sensors */
	int dev_minor; /* Video device minor number */
	int dev_type; /* OMAP34XXCAM_SLAVE_* */
	int interface_type; /* Interface type */
	union {
		struct omap34xxcam_sensor_config sensor;
		struct omap34xxcam_lens_config lens;
		struct omap34xxcam_flash_config flash;
	} u;
	union {
		struct omap34xxcam_hw_csi2 hw_csi2;
	} csi2;
};

/**
 * struct omap34xxcam_videodev - per /dev/video* structure
 * @mutex: serialises access to this structure
 * @cam: pointer to cam hw structure
 * @master: we are v4l2_int_device master
 * @sensor: sensor device
 * @lens: lens device
 * @flash: flash device
 * @slaves: how many slaves we have at the moment
 * @vfd: our video device
 * @capture_mem: maximum kernel-allocated capture memory
 * @if_u: sensor interface stuff
 * @index: index of this structure in cam->vdevs
 * @users: how many users we have
 * @sensor_config: ISP-speicific sensor configuration
 * @lens_config: ISP-speicific lens configuration
 * @flash_config: ISP-speicific flash configuration
 * @streaming: streaming file handle, if streaming is enabled
 */
struct omap34xxcam_videodev {
	struct mutex mutex; /* For serializing access to this structure */

	struct omap34xxcam_device *cam;
	struct v4l2_int_device master;

#define vdev_sensor slave[OMAP34XXCAM_SLAVE_SENSOR]
#define vdev_lens slave[OMAP34XXCAM_SLAVE_LENS]
#define vdev_flash slave[OMAP34XXCAM_SLAVE_FLASH]
	struct v4l2_int_device *slave[OMAP34XXCAM_SLAVE_FLASH + 1];

	/* number of slaves attached */
	int slaves;

	/*** video device parameters ***/
	struct video_device *vfd;
	int capture_mem;

	/*** general driver state information ***/
	/*
	 * Sensor interface parameters: interface type, CC_CTRL
	 * register value and interface specific data.
	 */
	u32 xclk;
	/* index to omap34xxcam_videodevs of this structure */
	int index;
	atomic_t users;

#define vdev_sensor_config slave_config[OMAP34XXCAM_SLAVE_SENSOR].u.sensor
#define vdev_lens_config slave_config[OMAP34XXCAM_SLAVE_LENS].u.lens
#define vdev_flash_config slave_config[OMAP34XXCAM_SLAVE_FLASH].u.flash
	struct omap34xxcam_hw_config slave_config[OMAP34XXCAM_SLAVE_FLASH + 1];

	/*** capture data ***/
	struct v4l2_fract want_timeperframe;
	struct v4l2_pix_format want_pix;
	/* file handle, if streaming is on */
	struct file *streaming;
};

/**
 * struct omap34xxcam_device - per-device data structure
 * @mutex: mutex serialises access to this structure
 * @sgdma_in_queue: Number or sgdma requests in scatter-gather queue,
 * protected by the lock above.
 * @sgdma: ISP sgdma subsystem information structure
 * @dma_notify: DMA notify flag
 * @irq: irq number platform HW resource
 * @mmio_base: register map memory base (platform HW resource)
 * @mmio_base_phys: register map memory base physical address
 * @mmio_size: register map memory size
 * @dev: device structure
 * @vdevs: /dev/video specific structures
 * @fck: camera module fck clock information
 * @ick: camera module ick clock information
 */
struct omap34xxcam_device {
	struct mutex mutex; /* For serializing access to this structure */
	int sgdma_in_queue;
	struct isp_sgdma sgdma;
	int dma_notify;

	/*** platform HW resource ***/
	unsigned int irq;
	unsigned long mmio_base;
	unsigned long mmio_base_phys;
	unsigned long mmio_size;

	/*** interfaces and device ***/
	struct device *dev;
	struct omap34xxcam_videodev vdevs[OMAP34XXCAM_VIDEODEVS];

	/*** camera module clocks ***/
	struct clk *fck;
	struct clk *ick;
	bool sensor_if_enabled;
};

/**
 * struct omap34xxcam_fh - per-filehandle data structure
 * @vbq_lock: spinlock for the videobuf queue
 * @vbq: V4L2 video buffer queue structure
 * @pix: V4L2 pixel format structure (serialise pix by vbq->lock)
 * @field_count: field counter for videobuf_buffer
 * @vdev: our /dev/video specific structure
 */
struct omap34xxcam_fh {
	spinlock_t vbq_lock; /* For the videobuf queue */
	struct videobuf_queue vbq;
	struct v4l2_pix_format pix;
	atomic_t field_count;
	/* accessing cam here doesn't need serialisation: it's constant */
	struct omap34xxcam_videodev *vdev;
};

#ifdef CONFIG_VIDEO_OMAP3_HP3A
/**
 * struct cam_reg - per sensor register read/write
 * @len: length of register
 * @reg: address of register
 * @val: value of the register
 */
struct cam_reg{
	u16 len;
	u32 reg;
	u32 val;
};

/**
 * struct cam_sensor_settings - per sensor register read/write
 * @flags: length of register
 * @exposure: exposure value to be set
 * @gain: analog gain value tobe set
 * @regs: number of registers
 * @reg_data: pointer to a array of struct cam_reg
 */
struct cam_sensor_settings{
	u32 flags;
	u32 exposure;
	u16 gain;
        u16 fps;
	u16 regs;
	void *reg_data;
};

#define	OMAP34XXCAM_SET_EXPOSURE		0x1
#define	OMAP34XXCAM_SET_GAIN			         0x2
#define	OMAP34XXCAM_READ_REGS			   0x4
#define	OMAP34XXCAM_WRITE_REGS			   0x8
#define	OMAP34XXCAM_SET_FPS                   0x10

#define	OMAP34XXCAM_REG_8BIT			         0x1
#define	OMAP34XXCAM_REG_16BIT			      0x2
#define	OMAP34XXCAM_REG_32BIT			      0x4
#define	OMAP34XXCAM_REG_END			         0xFF

#define V4L2_CID_PRIVATE_SENSOR_READ_REG	(V4L2_CID_PRIVATE_BASE + 20)
#define V4L2_CID_PRIVATE_SENSOR_WRITE_REG	(V4L2_CID_PRIVATE_BASE + 21)

int omap34xxcam_sensor_settings(int dev, struct cam_sensor_settings *settings);
#endif

#endif /* ifndef OMAP34XXCAM_H */
