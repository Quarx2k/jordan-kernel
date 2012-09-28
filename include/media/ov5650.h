/*
 * drivers/media/video/ov5650.h
 *
 * Register definitions for the OV5650 CameraChip.
 *
 * Author: Pallavi Kulkarni (ti.com)
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef OV5650_H
#define OV5650_H
#define OV5650_I2C_ADDR		(0x6c >> 1)

/* ISP uses a 10-bit value, OV5650 also uses a 10-bit value */
#define OV5650_BLACK_LEVEL_10BIT	8

struct ov5650_platform_data {
	/* Set power state, zero is off, non-zero is on. */
	int (*power_set)(struct device *dev, enum v4l2_power power);
	/* Default registers written after power-on or reset. */
	const struct ov5650_reg *default_regs;
	int (*ifparm)(struct v4l2_ifparm *p);
	int (*priv_data_set)(void *);
	void (*lock_cpufreq)(int lock);
};

#if defined(CONFIG_LEDS_FLASH_RESET)
extern bool bd7885_device_detection(void);
#endif

#endif /* ifndef OV5650_H */

