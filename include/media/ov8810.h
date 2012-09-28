/*
 * drivers/media/video/ov8810.h
 *
 * Register definitions for the OV8810 CameraChip.
 *
 * Author: Pallavi Kulkarni (ti.com)
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef OV8810_H
#define OV8810_H
#define OV8810_I2C_ADDR		(0x6c >> 1)

/* ISP uses a 10-bit value, OV8810 uses a 12-bit value */
#define OV8810_BLACK_LEVEL_10BIT	8

struct ov8810_platform_data {
	/* Set power state, zero is off, non-zero is on. */
	int (*power_set)(struct device *dev, enum v4l2_power power);
	/* Default registers written after power-on or reset. */
	const struct ov8810_reg *default_regs;
	int (*ifparm)(struct v4l2_ifparm *p);
	int (*priv_data_set)(void *);
	void (*lock_cpufreq)(int lock);
};

#if defined(CONFIG_LEDS_FLASH_RESET)
/* To be removed ! */
extern bool bd7885_device_detection(void);
extern bool bd7885_device_enable(void);
extern bool bd7885_device_disable(void);
#endif

#endif /* ifndef OV8810_H */

