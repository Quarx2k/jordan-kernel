/*
 * mt9p012.h - Register definitions for the MT9P012 camera sensor.
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Contributors:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Sergio Aguirre <saaguirre@ti.com>
 * 	Martinez Leonides
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef MT9P012_H
#define MT9P012_H


#define MT9P012_I2C_ADDR		0x10

/**
 * struct mt9p012_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @default_regs: Default registers written after power-on or reset.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct mt9p012_platform_data {
#if defined(CONFIG_VIDEO_OLDOMAP3)
	int (*power_set)(struct device* dev, enum v4l2_power power);
	u32 (*set_xclk)(u32 xclkfreq);
	int (*priv_data_set)(void *);
	u8 (*get_config_flags)(void);
#else
	int (*power_set)(struct device*, enum v4l2_power power);
	int (*ifparm)(struct v4l2_ifparm *p);
	int (*priv_data_set)(void *);
	void (*lock_cpufreq)(int lock);
	u8 (*get_config_flags)(void);
	int (*cfg_interface_bridge)(u32);
	int (*csi2_lane_count)(int count);
	int (*csi2_cfg_vp_out_ctrl)(u8 vp_out_ctrl);
	int (*csi2_ctrl_update)(bool);
	int (*csi2_cfg_virtual_id)(u8 ctx, u8 id);
	int (*csi2_ctx_update)(u8 ctx, bool);
	int (*csi2_calc_phy_cfg0)(u32, u32, u32);
#endif
};

#endif /* ifndef MT9P012_H */
