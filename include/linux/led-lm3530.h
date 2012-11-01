/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef _LINUX_LED_LD_LM3530_H__
#define _LINUX_LED_LD_LM3530_H__

#define	MANUAL		0
#define	AUTOMATIC	1
#define	MANUAL_SENSOR	2


#define LD_LM3530_LED_DEV "lcd-backlight"

#define LD_LM3530_NAME "lm3530_led"

#define LD_LM3530_LAST_BRIGHTNESS_MASK 0xFE

#define LD_LM3530_ALLOWED_R_BYTES 1
#define LD_LM3530_ALLOWED_W_BYTES 2
#define LD_LM3530_MAX_RW_RETRIES 5
#define LD_LM3530_I2C_RETRY_DELAY 10

#define LM3530_VERSION_REG		0xCC
#define LM3530_GEN_CONFIG		0x10
#define LM3530_ALS_CONFIG		0x20
#define LM3530_BRIGHTNESS_RAMP_RATE	0x30
#define LM3530_ALS_ZONE_REG		0x40
#define LM3530_ALS_RESISTOR_SELECT	0x41
#define LM3530_BRIGHTNESS_CTRL_REG	0xA0
#define LM3530_ALS_ZB0_REG		0x60
#define LM3530_ALS_ZB1_REG		0x61
#define LM3530_ALS_ZB2_REG		0x62
#define LM3530_ALS_ZB3_REG		0x63
#define LM3530_ALS_Z0T_REG		0x70
#define LM3530_ALS_Z1T_REG		0x71
#define LM3530_ALS_Z2T_REG		0x72
#define LM3530_ALS_Z3T_REG		0x73
#define LM3530_ALS_Z4T_REG		0x74

#define LM3530_ALS_READ_MASK	0x07
#define LM3530_GEN_CONF_MASK	0xE3
#define LM3530_MAX_LED_VALUE	0xFF
#define LM3530_MANUAL_VALUE		0x64
#define LM3530_SENSOR_ENABLE	0x08

#ifdef __KERNEL__
struct lm3530_platform_data {
	u8  power_up_gen_config;
	u8  gen_config;
	u8  als_config;
	u8  brightness_ramp;
	u8  als_zone_info;
	u8  als_resistor_sel;
	u8  brightness_control;
	u8 zone_boundary_0;
	u8 zone_boundary_1;
	u8 zone_boundary_2;
	u8 zone_boundary_3;
	u8 zone_target_0;
	u8 zone_target_1;
	u8 zone_target_2;
	u8 zone_target_3;
	u8 zone_target_4;
	u8 zone_data_0;
	u8 zone_data_1;
	u8 zone_data_2;
	u8 zone_data_3;
	u8 zone_data_4;
	u8 manual_current;
	u8 upper_curr_sel;
	u8 lower_curr_sel;
	u8 lens_loss_coeff;
	u8 manual_als_config;
	u8 als_enabled;
} __attribute__ ((packed));

#endif	/* __KERNEL__ */
#endif	/* _LINUX_LED_LD_LM3530_H__ */
