/* include/linux/vib-omap-pwm.h
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __VIB_PWM_H
#define __VIB_PWM_H
#define VIB_PWM_NAME "vib-pwm"

struct vib_pwm_platform_data {
	int initial_vibrate;
	int (*init) (void);
	void (*exit) (void);
	void (*power_on) (void);
	void (*power_off) (void);
#ifdef CONFIG_VIB_PWM_SWEEP
	void (*pattern) (int);
#endif /* CONFIG_VIB_PWM_SWEEP */
	char *device_name;
};

#endif
