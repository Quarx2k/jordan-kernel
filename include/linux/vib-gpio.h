/* include/linux/timed_gpio.h
 *
 * Copyright (C) 2008 Google, Inc.
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

#ifndef _LINUX_VIB_GPIO_H
#define _LINUX_VIB_GPIO_H

#ifdef __KERNEL__

#define VIB_GPIO_NAME "vib-gpio"

struct vib_gpio_platform_data {
	unsigned 	gpio;
	int		max_timeout;
	u8 		active_low;
	int		initial_vibrate;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

#endif /* __KERNEL__ */

void vibrator_haptic_fire(int value);

#endif /* _LINUX_VIB_GPIO_H */
