/*
 * Bluetooth TI wl127x rfkill power control via GPIO
 *
 * Copyright (C) 2009 Motorola, Inc.
 * Copyright (C) 2008 Texas Instruments
 * Initial code: Pavan Savoy <pavan.savoy@gmail.com> (wl127x_power.c)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef _LINUX_WL127X_RFKILL_H
#define _LINUX_WL127X_RFKILL_H

#include <linux/rfkill.h>

enum wl127x_devices {
	WL127X_BLUETOOTH = 0,
	WL127X_FM,
	WL127X_MAX_DEV,
};

/* Set bt_nshutdown_gpio or fm_enable_gpio to -1 to disable the corresponding
 * rfkill driver */
struct wl127x_rfkill_platform_data {
	int bt_nshutdown_gpio;
	int fm_enable_gpio;
	int (*bt_hw_init)(void);
	int (*bt_hw_release)(void);
	int (*bt_hw_enable)(void);
	int (*bt_hw_disable)(void);
	struct rfkill *rfkill[WL127X_MAX_DEV];  /* for driver only */
};

#endif
