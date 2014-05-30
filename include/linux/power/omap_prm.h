/*
 * Copyright (C) 2014 Motorola Mobility LLC
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

#ifndef _POWER_OMAP_PRM_H
#define _POWER_OMAP_PRM_H

#if IS_ENABLED(CONFIG_POWER_TI_HARDWARE_VOLTAGE_CONTROL)
/*
 * omap_prm_configure - Will configure global PRM register to low power values
 *			for voltage setup timings and state according device
 *			device tree properties.
 * bool off:		If true then OFF state will be configured, otherwise
 *			retention state. At least one of these state has to be
 *			enabled in device tree with auto_off or auto_retention
 *			flags&pointer
 * Returns 0 if success, otherwise error code.
 */
int omap_prm_configure(bool off);

void omap_pm_enable_off_mode(void);
void omap_pm_disable_off_mode(void);
bool omap_pm_get_off_mode(void);
#else
static inline int omap_prm_configure(bool off)
{
	return -ENODEV;
}
static inline void omap_pm_enable_off_mode(void)
{
}
static inline void omap_pm_disable_off_mode(void)
{
}
static inline bool omap_pm_get_off_mode(void)
{
	return false;
}
#endif

#endif				/* _POWER_OMAP_PRM_H */
