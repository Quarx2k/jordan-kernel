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

#ifndef _POWER_AVS_OMAP_VC_PRM_H
#define _POWER_AVS_OMAP_VC_PRM_H

struct omap_pmic;

#if IS_ENABLED(CONFIG_POWER_TI_HARDWARE_VOLTAGE_CONTROL)
int __init omap_glbl_prm_init(void);
void __exit omap_glbl_prm_exit(void);
int omap_prm_voltsetup(struct device *dev, struct omap_pmic *pmic, u32 uv);
#else
static inline int __init omap_glbl_prm_init(void)
{
	return -ENODEV;
}
static inline void __exit omap_glbl_prm_exit(void)
{
}
static inline int omap_prm_voltsetup(struct device *dev, struct omap_pmic *pmic,
				u32 uv)
{
	return -ENODEV;
}
#endif

#endif				/* _POWER_AVS_OMAP_VC_PRM_H */
