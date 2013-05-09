/*
 * OMAP Voltage Controller (VC) interface exported functions
 *
 * Idea based on arch/arm/mach-omap2/vc.h
 * Copyright (C) 2007, 2010 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 * Lesly A M <x0080970@ti.com>
 * Thara Gopinath <thara@ti.com>
 *
 * Copyright (C) 2008, 2011 Nokia Corporation
 * Kalle Jokiniemi
 * Paul Walmsley
 *
 * Copyright (C) 2013 Texas Instruments Incorporated
 * Grygorii Strashko
 * Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _POWER_AVS_OMAP_VC_H
#define _POWER_AVS_OMAP_VC_H

struct omap_pmic;
/* Internal to VC */
struct omap_vc_channel;

/**
 * struct omap_vc_channel_info - Channel information visible to users
 * @pmic:		PMIC pointer
 * @retention_uV:	retention voltage in micro volts
 * @off_uV:		OFF voltage in micro volts
 */
struct omap_vc_channel_info {
	struct omap_pmic *pmic;
	u32 retention_uV;
	u32 off_uV;
	/* private: */
	/* Used internally by Voltage Controller driver */
	struct omap_vc_channel *ch;
};

#if IS_ENABLED(CONFIG_POWER_TI_HARDWARE_VOLTAGE_CONTROL)
struct omap_vc_channel_info *devm_omap_vc_channel_get(struct device *dev,
						      struct omap_pmic *pmic);
int omap_vc_channel_set_on_voltage(struct omap_vc_channel_info *info, u32 uv);
#else
struct inline omap_vc_channel_info *
	devm_omap_vc_channel_get(struct device *dev, struct omap_pmic *pmic)
{
	return ERR_PTR(-ENODEV);
}

static inline int omap_vc_channel_set_on_voltage(struct omap_vc_channel_info
						 *info, u32 uv)
{
	return -ENODEV;
}
#endif

#endif				/* _POWER_AVS_OMAP_VC_H */
