/*
 *  OMAP2PLUS cpufreq driver
 *
 *  Copyright (C) 2005 Nokia Corporation
 *  Written by Tony Lindgren <tony@atomide.com>
 *
 *  Based on cpu-sa1110.c, Copyright (C) 2001 Russell King
 *
 * Copyright (C) 2007-2011 Texas Instruments, Inc.
 * Updated to support OMAP3
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _ARCH_ARM_MACH_OMAP2_OMAP2PLUS_CPUFREQ_H
#define _ARCH_ARM_MACH_OMAP2_OMAP2PLUS_CPUFREQ_H

struct omap_cpufreq_platform_data {
	int max_nominal_freq;
};

void omap_cpufreq_set_platform_data(struct omap_cpufreq_platform_data *pdata);

#endif
