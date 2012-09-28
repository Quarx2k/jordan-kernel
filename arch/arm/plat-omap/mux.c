/*
 * linux/arch/arm/plat-omap/mux.c
 *
 * Utility to set the Omap MUX and PULL_DWN registers from a table in mux.h
 *
 * Copyright (C) 2003 - 2008 Nokia Corporation
 *
 * Written by Tony Lindgren
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <asm/system.h>
#include <linux/spinlock.h>
#include <plat/mux.h>

#ifdef CONFIG_OMAP_MUX

static struct omap_mux_cfg *mux_cfg;

int __init omap_mux_register(struct omap_mux_cfg *arch_mux_cfg)
{
	if (!arch_mux_cfg || !arch_mux_cfg->pins || arch_mux_cfg->size == 0
			|| !arch_mux_cfg->cfg_reg) {
		printk(KERN_ERR "Invalid pin table\n");
		return -EINVAL;
	}

	mux_cfg = arch_mux_cfg;

	return 0;
}

/*
 * Sets the Omap MUX and PULL_DWN registers based on the table
 */
int __init_or_module omap_cfg_reg(const unsigned long index)
{
	struct pin_config *reg;

	if (cpu_is_omap44xx())
		return 0;

#if defined(CONFIG_MOT_FEAT_MDTV) || defined(CONFIG_PANEL_HDTV)\
    || defined(CONFIG_VIB_PWM) || defined(CONFIG_VIDEO_MIPI_INTERFACE)
	int is_mux_config = 0;
#endif

	if (mux_cfg == NULL) {
		printk(KERN_ERR "Pin mux table not initialized\n");
		return -ENODEV;
	}

	if (index >= mux_cfg->size) {
		printk(KERN_ERR "Invalid pin mux index: %lu (%lu)\n",
		       index, mux_cfg->size);
		dump_stack();
		return -ENODEV;
	}

	reg = (struct pin_config *)&mux_cfg->pins[index];

	if (!mux_cfg->cfg_reg)
		return -ENODEV;
#ifdef CONFIG_MOT_FEAT_MDTV
	if (index >= F1_34XX_MDTV_INT_OFF && index <= AA3_34XX_MDTV_CLK_ON)
		is_mux_config = 1;
#endif
#ifdef CONFIG_PANEL_HDTV
	if ((index >= AG22_34XX_DSS_DATA0 && index <= AH24_34XX_DSS_DATA5)
		|| (index >= AG22_34XX_DSI_DX0 && index <= AH24_34XX_DSI_DY2))
		is_mux_config = 1;
#endif
#ifdef CONFIG_VIB_PWM
	if (index == AF22_34XX_GPIO9_OUT)
		is_mux_config = 1;
#endif
#if defined(CONFIG_VIDEO_MIPI_INTERFACE)
	if ((index >= A24_34XX_CAM_HS && index <= H2_34XX_GPMC_A3) ||\
		(index >= AG17_34XX_CAM_D0 && index <= AE17_34XX_CSI2_DY1))
		is_mux_config = 1;
#endif
#if defined(CONFIG_MOT_FEAT_MDTV) || defined(CONFIG_PANEL_HDTV)\
	|| defined(CONFIG_VIB_PWM) || defined(CONFIG_VIDEO_MIPI_INTERFACE)
	if (is_mux_config == 1)
		return mux_cfg->cfg_reg(reg);
	else
		return 0;
#else
	return mux_cfg->cfg_reg(reg);
#endif
}
EXPORT_SYMBOL(omap_cfg_reg);
#else
#define omap_mux_init() do {} while(0)
#define omap_cfg_reg(x)	do {} while(0)
#endif	/* CONFIG_OMAP_MUX */
