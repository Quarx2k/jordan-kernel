/*
 * linux/arch/arm/mach-omap2/board-sholes-panel.c
 *
 * Copyright (C) 2009 Google, Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/omapfb.h>

#include <plat/display.h>
#include <plat/gpio.h>
#include <plat/mux.h>
#include <plat/resource.h>

#define SHOLES_DISPLAY_RESET_GPIO	136

struct regulator *display_regulator;

static int sholes_panel_enable(struct omap_dss_device *dssdev)
{
	if (!display_regulator) {
		display_regulator = regulator_get(NULL, "vhvio");
		if (IS_ERR(display_regulator)) {
			printk(KERN_ERR "failed to get regulator for display");
			return PTR_ERR(display_regulator);
		}
		regulator_enable(display_regulator);
		return 0;
	}

	regulator_enable(display_regulator);
	msleep(1);
	gpio_request(SHOLES_DISPLAY_RESET_GPIO, "display reset");
	gpio_direction_output(SHOLES_DISPLAY_RESET_GPIO, 1);
	msleep(5);
	gpio_set_value(SHOLES_DISPLAY_RESET_GPIO, 0);
	msleep(5);
	gpio_set_value(SHOLES_DISPLAY_RESET_GPIO, 1);
	msleep(10);

	return 0;
}

static void sholes_panel_disable(struct omap_dss_device *dssdev)
{
	gpio_direction_output(SHOLES_DISPLAY_RESET_GPIO, 1);
	gpio_set_value(SHOLES_DISPLAY_RESET_GPIO, 0);
	msleep(1);
	regulator_disable(display_regulator);
}

static struct omapfb_platform_data sholes_fb_data = {
	.mem_desc = {
		.region_cnt = 1,
		.region = {
			{
				.format = OMAPFB_COLOR_ARGB32,
				.format_used = 1,
			},
		},
	},
};

static struct omap_dss_device sholes_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DSI,
	.name = "lcd",
	.driver_name = "sholes-panel",
	.phy.dsi.clk_lane = 1,
	.phy.dsi.clk_pol = 0,
	.phy.dsi.data1_lane = 2,
	.phy.dsi.data1_pol = 0,
	.phy.dsi.data2_lane = 3,
	.phy.dsi.data2_pol = 0,
	.phy.dsi.div.regn = 13,
	.phy.dsi.div.regm = 160,
	.phy.dsi.div.regm3 = 7,
	.phy.dsi.div.regm4 = 7,
	.phy.dsi.div.lck_div = 1,
	.phy.dsi.div.pck_div = 4,
	.phy.dsi.div.lp_clk_div = 5,
	.reset_gpio = SHOLES_DISPLAY_RESET_GPIO,
	.platform_enable = sholes_panel_enable,
	.platform_disable = sholes_panel_disable,
};

static struct omap_dss_device *sholes_dss_devices[] = {
	&sholes_lcd_device,
};

static struct omap_dss_board_info sholes_dss_data = {
	.num_devices = ARRAY_SIZE(sholes_dss_devices),
	.devices = sholes_dss_devices,
	.default_device = &sholes_lcd_device,
};

struct platform_device sholes_dss_device = {
        .name          = "omapdss",
        .id            = -1,
        .dev            = {
                .platform_data = &sholes_dss_data,
        },
};

void __init sholes_panel_init(void)
{
	int ret;

	omap_cfg_reg(AG22_34XX_DSI_DX0);
	omap_cfg_reg(AH22_34XX_DSI_DY0);
	omap_cfg_reg(AG23_34XX_DSI_DX1);
	omap_cfg_reg(AH23_34XX_DSI_DY1);
	omap_cfg_reg(AG24_34XX_DSI_DX2);
	omap_cfg_reg(AH24_34XX_DSI_DY2);
	/* disp reset b */
	omap_cfg_reg(AE4_34XX_GPIO136_OUT);

	omapfb_set_platform_data(&sholes_fb_data);

	ret = gpio_request(SHOLES_DISPLAY_RESET_GPIO, "display reset");
	if (ret) {
		printk(KERN_ERR "failed to get display reset gpio\n");
		goto error;
	}

	platform_device_register(&sholes_dss_device);
	return;

error:
	gpio_free(SHOLES_DISPLAY_RESET_GPIO);
}
