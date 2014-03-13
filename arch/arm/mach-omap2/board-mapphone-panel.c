/*
 * linux/arch/arm/mach-omap2/board-mapphone-panel.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 Motorola, Inc.
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
#include <plat/panel.h>
#include <plat/control.h>

#include <mach/dt_path.h>
#include <asm/prom.h>

#ifdef DEBUG
#define PANELDBG(format, ...) \
	printk(KERN_DEBUG "board_panel: " format, \
		## __VA_ARGS__)
#else /* DEBUG */
#define PANELDBG(format, ...)
#endif

#ifndef CONFIG_ARM_OF
#error CONFIG_ARM_OF must be defined for Mapphone to compile
#endif
#ifndef CONFIG_USER_PANEL_DRIVER
#error CONFIG_USER_PANEL_DRIVER must be defined for Mapphone to compile
#endif

static bool mapphone_panel_device_read_dt; /* This is by default false */

/* This must be match in the DT */
enum omap_dss_device_disp_intf {
	OMAP_DSS_DISP_INTF_RGB16	= 1,
	OMAP_DSS_DISP_INTF_RGB24	= 2,
	OMAP_DSS_DISP_INTF_MIPI_L4_CM	= 3,
	OMAP_DSS_DISP_INTF_MIPI_VP_CM	= 4,
	OMAP_DSS_DISP_INTF_MIPI_L4_VM 	= 5,
	OMAP_DSS_DISP_INTF_MIPI_VP_VM 	= 6
};

struct regulator *display_regulator;
static bool first_boot;

static int mapphone_panel_enable(struct omap_dss_device *dssdev);
static void mapphone_panel_disable(struct omap_dss_device *dssdev);

static struct omap_dss_device mapphone_lcd_device = {
	.type = OMAP_DISPLAY_TYPE_DSI,
	.name = "lcd",
	.driver_name = "mapphone-panel",
	.phy.dsi.clk_lane = 1,
	.phy.dsi.clk_pol = 0,
	.phy.dsi.data1_lane = 2,
	.phy.dsi.data1_pol = 0,
	.phy.dsi.data2_lane = 3,
	.phy.dsi.data2_pol = 0,
	.phy.dsi.div.regn = 13,
	.phy.dsi.div.regm = 170,
	.phy.dsi.div.regm3 = 5,
	.phy.dsi.div.regm4 = 5,
	.phy.dsi.div.lck_div = 1,
	.phy.dsi.div.pck_div = 4,
	.phy.dsi.div.lp_clk_div = 7,
	.reset_gpio = 0,
	.phy.dsi.xfer_mode = OMAP_DSI_XFER_CMD_MODE,
	.platform_enable = mapphone_panel_enable,
	.platform_disable = mapphone_panel_disable,
};

static void mapphone_panel_reset(bool reset)
{
	printk("%s\n",__func__);
	if (reset) {
		if (!first_boot) {
			/* don't toggle reset line when the kernel is booting*/
			gpio_set_value(mapphone_lcd_device.reset_gpio, 0);
			msleep(5);
			gpio_set_value(mapphone_lcd_device.reset_gpio, 1);
			msleep(10);
		} else
			first_boot = false;
	} else
		gpio_set_value(mapphone_lcd_device.reset_gpio, 0);
}

static int mapphone_panel_enable(struct omap_dss_device *dssdev)
{
	printk("%s\n",__func__);
	mapphone_panel_reset(true);

	return 0;
}

static void mapphone_panel_disable(struct omap_dss_device *dssdev)
{
	bool deep_sleep_mode_sup = false;
	printk("%s\n",__func__);
	if (dssdev->driver->deep_sleep_mode)
		deep_sleep_mode_sup = dssdev->driver->deep_sleep_mode(dssdev);

	if (!deep_sleep_mode_sup)
		mapphone_panel_reset(false);
}

static struct omapfb_platform_data mapphone_fb_data = {
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

static struct omap_dss_device *mapphone_dss_devices[] = {
	&mapphone_lcd_device,
};

static struct omap_dss_board_info mapphone_dss_data = {
	.num_devices = ARRAY_SIZE(mapphone_dss_devices),
	.devices = mapphone_dss_devices,
	.default_device = &mapphone_lcd_device,
};

struct platform_device mapphone_dss_device = {
	.name = "omapdss",
	.id = -1,
	.dev = {
		.platform_data = &mapphone_dss_data,
	},
};

static int mapphone_dt_get_dsi_panel_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;
	int disp_intf;

	PANELDBG("dt_get_dsi_panel_info()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node == NULL)
		return -ENODEV;

	/* Retrieve the panel information */
	panel_prop = of_get_property(panel_node, "dsi_clk_lane", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.clk_lane = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_clk_pol", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.clk_pol = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_data1_lane", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.data1_lane = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_data1_pol", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.data1_pol = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_data2_lane", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.data2_lane = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_data2_pol", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.data2_pol = *(u8 *)panel_prop;

	panel_prop = of_get_property(panel_node, "gpio_reset", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.reset_gpio = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "type", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.panel.panel_id = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regn", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.regn = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regm", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.regm = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regm3", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.regm3 = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regm4", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.regm4 = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "lp_clk_div", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.lp_clk_div = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "lck_div", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.lck_div = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "pck_div", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.div.pck_div = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "disp_intf", NULL);
	if (panel_prop != NULL) {
		disp_intf = *(u8 *)panel_prop;

		if ((disp_intf == OMAP_DSS_DISP_INTF_MIPI_VP_CM) ||
			(disp_intf == OMAP_DSS_DISP_INTF_MIPI_L4_CM))
			mapphone_lcd_device.phy.dsi.xfer_mode =
				OMAP_DSI_XFER_CMD_MODE;
		else if ((disp_intf == OMAP_DSS_DISP_INTF_MIPI_VP_VM) ||
			(disp_intf == OMAP_DSS_DISP_INTF_MIPI_L4_VM))
			mapphone_lcd_device.phy.dsi.xfer_mode =
				OMAP_DSI_XFER_VIDEO_MODE;
		else {
			printk(KERN_ERR "Invalid disp_intf in dt = %d\n",
				disp_intf);
			return -ENODEV;
		}
	}

	of_node_put(panel_node);

	return 0;
}

static int mapphone_dt_get_dsi_vm_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;

	PANELDBG("dt_get_dsi_vm_info()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node == NULL)
		return -ENODEV;

	panel_prop = of_get_property(panel_node, "dsi_timing_hsa", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.hsa = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_timing_hfp", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.hfp = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_timing_hbp", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.hbp = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_timing_vsa", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.vsa = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_timing_vfp", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.vfp = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "dsi_timing_vbp", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.vm_timing.vbp = *(u16 *)panel_prop;

	of_node_put(panel_node);

	PANELDBG("DT: hsa=%d hfp=%d hbp=%d vsa=%d vfp=%d vbp=%d \n",
		mapphone_lcd_device.phy.dsi.vm_timing.hsa,
		mapphone_lcd_device.phy.dsi.vm_timing.hfp,
		mapphone_lcd_device.phy.dsi.vm_timing.hbp,
		mapphone_lcd_device.phy.dsi.vm_timing.vsa,
		mapphone_lcd_device.phy.dsi.vm_timing.vfp,
		mapphone_lcd_device.phy.dsi.vm_timing.vbp);

	return 0;
}

static int __init mapphone_dt_panel_init(void)
{
	int ret = 0;

	PANELDBG("dt_panel_init\n");

	if (mapphone_panel_device_read_dt == false) {
		if (mapphone_dt_get_dsi_panel_info() != 0) {
			printk(KERN_ERR "failed to parse DSI panel info \n");
			ret = -ENODEV;
		} else if ((mapphone_lcd_device.phy.dsi.xfer_mode ==
						OMAP_DSI_XFER_VIDEO_MODE) &&
				(mapphone_dt_get_dsi_vm_info() != 0)) {
			printk(KERN_ERR "failed to parse DSI VM info \n");
			ret = -ENODEV;
		} else {
			mapphone_panel_device_read_dt = true;
		}
	}
	return ret;
}

void panel_print(void)
{
		PANELDBG(" DT: clk_lane= %d clk_pos= %d\n",
			mapphone_lcd_device.phy.dsi.clk_lane,
			mapphone_lcd_device.phy.dsi.clk_pol);

		PANELDBG(" DT: data1_lane= %d data1_pos= %d\n",
			mapphone_lcd_device.phy.dsi.data1_lane,
			mapphone_lcd_device.phy.dsi.data1_pol);

		PANELDBG(" DT: data2_lane= %d data2_pos= %d\n",
			mapphone_lcd_device.phy.dsi.data2_lane,
			mapphone_lcd_device.phy.dsi.data2_pol);
		PANELDBG(" DT: gpio_reset=%d xfer_mode=%d panel_id=0x%lx\n",
			mapphone_lcd_device.reset_gpio,
			mapphone_lcd_device.phy.dsi.xfer_mode,
			mapphone_lcd_device.panel.panel_id);


}

static struct platform_device omap_panel_device = {
	.name = "omap-panel",
	.id = -1,
};

void __init mapphone_panel_init(void)
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

	omapfb_set_platform_data(&mapphone_fb_data);

	if (mapphone_dt_panel_init())
		printk(KERN_INFO "panel: using non-dt configuration\n");

	ret = gpio_request(mapphone_lcd_device.reset_gpio, "display reset");
	if (ret) {
		printk(KERN_ERR "failed to get display reset gpio\n");
		return;
	}

	first_boot = true;

	gpio_direction_output(mapphone_lcd_device.reset_gpio, 1);

	platform_device_register(&omap_panel_device);
	platform_device_register(&mapphone_dss_device);

	return;
}

