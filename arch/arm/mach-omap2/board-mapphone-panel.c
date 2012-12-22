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

#include <plat/dispsw.h>
#include <media/tda19989_platform.h>

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

#define MAPPHONE_HDMI_DSS_0_5_MUX  (OMAP34XX_MUX_MODE0 | OMAP34XX_PIN_OUTPUT)

static u16 mapphone_hdmi_stored_mux[6];
static int mapphone_feature_hdmi;

static struct tda19989_platform_data mapphone_tda19989_data = {
	.pwr_en_gpio = 0,
	.int_gpio = 0,
	.cec_i2c_dev = 0x34,  /* Hardcoding is ok here */
	.cec_reg_name = "vwlan2",
};

static struct platform_device mapphone_tda19989_device = {
	.name = "tda19989",
	.dev = {
		.platform_data = &mapphone_tda19989_data,
	},
};

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

static int mapphone_hdtv_mux_en_gpio;	/* 0 by default */
static int mapphone_hdtv_mux_sel_gpio;	/* 0 by default */

static int mapphone_panel_enable_hdtv(struct omap_dss_device *dssdev);
static void mapphone_panel_disable_hdtv(struct omap_dss_device *dssdev);

static struct omap_dss_device mapphone_hdtv_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "hdtv",
	.driver_name = "hdtv-panel",
	.phy.dpi.data_lines = 24,
	.panel.config = OMAP_DSS_LCD_TFT,
	.platform_enable = mapphone_panel_enable_hdtv,
	.platform_disable = mapphone_panel_disable_hdtv,
};

static int mapphone_panel_regulator(bool enable)
{
	static int regulator_cnt;

	if (enable) {
		if (!regulator_cnt) {
			display_regulator = regulator_get(NULL, "vhvio");
			if (IS_ERR(display_regulator)) {
				printk(KERN_ERR "failed to get regulator "
						"for display \n");
				return PTR_ERR(display_regulator);
			}

			regulator_enable(display_regulator);
			mdelay(1);
		}
		regulator_cnt++;
	} else {
		if (regulator_cnt != 0) {
			regulator_cnt--;
			if (!regulator_cnt)
				regulator_disable(display_regulator);
		} else
			printk(KERN_WARNING "regulatore is reached zero \n");
	}

	return regulator_cnt;
}

static void mapphone_panel_reset(bool reset)
{
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
	mapphone_panel_regulator(true);
	mapphone_panel_reset(true);

	return 0;
}

static void mapphone_panel_disable(struct omap_dss_device *dssdev)
{
	bool deep_sleep_mode_sup = false;

	if (dssdev->driver->deep_sleep_mode)
		deep_sleep_mode_sup = dssdev->driver->deep_sleep_mode(dssdev);

	if (!deep_sleep_mode_sup)
		mapphone_panel_reset(false);

	mapphone_panel_regulator(false);
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
	&mapphone_hdtv_device,
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

static int mapphone_dt_get_tda19989_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;

	PANELDBG("dt_get_tda19989_info()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_PATH_DISPLAY2);
	if (panel_node == NULL)
		return -ENODEV;

	panel_prop = of_get_property(panel_node, "gpio_pwr_en", NULL);
	if (panel_prop != NULL)
		mapphone_tda19989_data.pwr_en_gpio = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "gpio_int", NULL);
	if (panel_prop != NULL)
		mapphone_tda19989_data.int_gpio = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "cec_reg_name", NULL);
	if (panel_prop != NULL) {
		strncpy(mapphone_tda19989_data.cec_reg_name,
				(char *)panel_prop,
				(TDA19989_CEC_REGULATOR_NAME_SIZE - 1));
		mapphone_tda19989_data.cec_reg_name \
			[TDA19989_CEC_REGULATOR_NAME_SIZE - 1] = '\0';
	}

	return 0;
}

static int mapphone_dt_get_hdtv_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;

	PANELDBG("dt_get_hdtv_info()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_PATH_DISPLAY2);
	if (panel_node == NULL)
		return -ENODEV;

	panel_prop = of_get_property(panel_node, "gpio_mux_en", NULL);
	if (panel_prop != NULL)
		mapphone_hdtv_mux_en_gpio = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "gpio_mux_select", NULL);
	if (panel_prop != NULL)
		mapphone_hdtv_mux_sel_gpio = *(u32 *)panel_prop;

	return 0;
}

static int mapphone_dt_get_feature_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;

	PANELDBG("dt_get_feature_info()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (panel_node == NULL)
		return -ENODEV;

	panel_prop = of_get_property(panel_node, "feature_hdmi", NULL);
	if (panel_prop != NULL)
		mapphone_feature_hdmi = *(u8 *)panel_prop;

	return 0;
}

static int __init mapphone_dt_panel_init(void)
{
	int ret = 0;

	PANELDBG("dt_panel_init\n");

	if (mapphone_panel_device_read_dt == false) {
		if (mapphone_dt_get_feature_info() != 0) {
			printk(KERN_ERR "failed to parse feature info \n");
			ret = -ENODEV;
		} else if (mapphone_dt_get_dsi_panel_info() != 0) {
			printk(KERN_ERR "failed to parse DSI panel info \n");
			ret = -ENODEV;
		} else if ((mapphone_lcd_device.phy.dsi.xfer_mode ==
						OMAP_DSI_XFER_VIDEO_MODE) &&
				(mapphone_dt_get_dsi_vm_info() != 0)) {
			printk(KERN_ERR "failed to parse DSI VM info \n");
			ret = -ENODEV;
		} else if (mapphone_feature_hdmi &&
				mapphone_dt_get_tda19989_info() != 0) {
			printk(KERN_ERR "failed to parse TDA19989 info \n");
			ret = -ENODEV;
		} else if (mapphone_feature_hdmi &&
				mapphone_dt_get_hdtv_info() != 0) {
			printk(KERN_ERR "failed to parse hdtv info \n");
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

#if 0
		PANELDBG(" DT: width= %d height= %d\n",
			dt_panel_timings.x_res, dt_panel_timings.y_res);

		PANELDBG(" DT: dsi1_pll_fclk= %d dsi2_pll_fclk= %d\n",
			dt_panel_timings.dsi1_pll_fclk,
			dt_panel_timings.dsi2_pll_fclk);

		PANELDBG(" DT: hfp=%d hsw=%d hbp=%d vfp=%d vsw=%d vbp=%d\n",
			dt_panel_timings.hfp, dt_panel_timings.hsw,
			dt_panel_timings.hbp, dt_panel_timings.vfp,
			dt_panel_timings.vsw, dt_panel_timings.vbp);
#endif
}

static int mapphone_panel_enable_hdtv(struct omap_dss_device *dssdev)
{
	int i;
	int offset;

	PANELDBG("mapphone_panel_enable_hdtv\n");

	offset = 0xDC;
	for (i = 0; i < 6; i++) {
		mapphone_hdmi_stored_mux[i] = omap_ctrl_readw(offset);
		omap_ctrl_writew(MAPPHONE_HDMI_DSS_0_5_MUX, offset);
		offset += 2;
	}

	gpio_set_value(mapphone_hdtv_mux_sel_gpio, 1);

	return 0;
}

static void mapphone_panel_disable_hdtv(struct omap_dss_device *dssdev)
{
	int i;
	int offset;

	PANELDBG("mapphone_panel_disable_hdtv\n");

	offset = 0xDC;
	for (i = 0; i < 6; i++) {
		omap_ctrl_writew(mapphone_hdmi_stored_mux[i], offset);
		offset += 2;
	}

	gpio_set_value(mapphone_hdtv_mux_sel_gpio, 0);
}

static struct dispsw_mr_support mapphone_dispsw_hdtv_1_60Hz = {
	.dev_name = "hdtv",
	.res_name = "hdtv_1_60hz",
	.dev_timing = {
		.x_res	= 640,
		.y_res	= 480,
		.pixel_clock = 25200,
		.hsw	= 96,
		.hfp	= 16,
		.hbp	= 48,
		.vsw	= 2,
		.vfp	= 10,
		.vbp	= 33,
	},
	.panel_config = (OMAP_DSS_LCD_TFT|OMAP_DSS_LCD_IVS|OMAP_DSS_LCD_IHS),
};

static struct dispsw_mr_support mapphone_dispsw_hdtv_2_3_60Hz = {
	.dev_name = "hdtv",
	.res_name = "hdtv_2_3_60hz",
	.dev_timing = {
		.x_res	= 720,
		.y_res	= 480,
		.pixel_clock = 27027,
		.hsw	= 62,
		.hfp	= 16,
		.hbp	= 60,
		.vsw	= 6,
		.vfp	= 9,
		.vbp	= 30,
	},
	.panel_config = (OMAP_DSS_LCD_TFT|OMAP_DSS_LCD_IVS|OMAP_DSS_LCD_IHS),
};

static struct dispsw_mr_support mapphone_dispsw_hdtv_4_60Hz = {
	.dev_name = "hdtv",
	.res_name = "hdtv_4_60hz",
	.dev_timing = {
		.x_res	= 1280,
		.y_res	= 720,
		.pixel_clock = 74250,
		.hsw	= 40,
		.hfp	= 110,
		.hbp	= 220,
		.vsw	= 5,
		.vfp	= 5,
		.vbp	= 20,
	},
	.panel_config = OMAP_DSS_LCD_TFT,
};

static struct dispsw_mr_support mapphone_dispsw_hdtv_17_18_50Hz = {
	.dev_name = "hdtv",
	.res_name = "hdtv_17_18_50hz",
	.dev_timing = {
		.x_res	= 720,
		.y_res	= 576,
		.pixel_clock = 27000,
		.hsw	= 64,
		.hfp	= 12,
		.hbp	= 68,
		.vsw	= 5,
		.vfp	= 5,
		.vbp	= 39,
	},
	.panel_config = (OMAP_DSS_LCD_TFT|OMAP_DSS_LCD_IVS|OMAP_DSS_LCD_IHS),
};

static struct dispsw_mr_support mapphone_dispsw_hdtv_19_50Hz = {
	.dev_name = "hdtv",
	.res_name = "hdtv_19_50hz",
	.dev_timing = {
		.x_res	= 1280,
		.y_res	= 720,
		.pixel_clock = 74250,
		.hsw	= 40,
		.hfp	= 440,
		.hbp	= 220,
		.vsw	= 5,
		.vfp	= 5,
		.vbp	= 20,
	},
	.panel_config = OMAP_DSS_LCD_TFT,
};

static struct dispsw_mr_support mapphone_dispsw_hdtv_21_22_50Hz = {
	.dev_name = "hdtv",
	.res_name = "hdtv_21_22_50hz",
	.dev_timing = {
		.x_res	= 720,
		.y_res	= 576,
		.pixel_clock = 13500,
		.hsw	= 63,
		.hfp	= 12,
		.hbp	= 69,
		.vsw	= 2,
		.vfp	= 3,
		.vbp	= 19,
	},
	.panel_config = (OMAP_DSS_LCD_TFT|OMAP_DSS_LCD_IVS|OMAP_DSS_LCD_IHS),
};

static struct dispsw_mr_support *mapphone_dispsw_resolutions[] = {
	&mapphone_dispsw_hdtv_1_60Hz,
	&mapphone_dispsw_hdtv_2_3_60Hz,
	&mapphone_dispsw_hdtv_4_60Hz,
	&mapphone_dispsw_hdtv_17_18_50Hz,
	&mapphone_dispsw_hdtv_19_50Hz,
	&mapphone_dispsw_hdtv_21_22_50Hz,
};

static struct dispsw_board_info mapphone_dispsw_data = {
	.num_resolutions = ARRAY_SIZE(mapphone_dispsw_resolutions),
	.resolutions = mapphone_dispsw_resolutions,
};

static struct platform_device mapphone_dispsw_device = {
	.name = "dispsw",
	.id = -1,
	.dev = {
		.platform_data = &mapphone_dispsw_data,
	},
};

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
		goto error;
	}

	first_boot = true;

	gpio_direction_output(mapphone_lcd_device.reset_gpio, 1);

	if (mapphone_feature_hdmi) {
		ret = gpio_request(mapphone_hdtv_mux_en_gpio,
							"HDMI-mux-enable");
		if (ret) {
			printk(KERN_ERR "Failed hdtv mux en gpio request\n");
			goto failed_mux_en;
		}
		gpio_direction_output(mapphone_hdtv_mux_en_gpio, 0);
		gpio_set_value(mapphone_hdtv_mux_en_gpio, 0);

		ret = gpio_request(mapphone_hdtv_mux_sel_gpio,
							"HDMI-mux-select");
		if (ret) {
			printk(KERN_ERR "failed hdtv mux sel gpio request\n");
			goto failed_mux_sel;
		}
		gpio_direction_output(mapphone_hdtv_mux_sel_gpio, 0);
		gpio_set_value(mapphone_hdtv_mux_sel_gpio, 0);

		platform_device_register(&mapphone_dispsw_device);
		platform_device_register(&mapphone_tda19989_device);
	} else {
		/* Remove HDTV from the DSS device list */
		mapphone_dss_data.num_devices--;
	}

	platform_device_register(&omap_panel_device);
	platform_device_register(&mapphone_dss_device);

	return;

failed_mux_sel:
	gpio_free(mapphone_hdtv_mux_en_gpio);
failed_mux_en:
	gpio_free(mapphone_lcd_device.reset_gpio);
error:
	return;
}

