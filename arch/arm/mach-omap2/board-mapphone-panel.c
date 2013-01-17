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
#include <linux/err.h>
#include <linux/omapfb.h>

#include <video/omapdss.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/consumer.h>

#include <plat/gpio.h>
#include <plat/omap_hwmod.h>
#include <plat/vram.h>
#include <video/panel.h>
#include "control.h"

#include "dt_path.h"
#include <asm/prom.h>

#include <video/omap-panel-mapphone-dsi.h>

#include "pm.h"
/*#define DISABLED_FOR_BRINGUP*/
/*#define DEBUG*/
#ifdef DEBUG
static unsigned int board_panel_debug;
#define PANELDBG(format, ...) \
	if (board_panel_debug) \
			printk(KERN_DEBUG "board_panel: " format, \
					## __VA_ARGS__)
#else /* DEBUG */
#define PANELDBG(format, ...)
#endif

#define PANELERR(format, ...) \
	printk(KERN_ERR "board_panel ERR: " format, ## __VA_ARGS__)

#define PANELWARN(format, ...) \
	printk(KERN_WARNING "board_panel WARNING: " format, ## __VA_ARGS__)

#define PANELINFO(format, ...) \
	printk(KERN_INFO "board_panel: " format, ## __VA_ARGS__)

#define HDMI_CONTROL_I2C_1_REG          (0x4A100624)
#define HDMI_CONTROL_I2C_1_DDC_PU_DIS   (0x11000000)

static bool mapphone_panel_device_read_dt; /* This is by default false */

static bool first_boot = true;

/* This must be match in the DT */
enum omap_dss_device_disp_intf {
	OMAP_DSS_DISP_INTF_RGB16		= 1,
	OMAP_DSS_DISP_INTF_RGB24		= 2,
	OMAP_DSS_DISP_INTF_MIPI_L4_CM		= 3,
	OMAP_DSS_DISP_INTF_MIPI_VP_CM		= 4,
	OMAP_DSS_DISP_INTF_MIPI_L4_VM		= 5,
	OMAP_DSS_DISP_INTF_MIPI_VP_VM		= 6,
	OMAP_DSS_DISP_INTF_MIPI_LVDS_VP_VM	= 7
};

/* these enum must be matched with MOT DT */
enum omap_dss_device_disp_pxl_fmt {
	OMAP_DSS_DISP_PXL_FMT_RGB565	= 1,
	OMAP_DSS_DISP_PXL_FMT_RGB888	= 5
};

static int mapphone_panel_enable(struct omap_dss_device *dssdev);
static void mapphone_panel_disable(struct omap_dss_device *dssdev);

static struct omap_video_timings mapphone_panel_timings = {
	.x_res          = 480,
	.y_res          = 854,
	/*.pixel_clock  = 25000,*/
	.hfp            = 0,
	.hsw            = 2,
	.hbp            = 2,
	.vfp            = 0,
	.vsw            = 1,
};

static struct mapphone_dsi_panel_data mapphone_panel_data = {
	.name			= "mapphone",
	.reset_gpio		= 0,
	.rst_delay_after_pwr	= 10,
	/*
	 * Change default delay of RESET HIGH to 1st cmd from min 10 to 15ms.
	 * To account for temperature variation, propagation
	 * delays due to flex and railing variation, margin is needed
	 * according to HW team request, 2ms is proposed based on engineering.
	 * 5ms is choosed.
	 */
	.rst_delay_after_high	= 15,
	.use_ext_te		= false,
	.use_esd_check		= true,
	.set_backlight		= NULL,
	.te_support		= true,
#ifdef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	.te_scan_line		= 0x300,
#else
	.te_scan_line		= 300,
#endif
	.te_type		= OMAP_DSI_TE_MIPI_PHY,
	.cmoste_wr		= false,
	.ftr_support.som	= false,
};

static struct omap_dss_device mapphone_lcd_device = {
	.name = "lcd",
	.driver_name = "mapphone-panel",
	.type = OMAP_DISPLAY_TYPE_DSI,
	.data                   = &mapphone_panel_data,
	.phy.dsi.clk_lane = 1,
	.phy.dsi.clk_pol = 0,
	.phy.dsi.data1_lane = 2,
	.phy.dsi.data1_pol = 0,
	.phy.dsi.data2_lane = 3,
	.phy.dsi.data2_pol = 0,
	.clocks.dsi.regn = 13,
	.clocks.dsi.regm = 170,
	.clocks.dsi.regm_dispc = 5,
	.clocks.dsi.regm_dsi = 5,
	.clocks.dsi.dsi_fclk_src = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DSI,
	.clocks.dispc.channel.lck_div = 1,
	.clocks.dispc.channel.pck_div = 4,
	.clocks.dispc.channel.lcd_clk_src =
				OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
	.clocks.dispc.dispc_fclk_src = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
	.clocks.dsi.lp_clk_div = 7,
	.phy.dsi.type = OMAP_DSS_DSI_TYPE_CMD_MODE,
	.platform_enable = mapphone_panel_enable,
	.platform_disable = mapphone_panel_disable,
};

/* It must be matched with device_tree,
* AUO = 001a,
* AUO_1007_wxga_lvds_1280_800 = 0004
*/
#define MOT_DISP_LVDS_MIPI_VM_1007_1280_800	 0x001a0004

static int mapphone_displ_lvds_cabc_en = -1;
static int mapphone_displ_color_en = -1;
static int mapphone_displ_lcd_bl_pwm = -1;
static int mapphone_displ_lvds_wp_g = -1;
static int mapphone_displ_lvds_wp_e = -1;

/* dss powerdomain */
static struct powerdomain *dss_pwrdm;

static int  mapphone_feature_hdmi;
static int  mapphone_hdmi_5v_enable;       /* 0 by default */
static int  mapphone_hdmi_platform_hpd_en; /* 0 by default */
static int  mapphone_hdmi_5v_force_off;    /* 0 by default */
#define HDMI_DAC_REGULATOR_NAME_SIZE  (32)
static char mapphone_hdmi_dac_reg_name[HDMI_DAC_REGULATOR_NAME_SIZE + 1];
struct regulator *mapphone_hdmi_dac_reg;
struct regulator *mapphone_hdmi_5v_reg;

static int  mapphone_panel_hdmi_5v_enable(void);
static int  mapphone_panel_hdmi_5v_disable(void);
static int  mapphone_panel_enable_hdtv(struct omap_dss_device *dssdev);
static void mapphone_panel_disable_hdtv(struct omap_dss_device *dssdev);
static int  mapphone_panel_enable_hpd_hdtv(struct omap_dss_device *dssdev);
static void mapphone_panel_disable_hpd_hdtv(struct omap_dss_device *dssdev);
static int  mapphone_panel_hdtv_test(struct omap_dss_device *dssdev, int level);

static struct omap_dss_device mapphone_hdtv_device = {
	.name                  = "hdmi",
	.driver_name           = "hdmi_panel",
	.type                  = OMAP_DISPLAY_TYPE_HDMI,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 10,
			.regm2	= 1,
		},
	},
	.hpd_gpio = 63,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
	.platform_enable       = mapphone_panel_enable_hdtv,
	.platform_disable      = mapphone_panel_disable_hdtv,

#ifdef CONFIG_DEBUG_FS
	/* Used as a simple engineering test interface */
	.set_backlight         = mapphone_panel_hdtv_test,
#endif
};

struct regulator_consumer_supply hdmi_hpd_consumers =
	REGULATOR_SUPPLY("hdmi_5V_en", NULL);

struct regulator_init_data hpd_en_initdata = {
	.consumer_supplies = &hdmi_hpd_consumers,
	.num_consumer_supplies = 1,
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static struct fixed_voltage_config hpd_en_config = {
	.supply_name		= "hdmi_5v_en",
	.microvolts		= 5000000,
	.gpio			= 59,
	.startup_delay          = 10,
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &hpd_en_initdata,
};

static struct platform_device hpd_en_device = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &hpd_en_config,
	},
};

static struct platform_device *hdmi_regulator_devices[] = {
	&hpd_en_device,
};

static int  mapphone_panel_hdmi_5v_enable(void)
{
	int rc = -1;

	if (!mapphone_hdmi_5v_enable && !mapphone_hdmi_5v_force_off) {
		if (!mapphone_hdmi_5v_reg) {
			mapphone_hdmi_5v_reg =
				regulator_get(NULL, "hdmi_5V_en");
			if (!mapphone_hdmi_5v_reg) {
				regulator_put(mapphone_hdmi_5v_reg);
				return rc;
			}
		}
		rc = regulator_enable(mapphone_hdmi_5v_reg);
		if (rc != 0) {
			PANELERR("Failed HDMI_5V regulator_enable (%d)\n", rc);
		} else {
			PANELDBG("Enabled HDMI_5V regulator\n");
			mapphone_hdmi_5v_enable = 1;
		}
	}

	return rc;
}

static int mapphone_panel_hdmi_5v_disable(void)
{
	int rc = 0;

	if (mapphone_hdmi_5v_enable) {
		rc = regulator_disable(mapphone_hdmi_5v_reg);
		if (rc != 0) {
			PANELERR("Failed HDMI_5V regulator_disable (%d)\n", rc);
		} else {
			PANELDBG("Disabled  HDMI_5V regulator\n");
			mapphone_hdmi_5v_enable = 0;
		}
	}
	return rc;
}

static void mapphone_panel_reset(bool enable)
{
	if (enable) { /* enable panel */
		if (!first_boot) {
			gpio_set_value(mapphone_panel_data.reset_gpio, 0);
			msleep(mapphone_panel_data.rst_delay_after_pwr);

			gpio_set_value(mapphone_panel_data.reset_gpio, 1);
			msleep(mapphone_panel_data.rst_delay_after_high);
		} else
			first_boot = false;
	} else { /* disable panel */
		gpio_set_value(mapphone_panel_data.reset_gpio, 0);
		msleep(1);
	}
}

static int mapphone_panel_regulator_enable(void)
{
	int i;
	struct mapphone_dsi_panel_pwr_supply *supply;
	struct mapphone_dsi_panel_data *panel_data =
		(struct mapphone_dsi_panel_data *)mapphone_lcd_device.data;

	if (!panel_data->num_pwr_supply)
		PANELWARN("No display regulator is requested\n");

	if (first_boot) {
		for (i = 0; i < panel_data->num_pwr_supply; i++) {
			supply = &(panel_data->disp_vol_supply[i]);

			if (supply->name[0] == '\0') {
				PANELDBG("Display power supply[%d] = none\n",
					i);
				supply->reg_handle = NULL;
				continue;
			}

			PANELDBG("Display power supply[%d] = %s\n",
				i, supply->name);

			supply->reg_handle = regulator_get(NULL, supply->name);
			if (IS_ERR(supply->reg_handle)) {
				PANELERR("fail to get reg for displ.\n");
				return PTR_ERR(supply->reg_handle);
			}
		}
	}

	for (i = 0; i < panel_data->num_pwr_supply; i++) {
		supply = &(panel_data->disp_vol_supply[i]);
		if (supply->reg_handle) {
			PANELDBG("Display power supply[%d], regulator on\n", i);
			regulator_enable(supply->reg_handle);
		}

		if (supply->en_gpio != 0) {
			PANELDBG("Display power supply[%d], gpio %d set"
				"to %d \n", i, supply->en_gpio,
				supply->en_gpio_value);
			gpio_set_value(supply->en_gpio, supply->en_gpio_value);
		}
	}

	msleep(10);

	return 0;
}

static void mapphone_panel_regulator_disable(void)
{
	int i;
	struct mapphone_dsi_panel_pwr_supply *supply;
	struct mapphone_dsi_panel_data *panel_data =
		(struct mapphone_dsi_panel_data *)mapphone_lcd_device.data;

	for (i = 0; i < panel_data->num_pwr_supply; i++) {
		supply = &(panel_data->disp_vol_supply[i]);
		if (supply->en_gpio != 0) {
			PANELDBG("Display power supply[%d], "
				"gpio %d set to %d\n",
				i, supply->en_gpio, !supply->en_gpio_value);
			gpio_set_value(supply->en_gpio,
				!supply->en_gpio_value);
		}

		if (supply->reg_handle) {
			PANELDBG("Display power supply[%d], "
				"regulator off\n", i);
			regulator_disable(supply->reg_handle);
		}
	}
}

static void mapphone_panel_lvds_mipi_vm_1007_1280_800_enable(bool enable)
{
	if (mapphone_displ_lvds_wp_g >= 0)
		gpio_set_value(mapphone_displ_lvds_wp_g, enable ? 1 : 0);
	if (mapphone_displ_lvds_wp_e >= 0)
		gpio_set_value(mapphone_displ_lvds_wp_e, enable ? 1 : 0);
	if (mapphone_displ_lvds_cabc_en >= 0)
		gpio_set_value(mapphone_displ_lvds_cabc_en, enable ? 1 : 0);
	if (mapphone_displ_color_en >= 0)
		gpio_set_value(mapphone_displ_color_en,  enable ? 1 : 0);
	if (mapphone_displ_lcd_bl_pwm >= 0)
		gpio_set_value(mapphone_displ_lcd_bl_pwm,  enable ? 0 : 1);
}

static int mapphone_panel_enable(struct omap_dss_device *dssdev)
{
	int ret;

	/* change the DSS power state to INACTIVE with LCD on */
	if (dss_pwrdm)
		omap_set_pwrdm_state(dss_pwrdm, PWRDM_POWER_INACTIVE);
	/*
	 * TODO:
	 * 1. mapphone_panel_regulator_enable() and mapphone_panel_reset() are
	 * sharing the first_boot flag, and it MUST maintain this order.
	 * 2. there is a first_boot flag in panel-mapphone.c, they need to be
	 * to be consolidated later.
	 * 3. mapphone_panel_reset() will reset panel base on the first_boot
	 * flasg, but there is a macro CONFIG_FB_OMAP_BOOTLOADER_INIT will be
	 * used to determine if the panel need to be reset at the first_boot
	 */
	ret = mapphone_panel_regulator_enable();
	if (!ret) {
		if (dssdev->panel.panel_id ==
			MOT_DISP_LVDS_MIPI_VM_1007_1280_800) {
			/* give display gpio level switchers time to power-up */
			msleep(5);
			mapphone_panel_lvds_mipi_vm_1007_1280_800_enable(true);
		}
		mapphone_panel_reset(true);
	}

	return ret;
}

static void mapphone_panel_disable(struct omap_dss_device *dssdev)
{
	if (!dssdev->phy.dsi.d2l_use_ulps)
		mapphone_panel_reset(false);

	if (dssdev->panel.panel_id == MOT_DISP_LVDS_MIPI_VM_1007_1280_800)
		mapphone_panel_lvds_mipi_vm_1007_1280_800_enable(false);

	mapphone_panel_regulator_disable();
	/* change the DSS power state to RET with LCD off */
	if (dss_pwrdm)
		omap_set_pwrdm_state(dss_pwrdm, PWRDM_POWER_RET);
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
	/*.feature_fb1_to_vid2 = 0,*/
	/*.num_fbs = 1,*/
};

static struct omap_dss_device *mapphone_dss_devices[] = {
	&mapphone_lcd_device,
#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	&mapphone_hdtv_device,
#endif
};

static struct omap_dss_board_info mapphone_dss_data = {
	.num_devices = ARRAY_SIZE(mapphone_dss_devices),
	.devices = mapphone_dss_devices,
	.default_device = &mapphone_lcd_device,
};

/*
 * This api will parse the display power supply name and power supply enable
 * GPIO from device tree. The "num_disp_pwr_sup" in the device_tree tells us
 * how many power supplies we need for the display
 *
 * If "num_disp_pwr_sup" is not defined then number of power supply will be '1"
 *
 * Each power supply can have an associated regulator, enable GPIO line, and
 * enable GPIO value (high vs low).  For the regulator, the value is the name
 * of the regulator.  If the name is an empty string, a regulator is not used.
 * For the enable GPIO line, if the value is 0, there is no line.
 *
 */
static int mapphone_dt_get_panel_power_supply(struct device_node *panel_node)
{
	int len = 0, i, r = 0;
	const void *panel_prop;
	char dt_node_pwr_name[MAPPHONE_DSI_MAX_NAME_SIZE];
	char dt_node_pwr_en_name[MAPPHONE_DSI_MAX_NAME_SIZE];
	char dt_node_pwr_en_val_name[MAPPHONE_DSI_MAX_NAME_SIZE];
	struct mapphone_dsi_panel_data *panel_data =
		(struct mapphone_dsi_panel_data *)mapphone_lcd_device.data;
	struct mapphone_dsi_panel_pwr_supply *supply = NULL;

	panel_prop = of_get_property(panel_node, "num_disp_pwr_sup", NULL);
	if (panel_prop != NULL) {
		panel_data->num_pwr_supply = *(u8 *)panel_prop;
		if (panel_data->num_pwr_supply > MAPPHONE_DSI_MAX_PWR_SUPPLY) {
			PANELERR("invalid num_disp_pwr_sup=%d\n",
				panel_data->num_pwr_supply);
			panel_data->num_pwr_supply = 0;
			r = -EINVAL;
			goto end;
		}
	} else
		panel_data->num_pwr_supply = 1;

	for (i = 0; i < panel_data->num_pwr_supply; i++) {
		supply = &(panel_data->disp_vol_supply[i]);

		if (i == 0) {
			/* By default, using sw5 */
			strncpy(supply->name, "vhvio",
				MAPPHONE_DSI_MAX_NAME_SIZE - 1);
			supply->name[MAPPHONE_DSI_MAX_NAME_SIZE - 1] = '\0';

			/* Default at 96, active high */
			supply->en_gpio = 96;
			supply->en_gpio_value = 1;

			snprintf(dt_node_pwr_name, MAPPHONE_DSI_MAX_NAME_SIZE,
					"displ_pwr_sup");
			snprintf(dt_node_pwr_en_name,
					MAPPHONE_DSI_MAX_NAME_SIZE,
					"displ_pwr_sup_en");
			snprintf(dt_node_pwr_en_val_name,
				MAPPHONE_DSI_MAX_NAME_SIZE,
				"displ_pwr_sup_en_val");
		} else {
			supply->en_gpio = 0;
			supply->en_gpio_value = 1;
			supply->name[0] = '\0';
			snprintf(dt_node_pwr_name, MAPPHONE_DSI_MAX_NAME_SIZE,
						"displ_pwr_sup_%d", i);
			snprintf(dt_node_pwr_en_name,
					MAPPHONE_DSI_MAX_NAME_SIZE,
					"displ_pwr_sup_en_%d", i);
			snprintf(dt_node_pwr_en_val_name,
				MAPPHONE_DSI_MAX_NAME_SIZE,
				"displ_pwr_sup_en_val_%d", i);
		}

		panel_prop = of_get_property(panel_node, dt_node_pwr_name,
									&len);
		/* If regulator name property is present, but len == 0, set
		   name to empty string.  If property is not present at all,
		   leave as default value defined above */
		if (panel_prop != NULL) {
			if (len) {
				printk("dt_node_pwr_name exist\n");
				strncpy(supply->name, (char *)panel_prop,
					MAPPHONE_DSI_MAX_NAME_SIZE - 1);
				supply->name[MAPPHONE_DSI_MAX_NAME_SIZE - 1] =
					'\0';
			} else
				printk("dt_node_pwr_name not exist\n");
				supply->name[0] = '\0';
		}

		panel_prop = of_get_property(panel_node, dt_node_pwr_en_name,
						NULL);
		if (panel_prop != NULL)
			supply->en_gpio = *(u32 *)panel_prop;
		panel_prop = of_get_property(panel_node,
					dt_node_pwr_en_val_name, NULL);
		if (panel_prop != NULL)
			supply->en_gpio_value = *(u8 *)panel_prop;
	}
		
end:
	return r;
}


static int mapphone_dt_get_dsi_panel_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;
	int disp_intf, r = 0;

	PANELDBG("dt_get_dsi_panel_info()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node == NULL) {
		r = -ENODEV;
		goto err;
	}

	/* Retrieve the panel information */
	panel_prop = of_get_property(panel_node, "dsi_clk_lane", NULL);
	if (panel_prop != NULL) {
		mapphone_lcd_device.phy.dsi.clk_lane = *(u8 *)panel_prop;
	} else {
		printk("Can't get dsi_clk_lane\n");
	}

	panel_prop = of_get_property(panel_node, "dsi_clk_pol", NULL);
	if (panel_prop != NULL) {
		mapphone_lcd_device.phy.dsi.clk_pol = *(u8 *)panel_prop;
	} else {
		printk("Can't get dsi_clk_pol\n");
	}

	panel_prop = of_get_property(panel_node, "dsi_data1_lane", NULL);
	if (panel_prop != NULL) {
		mapphone_lcd_device.phy.dsi.data1_lane = *(u8 *)panel_prop;
	} else {
		printk("Can't get dsi_data1_lane\n");
	}

	panel_prop = of_get_property(panel_node, "dsi_data1_pol", NULL);
	if (panel_prop != NULL) {
		mapphone_lcd_device.phy.dsi.data1_pol = *(u8 *)panel_prop;
	} else {
		printk("Can't get dsi_data1_pol\n");
	}

	panel_prop = of_get_property(panel_node, "dsi_data2_lane", NULL);
	if (panel_prop != NULL) {
		mapphone_lcd_device.phy.dsi.data2_lane = *(u8 *)panel_prop;
	} else {
		printk("Can't get dsi_data2_lane\n");
	}

	panel_prop = of_get_property(panel_node, "dsi_data2_pol", NULL);
	if (panel_prop != NULL) {
		mapphone_lcd_device.phy.dsi.data2_pol = *(u8 *)panel_prop;
	} else {
		printk("Can't get dsi_data2_pol\n");
	}

	panel_prop = of_get_property(panel_node, "dsi_data3_lane", NULL);
	if (panel_prop != NULL) {
		mapphone_lcd_device.phy.dsi.data3_lane = *(u8 *)panel_prop;
	} else {
		printk("Can't get dsi_data3_lane\n");
	}

	panel_prop = of_get_property(panel_node, "dsi_data3_pol", NULL);
	if (panel_prop != NULL) {
		mapphone_lcd_device.phy.dsi.data3_pol = *(u8 *)panel_prop;
	} else {
		printk("Can't get dsi_data3_pol\n");
	}

	panel_prop = of_get_property(panel_node, "dsi_data4_lane", NULL);
	if (panel_prop != NULL) {
		mapphone_lcd_device.phy.dsi.data4_lane = *(u8 *)panel_prop;
	} else {
		printk("Can't get dsi_data4_lane\n");
	}

	panel_prop = of_get_property(panel_node, "dsi_data4_pol", NULL);
	if (panel_prop != NULL) {
		mapphone_lcd_device.phy.dsi.data4_pol = *(u8 *)panel_prop;
	} else {
		printk("Can't get dsi_data4_pol\n");
	}

	panel_prop = of_get_property(panel_node, "gpio_reset", NULL);
	if (panel_prop != NULL) {
		mapphone_panel_data.reset_gpio = *(u32 *)panel_prop;
	} else {
		printk("Can't get gpio_reset\n");
	}

	panel_prop = of_get_property(panel_node, "rst_delay_after_pwr", NULL);
	if (panel_prop != NULL)
		mapphone_panel_data.rst_delay_after_pwr = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "rst_delay_after_high", NULL);
	if (panel_prop != NULL)
		mapphone_panel_data.rst_delay_after_high = *(u32 *)panel_prop;

	r = mapphone_dt_get_panel_power_supply(panel_node);
	if (r)
		goto end;

	panel_prop = of_get_property(panel_node, "disp_cabc_en", NULL);
	if (panel_prop != NULL)
		mapphone_displ_lvds_cabc_en = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "color_en", NULL);
	if (panel_prop != NULL)
		mapphone_displ_color_en = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "lcd_bl_pwm", NULL);
	if (panel_prop != NULL)
		mapphone_displ_lcd_bl_pwm = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "lvds_wp_g", NULL);
	if (panel_prop != NULL)
		mapphone_displ_lvds_wp_g = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "lvds_wp_e", NULL);
	if (panel_prop != NULL)
		mapphone_displ_lvds_wp_e = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "type", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.panel.panel_id = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regn", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.clocks.dsi.regn = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regm", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.clocks.dsi.regm = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regm3", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.clocks.dsi.regm_dispc = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "regm4", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.clocks.dsi.regm_dsi = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "lp_clk_div", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.clocks.dsi.lp_clk_div = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "lck_div", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.clocks.dispc.channel.lck_div =
			 *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "pck_div", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.clocks.dispc.channel.pck_div =
			 *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "d2l_use_ulps", NULL);
	if (panel_prop != NULL)
		mapphone_lcd_device.phy.dsi.d2l_use_ulps = *(u16 *)panel_prop;

	panel_prop = of_get_property(panel_node, "disp_intf", NULL);
	if (panel_prop != NULL) {
		disp_intf = *(u8 *)panel_prop;

		if ((disp_intf == OMAP_DSS_DISP_INTF_MIPI_VP_CM) ||
			(disp_intf == OMAP_DSS_DISP_INTF_MIPI_L4_CM))
			mapphone_lcd_device.phy.dsi.type =
				OMAP_DSS_DSI_TYPE_CMD_MODE;
		else if ((disp_intf == OMAP_DSS_DISP_INTF_MIPI_VP_VM) ||
			(disp_intf == OMAP_DSS_DISP_INTF_MIPI_L4_VM) ||
			(disp_intf == OMAP_DSS_DISP_INTF_MIPI_LVDS_VP_VM)) {
			mapphone_lcd_device.phy.dsi.type =
				OMAP_DSS_DSI_TYPE_VIDEO_MODE;
#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
			mapphone_lcd_device.skip_init = true;
#else
			mapphone_lcd_device.skip_init = false;
#endif
		} else {
			PANELERR("Invalid disp_intf in dt = %d\n", disp_intf);
			r = -ENODEV;
		}
	}

end:
	of_node_put(panel_node);
err:
	return r;
}

static int mapphone_dt_get_panel_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;
	int panel_pixel_fmt;
	int pixel_size = 24;
	struct mapphone_dsi_panel_data *panel_data =
		(struct mapphone_dsi_panel_data *)mapphone_lcd_device.data;

	PANELDBG("mapphone_dt_get_panel_info\n");

	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node != NULL) {
		/* Retrieve the panel DSI timing */
		panel_prop = of_get_property(panel_node, "width", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.x_res = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node, "height", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.y_res = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hfp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hfp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hsw", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hsw = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hbp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hbp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vfp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vfp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vsw", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vsw = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vbp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vbp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_pixel_clk", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.pixel_clock = *(u32 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"phy_width_mm", NULL);
		if (panel_prop != NULL) {
			mapphone_lcd_device.panel.width_in_mm =
				*(u16 *)panel_prop;
			mapphone_lcd_device.panel.width_in_um =
				*(u16 *)panel_prop * 1000;
		}

		panel_prop = of_get_property(panel_node,
						"phy_height_mm", NULL);
		if (panel_prop != NULL) {
			mapphone_lcd_device.panel.height_in_mm =
				*(u16 *)panel_prop;
			mapphone_lcd_device.panel.height_in_um =
				*(u16 *)panel_prop * 1000;
		}

		panel_prop = of_get_property(panel_node,
						"use_esd_check", NULL);
		if (panel_prop != NULL)
			panel_data->use_esd_check = *(u32 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"te_support", NULL);
		if (panel_prop != NULL)
			panel_data->te_support = *(u32 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"cmoste_wr", NULL);
		if (panel_prop != NULL)
			panel_data->cmoste_wr = *(u32 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"te_scan_line", NULL);
		if (panel_prop != NULL)
			panel_data->te_scan_line =  *(u32 *)panel_prop;

		panel_prop = of_get_property(panel_node, "te_type", NULL);
		if (panel_prop != NULL)
			panel_data->te_type = *(enum omap_dsi_te_type *)
				panel_prop;

		panel_prop = of_get_property(panel_node, "pixel_fmt", NULL);
		if (panel_prop != NULL) {
			panel_pixel_fmt = *(u32 *)panel_prop;
			if (panel_pixel_fmt == OMAP_DSS_DISP_PXL_FMT_RGB888)
				pixel_size = 24;
			else if (panel_pixel_fmt ==
					OMAP_DSS_DISP_PXL_FMT_RGB565)
				pixel_size = 16;
			else {
				PANELERR(" Invalid panel_pxl_fmt=%d",
							panel_pixel_fmt);
				return -ENODEV;
			}
		}

		of_node_put(panel_node);

		mapphone_lcd_device.ctrl.pixel_size  =  pixel_size;
		mapphone_lcd_device.panel.timings  =  mapphone_panel_timings;

	}

	return panel_node ? 0 : -ENODEV;

}

static int mapphone_dt_get_panel_feature(void)
{
	struct device_node *panel_node;
	const void *panel_prop;
	struct mapphone_dsi_panel_data *panel_data =
		(struct mapphone_dsi_panel_data *)mapphone_lcd_device.data;
	int r = 0;

	PANELDBG("mapphone_dt_get_panel_feature()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node == NULL) {
		r = -ENODEV;
		goto err;
	}

	/* Retrieve which features are supported */
	panel_prop = of_get_property(panel_node,
				"ftr_support_som", NULL);
	if (panel_prop != NULL)
		panel_data->ftr_support.som = *(bool *)panel_prop;

end:
	of_node_put(panel_node);
err:
	return r;

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

	PANELDBG("DT: phy.dsi.vm_timing: hsa=%d hfp=%d \
		hbp=%d vsa=%d vfp=%d vbp=%d\n",
		mapphone_lcd_device.phy.dsi.vm_timing.hsa,
		mapphone_lcd_device.phy.dsi.vm_timing.hfp,
		mapphone_lcd_device.phy.dsi.vm_timing.hbp,
		mapphone_lcd_device.phy.dsi.vm_timing.vsa,
		mapphone_lcd_device.phy.dsi.vm_timing.vfp,
		mapphone_lcd_device.phy.dsi.vm_timing.vbp);

	return 0;
}

static int mapphone_dt_get_lvds_panel_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;

	struct mapphone_dsi_panel_data *panel_data =
		(struct mapphone_dsi_panel_data *)mapphone_lcd_device.data;

	PANELDBG("mapphone_dt_get_lvds_panel_info\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node == NULL)
		return -ENODEV;

	panel_prop =
		of_get_property(panel_node, "lvds_panel_timing_hbpr", NULL);
	if (panel_prop != NULL)
		panel_data->lvds_panel_timing.hbpr = *(u16 *)panel_prop;

	panel_prop =
		of_get_property(panel_node, "lvds_panel_timing_hpw", NULL);
	if (panel_prop != NULL)
		panel_data->lvds_panel_timing.hpw = *(u16 *)panel_prop;

	panel_prop =
		of_get_property(panel_node, "lvds_panel_timing_hfpr", NULL);
	if (panel_prop != NULL)
		panel_data->lvds_panel_timing.hfpr = *(u16 *)panel_prop;

	panel_prop =
		of_get_property(panel_node, "lvds_panel_timing_vbpr", NULL);
	if (panel_prop != NULL)
		panel_data->lvds_panel_timing.vbpr = *(u16 *)panel_prop;

	panel_prop =
		of_get_property(panel_node, "lvds_panel_timing_vspr", NULL);
	if (panel_prop != NULL)
		panel_data->lvds_panel_timing.vspr = *(u16 *)panel_prop;

	panel_prop =
		of_get_property(panel_node, "lvds_panel_timing_vfpr", NULL);
	if (panel_prop != NULL)
		panel_data->lvds_panel_timing.vfpr = *(u16 *)panel_prop;

	of_node_put(panel_node);

	PANELDBG("DT: lvds_panel_timing: hbpr=%d hpw=%d \
		hfpr=%d vbpr=%d vspr=%d vfpr=%d\n",
		panel_data->lvds_panel_timing.hbpr,
		panel_data->lvds_panel_timing.hpw,
		panel_data->lvds_panel_timing.hfpr,
		panel_data->lvds_panel_timing.vbpr,
		panel_data->lvds_panel_timing.vspr,
		panel_data->lvds_panel_timing.vfpr);

	return 0;
}

static int mapphone_dt_get_hdtv_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;
	struct omap_ovl2mgr_mapping *read_ovl2mgr_mapping = NULL;
	int len = 0, i = 0;

	PANELDBG("dt_get_hdtv_info()\n");

	/* return err if fail to open DT */
	panel_node = of_find_node_by_path(DT_PATH_DISPLAY2);
	if (panel_node == NULL)
		return -ENODEV;

	panel_prop = of_get_property(panel_node, "max_width", NULL);
	if (panel_prop != NULL)
		mapphone_hdtv_device.panel.timings.x_res = *(u32 *)panel_prop;

	panel_prop = of_get_property(panel_node, "max_height", NULL);
	if (panel_prop != NULL)
		mapphone_hdtv_device.panel.timings.y_res = *(u32 *)panel_prop;

	if (mapphone_hdtv_device.panel.timings.x_res > 2048)
		mapphone_hdtv_device.panel.timings.x_res = 1920;

	if (mapphone_hdtv_device.panel.timings.y_res > 2048)
		mapphone_hdtv_device.panel.timings.x_res = 1080;

	PANELDBG("Getting the xres = %u yres = %u\n",
		mapphone_hdtv_device.panel.timings.x_res,
		mapphone_hdtv_device.panel.timings.y_res);

	panel_prop = of_get_property(panel_node, "gpio_pwr_en", NULL);
	if (panel_prop != NULL) {
		hpd_en_config.gpio = *(u32 *)panel_prop;
		platform_add_devices(hdmi_regulator_devices,
		    ARRAY_SIZE(hdmi_regulator_devices));

	} else {
		PANELDBG("Getting the property gpio_pwr_en failed");
	}

	panel_prop = of_get_property(panel_node, "dac_reg_name", NULL);
	if (panel_prop != NULL) {
		strncpy(mapphone_hdmi_dac_reg_name,
				(char *)panel_prop,
				(HDMI_DAC_REGULATOR_NAME_SIZE));
		mapphone_hdmi_dac_reg_name \
			[HDMI_DAC_REGULATOR_NAME_SIZE] = '\0';
	}

	of_node_put(panel_node);
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

static void panel_print_dt(void)
{
	struct mapphone_dsi_panel_data *panel_data =
		(struct mapphone_dsi_panel_data *)mapphone_lcd_device.data;

	PANELINFO("DT: width= %d height= %d\n",
		mapphone_lcd_device.panel.timings.x_res,
		mapphone_lcd_device.panel.timings.y_res);

	PANELINFO("DT: hfp=%d hsw=%d hbp=%d vfp=%d vsw=%d vbp=%d\n",
		mapphone_lcd_device.panel.timings.hfp,
		mapphone_lcd_device.panel.timings.hsw,
		mapphone_lcd_device.panel.timings.hbp,
		mapphone_lcd_device.panel.timings.vfp,
		mapphone_lcd_device.panel.timings.vsw,
		mapphone_lcd_device.panel.timings.vbp);

	PANELINFO("DT: gpio_reset=%d panel_id=0x%lx\n",
		mapphone_panel_data.reset_gpio,
		mapphone_lcd_device.panel.panel_id);

	PANELINFO("DT: phy.dsi: pixel_size=%d te_scan_line=%d\n",
		mapphone_lcd_device.ctrl.pixel_size,
		panel_data->te_scan_line);

	PANELINFO("DT: phy.dsi: data1_lane=%d data1_pol=%d\n",
		mapphone_lcd_device.phy.dsi.data1_lane,
		mapphone_lcd_device.phy.dsi.data1_pol);

	PANELINFO("DT: phy.dsi: data2_lane=%d data2_pol=%d\n",
		mapphone_lcd_device.phy.dsi.data2_lane,
		mapphone_lcd_device.phy.dsi.data2_pol);

	PANELINFO("DT: clocks.dsi : regn=%d regm=%d\n",
		mapphone_lcd_device.clocks.dsi.regn,
		mapphone_lcd_device.clocks.dsi.regm);

	PANELINFO("DT: clocks.dsi : regm3=%d regm4=%d\n",
		mapphone_lcd_device.clocks.dsi.regm_dispc,
		mapphone_lcd_device.clocks.dsi.regm_dsi);

	PANELINFO("DT: clocks.dsi : lp_clk_div=%d, lck_div=%d, pck_div=%d\n",
		mapphone_lcd_device.clocks.dsi.lp_clk_div,
		mapphone_lcd_device.clocks.dispc.channel.lck_div,
		mapphone_lcd_device.clocks.dispc.channel.pck_div);

	PANELINFO("DT: phy.dsi.type=%d\n",
		mapphone_lcd_device.phy.dsi.type);

}

static int __init mapphone_dt_panel_init(void)
{
	int ret = 0;

	PANELDBG("dt_panel_init\n");

	if (mapphone_panel_device_read_dt == false) {
		if (mapphone_dt_get_feature_info() != 0) {
			PANELERR("failed to parse feature info\n");
			ret = -ENODEV;
		} else if (mapphone_dt_get_dsi_panel_info() != 0) {
			PANELERR("failed to parse DSI panel info\n");
			ret = -ENODEV;
		} else if ((mapphone_lcd_device.phy.dsi.type ==
						OMAP_DSS_DSI_TYPE_VIDEO_MODE) &&
				(mapphone_dt_get_dsi_vm_info() != 0)) {
			PANELERR("failed to parse DSI VM info\n");
			ret = -ENODEV;
		} else if ((mapphone_lcd_device.phy.dsi.type ==
						OMAP_DSS_DSI_TYPE_VIDEO_MODE) &&
				(mapphone_lcd_device.panel.panel_id ==
					MOT_DISP_LVDS_MIPI_VM_1007_1280_800) &&
				(mapphone_dt_get_lvds_panel_info() != 0)) {
			PANELERR("failed to parse DSI lvds panel info\n");
			ret = -ENODEV;
		} else if (mapphone_dt_get_panel_info() != 0) {
			PANELERR("failed to parse panel info\n");
			ret = -ENODEV;
		} else if (mapphone_dt_get_panel_feature() != 0) {
			PANELERR("failed to parse panel feature info\n");
			ret = -ENODEV;
#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
		} else if (mapphone_feature_hdmi &&
				mapphone_dt_get_hdtv_info() != 0) {
			PANELERR("failed to parse hdtv info\n");
			ret = -ENODEV;
#endif
		} else {
			mapphone_panel_device_read_dt = true;
		}
	}

	panel_print_dt();
	return ret;
}


static int mapphone_panel_enable_hdtv(struct omap_dss_device *dssdev)
{
	int rc = 0;

	PANELDBG("mapphone_panel_enable_hdtv\n");

	if (!mapphone_hdmi_dac_reg) {
		if (mapphone_hdmi_dac_reg_name[0] == 0) {
			PANELERR("No HDMI regulator defined\n");
			rc = -1;
			goto exit;
		}
		mapphone_hdmi_dac_reg = regulator_get(NULL,
					mapphone_hdmi_dac_reg_name);
		if (IS_ERR(mapphone_hdmi_dac_reg)) {
			PANELERR("Failed HDMI regulator_get\n");
			rc = -1;
			goto exit;
		}
	}

	rc = regulator_enable(mapphone_hdmi_dac_reg);
	if (rc != 0) {
		PANELERR("Failed HDMI regulator_enable (%d)\n", rc);
	} else {
		PANELDBG("Enabled HDMI DAC regulator\n");
		/* Settling time */
		msleep(2);
	}

exit:
	return rc;
}

static void mapphone_panel_disable_hdtv(struct omap_dss_device *dssdev)
{
	PANELDBG("mapphone_panel_disable_hdtv\n");

	if (mapphone_hdmi_dac_reg)
		regulator_disable(mapphone_hdmi_dac_reg);
}

static int  mapphone_panel_enable_hpd_hdtv(struct omap_dss_device *dssdev)
{
	int rc = 0;

	PANELDBG("mapphone_panel_enable_hpd_hdtv\n");

	/* Only enable if not currently enabled */
	if (mapphone_hdmi_platform_hpd_en == 0) {
		mapphone_hdmi_platform_hpd_en = 1;
		rc = mapphone_panel_hdmi_5v_enable();
	}

	return rc;
}

static void mapphone_panel_disable_hpd_hdtv(struct omap_dss_device *dssdev)
{
	PANELDBG("mapphone_panel_disable_hpd_hdtv\n");

	mapphone_hdmi_platform_hpd_en = 0;
	mapphone_panel_hdmi_5v_disable();
}

static int mapphone_panel_hdtv_test(struct omap_dss_device *not_used, int tst)
{
	if (tst == 0) {
		mapphone_hdmi_5v_force_off = 0;
		if (mapphone_hdmi_platform_hpd_en)
			mapphone_panel_hdmi_5v_enable();
	} else if (tst == 1) {
		mapphone_hdmi_5v_force_off = 1;
		if (mapphone_hdmi_platform_hpd_en)
			mapphone_panel_hdmi_5v_disable();
	}
	return 0;
}

static struct platform_device omap_panel_device = {
	.name = "omap-panel",
	.id = -1,
};

static struct platform_device omap_dssmgr_device = {
	.name = "omap-dssmgr",
	.id = -1,
};

static void mapphone_panel_get_fb_info(void)
{
	struct device_node *panel_node;
	const void *panel_prop;

	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node) {
		panel_prop = of_get_property(panel_node, "fb_width", NULL);
		if (panel_prop != NULL)
			mapphone_fb_data.xres_virtual = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node, "fb_height", NULL);
		if (panel_prop != NULL)
			mapphone_fb_data.yres_virtual = *(u16 *)panel_prop;

		of_node_put(panel_node);
	}

#ifdef DISABLED_FOR_BRINGUP

	mapphone_fb_data.feature_fb1_to_vid2 = mapphone_feature_hdmi;

	panel_node = of_find_node_by_path(DT_PATH_DISPLAY2);
	if (panel_node == NULL)
		return;

	panel_prop = of_get_property(panel_node, "num_fbs", NULL);
	if (panel_prop != NULL)
		mapphone_fb_data.num_fbs = *(u32 *)panel_prop;

	of_node_put(panel_node);
#endif
}

void __init mapphone_panel_init(void)
{
	struct mapphone_dsi_panel_pwr_supply *supply;
	struct mapphone_dsi_panel_data *panel_data =
		(struct mapphone_dsi_panel_data *)mapphone_lcd_device.data;
	int ret;
	int i;
	int num_gpio_handled = 0;

	dss_pwrdm = pwrdm_lookup("dss_pwrdm");
	if (!dss_pwrdm)
		pr_info("%s: Not found dss_pwrdm\n", __func__);

	if (mapphone_dt_panel_init())
		PANELINFO(": using non-dt configuration\n");

	mapphone_panel_get_fb_info();
	omapfb_set_platform_data(&mapphone_fb_data);

	ret = gpio_request(mapphone_panel_data.reset_gpio, "display reset");
	if (ret) {
		PANELERR("failed to get display reset gpio\n");
		goto failed_reset;
	}

	gpio_direction_output(mapphone_panel_data.reset_gpio, 1);

	for (i = 0; i < panel_data->num_pwr_supply; i++) {
		supply = &(panel_data->disp_vol_supply[i]);
		if (supply->en_gpio != 0) {
			ret = gpio_request(supply->en_gpio,
							"LCD-pwr_sup_en");
			if (ret) {
				PANELERR("Supply %d, failed to req for "
					"LCD-pwr_sup_en\n", i);
				num_gpio_handled = i;
				goto failed_pwr_supply;
			}

			gpio_direction_output(supply->en_gpio,
					supply->en_gpio_value);
		}
	}
	num_gpio_handled = panel_data->num_pwr_supply;

	if (mapphone_displ_lvds_cabc_en >= 0) {
		ret = gpio_request(mapphone_displ_lvds_cabc_en,
					"LCD-lvds_cabc_en");
		if (ret) {
			printk(KERN_ERR "Failed LCD-lvds_cabc_en req\n");
			goto failed_req_lvds_en;
		}
		gpio_direction_output(mapphone_displ_lvds_cabc_en, 1);
	}

	if (mapphone_displ_color_en >= 0) {
		ret = gpio_request(mapphone_displ_color_en, "LCD-color_en");
		if (ret) {
			printk(KERN_ERR "Failed LCD-color_en req\n");
			goto failed_req_color_en;
		}
		gpio_direction_output(mapphone_displ_color_en, 1);
	}

	if (mapphone_displ_lcd_bl_pwm >= 0) {
		ret = gpio_request(mapphone_displ_lcd_bl_pwm,
					"LCD-lcd_bl_pwm");
		if (ret) {
			printk(KERN_ERR "Failed LCD-lcd_bl_pwm req\n");
			goto failed_req_lcd_bl_pwm;
		}
		gpio_direction_output(mapphone_displ_lcd_bl_pwm, 0);
	}

	if (mapphone_displ_lvds_wp_g >= 0) {
		ret = gpio_request(mapphone_displ_lvds_wp_g,
					"LCD-lvds_wp_g");
		if (ret) {
			printk(KERN_ERR "Failed LCD-lvds_wp_g req\n");
			goto failed_req_lvds_wp_g;
		}
		gpio_direction_output(mapphone_displ_lvds_wp_g, 1);
	}

	if (mapphone_displ_lvds_wp_e >= 0) {
		ret = gpio_request(mapphone_displ_lvds_wp_e,
					"LCD-lvds_wp_e");
		if (ret) {
			printk(KERN_ERR "Failed LCD-lvds_wp_e req\n");
			goto failed_req_lvds_wp_e;
		}
		gpio_direction_output(mapphone_displ_lvds_wp_e, 1);
	}

#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	if (mapphone_feature_hdmi) {
		/* Set the bits to disable "internal pullups" for the DDC
		 * clk and data lines.  This is required for ES2.3 parts
		 * and beyond.  If these are not set EDID reads fails.
		 */
		if (cpu_is_omap44xx()) {
			omap_writel(HDMI_CONTROL_I2C_1_DDC_PU_DIS,
						HDMI_CONTROL_I2C_1_REG);
		}
		platform_device_register(&omap_dssmgr_device);
	} else {
		/* Remove HDTV from the DSS device list */
		mapphone_dss_data.num_devices--;
	}
#endif

	platform_device_register(&omap_panel_device);
	omap_display_init(&mapphone_dss_data);

	return;

failed_hdmi_5v:
	gpio_free(mapphone_displ_lvds_wp_e);
failed_req_lvds_wp_e:
	gpio_free(mapphone_displ_lvds_wp_g);
failed_req_lvds_wp_g:
	gpio_free(mapphone_displ_lcd_bl_pwm);
failed_req_lcd_bl_pwm:
	gpio_free(mapphone_displ_color_en);
failed_req_color_en:
	gpio_free(mapphone_displ_lvds_cabc_en);
failed_req_lvds_en:
failed_pwr_supply:
	for (i = 0; i < num_gpio_handled; i++) {
		supply = &(panel_data->disp_vol_supply[i]);
		if (supply->en_gpio != 0)
			gpio_free(supply->en_gpio);
	}
failed_reset:
	gpio_free(mapphone_panel_data.reset_gpio);
}
