/*
 * Copyright (C) 2010 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-zoom-peripherals.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/spi/spi.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <plat/mcspi.h>
#include <video/omapdss.h>
#include <plat/omap-pm.h>
#include "mux.h"
#ifdef CONFIG_PANEL_SIL9022
#include <mach/sil9022.h>
#endif

#define LCD_PANEL_RESET_GPIO_PROD	96
#define LCD_PANEL_RESET_GPIO_PILOT	55
#define LCD_PANEL_QVGA_GPIO		56
#define TV_PANEL_ENABLE_GPIO		95
#define SIL9022_RESET_GPIO		97

#ifdef CONFIG_PANEL_SIL9022
void config_hdmi_gpio(void)
{
	/* HDMI_RESET uses CAM_PCLK mode 4*/
	omap_mux_init_signal("gpio_97", OMAP_PIN_INPUT_PULLUP);
}

void zoom_hdmi_reset_enable(int level)
{
	/* Set GPIO_97 to high to pull SiI9022 HDMI transmitter out of reset
	* and low to disable it.
	*/
	gpio_request(SIL9022_RESET_GPIO, "hdmi reset");
	gpio_direction_output(SIL9022_RESET_GPIO, level);
}

static int zoom_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	zoom_hdmi_reset_enable(1);
	return 0;
}

static void zoom_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
	zoom_hdmi_reset_enable(0);
}

struct hdmi_platform_data zoom_hdmi_data = {

};

static struct omap_dss_device zoom_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_DPI,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
		},
	},
	.phy.dpi.data_lines = 24,
	.platform_enable = zoom_panel_enable_hdmi,
	.platform_disable = zoom_panel_disable_hdmi,
	.dev		= {
		.platform_data = &zoom_hdmi_data,
	},
};
#endif

static struct gpio zoom_lcd_gpios[] __initdata = {
	{ -EINVAL,		GPIOF_OUT_INIT_HIGH, "lcd reset" },
	{ LCD_PANEL_QVGA_GPIO,	GPIOF_OUT_INIT_HIGH, "lcd qvga"	 },
};

static void __init zoom_lcd_panel_init(void)
{
	zoom_lcd_gpios[0].gpio = (omap_rev() > OMAP3430_REV_ES3_0) ?
			LCD_PANEL_RESET_GPIO_PROD :
			LCD_PANEL_RESET_GPIO_PILOT;

	if (gpio_request_array(zoom_lcd_gpios, ARRAY_SIZE(zoom_lcd_gpios)))
		pr_err("%s: Failed to get LCD GPIOs.\n", __func__);
}

static void __init zoom_tv_panel_init(void)
{
	int ret;

	ret = gpio_request(TV_PANEL_ENABLE_GPIO, "tv panel");
	if (ret) {
		pr_err("Failed to get TV_PANEL_ENABLE_GPIO.\n");
		goto err1;
	}
	gpio_direction_output(TV_PANEL_ENABLE_GPIO, 0);

err1:
	return;
}

static int zoom_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	return 0;
}

static void zoom_panel_disable_lcd(struct omap_dss_device *dssdev)
{
}

/*
 * PWMA/B register offsets (TWL4030_MODULE_PWMA)
 */
#define TWL_INTBR_PMBR1	0xD
#define TWL_INTBR_GPBR1	0xC
#define TWL_LED_PWMON	0x0
#define TWL_LED_PWMOFF	0x1

static int zoom_set_bl_intensity(struct omap_dss_device *dssdev, int level)
{
	unsigned char c;
	u8 mux_pwm, enb_pwm;

	if (level > 100)
		return -1;

	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &mux_pwm, TWL_INTBR_PMBR1);
	twl_i2c_read_u8(TWL4030_MODULE_INTBR, &enb_pwm, TWL_INTBR_GPBR1);

	if (level == 0) {
		/* disable pwm1 output and clock */
		enb_pwm = enb_pwm & 0xF5;
		/* change pwm1 pin to gpio pin */
		mux_pwm = mux_pwm & 0xCF;
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					enb_pwm, TWL_INTBR_GPBR1);
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					mux_pwm, TWL_INTBR_PMBR1);
		return 0;
	}

	if (!((enb_pwm & 0xA) && (mux_pwm & 0x30))) {
		/* change gpio pin to pwm1 pin */
		mux_pwm = mux_pwm | 0x30;
		/* enable pwm1 output and clock*/
		enb_pwm = enb_pwm | 0x0A;
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					mux_pwm, TWL_INTBR_PMBR1);
		twl_i2c_write_u8(TWL4030_MODULE_INTBR,
					enb_pwm, TWL_INTBR_GPBR1);
	}

	c = ((50 * (100 - level)) / 100) + 1;
	twl_i2c_write_u8(TWL4030_MODULE_PWM1, 0x7F, TWL_LED_PWMOFF);
	twl_i2c_write_u8(TWL4030_MODULE_PWM1, c, TWL_LED_PWMON);

	return 0;
}

static int zoom_panel_enable_tv(struct omap_dss_device *dssdev)
{
	int ret;
	struct regulator *vdac_reg;

	vdac_reg = regulator_get(NULL, "vdda_dac");
	if (IS_ERR(vdac_reg)) {
		pr_err("Unable to get vdac regulator\n");
		return PTR_ERR(vdac_reg);
	}
	ret = regulator_enable(vdac_reg);
	if (ret < 0)
		return ret;
	gpio_set_value(TV_PANEL_ENABLE_GPIO, 0);

	return 0;
}

static void zoom_panel_disable_tv(struct omap_dss_device *dssdev)
{
	struct regulator *vdac_reg;

	vdac_reg = regulator_get(NULL, "vdda_dac");
	if (IS_ERR(vdac_reg)) {
		pr_err("Unable to get vpll2 regulator\n");
		return;
	}
	regulator_disable(vdac_reg);
	gpio_set_value(TV_PANEL_ENABLE_GPIO, 1);
}

static struct omap_dss_device zoom_lcd_device = {
	.name			= "lcd",
	.driver_name		= "NEC_8048_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.clocks = {
		.dispc  = {
			.dispc_fclk_src = OMAP_DSS_CLK_SRC_DSI_PLL_HSDIV_DISPC,
		},
	},
	.phy.dpi.data_lines	= 24,
	.platform_enable	= zoom_panel_enable_lcd,
	.platform_disable	= zoom_panel_disable_lcd,
	.max_backlight_level	= 100,
	.set_backlight		= zoom_set_bl_intensity,
};

static struct omap_dss_device zoom_tv_device = {
	.name                   = "tv",
	.driver_name            = "venc",
	.type                   = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type          = OMAP_DSS_VENC_TYPE_COMPOSITE,
	.platform_enable        = zoom_panel_enable_tv,
	.platform_disable       = zoom_panel_disable_tv,
};

static struct omap_dss_device *zoom_dss_devices[] = {
	&zoom_lcd_device,
	#ifdef CONFIG_PANEL_SIL9022
		&zoom_hdmi_device,
	#endif
	&zoom_tv_device
};

static struct omap_dss_board_info zoom_dss_data = {
	.num_devices		= ARRAY_SIZE(zoom_dss_devices),
	.devices		= zoom_dss_devices,
	.default_device		= &zoom_lcd_device,
};

static struct omap2_mcspi_device_config dss_lcd_mcspi_config = {
	.turbo_mode		= 1,
	.single_channel	= 1,  /* 0: slave, 1: master */
};

static struct spi_board_info nec_8048_spi_board_info[] __initdata = {
	[0] = {
		.modalias		= "nec_8048_spi",
		.bus_num		= 1,
		.chip_select		= 2,
		.max_speed_hz		= 375000,
		.controller_data	= &dss_lcd_mcspi_config,
	},
};

void __init zoom_display_init(void)
{
	omap_display_init(&zoom_dss_data);
	spi_register_board_info(nec_8048_spi_board_info,
				ARRAY_SIZE(nec_8048_spi_board_info));
	zoom_lcd_panel_init();
	zoom_tv_panel_init();
}

