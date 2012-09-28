/*
 * linux/arch/arm/mach-omap2/board-MAPPHONE-sensors.c
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
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/keypad.h>

#include "linux/i2c/lp3907_i2c.h"

#define MAPPHONE_MDTV_INT_GPIO			38
#define MAPPHONE_MDTV_PWDN_GPIO			53
#define MAPPHONE_MDTV_RESET_N_GPIO		54
#define MAPPHONE_MDTV_REG_EN_GPIO		21

static int mapphone_lp3907_init(void)
{
	printk(KERN_INFO "mapphone_lp3907_init()");
	return 0;
}

static void mapphone_lp3907_exit(void)
{
	/*regulator_put(mapphone_akm8973_regulator);*/
}

static int mapphone_lp3907_power_on(void)
{
	/* EN_T is high */
	gpio_set_value(MAPPHONE_MDTV_REG_EN_GPIO, 1);
	mdelay(6);	/* stable time */

	/* SPI pin control */
	/*omap_cfg_reg(F1_34XX_MDTV_INT_ON);*/
	omap_cfg_reg(AC3_34XX_MDTV_SIMO_ON);
	omap_cfg_reg(AD4_34XX_MDTV_SOMI_ON);
	omap_cfg_reg(AD3_34XX_MDTV_CS_ON);
	omap_cfg_reg(AA3_34XX_MDTV_CLK_ON);
	mdelay(5);
	printk(KERN_INFO "mapphone_lp3907_power_on()");
	return 0;
}

static int mapphone_lp3907_power_off(void)
{
	/* SPI pin control */
	/*omap_cfg_reg(F1_34XX_MDTV_INT_OFF);*/
	omap_cfg_reg(AC3_34XX_MDTV_SIMO_OFF);
	omap_cfg_reg(AD4_34XX_MDTV_SOMI_OFF);
	omap_cfg_reg(AD3_34XX_MDTV_CS_OFF);
	omap_cfg_reg(AA3_34XX_MDTV_CLK_OFF);

	/* EN_T is low */
	gpio_set_value(MAPPHONE_MDTV_REG_EN_GPIO, 0);
	mdelay(6); /* stable time */

	printk(KERN_INFO "mapphone_lp3907_power_off()");
	return 0;
}

struct lp3907_platform_data mapphone_lp3907_data = {
	.init = mapphone_lp3907_init,
	.exit = mapphone_lp3907_exit,
	.power_on = mapphone_lp3907_power_on,
	.power_off = mapphone_lp3907_power_off,
};

/*
*	TDMB module initialize.
*/
void __init mapphone_mdtv_init(void)
{
	/* MTV_INT pin */
	gpio_request(MAPPHONE_MDTV_INT_GPIO, "sms1130 int");
	gpio_direction_input(MAPPHONE_MDTV_INT_GPIO);
	omap_cfg_reg(T3_34XX_GPIO38);

	/* MTV_PWDN pin - low */
	gpio_request(MAPPHONE_MDTV_PWDN_GPIO, "sms1130 pwdn");
	gpio_direction_output(MAPPHONE_MDTV_PWDN_GPIO, 0);
	omap_cfg_reg(V8_34XX_GPIO53_OUT);

	/* MTV_RST_N pin - low */
	gpio_request(MAPPHONE_MDTV_RESET_N_GPIO, "sms1130 reset");
	gpio_direction_output(MAPPHONE_MDTV_RESET_N_GPIO, 0);
	omap_cfg_reg(U8_34XX_GPIO54_OUT);

	/* MTV_REG_EN pin - low */
	gpio_request(MAPPHONE_MDTV_REG_EN_GPIO, "lp3907 en");
	gpio_direction_output(MAPPHONE_MDTV_REG_EN_GPIO, 0);
	omap_cfg_reg(V8_34XX_GPIO53_OUT);

	printk(KERN_INFO "[TDMB] mapphone_mdtv_init()\n");
}
