/*
 * linux/arch/arm/mach-omap2/board-sholes-sensors.c
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
#include <linux/input.h>
#include <linux/sfh7743.h>
#include <linux/bu52014hfv.h>
#include <linux/lis331dlh.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/vib-gpio.h>

#include <plat/mux.h>
#include <plat/gpio.h>
#include <plat/keypad.h>

#define SHOLES_PROX_INT_GPIO		180
#define SHOLES_HF_NORTH_GPIO		10
#define SHOLES_HF_SOUTH_GPIO		111
#define SHOLES_AKM8973_INT_GPIO		175
#define SHOLES_AKM8973_RESET_GPIO	28
#define SHOLES_VIBRATOR_GPIO		181

static struct regulator *sholes_vibrator_regulator;
static int sholes_vibrator_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vvib");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	sholes_vibrator_regulator = reg;
	return 0;
}

static void sholes_vibrator_exit(void)
{
	regulator_put(sholes_vibrator_regulator);
}

static int sholes_vibrator_power_on(void)
{
	regulator_set_voltage(sholes_vibrator_regulator, 3000000, 3000000);
	return regulator_enable(sholes_vibrator_regulator);
}

static int sholes_vibrator_power_off(void)
{
	if (sholes_vibrator_regulator)
		return regulator_disable(sholes_vibrator_regulator);
	return 0;
}

static struct vib_gpio_platform_data sholes_vib_gpio_data = {
	.gpio = SHOLES_VIBRATOR_GPIO,
	.max_timeout = 15000,
	.active_low = 0,
	.initial_vibrate = 0,

	.init = sholes_vibrator_initialization,
	.exit = sholes_vibrator_exit,
	.power_on = sholes_vibrator_power_on,
	.power_off = sholes_vibrator_power_off,
};

static struct platform_device sholes_vib_gpio = {
	.name           = "vib-gpio",
	.id             = -1,
	.dev            = {
		.platform_data  = &sholes_vib_gpio_data,
	},
};

static struct regulator *sholes_sfh7743_regulator;
static int sholes_sfh7743_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vsdio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	sholes_sfh7743_regulator = reg;
	return 0;
}

static void sholes_sfh7743_exit(void)
{
	regulator_put(sholes_sfh7743_regulator);
}

static int sholes_sfh7743_power_on(void)
{
	return regulator_enable(sholes_sfh7743_regulator);
}

static int sholes_sfh7743_power_off(void)
{
	if (sholes_sfh7743_regulator)
		return regulator_disable(sholes_sfh7743_regulator);
	return 0;
}

static struct sfh7743_platform_data sholes_sfh7743_data = {
	.init = sholes_sfh7743_initialization,
	.exit = sholes_sfh7743_exit,
	.power_on = sholes_sfh7743_power_on,
	.power_off = sholes_sfh7743_power_off,

	.gpio = SHOLES_PROX_INT_GPIO,
};

static void __init sholes_sfh7743_init(void)
{
	gpio_request(SHOLES_PROX_INT_GPIO, "sfh7743 proximity int");
	gpio_direction_input(SHOLES_PROX_INT_GPIO);
	omap_cfg_reg(Y3_34XX_GPIO180);
}


static struct bu52014hfv_platform_data bu52014hfv_platform_data = {
	.docked_north_gpio = SHOLES_HF_NORTH_GPIO,
	.docked_south_gpio = SHOLES_HF_SOUTH_GPIO,
	.north_is_desk = 1,
};

static struct regulator *sholes_lis331dlh_regulator;
static int sholes_lis331dlh_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	sholes_lis331dlh_regulator = reg;
	return 0;
}

static void sholes_lis331dlh_exit(void)
{
	regulator_put(sholes_lis331dlh_regulator);
}

static int sholes_lis331dlh_power_on(void)
{
	return regulator_enable(sholes_lis331dlh_regulator);
}

static int sholes_lis331dlh_power_off(void)
{
	if (sholes_lis331dlh_regulator)
		return regulator_disable(sholes_lis331dlh_regulator);
	return 0;
}

struct lis331dlh_platform_data sholes_lis331dlh_data = {
	.init = sholes_lis331dlh_initialization,
	.exit = sholes_lis331dlh_exit,
	.power_on = sholes_lis331dlh_power_on,
	.power_off = sholes_lis331dlh_power_off,

	.min_interval	= 1,
	.poll_interval	= 200,

	.g_range	= LIS331DLH_G_8G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,
};

static void __init sholes_akm8973_init(void)
{
	gpio_request(SHOLES_AKM8973_RESET_GPIO, "akm8973 reset");
	gpio_direction_output(SHOLES_AKM8973_RESET_GPIO, 1);
	omap_cfg_reg(AB10_34XX_GPIO28_OUT);

	gpio_request(SHOLES_AKM8973_INT_GPIO, "akm8973 irq");
	gpio_direction_input(SHOLES_AKM8973_INT_GPIO);
	omap_cfg_reg(AC3_34XX_GPIO175);
}

struct platform_device sfh7743_platform_device = {
	.name = "sfh7743",
	.id = -1,
	.dev = {
		.platform_data = &sholes_sfh7743_data,
	},
};

static struct platform_device omap3430_hall_effect_dock = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52014hfv_platform_data,
	},
};

static void sholes_vibrator_init(void)
{
	gpio_request(SHOLES_VIBRATOR_GPIO, "vibrator");
	gpio_direction_output(SHOLES_VIBRATOR_GPIO, 0);
	omap_cfg_reg(Y4_34XX_GPIO181);
}

static struct platform_device *sholes_sensors[] __initdata = {
	&sfh7743_platform_device,
	&omap3430_hall_effect_dock,
	&sholes_vib_gpio,
};

static void sholes_hall_effect_init(void)
{
	gpio_request(SHOLES_HF_NORTH_GPIO, "sholes dock north");
	gpio_direction_input(SHOLES_HF_NORTH_GPIO);
	omap_cfg_reg(AG25_34XX_GPIO10);

	gpio_request(SHOLES_HF_SOUTH_GPIO, "sholes dock south");
	gpio_direction_input(SHOLES_HF_SOUTH_GPIO);
	omap_cfg_reg(B26_34XX_GPIO111);
}

void __init sholes_sensors_init(void)
{
	sholes_sfh7743_init();
	sholes_hall_effect_init();
	sholes_vibrator_init();
	sholes_akm8973_init();
	platform_add_devices(sholes_sensors, ARRAY_SIZE(sholes_sensors));
}
