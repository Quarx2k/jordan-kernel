/*
 * linux/arch/arm/mach-omap2/board-minnow-sensors.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2012 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#if defined(CONFIG_MFD_M4SENSORHUB) || defined(CONFIG_MFD_M4SENSORHUB_MODULE)
#include <linux/m4sensorhub.h>
#include <linux/m4sensorhub_gpio.h>
#endif
#include <linux/regulator/consumer.h>
#include <linux/vib-gpio.h>

#include <linux/gpio.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

static struct regulator *minnow_vibrator_regulator;
static int minnow_vibrator_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vvib");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	minnow_vibrator_regulator = reg;
	return 0;
}

static void minnow_vibrator_exit(void)
{
	regulator_put(minnow_vibrator_regulator);
}

static int minnow_vibrator_power_on(void)
{
	if (minnow_vibrator_regulator)
		return regulator_enable(minnow_vibrator_regulator);
	return 0;
}

static int minnow_vibrator_power_off(void)
{
	if (minnow_vibrator_regulator)
		return regulator_disable(minnow_vibrator_regulator);
	return 0;
}

static struct vib_gpio_platform_data minnow_vib_gpio_data = {
	.gpio = -1,
	.max_timeout = 15000,
	.active_low = 0,
	.initial_vibrate = 0,

	.init = minnow_vibrator_initialization,
	.exit = minnow_vibrator_exit,
	.power_on = minnow_vibrator_power_on,
	.power_off = minnow_vibrator_power_off,
};

static struct platform_device minnow_vib_gpio = {
	.name           = "vib-gpio",
	.id             = -1,
	.dev            = {
		.platform_data  = &minnow_vib_gpio_data,
	},
};



/*
 * Sensors
 */

void __init minnow_sensors_init(void)
{
	platform_device_register(&minnow_vib_gpio);
}
