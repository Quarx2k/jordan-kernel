/*
 * linux/arch/arm/mach-omap2/board-mapphone-sensors.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/airc.h>
#include <linux/akm8973_akmd.h>
#include <linux/akm8975.h>
#include <linux/bu52014hfv.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio_mapping.h>
#include <linux/i2c/adp8870.h>
#include <linux/input.h>
#include <linux/isl29030.h>
#include <linux/kxtf9.h>
#include <linux/leds.h>
#include <linux/lis331dlh.h>
#include <linux/regulator/consumer.h>
#include <linux/sfh7743.h>
#include <linux/vib-gpio.h>
#include <linux/vib-pwm.h>
#include <linux/interrupt.h>

#include <plat/dmtimer.h>
#include <plat/gpio.h>
#include <plat/keypad.h>
#include <plat/mux.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#define MAPPHONE_PROX_INT_GPIO		180
#define MAPPHONE_VIBRATOR_GPIO		181

#define PROXIMITY_SFH7743		0
#define PROXIMITY_ISL29030		1

/*
 * Vibe
 */
static struct regulator *mapphone_vibrator_regulator;
static int mapphone_vibrator_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vvib");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_vibrator_regulator = reg;
	return 0;
}

static void mapphone_vibrator_exit(void)
{
	regulator_put(mapphone_vibrator_regulator);
}

static int mapphone_vibrator_power_on(void)
{
	regulator_set_voltage(mapphone_vibrator_regulator, 3000000, 3000000);
	return regulator_enable(mapphone_vibrator_regulator);
}

static int mapphone_vibrator_power_off(void)
{
	if (mapphone_vibrator_regulator)
		return regulator_disable(mapphone_vibrator_regulator);
	return 0;
}

static struct vib_gpio_platform_data mapphone_vib_gpio_data = {
	.gpio = MAPPHONE_VIBRATOR_GPIO,
	.max_timeout = 15000,
	.active_low = 0,
	.initial_vibrate = 0,

	.init = mapphone_vibrator_initialization,
	.exit = mapphone_vibrator_exit,
	.power_on = mapphone_vibrator_power_on,
	.power_off = mapphone_vibrator_power_off,
};

static struct platform_device mapphone_vib_gpio = {
	.name           = "vib-gpio",
	.id             = -1,
	.dev            = {
		.platform_data  = &mapphone_vib_gpio_data,
	},
};

static void mapphone_vibrator_init(void)
{
	int vibrator_gpio = MAPPHONE_VIBRATOR_GPIO;
#ifdef CONFIG_ARM_OF
	vibrator_gpio = get_gpio_by_name("vib_control_en");
	if (vibrator_gpio < 0) {
		printk(KERN_DEBUG
			"cannot retrieve vib_control_en from device tree\n");
		vibrator_gpio = MAPPHONE_VIBRATOR_GPIO;
	}
	mapphone_vib_gpio_data.gpio = vibrator_gpio;
#endif
	if (gpio_request(mapphone_vib_gpio_data.gpio, "vib_ctrl_en")) {
		printk(KERN_ERR "vib_control_en GPIO request failed!\n");
		return;
	}

	gpio_direction_output(vibrator_gpio, 0);
	omap_cfg_reg(Y4_34XX_GPIO181);
}
/*
 * KXTF9
 */

#define MAPPHONE_KXTF9_INT_GPIO		22

static struct regulator *mapphone_kxtf9_regulator;
static int mapphone_kxtf9_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_kxtf9_regulator = reg;
	return 0;
}

static void mapphone_kxtf9_exit(void)
{
	regulator_put(mapphone_kxtf9_regulator);
}

static int mapphone_kxtf9_power_on(void)
{
	return regulator_enable(mapphone_kxtf9_regulator);
}

static int mapphone_kxtf9_power_off(void)
{
	if (mapphone_kxtf9_regulator)
		return regulator_disable(mapphone_kxtf9_regulator);
	return 0;
}

struct kxtf9_platform_data mapphone_kxtf9_data = {
	.init = mapphone_kxtf9_initialization,
	.exit = mapphone_kxtf9_exit,
	.power_on = mapphone_kxtf9_power_on,
	.power_off = mapphone_kxtf9_power_off,

	.min_interval	= 2,
	.poll_interval	= 200,

	.g_range	= KXTF9_G_8G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,

	.data_odr_init		= ODR12_5,
	.ctrl_reg1_init		= RES_12BIT | KXTF9_G_2G | TPE | WUFE | TDTE,
	.int_ctrl_init		= IEA | IEN,
	.tilt_timer_init	= 0x03,
	.engine_odr_init	= OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init		= 0x0A,
	.wuf_thresh_init	= 0x20,
	.tdt_timer_init		= 0x78,
	.tdt_h_thresh_init	= 0xB6,
	.tdt_l_thresh_init	= 0x1A,
	.tdt_tap_timer_init	= 0xA2,
	.tdt_total_timer_init	= 0x24,
	.tdt_latency_timer_init	= 0x28,
	.tdt_window_timer_init	= 0xA0,

	.gpio = MAPPHONE_KXTF9_INT_GPIO,
	.gesture = 0,
	.sensitivity_low = {
		0x50, 0xFF, 0xB8, 0x32, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_medium = {
		0x50, 0xFF, 0x68, 0x32, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_high = {
		0x78, 0xB6, 0x1A, 0xA2, 0x24, 0x28, 0xA0,
	},
};

static ssize_t kxtf9_sysfs_show(struct class *dev, char *buf)
{
	int data;
	char *str = buf;
	ssize_t count;

	data = gpio_get_value(MAPPHONE_KXTF9_INT_GPIO);
	str += sprintf(str, "%d\n", data);
	count = (ssize_t) (str - buf);
	return count;
}

static CLASS_ATTR(kxtf9, S_IRUGO, kxtf9_sysfs_show, NULL);

static void __init mapphone_kxtf9_init(void)
{
	struct class *class;

#ifdef CONFIG_ARM_OF
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_path(DT_PATH_ACCELEROMETER);
	if (node) {
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_X, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_x = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_Y, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_y = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_Z, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_z = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_X, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_x = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_Y, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_y = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_Z, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_z = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_LOW, &len);
		if (prop && len)
				memcpy(mapphone_kxtf9_data.sensitivity_low,
						(u8 *)prop, len);
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_MEDIUM, &len);
		if (prop && len)
				memcpy(mapphone_kxtf9_data.sensitivity_medium,
						(u8 *)prop, len);
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_HIGH, &len);
		if (prop && len)
				memcpy(mapphone_kxtf9_data.sensitivity_high,
						(u8 *)prop, len);
		of_node_put(node);
	}
#endif
	gpio_request(MAPPHONE_KXTF9_INT_GPIO, "kxtf9 accelerometer int");
	gpio_direction_input(MAPPHONE_KXTF9_INT_GPIO);
	omap_cfg_reg(AF9_34XX_GPIO22_DOWN);

	class = class_create(THIS_MODULE, "kxtf9");
	if (IS_ERR(class))
		printk(KERN_ERR "kxtf9 can't register class\n");
	else if (class_create_file(class, &class_attr_kxtf9)) {
		printk(KERN_ERR "kxtf9: can't create sysfs\n");
		class_destroy(class);
	}

}

/*
 * AKM8973
 */

#define MAPPHONE_AKM8973_INT_GPIO	175
#define MAPPHONE_AKM8973_RESET_GPIO	28

static void __init mapphone_akm8973_init(void)
{
	gpio_request(MAPPHONE_AKM8973_RESET_GPIO, "akm8973 reset");
	gpio_direction_output(MAPPHONE_AKM8973_RESET_GPIO, 1);
	omap_cfg_reg(AB10_34XX_GPIO28_OUT);

	gpio_request(MAPPHONE_AKM8973_INT_GPIO, "akm8973 irq");
	gpio_direction_input(MAPPHONE_AKM8973_INT_GPIO);
	omap_cfg_reg(AC3_34XX_GPIO175);
}

/*
 * ISL29030
 */

struct isl29030_platform_data isl29030_pdata = {
	.configure = 0x62,
	.interrupt_cntrl = 0x20,
	.prox_lower_threshold = 0x1e,
	.prox_higher_threshold = 0x32,
	.crosstalk_vs_covered_threshold = 0x30,
	.default_prox_noise_floor = 0x30,
	.num_samples_for_noise_floor = 0x05,
	.lens_percent_t = 20,
	.regulator_name = {0},
};

static ssize_t isl29030_sysfs_show(struct class *dev, char *buf)
{
	int data;
	char *str = buf;
	ssize_t count;
	int gpio = get_gpio_by_name("als_int");

	if (gpio < 0) {
		printk(KERN_DEBUG "can't retrieve als_int.\n");
		return -ENODEV;
	}

	data = gpio_get_value(gpio);
	str += sprintf(str, "%d\n", data);
	count = (ssize_t) (str - buf);
	return count;
}

static CLASS_ATTR(isl29030, S_IRUGO, isl29030_sysfs_show, NULL);

static int mapphone_isl29030_init(void)
{
	int err = 0;
#ifdef CONFIG_ARM_OF
	struct device_node *prox_node = NULL;
	const void *prop = NULL;
	const char *prop_str = NULL;
	int len = 0;
	int i;
	int gpio = 0;
	struct class *class;
	prox_node = of_find_node_by_path(DT_PATH_PROX);
	if (prox_node != NULL) {
		prop_str = DT_PROP_ISL29030_CONF;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.configure = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_INT_CNTL;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
					 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.interrupt_cntrl = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_PROX_LOW_TH;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.prox_lower_threshold = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_PROX_HIGH_TH;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.prox_higher_threshold = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_XTALK_V_COV_TH;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.crosstalk_vs_covered_threshold =
				*(u8 *)prop;

		prop_str = DT_PROP_ISL29030_DEF_PROX_NOISE;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.default_prox_noise_floor = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_NUM_SAMP_NOISE;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.num_samples_for_noise_floor =
				*(u8 *)prop;

		prop_str = DT_PROP_ISL29030_LENS_PERCENT;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.lens_percent_t = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_REGULATOR;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_info("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
		} else {
			if (len > ISL29030_REGULATOR_NAME_LENGTH) {
				pr_err("%s - %s entry %s too long\n",
					__func__, prop_str, (char *)prop);
				/* Truncation should cause driver to err out,
				 * unless truncated name is valid */
				len = ISL29030_REGULATOR_NAME_LENGTH - 1;
			}
			for (i = 0; i < len; i++)
				isl29030_pdata.regulator_name[i] =
					((char *)prop)[i];
		}

		of_node_put(prox_node);
	} else {
		pr_err("%s - unable to read %s node from device tree.\n",
			__func__, DT_PATH_PROX);
		err = -1;
	}

	gpio = get_gpio_by_name("als_int");
	if (gpio >= 0) {
		isl29030_pdata.irq = gpio_to_irq(gpio);
		gpio_request(gpio, "isl29030 proximity int");
		gpio_direction_input(gpio);
		class = class_create(THIS_MODULE, "isl29030");
		if (IS_ERR(class))
			printk(KERN_ERR "isl29030 can't register class\n");
		else if (class_create_file(class, &class_attr_isl29030) != 0) {
			printk(KERN_ERR "isl29030: can't create sysfs\n");
			class_destroy(class);
		}
	}
#endif /* CONFIG_ARM_OF */
	return err;
}

/*
 * BU52014HFV
 */

#define MAPPHONE_HF_NORTH_GPIO		10
#define MAPPHONE_HF_SOUTH_GPIO		111

static struct bu52014hfv_platform_data bu52014hfv_platform_data = {
	.docked_north_gpio = MAPPHONE_HF_NORTH_GPIO,
	.docked_south_gpio = MAPPHONE_HF_SOUTH_GPIO,
	.north_is_desk = 1,
};

static void mapphone_bu52014hfv_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_HALLEFFECT_DOCK);

	if (node == NULL)
		return;

	prop = of_get_property(node, DT_PROP_DEV_NORTH_IS_DESK, NULL);

	if (prop)
		bu52014hfv_platform_data.north_is_desk = *(u8 *)prop;

	of_node_put(node);
#endif

	gpio_request(MAPPHONE_HF_NORTH_GPIO, "mapphone dock north");
	gpio_direction_input(MAPPHONE_HF_NORTH_GPIO);
	omap_cfg_reg(AG25_34XX_GPIO10);

	gpio_request(MAPPHONE_HF_SOUTH_GPIO, "mapphone dock south");
	gpio_direction_input(MAPPHONE_HF_SOUTH_GPIO);
	omap_cfg_reg(B26_34XX_GPIO111);
}

static struct platform_device omap3430_hall_effect_dock = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52014hfv_platform_data,
	},
};


/*
 * Sensors
 */

void __init mapphone_sensors_init(void)
{
	mapphone_kxtf9_init();

	mapphone_isl29030_init();

	mapphone_bu52014hfv_init();

	mapphone_akm8973_init();

	platform_device_register(&omap3430_hall_effect_dock);

	mapphone_vibrator_init();
	platform_device_register(&mapphone_vib_gpio);
}
