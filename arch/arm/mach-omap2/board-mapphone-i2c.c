/*
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <plat/common.h>
#include <mach/board-mapphone.h>
#include <linux/gpio.h>
#include <linux/gpio_mapping.h>
#include <linux/regulator/consumer.h>

#include "dt_path.h"
#include <linux/of.h>

#include <linux/led-lm3530.h>
#include <linux/led-cpcap-lm3554.h>
#include <linux/kxtf9.h>
#include <linux/isl29030.h>
#include <linux/bu52014hfv.h>
#include <linux/vib-gpio.h>

#define MAPPHONE_LM_3530_INT_GPIO	92
#define MAPPHONE_AKM8973_INT_GPIO	175
#define MAPPHONE_AKM8973_RESET_GPIO	28
#define MAPPHONE_KXTF9_INT_GPIO		22
#define MAPPHONE_VIBRATOR_GPIO		181

static struct i2c_board_info __initdata
	mapphone_i2c_bus1_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus2_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus3_board_info[I2C_BUS_MAX_DEVICES];

/*
 * LM3530
 */


static struct lm3530_platform_data omap3430_als_light_data = {
	.power_up_gen_config = 0x0b,
	.gen_config = 0x3b,
	.als_config = 0x6c,
	.brightness_ramp = 0x00,
	.als_zone_info = 0x00,
	.als_resistor_sel = 0x31,
	.brightness_control = 0x00,
	.zone_boundary_0 = 0x02,
	.zone_boundary_1 = 0x10,
	.zone_boundary_2 = 0x43,
	.zone_boundary_3 = 0xfc,
	.zone_target_0 = 0x51,
	.zone_target_1 = 0x6c,
	.zone_target_2 = 0x6c,
	.zone_target_3 = 0x6c,
	.zone_target_4 = 0x7e,
	.manual_current = 0x2f,
	.upper_curr_sel = 6,
	.lower_curr_sel = 3,
	.lens_loss_coeff = 6,
	.manual_als_config = 0x64,
	.als_enabled = 1,
};

static struct lm3530_platform_data omap3430_als_light_data;

void __init mapphone_als_init(void)
{
	int lm3530_int_gpio = MAPPHONE_LM_3530_INT_GPIO;
	struct device_node *als_node;
	int lm3530_reset_gpio;
	const u8 *als_val;
	int len = 0;
	als_node = of_find_node_by_path(DT_LCD_BACKLIGHT);
	if (als_node != NULL) {
		als_val = of_get_property(als_node, DT_PROP_POWERUP_GEN_CNFG,
									&len);
		if (als_val && len)
			omap3430_als_light_data.power_up_gen_config = *als_val;
		else
			pr_err("%s: Cann't get powerup gen cnfg\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_GEN_CNFG, &len);
		if (als_val && len)
			omap3430_als_light_data.gen_config = *als_val;
		else
			pr_err("%s: Cann't get gen cnfg\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ALS_CNFG, &len);
		if (als_val && len)
			omap3430_als_light_data.als_config = *als_val;
		else
			pr_err("%s: Cann't get als cnfg\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_BRIGHTNESS_RAMP,
									&len);
		if (als_val && len)
			omap3430_als_light_data.brightness_ramp = *als_val;
		else
			pr_err("%s: Cann't get brightness ramp", __func__);

		als_val = of_get_property(als_node, DT_PROP_ALS_ZONE_INFO,
									&len);
		if (als_val && len)
			omap3430_als_light_data.als_zone_info = *als_val;
		else
			pr_err("%s: Cann't get als zone info\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ALS_RESISTOR_SEL,
									&len);
		if (als_val && len)
			omap3430_als_light_data.als_resistor_sel = *als_val;
		else
			pr_err("%s: Cann't get als resistor sel\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_BRIGHTNESS_CTRL,
									&len);
		if (als_val && len)
			omap3430_als_light_data.brightness_control = *als_val;
		else
			pr_err("%s: Cann't get brightness control\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZB0, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_boundary_0 = *als_val;
		else
			pr_err("%s: Cann't get zone boundary 0\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZB1, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_boundary_1 = *als_val;
		else
			pr_err("%s: Cann't get zone boundary 1\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZB2, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_boundary_2 = *als_val;
		else
			pr_err("%s: Cann't get zone boundary 2\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZB3, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_boundary_3 = *als_val;
		else
			pr_err("%s: Cann't get zone boundary 3\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZT0, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_target_0 = *als_val;
		else
			pr_err("%s: Cann't get zone target 0\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZT1, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_target_1 = *als_val;
		else
			pr_err("%s: Cann't get zone target 1\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZT2, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_target_2 = *als_val;
		else
			pr_err("%s: Cann't get zone target 2\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZT3, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_target_3 = *als_val;
		else
			pr_err("%s: Cann't get zone target 3\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZT4, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_target_4 = *als_val;
		else
			pr_err("%s: Cann't get zone target 4\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_MANUAL_CURRENT,
									&len);
		if (als_val && len)
			omap3430_als_light_data.manual_current = *als_val;
		else
			pr_err("%s: Cann't get manual current\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_UPPER_CURR_SEL,
									&len);
		if (als_val && len)
			omap3430_als_light_data.upper_curr_sel = *als_val;
		else
			pr_err("%s: Cann't get upper curr sel\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_LOWER_CURR_SEL,
									&len);
		if (als_val && len)
			omap3430_als_light_data.lower_curr_sel = *als_val;
		else
			pr_err("%s: Cann't get lower curr sel\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_LENS_LOSS_COEFF,
									&len);
		if (als_val && len)
			omap3430_als_light_data.lens_loss_coeff = *als_val;
		else
			pr_err("%s: Cann't get lens loss coeff\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_MANUAL_ALS_CONFIG,
									&len);
		if (als_val && len)
			omap3430_als_light_data.manual_als_config = *als_val;
		else
			pr_err("%s: Cann't get manual als config\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ALS_ENABLED, &len);
		if (als_val && len)
			omap3430_als_light_data.als_enabled = *als_val;
		else
			pr_err("%s: Cann't get manual als config\n", __func__);
		of_node_put(als_node);
	}

	lm3530_int_gpio = get_gpio_by_name("lm3530_int");
	if (lm3530_int_gpio < 0) {
		printk(KERN_DEBUG"mapphone_als_init: cann't get lm3530_int from device_tree\n");
		lm3530_int_gpio = MAPPHONE_LM_3530_INT_GPIO;
	} else {
		mapphone_i2c_bus1_board_info[1].irq =
				 OMAP_GPIO_IRQ(lm3530_int_gpio);
		mapphone_i2c_bus2_board_info[3].irq =
				 OMAP_GPIO_IRQ(lm3530_int_gpio);
	}
	lm3530_reset_gpio = get_gpio_by_name("lm3530_reset");
	if (lm3530_int_gpio >= 0) {
		gpio_request(lm3530_reset_gpio, "LED reset");
		gpio_direction_output(lm3530_reset_gpio, 1);
		msleep(10);
	}

	printk(KERN_INFO "%s:Initializing\n", __func__);
	gpio_request(lm3530_int_gpio, "mapphone als int");
	gpio_direction_input(lm3530_int_gpio);
}

/*
 * AKM8973
 */

static void __init mapphone_akm8973_init(void)
{
	gpio_request(MAPPHONE_AKM8973_RESET_GPIO, "akm8973 reset");
	gpio_direction_output(MAPPHONE_AKM8973_RESET_GPIO, 1);

	gpio_request(MAPPHONE_AKM8973_INT_GPIO, "akm8973 irq");
	gpio_direction_input(MAPPHONE_AKM8973_INT_GPIO);
}

/*
 * KXTF9
 */

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

static ssize_t kxtf9_sysfs_show(struct class *dev,
	struct class_attribute *attr, char *buf)
{
	int data;
	char *str = buf;
	ssize_t count;

	data = gpio_get_value(MAPPHONE_KXTF9_INT_GPIO);
	str += sprintf(str, "%d\n", data);
	count = (ssize_t) (str - buf);
	return count;
}

static CLASS_ATTR(kxtf9, S_IRUGO, kxtf9_sysfs_show, NULL );

static void __init mapphone_kxtf9_init(void)
{
	struct class *class;

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

	gpio_request(MAPPHONE_KXTF9_INT_GPIO, "kxtf9 accelerometer int");
	gpio_direction_input(MAPPHONE_KXTF9_INT_GPIO);

	class = class_create(THIS_MODULE, "kxtf9");
	if (IS_ERR(class))
		printk(KERN_ERR "kxtf9 can't register class\n");
	else if (class_create_file(class, &class_attr_kxtf9)) {
		printk(KERN_ERR "kxtf9: can't create sysfs\n");
		class_destroy(class);
	}

}

/*
 * LM3554
 */

static struct lm3554_platform_data mapphone_camera_flash_3554 = {
	.flags	= 0x0,
	.torch_brightness_def = 0xa0,
	.flash_brightness_def = 0x30,//0x78, GB Devtree doesn't have this value, it can cause burn the flashlight. So set this value as in froyo devtree.
	.flash_duration_def = 0x28,
	.config_reg_1_def = 0xe0,
	.config_reg_2_def = 0xf0,
	.vin_monitor_def = 0x00,
	.gpio_reg_def = 0x0,
};


static void __init mapphone_lm3554_init(void) {
	struct device_node *node;
	int len = 0;
	const uint32_t *val;

	node = of_find_node_by_path(DT_PATH_LM3554);
	if (node != NULL) {
		val =
			of_get_property(node, "device_available", &len);
		if (val && len)
			mapphone_camera_flash_3554.flags = *val;
		val = of_get_property(node, "flash_duration_def", &len);
		if (val && len)
			mapphone_camera_flash_3554.flash_duration_def = *val;
	}
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

static ssize_t isl29030_sysfs_show(struct class *dev,
	struct class_attribute *attr, char *buf)
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
	struct device_node *prox_node = NULL;
	const void *prop = NULL;
	const char *prop_str = NULL;
	int len = 0;
	int i;
	int gpio = 0;
	struct class *class;
	prox_node = of_find_node_by_path(DT_PATH_PROX);
	if (prox_node != NULL) {
		pr_err("%s - opened node %s from device tree\n",
			__func__, DT_PATH_PROX);

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
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_HALLEFFECT_DOCK);

	if (node == NULL)
		return;

	prop = of_get_property(node, DT_PROP_DEV_NORTH_IS_DESK, NULL);

	if (prop)
		bu52014hfv_platform_data.north_is_desk = *(u8 *)prop;

	of_node_put(node);
	gpio_request(MAPPHONE_HF_NORTH_GPIO, "mapphone dock north");
	gpio_direction_input(MAPPHONE_HF_NORTH_GPIO);

	gpio_request(MAPPHONE_HF_SOUTH_GPIO, "mapphone dock south");
	gpio_direction_input(MAPPHONE_HF_SOUTH_GPIO);
}

static struct platform_device omap3430_hall_effect_dock = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52014hfv_platform_data,
	},
};

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

	vibrator_gpio = get_gpio_by_name("vib_control_en");
	if (vibrator_gpio < 0) {
		printk(KERN_DEBUG
			"cannot retrieve vib_control_en from device tree\n");
		vibrator_gpio = MAPPHONE_VIBRATOR_GPIO;
	}
	mapphone_vib_gpio_data.gpio = vibrator_gpio;
	if (gpio_request(mapphone_vib_gpio_data.gpio, "vib_ctrl_en")) {
		printk(KERN_ERR "vib_control_en GPIO request failed!\n");
		return;
	}

	gpio_direction_output(vibrator_gpio, 0);
}

/* Init I2C Bus Interfaces */

static struct i2c_board_info *get_board_info
(
	char *dev_name,
	int bus_num,
	struct i2c_board_info *board_info_table,
	int size
)
{
	int i;
	char *entry_name;

	if (dev_name != NULL && board_info_table) {
		/* search for the name in the table */
		for (i = 0; i < size; i++) {
				entry_name = board_info_table[i].type;
				if (strncmp(entry_name, dev_name,\
					strlen(entry_name)) == 0)
					return &board_info_table[i];
			}
	}

	return NULL;
}

static int initialize_i2c_bus_info
(
	int bus_num,
	struct i2c_board_info *board_info,
	int info_size,
	struct i2c_board_info *master_board_info,
	int master_info_size
)
{
	int dev_cnt = 0;
	struct device_node *bus_node;
	const void *feat_prop;
	char *device_names;
	char dev_name[I2C_MAX_DEV_NAME_LEN];
	int device_name_len, i, j;
	struct i2c_board_info *master_entry;
	char prop_name[I2C_BUS_PROP_NAME_LEN];

	j = 0;

	bus_node = of_find_node_by_path(DT_PATH_I2C);
	if (bus_node == NULL || board_info == NULL)
		return dev_cnt;

	snprintf(prop_name, I2C_BUS_PROP_NAME_LEN,
		"bus%1ddevices", bus_num);

	feat_prop = of_get_property(bus_node,
			prop_name, NULL);
	if (NULL != feat_prop) {
		device_names = (char *)feat_prop;
		printk(KERN_INFO
			"I2C-%d devices: %s\n", bus_num, device_names);
		device_name_len = strlen(device_names);

		memset(dev_name, 0x0, I2C_MAX_DEV_NAME_LEN);

		for (i = 0; i < device_name_len; i++) {

			if (device_names[i] != '\0' &&
				device_names[i] != ',')
				dev_name[j++] = device_names[i];
			/* parse for ',' in string */
			if (device_names[i] == ',' ||
				(i == device_name_len-1)) {

				if (dev_cnt < info_size) {
					master_entry =
						get_board_info(dev_name,
							bus_num,
							master_board_info,
							master_info_size);
					if (master_entry != NULL) {
						memcpy(
							&board_info[dev_cnt++],
							master_entry,
							sizeof(
							struct i2c_board_info));
						printk(KERN_INFO
							"%s -> I2C bus-%d\n",
							master_entry->type,
							bus_num);

					}
					j = 0;
					memset(
							dev_name,
							0x0,
							I2C_MAX_DEV_NAME_LEN);
				}
			}
		}
	}
	return dev_cnt;
}

static struct i2c_board_info __initdata
	mapphone_i2c_1_boardinfo[] = {
	{
		I2C_BOARD_INFO("invalid_touch_panel", 0x11),
		.platform_data = NULL,
		.irq = OMAP_GPIO_IRQ(99),  /* Legacy val */
	},
	{
		I2C_BOARD_INFO("invalid_touch_btn", 0x11),
		.platform_data = NULL,
	},
	{
		I2C_BOARD_INFO(LD_LM3530_NAME, 0x38),
		.platform_data = &omap3430_als_light_data,
		.irq = OMAP_GPIO_IRQ(MAPPHONE_LM_3530_INT_GPIO),
	},
	{
		I2C_BOARD_INFO(LD_ISL29030_NAME, 0x44),
		.platform_data = &isl29030_pdata,
	},
};

static struct i2c_board_info __initdata
	mapphone_i2c_2_boardinfo[] = {
	{
		I2C_BOARD_INFO("akm8973", 0x1C),
		.irq = OMAP_GPIO_IRQ(MAPPHONE_AKM8973_INT_GPIO),
	},
	{
		I2C_BOARD_INFO("kxtf9", 0x0F),
		.platform_data = &mapphone_kxtf9_data,
	},
};

static struct i2c_board_info __initdata
	mapphone_i2c_3_boardinfo[] = {
	{
		I2C_BOARD_INFO("lm3554_led", 0x53),
		.platform_data = &mapphone_camera_flash_3554,
	},
};

static struct omap_i2c_bus_board_data __initdata mapphone_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata mapphone_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata mapphone_i2c_3_bus_pdata;

void __init mapphone_i2c_init(void)
{
	int i2c_bus_devices = 0;

	/* touch_init() must run before i2c_init() */
	mapphone_touch_panel_init(&mapphone_i2c_1_boardinfo[0]);
	mapphone_touch_btn_init(&mapphone_i2c_1_boardinfo[1]);

	omap_register_i2c_bus_board_data(1, &mapphone_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &mapphone_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &mapphone_i2c_3_bus_pdata);

	/* Populate I2C bus 1 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			1, mapphone_i2c_bus1_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_1_boardinfo,
			ARRAY_SIZE(mapphone_i2c_1_boardinfo));
	omap_register_i2c_bus(1, 400,
			mapphone_i2c_bus1_board_info, i2c_bus_devices);

	/* Populate I2C bus 2 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			2, mapphone_i2c_bus2_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_2_boardinfo,
			ARRAY_SIZE(mapphone_i2c_2_boardinfo));
	omap_register_i2c_bus(2, 400,
			mapphone_i2c_bus2_board_info, i2c_bus_devices);

	/* Populate I2C bus 3 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			3, mapphone_i2c_bus3_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_3_boardinfo,
			ARRAY_SIZE(mapphone_i2c_3_boardinfo));
	omap_register_i2c_bus(3, 400,
			mapphone_i2c_bus3_board_info, i2c_bus_devices);

	mapphone_akm8973_init();
	mapphone_kxtf9_init();
	mapphone_lm3554_init();
	mapphone_isl29030_init();
	mapphone_bu52014hfv_init();
	platform_device_register(&omap3430_hall_effect_dock);

	mapphone_vibrator_init();
	platform_device_register(&mapphone_vib_gpio);

}

