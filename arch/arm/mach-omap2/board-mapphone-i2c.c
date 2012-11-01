/*
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/led-lm3530.h>

#include <plat/common.h>
#include <mach/board-mapphone.h>

#include "dt_path.h"

#define MAPPHONE_LM_3530_INT_GPIO	92

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
#ifdef CONFIG_ARM_OF
	struct device_node *als_node;
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
		mapphone_i2c_bus1_master_board_info[1].irq =
				 OMAP_GPIO_IRQ(lm3530_int_gpio);
		mapphone_i2c_bus2_master_board_info[3].irq =
				 OMAP_GPIO_IRQ(lm3530_int_gpio);
	}
	lm3530_reset_gpio = get_gpio_by_name("lm3530_reset");
	if (lm3530_int_gpio >= 0) {
		gpio_request(lm3530_reset_gpio, "LED reset");
		gpio_direction_output(lm3530_reset_gpio, 1);
		msleep(10);
	}
#endif
	printk(KERN_INFO "%s:Initializing\n", __func__);
	gpio_request(lm3530_int_gpio, "mapphone als int");
	gpio_direction_input(lm3530_int_gpio);
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
	mapphone_i2c_bus1_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus2_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus3_board_info[I2C_BUS_MAX_DEVICES];

static struct i2c_board_info __initdata
	mapphone_i2c_1_boardinfo[] = {
	{
		I2C_BOARD_INFO(LD_LM3530_NAME, 0x38),
		.platform_data = &omap3430_als_light_data,
		.irq = OMAP_GPIO_IRQ(MAPPHONE_LM_3530_INT_GPIO),
	},
};

static struct i2c_board_info __initdata
	mapphone_i2c_2_boardinfo[] = {

};
static struct i2c_board_info __initdata
	mapphone_i2c_3_boardinfo[] = {

};
static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata mapphone_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata mapphone_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata mapphone_i2c_3_bus_pdata;

void __init mapphone_i2c_init(void)
{
	int i2c_bus_devices = 0;

	omap_i2c_hwspinlock_init(1, 0, &mapphone_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &mapphone_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &mapphone_i2c_3_bus_pdata);

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

}

