/*
 * Copyright (C) 2010-2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/types.h>
#include <linux/input/touch_platform.h>
#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/prom.h>
#include <linux/gpio.h>
#include <linux/gpio_mapping.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/qtouch_obp_ts.h>
#include "dt_path.h"
#include <linux/interrupt.h>

void __init mapphone_touch_panel_init(struct i2c_board_info *i2c_info);
void __init mapphone_touch_btn_init(struct i2c_board_info *i2c_info);
static int __init mapphone_touch_driver_settings_init(
		struct touch_platform_data *tpdata,
		struct device_node *dtnode,
		struct i2c_board_info *i2c_info);
static int __init mapphone_touch_ic_settings_init(
		struct touch_platform_data *tpdata,
		struct device_node *dtnode,
		struct i2c_board_info *i2c_info);
static int __init mapphone_touch_firmware_init(
		struct touch_platform_data *tpdata,
		struct device_node *dtnode);
static int __init mapphone_touch_framework_settings_init(
		struct touch_platform_data *tpdata,
		struct device_node *dtnode);
static void __init mapphone_touch_free(
		struct touch_platform_data *tpdata,
		struct device_node *dtnodes[]);
static int __init mapphone_touch_gpio_init(const char *rst_name,
		const char *int_name,
		const char *power_rst_name,
		struct i2c_board_info *i2c_info);
static int __init mapphone_touch_vkeys_init(
		struct device_node *dtnode,
		struct i2c_board_info *i2c_info);
static const uint16_t mapphone_touch_vkeys_map[VKEY_DATA_SIZE];
static int mapphone_touch_vkeys_map_size;
static struct attribute_group mapphone_attribute_group;
static struct attribute *mapphone_attributes[];
static struct kobj_attribute mapphone_touch_vkeys_attr;
static ssize_t mapphone_touch_vkeys_kobj_output(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf);
static int mapphone_touch_reset(const char *rst_name, bool level);
static int mapphone_touch_recovery(const char *rst_name,
		const char *int_name, int mode, bool rst_level);
static int mapphone_touch_recovery_mode0(const char *rst_name,
		const char *int_name, bool rst_level);
static int mapphone_touch_recovery_mode1(void);
static int mapphone_touch_irq_status(const char *int_name);
static struct qtouch_ts_platform_data *qtouch_pdata;
static void __init mapphone_legacy_qtouch_init(struct i2c_board_info *i2c_info);
static void __init mapphone_legacy_qtouch_gpio_init(
		struct i2c_board_info *i2c_info);
static ssize_t mapphone_legacy_qtouch_virtual_keys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf);
static struct kobj_attribute mapphone_legacy_qtouch_virtual_keys_attr;
static struct attribute *mapphone_legacy_qtouch_properties_attrs[];
static struct attribute_group mapphone_legacy_qtouch_properties_attr_group;
static int mapphone_legacy_qtouch_reset(void);
static int mapphone_touch_btn_reset(void);
static int mapphone_touch_btn_recovery(int mode);
static int mapphone_touch_btn_irq_status(void);
static int mapphone_touch_panel_reset(void);
static int mapphone_touch_panel_recovery(int mode);
static int mapphone_touch_panel_irq_status(void);
struct attribute_group *mapphone_touch_vkey_prop_attr_group;

void __init mapphone_touch_panel_init(struct i2c_board_info *i2c_info)
{
	int err = 0;
	struct device_node *dtnodes[5] = {
		NULL, NULL, NULL, NULL, NULL};
	struct touch_platform_data *tpdata = NULL;

	printk(KERN_INFO "%s: Starting touch init...\n", __func__);

	if (i2c_info == NULL) {
		printk(KERN_ERR "%s: NULL i2c_board_info pointer passed.\n",
			__func__);
		goto touch_init_fail;
	}

	dtnodes[0] = of_find_node_by_path("/System@0/I2C@0/Touch@0");
	if (dtnodes[0] == NULL) {
		/* Normally this is a fatal error */
		mapphone_legacy_qtouch_init(i2c_info);  /* Legacy support */
		mapphone_legacy_qtouch_gpio_init(i2c_info);  /* Legacy support */
		/* The properties group register in board-mapphone.c*/
		mapphone_touch_vkey_prop_attr_group =
			&mapphone_legacy_qtouch_properties_attr_group;
		goto legacy_qtouch_complete;  /* Legacy support */
	}

	tpdata = kzalloc(sizeof(struct touch_platform_data), GFP_KERNEL);
	if (tpdata == NULL) {
		printk(KERN_ERR "%s: Unable to create platform data.\n",
			__func__);
		err = -ENOMEM;
		goto touch_init_fail;
	}

	dtnodes[1] = of_find_node_by_path("/System@0/I2C@0/Touch@0/Driver@0");
	if (dtnodes[1] == NULL) {
		printk(KERN_ERR "%s: Driver@0 node is missing.\n", __func__);
		err = -ENOENT;
		goto touch_init_fail;
	} else {
		err = mapphone_touch_driver_settings_init(tpdata, dtnodes[1],
				i2c_info);
		if (err < 0)
			goto touch_init_fail;
	}

	dtnodes[2] = of_find_node_by_path("/System@0/I2C@0/Touch@0/IC@0");
	if (dtnodes[2] == NULL) {
		printk(KERN_ERR "%s: IC@0 node is missing.\n", __func__);
		err = -ENOENT;
		goto touch_init_fail;
	} else {
		err = mapphone_touch_ic_settings_init(tpdata, dtnodes[2],
				i2c_info);
		if (err < 0)
			goto touch_init_fail;
	}

	dtnodes[3] = of_find_node_by_path("/System@0/I2C@0/Touch@0/Firmware@0");
	if (dtnodes[3] == NULL) {
		printk(KERN_ERR "%s: Firmware@0 node is missing.\n", __func__);
		err = -ENOENT;
		goto touch_init_fail;
	} else {
		err = mapphone_touch_firmware_init(tpdata, dtnodes[3]);
		if (err < 0)
			goto touch_init_fail;
	}

	dtnodes[4] =
		of_find_node_by_path("/System@0/I2C@0/Touch@0/Framework@0");
	if (dtnodes[4] == NULL) {
		printk(KERN_ERR "%s: Framework@0 node is missing.\n", __func__);
		err = -ENOENT;
		goto touch_init_fail;
	} else {
		err = mapphone_touch_framework_settings_init(
			tpdata, dtnodes[4]);
		if (err < 0)
			goto touch_init_fail;
	}

	err = mapphone_touch_gpio_init("touch_panel_rst", "touch_panel_int",
						"touch_panel_en", i2c_info);
	if (err < 0)
		goto touch_init_fail;

	if (tpdata->frmwrk->enable_vkeys) {
		err = mapphone_touch_vkeys_init(dtnodes[4], i2c_info);
		if (err < 0)
			goto touch_init_fail;
	}

	tpdata->hw_reset = mapphone_touch_panel_reset;
	tpdata->hw_recov = mapphone_touch_panel_recovery;
	tpdata->irq_stat = mapphone_touch_panel_irq_status;

	i2c_info->platform_data = tpdata;
	goto touch_init_pass;

touch_init_fail:
	mapphone_touch_free(tpdata, &dtnodes[0]);
	printk(KERN_ERR
		"%s: Touch init failed for %s driver with error code %d.\n",
		__func__, i2c_info->type, err);
	return;

touch_init_pass:
	printk(KERN_INFO "%s: Touch init successful for %s driver.\n",
		__func__, i2c_info->type);
legacy_qtouch_complete:  /* Legacy support */
	return;
}

void __init mapphone_touch_btn_init(struct i2c_board_info *i2c_info)
{
	int err = 0;
	struct device_node *dtnodes[5] = {
		NULL, NULL, NULL, NULL, NULL};
	struct touch_platform_data *tpdata = NULL;

	if (i2c_info == NULL) {
		printk(KERN_ERR "%s: NULL i2c_board_info pointer passed.\n",
			__func__);
		goto touch_btn_init_fail;
	}

	dtnodes[0] = of_find_node_by_path("/System@0/I2C@0/Touch@1");
	if (dtnodes[0] == NULL)
		goto touch_btn_init_complete;
	printk(KERN_INFO "%s: Starting touch button init...\n", __func__);

	tpdata = kzalloc(sizeof(struct touch_platform_data), GFP_KERNEL);
	if (tpdata == NULL) {
		printk(KERN_ERR "%s: Unable to create platform data.\n",
			__func__);
		err = -ENOMEM;
		goto touch_btn_init_fail;
	}

	dtnodes[1] = of_find_node_by_path("/System@0/I2C@0/Touch@1/Driver@0");
	if (dtnodes[1] == NULL) {
		printk(KERN_ERR "%s: Driver@0 node is missing.\n", __func__);
		err = -ENOENT;
		goto touch_btn_init_fail;
	} else {
		err = mapphone_touch_driver_settings_init(tpdata, dtnodes[1],
				i2c_info);
		if (err < 0)
			goto touch_btn_init_fail;
	}

	dtnodes[2] = of_find_node_by_path("/System@0/I2C@0/Touch@1/IC@0");
	if (dtnodes[2] == NULL) {
		printk(KERN_ERR "%s: IC@0 node is missing.\n", __func__);
		err = -ENOENT;
		goto touch_btn_init_fail;
	} else {
		err = mapphone_touch_ic_settings_init(tpdata, dtnodes[2],
				i2c_info);
		if (err < 0)
			goto touch_btn_init_fail;
	}

	dtnodes[3] = of_find_node_by_path("/System@0/I2C@0/Touch@1/Firmware@0");
	if (dtnodes[3] == NULL) {
		printk(KERN_ERR "%s: Firmware@0 node is missing.\n", __func__);
		err = -ENOENT;
		goto touch_btn_init_fail;
	} else {
		err = mapphone_touch_firmware_init(tpdata, dtnodes[3]);
		if (err < 0)
			goto touch_btn_init_fail;
	}

	dtnodes[4] =
		of_find_node_by_path("/System@0/I2C@0/Touch@1/Framework@0");
	if (dtnodes[4] == NULL) {
		printk(KERN_ERR "%s: Framework@0 node is missing.\n", __func__);
		err = -ENOENT;
		goto touch_btn_init_fail;
	} else {
		err = mapphone_touch_framework_settings_init(
			tpdata, dtnodes[4]);
		if (err < 0)
			goto touch_btn_init_fail;
	}

	err = mapphone_touch_gpio_init("touch_btn_rst", "touch_btn_int",
						NULL, i2c_info);
	if (err < 0)
		goto touch_btn_init_fail;

	if (tpdata->frmwrk->enable_vkeys) {
		err = mapphone_touch_vkeys_init(dtnodes[4], i2c_info);
		if (err < 0)
			goto touch_btn_init_fail;
	}

	tpdata->hw_reset = mapphone_touch_btn_reset;
	tpdata->hw_recov = mapphone_touch_btn_recovery;
	tpdata->irq_stat = mapphone_touch_btn_irq_status;

	i2c_info->platform_data = tpdata;
	goto touch_btn_init_pass;

touch_btn_init_fail:
	mapphone_touch_free(tpdata, &dtnodes[0]);
	printk(KERN_ERR
		"%s: Touch button init failed for %s driver with error code %d.\n",
		__func__, i2c_info->type, err);
	return;

touch_btn_init_pass:
	printk(KERN_INFO "%s: Touch button init successful for %s driver.\n",
		__func__, i2c_info->type);
touch_btn_init_complete:
	return;
}

static int __init mapphone_touch_driver_settings_init(
		struct touch_platform_data *tpdata,
		struct device_node *dtnode,
		struct i2c_board_info *i2c_info)
{
	int err = 0;
	const void *prop;
	int size = 0;

	prop = of_get_property(dtnode, "touch_driver_name", &size);
	if (prop == NULL || size <= 0) {
		printk(KERN_ERR "%s: Driver name is missing.\n", __func__);
		err = -ENOENT;
		i2c_info->type[0] = '\0';
		goto touch_driver_settings_init_fail;
	} else if (size >= I2C_NAME_SIZE) {
		printk(KERN_ERR "%s: Driver name is too long.\n", __func__);
		err = -ENAMETOOLONG;
		i2c_info->type[0] = '\0';
		goto touch_driver_settings_init_fail;
	}

	strncpy((char *) &i2c_info->type, (char *) prop, I2C_NAME_SIZE);
	i2c_info->type[I2C_NAME_SIZE-1] = '\0';

	prop = of_get_property(dtnode, "touch_driver_flags", &size);
	if (prop != NULL && size > 0)
		tpdata->flags = *((uint16_t *) prop);
	else
		tpdata->flags = 0;

touch_driver_settings_init_fail:
	return err;
}

static int __init mapphone_touch_ic_settings_init(
		struct touch_platform_data *tpdata,
		struct device_node *dtnode,
		struct i2c_board_info *i2c_info)
{
	int err = 0;
	const void *prop;
	int size = 0;
	const void *list_prop;
	int list_size = 0;
	char *prop_name = NULL;
	char *str_num = NULL;
	const char *str1 = "tsett";
	const char *str2 = "_tag";
	uint8_t tsett_num;
	int i = 0;

	prop = of_get_property(dtnode, "i2c_addrs", &size);
	if (prop == NULL || size <= 0) {
		printk(KERN_ERR "%s: I2C address data is missing.\n",
			__func__);
		err = -ENOENT;
		goto touch_ic_settings_init_fail;
	} else if (size > ARRAY_SIZE(tpdata->addr)) {
		printk(KERN_ERR "%s: Too many I2C addresses provided.\n",
			__func__);
		err = -E2BIG;
		goto touch_ic_settings_init_fail;
	}

	for (i = 0; i < size; i++)
		tpdata->addr[i] = ((uint8_t *) prop)[i];

	for (i = size; i < ARRAY_SIZE(tpdata->addr); i++)
		tpdata->addr[i] = 0;

	i2c_info->addr = (unsigned short) tpdata->addr[0];

	list_prop = of_get_property(dtnode, "tsett_list", &list_size);
	if (list_prop == NULL || size <= 0) {
		printk(KERN_ERR "%s: No settings list provided.\n", __func__);
		err = -ENOENT;
		goto touch_ic_settings_init_fail;
	}

	prop_name = kzalloc(strlen(str1) + (sizeof(char) * 4) +
					strlen(str2), GFP_KERNEL);
	if (prop_name == NULL) {
		printk(KERN_ERR "%s: No memory for prop_name.\n", __func__);
		err = -ENOMEM;
		goto touch_ic_settings_init_fail;
	}

	str_num = kzalloc(sizeof(char) * 4, GFP_KERNEL);
	if (str_num == NULL) {
		printk(KERN_ERR "%s: No memory for str_num.\n", __func__);
		err = -ENOMEM;
		goto touch_ic_settings_init_fail;
	}

	for (i = 0; i < ARRAY_SIZE(tpdata->sett); i++)
		tpdata->sett[i] = NULL;

	for (i = 0; i < list_size; i++) {
		tsett_num = ((uint8_t *) list_prop)[i];
		err = snprintf(str_num, 3, "%hu", tsett_num);
		if (err < 0) {
			printk(KERN_ERR
				"%s: Error in snprintf converting %hu.\n",
				__func__, tsett_num);
			goto touch_ic_settings_init_fail;
		}
		prop_name[0] = '\0';
		strncat(prop_name, str1, strlen(str1));
		strncat(prop_name, str_num, 3);

		prop = of_get_property(dtnode, prop_name, &size);
		if (prop == NULL || size <= 0) {
			printk(KERN_ERR
				"%s: Entry %s from tsett_list is missing.\n",
				__func__, prop_name);
			err = -ENOENT;
			goto touch_ic_settings_init_fail;
		}

		tpdata->sett[tsett_num] =
			kzalloc(sizeof(struct touch_settings), GFP_KERNEL);
		if (tpdata->sett[tsett_num] == NULL) {
			printk(KERN_ERR
				"%s: Unable to create tsett node %s.\n",
				__func__, str_num);
			err = -ENOMEM;
			goto touch_ic_settings_init_fail;
		}

		tpdata->sett[tsett_num]->data = (const uint8_t *) prop;
		tpdata->sett[tsett_num]->size =
			(uint8_t) (size / sizeof(uint8_t));

		strncat(prop_name, str2, strlen(str2));
		prop = of_get_property(dtnode, prop_name, &size);
		if (prop != NULL && size > 0)
			tpdata->sett[tsett_num]->tag = *((uint8_t *) prop);
		else
			tpdata->sett[tsett_num]->tag = 0;
	}

touch_ic_settings_init_fail:
	kfree(prop_name);
	kfree(str_num);

	return err;
}

static int __init mapphone_touch_firmware_init(
		struct touch_platform_data *tpdata,
		struct device_node *dtnode)
{
	int err = 0;
	const void *prop;
	int size = 0;

	tpdata->fw =
		kzalloc(sizeof(struct touch_firmware), GFP_KERNEL);
	if (tpdata->fw == NULL) {
		printk(KERN_ERR "%s: Unable to create fw node.\n", __func__);
		err = -ENOMEM;
		goto touch_firmware_init_fail;
	}

	prop = of_get_property(dtnode, "fw_image", &size);
	if (prop == NULL || size <= 0) {
		printk(KERN_ERR "%s: Firmware image is missing.\n", __func__);
		err = -ENOENT;
		goto touch_firmware_init_fail;
	} else {
		tpdata->fw->img = (const uint8_t *) prop;
		tpdata->fw->size = (uint32_t) (size / sizeof(uint8_t));
	}

	prop = of_get_property(dtnode, "fw_version", &size);
	if (prop == NULL || size <= 0) {
		printk(KERN_ERR "%s: Firmware version is missing.\n", __func__);
		err = -ENOENT;
		goto touch_firmware_init_fail;
	} else {
		tpdata->fw->ver = (const uint8_t *) prop;
		tpdata->fw->vsize = (uint8_t) (size / sizeof(uint8_t));
	}

touch_firmware_init_fail:
	return err;
}

static int __init mapphone_touch_framework_settings_init(
		struct touch_platform_data *tpdata,
		struct device_node *dtnode)
{
	int err = 0;
	const void *prop;
	int size = 0;

	tpdata->frmwrk =
		kzalloc(sizeof(struct touch_framework), GFP_KERNEL);
	if (tpdata->frmwrk == NULL) {
		printk(KERN_ERR "%s: Unable to create frmwrk node.\n",
			__func__);
		err = -ENOMEM;
		goto touch_framework_settings_fail;
	}

	prop = of_get_property(dtnode, "abs_params", &size);
	if (prop == NULL || size <= 0) {
		printk(KERN_ERR "%s: Abs parameters are missing.\n", __func__);
		err = -ENOENT;
		goto touch_framework_settings_fail;
	} else if (size > ABS_DATA_SIZE) {
		printk(KERN_ERR "%s: Abs parameters exceed size.\n", __func__);
		err = -ENOENT;
		goto touch_framework_settings_fail;
	} else {
		memcpy((void *)tpdata->frmwrk->abs, prop, size);
		tpdata->frmwrk->size = (uint8_t) (size / sizeof(uint16_t));
	}

	prop = of_get_property(dtnode, "enable_touch_vkeys", &size);
	if (prop == NULL || size <= 0) {
		printk(KERN_ERR "%s: Vkeys flag is missing.\n", __func__);
		err = -ENOENT;
		goto touch_framework_settings_fail;
	} else {
		tpdata->frmwrk->enable_vkeys = *((uint8_t *) prop);
	}

touch_framework_settings_fail:
	return err;
}

static void __init mapphone_touch_free(
		struct touch_platform_data *tpdata,
		struct device_node *dtnodes[])
{
	int i = 0;

	if (tpdata != NULL) {
		for (i = 0; i < ARRAY_SIZE(tpdata->sett); i++) {
			if (tpdata->sett[i] != NULL) {
				kfree(tpdata->sett[i]);
				tpdata->sett[i] = NULL;
			}
		}

		if (tpdata->fw != NULL) {
			kfree(tpdata->fw);
			tpdata->fw = NULL;
		}

		if (tpdata->frmwrk != NULL) {
			kfree(tpdata->frmwrk);
			tpdata->frmwrk = NULL;
		}

		kfree(tpdata);
		tpdata = NULL;
	}

	for (i = 0; i < 5; i++) {
		if (dtnodes[i] != NULL) {
			of_node_put(dtnodes[i]);
			dtnodes[i] = NULL;
		}
	}

	return;
}

static int __init mapphone_touch_gpio_init(const char *rst_name,
				const char *int_name,
				const char *power_rst_name,
				struct i2c_board_info *i2c_info)
{
	int err = 0;
	int pin = 0;

	pin = get_gpio_by_name(rst_name);
	if (pin >= 0) {
		err = gpio_request(pin, rst_name);
		if (err >= 0) {
			err = gpio_direction_output(pin, 0);
			if (err < 0) {
				printk(KERN_ERR "%s: Unable to config reset.\n",
						__func__);
				goto touch_gpio_init_fail;
			}
		} else {
			printk(KERN_ERR "%s: Reset GPIO request failed.\n",
					__func__);
			goto touch_gpio_init_fail;
		}
	} else {
		printk(KERN_ERR "%s: Cannot acquire reset pin.\n", __func__);
		err = pin;
		goto touch_gpio_init_fail;
	}

	pin = get_gpio_by_name(int_name);
	if (pin >= 0) {
		err = gpio_request(pin, int_name);
		if (err >= 0) {
			err = gpio_direction_input(pin);
			if (err < 0) {
				printk(KERN_ERR "%s: Unable to config irq.\n",
						__func__);
				goto touch_gpio_init_fail;
			}
		} else {
			printk(KERN_ERR "%s: IRQ GPIO request failed.\n",
					__func__);
			goto touch_gpio_init_fail;
		}
	} else {
		printk(KERN_ERR "%s: Cannot acquire irq pin.\n", __func__);
		err = pin;
		goto touch_gpio_init_fail;
	}
	i2c_info->irq = gpio_to_irq(pin);

	/*
	* when dinara has the ESD effect, it may not recovery even if use the
	* hardware reset. so add a specifical GPIO to control the touch_ic's
	* reset.
	*/
	if (power_rst_name == NULL)
		goto touch_gpio_init_fail;

	pin = get_gpio_by_name(power_rst_name);
	if (pin >= 0) {
		err = gpio_request(pin, power_rst_name);
		if (err >= 0) {
			/* Set high level to enable power */
			gpio_direction_output(pin, 1);
		} else {
			printk(KERN_ERR "%s: Power reset GPIO request failed.\n",
					__func__);
		}
	} else {
		printk(KERN_ERR "%s: Cannot acquire power reset pin.\n",
			__func__);
	}

touch_gpio_init_fail:
	return err;
}

static int __init mapphone_touch_vkeys_init(
		struct device_node *dtnode,
		struct i2c_board_info *i2c_info)
{
	int err = 0;
	const void *prop;
	int size = 0;
	const char *str1 = "virtualkeys.";
	char *filename = NULL;

	prop = of_get_property(dtnode, "vkey_map", &size);
	if (prop == NULL || size <= 0) {
		printk(KERN_ERR "%s: Virtual keymap is missing.\n", __func__);
		err = -ENOENT;
		goto touch_vkeys_init_fail;
	} else if (size > VKEY_DATA_SIZE) {
		printk(KERN_ERR "%s: Virtual keymap exceed the size.\n",
				__func__);
		err = -ENOENT;
		goto touch_vkeys_init_fail;
	} else {
		memcpy((void *)mapphone_touch_vkeys_map, prop, size);
		mapphone_touch_vkeys_map_size = (size / sizeof(uint16_t));
	}

	filename = kzalloc(strlen(str1) + (sizeof(char) * (I2C_NAME_SIZE + 1)),
				GFP_KERNEL);
	if (filename == NULL) {
		printk(KERN_ERR "%s: No memory for filename.\n", __func__);
		err = -ENOMEM;
		goto touch_vkeys_init_fail;
	}
	strncat(filename, str1, strlen(str1));
	strncat(filename, i2c_info->type, I2C_NAME_SIZE);
	mapphone_touch_vkeys_attr.attr.name = filename;
	/* The properties group register in board-44xx-identity.c*/
	mapphone_touch_vkey_prop_attr_group = &mapphone_attribute_group;

touch_vkeys_init_fail:
	return err;
}


static struct attribute_group mapphone_attribute_group = {
	.attrs = mapphone_attributes,
};

static struct attribute *mapphone_attributes[] = {
	&mapphone_touch_vkeys_attr.attr,
	NULL,
};

static struct kobj_attribute mapphone_touch_vkeys_attr = {
	.attr = {
		.name = NULL,
		.mode = S_IRUGO,
	},
	.show = &mapphone_touch_vkeys_kobj_output,
};

static ssize_t mapphone_touch_vkeys_kobj_output(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int err = 0;
	const uint16_t *map;
	int size = 0;
	int i = 0;

	map = mapphone_touch_vkeys_map;
	size = mapphone_touch_vkeys_map_size;

	if (map == NULL) {
		printk(KERN_ERR "%s Virtual keymap is missing.\n", __func__);
		err = -ENODATA;
		goto touch_vkeys_kobj_output_fail;
	}

	if ((size <= 0) || ((size % 5) != 0)) {
		printk(KERN_ERR "%s: Virtual keymap is invalid.\n", __func__);
		err = -EINVAL;
		goto touch_vkeys_kobj_output_fail;
	}

	for (i = 0; i < size; i++) {
		if (i % 5 == 0) {
			err = sprintf(buf, "%s" __stringify(EV_KEY) ":", buf);
			if (err < 0) {
				printk(KERN_ERR
					"%s: Error in ev_key sprintf %d.\n",
					__func__, i);
				goto touch_vkeys_kobj_output_fail;
			}
		}

		err = sprintf(buf, "%s%hu:", buf, map[i]);
		if (err < 0) {
			printk(KERN_ERR "%s: Error in sprintf %d.\n",
				__func__, i);
			goto touch_vkeys_kobj_output_fail;
		}
	}

	buf[err-1] = '\n';
	buf[err] = '\0';

touch_vkeys_kobj_output_fail:
	return (ssize_t) err;
}

static int mapphone_touch_reset(const char *rst_name, bool level)
{
	int err = 0;
	int reset_pin = 0;

	reset_pin = get_gpio_by_name(rst_name);
	if (reset_pin < 0) {
		printk(KERN_ERR "%s: Cannot acquire reset pin.\n", __func__);
		err = reset_pin;
	} else {
		gpio_set_value(reset_pin, level ? 1 : 0);
		msleep(9);
		gpio_set_value(reset_pin, level ? 0 : 1);
	}

	return err;
}

static int mapphone_touch_recovery(const char *rst_name, const char *int_name,
	int mode, bool rst_level)
{
	int err = 0;

	switch (mode) {
	case 0:
		err = mapphone_touch_recovery_mode0(rst_name, int_name,
			rst_level);
		break;
	case 1:
		err = mapphone_touch_recovery_mode1();
		break;
	default:
		err = -EINVAL;
		printk(KERN_ERR "%s: Recovery mode %d is invalid.\n",
			__func__, mode);
		goto touch_recovery_fail_default;
	}

	if (err < 0) {
		printk(KERN_ERR "%s: Recovery mode %d failed.\n",
			__func__, mode);
	}

touch_recovery_fail_default:
	return err;
}

static int mapphone_touch_recovery_mode0(const char *rst_name,
			const char *int_name, bool rst_level)
{
	int err = 0;
	int pin = 0;

	pin = get_gpio_by_name(int_name);
	if (pin < 0) {
		printk(KERN_ERR "%s: Cannot acquire irq pin.\n", __func__);
		err = pin;
		goto touch_recovery_mode0_fail;
	}

	err = gpio_direction_output(pin, 0);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to switch irq to output.\n",
			__func__);
		goto touch_recovery_mode0_fail;
	}

	err = mapphone_touch_reset(rst_name, rst_level);
	if (err < 0) {
		printk(KERN_ERR "%s: Hardware reset failed.\n", __func__);
		goto touch_recovery_mode0_fail;
	}

	msleep(80);

	err = gpio_direction_input(pin);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to restore irq to input.\n",
			__func__);
		goto touch_recovery_mode0_fail;
	}

touch_recovery_mode0_fail:
	return err;
}

static int mapphone_touch_recovery_mode1(void)
{
	int retval = 0;
	int int_pin = 0;
	const char *int_name = "touch_panel_int";

	int_pin = get_gpio_by_name(int_name);
	if (int_pin >= 0) {
		retval = gpio_direction_output(int_pin, 1);
		if (retval < 0) {
			pr_err("%s: Failed to switch irq to output.\n",
				__func__);
		}
		gpio_set_value(int_pin, 0);
		msleep(20);
		gpio_set_value(int_pin, 1);
		retval = gpio_direction_input(int_pin);
		if (retval < 0) {
			pr_err("%s: Failed to restore irq to input.\n",
				__func__);
		}
	} else {
		pr_err("%s: Cannot acquire irq pin.\n", __func__);
	}

	return retval;
}

static int mapphone_touch_irq_status(const char *int_name)
{
	int err = 0;
	int irq_pin = 0;

	irq_pin = get_gpio_by_name(int_name);
	if (irq_pin < 0) {
		printk(KERN_ERR "%s: Cannot acquire irq pin.\n", __func__);
		err = irq_pin;
	} else {
		err = gpio_get_value(irq_pin);
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to read irq level.\n",
				__func__);
		}
	}

	return err;
}

static int mapphone_touch_btn_reset(void)
{
	return mapphone_touch_reset("touch_btn_rst", 1);
}

static int mapphone_touch_btn_irq_status(void)
{
	return mapphone_touch_irq_status("touch_btn_int");
}

static int mapphone_touch_btn_recovery(int mode)
{
	return mapphone_touch_recovery("touch_btn_rst", "touch_btn_int",
			mode, 1);
}

static int mapphone_touch_panel_reset(void)
{
	return mapphone_touch_reset("touch_panel_rst", 0);
}

static int mapphone_touch_panel_irq_status(void)
{
	return mapphone_touch_irq_status("touch_panel_int");
}

static int mapphone_touch_panel_recovery(int mode)
{
	return mapphone_touch_recovery("touch_panel_rst", "touch_panel_int",
			mode, 0);
}

static void __init mapphone_legacy_qtouch_init(struct i2c_board_info *i2c_info)
{
	int len = 0;
	struct device_node *touch_node;
	const void *touch_prop;
	const uint32_t *touch_val;

	printk(KERN_INFO "%s: Selecting legacy qtouch driver.\n",
		__func__);

	touch_node = of_find_node_by_path(DT_PATH_TOUCH);
	if (touch_node == NULL) {
		printk(KERN_INFO "%s: No device tree data available.\n",
			__func__);
		goto mapphone_legacy_qtouch_init_ret;
	}

	qtouch_pdata = kzalloc(sizeof(struct qtouch_ts_platform_data), GFP_KERNEL);
	if (qtouch_pdata == NULL) {
		printk(KERN_ERR "%s: Unable to create platform data.\n",
			__func__);
		goto mapphone_legacy_qtouch_init_err;
	}
	/* init fields with common configurations for all products */
	qtouch_pdata->flags = QTOUCH_USE_MULTITOUCH |
			   QTOUCH_CFG_BACKUPNV |
			   QTOUCH_EEPROM_CHECKSUM;
	qtouch_pdata->irqflags = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW;
	qtouch_pdata->hw_reset = mapphone_legacy_qtouch_reset;

	/*
	 * To be backward compatible, use default values for x_delta
	 * and y_delta if no valid configs available in devtree.
	 */
	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_X_DELTA, &len);
	if (touch_val && len)
		qtouch_pdata->x_delta = *touch_val;
	else
		qtouch_pdata->x_delta = 400;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_Y_DELTA, &len);
	if (touch_val && len)
		qtouch_pdata->y_delta = *touch_val;
	else
		qtouch_pdata->y_delta = 250;

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_KEYMAP, &len);
	if (touch_prop && len && (0 == len % sizeof(struct vkey))) {
		qtouch_pdata->vkeys.count = len / sizeof(struct vkey);
		qtouch_pdata->vkeys.keys = (struct vkey *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_I2C_ADDRESS,
		&len);
	if (touch_prop)
		i2c_info->addr = *((int *)touch_prop);
	else
		i2c_info->addr = 0x11;
	strncpy((char *) &i2c_info->type, QTOUCH_TS_NAME, I2C_NAME_SIZE);
	i2c_info->type[I2C_NAME_SIZE-1] = '\0';

	touch_prop = of_get_property(touch_node,
		DT_PROP_TOUCH_BOOT_I2C_ADDRESS, &len);
	if (touch_prop)
		qtouch_pdata->boot_i2c_addr = *((int *)touch_prop);

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_CHECKSUM, &len);
	if (touch_val && len)
		qtouch_pdata->nv_checksum = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_FLAGS, &len);
	if (touch_val && len)
		qtouch_pdata->flags = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MIN_X, &len);
	if (touch_val && len)
		qtouch_pdata->abs_min_x = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MAX_X, &len);
	if (touch_val && len)
		qtouch_pdata->abs_max_x = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MIN_Y, &len);
	if (touch_val && len)
		qtouch_pdata->abs_min_y = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MAX_Y, &len);
	if (touch_val && len)
		qtouch_pdata->abs_max_y = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MIN_P, &len);
	if (touch_val && len)
		qtouch_pdata->abs_min_p = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MAX_P, &len);
	if (touch_val && len)
		qtouch_pdata->abs_max_p = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MIN_W, &len);
	if (touch_val && len)
		qtouch_pdata->abs_min_w = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MAX_W, &len);
	if (touch_val && len)
		qtouch_pdata->abs_max_w = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_FUZZ_X, &len);
	if (touch_val && len)
		qtouch_pdata->fuzz_x = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_FUZZ_Y, &len);
	if (touch_val && len)
		qtouch_pdata->fuzz_y = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_FUZZ_P, &len);
	if (touch_val && len)
		qtouch_pdata->fuzz_p = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_FUZZ_W, &len);
	if (touch_val && len)
		qtouch_pdata->fuzz_w = *touch_val;

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T15, &len);
	if (touch_prop) {
		qtouch_pdata->key_array.cfg =
			(struct qtm_touch_keyarray_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_KEY_ARRAY_MAP,
		&len);
	if (touch_prop) {
		qtouch_pdata->key_array.keys = (struct qtouch_key *)touch_prop;
	}

	touch_val = of_get_property(touch_node,
		DT_PROP_TOUCH_KEY_ARRAY_COUNT, &len);
	if (touch_val && len)
		qtouch_pdata->key_array.num_keys = *touch_val;

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T7, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->power_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->power_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T8, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->acquire_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->acquire_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T9, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->multi_touch_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->multi_touch_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T17, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->linear_tbl_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->linear_tbl_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T18, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->comms_config_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->comms_config_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T19, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->gpio_pwm_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->gpio_pwm_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T20, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->grip_suppression_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->grip_suppression_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T22, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->noise_suppression_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->noise_suppression_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T23, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->touch_proximity_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->touch_proximity_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T24, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->one_touch_gesture_proc_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->one_touch_gesture_proc_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T25, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->self_test_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->self_test_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T27, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->two_touch_gesture_proc_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->two_touch_gesture_proc_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T28, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->cte_config_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->cte_config_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T36, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->noise1_suppression_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->noise1_suppression_cfg), len));
	}
#if 0
	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T38, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->userdata, touch_prop,
		       min((int)sizeof(qtouch_pdata->userdata), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T40, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->grip_suppression2_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->grip_suppression2_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T41, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->palm_suppression_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->palm_suppression_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T42, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->touch_suppression2_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->touch_suppression2_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T43, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->spt_digitizer_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->spt_digitizer_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T46, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->cte_config2_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->cte_config2_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T47, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->stylus_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->stylus_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T48, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->noise_suppression2_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->noise_suppression2_cfg), len));
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T54, &len);
	if (touch_prop && len) {
		memcpy(&qtouch_pdata->noise_suppression3_cfg, touch_prop,
		       min((int)sizeof(qtouch_pdata->noise_suppression3_cfg), len));
	}
#endif
	i2c_info->platform_data = qtouch_pdata;
	goto mapphone_legacy_qtouch_init_ret;

mapphone_legacy_qtouch_init_err:
	of_node_put(touch_node);
mapphone_legacy_qtouch_init_ret:
	return;
}

static void __init mapphone_legacy_qtouch_gpio_init(
		struct i2c_board_info *i2c_info)
{
	int retval = 0;
	int pin = 0;
	int err = 0;

	pin = get_gpio_by_name("touch_panel_rst");
	if (pin >= 0) {
		err = gpio_request(pin, "touch_reset");
		if (err >= 0) {
			err = gpio_direction_output(pin, 1);
			if (err < 0) {
				printk(KERN_ERR "%s: Unable to config reset.\n",
						__func__);
				retval = err;
				goto legacy_qtouch_gpio_init_fail;
			}
		} else {
			printk(KERN_ERR "%s: Reset GPIO request failed.\n",
					__func__);
			retval = err;
			goto legacy_qtouch_gpio_init_fail;
		}
	} else {
		printk(KERN_ERR "%s: Cannot acquire reset pin.\n", __func__);
		retval = pin;
		goto legacy_qtouch_gpio_init_fail;
	}

	pin = get_gpio_by_name("touch_panel_int");
	if (pin >= 0) {
		err = gpio_request(pin, "touch_irq");
		if (err >= 0) {
			err = gpio_direction_input(pin);
			if (err < 0) {
				printk(KERN_ERR "%s: Unable to config irq.\n",
						__func__);
				retval = err;
				goto legacy_qtouch_gpio_init_fail;
			}
		} else {
			printk(KERN_ERR "%s: IRQ GPIO request failed.\n",
					__func__);
			retval = err;
			goto legacy_qtouch_gpio_init_fail;
		}
	} else {
		printk(KERN_ERR "%s: Cannot acquire irq pin.\n", __func__);
		goto legacy_qtouch_gpio_init_fail;
	}
	i2c_info->irq = gpio_to_irq(pin);

	pin = get_gpio_by_name("touch_panel_wake");
	if (pin >= 0) {
		printk(KERN_INFO "%s: Acquire wake pin.\n", __func__);
		err = gpio_request(pin, "touch_wake");
		if (err >= 0) {
			err = gpio_direction_output(pin, 1);
			if (err < 0) {
				printk(KERN_ERR "%s: Unable to config wake.\n",
						__func__);
				retval = err;
				goto legacy_qtouch_gpio_init_fail;
			}
		} else {
			printk(KERN_ERR "%s: Wake GPIO request failed.\n",
					__func__);
			retval = err;
			goto legacy_qtouch_gpio_init_fail;
		}
	}

legacy_qtouch_gpio_init_fail:
	if (retval < 0) {
		printk(KERN_ERR "%s: GPIO init failed with error code %d.\n",
				__func__, retval);
	}
	return;
}

static ssize_t mapphone_legacy_qtouch_virtual_keys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int i;
	int string_loc = 0;
	int num_chars;

	if (qtouch_pdata == NULL) {
		pr_err("%s: qtouch platform data unavailable !", __func__);
		string_loc = sprintf(buf, "qtouch platform data unavailable!");
		goto qtouch_virtual_keys_show_ret;
	}

	for (i = 0; i < qtouch_pdata->vkeys.count; i++) {
		if (i != 0) {
			num_chars = sprintf((buf + string_loc), ":");
			string_loc += num_chars;
		}

		num_chars = sprintf((buf + string_loc),
			__stringify(EV_KEY) ":%d:%d:%d:%d:%d",
			qtouch_pdata->vkeys.keys[i].code,
			qtouch_pdata->vkeys.keys[i].center_x,
			qtouch_pdata->vkeys.keys[i].center_y,
			qtouch_pdata->vkeys.keys[i].width,
			qtouch_pdata->vkeys.keys[i].height);
		string_loc += num_chars;
	}

qtouch_virtual_keys_show_ret:
	sprintf((buf + string_loc), "\n");

	return string_loc;
}

static struct kobj_attribute mapphone_legacy_qtouch_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.qtouch-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &mapphone_legacy_qtouch_virtual_keys_show,
};

static struct attribute *mapphone_legacy_qtouch_properties_attrs[] = {
	&mapphone_legacy_qtouch_virtual_keys_attr.attr,
	NULL,
};

static struct attribute_group mapphone_legacy_qtouch_properties_attr_group = {
	.attrs = mapphone_legacy_qtouch_properties_attrs,
};

static int mapphone_legacy_qtouch_reset(void)
{
	int reset_pin;
	int retval = 0;

	reset_pin = get_gpio_by_name("touch_panel_rst");
	if (reset_pin < 0) {
		printk(KERN_ERR "%s: Cannot acquire reset pin.\n", __func__);
		retval = reset_pin;
	} else {
		gpio_direction_output(reset_pin, 1);
		msleep(1);
		gpio_set_value(reset_pin, 0);
		msleep(QTM_OBP_SLEEP_RESET_HOLD);
		gpio_set_value(reset_pin, 1);
	}

	return retval;
}
