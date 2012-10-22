/*
 * board-mapphone-gpio.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* Date	 Author	  Comment
 * ===========  ==============  ==============================================
 * Jun-23-2009  Motorola	Initial revision.
 */

#include <linux/module.h>
#include <mach/gpio.h>

#ifdef CONFIG_GPIODEV
#include <linux/gpiodev.h>
#endif

#ifdef CONFIG_GPIO_MAPPING
#include <linux/gpio_mapping.h>
#endif

#include "dt_path.h"
#include <linux/of.h>

#ifdef CONFIG_GPIODEV
#define GPIO_DEVICE_SIZE 20
#define GPIO_DEVICE_UNUSED 0xFFFF

static struct gpio_device gpio_devs[GPIO_DEVICE_SIZE] = {
	{
		111,
		"slide_interrupt",
		GPIODEV_CONFIG_INPUT | GPIODEV_CONFIG_INT_LLEV,
		GPIODEV_CONFIG_INVALID,
		GPIODEV_FLAG_CONFIGURABLE | GPIODEV_FLAG_LOWLEVELACCESS,
	},
	{
		149,
		"gps_rts",
		GPIODEV_CONFIG_OUTPUT_LOW,
		GPIODEV_CONFIG_INVALID,
		GPIODEV_FLAG_CONFIGURABLE | GPIODEV_FLAG_LOWLEVELACCESS,
	},
	{
		59,
		"gps_reset",
		GPIODEV_CONFIG_OUTPUT_LOW,
		GPIODEV_CONFIG_INVALID,
		GPIODEV_FLAG_CONFIGURABLE | GPIODEV_FLAG_LOWLEVELACCESS,
	},
	{
		136,
		"gps_standby",
		GPIODEV_CONFIG_OUTPUT_LOW,
		GPIODEV_CONFIG_INVALID,
		GPIODEV_FLAG_CONFIGURABLE | GPIODEV_FLAG_LOWLEVELACCESS,
	},
	{
		160,
		"gps_interrupt",
		GPIODEV_CONFIG_INPUT | GPIODEV_CONFIG_INT_REDG,
		GPIODEV_CONFIG_INVALID,
		GPIODEV_FLAG_CONFIGURABLE | GPIODEV_FLAG_LOWLEVELACCESS,
	},
	{
		GPIO_DEVICE_UNUSED,
	},
};

static struct gpio_device_platform_data gpio_device_data = {
	.name = "mapphone-gpiodev",
	.info = gpio_devs,
};

static struct platform_device mapphone_gpiodev_device = {
	.name = GPIO_DEVICE_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data = &gpio_device_data,
	},
};

/**
 * Below structure definition should strictly comform to corresponding
 * HW device tree format
 */
struct omap_gpiodev_entry {
	u32 pin_num;				/* GPIO pin number  */
	char name[GPIO_DEVICE_NAME_LEN];	/* GPIODev name */
	u32 setting;				/* GPIO pin setting */
} __attribute__ ((__packed__));

static void gpiodev_devs_init(void *p_data)
{
	struct omap_gpiodev_entry *p = p_data;
	struct gpio_device *p_devs = gpio_devs;
	int i = 0;

	while ((i < GPIO_DEVICE_NAME_LEN - 1) && (' ' != p->name[i]))
		i++;
	p->name[i] = '\0';

	for (i = 0; i < GPIO_DEVICE_SIZE; i++) {
		if (p_devs[i].pin_nr == GPIO_DEVICE_UNUSED) {
			p_devs[i].pin_nr = p->pin_num;
			strcpy(p_devs[i].device_name, p->name);
			p_devs[i].init_config = p->setting;
			p_devs[i].current_config = GPIODEV_CONFIG_INVALID;
			p_devs[i].flags = GPIODEV_FLAG_CONFIGURABLE |
						GPIODEV_FLAG_LOWLEVELACCESS;

			if (i != (GPIO_DEVICE_SIZE - 1))
				p_devs[i + 1].pin_nr = GPIO_DEVICE_UNUSED;

			pr_debug("GPIODev: Add new device [%s] setting!\n",
					p->name);
			return;
		}

		if (strncmp(p_devs[i].device_name, p->name,
			GPIO_DEVICE_NAME_LEN) == 0) {
			p_devs[i].pin_nr = p->pin_num;
			p_devs[i].init_config = p->setting;
			p_devs[i].current_config = GPIODEV_CONFIG_INVALID;
			p_devs[i].flags = GPIODEV_FLAG_CONFIGURABLE |
						GPIODEV_FLAG_LOWLEVELACCESS;

			pr_debug("GPIODev: Overwrite device [%s] setting!\n",
					p->name);
			return;
		}

		if (i == (GPIO_DEVICE_SIZE - 1))
			pr_err("GPIODev: Too big gpiodev count!\n");
	}
}

static void gpio_devs_of_init(void)
{
	int size, unit_size, i, count;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_GPIOGEV);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
				DT_PATH_GPIOGEV);
		return;
	}

	unit_size = sizeof(struct omap_gpiodev_entry);
	prop = of_get_property(node, DT_PROP_GPIODEV_INIT, &size);
	if ((!prop) || (size % unit_size)) {
		pr_err("Read property %s error!\n", DT_PROP_GPIODEV_INIT);
		of_node_put(node);
		return;
	}

	count = size / unit_size;
	pr_debug("gpio_dev_size = %d\n", count);

	for (i = 0; i < count; i++)
		gpiodev_devs_init((struct omap_gpiodev_entry *)prop + i);

	of_node_put(node);
	return;
}

static int __init mapphone_init_gpiodev(void)
{
	int i;

	gpio_devs_of_init();

	for (i = 0; i < GPIO_DEVICE_SIZE; i++) {
		if (gpio_devs[i].pin_nr == GPIO_DEVICE_UNUSED)
			break;
	}
	gpio_device_data.info_count = i;

	return platform_device_register(&mapphone_gpiodev_device);
}
device_initcall(mapphone_init_gpiodev);
#endif

#ifdef CONFIG_GPIO_MAPPING
#define GPIO_MAP_SIZE 50

static struct gpio_mapping gpio_map_table[GPIO_MAP_SIZE] = {
	{1, 92, "lcd_panel_reset"},
	{1, 93, "lcd_panel_sd"},
	{1, 149, "usb_ipc_phy_reset"},
	{1, 164, "touch_panel_reset"},
	{1, 163, "mmc_detect"},
	{1, 177, "slider_data"},
	{1, 65, "wlan_host_wake"},
};

struct omap_gpio_map_entry {
	u32 pin_num;
	char name[GPIO_MAP_NAME_SIZE];
} __attribute__ ((__packed__));

struct sw_f_irq_entry {
	u32 pin_num;
	u16 offset;
} __attribute__ ((__packed__));

void trim_gpio_map_string(char *s)
{
	int i;

	/* ignore all characters behind space key */
	for (i = 0; i < GPIO_MAP_NAME_SIZE; i++) {
		if (' ' == s[i]) {
			s[i] = '\0';
			return;
		}
	}

	pr_err("Too long gpio map string name!\n");
}
#endif

void __init mapphone_gpio_mapping_init(void)
{
#ifdef CONFIG_GPIO_MAPPING
	struct device_node *node;
	const void *prop;
	int i, j, size, unit_size;
	char name[GPIO_MAP_NAME_SIZE];
	u32 base = 0;

	node = of_find_node_by_path(DT_PATH_GPIO);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
				DT_PATH_GPIO);
		return;
	}

	/* Get the gpio sw force irq map 
	if (cpu_is_omap44xx())
		base = OMAP443X_CTRL_BASE;

	if (base > 0) {
		unit_size = sizeof(struct sw_f_irq_entry);
		prop = of_get_property(node, DT_PROP_GPIO_SW_FORCE_IRQ, &size);
		if (prop && (size % unit_size) == 0) {
			for (i = 0; i < size / unit_size; i++) {
				struct sw_f_irq_entry *p =
					(struct sw_f_irq_entry *) prop;
				gpio_force_irq_add(p->pin_num, base, p->offset);
				prop += unit_size;
			}
		} else
			pr_err("Read property %s error!\n",
				DT_PROP_GPIO_SW_FORCE_IRQ);
	}
*/
	/* Get the gpio name map */
	unit_size = sizeof(struct omap_gpio_map_entry);
	prop = of_get_property(node, DT_PROP_GPIO_MAP, &size);
	if ((!prop) || (size % unit_size)) {
		pr_err("Read property %s error!\n", DT_PROP_GPIO_MAP);
		of_node_put(node);
		return;
	}

	for (i = 0; i < size / unit_size; i++) {
		struct omap_gpio_map_entry *p =
				(struct omap_gpio_map_entry *) prop;

		memcpy((void *) name, p->name, GPIO_MAP_NAME_SIZE);
		trim_gpio_map_string(name);

		for (j = 0; j < GPIO_MAP_SIZE; j++) {
			if (gpio_map_table[j].used == 0) {
				gpio_map_table[j].used = 1;
				gpio_map_table[j].pin_num = p->pin_num;
				strncpy(gpio_map_table[j].name, name,
						GPIO_MAP_NAME_SIZE);
				break;
			} else if (strncmp(gpio_map_table[j].name, name,
					GPIO_MAP_NAME_SIZE) == 0) {
				gpio_map_table[j].pin_num = p->pin_num;
				break;
			}
		}

		if (j == GPIO_MAP_SIZE)
			pr_err("Unable to write gpio_map_table\n");
		else
			pr_debug("GPIO mapping write: pin = %d, name = %s\n",
						gpio_map_table[j].pin_num,
						gpio_map_table[j].name);

		prop += unit_size;
	}

	of_node_put(node);
	pr_debug("DT overwrite GPIO Mapping done!\n");
	gpio_mapping_init(gpio_map_table, GPIO_MAP_SIZE);
	pr_debug("GPIO Mapping init done!\n");
#else
	pr_debug("GPIO Mapping unused!\n");
#endif
}
