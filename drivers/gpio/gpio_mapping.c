/*
 *                                        gpio_mapping.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 */

/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

/* Date         Author      Comment
 * ===========  ==========  ==================================================
 * 10-Jun-2009  Motorola    Initial.
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/gpio_mapping.h>

static int gpio_map_size;
static struct gpio_mapping *gpio_map_table;

void __init gpio_mapping_init(struct gpio_mapping *table, int size)
{
	gpio_map_size = size;
	gpio_map_table = table;
}

int get_gpio_by_name(const char *name)
{
	int i;

	for (i = 0; i < gpio_map_size; i++) {
		if (gpio_map_table[i].used == 0)
			continue;

		if (strncmp(name, gpio_map_table[i].name, GPIO_MAP_NAME_SIZE)
			== 0)
			return gpio_map_table[i].pin_num;
	}

	printk(KERN_ERR "Unable to get gpio pin num for %s\n", name);
	return -EINVAL;
}
EXPORT_SYMBOL(get_gpio_by_name);
