/*
 * linux-2.6.x/include/linux/gpio_mapping.h
 *
 * Prototypes and definitions used for the implementation of the
 * GPIO API.
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * Initial code: Motorola
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/* Date        Author          Comment
 * ----------  --------------  ----------------------------------------------
 * 23/06/2009  Motorola        Initial revision.
 */

#ifndef __LINUX_GPIO_MAPPING_H
#define __LINUX_GPIO_MAPPING_H

#define GPIO_MAP_NAME_SIZE 20

struct gpio_mapping {
	u32 used;
	u32 pin_num;
	char name[GPIO_MAP_NAME_SIZE];
};

extern void __init gpio_mapping_init(struct gpio_mapping *table, int size);
extern int get_gpio_by_name(char *name);

#endif /* __LINUX_GPIO_MAPPING_H */
