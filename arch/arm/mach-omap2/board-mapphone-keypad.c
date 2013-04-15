/*
 * arch/arm/mach-omap2/board-mapphone-keypad.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_event.h>
#include <linux/keyreset.h>

#include <plat/mux.h>
#include <plat/gpio.h>
#include <linux/gpio_mapping.h>
#include <plat/keypad.h>
#include <plat/board-mapphone.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#define GPIO_SLIDER			177

static unsigned int mapphone_col_gpios[] = { 43, 53, 54, 55, 56, 57, 58, 63 };
static unsigned int mapphone_row_gpios[] = { 34, 35, 36, 37, 38, 39, 40, 41 };

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(mapphone_row_gpios) + (row))

static const unsigned short mapphone_p3_keymap[ARRAY_SIZE(mapphone_col_gpios) *
					     ARRAY_SIZE(mapphone_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_9,
	[KEYMAP_INDEX(0, 1)] = KEY_R,
	[KEYMAP_INDEX(0, 2)] = KEY_SEND, /* n/c dummy for CALLSEND testing*/
	[KEYMAP_INDEX(0, 3)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(0, 4)] = KEY_F4,   /* n/c dummy for CALLEND testing */
	[KEYMAP_INDEX(0, 5)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 6)] = KEY_SEARCH,
	[KEYMAP_INDEX(0, 7)] = KEY_D,

	[KEYMAP_INDEX(1, 0)] = KEY_7,
	[KEYMAP_INDEX(1, 1)] = KEY_M,
	[KEYMAP_INDEX(1, 2)] = KEY_L,
	[KEYMAP_INDEX(1, 3)] = KEY_K,
	[KEYMAP_INDEX(1, 4)] = KEY_N,
	[KEYMAP_INDEX(1, 5)] = KEY_C,
	[KEYMAP_INDEX(1, 6)] = KEY_Z,
	[KEYMAP_INDEX(1, 7)] = KEY_RIGHTSHIFT,

	[KEYMAP_INDEX(2, 0)] = KEY_1,
	[KEYMAP_INDEX(2, 1)] = KEY_Y,
	[KEYMAP_INDEX(2, 2)] = KEY_I,
	[KEYMAP_INDEX(2, 3)] = KEY_COMMA,
	[KEYMAP_INDEX(2, 4)] = KEY_LEFTALT,
	[KEYMAP_INDEX(2, 5)] = KEY_DOT,
	[KEYMAP_INDEX(2, 6)] = KEY_G,
	[KEYMAP_INDEX(2, 7)] = KEY_E,

/*	[KEYMAP_INDEX(3, 0)] = KEY_, */
	[KEYMAP_INDEX(3, 1)] = KEY_6,
	[KEYMAP_INDEX(3, 2)] = KEY_3,
	[KEYMAP_INDEX(3, 3)] = KEY_DOWN,
	[KEYMAP_INDEX(3, 4)] = KEY_UP,
	[KEYMAP_INDEX(3, 5)] = KEY_LEFT,
	[KEYMAP_INDEX(3, 6)] = KEY_RIGHT,
	[KEYMAP_INDEX(3, 7)] = KEY_REPLY,	/* d-pad center key */

	[KEYMAP_INDEX(4, 0)] = KEY_5,
	[KEYMAP_INDEX(4, 1)] = KEY_J,
	[KEYMAP_INDEX(4, 2)] = KEY_B,
	[KEYMAP_INDEX(4, 3)] = KEY_CAMERA-1,	/* camera 1 key, steal KEY_HP*/
	[KEYMAP_INDEX(4, 4)] = KEY_T,
	[KEYMAP_INDEX(4, 5)] = KEY_CAMERA,	/* "camera 2" key */
	[KEYMAP_INDEX(4, 6)] = KEY_MENU,
	[KEYMAP_INDEX(4, 7)] = KEY_X,

	[KEYMAP_INDEX(5, 0)] = KEY_8,
	[KEYMAP_INDEX(5, 1)] = KEY_SPACE,
	[KEYMAP_INDEX(5, 2)] = KEY_RIGHTALT,
/*	[KEYMAP_INDEX(5, 3)] = KEY_, */
	[KEYMAP_INDEX(5, 4)] = KEY_SLASH,
	[KEYMAP_INDEX(5, 5)] = KEY_EMAIL,	/* @ */
	[KEYMAP_INDEX(5, 6)] = KEY_BACKSPACE,
	[KEYMAP_INDEX(5, 7)] = KEY_A,

	[KEYMAP_INDEX(6, 0)] = KEY_2,
	[KEYMAP_INDEX(6, 1)] = KEY_0,
	[KEYMAP_INDEX(6, 2)] = KEY_F,
	[KEYMAP_INDEX(6, 3)] = KEY_LEFTSHIFT,
	[KEYMAP_INDEX(6, 4)] = KEY_ENTER,
	[KEYMAP_INDEX(6, 5)] = KEY_O,
	[KEYMAP_INDEX(6, 6)] = KEY_H,
	[KEYMAP_INDEX(6, 7)] = KEY_Q,

	[KEYMAP_INDEX(7, 0)] = KEY_4,
	[KEYMAP_INDEX(7, 1)] = KEY_V,
	[KEYMAP_INDEX(7, 2)] = KEY_S,
	[KEYMAP_INDEX(7, 3)] = KEY_P,
	[KEYMAP_INDEX(7, 4)] = KEY_QUESTION,
	[KEYMAP_INDEX(7, 5)] = KEY_MUTE,
	[KEYMAP_INDEX(7, 6)] = KEY_U,
	[KEYMAP_INDEX(7, 7)] = KEY_W,
};

static const unsigned short *mapphone_keymap_closed;

static struct gpio_event_direct_entry mapphone_keypad_switch_map[] = {
	{GPIO_SLIDER,		SW_LID}
};

static struct gpio_event_matrix_info mapphone_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = mapphone_p3_keymap,
	.output_gpios = mapphone_col_gpios,
	.input_gpios = mapphone_row_gpios,
	.noutputs = ARRAY_SIZE(mapphone_col_gpios),
	.ninputs = ARRAY_SIZE(mapphone_row_gpios),
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_REMOVE_PHANTOM_KEYS |
		 GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS*/
};

static struct gpio_event_input_info mapphone_keypad_switch_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_SW,
	.keymap = mapphone_keypad_switch_map,
	.keymap_size = ARRAY_SIZE(mapphone_keypad_switch_map)
};

static struct gpio_event_info *mapphone_keypad_info[] = {
	&mapphone_keypad_matrix_info.info,
	&mapphone_keypad_switch_info.info,
};

static struct gpio_event_platform_data mapphone_keypad_data = {
	.name = "mapphone-keypad",
	.info = mapphone_keypad_info,
	.info_count = ARRAY_SIZE(mapphone_keypad_info)
};

static struct platform_device mapphone_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &mapphone_keypad_data,
	},
};

static int __init mapphone_dt_kp_init(void)
{
	struct device_node *kp_node;
	const void *kp_prop;
	int slider_gpio;

	if ((kp_node = of_find_node_by_path(DT_PATH_KEYPAD))) {
		if ((kp_prop = of_get_property(kp_node, \
				DT_PROP_KEYPAD_ROWS, NULL)))
			mapphone_keypad_matrix_info.ninputs = \
				*(int *)kp_prop;

		if ((kp_prop = of_get_property(kp_node, \
				DT_PROP_KEYPAD_COLS, NULL)))
			mapphone_keypad_matrix_info.noutputs = \
				*(int *)kp_prop;

		if ((kp_prop = of_get_property(kp_node, \
				DT_PROP_KEYPAD_ROWREG, NULL)))
			mapphone_keypad_matrix_info.input_gpios = \
				(int *)kp_prop;

		if ((kp_prop = of_get_property(kp_node, \
				DT_PROP_KEYPAD_COLREG, NULL)))
			mapphone_keypad_matrix_info.output_gpios = \
				(int *)kp_prop;

		if ((kp_prop = of_get_property(kp_node, \
				DT_PROP_KEYPAD_MAPS, NULL)))
			mapphone_keypad_matrix_info.keymap = \
				(unsigned short *)kp_prop;

		slider_gpio = get_gpio_by_name("slider_data");
		if (slider_gpio < 0)
			slider_gpio = GPIO_SLIDER;
		mapphone_keypad_switch_map[0].gpio = slider_gpio;

		kp_prop = of_get_property(kp_node, \
				DT_PROP_KEYPAD_CLOSED_MAPS, NULL);
		if (kp_prop) {
			mapphone_keymap_closed = (unsigned short *)kp_prop;
		}

		of_node_put(kp_node);
	}

	return kp_node ? 0 : -ENODEV;
}

static int __init mapphone_init_keypad(void)
{
	if (mapphone_dt_kp_init())
		printk(KERN_INFO "Keypad: using non-dt configuration\n");

	return platform_device_register(&mapphone_keypad_device);
}

device_initcall(mapphone_init_keypad);
