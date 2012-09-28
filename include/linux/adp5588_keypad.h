/*
 * Copyright (c) 2008-2010, Motorola, All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __ADP5588_KEYPAD_H__
#define __ADP5588_KEYPAD_H__

#include <linux/hrtimer.h>
#include <linux/gpio_event.h>
#include <linux/workqueue.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef __KERNEL__
#define ADP5588_I2C_ADDRESS 0x34

/* Motorola Platform requires keypad devices to be named as "sholes-xxx" */
#define ADP5588_KEYPAD_NAME    "sholes-keypad"
#define ADP5588_BACKLIGHT_NAME "sholes-backlight"

/* These are the default GPIOs for ADP5588's INT and RESET. They can */
/* be overriden by configuration provided in device tree.            */
#define ADP5588_RESET_GPIO 67
#define ADP5588_INT_GPIO   68

/* These are the maximum columns and rows supported by ADP5588 HW,  */
/* against which the configuration provided in device tree must be  */
/* validated.                                                       */
#define ADP5588_COL_NUM_MAX 10   /* Col 0 to Col 9 */
#define ADP5588_ROW_NUM_MAX  8   /* Row 0 to Row 7 */

struct adp5588_platform_data {
	uint16_t            addr;
	struct i2c_client  *client;
	struct input_dev   *input_dev;
	uint8_t             use_irq;
	uint8_t             dev_id;
	struct              hrtimer timer;
	struct work_struct  work;
	uint16_t            last_key;
	uint16_t            last_key_state;
	uint8_t             ringer_switch;
	int                 update_config;
	struct mutex	    mutex;   /* Access mutex for this device-driver */
	uint16_t           *keymap;
	int                 reset_gpio;
	int                 int_gpio;
	int                 leds_mask;
	uint8_t             use_adp5588;
	struct platform_device *leds_device;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

struct adp5588_leds_platform_data {
	uint8_t             use_leds;
};

extern struct adp5588_platform_data mapphone_adp5588_pdata;

extern int adp5588_get_backlight(void);
extern int adp5588_set_backlight(uint8_t mask);
extern int gpio_event_adp5588_func(struct gpio_event_input_devs *input_devs,
			struct gpio_event_info *info, void **data, int func);
#endif /* __KERNEL__ */

#endif /* __ADP5588_KEYPAD_H__ */


