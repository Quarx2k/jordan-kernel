/*
 * Copyright (C) 2009-2010 Motorola, Inc.
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

#ifndef __LED_CPCAP_RGB_H__
#define __LED_CPCAP_RGB_H__

#define CPCAP_RGB_LED_CLASS_NAME_SIZE 64
#define CPCAP_RGB_LED_RED_CLASS_NAME   "cpcap-rgb-led-red"
#define CPCAP_RGB_LED_GREEN_CLASS_NAME "cpcap-rgb-led-green"
#define CPCAP_RGB_LED_BLUE_CLASS_NAME  "cpcap-rgb-led-blue"
#define CPCAP_RGB_LED_DRV_NAME         "leds-cpcap-rgb"

#define CPCAP_RGB_LED_REG "sw5"

#define CPCAP_RGB_RED_LED      0x01
#define CPCAP_RGB_GREEN_LED    0x02
#define CPCAP_RGB_BLUE_LED     0x04

#define CPCAP_RGB_ON_OFF_MASK  0x03FF

#define CPCAP_RGB_LOW_LIMIT      51
#define CPCAP_RGB_LOW_MED_LIMIT  104
#define CPCAP_RGB_MEDIUM_LIMIT   155
#define CPCAP_RGB_MED_HIGH_LIMIT 201

#define CPCAP_RGB_LOW_VALUE      0x23
#define CPCAP_RGB_LOW_MED_VALUE  0x23
#define CPCAP_RGB_MEDIUM_VALUE   0x33
#define CPCAP_RGB_MED_HIGH_VALUE 0x43
#define CPCAP_RGB_HIGH_VALUE     0x53

#define CPCAP_RGB_OFF_1        0x0001
#define CPCAP_RGB_OFF_2        0x0000

struct cpcap_rgb_led_config_data {
	bool red_enable;
	bool green_enable;
	bool blue_enable;
	bool blink_enable;
	char class_name_red[CPCAP_RGB_LED_CLASS_NAME_SIZE];
	char class_name_green[CPCAP_RGB_LED_CLASS_NAME_SIZE];
	char class_name_blue[CPCAP_RGB_LED_CLASS_NAME_SIZE];
};

#endif  /* __LED_CPCAP_RGB_H__ */
