/*
 * Copyright (C) 2011 Motorola Mobility, Inc.
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

#ifndef __LED_CPCAP_BT_H__
#define __LED_CPCAP_BT_H__

#define CPCAP_BT_LED_CLASS_NAME_SIZE 64
#define CPCAP_BT_LED_CLASS_NAME "cpcap-bt-led"
#define CPCAP_BT_LED_DRV_NAME "leds-cpcap-bt"

#define CPCAP_BT_LED_REG "sw5"

#define CPCAP_BT_INIT_MASK   0x03FF
#define CPCAP_BT_INIT        0x0010

#define CPCAP_BT_ON_OFF_MASK 0x0007
#define CPCAP_BT_ON          0x0001
#define CPCAP_BT_OFF         0x0000

struct cpcap_bt_led_config_data {
	u16 init;
	u16 on;
	char class_name[CPCAP_BT_LED_CLASS_NAME_SIZE];
};

#endif  /* __LED_CPCAP_BT_H__ */
