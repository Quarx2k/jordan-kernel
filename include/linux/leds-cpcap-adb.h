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

#ifndef __LED_CPCAP_ADB_H__
#define __LED_CPCAP_ADB_H__

#define CPCAP_ADB_LED_CLASS_NAME_SIZE 64
#define CPCAP_ADB_LED_CLASS_NAME "cpcap-adb-led"
#define CPCAP_ADB_LED_DRV_NAME "leds-cpcap-adb"

#define CPCAP_ADB_LED_REG "sw5"

#define CPCAP_ADB_INIT_MASK        0x7FFF
#define CPCAP_ADB_INIT             0x5FF0

#define CPCAP_ADB_ON_OFF_MASK 0x000F
#define CPCAP_ADB_ON          0x000B
#define CPCAP_ADB_OFF_1       0x0001
#define CPCAP_ADB_OFF_2       0x0000

struct cpcap_adb_led_config_data {
	u16 init;
	u16 on;
	struct cpcap_abmode_config_data *abmode_config;
	char class_name[CPCAP_ADB_LED_CLASS_NAME_SIZE];
};

#endif  /* __LED_CPCAP_ADB_H__ */
