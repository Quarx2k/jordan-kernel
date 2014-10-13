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

#ifndef __LED_CPCAP_MDB_H__
#define __LED_CPCAP_MDB_H__

#define CPCAP_MDB_LED_CLASS_NAME_SIZE 64
#define CPCAP_MDB_LED_CLASS_NAME "cpcap-mdb-led"
#define CPCAP_MDB_LED_DRV_NAME "leds-cpcap-mdb"

#define CPCAP_MDB_LED_REG "sw5"

#define CPCAP_MDB_INIT_MASK        0xFFFF
#define CPCAP_MDB_INIT             0xB008
#define CPCAP_MDB_ABMODE_INIT_MASK 0x001C
#define CPCAP_MDB_ABMODE_INIT      0x001C

#define CPCAP_MDB_ON_OFF_MASK 0x0FE1
#define CPCAP_MDB_ON          0x0001
#define CPCAP_MDB_OFF_1       0x0001
#define CPCAP_MDB_OFF_2       0x0000

struct cpcap_mdb_led_config_data {
	u16 init;
	struct cpcap_abmode_config_data *abmode_config;
	char class_name[CPCAP_MDB_LED_CLASS_NAME_SIZE];
};

#endif  /* __LED_CPCAP_MDB_H__ */
