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

#ifndef __LED_CPCAP_ABMODE_H__
#define __LED_CPCAP_ABMODE_H__

#define CPCAP_ABMODE_INIT_MASK 0x003C
#define CPCAP_ABMODE_INIT      0x0000

struct cpcap_abmode_config_data {
	u16 abmode_init;
};

#endif  /* __LED_CPCAP_ADB_H__ */
