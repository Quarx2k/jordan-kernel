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

#ifndef __LED_CPCAP_GPIO_H__
#define __LED_CPCAP_GPIO_H__

#define CPCAP_GPIO_LED_CLASS_NAME_SIZE 64
#define CPCAP_GPIO_LED_CLASS_NAME "cpcap-gpio-led"
#define CPCAP_GPIO_LED_DRV_NAME "leds-cpcap-gpio"

#define CPCAP_GPIO_INIT_MASK   0x0004
#define CPCAP_GPIO_INIT        0x0004

#define CPCAP_GPIO_ON_OFF_MASK 0x0002
#define CPCAP_GPIO_ON          0x0002
#define CPCAP_GPIO_OFF         0x0000

struct cpcap_gpio_led_config_data {
	enum cpcap_reg reg;
	u16 init_mask;
	u16 init;
	char class_name[CPCAP_GPIO_LED_CLASS_NAME_SIZE];
};

#endif  /* __LED_CPCAP_GPIO_H__ */
