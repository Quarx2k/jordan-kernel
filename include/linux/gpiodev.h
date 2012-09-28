/* 
 * GPIODev                                       gpiodev.h
 *
 * Copyright 2006-2009 Motorola, Inc.
 */

/*
 * This library is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of the
 * License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
 * USA
 *
 */

#ifndef _GPIODEV_H_
#define _GPIODEV_H_

#include <linux/ioctl.h>

/*
 * GPIODev state flags
 */
#define GPIODEV_FLAG_OPEN           (0x1 << 0)
#define GPIODEV_FLAG_LOWLEVELACCESS (0x1 << 1)
#define GPIODEV_FLAG_CONFIGURABLE   (0x1 << 2)
#define GPIODEV_FLAG_INTERRUPTED    (0x1 << 3)

#define GPIO_DEVICE_NAME_LEN 20
#define GPIO_DEVICE_DEV_NAME "gpio-device"

/*
 * gpio_device defines a GPIO
 *     pin_nr - pin number of the GPIO
 *     device_name - name which shows up in /dev
 *     init_config - the default configuration of a GPIO
 *     current_config - the current configuration the GPIO
 *     flags -  status of the device node (see GPIODev state flags above)
 *     event_queue - queue to sleep on while waiting for an interrupt
 */
struct gpio_device {
	u32 pin_nr;
	char device_name[GPIO_DEVICE_NAME_LEN];
	u32 init_config;
	u32 current_config;
	u32 flags;
	wait_queue_head_t event_queue;
	struct mutex lock;
};

struct gpio_device_platform_data {
	const char *name;
	struct gpio_device *info;
	size_t info_count;
};

/*
 * GPIODEV_LOWLEVEL_CONFIG is used to specify the low level configuration
 * of a device. This structure is passed as a parameter to an ioctl()
 * with the type of either GPIODEV_GET_LOWLEVELCONFIG or
 * GPIODEV_SET_LOWLEVEL_CONFIG. 
 *
 * config - GPIO configuration: invalid, normal GPIO, interruptable GPIO
 * int_trigger - trigger condition if interruptable GPIO 
 */
typedef struct GPIODEV_LOWLEVEL_CONFIG {
	unsigned char config;
} GPIODEV_LOWLEVEL_CONFIG;


#define GPIODEV_BASE 'g'
/*
 * Definition of supported ioctl()'s 
 *
 * GPIODEV_GET_CONFIG - Get the configuration byte of a particular device.
 *                      The configuration byte is a bit mask of the various
 *                      defines described below.
 *
 * GPIODEV_SET_CONFIG - Set the configuration byte of a particular device.
 *                      The ioctl() will return an error if the configuration
 *                      could not be set. The configuration byte is a bit mask
 *                      of the various defines described below.
 *
 * GPIODEV_INT_REENABLE - After an interrupt is received, this ioctl() must be
 *                        performed before another interrupt from the same device
 *                        can be received.
 *
 * GPIODEV_GET_LOWLEVELCONFIG - Get the low level configuration of a device.
 *                              An error will be returned if this operation is
 *                              not allowed on a particular GPIO.
 *
 * GPIODEV_SET_LOWLEVELCONFIG - Set the low level configuartion of a device.
 *                              An error will be returned if this operation is
 *                              not allowed on a particular GPIO.
 *
 * GPIODEV_INT_POLL - Monitor the specified GPIO interrupt event.
 */
#define GPIODEV_GET_CONFIG         _IOR(GPIODEV_BASE, 0, int*)
#define GPIODEV_SET_CONFIG         _IOW(GPIODEV_BASE, 1, int*)
#define GPIODEV_INT_REENABLE       _IO(GPIODEV_BASE, 2)
#define GPIODEV_GET_LOWLEVELCONFIG _IOR(GPIODEV_BASE, 3, GPIODEV_LOWLEVEL_CONFIG*)
#define GPIODEV_SET_LOWLEVELCONFIG _IOW(GPIODEV_BASE, 4, GPIODEV_LOWLEVEL_CONFIG*)
#define GPIODEV_INT_POLL           _IO(GPIODEV_BASE, 5)

/*
 * Definition of the format of the interrupt trigger configuration byte
 *
 * GPIODEV_*INTTYPE_LLEV - Sets the interrupt type as level low sensitive 
 * GPIODEV_*INTTYPE_HLEV - Sets the interrupt type as level high sensitive
 * GPIODEV_*INTTYPE_BEDG - Sets the interrupt type as both edge sensitive 
 * GPIODEV_*INTTYPE_REDG - Sets the interrupt type as rising edge sensitive
 * GPIODEV_*INTTYPE_FEDG - Sets the interrupt type as falling edge sensitive
 *
 */
enum gpiodev_int_type {
	GPIODEV_INTTYPE_NONE = 0,
	GPIODEV_INTTYPE_REDG = 1,
	GPIODEV_INTTYPE_FEDG = 2,
	GPIODEV_INTTYPE_HLEV = 3,
	GPIODEV_INTTYPE_LLEV = 4,
	GPIODEV_INTTYPE_MAX
};

/*
 * Definition of the format of GPIODev configuraion options
 */
#define GPIODEV_CONFIG_INVALID        (0x1 << 0)
#define GPIODEV_CONFIG_INPUT          (0x1 << 1)
#define GPIODEV_CONFIG_OUTPUT_HIGH    (0x1 << 2)
#define GPIODEV_CONFIG_OUTPUT_LOW     (0x1 << 3)

#define GPIODEV_CONFIG_INT_MASK_OFFSET 8
#define GPIODEV_CONFIG_INT_NONE        (GPIODEV_INTTYPE_NONE << GPIODEV_CONFIG_INT_MASK_OFFSET)
#define GPIODEV_CONFIG_INT_REDG        (GPIODEV_INTTYPE_REDG << GPIODEV_CONFIG_INT_MASK_OFFSET)
#define GPIODEV_CONFIG_INT_FEDG        (GPIODEV_INTTYPE_FEDG << GPIODEV_CONFIG_INT_MASK_OFFSET)
#define GPIODEV_CONFIG_INT_HLEV        (GPIODEV_INTTYPE_HLEV << GPIODEV_CONFIG_INT_MASK_OFFSET)
#define GPIODEV_CONFIG_INT_LLEV        (GPIODEV_INTTYPE_LLEV << GPIODEV_CONFIG_INT_MASK_OFFSET)
#define GPIODEV_CONFIG_INT_MASK        (0xFF << GPIODEV_CONFIG_INT_MASK_OFFSET)

#endif				/* #ifndef _GPIODEV_H_ */
