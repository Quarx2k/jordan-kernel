/*
 * Copyright (C) 2009 Motorola, Inc.
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

#ifndef _LINUX_AIRC_H_
#define _LINUX_AIRC_H_

#include <linux/ioctl.h>

#ifdef __KERNEL__

struct airc_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	int gpio;
} __attribute__ ((packed));

#endif /* __KERNEL__ */

#define AIRC_IO			0xA2

#define AIRC_IOCTL_GET_ENABLE	_IOR(AIRC_IO, 0x00, char)
#define AIRC_IOCTL_SET_ENABLE	_IOW(AIRC_IO, 0x01, char)

#endif /* _LINUX_AIRC_H__ */
