/*
 * Copyright (C) 2009 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __INCLUDED_TDA19989__
#define __INCLUDED_TDA19989__

#include <linux/types.h>

#define TDA19989_I2C_ARG_MAX_DATA	(128)

struct i2cMsgArg {
    unsigned char slaveAddr;
    unsigned char firstRegister;
    unsigned char lenData;
    unsigned char Data[TDA19989_I2C_ARG_MAX_DATA];
};

#define TDA19989_IOCTL_MAGIC	'g'
#define TDA19989_IOCTL_BASE	0x40
#define TDA19989_I2C_WRITE	_IOW(TDA19989_IOCTL_MAGIC, \
				    TDA19989_IOCTL_BASE+0, struct i2cMsgArg)
#define TDA19989_I2C_READ	_IOWR(TDA19989_IOCTL_MAGIC, \
				    TDA19989_IOCTL_BASE+1, struct i2cMsgArg)
#define TDA19989_PWR_ENABLE	_IOW(TDA19989_IOCTL_MAGIC, \
				    TDA19989_IOCTL_BASE+2, int)
#define TDA19989_INT_ENABLE	_IOW(TDA19989_IOCTL_MAGIC, \
				    TDA19989_IOCTL_BASE+3, int)
#define TDA19989_CEC_CAL_TIME	_IO(TDA19989_IOCTL_MAGIC, \
				    TDA19989_IOCTL_BASE+4)
#define TDA19989_CEC_PWR_ENABLE _IOW(TDA19989_IOCTL_MAGIC, \
				    TDA19989_IOCTL_BASE+5, int)
#define TDA19989_CEC_SUPPORT	_IO(TDA19989_IOCTL_MAGIC, \
				    TDA19989_IOCTL_BASE+6)

#endif

