/*
 * drivers/media/video/omap/hplens.c
 *
 * HP Generic Driver : Driver implementation for generic lens module.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
 *
 * Contributors:
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#ifndef __HPLENS_H_INCLUDED
#define __HPLENS_H_INCLUDED

#define HPLENS_AF_I2C_ADDR	0x04  /* Mini Universal Driver IC */
#define HPLENS_NAME		"HP_GEN_LENS"
#define HPLENS_I2C_RETRY_COUNT	5

/* State of lens */
#define LENS_DETECTED		1
#define LENS_NOT_DETECTED	0

/* #define DEBUG_I2C_TIME */

#define OMAP3_HPLENS_MAGIC	'L'
#define OMAP3_HPLENS_IOCTL_BASE	0xF
#define OMAP3_HPLENS_CMD_READ	\
		_IOWR(OMAP3_HPLENS_MAGIC, OMAP3_HPLENS_IOCTL_BASE+0, struct hplens_reg)
#define OMAP3_HPLENS_CMD_WRITE		\
		_IOWR(OMAP3_HPLENS_MAGIC, OMAP3_HPLENS_IOCTL_BASE+1, struct hplens_reg)
#define OMAP3_HPLENS_CMD_READ_PAGE	\
		_IOWR(OMAP3_HPLENS_MAGIC, OMAP3_HPLENS_IOCTL_BASE+2, struct hplens_reg_page)
#define OMAP3_HPLENS_CMD_WRITE_PAGE		\
		_IOWR(OMAP3_HPLENS_MAGIC, OMAP3_HPLENS_IOCTL_BASE+3, struct hplens_reg_page)
#define OMAP3_HPLENS_CMD_READ_CAL	\
		_IOWR(OMAP3_HPLENS_MAGIC, OMAP3_HPLENS_IOCTL_BASE+4, struct hplens_eeprom)

#define HPLENS_CID_BASE			(V4L2_CID_PRIVATE_BASE + 30)
#define V4L2_CID_HPLENS_CMD_READ	(HPLENS_CID_BASE+0)
#define V4L2_CID_HPLENS_CMD_WRITE	(HPLENS_CID_BASE+1)
#define V4L2_CID_HPLENS_CMD_READ_PAGE	(HPLENS_CID_BASE+2)
#define V4L2_CID_HPLENS_CMD_WRITE_PAGE	(HPLENS_CID_BASE+3)
#define V4L2_CID_HPLENS_CMD_CAL_READ	(HPLENS_CID_BASE+4)

struct hplens_reg {
   u8  dev_addr;
   u8  addr[4];
   u32 len_addr;
   u8  data[16];
   u32 len_data;
   struct timespec ts_end;
};

struct hplens_reg_page {
   u16 len;
   struct hplens_reg *regs;
};

struct hplens_eeprom {
   u8  dev_addr;
   u8  addr[4];
   u32 len_addr;
   u8  data[4096];
   u32 len_data;
   struct timespec ts_end;
};

/**
 * struct hplens_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @priv_data_set: device private data (pointer) access function
 */
struct hplens_platform_data {
	int (*power_set)(enum v4l2_power power);
	int (*priv_data_set)(void *);
};

#define CAMAF_LV8093_DISABLE		0x1
#define CAMAF_LV8093_ENABLE	    	0x0
#define CAMAF_LV8093_DRVPLS_REG		0x0
#define CAMAF_LV8093_CTL_REG		0x1
#define CAMAF_LV8093_RST_REG		0x2
#define CAMAF_LV8093_GTAS_REG		0x3
#define CAMAF_LV8093_GTBR_REG		0x4
#define CAMAF_LV8093_GTBS_REG		0x5
#define CAMAF_LV8093_STP_REG		0x6
#define CAMAF_LV8093_MOV_REG		0x7
#define CAMAF_LV8093_MAC_DIR        0x80
#define CAMAF_LV8093_INF_DIR        0x00
#define CAMAF_LV8093_GATE0          0x00
#define CAMAF_LV8093_GATE1          0x80
#define CAMAF_LV8093_ENIN           0x20
#define CAMAF_LV8093_CKSEL_ONE      0x18
#define CAMAF_LV8093_CKSEL_HALF     0x08
#define CAMAF_LV8093_CKSEL_QTR      0x00
#define CAMAF_LV8093_RET2           0x00
#define CAMAF_LV8093_RET1           0x02
#define CAMAF_LV8093_RET3           0x04
#define CAMAF_LV8093_RET4           0x06
#define CAMAF_LV8093_INIT_OFF       0x01
#define CAMAF_LV8093_INIT_ON        0x00
#define CAMAF_LV8093_BUSY           0x80

void hplens_lock(void);
void hplens_unlock(void);

#endif /* __HPLENS_H_INCLUDED */

