/*
 * drivers/media/video/hp3a/hp3a_af.c
 *
 * HP Imaging/3A Driver : AF specific function implementation.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
 *
 * Contributors:
 *		Tanvir Islam <tanvir.islam@hp.com>
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

#ifndef	__HP3A_AF_H_INCLUDED
#define	__HP3A_AF_H_INCLUDED

/* ISP af PCR bit values */
#define AF_ENABLE		(1<<0)
#define AF_ALAW_ENABLE		(1<<1)
#define AF_HMF_ENABLE		(1<<2)
#define AF_FVMODE_PEAK		(1<<14)
#define AF_BUSY			(1<<15)
#define AF_AEW_ENABLE		(1<<16)
#define AF_AEW_ALAW_ENABLE	(1<<17)
#define AF_BUSY_AEAWB		(1<<18)

/* ISP af PCR bit shifts */
#define AF_MED_TH_SHIFT		3
#define AF_RGBPOS_SHIFT		11
#define AF_AVE2LMT_SHIFT	22
#define AF_PAXW_SHIFT		16
#define AF_LINE_INCR_SHIFT	13
#define AF_VT_COUNT_SHIFT	6
#define AF_HZ_START_SHIFT	16
#define AF_COEF_SHIFT		16
#define AF_PAXEL_SIZE		48

/**
 * struct hp3a_af_hmf - data structure for Horizontal Median Filter
 */
struct hp3a_af_hmf {
	int  enable;	/* Status of Horizontal Median Filter  */
	u8 threshold;	/* Threshhold Value for Horizontal Median Filter */
};

/**
 * struct hp3a_af_iir - data structure for IIR Filters.
 */
#define AF_NUMBER_OF_COEF    11
struct hp3a_af_iir {
	u16 hz_start_pos;		/* IIR Start Register Value */
	int coeff_set0[AF_NUMBER_OF_COEF];/* IIR Filter Coefficient for Set 0 */
	int coeff_set1[AF_NUMBER_OF_COEF];/* IIR Filter Coefficient for* Set 1 */
};

/**
 * struct hp3a_af_paxel - AF statistics data to transfer between driver and user.
 **/
struct hp3a_af_paxel {
	u32 width;	/* Width of the Paxel */
	u32 height;	/* Height of the Paxel */
	u32 hz_start;	/* Horizontal Start Position */
	u32 vt_start;	/* Vertical Start Position */
	u32 hz_cnt;	/* Horizontal Count */
	u32 vt_cnt;	/* vertical Count */
	u32 line_incr;	/* Line Increment */
};

#endif /* __HP3A_AF_H_INCLUDED */
