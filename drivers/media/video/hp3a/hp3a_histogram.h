/*
 * drivers/media/video/hp3a/hp3a_isp_reg.h
 *
 * HP Imaging/3A Histogram specific definitions.
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

#ifndef	__HP3A_HISTOGRAM_H_INCLUDED
#define	__HP3A_HISTOGRAM_H_INCLUDED

#define MAX_REGIONS             0x4
#define MAX_WB_GAIN             255
#define MIN_WB_GAIN             0x0
#define MAX_BIT_WIDTH           14
#define MIN_BIT_WIDTH           8

#define BINS_32			0x0
#define BINS_64			0x1
#define BINS_128		0x2
#define BINS_256		0x3

#define ISPHIST_PCR_EN          (1 << 0)
#define HIST_MEM_SIZE           1024
#define ISPHIST_CNT_CLR_EN      (1 << 7)

#define WRITE_SOURCE(reg, source)       \
	(reg = (reg & ~(ISPHIST_CNT_SOURCE_MASK)) \
	| (source << ISPHIST_CNT_SOURCE_SHIFT))

#define WRITE_HV_INFO(reg, hv_info) \
	(reg = ((reg & ~(ISPHIST_HV_INFO_MASK)) \
	| (hv_info & ISPHIST_HV_INFO_MASK)))

#define WRITE_RADD(reg, radd) \
	(reg = (reg & ~(ISPHIST_RADD_MASK)) \
	| (radd << ISPHIST_RADD_SHIFT))

#define WRITE_RADD_OFF(reg, radd_off) \
	(reg = (reg & ~(ISPHIST_RADD_OFF_MASK)) \
	| (radd_off << ISPHIST_RADD_OFF_SHIFT))

#define WRITE_BIT_SHIFT(reg, bit_shift) \
	(reg = (reg & ~(ISPHIST_CNT_SHIFT_MASK)) \
	| (bit_shift << ISPHIST_CNT_SHIFT_SHIFT))

#define WRITE_DATA_SIZE(reg, data_size) \
	(reg = (reg & ~(ISPHIST_CNT_DATASIZE_MASK)) \
	| (data_size << ISPHIST_CNT_DATASIZE_SHIFT))

#define WRITE_NUM_BINS(reg, num_bins) \
	(reg = (reg & ~(ISPHIST_CNT_BINS_MASK)) \
	| (num_bins << ISPHIST_CNT_BINS_SHIFT))

#define WRITE_WB_R(reg, reg_wb_gain) \
	reg = ((reg & ~(ISPHIST_WB_GAIN_WG00_MASK)) \
	| (reg_wb_gain << ISPHIST_WB_GAIN_WG00_SHIFT))

#define WRITE_WB_RG(reg, reg_wb_gain) \
	(reg = (reg & ~(ISPHIST_WB_GAIN_WG01_MASK)) \
	| (reg_wb_gain << ISPHIST_WB_GAIN_WG01_SHIFT))

#define WRITE_WB_B(reg, reg_wb_gain) \
	(reg = (reg & ~(ISPHIST_WB_GAIN_WG02_MASK)) \
	| (reg_wb_gain << ISPHIST_WB_GAIN_WG02_SHIFT))

#define WRITE_WB_BG(reg, reg_wb_gain) \
	(reg = (reg & ~(ISPHIST_WB_GAIN_WG03_MASK)) \
	| (reg_wb_gain << ISPHIST_WB_GAIN_WG03_SHIFT))

#define WRITE_REG_HORIZ(reg, reg_n_hor) \
	(reg = ((reg & ~ISPHIST_REGHORIZ_MASK) \
	| (reg_n_hor & ISPHIST_REGHORIZ_MASK)))

#define WRITE_REG_VERT(reg, reg_n_vert) \
	(reg = ((reg & ~ISPHIST_REGVERT_MASK) \
	| (reg_n_vert & ISPHIST_REGVERT_MASK)))

#endif /* #define	__HP3A_HISTOGRAM_H_INCLUDED */
