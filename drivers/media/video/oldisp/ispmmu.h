/*
 * drivers/media/video/isp/ispmmu.h
 *
 * OMAP3430 Camera ISP MMU API
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Contributors:
 *	Senthilvadivu Guruswamy <svadivu@ti.com>
 *	Thara Gopinath <thara@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_MMU_H
#define OMAP_ISP_MMU_H

#define ISPMMU_L1D_TYPE_SHIFT		0
#define ISPMMU_L1D_TYPE_MASK		0x3
#define ISPMMU_L1D_TYPE_FAULT		0
#define ISPMMU_L1D_TYPE_FAULT1		3
#define ISPMMU_L1D_TYPE_PAGE		1
#define ISPMMU_L1D_TYPE_SECTION		2
#define ISPMMU_L1D_PAGE_ADDR_SHIFT	10

#define ISPMMU_L2D_TYPE_SHIFT		0
#define ISPMMU_L2D_TYPE_MASK		0x3
#define ISPMMU_L2D_TYPE_FAULT		0
#define ISPMMU_L2D_TYPE_LARGE_PAGE	1
#define ISPMMU_L2D_TYPE_SMALL_PAGE	2
#define ISPMMU_L2D_SMALL_ADDR_SHIFT	12
#define ISPMMU_L2D_SMALL_ADDR_MASK	0xFFFFF000
#define ISPMMU_L2D_M_ACCESSBASED	(1 << 11)
#define ISPMMU_L2D_E_BIGENDIAN		(1 << 9)
#define ISPMMU_L2D_ES_SHIFT		4
#define ISPMMU_L2D_ES_MASK		(~(3 << 4))
#define ISPMMU_L2D_ES_8BIT		0
#define ISPMMU_L2D_ES_16BIT		1
#define ISPMMU_L2D_ES_32BIT		2
#define ISPMMU_L2D_ES_NOENCONV		3

#define ISPMMU_TTB_ENTRIES_NR		4096

/* Number 1MB entries in TTB in one 32MB region */
#define ISPMMU_REGION_ENTRIES_NR	32

/* 128 region entries */
#define ISPMMU_REGION_NR (ISPMMU_TTB_ENTRIES_NR / ISPMMU_REGION_ENTRIES_NR)

/* Each region is 32MB */
#define ISPMMU_REGION_SIZE		(ISPMMU_REGION_ENTRIES_NR * (1 << 20))

/* Number of entries per L2 Page table */
#define ISPMMU_L2D_ENTRIES_NR		256

/* Size of MMU page in bytes */
#define ISPMMU_PAGE_SIZE		4096

/*
 * Statically allocate 16KB for L2 page tables. 16KB can be used for
 * up to 16 L2 page tables which cover up to 16MB space. We use an array of 16
 * to keep track of these 16 L2 page table's status.
 */
#define L2P_TABLE_SIZE			1024
#define L2P_TABLE_NR 			62 /* Currently supports 4*5MP shots */
#define L2P_TABLES_SIZE 		(L2P_TABLE_SIZE * L2P_TABLE_NR)

/* Extra memory allocated to get ttb aligned on 16KB */
#define ISPMMU_TTB_MISALIGN_SIZE	0x3000

#ifdef CONFIG_ARCH_OMAP3410
#include <linux/scatterlist.h>
#endif

enum ISPMMU_MAP_ENDIAN {
	L_ENDIAN,
	B_ENDIAN
};

enum ISPMMU_MAP_ELEMENTSIZE {
	ES_8BIT,
	ES_16BIT,
	ES_32BIT,
	ES_NOENCONV
};

enum ISPMMU_MAP_MIXEDREGION {
	ACCESS_BASED,
	PAGE_BASED
};

enum ISPMMU_MAP_SIZE {
	L1DFAULT,
	PAGE,
	SECTION,
	SUPERSECTION,
	L2DFAULT,
	LARGEPAGE,
	SMALLPAGE
};

dma_addr_t ispmmu_map(unsigned int p_addr, int size);

dma_addr_t ispmmu_map_sg(const struct scatterlist *sglist, int sglen);

dma_addr_t ispmmu_map_pages(struct page **pages, int page_nr);

int ispmmu_unmap(dma_addr_t isp_addr);

void ispmmu_print_status(void);

void ispmmu_save_context(void);

void ispmmu_restore_context(void);

#endif /* OMAP_ISP_MMU_H */
