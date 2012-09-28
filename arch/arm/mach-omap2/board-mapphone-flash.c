/*
 * linux/arch/arm/mach-omap2/board-mapphone-flash.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * Modified from mach-omap2/board-3430sdp-flash.c
 *
 * Copyright (c) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-2430sdp-flash.c
 * Author: Rohit Choraria <rohitkc@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/onenand_regs.h>
#include <linux/types.h>
#include <linux/io.h>

#include <asm/mach/flash.h>
#include <plat/board.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#include <linux/of.h>
#include <mach/dt_path.h>

#define NAND_GPMC_CS		0
#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

static struct mtd_partition sdp_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader-NAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Boot Env-NAND",

		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x100000 */
		.size		= 2 * (64 * 2048),
	},
	{
		.name		= "Kernel-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x140000 */
		.size		= 32 * (64 * 2048),


	},
	{
		.name		= "File System - NAND",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x540000 */
	},
};


static struct omap_nand_platform_data sdp_nand_data = {
	.parts		= sdp_nand_partitions,
	.nr_parts	= ARRAY_SIZE(sdp_nand_partitions),
	.nand_setup	= NULL,
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.dev_ready	= NULL,
	.cs		= NAND_GPMC_CS,
	.gpmc_cs_baseaddr = (void *)(OMAP34XX_GPMC_VIRT + GPMC_CS0_BASE +
		NAND_GPMC_CS*GPMC_CS_SIZE),
	.gpmc_baseaddr  = (void *)(OMAP34XX_GPMC_VIRT),
};


static struct resource sdp_nand_resource = {
	.flags		= IORESOURCE_MEM,
};


static struct platform_device sdp_nand_device = {
	.name		= "omap2-nand",
	.id		= 0,
	.dev		= {
		.platform_data	= &sdp_nand_data,
	},
	.num_resources	= 1,
	.resource	= &sdp_nand_resource,
};

static int omap_nand_dev_ready(struct omap_nand_platform_data *data)
{
	printk(KERN_INFO "RDY/BSY line is connected!\n");
	return 0;
}

/**
 * mapphone_flash_init - Identify devices connected to GPMC and register.
 *
 * @return - void.
 */
void __init mapphone_flash_init(void)
{
	struct device_node *feature_node;
	const void *nand_prop;

	/* If this phone doesn't have a NAND, don't waste time probing it */
	feature_node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (feature_node != NULL) {
		nand_prop = of_get_property(feature_node,
			DT_HIGH_LEVEL_FEATURE_NO_NAND, NULL);
		if (nand_prop != NULL && *(u8 *)nand_prop == 1)
			return;
	}

	/* We know the RDY/BSY line is connected now */
	sdp_nand_data.dev_ready = omap_nand_dev_ready;

	if (platform_device_register(&sdp_nand_device) < 0)
		printk(KERN_ERR "Unable to register NAND device\n");

	return;
}
