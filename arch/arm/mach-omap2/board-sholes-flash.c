/*
 * linux/arch/arm/mach-omap2/board-sholes-flash.c
 * 
 * Copyright (C) 2007-2008 Motorola, Inc.
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

/* Date         Author          Comment
 * ===========  ==============  ==============================================
 * 25-Oct-2007  Motorola        Disable probing for NOR and reference design.
 * 26-Oct-2007  Motorola        Allow NAND GPMC to be configured in kernel.
 * 07-Apr-2008  Motorola        Add 166 MHz L3 frequency support
 * 07-Apr-2008  Motorola 	Fix gcc warnings for unused variables
 * 07-Apr-2008  Motorola        Add 166 MHz L3 frequency support
 * 26-May-2008  Motorola        Config NAND timing by device tree
 * 07-Jul-2008  Motorola        Port Mot Device tree to kernel 2.6.24
 * 04-Dec-2008  Motorola        Add L3 re-configuration support
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
//#include <mach/onenand.h>
#include <plat/board.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#ifdef CONFIG_MOT_FEAT_MEM_TIMING_API
#include "board-sholes-timing.h"
#endif

#define NUM_GPMC_SUPPORTED_FREQ     2
#define NAND_GPMC_CS                0
#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#if defined(CONFIG_OMAP3_PM)
/*
 * Structure containing the gpmc cs values at different frequencies
 * This structure will be populated run time depending on the
 * values read from the hardware configuration tree.
 */
struct gpmc_freq_config freq_config[NUM_GPMC_SUPPORTED_FREQ];
#endif


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
        .cs             = NAND_GPMC_CS,
        .gpmc_cs_baseaddr = (void *)( OMAP34XX_GPMC_VIRT + GPMC_CS0_BASE +
                NAND_GPMC_CS*GPMC_CS_SIZE ),
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
 * sholes_flash_init - Identify devices connected to GPMC and register.
 *
 * @return - void.
 */
void __init sholes_flash_init(void)
{

#if 0
#if defined(CONFIG_MOT_FEAT_KERNEL_NAND_INIT)

        int cs=NAND_GPMC_CS;
#if defined(CONFIG_MOT_FEAT_MEM_TIMING_API)
        unsigned int init_opp = get_mot_init_opp();

        if (is_mot_gpmc_timing_valid(init_opp,cs)) { 
            gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1,get_mot_gpmc_config(init_opp,cs,1));
            gpmc_cs_write_reg(cs, GPMC_CS_CONFIG2,get_mot_gpmc_config(init_opp,cs,2));
            gpmc_cs_write_reg(cs, GPMC_CS_CONFIG3,get_mot_gpmc_config(init_opp,cs,3));
            gpmc_cs_write_reg(cs, GPMC_CS_CONFIG4,get_mot_gpmc_config(init_opp,cs,4));
            gpmc_cs_write_reg(cs, GPMC_CS_CONFIG5,get_mot_gpmc_config(init_opp,cs,5));
            gpmc_cs_write_reg(cs, GPMC_CS_CONFIG6,get_mot_gpmc_config(init_opp,cs,6));
            printk(KERN_INFO "Initializing Nand GPMC done @ L3 = %d Hz!\n", read_mot_l3_freq(init_opp));
        }
        else {
            printk(KERN_ERR "Nand GPMC setting (cs%d) is invalid in device tree\n",cs);
        }
#else
       /* initial bringup: setup NAND register here instead of by bootloader 
	* GPMC_CONFIG1-6_i: GMPC_VIRT + (0x000 0030*i) i=0 for nand device, 
	* use mbm bootldr to do the flash device setup while it is available.
	*/
	printk("kernel is doing the nand device registers setup ...\n");
        gpmc_cs_write_reg(cs,GPMC_CS_CONFIG1, 0x00601A00);   /* GPMC_CONFIG1 */
        gpmc_cs_write_reg(cs,GPMC_CS_CONFIG2, 0x00080A00);   /* GPMC_CONFIG2 */
        gpmc_cs_write_reg(cs,GPMC_CS_CONFIG3, 0x00080000);   /* GPMC_CONFIG3 */
        gpmc_cs_write_reg(cs,GPMC_CS_CONFIG4, 0x06020702);   /* GPMC_CONFIG4 */
        /*Below setting are same for both 166 MHz & 133 MHz due to P0A HW limitation*/
        gpmc_cs_write_reg(cs,GPMC_CS_CONFIG5,0x00060606);   /* GPMC_CONFIG5 */
        gpmc_cs_write_reg(cs,GPMC_CS_CONFIG6,0x050101C0);
#endif /* CONFIG_MOT_FEAT_MEM_TIMING_API */

#endif /* CONFIG_MOT_FEAT_KERNEL_NAND_INIT */
#endif

	/* RDY/BSY line is connected, use GPMC_IRQSTATUS pull instead of 50 udelay */
	sdp_nand_data.dev_ready = omap_nand_dev_ready;

        if (platform_device_register(&sdp_nand_device) < 0) {
            printk(KERN_ERR "Unable to register NAND device\n");
        }

        return;
}
