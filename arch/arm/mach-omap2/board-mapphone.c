/*
 * Copyright (C) 2009 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/of_fdt.h>
#include <linux/bootmem.h>
#include <asm/setup.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "common.h"
#include "pm.h"

#include "sdram-toshiba-hynix-numonyx.h"

#define ATAG_FLAT_DEV_TREE_ADDRESS 0xf100040A

struct tag_flat_dev_tree_address {
	u32 address;
	u32 size;
};

static u32 fdt_start_address;
static u32 fdt_size;

/* process flat device tree for hardware configuration */
static int __init parse_tag_flat_dev_tree_address(const struct tag *tag)
{
	struct tag_flat_dev_tree_address *fdt_addr =
		(struct tag_flat_dev_tree_address *)&tag->u;

	if (fdt_addr->size) {
		fdt_start_address = (u32)phys_to_virt(fdt_addr->address);
		fdt_size = fdt_addr->size;
	}

	printk(KERN_INFO
		"flat_dev_tree_virt_address=0x%08x, flat_dev_tree_address=0x%08x, flat_dev_tree_size == 0x%08X\n",
		fdt_start_address,
		fdt_addr->address,
		fdt_addr->size);

	return 0;
}

__tagtable(ATAG_FLAT_DEV_TREE_ADDRESS, parse_tag_flat_dev_tree_address);

static void __init omap_mapphone_init(void)
{
	struct clk *clkp;
	omap2_sdrc_init(JEDEC_JESD209A_sdrc_params,
				   JEDEC_JESD209A_sdrc_params);
	/* Enable sad2d iclk */
	clkp = clk_get(NULL, "sad2d_ick");
	if (clkp) {
             clk_enable(clkp);
             printk("sad2d_ick enabled\n");
	}

	omap_cpcap_init();
}

static void __init omap_mapphone_init_early(void)
{
	if (fdt_start_address) {
		void *mem;
		mem = __alloc_bootmem(fdt_size, __alignof__(int), 0);
		BUG_ON(!mem);
		memcpy(mem, (const void *)fdt_start_address, fdt_size);
		initial_boot_params = (struct boot_param_header *)mem;
		pr_info("Unflattening device tree: 0x%08x\n", (u32)mem);
		unflatten_device_tree();
	}
	omap3630_init_early();
}
MACHINE_START(MAPPHONE, "mapphone_")
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap_mapphone_init_early,
	.init_irq	= omap3_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= omap_mapphone_init,
	.init_late	= omap3630_init_late,
	.init_time	= omap3_sync32k_timer_init,
	.restart	= omap3xxx_restart,
MACHINE_END
