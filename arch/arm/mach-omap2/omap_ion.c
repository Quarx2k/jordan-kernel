/*
 * ION Initialization for OMAPXX.
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/ion.h>
#include <linux/memblock.h>
#include <linux/omap_ion.h>
#include <linux/platform_device.h>

#include "omap_ion.h"

static struct ion_platform_data omap_ion_data = {
#if defined(CONFIG_ARCH_OMAP4)
	.nr = 3,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = OMAP_ION_HEAP_SECURE_INPUT,
			.name = "secure_input",
			.base = PHYS_ADDR_SMC_MEM -
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE,
			.size = OMAP4_ION_HEAP_SECURE_INPUT_SIZE,
		},
		{	.type = OMAP_ION_HEAP_TYPE_TILER,
			.id = OMAP_ION_HEAP_TILER,
			.name = "tiler",
			.base = PHYS_ADDR_DUCATI_MEM -
					OMAP4_ION_HEAP_TILER_SIZE,
			.size = OMAP4_ION_HEAP_TILER_SIZE,
		},
		{
			.type = OMAP_ION_HEAP_TYPE_TILER,
			.id = OMAP_ION_HEAP_NONSECURE_TILER,
			.name = "nonsecure_tiler",
			.base = 0x80000000 + SZ_512M + SZ_2M,
			.size = OMAP4_ION_HEAP_NONSECURE_TILER_SIZE,
		},
	},
#elif defined(CONFIG_ARCH_OMAP3)
	.nr = 1,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = OMAP_ION_HEAP_SECURE_INPUT,
			.name = "omap3_carveout",
			.base = OMAP3_PHYS_ADDR_SMC_MEM -
					OMAP3_ION_HEAP_CARVEOUT_INPUT_SIZE,
			.size = OMAP3_ION_HEAP_CARVEOUT_INPUT_SIZE,
		},
	},
#endif
};

static struct platform_device omap_ion_device = {
	.name = "ion-omap",
	.id = -1,
	.dev = {
		.platform_data = &omap_ion_data,
	},
};

void __init omap_register_ion(void)
{
	int ret;
	printk("ion: %s: omap_register_ion\n", __func__);	
	ret = platform_device_register(&omap_ion_device);
	printk("ion: platform_device_register RET %d\n", ret);
}

void __init omap_ion_init(void)
{
	int i;
	int ret;
	printk("ion: %s: omap_ion_init %d\n", __func__, omap_ion_data.nr);
	
	for (i = 0; i < omap_ion_data.nr; i++)
		printk("ion: omap_ion_init: enter to for\n");
		if (omap_ion_data.heaps[i].type == ION_HEAP_TYPE_CARVEOUT ||
		    omap_ion_data.heaps[i].type == OMAP_ION_HEAP_TYPE_TILER) {
			printk("ion: omap_ion_init: after if\n");
			ret = memblock_remove(omap_ion_data.heaps[i].base,
					      omap_ion_data.heaps[i].size);
			
			if (ret)
				pr_err("ion: memblock remove of %x@%lx failed\n",
				       omap_ion_data.heaps[i].size,
				       omap_ion_data.heaps[i].base);
		}
	printk("ion: omap_ion_init: exit\n");
}
