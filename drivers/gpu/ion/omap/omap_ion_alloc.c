/*
 * drivers/gpu/ion/omap_ion_alloc.c
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/ion.h>
#include <linux/omap_ion.h>
#include <mach/tiler.h>
#include <asm/mach/map.h>
#include <asm/page.h>

#define BITS_PER_PIXEL  8

int omap_ion_mem_alloc(struct ion_client *client,
		struct omap_ion_tiler_alloc_data *sAllocData)
{
	int ret = 0;
	struct ion_allocation_data data;

	data.len = (sAllocData->w * sAllocData->h * sAllocData->fmt) /
					BITS_PER_PIXEL;
	data.align = 0;
	data.flags = 1 << ION_HEAP_TYPE_CARVEOUT;

	sAllocData->handle = ion_alloc(client,
			PAGE_ALIGN(data.len), data.align, data.flags);
	if (!sAllocData->handle) {
		pr_err("%s: Failed to allocate via ion_alloc\n", __func__);
		ret = -ENOMEM;
	}

	return ret;
}

int omap_ion_get_pages(struct ion_client *client, struct ion_handle *handle,
		int *n, unsigned long *ion_addr,
		struct omap_ion_tiler_alloc_data *sAllocData)
{
	size_t len;

	/* validate that the handle exists in this client */
	ion_phys(client, handle, ion_addr, &len);
	*n = len / PAGE_SIZE; /* Number of pages */

	return 0;
}

