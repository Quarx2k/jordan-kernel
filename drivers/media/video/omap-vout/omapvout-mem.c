/*
 * drivers/media/video/omap/omapvout-mem.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * Based on drivers/media/video/omap24xx/omap24xxvout.c&h
 *
 * Copyright (C) 2005-2006 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/gfp.h>

#include <asm/processor.h>
#include <asm/cacheflush.h>
#include <asm/page.h>

#include "omapvout.h"
#include "omapvout-mem.h"

int omapvout_mem_alloc(u32 size, unsigned long *phy_addr, void **virt_addr)
{
	void *page_addr;

	size = PAGE_ALIGN(size);

	page_addr = alloc_pages_exact(size, GFP_KERNEL);
	if (!page_addr) {
		printk(KERN_ERR "Failed to allocate pages!\n");
		return -ENOMEM;
	}

	*phy_addr = virt_to_phys(page_addr);
	*virt_addr = ioremap_cached(*phy_addr, size);

	DBG("mem_alloc: page/%08x; phy/%08lx; virt/%08x; size/%x\n",
		(unsigned int) page_addr, *phy_addr,
		(unsigned int) *virt_addr, size);

	return 0;
}

void omapvout_mem_free(unsigned long phy_addr, void *virt_addr, u32 size)
{
	void *page_addr;

	size = PAGE_ALIGN(size);
	page_addr = __va((void *)phy_addr);

	free_pages_exact(page_addr, size);

	iounmap(virt_addr);
}

int omapvout_mem_map(struct vm_area_struct *vma, unsigned long phy_addr)
{
	struct page *cpage;
	void *pos;
	u32 start;
	u32 size;

	vma->vm_flags |= VM_RESERVED;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	pos = (void *) phy_addr;
	start = vma->vm_start;
	size = (vma->vm_end - vma->vm_start);

	while (size > 0) {
		cpage = pfn_to_page(((unsigned int)pos) >> PAGE_SHIFT);
		if (vm_insert_page(vma, start, cpage)) {
			printk(KERN_ERR "Failed to insert page to VMA \n");
			return -EAGAIN;
		}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	vma->vm_flags &= ~VM_IO; /* using shared anonymous pages */

	return 0;
}


