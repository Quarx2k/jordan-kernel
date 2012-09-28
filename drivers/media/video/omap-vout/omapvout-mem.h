/*
 * drivers/media/video/omap/omapvout-mem.h
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

#ifndef __OMAPVOUT_MEM_H__
#define __OMAPVOUT_MEM_H__

extern int  omapvout_mem_alloc(u32 size, unsigned long *phy_addr,
						void **virt_addr);
extern void omapvout_mem_free(unsigned long phy_addr, void *virt_addr,
						u32 size);
extern int  omapvout_mem_map(struct vm_area_struct *vma,
						unsigned long phy_addr);

#endif /* __OMAPVOUT_MEM_H__ */

