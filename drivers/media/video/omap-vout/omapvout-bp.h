/*
 * drivers/media/video/omap/omapvout-bp.h
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __OMAPVOUT_BP_H__
#define __OMAPVOUT_BP_H__

struct omapvout_bp_entry {
	u32 in_use;
	u32 user;
	u32 size;
	unsigned long phy_addr;
	void *virt_addr;
};

struct omapvout_bp {
	struct mutex lock; /* Lock for all buffer pool accesses */
	int ref_cnt;
	int num_entries;
	struct omapvout_bp_entry buf[0];
};

extern struct omapvout_bp *omapvout_bp_create(u8 num_buffers, u32 buf_size);
extern void omapvout_bp_init(struct omapvout_device *vout);
extern void omapvout_bp_destroy(struct omapvout_device *vout);
extern bool omapvout_is_bp_buffer(struct omapvout_device *vout,
			unsigned long phy_addr);
extern int omapvout_bp_alloc(struct omapvout_device *vout, u32 req_size,
			unsigned long *phy_addr, void **virt_addr, u32 *size);
extern int omapvout_bp_release(struct omapvout_device *vout,
			unsigned long phy_addr);

#endif /* __OMAPVOUT_BP_H__ */

