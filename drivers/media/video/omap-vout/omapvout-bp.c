/*
 * drivers/media/video/omap/omapvout-bp.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/mm.h>

#include "omapvout.h"
#include "omapvout-mem.h"
#include "omapvout-bp.h"

#define OMAPVOUT_BP_MAX_BUFFERS (20)

struct omapvout_bp *omapvout_bp_create(u8 num_bufs, u32 buf_size)
{
	struct omapvout_bp *bp;
	int num = num_bufs;
	int size;
	int i;
	unsigned long paddr;
	void *vaddr;
	u32 bsize;

	if (num > OMAPVOUT_BP_MAX_BUFFERS)
		num = OMAPVOUT_BP_MAX_BUFFERS;

	size = sizeof(struct omapvout_bp) +
			(num * sizeof(struct omapvout_bp_entry));
	bp = kzalloc(size, GFP_KERNEL);
	if (!bp)
		return NULL;

	mutex_init(&bp->lock);
	mutex_lock(&bp->lock);

	bsize = PAGE_ALIGN(buf_size);
	bp->num_entries = num;
	for (i = 0; i < num; i++) {
		if (omapvout_mem_alloc(bsize, &paddr, &vaddr) == 0) {
			bp->buf[i].size = bsize;
			bp->buf[i].phy_addr = paddr;
			bp->buf[i].virt_addr = vaddr;
		}
	}

	mutex_unlock(&bp->lock);

	return bp;
}

void omapvout_bp_init(struct omapvout_device *vout)
{
	struct omapvout_bp *bp = vout->bp;

	mutex_lock(&bp->lock);

	bp->ref_cnt++;

	mutex_unlock(&bp->lock);
}

void omapvout_bp_destroy(struct omapvout_device *vout)
{
	struct omapvout_bp *bp = vout->bp;
	int i;

	if (bp == NULL)
		return;

	vout->bp = NULL;

	mutex_lock(&bp->lock);

	if (bp->ref_cnt > 0)
		bp->ref_cnt--;

	if (bp->ref_cnt > 0) {
		/* Release any buffers still alloc'd to this user */
		for (i = 0; i < bp->num_entries; i++) {
			if (bp->buf[i].size == 0)
				continue;

			if (bp->buf[i].in_use && bp->buf[i].user == vout->id)
				bp->buf[i].in_use = 0;
		}
		mutex_unlock(&bp->lock);
	} else {
		/* Destroy everything */
		for (i = 0; i < bp->num_entries; i++) {
			if (bp->buf[i].size == 0)
				continue;

			if (bp->buf[i].in_use)
				DBG("Destroying an in-use buffer\n");

			omapvout_mem_free(bp->buf[i].phy_addr,
					bp->buf[i].virt_addr,
					bp->buf[i].size);
		}
		mutex_unlock(&bp->lock);

		mutex_destroy(&bp->lock);
		kfree(bp);
	}
}

bool omapvout_is_bp_buffer(struct omapvout_device *vout,
			unsigned long phy_addr)
{
	struct omapvout_bp *bp = vout->bp;
	int i;

	if (!bp)
		return false;

	for (i = 0; i < bp->num_entries; i++) {
		if (phy_addr == bp->buf[i].phy_addr)
			return true;
	}

	return false;
}

int omapvout_bp_alloc(struct omapvout_device *vout, u32 req_size,
			unsigned long *phy_addr, void **virt_addr, u32 *size)
{
	struct omapvout_bp *bp = vout->bp;
	int i;
	int rc = -ENOMEM;

	if (!bp)
		return -ENOMEM;

	mutex_lock(&bp->lock);

	for (i = 0; i < bp->num_entries; i++) {
		if (bp->buf[i].in_use == 0 && bp->buf[i].size >= req_size) {
			bp->buf[i].in_use = 1;
			bp->buf[i].user = vout->id;
			*phy_addr = bp->buf[i].phy_addr;
			*virt_addr = bp->buf[i].virt_addr;
			*size = bp->buf[i].size;
			rc = 0;
			break;
		}
	}

	mutex_unlock(&bp->lock);

	return rc;
}

int omapvout_bp_release(struct omapvout_device *vout, unsigned long phy_addr)
{
	struct omapvout_bp *bp = vout->bp;
	int i;
	int rc = -EINVAL;

	mutex_lock(&bp->lock);

	for (i = 0; i < bp->num_entries; i++) {
		if (bp->buf[i].phy_addr == phy_addr) {
			if (bp->buf[i].in_use)
				bp->buf[i].in_use = 0;
			else
				DBG("Releasing an unused buffer\n");

			/* Found the buffer, so say so */
			rc = 0;

			break;
		}
	}

	mutex_unlock(&bp->lock);

	return rc;
}

