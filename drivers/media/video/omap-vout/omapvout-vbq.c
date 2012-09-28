/*
 * drivers/media/video/omap/omapvout-vbq.c
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
 *
 * This file contains the code necessary to interface with the
 * vbq (video buffer queue).
 */

#include <media/videobuf-core.h>
#include <linux/mm.h>

#include "omapvout.h"
#include "omapvout-vbq.h"
#include "omapvout-bp.h"
#include "omapvout-dss.h"
#include "omapvout-mem.h"

/*=== Local Functions ==================================================*/

static void omapvout_vbq_vm_open(struct vm_area_struct *vma)
{
	struct omapvout_device *vout = vma->vm_private_data;
	DBG("vm_open [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_cnt++;
}

static void omapvout_vbq_vm_close(struct vm_area_struct *vma)
{
	struct omapvout_device *vout = vma->vm_private_data;
	DBG("vm_close [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_cnt--;
}

static struct vm_operations_struct vm_ops = {
	.open = omapvout_vbq_vm_open,
	.close = omapvout_vbq_vm_close
};

static void omapvout_acquire_frames(struct omapvout_device *vout,
							int cnt, int size)
{
	struct omapvout_vbq *vbq;
	int i;
	int fcnt;
	unsigned long paddr;
	void *vaddr;
	u32 fsize;

	/* It is assumed that the vout->mtx is locked for this call */

	vbq = vout->vbq;

	vbq->min_size = 0x7FFFFFF; /* Some large value */
	fcnt = 0;
	for (i = 0; i < cnt; i++) {
#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT_BUFPOOL
		if (vout->bp &&	omapvout_bp_alloc(vout, size,
					&paddr, &vaddr, &fsize) == 0) {
			DBG("Alloc'd from the pool\n");
		} else {
#else
		{
#endif
			if (omapvout_mem_alloc(size, &paddr, &vaddr)) {
				DBG("Alloc failed %d\n", i);
				break;
			}
			fsize = size;
		}

		memset(vaddr, 0, fsize);

		vbq->buf[fcnt].size = fsize;
		vbq->buf[fcnt].phy_addr = paddr;
		vbq->buf[fcnt].virt_addr = vaddr;
		vbq->buf[fcnt].released = false;

		fcnt++;

		if (fsize < vbq->min_size)
			vbq->min_size = fsize;
	}

	vbq->cnt = fcnt;
}

static void omapvout_release_frames(struct omapvout_device *vout)
{
	struct omapvout_vbq *vbq;
	int i;
	unsigned long paddr;
	void *vaddr;
	u32 size;

	/* It is assumed that the vout->mtx is locked for this call */

	vbq = vout->vbq;

	for (i = 0; i < vbq->cnt; i++) {
		paddr = vbq->buf[i].phy_addr;
		vaddr = vbq->buf[i].virt_addr;
		size  = vbq->buf[i].size;

#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT_BUFPOOL
		if (omapvout_is_bp_buffer(vout, paddr)) {
			if (omapvout_bp_release(vout, paddr))
				DBG("Error releasing to the pool\n");
		} else {
#else
		{
#endif
			omapvout_mem_free(paddr, vaddr, size);
		}
	}

	vbq->cnt = 0;
	vbq->min_size = 0;
	memset(vbq->buf, 0, sizeof(vbq->buf));
}

static int omapvout_vbq_buf_setup(struct videobuf_queue *q,
			unsigned int *count, unsigned int *size)
{
	struct omapvout_device *vout;
	struct omapvout_vbq *vbq;
	int cnt = *count;
	int sz;

	vout = q->priv_data;
	if (!vout)
		return -EINVAL;

	vbq = vout->vbq;

	/* It is assumed that the video out format is correctly configured */
	if (vout->pix.sizeimage == 0)
		return -EINVAL;

	sz = vout->pix.sizeimage;

	/* Use the existing frames if possible */
	if (cnt <= vbq->cnt && sz <= vbq->min_size)
		goto success;

	if (cnt > VIDEO_MAX_FRAME)
		cnt = VIDEO_MAX_FRAME;

	omapvout_release_frames(vout);
	omapvout_acquire_frames(vout, cnt, sz);

	if (vbq->cnt <= 0) {
		DBG("Buffer allocation failed\n");
		return -ENOMEM;
	}
	cnt = vbq->cnt;

success:
	*count = cnt;
	*size = sz;

	return 0;
}

static int omapvout_vbq_buf_prepare(struct videobuf_queue *q,
			struct videobuf_buffer *vb, enum v4l2_field field)
{
	struct omapvout_device *vout;

	vout = q->priv_data;
	if (!vout)
		return -EINVAL;

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		vb->width = vout->pix.width;
		vb->height = vout->pix.height;
		vb->size = vout->pix.sizeimage;
		vb->field = field;
		vb->privsize = 0;  /* Reusing for buf seq number */
	}

	vb->state = VIDEOBUF_PREPARED;

	return 0;
}

static void omapvout_vbq_buf_queue(struct videobuf_queue *q,
			struct videobuf_buffer *vb)
{
	struct omapvout_device *vout;

	vout = q->priv_data;
	if (!vout)
		return;

	/* Add it to the incoming queue */
	list_add_tail(&vb->queue, &vout->q_list);

	vb->state = VIDEOBUF_QUEUED;

	omapvout_dss_update(vout);
}

static void omapvout_vbq_buf_release(struct videobuf_queue *q,
			struct videobuf_buffer *vb)
{
	struct omapvout_device *vout;
	int i;

	vout = q->priv_data;
	if (!vout)
		return;

	if (vout->vbq->cnt <= 0)
		return;

	vout->vbq->buf[vb->i].released = true;

	for (i = 0; i < vout->vbq->cnt; i++) {
		if (!vout->vbq->buf[i].released)
			break;
	}
	if (i > vout->vbq->cnt) /* All buffers have been released */
		omapvout_release_frames(vout);

	vb->state = VIDEOBUF_NEEDS_INIT;
}

static struct videobuf_queue_ops vbq_buf_ops = {
	.buf_setup = omapvout_vbq_buf_setup,
	.buf_prepare = omapvout_vbq_buf_prepare,
	.buf_queue = omapvout_vbq_buf_queue,
	.buf_release = omapvout_vbq_buf_release
};

static void *omapvout_vbq_alloc(size_t size)
{
	return kzalloc(size, GFP_KERNEL);
}

static void *omapvout_vbq_vmalloc(struct videobuf_buffer *buf)
{
	/* Do nothing */
	return NULL;
}

static int omapvout_vbq_iolock(struct videobuf_queue *q,
		struct videobuf_buffer *vb, struct v4l2_framebuffer *fbuf)
{
	/* Do nothing */
	return 0;
}

static int omapvout_vbq_mmap_op(struct videobuf_queue *q, unsigned int *count,
				unsigned int *size, enum v4l2_memory memory)
{
	/* Do nothing */
	return 0;
}

static int omapvout_vbq_sync(struct videobuf_queue *q,
				struct videobuf_buffer *buf)
{
	/* Do nothing */
	return 0;
}

static int omapvout_vbq_copy_user(struct videobuf_queue *q, char __user *data,
						size_t count, int nonblocking)
{
	/* Do nothing */
	return 0;
}

static int omapvout_vbq_copy_strm(struct videobuf_queue *q, char __user *data,
			size_t count, size_t pos, int vbihack, int nonblocking)
{
	/* Do nothing */
	return 0;
}

static int omapvout_vbq_mmap_free(struct videobuf_queue *q)
{
	/* Do nothing */
	return 0;
}

static int omapvout_vbq_mmap_mapper(struct videobuf_queue *q,
					struct vm_area_struct *vma)
{
	struct omapvout_device *vout;
	int rc;
	int idx;
	int cnt;
	u32 offset;

	vout = q->priv_data;
	if (!vout)
		return -EINVAL;

	cnt = vout->vbq->cnt;

	/* look for the buffer to map */
	offset = (vma->vm_pgoff << PAGE_SHIFT);
	for (idx = 0; idx < cnt; idx++) {
		if (q->bufs[idx]->boff == offset)
			break;
	}

	if (idx >= cnt) {
		DBG("Invalid offset 0x%lx\n", (unsigned long) offset);
		return -EINVAL;
	}

	DBG("omapvout_vbq_mmap %d\n", idx);

	vma->vm_ops = &vm_ops;
	vma->vm_private_data = (void *) vout;

	rc = omapvout_mem_map(vma, vout->vbq->buf[idx].phy_addr);
	if (rc != 0) {
		DBG("Failed mem_map %d\n", rc);
		return rc;
	}

	q->bufs[idx]->baddr = vout->vbq->buf[idx].phy_addr;

	return 0;
}

static struct videobuf_qtype_ops vbq_ops = {
	.magic = MAGIC_QTYPE_OPS,
	.alloc = omapvout_vbq_alloc,
	.vmalloc = omapvout_vbq_vmalloc,
	.iolock = omapvout_vbq_iolock,
	.mmap = omapvout_vbq_mmap_op,
	.sync = omapvout_vbq_sync,
	.video_copy_to_user = omapvout_vbq_copy_user,
	.copy_stream = omapvout_vbq_copy_strm,
	.mmap_free = omapvout_vbq_mmap_free,
	.mmap_mapper = omapvout_vbq_mmap_mapper
};

/*=== Public Interface Functions =========================================*/

int omapvout_vbq_init(struct omapvout_device *vout)
{
	struct omapvout_vbq *vbq;
	int rc;

	/* It is assumed that the caller has locked the vout mutex */

	if (vout->vbq) {
		rc = -EINVAL;
		goto failed;
	}

	vout->vbq = kzalloc(sizeof(struct omapvout_vbq), GFP_KERNEL);
	if (vout->vbq == NULL) {
		rc = -ENOMEM;
		goto failed;
	}

	vbq = vout->vbq;

	vbq->cnt = 0;
	vbq->min_size = 0;

	spin_lock_init(&vbq->lock);
	videobuf_queue_core_init(&vout->queue, &vbq_buf_ops, NULL,
			&vbq->lock, V4L2_BUF_TYPE_VIDEO_OUTPUT,
			V4L2_FIELD_NONE, sizeof(struct videobuf_buffer),
			vout, &vbq_ops);

	return 0;

failed:
	return rc;
}

void omapvout_vbq_destroy(struct omapvout_device *vout)
{
	/* It is assumed that the caller has locked the vout mutex */

	if (vout->vbq) {
		omapvout_release_frames(vout);
		kfree(vout->vbq);
		vout->vbq = NULL;
	}
}

int omapvout_vbq_buf_cnt(struct omapvout_device *vout)
{
	return vout->vbq->cnt;
}

