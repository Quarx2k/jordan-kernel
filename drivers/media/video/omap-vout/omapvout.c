/*
 * drivers/media/video/omap/omapvout.c
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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <plat/io.h>
#include <plat/board.h>

#include "omapvout.h"
#include "omapvout-dss.h"
#include "omapvout-mem.h"
#include "omapvout-vbq.h"
#include "omapvout-bp.h"

#define MODULE_NAME "omapvout"

/* list of image formats supported by OMAP2 video pipelines */
const static struct v4l2_fmtdesc omap2_formats[] = {
{
	/* Note:  V4L2 defines RGB565 as:
	 *      Byte 0                    Byte 1
	 *      g2 g1 g0 r4 r3 r2 r1 r0   b4 b3 b2 b1 b0 g5 g4 g3
	 *
	 * We interpret RGB565 as:
	 *      Byte 0                    Byte 1
	 *      g2 g1 g0 b4 b3 b2 b1 b0   r4 r3 r2 r1 r0 g5 g4 g3
	 */
	.description = "RGB565, le",
	.pixelformat = V4L2_PIX_FMT_RGB565,
},
{
	/* Note:  V4L2 defines RGB32 as: RGB-8-8-8-8  we use
	 *        this for RGB24 unpack mode, the last 8 bits are ignored
	 */
	.description = "RGB32, le",
	.pixelformat = V4L2_PIX_FMT_RGB32,
},
{
	.description = "YUYV (YUV 4:2:2), packed",
	.pixelformat = V4L2_PIX_FMT_YUYV,
},
{
	.description = "UYVY (YUV 4:2:2), packed",
	.pixelformat = V4L2_PIX_FMT_UYVY,
},
};

#define NUM_OUTPUT_FORMATS (sizeof(omap2_formats)/sizeof(omap2_formats[0]))

/* This is a way to allow other components to force a desired rotation.
 * This will take effect when streaming is next enabled.
 */
struct omapvout_override {
	int dirty;
	int force_rotation_dirty;
	int force_rotation_enable;
	int forced_rotation;
	int client_rotation;
};

#define NUM_PLANES (3)
static struct omapvout_override gOverride[NUM_PLANES];

/*=== Local Functions ==================================================*/

static void omapvout_chk_overrides(struct omapvout_device *vout)
{
	struct omapvout_override *ovr;

	ovr = &gOverride[vout->id];

	if (!ovr->dirty)
		return;

	if (ovr->force_rotation_dirty) {
		ovr->force_rotation_dirty = 0;
		if (ovr->force_rotation_enable) {
			ovr->client_rotation = vout->rotation;
			vout->rotation = ovr->forced_rotation;
		} else {
			vout->rotation = ovr->client_rotation;
		}
		printk(KERN_ERR "omapvout_chk_overrides/%d/%d/%d\n", \
			ovr->force_rotation_enable, \
			ovr->forced_rotation, ovr->client_rotation);
	}

	ovr->dirty = 0;
}

static int omapvout_crop_to_size(struct v4l2_rect *rect, int w, int h)
{
	struct v4l2_rect try;
	int t;

	try = *rect;

	if (try.left < 0)
		try.left = 0;
	if (try.top < 0)
		try.top = 0;
	if (try.width > w)
		try.width = w;
	if (try.height > h)
		try.height = h;
	t = ((try.left + try.width) - w);
	if (t > 0)
		try.width = w - t;
	t = ((try.top + try.height) - h);
	if (t > 0)
		try.height = h - t;
	try.width &= ~1;
	try.height &= ~1;

	if (try.width <= 0 || try.height <= 0)
		return -EINVAL;

	*rect = try;

	return 0;
}

static int omapvout_try_pixel_format(struct omapvout_device *vout,
				struct v4l2_pix_format *pix)
{
	int ifmt;
	int bpp = 0;

	if (pix->width > vout->max_video_width)
		pix->width = vout->max_video_width;

	if (pix->height > vout->max_video_height)
		pix->height = vout->max_video_height;

	for (ifmt = 0; ifmt < NUM_OUTPUT_FORMATS; ifmt++) {
		if (pix->pixelformat == omap2_formats[ifmt].pixelformat)
			break;
	}

	if (ifmt >= NUM_OUTPUT_FORMATS)
		ifmt = 0;

	pix->pixelformat = omap2_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->priv = 0;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_RGB565:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = 2;
		break;
	case V4L2_PIX_FMT_RGB32:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = 4;
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = 2;
		break;
	}

	pix->bytesperline = pix->width * bpp;
	pix->sizeimage = pix->bytesperline * pix->height;

	return 0;
}

static int omapvout_try_window(struct omapvout_device *vout,
					struct v4l2_window *win)
{
	int rc = 0;

	rc = omapvout_crop_to_size(&win->w, vout->disp_width,
							vout->disp_height);
	if (rc == 0)
		win->field = V4L2_FIELD_NONE;

	return rc;
}

static int omapvout_try_crop(struct omapvout_device *vout,
					struct v4l2_rect *crop)
{
	return omapvout_crop_to_size(crop, vout->pix.width, vout->pix.height);
}

/* Make sure the input size, window rectangle, crop rectangle, and rotation
 * parameters together make up a valid configuration for the hardware
 */
static int omapvout_validate_cfg(struct omapvout_device *vout)
{
	int rc = 0;
	int win_w, win_h;
	int crp_w, crp_h;

	/* Is it assumed:
	 * - The rotation value denotes 0, 90, 180, or 270
	 * - The input size is valid based on the platform limits
	 * - The output rectangle is within the display area
	 * - The crop rectangle is within the input frame
	 */

	/* Validate scaling */
	win_w = vout->win.w.width;
	win_h = vout->win.w.height;
	crp_w = vout->crop.width;
	crp_h = vout->crop.height;

	if ( vout->rotation == 0 || vout->rotation == 2 ) {
		win_w = vout->win.w.width;
		win_h = vout->win.w.height;
	} else {
		win_w = vout->win.w.height;
		win_h = vout->win.w.width;
	}

	/* Down-scaling */
	if (((win_w < crp_w) && ((win_w * 4) < crp_w)) ||
	    ((win_h < crp_w) && ((win_h * 4) < crp_h)))
		rc = -EINVAL;

	/* Up-scaling */
	if (((win_w > crp_w) && ((crp_w * 8) < win_w)) ||
	    ((win_h > crp_h) && ((crp_h * 8) < win_h)))
		rc = -EINVAL;

	return rc;
}

static void omapvout_free_resources(struct omapvout_device *vout)
{
	if (vout == NULL)
		return;

#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT_BUFPOOL
	if (vout->bp != NULL)
		omapvout_bp_destroy(vout);
#endif

	video_set_drvdata(&vout->vdev, NULL);
	kfree(vout);
}

/*=== Public Kernel Functions =========================================*/

int omapvout_force_rotation(int plane, int enable, int rotation)
{
	struct omapvout_override *ovr;
	int en;

	if (plane < 0 || plane >= NUM_PLANES) {
		printk(KERN_ERR "Invalid plane (%d)\n", plane);
		return -1;
	}

	ovr = &gOverride[plane];
	if (ovr->force_rotation_enable == enable &&
	    ovr->forced_rotation == rotation)
		return 0;

	en = (enable) ? 1 : 0;
	if (en) {
		if (rotation < 0 || rotation > 3) {
			printk(KERN_ERR "Invalid rotation (%d)\n", rotation);
			return -1;
		}

		ovr->forced_rotation = rotation;
	}

	ovr->force_rotation_dirty = 1;
	ovr->force_rotation_enable = en;
	ovr->dirty = 1;

	return 0;
}
EXPORT_SYMBOL(omapvout_force_rotation);

/*=== V4L2 Interface Functions =========================================*/

static int omapvout_open(struct file *file)
{
	struct omapvout_device *vout;
	u16 w, h;
	int rc;

	DBG("omapvout_open\n");

	vout = video_drvdata(file);

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	/* We only support single open */
	if (vout->opened) {
		DBG("Device already opened\n");
		rc = -EBUSY;
		goto failed;
	}

	rc = omapvout_dss_open(vout, &w, &h);
	if (rc != 0)
		goto failed;

	DBG("Overlay Display %dx%d\n", w, h);

	if (w == 0 || h == 0) {
		printk(KERN_ERR "Invalid display resolution\n");
		rc = -EINVAL;
		goto failed;
	}

	rc = omapvout_vbq_init(vout);
	if (rc != 0)
		goto failed;

	vout->disp_width = w;
	vout->disp_height = h;
	vout->opened = 1;

	memset(&vout->pix, 0, sizeof(vout->pix));
	vout->pix.width = w;
	vout->pix.height = h;
	vout->pix.field = V4L2_FIELD_NONE;
	vout->pix.pixelformat = V4L2_PIX_FMT_RGB565; /* Arbitrary */
	vout->pix.colorspace = V4L2_COLORSPACE_SRGB; /* Arbitrary */
	vout->pix.bytesperline = w * 2;
	vout->pix.sizeimage = w * h * 2;

	memset(&vout->win, 0, sizeof(vout->win));
	vout->win.w.width = w;
	vout->win.w.height = h;
	vout->win.field = V4L2_FIELD_NONE;

	memset(&vout->crop, 0, sizeof(vout->crop));
	vout->crop.width = w;
	vout->crop.height = h;

	memset(&vout->fbuf, 0, sizeof(vout->fbuf));

	vout->rotation = 0;
	vout->bg_color = 0;

	vout->mmap_cnt = 0;

	omapvout_chk_overrides(vout);

	mutex_unlock(&vout->mtx);

	file->private_data = vout;

	return 0;

failed:
	mutex_unlock(&vout->mtx);
	return rc;
}

static int omapvout_release(struct file *file)
{
	struct omapvout_device *vout;

	DBG("omapvout_release\n");

	vout = video_drvdata(file);

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	if (vout->queue.streaming) {
		omapvout_dss_disable(vout);
		videobuf_streamoff(&vout->queue);
	}

	if (vout->mmap_cnt) {
		vout->mmap_cnt = 0;
		DBG("Releasing with non-zero mmap_cnt\n");
	}

	omapvout_dss_release(vout);

	omapvout_vbq_destroy(vout);

	vout->opened = 0;

	mutex_unlock(&vout->mtx);

	file->private_data = NULL;

	return 0;
}

static int omapvout_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct omapvout_device *vout;
	int rc;

	vout = video_drvdata(file);

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	rc = videobuf_mmap_mapper(&vout->queue, vma);
	if (rc != 0)
		goto failed;

	vout->mmap_cnt++;

	mutex_unlock(&vout->mtx);

	return 0;

failed:
	mutex_unlock(&vout->mtx);
	return rc;
}

static int omapvout_vidioc_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	struct omapvout_device *vout = priv;

	memset(cap, 0, sizeof(*cap));
	strncpy(cap->driver, MODULE_NAME, sizeof(cap->driver));
	strncpy(cap->card, vout->vdev.name, sizeof(cap->card));
	cap->bus_info[0] = '\0';
	cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;

	return 0;
}

static int omapvout_vidioc_enum_output(struct file *file, void *priv,
				struct v4l2_output *output)
{
	int index = output->index;

	if (index > 0)
		return -EINVAL;

	memset(output, 0, sizeof(*output));
	output->index = index;

	strncpy(output->name, "video out", sizeof(output->name));
	output->type = V4L2_OUTPUT_TYPE_MODULATOR;

	return 0;
}

static int omapvout_vidioc_g_output(struct file *file, void *priv,
				unsigned int *i)
{
	*i = 0;

	return 0;
}

static int omapvout_vidioc_s_output(struct file *file, void *priv,
				unsigned int i)
{
	if (i > 0)
		return -EINVAL;

	return 0;
}

static int omapvout_vidioc_enum_fmt_vid_overlay(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	int index = f->index;
	enum v4l2_buf_type type = f->type;

	if (index >= NUM_OUTPUT_FORMATS)
		return -EINVAL;

	memset(f, 0, sizeof(*f));
	f->index = index;
	f->type = type;
	f->flags = omap2_formats[index].flags;
	strncpy(f->description, omap2_formats[index].description,
					sizeof(f->description));
	f->pixelformat = omap2_formats[index].pixelformat;

	return 0;
}

static int omapvout_vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	/* Same formats as the overlay */
	return omapvout_vidioc_enum_fmt_vid_overlay(file, priv, f);
}

static int omapvout_vidioc_g_fmt_vid_overlay(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_window *win = &f->fmt.win;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	/*
	 * The API has a bit of a problem here. We're returning a v4l2_window
	 * structure, but that structure contains pointers to variable-sized
	 * objects for clipping rectangles and clipping bitmaps.  We will just
	 * return NULLs for those pointers.
	 */

	mutex_lock(&vout->mtx);

	memset(win, 0, sizeof(*win));
	win->w = vout->win.w;
	win->field = vout->win.field;
	win->chromakey = vout->win.chromakey;
	win->global_alpha = vout->win.global_alpha;

	mutex_unlock(&vout->mtx);

	return 0;
}

static int omapvout_vidioc_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	memset(pix, 0, sizeof(*pix));
	*pix = vout->pix;

	mutex_unlock(&vout->mtx);

	return 0;
}

static int omapvout_vidioc_try_fmt_vid_overlay(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_window *win = &f->fmt.win;
	int rc;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	rc = omapvout_try_window(vout, win);

	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_try_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rc;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	rc = omapvout_try_pixel_format(vout, pix);

	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_s_fmt_vid_overlay(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_window *win = &f->fmt.win;
	int rc = 0;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	if (vout->queue.streaming) {
		rc = -EBUSY;
		goto failed;
	}

	omapvout_chk_overrides(vout);

	rc = omapvout_try_window(vout, win);
	if (rc != 0)
		goto failed;

	vout->win.w = win->w;
	vout->win.field = win->field;
	vout->win.chromakey = win->chromakey;
	vout->win.global_alpha = win->global_alpha;

	/* Streaming has to be disabled, so config the hardware
	 * later when streaming is enabled
	 */

failed:
	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct omapvout_device *vout = priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rc = 0;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	if (vout->queue.streaming) {
		rc = -EBUSY;
		goto failed;
	}

	omapvout_chk_overrides(vout);

	rc = omapvout_try_pixel_format(vout, pix);
	if (rc != 0)
		goto failed;

	memcpy(&vout->pix, pix, sizeof(*pix));

	/* Default the cropping rectangle to the input frame size */
	vout->crop.left = 0;
	vout->crop.top = 0;
	vout->crop.width = vout->pix.width;
	vout->crop.height = vout->pix.height;

	/* Streaming has to be disabled, so config the hardware
	 * later when streaming is enabled
	 */

failed:
	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_cropcap(struct file *file, void *priv,
				struct v4l2_cropcap *ccap)
{
	struct omapvout_device *vout = priv;
	enum v4l2_buf_type type = ccap->type;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	if (type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	mutex_lock(&vout->mtx);

	memset(ccap, 0, sizeof(*ccap));
	ccap->type = type;
	ccap->bounds.width = vout->pix.width & ~1;
	ccap->bounds.height = vout->pix.height & ~1;
	ccap->defrect.left = 0;
	ccap->defrect.top = 0;
	ccap->defrect.width = ccap->bounds.width;
	ccap->defrect.left = ccap->bounds.height;
	ccap->pixelaspect.numerator = 1;
	ccap->pixelaspect.denominator = 1;

	mutex_unlock(&vout->mtx);

	return 0;
}

static int omapvout_vidioc_g_crop(struct file *file, void *priv,
				struct v4l2_crop *crop)
{
	struct omapvout_device *vout = priv;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	mutex_lock(&vout->mtx);

	crop->c = vout->crop;

	mutex_unlock(&vout->mtx);

	return 0;
}

static int omapvout_vidioc_s_crop(struct file *file, void *priv,
				struct v4l2_crop *crop)
{
	struct omapvout_device *vout = priv;
	struct v4l2_rect rect = crop->c;
	int rc = 0;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	mutex_lock(&vout->mtx);

	if (vout->queue.streaming) {
		rc = -EBUSY;
		goto failed;
	}

	omapvout_chk_overrides(vout);

	rc = omapvout_try_crop(vout, &rect);
	if (rc != 0)
		goto failed;

	vout->crop = rect;

	/* Streaming has to be disabled, so config the hardware
	 * later when streaming is enabled
	 */

failed:
	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_reqbufs(struct file *file, void *priv,
				struct v4l2_requestbuffers *req)
{
	struct omapvout_device *vout = priv;
	int rc = 0;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	/* A limitation of this implementation */
	if (req->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	if (req->count == 0) {
		videobuf_queue_cancel(&vout->queue);
		return 0;
	}

	mutex_lock(&vout->mtx);

	/* Don't allow new buffers when some are still mapped */
	if (vout->mmap_cnt) {
		printk(KERN_ERR "Buffers are still mapped\n");
		rc = -EBUSY;
		goto failed;
	}

	INIT_LIST_HEAD(&vout->q_list);

	videobuf_reqbufs(&vout->queue, req);

	mutex_unlock(&vout->mtx);

	return 0;

failed:
	mutex_unlock(&vout->mtx);
	return rc;
}

static int omapvout_vidioc_querybuf(struct file *file, void *priv,
				struct v4l2_buffer *b)
{
	struct omapvout_device *vout = priv;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	return videobuf_querybuf(&vout->queue, b);
}

static int omapvout_vidioc_qbuf(struct file *file, void *priv,
				struct v4l2_buffer *b)
{
	int rc;
	struct omapvout_device *vout = priv;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	DBG("Q'ing Frame %d\n", b->index);

	mutex_lock(&vout->mtx);
	omapvout_chk_overrides(vout);
	rc = videobuf_qbuf(&vout->queue, b);
	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_dqbuf(struct file *file, void *priv,
				struct v4l2_buffer *b)
{
	struct omapvout_device *vout = priv;
	int block = 0;
	int rc;

	if (file->f_flags & O_NONBLOCK)
		block = 1;

	rc = videobuf_dqbuf(&vout->queue, b, block);

	DBG("DQ'ing Frame %d\n", b->index);

	return rc;
}

static int omapvout_vidioc_streamon(struct file *file, void *priv,
				enum v4l2_buf_type type)
{
	struct omapvout_device *vout = priv;
	int rc;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	omapvout_chk_overrides(vout);

	/* Not sure how else to do this.  We can't truly validate the
	 * configuration until all of the pieces have been provided, like
	 * input, output, crop sizes and rotation.  This is the only point
	 * where we can be sure the client has provided all the data, thus
	 * the only place to make sure we don't cause a DSS failure.
	 */
	rc = omapvout_validate_cfg(vout);
	if (rc) {
		printk(KERN_ERR "Configuration Validation Failed\n");
		goto failed;
	}

	rc = omapvout_dss_enable(vout);
	if (rc) {
		printk(KERN_ERR "DSS Enable Failed\n");
		goto failed;
	}

	rc = videobuf_streamon(&vout->queue);
	if (rc)
		omapvout_dss_disable(vout);

failed:
	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_streamoff(struct file *file, void *priv,
				enum v4l2_buf_type type)
{
	struct omapvout_device *vout = priv;
	int rc = 0;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	omapvout_dss_disable(vout);

	rc = videobuf_streamoff(&vout->queue);

	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_queryctrl(struct file *file, void *priv,
				struct v4l2_queryctrl *qctrl)
{
	switch (qctrl->id) {
	case V4L2_CID_ROTATE:
		v4l2_ctrl_query_fill(qctrl, 0, 270, 90, 0);
		break;
	case V4L2_CID_BG_COLOR:
		v4l2_ctrl_query_fill(qctrl, 0, 0xFFFFFF, 1, 0);
		break;
	default:
		qctrl->name[0] = '\0';
		return -EINVAL;
	}

	return 0;
}

static int omapvout_vidioc_g_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct omapvout_device *vout = priv;
	int rc = 0;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		ctrl->value = vout->rotation * 90;
		break;
	case V4L2_CID_BG_COLOR:
		ctrl->value = vout->bg_color;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_s_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct omapvout_device *vout = priv;
	int v = ctrl->value;
	int rc = 0;

	if (vout == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&vout->mtx);

	if (vout->queue.streaming) {
		rc = -EBUSY;
		goto failed;
	}

	switch (ctrl->id) {
	case V4L2_CID_ROTATE:
		if (!omapvout_dss_is_rotation_supported(vout) && v != 0) {
			rc = -EINVAL;
		} else if (v == 0 || v == 90 || v == 180 || v == 270) {
			vout->rotation = v / 90;
		} else {
			printk(KERN_ERR "Invalid rotation %d\n", v);
			rc = -ERANGE;
		}
		if (rc == 0 && gOverride[vout->id].force_rotation_enable) {
			gOverride[vout->id].client_rotation = vout->rotation;
			vout->rotation = gOverride[vout->id].forced_rotation;
			gOverride[vout->id].force_rotation_dirty = 0;
		}
		break;
	case V4L2_CID_BG_COLOR:
		if (v < 0 || v > 0xFFFFFF) {
			printk(KERN_ERR "Invalid BG color 0x%08lx\n",
							(unsigned long) v);
			rc = -ERANGE;
		} else {
			vout->bg_color = v;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}

	/* Streaming has to be disabled, so config the hardware
	 * later when streaming is enabled
	 */

failed:
	mutex_unlock(&vout->mtx);

	return rc;
}

static int omapvout_vidioc_g_fbuf(struct file *file, void *priv,
				struct v4l2_framebuffer *a)
{
	struct omapvout_device *vout = priv;

	mutex_lock(&vout->mtx);

	if (vout->dss->overlay->id == OMAP_DSS_VIDEO1) {
		a->capability = V4L2_FBUF_CAP_EXTERNOVERLAY |
				V4L2_FBUF_CAP_GLOBAL_ALPHA |
				V4L2_FBUF_CAP_CHROMAKEY	|
				V4L2_FBUF_CAP_SRC_CHROMAKEY;
	} else {
		a->capability = V4L2_FBUF_CAP_EXTERNOVERLAY |
				V4L2_FBUF_CAP_LOCAL_ALPHA |
				V4L2_FBUF_CAP_GLOBAL_ALPHA |
				V4L2_FBUF_CAP_CHROMAKEY	|
				V4L2_FBUF_CAP_SRC_CHROMAKEY;
	}
	a->flags = vout->fbuf.flags;
	memset(&a->fmt, 0, sizeof(a->fmt));

	mutex_unlock(&vout->mtx);

	return 0;
}

static int omapvout_vidioc_s_fbuf(struct file *file, void *priv,
				struct v4l2_framebuffer *a)
{
	struct omapvout_device *vout = priv;

	/* OMAP DSS doesn't support SRC & DST colorkey together */
	if ((a->flags & V4L2_FBUF_FLAG_CHROMAKEY) &&
			(a->flags & V4L2_FBUF_FLAG_SRC_CHROMAKEY))
		return -EINVAL;

	/* OMAP DSS doesn't support DST colorkey and alpha blending together */
	if ((a->flags & V4L2_FBUF_FLAG_CHROMAKEY) &&
			(a->flags & V4L2_FBUF_FLAG_LOCAL_ALPHA))
		return -EINVAL;

	mutex_lock(&vout->mtx);

	/* HACK: OMAP DSS doesn't support local alpha for video 1 --
	 * so if it requests it assume this is meant to be local alpha
	 * for gfx*/

	vout->fbuf.flags = a->flags;

	mutex_unlock(&vout->mtx);

	return 0;
}

/*=== Driver Functions =================================================*/

static struct v4l2_file_operations omapvout_fops = {
	.owner = THIS_MODULE,
	.open = omapvout_open,
	.release = omapvout_release,
	.mmap = omapvout_mmap,
	.ioctl = video_ioctl2,
};

static const struct v4l2_ioctl_ops omapvout_ioctl_ops = {
	.vidioc_querycap = omapvout_vidioc_querycap,
	.vidioc_enum_output = omapvout_vidioc_enum_output,
	.vidioc_g_output = omapvout_vidioc_g_output,
	.vidioc_s_output = omapvout_vidioc_s_output,
	.vidioc_enum_fmt_vid_overlay = omapvout_vidioc_enum_fmt_vid_overlay,
	.vidioc_enum_fmt_vid_out = omapvout_vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_overlay = omapvout_vidioc_g_fmt_vid_overlay,
	.vidioc_g_fmt_vid_out = omapvout_vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_overlay = omapvout_vidioc_try_fmt_vid_overlay,
	.vidioc_try_fmt_vid_out = omapvout_vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_overlay = omapvout_vidioc_s_fmt_vid_overlay,
	.vidioc_s_fmt_vid_out = omapvout_vidioc_s_fmt_vid_out,
	.vidioc_cropcap = omapvout_vidioc_cropcap,
	.vidioc_g_crop = omapvout_vidioc_g_crop,
	.vidioc_s_crop = omapvout_vidioc_s_crop,
	.vidioc_reqbufs = omapvout_vidioc_reqbufs,
	.vidioc_querybuf = omapvout_vidioc_querybuf,
	.vidioc_qbuf = omapvout_vidioc_qbuf,
	.vidioc_dqbuf = omapvout_vidioc_dqbuf,
	.vidioc_streamon = omapvout_vidioc_streamon,
	.vidioc_streamoff = omapvout_vidioc_streamoff,
	.vidioc_queryctrl = omapvout_vidioc_queryctrl,
	.vidioc_g_ctrl = omapvout_vidioc_g_ctrl,
	.vidioc_s_ctrl = omapvout_vidioc_s_ctrl,
	.vidioc_g_fbuf = omapvout_vidioc_g_fbuf,
	.vidioc_s_fbuf = omapvout_vidioc_s_fbuf,
};

static struct video_device omapvout_devdata = {
	.name = MODULE_NAME,
	.fops = &omapvout_fops,
	.ioctl_ops = &omapvout_ioctl_ops,
	.vfl_type = VFL_TYPE_GRABBER,
	.release = video_device_release,
	.minor = -1,
};

static int __init omapvout_probe_device(struct omap_vout_config *cfg,
					struct omapvout_bp *bp,
					enum omap_plane plane, int vid)
{
	struct omapvout_device *vout = NULL;
	int rc = 0;

	DBG("omapvout_probe_device %d %d\n", plane, vid);

	vout = kzalloc(sizeof(struct omapvout_device), GFP_KERNEL);
	if (vout == NULL) {
		rc = -ENOMEM;
		goto err0;
	}

	mutex_init(&vout->mtx);

	vout->max_video_width = cfg->max_width;
	vout->max_video_height = cfg->max_height;
	vout->max_video_buffer_size = cfg->max_buffer_size;

	rc = omapvout_dss_init(vout, plane);
	if (rc != 0) {
		printk(KERN_ERR "DSS init failed\n");
		kfree(vout);
		goto err0;
	}

#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT_BUFPOOL
	vout->bp = bp;
	omapvout_bp_init(vout);
#endif

	/* register the V4L2 interface */
	vout->vdev = omapvout_devdata;
	video_set_drvdata(&vout->vdev, vout);
	if (video_register_device(&vout->vdev, VFL_TYPE_GRABBER, vid) < 0) {
		printk(KERN_ERR MODULE_NAME": could not register with V4L2\n");
		rc = -EINVAL;
		goto cleanup;
	}

	vout->id = plane;

	memset(gOverride, 0, sizeof(gOverride));

	return 0;

cleanup:
	omapvout_free_resources(vout);
err0:
	return rc;
}

/* Some reasonable defaults if the platform does not supply a config */
static struct omap_vout_config default_cfg = {
	.max_width = 864,
	.max_height = 648,
	.max_buffer_size = 0x112000, /* (w * h * 2) page aligned */
	.num_buffers = 6,
	.num_devices = 2,
	.device_ids = {1, 2},
};

static int __init omapvout_probe(struct platform_device *pdev)
{
	struct omapvout_bp *bp = NULL;
	struct omap_vout_config *cfg;
	int i;
	int rc = 0;
	static const enum omap_plane planes[] = {
		OMAP_DSS_VIDEO1,
		OMAP_DSS_VIDEO2,
	};

	if (pdev->dev.platform_data) {
		cfg = pdev->dev.platform_data;
	} else {
		DBG("omapvout_probe - using default configuration\n");
		cfg = &default_cfg;
	}

	if (cfg->max_width > 2048) /* Hardware limitation */
		cfg->max_width = 2048;
	if (cfg->max_height > 2048) /* Hardware limitation */
		cfg->max_height = 2048;
	if (cfg->num_buffers > 16) /* Arbitrary limitation */
		cfg->num_buffers = 16;
	if (cfg->num_devices > 2) /* Hardware limitation */
		cfg->num_devices = 2;
	for (i = 0; i < cfg->num_devices; i++)
		if (cfg->device_ids[i] > 64)
			cfg->device_ids[i] = -1;

#ifdef CONFIG_VIDEO_OMAP_VIDEOOUT_BUFPOOL
	bp = omapvout_bp_create(cfg->num_buffers, cfg->max_buffer_size);
#endif

	for (i = 0; i < cfg->num_devices; i++) {
		rc = omapvout_probe_device(cfg, bp, planes[i],
						cfg->device_ids[i]);
		if (rc) {
			printk(KERN_ERR "omapvout_probe %d failed\n", (i + 1));
			return rc;
		}
	}

	return 0;
}

static int omapvout_remove(struct platform_device *pdev)
{
	struct omapvout_device *vout = platform_get_drvdata(pdev);

	DBG("omapvout_remove\n");

	omapvout_dss_remove(vout);
	omapvout_free_resources(vout);

	return 0;
}

static struct platform_driver omapvout_driver = {
	.remove         = omapvout_remove,
	.driver         = {
		.name   = MODULE_NAME,
	},
};

static int __init omapvout_init(void)
{
	int rc;

	DBG("omapvout_init\n");

	rc = platform_driver_probe(&omapvout_driver, omapvout_probe);
	if (rc != 0) {
		printk(KERN_ERR "failed omapvout register/probe %d\n", rc);
		return -ENODEV;
	}

	return 0;
}

static void __exit omapvout_exit(void)
{
	DBG("omapvout_exit\n");
	platform_driver_unregister(&omapvout_driver);
}

device_initcall_sync(omapvout_init);
module_exit(omapvout_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("OMAP2/3 Video Out for V4L2");
MODULE_LICENSE("GPL");

