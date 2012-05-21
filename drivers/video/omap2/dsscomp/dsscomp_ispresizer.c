#include "dsscomp_ispresizer.h"

static u32 isprsz;
struct isp_node pipe;
wait_queue_head_t wait;
u32 out_buf_phy_addr[MAX_VIDEO_BUFFERS];
static int index;
extern int isp_reset;
struct kobject *isprsz_kobj;
static int isprsz_enable_val = 1;

void dsscomp_isp_rsz_dma_tx_callback(void *arg)
{
	wake_up_interruptible(&wait);
}

int is_isprsz_enabled(void)
{
	return isprsz_enable_val;
}

int ispresizer_init(struct dss2_ovl_info *oi)
{
	struct dss2_ovl_cfg *cfg = &oi->cfg;
	struct v4l2_pix_format pix;
	int num_video_buffers = 0;
	int ret = 0;

	if (isp_reset) {
		isprsz = 0;
		isp_reset = 0;
	}

	if (isprsz)
		return 0;

	memset(out_buf_phy_addr, 0, sizeof(out_buf_phy_addr));
	index = 0;

	/* get the ISP resizer resource and configure it*/
	ispdss_put_resource();
	ret = ispdss_get_resource();
	if (ret) {
		pr_info("<%s>: <%s> failed to get ISP "
			"resizer resource = %d\n",
			__FILE__, __func__, ret);
		return ret;
	}

	pix.width = cfg->width;
	pix.height = cfg->height;
	pix.pixelformat = V4L2_PIX_FMT_UYVY;
	pix.field = V4L2_FIELD_NONE;
	pix.bytesperline = pix.width * 2;
	pix.sizeimage = pix.bytesperline * pix.height;
	pix.bytesperline =
		(pix.bytesperline + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);
	pix.priv = 0;
	pix.colorspace = V4L2_COLORSPACE_JPEG;

	/* clear data */
	memset(&pipe, 0, sizeof(pipe));
	pipe.in.path = RSZ_MEM_YUV;
	/* setup source parameters */
	pipe.in.image = pix;
	pipe.in.crop.left = 0;
	pipe.in.crop.top = 0;
	pipe.in.crop.width = cfg->crop.w;
	pipe.in.crop.height = cfg->crop.h;
	/* setup destination parameters */
	pipe.out.image.width = cfg->win.w;
	pipe.out.image.height = cfg->win.h;

	num_video_buffers = MAX_VIDEO_BUFFERS;

	ret = ispdss_configure(&pipe, dsscomp_isp_rsz_dma_tx_callback,
		num_video_buffers, (void *)oi);
	if (ret) {
		pr_info("<%s> failed to configure "
			"ISP_resizer = %d\n",
			__func__, ret);
		ispdss_put_resource();
		return ret;
	}

	isprsz = 1;
	return ret;
}


int ispresizer_begin(struct dss2_ovl_info *oi)
{
	struct dss2_ovl_cfg *cfg = &oi->cfg;
	int ret = 0;

	for (ret = 0; ret < MAX_VIDEO_BUFFERS ; ret++) {
		if (out_buf_phy_addr[ret] == 0) {
			out_buf_phy_addr[ret] = oi->ba;
			index = ret;
			break;
		} else if (out_buf_phy_addr[ret] == oi->ba) {
			index = ret;
			break;
		}
	}

	/*Start resizing*/
	ret = ispdss_begin(&pipe, index, index,
			pipe.out.image.width * 2,
			oi->ba,
			oi->ba,
			cfg->width * cfg->height * 2);

	return ret;
}


struct isprsz_attribute {
	struct attribute attr;
	ssize_t (*show)(char *);
	ssize_t (*store)(const char *, size_t);
};

static ssize_t isprsz_enable_show(char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", isprsz_enable_val);
}

static ssize_t isprsz_enable_store(const char *buf, size_t size)
{
	if (sscanf(buf, "%d", &isprsz_enable_val) != 1)
		return -EINVAL;

	if (isprsz_enable_val == 0) {
		ispdss_put_resource();
		isp_reset = 1;
	}

	return size;
}


#define ISPRSZ_ATTR(_name, _mode, _show, _store) \
	struct isprsz_attribute isprsz_attr_##_name = \
	__ATTR(_name, _mode, _show, _store)

static ISPRSZ_ATTR(enable, S_IRUGO|S_IWUSR,
		isprsz_enable_show, isprsz_enable_store);

static struct attribute *isprsz_sysfs_attrs[] = {
	&isprsz_attr_enable.attr,
	NULL
};


static ssize_t isprsz_attr_show(struct kobject *kobj, struct attribute *attr,
	char *buf)
{
	struct isprsz_attribute *isprsz_attr;

	isprsz_attr = container_of(attr, struct isprsz_attribute, attr);

	if (!isprsz_attr->show)
		return -ENOENT;

	return isprsz_attr->show(buf);
}

static ssize_t isprsz_attr_store(struct kobject *kobj, struct attribute *attr,
	const char *buf, size_t size)
{
	struct isprsz_attribute *isprsz_attr;

	isprsz_attr = container_of(attr, struct isprsz_attribute, attr);

	if (!isprsz_attr->store)
		return -ENOENT;

	return isprsz_attr->store(buf, size);
}

static const struct sysfs_ops isprsz_sysfs_ops = {
	.show = isprsz_attr_show,
	.store = isprsz_attr_store,
};

static struct kobj_type isprsz_ktype = {
	.sysfs_ops = &isprsz_sysfs_ops,
	.default_attrs = isprsz_sysfs_attrs,
};


int create_isprsz_sysfs(struct platform_device *pdev)
{
	int ret = 1;

	isprsz_kobj = kzalloc(sizeof(*isprsz_kobj), GFP_KERNEL);

	if (isprsz_kobj) {
		ret = kobject_init_and_add(isprsz_kobj, &isprsz_ktype,
			&pdev->dev.kobj, "isprsz");
	}

	return ret;
}
