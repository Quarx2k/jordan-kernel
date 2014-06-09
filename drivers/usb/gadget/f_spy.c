/*
 * f_spy.c -- USB CDC serial (SPY) function driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/usb/composite.h>
#include <linux/tty.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>

#include "s_serial.h"
#include "gadget_chips.h"

#include "f_spy.h"
/*-------------------------------------------------------------------------*/

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "s_serial.c"

/*-------------------------------------------------------------------------*/

/*
 * This CDC SPY function support just wraps control functions and
 * notifications around the generic serial-over-usb code.
 *
 * Because CDC SPY is standardized by the USB-IF, many host operating
 * systems have drivers for it.  Accordingly, SPY is the preferred
 * interop solution for serial-port type connections.  The control
 * models are often not necessary, and in any case don't do much in
 * this bare-bones implementation.
 *
 * Note that even MS-Windows has some support for SPY.  However, that
 * support is somewhat broken because when you use SPY in a composite
 * device, having multiple interfaces confuses the poor OS.  It doesn't
 * seem to understand CDC Union descriptors.  The new "association"
 * descriptors (roughly equivalent to CDC Unions) may sometimes help.
 */

/*
 *The Vendor String driver and the usb client driver have only two interacts.
 *	1. init together.
 *	2. when catch vendor string, usb client driver submit it to verdor
 *         string driver.
 */

/* vendor request command. */
#define USB_CDC_REQ_VENDOR_STRING_CODING 1
/* vendor string length. trust: vender string is less than 256.*/
#define USB_CDC_REQ_VENDOR_STRING_MAXLENGTH 256

struct device_vendor_string_dev {
	int usb_connect_flag;
	wait_queue_head_t device_vendor_string_wq;
	int g_device_type;
	atomic_t device_vendor_string_excl;

	/* lock the vendor string.
	 * only one read & one write, so it is no need to use it until now.
	 */
	spinlock_t	lock;
	u8	vender_string[USB_CDC_REQ_VENDOR_STRING_MAXLENGTH];
};

static struct device_vendor_string_dev *_device_vendor_string_dev;

struct spy_ep_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
};

struct f_spy {
	struct gserial			port;
	u8				ctrl_id, data_id;
	u8				port_num;

	u8				pending;

	struct spy_ep_descs		fs;
	struct spy_ep_descs		hs;
};

static inline struct f_spy *func_to_spy(struct usb_function *f)
{
	return container_of(f, struct f_spy, port.func);
}

static inline struct f_spy *port_to_spy(struct gserial *p)
{
	return container_of(p, struct f_spy, port);
}

/*-------------------------------------------------------------------------*/

/* interface and class descriptors: */

#ifdef CONFIG_USB_MOT_ANDROID
static struct usb_interface_descriptor spy_control_interface_desc = {
#else
static struct usb_interface_descriptor spy_control_interface_desc __initdata = {
#endif
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_COMM,
	.bInterfaceSubClass =	USB_CDC_SUBCLASS_MDLM,
	.bInterfaceProtocol =	0x81,
	/* .iInterface = DYNAMIC */
};

#ifdef CONFIG_USB_MOT_ANDROID
static struct usb_interface_descriptor spy_data_interface_desc = {
#else
static struct usb_interface_descriptor spy_data_interface_desc __initdata = {
#endif
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	/* .iInterface = DYNAMIC */
};

#ifdef CONFIG_USB_MOT_ANDROID
static struct usb_cdc_header_desc spy_header_desc = {
#else
static struct usb_cdc_header_desc spy_header_desc __initdata = {
#endif
	.bLength =		sizeof(spy_header_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,
	.bcdCDC =		cpu_to_le16(0x0110),
};

#ifdef CONFIG_USB_MOT_ANDROID
static struct usb_cdc_mdlm_desc spy_mdlm_desc = {
#else
static struct usb_cdc_mdlm_desc spy_mdlm_desc __initdata = {
#endif
	.bLength =		sizeof(spy_mdlm_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_MDLM_TYPE,
	.bcdVersion =		cpu_to_le16(0x0100),
	.bGUID = {0x39, 0x30, 0x36, 0xD5, 0x1F, 0x95, 0x42, 0xE7,
		  0xA1, 0x4D, 0xF2, 0x78, 0x0B, 0x0D, 0x5B, 0xBA},
};

/* since "usb_cdc_mdlm_detail_desc" is a variable length structure, we
 * can't really use its struct.  All we do here is say that we're using
 * the submode of "SAFE" which directly matches the CDC Subset.
 */
#ifdef CONFIG_USB_MOT_ANDROID
static u8 spy_mdlm_detail_desc[] = {
#else
static u8 spy_mdlm_detail_desc[] __initdata = {
#endif
	6,
	USB_DT_CS_INTERFACE,
	USB_CDC_MDLM_DETAIL_TYPE,
	0x00,
	0x00,
	0x01,
};

#ifdef CONFIG_USB_MOT_ANDROID
static struct usb_cdc_union_desc spy_union_desc = {
#else
static struct usb_cdc_union_desc spy_union_desc __initdata = {
#endif
	.bLength =		sizeof(spy_union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	/* .bMasterInterface0 =	DYNAMIC */
	/* .bSlaveInterface0 =	DYNAMIC */
};

/* full speed support: */
#ifdef CONFIG_USB_MOT_ANDROID
static struct usb_endpoint_descriptor spy_fs_in_desc = {
#else
static struct usb_endpoint_descriptor spy_fs_in_desc __initdata = {
#endif
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

#ifdef CONFIG_USB_MOT_ANDROID
static struct usb_endpoint_descriptor spy_fs_out_desc = {
#else
static struct usb_endpoint_descriptor spy_fs_out_desc __initdata = {
#endif
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

#ifdef CONFIG_USB_MOT_ANDROID
static struct usb_descriptor_header *spy_fs_function[] = {
#else
static struct usb_descriptor_header *spy_fs_function[] __initdata = {
#endif
	(struct usb_descriptor_header *) &spy_control_interface_desc,
	(struct usb_descriptor_header *) &spy_header_desc,
	(struct usb_descriptor_header *) &spy_mdlm_desc,
	(struct usb_descriptor_header *) &spy_mdlm_detail_desc,
	(struct usb_descriptor_header *) &spy_union_desc,
	(struct usb_descriptor_header *) &spy_data_interface_desc,
	(struct usb_descriptor_header *) &spy_fs_in_desc,
	(struct usb_descriptor_header *) &spy_fs_out_desc,
	NULL,
};

/* high speed support: */
#ifdef CONFIG_USB_MOT_ANDROID
static struct usb_endpoint_descriptor spy_hs_in_desc = {
#else
static struct usb_endpoint_descriptor spy_hs_in_desc __initdata = {
#endif
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

#ifdef CONFIG_USB_MOT_ANDROID
static struct usb_endpoint_descriptor spy_hs_out_desc = {
#else
static struct usb_endpoint_descriptor spy_hs_out_desc __initdata = {
#endif
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

#ifdef CONFIG_USB_MOT_ANDROID
static struct usb_descriptor_header *spy_hs_function[] = {
#else
static struct usb_descriptor_header *spy_hs_function[] __initdata = {
#endif
	(struct usb_descriptor_header *) &spy_control_interface_desc,
	(struct usb_descriptor_header *) &spy_header_desc,
	(struct usb_descriptor_header *) &spy_mdlm_desc,
	(struct usb_descriptor_header *) &spy_mdlm_detail_desc,
	(struct usb_descriptor_header *) &spy_union_desc,
	(struct usb_descriptor_header *) &spy_data_interface_desc,
	(struct usb_descriptor_header *) &spy_hs_in_desc,
	(struct usb_descriptor_header *) &spy_hs_out_desc,
	NULL,
};

/* string descriptors: */

#define SPY_CTRL_IDX	0
#define SPY_DATA_IDX	1

/* static strings, in UTF-8 */
static struct usb_string spy_string_defs[] = {
#ifdef CONFIG_USB_MOT_ANDROID
	[SPY_CTRL_IDX].s = "ST-Ericsson TD-HSPA Trace Port",
	[SPY_DATA_IDX].s = "ST-Ericsson TD-HSPA Trace bulk data",
#else
	[SPY_CTRL_IDX].s = "CDC Abstract Control Model (SPY)",
	[SPY_DATA_IDX].s = "CDC SPY Data",
#endif
	{  /* ZEROES END LIST */ },
};

static struct usb_gadget_strings spy_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		spy_string_defs,
};

static struct usb_gadget_strings *spy_strings[] = {
	&spy_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/

/* SPY control ... data handling is delegated to tty library code.
 * The main task of this function is to activate and deactivate
 * that code based on device state; track parameters like line
 * speed, handshake state, and so on; and issue notifications.
 */

static void spy_complete_set_vendor_string(struct usb_ep *ep,
		struct usb_request *req)
{
	struct usb_composite_dev *cdev = ep->driver_data;
	struct device_vendor_string_dev *dev_vendor_string =
	    _device_vendor_string_dev;

	if (dev_vendor_string == NULL) {
		printk(KERN_INFO "%s: dev_vendor_string is NULL.\n", __func__);
		return;
	}

	if (req->status != 0) {
		DBG(cdev, "spy ttyGS port completion, err %d\n", req->status);
		return;
	}

	/* normal completion */
	if (req->actual > USB_CDC_REQ_VENDOR_STRING_MAXLENGTH) {
		DBG(cdev, "spy ttyGS port short resp, len %d\n", req->actual);
		usb_ep_set_halt(ep);
	} else {
		printk(KERN_INFO "%s: %d  vendor string = 0x%x 0x%x 0x%x 0x%x.\n",
			__func__, __LINE__,
			((u8 *)(req->buf))[0], ((u8 *)(req->buf))[1],
			((u8 *)(req->buf))[2], ((u8 *)(req->buf))[3]);

		memcpy(dev_vendor_string->vender_string, req->buf,
				req->actual);

		dev_vendor_string->usb_connect_flag = 1;
		wake_up_interruptible(
				&dev_vendor_string->device_vendor_string_wq);
	}
}

int spy_ctrlrequest(struct usb_composite_dev *cdev,
		const struct usb_ctrlrequest *ctrl)
{
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);


	/* composite driver infrastructure handles everything except
	 * CDC class messages; interface activation uses set_alt().
	 *
	 * Note CDC spec table 4 lists the SPY request profile.  It requires
	 * encapsulated command support ... we don't handle any, and respond
	 * to them by stalling.  Options include get/set/clear comm features
	 * (not that useful) and SEND_BREAK.
	 */
	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {

	/* VENDOR_STRING ... just read and save the vender string. */
	case ((USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE) << 8)
			| USB_CDC_REQ_VENDOR_STRING_CODING:
		if (w_length > USB_CDC_REQ_VENDOR_STRING_MAXLENGTH)
			goto invalid;

		value = w_length;
		cdev->gadget->ep0->driver_data = cdev;
		req->complete = spy_complete_set_vendor_string;

	default:
invalid:
		VDBG(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		DBG(cdev, "spy ttyGS port req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "spy response on ttyGS port, err %d\n",
				 value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

static int spy_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_spy		*spy = func_to_spy(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	/* we know alt == 0, so this is an activation or a reset */
	if (intf == spy->ctrl_id) {
		/* Do nothing */
	} else if (intf == spy->data_id) {
		if (spy->port.in->driver_data) {
			DBG(cdev, "reset spy ttyGS%d\n", spy->port_num);
			gserial_disconnect(&spy->port);
		} else {
			DBG(cdev, "activate spy ttyGS%d\n", spy->port_num);
		}
		spy->port.in_desc = ep_choose(cdev->gadget,
				spy->hs.in, spy->fs.in);
		spy->port.out_desc = ep_choose(cdev->gadget,
				spy->hs.out, spy->fs.out);
		gserial_connect(&spy->port, spy->port_num);
	} else
		return -EINVAL;

	return 0;
}

static void spy_disable(struct usb_function *f)
{
	struct f_spy	*spy = func_to_spy(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	DBG(cdev, "spy ttyGS%d deactivated\n", spy->port_num);
	gserial_disconnect(&spy->port);
}

/*-------------------------------------------------------------------------*/

/* SPY function driver setup/binding */
static int
spy_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_spy		*spy = func_to_spy(f);
	int			status;
	struct usb_ep		*ep;

	/* allocate instance-specific interface IDs, and patch descriptors */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	spy->ctrl_id = status;

	spy_control_interface_desc.bInterfaceNumber = status;
	/*workaround, because auto alloc bInterfaceNumber when setup.*/
	spy_union_desc.bMasterInterface0 = 0; /* status; */

	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	spy->data_id = status;

	spy_data_interface_desc.bInterfaceNumber = status;
	spy_union_desc.bSlaveInterface0 = 1; /* status; */

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &spy_fs_in_desc);
	if (!ep)
		goto fail;
	spy->port.in = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &spy_fs_out_desc);
	if (!ep)
		goto fail;
	spy->port.out = ep;
	ep->driver_data = cdev;	/* claim */

	/* copy descriptors, and track endpoint copies */
#ifdef CONFIG_USB_MOT_ANDROID
	f->descriptors = spy_fs_function;
#else
	f->descriptors = usb_copy_descriptors(spy_fs_function);
	if (!f->descriptors)
		goto fail;
#endif

	spy->fs.in = usb_find_endpoint(spy_fs_function,
			f->descriptors, &spy_fs_in_desc);
	spy->fs.out = usb_find_endpoint(spy_fs_function,
			f->descriptors, &spy_fs_out_desc);

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		spy_hs_in_desc.bEndpointAddress =
				spy_fs_in_desc.bEndpointAddress;
		spy_hs_out_desc.bEndpointAddress =
				spy_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
#ifdef CONFIG_USB_MOT_ANDROID
		f->hs_descriptors = spy_hs_function;
#else
		f->hs_descriptors = usb_copy_descriptors(spy_hs_function);
#endif

		spy->hs.in = usb_find_endpoint(spy_hs_function,
				f->hs_descriptors, &spy_hs_in_desc);
		spy->hs.out = usb_find_endpoint(spy_hs_function,
				f->hs_descriptors, &spy_hs_out_desc);
	}

	DBG(cdev, "spy ttyGS%d: %s speed IN/%s OUT/%s\n",
			spy->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			spy->port.in->name, spy->port.out->name);
	return 0;

fail:

	/* we might as well release our claims on endpoints */
	if (spy->port.out)
		spy->port.out->driver_data = NULL;
	if (spy->port.in)
		spy->port.in->driver_data = NULL;

	ERROR(cdev, "%s/%p: can't bind, err %d\n", f->name, f, status);

	return status;
}

static void
spy_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_spy		*spy = func_to_spy(f);

#ifndef CONFIG_USB_MOT_ANDROID
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);
#endif
	kfree(spy);
}

/* Some controllers can't support CDC SPY ... */
static inline bool can_support_cdc(struct usb_configuration *c)
{
	/* SH3 doesn't support multiple interfaces */
	if (gadget_is_sh(c->cdev->gadget))
		return false;

	/* sa1100 doesn't have a third interrupt endpoint */
	if (gadget_is_sa1100(c->cdev->gadget))
		return false;

	/* everything else is *probably* fine ... */
	return true;
}

/**
 * spy_bind_config - add a CDC SPY function to a configuration
 * @c: the configuration to support the CDC SPY instance
 * @port_num: /dev/ttyGS* port this interface will use
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @gserial_setup() with enough ports to
 * handle all the ones it binds.  Caller is also responsible
 * for calling @gserial_cleanup() before module unload.
 */
int spy_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct f_spy	*spy;
	int		status;

	if (!can_support_cdc(c))
		return -EINVAL;

	printk(KERN_INFO "Gadget Usb: SPY Bind Config\n");

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	/* maybe allocate device-global string IDs, and patch descriptors */
	if (spy_string_defs[SPY_CTRL_IDX].id == 0) {

		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		spy_string_defs[SPY_CTRL_IDX].id = status;

		spy_control_interface_desc.iInterface = status;

		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		spy_string_defs[SPY_DATA_IDX].id = status;

		spy_data_interface_desc.iInterface = status;
	}

	/* allocate and initialize one new instance */
	spy = kzalloc(sizeof *spy, GFP_KERNEL);
	if (!spy)
		return -ENOMEM;

	spy->port_num = port_num;

	spy->port.func.name = "spy";
	spy->port.func.strings = spy_strings;
	/* descriptors are per-instance copies */
	spy->port.func.bind = spy_bind;
	spy->port.func.unbind = spy_unbind;
	spy->port.func.set_alt = spy_set_alt;
	spy->port.func.disable = spy_disable;
	status = usb_add_function(c, &spy->port.func);

	if (status)
		kfree(spy);
	return status;
}

/*-------------------------------------------------------------------------*/
/*
 * Device is used for spy tracer vender string reading
 */

static int device_vendor_string_open(struct inode *ip, struct file *fp)
{
	struct device_vendor_string_dev *dev_vendor_string =
	    _device_vendor_string_dev;

	if (atomic_inc_return(&dev_vendor_string->device_vendor_string_excl) !=
	    1) {
		atomic_dec(&dev_vendor_string->device_vendor_string_excl);
		return -EBUSY;
	}
	return 0;
}

static int device_vendor_string_release(struct inode *ip, struct file *fp)
{
	struct device_vendor_string_dev *dev_vendor_string =
	    _device_vendor_string_dev;

	atomic_dec(&dev_vendor_string->device_vendor_string_excl);
	return 0;
}

static int event_pending(void)
{
	struct device_vendor_string_dev *dev_vendor_string =
	    _device_vendor_string_dev;

	if (dev_vendor_string->usb_connect_flag)
		return 1;
	else
		return 0;
}

static unsigned int device_vendor_string_poll(struct file *file,
					    struct poll_table_struct *wait)
{
	struct device_vendor_string_dev *dev_vendor_string =
	    _device_vendor_string_dev;

	poll_wait(file, &dev_vendor_string->device_vendor_string_wq, wait);

	if (event_pending())
		return POLLIN | POLLRDNORM;
	else
		return 0;
}

static ssize_t device_vendor_string_read(struct file *file, char *buf,
				       size_t count, loff_t *ppos)
{
	struct device_vendor_string_dev *dev_vendor_string =
	    _device_vendor_string_dev;

	if (wait_event_interruptible
	    (dev_vendor_string->device_vendor_string_wq,
	    (file->f_flags & O_NONBLOCK || event_pending()))) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		return -ERESTARTSYS;
	}

	if (!event_pending())
		return 0;

	if (count > USB_CDC_REQ_VENDOR_STRING_MAXLENGTH)
		return 0;

	if (copy_to_user(buf, dev_vendor_string->vender_string,
			count))
		return -EFAULT;

	return count;
}

static const struct file_operations device_vendor_string_fops = {
	.owner = THIS_MODULE,
	.open = device_vendor_string_open,
	.poll = device_vendor_string_poll,
	.read = device_vendor_string_read,
	.release = device_vendor_string_release,
};

static struct miscdevice vendor_string_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "spy",
	.fops = &device_vendor_string_fops,
};

/*-------------------------------------------------------------------------*/

int spy_init(struct usb_composite_dev *cdev)
{
	struct device_vendor_string_dev *dev_vendor_string;
	int ret;

	printk(KERN_INFO "f_spy init\n");

	/* allocate device_vendor_string dev and wait queue */
	dev_vendor_string = kzalloc(sizeof(*dev_vendor_string), GFP_KERNEL);
	if (!dev_vendor_string) {
		printk(KERN_ERR "f_spy kzalloc err!\n");
		return -ENOMEM;/*failed, donot roll back.*/
	}
	_device_vendor_string_dev = dev_vendor_string;
	init_waitqueue_head(&dev_vendor_string->device_vendor_string_wq);
	spin_lock_init(&dev_vendor_string->lock);

	ret = misc_register(&vendor_string_device);
	if (ret) {
		kfree(_device_vendor_string_dev);
		printk(KERN_ERR "f_spy vendor_string_device register err!\n");
		return ret;/*failed, but other driver can work ok.*/
	}

	return gserial_setup(cdev->gadget, 1);
}

void spy_cleanup(void)
{
	printk(KERN_INFO "f_spy cleanup\n");
	gserial_cleanup();
	misc_deregister(&vendor_string_device);

	kfree(_device_vendor_string_dev);
	_device_vendor_string_dev = NULL;
}
