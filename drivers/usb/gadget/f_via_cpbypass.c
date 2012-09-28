/*
 * f_via_cpbypass.c -- USB ets gadget driver
 *
 * Copyright (C) 2009 Karfield Chen <kfchen@via-telecom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* #define VERBOSE_DEBUG */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/android_composite.h>

#include <linux/kthread.h>
#include <linux/usb/cdc.h>

#ifdef CONFIG_USB_MOT_ANDROID
#include "f_mot_android.h"
#endif

#include <linux/bypass.h>
#define BULK_BUFFER_SIZE           1024

/* number of rx and tx requests to allocate */
#define RX_REQ_MAX 8
#define TX_REQ_MAX 8

#define WRITE_BUF_SIZE		8192	/* TX only */

struct bypass_buf {
	unsigned buf_size;
	char *buf_buf;
	char *buf_get;
	char *buf_put;
};
struct cpbypass_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	int online;
	unsigned index;		/* for distinguish different ports */
	struct bypass_buf bp_buf;	/* write loop buffer */

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;

	struct list_head tx_idle;
	struct list_head rx_idle;
	struct list_head rx_done;

	/* the request we're currently reading from */
	struct usb_request *read_req;
	unsigned char *read_buf;
	unsigned read_count;
};

/* string descriptors: */

static struct usb_string cpbypass_string_defs[] = {
	[0].s = "INTF0",
	[1].s = "INTF1",
	{}			/* end of list */
};

static struct usb_gadget_strings cpbypass_string_table = {
	.language = 0x0409,	/* en-us */
	.strings = cpbypass_string_defs,
};

static struct usb_gadget_strings *cpbypass_strings[] = {
	&cpbypass_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*/

/* interface 1 descriptor:for via cbp ets intf */

static struct usb_interface_descriptor ets_interface_desc = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = 0xff,	/* USB_CLASS_VENDOR_SPEC */
	.bInterfaceSubClass = 0xff,
	.bInterfaceProtocol = 0xff,
	/*.iInterface = 0x01,*/
};

/* full speed support: */

static struct usb_endpoint_descriptor ets_fs_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	/* .wMaxPacketSize =       __constant_cpu_to_le16(64), */
	/* .bInterval      =       0, */
};

static struct usb_endpoint_descriptor ets_fs_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	/* .wMaxPacketSize =       __constant_cpu_to_le16(64), */
	/* .bInterval      =       0, */
};

static struct usb_descriptor_header *ets_fs_function[] = {
	(struct usb_descriptor_header *)&ets_interface_desc,
	(struct usb_descriptor_header *)&ets_fs_in_desc,
	(struct usb_descriptor_header *)&ets_fs_out_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor ets_hs_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor ets_hs_out_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	/* .bEndpointAddress       = USB_DIR_OUT, */
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = __constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *ets_hs_function[] = {
	(struct usb_descriptor_header *)&ets_interface_desc,
	(struct usb_descriptor_header *)&ets_hs_in_desc,
	(struct usb_descriptor_header *)&ets_hs_out_desc,
	NULL,
};

/*-------------------------------------------------------------------------*/

/* end gps intf */

static struct cpbypass_dev *_ets_dev;
struct tasklet_struct rx_tasklet;
/* struct work_struct rx_work;
   static struct workqueue_struct *rx_queue; */
struct tasklet_struct tx_tasklet;
static int select;
static int select1;

static inline struct cpbypass_dev *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct cpbypass_dev, function);
}

static struct usb_request *cpbypass_request_new(struct usb_ep *ep,
						int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void cpbypass_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

/* add a request to the tail of a list */
static void req_put(struct cpbypass_dev *dev, struct list_head *head,
		    struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *req_get(struct cpbypass_dev *dev,
				   struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static int bp_buf_alloc(struct bypass_buf *gb, unsigned size)
{
	gb->buf_buf = kmalloc(size, GFP_KERNEL);
	if (gb->buf_buf == NULL)
		return -ENOMEM;

	gb->buf_size = size;
	gb->buf_put = gb->buf_buf;
	gb->buf_get = gb->buf_buf;

	return 0;
}

static void bp_buf_free(struct bypass_buf *gb)
{
	kfree(gb->buf_buf);
	gb->buf_buf = NULL;
}

static unsigned bp_buf_data_avail(struct bypass_buf *gb)
{
	return (gb->buf_size + gb->buf_put - gb->buf_get) % gb->buf_size;
}

static unsigned bp_buf_space_avail(struct bypass_buf *gb)
{
	return (gb->buf_size + gb->buf_get - gb->buf_put - 1) % gb->buf_size;
}

static unsigned
bp_buf_put(struct bypass_buf *gb, const char *buf, unsigned count)
{
	unsigned len;

	len = bp_buf_space_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_put;
	if (count > len) {
		memcpy(gb->buf_put, buf, len);
		memcpy(gb->buf_buf, buf + len, count - len);
		gb->buf_put = gb->buf_buf + count - len;
	} else {
		memcpy(gb->buf_put, buf, count);
		if (count < len)
			gb->buf_put += count;
		else		/* count == len */
			gb->buf_put = gb->buf_buf;
	}

	return count;
}

static unsigned bp_buf_get(struct bypass_buf *gb, char *buf, unsigned count)
{
	unsigned len;

	len = bp_buf_data_avail(gb);
	if (count > len)
		count = len;

	if (count == 0)
		return 0;

	len = gb->buf_buf + gb->buf_size - gb->buf_get;
	if (count > len) {
		memcpy(buf, gb->buf_get, len);
		memcpy(buf + len, gb->buf_buf, count - len);
		gb->buf_get = gb->buf_buf + count - len;
	} else {
		memcpy(buf, gb->buf_get, count);
		if (count < len)
			gb->buf_get += count;
		else		/* count == len */
			gb->buf_get = gb->buf_buf;
	}

	return count;
}

static unsigned
bp_send_packet(struct cpbypass_dev *dev, char *packet, unsigned size)
{
	unsigned len;

	len = bp_buf_data_avail(&dev->bp_buf);
	if (len < size)
		size = len;
	if (size != 0)
		size = bp_buf_get(&dev->bp_buf, packet, size);
	return size;
}

static void write_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct cpbypass_dev *dev = ep->driver_data;


	req_put(dev, &dev->tx_idle, req);
	select1 = dev->index;
	tasklet_schedule(&tx_tasklet);

}

static void read_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct cpbypass_dev *dev = ep->driver_data;

	if (req->status == -ESHUTDOWN) {
		req_put(dev, &dev->rx_idle, req);
		return;
	}
	req_put(dev, &dev->rx_done, req);
	select = dev->index;
	tasklet_schedule(&rx_tasklet);
}

static int __init create_bulk_endpoints(struct cpbypass_dev *dev,
					struct usb_endpoint_descriptor *in_desc,
					struct usb_endpoint_descriptor
					*out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG(cdev, "create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	dev->ep_in = ep;
	ep->driver_data = dev;	/* important */

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ets ep_out got %s\n", ep->name);
	dev->ep_out = ep;
	ep->driver_data = dev;	/* this must add for distinguish endpoints */

	/* now allocate requests for our endpoints */
	for (i = 0; i < RX_REQ_MAX; i++) {
		req = cpbypass_request_new(dev->ep_out, BULK_BUFFER_SIZE);
		if (!req)
			goto fail;

		req->complete = read_complete;
		req_put(dev, &dev->rx_idle, req);
	}

	for (i = 0; i < TX_REQ_MAX; i++) {
		req = cpbypass_request_new(dev->ep_in, BULK_BUFFER_SIZE);
		if (!req)
			goto fail;
		req->complete = write_complete;
		req_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	printk(KERN_ERR "ets_function_bind() could not allocate requests\n");
	return -1;
}

static void bp_start_tx(unsigned long sl)
{
	int *sel = (int *)sl;
	struct usb_request *req = 0;
	struct cpbypass_dev *dev = NULL;
	int ret;

	if (*sel == 1)
		dev = _ets_dev;
	else {
		printk(KERN_ERR "select1 number is error\n");
		return;
	}


	while ((req = req_get(dev, &dev->tx_idle))) {
		int len;

		spin_lock(&dev->lock);
		len = bp_send_packet(dev, req->buf, dev->ep_in->maxpacket);
		spin_unlock(&dev->lock);
		if (len == 0)
			break;
		req->length = len;
		ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
		if (ret < 0) {
			printk(KERN_INFO "bp_start_tx, error %d\n", ret);
			break;
		}
	}
	if (req)
		req_put(dev, &dev->tx_idle, req);
}

static int bp_write(int port_num, const unsigned char *buf, int count)
{
	struct cpbypass_dev *dev = _ets_dev;
	unsigned long flags;

	select1 = 1;		/* select which port will write */

	if (!dev || !dev->online)
		return 0;

	spin_lock_irqsave(&dev->lock, flags);
	if (count)
		count = bp_buf_put(&dev->bp_buf, buf, count);
	spin_unlock_irqrestore(&dev->lock, flags);
	tasklet_schedule(&tx_tasklet);
	return count;
}

static unsigned cp_start_rx(struct cpbypass_dev *dev)
{
	struct usb_request *req;
	int ret;
	int start = 0;

	while ((req = req_get(dev, &dev->rx_idle))) {

		req->length = BULK_BUFFER_SIZE;
		ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);

		if (ret < 0) {
			req_put(dev, &dev->rx_idle, req);
			printk(KERN_ERR "cp_start usb_ep_queue error\n");
			break;
		}
		start++;
	}
	return start;
}

static void bypass_connect(int port_num)
{

	printk(KERN_INFO "bypass_connect:port_num:%d\n", port_num);
	switch (port_num) {
	case 1:
		if (_ets_dev->online)
			cp_start_rx(_ets_dev);
		break;
	default:
		printk(KERN_ERR "error port_num\n");
	}
}

static struct bypass_ops gb_ops = {
	.g_write = bp_write,
	.bp_connect = bypass_connect,
};

static void cp_rx_push(struct cpbypass_dev *dev)
{
	struct usb_request *req = NULL;
	int i;
	int port_num = dev->index - 1;
	struct bypass *bypass = bypass_get();
	if (bypass == NULL) {
		printk(KERN_ERR"%s - bypass_get error\n", __func__);
		return;
	}

	while ((req = req_get(dev, &dev->rx_done))) {

		unsigned char *packet = req->buf;
		unsigned size = req->actual;
		unsigned size1 = size / 64;
		unsigned size2 = size % 64;

		switch (req->status) {
		case -ESHUTDOWN:
			printk(KERN_ERR "cpbypass: shutdown\n");
			break;

		default:
			printk(KERN_INFO "cpbypass: unexpected RX status %d\n",
			       req->status);
		case 0:
			break;
		}
		if (req->actual == 0)
			goto add_to_idle;

		if (bypass->ops->h_write) {
			int wrote;
			int bypass_port_num = port_num + 1;
			for (i = 0; i < size1; i++) {
				do {
					wrote =
					    bypass->ops->
					    h_write(bypass_port_num,
						    packet + i * 64, 64);
					if (wrote < 0)
						break;
				} while (wrote != 64);
			}

			if (size2 > 0) {
				do {
					wrote =
					    bypass->ops->
					    h_write(bypass_port_num,
						    packet + i * 64, size2);
					if (wrote < 0)
						break;
				} while (wrote != size2);
			}

			if (port_num == 0) {
				if (*(packet + 6) == 0xdc) {
					if (*(packet + 7) == 0)
						bypass->ets_jump_flag = 1;
				}
			}
		} else
			printk(KERN_ERR "bypass->ops->h_write has not registered\n");
add_to_idle:
		req_put(dev, &dev->rx_idle, req);
		/* write to usb host///add read it self for cyle read */
	}
}

static void select_rx_push(unsigned long select1)
{
	int *sel = (int *)select1;
	struct cpbypass_dev *dev = _ets_dev;
	struct bypass *bypass = bypass_get();
	int status = 0;

	if (bypass == NULL) {
		printk(KERN_ERR "%s - bypass_get error\n", __func__);
		return;
	}
	if (*sel == 1) {
		status = bypass->ets_status;
	} else {
		printk(KERN_ERR "select number is error\n");
	}

	if (!list_empty(&dev->rx_done)) {
		cp_rx_push(dev);
		if (status)
			cp_start_rx(dev);
	}
}

static int __init
ets_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct cpbypass_dev *dev = _ets_dev;
	int id;
	int ret;

	dev->cdev = cdev;
	DBG(cdev, "ets_function_bind dev: %p\n", dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	ets_interface_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = create_bulk_endpoints(dev, &ets_fs_in_desc, &ets_fs_out_desc);
	printk(KERN_INFO "kevin:%s():1 ret=%d\n", __func__, ret);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		ets_hs_in_desc.bEndpointAddress =
		    ets_fs_in_desc.bEndpointAddress;
		ets_hs_out_desc.bEndpointAddress =
		    ets_fs_out_desc.bEndpointAddress;
	}
	if (dev->bp_buf.buf_buf == NULL)
		bp_buf_alloc(&dev->bp_buf, WRITE_BUF_SIZE);

	DBG(cdev, "%s speed %s: IN/%s, OUT/%s\n",
	    gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
	    f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}

static void
ets_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct cpbypass_dev *dev = func_to_dev(f);
	struct usb_request *req;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	printk(KERN_INFO "enter ets_function_unbind\n");
	while ((req = req_get(dev, &dev->rx_idle)))
		cpbypass_request_free(req, dev->ep_out);
	while ((req = req_get(dev, &dev->tx_idle)))
		cpbypass_request_free(req, dev->ep_in);
	while ((req = req_get(dev, &dev->rx_done)))
		cpbypass_request_free(req, dev->ep_out);

	spin_unlock_irqrestore(&dev->lock, flags);

	bypass_unregister(2);
	bp_buf_free(&dev->bp_buf);

	kfree(_ets_dev);
	_ets_dev = NULL;
}

static int ets_function_set_alt(struct usb_function *f,
				unsigned intf, unsigned alt)
{
	struct cpbypass_dev *dev = func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct bypass *bypass = bypass_get();
	int ret;

	printk(KERN_INFO "kevin:%s():1\n", __func__);
	if (bypass == NULL) {
		printk(KERN_ERR"%s - bypass_get error\n", __func__);
		return -1;
	}
	printk(KERN_INFO "kevin:%s():2\n", __func__);
	DBG(cdev, "ets_function_set_alt intf: %d alt: %d\n", intf, alt);
	dev->ep_in->driver_data = dev;
	dev->ep_out->driver_data = dev;
	printk(KERN_INFO "kevin:%s():3\n", __func__);
	ret = usb_ep_enable(dev->ep_in,
			    ep_choose(cdev->gadget,
				      &ets_hs_in_desc, &ets_fs_in_desc));
	printk(KERN_INFO "kevin:%s():4 ret=%d\n", __func__, ret);
	if (ret)
		return ret;
	printk(KERN_INFO "kevin:%s():5\n", __func__);
	ret = usb_ep_enable(dev->ep_out,
			    ep_choose(cdev->gadget,
				      &ets_hs_out_desc, &ets_fs_out_desc));
	if (ret) {
		usb_ep_disable(dev->ep_in);
		return ret;
	}
	dev->online = 1;
	if (bypass->ets_status)
		cp_start_rx(dev);

#ifdef CONFIG_USB_MOT_ANDROID
	usb_interface_enum_cb(ETS_TYPE_FLAG);
#endif
	return 0;
}

static void ets_function_disable(struct usb_function *f)
{
	struct cpbypass_dev *dev = func_to_dev(f);
	struct usb_composite_dev *cdev = dev->cdev;

	DBG(cdev, "ets_function_disable\n");
	dev->online = 0;
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);
	dev->ep_in->driver_data = NULL;
	dev->ep_out->driver_data = NULL;

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

int ets_bind_config(struct usb_configuration *c)
{
	struct cpbypass_dev *dev;
	int ret;

	/* data interface label */
	ret = usb_string_id(c->cdev);
	if (ret < 0)
		return ret;
	cpbypass_string_defs[1].id = ret;
	ets_interface_desc.iInterface = ret;
	dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	spin_lock_init(&dev->lock);

	atomic_set(&dev->open_excl, 0);
	atomic_set(&dev->read_excl, 0);
	atomic_set(&dev->write_excl, 0);

	INIT_LIST_HEAD(&dev->rx_idle);
	INIT_LIST_HEAD(&dev->rx_done);
	INIT_LIST_HEAD(&dev->tx_idle);

	dev->index = 1;

	dev->cdev = c->cdev;
	dev->function.name = "ets";
	dev->function.strings = cpbypass_strings;
	dev->function.descriptors = ets_fs_function;
	dev->function.hs_descriptors = ets_hs_function;
	dev->function.bind = ets_function_bind;
	dev->function.unbind = ets_function_unbind;
	dev->function.set_alt = ets_function_set_alt;
	dev->function.disable = ets_function_disable;

	/* _ets_dev must be set before calling usb_gadget_register_driver */
	_ets_dev = dev;

	ret = usb_add_function(c, &dev->function);

	tasklet_init(&rx_tasklet, select_rx_push, (unsigned long)&select);

	tasklet_init(&tx_tasklet, bp_start_tx, (unsigned long)&select1);
	bypass_register(&gb_ops);
	if (ret)
		goto err2;

	return 0;
err2:
	kfree(dev);
	printk(KERN_ERR "ets gadget driver failed to initialize\n");
	return ret;
}

#if defined(CONFIG_USB_MOT_ANDROID)
static struct android_usb_function ets_function = {
	.name = "ets",
	.bind_config = ets_bind_config,
};

static int __init init(void)
{
	printk(KERN_INFO "f_via_cpbypass init\n");
	android_register_function(&ets_function);
	return 0;
}

module_init(init);

#endif /* CONFIG_USB_MOT_ANDROID */
