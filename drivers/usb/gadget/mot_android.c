/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/android_composite.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/io.h>

#include "f_mot_android.h"
#include "u_serial.h"
#include "f_adb.h"

#include "gadget_chips.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("Motorola Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by platform data */
#define VENDOR_ID               0x22b8
#define PRODUCT_ID              0x41da
#define ADB_PRODUCT_ID          0x41da

#define MAX_DEVICE_TYPE_NUM   25
#define MAX_DEVICE_NAME_SIZE  20


struct device_pid_vid {
	char *name;
	u32 type;
	int vid;
	int pid;
	char *config_name;
	int class;
	int subclass;
	int protocol;
};

static struct device_pid_vid mot_android_vid_pid[MAX_DEVICE_TYPE_NUM] = {
	{"msc", MSC_TYPE_FLAG | CDROM_TYPE_FLAG, 0x22b8, 0x41d9,
	 "Motorola Config 14", USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE},
	{"msc_only", MSC_TYPE_FLAG, 0x22b8, 0x41d9,
	 "Motorola Config 14", USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE},
	{"cdrom", CDROM_TYPE_FLAG, 0x22b8, 0x41de, "Motorola CDROM Device",
	 USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE},
	{"msc_adb", MSC_TYPE_FLAG | CDROM_TYPE_FLAG | ADB_TYPE_FLAG, 0x22b8,
	 0x41db, "Motorola Config 42", USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE},
	{"eth", ETH_TYPE_FLAG, 0x22b8, 0x41d4, "Motorola Config 13",
	 USB_CLASS_COMM, USB_CLASS_COMM, USB_CLASS_PER_INTERFACE},
	{"mtp", MTP_TYPE_FLAG, 0x22b8, 0x41D6, "Motorola Config 15",
	 USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE},
	{"acm", ACM_TYPE_FLAG, 0x22b8, 0x6422, "Motorola Config 1",
	 USB_CLASS_COMM, USB_CLASS_COMM, USB_CLASS_PER_INTERFACE},
	{"eth_adb", ETH_TYPE_FLAG | ADB_TYPE_FLAG, 0x22b8, 0x41ea,
	"Motorola ADB BLAN Config", USB_CLASS_PER_INTERFACE,
	USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE},
	{"acm_eth_mtp", ACM_TYPE_FLAG | ETH_TYPE_FLAG | MTP_TYPE_FLAG,
	 0x22b8,
	 0x41d8, "Motorola Config 30", USB_CLASS_VENDOR_SPEC,
	 USB_CLASS_VENDOR_SPEC, USB_CLASS_VENDOR_SPEC},
	{"mtp_adb", MTP_TYPE_FLAG | ADB_TYPE_FLAG, 0x22b8, 0x41dc,
	 "Motorola Config 32", USB_CLASS_VENDOR_SPEC,
	 USB_CLASS_VENDOR_SPEC, USB_CLASS_VENDOR_SPEC},
	{"acm_eth_mtp_adb",
	 ACM_TYPE_FLAG | ETH_TYPE_FLAG | MTP_TYPE_FLAG | ADB_TYPE_FLAG,
	 0x22b8,
	 0x41da, "Motorola Config 31", USB_CLASS_VENDOR_SPEC,
	 USB_CLASS_VENDOR_SPEC, USB_CLASS_VENDOR_SPEC},
	{"acm_eth_adb", ACM_TYPE_FLAG | ETH_TYPE_FLAG | ADB_TYPE_FLAG,
	 0x22b8,
	 0x41e2, "Motorola Android Composite Device"},
	{"msc_eth", MSC_TYPE_FLAG | ETH_TYPE_FLAG, 0x22b8, 0x41d4,
	 "Motorola Android Composite Device"},
	{"msc_adb_eth", MSC_TYPE_FLAG | ADB_TYPE_FLAG | ETH_TYPE_FLAG,
	 0x22b8,
	 0x41d4, "Motorola Android Composite Device"},
	 {"charge_only", MSC_TYPE_FLAG, 0x22b8, 0x4287, "Motorola Config 49",
	 USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE},
	{"charge_adb", MSC_TYPE_FLAG | ADB_TYPE_FLAG, 0x22b8, 0x428c,
	 "Motorola Config 50", USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE},
	{"rndis", RNDIS_TYPE_FLAG, 0x22b8, 0x41E4, "Motorola RNDIS Device",
	 USB_CLASS_WIRELESS_CONTROLLER, USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE},
	{"rndis_adb", RNDIS_TYPE_FLAG | ADB_TYPE_FLAG, 0x22b8, 0x41E5,
	 "Motorola RNDIS ADB Device",
	 USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE},
	{"ets", ETS_TYPE_FLAG, 0x15eb, 0x0101, "CONF1",
	 USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE},
	{"ets_adb", ETS_TYPE_FLAG | ADB_TYPE_FLAG, 0x22b8, 0x8787, "CONF2",
	 USB_CLASS_MISC, 0x02, 0x01},
	{},
};


struct device_mode_change_dev {
	int adb_mode_changed_flag;
	int tethering_mode_changed_flag;
	int pc_mode_switch_flag;
	int usb_device_cfg_flag;
	int usb_get_desc_flag;
	int usb_data_transfer_flag;
	wait_queue_head_t device_mode_change_wq;
	wait_queue_head_t adb_cb_wq;
	wait_queue_head_t tethering_cb_wq;
	int g_device_type;
	atomic_t device_mode_change_excl;
};

struct android_dev {
	struct usb_composite_dev *cdev;
	struct usb_configuration *config;
	int num_products;
	struct android_usb_product *products;
	int num_functions;
	char **functions;

	int product_id;
	int version;
	struct switch_dev sdev;
};

static struct android_dev *_android_dev;
static struct device_mode_change_dev *_device_mode_change_dev;
atomic_t tethering_enable_excl;

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2
#define STRING_CONFIG_IDX               3

/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "Android",
	[STRING_PRODUCT_IDX].s = "Android",
	[STRING_SERIAL_IDX].s = "0123456789ABCDEF",
	[STRING_CONFIG_IDX].s = "Motorola Android Composite Device",
	{}			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language = 0x0409,	/* en-us */
	.strings = strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength = sizeof(device_desc),
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = __constant_cpu_to_le16(0x0200),
	.bDeviceClass = USB_CLASS_VENDOR_SPEC,
	.bDeviceSubClass = USB_CLASS_VENDOR_SPEC,
	.bDeviceProtocol = USB_CLASS_VENDOR_SPEC,
	.idVendor = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations = 1,
};

static struct list_head _functions = LIST_HEAD_INIT(_functions);
static int _registered_function_count;

void android_usb_set_connected(int connected)
{
	if (_android_dev && _android_dev->cdev
	    && _android_dev->cdev->gadget) {
		if (connected) {
			usb_gadget_disconnect(_android_dev->cdev->gadget);
			switch_set_state(&(_android_dev->sdev), 1);
		} else {
			switch_set_state(&(_android_dev->sdev), 0);
		}
	}
}

void android_usb_set_pid(char *fname, u16 usb_pid)
{
	int j;
	for (j = 0; j < MAX_DEVICE_TYPE_NUM; j++) {
		if (mot_android_vid_pid[j].name == NULL)
			break;

		if (strncmp(mot_android_vid_pid[j].name, fname,
			MAX_DEVICE_NAME_SIZE) == 0) {
			mot_android_vid_pid[j].pid = usb_pid;
			break;
		}
	}

	if (j == MAX_DEVICE_TYPE_NUM)
		printk(KERN_ERR
			"Unable to find a match for the DT entry %s \n",
			fname);
	else
		printk(KERN_INFO
			"USB PID overwritten for  %s with 0x%4x \n",
			mot_android_vid_pid[j].name,
			mot_android_vid_pid[j].pid);
}

static struct android_usb_function *get_function(const char *name)
{
	struct android_usb_function *f;
	list_for_each_entry(f, &_functions, list) {
		if (!strcmp(name, f->name))
			return f;
	}
	return 0;
}

static void bind_functions(struct android_dev *dev)
{
	struct android_usb_function *f;
	char **functions = dev->functions;
	int i;

	for (i = 0; i < dev->num_functions; i++) {
		char *name = *functions++;
		f = get_function(name);
		if (f)
			f->bind_config(dev->config);
		else
			printk(KERN_ERR
			       "function %s not found in bind_functions\n",
			       name);
	}
}

static int __init android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	printk(KERN_DEBUG "android_bind_config\n");
	dev->config = c;

	/* bind our functions if they have all registered */
	if (_registered_function_count == dev->num_functions)
		bind_functions(dev);

	return 0;
}

static int android_setup_config(struct usb_configuration *c,
				const struct usb_ctrlrequest *ctrl);

static struct usb_configuration android_config_driver = {
	.label = "android",
	.bind = android_bind_config,
	.setup = android_setup_config,
	.bConfigurationValue = 1,
	.bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower = 0xFA,	/* 500ma */
};


int get_func_thru_config(int mode)
{
	int i;
	char name[50];

	memset(name, 0, 50);
	sprintf(name, "Motorola Config %d", mode);
	for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
		if (!mot_android_vid_pid[i].config_name)
			break;
		if (!strcmp(mot_android_vid_pid[i].config_name, name))
			return i;
	}
	return -1;
}

void mode_switch_cb(int mode)
{
	struct device_mode_change_dev *dev_mode_change =
	    _device_mode_change_dev;

	dev_mode_change->pc_mode_switch_flag = mode;
	wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
}

static int android_generic_setup(struct usb_configuration *c,
				 const struct usb_ctrlrequest *ctrl)
{
	int value = -EOPNOTSUPP;
	u16 wIndex = le16_to_cpu(ctrl->wIndex);
	u16 wValue = le16_to_cpu(ctrl->wValue);
	u16 wLength = le16_to_cpu(ctrl->wLength);
	struct usb_composite_dev *cdev = c->cdev;
	struct usb_request *req = cdev->req;

	switch (ctrl->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_VENDOR:
		switch (ctrl->bRequest) {
		case 1:
			if ((wValue == 0) && (wLength == 0)) {
				mode_switch_cb(wIndex);
				value = 0;
				req->zero = 0;
				req->length = value;
				if (usb_ep_queue
				    (cdev->gadget->ep0, req, GFP_ATOMIC))
					printk(KERN_ERR
					       "ep0 in queue failed\n");
			}
			break;
		default:
			break;
		}
	default:
		break;
	}
	return value;
}

static int android_setup_config(struct usb_configuration *c,
				const struct usb_ctrlrequest *ctrl)
{
	int i;
	int ret = -EOPNOTSUPP;

	ret = android_generic_setup(c, ctrl);
	if (ret >= 0)
		return ret;

	for (i = 0; i < android_config_driver.next_interface_id; i++) {
		if (android_config_driver.interface[i]->setup &&
			!(android_config_driver.interface[i]->hidden)) {
			ret =
			    android_config_driver.
			    interface[i]->setup(android_config_driver.
						interface[i], ctrl);
			if (ret >= 0)
				return ret;
		}
	}
	return ret;
}

void usb_data_transfer_callback(void)
{
	struct device_mode_change_dev *dev_mode_change =
	    _device_mode_change_dev;

	if (dev_mode_change->usb_data_transfer_flag == 0) {
		dev_mode_change->usb_data_transfer_flag = 1;
		dev_mode_change->usb_get_desc_flag = 1;
		wake_up_interruptible
		    (&dev_mode_change->device_mode_change_wq);
	}
}

void usb_interface_enum_cb(int flag)
{
	struct device_mode_change_dev *dev_mode_change =
	    _device_mode_change_dev;

	dev_mode_change->usb_device_cfg_flag |= flag;
	if (dev_mode_change->usb_device_cfg_flag >=
	    dev_mode_change->g_device_type)
		wake_up_interruptible
		    (&dev_mode_change->device_mode_change_wq);
}

void adb_mode_change_cb(void)
{
	struct device_mode_change_dev *dev_mode_change =
	    _device_mode_change_dev;
	int ret;

	ret = wait_event_interruptible(dev_mode_change->adb_cb_wq,
		(!dev_mode_change->adb_mode_changed_flag));
	if (ret < 0)
		printk(KERN_ERR "adb_change_cb: %d\n", ret);

	dev_mode_change->adb_mode_changed_flag = 1;
	wake_up_interruptible(&dev_mode_change->device_mode_change_wq);
}

void tethering_mode_change_cb(void)
{
       struct device_mode_change_dev *dev_mode_change =
	       _device_mode_change_dev;
       int ret;

       ret = wait_event_interruptible(dev_mode_change->tethering_cb_wq,
		 (!dev_mode_change->tethering_mode_changed_flag));
	if (ret < 0) {
		printk(KERN_ERR "tethering_change_cb: %d\n", ret);
		return;
	}

       dev_mode_change->tethering_mode_changed_flag = 1;
       wake_up_interruptible(&dev_mode_change->device_mode_change_wq);

}

int tethering_enable_access(void)
{
	return atomic_read(&tethering_enable_excl);
}

static int product_has_function(struct android_usb_product *p,
				struct usb_function *f)
{
	char **functions = p->functions;
	int count = p->num_functions;
	const char *name = f->name;
	int i;

	for (i = 0; i < count; i++) {
		if (!strcmp(name, *functions++))
			return 1;
	}
	return 0;
}

static int product_matches_functions(struct android_usb_product *p)
{
	struct usb_function *f;
	list_for_each_entry(f, &android_config_driver.functions, list) {
		if (product_has_function(p, f) == !!f->hidden)
			return 0;
	}
	return 1;
}

static int get_product_id(struct android_dev *dev)
{
	struct android_usb_product *p = dev->products;
	int count = dev->num_products;
	int i;

	if (p) {
		for (i = 0; i < count; i++, p++) {
			if (product_matches_functions(p))
				return p->product_id;
		}
	}
	/* use default product ID */
	return dev->product_id;
}

static int __init android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget *gadget = cdev->gadget;
	int gcnum, id, product_id, ret;

	printk(KERN_INFO "android_bind\n");

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_CONFIG_IDX].id = id;
	android_config_driver.iConfiguration = id;


	/* Remove Remote Wakeup
	if (gadget->ops->wakeup)
		android_config_driver.bmAttributes |=
		    USB_CONFIG_ATT_WAKEUP;
	*/

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		printk(KERN_ERR "usb_add_config failed\n");
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			   longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	/* Do not Set Self Powered as WHQL tests are failing on Win7 */
	/*usb_gadget_set_selfpowered(gadget); */
	dev->cdev = cdev;
	product_id = get_product_id(dev);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	cdev->desc.idProduct = device_desc.idProduct;

	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name = "android_usb",
	.dev = &device_desc,
	.strings = dev_strings,
	.bind = android_bind,
	.enable_function = android_enable_function,
};

static void get_device_pid_vid(int type, int *pid, int *vid)
{
	int i;

	*vid = 0;
	*pid = 0;

	for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
		if (mot_android_vid_pid[i].type == type) {
			*pid = mot_android_vid_pid[i].pid;
			*vid = mot_android_vid_pid[i].vid;
			break;
		}
	}
}

int get_func_thru_type(int type)
{
	int i;

	for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
		if (mot_android_vid_pid[i].type == type)
			return i;
	}
	return -1;
}

char *get_name_thru_type(int type)
{
	int i;

	for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
		if (mot_android_vid_pid[i].type == type)
			return mot_android_vid_pid[i].name;
	}
	return NULL;

}

static int enable_android_usb_product_function(char *device_name, int cnt)
{
	struct usb_function *f;
	int enable = 1;
	int disable = 0;

	if (!strncmp(device_name, "eth", cnt - 1)) {
		list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "usbnet"))
				f->hidden = disable;
			else
				f->hidden = enable;
		}
		return 0;
	}
	if (!strncmp(device_name, "eth_adb", cnt - 1)) {
		list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "usbnet")
			    || !strcmp(f->name, "adb"))
				f->hidden = disable;
			else
				f->hidden = enable;
		}
		return 0;
	}
	if (!strncmp(device_name, "acm_eth_mtp", cnt - 1)) {
		list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "acm")
			    || !strcmp(f->name, "usbnet")
			    || !strcmp(f->name, "mtp"))
				f->hidden = disable;
			else
				f->hidden = enable;
		}
		return 0;
	}
	if (!strncmp(device_name, "acm_eth_mtp_adb", cnt - 1)) {
		list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "acm")
			    || !strcmp(f->name, "usbnet")
			    || !strcmp(f->name, "mtp")
			    || !strcmp(f->name, "adb"))
				f->hidden = disable;
			else
				f->hidden = enable;
		}
		return 0;
	}
	if (!strncmp(device_name, "mtp", cnt - 1)) {
		list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "mtp"))
				f->hidden = disable;
			else
				f->hidden = enable;
		}
		return 0;
	}
	if (!strncmp(device_name, "mtp_adb", cnt - 1)) {
		list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "mtp")
			    || !strcmp(f->name, "adb"))
				f->hidden = disable;
			else
				f->hidden = enable;
		}
		return 0;
	}
	if (!strncmp(device_name, "msc", cnt - 1)
	    || !strncmp(device_name, "msc_only", cnt - 1)
	    || !strncmp(device_name, "cdrom", cnt - 1)
	    || !strncmp(device_name, "charge_only", cnt - 1)) {
		list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "usb_mass_storage"))
				f->hidden = disable;
			else
				f->hidden = enable;
		}
		return 0;
	}
	if (!strncmp(device_name, "msc_adb", cnt - 1)
	    || !strncmp(device_name, "charge_adb", cnt - 1)) {
		list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "usb_mass_storage")
			    || !strcmp(f->name, "adb"))
				f->hidden = disable;
			else
				f->hidden = enable;
		}
		return 0;
	}
	if (!strncmp(device_name, "rndis", cnt - 1)) {
		list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "rndis"))
				f->hidden = disable;
			else
				f->hidden = enable;
		}
		return 0;
	}
	if (!strncmp(device_name, "rndis_adb", cnt - 1)) {
		list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "rndis")
			    || !strcmp(f->name, "adb"))
				f->hidden = disable;
			else
				f->hidden = enable;
		}
		return 0;
	}

	if (!strncmp(device_name, "ets", cnt - 1)) {
		list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "ets"))
				f->hidden = disable;
			else
				f->hidden = enable;
		}
		return 0;
	}

	if (!strncmp(device_name, "ets_adb", cnt - 1)) {
			list_for_each_entry(f, &android_config_driver.functions,
				    list) {
			if (!strcmp(f->name, "ets")
			    || !strcmp(f->name, "adb"))
				f->hidden = disable;
			else
				f->hidden = enable;
			}
		return 0;
	}

	printk(KERN_ERR "unknown mode: %s\n", device_name);
	return -1;
}

static void force_reenumeration(struct android_dev *dev, int dev_type)
{
	struct device_mode_change_dev *dev_mode_change =
	    _device_mode_change_dev;
	int vid, pid, i;

	/* using other namespace ??? */
	dev_mode_change->usb_device_cfg_flag = 0;
	dev_mode_change->usb_get_desc_flag = 0;
	dev_mode_change->usb_data_transfer_flag = 0;
	dev_mode_change->pc_mode_switch_flag = 0;

	get_device_pid_vid(dev_type, &pid, &vid);
	device_desc.idProduct = __constant_cpu_to_le16(pid);
	device_desc.idVendor = __constant_cpu_to_le16(vid);


	if (dev->cdev) {
		dev->cdev->desc.idProduct = device_desc.idProduct;
		dev->cdev->desc.idVendor = device_desc.idVendor;
		i = get_func_thru_type(dev_type);
		if (i != -1) {
			dev->cdev->desc.bDeviceClass =
			    mot_android_vid_pid[i].class;
			dev->cdev->desc.bDeviceSubClass =
			    mot_android_vid_pid[i].subclass;
			dev->cdev->desc.bDeviceProtocol =
			    mot_android_vid_pid[i].protocol;
		}

	}

	if (dev->cdev && dev->cdev->gadget) {
		/* dev->cdev->gadget->speed != USB_SPEED_UNKNOWN ? */

		/* According to our test, the interval between D+ pullup
		 * and next pulldown operation shall be bigger than 50ms,
		 * otherwise there will be enumeration issue.  It shall be
		 * relevant with USB transceiver on device/host side.
		 *  Sleep 50ms here to make it smoothly
		 */
		usb_composite_force_reset(dev->cdev);
	}
}

void android_register_function(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "android_register_function %s\n", f->name);
	list_add_tail(&f->list, &_functions);
	_registered_function_count++;

	/* bind our functions if they have all registered
	 * and the main driver has bound.
	 */
	if (dev && dev->config
	    && _registered_function_count == dev->num_functions)
		bind_functions(dev);
}

void android_enable_function(struct usb_function *f, int enable)
{
	struct android_dev *dev = _android_dev;
	int disable = !enable;
	int product_id;
	char *func_name;
	int func_name_len;
	int adb_enable = 0;

	if (!!f->hidden != disable) {
		if (!strcmp(f->name, "rndis")) {
			if (enable)
				atomic_set(&tethering_enable_excl, 1);
			else
				atomic_set(&tethering_enable_excl, 0);
			tethering_mode_change_cb();
			return;
		}
	}
}

/*
 * Device is used for USB mode switch
 */
static int device_mode_change_open(struct inode *ip, struct file *fp)
{
	struct device_mode_change_dev *dev_mode_change =
	    _device_mode_change_dev;

	if (atomic_inc_return(&dev_mode_change->device_mode_change_excl) !=
	    1) {
		atomic_dec(&dev_mode_change->device_mode_change_excl);
		return -EBUSY;
	}
	return 0;
}

static int device_mode_change_release(struct inode *ip, struct file *fp)
{
	struct device_mode_change_dev *dev_mode_change =
	    _device_mode_change_dev;

	atomic_dec(&dev_mode_change->device_mode_change_excl);
	dev_mode_change->adb_mode_changed_flag = 0;
	return 0;
}

static ssize_t
device_mode_change_write(struct file *file, const char __user * buffer,
			 size_t count, loff_t *ppos)
{
	struct device_mode_change_dev *dev_mode_change =
	    _device_mode_change_dev;
	unsigned char cmd[MAX_DEVICE_NAME_SIZE + 1];
	int cnt = MAX_DEVICE_NAME_SIZE;
	int i, temp_device_type;
	int ret;

	if (count <= 0) {
		printk(KERN_ERR "%s - buffer size is 0 \n", __func__);
		return -EFAULT;
	}

	if (cnt > count)
		cnt = count;

	if (copy_from_user(cmd, buffer, cnt)) {
		printk(KERN_ERR "%s -  Error Copying buffer \n", __func__);
		return -EFAULT;
	}
	cmd[cnt] = 0;

	printk(KERN_INFO "%s Mode change command=%s\n", __func__, cmd);

	/* USB cable detached Command */
	if (strncmp(cmd, "usb_cable_detach", 16) == 0) {
		dev_mode_change->usb_data_transfer_flag = 0;
		dev_mode_change->g_device_type = 0;
		dev_mode_change->usb_device_cfg_flag = 0;
		dev_mode_change->usb_get_desc_flag = 0;
		usb_gadget_disconnect(_android_dev->cdev->gadget);
		/*
		 * Set the composite switch to 0 during a disconnect.
		 * This is required to handle a few corner cases, where
		 * enumeration has not completed, but the cable is yanked out
		 */
		switch_set_state(&_android_dev->cdev->sdev, 0);
		printk(KERN_INFO "%s - Handled Detach\n", __func__);
		return count;
	}

	/* USB connect/disconnect Test Commands */
	if (strncmp(cmd, "usb_connect", 11) == 0) {
		usb_gadget_connect(_android_dev->cdev->gadget);
		printk(KERN_INFO "%s - Handled Connect\n", __func__);
		return count;
	}
	if (strncmp(cmd, "usb_disconnect", 14) == 0) {
		usb_gadget_disconnect(_android_dev->cdev->gadget);
		printk(KERN_INFO "%s - Handled disconnect\n", __func__);
		return count;
	}

	for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
		if (mot_android_vid_pid[i].name == NULL) {
			printk(KERN_ERR "%s - Function Not Found \n" ,
				__func__);
			return count;
		}
		if (strlen(mot_android_vid_pid[i].name) > cnt)
			continue;
		if (strncmp(cmd, mot_android_vid_pid[i].name, cnt - 1) ==
		    0) {
			temp_device_type = mot_android_vid_pid[i].type;
			strings_dev[STRING_CONFIG_IDX].s =
			    mot_android_vid_pid[i].config_name;
			break;
		}
	}

	if (i == MAX_DEVICE_TYPE_NUM) {
		printk(KERN_ERR "%s - No Matching Function Found \n" ,
			__func__);
		return count;
	}

	if (temp_device_type == dev_mode_change->g_device_type) {
		printk(KERN_ERR "%s - Function already configured \n" ,
			__func__);
		return count;
	}

	ret = enable_android_usb_product_function(cmd, cnt);
	if (ret != 0) {
		printk(KERN_ERR "%s - Error Enabling Function \n" ,
			__func__);
		return -EFAULT;
	}

	dev_mode_change->g_device_type = temp_device_type;
	force_reenumeration(_android_dev, dev_mode_change->g_device_type);
	printk(KERN_INFO "%s - Successfully enabled function - %s \n",
		__func__, cmd);

	return count;
}

static int event_pending(void)
{
	struct device_mode_change_dev *dev_mode_change =
	    _device_mode_change_dev;

	if ((dev_mode_change->usb_device_cfg_flag >=
	     dev_mode_change->g_device_type)
	    && (dev_mode_change->g_device_type != 0))
		return 1;
	else if (dev_mode_change->adb_mode_changed_flag)
		return 1;
	else if (dev_mode_change->tethering_mode_changed_flag)
		return 1;
	else if (dev_mode_change->usb_get_desc_flag)
		return 1;
	else if (dev_mode_change->pc_mode_switch_flag)
		return 1;
	else
		return 0;
}

static unsigned int device_mode_change_poll(struct file *file,
					    struct poll_table_struct *wait)
{
	struct device_mode_change_dev *dev_mode_change =
	    _device_mode_change_dev;

	poll_wait(file, &dev_mode_change->device_mode_change_wq, wait);

	if (event_pending())
		return POLLIN | POLLRDNORM;
	else
		return 0;
}

static ssize_t device_mode_change_read(struct file *file, char *buf,
				       size_t count, loff_t *ppos)
{
	struct device_mode_change_dev *dev_mode_change =
	    _device_mode_change_dev;
	int ret, size, cnt;
	/* double check last zero */
	unsigned char no_changed[] = "none:\0";
	unsigned char adb_en_str[] = "adb_enable:\0";
	unsigned char adb_dis_str[] = "adb_disable:\0";
	unsigned char tethering_en_str[] = "tethering_enable:\0";
	unsigned char tethering_dis_str[] = "tethering_disable:\0";
	unsigned char enumerated_str[] = "enumerated\0";
	unsigned char get_desc_str[] = "get_desc\0";
	unsigned char modswitch_str[50];

	/* Message format example:
	 * none:adb_enable:enumerated
	 */

	if (!event_pending())
		return 0;

	/* append PC request mode */
	if (!dev_mode_change->pc_mode_switch_flag) {
		size = strlen(no_changed);
		ret = copy_to_user(buf, no_changed, size);
	} else {
		memset(modswitch_str, 0, 50);
		ret =
		    get_func_thru_config
		    (dev_mode_change->pc_mode_switch_flag);
		dev_mode_change->pc_mode_switch_flag = 0;
		if (ret == -1) {
			size = strlen(no_changed);
			ret = copy_to_user(buf, no_changed, size);
		} else {
			sprintf(modswitch_str, "%s",
				mot_android_vid_pid[ret].name);
			strcat(modswitch_str, ":");
			size = strlen(modswitch_str);
			ret = copy_to_user(buf, modswitch_str, size);
		}
	}
	cnt = size;
	buf += size;

	/* append ADB status */
	if (!dev_mode_change->adb_mode_changed_flag) {
		size = strlen(no_changed);
		ret = copy_to_user(buf, no_changed, size);
	} else {
		if (adb_enable_access()) {
			size = strlen(adb_en_str);
			ret = copy_to_user(buf, adb_en_str, size);
		} else {
			size = strlen(adb_dis_str);
			ret = copy_to_user(buf, adb_dis_str, size);
		}
		dev_mode_change->adb_mode_changed_flag = 0;
		wake_up_interruptible(&dev_mode_change->adb_cb_wq);
	}
	cnt += size;
	buf += size;

	/* append tethering status */
	if (!dev_mode_change->tethering_mode_changed_flag) {
		size = strlen(no_changed);
		ret = copy_to_user(buf, no_changed, size);
	} else {
		if (tethering_enable_access()) {
			size = strlen(tethering_en_str);
			ret = copy_to_user(buf, tethering_en_str, size);
		} else {
			size = strlen(tethering_dis_str);
			ret = copy_to_user(buf, tethering_dis_str, size);
		}
		dev_mode_change->tethering_mode_changed_flag = 0;
		wake_up_interruptible(&dev_mode_change->tethering_cb_wq);
	}
	cnt += size;
	buf += size;


	/* append USB enumerated state */
	if ((dev_mode_change->usb_device_cfg_flag >=
	     dev_mode_change->g_device_type)
	    && (dev_mode_change->g_device_type != 0)) {
		dev_mode_change->usb_device_cfg_flag = 0;
		size = strlen(enumerated_str);
		ret += copy_to_user(buf, enumerated_str, size);

	} else {
		if (dev_mode_change->usb_get_desc_flag == 1) {
			dev_mode_change->usb_get_desc_flag = 0;
			size = strlen(get_desc_str);
			ret += copy_to_user(buf, get_desc_str, size);
		} else {
			size = strlen(no_changed) - 1;
			ret += copy_to_user(buf, no_changed, size);
		}
	}
	cnt += size;

	return cnt;
}

static const struct file_operations device_mode_change_fops = {
	.owner = THIS_MODULE,
	.open = device_mode_change_open,
	.write = device_mode_change_write,
	.poll = device_mode_change_poll,
	.read = device_mode_change_read,
	.release = device_mode_change_release,
};

static struct miscdevice mode_change_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "usb_device_mode",
	.fops = &device_mode_change_fops,
};



static int __init android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "android_probe pdata: %p\n", pdata);

	if (pdata) {
		dev->products = pdata->products;
		dev->num_products = pdata->num_products;
		dev->functions = pdata->functions;
		dev->num_functions = pdata->num_functions;
		if (pdata->vendor_id)
			device_desc.idVendor =
			    __constant_cpu_to_le16(pdata->vendor_id);
		if (pdata->product_id) {
			dev->product_id = pdata->product_id;
			device_desc.idProduct =
			    __constant_cpu_to_le16(pdata->product_id);
		}
		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s =
			    pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
			    pdata->manufacturer_name;
		if (pdata->serial_number)
			strings_dev[STRING_SERIAL_IDX].s =
			    pdata->serial_number;
	}

	return usb_composite_register(&android_usb_driver);
}

static struct platform_driver android_platform_driver = {
	.driver = {.name = "android_usb",},
	.probe = android_probe,
};

static int __init init(void)
{
	struct android_dev *dev;
	struct device_mode_change_dev *dev_mode_change;
	int ret;

	printk(KERN_INFO "android init\n");

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	/* set default values, which should be overridden by platform data */
	dev->product_id = PRODUCT_ID;
	_android_dev = dev;

	/* allocate device_mode_change dev and wait queue */
	dev_mode_change = kzalloc(sizeof(*dev_mode_change), GFP_KERNEL);
	if (!dev_mode_change) {
		kfree(_android_dev);
		return -ENOMEM;
	}
	_device_mode_change_dev = dev_mode_change;
	init_waitqueue_head(&dev_mode_change->device_mode_change_wq);
	init_waitqueue_head(&dev_mode_change->adb_cb_wq);
	init_waitqueue_head(&dev_mode_change->tethering_cb_wq);

	dev_mode_change->adb_mode_changed_flag = 0;
	dev_mode_change->tethering_mode_changed_flag = 0;
	_registered_function_count = 0;
	atomic_set(&tethering_enable_excl, 0);

	dev->sdev.name = "usb_connected";
	ret = switch_dev_register(&dev->sdev);
	if (ret < 0)
		return ret;

	ret = platform_driver_register(&android_platform_driver);
	if (ret) {
		switch_dev_unregister(&dev->sdev);
		kfree(_android_dev);
		kfree(_device_mode_change_dev);
		return ret;
	}
	ret = misc_register(&mode_change_device);
	if (ret) {
		switch_dev_unregister(&dev->sdev);
		kfree(_android_dev);
		kfree(_device_mode_change_dev);
		platform_driver_unregister(&android_platform_driver);
	}
	return ret;
}

module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	misc_deregister(&mode_change_device);
	platform_driver_unregister(&android_platform_driver);
	kfree(_device_mode_change_dev);
	_device_mode_change_dev = NULL;
	switch_dev_unregister(&(_android_dev->sdev));
	kfree(_android_dev);
	_android_dev = NULL;
}

module_exit(cleanup);
