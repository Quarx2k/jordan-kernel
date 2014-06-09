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

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/spi/cpcap.h>

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

#include "f_audio_source.c"
#include "f_mass_storage.c"
#include "u_serial.c"
#include "f_acm.c"
#include "f_adb.c"
#include "f_mtp.c"
#include "f_accessory.c"
#define USB_ETH_RNDIS y
#include "f_rndis.c"
#include "rndis.c"
#include "u_ether.c"
#include "f_usbnet.c"

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

struct android_usb_platform_data *andusb_plat;

static const char longname[] = "Gadget Android";

#define ID_VENDOR_MOTO 0x22b8
#define ID_PRODUCT_ETH 0x42d2
#define ID_PRODUCT_ETH_ADB 0x42d3
#define ID_PRODUCT_RNDIS_ACM_ETH_ADB 0x4304
#define ID_PRODUCT_ACM_ETH_ADB 0x4302
#define ID_PRODUCT_RNDIS_ACM_ETH 0x4303
#define ID_PRODUCT_ACM_ETH 0x4301
#define ID_PRODUCT_STE_CONF_ACM_ETH_ADB 0x4313

/* Default vendor and product IDs, overridden by userspace */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001


struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	/* for android_dev.enabled_functions */
	struct list_head enabled_list;

	/* Optional: initialization during gadget bind */
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct android_usb_function *);
	/* Optional: called when the function is added the list of
	 *		enabled functions */
	void (*enable)(struct android_usb_function *);
	/* Optional: called when it is removed */
	void (*disable)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *, struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct android_usb_function *, struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured */
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};

enum function_type {
	CDROM = 0,
	CDROM2,
	MTPUSBNET
};


struct android_dev {
	struct android_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;

	bool enabled;
        int disable_depth;
        struct mutex mutex;
	bool connected;
	bool sw_connected;
	struct work_struct work;
	enum function_type current_function_type;
	struct work_struct enumeration_work;
	int cdrom_enable;
	int cdrom_mount;
	int native_cdrom;
};

static struct class *android_class;
static struct android_dev *_android_dev;
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);

/* string IDs are assigned dynamically */
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];
static char cdrom_blkdev_path[256];


/* String Table */
static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.unbind		= android_unbind_config,
	.bConfigurationValue = 1,
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower	= 0xFA, /* 500ma */
};

static enum cpcap_accy cable_type = CPCAP_ACCY_NONE;

#define PC_COMMAND_ADB_ON 1
#define PC_COMMAND_ADB_OFF 2
static int pc_command_adb;

#define PC_COMMAND_MODEM_ON 1
#define PC_COMMAND_MODEM_OFF 2
static int pc_command_modem;

int android_usb_get_pid(char *fname)
{
	int j;

	for (j = 0; j < MAX_DEVICE_TYPE_NUM; j++) {
		if (andusb_plat->android_pid[j].name == NULL)
			break;

		if (strncmp(andusb_plat->android_pid[j].name, fname,
			MAX_DEVICE_NAME_SIZE) == 0) {
			return andusb_plat->android_pid[j].pid;
		}
	}

	printk(KERN_ERR "Unable to find a match for the usb mode  %s \n",
			fname);
	return 0;
}


void android_usb_set_connected(int connected, unsigned int accy)
{
	struct android_dev *dev = _android_dev;

	printk(KERN_INFO "%s - connected = %d, accy = %d\n",
	       __func__, connected, accy);

	switch (accy) {
	case CPCAP_ACCY_USB:
		if (connected)
			cable_type = CPCAP_ACCY_USB;
		else {
			cable_type = CPCAP_ACCY_NONE;
			dev->connected = 0;
			schedule_work(&dev->work);
		}
		break;

	case CPCAP_ACCY_FACTORY:
		if (connected)
			cable_type = CPCAP_ACCY_FACTORY;
		else
			cable_type = CPCAP_ACCY_NONE;
		break;

	default:
		return;
	}
}



static int mass_storage_function_set_cdrom_lun(char *lunpath);

static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
	char **uevent_envp = NULL;
	unsigned long flags;
	char ch = 0;
	int rc = 0;

	spin_lock_irqsave(&cdev->lock, flags);
	if (dev->connected != dev->sw_connected)
		uevent_envp = dev->connected ? connected : disconnected;
	else if (cdev->config)
		uevent_envp = configured;
	dev->sw_connected = dev->connected;
	spin_unlock_irqrestore(&cdev->lock, flags);

	/* update the lun sys file */
	if (dev->cdrom_enable) {
		/*
		 * mount cdrom only when connect with cable,
		 * cdrom image was not mounted, function type
		 *  is not "mtp,usbnet".
		 */
		if (!dev->cdrom_mount && cable_type == CPCAP_ACCY_USB
		    && dev->current_function_type != MTPUSBNET) {
			/* mount cdrom image */
			pr_info("%s: CDROM, mount cdrom image\n", __func__);
			rc = mass_storage_function_set_cdrom_lun(cdrom_blkdev_path);
			if (!rc)
				dev->cdrom_mount = 1;
			else
				pr_err("%s: mount cdrom image fail\n",
				       __func__);
		}
	} else if (dev->cdrom_mount) {
		pr_info("%s: none cdrom mode, umount cdrom image\n", __func__);
		rc = mass_storage_function_set_cdrom_lun(&ch);
		if (!rc)
			dev->cdrom_mount = 0;
		else
			pr_err("%s: umount cdrom image fail\n", __func__);
	}

	if (!dev->connected && cable_type != CPCAP_ACCY_NONE) {
		pr_debug("%s: fake disconnect uevent, ignore\n", __func__);
		return;
	}

	if (uevent_envp) {
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
		pr_info("%s: sent uevent %s\n", __func__, uevent_envp[0]);
	} else {
		pr_info("%s: did not send uevent (%d %d %p)\n", __func__,
			 dev->connected, dev->sw_connected, cdev->config);
	}
}

static int android_enable_function(struct android_dev *dev, char *name);

static char *get_function_name(struct android_dev *dev)
{
	static char function_name[50];
	int pid;

	device_desc.idVendor = 0x22b8;
	switch (dev->current_function_type) {
	case CDROM:
		/* cdrom */
		pid = android_usb_get_pid("cdrom");
		device_desc.idProduct = pid ? pid : 0x437f;
		device_desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceSubClass = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceProtocol = USB_CLASS_PER_INTERFACE;
		strncpy(function_name, "mass_storage", sizeof(function_name));
		break;
	case CDROM2:
		/* cdrom2 */
		pid = android_usb_get_pid("cdrom2");
		device_desc.idProduct = pid ? pid : 0x41ce;
		device_desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceSubClass = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceProtocol = USB_CLASS_PER_INTERFACE;
		strncpy(function_name, "mass_storage", sizeof(function_name));
		break;
	case MTPUSBNET:
		/* mtpusbnet */
		pid = android_usb_get_pid("mtp,usbnet");
		device_desc.idProduct = pid ? pid : 0x4361;
		device_desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceSubClass = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceProtocol = USB_CLASS_PER_INTERFACE;
		strncpy(function_name, "mtp,usbnet", sizeof(function_name));
		break;
	default:
		/* mtpusbnet */
		pid = android_usb_get_pid("mtp,usbnet");
		device_desc.idProduct = pid ? pid : 0x4361;
		device_desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceSubClass = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceProtocol = USB_CLASS_PER_INTERFACE;
		strncpy(function_name, "mtp,usbnet", sizeof(function_name));
		break;
	}
	return function_name;
}

static ssize_t
functions_core_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
        struct android_dev *dev = dev_get_drvdata(pdev);
        struct android_usb_function *f;
        char *buff = buf;

        list_for_each_entry(f, &dev->enabled_functions, enabled_list)
                buff += sprintf(buff, "%s,", f->name);
        if (buff != buf)
                *(buff-1) = '\n';
        return buff - buf;
}

static ssize_t
functions_core_store(struct device *pdev, struct device_attribute *attr,
                const char *buff, size_t size)
{
        struct android_dev *dev = dev_get_drvdata(pdev);
        char *name;
        char buf[256], *b;
        int err;

        INIT_LIST_HEAD(&dev->enabled_functions);

        strncpy(buf, buff, sizeof(buf)-1);
        buf[sizeof(buf)-1] = '\0';

        if (!strcmp(buf, "cdrom")) {
                dev->cdrom_enable = 1;
                dev->native_cdrom = 1;
                dev->current_function_type = CDROM;
                strncpy(buf, get_function_name(dev), sizeof(buf));
        } else if (strstr(buf, "mass_storage")) {
                /* Use the module parameter "cdrom" value */
                dev->cdrom_enable = cdrom_enable;
                dev->native_cdrom = 0;
                dev->current_function_type = CDROM;
        } else {
                dev->cdrom_enable = 0;
                dev->native_cdrom = 0;
        }

        b = strim(buf);

        while (b) {
                name = strsep(&b, ",");
                if (name) {
                        if (!((pc_command_adb == PC_COMMAND_ADB_OFF)
                                                && (!strcmp(name, "adb")))) {
                                err = android_enable_function(dev, name);
                                if (err)
                                        pr_err("android_usb: Cannot enable '%s'\n", name);
                        }
                }
        }
        if ((!strstr(buff, "adb")) && (pc_command_adb == PC_COMMAND_ADB_ON)) {
                err = android_enable_function(dev, "adb");
                if (err)
                        pr_err("android_usb: Cannot enable adb for pc command\n");
        }
        return size;
}


static void android_enumeration_work(struct work_struct *data)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	char *name;
	char buf[256], *b;
	int err = 0;
	char ch = 0;

	/* disconnect and remove config */
	usb_gadget_disconnect(cdev->gadget);
	usb_remove_config(cdev, &android_config_driver);
	dev->enabled = false;

	/*
	 * only umount when cdrom image was mounted,
	 * and function type is "mtp,usbnet".
	 */
	if (dev->cdrom_mount && dev->current_function_type == MTPUSBNET) {
		/* umount cdrom image */
		pr_info("%s: MTPUSBNET, umount cdrom image\n", __func__);
		err = mass_storage_function_set_cdrom_lun(&ch);
		if (!err)
			dev->cdrom_mount = 0;
		else
			pr_err("%s: umount cdrom image fail\n", __func__);
	}


	INIT_LIST_HEAD(&dev->enabled_functions);
	strncpy(buf, get_function_name(dev), sizeof(buf));
	pr_info("%s: reenumerate as: %s\n", __func__, buf);
	b = strim(buf);

	while (b) {
		name = strsep(&b, ",");
		if (name) {
			err = android_enable_function(dev, name);
			if (err)
				pr_err("android_usb: Cannot enable '%s'", name);
		}
	}

	/* connect */
	/* update values in composite driver's copy of device descriptor */
	cdev->desc.idVendor = device_desc.idVendor;
	cdev->desc.idProduct = device_desc.idProduct;
	cdev->desc.bcdDevice = device_desc.bcdDevice;
	cdev->desc.bDeviceClass = device_desc.bDeviceClass;
	cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
	cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
	usb_add_config(cdev, &android_config_driver,
		       android_bind_config);
	usb_gadget_connect(cdev->gadget);
	dev->enabled = true;

	return;
}


void update_function_type_and_reenumerate(int index)
{
	struct android_dev *dev = _android_dev;

	/* update function type */
	if (index == CDROM_INDEX)
		dev->current_function_type = CDROM;
	else if (index == CDROM2_INDEX)
		dev->current_function_type = CDROM2;
	else if (index == MTPUSBNET_INDEX)
		dev->current_function_type = MTPUSBNET;
	else {
		pr_err("invalidate switch index: 0x%x\n", index);
		return;
	}
	pr_info("%s: schedule re-enumeration work\n", __func__);
	schedule_work(&dev->enumeration_work);
}

void set_cdrom_umount(void)
{
	struct android_dev *dev = _android_dev;

	dev->cdrom_mount = 0;
	return;
}

static void android_enable(struct android_dev *dev)
{
       struct usb_composite_dev *cdev = dev->cdev;

        BUG_ON(!mutex_is_locked(&dev->mutex));
        BUG_ON(!dev->disable_depth);

       if (--dev->disable_depth == 0) {
               usb_add_config(cdev, &android_config_driver,
                                       android_bind_config);
               usb_gadget_connect(cdev->gadget);
       }
}

static void android_disable(struct android_dev *dev)
{
       struct usb_composite_dev *cdev = dev->cdev;

       BUG_ON(!mutex_is_locked(&dev->mutex));

       if (dev->disable_depth++ == 0) {
               usb_gadget_disconnect(cdev->gadget);
               /* Cancel pending control requests */
               usb_ep_dequeue(cdev->gadget->ep0, cdev->req);
               usb_remove_config(cdev, &android_config_driver);
       }
}
/*-------------------------------------------------------------------------*/
/* Supported functions initialization */

struct adb_data {
       bool opened;
       bool enabled;
};

static int adb_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
        f->config = kzalloc(sizeof(struct adb_data), GFP_KERNEL);
        if (!f->config)
                return -ENOMEM;
	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	adb_cleanup();
        kfree(f->config);
}

static int adb_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return adb_bind_config(c);
}

static void adb_android_function_enable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = true;

	/* Disable the gadget until adbd is ready */
	if (!data->opened)
		android_disable(dev);
}

static void adb_android_function_disable(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = false;

	/* Balance the disable that was called in closed_callback */
	if (!data->opened)
		android_enable(dev);
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.enable		= adb_android_function_enable,
	.disable	= adb_android_function_disable,
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
};

static void adb_ready_callback(void)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

        mutex_lock(&dev->mutex);

	data->opened = true;

	if (data->enabled)
		android_enable(dev);

        mutex_unlock(&dev->mutex);
}

static void adb_closed_callback(void)
{
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

        mutex_lock(&dev->mutex);
	data->opened = false;

	if (data->enabled)
		android_disable(dev);

        mutex_unlock(&dev->mutex);
}


#define MAX_ACM_INSTANCES 4
struct acm_function_config {
	int instances;
};

static int acm_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	struct acm_function_config *acm_config;

	f->config = kzalloc(sizeof(struct acm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	/*Set default enabled acm number to be 1.*/
	acm_config = f->config;
	if (andusb_plat->bp_tools_mode)
		acm_config->instances = 4;
	else
		/*Set default enabled acm number to be 1.*/
		acm_config->instances = 1;

	return gserial_setup(cdev->gadget, MAX_ACM_INSTANCES);
}

static void acm_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
	kfree(f->config);
	f->config = NULL;
}

static int acm_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	int i;
	int ret = 0;
	struct acm_function_config *config = f->config;

	for (i = 0; i < config->instances; i++) {
		ret = acm_bind_config(c, i);
		if (ret) {
			pr_err("Could not bind acm%u config\n", i);
			break;
		}
	}

	return ret;
}

static ssize_t acm_instances_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->instances);
}

static ssize_t acm_instances_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	int value;

	sscanf(buf, "%d", &value);
	if (value > MAX_ACM_INSTANCES)
		value = MAX_ACM_INSTANCES;
	config->instances = value;
	return size;
}

static DEVICE_ATTR(instances, S_IRUGO | S_IWUSR, acm_instances_show, acm_instances_store);
static struct device_attribute *acm_function_attributes[] = { &dev_attr_instances, NULL };

static struct android_usb_function acm_function = {
	.name		= "acm",
	.init		= acm_function_init,
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.attributes	= acm_function_attributes,
};


static int mtp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return mtp_setup();
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int mtp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int ptp_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	/* nothing to do - initialization is handled by mtp_function_init */
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	/* nothing to do - cleanup is handled by mtp_function_cleanup */
}

static int ptp_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return mtp_ctrlrequest(cdev, c);
}

static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
};

/* PTP function is same as MTP with slightly different interface descriptor */
static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
};


struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	bool	wceis;
};

static int rndis_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int rndis_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct rndis_function_config *rndis = f->config;

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, rndis->ethaddr, "rndis");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	if (rndis->wceis) {
		/* "Wireless" RNDIS; auto-detected by Windows */
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

	if (andusb_plat->performance_mode)
		andusb_plat->performance_mode(&(c->cdev->gadget->dev), true);
	return rndis_bind_config(c, rndis->ethaddr, rndis->vendorID,
				    rndis->manufacturer);
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
	if (andusb_plat->performance_mode)
		andusb_plat->performance_mode(&(c->cdev->gadget->dev), false);
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;
	if (sscanf(buf, "%s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
};


struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err;
	int j;
	char lunname[10];

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	config->fsg.cdrom_lun_num = andusb_plat->cdrom_lun_num;
	config->fsg.nluns = andusb_plat->nluns;
	for (j = 0; j < andusb_plat->nluns; j++)
		config->fsg.luns[j].removable = 1;

	config->fsg.vendor_name = andusb_plat->vendor;
	config->fsg.product_name = andusb_plat->product_name;


	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	for (j = 0; j < andusb_plat->nluns; j++) {
		sprintf(lunname, "lun%d", j);

		err = sysfs_create_link(&f->dev->kobj,
				&common->luns[j].dev.kobj,
				lunname);
		if (err) {
			kfree(config);
			return err;
		}
	}

	config->common = common;
	f->config = config;
	return 0;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return sprintf(buf, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (sscanf(buf, "%s", config->common->inquiry_string) != 1)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
	NULL
};

static int mass_storage_function_ctrlrequest(struct android_usb_function *f,
					     struct usb_composite_dev *cdev,
					     const struct usb_ctrlrequest *c)
{
	return fsg_ctrlrequest(cdev, c);
}

static int mass_storage_function_set_cdrom_lun(char *lunpath)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	struct mass_storage_function_config *config;
	int cdromlun_num;
	struct fsg_lun  *cdromlun;
	int rc = 0;

	/* look up for mass storage function instance */
	while ((f = *functions++)) {
		if (!strcmp("mass_storage", f->name)) {
			pr_info("get mass storage function instance\n");
			break;
		}
	}

	if (f == NULL) {
		pr_info("Can't find mass storage function\n");
		return -1;
	}
	config = f->config;
	cdromlun_num = config->fsg.cdrom_lun_num;
	cdromlun = &(config->common->luns[cdromlun_num]);

	/* Eject current medium */
	if (fsg_lun_is_open(cdromlun)) {
		pr_info("eject current medium\n");
		fsg_lun_close(cdromlun);
		cdromlun->unit_attention_data = SS_MEDIUM_NOT_PRESENT;
	}

	/* Load new medium */
	if (lunpath[0]) {
		pr_info("load new medium, lunpath: %s\n", lunpath);
		rc = fsg_lun_open(cdromlun, lunpath);
		if (rc == 0)
			cdromlun->unit_attention_data =
				SS_NOT_READY_TO_READY_TRANSITION;
	}

	return rc;

}

static struct android_usb_function mass_storage_function = {
	.name		= "mass_storage",
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
	.ctrlrequest    = mass_storage_function_ctrlrequest,
};


static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};

static int usbnet_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct usbnet_device *dev;
	struct usbnet_context *context;
	struct net_device *net_dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	net_dev = alloc_netdev(sizeof(struct usbnet_context),
		"usb%d", usb_ether_setup);
	if (!net_dev) {
		pr_err("%s: alloc_netdev error\n", __func__);
		kfree(dev);
		return -EINVAL;
	}

	ret = register_netdev(net_dev);
	if (ret) {
		pr_err("%s: register_netdev error\n", __func__);
		free_netdev(net_dev);
		kfree(dev);
		return -EINVAL;
	}

	ret = device_create_file(&net_dev->dev, &dev_attr_description);
	if (ret < 0) {
		pr_err("%s: sys file creation  error\n", __func__);
		unregister_netdev(net_dev);
		free_netdev(net_dev);
		kfree(dev);
		return -EINVAL;
	}

	context = netdev_priv(net_dev);
	INIT_WORK(&context->usbnet_config_wq, usbnet_if_config);

	context->config = 0;
	dev->net_ctxt = context;

	f->config = dev;

	switch_dev_register(&usbnet_enable_device);
	return 0;
}

static void usbnet_function_cleanup(struct android_usb_function *f)
{
	struct usbnet_device *dev = f->config;

	usbnet_cleanup(dev);
	switch_dev_unregister(&usbnet_enable_device);
	kfree(dev);
}

static int usbnet_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	struct usbnet_device *dev = f->config;

	return usbnet_ctrlrequest(dev, cdev, c);
}

static int usbnet_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	struct usbnet_device *dev = f->config;

	return usbnet_bind_config(dev, c);
}

static struct android_usb_function usbnet_function = {
	.name		= "usbnet",
	.init		= usbnet_function_init,
	.cleanup	= usbnet_function_cleanup,
	.bind_config	= usbnet_function_bind_config,
	.ctrlrequest	= usbnet_function_ctrlrequest,
};
static int audio_source_function_init(struct android_usb_function *f,
			struct usb_composite_dev *cdev)
{
	struct audio_source_config *config;

	config = kzalloc(sizeof(struct audio_source_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	f->config = config;
	return 0;
}

static void audio_source_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
}

static int audio_source_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	return audio_source_bind_config(c, config);
}

static void audio_source_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	config->card = -1;
	config->device = -1;
}

static ssize_t audio_source_pcm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;

	/* print PCM card and device numbers */
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(pcm, S_IRUGO | S_IWUSR, audio_source_pcm_show, NULL);

static struct device_attribute *audio_source_function_attributes[] = {
	&dev_attr_pcm,
	NULL
};

static struct android_usb_function audio_source_function = {
	.name		= "audio_source",
	.init		= audio_source_function_init,
	.cleanup	= audio_source_function_cleanup,
	.bind_config	= audio_source_function_bind_config,
	.unbind_config	= audio_source_function_unbind_config,
	.attributes	= audio_source_function_attributes,
};

static struct android_usb_function *supported_functions[] = {
	&adb_function,
	&acm_function,
	&mtp_function,
	&ptp_function,
	&rndis_function,
	&mass_storage_function,
	&accessory_function,
	&usbnet_function,
	&audio_source_function,
	NULL
};


static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err;
	int index = 0;

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_out;
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_out;
		}
	}
	return 0;

err_out:
	device_destroy(android_class, f->dev->devt);
err_create:
	kfree(f->dev_name);
	return err;
}

static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		}

		if (f->cleanup)
			f->cleanup(f);
	}
}

static int
android_bind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;
	int ret;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		ret = f->bind_config(f, c);
		if (ret) {
			pr_err("%s: %s failed", __func__, f->name);
			return ret;
		}
	}
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->unbind_config)
			f->unbind_config(f, c);
	}
}

static int android_enable_function(struct android_dev *dev, char *name)
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	while ((f = *functions++)) {
		if (!strcmp(name, f->name)) {
			list_add_tail(&f->enabled_list, &dev->enabled_functions);
			return 0;
		}
	}
	return -EINVAL;
}


/*-------------------------------------------------------------------------*/
/* /sys/class/android_usb/android%d/ interface */

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
        ssize_t size;
	struct android_dev *dev = dev_get_drvdata(pdev);
        mutex_lock(&dev->mutex);

        size =functions_core_show(pdev, attr, buf);

        mutex_unlock(&dev->mutex);
        return size;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
        mutex_lock(&dev->mutex);

        functions_core_store(pdev, attr, buff, size);

        mutex_unlock(&dev->mutex);
	return size;
}

static ssize_t
smversion_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	if (sm_vers_sz > 0)
		memcpy(buf, sm_vers, sm_vers_sz);
	return sm_vers_sz;
}

static ssize_t
smversion_store(struct device *pdev, struct device_attribute *attr,
		const char *buff, size_t size)
{
	/* update the smart version buff and sm size */
	sm_vers_sz = size;
	memset(sm_vers, 0, sizeof(sm_vers));
	memcpy(sm_vers, buff, size);

	return size;
}

static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return sprintf(buf, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	struct android_usb_function *f;
	int enabled = 0;
	char get_buf[256], set_buf[256], *end_char_addr;
	int functions_len;
	int pid;

        mutex_lock(&dev->mutex);

	sscanf(buff, "%d", &enabled);
	printk("%s: set enable = %d; old enable = %d\n", __func__,
			enabled, dev->enabled);
	if (enabled && !dev->enabled) {
		if (cable_type == CPCAP_ACCY_FACTORY) {
			device_desc.idVendor = ID_VENDOR_MOTO;
			if (pc_command_adb == PC_COMMAND_ADB_ON &&
				(pc_command_modem == PC_COMMAND_MODEM_ON)) {
				strncpy(set_buf, "acm,usbnet,adb",
						sizeof(set_buf));
				device_desc.idProduct =
						ID_PRODUCT_STE_CONF_ACM_ETH_ADB;
				device_desc.bDeviceClass =
						USB_CLASS_VENDOR_SPEC;
				device_desc.bDeviceSubClass =
						USB_CLASS_VENDOR_SPEC;
				device_desc.bDeviceProtocol =
						USB_CLASS_VENDOR_SPEC;
			} else if (pc_command_adb == PC_COMMAND_ADB_ON) {
				strncpy(set_buf, "usbnet,adb", sizeof(set_buf));
				device_desc.idProduct = ID_PRODUCT_ETH_ADB;
			} else {
				strncpy(set_buf, "usbnet", sizeof(set_buf));
				device_desc.idProduct = ID_PRODUCT_ETH;
			}
                        functions_core_store(pdev, NULL, set_buf, sizeof(set_buf));
		} else if (andusb_plat->bp_tools_mode) {
                        functions_len = functions_core_show(pdev, NULL, get_buf);
			end_char_addr = &get_buf[0] + functions_len - 1;
			*end_char_addr = 0;
			if (strstr(get_buf, "adb") && strstr(get_buf, "rndis")) {
				strncpy(set_buf, "rndis,acm,usbnet,adb", sizeof(set_buf));
				device_desc.idProduct = ID_PRODUCT_RNDIS_ACM_ETH_ADB;
			} else if (strstr(get_buf, "adb")) {
				strncpy(set_buf, "acm,usbnet,adb", sizeof(set_buf));
				device_desc.idProduct = ID_PRODUCT_ACM_ETH_ADB;
			} else if (strstr(get_buf, "rndis")) {
				strncpy(set_buf, "rndis,acm,usbnet", sizeof(set_buf));
				device_desc.idProduct = ID_PRODUCT_RNDIS_ACM_ETH;
			} else {
				strncpy(set_buf, "acm,usbnet", sizeof(set_buf));
				device_desc.idProduct = ID_PRODUCT_ACM_ETH;
			}
                        functions_core_store(pdev, NULL, set_buf, sizeof(set_buf));
		}

		/* update values in composite driver's copy of device descriptor */
		cdev->desc.idVendor = device_desc.idVendor;

                functions_len = functions_core_show(pdev, NULL, get_buf);
		end_char_addr = &get_buf[0] + functions_len - 1;
		*end_char_addr = 0;
		if (dev->native_cdrom && !strcmp(get_buf, "mass_storage"))
			strncpy(get_buf, "cdrom", sizeof(get_buf));
		pid = android_usb_get_pid(get_buf);
		cdev->desc.idProduct = pid ? pid : device_desc.idProduct;

		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->enable)
				f->enable(f);
		}
		android_enable(dev);
		dev->enabled = true;
	} else if (!enabled && dev->enabled) {
		android_disable(dev);
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			if (f->disable)
				f->disable(f);
		}
		dev->enabled = false;
		/* This is for checking a case when we have reenumerated without
				removing USB cable */
		if (dev->connected) {
				dev->connected = 0;
				schedule_work(&dev->work);
		}
	} else {
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
	}
        mutex_unlock(&dev->mutex);
	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
        if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return sprintf(buf, "%s\n", state);
}

static ssize_t
pc_command_adb_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	if (cable_type != CPCAP_ACCY_FACTORY)
		return 0;

	if (!strcmp(buff, "adb"))
		pc_command_adb = PC_COMMAND_ADB_ON;
	else
		pc_command_adb = PC_COMMAND_ADB_OFF;

	return size;
}
static ssize_t
pc_command_modem_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	if (cable_type != CPCAP_ACCY_FACTORY)
		return 0;

	if (!strcmp(buff, "modem"))
		pc_command_modem = PC_COMMAND_MODEM_ON;
	else
		pc_command_modem = PC_COMMAND_MODEM_OFF;

	return size;
}


#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	int value;					       		\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, "%s", buffer);				\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)		       		\
{									\
	if (size >= sizeof(buffer)) return -EINVAL;			\
	if (sscanf(buf, "%s", buffer) == 1) {			       	\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);


DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)
DESCRIPTOR_STRING_ATTR(cdrom_blkdev, cdrom_blkdev_path)

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show, functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
static DEVICE_ATTR(pc_command_adb, S_IWUSR, NULL, pc_command_adb_store);
static DEVICE_ATTR(pc_command_modem, S_IWUSR, NULL, pc_command_modem_store);
static DEVICE_ATTR(smversion, S_IRUGO|S_IWUSR, smversion_show, smversion_store);

static struct device_attribute *android_usb_attributes[] = {
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
	&dev_attr_cdrom_blkdev,
	&dev_attr_smversion,
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_state,
	&dev_attr_pc_command_adb,
	&dev_attr_pc_command_modem,
	NULL
};

/*-------------------------------------------------------------------------*/
/* Composite driver */

static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	android_unbind_enabled_functions(dev, c);
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;

	usb_gadget_disconnect(gadget);

	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;

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

	/* Default strings - should be updated by userspace */
	strncpy(manufacturer_string, andusb_plat->vendor,
				sizeof(manufacturer_string) - 1);
	strncpy(product_string, andusb_plat->product_name,
				sizeof(product_string) - 1);
	strncpy(serial_string, andusb_plat->device_serial,
				MAX_USB_SERIAL_NUM - 1);
	strncpy(cdrom_blkdev_path, "/dev/block/cdrom", sizeof(cdrom_blkdev_path));

	/* init CDROM state */
	dev->current_function_type = CDROM;
	dev->cdrom_enable = 0;
	dev->cdrom_mount = 0;
	dev->native_cdrom = 0;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

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

	/*
	 * As per USB compliance update, a device that is actively drawing
	 * more than 100mA from USB must report itself as bus-powered in
	 * the GetStatus(DEVICE) call.
	 */
	if (CONFIG_USB_GADGET_VBUS_DRAW <= USB_SELF_POWER_VBUS_MAX_DRAW)
		usb_gadget_set_selfpowered(gadget);

	dev->cdev = cdev;

	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;

	cancel_work_sync(&dev->work);
	cancel_work_sync(&dev->enumeration_work);
	android_cleanup_functions(dev->functions);
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= android_usb_unbind,
};

void print_current_functions_name(void)
{
	struct android_dev              *dev = _android_dev;
	char buf[256], *b;
	struct android_usb_function     *f;

	b = buf;
	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		strncpy(b, f->name, strlen(f->name));
		b = b + strlen(f->name);
		*b++ = ',';
	}
	*(b - 1) = '\0';
	pr_info("config functions: %s\n", buf);
	return;
}

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct android_dev		*dev = _android_dev;
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
	int value = -EOPNOTSUPP;
	unsigned long flags;
	u16 wIndex = le16_to_cpu(c->wIndex);
	u16 wValue = le16_to_cpu(c->wValue);
	u16 wLength = le16_to_cpu(c->wLength);

	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

	switch (c->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_VENDOR:
		switch (c->bRequest) {
		case 1:
			if ((wValue == 0) && (wLength == 0)) {
				value = 0;
				req->zero = 0;
				req->length = value;
				if (usb_ep_queue
				    (cdev->gadget->ep0, req, GFP_ATOMIC))
					printk(KERN_ERR
					       "ep0 in queue failed\n");
				pr_info("receive pc switch setup package, index: 0x%x\n",
					wIndex);
				update_function_type_and_reenumerate(wIndex);
			}
			break;
		default:
			break;
		}
	default:
		break;
	}

	/*if request processed ,return */
	if (value >= 0)
		return value;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}

	/* Special case the accessory function.
	 * It needs to handle control requests before it is enabled.
	 */
	if (value < 0)
		value = acc_ctrlrequest(cdev, c);

	if (value < 0)
		value = composite_setup(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		schedule_work(&dev->work);
	}
	else if (c->bRequest == USB_REQ_SET_CONFIGURATION && cdev->config) {
		schedule_work(&dev->work);
		print_current_functions_name();
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void android_disconnect(struct usb_gadget *gadget)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	unsigned long flags;

	composite_disconnect(gadget);
	/* accessory HID support can be active while the
	   accessory function is not actually enabled,
	   so we need to inform it when we are disconnected.
	 */
	acc_disconnect();

	spin_lock_irqsave(&cdev->lock, flags);
	/* reset cdrom state for next connection */
	if (cable_type == CPCAP_ACCY_NONE &&
	    dev->current_function_type != CDROM &&
	    dev->cdrom_enable) {
		update_function_type_and_reenumerate(CDROM_INDEX);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);
}

static int android_create_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, "android0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}


static int __init android_gadget_probe(struct platform_device *pdev)
{
	andusb_plat = pdev->dev.platform_data;
	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = {.name = "android_gadget",},
	.probe = android_gadget_probe,
};

static int __init init(void)
{
	struct android_dev *dev;
	int err;
	int ret;

	pc_command_adb = 0;
	pc_command_modem = 0;
	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->disable_depth = 1;
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);
        mutex_init(&dev->mutex);
	INIT_WORK(&dev->enumeration_work, android_enumeration_work);

	err = android_create_device(dev);
	if (err) {
		class_destroy(android_class);
		kfree(dev);
		return err;
	}

	_android_dev = dev;

	ret = platform_driver_register(&android_platform_driver);
	if (ret) {
		class_destroy(android_class);
		kfree(_android_dev);
		_android_dev = NULL;
	}
	/* Override composite driver functions */
	composite_driver.setup = android_setup;
	composite_driver.disconnect = android_disconnect;

	return usb_composite_probe(&android_usb_driver, android_bind);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	class_destroy(android_class);
	kfree(_android_dev);
	_android_dev = NULL;
	platform_driver_unregister(&android_platform_driver);
}
module_exit(cleanup);
