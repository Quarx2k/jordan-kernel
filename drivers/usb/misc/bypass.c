/*
 * bypass.c -- USB Driver to bypass data from USB HOST to OTG ETS
 *
 * Copyright (C) 2009 VIA TELECOM
 *
 * Author: shaoneng wang <snwang@via-telecom.com>
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
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/string.h>

#include <linux/device.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <linux/tty.h>
#include <linux/usb/serial.h>

#include <linux/bypass.h>

/* this variable will across the bypass progress */
struct bypass *bypass;

static ssize_t ets_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	int ret_val = 0;

	if (bypass->ets_status == 0)
		ret_val += sprintf(buf + ret_val, "tty\n");
	if (bypass->ets_status == 1)
		ret_val += sprintf(buf + ret_val, "gadget\n");
	return ret_val;

}

static ssize_t modem_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	int ret_val = 0;

	if (bypass->modem_status == 0)
		ret_val += sprintf(buf + ret_val, "tty\n");
	if (bypass->modem_status == 1)
		ret_val += sprintf(buf + ret_val, "gadget\n");
	return ret_val;

}

static ssize_t at_show(struct kobject *kobj, struct kobj_attribute *attr,
		       char *buf)
{
	int ret_val = 0;

	if (bypass->at_status == 0)
		ret_val += sprintf(buf + ret_val, "tty\n");
	if (bypass->at_status == 1)
		ret_val += sprintf(buf + ret_val, "gadget\n");
	return ret_val;
}

static ssize_t gps_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	int ret_val = 0;

	if (bypass->gps_status == 0)
		ret_val += sprintf(buf + ret_val, "tty\n");
	if (bypass->gps_status == 1)
		ret_val += sprintf(buf + ret_val, "gadget\n");
	return ret_val;
}

static ssize_t pcv_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf)
{
	int ret_val = 0;

	if (bypass->pcv_status == 0)
		ret_val += sprintf(buf + ret_val, "tty\n");
	if (bypass->pcv_status == 1)
		ret_val += sprintf(buf + ret_val, "gadget\n");
	return ret_val;
}

static ssize_t ets_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t n)
{
	if (!strncmp(buf, "tty", strlen("tty"))) {
		if (!bypass->ets_status)	/* for avoid reclose again */
			return n;
		bypass->ets_status = 0;
		usb_serial_port_close(bypass->h_ets_port);
	}
	if (!strncmp(buf, "gadget", strlen("gadget"))) {
		if (bypass->h_ets_port) {
			if (bypass->ets_status)	/* for avoid reopen again */
				return n;
			bypass->ets_status = 1;
			if (!bypass->h_ets_port->port.count)
				usb_serial_port_open(bypass->h_ets_port);
			if (bypass->ops->bp_connect)
				bypass->ops->bp_connect(1);
		} else
			printk(KERN_INFO
			       "bypass->h_ets_port is NULL, do nothing\n");
	}
	return n;

}

static ssize_t modem_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	if (!strncmp(buf, "tty", strlen("tty"))) {
		if (!bypass->modem_status)
			return n;
		bypass->modem_status = 0;
		usb_serial_port_close(bypass->h_modem_port);
		if (bypass->ops->acm_disconnect)
			bypass->ops->acm_disconnect();
		else
			printk(KERN_INFO
			       "do nothing....acm_connect not register\n");
	}
	if (!strncmp(buf, "gadget", strlen("gadget"))) {
		if (bypass->modem_status)
			return n;
		bypass->modem_status = 1;
		usb_serial_port_open(bypass->h_modem_port);
		if (bypass->ops->acm_connect)
			bypass->ops->acm_connect();
		else
			printk(KERN_INFO
			       "do nothing.....acm_disconnect not register \n");
	}
	return n;

}

static ssize_t at_store(struct kobject *kobj, struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	if (!strncmp(buf, "tty", strlen("tty"))) {
		if (!bypass->at_status)
			return n;
		usb_serial_port_close(bypass->h_atc_port);
		bypass->at_status = 0;
	}
	if (!strncmp(buf, "gadget", strlen("gadget"))) {
		if (bypass->h_atc_port) {
			if (bypass->at_status)
				return n;
			bypass->at_status = 1;
			usb_serial_port_open(bypass->h_atc_port);
			if (bypass->ops->bp_connect)
				bypass->ops->bp_connect(2);
		} else
			printk(KERN_INFO
			       "bypass->h_atc_port is NULL, do nothing\n");
	}
	return n;
}

static ssize_t gps_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t n)
{
	if (!strncmp(buf, "tty", strlen("tty"))) {
		if (!bypass->gps_status)
			return n;
		usb_serial_port_close(bypass->h_gps_port);
		bypass->gps_status = 0;
	}
	if (!strncmp(buf, "gadget", strlen("gadget"))) {
		if (bypass->h_gps_port) {
			if (bypass->gps_status)
				return n;
			bypass->gps_status = 1;
			usb_serial_port_open(bypass->h_gps_port);
			if (bypass->ops->bp_connect)
				bypass->ops->bp_connect(4);
		} else
			printk(KERN_INFO
			       "bypass->h_gps_port is NULL, do nothing\n");
	}
	return n;
}

static ssize_t pcv_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t n)
{
	if (!strncmp(buf, "tty", strlen("tty"))) {
		if (!bypass->pcv_status)
			return n;
		usb_serial_port_close(bypass->h_pcv_port);
		bypass->pcv_status = 0;
	}
	if (!strncmp(buf, "gadget", strlen("gadget"))) {
		if (bypass->h_pcv_port) {
			if (bypass->pcv_status)
				return n;
			bypass->pcv_status = 1;
			usb_serial_port_open(bypass->h_pcv_port);
			if (bypass->ops->bp_connect)
				bypass->ops->bp_connect(3);
		} else
			printk(KERN_INFO
			       "bypass->h_pcv_port is NULL, do nothing\n");

	}
	return n;
}

static struct kobj_attribute ets_attr = {
	.attr = {
		 .name = __stringify(ets),
		 .mode = 0644,
		 },
	.show = ets_show,
	.store = ets_store,
};

static struct kobj_attribute modem_attr = {
	.attr = {
		 .name = __stringify(modem),
		 .mode = 0644,
		 },
	.show = modem_show,
	.store = modem_store,
};

static struct kobj_attribute at_attr = {
	.attr = {
		 .name = __stringify(at),
		 .mode = 0644,
		 },
	.show = at_show,
	.store = at_store,
};

static struct kobj_attribute gps_attr = {
	.attr = {
		 .name = __stringify(gps),
		 .mode = 0644,
		 },
	.show = gps_show,
	.store = gps_store,
};

static struct kobj_attribute pcv_attr = {
	.attr = {
		 .name = __stringify(pcv),
		 .mode = 0644,
		 },
	.show = pcv_show,
	.store = pcv_store,
};

static struct attribute *g[] = {
	&ets_attr.attr,
	&at_attr.attr,
	&gps_attr.attr,
	&pcv_attr.attr,
	&modem_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

struct kobject *bypass_obj;

static int bypass_sysfs_init(void)
{
	bypass_obj = kobject_create_and_add("usb_bypass", NULL);
	if (!bypass_obj)
		return -ENOMEM;
	return sysfs_create_group(bypass_obj, &attr_group);
}

int bypass_register(struct bypass_ops *ops)
{

	printk(KERN_INFO "enter bypass_register\n");

	if (!(bypass && bypass->ops)) {
		printk(KERN_INFO "bypass or bypass->ops is NULL\n");
		return -1;
	}

	if (ops->h_write)
		bypass->ops->h_write = ops->h_write;
	if (ops->g_write)
		bypass->ops->g_write = ops->g_write;
	if (ops->g_modem_write)
		bypass->ops->g_modem_write = ops->g_modem_write;
	if (ops->acm_connect)
		bypass->ops->acm_connect = ops->acm_connect;
	if (ops->acm_disconnect)
		bypass->ops->acm_disconnect = ops->acm_disconnect;
	if (ops->bp_connect)
		bypass->ops->bp_connect = ops->bp_connect;
	return 0;
}
EXPORT_SYMBOL(bypass_register);

void bypass_unregister(int type)
{

	printk(KERN_INFO "enter bypass_unregister\n");
	switch (type) {
	case 1:
		bypass->ops->h_write = NULL;
		break;
	case 2:
		bypass->ops->g_write = NULL;
		bypass->ops->bp_connect = NULL;
		break;
	case 3:
		bypass->ops->g_modem_write = NULL;
		bypass->ops->acm_connect = NULL;
		bypass->ops->acm_disconnect = NULL;
		break;
	default:
		printk(KERN_INFO "bypass_unregister type error\n");
		break;
	};
}
EXPORT_SYMBOL(bypass_unregister);

struct bypass *bypass_get(void)
{

	if (!(bypass && bypass->ops)) {
		printk(KERN_INFO "--error, bypass or bypass->ops is NULL\n");
		return NULL;
	} else
		return bypass;
}
EXPORT_SYMBOL(bypass_get);

static int __init bypass_init(void)
{
	printk(KERN_INFO "enter bypass_init\n");
	bypass = kzalloc(sizeof(struct bypass), GFP_KERNEL);
	if (bypass == NULL)
		return -ENOMEM;
	bypass->ops = kzalloc(sizeof(struct bypass_ops), GFP_KERNEL);
	if (bypass->ops == NULL)
		return -ENOMEM;
	spin_lock_init(&bypass->lock);
	bypass_sysfs_init();
	return 0;
}

subsys_initcall(bypass_init);
