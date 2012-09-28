/*
 * drivers/video/omap2/displays/omap-panel.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/pagemap.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <plat/panel.h>

#include <linux/omap-panel.h>

/*#define DEBUG*/
#ifdef DEBUG
#define DBG(format, ...) \
	printk(KERN_DEBUG "OMAP-PANEL: " format, ## __VA_ARGS__)
#else
#define DBG(format, ...)
#endif

#define DEVICE_NAME  "omap-panel"

#define OMAP_PANEL_MAX_PANELS		(5)
#define OMAP_PANEL_HANDLE_MAGIC		(0x55930000)

#define OMAP_PANEL_IS_VALID_HANDLE(h) \
	((OMAP_PANEL_HANDLE_MAGIC == \
		(((unsigned long)h) & 0xFFFF0000)) ? 1 : 0)
#define OMAP_PANEL_GET_IDX(h) \
	(((unsigned long)h) & 0x0000FFFF)

struct omap_panel_data {
	int active;

	struct omap_panel_device dev;

	/* Fields for Freeze-On-Display */
	int fod_en;	/* Freeze-On-Display state */
	int panel_en;	/* Panel hardware state */
	int dss_en;	/* Last DSS state request */
};

struct omap_panel_device_data {
	struct mutex  mtx; /* Lock for all device accesses */

	int major;
	struct class *cls;
	struct device *dev;

	int cnt;
	struct omap_panel_data data[OMAP_PANEL_MAX_PANELS];
};

static struct omap_panel_device_data *g_dev;

/*=== Local Functions ==================================================*/

static int omap_panel_get_handle_index(void *handle)
{
	int idx;

	if (g_dev == NULL) {
		printk(KERN_ERR "omap_get_handle_index - No Device!\n");
		return -ENODEV;
	}

	if (!OMAP_PANEL_IS_VALID_HANDLE(handle)) {
		printk(KERN_ERR "omap_get_handle_index - Invalid Handle!\n");
		return -EINVAL;
	}

	idx = OMAP_PANEL_GET_IDX(handle);

	if (idx >= OMAP_PANEL_MAX_PANELS) {
		printk(KERN_ERR "omap_get_handle_index - Invalid Index!\n");
		return -EINVAL;
	}

	return idx;
}

static int omap_panel_get_name_index(char *name)
{
	int idx = -EINVAL;
	int i;

	if (g_dev == NULL) {
		printk(KERN_ERR "omap_get_name_index - No Device!\n");
		return -ENODEV;
	}

	for (i = 0; i < g_dev->cnt; i++) {
		if (strcmp(g_dev->data[i].dev.name, name) == 0) {
			idx = i;
			break;
		}
	}

	return idx;
}

static int omap_panel_query_panel(struct omap_panel_info *arg)
{
	struct omap_panel_info info;
	int rc;
	int idx;

	rc = copy_from_user(&info, arg, sizeof(struct omap_panel_info));
	if (rc != 0) {
		printk(KERN_ERR "QUERYPANEL copy from user failed\n");
		return rc;
	}

	idx = info.idx;
	if (idx >= g_dev->cnt)
		return -EINVAL;

	strcpy(info.name, g_dev->data[idx].dev.name);
	info.active = g_dev->data[idx].active;

	return copy_to_user(arg, &info, sizeof(info));
}

static int omap_panel_get_fod(struct omap_panel_fod *arg)
{
	struct omap_panel_fod fod;
	int rc;
	int idx;

	rc = copy_from_user(&fod, arg, sizeof(struct omap_panel_fod));
	if (rc != 0) {
		printk(KERN_ERR "G_FOD copy from user failed\n");
		return rc;
	}

	idx = omap_panel_get_name_index(fod.name);
	if (idx < 0)
		return idx; /* The idx is the error code */

	fod.enable = g_dev->data[idx].active;

	return copy_to_user(arg, &fod, sizeof(fod));
}

static int omap_panel_set_fod(struct omap_panel_fod *arg)
{
	struct omap_panel_fod fod;
	struct omap_panel_data *entry;
	int rc = 0;
	int idx;
	int en;

	rc = copy_from_user(&fod, arg, sizeof(struct omap_panel_fod));
	if (rc != 0) {
		printk(KERN_ERR "S_FOD copy from user failed\n");
		return rc;
	}

	idx = omap_panel_get_name_index(fod.name);
	if (idx < 0)
		return idx; /* The idx is the error code */

	en = (fod.enable) ? 1 : 0;

	entry = &g_dev->data[idx];
	if (en != entry->fod_en) {
		entry->fod_en = en;
		if (!en && !entry->dss_en && entry->panel_en &&
						entry->dev.fod_disable) {
			dsi_bus_lock();
			entry->panel_en = 0;
			entry->dev.fod_disable(entry->dev.dssdev);
			dsi_bus_unlock();
		}
	}

	return 0;
}

/*=== Public Functions =================================================*/

void *omap_panel_register(struct omap_panel_device *dev)
{
	void *handle = NULL;

	if (g_dev == NULL) {
		printk(KERN_ERR "omap_panel_register - No Device!\n");
		return NULL;
	}

	mutex_lock(&g_dev->mtx);

	if (g_dev->cnt >= OMAP_PANEL_MAX_PANELS) {
		printk(KERN_ERR "omap_panel_register - No Resource!\n");
		return NULL;
	}

	g_dev->data[g_dev->cnt].active = 1;
	g_dev->data[g_dev->cnt].dev = *dev;
	g_dev->data[g_dev->cnt].fod_en = 0;
	g_dev->data[g_dev->cnt].panel_en = 0;
	g_dev->data[g_dev->cnt].dss_en = 0;

	handle = (void *)(OMAP_PANEL_HANDLE_MAGIC + g_dev->cnt);

	g_dev->cnt++;

	mutex_unlock(&g_dev->mtx);

	return handle;
}

void omap_panel_unregister(void *handle)
{
	int idx;

	idx = omap_panel_get_handle_index(handle);
	if (idx < 0)
		return;

	mutex_lock(&g_dev->mtx);

	g_dev->data[idx].active = 0;

	mutex_unlock(&g_dev->mtx);
}

int omap_panel_fod_dss_state(void *handle, int en)
{
	int idx;

	idx = omap_panel_get_handle_index(handle);
	if (idx < 0)
		return idx; /* The idx is the error code */

	mutex_lock(&g_dev->mtx);

	g_dev->data[idx].dss_en = (en) ? 1 : 0;

	mutex_unlock(&g_dev->mtx);

	return 0;
}

int omap_panel_fod_panel_state(void *handle, int en)
{
	int idx;

	idx = omap_panel_get_handle_index(handle);
	if (idx < 0)
		return idx; /* The idx is the error code */

	mutex_lock(&g_dev->mtx);

	g_dev->data[idx].panel_en = (en) ? 1 : 0;

	mutex_unlock(&g_dev->mtx);

	return 0;
}

int omap_panel_fod_enabled(void *handle)
{
	int enabled = 0;
	int idx;

	idx = omap_panel_get_handle_index(handle);
	if (idx >= 0) {
		mutex_lock(&g_dev->mtx);
		enabled = g_dev->data[idx].fod_en;
		mutex_unlock(&g_dev->mtx);
	}

	return enabled;
}

/*=== Driver Interface Functions =======================================*/

static int omap_panel_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	DBG("omap_panel_open\n");

	if (g_dev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	return rc;
}

static int omap_panel_release(struct inode *inode, struct file *file)
{
	int rc = 0;

	DBG("omap_panel_release\n");

	if (g_dev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	return rc;
}

static int omap_panel_ioctl(struct inode *inode, struct file *file,
							u_int cmd, u_long arg)
{
	int rc = 0;

	if (unlikely(_IOC_TYPE(cmd) != OMAP_PANEL_IOCTL_MAGIC)) {
		printk(KERN_ERR "Bad command value (%d)\n", cmd);
		return -EINVAL;
	}

	mutex_lock(&g_dev->mtx);

	switch (cmd) {
	case OMAP_PANEL_QUERYPANEL:
		rc = omap_panel_query_panel((struct omap_panel_info *) arg);
		break;
	case OMAP_PANEL_G_FOD:
		rc = omap_panel_get_fod((struct omap_panel_fod *) arg);
		break;
	case OMAP_PANEL_S_FOD:
		rc = omap_panel_set_fod((struct omap_panel_fod *) arg);
		break;
	default:
		printk(KERN_ERR "Invalid ioctl (%x)\n", cmd);
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&g_dev->mtx);

	return rc;
}

static const struct file_operations omap_panel_fops = {
	.owner		= THIS_MODULE,
	.open		= omap_panel_open,
	.release	= omap_panel_release,
	.ioctl		= omap_panel_ioctl,
};

static struct miscdevice omap_panel_misc_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DEVICE_NAME,
	.fops	= &omap_panel_fops,
};

static int __init omap_panel_probe(struct platform_device *pdev)
{
	int rc = 0;

	DBG("omap_panel_probe\n");

	g_dev = kzalloc(sizeof(struct omap_panel_device_data), GFP_KERNEL);
	if (g_dev == NULL)
		return -ENOMEM;

	mutex_init(&g_dev->mtx);

	rc = misc_register(&omap_panel_misc_device);
	if (rc) {
		printk(KERN_ERR "misc register failed (%d)\n", rc);
		goto failed;
	}

	return 0;

failed:
	kfree(g_dev);
	g_dev = NULL;
	return rc;
}

static int omap_panel_remove(struct platform_device *pdev)
{
	struct omap_panel_device_data *data = platform_get_drvdata(pdev);

	DBG("omap_panel_remove\n");

	misc_deregister(&omap_panel_misc_device);
	kfree(data);

	return 0;
}

static struct platform_driver omap_panel_driver = {
	.remove         = omap_panel_remove,
	.driver         = {
		.name   = DEVICE_NAME,
	},
};

/*=== Init/Exit Interface Functions =======================================*/

static int __init omap_panel_init(void)
{
	int rc = 0;

	DBG("omap_panel_init\n");

	rc = platform_driver_probe(&omap_panel_driver, omap_panel_probe);
	if (rc != 0) {
		printk(KERN_ERR "failed panel register/probe %d\n", rc);
		return -ENODEV;
	}

	return 0;
}

static void __exit omap_panel_exit(void)
{
	DBG("omap_panel_exit\n");

	platform_driver_unregister(&omap_panel_driver);
}

module_init(omap_panel_init);
module_exit(omap_panel_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("OMAP Panel Driver");
MODULE_LICENSE("GPL");

