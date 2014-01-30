/*
 * Bluetooth TI wl18xx rfkill power control via GPIO
 *
 * Copyright (C) 2014 Motorola, Inc.
 * Copyright (C) 2008 Texas Instruments
 * Initial code: Pavan Savoy <pavan.savoy@gmail.com> (wl18xx_power.c)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/rfkill-wl18xx.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

enum wl18xx_devices {
	WL18XX_BLUETOOTH = 0,
	WL18XX_FM,
	WL18XX_MAX_DEV,
};

struct wl18xx_rfkill_driver_data {
	struct wl18xx_rfkill_platform_data *pdata;
	struct rfkill *rfkill[WL18XX_MAX_DEV];
};

#define WL18XX_BT_SUPPORTED(x) ((x)->bt_enable_gpio >= 0)
#define WL18XX_FM_SUPPORTED(x) ((x)->fm_enable_gpio >= 0)

static void wl18xx_rfkill_destroy(struct wl18xx_rfkill_driver_data *p_drvdata);

static int wl18xx_bt_rfkill_set_power(void *data, bool blocked)
{
	struct wl18xx_rfkill_platform_data *pdata =
		(struct wl18xx_rfkill_platform_data *)data;

	if (blocked)
		gpio_set_value(pdata->bt_enable_gpio, 0);
	else
		gpio_set_value(pdata->bt_enable_gpio, 1);

	return 0;
}

static int wl18xx_fm_rfkill_set_power(void *data, bool blocked)
{
	struct wl18xx_rfkill_platform_data *pdata =
		(struct wl18xx_rfkill_platform_data *)data;

	if (blocked)
		gpio_set_value(pdata->fm_enable_gpio, 0);
	else
		gpio_set_value(pdata->fm_enable_gpio, 1);

	return 0;
}

static const struct rfkill_ops wl18xx_bt_rfkill_ops = {
	.set_block = wl18xx_bt_rfkill_set_power,
};

static const struct rfkill_ops wl18xx_fm_rfkill_ops = {
	.set_block = wl18xx_fm_rfkill_set_power,
};

/**
Added to reset the BT chip after Ram download
*/
static ssize_t reset_wl18xx_chip(struct device *dev,
					struct device_attribute
					*attr, const char *buf, size_t size)
{
	struct wl18xx_rfkill_platform_data *pd = dev->platform_data;

	BUG_ON(!pd);
	if (WL18XX_BT_SUPPORTED(pd)) {
		gpio_set_value(pd->bt_enable_gpio, 0);
		msleep(5);
		gpio_set_value(pd->bt_enable_gpio, 1);
	}
	return size;
}
static DEVICE_ATTR(reset_vio, 0200, NULL, reset_wl18xx_chip);

#ifdef CONFIG_OF
static int wl18xx_rfkill_of_init(
				struct wl18xx_rfkill_platform_data *pdata,
				struct device_node *dt_node)
{
	int rc;
	int bt_fm_support = 0;
	int gpio;

	/* Set default values */
	pdata->bt_enable_gpio = -1;
	pdata->fm_enable_gpio = -1;

	/* Set default values */
	rc = of_property_read_u32(dt_node, "mot,bt_fm", &bt_fm_support);
	if (!rc) {
		if (bt_fm_support & 0x01) {
			gpio = of_get_named_gpio_flags(dt_node,
					"mot,bt_en-gpio", 0, NULL);
			pdata->bt_enable_gpio = (gpio < 0) ? -1 : gpio;
		} else {
			pr_err("wl18xx_rfkill: BT support disabled");
		}

		if (bt_fm_support & 0x02) {
			gpio = of_get_named_gpio_flags(dt_node,
					"mot,fm_en-gpio", 0, NULL);
			pdata->fm_enable_gpio = (gpio < 0) ? -1 : gpio;
		} else {
			pr_err("wl18xx_rfkill: FM support disabled");
		}
	} else {
		pr_err("%s: failed to read device tree", __func__);
	}

	return rc;
}
#endif

static int wl18xx_rfkill_gpio_init(struct wl18xx_rfkill_platform_data *pdata)
{
	int rc = 0;
	struct gpio wl18xx_rfkill_bt_gpio[] = {
		{pdata->bt_enable_gpio, GPIOF_OUT_INIT_LOW, "wl18xx_bt_en"},
	};
	struct gpio wl18xx_rfkill_fm_gpio[] = {
		{pdata->fm_enable_gpio, GPIOF_OUT_INIT_LOW, "wl18xx_fm_en"},
	};

	if (WL18XX_BT_SUPPORTED(pdata)) {
		rc = gpio_request_array(wl18xx_rfkill_bt_gpio,
					ARRAY_SIZE(wl18xx_rfkill_bt_gpio));
		if (rc)
			pr_err("%s: Failed to request BT GPIOs\n", __func__);
	}

	if (WL18XX_FM_SUPPORTED(pdata)) {
		rc = gpio_request_array(wl18xx_rfkill_fm_gpio,
					ARRAY_SIZE(wl18xx_rfkill_fm_gpio));
		if (rc)
			pr_err("%s: Failed to request FM GPIOs\n", __func__);
	}

	return rc;
}

static int wl18xx_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct wl18xx_rfkill_driver_data *dd = NULL;
	struct wl18xx_rfkill_platform_data *pd = NULL;

	pr_info("%s\n", __func__);

	dd = kzalloc(sizeof(struct wl18xx_rfkill_driver_data), GFP_KERNEL);
	if (dd == NULL) {
		pr_err("%s: memory allocation failure\n", __func__);
		rc = -ENOMEM;
		goto wl18xx_probe_fail;
	}

	/* set platform data */
	if (pdev->dev.of_node) {
		pd = devm_kzalloc(&pdev->dev,
				sizeof(struct wl18xx_rfkill_platform_data),
				GFP_KERNEL);
		if (!pd) {
			pr_err("%s: memory allocation failure\n", __func__);
			rc = -ENOMEM;
			goto wl18xx_probe_fail;
		}

		rc = wl18xx_rfkill_of_init(pd, pdev->dev.of_node);
		if (rc)
			goto wl18xx_probe_fail;
		pdev->dev.platform_data = pd;
	} else {
		pd = pdev->dev.platform_data;
	}
	dd->pdata = pd;

	/* set driver data */
	dev_set_drvdata(&pdev->dev, dd);

	/* verify pdata */
	if (!WL18XX_BT_SUPPORTED(pd) && WL18XX_FM_SUPPORTED(pd)) {
		pr_err("%s: invalid configuration\n", __func__);
		rc = -EINVAL;
		goto wl18xx_probe_fail;
	}

	/* requesting gpios */
	rc = wl18xx_rfkill_gpio_init(pd);
	if (rc)
		goto gpio_err;

	if (WL18XX_BT_SUPPORTED(pd)) {
		/* init rfkill for BT */
		dd->rfkill[WL18XX_BLUETOOTH] = rfkill_alloc(
					"wl18xx Bluetooth",
					&pdev->dev,
					RFKILL_TYPE_BLUETOOTH,
					&wl18xx_bt_rfkill_ops,
					(void *)pd);
		if (unlikely(!dd->rfkill[WL18XX_BLUETOOTH])) {
			pr_err("%s: memory allocation failure\n", __func__);
			rc = -ENOMEM;
			goto err_rfkill_register;
		}

		rfkill_set_states(dd->rfkill[WL18XX_BLUETOOTH],
				  true, false);
		rc = rfkill_register(dd->rfkill[WL18XX_BLUETOOTH]);
		if (unlikely(rc)) {
			pr_err("%s: failed to register BT rfkill\n", __func__);
			goto err_rfkill_register;
		}
	}

	if (WL18XX_FM_SUPPORTED(pd)) {
		/* init rfkill for FM */
		dd->rfkill[WL18XX_FM] = rfkill_alloc(
					"wl18xx FM Radio", &pdev->dev,
					RFKILL_TYPE_FM, &wl18xx_fm_rfkill_ops,
					(void *)pd);
		if (unlikely(!dd->rfkill[WL18XX_FM])) {
			pr_err("%s: memory allocation failure\n", __func__);
			rc = -ENOMEM;
			goto err_rfkill_register;
		}

		rfkill_set_states(dd->rfkill[WL18XX_FM],
				  true, false);
		rc = rfkill_register(dd->rfkill[WL18XX_FM]);
		if (unlikely(rc)) {
			pr_err("%s: failed to register BT rfkill\n", __func__);
			goto err_rfkill_register;
		}
	}

	/* Create device file to expose interface to user space to
	reset the vio of BT, this should be done independent of whether
	BT/FM is initialised as both run on the same w18xx chip
	*/
	rc = device_create_file(&pdev->dev, &dev_attr_reset_vio);
	if (rc) {
		pr_err("%s: failed to create sys entry\n", __func__);
		goto err_rfkill_register;
	}

	goto done;

	/* Clean up for BT generic registration */
err_rfkill_register:
	wl18xx_rfkill_destroy(dd);

gpio_err:
	if (pd->bt_enable_gpio > 0)
		gpio_free(pd->bt_enable_gpio);
	if (pd->fm_enable_gpio > 0)
		gpio_free(pd->fm_enable_gpio);

wl18xx_probe_fail:
	kfree(dd);

	pr_err("%s: failed with error = %d\n", __func__, rc);

done:
	return rc;
}

static void wl18xx_rfkill_destroy(struct wl18xx_rfkill_driver_data *p_drvdata)
{
	BUG_ON(!p_drvdata);

	if (p_drvdata->rfkill[WL18XX_BLUETOOTH]) {
		rfkill_unregister(p_drvdata->rfkill[WL18XX_BLUETOOTH]);
		rfkill_destroy(p_drvdata->rfkill[WL18XX_BLUETOOTH]);
	}

	if (p_drvdata->rfkill[WL18XX_FM]) {
		rfkill_unregister(p_drvdata->rfkill[WL18XX_FM]);
		rfkill_destroy(p_drvdata->rfkill[WL18XX_FM]);
	}
}

static int wl18xx_rfkill_remove(struct platform_device *pdev)
{
	struct wl18xx_rfkill_driver_data *dd;
	struct wl18xx_rfkill_platform_data *pd;

	dd =  dev_get_drvdata(&pdev->dev);
	pd = pdev->dev.platform_data;

	/* remove the sys fs file created as part of probe*/
	device_remove_file(&pdev->dev, &dev_attr_reset_vio);

	if (pd->bt_enable_gpio > 0)
		gpio_free(pd->bt_enable_gpio);
	if (pd->fm_enable_gpio > 0)
		gpio_free(pd->fm_enable_gpio);
	kfree(pd);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id mot_wl18xx_rfkill_table[] = {
	{ .compatible = "mot,wl18xx-rfkill",},
	{ },
};
#else
#define mot_wl18xx_rfkill_table NULL
#endif

static struct platform_driver wl18xx_rfkill_platform_driver = {
	.probe = wl18xx_rfkill_probe,
	.remove = wl18xx_rfkill_remove,
	.driver = {
		   .name = "wl18xx-rfkill",
		   .owner = THIS_MODULE,
		   .of_match_table = mot_wl18xx_rfkill_table,
		   },
};

static int __init wl18xx_rfkill_init(void)
{
	return platform_driver_register(&wl18xx_rfkill_platform_driver);
}

static void __exit wl18xx_rfkill_exit(void)
{
	platform_driver_unregister(&wl18xx_rfkill_platform_driver);
}

module_init(wl18xx_rfkill_init);
module_exit(wl18xx_rfkill_exit);

MODULE_ALIAS("platform:wl18xx");
MODULE_DESCRIPTION("wl18xx-rfkill");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
