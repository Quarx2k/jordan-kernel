/*
 * Bluetooth TI wl127x rfkill power control via GPIO
 *
 * Copyright (C) 2009 Motorola, Inc.
 * Copyright (C) 2008 Texas Instruments
 * Initial code: Pavan Savoy <pavan.savoy@gmail.com> (wl127x_power.c)
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
#include <linux/wl127x-rfkill.h>
#include <linux/delay.h>

static int wl127x_bt_rfkill_set_power(void *data, bool blocked)
{
	struct wl127x_rfkill_platform_data *pdata =
		(struct wl127x_rfkill_platform_data *) data;
	int nshutdown_gpio = pdata->bt_nshutdown_gpio;

	if (blocked) {
		gpio_set_value(nshutdown_gpio, 0);
		if (pdata->bt_hw_disable)
			pdata->bt_hw_disable();
	} else {
		if (pdata->bt_hw_enable)
			pdata->bt_hw_enable();
		gpio_set_value(nshutdown_gpio, 1);
	}
	return 0;
}

static int wl127x_fm_rfkill_set_power(void *data, bool blocked)
{
	int nshutdown_gpio = (int) data;

	if (blocked)
		gpio_set_value(nshutdown_gpio, 0);
	else
		gpio_set_value(nshutdown_gpio, 1);

	return 0;
}

static const struct rfkill_ops wl127x_bt_rfkill_ops = {
	.set_block = wl127x_bt_rfkill_set_power,
};

static const struct rfkill_ops wl127x_fm_rfkill_ops = {
	.set_block = wl127x_fm_rfkill_set_power,
};



/**
Added to reset the BT chip after Ram download
*/
static ssize_t reset_wl18xx_chip(struct device *dev,
					   struct device_attribute
					   *attr, const char *buf, size_t size)
{

	int bt_enable_gpio;
	printk(KERN_DEBUG  "Calling reset_wl18xx_chip \n");

	/* TODO, rework once device tree is pulled in */
	/* bt_enable_gpio = get_gpio_by_name("bt_reset_b"); */
	bt_enable_gpio = 83;

	printk(KERN_DEBUG "bt_enable_gpio = %d\n", bt_enable_gpio);

	if (bt_enable_gpio < 0) {

		printk(KERN_DEBUG "reset_wl18xx_chip: cannot retrieve bt_reset_b gpio from device tree\n");
		bt_enable_gpio = -1;
		return -EINVAL;
	}

	gpio_set_value(bt_enable_gpio, 0);
	msleep(5);
	gpio_set_value(bt_enable_gpio, 1);
	printk(KERN_DEBUG " successfully set the value\n");

	return 0;

}

static DEVICE_ATTR(reset_vio, 0644, NULL, reset_wl18xx_chip);


static int wl127x_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool fm_deinit_required_flag = false;
	struct wl127x_rfkill_platform_data *pdata = pdev->dev.platform_data;

	if (pdata->bt_nshutdown_gpio >= 0) {
		bool default_blocked = true;  /* power off */
		printk(KERN_DEBUG "Entered the bt enable part \n");
		rc = gpio_request(pdata->bt_nshutdown_gpio,
				  "wl127x_bt_nshutdown_gpio");
		if (unlikely(rc))
			return rc;

		rc = gpio_direction_output(pdata->bt_nshutdown_gpio, 0);
		if (unlikely(rc)) {
			printk(KERN_ERR "wl127x_rfkill_probe failure could not set output direction for gpio bt_nshutdown_gpio \n");
			goto bt_err_gpio_direction;
		}

		if (pdata->bt_hw_init)
			rc = pdata->bt_hw_init();
		if (unlikely(rc)) {
			printk(KERN_ERR "wl127x_rfkill_probe failure could not init hardware \n");
			goto bt_err_gpio_direction;
		}


		wl127x_bt_rfkill_set_power((void *)pdata, default_blocked);

		pdata->rfkill[WL127X_BLUETOOTH] = rfkill_alloc(
				"wl127x Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &wl127x_bt_rfkill_ops,
				(void *)pdata);
		if (unlikely(!pdata->rfkill[WL127X_BLUETOOTH])) {
			printk(KERN_ERR "wl127x_rfkill_probe failure could not allocate memory \n");
			rc = -ENOMEM;
			goto bt_err_rfkill_alloc;
		}

		rfkill_set_states(pdata->rfkill[WL127X_BLUETOOTH],
				default_blocked, false);

		rc = rfkill_register(pdata->rfkill[WL127X_BLUETOOTH]);
		if (unlikely(rc)) {
			printk(KERN_ERR "wl127x_rfkill_probe failure could to register BT \n");
			goto bt_err_rfkill_register;
		}

	}

	if (pdata->fm_enable_gpio >= 0) {
		bool default_blocked = true;  /* power off */
		printk(KERN_DEBUG "Entered the fm enable part \n");
		rc = gpio_request(pdata->fm_enable_gpio,
				  "wl127x_fm_enable_gpio");
		if (unlikely(rc))
			return rc;

		rc = gpio_direction_output(pdata->fm_enable_gpio, 0);
		if (unlikely(rc)) {
			printk(KERN_ERR "wl127x_rfkill_probe failure could not set output direction for gpio fm_enable_gpio \n");
			goto fm_err_gpio_direction;
		}

		wl127x_fm_rfkill_set_power((void *)pdata->fm_enable_gpio,
				default_blocked);

		pdata->rfkill[WL127X_FM] = rfkill_alloc("wl127x FM Radio",
				&pdev->dev, RFKILL_TYPE_FM,
				&wl127x_fm_rfkill_ops,
				(void *)pdata->fm_enable_gpio);
		if (unlikely(!pdata->rfkill[WL127X_FM])) {
			printk(KERN_ERR "wl127x_rfkill_probe failure could not allocate memory for fm\n");
			rc = -ENOMEM;
			goto fm_err_gpio_direction;

		}

		rfkill_set_states(pdata->rfkill[WL127X_FM], default_blocked,
				false);

		rc = rfkill_register(pdata->rfkill[WL127X_FM]);
		if (unlikely(rc)) {
			printk(KERN_ERR "wl127x_rfkill_probe failure could to register FM \n");
			goto fm_err_rfkill_register;
		}

		fm_deinit_required_flag = true;
	}

	/* Create device file to expose interface to user space to
	reset the vio of BT, this should be done independent of whether
	BT/FM is initialised as both run on the same w18xx chip
	*/
	if (device_create_file(&pdev->dev, &dev_attr_reset_vio)) {
		printk(KERN_DEBUG  "Error creating sys entry for reset vio\n");
		rc = -1;
		goto bt_err_rfkill_register;
	}

	goto done;

	/* Clean up for BT generic registration */
bt_err_rfkill_register:
	rfkill_destroy(pdata->rfkill[WL127X_BLUETOOTH]);
bt_err_rfkill_alloc:
	if (pdata->bt_hw_release)
		pdata->bt_hw_release();
bt_err_gpio_direction:
	if (pdata->bt_nshutdown_gpio >= 0)
		gpio_free(pdata->bt_nshutdown_gpio);

	if (fm_deinit_required_flag == false)
		goto done;

	fm_deinit_required_flag = false;

	/* Clean up for FM generic registration
	do not clean up BT process as we still want to use BT
	*/
fm_err_rfkill_register:
	rfkill_destroy(pdata->rfkill[WL127X_FM]);
fm_err_gpio_direction:
	gpio_free(pdata->fm_enable_gpio);

done:
		return rc;
}

static int wl127x_rfkill_remove(struct platform_device *pdev)
{
	struct wl127x_rfkill_platform_data *pdata = pdev->dev.platform_data;

	if (pdata->bt_nshutdown_gpio >= 0) {
		rfkill_unregister(pdata->rfkill[WL127X_BLUETOOTH]);
		rfkill_destroy(pdata->rfkill[WL127X_BLUETOOTH]);
		if (pdata->bt_hw_release)
			pdata->bt_hw_release();
		gpio_free(pdata->bt_nshutdown_gpio);
	}


	if (pdata->fm_enable_gpio >= 0) {
		rfkill_unregister(pdata->rfkill[WL127X_FM]);
		rfkill_destroy(pdata->rfkill[WL127X_FM]);
		gpio_free(pdata->fm_enable_gpio);
	}

	/* remove the sys fs file created as part of probe*/
	device_remove_file(&pdev->dev, &dev_attr_reset_vio);

	return 0;
}

static struct platform_driver wl127x_rfkill_platform_driver = {
	.probe = wl127x_rfkill_probe,
	.remove = wl127x_rfkill_remove,
	.driver = {
		   .name = "wl127x-rfkill",
		   .owner = THIS_MODULE,
		   },
};

static int __init wl127x_rfkill_init(void)
{
	return platform_driver_register(&wl127x_rfkill_platform_driver);
}

static void __exit wl127x_rfkill_exit(void)
{
	platform_driver_unregister(&wl127x_rfkill_platform_driver);
}

module_init(wl127x_rfkill_init);
module_exit(wl127x_rfkill_exit);

MODULE_ALIAS("platform:wl127x");
MODULE_DESCRIPTION("wl127x-rfkill");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
