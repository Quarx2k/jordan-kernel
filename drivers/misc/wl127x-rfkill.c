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


static int wl127x_bt_rfkill_set_power(void *data, bool blocked)
{
	struct wl127x_rfkill_platform_data *pdata =
		(struct wl127x_rfkill_platform_data *) data;
	int nshutdown_gpio = pdata->bt_nshutdown_gpio;

	if (blocked) {
		gpio_set_value(nshutdown_gpio, 0);
		if (pdata->bt_hw_disable)
			pdata->bt_hw_disable();
	}
	else {
		if (pdata->bt_hw_enable)
			pdata->bt_hw_enable();
		gpio_set_value(nshutdown_gpio, 1);
	}
	return 0;
}

static int wl127x_pwr_ctl_rfkill_set_power(void *data, bool blocked)
{
	struct wl127x_rfkill_platform_data *pdata =
		(struct wl127x_rfkill_platform_data *) data;

	if (blocked) {
		if (pdata->bt_hw_disable)
			pdata->bt_hw_disable();
	} else {
		if (pdata->bt_hw_enable)
			pdata->bt_hw_enable();
	}
	return 0;
}

static int wl127x_fm_rfkill_set_power(void *data, bool blocked)
{
	int nshutdown_gpio = (int) data;

	if (blocked) {
		gpio_set_value(nshutdown_gpio, 0);
	}
	else {
		gpio_set_value(nshutdown_gpio, 1);
	}
	return 0;
}

static const struct rfkill_ops wl127x_bt_rfkill_ops = {
	.set_block = wl127x_bt_rfkill_set_power,
};

static const struct rfkill_ops wl127x_pwr_ctl_rfkill_ops = {
	.set_block = wl127x_pwr_ctl_rfkill_set_power,
};

static const struct rfkill_ops wl127x_fm_rfkill_ops = {
	.set_block = wl127x_fm_rfkill_set_power,
};

static int wl127x_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct wl127x_rfkill_platform_data *pdata = pdev->dev.platform_data;

	if (pdata->bt_nshutdown_gpio >= 0) {
		bool default_blocked = true;  /* power off */
		rc = gpio_request(pdata->bt_nshutdown_gpio,
				  "wl127x_bt_nshutdown_gpio");
		if (unlikely(rc))
			return rc;

		rc = gpio_direction_output(pdata->bt_nshutdown_gpio, 0);
		if (unlikely(rc))
			return rc;

		if (pdata->bt_hw_init)
			rc = pdata->bt_hw_init();
		if (unlikely(rc))
			return rc;

		wl127x_bt_rfkill_set_power((void *)pdata, default_blocked);

		pdata->rfkill[WL127X_BLUETOOTH] = rfkill_alloc(
				"wl127x Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &wl127x_bt_rfkill_ops,
				(void *)pdata);
		if (unlikely(!pdata->rfkill[WL127X_BLUETOOTH]))
			return -ENOMEM;

		rfkill_set_states(pdata->rfkill[WL127X_BLUETOOTH],
				default_blocked, false);

		rc = rfkill_register(pdata->rfkill[WL127X_BLUETOOTH]);
		if (unlikely(rc)) {
			rfkill_destroy(pdata->rfkill[WL127X_BLUETOOTH]);
			return rc;
		}
	}

	if (pdata->pwr_ctl >= 0) {
		bool default_blocked = true;  /* power off */

		pdata->rfkill[WL127X_PWR_CTL] = rfkill_alloc(
				"wl127x Power_Control", &pdev->dev,
				RFKILL_TYPE_PWR_CTL, &wl127x_pwr_ctl_rfkill_ops,
				(void *)pdata);
		if (unlikely(!pdata->rfkill[WL127X_PWR_CTL]))
			return -ENOMEM;

		rfkill_set_states(pdata->rfkill[WL127X_PWR_CTL],
			default_blocked, false);

		rc = rfkill_register(pdata->rfkill[WL127X_PWR_CTL]);
		if (unlikely(rc)) {
			rfkill_destroy(pdata->rfkill[WL127X_PWR_CTL]);
			return rc;
		}
	}

	if (pdata->fm_enable_gpio >= 0) {
		bool default_blocked = true;  /* power off */
		rc = gpio_request(pdata->fm_enable_gpio,
				  "wl127x_fm_enable_gpio");
		if (unlikely(rc))
			return rc;

		rc = gpio_direction_output(pdata->fm_enable_gpio, 0);
		if (unlikely(rc))
			return rc;

		wl127x_fm_rfkill_set_power((void *)pdata->fm_enable_gpio,
				default_blocked);

		pdata->rfkill[WL127X_FM] = rfkill_alloc("wl127x FM Radio",
				&pdev->dev, RFKILL_TYPE_FM,
				&wl127x_fm_rfkill_ops,
				(void *)pdata->fm_enable_gpio);
		if (unlikely(!pdata->rfkill[WL127X_FM]))
			return -ENOMEM;

		rfkill_set_states(pdata->rfkill[WL127X_FM], default_blocked,
				false);

		rc = rfkill_register(pdata->rfkill[WL127X_FM]);
		if (unlikely(rc)) {
			rfkill_destroy(pdata->rfkill[WL127X_FM]);
			return rc;
		}
	}

	return 0;
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

	if (pdata->pwr_ctl >= 0) {
		rfkill_unregister(pdata->rfkill[WL127X_PWR_CTL]);
		rfkill_destroy(pdata->rfkill[WL127X_PWR_CTL]);
	}

	if (pdata->fm_enable_gpio >= 0) {
		rfkill_unregister(pdata->rfkill[WL127X_FM]);
		rfkill_destroy(pdata->rfkill[WL127X_FM]);
		gpio_free(pdata->fm_enable_gpio);
	}

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
