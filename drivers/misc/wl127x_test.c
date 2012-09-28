/*
 * Bluetooth TI wl127x GPIO test driver
 *
 * Copyright (C) 2009 Motorola, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/wl127x-test.h>
#include <net/bluetooth/bluetooth.h>
#include <plat/resource.h>

static int wl127x_hostwake_gpio;

static ssize_t hostwake_sysfs_show(struct class *dev, char *buf)
{
	int data;
	char *str = buf;
	ssize_t count;

	if (wl127x_hostwake_gpio >= 0) {
		data = gpio_get_value(wl127x_hostwake_gpio);
		str += sprintf(str, "%d\n", data);
		count = (ssize_t) (str - buf);
		return count;
	} else {
		return -EINVAL;
	}
}

static CLASS_ATTR(hostwake, S_IRUGO, hostwake_sysfs_show, NULL);

static int wl127x_test_probe(struct platform_device *pdev)
{
	struct wl127x_test_platform_data *pdata = pdev->dev.platform_data;

	wl127x_hostwake_gpio = pdata->hostwake_gpio;

	if (class_create_file(bt_class, &class_attr_hostwake) < 0) {
		printk(KERN_ERR "wl127x-test: failed creating hostwake attr\n");
		return -1;
	}

	resource_request("mpu_latency", &pdev->dev, 150+260-1);

	return 0;
}

static int wl127x_test_remove(struct platform_device *pdev)
{
	resource_release("mpu_latency", &pdev->dev);

	class_remove_file(bt_class, &class_attr_hostwake);

	return 0;
}

static struct platform_driver wl127x_test_platform_driver = {
	.probe = wl127x_test_probe,
	.remove = wl127x_test_remove,
	.driver = {
		   .name = "wl127x-test",
		   .owner = THIS_MODULE,
		   },
};

static int __init wl127x_test_init(void)
{
	return platform_driver_register(&wl127x_test_platform_driver);
}

static void __exit wl127x_test_exit(void)
{
	platform_driver_unregister(&wl127x_test_platform_driver);
}

module_init(wl127x_test_init);
module_exit(wl127x_test_exit);

MODULE_ALIAS("platform:wl127x-test");
MODULE_DESCRIPTION("wl127x GPIO test for board-sholes");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
