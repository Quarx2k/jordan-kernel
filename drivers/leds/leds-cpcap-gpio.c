/*
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/spi/cpcap.h>
#include <linux/platform_device.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/leds-cpcap-gpio.h>

struct gpio_led_data {
	struct led_classdev cpcap_gpio_led_class_dev;
	struct cpcap_device *cpcap;
};

static uint32_t debug;
module_param_named(debug, debug, uint, 0664);

static struct cpcap_gpio_led_config_data gpio_led_config_data;

static void cpcap_gpio_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	int cpcap_status = 0;

	struct gpio_led_data *gpio_led_data =
	    container_of(led_cdev, struct gpio_led_data,
			 cpcap_gpio_led_class_dev);

	if (debug)
		pr_info("%s %d\n", __func__, value);

	if (value > LED_OFF) {
		cpcap_status = cpcap_regacc_write(gpio_led_data->cpcap,
						  gpio_led_config_data.reg,
						  CPCAP_GPIO_ON,
						  CPCAP_GPIO_ON_OFF_MASK);
	} else {
		cpcap_status = cpcap_regacc_write(gpio_led_data->cpcap,
						  gpio_led_config_data.reg,
						  CPCAP_GPIO_OFF,
						  CPCAP_GPIO_ON_OFF_MASK);
	}

	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

	return;
}

static int cpcap_gpio_led_probe(struct platform_device *pdev)
{
	int ret;
	struct gpio_led_data *info;

	if (pdev == NULL) {
		pr_err("%s: platform device required\n", __func__);
		ret = -ENODEV;
		goto err_pd_or_kzalloc_failed;
	}
	info = kzalloc(sizeof(struct cpcap_gpio_led_config_data), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		goto err_pd_or_kzalloc_failed;
	}

	if (pdev->dev.platform_data == NULL) {
		pr_err("%s: platform data required\n", __func__);
		ret = -ENODEV;
		goto err_pd_or_kzalloc_failed;
	}
	memcpy(&gpio_led_config_data, pdev->dev.platform_data,
	       sizeof(gpio_led_config_data));

	info->cpcap = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, info);

	ret = cpcap_regacc_write(info->cpcap,
				 gpio_led_config_data.reg,
				 gpio_led_config_data.init,
				 gpio_led_config_data.init_mask);
	if (ret < 0) {
		pr_err("%s: Writing CPCAP failed: \n", __func__);
		goto err_probe_failed;
	}

	info->cpcap_gpio_led_class_dev.name = gpio_led_config_data.class_name;
	info->cpcap_gpio_led_class_dev.brightness_set = cpcap_gpio_led_set;

	ret = led_classdev_register(&pdev->dev,
				    &info->cpcap_gpio_led_class_dev);
	if (ret < 0) {
		pr_err("%s: Register led class failed: \n", __func__);
		goto err_probe_failed;
	}

	if (debug)
		pr_info("%s successful\n", __func__);

	return ret;

err_probe_failed:
	kfree(info);

err_pd_or_kzalloc_failed:
	if (debug)
		pr_info("%s failed\n", __func__);

	return ret;
}

static int cpcap_gpio_led_remove(struct platform_device *pdev)
{
	struct gpio_led_data *info = platform_get_drvdata(pdev);

	if (debug)
		pr_info("%s\n", __func__);

	led_classdev_unregister(&info->cpcap_gpio_led_class_dev);
	return 0;
}

static struct platform_driver cpcap_gpio_led_driver = {
	.probe = cpcap_gpio_led_probe,
	.remove = cpcap_gpio_led_remove,
	.driver = {
		   .name = CPCAP_GPIO_LED_DRV_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init cpcap_gpio_led_init(void)
{
	if (debug)
		pr_info("%s\n", __func__);

	return cpcap_driver_register(&cpcap_gpio_led_driver);
}

static void __exit cpcap_gpio_led_exit(void)
{
	if (debug)
		pr_info("%s\n", __func__);

	platform_driver_unregister(&cpcap_gpio_led_driver);
}

module_init(cpcap_gpio_led_init);
module_exit(cpcap_gpio_led_exit);

MODULE_DESCRIPTION("CPCAP GPIO LED driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
