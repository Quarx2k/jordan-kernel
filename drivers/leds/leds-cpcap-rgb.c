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

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/leds-cpcap-rgb.h>

struct cpcap_rgb_led_data {
	struct led_classdev cpcap_red_led_class_dev;
	struct led_classdev cpcap_green_led_class_dev;
	struct led_classdev cpcap_blue_led_class_dev;
	struct cpcap_device *cpcap;
	struct regulator *regulator;
	int regulator_state;
};

static uint32_t debug;
module_param_named(debug, debug, uint, 0664);

static struct cpcap_rgb_led_config_data rgb_led_config_data;

void cpcap_rgb_led_set_brightness(struct cpcap_rgb_led_data *cpcap_rgb_data,
				  int color,
				  enum led_brightness value)
{
	unsigned short brightness = 0;
	int cpcap_status = 0;
	int cpcap_register = 0;

	if (color & CPCAP_RGB_RED_LED)
		cpcap_register = CPCAP_REG_REDC;
	else if (color & CPCAP_RGB_GREEN_LED)
		cpcap_register = CPCAP_REG_GREENC;
	else if (color & CPCAP_RGB_BLUE_LED)
		cpcap_register = CPCAP_REG_BLUEC;

	if (value == LED_OFF) {
		/* Due to a HW issue turn off the current then
		turn off the duty cycle */
		brightness = CPCAP_RGB_OFF_1;
		cpcap_status = cpcap_regacc_write(cpcap_rgb_data->cpcap,
						  cpcap_register,
						  brightness,
						  CPCAP_RGB_ON_OFF_MASK);

		brightness = CPCAP_RGB_OFF_2;
	}
	else if (value <= CPCAP_RGB_LOW_LIMIT)
		brightness = CPCAP_RGB_LOW_VALUE;
	else if (value <= CPCAP_RGB_LOW_MED_LIMIT)
		brightness = CPCAP_RGB_LOW_MED_VALUE;
	else if (value <= CPCAP_RGB_MEDIUM_LIMIT)
		brightness = CPCAP_RGB_MEDIUM_VALUE;
	else if (value <= CPCAP_RGB_MED_HIGH_LIMIT)
		brightness = CPCAP_RGB_MED_HIGH_VALUE;
	else
		brightness = CPCAP_RGB_HIGH_VALUE;

	cpcap_status = cpcap_regacc_write(cpcap_rgb_data->cpcap,
					  cpcap_register,
					  brightness,
					  CPCAP_RGB_ON_OFF_MASK);

	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

	if (value > LED_OFF) {
		if (!(cpcap_rgb_data->regulator_state & color)) {
			if (cpcap_rgb_data->regulator) {
				regulator_enable(cpcap_rgb_data->regulator);
				cpcap_rgb_data->regulator_state |= color;
			}
		}
	} else {
		if (cpcap_rgb_data->regulator_state & color) {
			if (cpcap_rgb_data->regulator) {
				regulator_disable(cpcap_rgb_data->regulator);
				cpcap_rgb_data->regulator_state &= ~color;
			}
		}
	}

	return;
}

static void cpcap_rgb_led_red_set(struct led_classdev *led_cdev,
			    enum led_brightness value)
{
	struct cpcap_rgb_led_data *red_led_data =
	    container_of(led_cdev, struct cpcap_rgb_led_data,
			 cpcap_red_led_class_dev);

	if (debug)
		pr_info("%s %d\n", __func__, value);

	cpcap_rgb_led_set_brightness(red_led_data,
				     CPCAP_RGB_RED_LED,
				     value);
}

static void cpcap_rgb_led_green_set(struct led_classdev *led_cdev,
			      enum led_brightness value)
{
	struct cpcap_rgb_led_data *green_led_data =
	    container_of(led_cdev, struct cpcap_rgb_led_data,
			 cpcap_green_led_class_dev);

	if (debug)
		pr_info("%s %d\n", __func__, value);

	cpcap_rgb_led_set_brightness(green_led_data,
				     CPCAP_RGB_GREEN_LED,
				     value);
}

static void cpcap_rgb_led_blue_set(struct led_classdev *led_cdev,
			     enum led_brightness value)
{
	struct cpcap_rgb_led_data *blue_led_data =
	    container_of(led_cdev, struct cpcap_rgb_led_data,
			 cpcap_blue_led_class_dev);

	if (debug)
		pr_info("%s %d\n", __func__, value);

	cpcap_rgb_led_set_brightness(blue_led_data,
				     CPCAP_RGB_BLUE_LED,
				     value);
}

static ssize_t
cpcap_rgb_led_blink(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct cpcap_rgb_led_data *blink_led_data = dev_get_drvdata(dev);
	unsigned long led_blink = LED_OFF;
	int ret;

	ret = strict_strtoul(buf, 10, &led_blink);
	if (ret != 0) {
		pr_err("%s: Invalid parameter sent\n", __func__);
		return -1;
	}

	if (debug)
		pr_info("%s %ld\n", __func__, led_blink);

	if (led_blink > LED_OFF)
		cpcap_uc_start(blink_led_data->cpcap, CPCAP_MACRO_6);
	else
		cpcap_uc_stop(blink_led_data->cpcap, CPCAP_MACRO_6);

	return 0;
}

static DEVICE_ATTR(blink, 0644, NULL, cpcap_rgb_led_blink);

static int cpcap_rgb_led_rgb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct cpcap_rgb_led_data *info;

	if (pdev == NULL) {
		pr_err("%s: Platform device missing\n", __func__);
		ret = -ENODEV;
		goto err_pd_or_kzalloc_failed;
	}
	info = kzalloc(sizeof(struct cpcap_rgb_led_data), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		goto err_pd_or_kzalloc_failed;
	}

	memcpy(&rgb_led_config_data, pdev->dev.platform_data,
		sizeof(rgb_led_config_data));

	info->cpcap = platform_get_drvdata(pdev);
	if (info->cpcap == NULL) {
		pr_err("%s: Platform data missing\n", __func__);
		goto err_pd_or_kzalloc_failed;
	}

	platform_set_drvdata(pdev, info);

	info->regulator = regulator_get(NULL, CPCAP_RGB_LED_REG);
	if (IS_ERR(info->regulator)) {
		pr_err("%s: Cannot get %s regulator\n", __func__,
		       CPCAP_RGB_LED_REG);
		ret = PTR_ERR(info->regulator);
		goto err_request_reg_failed;
	}
	info->regulator_state = 0;

	if (rgb_led_config_data.red_enable) {
		info->cpcap_red_led_class_dev.name =
			rgb_led_config_data.class_name_red;
		info->cpcap_red_led_class_dev.brightness_set =
			cpcap_rgb_led_red_set;
		ret = led_classdev_register(&pdev->dev,
					    &info->cpcap_red_led_class_dev);
		if (ret < 0) {
			pr_err("%s:Register Red LED class failed\n", __func__);
			goto err_reg_red_class_failed;
		}

		if (rgb_led_config_data.blink_enable) {
			ret = device_create_file(
				info->cpcap_red_led_class_dev.dev,
				&dev_attr_blink);
			if (ret < 0) {
				pr_err("%s: File device creation failed: %d\n",
				       __func__, ret);
				goto err_create_blink_failed;
			}
		}
	}

	if (rgb_led_config_data.green_enable) {
		info->cpcap_green_led_class_dev.name =
			rgb_led_config_data.class_name_green;
		info->cpcap_green_led_class_dev.brightness_set =
			cpcap_rgb_led_green_set;
		ret = led_classdev_register(&pdev->dev,
					    &info->cpcap_green_led_class_dev);
		if (ret < 0) {
			pr_err("%s: Register Green LED class failed\n",
			       __func__);
			goto err_reg_green_class_failed;
		}
	}

	if (rgb_led_config_data.blue_enable) {
		info->cpcap_blue_led_class_dev.name =
			rgb_led_config_data.class_name_blue;
		info->cpcap_blue_led_class_dev.brightness_set =
			cpcap_rgb_led_blue_set;
		ret = led_classdev_register(&pdev->dev,
			&info->cpcap_blue_led_class_dev);
		if (ret < 0) {
			pr_err("%s: Register blue LED class failed\n",
			       __func__);
			goto err_reg_blue_class_failed;
		}
	}

	if (debug)
		pr_info("%s successful\n", __func__);

	return ret;

err_reg_blue_class_failed:
	led_classdev_unregister(&info->cpcap_green_led_class_dev);

err_reg_green_class_failed:
	device_remove_file(info->cpcap_red_led_class_dev.dev,
			   &dev_attr_blink);

err_create_blink_failed:
	led_classdev_unregister(&info->cpcap_red_led_class_dev);

err_reg_red_class_failed:
	if (info->regulator)
		regulator_put(info->regulator);

err_request_reg_failed:
	kfree(info);

err_pd_or_kzalloc_failed:
	if (debug)
		pr_info("%s failed\n", __func__);

	return ret;
}

static int cpcap_rgb_led_rgb_remove(struct platform_device *pdev)
{
	struct cpcap_rgb_led_data *info = platform_get_drvdata(pdev);

	if (debug)
		pr_info("%s\n", __func__);

	if (info->regulator)
		regulator_put(info->regulator);

	device_remove_file(info->cpcap_red_led_class_dev.dev,
			   &dev_attr_blink);

	led_classdev_unregister(&info->cpcap_red_led_class_dev);
	led_classdev_unregister(&info->cpcap_green_led_class_dev);
	led_classdev_unregister(&info->cpcap_blue_led_class_dev);
	kfree(info);
	return 0;
}

static struct platform_driver cpcap_rgb_led_rgb_driver = {
	.probe = cpcap_rgb_led_rgb_probe,
	.remove = cpcap_rgb_led_rgb_remove,
	.driver = {
		   .name = CPCAP_RGB_LED_DRV_NAME,
		   },
};

static int __init cpcap_rgb_led_rgb_init(void)
{
	if (debug)
		pr_info("%s\n", __func__);

	return cpcap_driver_register(&cpcap_rgb_led_rgb_driver);
}

static void __exit cpcap_rgb_led_rgb_exit(void)
{
	if (debug)
		pr_info("%s\n", __func__);

	platform_driver_unregister(&cpcap_rgb_led_rgb_driver);
}

module_init(cpcap_rgb_led_rgb_init);
module_exit(cpcap_rgb_led_rgb_exit);

MODULE_DESCRIPTION("CPCAP RGB LED driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
