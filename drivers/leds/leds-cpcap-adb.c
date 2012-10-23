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
#include <linux/leds-cpcap-abmode.h>
#include <linux/leds-cpcap-adb.h>

struct cpcap_adb_led_data {
	struct led_classdev cpcap_adb_led_class_dev;
	struct cpcap_device *cpcap;
	struct regulator *regulator;
	int regulator_state;
};

static uint32_t debug;
module_param_named(debug, debug, uint, 0664);

static struct cpcap_adb_led_config_data adb_led_config_data;

static void cpcap_adb_led_set(struct led_classdev *led_cdev,
			      enum led_brightness value)
{
	int cpcap_status = 0;

	struct cpcap_adb_led_data *cpcap_adb_led_data =
		container_of(led_cdev, struct cpcap_adb_led_data,
			     cpcap_adb_led_class_dev);

	if (debug)
		pr_info("%s %d\n", __func__, value);

	if (value > LED_OFF) {
		if ((cpcap_adb_led_data->regulator) &&
		    (cpcap_adb_led_data->regulator_state == 0)) {
			regulator_enable(cpcap_adb_led_data->regulator);
			cpcap_adb_led_data->regulator_state = 1;
		}

		cpcap_status = cpcap_regacc_write(cpcap_adb_led_data->cpcap,
					CPCAP_REG_ADLC,
					adb_led_config_data.on,
					CPCAP_ADB_ON_OFF_MASK);
	} else {
		if ((cpcap_adb_led_data->regulator) &&
		    (cpcap_adb_led_data->regulator_state == 1)) {
			regulator_disable(cpcap_adb_led_data->regulator);
			cpcap_adb_led_data->regulator_state = 0;
		}
		/* Due to a HW issue turn off the current then
		turn off the duty cycle */
		cpcap_status = cpcap_regacc_write(cpcap_adb_led_data->cpcap,
						  CPCAP_REG_ADLC,
						  CPCAP_ADB_OFF_1,
						  CPCAP_ADB_ON_OFF_MASK);

		cpcap_status = cpcap_regacc_write(cpcap_adb_led_data->cpcap,
						  CPCAP_REG_ADLC,
						  CPCAP_ADB_OFF_2,
						  CPCAP_ADB_ON_OFF_MASK);
	}

	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

}
EXPORT_SYMBOL(cpcap_adb_led_set);

static int cpcap_adb_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	int cpcap_status = 0;
	struct cpcap_adb_led_data *info;

	if (pdev == NULL) {
		pr_err("%s: Platform device required\n", __func__);
		ret = -ENODEV;
		goto err_pd_or_kzalloc_failed;
	}
	info = kzalloc(sizeof(struct cpcap_adb_led_data), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		goto err_pd_or_kzalloc_failed;
	}

	if (pdev->dev.platform_data == NULL) {
		pr_err("%s: Platforn data required\n", __func__);
		ret = -ENODEV;
		goto err_pd_or_kzalloc_failed;
	}
	memcpy(&adb_led_config_data, pdev->dev.platform_data,
		sizeof(adb_led_config_data));

	info->cpcap = platform_get_drvdata(pdev);
	if (info->cpcap == NULL) {
		pr_err("%s: Driver data required\n", __func__);
		ret = -ENODEV;
		goto err_request_reg_failed;
	}
	platform_set_drvdata(pdev, info);

	info->regulator = regulator_get(NULL, CPCAP_ADB_LED_REG);
	if (IS_ERR(info->regulator)) {
		pr_err("%s: Cannot get %s regulator\n", __func__,
		       CPCAP_ADB_LED_REG);
		ret = PTR_ERR(info->regulator);
		goto err_request_reg_failed;

	}

	info->regulator_state = 0;

	cpcap_status = cpcap_regacc_write(info->cpcap,
					  CPCAP_REG_ADLC,
					  adb_led_config_data.init,
					  CPCAP_ADB_INIT_MASK);

	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

	cpcap_status = cpcap_regacc_write(info->cpcap,
		CPCAP_REG_ABC,
		adb_led_config_data.abmode_config->abmode_init,
		CPCAP_ABMODE_INIT_MASK);

	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

	info->cpcap_adb_led_class_dev.name = adb_led_config_data.class_name;
	info->cpcap_adb_led_class_dev.brightness_set = cpcap_adb_led_set;
	ret = led_classdev_register(&pdev->dev, &info->cpcap_adb_led_class_dev);
	if (ret < 0) {
		pr_err("%s:Register button backlight class failed\n", __func__);
		goto err_reg_button_class_failed;
	}

	if (debug)
		pr_info("%s successful\n", __func__);

	return ret;

err_reg_button_class_failed:
	if (info->regulator)
		regulator_put(info->regulator);
err_request_reg_failed:
	kfree(info);

err_pd_or_kzalloc_failed:
	if (debug)
		pr_info("%s failed\n", __func__);

	return ret;
}

static int cpcap_adb_led_remove(struct platform_device *pdev)
{
	struct cpcap_adb_led_data *info = platform_get_drvdata(pdev);

	if (debug)
		pr_info("%s\n", __func__);

	if (info->regulator)
		regulator_put(info->regulator);

	led_classdev_unregister(&info->cpcap_adb_led_class_dev);
	kfree(info);
	return 0;
}

static struct platform_driver cpcap_adb_led_driver = {
	.probe = cpcap_adb_led_probe,
	.remove = cpcap_adb_led_remove,
	.driver = {
		   .name = CPCAP_ADB_LED_DRV_NAME,
		   },
};


static int __init cpcap_adb_led_init(void)
{
	if (debug)
		pr_info("%s\n", __func__);

	return cpcap_driver_register(&cpcap_adb_led_driver);
}

static void __exit cpcap_adb_led_exit(void)
{
	if (debug)
		pr_info("%s\n", __func__);

	platform_driver_unregister(&cpcap_adb_led_driver);
}

module_init(cpcap_adb_led_init);
module_exit(cpcap_adb_led_exit);

MODULE_DESCRIPTION("CPCAP Auxiliary Display Backlight driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
