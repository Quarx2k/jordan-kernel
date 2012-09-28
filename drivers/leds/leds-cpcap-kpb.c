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
#include <linux/spi/spi.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/leds-cpcap-kpb.h>


struct kpb_led_data {
	struct led_classdev cpcap_kpb_led_class_dev;
	struct cpcap_device *cpcap;
	struct regulator *regulator;
	int regulator_state;
};

static uint32_t debug;
module_param_named(debug, debug, uint, 0664);

static struct cpcap_kpb_led_config_data kpb_led_config_data;

static void cpcap_kpb_led_set(struct led_classdev *led_cdev,
			  enum led_brightness value)
{
	int cpcap_status = 0;

	struct kpb_led_data *led_data =
	    container_of(led_cdev, struct kpb_led_data,
			 cpcap_kpb_led_class_dev);

	if (debug)
		pr_info("%s %d\n", __func__, value);

	if (value > LED_OFF) {
		if ((led_data->regulator) &&
		    (led_data->regulator_state == 0)) {
			regulator_enable(led_data->regulator);
			led_data->regulator_state = 1;
		}

		cpcap_status =
			cpcap_regacc_write(led_data->cpcap,
					   CPCAP_REG_KLC,
					   kpb_led_config_data.on,
					   CPCAP_KPB_ON_OFF_MASK);
	} else {
		if ((led_data->regulator) &&
		    (led_data->regulator_state == 1)) {
			regulator_disable(led_data->regulator);
			led_data->regulator_state = 0;
		}

		cpcap_status = cpcap_regacc_write(led_data->cpcap,
						  CPCAP_REG_KLC,
						  CPCAP_KPB_OFF_1,
						  CPCAP_KPB_ON_OFF_MASK);

		cpcap_status = cpcap_regacc_write(led_data->cpcap,
						  CPCAP_REG_KLC,
						  CPCAP_KPB_OFF_2,
						  CPCAP_KPB_ON_OFF_MASK);
	}

	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

	return;
}

static int cpcap_kpb_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	int cpcap_status = 0;
	struct kpb_led_data *info;

	if (pdev == NULL) {
		pr_err("%s: Platform device missing\n", __func__);
		ret = -ENODEV;
		goto err_pd_or_kzalloc_failed;
	}
	info = kzalloc(sizeof(struct kpb_led_data), GFP_KERNEL);
	if (info == NULL) {
		pr_err("%s: Unable to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_pd_or_kzalloc_failed;
	}

	if (pdev->dev.platform_data == NULL) {
		pr_err("%s: Platform data missing\n", __func__);
		ret = -ENODEV;
		goto err_info_missing;
	}

	memcpy(&kpb_led_config_data, pdev->dev.platform_data,
	       sizeof(kpb_led_config_data));

	info->cpcap = platform_get_drvdata(pdev);
	if (info->cpcap == NULL) {
		pr_err("%s: Driver data mising\n", __func__);
		goto err_info_missing;
	}

	platform_set_drvdata(pdev, info);

	info->regulator = regulator_get(NULL, CPCAP_KPB_LED_REG);
	if (IS_ERR(info->regulator)) {
		pr_err("%s: Cannot get %s regulator\n", __func__,
		       CPCAP_KPB_LED_REG);
		ret = PTR_ERR(info->regulator);
		goto err_info_missing;
	}

	info->regulator_state = 0;

	cpcap_status = cpcap_regacc_write(info->cpcap,
					  CPCAP_REG_KLC,
					  kpb_led_config_data.init,
					  CPCAP_KPB_INIT_MASK);
	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

	info->cpcap_kpb_led_class_dev.name = kpb_led_config_data.class_name;
	info->cpcap_kpb_led_class_dev.brightness_set = cpcap_kpb_led_set;
	ret = led_classdev_register(&pdev->dev, &info->cpcap_kpb_led_class_dev);
	if (ret) {
		printk(KERN_ERR "Register kpb led class failed %d\n", ret);
		goto err_reg_kpb_failed;
	}

	if (debug)
		pr_info("%s successful\n", __func__);

	return ret;

err_reg_kpb_failed:
	if (info->regulator)
		regulator_put(info->regulator);

err_info_missing:
	kfree(info);

err_pd_or_kzalloc_failed:
	if (debug)
		pr_info("%s failed\n", __func__);

	return ret;
}

static int cpcap_kpb_led_remove(struct platform_device *pdev)
{
	struct kpb_led_data *info = platform_get_drvdata(pdev);;

	if (debug)
		pr_info("%s\n", __func__);

	if (info->regulator)
		regulator_put(info->regulator);

	led_classdev_unregister(&info->cpcap_kpb_led_class_dev);
	kfree(info);
	return 0;
}

static struct platform_driver cpcap_kpb_led_driver = {
	.probe   = cpcap_kpb_led_probe,
	.remove  = cpcap_kpb_led_remove,
	.driver  = {
		.name  = CPCAP_KPB_LED_DRV_NAME,
		.owner = THIS_MODULE,
	},
};


static int cpcap_kpb_led_init(void)
{
	if (debug)
		pr_info("%s\n", __func__);

	return cpcap_driver_register(&cpcap_kpb_led_driver);
}

static void cpcap_kpb_led_shutdown(void)
{
	if (debug)
		pr_info("%s\n", __func__);

	platform_driver_unregister(&cpcap_kpb_led_driver);
}

module_init(cpcap_kpb_led_init);
module_exit(cpcap_kpb_led_shutdown);

MODULE_DESCRIPTION("CPCAP Keypad Backlight driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GNU");






