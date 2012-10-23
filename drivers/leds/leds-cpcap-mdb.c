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
#include <linux/leds-cpcap-mdb.h>

struct cpcap_mdb_led_data {
	struct led_classdev cpcap_mdb_led_class_dev;
	struct cpcap_device *cpcap;
	struct regulator *regulator;
	int regulator_state;
};

static uint32_t debug;
module_param_named(debug, debug, uint, 0664);

static struct cpcap_mdb_led_config_data mdb_led_config_data;

static void cpcap_mdb_led_set(struct led_classdev *led_cdev,
			      enum led_brightness value)
{
	int cpcap_status = 0;
	u16 backlight_value;

	struct cpcap_mdb_led_data *cpcap_mdb_led_data =
	    container_of(led_cdev, struct cpcap_mdb_led_data,
			 cpcap_mdb_led_class_dev);

	if (debug)
		pr_info("%s %d\n", __func__, value);

	if (value > LED_OFF) {
		if ((cpcap_mdb_led_data->regulator) &&
		    (cpcap_mdb_led_data->regulator_state == 0)) {
			regulator_enable(cpcap_mdb_led_data->regulator);
			cpcap_mdb_led_data->regulator_state = 1;
		}

		backlight_value = value >> 1;
		backlight_value = (backlight_value << 5) | CPCAP_MDB_ON;
		cpcap_status = cpcap_regacc_write(cpcap_mdb_led_data->cpcap,
						  CPCAP_REG_MDLC,
						  backlight_value,
						  CPCAP_MDB_ON_OFF_MASK);
	} else {
		if ((cpcap_mdb_led_data->regulator) &&
		    (cpcap_mdb_led_data->regulator_state == 1)) {
			regulator_disable(cpcap_mdb_led_data->regulator);
			cpcap_mdb_led_data->regulator_state = 0;
		}
		/* Due to a HW issue turn off the current then
		turn off the duty cycle */
		cpcap_status = cpcap_regacc_write(cpcap_mdb_led_data->cpcap,
						  CPCAP_REG_MDLC,
						  CPCAP_MDB_OFF_1,
						  CPCAP_MDB_ON_OFF_MASK);

		cpcap_status = cpcap_regacc_write(cpcap_mdb_led_data->cpcap,
						  CPCAP_REG_MDLC,
						  CPCAP_MDB_OFF_2,
						  CPCAP_MDB_ON_OFF_MASK);
	}

	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

}
EXPORT_SYMBOL(cpcap_mdb_led_set);

static int cpcap_mdb_led_probe(struct platform_device *pdev)
{
	int ret = 0;
	int cpcap_status = 0;
	struct cpcap_mdb_led_data *info;

	if (pdev == NULL) {
		pr_err("%s: platform data required\n", __func__);
		ret = -ENODEV;
		goto err_pd_or_kzalloc_failed;
	}
	info = kzalloc(sizeof(struct cpcap_mdb_led_data), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		goto err_pd_or_kzalloc_failed;
	}

	if (pdev->dev.platform_data == NULL) {
		pr_err("%s: Platform device missing\n", __func__);
		ret = -ENODEV;
		goto err_info_missing;
	}
	memcpy(&mdb_led_config_data, pdev->dev.platform_data,
		sizeof(mdb_led_config_data));

	info->cpcap = platform_get_drvdata(pdev);
	if (info->cpcap == NULL) {
		pr_err("%s: Driver data mising\n", __func__);
		goto err_info_missing;
	}
	platform_set_drvdata(pdev, info);

	info->regulator = regulator_get(NULL, CPCAP_MDB_LED_REG);
	if (IS_ERR(info->regulator)) {
		pr_err("%s: Cannot get %s regulator\n", __func__,
		       CPCAP_MDB_LED_REG);
		ret = PTR_ERR(info->regulator);
		goto err_info_missing;

	}

	info->regulator_state = 0;

	cpcap_status = cpcap_regacc_write(info->cpcap,
					  CPCAP_REG_MDLC,
					  mdb_led_config_data.init,
					  CPCAP_MDB_INIT_MASK);

	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

	cpcap_status = cpcap_regacc_write(info->cpcap,
		CPCAP_REG_ABC,
		mdb_led_config_data.abmode_config->abmode_init,
		CPCAP_ABMODE_INIT_MASK);

	if (cpcap_status < 0)
		pr_err("%s: Writing to the register failed for %i\n",
		       __func__, cpcap_status);

	info->cpcap_mdb_led_class_dev.name = mdb_led_config_data.class_name;
	info->cpcap_mdb_led_class_dev.brightness_set = cpcap_mdb_led_set;
	ret = led_classdev_register(&pdev->dev, &info->cpcap_mdb_led_class_dev);
	if (ret < 0) {
		pr_err("%s:Register display backlight class failed\n",
		       __func__);
		goto err_reg_display_class_failed;
	}

	if (debug)
		pr_info("%s successful\n", __func__);

	return ret;

err_reg_display_class_failed:
	if (info->regulator)
		regulator_put(info->regulator);

err_info_missing:
	kfree(info);

err_pd_or_kzalloc_failed:
	if (debug)
		pr_info("%s failed\n", __func__);

	return ret;
}

static int cpcap_mdb_led_remove(struct platform_device *pdev)
{
	struct cpcap_mdb_led_data *info = platform_get_drvdata(pdev);

	if (info->regulator)
		regulator_put(info->regulator);

	led_classdev_unregister(&info->cpcap_mdb_led_class_dev);
	kfree(info);
	return 0;
}

static struct platform_driver cpcap_mdb_led_driver = {
	.probe = cpcap_mdb_led_probe,
	.remove = cpcap_mdb_led_remove,
	.driver = {
		   .name = CPCAP_MDB_LED_DRV_NAME,
		   },
};


static int __init cpcap_mdb_led_init(void)
{
	return cpcap_driver_register(&cpcap_mdb_led_driver);
}

static void __exit cpcap_mdb_led_exit(void)
{
	platform_driver_unregister(&cpcap_mdb_led_driver);
}

module_init(cpcap_mdb_led_init);
module_exit(cpcap_mdb_led_exit);

MODULE_DESCRIPTION("CPCAP Main Display Backlight driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
