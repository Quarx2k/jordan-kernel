/*
 * Copyright (C) 2009 Motorola, Inc.
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include <linux/led-cpcap-lm3554.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>

#define LM3554_ALLOWED_R_BYTES 1
#define LM3554_ALLOWED_W_BYTES 2
#define LM3554_MAX_RW_RETRIES 5
#define LM3554_I2C_RETRY_DELAY 10
#define LM3554_TORCH_STEP	32
#define LM3554_STROBE_STEP	16

#define LM3554_TORCH_BRIGHTNESS		0xA0
#define LM3554_FLASH_BRIGHTNESS		0xB0
#define LM3554_FLASH_DURATION		0xC0
#define LM3554_FLAG_REG			0xD0
#define LM3554_CONFIG_REG_1		0xE0
#define LM3554_CONFIG_REG_2		0xF0
#define LM3554_VIN_MONITOR		0x80
#define LM3554_GPIO_REG			0x20

#define LED_FAULT		0x04
#define THERMAL_SHUTDOWN 0x02
#define TX1_INTERRUPT_FAULT 0x08
#define THERMAL_MONITOR_FAULT 0x20
#define VOLTAGE_MONITOR_FAULT 0x80

struct lm3554_data {
	struct i2c_client *client;
	struct lm3554_platform_data *pdata;
	struct led_classdev led_dev;
	struct led_classdev spotlight_dev;
	int camera_strobe_brightness;
	int flash_light_brightness;
};

int lm3554_read_reg(struct lm3554_data *torch_data, uint8_t reg, uint8_t * val)
{
	int err = -1;
	int i = 0;
	unsigned char value[1];

	if (!val) {
		pr_err("%s: invalid value pointer\n", __func__);
		return -EINVAL;
	}
	/* If I2C client doesn't exist */
	if (torch_data->client == NULL) {
		pr_err("%s: null i2c client\n", __func__);
		return -EUNATCH;
	}
	value[0] = reg & 0xFF;

	do {
		err = i2c_master_send(torch_data->client, value, 1);
		if (err == 1)
			err = i2c_master_recv(torch_data->client, val, 1);
		if (err != 1)
			msleep_interruptible(LM3554_I2C_RETRY_DELAY);
	} while ((err != 1) && ((++i) < LM3554_MAX_RW_RETRIES));

	if (err != 1)
		return err;

	return 0;
}

int lm3554_write_reg(struct lm3554_data *torch_data, uint8_t reg, uint8_t val)
{
	int err = -1;
	int i = 0;
	uint8_t buf[2] = { reg, val };

	/* If I2C client doesn't exist */
	if (torch_data->client == NULL) {
		pr_err("%s: null i2c client\n", __func__);
		return -EUNATCH;
	}

	do {
		err = i2c_master_send(torch_data->client, buf, 2);

		if (err != 2)
			msleep_interruptible(LM3554_I2C_RETRY_DELAY);
	} while ((err != 2) && ((++i) < LM3554_MAX_RW_RETRIES));

	if (err != 2)
		return err;

	return 0;
}

int lm3554_init_registers(struct lm3554_data *torch_data)
{
	if (lm3554_write_reg(torch_data, LM3554_TORCH_BRIGHTNESS,
			     torch_data->pdata->torch_brightness_def) ||
	    lm3554_write_reg(torch_data, LM3554_FLASH_BRIGHTNESS,
			     torch_data->pdata->flash_brightness_def) ||
	    lm3554_write_reg(torch_data, LM3554_FLASH_DURATION,
			     torch_data->pdata->flash_duration_def) ||
	    lm3554_write_reg(torch_data, LM3554_CONFIG_REG_1,
			     torch_data->pdata->config_reg_1_def) ||
	    lm3554_write_reg(torch_data, LM3554_CONFIG_REG_2,
			     torch_data->pdata->config_reg_2_def) ||
	    lm3554_write_reg(torch_data, LM3554_VIN_MONITOR,
			     torch_data->pdata->vin_monitor_def) ||
	    lm3554_write_reg(torch_data, LM3554_GPIO_REG,
			     torch_data->pdata->gpio_reg_def)) {
		pr_err("%s:Register initialization failed\n", __func__);
		return -EIO;
	}

	return 0;
}

/* This is a dummy interface for the LED class this will clear
the error flag register */
static void lm3554_brightness_set(struct led_classdev *led_cdev,
				  enum led_brightness value)
{
	int err;
	uint8_t err_flags;

	struct lm3554_data *torch_data =
	    container_of(led_cdev, struct lm3554_data, led_dev);

	err = lm3554_read_reg(torch_data, LM3554_FLAG_REG, &err_flags);
	if (err) {
		pr_err("%s: Reading the status failed for %i\n", __func__,
		       err);
		return;
	}
	return;
}

static ssize_t lm3554_strobe_err_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int err;
	uint8_t err_flags;
	struct i2c_client *client = container_of(dev->parent,
						 struct i2c_client, dev);
	struct lm3554_data *torch_data = i2c_get_clientdata(client);

	err = lm3554_read_reg(torch_data, LM3554_FLAG_REG, &err_flags);
	if (err) {
		pr_err("%s: Reading the status failed for %i\n",
			__func__, err);
		return -EIO;
	}
	err = lm3554_write_reg(torch_data, LM3554_FLAG_REG, 0x00);
	if (err) {
		pr_err("%s: Clearing the err flags failed %i\n",
			__func__, err);
		return -EIO;
	}
	sprintf(buf, "%d\n", (err_flags & 0xbf));

	return sizeof(buf);
}
static DEVICE_ATTR(strobe_err, 0644, lm3554_strobe_err_show, NULL);

static ssize_t lm3554_torch_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev->parent,
						 struct i2c_client, dev);
	struct lm3554_data *torch_data = i2c_get_clientdata(client);

	sprintf(buf, "%d\n", torch_data->flash_light_brightness);

	return sizeof(buf);
}

static ssize_t lm3554_torch_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int err;
	unsigned long torch_val = LED_OFF;
	uint8_t val;
	struct i2c_client *client = container_of(dev->parent,
						 struct i2c_client, dev);
	struct lm3554_data *torch_data = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &torch_val);
	if (err) {
		pr_err("%s: Invalid parameter sent\n", __func__);
		return -1;
	}

	err = lm3554_read_reg(torch_data, LM3554_TORCH_BRIGHTNESS, &val);
	if (err)
		return err;
	/* Clear out the Enable and brightness bits */
	val &= 0xc4;

	if (torch_val) {
		val |= ((torch_val / LM3554_TORCH_STEP) << 3);
		val |= 0x02;
	}

	err = lm3554_write_reg(torch_data, LM3554_CONFIG_REG_2, 0x08);
	if (err) {
		pr_err("%s: Configuring the VIN Monitor failed for "
		       "%i\n", __func__, err);
		return -EIO;
	}

	err = lm3554_write_reg(torch_data,
			       LM3554_TORCH_BRIGHTNESS, val);
	if (err) {
		pr_err("%s: Configuring the flash light failed for "
		       "%i\n", __func__, err);
		return -EIO;
	}
	torch_data->flash_light_brightness = torch_val;

	return err;
}

static DEVICE_ATTR(flash_light, 0644, lm3554_torch_show, lm3554_torch_store);

static void lm3554_spot_light_brightness_set(struct led_classdev *led_cdev,
				  enum led_brightness value)
{
	int err;
	uint8_t val;
	unsigned long torch_val = value;

	struct lm3554_data *torch_data =
	    container_of(led_cdev, struct lm3554_data, spotlight_dev);

	err = lm3554_read_reg(torch_data, LM3554_TORCH_BRIGHTNESS, &val);
	if (err)
		return;
	/* Clear out the Enable and brightness bits */
	val &= 0xc4;

	if (torch_val) {
		val |= ((torch_val / LM3554_TORCH_STEP) << 3);
		val |= 0x02;
	}

	err = lm3554_write_reg(torch_data, LM3554_CONFIG_REG_2, 0x08);
	if (err) {
		pr_err("%s: Configuring the VIN Monitor failed for "
		       "%i\n", __func__, err);
		return;
	}

	err = lm3554_write_reg(torch_data,
			       LM3554_TORCH_BRIGHTNESS, val);
	if (err) {
		pr_err("%s: Configuring the flash light failed for "
		       "%i\n", __func__, err);
		return;
	}

	return;
}

static ssize_t lm3554_strobe_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev->parent,
						 struct i2c_client, dev);
	struct lm3554_data *torch_data = i2c_get_clientdata(client);

	sprintf(buf, "%d\n", torch_data->camera_strobe_brightness);

	return sizeof(buf);
}

static ssize_t lm3554_strobe_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int err;
	unsigned long strobe_val = 0;
	uint8_t err_flags;
	uint8_t val;
	uint8_t strobe_brightness;
	struct i2c_client *client = container_of(dev->parent,
						 struct i2c_client, dev);
	struct lm3554_data *torch_data = i2c_get_clientdata(client);



	err = strict_strtoul(buf, 10, &strobe_val);
	if (err) {
		pr_err("%s: Invalid parameter sent\n", __func__);
		return -1;
	}

	err = lm3554_read_reg(torch_data, LM3554_CONFIG_REG_1, &val);

	if (err) {
		pr_err("%s: Configuring the flash light failed for %i\n",
		     __func__, err);
		return -EIO;
	}


	err = lm3554_write_reg(torch_data,
			       LM3554_TORCH_BRIGHTNESS,
			       torch_data->
			       pdata->torch_brightness_def);
	if (err) {
		pr_err("%s:Configuring torch brightness failed\n",
		       __func__);
		return -EIO;
	}
	err = lm3554_read_reg(torch_data, LM3554_FLASH_BRIGHTNESS,
			      &strobe_brightness);
	if (err) {
		pr_err("%s: Configuring flash brightness failed\n",
		       __func__);
		return -EIO;
	}
	strobe_brightness &= 0x83;
	strobe_brightness |= ((strobe_val / LM3554_STROBE_STEP) << 3);
	err = lm3554_write_reg(torch_data, LM3554_FLASH_BRIGHTNESS,
			       strobe_brightness);

	if (err) {
		pr_err("%s: Configuring the strobe failed for "
		       "%i\n", __func__, err);
		return -EIO;
	}

	torch_data->camera_strobe_brightness = strobe_val;
	val |= 0x04;
	if (strobe_val)
		val &= 0xfb;

	err = lm3554_write_reg(torch_data, LM3554_CONFIG_REG_1, val);
	return 0;
}

static DEVICE_ATTR(camera_strobe, 0644, lm3554_strobe_show,
		   lm3554_strobe_store);

static int lm3554_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lm3554_platform_data *pdata = client->dev.platform_data;
	struct lm3554_data *torch_data;
	int err = -1;

	if (pdata == NULL) {
		err = -ENODEV;
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		goto error1;
	}

	if (!pdata->flags) {
		pr_err("%s: Device does not exist\n", __func__);
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		dev_err(&client->dev, "client not i2c capable\n");
		goto error1;
	}

	torch_data = kzalloc(sizeof(struct lm3554_data), GFP_KERNEL);
	if (torch_data == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "kzalloc failed\n");
		goto error1;
	}

	torch_data->client = client;
	torch_data->pdata = pdata;

	i2c_set_clientdata(client, torch_data);

	err = lm3554_init_registers(torch_data);
	if (err < 0)
		goto error2;

	torch_data->led_dev.name = LM3554_LED_DEV;
	torch_data->led_dev.brightness_set = lm3554_brightness_set;
	err = led_classdev_register((struct device *)
				    &client->dev, &torch_data->led_dev);
	if (err < 0) {
		err = -ENODEV;
		pr_err("%s: Register led class failed: %d\n", __func__, err);
		goto error3;
	}

	err = device_create_file(torch_data->led_dev.dev,
				 &dev_attr_flash_light);
	if (err < 0) {
		err = -ENODEV;
		pr_err("%s:File device creation failed: %d\n", __func__, err);
		goto error4;
	}

	err = device_create_file(torch_data->led_dev.dev,
				 &dev_attr_camera_strobe);
	if (err < 0) {
		err = -ENODEV;
		pr_err("%s:File device creation failed: %d\n", __func__, err);
		goto error5;
	}
	err = device_create_file(torch_data->led_dev.dev,
				 &dev_attr_strobe_err);
	if (err < 0) {
		err = -ENODEV;
		pr_err("%s:File device creation failed: %d\n", __func__, err);
		goto error6;
	}

	torch_data->spotlight_dev.name = LM3554_LED_SPOTLIGHT;
	torch_data->spotlight_dev.brightness_set =
		lm3554_spot_light_brightness_set;
	err = led_classdev_register((struct device *)
				    &client->dev, &torch_data->spotlight_dev);
	if (err < 0) {
		err = -ENODEV;
		pr_err("%s: Register led class failed: %d\n", __func__, err);
		goto error6;
	}
	pr_info("LM3554 torch initialized\n");

	return 0;

error6:
	device_remove_file(torch_data->led_dev.dev, &dev_attr_camera_strobe);
error5:
	device_remove_file(torch_data->led_dev.dev, &dev_attr_flash_light);
error4:
	led_classdev_unregister(&torch_data->led_dev);
error3:
error2:
	kfree(torch_data);
error1:
	return err;
}

static int lm3554_remove(struct i2c_client *client)
{
	struct lm3554_data *torch_data = i2c_get_clientdata(client);

	device_remove_file(torch_data->led_dev.dev, &dev_attr_camera_strobe);
	device_remove_file(torch_data->led_dev.dev, &dev_attr_flash_light);
	device_remove_file(torch_data->led_dev.dev, &dev_attr_strobe_err);

	led_classdev_unregister(&torch_data->led_dev);
	led_classdev_unregister(&torch_data->spotlight_dev);

	kfree(torch_data->pdata);
	kfree(torch_data);
	return 0;
}

static const struct i2c_device_id lm3554_id[] = {
	{LM3554_NAME, 0},
	{}
};

static struct i2c_driver lm3554_i2c_driver = {
	.probe = lm3554_probe,
	.remove = lm3554_remove,
	.id_table = lm3554_id,
	.driver = {
		   .name = LM3554_NAME,
		   .owner = THIS_MODULE,
		   },
};

/****************************************************************************/

struct lm3554_cpcap_data {
	struct cpcap_device *cpcap_dev;
};

static void cpcap_lm3554_power(struct cpcap_device *cpcap, int power_val)
{
	int gpio_state = 0;

	if (power_val != 0)
		gpio_state = CPCAP_BIT_GPIO0DRV;
	else
		gpio_state = 0;

	cpcap_regacc_write(cpcap,
			   CPCAP_REG_GPIO0, gpio_state, CPCAP_BIT_GPIO0DRV);
}

static int cpcap_lm3554_probe(struct platform_device *pdev)
{
	int ret;
	struct lm3554_cpcap_data *info;

	pr_info("%s:CPCAP Probe enter\n", __func__);

	if (pdev == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct lm3554_cpcap_data), GFP_KERNEL);
	if (info == NULL) {
		ret = -ENOMEM;
		return ret;
	}

	info->cpcap_dev = pdev->dev.platform_data;
	platform_set_drvdata(pdev, info);

	ret = cpcap_regacc_write(info->cpcap_dev,
				 CPCAP_REG_GPIO0, CPCAP_BIT_GPIO0DIR,
				 CPCAP_BIT_GPIO0DIR);
	if (ret < 0) {
		pr_err("%s: cpcap reg write failed\n", __func__);
		kfree(info);
		return ret;
	}

	/* Toggle only when the device is present */
	cpcap_lm3554_power(info->cpcap_dev, 0);
	mdelay(5);
	cpcap_lm3554_power(info->cpcap_dev, 1);

	ret = i2c_add_driver(&lm3554_i2c_driver);

	pr_info("%s:CPCAP torch probe exit\n", __func__);
	return ret;
}

static int cpcap_lm3554_remove(struct platform_device *pdev)
{
	struct lm3554_cpcap_data *info = platform_get_drvdata(pdev);

	i2c_del_driver(&lm3554_i2c_driver);

	kfree(info);

	return 0;
}

struct platform_driver cpcap_lm3554_driver = {
	.probe = cpcap_lm3554_probe,
	.remove = cpcap_lm3554_remove,
	.driver = {
		   .name = "flash-torch",
		   .owner = THIS_MODULE,
		   },
};

static int __init cpcap_lm3554_init(void)
{
	return cpcap_driver_register(&cpcap_lm3554_driver);
}

static void __exit cpcap_lm3554_exit(void)
{
	platform_driver_unregister(&cpcap_lm3554_driver);
}

module_init(cpcap_lm3554_init);
module_exit(cpcap_lm3554_exit);

/****************************************************************************/

MODULE_DESCRIPTION("Lighting driver for LM3554");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
