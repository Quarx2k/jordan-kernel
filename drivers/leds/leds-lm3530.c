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

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/led-lm3530.h>
#include <linux/types.h>
#include <linux/slab.h>

#define GEN_CONFIG_PWM_BIT 0x20

static int pwm_disable_manual = 0;
static int pwm_disable_auto = 0;

int als_resistor_val[16] = {1, 13531, 9011, 5411, 2271, 1946, 1815, 1600, 1138,
				1050, 1011, 941, 759, 719, 700, 667};

struct lux_data {
	int lux_value;
};

struct lm3530_data {
	struct input_dev *idev;
	struct led_classdev led_dev;
	struct i2c_client *client;
	struct work_struct wq;
	struct workqueue_struct *working_queue;
	struct lm3530_platform_data *als_pdata;
	struct early_suspend		early_suspend;
	struct lux_data lux_passed_value[5];
	uint8_t mode;
	uint8_t last_requested_brightness;
	uint8_t zone;
	uint8_t current_divisor;
	uint8_t current_array[8];
	uint8_t led_on;
};

struct lm3530_reg {
	const char *name;
	uint8_t reg;
} lm3530_regs[] = {
	{ "GEN_CONFIG",		   LM3530_GEN_CONFIG },
	{ "ALS_CONFIG",		   LM3530_ALS_CONFIG },
	{ "VERSION_REG",		  LM3530_VERSION_REG },
	{ "BRIGHTNESS_RAMP_RATE", LM3530_BRIGHTNESS_RAMP_RATE },
	{ "ALS_ZONE_REG",		 LM3530_ALS_ZONE_REG },
	{ "ALS_RESISTOR_SELECT",  LM3530_ALS_RESISTOR_SELECT },
	{ "BRIGHTNESS_CTRL_REG",  LM3530_BRIGHTNESS_CTRL_REG },
	{ "ALS_ZB0_REG",		  LM3530_ALS_ZB0_REG },
	{ "ALS_ZB1_REG",		  LM3530_ALS_ZB1_REG },
	{ "ALS_ZB2_REG",		  LM3530_ALS_ZB2_REG },
	{ "ALS_ZB3_REG",		  LM3530_ALS_ZB3_REG },
	{ "ALS_Z0T_REG",		  LM3530_ALS_Z0T_REG },
	{ "ALS_Z1T_REG",		  LM3530_ALS_Z1T_REG },
	{ "ALS_Z2T_REG",		  LM3530_ALS_Z2T_REG },
	{ "ALS_Z3T_REG",		  LM3530_ALS_Z3T_REG },
	{ "ALS_Z4T_REG",		  LM3530_ALS_Z4T_REG },
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3530_early_suspend(struct early_suspend *handler);
static void lm3530_late_resume(struct early_suspend *handler);
#endif

static uint32_t lm3530_debug;
module_param_named(als_debug, lm3530_debug, uint, 0664);

static int lm3530_read_reg(struct lm3530_data *als_data, uint8_t reg,
		   uint8_t *value)
{
	int error = 0;
	int i = 0;
	uint8_t dest_buffer;

	if (!value) {
		pr_err("%s: invalid value pointer\n", __func__);
		return -EINVAL;
	}
	do {
		dest_buffer = reg;
		error = i2c_master_send(als_data->client, &dest_buffer, 1);
		if (error == 1) {
			error = i2c_master_recv(als_data->client,
				&dest_buffer, LD_LM3530_ALLOWED_R_BYTES);
		}
		if (error != LD_LM3530_ALLOWED_R_BYTES) {
			pr_err("%s: read[%i] failed: %d\n", __func__, i, error);
			msleep_interruptible(LD_LM3530_I2C_RETRY_DELAY);
		}
	} while ((error != LD_LM3530_ALLOWED_R_BYTES) &&
			((++i) < LD_LM3530_MAX_RW_RETRIES));

	if (error == LD_LM3530_ALLOWED_R_BYTES) {
		error = 0;
		*value = dest_buffer;
	}

	return error;
}

static int lm3530_write_reg(struct lm3530_data *als_data,
							uint8_t reg,
							uint8_t value)
{
	uint8_t buf[LD_LM3530_ALLOWED_W_BYTES] = { reg, value };
	int bytes;
	int i = 0;

	do {
		bytes = i2c_master_send(als_data->client, buf,
					LD_LM3530_ALLOWED_W_BYTES);

		if (bytes != LD_LM3530_ALLOWED_W_BYTES) {
			pr_err("%s: write %d failed: %d\n", __func__, i, bytes);
			msleep_interruptible(LD_LM3530_I2C_RETRY_DELAY);
		}
	} while ((bytes != (LD_LM3530_ALLOWED_W_BYTES))
		 && ((++i) < LD_LM3530_MAX_RW_RETRIES));

	if (bytes != LD_LM3530_ALLOWED_W_BYTES) {
		pr_err("%s: i2c_master_send error\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int ld_lm3530_init_registers(struct lm3530_data *als_data)
{
	if (lm3530_write_reg(als_data, LM3530_ALS_CONFIG,
				 als_data->als_pdata->als_config) ||
		lm3530_write_reg(als_data, LM3530_BRIGHTNESS_RAMP_RATE,
				 als_data->als_pdata->brightness_ramp) ||
		lm3530_write_reg(als_data, LM3530_ALS_RESISTOR_SELECT,
				 als_data->als_pdata->als_resistor_sel) ||
		lm3530_write_reg(als_data, LM3530_ALS_ZB0_REG,
				 als_data->als_pdata->zone_boundary_0) ||
		lm3530_write_reg(als_data, LM3530_ALS_ZB1_REG,
				 als_data->als_pdata->zone_boundary_1) ||
		lm3530_write_reg(als_data, LM3530_ALS_ZB2_REG,
				 als_data->als_pdata->zone_boundary_2) ||
		lm3530_write_reg(als_data, LM3530_ALS_ZB3_REG,
				 als_data->als_pdata->zone_boundary_3) ||
		lm3530_write_reg(als_data, LM3530_ALS_Z0T_REG,
				 als_data->als_pdata->zone_target_0) ||
		lm3530_write_reg(als_data, LM3530_ALS_Z1T_REG,
				 als_data->als_pdata->zone_target_1) ||
		lm3530_write_reg(als_data, LM3530_ALS_Z2T_REG,
				 als_data->als_pdata->zone_target_2) ||
		lm3530_write_reg(als_data, LM3530_ALS_Z3T_REG,
				 als_data->als_pdata->zone_target_3) ||
		lm3530_write_reg(als_data, LM3530_ALS_Z4T_REG,
				 als_data->als_pdata->zone_target_4)) {
		pr_err("%s:Register initialization failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static void ld_lm3530_brightness_set(struct led_classdev *led_cdev,
					 enum led_brightness value)
{
	int brightness = 0;
	int error = 0;
	int old_led_on;
	struct lm3530_data *als_data =
		container_of(led_cdev, struct lm3530_data, led_dev);

	if (als_data->mode == AUTOMATIC)
		brightness = als_data->als_pdata->gen_config;
	else
		brightness = als_data->als_pdata->manual_current;

	if (lm3530_debug)
		pr_info("%s: value = 0x%x brightness = 0x%x\n",
			 __func__, value, brightness);

	if (value == LED_OFF) {
		als_data->led_on = 0;
		brightness &= LD_LM3530_LAST_BRIGHTNESS_MASK;
		if (lm3530_write_reg(als_data, LM3530_GEN_CONFIG, brightness)) {
			pr_err("%s:writing failed while setting brightness:%d\n",
				__func__, error);
			return;
		}
	} else {
		brightness |= 0x01;
		if (lm3530_write_reg(als_data, LM3530_GEN_CONFIG, brightness)) {
			pr_err("%s:writing failed while setting brightness:%d\n",
				__func__, error);
			return;
		}
		if (lm3530_write_reg(als_data, LM3530_BRIGHTNESS_CTRL_REG,
			value / 2)) {
				pr_err("%s:Failed to set brightness:%d\n",
				__func__, error);
			return;
		}
		als_data->last_requested_brightness = value;
		old_led_on = als_data->led_on;
		als_data->led_on = 1;

		if (als_data->als_pdata->als_enabled == 1 && old_led_on == 0) {
			disable_irq(als_data->client->irq);
			queue_work(als_data->working_queue, &als_data->wq);
		}
	}
}

static ssize_t ld_lm3530_als_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev->parent, struct i2c_client,
						 dev);
	struct lm3530_data *als_data = i2c_get_clientdata(client);

	return sprintf(buf, "%u\n", als_data->mode);
}

static ssize_t ld_lm3530_als_store(struct device *dev, struct device_attribute
				   *attr, const char *buf, size_t size)
{
	int error = 0;
	struct i2c_client *client = container_of(dev->parent, struct i2c_client,
						 dev);
	struct lm3530_data *als_data = i2c_get_clientdata(client);
	int config;
	unsigned long mode_value;
	uint8_t brightness = 0;

	if (als_data->als_pdata->als_enabled == 0)
		return MANUAL;

	error = strict_strtoul(buf, 10, &mode_value);

	if (error < 0)
		return -1;
	if (mode_value != MANUAL
			&& mode_value != AUTOMATIC
			&& mode_value != MANUAL_SENSOR)
		return -1;

	if (mode_value == AUTOMATIC)
		brightness = als_data->als_pdata->gen_config;
	else
		brightness = als_data->als_pdata->manual_current;

	if (mode_value == AUTOMATIC) {
		ld_lm3530_init_registers(als_data);
		error = lm3530_write_reg(als_data, LM3530_GEN_CONFIG,
					 brightness);
		if (error) {
			pr_err("%s:Initialize Gen Config Reg failed %d\n",
				   __func__, error);
			als_data->mode = -1;
			return -1;
		}
		als_data->mode = AUTOMATIC;
	} else {
		als_data->mode = mode_value;
		config = als_data->als_pdata->manual_als_config;
		if (mode_value != MANUAL)
			config |= LM3530_SENSOR_ENABLE;
		error = lm3530_write_reg(als_data, LM3530_ALS_CONFIG, config);
		if (error) {
			pr_err("%s:Failed to set manual mode:%d\n",
				   __func__, error);
			return -1;
		}

		if (lm3530_write_reg(als_data, LM3530_GEN_CONFIG, brightness)) {
			pr_err("%s:writing failed while setting brightness:%d\n",
				__func__, error);
			return -1;
		}

		config = (mode_value == MANUAL ? 0 : als_data->als_pdata->als_resistor_sel);
		lm3530_write_reg(als_data, LM3530_ALS_RESISTOR_SELECT, config);

		error = lm3530_write_reg(als_data,
					 LM3530_BRIGHTNESS_CTRL_REG,
					 als_data->last_requested_brightness /
					 2);
		if (error) {
			pr_err("%s:Failed to set brightness:%d\n",
				   __func__, error);
			return -1;
		}
	}

	if (mode_value != MANUAL) {
		disable_irq(als_data->client->irq);
		queue_work(als_data->working_queue, &als_data->wq);
	}

	return als_data->mode;
}

static DEVICE_ATTR(als, 0644, ld_lm3530_als_show, ld_lm3530_als_store);

static ssize_t ld_lm3530_pwm_store(struct device *dev, struct device_attribute
				   *attr, const char *buf, size_t size)
{
	int error = 0;
	unsigned long pwm_value;
	uint8_t pwm_val;
	uint8_t temp_gen_config;
	struct i2c_client *client = container_of(dev->parent, struct i2c_client,
						 dev);
	struct lm3530_data *als_data = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &pwm_value);
	if (error < 0)
		return -1;

	if (als_data->mode == MANUAL)
		temp_gen_config = als_data->als_pdata->manual_current;
	else
		temp_gen_config = als_data->als_pdata->gen_config;

	if (pwm_value >= 1)
		pwm_val = temp_gen_config | 0x20;
	else
		pwm_val = temp_gen_config & 0xdf;

	if (lm3530_write_reg(als_data, LM3530_GEN_CONFIG, pwm_val)) {
		pr_err("%s:writing failed while setting pwm mode:%d\n",
			   __func__, error);
		return -1;
	}

	return pwm_value;
}
static DEVICE_ATTR(pwm_mode, 0644, NULL, ld_lm3530_pwm_store);

irqreturn_t ld_lm3530_irq_handler(int irq, void *dev)
{
	struct lm3530_data *als_data = dev;

	disable_irq_nosync(als_data->client->irq);
	queue_work(als_data->working_queue, &als_data->wq);

	return IRQ_HANDLED;
}

static ssize_t ld_lm3530_registers_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev->parent, struct i2c_client,
						 dev);
	struct lm3530_data *als_data = i2c_get_clientdata(client);
	unsigned i, n, reg_count;
	uint8_t value = 0;

	reg_count = sizeof(lm3530_regs) / sizeof(lm3530_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		lm3530_read_reg(als_data, lm3530_regs[i].reg, &value);
		n += scnprintf(buf + n, PAGE_SIZE - n,
				   "%-20s = 0x%02X\n",
				   lm3530_regs[i].name,
				   value);
	}

	return n;
}

static ssize_t ld_lm3530_registers_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev->parent, struct i2c_client,
						 dev);
	struct lm3530_data *als_data = i2c_get_clientdata(client);
	unsigned i, reg_count, value;
	int error;
	char name[30];

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%29s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}
	name[sizeof(name)-1] = '\0';

	reg_count = sizeof(lm3530_regs) / sizeof(lm3530_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, lm3530_regs[i].name)) {
			error = lm3530_write_reg(als_data,
				lm3530_regs[i].reg,
				value);
			if (error) {
				pr_err("%s:Failed to write register %s\n",
					__func__, name);
				return -1;
			}
			return count;
		}
	}

	pr_err("%s:no such register %s\n", __func__, name);
	return -1;
}

static DEVICE_ATTR(registers, 0644, ld_lm3530_registers_show,
		ld_lm3530_registers_store);

void ld_lm3530_work_queue(struct work_struct *work)
{
	int ret;
	int light_value;
	struct lm3530_data *als_data =
		container_of(work, struct lm3530_data, wq);

	ret = lm3530_read_reg(als_data,
				  LM3530_ALS_ZONE_REG, &als_data->zone);
	if (ret != 0) {
		pr_err("%s:Unable to read ALS Zone read back: %d\n",
			   __func__, ret);

		enable_irq(als_data->client->irq);

		return;
	}
	/* Don't allow the data to be sent if the LED is supposed to
	be off */
	if (als_data->led_on == 0) {
		if (lm3530_debug)
			pr_info("%s:Skipping this interrupt\n", __func__);
		enable_irq(als_data->client->irq);
		return;
	}

	als_data->zone = als_data->zone & LM3530_ALS_READ_MASK;
	if (lm3530_debug)
		pr_info("%s:ALS Zone read back: %d\n",
			   __func__, als_data->zone);

	light_value = als_data->zone *  (als_data->current_divisor - 1);

	/* Need to indicate a zone 0 but this would indicate it is off
	so send up a low value and not a 0 */
	if (light_value == 0)
		light_value = 10;

	if (lm3530_debug) {
		pr_err("%s:Modified ALS Zone being sent: %d\n",
			   __func__, light_value);
		pr_err("%s:Lux value: %i\n", __func__,
			als_data->lux_passed_value[als_data->zone].lux_value);
	}

	input_event(als_data->idev, EV_MSC, MSC_RAW, light_value);
	input_event(als_data->idev, EV_LED, LED_MISC,
		als_data->lux_passed_value[als_data->zone].lux_value);
	input_sync(als_data->idev);

	enable_irq(als_data->client->irq);
}

static int convert_to_lux(struct lm3530_data *als_data, int zone_value)
{
	int mv_conv;
	int current_conv;
	int divisor;

	if (zone_value == 0)
		return 0x40;

	divisor = als_data->als_pdata->lens_loss_coeff;
	if (divisor == 0)
		divisor = 1;

	mv_conv = ((zone_value * 1000) / 255);
	if ((als_data->als_pdata->als_resistor_sel & 0xF0) == 0)
		current_conv = (mv_conv * 1000) /
		als_resistor_val[als_data->als_pdata->als_resistor_sel];
	else
		current_conv = (mv_conv * 1000) /
		als_resistor_val[(als_data->als_pdata->als_resistor_sel & 0xF0) >> 4];

	return (current_conv * 100) / divisor;
}

static void ld_lm3530_lux_conv(struct lm3530_data *als_data)
{
	int zone_boundary;

	als_data->lux_passed_value[0].lux_value = 10;

	zone_boundary = als_data->als_pdata->zone_boundary_0;
	als_data->lux_passed_value[1].lux_value =
		convert_to_lux(als_data, zone_boundary);

	zone_boundary = als_data->als_pdata->zone_boundary_1;
	als_data->lux_passed_value[2].lux_value =
		convert_to_lux(als_data, zone_boundary);

	zone_boundary = als_data->als_pdata->zone_boundary_2;
	als_data->lux_passed_value[3].lux_value =
		convert_to_lux(als_data, zone_boundary);

	zone_boundary = als_data->als_pdata->zone_boundary_3;
	als_data->lux_passed_value[4].lux_value =
		convert_to_lux(als_data, zone_boundary);
}

static int ld_lm3530_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lm3530_platform_data *pdata = client->dev.platform_data;
	struct lm3530_data *als_data;
	int error = 0;
	int i = 0;

	if (pdata == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;
	} else if (!client->irq) {
		pr_err("%s: polling mode currently not supported\n", __func__);
		return -ENODEV;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s:I2C_FUNC_I2C not supported\n", __func__);
		return -ENODEV;
	}

	als_data = kzalloc(sizeof(struct lm3530_data), GFP_KERNEL);
	if (als_data == NULL) {
		error = -ENOMEM;
		goto err_alloc_data_failed;
	}

	als_data->client = client;
	als_data->als_pdata = pdata;
	als_data->mode = MANUAL;
	als_data->zone = 3;

	memset(als_data->current_array, 0, ARRAY_SIZE(als_data->current_array));

	if ((pdata->upper_curr_sel > 7) ||
		(pdata->lower_curr_sel > pdata->upper_curr_sel)) {
		pr_err("%s: Incorrect current select values\n", __func__);
		error = -ENODEV;
		goto error_invalid_current_select;
	}
	/* Determine the value to divide the brightness value passed in */
	als_data->current_divisor =
		(LM3530_MAX_LED_VALUE /
		 ((pdata->upper_curr_sel - pdata->lower_curr_sel) + 1)) + 1;
	if (als_data->current_divisor <= 0) {
		pr_err("%s: Incorrect divisor for current\n", __func__);
		error = -ENODEV;
		goto error_invalid_current_select;
	}

	/* Populate the table with the current select values */
	for (i = 0; i <= (pdata->upper_curr_sel - pdata->lower_curr_sel); i++)
		als_data->current_array[i] = pdata->lower_curr_sel + i;

	ld_lm3530_lux_conv(als_data);

	if (als_data->als_pdata->als_enabled == 1) {
		als_data->idev = input_allocate_device();
		if (!als_data->idev) {
			error = -ENOMEM;
			pr_err("%s: input device allocate failed: %d\n",
						__func__, error);
			goto error_input_allocate_failed;
		}

		als_data->idev->name =  LD_LM3530_NAME;
		input_set_capability(als_data->idev, EV_MSC, MSC_RAW);
		input_set_capability(als_data->idev, EV_LED, LED_MISC);
	}

	als_data->led_dev.name = LD_LM3530_LED_DEV;
	als_data->led_dev.brightness_set = ld_lm3530_brightness_set;
	als_data->led_on = 1;

	als_data->last_requested_brightness = pdata->power_up_gen_config;

	if (als_data->als_pdata->als_enabled == 1) {
		als_data->working_queue =
				create_singlethread_workqueue("als_wq");
		if (!als_data->working_queue) {
			pr_err("%s: Cannot create work queue\n", __func__);
			error = -ENOMEM;
			goto error_create_wq_failed;
		}
	}

	INIT_WORK(&als_data->wq, ld_lm3530_work_queue);

	if (als_data->als_pdata->als_enabled == 1) {
		error = request_irq(als_data->client->irq,
			ld_lm3530_irq_handler,
			IRQF_TRIGGER_FALLING,
			LD_LM3530_NAME, als_data);
		if (error != 0) {
			pr_err("%s: irq request failed: %d\n", __func__, error);
			error = -ENODEV;
			goto err_req_irq_failed;
		}
	}

	i2c_set_clientdata(client, als_data);

	if (als_data->als_pdata->als_enabled == 1) {
		error = input_register_device(als_data->idev);
		if (error) {
			pr_err("%s: input device register failed:%d\n",
						__func__, error);
			goto error_input_register_failed;
		}
	}

	error = ld_lm3530_init_registers(als_data);
	if (error < 0) {
		pr_err("%s: Register Initialization failed: %d\n",
			   __func__, error);
		error = -ENODEV;
		goto err_reg_init_failed;
	}

	error = lm3530_write_reg(als_data, LM3530_GEN_CONFIG,
				 pdata->power_up_gen_config);
	if (error) {
		pr_err("%s:Initialize Gen Config Reg failed %d\n",
			   __func__, error);
		error = -ENODEV;
		goto err_reg_init_failed;
	}

	error = led_classdev_register((struct device *)
					  &client->dev, &als_data->led_dev);
	if (error < 0) {
		pr_err("%s: Register led class failed: %d\n", __func__, error);
		error = -ENODEV;
		goto err_class_reg_failed;
	}

	error = device_create_file(als_data->led_dev.dev, &dev_attr_als);
	if (error < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, error);
		error = -ENODEV;
		goto err_create_file_als_failed;
	}

	error = device_create_file(als_data->led_dev.dev, &dev_attr_pwm_mode);
	if (error < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, error);
		error = -ENODEV;
		goto err_create_pwm_file_failed;
	}

	error = device_create_file(als_data->led_dev.dev, &dev_attr_registers);
	if (error < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, error);
		error = -ENODEV;
		goto err_create_registers_file_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	als_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	als_data->early_suspend.suspend = lm3530_early_suspend;
	als_data->early_suspend.resume = lm3530_late_resume;
	register_early_suspend(&als_data->early_suspend);
#endif
	if (als_data->als_pdata->als_enabled == 1) {
		disable_irq(als_data->client->irq);
		queue_work(als_data->working_queue, &als_data->wq);
	}

	return 0;

err_create_registers_file_failed:
	device_remove_file(als_data->led_dev.dev, &dev_attr_pwm_mode);
err_create_pwm_file_failed:
	device_remove_file(als_data->led_dev.dev, &dev_attr_als);
err_create_file_als_failed:
	led_classdev_unregister(&als_data->led_dev);
err_class_reg_failed:
err_reg_init_failed:
	if (als_data->als_pdata->als_enabled == 1) {
		input_unregister_device(als_data->idev);
		als_data->idev = NULL;
	}
error_input_register_failed:
	if (als_data->als_pdata->als_enabled == 1)
		free_irq(als_data->client->irq, als_data);
err_req_irq_failed:
	if (als_data->als_pdata->als_enabled == 1)
		destroy_workqueue(als_data->working_queue);
error_create_wq_failed:
	if (als_data->als_pdata->als_enabled == 1)
		input_free_device(als_data->idev);
error_input_allocate_failed:
error_invalid_current_select:
	kfree(als_data);
err_alloc_data_failed:
	return error;
}

static int ld_lm3530_remove(struct i2c_client *client)
{
	struct lm3530_data *als_data = i2c_get_clientdata(client);
	device_remove_file(als_data->led_dev.dev, &dev_attr_als);
	device_remove_file(als_data->led_dev.dev, &dev_attr_pwm_mode);
	device_remove_file(als_data->led_dev.dev, &dev_attr_registers);
	led_classdev_unregister(&als_data->led_dev);
	free_irq(als_data->client->irq, als_data);
	if (als_data->working_queue)
		destroy_workqueue(als_data->working_queue);
	kfree(als_data);
	return 0;
}

static int lm3530_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lm3530_data *als_data = i2c_get_clientdata(client);
	if (lm3530_debug)
		pr_info("%s: Suspending\n", __func__);

	if (als_data->als_pdata->gen_config & GEN_CONFIG_PWM_BIT) {
		pwm_disable_auto = 1;
		als_data->als_pdata->gen_config &= ~GEN_CONFIG_PWM_BIT;
	}
	if (als_data->als_pdata->manual_current & GEN_CONFIG_PWM_BIT) {
		pwm_disable_manual = 1;
		als_data->als_pdata->manual_current &= ~GEN_CONFIG_PWM_BIT;
	}

	if (als_data->als_pdata->als_enabled == 1) {
		int ret;

		disable_irq_nosync(als_data->client->irq);
		ret = cancel_work_sync(&als_data->wq);
		if (ret) {
			pr_info("%s: Not Suspending\n", __func__);
			enable_irq(als_data->client->irq);
			return -EBUSY;
		}
	}

	return 0;
}

static int lm3530_resume(struct i2c_client *client)
{
	struct lm3530_data *als_data = i2c_get_clientdata(client);

	if (lm3530_debug)
		pr_info("%s: Resuming\n", __func__);

	if (pwm_disable_auto) {
		pwm_disable_auto = 0;
		als_data->als_pdata->gen_config |= GEN_CONFIG_PWM_BIT;
	}
	if (pwm_disable_manual) {
		pwm_disable_manual = 0;
		als_data->als_pdata->manual_current |= GEN_CONFIG_PWM_BIT;
	}

	if (als_data->als_pdata->als_enabled == 1) {
		/* Work around a HW issue that the HW will not generate an
		interrupt when enabled */
		queue_work(als_data->working_queue, &als_data->wq);
	}
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3530_early_suspend(struct early_suspend *handler)
{
	struct lm3530_data *als_data;

	als_data = container_of(handler, struct lm3530_data, early_suspend);
	lm3530_suspend(als_data->client, PMSG_SUSPEND);
}

static void lm3530_late_resume(struct early_suspend *handler)
{
	struct lm3530_data *als_data;

	als_data = container_of(handler, struct lm3530_data, early_suspend);
	lm3530_resume(als_data->client);
}
#endif

static const struct i2c_device_id lm3530_id[] = {
	{LD_LM3530_NAME, 0},
	{}
};

static struct i2c_driver ld_lm3530_i2c_driver = {
	.probe = ld_lm3530_probe,
	.remove = ld_lm3530_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= lm3530_suspend,
	.resume		= lm3530_resume,
#endif
	.id_table = lm3530_id,
	.driver = {
		   .name = LD_LM3530_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init ld_lm3530_init(void)
{
	return i2c_add_driver(&ld_lm3530_i2c_driver);
}

static void __exit ld_lm3530_exit(void)
{
	i2c_del_driver(&ld_lm3530_i2c_driver);

}

module_init(ld_lm3530_init);
module_exit(ld_lm3530_exit);

MODULE_DESCRIPTION("Lighting driver for LM3530");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
