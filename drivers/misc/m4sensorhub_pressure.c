/*
 *  Copyright (C) 2012 Motorola, Inc.
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
 *  Adds ability to program periodic interrupts from user space that
 *  can wake the phone out of low power modes.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/m4sensorhub.h>
#include <linux/m4sensorhub/MemMapPressureSensor.h>
#include <linux/m4sensorhub/m4sensorhub_registers.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/m4sensorhub_pressure.h>

#define m4sensorhub_pressure_DRIVER_NAME	"m4sensorhub_pressure"

struct m4sensorhub_pressure_drvdata {
	struct m4sensorhub_data *p_m4sensorhub;
	int samplerate;
	bool enable;
};

#define DATA_SIZE_IN_BITS  (sizeof(struct m4sensorhub_pressure_data) * 8)

/* We are ADC with 1 channel */
static const struct iio_chan_spec m4sensorhub_pressure_channels[] = {
	{
		.type = IIO_PRESSURE,
		/* Channel has a numeric index of 0 */
		.indexed = 1,
		.channel = 0,
		.info_mask_separate =
		/*
		* in_pressure0_raw
		* Raw (unscaled no bias removal etc) measurement
		* from the device.
		*/
		BIT(IIO_CHAN_INFO_RAW),

		.scan_index = 0,
		.scan_type = { /* Description of storage in buffer */
			.sign = 'u', /* unsigned */
			.realbits = DATA_SIZE_IN_BITS,
			.storagebits = DATA_SIZE_IN_BITS,
			.shift = 0, /* zero shift */
		},
	},
};


static void m4_handle_pressure_irq(enum m4sensorhub_irqs int_event,
					void *p_data)
{
	struct iio_dev *p_iio_dev = (struct iio_dev *)p_data;
	struct m4sensorhub_pressure_drvdata *p_priv_data = iio_priv(p_iio_dev);
	struct m4sensorhub_data *p_m4sensorhub = p_priv_data->p_m4sensorhub;
	sPressureData pressure;
	struct m4sensorhub_pressure_data data;

	m4sensorhub_reg_read(p_m4sensorhub,
				M4SH_REG_PRESSURE_PRESSURE,
				(char *)&pressure.pressure);
	m4sensorhub_reg_read(p_m4sensorhub,
				M4SH_REG_PRESSURE_ABSOLUTEALTITUDE,
				(char *)&pressure.absoluteAltitude);

	data.pressure = pressure.pressure;
	data.altitude = pressure.absoluteAltitude;
	data.timestamp = iio_get_time_ns();

	iio_push_to_buffers(p_iio_dev, (unsigned char *)&data);
}


static int m4sensorhub_pressure_driver_initcallback(struct init_calldata *p_arg)
{
	struct iio_dev *p_iio_dev = (struct iio_dev *)(p_arg->p_data);
	struct m4sensorhub_data *p_m4sensorhub = p_arg->p_m4sensorhub_data;
	struct m4sensorhub_pressure_drvdata *p_priv_data = iio_priv(p_iio_dev);
	int ret;

	p_priv_data->p_m4sensorhub = p_m4sensorhub;

	/* register for pressure interrupt from M4 */
	ret = m4sensorhub_irq_register(p_m4sensorhub,
					M4SH_IRQ_PRESSURE_DATA_READY,
					m4_handle_pressure_irq,
					p_iio_dev);

	if (ret < 0)
		pr_err("%s: Failed to register interrupt handler\n", __func__);

	return ret;
}

/* Sysfs interface for enable and setdelay */
/* enable */
static ssize_t m4sensorhub_pressure_store_enable(struct device *p_dev,
			struct device_attribute *p_attr,
			const char *p_buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(p_dev);
	struct iio_dev *p_iio_dev =
						platform_get_drvdata(pdev);
	struct m4sensorhub_pressure_drvdata *p_priv_data = iio_priv(p_iio_dev);
	struct m4sensorhub_data *p_m4sensorhub = p_priv_data->p_m4sensorhub;
	bool requested_state;
	int ret;

	ret = strtobool(p_buf, &requested_state);
	if (ret < 0)
		return ret;

	if (requested_state == false) {
		ret = m4sensorhub_irq_disable(
				p_m4sensorhub,
				M4SH_IRQ_PRESSURE_DATA_READY);

		if (ret < 0) {
			pr_err("%s: failed to disable irq\n", __func__);
			return ret;
		}
	}

	p_priv_data->enable = requested_state;

	return count;
}

static ssize_t m4sensorhub_pressure_show_enable(struct device *p_dev,
				struct device_attribute *p_attr, char *p_buf)
{
	struct platform_device *pdev = to_platform_device(p_dev);
	struct iio_dev *p_iio_dev =
						platform_get_drvdata(pdev);
	struct m4sensorhub_pressure_drvdata *p_priv_data = iio_priv(p_iio_dev);
	int ret = 0;
	if (p_priv_data->enable)
		ret = 1;
	return snprintf(p_buf, PAGE_SIZE, "%d\n", ret);
}

/* setdelay */
static ssize_t m4sensorhub_pressure_store_setdelay(struct device *p_dev,
			struct device_attribute *p_attr,
			const char *p_buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(p_dev);
	struct iio_dev *p_iio_dev =
						platform_get_drvdata(pdev);
	struct m4sensorhub_pressure_drvdata *p_priv_data = iio_priv(p_iio_dev);
	int ret;
	int samplerate;

	ret = kstrtoint(p_buf, 10, &samplerate);
	if (ret < 0)
		return ret;

	if (samplerate < 0) {
		pr_err("%s: negative sample rate, rejecting\n", __func__);
		return -EINVAL;
	}
	if (samplerate != p_priv_data->samplerate) {
		ret = m4sensorhub_reg_write(p_priv_data->p_m4sensorhub,
				M4SH_REG_PRESSURE_SAMPLERATE,
				(char *)&samplerate, m4sh_no_mask);
		if (ret != m4sensorhub_reg_getsize(
				p_priv_data->p_m4sensorhub,
				M4SH_REG_PRESSURE_SAMPLERATE)) {
			pr_err("%s:Unable to set delay\n", __func__);
			return ret;
		}

		p_priv_data->samplerate = samplerate;

		ret = m4sensorhub_irq_enable(
				p_priv_data->p_m4sensorhub,
				M4SH_IRQ_PRESSURE_DATA_READY);
		if (ret < 0) {
			pr_err("%s: failed to disable irq\n", __func__);
			return ret;
		}
	}

	return count;
}

static ssize_t m4sensorhub_pressure_show_setdelay(struct device *p_dev,
				struct device_attribute *p_attr, char *p_buf)
{
	struct platform_device *pdev = to_platform_device(p_dev);
	struct iio_dev *p_iio_dev =
						platform_get_drvdata(pdev);
	struct m4sensorhub_pressure_drvdata *p_priv_data = iio_priv(p_iio_dev);
	return snprintf(p_buf, PAGE_SIZE, "%d\n", p_priv_data->samplerate);
}

static IIO_DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
					m4sensorhub_pressure_show_enable,
					m4sensorhub_pressure_store_enable, 0);
static IIO_DEVICE_ATTR(setdelay, S_IRUGO | S_IWUSR,
					m4sensorhub_pressure_show_setdelay,
					m4sensorhub_pressure_store_setdelay, 0);

#define M4_DEV_ATTR(name) (&iio_dev_attr_##name.dev_attr.attr)

static struct attribute *m4sensorhub_pressure_attributes[] = {
	M4_DEV_ATTR(enable),
	M4_DEV_ATTR(setdelay),
	NULL
};

static const struct attribute_group m4sensorhub_pressure_group = {
	.attrs = m4sensorhub_pressure_attributes,
};

static const struct iio_info m4sensorhub_pressure_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &m4sensorhub_pressure_group,
};

static int m4sensorhub_pressure_setup_buffer(struct iio_dev *p_iio_dev)
{
	struct iio_buffer *p_buffer;
	int ret;
	p_buffer = iio_kfifo_allocate(p_iio_dev);
	if (p_buffer == NULL) {
		pr_err("%s: failed to allocate buffer\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	p_iio_dev->buffer = p_buffer;

    /* need timestamps */
	p_buffer->scan_timestamp = true;
	ret = iio_buffer_register(p_iio_dev, p_iio_dev->channels,
						p_iio_dev->num_channels);

	if (ret < 0) {
		pr_err("%s: failed to register buffer\n", __func__);
		goto err;
	}
	p_buffer->access->set_bytes_per_datum(p_buffer,
				sizeof(struct m4sensorhub_pressure_data));

	ret = 0;
	return ret;
err:
	iio_kfifo_free(p_buffer);

	return ret;
}

static int m4sensorhub_pressure_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct iio_dev *p_iio_dev;
	struct m4sensorhub_pressure_drvdata *p_priv_data;

	p_iio_dev = iio_device_alloc(
			sizeof(struct m4sensorhub_pressure_drvdata));

	if (p_iio_dev == NULL) {
		pr_err("%s: no mem", __func__);
		ret = -ENOMEM;
		goto err;
	}

	p_priv_data = iio_priv(p_iio_dev);
	p_priv_data->samplerate = -1;
	p_priv_data->enable = false;
	p_priv_data->p_m4sensorhub = NULL;

	platform_set_drvdata(pdev, p_iio_dev);

	p_iio_dev->info = &m4sensorhub_pressure_iio_info;
	p_iio_dev->name = m4sensorhub_pressure_DRIVER_NAME;
	p_iio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	p_iio_dev->channels = m4sensorhub_pressure_channels;
	p_iio_dev->num_channels = ARRAY_SIZE(m4sensorhub_pressure_channels);

	/* Register the channel with a buffer */
	ret = m4sensorhub_pressure_setup_buffer(p_iio_dev);
	if (ret < 0) {
		pr_err("%s: can't setup buffer", __func__);
		goto cleanup1;
	}

	ret = iio_device_register(p_iio_dev);
	if (ret < 0) {
		pr_err("%s: iio_register failed", __func__);
		goto cleanup2;
	}

	ret = m4sensorhub_register_initcall(
			m4sensorhub_pressure_driver_initcallback,
			p_iio_dev);
	if (ret < 0) {
		pr_err("%s:Register init failed, ret = %d\n", __func__, ret);
		goto cleanup3;
	}

	return 0;
cleanup3:
	iio_device_unregister(p_iio_dev);
cleanup2:
	iio_kfifo_free(p_iio_dev->buffer);
	iio_buffer_unregister(p_iio_dev);
cleanup1:
	iio_device_free(p_iio_dev);
	platform_set_drvdata(pdev, NULL);
err:
	return ret;
}

static int __exit m4sensorhub_pressure_remove(struct platform_device *pdev)
{
	struct iio_dev *p_iio_dev =
						platform_get_drvdata(pdev);
	struct m4sensorhub_pressure_drvdata *p_priv_data = iio_priv(p_iio_dev);

	m4sensorhub_unregister_initcall(
				m4sensorhub_pressure_driver_initcallback);
	m4sensorhub_irq_unregister(p_priv_data->p_m4sensorhub,
						M4SH_IRQ_PRESSURE_DATA_READY);

	iio_kfifo_free(p_iio_dev->buffer);
	iio_buffer_unregister(p_iio_dev);
	iio_device_unregister(p_iio_dev);
	iio_device_free(p_iio_dev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static void m4sensorhub_pressure_shutdown(struct platform_device *pdev)
{
	return;
}
/* when we suspend, we disable pressure IRQ and set samplerate to -1
when we resume, we wait for the app to re-register and that re-enables
the sensor. This is global M4 strategy, nothing specific for this driver */
#ifdef CONFIG_PM
static int m4sensorhub_pressure_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	return 0;
}

static int m4sensorhub_pressure_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define m4sensorhub_pressure_suspend NULL
#define m4sensorhub_pressure_resume  NULL
#endif

static struct of_device_id m4sensorhub_pressure_match_tbl[] = {
	{ .compatible = "mot,m4sensorhub_pressure" },
	{},
};

static struct platform_driver m4sensorhub_pressure_driver = {
	.probe		= m4sensorhub_pressure_probe,
	.remove		= __exit_p(m4sensorhub_pressure_remove),
	.shutdown	= m4sensorhub_pressure_shutdown,
	.suspend	= m4sensorhub_pressure_suspend,
	.resume		= m4sensorhub_pressure_resume,
	.driver		= {
		.name	= m4sensorhub_pressure_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(m4sensorhub_pressure_match_tbl),
	},
};

module_platform_driver(m4sensorhub_pressure_driver);

MODULE_ALIAS("platform:m4sensorhub_pressure");
MODULE_DESCRIPTION("M4 Sensor Hub Pressure IIO driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
