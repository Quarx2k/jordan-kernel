/*
 *  Copyright (C) 2014 Motorola, Inc.
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
#include <linux/fs.h>
#include <linux/m4sensorhub.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/m4sensorhub/m4sensorhub_heartrate.h>

#define m4hrt_err(format, args...)  KDEBUG(M4SH_ERROR, format, ## args)

#define M4HRT_IRQ_ENABLED_BIT       0

struct m4hrt_driver_data {
	struct platform_device      *pdev;
	struct m4sensorhub_data     *m4;
	struct mutex                mutex; /* controls driver entry points */
	struct iio_dev              *iio;

	uint16_t        heartrate;
	int16_t         samplerate;

	uint16_t        status;
};


static void m4hrt_isr(enum m4sensorhub_irqs int_event, void *handle)
{
	int err = 0;
	struct m4hrt_driver_data *dd = handle;
	int size = 0;
	struct m4sensorhub_heartrate_iio_data databuf;

	mutex_lock(&(dd->mutex));

	size = m4sensorhub_reg_getsize(dd->m4, M4SH_REG_HEARTRATE_HEARTRATE);
	if (size < 0) {
		m4hrt_err("%s: Reading from invalid register %d.\n",
			  __func__, size);
		err = size;
		goto m4hrt_isr_fail;
	}

	err = m4sensorhub_reg_read(dd->m4, M4SH_REG_HEARTRATE_HEARTRATE,
		(char *)&(databuf.heartrate));
	if (err < 0) {
		m4hrt_err("%s: Failed to read heartrate data.\n", __func__);
		goto m4hrt_isr_fail;
	} else if (err != size) {
		m4hrt_err("%s: Read %d bytes instead of %d.\n",
			  __func__, err, size);
		goto m4hrt_isr_fail;
	}

	dd->heartrate = databuf.heartrate;
	databuf.timestamp = iio_get_time_ns();
	iio_push_to_buffers(dd->iio, (unsigned char *)&databuf);

m4hrt_isr_fail:
	if (err < 0)
		m4hrt_err("%s: Failed with error code %d.\n", __func__, err);

	mutex_unlock(&(dd->mutex));

	return;
}

static int m4hrt_set_samplerate(struct m4hrt_driver_data *dd, int16_t rate)
{
	int err = 0;
	int size = 0;

	if (rate == dd->samplerate)
		goto m4hrt_set_samplerate_fail;

	size = m4sensorhub_reg_getsize(dd->m4,
		M4SH_REG_HEARTRATESENSOR_SAMPLERATE);
	if (size < 0) {
		m4hrt_err("%s: Writing to invalid register %d.\n",
			  __func__, size);
		err = size;
		goto m4hrt_set_samplerate_fail;
	}

	err = m4sensorhub_reg_write(dd->m4, M4SH_REG_HEARTRATESENSOR_SAMPLERATE,
		(char *)&rate, m4sh_no_mask);
	if (err < 0) {
		m4hrt_err("%s: Failed to set sample rate.\n", __func__);
		goto m4hrt_set_samplerate_fail;
	} else if (err != size) {
		m4hrt_err("%s:  Wrote %d bytes instead of %d.\n",
			  __func__, err, size);
		goto m4hrt_set_samplerate_fail;
	}

	dd->samplerate = rate;

m4hrt_set_samplerate_fail:
	return err;
}

static ssize_t m4hrt_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iio_dev *iio = platform_get_drvdata(pdev);
	struct m4hrt_driver_data *dd = iio_priv(iio);
	ssize_t size = 0;

	mutex_lock(&(dd->mutex));

	if (dd->status & (1 << M4HRT_IRQ_ENABLED_BIT))
		size = snprintf(buf, PAGE_SIZE, "Sensor is ENABLED.\n");
	else
		size = snprintf(buf, PAGE_SIZE, "Sensor is DISABLED.\n");

	mutex_unlock(&(dd->mutex));
	return size;
}
static ssize_t m4hrt_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct iio_dev *iio = platform_get_drvdata(pdev);
	struct m4hrt_driver_data *dd = iio_priv(iio);
	int value = 0;

	mutex_lock(&(dd->mutex));

	err = kstrtoint(buf, 10, &value);
	if (err < 0) {
		m4hrt_err("%s: Failed to convert value.\n", __func__);
		goto m4hrt_enable_store_exit;
	}

	switch (value) {
	case 0:
		if (dd->status & (1 << M4HRT_IRQ_ENABLED_BIT)) {
			err = m4sensorhub_irq_disable(dd->m4,
				M4SH_IRQ_HEARTRATESENSOR_DATA_READY);
			if (err < 0) {
				m4hrt_err("%s: Failed to disable interrupt.\n",
					  __func__);
				goto m4hrt_enable_store_exit;
			}
			dd->status = dd->status & ~(1 << M4HRT_IRQ_ENABLED_BIT);
		}
		break;

	case 1:
		if (!(dd->status & (1 << M4HRT_IRQ_ENABLED_BIT))) {
			err = m4sensorhub_irq_enable(dd->m4,
				M4SH_IRQ_HEARTRATESENSOR_DATA_READY);
			if (err < 0) {
				m4hrt_err("%s: Failed to enable interrupt.\n",
					  __func__);
				goto m4hrt_enable_store_exit;
			}
			dd->status = dd->status | (1 << M4HRT_IRQ_ENABLED_BIT);
		}
		break;

	default:
		m4hrt_err("%s: Invalid value %d passed.\n", __func__, value);
		err = -EINVAL;
		goto m4hrt_enable_store_exit;
	}

m4hrt_enable_store_exit:
	if (err < 0) {
		m4hrt_err("%s: Failed with error code %d.\n", __func__, err);
		size = err;
	}

	mutex_unlock(&(dd->mutex));

	return size;
}
static IIO_DEVICE_ATTR(enable, S_IRUSR | S_IWUSR,
		m4hrt_enable_show, m4hrt_enable_store, 0);

static ssize_t m4hrt_setrate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iio_dev *iio = platform_get_drvdata(pdev);
	struct m4hrt_driver_data *dd = iio_priv(iio);
	ssize_t size = 0;

	mutex_lock(&(dd->mutex));
	size = snprintf(buf, PAGE_SIZE, "Current rate: %hd\n", dd->samplerate);
	mutex_unlock(&(dd->mutex));
	return size;
}
static ssize_t m4hrt_setrate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int err = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct iio_dev *iio = platform_get_drvdata(pdev);
	struct m4hrt_driver_data *dd = iio_priv(iio);
	int value = 0;

	mutex_lock(&(dd->mutex));

	err = kstrtoint(buf, 10, &value);
	if (err < 0) {
		m4hrt_err("%s: Failed to convert value.\n", __func__);
		goto m4hrt_enable_store_exit;
	}

	if ((value < -32768) || (value > 32767)) {
		m4hrt_err("%s: Value of %d is outside range of int16_t.\n",
			  __func__, value);
		err = -EOVERFLOW;
		goto m4hrt_enable_store_exit;
	}

	err = m4hrt_set_samplerate(dd, value);
	if (err < 0) {
		m4hrt_err("%s: Failed to set sample rate.\n", __func__);
		goto m4hrt_enable_store_exit;
	}

m4hrt_enable_store_exit:
	if (err < 0) {
		m4hrt_err("%s: Failed with error code %d.\n", __func__, err);
		size = err;
	}

	mutex_unlock(&(dd->mutex));

	return size;
}
static IIO_DEVICE_ATTR(setrate, S_IRUSR | S_IWUSR,
		m4hrt_setrate_show, m4hrt_setrate_store, 0);

static ssize_t m4hrt_heartrate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iio_dev *iio = platform_get_drvdata(pdev);
	struct m4hrt_driver_data *dd = iio_priv(iio);
	ssize_t size = 0;

	mutex_lock(&(dd->mutex));
	size = snprintf(buf, PAGE_SIZE, "Current heartrate: %d\n",
		dd->heartrate);
	mutex_unlock(&(dd->mutex));
	return size;
}
static IIO_DEVICE_ATTR(heartrate, S_IRUGO, m4hrt_heartrate_show, NULL, 0);

static struct attribute *m4hrt_iio_attributes[] = {
	&iio_dev_attr_enable.dev_attr.attr,
	&iio_dev_attr_setrate.dev_attr.attr,
	&iio_dev_attr_heartrate.dev_attr.attr,
	NULL,
};

static const struct attribute_group m4hrt_iio_attr_group = {
	.attrs = m4hrt_iio_attributes,
};

static const struct iio_info m4hrt_iio_info = {
	.driver_module = THIS_MODULE,
	.attrs = &m4hrt_iio_attr_group,
};

static const struct iio_chan_spec m4hrt_iio_channels[] = {
	{
		.type = IIO_HEARTRATE,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = M4HRT_DATA_STRUCT_SIZE_BITS,
			.storagebits = M4HRT_DATA_STRUCT_SIZE_BITS,
			.shift = 0,
		},
	},
};

static void m4hrt_remove_iiodev(struct m4hrt_driver_data *dd)
{
	iio_kfifo_free(dd->iio->buffer);
	iio_buffer_unregister(dd->iio);
	iio_device_unregister(dd->iio);
	mutex_destroy(&(dd->mutex));
	iio_device_free(dd->iio); /* dd is freed here */
	return;
}

static int m4hrt_create_iiodev(struct m4hrt_driver_data *dd)
{
	int err = 0;

	dd->iio->name = M4HRT_DRIVER_NAME;
	dd->iio->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	dd->iio->num_channels = 1;
	dd->iio->info = &m4hrt_iio_info;
	dd->iio->channels = m4hrt_iio_channels;

	dd->iio->buffer = iio_kfifo_allocate(dd->iio);
	if (dd->iio->buffer == NULL) {
		m4hrt_err("%s: Failed to allocate IIO buffer.\n", __func__);
		err = -ENOMEM;
		goto m4hrt_create_iiodev_kfifo_fail;
	}

	dd->iio->buffer->scan_timestamp = true;
	dd->iio->buffer->access->set_bytes_per_datum(dd->iio->buffer,
		sizeof(struct m4sensorhub_heartrate_iio_data));
	err = iio_buffer_register(dd->iio, dd->iio->channels,
		dd->iio->num_channels);
	if (err < 0) {
		m4hrt_err("%s: Failed to register IIO buffer.\n", __func__);
		goto m4hrt_create_iiodev_buffer_fail;
	}

	err = iio_device_register(dd->iio);
	if (err < 0) {
		m4hrt_err("%s: Failed to register IIO device.\n", __func__);
		goto m4hrt_create_iiodev_iioreg_fail;
	}

	goto m4hrt_create_iiodev_exit;

m4hrt_create_iiodev_iioreg_fail:
	iio_buffer_unregister(dd->iio);
m4hrt_create_iiodev_buffer_fail:
	iio_kfifo_free(dd->iio->buffer);
m4hrt_create_iiodev_kfifo_fail:
	iio_device_free(dd->iio);
	dd->iio = NULL;
m4hrt_create_iiodev_exit:
	return err;
}

static int m4hrt_driver_init(struct init_calldata *p_arg)
{
	struct m4hrt_driver_data *dd = p_arg->p_data;
	int err = 0;

	mutex_lock(&(dd->mutex));

	dd->m4 = p_arg->p_m4sensorhub_data;
	if (dd->m4 == NULL) {
		m4hrt_err("%s: M4 sensor data is NULL.\n", __func__);
		err = -ENODATA;
		goto m4hrt_driver_init_fail;
	}

	err = m4hrt_create_iiodev(dd);
	if (err < 0) {
		m4hrt_err("%s: Failed to create M4 event device.\n", __func__);
		goto m4hrt_driver_init_fail;
	}

	err = m4sensorhub_irq_register(dd->m4,
		M4SH_IRQ_HEARTRATESENSOR_DATA_READY, m4hrt_isr, dd);
	if (err < 0) {
		m4hrt_err("%s: Failed to register M4 IRQ.\n", __func__);
		goto m4hrt_driver_init_irq_fail;
	}

	mutex_unlock(&(dd->mutex));

	goto m4hrt_driver_init_exit;

m4hrt_driver_init_irq_fail:
	m4hrt_remove_iiodev(dd); /* dd is freed here */
m4hrt_driver_init_fail:
	m4hrt_err("%s: Init failed with error code %d.\n", __func__, err);
m4hrt_driver_init_exit:
	return err;
}

static int m4hrt_probe(struct platform_device *pdev)
{
	struct m4hrt_driver_data *dd = NULL;
	struct iio_dev *iio = NULL;
	int err = 0;

	iio = iio_device_alloc(sizeof(dd));
	if (iio == NULL) {
		m4hrt_err("%s: Failed to allocate IIO data.\n", __func__);
		err = -ENOMEM;
		goto m4hrt_probe_fail_noiio;
	}

	dd = iio_priv(iio);
	dd->iio = iio;
	dd->pdev = pdev;
	mutex_init(&(dd->mutex));
	platform_set_drvdata(pdev, dd);

	err = m4sensorhub_register_initcall(m4hrt_driver_init, dd);
	if (err < 0) {
		m4hrt_err("%s: Failed to register initcall.\n", __func__);
		goto m4hrt_probe_fail;
	}

	return 0;

m4hrt_probe_fail:
	mutex_destroy(&(dd->mutex));
	iio_device_free(dd->iio); /* dd is freed here */
m4hrt_probe_fail_noiio:
	m4hrt_err("%s: Probe failed with error code %d.\n", __func__, err);
	return err;
}

static int __exit m4hrt_remove(struct platform_device *pdev)
{
	struct m4hrt_driver_data *dd = platform_get_drvdata(pdev);

	if (dd == NULL)
		goto m4hrt_remove_exit;

	mutex_lock(&(dd->mutex));
	if (dd->status & (1 << M4HRT_IRQ_ENABLED_BIT)) {
		m4sensorhub_irq_disable(dd->m4,
					M4SH_IRQ_HEARTRATESENSOR_DATA_READY);
		dd->status = dd->status & ~(1 << M4HRT_IRQ_ENABLED_BIT);
	}
	m4sensorhub_irq_unregister(dd->m4,
				   M4SH_IRQ_HEARTRATESENSOR_DATA_READY);
	m4sensorhub_unregister_initcall(m4hrt_driver_init);
	mutex_destroy(&(dd->mutex));
	if (dd->iio != NULL)
		m4hrt_remove_iiodev(dd);  /* dd is freed here */

m4hrt_remove_exit:
	return 0;
}

static struct of_device_id m4heartrate_match_tbl[] = {
	{ .compatible = "mot,m4heartrate" },
	{},
};

static struct platform_driver m4hrt_driver = {
	.probe		= m4hrt_probe,
	.remove		= __exit_p(m4hrt_remove),
	.shutdown	= NULL,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= M4HRT_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(m4heartrate_match_tbl),
	},
};

static int __init m4hrt_init(void)
{
	return platform_driver_register(&m4hrt_driver);
}

static void __exit m4hrt_exit(void)
{
	platform_driver_unregister(&m4hrt_driver);
}

module_init(m4hrt_init);
module_exit(m4hrt_exit);

MODULE_ALIAS("platform:m4hrt");
MODULE_DESCRIPTION("M4 Sensor Hub Heartrate client driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
