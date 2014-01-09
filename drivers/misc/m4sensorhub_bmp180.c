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
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/m4sensorhub.h>
#include <linux/m4sensorhub_client_ioctl.h>
#include <linux/m4sensorhub/MemMapPressureSensor.h>
#include <linux/slab.h>

#define PRESSURE_CLIENT_DRIVER_NAME "m4sensorhub_bmp180"
#define PRESSURE_MIN		30000
#define PRESSURE_MAX		110000

struct pressure_client {
	struct m4sensorhub_data *m4sensorhub;
	struct input_dev *input_dev;
	int pressure;
	int altitude;
	signed short samplerate;
};

struct pressure_client *misc_pressure_data;

static int pressure_client_open(struct inode *inode, struct file *file)
{
	int err = 0;

	err = nonseekable_open(inode, file);
	if (err < 0) {
		KDEBUG(M4SH_ERROR, " %s failed\n", __func__);
		return err;
	}
	file->private_data = misc_pressure_data;

	return 0;
}

static int pressure_client_close(struct inode *inode, struct file *file)
{
	KDEBUG(M4SH_DEBUG, " pressure_client in %s\n", __func__);
	return 0;
}

static void m4_report_pressure_inputevent(struct pressure_client
						*pressure_client_data)
{
	input_report_abs(pressure_client_data->input_dev, ABS_PRESSURE,
			pressure_client_data->pressure);
	input_report_abs(pressure_client_data->input_dev, ABS_ALTITUDE,
			pressure_client_data->altitude);
	input_sync(pressure_client_data->input_dev);
}

static void m4_read_pressure_data(struct pressure_client *pressure_data)
{
	sPressureData pressure;

	m4sensorhub_reg_read(pressure_data->m4sensorhub,
			M4SH_REG_PRESSURE_PRESSURE,
			(char *)&pressure.pressure);
	pressure_data->pressure = pressure.pressure;
	m4sensorhub_reg_read(pressure_data->m4sensorhub,
			M4SH_REG_PRESSURE_ABSOLUTEALTITUDE,
			(char *)&pressure.absoluteAltitude);
	pressure_data->altitude = pressure.absoluteAltitude;
}

static void m4_handle_pressure_irq(enum m4sensorhub_irqs int_event,
					void *pressure_data)
{
	struct pressure_client *pressure_client_data = pressure_data;

	m4_read_pressure_data(pressure_client_data);
	m4_report_pressure_inputevent(pressure_client_data);
}

static int m4_set_pressure_samplerate(
			struct pressure_client *pressure_client_data,
			signed int samplerate)
{
	int ret = 0;

	if (samplerate != pressure_client_data->samplerate) {
		ret = m4sensorhub_reg_write(pressure_client_data->m4sensorhub,
				M4SH_REG_PRESSURE_SAMPLERATE,
				(char *)&samplerate, m4sh_no_mask);
		if (ret != m4sensorhub_reg_getsize(
			pressure_client_data->m4sensorhub,
			M4SH_REG_PRESSURE_SAMPLERATE)) {
				KDEBUG(M4SH_ERROR, "Unable to set delay for \
						pressure sensor\n");
			return ret;
		}

		KDEBUG(M4SH_DEBUG, "%s() updating samplerate from %d to %d\n",
				   __func__, pressure_client_data->samplerate,
				   samplerate);
		pressure_client_data->samplerate = samplerate;

		if (samplerate >= 0)
			ret = m4sensorhub_irq_enable(
				pressure_client_data->m4sensorhub,
				M4SH_IRQ_PRESSURE_DATA_READY);
		else
			ret = m4sensorhub_irq_disable(
				pressure_client_data->m4sensorhub,
				M4SH_IRQ_PRESSURE_DATA_READY);
		if (ret != 0)
			KDEBUG(M4SH_ERROR, "Unable to enable/disable \
				pressure irq\n");
	}

	return ret;
}

/*
 * Handle commands from user-space.
 */
static long pressure_client_ioctl(struct file *filp,
				 unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int delay;
	unsigned char mask[] = {0xff, 0xff, 0xff, 0xff};
	static int status;
	int altitude = 0;
	void __user *argp = (void __user *)arg;
	struct pressure_client *pressure_client_data = filp->private_data;

	switch (cmd) {
	case M4_SENSOR_IOCTL_GET_PRESSURE:
		m4_read_pressure_data(pressure_client_data);
		m4_report_pressure_inputevent(pressure_client_data);
		break;

	case M4_SENSOR_IOCTL_SET_DELAY:
		if (copy_from_user(&delay, argp, sizeof(delay)))
			return -EFAULT;
		if (delay >= 0)
			ret = m4_set_pressure_samplerate(pressure_client_data,
							 delay);
		if (ret < 0) {
			KDEBUG(M4SH_ERROR, "Error enabling int %d (%d)\n",
				M4SH_IRQ_PRESSURE_DATA_READY, ret);
				return -EFAULT;
		}
		break;

	case M4_SENSOR_IOCTL_APP_GET_FLAG:
		if (copy_to_user(argp, &status, sizeof(status)))
			return -EFAULT;
		break;

	case M4_SENSOR_IOCTL_APP_SET_FLAG:
		if (copy_from_user(&status, argp, sizeof(status)))
			return -EFAULT;
		break;
	case M4_SENSOR_IOCTL_SET_ALTITUDE:
		if (copy_from_user(&altitude, argp, sizeof(altitude)))
			return -EFAULT;
		if (altitude > 0)
			m4sensorhub_reg_write(pressure_client_data->m4sensorhub,
				M4SH_REG_PRESSURE_REFERENCEALTITUDE,
				(char *)&altitude, mask);

		break;
	default:
		KDEBUG(M4SH_ERROR, "Invalid IOCTL Command in %s \n", __func__);
		ret = -EINVAL;
	}
	return ret;
}

static ssize_t pressure_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pressure_client *pressure_client_data
				= platform_get_drvdata(pdev);

	m4_read_pressure_data(pressure_client_data);
	KDEBUG(M4SH_DEBUG, "%s  : Pressure : = %d",
		__func__, pressure_client_data->pressure);
	return sprintf(buf, "%d\n", pressure_client_data->pressure);
}

static ssize_t altitude_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pressure_client *pressure_client_data
				= platform_get_drvdata(pdev);

	m4_read_pressure_data(pressure_client_data);
	KDEBUG(M4SH_DEBUG, "%s  : Altitude : %d",
		__func__, pressure_client_data->altitude);
	return sprintf(buf, "%d\n", pressure_client_data->altitude);
}

static DEVICE_ATTR(pressure, 0444, pressure_show, NULL);
static DEVICE_ATTR(altitude, 0444, altitude_show, NULL);

static const struct file_operations pressure_client_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = pressure_client_ioctl,
	.open  = pressure_client_open,
	.release = pressure_client_close,
};

static struct miscdevice pressure_client_miscdrv = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = PRESSURE_CLIENT_DRIVER_NAME,
	.fops = &pressure_client_fops,
};


static int pressure_driver_init(struct m4sensorhub_data *m4sensorhub)
{
	int ret;
	ret = m4sensorhub_irq_register(m4sensorhub,
					M4SH_IRQ_PRESSURE_DATA_READY,
					m4_handle_pressure_irq,
					misc_pressure_data);
	if (ret < 0)
		KDEBUG(M4SH_ERROR, "Error registering int %d (%d)\n",
			M4SH_IRQ_PRESSURE_DATA_READY, ret);
	return ret;
}

static int pressure_client_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct pressure_client *pressure_client_data;
	struct m4sensorhub_data *m4sensorhub = m4sensorhub_client_get_drvdata();

	if (!m4sensorhub)
		return -EFAULT;

	pressure_client_data = kzalloc(sizeof(*pressure_client_data),
						GFP_KERNEL);
	if (!pressure_client_data)
		return -ENOMEM;

	pressure_client_data->m4sensorhub = m4sensorhub;
	platform_set_drvdata(pdev, pressure_client_data);

	pressure_client_data->input_dev = input_allocate_device();
	if (!pressure_client_data->input_dev) {
		ret = -ENOMEM;
		KDEBUG(M4SH_ERROR, "%s: input device allocate failed: %d\n",
		__func__, ret);
		goto free_mem;
	}

	pressure_client_data->input_dev->name = PRESSURE_CLIENT_DRIVER_NAME;
	set_bit(EV_ABS, pressure_client_data->input_dev->evbit);
	input_set_abs_params(pressure_client_data->input_dev, ABS_ALTITUDE,
		-2147483647, 2147483647, 0, 0);
	input_set_abs_params(pressure_client_data->input_dev, ABS_PRESSURE,
					PRESSURE_MIN, PRESSURE_MAX, 0, 0);

	if (input_register_device(pressure_client_data->input_dev)) {
		KDEBUG(M4SH_ERROR, "%s: input device register failed\n",
			__func__);
		input_free_device(pressure_client_data->input_dev);
		goto free_mem;
	}

	ret = misc_register(&pressure_client_miscdrv);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering %s driver\n",
				 PRESSURE_CLIENT_DRIVER_NAME);
		goto unregister_input_device;
	}
	misc_pressure_data = pressure_client_data;
	ret = m4sensorhub_register_initcall(pressure_driver_init);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Unable to register init function "
			"for pressure client = %d\n", ret);
		goto unregister_misc_device;
	}
	if (device_create_file(&pdev->dev, &dev_attr_pressure)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n",
				PRESSURE_CLIENT_DRIVER_NAME);
		ret = -1;
		goto unregister_initcall;
	}

	if (device_create_file(&pdev->dev, &dev_attr_altitude)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n",
				PRESSURE_CLIENT_DRIVER_NAME);
		ret = -1;
		goto remove_device_file;
	}
	KDEBUG(M4SH_ERROR, "Initialized %s driver\n", __func__);
	return 0;

remove_device_file:
	device_remove_file(&pdev->dev, &dev_attr_pressure);
unregister_initcall:
	m4sensorhub_unregister_initcall(pressure_driver_init);
unregister_misc_device:
	misc_pressure_data = NULL;
	misc_deregister(&pressure_client_miscdrv);
unregister_input_device:
	input_unregister_device(pressure_client_data->input_dev);
free_mem:
	platform_set_drvdata(pdev, NULL);
	pressure_client_data->m4sensorhub = NULL;
	kfree(pressure_client_data);
	pressure_client_data = NULL;
	return ret;
}

static int __exit pressure_client_remove(struct platform_device *pdev)
{
	struct pressure_client *pressure_client_data =
					platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_pressure);
	device_remove_file(&pdev->dev, &dev_attr_altitude);
	m4sensorhub_irq_disable(pressure_client_data->m4sensorhub,
					M4SH_IRQ_PRESSURE_DATA_READY);
	m4sensorhub_irq_unregister(pressure_client_data->m4sensorhub,
					M4SH_IRQ_PRESSURE_DATA_READY);
	m4sensorhub_unregister_initcall(pressure_driver_init);
	misc_pressure_data = NULL;
	misc_deregister(&pressure_client_miscdrv);
	input_unregister_device(pressure_client_data->input_dev);
	platform_set_drvdata(pdev, NULL);
	pressure_client_data->m4sensorhub = NULL;
	kfree(pressure_client_data);
	pressure_client_data = NULL;
	return 0;
}

static void pressure_client_shutdown(struct platform_device *pdev)
{
	return;
}
#ifdef CONFIG_PM
static int pressure_client_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	struct pressure_client *pressure_client_data =
			platform_get_drvdata(pdev);

	return m4_set_pressure_samplerate(pressure_client_data, -1);
}

static int pressure_client_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define pressure_client_suspend NULL
#define pressure_client_resume  NULL
#endif

static struct of_device_id m4pressure_match_tbl[] = {
	{.compatible = "mot,m4pressure" },
	{},
};

static struct platform_driver pressure_client_driver = {
	.probe		= pressure_client_probe,
	.remove		= __exit_p(pressure_client_remove),
	.shutdown	= pressure_client_shutdown,
	.suspend	= pressure_client_suspend,
	.resume		= pressure_client_resume,
	.driver		= {
		.name	= PRESSURE_CLIENT_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(m4pressure_match_tbl),
	},
};

static int __init pressure_client_init(void)
{
	return platform_driver_register(&pressure_client_driver);
}

static void __exit pressure_client_exit(void)
{
	platform_driver_unregister(&pressure_client_driver);
}

module_init(pressure_client_init);
module_exit(pressure_client_exit);

MODULE_ALIAS("platform:pressure_client");
MODULE_DESCRIPTION("M4 Sensor Hub Pressure client driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

