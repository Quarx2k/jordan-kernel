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
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/m4sensorhub.h>
#include <linux/m4sensorhub_client_ioctl.h>
#include <linux/m4sensorhub/MemMapTempSensor.h>

#define TMP_CLIENT_DRIVER_NAME "m4sensorhub_tmp006"
#define TMP_MAX		1250
#define TMP_MIN		-400

struct tmp_client {
	struct m4sensorhub_data *m4sensorhub;
	struct input_dev *input_dev;
	int internal_tmp;
	int external_tmp;
	signed short samplerate;
};

struct tmp_client *misc_tmp_data;

static int temperature_client_open(struct inode *inode, struct file *file)
{
	int err = 0;

	err = nonseekable_open(inode, file);
	if (err < 0) {
		KDEBUG(M4SH_DEBUG, "temperature_clientopen failed\n");
		return err;
	}
	file->private_data = misc_tmp_data;

	return 0;
}

static int temperature_client_close(struct inode *inode, struct file *file)
{
	KDEBUG(M4SH_DEBUG, "temperature_client in %s\n", __func__);
	return 0;
}

static void  m4_report_temp_inputevent(struct tmp_client *tmp_data)
{
	input_report_abs(tmp_data->input_dev, ABS_THROTTLE,
		tmp_data->external_tmp);
	input_sync(tmp_data->input_dev);
}

static void m4_read_temp_data(struct tmp_client *tmp_data)
{
	sTempData tmp;

	m4sensorhub_reg_read(tmp_data->m4sensorhub,
			M4SH_REG_TEMP_EXTRNLTEMP, (char *)&tmp.extrnlTemp);
	m4sensorhub_reg_read(tmp_data->m4sensorhub,
			M4SH_REG_TEMP_INTRNLTEMP, (char *)&tmp.intrnlTemp);
	tmp_data->internal_tmp = tmp.intrnlTemp;
	tmp_data->external_tmp = tmp.extrnlTemp;
}

static void m4_handle_tmp_irq(enum m4sensorhub_irqs int_event,
					void *tmp_data)
{
	struct tmp_client *tmp_client_data = tmp_data;

	m4_read_temp_data(tmp_client_data);
	m4_report_temp_inputevent(tmp_client_data);
}

static int m4_set_tmp_samplerate(
			struct tmp_client *tmp_client_data,
			signed int samplerate)
{
	int ret = 0;

	if (samplerate != tmp_client_data->samplerate) {
		ret = m4sensorhub_reg_write(tmp_client_data->m4sensorhub,
				M4SH_REG_TEMP_SAMPLERATE,
				(char *)&samplerate, m4sh_no_mask);
		if (ret != m4sensorhub_reg_getsize(
				tmp_client_data->m4sensorhub,
				M4SH_REG_TEMP_SAMPLERATE)) {
				KDEBUG(M4SH_ERROR, "Unable to set delay \
					for temperature sensor\n");
				return ret;
		}

		KDEBUG(M4SH_DEBUG, "%s() updating samplerate from %d to %d\n",
				   __func__, tmp_client_data->samplerate,
				   samplerate);
		tmp_client_data->samplerate = samplerate;

		if (samplerate >= 0)
			ret = m4sensorhub_irq_enable(
					tmp_client_data->m4sensorhub,
					M4SH_IRQ_TMP_DATA_READY);
		else
			ret = m4sensorhub_irq_disable(
					tmp_client_data->m4sensorhub,
					M4SH_IRQ_TMP_DATA_READY);
		if (ret != 0)
			KDEBUG(M4SH_ERROR, "Unable to enable/disable \
				temperature irq\n");
	}

	return ret;
}


/*
 * Handle commands from user-space.
 */
static long temperature_client_ioctl(struct file *filp,
				 unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int delay = 0;
	static int status;

	void __user *argp = (void __user *)arg;
	struct tmp_client *tmp_data = filp->private_data;

	switch (cmd) {
	case M4_SENSOR_IOCTL_GET_TEMPRATURE:
		m4_read_temp_data(tmp_data);
		m4_report_temp_inputevent(tmp_data);
		break;
	case M4_SENSOR_IOCTL_SET_DELAY:
		if (copy_from_user(&delay, argp, sizeof(delay)))
			return -EFAULT;
		if (delay >= 0)
			ret = m4_set_tmp_samplerate(tmp_data, delay);
		if (ret < 0) {
			KDEBUG(M4SH_ERROR, "Error setting samplerate to %d"
				" (%d)\n", delay, ret);
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
	default:
		KDEBUG(M4SH_ERROR, "Invalid IOCTL Command %d\n", cmd);
		 ret = -EINVAL;
	}
	return ret;
}

static ssize_t GetExternalTemp(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tmp_client *tmp_client_data = platform_get_drvdata(pdev);

	m4_read_temp_data(tmp_client_data);
	KDEBUG(M4SH_DEBUG, "%s  : external temp = %d",
		__func__, tmp_client_data->external_tmp);
	return sprintf(buf, "%d\n", tmp_client_data->external_tmp);
}

static ssize_t GetInternalTemp(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tmp_client *tmp_client_data = platform_get_drvdata(pdev);

	m4_read_temp_data(tmp_client_data);
	KDEBUG(M4SH_DEBUG, "%s  : internal temp = %d",
		__func__, tmp_client_data->internal_tmp);
	return sprintf(buf, "%d\n", tmp_client_data->internal_tmp);
}

static ssize_t tmp_get_loglevel(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned long long loglevel;
	struct platform_device *pdev = to_platform_device(dev);
	struct tmp_client *tmp_client_data = platform_get_drvdata(pdev);

	m4sensorhub_reg_read(tmp_client_data->m4sensorhub,
		M4SH_REG_LOG_LOGENABLE, (char *)&loglevel);
	loglevel = get_log_level(loglevel, TMP_MASK_BIT_1);
	return sprintf(buf, "%llu\n", loglevel);
}

static ssize_t tmp_set_loglevel(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	unsigned long level;
	unsigned long long mask = 0, newlevel;
	struct platform_device *pdev = to_platform_device(dev);
	struct tmp_client *tmp_client_data = platform_get_drvdata(pdev);

	if ((strict_strtoul(buf, 10, &level)) < 0)
		return -1;
	if (level > M4_MAX_LOG_LEVEL) {
		KDEBUG(M4SH_ERROR, " Invalid log level - %d\n", (int)level);
		return -1;
	}
	mask = (1ULL << TMP_MASK_BIT_1) | (1ULL << TMP_MASK_BIT_2);
	newlevel = (level << TMP_MASK_BIT_1);
	return m4sensorhub_reg_write(tmp_client_data->m4sensorhub,
	M4SH_REG_LOG_LOGENABLE, (char *)&newlevel, (unsigned char *)&mask);
}

static DEVICE_ATTR(internal, 0444, GetInternalTemp, NULL);
static DEVICE_ATTR(external, 0444, GetExternalTemp, NULL);
static DEVICE_ATTR(LogLevel, 0444, tmp_get_loglevel, tmp_set_loglevel);

static const struct file_operations temperature_client_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = temperature_client_ioctl,
	.open  = temperature_client_open,
	.release = temperature_client_close,
};

static struct miscdevice temperature_client_miscdrv = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = TMP_CLIENT_DRIVER_NAME,
	.fops = &temperature_client_fops,
};

static int temperature_client_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct tmp_client *tmp_client_data;
	struct m4sensorhub_data *m4sensorhub = m4sensorhub_client_get_drvdata();

	if (!m4sensorhub)
		return -EFAULT;

	tmp_client_data = kzalloc(sizeof(*tmp_client_data), GFP_KERNEL);
	if (!tmp_client_data)
		return -ENOMEM;

	tmp_client_data->m4sensorhub = m4sensorhub;
	platform_set_drvdata(pdev, tmp_client_data);

	tmp_client_data->input_dev = input_allocate_device();
	if (!tmp_client_data->input_dev) {
		ret = -ENOMEM;
		KDEBUG(M4SH_ERROR, "%s: input device allocate failed: %d\n",
		__func__, ret);
		goto free_memory;
	}

	tmp_client_data->input_dev->name = TMP_CLIENT_DRIVER_NAME;
	set_bit(EV_ABS, tmp_client_data->input_dev->evbit);
	set_bit(ABS_THROTTLE, tmp_client_data->input_dev->absbit);
	input_set_abs_params(tmp_client_data->input_dev, ABS_THROTTLE,
				TMP_MIN, TMP_MAX, 0, 0);

	if (input_register_device(tmp_client_data->input_dev)) {
		KDEBUG(M4SH_INFO, "%s: input device register failed\n",
			__func__);
		input_free_device(tmp_client_data->input_dev);
		goto free_memory;
	}

	ret = misc_register(&temperature_client_miscdrv);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering %s driver\n",
				 TMP_CLIENT_DRIVER_NAME);
		goto unregister_input_device;
	}
	misc_tmp_data = tmp_client_data;
	ret = m4sensorhub_irq_register(m4sensorhub, M4SH_IRQ_TMP_DATA_READY,
					m4_handle_tmp_irq,
					tmp_client_data);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering int %d (%d)\n",
				M4SH_IRQ_TMP_DATA_READY, ret);
		goto unregister_misc_device;
	}

	if (device_create_file(&pdev->dev, &dev_attr_internal)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n",
				TMP_CLIENT_DRIVER_NAME);
		ret = -1;
		goto unregister_irq;
	}

	if (device_create_file(&pdev->dev, &dev_attr_external)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n",
				TMP_CLIENT_DRIVER_NAME);
		ret = -1;
		goto remove_internal_device_file;
	}
	if (device_create_file(&pdev->dev, &dev_attr_LogLevel)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n",
				TMP_CLIENT_DRIVER_NAME);
		ret = -1;
		goto remove_external_device_file;
	}
	KDEBUG(M4SH_INFO, "Initialized %s driver\n", TMP_CLIENT_DRIVER_NAME);
	return 0;

remove_external_device_file:
	device_remove_file(&pdev->dev, &dev_attr_external);
remove_internal_device_file:
	device_remove_file(&pdev->dev, &dev_attr_internal);
unregister_irq:
	m4sensorhub_irq_unregister(m4sensorhub, M4SH_IRQ_TMP_DATA_READY);
unregister_misc_device:
	misc_tmp_data = NULL;
	misc_deregister(&temperature_client_miscdrv);
unregister_input_device:
	input_unregister_device(tmp_client_data->input_dev);
free_memory:
	platform_set_drvdata(pdev, NULL);
	tmp_client_data->m4sensorhub = NULL;
	kfree(tmp_client_data);
	tmp_client_data = NULL;
	return ret;
}

static int __exit temperature_client_remove(struct platform_device *pdev)
{
	struct tmp_client *tmp_client_data = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_LogLevel);
	device_remove_file(&pdev->dev, &dev_attr_external);
	device_remove_file(&pdev->dev, &dev_attr_internal);
	m4sensorhub_irq_disable(tmp_client_data->m4sensorhub,
				M4SH_IRQ_TMP_DATA_READY);
	m4sensorhub_irq_unregister(tmp_client_data->m4sensorhub,
				M4SH_IRQ_TMP_DATA_READY);
	misc_tmp_data = NULL;
	misc_deregister(&temperature_client_miscdrv);
	input_unregister_device(tmp_client_data->input_dev);
	platform_set_drvdata(pdev, NULL);
	tmp_client_data->m4sensorhub = NULL;
	kfree(tmp_client_data);
	tmp_client_data = NULL;
	return 0;
}

static void temperature_client_shutdown(struct platform_device *pdev)
{
	return;
}

#ifdef CONFIG_PM

static int temperature_client_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	struct tmp_client *tmp_client_data = platform_get_drvdata(pdev);

	return m4_set_tmp_samplerate(tmp_client_data, -1);
}

static int temperature_client_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define temperature_client_suspend NULL
#define temperature_client_resume  NULL
#endif

static struct of_device_id m4temp_match_tbl[] = {
	{ .compatible = "mot,m4temperature" },
	{},
};

static struct platform_driver temp_client_driver = {
	.probe		= temperature_client_probe,
	.remove		= __exit_p(temperature_client_remove),
	.shutdown	= temperature_client_shutdown,
	.suspend	= temperature_client_suspend,
	.resume		= temperature_client_resume,
	.driver		= {
		.name	= TMP_CLIENT_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(m4temp_match_tbl),
	},
};

static int __init temperature_client_init(void)
{
	return platform_driver_register(&temp_client_driver);
}

static void __exit temperature_client_exit(void)
{
	platform_driver_unregister(&temp_client_driver);
}

module_init(temperature_client_init);
module_exit(temperature_client_exit);

MODULE_ALIAS("platform:temperature_client");
MODULE_DESCRIPTION("M4 Sensor Hub driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

