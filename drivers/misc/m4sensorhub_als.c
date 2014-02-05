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
#include <linux/m4sensorhub.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define ALS_CLIENT_DRIVER_NAME	"m4sensorhub_als"

struct als_client {
	struct m4sensorhub_data *m4sensorhub;
	u16 als_signal;
};

static struct als_client *misc_als_data;

static int als_client_open(struct inode *inode, struct file *file)
{
	int err = 0;

	err = nonseekable_open(inode, file);
	if (err < 0) {
		KDEBUG(M4SH_ERROR, "%s failed\n", __func__);
		return err;
	}
	file->private_data = misc_als_data;

	return 0;
}

static int als_client_close(struct inode *inode, struct file *file)
{
	KDEBUG(M4SH_DEBUG, "als_client in %s\n", __func__);
	return 0;
}


static void m4_read_als_data(struct als_client *als_client_data)
{
	m4sensorhub_reg_read(
			als_client_data->m4sensorhub,
			M4SH_REG_LIGHTSENSOR_SIGNAL,
			(char *)&als_client_data->als_signal
			);
}

static void m4_handle_als_irq(enum m4sensorhub_irqs int_event,
					void *als_data)
{
	struct als_client *als_client_data = als_data;
	m4_read_als_data(als_client_data);
}

static const struct file_operations als_client_fops = {
	.owner = THIS_MODULE,
	.open  = als_client_open,
	.release = als_client_close,
};

static struct miscdevice als_client_miscdrv = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = ALS_CLIENT_DRIVER_NAME,
	.fops = &als_client_fops,
};

static ssize_t als_set_samplerate(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int samplerate, ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct als_client *als_client_data = platform_get_drvdata(pdev);

	sscanf(buf, "%d", &samplerate);
	/* set sample rate */
	ret = m4sensorhub_reg_write(
				als_client_data->m4sensorhub,
				M4SH_REG_LIGHTSENSOR_SAMPLERATE,
				(char *)&samplerate, m4sh_no_mask
				);
	if (ret < 0)
		KDEBUG(
			M4SH_ERROR, "Failed to set als samplerate to %d\n",
			samplerate
			);
	/* enable interrupt */
	ret = m4sensorhub_irq_enable(
				als_client_data->m4sensorhub,
				M4SH_IRQ_LIGHTSENSOR_DATA_READY
				);
	if (ret < 0)
		KDEBUG(M4SH_ERROR, "Error enabling als int(%d)\n", ret);

	return count;
}

static ssize_t als_get_signal(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct als_client *als_client_data = platform_get_drvdata(pdev);

	return sprintf(buf, "%ud\n", als_client_data->als_signal);
}

static DEVICE_ATTR(samplerate, 0222, NULL, als_set_samplerate);
static DEVICE_ATTR(signal, 0444, als_get_signal, NULL);

static struct attribute *als_attributes[] = {
	&dev_attr_samplerate.attr,
	&dev_attr_signal.attr,
	NULL
};

static const struct attribute_group als_attribute_group = {
	.attrs = als_attributes,
};

static int als_driver_init(struct m4sensorhub_data *m4sensorhub)
{
	int ret;
	ret = m4sensorhub_irq_register(m4sensorhub,
					M4SH_IRQ_LIGHTSENSOR_DATA_READY,
					m4_handle_als_irq,
					misc_als_data);
	if (ret < 0) {
		KDEBUG(
			M4SH_ERROR,
			"Error registering int %d (%d)\n",
			M4SH_IRQ_PASSIVE_BUFFER_FULL, ret
			);
	}

	return ret;
}

static int als_client_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct als_client *als_client_data;
	struct m4sensorhub_data *m4sensorhub = m4sensorhub_client_get_drvdata();

	if (!m4sensorhub)
		return -EFAULT;

	als_client_data = kzalloc(sizeof(*als_client_data),
						GFP_KERNEL);
	if (!als_client_data)
		return -ENOMEM;

	als_client_data->m4sensorhub = m4sensorhub;
	platform_set_drvdata(pdev, als_client_data);

	ret = misc_register(&als_client_miscdrv);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering %s driver\n", __func__);
		goto free_mem;
	}
	misc_als_data = als_client_data;
	ret = m4sensorhub_register_initcall(als_driver_init);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Unable to register init function"
			"for als client = %d\n", ret);
		goto unregister_misc_device;
	}
	ret = sysfs_create_group(&pdev->dev.kobj, &als_attribute_group);
	if (ret)
		goto unregister_initcall;


	KDEBUG(M4SH_INFO, "Initialized %s driver\n", __func__);
	return 0;
unregister_initcall:
	m4sensorhub_unregister_initcall(als_driver_init);
unregister_misc_device:
	misc_als_data = NULL;
	misc_deregister(&als_client_miscdrv);
free_mem:
	platform_set_drvdata(pdev, NULL);
	als_client_data->m4sensorhub = NULL;
	kfree(als_client_data);
	als_client_data = NULL;
	return ret;
}

static int __exit als_client_remove(struct platform_device *pdev)
{
	struct als_client *als_client_data =
						platform_get_drvdata(pdev);

	m4sensorhub_irq_disable(als_client_data->m4sensorhub,
				M4SH_IRQ_LIGHTSENSOR_DATA_READY);
	m4sensorhub_irq_unregister(
				als_client_data->m4sensorhub,
				M4SH_IRQ_LIGHTSENSOR_DATA_READY
				);
	m4sensorhub_unregister_initcall(als_driver_init);

	misc_als_data = NULL;
	misc_deregister(&als_client_miscdrv);
	platform_set_drvdata(pdev, NULL);
	als_client_data->m4sensorhub = NULL;
	kfree(als_client_data);
	als_client_data = NULL;
	return 0;
}

static void als_client_shutdown(struct platform_device *pdev)
{
	return;
}
#ifdef CONFIG_PM
static int als_client_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	return 0;
}

static int als_client_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define als_client_suspend NULL
#define als_client_resume  NULL
#endif

static struct of_device_id m4als_match_tbl[] = {
	{ .compatible = "mot,m4als" },
	{},
};

static struct platform_driver als_client_driver = {
	.probe		= als_client_probe,
	.remove		= __exit_p(als_client_remove),
	.shutdown	= als_client_shutdown,
	.suspend	= als_client_suspend,
	.resume		= als_client_resume,
	.driver		= {
		.name	= ALS_CLIENT_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(m4als_match_tbl),
	},
};

static int __init als_client_init(void)
{
	return platform_driver_register(&als_client_driver);
}

static void __exit als_client_exit(void)
{
	platform_driver_unregister(&als_client_driver);
}

module_init(als_client_init);
module_exit(als_client_exit);

MODULE_ALIAS("platform:als_client");
MODULE_DESCRIPTION("M4 Sensor Hub Passive mode client driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

