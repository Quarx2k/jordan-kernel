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
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/m4sensorhub.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define HEARTRATE_CLIENT_DRIVER_NAME "m4sensorhub_heartrate"


struct heartrate_client {
	struct m4sensorhub_data *m4sensorhub;
};

static void m4_read_and_report_hr_data(struct heartrate_client *hr_client_data)
{
	/* TODO */
}

static void m4_handle_heartrate_irq(enum m4sensorhub_irqs int_event,
					void *heartrate_data)
{
	struct heartrate_client *heartrate_client_data = heartrate_data;

	m4_read_and_report_hr_data(heartrate_client_data);
}

static struct miscdevice heartrate_client_miscdrv = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = HEARTRATE_CLIENT_DRIVER_NAME,
};
static ssize_t hrsensor_set_samplerate(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int samplerate, ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct heartrate_client *heartrate_client_data =
			 platform_get_drvdata(pdev);

	sscanf(buf, "%d", &samplerate);
	/* set sample rate */
	ret = m4sensorhub_reg_write(
				heartrate_client_data->m4sensorhub,
				M4SH_REG_HEARTRATESENSOR_SAMPLERATE,
				(char *)&samplerate, m4sh_no_mask
				);
	if (ret < 0)
		KDEBUG(
			M4SH_ERROR, "Failed to set hr samplerate to %d\n",
			samplerate
			);
	if (samplerate == -1) {
		/* disable interrupt */
		ret = m4sensorhub_irq_disable(
					heartrate_client_data->m4sensorhub,
					M4SH_IRQ_HEARTRATESENSOR_DATA_READY
					);
	} else {
		/* enable interrupt */
		ret = m4sensorhub_irq_enable(
					heartrate_client_data->m4sensorhub,
					M4SH_IRQ_HEARTRATESENSOR_DATA_READY
					);
	}
	if (ret < 0)
		KDEBUG(M4SH_ERROR, "Error enabling hr int(%d)\n", ret);

	return count;
}
static DEVICE_ATTR(samplerate, S_IWUSR, NULL, hrsensor_set_samplerate);

/* This interface is provided to be able to write to afe registers
   eg: If we need to write 0x334455 to register 0x22 then
	echo 0x22 0x334455 > reg_write
*/
#define HR_REG_WRITE_COMMAND  0x00
#define HR_REG_READ_COMMAND   0x01
static ssize_t m4_hr_write_register(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct heartrate_client *heartrate_client_data =
			platform_get_drvdata(pdev);
	int value, reg;

	sscanf(buf, "0x%x 0x%x", &reg, &value);
	/* Write register addr */
	m4sensorhub_reg_write_1byte(
				heartrate_client_data->m4sensorhub,
				M4SH_REG_HEARTRATESENSOR_REGADDR,
				(reg & 0xFF), 0xff
				);
	/* Write register value */
	m4sensorhub_reg_write(
				heartrate_client_data->m4sensorhub,
				M4SH_REG_HEARTRATESENSOR_REGVALUE,
				(char *)&value, m4sh_no_mask
				);
	/* Write the command to be write register command */
	m4sensorhub_reg_write_1byte(
				heartrate_client_data->m4sensorhub,
				M4SH_REG_HEARTRATESENSOR_REGRWCMD,
				HR_REG_WRITE_COMMAND, 0xff
				);
	return count;
}
static DEVICE_ATTR(reg_write, S_IWUSR, NULL, m4_hr_write_register);

/* This interface is provided to be able to read a afe register
	eg: To read register 0x22
		echo 0x22 > reg_read
		cat reg_read
	expected output: 0x334455
*/
static ssize_t m4_write_regaddr(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct heartrate_client *heartrate_client_data =
		platform_get_drvdata(pdev);
	int reg;

	sscanf(buf, "0x%x", &reg);
	/* Write register addr */
	m4sensorhub_reg_write_1byte(
				heartrate_client_data->m4sensorhub,
				M4SH_REG_HEARTRATESENSOR_REGADDR,
				(reg & 0xFF), 0xff
				);
	/* Write the command to be read register */
	m4sensorhub_reg_write_1byte(
				heartrate_client_data->m4sensorhub,
				M4SH_REG_HEARTRATESENSOR_REGRWCMD,
				HR_REG_READ_COMMAND, 0xff
				);
	return count;
}

static ssize_t m4_read_value(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct heartrate_client *heartrate_client_data =
			platform_get_drvdata(pdev);
	char value[3];
	/* read register value */
	m4sensorhub_reg_read(
			heartrate_client_data->m4sensorhub,
			M4SH_REG_HEARTRATESENSOR_REGVALUE,
			value
			);
	return sprintf(buf, "0x%02x%02x%02x\n", value[0], value[1], value[2]);
}

static DEVICE_ATTR(reg_read, S_IRUSR|S_IWUSR, m4_read_value, m4_write_regaddr);

struct attribute *hr_attributes[] = {
	&dev_attr_samplerate.attr,
	&dev_attr_reg_write.attr,
	&dev_attr_reg_read.attr,
	NULL
};

static const struct attribute_group hr_attribute_group = {
	.attrs = hr_attributes,
};

static int heartrate_driver_init(struct init_calldata *data)
{
	int ret;
	struct m4sensorhub_data *m4sensorhub = data->p_m4sensorhub_data;

	ret = m4sensorhub_irq_register(m4sensorhub,
					M4SH_IRQ_HEARTRATESENSOR_DATA_READY,
					m4_handle_heartrate_irq,
					(struct heartrate_client *)data->p_data
					);
	if (ret < 0) {
		KDEBUG(
			M4SH_ERROR,
			"Error registering int %d (%d)\n",
			M4SH_IRQ_HEARTRATESENSOR_DATA_READY, ret
			);
	}
	return ret;
}

static int __init heartrate_client_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct heartrate_client *heartrate_client_data;
	struct m4sensorhub_data *m4sensorhub = m4sensorhub_client_get_drvdata();


	if (!m4sensorhub)
		return -EFAULT;

	heartrate_client_data = kzalloc(sizeof(struct heartrate_client),
						GFP_KERNEL);
	if (!heartrate_client_data)
		return -ENOMEM;

	heartrate_client_data->m4sensorhub = m4sensorhub;
	platform_set_drvdata(pdev, heartrate_client_data);

	ret = misc_register(&heartrate_client_miscdrv);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering %s driver\n", __func__);
		goto free_mem;
	}
	ret = m4sensorhub_register_initcall(
					heartrate_driver_init,
					heartrate_client_data
					);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Unable to register init function"
			"for heartrate client = %d\n", ret
			);
			goto unregister_misc_device;
	}
	ret = sysfs_create_group(&pdev->dev.kobj, &hr_attribute_group);
	if (ret) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n", __func__);
		ret = -1;
		goto unregister_initcall;
	}
	KDEBUG(M4SH_INFO, "Initialized %s driver\n", __func__);
	return 0;

unregister_initcall:
	m4sensorhub_unregister_initcall(heartrate_driver_init);
unregister_misc_device:
	misc_deregister(&heartrate_client_miscdrv);
free_mem:
	platform_set_drvdata(pdev, NULL);
	heartrate_client_data->m4sensorhub = NULL;
	kfree(heartrate_client_data);
	heartrate_client_data = NULL;
	return ret;
}

static int __exit heartrate_client_remove(struct platform_device *pdev)
{
	struct heartrate_client *heartrate_client_data =
						platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &hr_attribute_group);
	m4sensorhub_irq_disable(
				heartrate_client_data->m4sensorhub,
				M4SH_IRQ_HEARTRATESENSOR_DATA_READY
				);
	m4sensorhub_irq_unregister(
				heartrate_client_data->m4sensorhub,
				M4SH_IRQ_HEARTRATESENSOR_DATA_READY
				);
	m4sensorhub_unregister_initcall(heartrate_driver_init);
	misc_deregister(&heartrate_client_miscdrv);
	platform_set_drvdata(pdev, NULL);
	heartrate_client_data->m4sensorhub = NULL;
	kfree(heartrate_client_data);
	heartrate_client_data = NULL;
	return 0;
}

static void heartrate_client_shutdown(struct platform_device *pdev)
{
	return;
}
#ifdef CONFIG_PM
static int heartrate_client_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	return 0;
}

static int heartrate_client_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define heartrate_client_suspend NULL
#define heartrate_client_resume  NULL
#endif

static struct of_device_id m4heartrate_match_tbl[] = {
	{ .compatible = "mot,m4heartrate" },
	{},
};

static struct platform_driver heartrate_client_driver = {
	.probe		= heartrate_client_probe,
	.remove		= __exit_p(heartrate_client_remove),
	.shutdown	= heartrate_client_shutdown,
	.suspend	= heartrate_client_suspend,
	.resume		= heartrate_client_resume,
	.driver		= {
		.name	= HEARTRATE_CLIENT_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(m4heartrate_match_tbl),
	},
};

static int __init heartrate_client_init(void)
{
	return platform_driver_register(&heartrate_client_driver);
}

static void __exit heartrate_client_exit(void)
{
	platform_driver_unregister(&heartrate_client_driver);
}

module_init(heartrate_client_init);
module_exit(heartrate_client_exit);

MODULE_ALIAS("platform:heartrate_client");
MODULE_DESCRIPTION("M4 Sensor Hub heartrate client driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

