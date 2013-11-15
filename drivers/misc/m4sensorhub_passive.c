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
#include <linux/m4sensorhub/MemMapPassive.h>
#include <linux/m4sensorhub_client_ioctl.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define PASSIVE_CLIENT_DRIVER_NAME	"m4sensorhub_passive"

struct m4_passive_data {
	u32 mets;
	u32 steps;
	u32 floorsClimbed;
	u32 timestamp;
};

struct passive_client {
	struct m4sensorhub_data *m4sensorhub;
	struct input_dev *input_dev;
};

static struct passive_client *misc_passive_data;
static struct m4_passive_data pdata_buffer[MAX_PASSIVE_BUFFERS] = {};

static int passive_client_open(struct inode *inode, struct file *file)
{
	int err = 0;

	err = nonseekable_open(inode, file);
	if (err < 0) {
		KDEBUG(M4SH_ERROR, "%s failed\n", __func__);
		return err;
	}
	file->private_data = misc_passive_data;

	return 0;
}

static int passive_client_close(struct inode *inode, struct file *file)
{
	KDEBUG(M4SH_DEBUG, "passive_client in %s\n", __func__);
	return 0;
}

static void m4_report_passive_inputevent(
		struct passive_client *passive_client_data)
{
	int i;
	for (i = 0; i < MAX_PASSIVE_BUFFERS; i++) {
		input_event(passive_client_data->input_dev, EV_MSC,
				MSC_PASSIVE_STEPS,
				pdata_buffer[i].steps);
		input_event(passive_client_data->input_dev, EV_MSC,
				MSC_PASSIVE_METS,
				pdata_buffer[i].mets);
		input_event(passive_client_data->input_dev, EV_MSC,
				MSC_PASSIVE_TIMESTAMP,
				pdata_buffer[i].timestamp);
		input_event(passive_client_data->input_dev, EV_MSC,
				MSC_PASSIVE_FLOORSCLIMBED,
				pdata_buffer[i].floorsClimbed);
		input_sync(passive_client_data->input_dev);
	}
}


static void m4_read_passive_data(struct passive_client *passive_client_data)
{
	int i;
	u32 steps[MAX_PASSIVE_BUFFERS] = {0};
	u32 mets[MAX_PASSIVE_BUFFERS] = {0};
	u32 timestamp[12] = {0};
	u32 floorsClimbed[MAX_PASSIVE_BUFFERS] = {0};

	/*read all buffers of steps*/
	m4sensorhub_reg_read(passive_client_data->m4sensorhub,
			M4SH_REG_PASSIVE_STEPS,
			(char *)&steps);
	m4sensorhub_reg_read(passive_client_data->m4sensorhub,
			M4SH_REG_PASSIVE_METS,
			(char *)&mets);
	m4sensorhub_reg_read(passive_client_data->m4sensorhub,
			M4SH_REG_PASSIVE_TIMESTAMP,
			(char *)&timestamp);
	m4sensorhub_reg_read(passive_client_data->m4sensorhub,
			M4SH_REG_PASSIVE_FLOORSCLIMBED,
			(char *)&floorsClimbed);
	for (i = 0; i < MAX_PASSIVE_BUFFERS; i++) {
		pdata_buffer[i].steps = steps[i];
		pdata_buffer[i].mets = mets[i];
		pdata_buffer[i].timestamp = timestamp[i];
		pdata_buffer[i].floorsClimbed = floorsClimbed[i];
		KDEBUG(M4SH_DEBUG, "steps = %u, mets = %u, timestamp = %u,\
			floorsClimbed = %u", pdata_buffer[i].steps,
			pdata_buffer[i].mets, pdata_buffer[i].timestamp,
			pdata_buffer[i].floorsClimbed);
	}
}

static void m4_handle_passive_irq(enum m4sensorhub_irqs int_event,
					void *passive_data)
{
	struct passive_client *passive_client_data = passive_data;

	m4_read_passive_data(passive_client_data);
	m4_report_passive_inputevent(passive_client_data);
}

/*
 * Handle commands from user-space.
 */
static long passive_client_ioctl(struct file *filp,
				 unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct passive_client *passive_client_data = filp->private_data;

	switch (cmd) {
	case M4_SENSOR_IOCTL_GET_PASSIVE_DATA:
		m4_read_passive_data(passive_client_data);
		m4_report_passive_inputevent(passive_client_data);
		break;
	default:
		KDEBUG(M4SH_ERROR, "Invalid IOCTL Command in %s\n", __func__);
		 ret = -EINVAL;
	}
	return ret;
}

static const struct file_operations passive_client_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = passive_client_ioctl,
	.open  = passive_client_open,
	.release = passive_client_close,
};

static struct miscdevice passive_client_miscdrv = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = PASSIVE_CLIENT_DRIVER_NAME,
	.fops = &passive_client_fops,
};

static int passive_client_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct passive_client *passive_client_data;
	struct m4sensorhub_data *m4sensorhub = m4sensorhub_client_get_drvdata();

	if (!m4sensorhub)
		return -EFAULT;

	passive_client_data = kzalloc(sizeof(*passive_client_data),
						GFP_KERNEL);
	if (!passive_client_data)
		return -ENOMEM;

	passive_client_data->m4sensorhub = m4sensorhub;
	platform_set_drvdata(pdev, passive_client_data);

	passive_client_data->input_dev = input_allocate_device();
	if (!passive_client_data->input_dev) {
		ret = -ENOMEM;
		KDEBUG(M4SH_ERROR, "%s: input device allocate failed: %d\n",
			__func__, ret);
		goto free_mem;
	}

	passive_client_data->input_dev->name = PASSIVE_CLIENT_DRIVER_NAME;
	set_bit(EV_MSC, passive_client_data->input_dev->evbit);
	set_bit(MSC_PASSIVE_STEPS, passive_client_data->input_dev->mscbit);
	set_bit(MSC_PASSIVE_METS, passive_client_data->input_dev->mscbit);
	set_bit(MSC_PASSIVE_TIMESTAMP, passive_client_data->input_dev->mscbit);
	set_bit(MSC_PASSIVE_FLOORSCLIMBED,
		passive_client_data->input_dev->mscbit);
	if (input_register_device(passive_client_data->input_dev)) {
		KDEBUG(M4SH_ERROR, "%s: input device register failed\n",
			__func__);
		input_free_device(passive_client_data->input_dev);
		goto free_mem;
	}

	ret = misc_register(&passive_client_miscdrv);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering %s driver\n", __func__);
		goto unregister_input_device;
	}
	misc_passive_data = passive_client_data;

	ret = m4sensorhub_irq_register(m4sensorhub,
				M4SH_IRQ_PASSIVE_BUFFER_FULL,
				m4_handle_passive_irq,
				passive_client_data);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering int %d (%d)\n",
					M4SH_IRQ_PASSIVE_BUFFER_FULL, ret);
		goto unregister_misc_device;
	}
	ret = m4sensorhub_irq_enable(m4sensorhub, M4SH_IRQ_PASSIVE_BUFFER_FULL);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error enabling int %d (%d)\n",
					M4SH_IRQ_PASSIVE_BUFFER_FULL, ret);
		goto unregister_irq;
	}
	KDEBUG(M4SH_INFO, "Initialized %s driver\n", __func__);
	return 0;

unregister_irq:
	m4sensorhub_irq_unregister(m4sensorhub, M4SH_IRQ_PASSIVE_BUFFER_FULL);
unregister_misc_device:
	misc_passive_data = NULL;
	misc_deregister(&passive_client_miscdrv);
unregister_input_device:
	input_unregister_device(passive_client_data->input_dev);
free_mem:
	platform_set_drvdata(pdev, NULL);
	passive_client_data->m4sensorhub = NULL;
	kfree(passive_client_data);
	passive_client_data = NULL;
	return ret;
}

static int __exit passive_client_remove(struct platform_device *pdev)
{
	struct passive_client *passive_client_data =
						platform_get_drvdata(pdev);

	m4sensorhub_irq_disable(passive_client_data->m4sensorhub,
				M4SH_IRQ_PASSIVE_BUFFER_FULL);
	m4sensorhub_irq_unregister(passive_client_data->m4sensorhub,
				M4SH_IRQ_PASSIVE_BUFFER_FULL);

	misc_passive_data = NULL;
	misc_deregister(&passive_client_miscdrv);
	input_unregister_device(passive_client_data->input_dev);
	platform_set_drvdata(pdev, NULL);
	passive_client_data->m4sensorhub = NULL;
	kfree(passive_client_data);
	passive_client_data = NULL;
	return 0;
}

static void passive_client_shutdown(struct platform_device *pdev)
{
	return;
}
#ifdef CONFIG_PM
static int passive_client_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	return 0;
}

static int passive_client_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define passive_client_suspend NULL
#define passive_client_resume  NULL
#endif

static struct of_device_id m4passive_match_tbl[] = {
	{ .compatible = "mot,m4passive" },
	{},
};

static struct platform_driver passive_client_driver = {
	.probe		= passive_client_probe,
	.remove		= __exit_p(passive_client_remove),
	.shutdown	= passive_client_shutdown,
	.suspend	= passive_client_suspend,
	.resume		= passive_client_resume,
	.driver		= {
		.name	= PASSIVE_CLIENT_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(m4passive_match_tbl),
	},
};

static int __init passive_client_init(void)
{
	return platform_driver_register(&passive_client_driver);
}

static void __exit passive_client_exit(void)
{
	platform_driver_unregister(&passive_client_driver);
}

module_init(passive_client_init);
module_exit(passive_client_exit);

MODULE_ALIAS("platform:passive_client");
MODULE_DESCRIPTION("M4 Sensor Hub Passive mode client driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

