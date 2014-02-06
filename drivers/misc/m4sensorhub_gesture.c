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
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
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
#include <linux/m4sensorhub_client_ioctl.h>
#include <linux/uaccess.h>
#include <linux/m4sensorhub/MemMapGesture.h>
#include <linux/slab.h>

#define GESTURE_CLIENT_DRIVER_NAME	"m4sensorhub_gesture"

struct gesture_client {
	struct m4sensorhub_data *m4sensorhub;
	struct input_dev *input_dev;
	struct memMapGesture gesture_data;
};

static bool read_gesture_value(struct gesture_client *list,
				signed char *value,
				eGestureType gesture)
{
	if (list->gesture_data.gesture1 == gesture) {
		*value = list->gesture_data.value1;
		return true;
	} else if (list->gesture_data.gesture2 == gesture) {
		*value = list->gesture_data.value2;
		return true;
	} else if (list->gesture_data.gesture3 == gesture) {
		*value = list->gesture_data.value3;
		return true;
	} else {
		*value = 0;
		return false;
	}
}

static struct gesture_client *misc_gesture_data;

static int gesture_client_open(struct inode *inode, struct file *file)
{
	int err = 0;

	err = nonseekable_open(inode, file);
	if (err < 0) {
		KDEBUG(M4SH_ERROR, "%s failed\n", __func__);
		return err;
	}
	file->private_data = misc_gesture_data;

	return 0;
}

static int gesture_client_close(struct inode *inode, struct file *file)
{
	KDEBUG(M4SH_DEBUG, "gesture_client in %s\n", __func__);
	return 0;
}

static char m4_read_gesture_data(struct gesture_client *gesture_client_data)
{
	int ret;
	struct memMapGesture *gdata = &(gesture_client_data->gesture_data);

	ret = m4sensorhub_reg_read_n(gesture_client_data->m4sensorhub,
				M4SH_REG_GESTURE_VERSION,
				(char *)gdata,
				sizeof(gesture_client_data->gesture_data));

	if (ret != sizeof(gesture_client_data->gesture_data))
		goto ERR;

	KDEBUG(M4SH_DEBUG, "Gesture1 = %d, gesture2 = %d, gesture3 = %d\n",
		gdata->gesture1, gdata->gesture2, gdata->gesture3);
	KDEBUG(M4SH_DEBUG, "Confidence1 = %d, confidence2 = %d, confidence3 = %d\n",
		gdata->confidence1, gdata->confidence2, gdata->confidence3);
	KDEBUG(M4SH_DEBUG, "Value1 = %d, value2 = %d, value3 = %d\n",
		gdata->value1, gdata->value2, gdata->value3);
	return 0;
ERR:
	KDEBUG(M4SH_ERROR, "Gesture read failed\n");
	return -1;
}

static void m4_handle_gesture_irq(enum m4sensorhub_irqs int_event,
					void *gesture_data)
{
	signed char value;
	/*Trigger broadcast of display gesture intent*/
	struct gesture_client *gesture_client_data =
		(struct gesture_client *)gesture_data;

	struct memMapGesture *gdata = &(gesture_client_data->gesture_data);

	if (m4_read_gesture_data(gesture_client_data) < 0) {
		KDEBUG(M4SH_ERROR, "m4_read_gesture_data returned \
			error %s\n", __func__);
		return;
	}

	if (read_gesture_value(gesture_client_data, &value, GESTURE_TILT_SCROLL)) {
		input_event(gesture_client_data->input_dev, EV_ABS,
				ABS_TILTSCROLL, value);
	} else {
		if (read_gesture_value(gesture_client_data, &value, GESTURE_WRIST_ROTATE)) {
			/* send event to stop scrolling for wrist rotate */
			input_event(gesture_client_data->input_dev, EV_ABS,
					ABS_TILTSCROLL, 0);
		}
	}

	input_event(gesture_client_data->input_dev, EV_MSC,
			MSC_GESTURE1, gdata->gesture1);
	input_event(gesture_client_data->input_dev, EV_MSC,
			MSC_GESTURE2, gdata->gesture2);
	input_event(gesture_client_data->input_dev, EV_MSC,
			MSC_GESTURE3, gdata->gesture3);
	input_event(gesture_client_data->input_dev, EV_MSC,
			MSC_GESTURE_CONFIDENCE1, gdata->confidence1);
	input_event(gesture_client_data->input_dev, EV_MSC,
			MSC_GESTURE_CONFIDENCE2, gdata->confidence2);
	input_event(gesture_client_data->input_dev, EV_MSC,
			MSC_GESTURE_CONFIDENCE3, gdata->confidence3);
	input_event(gesture_client_data->input_dev, EV_MSC,
			MSC_GESTURE_VALUE1, gdata->value1);
	input_event(gesture_client_data->input_dev, EV_MSC,
			MSC_GESTURE_VALUE2, gdata->value2);
	input_event(gesture_client_data->input_dev, EV_MSC,
			MSC_GESTURE_VALUE3, gdata->value3);

	input_sync(gesture_client_data->input_dev);

}

/*
 * Handle commands from user-space.
 */
static long gesture_client_ioctl(struct file *filp,
				 unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	void __user *argp = (void __user *)arg;
	struct gesture_client *gesture_client_data = filp->private_data;
	unsigned char byte;

	switch (cmd) {
	case M4_SENSOR_IOCTL_SET_SCREEN_ON_GESTURE_STATUS:
		/* TODO
		Turn on/off the gesture feature on M4 */
		break;
	case M4_SENSOR_IOCTL_SET_SCREEN_STATUS:
		if (copy_from_user(&byte, argp, sizeof(byte))) {
			KDEBUG(M4SH_ERROR, "Copy frm usr err:screen status\n");
			ret = -EFAULT;
			break;
		}
		/* validate data */
		if (byte > 1) {
			KDEBUG(M4SH_DEBUG, "Invalid screen status=0x%x", byte);
			ret = -EINVAL;
			break;
		}
		KDEBUG(M4SH_DEBUG, "Screen status set to = 0x%x", byte);
		ret = m4sensorhub_reg_write_1byte(
			gesture_client_data->m4sensorhub,
			M4SH_REG_USERSETTINGS_SCREENSTATUS, byte, 0xFF);
		if (ret != 1)
			KDEBUG(M4SH_ERROR, "Error writing screen status\n");
		break;
	default:
		KDEBUG(M4SH_ERROR, "Invalid IOCTL Command in %s\n", __func__);
		 ret = -EINVAL;
	}
	return ret;
}

static ssize_t m4_gesture_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gesture_client *gesture_client_data = platform_get_drvdata(pdev);

	if (m4_read_gesture_data(gesture_client_data) < 0)
		KDEBUG(M4SH_ERROR, "%s: Read gesture data failed \n", __func__);
	KDEBUG(M4SH_DEBUG, "%s:Gesture1,2,3 = %d %d %d conf1,2,3 = %d %d %d value1,2,3 = %d %d %d\n",
		__func__, gesture_client_data->gesture_data.gesture1,
			gesture_client_data->gesture_data.gesture2,
			gesture_client_data->gesture_data.gesture3,
			gesture_client_data->gesture_data.confidence1,
			gesture_client_data->gesture_data.confidence2,
			gesture_client_data->gesture_data.confidence3,
			gesture_client_data->gesture_data.value1,
			gesture_client_data->gesture_data.value2,
			gesture_client_data->gesture_data.value3);

	return sprintf(buf, "gesture1,2,3=%d %d %d,confidence1,2,3=%d %d %d value1,2,3 = %d %d %d\n",
			gesture_client_data->gesture_data.gesture1,
			gesture_client_data->gesture_data.gesture2,
			gesture_client_data->gesture_data.gesture3,
			gesture_client_data->gesture_data.confidence1,
			gesture_client_data->gesture_data.confidence2,
			gesture_client_data->gesture_data.confidence3,
			gesture_client_data->gesture_data.value1,
			gesture_client_data->gesture_data.value2,
			gesture_client_data->gesture_data.value3);
}

static DEVICE_ATTR(gesture_status, 0444, m4_gesture_status, NULL);

static const struct file_operations gesture_client_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gesture_client_ioctl,
	.open  = gesture_client_open,
	.release = gesture_client_close,
};

static struct miscdevice gesture_client_miscdrv = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = GESTURE_CLIENT_DRIVER_NAME,
	.fops = &gesture_client_fops,
};

static int gesture_driver_init(struct init_calldata *p_arg)
{
	int ret = 0;
	struct m4sensorhub_data *m4sensorhub = p_arg->p_m4sensorhub_data;
	ret = m4sensorhub_irq_register(m4sensorhub, M4SH_IRQ_GESTURE_DETECTED,
					m4_handle_gesture_irq,
					misc_gesture_data);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering int %d (%d)\n",
			M4SH_IRQ_GESTURE_DETECTED, ret);
		return ret;
	}
	ret = m4sensorhub_irq_enable(m4sensorhub, M4SH_IRQ_GESTURE_DETECTED);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error enabling int %d (%d)\n",
			M4SH_IRQ_GESTURE_DETECTED, ret);
		goto exit;
	}
	return ret;

exit:
	m4sensorhub_irq_unregister(m4sensorhub, M4SH_IRQ_GESTURE_DETECTED);
	return ret;
}

static int gesture_client_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct gesture_client *gesture_client_data;
	struct m4sensorhub_data *m4sensorhub = m4sensorhub_client_get_drvdata();

	if (!m4sensorhub)
		return -EFAULT;

	gesture_client_data = kzalloc(sizeof(*gesture_client_data),
						GFP_KERNEL);
	if (!gesture_client_data) {
		KDEBUG(M4SH_ERROR, "%s failed: unable to allocate"
				"for client_data\n", __func__);
		return -ENOMEM;
	}

	gesture_client_data->m4sensorhub = m4sensorhub;
	platform_set_drvdata(pdev, gesture_client_data);

	gesture_client_data->input_dev = input_allocate_device();
	if (!gesture_client_data->input_dev) {
		ret = -ENOMEM;
		KDEBUG(M4SH_ERROR, "%s: input device allocate failed: %d\n",
			__func__, ret);
		goto free_mem;
	}

	gesture_client_data->input_dev->name = GESTURE_CLIENT_DRIVER_NAME;
	set_bit(EV_MSC, gesture_client_data->input_dev->evbit);
	set_bit(MSC_GESTURE1, gesture_client_data->input_dev->mscbit);
	set_bit(MSC_GESTURE2, gesture_client_data->input_dev->mscbit);
	set_bit(MSC_GESTURE3, gesture_client_data->input_dev->mscbit);
	set_bit(MSC_GESTURE_CONFIDENCE1, gesture_client_data->input_dev->mscbit);
	set_bit(MSC_GESTURE_CONFIDENCE2, gesture_client_data->input_dev->mscbit);
	set_bit(MSC_GESTURE_CONFIDENCE3, gesture_client_data->input_dev->mscbit);
	set_bit(MSC_GESTURE_VALUE1, gesture_client_data->input_dev->mscbit);
	set_bit(MSC_GESTURE_VALUE2, gesture_client_data->input_dev->mscbit);
	set_bit(MSC_GESTURE_VALUE3, gesture_client_data->input_dev->mscbit);

	set_bit(EV_ABS, gesture_client_data->input_dev->evbit);
	input_set_abs_params(gesture_client_data->input_dev, ABS_TILTSCROLL,
		-128, 127, 0, 0);

	if (input_register_device(gesture_client_data->input_dev)) {
		KDEBUG(M4SH_ERROR, "%s: input device register failed\n",
			__func__);
		input_free_device(gesture_client_data->input_dev);
		goto free_mem;
	}

	ret = misc_register(&gesture_client_miscdrv);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering %s driver\n", __func__);
		goto unregister_input_device;
	}
	misc_gesture_data = gesture_client_data;
	ret = m4sensorhub_register_initcall(gesture_driver_init,
						gesture_client_data);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Unable to register init function"
			"for gesture client = %d\n", ret);
		goto unregister_misc_device;
	}
	if (device_create_file(&pdev->dev, &dev_attr_gesture_status)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n", __func__);
		ret = -1;
		goto unregister_initcall;
	}
	KDEBUG(M4SH_INFO, "Initialized %s driver\n", __func__);
	return 0;

unregister_initcall:
	m4sensorhub_unregister_initcall(gesture_driver_init);
unregister_misc_device:
	misc_gesture_data = NULL;
	misc_deregister(&gesture_client_miscdrv);
unregister_input_device:
	input_unregister_device(gesture_client_data->input_dev);
free_mem:
	platform_set_drvdata(pdev, NULL);
	gesture_client_data->m4sensorhub = NULL;
	kfree(gesture_client_data);
	gesture_client_data = NULL;
	return ret;
}

static int __exit gesture_client_remove(struct platform_device *pdev)
{
	struct gesture_client *gesture_client_data =
						platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_gesture_status);
	m4sensorhub_irq_disable(gesture_client_data->m4sensorhub,
				M4SH_IRQ_GESTURE_DETECTED);
	m4sensorhub_irq_unregister(gesture_client_data->m4sensorhub,
				M4SH_IRQ_GESTURE_DETECTED);
	m4sensorhub_unregister_initcall(gesture_driver_init);
	misc_gesture_data = NULL;
	misc_deregister(&gesture_client_miscdrv);
	input_unregister_device(gesture_client_data->input_dev);
	platform_set_drvdata(pdev, NULL);
	gesture_client_data->m4sensorhub = NULL;
	kfree(gesture_client_data);
	gesture_client_data = NULL;
	return 0;
}

static void gesture_client_shutdown(struct platform_device *pdev)
{
	return;
}
#ifdef CONFIG_PM
static int gesture_client_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	return 0;
}

static int gesture_client_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define gesture_client_suspend NULL
#define gesture_client_resume  NULL
#endif

static struct of_device_id m4gesture_match_tbl[] = {
	{ .compatible = "mot,m4gesture" },
	{},
};

static struct platform_driver gesture_client_driver = {
	.probe		= gesture_client_probe,
	.remove		= __exit_p(gesture_client_remove),
	.shutdown	= gesture_client_shutdown,
	.suspend	= gesture_client_suspend,
	.resume		= gesture_client_resume,
	.driver		= {
		.name	= GESTURE_CLIENT_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(m4gesture_match_tbl),
	},
};

static int __init gesture_client_init(void)
{
	return platform_driver_register(&gesture_client_driver);
}

static void __exit gesture_client_exit(void)
{
	platform_driver_unregister(&gesture_client_driver);
}

module_init(gesture_client_init);
module_exit(gesture_client_exit);

MODULE_ALIAS("platform:gesture_client");
MODULE_DESCRIPTION("M4 Sensor Hub Gesture client driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

