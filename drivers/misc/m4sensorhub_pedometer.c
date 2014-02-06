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
#include <linux/m4sensorhub_client_ioctl.h>
#include <linux/m4sensorhub/MemMapPedometer.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define PEDOMETER_CLIENT_DRIVER_NAME "m4sensorhub_pedometer"

struct pedometer_data {
	unsigned char activity;
	unsigned int distance;
	unsigned int mets;
	unsigned char metsactivity;
	unsigned int calories;
	unsigned short stepcount;
	unsigned short speed;
	unsigned short floorsclimbed;
};

struct pedometer_client {
	struct m4sensorhub_data *m4sensorhub;
	struct input_dev *input_dev;
	struct pedometer_data prev_data;
	struct pedometer_data curr_data;
};

struct pedometer_client *misc_pedometer_data;

static int pedometer_client_open(struct inode *inode, struct file *file)
{
	int err = 0;

	err = nonseekable_open(inode, file);
	if (err < 0) {
		KDEBUG(M4SH_ERROR, "%s failed\n", __func__);
		return err;
	}
	file->private_data = misc_pedometer_data;

	return 0;
}

static int pedometer_client_close(struct inode *inode, struct file *file)
{
	KDEBUG(M4SH_DEBUG, "pedometer_client in %s\n", __func__);
	return 0;
}

static void m4_report_pedometer_inputevent(
	struct pedometer_client *pedo_client_data)
{
	input_event(pedo_client_data->input_dev, EV_MSC, MSC_ACTIVITY_TYPE,
		pedo_client_data->curr_data.activity);
	input_event(pedo_client_data->input_dev, EV_MSC, MSC_STEPCOUNT,
		pedo_client_data->curr_data.stepcount);
	input_event(pedo_client_data->input_dev, EV_MSC, MSC_DISTANCE,
		pedo_client_data->curr_data.distance);
	input_event(pedo_client_data->input_dev, EV_MSC, MSC_SPEED,
		pedo_client_data->curr_data.speed);
	input_event(pedo_client_data->input_dev, EV_MSC, MSC_METS,
		pedo_client_data->curr_data.mets);
	input_event(pedo_client_data->input_dev, EV_MSC, MSC_CALORIES,
		pedo_client_data->curr_data.calories);
	input_event(pedo_client_data->input_dev, EV_MSC, MSC_FLOORSCLIMBED,
		pedo_client_data->curr_data.floorsclimbed);
	input_event(pedo_client_data->input_dev, EV_MSC, MSC_METSACTIVITY,
		pedo_client_data->curr_data.metsactivity);
	input_sync(pedo_client_data->input_dev);

	KDEBUG(M4SH_DEBUG, "Sending pedometer data : stepcount = %d,\
		speed = %d,distance = %d,mets = %d,calories = %d, \
		activity = %d,floorsclimbed = %d, metsactivity = %d\n",
		pedo_client_data->curr_data.stepcount,
		pedo_client_data->curr_data.speed,
		pedo_client_data->curr_data.distance,
		pedo_client_data->curr_data.mets,
		pedo_client_data->curr_data.calories,
		pedo_client_data->curr_data.activity,
		pedo_client_data->curr_data.floorsclimbed,
		pedo_client_data->curr_data.metsactivity);
}


static void m4_set_delay(int delay)
{

}

static void m4_read_pedometer_data(struct pedometer_client *pedo_client_data)
{
	m4sensorhub_reg_read(pedo_client_data->m4sensorhub,
		M4SH_REG_PEDOMETER_ACTIVITY,
		(char *)&pedo_client_data->curr_data.activity);
	m4sensorhub_reg_read(pedo_client_data->m4sensorhub,
		M4SH_REG_PEDOMETER_TOTATDISTANCE,
		(char *)&pedo_client_data->curr_data.distance);
	m4sensorhub_reg_read(pedo_client_data->m4sensorhub,
		M4SH_REG_PEDOMETER_TOTALSTEPS,
		(char *)&pedo_client_data->curr_data.stepcount);
	m4sensorhub_reg_read(pedo_client_data->m4sensorhub,
		M4SH_REG_PEDOMETER_CURRENTSPEED,
		(char *)&pedo_client_data->curr_data.speed);
	m4sensorhub_reg_read(pedo_client_data->m4sensorhub,
		M4SH_REG_METS_METS,
		(char *)&pedo_client_data->curr_data.mets);
	m4sensorhub_reg_read(pedo_client_data->m4sensorhub,
		M4SH_REG_METS_CALORIES,
		(char *)&pedo_client_data->curr_data.calories);
	m4sensorhub_reg_read(pedo_client_data->m4sensorhub,
		M4SH_REG_PEDOMETER_FLOORSCLIMBED,
		(char *)&pedo_client_data->curr_data.floorsclimbed);
	m4sensorhub_reg_read(pedo_client_data->m4sensorhub,
		M4SH_REG_METS_METSACTIVITY,
		(char *)&pedo_client_data->curr_data.metsactivity);
}

static void m4_handle_pedometer_irq(enum m4sensorhub_irqs int_event,
					void *pedometer_data)
{
	struct pedometer_client *pedometer_client_data = pedometer_data;

	m4_read_pedometer_data(pedometer_client_data);
	m4_report_pedometer_inputevent(pedometer_client_data);
}

/*
 * Handle commands from user-space.
 */
static long pedometer_client_ioctl(struct file *filp,
				 unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int flag;
	unsigned char byte;
	void __user *argp = (void __user *)arg;
	struct m4sh_user_profile user;
	struct m4sh_workout_data workout_data;
	struct pedometer_client *pedometer_client_data = filp->private_data;

	switch (cmd) {
	case M4_SENSOR_IOCTL_GET_PEDOMETER:
		m4_read_pedometer_data(pedometer_client_data);
		m4_report_pedometer_inputevent(pedometer_client_data);
		break;
	case M4_SENSOR_IOCTL_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		m4_set_delay(flag);
		break;
	/* TO DO
	Need to implement the following ioctl's when M4 side implementation
	will be ready
	*/
	case M4_SENSOR_IOCTL_SET_POSIX_TIME:
		break;
	case M4_SENSOR_IOCTL_SET_EQUIPMENT_TYPE:
		if (copy_from_user(&byte, argp, sizeof(byte))) {
			printk(KERN_ERR "copy from user returned error eq type\n");
			ret = -EFAULT;
			break;
		}
		m4sensorhub_reg_write(pedometer_client_data->m4sensorhub,
			M4SH_REG_PEDOMETER_EQUIPMENTTYPE, &byte, m4sh_no_mask);
		break;
	case M4_SENSOR_IOCTL_SET_MANUAL_CALIB_WALK_SPEED:
		break;
	case M4_SENSOR_IOCTL_SET_MANUAL_CALIB_JOG_SPEED:
		break;
	case M4_SENSOR_IOCTL_SET_MANUAL_CALIB_RUN_SPEED:
		break;
	case M4_SENSOR_IOCTL_SET_MANUAL_CALIB_STATUS:
		break;
	case M4_SENSOR_IOCTL_SET_USER_PROFILE:
		if (copy_from_user(&user, argp, sizeof(user))) {
			printk(KERN_ERR "copy from user returned error\n");
			ret = -EFAULT;
			break;
		}
		m4sensorhub_reg_write_1byte(pedometer_client_data->m4sensorhub,
			M4SH_REG_USERSETTINGS_USERAGE, user.age, 0xff);
		m4sensorhub_reg_write_1byte(pedometer_client_data->m4sensorhub,
			M4SH_REG_USERSETTINGS_USERGENDER, user.gender, 0xff);
		m4sensorhub_reg_write_1byte(pedometer_client_data->m4sensorhub,
			M4SH_REG_USERSETTINGS_USERHEIGHT, user.height, 0xff);
		m4sensorhub_reg_write_1byte(pedometer_client_data->m4sensorhub,
			M4SH_REG_USERSETTINGS_USERWEIGHT, user.weight, 0xff);
		break;
	case M4_SENSOR_IOCTL_SET_USER_DISTANCE:
		if (copy_from_user(&workout_data, argp, sizeof(workout_data))) {
			printk(KERN_ERR "copy from user returned error\n");
			ret = -EFAULT;
			break;
		}
		m4sensorhub_reg_write(pedometer_client_data->m4sensorhub,
			M4SH_REG_PEDOMETER_USERDISTANCE,
			(unsigned char *)&workout_data.user_distance,
			m4sh_no_mask);
		m4sensorhub_reg_write(pedometer_client_data->m4sensorhub,
			M4SH_REG_PEDOMETER_REPORTEDDISTANCE,
			(unsigned char *)&workout_data.msp_distance,
			m4sh_no_mask);
		break;
	case M4_SENSOR_IOCTL_SET_USER_CALIB_TABLE:
		break;
	case M4_SENSOR_IOCTL_GET_MANUAL_CALIB_STATUS:
		break;
	case M4_SENSOR_IOCTL_ERASE_CALIB:
		break;
	default:
		KDEBUG(M4SH_ERROR, "Invalid IOCTL Command in %s\n", __func__);
		 ret = -EINVAL;
	}
	return ret;
}

static ssize_t m4_pedometer_activity(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pedometer_client *pedo_client_data = platform_get_drvdata(pdev);

	m4_read_pedometer_data(pedo_client_data);
	KDEBUG(M4SH_DEBUG, "%s  : activity = %d\n",
			__func__, pedo_client_data->curr_data.activity);
	return sprintf(buf, "%d \n", pedo_client_data->curr_data.activity);
}

static ssize_t m4_pedometer_distance(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pedometer_client *pedo_client_data = platform_get_drvdata(pdev);

	m4_read_pedometer_data(pedo_client_data);
	KDEBUG(M4SH_DEBUG, "%s  : distance = %d\n",
			__func__, pedo_client_data->curr_data.distance);
	return sprintf(buf, "%d \n", pedo_client_data->curr_data.distance);
}

static ssize_t m4_pedometer_speed(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pedometer_client *pedo_client_data = platform_get_drvdata(pdev);

	m4_read_pedometer_data(pedo_client_data);
	KDEBUG(M4SH_DEBUG, "%s  : speed = %d\n",
			__func__, pedo_client_data->curr_data.speed);
	return sprintf(buf, "%d \n", pedo_client_data->curr_data.speed);
}

static ssize_t m4_pedometer_stepcount(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pedometer_client *pedo_client_data = platform_get_drvdata(pdev);

	m4_read_pedometer_data(pedo_client_data);
	KDEBUG(M4SH_DEBUG, "%s  : stepcount = %d\n",
			__func__, pedo_client_data->curr_data.stepcount);
	return sprintf(buf, "%d \n", pedo_client_data->curr_data.stepcount);
}

static ssize_t m4_pedometer_mets(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pedometer_client *pedo_client_data = platform_get_drvdata(pdev);

	KDEBUG(M4SH_DEBUG, "%s  : mets = %d\n",
			__func__, pedo_client_data->curr_data.mets);
	return sprintf(buf, "%d \n", pedo_client_data->curr_data.mets);
}

static ssize_t m4_pedometer_calories(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pedometer_client *pedo_client_data = platform_get_drvdata(pdev);

	KDEBUG(M4SH_DEBUG, "%s  : calories = %d\n",
			__func__, pedo_client_data->curr_data.calories);
	return sprintf(buf, "%d \n", pedo_client_data->curr_data.calories);
}

static ssize_t m4_pedometer_floorsclimbed(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pedometer_client *pedo_client_data = platform_get_drvdata(pdev);

	m4_read_pedometer_data(pedo_client_data);
	KDEBUG(M4SH_DEBUG, "%s  : floorsclimbed = %d\n",
			__func__, pedo_client_data->curr_data.floorsclimbed);
	return sprintf(buf, "%d\n", pedo_client_data->curr_data.floorsclimbed);
}

static ssize_t m4_pedometer_metsactivity(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pedometer_client *pedo_client_data = platform_get_drvdata(pdev);

	KDEBUG(M4SH_DEBUG, "%s  : metsactivity = %d\n",
			__func__, pedo_client_data->curr_data.metsactivity);
	return sprintf(buf, "%d\n", pedo_client_data->curr_data.metsactivity);
}

static DEVICE_ATTR(activity, 0444, m4_pedometer_activity, NULL);
static DEVICE_ATTR(distance, 0444, m4_pedometer_distance, NULL);
static DEVICE_ATTR(speed, 0444, m4_pedometer_speed, NULL);
static DEVICE_ATTR(stepcount, 0444, m4_pedometer_stepcount, NULL);
static DEVICE_ATTR(mets, 0444, m4_pedometer_mets, NULL);
static DEVICE_ATTR(calories, 0444, m4_pedometer_calories, NULL);
static DEVICE_ATTR(floorsclimbed, 0444, m4_pedometer_floorsclimbed, NULL);
static DEVICE_ATTR(metsactivity, 0444, m4_pedometer_metsactivity, NULL);

static const struct file_operations pedometer_client_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = pedometer_client_ioctl,
	.open  = pedometer_client_open,
	.release = pedometer_client_close,
};

static struct miscdevice pedometer_client_miscdrv = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = PEDOMETER_CLIENT_DRIVER_NAME,
	.fops = &pedometer_client_fops,
};

static int pedometer_driver_init(struct init_calldata *p_arg)
{
	int ret;
	struct m4sensorhub_data *m4sensorhub = p_arg->p_m4sensorhub_data;

	ret = m4sensorhub_irq_register(m4sensorhub,
					M4SH_IRQ_PEDOMETER_DATA_READY,
					m4_handle_pedometer_irq,
					misc_pedometer_data);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering m4 int %d (%d)\n",
			M4SH_IRQ_PEDOMETER_DATA_READY, ret);
		return ret;
	}
	ret = m4sensorhub_irq_register(m4sensorhub,
					M4SH_IRQ_ACTIVITY_CHANGE,
					m4_handle_pedometer_irq,
					misc_pedometer_data);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering m4 int %d (%d)\n",
			M4SH_IRQ_ACTIVITY_CHANGE, ret);
		goto exit1;
	}
	ret = m4sensorhub_irq_enable(m4sensorhub, M4SH_IRQ_ACTIVITY_CHANGE);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error enabling m4 int %d (%d)\n",
			M4SH_IRQ_ACTIVITY_CHANGE, ret);
		goto exit;
	}

	return ret;
exit:
	m4sensorhub_irq_unregister(m4sensorhub, M4SH_IRQ_ACTIVITY_CHANGE);
exit1:
	m4sensorhub_irq_unregister(m4sensorhub, M4SH_IRQ_PEDOMETER_DATA_READY);
	return ret;
}

static int pedometer_client_probe(struct platform_device *pdev)
{
	int ret = -1;
	struct pedometer_client *pedometer_client_data;
	struct m4sensorhub_data *m4sensorhub = m4sensorhub_client_get_drvdata();

	if (!m4sensorhub)
		return -EFAULT;

	pedometer_client_data = kzalloc(sizeof(*pedometer_client_data),
						GFP_KERNEL);
	if (!pedometer_client_data)
		return -ENOMEM;

	pedometer_client_data->m4sensorhub = m4sensorhub;
	platform_set_drvdata(pdev, pedometer_client_data);

	pedometer_client_data->prev_data.stepcount = 0;
	pedometer_client_data->prev_data.distance = 0;
	pedometer_client_data->prev_data.activity = 0;
	pedometer_client_data->prev_data.speed = 0;
	pedometer_client_data->prev_data.floorsclimbed = 0;

	pedometer_client_data->input_dev = input_allocate_device();
	if (!pedometer_client_data->input_dev) {
		ret = -ENOMEM;
		KDEBUG(M4SH_ERROR, "%s: input device allocate failed: %d\n",
			__func__, ret);
		goto free_mem;
	}

	pedometer_client_data->input_dev->name = PEDOMETER_CLIENT_DRIVER_NAME;
	set_bit(EV_MSC, pedometer_client_data->input_dev->evbit);
	set_bit(MSC_ACTIVITY_TYPE, pedometer_client_data->input_dev->mscbit);
	set_bit(MSC_STEPCOUNT, pedometer_client_data->input_dev->mscbit);
	set_bit(MSC_SPEED, pedometer_client_data->input_dev->mscbit);
	set_bit(MSC_DISTANCE, pedometer_client_data->input_dev->mscbit);
	set_bit(MSC_METS, pedometer_client_data->input_dev->mscbit);
	set_bit(MSC_CALORIES, pedometer_client_data->input_dev->mscbit);
	set_bit(MSC_FLOORSCLIMBED, pedometer_client_data->input_dev->mscbit);
	set_bit(MSC_METSACTIVITY, pedometer_client_data->input_dev->mscbit);

	if (input_register_device(pedometer_client_data->input_dev)) {
		KDEBUG(M4SH_ERROR, "%s: input device register failed\n",
			__func__);
		input_free_device(pedometer_client_data->input_dev);
		goto free_mem;
	}

	ret = misc_register(&pedometer_client_miscdrv);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Error registering %s driver\n", __func__);
		goto unregister_input_device;
	}
	misc_pedometer_data = pedometer_client_data;
	ret = m4sensorhub_register_initcall(pedometer_driver_init,
					pedometer_client_data);
	if (ret < 0) {
		KDEBUG(M4SH_ERROR, "Unable to register init function "
			"for pedometer client = %d\n", ret);
		goto unregister_misc_device;
	}
	if (device_create_file(&pdev->dev, &dev_attr_activity)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n", __func__);
		ret = -1;
		goto unregister_initcall;
	}
	if (device_create_file(&pdev->dev, &dev_attr_distance)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n", __func__);
		ret = -1;
		goto remove_activity_device_file;
	}
	if (device_create_file(&pdev->dev, &dev_attr_speed)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n", __func__);
		ret = -1;
		goto remove_distance_device_file;
	}
	if (device_create_file(&pdev->dev, &dev_attr_stepcount)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n", __func__);
		ret = -1;
		goto remove_speed_device_file;
	}
	if (device_create_file(&pdev->dev, &dev_attr_mets)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n", __func__);
		ret = -1;
		goto remove_stepcount_device_file;
	}
	if (device_create_file(&pdev->dev, &dev_attr_calories)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n", __func__);
		ret = -1;
		goto remove_mets_device_file;
	}
	if (device_create_file(&pdev->dev, &dev_attr_floorsclimbed)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n", __func__);
		ret = -1;
		goto remove_cals_device_file;
	}
	if (device_create_file(&pdev->dev, &dev_attr_metsactivity)) {
		KDEBUG(M4SH_ERROR, "Error creating %s sys entry\n", __func__);
		ret = -1;
		goto remove_floorsclimbed_device_file;
	}
	KDEBUG(M4SH_INFO, "Initialized %s driver\n", __func__);
	return 0;

remove_floorsclimbed_device_file:
	device_remove_file(&pdev->dev, &dev_attr_floorsclimbed);
remove_cals_device_file:
	device_remove_file(&pdev->dev, &dev_attr_calories);
remove_mets_device_file:
	device_remove_file(&pdev->dev, &dev_attr_mets);
remove_stepcount_device_file:
	device_remove_file(&pdev->dev, &dev_attr_stepcount);
remove_speed_device_file:
	device_remove_file(&pdev->dev, &dev_attr_speed);
remove_distance_device_file:
	device_remove_file(&pdev->dev, &dev_attr_distance);
remove_activity_device_file:
	device_remove_file(&pdev->dev, &dev_attr_activity);
unregister_initcall:
	m4sensorhub_unregister_initcall(pedometer_driver_init);
unregister_misc_device:
	misc_pedometer_data = NULL;
	misc_deregister(&pedometer_client_miscdrv);
unregister_input_device:
	input_unregister_device(pedometer_client_data->input_dev);
free_mem:
	platform_set_drvdata(pdev, NULL);
	pedometer_client_data->m4sensorhub = NULL;
	kfree(pedometer_client_data);
	pedometer_client_data = NULL;
	return ret;
}

static int __exit pedometer_client_remove(struct platform_device *pdev)
{
	struct pedometer_client *pedometer_client_data =
						platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_mets);
	device_remove_file(&pdev->dev, &dev_attr_calories);
	device_remove_file(&pdev->dev, &dev_attr_stepcount);
	device_remove_file(&pdev->dev, &dev_attr_speed);
	device_remove_file(&pdev->dev, &dev_attr_distance);
	device_remove_file(&pdev->dev, &dev_attr_activity);
	device_remove_file(&pdev->dev, &dev_attr_floorsclimbed);
	device_remove_file(&pdev->dev, &dev_attr_metsactivity);

	m4sensorhub_irq_unregister(pedometer_client_data->m4sensorhub,
				M4SH_IRQ_PEDOMETER_DATA_READY);
	m4sensorhub_irq_disable(pedometer_client_data->m4sensorhub,
				M4SH_IRQ_ACTIVITY_CHANGE);
	m4sensorhub_irq_unregister(pedometer_client_data->m4sensorhub,
				M4SH_IRQ_ACTIVITY_CHANGE);
	m4sensorhub_unregister_initcall(pedometer_driver_init);
	misc_pedometer_data = NULL;
	misc_deregister(&pedometer_client_miscdrv);
	input_unregister_device(pedometer_client_data->input_dev);
	platform_set_drvdata(pdev, NULL);
	pedometer_client_data->m4sensorhub = NULL;
	kfree(pedometer_client_data);
	pedometer_client_data = NULL;
	return 0;
}

static void pedometer_client_shutdown(struct platform_device *pdev)
{
	return;
}
#ifdef CONFIG_PM
static int pedometer_client_suspend(struct platform_device *pdev,
				pm_message_t message)
{
	return 0;
}

static int pedometer_client_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define pedometer_client_suspend NULL
#define pedometer_client_resume  NULL
#endif


static struct of_device_id m4pedometer_match_tbl[] = {
	{ .compatible = "mot,m4pedometer" },
	{},
};


static struct platform_driver pedometer_client_driver = {
	.probe		= pedometer_client_probe,
	.remove		= __exit_p(pedometer_client_remove),
	.shutdown	= pedometer_client_shutdown,
	.suspend	= pedometer_client_suspend,
	.resume		= pedometer_client_resume,
	.driver		= {
		.name	= PEDOMETER_CLIENT_DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(m4pedometer_match_tbl),
	},
};

static int __init pedometer_client_init(void)
{
	return platform_driver_register(&pedometer_client_driver);
}

static void __exit pedometer_client_exit(void)
{
	platform_driver_unregister(&pedometer_client_driver);
}

module_init(pedometer_client_init);
module_exit(pedometer_client_exit);

MODULE_ALIAS("platform:pedometer_client");
MODULE_DESCRIPTION("M4 Sensor Hub Pedometer client driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

