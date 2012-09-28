/*
 * drivers/media/video/omap/hplens.c
 *
 * HP Generic Driver : Driver implementation for generic lens module.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
 *
 * Contributors:
 *
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>
#include <plat/gpio.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/fs.h>

#include "omap34xxcam.h"
#include "hplens.h"

#define DRIVER_NAME  "hplens"
#define HPLENS_DRV_NAME	"hplens-i2c"
#define HPLENS_DRV_SYSFS	"hplens-omap"

static int hplens_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
static int __exit hplens_i2c_remove(struct i2c_client *client);

static DEFINE_MUTEX(hplens_mutex);

struct hplens_device {
	const struct hplens_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	int opened;
	int state;
	int power_state;
};

struct hplens_dev {
	struct device *dev;
};

/**
 * Global variables.
 **/
static int hplens_major = -1;
static struct hplens_dev *g_device;
static struct class *hplens_class;

static const struct i2c_device_id hplens_id[] = {
	{ HPLENS_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hplens_id);


/**
 * struct hplens_fh - Per-filehandle data structure
 * @device: hplens device reference.
 **/
struct hplens_fh {
	struct hplens_dev *device;
};

static struct i2c_driver hplens_i2c_driver = {
	.driver = {
		.name = HPLENS_NAME,
		.owner = THIS_MODULE,
	},
	.probe = hplens_i2c_probe,
	.remove = __exit_p(hplens_i2c_remove),
	.id_table = hplens_id,
};

static struct hplens_device hplens_dev = {
	.state = LENS_NOT_DETECTED,
	.power_state = 0,
};

static struct vcontrol {
	struct v4l2_queryctrl qc;
} video_control[] = {
   {
		{
		.id = V4L2_CID_HPLENS_CMD_READ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Lens Cmd Read",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		},
   },
   {
      {
		.id = V4L2_CID_HPLENS_CMD_WRITE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Lens Cmd Write",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		}
   },
   {
      {
		.id = V4L2_CID_HPLENS_CMD_READ_PAGE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Lens Cmd Read Page",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		}
   },
   {
      {
		.id = V4L2_CID_HPLENS_CMD_WRITE_PAGE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Lens Cmd Write Page",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		}
   },
   {
      {
		.id = V4L2_CID_HPLENS_CMD_CAL_READ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Lens Cmd Cal Read",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		}
   }
};

/**
 * hplens_find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
int hplens_find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/**
 * hplens_reg_read - Reads a value from a register in an I2C driver device.
 * @client: Pointer to structure of I2C client.
 * @value: Pointer to u16 for returning value of register to read.
 *
 * Returns zero if successful, or non-zero otherwise.
 **/
int hplens_reg_read(u8 dev_addr, u8 *value, u16 len)
{
	struct hplens_device *lens = &hplens_dev;
	struct i2c_client *client = lens->i2c_client;
	int err;
	struct i2c_msg msg[1];

	if (!client->adapter)
		return -ENODEV;

	client->addr = dev_addr;  /* set slave address */

	msg->addr = client->addr;
	msg->flags = I2C_M_RD;
	msg->len = len;
	msg->buf = value;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0)
		return 0;

	return err;
}

/**
 * hplens_reg_write - Writes a value to a register in LENS Coil driver device.
 * @client: Pointer to structure of I2C client.
 * @value: Value of register to write.
 *
 * Returns zero if successful, or non-zero otherwise.
 **/
int hplens_reg_write(u8 dev_addr, u8 *write_buf, u16 len)
{
	struct hplens_device *lens = &hplens_dev;
	struct i2c_client *client = lens->i2c_client;
	int err;
	struct i2c_msg msg[1];

	if (!client->adapter)
		return -ENODEV;

	client->addr = dev_addr;  /* set slave address */

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = len;
	msg->buf = write_buf;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0)
		return 0;

	return err;
}

void hplens_lock(void)
{
	mutex_lock(&hplens_mutex);
}
EXPORT_SYMBOL(hplens_lock);

void hplens_unlock(void)
{
	mutex_unlock(&hplens_mutex);
}
EXPORT_SYMBOL(hplens_unlock);

/**
 * hplens_ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int hplens_ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct hplens_device *lens = s->priv;

	if (lens->pdata->priv_data_set)
		return lens->pdata->priv_data_set(p);

	return -1;
}

 /**
 * hplens_ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int hplens_ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct hplens_device *lens = s->priv;

	if (lens->pdata->power_set)
		lens->pdata->power_set(on);

	/* May want to replace with api_ReportMotorType(); */

	lens->power_state = on;
	return 0;
}

/**
 * hplens_ioctl_queryctrl - V4L2 lens interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int hplens_ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qc)
{
	int i;

	i = hplens_find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;

	return 0;
}

/**
 * __hplens_ioctl - Internal ioctl handler.
 **/
static int __hplens_ioctl(unsigned int cmd, void *arg)
{
	int err = -1;
	struct hplens_reg reg;
	struct hplens_eeprom *eeprom = NULL;
	u8 write_buffer[20];
	u32 i;
	u16 write_len;

	switch (cmd) {
	case OMAP3_HPLENS_CMD_READ:
	{
		err = copy_from_user(&reg, arg,  sizeof(struct hplens_reg));
		if (err == 0) {
			mutex_lock(&hplens_mutex);
			if (reg.addr[0] != 0xff) {   /* valid register address */
				/* write the register address to read */
				err = hplens_reg_write(reg.dev_addr, reg.addr, reg.len_addr);
			}

			if (err == 0) {
				/* Read the register */
				err = hplens_reg_read(reg.dev_addr, reg.data, reg.len_data);
			}

			/* Save time stamp */
			ktime_get_ts(&reg.ts_end);
			mutex_unlock(&hplens_mutex);

			if (err == 0)
				err = copy_to_user(arg, &reg, sizeof(struct hplens_reg));
		}
		break;
	}

	case OMAP3_HPLENS_CMD_WRITE:
	{
		err = copy_from_user(&reg, arg,  sizeof(struct hplens_reg));
		if (err == 0) {
			mutex_lock(&hplens_mutex);

			if ((reg.len_addr + reg.len_data) > sizeof(write_buffer)) {
				err = -EINVAL;
				mutex_unlock(&hplens_mutex);
				break;
			}

			/* Initialize length of write buffer. */
			write_len = 0;

			 /* The following check is a temporary HACK to allow not
					writing address for certain parts. */
			if (reg.addr[0] != 0xff) {
				write_len = reg.len_addr;
				for (i = 0; i < reg.len_addr; ++i) {
					write_buffer[i] = reg.addr[i];
				}
			}
			for (i = 0; i < reg.len_data; ++i) {
				write_buffer[i + write_len] = reg.data[i];
			}
			write_len += reg.len_data;
			err = hplens_reg_write(reg.dev_addr, write_buffer, write_len);

			/* Save time stamp */
			ktime_get_ts(&reg.ts_end);
			mutex_unlock(&hplens_mutex);

			if (err == 0)
				err = copy_to_user(arg, &reg, sizeof(struct hplens_reg));
		}
		break;
	}

	case OMAP3_HPLENS_CMD_READ_CAL:
	{
		/* Using dynamic memory. */
		eeprom = kmalloc(sizeof(struct hplens_eeprom), GFP_KERNEL);
		if (eeprom == NULL)
			return -EINVAL;

		err = copy_from_user(eeprom, arg,  sizeof(struct hplens_eeprom));
		if (err == 0) {
			mutex_lock(&hplens_mutex);
			if (eeprom->addr[0] != 0xff) {   /* valid register address */
				/* write the register address to read */
				err = hplens_reg_write(eeprom->dev_addr, eeprom->addr, eeprom->len_addr);
			}

			if (err == 0) {
				/* Read the register */
				err = hplens_reg_read(eeprom->dev_addr, eeprom->data, eeprom->len_data);
			}

			/* Save time stamp */
			ktime_get_ts(&(eeprom->ts_end));
			mutex_unlock(&hplens_mutex);

			if (err == 0)
				err = copy_to_user(arg, eeprom, sizeof(struct hplens_eeprom));
		}

		/* clean up. */
		if (eeprom != NULL)
			kfree(eeprom);
		break;
	}

	case OMAP3_HPLENS_CMD_READ_PAGE:
		/* TODO: Implement */
		break;
	case OMAP3_HPLENS_CMD_WRITE_PAGE:
		/* TODO: Implement */
		break;
	default:
		break;
	};

	return err;
}

/**
 * hplens_ioctl_s_ctrl - V4L2 lens interface handler for  VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the imx046sensor_video_control[] array).
 * Otherwise, * returns -EINVAL if the control is not supported.
 */
static int hplens_ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	int ret = -EINVAL;

	if (hplens_find_vctrl(vc->id) < 0)
		return -EINVAL;

	switch (vc->id) {
	case V4L2_CID_HPLENS_CMD_READ:
		ret = __hplens_ioctl(OMAP3_HPLENS_CMD_READ, (void *)vc->value);
		break;
	case V4L2_CID_HPLENS_CMD_WRITE:
		ret = __hplens_ioctl(OMAP3_HPLENS_CMD_WRITE, (void *)vc->value);
		break;
	case V4L2_CID_HPLENS_CMD_READ_PAGE:
		ret = __hplens_ioctl(OMAP3_HPLENS_CMD_READ_PAGE, (void *)vc->value);
		break;
	case V4L2_CID_HPLENS_CMD_WRITE_PAGE:
		ret = __hplens_ioctl(OMAP3_HPLENS_CMD_WRITE_PAGE, (void *)vc->value);
		break;
	case V4L2_CID_HPLENS_CMD_CAL_READ:
		ret = __hplens_ioctl(OMAP3_HPLENS_CMD_READ_CAL, (void *)vc->value);
		break;
	}

	return ret;
}

static struct v4l2_int_ioctl_desc hplens_ioctl_desc[] = {
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)hplens_ioctl_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)hplens_ioctl_g_priv },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)hplens_ioctl_queryctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)hplens_ioctl_s_ctrl },
};

static struct v4l2_int_slave hplens_slave = {
	.ioctls = hplens_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(hplens_ioctl_desc),
};

static struct v4l2_int_device hplens_int_device = {
	.module = THIS_MODULE,
	.name = DRIVER_NAME,
	.priv = &hplens_dev,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &hplens_slave,
	},
};

/**
 * lens_probe - Probes the driver for valid I2C attachment.
 * @client: Pointer to structure of I2C client.
 *
 * Returns 0 if successful, or -EBUSY if unable to get client attached data.
 **/
static int hplens_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct hplens_device *lens = &hplens_dev;
	int err;

	dev_info(&client->dev, "lens probe called....\n");

	if (i2c_get_clientdata(client)) {
		printk(KERN_ERR " DTA BUSY %s\n", client->name);
		return -EBUSY;
	}

	lens->pdata = client->dev.platform_data;

	if (!lens->pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	lens->v4l2_int_device = &hplens_int_device;

	lens->i2c_client = client;
	i2c_set_clientdata(client, lens);

	err = v4l2_int_device_register(lens->v4l2_int_device);
	if (err) {
		printk(KERN_ERR "Failed to Register " DRIVER_NAME " as V4L2 device.\n");
		i2c_set_clientdata(client, NULL);
	} else {
		printk(KERN_ERR "Registered " DRIVER_NAME " as V4L2 device.\n");
	}

	return 0;
}

 /**
 * lens_remove - Routine when device its unregistered from I2C
 * @client: Pointer to structure of I2C client.
 *
 * Returns 0 if successful, or -ENODEV if the client isn't attached.
 **/
static int __exit hplens_i2c_remove(struct i2c_client *client)
{
	if (!client->adapter)
		return -ENODEV;

	i2c_set_clientdata(client, NULL);
	return 0;
}

/**
 * hplens_open - Initializes and opens the hplens device
 * @inode: Inode structure associated with hplens driver
 * @filp: File structure associated with the hplens driver
 *
 * Returns 0 if successful, -EBUSY if its already opened or the ISP module is
 * not available, or -ENOMEM if its unable to allocate the device in kernel
 * space memory.
 **/
static int hplens_open(struct inode *inode, struct file *file)
{
	struct hplens_fh *fh;

	/* dev_info(g_device->dev , "open\n"); */

	fh = kzalloc(sizeof(struct hplens_fh), GFP_KERNEL);
	if (unlikely(fh == NULL))
		return -ENOMEM;

	/* Save context in file handle. */
	fh->device = g_device;
	file->private_data = fh;

	return 0;
}

/**
 * hplens_release - Releases hplens device and frees up allocated memory
 * @inode: Inode structure associated with the hp3a driver
 * @filp: File structure associated with the hp3a driver
 *
 * Returns 0 if successful, or -EBUSY if channel is being used.
 **/
static int hplens_release(struct inode *inode, struct file *file)
{
	struct hplens_fh *fh = file->private_data;

	/* dev_info(g_device->dev , "release\n"); */

	/* Releasing session specific data. */
	file->private_data = NULL;
	kfree(fh);

	return 0;
}

/**
 * hplens_mmap - Memory maps hplens module.
 * @file: File structure associated with the hplens driver
 * @vma: Virtual memory area structure.
 *
 * Returns 0 if successful, or returned value by the videobuf_mmap_mapper()
 * function
 **/
static int hplens_mmap(struct file *file, struct vm_area_struct *vma)
{
	return -EINVAL;
}

 /**
 * hplens_unlocked_ioctl - I/O control function for hplens module
 * @inode: Inode structure associated with the hplens Wrapper.
 * @file: File structure associated with the hplens driver.
 * @cmd: Type of command to execute.
 * @arg: Argument to send to requested command.
 *
 * Returns 0 if successful, -1 if bad command passed or access is denied,
 * -EFAULT if copy_from_user() or copy_to_user()  fails,
 * -EINVAL if parameter validation fails or parameter structure is not present.
 **/
long hplens_unlocked_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct hplens_fh *fh = file->private_data;
	struct hplens_dev *device = fh->device;

	if (unlikely(_IOC_TYPE(cmd) != OMAP3_HPLENS_MAGIC)) {
		dev_err(device->dev, "Bad command value (%d)\n", cmd);
		return  -ENOTTY;
	}

	return __hplens_ioctl(cmd, (void *)arg);
}

/**
 * hplens_platform_release - Place holder
 * @device: Structure containing hp3a driver global information
 *
 * This is called when the reference count goes to zero.
 **/
static void hplens_platform_release(struct device *device)
{
}

static struct file_operations hplens_fops = {
	.owner = THIS_MODULE,
	.open = hplens_open,
	.release = hplens_release,
	.mmap = hplens_mmap,
	.unlocked_ioctl = hplens_unlocked_ioctl,
};

static struct platform_device __hplens_device = {
	.name = HPLENS_DRV_NAME,
	.id = -1,
	.dev = {
		.release = hplens_platform_release,
	}
};

/**
 * lens_init - Module initialisation.
 *
 * Returns 0 if successful, or -EINVAL if device couldn't be initialized, or
 * added as a character device.
 **/
static int __init hplens_init(void)
{
	int err = -EINVAL;
	struct hplens_dev *device;

	device = kzalloc(sizeof(struct hplens_dev), GFP_KERNEL);
	if (!device) {
		dev_err(0 , HPLENS_DRV_NAME ": could not allocate memory\n");
		return -ENOMEM;
	}
	hplens_major = register_chrdev(0, HPLENS_DRV_SYSFS, &hplens_fops);
	if (hplens_major < 0) {
		dev_err(device->dev , "initialization failed. could"
				" not register character device\n");
		err = -ENODEV;
		goto exit_error_1;
	}
	err = platform_device_register(&__hplens_device);
	if (err) {
		dev_err(device->dev , "Failed to register platform device!\n");
		goto exit_error_2;
	}
	hplens_class = class_create(THIS_MODULE, HPLENS_DRV_NAME);
	if (!hplens_class) {
		dev_err(device->dev , "Failed to create class!\n");
		goto exit_error_3;
	}
	/* make entry in the devfs */
	device->dev = device_create(hplens_class, device->dev,
		MKDEV(hplens_major, 0), NULL, HPLENS_DRV_SYSFS);
	/* Save device instance. */
	g_device = device;

	err = i2c_add_driver(&hplens_i2c_driver);
	if (err)
		goto exit_error_4;
	return err;

exit_error_4:
	class_destroy(hplens_class);
exit_error_3:
	platform_device_unregister(&__hplens_device);
exit_error_2:
	unregister_chrdev(hplens_major,  HPLENS_DRV_SYSFS);
	hplens_major = -1;
exit_error_1:
	kfree(device);
	g_device = NULL;
	return err;
}
late_initcall(hplens_init);

/**
 * lens_cleanup - Module cleanup.
 **/
static void __exit hplens_cleanup(void)
{
	i2c_del_driver(&hplens_i2c_driver);
	class_destroy(hplens_class);
	platform_device_unregister(&__hplens_device);
	unregister_chrdev(hplens_major,  HPLENS_DRV_NAME);
	kfree(g_device);
	hplens_major = -1;
}
module_exit(hplens_cleanup);

MODULE_AUTHOR("Hewlett-Packard Co.");
MODULE_DESCRIPTION("HP Generic Lens Driver");
MODULE_LICENSE("GPL");
