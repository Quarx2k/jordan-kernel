/*
 * Copyright (C) 2007-2008 Motorola, Inc
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Changelog:
 * Date               Author           Comment
 * -----------------------------------------------------------------------------
 * 12/07/2007      Motorola        USB-IPC initial
 * 03/22/2007      Motorola        USB-IPC header support
 * 05/09/2008      Motorola        Change Copyright and Changelog
 *
 */

/*!
 * @file drivers/usb/ipchost/ipc_api_test.c
 * @brief USB-IPC test Set
 *
 * This is the generic portion of the USB-IPC driver.
 *
 * @ingroup IPCFunction
 */



/*
 * Include Files
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/usb_ipc.h>
#include <linux/ipc_api.h>


/* Module */
MODULE_DESCRIPTION("OMAP SAM IPC Test Module");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

#define DEBUG(args...) /* printk(args) */
#define ENTER_FUNC() DEBUG("Enter: %s\n", __func__)

#define API_DATA_CH_MINOR_NUM  240 /*  */
#define API_SHORT_CH_MINOR_NUM 241 /*  */

#define MAX_DATA_WRITE_BUF_SIZE MAX_FRAME_SIZE
#define MAX_DATA_READ_BUF_SIZE  MAX_FRAME_SIZE

HW_CTRL_IPC_CHANNEL_T *ipc_data_ch;
HW_CTRL_IPC_OPEN_T ipc_data_ch_desc;
HW_CTRL_IPC_OPEN_T ipc_short_ch_desc;

IPC_CHANNEL_ACCESS_TYPE ipc_data_ch_read_type = HW_CTRL_IPC_READ_TYPE;
IPC_CHANNEL_ACCESS_TYPE ipc_data_ch_write_type = HW_CTRL_IPC_WRITE_TYPE;

IPC_CHANNEL_ACCESS_TYPE ipc_short_ch_read_type = HW_CTRL_IPC_READ_TYPE;
IPC_CHANNEL_ACCESS_TYPE ipc_short_ch_write_type = HW_CTRL_IPC_WRITE_TYPE;

int ipc_data_read_actual_length;
int ipc_data_write_actual_length;
unsigned char ipc_data_read_buffer[MAX_FRAME_NUM][MAX_DATA_READ_BUF_SIZE];
unsigned char ipc_data_write_buffer[MAX_FRAME_NUM][MAX_DATA_WRITE_BUF_SIZE];

static struct semaphore ipc_data_read_wait;
static struct semaphore ipc_data_write_wait;

#define NODE_DESCRIPTOR_END_BIT  0x4000
#define NODE_DESCRIPTOR_LAST_BIT 0x8000
#define NODE_DESCRIPTOR_DONE_BIT 0x2000

/* file operation for USB DATA test */
static int ipc_api_data_open(struct inode *inode, struct file *file)
{
	ENTER_FUNC();

	if (ipc_data_ch != NULL)
		return -EBUSY;

	ipc_data_ch = hw_ctrl_ipc_open(&ipc_data_ch_desc);
	if (ipc_data_ch == NULL)
		return -ENOMEM;

	return 0;
}

static int ipc_api_data_release(struct inode *inode, struct file *file)
{
	ENTER_FUNC();
	hw_ctrl_ipc_close(ipc_data_ch);
	ipc_data_ch = NULL;
	return 0;
}

static ssize_t ipc_api_data_read(struct file *filp, char __user * buf,
				 size_t count, loff_t *l)
{
	HW_CTRL_IPC_STATUS_T status;
	int size, ret, num;
	HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T temp_desc[MAX_FRAME_NUM];
	int i;

	ENTER_FUNC();

	ipc_data_read_actual_length = 0;
	size = count > MAX_DATA_READ_BUF_SIZE ? MAX_DATA_READ_BUF_SIZE : count;

	for (i = 0; i < MAX_FRAME_NUM; i++) {
		temp_desc[i].comand = 0;
		temp_desc[i].length = MAX_DATA_READ_BUF_SIZE;
		temp_desc[i].data_ptr = &(ipc_data_read_buffer[i][0]);
	}
	temp_desc[MAX_FRAME_NUM - 1].comand = NODE_DESCRIPTOR_END_BIT;
	status = hw_ctrl_ipc_read_ex2(ipc_data_ch, &temp_desc[0]);
	if (status == HW_CTRL_IPC_STATUS_OK)
		down(&ipc_data_read_wait);

	num = ipc_data_read_actual_length;
	if (num > count)
		num = count;

	ret = copy_to_user(buf, ipc_data_read_buffer, num);
	return ipc_data_read_actual_length;
}

static ssize_t ipc_api_data_write(struct file *filp, const char *buf,
				  size_t count, loff_t *l)
{
	HW_CTRL_IPC_STATUS_T status = HW_CTRL_IPC_STATUS_ERROR;
	int i, j;
	unsigned short temp_buf[MAX_FRAME_NUM + 1];
	HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T temp_desc[MAX_FRAME_NUM];
	int ret;

	ENTER_FUNC();

	ipc_data_write_actual_length = 0;
	ret = copy_from_user((void *)&temp_buf[0], buf, 2*MAX_FRAME_NUM+2);

	for (i = 0; (i < MAX_FRAME_NUM) && (i < temp_buf[0]); i++) {
		temp_desc[i].comand = NODE_DESCRIPTOR_LAST_BIT;
		temp_desc[i].length = temp_buf[i + 1];
		for (j = 0; j < temp_buf[i + 1]; j++)
			ipc_data_write_buffer[i][j] = j & 0xff;

		temp_desc[i].data_ptr = &(ipc_data_write_buffer[i][0]);
	}
	if (i > 0) {
		temp_desc[i - 1].comand =
		    NODE_DESCRIPTOR_END_BIT | NODE_DESCRIPTOR_LAST_BIT;
	}
	status = hw_ctrl_ipc_write_ex2(ipc_data_ch, &(temp_desc[0]));
	if (status == HW_CTRL_IPC_STATUS_OK)
		down(&ipc_data_write_wait);

	return ipc_data_write_actual_length;
}

static int ipc_api_data_ioctl(struct inode *inode, struct file *file,
			      unsigned int cmd, unsigned long arg)
{
	if (arg == 0x01) {
		ipc_data_ch_read_type = HW_CTRL_IPC_READ_TYPE;
	} else if (arg == 0x02) {
		ipc_data_ch_read_type = HW_CTRL_IPC_READ_EX2_TYPE;
	} else if (arg == 0x11) {
		ipc_data_ch_write_type = HW_CTRL_IPC_WRITE_TYPE;
	} else if (arg == 0x12) {
		ipc_data_ch_write_type = HW_CTRL_IPC_WRITE_EX_CONT_TYPE;
	} else if (arg == 0x13) {
		ipc_data_ch_write_type = HW_CTRL_IPC_WRITE_EX_LIST_TYPE;
	} else if (arg == 0x14) {
		ipc_data_ch_write_type = HW_CTRL_IPC_WRITE_EX2_TYPE;
	} else {
		printk(KERN_ERR "%s: Error IOCTL cmd\n", __func__);
		return -EIO;
	}

	return 0;
}

/* file operation struction for IPC API DATA channel */
static const struct file_operations ipc_api_data_fops = {
	.owner   = THIS_MODULE,
	.open    = ipc_api_data_open,
	.release = ipc_api_data_release,
	.read    = ipc_api_data_read,
	.ioctl   = ipc_api_data_ioctl,
	.write   = ipc_api_data_write,
};
static struct miscdevice ipc_api_data_device = {
	API_DATA_CH_MINOR_NUM, "ipc_api_data", &ipc_api_data_fops
};

/* file operation for USB DATA test */
static int ipc_api_short_open(struct inode *inode, struct file *file)
{
	ENTER_FUNC();
	return 0;
}

static int ipc_api_short_release(struct inode *inode, struct file *file)
{
	ENTER_FUNC();
	return 0;
}

static ssize_t ipc_api_short_read(struct file *filp, char __user * buf,
				  size_t count, loff_t *l)
{
	ENTER_FUNC();
	return 0;
}

static ssize_t ipc_api_short_write(struct file *filp, const char *buf,
				   size_t count, loff_t *l)
{
	ENTER_FUNC();
	return 0;
}

static int ipc_api_short_ioctl(struct inode *inode, struct file *file,
			       unsigned int cmd, unsigned long arg)
{
	ENTER_FUNC();
	return 0;
}

/* file operation struction for IPC API SHORT channel */
static const struct file_operations ipc_api_short_fops = {
	.owner   = THIS_MODULE,
	.open    = ipc_api_short_open,
	.release = ipc_api_short_release,
	.read    = ipc_api_short_read,
	.write   = ipc_api_short_write,
	.ioctl   = ipc_api_short_ioctl,
};
static struct miscdevice ipc_api_short_device = {
	API_SHORT_CH_MINOR_NUM, "ipc_api_short", &ipc_api_short_fops
};

void shortmsg_read_callback(HW_CTRL_IPC_READ_STATUS_T *status)
{
	ENTER_FUNC();
}

void shortmsg_write_callback(HW_CTRL_IPC_WRITE_STATUS_T *status)
{
	ENTER_FUNC();
}

void shortmsg_notify_callback(HW_CTRL_IPC_NOTIFY_STATUS_T *status)
{
	ENTER_FUNC();
}

void data_read_callback(HW_CTRL_IPC_READ_STATUS_T *status)
{
	ENTER_FUNC();
	ipc_data_read_actual_length = status->nb_bytes;
	up(&ipc_data_read_wait);
}

void data_write_callback(HW_CTRL_IPC_WRITE_STATUS_T *status)
{
	ENTER_FUNC();
	ipc_data_write_actual_length = status->nb_bytes;
	up(&ipc_data_write_wait);
}

void data_notify_callback(HW_CTRL_IPC_NOTIFY_STATUS_T *status)
{
	ENTER_FUNC();
}

/*
 * driver module init/exit functions
 */
static int __init ipc_api_test_init(void)
{
	int retval;

	sema_init(&ipc_data_read_wait, 0);
	sema_init(&ipc_data_write_wait, 0);

	ENTER_FUNC();

	retval = misc_register(&ipc_api_data_device);
	if (retval) {
		printk(KERN_ERR "%s: Register IPC API DATA device failed\n",
		       __func__);
		return -ENODEV;
	}

	retval = misc_register(&ipc_api_short_device);
	if (retval) {
		printk(KERN_ERR "%s: Register IPC API Short device failed\n",
		       __func__);
		return -ENODEV;
	}

	ipc_short_ch_desc.type = HW_CTRL_IPC_SHORT_MSG;
	ipc_short_ch_desc.index = 0;
	ipc_short_ch_desc.read_callback = &shortmsg_read_callback;
	ipc_short_ch_desc.write_callback = &shortmsg_write_callback;
	ipc_short_ch_desc.notify_callback = &shortmsg_notify_callback;

	ipc_data_ch_desc.type = HW_CTRL_IPC_PACKET_DATA;
	ipc_data_ch_desc.index = 0;
	ipc_data_ch_desc.read_callback = &data_read_callback;
	ipc_data_ch_desc.write_callback = &data_write_callback;
	ipc_data_ch_desc.notify_callback = &data_notify_callback;

	return retval;
}

static void __exit ipc_api_test_exit(void)
{
	ENTER_FUNC();
	misc_deregister(&ipc_api_data_device);
	misc_deregister(&ipc_api_short_device);
}

/* the module entry declaration of this driver */
module_init(ipc_api_test_init);
module_exit(ipc_api_test_exit);
