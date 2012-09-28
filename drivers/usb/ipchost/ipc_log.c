/*
 * Copyright (C) 2007 - 2008 Motorola, Inc, All Rights Reserved.
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
 *
 * Changelog:
 * Date			Author			Comment
 * -----------------------------------------------------------------------------
 * 12/07/2007		Motorola		USB-IPC initial
 * 03/22/2007		Motorola		USB-IPC header support
 * 05/09/2008		Motorola		Change Copyright and Changelog
 *
 */

/*!
 * @file drivers/usb/ipchost/ipc_log.c
 * @brief USB-IPC Descriptor Set
 *
 * This is the generic portion of the USB-IPC driver.
 *
 * @ingroup IPCFunction
 */


/*
 *	usb_ipc_log.c
 *	USB IPC Log driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/usb_ipc.h>
#include <linux/ipc_api.h>

#define DEBUG(args...) /*printk(args)*/

/* USB endpoint detectio */
#define IS_EP_BULK(ep)		(((ep)->bmAttributes) == \
				USB_ENDPOINT_XFER_BULK ? 1 : 0)
#define IS_EP_BULK_IN(ep)	(IS_EP_BULK(ep) && (((ep)->bEndpointAddress) & \
				USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)

static DEFINE_SPINLOCK(ipc_log_spinlock);
static DECLARE_WAIT_QUEUE_HEAD(ipc_log_wait_q);

USB_LOG_IFS_STRUCT ipc_log_param;

/*
 * "mknod /dev/logger c 180 250" to generate USB IPC Log device node
 */

/******************************************************************************
 * Function Name: ipc_log_read_buffer
 *****************************************************************************/
int ipc_log_fill_user_buf(unsigned char *buf, int count)
{
	int i, size, num, ret;
	IPC_LOG_DATA_BUFFER *read_buf;

	DEBUG("%s: Enter count=%d\n", __func__, count);

	size = 0;
	/* loop all Buffer ... */
	for (i = 0; i < ipc_log_param.buf_num; i++) {
		/* all data in buffer is retrieved or input "buf" is full */
		if ((ipc_log_param.read_buf == ipc_log_param.write_buf) ||
			 (count <= 0)) {
			DEBUG("%s: break\n", __func__);
			break;
		}
		/*	*/
		num = count > (ipc_log_param.read_buf->data_num -
			ipc_log_param.read_buf->read_num) ?
			(ipc_log_param.read_buf->data_num -
			ipc_log_param.read_buf->read_num) : count;
		ret = copy_to_user((unsigned char *)((unsigned long)buf + size),
			(unsigned char *)&(ipc_log_param.read_buf->ptr[
			ipc_log_param.read_buf->read_num]), num);
		count -= num;
		ipc_log_param.read_buf->read_num += num;
		size += num;
		/* current buffer is empty, move it to the tail of list */
		if (ipc_log_param.read_buf->data_num <=
			ipc_log_param.read_buf->read_num) {
			DEBUG("%s: Move from head to the tail\n", __func__);
			spin_lock(&ipc_log_spinlock);
			read_buf = ipc_log_param.read_buf;
			read_buf->read_num	= 0;
			read_buf->data_num	= 0;
			if (ipc_log_param.read_buf != ipc_log_param.end_buf) {
				ipc_log_param.read_buf	= read_buf->next;
				ipc_log_param.end_buf->next = read_buf;
				ipc_log_param.end_buf		= read_buf;
				read_buf->next				= 0;
			}
			if (ipc_log_param.write_buf == NULL)
				ipc_log_param.write_buf = read_buf;

			spin_unlock(&ipc_log_spinlock);
		}
	}

	DEBUG("%s: return %d\n", __func__, size);

	return size;
}

/* alloc buffer */
IPC_LOG_DATA_BUFFER *ipc_log_alloc_read_buffer(void)
{
	IPC_LOG_DATA_BUFFER *temp_buf;

	DEBUG("%s: Enter\n", __func__);

	temp_buf = kmalloc(sizeof(IPC_LOG_DATA_BUFFER), GFP_ATOMIC);
	if (temp_buf == NULL)
		return NULL;

	temp_buf->ptr = kmalloc(ipc_log_param.read_bufsize, GFP_ATOMIC);
	if (temp_buf->ptr == NULL) {
		kfree(temp_buf);
		return NULL;
	}
	temp_buf->data_num = 0;
	temp_buf->read_num = 0;
	temp_buf->next	 = 0;

	ipc_log_param.buf_num++;

	DEBUG("%s alloc buffer %d, %d\n", __func__, ipc_log_param.buf_num,
		ipc_log_param.read_bufsize);
	return temp_buf;
}

#define IPC_LOG_URB_START()	\
	do {					\
		spin_lock(&ipc_log_spinlock);	\
		DEBUG("submit urb %d ... \n", ipc_log_param.urb_flag);\
		if ((ipc_log_param.write_buf != NULL) && 	\
			(ipc_log_param.urb_flag == 0) && 		\
			(usb_ipc_data_param.sleeping == 0)) { 	\
			ipc_log_param.read_urb.transfer_buffer	= \
				 ipc_log_param.write_buf->ptr;	\
			ipc_log_param.read_urb.transfer_buffer_length =\
			 ipc_log_param.read_bufsize;	\
			LOG_IPC_ACTIVITY(aLogR, iLogR, 0x1); \
			if (usb_submit_urb(&ipc_log_param.read_urb,\
				 GFP_KERNEL)) {\
				LOG_IPC_ACTIVITY(aLogR, iLogR, 0xE); \
				DEBUG("submit urb is error \n");\
			}				\
			else {			\
				DEBUG("submit urb is OK \n");\
				ipc_log_param.urb_flag = 1;	\
			}	\
		}		\
		spin_unlock(&ipc_log_spinlock);	\
	} while (0);

/******************************************************************************
 * Function Name: usb_ipc_log_open
 *****************************************************************************/
static int usb_ipc_log_open(struct inode *inode, struct file *file)
{
	DEBUG("Enter %s\n", __func__);
	if ((ipc_log_param.isopen) || (ipc_log_param.probe_flag == 0)) {
		DEBUG("%s: USB IPC LOG open error\n", __func__);
		return -EBUSY;
	}

	ipc_log_param.isopen = 1;

	return 0;
}

/******************************************************************************
 * Function Name: usb_ipc_log_release
 *****************************************************************************/
static int usb_ipc_log_release(struct inode *inode, struct file *filp)
{
	if (!ipc_log_param.isopen)
		return 0;

	ipc_log_param.isopen = 0;

	return 0;
}

/******************************************************************************
 * Function Name: usb_ipc_log_poll
 *****************************************************************************/
static unsigned int usb_ipc_log_poll(struct file *file,
	struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	DEBUG("Enter %s\n", __func__);
	if ((ipc_log_param.isopen == 0) || (ipc_log_param.probe_flag == 0)) {
		DEBUG("%s: USB IPC LOG calling poll error\n", __func__);
		LOG_IPC_ACTIVITY(aLogR, iLogR, 0xF);
		return -EBUSY;
	}

	poll_wait(file, &ipc_log_wait_q, wait);

	if (ipc_log_param.read_buf == ipc_log_param.write_buf) {
		IPC_LOG_URB_START();
	} else
		mask = (POLLIN | POLLRDNORM);

	DEBUG("%s: return mask=%d\n", __func__, mask);

	return mask;
}

/******************************************************************************
 * Function Name: usb_ipc_log_read
 *****************************************************************************/
static ssize_t usb_ipc_log_read(struct file *filp,
		char __user *buf, size_t count, loff_t *l)
{
	int size;

	DEBUG("%s: Enter count=%d\n", __func__, count);

	if (!ipc_log_param.isopen) {
		DEBUG("%s: Not Opend\n", __func__);
		return 0;
	}

	/* detect whether resume is need */

	/* detect whether BP is ready? */

	if (count <= 0) {	/* read buffer is 0 */
		return 0;
	}

	/* get out the received data */
	size = ipc_log_fill_user_buf(buf, count);
	/*init_MUTEX_LOCKED(&ipc_log_param.mutex);*/

	DEBUG("%s: return %d\n", __func__, size);

	return size;
}

/* file operation struction */
static const struct file_operations usb_ipc_log_fops = {
	.owner	=	THIS_MODULE,
	open	:	usb_ipc_log_open,
	release	:	usb_ipc_log_release,
	read	:	usb_ipc_log_read,
	poll	:	usb_ipc_log_poll,
};
static struct usb_class_driver usb_ipc_log_device = {
	.name = USB_IPC_LOG_DRV_NAME,
	.fops = &usb_ipc_log_fops,
	.minor_base = USB_IPC_LOG_MINOR_NUM,
};

/*
 *	USB class driver Functions
 */
static void ipc_log_read_callback(struct urb *urb)
{
	unsigned long flags;
	IPC_LOG_DATA_BUFFER	*temp_buf;

	DEBUG("%s: %d\n", __func__, urb->actual_length);
	LOG_IPC_ACTIVITY(aLogR, iLogR, 0x11);
	LOG_IPC_ACTIVITY(aLogR, iLogR, urb->actual_length);

	ipc_log_param.urb_flag = 0;
	if (urb->actual_length != 0)	{
		ipc_log_param.write_buf->read_num = 0;
		ipc_log_param.write_buf->data_num = urb->actual_length;

		ipc_log_param.write_buf = ipc_log_param.write_buf->next;
		/*"write_buf->next = NULL" is same with "write_buf = end_buf"*/
		if ((ipc_log_param.write_buf == NULL) &&
			(ipc_log_param.buf_num < MAX_LOG_BUF_NUM)) {
			DEBUG("%s: alloc read buffer \n", __func__);
			temp_buf = ipc_log_alloc_read_buffer();
			if (temp_buf == NULL)	{
				/* alloc new buffer is failed */
				DEBUG("%s: Error alloc read buffer\n",
					 __func__);
				/*up(&ipc_log_param.mutex);*/
				wake_up_interruptible(&ipc_log_wait_q);
				ipc_log_param.urb_flag = 0;
				LOG_IPC_ACTIVITY( \
					aLogR, iLogR, 0x1F);
				return;
			}
			/* add new buffer to buffer list .... */
			spin_lock(&ipc_log_spinlock);
			ipc_log_param.end_buf->next = temp_buf;
			ipc_log_param.end_buf		= temp_buf;
			ipc_log_param.write_buf	 = temp_buf;
			spin_unlock(&ipc_log_spinlock);
		}
		/*up(&ipc_log_param.mutex);*/
		wake_up_interruptible(&ipc_log_wait_q);
	}

	/* start URB */
	if ((ipc_log_param.isopen == 0) || (ipc_log_param.probe_flag == 0)) {
		DEBUG("%s: USB IPC LOG callback error\n", __func__);
		LOG_IPC_ACTIVITY(aLogR, iLogR, 0x1E);
		return;
	}

	IPC_LOG_URB_START();

	spin_lock_irqsave(&ipc_event_lock, flags);
	usb_ipc_data_param.ipc_events |= IPC_LOG_RD_CB;
	spin_unlock_irqrestore(&ipc_event_lock, flags);
	wake_up(&kipcd_wait);
}

/*
 * usb ipc data driver probe function
 */
int usb_ipc_log_probe(struct usb_interface *intf,
	const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor	*ipc_endpoint;
	struct usb_device *dev = interface_to_usbdev(intf);
	int retval;
	IPC_LOG_DATA_BUFFER *temp_buf;

	ipc_endpoint = &(intf->cur_altsetting->endpoint[0].desc);

	DEBUG("%s: ep num = %d, ep Attr=0x%x, Addr = 0x%x\n", __func__,
		intf->cur_altsetting->desc.bNumEndpoints,
	ipc_endpoint->bmAttributes, ipc_endpoint->bEndpointAddress);

	/* generate read URB */
	/* urb size is the max value of ep MaxPacksize or FrameSize */
	if (!IS_EP_BULK_IN(ipc_endpoint)) 	{
		DEBUG("%s: Undetected IN endpoint\n", __func__);
		/* Shouldn't ever get here unless we have something weird */
		return -ENOMEM;
	}
	ipc_log_param.udev = dev;
	ipc_log_param.read_bufsize = (MAX_LOG_BUF_SIZE >
		ipc_endpoint->wMaxPacketSize) ? MAX_LOG_BUF_SIZE :
		ipc_endpoint->wMaxPacketSize;
	temp_buf = ipc_log_alloc_read_buffer();
	if (temp_buf == NULL)
		return -ENODEV;

	ipc_log_param.read_buf	= temp_buf;
	ipc_log_param.end_buf	= temp_buf;
	ipc_log_param.write_buf = temp_buf;

	usb_set_intfdata(intf, &ipc_log_param);

	retval = usb_register_dev(intf, &usb_ipc_log_device);
	if (retval) {
		DEBUG("%s: Register USB Log device failed\n", __func__);
		return -ENODEV;
	}

	usb_fill_bulk_urb(&ipc_log_param.read_urb, dev,
		usb_rcvbulkpipe(dev, ipc_endpoint->bEndpointAddress),
			0, 0, ipc_log_read_callback, 0);

	ipc_log_param.probe_flag = 1;

	return 0;
}

/*
 * usb ipc data disconnect
 */
void usb_ipc_log_disconnect(struct usb_interface *intf)
{
	int i;
	IPC_LOG_DATA_BUFFER *temp_buf;

	if (ipc_log_param.probe_flag != 0)	{
		DEBUG("%s: deregister USB device\n", __func__);
		LOG_IPC_ACTIVITY(aLogR, iLogR, 0xD);

		/* remove device node */
		usb_deregister_dev(intf, &usb_ipc_log_device);

		DEBUG("%s:	Free URB\n", __func__);
		/* unlink URBs */
		usb_unlink_urb(&ipc_log_param.read_urb);

		/* free memory */
		for (i = 0 ; i < ipc_log_param.buf_num; i++) {
			temp_buf = ipc_log_param.read_buf->next;
			kfree(ipc_log_param.read_buf->ptr);
			kfree(ipc_log_param.read_buf);
			ipc_log_param.read_buf = temp_buf;
		}

		ipc_log_param.probe_flag = 0;

		usb_set_intfdata(intf, NULL);

		/* re-initialize "ipc_log_param" */
		usb_ipc_log_init();
	}
}

/*
 * driver init function
 */
int usb_ipc_log_init(void)
{
	memset((void *)&ipc_log_param, 0, sizeof(ipc_log_param));
	usb_init_urb(&ipc_log_param.read_urb);
	/* init_MUTEX_LOCKED(&ipc_log_param.mutex); */
	return 0;
}

/*
 * driver exit function
 */
void usb_ipc_log_exit(void)
{
}

