/*
 * Copyright (c) 2007 - 2008 Motorola, Inc, All Rights Reserved.
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
 * 03/22/2008      Motorola        USB-IPC header support
 * 10/09/2008      Motorola        USB-IPC suspend/resume support
 *
 */

/*!
 * @file drivers/usb/ipchost/ipc_data.c
 * @brief USB-IPC Descriptor Set
 *
 * This is the generic portion of the USB-IPC driver.
 *
 * @ingroup IPCFunction
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/usb.h>
#include <linux/usb_ipc.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#ifdef USE_OMAP_SDMA
#include <plat/dma.h>
#endif

/* For debug only */
#include <linux/io.h>
#include <plat/io.h>

#ifdef CONFIG_IPC_USBHOST_DBG
#include <linux/debugfs.h>
#endif

/* Module */
MODULE_DESCRIPTION("OMAP SAM IPC Test Module");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

#define DEBUG(args...) /*printk(args)*/

/* #define USB_DATA_LOG */

static struct usb_device_id usb_ipc_id_table[] = {
	{USB_DEVICE(MOTO_USBIPC_VID, MOTO_USBIPC_PID)},
	{}			/* Terminating entry */
};

/* USB endpoint detection */
#define IS_EP_BULK(ep)     (((ep)->bmAttributes) == USB_ENDPOINT_XFER_BULK ? 1 : 0)
#define IS_EP_BULK_IN(ep)  (IS_EP_BULK(ep) && (((ep)->bEndpointAddress) & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
#define IS_EP_BULK_OUT(ep) (IS_EP_BULK(ep) && (((ep)->bEndpointAddress) & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT)

/* Wakes up kipcd */
static struct task_struct *kipcd_task;
DEFINE_SPINLOCK(ipc_event_lock);
DECLARE_WAIT_QUEUE_HEAD(kipcd_wait);

/*  */
USB_IPC_IFS_STRUCT usb_ipc_data_param;

#ifdef CONFIG_PM
extern USB_LOG_IFS_STRUCT ipc_log_param;
#endif	/* CONFIG_PM */

/******************************************************/

/* #define USB_STACK_SEND_ZERO_PACKET */

#ifndef USB_STACK_SEND_ZERO_PACKET
static int ipc_data_urb_actual_len;
#endif

#ifdef CONFIG_IPC_USBHOST_DBG
struct IPC_DBG_INFO sIpcDbg;
struct debugfs_blob_wrapper ipc_dbg_blob;
struct dentry *ipc_dbg_dentry;
#endif /* CONFIG_IPC_USBHOST_DBG */

#ifdef CONFIG_PM
static void ipc_suspend_work(struct work_struct *work)
{
	unsigned long flags;
	USB_IPC_IFS_STRUCT *usb_ifs =
	    container_of(work, USB_IPC_IFS_STRUCT, suspend_work.work);
	if (usb_ifs->sleeping == 0) {
		spin_lock_irqsave(&ipc_event_lock, flags);
		usb_ipc_data_param.ipc_events |= IPC_PM_SUSPEND;
		spin_unlock_irqrestore(&ipc_event_lock, flags);
		wake_up(&kipcd_wait);
	}
}
#endif

/*
 * write buffer
 */
static int ipc_data_write_buffer(unsigned char *buff, int size)
{
	int ret = 0;
	unsigned long flags;

#ifndef USB_STACK_SEND_ZERO_PACKET
	ipc_data_urb_actual_len = 0;
#endif

#ifdef USB_DATA_LOG
	printk(KERN_INFO "Enter %s:   size = %d\n", __func__, size);
	for (ret = 0; ret < size; ret++) {
		printk(KERN_INFO "0x%x, ", buff[ret]);
		if (((ret + 1) % 10) == 0)
			printk("\n");
	}
	ret = 0;
#endif

#ifdef USB_STACK_SEND_ZERO_PACKET
	usb_ipc_data_param.write_urb.transfer_flags |= URB_ZERO_PACKET;
#endif
	usb_ipc_data_param.write_urb.transfer_buffer = buff;
	usb_ipc_data_param.write_urb.transfer_buffer_length = size;
	usb_ipc_data_param.write_urb.dev = usb_ipc_data_param.udev;

	spin_lock_irqsave(&ipc_event_lock, flags);
	usb_ipc_data_param.ipc_events |= IPC_DATA_WR;
	LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x5);
	spin_unlock_irqrestore(&ipc_event_lock, flags);
	wake_up(&kipcd_wait);

	return ret;
}

/*
 * read buffer
 */
static int ipc_data_read_buffer(unsigned char *buff, int size)
{
	int ret = 0;
	unsigned long flags;

	usb_ipc_data_param.read_urb.transfer_buffer = buff;
	usb_ipc_data_param.read_urb.transfer_buffer_length = size;
	usb_ipc_data_param.read_urb.dev = usb_ipc_data_param.udev;

	spin_lock_irqsave(&ipc_event_lock, flags);
	usb_ipc_data_param.ipc_events |= IPC_DATA_RD;
	LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x1);
	spin_unlock_irqrestore(&ipc_event_lock, flags);
	wake_up(&kipcd_wait);

	return ret;
}

/*
 *  BULK IN callback
 */
static void ipc_data_read_callback(struct urb *urb)
{
	unsigned long flags;
	DEBUG("\n%s: received %d bytes @ jiffies = %lu\n", __func__,
	      urb->actual_length, jiffies);

#ifdef USB_DATA_LOG
	int ret;
	char *buff;
	buff = (char *) urb->transfer_buffer;
	for (ret = 0; ret < urb->actual_length; ret++) {
		printk(KERN_INFO "0x%x, ", buff[ret]);
		if (((ret + 1) % 10) == 0)
			printk("\n");
	}
#endif

	spin_lock_irqsave(&ipc_event_lock, flags);
	usb_ipc_data_param.ipc_events |= IPC_DATA_RD_CB;
	LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x11);
	LOG_IPC_ACTIVITY(aIpcR, iIpcR, urb->actual_length);
	spin_unlock_irqrestore(&ipc_event_lock, flags);
	wake_up(&kipcd_wait);
}

/*
 *  BULK OUT callback
 */
static void ipc_data_write_callback(struct urb *urb)
{
	unsigned long flags;
	DEBUG("\n%s: transmitted %d bytes @ jiffies = %lu\n", __func__,
	      urb->actual_length, jiffies);

	/* if the last transmit is not zero, but it is multiple
	 * of MaxPacketSize, send zero package
	 */
#ifndef USB_STACK_SEND_ZERO_PACKET
	ipc_data_urb_actual_len += urb->actual_length;

	if ((usb_ipc_data_param.write_urb.transfer_buffer_length != 0) &&
	    ((usb_ipc_data_param.write_urb.transfer_buffer_length %
	      usb_ipc_data_param.write_wMaxPacketSize) == 0)) {
		usb_ipc_data_param.write_urb.transfer_buffer_length = 0;
		usb_ipc_data_param.write_urb.dev = usb_ipc_data_param.udev;
		usb_submit_urb(&usb_ipc_data_param.write_urb,
			       GFP_ATOMIC | GFP_DMA);
	} else {
		spin_lock_irqsave(&ipc_event_lock, flags);
		usb_ipc_data_param.ipc_events |= IPC_DATA_WR_CB;
		LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x11);
		LOG_IPC_ACTIVITY(aIpcW, iIpcW, ipc_data_urb_actual_len);
		spin_unlock_irqrestore(&ipc_event_lock, flags);
		wake_up(&kipcd_wait);
	}
#else				/* USB_STACK_SEND_ZERO_PACKET */
	spin_lock_irqsave(&ipc_event_lock, flags);
	usb_ipc_data_param.ipc_events |= IPC_DATA_WR_CB;
	spin_unlock_irqrestore(&ipc_event_lock, flags);
	wake_up(&kipcd_wait);
#endif				/* USB_STACK_SEND_ZERO_PACKET */
}

static void ipc_events(void)
{
	int ret = 0;
	int pending_events;
	unsigned long flags;

	spin_lock_irqsave(&ipc_event_lock, flags);
	pending_events = usb_ipc_data_param.ipc_events;
	usb_ipc_data_param.ipc_events = 0;
	spin_unlock_irqrestore(&ipc_event_lock, flags);

	while ((pending_events != 0) && (ret == 0)) {
		/* process ipc_data_write_buffer */
		if (pending_events & IPC_DATA_WR) {
			pending_events &= ~IPC_DATA_WR;
#ifdef CONFIG_PM
			cancel_delayed_work_sync(&usb_ipc_data_param.
						 suspend_work);
			spin_lock_bh(&usb_ipc_data_param.pm_lock);
			if (usb_ipc_data_param.sleeping == 0) {
				usb_ipc_data_param.working = 1;
				LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x6);
				ret = usb_submit_urb(&usb_ipc_data_param.
						   write_urb,
						   GFP_ATOMIC | GFP_DMA);
				spin_unlock_bh(&usb_ipc_data_param.
					       pm_lock);
				pending_events &= ~IPC_PM_SUSPEND;
				spin_lock_irqsave(&ipc_event_lock, flags);
				usb_ipc_data_param.ipc_events &=
				    ~IPC_PM_SUSPEND;
				spin_unlock_irqrestore(&ipc_event_lock,
						       flags);
			} else {
				usb_ipc_data_param.write_urb_used = 1;
				spin_unlock_bh(&usb_ipc_data_param.
					       pm_lock);
				LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x7);
				usb_autopm_get_interface(usb_ifnum_to_if
							 (usb_ipc_data_param.
							  udev,
							  IPC_DATA_CH_NUM));
			}
#else
			ret = usb_submit_urb(&usb_ipc_data_param.write_urb,
					GFP_ATOMIC | GFP_DMA);
#endif
		}
		/* process ipc_data_read_buffer */
		if (pending_events & IPC_DATA_RD) {
			pending_events &= ~IPC_DATA_RD;
#ifdef CONFIG_PM
			spin_lock_bh(&usb_ipc_data_param.pm_lock);
			if (usb_ipc_data_param.sleeping == 0) {
				LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x2);
				ret = usb_submit_urb(&usb_ipc_data_param.
							read_urb,
						     GFP_ATOMIC | GFP_DMA);
				spin_unlock_bh(&usb_ipc_data_param.
					       pm_lock);
			} else {
				usb_ipc_data_param.read_urb_used = 1;
				LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x3);
				spin_unlock_bh(&usb_ipc_data_param.
					       pm_lock);
			}
#else
			ret = usb_submit_urb(&usb_ipc_data_param.read_urb,
					GFP_ATOMIC | GFP_DMA);
#endif
		}
		/* process ipc_data_write_callback */
		if (pending_events & IPC_DATA_WR_CB) {
			pending_events &= ~IPC_DATA_WR_CB;
#ifdef CONFIG_PM
			spin_lock_bh(&usb_ipc_data_param.pm_lock);
			usb_ipc_data_param.working = 0;
			if (usb_ipc_data_param.sleeping == 0) {
				spin_unlock_bh(&usb_ipc_data_param.
					       pm_lock);
				pending_events &= ~IPC_PM_SUSPEND;
				spin_lock_irqsave(&ipc_event_lock, flags);
				usb_ipc_data_param.ipc_events &=
				    ~IPC_PM_SUSPEND;
				LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x12);
				spin_unlock_irqrestore(&ipc_event_lock,
						       flags);
				cancel_delayed_work_sync(
					&usb_ipc_data_param.suspend_work);
				queue_delayed_work(usb_ipc_data_param.
							ksuspend_usb_wq,
						   &usb_ipc_data_param.
							suspend_work,
						   msecs_to_jiffies(
							USB_IPC_SUSPEND_DELAY)
						  );
			} else
				spin_unlock_bh(&usb_ipc_data_param.
					       pm_lock);
#endif				/* CONFIG_PM */
#ifndef USB_STACK_SEND_ZERO_PACKET
			if (usb_ipc_data_param.ipc_write_cb != NULL) {
				usb_ipc_data_param.ipc_write_cb(
					IPC_DATA_CH_NUM, 0,
					ipc_data_urb_actual_len);
			}
#else				/* USB_STACK_SEND_ZERO_PACKET */
			if (usb_ipc_data_param.ipc_write_cb != NULL) {
				usb_ipc_data_param.ipc_write_cb(
					IPC_DATA_CH_NUM, 0,
					usb_ipc_data_param.
						write_urb.actual_length);
			}
#endif				/* USB_STACK_SEND_ZERO_PACKET */
		}
		/* process ipc_data_read_callback */
		if (pending_events & IPC_DATA_RD_CB) {
			pending_events &= ~IPC_DATA_RD_CB;
#ifdef CONFIG_PM
			spin_lock_bh(&usb_ipc_data_param.pm_lock);
			if (usb_ipc_data_param.sleeping == 0) {
				spin_unlock_bh(&usb_ipc_data_param.
					       pm_lock);
				pending_events &= ~IPC_PM_SUSPEND;
				spin_lock_irqsave(&ipc_event_lock, flags);
				usb_ipc_data_param.ipc_events &=
				    ~IPC_PM_SUSPEND;
				spin_unlock_irqrestore(&ipc_event_lock,
						       flags);
				cancel_delayed_work_sync
				    (&usb_ipc_data_param.suspend_work);
				queue_delayed_work(usb_ipc_data_param.
							ksuspend_usb_wq,
						   &usb_ipc_data_param.
							suspend_work,
						   msecs_to_jiffies(
							USB_IPC_SUSPEND_DELAY)
						  );
			} else
				spin_unlock_bh(&usb_ipc_data_param.pm_lock);
#endif
			if ((usb_ipc_data_param.read_urb.status < 0) &&
			    (usb_ipc_data_param.read_urb.actual_length != 0) &&
			    (usb_ipc_data_param.read_urb.actual_length % 512
					== 0)) {
				printk(KERN_INFO
				       "incomplete IN transfer status%d"
				       "length%d\n",
				       usb_ipc_data_param.read_urb.status,
				       usb_ipc_data_param.read_urb.
				       actual_length);

				if (usb_ipc_data_param.truncated_buf == NULL)
					usb_ipc_data_param.truncated_buf =
						(char *)usb_ipc_data_param.
						  read_urb.
						    transfer_buffer;
				usb_ipc_data_param.truncated_size +=
					usb_ipc_data_param.read_urb.
						actual_length;
				usb_ipc_data_param.read_urb.transfer_buffer =
					usb_ipc_data_param.truncated_buf +
					  usb_ipc_data_param.truncated_size;
				usb_ipc_data_param.read_urb.
				  transfer_buffer_length -=
				    usb_ipc_data_param.read_urb.
				      actual_length;
				spin_lock_bh(&usb_ipc_data_param.pm_lock);
				if (usb_ipc_data_param.sleeping == 1) {
					spin_unlock_bh(&usb_ipc_data_param.
						       pm_lock);
					usb_ipc_data_param.read_urb_used = 1;
				} else {
					spin_unlock_bh(&usb_ipc_data_param.
						       pm_lock);
					ret = usb_submit_urb(
						&usb_ipc_data_param.read_urb,
						GFP_ATOMIC | GFP_DMA);
				}
			} else {
				if ((usb_ipc_data_param.truncated_buf !=
				     NULL) &&
				    (usb_ipc_data_param.truncated_size != 0)) {
					printk(KERN_INFO
					       "trying to re-assemble "
					       "incomplete IN transfer\n");
					usb_ipc_data_param.read_urb.
					  transfer_buffer =
					    usb_ipc_data_param.
					      truncated_buf;
					usb_ipc_data_param.read_urb.
					  actual_length +=
					    usb_ipc_data_param.
					      truncated_size;
					usb_ipc_data_param.truncated_buf =
					  NULL;
					usb_ipc_data_param.truncated_size = 0;
				}
				if (usb_ipc_data_param.ipc_read_cb != NULL)
					usb_ipc_data_param.ipc_read_cb(
						IPC_DATA_CH_NUM, 0,
						usb_ipc_data_param.
						  read_urb.actual_length);
			}
		}
		/* process ipc_log_read_callback */
		if (pending_events & IPC_LOG_RD_CB) {
			pending_events &= ~IPC_LOG_RD_CB;
#ifdef CONFIG_PM
			spin_lock_bh(&usb_ipc_data_param.pm_lock);
			if (usb_ipc_data_param.sleeping == 0) {
				spin_unlock_bh(&usb_ipc_data_param.
					       pm_lock);
				pending_events &= ~IPC_PM_SUSPEND;
				spin_lock_irqsave(&ipc_event_lock, flags);
				LOG_IPC_ACTIVITY(aLogR, iLogR, 0x12);
				usb_ipc_data_param.ipc_events &=
					~IPC_PM_SUSPEND;
				spin_unlock_irqrestore(&ipc_event_lock,
						       flags);
				cancel_delayed_work_sync(
					&usb_ipc_data_param.suspend_work);
				queue_delayed_work(usb_ipc_data_param.
							ksuspend_usb_wq,
						   &usb_ipc_data_param.
							suspend_work,
						   msecs_to_jiffies(
							USB_IPC_SUSPEND_DELAY)
						  );
			} else
				spin_unlock_bh(&usb_ipc_data_param.
					       pm_lock);
#endif
		}
#ifdef CONFIG_PM
		if (pending_events & IPC_PM_SUSPEND) {
			pending_events &= ~IPC_PM_SUSPEND;
			DEBUG("%s @ jiffies=%lu\n", __func__, jiffies);
			usb_ipc_data_param.allow_suspend = 1;
			LOG_IPC_ACTIVITY(aIpcW, iIpcW, jiffies);
			usb_autopm_put_interface(
				usb_ifnum_to_if(usb_ipc_data_param.udev,
						IPC_DATA_CH_NUM));
		}
		if (pending_events & IPC_PM_RESUME) {
			pending_events &= ~IPC_PM_RESUME;
			DEBUG("%s @ jiffies=%lu\n", __func__, jiffies);
			usb_autopm_get_interface(
				usb_ifnum_to_if(usb_ipc_data_param.udev,
						IPC_DATA_CH_NUM));
		}
#endif

		spin_lock_irqsave(&ipc_event_lock, flags);
		pending_events = usb_ipc_data_param.ipc_events;
		usb_ipc_data_param.ipc_events = 0;
		spin_unlock_irqrestore(&ipc_event_lock, flags);
	}
}

static int ipc_thread(void *__unused)
{
	set_freezable();
	do {
		ipc_events();
		wait_event_freezable(kipcd_wait,
				     (usb_ipc_data_param.ipc_events != 0)
				     || kthread_should_stop());
	} while (!kthread_should_stop() ||
		 (usb_ipc_data_param.ipc_events != 0));

	return 0;
}

/*
 * usb ipc data driver probe function
 */
int usb_ipc_data_probe(struct usb_interface *intf,
		       const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *ipc_endpoint;
	struct usb_device *dev = interface_to_usbdev(intf);

	usb_ipc_data_param.udev = dev;

#ifdef USE_OMAP_SDMA
	if (0 != omap_request_dma(IPC_DMA_NODE2BUF_ID, NULL,
				  ipc_dma_node2buf_callback, NULL,
				  &ipc_memcpy_node2buf.dma_ch)) {
		printk(KERN_ERR
		       "%s: Failed to allocate DMA channel for IPC write\n",
		       __func__);
		ipc_memcpy_node2buf.dma_ch = -1;
	}
	if (0 != omap_request_dma(IPC_DMA_BUF2NODE_ID, NULL,
				  ipc_dma_buf2node_callback, NULL,
				  &ipc_memcpy_buf2node.dma_ch)) {
		printk(KERN_ERR
		       "%s: Failed to allocate DMA channel for IPC read\n",
		       __func__);
		ipc_memcpy_buf2node.dma_ch = -1;
	}
	printk(KERN_INFO "IPC DMA: ch%d for read, ch%d for write\n",
	       ipc_memcpy_buf2node.dma_ch, ipc_memcpy_node2buf.dma_ch);
#endif

	/* endpoint bulk in */
	ipc_endpoint = &(intf->cur_altsetting->endpoint[0].desc);

	if ((!IS_EP_BULK_IN(ipc_endpoint))) {
		printk(KERN_ERR "%s: Bulk endpoint bulk in type error\n",
		       __func__);
		return -ENOMEM;
	}

	usb_set_intfdata(intf, &usb_ipc_data_param);
	/* generate read URB */
	/* urb size is the max value of ep MaxPacksize or FrameSize */
	usb_ipc_data_param.read_wMaxPacketSize =
		ipc_endpoint->wMaxPacketSize;
	usb_fill_bulk_urb(&usb_ipc_data_param.read_urb, dev,
			  usb_rcvbulkpipe(dev, ipc_endpoint->bEndpointAddress),
			  0, 0, ipc_data_read_callback, 0);

	/* endpoint bulk out */
	ipc_endpoint = &(intf->cur_altsetting->endpoint[1].desc);

	if ((!IS_EP_BULK_OUT(ipc_endpoint))) {
		printk(KERN_ERR "%s: Bulk endpoint bulk out type error\n",
		       __func__);
		return -ENOMEM;
	}

	/* generate write URB */
	usb_ipc_data_param.write_wMaxPacketSize =
		ipc_endpoint->wMaxPacketSize;
	usb_fill_bulk_urb(&usb_ipc_data_param.write_urb, dev,
			  usb_sndbulkpipe(dev, ipc_endpoint->bEndpointAddress),
			  0, 0, ipc_data_write_callback, 0);

	/* initialize parameters in IPC APIs, register this driver to
	 * IPC APIs.
	 */
	ipc_api_usb_probe(IPC_DATA_CH_NUM, &usb_ipc_data_param);

#ifdef CONFIG_PM
	spin_lock_init(&usb_ipc_data_param.pm_lock);
	INIT_DELAYED_WORK(&usb_ipc_data_param.suspend_work, ipc_suspend_work);

	usb_ipc_data_param.ksuspend_usb_wq =
		create_singlethread_workqueue("ksuspend_usb_ipcd");

	usb_ipc_data_param.sleeping = 0;
	usb_ipc_data_param.working = 0;
	usb_ipc_data_param.write_urb_used = 0;
	usb_ipc_data_param.read_urb_used = 0;
	usb_autopm_put_interface_no_suspend(usb_ifnum_to_if(usb_ipc_data_param.udev,
						 IPC_DATA_CH_NUM));   //TODO CHECK!!!
#endif
	usb_ipc_data_param.ipc_events = 0;
	kipcd_task = kthread_run(ipc_thread, NULL, "kipcd");

	return 0;
}

/*
 * usb ipc data disconnect
 */
void usb_ipc_data_disconnect(struct usb_interface *intf)
{
#ifdef CONFIG_IPC_USBHOST_DBG
#ifdef USE_OMAP_SDMA
	int i, iDma;
	u32 dma_ch, uDmaRegAddr;
#endif
#endif /* CONFIG_IPC_USBHOST_DBG */

	DEBUG("Enter %s\n", __func__);
	/* unlink URBs */
	kthread_stop(kipcd_task);
#ifdef CONFIG_PM
	cancel_delayed_work_sync(&usb_ipc_data_param.suspend_work);
	destroy_workqueue(usb_ipc_data_param.ksuspend_usb_wq);
#endif

	usb_unlink_urb(&usb_ipc_data_param.read_urb);
	usb_unlink_urb(&usb_ipc_data_param.write_urb);

#ifdef CONFIG_IPC_USBHOST_DBG
#ifdef USE_OMAP_SDMA
	iDma = 0;
	sIpcDbg.aDma[iDma++] = omap_readl(0x48056000);
	uDmaRegAddr = 0x48056008;
	for (i = 0; i < 10 ; i++) {
		sIpcDbg.aDma[iDma++] = omap_readl(uDmaRegAddr);
		uDmaRegAddr += 4;
	}
	sIpcDbg.aDma[iDma++] = omap_readl(0x48056064);
	uDmaRegAddr = 0x4805606C;
	for (i = 0; i < 4 ; i++) {
		sIpcDbg.aDma[iDma++] = omap_readl(uDmaRegAddr);
		uDmaRegAddr += 4;
	}

	dma_ch = (unsigned int)ipc_memcpy_buf2node.dma_ch;
	sIpcDbg.aDma[iDma++] = dma_ch;
	dma_ch = 0x60 * dma_ch;
	uDmaRegAddr = 0x48056080;
	for (i = 0; i < 18 ; i++) {
		sIpcDbg.aDma[iDma++] = omap_readl(uDmaRegAddr + dma_ch);
		uDmaRegAddr += 4;
	}

	dma_ch = (unsigned int)ipc_memcpy_node2buf.dma_ch;
	sIpcDbg.aDma[iDma++] = dma_ch;
	dma_ch = 0x60 * dma_ch;
	uDmaRegAddr = 0x48056080;
	for (i = 0; i < 18 ; i++) {
		sIpcDbg.aDma[iDma++] = omap_readl(uDmaRegAddr + dma_ch);
		uDmaRegAddr += 4;
	}

	sIpcDbg.aDma[iDma++] = omap_readl(0x48200080);
	sIpcDbg.aDma[iDma++] = omap_readl(0x48200080 + 0x20);
	sIpcDbg.aDma[iDma++] = omap_readl(0x48200080 + 0x40);
	sIpcDbg.aDma[iDma++] = omap_readl(0x48200084);
	sIpcDbg.aDma[iDma++] = omap_readl(0x48200084 + 0x20);
	sIpcDbg.aDma[iDma++] = omap_readl(0x48200084 + 0x40);
#endif
#endif /* CONFIG_IPC_USBHOST_DBG */

	usb_set_intfdata(intf, NULL);

	ipc_api_usb_disconnect(IPC_DATA_CH_NUM);

#ifdef USE_OMAP_SDMA
	omap_stop_dma(ipc_memcpy_node2buf.dma_ch);
	omap_free_dma(ipc_memcpy_node2buf.dma_ch);
	omap_stop_dma(ipc_memcpy_buf2node.dma_ch);
	omap_free_dma(ipc_memcpy_buf2node.dma_ch);
	ipc_memcpy_node2buf.dma_ch = -1;
	ipc_memcpy_buf2node.dma_ch = -1;
#endif

	/* re-init "usb_ipc_data_param" */
	usb_ipc_data_init();
}

int usb_ipc_data_init(void)
{
	memset((void *) &usb_ipc_data_param, 0, sizeof(usb_ipc_data_param));
	usb_ipc_data_param.usb_read = ipc_data_read_buffer;
	usb_ipc_data_param.usb_write = ipc_data_write_buffer;
	usb_init_urb(&usb_ipc_data_param.read_urb);
	usb_init_urb(&usb_ipc_data_param.write_urb);

	return 0;
}

/*
 * driver exit function
 */
void usb_ipc_data_exit(void)
{
}

/************************************************************************
 * IPC USB DRIVER REGISTER
 ************************************************************************/

/*
 * usb ipc disconnect
 */
static void usb_ipc_disconnect(struct usb_interface *intf)
{
	if (intf->cur_altsetting->desc.bInterfaceNumber ==
	    USB_IPC_DATA_IF_NUM) {
		usb_ipc_data_disconnect(intf);
	}
	if (intf->cur_altsetting->desc.bInterfaceNumber ==
	    USB_IPC_LOG_IF_NUM) {
		usb_ipc_log_disconnect(intf);
	}
}

/*
 * usb ipc probe
 */
static int usb_ipc_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	/* according to interface number to detect
	 * whether this is IPC DATA/LOG MSG interface
	 */
	if (intf->cur_altsetting->desc.bInterfaceNumber ==
	    USB_IPC_DATA_IF_NUM) {
		return usb_ipc_data_probe(intf, id);
	}

	if (intf->cur_altsetting->desc.bInterfaceNumber ==
	    USB_IPC_LOG_IF_NUM) {
		return usb_ipc_log_probe(intf, id);
	}

	return -ENOMEM;
}

MODULE_DEVICE_TABLE(usb, usb_ipc_id_table);

#ifdef CONFIG_PM
static int usb_ipc_suspend(struct usb_interface *iface,
			   pm_message_t message)
{
	DEBUG("%s:sleeping=%d working=%d\n", __func__,
	      usb_ipc_data_param.sleeping, usb_ipc_data_param.working);
	spin_lock_bh(&usb_ipc_data_param.pm_lock);
	if (!usb_ipc_data_param.allow_suspend) {
		spin_unlock_bh(&usb_ipc_data_param.pm_lock);
		return -EBUSY;
	}
	if (usb_ipc_data_param.working == 1) {
		spin_unlock_bh(&usb_ipc_data_param.pm_lock);
		DEBUG("%s:working, can not suspend\n", __func__);
		return -1;
	}
	if (iface->cur_altsetting->desc.bInterfaceNumber ==
	    USB_IPC_DATA_IF_NUM) {
		if (usb_ipc_data_param.sleeping == 1) {
			DEBUG("%s:data interface has already suspended\n",
			      __func__);
		} else {
			usb_ipc_data_param.sleeping = 1;
			DEBUG("%s:suspend ipc data interface @ jiffies=%lu\n",
				__func__, jiffies);
		}
	} else if (iface->cur_altsetting->desc.bInterfaceNumber ==
		   USB_IPC_LOG_IF_NUM) {
		DEBUG("%s:suspend ipc log interface @ jiffies=%lu\n",
		      __func__, jiffies);
	}
	spin_unlock_bh(&usb_ipc_data_param.pm_lock);

	return 0;
}

static int usb_ipc_resume(struct usb_interface *iface)
{
	int ret;
	DEBUG("%s:sleeping=%d working=%d\n", __func__,
	      usb_ipc_data_param.sleeping, usb_ipc_data_param.working);
	spin_lock_bh(&usb_ipc_data_param.pm_lock);
	usb_ipc_data_param.allow_suspend = 0;
	if (iface->cur_altsetting->desc.bInterfaceNumber
	    == USB_IPC_DATA_IF_NUM) {
		LOG_IPC_ACTIVITY(aIpcR, iIpcR, jiffies);
		if (usb_ipc_data_param.sleeping == 0) {
			DEBUG("%s:data interface has already resumed\n",
			      __func__);
			spin_unlock_bh(&usb_ipc_data_param.pm_lock);
			return -1;
		} else {
			usb_ipc_data_param.sleeping = 0;
			DEBUG("%s:resume ipc data interface @ jiffies=%lu\n",
				__func__, jiffies);
		}
		if (usb_ipc_data_param.read_urb_used) {
			LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x4);
			ret = usb_submit_urb(&usb_ipc_data_param.read_urb,
					     GFP_ATOMIC | GFP_DMA);
			usb_ipc_data_param.read_urb_used = 0;
			DEBUG("data read urb restarted, ret=%d.\n", ret);
		}
		if (usb_ipc_data_param.write_urb_used) {
			usb_ipc_data_param.working = 1;
			LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x8);
			ret = usb_submit_urb(&usb_ipc_data_param.write_urb,
					     GFP_ATOMIC | GFP_DMA);
			usb_ipc_data_param.write_urb_used = 0;
			DEBUG("data write urb restarted, ret=%d.\n", ret);
		}
	} else if (iface->cur_altsetting->desc.bInterfaceNumber ==
		   USB_IPC_LOG_IF_NUM) {
		DEBUG("%s:resume ipc log interface @ jiffies=%lu\n",
		      __func__, jiffies);
		if ((ipc_log_param.isopen == 1) &&
		    (ipc_log_param.write_buf != NULL) &&
		    (ipc_log_param.urb_flag == 0)) {
			LOG_IPC_ACTIVITY(aLogR, iLogR, 0x3);
			LOG_IPC_ACTIVITY(aLogR, iLogR, jiffies);
			ipc_log_param.read_urb.transfer_buffer =
			    ipc_log_param.write_buf->ptr;
			ipc_log_param.read_urb.transfer_buffer_length =
			    ipc_log_param.read_bufsize;
			ret = usb_submit_urb(&ipc_log_param.read_urb,
					     GFP_KERNEL);
			if (!ret)
				ipc_log_param.urb_flag = 1;
			DEBUG("log read urb restarted, ret=%d.\n", ret);
		}
	}
	spin_unlock_bh(&usb_ipc_data_param.pm_lock);

	return 0;
}
#endif

/* USB host stack entry fucntion for this driver */
static struct usb_driver usb_ipc_driver = {
      .name = "usb_ipc_data",
      .probe = usb_ipc_probe,
      .disconnect = usb_ipc_disconnect,
      .id_table = usb_ipc_id_table,
#ifdef CONFIG_PM
      .supports_autosuspend = 1,
      .suspend = usb_ipc_suspend,
      .resume = usb_ipc_resume,
#endif
};

/*
 * driver module init/exit functions
 */
static int __init usb_ipc_init(void)
{
	int result;

#ifdef CONFIG_IPC_USBHOST_DBG
	memset((void *)&sIpcDbg, 0, sizeof(sIpcDbg));
	/* debugfs change */
	ipc_dbg_blob.data = &sIpcDbg;
	ipc_dbg_blob.size = sizeof(sIpcDbg);
	ipc_dbg_dentry = debugfs_create_blob("ipc_dbg_info",
		S_IRUGO, NULL, &ipc_dbg_blob);
#endif /* CONFIG_IPC_USBHOST_DBG */

	/* IPC API relevant initialization */
	ipc_api_init();

	/* ipc DATA interface relevant initialization */
	result = usb_ipc_data_init();
	if (result != 0)
		return result;

	/* ipc DATA interface log initialization */
	result = usb_ipc_log_init();
	if (result != 0) {
		usb_ipc_data_exit();
		return result;
	}

	result = usb_register(&usb_ipc_driver);
	if (result < 0) {
		usb_ipc_data_exit();
		usb_ipc_log_exit();
		printk(KERN_ERR "%s: Register USB IPC driver failed", __func__);
		return -1;
	}

	return 0;
}

/*
 * driver exit function
 */
static void __exit usb_ipc_exit(void)
{
	/* IPC API relevant exit */
	ipc_api_exit();

	/* USB IPC DATA driver exit */
	usb_ipc_data_exit();

	/* USB IPC LOG driver exit */
	usb_ipc_log_exit();

	/* unregister USB IPC driver */
	usb_deregister(&usb_ipc_driver);

#ifdef CONFIG_IPC_USBHOST_DBG
	/* Remove the debugfs entry */
	debugfs_remove(ipc_dbg_dentry);
#endif
}

/* the module entry declaration of this driver */
module_init(usb_ipc_init);
module_exit(usb_ipc_exit);
