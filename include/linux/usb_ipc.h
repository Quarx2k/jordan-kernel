/*
 * Copyright (c) 2007 - 2008 Motorola, Inc, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Changelog:
 * Date               Author           Comment
 * -----------------------------------------------------------------------------
 * 12/07/2007      Motorola        USB-IPC initial
 * 03/22/2007      Motorola        USB-IPC header support
 * 10/09/2008      Motorola        USB-IPC suspend/resume support
 *
 */

/*!
 * @file usb_ipc.h
 * @brief Header file for USB-IPC
 *
 * This is the generic portion of the USB-IPC driver.
 *
 * @ingroup IPCFunction
 */


#ifndef _USB_IPC_H_
#define _USB_IPC_H_

#include <linux/ipc_api.h>

#ifndef __KERNEL__

/* device node for BP log:  major=180, minor=250 */
#define USB_IPC_LOG_DRV_NAME        "/dev/logger"

#else

/* device name in kernel */
#define USB_IPC_LOG_DRV_NAME       "logger"
#define USB_IPC_LOG_MINOR_NUM      250

/* just for test */
#define USB_IPC_DATA_DRV_NAME      "usb_data"
#define USB_IPC_DATA_MINOR_NUM     251

/* USB device descriptor */
#define MOTO_USBIPC_VID            0x22b8
#define MOTO_USBIPC_PID            0x40e6
#define USB_IPC_DATA_IF_NUM        0
#define USB_IPC_LOG_IF_NUM         1


/* USB IPC events */
#define IPC_DATA_WR 0x1
#define IPC_DATA_RD 0x2
#define IPC_DATA_WR_CB 0x4
#define IPC_DATA_RD_CB 0x8
#define IPC_LOG_RD_CB 0x10

#ifdef CONFIG_PM
#define IPC_PM_SUSPEND 0x20
#define IPC_PM_RESUME 0x40

/* 500ms delay required by IPC */
#define USB_IPC_SUSPEND_DELAY 500
#endif	/* CONFIG_PM */

/* define IPC channels which will map with each USB class driver */
typedef enum {
	IPC_DATA_CH_NUM = 0,
	IPC_LOG_CH_NUM,
	MAX_USB_IPC_CHANNELS
} USB_IPC_CHANNEL_INDEX;

/* define the read/write type */
typedef enum {
	/* read type */
	HW_CTRL_IPC_READ_TYPE,
	HW_CTRL_IPC_READ_EX2_TYPE,

	/* write type */
	HW_CTRL_IPC_WRITE_TYPE,
	HW_CTRL_IPC_WRITE_EX_CONT_TYPE,
	HW_CTRL_IPC_WRITE_EX_LIST_TYPE,
	HW_CTRL_IPC_WRITE_EX2_TYPE
} IPC_CHANNEL_ACCESS_TYPE;

/* structure is used to manage data transfer for USB class driver */
typedef struct {
	/* URB */
	struct usb_device *udev;	/* USB device handle */
	struct urb read_urb;
	struct urb write_urb;

	/* endpoint wMaxPacketSize */
	int read_wMaxPacketSize;
	int write_wMaxPacketSize;

	/* the following two functions are called by IPC API to send/read data:
	 * "buff" is the buffer pointer
	 * "size" is buffer size to read/write
	 * If return is 0, the function call is correct, otherwise, error
	 */
	int (*usb_write) (unsigned char *buff, int size);
	int (*usb_read) (unsigned char *buff, int size);

	/* When URB is completed, the following callback is used to notify
	 * the IPC API.
	 * "ch" is the IPC API channel
	 * "flag" is used to indicate whether data transmit is correct.
	 * If 0, ok, otherwise, error.
	 * "size" is the actual size for read/write
	 */
	void (*ipc_read_cb) (USB_IPC_CHANNEL_INDEX ch, int flag, int size);
	void (*ipc_write_cb) (USB_IPC_CHANNEL_INDEX ch, int flag,
			      int size);
#ifdef CONFIG_PM
	/* flag to indicate whether URB is used */
	int read_urb_used;
	int write_urb_used;
	int sleeping;
	int working;
	int allow_suspend;
	spinlock_t pm_lock; /* lock suspend/resume operations */
	struct delayed_work suspend_work;
	struct workqueue_struct *ksuspend_usb_wq;
#endif
	int ipc_events;
	char *truncated_buf;
	int truncated_size;
} USB_IPC_IFS_STRUCT;

extern USB_IPC_IFS_STRUCT usb_ipc_data_param;
extern spinlock_t ipc_event_lock;
extern wait_queue_head_t kipcd_wait;

/* macros for LinkDriver read/write semphore */
#define SEM_LOCK_INIT(x)     init_MUTEX_LOCKED(x)
#define SEM_LOCK(x)          down(x)
#define SEM_UNLOCK(x)        up(x)

/* if defined, the temporary buffer is used to merge scatter buffers
 * into one frame buffer only useful, if one frame is saved into i
 * more than one scatter buffers
 */
#define USE_OMAP_SDMA

/* Structure used to manage data read/write of IPC APIs */
struct USB_IPC_API_PARAMS {

	/* parameters for IPC channels */
	HW_CTRL_IPC_CHANNEL_T ch;
	HW_CTRL_IPC_OPEN_T cfg;
	int open_flag;
	int read_flag;
	int write_flag;
	/* max node number for xxx_write/read_ex2 */
	int max_node_num;

	/* read APIs parameters */
	/* indicate read function types: xxx_read or xxx_read_ex2 */
	IPC_CHANNEL_ACCESS_TYPE read_type;
	/* all parameters used for xxx_read/read_ex2() function call */
	struct {
		struct {
			unsigned char *buf;
			unsigned short nb_bytes;
		} gen_ptr;

		/* the following parameters are used as
			the node descriptor read operation */
		HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T *node_ptr;
		int node_index;
		int node_num;
		/* temporary buffer if the node buffer is smaller
			than EP maxpacksize or frame size */
		unsigned char *temp_buff;
#if defined(USE_OMAP_SDMA)
		dma_addr_t temp_buff_phy_addr;
#endif
		/* totally read byte number for callback arguments */
		int total_num;
		/* only useful, if LinkDriver read callback is NULL */
		struct semaphore read_mutex;
	} read_ptr;

	/* write API parameters */
	/* indicate write function types: xxx_write, xxx_write_ex,
		or xxx_write_ex2 */
	IPC_CHANNEL_ACCESS_TYPE write_type;
	struct {
		struct {
			unsigned char *buf;
			unsigned short nb_bytes;
		} gen_ptr;

		HW_CTRL_IPC_CONTIGUOUS_T *cont_ptr;
		HW_CTRL_IPC_LINKED_LIST_T *list_ptr;

		/* the following parameters are used as
			the node descriptor read operation */
		HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T *node_ptr;
		int node_index;
		int node_num;
		/* indicate whether the last transmit is
			a zero package of all frames */
		int end_flag;
		/* totally write byte number for callback arguments */
		int total_num;
		/* in case, LinkDriver write callback is NULL */
		struct semaphore write_mutex;
		/* temporary buffer if the node buffer is smaller than
			EP maxpacksize or frame size */
		unsigned char *temp_buff;
#if defined(USE_OMAP_SDMA)
		dma_addr_t temp_buff_phy_addr;
#endif
	} write_ptr;

	unsigned long max_temp_buff_size;
	/* USB Class driver interfaces for the relevant IPC channel */
	USB_IPC_IFS_STRUCT *usb_ifs;
};

typedef struct {
	void *next;
	int data_num;
	int read_num;
	unsigned char *ptr;
} IPC_LOG_DATA_BUFFER;

typedef struct {
	struct usb_device *udev;	/* USB device handle */
	/* each receive buffer size, NOT less than MaxPacketSize */
	int read_bufsize;
	/* pointer to IPC_LOG_DATA_BUFFER for read function call */
	IPC_LOG_DATA_BUFFER *read_buf;
	/* pointer to IPC_LOG_DATA_BUFFER for submit URB */
	IPC_LOG_DATA_BUFFER *write_buf;
	/* pointer to last IPC_LOG_DATA_BUFFER */
	IPC_LOG_DATA_BUFFER *end_buf;
	int buf_num;		/* totally IPC_LOG_DATA_BUFFER */

	/* URB */
	struct urb read_urb;	/* URB for data read */
	int isopen;		/* whether open function is called */
	int probe_flag;		/* whether probed */
	int urb_flag;
} USB_LOG_IFS_STRUCT;

#ifdef USE_OMAP_SDMA
#define IPC_DMA_NODE2BUF_ID      1
#define IPC_DMA_BUF2NODE_ID      2

struct IPC_DMA_MEMCPY {
	int dma_ch;
	int node_index;
	int frame_index;
	int total_size;
	struct USB_IPC_API_PARAMS *ipc_ch;
	unsigned long buf_phy;
	IPC_DATA_HEADER *header;
	HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T *node_ptr;
};

extern struct IPC_DMA_MEMCPY ipc_memcpy_node2buf;
extern struct IPC_DMA_MEMCPY ipc_memcpy_buf2node;
extern struct USB_IPC_API_PARAMS usb_ipc_channels[MAX_USB_IPC_CHANNELS];
extern void ipc_dma_memcpy_buf2node(struct USB_IPC_API_PARAMS *ipc_ch);
extern int ipc_dma_memcpy_node2buf(struct USB_IPC_API_PARAMS *ipc_ch,
				HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T *ctrl_ptr);
extern void ipc_dma_node2buf_callback(int lch, u16 ch_status, void *data);
extern void ipc_dma_buf2node_callback(int lch, u16 ch_status, void *data);
#endif

#ifdef CONFIG_IPC_USBHOST_DBG
#define IPC_DBG_ARRAY_SIZE 64
/* The values used in this debug array were arranged as following
 * 0x1  - 0xF  to log ipc_read, ipc_write and urb submitting
 * 0x11 - 0x1F to log urb completion, ipc_read_callback and ipc_write_callback
 * size and frame sequence number of the transfer are also logged after 0x11
 * jiffies is logged when IPC driver initiates port suspend
 * there are three columns in this log,
 *   first  column is for ipc_data_read (bp->ap) activities
 *   second column is for ipc_log (bp->ap) activities
 *   third  column is for ipc_data_write (ap->bp) activities
 */
struct IPC_DBG_INFO {
	int iIpcR;
	int iIpcW;
	int iLogR;
	u32 aIpcR[IPC_DBG_ARRAY_SIZE];
	u32 aIpcW[IPC_DBG_ARRAY_SIZE];
	u32 aLogR[IPC_DBG_ARRAY_SIZE];
	u32 aDma[IPC_DBG_ARRAY_SIZE];
};
extern struct IPC_DBG_INFO sIpcDbg;
#define LOG_IPC_ACTIVITY(array, index, event) \
	do { \
		sIpcDbg.array[sIpcDbg.index++] = event; \
		if (sIpcDbg.index >= IPC_DBG_ARRAY_SIZE) \
			sIpcDbg.index = 0; \
	} while (0)
#else
#define LOG_IPC_ACTIVITY(array, index, event)
#endif /* CONFIG_IPC_USBHOST_DBG */

extern void ipc_api_usb_probe(USB_IPC_CHANNEL_INDEX ch_index,
			      USB_IPC_IFS_STRUCT *usb_ifs);
extern void ipc_api_usb_disconnect(USB_IPC_CHANNEL_INDEX ch_index);
extern void ipc_api_init(void);
extern void ipc_api_exit(void);

/* IPC Data interface */
extern int usb_ipc_data_probe(struct usb_interface *intf,
			      const struct usb_device_id *id);
extern void usb_ipc_data_disconnect(struct usb_interface *intf);
extern int usb_ipc_data_init(void);
extern void usb_ipc_data_exit(void);

/* IPC Log interface */
extern int usb_ipc_log_probe(struct usb_interface *intf,
			     const struct usb_device_id *id);
extern void usb_ipc_log_disconnect(struct usb_interface *intf);
extern int usb_ipc_log_init(void);
extern void usb_ipc_log_exit(void);

#endif				/* __KERNEL__ */

#endif				/*_USB_IPC_H_*/
