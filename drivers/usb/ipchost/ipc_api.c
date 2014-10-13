/*
 * Copyright (c) 2007 - 2008 Motorola, Inc, All Rights Reserved.
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
 * 03/22/2008      Motorola        USB-IPC header support
 * 07/09/2008      Motorola        Change multi-node sending for Phase 1
 * 11/03/2008      Motorola        Support sequence number
 *
 */

/*!
 * @file drivers/usb/ipchost/ipc_api.c
 * @brief USB-IPC Descriptor Set
 *
 * This is the generic portion of the USB-IPC driver.
 *
 * @ingroup IPCFunction
 */


/*
 * Include Files
 */
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/usb.h>
#include <linux/usb_ipc.h>
#include <linux/ipc_api.h>
#include <linux/slab.h>
#define DEBUG(args...) /*printk(args)*/
#define ENTER_FUNC() DEBUG("Enter %s\n", __func__)

/* IPC API channels, define IPC APIs supported channels */
struct USB_IPC_API_PARAMS usb_ipc_channels[MAX_USB_IPC_CHANNELS];
unsigned char sent_sequence_number;
unsigned char recv_sequence_number;

#ifdef USE_OMAP_SDMA
#define USB_BUF_PHYS_ADDR(x)    virt_to_phys(x)
#endif

#define IPC_USB_WRITE_ERROR(channel, status) \
	do { \
		DEBUG("%s: Submit URB error\n", __func__); \
		channel.write_flag = 0; \
		channel.write_ptr.end_flag = 0; \
		channel.cfg.notify_callback(&status); \
		SEM_UNLOCK(&channel.write_ptr.write_mutex); \
	} while (0)

/*
 * this function will be called by USB URB OUT callback ...
 */
void usb_ipc_api_write_callback(USB_IPC_CHANNEL_INDEX ch_index,
				int trans_flag, int trans_size)
{
	HW_CTRL_IPC_WRITE_STATUS_T ipc_status;
	HW_CTRL_IPC_NOTIFY_STATUS_T ipc_notify_status;
	USB_IPC_IFS_STRUCT *usb_ifs;
	ENTER_FUNC();
	DEBUG("\n%s: trans_size = %d\n", __func__, trans_size);

	ipc_notify_status.channel = &usb_ipc_channels[ch_index].ch;
	ipc_notify_status.status = HW_CTRL_IPC_STATUS_ERROR;

	usb_ifs = usb_ipc_channels[ch_index].usb_ifs;

	/* Error, notify callback, and return */
	if ((trans_flag != 0) || (usb_ifs == NULL)) {
		IPC_USB_WRITE_ERROR(usb_ipc_channels[ch_index],
				    ipc_notify_status);
	} else {		/* the previous submit URB is ok */
		/* sent_sequence_number will increase 1 on every read operation,
		 * and modulo 256 by using wrap around on a unsigned char.
		 */
		LOG_IPC_ACTIVITY(aIpcW, iIpcW, sent_sequence_number);
		sent_sequence_number++;

		/* update the totally transmitted data number */
		if (usb_ipc_channels[ch_index].write_ptr.total_num > trans_size)
			usb_ipc_channels[ch_index].write_ptr.total_num =
			    trans_size;
		/* init write callback argument */
		ipc_status.channel = &usb_ipc_channels[ch_index].ch;

		/* all transmitted is done */
		if (usb_ipc_channels[ch_index].write_ptr.end_flag) {
			usb_ipc_channels[ch_index].write_flag = 0;
			usb_ipc_channels[ch_index].write_ptr.end_flag = 0;

			ipc_status.nb_bytes =
			    usb_ipc_channels[ch_index].write_ptr.total_num;
			if (usb_ipc_channels[ch_index].cfg.
			    write_callback != NULL) {
				LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x13);
				usb_ipc_channels[ch_index].cfg.
				    write_callback(&ipc_status);
			} else {
				LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x1F);
				SEM_UNLOCK(&usb_ipc_channels[ch_index].
					   write_ptr.write_mutex);
			}
			return;
		}
	}
}

/*
 * The following macro is called if submit call USB read error
 */
#define IPC_USB_READ_ERROR(channel, status) \
	do { \
		DEBUG("%s: Submit URB error\n", __func__); \
		channel.read_flag = 0; \
		channel.cfg.notify_callback(&status); \
		SEM_UNLOCK(&channel.read_ptr.read_mutex); \
		return; \
	} while (0)

void usb_ipc_exchange_endian16(unsigned short *data)
{
	unsigned char ch;
	ch = ((unsigned char *) data)[0];
	((unsigned char *) data)[0] = ((unsigned char *) data)[1];
	((unsigned char *) data)[1] = ch;
}

/*
 * called by USB URB IN callback ...
 */
void usb_ipc_api_read_callback(USB_IPC_CHANNEL_INDEX ch_index,
			       int read_flag, int real_size)
{
	HW_CTRL_IPC_READ_STATUS_T ipc_status;
	HW_CTRL_IPC_NOTIFY_STATUS_T ipc_notify_status;
	USB_IPC_IFS_STRUCT *usb_ifs;
	int ret;
	IPC_DATA_HEADER *header;
	int i;
#ifndef USE_OMAP_SDMA
	int size, len, num;
#endif

	ENTER_FUNC();

	/* initialize notify callback argument */
	ipc_notify_status.channel = &usb_ipc_channels[ch_index].ch;
	ipc_notify_status.status = HW_CTRL_IPC_STATUS_ERROR;

	/* initialize read callback argument */
	ipc_status.channel = &usb_ipc_channels[ch_index].ch;
	usb_ifs = usb_ipc_channels[ch_index].usb_ifs;
	if (usb_ifs == NULL) {
		IPC_USB_READ_ERROR(usb_ipc_channels[ch_index],
				   ipc_notify_status);
	}
	/* if return is 0, re-call usb_read to receiving data */
	if (real_size == 0) {
		LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x12);
		ret = usb_ifs->usb_read(usb_ipc_channels[ch_index].read_ptr.
						temp_buff,
					usb_ipc_channels[ch_index].
						max_temp_buff_size);
		/* error */
		if (ret != 0) {
			printk("IPC_USB_READ_ERROR\n");
			IPC_USB_READ_ERROR(usb_ipc_channels[ch_index],
					   ipc_notify_status);
		}
		return;
	}

	header = (IPC_DATA_HEADER *) usb_ipc_channels[ch_index].read_ptr.
					temp_buff;
	/* change to little endian */
	if ((real_size >= sizeof(IPC_DATA_HEADER_INDEX))) {
		usb_ipc_exchange_endian16(&(header->version));
		usb_ipc_exchange_endian16(&(header->nb_frame));
#if defined(USE_IPC_FRAME_HEADER_CHECKSUM)
		usb_ipc_exchange_endian16(&(header->checksum));
		/*TODO: calculate checksum here */
#endif
		if (header->nb_frame > 0)
			for (i = 0; i < header->nb_frame; i++)
				usb_ipc_exchange_endian16(
					&(header->frames[i].length));
	}
	/* change to little endian end */
	if ((header->version != IPC_FRAME_VERSION) ||
	    (header->sequence_number != recv_sequence_number)) {
		if (header->version != IPC_FRAME_VERSION)
			printk(KERN_INFO "Wrong IPC Frame Header Version\n");
		else
			printk(KERN_INFO "Wrong Frame Sequence Number\n");
		printk(KERN_INFO "channel:%d, buffer:%08x, "
			"header:%08x\n", ch_index, (unsigned int)
			usb_ipc_channels[ch_index].read_ptr.temp_buff,
			(unsigned int) header);
		printk(KERN_INFO "version:%04x, nb_frame:%04x, "
			"sequence:%02x, options:%02x, checksum:%04x\n",
			header->version, header->nb_frame,
			header->sequence_number, header->options,
			header->checksum);
		panic("panic for wrong verion or sequence number");
	}
	DEBUG("%s:version(0x%x), sequence number(%d)\n", __func__,
		header->version, header->sequence_number);
	/* recv_sequence_number will increase 1 on every read operation,
	 * and modulo 256 by using wrap around on a unsigned char.
	 */
	LOG_IPC_ACTIVITY(aIpcR, iIpcR, recv_sequence_number);
	recv_sequence_number++;

	if ((real_size <= sizeof(IPC_DATA_HEADER_INDEX)) &&
	    (header->nb_frame <= 0)) {
		IPC_USB_READ_ERROR(usb_ipc_channels[ch_index],
				ipc_notify_status);
	}
#ifdef USE_OMAP_SDMA
	LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x13);
	ipc_dma_memcpy_buf2node(&usb_ipc_channels[ch_index]);
#else
	size = header->frames[0].length;
	len = sizeof(IPC_DATA_HEADER_INDEX) +
		(sizeof(IPC_FRAME_DESCRIPTOR) * (header->nb_frame));
	/* copy data from temporary buffer to scatter buffers */
	for (index = 0, frame_index = 0;
	     index < usb_ipc_channels[ch_index].read_ptr.node_num;
	     index++) {
		num = (size >
			usb_ipc_channels[ch_index].read_ptr.node_ptr[index].
			length) ? usb_ipc_channels[ch_index].
		      read_ptr.node_ptr[index].length : size;
		memcpy(usb_ipc_channels[ch_index].read_ptr.node_ptr[index].
			data_ptr,
		       &(usb_ipc_channels[ch_index].read_ptr.temp_buff[len]),
		       num);
		usb_ipc_channels[ch_index].read_ptr.total_num += num;
		/* set flag to indicate received data is filled ... */
		usb_ipc_channels[ch_index].read_ptr.node_ptr[index].
			comand |= NODE_DESCRIPTOR_DONE_BIT;
		size -= num;
		len += num;
		if (size == 0) {
			usb_ipc_channels[ch_index].read_ptr.
				node_ptr[index].length = num;
			usb_ipc_channels[ch_index].read_ptr.
				node_ptr[index].comand |=
					NODE_DESCRIPTOR_LAST_BIT;
			frame_index++;
			if (frame_index >= header->nb_frame)
				break;

			size = header->frames[frame_index].length;
		}
	}

	if (index < usb_ipc_channels[ch_index].read_ptr.node_num) {
		usb_ipc_channels[ch_index].read_ptr.node_ptr[index].
			comand |= NODE_DESCRIPTOR_END_BIT;
	} else {
		usb_ipc_channels[ch_index].read_ptr.
			node_ptr[index - 1].comand |= NODE_DESCRIPTOR_END_BIT;
	}

	/* clear flag to indicate API read function call is done */
	usb_ipc_channels[ch_index].read_flag = 0;
	if (usb_ipc_channels[ch_index].cfg.read_callback != NULL) {
		ipc_status.nb_bytes =
			usb_ipc_channels[ch_index].read_ptr.total_num;
		usb_ipc_channels[ch_index].cfg.read_callback(&ipc_status);
	} else {
		SEM_UNLOCK(&usb_ipc_channels[ch_index].read_ptr.read_mutex);
	}
#endif
}

/*!
 * Opens an IPC link. This functions can be called directly by kernel
 * modules. POSIX implementation of the IPC Driver also calls it.
 *
 * @param config        Pointer to a struct containing configuration para
 *                      meters for the channel to open (type of channel,
 *                      callbacks, etc)
 *
 * @return              returns a virtual channel handler on success, a NULL
 *                      pointer otherwise.
 */
HW_CTRL_IPC_CHANNEL_T *hw_ctrl_ipc_open(const HW_CTRL_IPC_OPEN_T * config)
{
	USB_IPC_CHANNEL_INDEX channel;
	IPC_DATA_HEADER *header = NULL;

	ENTER_FUNC();
	switch (config->type) {
	case HW_CTRL_IPC_PACKET_DATA:
		channel = IPC_DATA_CH_NUM;
		break;
	case HW_CTRL_IPC_CHANNEL_LOG:
		channel = IPC_LOG_CH_NUM;
		break;
	default:
		return NULL;
	}

	if ((usb_ipc_channels[channel].open_flag != 0) ||
	    (usb_ipc_channels[channel].usb_ifs == 0))
		return NULL;

	/* init USB class driver IPC callback */
	usb_ipc_channels[channel].usb_ifs->ipc_read_cb =
		usb_ipc_api_read_callback;
	usb_ipc_channels[channel].usb_ifs->ipc_write_cb =
		usb_ipc_api_write_callback;

	/* init IPC API channel parameters */
	usb_ipc_channels[channel].ch.channel_nb = (int) channel;
	memcpy((void *) &(usb_ipc_channels[channel].cfg), (void *) config,
		sizeof(HW_CTRL_IPC_OPEN_T));
	usb_ipc_channels[channel].max_node_num = MAX_IPC_EX2_NODE_NUM;
	usb_ipc_channels[channel].max_temp_buff_size =
		(MAX_FRAME_SIZE * MAX_FRAME_NUM) + sizeof(IPC_DATA_HEADER);


	usb_ipc_channels[channel].read_ptr.temp_buff =
		kmalloc(usb_ipc_channels[channel].max_temp_buff_size,
			GFP_ATOMIC | GFP_DMA);
	if (usb_ipc_channels[channel].read_ptr.temp_buff == NULL) {
		printk(KERN_ERR "%s: alloc temporary buffer for read transfer"
			" is failed\n", __func__);
		return NULL;
	}
#if defined(USE_OMAP_SDMA)
	usb_ipc_channels[channel].read_ptr.temp_buff_phy_addr =
		USB_BUF_PHYS_ADDR(usb_ipc_channels[channel].read_ptr.
					temp_buff);
#endif

	usb_ipc_channels[channel].write_ptr.temp_buff =
		kmalloc(usb_ipc_channels[channel].max_temp_buff_size,
				GFP_ATOMIC | GFP_DMA);
	if (usb_ipc_channels[channel].write_ptr.temp_buff == NULL) {
		kfree(usb_ipc_channels[channel].read_ptr.temp_buff);
		printk(KERN_ERR "%s: alloc temporary buffer for write transfer"
			" is failed\n", __func__);
		return NULL;
	}
#if defined(USE_OMAP_SDMA)
	usb_ipc_channels[channel].write_ptr.temp_buff_phy_addr =
		USB_BUF_PHYS_ADDR(usb_ipc_channels[channel].write_ptr.
				temp_buff);
#endif

	/* read header init */
	header = (IPC_DATA_HEADER *) usb_ipc_channels[channel].read_ptr.
					temp_buff;
	header->version = IPC_FRAME_VERSION;
	header->nb_frame = 0;
	header->sequence_number = sent_sequence_number = 0;
#if defined(USE_IPC_FRAME_HEADER_CHECKSUM)
	header->options = 1;
#else
	header->options = 0;
#endif

	/* write header init */
	header = (IPC_DATA_HEADER *) usb_ipc_channels[channel].write_ptr.
					temp_buff;
	header->version = IPC_FRAME_VERSION;
	header->nb_frame = 0;
	header->sequence_number = recv_sequence_number = 0;
#if defined(USE_IPC_FRAME_HEADER_CHECKSUM)
	header->options = 1;
#else
	header->options = 0;
#endif
	/* set flag to indicate this channel is opened */
	usb_ipc_channels[channel].open_flag = 1;

	return &usb_ipc_channels[channel].ch;
}

/*!
 * Close an IPC link. This functions can be called directly by kernel
 * modules.
 *
 * @param channel       handler to the virtual channel to close.
 *
 * @return              returns HW_CTRL_IPC_STATUS_OK on success, an error code
 *                      otherwise.
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_close(HW_CTRL_IPC_CHANNEL_T *channel)
{
	DEBUG("%s: channel_nb = %d\n", __func__, channel->channel_nb);
	if ((usb_ipc_channels[channel->channel_nb].open_flag == 0) ||
	    (usb_ipc_channels[channel->channel_nb].write_flag) ||
	    (usb_ipc_channels[channel->channel_nb].read_flag)) {
		return HW_CTRL_IPC_STATUS_ERROR;
	}
	if (usb_ipc_channels[channel->channel_nb].read_ptr.temp_buff != 0) {
		kfree(usb_ipc_channels[channel->channel_nb].read_ptr.
		      temp_buff);
		usb_ipc_channels[channel->channel_nb].read_ptr.temp_buff =
		    0;
	}
	if (usb_ipc_channels[channel->channel_nb].write_ptr.temp_buff != 0) {
		kfree(usb_ipc_channels[channel->channel_nb].write_ptr.
		      temp_buff);
		usb_ipc_channels[channel->channel_nb].write_ptr.temp_buff =
		    0;
	}
	usb_ipc_channels[channel->channel_nb].open_flag = 0;
	return HW_CTRL_IPC_STATUS_OK;
}

/*!
 * Reads data from an IPC link. This functions can be called directly by kernel
 * modules. POSIX implementation of the IPC Driver also calls it.
 *
 * @param channel       handler to the virtual channel where read has been requested
 * @param buf           physical address of DMA'able read buffer to store data read from
 *                      the channel.
 * @param nb_bytes      size of the buffer
 *
 * @return              returns HW_CTRL_IPC_STATUS_OK on success, an error code
 *                      otherwise.
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_read(HW_CTRL_IPC_CHANNEL_T *channel,
				      unsigned char *buf,
				      unsigned short nb_bytes)
{
	return HW_CTRL_IPC_STATUS_ERROR;
}

/*!
 * Writes data to an IPC link. This functions can be called directly by kernel
 * modules. POSIX implementation of the IPC Driver also calls it.
 *
 * @param channel       handler to the virtual channel where read has been requested.
 * @param buf           physical address of DMA'able write buffer containing data to
 *                      be written on the channel.
 * @param nb_bytes      size of the buffer
 *
 * @return              returns HW_CTRL_IPC_STATUS_OK on success, an error code
 *                      otherwise.
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_write(HW_CTRL_IPC_CHANNEL_T *channel,
				       unsigned char *buf,
				       unsigned short nb_bytes)
{
	return HW_CTRL_IPC_STATUS_ERROR;
}

/*!
 * Writes data to an IPC link. This function can be called directly by kernel
 * modules. It accepts a linked list or contiguous data.
 *
 * @param channel       handler to the virtual channel where read has
 *                      been requested.
 * @param mem_ptr       pointer of type HW_CTRL_IPC_WRITE_PARAMS_T. Each element
 *                      points to the physical address of a DMA'able buffer
 *
 * @return              returns HW_CTRL_IPC_STATUS_OK on success, an error code
 *                      otherwise.
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_write_ex(HW_CTRL_IPC_CHANNEL_T *channel,
					  HW_CTRL_IPC_WRITE_PARAMS_T *
					  mem_ptr)
{
	return HW_CTRL_IPC_STATUS_ERROR;
}

/*!
 * Used to set various channel parameters
 *
 * @param channel handler to the virtual channel where read has
 *                been requested.
 * @param action  IPC driver control action to perform.
 * @param param   parameters required to complete the requested action
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_ioctl(HW_CTRL_IPC_CHANNEL_T *channel,
				       HW_CTRL_IPC_IOCTL_ACTION_T action,
				       void *param)
{
	/* detect channel */
	if (usb_ipc_channels[channel->channel_nb].open_flag == 0)
		return HW_CTRL_IPC_STATUS_ERROR;

	switch (action) {
	case HW_CTRL_IPC_SET_READ_CALLBACK:
		usb_ipc_channels[channel->channel_nb].cfg.read_callback =
		    param;
		break;
	case HW_CTRL_IPC_SET_WRITE_CALLBACK:
		usb_ipc_channels[channel->channel_nb].cfg.write_callback =
		    param;
		break;
	case HW_CTRL_IPC_SET_NOTIFY_CALLBACK:
		usb_ipc_channels[channel->channel_nb].cfg.notify_callback =
		    param;
		break;
	case HW_CTRL_IPC_SET_MAX_CTRL_STRUCT_NB:
		usb_ipc_channels[channel->channel_nb].max_node_num =
		    *(int *) param;
		break;
	default:
		return HW_CTRL_IPC_STATUS_ERROR;
	}

	return HW_CTRL_IPC_STATUS_OK;
}

/*!
 * This function is a variant on the write() function, and is used to send a
 * group of frames made of various pieces each to the IPC driver.
 * It is mandatory to allow high throughput on IPC while minimizing the time
 * spent in the drivers / interrupts.
 *
 * @param channel       handler to the virtual channel where read has
 *                      been requested.
 * @param ctrl_ptr      Pointer on the control structure.
 *
 * @return              returns HW_CTRL_IPC_STATUS_OK on success, an error code
 *                      otherwise.
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_write_ex2(HW_CTRL_IPC_CHANNEL_T *channel,
					   HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T
						* ctrl_ptr)
{
	int ret = -1;
	int i;
	USB_IPC_IFS_STRUCT *usb_ifs;
	IPC_DATA_HEADER *header = NULL;

	ENTER_FUNC();

	if (usb_ipc_channels[channel->channel_nb].open_flag == 0) {
		printk(KERN_INFO "%s: Error: channel unavailable\n", __func__);
		return HW_CTRL_IPC_STATUS_CHANNEL_UNAVAILABLE;
	}
	if (usb_ipc_channels[channel->channel_nb].write_flag == 1) {
		printk(KERN_INFO "%s: Error: channel write ongoing\n",
			__func__);
		return HW_CTRL_IPC_STATUS_WRITE_ON_GOING;
	}

	if ((ctrl_ptr == NULL) || (channel == NULL)) {
		printk(KERN_INFO "%s: Error: Input arguments\n", __func__);
		return HW_CTRL_IPC_STATUS_ERROR;
	}

	usb_ipc_channels[channel->channel_nb].write_flag = 1;

	/* save the parameters and start write operation */
	usb_ipc_channels[channel->channel_nb].write_type =
		HW_CTRL_IPC_WRITE_EX2_TYPE;
	usb_ipc_channels[channel->channel_nb].write_ptr.node_ptr =
		ctrl_ptr;
	usb_ipc_channels[channel->channel_nb].write_ptr.node_index = 0;

	for (i = 0; i < usb_ipc_channels[channel->channel_nb].max_node_num;
	     i++) {
		if (ctrl_ptr[i].comand & NODE_DESCRIPTOR_END_BIT)
			break;
	}
	usb_ipc_channels[channel->channel_nb].write_ptr.node_num = i + 1;
	usb_ipc_channels[channel->channel_nb].write_ptr.total_num = 0;
	usb_ipc_channels[channel->channel_nb].write_ptr.end_flag = 0;

	usb_ifs = usb_ipc_channels[channel->channel_nb].usb_ifs;

	if ((usb_ifs == NULL) || (usb_ifs->usb_write == NULL)) {
		printk(KERN_INFO "%s: Error: channel unavailable\n", __func__);
		return HW_CTRL_IPC_STATUS_CHANNEL_UNAVAILABLE;
	}
	header = (IPC_DATA_HEADER *) usb_ipc_channels[channel->channel_nb].
			write_ptr.temp_buff;
	/* This re-assign is for avoiding wrong version in next sending */
	header->version = IPC_FRAME_VERSION;
	/* get frame num */
	header->nb_frame = 0;
	header->sequence_number = sent_sequence_number;
	DEBUG("%s:version(0x%x), sequence number(%d)\n", __func__,
		header->version, header->sequence_number);
	for (i = 0;
	     i < usb_ipc_channels[channel->channel_nb].write_ptr.node_num;
	     i++) {
		if (ctrl_ptr[i].comand & NODE_DESCRIPTOR_LAST_BIT) {
			header->nb_frame++;
			if (header->nb_frame >= MAX_FRAME_NUM)
				break;
		}
	}

#ifdef USE_OMAP_SDMA
	LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x1);
	ret = ipc_dma_memcpy_node2buf(&usb_ipc_channels[channel->channel_nb],
			ctrl_ptr);
#else
	/* copy data into write URB buffer */
	frame_size = 0;
	frame_index = 0;
	len = sizeof(IPC_DATA_HEADER_INDEX) +
		((header->nb_frame) * sizeof(IPC_FRAME_DESCRIPTOR));
	for (i = 0; i < usb_ipc_channels[channel->channel_nb].max_node_num;
	     i++) {
		if ((len + ctrl_ptr[i].length) >
		    usb_ipc_channels[channel->channel_nb].max_temp_buff_size)
			break;
		memcpy(&usb_ipc_channels[channel->channel_nb].write_ptr.
			temp_buff[len], ctrl_ptr[i].data_ptr,
			ctrl_ptr[i].length);
		frame_size += ctrl_ptr[i].length;
		len += ctrl_ptr[i].length;
		ctrl_ptr[i].comand |= NODE_DESCRIPTOR_DONE_BIT;
		usb_ipc_channels[channel->channel_nb].write_ptr.
			total_num += ctrl_ptr[i].length;
		if (ctrl_ptr[i].comand & NODE_DESCRIPTOR_LAST_BIT) {
			header->frames[frame_index].length = frame_size;
			frame_index++;
			frame_size = 0;
		}
		if (ctrl_ptr[i].comand & NODE_DESCRIPTOR_END_BIT)
			break;
	}
	/* change to big endian */
	usb_ipc_exchange_endian16(&(header->version));
	if (header->nb_frame > 0) {
		for (i = 0; i < header->nb_frame; i++)
			usb_ipc_exchange_endian16(&
						  (header->frames[i].
						   length));
		usb_ipc_exchange_endian16(&(header->nb_frame));
	}
	/* change to big endian end */
	usb_ipc_channels[channel->channel_nb].write_ptr.end_flag = 1;
	ret =
	    usb_ifs->usb_write(usb_ipc_channels[channel->channel_nb].
			       write_ptr.temp_buff, len);
#endif				/* USE_OMAP_SDMA */
	if (ret != 0) {
		usb_ipc_channels[channel->channel_nb].write_flag = 0;
		return HW_CTRL_IPC_STATUS_ERROR;
	}

	if (usb_ipc_channels[channel->channel_nb].cfg.write_callback ==
	    NULL) {
		SEM_LOCK(&usb_ipc_channels[channel->channel_nb].write_ptr.
			 write_mutex);
		usb_ipc_channels[channel->channel_nb].write_flag = 0;
	}

	return HW_CTRL_IPC_STATUS_OK;
}

/*
 * This function is used to give a set of buffers to the IPC and enable data
 * transfers.
 *
 * @param channel       handler to the virtual channel where read has
 *                      been requested.
 * @param ctrl_ptr      Pointer on the control structure.
 *
 * @return              returns HW_CTRL_IPC_STATUS_OK on success, an error code
 *                      otherwise.
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_read_ex2(HW_CTRL_IPC_CHANNEL_T *channel,
					  HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T
						* ctrl_ptr)
{
	int ret = -1;
	int i;
	USB_IPC_IFS_STRUCT *usb_ifs;

	ENTER_FUNC();

	if ((channel == NULL) || (ctrl_ptr == NULL)) {
		printk(KERN_INFO "%s: Error: Input arguments\n", __func__);
		return HW_CTRL_IPC_STATUS_ERROR;
	}

	if (usb_ipc_channels[channel->channel_nb].open_flag == 0) {
		printk(KERN_INFO "%s: Error: channel unavailable\n", __func__);
		return HW_CTRL_IPC_STATUS_CHANNEL_UNAVAILABLE;
	}
	if (usb_ipc_channels[channel->channel_nb].read_flag == 1) {
		printk(KERN_INFO "%s: Error: channel read onging\n", __func__);
		return HW_CTRL_IPC_STATUS_READ_ON_GOING;
	}

	usb_ipc_channels[channel->channel_nb].read_flag = 1;

	/* save the parameters and start read operation */
	usb_ipc_channels[channel->channel_nb].read_type =
	    HW_CTRL_IPC_READ_EX2_TYPE;
	usb_ipc_channels[channel->channel_nb].read_ptr.node_ptr = ctrl_ptr;
	usb_ipc_channels[channel->channel_nb].read_ptr.node_index = 0;

	for (i = 0; i < usb_ipc_channels[channel->channel_nb].max_node_num;
	     i++) {
		if (ctrl_ptr[i].comand & NODE_DESCRIPTOR_END_BIT)
			break;
	}
	usb_ipc_channels[channel->channel_nb].read_ptr.node_num = i + 1;
	usb_ipc_channels[channel->channel_nb].read_ptr.total_num = 0;

	usb_ifs = usb_ipc_channels[channel->channel_nb].usb_ifs;
	if ((usb_ifs == NULL) || (usb_ifs->usb_read == NULL)) {
		printk(KERN_ERR "%s: Error: channel unavailable\n", __func__);
		return HW_CTRL_IPC_STATUS_CHANNEL_UNAVAILABLE;
	}
	ret = usb_ifs->usb_read(usb_ipc_channels[channel->channel_nb].
				read_ptr.temp_buff,
				usb_ipc_channels[channel->channel_nb].
				max_temp_buff_size);
	if (ret != 0) {
		usb_ipc_channels[channel->channel_nb].read_flag = 0;
		return HW_CTRL_IPC_STATUS_ERROR;
	}

	/* No callback, waiting receiving is done */
	if (usb_ipc_channels[channel->channel_nb].cfg.read_callback ==
	    NULL) {
		SEM_LOCK(&usb_ipc_channels[channel->channel_nb].read_ptr.
			 read_mutex);
		usb_ipc_channels[channel->channel_nb].read_flag = 0;
	}
	return HW_CTRL_IPC_STATUS_OK;
}

/*
 * This function is used by netmux_linkdriver to register a callback function,
 * the IPC could call this callback when BP enumerate.
 *
 * @param cb            the callback function
 */

#define LINK_DRIVER_CB_DELAY	(MSEC_PER_SEC/2)
struct delayed_work link_driver_work;
struct workqueue_struct *klink_driver_wq;
void (*link_driver_cb) (void) = NULL;
void hw_ctrl_ipc_register(void *param)
{
	link_driver_cb = param;
	if (usb_ipc_channels[IPC_DATA_CH_NUM].usb_ifs != 0) {
		queue_delayed_work(klink_driver_wq, &link_driver_work,
				   msecs_to_jiffies(LINK_DRIVER_CB_DELAY));
	}
}

static void link_driver_work_handler(struct work_struct *work)
{
	if (link_driver_cb) {
		DEBUG("%s:jiffies=%lu execute link_driver callback function\n",
		     __func__, jiffies);
		link_driver_cb();
	}
}

/*
 * this function will be called by probe function of USB class driver.
 */
void ipc_api_usb_probe(USB_IPC_CHANNEL_INDEX ch_index,
		       USB_IPC_IFS_STRUCT *usb_ifs)
{
	/* save the relevant USB class driver paramters */
	usb_ipc_channels[ch_index].usb_ifs = usb_ifs;

	if (link_driver_cb != NULL) {
		queue_delayed_work(klink_driver_wq, &link_driver_work,
				   msecs_to_jiffies(LINK_DRIVER_CB_DELAY));
	}
}

/*
 * this function will be called by disconnect function of USB class driver.
 */
void ipc_api_usb_disconnect(USB_IPC_CHANNEL_INDEX ch_index)
{
	/* clear the relevant USB class driver paramters */
	usb_ipc_channels[ch_index].usb_ifs = 0;

	cancel_delayed_work_sync(&link_driver_work);
	destroy_workqueue(klink_driver_wq);
}

/*
 * IPC API module init function
 */
void ipc_api_init(void)
{
	int i;

	for (i = 0; i < MAX_USB_IPC_CHANNELS; i++) {
		memset((void *) &usb_ipc_channels[i], 0,
		       sizeof(struct USB_IPC_API_PARAMS));
		/* initialize the semphores */
		SEM_LOCK_INIT(&usb_ipc_channels[i].write_ptr.write_mutex);
		SEM_LOCK_INIT(&usb_ipc_channels[i].read_ptr.read_mutex);
	}
	INIT_DELAYED_WORK(&link_driver_work, link_driver_work_handler);
	klink_driver_wq = create_singlethread_workqueue("klink_driver_wq");
}

/*
 * IPC API module exit function
 */
void ipc_api_exit(void)
{
}

/*  */
EXPORT_SYMBOL(hw_ctrl_ipc_open);
EXPORT_SYMBOL(hw_ctrl_ipc_close);
EXPORT_SYMBOL(hw_ctrl_ipc_read);
EXPORT_SYMBOL(hw_ctrl_ipc_write);
EXPORT_SYMBOL(hw_ctrl_ipc_write_ex);
EXPORT_SYMBOL(hw_ctrl_ipc_write_ex2);
EXPORT_SYMBOL(hw_ctrl_ipc_read_ex2);
EXPORT_SYMBOL(hw_ctrl_ipc_ioctl);
EXPORT_SYMBOL(hw_ctrl_ipc_register);
