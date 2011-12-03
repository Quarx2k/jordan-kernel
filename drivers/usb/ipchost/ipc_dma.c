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
 * 03/22/2008      Motorola        USB-IPC header support
 * 05/09/2008      Motorola        Change Copyright and Changelog
 * 07/09/2008      Motorola        upmerge for 23.5
 * 11/03/2008      Motorola        Support sequence number
 *
 */

/*!
 * @file drivers/usb/ipchost/ipc_dma.c
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
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/usb_ipc.h>
#include <linux/ipc_api.h>
#include <linux/dma-mapping.h>

#include <plat/dma.h>

#if defined(USE_OMAP_SDMA)

#define DEBUG(args...) /*printk(args)*/

struct IPC_DMA_MEMCPY ipc_memcpy_node2buf;
struct IPC_DMA_MEMCPY ipc_memcpy_buf2node;

extern void usb_ipc_exchange_endian16(unsigned short *data);

#define NODE_BUF_PHYS_ADDR(x)   virt_to_phys(x)
#define USB_BUF_PHYS_ADDR(x)    virt_to_phys(x)

/* Start DMA data transfer from memory to memory */
#define OMAP_DMA_MEM2MEM_START(ch, src, dest, len) \
	do { \
		DEBUG("\n%s: dest = 0x%lx, src=0x%lx\n", __func__, dest, src); \
		omap_set_dma_transfer_params(ch, OMAP_DMA_DATA_TYPE_S8, \
			len, 1, OMAP_DMA_SYNC_FRAME, 0, 0); \
		omap_set_dma_dest_params(ch, 0, OMAP_DMA_AMODE_POST_INC, \
			dest, 0, 0); \
		omap_set_dma_src_params(ch, 0, OMAP_DMA_AMODE_POST_INC, src, \
			0, 0); \
		omap_start_dma(ch); \
	} while (0)

/*
 *  Node 2 Buffer DMA callback
 */
void ipc_dma_node2buf_callback(int lch, u16 ch_status, void *data)
{
	int ret, i;
	struct USB_IPC_API_PARAMS *ipc_ch;
	IPC_DATA_HEADER *header;
	struct device *dev = &usb_ipc_data_param.udev->dev;
	DEBUG("%s\n", __func__);

	ipc_ch = ipc_memcpy_node2buf.ipc_ch;
	/* Set DONE bit. If all node buffer are copied to URB buffer */
	ipc_memcpy_node2buf.node_ptr[ipc_memcpy_node2buf.node_index].
		comand |= NODE_DESCRIPTOR_DONE_BIT;

	ipc_memcpy_node2buf.buf_phy +=
		ipc_memcpy_node2buf.node_ptr[ipc_memcpy_node2buf.node_index].
			length;
	ipc_memcpy_node2buf.total_size +=
		ipc_memcpy_node2buf.node_ptr[ipc_memcpy_node2buf.node_index].
			length;
	ipc_ch->write_ptr.total_num +=
		ipc_memcpy_node2buf.node_ptr[ipc_memcpy_node2buf.node_index].
			length;

	if (ipc_memcpy_node2buf.node_ptr[ipc_memcpy_node2buf.node_index].
	    comand & NODE_DESCRIPTOR_LAST_BIT)
		ipc_memcpy_node2buf.frame_index++;

	header = (IPC_DATA_HEADER *) ipc_ch->write_ptr.temp_buff;

	if ((ipc_memcpy_node2buf.node_ptr[ipc_memcpy_node2buf.node_index].
	     comand & NODE_DESCRIPTOR_END_BIT) ||
	    (ipc_memcpy_node2buf.node_index >=
	     ipc_memcpy_node2buf.ipc_ch->max_node_num) ||
	    ipc_memcpy_node2buf.frame_index >= header->nb_frame) {
		omap_stop_dma(ipc_memcpy_node2buf.dma_ch);

		dma_map_single(dev,(void *) ipc_ch->write_ptr.temp_buff,
				ipc_ch->max_temp_buff_size,
				DMA_FROM_DEVICE);

		/* change to big endian */
		header = (IPC_DATA_HEADER *) ipc_ch->write_ptr.temp_buff;
		DEBUG("DMA copy ok:header->version=%x header->nb_frame=%d"
		      "total_size=%d total_num=%d\n",
			header->version, header->nb_frame,
			ipc_memcpy_node2buf.total_size,
			ipc_ch->write_ptr.total_num);
		usb_ipc_exchange_endian16(&(header->version));
		if (header->nb_frame > 0) {
			for (i = 0; i < header->nb_frame; i++)
				usb_ipc_exchange_endian16(&(header->
							     frames[i].
								length));
			usb_ipc_exchange_endian16(&(header->nb_frame));
#if defined(USE_IPC_FRAME_HEADER_CHECKSUM)
			if (header->options == 1) {
				/*TODO: calculate checksum here */
				usb_ipc_exchange_endian16(&(header->
								checksum));
			}
#endif
		}
		/* change to big endian end */

		ipc_ch->write_ptr.end_flag = 1;
		dma_map_single(dev,( void *) ipc_ch->write_ptr.temp_buff,
				sizeof(IPC_DATA_HEADER_INDEX) +
					(sizeof(IPC_FRAME_DESCRIPTOR) *
						(header->nb_frame)),
				DMA_TO_DEVICE);
		if (!ipc_ch->usb_ifs)
			return;
		LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x4);
		ret = ipc_ch->usb_ifs->usb_write(
				(unsigned char *) ipc_ch->write_ptr.temp_buff,
				ipc_memcpy_node2buf.
				total_size);
		if (ret != 0)
			ipc_ch->write_flag = 0;

		return;
	}

	ipc_memcpy_node2buf.node_index++;

	dma_map_single(dev,ipc_memcpy_node2buf.
			node_ptr[ipc_memcpy_node2buf.node_index].data_ptr,
		ipc_memcpy_node2buf.node_ptr[ipc_memcpy_node2buf.
			node_index].length,
		DMA_TO_DEVICE);

	DEBUG("Continue DMA:buf_phy=%lx total_size=%d node_index=%d\n",
		ipc_memcpy_node2buf.buf_phy, ipc_memcpy_node2buf.total_size,
		ipc_memcpy_node2buf.node_index);
	LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x3);
	OMAP_DMA_MEM2MEM_START(ipc_memcpy_node2buf.dma_ch,
		NODE_BUF_PHYS_ADDR(ipc_memcpy_node2buf.
			node_ptr[ipc_memcpy_node2buf.node_index].data_ptr),
		ipc_memcpy_node2buf.buf_phy,
		ipc_memcpy_node2buf.
			node_ptr[ipc_memcpy_node2buf.node_index].
				length);
}

/*
 *  Node 2 Buffer DMA transfer
 */
int ipc_dma_memcpy_node2buf(struct USB_IPC_API_PARAMS *ipc_ch,
			    HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T *ctrl_ptr)
{
	int ret, i;
	int frame_size, frame_index, len;
	IPC_DATA_HEADER *header;
	struct device *dev = &usb_ipc_data_param.udev->dev;

	DEBUG("%s\n", __func__);

	if (ipc_memcpy_node2buf.dma_ch == -1) {	/* error: using memcpy */
		/* copy data into write URB buffer */
		frame_size = 0;
		frame_index = 0;
		header = (IPC_DATA_HEADER *) ipc_ch->write_ptr.temp_buff;
		len = sizeof(IPC_DATA_HEADER_INDEX) +
			((header->nb_frame) * sizeof(IPC_FRAME_DESCRIPTOR));
		for (i = 0; i < ipc_ch->max_node_num; i++) {
			if ((len + ctrl_ptr[i].length) >
			    ipc_ch->max_temp_buff_size) {
				break;
			}
			memcpy((void *) &ipc_ch->write_ptr.temp_buff[len],
			       ctrl_ptr[i].data_ptr, ctrl_ptr[i].length);
			frame_size += ctrl_ptr[i].length;
			len += ctrl_ptr[i].length;
			ctrl_ptr[i].comand |= NODE_DESCRIPTOR_DONE_BIT;
			ipc_ch->write_ptr.total_num += ctrl_ptr[i].length;
			if (ctrl_ptr[i].comand & NODE_DESCRIPTOR_LAST_BIT) {
				header->frames[frame_index].length = frame_size;
				frame_index++;
				frame_size = 0;
			}
			if (ctrl_ptr[i].comand & NODE_DESCRIPTOR_END_BIT)
				break;
		}
		header->nb_frame = frame_index;
		/* change to little endian */
		usb_ipc_exchange_endian16(&(header->version));
		if (header->nb_frame > 0) {
			for (i = 0; i < header->nb_frame; i++)
				usb_ipc_exchange_endian16(&(header->
						frames[i].length));
			usb_ipc_exchange_endian16(&(header->nb_frame));
		}
		/* change to little endian end */
		ipc_ch->write_ptr.end_flag = 1;
		if (!ipc_ch->usb_ifs)
			return 0;
		ret = ipc_ch->usb_ifs->usb_write((unsigned char *)ipc_ch->
					write_ptr.temp_buff, len);
		if (ret != 0)
			ipc_ch->write_flag = 0;

		return 0;
	}

	/* generate header */
	header = (IPC_DATA_HEADER *)ipc_ch->write_ptr.temp_buff;
	frame_size = 0;
	frame_index = 0;
	len = sizeof(IPC_DATA_HEADER_INDEX) +
		((header->nb_frame) * sizeof(IPC_FRAME_DESCRIPTOR));
	for (i = 0; i < ipc_ch->max_node_num; i++) {
		if (((len + ctrl_ptr[i].length) >
		     ipc_ch->max_temp_buff_size) ||
		    (ctrl_ptr[i].length <= 0) ||
		    (ctrl_ptr[i].length > MAX_FRAME_SIZE)) {
			break;
		}
		frame_size += ctrl_ptr[i].length;
		len += ctrl_ptr[i].length;
		if (ctrl_ptr[i].comand & NODE_DESCRIPTOR_LAST_BIT) {
			header->frames[frame_index].length = frame_size;
			DEBUG("header->frames[%d]=%d\n", frame_index,
			      frame_size);
			frame_index++;
			frame_size = 0;
		}
		if (ctrl_ptr[i].comand & NODE_DESCRIPTOR_END_BIT)
			break;
	}
	header->nb_frame = frame_index;
	if (frame_index == 0) {
		omap_free_dma(ipc_memcpy_node2buf.dma_ch);
		usb_ipc_exchange_endian16(&(header->version));
		ipc_ch->write_ptr.end_flag = 1;
		return -1;
	}
	DEBUG("header->version=%x header->nb_frame=%d"
	      "write_ptr.temp_buff_phy_addr=%x\n",
	      header->version, header->nb_frame,
	      ipc_ch->write_ptr.temp_buff_phy_addr);

	ipc_memcpy_node2buf.ipc_ch = ipc_ch;
	ipc_memcpy_node2buf.node_index = 0;
	ipc_memcpy_node2buf.frame_index = 0;
	ipc_memcpy_node2buf.node_ptr = ctrl_ptr;
	ipc_memcpy_node2buf.total_size = sizeof(IPC_DATA_HEADER_INDEX) +
		(sizeof(IPC_FRAME_DESCRIPTOR) * (header->nb_frame));
	ipc_memcpy_node2buf.buf_phy = ipc_ch->write_ptr.temp_buff_phy_addr +
		(sizeof(IPC_DATA_HEADER_INDEX) +
			(sizeof(IPC_FRAME_DESCRIPTOR) * (header->nb_frame)));

	DEBUG("Start DMA:buf_phy=%lx total_size=%d node_index=%d\n",
		ipc_memcpy_node2buf.buf_phy,
		ipc_memcpy_node2buf.total_size,
		ipc_memcpy_node2buf.node_index);

	dma_map_single(dev,(void *) ipc_ch->write_ptr.temp_buff,
			sizeof(IPC_DATA_HEADER_INDEX) +
			  (sizeof(IPC_FRAME_DESCRIPTOR) * (header->nb_frame)),
			DMA_TO_DEVICE);
	dma_map_single(dev,ipc_memcpy_node2buf.
			node_ptr[ipc_memcpy_node2buf.node_index].data_ptr,
		ipc_memcpy_node2buf.node_ptr
			[ipc_memcpy_node2buf.node_index].length,
			DMA_TO_DEVICE);

	LOG_IPC_ACTIVITY(aIpcW, iIpcW, 0x2);
	/* set DMA parameters, then start DMA transfer */
	OMAP_DMA_MEM2MEM_START(ipc_memcpy_node2buf.dma_ch,
			       NODE_BUF_PHYS_ADDR(ipc_memcpy_node2buf.
					node_ptr[ipc_memcpy_node2buf.
						node_index].data_ptr),
			       ipc_memcpy_node2buf.buf_phy,
			       ipc_memcpy_node2buf.
				node_ptr[ipc_memcpy_node2buf.node_index].
					length);
	return 0;
}

/*
 *  Buffer 2 Node DMA callback
 */
void ipc_dma_buf2node_callback(int lch, u16 ch_status, void *data)
{
	int size;
	HW_CTRL_IPC_WRITE_STATUS_T ipc_status;
	struct USB_IPC_API_PARAMS *ipc_ch;
	struct device *dev = &usb_ipc_data_param.udev->dev;
	DEBUG("%s\n", __func__);

	ipc_ch = ipc_memcpy_buf2node.ipc_ch;
	/* Set DONE bit. If all node buffer are copied to URB buffer,
	 * finished
	 */
	ipc_ch->read_ptr.node_ptr[ipc_memcpy_buf2node.node_index].comand |=
		NODE_DESCRIPTOR_DONE_BIT;
	ipc_memcpy_buf2node.buf_phy +=
		ipc_ch->read_ptr.node_ptr[ipc_memcpy_buf2node.node_index].
			length;
	ipc_memcpy_buf2node.header->frames[ipc_memcpy_buf2node.
					   frame_index].length -=
		ipc_ch->read_ptr.node_ptr[ipc_memcpy_buf2node.node_index].
			length;
	if (ipc_memcpy_buf2node.header->
	    frames[ipc_memcpy_buf2node.frame_index].length == 0) {
		ipc_ch->read_ptr.node_ptr[ipc_memcpy_buf2node.node_index].
			comand |= NODE_DESCRIPTOR_LAST_BIT;
		ipc_memcpy_buf2node.frame_index++;
	}

	dma_map_single(dev,ipc_ch->read_ptr.
			node_ptr[ipc_memcpy_buf2node.node_index].data_ptr,
		ipc_ch->read_ptr.node_ptr
			[ipc_memcpy_buf2node.node_index].length,
		DMA_FROM_DEVICE);

	ipc_memcpy_buf2node.total_size +=
		ipc_ch->read_ptr.node_ptr[ipc_memcpy_buf2node.node_index].
			length;

	ipc_memcpy_buf2node.node_index++;
	if ((ipc_memcpy_buf2node.node_index >=
	     ipc_memcpy_buf2node.ipc_ch->read_ptr.node_num) ||
	    (ipc_memcpy_buf2node.frame_index >=
	     ipc_memcpy_buf2node.header->nb_frame)) {
		omap_stop_dma(ipc_memcpy_buf2node.dma_ch);

		ipc_ch->read_ptr.node_ptr
			[ipc_memcpy_buf2node.node_index-1].comand |=
		    NODE_DESCRIPTOR_END_BIT;

		DEBUG("DMA copy ok:header->version=%x header->nb_frame=%d\n",
			ipc_memcpy_buf2node.header->version,
			ipc_memcpy_buf2node.header->nb_frame);
		DEBUG("buf_phy=%lx total_size=%d node_index=%d "
		      "frame_index=%d\n",
			ipc_memcpy_buf2node.buf_phy,
			ipc_memcpy_buf2node.total_size,
			ipc_memcpy_buf2node.node_index,
			ipc_memcpy_buf2node.frame_index);

		/* clear flag to indicate API read function call is done */
		ipc_memcpy_buf2node.ipc_ch->read_flag = 0;

		/* read callback, ... */
		if (ipc_memcpy_buf2node.ipc_ch->cfg.read_callback != NULL) {
			ipc_status.nb_bytes = ipc_memcpy_buf2node.total_size;
			ipc_status.channel = &ipc_memcpy_buf2node.ipc_ch->ch;
			LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x16);
			ipc_memcpy_buf2node.ipc_ch->cfg.
				read_callback(&ipc_status);
		} else {
			LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x1E);
			SEM_UNLOCK(&ipc_memcpy_buf2node.ipc_ch->read_ptr.
				read_mutex);
		}
		return;
	}

	/* set DMA parameters, then start DMA transfer */
	size = ipc_memcpy_buf2node.header->frames
			[ipc_memcpy_buf2node.frame_index].length;
	if (size >
	    ipc_ch->read_ptr.node_ptr[ipc_memcpy_buf2node.node_index].
	    length) {
		size = ipc_ch->read_ptr.node_ptr[ipc_memcpy_buf2node.
				node_index].length;
	}
	ipc_ch->read_ptr.node_ptr[ipc_memcpy_buf2node.node_index].length =
	    size;

	DEBUG("Continue DMA:buf_phy=%lx total_size=%d"
	      "node_index=%d frame_index=%d\n",
		ipc_memcpy_buf2node.buf_phy, ipc_memcpy_buf2node.total_size,
		ipc_memcpy_buf2node.node_index,
		ipc_memcpy_buf2node.frame_index);
	LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x15);
	/* set DMA parameters, then start DMA transfer */
	OMAP_DMA_MEM2MEM_START(ipc_memcpy_buf2node.dma_ch,
		ipc_memcpy_buf2node.buf_phy,
		NODE_BUF_PHYS_ADDR(ipc_ch->read_ptr.
			node_ptr[ipc_memcpy_buf2node.node_index].data_ptr),
		size);
}

/*
 *  Buffer 2 Node DMA transfer
 */
void ipc_dma_memcpy_buf2node(struct USB_IPC_API_PARAMS *ipc_ch)
{
	int size, len, index, frame_index, num;
	IPC_DATA_HEADER *header;
	HW_CTRL_IPC_WRITE_STATUS_T ipc_status;

	DEBUG("%s\n", __func__);

	if (ipc_memcpy_buf2node.dma_ch == -1) {	/* error: using memcpy */
		header = (IPC_DATA_HEADER *) (ipc_ch->read_ptr.temp_buff);
		size = header->frames[0].length;
		len = sizeof(IPC_DATA_HEADER_INDEX) +
			(sizeof(IPC_FRAME_DESCRIPTOR) * (header->nb_frame));
		/* copy data from temporary buffer to scatter buffers */
		for (index = 0, frame_index = 0;
		     index < ipc_ch->read_ptr.node_num; index++) {
			num = (size > (ipc_ch->read_ptr.node_ptr)[index].
				length) ?
				(ipc_ch->read_ptr.node_ptr)[index].length :
				size;
			memcpy((ipc_ch->read_ptr.node_ptr)[index].data_ptr,
				(void *)&((ipc_ch->read_ptr.temp_buff)[len]),
				num);
			ipc_ch->read_ptr.total_num += num;
			/* set flag to indicate received data is filled ... */
			(ipc_ch->read_ptr.node_ptr)[index].comand |=
				NODE_DESCRIPTOR_DONE_BIT;
			size -= num;
			len += num;
			if (size == 0) {
				(ipc_ch->read_ptr.node_ptr)[index].length =
					num;
				(ipc_ch->read_ptr.node_ptr)[index].
					comand |= NODE_DESCRIPTOR_LAST_BIT;
				frame_index++;
				if ((frame_index >= header->nb_frame) ||
					(frame_index >= MAX_FRAME_NUM))
					break;

				size = header->frames[frame_index].length;
			}
		}
		if (index < ipc_ch->read_ptr.node_num) {
			ipc_ch->read_ptr.node_ptr[index].comand |=
			    NODE_DESCRIPTOR_END_BIT;
		} else {
			ipc_ch->read_ptr.node_ptr[index - 1].comand |=
			    NODE_DESCRIPTOR_END_BIT;
		}

		/* clear flag to indicate API read function call is done */
		ipc_ch->read_flag = 0;
		if (ipc_ch->cfg.read_callback != NULL) {
			LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x1F);
			ipc_status.nb_bytes = ipc_ch->read_ptr.total_num;
			ipc_status.channel = &ipc_ch->ch;
			ipc_ch->cfg.read_callback(&ipc_status);
		} else {
			SEM_UNLOCK(&ipc_ch->read_ptr.read_mutex);
		}
		return;
	}

	header = (IPC_DATA_HEADER *) (ipc_ch->read_ptr.temp_buff);
	DEBUG("header->version=%x header->nb_frame=%d"
		"read_ptr.temp_buff_phy_addr=%x\n",
		header->version, header->nb_frame,
		ipc_ch->read_ptr.temp_buff_phy_addr);

	ipc_memcpy_buf2node.node_index = 0;
	ipc_memcpy_buf2node.frame_index = 0;
	ipc_memcpy_buf2node.total_size = 0;
	ipc_memcpy_buf2node.ipc_ch = ipc_ch;
	ipc_memcpy_buf2node.header =
		(IPC_DATA_HEADER *)(ipc_ch->read_ptr.temp_buff);
	ipc_memcpy_buf2node.buf_phy = ipc_ch->read_ptr.temp_buff_phy_addr +
		(sizeof(IPC_DATA_HEADER_INDEX) +
			(sizeof(IPC_FRAME_DESCRIPTOR) *
				(ipc_memcpy_buf2node.header->nb_frame)));

	/* set DMA parameters, then start DMA transfer */
	size = ipc_memcpy_buf2node.header->frames
		[ipc_memcpy_buf2node.frame_index].length;
	if (size > ipc_ch->read_ptr.node_ptr[0].length)
		size = ipc_ch->read_ptr.node_ptr[0].length;

	ipc_ch->read_ptr.node_ptr[0].length = size;

	DEBUG("Start DMA:buf_phy=%lx total_size=%d"
	      "node_index=%d frame_index=%d\n",
	      ipc_memcpy_buf2node.buf_phy, ipc_memcpy_buf2node.total_size,
	      ipc_memcpy_buf2node.node_index,
	      ipc_memcpy_buf2node.frame_index);

	LOG_IPC_ACTIVITY(aIpcR, iIpcR, 0x14);
	/* set DMA parameters, then start DMA transfer */
	OMAP_DMA_MEM2MEM_START(ipc_memcpy_buf2node.dma_ch,
			ipc_memcpy_buf2node.buf_phy,
			NODE_BUF_PHYS_ADDR(ipc_ch->read_ptr.node_ptr
				[ipc_memcpy_buf2node.node_index].data_ptr),
			size);
}
#endif
