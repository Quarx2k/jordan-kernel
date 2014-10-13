/*
 * Copyright (C) 2007-2008 Motorola, Inc
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
 * 03/22/2008      Motorola        USB-IPC header support
 * 05/09/2008      Motorola        Change Copyright and Changelog
 * 07/09/2008      Motorola        Change MAX_FRAME_SIZE for Phase 1
 * 11/03/2008      Motorola        Support sequence number
 */

/*!
 * @file ipc_api.h
 * @brief Header file for USB-IPC
 *
 * This is the generic portion of the USB-IPC driver.
 *
 * @ingroup IPCFunction
 */


#ifndef __IPC_API_H__
#define __IPC_API_H__

/* the following macros shall be same with BP and NetMux */
#define MAX_FRAME_SIZE           1552
#define MAX_FRAME_NUM            10

/* supported max node number for write_ex2/read_ex2 */
#define MAX_IPC_EX2_NODE_NUM     50

/* bits for node comand */
#define NODE_DESCRIPTOR_END_BIT  0x4000
#define NODE_DESCRIPTOR_LAST_BIT 0x8000
#define NODE_DESCRIPTOR_DONE_BIT 0x2000

/* Log buffer in AP */
#define MAX_LOG_BUF_NUM   20
/* Max buffer size for each Log BULK IN transfer: 3K for 2 frames */
#define MAX_LOG_BUF_SIZE  3072

/* the extra-header will be used for DATA channel */
#define IPC_FRAME_VERSION   0x0200
typedef struct {
	unsigned short length;
} IPC_FRAME_DESCRIPTOR;

typedef struct {
	unsigned short version;
	unsigned short nb_frame;
	unsigned char sequence_number;
	unsigned char options;
	unsigned short checksum;
	IPC_FRAME_DESCRIPTOR frames[MAX_FRAME_NUM];
} IPC_DATA_HEADER;

typedef struct {
	unsigned short version;
	unsigned short nb_frame;
	unsigned char sequence_number;
	unsigned char options;
	unsigned short checksum;
} IPC_DATA_HEADER_INDEX;


/*!
 * This enum defines the different status of an IPC link
 */
typedef enum {
	HW_CTRL_IPC_STATUS_OK = 0,
	HW_CTRL_IPC_STATUS_CHANNEL_UNAVAILABLE,
	HW_CTRL_IPC_STATUS_IPC_SUSPENDED,
	HW_CTRL_IPC_STATUS_READ_ON_GOING,
	HW_CTRL_IPC_STATUS_WRITE_ON_GOING,
	HW_CTRL_IPC_STATUS_INIT_ALREADY_CALLED,
	HW_CTRL_IPC_STATUS_ERROR
} HW_CTRL_IPC_STATUS_T;

/*!
 * This typedef defines the ioctls available
 * for kernel modules using IPC.
 */
typedef enum {
	HW_CTRL_IPC_SET_READ_CALLBACK = 0,
	HW_CTRL_IPC_SET_WRITE_CALLBACK,
	HW_CTRL_IPC_SET_NOTIFY_CALLBACK,
	HW_CTRL_IPC_SET_MAX_CTRL_STRUCT_NB
} HW_CTRL_IPC_IOCTL_ACTION_T;

/*!
 * Definition of IPC channel types
 *
 * There are currently three types of channels:
 *
 * - Short Message channels. Used to transfer 32-bits
 *   messages from MCU to DSP and vice versa
 *
 * - Packet Data channels. Useful to transfer data between
 *   the two cores.
 *
 * - Logging channel. This type of channel is read-only from
 *   the MCU. It is used to report log events to the MCU.
 *
 */
typedef enum {
	HW_CTRL_IPC_PACKET_DATA = 0,
	HW_CTRL_IPC_CHANNEL_LOG,
	HW_CTRL_IPC_SHORT_MSG
} HW_CTRL_IPC_CHANNEL_TYPE_T;

/*!
 * This enum defines the write modes IPC can support.
 *
 * - Contigous mode: All data is stored in a contigous
 *   memory zone.
 *
 * - LinkedList mode: Data can be presented in chunks of
 *   non-contiguous memory. Up to 11 chunks can be processed
 *   by this mode.
 *
 *  Note that only Packet Data channels can support LinkedList mode.
 */
typedef enum {
	HW_CTRL_IPC_MODE_CONTIGUOUS = 0,
	HW_CTRL_IPC_MODE_LINKED_LIST
} HW_CTRL_IPC_MODE_T;

typedef struct {
} HW_CTRL_IPC_INIT_T;

typedef struct {
	/*!
	 * the channel from which data was read
	 */
	int channel_nb;
} HW_CTRL_IPC_CHANNEL_T;

/*!
 * A structure of this type is passed as parameter when a read
 * callback is invoked. This normally happens when a read transfer has been
 * completed.
 */
typedef struct {
	/*!
	 * the channel handler from which data was read
	 */
	HW_CTRL_IPC_CHANNEL_T *channel;

	/*!
	 * number of bytes read.
	 */
	int nb_bytes;
} HW_CTRL_IPC_READ_STATUS_T;

/*!
 * A structure of this type is passed as parameter when a write
 * callback is invoked. This normally happens when a write transfer has been
 * completed.
 */
#define HW_CTRL_IPC_WRITE_STATUS_T     HW_CTRL_IPC_READ_STATUS_T

/*!
 * A structure of this type is passed as parameter when a notify
 * callback is invoked. This happens only when a transfer has been
 * abnormally terminated.
 */
typedef struct {
	/*!
	 * the channel handler from which data was read
	 */
	HW_CTRL_IPC_CHANNEL_T *channel;

	/*!
	 * status code
	 */
	HW_CTRL_IPC_STATUS_T status;
} HW_CTRL_IPC_NOTIFY_STATUS_T;

/*
 * This struct defines parameters needed to
 * execute a write using the linked list mode
 */
typedef struct HW_CTRL_IPC_LINKED_LIST_T {
	/*!
	 * pointer to data to be transfered
	 */
	unsigned char *data_ptr;

	/*!
	 * Lenght of data
	 */
	unsigned int length;

	/*!
	 * Pointer to the next chunk of memory containing data
	 * to be transferred
	 */
	struct HW_CTRL_IPC_LINKED_LIST_T *next;
} HW_CTRL_IPC_LINKED_LIST_T;

/*
 * This struct defines parameters needed to
 * execute a write using the normal mode
 */
typedef struct {
	/*!
	 * pointer to data to be transfered
	 */
	unsigned char *data_ptr;

	/*!
	 * Lenght of data
	 */
	unsigned int length;
} HW_CTRL_IPC_CONTIGUOUS_T;

/*
 * This structure is used by the write_ex function
 * which is in charge of execute the LinkedList mode
 * transfer.
 */
typedef struct {
	/*!
	 * Type of transfer to execute
	 */
	HW_CTRL_IPC_MODE_T ipc_memory_read_mode;

	/*!
	 * Pointer to a buffer holding all data to
	 * transfer orpointer to a buffer holding
	 * first chunk of data to transfer
	 */
	union {
		HW_CTRL_IPC_CONTIGUOUS_T *cont_ptr;
		HW_CTRL_IPC_LINKED_LIST_T *list_ptr;
	} read;
} HW_CTRL_IPC_WRITE_PARAMS_T;

/*
 * Structure used to pass configuration parameters needed
 * to open an IPC channel.
 */
typedef struct {
	/*!
	 * type of IPC channel to open
	 */
	HW_CTRL_IPC_CHANNEL_TYPE_T type;

	/*!
	 * index defining the physical channel
	 * that will be used for this IPC channel
	 */
	int index;

	/*!
	 * read callback provided by the user, called when a read
	 * transfer has been finished
	 */
	void (*read_callback) (HW_CTRL_IPC_READ_STATUS_T *status);

	/*!
	 * write callback provided by the user, called when a write
	 * transfer has been finished
	 */
	void (*write_callback) (HW_CTRL_IPC_WRITE_STATUS_T *status);

	/*!
	 * notify callback provided by the user, called when an error
	 * occurs during a transfer.
	 */
	void (*notify_callback) (HW_CTRL_IPC_NOTIFY_STATUS_T *status);
} HW_CTRL_IPC_OPEN_T;

/*!@param *data_control_struct_ipcv2
 *   Data Node Descriptor (Buffer Descriptor):
 *------------------------------------------------------------------------------
 *| 31	30	29	28	27	26	25	24	23	22
21	20	19	18	17	16	15	0|
 *------------------------------------------------------------------------------
 *| L	E	D	R	R	R	R	R|<---- Reserved---->
		|<- Length-> |
 *------------------------------------------------------------------------------
 *| <---------------------------- Data Ptr ----------------------------------->|
 *------------------------------------------------------------------------------
 *
 * L bit (LAST): If set, means that this buffer of data is the last buffer of
 * the frame.
 * E bit (END): If set, we reached the end of the buffers passed to the function
 * D bit (DONE): Only valid on the read callback. When set,
 * means that the buffer has been filled by the SDMA.
 * Length: Length of data pointed by this node in bytes
 * Data Ptr: Pointer to the data pointed to by this node.
 */
typedef struct ipc_dataNodeDescriptor {
	unsigned short length;
	unsigned short comand;
	void *data_ptr;
} HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T;

/*!
 * This function is called when kernel is loading the module.
 * It initializes all low-level resources needed by the POSIX
 * layer in order to make SDMA/MU transfers.
 *
 * @param init_params	Pointer to a struct containing global
 * 			initialization parameters.
 *
 * @return		returns HW_CTRL_IPC_STATUS_OK on success, an error code
 *			otherwise.
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_init(const HW_CTRL_IPC_INIT_T *init_params);

/*!
 * Opens an IPC link. This functions can be called directly by kernel
 * modules. POSIX implementation of the IPC Driver also calls it.
 *
 * @param config	Pointer to a struct containing configuration para
 * 			meters for the channel to open (type of channel,
 *			callbacks, etc)
 *
 * @return		returns a virtual channel handler on success, a NULL pointer otherwise.
 */
HW_CTRL_IPC_CHANNEL_T *hw_ctrl_ipc_open(const HW_CTRL_IPC_OPEN_T * config);

/*!
 * Close an IPC link. This functions can be called directly by kernel
 * modules. POSIX implementation of the IPC Driver also calls it.
 *
 * @param channel	handler to the virtual channel to close.
 *
 * @return		returns HW_CTRL_IPC_STATUS_OK on success, an error code
 *			otherwise.
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_close(HW_CTRL_IPC_CHANNEL_T *channel);

/*!
 * Reads data from an IPC link. This functions can be called directly by kernel
 * modules. POSIX implementation of the IPC Driver also calls it.
 *
 * @param channel	virtual channel handler where read has been requested.
 * @param buf		buffer to store data read from the channel.
 * @param buffer_size	size of the buffer
 *
 * @return		returns HW_CTRL_IPC_STATUS_OK on success, an error code
 *			otherwise.
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_read(HW_CTRL_IPC_CHANNEL_T *channel,
				      unsigned char *buf,
				      unsigned short buffer_size);

/*!
 * Writes data to an IPC link. This functions can be called directly by kernel
 * modules. POSIX implementation of the IPC Driver also calls it.
 *
 * @param channel	virtual handler to the channel where read has been requested.
 * @param buf		buffer containing data t be written on the channel.
 * @param buffer_size	size of the buffer
 *
 * @return		returns HW_CTRL_IPC_STATUS_OK on success, an error code
 *			otherwise.
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_write(HW_CTRL_IPC_CHANNEL_T *channel,
				       unsigned char *buf,
				       unsigned short nb_bytes);

/*!
 * Writes data to an IPC link. This function can be called directly by kernel
 * modules. It accepts a linked list or contiguous data.
 *
 * @param channel       handler to the virtual channel where read has
 *                      been requested.
 * @param mem_ptr       of type HW_CTRL_IPC_WRITE_PARAMS_T.
 *
 * @return              returns HW_CTRL_IPC_STATUS_OK on success, an error code
 *                      otherwise.
 */
HW_CTRL_IPC_STATUS_T hw_ctrl_ipc_write_ex(HW_CTRL_IPC_CHANNEL_T *channel,
					  HW_CTRL_IPC_WRITE_PARAMS_T *mem_ptr);

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
				       void *param);
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
			HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T *ctrl_ptr);

/*!
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
			HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T *ctrl_ptr);

void hw_ctrl_ipc_register(void *param);

#endif	/*_IPC_API_H__*/
