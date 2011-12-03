/******************************************************************************
 * IPC Link Driver netmux_usb_linkdriver.c                                   *
 *                                                                            *
 * Copyright (C) 2006-2010 Motorola, Inc.                                     *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions are     *
 * met:                                                                       *
 *                                                                            *
 * o Redistributions of source code must retain the above copyright notice,   *
 *   this list of conditions and the following disclaimer.                    *
 * o Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * o Neither the name of Motorola nor the names of its contributors may be    *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS    *
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,  *
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR     *
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR           *
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,      *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,        *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR         *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF     *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING       *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS         *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.               *
 *                                                                            *
 ******************************************************************************/
/*   DATE        OWNER       COMMENT                                          *
 *   ----------  ----------  -----------------------------------------------  *
 *   2006/09/28  Motorola    Initial version                                  *
 *   2006/10/19  Motorola    Fixed scheduling while atomic issues             *
 *   2006/12/26  Motorola    Check bd status when mxc_dma_get_config() returns*
 *   2007/01/05  Motorola    Fixed race conditions that existed between USB  *
 *                           and MU tasklets                                  *
 *   2007/01/24  Motorola    Change Linkdriver to utilize IPC V2              *
 *   2007/04/04  Motorola    Handle error when MU write fails.                *
 *                           Add proc interface for debugging.                *
 *   2007/04/11  Motorola    Increased LOC_MAX_RCV_SIZ and REM_MAX_RCV_SIZ to *
 *                           1552                                             *
 *   2007/07/17  Motorola    Treat SUSPEND_ACK like SUSPEND_REQ               *
 *   2007/12/05  Motorola    Change code as kernel upgrade, sdma ipc not used *
 *                           in OMAP, usb ipc used instead                    *
 *   2008/04/10  Motorola    split large package before transmit to ipc       *
 *   2008/06/04  Motorola    add macro to choose phaseI or phaseII            *
 *   2008/07/09  Motorola    upmerge to kernel 2.6.24 to support TI 23.5      *
 *                           make phaseI and phaseII same behavior            *
 *   2008/09/30  Motorola    Turn panics into printks                         *
 *   2008/10/30  Motorola    Add dynamic log for linkdriver                   *
 *   2008/12/25  Motorola    change init_module, call IPC register func       *
 *   2009/05/20  Motorola    Add mini trace functionality                     *
 *   2009/07/23  Motorola    Add wake lock functionality                      *
 *   2009/08/06  Motorola    Change permissions for /proc/linkdriver to 660   *
 *   2009/08/07  Motorola    Fix minitrace functionality                      *
 *   2009/09/16  Motorola    Comment out cleanup_module() since linkdriver    *
 *                           should not be removed and causes rmmod to panic  *
 *   2010/04/28  Motorola    Format cleanup                                   *
 *   2010/09/02  Motorola    Temp kernel memory allocation fix                *
 *   2010/09/03  Motorola    Enable proc logging for Engineering builds only  *
 ******************************************************************************/

/* netmux_usb_linkdriver.c is responsible for communicating with the NetMUX  *
 * above and with the physical link driver below.  A usb data channel is     *
 * used to send and receive NetMUX data over the IPC link.                   */

/* Note that this driver employs the defined asynchronous read and write      *
 * mechanism of the IPC driver, which buffers this driver from the usb driver,*
 * below.  Since it is not guaranteed that reads and/or writes will not occur *
 * immediately upon being set up, this driver can not count on the fact that  *
 * the operations are completed until the callbacks are invoked.              */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/fcntl.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/netdevice.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/ipc_api.h>
#include <linux/wakelock.h>

#include <linux/io.h>

#include <ldprotocol.h>

#include <linux/usb.h>
#include <linux/usb_ipc.h>
#include <linux/ipc_api.h>
#include <linux/dma-mapping.h>

/*
 * Define our module
 */
MODULE_DESCRIPTION("NetMUX Link driver for Linux");
MODULE_LICENSE("Dual BSD/GPL");


/*
 * If debugging or logging is turned on setup the appropriate
 * macros.
 */
#if defined(CONFIG_NETMUX_LINKDRIVER_SOFTERROR)
#define NETMUX_FAIL(args...)    printk(args)
#else
#define NETMUX_FAIL(args...)    printk(args)
#endif

#define DEBUG(args...) LDOutput("IPC Link Driver: " args)
#define LOG(args...)   LOGOutput(args)

#define DEFAULT_PROC_DATA_SIZE 4096

/* Linkdriver info accessible via the proc interface */
#define LDLOG_COMMAND_ALL_WORK   49	/* ASCII 1 */
#define LDLOG_COMMAND_BUFFER     50	/* ASCII 2 */
#define LDLOG_COMMAND_FUNCTION   51	/* ASCII 3 */
#define LDLOG_COMMAND_MINI_TRACE 52	/* ASCII 4 */

#define LDLOG_COMMAND_LEN 2

static char *ld_info;
static char LDLogState[LDLOG_COMMAND_LEN];

/*
 * Define the receive buffer sizes.  Note that the remote receive buffer size
 * will be reset by the remote IPC Link Driver so don't depend on this
 * definition.
 */

#define LOC_MAX_RCV_SIZ  MAX_FRAME_SIZE	/*1552 */
#define REM_MAX_RCV_SIZ  MAX_FRAME_SIZE	/*1552 */
#define USB_CHANNEL  0

#define IOI_OFF 0
#define IOI_ON  1

#define END_BIT  NODE_DESCRIPTOR_END_BIT	/*0x4000 */
#define LAST_BIT NODE_DESCRIPTOR_LAST_BIT	/*0x8000 */
#define DONE_BIT NODE_DESCRIPTOR_DONE_BIT	/*0x2000 */

/* Memory allocation size for retry attempt */
#define LOC_RETRY_RCV_SIZ 2100

/* To keep things simple, let's have just one lock that we can use to protect
 * the critical areas of the IPC Link Driver.  The two classes of code that need
 * protection are operations involving the nm_ld_pm_state and operations
 * involving synchronization between the ipc_transmitting flag and the
 * mux_deferred flag.
 */

static DEFINE_SPINLOCK(ild_lock);

USB_IPC_IFS_STRUCT usb_ipc_data_param;
/*
 * Define all the globals required.
 */
/* Define a place to hold the remote receive buffer size */
static unsigned int remMaxRcvSiz = REM_MAX_RCV_SIZ;

static int processed_queue_length = 0;

static struct INTERFACELINK iflink;
static struct INTERFACEMUX ifmux;

static struct tasklet_struct write_callback;
static struct tasklet_struct read_callback;

static struct sk_buff_head send_queue;
#define SEND_QUEUE_MAX_SIZE     10
#define SEND_QUEUE_LOW_MARK	((SEND_QUEUE_MAX_SIZE / 2) + 1)
#define SEND_QUEUE_LEN		(skb_queue_len(&send_queue))
#define SEND_QUEUE_ISEMPTY	(skb_queue_empty(&send_queue))
/* the reference count for the commbuff is incremented before
 * it is added to the send_queue
 */
#define SEND_QUEUE_ADD(buf)				\
	do {						\
		skb_get(buf);				\
		skb_queue_tail(&send_queue, buf);	\
	} while (0)

/* after removing the commbuff from the send_queue, its reference
 * count will be decremented
 */
#define SEND_QUEUE_REMOVE	(kfree_skb(skb_dequeue(&send_queue)))
#define SEND_QUEUE_GET_HEAD	(skb_peek(&send_queue))
#define SEND_QUEUE_PROCESSED    (processed_queue_length)
#define INCR_SEND_QUEUE_PROCESSED    (processed_queue_length++)

/* Get next send_queue element */
#define SEND_QUEUE_NEXT(buf)     ((buf)->next)

#define RECEIVE_LIST_MAX_SIZE MAX_FRAME_NUM
/* This should depend on the BP transmitting side*/

typedef struct LD_MINI_TRACE {
	unsigned long function_index;
	unsigned long param;
	unsigned long mux_deferred;
	unsigned long time_stamp;
} LD_MINI_TRACE;

#define LD_MINI_TRACE_LENGTH 100
static int LD_MINI_TRACE_INDEX = 0;
static LD_MINI_TRACE LD_MINI_TRACE_TABLE[LD_MINI_TRACE_LENGTH];

static struct sk_buff *receive_commbuff[RECEIVE_LIST_MAX_SIZE];
static HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T
    ipc_readbuff[RECEIVE_LIST_MAX_SIZE];
static unsigned long mux_deferred;

static HW_CTRL_IPC_OPEN_T ipc_channel_desc;
static HW_CTRL_IPC_CHANNEL_T *ipc_channel_handle;

static struct proc_dir_entry *proc_linkdriver_entry = NULL;

static void MUXTransmitComplete(unsigned long nb_bytes);
static void MUXReceiveComplete(unsigned long arg);
static void USBReadCallback(HW_CTRL_IPC_READ_STATUS_T *read_status);
static void USBWriteCallback(HW_CTRL_IPC_WRITE_STATUS_T *write_status);
static void USBTransmit(void);
static unsigned long USBQueue(void *commbuff);
static unsigned long USBInform(void *param1, void *param2);

HW_CTRL_IPC_DATA_NODE_DESCRIPTOR_T write_buff[SEND_QUEUE_MAX_SIZE];

struct wake_lock netmux_to_usb;
struct wake_lock usb_to_netmux;

void TRACE(unsigned long function_index, unsigned long param)
{
	if (LDLogState[0] == LDLOG_COMMAND_MINI_TRACE) {
		LD_MINI_TRACE_TABLE[LD_MINI_TRACE_INDEX].function_index =
		    function_index;
		LD_MINI_TRACE_TABLE[LD_MINI_TRACE_INDEX].param = param;
		LD_MINI_TRACE_TABLE[LD_MINI_TRACE_INDEX].mux_deferred =
		    mux_deferred;
		LD_MINI_TRACE_TABLE[LD_MINI_TRACE_INDEX].time_stamp =
		    jiffies;

		LD_MINI_TRACE_INDEX =
		    (LD_MINI_TRACE_INDEX + 1) % LD_MINI_TRACE_LENGTH;
	}
}

/*
 * LDOutput is used to printk function log on NetMUX LD.
 *
 */

void LDOutput(const char *fmt, ...)
{
#ifdef _DEBUG_
	printk(fmt);
#endif
	/* _DEBUG_ */

	if ((LDLogState[0] == LDLOG_COMMAND_ALL_WORK)
		|| (LDLogState[0] == LDLOG_COMMAND_FUNCTION))
		printk(fmt);
}

/*
 * LOGOutput is used to printk buffer log on NetMUX LD.
 *
 */

void LOGOutput(const char *fmt, ...)
{
#ifdef _LOG_
	printk(fmt);
#endif
	if ((LDLogState[0] == LDLOG_COMMAND_ALL_WORK)
	    || (LDLogState[0] == LDLOG_COMMAND_BUFFER))
		printk(fmt);
}


/*
 * LDPutBuffer is used to printk buffer on NetMUX LD.
 *
 */

void LDPutBuffer(struct sk_buff *skb)
{
	int index;

	printk(KERN_DEBUG "Commbuff: 0x%p  Length: %d  Data:", skb, skb->len);
	for (index = 0; index < skb->len; index++) {
		if (!(index & 0x0F))
			printk(KERN_DEBUG "\n");

		printk(KERN_DEBUG "0x%x ", skb->data[index]);
	}

	printk(KERN_DEBUG "\n");
}


void LOGSKBUFF(struct sk_buff *skb)
{
#ifdef _LOG_
	LDPutBuffer(skb);
#endif
	if ((LDLogState[0] == LDLOG_COMMAND_ALL_WORK)
	    || (LDLogState[0] == LDLOG_COMMAND_BUFFER))
		LDPutBuffer(skb);
}


/*
 * MUXTransmitComplete is triggered by the write callback and
 * runs in a non-interrupt, tasklet context. This function frees
 * the transmitted buffer and lets the NetMUX know that it is safe
 * to send again (if needed).
 *
 * Params:
 * nb_bytes -- no. of bytes transferred
 */
static void MUXTransmitComplete(unsigned long nb_bytes)
{
	int queue_index;

	DEBUG("%s(%lu)\n", __func__, nb_bytes);
	TRACE(3, nb_bytes);

	spin_lock_bh(&ild_lock);

	/* Remove all the buffers from the Q which are sent to BP */
	for (queue_index = 0; queue_index < SEND_QUEUE_PROCESSED;
	     queue_index++) {
		SEND_QUEUE_REMOVE;
	}

	DEBUG("%s(%u)\n", __func__, SEND_QUEUE_LEN);
	DEBUG("%s(%d)\n", __func__, SEND_QUEUE_PROCESSED);

	/* Reset the send queue length processed */
	processed_queue_length = 0;


	if (mux_deferred) {
		if (SEND_QUEUE_LEN <= SEND_QUEUE_LOW_MARK) {
			mux_deferred = 0;
			ifmux.MUXInform((void *) LDP_INFORM_RECOVERED,
					ifmux.id);
		}
	}
	/* we are guaranteed that currently there is no USB transfer in
	 * progress
	 */
	if (!SEND_QUEUE_ISEMPTY) {
		spin_unlock_bh(&ild_lock);
		USBTransmit();
	} else {
		spin_unlock_bh(&ild_lock);

		/* Remove netmux_to_usb wakelock */
		DEBUG("Releasing netmux_to_usb\n");
		wake_unlock(&netmux_to_usb);
	}
}

/*
 * MUXReceiveComplete is triggered by the read callback and runs
 * in a non-interrupt, tasklet context. This function passes the
 * received data up to the NetMUX, allocates a new receive buffer,
 * and starts the next USB read.
 *
 * Params:
 * arg -- not used.
 */
static void MUXReceiveComplete(unsigned long arg)
{
	unsigned int comand;
	HW_CTRL_IPC_STATUS_T status;
	int index;
	struct device *dev = &usb_ipc_data_param.udev->dev;

	DEBUG("%s(%lu)\n", __func__, arg);
	TRACE(6, 0);
	LOG("MUXReceiveComplete()-->\n");

	for (index = 0; index < RECEIVE_LIST_MAX_SIZE; index++) {
		comand = ipc_readbuff[index].comand;

		/* if D bit not set break */
		if (!(comand & DONE_BIT)) {
			NETMUX_FAIL
			("An error occurred on the IPC channel: D bit not set");
		}

		if (ipc_readbuff[index].length > MAX_FRAME_SIZE) {
			panic
			    ("An error occurred on the IPC packet length");
		}

		skb_put(receive_commbuff[index],
			ipc_readbuff[index].length);

		/* Send buffers to MUX without checking for L bit */
		DEBUG("Calling ifmux.MUXReceive\n");
		ifmux.MUXReceive((void *) receive_commbuff[index],
				 ifmux.id);

		LOGSKBUFF(receive_commbuff[index]);

		/* note that this kfree call only decrements our reference*/
		dev_kfree_skb(receive_commbuff[index]);

		/* Allocate new buffer and update ipc_readbuff*/
		receive_commbuff[index] = dev_alloc_skb(LOC_MAX_RCV_SIZ);
		if (!receive_commbuff[index]) {
			NETMUX_FAIL
			("IPC Link Driver 1st attempt failed to obtain receive skbuff\n");

			/* Try allocation again with 2100 bytes */
			receive_commbuff[index] = dev_alloc_skb(LOC_RETRY_RCV_SIZ);
			if (!receive_commbuff[index]) {
				panic("IPC Link Driver 2nd attempt failed to obtain receive skbuff\n");
			}
		}

		ipc_readbuff[index].comand = 0;
		ipc_readbuff[index].length = LOC_MAX_RCV_SIZ;

		ipc_readbuff[index].data_ptr =
		    (char *) (receive_commbuff[index]->data);

		/* invalidate the cache before starting the next
		 * read note that performance may be better if
		 * DMA_FROM_DEVICE is used but that change will
		 * require taking care of possible cache line crossings
		 */
		dma_map_single(dev,receive_commbuff[index]->data,
				LOC_MAX_RCV_SIZ, DMA_BIDIRECTIONAL);
		/* if E bit set stop reading*/
		if (comand & END_BIT)
			break;
	}
	/* Assuming that ipc_readbuff[index+1] to
	 * ipc_readbuff[RECEIVE_LIST_MAX_SIZE -1] remains same
	 * and so initialisation of its fields are not required
	 */
	ipc_readbuff[RECEIVE_LIST_MAX_SIZE - 1].comand = END_BIT;

	/* Release usb_to_netmux wakelock */
	/* This is done here before the driver read as just after
	 * IT can be raised which will then acquire the wakelock.
	 * This is a possible race condition that is avoided by design
	 * if wakelock is released before calliing drive
	 */
	DEBUG("Releasing usb_to_netmux\n");
	wake_unlock(&usb_to_netmux);

	status = hw_ctrl_ipc_read_ex2(ipc_channel_handle, ipc_readbuff);

	if (status != HW_CTRL_IPC_STATUS_OK) {
		for (index = 0; index < RECEIVE_LIST_MAX_SIZE; index++)
			dev_kfree_skb(receive_commbuff[index]);
		NETMUX_FAIL
		("IPC link driver failed to setup IPC receive request: %d\n",
		     status);
	}
}

/*
 * USBReadCallback is the read callback for the ipc.
 * All we do here is let the PM code know that an interesting
 * interrupt has occurred and jump out of interrupt context to
 * let the bottom half do the rest of the work.
 *
 * Params:
 * arg -- the channel number
 */
static void USBReadCallback(HW_CTRL_IPC_READ_STATUS_T *read_status)
{
	DEBUG("%s(0x%p)\n", __func__, read_status);
	TRACE(5, read_status->nb_bytes);

	/* Acquire usb_to_netmux wakelock */
	/* Acquire it as soon as we are notified that data are available
	   in the usb driver */
	DEBUG("Acquire usb_to_netmux\n");
	wake_lock(&usb_to_netmux);

	tasklet_schedule(&read_callback);
}

/*
 * USBWriteCallback is the write callback for the ipc.
 * All we do here is jump out of interrupt context and let
 * the bottom half do all the work.
 *
 * Params:
 * arg -- the channel
 */
static void USBWriteCallback(HW_CTRL_IPC_WRITE_STATUS_T *write_status)
{
	DEBUG("%s(0x%p)\n", __func__, write_status);
	TRACE(2, write_status->nb_bytes);
	/* copy nb_bytes */
	write_callback.data = write_status->nb_bytes;
	tasklet_schedule(&write_callback);
}

/*
 * USBNotifyCallback informs us of any error on the USB channel
 */
static void USBNotifyCallback(HW_CTRL_IPC_NOTIFY_STATUS_T *status)
{
	DEBUG("%s(0x%p)\n", __func__, status);

	printk(KERN_ERR "An error occurred on the IPC channel: %d, %d\n",
	       status->status, status->channel->channel_nb);
}

/*
 * USBTransmit is called whenever data needs to be pushed to
 * the USB, from the send_queue. The ild_lock must not be held
 * when invoking this function. It is assumed that this function
 * will only be invoked when the send_queue is not empty.
 */
static void USBTransmit(void)
{
	struct sk_buff *transmit_commbuff = NULL;
	int index = 0;
	int buff_index = 0;
	int buff_len = 0;
	int frame_num = 0;
	HW_CTRL_IPC_STATUS_T status;
	struct device *dev = &usb_ipc_data_param.udev->dev;
	DEBUG("%s\n", __func__);

	spin_lock_bh(&ild_lock);

	LOG("USBTransmit()-->\n");

	/* the head element is _not_ removed from the list and the
	 * ref count is unchanged
	 */
	transmit_commbuff = SEND_QUEUE_GET_HEAD;

	/* Get all the buffers in the Queue */
	for (index = 0; index < SEND_QUEUE_LEN; index++) {
		frame_num =
		    (transmit_commbuff->len - 1) / remMaxRcvSiz + 1;
		if (frame_num > SEND_QUEUE_MAX_SIZE) {
			NETMUX_FAIL
			("size too big, can\'t send, discard the package\n");
		} else if (buff_index + frame_num > SEND_QUEUE_MAX_SIZE) {
			LOG("send queue full\n");
			break;
		}

		INCR_SEND_QUEUE_PROCESSED;
		buff_len = 0;
		while (transmit_commbuff->len - buff_len > remMaxRcvSiz) {
			write_buff[buff_index].length = remMaxRcvSiz;
			write_buff[buff_index].comand = LAST_BIT;
			write_buff[buff_index].data_ptr =
			    (char *) (transmit_commbuff->data + buff_len);
			buff_index++;
			buff_len += remMaxRcvSiz;
		}
		write_buff[buff_index].comand = LAST_BIT;
		write_buff[buff_index].length =
		    transmit_commbuff->len - buff_len;
		write_buff[buff_index].data_ptr =
		    (char *) (transmit_commbuff->data + buff_len);
		buff_index++;

		/* invalidate the cache before starting the next write
		 * note that performance may be better if DMA_FROM_DEVICE
		 * is used but that change will require taking care of
		 * possible cache line crossings
		 */
		dma_map_single(dev,transmit_commbuff->data,
				transmit_commbuff->len, DMA_BIDIRECTIONAL);
		LOGSKBUFF(transmit_commbuff);

		transmit_commbuff = SEND_QUEUE_NEXT(transmit_commbuff);
	}
	/* Set the E bit in the last buffer
	 * if(buff_index != 0), this can't be the case so not needed
	 */
	write_buff[buff_index - 1].comand |= END_BIT;
	spin_unlock_bh(&ild_lock);

	TRACE(1, SEND_QUEUE_LEN);
	status = hw_ctrl_ipc_write_ex2(ipc_channel_handle, write_buff);

	if (status != HW_CTRL_IPC_STATUS_OK)
		NETMUX_FAIL
		    ("IPC Link Driver failed to send data to IPC: %d\n",
		     status);
}

/*
 * USBQueue is called by the NetMUX to send data. If the send_queue is not
 * full, data will be added to it, transmission will be started (if the USB
 * was free), and ERROR_NONE will be returned to the NetMUX; otherwise the
 * NetMUX will be deferred and asked to contact us later.
 *
 * Params:
 * param -- the data to be sent.
 */
static unsigned long USBQueue(void *param)
{

	spin_lock_bh(&ild_lock);

	/* acquire netmux_to_usb wakelock */
	DEBUG("Acquire netmux_to_usb\n");
	wake_lock(&netmux_to_usb);


	/* tell NetMUX to wait since the queue is full */
	if (SEND_QUEUE_LEN == SEND_QUEUE_MAX_SIZE) {
		mux_deferred = 1;
		spin_unlock_bh(&ild_lock);
		return LDP_ERROR_RECOVERABLE;
	}

	SEND_QUEUE_ADD((struct sk_buff *) param);

	/* if the queue was empty before our addition above, then
	 * the USB was free and therefore we can begin the next
	 * transmission
	 */
	if (SEND_QUEUE_LEN - 1 == 0) {
		spin_unlock_bh(&ild_lock);
		/* we're going to send a packet */
		USBTransmit();
	} else {
		spin_unlock_bh(&ild_lock);
	}

	return LDP_ERROR_NONE;
}

/*
 * USBInform is not used currently. Theoretically the NetMUX can use
 * this function to communicate with the USB.
 *
 * Params:
 * param1 -- type?
 * param2 -- data?
 */
static unsigned long USBInform(void *param1, void *param2)
{
	DEBUG("%s(0x%p, 0x%p)\n", __func__, param1, param2);

	switch ((unsigned long) param1) {
	case LDP_INFORM_SHUTDOWN:
		{
		}
		break;

	default:
		break;
	}

	return LDP_ERROR_NONE;
}

/*
 * GetLinkdriverInfo is a read callback function
 * of proc interface on NetMUX LD.
 *
 */

static int GetLinkdriverInfo(char *buf, char **start, off_t offset,
			     int count, int *eof, void *data)
{
	int len = 0;
	int index, index_offset;

	if (offset) {
		len -= offset;
		if (len < 0) {
			*eof = 1;
			return 0;
		}

		if (len > count)
			len = count;
		memcpy(buf, &ld_info[offset], len);
		*start = buf;
		return len;
	}

	len = 0;

	len += sprintf(&ld_info[len], "Linkdriver Status:\n");
	len +=
	    sprintf(&ld_info[len], "\tmux_deferred:      %lu\n",
		    mux_deferred);
	len += sprintf(&ld_info[len], "\tqueue_len: %d\n", SEND_QUEUE_LEN);
	len +=
	    sprintf(&ld_info[len], "\tlinkdriver log status: %s\n",
		    LDLogState);

	if (LDLogState[0] == LDLOG_COMMAND_MINI_TRACE) {
		len +=
		    sprintf(&ld_info[len], "\nLinkdriver Mini Trace:\n");
		index_offset = LD_MINI_TRACE_INDEX;
		for (index = 0; index < LD_MINI_TRACE_LENGTH; index++) {
			len +=
			    sprintf(&ld_info[len],
				    "\t%lu \t%lu \t%lu \t%lu\n",
				    LD_MINI_TRACE_TABLE[index_offset].
				    function_index,
				    LD_MINI_TRACE_TABLE[index_offset].
				    param,
				    LD_MINI_TRACE_TABLE[index_offset].
				    mux_deferred,
				    LD_MINI_TRACE_TABLE[index_offset].
				    time_stamp);

			index_offset =
			    (index_offset + 1) % LD_MINI_TRACE_LENGTH;
		}
	}

	if (len > count)
		len = count;
	memcpy(buf, &ld_info[offset], len);
	*start = buf;

	return len;
}

/*
 * WriteLDLogCommand is a write callback function of
 * proc interface on NetMUX LD.
 *
 */

static int WriteLDLogCommand(struct file *file, const char *buffer,
			     unsigned long count, void *data)
{
	int len;
	int index;

	len = count;
	index = 0;
	if (len > LDLOG_COMMAND_LEN) {
		printk(KERN_ERR "Error : LD log command is invalid\n");
		return -ENOSPC;
	}
	if (copy_from_user(&LDLogState[index], buffer, len))
		return -EFAULT;
	LDLogState[LDLOG_COMMAND_LEN - 1] = '\0';

	return len;
}

/*
 * LDProcCleanup is a exit function of proc interface on NetMUX LD.
 *
 */

static void LDProcCleanup(void)
{
#ifdef CONFIG_DEBUG_NETMUX
	remove_proc_entry("linkdriver", 0);
	kfree(ld_info);
#endif
}

/*
 * LDInit, initialize everything.
 */
void LDInit(void)
{
	HW_CTRL_IPC_STATUS_T status;
	int index;
	int result = -1;

	struct device *dev = &usb_ipc_data_param.udev->dev;
	DEBUG("%s()\n", __func__);

	tasklet_init(&write_callback, &MUXTransmitComplete, 0);
	tasklet_init(&read_callback, &MUXReceiveComplete, 0);
	skb_queue_head_init(&send_queue);

	wake_lock_init(&netmux_to_usb, WAKE_LOCK_SUSPEND,
		       "LD_netmux_to_usb");
	wake_lock_init(&usb_to_netmux, WAKE_LOCK_SUSPEND,
		       "LD_usb_to_netmux");

#ifdef CONFIG_DEBUG_NETMUX
	LDLogState[0] = '0';
	LDLogState[1] = '\0';

	proc_linkdriver_entry = create_proc_entry("linkdriver", 0x660, 0);
	if (!proc_linkdriver_entry) {
		remove_proc_entry("linkdriver", 0);
	} else {
		proc_linkdriver_entry->read_proc = GetLinkdriverInfo;
		proc_linkdriver_entry->write_proc = WriteLDLogCommand;
	}

	ld_info = kmalloc(DEFAULT_PROC_DATA_SIZE, GFP_KERNEL);
	if (!ld_info) {
		remove_proc_entry("linkdriver", 0);
		goto unregDev;
	}
#endif
	/* Populate channel desc and open USB channel for data transfer */
	ipc_channel_desc.type = HW_CTRL_IPC_PACKET_DATA;
	ipc_channel_desc.index = USB_CHANNEL;
	ipc_channel_desc.read_callback = &USBReadCallback;
	ipc_channel_desc.write_callback = &USBWriteCallback;
	ipc_channel_desc.notify_callback = &USBNotifyCallback;

	ipc_channel_handle = hw_ctrl_ipc_open(&ipc_channel_desc);
	if (ipc_channel_handle == NULL) {
		printk(KERN_ERR "IPC link driver: failed to obtain IPC channel\n");
		goto unregDev;
	}

	iflink.LinkSend = &USBQueue;
	iflink.LinkInform = &USBInform;
	iflink.localMaxRcvSize = LOC_MAX_RCV_SIZ;
	iflink.remoteMaxRcvSize = remMaxRcvSiz;
	mux_deferred = 0;

	result = RegisterMUXLink(&iflink, &ifmux);
	if (result != LDP_ERROR_NONE) {
		printk
		    ("IPC link driver: failed to register with NetMUX: %d\n",
		     result);
		goto closeUSBChan;
	}
	/* Start the first read */
	for (index = 0; index < RECEIVE_LIST_MAX_SIZE; index++) {
		receive_commbuff[index] = dev_alloc_skb(LOC_MAX_RCV_SIZ);
		if (!receive_commbuff[index]) {
			/* free allocated buffers */
			while (index) {
				dev_kfree_skb(receive_commbuff[index - 1]);
				index--;
			}
			printk(KERN_ERR
			"IPC link driver: \
			failed to allocate receive buffer\n");
			goto unregMux;
		}
		ipc_readbuff[index].comand = 0;
		ipc_readbuff[index].length = LOC_MAX_RCV_SIZ;

		ipc_readbuff[index].data_ptr =
		    (char *) (receive_commbuff[index]->data);

		/* invalidate the cache before starting the next read
		 * note that performance may be better if DMA_FROM_DEVICE
		 * is used but that change will require taking care
		 * of possible cache line crossings
		 */
		dma_map_single(dev,receive_commbuff[index]->data,
				LOC_MAX_RCV_SIZ, DMA_BIDIRECTIONAL);
	}

	ipc_readbuff[RECEIVE_LIST_MAX_SIZE - 1].comand = END_BIT;

	status = hw_ctrl_ipc_read_ex2(ipc_channel_handle, ipc_readbuff);
	if (status != HW_CTRL_IPC_STATUS_OK) {
		printk
		    ("IPC link driver: failed to post ipc read buffer: %d\n",
		     status);

		goto freeDataBuf;
	}

	/* return good status */
	return;

freeDataBuf:
	for (index = 0; index < RECEIVE_LIST_MAX_SIZE; index++)
		dev_kfree_skb(receive_commbuff[index]);
unregMux:
	UnregisterMUXLink(ifmux.id);
closeUSBChan:
	hw_ctrl_ipc_close(ipc_channel_handle);
unregDev:
	/* return a failure status */
	return;
}

/*
 * init_module, register init function to IPC.
 */
int init_module(void)
{
	hw_ctrl_ipc_register(LDInit);
	return 0;
}


/*
 * cleanup_module cleans up everything.
 *
 * NOTE: commented out at this time since LD should not be removed
 *       Maybe reinstated later
 */
/*
void cleanup_module (void)
{
    int index;
    HW_CTRL_IPC_STATUS_T status;

    DEBUG("%s()\n", __func__);

    UnregisterMUXLink(ifmux.id);

    LDProcCleanup();

    status = hw_ctrl_ipc_close(ipc_channel_handle);
    if (status != HW_CTRL_IPC_STATUS_OK)
	printk(KERN_INFO "ipc close error, status = %d\n", status);
    else
	printk(KERN_INFO "ipc close OK\n");

    skb_queue_purge(&send_queue);

    for(index =0;index < RECEIVE_LIST_MAX_SIZE;index++)
    {
	if (receive_commbuff[index])
		dev_kfree_skb(receive_commbuff[index]);
    }

    wake_lock_destroy(&netmux_to_usb);
    wake_lock_destroy(&usb_to_netmux);


}
*/
