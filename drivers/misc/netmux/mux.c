/******************************************************************************
 * NetMUX mux.c                                                               *
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
 *   2006/12/19  Motorola    Combine header and data into one transfer        *
 *   2007/04/11  Motorola    Reduce lock/unlocks in ProcessSendQueues         *
 *   2007/05/01  Motorola    Change codes to ensure "shared" netmux           *
 *                           code is identical between AP and BP.             *
 *   2007/08/16  Motorola    Add support for larger bytecredit                *
 *   2007/12/05  Motorola    Change codes as INIT_WORK changes in kernel      *
 *   2008/07/14  Motorola    fix memory leak issue                            *
 *   2008/10/14  Motorola    Panic if receive packet is invalide              *
 *   2008/10/28  Motorola    fix issue in ReceivePartial                      *
 *   2009/07/23  Motorola    Add wake lock functionality                      *
 *   2009/11/18  Motorola    Switch host/client interface in DC resp          *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* mux.c defines all the functionality of the mux. This functionality allows  */
/* an interface to communicate over a link.                                   */

#include "mux.h"
#include "protocol.h"
#include "debug.h"
#include <linux/wakelock.h>

extern struct wake_lock netmux_send_wakelock;
extern struct wake_lock netmux_receive_wakelock;

void check_all_receive_queues_emptiness(MUX *mux);
void check_all_send_queues_emptiness(MUX *mux);

/*
 * CloseChannel encapsulates the set of actions that are needed to close a
 * channel
 *
 * Callers are responsible for obtaining the mux lock before calling
 * CloseChannel and for releasing the lock after this function returns.
 *
 * Params:
 * chanlNum -- the number of the channel being closed
 * mux -- the mux object pointer
 */
static
void CloseChannel(int32 chanlNum, MUX *mux)
{
	CHANNEL *channel = NULL;

	channel = mux->channels[chanlNum];

	if (channel) {

		mux->channels[chanlNum] = 0;
		mux->total_queued_amount -= channel->qed_totl_amount;

		empty_commbuff_queue(&channel->receive_queue);
		empty_commbuff_queue(&channel->send_queue);

		destroy_commbuff_queue(&channel->receive_queue);
		destroy_commbuff_queue(&channel->send_queue);

		free_mem(channel);
	}
}


/*
 * ExecuteStateTransition is a helper function that can change a channel's
 * state based on a particular event happening and perform some of the
 * actions associated with the state transition.
 *
 * Callers must obtain the mux lock before calling and are responsible for
 * releasing the lock after this function returns.
 *
 * MAINTENANCE NOTE: not all state transitions and not all actions are handled
 * by this function.  For example, this function does not call any interface
 * inform functions since the mux lock is held here.
 *
 * Params:
 * event -- the event that occurred
 * mux -- the mux object
 * channel_num -- the channel number the event occurred on
 */
int32 ExecuteStateTransition(int32 event, MUX *mux, int32 channel_num)
{
	CHANNEL *channel;

	DEBUG("ExecuteStateTransition(0x%lu, %p, 0x%lu)\n", event, mux,
	      channel_num);

	channel = mux->channels[channel_num];

	switch (channel->state) {
	case OPENING:
		{
			switch (event) {
			case LOCAL_CLOSE:
				{
					/* send client close and wait for ack */
					TransmitDisableChannel((int8)
							       client_end
							       (COMMAND),
							       (int8)
							       channel_num,
							       (int8)
							       channel->
							       host_interface,
							       (int8)
							       channel->
							       client_interface,
							       mux);

					channel->state = LOCAL_CLOSING;
				}
				break;

			default:
				{
					return
					    DEBUGERROR
					    (ERROR_OPERATIONFAILED);
				}
			}
		}
		break;

	case OPEN:
		{
			switch (event) {
				/* the REMOTE_DATA event is
				 * handled outside of this function
				 */

			case LOCAL_DATA:
				{
					channel->state = SENDING;
					/* data is queued and send task
					 * is scheduled by the caller
					 */
				}
				break;

			case REMOTE_CLOSE:
				{
					channel->state = REMOTE_CLOSING;
					/* the interface will be informed
					 * by the caller
					 */
				}
				break;

			case LOCAL_CLOSE:
				{
					/* send client close and wait for ack*/
					TransmitDisableChannel((int8)
						       client_end
						       (COMMAND),
						       (int8)
						       channel_num,
						       (int8)
						       channel->
						       host_interface,
						       (int8)
						       channel->
						       client_interface,
						       mux);

					channel->state = LOCAL_CLOSING;
				}
				break;

			default:
				{
					return
					    DEBUGERROR
					    (ERROR_OPERATIONFAILED);
				}
			}
		}
		break;

	case SENDING:
		{
			switch (event) {
				/* the REMOTE_DATA event is
				 * handled outside of this function
				 */

			case LOCAL_CLOSE:
				{
					/* finish sending the data first
					 * we'll send a close response later
					 */
					channel->state = FLUSHING;
				}
				break;

			case REMOTE_CLOSE:
				{
					/* purge this channel's send queue */
					empty_commbuff_queue(&channel->
							     send_queue);
					channel->state = REMOTE_CLOSING;
				/* the interfaces will be informed
				 * by the caller
				 */
				}
				break;

			case SEND_COMPLETE:
				{
					channel->state = OPEN;
				}
				break;

			case LOCAL_DATA:
				{
					/* we're already sending
					 * so no state change or
					 * action is needed
					 */
				}
				break;

			default:
				{
					return
					    DEBUGERROR
					    (ERROR_OPERATIONFAILED);
				}
			}
		}
		break;

	case FLUSHING:
		{
			switch (event) {
			/* the REMOTE_DATA event is handled
			 * outside of this function
			 */

			case REMOTE_CLOSE:
				{
					/* we got a close from the
					 * remote while we getting ready to
					 * send our own close
					 * so ditch the data
					 * and send a response
					 */
					empty_commbuff_queue(&channel->
							     send_queue);

					/* the local channel has already
					 * triggered a close immediately
					 * sending back a response to
					 * the remote switch the
					 * interfaces as a result
					 */
					TransmitDisableChannel((int8)
						       host_end
						       (SUCCESS),
						       (int8)
						       channel_num,
						       (int8)
						       channel->
						       client_interface,
						       (int8)
						       channel->
						       host_interface,
						       mux);
					/* this channel will be closed
					 * by the caller
					 */
				}
				break;

			case SEND_COMPLETE:
				{
					/* now that the data is flushed
					 * we can send a close
					 */
					channel->state = LOCAL_CLOSING;

					TransmitDisableChannel((int8)
						       client_end
						       (COMMAND),
						       (int8)
						       channel_num,
						       (int8)
						       channel->
						       host_interface,
						       (int8)
						       channel->
						       client_interface,
						       mux);
				}
				break;

			default:
				{
					return
					    DEBUGERROR
					    (ERROR_OPERATIONFAILED);
				}
			}
		}
		break;

	case LOCAL_CLOSING:
		{
			switch (event) {
				/* the REMOTE_DATA event is handled outside of
				 * this function
				 * the OPEN_RESPONSE event is
				 * handled outside of this function
				 */

			case REMOTE_CLOSE:
				{
					/* we got a close from the remote
					 * while we are closing so just
					 * send a response and keep waiting
					 * for a response to the close we sent
					 */

					/* the local channel has already
					 * triggered a close immediately
					 * sending back a response to the
					 * remote switch the interfaces
					 * as a result
					 */
					TransmitDisableChannel((int8)
						       host_end
						       (SUCCESS),
						       (int8)
						       channel_num,
						       (int8)
						       channel->
						       client_interface,
						       (int8)
						       channel->
						       host_interface,
						       mux);
				}
				break;

			case CLOSE_RESPONSE:
				{
					/* we got the response we were
					 * expecting, now we're closed
					 */
					CloseChannel(channel_num, mux);
					/* the interface is informed
					 * by the caller
					 */
				}
				break;

			default:
				{
					return
					    DEBUGERROR
					    (ERROR_OPERATIONFAILED);
				}
			}
		}
		break;

	case REMOTE_CLOSING:
		{
			switch (event) {
			case LOCAL_CLOSE:
				{
					/* send the close response */
					TransmitDisableChannel((int8)
						       host_end
						       (SUCCESS),
						       (int8)
						       channel_num,
						       (int8)
						       channel->
						       host_interface,
						       (int8)
						       channel->
						       client_interface,
						       mux);

					/* free the channel resources */
					CloseChannel(channel_num, mux);
				}
				break;

			default:
				{
					return
					DEBUGERROR(ERROR_OPERATIONFAILED);
				}
			}
		}
		break;

	default:
		{
			return DEBUGERROR(ERROR_OPERATIONFAILED);
		}
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * InformMUX is called by the linkdriver when a change of status
 * needs to be reported to the NetMUX. The status could be an
 * error state or a working state. If the state is made 'working'
 * then the mux continues its work.
 *
 * Params:
 * type -- the state to switch to
 * param -- data associated with the type
 */
int32 InformMUX(void *type, void *param)
{
	MUX *mux;

	DEBUG("InformMUX(%p, %p)\n", type, param);

	mux = (MUX *) param;

	if (!mux)
		return DEBUGLDP(LDP_ERROR_FATAL);

	switch ((int32) type) {
	case LDP_INFORM_SHUTDOWN:
		{
			task_schedule(&mux->shutdown_task);
		}
		break;

	case LDP_INFORM_RECOVERED:
		{
			enable_task(&mux->send_task);
			task_schedule(&mux->send_task);
		}
		break;
	}

	return DEBUGLDP(LDP_ERROR_NONE);
}

/*
 * CommBuffReleased is called when a commbuff is freed somewhere
 * on the system. This function then handles any credit adjustments.
 *
 * Params:
 * channel -- the channel the commbuff belongs to
 * size -- the size of the commbuff being released
 * param -- the mux object
 */
void CommBuffReleased(int32 channel, int32 size, void *param)
{
	MUX *mux;
	CHANNELCREDIT *credit;
	int32 replenish_send_limit;
	int32 replenish_byte_limit;
	int32 buffers_used;

	DEBUG("CommBuffReleased(%lu, %lu, %p)\n", channel, size, param);
	DBGFC("CommBuffReleased(%lu, %lu, %p)\n", channel, size, param);

	mux = (MUX *) param;

	enter_write_criticalsection(&mux->lock);

	buffers_used = size / mux->local_rcv_buffer_size;

	if (size % mux->local_rcv_buffer_size)
		buffers_used++;

	credit = &mux->channelcredit[channel];

	credit->replenished_byte_credit +=
	    (size - sizeof(DATA_PACKET_HDR));
	credit->replenished_send_credit += buffers_used;

	replenish_send_limit =
	    credit->max_host_send_credit /
	    MUX_SENDCREDIT_SEND_LIMIT_DIVISOR;
	replenish_byte_limit =
	    credit->max_host_byte_credit /
	    MUX_BYTECREDIT_SEND_LIMIT_DIVISOR;

	if (credit->replenished_send_credit >= replenish_send_limit ||
	    credit->replenished_byte_credit >= replenish_byte_limit) {
		DBGFC("Sending Credit: %d, %d, %d", channel,
		      credit->replenished_byte_credit,
		      credit->replenished_send_credit);

		AdjustCredit(host_end(COMMAND),
			     (int8) channel,
			     credit->replenished_byte_credit,
			     credit->replenished_send_credit, mux);

		credit->replenished_byte_credit = 0;
		credit->replenished_send_credit = 0;
	}

	exit_write_criticalsection(&mux->lock);
}

/*
 * ProcessSendQueues is responsible for the actual sending of data
 * from the mux to a linkdriver. Data will be delivered from the lowest
 * numbered channel first. If the linkdriver returns an error on the
 * send the mux stops sending and waits for the linkdriver to inform
 * the mux that it is okay to run again.
 *
 * Params:
 * param -- the pointer to a mux object
 */
void ProcessSendQueues(struct work_struct *work)
{
	INTERFACEINFORM informdata;
	CHANNEL **channels;
	CHANNELCREDIT *channelcredits;
	CHANNELCREDIT *credit;
	int32(*Send) (void *);
	MUX *mux;
	COMMBUFF *commbuff;
	int32 result;
	int32 buffLength;
	int32 dataLength;
	int32 numBuffers;
	int32 ignore_amount = 0;
	COMMBUFFQUEUE *currentQ;
	CHANNEL *chanlPtr;
	static int32 chanlNum = 0;
	int32 startChanlNum = chanlNum;
	int32 holding_lock = 0;

	DEBUG("ProcessSendQueues(%p)\n", work);

	mux = container_of(work, MUX, send_task.work);

	Send = mux->Send;
	channels = mux->channels;
	channelcredits = mux->channelcredit;

	credit = NULL;

	disable_task(&mux->send_task);

	/* tell the interfaces that they can do their send prep work */
	informdata.inform_type = INFORM_INTERFACE_PREPSEND;
	informdata.source = mux;
	informdata.data = NULL;
	BroadcastLibraryInform(&informdata, mux->interface_lib);

	/* pre-set the inform type used when we let the interfaces */
	/* know that we finished sending their data                */
	informdata.inform_type = INFORM_INTERFACE_DATA;

	enter_write_criticalsection(&mux->lock);
	holding_lock = 1;

	/* while there is queued data that we can't ignore, keep trying */
	/* to send unless the Link Driver tells us to go away           */
	while (mux->total_queued_amount > ignore_amount) {
		if (!holding_lock) {
			enter_write_criticalsection(&mux->lock);
			holding_lock = 1;
		}

		currentQ = &(mux->send_queue);
		chanlPtr = NULL;
		commbuff = NULL;
		/* these next assignments make the compiler happier although */
		/* they do nothing practical for the code                    */
		numBuffers = 0;
		buffLength = 0;
		dataLength = 0;

		/* see whether there is some NetMUX control info to send */
		if (queue_length(currentQ)) {
			if (mux->send_buffers_available == 0) {
				exit_write_criticalsection(&mux->lock);
				enable_task(&mux->send_task);
				return;
			}
			commbuff = dequeue_commbuff(currentQ);
			buffLength = commbuff_length(commbuff);
		}
		/* otherwise, starting at the top, look for an
		 * open channel that has data and the credit to send it
		 */
		else {
			ignore_amount = 0;

			/* make one pass through the channels
			 * looking for something to send
			 */
			do {
				chanlPtr = channels[chanlNum];
				credit = &channelcredits[chanlNum];

				/* look for open channels with data on them*/
				if (chanlPtr
				    && (chanlPtr->
					state & (SENDING | FLUSHING))) {
					currentQ = &(chanlPtr->send_queue);
					commbuff =
					    queue_frontbuff(currentQ);
					buffLength =
					    commbuff_length(commbuff);
					dataLength =
					    buffLength -
					    sizeof(DATA_PACKET_HDR);

					numBuffers =
					    buffLength /
					    mux->remote_rcv_buffer_size;
					if (buffLength %
					    mux->remote_rcv_buffer_size)
						numBuffers++;

					/* look for channels
					 * with available credit
					 */
					if (numBuffers >
					    credit->client_send_credit
					    || dataLength >
					    credit->client_byte_credit
					    || numBuffers >
					    mux->send_buffers_available) {
						commbuff = NULL;
						ignore_amount +=
						    chanlPtr->
						    qed_totl_amount;
					} else {
						/* we have a winner! */
						commbuff =
						    dequeue_commbuff
						    (currentQ);
						break;
					}
				}

				/* increment the loop index with wrap-around */
				chanlNum++;
				if (chanlNum == mux->maxchannels)
					chanlNum = 0;

			} while (chanlNum != startChanlNum);
			startChanlNum = chanlNum;
		}

		/* if we found some data... */
		if (commbuff) {
			/* first, log it
			 * to address a BP race condition over
			 * the buffer ownership, log the contents here
			 * and log whether or not the send succeeded below
			 */
			LOGCOMMBUFF_CH(chanlNum, "ProcessSendQueues()-->",
				       commbuff, buffLength);

			/* ...then try to send it */
			result = Send((void *) commbuff);

			/* if we were deferred... */
			if (result == LDP_ERROR_RECOVERABLE) {
				/* ...re-queue the data */
				queuefront_commbuff(commbuff, currentQ);

				exit_write_criticalsection(&mux->lock);

				/* indicate that the send is delayed */
				LOG_TRACE
				    ("ProcessSendQueues()-->send delayed,  \
					data re-queued\n");

				/* leave without enabling the task so that */
				/* it can be scheduled if necessary        */
				return;
			} else if (result != LDP_ERROR_NONE) {
				PANIC(netmuxPanicILDFail,
				"NetMUX Link Driver delivered a fatal error\n");
			}

			/* indicate that the send succeeded */
			LOG_TRACE
			    ("ProcessSendQueues()-->send succeeded\n");

			/* free up the buffer we just sent successfully
			 * to the Link Driver
			 */
			deref_commbuff(commbuff);

			/* do the accounting */
			mux->total_queued_amount -= buffLength;

			if (chanlPtr != NULL) {
				chanlPtr->qed_data_amount -= dataLength;
				chanlPtr->qed_totl_amount -= buffLength;
				credit->client_byte_credit -= dataLength;
				credit->client_send_credit -= numBuffers;
				mux->send_buffers_available -= numBuffers;

				if (!(chanlPtr->qed_data_amount)) {
					/* let the interface know that
					 *  we finished sending its data
					 */
					informdata.data =
					    (void *) chanlNum;

					(void)
					    ExecuteStateTransition
					    (SEND_COMPLETE, mux, chanlNum);

					holding_lock = 0;
					exit_write_criticalsection(&mux->
								   lock);

					LIBRARY_INFORM(&informdata,
						       chanlPtr->
						       connected_interface->
						       interface_index,
						       mux->interface_lib);
				}

				/* since we broke from the inner
				 * while loop above, increment the channel
				 * index to go to the next channel
				 */
				chanlNum++;
			}
		}
	}

	if (holding_lock)
		exit_write_criticalsection(&mux->lock);

	enter_write_criticalsection(&mux->lock);
	check_all_send_queues_emptiness(mux);
	exit_write_criticalsection(&mux->lock);

	enable_task(&mux->send_task);
}


/*
 * ProcessReceiveQueues takes completed data from the receive
 * queues and delivers it to the appropriate place. If the
 * data belongs to the mux it is delivered as a command to
 * be parsed and if it is from a channel it goes to the
 * respective interface.
 *
 * Params:
 * param -- the mux object
 */
void ProcessReceiveQueues(struct work_struct *work)
{
	INTERFACEINFORM inform_data;
	CHANNEL **channels;
	MUX *mux;
	CHANNEL *recv_channel;
	COMMBUFF *commbuff;
	COMMBUFFQUEUE *queue;
	int32 channel;
	int32 result;
	int32 length;

	DEBUG("ProcessReceiveQueues(%p)\n", work);

	mux = container_of(work, MUX, receive_task.work);

	channels = mux->channels;
	inform_data.source = mux;
	queue = &mux->receive_queue;

	disable_task(&mux->receive_task);

	/* move everything from the mux receive queue
	 * to the application queues
	 */
	while (queue_length(queue)) {
		commbuff = dequeue_commbuff(queue);
		ReceivePartial(commbuff, mux);
	}

	enter_write_criticalsection(&mux->lock);

	/* then process the application receive queues */
	for (channel = 0; channel < mux->maxchannels; channel++) {
		recv_channel = channels[channel];
		if (recv_channel) {
			inform_data.data = (void *) channel;
			inform_data.inform_type =
			    recv_channel->connected_interface->param;
			queue = &recv_channel->receive_queue;

			while (queue_length(queue)) {
				commbuff = dequeue_commbuff(queue);

				if (!
				    (recv_channel->
				     state & (OPEN | SENDING))) {
					/* if we're not in a position to
					 * forward this data to the interface
					 * then we should discard it
					 */
					free_commbuff(commbuff);
					continue;
				}

				length = commbuff_length(commbuff);

				result =
				    recv_channel->connected_interface->
				    Receive(commbuff, &inform_data);
				if (result != ERROR_NONE) {
					queuefront_commbuff(commbuff,
							    queue);

					if (result !=
					    ERROR_OPERATIONRESTRICTED)
						task_schedule(&mux->
							      receive_task);

					break;
				}
			}
		}
	}

	check_all_receive_queues_emptiness(mux);
	exit_write_criticalsection(&mux->lock);
	enable_task(&mux->receive_task);
}

/*
 * ShutdownMUX is called if the linkdriver informs the mux of a
 * fatal error. This task is scheduled to ensure we are in a safe
 * context and then the mux gets shutdown.
 *
 * Params:
 * param -- the mux object
 */
void ShutdownMUX(struct work_struct *work)
{
	INTERFACEINFORM inform_data;
	MUX *mux;

	DEBUG("ShutdownMUX(%p)\n", work);

	mux = container_of(work, MUX, shutdown_task.work);

	inform_data.source = mux;
	inform_data.inform_type = LDP_INFORM_SHUTDOWN;

	BroadcastLibraryInform(&inform_data, mux->interface_lib);
	DisableMUX(client_end(COMMAND), mux);
}

/*
 * ReceiveFromLink is called whenever data is received from the linkdriver.
 * The data will be pieced together and routed appropriately.
 *
 * Params:
 * buff -- the data received
 * param -- the mux object
 */
int32 ReceiveFromLink(void *buff, void *param)
{
	MUX *mux;
	COMMBUFF *commbuff;

	DEBUG("ReceiveFromLink(%p, %p)\n", buff, param);

	/* Acquire NM_receive wakelock */
	DEBUG("Acquire netmux_receive_wakelock\n");
	wake_lock(&netmux_receive_wakelock);

	commbuff = (COMMBUFF *) buff;
	mux = (MUX *) param;

	if (!commbuff || !mux)
		return DEBUGLDP(LDP_ERROR_FATAL);

	LOGCOMMBUFF_RECV_LD("ReceiveFromLink()-->", commbuff,
			    commbuff_length(commbuff));

	ref_commbuff(commbuff);

	queue_commbuff(commbuff, &mux->receive_queue);
	task_schedule(&mux->receive_task);

	return DEBUGLDP(LDP_ERROR_NONE);
}

/*
 * ReceivePartial is called to handle data received on the mux receive queue.
 * The algorithm here depends on the fact that multiple messages will never
 * cross commbuff boundaries.
 *
 * Params:
 *   commbuff -- the data received
 *   mux      -- the mux object
 */
void ReceivePartial(COMMBUFF *commbuff, MUX *mux)
{
	PARTIAL_RECEIVE *part_recv;

	DEBUG("ReceivePartial(%p, %p)\n", commbuff, mux);

	part_recv = &mux->partial_receive;

	if (part_recv->buffer) {
		/* we're already building up from something previously
		 * received so just merge this one into the previous
		 * one and update the stats
		 */
		part_recv->buffer =
		    commbuff_merge(part_recv->buffer, commbuff);
		part_recv->packet.payload =
		    commbuff_data(part_recv->buffer);
	} else {
		/* we're starting a new message so start everything over */

		part_recv->buffer = commbuff;
		part_recv->packet.payload = commbuff_data(commbuff);
		part_recv->type = part_recv->packet.payload[0];

		switch (part_recv->type) {
		case PACKETTYPE_DATA_HDR:
			{
				part_recv->expectedLen =
				    sizeof(DATA_PACKET_HDR);
			}
			break;

		case PACKETTYPE_CREDIT:
			{
				part_recv->expectedLen =
				    sizeof(CREDIT_PACKET);
			}
			break;

		case PACKETTYPE_ENABLEMUX:
			{
				part_recv->expectedLen =
				    sizeof(ENABLEMUX_PACKET);
			}
			break;

		case PACKETTYPE_DISABLEMUX:
			{
				part_recv->expectedLen =
				    sizeof(DISABLEMUX_PACKET);
			}
			break;

		case PACKETTYPE_ENABLECHANNEL:
			{
				part_recv->expectedLen =
				    sizeof(ENABLECHANNEL_PACKET);
			}
			break;

		case PACKETTYPE_DISABLECHANNEL:
			{
				part_recv->expectedLen =
				    sizeof(DISABLECHANNEL_PACKET);
			}
			break;

		case PACKETTYPE_QUERYINTERFACE:
			{
				part_recv->expectedLen =
				    sizeof(QUERYINTERFACE_PACKET);
			}
			break;

		case PACKETTYPE_CHANNELSIGNAL:
			{
				part_recv->expectedLen =
				    sizeof(CHANNELSIGNAL_PACKET);
			}
			break;

		default:
			{
				DEBUG("NETMUX ERROR: DISCARDING INVALID \
				 PACKET TYPE, type %u\n",
				(part_recv->type));
				free_commbuff(commbuff);
				PANIC(netmuxPanicPkgFail1,
				      "packet type invalid");
			}
		}
	}

	part_recv->currentLen = commbuff_length(part_recv->buffer);

	if (part_recv->currentLen >= part_recv->expectedLen) {
		/* make sure that the packet/header is in contiguous memory */
		/* this is only a cbuf issue at the moment and will only be */
		/* a real problem if the link driver passes up buffers that */
		/* are smaller than the largest packet/header size          */
		if (part_recv->type != PACKETTYPE_DATA_BDY) {
			commbuff_data_pullup(part_recv->buffer,
					     part_recv->expectedLen);
		}

		/* we got at least a header so convert the
		 * longer-than-one-byte header fields where
		 * needed and adjust for the data case
		 */
		switch (part_recv->type) {
		case PACKETTYPE_DATA_HDR:
			{
				part_recv->type = PACKETTYPE_DATA_BDY;
				convert_word(part_recv->packet.datapacket->
					     len);
				part_recv->expectedLen +=
				    part_recv->packet.datapacket->len;
			}
			break;

		case PACKETTYPE_CREDIT:
			{
				convert_dword(part_recv->packet.
					      creditpacket->bytecredit);
				convert_dword(part_recv->packet.
					      creditpacket->sendcredit);
			}
			break;

		case PACKETTYPE_ENABLECHANNEL:
			{
				convert_dword(part_recv->packet.
					      enablechannel->bytecredit);
				convert_dword(part_recv->packet.
					      enablechannel->sendcredit);
			}
			break;

			/* note that in the PACKETTYPE_CHANNELSIGNAL
			 * case we let the tty interface deal
			 * with the int32 contained there
			 */
		}

		LOG_TRACE("currentLen=%ld expectedLen=%ld\n",
			  part_recv->currentLen, part_recv->expectedLen);

		if (part_recv->currentLen == part_recv->expectedLen) {
			/* we have it all, pass it on */

			LOG_PACKETTYPE(part_recv->buffer);

			if (part_recv->type == PACKETTYPE_DATA_BDY) {
				ReceiveData(mux);
			} else {
				ReceiveCommand(mux);
				free_commbuff(part_recv->buffer);
			}

			/* now that we've handled this one,
			 * clear up the partial so that
			 * we can receive a new message
			 */
			memset(part_recv, 0, sizeof(PARTIAL_RECEIVE));
		} else if (part_recv->currentLen > part_recv->expectedLen) {
			printk(KERN_ERR "NETMUX ERROR: data length is over, \
				expectedLen = %ld, currentLen = %ld\n",
			     part_recv->expectedLen,
			     part_recv->currentLen);
			PANIC(netmuxPanicPkgFail2,
			      "Packet received over length");
		}
	}
}

/*
 * ReceiveCommand is called when a complete command message arrives.
 * The command will be parsed and processed.
 *
 * Params:
 * commbuff -- the data
 * mux -- the mux object
 */
void ReceiveCommand(MUX *mux)
{
	INTERFACEINFORM inform_data;
	PARTIAL_RECEIVE *part_recv;

	DEBUG("ReceiveCommand(%p)\n", mux);

	part_recv = &mux->partial_receive;
	inform_data.source = mux;

	switch (part_recv->type) {
	case PACKETTYPE_CREDIT:
		{
			AdjustCredit(client_end(COMMAND),
				     (int8) part_recv->packet.
				     creditpacket->channel,
				     part_recv->packet.creditpacket->
				     bytecredit,
				     part_recv->packet.creditpacket->
				     sendcredit, mux);
		}
		break;


	case PACKETTYPE_ENABLEMUX:
		{
			EnableMUX(part_recv->packet.enablemux->acktype,
				  mux);
		}
		break;


	case PACKETTYPE_DISABLEMUX:
		{
			DisableMUX(part_recv->packet.disablemux->acktype,
				   mux);
		}
		break;


	case PACKETTYPE_ENABLECHANNEL:
		{
			if (part_recv->packet.enablechannel->acktype ==
			    client_end(COMMAND)) {
				inform_data.inform_type =
				    INFORM_INTERFACE_ENABLECHANNEL;
				inform_data.data =
				    (void *) part_recv->packet.
				    enablechannel;

				LIBRARY_INFORM(&inform_data,
					       part_recv->packet.
					       enablechannel->
					       client_interface,
					       mux->interface_lib);
			} else {
				EnableChannel(part_recv->packet.
					      enablechannel->acktype,
					      part_recv->packet.
					      enablechannel->channel, 0, 0,
					      part_recv->packet.
					      enablechannel->
					      host_interface,
					      part_recv->packet.
					      enablechannel->
					      client_interface, 0, 0,
					      part_recv->packet.
					      enablechannel->bytecredit,
					      part_recv->packet.
					      enablechannel->sendcredit,
					      mux);
			}
		}
		break;


	case PACKETTYPE_DISABLECHANNEL:
		{
			DisableChannel(part_recv->packet.disablechannel->
				       acktype,
				       part_recv->packet.disablechannel->
				       channel,
				       part_recv->packet.disablechannel->
				       host_interface,
				       part_recv->packet.disablechannel->
				       client_interface, mux);
		}
		break;


	case PACKETTYPE_QUERYINTERFACE:
		{
			QueryExternalInterface(part_recv->packet.
					       queryinterface->acktype,
					       part_recv->packet.
					       queryinterface->
					       host_interface,
					       part_recv->packet.
					       queryinterface->result,
					       (sint8 *) part_recv->packet.
					       queryinterface->name, mux);
		}
		break;

	case PACKETTYPE_CHANNELSIGNAL:
		{
			ChannelSignal(part_recv->packet.channelsignal->
				      acktype,
				      part_recv->packet.channelsignal->
				      channel,
				      part_recv->packet.channelsignal->
				      signal, mux);
		}
		break;
	}

	return;
}


/*
 * ReceiveData is called to process data being received from the remote NetMUX.
 * The data has already been pieced together if it was fragmented by the link
 * driver so now it just needs  delivered to the right interface.
 *
 * Params:
 * commbuff -- the data
 * mux -- the mux object
 */
void ReceiveData(MUX *mux)
{
	COMMBUFF *databuffer;
	CHANNEL *channel;
	int8 chanlNum;

	DEBUG("ReceiveData(%p)\n", mux);

	enter_write_criticalsection(&mux->lock);

	databuffer = mux->partial_receive.buffer;
	chanlNum = mux->partial_receive.packet.datapacket->channel;
	channel = mux->channels[chanlNum];

	if (!channel) {
		/* we've just received data for a closed channel
		 * let's log it
		 */
		DEBUG("NETMUX ERROR: DISCARDING DATA RECEIVED FOR  \
			CLOSED CHANNEL NUMBER %d\n",
			chanlNum);
		free_commbuff(databuffer);
		exit_write_criticalsection(&mux->lock);
		return;
	}

	/* tag before stripping the header so we can account for */
	/* receive buffers freed when this one is released       */
	tag_commbuff(databuffer, chanlNum, (void *) mux,
		     &CommBuffReleased);

	/* get rid of the header before it goes on the channel queue */
	commbuff_remove_front(databuffer, sizeof(DATA_PACKET_HDR));

	queue_commbuff(databuffer, &channel->receive_queue);

	exit_write_criticalsection(&mux->lock);
}

/*
 * CreateMUX creates a mux object that can then be used for
 * communication via assigning interfaces and a linkdriver to it.
 *
 * Params:
 * maxchannels -- the max number of channels for the mux to support
 * loc_max_rcv_siz -- largest message the local linkdriver will receive
 * rem_max_rcv_siz -- largest message the remote linkdriver can receive
 * sendfunction -- the function to send data with
 * interface_lib -- the interface library to assign the mux to
 * mux -- a pointer to where to save the mux object
 */
int32 CreateMUX(int32 maxchannels,
		int32 max_send_buffers,
		int32 loc_max_rcv_siz,
		int32 rem_max_rcv_siz,
		int32(*sendfunction) (void *),
		MUXINTERFACE_LIBRARY *interface_lib, MUX **mux)
{
	CHANNEL **newchannels;
	CHANNELCREDIT *newcredits;
	MUX *newmux;
	int32 size;

	DEBUG("CreateMUX(%lu, %lu, %lu, %lu, %p, %p, %p)\n",
	      maxchannels,
	      max_send_buffers,
	      loc_max_rcv_siz,
	      rem_max_rcv_siz, sendfunction, interface_lib, mux);

	if (!maxchannels || !sendfunction || !interface_lib || !mux)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	newmux = alloc_mem(sizeof(MUX));

	size = sizeof(CHANNEL *) * maxchannels;
	newchannels = alloc_mem(size);

	memset(newchannels, 0, size);

	size = sizeof(CHANNELCREDIT) * maxchannels;
	newcredits = alloc_mem(size);

	memset(newcredits, 0, size);
	memset(newmux, 0, sizeof(MUX));

	newmux->Send = sendfunction;
	newmux->channels = newchannels;
	newmux->channelcredit = newcredits;
	newmux->maxchannels = maxchannels;
	newmux->interface_lib = interface_lib;
	newmux->status = STATUS_DEFAULT;
	newmux->local_rcv_buffer_size = loc_max_rcv_siz;
	newmux->remote_rcv_buffer_size = rem_max_rcv_siz;
	newmux->send_buffers_available = max_send_buffers;

	initialize_commbuff_queue(&newmux->send_queue);
	initialize_commbuff_queue(&newmux->receive_queue);

	initialize_task(&newmux->send_task, &ProcessSendQueues);
	initialize_task(&newmux->receive_task, &ProcessReceiveQueues);
	initialize_task(&newmux->shutdown_task, &ShutdownMUX);

	initialize_criticalsection_lock(&newmux->lock);

	disable_task(&newmux->send_task);

	*mux = newmux;

	return DEBUGERROR(ERROR_NONE);
}

/*
 * DestroyMUX will destroy a mux object and any allocated resources.
 * This call assumes the mux object is not currently in use.
 *
 * Params:
 * mux -- the mux object
 */
int32 DestroyMUX(MUX *mux)
{
	DEBUG("DestroyMUX(%p)\n", mux);

	if (!mux)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	if (mux->status != STATUS_DEFAULT)
		return DEBUGERROR(ERROR_OPERATIONFAILED);

	destroy_task(&mux->send_task);
	destroy_task(&mux->receive_task);

	destroy_commbuff_queue(&mux->send_queue);
	destroy_commbuff_queue(&mux->receive_queue);

	destroy_criticalsection_lock(&mux->lock);

	free_mem(mux->channels);
	free_mem(mux->channelcredit);
	free_mem(mux);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * AdjustCredit is called to send a credit message to the client
 * mux.
 *
 * Params:
 * acktype -- the direction of the packet
 * channel -- where the packet belongs
 * space -- the number of bytes of credit to deliver
 * mux -- the mux object
 */
int32 AdjustCredit(int32 acktype, int32 channel, int32 bytecredit,
		   int32 sendcredit, MUX *mux)
{
	CHANNELCREDIT *credit;

	DEBUG("AdjustCredit(%lu, %lu, %lu, %lu, %p)\n", acktype, channel,
	      bytecredit, sendcredit, mux);

	switch (acktype) {
	case host_end(COMMAND):
		{
			TransmitCredit(client_end(COMMAND), (int8) channel,
				       bytecredit, sendcredit, mux);
		}
		break;

	case client_end(COMMAND):
		{
			enter_write_criticalsection(&mux->lock);

			credit = &mux->channelcredit[channel];

			mux->send_buffers_available += sendcredit;

			credit->client_byte_credit += bytecredit;
			credit->client_send_credit += sendcredit;

			task_schedule(&mux->send_task);

			exit_write_criticalsection(&mux->lock);
		}
		break;

	default:
		{
			return DEBUGERROR(ERROR_OPERATIONFAILED);
		}
		break;
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * EnableMUX delivers an enable message to the client mux.
 *
 * Params:
 * acktype -- the direction of the packet
 * mux -- the mux object
 */
int32 EnableMUX(int32 acktype, MUX *mux)
{
	ENABLEMUX_PACKET enablemux;
	INTERFACEINFORM inform_data;

	DEBUG("EnableMUX(%lu, %p)\n", acktype, mux);

	enablemux.acktype = (int8) acktype;
	enablemux.type = PACKETTYPE_ENABLEMUX;

	inform_data.source = mux;
	inform_data.inform_type = INFORM_INTERFACE_ENABLEMUX;
	inform_data.data = (void *) &enablemux;

	switch (acktype) {
	case host_end(COMMAND):
		{
			if (!(mux->status & STATUS_DEFAULT))
				return DEBUGERROR(ERROR_OPERATIONFAILED);

			mux->status ^= (STATUS_DEFAULT | STATUS_ENABLE);

			TransmitEnableMUX(client_end(COMMAND), mux);
			enable_task(&mux->send_task);
		}
		break;

	case host_end(SUCCESS):
		{
			if (!(mux->status & STATUS_ENABLE))
				return DEBUGERROR(ERROR_OPERATIONFAILED);

			mux->status ^= (STATUS_ENABLE | STATUS_ACTIVE);

			BroadcastLibraryInform(&inform_data,
					       mux->interface_lib);
		}
		break;

	case host_end(FAILURE):
		{
			if (!(mux->status & STATUS_ENABLE))
				return DEBUGERROR(ERROR_OPERATIONFAILED);

			mux->status ^= (STATUS_ENABLE | STATUS_DEFAULT);

			BroadcastLibraryInform(&inform_data,
					       mux->interface_lib);
		}
		break;

	case client_end(COMMAND):
		{
			if (!(mux->status & STATUS_DEFAULT))
				return DEBUGERROR(ERROR_OPERATIONFAILED);

			mux->status ^= (STATUS_DEFAULT | STATUS_ACTIVE);
			enablemux.acktype = (int8) client_end(SUCCESS);

			TransmitEnableMUX(host_end(SUCCESS), mux);
			enable_task(&mux->send_task);

			BroadcastLibraryInform(&inform_data,
					       mux->interface_lib);
		}
		break;

	default:
		return DEBUGERROR(ERROR_INVALIDPARAMETER);
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * DisableMUX will send a disable mux packet to the client NetMUX.
 *
 * Params:
 * acktype -- the direction of the disable
 * mux -- the mux object
 */
int32 DisableMUX(int32 acktype, MUX *mux)
{
	DISABLEMUX_PACKET disablemux;
	INTERFACEINFORM informdata;
	CHANNEL **channels;
	int32 index;

	DEBUG("DisableMUX(%lu, %p)\n", acktype, mux);

	disablemux.acktype = (int8) acktype;
	disablemux.type = PACKETTYPE_DISABLEMUX;

	informdata.source = mux;
	informdata.inform_type = INFORM_INTERFACE_DISABLEMUX;
	informdata.data = (void *) &disablemux;

	switch (acktype) {
	case host_end(COMMAND):
		{
			enter_write_criticalsection(&mux->lock);

			if (!(mux->status & STATUS_ACTIVE)) {
				exit_write_criticalsection(&mux->lock);

				return DEBUGERROR(ERROR_OPERATIONFAILED);
			}

			TransmitDisableMUX(client_end(COMMAND), mux);

			mux->status ^= (STATUS_ACTIVE | STATUS_DEFAULT);
			channels = mux->channels;

			for (index = 0; index < mux->maxchannels; index++)
				CloseChannel(index, mux);

			exit_write_criticalsection(&mux->lock);

			BroadcastLibraryInform(&informdata,
					       mux->interface_lib);
		}
		break;

	case client_end(COMMAND):
		{
			enter_write_criticalsection(&mux->lock);

			if (!(mux->status & STATUS_ACTIVE)) {
				exit_write_criticalsection(&mux->lock);

				return DEBUGERROR(ERROR_OPERATIONFAILED);
			}

			mux->status ^= (STATUS_ACTIVE | STATUS_DEFAULT);
			mux->total_queued_amount = 0;
			channels = mux->channels;

			empty_commbuff_queue(&mux->send_queue);

			for (index = 0; index < mux->maxchannels; index++)
				CloseChannel(index, mux);

			exit_write_criticalsection(&mux->lock);

			BroadcastLibraryInform(&informdata,
					       mux->interface_lib);
		}
		break;

	default:
		return DEBUGERROR(ERROR_INVALIDPARAMETER);
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * EnableChannel will deliver an enable channel packet to the client NetMUX.
 *
 * Params:
 * acktype -- the direction of the packet
 * channel -- the channel to associate the data with
 * burst_size -- the burst size for the channel
 * max_data_amount -- the maximum data to be queued
 * host_interface -- the host interface
 * client_interface -- the client interface
 * host_credit -- the host's credit
 * client_credit -- the client's credit
 * mux -- the mux object
 */
int32 EnableChannel(int32 acktype,
		    int32 channel,
		    int32 burst_size,
		    int32 max_data_amount,
		    int32 host_interface,
		    int32 client_interface,
		    int32 host_byte_credit,
		    int32 host_send_credit,
		    int32 client_byte_credit,
		    int32 client_send_credit, MUX *mux)
{
	ENABLECHANNEL_PACKET enablechp;
	INTERFACEINFORM inform_data;
	CHANNEL **channels;
	CHANNEL *enable_channel;
	CHANNELCREDIT *credit;
	int32 result;

	DEBUG("EnableChannel(%lu, %lu, %lu, %lu, %lu, %lu, \
		%lu, %lu, %lu, %lu, %p)\n",
	     acktype, channel, burst_size, max_data_amount, host_interface,
	     client_interface, host_byte_credit, host_send_credit,
	     client_byte_credit, client_send_credit, mux);

	switch (acktype) {
	case host_end(COMMAND):
		{
			/* This is a LOCAL_OPEN event in the state machine */

			if (channel >= mux->maxchannels)
				return DEBUGERROR(ERROR_OPERATIONFAILED);

			enter_write_criticalsection(&mux->lock);

			channels = mux->channels;
			credit = &mux->channelcredit[channel];
			enable_channel = channels[channel];

			/* if we're already open, fail*/
			if (enable_channel) {
				exit_write_criticalsection(&mux->lock);

				return
				    DEBUGERROR(ERROR_OPERATIONRESTRICTED);
			}
			/* otherwise, continue*/
			else {
				enable_channel =
				    alloc_mem(sizeof(CHANNEL));

				memset(enable_channel, 0, sizeof(CHANNEL));

				result =
				    ConnectInterface(host_interface,
						     mux->interface_lib,
						     &enable_channel->
						     connected_interface);

				if (result != ERROR_NONE) {
					exit_write_criticalsection(&mux->
								   lock);

					free_mem(enable_channel);

					return DEBUGERROR(result);
				}

				initialize_commbuff_queue(&enable_channel->
							  send_queue);
				initialize_commbuff_queue(&enable_channel->
							  receive_queue);

				if (!credit->initialized) {
					credit->max_host_byte_credit =
					    host_byte_credit;
					credit->max_host_send_credit =
					    host_send_credit;
				}

				enable_channel->burst_size = burst_size;
				enable_channel->max_data_amount =
				    max_data_amount;
				enable_channel->state = OPENING;

				channels[channel] = enable_channel;

				exit_write_criticalsection(&mux->lock);

				TransmitEnableChannel((int8)
						      client_end(COMMAND),
						      (int8) channel,
						      (int8)
						      host_interface,
						      (int8)
						      client_interface,
						      host_byte_credit,
						      host_send_credit,
						      mux);
			}
		}
		break;

	case host_end(SUCCESS):
		{
			/* This is a OPEN_RESPONSE event in the state machine*/

			if (channel >= mux->maxchannels)
				return DEBUGERROR(ERROR_OPERATIONFAILED);

			enter_write_criticalsection(&mux->lock);

			channels = mux->channels;
			credit = &mux->channelcredit[channel];
			enable_channel = channels[channel];

			if (enable_channel) {
				if (!(enable_channel->state & OPENING)) {
					exit_write_criticalsection(&mux->
								   lock);

					return
					    DEBUGERROR
					    (ERROR_OPERATIONRESTRICTED);
				}

				if (!credit->initialized) {
					credit->client_byte_credit =
					    client_byte_credit;
					credit->client_send_credit =
					    client_send_credit;
					credit->replenished_byte_credit =
					    0;
					credit->replenished_send_credit =
					    0;

					credit->initialized = 1;
				}

				enable_channel->state = OPEN;
			} else {
				exit_write_criticalsection(&mux->lock);

				return DEBUGERROR(ERROR_OPERATIONFAILED);
			}

			enablechp.acktype = (int8) acktype;
			enablechp.channel = (int8) channel;
			enablechp.client_interface =
			    (int8) client_interface;
			enablechp.host_interface = (int8) host_interface;
			enablechp.bytecredit = client_byte_credit;
			enablechp.sendcredit = client_send_credit;
			enablechp.type = PACKETTYPE_ENABLECHANNEL;

			inform_data.source = mux;
			inform_data.inform_type =
			    INFORM_INTERFACE_ENABLECHANNEL;
			inform_data.data = (void *) &enablechp;

			exit_write_criticalsection(&mux->lock);

			LIBRARY_INFORM(&inform_data, host_interface,
				       mux->interface_lib);
		} break;

	case host_end(FAILURE):
		{
			/* This is a OPEN_RESPONSE event in the state machine*/

			if (channel >= mux->maxchannels)
				return DEBUGERROR(ERROR_OPERATIONFAILED);

			enter_write_criticalsection(&mux->lock);

			channels = mux->channels;
			enable_channel = channels[channel];

			if (enable_channel) {
				if (!(enable_channel->state & OPENING)) {
					exit_write_criticalsection(&mux->
								   lock);

					return
					    DEBUGERROR
					    (ERROR_OPERATIONFAILED);
				}

				enablechp.acktype = (int8) acktype;
				enablechp.channel = (int8) channel;
				enablechp.client_interface =
				    (int8) client_interface;
				enablechp.host_interface =
				    (int8) host_interface;
				enablechp.bytecredit = client_byte_credit;
				enablechp.sendcredit = client_send_credit;
				enablechp.type = PACKETTYPE_ENABLECHANNEL;

				inform_data.source = mux;
				inform_data.inform_type =
				    INFORM_INTERFACE_ENABLECHANNEL;
				inform_data.data = (void *) &enablechp;

				CloseChannel(channel, mux);

				exit_write_criticalsection(&mux->lock);

				LIBRARY_INFORM(&inform_data,
					       host_interface,
					       mux->interface_lib);
			} else {
				exit_write_criticalsection(&mux->lock);

				return DEBUGERROR(ERROR_OPERATIONFAILED);
			}
		}
		break;

	case client_end(COMMAND):
		{
			/* This is a REMOTE_OPEN event in the state machine*/

			if (channel >= mux->maxchannels)
				return DEBUGERROR(ERROR_OPERATIONFAILED);

			enter_write_criticalsection(&mux->lock);

			channels = mux->channels;
			credit = &mux->channelcredit[channel];
			enable_channel = channels[channel];

			/* if we're already open, fail*/
			if (enable_channel) {
				exit_write_criticalsection(&mux->lock);

				return
				    DEBUGERROR(ERROR_OPERATIONRESTRICTED);
			}
			/* otherwise, continue*/
			else {
				enable_channel =
				    alloc_mem(sizeof(CHANNEL));

				memset(enable_channel, 0, sizeof(CHANNEL));

				result =
				    ConnectInterface(client_interface,
						     mux->interface_lib,
						     &enable_channel->
						     connected_interface);

				if (result != ERROR_NONE) {
					exit_write_criticalsection(&mux->
								   lock);

					free_mem(enable_channel);

					return DEBUGERROR(result);
				}

				initialize_commbuff_queue(&enable_channel->
							  send_queue);
				initialize_commbuff_queue(&enable_channel->
							  receive_queue);

				if (!credit->initialized) {
					credit->max_host_byte_credit =
					    host_byte_credit;
					credit->max_host_send_credit =
					    host_send_credit;
					credit->client_byte_credit =
					    client_byte_credit;
					credit->client_send_credit =
					    client_send_credit;
					credit->replenished_byte_credit =
					    0;
					credit->replenished_send_credit =
					    0;

					credit->initialized = 1;
				}

				enable_channel->burst_size = burst_size;
				enable_channel->max_data_amount =
				    max_data_amount;
				enable_channel->state = OPEN;

				channels[channel] = enable_channel;

				exit_write_criticalsection(&mux->lock);

				TransmitEnableChannel((int8)
						      host_end(SUCCESS),
						      (int8) channel,
						      (int8)
						      host_interface,
						      (int8)
						      client_interface,
						      host_byte_credit,
						      host_send_credit,
						      mux);
			}
		}
		break;

	case client_end(FAILURE):
		{
			TransmitEnableChannel((int8) host_end(FAILURE),
					      (int8) channel,
					      (int8) host_interface,
					      (int8) client_interface,
					      host_byte_credit,
					      host_send_credit, mux);
		}
		break;
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * DisableChannel will deliver a disable channel message to a client NetMUX.
 *
 * Params:
 * acktype -- the direction of the packet
 * channel -- the channel to associate it with
 * host_interface -- the host interface
 * client_interface -- the client_interface
 * mux -- the mux object
 */
int32 DisableChannel(int32 acktype, int32 channel, int32 host_interface,
		     int32 client_interface, MUX *mux)
{
	CHANNEL **channels;
	CHANNEL *disable_channel;
	int32 result;
	DISABLECHANNEL_PACKET disablechp;
	INTERFACEINFORM informdata;


	DEBUG("DisableChannel(%lu, %lu, %lu, %lu, %p)\n", acktype, channel,
	      host_interface, client_interface, mux);

	switch (acktype) {
	case host_end(COMMAND):
		{
			/* This is a LOCAL_CLOSE event in the state machine */

			if (channel >= mux->maxchannels)
				return DEBUGERROR(ERROR_OPERATIONFAILED);

			enter_write_criticalsection(&mux->lock);

			channels = mux->channels;
			disable_channel = channels[channel];

			if (disable_channel) {
				disable_channel->client_interface =
				    (int8) client_interface;
				disable_channel->host_interface =
				    (int8) host_interface;

				if (disable_channel->state &
				    (OPENING | OPEN | SENDING |
				     REMOTE_CLOSING)) {
					result =
					    ExecuteStateTransition
					    (LOCAL_CLOSE, mux, channel);

					exit_write_criticalsection(&mux->
								   lock);

					if (result != ERROR_NONE)
						return
						    DEBUGERROR
						    (ERROR_OPERATIONFAILED);
				} else {
					exit_write_criticalsection(&mux->
								   lock);

					return
					    DEBUGERROR
					    (ERROR_OPERATIONFAILED);
				}
			} else {
				exit_write_criticalsection(&mux->lock);

				return DEBUGERROR(ERROR_OPERATIONFAILED);
			}
		}
		break;

	case host_end(SUCCESS):
		{
			/* This is a CLOSE_RESPONSE event in the state machine*/

			if (channel >= mux->maxchannels)
				return DEBUGERROR(ERROR_OPERATIONFAILED);

			enter_write_criticalsection(&mux->lock);

			channels = mux->channels;
			disable_channel = channels[channel];

			if (disable_channel) {
				disable_channel->client_interface =
				    (int8) client_interface;
				disable_channel->host_interface =
				    (int8) host_interface;

				if (disable_channel->
				    state & (LOCAL_CLOSING)) {
					result =
					    ExecuteStateTransition
					    (CLOSE_RESPONSE, mux, channel);

					if (result != ERROR_NONE) {
						exit_write_criticalsection
						    (&mux->lock);
						return
						    DEBUGERROR
						    (ERROR_OPERATIONFAILED);
					}

					disablechp.acktype =
					    (int8) host_end(SUCCESS);
					disablechp.channel =
					    (int8) channel;
					disablechp.client_interface =
					    (int8) client_interface;
					disablechp.host_interface =
					    (int8) host_interface;
					disablechp.type =
					    PACKETTYPE_DISABLECHANNEL;

					informdata.source = mux;
					informdata.data =
					    (void *) &disablechp;
					informdata.inform_type =
					    INFORM_INTERFACE_DISABLECHANNEL;

					exit_write_criticalsection(&mux->
								   lock);

					LIBRARY_INFORM(&informdata,
						       client_interface,
						       mux->interface_lib);

				} else {
					exit_write_criticalsection(&mux->
								   lock);

					return
					    DEBUGERROR
					    (ERROR_OPERATIONFAILED);
				}

			} else {
				exit_write_criticalsection(&mux->lock);

				return DEBUGERROR(ERROR_OPERATIONFAILED);
			}
		}
		break;

	case client_end(COMMAND):
		{
			/* This is a REMOTE_CLOSE event in the state machine*/

			if (channel >= mux->maxchannels)
				return DEBUGERROR(ERROR_OPERATIONFAILED);

			enter_write_criticalsection(&mux->lock);

			channels = mux->channels;
			disable_channel = channels[channel];

			if (disable_channel) {
				disable_channel->client_interface =
				    (int8) client_interface;
				disable_channel->host_interface =
				    (int8) host_interface;

				if (disable_channel->state &
				    (OPEN | SENDING | FLUSHING |
				     LOCAL_CLOSING)) {
					result =
					    ExecuteStateTransition
					    (REMOTE_CLOSE, mux, channel);

					if (result != ERROR_NONE) {
						exit_write_criticalsection
						    (&mux->lock);
						return
						    DEBUGERROR
						    (ERROR_OPERATIONFAILED);
					}

					if (disable_channel->
					    state & (FLUSHING)) {
						/* time to close the channel */
						CloseChannel(channel, mux);

						exit_write_criticalsection
						    (&mux->lock);
					} else if (disable_channel->
						   state &
						   (REMOTE_CLOSING)) {
						/* the OPEN and SENDING cases
						 * both end up here
						 */
						disablechp.acktype =
						    (int8)
						    client_end(COMMAND);
						disablechp.channel =
						    (int8) channel;
						disablechp.
						    client_interface =
						    (int8)
						    disable_channel->
						    client_interface;
						disablechp.host_interface =
						    (int8)
						    disable_channel->
						    host_interface;
						disablechp.type =
						    PACKETTYPE_DISABLECHANNEL;

						informdata.source = mux;
						informdata.data =
						    (void *) &disablechp;
						informdata.inform_type =
						INFORM_INTERFACE_DISABLECHANNEL;

						exit_write_criticalsection
						    (&mux->lock);

						LIBRARY_INFORM(&informdata,
						       disable_channel->
						       client_interface,
						       mux->
						       interface_lib);
					} else {
						/* this catches
						 * the LOCAL_CLOSING case
						 */
						exit_write_criticalsection
						    (&mux->lock);
					}
				} else {
					exit_write_criticalsection(&mux->
								   lock);
					return
					    DEBUGERROR
					    (ERROR_OPERATIONFAILED);
				}
			} else {
				/* we are closed here but
				 * we need to respond anyway
				 */
				TransmitDisableChannel((int8)
						       host_end(SUCCESS),
						       (int8) channel,
						       (int8)
						       host_interface,
						       (int8)
						       client_interface,
						       mux);

				exit_write_criticalsection(&mux->lock);
				return DEBUGERROR(ERROR_OPERATIONFAILED);
			}
		}
		break;

	default:
		return DEBUGERROR(ERROR_OPERATIONFAILED);
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * QueryExternalInterface will query an interfaces id from a client NetMUX.
 *
 * Params:
 * acktype -- the direction of the packet
 * host_interface -- the host interface
 * res -- the result of the query (not always used)
 * name -- the name of the interface to query
 * mux -- the mux object
 */
int32 QueryExternalInterface(int32 acktype, int32 host_interface,
			     int32 res, sint8 *name, MUX *mux)
{
	QUERYINTERFACE_PACKET queryinterface;
	INTERFACEINFORM inform_data;
	int32 result;
	int32 queried_interface;

	DEBUG("QueryExternalInterface(%lu, %lu, %lu, %p, %p)\n", acktype,
	      host_interface, res, name, mux);

	queried_interface = 0;

	switch (acktype) {
	case host_end(COMMAND):
		{
			TransmitQueryInterface((int8) client_end(COMMAND),
					       (int8) host_interface,
					       (int8) res, name, mux);
		}
		break;

	case host_end(SUCCESS):
	case host_end(FAILURE):
		{
			memcpy((void *) queryinterface.name, name,
			       PACKET_MAXNAME_LENGTH);

			queryinterface.acktype = (int8) acktype;
			queryinterface.result = (int8) res;
			queryinterface.host_interface =
			    (int8) host_interface;
			queryinterface.type = PACKETTYPE_QUERYINTERFACE;

			inform_data.source = mux;
			inform_data.inform_type =
			    INFORM_INTERFACE_QUERYINTERFACE;
			inform_data.data = (void *) &queryinterface;

			LIBRARY_INFORM(&inform_data, host_interface,
				       mux->interface_lib);
		} break;

	case client_end(COMMAND):
		{
			if (!name)
				return DEBUGERROR(ERROR_INVALIDPARAMETER);

			result =
			    QueryInterfaceIndex(name, mux->interface_lib,
						&queried_interface);

			if (result != ERROR_NONE)
				TransmitQueryInterface((int8)
						       host_end(FAILURE),
						       (int8)
						       host_interface,
						       (int8)
						       queried_interface,
						       name, mux);
			else
				TransmitQueryInterface((int8)
						       host_end(SUCCESS),
						       (int8)
						       host_interface,
						       (int8)
						       queried_interface,
						       name, mux);
		}
		break;
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * ChannelSignal will signal a channel on the client NetMUX.
 *
 * acktype -- the direction
 * signal -- the signal
 * channel -- where to associate the data
 * mux -- the mux
 */
int32 ChannelSignal(int8 acktype, int32 signal, int32 channel, MUX *mux)
{
	CHANNELSIGNAL_PACKET channelsignal;
	INTERFACEINFORM inform_data;
	CHANNEL *chdat;
	int32 ifindex;

	DEBUG("ChannelSignal(%u, %lu, %lu, %p)\n", acktype, signal,
	      channel, mux);

	switch (acktype) {
	case host_end(COMMAND):
		{
			TransmitChannelSignal((int8) client_end(COMMAND),
					      (int8) channel, signal, mux);
		}
		break;

	case client_end(COMMAND):
		{
			enter_read_criticalsection(&mux->lock);

			chdat = mux->channels[channel];
			if (!chdat) {
				exit_read_criticalsection(&mux->lock);

				return DEBUGERROR(ERROR_OPERATIONFAILED);
			}

			ifindex =
			    chdat->connected_interface->interface_index;

			exit_read_criticalsection(&mux->lock);

			channelsignal.type = PACKETTYPE_CHANNELSIGNAL;
			channelsignal.acktype = acktype;
			channelsignal.channel = (int8) channel;
			channelsignal.signal = signal;

			inform_data.inform_type =
			    INFORM_INTERFACE_CHANNELSIGNAL;
			inform_data.source = (void *) mux;
			inform_data.data = (void *) &channelsignal;

			LIBRARY_INFORM(&inform_data, ifindex,
				       mux->interface_lib);
		} break;
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * SendData will deliver data to a particular channel on the client NetMUX.
 *
 * IMPORTANT NOTE: the (interface) caller is responsible for guaranteeing
 * that all buffers sent to this interface contain sufficient room reserved
 * for the header to be added.  This is especially important to cosider if any
 * interface expects to be able to re-send a split buffer then it must first
 * make sure that the buffer being sent has the header reservation.  More
 * clearly, this function does NOT return split buffers with reserved header
 * space.  Note further that this caution does not apply to cbufs.
 *
 * Params:
 * channel -- the channel to deliver the data to
 * commbuff -- the buffer of data
 * split -- if a split is needed the user pointer is set to the split location
 * mux -- the mux object
 */
int32 SendData(int32 channel, COMMBUFF *commbuff, COMMBUFF **split,
	       MUX *mux)
{
	CHANNEL *send_channel;
	int32 room;
	int32 buffer_length;

	DEBUG("SendData(%lu, %p, %p, %p)\n", channel, commbuff, split,
	      mux);

	if (split)
		*split = 0;

	if ((commbuff == 0)
	    || (buffer_length = commbuff_length(commbuff)) == 0)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	enter_write_criticalsection(&mux->lock);

	send_channel = mux->channels[channel];

	if (!send_channel || !(send_channel->state & (OPEN | SENDING))) {
		/* we're in a state where we shouldn't send data */
		exit_write_criticalsection(&mux->lock);
		return DEBUGERROR(ERROR_OPERATIONFAILED);
	}

	/* calculate the room left on this channel's send queue */
	room =
	    send_channel->max_data_amount - send_channel->qed_data_amount;

	/* regardless of how much room there is we'll only */
	/* take up to the burst size in a single message   */
	if (room > send_channel->burst_size)
		room = send_channel->burst_size;

	/* if the message is bigger than that... */
	if (buffer_length > room) {
		/* if we can't or shouldn't split the message then */
		/* the caller will have to try again later         */
		if ((!split) || (room == 0)) {
			exit_write_criticalsection(&mux->lock);
			return DEBUGERROR(ERROR_OPERATIONRESTRICTED);
		}

		*split = commbuff_split(commbuff, room);

		buffer_length = room;
	}

	/* Acquire NM_send wakelock */
	DEBUG("Acquire netmux_send_wakelock\n");
	wake_lock(&netmux_send_wakelock);

	/* update the queue counters, note that the */
	/* header is accounted for in ApplyDataHdr  */
	send_channel->qed_data_amount += buffer_length;
	send_channel->qed_totl_amount += buffer_length;
	mux->total_queued_amount += buffer_length;

	/* add a header to the commbuff */
	ApplyDataHdr((int8) channel, &commbuff, mux);

	queue_commbuff(commbuff, &send_channel->send_queue);

	/* the channel has data to send, change its state */
	(void) ExecuteStateTransition(LOCAL_DATA, mux, channel);

	exit_write_criticalsection(&mux->lock);

	enable_task(&mux->send_task);
	task_schedule(&mux->send_task);

	if (split && (*split))
		return DEBUGERROR(ERROR_INCOMPLETE);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * SendDataAvailable returns how much data is available to be sent.
 *
 * Params:
 * channel -- the channel we are interested in
 * mux -- the mux object
 */
int32 SendDataAvailable(int32 channel, MUX *mux)
{
	CHANNEL *send_channel;
	int32 available;

	DEBUG("SendDataAvailable(%lu, %p)\n", channel, mux);

	available = 0;

	enter_read_criticalsection(&mux->lock);

	send_channel = mux->channels[channel];
	if (!send_channel) {
		exit_read_criticalsection(&mux->lock);

		return 0;
	}

	available =
	    send_channel->max_data_amount - send_channel->qed_data_amount;

	exit_read_criticalsection(&mux->lock);

	return available;
}

/*
 * ReadData is called only by the direct interface to get a buffer from
 * the queue during a read( ).  If the number of bytes to be read is less
 * than the number of bytes in the first buffer then that buffer is split.
 * This function trusts that it will never be called with a 0 count.
 *
 * Params:
 * channel -- the channel to read
 * mux -- the mux object
 * commbuff -- the location the data is to be stored at
 * count -- the amount to read
 */
int32 ReadData(int32 channel, MUX *mux, COMMBUFF **commbuff, int32 count)
{
	COMMBUFF *split;
	CHANNEL *read_channel;
	int32 length;

	DEBUG("ReadData(%lu, %p, %p, %lu)\n", channel, mux, commbuff,
	      count);

	enter_read_criticalsection(&mux->lock);

	read_channel = mux->channels[channel];
	if (!read_channel) {
		exit_read_criticalsection(&mux->lock);

		return DEBUGERROR(ERROR_NONE);
	}

	if (!(queue_length(&read_channel->receive_queue))) {
		*commbuff = 0;
		exit_read_criticalsection(&mux->lock);

		return DEBUGERROR(ERROR_NONE);
	}

	*commbuff = dequeue_commbuff(&read_channel->receive_queue);

	length = commbuff_length(*commbuff);
	if (length > count) {
		split = commbuff_split(*commbuff, count);

		exit_read_criticalsection(&mux->lock);
		enter_write_criticalsection(&mux->lock);

		read_channel = mux->channels[channel];
		if (read_channel)
			queuefront_commbuff(split,
					    &read_channel->receive_queue);

		check_all_receive_queues_emptiness(mux);

		exit_write_criticalsection(&mux->lock);

		return DEBUGERROR(ERROR_NONE);
	}

	check_all_receive_queues_emptiness(mux);

	exit_read_criticalsection(&mux->lock);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * ReadDataAvailable returns the number of bytes available to be read.
 *
 * channel -- the channel we are interested in
 * mux -- the mux object
 */
int32 ReadDataAvailable(int32 channel, MUX *mux)
{
	CHANNEL *read_channel;
	int32 available;

	DEBUG("ReadDataAvailable(%lu, %p)\n", channel, mux);

	available = 0;

	enter_read_criticalsection(&mux->lock);

	read_channel = mux->channels[channel];
	if (!read_channel) {
		exit_read_criticalsection(&mux->lock);

		return 0;
	}

	available = queue_length(&read_channel->receive_queue);

	exit_read_criticalsection(&mux->lock);

	return available;
}

/*
 * RunReceive will run the mux's receive task.
 *
 * Params:
 * mux -- the mux object
 */
int32 RunReceive(MUX *mux)
{
	DEBUG("RunReceive(%p)\n", mux);

	task_schedule(&mux->receive_task);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * RunSend runs the mux's send task.
 *
 * Params:
 * mux -- the mux object
 */
int32 RunSend(MUX *mux)
{
	DEBUG("RunSend(%p)\n", mux);

	task_schedule(&mux->send_task);

	return DEBUGERROR(ERROR_NONE);
}


/* MUST be called under critical section */
void check_all_receive_queues_emptiness(MUX *mux)
{
	int32 channel;
	CHANNEL *recv_channel;
	COMMBUFFQUEUE *queue;
	CHANNEL **channels;

	DEBUG("check_all_receive_queues_emptiness\n");

	/* First check netmux receive queue */
	queue = &mux->receive_queue;
	if (queue_length(queue) != 0)
		return;

	DEBUG("mux receive_queue is empty\n");

	channels = mux->channels;

	/* then process the application receive queues */
	for (channel = 0; channel < mux->maxchannels; channel++) {
		recv_channel = channels[channel];
		if (recv_channel) {
			queue = &recv_channel->receive_queue;
			if (queue_length(queue) != 0) {
				DEBUG
				("Channel %d receive_queue is not empty\n");
				return;	/* exit wo doing anything */
			}
		}
	}

	/* Release NM_receive wakelock */
	DEBUG("Releasing netmux_receive_wakelock\n");
	wake_unlock(&netmux_receive_wakelock);
}

/* MUST be called under critical section */
void check_all_send_queues_emptiness(MUX *mux)
{
	int32 channel;
	CHANNEL *send_channel;
	COMMBUFFQUEUE *queue;
	CHANNEL **channels;

	DEBUG("check_all_send_queues_emptiness\n");

	/* First check netmux send queue */
	queue = &mux->send_queue;
	if (queue_length(queue) != 0)
		return;

	DEBUG("mux send_queue is empty\n");

	channels = mux->channels;

	/* then process the application receive queues */
	for (channel = 0; channel < mux->maxchannels; channel++) {
		send_channel = channels[channel];
		if (send_channel) {
			queue = &send_channel->send_queue;
			if (queue_length(queue) != 0) {
				DEBUG
				    ("Channel %d send_queue is not empty\n");
				return;	/* exit wo doing anything */
			}
		}
	}

	/* Release NM_send wakelock */
	DEBUG("Releasing netmux_send_wakelock\n");
	wake_unlock(&netmux_send_wakelock);
}

