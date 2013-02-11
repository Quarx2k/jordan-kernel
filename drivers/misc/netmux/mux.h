/******************************************************************************
 * NetMUX mux.h                                                               *
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
 *   2007/12/05  Motorola    Change code as kernel upgrade                    *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* mux.h defines all the data types and constant values to be used by the     */
/* mux.c.  The mux is responsible for managing all data that goes across the  */
/* linkdriver and between interfaces.                                         */

#ifndef _NETMUX_MUX_H_
#define _NETMUX_MUX_H_


#include "shared/ldprotocol.h"

#include "types.h"
#include "errorcodes.h"
#include "utility.h"
#include "packet.h"
#include "interface.h"


/*
 * Define different status's that apply to the MUX
 */
#define STATUS_DEFAULT 0x00000001
#define STATUS_ENABLE  0x00000002
#define STATUS_DISABLE 0x00000004
#define STATUS_ACTIVE  0x00000008

/*
 * Define different states that channels can be in
 */
#define CLOSED         0x00000001
#define OPENING        0x00000002
#define OPEN           0x00000004
#define SENDING        0x00000008
#define FLUSHING       0x00000010
#define LOCAL_CLOSING  0x00000020
#define REMOTE_CLOSING 0x00000040

/*
 * Define different events that can occur on channels
 */
#define LOCAL_OPEN     0x00000001
#define OPEN_RESPONSE  0x00000002
#define REMOTE_OPEN    0x00000004
#define LOCAL_CLOSE    0x00000008
#define CLOSE_RESPONSE 0x00000010
#define REMOTE_CLOSE   0x00000020
#define LOCAL_DATA     0x00000040
#define SEND_COMPLETE  0x00000080
#define REMOTE_DATA    0x00000100

/*
 * Define control packet direction codes
 */
#define HOST   0x00000040
#define CLIENT 0x00000080

#define COMMAND 0x00000001
#define SUCCESS 0x00000002
#define FAILURE 0x00000004

#define host_end(type)   (HOST|type)
#define client_end(type) (CLIENT|type)
#define strip_end(type)  (type&(~(HOST|CLIENT)))

/*
 * Define thresholds for sending credit
 */
#define MUX_BYTECREDIT_SEND_LIMIT_DIVISOR 4
#define MUX_SENDCREDIT_SEND_LIMIT_DIVISOR 2


/*
 * Define inform types that can be delivered to MUX interfaces
 */
#define INFORM_INTERFACE_ENABLEMUX      1
#define INFORM_INTERFACE_DISABLEMUX     2
#define INFORM_INTERFACE_ENABLECHANNEL  3
#define INFORM_INTERFACE_DISABLECHANNEL 4
#define INFORM_INTERFACE_QUERYINTERFACE 5
#define INFORM_INTERFACE_CONFIGPACKET   6
#define INFORM_INTERFACE_CHANNELSIGNAL  7
#define INFORM_INTERFACE_DATA           8
#define INFORM_INTERFACE_PREPSEND       9


/*
 * PARTIAL_RECEIVE stores information about the packet currently being
 * received from the link.  The description of this structure is:
 *
 * type: the type of packet being received
 * hdr: a pointer to the header within the buffer
 * expectedLen: the number of bytes we are expecting to receive including
 *              both the packet header length and the packet data length
 * currentLen: the number of bytes received so far
 * packet: a pointer to the commbuff cast available in a variety of types
 */
typedef struct PARTIAL_RECEIVE {
	COMMBUFF *buffer;
	int32 expectedLen;
	int32 currentLen;

	union {
		int8 *payload;
		DATA_PACKET_HDR *datapacket;
		CREDIT_PACKET *creditpacket;
		ENABLEMUX_PACKET *enablemux;
		DISABLEMUX_PACKET *disablemux;
		ENABLECHANNEL_PACKET *enablechannel;
		DISABLECHANNEL_PACKET *disablechannel;
		QUERYINTERFACE_PACKET *queryinterface;
		CHANNELSIGNAL_PACKET *channelsignal;
	} packet;

	int8 type;

} PARTIAL_RECEIVE;

/*
 * CHANNEL defines a MUX channel. The description is as follows
 *
 * connected_interface points to a registered interface
 * 	associated with the channel
 * send_queue holds the commbuffs to be sent on this channel
 * receive_queue holds the commbuffs to be received on this channel
 * burst_size is the maximum number of bytes allowed to be sent at a time
 * max_data_amount is the maximum amount of data that
 * 	can be stored in the send queue
 * qed_data_amount is the current amount of data stored in the send queue
 * qed_totl_amount is the current amount of data and header stored in the queue
 * state represents the state the channel is in
 * client_interface is the destination interface on
 * 	the client side for this channel
 * host_interface is the source interface on the host side for this channel
 */
typedef struct CHANNEL {
	MUXINTERFACE *connected_interface;

	COMMBUFFQUEUE send_queue;
	COMMBUFFQUEUE receive_queue;

	int32 burst_size;
	int32 max_data_amount;
	int32 qed_data_amount;
	int32 qed_totl_amount;
	int32 state;
	int32 client_interface;
	int32 host_interface;
} CHANNEL;

/*
 * CHANNELCREDIT defines credit values for channels.
 * The description is as follows
 *
 * initialized indicates whether the channel has had its credit values setup
 * max_host_byte_credit is the maximum amount of data that can ever be received
 * max_host_send_credit is the maximum amount of sends a channel can perform
 * client_byte_credit is the current amount of data that can be sent
 * client_send_credit is the number of sends a channel can perform
 * replenished_byte_credit is the number of bytes of credit
 * 	to eventually deliver
 * replenished_send_credit is the amount of send credit to eventually deliver
 */
typedef struct CHANNELCREDIT {
	int32 initialized;

	int32 max_host_byte_credit;
	int32 max_host_send_credit;
	int32 client_byte_credit;
	int32 client_send_credit;
	int32 replenished_byte_credit;
	int32 replenished_send_credit;
} CHANNELCREDIT;

/*
 * MUX defines an object to manage a set of channels. A brief description
 * follows.
 *
 * channels is the list of channels that can be used
 * maxchannels is the maximum amount of channels available on the MUX
 * status states the condition the MUX is in
 * total_queued_amount is the current amount of data queued in the mux
 * local_rcv_buffer_size is set by the linkdriver to record
 * 	the local receive buffer size
 * remote_rcv_buffer_size is set by the linkdriver to record
 * 	the remote receive buffer size
 * send_buffers_available is the current number of sends the mux can perform
 * interface_lib is a list of registered interfaces
 * send_queue is used to store control packets to be sent
 * receive_queue is used to store control packets to be sent
 * Send is a pointer to a function that will deliver data
 * send_task defines a task to be enabled when processing send queues
 * receive_task defines a task to be enabled when processing received data
 * shutdown_task defines a task to be used when the linkdriver
 * 	shuts down the mux
 * partial_receive defines the MUX receiving state
 * lock synchronizes the mux
 */
typedef struct MUX {
	CHANNEL **channels;
	CHANNELCREDIT *channelcredit;

	int32 maxchannels;
	int32 status;
	int32 total_queued_amount;

	int32 local_rcv_buffer_size;
	int32 remote_rcv_buffer_size;
	int32 send_buffers_available;

	MUXINTERFACE_LIBRARY *interface_lib;

	COMMBUFFQUEUE send_queue;
	COMMBUFFQUEUE receive_queue;

	 int32(*Send) (void *);

	TASKDATA send_task;
	TASKDATA receive_task;
	TASKDATA shutdown_task;

	PARTIAL_RECEIVE partial_receive;

	CRITICALSECTION lock;
} MUX;


/*
 * Decalre functions defined in mux.c
 */

int32 ExecuteStateTransition(int32, MUX *, int32);
int32 InformMUX(void *, void *);

void CommBuffReleased(int32, int32, void *);

void ProcessSendQueues(struct work_struct *);
void ProcessReceiveQueues(struct work_struct *);
void ShutdownMUX(struct work_struct *);


int32 ReceiveFromLink(void *, void *);
void ReceivePartial(COMMBUFF *, MUX *);
void ReceiveData(MUX *);
void ReceiveCommand(MUX *);

int32 CreateMUX(int32,
		int32,
		int32,
		int32,
		int32(*sendfunction) (void *),
		MUXINTERFACE_LIBRARY *, MUX **);
int32 DestroyMUX(MUX *);

int32 AdjustCredit(int32, int32, int32, int32, MUX *);

int32 EnableMUX(int32, MUX *);
int32 DisableMUX(int32, MUX *);

int32 EnableChannel(int32,
		    int32,
		    int32,
		    int32,
		    int32, int32, int32, int32, int32, int32, MUX *);


int32 DisableChannel(int32, int32, int32, int32, MUX *);

int32 QueryExternalInterface(int32, int32, int32, sint8 *, MUX *);

int32 ChannelSignal(int8, int32, int32, MUX *);

int32 SendData(int32, COMMBUFF *, COMMBUFF **, MUX *);
int32 SendDataAvailable(int32, MUX *);
int32 ReadData(int32, MUX *, COMMBUFF **, int32);
int32 ReadDataAvailable(int32, MUX *);
int32 RunReceive(MUX *);
int32 RunSend(MUX *);

#endif
