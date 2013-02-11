/******************************************************************************
 * NetMUX protocol.c                                                          *
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
 *   2007/01/22  Motorola    Fixed MMU exception in ApplyDataHdr              *
 *   2007/04/11  Motorola    Reduce memcpys in ApplyDataHdr                   *
 *   2007/05/01  Motorola    Change codes to ensure "shared" netmux           *
 *                           code is identical between AP and BP.             *
 *   2007/08/16  Motorola    Add supprot for larger bytecredit                *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* protocol.c contains functions used to transmit NetMUX protocol messages    */

#include "protocol.h"
#include "debug.h"


/*
 * ApplyDataHdr adds a data header to a data buffer
 *
 * Params:
 * channel is the channel to which the data belongs
 * commbuff is the buffer with the data to be sent
 * mux is the MUX object the data is to be transmitted through
 */
void ApplyDataHdr(int8 channel, COMMBUFF **commbuff, MUX *mux)
{
	CHANNEL *chanlPtr;
	DATA_PACKET_HDR packetdatahdr;
	int16 length;
	COMMBUFF *cb = *commbuff;

	DEBUG("ApplyDataHdr(%lu, %p, %p)\n", (int32) channel, commbuff,
	      mux);

	length = (int16) commbuff_length(cb);
	convert_word(length);

	chanlPtr = mux->channels[channel];

	packetdatahdr.type = PACKETTYPE_DATA_HDR;
	packetdatahdr.channel = channel;
	packetdatahdr.len = length;

	commbuff_add_header(cb, &packetdatahdr, sizeof(DATA_PACKET_HDR));
	*commbuff = cb;

	chanlPtr->qed_totl_amount += sizeof(DATA_PACKET_HDR);
	mux->total_queued_amount += sizeof(DATA_PACKET_HDR);
}

/*
 * TransmitCredit assembles a control credit packet and
 * initializes the transmit procedure
 *
 * Params:
 * acktype specifies host or client as well as failure or success
 * channel is the channel the credit should be applied to
 * bytecredit is the amount the byte credit should be increased
 * sendcredit is the amount the send credit should be increased
 * mux is the MUX object the data is to be transmitted through
 */
int32 TransmitCredit(int8 acktype, int8 channel, int32 bytecredit,
		     int32 sendcredit, MUX *mux)
{
	COMMBUFF *commbuff;
	CREDIT_PACKET *packetdata;

	DEBUG("TransmitCredit(%lu, %lu, %lu, 0x%lu, %p)\n",
	      (int32) acktype,
	      (int32) channel,
	      (int32) bytecredit, (int32) sendcredit, mux);

	commbuff = int_alloc_commbuff(sizeof(CREDIT_PACKET));

	convert_dword(bytecredit);
	convert_dword(sendcredit);

	packetdata = (CREDIT_PACKET *) commbuff_data(commbuff);
	packetdata->type = PACKETTYPE_CREDIT;

	commbuff_copyin_byte(commbuff, offsetof(CREDIT_PACKET, acktype),
			     acktype);
	commbuff_copyin_byte(commbuff, offsetof(CREDIT_PACKET, channel),
			     channel);
	commbuff_copyin_dword(commbuff,
			      offsetof(CREDIT_PACKET, bytecredit),
			      bytecredit);
	commbuff_copyin_dword(commbuff,
			      offsetof(CREDIT_PACKET, sendcredit),
			      sendcredit);

	queue_commbuff(commbuff, &mux->send_queue);

	mux->total_queued_amount += sizeof(CREDIT_PACKET);

	task_schedule(&mux->send_task);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * TransmitEnableMUX assembles a control enable mux packet and
 * initializes the transmit procedure
 *
 * Params:
 * acktype specifies host or client as well as failure or success
 * mux is the MUX object the data is to be transmitted through
 */
int32 TransmitEnableMUX(int8 acktype, MUX *mux)
{
	COMMBUFF *commbuff;
	ENABLEMUX_PACKET *packetdata;

	DEBUG("TransmitEnableMUX(%lu, %p)\n", (int32) acktype, mux);

	commbuff = int_alloc_commbuff(sizeof(ENABLEMUX_PACKET));

	packetdata = (ENABLEMUX_PACKET *) commbuff_data(commbuff);
	packetdata->type = PACKETTYPE_ENABLEMUX;

	commbuff_copyin_byte(commbuff, offsetof(ENABLEMUX_PACKET, acktype),
			     acktype);

	queue_commbuff(commbuff, &mux->send_queue);

	mux->total_queued_amount += sizeof(ENABLEMUX_PACKET);

	task_schedule(&mux->send_task);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * TransmitDisableMUX assembles a control disable mux packet and
 * initializes the transmit procedure
 *
 * Params:
 * acktype specifies host or client as well as failure or success
 * mux is the MUX object the data is to be transmitted through
 */
int32 TransmitDisableMUX(int8 acktype, MUX *mux)
{
	COMMBUFF *commbuff;
	DISABLEMUX_PACKET *packetdata;

	DEBUG("TransmitDisableMUX(%lu, %p)\n", (int32) acktype, mux);

	commbuff = int_alloc_commbuff(sizeof(DISABLEMUX_PACKET));

	packetdata = (DISABLEMUX_PACKET *) commbuff_data(commbuff);
	packetdata->type = PACKETTYPE_DISABLEMUX;

	commbuff_copyin_byte(commbuff,
			     offsetof(DISABLEMUX_PACKET, acktype),
			     acktype);

	queue_commbuff(commbuff, &mux->send_queue);

	mux->total_queued_amount += sizeof(DISABLEMUX_PACKET);

	task_schedule(&mux->send_task);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * TransmitEnableChannel assembles a control enable channel packet and
 * initializes the transmit procedure
 *
 * Params:
 * acktype specifies host or client as well as failure or success
 * channel is the channel to be enabled
 * host_interface is the interface which belongs to the host and
 * 	is requesting the enable
 * client_interface is the interface which belongs to the client and
 * 	is responding to the enable
 * bytecredit is the size of the receive buffer on the senders side
 * sendcredit is the number of sends available to the senders side
 * mux is the MUX object the data is to be transmitted through
 */
int32 TransmitEnableChannel(int8 acktype,
			    int8 channel,
			    int8 host_interface,
			    int8 client_interface,
			    int32 bytecredit, int32 sendcredit, MUX *mux)
{
	COMMBUFF *commbuff;
	ENABLECHANNEL_PACKET *packetdata;

	DEBUG("TransmitEnableChannel(%lu, %lu, %lu, %lu, %lu, %lu, %p)\n",
	      (int32) acktype,
	      (int32) channel,
	      (int32) host_interface,
	      (int32) client_interface,
	      (int32) bytecredit, (int32) sendcredit, mux);

	commbuff = int_alloc_commbuff(sizeof(ENABLECHANNEL_PACKET));

	convert_dword(bytecredit);
	convert_dword(sendcredit);

	packetdata = (ENABLECHANNEL_PACKET *) commbuff_data(commbuff);
	packetdata->type = PACKETTYPE_ENABLECHANNEL;

	commbuff_copyin_byte(commbuff,
			     offsetof(ENABLECHANNEL_PACKET, acktype),
			     acktype);
	commbuff_copyin_byte(commbuff,
			     offsetof(ENABLECHANNEL_PACKET, channel),
			     channel);
	commbuff_copyin_byte(commbuff,
			     offsetof(ENABLECHANNEL_PACKET,
				      host_interface), host_interface);
	commbuff_copyin_byte(commbuff,
			     offsetof(ENABLECHANNEL_PACKET,
				      client_interface), client_interface);
	commbuff_copyin_dword(commbuff,
			      offsetof(ENABLECHANNEL_PACKET, bytecredit),
			      bytecredit);
	commbuff_copyin_dword(commbuff,
			      offsetof(ENABLECHANNEL_PACKET, sendcredit),
			      sendcredit);

	queue_commbuff(commbuff, &mux->send_queue);

	mux->total_queued_amount += sizeof(ENABLECHANNEL_PACKET);

	task_schedule(&mux->send_task);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * TransmitDisableChannel assembles a control disable channel packet and
 * initializes the transmit procedure
 *
 * Params:
 * acktype specifies host or client as well as failure or success
 * channel is the channel to be disabled
 * host_interface is the interface which belongs to the host and
 * 	is requesting the disable
 * client_interface is the interface which belongs to the client and
 * is responding to the disable
 * mux is the MUX object the data is to be transmitted through
 */
int32 TransmitDisableChannel(int8 acktype, int8 channel,
			     int8 host_interface, int8 client_interface,
			     MUX *mux)
{
	COMMBUFF *commbuff;
	DISABLECHANNEL_PACKET *packetdata;

	DEBUG("TransmitDisableChannel(%lu, %lu, %lu, %lu, %p)\n",
	      (int32) acktype, (int32) channel, (int32) host_interface,
	      (int32) client_interface, mux);

	commbuff = int_alloc_commbuff(sizeof(DISABLECHANNEL_PACKET));

	packetdata = (DISABLECHANNEL_PACKET *) commbuff_data(commbuff);
	packetdata->type = PACKETTYPE_DISABLECHANNEL;

	commbuff_copyin_byte(commbuff,
			     offsetof(DISABLECHANNEL_PACKET, acktype),
			     acktype);
	commbuff_copyin_byte(commbuff,
			     offsetof(DISABLECHANNEL_PACKET, channel),
			     channel);
	commbuff_copyin_byte(commbuff,
			     offsetof(DISABLECHANNEL_PACKET,
				      host_interface), host_interface);
	commbuff_copyin_byte(commbuff,
			     offsetof(DISABLECHANNEL_PACKET,
				      client_interface), client_interface);

	queue_commbuff(commbuff, &mux->send_queue);

	mux->total_queued_amount += sizeof(DISABLECHANNEL_PACKET);

	task_schedule(&mux->send_task);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * TransmitQueryInterface assembles a control query interface packet and
 * initializes the transmit procedure
 *
 * Params:
 * acktype specifies host or client as well as failure or success
 * host_interface is the interface which belongs to the host of the query
 * result is the resultant interface id which is relevent only when acktype
 *  	is host_end(SUCCESS)
 * mux is the MUX object the data is to be transmitted through
 */
int32 TransmitQueryInterface(int8 acktype, int8 host_interface,
			     int8 result, sint8 *name, MUX *mux)
{
	COMMBUFF *commbuff;
	QUERYINTERFACE_PACKET *packetdata;
	int32 namesize;

	DEBUG("TransmitQueryInterface(%lu, %lu, %lu, %p, %p)\n",
	      (int32) acktype, (int32) host_interface, (int32) result,
	      name, mux);

	if (name)
		namesize = strlen((char *) name) + 1;
	else
		namesize = 0;

	if (namesize > PACKET_MAXNAME_LENGTH)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	commbuff = int_alloc_commbuff(sizeof(QUERYINTERFACE_PACKET));

	packetdata = (QUERYINTERFACE_PACKET *) commbuff_data(commbuff);
	packetdata->type = PACKETTYPE_QUERYINTERFACE;

	commbuff_copyin_byte(commbuff,
			     offsetof(QUERYINTERFACE_PACKET, acktype),
			     acktype);
	commbuff_copyin_byte(commbuff,
			     offsetof(QUERYINTERFACE_PACKET,
				      host_interface), host_interface);
	commbuff_copyin_byte(commbuff,
			     offsetof(QUERYINTERFACE_PACKET, result),
			     result);
	commbuff_copyin(commbuff, offsetof(QUERYINTERFACE_PACKET, name),
			name, namesize);

	queue_commbuff(commbuff, &mux->send_queue);

	mux->total_queued_amount += sizeof(QUERYINTERFACE_PACKET);

	task_schedule(&mux->send_task);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * TransmitChannelSignal transmits a four int8 value to the channel
 *
 * Params:
 * acktype specifies host or client as well as failure or success
 * channel is the channel to be signaled
 * signal is the value to be delivered
 * mux is the MUX object the data is to be transmitted through
 */
int32 TransmitChannelSignal(int8 acktype, int8 channel, int32 signal,
			    MUX *mux)
{
	COMMBUFF *commbuff;
	CHANNELSIGNAL_PACKET *packetdata;

	DEBUG("TransmitChannelSignal(%lu, %lu, %lu, %p)\n",
	      (int32) acktype, (int32) channel, signal, mux);

	commbuff = int_alloc_commbuff(sizeof(CHANNELSIGNAL_PACKET));

	packetdata = (CHANNELSIGNAL_PACKET *) commbuff_data(commbuff);
	packetdata->type = PACKETTYPE_CHANNELSIGNAL;

	commbuff_copyin_byte(commbuff,
			     offsetof(CHANNELSIGNAL_PACKET, acktype),
			     acktype);
	commbuff_copyin_byte(commbuff,
			     offsetof(CHANNELSIGNAL_PACKET, channel),
			     channel);
	commbuff_copyin_dword(commbuff,
			      offsetof(CHANNELSIGNAL_PACKET, signal),
			      signal);

	queue_commbuff(commbuff, &mux->send_queue);

	mux->total_queued_amount += sizeof(CHANNELSIGNAL_PACKET);

	task_schedule(&mux->send_task);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * Log protocol messages
 *
 * Params:
 * cb is the commbuff to be logged
 */
void log_packettype(COMMBUFF *cb)
{

	int32 length = 0;
	int8 *data;
	int32 msgid = 0;

	if (!cb)
		return;

	data = commbuff_data(cb);
	length = commbuff_length(cb);

	switch (data[0]) {
	case PACKETTYPE_INVALID:
		msgid = NETMUX_PACKET_INVALID;
		break;
	case PACKETTYPE_DATA_HDR:
		msgid = NETMUX_PACKET_DATA;
		break;
	case PACKETTYPE_CREDIT:
		msgid = NETMUX_PACKET_CREDIT;
		break;
	case PACKETTYPE_ENABLEMUX:
		msgid = NETMUX_PACKET_ENABLEMUX;
		break;
	case PACKETTYPE_DISABLEMUX:
		msgid = NETMUX_PACKET_DISABLEMUX;
		break;
	case PACKETTYPE_ENABLECHANNEL:
		msgid = NETMUX_PACKET_ENABLECHANNEL;
		break;
	case PACKETTYPE_DISABLECHANNEL:
		msgid = NETMUX_PACKET_DISABLECHANNEL;
		break;
	case PACKETTYPE_QUERYINTERFACE:
		msgid = NETMUX_PACKET_QUERYINTERFACE;
		break;
	case PACKETTYPE_CHANNELSIGNAL:
		msgid = NETMUX_PACKET_CHANNELSIGNAL;
		break;
	default:
		return;
	};

	LOGCOMMBUFF_PROTO(netmux_log_port_handle, msgid, length, cb);
}
