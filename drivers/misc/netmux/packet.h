/******************************************************************************
 * NetMUX packet.h                                                            *
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
 *   2006/12/19  Motorola    Combined header and data into one transfer       *
 *                           Align packets on 4 byte boundaries               *
 *   2007/07/31  Motorola    Add support for larger bytecredit
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* packet.h defines a list of structures to hold each packet type. Each       */
/* structure must be packed so it can be sent and received without any        */
/* difference between environments.                                           */

#ifndef _NETMUX_PACKET_H_
#define _NETMUX_PACKET_H_


#include "utility.h"


/*
 * The following define packet type codes
 */
#define PACKETTYPE_INVALID        0
#define PACKETTYPE_DATA_HDR       1
#define PACKETTYPE_DATA_BDY       2
#define PACKETTYPE_CREDIT         3
#define PACKETTYPE_ENABLEMUX      4
#define PACKETTYPE_DISABLEMUX     5
#define PACKETTYPE_ENABLECHANNEL  6
#define PACKETTYPE_DISABLECHANNEL 7
#define PACKETTYPE_QUERYINTERFACE 8
#define PACKETTYPE_CHANNELSIGNAL  9

#define PACKET_MAXNAME_LENGTH 16


/*
 * DATA_PACKET_HDR defines the format of a header for a data packet.
 * A brief description follows.
 *
 * type identifies the packet
 * channel is the channel to which the data belongs
 * len is the number of data bytes
 */
START_PACKED_STRUCT(DATA_PACKET_HDR)
    PACKED_MEMBER(int8 type);
PACKED_MEMBER(int8 channel);
PACKED_MEMBER(int16 len);
END_PACKED_STRUCT(DATA_PACKET_HDR)

/*
 * CREDIT_PACKET defines a credit control packet to change
 * the amount of receive space.
 * A brief description follows.
 *
 * type identifies the packet
 * acktype defines direction and result of the control packet
 * channel defines the channel the credit is to be applied to
 * padding is only used for data alignment
 * bytecredit is the number of bytes to augment the credit by
 * sendcredit is the number of sends to augment the credit by
 */
    START_PACKED_STRUCT(CREDIT_PACKET)
    PACKED_MEMBER(int8 type);
PACKED_MEMBER(int8 acktype);
PACKED_MEMBER(int8 channel);
PACKED_MEMBER(int8 padding);
PACKED_MEMBER(int32 bytecredit);
PACKED_MEMBER(int32 sendcredit);
END_PACKED_STRUCT(CREDIT_PACKET)

/*
 * ENABLEMUX_PACKET defines an enable mux control packet to enable the MUX
 * A brief description follows.
 *
 * type identifies the packet
 * acktype defines direction and result of the control packet
 * padding_byte1 is only used for data alignment
 * padding_byte2 is only used for data alignment
 */
    START_PACKED_STRUCT(ENABLEMUX_PACKET)
    PACKED_MEMBER(int8 type);
PACKED_MEMBER(int8 acktype);
PACKED_MEMBER(int8 padding_byte1);
PACKED_MEMBER(int8 padding_byte2);
END_PACKED_STRUCT(ENABLEMUX_PACKET)

/*
 * DISABLEMUX_PACKET defines a disable mux control packet to disable the MUX
 * A brief description follows.
 *
 * type identifies the packet
 * acktype defines direction and result of the control packet
 * padding_byte1 is only used for data alignment
 * padding_byte2 is only used for data alignment
 */
    START_PACKED_STRUCT(DISABLEMUX_PACKET)
    PACKED_MEMBER(int8 type);
PACKED_MEMBER(int8 acktype);
PACKED_MEMBER(int8 padding_byte1);
PACKED_MEMBER(int8 padding_byte2);
END_PACKED_STRUCT(DISABLEMUX_PACKET)

/*
 * ENABLECHANNEL_PACKET defines a enable channel control packet
 * to enable a specific channel.
 * A brief description follows,
 *
 * type identifies the packet
 * acktype defines direction and result of the control packet
 * channel defines the channel to be enabled
 * host_interface represents the interface id on the host side
 * client_interface represents the interface id on the client side
 * padding is only used for data alignment
 * bytecredit describes the senders receive space
 * sendcredit describes the senders send space
 */
    START_PACKED_STRUCT(ENABLECHANNEL_PACKET)
    PACKED_MEMBER(int8 type);
PACKED_MEMBER(int8 acktype);
PACKED_MEMBER(int8 channel);
PACKED_MEMBER(int8 host_interface);
PACKED_MEMBER(int8 client_interface);
PACKED_MEMBER(int8 padding_byte1);
PACKED_MEMBER(int8 padding_byte2);
PACKED_MEMBER(int8 padding_byte3);
PACKED_MEMBER(int32 bytecredit);
PACKED_MEMBER(int32 sendcredit);
END_PACKED_STRUCT(ENABLECHANNEL_PACKET)

/*
 * DISABLECHANNEL_PACKET defines a disable channel control packet to disable a
 * specific channel. A brief description follows.
 *
 * type identifies the packet
 * acktype defines direction and result of the control packet
 * channel defines the channel to be disabled
 * host_interface represents the interface id on the host side
 * client_interface represents the interface id on the client side
 * padding_byte1 is only used for data alignment
 * padding_byte2 is only used for data alignment
 * padding_byte3 is only used for data alignment
 */
    START_PACKED_STRUCT(DISABLECHANNEL_PACKET)
    PACKED_MEMBER(int8 type);
PACKED_MEMBER(int8 acktype);
PACKED_MEMBER(int8 channel);
PACKED_MEMBER(int8 host_interface);
PACKED_MEMBER(int8 client_interface);
PACKED_MEMBER(int8 padding_byte1);
PACKED_MEMBER(int8 padding_byte2);
PACKED_MEMBER(int8 padding_byte3);
END_PACKED_STRUCT(DISABLECHANNEL_PACKET)

/*
 * QUERYINTERFACE_PACKET defines a query interface control packet to
 * retrieve an interface id.
 * A brief description follows.
 *
 * type identifies the packet
 * acktype defines direction and result of the control packet
 * host_interface is the host's interface id
 * result only applies to a successful query and is the requested interface id
 * name is the name of the interface queried
 */
    START_PACKED_STRUCT(QUERYINTERFACE_PACKET)
    PACKED_MEMBER(int8 type);
PACKED_MEMBER(int8 acktype);
PACKED_MEMBER(int8 host_interface);
PACKED_MEMBER(int8 result);
PACKED_MEMBER(sint8 name[PACKET_MAXNAME_LENGTH]);
END_PACKED_STRUCT(QUERYINTERFACE_PACKET)

/*
 * CHANNELSIGNAL_PACKET defines a channel signal control packet to
 * signal a channel
 * A brief description follows.
 *
 * type identifies the packet
 * acktype defines direction and result of the control packet
 * channel identifies the destination
 * padding is only used for data alignment
 * signal is the signal to send
 */
    START_PACKED_STRUCT(CHANNELSIGNAL_PACKET)
    PACKED_MEMBER(int8 type);
PACKED_MEMBER(int8 acktype);
PACKED_MEMBER(int8 channel);
PACKED_MEMBER(int8 padding);
PACKED_MEMBER(int32 signal);
END_PACKED_STRUCT(CHANNELSIGNAL_PACKET)

#endif
