/******************************************************************************
 * NetMUX config.h                                                            *
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
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* config.h is responsible for setting up constant values and data types to   */
/* be used by config.c as well as channelconfig.c                             */

#ifndef _NETMUX_CONFIG_H_
#define _NETMUX_CONFIG_H_


#include "types.h"
#include "errorcodes.h"
#include "utility.h"
#include "mux.h"
#include "protocol.h"


/*
 * Definition of different commands the config interface knows about
 */
#define CONFIG_CORE_TYPE           0
#define CONFIG_USERIF_STATUSREPORT 1
#define CONFIG_USERIF_ADJUSTCREDIT 2

/*
 * Definitions to define standard parameters
 */
#define CONFIG_BURSTSIZE   10000
#define CONFIG_MAXDATA     10000
#define CONFIG_BYTECREDIT  10000
#define CONFIG_SENDCREDIT  10

/*
 * Definitions to declare possible states for the interface
 */
#define CONFIG_STATE_DEFAULT    0
#define CONFIG_STATE_ENABLING   1
#define CONFIG_STATE_CONNECTING 2
#define CONFIG_STATE_CONNECTED  3

/*
 * Definitions to declare possible states for the configuration data
 */
#define CONFIGDATA_STATE_DEFAULT    0
#define CONFIGDATA_STATE_VERIFIED   1
#define CONFIGDATA_STATE_DISTRIBUTE 2

/*
 * Definition of the config interface name
 */
#define CONFIG_INTERFACE_NAME "netmux_config"

/*
 * Initial amount of memory used to hold a status report
 */
#define DEFAULT_CONFIG_DATASIZE 4096


/*
 * CONFIGDATA defines a structure used for channel description and declaration.
 * The format is briefly described below.
 *
 * channel represents the channel number
 * channel_extra is additional data to be associated with the channel
 * channel_name names the channel
 * host_interface defines the host interface to be used with this channel
 * host_interface_index is the interface id of the host interface
 * host_burstsize is the maximum amount of data that
 * 	can be sent at a time hostly
 * host_maxdata is the maximum amount of data that can be queued hostly
 * host_byte_credit is the initial amount of byte credit
 * 	delivered to the client on open
 * host_send_credit is the initial amount of send
 * 	credit delivered to the client on open
 * client_interface defines the client interface to be used with this channel
 * client_interface_index defines the interface id of the client interface
 * client_burstsize defines the maximum amount of
 * 	data that can be sent a time cliently
 * client_maxdata defines the maximum amount of
 * 	data the can be queued cliently
 * client_byte_credit is the max number of bytes
 * 	the host can send to the client
 * client_send_credit is the max number of sends
 * 	the host can perform to the client
 * state is the current state of this configuration data entry
 * msgid stores the 4 byte message id, used by RTA
 */
typedef struct CONFIGDATA {
	int32 channel;
	int32 channel_extra;
	sint8 channel_name[PACKET_MAXNAME_LENGTH];
	sint8 host_interface[PACKET_MAXNAME_LENGTH];
	int32 host_interface_id;
	int32 host_burstsize;
	int32 host_maxdata;
	int32 host_byte_credit;
	int32 host_send_credit;
	sint8 client_interface[PACKET_MAXNAME_LENGTH];
	int32 client_interface_id;
	int32 client_burstsize;
	int32 client_maxdata;
	int32 client_byte_credit;
	int32 client_send_credit;
	int32 state;
	int32 msgid;
} CONFIGDATA;

/*
 * CONFIGPACKET defines the configuration data to be transmitted.
 * The format is briefly described below.
 *
 * channel represents the channel to associate this data with
 * host_interface is the interface id for the channel to communicate on
 * client_interface is the interface id on which this data should be used
 * client_burstsize is the maximum amount of data that can be sent at a time
 * client_maxdata is the maximum amount of data that can be queued at a time
 * client_byte_credit is the max number of bytes
 * 	the host can send to the client
 * client_send_credit is the max number of sends
 * 	the host can perform to the client
 * channel_extra is additional data to be associated with the channel
 * channel_name is the name of the channel
 */
START_PACKED_STRUCT(CONFIGPACKET)
PACKED_MEMBER(int32 channel);
PACKED_MEMBER(int32 host_interface);
PACKED_MEMBER(int32 client_interface);
PACKED_MEMBER(int32 client_burstsize);
PACKED_MEMBER(int32 client_maxdata);
PACKED_MEMBER(int32 client_byte_credit);
PACKED_MEMBER(int32 client_send_credit);
PACKED_MEMBER(int32 channel_extra);
PACKED_MEMBER(sint8 channel_name[PACKET_MAXNAME_LENGTH]);
END_PACKED_STRUCT(CONFIGPACKET)

/*
 * USERCONFIG_ADJUSTCREDIT defines a packet type that is used when a user
 * issues a command to adjust mux credit levels.
 *
 * channel_valid indicates whether or not a channels
 * 	credit levels should be tweaked
 * channel is the channel to tweak the credit levels on
 * max_host_byte_credit is the amount to tweak the channels
 * 	max byte credit value by
 * max_host_send_credit is the amount to tweak the channels
 * 	max send credit value by
 * replenished_byte_credit is the amount to tweak the channels
 * 	replenished byte credit value by
 * replenished_send_credit is the amount to tweak the channels
 * 	replenished send credit value by
 * client_byte_credit is the amount to tweak the channels
 * 	client byte credit value by
 * client_send_credit is the amount to tweak the channels
 * 	client send credit value by
 * global_send_credit is the amount to tweak the muxs send credit by
 * max_global_send_credit is the amount to tweak the muxs max send credit by
 */
START_PACKED_STRUCT(USERCONFIG_ADJUSTCREDIT)
PACKED_MEMBER(int32 channel_valid);
PACKED_MEMBER(int32 channel);
PACKED_MEMBER(int32 max_host_byte_credit);
PACKED_MEMBER(int32 max_host_send_credit);
PACKED_MEMBER(int32 replenished_byte_credit);
PACKED_MEMBER(int32 replenished_send_credit);
PACKED_MEMBER(int32 client_byte_credit);
PACKED_MEMBER(int32 client_send_credit);
PACKED_MEMBER(int32 global_send_credit);
PACKED_MEMBER(int32 max_global_send_credit);
END_PACKED_STRUCT(USERCONFIG_ADJUSTCREDIT)

/*
 * USERCONFIG_STATUS defines a packet type that is used when a user
 * issues a command to request or deliver a mux status report
 *
 * request indicates whether the user is asking for a report or delivering one
 * size is only valid if request = 0 and it indicates the size of the report
 * data is an array of size 'size' and it contains the status report
 */
START_PACKED_STRUCT(USERCONFIG_STATUS)
PACKED_MEMBER(int32 request);
PACKED_MEMBER(int32 size);

PACKED_MEMBER(int8 data[0]);
END_PACKED_STRUCT(USERCONFIG_STATUS)


/*
 * CONFIGINTERFACE defines parameters for the configuration interface.
 * The format is briefly described below.
 *
 * configdata points to channel configuration data
 * configdata_count specifies the number of entries in configdata
 * host_interface is the host interface id of the config interface
 * client_interface is the client interface id of the config interface
 * channel is the channel the config interface operates on
 * state is the current state of the config interface
 * host is non zero if the config interface is to act as a host
 * mux points to the MUX object to be oeprated on
 * user_interface_data points to environment specific data
 * 	used for user configuration
 */
typedef struct CONFIGINTERFACE {
	CONFIGDATA *configdata;
	int32 configdata_count;
	int32 host_interface;
	int32 client_interface;
	int32 channel;
	int32 state;
	int32 host;
	MUX *mux;

	void *user_interface_data;
} CONFIGINTERFACE;


/*
 * Define various functions used by the config interface
 */

int32 ConfigInform(void *, void *);
int32 ConfigReceive(COMMBUFF *, void *);

int32 CreateConfigInterface(int32, MUX *, int32, CONFIGDATA *,
			    CONFIGINTERFACE **);
int32 DestroyConfigInterface(CONFIGINTERFACE *);

int32 ActivateConfigInterface(int32, CONFIGINTERFACE *);
int32 DeactivateConfigInterface(CONFIGINTERFACE *);

int32 GenerateConfigPacket(CONFIGDATA *, CONFIGINTERFACE *);

int32 StartupConfigUserInterface(CONFIGINTERFACE *);
void ShutdownConfigUserInterface(CONFIGINTERFACE *);

void ReceiveClientConfigRequest(CONFIGINTERFACE *, COMMBUFF *);


#endif
