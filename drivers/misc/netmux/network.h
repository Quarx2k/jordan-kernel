/******************************************************************************
 * NetMUX network.h                                                           *
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
 *   2007/02/25  Motorola    Added data_amount and mux_channel_queue_space    *
 *                           fields to the NETWORKDEVICE structure            *
 *   2008/07/09  Motorola    upmerge to kernel 2.6.24 for TI 23.5             *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* network.h is responsible for setting up a method of communication between  */
/* the NetMUX and user space applications.                                    */

#ifndef _NETMUX_NETWORK_H_
#define _NETMUX_NETWORK_H_


#include "types.h"
#include "errorcodes.h"
#include "utility.h"
#include "mux.h"

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if.h>
#include <linux/if_arp.h>
#include <linux/sockios.h>
#include <linux/workqueue.h>


/*
 * Definitions for specific states a network device can be in
 */
#define NETWORK_STATE_DEFAULT    0
#define NETWORK_STATE_CONFIGURED 1
#define NETWORK_STATE_CONNECTED  2
#define NETWORK_STATE_EVENT      4


/*
 * NETWORKDEVICE defines a network device associated with a channel
 * A brief description follows.
 *
 * netint points to the core network interface
 * channel declares the channel this network device works on
 * state defines the network device's state
 * client_interface is the interface id for
 *  	the external interface communicated with
 * burstsize is the maximum amount of data that can be transfered at a time
 * maxdata is the maximum amount of data the network device can queue
 * data_amount is the current amount of data stored in the process_queue
 * host_byte_credit is the maximum amount of data the device can receive
 * host_send_credit is the maximum amount of sends the device can receive
 * mux_channel_queue_space is the current amount of
 * 	 space available in the mux channel queue
 * event_wait defines a structure that network devices can sleep on
 * process_queue holds a list of commbuffs to be transmitted
 * netdevice points to the registered network device
 */
typedef struct NETWORKDEVICE {
	struct NETWORKINTERFACE *netint;

	int32 channel;
	int32 state;
	int32 client_interface;
	int32 burstsize;
	int32 maxdata;
	int32 data_amount;
	int32 host_byte_credit;
	int32 host_send_credit;
	int32 mux_channel_queue_space;
	struct net_device_stats stats;
	wait_queue_head_t event_wait;
	COMMBUFFQUEUE process_queue;
	struct net_device *netdevice;
} NETWORKDEVICE;

/*
 * NETWORKINTERFACE manages a list of network devices. A brief description
 * follows.
 *
 * channel_min is the inclusive lower boundary channel number
 * channel_max is the inclusive upper boundary channel number
 * host_interface defines the local interface id
 * mux points to the mux object associated with this interface
 * netdevs contains a list of network devices
 * lock keeps things in synch
 */
typedef struct NETWORKINTERFACE {
	int32 channel_min;
	int32 channel_max;
	int32 host_interface;

	MUX *mux;
	NETWORKDEVICE *netdevs;

	CRITICALSECTION lock;
} NETWORKINTERFACE;


/*
 * Functions defined in network.c
 */

int32 NetworkInform(void *, void *);
int32 NetworkReceive(COMMBUFF *, void *);

int32 CreateNetworkInterface(sint8 *, int32, int32, MUX *,
			     NETWORKINTERFACE **);
int32 DestroyNetworkInterface(NETWORKINTERFACE *);

void NetworkInit(struct net_device *);
int NetworkOpen(struct net_device *);
int NetworkClose(struct net_device *);
int NetworkTransmit(COMMBUFF *, struct net_device *);
struct net_device_stats *NetworkStats(struct net_device *);


#endif
