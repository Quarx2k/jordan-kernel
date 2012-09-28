/******************************************************************************
 * NetMUX network.c                                                           *
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
 *   2007/02/25  Motorola    Fixed NetworkTransmit related code               *
 *   2007/12/05  Motorola    Change code as kernel upgrade                    *
 *   2008/07/09  Motorola    upmerge to kernel 2.6.24 for TI 23.5             *
 *   2009/04/27  Motorola    Increment receive/transmit packet number         *
 *   2009/07/23  Motorola    Add wake lock functionality                      *
 *   2009/10/05  Motorola    Support IPv6                                     *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* network.c defines an interface between a NetMUX and the Linux networking   */
/* stack. This code is responsible for assisting application in communication */
/* with the NetMUX by implementing the standard file operations such as       */
/* open(), close(), read(), and write().                                      */

#include "network.h"
#include "config.h"
#include "debug.h"
#include <linux/wakelock.h>
#include <linux/ip.h>

extern struct wake_lock netmux_send_wakelock;

/*
 * NetworkInform is called by the mux (and sometimes the
 * config interface) to inform the interface that something
 * important has happened. The network interface listens for
 * several things of importance.
 *
 * First, it listens for config packets. If a config packet
 * is recieved it processes it and sets up the appropriate
 * channel.
 *
 * Second, it listens for a disable mux event. If such an
 * event happens all channels are forced closed and no
 * application can perform any operation on the channel.
 *
 * Third, it waits for enable channel messages. If such a
 * message is ever received it immediately responds with a
 * failure because the network interface does not support
 * external open requests.
 *
 * Fourth, it waits for a disable channel message. If
 * one is received the channel is placed into an inoperable
 * state.
 *
 * Fifth, it waits a data message, which signifies that the
 * channel has successfuly delivered data. The network interface
 * uses this information push any more data into the mux.
 *
 * Params:
 * param1 -- a custom pointer, in this case an INTERFACEINFORM struct
 * param2 -- a custom pointer, in this case a NETWORKINTERFACE struct
 */
int32 NetworkInform(void *param1, void *param2)
{
	NETWORKINTERFACE *network;
	NETWORKDEVICE *netdev;
	NETWORKDEVICE **priv_netdev;
	INTERFACEINFORM *informdata;
	CONFIGPACKET *configpacket;
	ENABLECHANNEL_PACKET *enablechannel;
	COMMBUFF *transmit;
	int32 channel_count;
	int32 channel;
	int32 type;
	int32 result;

	DEBUG("NetworkInform(0x%p, 0x%p)\n", param1, param2);

	informdata = (INTERFACEINFORM *) param1;
	network = (NETWORKINTERFACE *) param2;

	switch (informdata->inform_type) {
	case INFORM_INTERFACE_CONFIGPACKET:
		{
			channel =
			    ((CONFIGPACKET *) informdata->data)->channel;

			if (channel > network->channel_max
			    || channel < network->channel_min)
				return DEBUGERROR(ERROR_INVALIDPARAMETER);

			netdev =
			    &network->netdevs[channel -
					      network->channel_min];
			configpacket = (CONFIGPACKET *) informdata->data;

			netdev->netint = network;
			netdev->channel = channel;
			netdev->client_interface =
			    configpacket->host_interface;
			netdev->burstsize = configpacket->client_burstsize;
			netdev->maxdata = configpacket->client_maxdata;
			netdev->host_byte_credit =
			    configpacket->client_byte_credit;
			netdev->host_send_credit =
			    configpacket->client_send_credit;
			netdev->netdevice =
			    alloc_netdev(sizeof(struct NETWORKDEVICE *),
					 configpacket->channel_name,
					 NetworkInit);
			netdev->data_amount = 0;
			netdev->mux_channel_queue_space =
			    configpacket->client_maxdata;

			priv_netdev = netdev_priv(netdev->netdevice);
			*priv_netdev = netdev;
			memset(&netdev->stats, 0,
			       sizeof(struct net_device_stats));

			init_waitqueue_head(&netdev->event_wait);
			initialize_commbuff_queue(&netdev->process_queue);

			result = register_netdev(netdev->netdevice);
			if (result) {
				netdev->state = NETWORK_STATE_DEFAULT;
				destroy_commbuff_queue(&netdev->
						       process_queue);

				return DEBUGERROR(ERROR_INVALIDPARAMETER);
			}

			netdev->state = NETWORK_STATE_CONFIGURED;
		}
		break;

	case INFORM_INTERFACE_DISABLEMUX:
		{
			channel_count =
			    network->channel_max - network->channel_min +
			    1;
			for (channel = 0; channel < channel_count;
			     channel++) {
				if (network->netdevs[channel].state) {
					wake_up_interruptible(&network->
							      netdevs
							      [channel].
							      event_wait);

					NetworkClose(network->
						     netdevs[channel].
						     netdevice);

					unregister_netdev(network->
							  netdevs[channel].
							  netdevice);
				}
			}
		}
		break;

	case INFORM_INTERFACE_ENABLECHANNEL:
		{
			enablechannel =
			    (ENABLECHANNEL_PACKET *) informdata->data;
			type = strip_end(enablechannel->acktype);

			if (type == COMMAND) {
				EnableChannel(client_end(FAILURE),
					      enablechannel->channel,
					      0,
					      0,
					      enablechannel->
					      host_interface,
					      enablechannel->
					      client_interface, 0, 0, 0, 0,
					      network->mux);
			} else if (type == SUCCESS) {
				channel = enablechannel->channel;
				netdev =
				    &network->netdevs[channel -
						      network->
						      channel_min];

				netdev->state =
				    NETWORK_STATE_CONNECTED |
				    NETWORK_STATE_EVENT;

				wake_up_interruptible(&netdev->event_wait);
			} else if (type == FAILURE) {
				channel = enablechannel->channel;
				netdev =
				    &network->netdevs[channel -
						      network->
						      channel_min];

				netdev->state |= NETWORK_STATE_EVENT;

				wake_up_interruptible(&netdev->event_wait);
			}
		}
		break;

	case INFORM_INTERFACE_DISABLECHANNEL:
		{
			channel =
			    ((DISABLECHANNEL_PACKET *) informdata->data)->
			    channel;

			NetworkClose(network->
				     netdevs[channel -
					     network->channel_min].
				     netdevice);
		}
		break;

	case INFORM_INTERFACE_PREPSEND:
		{
			for (channel = 0;
			     channel <
			     network->channel_max - network->channel_min +
			     1; channel++) {
				netdev = &network->netdevs[channel];

				enter_write_criticalsection(&network->
							    lock);

				if (netdev->state & NETWORK_STATE_CONNECTED
				    && queue_length(&netdev->
						    process_queue)) {
					do {
						transmit =
						    dequeue_commbuff
						    (&netdev->
						     process_queue);
						netdev->data_amount -=
						    commbuff_length
						    (transmit);
						netdev->
						    mux_channel_queue_space
						    -=
						    commbuff_length
						    (transmit);

						result =
						    SendData(channel +
							     network->
							     channel_min,
							     transmit,
							     NULL,
							     network->mux);
					} while (result == ERROR_NONE
						 && queue_length(&netdev->
							 process_queue));

					if (result != ERROR_NONE) {
						queuefront_commbuff
						    (transmit,
						     &netdev->
						     process_queue);
						netdev->data_amount +=
						    commbuff_length
						    (transmit);
						netdev->
						    mux_channel_queue_space
						    +=
						    commbuff_length
						    (transmit);
					}
				}

				exit_write_criticalsection(&network->lock);
			}
		}
		break;

	case INFORM_INTERFACE_DATA:
		{
			channel = (int32) informdata->data;
			result = SendDataAvailable(channel, network->mux);

			enter_write_criticalsection(&network->lock);

			netdev =
			    &network->netdevs[channel -
					      network->channel_min];
			netdev->mux_channel_queue_space = result;

			exit_write_criticalsection(&network->lock);

			netif_wake_queue(netdev->netdevice);

			if (netdev->state & NETWORK_STATE_CONNECTED
			    && queue_length(&netdev->process_queue))
				RunSend(network->mux);

		}
		break;

	default:
		break;
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * NetworkReceive is called when a buffer is available for
 * delivery to an application. The network interface will
 * simply forward the buffer onto the tcp/ip stack and
 * let it do the remainder of the work.
 *
 * Params:
 * commbuff -- a pointer to the received data
 * param -- a custom pointer, in this case an INTERFACEINFORM struct
 */
int32 NetworkReceive(COMMBUFF *commbuff, void *param)
{
	INTERFACEINFORM *inform_data;
	NETWORKINTERFACE *netint;
	int32 channel;
	struct iphdr *iph;

	DEBUG("NetworkReceive(0x%p, 0x%p)\n", commbuff, param);

	inform_data = (INTERFACEINFORM *) param;

	netint = (NETWORKINTERFACE *) inform_data->inform_type;
	channel = (int32) inform_data->data;

	commbuff->dev =
	    netint->netdevs[channel - netint->channel_min].netdevice;
	commbuff->ip_summed = CHECKSUM_NONE;
	commbuff->pkt_type = PACKET_HOST;
	commbuff->mac_header = commbuff->data;
	/*Strip off MAC Header */
	skb_pull(commbuff, commbuff->dev->hard_header_len);
	/* Point to Network Layer Header */
	iph = (struct iphdr *) commbuff->data;
	/*Determine IP ver. Type  */
	commbuff->protocol = (iph->version == 6) ?
	    htons(ETH_P_IPV6) : htons(ETH_P_IP);

	LOGCOMMBUFF_CH(channel, "NetworkReceive()-->", commbuff,
		       commbuff_length(commbuff));

	netint->netdevs[channel - netint->channel_min].stats.rx_bytes +=
	    commbuff_length(commbuff);
	netint->netdevs[channel - netint->channel_min].stats.rx_packets++;

	/*
	 * To fix the behavior of a subpar kernel, we must call the commbuff
	 * destructor here, otherwise the Linux network stack overwrites
	 * the destructor on us.
	 */
	detag_commbuff(commbuff);
	netif_rx(commbuff);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * CreateNetworkInterface will create an interface object to be used by
 * a NetMUX. This object will keep track of all settings required to
 * communicate with the raw device layer. As such, any network operation
 * will make reference to this interface object.
 *
 * Params:
 * name -- the name of the interface
 * channel_min -- the inclusive lower bound of channel numbers
 * 	assigned to the interface
 * channel_max -- the inclusive upper bound of channel numbers
 *  	assigned to the interface
 * mux -- the mux object this interface is associated with
 * netint -- a pointer to a pointer to receive the newly created object
 */
int32 CreateNetworkInterface(sint8 *name, int32 channel_min,
			     int32 channel_max, MUX *mux,
			     NETWORKINTERFACE **netint)
{
	NETWORKINTERFACE *newnetint;
	int32 size;
	int32 result;

	DEBUG("CreateNetworkInterface(0x%p, %lu, %lu, 0x%p, 0x%p)\n", name,
	      channel_min, channel_max, mux, netint);

	if (!name || !mux || !netint)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	if (channel_min > channel_max)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	if (channel_max >= mux->maxchannels)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	if (strlen(name) >= PACKET_MAXNAME_LENGTH)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	newnetint =
	    (NETWORKINTERFACE *) alloc_mem(sizeof(NETWORKINTERFACE));
	size = (channel_max - channel_min + 1) * sizeof(NETWORKDEVICE);
	newnetint->netdevs = (NETWORKDEVICE *) alloc_mem(size);

	memset(newnetint->netdevs, 0, size);

	newnetint->channel_max = channel_max;
	newnetint->channel_min = channel_min;
	newnetint->mux = mux;

	result =
	    RegisterInterface(name, &NetworkInform, &NetworkReceive,
			      (int32) newnetint, mux->interface_lib);
	if (result != ERROR_NONE) {
		free_mem(newnetint->netdevs);
		free_mem(newnetint);

		return DEBUGERROR(result);
	}

	QueryInterfaceIndex(name, mux->interface_lib,
			    &newnetint->host_interface);
	initialize_criticalsection_lock(&newnetint->lock);

	*netint = newnetint;

	return DEBUGERROR(ERROR_NONE);
}

/*
 * DestroyNetworkInterface is invoked when a NetMUX is being destroyed.
 * The interface object will be 'undone'.
 *
 * Params:
 * netint -- the interface to be destroyed
 */
int32 DestroyNetworkInterface(NETWORKINTERFACE *netint)
{
	int32 channel;
	int32 channel_count;

	DEBUG("DestroyNetworkInterface(0x%p)\n", netint);

	channel_count = netint->channel_max - netint->channel_min + 1;
	for (channel = 0; channel < channel_count; channel++) {
		if (netint->netdevs[channel].state) {
			NetworkClose(netint->netdevs[channel].netdevice);

			destroy_commbuff_queue(&netint->netdevs[channel].
					       process_queue);
			unregister_netdev(netint->netdevs[channel].
					  netdevice);
			free_netdev(netint->netdevs[channel].netdevice);
		}
	}

	UnregisterInterface(netint->host_interface,
			    netint->mux->interface_lib);
	destroy_criticalsection_lock(&netint->lock);

	free_mem(netint->netdevs);
	free_mem(netint);

	return DEBUGERROR(ERROR_NONE);
}

static const struct net_device_ops netmux_netdev_ops = {
	.ndo_open = NetworkOpen,
	.ndo_stop = NetworkClose,
	.ndo_start_xmit = NetworkTransmit,
	.ndo_get_stats = NetworkStats,
};


/*
 * NetworkInit is called by the linux whenever a network interface
 * needs to be brought online. This function merely establishes
 * a method of communication between the linux network interface
 * and the NetMUX
 *
 * Params:
 * netdev -- structure to store the method of communication
 */
void NetworkInit(struct net_device *netdev)
{
	DEBUG("NetworkInit(0x%p)\n", netdev);

	netdev->hard_header_len = 0;
	netdev->addr_len = 0;
	netdev->mtu = 1500;
	netdev->tx_queue_len = 1000;	/* determine appropriate value */
	netdev->netdev_ops = &netmux_netdev_ops;
	netdev->flags = IFF_NOARP;
	netdev->type = ARPHRD_NONE;
}

/*
 * NetworkOpen is called by the Linux networking system to connect
 * a network device. The connection event will cause the enable
 * channel protocol to commence.
 *
 * Params:
 * netdev -- a pointer to a net device
 */
int NetworkOpen(struct net_device *netdev)
{
	NETWORKDEVICE *device;
	NETWORKDEVICE **priv_netdev;
	int32 result;

	DEBUG("NetworkOpen(0x%p)\n", netdev);

	priv_netdev = netdev_priv(netdev);
	device = *priv_netdev;

	if (!(device->state & NETWORK_STATE_CONFIGURED))
		return -EADDRNOTAVAIL;

	result = EnableChannel(host_end(COMMAND),
			       device->channel,
			       device->burstsize,
			       device->maxdata,
			       device->netint->host_interface,
			       device->client_interface,
			       device->host_byte_credit,
			       device->host_send_credit,
			       0, 0, device->netint->mux);

	if (result != ERROR_NONE)
		return -ECONNABORTED;

	/* Wake up when either one of two conditions occur:
	   1 - A response is received from the BP for our open request.
	   2 - The channel is not configured and running properly.
	   In case the latter is true an error will be returned eventually. */
	wait_event_interruptible(device->event_wait,
				 (device->state & NETWORK_STATE_EVENT) ||
				 !(device->
				   state & NETWORK_STATE_CONFIGURED));

	device->state &= ~NETWORK_STATE_EVENT;

	if (!(device->state & NETWORK_STATE_CONNECTED))
		return -ECONNREFUSED;

	netif_start_queue(netdev);

	return 0;
}

/*
 * NetworkClose is called by the Linux networking system to
 * disconnect a network device. This function will cause the
 * commencement of the disconnect channel protocol.
 *
 * Params:
 * netdev -- a pointer to a net device
 */
int NetworkClose(struct net_device *netdev)
{
	NETWORKDEVICE **priv_netdev;
	NETWORKDEVICE *device;
	int32 result;

	DEBUG("NetworkClose(0x%p)\n", netdev);

	priv_netdev = netdev_priv(netdev);
	device = *priv_netdev;

	if (device->state & NETWORK_STATE_CONNECTED) {
		result =
		    DisableChannel(host_end(COMMAND), device->channel,
				   device->netint->host_interface,
				   device->client_interface,
				   device->netint->mux);
		if (result != ERROR_NONE)
			return -EPIPE;

		enter_write_criticalsection(&device->netint->lock);

		device->state = NETWORK_STATE_CONFIGURED;
		empty_commbuff_queue(&device->process_queue);
		device->data_amount = 0;
		device->mux_channel_queue_space = device->maxdata;

		exit_write_criticalsection(&device->netint->lock);
	} else
		return -ENOTCONN;

	netif_stop_queue(netdev);

	return 0;
}

/*
 * NetworkTransmit is called by the linux networking system when there
 * is data to be sent over the link. The data will be queued by the network
 * interface until the mux is ready to send the data. At this time the network
 * interface must push what is queued onto the mux.
 *
 * commbuff -- the data to be sent
 * netdev -- the device to send the data on
 */
int NetworkTransmit(COMMBUFF *commbuff, struct net_device *netdev)
{
	NETWORKDEVICE **priv_netdev;
	NETWORKDEVICE *device;
	int32 length = commbuff_length(commbuff);

	DEBUG("NetworkTransmit(0x%p, 0x%p)\n", commbuff, netdev);

	/* Acquire NM_send wakelock */
	DEBUG("Acquire netmux_send_wakelock\n");
	wake_lock(&netmux_send_wakelock);

	priv_netdev = netdev_priv(netdev);
	device = *priv_netdev;

	device->stats.tx_bytes += length;
	device->stats.tx_packets++;

	queue_commbuff(commbuff, &device->process_queue);
	device->data_amount += length;

	/* Due to a race condition that exists between the time
	 * we decrement and increment the process_queue length,
	 * more data could have been added to the NetMUX queue than
	 * the burst size for the channel. Also,
	 * the amount of combined data in the two queues
	 * may surpass the channel queue size.
	 */
	if ((device->mux_channel_queue_space - device->data_amount)
	    < device->burstsize) {
		netif_stop_queue(netdev);
	}

	RunSend(device->netint->mux);

	return 0;
}

struct net_device_stats *NetworkStats(struct net_device *netdev)
{
	NETWORKDEVICE **priv_netdev;
	NETWORKDEVICE *device;

	DEBUG("NetworkStats(0x%p)\n", netdev);

	priv_netdev = netdev_priv(netdev);
	device = *priv_netdev;

	return &device->stats;
}
