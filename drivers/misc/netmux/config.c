/******************************************************************************
 * NetMUX config.c                                                            *
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
 *   2006/11/10  Motorola    Fixed /proc/netmux_config_host0 bugs             *
 *   2006/12/19  Motorola    Combine header and data into one transfer        *
 *   2007/12/05  Motorola    Change code for INIT_WORK changed in kernel      *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* config.c defines the various functionality responsible for setting up the  */
/* NetMUX and maintaining its integrity.                                      */

#include <linux/proc_fs.h>

#include "config.h"
#include "debug.h"
typedef struct USERCONFIGDATA {
	struct proc_dir_entry *host_config_file;
	struct proc_dir_entry *client_config_file;

	sint8 *host_config_data;
	sint8 *client_config_data;

	int32 host_config_datasize;
	int32 client_config_datasize;
	int32 host_config_dataused;

	int32 mux_id;

	COMMBUFFQUEUE commandqueue;
	TASKDATA commandtask;
	CRITICALSECTION config_synch;

	wait_queue_head_t event_wait;
	CONFIGINTERFACE *config;
} USERCONFIGDATA;


/*
 * ConfigInform is notified by the mux when an interesting
 * event happens. The config interface pays attention to
 * these events to accomplish several things.
 *
 * First, the config interface listens for enable mux,
 * enable channel, and query interface to help
 * initialize the NetMUX.
 *
 * Second, the config interface listens for disable mux.
 * This case signifies an error has happened within the
 * NetMUX or a linkdriver and the config interface should
 * attempt to bring the NetMUX back into a working state.
 *
 * Third, the config interface listens for disable channel.
 * If this signal is received the config interface assumes
 * the NetMUX is shutting down, and as such, the config
 * interface will help bring down anything it's
 * responsible for.
 *
 * Params:
 *
 * param1 -- a custom value, in this case the mux always
 *           delivers a pointer to an INTERFACEINFORM struct
 * param2 -- a custom value provided by the interface
 *           creator. In this case, the config interface
 *           specified a CONFIGINTERFACE struct
 */
int32 ConfigInform(void *param1, void *param2)
{
	INTERFACEINFORM passinformdata;
	CONFIGPACKET configpacket;
	ENABLECHANNEL_PACKET *enablechannel;
	QUERYINTERFACE_PACKET *queryinterface;
	ENABLEMUX_PACKET *enablemux;
	CONFIGINTERFACE *config;
	CONFIGDATA *configdata;
	INTERFACEINFORM *informdata;
	int32 result;
	int32 index;
	int8 type;

	DEBUG("ConfigInform(0x%p, 0x%p)\n", param1, param2);

	informdata = (INTERFACEINFORM *) param1;
	config = (CONFIGINTERFACE *) param2;

	switch (informdata->inform_type) {
	case INFORM_INTERFACE_ENABLEMUX:
		{
			if (config->state != CONFIG_STATE_ENABLING)
				break;

			enablemux = (ENABLEMUX_PACKET *) informdata->data;

			type = (int8) strip_end(enablemux->acktype);
			if (type == SUCCESS) {
				config->state = CONFIG_STATE_CONNECTING;
				configdata = config->configdata;

				for (index = 0;
				     index < config->configdata_count;
				     index++) {
					result =
					    QueryInterfaceIndex(configdata
							[index].
							host_interface,
							config->
							mux->
							interface_lib,
							&configdata
							[index].
							host_interface_id);

					if (result == ERROR_NONE)
						configdata[index].state =
						    CONFIGDATA_STATE_VERIFIED;
					else
						configdata[index].state =
						    CONFIGDATA_STATE_DEFAULT;
				}

				if (config->host)
					QueryExternalInterface(host_end
						       (COMMAND),
						       config->
						       host_interface,
						       0,
						       CONFIG_INTERFACE_NAME,
						       config->
						       mux);
			} else {
				if (config->host)
					EnableMUX(host_end(COMMAND),
						  config->mux);
			}
		}
		break;

	case INFORM_INTERFACE_DISABLEMUX:
		{
			if (config->state == CONFIG_STATE_DEFAULT)
				break;

			config->state = CONFIG_STATE_ENABLING;

			if (config->host)
				EnableMUX(host_end(COMMAND), config->mux);
		}
		break;

	case INFORM_INTERFACE_ENABLECHANNEL:
		{
			enablechannel =
			    (ENABLECHANNEL_PACKET *) informdata->data;

			type = (int8) strip_end(enablechannel->acktype);
			if (type == SUCCESS) {
				config->state = CONFIG_STATE_CONNECTED;

				if (config->configdata_count)
					QueryExternalInterface(host_end
							       (COMMAND),
							       config->
							       host_interface,
							       0,
							       config->
							       configdata
							       [0].
							       client_interface,
							       config->
							       mux);
			} else {
				if (config->host) {
					EnableChannel(host_end(COMMAND),
						      config->channel,
						      CONFIG_BURSTSIZE,
						      CONFIG_MAXDATA,
						      config->
						      host_interface,
						      config->
						      client_interface,
						      CONFIG_BYTECREDIT,
						      CONFIG_SENDCREDIT, 0,
						      0, config->mux);
				} else {
					config->client_interface =
					    enablechannel->host_interface;
					config->state =
					    CONFIG_STATE_CONNECTED;

					EnableChannel(client_end(COMMAND),
						      enablechannel->
						      channel,
						      CONFIG_BURSTSIZE,
						      CONFIG_MAXDATA,
						      enablechannel->
						      host_interface,
						      enablechannel->
						      client_interface,
						      CONFIG_BYTECREDIT,
						      CONFIG_SENDCREDIT,
						      enablechannel->
						      bytecredit,
						      enablechannel->
						      sendcredit,
						      config->mux);

					if (config->configdata_count)
						QueryExternalInterface
						    (host_end(COMMAND),
						     config->
						     host_interface, 0,
						     config->configdata[0].
						     client_interface,
						     config->mux);
				}
			}
		}
		break;

	case INFORM_INTERFACE_DISABLECHANNEL:
		{
			config->state = CONFIG_STATE_CONNECTING;
			if (config->host) {
				EnableChannel(host_end(COMMAND),
					      config->channel,
					      CONFIG_BURSTSIZE,
					      CONFIG_MAXDATA,
					      config->host_interface,
					      config->client_interface,
					      CONFIG_BYTECREDIT,
					      CONFIG_SENDCREDIT,
					      0, 0, config->mux);
			}
		}
		break;

	case INFORM_INTERFACE_QUERYINTERFACE:
		{
			queryinterface =
			    (QUERYINTERFACE_PACKET *) informdata->data;

			type = (int8) strip_end(queryinterface->acktype);
			if (config->state == CONFIG_STATE_CONNECTING) {
				if (type == SUCCESS) {
					config->client_interface =
					    queryinterface->result;

					if (config->host) {
						EnableChannel(host_end
							      (COMMAND),
							      config->
							      channel,
							      CONFIG_BURSTSIZE,
							      CONFIG_MAXDATA,
							      config->
							      host_interface,
							      config->
							      client_interface,
							      CONFIG_BYTECREDIT,
							      CONFIG_SENDCREDIT,
							      0, 0,
							      config->mux);
					}
				} else
					QueryExternalInterface(host_end
						       (COMMAND),
						       config->
						       host_interface,
						       0,
						       CONFIG_INTERFACE_NAME,
						       config->
						       mux);
			} else if (config->state == CONFIG_STATE_CONNECTED) {
				configdata = config->configdata;

				if (type == SUCCESS)
					result = 0;
				else
					result = 1;

				for (index = 0;
				     index < config->configdata_count;
				     index++) {
					if (configdata[index].state !=
					    CONFIGDATA_STATE_VERIFIED)
						continue;

					if (type == FAILURE) {
						if (result) {
							configdata[index].
							state =
						CONFIGDATA_STATE_DEFAULT;
							result = 0;

							continue;
						} else {
							result = 1;

							QueryExternalInterface
							    (host_end
							     (COMMAND),
							     config->
							     host_interface,
							     0,
							     configdata
							     [index].
							     client_interface,
							     config->mux);

							break;
						}
					}

					if (!strcmp
					    ((char *) configdata[index].
					     client_interface,
					     (char *) queryinterface->
					     name)) {
						configdata[index].state =
						    CONFIGDATA_STATE_DISTRIBUTE;
						configdata[index].
						    client_interface_id =
						    queryinterface->result;
					} else if (!result) {
						result = 1;
						QueryExternalInterface
						    (host_end(COMMAND),
						     config->
						     host_interface, 0,
						     configdata[index].
						     client_interface,
						     config->mux);
					}
				}

				if (!result) {
					passinformdata.source =
					    config->mux;
					passinformdata.inform_type =
					    INFORM_INTERFACE_CONFIGPACKET;

					for (index = 0;
					     index <
					     config->configdata_count;
					     index++) {
						if (configdata[index].
						    state !=
						    CONFIGDATA_STATE_DISTRIBUTE)
							continue;

						configpacket.channel =
						    configdata[index].
						    channel;
						configpacket.
						    client_burstsize =
						    configdata[index].
						    host_burstsize;
						configpacket.
						    client_maxdata =
						    configdata[index].
						    host_maxdata;
						configpacket.
						    client_byte_credit =
						    configdata[index].
						    host_byte_credit;
						configpacket.
						    client_send_credit =
						    configdata[index].
						    host_send_credit;
						configpacket.
						    host_interface =
						    configdata[index].
						    client_interface_id;
						configpacket.
						    client_interface =
						    configdata[index].
						    host_interface_id;
						configpacket.
						    channel_extra =
						    configdata[index].
						    channel_extra;

						memcpy((void *)
						       configpacket.
						       channel_name,
						       configdata[index].
						       channel_name,
						       PACKET_MAXNAME_LENGTH);

						passinformdata.data =
						    (void *) &configpacket;

						LIBRARY_INFORM(&passinformdata,
						     configpacket.
						     client_interface,
						     config->mux->
						     interface_lib);
						GenerateConfigPacket
						    (&configdata[index],
						     config);
					}
				}
			}
		} break;

	default:
		{
			return DEBUGERROR(ERROR_OPERATIONRESTRICTED);
		}
		break;
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * ConfigReceive is called by the mux whenever data is received
 * on the config interface's channel.
 *
 * All data received on this channel is assumed to be a
 * configuration packet.
 *
 * Once a packet is received it is parsed and communicated to
 * the responsible interface. That interface then configures
 * what is necessary. The config interface merely acts as
 * the middle man.
 *
 * Params:
 * commbuff -- the buffer being delivered to the interface
 * param -- a custom value, in this case the mux provides
 *          an INTERFACEINFORM struct
 */
int32 ConfigReceive(COMMBUFF *commbuff, void *param)
{
	CONFIGPACKET configpacket;
	INTERFACEINFORM passinformdata;
	CONFIGINTERFACE *config;
	INTERFACEINFORM *informdata;
	MUX *mux;
	int8 type;

	DEBUG("ConfigReceive(0x%p, 0x%p)\n", commbuff, param);

	informdata = (INTERFACEINFORM *) param;

	mux = informdata->source;
	config = (CONFIGINTERFACE *) informdata->inform_type;

	commbuff_copyout((void *) &type, commbuff, 0, sizeof(int8));

	if (type != CONFIG_CORE_TYPE)
		ReceiveClientConfigRequest(config, commbuff);
	else {
		commbuff_remove_front(commbuff, sizeof(sint8));
		commbuff_copyout((void *) &configpacket, commbuff, 0,
				 sizeof(CONFIGPACKET));
		free_commbuff(commbuff);

		passinformdata.source = mux;
		passinformdata.inform_type = INFORM_INTERFACE_CONFIGPACKET;
		passinformdata.data = (void *) &configpacket;

		convert_dword(configpacket.channel);
		convert_dword(configpacket.host_interface);
		convert_dword(configpacket.client_interface);
		convert_dword(configpacket.client_burstsize);
		convert_dword(configpacket.client_maxdata);
		convert_dword(configpacket.client_byte_credit);
		convert_dword(configpacket.client_send_credit);
		convert_dword(configpacket.channel_extra);

		LIBRARY_INFORM(&passinformdata, configpacket.client_interface,
			       mux->interface_lib);
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * CreateConfigInterface creates a config interface object.
 * There is only one config interface object per NetMUX and each
 * object keeps track of the necessary data to bring up the NetMUX
 * and keep it running.
 *
 * Params:
 * channel -- the channel the config interface works over
 * mux -- the mux to be associated with this interface
 * configdata_count -- number of configuration entries
 * configdata -- list of configuration entries
 * config -- a poitner to a pointer to receive the new object
 */
int32 CreateConfigInterface(int32 channel,
			    MUX *mux,
			    int32 configdata_count,
			    CONFIGDATA *configdata,
			    CONFIGINTERFACE **config)
{
	CONFIGINTERFACE *newconfig;
	int32 result;

	DEBUG("CreateConfigInterface(%lu, 0x%p, %lu, 0x%p, 0x%p)\n",
	      channel, mux, configdata_count, configdata, config);

	if (!mux || !config)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	newconfig = alloc_mem(sizeof(CONFIGINTERFACE));

	newconfig->channel = channel;
	newconfig->configdata = configdata;
	newconfig->configdata_count = configdata_count;
	newconfig->mux = mux;
	newconfig->state = CONFIG_STATE_DEFAULT;

	result =
	    RegisterInterface(CONFIG_INTERFACE_NAME, &ConfigInform,
			      &ConfigReceive, (int32) newconfig,
			      mux->interface_lib);
	if (result != ERROR_NONE)
		goto REGISTER_CONFIGIF_FAILED;

	QueryInterfaceIndex(CONFIG_INTERFACE_NAME, mux->interface_lib,
			    &newconfig->host_interface);

	result = StartupConfigUserInterface(newconfig);
	if (result != ERROR_NONE)
		goto INIT_USERIF_FAILED;

	*config = newconfig;

	return DEBUGERROR(ERROR_NONE);

INIT_USERIF_FAILED:
	UnregisterInterface(newconfig->host_interface,
			    newconfig->mux->interface_lib);
REGISTER_CONFIGIF_FAILED:
	free_mem(newconfig);

	return DEBUGERROR(result);
}

/*
 * DestroyConfigInterface will free up any resources consumed by a
 * config interface object. This call depends on there being
 * absolutely no activity on the specified object.
 *
 * Params:
 * config -- the config object to be destroyed
 */
int32 DestroyConfigInterface(CONFIGINTERFACE *config)
{
	int32 result;

	DEBUG("DestroyConfigInterface(0x%p)\n", config);

	if (config->state != CONFIG_STATE_DEFAULT)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	ShutdownConfigUserInterface(config);

	result =
	    UnregisterInterface(config->host_interface,
				config->mux->interface_lib);
	if (result != ERROR_NONE)
		return DEBUGERROR(result);

	free_mem(config);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * ActivateConfigInterface is called when a NetMUx is to be initialized.
 * This is currently done once a linkdriver registers itself.
 *
 * Params:
 * host -- non-zero if the specified interface is to act as a host
 * config -- a pointer to the interface to be activated
 */
int32 ActivateConfigInterface(int32 host, CONFIGINTERFACE *config)
{
	DEBUG("ActivateConfigInterface(%lu, 0x%p)\n", host, config);

	config->state = CONFIG_STATE_DEFAULT;

	config->host = host;
	config->state = CONFIG_STATE_ENABLING;

	if (host)
		EnableMUX(host_end(COMMAND), config->mux);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * DeactivateconfigInterface is called when it's time to bringdown a
 * NetMUX. This is currently done when a linkdriver unregisters
 * itself.
 *
 * Params:
 * config -- the interface to be deactivated
 */
int32 DeactivateConfigInterface(CONFIGINTERFACE *config)
{
	DEBUG("DeactivateConfigInterface(0x%p)\n", config);

	config->state = CONFIG_STATE_DEFAULT;

	DisableMUX(host_end(COMMAND), config->mux);

	return DEBUGERROR(ERROR_NONE);
}

/* GenerateConfigPacket is a helper function for the config interface.
 * This function is called when configuration data needs to be delivered
 * to a participating NetMUX. The config data will be formed into a
 * packet and sent over the link.
 *
 * Params:
 * configdata -- pointer to the configuration data to be sent
 * config -- the config interface to deliver the data on
 */
int32 GenerateConfigPacket(CONFIGDATA *configdata,
			   CONFIGINTERFACE *config)
{
	CONFIGPACKET configpacket;
	COMMBUFF *commbuff;
	int32 result;
	int8 type;

	DEBUG("GenerateConfigPacket(0x%p, 0x%p)\n", configdata, config);

	if (config->state != CONFIG_STATE_CONNECTED)
		return DEBUGERROR(ERROR_OPERATIONFAILED);

	commbuff = alloc_commbuff(sizeof(CONFIGPACKET) + sizeof(int8),
				  sizeof(DATA_PACKET_HDR));

	type = CONFIG_CORE_TYPE;

	configpacket.channel = configdata->channel;
	configpacket.host_interface = configdata->host_interface_id;
	configpacket.client_interface = configdata->client_interface_id;
	configpacket.client_burstsize = configdata->client_burstsize;
	configpacket.client_maxdata = configdata->client_maxdata;
	configpacket.client_byte_credit = configdata->client_byte_credit;
	configpacket.client_send_credit = configdata->client_send_credit;
	configpacket.channel_extra = configdata->channel_extra;

	memcpy((void *) configpacket.channel_name,
	       configdata->channel_name, PACKET_MAXNAME_LENGTH);

	convert_dword(configpacket.channel);
	convert_dword(configpacket.host_interface);
	convert_dword(configpacket.client_interface);
	convert_dword(configpacket.client_burstsize);
	convert_dword(configpacket.client_maxdata);
	convert_dword(configpacket.client_byte_credit);
	convert_dword(configpacket.client_send_credit);
	convert_dword(configpacket.channel_extra);

	commbuff_copyin(commbuff, 0, (void *) &type, sizeof(int8));
	commbuff_copyin(commbuff, sizeof(int8), (void *) &configpacket,
			sizeof(CONFIGPACKET));

	result = SendData(config->channel, commbuff, NULL, config->mux);
	if (result != ERROR_NONE) {
		free_commbuff(commbuff);

		return DEBUGERROR(result);
	}

	deref_commbuff(commbuff);

	return DEBUGERROR(ERROR_NONE);
}


static
void GenerateStatusReport(CONFIGINTERFACE *config)
{
	CHANNEL **channels;
	CHANNELCREDIT *channelcredit;
	CHANNEL *channel;
	MUX *mux;
	USERCONFIGDATA *cdata;
	sint8 *newdata;
	int32 enabled_channels;
	int32 sendqueue_length;
	int32 receivequeue_length;
	int32 index;
	int32 remaining_space;
	int32 result;

	DEBUG("GenerateStatusReport(0x%p)\n", config);

	mux = config->mux;
	cdata = (USERCONFIGDATA *) config->user_interface_data;

	enter_write_criticalsection(&mux->lock);

	remaining_space = cdata->host_config_datasize;
	cdata->host_config_dataused = 0;
	enabled_channels = 0;
	result = 0;
	channels = mux->channels;
	receivequeue_length = queue_length(&mux->receive_queue);
	sendqueue_length = queue_length(&mux->send_queue);

	for (index = 0; index < mux->maxchannels; index++) {
		if (channels[index])
			enabled_channels++;
	}

	do {
		if (result) {
			newdata = alloc_mem(result + 1);

			memcpy(newdata, cdata->host_config_data,
			       cdata->host_config_dataused);
			free_mem(cdata->host_config_data);

			cdata->host_config_data = newdata;
			cdata->host_config_datasize = result + 1;
			remaining_space =
			    cdata->host_config_datasize -
			    cdata->host_config_dataused;
		}

		result =
		    (int32) snprintf(&cdata->
				     host_config_data[cdata->
						      host_config_dataused],
				     remaining_space,
				     "\n\nStatus of the MUX:\n"
				     "\tStatus flag value: %lu\n"
				     "\tTotal amount of data queued: %lu\n"
				     "\tNumber of channels enabled: %lu\n"
				     "\tLocal receive buffer size: %lu\n"
				     "\tRemote receive buffer size: %lu\n"
				     "\tSend buffers available: %lu\n"
				     "\tMUX Send Queue Status:\n"
				     "\t\tNumber of items queued: %lu\n"
				     "\tMux Receive Queue Status:\n"
				     "\t\tNumber of items queued: %lu\n"
				     "\tSend task state value: %u\n"
				     "\tReceive task state value: %u\n"
				     "\tShutdown task state value: %u\n"
				     "\tMUX Partial Receive Status:\n"
				     "\t\tType: %u\n"
				     "\t\tExpected length: %lu\n"
				     "\t\tCurrent length: %lu\n" "\n\n",
				     mux->status, mux->total_queued_amount,
				     enabled_channels,
				     mux->local_rcv_buffer_size,
				     mux->remote_rcv_buffer_size,
				     mux->send_buffers_available,
				     sendqueue_length, receivequeue_length,
				     mux->send_task.state,
				     mux->receive_task.state,
				     mux->shutdown_task.state,
				     mux->partial_receive.type,
				     mux->partial_receive.expectedLen,
				     mux->partial_receive.currentLen);
	} while (result >= remaining_space);

	cdata->host_config_dataused += result;
	remaining_space -= result;

	for (index = 0; index < mux->maxchannels; index++) {
		if (!channels[index])
			continue;

		result = 0;
		channel = channels[index];
		channelcredit = &mux->channelcredit[index];

		do {
			if (result) {
				newdata =
				    alloc_mem(cdata->host_config_dataused +
					      result + 1);

				memcpy(newdata, cdata->host_config_data,
				       cdata->host_config_dataused);
				free_mem(cdata->host_config_data);

				cdata->host_config_data = newdata;
				cdata->host_config_datasize =
				    cdata->host_config_dataused + result +
				    1;
				remaining_space =
				    cdata->host_config_datasize -
				    cdata->host_config_dataused;
			}

			receivequeue_length =
			    queue_length(&channel->receive_queue);
			sendqueue_length =
			    queue_length(&channel->send_queue);

			result =
			    (int32) snprintf(&cdata->
				     host_config_data[cdata->
						      host_config_dataused],
				     remaining_space,
				     "\n\nStatus of Channel %lu:\n"
				     "\tChannel Send Queue Status:\n"
				     "\t\tNumber of items queued: %lu\n"
				     "\tChannel Receive Queue Status:\n"
				     "\t\tNumber of items queued: %lu\n"
				     "\tBurst size: %lu\n"
				     "\tMax queued data: %lu\n"
				     "\tCurrent queued data: %lu\n"
				     "\tState: %lu\n"
				     "\tChannel Credit Status:\n"
				     "\t\tMax host byte credit: %lu\n"
				     "\t\tMax host send credit: %lu\n"
				     "\t\tClient byte credit: %lu\n"
				     "\t\tClient send credit: %lu\n"
				     "\t\tReplenished byte credit: %lu\n"
				     "\t\tReplenished send credit: %lu\n"
				     "\n\n", index,
				     sendqueue_length,
				     receivequeue_length,
				     channel->burst_size,
				     channel->max_data_amount,
				     channel->qed_data_amount,
				     channel->state,
				     channelcredit->
				     max_host_byte_credit,
				     channelcredit->
				     max_host_send_credit,
				     channelcredit->
				     client_byte_credit,
				     channelcredit->
				     client_send_credit,
				     channelcredit->
				     replenished_byte_credit,
				     channelcredit->
				     replenished_send_credit);
		} while (result >= remaining_space);

		cdata->host_config_dataused += result;
		remaining_space -= result;
	}

	cdata->host_config_dataused++;

	exit_write_criticalsection(&mux->lock);
}

static
void ProcessClientConfigRequest(int8 type, CONFIGINTERFACE *config,
				COMMBUFF *commbuff)
{
	USERCONFIG_STATUS statuspacket;
	USERCONFIG_ADJUSTCREDIT creditpacket;
	USERCONFIGDATA *cdata;
	CHANNELCREDIT *credit;
	MUX *mux;
	COMMBUFF *senddata;
	int32 size;
	int32 result;

	DEBUG("ProcessClientConfigRequest(%lu, 0x%p, 0x%p)\n",
	      (int32) type, config, commbuff);

	cdata = (USERCONFIGDATA *) config->user_interface_data;
	mux = config->mux;

	switch (type) {
	case CONFIG_USERIF_ADJUSTCREDIT:
		{
			commbuff_copyout((void *) &creditpacket, commbuff,
					 0,
					 sizeof(USERCONFIG_ADJUSTCREDIT));
			commbuff_remove_front(commbuff,
					      sizeof
					      (USERCONFIG_ADJUSTCREDIT));

			convert_dword(creditpacket.channel_valid);
			convert_dword(creditpacket.channel);
			convert_dword(creditpacket.max_host_byte_credit);
			convert_dword(creditpacket.max_host_send_credit);
			convert_dword(creditpacket.
				      replenished_byte_credit);
			convert_dword(creditpacket.
				      replenished_send_credit);
			convert_dword(creditpacket.client_byte_credit);
			convert_dword(creditpacket.client_send_credit);
			convert_dword(creditpacket.global_send_credit);

			enter_write_criticalsection(&mux->lock);

			if (creditpacket.channel_valid) {
				credit =
				    &mux->channelcredit[creditpacket.
							channel];

				credit->client_byte_credit +=
				    creditpacket.client_byte_credit;
				credit->client_send_credit +=
				    creditpacket.client_send_credit;
				credit->replenished_byte_credit +=
				    creditpacket.replenished_byte_credit;
				credit->replenished_send_credit +=
				    creditpacket.replenished_send_credit;
				credit->max_host_byte_credit +=
				    creditpacket.max_host_byte_credit;
				credit->max_host_send_credit +=
				    creditpacket.max_host_send_credit;
			}

			mux->send_buffers_available +=
			    creditpacket.global_send_credit;

			exit_write_criticalsection(&mux->lock);
		}
		break;

	case CONFIG_USERIF_STATUSREPORT:
		{
			commbuff_copyout((void *) &statuspacket, commbuff,
					 0, sizeof(USERCONFIG_STATUS));
			commbuff_remove_front(commbuff,
					      sizeof(USERCONFIG_STATUS));

			convert_dword(statuspacket.request);
			convert_dword(statuspacket.size);

			if (statuspacket.request) {
				enter_write_criticalsection(&cdata->
							    config_synch);

				GenerateStatusReport(config);

				size =
				    cdata->host_config_dataused +
				    sizeof(USERCONFIG_STATUS) +
				    sizeof(int8);
				senddata =
				    alloc_commbuff(size,
						   sizeof
						   (DATA_PACKET_HDR));

				statuspacket.request = 0;
				statuspacket.size =
				    cdata->host_config_dataused;

				convert_dword(statuspacket.request);
				convert_dword(statuspacket.size);

				type = CONFIG_USERIF_STATUSREPORT;

				commbuff_copyin(senddata, 0,
						(void *) &type,
						sizeof(int8));
				commbuff_copyin(senddata, sizeof(int8),
						(void *) &statuspacket,
						sizeof(USERCONFIG_STATUS)
				    );
				commbuff_copyin(senddata,
						sizeof(int8) +
						sizeof(USERCONFIG_STATUS),
						cdata->host_config_data,
						cdata->
						host_config_dataused);

				result =
				    SendData(config->channel, senddata, 0,
					     config->mux);
				if (result != ERROR_NONE)
					free_commbuff(senddata);

				exit_write_criticalsection(&cdata->
							   config_synch);
			} else {
				enter_write_criticalsection(&cdata->
							    config_synch);

				if (cdata->client_config_data) {
					free_mem(cdata->
						 client_config_data);

					cdata->client_config_data = 0;
					cdata->client_config_datasize = 0;
				}

				cdata->client_config_data =
				    int_alloc_mem(statuspacket.size);
				cdata->client_config_datasize =
				    statuspacket.size;

				commbuff_copyout(cdata->client_config_data,
						 commbuff, 0,
						 statuspacket.size);

				exit_write_criticalsection(&cdata->
							   config_synch);

				wake_up_interruptible(&cdata->event_wait);
			}
		}
		break;
	}
}

static void ProcessClientCommands(struct work_struct *work)
{
	CONFIGINTERFACE *config;
	USERCONFIGDATA *cdata;
	COMMBUFF *commanddata;
	int8 type;

	cdata = container_of(work, USERCONFIGDATA, commandtask.work);
	config = cdata->config;

	commanddata = dequeue_commbuff(&cdata->commandqueue);

	while (commanddata) {
		commbuff_copyout((void *) &type, commanddata, 0,
				 sizeof(int8));
		commbuff_remove_front(commanddata, sizeof(int8));

		ProcessClientConfigRequest(type, config, commanddata);
		free_commbuff(commanddata);

		commanddata = dequeue_commbuff(&cdata->commandqueue);
	}
}

int ReadConfiguredHostNetMUXStatus(char *destination,
				   char **start,
				   off_t offset,
				   int count, int *eof, void *param)
{
	CONFIGINTERFACE *config;
	USERCONFIGDATA *cdata;
	int bytes;

	DEBUG
	("ReadConfiguredHostNetMUXStatus(0x%p, 0x%p, %lu, %d, 0x%p, 0x%p)\n",
	destination, start, offset, count, eof, param);

	config = (CONFIGINTERFACE *) param;
	cdata = config->user_interface_data;

	enter_write_criticalsection(&cdata->config_synch);

	if (offset) {
		bytes = cdata->host_config_dataused - offset;
		if (bytes < 0) {
			*eof = 1;
			return 0;
		}

		if (bytes > count)
			bytes = count;

		memcpy(destination, &cdata->host_config_data[offset],
		       bytes);

		exit_write_criticalsection(&cdata->config_synch);

		*start = destination;

		return bytes;
	}

	GenerateStatusReport(config);

	bytes = cdata->host_config_dataused;
	if (bytes > count)
		bytes = count;

	memcpy(destination, cdata->host_config_data, bytes);

	exit_write_criticalsection(&cdata->config_synch);

	*start = destination;

	return bytes;
}

int ConfigureHostNetMUXStatus(struct file *fileid,
			      const char *data,
			      unsigned long count, void *param)
{
	char buffer[128] = { 0 };
	char command[128];
	USERCONFIG_ADJUSTCREDIT credit;
	CONFIGINTERFACE *config;
	USERCONFIGDATA *cdata;
	COMMBUFF *senddata;
	int32 size;
	int bytes;
	int input;

	DEBUG("ConfigureHostNetMUXStatus(0x%p, 0x%p, %lu, 0x%p)\n",
	      fileid, data, count, param);

	config = (CONFIGINTERFACE *) param;
	cdata = config->user_interface_data;

	if (count >= 128)
		return -EINVAL;

	bytes = copy_from_user(buffer, data, count);
	if (bytes)
		return -EFAULT;

	sscanf(buffer, "%s ", command);

	if (!strcmp(command, "credit")) {
		input = sscanf(buffer,
			       "%s %d %lu %lu %lu %lu %lu %lu %lu",
			       command,
			       (int *) &credit.channel,
			       &credit.max_host_byte_credit,
			       &credit.max_host_send_credit,
			       &credit.replenished_byte_credit,
			       &credit.replenished_send_credit,
			       &credit.client_byte_credit,
			       &credit.client_send_credit,
			       &credit.global_send_credit);
		if (input < 10)
			return -EINVAL;

		if (credit.channel == (int32) -1)
			credit.channel_valid = 0;
		else
			credit.channel_valid = 1;

		convert_dword(&credit.channel_valid);
		convert_dword(&credit.channel);
		convert_dword(&credit.max_host_byte_credit);
		convert_dword(&credit.max_host_send_credit);
		convert_dword(&credit.replenished_byte_credit);
		convert_dword(&credit.replenished_send_credit);
		convert_dword(&credit.client_byte_credit);
		convert_dword(&credit.client_send_credit);
		convert_dword(&credit.global_send_credit);

		size = sizeof(USERCONFIG_ADJUSTCREDIT);
		senddata = alloc_commbuff(size, sizeof(DATA_PACKET_HDR));

		commbuff_copyin(senddata,
				0,
				(void *) &credit,
				sizeof(USERCONFIG_ADJUSTCREDIT)
		    );

		ProcessClientConfigRequest(CONFIG_USERIF_ADJUSTCREDIT,
					   config, senddata);

		deref_commbuff(senddata);
	} else
		return -EINVAL;

	return count;
}

int ReadConfiguredClientNetMUXStatus(char *destination,
				     char **start,
				     off_t offset,
				     int count, int *eof, void *param)
{
	USERCONFIG_STATUS statuspacket;
	CONFIGINTERFACE *config;
	USERCONFIGDATA *cdata;
	COMMBUFF *senddata;
	int32 size;
	int32 result;
	int8 type;
	int bytes;

	DEBUG
	("ReadConfiguredClientNetMUXStatus(0x%p, 0x%p, %d, %d, 0x%p, 0x%p)\n",
	 destination, start, (int) offset, count, eof, param);

	config = (CONFIGINTERFACE *) param;
	cdata = config->user_interface_data;

	enter_write_criticalsection(&cdata->config_synch);

	if (offset) {
		bytes = cdata->client_config_datasize - offset;
		if (bytes < 0) {
			*eof = 1;

			return 0;
		}

		if (bytes > count)
			bytes = count;
		else
			*eof = 1;

		memcpy(destination, &cdata->client_config_data[offset],
		       bytes);

		exit_write_criticalsection(&cdata->config_synch);

		return bytes;
	}

	if (cdata->client_config_data) {
		free_mem(cdata->client_config_data);

		cdata->client_config_data = 0;
		cdata->client_config_datasize = 0;
	}

	size = sizeof(USERCONFIG_STATUS) + sizeof(int8);
	senddata = alloc_commbuff(size, sizeof(DATA_PACKET_HDR));

	statuspacket.request = 1;
	statuspacket.size = 0;

	convert_dword(statuspacket.request);
	convert_dword(statuspacket.size);

	type = CONFIG_USERIF_STATUSREPORT;

	commbuff_copyin(senddata, 0, (void *) &type, sizeof(int8));
	commbuff_copyin(senddata,
			sizeof(int8),
			(void *) &statuspacket, sizeof(USERCONFIG_STATUS)
	    );

	result = SendData(config->channel, senddata, 0, config->mux);
	if (result != ERROR_NONE) {
		exit_write_criticalsection(&cdata->config_synch);
		free_commbuff(senddata);

		return -EBUSY;
	}

	exit_write_criticalsection(&cdata->config_synch);
	interruptible_sleep_on_timeout(&cdata->event_wait, HZ * 10);
	enter_write_criticalsection(&cdata->config_synch);

	bytes = cdata->client_config_datasize;

	if (cdata->client_config_data) {
		if (bytes > count)
			bytes = count;
		else
			*eof = 1;

		memcpy(destination, cdata->client_config_data, bytes);
	}

	exit_write_criticalsection(&cdata->config_synch);

	return bytes;
}

int ConfigureClientNetMUXStatus(struct file *fileid,
				const char *data,
				unsigned long count, void *param)
{
	char buffer[128] = { 0 };
	char command[128];
	USERCONFIG_ADJUSTCREDIT credit;
	CONFIGINTERFACE *config;
	USERCONFIGDATA *cdata;
	COMMBUFF *senddata;
	int32 size;
	int32 type;
	int32 result;
	int bytes;
	int input;

	DEBUG("ConfigureClientNetMUXStatus(0x%p, 0x%p, %lu, 0x%p)\n",
	      fileid, data, count, param);

	config = (CONFIGINTERFACE *) param;
	cdata = config->user_interface_data;

	if (count >= 128)
		return -EINVAL;

	bytes = copy_from_user(buffer, data, count);
	if (bytes)
		return -EFAULT;

	sscanf(buffer, "%s ", command);

	if (!strcmp(command, "credit")) {
		input = sscanf(buffer,
			       "%s %d %lu %lu %lu %lu %lu %lu %lu",
			       command,
			       (int *) &credit.channel,
			       &credit.max_host_byte_credit,
			       &credit.max_host_send_credit,
			       &credit.replenished_byte_credit,
			       &credit.replenished_send_credit,
			       &credit.client_byte_credit,
			       &credit.client_send_credit,
			       &credit.global_send_credit);
		if (input < 10)
			return -EINVAL;

		if (credit.channel == (int32) -1)
			credit.channel_valid = 0;
		else
			credit.channel_valid = 1;

		convert_dword(&credit.channel_valid);
		convert_dword(&credit.channel);
		convert_dword(&credit.max_host_byte_credit);
		convert_dword(&credit.max_host_send_credit);
		convert_dword(&credit.replenished_byte_credit);
		convert_dword(&credit.replenished_send_credit);
		convert_dword(&credit.client_byte_credit);
		convert_dword(&credit.client_send_credit);
		convert_dword(&credit.global_send_credit);

		size = sizeof(USERCONFIG_ADJUSTCREDIT) + sizeof(int8);
		senddata = alloc_commbuff(size, sizeof(DATA_PACKET_HDR));

		type = CONFIG_USERIF_ADJUSTCREDIT;

		commbuff_copyin(senddata, 0, (void *) &type, sizeof(int8));
		commbuff_copyin(senddata,
				sizeof(int8),
				(void *) &credit,
				sizeof(USERCONFIG_ADJUSTCREDIT)
		    );

		result =
		    SendData(config->channel, senddata, 0, config->mux);
		if (result != ERROR_NONE) {
			free_commbuff(senddata);

			return -EBUSY;
		}
	} else
		return -EINVAL;

	return count;
}

int32 StartupConfigUserInterface(CONFIGINTERFACE *config)
{
	static int32 id = 0;
	sint8 host_entry_name[32];
	sint8 client_entry_name[32];
	USERCONFIGDATA *cdata;
	int32 size;

	DEBUG("StartupConfigUserInterface(0x%p)\n", config);

	sprintf(host_entry_name, "%s%lu", CONFIG_INTERFACE_NAME "_host",
		id);
	sprintf(client_entry_name, "%s%lu",
		CONFIG_INTERFACE_NAME "_client", id);

	cdata = alloc_mem(sizeof(USERCONFIGDATA));
	cdata->mux_id = id;

	id++;

	cdata->host_config_file =
	    create_proc_entry(host_entry_name, 0644, 0);
	if (!cdata->host_config_file)
		goto CREATE_HOSTPROCENTRY_FAILED;

	cdata->host_config_file->data = (void *) config;
	cdata->host_config_file->read_proc =
	    &ReadConfiguredHostNetMUXStatus;
	cdata->host_config_file->write_proc = &ConfigureHostNetMUXStatus;

	cdata->client_config_file =
	    create_proc_entry(client_entry_name, 0644, 0);
	if (!cdata->client_config_file)
		goto CREATE_CLIENTPROCENTRY_FAILED;

	cdata->client_config_file->data = (void *) config;
	cdata->client_config_file->read_proc =
	    &ReadConfiguredClientNetMUXStatus;
	cdata->client_config_file->write_proc =
	    &ConfigureClientNetMUXStatus;

	size = sizeof(sint8) * DEFAULT_CONFIG_DATASIZE;

	cdata->host_config_data = alloc_mem(size);
	cdata->client_config_data = 0;

	cdata->config = config;

	cdata->host_config_datasize = size;
	cdata->client_config_datasize = 0;
	cdata->host_config_dataused = 0;

	initialize_commbuff_queue(&cdata->commandqueue);
	initialize_criticalsection_lock(&cdata->config_synch);
	init_waitqueue_head(&cdata->event_wait);
	initialize_task(&cdata->commandtask, &ProcessClientCommands);

	config->user_interface_data = (void *) cdata;

	return DEBUGERROR(ERROR_NONE);

CREATE_CLIENTPROCENTRY_FAILED:
	remove_proc_entry(host_entry_name, 0);

CREATE_HOSTPROCENTRY_FAILED:
	return DEBUGERROR(ERROR_OPERATIONFAILED);
}

void ShutdownConfigUserInterface(CONFIGINTERFACE *config)
{
	sint8 host_entry_name[32];
	sint8 client_entry_name[32];
	USERCONFIGDATA *cdata;

	DEBUG("StartupConfigUserInterface(0x%p)\n", config);

	cdata = (USERCONFIGDATA *) config->user_interface_data;

	sprintf(host_entry_name, "%s%lu", CONFIG_INTERFACE_NAME "_host",
		cdata->mux_id);
	sprintf(client_entry_name, "%s%lu",
		CONFIG_INTERFACE_NAME "_client", cdata->mux_id);

	remove_proc_entry(host_entry_name, 0);
	remove_proc_entry(client_entry_name, 0);

	destroy_task(&cdata->commandtask);
	destroy_commbuff_queue(&cdata->commandqueue);
	destroy_criticalsection_lock(&cdata->config_synch);

	if (cdata->client_config_data)
		free_mem(cdata->client_config_data);

	free_mem(cdata->host_config_data);
	free_mem(cdata);
}

void ReceiveClientConfigRequest(CONFIGINTERFACE *config,
				COMMBUFF *commbuff)
{
	USERCONFIGDATA *cdata;

	DEBUG("ReceiveClientConfigRequest(0x%p, 0x%p)\n",
	      config, commbuff);

	cdata = (USERCONFIGDATA *) config->user_interface_data;

	queue_commbuff(commbuff, &cdata->commandqueue);
	task_schedule(&cdata->commandtask);
}
