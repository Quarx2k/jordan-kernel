/******************************************************************************
 * NetMUX tty.c                                                               *
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
 *   2006/11/15  Motorola    Fixed DestroyTTYInterface disable channel code   *
 *   2006/12/19  Motorola    Combined header and data into one transfer       *
 *   2007/01/15  Motorola    Fixed TTYWrite related code                      *
 *   2007/02/08  Motorola    Added mux_channel_queue_space to fix scheduling  *
 *                           while atomic bug                                 *
 *   2007/12/05  Motorola    change code as kernel upgrade                    *
 *   2007/10/25  Motorola    change code as kernel upgrade                    *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* tty.c is responsible for setting up a method of communication between the  */
/* NetMUX and user space applications.                                        */

#include "tty.h"
#include "protocol.h"
#include "config.h"
#include "debug.h"

extern struct class *netmux_class;

static const struct tty_operations tty_ops = {
	.open = TTYOpen,
	.close = TTYClose,
	.write = TTYWrite,
	.write_room = TTYWriteRoom,
	.chars_in_buffer = TTYDataInBuffer,
	.ioctl = TTYIOCtl,
};


/*
 * TTYInform is called by the mux (and sometimes the
 * config interface) to inform the interface that something
 * important has happened. The tty interface listens for
 * several things of importance.
 *
 * First, it listens for config packets. If a config packet
 * is recieved it processes it and sets up the appropriate
 * channel and entries in /dev/netmux.
 *
 * Second, it listens for a disable mux event. If such an
 * event happens all channels are forced closed and no
 * application can perform any operation on the channel until
 * all application accessing that device perform a close().
 *
 * Third, it waits for enable channel messages. If such a
 * message is ever received it immediately responds with a
 * failure because the direct interface does not support
 * external open requests.
 *
 * Fourth, it waits for a disable channel message. If
 * one is received the channel is placed into an inoperable
 * state until all applications accessing the device recognize
 * the closure by performing a close().
 *
 * Fifth, it waits for a data message, which signifies that the
 * channel has successfuly delivered data. The tty interface
 * uses this information to wakeup any event waiting to write
 * data to the channel. Also the mux send task is scheduled if
 * more data is ready to be sent.
 *
 * Sixth, it waits for a channel signal message. This signifies
 * a change to the modem flags and the signal will be processed
 * accordingly.
 *
 * Seventh, it waits for a prepare to send message. If one is received
 * the tty interface will attempt to send any data contained in any
 * of the tty channel queues.
 *
 * Seventh, it waits for a prepare to send message. If one is received
 * the tty interface will attempt to send any data contained in any
 * of the tty channel queues.
 *
 * Params:
 * param1 -- a custom pointer, in this case an INTERFACEINFORM struct
 * param2 -- a custom pointer, in this case a TTYINTERFACE struct
 */
int32 TTYInform(void *param1, void *param2)
{
	TTYINTERFACE *ttyif;
	struct tty_struct *tty;
	TTY_CHANNELDATA *chdat;
	CONFIGPACKET *configpacket;
	ENABLECHANNEL_PACKET *enablechannel;
	CHANNELSIGNAL_PACKET *channelsignal;
	INTERFACEINFORM *informdata;
	COMMBUFF *transmit;
	COMMBUFF *split;
	int32 channel;
	int32 type;
	int32 result;
	int16 xormask;
	int16 maskbase;
	int16 signal_maskbase;
	int16 signal_xormask;

	DEBUG("TTYInform(0x%p, 0x%p)\n", param1, param2);

	informdata = (INTERFACEINFORM *) param1;
	ttyif = (TTYINTERFACE *) param2;

	switch (informdata->inform_type) {
	case INFORM_INTERFACE_CONFIGPACKET:
		{
			channel =
			    ((CONFIGPACKET *) informdata->data)->channel;

			if (channel > ttyif->channel_max
			    || channel < ttyif->channel_min)
				return DEBUGERROR(ERROR_INVALIDPARAMETER);

			chdat =
			    &ttyif->channel_data[channel -
						 ttyif->channel_min];
			configpacket = (CONFIGPACKET *) informdata->data;

			chdat->commbuff = 0;
			chdat->client_interface =
			    configpacket->host_interface;
			chdat->burstsize = configpacket->client_burstsize;
			chdat->maxdata = configpacket->client_maxdata;
			chdat->host_byte_credit =
			    configpacket->client_byte_credit;
			chdat->host_send_credit =
			    configpacket->client_send_credit;
			chdat->modem_flags = TTY_DEFAULT_MODEM_FLAGS;
			chdat->data_amount = 0;
			chdat->mux_channel_queue_space =
			    configpacket->client_maxdata;

			initialize_commbuff_queue(&chdat->process_queue);

			memcpy(chdat->device_file,
			       configpacket->channel_name,
			       PACKET_MAXNAME_LENGTH);

			init_waitqueue_head(&chdat->event_wait);
			init_waitqueue_head(&chdat->close_wait);

			if (netmux_class) {
				if (IS_ERR
				    (device_create
				     (netmux_class, NULL,
				      MKDEV(ttyif->driver.major, channel),
				      NULL, "%s", chdat->device_file)))
					return
					    DEBUGERROR
					    (ERROR_OPERATIONFAILED);
			} else {
				DEBUGERROR(ERROR_INVALIDPARAMETER);
			}

			if (chdat->refcount == 0)
				chdat->state = TTY_STATE_READY;
		}
		break;

	case INFORM_INTERFACE_DISABLEMUX:
		{
			for (channel = 0;
			     channel <
			     ttyif->channel_max - ttyif->channel_min + 1;
			     channel++) {
				wake_up_interruptible(&ttyif->
						      channel_data
						      [channel].
						      event_wait);

				ttyif->channel_data[channel].state =
				    TTY_STATE_DEFAULT;
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
					      ttyif->mux);
			} else if (type == SUCCESS) {
				channel = enablechannel->channel;
				chdat =
				    &ttyif->channel_data[channel -
							 ttyif->
							 channel_min];

				chdat->refcount++;
				chdat->state |=
				    TTY_STATE_EVENT |
				    TTY_STATE_EVENT_SUCCESS;

				wake_up_interruptible(&chdat->event_wait);
			} else if (type == FAILURE) {
				channel = enablechannel->channel;
				chdat =
				    &ttyif->channel_data[channel -
							 ttyif->
							 channel_min];

				chdat->state |= TTY_STATE_EVENT;

				wake_up_interruptible(&chdat->event_wait);
			}
		}
		break;

	case INFORM_INTERFACE_DISABLECHANNEL:
		{
			channel =
			    ((DISABLECHANNEL_PACKET *) informdata->data)->
			    channel;
			chdat =
			    &ttyif->channel_data[channel -
						 ttyif->channel_min];

			chdat->refcount--;
			wake_up_interruptible(&chdat->close_wait);
		}
		break;

	case INFORM_INTERFACE_CHANNELSIGNAL:
		{
			channelsignal =
			    (CHANNELSIGNAL_PACKET *) informdata->data;
			channel = channelsignal->channel;

			enter_write_criticalsection(&ttyif->lock);

			chdat =
			    &ttyif->channel_data[channel -
						 ttyif->channel_min];

			signal_maskbase = channelsignal->signal >> 16;
			signal_xormask = channelsignal->signal & 0xFFFF;
			maskbase = chdat->modem_flags & 0xFFFF;
			xormask = maskbase ^ signal_maskbase;
			xormask = xormask ^ signal_xormask;

			chdat->modem_flags = maskbase ^ xormask;

			exit_write_criticalsection(&ttyif->lock);

			wake_up_interruptible(&chdat->event_wait);
		}
		break;

	case INFORM_INTERFACE_PREPSEND:
		{
			for (channel = 0; channel < ttyif->channel_max -
			     ttyif->channel_min + 1; channel++) {
				chdat = &ttyif->channel_data[channel];
				enter_write_criticalsection(&ttyif->lock);

				if (chdat->refcount
				    && queue_length(&chdat->
						    process_queue)) {
					do {
						transmit =
						    dequeue_commbuff
						    (&chdat->
						     process_queue);
						chdat->data_amount -=
						    commbuff_length
						    (transmit);
						chdat->
						    mux_channel_queue_space
						    -=
						    commbuff_length
						    (transmit);

						result =
						    SendData(channel +
							     ttyif->
							     channel_min,
							     transmit,
							     &split,
							     ttyif->mux);

						if (result ==
						    ERROR_INCOMPLETE) {
							queuefront_commbuff
							    (split,
							     &chdat->
							     process_queue);
							chdat->
							    data_amount +=
							    commbuff_length
							    (split);
							chdat->
							mux_channel_queue_space
							 +=
							 commbuff_length
							 (split);
						}
					} while (((result == ERROR_NONE) ||
						  (result ==
						   ERROR_INCOMPLETE))
						 && queue_length(&chdat->
							 process_queue));

					if ((result != ERROR_NONE)
					    && (result !=
						ERROR_INCOMPLETE)) {
						queuefront_commbuff
						    (transmit,
						     &chdat->
						     process_queue);
						chdat->data_amount +=
						    commbuff_length
						    (transmit);
						chdat->
						    mux_channel_queue_space
						    +=
						    commbuff_length
						    (transmit);
					}
				}

				exit_write_criticalsection(&ttyif->lock);
			}
		}
		break;

	case INFORM_INTERFACE_DATA:
		{
			channel = (int32) (informdata->data);

			result = SendDataAvailable(channel, ttyif->mux);

			enter_write_criticalsection(&ttyif->lock);

			chdat =
			    &ttyif->channel_data[channel -
						 ttyif->channel_min];
			chdat->mux_channel_queue_space = result;

			exit_write_criticalsection(&ttyif->lock);

			tty =
			    ttyif->driver.ttys[channel -
					       ttyif->channel_min];
			if (tty->ldisc->ops->flags & TTY_DO_WRITE_WAKEUP)
				tty->ldisc->ops->write_wakeup(tty);

			if (queue_length(&chdat->process_queue))
				RunSend(ttyif->mux);
		}
		break;

	default:
		break;
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * TTYReceive is called when a buffer is available for
 * delivery to an application. The tty interface will
 * attempt to deliver the buffer to the tty layer,
 * however, if that is not possible the interface will
 * poll and try to deliver it again later.
 *
 * Params:
 * commbuff -- a pointer to the received data
 * param -- a custom pointer, in this case an INTERFACEINFORM struct
 */
int32 TTYReceive(COMMBUFF *commbuff, void *param)
{
	struct tty_struct *tty;
	TTYINTERFACE *ttyif;
	INTERFACEINFORM *inform_data;
	int32 channel;

	DEBUG("TTYReceive(0x%p, 0x%p)\n", commbuff, param);

	inform_data = (INTERFACEINFORM *) param;
	ttyif = (TTYINTERFACE *) inform_data->inform_type;
	channel = (int32) inform_data->data;
	tty = ttyif->driver.ttys[channel - ttyif->channel_min];

	if (commbuff_length(commbuff) <= tty->receive_room)
		(tty->ldisc->ops->receive_buf) (tty,
						commbuff_data(commbuff), 0,
						commbuff_length(commbuff));
	else
		return DEBUGERROR(ERROR_INCOMPLETE);

	LOGCOMMBUFF_CH(channel, "TTYReceive()-->", commbuff,
		       commbuff_length(commbuff));

	free_commbuff(commbuff);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * CreateTTYInterface will create an interface object to be used by
 * a NetMUX. This object will keep track of all settings required to
 * communicate with the tty layer. As such, any tty operation
 * will make reference to this interface object.
 *
 * Params:
 * name -- the name of the interface
 * path -- the path to create device entries in
 * major -- the major number for the driver, 0 if it's to be dynamic
 * channel_min -- the inclusive lower bound of channel numbers
 * 		assigned to the interface
 * channel_max -- the inclusive upper bound of channel numbers
 * 		assigned to the interface
 * mux -- the mux object this interface is associated with
 * tty -- a pointer to a pointer to receive the newly created object
 */
int32 CreateTTYInterface(sint8 *name, sint8 *path, int32 major,
			 int32 channel_min, int32 channel_max, MUX *mux,
			 TTYINTERFACE **tty)
{
	TTYINTERFACE *newtty;
	TTY_CHANNELDATA *channel_data;
	int32 result;
	int32 namesize;
	int32 pathsize;
	int32 size;

	DEBUG
	    ("CreateTTYInterface(0x%p, 0x%p, %lu, %lu, %lu, 0x%p, 0x%p)\n",
	     name, path, major, channel_min, channel_max, mux, tty);

	if (!tty || !name || !path || !mux)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	if (channel_min > channel_max)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	if (channel_max >= mux->maxchannels)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	size = (channel_max - channel_min + 1) * sizeof(TTY_CHANNELDATA);

	namesize = strlen(name) + 1;
	if (namesize > PACKET_MAXNAME_LENGTH)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	pathsize = strlen(path) + 1;
	if (pathsize > PACKET_MAXNAME_LENGTH)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	channel_data = (TTY_CHANNELDATA *) alloc_mem(size);
	newtty = (TTYINTERFACE *) alloc_mem(sizeof(TTYINTERFACE));

	memset(newtty, 0, sizeof(TTYINTERFACE));

	result =
	    RegisterInterface(name, &TTYInform, &TTYReceive,
			      (int32) newtty, mux->interface_lib);
	if (result != ERROR_NONE) {
		free_mem(newtty);
		free_mem(channel_data);

		return DEBUGERROR(result);
	}

	QueryInterfaceIndex(name, mux->interface_lib,
			    &newtty->host_interface);

	newtty->driver.magic = TTY_DRIVER_MAGIC;
	newtty->driver.driver_name = "netmux";
	newtty->driver.name = "netmux";
	newtty->driver.major = TTY_DYNAMIC_MAJOR_ASSIGNMENT;
	newtty->driver.minor_start = channel_min;
	newtty->driver.num = channel_max - channel_min + 1;
	newtty->driver.type = TTY_DRIVER_TYPE_SERIAL;
	newtty->driver.subtype = SERIAL_TYPE_NORMAL;
	newtty->driver.init_termios = tty_std_termios;
	newtty->driver.init_termios.c_iflag = 0;
	newtty->driver.init_termios.c_oflag = 0;
	newtty->driver.init_termios.c_cflag = B38400 | CS8 | CREAD;
	newtty->driver.init_termios.c_lflag = 0;
	newtty->driver.flags =
	    TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW |
	    TTY_DRIVER_DYNAMIC_DEV;
	newtty->driver.driver_state = (struct tty_driver *) newtty;

	tty_set_operations(&newtty->driver, &tty_ops);
	result = tty_register_driver(&newtty->driver);
	if (result < 0) {
		UnregisterInterface(newtty->host_interface,
				    mux->interface_lib);

		free_mem(channel_data);
		free_mem(newtty);

		return DEBUGERROR(ERROR_OPERATIONFAILED);
	}

	newtty->mux = mux;
	newtty->channel_min = channel_min;
	newtty->channel_max = channel_max;
	newtty->channel_data = channel_data;

	memcpy(newtty->device_directory, path, pathsize);
	memcpy(newtty->interface_name, name, namesize);
	memset(channel_data, 0, size);

	initialize_criticalsection_lock(&newtty->lock);

	*tty = newtty;

	return DEBUGERROR(ERROR_NONE);
}

/*
 * DestroyTTYInterface is invoked when a NetMUX is being destroyed.
 * The interface object will be 'undone'.
 *
 * Params:
 * tty -- the interface to be destroyed
 */
int32 DestroyTTYInterface(TTYINTERFACE *tty)
{
	TTY_CHANNELDATA *chdat;
	int32 channels;
	int32 index;

	DEBUG("DestroyTTYInterface(0x%p)\n", tty);

	chdat = tty->channel_data;
	channels = tty->channel_max - tty->channel_min + 1;

	for (index = 0; index < channels; index++) {
		if (chdat[index].state & TTY_STATE_READY) {
			DisableChannel(host_end(COMMAND),
				       index + tty->channel_min,
				       tty->host_interface,
				       chdat[index].client_interface,
				       tty->mux);

			if (chdat[index].refcount)
				wait_event_interruptible(chdat[index].
							 close_wait,
							 !(chdat[index].
							   refcount));

			device_destroy(netmux_class,
				       MKDEV(tty->driver.major, index));
		}
	}

	tty_unregister_driver(&tty->driver);
	UnregisterInterface(tty->host_interface, tty->mux->interface_lib);
	destroy_criticalsection_lock(&tty->lock);

	free_mem(tty->channel_data);
	free_mem(tty);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * TTYOpen is called by the linux file system whenever an open
 * is performed on a device belonging to this interface. The open
 * will send an enable request to a client netmux and wait for a
 * reply. If any errors occurr the device will not be open.
 *
 * Params:
 * tty-- used to let us fetch the major/minor of the device
 * filp -- some private data is set within this structure
 */
int TTYOpen(struct tty_struct *tty, struct file *filp)
{
	TTYINTERFACE *ttyif;
	TTY_CHANNELDATA *chdat;
	int32 minor;
	int32 result;

	DEBUG("TTYOpen(0x%p, 0x%p)\n", tty, filp);

	ttyif = (TTYINTERFACE *) tty->driver->driver_state;
	minor = tty->index + ttyif->channel_min;

	if (minor < ttyif->channel_min || minor > ttyif->channel_max)
		return -EADDRNOTAVAIL;

	chdat = &ttyif->channel_data[minor - ttyif->channel_min];
	if (!(chdat->state & TTY_STATE_READY))
		return -EADDRNOTAVAIL;

	if (chdat->refcount > 0)
		return -EBUSY;

	result = EnableChannel(host_end(COMMAND),
			       minor,
			       chdat->burstsize,
			       chdat->maxdata,
			       ttyif->host_interface,
			       chdat->client_interface,
			       chdat->host_byte_credit,
			       chdat->host_send_credit, 0, 0, ttyif->mux);

	if (result != ERROR_NONE)
		return -ECONNABORTED;

	/* Wake up when either one of two conditions occur:
	   1 - A response is received from the BP for our open request.
	   2 - The channel is not configured and running properly.
	   In case the latter is true an error will be returned eventually. */
	wait_event_interruptible(chdat->event_wait,
				 (chdat->state & TTY_STATE_EVENT) ||
				 !(chdat->state & TTY_STATE_READY));

	chdat->state &= ~TTY_STATE_EVENT;

	if (!(chdat->state & TTY_STATE_EVENT_SUCCESS))
		return -ECONNREFUSED;

	chdat->state &= ~TTY_STATE_EVENT_SUCCESS;

	ttyif->driver.ttys[minor - ttyif->channel_min] = tty;

	return 0;
}

/*
 * TTYClose is called by the file system whenever a close is
 * performed on an open device. A disable is sent to the client
 * NetMUX but no response is expected or required. If there is
 * an error with the device being closed an error will be
 * returned.
 *
 * Params:
 * tty -- used to fetch the minor number of the device
 * filp -- stores some private data which we use to fetch the tty
 *         interface structure
 */
void TTYClose(struct tty_struct *tty, struct file *filp)
{
	TTYINTERFACE *ttyif;
	TTY_CHANNELDATA *chdat;
	int32 minor;
	int32 result;

	DEBUG("TTYClose(0x%p, 0x%p)\n", tty, filp);

	ttyif = (TTYINTERFACE *) tty->driver->driver_state;
	minor = tty->index + ttyif->channel_min;

	if (minor < ttyif->channel_min || minor > ttyif->channel_max)
		return;

	chdat = &ttyif->channel_data[minor - ttyif->channel_min];

	if (chdat->refcount) {
		result =
		    DisableChannel(host_end(COMMAND), minor,
				   ttyif->host_interface,
				   chdat->client_interface, ttyif->mux);
		if (result != ERROR_NONE) {
			ttyif->driver.ttys[minor - ttyif->channel_min] = 0;

			return;
		}

		chdat->state = TTY_STATE_READY;
		wait_event_interruptible(chdat->close_wait,
					 !chdat->refcount);
	} else {
		(void) DisableChannel(host_end(COMMAND), minor,
				      ttyif->host_interface,
				      chdat->client_interface, ttyif->mux);
	}

	return;
}

/*
 * TTYWrite is called whenever the user tries to write data
 * to a device. If the device cannot take the data or there
 * is some other error the user will be notified. The data
 * to be delivered is copied out of the user supplied buffer
 * and into an allocated commbuff.
 *
 * Params:
 * tty -- used to get the direct interface and the minor number
 * buf -- a pointer to the data to be copied
 * count -- the number of bytes to be copied
 */
ssize_t TTYWrite(struct tty_struct *tty, const unsigned char *buf,
		 int count)
{
	COMMBUFF *commbuff;
	TTYINTERFACE *ttyif;
	TTY_CHANNELDATA *chdat;
	int32 minor;
	sint32 amount_written;

	DEBUG("TTYWrite(0x%p, 0x%p, %d)\n", tty, buf, count);

	/* This function can be called in an atomic context so we shouldn't call
	   any APIs that can sleep */

	ttyif = (TTYINTERFACE *) tty->driver->driver_state;
	minor = tty->index + ttyif->channel_min;

	chdat = &ttyif->channel_data[minor - ttyif->channel_min];
	if (!(chdat->state & TTY_STATE_READY) || chdat->refcount == 0)
		return -ENOTCONN;

	/* In some instances, we might add more data to the NetMUX
	 * queue than the burst size for the channel. Also,
	 * the amount of combined data in the two queues
	 * may surpass the channel queue size. */
	amount_written =
	    chdat->mux_channel_queue_space - chdat->data_amount;

	if (count < amount_written)
		amount_written = count;

	if (amount_written > 0) {
		commbuff =
		    alloc_commbuff(amount_written,
				   sizeof(DATA_PACKET_HDR));
		memcpy(commbuff_data(commbuff), buf, amount_written);
		queue_commbuff(commbuff, &chdat->process_queue);
		chdat->data_amount += commbuff_length(commbuff);
		RunSend(ttyif->mux);
	} else {
		amount_written = 0;
	}

	return amount_written;
}

/*
 * TTYWriteRoom is used to determine if there is any available
 * space in the write buffers. The value returned is the
 * number of bytes available.
 *
 * Params:
 * tty -- used to get the devices minor number
 */
int TTYWriteRoom(struct tty_struct *tty)
{
	TTYINTERFACE *ttyif;
	TTY_CHANNELDATA *chdat;
	int32 minor;
	sint32 result;

	DEBUG("TTYWriteRoom(0x%p)\n", tty);

	ttyif = (TTYINTERFACE *) tty->driver->driver_state;
	minor = tty->index + ttyif->channel_min;

	chdat = &ttyif->channel_data[minor - ttyif->channel_min];
	if (!(chdat->state & TTY_STATE_READY) || chdat->refcount == 0)
		return -ENOTCONN;

	result = chdat->mux_channel_queue_space - chdat->data_amount;

	return (result < 0 ? 0 : result);
}

/*
 * TTYDataInBuffer is used to determine if there is any
 * data available in the read buffers to be read by a
 * device.
 *
 * Params:
 * tty -- used to get the devices minor number
 */
int TTYDataInBuffer(struct tty_struct *tty)
{
	TTYINTERFACE *ttyif;
	TTY_CHANNELDATA *chdat;
	int32 minor;
	int32 result;

	DEBUG("TTYDataInBuffer(0x%p)\n", tty);

	ttyif = (TTYINTERFACE *) tty->driver->driver_state;
	minor = tty->index + ttyif->channel_min;

	chdat = &ttyif->channel_data[minor - ttyif->channel_min];
	if (!(chdat->state & TTY_STATE_READY) || chdat->refcount == 0)
		return -ENOTCONN;

	result = ReadDataAvailable(minor, ttyif->mux);

	return result;
}

/*
 * TTYIOCtl is used to get and set the modem flags of a
 * particular device. Any application can modify the
 * modem flags and expect the result to be delivered to
 * the client NetMUX. The user can also tell the function
 * to block until a certain modem signal changed.
 *
 * Params:
 * tty -- used to get the devices minor number
 * cmd -- the IOCtl type
 * arg -- the argument to the IOCtl
 */
int TTYIOCtl(struct tty_struct *tty, struct file *file, unsigned int cmd,
	     unsigned long arg)
{
	wait_queue_head_t *queue;
	TTYINTERFACE *ttyif;
	TTY_CHANNELDATA *chdat;
	int32 minor;
	int16 newflags;
	int16 xormask;
	int32 signal;
	int32 change;
	int32 original_flags;
	int32 wait_flags;

	DEBUG("TTYIOCtl(0x%p, 0x%p, %d, %lu)\n", tty, file, cmd, arg);

	ttyif = (TTYINTERFACE *) tty->driver->driver_state;
	minor = tty->index + ttyif->channel_min;

	chdat = &ttyif->channel_data[minor - ttyif->channel_min];
	if (!(chdat->state & TTY_STATE_READY) || chdat->refcount == 0)
		return -ENOTCONN;

	switch (cmd) {
	case TIOCMGET:
		{
			original_flags = chdat->modem_flags;

			if (copy_to_user
			    ((unsigned int *) arg, &original_flags,
			     sizeof(int16)))
				return -EFAULT;

			return 0;
		}
		break;

	case TIOCMSET:
		{
			if (copy_from_user
			    (&newflags, (unsigned int *) arg,
			     sizeof(int16)))
				return -EFAULT;

			enter_write_criticalsection(&ttyif->lock);

			xormask = newflags ^ chdat->modem_flags;
			signal = (chdat->modem_flags << 16) | (xormask);
			chdat->modem_flags ^= xormask;

			exit_write_criticalsection(&ttyif->lock);

			ChannelSignal(host_end(COMMAND), signal, minor,
				      ttyif->mux);

			return 0;
		}
		break;

	case TIOCMIWAIT:
		{
			change = 0;
			original_flags = chdat->modem_flags;
			queue = &chdat->event_wait;

			if (copy_from_user
			    (&wait_flags, (unsigned int *) arg,
			     sizeof(unsigned int)))
				return -EFAULT;

			if (!wait_flags)
				return 0;

			while (!(wait_flags & change)) {
				interruptible_sleep_on(queue);

				change =
				    chdat->modem_flags ^ original_flags;
			}

			return 0;
		}
		break;
	}

	return -ENOIOCTLCMD;
}
