/******************************************************************************
 * NetMUX direct.c                                                            *
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
 *   2006/11/15  Motorola    Fixed DestroyDirectInterface disable channel code*
 *   2006/11/18  Motorola    Fixed CreateDirectInterface() such that memory   *
 *                           is now being allocated, while interrupts are not *
 *                           disabled.                                        *
 *   2006/12/19  Motorola    Combine header and data into one transfer        *
 *   2007/05/01  Motorola    Change codes to ensure "shared" netmux           *
 *                           code is identical between AP and BP.             *
 *   2007/12/05  Motorola    port to kernel 2.6.22 for OMAP3430               *
 *   2008/07/09  Motorola    port to kernel 2.6.24 for TI 23.5                *
 *   2008/10/25  Motorola    update  kernel to TI 25.1                        *
 *   2009/08/13  Motorola    Remove wait in DirectClose()                     *
 *   2009/10/02  Motorola    Replace LOGCOMMBUFF with DEBUG                   *
 *   2010/04/28  Motorola    Format cleanup                                   *
 *   2010/08/12  Motorola    Klocwork Fix
 ******************************************************************************/

/* direct.c defines an interface between a NetMUX and the Linux raw character */
/* device layer.  This code is responsible for assisting application in       */
/* communication with the NetMUX by implementing the standard file operations */
/* such as open(), close(), read(), and write().                              */

#include "direct.h"
#include "protocol.h"
#include "config.h"
#include "debug.h"
#include <linux/sched.h>

//#define DEBUG_NETMUX_AUDIO

/*
 * major_list keeps track of a list of major numbers
 * associated with a NetMUX. Each item in the list
 * contains information to be associated with the
 * major number.
 */
DIRECT_MAJOR_LIST *major_list = 0;

extern struct class *netmux_class;
#ifdef DEBUG_NETMUX_AUDIO
static void dump_hex(char *buf, int size)
{
    int i = 0 ;
    unsigned char c;
    unsigned int tmp;
    char buffer[257];

    for(; i < (size * 2) && i < ((sizeof(buffer) - 1));)
    {
        //printk(KERN_INFO "(%s), i[%d], buf[0x%2x]\n", __func__, i, *buf);
        c = *(buf++);
        tmp = ((c & 0xF0) >> 4) & 0x0F;
        if (tmp > 9)
            buffer[i++] = 'A' + (tmp - 10);
        else
            buffer[i++] = '0' + tmp;

        tmp = (c & 0x0F);
        if (tmp > 9)
            buffer[i++] = 'A' + (tmp - 10);
        else
            buffer[i++] = '0' + tmp;
    }
    buffer[i] = 0;
    printk(KERN_INFO "(%s): buffer[%s]\n", __func__, buffer);
}
#endif
/*
 * DirectInform is called by the mux (and sometimes the
 * config interface) to inform the interface that something
 * important has happened. The direct interface listens for
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
 * Fifth, it waits a data message, which signifies that the
 * channel has successfuly delivered data. The direct interface
 * uses this information to wakeup any event waiting to write
 * data to the channel.
 *
 * Params:
 * param1 -- a custom pointer, in this case an INTERFACEINFORM struct
 * param2 -- a custom pointer, in this case a DIRECTINTERFACE struct
 */
int32 DirectInform(void *param1, void *param2)
{
	DIRECTINTERFACE *direct;
	DIRECT_CHANNELDATA *chdat;
	CONFIGPACKET *configpacket;
	ENABLECHANNEL_PACKET *enablechannel;
	INTERFACEINFORM *informdata;
	int32 channel;
	int32 type;
#ifdef DEBUG_NETMUX_AUDIO
	//printk(KERN_INFO "(%s): DirectInform(0x%p, 0x%p)\n", __func__, param1, param2);
#endif
	informdata = (INTERFACEINFORM *) param1;
	direct = (DIRECTINTERFACE *) param2;

	switch (informdata->inform_type) {
	case INFORM_INTERFACE_CONFIGPACKET:
		{
			channel =
			    ((CONFIGPACKET *) informdata->data)->channel;

			if (channel > direct->channel_max
			    || channel < direct->channel_min) {
				return DEBUGERROR(ERROR_INVALIDPARAMETER);
			}

			chdat =
			    &direct->channel_data[channel -
						  direct->channel_min];
			configpacket = (CONFIGPACKET *) informdata->data;

			chdat->client_interface =
			    configpacket->host_interface;
			chdat->burstsize = configpacket->client_burstsize;
			chdat->maxdata = configpacket->client_maxdata;
			chdat->host_byte_credit =
			    configpacket->client_byte_credit;
			chdat->host_send_credit =
			    configpacket->client_send_credit;

			memcpy(chdat->device_file,
			       configpacket->channel_name,
			       PACKET_MAXNAME_LENGTH);
#ifdef DEBUG_NETMUX_AUDIO
			printk(KERN_INFO "(%s): DirectInform(0x%p, 0x%p), device_file[%s]\n", __func__, param1, param2, chdat->device_file);
#endif
			init_waitqueue_head(&chdat->event_wait);
			init_waitqueue_head(&chdat->close_wait);
			init_waitqueue_head(&chdat->rdevent);
			init_waitqueue_head(&chdat->wrevent);

			if (netmux_class) {
				if (IS_ERR
				    (device_create
				     (netmux_class, NULL,
				      MKDEV(direct->major, channel), NULL,
				      "%s", chdat->device_file)))
					return
					    DEBUGERROR
					    (ERROR_OPERATIONFAILED);
			} else {
				DEBUGERROR(ERROR_INVALIDPARAMETER);
			}

			if (chdat->refcount == 0)
				chdat->state = DIRECT_STATE_READY;
		}
		break;

	case INFORM_INTERFACE_DISABLEMUX:
		{
			for (channel = 0;
			     channel <
			     direct->channel_max - direct->channel_min + 1;
			     channel++) {
				wake_up_interruptible(&direct->
						      channel_data
						      [channel].
						      event_wait);

				direct->channel_data[channel].state =
				    DIRECT_STATE_DEFAULT;
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
					      direct->mux);
			} else if (type == SUCCESS) {
				channel = enablechannel->channel;
				chdat =
				    &direct->channel_data[channel -
							  direct->
							  channel_min];

				chdat->refcount++;
				chdat->state |=
				    DIRECT_STATE_EVENT |
				    DIRECT_STATE_EVENT_SUCCESS;

				wake_up_interruptible(&chdat->event_wait);
			} else if (type == FAILURE) {
				channel = enablechannel->channel;
				chdat =
				    &direct->channel_data[channel -
							  direct->
							  channel_min];

				chdat->state |= DIRECT_STATE_EVENT;

				wake_up_interruptible(&chdat->event_wait);
			}
		}
		break;

	case INFORM_INTERFACE_DISABLECHANNEL:
		{
			printk
			    (KERN_INFO "DisableChannel: ackType = %x,   \
			     channel = %x, host = %x, client = %x \n",
			     ((DISABLECHANNEL_PACKET *) informdata->data)->
			     acktype,
			     ((DISABLECHANNEL_PACKET *) informdata->data)->
			     channel,
			     ((DISABLECHANNEL_PACKET *) informdata->data)->
			     host_interface,
			     ((DISABLECHANNEL_PACKET *) informdata->data)->
			     client_interface);
			channel =
			    ((DISABLECHANNEL_PACKET *) informdata->data)->
			    channel;
			chdat =
			    &direct->channel_data[channel -
						  direct->channel_min];

			chdat->refcount--;
			wake_up_interruptible(&chdat->close_wait);
		}
		break;

	case INFORM_INTERFACE_DATA:
		{
			channel = (int32) informdata->data;

			wake_up_interruptible(&direct->
					      channel_data[channel -
							   direct->
							   channel_min].
					      wrevent);
		}
		break;

	default:
		break;
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * DirectReceive is called when a buffer is available for
 * delivery to an application. The direct interface can not
 * force a buffer onto an application, so this function always
 * returns an error so the mux knows to stop trying to deliver
 * the buffer. The buffer will eventually be pulled from the
 * mux as soon as a user requests the data.
 *
 * Params:
 * commbuff -- a pointer to the received data
 * param -- a custom pointer, in this case an INTERFACEINFORM struct
 */
int32 DirectReceive(COMMBUFF *commbuff, void *param)
{
	DIRECTINTERFACE *direct;
	INTERFACEINFORM *inform_data;
	int32 channel;

	DEBUG("DirectReceive(0x%p, 0x%p)\n", commbuff, param);

	inform_data = (INTERFACEINFORM *) param;
	direct = (DIRECTINTERFACE *) inform_data->inform_type;
	channel = (int32) inform_data->data;

	wake_up_interruptible(&direct->
			      channel_data[channel -
					   direct->channel_min].rdevent);

	return DEBUGERROR(ERROR_OPERATIONRESTRICTED);
}

/*
 * CreateDirectInterface will create an interface object to be used by
 * a NetMUX. This object will keep track of all settings required to
 * communicate with the raw device layer. As such, any direct operation
 * will make reference to this interface object.
 *
 * Params:
 * name -- the name of the interface
 * path -- the path to create device entries in
 * major -- the major number for the driver, 0 if it's to be dynamic
 * channel_min -- the inclusive lower bound of channel numbers
 * 	 assigned to the interface
 * channel_max -- the inclusive upper bound of channel numbers
 * 	 assigned to the interface
 * mux -- the mux object this interface is associated with
 * direct -- a pointer to a pointer to receive the newly created object
 */
int32 CreateDirectInterface(sint8 *name, sint8 *path, int32 major,
			    int32 channel_min, int32 channel_max,
			    MUX *mux, DIRECTINTERFACE **direct)
{
	DIRECT_MAJOR_LIST **browse;
	DIRECT_MAJOR_LIST *node;
	DIRECTINTERFACE *newdirect;
	DIRECT_CHANNELDATA *chdat;
	INTERRUPT_STATE state;
	int32 result;
	int32 namesize;
	int32 pathsize;
	int32 size;

	DEBUG("CreateDirectInterface(0x%p, %lu, %lu, %lu, 0x%p, 0x%p)\n",
	      name, major, channel_min, channel_max, mux, direct);

	if (!direct || !name || !mux)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	if (channel_min > channel_max)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	if (channel_max >= mux->maxchannels)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	size =
	    (channel_max - channel_min + 1) * sizeof(DIRECT_CHANNELDATA);

	namesize = strlen(name) + 1;
	if (namesize > PACKET_MAXNAME_LENGTH)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	pathsize = strlen(path) + 1;
	if (pathsize > PACKET_MAXNAME_LENGTH)
		return DEBUGERROR(ERROR_OPERATIONRESTRICTED);

	chdat = (DIRECT_CHANNELDATA *) alloc_mem(size);
	newdirect = (DIRECTINTERFACE *) alloc_mem(sizeof(DIRECTINTERFACE));

	memset(newdirect, 0, sizeof(DIRECTINTERFACE));

	result =
	    RegisterInterface(name, &DirectInform, &DirectReceive,
			      (int32) newdirect, mux->interface_lib);
	if (result != ERROR_NONE) {
		free_mem(newdirect);
		free_mem(chdat);

		return DEBUGERROR(result);
	}

	QueryInterfaceIndex(name, mux->interface_lib,
			    &newdirect->host_interface);

	newdirect->operations.open = &DirectOpen;
	newdirect->operations.release = &DirectClose;
	newdirect->operations.read = &DirectRead;
	newdirect->operations.write = &DirectWrite;
	newdirect->operations.poll = &DirectPoll;

	newdirect->operations.owner = THIS_MODULE;

	result = register_chrdev(major, name, &newdirect->operations);
	if (result < 0) {
		UnregisterInterface(newdirect->host_interface,
				    mux->interface_lib);

		free_mem(chdat);
		free_mem(newdirect);

		return DEBUGERROR(ERROR_OPERATIONFAILED);
	}

	if (major == DIRECT_DYNAMIC_MAJOR_ASSIGNMENT)
		major = result;

	newdirect->major = major;
	newdirect->channel_min = channel_min;
	newdirect->channel_max = channel_max;
	newdirect->channel_data = chdat;
	newdirect->mux = mux;

	memcpy(newdirect->device_directory, path, pathsize);
	memcpy(newdirect->interface_name, name, namesize);
	memset(chdat, 0, size);

	node = alloc_mem(sizeof(DIRECT_MAJOR_LIST));
	node->major = major;
	node->directif = newdirect;
	node->next = 0;

	disable_interrupts(state);

	browse = &major_list;
	while (*browse)
		browse = &(*browse)->next;

	*browse = node;

	enable_interrupts(state);

	*direct = newdirect;

	return DEBUGERROR(ERROR_NONE);
}

/*
 * DestroyDirectInterface is invoked when a NetMUX is being destroyed.
 * The interface object will be 'undone'.
 *
 * Params:
 * direct -- the interface to be destroyed
 */
int32 DestroyDirectInterface(DIRECTINTERFACE *direct)
{
	DIRECT_MAJOR_LIST **browse;
	DIRECT_MAJOR_LIST *found;
	DIRECT_CHANNELDATA *chdat;
	INTERRUPT_STATE state;
	int32 channels;
	int32 index;

	DEBUG("DestroyDirectInterface(0x%p)\n", direct);

	chdat = direct->channel_data;
	channels = direct->channel_max - direct->channel_min + 1;

	for (index = 0; index < channels; index++) {
		if (chdat[index].state & DIRECT_STATE_READY) {
			DisableChannel(host_end(COMMAND),
				       index + direct->channel_min,
				       direct->host_interface,
				       chdat[index].client_interface,
				       direct->mux);

			if (chdat[index].refcount)
				wait_event_interruptible(chdat[index].
							 close_wait,
							 !(chdat[index].
							   refcount));

			device_destroy(netmux_class,
				       MKDEV(direct->major, index));
		}
	}

	unregister_chrdev(direct->major, direct->interface_name);

	disable_interrupts(state);

	browse = &major_list;
	while (*browse && (*browse)->major != direct->major)
		browse = &(*browse)->next;

	if (*browse) {
		found = *browse;
		*browse = found->next;

		free_mem(found);
	}

	enable_interrupts(state);

	UnregisterInterface(direct->host_interface,
			    direct->mux->interface_lib);

	free_mem(direct->channel_data);
	free_mem(direct);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * DirectOpen is called by the linux file system whenever an open
 * is performed on a device belonging to this interface. The open
 * will send an enable request to a client netmux and wait for a
 * reply. If any errors occurr the device will not be open.
 *
 * Params:
 * inode -- used to let us fetch the major/minor of the device
 * filp -- some private data is set within this structure
 */
int DirectOpen(struct inode *inode, struct file *filp)
{
	DIRECT_MAJOR_LIST *browse;
	DIRECTINTERFACE *direct;
	DIRECT_CHANNELDATA *chdat;
	INTERRUPT_STATE state;
	int32 major;
	int32 minor;
	int32 result;
#ifdef DEBUG_NETMUX_AUDIO
	char *pathname = 0, pathbuf[256];
	struct path path = filp->f_path;
	path_get(&filp->f_path);
	pathname = d_path(&path, pathbuf, sizeof(pathbuf));
	path_put(&path);
	if (!strcmp(pathname, "/dev/audio"))
		printk(KERN_INFO "(%s): enter,  pathname[%s]\n", __func__, (pathname ? pathname : "NULL"));
#endif
	DEBUG("DirectOpen(0x%p, 0x%p)\n", inode, filp);

	major = MAJOR(inode->i_rdev);
	minor = MINOR(inode->i_rdev);

	disable_interrupts(state);

	browse = major_list;
	while (browse && browse->major != major)
		browse = browse->next;
    
        if(browse)
        {
	   direct = browse->directif;
        }
        else
        {
           return -ECONNABORTED;
        }

	enable_interrupts(state);

	if (minor < direct->channel_min || minor > direct->channel_max)
		return -EADDRNOTAVAIL;

	chdat = &direct->channel_data[minor - direct->channel_min];
	if (!(chdat->state & DIRECT_STATE_READY))
		return -EADDRNOTAVAIL;

	if (chdat->refcount > 0)
		return -EBUSY;

	result = EnableChannel(host_end(COMMAND),
			       minor,
			       chdat->burstsize,
			       chdat->maxdata,
			       direct->host_interface,
			       chdat->client_interface,
			       chdat->host_byte_credit,
			       chdat->host_send_credit, 0, 0, direct->mux);
	if (result != ERROR_NONE)
		return -ECONNABORTED;

	/* Wake up when either one of two conditions occur:
	   1 - A response is received from the BP for our open request.
	   2 - The channel is not configured and running properly.
	   In case the latter is true an error will be returned eventually. */
	wait_event_interruptible(chdat->event_wait,
				 (chdat->state & DIRECT_STATE_EVENT) ||
				 !(chdat->state & DIRECT_STATE_READY));

	chdat->state &= ~DIRECT_STATE_EVENT;

	if (!(chdat->state & DIRECT_STATE_EVENT_SUCCESS))
		return -ECONNREFUSED;

	chdat->state &= ~DIRECT_STATE_EVENT_SUCCESS;

	filp->private_data = direct;

	return 0;
}

/*
 * DirectClose is called by the file system whenever a close is
 * performed on an open device. A disable is sent to the client
 * NetMUX but no response is expected or required. If there is
 * an error with the device being closed an error will be
 * returned.
 *
 * Params:
 * inode -- used to fetch the minor number of the device
 * filp -- stores some private data which we use to fetch the direct
 *         interface structure
 */
int DirectClose(struct inode *inode, struct file *filp)
{
	DIRECTINTERFACE *direct;
	DIRECT_CHANNELDATA *chdat;
	int32 minor;
	int32 client_interface;
	int32 result;
#ifdef DEBUG_NETMUX_AUDIO
	char *pathname = 0, pathbuf[256];
	struct path path = filp->f_path;
	path_get(&filp->f_path);
	pathname = d_path(&path, pathbuf, sizeof(pathbuf));
	path_put(&path);
	//printk(KERN_INFO "(%s): enter,  pathname[%s]\n", __func__, (pathname ? pathname : "NULL"));
#endif
	DEBUG("DirectClose(0x%p, 0x%p)\n", inode, filp);

	direct = filp->private_data;
	minor = MINOR(inode->i_rdev);

	if (minor < direct->channel_min || minor > direct->channel_max)
		return -EADDRNOTAVAIL;

	chdat = &direct->channel_data[minor - direct->channel_min];

	chdat->state = DIRECT_STATE_READY;

	client_interface = chdat->client_interface;

	result =
	    DisableChannel(host_end(COMMAND), minor,
			   direct->host_interface, client_interface,
			   direct->mux);

	if (chdat->refcount <= 0) {
		if (result != ERROR_NONE)
			return -ENOTCONN;
	}

	return 0;
}

/*
 * DirectPoll is invoked whenever a user performs a select() or
 * a poll(). This function allows a user to wait for a certain
 * set of conditions to be met before it returns. The direct
 * interface supports notification of the following events:
 * Data available to be read, Space available to be written
 * to, and whether or not the device is functioning.
 *
 * Params:
 * filp -- used to fetch the minor and direct structure
 * table -- used to notify the upper driver layer that
 *          the function needs to wait for some conditions
 *          to be met.
 */
unsigned int DirectPoll(struct file *filp, poll_table * table)
{
	DIRECTINTERFACE *direct;
	DIRECT_CHANNELDATA *chdat;
	int32 minor;
	int32 mask;
#ifdef DEBUG_NETMUX_AUDIO
	char *pathname = 0, pathbuf[256];
	struct path path = filp->f_path;
	path_get(&filp->f_path);
	pathname = d_path(&path, pathbuf, sizeof(pathbuf));
	path_put(&path);

	//fprintk(KERN_INFO "(%s): enter,  pathname[%s]\n", __func__, (pathname ? pathname : "NULL"));
#endif
	DEBUG("DirectPoll(0x%p, 0x%p)\n", filp, table);

	direct = filp->private_data;
	minor = MINOR(filp->f_dentry->d_inode->i_rdev);
	mask = 0;

	if (minor < direct->channel_min || minor > direct->channel_max)
		return POLLERR;

	chdat = &direct->channel_data[minor - direct->channel_min];
	if (!(chdat->state & DIRECT_STATE_READY) || chdat->refcount == 0)
		return POLLERR;

	poll_wait(filp, &chdat->rdevent, table);
	poll_wait(filp, &chdat->wrevent, table);

	if (ReadDataAvailable(minor, direct->mux))
		mask |= POLLIN | POLLRDNORM;

	if (SendDataAvailable(minor, direct->mux))
		mask |= POLLOUT | POLLWRNORM;

	return mask;
}

/*
 * DirectRead is called whenever a device is read from. This function
 * will pull data from the mux and copy it into a user supplied buffer.
 * If the device is in a blocking mode this call will block until
 * data is available to be read. If any errors occur the user will
 * be notified.
 *
 * Params:
 * filp -- used to get the direct struct and the minor number
 * buf -- pointer to where the data is to be received
 * count -- the amount of data to attempt to read
 * f_pos -- an unused paramater
 */
ssize_t DirectRead(struct file *filp, char *buf, size_t count,
		   loff_t *f_pos)
{
	DIRECTINTERFACE *direct;
	DIRECT_CHANNELDATA *chdat;
	COMMBUFF *commbuff = 0;
	int32 minor;
	int32 result;
	int32 commbuffsize;
#ifdef DEBUG_NETMUX_AUDIO
	char *pathname = 0, pathbuf[256];
	struct path path = filp->f_path;
	path_get(&filp->f_path);
	pathname = d_path(&path, pathbuf, sizeof(pathbuf));
	path_put(&path);
#endif
	DEBUG("DirectRead(0x%p, 0x%p, %d, 0x%p)\n", filp, buf, count,
	      f_pos);
#ifdef DEBUG_NETMUX_AUDIO
	//printk(KERN_INFO "(%s): DirectWrite begin(0x%p, 0x%p, %d, 0x%p), pathname[%s]\n", __func__, filp, buf, count, f_pos, (pathname ? pathname : "NULL"));
#endif
	direct = filp->private_data;
	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	if (minor < direct->channel_min || minor > direct->channel_max)
		return -EADDRNOTAVAIL;

	chdat = &direct->channel_data[minor - direct->channel_min];
	if (!(chdat->state & DIRECT_STATE_READY) || chdat->refcount == 0)
		return -ENOTCONN;

	if (count <= 0)
		return 0;

	if (!(filp->f_flags & O_NONBLOCK))
		wait_event_interruptible(chdat->rdevent,
					 ReadDataAvailable(minor,
							   direct->mux));

	ReadData(minor, direct->mux, &commbuff, count);
	if (!commbuff)
		return -EAGAIN;

	commbuffsize = commbuff_length(commbuff);

	result = copy_to_user(buf, commbuff_data(commbuff), commbuffsize);
	if (result) {
		free_commbuff(commbuff);

		return -EFAULT;
	}
#ifdef DEBUG_NETMUX_AUDIO
/*
	if (!strcmp(pathname, "/dev/audio"))
	{
		dump_hex(commbuff_data(commbuff), commbuffsize);
	}
*/
#endif
	LOGCOMMBUFF_CH(minor, "DirectRead( )-->", commbuff, commbuffsize);

	free_commbuff(commbuff);

	return commbuffsize;
}



/*
 * DirectWrite is called whenever the user tries to write data
 * to a device. If the device cannot take the data or there
 * is some other error the user will be notified. The data
 * to be delivered is copied out of the user supplied buffer
 * and into an allocated commbuff.
 *
 * Params:
 * filp -- used to get the direct interface and the minor number
 * buf -- a pointer to the data to be copied
 * count -- the number of bytes to be copied
 * f_pos -- not used
 */
ssize_t DirectWrite(struct file *filp, const char *buf, size_t count,
		    loff_t *f_pos)
{
	COMMBUFF *commbuff;
	DIRECTINTERFACE *direct;
	DIRECT_CHANNELDATA *chdat;
	int32 minor;
	int32 result;
#ifdef DEBUG_NETMUX_AUDIO
	char *pathname = 0, pathbuf[256];
	struct path path = filp->f_path;
	path_get(&filp->f_path);
	pathname = d_path(&path, pathbuf, sizeof(pathbuf));
	path_put(&path);
#endif
	DEBUG("DirectWrite begin(0x%p, 0x%p, %d, 0x%p)\n", filp, buf,
	      count, f_pos);
#ifdef DEBUG_NETMUX_AUDIO
	//printk(KERN_INFO "(%s): DirectWrite begin(0x%p, 0x%p, %d, 0x%p), pathname[%s]\n", __func__, filp, buf, count, f_pos, (pathname ? pathname : "NULL"));
#endif
	direct = filp->private_data;
	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	if (minor < direct->channel_min || minor > direct->channel_max)
		return -EADDRNOTAVAIL;

	chdat = &direct->channel_data[minor - direct->channel_min];
	if (!(chdat->state & DIRECT_STATE_READY) || chdat->refcount == 0)
		return -ENOTCONN;

	commbuff = alloc_commbuff(count, sizeof(DATA_PACKET_HDR));

	result = copy_from_user(commbuff_data(commbuff), buf, count);
	if (result) {
		free_commbuff(commbuff);

		return -EFAULT;
	}
#ifdef DEBUG_NETMUX_AUDIO
	if (!strcmp(pathname, "/dev/audio"))
	{
		dump_hex(commbuff_data(commbuff), count);
	}
#endif
	result = SendData(minor, commbuff, NULL, direct->mux);
	if (result != ERROR_NONE) {
		free_commbuff(commbuff);

		return 0;
	}

	DEBUG("DirectWrite end(0x%p, 0x%p, %d, 0x%p)\n", filp, buf, count,
	      f_pos);

	return count;
}
