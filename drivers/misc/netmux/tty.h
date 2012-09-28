/******************************************************************************
 * NetMUX tty.h                                                               *
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
 *   2007/01/15  Motorola    Removed able_to_send flag and added data_amount  *
 *                           field to the TTY_CHANNELDATA structure           *
 *   2007/02/01  Motorola    Added mux_channel_queue_space                    *
 *   2007/12/05  Motorola    Change kernel file as kernel upgrade             *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* tty.h is responsible for setting up a method of communication between the  */
/* NetMUX and user space applications.                                        */

#ifndef _NETMUX_TTY_H_
#define _NETMUX_TTY_H_


#include "types.h"
#include "errorcodes.h"
#include "utility.h"
#include "mux.h"

#include <linux/device.h>

#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>


/*
 * Defined to allow for dynamic major assignment upon driver registration
 */
#define TTY_DYNAMIC_MAJOR_ASSIGNMENT 0

#define TTY_DEFAULT_MODEM_FLAGS 0

/*
 * Declares different states a tty channel can be placed in
 */
#define TTY_STATE_DEFAULT       0
#define TTY_STATE_READY         1
#define TTY_STATE_EVENT         2
#define TTY_STATE_EVENT_SUCCESS 4


/*
 * TTY_CHANNELDATA defines a structure to bind channels
 * to a certain configuration
 * A brief description is provided below.
 *
 * commbuff represents a commbuff currently being received
 * refcount is the total number of times the channel has been enabled
 * state represents the current state of this channel
 * client_interface is the interface id of the external interface
 * burstsize is the default maximum amount of data to be sent at a time
 * maxdata is the default maximum amount of data to be queued
 * data_amount is the current amount of data stored in the process_queue
 * host_byte_credit is the default number of bytes that can be received
 * host_send_credit is the default number of sends that can be received
 * modem_flags is modified on an ioctl
 * mux_channel_queue_space is the current amount of space available
 * 	in the mux channel queue
 * process_queue holds a list of commbuffs to be transmitted
 * event_wait defines a structure that a tty device can sleep on
 * close_wait defines a structure that a close operation can sleep on
 * device_file is the name of the file entry in /dev
 */
typedef struct TTY_CHANNELDATA {
	COMMBUFF *commbuff;

	int32 refcount;
	int32 state;
	int32 client_interface;
	int32 burstsize;
	int32 maxdata;
	int32 data_amount;
	int32 host_byte_credit;
	int32 host_send_credit;
	int32 modem_flags;
	int32 mux_channel_queue_space;

	COMMBUFFQUEUE process_queue;

	wait_queue_head_t event_wait;
	wait_queue_head_t close_wait;

	sint8 device_file[PACKET_MAXNAME_LENGTH];
} TTY_CHANNELDATA;

/*
 * TTY_INTERFACE declares a structure to manage tty channel data pointers
 * A brief description is provided below.
 *
 * channel_min is the inclusive lower boundary of channel numbers
 * channel_max is the inclusive upper boundary of channel numbers
 * host_interface is the interface id for the local interface
 * channel_data is a list of structures bound to specific channels
 * mux points to the MUX object associated with this tty interface
 * device_directory is the directory within /dev to create devices
 * interface_name is the name of this interface
 */
typedef struct TTYINTERFACE {
	int32 channel_min;
	int32 channel_max;
	int32 host_interface;

	TTY_CHANNELDATA *channel_data;
	MUX *mux;
	CRITICALSECTION lock;

	sint8 device_directory[PACKET_MAXNAME_LENGTH];
	sint8 interface_name[PACKET_MAXNAME_LENGTH];

	struct tty_driver driver;
} TTYINTERFACE;


/*
 * Define various functions used by the tty interface
 */

int32 TTYInform(void *, void *);
int32 TTYReceive(COMMBUFF *, void *);

int32 CreateTTYInterface(sint8 *, sint8 *, int32, int32, int32, MUX *,
			 TTYINTERFACE **);
int32 DestroyTTYInterface(TTYINTERFACE *);

int TTYOpen(struct tty_struct *, struct file *);
void TTYClose(struct tty_struct *, struct file *);
ssize_t TTYWrite(struct tty_struct *, const unsigned char *, int);
int TTYDataInBuffer(struct tty_struct *);
int TTYWriteRoom(struct tty_struct *);
int TTYIOCtl(struct tty_struct *, struct file *, unsigned int,
	     unsigned long);


#endif
