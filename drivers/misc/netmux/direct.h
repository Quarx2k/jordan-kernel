/******************************************************************************
 * NetMUX direct.h                                                            *
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
 *   2007/12/05  Motorola    change header file as kernel change              *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* direct.h is responsible for setting up a method of communication between   */
/* the NetMUX and user space applications.                                    */

#ifndef _NETMUX_DIRECT_H_
#define _NETMUX_DIRECT_H_


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
#include <linux/uaccess.h>


/*
 * Defined to allow for dynamic major assignment upon driver registration
 */
#define DIRECT_DYNAMIC_MAJOR_ASSIGNMENT 0

/*
 * Declares different states a direct channel can be placed in
 */
#define DIRECT_STATE_DEFAULT       0
#define DIRECT_STATE_READY         1
#define DIRECT_STATE_EVENT         2
#define DIRECT_STATE_EVENT_SUCCESS 4


/*
 * DIRECT_MAJOR_LIST defines a structure to hold information about a
 * major number. A brief description of the members is below.
 *
 * major is the major# to associate the data with
 * directif points to direct interface
 * next points to the next major number
 */
typedef struct DIRECT_MAJOR_LIST {
	int32 major;
	struct DIRECTINTERFACE *directif;

	struct DIRECT_MAJOR_LIST *next;
} DIRECT_MAJOR_LIST;

/*
 * DIRECT_CHANNELDATA defines a structure to bind channels to a
 * certain configuration. A brief description of the members is
 * below.
 *
 * refcount is the total number of times the channel has been enabled
 * state represents the current state of this channel
 * remote_interface is the interface id of the external interface
 * burstsize is the default maximum amount of data to be sent at a time
 * maxdata is the default maximum amount of data to be queued
 * host_byte_credit is the default number of bytes that can be received
 * host_send_credit is the default number of sends that can be received
 * event_wait defines a structure that a direct device can sleep on
 * close_wait defines a structure that a close operation can sleep on
 * rdevent defines a wait queue for read operations to wake up
 * wrevent defines a wait queue for write operations to wake up
 * device_file is the name of the file entry in /dev
 */
typedef struct DIRECT_CHANNELDATA {
	int32 refcount;
	int32 state;
	int32 client_interface;
	int32 burstsize;
	int32 maxdata;
	int32 host_byte_credit;
	int32 host_send_credit;

	wait_queue_head_t event_wait;
	wait_queue_head_t close_wait;
	wait_queue_head_t rdevent;
	wait_queue_head_t wrevent;

	sint8 device_file[PACKET_MAXNAME_LENGTH];
} DIRECT_CHANNELDATA;

/*
 * DIRECTINTERFACE declares a structure to manage direct channel
 * data pointers. A brief description of the members is below.
 *
 * major is the major number of the registered direct interface
 * channel_min is the inclusive lower boundary of channel numbers
 * channel_max is the inclusive upper boundary of channel numbers
 * local_interface is the interface id for the local interface
 * channel_data is a list of structures bound to specific channels
 * mux points to the MUX object associated with this direct interface
 * device_directory is the directory within /dev to create devices
 * interface_name is the name of this interface
 * operations defines standard operations a direct device should use
 */
typedef struct DIRECTINTERFACE {
	int32 major;
	int32 channel_min;
	int32 channel_max;
	int32 host_interface;

	DIRECT_CHANNELDATA *channel_data;
	MUX *mux;

	sint8 device_directory[PACKET_MAXNAME_LENGTH];
	sint8 interface_name[PACKET_MAXNAME_LENGTH];

	struct file_operations operations;
} DIRECTINTERFACE;


/*
 * Define various functions used by the direct interface
 */

int32 DirectInform(void *, void *);
int32 DirectReceive(COMMBUFF *, void *);

int32 CreateDirectInterface(sint8 *, sint8 *, int32, int32, int32, MUX *,
			    DIRECTINTERFACE **);
int32 DestroyDirectInterface(DIRECTINTERFACE *);

int DirectOpen(struct inode *, struct file *);
int DirectClose(struct inode *, struct file *);
unsigned int DirectPoll(struct file *, poll_table *);
ssize_t DirectRead(struct file *, char *, size_t, loff_t *);
ssize_t DirectWrite(struct file *, const char *, size_t, loff_t *);


#endif
