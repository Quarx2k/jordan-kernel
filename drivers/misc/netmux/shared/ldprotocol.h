/******************************************************************************
 * NetMUX ldprotocol.h                                                        *
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
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* ldprotocol.h is shared between the NetMUX and any participating link       */
/* driver.  This file defines the necessary data types and constants needed   */
/* to successfully participate in a link-driver <--> NetMUX protocol          */


#ifndef _NETMUX_LDPROTOCOL_H_
#define _NETMUX_LDPROTOCOL_H_


/*
 * Define a set of error codes that can be used for NetMUX <--> Link driver
 * communication.
 *
 * LDP_ERROR_NONE        -- no error
 * LDP_ERROR_FATAL       -- an error happend that can not be recovered from
 * LDP_ERROR_RECOVERABLE -- an error happend that will be fixed
 */
#define LDP_ERROR_NONE        0x00000000
#define LDP_ERROR_FATAL       0x00000001
#define LDP_ERROR_RECOVERABLE 0x00000002


/*
 * Define the inform types that the NetMUX and Link driver may use to
 * communicate states between each other.
 *
 * LDP_INFORM_SHUTDOWN  -- says the device is being disconnected
 * LDP_INFORM_RECOVERED -- says an error has been recovered from
 */
#define LDP_INFORM_SHUTDOWN  0x00000001
#define LDP_INFORM_RECOVERED 0x00000002

/*
 * INTERFACELINK defines the link driver's communication interface.
 * A brief description follows below.
 *
 * LinkSend - points to a function that will physically transmit data.
 *            The function takes a pointer to a commbuff and returns
 *            an LDP error code.
 *
 * LinkInform - points to an inform function that can be used
 * 		to configure the link driver
 *              The first parameter is the inform type and the
 *              second is any data to be associated with the inform type.
 *
 * localMaxRcvSize - the max size that we can receive
 *
 * remoteMaxRcvSize - the max size the remote can receive
 */
typedef struct INTERFACELINK {
	unsigned long (*LinkSend) (void *);
	unsigned long (*LinkInform) (void *, void *);

	unsigned long localMaxRcvSize;
	unsigned long remoteMaxRcvSize;
} INTERFACELINK;


/*
 * INTERFACEMUX defines the mux's communication interface. A brief
 * description follows below.
 *
 * MUXReceive - defines a function the link driver can call
 * 		to give data to the NetMUX. The first parameter
 * 		is the buffer being received, the second is
 * 		the id member of the INTERFACEMUX structure.
 *
 * MUXInform - points to a function the link driver can use
 * 		to configure the NetMUX. The first parameter
 * 		is the inform type and the second is
 * 		any data to be associated with the inform type.
 *
 * CommBuffRequest - a function pointer provided by the NetMUX that
 * 		the linkdriver must use to obtain a COMMBUFF to receive into.
 *
 * id represents a MUX object specific value to
 * allow for more than one MUX object
 */
typedef struct INTERFACEMUX {
	unsigned long (*MUXReceive) (void *, void *);
	unsigned long (*MUXInform) (void *, void *);

	void *id;
} INTERFACEMUX;


/*
 * Define two functions used in registering and unregistering a NetMUX
 * with a link driver.
 *
 * RegisterMUXLink takes a pointer to a completed INTERFACELINK
 * structure and a pointer to an uninitialized INTERFACEMUX pointer.
 * The INTERFACEMUX pointer will be filled in with valid values
 * for the linkdriver to use.
 *
 * UnregisterMUXLink takes a single parameter. The parameter is
 * the 'id' member of the INTERFACEMUX structure. This value
 * is filled in by the NetMUX upon calling RegisterMUXLink.
 */
extern unsigned long RegisterMUXLink(INTERFACELINK *, INTERFACEMUX *);
extern unsigned long UnregisterMUXLink(void *);


#endif
