/******************************************************************************
 * NetMUX register.h                                                          *
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
 *   2007/04/06  Motorola    Reduced maximum channel numbers and increase     *
 *                           the number of send buffers                       *
 *   2007/05/01  Motorola    Change codes to ensure "shared" netmux           *
 *                           code is identical between AP and BP.             *
 *   2008/04/10  Motorola    Add proc_fs for AP Log re-work                   *
 *   2008/07/16  Motorola    Add NetmuxLogInit declaration for AP config log  *
 *   2010/04/28  Motorola    Format cleanup                                   *
 *   2010/10/29  Motorola    Enable Test channels                             * 
 ******************************************************************************/

/* register.h defines data types used by register.c. The types may very based */
/* on the NetMUX environment but the overall usage of the types remains the   */
/* same.                                                                      */

#ifndef _NETMUX_REGISTER_H_
#define _NETMUX_REGISTER_H_


#include "shared/ldprotocol.h"

#include "types.h"
#include "errorcodes.h"
#include "utility.h"
#include "mux.h"
#include "config.h"
#include <linux/proc_fs.h>

/*
 * Define some magic numbers to be used by the
 * registration interface.
 */
#define REGISTER_MAX_CHANNELS       48
#define REGISTER_CONFIG_CHANNEL     0
#define REGISTER_NETWORK_MINCH      1
#define REGISTER_NETWORK_MAXCH      8
#define REGISTER_DIRECT_MINCH       9
#define REGISTER_DIRECT_MAXCH       41
#define REGISTER_TTY_MINCH          42
#define REGISTER_TTY_MAXCH          47
#define REGISTER_CONNECTIVITY_MINCH 1
#define REGISTER_CONNECTIVITY_MAXCH 47
/*
 * The definitions define where to retrieve the NetMUX
 * channel configuration info
 */
#define REGISTER_CONFIGDATA_COUNT netmux_configdata_count
#define REGISTER_CONFIGDATA       netmux_configdata


/*
 * INTERFACELIST holds a list of link drivers that
 * are associated with MUX objects
 * A brief description follows below.
 *
 * muxinterface defines the interface to the MUX
 * linkinterface defines the interface to the link driver
 * mux points to a specific MUX object
 * config points to the config interface for this connection
 * direct points to a direct interface for this connection
 * network points to a network interface for this connection
 * tty points to a tty interface for this connection
 * connectivity points to a connectivity interface for this connection
 * next points to the next defined interface
 */
typedef struct INTERFACELIST {
	INTERFACEMUX muxinterface;
	INTERFACELINK linkinterface;
	MUX *mux;

	CONFIGINTERFACE *config;

	void *direct;
	void *network;
	void *tty;
	void *connectivity;

	struct INTERFACELIST *next;
} INTERFACELIST;


/*
 * Define variables that must be defined in channelconfig.c
 */
extern int32 netmux_configdata_count;
extern CONFIGDATA netmux_configdata[];


/*
 * Define various functions defined in register.c
 */

int32 RegisterMUXLink(INTERFACELINK *, INTERFACEMUX *);
int32 UnregisterMUXLink(void *);
int32 ActivateMUX(MUX *, INTERFACELIST *);
void DeactivateMUX(INTERFACELIST *);
void NetmuxLogInit(void);

#endif
