/******************************************************************************
 * NetMUX debug_host.h                                                        *
 *                                                                            *
 * Copyright (C) 2006-2008 Motorola, Inc.                                     *
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
 *   2007/06/25  Motorola    Modified Copyright                               *
 *   2008/10/14  Motorola    Add two new panic ID                             *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* debug_host.h provides conditional macros that will resolve to something    */
/* meaningful if debugging or logging is enabled. These macros will record    */
/* data via the output function provided by the NetMUX utilities.             */

#ifndef _NETMUX_DEBUG_HOST_H_
#define _NETMUX_DEBUG_HOST_H_


#include "shared/ldprotocol.h"

#include "types.h"
#include "errorcodes.h"
#include "utility.h"


/* Reserved NetMUX Message/Primitive IDs */
#define NETMUX_MSGID_BASE 0xA6800
enum {
	NETMUX_TRACE		/* @LOG */
	    = NETMUX_MSGID_BASE,
	NETMUX_PACKET_INVALID,	/* @LOG */
	NETMUX_PACKET_DATA,	/* @LOG */
	NETMUX_PACKET_CREDIT,	/* @LOG */
	NETMUX_PACKET_ENABLEMUX,	/* @LOG */
	NETMUX_PACKET_DISABLEMUX,	/* @LOG */
	NETMUX_PACKET_ENABLECHANNEL,	/* @LOG */
	NETMUX_PACKET_DISABLECHANNEL,	/* @LOG */
	NETMUX_PACKET_QUERYINTERFACE,	/* @LOG */
	NETMUX_PACKET_CHANNELSIGNAL,	/* @LOG */
	NETMUX_COMMBUFF_HEADER,	/* @LOG NETMUX_COMMBUFF_HEADER_T */
	NETMUX_RECEIVE_FROM_LD,	/* @LOG */
	NETMUX_CH_DATA_NSAPI5,	/* @LOG */
	NETMUX_CH_DATA_NSAPI6,	/* @LOG */
	NETMUX_CH_DATA_NSAPI7,	/* @LOG */
	NETMUX_CH_DATA_NSAPI8,	/* @LOG */
	NETMUX_CH_DATA_NSAPI9,	/* @LOG */
	NETMUX_CH_DATA_NSAPI10,	/* @LOG */
	NETMUX_CH_MUXTESTNET,	/* @LOG */
	NETMUX_CH_UDI_CTRL,	/* @LOG */
	NETMUX_CH_UDI_DATA,	/* @LOG */
	NETMUX_CH_TEL,		/* @LOG */
	NETMUX_CH_MM_AUDIO_DEC,	/* @LOG */
	NETMUX_CH_MM_AUDIO_ENC,	/* @LOG */
	NETMUX_CH_UDI_PTP,	/* @LOG */
	NETMUX_CH_PSC5,		/* @LOG */
	NETMUX_CH_PSC6,		/* @LOG */
	NETMUX_CH_PSC7,		/* @LOG */
	NETMUX_CH_PSC8,		/* @LOG */
	NETMUX_CH_PSC9,		/* @LOG */
	NETMUX_CH_PSC10,	/* @LOG */
	NETMUX_CH_PS_CONTROL,	/* @LOG */
	NETMUX_CH_AUDIO,	/* @LOG */
	NETMUX_CH_NVM_PROXY,	/* @LOG */
	NETMUX_CH_SIM_PROXY,	/* @LOG */
	NETMUX_CH_POWER,	/* @LOG */
	NETMUX_CH_GPS,		/* @LOG */
	NETMUX_CH_TESTCMD,	/* @LOG */
	NETMUX_CH_SUBSIDY_LOCK,	/* @LOG */
	NETMUX_CH_EFEM,		/* @LOG */
	NETMUX_CH_NETMON,	/* @LOG */
	NETMUX_CH_OPPROF,	/* @LOG */
	NETMUX_CH_UMA_URLC,	/* @LOG */
	NETMUX_CH_UMA_RRC,	/* @LOG */
	NETMUX_CH_DLOG,		/* @LOG */
	NETMUX_CH_MUXTESTDIR1,	/* @LOG */
	NETMUX_CH_MUXTESTDIR2,	/* @LOG */
	NETMUX_CH_CS_DATA,	/* @LOG */
	NETMUX_CH_MUXTESTTTY1,	/* @LOG */
	NETMUX_CH_MUXTESTTTY2	/* @LOG */
};

typedef void *SU_PORT_HANDLE;

void output(SU_PORT_HANDLE, int32, sint8 *, ...);

/*
 * Define debugging macros
 */
int32 debug_error(sint8 *, sint8 *, int32, int32);
int32 debug_ldp(sint8 *, sint8 *, int32, int32);

#define DEBUG(...) \
  output(0, 0, __VA_ARGS__)
#define DBGFC(...) \
  output(0, 0, __VA_ARGS__)
#define DEBUGERROR(code) \
  debug_error((sint8 *)__func__, " ", __LINE__, code)
#define DEBUGLDP(code) \
  debug_ldp((sint8 *)__func__, " ", __LINE__, code)

/*
 * Define logging macros
 */
void log_commbuff(int32, sint8 *, COMMBUFF *, int32);
void log_packettype(COMMBUFF *);

#define LOG_TRACE(...) \
  output(0, 0, __VA_ARGS__)
#define LOGCOMMBUFF_RECV_LD(h, cb, l) \
  log_commbuff(0, h, cb, l)
#define LOGCOMMBUFF_CH(ch_num, h, cb, l) \
  log_commbuff(0, h, cb, l)
#define LOGCOMMBUFF_PROTO(port, id, len, buff)
#define LOG_PACKETTYPE(cb) \
  log_packettype(cb)

/*
 * Define some panic location codes
 */
/*----------------------------------------------*/
#define netmuxPanicFirst 0x00080c00
/*----------------------------------------------*/
#define netmuxPanicILDFail  0x00080c00
#define netmuxPanicABSFail  0x00080c01
#define netmuxPanicCBUFFail 0x00080c02
#define netmuxPanicSKBFail1 0x00080c03
#define netmuxPanicSKBFail2 0x00080c04
#define netmuxPanicSKBFail3 0x00080c05
#define netmuxPanicSKBFail4 0x00080c06
#define netmuxPanicMemFail1 0x00080c07
#define netmuxPanicMemFail2 0x00080c08
#define netmuxPanicPkgFail1 0x00080c09
#define netmuxPanicPkgFail2 0x00080c10
/*----------------------------------------------*/
#define netmuxPanicLast  0x00080c7f
/*----------------------------------------------*/


#endif
