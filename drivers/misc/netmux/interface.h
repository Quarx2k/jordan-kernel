/******************************************************************************
 * NetMUX interface.h                                                         *
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
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* interface.h provides the necessary data types and macros to interface.c.   */
/* Each interface and mux should include this header file if they intend on   */
/* using the interface API.                                                   */

#ifndef _NETMUX_MUXINTERFACE_H_
#define _NETMUX_MUXINTERFACE_H_


#include "types.h"
#include "errorcodes.h"
#include "utility.h"


/*
 * LIBRARY_INFORM defines a macro to easily invoke an interfaces
 * inform function
 */
#define LIBRARY_INFORM(if, index, l)				\
		((l)->interfaces[index]->Inform((void *)if,	\
		 (void *)(l)->interfaces[index]->param))

/*
 * INTERFACEINFORM defines a structure to be delivered to
 * interface inform functions. The members are briefly described
 * below.
 *
 * inform_type is the reason why the interface inform function is
 *             being invoked
 * inform_type is the reason why the interface inform function is
 *             being invoked
 * data associates a particular set of data with the inform function call
 */
typedef struct INTERFACEINFORM {
	int32 inform_type;
	struct MUX *source;
	void *data;
} INTERFACEINFORM;

/*
 * MUXINTERFACE defines and associates an interface with specific
 * functionality . The members are briefly described below
 *
 * Inform points to an inform function the mux can invoke
 * Receive points to a receive function the mux can invoke
 * interface_index stores the index of the interface which is
 * 	 equivlent to the id
 * param is a parameter passed to the invoke and receive functions
 */
typedef struct MUXINTERFACE {
	int32(*Inform) (void *, void *);
	int32(*Receive) (COMMBUFF *, void *);

	int32 interface_index;
	int32 param;
} MUXINTERFACE;

/*
 * MUXINTERFACE_LIBRARY stores a list of registered interfaces. The
 * members are briefly described below.
 *
 * maxinterfaces is the maximum number of interfaces that the library
 *               can hold
 * names is a list of names for each interface in the library
 * interfaces is a list of interfaces within the library
 */
typedef struct MUXINTERFACE_LIBRARY {
	int32 maxinterfaces;
	sint8 **names;
	MUXINTERFACE **interfaces;
	CRITICALSECTION lock;
} MUXINTERFACE_LIBRARY;


/*
 * Define various functions used by the interface API
 */

int32 CreateInterfaceLibrary(int32, MUXINTERFACE_LIBRARY **);
int32 DestroyInterfaceLibrary(MUXINTERFACE_LIBRARY *);

int32 RegisterInterface(sint8 *, int32(*inform) (void *, void *),
			int32(*receive) (COMMBUFF *, void *), int32,
			MUXINTERFACE_LIBRARY *);
int32 UnregisterInterface(int32, MUXINTERFACE_LIBRARY *);

int32 QueryInterfaceIndex(sint8 *, MUXINTERFACE_LIBRARY *, int32 *);
int32 ConnectInterface(int32, MUXINTERFACE_LIBRARY *, MUXINTERFACE **);

int32 BroadcastLibraryInform(INTERFACEINFORM *, MUXINTERFACE_LIBRARY *);


#endif
