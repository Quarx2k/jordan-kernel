/******************************************************************************
 * NetMUX interface.c                                                         *
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
 *   2007/05/01  Motorola    Change codes to ensure "shared" netmux           *
 *                           code is identical between AP and BP.             *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* interface.c provides an API to the mux and its interfaces which allows     */
/* them to communicate easily. This API merely allows simple searching of     */
/* interfaces to be performed as well as simple communication routines.       */

#include "interface.h"
#include "debug.h"


/*
 * CreateInterfaceLibrary creates an object to store a list of
 * interfaces. These interfaces define a receive function and
 * an inform function with which the mux can communicate. By
 * placing all interfaces in a common library the mux can
 * access all the functionality attached to it on a whim.
 *
 * Params:
 * maxinterfaces -- the number of shelves in the library
 * lib -- a poitner to receive the library object
 */
int32 CreateInterfaceLibrary(int32 maxinterfaces,
			     MUXINTERFACE_LIBRARY **lib)
{
	MUXINTERFACE **newinterfaces;
	sint8 **newnames;
	MUXINTERFACE_LIBRARY *newlib;
	int32 size;

	DEBUG("CreateInterfaceLibrary(%lu, %p)\n", maxinterfaces, lib);

	if (!maxinterfaces || !lib)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	newlib =
	    (MUXINTERFACE_LIBRARY *)
	    alloc_mem(sizeof(MUXINTERFACE_LIBRARY));
	size = maxinterfaces * sizeof(void *);
	newinterfaces = (MUXINTERFACE **) alloc_mem(size);

	newnames = (sint8 **) alloc_mem(size);

	memset(newinterfaces, 0, size);
	memset(newnames, 0, size);

	newlib->interfaces = newinterfaces;
	newlib->names = newnames;
	newlib->maxinterfaces = maxinterfaces;

	initialize_criticalsection_lock(&newlib->lock);

	*lib = newlib;

	return DEBUGERROR(ERROR_NONE);
}

/*
 * DestroyInterfaceLibrary will free any resources currently consumed
 * by the library object itself. If there are any interfaces still
 * contained in the library this function call will fail. It is assumed
 * that there are no operations happening on this object at the time
 * the destroy is called.
 *
 * Params:
 * lib -- a pointer to the library to be destroyed
 */
int32 DestroyInterfaceLibrary(MUXINTERFACE_LIBRARY *lib)
{
	int32 index;

	DEBUG("DestroyInterfaceLibrary(%p)\n", lib);

	for (index = 0; index < lib->maxinterfaces; index++) {
		if (lib->interfaces[index])
			return DEBUGERROR(ERROR_OPERATIONFAILED);
	}

	destroy_criticalsection_lock(&lib->lock);

	free_mem(lib->interfaces);
	free_mem(lib->names);
	free_mem(lib);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * RegisterInterface is invoked when an interface wishes to register
 * itself with a certain interface library. Normally that library is
 * attached to the NetMUX and by registering the interface the mux
 * can communicate with it. The interface must supply its inform and
 * receive functions the mux is to use as well as a custom parameter
 * and a name.
 *
 * Params:
 * name -- the name of the interface
 * inform -- the inform function to be called by the mux
 * receive -- the receive function to be called by the mux
 * param -- the custom value the interface wishes to be passed
 *          into an inform or receive call
 * lib -- the library object to attach the interface to
 */
int32 RegisterInterface(sint8 *name, int32(*inform) (void *, void *),
			int32(*receive) (COMMBUFF *, void *), int32 param,
			MUXINTERFACE_LIBRARY *lib)
{
	MUXINTERFACE *newinterface;
	sint8 *newname;
	int32 namelength;
	int32 index;

	DEBUG("RegisterInterface(%p, %p, %p, %lu, %p)\n", name, inform,
	      receive, param, lib);

	if (!name || !inform || !receive || !lib)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	enter_write_criticalsection(&lib->lock);

	for (index = 0; index < lib->maxinterfaces; index++) {
		if (!lib->interfaces[index])
			break;
	}

	if (index >= lib->maxinterfaces) {
		exit_write_criticalsection(&lib->lock);

		return DEBUGERROR(ERROR_OPERATIONFAILED);
	}

	namelength = strlen((char *) name) + 1;

	newinterface = (MUXINTERFACE *) alloc_mem(sizeof(MUXINTERFACE));
	newname = (sint8 *) alloc_mem(namelength);

	memcpy(newname, name, namelength);

	newinterface->Inform = inform;
	newinterface->Receive = receive;
	newinterface->interface_index = index;
	newinterface->param = param;

	lib->interfaces[index] = newinterface;
	lib->names[index] = newname;

	exit_write_criticalsection(&lib->lock);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * UnregisterInterface will pull a interface from the library. The caller
 * should take care to ensure no operations are depending on this interface.
 *
 * Params:
 * interface_index -- the interface id/index for the interface to be removed
 * lib -- the library which holds the interface
 */
int32 UnregisterInterface(int32 interface_index,
			  MUXINTERFACE_LIBRARY *lib)
{
	MUXINTERFACE *unreg_interface;

	DEBUG("UnregisterInterface(%lu, %p)\n", interface_index, lib);

	if (interface_index >= lib->maxinterfaces)
		return DEBUGERROR(ERROR_OPERATIONFAILED);

	enter_write_criticalsection(&lib->lock);

	unreg_interface = lib->interfaces[interface_index];
	if (!unreg_interface) {
		exit_write_criticalsection(&lib->lock);

		return DEBUGERROR(ERROR_OPERATIONFAILED);
	}

	free_mem(unreg_interface);
	free_mem(lib->names[interface_index]);

	lib->interfaces[interface_index] = 0;
	lib->names[interface_index] = 0;

	exit_write_criticalsection(&lib->lock);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * QueryInterfaceIndex allows an interface or a mux to locate the interface
 * id by supplying the ascii name of the interface assigned upon registration.
 * The interface index is a speedier and more convenient way of accessing
 * an interface within a library.
 *
 * Params:
 * name -- the name of the interface to search for
 * lib -- the library the interface belongs to
 * interface_index -- a pointer to a location to receive the index
 */
int32 QueryInterfaceIndex(sint8 *name, MUXINTERFACE_LIBRARY *lib,
			  int32 *interface_index)
{
	int32 index;

	DEBUG("QueryInterfaceIndex(%p, %p, %p)\n", name, lib,
	      interface_index);

	enter_read_criticalsection(&lib->lock);

	for (index = 0; index < lib->maxinterfaces; index++) {
		if (lib->names[index]
		    &&
		    (!strcmp((char *) lib->names[index], (char *) name)))
			break;
	}

	exit_read_criticalsection(&lib->lock);

	if (index >= lib->maxinterfaces)
		return DEBUGERROR(ERROR_OPERATIONFAILED);

	if (interface_index)
		*interface_index = index;

	return DEBUGERROR(ERROR_NONE);
}

/*
 * ConnectInterface simply assigns a mux connection to a registered interface.
 * This function call is very simple and almost not needed. Extra verification
 * is done to ensure that the interface is available.
 *
 * Params:
 * interface_index -- interface id of the interface to be connected
 * lib -- the library holding the interface
 * interface_connection -- pointer to a variable to hold the interface pointer
 */
int32 ConnectInterface(int32 interface_index, MUXINTERFACE_LIBRARY *lib,
		       MUXINTERFACE **interface_connection)
{
	MUXINTERFACE *connecting_interface;

	DEBUG("ConnectInterface(%lu, %p, %p)\n", interface_index, lib,
	      interface_connection);

	if (!lib || !interface_connection)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	if (interface_index >= lib->maxinterfaces)
		return DEBUGERROR(ERROR_OPERATIONFAILED);

	enter_write_criticalsection(&lib->lock);

	connecting_interface = lib->interfaces[interface_index];
	if (!connecting_interface) {
		exit_write_criticalsection(&lib->lock);

		return DEBUGERROR(ERROR_OPERATIONFAILED);
	}

	*interface_connection = connecting_interface;

	exit_write_criticalsection(&lib->lock);

	return DEBUGERROR(ERROR_NONE);
}

/*
 * BroadcastLibraryInform is used to communicate a message to every
 * interface within a library. The message is distributed via the
 * interface's registered inform function.
 *
 * informdata -- some known data to be passed to the inform function
 * interface_lib -- the library to perform the broadcast on
 */
int32 BroadcastLibraryInform(INTERFACEINFORM *informdata,
			     MUXINTERFACE_LIBRARY *interface_lib)
{
	int32(*Inform) (void *, void *);
	void *param;
	int32 index;

	DEBUG("BroadcastLibraryInform(%p, %p)\n", informdata,
	      interface_lib);

	for (index = 0; index < interface_lib->maxinterfaces; index++) {
		enter_read_criticalsection(&interface_lib->lock);

		if (interface_lib->interfaces[index]) {
			Inform = interface_lib->interfaces[index]->Inform;
			param =
			    (void *) interface_lib->interfaces[index]->
			    param;

			exit_read_criticalsection(&interface_lib->lock);

			Inform((void *) informdata, param);
		} else
			exit_read_criticalsection(&interface_lib->lock);
	}

	return DEBUGERROR(ERROR_NONE);
}
