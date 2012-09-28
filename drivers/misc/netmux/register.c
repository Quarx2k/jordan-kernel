/******************************************************************************
 * NetMUX register.c                                                          *
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
 *   2006/11/15  Motorola    Fixed shutdown order in DeactivateMUX            *
 *   2007/05/01  Motorola    Change codes to ensure "shared" netmux           *
 *                           code is identical between AP and BP.             *
 *   2008/04/10  Motorola    Add AP Log re-work                               *
 *   2008/04/15  Motorola    add DeactivateConfigInterface in DeactivateMUX   *
 *   2008/07/15  Motorola    add NetmuxLogInit for AP config log              *
 *   2009/07/10  Motorola    Update send buffers number                       *
 *   2009/08/06  Motorola    Change permission for /proc/netmuxlog to 660     *
 *   2010/04/28  Motorola    Format cleanup                                   *
 *   2010/09/03  Motorola    Enable proc logging for Engineering builds only  *
 ******************************************************************************/

/* register.c handles the communication process between a link driver and a   */
/* NetMUX.                                                                    */

#include "register.h"
#include "debug.h"
#include "direct.h"
#include "network.h"
#include "tty.h"


#define REGISTER_MAX_INTERFACES 4
#define REGISTER_HOST           1
#define REGISTER_SEND_BUFFERS   336

#define LOG_COMMAND_LEN            2

/*Netmux info accessible via the proc interface*/
static struct proc_dir_entry *proc_netmux_log_entry = NULL;
char *NetmuxLogState = NULL;
static int GetNetmuxLogState(char *buf, char **start, off_t offset,
			     int count, int *eof, void *data);
static int WriteNetmuxLogCommand(struct file *file, const char *buffer,
				 unsigned long count, void *data);
/*
 * Declare a list to hold each registered interface
 */
INTERFACELIST *interfacelist = 0;

/*
 * RegisterMUXLink is called by a linkdriver when registration
 * is required. Registration will setup a valid NetMUX with
 * connected interfaces and begin the communication routine.
 *
 * Params:
 * linkif -- defines a method of communication with the linkdriver
 * muxif -- defines a method of communication with a NetMUX
 */
int32 RegisterMUXLink(INTERFACELINK *linkif, INTERFACEMUX *muxif)
{
	INTERRUPT_STATE state;
	MUX *mux;
	MUXINTERFACE_LIBRARY *muxlib;
	INTERFACELIST **ilist;
	int32 result;

	DEBUG("RegisterMUXLink(%p, %p)\n", linkif, muxif);

	if (!linkif || !muxif)
		return DEBUGERROR(ERROR_INVALIDPARAMETER);

	result = CreateInterfaceLibrary(REGISTER_MAX_INTERFACES, &muxlib);
	if (result != ERROR_NONE)
		return DEBUGERROR(result);

	result = CreateMUX(REGISTER_MAX_CHANNELS,
			   REGISTER_SEND_BUFFERS,
			   linkif->localMaxRcvSize,
			   linkif->remoteMaxRcvSize,
			   linkif->LinkSend, muxlib, &mux);
	if (result != ERROR_NONE) {
		DestroyInterfaceLibrary(muxlib);

		return DEBUGLDP(LDP_ERROR_FATAL);
	}

	muxif->MUXReceive = &ReceiveFromLink;
	muxif->MUXInform = &InformMUX;
	muxif->id = mux;

	disable_interrupts(state);

	ilist = &interfacelist;

	while (*ilist)
		ilist = &(*ilist)->next;

	(*ilist) = (INTERFACELIST *) int_alloc_mem(sizeof(INTERFACELIST));

	(*ilist)->next = 0;

	enable_interrupts(state);

	(*ilist)->mux = mux;

	memcpy(&(*ilist)->muxinterface, muxif, sizeof(INTERFACEMUX));
	memcpy(&(*ilist)->linkinterface, linkif, sizeof(INTERFACELINK));

	result = ActivateMUX(mux, *ilist);
	if (result != ERROR_NONE) {
		DestroyMUX(mux);
		DestroyInterfaceLibrary(muxlib);

		free_mem((*ilist));

		(*ilist) = 0;

		return DEBUGLDP(LDP_ERROR_FATAL);
	}

	return DEBUGLDP(LDP_ERROR_NONE);
}

/*
 * UnregisterMUXLink will unregister a linkdriver from a NetMUX. This
 * process will causes any NetMUX resources to be released and
 * any communication in process to be halted. Generally, the link
 * driver which registered should also be responsible for
 * unregistering.
 *
 * Params --
 * id -- the value provided to the linkdriver upon registration
 */
int32 UnregisterMUXLink(void *id)
{
	INTERRUPT_STATE state;
	INTERFACELIST **ilist;
	INTERFACELIST *found;

	DEBUG("UnregisterMUXLink(%p)\n", id);

	ilist = &interfacelist;

	disable_interrupts(state);

	while ((*ilist) && (*ilist)->muxinterface.id != id)
		ilist = &(*ilist)->next;

	if (!(*ilist)) {
		enable_interrupts(state);

		return DEBUGLDP(LDP_ERROR_FATAL);
	}

	found = (*ilist);
	(*ilist) = found->next;

	enable_interrupts(state);

	DeactivateMUX(found);

	return DEBUGLDP(LDP_ERROR_NONE);
}

/*
 * ActivateMUX is a helper function to RegisterMUXLink.
 * This function will work to activate the necessary
 * interfaces and begin communication.
 *
 * Params:
 * mux -- the mux to be activated
 * ilist -- the interfacelist to be setup
 */
int32 ActivateMUX(MUX *mux, INTERFACELIST *ilist)
{
	int32 result;

	DEBUG("ActivateMUX(%p, %p)\n", mux, ilist);

	result =
	    CreateConfigInterface(REGISTER_CONFIG_CHANNEL, mux,
				  REGISTER_CONFIGDATA_COUNT,
				  REGISTER_CONFIGDATA, &ilist->config);
	if (result != ERROR_NONE)
		return DEBUGERROR(result);

	result =
	    CreateDirectInterface("direct", "netmux",
				  DIRECT_DYNAMIC_MAJOR_ASSIGNMENT,
				  REGISTER_DIRECT_MINCH,
				  REGISTER_DIRECT_MAXCH, mux,
				  (DIRECTINTERFACE **) &ilist->direct);
	if (result != ERROR_NONE) {
		DestroyConfigInterface(ilist->config);

		return DEBUGERROR(result);
	}

	result =
	    CreateTTYInterface("tty", "netmux",
			       TTY_DYNAMIC_MAJOR_ASSIGNMENT,
			       REGISTER_TTY_MINCH, REGISTER_TTY_MAXCH, mux,
			       (TTYINTERFACE **) &ilist->tty);
	if (result != ERROR_NONE) {
		DestroyConfigInterface(ilist->config);
		DestroyDirectInterface((DIRECTINTERFACE *) ilist->direct);

		return DEBUGERROR(result);
	}

	result =
	    CreateNetworkInterface("network", REGISTER_NETWORK_MINCH,
				   REGISTER_NETWORK_MAXCH, mux,
				   (NETWORKINTERFACE **) &ilist->network);
	if (result != ERROR_NONE) {
		DestroyConfigInterface(ilist->config);
		DestroyDirectInterface((DIRECTINTERFACE *) ilist->direct);
		DestroyTTYInterface((TTYINTERFACE *) ilist->tty);

		return DEBUGERROR(result);
	}

	result = ActivateConfigInterface(REGISTER_HOST, ilist->config);
	if (result != ERROR_NONE) {
		DestroyConfigInterface(ilist->config);

		DestroyDirectInterface((DIRECTINTERFACE *) ilist->direct);
		DestroyNetworkInterface((NETWORKINTERFACE *) ilist->
					network);
		DestroyTTYInterface((TTYINTERFACE *) ilist->tty);

		return DEBUGERROR(result);
	}

	return DEBUGERROR(ERROR_NONE);
}

/*
 * DeactivateMUX is a helper function to UnregisterMUXLink. This
 * function will help release any consumed resources and halt
 * any communication on a NetMUX.
 *
 * Params:
 * ilist -- the interface to be deactivated
 */
void DeactivateMUX(INTERFACELIST *ilist)
{
	DEBUG("DeactivateMUX(%p)\n", ilist);

	DestroyDirectInterface((DIRECTINTERFACE *) ilist->direct);
	DestroyNetworkInterface((NETWORKINTERFACE *) ilist->network);
	DestroyTTYInterface((TTYINTERFACE *) ilist->tty);

	DeactivateConfigInterface(ilist->config);
	DestroyConfigInterface(ilist->config);

	ilist->linkinterface.LinkInform((void *) LDP_INFORM_SHUTDOWN,
					(void *) ilist->muxinterface.id);

	DestroyMUX(ilist->mux);

	free_mem(ilist);
}

/*
 * GetNetmuxLogState is a read callback function of proc interface on NetMUX.
 *
 */

static int GetNetmuxLogState(char *buf, char **start, off_t offset,
			     int count, int *eof, void *data)
{
	int len;
	int index;

	len = LOG_COMMAND_LEN;
	index = 0;

	len = sprintf(buf, "%s\n", &NetmuxLogState[index]);
	*eof = 1;
	return len;
}

/*
 * WriteNetmuxLogCommand is a write callback
 * function of proc interface on NetMUX.
 *
 */

static int WriteNetmuxLogCommand(struct file *file, const char *buffer,
				 unsigned long count, void *data)
{
	int len;
	int index;

	len = count;
	index = 0;
	if (len > LOG_COMMAND_LEN) {
		printk(KERN_ERR "Error : Netmux log command is invalid\n");
		return -ENOSPC;
	}
	if (copy_from_user(&NetmuxLogState[index], buffer, len))
		return -EFAULT;
	NetmuxLogState[LOG_COMMAND_LEN - 1] = '\0';

	return len;
}

void NetmuxLogInit(void)
{
#ifdef CONFIG_DEBUG_NETMUX
	NetmuxLogState = alloc_mem(LOG_COMMAND_LEN);
	NetmuxLogState[0] = '0';
	NetmuxLogState[1] = '\0';

	proc_netmux_log_entry = create_proc_entry("netmuxlog", 0x660, 0);
	if ((!proc_netmux_log_entry) || (!NetmuxLogState)) {
		remove_proc_entry("netmuxlog", 0);
	} else {
		proc_netmux_log_entry->data = NetmuxLogState;
		proc_netmux_log_entry->read_proc = GetNetmuxLogState;
		proc_netmux_log_entry->write_proc = WriteNetmuxLogCommand;
	}
#endif
}
