/******************************************************************************
 * NetMUX main.c                                                              *
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
 *   2007/04/29  Motorola    Removed DEBUG statement from init                *
 *   2007/05/01  Motorola    Use printk to replace DEBUG.                     *
 *   2007/12/05  Motorola    change code as kernel upgrade                    *
 *   2007/12/05  Motorola    Add NetmuxLogInit in init module                 *
 *   2009/07/23  Motorola    Add wake lock functionality                      *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* main.c defines entry points for the NetMUX.  Each entry point is unique    */
/* based on the current  environment.                                         */

#include "types.h"
#include "errorcodes.h"
#include "utility.h"
#include "register.h"
#include "debug.h"

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/wakelock.h>

/*
 * Decribe the module to Linux so it doesn't complain that
 * we are tainting the kernel.
 */
MODULE_DESCRIPTION("NetMUX driver for Linux");
MODULE_LICENSE("Dual BSD/GPL");


/*
 * Define an interface for the linkdrivers
 */
EXPORT_SYMBOL(RegisterMUXLink);
EXPORT_SYMBOL(UnregisterMUXLink);


struct wake_lock netmux_send_wakelock;
struct wake_lock netmux_receive_wakelock;

/*
 * A list of registered interfaces is defined in register.c
 */
extern INTERFACELIST *interfacelist;

/*
 * create /dev/netmux
 */
struct class *netmux_class = NULL;

/*
 * When the module is insmod'd this is the first thing that happens.
 * Currently the only thing that needs to be done is the initialization
 * of the utilities xpi.
 */
static int __init netmux_init(void)
{
	printk(KERN_INFO "Running NetMUX\n");
	initialize_utilities();

	netmux_class = class_create(THIS_MODULE, "netmux");
	if (IS_ERR(netmux_class)) {
		printk(KERN_ERR "Error in creating netmux_class class\n");
		return PTR_ERR(netmux_class);
	}

	NetmuxLogInit();

	wake_lock_init(&netmux_send_wakelock, WAKE_LOCK_SUSPEND,
		       "NETMUX_send");
	wake_lock_init(&netmux_receive_wakelock, WAKE_LOCK_SUSPEND,
		       "NETMUX_receive");

	return 0;
}

/*
 * When the NetMUX is rmmod'd we unregister ourself from all the
 * connected linkdrivers and free up any resources.
 */
static void __exit netmux_exit(void)
{
	printk(KERN_INFO "Cleaning Up NetMUX\n");
	while (interfacelist)
		DeactivateMUX(interfacelist);
	shutdown_utilities();

	wake_lock_destroy(&netmux_send_wakelock);
	wake_lock_destroy(&netmux_receive_wakelock);

	class_destroy(netmux_class);
}

device_initcall(netmux_init);
module_exit(netmux_exit);
