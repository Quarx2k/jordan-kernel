/**  \file syspanic.c

 \brief This file contains the source to the syspanic kernel driver

 \if INCLUDE_LICENSE_SECTION
 * Copyright (C) 2009 Motorola, Inc.
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at
 * your option) any later version.  You should have
 * received a copy of the GNU General Public License
 * along with this program; if not, write to the Free
 * Software Foundation 51 Franklin Street, Fifth Floor
 * Boston, MA 02110-1301 USA
 \endif

  External Revision History:

 Modification Date | Release ID | Description of Changes \n
 ----------------- | ---------- | -------------------------------- \n
 -- 2009-02-18 --- | Version_01 | Initial Creation
 -- 2009-08-24 --- | Version_02 | To prevent suspend when BP panic happen
 -- 2010-05-10 --- | Version_03 | To prevent spin lock on kernel 2.6.32 version

 */

/*=============================================================================
 ................................INCLUDE FILES
 ============================================================================*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <mach/irqs.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/io.h>
#include <plat/mailbox.h>
#include <linux/delay.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
/*=============================================================================
 LOCAL CONSTANTS
 ============================================================================*/

/**
 * Identifier of the driver. Used in all kernel APIs requiring a text
 * string to identify the driver, and in creating the devfs entry
 */
#define SYSPANIC_ID    "syspanic"

/* Syspanic IRQ is 88 */
#define SYSPANIC_IRQ 88

/* #define SYSPDEBUG 1 */
#ifdef SYSPDEBUG
#  define DPRINTK(fmt, args...) printk(KERN_INFO "%s: " fmt, __func__ ,\
									## args)
#else
#  define DPRINTK(fmt, args...)
#endif
/*=============================================================================
 LOCAL MACROS
 =============================================================================*/

/*=============================================================================
 LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
 =============================================================================*/

enum sysp_state {
	SYSP_IDLE, SYSP_WAIT, SYSP_CLEAR, SYSP_PANIC, SYSP_MEMORY_PANIC
};

/**
 * Internal representation of the syspanic driver.
 */
struct SysPanicDriverType {
	wait_queue_head_t waitQueue; /**< Wait queue used for sleeping */
	struct timer_list timerList;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock syspanic_wake_lock;
#endif
	atomic_t state;
	mbox_msg_t panic_id;
};

/*=============================================================================
 LOCAL FUNCTION PROTOTYPES
 =============================================================================*/

/**
 * \brief Initializes the syspanic module when it's loaded into kernel
 *
 * \param None
 *
 * \return Return Type \n
 * - Zero - Success. \n
 * - Non-Zero - An error has occurred while initializing the driver. \n
 */
static int __init syspanicInit(void);

/**
 * \brief Cleans up the syspanic module when it's unloaded from kernel
 *
 * \param None
 *
 * \return None
 *
 */
static void __exit syspanicExit(void);

static void syspTimeout(unsigned long ptr);

/**
 * \brief Interrupt handler called when IRQ is received from BP syspanic
 *
 * \param irq   - Interrupt number
 * \param data  - Private data
 *
 * \return Return Type \n
 * - IRQ_HANDLED - Interrupt has been handled
 * - IRQ_NONE    - No real interrupt occurred
 */
static irqreturn_t irqHandler(int irq, void *data);



/**
 * \brief Callback function when user-space invokes read()
 *
 * \return Return size read
 */
static ssize_t syspanicRead(struct file *f, __user char *buf, size_t sz,
		loff_t *off);

/**
 * \brief Callback function when user-space invokes select()
 *
 * \param filp      - Pointer to open file structure
 * \param wait      - Opaque structure passed to kernel
 *
 * \return Return Type \n
 * - POLLIN - Data available for read
 * - 0 - No data available
 */
static unsigned int syspanicPoll(struct file *filp, poll_table * wait);

/**
 * \brief Wrapper function to request for interrupt
 */
static int syspanicRequestIRQ(void);

/**
 * \brief Wrapper function to free interrupt
 */
static void syspanicFreeIRQ(void);

/*=============================================================================
 GLOBAL VARIABLES
 =============================================================================*/

/**
 * This structure exposes the available I/O
 * operations of this driver to the kernel.
 * Each member points to a corresponding function
 */
static const struct file_operations syspanicFops = { .owner = THIS_MODULE,
		.poll = syspanicPoll,
		.read = syspanicRead, };

static struct class *syspanic_class;
struct device *syspanic_device_class;
static int major_syspanic;
static struct SysPanicDriverType sysp; /**< Internal structure of syspanic */
static struct omap_mbox *wrigley_mbox;
/*=============================================================================
 LOCAL FUNCTIONS
 =============================================================================*/

int wrigley_notifier(struct notifier_block *self, unsigned long len, void *msg)
{
	return NOTIFY_OK;
}

static struct notifier_block wrigley_mbox_notifier = {
	.notifier_call = wrigley_notifier,
};

static int __init syspanicInit(void)
{
	int ret;
	/* Allocate a major number for the character device */
	major_syspanic = register_chrdev(0, "syspanic", &syspanicFops);
	if (major_syspanic < 0) {
		printk(KERN_INFO "Unable to get a major for syspanic\n");
		return major_syspanic;
	}

	syspanic_class = class_create(THIS_MODULE, "syspanic");
	if (IS_ERR(syspanic_class)) {
		printk(KERN_INFO "Not able to do the class_create\n");
		unregister_chrdev(major_syspanic, "syspanic");
		return (int) syspanic_class;
	}

	syspanic_device_class =
	device_create(syspanic_class, NULL, MKDEV(major_syspanic, 0),
			NULL, "syspanic");
	if (IS_ERR(syspanic_device_class)) {
		class_destroy(syspanic_class);
		unregister_chrdev(major_syspanic, "syspanic");
		printk(KERN_INFO
				"Not able to do the class_device_create\n");
		return (int) syspanic_device_class;
	}

	wrigley_mbox = omap_mbox_get("wrigley",&wrigley_mbox_notifier);
	if (IS_ERR(wrigley_mbox)) {
		printk(KERN_INFO "Syspanic cannot get wrigley mbox\n");
		wrigley_mbox = NULL;
	}

	init_waitqueue_head(&sysp.waitQueue);

	init_timer(&sysp.timerList);
	sysp.timerList.data = 0;
	sysp.timerList.function = syspTimeout;

	atomic_set(&sysp.state, SYSP_IDLE);
	printk(KERN_INFO "Loaded SysPanic device driver\n");

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&sysp.syspanic_wake_lock,
			WAKE_LOCK_SUSPEND,
			"syspanic");
#endif

	/* Request for an interrupt from kernel */
	ret = syspanicRequestIRQ();
	if (ret < 0) {
		printk(KERN_ERR "Cannot request IRQ from kernel\n");
		syspanicFreeIRQ();
		return ret;
	}
	return 0;
}

static void __exit syspanicExit(void)
{
	if (major_syspanic >= 0) {
		device_destroy(syspanic_class, MKDEV(major_syspanic, 0));
		class_destroy(syspanic_class);
		unregister_chrdev(major_syspanic, "syspanic");
	}
	/* Free the requested IRQ */
	syspanicFreeIRQ();

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&sysp.syspanic_wake_lock);
#endif
	if (wrigley_mbox)
		omap_mbox_put(wrigley_mbox, &wrigley_mbox_notifier);

	printk(KERN_INFO "Unloaded SysPanic device driver\n");
}

static ssize_t syspanicRead(struct file *f, __user char *buf, size_t sz,
		loff_t *off)
{
	int len = sizeof(sysp.panic_id);

	if (*off == 0 && sz >= len) {
		if (copy_to_user(buf, &sysp.panic_id, len))
			return -EFAULT;
		*off += len;
		return len;
	}
	return 0;
}

static void syspTimeout(unsigned long ptr)
{
	if (atomic_read(&sysp.state) == SYSP_WAIT || atomic_read(&sysp.state)
			== SYSP_CLEAR) {
		atomic_set(&sysp.state, SYSP_MEMORY_PANIC);
		disable_irq(SYSPANIC_IRQ);
		DPRINTK("Syspanic Memory Panic\n");
		/* wake up panic daemon */
		wake_up_interruptible(&sysp.waitQueue);
	}
}

static irqreturn_t irqHandler(int irq, void *data)
{
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock(&sysp.syspanic_wake_lock);
#endif

	switch (atomic_read(&sysp.state)) {
	case SYSP_IDLE:
		atomic_set(&sysp.state, SYSP_WAIT);
		sysp.timerList.expires = jiffies + (HZ / 10); /*timeout 100ms*/
		add_timer(&sysp.timerList);
		DPRINTK("Syspanic first irq Panic\n");
		udelay(1000);
		break;
	case SYSP_WAIT:
		atomic_set(&sysp.state, SYSP_CLEAR);
		udelay(1000);
		break;
	case SYSP_CLEAR:
		atomic_set(&sysp.state, SYSP_PANIC);
		del_timer(&sysp.timerList);
		disable_irq_nosync(SYSPANIC_IRQ);
		/* wake up panic daemon */
		DPRINTK("Syspanic Standard BP Panic\n");
		wake_up_interruptible(&sysp.waitQueue);
		break;
	default:
		/* irq should be disabled */
		disable_irq_nosync(SYSPANIC_IRQ);
		break;
	}
	return IRQ_HANDLED;
}

static void sysGetPanicId(void)
{
	if (!wrigley_mbox)
		return;
	if (!wrigley_mbox->ops->fifo_empty(wrigley_mbox))
		sysp.panic_id = wrigley_mbox->ops->fifo_read(wrigley_mbox);
}

static unsigned int syspanicPoll(struct file *filp, poll_table * wait)
{
	poll_wait(filp, &sysp.waitQueue, wait);
	switch (atomic_read(&sysp.state)) {
	case SYSP_PANIC:
		sysGetPanicId();
		return POLLIN;
	case SYSP_MEMORY_PANIC:
		return POLLRDNORM;
	case SYSP_IDLE:
	case SYSP_WAIT:
	case SYSP_CLEAR:
	default:
		break;
	}
	return 0;
}

static int syspanicRequestIRQ(void)
{
	int retval;
	retval = request_irq(SYSPANIC_IRQ, irqHandler,
			IRQF_DISABLED, SYSPANIC_ID,	NULL);
	if (retval < 0) {
		printk(KERN_ERR "\nCan't get SysPanic IRQ\n");
		return retval;
	}
	enable_irq_wake(SYSPANIC_IRQ);
	return 0;
}

static void syspanicFreeIRQ(void)
{
	free_irq(SYSPANIC_IRQ, NULL);
	return;
}

module_init(syspanicInit);
module_exit(syspanicExit);

MODULE_AUTHOR("Motorola MDB");
MODULE_DESCRIPTION("Syspanic Driver");
MODULE_LICENSE("GPL");
