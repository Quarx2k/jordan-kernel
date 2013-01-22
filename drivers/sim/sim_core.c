/*
 * Copyright (C) 2005-2010 Motorola, Inc.
 */

/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 */

/*
 * This file includes all of the initialization and tear-down code for the SIM
 * low-level driver and the basic user-mode interface (open(), close(), ioctl(), etc.).
 */

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <plat/dma.h>

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqflags.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/pm.h>
#ifdef CONFIG_QUICK_WAKEUP
#include <linux/quickwakeup.h>
#include <linux/wakeup_timer_kernel.h>
#endif

#include <linux/regulator/consumer.h>

#include <plat/clock.h>
#include <plat/hardware.h>
#include <plat/prcm.h>
//#include <plat/resource.h>
#include <plat/omap-pm.h>

#include "smart_card.h"
#include "smart_card_kernel.h"

/******************************************************************************
 * Global Variables
 *****************************************************************************/

/* OMAP Clock Framework */
static struct clk *usim_fck;
static struct clk *usim_ick;
static struct clk *omap_96m_fck;
static struct clk *omap_120m_fck;

static struct pm_qos_request_list **sim_dev;
static struct device *sim_device;
/******************************************************************************
* Constants
******************************************************************************/
/*
  H/W BASE VIRTUAL ADDRESSES
  
  IMPORTANT NOTE(S) :
  
  1) each S/W platform may have their own memory map be careful
     to use the proper virtual address, as the virtual memory may not be contiguous
     like the physical memory is. 
*/
#define CM_BASE		 (L4_34XX_BASE + 0x4000)
#define CM_AUTOIDLE_WK (CM_BASE + 0x0C30)
#define CM_CLKSEL_WK (CM_BASE + 0x0C40)
#define PBIAS_CONTROL_LITE (0x48002520)
#define DMA_SYSCONFIG (0x4805602C)
#define CONTROL_WKUP_CTRL (0x48002A5C)
#define INTCPS_ITR1 (0x482000A0)
#define	WP_TPIR	(1 << 5)

/*
  OFFSET TO DMA REGISTER
  This is required to keep DMA from going to sleep on us as the lat APIs don't seem to work
*/

#define DMA_MIDLE 0x3000

/*
  OFFSETS TO PBIAS LITE REGISTERS

  IMPORTANT NOTE(S) :

  1) using direct register writes for PBIAS settings is not the prefferred method to do this. an API
     function should be provided to handle this, but i cannot find one as of yet. 
*/
#define PBIASVMODE1	0x0100
#define PBIASPWRDNZ1       0x0200
#define PBIASSPEEDCNTL1    0x0400
#define PBIASVMODEERROR1   0x0800


/*
  OFFSET TO CONTROL WKUP CTRL register

  IMPORTANT NOTE(S) :
  None

*/
#define CONTROL_WKUP_CTRL_MASK 			0x00000040

/*
  SIM SOURCE CLOCK SELECTION

  IMPORTANT NOTE(S) :
  None
*/
#define SIM_CLK_SEL_MASK   0x0078
#define SIM_CLK_120_DIV16  0x0048
#define SIM_CLK_96_DIV10   0x0030
#define SIM_CLK_SYSCLOCK   0x0008

#define SIM_ICLK_AUTOIDLE  0x0200

/*
  SIM MODULE CONSTANTS

  IMPORTANT NOTE(S) :
  None

*/
#define NO_OF_SIM_REGS     (sizeof(SIM_MODULE_REGISTER_BANK) / sizeof(UINT32))
#define SIM_REMOVED 0
#define SIM_PRESENT 1
#define SIM_MODULE_FREQUENCY_4  3750000
#define SIM_MODULE_FREQUENCY_5 4800000
#define SIM_MODULE_NACK_THRESHOLD 4
#define SIM_MODULE_RX_DMA_REQ     79

#define CMD_PARAMETER_SIZE_1 (sizeof(UINT32))
#define CMD_PARAMETER_SIZE_2 (2 * sizeof(UINT32))
#define CMD_PARAMETER_SIZE_3 (3 * sizeof(UINT32))
#define CMD_PARAMETER_SIZE_4 (4 * sizeof(UINT32))
#define CMD_PARAMETER_SIZE_5 (5 * sizeof(UINT32))

/*
  SIM MODULE OMAP CLOCK FRAMEWORK CONSTANTS

  IMPORTANT NOTE(S) :
  None

*/
#define USIM_FCK "usim_fck"
#define USIM_ICK "usim_ick"
#define OMAP_96M_FCK "omap_96m_fck"
#define OMAP_120M_FCK "dpll5_m2_ck"

/*
  SIM CARD TYPE CONSTANTS

  IMPORTANT NOTE(S) :
  These variables must match the values from user space.
*/
#define UNKNOWN_CARD 0x04
#define GSM_SIM      0x09
#define UICC	     0x0A

UINT8 uicc_status[5] = {0x80, 0xF2, 0x00, 0x0C, 0x00};
UINT8 gsm_status[5]  = {0xA0, 0xF2, 0x00, 0x00, 0x16};

#define SIM_NULL_PROC_BYTE 0x60
#define SIM_STATUS_INS 0xF2
#define SIM_ACK_ONE_MASK 0xFF

/******************************************************************************
 * Local Macros
 *****************************************************************************/
/*
  SIM MODULE TRACEMSG MACRO

  IMPORTANT NOTE(S) :

  Do not enable logging by default. Only use this for debug purposes.
*/
#if 0
#define tracemsg(fmt,args...)  printk(fmt,##args)
#else
#define tracemsg(fmt,args...)
#endif



/*
  SIM MODULE REGISTER ACCESS MACROS

  IMPORTANT NOTE(S) :

  1) to be used only for SIM module register access
*/

#define read_reg(reg)		   (omap_readl((u32)(reg)))
#define write_reg(reg, value)	   (omap_writel((value), (u32)(reg)))
#define write_reg_bits(reg, mask, bits) (omap_writel((omap_readl((u32)(reg)) & ~(mask)) | (bits), (u32)(reg)))

#define SIM_STATUS_ACK_ALL(byte)       ((SIM_STATUS_INS ^ byte) == 0)
#define SIM_STATUS_ACK_ONE(byte)       (((SIM_STATUS_INS ^ byte) & SIM_ACK_ONE_MASK) \
					 == SIM_ACK_ONE_MASK)

#define expected_status_data(sim_card) (sim_card == UICC ? 2 : 24)

/******************************************************************************
* Local type definitions
******************************************************************************/

typedef struct {
	UINT16 buffer_index;
	UINT16 rx_last_index;
	UINT16 tx_length;
	UINT8 error_flag;
	UINT8 *buffer;
	dma_addr_t dma_buffer;
} SIM_MODULE_BUFFER_DATA;

/******************************************************************************
* Global function prototypes
******************************************************************************/

/******************************************************************************
* Local function prototypes
******************************************************************************/
static long sim_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int sim_open(struct inode *inode, struct file *file);
static int sim_free(struct inode *inode, struct file *file);
static unsigned int sim_poll(struct file *file, poll_table * wait);
static int sim_suspend(struct platform_device *pdev, pm_message_t state);
static int sim_resume(struct platform_device *pdev);
static irqreturn_t sim_module_int_irq_1(int irq, void *dev_id);

static void sim_module_int_reset_detect(UINT8 reader_id);
static void sim_module_int_rx(UINT8 reader_id);
static void sim_module_init_rx_mode(UINT8 reader_id);
static void sim_module_int_tx(UINT8 reader_id);
static void sim_module_set_voltage_level(SIM_MODULE_VOLTAGE_LEVEL level);
static void sim_module_set_clock_rate(UINT8 reader_id,
				      SIM_MODULE_CLOCK_RATE rate);

static int sim_probe(struct platform_device *pdev);
static int sim_remove(struct platform_device *pdev);
static int sim_slim_status_handler(void);
static BOOL sim_mutex_update(UINT8 mutex_request);
static void sim_module_dma_callback(INT32 lch, UINT16 ch_status,
				    void *data);
static int sim_qw_callback(void);
static int sim_qw_check(void);
static int start_clk_stop_procedure(UINT8 reader_id, UINT32 clock_count);

static int regulator_enabled_flag;

/******************************************************************************
* Local Structures
******************************************************************************/

/* Platform driver structure for the SIM driver */
static struct platform_driver sim_driver = {
	.driver = {
		   .name = SIM_DEV_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = sim_probe,
	.remove = sim_remove,
	.suspend = sim_suspend,
	.resume = sim_resume,
};

/*This structure defines the file operations for the SIM device */
static struct file_operations sim_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = sim_ioctl,
	.open = sim_open,
	.release = sim_free,
	.poll = sim_poll,
};

#ifdef CONFIG_QUICK_WAKEUP
/* Used for the timer call back */
static struct quickwakeup_ops sim_qw_ops = {
	.qw_callback = sim_qw_callback,
	.qw_check = sim_qw_check
};
#endif
/******************************************************************************
* Local variables
******************************************************************************/
static int sim_module_major;
static int sim_module_dma_channel = 0;

static DEFINE_SPINLOCK(sim_module_lock);
static DEFINE_SPINLOCK(sim_status_lock);

static UINT32 sim_module_clock_frequency = SIM_MODULE_FREQUENCY_4;

static wait_queue_head_t sim_module_wait =
__WAIT_QUEUE_HEAD_INITIALIZER(sim_module_wait);

static struct class *sim_class;

static UINT32 sim_module_rx_event = SIM_MODULE_EVENT_NONE;

static BOOLEAN sim_module_all_tx_data_sent = FALSE;

static UINT8 sim_module_current_pd_state_sim1 = SIM_REMOVED;
static UINT8 sim_module_nack_counter = 0;
static UINT8 sim_module_opens = 0;

static BOOLEAN sim_low_power_enabled = FALSE;

static SIM_MODULE_INTERRUPT_MODE sim_module_interrupt_mode =
    SIM_MODULE_RX_MODE;
static SIM_MODULE_BUFFER_DATA sim_module_card_data[NUM_SIM_MODULES];

static volatile SIM_MODULE_REGISTER_BANK *sim_registers[NUM_SIM_MODULES] = {
	((volatile SIM_MODULE_REGISTER_BANK *) SIM1_BASE_ADDR)
};

static volatile SIM_MODULE_REGISTER_BANK
    *sim_phys_registers[NUM_SIM_MODULES] = {
	(volatile SIM_MODULE_REGISTER_BANK *) SIM1_BASE_ADDR
};

struct regulator *vsim_regulator;
struct regulator *vsimcard_regulator;

static UINT16 card_type = UNKNOWN_CARD;
static BOOL preforming_status_command = FALSE;
static BOOL status_slow_card = FALSE;
static BOOL currently_in_call = FALSE;
static BOOL proc_bytes_done = FALSE;

static BOOL all_data_received = FALSE;
static UINT16 bytes_recieved = 0;

static UINT8 sim_mutex_locked_by_id = 0;
static UINT8 sim_mutex_requested_by_id = 0;

static UINT32 sim_poll_interval = 0;
static UINT8 last_byte_checked = 0;
static UINT16 null_byte_counter = 0;
static UINT8 ack_counter = 0;

static struct timer_cascade_root * sim_timer = NULL;

/******************************************************************************
* Local Functions
******************************************************************************/

/* DESCRIPTION:
       The IOCTL handler for the SIM device
 
   INPUTS:
       inode       inode pointer
       file	file pointer
       cmd	 the ioctl() command
       arg	 the ioctl() argument

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.   
*/

static long sim_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int status = 0;

	UINT32 args_kernel[CMD_PARAMETER_SIZE_5] = { 0, 0, 0, 0, 0 };
	volatile UINT32 *sim_module_register_ptr = NULL;

	unsigned long flags;

	UINT8 tx_index;
	UINT8 length;
	int error;

	tracemsg("SIM KERNEL IOCTL -> %X \n", cmd);
	/*
	   These IO control requests must only be used by the protocol layer in user space.
	   Any deviation from this rule WILL cause problems.
	 */

	switch (cmd) {
	case SIM_IOCTL_CONFIGURE_GPIO:

		/* enable the interface clock */
		clk_enable(usim_ick);

		/* enable auto idle mode for the interface clock */
		write_reg_bits((volatile UINT32 *) CM_AUTOIDLE_WK,
			       SIM_ICLK_AUTOIDLE, SIM_ICLK_AUTOIDLE);

		/* enable the functional clock */
		clk_enable(usim_fck);

		/* sleep some time waiting for clocks to stabalize */
		msleep(1);

		/* now that we've got the interface clock configured, reset the SIM module */
		write_reg_bits(&(sim_registers[(UINT8) arg]->sysconfig),
			       SOFTRESET_MASK, SOFTRESET_MASK);

		tracemsg("SIM Module: soft reset SIM module\n\n");

		/* block while the module is in reset ... */
		while ((read_reg(&(sim_registers[(UINT8) arg]->sysstatus))
			& RESETDONE_MASK) == 0) {
			/* yield the processor one time per iteration */
			msleep(1);
			tracemsg
			    ("SIM Module: soft reset not complete yet\n\n");
		}

		/* set the internal SIM clock divider to divide by 2 to obtain
		   a SIM clock of 3.75 MHz */
		write_reg_bits(&(sim_registers[(UINT8) arg]->usimconf2),
			       CONFSCLKDIV_MASK, 0);

		sim_module_set_clock_rate((UINT8) arg, SIM_MODULE_CLOCK_4);

		/* turn the clock back off for now */
		clk_disable(usim_fck);

		break;
	case SIM_IOCTL_READ_DATA_BUFFER:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_4);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg
			    ("buffer_index(read data buffer): %X offset: %X rx_length: %X\n",
			     sim_module_card_data[args_kernel[0]].
			     buffer_index, args_kernel[2], args_kernel[3]);

			if (copy_to_user
			    ((UINT8 *) args_kernel[1],
			     (&
			      (sim_module_card_data[args_kernel[0]].
			       buffer[args_kernel[2]])), args_kernel[3])) {
				tracemsg
				    ("Warning: failed to copy data to user-space while retrieving SIM card reader data for module 2\n");
				status = -EFAULT;
			}
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_UPDATE_DATA_BUFFER:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_3);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			status =
			    copy_from_user(sim_module_card_data
					   [args_kernel[0]].buffer,
					   (UINT8 *) args_kernel[1],
					   args_kernel[2]);
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_INTERRUPT_INIT:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_3);

		if ((UINT8) args_kernel[0] == SIM_MODULE_1) {
			status =
			    request_irq(INT_SIM_GENERAL,
					sim_module_int_irq_1,
					(IRQF_DISABLED | IRQF_NOBALANCING),
					SIM_DEV_NAME, 0);

			if (status != 0) {
				tracemsg
				    ("Warning: the SIM driver failed to request an IRQ. IRQ -> %X Error -> %X\n",
				     INT_SIM_GENERAL, status);
			}
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_INTERRUPT_MODE:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if ((UINT8) args_kernel[0] == SIM_MODULE_1) {
			tracemsg("Set SIM interrupt mode -> arg1: %X\n",
				 args_kernel[1]);

			switch (args_kernel[1]) {
			case SIM_MODULE_RX_MODE:
				tracemsg("SIM Module: RX mode \n\n");
				sim_module_interrupt_mode =
				    SIM_MODULE_RX_MODE;

				if (sim_module_dma_channel != 0) {
					omap_free_dma
					    (sim_module_dma_channel);
					sim_module_dma_channel = 0;
				}
				/* request a DMA logical channel */
				error =
				    omap_request_dma(OMAP34XX_DMA_USIM_RX,
						     SIM_DEV_NAME,
						     sim_module_dma_callback,
						     (void *)
						     sim_module_card_data
						     [args_kernel[0]].
						     dma_buffer,
						     &sim_module_dma_channel);
				if (error == 0) {
					/* configure the DMA parameters */
					omap_set_dma_transfer_params
					    (sim_module_dma_channel,
					     OMAP_DMA_DATA_TYPE_S8, 1,
					     SIM_MODULE_MAX_DATA,
					     OMAP_DMA_SYNC_ELEMENT,
					     OMAP34XX_DMA_USIM_RX,
					     OMAP_DMA_SRC_SYNC);
					omap_set_dma_src_params
					    (sim_module_dma_channel, 0,
					     OMAP_DMA_AMODE_CONSTANT,
					     (unsigned
					      long) (&(sim_phys_registers
						       [args_kernel[0]]->
						       usim_drx)), 0, 0);
					omap_set_dma_dest_params
					    (sim_module_dma_channel, 0,
					     OMAP_DMA_AMODE_POST_INC,
					     (unsigned long)
					     sim_module_card_data
					     [args_kernel[0]].dma_buffer,
					     0, 0);



					omap_start_dma
					    (sim_module_dma_channel);
				}

				if (error != 0) {
					tracemsg
					    ("SIM MODULE DMA CONFIGURATION ERROR -> %X \n",
					     error);
				}

				break;
			case SIM_MODULE_TX_MODE:
				tracemsg("SIM Module: TX mode \n\n");
				sim_module_interrupt_mode =
				    SIM_MODULE_TX_MODE;
				break;
			case SIM_MODULE_RESET_DETECT_MODE:
				tracemsg("SIM Module: reset detect \n\n");

				sim_module_card_data[args_kernel[0]].
				    buffer_index = 0;

				if (sim_module_dma_channel != 0) {
					omap_free_dma
					    (sim_module_dma_channel);
					sim_module_dma_channel = 0;
				}

				sim_module_interrupt_mode =
				    SIM_MODULE_RESET_DETECT_MODE;
				break;
                        case SIM_MODULE_MODE_NONE:
				sim_module_interrupt_mode =
					SIM_MODULE_MODE_NONE;
				break;
			default:
				tracemsg
				    ("Warning: Invalid SIM interrupt mode. \n");
				break;
			}
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_WRITE_SIM_REG_DATA:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_3);

		tracemsg
		    ("Write SIM module register data -> reader ID: %X register: %X data: %X\n",
		     args_kernel[0], args_kernel[1], args_kernel[2]);

		if (args_kernel[1] <= sizeof(SIM_MODULE_REGISTER_BANK)) {
			if (args_kernel[0] < NUM_SIM_MODULES) {
				sim_module_register_ptr =
				    (UINT32 *) ((UINT32)
						sim_registers[args_kernel
							      [0]] +
						(UINT32) args_kernel[1]);
			} else {
				tracemsg
				    ("Warning: Invalid reader ID in SIM driver request.\n");
				status = -EFAULT;
			}

			if (sim_module_register_ptr != NULL) {
				write_reg(sim_module_register_ptr,
					  args_kernel[2]);

				/* if writing to the transmit FIFO increment the buffer index */
				if (args_kernel[1] == 0x0000003C) {
					sim_module_card_data[args_kernel
							     [0]].
					    buffer_index++;
				}
			}
		} else {
			tracemsg
			    ("Warning: Invalid register address in the SIM driver.\n");
			status = -EFAULT;
		}

		sim_module_register_ptr = NULL;
		break;
	case SIM_IOCTL_READ_SIM_REG_DATA:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_3);
		tracemsg
		    ("Read SIM register data -> Reader ID: %X, register: %X \n",
		     args_kernel[0], args_kernel[1]);
		if (args_kernel[1] <= sizeof(SIM_MODULE_REGISTER_BANK)) {
			if (args_kernel[0] < NUM_SIM_MODULES) {
				sim_module_register_ptr =
				    (UINT32 *) ((UINT32)
						sim_registers[args_kernel
							      [0]] +
						(UINT32) args_kernel[1]);
			} else {
				tracemsg
				    ("Warning: Invalid reader ID in SIM driver request.\n");
				status = -EFAULT;
			}

			if (sim_module_register_ptr != NULL) {
				args_kernel[2] =
				    read_reg(sim_module_register_ptr);

				if (copy_to_user
				    ((UINT32 *) arg, args_kernel,
				     CMD_PARAMETER_SIZE_3)) {
					tracemsg
					    ("Warning: failed to copy data to user-space while reading SIM module register data.\n");
					status = -EFAULT;
				}
			}
		} else {
			tracemsg
			    ("Warning: Invalid register address in the SIM driver.\n");
			status = -EFAULT;
		}

		sim_module_register_ptr = NULL;
		break;
	case SIM_IOCTL_WRITE_SIM_REG_BIT:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_4);

		tracemsg
		    ("Write SIM register bits -> reader ID: %X register: %X bits: %X mask: %X\n",
		     args_kernel[0], args_kernel[2], args_kernel[3],
		     args_kernel[1]);

		if (args_kernel[2] <= sizeof(SIM_MODULE_REGISTER_BANK)) {
			if (args_kernel[0] < NUM_SIM_MODULES) {
				sim_module_register_ptr =
				    (UINT32 *) ((UINT32)
						sim_registers[args_kernel
							      [0]] +
						(UINT32) args_kernel[2]);
			} else {
				tracemsg
				    ("Warning: Invalid reader ID in SIM driver request.\n");
				status = -EFAULT;
			}

			if (sim_module_register_ptr != NULL) {
				write_reg_bits(sim_module_register_ptr,
					       args_kernel[1],
					       args_kernel[3]);

				/* if writing to the transmit FIFO increment the buffer index */
				if (args_kernel[2] == 0x0000003C) {
					sim_module_card_data[args_kernel
							     [0]].
					    buffer_index++;
				}
			}
		} else {
			tracemsg
			    ("Warning: Invalid register address in the SIM driver.\n");
			status = -EFAULT;
		}

		sim_module_register_ptr = NULL;
		break;
	case SIM_IOCTL_SET_SIM_CLOCK_RATE:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		tracemsg
		    ("Set SIM clock rate, reader id -> %X, rate -> %X\n",
		     args_kernel[0], args_kernel[1]);

		sim_module_set_clock_rate(args_kernel[0], args_kernel[1]);
		break;
	case SIM_IOCTL_READ_RX_EVENT:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			args_kernel[1] = sim_module_rx_event;
			if (copy_to_user
			    ((UINT32 *) arg, args_kernel,
			     CMD_PARAMETER_SIZE_2)) {
				tracemsg
				    ("Warning: failed to copy data to user-space while reading SIM RX event type.\n");
				status = -EFAULT;
			}

			sim_module_rx_event = SIM_MODULE_EVENT_NONE;
		}

		else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}

		break;

	case SIM_IOCTL_SET_VOLTAGE_LEVEL:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			sim_module_set_voltage_level(args_kernel[1]);
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_READ_BUFFER_INDEX:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM reader buffer index\n");
			if (sim_module_dma_channel != 0) {
				/* if a DMA transfer already started ... */
				if (omap_get_dma_dst_pos
				    (sim_module_dma_channel) != 0) {
					/* set the number of characters received via DMA */
					args_kernel[1] =
					    (UINT32) (omap_get_dma_dst_pos
						      (sim_module_dma_channel)
						      -
						      sim_module_card_data
						      [args_kernel[0]].
						      dma_buffer);
				}

				/* else, DMA has not yet started ... */
				else {
					/* set the number of characters received via DMA to 0 */
					args_kernel[1] = 0;
				}
			}

			else {
				args_kernel[1] =
				    sim_module_card_data[args_kernel[0]].
				    buffer_index;
			}

			if (copy_to_user
			    ((UINT32 *) arg, args_kernel,
			     CMD_PARAMETER_SIZE_2)) {
				tracemsg
				    ("Warning: failed to copy data to user-space during buffer index retrieval\n");
				status = -EFAULT;
			}
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}

		break;
	case SIM_IOCTL_RESET_CARD_READER_DATA:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_5);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			sim_module_card_data[args_kernel[0]].buffer_index =
			    args_kernel[1];
			sim_module_card_data[args_kernel[0]].
			    rx_last_index = args_kernel[2];
			sim_module_card_data[args_kernel[0]].tx_length =
			    args_kernel[3];
			sim_module_card_data[args_kernel[0]].error_flag =
			    args_kernel[4];
			preforming_status_command = FALSE;
			status_slow_card = FALSE;
			all_data_received = FALSE;
			bytes_recieved = 0;
			last_byte_checked = 0;
			null_byte_counter = 0;
			ack_counter = 0;
                        proc_bytes_done = FALSE;

			tracemsg
			    ("reset card reader data -> buffer_index: %X rx_last_index: %X tx_length: %X error_flag: %X\n",
			     args_kernel[1], args_kernel[2],
			     args_kernel[3], args_kernel[4]);
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}

		break;
	case SIM_IOCTL_UPDATE_RX_LAST_INDEX:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM update RX last index, index -> %X\n",
				 args_kernel[1]);
			sim_module_card_data[args_kernel[0]].
			    rx_last_index = args_kernel[1];
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_UPDATE_BUFFER_INDEX:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM update buffer index, index -> %X\n",
				 args_kernel[1]);
			sim_module_card_data[args_kernel[0]].buffer_index =
			    args_kernel[1];
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_ALL_TX_DATA_SENT:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM read all TX data sent\n");
			args_kernel[1] = sim_module_all_tx_data_sent;

			if (copy_to_user
			    ((UINT32 *) arg, args_kernel,
			     CMD_PARAMETER_SIZE_2)) {
				tracemsg
				    ("Warning: failed to copy data to user-space during card reader data retrieval\n");
				status = -EFAULT;
			}
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;
	case SIM_IOCTL_RESET_ALL_TX_DATA_SENT:
		if (arg < NUM_SIM_MODULES) {
			tracemsg("SIM reset all TX data sent\n");
			sim_module_all_tx_data_sent = FALSE;
		} else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}
		break;

	case SIM_IOCTL_DATA_TX:

		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM data TX\n");
			length = (UINT8) args_kernel[1];

			sim_module_card_data[args_kernel[0]].buffer_index =
			    0;

			/* Enable DMA mode */
			write_reg_bits(&
				       (sim_registers[args_kernel[0]]->
					usim_fifos), SIM_DMA_MODE_MASK,
				       SIM_DMA_MODE_MASK);

			if (sim_module_dma_channel != 0) {
				omap_free_dma(sim_module_dma_channel);
				sim_module_dma_channel = 0;
			}

			/* request a DMA logical channel */
			error =
			    omap_request_dma(OMAP34XX_DMA_USIM_RX,
					     SIM_DEV_NAME,
					     sim_module_dma_callback,
					     (void *)
					     sim_module_card_data
					     [args_kernel[0]].dma_buffer,
					     &sim_module_dma_channel);
			if (error == 0) {

				/* configure the DMA parameters */
				omap_set_dma_transfer_params
				    (sim_module_dma_channel,
				     OMAP_DMA_DATA_TYPE_S8, 1,
				     SIM_MODULE_MAX_DATA,
				     OMAP_DMA_SYNC_ELEMENT,
				     OMAP34XX_DMA_USIM_RX,
				     OMAP_DMA_SRC_SYNC);
				omap_set_dma_src_params
				    (sim_module_dma_channel, 0,
				     OMAP_DMA_AMODE_CONSTANT,
				     (unsigned
				      long) (&(sim_phys_registers
					       [args_kernel[0]]->
					       usim_drx)), 0, 0);
				omap_set_dma_dest_params
				    (sim_module_dma_channel, 0,
				     OMAP_DMA_AMODE_POST_INC,
				     (unsigned long)
				     sim_module_card_data[args_kernel[0]].
				     dma_buffer, 0, 0);

				omap_start_dma(sim_module_dma_channel);
			}

			local_irq_save(flags);

			/* for each character to transmit ... */
			for (tx_index = 0; tx_index < length; tx_index++) {
				/* copy the byte to the TX FIFO */
				write_reg(&
					  (sim_registers[args_kernel[0]]->
					   usim_dtx),
					  sim_module_card_data[args_kernel
							       [0]].
					  buffer[sim_module_card_data
						 [args_kernel[0]].
						 buffer_index++]);
			}

			/* setup the TX threshold value to write more data into the FIFO */
			write_reg_bits(&
				       (sim_registers[args_kernel[0]]->
					usim_fifos), FIFO_TX_TRIGGER_MASK,
				       (SIM_MODULE_TX_FIFO_SIZE - 1) << 2);

			/* clear the transmit complete interrupt */
			write_reg_bits(&
				       (sim_registers[args_kernel[0]]->
					irqstatus), USIM_TX_MASK,
				       USIM_TX_MASK);

			/* enable the transmit complete interrupt */
			write_reg_bits(&
				       (sim_registers[args_kernel[0]]->
					irqenable), USIM_TX_EN_MASK,
				       USIM_TX_EN_MASK);

			/* enable the transmitter */
			write_reg_bits(&
				       (sim_registers[args_kernel[0]]->
					usimconf2), TXNRX_MASK,
				       TXNRX_MASK);

			/* if there is just one block of data to send ... */
			if (sim_module_card_data[args_kernel[0]].
			    tx_length <= SIM_MODULE_TX_FIFO_SIZE) {
				sim_module_card_data[args_kernel[0]].
				    buffer_index = 0;

				/* enable the receiver right away */
				write_reg_bits(&
					       (sim_registers
						[args_kernel[0]]->
						usimconf2), TXNRX_MASK, 0);

				/* indicate this is the last block */
				sim_module_all_tx_data_sent = TRUE;

				/* initialize rx mode */
				sim_module_init_rx_mode(args_kernel[0]);

			}

			local_irq_restore(flags);
		}

		break;

	case SIM_IOCTL_GET_SIM_CLOCK_FREQ:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM get SIM clock frequency\n");
			args_kernel[1] = sim_module_clock_frequency;
			if (copy_to_user
			    ((UINT32 *) arg, args_kernel,
			     CMD_PARAMETER_SIZE_2)) {
				tracemsg
				    ("Warning: failed to copy data to user-space for get frequency\n");
				status = -EFAULT;
			}
		}

		else {
			tracemsg
			    ("Warning: Invalid reader ID in SIM driver request.\n");
			status = -EFAULT;
		}

		break;

	case SIM_IOCTL_LOW_POWER_STATE:
		status =
		    copy_from_user(args_kernel, (UINT32 *) arg,
				   CMD_PARAMETER_SIZE_2);

		if (args_kernel[0] < NUM_SIM_MODULES) {
			tracemsg("SIM set constraint, state -> %X\n",
				 args_kernel[1]);
			/*  if we are active ...  */
			if ((BOOL) args_kernel[1] == FALSE) {
				/* Stop DMA from AKC'ing idle requests */
				write_reg_bits((volatile UINT32 *)DMA_SYSCONFIG,DMA_MIDLE, 
				    DMA_SYSCONFIG_MIDLEMODE(1));

				/* Request the latency constraint */
				omap_pm_set_max_mpu_wakeup_lat(sim_dev, 10);

				sim_low_power_enabled = (BOOL) args_kernel[1];
				/* enable the SIM FCLK */
				clk_enable(usim_fck);

				/* wait for the clock to settle */
				msleep(1);
			}

			/* else, we are inactive ... */
			else {
				/* Allow DMA to ACK idle requests */
				write_reg_bits((volatile UINT32 *)DMA_SYSCONFIG,DMA_MIDLE, 
				    DMA_SYSCONFIG_MIDLEMODE(2));

				/* Disable DMA mode for low power mode */
				write_reg_bits(&
					       (sim_registers
						[args_kernel[0]]->
						usim_fifos),
					       SIM_DMA_MODE_MASK, 0);

				/* disable the SIM FCLK */
				clk_disable(usim_fck);
				sim_low_power_enabled = (BOOL) args_kernel[1];

				/* Release the latency constraint */
				omap_pm_set_max_mpu_wakeup_lat(sim_dev, -1);
			}

		}
		break;

	case SIM_IOCTL_SLEEP:
		msleep(arg);
		break;

	case SIM_IOCTL_CARD_TYPE:
		if(copy_from_user(args_kernel, (UINT32 *) arg, CMD_PARAMETER_SIZE_2)){
			tracemsg("Warning: failed to copy data from user-space for card type\n");
			status = -EFAULT;
		}
		else {
			if (args_kernel[0] < NUM_SIM_MODULES) {
				if((args_kernel[1] == UICC) ||(args_kernel[1] == GSM_SIM)){
					card_type = args_kernel[1];
				}
				else {
					card_type = UNKNOWN_CARD;
				}
			}
			else {
				tracemsg("Warning: Invalid reader ID in SIM driver request.\n");
				status = -EFAULT;
			}
		}
		break;

	case SIM_IOCTL_MUTEX_UPDATE:
		if(copy_from_user(args_kernel, (UINT32 *) arg, CMD_PARAMETER_SIZE_3)){
			tracemsg("Warning: failed to copy data from user-space for mutex update\n");
			status = -EFAULT;
		}
		else {
    
			if (args_kernel[0] < NUM_SIM_MODULES) {
				args_kernel[2] = sim_mutex_update(args_kernel[1]);

				if (copy_to_user((UINT32 *) arg, args_kernel,CMD_PARAMETER_SIZE_3)) {
					tracemsg("Warning: failed to copy data to user-space for get mutex update\n");
					status = -EFAULT;
				}
			}
			else {
				tracemsg("Warning: Invalid reader ID in SIM driver request.\n");
				status = -EFAULT;
			}
		}
		break;

	case SIM_IOCTL_POLL_INTERVAL:
		if(copy_from_user(args_kernel, (UINT32 *) arg, CMD_PARAMETER_SIZE_2)) {
			tracemsg("Warning: failed to copy data from user-space for poll interval\n");
			status = -EFAULT;
		}
		else {

			if (args_kernel[0] < NUM_SIM_MODULES) {
				sim_poll_interval = args_kernel[1];
			} 
			else {
				tracemsg("Warning: Invalid reader ID in SIM driver request.\n");
				status = -EFAULT;
			} 
		} 
		break;

	case SIM_IOCTL_CALL_STATUS:
		if(copy_from_user(args_kernel, (UINT32 *) arg, CMD_PARAMETER_SIZE_2)) {
			tracemsg("Warning: failed to copy data from user-space for call status\n");
			status = -EFAULT;
		}
		else {

			if (args_kernel[0] < NUM_SIM_MODULES) {
				currently_in_call = args_kernel[1];
			}
			else {
				tracemsg("Warning: Invalid reader ID in SIM driver request.\n");
				status = -EFAULT;
			}
		}
		break;
#ifdef CONFIG_QUICK_WAKEUP
	case SIM_IOCTL_START_TIMER:
	        wakeup_start_status_timer(sim_timer, arg);
	        break;

	case SIM_IOCTL_STOP_TIMER:
	        wakeup_stop_status_timer(sim_timer);
	        break;
#endif

	case SIM_IOCTL_START_STOP_CLK_PROCEDURE:
		if (copy_from_user(args_kernel, (UINT32 *) arg,
			CMD_PARAMETER_SIZE_2)) {
			tracemsg("%s%s",
				"Warning: failed to copy data from ",
				"user-space for start clock stop procedure\n");
			status = -EFAULT;
		} else {
			if (args_kernel[0] < NUM_SIM_MODULES) {
				status = start_clk_stop_procedure(
						args_kernel[0], args_kernel[1]);
			} else {
				tracemsg("%s%s",
					"Warning: Invalid reader ID ",
					"in SIM driver request.\n");
				status = -EFAULT;
			}
		}
		break;

	default:
		tracemsg
		    ("Warning: Invalid request sent to the SIM driver.\n");
		status = -ENOTTY;
		break;
	}

	return status;
}

/* DESCRIPTION:
       The open() handler for the SIM device.
 
   INPUTS:
       inode       inode pointer
       file	file pointer

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.   
*/
static int sim_open(struct inode *inode, struct file *file)
{
	int status = 0;

	spin_lock(&sim_module_lock);

	/* only allow 1 open */
	if (sim_module_opens > 0) {
		spin_unlock(&sim_module_lock);

		status = -ENODEV;
	}

	if (status == 0) {
		sim_module_opens++;
		spin_unlock(&sim_module_lock);

		/* allocate non-bufferable, or cacheable memory for DMA */
		sim_module_card_data[SIM_MODULE_1].buffer =
		    (UINT8 *) dma_alloc_coherent(NULL, SIM_MODULE_MAX_DATA,
						 &(sim_module_card_data
						   [SIM_MODULE_1].
						   dma_buffer), 0);

		/* if there is not a valid buffer */
		if ((sim_module_card_data[SIM_MODULE_1].buffer == NULL) ||
		    (sim_module_card_data[SIM_MODULE_1].dma_buffer == 0)) {
			tracemsg
			    ("The data buffer was not allocated buffer -> %X, dma_buffer -> %X\n",
			     (unsigned int)
			     sim_module_card_data[SIM_MODULE_1].buffer,
			     (unsigned int)
			     sim_module_card_data[SIM_MODULE_1].
			     dma_buffer);
			status = -EFAULT;
		}
	}

	if (status == 0) {
		/* initialize the dma channel to 0 */
		sim_module_dma_channel = 0;

	}

	tracemsg("sim : sim_open, status -> %X\n", status);

	return status;
}

/* DESCRIPTION:
       The close() handler for the SIM device
 
   INPUTS:
       inode       inode pointer
       file	file pointer

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.   
*/
static int sim_free(struct inode *inode, struct file *file)
{
	tracemsg("sim : sim_free()\n");
	spin_lock(&sim_module_lock);
	sim_module_opens--;
	spin_unlock(&sim_module_lock);
	return 0;
}

/* DESCRIPTION:
       The poll() handler for the SIM driver
 
   INPUTS:
       file	file pointer
       wait	poll table for this poll()

   OUTPUTS:
       Returns 0 if no data to read or POLLIN if data available.

   IMPORTANT NOTES:
       None.   
*/
static unsigned int sim_poll(struct file *file, poll_table * wait)
{
	unsigned int retval = 0;

	if (sim_module_rx_event == SIM_MODULE_EVENT_NONE) {
		/* Add our wait queue to the poll table */
		poll_wait(file, &sim_module_wait, wait);
	}

	if (sim_module_rx_event != SIM_MODULE_EVENT_NONE) {
		retval = POLLIN;
	}

	return retval;
}

/*

DESCRIPTION:
    This function is the interrupt handler SIM receive data.

INPUTS:
    UINT8 reader_id: the card reader ID of the interrupt to handle

OUTPUT:
    None

IMPORTANT NOTES:
    None

*/
static void sim_module_int_rx(UINT8 reader_id)
{
	UINT8 new_byte = 0;
	UINT8 i = 0;
	UINT16 loop = 0;
	BOOL next_is_data_byte = FALSE;

	write_reg(&(sim_registers[reader_id]->irqstatus), USIM_RX_MASK);

	if((preforming_status_command == FALSE) && (status_slow_card == FALSE)){
		/* notify user space that data was received */
		sim_module_rx_event |= SIM_MODULE_EVENT_RX_A;
	}
	else{
		bytes_recieved = ((UINT16)(omap_get_dma_dst_pos(sim_module_dma_channel) -
					sim_module_card_data[0].dma_buffer));

		if(proc_bytes_done == FALSE){
			for(i = last_byte_checked; i < bytes_recieved; i++) {
				new_byte = sim_module_card_data[0].buffer[i];
				if(SIM_STATUS_ACK_ALL(new_byte)) {
					ack_counter++;
					proc_bytes_done = TRUE;
					break;
				}
				else if(new_byte == SIM_NULL_PROC_BYTE) {
					null_byte_counter++;
				}
				else if(SIM_STATUS_ACK_ONE(new_byte)) {
					ack_counter++;
					/* skip over the next (data) byte */
					i++;
					/* if we ended on an ACK, the next byte should be data (so skip it) */
					if(i >= bytes_recieved){
						next_is_data_byte = TRUE;
					}
				}
				else {
					proc_bytes_done = TRUE;
					break;
				}
			}

			last_byte_checked = bytes_recieved;
			/* preemptively skip the next (data) byte if we ended on an ACK (in the ACK ONE case) */
			if(next_is_data_byte == TRUE){
				last_byte_checked++;
			}
		}

		if(bytes_recieved > (expected_status_data(card_type) +
			null_byte_counter + ack_counter - 2 )){

			if((sim_module_card_data[0].buffer[expected_status_data(card_type)+
				null_byte_counter + ack_counter - 2]) == SIM_NULL_PROC_BYTE){

				loop = bytes_recieved -
						(expected_status_data(card_type) +
						null_byte_counter + ack_counter - 2);

				while((loop > 0) && (sim_module_card_data[0].
					buffer[expected_status_data(card_type) +
					null_byte_counter + ack_counter-2] == SIM_NULL_PROC_BYTE)){
					loop--;
					null_byte_counter++;
				}
			}
		}
		if (bytes_recieved - (expected_status_data(card_type) +
			null_byte_counter + ack_counter) == 0){
			write_reg(&(sim_registers[reader_id]->irqstatus),
				USIM_RX_MASK);

			/* Disable WWT */
			write_reg_bits (&(sim_registers[0]->irqenable), USIM_WT_EN_MASK, 0);
			all_data_received = TRUE;
			status_slow_card = FALSE;
			if(preforming_status_command == FALSE) {
				sim_module_rx_event |= SIM_MODULE_EVENT_RX_A;
			}
		}
	}
	return;
}

/*

DESCRIPTION:
    This function handles the SIM TX interrupt.  In cases where the TX data exceeds the TX FIFO
    length, it refills the TX FIFO on TX threshold interrupts.  On TX complete, it switches the
    system into receive mode.

INPUTS:
    UINT8 reader_id: the card reader ID of the interrupt to handle

OUTPUT:
    None

IMPORTANT NOTES:
    None

*/
static void sim_module_int_tx(UINT8 reader_id)
{
    UINT16 bytes_remaining = 0;
    UINT8 tx_index = 0;
    BOOL last_block = FALSE;

    /* disable the transmit complete interrupt */
    write_reg_bits(&(sim_registers[reader_id]->irqenable), USIM_TX_EN_MASK, 0);

    /* disable the transmitter */
    write_reg_bits(&(sim_registers[reader_id]->usimconf2), TXNRX_MASK, 0);

    if(preforming_status_command == FALSE){

	/* determine remaining number of bytes to transmit */
	bytes_remaining = sim_module_card_data[reader_id].tx_length -
	    sim_module_card_data[reader_id].buffer_index;

	/* if the number of bytes remaining exceeds the FIFO size ... */
	if (bytes_remaining > SIM_MODULE_TX_FIFO_SIZE) {
		/* only transmit the free TX FIFO size */
		bytes_remaining = SIM_MODULE_TX_FIFO_SIZE;
	} else {
		last_block = TRUE;
	}

	/* for each byte of data to transmit ... */
	for (tx_index = 0; tx_index < bytes_remaining; tx_index++) {
		/* write the character to the TX FIFO */
		write_reg(&(sim_registers[reader_id]->usim_dtx),
			  sim_module_card_data[reader_id].
			  buffer[sim_module_card_data[reader_id].
				 buffer_index++]);

	}

	/* setup the TX threshold value to write more data into the FIFO */
	write_reg_bits(&(sim_registers[reader_id]->usim_fifos),
		       FIFO_TX_TRIGGER_MASK,
		       (SIM_MODULE_TX_FIFO_SIZE - 1) << 2);

	/* clear the transmit complete interrupt */
	write_reg_bits(&(sim_registers[reader_id]->irqstatus),
		       USIM_TX_MASK, USIM_TX_MASK);

	/* enable the transmit complete interrupt */
	write_reg_bits(&(sim_registers[reader_id]->irqenable),
		       USIM_TX_EN_MASK, USIM_TX_EN_MASK);

	/* enable the transmitter */
	write_reg_bits(&(sim_registers[reader_id]->usimconf2), TXNRX_MASK,
		       TXNRX_MASK);

	/* if there is just one block of data to send ... */
	if (last_block == TRUE) {
		/* reset the data buffer index */
		sim_module_card_data[reader_id].buffer_index = 0;

		/* enable the receiver right away */
		write_reg_bits(&(sim_registers[reader_id]->usimconf2),
			       TXNRX_MASK, 0);

		/* indicate this is the last block */
		sim_module_all_tx_data_sent = TRUE;

		/* initialize rx mode */
		sim_module_init_rx_mode(reader_id);

	}
        } else {
		/* enable the receiver right away */
		write_reg_bits(&(sim_registers[0]->usimconf2), TXNRX_MASK, 0);

		/* indicate this is the last block */
		sim_module_all_tx_data_sent = TRUE;

		/* initialize rx mode */
		sim_module_init_rx_mode(0);
	}
    return;
}

/*

DESCRIPTION:
    This function is the interrupt handler for the SIM Reset Detect Mode.  This function only reads
    data and copies it into the receive buffer without checking for parity errors.  The purpose is
    to advance the buffer_index variable for the detection of an incoming ATR message which
    indicates that the SIM card has reset.  An ATR message is the only thing which should be
    received (and then only on SIM card resets) when this interrupt handler is active.

INPUTS:
    UINT8 reader_id: the card reader ID of the interrupt to handle

OUTPUT:
    None

IMPORTANT NOTES:
    None

*/
static void sim_module_int_reset_detect(UINT8 reader_id)
{
	/* mask the RX FIFO full interrupt */
	write_reg_bits(&(sim_registers[reader_id]->irqenable),
		       USIM_RX_EN_MASK, 0);

	/* while there is data in the FIFO ... */
	while (((sim_registers[reader_id]->
		 usim_fifos) & FIFORX_EMPTY_MASK) == 0) {
		/* copy the character into the data buffer */
		sim_module_card_data[reader_id].
		    buffer[sim_module_card_data[reader_id].buffer_index++]
		    = (read_reg(&(sim_registers[reader_id]->usim_drx)) &
		       USIMDRX_MASK);
	}

	/* set an event for user space */
	sim_module_rx_event |= SIM_MODULE_EVENT_BUFFER_INDEX;

	/* clear the interrupt */
	write_reg(&(sim_registers[reader_id]->irqstatus), USIM_RX_MASK);

	return;
}


/*

DESCRIPTION:
    This function is the interrupt service routine that handles non-data interrupts of smart card
    interface module 1.

INPUTS:
    int irq       : the interrupt request number
    void * dev_id : pointer to the device associated with the interrupt

OUTPUT:
    irqreturn_t status : a status indicating if an interrupt was handled successfully

IMPORTANT NOTES:
    None

*/
static irqreturn_t sim_module_int_irq_1(int irq, void *dev_id)
{
	if (sim_low_power_enabled == TRUE) {
		/* enable the SIM FCLK */
		clk_enable(usim_fck);
	}

	/* if the byte resent interrupt fired ... */
	if (((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	      USIM_RESENT_MASK) == USIM_RESENT_MASK)
	    &&
	    ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	      USIM_RESENT_EN_MASK) == USIM_RESENT_EN_MASK)) {
		sim_module_nack_counter++;

		/* if the number of nacks on a character exceed the threshold ... */
		if (sim_module_nack_counter > SIM_MODULE_NACK_THRESHOLD) {
			/* we must have a bad ME->SIM connection.
			   user space must be notified of this fatal error */
			sim_module_rx_event |= SIM_MODULE_EVENT_ERROR_FLAG;
			sim_module_nack_counter = 0;
		}

		write_reg(&(sim_registers[SIM_MODULE_1]->irqstatus),
			  USIM_RESENT_MASK);
	}

	/* if the position detect interrupt fired ... */
	if (((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	      USIM_CD_MASK) == USIM_CD_MASK)
	    &&
	    ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	      USIM_CD_EN_MASK) == USIM_CD_EN_MASK)) {
		/* if card status is changing from removed -> present ... */
		if (((read_reg(&(sim_registers[SIM_MODULE_1]->usimstat)) &
		      STATNOCARD_MASK) == STATNOCARD_MASK)
		    && (sim_module_current_pd_state_sim1 == SIM_REMOVED)) {
			sim_module_rx_event |=
			    SIM_MODULE_EVENT_SIMPD_INSERTION;
			sim_module_current_pd_state_sim1 = SIM_PRESENT;
		}

		/* else if card status is changing from present -> removed ... */

		else if (((read_reg
			   (&(sim_registers[SIM_MODULE_1]->usimstat)) &
			   STATNOCARD_MASK) == 0)
			 && (sim_module_current_pd_state_sim1 ==
			     SIM_PRESENT)) {
			sim_module_rx_event |=
			    SIM_MODULE_EVENT_SIMPD_REMOVAL;
			sim_module_current_pd_state_sim1 = SIM_REMOVED;
		}

		/* else the CD interrupt fired, but the status didn't change, so ignore */

		write_reg(&(sim_registers[SIM_MODULE_1]->irqstatus),
			  USIM_CD_MASK);
	}

	/* if the character timer interrupt fired ... */
	if (((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	      USIM_WT_MASK) == USIM_WT_MASK)
	    &&
	    ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	      USIM_WT_EN_MASK) == USIM_WT_EN_MASK)) {
		/* notify user space */
		sim_module_rx_event |= SIM_MODULE_EVENT_WWT_VIOLATION;
		write_reg_bits(&(sim_registers[SIM_MODULE_1]->irqenable),
			       USIM_WT_EN_MASK, 0);
	}

	/* if the no ATR interrupt fired ... */
	if (((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	      USIM_NATR_MASK) == USIM_NATR_MASK)
	    &&
	    ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	      USIM_NATR_EN_MASK) == USIM_NATR_EN_MASK)) {
		write_reg_bits(&(sim_registers[SIM_MODULE_1]->irqenable),
			       USIM_NATR_EN_MASK, 0);
		/* notify user space */
		sim_module_rx_event |= SIM_MODULE_EVENT_NO_ATR_FLAG;
	}

	/* if one of the data interrupts fired ... */
	if ((((sim_module_interrupt_mode == SIM_MODULE_RX_MODE)
	      || (sim_module_interrupt_mode ==
		  SIM_MODULE_RESET_DETECT_MODE))
	     &&
	     ((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	       USIM_RX_MASK) == USIM_RX_MASK)
	     &&
	     ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	       USIM_RX_EN_MASK) == USIM_RX_EN_MASK))
	    || ((sim_module_interrupt_mode == SIM_MODULE_TX_MODE)
		&&
		((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
		  USIM_TX_MASK) == USIM_TX_MASK)
		&&
		((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
		  USIM_TX_EN_MASK) == USIM_TX_EN_MASK))) {
		/* execute the mode specific data interrupt handler */
		if (sim_module_interrupt_mode == SIM_MODULE_RX_MODE) {
			sim_module_int_rx(SIM_MODULE_1);
		} else if (sim_module_interrupt_mode == SIM_MODULE_TX_MODE) {
			sim_module_int_tx(SIM_MODULE_1);
		} else if (sim_module_interrupt_mode ==
			   SIM_MODULE_RESET_DETECT_MODE)
		{
			sim_module_int_reset_detect(SIM_MODULE_1);
                } else if (sim_module_interrupt_mode == SIM_MODULE_MODE_NONE) {
			sim_module_rx_event |= SIM_MODULE_EVENT_SPURIOUS_DATA;
		}
	}

	/* if there was a FIFO overflow ... */
	if (((read_reg(&(sim_registers[SIM_MODULE_1]->irqstatus)) &
	      USIM_RXFULL_MASK) == USIM_RXFULL_MASK)
	    &&
	    ((read_reg(&(sim_registers[SIM_MODULE_1]->irqenable)) &
	      USIM_RXFULL_EN_MASK) == USIM_RXFULL_EN_MASK)) {
		/* there is no reason to read the data in the FIFO, as we've already missed data
		   at this point. this is a fatal error. the OMAP3430 does not have the ability
		   to automatically NACK characters when the FIFO is full, therefore characters will
		   be lost. this can never be allowed to happen because of the lack of flow control
		   on this IC. */

		/* clear and disable the interrupt */
		write_reg(&(sim_registers[SIM_MODULE_1]->irqstatus),
			  USIM_RXFULL_MASK);
		write_reg_bits(&(sim_registers[SIM_MODULE_1]->irqenable),
			       USIM_RXFULL_EN_MASK, 0);

		/* notify user space as this could be a fatal error */
		sim_module_rx_event |= SIM_MODULE_EVENT_FIFO_OVERFLOW;
	}

	/* if this interrupt caused a user space event ... */
	if (sim_module_rx_event != SIM_MODULE_EVENT_NONE) {
		/* wake up the user space event thread */
		wake_up_interruptible(&sim_module_wait);
	}

	if (sim_low_power_enabled == TRUE) {
		/* disable the SIM FCLK */
		clk_disable(usim_fck);
	}

	return (IRQ_RETVAL(1));
}

/*

DESCRIPTION:
    This routine sets up the hardware and interrupt system to receive data.

INPUTS:
    UINT8 card_reader_id: card reader ID for which to setup

OUTPUT:
    None

IMPORTANT NOTES:
    None
*/
static void sim_module_init_rx_mode(UINT8 reader_id)
{
	/* if the reader ID is valid ... */
	if (reader_id < NUM_SIM_MODULES) {
		/* disable TX mode */
		write_reg_bits(&(sim_registers[reader_id]->irqenable),
			       USIM_TX_EN_MASK, 0);

		/* clear the RX fifo threshold */
		write_reg_bits(&(sim_registers[reader_id]->usim_fifos),
			       FIFO_RX_TRIGGER_MASK, 0);

		/* enable DMA mode */
		write_reg_bits(&(sim_registers[reader_id]->usim_fifos),
			       SIM_DMA_MODE_MASK, SIM_DMA_MODE_MASK);

		/* indicate the card is in the non-ATR receive mode */
		sim_module_interrupt_mode = SIM_MODULE_RX_MODE;

		/* if there is no data in the FIFO ... */
		if ((read_reg(&(sim_registers[reader_id]->usim_fifos)) &
		     FIFORX_EMPTY_MASK) == FIFORX_EMPTY_MASK) {
			/* clear the RX fifo interrupt */
			write_reg(&(sim_registers[reader_id]->irqstatus),
				  (USIM_RX_MASK | USIM_RXFULL_MASK));
		}

		/* enable the RX full interrupt */
		write_reg_bits(&(sim_registers[reader_id]->irqenable),
			       (USIM_RX_MASK | USIM_RXFULL_EN_MASK),
			       (USIM_RX_MASK | USIM_RXFULL_EN_MASK));

	}

	return;
}

/*

DESCRIPTION:
    This routine configures the voltage level

INPUTS:
    SIM_MODULE_VOLTAGE_LEVEL level : the voltage level to use

OUTPUT:
    None

IMPORTANT NOTES:
    None
*/
static void sim_module_set_voltage_level(SIM_MODULE_VOLTAGE_LEVEL level)
{
	/* Clear the USIM_IO_PWRDNZ bit to protect
	   the USIM IO cell while changing voltage */
	write_reg_bits((volatile UINT32 *) CONTROL_WKUP_CTRL,
			CONTROL_WKUP_CTRL_MASK, 0);

	/* power down the voltage regulator */
	if (regulator_enabled_flag) {
		regulator_disable(vsim_regulator);
		regulator_disable(vsimcard_regulator);
		regulator_enabled_flag = 0;
	}

	/* power down the pads */
	write_reg_bits((volatile UINT32 *) PBIAS_CONTROL_LITE,
		       PBIASPWRDNZ1, 0);

	/* enable fast I/O */
	write_reg_bits((volatile UINT32 *) PBIAS_CONTROL_LITE,
		       PBIASSPEEDCNTL1, PBIASSPEEDCNTL1);

	/* 3.0V is desired */
	if (level == SIM_MODULE_VOLTAGE_HIGH) {
		/* power on the voltagage regulator at 3 volts */
		regulator_set_voltage(vsim_regulator, 2900000, 2900000);
		regulator_set_voltage(vsimcard_regulator, 2900000,
				      2900000);

		if (!(regulator_enabled_flag)) {
			regulator_enable(vsim_regulator);
			regulator_enable(vsimcard_regulator);
			regulator_enabled_flag = 1;
		}

		/* configure the pad for 3.0V operation */
		write_reg_bits((volatile UINT32 *) PBIAS_CONTROL_LITE,
			       PBIASVMODE1, PBIASVMODE1);
	}

	/* 1.8V is desired */
	else if (level == SIM_MODULE_VOLTAGE_LOW) {
		regulator_set_voltage(vsim_regulator, 1800000, 1800000);
		regulator_set_voltage(vsimcard_regulator, 1800000,
				      1800000);
		if (!(regulator_enabled_flag)) {
			regulator_enable(vsim_regulator);
			regulator_enable(vsimcard_regulator);
			regulator_enabled_flag = 1;
		}

		/* configure the pad for 1.8V operation */
		write_reg_bits((volatile UINT32 *) PBIAS_CONTROL_LITE,
			       PBIASVMODE1, 0);
	}

	/* else, leave the voltage supply */
	if ((level == SIM_MODULE_VOLTAGE_HIGH)
	    || (level == SIM_MODULE_VOLTAGE_LOW)) {
		/* the pad bias level doesn't equal the VSIM level ... */
		while ((read_reg((volatile UINT32 *) PBIAS_CONTROL_LITE) &
			PBIASVMODEERROR1) != 0) {
			/* block the current thread, but yeild the processor
			   every iteration of the loop */
			msleep(1);
		}

		/* enable the pad */
		write_reg_bits((volatile UINT32 *) PBIAS_CONTROL_LITE,
			       PBIASPWRDNZ1, PBIASPWRDNZ1);

		/* let voltages stabalize */
		msleep(1);
	}
	/* Set the USIM_IO_PWRDNZ bit after voltage is stable */
	write_reg_bits((volatile UINT32 *) CONTROL_WKUP_CTRL,
			CONTROL_WKUP_CTRL_MASK, CONTROL_WKUP_CTRL_MASK);

	return;

}

/*

DESCRIPTION:
    This routine sets the clock rate

INPUTS:
    UINT8 reader_id		: the reader ID for which to set the clock rate
    SIM_MODULE_CLOCK_RATE level : the clock rate to set

OUTPUT:
    None

IMPORTANT NOTES:
    None
*/
static void sim_module_set_clock_rate(UINT8 reader_id,
				      SIM_MODULE_CLOCK_RATE rate)
{
	/* if the reader ID is valid ... */
	if (reader_id < NUM_SIM_MODULES) {
		switch (rate) {
		case SIM_MODULE_CLOCK_0:

			/* disable the SIM FCLK */
			clk_disable(usim_fck);
			break;
		case SIM_MODULE_CLOCK_4:
			/* set the internal SIM clock divider to divide by 2 */
			write_reg_bits(&
				       (sim_registers[reader_id]->
					usimconf2), CONFSCLKDIV_MASK, 0);
			/* disable the SIM FCLK */
			clk_disable(usim_fck);
			/* set the source of the SIM FCLK to the 120 MHz clock divided
			   by 16 to obtain a 3.75 MHz SIM clock */
			clk_set_parent(usim_fck, omap_120m_fck);
			clk_set_rate(usim_fck, 7500000);

			/* enable the SIM FCLK */
			clk_enable(usim_fck);
			sim_module_clock_frequency =
			    SIM_MODULE_FREQUENCY_4;
			break;
		case SIM_MODULE_CLOCK_5:
			/* set the internal SIM clock divider to divide by 2 */
			write_reg_bits(&
				       (sim_registers[reader_id]->
					usimconf2), CONFSCLKDIV_MASK, 0);
			/* disable the SIM FCLK */
			clk_disable(usim_fck);
			/* set the source of the SIM FCLK to the 96 MHz clock divided
			   by 10 to obtain a 4.8 MHz SIM clock */
			clk_set_parent(usim_fck, omap_96m_fck);
			clk_set_rate(usim_fck, 9600000);

			/* enable the SIM FCLK */
			clk_enable(usim_fck);
			sim_module_clock_frequency =
			    SIM_MODULE_FREQUENCY_5;
			break;
		default:
			tracemsg
			    ("Warning: Invalid SIM clock selection.\n");
			break;
		}
	}

	if ((rate == SIM_MODULE_CLOCK_4) || (rate == SIM_MODULE_CLOCK_5)) {
		/* wait for the clock to stabalize */
		msleep(1);
	}

	return;
}


/*
DESCRIPTION:
    This routine runs when the GPT1 expires and the system is in full suspend. The routine will
issue a STATUS command to the SIM card, and verify the response. If the returned status words 
indicate something that can't be fully handled in user space, the handler will return and the phone
will be resumed

INPUTS:
    None

OUTPUT:
    int status: -1 if user space needs to be woken to handle an event, 0 for success

IMPORTANT NOTES:
    None
*/
int sim_slim_status_handler()
{
	int status = -1;
	int error = 0;
	UINT8 length = 5;
	UINT8 tx_index = 0;
	unsigned long flags = 0;
	UINT8 sleep_counter = 0;
	UINT8 sw1 = 0;
	UINT8 sw2 = 0;

	if((currently_in_call == FALSE) && (sim_mutex_update(KERNEL_LOCK_MUTEX)) && 
                (card_type != UNKNOWN_CARD))
	{
		preforming_status_command = TRUE;
		status_slow_card = TRUE;
		all_data_received = FALSE;
		bytes_recieved = 0;
		last_byte_checked = 0;
		null_byte_counter = 0;
		ack_counter = 0;
		proc_bytes_done = FALSE;

		/* steps normally done from user space: */
		/* disable low power */
		sim_low_power_enabled = FALSE;
		clk_enable(usim_fck);
		write_reg_bits((volatile UINT32 *)DMA_SYSCONFIG,DMA_MIDLE,DMA_SYSCONFIG_MIDLEMODE(1));

		/* enable the module clock */
		write_reg_bits (&(sim_registers[0]->usimcmd), MODULE_CLK_EN_MASK, MODULE_CLK_EN_MASK);

		/* reset card reader data */
		sim_module_card_data[0].buffer_index = 0;
		sim_module_card_data[0].rx_last_index = 0;
		sim_module_card_data[0].tx_length = 5;
		sim_module_card_data[0].error_flag = ISR_NO_ERROR;


		/* clear interrupt sources */
		write_reg_bits (&(sim_registers[0]->irqenable), USIM_IRQEN_MASK_ALL, 0);

		/* set the interrupt mode */
		sim_module_interrupt_mode = SIM_MODULE_TX_MODE;    

		/* rx_fifo threashold to 1 */
		write_reg_bits (&(sim_registers[0]->usim_fifos), FIFO_RX_TRIGGER_MASK, 0);

		/* disable the transmitter */
		write_reg_bits (&(sim_registers[0]->usimconf2), TXNRX_MASK, 0);

		/* enable the module clock */
		write_reg_bits (&(sim_registers[0]->usimcmd), CMD_CLOCK_STOP_MASK, 0);  

		if(card_type == UICC){
			memcpy(sim_module_card_data[0].buffer, uicc_status, length);
		}
		else{
			memcpy(sim_module_card_data[0].buffer, gsm_status, length);
		}
    
		/* Enable DMA mode */
		write_reg_bits(&(sim_registers[0]->usim_fifos), SIM_DMA_MODE_MASK, SIM_DMA_MODE_MASK);

		if (sim_module_dma_channel != 0) {
			omap_free_dma(sim_module_dma_channel);
			sim_module_dma_channel = 0;
		}

		/* request a DMA logical channel */
		error = omap_request_dma(OMAP34XX_DMA_USIM_RX, SIM_DEV_NAME, sim_module_dma_callback,
			(void *)sim_module_card_data[0].dma_buffer, &sim_module_dma_channel);
		/* if we failed to get our logical channel */
		if (error != 0) {
			tracemsg("sim_slim_status_hdlr: Fatal error - request_dma API failed\n");
			sim_module_rx_event |= SIM_MODULE_EVENT_NO_DMA_CN_AVB;
			return 1;
		}
		/* configure the DMA parameters */
		omap_set_dma_transfer_params(sim_module_dma_channel, OMAP_DMA_DATA_TYPE_S8, 1,
			SIM_MODULE_MAX_DATA, OMAP_DMA_SYNC_ELEMENT, OMAP34XX_DMA_USIM_RX,
			OMAP_DMA_SRC_SYNC);
		omap_set_dma_src_params(sim_module_dma_channel, 0, OMAP_DMA_AMODE_CONSTANT,
			(unsigned long) (&(sim_phys_registers[0]->usim_drx)), 0, 0);
		omap_set_dma_dest_params(sim_module_dma_channel, 0, OMAP_DMA_AMODE_POST_INC,
			(unsigned long) sim_module_card_data[0].dma_buffer, 0, 0);

		omap_start_dma(sim_module_dma_channel);

		local_irq_save(flags);
	
		/* for each character to transmit ... */
		for (tx_index = 0; tx_index < length; tx_index++) {
			/* copy the byte to the TX FIFO */
			write_reg(&(sim_registers[0]->usim_dtx), sim_module_card_data[0].
				buffer[sim_module_card_data[0].buffer_index++]);
		}

		/* setup the TX threshold value to write more data into the FIFO */
		write_reg_bits(&(sim_registers[0]->usim_fifos), FIFO_TX_TRIGGER_MASK,
			(SIM_MODULE_TX_FIFO_SIZE - 1) << 2);

		/* clear the transmit complete interrupt */
		write_reg_bits(&(sim_registers[0]->irqstatus), USIM_TX_MASK, USIM_TX_MASK);

		/* enable the transmit complete interrupt */
		write_reg_bits(&(sim_registers[0]->irqenable), USIM_TX_EN_MASK, USIM_TX_EN_MASK);

		/* enable WWT */
		write_reg_bits (&(sim_registers[0]->irqstatus), USIM_WT_MASK, USIM_WT_MASK);
		write_reg_bits (&(sim_registers[0]->irqenable), USIM_WT_EN_MASK, USIM_WT_EN_MASK);

		/* enable the transmitter */
		write_reg_bits(&(sim_registers[0]->usimconf2), TXNRX_MASK, TXNRX_MASK);

		sim_module_card_data[0].buffer_index = 0;


		local_irq_restore(flags);
     
		while ((!all_data_received) && (sleep_counter <= 19)) {
			sleep_counter++;
			msleep(5);
		}
		if (all_data_received ==TRUE) {
			sim_module_interrupt_mode =  SIM_MODULE_MODE_NONE;
		}

		preforming_status_command = FALSE;

		if(all_data_received == TRUE){

			status_slow_card = FALSE;
			all_data_received = FALSE;

			sw1 = sim_module_card_data[0].buffer[bytes_recieved - 2];
			sw2 = sim_module_card_data[0].buffer[bytes_recieved - 1];

			if(((sw1 == 0x90) || ((card_type == GSM_SIM) &&
				(sw1==0x67))) && (sw2 == 0x00)){
				sim_low_power_enabled = TRUE;

				/* Allow DMA to ACK idle requests */
				write_reg_bits((volatile UINT32 *)DMA_SYSCONFIG,DMA_MIDLE,DMA_SYSCONFIG_MIDLEMODE(2));

				/* Disable DMA mode for low power mode */
				write_reg_bits(&(sim_registers[0]->usim_fifos), SIM_DMA_MODE_MASK, 0);

				/*Start the clock stop procedure &
				 wait maximum for 20 milli sec*/
				start_clk_stop_procedure(SIM_MODULE_1, 20000);

				/* disable the module clock */
				write_reg_bits(&(sim_registers[0]->usimcmd),
					MODULE_CLK_EN_MASK, 0);

				/* disable the SIM FCLK */
				clk_disable(usim_fck);
				if(sim_mutex_update(KERNEL_UNLOCK_MUTEX)) {
					status = 0;
				}
				else {
					sim_module_rx_event |= SIM_MODULE_EVENT_MUTEX_FREE;
					status = 1;
				}
			}
			else {
				sim_module_rx_event |= SIM_MODULE_EVENT_RX_A;
				sim_module_rx_event |= SIM_MODULE_EVENT_INCOMPLETE_SLIM_STATUS;
				status = 1;
			}
		}
		else if(null_byte_counter >= 500){
			sim_module_rx_event |= SIM_MODULE_EVENT_RX_A;
			sim_module_rx_event |= SIM_MODULE_EVENT_INCOMPLETE_SLIM_STATUS;
			sim_module_rx_event |= SIM_MODULE_EVENT_NULL_BYTE_OVERFLOW;
			status = 1;
		}
		/* send incomplete slim status with RX_A event */
		else {
			sim_module_rx_event |= SIM_MODULE_EVENT_RX_A |
				SIM_MODULE_EVENT_INCOMPLETE_SLIM_STATUS;
			status = 1;
		}
	}

	/* if this interrupt caused a user space event ... */
	if (sim_module_rx_event != SIM_MODULE_EVENT_NONE) {
		/* wake up the user space event thread */
		wake_up_interruptible(&sim_module_wait);
	}

	return(status);
}


/*

DESCRIPTION:
    This function manipulates the mutex protecting the SIM HW

INPUTS:
    UINT8 reader_id: the card reader ID of the HW to protect
    UINT8 mutex_request: the requester ID and action for the mutex   

OUTPUT:
    BOOL success: the status of the request

IMPORTANT NOTES:
    None

*/
static BOOL sim_mutex_update(UINT8 mutex_request)
{
	UINT8 mutex_requester = 0;
	UINT8 mutex_action = 0;
	BOOL success = FALSE;

	/* Extract the requester's ID and the action from the request */
	mutex_requester = mutex_request & MUTEX_REQUESTER_MASK;
	mutex_action = mutex_request & MUTEX_UNLOCK_MASK;

	spin_lock(&sim_status_lock);

	/* If the request is to unlock the mutex */
	if(mutex_action == MUTEX_UNLOCK_MASK) {
		/* If the HW is currently not locked */
		if(sim_mutex_locked_by_id == 0) {
			success = TRUE;
		}
		/* If the holder of the mutex is requesting to unlock */
		else if(mutex_requester == sim_mutex_locked_by_id) {
			/* If there are no pending requests, simply unlock */
			if(sim_mutex_requested_by_id == 0){
				sim_mutex_locked_by_id = 0;
				success = TRUE;
			}
			/* The mutex was requested while we were using it, transfer control to the requester */
			else {
				sim_mutex_locked_by_id = sim_mutex_requested_by_id;
				sim_mutex_requested_by_id = 0;
			}
		}
		/* If we get any requests to unlock from a non-lock holder, we'll ignore them */
	}
	/* Else we're trying to lock the SIM HW */
	else {
		/* If it's not locked, then go ahead and let the requester have it */
		if(sim_mutex_locked_by_id == 0){
			sim_mutex_locked_by_id = mutex_requester;
			success = TRUE;
		}
		/* If it's locked by the kernel, and SCPC wants it, transfer control to SCPC */
		else if((sim_mutex_locked_by_id == KERNEL_MUTEX_ID) &&
			(mutex_requester == SCPC_MUTEX_ID)) {
			sim_mutex_locked_by_id = SCPC_MUTEX_ID;
			success = TRUE;
		}
		else if (sim_mutex_locked_by_id == mutex_requester) {
			success = TRUE;
		}
		/* If it's locked, and requested by anyone but the KERNEL, then queue the request */
		else{
			if(mutex_requester != KERNEL_MUTEX_ID) {
				sim_mutex_requested_by_id = mutex_requester;
			}
		}
	}
	spin_unlock(&sim_status_lock);

	return success;
}


/*

DESCRIPTION:
    This routine is the callback function for the SIM DMA request

INPUTS:
    INT32 lch : the logical channel which caused the callback to execute
    UINT16 ch_status : the status of the logical channel
    void * data : a pointer to the DMA data.

OUTPUT:
    None

IMPORTANT NOTES:
    Given the way we're using DMA due to H/W design and T=0 protocol limitations
    , this shouldn't ever be called unless there is a problem. Either there was an error,
    or we ran out of room in the RAM buffer because of too many errors.
*/
void sim_module_dma_callback(INT32 lch, UINT16 ch_status, void *data)
{

	if (ch_status != 0) {
		tracemsg
		    ("SIM MODULE DMA CALLBACK ERROR lch -> %X, ch_status -> %X\nregister contents: ",
		     lch, ch_status);

		tracemsg(" **************************"
			 " DMA Channel (%d) Registers"
			 "***************************\n",
			 sim_module_dma_channel);
#if(0)
/* TODO: These don't exist on the new kernel, submit a CR to check out where they went */
		tracemsg("OMAP_DMA_CCR_REG(%d)	   : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CCR_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CLNK_CTRL_REG(%d)     : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CLNK_CTRL_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CICR_REG(%d)	  : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CICR_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CSR_REG(%d)	   : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CSR_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CSDP_REG(%d)	  : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CSDP_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CEN_REG(%d)	   : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CEN_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CFN_REG(%d)	   : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CFN_REG(sim_module_dma_channel));
		tracemsg("OMAP2_DMA_CSSA_REG(%d)	 : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP2_DMA_CSSA_REG(sim_module_dma_channel));
		tracemsg("OMAP2_DMA_CDSA_REG(%d)	 : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP2_DMA_CDSA_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CSEI_REG(%d)	  : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CSEI_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CSFI_REG(%d)	  : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CSFI_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CDEI_REG(%d)	  : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CDEI_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CDFI_REG(%d)	  : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CDFI_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CSAC_REG(%d)	  : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CSAC_REG(sim_module_dma_channel));
		tracemsg("OMAP_DMA_CDAC_REG(%d)	  : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP_DMA_CDAC_REG(sim_module_dma_channel));
		tracemsg("OMAP2_DMA_CCEN_REG(%d)	 : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP2_DMA_CCEN_REG(sim_module_dma_channel));
		tracemsg("OMAP2_DMA_CCFN_REG(%d)	 : 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP2_DMA_CCFN_REG(sim_module_dma_channel));
		tracemsg("OMAP2_DMA_COLOR_REG(%d)	: 0x%08x\n",
			 sim_module_dma_channel,
			 OMAP2_DMA_COLOR_REG(sim_module_dma_channel));
#endif

	}

	sim_module_rx_event |= SIM_MODULE_EVENT_ERROR_FLAG;

	/* wake up the user space event thread */
	wake_up_interruptible(&sim_module_wait);

	return;
}

/* DESCRIPTION:
       The SIM timer callback function. This is called by the kernel wakeup
       timer, when the sim timer expire, and the system is NOT suspended.

   INPUTS:
       None.

   OUTPUTS:
       int ret - the return value is currently ignored

   IMPORTANT NOTES:
       None.
*/
static int sim_timer_callback(void)
{
    int ret = 0;

    sim_module_rx_event |= SIM_MODULE_EVENT_TIMER_EXP;

    /* wake up the user space event thread */
    wake_up_interruptible(&sim_module_wait);

    return ret;
}

/* DESCRIPTION:
       The probe routine.
 
   INPUTS:
       struct platform_device *pdev : the platform device pointer.  

   OUTPUTS:
       Returns probe sucess/error indication.

   IMPORTANT NOTES:
       None.
*/
static int sim_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *temp_class;

	sim_module_major = register_chrdev(0, SIM_DEV_NAME, &sim_fops);
	if (sim_module_major < 0) {
		tracemsg
		    ("sim_probe: Unable to get a major for SIM driver\n");
		return sim_module_major;
	}

	sim_class = class_create(THIS_MODULE, SIM_DEV_NAME);
	if (IS_ERR(sim_class)) {
		unregister_chrdev(sim_module_major, SIM_DEV_NAME);
		tracemsg("sim_probe: Error creating SIM class.\n");
		ret = PTR_ERR(sim_class);
		return ret;
	}

	temp_class =
	    device_create(sim_class, NULL, MKDEV(sim_module_major, 0),
			  NULL, SIM_DEV_NAME);

	if (IS_ERR(temp_class)) {
		class_destroy(sim_class);
		unregister_chrdev(sim_module_major, SIM_DEV_NAME);
		tracemsg("sim_probe: Error creating SIM class device.\n");
		ret = PTR_ERR(temp_class);
		return ret;
	}

	vsim_regulator = regulator_get(NULL, "vsim");
	if (IS_ERR(vsim_regulator)) {
		printk(KERN_ERR"sim_probe: Could not get VSIM regulator\n");
		return PTR_ERR(vsim_regulator);
	}
	vsimcard_regulator = regulator_get(NULL, "vsimcard");
	if (IS_ERR(vsimcard_regulator)) {
		printk(KERN_ERR"sim_probe: Could not get VSIMCARD regulator\n");
		return PTR_ERR(vsimcard_regulator);
	}

	sim_device = &(pdev->dev);
	usim_fck = clk_get(sim_device, USIM_FCK);
	if (IS_ERR(usim_fck)) {
		tracemsg("sim_probe: Error getting USIM FCLK.\n");
		ret = PTR_ERR(usim_fck);
	}

	usim_ick = clk_get(sim_device, USIM_ICK);
	if (IS_ERR(usim_ick)) {
		tracemsg("sim_probe: Error getting USIM ICLK.\n");
		ret = PTR_ERR(usim_ick);
	}

	omap_96m_fck = clk_get(sim_device, OMAP_96M_FCK);
	if (IS_ERR(omap_96m_fck)) {
		tracemsg("sim_probe: Error getting 96M FCLK.\n");
		ret = PTR_ERR(omap_96m_fck);
	}

	omap_120m_fck = clk_get(sim_device, OMAP_120M_FCK);
	if (IS_ERR(omap_120m_fck)) {
		tracemsg("sim_probe: Error getting 120M FCLK.\n");
		ret = PTR_ERR(omap_120m_fck);
	}
#ifdef  CONFIG_QUICK_WAKEUP
	quickwakeup_register(&sim_qw_ops);
	sim_timer = wakeup_create_status_timer(sim_timer_callback);
	if (sim_timer == NULL) {
		tracemsg("sim_probe: sim_timer creation failed.\n");
		ret = -ENOMEM;
	}

#endif
	tracemsg("sim_probe: SIM Module successfully probed\n");
	return ret;
}

/* DESCRIPTION:
       The remove routine to disassociate the SIM driver with the device.
 
   INPUTS:
       struct platform_device *pdev : the platform device pointer.

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.   
*/
static int sim_remove(struct platform_device *pdev)
{
	sim_dev = NULL;
	regulator_put(vsim_regulator);
	regulator_put(vsimcard_regulator);

	/* release the clock resources */
	clk_put(usim_fck);
	clk_put(usim_ick);
	clk_put(omap_96m_fck);
	clk_put(omap_120m_fck);
#ifdef CONFIG_QUICK_WAKEUP
	/* free the SIM status timer */
	wakeup_del_status_timer(sim_timer);
	sim_timer = NULL;
#endif
	device_destroy(sim_class, MKDEV(sim_module_major, 0));
	class_destroy(sim_class);
	unregister_chrdev(sim_module_major, SIM_DEV_NAME);
	tracemsg("sim_remove: Driver-device disassociation complete.\n");
	return 0;
}

/* DESCRIPTION:
       This routine suspends the device.

   INPUTS:
       struct platform_device *pdev : the platform device pointer
       pm_message_t state : the power mode that the system is going into

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.
*/
static int sim_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
	if (sim_low_power_enabled == FALSE)
		ret = -EBUSY;
	return ret;
}

/* DESCRIPTION:
       This routine resumes the device.

   INPUTS:
       struct platform_device *pdev : the platform device pointer

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.
*/
static int sim_resume(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_QUICK_WAKEUP
/* DESCRIPTION:
       The SIM quick wakeup callback function. This is called by the kernel clock handler when
   a GPT1 experation happens due to a SCIM set timer. This function also restarts the clock if
   the last status operation returned successfully.
 
   INPUTS:
       None. 

   OUTPUTS:
       int ret - the value return from the sim_slim_status_handler. If the value is success
                 the kernel puts SIM back to sleep. If it's failure, all drivers are resumed

   IMPORTANT NOTES:
       None.   
*/
static int sim_qw_callback(void)
{
    int ret = 0;
	
    ret = sim_slim_status_handler();

    if(ret == 0){
	    wakeup_start_status_timer(sim_timer, sim_poll_interval);
    }
    else {
            wakeup_stop_status_timer(sim_timer);
    }
    return ret;
}

/* DESCRIPTION:
       The SIM quick wakeup check looks to see if the GPT1 expiration was due to SCIM or some other
   source.
 
   INPUTS:
       None. 

   OUTPUTS:
       int success - 1 if SIM's timer was the cause of the wakeup, otherwise 0

   IMPORTANT NOTES:
       None.   
*/
static int sim_qw_check(void)
{
	UINT32 reg;
	int success = 0;

	/* check GPT1 irq wake up source*/
	reg = (volatile UINT32)read_reg((volatile UINT32 *)INTCPS_ITR1);
	reg &= WP_TPIR;

	/* 0 is for GPIO/USB wakeups */
	if ((reg != 0) && (wakeup_check_status_timer(sim_timer) == 0)){
		success = 1;
	}
	return success;
}
#endif

/* DESCRIPTION:
		The function starts the clock stop procedure and waits untill
		the procedure completes. The maximum wait is provided by the
		caller. This is called during slim sim and from User Space
		through IOCTL.
	INPUTS:
		reader_id sim reader ID
		clock_count wait time in micro seconds

	OUTPUTS:
		Returns 0 if successful.

	IMPORTANT NOTES:
		None.
*/
static int start_clk_stop_procedure(UINT8 reader_id, UINT32 clock_count)
{
	int status = 0;

	if (reader_id < NUM_SIM_MODULES) {

		/* clear the clock stop interrupt */
		write_reg_bits(&(sim_registers[reader_id]->irqstatus),
			USIM_STOP_CLK_MASK, 0);

		/* start the clock stop procedure */
		write_reg_bits(&(sim_registers[reader_id]->usimcmd),
			CMD_CLOCK_STOP_MASK, CMD_CLOCK_STOP_MASK);

		while (clock_count--) {
			if (((read_reg(&(sim_registers[reader_id]->irqstatus)) &
				USIM_STOP_CLK_MASK) == USIM_STOP_CLK_MASK)) {
				break;
			}
			udelay(1);
		}

		if (((read_reg(&(sim_registers[reader_id]->irqstatus)) &
				USIM_STOP_CLK_MASK) != USIM_STOP_CLK_MASK)) {
			tracemsg("Warning: start_clk_stop_procedure Failed\n");
			status = -EFAULT;
		}

		/* clear the Clock Stop Interrupt */
		write_reg_bits(&(sim_registers[reader_id]->irqstatus),
			USIM_STOP_CLK_MASK, 0);
    msleep(5);
	} else{
		tracemsg("%s%s",
			"Warning: Invalid reader ID passed ",
			"to start_clk_stop_procedure\n");
		status = -EFAULT;
	}
	return status;
}

/* DESCRIPTION:
       The SIM intialization function.
 
   INPUTS:
       None.

   OUTPUTS:
       Returns 0 if successful.

   IMPORTANT NOTES:
       None.   
*/
int __init sim_init(void)
{
	int ret = 0;

	tracemsg("sim_init: SIM driver loading...\n");

	/* Register the Driver */
	ret = platform_driver_register(&sim_driver);
	if (ret)
		tracemsg("sim_init: Driver registration failed.\n");
	else
		tracemsg("sim_init: Driver regristration passed.\n");

	return ret;
}

/* DESCRIPTION:
       The SIM device cleanup function
 
   INPUTS:
       None. 

   OUTPUTS:
       None.

   IMPORTANT NOTES:
       None.   
*/
static void __exit sim_exit(void)
{
	/* unregister the driver */
	platform_driver_unregister(&sim_driver);
	tracemsg("sim_exit: SIM driver successfully unloaded.\n");
}


/*
 * Module entry points
 */
module_init(sim_init);
module_exit(sim_exit);

MODULE_DESCRIPTION("SIM driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
