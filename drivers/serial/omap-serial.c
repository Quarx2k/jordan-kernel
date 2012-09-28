/*
 *  linux/drivers/serial/omap-serial.c
 *
 *  Modified: Manasa Gangaiah
 *  Copyright (C) 2009 Texas Instrument Inc.
 *
 *  Initial driver: Juha Yrjola
 *  Copyright (C) 2007 Nokia Corporation
 *
 *  Based on drivers/serial/pxa.c by Nicolas Pitre.
 *  Copyright (C) 2003 Monta Vista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#if defined(CONFIG_SERIAL_OMAP_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/serial_reg.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include <asm/irq.h>
#include <asm/dma.h>

#include <plat/dmtimer.h>
#include <plat/omap-serial.h>
#include <mach/gpio.h>
#include <plat/dma.h>
#include <plat/io.h>
#ifdef CONFIG_OMAP3_PM
#include <../arch/arm/mach-omap2/ti-compat.h>
#include <../arch/arm/mach-omap2/prcm-regs.h>
#endif
#include <asm/mach/serial_omap.h>

unsigned long isr8250_activity;
static int gps_port;

void __init omap_uart_set_gps_port(int port)
{
	gps_port = port;
}

#define CONSOLE_NAME	"console="

#ifdef CONFIG_ARCH_OMAP34XX
#define OMAP_MDR1_DISABLE 0x07
#define OMAP_MDR1_MODE13X 0x03
#define OMAP_MDR1_MODE16X 0x00
#define OMAP_MODE13X_SPEED	230400
#endif


/* Debugging Macros */
#undef DEBUG

#ifdef DEBUG
#define DPRINTK  printk
#else
#define DPRINTK(x...)
#endif

/*
 * We default to IRQ0 for the "no irq" hack.   Some
 * machine types want others as well - they're free
 * to redefine this in their header file.
 */
#define is_real_interrupt(irq)  ((irq) != 0)

/* TBD: move this to header file */
static u8 uart_dma_tx[MAX_UARTS + 1] =
    { OMAP24XX_DMA_UART1_TX, OMAP24XX_DMA_UART2_TX, OMAP24XX_DMA_UART3_TX };
static u8 uart_dma_rx[MAX_UARTS + 1] =
    { OMAP24XX_DMA_UART1_RX, OMAP24XX_DMA_UART2_RX, OMAP24XX_DMA_UART3_RX };


struct uart_omap_dma {
	int rx_dma_channel;
	int tx_dma_channel;
	dma_addr_t rx_buf_dma_phys;	/* Physical adress of RX DMA buffer */
	dma_addr_t tx_buf_dma_phys;	/* Physical adress of TX DMA buffer */
	/*
	 * Buffer for rx dma.It is not required for tx because the buffer
	 * comes from port structure
	 */
	unsigned char *rx_buf;
	unsigned int prev_rx_dma_pos;
	int tx_buf_size;
	int tx_dma_state;
	int rx_dma_state;
	spinlock_t tx_lock;
	spinlock_t rx_lock;
	struct timer_list 	rx_timer;/* timer to poll activity on rx dma */
	int rx_buf_size;
	int rx_timeout;
};

struct uart_omap_port {
	struct uart_port	port;
	struct uart_omap_dma	uart_dma;
	struct platform_device	*pdev;

	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	int			use_dma;
	int			is_buf_dma_alloced;
	int			restore_autorts;
	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
	unsigned int		lsr_break_flag;
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
	unsigned char		msr_saved_flags;
	char			name[12];
	int			use_console;
	spinlock_t		uart_lock;
	char			dev_name[50];
#ifdef CONFIG_SERIAL_OMAP3430_HW_FLOW_CONTROL
	unsigned char ctsrts;
#endif
	struct work_struct	tty_work;
};

static struct uart_omap_port *ui[MAX_UARTS + 1];
unsigned int fcr[MAX_UARTS];

static struct wake_lock omap_serial_wakelock;
static struct workqueue_struct *omap_serial_workqueue;

/* Forward declaration of dma callback functions */
static void uart_tx_dma_callback(int lch, u16 ch_status, void *data);
static void serial_omap_display_reg(struct uart_port *port);
static void serial_omap_rx_timeout(unsigned long uart_no);
static void serial_omap_start_rxdma(struct uart_omap_port *up);
static void serial_omap_set_autorts(struct uart_omap_port *p, int set);

#define DBG_RX_DATA 0

int console_detect(char *str)
{
	extern char *saved_command_line;
	char *next, *start = NULL;
	int i;

	i = strlen(CONSOLE_NAME);
	next = saved_command_line;

	while ((next = strchr(next, 'c')) != NULL) {
		if (!strncmp(next, CONSOLE_NAME, i)) {
			start = next;
			break;
		} else {
			next++;
		}

	}
	if (!start)
		return -EPERM;
	i = 0;
	start = strchr(start, '=') + 1;
	while (*start != ',') {
		str[i++] = *start++;
		if (i > 6) {
			printk(KERN_ERR "Invalid Console Name\n");
			return -EPERM;
		}
	}
	str[i] = '\0';
	return 0;
}

static inline unsigned int serial_in(struct uart_omap_port *up, int offset)
{
	offset <<= up->port.regshift;
	if (up->pdev->id != 4)
		return readb(up->port.membase + offset);
	else
		return readw(up->port.membase + offset);
}

static inline void serial_out(struct uart_omap_port *up, int offset, int value)
{
	offset <<= up->port.regshift;
	if (up->pdev->id != 4)
		writeb(value, up->port.membase + offset);
	else
		writew(value, up->port.membase + offset);
}

static inline void serial_omap_clear_fifos(struct uart_omap_port *p)
{
		serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO |
			       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_out(p, UART_FCR, 0);
		fcr[p->pdev->id - 1] = 0;
}

/*
 * Work Around for Errata i202 (3430 - 1.12, 3630 - 1.6)
 * The access to uart register after MDR1 Access
 * causes UART to corrupt data.
 *
 * Need a delay =
 * 5 L4 clock cycles + 5 UART functional clock cycle (@48MHz = ~0.2uS)
 * give 5 times as much
 *
 * uart_no : Should be a Zero Based Index Value always.
 */
void omap_uart_mdr1_errataset(struct uart_omap_port *up, u8 mdr1_val,
		u8 fcr_val)
{
	/* 10 retries, in this the FiFO's should get cleared */
	u8 timeout = 0x05;

	serial_out(up, UART_OMAP_MDR1, mdr1_val);
	udelay(1);
	serial_out(up, UART_FCR, fcr_val | UART_FCR_CLEAR_XMIT |
			UART_FCR_CLEAR_RCVR);
	/*
	 * Wait for FIFO to empty: when empty, RX_FIFO_E bit is 0 and
	 * TX_FIFO_E bit is 1.
	 */
	while (UART_LSR_THRE != (serial_in(up, UART_LSR) &
				(UART_LSR_THRE | UART_LSR_DR))) {
		timeout--;
		if (!timeout) {
			/* Should *never* happen. we warn and carry on */
			printk(KERN_WARNING "Errata i202: timedout %x %d\n", \
				serial_in(up, UART_LSR), up->pdev->id);
			break;
		}
		udelay(1);
	}
}

/*
 * We have written our own function to get the divisor so as to support
 * 13x mode. TBD see if this can be integrated with uart_get_divisor.
 */
static unsigned int
serial_omap_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int divisor;
	if (baud > OMAP_MODE13X_SPEED && baud != 3000000)
		divisor = 13;
	else
		divisor = 16;
	return port->uartclk/(baud * divisor);
}

static void serial_omap_stop_rxdma(struct uart_omap_port *up)
{
	if (up->uart_dma.rx_dma_state) {
		del_timer(&up->uart_dma.rx_timer);
		omap_stop_dma(up->uart_dma.rx_dma_channel);
		omap_free_dma(up->uart_dma.rx_dma_channel);
		up->uart_dma.rx_dma_channel = 0xFF;
		up->uart_dma.rx_dma_state = 0x0;
	}
}

static void serial_omap_enable_ms(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	DPRINTK("serial_omap_enable_ms+%d\n", up->pdev->id);
	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}

static void serial_omap_stop_tx(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	if (up->use_dma && up->uart_dma.tx_dma_channel != 0xFF) {
		/*
		 * Check if dma is still active . If yes do nothing,
		 * return. Else stop dma.
		 */
		int status = omap_readl(OMAP34XX_DMA4_BASE +
				OMAP_DMA4_CCR(up->uart_dma.tx_dma_channel));
		if (status & (1 << 7))
			return;
		omap_stop_dma(up->uart_dma.tx_dma_channel);
		omap_free_dma(up->uart_dma.tx_dma_channel);
		up->uart_dma.tx_dma_channel = 0xFF;
	}

	if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
#ifdef CONFIG_PM
	if (!up->uart_dma.rx_dma_state) {
		unsigned int tmp;
		tmp = (serial_in(up, UART_OMAP_SYSC) & 0x7) | (2 << 3);
		serial_out(up, UART_OMAP_SYSC, tmp); /* smart-idle */
	}
#endif
}

static void serial_omap_stop_rx(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	serial_omap_stop_rxdma(up);
	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);

}

static inline void receive_chars(struct uart_omap_port *up, int *status)
{
	unsigned int ch, flag;
	int max_count = 256;

#if DBG_RX_DATA
	printk("[RX]: ");
#endif
	do {
		ch = serial_in(up, UART_RX);
#if DBG_RX_DATA
		if ((ch >= 32) && (ch < 127))
			printk("%c", ch);
		else
			printk("{0x%.2x}", ch);
#endif
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(*status & (UART_LSR_BI | UART_LSR_PE |
				       UART_LSR_FE | UART_LSR_OE))) {
			/*
			 * For statistics only
			 */
			if (*status & UART_LSR_BI) {
				*status &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				/*
				 * We do the SysRQ and SAK checking
				 * here because otherwise the break
				 * may get masked by ignore_status_mask
				 * or read_status_mask.
				 */
				if (uart_handle_break(&up->port))
					goto ignore_char;
			} else if (*status & UART_LSR_PE)
				up->port.icount.parity++;
			else if (*status & UART_LSR_FE)
				up->port.icount.frame++;
			if (*status & UART_LSR_OE)
				up->port.icount.overrun++;

			/*
			 * Mask off conditions which should be ignored.
			 */
			*status &= up->port.read_status_mask;

#ifdef CONFIG_SERIAL_OMAP_CONSOLE
			if (up->port.line == up->port.cons->index) {
				/* Recover the break flag from console xmit */
				*status |= up->lsr_break_flag;
				up->lsr_break_flag = 0;
			}
#endif
			if (*status & UART_LSR_BI)
				flag = TTY_BREAK;
			else if (*status & UART_LSR_PE)
				flag = TTY_PARITY;
			else if (*status & UART_LSR_FE)
				flag = TTY_FRAME;
		}

		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

		uart_insert_char(&up->port, *status, UART_LSR_OE, ch, flag);

ignore_char:
		*status = serial_in(up, UART_LSR);
	} while ((*status & UART_LSR_DR) && (max_count-- > 0));

	queue_work(omap_serial_workqueue, &up->tty_work);

#if DBG_RX_DATA
	printk("\n");
#endif
}

static void tty_flip_buffer_work(struct work_struct *work)
{
	struct uart_omap_port *up =
			container_of(work, struct uart_omap_port, tty_work);
	struct tty_struct *tty = up->port.state->port.tty;

	tty_flip_buffer_push(tty);
}

static void transmit_chars(struct uart_omap_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count;

	if (up->port.x_char) {
		serial_out(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
		serial_omap_stop_tx(&up->port);
		return;
	}

	count = up->port.fifosize;

	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		serial_omap_stop_tx(&up->port);
}

static void serial_omap_start_tx(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
#ifdef CONFIG_PM
		/* Disallow OCP bus idle. UART TX irqs are not seen during
		 * bus idle. Alternative is to set kernel timer at fifo
		 * drain rate.
		 */
		unsigned int tmp;
		tmp = (serial_in(up, UART_OMAP_SYSC) & 0x7) | (1 << 3);
		serial_out(up, UART_OMAP_SYSC, tmp); /* no-idle */
#endif

	if (up->use_dma && !(up->port.x_char)) {

		struct circ_buf *xmit = &up->port.state->xmit;
		unsigned int start = up->uart_dma.tx_buf_dma_phys +
				     (xmit->tail & (UART_XMIT_SIZE - 1));
		if (uart_circ_empty(xmit) || up->uart_dma.tx_dma_state)
			return;
		spin_lock(&(up->uart_dma.tx_lock));
		up->uart_dma.tx_dma_state = 1;
		spin_unlock(&(up->uart_dma.tx_lock));

		up->uart_dma.tx_buf_size = uart_circ_chars_pending(xmit);
		/* It is a circular buffer. See if the buffer has wounded back.
		 * If yes it will have to be transferred in two separate dma
		 * transfers */
		if (start + up->uart_dma.tx_buf_size >= up->uart_dma.tx_buf_dma_phys + UART_XMIT_SIZE)
			up->uart_dma.tx_buf_size = (up->uart_dma.tx_buf_dma_phys + UART_XMIT_SIZE) - start;

		if (up->uart_dma.tx_dma_channel == 0xFF) {
			omap_request_dma(uart_dma_tx[up->pdev->id-1],
					 "UART Tx DMA",
					 (void *)uart_tx_dma_callback, up,
					&(up->uart_dma.tx_dma_channel));
		}
		omap_set_dma_dest_params(up->uart_dma.tx_dma_channel, 0,
					 OMAP_DMA_AMODE_CONSTANT,
					 UART_BASE(up->pdev->id - 1), 0, 0);
		omap_set_dma_src_params(up->uart_dma.tx_dma_channel, 0,
			OMAP_DMA_AMODE_POST_INC, start, 0, 0);

		omap_set_dma_transfer_params(up->uart_dma.tx_dma_channel,
					     OMAP_DMA_DATA_TYPE_S8,
					     up->uart_dma.tx_buf_size, 1,
					     OMAP_DMA_SYNC_ELEMENT,
					     uart_dma_tx[(up->pdev->id)-1], 0);

		omap_start_dma(up->uart_dma.tx_dma_channel);

	} else if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}

	if (up->restore_autorts) {
		serial_omap_set_autorts(up, 1);
		up->restore_autorts = 0;
	}
}

static unsigned int check_modem_status(struct uart_omap_port *up)
{
	int status;
	status = serial_in(up, UART_MSR);

	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;

	if ((status & UART_MSR_ANY_DELTA) == 0)
		return status;
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.state != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change
				(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change
				(&up->port, status & UART_MSR_CTS);
		wake_up_interruptible(&up->port.state->port.delta_msr_wait);
	}

	return status;
}

/*
 * This handles the interrupt from one port.
 */
static inline irqreturn_t serial_omap_irq(int irq, void *dev_id)
{
	struct uart_omap_port *up = dev_id;
	unsigned int iir, lsr;

	iir = serial_in(up, UART_IIR);
	if (iir & UART_IIR_NO_INT)
		return IRQ_NONE;
	lsr = serial_in(up, UART_LSR);
	if ((iir & 0x4) && up->use_dma) {
		up->ier &= ~UART_IER_RDI;
		serial_out(up, UART_IER, up->ier);
		serial_omap_start_rxdma(up);
	} else if (lsr & UART_LSR_DR) {
		receive_chars(up, &lsr);
		/* XXX: After driver resume optimization, lower this */
		wake_lock_timeout(&omap_serial_wakelock, (HZ * 1));
	}
	check_modem_status(up);
	if ((lsr & UART_LSR_THRE) && (iir & 0x2))
		transmit_chars(up);
	isr8250_activity = jiffies;

	return IRQ_HANDLED;
}

static unsigned int serial_omap_tx_empty(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags;
	unsigned int ret;

	DPRINTK("serial_omap_tx_empty+%d\n", up->pdev->id);
	spin_lock_irqsave(&up->port.lock, flags);
	ret = serial_in(up, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return ret;
}

static unsigned int serial_omap_get_mctrl(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char status;
	unsigned int ret;

	status = check_modem_status(up);
	DPRINTK("serial_omap_get_mctrl+%d\n", up->pdev->id);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial_omap_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char mcr = 0;

	DPRINTK("serial_omap_set_mctrl+%d\n", up->pdev->id);
	if (mctrl & TIOCM_RTS) {
		/*
		 * We need to be careful not to cause
		 * RTS to assert when we have a pending
		 * auto-rts restore.
		 */
		if (!up->restore_autorts)
			mcr |= UART_MCR_RTS;
	}
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr |= up->mcr;
	serial_out(up, UART_MCR, mcr);
}

static void serial_omap_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags;

	DPRINTK("serial_omap_break_ctl+%d\n", up->pdev->id);
	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static void serial_omap_wake_peer(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	struct plat_serialomap_port *pd = up->pdev->dev.platform_data;
	if (pd->wake_gpio_strobe) {
		gpio_direction_output(pd->wake_gpio_strobe, 1);
		udelay(5);
		gpio_direction_output(pd->wake_gpio_strobe, 0);
		udelay(5);
	}
}

static void serial_omap_set_autorts(struct uart_omap_port *p, int set)
{
        u8 lcr_val = 0, mcr_val = 0, efr_val = 0;
        u8 lcr_backup = 0, mcr_backup = 0, efr_backup = 0;

        lcr_val = serial_in(p, UART_LCR);
        lcr_backup = lcr_val;
	/* Enter Config mode B */
        serial_out(p, UART_LCR, 0xbf);

        efr_val = serial_in(p, UART_EFR);
        efr_backup = efr_val;

	/*
	 * Enhanced functions write enable.
	 * Enables writes to IER[7:4], FCR[5:4], MCR[7:5]
	 */
        serial_out(p, UART_EFR, efr_val | 0x10);

        mcr_val = serial_in(p, UART_MCR);
        mcr_backup = mcr_val;
	/* Enable access to TCR_REG and TLR_REG */
        serial_out(p, UART_MCR, mcr_val | 0x40);

	/* Set RX_FIFO_TRIG levels */
        serial_out(p, 0x18, 0x0f);

        efr_val = serial_in(p, UART_EFR);
        if (set)
		serial_out(p, UART_EFR, efr_val | (1 << 6));
        else
                serial_out(p, UART_EFR, efr_val & ~(1 << 6));


        mcr_val = serial_in(p, UART_MCR);
	/* Restore original state of TCR_TLR access */
        serial_out(p, UART_MCR, (mcr_val & ~0x40) | (mcr_backup & 0x40));

	/* Enhanced function write disable. */
	serial_out(p, UART_EFR, serial_in(p, UART_EFR) & ~0x10);

	/* Normal operation */
        serial_out(p, UART_LCR, lcr_backup);
}

static int serial_omap_startup(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags;
	int irq_flags = port->flags & UPF_SHARE_IRQ ? IRQF_SHARED : 0;
	int retval;

	/* Zoom2 has GPIO_102 connected to Serial device:
	* Active High
	*/

	if (up->port.flags & UPF_TRIGGER_HIGH)
		irq_flags |= IRQF_TRIGGER_HIGH;

	if (up->port.flags & UPF_SHARE_IRQ)
		irq_flags |= IRQF_SHARED;

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(up->port.irq, serial_omap_irq, irq_flags,
			     up->name, up);
	if (retval) {
		printk(KERN_ERR "%s: Failed to register IRQ %d for %s (%d)\n",
		       __func__, up->port.irq, up->name, retval);
		return retval;
	}


	/* do not let tty layer execute RX in global workqueue, use a
	 * dedicated workqueue managed by this driver */
	port->state->port.tty->low_latency = 1;

	/*
	 * Stop the baud clock and disable the UART. UART will be enabled
	 * back in set_termios. This is essential for DMA mode operations.
	 */
	serial_out(up, UART_LCR, UART_LCR_DLAB);
	serial_out(up, UART_DLL, 0);
	serial_out(up, UART_DLM, 0);
	serial_out(up, UART_LCR, 0);
	omap_uart_mdr1_errataset(up, OMAP_MDR1_DISABLE, 0);

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial_omap_clear_fifos(up);
	serial_out(up, UART_SCR, 0x00);
	/* For Hardware flow control */
//	serial_out(up, UART_MCR, 0x2);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_in(up, UART_LSR);
	if (serial_in(up, UART_LSR) & UART_LSR_DR)
		(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);
	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);
	spin_lock_irqsave(&up->port.lock, flags);
	if (up->port.flags & UPF_FOURPORT) {
		if (!is_real_interrupt(up->port.irq))
			up->port.mctrl |= TIOCM_OUT1;
	} else {
		/*
		 * Most PC uarts need OUT2 raised to enable interrupts.
		 */
		if (is_real_interrupt(up->port.irq))
			up->port.mctrl |= TIOCM_OUT2;
	}
	serial_omap_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	up->msr_saved_flags = 0;



	if (up->port.flags & UPF_FOURPORT) {
		unsigned int icp;
		/*
		 * Enable interrupts on the AST Fourport board
		 */
		icp = (up->port.iobase & 0xfe0) | 0x01f;
		outb_p(0x80, icp);
		(void) inb_p(icp);
	}
	if (up->use_dma) {
		if (!up->is_buf_dma_alloced) {
			free_page((unsigned long)up->port.state->xmit.buf);
			up->port.state->xmit.buf = NULL;
			up->port.state->xmit.buf = dma_alloc_coherent(NULL,
							UART_XMIT_SIZE,
						(dma_addr_t *)&(up->uart_dma.tx_buf_dma_phys), 0);
			up->is_buf_dma_alloced = 1;
		}
		init_timer(&(up->uart_dma.rx_timer));
		up->uart_dma.rx_timer.function = serial_omap_rx_timeout;
		up->uart_dma.rx_timer.data = up->pdev->id;
		/* Currently the buffer size is 4KB. Can increase it later*/
		up->uart_dma.rx_buf = dma_alloc_coherent(NULL,
					up->uart_dma.rx_buf_size,
					(dma_addr_t *)&(up->uart_dma.rx_buf_dma_phys), 0);
		serial_omap_start_rxdma(up);
	} else {
		/*
		* Finally, enable interrupts.  Note: Modem status interrupts
		* are set via set_termios(), which will be occurring imminently
		* anyway, so we don't enable them here.
		*/
		up->ier = UART_IER_RLSI | UART_IER_RDI;
			/*| UART_IER_RTOIE |UART_IER_THRI; */
		serial_out(up, UART_IER, up->ier);
	}

	return 0;
}

static void serial_omap_shutdown(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned long flags;
	u8 lcr, efr;

	DPRINTK("serial_omap_shutdown+%d\n", up->pdev->id);
	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_out(up, UART_IER, 0);

	/* 
	 * If we're using auto-rts then disable it.
	 */
	lcr = serial_in(up, UART_LCR);
	serial_out(up, UART_LCR, 0xbf);
	efr = serial_in(up, UART_EFR);
	serial_out(up, UART_LCR, lcr);

	if (efr & UART_EFR_RTS) {
		serial_omap_set_autorts(up, 0);
		up->restore_autorts = 1;
	}

	spin_lock_irqsave(&up->port.lock, flags);
	if (up->port.flags & UPF_FOURPORT) {
		/* reset interrupts on the AST Fourport board */
		inb((up->port.iobase & 0xfe0) | 0x1f);
		up->port.mctrl |= TIOCM_OUT1;
	} else
		up->port.mctrl &= ~TIOCM_OUT2;
	serial_omap_set_mctrl(&up->port, (up->port.mctrl & ~TIOCM_RTS));
	spin_unlock_irqrestore(&up->port.lock, flags);

	if (up->pdev->id == gps_port) {
		serial_out(up, UART_LCR, UART_LCR_DLAB);
		serial_out(up, UART_DLL, 0);
		serial_out(up, UART_DLM, 0);
		serial_out(up, UART_LCR, 0);
		omap_uart_mdr1_errataset(up, OMAP_MDR1_DISABLE, 0);
	}

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_in(up, UART_LCR) & ~UART_LCR_SBC);
	serial_omap_clear_fifos(up);

	/*
	 * Read data port to reset things, and then free the irq
	 */
	if (serial_in(up, UART_LSR) & UART_LSR_DR)
		(void) serial_in(up, UART_RX);
	if (up->use_dma) {
		int tmp;
		if (up->is_buf_dma_alloced) {
			dma_free_coherent(up->port.dev,
					  UART_XMIT_SIZE,
					  up->port.state->xmit.buf,
					  up->uart_dma.tx_buf_dma_phys);
			up->port.state->xmit.buf = NULL;
			up->is_buf_dma_alloced = 0;
		}
		/*TBD: Check if this is really needed here*/
		serial_omap_stop_rx(port);
		dma_free_coherent(up->port.dev,
				  up->uart_dma.rx_buf_size,
				  up->uart_dma.rx_buf, up->uart_dma.rx_buf_dma_phys);
		up->uart_dma.rx_buf = NULL;
		tmp = serial_in(up, UART_OMAP_SYSC) & 0x7;
		serial_out(up, UART_OMAP_SYSC, tmp); /* force-idle */
	}

	free_irq(up->port.irq, up);

	if (cancel_work_sync(&up->tty_work))
		tty_flip_buffer_work(&up->tty_work);
}

static void
serial_omap_set_termios(struct uart_port *port, struct ktermios *termios,
			struct ktermios *old)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char cval;
	unsigned char efr = 0;
	unsigned long flags;
	unsigned int baud, quot;
	unsigned int status;

	/*
	 * Disable interrupt
	 */
	serial_out(up, UART_IER, 0);

	serial_out(up, UART_LCR, UART_LCR_DLAB);
	serial_out(up, UART_DLL, 0);
	serial_out(up, UART_DLM, 0);
	serial_out(up, UART_LCR, 0);
	omap_uart_mdr1_errataset(up, OMAP_MDR1_DISABLE, 0);
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;

	/*
	 * Ask the core to calculate the divisor for us.
	 */

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/13);
	quot = serial_omap_get_divisor(port, baud);

	if (up->use_dma)
		fcr[up->pdev->id - 1] = UART_FCR_ENABLE_FIFO | 0x1 << 6 | 0x1 << 4 | UART_FCR_DMA_SELECT;
	else
		fcr[up->pdev->id - 1] = UART_FCR_ENABLE_FIFO;

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characters to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	if (termios->c_cflag & CRTSCTS) {
#ifdef CONFIG_SERIAL_OMAP3430_HW_FLOW_CONTROL
		efr |= ((up->ctsrts & UART_EFR_CTS) |
				(up->restore_autorts ? 0 : UART_EFR_RTS));
#else
		efr |= (UART_EFR_CTS | (up->restore_autorts ? 0 : UART_EFR_RTS));
#endif
	}

	serial_out(up, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */
	serial_out(up, UART_DLL, quot & 0xff);		/* LS of divisor */
	serial_out(up, UART_DLM, quot >> 8);		/* MS of divisor */

	serial_out(up, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;				/* Save LCR */
	if (up->use_dma)
		serial_out(up, UART_OMAP_SCR  , ((1 << 6) | (1 << 7)));

	serial_out(up, UART_LCR, 0xbf);	/* Access EFR */
	serial_out(up, UART_EFR, UART_EFR_ECB);
	serial_out(up, UART_LCR, 0x0);		/* Access FCR */
	serial_out(up, UART_FCR, fcr[up->pdev->id - 1]);
	serial_out(up, UART_LCR, 0xbf);	/* Access EFR */
	serial_out(up, UART_EFR, efr);
	serial_out(up, UART_LCR, cval);	/* Access FCR */

	serial_omap_set_mctrl(&up->port, up->port.mctrl);
	/*
	 * Clear all the status registers and RX register before
	 * enabling UART
	 */
	(void) serial_in(up, UART_LSR);
	if (serial_in(up, UART_LSR) & UART_LSR_DR)
		(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);

	if (baud > 230400 && baud != 3000000)
		omap_uart_mdr1_errataset(up, OMAP_MDR1_MODE13X,
				fcr[up->pdev->id - 1]);
	else
		omap_uart_mdr1_errataset(up, OMAP_MDR1_MODE16X,
				fcr[up->pdev->id - 1]);

	if (UART_ENABLE_MS(&up->port, termios->c_cflag)) {
		status = serial_in(up, UART_MSR);
		if (status & UART_MSR_CTS)
			uart_handle_cts_change
					(&up->port, status & UART_MSR_CTS);
	}

	spin_unlock_irqrestore(&up->port.lock, flags);

	DPRINTK("serial_omap_set_termios+%d\n", up->pdev->id);
	/*
	 * Comment out the serial_omap_display_reg()
	 * to avoid any register access after MDR1.
	 * For debugging, it can be enabled manually.
	 */
	/*
	serial_omap_display_reg(port);
	*/
}

static void
serial_omap_pm(struct uart_port *port, unsigned int state,
	       unsigned int oldstate)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned char efr;
	DPRINTK("serial_omap_pm+%d\n", up->pdev->id);
	serial_out(up, UART_LCR, 0xBF);
	efr = serial_in(up, UART_EFR);
	serial_out(up, UART_EFR, efr | UART_EFR_ECB);
	serial_out(up, UART_LCR, 0);

	serial_out(up, UART_IER, (state != 0) ? UART_IERX_SLEEP : 0);
	serial_out(up, UART_LCR, 0xBF);
	serial_out(up, UART_EFR, efr);
	serial_out(up, UART_LCR, 0);
}

static void serial_omap_release_port(struct uart_port *port)
{
	DPRINTK("serial_omap_release_port+\n");
}

static int serial_omap_request_port(struct uart_port *port)
{
	DPRINTK("serial_omap_request_port+\n");
	return 0;
}

static void serial_omap_config_port(struct uart_port *port, int flags)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	DPRINTK("serial_omap_config_port+%d\n", up->pdev->id);
	up->port.type = PORT_OMAP;
}

static int
serial_omap_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	DPRINTK("serial_omap_verify_port+\n");
	return -EINVAL;
}

static const char *
serial_omap_type(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	DPRINTK("serial_omap_type+%d\n", up->pdev->id);
	return up->name;
}

#ifdef CONFIG_SERIAL_OMAP_CONSOLE

static struct uart_omap_port *serial_omap_console_ports[4];

static struct uart_driver serial_omap_reg;

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

/*
 *	Wait for transmitter & holding register to empty
 */
static inline void wait_for_xmitr(struct uart_omap_port *up)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	do {
		status = serial_in(up, UART_LSR);

		if (status & UART_LSR_BI)
			up->lsr_break_flag = UART_LSR_BI;

		if (--tmout == 0)
			break;
		udelay(1);
	} while ((status & BOTH_EMPTY) != BOTH_EMPTY);

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		tmout = 1000000;
		for (tmout = 1000000; tmout; tmout--) {
			unsigned int msr = serial_in(up, UART_MSR);
			up->msr_saved_flags |= msr & MSR_SAVE_FLAGS;
			if (msr & UART_MSR_CTS)
				break;
			udelay(1);
		}
	}
}

static void serial_omap_console_putchar(struct uart_port *port, int ch)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;

	wait_for_xmitr(up);
	serial_out(up, UART_TX, ch);
}

/*
 * Print a string to the serial port trying not to disturb
 * any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
serial_omap_console_write(struct console *co, const char *s,
		unsigned int count)
{
	/* TBD: In 8250 interrupts were disabled in the beginning of this
	 * function and enabled in the end. We might need to do the same*/
	struct uart_omap_port *up = serial_omap_console_ports[co->index];
	unsigned int ier;

	/*
	 *	First save the IER then disable the interrupts
	 */
	ier = serial_in(up, UART_IER);
	serial_out(up, UART_IER, 0);

	uart_console_write(&up->port, s, count, serial_omap_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	wait_for_xmitr(up);
	serial_out(up, UART_IER, ier);
	/*
	 *	The receive handling will happen properly because the
	 *	receive ready bit will still be set; it is not cleared
	 *	on read.  However, modem control will not, we must
	 *	call it if we have saved something in the saved flags
	 *	while processing with interrupts off.
	 */
	if (up->msr_saved_flags)
		check_modem_status(up);
}

static int __init
serial_omap_console_setup(struct console *co, char *options)
{
	struct uart_omap_port *up;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int r;

	if (serial_omap_console_ports[co->index] == NULL)
		return -ENODEV;
	up = serial_omap_console_ports[co->index];

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	r = uart_set_options(&up->port, co, baud, parity, bits, flow);

	return r;
}

static struct console serial_omap_console = {
	.name		= "ttyS",
	.write		= serial_omap_console_write,
	.device		= uart_console_device,
	.setup		= serial_omap_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial_omap_reg,
};

static void serial_omap_add_console_port(struct uart_omap_port *up)
{
	serial_omap_console_ports[up->pdev->id - 1] = up;
}

#define OMAP_CONSOLE	(&serial_omap_console)

#else

#define OMAP_CONSOLE	NULL

static inline void serial_omap_add_console_port(struct uart_omap_port *up) {}

#endif

struct uart_ops serial_omap_pops = {
	.tx_empty	= serial_omap_tx_empty,
	.set_mctrl	= serial_omap_set_mctrl,
	.get_mctrl	= serial_omap_get_mctrl,
	.stop_tx	= serial_omap_stop_tx,
	.start_tx	= serial_omap_start_tx,
	.stop_rx	= serial_omap_stop_rx,
	.enable_ms	= serial_omap_enable_ms,
	.break_ctl	= serial_omap_break_ctl,
	.startup	= serial_omap_startup,
	.shutdown	= serial_omap_shutdown,
	.set_termios	= serial_omap_set_termios,
	.pm		= serial_omap_pm,
	.type		= serial_omap_type,
	.release_port	= serial_omap_release_port,
	.request_port	= serial_omap_request_port,
	.config_port	= serial_omap_config_port,
	.verify_port	= serial_omap_verify_port,
	.wake_peer	= serial_omap_wake_peer,
};

static struct uart_driver serial_omap_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "OMAP-SERIAL",
	.dev_name	= "ttyS",
	.major		= TTY_MAJOR,
	.minor		= 64,
	.nr		= 4,
	.cons		= OMAP_CONSOLE,
};

static int serial_omap_remove(struct platform_device *dev);
static int serial_omap_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct uart_omap_port *up = platform_get_drvdata(pdev);
	unsigned int tmp;
	u8 lcr, efr;
	static unsigned int fifo_suspendbrks;

	/* Disable interrupts from this port */
	serial_out(up, UART_IER, 0);

	/* If we're using auto-rts then disable it. */
	lcr = serial_in(up, UART_LCR);
	serial_out(up, UART_LCR, 0xbf);
	efr = serial_in(up, UART_EFR);
	serial_out(up, UART_LCR, lcr);

	if (efr & UART_EFR_RTS) {
		serial_omap_set_autorts(up, 0);
		up->restore_autorts = 1;
		/*
		 * Force RTS output to inactive (high) after disable autorts
		 * mode. This RTS bit might not be restored when enable autorts
		 * next time, since the RTS output controlled by hardware
		 * flow control.
		 */
		serial_omap_set_mctrl(&up->port, (up->port.mctrl & ~TIOCM_RTS));
	}

	/*
	 * There seems to be a window here where
	 * data could still be on the way to the
	 * fifo. This delay is ~1 byte time @ 115.2k
	 */
	udelay(80);

	if (are_driveromap_uarts_active(up->port.line)) {
		fifo_suspendbrks++;
		printk(KERN_WARNING "UART FIFO break suspend %d\n",
					fifo_suspendbrks);

		if (up->restore_autorts) {
			serial_omap_set_autorts(up, 1);
			up->restore_autorts = 0;
		}
		serial_out(up, UART_IER, up->ier);
		return -EBUSY;
	}

	serial_out(up, UART_IER, up->ier);

	if (up)
		uart_suspend_port(&serial_omap_reg, &up->port);
	if (up->use_dma) {
		/*
		 * Silicon Errata i291 workaround.
		 * UART Module has to be put in force idle if it is
		 * configured in DMA mode and when there is no activity
		 * expected.
		 */
		tmp = (serial_in(up, UART_OMAP_SYSC) & 0x7);
		serial_out(up, UART_OMAP_SYSC, tmp); /* force-idle */
	}
	return 0;
}

static int serial_omap_resume(struct platform_device *dev)
{
	struct uart_omap_port *up = platform_get_drvdata(dev);
	if (up)
		uart_resume_port(&serial_omap_reg, &up->port);

	return 0;
}

static void serial_omap_rx_timeout(unsigned long uart_no)
{
	struct uart_omap_port *up = ui[uart_no - 1];
	unsigned int curr_dma_pos;
	curr_dma_pos = omap_readl(OMAP34XX_DMA4_BASE + OMAP_DMA4_CDAC(up->uart_dma.rx_dma_channel));
	if ((curr_dma_pos == up->uart_dma.prev_rx_dma_pos) || (curr_dma_pos == 0)) {
		/*
		 * If there is no transfer rx happening for 10sec then stop the dma
		 * else just restart the timer. See if 10 sec can be improved.
		 */
		if (jiffies_to_msecs(jiffies - isr8250_activity) < 10000)
			mod_timer(&up->uart_dma.rx_timer, jiffies +
				usecs_to_jiffies(up->uart_dma.rx_timeout));
		else {
			del_timer(&up->uart_dma.rx_timer);
			serial_omap_stop_rxdma(up);
			up->ier |= UART_IER_RDI;
			serial_out(up, UART_IER, up->ier);
		}

		return;
	} else {
		unsigned int curr_transmitted_size = curr_dma_pos - up->uart_dma.prev_rx_dma_pos;
		up->port.icount.rx += curr_transmitted_size;
		tty_insert_flip_string(up->port.state->port.tty, up->uart_dma.rx_buf + (up->uart_dma.prev_rx_dma_pos - up->uart_dma.rx_buf_dma_phys), curr_transmitted_size);
		queue_work(omap_serial_workqueue, &up->tty_work);
		up->uart_dma.prev_rx_dma_pos = curr_dma_pos;
		if (up->uart_dma.rx_buf_size + up->uart_dma.rx_buf_dma_phys == curr_dma_pos) {
			serial_omap_start_rxdma(up);
		} else
			mod_timer(&up->uart_dma.rx_timer,
				  jiffies + usecs_to_jiffies(up->uart_dma.rx_timeout));
		isr8250_activity = jiffies;
	}
}

static void uart_rx_dma_callback(int lch, u16 ch_status, void *data)
{
	return;
}

static void serial_omap_start_rxdma(struct uart_omap_port *up)
{
#ifdef CONFIG_PM
	/* Disallow OCP bus idle. UART TX irqs are not seen during
	 * bus idle. Alternative is to set kernel timer at fifo
	 * drain rate.
	 */
	unsigned int tmp;
	tmp = (serial_in(up, UART_OMAP_SYSC) & 0x7) | (1 << 3);
	serial_out(up, UART_OMAP_SYSC, tmp); /* no-idle */
#endif
	if (up->uart_dma.rx_dma_channel == 0xFF) {
		omap_request_dma(uart_dma_rx[up->pdev->id-1], "UART Rx DMA",
				(void *)uart_rx_dma_callback, up,
				&(up->uart_dma.rx_dma_channel));
		omap_set_dma_src_params(up->uart_dma.rx_dma_channel, 0,
					OMAP_DMA_AMODE_CONSTANT,
					UART_BASE(up->pdev->id - 1), 0, 0);
		omap_set_dma_dest_params(up->uart_dma.rx_dma_channel, 0,
					OMAP_DMA_AMODE_POST_INC,
					up->uart_dma.rx_buf_dma_phys, 0, 0);
		omap_set_dma_transfer_params(up->uart_dma.rx_dma_channel,
					OMAP_DMA_DATA_TYPE_S8,
					up->uart_dma.rx_buf_size, 1,
					OMAP_DMA_SYNC_ELEMENT,
					uart_dma_rx[up->pdev->id-1], 0);
	}
	up->uart_dma.prev_rx_dma_pos = up->uart_dma.rx_buf_dma_phys;
	omap_writel(0, OMAP34XX_DMA4_BASE +
		OMAP_DMA4_CDAC(up->uart_dma.rx_dma_channel));
	omap_start_dma(up->uart_dma.rx_dma_channel);
	mod_timer(&up->uart_dma.rx_timer, jiffies +
			usecs_to_jiffies(up->uart_dma.rx_timeout));
	up->uart_dma.rx_dma_state = 1;
}

static void serial_omap_continue_tx(struct uart_omap_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int start = up->uart_dma.tx_buf_dma_phys + (xmit->tail & (UART_XMIT_SIZE - 1));
	if (uart_circ_empty(xmit))
		return;

	up->uart_dma.tx_buf_size = uart_circ_chars_pending(xmit);
	/* It is a circular buffer. See if the buffer has wounded back.
	* If yes it will have to be transferred in two separate dma
	* transfers
	*/
	if (start + up->uart_dma.tx_buf_size >=
			up->uart_dma.tx_buf_dma_phys + UART_XMIT_SIZE)
		up->uart_dma.tx_buf_size =
			(up->uart_dma.tx_buf_dma_phys + UART_XMIT_SIZE) - start;
	omap_set_dma_dest_params(up->uart_dma.tx_dma_channel, 0,
				 OMAP_DMA_AMODE_CONSTANT,
				 UART_BASE(up->pdev->id - 1), 0, 0);
	omap_set_dma_src_params(up->uart_dma.tx_dma_channel, 0,
				OMAP_DMA_AMODE_POST_INC, start, 0, 0);

	omap_set_dma_transfer_params(up->uart_dma.tx_dma_channel,
				     OMAP_DMA_DATA_TYPE_S8,
				     up->uart_dma.tx_buf_size, 1,
				     OMAP_DMA_SYNC_ELEMENT,
				     uart_dma_tx[(up->pdev->id)-1], 0);

	omap_start_dma(up->uart_dma.tx_dma_channel);
}

static void uart_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct uart_omap_port *up = (struct uart_omap_port *)data;
	struct circ_buf *xmit = &up->port.state->xmit;
	xmit->tail = (xmit->tail + up->uart_dma.tx_buf_size) & (UART_XMIT_SIZE - 1);
	up->port.icount.tx += up->uart_dma.tx_buf_size;

	/* Revisit: Not sure about the below two steps. Seen some instabilities
	* with them. might not be needed in the DMA path
	*/
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit)) {

		spin_lock(&(up->uart_dma.tx_lock));
		serial_omap_stop_tx(&up->port);
		up->uart_dma.tx_dma_state = 0;
		spin_unlock(&(up->uart_dma.tx_lock));
	} else {
		omap_stop_dma(up->uart_dma.tx_dma_channel);
		serial_omap_continue_tx(up);
	}
	isr8250_activity = jiffies;

	return;
}

static int serial_omap_probe(struct platform_device *pdev)
{
	struct plat_serialomap_port *pdata = pdev->dev.platform_data;
	struct uart_omap_port	*up;
	struct resource		*mem, *irq;
	int ret = -ENOSPC;
	char str[7];

	if (!pdata) {
		dev_err(&pdev->dev, "no platform data?\n");
		return -ENODEV;
	}

	if (pdata->disabled) {
		dev_err(&pdev->dev, "device disabled\n");
		return -ENODEV;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	ret = (int) request_mem_region(mem->start, (mem->end - mem->start) + 1,
				     pdev->dev.driver->name);
	if (!ret) {
		dev_err(&pdev->dev, "memory region already claimed\n");
		return -EBUSY;
	}
	up = kzalloc(sizeof(*up), GFP_KERNEL);
	if (up == NULL) {
		ret = -ENOMEM;
		goto do_release_region;
	}
	sprintf(up->name, "OMAP UART%d", pdev->id);

	up->pdev = pdev;
	up->port.dev = &pdev->dev;
	up->port.type = PORT_OMAP;
	up->port.iotype = UPIO_MEM;
	up->port.mapbase = mem->start;
	up->port.irq = irq->start;
	up->port.fifosize = 64;
	up->port.ops = &serial_omap_pops;
	up->port.line = pdev->id - 1;
#define QUART_CLK (1843200)
	if (pdev->id == 4) {
		up->port.membase = ioremap_nocache(mem->start, 0x16 << 1);
		up->port.flags = UPF_BOOT_AUTOCONF | UPF_IOREMAP |
			UPF_SHARE_IRQ | UPF_TRIGGER_HIGH;
		up->port.uartclk = QUART_CLK;
		up->port.regshift = 1;
	} else {
		up->port.membase = (void *) OMAP2_L4_IO_ADDRESS(mem->start);
		up->port.flags = pdata->flags;
		up->port.uartclk = 48000000;
		up->port.regshift = 2;
#ifdef CONFIG_SERIAL_OMAP3430_HW_FLOW_CONTROL
		up->ctsrts = pdata->ctsrts;
#endif
	}


	if (pdev->id == (UART1+1)) {
#ifdef CONFIG_SERIAL_OMAP_DMA_UART1
		up->use_dma = 1;
		up->uart_dma.rx_buf_size =
			CONFIG_SERIAL_OMAP_UART1_RXDMA_BUFSIZE;
		up->uart_dma.rx_timeout =
			CONFIG_SERIAL_OMAP_UART1_RXDMA_TIMEOUT;
#endif
	} else if (pdev->id == (UART2+1)) {
#ifdef CONFIG_SERIAL_OMAP_DMA_UART2
		up->use_dma = 1;
		up->uart_dma.rx_buf_size =
			CONFIG_SERIAL_OMAP_UART2_RXDMA_BUFSIZE;
		up->uart_dma.rx_timeout =
			CONFIG_SERIAL_OMAP_UART2_RXDMA_TIMEOUT;
#endif
	} else if (pdev->id == (UART3+1)) {
#ifdef CONFIG_SERIAL_OMAP_DMA_UART3
		up->use_dma = 1;
		up->uart_dma.rx_buf_size =
			CONFIG_SERIAL_OMAP_UART3_RXDMA_BUFSIZE;
		up->uart_dma.rx_timeout =
			CONFIG_SERIAL_OMAP_UART3_RXDMA_TIMEOUT;
#endif
	}

	if (up->use_dma) {
		spin_lock_init(&(up->uart_dma.tx_lock));
		spin_lock_init(&(up->uart_dma.rx_lock));
		up->uart_dma.tx_dma_channel = 0xFF;
		up->uart_dma.rx_dma_channel = 0xFF;
	}
	if (console_detect(str))
		printk("Invalid console paramter. UART Library Init Failed!\n");
	up->use_console = 0;
	fcr[pdev->id - 1] = 0;
	if (!strcmp(str, "ttyS0"))
		up->use_console = 1;
	else if (!strcmp(str, "ttyS1"))
		up->use_console = 1;
	else if (!strcmp(str, "ttyS2"))
		up->use_console = 1;
	else if (!strcmp(str, "ttyS3"))
		up->use_console = 1;
	else
		printk(KERN_INFO
		       "!!!!!!!! Unable to recongnize Console UART........\n");
	ui[pdev->id - 1] = up;
	serial_omap_add_console_port(up);
	serial_omap_clear_fifos(up);

	ret = uart_add_one_port(&serial_omap_reg, &up->port);
	if (ret != 0)
		goto do_release_region;
	platform_set_drvdata(pdev, up);

	if (pdata->wake_gpio_strobe) {
		if (gpio_request(pdata->wake_gpio_strobe,
				 "UART AP -> BP wakeup strobe")) {
			printk(KERN_ERR "Error requesting GPIO\n");
		} else
			gpio_direction_output(pdata->wake_gpio_strobe, 0);
	}

	INIT_WORK(&up->tty_work, tty_flip_buffer_work);
		
	return 0;
do_release_region:
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return ret;
}

static int serial_omap_remove(struct platform_device *dev)
{
	struct uart_omap_port *up = platform_get_drvdata(dev);

	platform_set_drvdata(dev, NULL);
	if (up) {
		uart_remove_one_port(&serial_omap_reg, &up->port);
		kfree(up);
	}
	return 0;
}

static struct platform_driver serial_omap_driver = {
	.probe          = serial_omap_probe,
	.remove         = serial_omap_remove,

	.suspend	= serial_omap_suspend,
	.resume		= serial_omap_resume,
	.driver		= {
		.name	= "omap-uart",
	},
};

int __init serial_omap_init(void)
{
	int ret;

	wake_lock_init(&omap_serial_wakelock, WAKE_LOCK_SUSPEND,
		       "omap_serial");
	omap_serial_workqueue = create_singlethread_workqueue("omap_serial");
	ret = uart_register_driver(&serial_omap_reg);
	if (ret != 0)
		return ret;
	ret = platform_driver_register(&serial_omap_driver);
	if (ret != 0)
		uart_unregister_driver(&serial_omap_reg);
	return ret;
}

void __exit serial_omap_exit(void)
{
	wake_lock_destroy(&omap_serial_wakelock);
	destroy_workqueue(omap_serial_workqueue);
	platform_driver_unregister(&serial_omap_driver);
	uart_unregister_driver(&serial_omap_reg);
}

#if defined(CONFIG_OMAP3_PM)
int omap24xx_uart_cts_wakeup(int uart_no, int state)
{
	u32 *ptr;
	unsigned char lcr, efr;
	struct uart_omap_port *up = ui[uart_no];

	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		printk(KERN_INFO "Bad uart id %d \n", uart_no);
		return -EPERM;
	}

	if (state) {
		/*
		 * Enable the CTS for io pad wakeup
		 */
		switch (uart_no) {
		case UART1:
			ptr = (u32 *) (&CONTROL_PADCONF_UART1_CTS);
			break;
		case UART2:
			ptr = (u32 *) (&CONTROL_PADCONF_UART2_CTS);
			break;
		default:
			printk(KERN_ERR
			"Wakeup on Uart%d is not supported\n", uart_no);
			return -EPERM;
		}
		*ptr |= (u32)((IO_PAD_WAKEUPENABLE | IO_PAD_OFFPULLUDENABLE |
				IO_PAD_OFFOUTENABLE | IO_PAD_OFFENABLE |
				IO_PAD_INPUTENABLE | IO_PAD_PULLUDENABLE|
				IO_PAD_MUXMODE0)
				);
		/*
		 * Enable the CTS for module level wakeup
		 */
		lcr = serial_in(up, UART_LCR);
		serial_out(up, UART_LCR, 0xbf);
		efr = serial_in(up, UART_EFR);
		serial_out(up, UART_EFR, efr | UART_EFR_ECB);
		serial_out(up, UART_LCR, lcr);
		up->ier	|= (1 << 7);
		serial_out(up, UART_IER, up->ier);
		serial_out(up, UART_OMAP_WER,
				serial_in(up, UART_OMAP_WER) | 0x1);
		serial_out(up, UART_LCR, 0xbf);
		serial_out(up, UART_EFR, efr);
		serial_out(up, UART_LCR, lcr);

	} else {
		/*
		 * Disable the CTS capability for io pad wakeup
		 */
		switch (uart_no) {
		case UART1:
			ptr = (u32 *) (&CONTROL_PADCONF_UART1_CTS);
			break;
		case UART2:
			ptr = (u32 *) (&CONTROL_PADCONF_UART2_CTS);
			break;
		default:
			printk(KERN_ERR
			"Wakeup on Uart%d is not supported\n", uart_no);
			return -EPERM;
		}
		*ptr &= (u32) (~(IO_PAD_WAKEUPENABLE | IO_PAD_OFFPULLUDENABLE |
				IO_PAD_OFFOUTENABLE | IO_PAD_OFFENABLE |
				IO_PAD_INPUTENABLE | IO_PAD_PULLUDENABLE));
		*ptr |= IO_PAD_MUXMODE7;

		/*
		 * Disable the CTS for module level wakeup
		 */
		lcr = serial_in(up, UART_LCR);
		serial_out(up, UART_LCR, 0xbf);
		efr = serial_in(up, UART_EFR);
		serial_out(up, UART_EFR, efr | UART_EFR_ECB);
		serial_out(up, UART_LCR, lcr);
		up->ier	&= ~(1 << 7);
		serial_out(up, UART_IER, up->ier);
		/* TBD:Do we really want to disable module wake up for this in WER*/
		serial_out(up, UART_LCR, 0xbf);
		serial_out(up, UART_EFR, efr);
		serial_out(up, UART_LCR, lcr);
	}
	return 0;
}
EXPORT_SYMBOL(omap24xx_uart_cts_wakeup);
#endif

#ifdef CONFIG_PM
/**
 * are_driver8250_uarts_active() - Check if any ports managed by this
 * driver are currently busy.  This should be called with interrupts
 * disabled.
 */
int are_driveromap_uarts_active(int num)
{
	struct circ_buf *xmit;
	unsigned int status;
	struct uart_omap_port *up = ui[num];

	if (!up)
		return 0;

	/* check ownership of port */
	/* Check only ports managed by this driver and open */
	if ((up->port.dev == NULL) || (up->port.type == PORT_UNKNOWN))
		return 0;

	/* driver owns this port but it's closed */
	if (up->port.state == NULL)
		return 0;

	/* check for any current pending activity */
	/* Any queued work in ring buffer which can be handled still? */
	xmit = &up->port.state->xmit;
	if (!(uart_circ_empty(xmit) || uart_tx_stopped(&up->port)))
		return 1;
	status = serial_in(up, UART_LSR);

	/* TX hardware not empty */
	if (!(status & (UART_LSR_TEMT | UART_LSR_THRE)))
		return 1;

	/* Any rx activity? */
	if (status & UART_LSR_DR)
		return 1;

	if (up->use_dma) {
		/*
		 * Silicon Errata i291 workaround.
		 * UART Module has to be put in force idle if it is
		 * configured in DMA mode and when there is no activity
		 * expected.
		 */
		unsigned int tmp;
		del_timer(&up->uart_dma.rx_timer);
		serial_omap_stop_rxdma(up);
		up->ier |= UART_IER_RDI;
		serial_out(up, UART_IER, up->ier);
		tmp = (serial_in(up, UART_OMAP_SYSC) & 0x7);
		serial_out(up, UART_OMAP_SYSC, tmp); /* force-idle */
	}

	return 0;
}
EXPORT_SYMBOL(are_driveromap_uarts_active);

#endif

static void serial_omap_display_reg(struct uart_port *port)
{
	struct uart_omap_port *up = (struct uart_omap_port *)port;
	unsigned int lcr, efr, mcr, dll, dlh, xon1, xon2, xoff1, xoff2;
	unsigned int tcr, tlr, uasr;
	DPRINTK("Register dump for UART%d\n", up->pdev->id);
	DPRINTK("IER_REG=0x%x\n", serial_in(up, UART_IER));
	DPRINTK("IIR_REG=0x%x\n", serial_in(up, UART_IIR));
	lcr = serial_in(up, UART_LCR);
	DPRINTK("LCR_REG=0x%x\n", lcr);
	mcr = serial_in(up, UART_MCR);
	DPRINTK("MCR_REG=0x%x\n", mcr);
	DPRINTK("LSR_REG=0x%x\n", serial_in(up, UART_LSR));
	DPRINTK("MSR_REG=0x%x\n", serial_in(up, UART_MSR));
	DPRINTK("SPR_REG=0x%x\n", serial_in(up, UART_OMAP_SPR));
	DPRINTK("MDR1_REG=0x%x\n", serial_in(up, UART_OMAP_MDR1));
	DPRINTK("MDR2_REG=0x%x\n", serial_in(up, UART_OMAP_MDR2));
	DPRINTK("SCR_REG=0x%x\n", serial_in(up, UART_OMAP_SCR));
	DPRINTK("SSR_REG=0x%x\n", serial_in(up, UART_OMAP_SSR));
	DPRINTK("MVR_REG=0x%x\n", serial_in(up, UART_OMAP_MVER));
	DPRINTK("SYSC_REG=0x%x\n", serial_in(up, UART_OMAP_SYSC));
	DPRINTK("SYSS_REG=0x%x\n", serial_in(up, UART_OMAP_SYSS));
	DPRINTK("WER_REG=0x%x\n", serial_in(up, UART_OMAP_WER));

	serial_out(up, UART_LCR, 0xBF);
	dll = serial_in(up, UART_DLL);
	dlh = serial_in(up, UART_DLM);
	efr = serial_in(up, UART_EFR);
	xon1 = serial_in(up, UART_XON1);
	xon2 = serial_in(up, UART_XON2);

	serial_out(up, UART_EFR, efr | UART_EFR_ECB);
	serial_out(up, UART_LCR, lcr);
	serial_out(up, UART_MCR, mcr | UART_MCR_TCRTLR);
	serial_out(up, UART_LCR, 0xBF);

	tcr = serial_in(up, UART_TI752_TCR);
	tlr = serial_in(up, UART_TI752_TLR);

	serial_out(up, UART_LCR, lcr);
	serial_out(up, UART_MCR, mcr);
	serial_out(up, UART_LCR, 0xBF);

	xoff1 = serial_in(up, UART_XOFF1);
	xoff2 = serial_in(up, UART_XOFF2);
	uasr = serial_in(up, UART_OMAP_UASR);

	serial_out(up, UART_EFR, efr);
	serial_out(up, UART_LCR, lcr);


	DPRINTK("DLL_REG=0x%x\n", dll);
	DPRINTK("DLH_REG=0x%x\n", dlh);
	DPRINTK("EFR_REG=0x%x\n", efr);

	DPRINTK("XON1_ADDR_REG=0x%x\n", xon1);
	DPRINTK("XON2_ADDR_REG=0x%x\n", xon2);
	DPRINTK("TCR_REG=0x%x\n", tcr);
	DPRINTK("TLR_REG=0x%x\n", tlr);


	DPRINTK("XOFF1_REG=0x%x\n", xoff1);
	DPRINTK("XOFF2_REG=0x%x\n", xoff2);
	DPRINTK("UASR_REG=0x%x\n", uasr);

}

subsys_initcall(serial_omap_init);
module_exit(serial_omap_exit);

MODULE_LICENSE("GPL");
