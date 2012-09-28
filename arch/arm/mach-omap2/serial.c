/*
 * arch/arm/mach-omap2/serial.c
 *
 * OMAP2 serial support.
 *
 * Copyright (C) 2005-2008 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * Major rework for PM support by Kevin Hilman
 *
 * Based off of arch/arm/mach-omap/omap1/serial.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/serial_reg.h>
#include <linux/clk.h>
#ifdef CONFIG_SERIAL_OMAP
#include <linux/platform_device.h>
#endif
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/debugfs.h>

#include <plat/common.h>
#include <plat/board.h>
#include <plat/clock.h>
#include <plat/control.h>
#include <mach/gpio.h>
#include <plat/omap-serial.h>

#include <asm/mach/serial_omap.h>

#include "prm.h"
#include "pm.h"
#include "prm-regbits-34xx.h"

#define DEFAULT_TIMEOUT HZ

struct omap_uart_state {
	int num;
	int can_sleep;
	struct timer_list timer;
	u32 timeout;

	void __iomem *wk_st;
	void __iomem *wk_en;
	u32 wk_mask;
	u32 padconf;

	u32 rts_padconf;
	int rts_override;
	u16 rts_padvalue;

	struct clk *ick;
	struct clk *fck;
	int clocked;

        struct plat_serialomap_port *p;
	struct list_head node;

#if defined(CONFIG_ARCH_OMAP3) && defined(CONFIG_PM)
	int context_valid;

	/* Registers to be saved/restored for OFF-mode */
	u16 dll;
	u16 dlh;
	u16 ier;
	u16 sysc;
	u16 scr;
	u16 wer;
#endif
};

static struct omap_uart_state omap_uart[OMAP_MAX_NR_PORTS];
static LIST_HEAD(uart_list);
static unsigned int fifo_idleblks = 0;
static int uart0_padconf = 0x180;

#ifdef CONFIG_SERIAL_OMAP
static struct plat_serialomap_port serial_platform_data[] = {
	{
		.membase	= OMAP2_L4_IO_ADDRESS(OMAP_UART1_BASE),
		.irq		= 72,
		.regshift	= 2,
#ifdef CONFIG_SERIAL_OMAP3430_HW_FLOW_CONTROL
		.ctsrts		= UART_EFR_CTS | UART_EFR_RTS,
#endif
		.flags		= UPF_BOOT_AUTOCONF,
	},
	{
		.membase	= OMAP2_L4_IO_ADDRESS(OMAP_UART2_BASE),
		.irq		= 73,
		.regshift	= 2,
#ifdef CONFIG_SERIAL_OMAP3430_HW_FLOW_CONTROL
		.ctsrts		= UART_EFR_CTS | UART_EFR_RTS,
#endif
		.flags		= UPF_BOOT_AUTOCONF,
	},
	{
		.membase	= OMAP2_L4_IO_ADDRESS(OMAP_UART3_BASE),
		.irq		= 74,
		.regshift	= 2,
#ifdef CONFIG_SERIAL_OMAP3430_HW_FLOW_CONTROL
		.ctsrts		= UART_EFR_RTS,
#endif
		.flags		= UPF_BOOT_AUTOCONF,
	},
};

static struct resource omap2_uart1_resources[] = {
	{
		.start		= OMAP_UART1_BASE,
		.end		= OMAP_UART1_BASE + 0x3ff,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= 72,
                .flags		= IORESOURCE_IRQ,
	}
};

static struct resource omap2_uart2_resources[] = {
	{
		.start		= OMAP_UART2_BASE,
		.end		= OMAP_UART2_BASE + 0x3ff,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= 73,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct resource omap2_uart3_resources[] = {
	{
		.start		= OMAP_UART3_BASE,
		.end		= OMAP_UART3_BASE + 0x3ff,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= 74,
		.flags		= IORESOURCE_IRQ,
	}
};

#ifdef CONFIG_MACH_OMAP_ZOOM2
static struct resource omap2_quaduart_resources[] = {
	{
		.start		= 0x10000000,
		.end		= 0x10000000 + (0x16 << 1),
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= OMAP_GPIO_IRQ(102),
		.flags		= IORESOURCE_IRQ,
	}
};
#endif

/* OMAP UART platform structure */
static struct platform_device uart1_device = {
	.name			= "omap-uart",
	.id			= 1,
	.num_resources		= ARRAY_SIZE(omap2_uart1_resources),
	.resource		= omap2_uart1_resources,
	.dev.platform_data	= &serial_platform_data[0],
};
static struct platform_device uart2_device = {
	.name			= "omap-uart",
	.id			= 2,
	.num_resources		= ARRAY_SIZE(omap2_uart2_resources),
	.resource		= omap2_uart2_resources,
	.dev.platform_data	= &serial_platform_data[1],
};
static struct platform_device uart3_device = {
	.name			= "omap-uart",
	.id			= 3,
	.num_resources		= ARRAY_SIZE(omap2_uart3_resources),
	.resource		= omap2_uart3_resources,
	.dev.platform_data	= &serial_platform_data[2],
};

#ifdef CONFIG_MACH_OMAP_ZOOM2
static struct platform_device quaduart_device = {

	.name			= "omap-uart",
	.id			= 4,
	.num_resources		= ARRAY_SIZE(omap2_quaduart_resources),
	.resource		= omap2_quaduart_resources,
};
#endif

static struct platform_device *uart_devices[] = {
	&uart1_device,
	&uart2_device,
	&uart3_device,
#ifdef CONFIG_MACH_OMAP_ZOOM2
	&quaduart_device
#endif
};
#endif

static inline unsigned int serial_read_reg(struct plat_serialomap_port *up,
					   int offset)
{
	offset <<= up->regshift;
	return (unsigned int)__raw_readb(up->membase + offset);
}

static inline void serial_write_reg(struct plat_serialomap_port *p, int offset,
				    int value)
{
	offset <<= p->regshift;
	__raw_writeb(value, p->membase + offset);
}

/*
 * Internal UARTs need to be initialized for the 8250 autoconfig to work
 * properly. Note that the TX watermark initialization may not be needed
 * once the 8250.c watermark handling code is merged.
 */
static inline void __init omap_uart_reset(struct omap_uart_state *uart)
{
	struct plat_serialomap_port *p = uart->p;

	serial_write_reg(p, UART_OMAP_MDR1, 0x07);
	serial_write_reg(p, UART_OMAP_SCR, 0x08);
	serial_write_reg(p, UART_OMAP_MDR1, 0x00);
	serial_write_reg(p, UART_OMAP_SYSC, (0x02 << 3) | (1 << 2) | (1 << 0));
}

static inline void omap_uart_enable_clocks(struct omap_uart_state *uart)
{
	if (uart->clocked)
		return;

	clk_enable(uart->ick);
	clk_enable(uart->fck);
	uart->clocked = 1;
}

#ifdef CONFIG_PM
#ifdef CONFIG_ARCH_OMAP3

static void omap_uart_save_context(struct omap_uart_state *uart)
{
	u16 lcr = 0;
	struct plat_serialomap_port *p = uart->p;

	if (!enable_off_mode)
		return;

	/* FIXME
	 * For omap3430 CORE/PERR OFF isn't temporarily supported,
	 * so no need to save&restore the context of serial.
	 */
	if (cpu_is_omap34xx())
		return;

	lcr = serial_read_reg(p, UART_LCR);
	serial_write_reg(p, UART_LCR, 0xBF);
	uart->dll = serial_read_reg(p, UART_DLL);
	uart->dlh = serial_read_reg(p, UART_DLM);
	serial_write_reg(p, UART_LCR, lcr);
	uart->ier = serial_read_reg(p, UART_IER);
	uart->sysc = serial_read_reg(p, UART_OMAP_SYSC);
	uart->scr = serial_read_reg(p, UART_OMAP_SCR);
	uart->wer = serial_read_reg(p, UART_OMAP_WER);

	uart->context_valid = 1;
}

static void omap_uart_restore_context(struct omap_uart_state *uart)
{
	u16 efr = 0;
	struct plat_serialomap_port *p = uart->p;

	if (!enable_off_mode)
		return;

	if (!uart->context_valid)
		return;

	/* FIXME
	 * For omap3430 CORE/PERR OFF isn't temporarily supported,
	 * so no need to save&restore the context of serial.
	 */
	if (cpu_is_omap34xx())
		return;

	uart->context_valid = 0;

	serial_write_reg(p, UART_OMAP_MDR1, 0x7);
	serial_write_reg(p, UART_LCR, 0xBF); /* Config B mode */
	efr = serial_read_reg(p, UART_EFR);
	serial_write_reg(p, UART_EFR, UART_EFR_ECB);
	serial_write_reg(p, UART_LCR, 0x0); /* Operational mode */
	serial_write_reg(p, UART_IER, 0x0);
#ifdef CONFIG_SERIAL_OMAP
	serial_write_reg(p, UART_FCR, fcr[uart->num]);
#else
	serial_write_reg(p, UART_FCR, 0xA1);
#endif
	serial_write_reg(p, UART_LCR, 0xBF); /* Config B mode */
	serial_write_reg(p, UART_DLL, uart->dll);
	serial_write_reg(p, UART_DLM, uart->dlh);
	serial_write_reg(p, UART_LCR, 0x0); /* Operational mode */
	serial_write_reg(p, UART_IER, uart->ier);
	serial_write_reg(p, UART_LCR, 0xBF); /* Config B mode */
	serial_write_reg(p, UART_EFR, efr);
	serial_write_reg(p, UART_LCR, UART_LCR_WLEN8);
	serial_write_reg(p, UART_OMAP_SCR, uart->scr);
	serial_write_reg(p, UART_OMAP_WER, uart->wer);
	serial_write_reg(p, UART_OMAP_SYSC, uart->sysc);
	serial_write_reg(p, UART_OMAP_MDR1, 0x00); /* UART 16x mode */
}
#else
static inline void omap_uart_save_context(struct omap_uart_state *uart) {}
static inline void omap_uart_restore_context(struct omap_uart_state *uart) {}
#endif /* CONFIG_ARCH_OMAP3 */

static void omap_uart_smart_idle_enable(struct omap_uart_state *uart,
					  int enable)
{
	struct plat_serialomap_port *p = uart->p;
	u16 sysc;

	sysc = serial_read_reg(p, UART_OMAP_SYSC) & 0x7;
	if (enable)
		sysc |= 0x2 << 3;
	else
		sysc |= 0x1 << 3;

	serial_write_reg(p, UART_OMAP_SYSC, sysc);
}

static inline void omap_uart_disable_rtspullup(struct omap_uart_state *uart)
{
	if (!uart->rts_padconf || !uart->rts_override)
		return;
	omap_ctrl_writew(uart->rts_padvalue, uart->rts_padconf);
	uart->rts_override = 0;
}

static inline void omap_uart_enable_rtspullup(struct omap_uart_state *uart)
{
	if (!uart->rts_padconf || uart->rts_override)
		return;

	uart->rts_padvalue = omap_ctrl_readw(uart->rts_padconf);
	omap_ctrl_writew(0x18 | 0x7, uart->rts_padconf);
	uart->rts_override = 1;
}

static inline void omap_uart_restore(struct omap_uart_state *uart)
{
	omap_uart_enable_clocks(uart);
	omap_uart_restore_context(uart);
}

static inline void omap_uart_disable_clocks(struct omap_uart_state *uart)
{
	if (!uart->clocked)
		return;
	omap_uart_save_context(uart);
	uart->clocked = 0;
	clk_disable(uart->ick);
	clk_disable(uart->fck);
}

static void _omap_uart_block_sleep(struct omap_uart_state *uart)
{
	omap_uart_restore(uart);

	omap_uart_smart_idle_enable(uart, 0);
	uart->can_sleep = 0;
	if (uart->timeout)
		mod_timer(&uart->timer, jiffies + uart->timeout);
	else
		del_timer(&uart->timer);
}

void omap_uart_block_sleep(int num)
{
	struct omap_uart_state *uart;

	list_for_each_entry(uart, &uart_list, node) {
		if (num == uart->num)
			_omap_uart_block_sleep(uart);
		return;
	}
}
EXPORT_SYMBOL(omap_uart_block_sleep);

static void omap_uart_allow_sleep(struct omap_uart_state *uart)
{
	if (!uart->clocked)
		return;

	omap_uart_smart_idle_enable(uart, 1);
	uart->can_sleep = 1;
	del_timer(&uart->timer);
}

static void omap_uart_idle_timer(unsigned long data)
{
	struct omap_uart_state *uart = (struct omap_uart_state *)data;

	omap_uart_allow_sleep(uart);
}

void omap_uart_prepare_idle(int num)
{
	struct omap_uart_state *uart;

	list_for_each_entry(uart, &uart_list, node) {
		if (num == uart->num && uart->can_sleep) {

			omap_uart_enable_rtspullup(uart);
			/*
			 * There seems to be a window here where
			 * data could still be on the way to the
			 * fifo. This delay is ~1 byte time @ 115.2k
			 */
			if (uart->num == 0)
				udelay(80);

#ifdef CONFIG_SERIAL_OMAP
			if (are_driveromap_uarts_active(num)) {
				fifo_idleblks++;
				_omap_uart_block_sleep(uart);
				omap_uart_disable_rtspullup(uart);
				return;
			}
#endif
			omap_uart_disable_clocks(uart);
			return;
		}
	}
}

void omap_uart_resume_idle(int num)
{
	struct omap_uart_state *uart;

	list_for_each_entry(uart, &uart_list, node) {
		if (num == uart->num) {
			omap_uart_restore(uart);
			omap_uart_disable_rtspullup(uart);

			/* Check for IO pad wakeup */
			if (cpu_is_omap34xx() && uart->padconf) {
				u16 p = omap_ctrl_readw(uart->padconf);

				if (p & OMAP3_PADCONF_WAKEUPEVENT0)
					_omap_uart_block_sleep(uart);
			}

			/* Check for normal UART wakeup */
			if (__raw_readl(uart->wk_st) & uart->wk_mask) {
				_omap_uart_block_sleep(uart);
			}

			return;
		}
	}
}

void omap_uart_prepare_suspend(void)
{
	struct omap_uart_state *uart;

	list_for_each_entry(uart, &uart_list, node) {
		omap_uart_allow_sleep(uart);
	}
}

int omap_uart_can_sleep(void)
{
	struct omap_uart_state *uart;
	int can_sleep = 1;

	list_for_each_entry(uart, &uart_list, node) {
		if (!uart->clocked)
			continue;

		if (!uart->can_sleep) {
			can_sleep = 0;
			continue;
		}

#ifdef CONFIG_SERIAL_OMAP
		if (are_driveromap_uarts_active(uart->num)) {
			can_sleep = 0;
			continue;
		}
#endif

		/* This UART can now safely sleep. */
		omap_uart_allow_sleep(uart);
	}

	return can_sleep;
}

/**
 * omap_uart_interrupt()
 *
 * This handler is used only to detect that *any* UART interrupt has
 * occurred.  It does _nothing_ to handle the interrupt.  Rather,
 * any UART interrupt will trigger the inactivity timer so the
 * UART will not idle or sleep for its timeout period.
 *
 **/
static irqreturn_t omap_uart_interrupt(int irq, void *dev_id)
{
	struct omap_uart_state *uart = dev_id;

	_omap_uart_block_sleep(uart);

	return IRQ_NONE;
}

static u32 sleep_timeout = DEFAULT_TIMEOUT;

static void omap_uart_rtspad_init(struct omap_uart_state *uart)
{
	if (!cpu_is_omap34xx())
		return;
	switch(uart->num) {
	case 0:
		uart->rts_padconf = 0x17e;
		break;
	case 1:
		uart->rts_padconf = 0x176;
		break;
	case 2:
/*		uart->rts_padconf = 0x19c; */
		break;
	default:
		uart->rts_padconf = 0;
		break;
	}
}

static void omap_uart_idle_init(struct omap_uart_state *uart)
{
	u32 v;
	struct plat_serialomap_port *p = uart->p;
	int ret;

	uart->can_sleep = 0;
	uart->timeout = sleep_timeout;
	if (!uart->timeout)
		_omap_uart_block_sleep(uart);
	else {
		setup_timer(&uart->timer, omap_uart_idle_timer,
			    (unsigned long) uart);
		mod_timer(&uart->timer, jiffies + uart->timeout);
		omap_uart_smart_idle_enable(uart, 0);
	}

	if (cpu_is_omap34xx()) {
		u32 mod = (uart->num == 2) ? OMAP3430_PER_MOD : CORE_MOD;
		u32 wk_mask = 0;
		u32 padconf = 0;

		uart->wk_en = OMAP34XX_PRM_REGADDR(mod, PM_WKEN1);
		uart->wk_st = OMAP34XX_PRM_REGADDR(mod, PM_WKST1);
		switch (uart->num) {
		case 0:
			wk_mask = OMAP3430_ST_UART1_MASK;
			padconf = uart0_padconf;
			break;
		case 1:
			wk_mask = OMAP3430_ST_UART2_MASK;
			padconf = 0x17a;
			break;
		case 2:
			wk_mask = OMAP3430_ST_UART3_MASK;
			padconf = 0x19e;
			break;
		}
		uart->wk_mask = wk_mask;
		uart->padconf = padconf;
	} else if (cpu_is_omap24xx()) {
		u32 wk_mask = 0;

		if (cpu_is_omap2430()) {
			uart->wk_en = OMAP2430_PRM_REGADDR(CORE_MOD, PM_WKEN1);
			uart->wk_st = OMAP2430_PRM_REGADDR(CORE_MOD, PM_WKST1);
		} else if (cpu_is_omap2420()) {
			uart->wk_en = OMAP2420_PRM_REGADDR(CORE_MOD, PM_WKEN1);
			uart->wk_st = OMAP2420_PRM_REGADDR(CORE_MOD, PM_WKST1);
		}
		switch (uart->num) {
		case 0:
			wk_mask = OMAP24XX_ST_UART1_MASK;
			break;
		case 1:
			wk_mask = OMAP24XX_ST_UART2_MASK;
			break;
		case 2:
			wk_mask = OMAP24XX_ST_UART3_MASK;
			break;
		}
		uart->wk_mask = wk_mask;
	} else {
		uart->wk_en = 0;
		uart->wk_st = 0;
		uart->wk_mask = 0;
		uart->padconf = 0;
	}

	/* Set wake-enable bit */
	if (uart->wk_en && uart->wk_mask) {
		v = __raw_readl(uart->wk_en);
		v |= uart->wk_mask;
		__raw_writel(v, uart->wk_en);
	}

	/* Ensure IOPAD wake-enables are set */
	if (cpu_is_omap34xx() && uart->padconf) {
		u16 v;

		v = omap_ctrl_readw(uart->padconf);
		v |= OMAP3_PADCONF_WAKEUPENABLE0;
		omap_ctrl_writew(v, uart->padconf);
	}

	p->flags |= UPF_SHARE_IRQ;
	ret = request_irq(p->irq, omap_uart_interrupt, IRQF_SHARED,
			  "serial idle", (void *)uart);
	WARN_ON(ret);
}

void omap_uart_enable_irqs(int enable)
{
	int ret;
	struct omap_uart_state *uart;

	list_for_each_entry(uart, &uart_list, node) {
		if (enable)
			ret = request_irq(uart->p->irq, omap_uart_interrupt,
				IRQF_SHARED, "serial idle", (void *)uart);
		else
			free_irq(uart->p->irq, (void *)uart);
	}
}

static ssize_t sleep_timeout_show(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	return sprintf(buf, "%u\n", sleep_timeout / HZ);
}

static ssize_t sleep_timeout_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t n)
{
	struct omap_uart_state *uart;
	unsigned int value;

	if (sscanf(buf, "%u", &value) != 1) {
		printk(KERN_ERR "sleep_timeout_store: Invalid value\n");
		return -EINVAL;
	}
	sleep_timeout = value * HZ;
	list_for_each_entry(uart, &uart_list, node) {
		uart->timeout = sleep_timeout;
		if (uart->timeout)
			mod_timer(&uart->timer, jiffies + uart->timeout);
		else
			/* A zero value means disable timeout feature */
			_omap_uart_block_sleep(uart);
	}
	return n;
}

static struct kobj_attribute sleep_timeout_attr =
	__ATTR(sleep_timeout, 0644, sleep_timeout_show, sleep_timeout_store);

#else
static inline void omap_uart_idle_init(struct omap_uart_state *uart) {}
#endif /* CONFIG_PM */

static int fifo_idleblk_get(void *data, u64 *val)
{
	*val = fifo_idleblks;
	return 0;
}

static int fifo_idleblk_set(void *data, u64 val)
{
	fifo_idleblks = 0;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fifo_idleblk_fops, fifo_idleblk_get, fifo_idleblk_set, "%llu\n");
void __init omap_serial_early_init(void)
{
}

void __init omap_serial_ctsrts_init(unsigned char ctsrts[])
{
#if defined(CONFIG_SERIAL_OMAP) && \
	defined(CONFIG_SERIAL_OMAP3430_HW_FLOW_CONTROL)
	serial_platform_data[0].ctsrts = ctsrts[0];
	serial_platform_data[1].ctsrts = ctsrts[1];
	serial_platform_data[2].ctsrts = ctsrts[2];
#endif
}

void __init omap_uart_set_uart0_padconf(int padconf)
{
	uart0_padconf = padconf;
}

void __init omap_serial_init(int wake_gpio_strobe,
			     unsigned int wake_strobe_enable_mask)
{
	int i;
	const struct omap_uart_config *info;
	char name[16];

	debugfs_create_file("fifo_idle_block_count", 0644, NULL, NULL, &fifo_idleblk_fops);
	/*
	 * Make sure the serial ports are muxed on at this point.
	 * You have to mux them off in device drivers later on
	 * if not needed.
	 */

	info = omap_get_config(OMAP_TAG_UART, struct omap_uart_config);

	if (info == NULL)
		return;

	for (i = 0; i < OMAP_MAX_NR_PORTS; i++) {
		struct plat_serialomap_port *p = serial_platform_data + i;
		struct omap_uart_state *uart = &omap_uart[i];

		if (!(info->enabled_uarts & (1 << i))) {
			p->disabled = 1;
			continue;
		}

		if (wake_strobe_enable_mask & (1 << i))
			p->wake_gpio_strobe = wake_gpio_strobe;

		sprintf(name, "uart%d_ick", i+1);
		uart->ick = clk_get(NULL, name);
		if (IS_ERR(uart->ick)) {
			printk(KERN_ERR "Could not get uart%d_ick\n", i+1);
			uart->ick = NULL;
		}

		sprintf(name, "uart%d_fck", i+1);
		uart->fck = clk_get(NULL, name);
		if (IS_ERR(uart->fck)) {
			printk(KERN_ERR "Could not get uart%d_fck\n", i+1);
			uart->fck = NULL;
		}

		if (!uart->ick || !uart->fck)
			continue;

		uart->num = i;
		p->private_data = uart;
		uart->p = p;
		list_add(&uart->node, &uart_list);

		omap_uart_enable_clocks(uart);
		omap_uart_reset(uart);
		omap_uart_rtspad_init(uart);
		omap_uart_idle_init(uart);
	}
}

#ifdef CONFIG_SERIAL_OMAP
static int __init omap_hs_init(void)
{
	int ret = 0;

	ret = platform_add_devices(uart_devices, ARRAY_SIZE(uart_devices));
	if (ret) {
		printk(KERN_ERR "Error adding uart devices (%d)\n", ret);
		return ret;
	}
	ret = sysfs_create_file(&uart1_device.dev.kobj,
				&sleep_timeout_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Error creating uart sleep_timeout sysfs file (%d)\n",
			ret);
		return ret;
	}
	return ret;
}
arch_initcall(omap_hs_init);
#endif
