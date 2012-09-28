/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2005 David Brownell
 * (C) Copyright 2002 Hewlett-Packard Company
 *
 * OMAP Bus Glue
 *
 * Modified for OMAP by Tony Lindgren <tony@atomide.com>
 * Based on the 2.4 OMAP OHCI driver originally done by MontaVista Software Inc.
 * and on ohci-sa1111.c by Christopher Hoover <ch@hpl.hp.com>
 *
 * This file is licenced under the GPL.
 */

#include <linux/signal.h>	/* IRQF_DISABLED */
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio.h>

#include <plat/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <plat/mux.h>
#include <plat/irqs.h>
#include <plat/gpio.h>
#include <plat/fpga.h>
#include <plat/usb.h>

/* OMAP-1510 OHCI has its own MMU for DMA */
#define OMAP1510_LB_MEMSIZE	32	/* Should be same as SDRAM size */
#define OMAP1510_LB_CLOCK_DIV	0xfffec10c
#define OMAP1510_LB_MMU_CTL	0xfffec208
#define OMAP1510_LB_MMU_LCK	0xfffec224
#define OMAP1510_LB_MMU_LD_TLB	0xfffec228
#define OMAP1510_LB_MMU_CAM_H	0xfffec22c
#define OMAP1510_LB_MMU_CAM_L	0xfffec230
#define OMAP1510_LB_MMU_RAM_H	0xfffec234
#define OMAP1510_LB_MMU_RAM_L	0xfffec238


#ifndef CONFIG_ARCH_OMAP
#error "This file is OMAP bus glue.  CONFIG_OMAP must be defined."
#endif

#ifdef CONFIG_TPS65010
#include <linux/i2c/tps65010.h>
#else

#define LOW	0
#define HIGH	1

#define GPIO1	1

static inline int tps65010_set_gpio_out_value(unsigned gpio, unsigned value)
{
	return 0;
}

#endif

extern int usb_disabled(void);
extern int ocpi_enable(void);

#ifndef CONFIG_ARCH_OMAP34XX
static struct clk *usb_host_ck;
static struct clk *usb_dc_ck;
#endif

static int host_enabled;
static int host_initialized;

#ifdef CONFIG_MACH_MAPPHONE
static int ohci_irq_num;
#endif
static void omap_ohci_clock_power(int on)
{
#ifndef CONFIG_ARCH_OMAP34XX
	if (on) {
		clk_enable(usb_dc_ck);
		clk_enable(usb_host_ck);
		/* guesstimate for T5 == 1x 32K clock + APLL lock time */
		udelay(100);
	} else {
		clk_disable(usb_host_ck);
		clk_disable(usb_dc_ck);
	}
#endif
}

/*
 * Board specific gang-switched transceiver power on/off.
 * NOTE:  OSK supplies power from DC, not battery.
 */
static int omap_ohci_transceiver_power(int on)
{
	if (on) {
		if (machine_is_omap_innovator() && cpu_is_omap1510())
			fpga_write(fpga_read(INNOVATOR_FPGA_CAM_USB_CONTROL)
				| ((1 << 5/*usb1*/) | (1 << 3/*usb2*/)),
			       INNOVATOR_FPGA_CAM_USB_CONTROL);
		else if (machine_is_omap_osk())
			tps65010_set_gpio_out_value(GPIO1, LOW);
	} else {
		if (machine_is_omap_innovator() && cpu_is_omap1510())
			fpga_write(fpga_read(INNOVATOR_FPGA_CAM_USB_CONTROL)
				& ~((1 << 5/*usb1*/) | (1 << 3/*usb2*/)),
			       INNOVATOR_FPGA_CAM_USB_CONTROL);
		else if (machine_is_omap_osk())
			tps65010_set_gpio_out_value(GPIO1, HIGH);
	}

	return 0;
}

#ifdef CONFIG_ARCH_OMAP15XX
/*
 * OMAP-1510 specific Local Bus clock on/off
 */
static int omap_1510_local_bus_power(int on)
{
	if (on) {
		omap_writel((1 << 1) | (1 << 0), OMAP1510_LB_MMU_CTL);
		udelay(200);
	} else {
		omap_writel(0, OMAP1510_LB_MMU_CTL);
	}

	return 0;
}

/*
 * OMAP-1510 specific Local Bus initialization
 * NOTE: This assumes 32MB memory size in OMAP1510LB_MEMSIZE.
 *       See also arch/mach-omap/memory.h for __virt_to_dma() and
 *       __dma_to_virt() which need to match with the physical
 *       Local Bus address below.
 */
static int omap_1510_local_bus_init(void)
{
	unsigned int tlb;
	unsigned long lbaddr, physaddr;

	omap_writel((omap_readl(OMAP1510_LB_CLOCK_DIV) & 0xfffffff8) | 0x4,
	       OMAP1510_LB_CLOCK_DIV);

	/* Configure the Local Bus MMU table */
	for (tlb = 0; tlb < OMAP1510_LB_MEMSIZE; tlb++) {
		lbaddr = tlb * 0x00100000 + OMAP1510_LB_OFFSET;
		physaddr = tlb * 0x00100000 + PHYS_OFFSET;
		omap_writel((lbaddr & 0x0fffffff) >> 22, OMAP1510_LB_MMU_CAM_H);
		omap_writel(((lbaddr & 0x003ffc00) >> 6) | 0xc,
		       OMAP1510_LB_MMU_CAM_L);
		omap_writel(physaddr >> 16, OMAP1510_LB_MMU_RAM_H);
		omap_writel((physaddr & 0x0000fc00) | 0x300, OMAP1510_LB_MMU_RAM_L);
		omap_writel(tlb << 4, OMAP1510_LB_MMU_LCK);
		omap_writel(0x1, OMAP1510_LB_MMU_LD_TLB);
	}

	/* Enable the walking table */
	omap_writel(omap_readl(OMAP1510_LB_MMU_CTL) | (1 << 3), OMAP1510_LB_MMU_CTL);
	udelay(200);

	return 0;
}
#else
#define omap_1510_local_bus_power(x)	{}
#define omap_1510_local_bus_init()	{}
#endif

#ifdef	CONFIG_USB_OTG

static void start_hnp(struct ohci_hcd *ohci)
{
	const unsigned	port = ohci_to_hcd(ohci)->self.otg_port - 1;
	unsigned long	flags;
	u32 l;

	otg_start_hnp(ohci->transceiver);

	local_irq_save(flags);
	ohci->transceiver->state = OTG_STATE_A_SUSPEND;
	writel (RH_PS_PSS, &ohci->regs->roothub.portstatus [port]);
	l = omap_readl(OTG_CTRL);
	l &= ~OTG_A_BUSREQ;
	omap_writel(l, OTG_CTRL);
	local_irq_restore(flags);
}

#endif

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_MACH_MAPPHONE
/* TLL Register Set */
#define	OMAP_USBTLL_REVISION				(0x00)
#define	OMAP_USBTLL_SYSCONFIG				(0x10)
#define	OMAP_USBTLL_SYSCONFIG_CACTIVITY			(1 << 8)
#define	OMAP_USBTLL_SYSCONFIG_SIDLEMODE			(2 << 3)
#define	OMAP_USBTLL_SYSCONFIG_ENAWAKEUP			(1 << 2)
#define	OMAP_USBTLL_SYSCONFIG_SOFTRESET			(1 << 1)
#define	OMAP_USBTLL_SYSCONFIG_AUTOIDLE			(1 << 0)

#define	OMAP_USBTLL_SYSSTATUS				(0x14)
#define	OMAP_USBTLL_SYSSTATUS_RESETDONE			(1 << 0)

#define	OMAP_USBTLL_IRQSTATUS				(0x18)
#define	OMAP_USBTLL_IRQENABLE				(0x1C)

#define	OMAP_TLL_SHARED_CONF				(0x30)
#define	OMAP_TLL_SHARED_CONF_USB_90D_DDR_EN		(1 << 6)
#define	OMAP_TLL_SHARED_CONF_USB_180D_SDR_EN		(1 << 5)
#define	OMAP_TLL_SHARED_CONF_USB_DIVRATION		(1 << 2)
#define	OMAP_TLL_SHARED_CONF_FCLK_REQ			(1 << 1)
#define	OMAP_TLL_SHARED_CONF_FCLK_IS_ON			(1 << 0)

#define	OMAP_TLL_CHANNEL_CONF(num)			(0x040 + 0x004 * num)
#define	OMAP_TLL_CHANNEL_CONF_FSLSMODE(x)               (((x)&0xf)<<24)
#define	OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF		(1 << 11)
#define	OMAP_TLL_CHANNEL_CONF_ULPI_ULPIAUTOIDLE		(1 << 10)
#define	OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE		(1 << 9)
#define	OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE		(1 << 8)
#define	OMAP_TLL_CHANNEL_CONF_CHANMODE(x)		(((x)&0x3) << 1)
#define	OMAP_TLL_CHANNEL_CONF_CHANEN			(1 << 0)

#define	OMAP_TLL_ULPI_FUNCTION_CTRL(num)		(0x804 + 0x100 * num)
#define	OMAP_TLL_ULPI_INTERFACE_CTRL(num)		(0x807 + 0x100 * num)
#define	OMAP_TLL_ULPI_OTG_CTRL(num)			(0x80A + 0x100 * num)
#define	OMAP_TLL_ULPI_INT_EN_RISE(num)			(0x80D + 0x100 * num)
#define	OMAP_TLL_ULPI_INT_EN_FALL(num)			(0x810 + 0x100 * num)
#define	OMAP_TLL_ULPI_INT_STATUS(num)			(0x813 + 0x100 * num)
#define	OMAP_TLL_ULPI_INT_LATCH(num)			(0x814 + 0x100 * num)
#define	OMAP_TLL_ULPI_DEBUG(num)			(0x815 + 0x100 * num)
#define	OMAP_TLL_ULPI_SCRATCH_REGISTER(num)		(0x816 + 0x100 * num)

#define OMAP_TLL_CHANNEL_COUNT				3
#define OMAP_TLL_CHANNEL_1_EN_MASK			(1 << 1)
#define OMAP_TLL_CHANNEL_2_EN_MASK			(1 << 2)
#define OMAP_TLL_CHANNEL_3_EN_MASK			(1 << 4)

/* UHH Register Set */
#define	OMAP_UHH_REVISION				(0x00)
#define	OMAP_UHH_SYSCONFIG				(0x10)
#define	OMAP_UHH_SYSCONFIG_NOSTBYMODE			(1 << 12)
#define	OMAP_UHH_SYSCONFIG_MIDLEMODE			(2 << 12)
#define	OMAP_UHH_SYSCONFIG_CACTIVITY			(1 << 8)
#define	OMAP_UHH_SYSCONFIG_NOIDLEMODE			(1 << 3)
#define	OMAP_UHH_SYSCONFIG_SIDLEMODE			(2 << 3)
#define	OMAP_UHH_SYSCONFIG_ENAWAKEUP			(1 << 2)
#define	OMAP_UHH_SYSCONFIG_SOFTRESET			(1 << 1)
#define	OMAP_UHH_SYSCONFIG_AUTOIDLE			(1 << 0)

#define	OMAP_UHH_SYSSTATUS				(0x14)
#define	OMAP_UHH_HOSTCONFIG				(0x40)
#define	OMAP_UHH_HOSTCONFIG_ULPI_BYPASS			(1 << 0)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P1_BYPASS		(1 << 0)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P2_BYPASS		(1 << 11)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P3_BYPASS		(1 << 12)
#define OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN		(1 << 2)
#define OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN		(1 << 3)
#define OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN		(1 << 4)
#define OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN		(1 << 5)
#define OMAP_UHH_HOSTCONFIG_P1_CONNECT_STATUS		(1 << 8)
#define OMAP_UHH_HOSTCONFIG_P2_CONNECT_STATUS		(1 << 9)
#define OMAP_UHH_HOSTCONFIG_P3_CONNECT_STATUS		(1 << 10)

#define	OMAP_UHH_DEBUG_CSR				(0x44)

struct ehci_hcd_omap {
    struct ehci_hcd     *ehci;
    struct platform_device  *dev;

    struct clk      *usbhost_ick;
    struct clk      *usbhost2_120m_fck;
    struct clk      *usbhost1_48m_fck;
    struct clk      *usbtll_fck;
    struct clk      *usbtll_ick;

    struct ehci_hcd_omap_port_data  port_data[OMAP3_HS_USB_PORTS];

    void __iomem        *uhh_base;
    void __iomem        *tll_base;
    void __iomem        *ehci_base;
};

static inline void ehci_omap_writel(void __iomem *base, u32 reg, u32 val)
{
	__raw_writel(val, base + reg);
}

static inline u32 ehci_omap_readl(void __iomem *base, u32 reg)
{
	return __raw_readl(base + reg);
}

static inline void ehci_omap_writeb(void __iomem *base, u8 reg, u8 val)
{
	__raw_writeb(val, base + reg);
}

static inline u8 ehci_omap_readb(void __iomem *base, u8 reg)
{
	return __raw_readb(base + reg);
}

static int omap_usb_port_ulpi_bypass(enum ehci_hcd_omap_mode mode)
{
	return mode != EHCI_HCD_OMAP_MODE_ULPI_PHY;
}

static int omap_usb_port_utmi_chanel_config(enum ehci_hcd_omap_mode mode)
{
	return (mode == EHCI_HCD_OMAP_MODE_UTMI_PHY_6PIN) ||
		(mode == EHCI_HCD_OMAP_MODE_UTMI_PHY_6PIN_ALT) ||
		(mode == EHCI_HCD_OMAP_MODE_UTMI_PHY_3PIN) ||
		(mode == EHCI_HCD_OMAP_MODE_UTMI_PHY_4PIN) ||
		(mode == EHCI_HCD_OMAP_MODE_UTMI_TLL_6PIN) ||
		(mode == EHCI_HCD_OMAP_MODE_UTMI_TLL_6PIN_ALT) ||
		(mode == EHCI_HCD_OMAP_MODE_UTMI_TLL_3PIN) ||
		(mode == EHCI_HCD_OMAP_MODE_UTMI_TLL_4PIN) ||
		(mode == EHCI_HCD_OMAP_MODE_UTMI_TLL_2PIN) ||
		(mode == EHCI_HCD_OMAP_MODE_UTMI_TLL_2PIN_ALT);
}

static unsigned omap_usb_port_fslsmode(enum ehci_hcd_omap_mode mode)
{
	switch (mode) {
		/* non serial modes */
	case EHCI_HCD_OMAP_MODE_ULPI_PHY:
	case EHCI_HCD_OMAP_MODE_ULPI_TLL_SDR:
	case EHCI_HCD_OMAP_MODE_ULPI_TLL_DDR:
		return 0x0;

		/* serial modes */
	case EHCI_HCD_OMAP_MODE_UTMI_PHY_6PIN:
		return 0x0;

	case EHCI_HCD_OMAP_MODE_UTMI_PHY_6PIN_ALT:
		return 0x1;

	case EHCI_HCD_OMAP_MODE_UTMI_PHY_3PIN:
		return 0x2;

	case EHCI_HCD_OMAP_MODE_UTMI_PHY_4PIN:
		return 0x3;

	case EHCI_HCD_OMAP_MODE_UTMI_TLL_6PIN:
		return 0x4;

	case EHCI_HCD_OMAP_MODE_UTMI_TLL_6PIN_ALT:
		return 0x5;

	case EHCI_HCD_OMAP_MODE_UTMI_TLL_3PIN:
		return 0x6;

	case EHCI_HCD_OMAP_MODE_UTMI_TLL_4PIN:
		return 0x7;

	case EHCI_HCD_OMAP_MODE_UTMI_TLL_2PIN:
		return 0xa;

	case EHCI_HCD_OMAP_MODE_UTMI_TLL_2PIN_ALT:
		return 0xB;
	}

	return 0x0;
}

static void omap_usb_utmi_init(struct ehci_hcd_omap *omap)
{
	unsigned reg;
	int i;

	/* Program the 3 TLL channels upfront */
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		reg = ehci_omap_readl(omap->tll_base, OMAP_TLL_CHANNEL_CONF(i));
		dev_dbg(&omap->dev->dev,
			"port %d: OMAP_TTL_CHANNEL_CONF_%d=%08x\n",
			i, i+1, ehci_omap_readl(omap->tll_base,
						OMAP_TLL_CHANNEL_CONF(i)));

		if (omap->port_data[i].flags & EHCI_HCD_OMAP_FLAG_NOBITSTUFF)
			reg |= OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF;
		else
			reg &= ~OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF;


		if (omap_usb_port_utmi_chanel_config(omap->port_data[i].mode)) {
			if (omap->port_data[i].flags
				& EHCI_HCD_OMAP_FLAG_AUTOIDLE)
				reg |= OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE;
			else
				reg &= ~OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE;
		} else {
			if (omap->port_data[i].flags
				& EHCI_HCD_OMAP_FLAG_AUTOIDLE)
				reg |= OMAP_TLL_CHANNEL_CONF_ULPI_ULPIAUTOIDLE;
			else
				reg &= ~OMAP_TLL_CHANNEL_CONF_ULPI_ULPIAUTOIDLE;
		}

		if (omap->port_data[i].mode == EHCI_HCD_OMAP_MODE_ULPI_TLL_DDR)
			reg |= OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE;
		else
			reg &= ~OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE;
		ehci_omap_writel(omap->tll_base, OMAP_TLL_CHANNEL_CONF(i), reg);
	}

	/* Program Common TLL register */
	ehci_omap_writel(omap->tll_base, OMAP_TLL_SHARED_CONF,
			 OMAP_TLL_SHARED_CONF_FCLK_IS_ON
			| OMAP_TLL_SHARED_CONF_USB_DIVRATION);

	/* Enable channels now */
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		/* Enable only the reg that is needed */
		if (!(omap->port_data[i].flags & EHCI_HCD_OMAP_FLAG_ENABLED))
			continue;

		reg = ehci_omap_readl(omap->tll_base, OMAP_TLL_CHANNEL_CONF(i));

		reg |= OMAP_TLL_CHANNEL_CONF_CHANEN;

		if (omap_usb_port_utmi_chanel_config(omap->port_data[i].mode))
			reg |= OMAP_TLL_CHANNEL_CONF_CHANMODE(1);
		else
			reg &= ~OMAP_TLL_CHANNEL_CONF_CHANMODE(3);

		reg &= ~(OMAP_TLL_CHANNEL_CONF_FSLSMODE(0xf));
		reg |= OMAP_TLL_CHANNEL_CONF_FSLSMODE(
			omap_usb_port_fslsmode(omap->port_data[i].mode));

		ehci_omap_writel(omap->tll_base, OMAP_TLL_CHANNEL_CONF(i), reg);
		dev_dbg(&omap->dev->dev,
			"port %d enabled: OMAP_TTL_CHANNEL_CONF_%d=%08x:%08x\n",
			i, i+1, reg, ehci_omap_readl(omap->tll_base,
			OMAP_TLL_CHANNEL_CONF(i)));

		ehci_omap_writeb(omap->tll_base,
				OMAP_TLL_ULPI_SCRATCH_REGISTER(i), 0xbe);
		dev_dbg(&omap->dev->dev, "ULPI_SCRATCH_REG[ch=%d]= 0x%02x\n",
				i+1, ehci_omap_readb(omap->tll_base,
				OMAP_TLL_ULPI_SCRATCH_REGISTER(i)));
	}
}
#endif
static int ohci_omap_init(struct usb_hcd *hcd)
{
	struct ohci_hcd		*ohci = hcd_to_ohci(hcd);
	struct omap_usb_config	*config = hcd->self.controller->platform_data;
	int			need_transceiver = (config->otg != 0);
	int			ret;

	dev_dbg(hcd->self.controller, "starting USB Controller\n");

	if (config->otg) {
		ohci_to_hcd(ohci)->self.otg_port = config->otg;
		/* default/minimum OTG power budget:  8 mA */
		ohci_to_hcd(ohci)->power_budget = 8;
	}

	/* boards can use OTG transceivers in non-OTG modes */
	need_transceiver = need_transceiver
			|| machine_is_omap_h2() || machine_is_omap_h3();

	if (cpu_is_omap16xx())
		ocpi_enable();

#ifdef	CONFIG_USB_OTG
	if (need_transceiver) {
		ohci->transceiver = otg_get_transceiver();
		if (ohci->transceiver) {
			int	status = otg_set_host(ohci->transceiver,
						&ohci_to_hcd(ohci)->self);
			dev_dbg(hcd->self.controller, "init %s transceiver, status %d\n",
					ohci->transceiver->label, status);
			if (status) {
				if (ohci->transceiver)
					put_device(ohci->transceiver->dev);
				return status;
			}
		} else {
			dev_err(hcd->self.controller, "can't find transceiver\n");
			return -ENODEV;
		}
		ohci->start_hnp = start_hnp;
	}
#endif

	omap_ohci_clock_power(1);

	if (cpu_is_omap15xx()) {
		omap_1510_local_bus_power(1);
		omap_1510_local_bus_init();
	}

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	/* board-specific power switching and overcurrent support */
	if (machine_is_omap_osk() || machine_is_omap_innovator()) {
		u32	rh = roothub_a (ohci);

		/* power switching (ganged by default) */
		rh &= ~RH_A_NPS;

		/* TPS2045 switch for internal transceiver (port 1) */
		if (machine_is_omap_osk()) {
			ohci_to_hcd(ohci)->power_budget = 250;

			rh &= ~RH_A_NOCP;

#ifndef CONFIG_ARCH_OMAP34XX
			/* gpio9 for overcurrent detction */
			omap_cfg_reg(W8_1610_GPIO9);
			gpio_request(9, "OHCI overcurrent");
			gpio_direction_input(9);

			/* for paranoia's sake:  disable USB.PUEN */
			omap_cfg_reg(W4_USB_HIGHZ);
#endif
		}
		ohci_writel(ohci, rh, &ohci->regs->roothub.a);
		ohci->flags &= ~OHCI_QUIRK_HUB_POWER;
	} else if (machine_is_nokia770()) {
		/* We require a self-powered hub, which should have
		 * plenty of power. */
		ohci_to_hcd(ohci)->power_budget = 0;
	}

	/* FIXME khubd hub requests should manage power switching */
	omap_ohci_transceiver_power(1);

	/* board init will have already handled HMC and mux setup.
	 * any external transceiver should already be initialized
	 * too, so all configured ports use the right signaling now.
	 */

	return 0;
}

#ifdef CONFIG_MACH_MAPPHONE
/* omap_start_ehc
 *	- Start the TI USBHOST controller
 */
static int omap_start_ehc(struct ehci_hcd_omap *omap, struct usb_hcd *hcd)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	unsigned reg = 0;
	int ret = 0;
	int reset_delay;
	int i;

	dev_dbg(&omap->dev->dev, "starting TI EHCI USB Controller\n");

	/* Enable Clocks for USBHOST */
	omap->usbhost_ick = clk_get(&omap->dev->dev, "usbhost_ick");
	if (IS_ERR(omap->usbhost_ick)) {
		ret =  PTR_ERR(omap->usbhost_ick);
		goto err_host_ick;
	}
	clk_enable(omap->usbhost_ick);

	omap->usbhost2_120m_fck = clk_get(&omap->dev->dev, "usbhost_120m_fck");
	if (IS_ERR(omap->usbhost2_120m_fck)) {
		ret = PTR_ERR(omap->usbhost2_120m_fck);
		goto err_host_120m_fck;
	}
	clk_enable(omap->usbhost2_120m_fck);

	omap->usbhost1_48m_fck = clk_get(&omap->dev->dev, "usbhost_48m_fck");
	if (IS_ERR(omap->usbhost1_48m_fck)) {
		ret = PTR_ERR(omap->usbhost1_48m_fck);
		goto err_host_48m_fck;
	}
	clk_enable(omap->usbhost1_48m_fck);

	reset_delay = 0;
	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
		reset_delay = reset_delay > omap->port_data[i].reset_delay ?
			reset_delay : omap->port_data[i].reset_delay;

		if (omap->port_data[i].startup) {
			ret = omap->port_data[i].startup(omap->dev, i);
			if (ret < 0)
				return ret;
		}

		if (omap->port_data[i].reset)
			omap->port_data[i].reset(omap->dev, i, 0);
	}
	if (reset_delay)
		udelay(reset_delay);

	/* Configure TLL for 60Mhz clk for ULPI */
	omap->usbtll_fck = clk_get(&omap->dev->dev, "usbtll_fck");
	if (IS_ERR(omap->usbtll_fck)) {
		ret = PTR_ERR(omap->usbtll_fck);
		goto err_tll_fck;
	}
	clk_enable(omap->usbtll_fck);

	omap->usbtll_ick = clk_get(&omap->dev->dev, "usbtll_ick");
	if (IS_ERR(omap->usbtll_ick)) {
		ret = PTR_ERR(omap->usbtll_ick);
		goto err_tll_ick;
	}
	clk_enable(omap->usbtll_ick);

	/* perform TLL soft reset, and wait until reset is complete */
	ehci_omap_writel(omap->tll_base, OMAP_USBTLL_SYSCONFIG,
			OMAP_USBTLL_SYSCONFIG_SOFTRESET);

	/* Wait for TLL reset to complete */
	while (!(ehci_omap_readl(omap->tll_base, OMAP_USBTLL_SYSSTATUS)
			& OMAP_USBTLL_SYSSTATUS_RESETDONE)) {
		cpu_relax();

		if (time_after(jiffies, timeout)) {
			dev_dbg(&omap->dev->dev, "operation timed out\n");
			ret = -EINVAL;
			goto err_sys_status;
		}
	}

	dev_dbg(&omap->dev->dev, "TLL RESET DONE\n");

	/* (1<<3) = no idle mode only for initial debugging */
	ehci_omap_writel(omap->tll_base, OMAP_USBTLL_SYSCONFIG,
			OMAP_USBTLL_SYSCONFIG_ENAWAKEUP |
			OMAP_USBTLL_SYSCONFIG_SIDLEMODE |
			OMAP_USBTLL_SYSCONFIG_CACTIVITY);


	/* Put UHH in NoIdle/NoStandby mode */
	ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG,
			 OMAP_UHH_SYSCONFIG_ENAWAKEUP
			 | OMAP_UHH_SYSCONFIG_SIDLEMODE
			 | OMAP_UHH_SYSCONFIG_MIDLEMODE
			 | OMAP_UHH_SYSCONFIG_AUTOIDLE);

	reg = ehci_omap_readl(omap->uhh_base, OMAP_UHH_HOSTCONFIG);

	/* setup ULPI bypass and burst configurations */
	reg |= (OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN
			| OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN
			| OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN);
	reg &= ~OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN;


	if (omap->port_data[0].flags != EHCI_HCD_OMAP_FLAG_ENABLED)
		reg &= ~OMAP_UHH_HOSTCONFIG_P1_CONNECT_STATUS;
	if (omap->port_data[1].flags != EHCI_HCD_OMAP_FLAG_ENABLED)
		reg &= ~OMAP_UHH_HOSTCONFIG_P2_CONNECT_STATUS;
	if (omap->port_data[2].flags != EHCI_HCD_OMAP_FLAG_ENABLED)
		reg &= ~OMAP_UHH_HOSTCONFIG_P3_CONNECT_STATUS;

	/* Bypass the TLL module for PHY mode operation */
	 if (omap_rev() <= OMAP3430_REV_ES2_1) {
		dev_dbg(&omap->dev->dev, "OMAP3 ES version <= ES2.1 \n");
		if (omap_usb_port_ulpi_bypass(omap->port_data[0].mode))
			reg |= OMAP_UHH_HOSTCONFIG_ULPI_BYPASS;
		else
			reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_BYPASS;
	} else {
		dev_dbg(&omap->dev->dev, "OMAP3 ES version > ES2.1\n");
		if (omap_usb_port_ulpi_bypass(omap->port_data[0].mode))
			reg |= OMAP_UHH_HOSTCONFIG_ULPI_P1_BYPASS;
		else
			reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_P1_BYPASS;

		if (omap_usb_port_ulpi_bypass(omap->port_data[1].mode))
			reg |= OMAP_UHH_HOSTCONFIG_ULPI_P2_BYPASS;
		else
			reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_P2_BYPASS;

		if (omap_usb_port_ulpi_bypass(omap->port_data[2].mode))
			reg |= OMAP_UHH_HOSTCONFIG_ULPI_P3_BYPASS;
		else
			reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_P3_BYPASS;
	}
	ehci_omap_writel(omap->uhh_base, OMAP_UHH_HOSTCONFIG, reg);
	dev_dbg(&omap->dev->dev, "UHH setup done, uhh_hostconfig=%x\n", reg);

	/* Enable UTMI mode for required TLL channels */
	omap_usb_utmi_init(omap);

	reset_delay = 0;
	for (i = 0; i < OMAP3_HS_USB_PORTS; i++) {
		reset_delay = reset_delay > omap->port_data[i].reset_delay ?
				reset_delay : omap->port_data[i].reset_delay;
	}
	if (reset_delay)
		udelay(reset_delay);

	for (i = 0; i < OMAP3_HS_USB_PORTS; i++) {
		if (omap->port_data[i].reset)
			omap->port_data[i].reset(omap->dev, i, 1);
	}
	return 0;

err_sys_status:
	clk_disable(omap->usbtll_ick);
	clk_put(omap->usbtll_ick);

err_tll_ick:
	clk_disable(omap->usbtll_fck);
	clk_put(omap->usbtll_fck);

err_tll_fck:
	clk_disable(omap->usbhost1_48m_fck);
	clk_put(omap->usbhost1_48m_fck);

err_host_48m_fck:
	clk_disable(omap->usbhost2_120m_fck);
	clk_put(omap->usbhost2_120m_fck);

err_host_120m_fck:
	clk_disable(omap->usbhost_ick);
	clk_put(omap->usbhost_ick);

err_host_ick:
	return ret;
}
#endif
static void ohci_omap_stop(struct usb_hcd *hcd)
{
	dev_dbg(hcd->self.controller, "stopping USB Controller\n");
	ohci_stop(hcd);
	omap_ohci_clock_power(0);
}


/*-------------------------------------------------------------------------*/

/**
 * usb_hcd_omap_probe - initialize OMAP-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 */
static int usb_hcd_omap_probe (const struct hc_driver *driver,
			  struct platform_device *pdev)
{
	int retval, irq;
	struct usb_hcd *hcd = 0;
	struct ohci_hcd *ohci;
#ifdef CONFIG_MACH_MAPPHONE
	struct ehci_hcd_omap *omap;
	struct omap_usb_config *pdata = pdev->dev.platform_data;
	struct resource *res;
	int ret = -ENODEV;
	retval = 0;
#else
	if (pdev->num_resources != 2) {
		printk(KERN_ERR "hcd probe: invalid num_resources: %i\n",
		       pdev->num_resources);
		return -ENODEV;
	}

	if (pdev->resource[0].flags != IORESOURCE_MEM
			|| pdev->resource[1].flags != IORESOURCE_IRQ) {
		printk(KERN_ERR "hcd probe: invalid resource type\n");
		return -ENODEV;
	}
#endif

#ifndef CONFIG_ARCH_OMAP34XX
	usb_host_ck = clk_get(&pdev->dev, "usb_hhc_ck");
	if (IS_ERR(usb_host_ck))
		return PTR_ERR(usb_host_ck);

	if (!cpu_is_omap15xx())
		usb_dc_ck = clk_get(&pdev->dev, "usb_dc_ck");
	else
		usb_dc_ck = clk_get(&pdev->dev, "lb_ck");

	if (IS_ERR(usb_dc_ck)) {
		clk_put(usb_host_ck);
		return PTR_ERR(usb_dc_ck);
	}
#endif

#ifdef CONFIG_MACH_MAPPHONE
	omap = kzalloc(sizeof(*omap), GFP_KERNEL);
	if (!omap) {
		ret = -ENOMEM;
		goto err0;
	}
#endif

	hcd = usb_create_hcd (driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto err0;
	}
#ifndef CONFIG_MACH_MAPPHONE
#if defined(CONFIG_ARCH_OMAP34XX)
	clk_enable(clk_get(NULL, "usbhost_ick"));
	clk_enable(clk_get(NULL, "usbtll_ick"));
	clk_enable(clk_get(NULL, "usbtll_fck"));
	clk_enable(clk_get(NULL, "usbhost_120m_fck"));
	clk_enable(clk_get(NULL, "usbhost_48m_fck"));
#endif
	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;
#else
	platform_set_drvdata(pdev, hcd);
	omap->dev		= pdev;
	memcpy(&omap->port_data, &pdata->port_data, sizeof(omap->port_data));

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
#endif

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		dev_dbg(&pdev->dev, "request_mem_region failed\n");
		retval = -EBUSY;
		goto err1;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "can't ioremap OHCI HCD\n");
		retval = -ENOMEM;
		goto err2;
	}

	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);

	host_initialized = 0;
	host_enabled = 1;
#ifdef CONFIG_MACH_MAPPHONE
	ohci_irq_num = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	omap->uhh_base = ioremap(res->start, resource_size(res));
	if (!omap->uhh_base) {
		dev_err(&pdev->dev, "UHH ioremap failed\n");
		ret = -ENOMEM;
		goto err3;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	omap->tll_base = ioremap(res->start, resource_size(res));
	if (!omap->tll_base) {
		dev_err(&pdev->dev, "TLL ioremap failed\n");
		ret = -ENOMEM;
		goto err4;
	}
#endif

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		retval = -ENXIO;
#ifdef CONFIG_MACH_MAPPHONE
		goto err5;
#else
		goto err3;
#endif
	}
#ifdef CONFIG_MACH_MAPPHONE
	retval = omap_start_ehc(omap, hcd);
	if (retval) {
		dev_dbg(&pdev->dev, "failed to start ohci\n");
		goto err5;
	}
#endif
	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED);
	if (retval)
#ifndef CONFIG_MACH_MAPPHONE
		goto err3;
#else
		goto err5;
	else
		ohci_irq_num = irq;
#endif

	host_initialized = 1;

	if (!host_enabled)
		omap_ohci_clock_power(0);

	return 0;
#ifdef CONFIG_MACH_MAPPHONE
err5:
	iounmap(omap->tll_base);
err4:
	iounmap(omap->uhh_base);
#endif
err3:
	iounmap(hcd->regs);
err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
	usb_put_hcd(hcd);
err0:

#ifndef CONFIG_ARCH_OMAP34XX
	clk_put(usb_dc_ck);
	clk_put(usb_host_ck);
#endif
	return retval;
}


/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_omap_remove - shutdown processing for OMAP-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 */
static inline void
usb_hcd_omap_remove (struct usb_hcd *hcd, struct platform_device *pdev)
{
	struct ohci_hcd		*ohci = hcd_to_ohci (hcd);

	usb_remove_hcd(hcd);
	if (ohci->transceiver) {
		(void) otg_set_host(ohci->transceiver, 0);
		put_device(ohci->transceiver->dev);
	}
#ifndef CONFIG_ARCH_OMAP34XX
	if (machine_is_omap_osk())
		gpio_free(9);
#endif
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

#ifndef CONFIG_ARCH_OMAP34XX
	clk_put(usb_dc_ck);
	clk_put(usb_host_ck);
#endif
}

/*-------------------------------------------------------------------------*/

static int
ohci_omap_start (struct usb_hcd *hcd)
{
	struct omap_usb_config *config;
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	int		ret;

	if (!host_enabled)
		return 0;
	config = hcd->self.controller->platform_data;
	if (config->otg || config->rwc) {
		ohci->hc_control = OHCI_CTRL_RWC;
		writel(OHCI_CTRL_RWC, &ohci->regs->control);
	}

	if ((ret = ohci_run (ohci)) < 0) {
		dev_err(hcd->self.controller, "can't start\n");
		ohci_stop (hcd);
		return ret;
	}
	return 0;
}

#ifdef CONFIG_PM
static int omap_ohci_bus_suspend(struct usb_hcd *hcd)
{
	int res = 0;
	int ret = 0;
#ifdef CONFIG_MACH_MAPPHONE
	int chix;
	struct usb_device *childdev;
	int count = 0;
	int i = 0;
	struct ehci_hcd_omap *omap = dev_get_drvdata(hcd->self.controller);
#endif
	struct ohci_hcd *ohci = hcd_to_ohci(hcd);
#if defined(CONFIG_ARCH_OMAP34XX)
	struct omap_usb_config *config = hcd->self.controller->platform_data;
#endif
#ifdef CONFIG_MACH_MAPPHONE

	for (chix = 0; chix < hcd->self.root_hub->maxchild; chix++) {
		childdev = hcd->self.root_hub->children[chix];
		if (childdev)
			count++;
	}

	if (!count) {
		printk(KERN_ERR " %s no device, suspend failed. \n", __func__);
		return -EBUSY ;
	}
#endif

	ret = ohci_bus_suspend(hcd);
	if (ret)
		return ret;
	mdelay(8); /* MSTANDBY assertion is delayed by ~8ms */

#if defined(CONFIG_ARCH_OMAP34XX)
	if (config->usbhost_standby_status)
		res = config->usbhost_standby_status();
#endif
	if (res == 0) {
		printk(KERN_ERR "ohci: suspend failed!\n");
		ohci_bus_resume(hcd);
		return -EBUSY ;
	}

#ifdef CONFIG_MACH_MAPPHONE
	if (ohci_irq_num)
		disable_irq(ohci_irq_num);
#endif
	/* go ahead turn off clock */
	clk_disable(clk_get(NULL, "usbtll_fck"));
	clk_disable(clk_get(NULL, "usbhost_120m_fck"));
	clk_disable(clk_get(NULL, "usbhost_48m_fck"));

	/* the omap usb host auto-idle is not fully functional,
	 * manually enable/disable usbtll_ick during
	 * the suspend/resume time.
	 */
	clk_disable(clk_get(NULL, "usbtll_ick"));

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
#ifdef CONFIG_MACH_MAPPHONE
	if (ohci_irq_num)
		enable_irq(ohci_irq_num);
#endif
	ohci_to_hcd(ohci)->state = HC_STATE_SUSPENDED;

#ifdef CONFIG_MACH_MAPPHONE
	/* Call the Platform Suspend for Each port */
	if (omap) {
		for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
			if (config->port_data[i].suspend)
				config->port_data[i].suspend(omap->dev, i, 1);
		}
	}
#endif

	return ret;
}

static int omap_ohci_bus_resume(struct usb_hcd *hcd)
{
	int ret = 0;

#ifdef CONFIG_MACH_MAPPHONE
	int i = 0;
	struct omap_usb_config *config = hcd->self.controller->platform_data;
	struct ehci_hcd_omap *omap = dev_get_drvdata(hcd->self.controller);

	if (ohci_irq_num)
		disable_irq(ohci_irq_num);
	/* Call the Platform Resume for Each port */
	if (omap) {
		for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {
			if (config->port_data[i].suspend)
				config->port_data[i].suspend(omap->dev, i, 0);
		}
	}
#endif
	/* the omap usb host auto-idle is not fully functional,
	 * manually enable/disable usbtll_ick during
	 * the suspend/resume time.
	 */
	clk_enable(clk_get(NULL, "usbtll_ick"));
	clk_enable(clk_get(NULL, "usbtll_fck"));
	clk_enable(clk_get(NULL, "usbhost_120m_fck"));
	clk_enable(clk_get(NULL, "usbhost_48m_fck"));
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

#ifdef CONFIG_MACH_MAPPHONE
	if (ohci_irq_num)
		enable_irq(ohci_irq_num);
#endif
	ohci_finish_controller_resume(hcd);
	ret = ohci_bus_resume(hcd);
	return ret;
}
#endif /* CONFIG_PM */

static void omap_ohci_shutdown(struct usb_hcd *hcd)
{
	dev_dbg(hcd->self.controller, "%s %lu\n", __func__, jiffies);
	clk_enable(clk_get(NULL, "usbtll_fck"));
	clk_enable(clk_get(NULL, "usbhost_120m_fck"));
	clk_enable(clk_get(NULL, "usbhost_48m_fck"));
	ohci_shutdown(hcd);
#ifdef CONFIG_MACH_MAPPHONE
	ohci_irq_num = 0;
#endif
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_omap_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"OMAP OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ohci_omap_init,
	.start =		ohci_omap_start,
	.stop =			ohci_omap_stop,
	.shutdown =		omap_ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend =		omap_ohci_bus_suspend,
	.bus_resume =		omap_ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_omap_drv_probe(struct platform_device *dev)
{
	return usb_hcd_omap_probe(&ohci_omap_hc_driver, dev);
}

static int ohci_hcd_omap_drv_remove(struct platform_device *dev)
{
	struct usb_hcd		*hcd = platform_get_drvdata(dev);

	usb_hcd_omap_remove(hcd, dev);
	platform_set_drvdata(dev, NULL);

	return 0;
}

/*-------------------------------------------------------------------------*/

#ifdef	CONFIG_PM

static int ohci_omap_suspend(struct platform_device *dev, pm_message_t message)
{
	return 0;
}

static int ohci_omap_resume(struct platform_device *dev)
{
	return 0;
}

#endif

/*-------------------------------------------------------------------------*/

/*
 * Driver definition to register with the OMAP bus
 */
static struct platform_driver ohci_hcd_omap_driver = {
	.probe		= ohci_hcd_omap_drv_probe,
	.remove		= ohci_hcd_omap_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
#ifdef CONFIG_PM
	.suspend        = ohci_omap_suspend,
	.resume         = ohci_omap_resume,
#endif
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ohci",
	},
};

MODULE_ALIAS("platform:ohci");
