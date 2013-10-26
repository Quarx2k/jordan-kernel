/*
 * ehci-omap.c - driver for USBHOST on OMAP 34xx processor
 *
 * Bus Glue for OMAP34xx USBHOST 3 port EHCI controller
 * Tested on OMAP3430 ES2.0 SDP
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 *	Author: Vikram Pandita <vikram.pandita@ti.com>
 *
 * Copyright (C) 2009 Nokia Corporation
 *	Contact: Felipe Balbi <felipe.balbi@nokia.com>
 *
 * Based on "ehci-fsl.c" and "ehci-au1xxx.c" ehci glue layers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * TODO (last updated Feb 23rd, 2009):
 *	- add kernel-doc
 *	- enable AUTOIDLE
 *	- move DPLL5 programming to clock fw
 *	- add suspend/resume
 *	- move workarounds to board-files
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <plat/usb.h>


/*
 * OMAP USBHOST Register addresses: VIRTUAL ADDRESSES
 *	Use ehci_omap_readl()/ehci_omap_writel() functions
 */

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
#define	OMAP_UHH_SYSCONFIG_MIDLEMODE_MASK		(3 << 12)
#define	OMAP_UHH_SYSCONFIG_CACTIVITY			(1 << 8)
#define	OMAP_UHH_SYSCONFIG_NOIDLEMODE			(1 << 3)
#define	OMAP_UHH_SYSCONFIG_SIDLEMODE			(2 << 3)
#define	OMAP_UHH_SYSCONFIG_SIDLEMODE_MASK		(3 << 3)
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

/* EHCI Register Set */
#define	EHCI_INSNREG05_ULPI				(0xA4)
#define	EHCI_INSNREG05_ULPI_CONTROL_SHIFT		31
#define	EHCI_INSNREG05_ULPI_PORTSEL_SHIFT		24
#define	EHCI_INSNREG05_ULPI_OPSEL_SHIFT			22
#define	EHCI_INSNREG05_ULPI_REGADD_SHIFT		16
#define	EHCI_INSNREG05_ULPI_EXTREGADD_SHIFT		8
#define	EHCI_INSNREG05_ULPI_WRDATA_SHIFT		0

#define	OHCI_BASE_ADDR		0x48064400
#define	OHCI_HC_CONTROL		(OHCI_BASE_ADDR + 0x4)
#define	OHCI_HC_CTRL_SUSPEND	(3 << 6)
#define	OHCI_HC_CTRL_RESUME	(1 << 6)

/*-------------------------------------------------------------------------*/

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

/*-------------------------------------------------------------------------*/

struct ehci_hcd_omap {
	struct ehci_hcd		*ehci;
	struct platform_device	*dev;

	struct clk		*usbhost_ick;
	struct clk		*usbhost2_120m_fck;
	struct clk		*usbhost1_48m_fck;
	struct clk		*usbtll_fck;
	struct clk		*usbtll_ick;

	struct ehci_hcd_omap_port_data	port_data[OMAP3_HS_USB_PORTS];

	void __iomem		*uhh_base;
	void __iomem		*tll_base;
	void __iomem		*ehci_base;
};

static DEFINE_SPINLOCK(usb_clocks_lock);

/*-------------------------------------------------------------------------*/

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
			if (omap->port_data[i].flags & EHCI_HCD_OMAP_FLAG_AUTOIDLE)
				reg |= OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE;
			else
				reg &= ~OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE;
		} else {
			if (omap->port_data[i].flags & EHCI_HCD_OMAP_FLAG_AUTOIDLE)
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
			i, i+1, reg, ehci_omap_readl(omap->tll_base, OMAP_TLL_CHANNEL_CONF(i)));

		ehci_omap_writeb(omap->tll_base,
				OMAP_TLL_ULPI_SCRATCH_REGISTER(i), 0xbe);
		dev_dbg(&omap->dev->dev, "ULPI_SCRATCH_REG[ch=%d]= 0x%02x\n",
				i+1, ehci_omap_readb(omap->tll_base,
				OMAP_TLL_ULPI_SCRATCH_REGISTER(i)));
	}
}

/*-------------------------------------------------------------------------*/

/* omap_start_ehc
 *	- Start the TI USBHOST controller
 */
static int omap_start_ehc(struct ehci_hcd_omap *omap, struct usb_hcd *hcd)
{
#ifndef CONFIG_MAPPHONE_2NDBOOT
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
#endif
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

	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

#ifndef CONFIG_MAPPHONE_2NDBOOT
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
#endif

	dev_dbg(&omap->dev->dev, "TLL RESET DONE\n");

	/* SmartIdle mode */
	ehci_omap_writel(omap->tll_base, OMAP_USBTLL_SYSCONFIG,
			OMAP_USBTLL_SYSCONFIG_ENAWAKEUP |
			OMAP_USBTLL_SYSCONFIG_SIDLEMODE |
			OMAP_USBTLL_SYSCONFIG_AUTOIDLE);

	/* Put UHH in NoIdle/NoStandby mode */
	ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG,
			OMAP_UHH_SYSCONFIG_ENAWAKEUP |
			OMAP_UHH_SYSCONFIG_NOIDLEMODE |
			OMAP_UHH_SYSCONFIG_NOSTBYMODE |
			OMAP_UHH_SYSCONFIG_AUTOIDLE);
#ifdef CONFIG_MACH_MAPPHONE
	/* We need to suspend OHCI in order for the usbhost
	 * domain to go standby.
	 * OHCI would never be resumed for UMTS modem */
	if (!is_cdma_phone())
		omap_writel(OHCI_HC_CTRL_SUSPEND, OHCI_HC_CONTROL);
#endif

	reg = ehci_omap_readl(omap->uhh_base, OMAP_UHH_HOSTCONFIG);

	/* setup ULPI bypass and burst configurations */
	reg |= (OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN
			| OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN
			| OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN);
	reg &= ~OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN;


	if (!(omap->port_data[0].flags & EHCI_HCD_OMAP_FLAG_ENABLED))
		reg &= ~OMAP_UHH_HOSTCONFIG_P1_CONNECT_STATUS;
	if (!(omap->port_data[1].flags & EHCI_HCD_OMAP_FLAG_ENABLED))
		reg &= ~OMAP_UHH_HOSTCONFIG_P2_CONNECT_STATUS;
	if (!(omap->port_data[2].flags & EHCI_HCD_OMAP_FLAG_ENABLED))
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

#if defined(CONFIG_MACH_MAPPHONE)
	/* Refer ISSUE2: LINK assumes external charge pump */
	/* use Port1 VBUS to charge externally Port2:
	 *      So for PHY mode operation use Port2 only */
	ehci_omap_writel(omap->ehci_base, EHCI_INSNREG05_ULPI,
		(0xA << EHCI_INSNREG05_ULPI_REGADD_SHIFT) |/* OTG ctrl reg*/
		(2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT) |/*   Write */
		(2 << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT) |/* Port1 */
		(1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT) |/* Start */
		(0x26));
	while (!(ehci_omap_readl(omap->ehci_base, EHCI_INSNREG05_ULPI) &
		(1<<EHCI_INSNREG05_ULPI_CONTROL_SHIFT))) {
		cpu_relax();
	}
#endif

	return 0;

#ifndef CONFIG_MAPPHONE_2NDBOOT
err_sys_status:
	clk_disable(omap->usbtll_ick);
	clk_put(omap->usbtll_ick);
#endif

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

static void omap_stop_ehc(struct ehci_hcd_omap *omap, struct usb_hcd *hcd)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(100);

	dev_dbg(&omap->dev->dev, "stopping TI EHCI USB Controller\n");

	/* Reset OMAP modules for insmod/rmmod to work */
	ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG,
			OMAP_UHH_SYSCONFIG_SOFTRESET);
	while (!(ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSSTATUS)
				& (1 << 0))) {
		cpu_relax();

		if (time_after(jiffies, timeout))
			dev_dbg(&omap->dev->dev, "operation timed out\n");
	}

	while (!(ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSSTATUS)
				& (1 << 1))) {
		cpu_relax();

		if (time_after(jiffies, timeout))
			dev_dbg(&omap->dev->dev, "operation timed out\n");
	}

	while (!(ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSSTATUS)
				& (1 << 2))) {
		cpu_relax();

		if (time_after(jiffies, timeout))
			dev_dbg(&omap->dev->dev, "operation timed out\n");
	}

	ehci_omap_writel(omap->tll_base, OMAP_USBTLL_SYSCONFIG, (1 << 1));

	while (!(ehci_omap_readl(omap->tll_base, OMAP_USBTLL_SYSSTATUS)
				& (1 << 0))) {
		cpu_relax();

		if (time_after(jiffies, timeout))
			dev_dbg(&omap->dev->dev, "operation timed out\n");
	}

	if (omap->usbtll_fck != NULL) {
		clk_disable(omap->usbtll_fck);
		clk_put(omap->usbtll_fck);
		omap->usbtll_fck = NULL;
	}

	if (omap->usbhost_ick != NULL) {
		clk_disable(omap->usbhost_ick);
		clk_put(omap->usbhost_ick);
		omap->usbhost_ick = NULL;
	}

	if (omap->usbhost1_48m_fck != NULL) {
		clk_disable(omap->usbhost1_48m_fck);
		clk_put(omap->usbhost1_48m_fck);
		omap->usbhost1_48m_fck = NULL;
	}

	if (omap->usbhost2_120m_fck != NULL) {
		clk_disable(omap->usbhost2_120m_fck);
		clk_put(omap->usbhost2_120m_fck);
		omap->usbhost2_120m_fck = NULL;
	}

	if (omap->usbtll_ick != NULL) {
		clk_disable(omap->usbtll_ick);
		clk_put(omap->usbtll_ick);
		omap->usbtll_ick = NULL;
	}

	dev_dbg(&omap->dev->dev, "Clock to USB host has been disabled\n");
}

static irqreturn_t usbtll_irq(int irq, void *pdev)
{
	unsigned long flags;
	u32 sysconfig;
	u32 usbtll_irqstatus;
	struct ehci_hcd_omap *omap = platform_get_drvdata(
					(struct platform_device *)pdev);
	struct usb_hcd *hcd = ehci_to_hcd(omap->ehci);

	usbtll_irqstatus = ehci_omap_readl(
				omap->tll_base, OMAP_USBTLL_IRQSTATUS);

	spin_lock_irqsave(&usb_clocks_lock, flags);
	if (usbtll_irqstatus & 1) {
		if (test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
			ehci_omap_writel(omap->tll_base,
				OMAP_USBTLL_IRQSTATUS, usbtll_irqstatus);
			ehci_omap_writel(omap->tll_base, OMAP_USBTLL_IRQENABLE,
				 0);
			spin_unlock_irqrestore(&usb_clocks_lock, flags);
			return IRQ_HANDLED;
		}
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_timeout(&omap->ehci->wake_lock_ehci_rwu, HZ/2);
#endif
		if (omap->usbtll_fck)
			clk_enable(omap->usbtll_fck);
		/* Disable usbtll irq to prevent race condition in suspend */
		ehci_omap_writel(omap->tll_base, OMAP_TLL_SHARED_CONF,
				1 | ehci_omap_readl(omap->tll_base,
					OMAP_TLL_SHARED_CONF));
		ehci_omap_writel(omap->tll_base,
			OMAP_USBTLL_IRQSTATUS, usbtll_irqstatus);
		if (omap->usbhost2_120m_fck)
			clk_enable(omap->usbhost2_120m_fck);
		if (omap->usbhost1_48m_fck)
			clk_enable(omap->usbhost1_48m_fck);

		/* Put UHH in NoStandby mode */
		sysconfig = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);
		sysconfig &= ~OMAP_UHH_SYSCONFIG_MIDLEMODE_MASK;
		sysconfig |= OMAP_UHH_SYSCONFIG_NOSTBYMODE;
		ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG, sysconfig);

		/* Put UHH in NoIdle mode */
		sysconfig = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);
		sysconfig &= ~OMAP_UHH_SYSCONFIG_SIDLEMODE_MASK;
		sysconfig |= OMAP_UHH_SYSCONFIG_NOIDLEMODE;
		ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG, sysconfig);
		sysconfig = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);

		ehci_omap_writel(omap->tll_base, OMAP_USBTLL_IRQENABLE, 0);
		set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
		enable_irq(hcd->irq);
	}
	spin_unlock_irqrestore(&usb_clocks_lock, flags);

	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_omap_hc_driver;

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * ehci_hcd_omap_probe - initialize TI-based HCDs
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 */
static int ehci_hcd_omap_probe(struct platform_device *pdev)
{
	struct ehci_hcd_omap_platform_data *pdata = pdev->dev.platform_data;
	struct ehci_hcd_omap *omap;
	struct resource *res;
	struct usb_hcd *hcd;

	int irq = platform_get_irq(pdev, 0);
	int ret = -ENODEV;

	if (!pdata) {
		dev_dbg(&pdev->dev, "missing platform_data\n");
		goto err_pdata;
	}

#ifdef CONFIG_MAPPHONE_2NDBOOT

	/* FIXME: Find proper defines for the addresses */

	/* We need to get the interrupts in order */
	printk(KERN_INFO "EHCI-OMAP: Fixing after 2nd-boot\n");
	
	/* Mask IRQs */
	omap_writel(0x2000, 0x482000cc);
	
	/* Reset USBINTR */
	omap_writel(0x00, 0x48064818);
	
	/* Set ERROR status on everything (this may not be necessary) */
	omap_writel(0x3f, 0x48064814);
	
	/* Clear HCINTERRUPTSTATUS */
	omap_writel(0x4000007f, 0x4806440c);
	
	/* Clear HCINTERRUPTDISABLE */
	omap_writel(0xc000007f, 0x48064414);

#endif

	if (usb_disabled())
		goto err_disabled;
	
#ifdef CONFIG_MAPPHONE_2NDBOOT

	/* Enable IRQs */
	omap_writel(0x0, 0x4806201c);
	
	/* Set them to event pending */
	omap_writel(0x7, 0x48062018);

#endif

	ret = request_irq(78, usbtll_irq, IRQF_DISABLED | IRQF_SHARED,
				"usbtll", pdev);
	if (ret < 0) {
		printk(KERN_ERR "\nCan't get USBTLL IRQ\n");
		goto err_disabled;
	}

	omap = kzalloc(sizeof(*omap), GFP_KERNEL);
	if (!omap) {
		ret = -ENOMEM;
		goto err_create_hcd;
	}

	hcd = usb_create_hcd(&ehci_omap_hc_driver, &pdev->dev,
			dev_name(&pdev->dev));
	if (!hcd) {
		dev_dbg(&pdev->dev, "failed to create hcd with err %d\n", ret);
		ret = -ENOMEM;
		goto err_create_hcd;
	}

	platform_set_drvdata(pdev, omap);
	omap->dev		= pdev;

	memcpy(&omap->port_data, &pdata->port_data, sizeof(omap->port_data));

	omap->ehci		= hcd_to_ehci(hcd);
	omap->ehci->sbrn	= 0x20;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "EHCI ioremap failed\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}

	/* we know this is the memory we want, no need to ioremap again */
	omap->ehci->caps = hcd->regs;
	omap->ehci_base = hcd->regs;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	omap->uhh_base = ioremap(res->start, resource_size(res));
	if (!omap->uhh_base) {
		dev_err(&pdev->dev, "UHH ioremap failed\n");
		ret = -ENOMEM;
		goto err_uhh_ioremap;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	omap->tll_base = ioremap(res->start, resource_size(res));
	if (!omap->tll_base) {
		dev_err(&pdev->dev, "TLL ioremap failed\n");
		ret = -ENOMEM;
		goto err_tll_ioremap;
	}

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&omap->ehci->wake_lock_ehci_rwu,
			WAKE_LOCK_IDLE, "ehci_rwu");
#endif

	ret = omap_start_ehc(omap, hcd);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to start ehci\n");
		goto err_start;
	}

	omap->ehci->regs = hcd->regs
		+ HC_LENGTH(readl(&omap->ehci->caps->hc_capbase));

	/* cache this readonly data; minimize chip reads */
	omap->ehci->hcs_params = readl(&omap->ehci->caps->hcs_params);

	/* SET 1 micro-frame Interrupt interval */
	writel(readl(&omap->ehci->regs->command) | (1 << 16),
			&omap->ehci->regs->command);

	ret = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to add hcd with err %d\n", ret);
		goto err_add_hcd;
	}

	return 0;

err_add_hcd:
	omap_stop_ehc(omap, hcd);

err_start:
	iounmap(omap->tll_base);

err_tll_ioremap:
	iounmap(omap->uhh_base);

err_uhh_ioremap:
	iounmap(hcd->regs);

err_ioremap:
	usb_put_hcd(hcd);

err_create_hcd:
err_disabled:
err_pdata:
	return ret;
}

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * ehci_hcd_omap_remove - shutdown processing for EHCI HCDs
 * @pdev: USB Host Controller being removed
 *
 * Reverses the effect of usb_ehci_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 */
static int ehci_hcd_omap_remove(struct platform_device *pdev)
{
	unsigned long flags;
	struct ehci_hcd_omap *omap = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(omap->ehci);

	spin_lock_irqsave(&usb_clocks_lock, flags);
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		/* Enable All F-clks now before accessing controller */
		if (omap->usbtll_fck)
			clk_enable(omap->usbtll_fck);
		if (omap->usbhost2_120m_fck)
			clk_enable(omap->usbhost2_120m_fck);
		if (omap->usbhost1_48m_fck)
			clk_enable(omap->usbhost1_48m_fck);
		set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	}
	spin_unlock_irqrestore(&usb_clocks_lock, flags);

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&omap->ehci->wake_lock_ehci_rwu);
#endif

	usb_remove_hcd(hcd);
	omap_stop_ehc(omap, hcd);
	iounmap(hcd->regs);
	iounmap(omap->tll_base);
	iounmap(omap->uhh_base);
	usb_put_hcd(hcd);

	return 0;
}

static void ehci_hcd_omap_shutdown(struct platform_device *pdev)
{
	struct ehci_hcd_omap *omap = platform_get_drvdata(pdev);
	struct usb_hcd *hcd = ehci_to_hcd(omap->ehci);

	if (hcd->driver->shutdown)
		hcd->driver->shutdown(hcd);
}

static struct platform_driver ehci_hcd_omap_driver = {
	.probe			= ehci_hcd_omap_probe,
	.remove			= ehci_hcd_omap_remove,
	.shutdown		= ehci_hcd_omap_shutdown,
	/*.suspend		= ehci_hcd_omap_suspend, */
	/*.resume		= ehci_hcd_omap_resume, */
	.driver = {
		.name		= "ehci-omap",
	}
};

/*-------------------------------------------------------------------------*/
#ifdef CONFIG_PM
static int ehci_omap_bus_suspend(struct usb_hcd *hcd)
{
	int ret;
	u32 status;
	u32 sysconfig;
	unsigned long flags;
	struct ehci_hcd *ehci;
	struct ehci_hcd_omap_platform_data *pdata;
	struct ehci_hcd_omap *omap = dev_get_drvdata(hcd->self.controller);

	dev_dbg(hcd->self.controller, "%s %ld %lu\n", __func__,
		in_interrupt(), jiffies);
	ehci = hcd_to_ehci(hcd);
	pdata = omap->dev->dev.platform_data;

	/* mask interrupt 77 to avoid race condition with ehci_irq */
	/* omap_writel(0x2000, 0x482000CC); */
	disable_irq(hcd->irq);

	ret = ehci_bus_suspend(hcd);
	if (ret) {
		enable_irq(hcd->irq);
		return ret;
	}

	/* Put UHH in SmartStandby mode */
	sysconfig = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);
	sysconfig &= ~OMAP_UHH_SYSCONFIG_MIDLEMODE_MASK;
	sysconfig |= OMAP_UHH_SYSCONFIG_MIDLEMODE;
	ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG, sysconfig);

	spin_lock_irqsave(&usb_clocks_lock, flags);
	if (test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		if (pdata->usbhost_standby_status)
			ret = pdata->usbhost_standby_status();
		if (ret == 0) {
			printk(KERN_ERR "ehci: suspend failed!\n");
			ret = -EBUSY;
			goto end;
		} else
			ret = 0;
		status = ehci_readl(ehci, &ehci->regs->status);
		if (status & INTR_MASK) {
			printk(KERN_ERR "ehci: pending irq, resume!\n");
			ret = -EBUSY;
			goto end;
		}
		ehci_omap_writel(omap->tll_base, OMAP_TLL_SHARED_CONF,
			ehci_omap_readl(omap->tll_base, OMAP_TLL_SHARED_CONF) &
			~(1));
		/* Enable the interrupt so that the remote-wakeup
		 * can be detected */
		ehci_omap_writel(omap->tll_base, OMAP_USBTLL_IRQSTATUS, 7);
		ehci_omap_writel(omap->tll_base, OMAP_USBTLL_IRQENABLE, 1);

		/* Put UHH in ForceIdle mode */
		sysconfig = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);
		sysconfig &= ~OMAP_UHH_SYSCONFIG_SIDLEMODE_MASK;
		ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG, sysconfig);

		if (omap->usbhost1_48m_fck)
			clk_disable(omap->usbhost1_48m_fck);
		if (omap->usbhost2_120m_fck)
			clk_disable(omap->usbhost2_120m_fck);
		if (omap->usbtll_fck)
			clk_disable(omap->usbtll_fck);
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	}
	spin_unlock_irqrestore(&usb_clocks_lock, flags);

	return 0;

end:
	/* Put UHH in NoStandby mode */
	sysconfig = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);
	sysconfig &= ~OMAP_UHH_SYSCONFIG_MIDLEMODE_MASK;
	sysconfig |= OMAP_UHH_SYSCONFIG_NOSTBYMODE;
	ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG, sysconfig);

	spin_unlock_irqrestore(&usb_clocks_lock, flags);
	ehci_bus_resume(hcd);
	/* unmask irq 77 */
	/* omap_writel(0x2000, 0x482000C8); */
	enable_irq(hcd->irq);
	return ret;
}

static int ehci_omap_bus_resume(struct usb_hcd *hcd)
{
	u32 sysconfig;
	unsigned long flags;
	struct ehci_hcd_omap *omap = dev_get_drvdata(hcd->self.controller);

	dev_dbg(hcd->self.controller, "%s %ld %lu\n", __func__,
	in_interrupt(), jiffies);

	spin_lock_irqsave(&usb_clocks_lock, flags);
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		if (omap->usbtll_fck)
			clk_enable(omap->usbtll_fck);
		mdelay(1);
		ehci_omap_writel(omap->tll_base, OMAP_USBTLL_IRQENABLE, 0);
		ehci_omap_writel(omap->tll_base, OMAP_USBTLL_IRQSTATUS, 7);
		ehci_omap_writel(omap->tll_base, OMAP_TLL_SHARED_CONF,
			1 | ehci_omap_readl(omap->tll_base,
				OMAP_TLL_SHARED_CONF));
		if (omap->usbhost2_120m_fck)
			clk_enable(omap->usbhost2_120m_fck);
		if (omap->usbhost1_48m_fck)
			clk_enable(omap->usbhost1_48m_fck);

		/* Put UHH in NoStandby mode */
		sysconfig = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);
		sysconfig &= ~OMAP_UHH_SYSCONFIG_MIDLEMODE_MASK;
		sysconfig |= OMAP_UHH_SYSCONFIG_NOSTBYMODE;
		ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG, sysconfig);

		/* Put UHH in NoIdle mode */
		sysconfig = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);
		sysconfig &= ~OMAP_UHH_SYSCONFIG_SIDLEMODE_MASK;
		sysconfig |= OMAP_UHH_SYSCONFIG_NOIDLEMODE;
		ehci_omap_writel(omap->uhh_base, OMAP_UHH_SYSCONFIG, sysconfig);
		sysconfig = ehci_omap_readl(omap->uhh_base, OMAP_UHH_SYSCONFIG);

		set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
		enable_irq(hcd->irq);
	}
	spin_unlock_irqrestore(&usb_clocks_lock, flags);

	return ehci_bus_resume(hcd);
}

static void ehci_omap_shutdown(struct usb_hcd *hcd)
{
	unsigned long flags;
	struct ehci_hcd_omap *omap = dev_get_drvdata(hcd->self.controller);

	dev_dbg(hcd->self.controller, "%s %lu\n", __func__, jiffies);

	spin_lock_irqsave(&usb_clocks_lock, flags);
	if (!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags)) {
		if (omap->usbtll_fck)
			clk_enable(omap->usbtll_fck);
		if (omap->usbhost2_120m_fck)
			clk_enable(omap->usbhost2_120m_fck);
		if (omap->usbhost1_48m_fck)
			clk_enable(omap->usbhost1_48m_fck);

		set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	}
	spin_unlock_irqrestore(&usb_clocks_lock, flags);

	ehci_shutdown(hcd);
}

#endif

static const struct hc_driver ehci_omap_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "OMAP-EHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset			= ehci_init,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_omap_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.endpoint_reset		= ehci_endpoint_reset,

	/*
	 * scheduling support
	 */
	.get_frame_number	= ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
#ifdef CONFIG_PM
	.bus_suspend		= ehci_omap_bus_suspend,
	.bus_resume		= ehci_omap_bus_resume,
#endif
	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};

MODULE_ALIAS("platform:omap-ehci");
MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

