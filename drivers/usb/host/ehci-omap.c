/*
 * ehci-omap.c - driver for USBHOST on OMAP3/4 processors
 *
 * Bus Glue for the EHCI controllers in OMAP3/4
 * Tested on several OMAP3 boards, and OMAP4 Pandaboard
 *
 * Copyright (C) 2007-2011 Texas Instruments, Inc.
 *	Author: Vikram Pandita <vikram.pandita@ti.com>
 *	Author: Anand Gadiyar <gadiyar@ti.com>
 *	Author: Keshava Munegowda <keshava_mgowda@ti.com>
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
 * TODO (last updated Feb 27, 2010):
 *	- add kernel-doc
 *	- enable AUTOIDLE
 *	- add suspend/resume
 *	- add HSIC and TLL support
 *	- convert to use hwmod and runtime PM
 */

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/usb/ulpi.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>

#include <plat/omap_hwmod.h>
#include <plat/usb.h>
#include <plat/clock.h>
#include <plat/omap-pm.h>

/* EHCI Register Set */
#define EHCI_INSNREG04					(0xA0)
#define EHCI_INSNREG04_DISABLE_UNSUSPEND		(1 << 5)
#define	EHCI_INSNREG05_ULPI				(0xA4)
#define	EHCI_INSNREG05_ULPI_CONTROL_SHIFT		31
#define	EHCI_INSNREG05_ULPI_PORTSEL_SHIFT		24
#define	EHCI_INSNREG05_ULPI_OPSEL_SHIFT			22
#define	EHCI_INSNREG05_ULPI_REGADD_SHIFT		16
#define	EHCI_INSNREG05_ULPI_EXTREGADD_SHIFT		8
#define	EHCI_INSNREG05_ULPI_WRDATA_SHIFT		0
#define	L3INIT_HSUSBHOST_CLKCTRL			0x4A009358
#define	L3INIT_HSUSBTLL_CLKCTRL				0x4A009368

#define USB_INT_EN_RISE_CLR_0				0x4A06280F
#define USB_INT_EN_FALL_CLR_0				0x4A062812
#define USB_INT_EN_RISE_CLR_1				0x4A06290F
#define USB_INT_EN_FALL_CLR_1				0x4A062912
#define OTG_CTRL_SET_0					0x4A06280B
#define OTG_CTRL_SET_1					0x4A06290B

#define	OHCI_BASE_ADDR		0x48064400
#define	OHCI_HC_CONTROL		(OHCI_BASE_ADDR + 0x4)
#define	OHCI_HC_CTRL_SUSPEND	(3 << 6)
#define	OHCI_HC_CTRL_RESUME	(1 << 6)
/*-------------------------------------------------------------------------*/

static struct hc_driver ehci_omap_hc_driver;


static inline void ehci_write(void __iomem *base, u32 reg, u32 val)
{
	__raw_writel(val, base + reg);
}

static inline u32 ehci_read(void __iomem *base, u32 reg)
{
	return __raw_readl(base + reg);
}

/* For ISP1703 phy on OMAP4430 port 1, we need to force the PHY
 * to resume immediately after clearing PORTSC, to avoid disconnect.
 */
static void usb_phy_resume_workaround(struct ehci_hcd *ehci)
{
	int port = 1;
	int reg_id = 4; /* Function Control */
	int data = 0x40; /* MSUSPEND=1 */
	int reg;

	reg = (data & 0xFF)
		| (reg_id << EHCI_INSNREG05_ULPI_REGADD_SHIFT)
		| (2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT) /* write */
		| ((port + 1) << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT)
		| (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT);

	ehci_write(ehci->caps, EHCI_INSNREG05_ULPI, reg);

	/* in practice, we measure this register write to take effect
	 * within 1-10 usec
	 */
	udelay(20);
}

static int omap4_ehci_phy_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
) {
	struct ehci_hcd	*ehci = hcd_to_ehci(hcd);
	int		ports = HCS_N_PORTS(ehci->hcs_params);
	u32 __iomem	*status_reg = &ehci->regs->port_status[
				(wIndex & 0xff) - 1];
	u32 __iomem	*hostpc_reg = NULL;
	u32		temp, temp1, temp_reg, status;
	unsigned long	flags;
	int		retval = 0;
	unsigned	selector;

	/*
	 * FIXME:  support SetPortFeatures USB_PORT_FEAT_INDICATOR.
	 * HCS_INDICATOR may say we can change LEDs to off/amber/green.
	 * (track current state ourselves) ... blink for diagnostics,
	 * power, "this is the one", etc.  EHCI spec supports this.
	 */

	if (ehci->has_hostpc)
		hostpc_reg = (u32 __iomem *)((u8 *)ehci->regs
				+ HOSTPC0 + 4 * ((wIndex & 0xff) - 1));
	spin_lock_irqsave(&ehci->lock, flags);
	switch (typeReq) {
	case GetPortStatus:
		if (!wIndex || wIndex > ports)
			goto error;
		wIndex--;
		status = 0;
		temp = ehci_readl(ehci, status_reg);

		/* wPortChange bits */
		if (temp & PORT_CSC)
			status |= USB_PORT_STAT_C_CONNECTION << 16;
		if (temp & PORT_PEC)
			status |= USB_PORT_STAT_C_ENABLE << 16;

		if ((temp & PORT_OCC) && !ignore_oc) {
			status |= USB_PORT_STAT_C_OVERCURRENT << 16;

			/*
			 * Hubs should disable port power on over-current.
			 * However, not all EHCI implementations do this
			 * automatically, even if they _do_ support per-port
			 * power switching; they're allowed to just limit the
			 * current.  khubd will turn the power back on.
			 */
			if (HCS_PPC(ehci->hcs_params)) {
				ehci_writel(ehci,
					temp & ~(PORT_RWC_BITS | PORT_POWER),
					status_reg);
			}
		}

		/* whoever resumes must GetPortStatus to complete it!! */
		if (temp & PORT_RESUME) {

			/* Remote Wakeup received? */
			if (!ehci->reset_done[wIndex]) {
				/* resume signaling for 20 msec */
				ehci->reset_done[wIndex] = jiffies
						+ msecs_to_jiffies(20);
				/* check the port again */
				mod_timer(&ehci_to_hcd(ehci)->rh_timer,
						ehci->reset_done[wIndex]);
			}

			/* resume completed? */
			else if (time_after_eq(jiffies,
					ehci->reset_done[wIndex])) {
				clear_bit(wIndex, &ehci->suspended_ports);
				set_bit(wIndex, &ehci->port_c_suspend);
				ehci->reset_done[wIndex] = 0;

				/* stop resume signaling */
				temp = ehci_readl(ehci, status_reg);
				ehci_writel(ehci,
					temp & ~(PORT_RWC_BITS | PORT_RESUME),
					status_reg);
				usb_phy_resume_workaround(ehci);
				retval = handshake(ehci, status_reg,
					   PORT_RESUME, 0, 2000 /* 2msec */);
				if (retval != 0) {
					ehci_err(ehci,
						"port %d resume error %d\n",
						wIndex + 1, retval);
					goto error;
				}
				temp &= ~(PORT_SUSPEND|PORT_RESUME|(3<<10));
			}
		}

		/* whoever resets must GetPortStatus to complete it!! */
		if ((temp & PORT_RESET)
				&& time_after_eq(jiffies,
					ehci->reset_done[wIndex])) {
			status |= USB_PORT_STAT_C_RESET << 16;
			ehci->reset_done[wIndex] = 0;

			/* force reset to complete */
			ehci_writel(ehci, temp & ~(PORT_RWC_BITS | PORT_RESET),
					status_reg);
			/* REVISIT:  some hardware needs 550+ usec to clear
			 * this bit; seems too long to spin routinely...
			 */
			retval = handshake(ehci, status_reg,
					PORT_RESET, 0, 1000);
			if (retval != 0) {
				ehci_err(ehci, "port %d reset error %d\n",
					wIndex + 1, retval);
				goto error;
			}

			/* see what we found out */
			temp = check_reset_complete(ehci, wIndex, status_reg,
					ehci_readl(ehci, status_reg));
		}

		if (!(temp & (PORT_RESUME|PORT_RESET)))
			ehci->reset_done[wIndex] = 0;

		/* transfer dedicated ports to the companion hc */
		if ((temp & PORT_CONNECT) &&
				test_bit(wIndex, &ehci->companion_ports)) {
			temp &= ~PORT_RWC_BITS;
			temp |= PORT_OWNER;
			ehci_writel(ehci, temp, status_reg);
			ehci_dbg(ehci, "port %d --> companion\n", wIndex + 1);
			temp = ehci_readl(ehci, status_reg);
		}

		/*
		 * Even if OWNER is set, there's no harm letting khubd
		 * see the wPortStatus values (they should all be 0 except
		 * for PORT_POWER anyway).
		 */

		if (temp & PORT_CONNECT) {
			status |= USB_PORT_STAT_CONNECTION;
			/* status may be from integrated TT */
			if (ehci->has_hostpc) {
				temp1 = ehci_readl(ehci, hostpc_reg);
				status |= ehci_port_speed(ehci, temp1);
			} else
				status |= ehci_port_speed(ehci, temp);
		}
		if (temp & PORT_PE)
			status |= USB_PORT_STAT_ENABLE;

		/* maybe the port was unsuspended without our knowledge */
		if (temp & (PORT_SUSPEND|PORT_RESUME)) {
			status |= USB_PORT_STAT_SUSPEND;
		} else if (test_bit(wIndex, &ehci->suspended_ports)) {
			clear_bit(wIndex, &ehci->suspended_ports);
			ehci->reset_done[wIndex] = 0;
			if (temp & PORT_PE)
				set_bit(wIndex, &ehci->port_c_suspend);
		}

		if (temp & PORT_OC)
			status |= USB_PORT_STAT_OVERCURRENT;
		if (temp & PORT_RESET)
			status |= USB_PORT_STAT_RESET;
		if (temp & PORT_POWER)
			status |= USB_PORT_STAT_POWER;
		if (test_bit(wIndex, &ehci->port_c_suspend))
			status |= USB_PORT_STAT_C_SUSPEND << 16;

#ifndef	VERBOSE_DEBUG
	if (status & ~0xffff)	/* only if wPortChange is interesting */
#endif
		dbg_port(ehci, "GetStatus", wIndex + 1, temp);
		put_unaligned_le32(status, buf);
		break;
	case SetPortFeature:
		selector = wIndex >> 8;
		wIndex &= 0xff;
		if (unlikely(ehci->debug)) {
			/* If the debug port is active any port
			 * feature requests should get denied */
			if (wIndex == HCS_DEBUG_PORT(ehci->hcs_params) &&
			    (readl(&ehci->debug->control) & DBGP_ENABLED)) {
				retval = -ENODEV;
				goto error_exit;
			}
		}
		if (!wIndex || wIndex > ports)
			goto error;
		wIndex--;
		temp = ehci_readl(ehci, status_reg);
		if (temp & PORT_OWNER)
			break;

		temp &= ~PORT_RWC_BITS;
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			if (ehci->no_selective_suspend)
				break;
			if ((temp & PORT_PE) == 0
					|| (temp & PORT_RESET) != 0)
				goto error;

			/* After above check the port must be connected.
			 * Set appropriate bit thus could put phy into low power
			 * mode if we have hostpc feature
			 */
			temp &= ~PORT_WKCONN_E;
			temp |= PORT_WKDISC_E | PORT_WKOC_E;
			ehci_writel(ehci, temp | PORT_SUSPEND, status_reg);
			/*
			 * Special workaround sequence:
			 * - Set suspend bit
			 * - Wait 4ms for suspend to take effect
			 *   - alternatively read the line state
			 *     in PORTSC
			 * - switch to internal 60 MHz clock
			 * - wait 1ms
			 * - switch back to external clock
			 */
			mdelay(4);
			temp_reg = omap_readl(L3INIT_HSUSBHOST_CLKCTRL);
			temp_reg |= 1 << 9;
			temp_reg &= ~(1 << 25);
			omap_writel(temp_reg, L3INIT_HSUSBHOST_CLKCTRL);

			mdelay(1);
			temp_reg &= ~(1 << 9);
			temp_reg |= 1 << 25;
			omap_writel(temp_reg, L3INIT_HSUSBHOST_CLKCTRL);

			if (hostpc_reg) {
				spin_unlock_irqrestore(&ehci->lock, flags);
				msleep(5);/* 5ms for HCD enter low pwr mode */
				spin_lock_irqsave(&ehci->lock, flags);
				temp1 = ehci_readl(ehci, hostpc_reg);
				ehci_writel(ehci, temp1 | HOSTPC_PHCD,
					hostpc_reg);
				temp1 = ehci_readl(ehci, hostpc_reg);
				ehci_dbg(ehci, "Port%d phy low pwr mode %s\n",
					wIndex, (temp1 & HOSTPC_PHCD) ?
					"succeeded" : "failed");
			}
			set_bit(wIndex, &ehci->suspended_ports);
			break;
		default:
			goto error;
		}
		/* unblock posted writes */
		ehci_readl(ehci, &ehci->regs->command);
		break;

	default:
error:
		pr_err("\nWe should not get here %s\n", __func__);
		/* "stall" on error */
		retval = -EPIPE;
	}
error_exit:
	spin_unlock_irqrestore(&ehci->lock, flags);
	return retval;
}

static int omap4_ehci_update_device_disconnect(
	struct usb_hcd	*hcd,
	unsigned	port_num
) {
	struct device *dev = hcd->self.controller;
	struct ehci_hcd_omap_platform_data *pdata = dev->platform_data;
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 __iomem	*status_reg = &ehci->regs->port_status[
				port_num - 1];
	u32		temp;

	if (!(cpu_is_omap44xx()) || (port_num < 1) ||
		(port_num > OMAP3_HS_USB_PORTS) ||
		(pdata->port_mode[port_num-1] != OMAP_EHCI_PORT_MODE_TLL))
		return -ENOTSUPP;

	ehci_err(ehci, "Updating device disconnect for port %d\n", port_num);

	if (HCS_PPC(ehci->hcs_params)) {
		temp = ehci_readl(ehci, status_reg);
		ehci_writel(ehci,
			temp & ~(PORT_POWER),
			status_reg);
	}

	udelay(3);

	if (HCS_PPC(ehci->hcs_params)) {
		temp = ehci_readl(ehci, status_reg);
		ehci_writel(ehci,
			temp | PORT_POWER,
			status_reg);
	}

	ehci_err(ehci, "Port status after disconnect is 0x%08x\n",
				ehci_readl(ehci, status_reg));
	return 0;
}


static int omap4_ehci_tll_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
) {
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 __iomem	*status_reg = &ehci->regs->port_status[
				(wIndex & 0xff) - 1];
	u32 __iomem	*hostpc_reg = NULL;
	u32		temp, temp1, status;
	unsigned long	flags;
	int		retval = 0;
	u32		runstop, temp_reg;
	u32             tll_reg = (u32)OMAP2_L4_IO_ADDRESS(
					L3INIT_HSUSBTLL_CLKCTRL);

	spin_lock_irqsave(&ehci->lock, flags);
	switch (typeReq) {
	case GetPortStatus:
		wIndex--;
		status = 0;
		temp = ehci_readl(ehci, status_reg);

		/* wPortChange bits */
		if (temp & PORT_CSC)
			status |= USB_PORT_STAT_C_CONNECTION << 16;
		if (temp & PORT_PEC)
			status |= USB_PORT_STAT_C_ENABLE << 16;

		if ((temp & PORT_OCC) && !ignore_oc) {
			status |= USB_PORT_STAT_C_OVERCURRENT << 16;

			/*
			 * Hubs should disable port power on over-current.
			 * However, not all EHCI implementations do this
			 * automatically, even if they _do_ support per-port
			 * power switching; they're allowed to just limit the
			 * current.  khubd will turn the power back on.
			 */
			if (HCS_PPC(ehci->hcs_params)) {
				ehci_writel(ehci,
					temp & ~(PORT_RWC_BITS | PORT_POWER),
					status_reg);
			}
		}

		/* whoever resumes must GetPortStatus to complete it!! */
		if (temp & PORT_RESUME) {

			/* Remote Wakeup received? */
			if (!ehci->reset_done[wIndex]) {
				/* resume signaling for 20 msec */
				ehci->reset_done[wIndex] = jiffies
						+ msecs_to_jiffies(20);
				/* check the port again */
				mod_timer(&ehci_to_hcd(ehci)->rh_timer,
						ehci->reset_done[wIndex]);
			}

			/* resume completed? */
			else if (time_after_eq(jiffies,
					ehci->reset_done[wIndex])) {
				clear_bit(wIndex, &ehci->suspended_ports);
				set_bit(wIndex, &ehci->port_c_suspend);
				ehci->reset_done[wIndex] = 0;

				/*
				 * Workaround for OMAP errata:
				 * To Stop Resume Signalling, it is required
				 * to Stop the Host Controller and disable the
				 * TLL Functional Clock.
				 */

				/* Stop the Host Controller */
				runstop = ehci_readl(ehci,
							&ehci->regs->command);
				ehci_writel(ehci, (runstop & ~CMD_RUN),
							&ehci->regs->command);
				(void) ehci_readl(ehci, &ehci->regs->command);
				if (handshake(ehci, &ehci->regs->status,
							STS_HALT,
							STS_HALT,
							2000) != 0)
					ehci_err(ehci,
						"port %d would not halt!\n",
						wIndex);

				temp_reg = __raw_readl(tll_reg);
				temp_reg &= ~(1 << (wIndex + 8));


				/* stop resume signaling */
				temp = __raw_readl(status_reg) &
					~(PORT_RWC_BITS | PORT_RESUME);
				__raw_writel(temp, status_reg);

				/* Disable the Channel Optional Fclk */
				__raw_writel(temp_reg, tll_reg);
				retval = handshake(ehci, status_reg,
					   PORT_RESUME, 0, 2000 /* 2msec */);

				/*
				 * Enable the Host Controller and start the
				 * Channel Optional Fclk since resume has
				 * finished.
				 */
				udelay(3);
				omap_writel((temp_reg | (1 << (wIndex + 8))),
						L3INIT_HSUSBTLL_CLKCTRL);
				ehci_writel(ehci, (runstop),
						&ehci->regs->command);
				(void) ehci_readl(ehci, &ehci->regs->command);
				handshake(ehci, &ehci->regs->status,
							STS_HALT,
							0,
							2000);

				if (retval != 0) {
					ehci_err(ehci,
						"port %d resume error %d\n",
						wIndex + 1, retval);
					spin_unlock_irqrestore(&ehci->lock,
									flags);
					return -EPIPE;
				}

				if (!(ehci_readl(ehci, status_reg) & PORT_PE))
					ehci_err(ehci,
						"port %d disabled after resume\n",
						wIndex + 1);

				temp &= ~(PORT_SUSPEND|PORT_RESUME|(3<<10));
			}
		}

		/* whoever resets must GetPortStatus to complete it!! */
		if ((temp & PORT_RESET)
				&& time_after_eq(jiffies,
					ehci->reset_done[wIndex])) {
			status |= USB_PORT_STAT_C_RESET << 16;
			ehci->reset_done[wIndex] = 0;

			/* force reset to complete */
			ehci_writel(ehci, temp & ~(PORT_RWC_BITS | PORT_RESET),
					status_reg);
			/* REVISIT:  some hardware needs 550+ usec to clear
			 * this bit; seems too long to spin routinely...
			 */
			retval = handshake(ehci, status_reg,
					PORT_RESET, 0, 1000);
			if (retval != 0) {
				ehci_err(ehci, "port %d reset error %d\n",
					wIndex + 1, retval);
				spin_unlock_irqrestore(&ehci->lock, flags);
				return -EPIPE;
			}

			/* see what we found out */
			temp = check_reset_complete(ehci, wIndex, status_reg,
					ehci_readl(ehci, status_reg));
		}

		if (!(temp & (PORT_RESUME|PORT_RESET)))
			ehci->reset_done[wIndex] = 0;

		/* transfer dedicated ports to the companion hc */
		if ((temp & PORT_CONNECT) &&
				test_bit(wIndex, &ehci->companion_ports)) {
			temp &= ~PORT_RWC_BITS;
			temp |= PORT_OWNER;
			ehci_writel(ehci, temp, status_reg);
			ehci_dbg(ehci, "port %d --> companion\n", wIndex + 1);
			temp = ehci_readl(ehci, status_reg);
		}

		/*
		 * Even if OWNER is set, there's no harm letting khubd
		 * see the wPortStatus values (they should all be 0 except
		 * for PORT_POWER anyway).
		 */

		if (temp & PORT_CONNECT) {
			status |= USB_PORT_STAT_CONNECTION;
			/* status may be from integrated TT */
			if (ehci->has_hostpc) {
				temp1 = ehci_readl(ehci, hostpc_reg);
				status |= ehci_port_speed(ehci, temp1);
			} else
				status |= ehci_port_speed(ehci, temp);
		}
		if (temp & PORT_PE)
			status |= USB_PORT_STAT_ENABLE;

		/* maybe the port was unsuspended without our knowledge */
		if (temp & (PORT_SUSPEND|PORT_RESUME)) {
			status |= USB_PORT_STAT_SUSPEND;
		} else if (test_bit(wIndex, &ehci->suspended_ports)) {
			clear_bit(wIndex, &ehci->suspended_ports);
			ehci->reset_done[wIndex] = 0;
			if (temp & PORT_PE)
				set_bit(wIndex, &ehci->port_c_suspend);
		}

		if (temp & PORT_OC)
			status |= USB_PORT_STAT_OVERCURRENT;
		if (temp & PORT_RESET)
			status |= USB_PORT_STAT_RESET;
		if (temp & PORT_POWER)
			status |= USB_PORT_STAT_POWER;
		if (test_bit(wIndex, &ehci->port_c_suspend))
			status |= USB_PORT_STAT_C_SUSPEND << 16;

#ifndef	VERBOSE_DEBUG
	if (status & ~0xffff)	/* only if wPortChange is interesting */
#endif
		dbg_port(ehci, "GetStatus", wIndex + 1, temp);
		put_unaligned_le32(status, buf);
		break;
	default:
		spin_unlock_irqrestore(&ehci->lock, flags);
		return ehci_hub_control(hcd, typeReq, wValue, wIndex,
								buf, wLength);
	}
	spin_unlock_irqrestore(&ehci->lock, flags);
	return retval;
}

static int omap_ehci_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
) {
	struct device *dev = hcd->self.controller;
	struct ehci_hcd_omap_platform_data	*pdata = dev->platform_data;

	if (cpu_is_omap44xx() && (omap_rev() <= OMAP4430_REV_ES2_2) &&
		(wIndex > 0) && (wIndex < OMAP3_HS_USB_PORTS)) {
		if (pdata->port_mode[wIndex-1] == OMAP_EHCI_PORT_MODE_TLL)
			return omap4_ehci_tll_hub_control(hcd, typeReq, wValue,
						wIndex, buf, wLength);
	}

	/*
	 * This is an initial version of a SW workaround for
	 * an OMAP4 errata which causes suspend-resume to fail.
	 * This is needed on at least one particular USB PHY,
	 * however more testing is needed before making this
	 * code unconditional. Hence limit it to the one case
	 * where we use a PHY on Port2.
	 * FIXME: combine this wrapper with the previous one
	 * for TLL mode
	 */

	if (cpu_is_omap44xx() && (wIndex == 2) &&
	    (pdata->port_mode[wIndex-1] == OMAP_EHCI_PORT_MODE_PHY)) {
		if ((typeReq == SetPortFeature) &&
				(wValue == USB_PORT_FEAT_SUSPEND)) {
			pr_debug("OMAP EHCI WAR- missing clocks at suspend\n");
			pr_debug("wIndex 0x%x, typeReq 0x%x, wValue 0x%x\n",
					wIndex, typeReq, wValue);
			return omap4_ehci_phy_hub_control(hcd, typeReq, wValue,
						wIndex, buf, wLength);
		} else if (typeReq == GetPortStatus) {
			pr_debug("OMAP EHCI WAR- Slow end-resume sequence\n");
			pr_debug("wIndex 0x%x, typeReq 0x%x, wValue 0x%x\n",
					wIndex, typeReq, wValue);
			return omap4_ehci_phy_hub_control(hcd, typeReq, wValue,
						wIndex, buf, wLength);
		}
	}

	return ehci_hub_control(hcd, typeReq, wValue, wIndex, buf, wLength);
}

u8 omap_ehci_ulpi_read(const struct usb_hcd *hcd, u8 reg)
{
	unsigned reg_internal = 0;
	u8 val;
	int count = 2000;

	reg_internal = ((reg) << EHCI_INSNREG05_ULPI_REGADD_SHIFT)
			/* Write */
			| (3 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT)
			/* PORTn */
			| ((1) << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT)
			/* start ULPI access*/
			| (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT);

	ehci_write(hcd->regs, EHCI_INSNREG05_ULPI, reg_internal);

	/* Wait for ULPI access completion */
	while ((ehci_read(hcd->regs, EHCI_INSNREG05_ULPI)
			& (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT))) {
		udelay(1);
		if (count-- == 0) {
			pr_err("ehci: omap_ehci_ulpi_read: Error");
			break;
		}
	}

	val = ehci_read(hcd->regs, EHCI_INSNREG05_ULPI) & 0xFF;
	return val;
}

int omap_ehci_ulpi_write(const struct usb_hcd *hcd, u8 val, u8 reg, u8 retry_times)
{
	unsigned reg_internal = 0;
	int status = 0;
	int count;

again:
	count = 2000;
	reg_internal = val |
			((reg) << EHCI_INSNREG05_ULPI_REGADD_SHIFT)
			/* Write */
			| (2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT)
			/* PORTn */
			| ((1) << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT)
			/* start ULPI access*/
			| (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT);

	ehci_write(hcd->regs, EHCI_INSNREG05_ULPI, reg_internal);

	/* Wait for ULPI access completion */
	while ((ehci_read(hcd->regs, EHCI_INSNREG05_ULPI)
			& (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT))) {
		udelay(1);
		if (count-- == 0) {
			if (retry_times--) {
				ehci_write(hcd->regs, EHCI_INSNREG05_ULPI, 0);
				goto again;
			} else {
				pr_err("ehci: omap_ehci_ulpi_write: Error");
				status = -ETIMEDOUT;
				break;
			}
		}
	}
	return status;
}
#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
static void omap_ehci_soft_phy_reset(struct platform_device *pdev, u8 port)
{
	struct usb_hcd	*hcd = dev_get_drvdata(&pdev->dev);
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	unsigned reg = 0;

	reg = ULPI_FUNC_CTRL_RESET
		/* FUNCTION_CTRL_SET register */
		| (ULPI_SET(ULPI_FUNC_CTRL) << EHCI_INSNREG05_ULPI_REGADD_SHIFT)
		/* Write */
		| (2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT)
		/* PORTn */
		| ((port + 1) << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT)
		/* start ULPI access*/
		| (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT);

	ehci_write(hcd->regs, EHCI_INSNREG05_ULPI, reg);

	/* Wait for ULPI access completion */
	while ((ehci_read(hcd->regs, EHCI_INSNREG05_ULPI)
			& (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT))) {
		cpu_relax();

		if (time_after(jiffies, timeout)) {
			dev_dbg(&pdev->dev, "phy reset operation timed out\n");
			break;
		}
	}
}
#endif
static void omap_ehci_intr_vbus_valid_clear(struct platform_device *pdev,
									u8 port)
{
	struct usb_hcd	*hcd = dev_get_drvdata(&pdev->dev);
	unsigned long timeout;
	unsigned reg = 0;

	reg = ULPI_INT_VBUS_VALID
	| (ULPI_CLR(ULPI_USB_INT_EN_RISE) << EHCI_INSNREG05_ULPI_REGADD_SHIFT)
	| (2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT) /* Write */
	| ((port + 1) << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT)
	| (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT); /* start ULPI access*/

	ehci_write(hcd->regs, EHCI_INSNREG05_ULPI, reg);

	timeout = jiffies + msecs_to_jiffies(1000);
	/* Wait for ULPI access completion */
	while ((ehci_read(hcd->regs, EHCI_INSNREG05_ULPI)
			& (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT))) {
		cpu_relax();

		if (time_after(jiffies, timeout)) {
			dev_err(&pdev->dev, "Vbus valid rise clear timed out\n");
			return;
		}
	}

	reg = ULPI_INT_VBUS_VALID
	| (ULPI_CLR(ULPI_USB_INT_EN_FALL) << EHCI_INSNREG05_ULPI_REGADD_SHIFT)
	| (2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT) /* Write */
	| ((port + 1) << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT)
	| (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT); /* start ULPI access*/

	ehci_write(hcd->regs, EHCI_INSNREG05_ULPI, reg);

	timeout = jiffies + msecs_to_jiffies(1000);
	/* Wait for ULPI access completion */
	while ((ehci_read(hcd->regs, EHCI_INSNREG05_ULPI)
			& (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT))) {
		cpu_relax();

		if (time_after(jiffies, timeout)) {
			dev_err(&pdev->dev, "Vbus valid fall clear timed out\n");
			break;
		}
	}
}

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
	struct device				*dev = &pdev->dev;
	struct ehci_hcd_omap_platform_data	*pdata = dev->platform_data;
	struct resource				*res;
	struct usb_hcd				*hcd;
	void __iomem				*regs;
	struct ehci_hcd				*omap_ehci;
	struct usb_device			*udev;
	int					ret = -ENODEV;
	int					irq;
	int					i;
	char					supply[7];

	if (usb_disabled())
		return -ENODEV;

	if (!dev->parent) {
		dev_err(dev, "Missing parent device\n");
		return -ENODEV;
	}

	irq = platform_get_irq_byname(pdev, "ehci-irq");
	if (irq < 0) {
		dev_err(dev, "EHCI irq failed\n");
		return -ENODEV;
	}

	res =  platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "ehci");
	if (!res) {
		dev_err(dev, "UHH EHCI get resource failed\n");
		return -ENODEV;
	}

	regs = ioremap(res->start, resource_size(res));
	if (!regs) {
		dev_err(dev, "UHH EHCI ioremap failed\n");
		return -ENOMEM;
	}

	ehci_omap_hc_driver.hub_control	= omap_ehci_hub_control;

	hcd = usb_create_hcd(&ehci_omap_hc_driver, dev,
			dev_name(dev));
	if (!hcd) {
		dev_err(dev, "failed to create hcd with err %d\n", ret);
		ret = -ENOMEM;
		goto err_io;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = regs;

	/* get ehci regulator and enable */
	for (i = 0 ; i < OMAP3_HS_USB_PORTS ; i++) {
		if (pdata->port_mode[i] != OMAP_EHCI_PORT_MODE_PHY) {
			pdata->regulator[i] = NULL;
			continue;
		}
		snprintf(supply, sizeof(supply), "hsusb%d", i);
		pdata->regulator[i] = regulator_get(dev, supply);
		if (IS_ERR(pdata->regulator[i])) {
			pdata->regulator[i] = NULL;
			dev_dbg(dev,
			"failed to get ehci port%d regulator\n", i);
		} else {
			regulator_enable(pdata->regulator[i]);
		}
	}

	pm_runtime_get_sync(dev->parent);

	/*
	 * An undocumented "feature" in the OMAP3 EHCI controller,
	 * causes suspended ports to be taken out of suspend when
	 * the USBCMD.Run/Stop bit is cleared (for example when
	 * we do ehci_bus_suspend).
	 * This breaks suspend-resume if the root-hub is allowed
	 * to suspend. Writing 1 to this undocumented register bit
	 * disables this feature and restores normal behavior.
	 */
	ehci_write(regs, EHCI_INSNREG04,
				EHCI_INSNREG04_DISABLE_UNSUSPEND);


#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	/* Soft reset the PHY using PHY reset command over ULPI */
	if (pdata->port_mode[0] == OMAP_EHCI_PORT_MODE_PHY)
		omap_ehci_soft_phy_reset(pdev, 0);
	if (pdata->port_mode[1] == OMAP_EHCI_PORT_MODE_PHY)
		omap_ehci_soft_phy_reset(pdev, 1);
#endif
	if (cpu_is_omap44xx()) {
		/*
		 * Undocumented HW Errata for OMAP4 TLL
		 * the IDGND ULPI bits generate an interrupt when the
		 * controller is enabled from OFF mode. This causes the
		 * port status to be reported as disabled. Mask these
		 * interrupts since TLL does not require them.
		 */
		if (pdata->port_mode[0] == OMAP_EHCI_PORT_MODE_TLL) {
			omap_writeb(0x10, USB_INT_EN_RISE_CLR_0);
			omap_writeb(0x10, USB_INT_EN_FALL_CLR_0);
			omap_writeb(0x01, OTG_CTRL_SET_0);
		}
		if (pdata->port_mode[1] == OMAP_EHCI_PORT_MODE_TLL) {
			omap_writeb(0x10, USB_INT_EN_RISE_CLR_1);
			omap_writeb(0x10, USB_INT_EN_FALL_CLR_1);
			omap_writeb(0x01, OTG_CTRL_SET_1);
		}

		/*
		 * If Vbus pin of the PHY is not in use, Vbus valid rising and
		 * falling interrupt can be disabled over ULPI.
		 * Current consumption in low power mode can be reduced by
		 * disabling interrupt because Vbus valid detector is disabled.
		 */
		if (pdata->ehci_phy_vbus_not_used) {
			if (pdata->port_mode[0] == OMAP_EHCI_PORT_MODE_PHY)
				omap_ehci_intr_vbus_valid_clear(pdev, 0);
			if (pdata->port_mode[1] == OMAP_EHCI_PORT_MODE_PHY)
				omap_ehci_intr_vbus_valid_clear(pdev, 1);
		}
	}
	omap_ehci = hcd_to_ehci(hcd);
	omap_ehci->sbrn = 0x20;

	omap_ehci->no_companion_port_handoff = 1;
	/* we know this is the memory we want, no need to ioremap again */
	omap_ehci->caps = hcd->regs;
	omap_ehci->regs = hcd->regs
		+ HC_LENGTH(ehci, readl(&omap_ehci->caps->hc_capbase));

#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	dbg_hcs_params(omap_ehci, "reset");
	dbg_hcc_params(omap_ehci, "reset");
#endif

	/* cache this readonly data; minimize chip reads */
	omap_ehci->hcs_params = readl(&omap_ehci->caps->hcs_params);

	ret = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (ret) {
		dev_err(dev, "failed to add hcd with err %d\n", ret);
		goto err_add_hcd;
	}

	/*
	 * Due to Silicon Errata on this OMAP revision, the devices attached
	 * should not resume frequently. Set the Lazy Resume flag in the
	 * root hub.
	 */
	if (cpu_is_omap44xx() && (omap_rev() < OMAP4430_REV_ES2_3)) {
		udev =  hcd->self.root_hub;
		if (udev)
			udev->lazy_resume = 1;
	}

#ifdef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	/* We need to suspend OHCI in order for the usbhost
	 * domain to go standby.
	 * OHCI would never be resumed for UMTS modem */
	omap_writel(OHCI_HC_CTRL_SUSPEND, OHCI_HC_CONTROL);

	/* Refer ISSUE2: LINK assumes external charge pump */
	/* use Port1 VBUS to charge externally Port2:
	 *      So for PHY mode operation use Port2 only */
	ehci_write(hcd->regs, EHCI_INSNREG05_ULPI,
		(0xA << EHCI_INSNREG05_ULPI_REGADD_SHIFT) |/* OTG ctrl reg*/
		(2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT) |/*   Write */
		(2 << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT) |/* Port1 */
		(1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT) |/* Start */
		(0x26));
	while (!(ehci_read(hcd->regs, EHCI_INSNREG05_ULPI) &
		(1<<EHCI_INSNREG05_ULPI_CONTROL_SHIFT))) {
		cpu_relax();
	}
#endif

	/* root ports should always stay powered */
	ehci_port_power(omap_ehci, 1);

	return 0;

err_add_hcd:
	pm_runtime_put_sync(dev->parent);

err_io:
	return ret;
}


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
	struct device *dev	= &pdev->dev;
	struct usb_hcd *hcd	= dev_get_drvdata(dev);

	usb_remove_hcd(hcd);
	pm_runtime_put_sync(dev->parent);
	usb_put_hcd(hcd);
	return 0;
}

static void ehci_hcd_omap_shutdown(struct platform_device *pdev)
{
	struct device *dev	= &pdev->dev;
	struct usb_hcd *hcd = dev_get_drvdata(&pdev->dev);

	if (hcd->driver->shutdown) {
		pm_runtime_get_sync(dev->parent);
		hcd->driver->shutdown(hcd);
		pm_runtime_put(dev->parent);
	}
}

static int ehci_omap_bus_suspend(struct usb_hcd *hcd)
{
	struct device *dev = hcd->self.controller;
	struct ehci_hcd_omap_platform_data  *pdata;
#ifndef CONFIG_USB_OOBWAKE
	struct omap_hwmod	*oh;
#endif
	struct clk *clk;
	int ret = 0;
	int i;

	dev_dbg(dev, "ehci_omap_bus_suspend\n");

	ret = ehci_bus_suspend(hcd);

	if (hcd->self.connection_change) {
		dev_err(dev, "Connection state changed\n");
		//save_usb_sar_regs(); //this may be omap4 only, hack it out for now
		hcd->self.connection_change = 0;
	}

	if (ret != 0) {
		dev_dbg(dev, "ehci_omap_bus_suspend failed %d\n", ret);
		return ret;
	}

	disable_irq(hcd->irq);
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

#ifndef CONFIG_USB_OOBWAKE
	oh = omap_hwmod_lookup(USBHS_EHCI_HWMODNAME);

	if (oh)
		omap_hwmod_enable_ioring_wakeup(oh);
#endif

	if (dev->parent)
		pm_runtime_put_sync(dev->parent);

	/* At the end, disable any external transceiver clocks */
	pdata = dev->platform_data;
	for (i = 0 ; i < OMAP3_HS_USB_PORTS ; i++) {
		clk = pdata->transceiver_clk[i];
		if (clk)
			clk_disable(clk);
	}

	omap_pm_set_min_bus_tput(dev,
			OCP_INITIATOR_AGENT,
			-1);

	return ret;
}

static int ehci_omap_bus_resume(struct usb_hcd *hcd)
{
	struct device *dev = hcd->self.controller;
	struct ehci_hcd_omap_platform_data  *pdata;
	struct clk *clk;
	int i;

	dev_dbg(dev, "ehci_omap_bus_resume\n");

	/* Re-enable any external transceiver clocks first */
	pdata = dev->platform_data;
	for (i = 0 ; i < OMAP3_HS_USB_PORTS ; i++) {
		clk = pdata->transceiver_clk[i];
		if (clk)
			clk_enable(clk);
	}

	omap_pm_set_min_bus_tput(dev,
			OCP_INITIATOR_AGENT,
			(200*1000*4));

	if (dev->parent) {
		pm_runtime_get_sync(dev->parent);
	}

	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	enable_irq(hcd->irq);

	return ehci_bus_resume(hcd);
}

static struct platform_driver ehci_hcd_omap_driver = {
	.probe			= ehci_hcd_omap_probe,
	.remove			= ehci_hcd_omap_remove,
	.shutdown		= ehci_hcd_omap_shutdown,
	.driver = {
		.name		= "ehci-omap",
	}
};

/*-------------------------------------------------------------------------*/

static struct hc_driver ehci_omap_hc_driver = {
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
	.shutdown		= ehci_shutdown,

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
	.bus_suspend		= ehci_omap_bus_suspend,
	.bus_resume		= ehci_omap_bus_resume,

	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
	.update_device_disconnect = omap4_ehci_update_device_disconnect,
};

MODULE_ALIAS("platform:omap-ehci");
MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

