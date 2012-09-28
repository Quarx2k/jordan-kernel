/*
 * OMAP3 Power Management Routines
 *
 * Copyright (C) 2006-2008 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 * Jouni Hogander
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/reboot.h>

#include <plat/sram.h>
#include <plat/clockdomain.h>
#include <plat/powerdomain.h>
#include <plat/control.h>
#include <plat/serial.h>
#include <plat/sdrc.h>
#include <plat/prcm.h>
#include <plat/gpmc.h>
#include <plat/dma.h>
#include <plat/dmtimer.h>
#include <plat/usb.h>
#include <plat/omap-pm.h>

#include <plat/resource.h>
#include <trace/pm.h>

#include <asm/tlbflush.h>

#include <linux/delay.h>

#include <asm/trace-clock.h>

#include "cm.h"
#include "cm-regbits-34xx.h"
#include "prm-regbits-34xx.h"

#include "smartreflex.h"
#include "prm.h"
#include "pm.h"
#include "sdrc.h"

static int regset_save_on_suspend;

/* Scratchpad offsets */
#define OMAP343X_TABLE_ADDRESS_OFFSET	   0x31
#define OMAP343X_TABLE_VALUE_OFFSET	   0x30
#define OMAP343X_CONTROL_REG_VALUE_OFFSET  0x32
#define ABB_TRANXDONE_TIMEOUT	30
#define ABB_FAST_OPP		1
#define ABB_NOMINAL_OPP		2

u32 enable_off_mode;
u32 sleep_while_idle;
u32 wakeup_timer_seconds;
u32 voltage_off_while_idle;
u32 enable_abb_mode;

struct power_state {
	struct powerdomain *pwrdm;
	u32 next_state;
#ifdef CONFIG_SUSPEND
	u32 saved_state;
#endif
	struct list_head node;
};

DEFINE_TRACE(pm_idle_entry);
DEFINE_TRACE(pm_idle_exit);
DEFINE_TRACE(pm_suspend_entry);
DEFINE_TRACE(pm_suspend_exit);

static LIST_HEAD(pwrst_list);

static void (*_omap_sram_idle)(u32 *addr, int save_state);

static int (*_omap_save_secure_sram)(u32 *addr);

static struct powerdomain *mpu_pwrdm, *neon_pwrdm;
static struct powerdomain *core_pwrdm, *per_pwrdm;
static struct powerdomain *wkup_pwrdm;
static struct powerdomain *cam_pwrdm;

static struct prm_setup_vc prm_setup = {
	.clksetup = 0xff,
	.voltsetup_time1 = 0xfff,
	.voltsetup_time2 = 0xfff,
	.voltoffset = 0xff,
	.voltsetup2 = 0xff,
	.vdd0_on = 0x30,	/* 1.2v */
	.vdd0_onlp = 0x20,	/* 1.0v */
	.vdd0_ret = 0x1e,	/* 0.975v */
	.vdd0_off = 0x00,	/* 0.6v */
	.vdd1_on = 0x2c,	/* 1.15v */
	.vdd1_onlp = 0x20,	/* 1.0v */
	.vdd1_ret = 0x1e,	/* .975v */
	.vdd1_off = 0x00,	/* 0.6v */
	.i2c_slave_ra = (R_SRI2C_SLAVE_ADDR << OMAP3430_SMPS_SA1_SHIFT) |
			(R_SRI2C_SLAVE_ADDR << OMAP3430_SMPS_SA0_SHIFT),
	.vdd_vol_ra = (R_VDD2_SR_CONTROL << OMAP3430_VOLRA1_SHIFT) |
			(R_VDD1_SR_CONTROL << OMAP3430_VOLRA0_SHIFT),
	/* vdd_vol_ra controls both cmd and vol, set the address equal */
	.vdd_cmd_ra = (R_VDD2_SR_CONTROL << OMAP3430_VOLRA1_SHIFT) |
			(R_VDD1_SR_CONTROL << OMAP3430_VOLRA0_SHIFT),
	.vdd_ch_conf = OMAP3430_CMD1 | OMAP3430_RAV1,
	.vdd_i2c_cfg = OMAP3430_MCODE_SHIFT | OMAP3430_HSEN | OMAP3430_SREN,
};

static inline void omap3_per_save_context(void)
{
	omap_gpio_save_context();
}

static inline void omap3_per_restore_context(void)
{
	omap_gpio_restore_context();
}

static void omap3_enable_io_chain(void)
{
	int timeout = 0;

	if (omap_rev() >= OMAP3430_REV_ES3_1) {
		prm_set_mod_reg_bits(OMAP3430_EN_IO_CHAIN, WKUP_MOD, PM_WKEN);
		/* Do a readback to assure write has been done */
		prm_read_mod_reg(WKUP_MOD, PM_WKEN);

		while (!(prm_read_mod_reg(WKUP_MOD, PM_WKST) &
			 OMAP3430_ST_IO_CHAIN)) {
			timeout++;
			if (timeout > 1000) {
				printk(KERN_ERR "Wake up daisy chain "
				       "activation failed.\n");
				return;
			}
		}
		prm_rmw_mod_reg_bits(0xFFFFFFFF, OMAP3430_ST_IO_CHAIN,
				     WKUP_MOD, PM_WKST);
	}
}

static void omap3_disable_io_chain(void)
{
	if (omap_rev() >= OMAP3430_REV_ES3_1)
		prm_clear_mod_reg_bits(OMAP3430_EN_IO_CHAIN, WKUP_MOD, PM_WKEN);
}

static void omap3_core_save_context(void)
{
	u32 control_padconf_off;

	/* Save the padconf registers */
	control_padconf_off = omap_ctrl_readl(OMAP343X_CONTROL_PADCONF_OFF);
	control_padconf_off |= START_PADCONF_SAVE;
	omap_ctrl_writel(control_padconf_off, OMAP343X_CONTROL_PADCONF_OFF);
	/* wait for the save to complete */
	while (!(omap_ctrl_readl(OMAP343X_CONTROL_GENERAL_PURPOSE_STATUS)
			& PADCONF_SAVE_DONE))
		udelay(1);

	/*
	 * Force write last pad into memory, as this can fail in some
	 * cases according to erratas 1.157, 1.185
	 */
	omap_ctrl_writel(omap_ctrl_readl(OMAP343X_PADCONF_ETK_D14),
		OMAP343X_CONTROL_MEM_WKUP + 0x2a0);

	/* Save the Interrupt controller context */
	omap_intc_save_context();
	/* Save the GPMC context */
	omap3_gpmc_save_context();
	/* Save the system control module context, padconf already save above*/
	omap3_control_save_context();
	omap_dma_global_context_save();
}

static void omap3_core_restore_context(void)
{
	/* Restore the control module context, padconf restored by h/w */
	omap3_control_restore_context();
	/* Restore the GPMC context */
	omap3_gpmc_restore_context();
	/* Restore the interrupt controller context */
	omap_intc_restore_context();
	omap_dma_global_context_restore();
}

/*
 * FIXME: This function should be called before entering off-mode after
 * OMAP3 secure services have been accessed. Currently it is only called
 * once during boot sequence, but this works as we are not using secure
 * services.
 */
static void omap3_save_secure_ram_context(u32 target_mpu_state)
{
	u32 ret;

	if (omap_type() != OMAP2_DEVICE_TYPE_GP) {
		/*
		 * MPU next state must be set to POWER_ON temporarily,
		 * otherwise the WFI executed inside the ROM code
		 * will hang the system.
		 */
		pwrdm_set_next_pwrst(mpu_pwrdm, PWRDM_POWER_ON);
		ret = _omap_save_secure_sram((u32 *)
				__pa(omap3_secure_ram_storage));
		pwrdm_set_next_pwrst(mpu_pwrdm, target_mpu_state);
		/* Following is for error tracking, it should not happen */
		if (ret) {
			printk(KERN_ERR "save_secure_sram() returns %08x\n",
				ret);
			while (1)
				;
		}
	}
}

/*
 * PRCM Interrupt Handler Helper Function
 *
 * The purpose of this function is to clear any wake-up events latched
 * in the PRCM PM_WKST_x registers. It is possible that a wake-up event
 * may occur whilst attempting to clear a PM_WKST_x register and thus
 * set another bit in this register. A while loop is used to ensure
 * that any peripheral wake-up events occurring while attempting to
 * clear the PM_WKST_x are detected and cleared.
 */
static int prcm_clear_mod_irqs(s16 module, u8 regs)
{
	u32 wkst, fclk, iclk, clken;
	u16 wkst_off = (regs == 3) ? OMAP3430ES2_PM_WKST3 : PM_WKST1;
	u16 fclk_off = (regs == 3) ? OMAP3430ES2_CM_FCLKEN3 : CM_FCLKEN1;
	u16 iclk_off = (regs == 3) ? CM_ICLKEN3 : CM_ICLKEN1;
	u16 grpsel_off = (regs == 3) ?
		OMAP3430ES2_PM_MPUGRPSEL3 : OMAP3430_PM_MPUGRPSEL;
	int c = 0;

	wkst = prm_read_mod_reg(module, wkst_off);
	wkst &= prm_read_mod_reg(module, grpsel_off);
	if (wkst) {
		iclk = cm_read_mod_reg(module, iclk_off);
		fclk = cm_read_mod_reg(module, fclk_off);
		while (wkst) {
			clken = wkst;
			if (module != OMAP3430ES2_USBHOST_MOD)
				cm_set_mod_reg_bits(clken, module, iclk_off);
			/*
			 * For USBHOST, we don't know whether HOST1 or
			 * HOST2 woke us up, so enable both f-clocks
			 */
			if (module == OMAP3430ES2_USBHOST_MOD)
				clken |= 1 << OMAP3430ES2_EN_USBHOST2_SHIFT;
			cm_set_mod_reg_bits(clken, module, fclk_off);
			prm_write_mod_reg(wkst, module, wkst_off);
			wkst = prm_read_mod_reg(module, wkst_off);
			c++;
		}
		if (module != OMAP3430ES2_USBHOST_MOD) {
			cm_write_mod_reg(iclk, module, iclk_off);
			cm_write_mod_reg(fclk, module, fclk_off);
		}
	}

	return c;
}

static int _prcm_int_handle_wakeup(void)
{
	int c;

	c = prcm_clear_mod_irqs(WKUP_MOD, 1);
	c += prcm_clear_mod_irqs(CORE_MOD, 1);
	c += prcm_clear_mod_irqs(OMAP3430_PER_MOD, 1);
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		c += prcm_clear_mod_irqs(CORE_MOD, 3);
		c += prcm_clear_mod_irqs(OMAP3430ES2_USBHOST_MOD, 1);
	}

	return c;
}

/*
 * PRCM Interrupt Handler
 *
 * The PRM_IRQSTATUS_MPU register indicates if there are any pending
 * interrupts from the PRCM for the MPU. These bits must be cleared in
 * order to clear the PRCM interrupt. The PRCM interrupt handler is
 * implemented to simply clear the PRM_IRQSTATUS_MPU in order to clear
 * the PRCM interrupt. Please note that bit 0 of the PRM_IRQSTATUS_MPU
 * register indicates that a wake-up event is pending for the MPU and
 * this bit can only be cleared if the all the wake-up events latched
 * in the various PM_WKST_x registers have been cleared. The interrupt
 * handler is implemented using a do-while loop so that if a wake-up
 * event occurred during the processing of the prcm interrupt handler
 * (setting a bit in the corresponding PM_WKST_x register and thus
 * preventing us from clearing bit 0 of the PRM_IRQSTATUS_MPU register)
 * this would be handled.
 */
static irqreturn_t prcm_interrupt_handler (int irq, void *dev_id)
{
	u32 irqstatus_mpu;
	u32 irqenable;
	int c = 0;

	do {
		irqenable = prm_read_mod_reg(OCP_MOD,
				OMAP3_PRM_IRQENABLE_MPU_OFFSET);
		irqstatus_mpu = prm_read_mod_reg(OCP_MOD,
				OMAP3_PRM_IRQSTATUS_MPU_OFFSET);

		if (irqstatus_mpu & (OMAP3430_WKUP_ST | OMAP3430_IO_ST)) {
			c = _prcm_int_handle_wakeup();
			/*
			 * Is the MPU PRCM interrupt handler racing with the
			 * IVA2 PRCM interrupt handler ?
			 */
			WARN(c == 0, "prcm: WARNING: PRCM indicated MPU wakeup "
					"but no wakeup sources are marked\n");
		} else {
			/* XXX we need to expand our PRCM interrupt handler */
			WARN(1, "prcm: WARNING: PRCM interrupt received, but "
				"no code to handle it (%08x)\n", irqstatus_mpu);
		}

		irqenable = prm_read_mod_reg(OCP_MOD,
				OMAP3_PRM_IRQENABLE_MPU_OFFSET);
		irqstatus_mpu = prm_read_mod_reg(OCP_MOD,
				OMAP3_PRM_IRQSTATUS_MPU_OFFSET);

		prm_write_mod_reg((irqstatus_mpu & irqenable), OCP_MOD,
				OMAP3_PRM_IRQSTATUS_MPU_OFFSET);

	} while (prm_read_mod_reg(OCP_MOD, OMAP3_PRM_IRQSTATUS_MPU_OFFSET)
			& irqenable);

	return IRQ_HANDLED;
}

static void restore_control_register(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c1, c0, 0" : : "r" (val));
}

static inline void omap3_save_neon_context(void)
{
#ifdef CONFIG_VFP
	vfp_pm_save_context();
#endif
}

/* Function to restore the table entry that was modified for enabling MMU */
static void restore_table_entry(void)
{
	u32 *scratchpad_address;
	u32 previous_value, control_reg_value;
	u32 *address;

	scratchpad_address = OMAP2_L4_IO_ADDRESS(OMAP343X_SCRATCHPAD);

	/* Get address of entry that was modified */
	address = (u32 *)__raw_readl(scratchpad_address +
				     OMAP343X_TABLE_ADDRESS_OFFSET);
	/* Get the previous value which needs to be restored */
	previous_value = __raw_readl(scratchpad_address +
				     OMAP343X_TABLE_VALUE_OFFSET);
	address = __va(address);
	*address = previous_value;
	flush_tlb_all();
	control_reg_value = __raw_readl(scratchpad_address
					+ OMAP343X_CONTROL_REG_VALUE_OFFSET);
	/* This will enable caches and prediction */
	restore_control_register(control_reg_value);
}

void omap_sram_idle(void)
{
	/* Variable to tell what needs to be saved and restored
	 * in omap_sram_idle*/
	/* save_state = 0 => Nothing to save and restored */
	/* save_state = 1 => Only L1 and logic lost */
	/* save_state = 2 => Only L2 lost */
	/* save_state = 3 => L1, L2 and logic lost */
	int save_state = 0;
	int mpu_next_state = PWRDM_POWER_ON;
	int neon_next_state = PWRDM_POWER_ON;
	int per_next_state = PWRDM_POWER_ON;
	int core_next_state = PWRDM_POWER_ON;
	int core_prev_state, per_prev_state;
	u32 sdrc_pwr = 0;
	int per_state_modified = 0;

	if (!_omap_sram_idle)
		return;

	pwrdm_clear_all_prev_pwrst(mpu_pwrdm);
	pwrdm_clear_all_prev_pwrst(neon_pwrdm);
	pwrdm_clear_all_prev_pwrst(core_pwrdm);
	pwrdm_clear_all_prev_pwrst(per_pwrdm);

	mpu_next_state = pwrdm_read_next_pwrst(mpu_pwrdm);
	switch (mpu_next_state) {
	case PWRDM_POWER_ON:
	case PWRDM_POWER_RET:
		/* No need to save context */
		save_state = 0;
		break;
	case PWRDM_POWER_OFF:
		save_state = 3;
		break;
	default:
		/* Invalid state */
		printk(KERN_ERR "Invalid mpu state in sram_idle\n");
		return;
	}

	pwrdm_pre_transition();

	/* NEON control */
	if (pwrdm_read_pwrst(neon_pwrdm) == PWRDM_POWER_ON) {
		pwrdm_set_next_pwrst(neon_pwrdm, mpu_next_state);
		neon_next_state = mpu_next_state;
		if (neon_next_state == PWRDM_POWER_OFF)
			omap3_save_neon_context();
	}

	/* Enable IO-PAD and IO-CHAIN wakeups */
	per_next_state = pwrdm_read_next_pwrst(per_pwrdm);
	core_next_state = pwrdm_read_next_pwrst(core_pwrdm);
#ifdef CONFIG_PM_DEEPSLEEP
	if ((per_next_state < PWRDM_POWER_ON ||
		core_next_state < PWRDM_POWER_ON) && !get_deepsleep_mode()) {
		prm_set_mod_reg_bits(OMAP3430_EN_IO, WKUP_MOD, PM_WKEN);
		omap3_enable_io_chain();
	}
#else
	if (per_next_state < PWRDM_POWER_ON ||
			core_next_state < PWRDM_POWER_ON) {
		prm_set_mod_reg_bits(OMAP3430_EN_IO, WKUP_MOD, PM_WKEN);
		omap3_enable_io_chain();
	}
#endif
	/* PER */
	if (per_next_state < PWRDM_POWER_ON) {
		omap_uart_prepare_idle(2);
		omap2_gpio_prepare_for_idle(per_next_state);
		if (per_next_state == PWRDM_POWER_OFF) {
			if (core_next_state == PWRDM_POWER_ON) {
				per_next_state = PWRDM_POWER_RET;
				pwrdm_set_next_pwrst(per_pwrdm, per_next_state);
				per_state_modified = 1;
			} else
				omap3_per_save_context();
		}
	}

	if (pwrdm_read_pwrst(cam_pwrdm) == PWRDM_POWER_ON)
		omap2_clkdm_deny_idle(mpu_pwrdm->pwrdm_clkdms[0]);

	/*
	 * Disable smartreflex before entering WFI.
	 * Only needed if we are going to enter retention or off.
	 */
	if (mpu_next_state <= PWRDM_POWER_RET)
		disable_smartreflex(SR1);
	if (core_next_state <= PWRDM_POWER_RET)
		disable_smartreflex(SR2);

	/* CORE */
	if (core_next_state < PWRDM_POWER_ON) {
		omap_uart_prepare_idle(0);
		omap_uart_prepare_idle(1);
		if (core_next_state == PWRDM_POWER_OFF) {
			prm_set_mod_reg_bits(OMAP3430_AUTO_OFF,
					     OMAP3430_GR_MOD,
					     OMAP3_PRM_VOLTCTRL_OFFSET);
			omap3_core_save_context();
			omap3_prcm_save_context();
		} else if (core_next_state == PWRDM_POWER_RET) {
			prm_set_mod_reg_bits(OMAP3430_AUTO_RET,
						OMAP3430_GR_MOD,
						OMAP3_PRM_VOLTCTRL_OFFSET);
		}
	}

	omap3_intc_prepare_idle();

	/*
	* On EMU/HS devices ROM code restores a SRDC value
	* from scratchpad which has automatic self refresh on timeout
	* of AUTO_CNT = 1 enabled. This takes care of errata 1.142.
	* Hence store/restore the SDRC_POWER register here.
	*/
	if (omap_rev() >= OMAP3430_REV_ES3_0 &&
	    omap_type() != OMAP2_DEVICE_TYPE_GP &&
	    core_next_state == PWRDM_POWER_OFF)
		sdrc_pwr = sdrc_read_reg(SDRC_POWER);

	if (regset_save_on_suspend)
		pm_dbg_regset_save(1);

	/*
	 * omap3_arm_context is the location where ARM registers
	 * get saved. The restore path then reads from this
	 * location and restores them back.
	 */
	_omap_sram_idle(omap3_arm_context, save_state);
	cpu_init();

	if (regset_save_on_suspend)
		pm_dbg_regset_save(2);

	/*
	* FIXME:
	* In the case of wakeup from MPU OFF, ROM code will bring EMU clock
	* domain up, which leads to extra current consumption. So temporarily
	* work around it until the change from ROM code.
	*/
	if (regset_save_on_suspend) {
		/* Disable WDT1 iclk */
		cm_rmw_mod_reg_bits(1 << 4, 0, WKUP_MOD, CM_ICLKEN);
		/* Software supervised sleep on EMU clock domain */
		cm_rmw_mod_reg_bits(0x3, 0x1, OMAP3430_EMU_MOD, CM_CLKSTCTRL);
	}

	/* Restore normal SDRC POWER settings */
	if (omap_rev() >= OMAP3430_REV_ES3_0 &&
	    omap_type() != OMAP2_DEVICE_TYPE_GP &&
	    core_next_state == PWRDM_POWER_OFF)
		sdrc_write_reg(sdrc_pwr, SDRC_POWER);

	/* Restore table entry modified during MMU restoration */
	if (pwrdm_read_prev_pwrst(mpu_pwrdm) == PWRDM_POWER_OFF)
		restore_table_entry();

	/* CORE */
	if (core_next_state < PWRDM_POWER_ON) {
		core_prev_state = pwrdm_read_prev_pwrst(core_pwrdm);
		if (core_prev_state == PWRDM_POWER_OFF) {
			omap3_core_restore_context();
			omap3_prcm_restore_context();
			omap3_sram_restore_context();
			omap2_sms_restore_context();
			/*
			 * Errata 1.164 fix : OTG autoidle can prevent
			 * sleep
			 */
			usb_musb_disable_autoidle();
		}
		omap_uart_resume_idle(0);
		omap_uart_resume_idle(1);
		if (core_next_state == PWRDM_POWER_OFF)
			prm_clear_mod_reg_bits(OMAP3430_AUTO_OFF,
					       OMAP3430_GR_MOD,
					       OMAP3_PRM_VOLTCTRL_OFFSET);
		else if (core_next_state == PWRDM_POWER_RET)
			prm_clear_mod_reg_bits(OMAP3430_AUTO_RET,
						OMAP3430_GR_MOD,
						OMAP3_PRM_VOLTCTRL_OFFSET);
	}
	omap3_intc_resume_idle();

	/*
	 * Enable smartreflex after WFI. Only needed if we entered
	 * retention or off
	 */
	if (mpu_next_state <= PWRDM_POWER_RET)
		enable_smartreflex(SR1);
	if (core_next_state <= PWRDM_POWER_RET)
		enable_smartreflex(SR2);

	/* PER */
	if (per_next_state < PWRDM_POWER_ON) {
		if (per_next_state == PWRDM_POWER_OFF) {
			/*
			 * Reading the prev-state takes long time (11us@OPP2),
			 * only do it, if we really tried to put PER in OFF
			 */
			per_prev_state = pwrdm_read_prev_pwrst(per_pwrdm);
			if (per_prev_state == PWRDM_POWER_OFF) {
				omap3_per_restore_context();
				omap3_gpio_restore_pad_context(0);
			} else if (per_next_state == PWRDM_POWER_OFF) {
				omap3_gpio_restore_pad_context(1);
			}
		}
		omap2_gpio_resume_after_idle();
		omap_uart_resume_idle(2);
		if (per_state_modified)
			pwrdm_set_next_pwrst(per_pwrdm, PWRDM_POWER_OFF);
	}

	/* Disable IO-PAD and IO-CHAIN wakeup */
#ifdef CONFIG_PM_DEPPSLEEP
	if ((per_next_state < PWRDM_POWER_ON ||
		core_next_state < PWRDM_POWER_ON) && !get_deepsleep_mode()) {
		prm_clear_mod_reg_bits(OMAP3430_EN_IO, WKUP_MOD, PM_WKEN);
		omap3_disable_io_chain();
	}
#else
	if (per_next_state < PWRDM_POWER_ON ||
			core_next_state < PWRDM_POWER_ON) {
		prm_clear_mod_reg_bits(OMAP3430_EN_IO, WKUP_MOD, PM_WKEN);
		omap3_disable_io_chain();
	}
#endif
	pwrdm_post_transition();

	omap2_clkdm_allow_idle(mpu_pwrdm->pwrdm_clkdms[0]);
	/*
	 * FIXME
	 * ROM code might issue the WFI intruction while without saving
	 * and restoring MPU context, so just set the MPU next state to RET.
	 */
	pwrdm_set_next_pwrst(mpu_pwrdm, PWRDM_POWER_RET);
}

int omap3_can_sleep(void)
{
	if (!sleep_while_idle)
		return 0;
	if (!omap_uart_can_sleep())
		return 0;
	return 1;
}

/* This sets pwrdm state (other than mpu & core. Currently only ON &
 * RET are supported. Function is assuming that clkdm doesn't have
 * hw_sup mode enabled. */
int set_pwrdm_state(struct powerdomain *pwrdm, u32 state)
{
	u32 cur_state;
	int sleep_switch = 0;
	int ret = 0;

	if (pwrdm == NULL || IS_ERR(pwrdm))
		return -EINVAL;

	/*
	 * FIXME
	 * For omap3430, DSS, PERR and CORE OFF mode is not supported.
	 * Temporarily disabled OFF mode for omap3630 until ES1.1 applied
	 */
	if (cpu_is_omap34xx())
		if ((state == PWRDM_POWER_OFF) &&
				(!strcmp(pwrdm->name, "per_pwrdm") ||
				!strcmp(pwrdm->name, "dss_pwrdm") ||
				!strcmp(pwrdm->name, "core_pwrdm")))
			state = PWRDM_POWER_RET;

	while (!(pwrdm->pwrsts & (1 << state))) {
		if (state == PWRDM_POWER_OFF)
			return ret;
		state--;
	}

	cur_state = pwrdm_read_next_pwrst(pwrdm);
	if (cur_state == state)
		return ret;

	/*
	 * check if bridge has hibernated? if yes then just return success
	 * If OFF mode is not enabled, sleep switch is performed for IVA
	 * which is not necessory.
	 * REVISIT: Bridge has to set powerstate based on enable_off_mode state
	 */
	if (!strcmp(pwrdm->name, "iva2_pwrdm")) {
		if (cur_state == PWRDM_POWER_OFF)
			return ret;
	}

	if (pwrdm_read_pwrst(pwrdm) < PWRDM_POWER_ON) {
		omap2_clkdm_wakeup(pwrdm->pwrdm_clkdms[0]);
		sleep_switch = 1;
		pwrdm_wait_transition(pwrdm);
	}

	ret = pwrdm_set_next_pwrst(pwrdm, state);
	if (ret) {
		printk(KERN_ERR "Unable to set state of powerdomain: %s\n",
		       pwrdm->name);
		goto err;
	}

	if (sleep_switch) {
		omap2_clkdm_allow_idle(pwrdm->pwrdm_clkdms[0]);
		pwrdm_wait_transition(pwrdm);
		pwrdm_state_switch(pwrdm);
	}

err:
	return ret;
}

static void omap3_pm_idle(void)
{
	local_irq_disable();
	local_fiq_disable();

	if (!omap3_can_sleep())
		goto out;

	if (omap_irq_pending() || need_resched())
		goto out;

	trace_pm_idle_entry();
	save_sync_trace_clock();

	omap_sram_idle();

	/*
	 * Resyncing the trace clock should ideally be done much sooner. When
	 * we arrive here, there are already some interrupt handlers which have
	 * run before us, using potentially wrong timestamps. This leads
	 * to problems when restarting the clock (and synchronizing on the 32k
	 * clock) if the cycle counter was still active.
	 * resync_track_clock must ensure that timestamps never ever go
	 * backward.
	 */
	resync_trace_clock();
	trace_pm_idle_exit();

out:
	local_fiq_enable();
	local_irq_enable();
}

#ifdef CONFIG_SUSPEND
static suspend_state_t suspend_state;

static void omap2_pm_wakeup_on_timer(u32 seconds, u32 nseconds)
{
	u32 tick_rate, cycles;

	if (!seconds && !nseconds)
		return;

	tick_rate = clk_get_rate(omap_dm_timer_get_fclk(gptimer_wakeup));
	cycles = tick_rate * seconds;
	cycles += (nseconds / NSEC_PER_MSEC) * tick_rate / MSEC_PER_SEC;
	omap_dm_timer_stop(gptimer_wakeup);
	omap_dm_timer_set_load_start(gptimer_wakeup, 0, 0xffffffff - cycles);

	pr_info("PM: Resume timer in %d secs (%d ticks at %d ticks/sec.)\n",
		seconds, cycles, tick_rate);
}

static int omap3_pm_prepare(void)
{
	disable_hlt();
	return 0;
}

static int omap3_pm_suspend(void)
{
	struct power_state *pwrst;
	int state, ret = 0;

	if (wakeup_timer_seconds || wakeup_timer_nseconds)
		omap2_pm_wakeup_on_timer(wakeup_timer_seconds,
					wakeup_timer_nseconds);

	/* Read current next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node)
		pwrst->saved_state = pwrdm_read_next_pwrst(pwrst->pwrdm);
	/* Set ones wanted by suspend */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (set_pwrdm_state(pwrst->pwrdm, pwrst->next_state))
			goto restore;
		if (pwrdm_clear_all_prev_pwrst(pwrst->pwrdm))
			goto restore;
	}

	omap_uart_prepare_suspend();
	trace_pm_suspend_entry();
	save_sync_trace_clock();
	omap3_intc_suspend();

	regset_save_on_suspend = 1;
	omap_sram_idle();
	regset_save_on_suspend = 0;
	resync_trace_clock();
	trace_pm_suspend_exit();

restore:
	/* Restore next_pwrsts */
	list_for_each_entry(pwrst, &pwrst_list, node) {
		state = pwrdm_read_prev_pwrst(pwrst->pwrdm);
		if (state > pwrdm_read_next_pwrst(pwrst->pwrdm)) {
			printk(KERN_INFO "Powerdomain (%s) didn't enter "
				"target state %d\n",
				pwrst->pwrdm->name,
				pwrdm_read_next_pwrst(pwrst->pwrdm));
			if (pwrst->pwrdm == core_pwrdm)
				pm_dbg_show_core_regs();
			ret = -1;
		}
		set_pwrdm_state(pwrst->pwrdm, pwrst->saved_state);
	}
	if (ret)
		printk(KERN_ERR "Could not enter target state in pm_suspend\n");
	else {
		printk(KERN_INFO "Successfully put all powerdomains "
		       "to target state\n");
		pm_dbg_show_wakeup_source();
	}

	return ret;
}

static int omap3_pm_enter(suspend_state_t unused)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap3_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void omap3_pm_finish(void)
{
	enable_hlt();
}

/* Hooks to enable / disable UART interrupts during suspend */
static int omap3_pm_begin(suspend_state_t state)
{
	suspend_state = state;
	omap_uart_enable_irqs(0);
	return 0;
}

static void omap3_pm_end(void)
{
	suspend_state = PM_SUSPEND_ON;
	omap_uart_enable_irqs(1);
	return;
}

static struct platform_suspend_ops omap_pm_ops = {
	.begin		= omap3_pm_begin,
	.end		= omap3_pm_end,
	.prepare	= omap3_pm_prepare,
	.enter		= omap3_pm_enter,
	.finish		= omap3_pm_finish,
	.valid		= suspend_valid_only_mem,
};
#endif /* CONFIG_SUSPEND */


/**
 * omap3_iva_idle(): ensure IVA is in idle so it can be put into
 *                   retention
 *
 * In cases where IVA2 is activated by bootcode, it may prevent
 * full-chip retention or off-mode because it is not idle.  This
 * function forces the IVA2 into idle state so it can go
 * into retention/off and thus allow full-chip retention/off.
 *
 **/
static void __init omap3_iva_idle(void)
{
	/* ensure IVA2 clock is disabled */
	cm_write_mod_reg(0, OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* if no clock activity, nothing else to do */
	if (!(cm_read_mod_reg(OMAP3430_IVA2_MOD, OMAP3430_CM_CLKSTST) &
	      OMAP3430_CLKACTIVITY_IVA2_MASK))
		return;

	/* Reset IVA2 */
	prm_write_mod_reg(OMAP3430_RST1_IVA2 |
			  OMAP3430_RST2_IVA2 |
			  OMAP3430_RST3_IVA2,
			  OMAP3430_IVA2_MOD, RM_RSTCTRL);

	/* Enable IVA2 clock */
	cm_write_mod_reg(OMAP3430_CM_FCLKEN_IVA2_EN_IVA2,
			 OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* Set IVA2 boot mode to 'idle' */
	omap_ctrl_writel(OMAP3_IVA2_BOOTMOD_IDLE,
			 OMAP343X_CONTROL_IVA2_BOOTMOD);

	/* Un-reset IVA2 */
	prm_write_mod_reg(0, OMAP3430_IVA2_MOD, RM_RSTCTRL);

	/* Disable IVA2 clock */
	cm_write_mod_reg(0, OMAP3430_IVA2_MOD, CM_FCLKEN);

	/* Reset IVA2 */
	prm_write_mod_reg(OMAP3430_RST1_IVA2 |
			  OMAP3430_RST2_IVA2 |
			  OMAP3430_RST3_IVA2,
			  OMAP3430_IVA2_MOD, RM_RSTCTRL);
}

static void __init omap3_d2d_idle(void)
{
	u16 mask, padconf;

	/* In a stand alone OMAP3430 where there is not a stacked
	 * modem for the D2D Idle Ack and D2D MStandby must be pulled
	 * high. S CONTROL_PADCONF_SAD2D_IDLEACK and
	 * CONTROL_PADCONF_SAD2D_MSTDBY to have a pull up. */
	mask = (1 << 4) | (1 << 3); /* pull-up, enabled */
	padconf = omap_ctrl_readw(OMAP3_PADCONF_SAD2D_MSTANDBY);
	padconf |= mask;
	omap_ctrl_writew(padconf, OMAP3_PADCONF_SAD2D_MSTANDBY);

	padconf = omap_ctrl_readw(OMAP3_PADCONF_SAD2D_IDLEACK);
	padconf |= mask;
	omap_ctrl_writew(padconf, OMAP3_PADCONF_SAD2D_IDLEACK);

	/* reset modem */
#ifndef CONFIG_MACH_MAPPHONE
	prm_write_mod_reg(OMAP3430_RM_RSTCTRL_CORE_MODEM_SW_RSTPWRON |
			  OMAP3430_RM_RSTCTRL_CORE_MODEM_SW_RST,
			  CORE_MOD, RM_RSTCTRL);
	prm_write_mod_reg(0, CORE_MOD, RM_RSTCTRL);
#endif
}

static void __init prcm_setup_regs(void)
{
	/* XXX Reset all wkdeps. This should be done when initializing
	 * powerdomains */
	prm_write_mod_reg(0, OMAP3430_IVA2_MOD, PM_WKDEP);
	prm_write_mod_reg(0, MPU_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_DSS_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_NEON_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_CAM_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP3430_PER_MOD, PM_WKDEP);
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		prm_write_mod_reg(0, OMAP3430ES2_SGX_MOD, PM_WKDEP);
		prm_write_mod_reg(0, OMAP3430ES2_USBHOST_MOD, PM_WKDEP);
		prm_write_mod_reg(0, OMAP3430ES2_USBHOST_MOD, OMAP3430_PM_IVAGRPSEL);
	} else
		prm_write_mod_reg(0, GFX_MOD, PM_WKDEP);

	/*
	 * Enable interface clock autoidle for all modules.
	 * Note that in the long run this should be done by clockfw
	 */
	cm_write_mod_reg(
		OMAP3430_AUTO_MODEM |
		OMAP3430ES2_AUTO_MMC3 |
		OMAP3430ES2_AUTO_ICR |
		OMAP3430_AUTO_AES2 |
		OMAP3430_AUTO_SHA12 |
		OMAP3430_AUTO_DES2 |
		OMAP3430_AUTO_MMC2 |
		OMAP3430_AUTO_MMC1 |
		OMAP3430_AUTO_MSPRO |
		OMAP3430_AUTO_HDQ |
		OMAP3430_AUTO_MCSPI4 |
		OMAP3430_AUTO_MCSPI3 |
		OMAP3430_AUTO_MCSPI2 |
		OMAP3430_AUTO_MCSPI1 |
		OMAP3430_AUTO_I2C3 |
		OMAP3430_AUTO_I2C2 |
		OMAP3430_AUTO_I2C1 |
		OMAP3430_AUTO_UART2 |
		OMAP3430_AUTO_UART1 |
		OMAP3430_AUTO_GPT11 |
		OMAP3430_AUTO_GPT10 |
		OMAP3430_AUTO_MCBSP5 |
		OMAP3430_AUTO_MCBSP1 |
		OMAP3430ES1_AUTO_FAC | /* This is es1 only */
		OMAP3430_AUTO_MAILBOXES |
		OMAP3430_AUTO_OMAPCTRL |
		OMAP3430ES1_AUTO_FSHOSTUSB |
		OMAP3430_AUTO_HSOTGUSB |
		OMAP3430_AUTO_SAD2D |
		OMAP3430_AUTO_SSI,
		CORE_MOD, CM_AUTOIDLE1);

	prm_rmw_mod_reg_bits(0x80000008,
		0,
		CORE_MOD,
		OMAP3430_PM_MPUGRPSEL1);

	cm_write_mod_reg(
		OMAP3430_AUTO_PKA |
		OMAP3430_AUTO_AES1 |
		OMAP3430_AUTO_RNG |
		OMAP3430_AUTO_SHA11 |
		OMAP3430_AUTO_DES1,
		CORE_MOD, CM_AUTOIDLE2);

	if (omap_rev() > OMAP3430_REV_ES1_0) {
		cm_write_mod_reg(
			OMAP3430_AUTO_MAD2D |
			OMAP3430ES2_AUTO_USBTLL,
			CORE_MOD, CM_AUTOIDLE3);
	}

	cm_write_mod_reg(
		OMAP3430_AUTO_WDT2 |
		OMAP3430_AUTO_WDT1 |
		OMAP3430_AUTO_GPIO1 |
		OMAP3430_AUTO_32KSYNC |
		OMAP3430_AUTO_GPT12 |
		OMAP3430_AUTO_GPT1 ,
		WKUP_MOD, CM_AUTOIDLE);

	cm_write_mod_reg(
		OMAP3430_AUTO_DSS,
		OMAP3430_DSS_MOD,
		CM_AUTOIDLE);

	cm_write_mod_reg(
		OMAP3430_AUTO_CAM,
		OMAP3430_CAM_MOD,
		CM_AUTOIDLE);

	cm_write_mod_reg(
		OMAP3430_AUTO_GPIO6 |
		OMAP3430_AUTO_GPIO5 |
		OMAP3430_AUTO_GPIO4 |
		OMAP3430_AUTO_GPIO3 |
		OMAP3430_AUTO_GPIO2 |
		OMAP3430_AUTO_WDT3 |
		OMAP3430_AUTO_UART3 |
		OMAP3430_AUTO_GPT9 |
		OMAP3430_AUTO_GPT8 |
		OMAP3430_AUTO_GPT7 |
		OMAP3430_AUTO_GPT6 |
		OMAP3430_AUTO_GPT5 |
		OMAP3430_AUTO_GPT4 |
		OMAP3430_AUTO_GPT3 |
		OMAP3430_AUTO_GPT2 |
		OMAP3430_AUTO_MCBSP4 |
		OMAP3430_AUTO_MCBSP3 |
		OMAP3430_AUTO_MCBSP2,
		OMAP3430_PER_MOD,
		CM_AUTOIDLE);

	if (omap_rev() > OMAP3430_REV_ES1_0) {
		cm_write_mod_reg(
			OMAP3430ES2_AUTO_USBHOST,
			OMAP3430ES2_USBHOST_MOD,
			CM_AUTOIDLE);
	}

	omap_ctrl_writel(OMAP3430_AUTOIDLE, OMAP2_CONTROL_SYSCONFIG);

	/*
	 * Set all plls to autoidle. This is needed until autoidle is
	 * enabled by clockfw
	 */
	cm_write_mod_reg(1 << OMAP3430_AUTO_IVA2_DPLL_SHIFT,
			 OMAP3430_IVA2_MOD, CM_AUTOIDLE2);
	cm_write_mod_reg(1 << OMAP3430_AUTO_MPU_DPLL_SHIFT,
			 MPU_MOD,
			 CM_AUTOIDLE2);
	cm_write_mod_reg((1 << OMAP3430_AUTO_PERIPH_DPLL_SHIFT) |
			 (1 << OMAP3430_AUTO_CORE_DPLL_SHIFT),
			 PLL_MOD,
			 CM_AUTOIDLE);
	cm_write_mod_reg(1 << OMAP3430ES2_AUTO_PERIPH2_DPLL_SHIFT,
			 PLL_MOD,
			 CM_AUTOIDLE2);

	/*
	 * Enable control of expternal oscillator through
	 * sys_clkreq. In the long run clock framework should
	 * take care of this.
	 */
	/* FIXME
	 * Deassert sys_clk only if it is in bypass mode.
	 * Due to some h/w issue, the system can't be woken up from sleep mode
	 * when it's in oscillator mode with sys_clk deasserted. So just keep
	 * sys_clk asserted in this mode which will cause extra about 8mA
	 * current drain. But new version h/w will fix this issue.
	 */
	if ((prm_read_mod_reg(OMAP3430_GR_MOD, OMAP3_PRM_CLKSRC_CTRL_OFFSET)
		& 0x3) == 0)
		prm_rmw_mod_reg_bits(OMAP_AUTOEXTCLKMODE_MASK,
			     1 << OMAP_AUTOEXTCLKMODE_SHIFT,
			     OMAP3430_GR_MOD,
			     OMAP3_PRM_CLKSRC_CTRL_OFFSET);

	/* setup wakup source */
	prm_write_mod_reg(OMAP3430_EN_IO | OMAP3430_EN_GPIO1 |
			  OMAP3430_EN_GPT1 | OMAP3430_EN_GPT12,
			  WKUP_MOD, PM_WKEN);
	/* No need to write EN_IO, that is always enabled */
	prm_write_mod_reg(OMAP3430_EN_GPIO1 | OMAP3430_EN_GPT1 |
			  OMAP3430_EN_GPT12,
			  WKUP_MOD, OMAP3430_PM_MPUGRPSEL);
	/* For some reason IO doesn't generate wakeup event even if
	 * it is selected to mpu wakeup goup */
	prm_write_mod_reg(OMAP3430_IO_EN | OMAP3430_WKUP_EN,
			  OCP_MOD, OMAP3_PRM_IRQENABLE_MPU_OFFSET);

	/* Enable wakeups in PER */
	prm_write_mod_reg(OMAP3430_EN_GPIO2 | OMAP3430_EN_GPIO3 |
			  OMAP3430_EN_GPIO4 | OMAP3430_EN_GPIO5 |
			  OMAP3430_EN_GPIO6 | OMAP3430_EN_UART3 |
			  OMAP3430_EN_MCBSP2 |  OMAP3430_EN_MCBSP3,
			  OMAP3430_PER_MOD, PM_WKEN);
	/* and allow them to wake up MPU */
	prm_write_mod_reg(OMAP3430_GRPSEL_GPIO2 | OMAP3430_EN_GPIO3 |
			  OMAP3430_GRPSEL_GPIO4 | OMAP3430_EN_GPIO5 |
			  OMAP3430_GRPSEL_GPIO6 | OMAP3430_EN_UART3 |
			  OMAP3430_EN_MCBSP2 | OMAP3430_EN_MCBSP3,
			  OMAP3430_PER_MOD, OMAP3430_PM_MPUGRPSEL);

	/* Don't attach IVA interrupts */
	prm_write_mod_reg(0, WKUP_MOD, OMAP3430_PM_IVAGRPSEL);
	prm_write_mod_reg(0, CORE_MOD, OMAP3430_PM_IVAGRPSEL1);
	prm_write_mod_reg(0, CORE_MOD, OMAP3430ES2_PM_IVAGRPSEL3);
	prm_write_mod_reg(0, OMAP3430_PER_MOD, OMAP3430_PM_IVAGRPSEL);

	/* Clear any pending 'reset' flags */
	prm_write_mod_reg(0xffffffff, MPU_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, CORE_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_PER_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_EMU_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_NEON_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430_DSS_MOD, RM_RSTST);
	prm_write_mod_reg(0xffffffff, OMAP3430ES2_USBHOST_MOD, RM_RSTST);

	if (cpu_is_omap3430())
		cm_rmw_mod_reg_bits(OMAP3430_IVA2_CLK_SRC_MASK,
				(0x4 << OMAP3430_IVA2_CLK_SRC_SHIFT),
				OMAP3430_IVA2_MOD, CM_CLKSEL1);
	else if (cpu_is_omap3630())
		cm_rmw_mod_reg_bits(OMAP3430_IVA2_CLK_SRC_MASK,
				(0x2 << OMAP3430_IVA2_CLK_SRC_SHIFT),
				OMAP3430_IVA2_MOD, CM_CLKSEL1);

	/* Clear any pending PRCM interrupts */
	prm_write_mod_reg(0xffffffff, OCP_MOD, OMAP3_PRM_IRQSTATUS_MPU_OFFSET);

	/* Initialize Domains PREPWSTST to ON */
	pwrdm_clear_all_prev_pwrst(mpu_pwrdm);
	pwrdm_clear_all_prev_pwrst(neon_pwrdm);
	pwrdm_clear_all_prev_pwrst(core_pwrdm);
	pwrdm_clear_all_prev_pwrst(per_pwrdm);

#ifdef CONFIG_PM_DEEPSLEEP
	prm_clear_mod_reg_bits(OMAP3430_EN_IO, WKUP_MOD, PM_WKEN);
#endif

	omap3_iva_idle();
	omap3_d2d_idle();
}

void omap3_pm_off_mode_enable(int enable)
{
	struct power_state *pwrst;
	u32 state;

	if (enable)
		state = PWRDM_POWER_OFF;
	else
		state = PWRDM_POWER_RET;

#ifdef CONFIG_OMAP_PM_SRF
	resource_lock_opp(VDD1_OPP);
	resource_lock_opp(VDD2_OPP);
	if (resource_refresh())
		printk(KERN_ERR "Error: could not refresh resources\n");
	resource_unlock_opp(VDD1_OPP);
	resource_unlock_opp(VDD2_OPP);
#endif
	list_for_each_entry(pwrst, &pwrst_list, node) {
		pwrst->next_state = state;
		/*
		 * FIXME
		 * ROM code might issue WFI instructions while without saving
		 * and restoring the MPU context, so the MPU next state OFF
		 * setting will be postponed until the suspend and idle are
		 * triggered if the next state is OFF.
		*/
		if (!strcmp(pwrst->pwrdm->name, "mpu_pwrdm"))
			continue;
		set_pwrdm_state(pwrst->pwrdm, state);
	}
}

int omap3_pm_get_suspend_state(struct powerdomain *pwrdm)
{
	struct power_state *pwrst;

	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (pwrst->pwrdm == pwrdm)
			return pwrst->next_state;
	}
	return -EINVAL;
}

int omap3_pm_set_suspend_state(struct powerdomain *pwrdm, int state)
{
	struct power_state *pwrst;

	list_for_each_entry(pwrst, &pwrst_list, node) {
		if (pwrst->pwrdm == pwrdm) {
			pwrst->next_state = state;
			return 0;
		}
	}
	return -EINVAL;
}

void omap3_pm_init_vc(struct prm_setup_vc *setup_vc)
{
	if (!setup_vc)
		return;

	prm_setup.clksetup = setup_vc->clksetup;
	prm_setup.voltsetup_time1 = setup_vc->voltsetup_time1;
	prm_setup.voltsetup_time2 = setup_vc->voltsetup_time2;
	prm_setup.voltoffset = setup_vc->voltoffset;
	prm_setup.voltsetup2 = setup_vc->voltsetup2;
	prm_setup.vdd0_on = setup_vc->vdd0_on;
	prm_setup.vdd0_onlp = setup_vc->vdd0_onlp;
	prm_setup.vdd0_ret = setup_vc->vdd0_ret;
	prm_setup.vdd0_off = setup_vc->vdd0_off;
	prm_setup.vdd1_on = setup_vc->vdd1_on;
	prm_setup.vdd1_onlp = setup_vc->vdd1_onlp;
	prm_setup.vdd1_ret = setup_vc->vdd1_ret;
	prm_setup.vdd1_off = setup_vc->vdd1_off;
	prm_setup.i2c_slave_ra = setup_vc->i2c_slave_ra;
	prm_setup.vdd_vol_ra = setup_vc->vdd_vol_ra;
	prm_setup.vdd_cmd_ra = setup_vc->vdd_cmd_ra;
	prm_setup.vdd_ch_conf = setup_vc->vdd_ch_conf;
	prm_setup.vdd_i2c_cfg = setup_vc->vdd_i2c_cfg;
}

static int __init pwrdms_setup(struct powerdomain *pwrdm, void *unused)
{
	struct power_state *pwrst;

	if (!pwrdm->pwrsts)
		return 0;

	pwrst = kmalloc(sizeof(struct power_state), GFP_ATOMIC);
	if (!pwrst)
		return -ENOMEM;
	pwrst->pwrdm = pwrdm;
	pwrst->next_state = PWRDM_POWER_RET;
	list_add(&pwrst->node, &pwrst_list);

	if (pwrdm_has_hdwr_sar(pwrdm))
		pwrdm_enable_hdwr_sar(pwrdm);

	return set_pwrdm_state(pwrst->pwrdm, pwrst->next_state);
}

/*
 * Enable hw supervised mode for all clockdomains if it's
 * supported. Initiate sleep transition for other clockdomains, if
 * they are not used
 */
static int __init clkdms_setup(struct clockdomain *clkdm, void *unused)
{
	if (clkdm->flags & CLKDM_CAN_ENABLE_AUTO)
		omap2_clkdm_allow_idle(clkdm);
	else if (clkdm->flags & CLKDM_CAN_FORCE_SLEEP &&
		 atomic_read(&clkdm->usecount) == 0)
		omap2_clkdm_sleep(clkdm);
	return 0;
}

void omap_push_sram_idle(void)
{
	_omap_sram_idle = omap_sram_push(omap34xx_cpu_suspend,
					omap34xx_cpu_suspend_sz);
	if (omap_type() != OMAP2_DEVICE_TYPE_GP)
		_omap_save_secure_sram = omap_sram_push(save_secure_ram_context,
				save_secure_ram_context_sz);
}

#ifdef CONFIG_OMAP_PM_SRF
static void set_opps_max(bool panic)
{
	int flags = OPP_IGNORE_LOCK;

	if (panic)
		flags |= OPP_IGNORE_NOTIFIER;

	resource_set_opp_level(VDD2_OPP, MAX_VDD2_OPP, flags);
	resource_set_opp_level(VDD1_OPP, MAX_VDD1_OPP, flags);
	return;
}
#else
static void set_opps_max(bool panic)
{
	return;
}
#endif

static int prcm_prepare_reboot(struct notifier_block *this, unsigned long code,
					void *x)
{
	if ((code == SYS_DOWN) || (code == SYS_HALT) ||
		(code == SYS_POWER_OFF)) {
		set_opps_max(false);
	}
	return NOTIFY_DONE;
}

static struct notifier_block prcm_notifier = {
	.notifier_call	= prcm_prepare_reboot,
	.next		= NULL,
	.priority	= INT_MAX,
};

static int panic_prepare_reboot(struct notifier_block *this,
					unsigned long code, void *x)
{
	return NOTIFY_DONE;
}

static struct notifier_block prcm_panic_notifier = {
	.notifier_call	= panic_prepare_reboot,
	.next		= NULL,
	.priority	= INT_MAX,
};

static int __init omap3_pm_init(void)
{
	struct power_state *pwrst, *tmp;
	int ret;
	u32 reg;

	if (!cpu_is_omap34xx())
		return -ENODEV;

	/*
	 * Issue: failure is seen after second or minutes of operation
	 * while doing intensive DSP operation like video record
	 * or playback. It only happen on phones using SEC silicon
	 *
	 * Workaround: Disable MEM RTA (Retention Till Access)
	 * for second source Samsung OMAP3630 Silicon
	 */
	if (omap_is_SEC()) {
		reg = omap_ctrl_readl(OMAP36XX_CONTROL_MEM_RTA_CTRL);
		omap_ctrl_writel(0x0, OMAP36XX_CONTROL_MEM_RTA_CTRL);
	}

	printk(KERN_ERR "Power Management for TI OMAP3.\n");

	/* XXX prcm_setup_regs needs to be before enabling hw
	 * supervised mode for powerdomains */
	prcm_setup_regs();

	ret = request_irq(INT_34XX_PRCM_MPU_IRQ,
			  (irq_handler_t)prcm_interrupt_handler,
			  IRQF_DISABLED, "prcm", NULL);
	if (ret) {
		printk(KERN_ERR "request_irq failed to register for 0x%x\n",
		       INT_34XX_PRCM_MPU_IRQ);
		goto err1;
	}

	ret = pwrdm_for_each(pwrdms_setup, NULL);
	if (ret) {
		printk(KERN_ERR "Failed to setup powerdomains\n");
		goto err2;
	}

	(void) clkdm_for_each(clkdms_setup, NULL);

	mpu_pwrdm = pwrdm_lookup("mpu_pwrdm");
	if (mpu_pwrdm == NULL) {
		printk(KERN_ERR "Failed to get mpu_pwrdm\n");
		goto err2;
	}

	neon_pwrdm = pwrdm_lookup("neon_pwrdm");
	if (neon_pwrdm == NULL) {
		printk(KERN_ERR "Failed to get neon_pwrdm\n");
		goto err2;
	}

	per_pwrdm = pwrdm_lookup("per_pwrdm");
	if (per_pwrdm == NULL) {
		printk(KERN_ERR "Failed to get per_pwrdm\n");
		goto err2;
	}

	core_pwrdm = pwrdm_lookup("core_pwrdm");
	if (core_pwrdm == NULL) {
		printk(KERN_ERR "Failed to get core_pwrdm\n");
		goto err2;
	}

	cam_pwrdm = pwrdm_lookup("cam_pwrdm");
	if (cam_pwrdm == NULL) {
		printk(KERN_ERR "Failed to get cam_pwrdm\n");
		goto err2;
	}

	wkup_pwrdm = pwrdm_lookup("wkup_pwrdm");
	if (wkup_pwrdm == NULL) {
		printk(KERN_ERR "Failed to get wkup_pwrdm_pwrdm\n");
		goto err2;
	}

	omap_push_sram_idle();
#ifdef CONFIG_SUSPEND
	suspend_set_ops(&omap_pm_ops);
#endif /* CONFIG_SUSPEND */

	pm_idle = omap3_pm_idle;
	omap3_idle_init();

	/*
	 * OFF mode support
	 *
	 * MPU can't be waken up prior to the INTC restore when CORE hit OFF,
	 * so put wakeup dependency between MPU and CORE. This is valid only
	 * the cooresponding bits both in PM_WKEN1_CORE and PM_MPUGRPSEL1_CORE
	 * are set simultaneously.
	 */
	pwrdm_add_wkdep(mpu_pwrdm,  core_pwrdm);

	pwrdm_add_wkdep(neon_pwrdm, mpu_pwrdm);
	/*
	 * REVISIT: This wkdep is only necessary when GPIO2-6 are enabled for
	 * IO-pad wakeup.  Otherwise it will unnecessarily waste power
	 * waking up PER with every CORE wakeup - see
	 * http://marc.info/?l=linux-omap&m=121852150710062&w=2
	*/
	/* pwrdm_add_wkdep(per_pwrdm, core_pwrdm); */
	/*
	 * A part of the fix for errata 1.158.
	 * GPIO pad spurious transition (glitch/spike) upon wakeup
	 * from SYSTEM OFF mode. The remaining fix is in:
	 * omap3_gpio_save_context, omap3_gpio_restore_context.
	 */
	if (omap_rev() <= OMAP3430_REV_ES3_1)
		pwrdm_add_wkdep(per_pwrdm, wkup_pwrdm);

	if (omap_type() != OMAP2_DEVICE_TYPE_GP) {
		omap3_secure_ram_storage =
			kmalloc(0x803F, GFP_KERNEL);
		if (!omap3_secure_ram_storage)
			printk(KERN_ERR "Memory allocation failed when"
					"allocating for secure sram context\n");

		local_irq_disable();
		local_fiq_disable();

		omap3_save_secure_ram_context(PWRDM_POWER_ON);

		local_irq_enable();
		local_fiq_enable();
	}

	omap3_save_scratchpad_contents();
	register_reboot_notifier(&prcm_notifier);
	atomic_notifier_chain_register(&panic_notifier_list,
					&prcm_panic_notifier);

	sleep_while_idle = 1;
	if (cpu_is_omap3630())
		enable_abb_mode = 1;
	/* Enable the l2 cache toggling in sleep logic */
	if (cpu_is_omap3630())
		enable_omap3630_toggle_l2_on_restore();
	enable_off_mode = 1;
	omap3_pm_off_mode_enable(enable_off_mode);

err1:
	return ret;
err2:
	free_irq(INT_34XX_PRCM_MPU_IRQ, NULL);
	list_for_each_entry_safe(pwrst, tmp, &pwrst_list, node) {
		list_del(&pwrst->node);
		kfree(pwrst);
	}
	return ret;
}

/* Program Power IC via bypass interface */
int omap3_bypass_cmd(u8 slave_addr, u8 reg_addr, u8 cmd) {
	u32 vc_bypass_value;
	u32 loop_cnt = 0, retries_cnt = 0;

	vc_bypass_value = (cmd << OMAP3430_DATA_SHIFT) |
			(reg_addr << OMAP3430_REGADDR_SHIFT) |
			(slave_addr << OMAP3430_SLAVEADDR_SHIFT);

	prm_write_mod_reg(vc_bypass_value, OMAP3430_GR_MOD,
			OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	vc_bypass_value = prm_set_mod_reg_bits(OMAP3430_VALID, OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	while ((vc_bypass_value & OMAP3430_VALID) != 0x0) {
		loop_cnt++;
		if (retries_cnt > 10) {
			printk(KERN_ERR"Loop count exceeded in check SR I2C"
								"write\n");
			return 1;
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
		vc_bypass_value = prm_read_mod_reg(OMAP3430_GR_MOD,
					OMAP3_PRM_VC_BYPASS_VAL_OFFSET);
	}

	return 0;
}

static void __init configure_vc(void)
{
	prm_write_mod_reg(prm_setup.i2c_slave_ra, OMAP3430_GR_MOD,
				OMAP3_PRM_VC_SMPS_SA_OFFSET);
	prm_write_mod_reg(prm_setup.vdd_vol_ra, OMAP3430_GR_MOD,
				OMAP3_PRM_VC_SMPS_VOL_RA_OFFSET);

	/* Only set if power_ic has different voltage and cmd addrs */
	if (prm_setup.vdd_vol_ra != prm_setup.vdd_cmd_ra)
		prm_write_mod_reg(prm_setup.vdd_cmd_ra, OMAP3430_GR_MOD,
				OMAP3_PRM_VC_SMPS_CMD_RA_OFFSET);

	prm_write_mod_reg((prm_setup.vdd0_on << OMAP3430_VC_CMD_ON_SHIFT) |
		(prm_setup.vdd0_onlp << OMAP3430_VC_CMD_ONLP_SHIFT) |
		(prm_setup.vdd0_ret << OMAP3430_VC_CMD_RET_SHIFT) |
		(prm_setup.vdd0_off << OMAP3430_VC_CMD_OFF_SHIFT),
		OMAP3430_GR_MOD, OMAP3_PRM_VC_CMD_VAL_0_OFFSET);

	prm_write_mod_reg((prm_setup.vdd1_on << OMAP3430_VC_CMD_ON_SHIFT) |
		(prm_setup.vdd1_onlp << OMAP3430_VC_CMD_ONLP_SHIFT) |
		(prm_setup.vdd1_ret << OMAP3430_VC_CMD_RET_SHIFT) |
		(prm_setup.vdd1_off << OMAP3430_VC_CMD_OFF_SHIFT),
		OMAP3430_GR_MOD, OMAP3_PRM_VC_CMD_VAL_1_OFFSET);

	prm_write_mod_reg(prm_setup.vdd_ch_conf, OMAP3430_GR_MOD,
				OMAP3_PRM_VC_CH_CONF_OFFSET);

	prm_write_mod_reg(prm_setup.vdd_i2c_cfg, OMAP3430_GR_MOD,
				OMAP3_PRM_VC_I2C_CFG_OFFSET);

	/* Write setup times */
	prm_write_mod_reg(prm_setup.clksetup, OMAP3430_GR_MOD,
			OMAP3_PRM_CLKSETUP_OFFSET);
	prm_write_mod_reg((prm_setup.voltsetup_time2 <<
			OMAP3430_SETUP_TIME2_SHIFT) |
			(prm_setup.voltsetup_time1 <<
			OMAP3430_SETUP_TIME1_SHIFT),
			OMAP3430_GR_MOD, OMAP3_PRM_VOLTSETUP1_OFFSET);

	prm_write_mod_reg(prm_setup.voltoffset, OMAP3430_GR_MOD,
			OMAP3_PRM_VOLTOFFSET_OFFSET);
	prm_write_mod_reg(prm_setup.voltsetup2, OMAP3430_GR_MOD,
			OMAP3_PRM_VOLTSETUP2_OFFSET);

	pm_dbg_regset_init(1);
	pm_dbg_regset_init(2);
}

static int __init omap3_pm_early_init(void)
{
	prm_clear_mod_reg_bits(OMAP3430_OFFMODE_POL, OMAP3430_GR_MOD,
				OMAP3_PRM_POLCTRL_OFFSET);

	configure_vc();

	return 0;
}

arch_initcall(omap3_pm_early_init);
late_initcall(omap3_pm_init);

/**
 * omap3630_abb_change_active_opp - handle OPP changes with Adaptive Body-Bias
 * @target_opp_no: OPP we're transitioning to
 *
 * Adaptive Body-Bias is a 3630-specific technique to boost voltage in high
 * OPPs for silicon with weak characteristics as well as lower voltage in low
 * OPPs for silicon with strong characteristics.
 *
 * Only Foward Body-Bias for operating at high OPPs is implemented below.
 * Reverse Body-Bias for saving power in active cases and sleep cases is not
 * yet implemented.
 */
int omap3630_abb_change_active_opp(u32 target_opp_no)
{
	u32 active_fbb_enabled;
	u32 sr2en_enabled;
	int timeout;

	/* FIXME: force ACTIVE_FBB_SEL until EFUSE values are burnt in */
	prm_set_mod_reg_bits(OMAP3630_ACTIVE_FBB_SEL,
			OMAP3430_GR_MOD,
			OMAP3_PRM_LDO_ABB_CTRL_OFFSET);

	/* program settling time of 30us for ABB ldo transition */
	prm_rmw_mod_reg_bits(OMAP3630_SR2_WTCNT_VALUE_MASK,
			(0x62 << OMAP3630_SR2_WTCNT_VALUE_SHIFT),
			OMAP3430_GR_MOD,
			OMAP3_PRM_LDO_ABB_CTRL_OFFSET);

	/* FIXME: code below anticipates us relying on EFUSE someday */
	active_fbb_enabled = (prm_read_mod_reg(OMAP3430_GR_MOD,
				OMAP3_PRM_LDO_ABB_CTRL_OFFSET) &
			OMAP3630_ACTIVE_FBB_SEL);

	/* has SR2EN been enabled previously? */
	sr2en_enabled = (prm_read_mod_reg(OMAP3430_GR_MOD,
				OMAP3_PRM_LDO_ABB_CTRL_OFFSET) &
			OMAP3630_SR2EN);

	/* select OPP */
	if (target_opp_no >= 4 && active_fbb_enabled && enable_abb_mode) {
		/* program for fast opp */
		prm_rmw_mod_reg_bits(OMAP3630_OPP_SEL_MASK,
				(ABB_FAST_OPP << OMAP3630_OPP_SEL_SHIFT),
				OMAP3430_GR_MOD,
				OMAP3_PRM_LDO_ABB_SETUP_OFFSET);

		/* enable the ABB ldo if not done already */
		if (!sr2en_enabled)
			prm_set_mod_reg_bits(OMAP3630_SR2EN,
					OMAP3430_GR_MOD,
					OMAP3_PRM_LDO_ABB_CTRL_OFFSET);
	} else if (active_fbb_enabled && sr2en_enabled) {
		/* program for nominal opp */
		prm_rmw_mod_reg_bits(OMAP3630_OPP_SEL_MASK,
				(ABB_NOMINAL_OPP << OMAP3630_OPP_SEL_SHIFT),
				OMAP3430_GR_MOD,
				OMAP3_PRM_LDO_ABB_SETUP_OFFSET);
	}

	/* clear ABB ldo interrupt status */
	prm_write_mod_reg(OMAP3630_ABB_LDO_TRANXDONE_ST,
			OCP_MOD,
			OMAP3_PRM_IRQSTATUS_MPU_OFFSET);

	/* enable ABB LDO OPP change */
	prm_set_mod_reg_bits(OMAP3630_OPP_CHANGE,
			OMAP3430_GR_MOD,
			OMAP3_PRM_LDO_ABB_SETUP_OFFSET);

	timeout = 0;

	/* wait until OPP change completes */
	while ((timeout < ABB_TRANXDONE_TIMEOUT) &&
			(!(prm_read_mod_reg(OCP_MOD,
					    OMAP3_PRM_IRQSTATUS_MPU_OFFSET) &
			   OMAP3630_ABB_LDO_TRANXDONE_ST))) {
		udelay(1);
		timeout++;
	}

	if (timeout == ABB_TRANXDONE_TIMEOUT)
		pr_debug("ABB: TRANXDONE timed out waiting for OPP change\n");

	/* clear ABB ldo interrupt status */
	prm_write_mod_reg(OMAP3630_ABB_LDO_TRANXDONE_ST,
			OCP_MOD,
			OMAP3_PRM_IRQSTATUS_MPU_OFFSET);
	return 0;
}

#ifdef CONFIG_OMAP_SMARTREFLEX_CLASS1P5
static ssize_t sr_adjust_vsel_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	u8 num_mpu_opps = omap_pm_get_max_vdd1_opp();
	u8 num_l3_opps = omap_pm_get_max_vdd2_opp();
	int i;
	char *tbuf = buf;

	tbuf += sprintf(tbuf, "oppid:\t[nominal v]\t[calib v]\t"
			"[nval]\t[error]\n");
	for (i = 1; i <= num_mpu_opps; i++)
		if (mpu_opps[i].rate)
			tbuf += sprintf(tbuf, "mpu %d:\t0x%02x\t\t"
					"0x%02x\t0x%08x\t0x%08x\n",
					i,
					mpu_opps[i].vsel,
					mpu_opps[i].sr_adjust_vsel,
					mpu_opps[i].sr_nval,
					mpu_opps[i].sr_err);
	for (i = 1; i <= num_l3_opps; i++)
		if (l3_opps[i].rate)
			tbuf += sprintf(tbuf, "l3 %d:\t0x%02x\t\t"
					"0x%02x\t0x%08x\t0x%08x\n",
					i,
					l3_opps[i].vsel,
					l3_opps[i].sr_adjust_vsel,
					l3_opps[i].sr_nval,
					l3_opps[i].sr_err);
	return tbuf - buf;
}


static ssize_t sr_adjust_vsel_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t n)
{
	unsigned short value;
	u8 num_mpu_opps;
	u8 num_l3_opps;
	int i;

	if ((sscanf(buf, "%hu", &value) > 1) || value) {
		pr_err("%s: Invalid value %d\n", __func__, value);
		return -EINVAL;
	}
	num_mpu_opps = omap_pm_get_max_vdd1_opp();
	num_l3_opps = omap_pm_get_max_vdd2_opp();
	/* reset the calibrated voltages which are enabled */
	for (i = 1; i <= num_mpu_opps; i++)
		if (mpu_opps[i].rate) {
			mpu_opps[i].sr_adjust_vsel = 0;
			mpu_opps[i].sr_err = 0;
			mpu_opps[i].sr_nval = 0;
		}
	for (i = 1; i <= num_l3_opps; i++)
		if (l3_opps[i].rate) {
			l3_opps[i].sr_adjust_vsel = 0;
			l3_opps[i].sr_err = 0;
			l3_opps[i].sr_nval = 0;
		}
	return n;
}

static struct kobj_attribute sr_adjust_vsel_attr =
__ATTR(sr_adjust_vsel, 0644, sr_adjust_vsel_show, sr_adjust_vsel_store);

static int __init omap_sr_adjust_vsel_init(void)
{
	if (sysfs_create_file(power_kobj, &sr_adjust_vsel_attr.attr))
		pr_warning("sr_adjust_vsel: sysfs_create_file failed\n");
	return 0;
}
late_initcall(omap_sr_adjust_vsel_init);
#endif /*  OMAP_SMARTREFLEX_CLASS1P5 */

