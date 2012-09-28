/*
 * OMAP2/3 Power Management Routines
 *
 * Copyright (C) 2008 Nokia Corporation
 * Jouni Hogander
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ARCH_ARM_MACH_OMAP2_PM_H
#define __ARCH_ARM_MACH_OMAP2_PM_H

#include <plat/powerdomain.h>

extern u32 enable_off_mode;
extern u32 sleep_while_idle;
extern u32 voltage_off_while_idle;
extern unsigned int wakeup_timer_nseconds;
extern u32 enable_abb_mode;

extern void *omap3_secure_ram_storage;
extern void omap3_pm_off_mode_enable(int);
extern void omap_sram_idle(void);
extern int omap3_can_sleep(void);
extern int set_pwrdm_state(struct powerdomain *pwrdm, u32 state);
extern int omap3_idle_init(void);
extern void vfp_pm_save_context(void);

extern void lock_scratchpad_sem(void);
extern void unlock_scratchpad_sem(void);

struct prm_setup_vc {
	u16 clksetup;
	u16 voltsetup_time1;
	u16 voltsetup_time2;
	u16 voltoffset;
	u16 voltsetup2;

/* PRM_VC_CMD_VAL_0 specific bits */
	u16 vdd0_on;
	u16 vdd0_onlp;
	u16 vdd0_ret;
	u16 vdd0_off;
/* PRM_VC_CMD_VAL_1 specific bits */
	u16 vdd1_on;
	u16 vdd1_onlp;
	u16 vdd1_ret;
	u16 vdd1_off;

/* Values for VDD registers */
	u32 i2c_slave_ra;
	u32 vdd_vol_ra;
	u32 vdd_cmd_ra;
	u32 vdd_ch_conf;
	u32 vdd_i2c_cfg;
};

struct cpuidle_params {
	u8  valid;
	u32 sleep_latency;
	u32 wake_latency;
	u32 threshold;
};

extern void omap3_pm_init_vc(struct prm_setup_vc *setup_vc);
#ifdef CONFIG_CPU_IDLE
extern void omap3_pm_init_cpuidle(struct cpuidle_params *cpuidle_board_params);
extern int omap3_bypass_cmd(u8 slave_addr, u8 reg_addr, u8 cmd);
#else
static inline void omap3_pm_init_cpuidle(
			struct cpuidle_params *cpuidle_board_params)
{
}
#endif

extern int resource_set_opp_level(int res, u32 target_level, int flags);
extern int resource_access_opp_lock(int res, int delta);
#define resource_lock_opp(res) resource_access_opp_lock(res, 1)
#define resource_unlock_opp(res) resource_access_opp_lock(res, -1)
#define resource_get_opp_lock(res) resource_access_opp_lock(res, 0)

#define OPP_IGNORE_LOCK 0x1
#define OPP_IGNORE_NOTIFIER 0x2

extern int omap3_pm_get_suspend_state(struct powerdomain *pwrdm);
extern int omap3_pm_set_suspend_state(struct powerdomain *pwrdm, int state);

extern u32 wakeup_timer_seconds;
extern struct omap_dm_timer *gptimer_wakeup;

#ifdef CONFIG_PM_DEBUG
extern void omap2_pm_dump(int mode, int resume, unsigned int us);
extern int omap2_pm_debug;
#else
#define omap2_pm_dump(mode, resume, us)		do {} while (0);
#define omap2_pm_debug				0
#endif

#if defined(CONFIG_PM_DEBUG) && defined(CONFIG_DEBUG_FS)
extern void pm_dbg_update_time(struct powerdomain *pwrdm, int prev);
extern int pm_dbg_regset_save(int reg_set);
extern int pm_dbg_regset_init(int reg_set);
extern void pm_dbg_show_core_regs(void);
extern void pm_dbg_show_wakeup_source(void);
#else
#define pm_dbg_update_time(pwrdm, prev) do {} while (0);
#define pm_dbg_regset_save(reg_set) do {} while (0);
#define pm_dbg_regset_init(reg_set) do {} while (0);
#define pm_dbg_show_core_regs() do {} while (0);
#define pm_dbg_show_wakeup_source() do {} while (0);
#endif /* CONFIG_PM_DEBUG */

extern void omap24xx_idle_loop_suspend(void);

extern void omap24xx_cpu_suspend(u32 dll_ctrl, void __iomem *sdrc_dlla_ctrl,
					void __iomem *sdrc_power);
extern void omap34xx_cpu_suspend(u32 *addr, int save_state);
extern void save_secure_ram_context(u32 *addr);
extern void omap3_save_scratchpad_contents(void);

extern unsigned int omap24xx_idle_loop_suspend_sz;
extern unsigned int omap34xx_suspend_sz;
extern unsigned int save_secure_ram_context_sz;
extern unsigned int omap24xx_cpu_suspend_sz;
extern unsigned int omap34xx_cpu_suspend_sz;

#if defined(CONFIG_PM)
extern void enable_omap3630_toggle_l2_on_restore(void);
#else
static inline void enable_omap3630_toggle_l2_on_restore(void) { }
#endif

#endif
