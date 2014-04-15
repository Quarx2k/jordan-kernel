/*
 * Copyright (C) 2013 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PM_DEBUG_REGS_H__
#define __PM_DEBUG_REGS_H__

#include <linux/dcache.h>

#ifdef CONFIG_PM_DEBUG
extern int  pm_dbg_regs_init(struct dentry *d);
extern void pm_dbg_regs_save(int reg_set);
extern void pm_dbg_regs_dump(int reg_set);
extern void pm_dbg_show_wakeup_source(void);
#else
static inline int pm_dbg_regs_init(struct dentry *d) { return 0; }
static inline void pm_dbg_regs_save(int reg_set) {};
static inline void pm_dbg_regs_dump(int reg_set) {};
static inline void pm_dbg_show_wakeup_source(void) {};
#endif

#endif
