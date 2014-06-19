/*
 * Copyright (C) 2014 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Work based on OMAP Power Management debug routines
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Copyright (C) 2006-2008 Nokia Corporation
 *
 * Written by:
 *   Richard Woodruff <r-woodruff2@ti.com>
 *   Tony Lindgren
 *   Juha Yrjola
 *   Amit Kucheria <amit.kucheria@nokia.com>
 *   Igor Stoppa <igor.stoppa@nokia.com>
 *   Jouni Hogander
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "iomap.h"
#include "clock.h"
#include "powerdomain.h"
#include "clockdomain.h"
#include "omap-pm.h"

#include "soc.h"
#include "cm2xxx_3xxx.h"
#include "prm3xxx.h"
#include "prm2xxx_3xxx.h"
#include "pm.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static struct dentry *pm_dbg_dir;
#endif

struct pm_module_def {
	char name[8];           /* Name of the module     */
	short type;             /* CM or PRM              */
	unsigned short offset;
	int low; /* First register address on this module */
	int high; /* Last register address on this module */
};

#define MOD_CM 0
#define MOD_PRM 1


static const struct pm_module_def pm_dbg_reg_modules[] = {
	{ "IVA2", MOD_CM, OMAP3430_IVA2_MOD, 0, 0x4c },
	{ "OCP",  MOD_CM, OCP_MOD, 0, 0x10 },
	{ "MPU",  MOD_CM, MPU_MOD, 4, 0x4c },
	{ "CORE", MOD_CM, CORE_MOD, 0, 0x4c },
	{ "SGX",  MOD_CM, OMAP3430ES2_SGX_MOD, 0, 0x4c },
	{ "WKUP", MOD_CM, WKUP_MOD, 0, 0x40 },
	{ "CCR",  MOD_CM, PLL_MOD, 0, 0x70 },
	{ "DSS",  MOD_CM, OMAP3430_DSS_MOD, 0, 0x4c },
	{ "CAM",  MOD_CM, OMAP3430_CAM_MOD, 0, 0x4c },
	{ "PER",  MOD_CM, OMAP3430_PER_MOD, 0, 0x4c },
	{ "EMU",  MOD_CM, OMAP3430_EMU_MOD, 0x40, 0x54 },
	{ "NEON", MOD_CM, OMAP3430_NEON_MOD, 0x20, 0x48 },
	{ "USB",  MOD_CM, OMAP3430ES2_USBHOST_MOD, 0, 0x4c },

	{ "IVA2", MOD_PRM, OMAP3430_IVA2_MOD, 0x50, 0xfc },
	{ "OCP",  MOD_PRM, OCP_MOD, 4, 0x1c },
	{ "MPU",  MOD_PRM, MPU_MOD, 0x58, 0xe8 },
	{ "CORE", MOD_PRM, CORE_MOD, 0x58, 0xf8 },
	{ "SGX",  MOD_PRM, OMAP3430ES2_SGX_MOD, 0x58, 0xe8 },
	{ "WKUP", MOD_PRM, WKUP_MOD, 0xa0, 0xb0 },
	{ "CCR",  MOD_PRM, PLL_MOD, 0x40, 0x70 },
	{ "DSS",  MOD_PRM, OMAP3430_DSS_MOD, 0x58, 0xe8 },
	{ "CAM",  MOD_PRM, OMAP3430_CAM_MOD, 0x58, 0xe8 },
	{ "PER",  MOD_PRM, OMAP3430_PER_MOD, 0x58, 0xe8 },
	{ "EMU",  MOD_PRM, OMAP3430_EMU_MOD, 0x58, 0xe4 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0x20, 0xe4 },
	{ "NEON", MOD_PRM, OMAP3430_NEON_MOD, 0x58, 0xe8 },
	{ "USB",  MOD_PRM, OMAP3430ES2_USBHOST_MOD, 0x58, 0xe8 },
	{ "", 0, 0, 0, 0 },
};

#define PM_DBG_MAX_REG_SETS 4

static void *pm_dbg_reg_set[PM_DBG_MAX_REG_SETS] = {0};

static int pm_dbg_get_regset_size(void)
{
	static int regset_size = 0;

	if (regset_size == 0) {
		int i = 0;
		while (pm_dbg_reg_modules[i].name[0] != 0) {
			regset_size += pm_dbg_reg_modules[i].high +
				4 - pm_dbg_reg_modules[i].low;
			i++;
		}
	}
	return regset_size;
}

static void pm_dbg_regset_store(u32 *ptr)
{
	int i = 0;
	int j;
	u32 val;

	while (pm_dbg_reg_modules[i].name[0] != 0) {
		for (j = pm_dbg_reg_modules[i].low;
			j <= pm_dbg_reg_modules[i].high; j += 4) {
			if (pm_dbg_reg_modules[i].type == MOD_CM)
				val = omap2_cm_read_mod_reg(
					pm_dbg_reg_modules[i].offset, j);
			else
				val = omap2_prm_read_mod_reg(
					pm_dbg_reg_modules[i].offset, j);
			*(ptr++) = val;
		}
		i++;
	}
}

void pm_dbg_regs_copy(int tgt, int src)
{
	size_t sz = pm_dbg_get_regset_size();

	pr_debug("saved reference copy %s(%d, %d) size = %u <- %pS\n",
		__func__, tgt, src, sz, __builtin_return_address(0));
	memcpy(pm_dbg_reg_set[tgt - 1], pm_dbg_reg_set[src - 1], sz);
}

void pm_dbg_regs_save(int reg_set)
{
	if (pm_dbg_reg_set[reg_set - 1] == NULL)
		return;

	pm_dbg_regset_store(pm_dbg_reg_set[reg_set - 1]);
}

#ifdef CONFIG_DEBUG_FS
static int pm_dbg_show_regs(struct seq_file *s, void *unused)
{
	int i, j;
	unsigned long val;
	int reg_set = (int)s->private;
	u32 *ptr;
	void *store = NULL;
	int regs;
	int linefeed;

	if (!cpu_is_omap34xx())
		return -EINVAL;

	if (reg_set == 0) {
		store = kmalloc(pm_dbg_get_regset_size(), GFP_KERNEL);
		if (store == NULL)
			return -ENOMEM;
		ptr = store;
		pm_dbg_regset_store(ptr);
	} else {
		ptr = pm_dbg_reg_set[reg_set - 1];
	}

	i = 0;

	while (pm_dbg_reg_modules[i].name[0] != 0) {
		regs = 0;
		linefeed = 0;
		if (pm_dbg_reg_modules[i].type == MOD_CM)
			seq_printf(s, "MOD: CM_%s (%08x)\n",
				pm_dbg_reg_modules[i].name,
				(u32)(OMAP3430_CM_BASE +
				pm_dbg_reg_modules[i].offset));
		else
			seq_printf(s, "MOD: PRM_%s (%08x)\n",
				pm_dbg_reg_modules[i].name,
				(u32)(OMAP3430_PRM_BASE +
				pm_dbg_reg_modules[i].offset));

		for (j = pm_dbg_reg_modules[i].low;
			j <= pm_dbg_reg_modules[i].high; j += 4) {
			val = *(ptr++);
			if (val != 0) {
				regs++;
				if (linefeed) {
					seq_printf(s, "\n");
					linefeed = 0;
				}
				seq_printf(s, "  %02x => %08lx", j, val);
				if (regs % 4 == 0)
					linefeed = 1;
			}
		}
		seq_printf(s, "\n");
		i++;
	}

	if (store != NULL)
		kfree(store);

	return 0;
}

#define WAKEUP_SOURCE_LEN 512
void pm_dbg_show_wakeup_source(void)
{
	u32 val = 0;
	int len = 0;
	static char buf[WAKEUP_SOURCE_LEN];
	char *pbuf;
	u32 gpio_bit = 0;

	/* print the real wkup sources */
	memset(buf, 0, WAKEUP_SOURCE_LEN);
	pbuf = buf;
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (len > 16)
		pbuf += snprintf(pbuf, len, "WAKEDUP BY: ");

	val = omap2_prm_read_mod_reg(WKUP_MOD, PM_WKST);
	val &= omap2_prm_read_mod_reg(WKUP_MOD, PM_WKEN);
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "WKUP_MOD(0x%x), ", val);

	val = omap2_prm_read_mod_reg(CORE_MOD, PM_WKST1);
	val &= omap2_prm_read_mod_reg(CORE_MOD, PM_WKEN1);
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "CORE_MOD(0x%x), ", val);

	val = omap2_prm_read_mod_reg(CORE_MOD, OMAP3430ES2_PM_WKST3);
	val &= omap2_prm_read_mod_reg(CORE_MOD, OMAP3430ES2_PM_WKEN3);
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "CORE3_MOD(0x%x), ", val);

	val = omap2_prm_read_mod_reg(OMAP3430_PER_MOD, PM_WKST);
	val &= omap2_prm_read_mod_reg(OMAP3430_PER_MOD, PM_WKEN);
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "PER_MOD(0x%x), ", val);

	val = omap2_prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, PM_WKST);
	val &= omap2_prm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, PM_WKEN);
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "USBHOST(0x%x), ", val);

	val = omap2_prm_read_mod_reg(OMAP3430_DSS_MOD, PM_WKST);
	val &= omap2_prm_read_mod_reg(OMAP3430_DSS_MOD, PM_WKEN);
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "DSS(0x%x), ", val);

	val = omap2_prm_read_mod_reg(OCP_MOD, OMAP3_PRM_IRQSTATUS_MPU_OFFSET);
	val &= omap2_prm_read_mod_reg(OCP_MOD, OMAP3_PRM_IRQENABLE_MPU_OFFSET);
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "MPU_IRQSTATUS(0x%x), ", val);

	val = omap2_prm_read_mod_reg(OMAP3430_IVA2_MOD, OMAP3430_PRM_IRQSTATUS_IVA2);
	val &= omap2_prm_read_mod_reg(OMAP3430_IVA2_MOD, OMAP3430_PRM_IRQENABLE_IVA2);
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "IVA2_IRQSTATUS(0x%x), ", val);

	val = __raw_readl(OMAP2_L4_IO_ADDRESS(OMAP34XX_IC_BASE + (0x009C)));
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "INTC_FIQ0(0x%x), ", val);

	val = __raw_readl(OMAP2_L4_IO_ADDRESS(OMAP34XX_IC_BASE + (0x00bC)));
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "INTC_FIQ1(0x%x), ", val);

	val = __raw_readl(OMAP2_L4_IO_ADDRESS(OMAP34XX_IC_BASE + (0x00dC)));
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "INTC_FIQ2(0x%x), ", val);


	val = __raw_readl(OMAP2_L4_IO_ADDRESS(OMAP34XX_IC_BASE + (0x0098)));
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "INTC_IRQ0(0x%x), ", val);

	val = __raw_readl(OMAP2_L4_IO_ADDRESS(OMAP34XX_IC_BASE + (0x00B8)));
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "INTC_IRQ1(0x%x), ", val);

	val = __raw_readl(OMAP2_L4_IO_ADDRESS(OMAP34XX_IC_BASE + (0x00D8)));
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "INTC_IRQ2(0x%x), ", val);

	val = __raw_readl(OMAP2_L4_IO_ADDRESS(OMAP34XX_IC_BASE + (0x009C)));
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "INTC_FIQ0(0x%x), ", val);

	val = __raw_readl(OMAP2_L4_IO_ADDRESS(OMAP34XX_IC_BASE + (0x00BC)));
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "INTC_FIQ1(0x%x), ", val);

	val = __raw_readl(OMAP2_L4_IO_ADDRESS(OMAP34XX_IC_BASE + (0x00DC)));
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "INTC_FIQ2(0x%x), ", val);

	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if ((val & (1<<29)) && len > 20) {
		gpio_bit = __raw_readl(OMAP2_L4_IO_ADDRESS(0x48310018)) &
			__raw_readl(OMAP2_L4_IO_ADDRESS(0x4831001C));
		pbuf += snprintf(pbuf, len, "GPIO1(0x%x), ", gpio_bit);
	}
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if ((val & (1<<30)) && len > 20) {
		gpio_bit = __raw_readl(OMAP2_L4_IO_ADDRESS(0x49050018)) &
			__raw_readl(OMAP2_L4_IO_ADDRESS(0x4905001C));
		pbuf += snprintf(pbuf, len, "GPIO2(0x%x), ", gpio_bit);
	}
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if ((val & (1<<31)) && len > 20) {
		gpio_bit = __raw_readl(OMAP2_L4_IO_ADDRESS(0x49052018)) &
			__raw_readl(OMAP2_L4_IO_ADDRESS(0x4905201C));
		pbuf += snprintf(pbuf, len, "GPIO3(0x%x), ", gpio_bit);
	}

	val = __raw_readl(OMAP2_L4_IO_ADDRESS(OMAP34XX_IC_BASE + (0x00b8)));
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "INTC_IRQ1(0x%x), ", val);

	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if ((val & (1<<0)) && len > 20) {
		gpio_bit = __raw_readl(OMAP2_L4_IO_ADDRESS(0x49054018)) &
			__raw_readl(OMAP2_L4_IO_ADDRESS(0x4905401C));
		pbuf += snprintf(pbuf, len, "GPIO4(0x%x), ", gpio_bit);
	}
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if ((val & (1<<1)) && len > 20) {
		gpio_bit = __raw_readl(OMAP2_L4_IO_ADDRESS(0x49056018)) &
			__raw_readl(OMAP2_L4_IO_ADDRESS(0x4905601C));
		pbuf += snprintf(pbuf, len, "GPIO5(0x%x), ", gpio_bit);
	}
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if ((val & (1<<2)) && len > 20) {
		gpio_bit = __raw_readl(OMAP2_L4_IO_ADDRESS(0x49058018)) &
			__raw_readl(OMAP2_L4_IO_ADDRESS(0x4905801C));
		pbuf += snprintf(pbuf, len, "GPIO6(0x%x), ", gpio_bit);
	}

	val = __raw_readl(OMAP2_L4_IO_ADDRESS(OMAP34XX_IC_BASE + (0x00d8)));
	len = WAKEUP_SOURCE_LEN - (pbuf - buf);
	if (val && len > 30)
		pbuf += snprintf(pbuf, len, "INTC_IRQ2(0x%x)", val);

	pr_debug("%s\n", buf);

}
EXPORT_SYMBOL(pm_dbg_show_wakeup_source);

static int pm_dbg_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pm_dbg_show_regs, inode->i_private);
}

static const struct file_operations debug_reg_fops = {
	.open           = pm_dbg_reg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};
#endif

int pm_dbg_regs_dump(int reg_set)
{
	int i, j;
	unsigned long val;
	u32 *ptr;
	int regs;

	if ((reg_set <= 0) || (reg_set > PM_DBG_MAX_REG_SETS))
		return -EINVAL;

	ptr = pm_dbg_reg_set[reg_set - 1];

	i = 0;

	while (pm_dbg_reg_modules[i].name[0] != 0) {
		regs = 0;
		if (pm_dbg_reg_modules[i].type == MOD_CM)
			pr_debug("MOD: CM_%s (%08x)\n",
				pm_dbg_reg_modules[i].name,
				(u32)(OMAP3430_CM_BASE +
				pm_dbg_reg_modules[i].offset));
		else
			pr_debug("MOD: PRM_%s (%08x)\n",
				pm_dbg_reg_modules[i].name,
				(u32)(OMAP3430_PRM_BASE +
				pm_dbg_reg_modules[i].offset));

		for (j = pm_dbg_reg_modules[i].low;
			j <= pm_dbg_reg_modules[i].high; j += 4) {
			val = *(ptr++);
			if (val != 0) {
				regs++;
				pr_debug("  %02x => %08lx\n", j, val);
			}
		}
		i++;
	}

	return 0;
}
EXPORT_SYMBOL(pm_dbg_regs_dump);

int pm_dbg_regs_dump_delta(int cur, int ref)
{
	int i, j;
	unsigned long val_cur;
	u32 *ptr_cur;
	unsigned long val_ref;
	u32 *ptr_ref;

	if ((cur <= 0) || (cur > PM_DBG_MAX_REG_SETS) ||
	    (ref <= 0) || (ref > PM_DBG_MAX_REG_SETS)) {
		return -EINVAL;
	}

	ptr_cur = pm_dbg_reg_set[cur - 1];
	ptr_ref = pm_dbg_reg_set[ref - 1];

	i = 0;

	pr_debug("   module      ( address) reg        %d          %d\n",
			cur, ref);
	while (pm_dbg_reg_modules[i].name[0] != 0) {
		bool cm = pm_dbg_reg_modules[i].type == MOD_CM;
		u32 base_addr = cm ?  OMAP3430_CM_BASE : OMAP3430_PRM_BASE;

		for (j = pm_dbg_reg_modules[i].low;
		     j <= pm_dbg_reg_modules[i].high; j += 4) {
			u32 addr = (u32)(base_addr + pm_dbg_reg_modules[i].offset);

			val_cur = *(ptr_cur++);
			val_ref = *(ptr_ref++);
			if (val_cur != val_ref) {
				pr_debug("MOD: %s_%-4s %s(%08x) "
					"%02x => 0x%08lx 0x%08lx\n",
					cm ? "CM" : "PRM",
					pm_dbg_reg_modules[i].name,
					cm ? " " : "",
					addr, j, val_cur, val_ref);
			}
		}
		i++;
	}
	return 0;
}
EXPORT_SYMBOL(pm_dbg_regs_dump_delta);

static void __init pm_dbg_regset_init(void)
{
	int i;

	for (i = 0; i < PM_DBG_MAX_REG_SETS; i++) {
		pm_dbg_reg_set[i] =
			kmalloc(pm_dbg_get_regset_size(), GFP_KERNEL);
	}
}

#ifdef CONFIG_DEBUG_FS
int __init pm_dbg_regs_init(struct dentry *d)
{
	int i;
	char name[2];

	if (!cpu_is_omap34xx()) {
		pr_err("%s: only OMAP3 supported\n", __func__);
		return -ENODEV;
	}

	pm_dbg_dir = debugfs_create_dir("registers", d);
	if (IS_ERR(pm_dbg_dir))
		return PTR_ERR(pm_dbg_dir);

	(void) debugfs_create_file("current", S_IRUGO,
		pm_dbg_dir, (void *)0, &debug_reg_fops);

	pm_dbg_regset_init();
	for (i = 0; i < PM_DBG_MAX_REG_SETS; i++) {
		if (pm_dbg_reg_set[i] != NULL) {
			sprintf(name, "%d", i + 1);
			(void) debugfs_create_file(name, S_IRUGO,
				pm_dbg_dir, (void *)(i+1), &debug_reg_fops);
		}
	}

	return 0;
}
#else
static int __init pm_dbg_regs_init(void)
{
	if (!cpu_is_omap34xx()) {
		pr_err("%s: only OMAP3 supported\n", __func__);
		return -ENODEV;
	}

	pm_dbg_regset_init();
	return 0;
}
omap_arch_initcall(pm_dbg_regs_init);
#endif
