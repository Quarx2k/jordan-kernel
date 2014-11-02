#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/io.h>

#include "../../../arch/arm/mach-omap2/omap34xx.h"
#include "../../../arch/arm/mach-omap2/iomap.h"

#include "padconf.h"

struct iomux_range {
	unsigned int start;
	unsigned int end;
};

static struct iomux_range iomux_range_base[] = {
	{OMAP343X_PADCONF_CORE_CTRL_BASE, OMAP343X_PADCONF_CORE_CTRL_TOP},
	{OMAP343X_PADCONF_CORE_ETK_BASE, OMAP343X_PADCONF_CORE_ETK_TOP},
	{OMAP343X_PADCONF_CORE_D2D_BASE, OMAP343X_PADCONF_CORE_D2D_TOP},
	{OMAP343X_PADCONF_WKUP_BASE, OMAP343X_PADCONF_WKUP_TOP},
	{OMAP343X_PADCONF_WKUP_D2D_BASE, OMAP343X_PADCONF_WKUP_D2D_TOP},
	//{OMAP343X_PADCONF_3630_GPIO_BASE, OMAP343X_PADCONF_3630_GPIO_TOP}
};

u16 omap_readw(u32 pa)
{
         return __raw_readw(OMAP2_L4_IO_ADDRESS(pa));
}
EXPORT_SYMBOL(omap_readw);

inline bool is_omap343x_padconf_register(uint16_t offset)
{
	unsigned long addr = offset + OMAP343X_CTRL_BASE;
	unsigned int i;

	if  ((offset & 0x1) != 0x0)
		return 0;

	for (i = 0; i < ARRAY_SIZE(iomux_range_base); i++)
		if (addr >= iomux_range_base[i].start &&
			 addr <= iomux_range_base[i].end)
			return 1;

	return 0;
}

static int __init muxdump_init(void) {
	int i = 0;
	unsigned short val = 0;
	unsigned long addr = 0;
	printk("Muxdump!\n");
	for (i = 0x30; i <= 0x0A50; i++) {
		if (is_omap343x_padconf_register(i))
		{
			addr = i + OMAP343X_CTRL_BASE;
			val = omap_readw(addr);
			printk("\nMuxdump0: 0x%0x: ",i);
			if (val & OMAP343X_PADCONF_OFF_WAKEUP_ENABLED)
				printk("OMAP343X_PADCONF_OFF_WAKEUP_ENABLED ");
			if (val & OMAP343X_PADCONF_OFF_PULL_UP)
				printk("| OMAP343X_PADCONF_OFF_PULL_UP");
			if (val & OMAP343X_PADCONF_OFF_PULL_DOWN)
				printk("| OMAP343X_PADCONF_OFF_PULL_DOWN");
			if (val & OMAP343X_PADCONF_OFF_PUD_ENABLED)
				printk("| OMAP343X_PADCONF_OFF_PUD_ENABLED");
			if (val & OMAP343X_PADCONF_OFF_PUD_DISABLED)
				printk("| OMAP343X_PADCONF_OFF_PUD_DISABLED");
			if (val & OMAP343X_PADCONF_OFF_OUTPUT_HIGH)
				printk("| OMAP343X_PADCONF_OFF_OUTPUT_HIGH");
			if (val & OMAP343X_PADCONF_OFF_OUTPUT_LOW)
				printk("| OMAP343X_PADCONF_OFF_OUTPUT_LOW");
			if (val & OMAP343X_PADCONF_OFF_OUTPUT_ENABLED)
				printk("| OMAP343X_PADCONF_OFF_OUTPUT_ENABLED");
			if (val & OMAP343X_PADCONF_OFF_OUTPUT_DISABLED)
				printk("| OMAP343X_PADCONF_OFF_OUTPUT_DISABLED");
			if (val & OMAP343X_PADCONF_OFFMODE_ENABLED)
				printk("| OMAP343X_PADCONF_OFFMODE_ENABLED");
			if (val & OMAP343X_PADCONF_OFFMODE_DISABLED)
				printk("| OMAP343X_PADCONF_OFFMODE_DISABLED");
			if (val & OMAP343X_PADCONF_INPUT_ENABLED)
				printk("| OMAP343X_PADCONF_INPUT_ENABLED");
			if (val & OMAP343X_PADCONF_PULL_UP)
				printk("| OMAP343X_PADCONF_PULL_UP");
			if (val & OMAP343X_PADCONF_PULL_DOWN)
				printk("| OMAP343X_PADCONF_PULL_DOWN");
			if (val & OMAP343X_PADCONF_PUD_ENABLED)
				printk("| OMAP343X_PADCONF_PUD_ENABLED");
			if (val & OMAP343X_PADCONF_PUD_DISABLED)
				printk("| OMAP343X_PADCONF_PUD_DISABLED");
			if (val & 4)
			{
				if (val & 2)
				{
					if (val & 1)
					{
					printk("| OMAP343X_PADCONF_MUXMODE7");
					}
					else
					{
					printk("| OMAP343X_PADCONF_MUXMODE6");
					}
				}
				else
				{
					if (val & 1)
					{
					printk("| OMAP343X_PADCONF_MUXMODE5");
					}
					else
					{
					printk("| OMAP343X_PADCONF_MUXMODE4");
					}
				}
			}

			else
			{
				if (val & 2)
				{
					if (val & 1)
					{
					printk("| OMAP343X_PADCONF_MUXMODE3");
					}
					else
					{
					printk("| OMAP343X_PADCONF_MUXMODE2");
					}
				}
				else
				{
					if (val & 1)
					{
					printk("| OMAP343X_PADCONF_MUXMODE1");
					}
					else
					{
					printk("| OMAP343X_PADCONF_MUXMODE0");
					}
				}
			}
		}
	}
	return 0;
}

static void __exit muxdump_exit(void) {
	printk("Bye!\n");
}
late_initcall(muxdump_init);
module_exit(muxdump_exit);

MODULE_ALIAS("muxdump");
MODULE_VERSION("0.1");
MODULE_DESCRIPTION("dump mux register contents");
MODULE_AUTHOR("kfazz");
MODULE_LICENSE("GPL");
