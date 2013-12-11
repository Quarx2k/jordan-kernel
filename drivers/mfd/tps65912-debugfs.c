/*
 * Copyright (C) 2013 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/mfd/tps65912.h>


struct  tps65912_debugfs_reg_descr {
	char *name;	/* Debugfs file name */
	u8 reg;		/* Register address */
};

/* Registers exposed in debugfs */
static const struct tps65912_debugfs_reg_descr debugfs_regs[] = {
	{ "DCDC1_CTRL",		0x00 },
	{ "DCDC2_CTRL",		0x01 },
	{ "DCDC3_CTRL",		0x02 },
	{ "DCDC4_CTRL",		0x03 },
	{ "DCDC1_OP",		0x04 },
	{ "DCDC1_AVS",		0x05 },
	{ "DCDC1_LIMIT",	0x06 },
	{ "DCDC2_OP",		0x07 },
	{ "DCDC2_AVS",		0x08 },
	{ "DCDC2_LIMIT",	0x09 },
	{ "DCDC3_OP",		0x0A },
	{ "DCDC3_AVS",		0x0B },
	{ "DCDC3_LIMIT",	0x0C },
	{ "DCDC4_OP",		0x0D },
	{ "DCDC4_AVS",		0x0E },
	{ "DCDC4_LIMIT",	0x0F },
	{ "LDO1_OP",		0x10 },
	{ "LDO1_AVS",		0x11 },
	{ "LDO1_LIMIT",		0x12 },
	{ "LDO2_OP",		0x13 },
	{ "LDO2_AVS",		0x14 },
	{ "LDO2_LIMIT",		0x15 },
	{ "LDO3_OP",		0x16 },
	{ "LDO3_AVS",		0x17 },
	{ "LDO3_LIMIT",		0x18 },
	{ "LDO4_OP",		0x19 },
	{ "LDO4_AVS",		0x1A },
	{ "LDO4_LIMIT",		0x1B },
	{ "LDO5",		0x1C },
	{ "LDO6",		0x1D },
	{ "LDO7",		0x1E },
	{ "LDO8",		0x1F },
	{ "LDO9",		0x20 },
	{ "LDO10",		0x21 },
	{ "THRM",		0x22 },
	{ "CLK32OUT",		0x23 },
	{ "DEVCTRL",		0x24 },
	{ "DEVCTRL2",		0x25 },
	{ "I2C_SPI_CFG",	0x26 },
	{ "KEEP_ON",		0x27 },
	{ "KEEP_ON2",		0x28 },
	{ "SET_OFF1",		0x29 },
	{ "SET_OFF2",		0x2A },
	{ "DEF_VOLT",		0x2B },
	{ "DEF_VOLT_MAPPING",	0x2C },
	{ "DISCHARGE",		0x2D },
	{ "DISCHARGE2",		0x2E },
	{ "EN1_SET1",		0x2F },
	{ "EN1_SET2",		0x30 },
	{ "EN2_SET1",		0x31 },
	{ "EN2_SET2",		0x32 },
	{ "EN3_SET1",		0x33 },
	{ "EN3_SET2",		0x34 },
	{ "EN4_SET1",		0x35 },
	{ "EN4_SET2",		0x36 },
	{ "PGOOD",		0x37 },
	{ "PGOOD2",		0x38 },
	{ "INT_STS",		0x39 },
	{ "INT_MSK",		0x3A },
	{ "INT_STS2",		0x3B },
	{ "INT_MSK2",		0x3C },
	{ "INT_STS3",		0x3D },
	{ "INT_MSK3",		0x3E },
	{ "INT_STS4",		0x3F },
	{ "INT_MSK4",		0x40 },
	{ "GPIO1",		0x41 },
	{ "GPIO2",		0x42 },
	{ "GPIO3",		0x43 },
	{ "GPIO4",		0x44 },
	{ "GPIO5",		0x45 },
	{ "VMON",		0x46 },
	{ "LEDA_CTRL1",		0x47 },
	{ "LEDA_CTRL2",		0x48 },
	{ "LEDA_CTRL3",		0x49 },
	{ "LEDA_CTRL4",		0x4A },
	{ "LEDA_CTRL5",		0x4B },
	{ "LEDA_CTRL6",		0x4C },
	{ "LEDA_CTRL7",		0x4D },
	{ "LEDA_CTRL8",		0x4E },
	{ "LEDB_CTRL1",		0x4F },
	{ "LEDB_CTRL2",		0x50 },
	{ "LEDB_CTRL3",		0x51 },
	{ "LEDB_CTRL4",		0x52 },
	{ "LEDB_CTRL5",		0x53 },
	{ "LEDB_CTRL6",		0x54 },
	{ "LEDB_CTRL7",		0x55 },
	{ "LEDB_CTRL8",		0x56 },
	{ "LEDC_CTRL1",		0x57 },
	{ "LEDC_CTRL2",		0x58 },
	{ "LEDC_CTRL3",		0x59 },
	{ "LEDC_CTRL4",		0x5A },
	{ "LEDC_CTRL5",		0x5B },
	{ "LEDC_CTRL6",		0x5C },
	{ "LEDC_CTRL7",		0x5D },
	{ "LEDC_CTRL8",		0x5E },
	{ "LED_RAMP_UP_TIME",	0x5F },
	{ "LED_RAMP_DOWN_TIME",	0x60 },
	{ "LED_SEQ_EN",		0x61 },
	{ "LOADSWITCH",		0x62 },
	{ "SPARE",		0x63 },
	{ "VERNUM",		0x64 },
};

struct tps65912_debugfs_reg_data {
	struct tps65912 *tps65912;	/* Device */
	u8 reg;				/* Register address */
};

struct tps65912_debugfs_data {
	struct dentry *root;	/* Debugfs directory of the device */
	struct tps65912_debugfs_reg_data reg_data[ARRAY_SIZE(debugfs_regs)];
};

static int reg_read(void *data, u64 *val)
{
	struct tps65912_debugfs_reg_data *reg_data;

	reg_data = (struct tps65912_debugfs_reg_data *) data;
	*val = tps65912_reg_read(reg_data->tps65912, reg_data->reg);
	return 0;
}

static int reg_write(void *data, u64 val)
{
	struct tps65912_debugfs_reg_data *reg_data;

	reg_data = (struct tps65912_debugfs_reg_data *) data;
	return tps65912_reg_write(reg_data->tps65912, reg_data->reg, val);
}

DEFINE_SIMPLE_ATTRIBUTE(reg_fops, reg_read, reg_write, "0x%02llx\n");

void tps65912_dump_registers(struct tps65912 *tps65912)
{
	int i, reg_data;
	for (i = 0; i < ARRAY_SIZE(debugfs_regs); i++) {
		reg_data = tps65912_reg_read(tps65912, debugfs_regs[i].reg);
		pr_info("dump register %s addr %02x with value %02x\n",
			debugfs_regs[i].name, debugfs_regs[i].reg, reg_data);
	}
}

int tps65912_debugfs_create(struct tps65912 *tps65912)
{
	int i;
	struct tps65912_debugfs_data *debugfs_data;

	debugfs_data = kzalloc(sizeof(struct tps65912_debugfs_data),
			       GFP_KERNEL);
	if (!debugfs_data)
		goto err_data;

	debugfs_data->root = debugfs_create_dir(dev_name(tps65912->dev), NULL);
	if (!debugfs_data->root)
		goto err_root;

	for (i = 0; i < ARRAY_SIZE(debugfs_regs); i++) {
		debugfs_data->reg_data[i].tps65912 = tps65912;
		debugfs_data->reg_data[i].reg = debugfs_regs[i].reg;
		if (!debugfs_create_file(debugfs_regs[i].name,
					S_IRUGO | S_IWUSR, debugfs_data->root,
					debugfs_data->reg_data + i, &reg_fops))
			goto err_file;
	}

	tps65912->debugfs_data = debugfs_data;
	return 0;

err_file:
	debugfs_remove_recursive(debugfs_data->root);
err_root:
	kfree(debugfs_data);
err_data:
	tps65912->debugfs_data = NULL;
	return -ENOMEM;
}

void tps65912_debugfs_remove(struct tps65912 *tps65912)
{
	struct tps65912_debugfs_data *debugfs_data;

	debugfs_data = (struct tps65912_debugfs_data *) tps65912->debugfs_data;
	if (debugfs_data) {
		debugfs_remove_recursive(debugfs_data->root);
		kfree(debugfs_data);
	}
}
