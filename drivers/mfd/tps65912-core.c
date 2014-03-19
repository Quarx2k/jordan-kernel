/*
 * tps65912-core.c  --  TI TPS65912x
 *
 * Copyright 2011 Texas Instruments Inc.
 *
 * Author: Margarita Olaya Cabrera <magi@slimlogic.co.uk>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  This driver is based on wm8350 implementation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tps65912.h>
#include <linux/of_device.h>
#include <linux/irq.h>
#include <linux/input.h>

static struct mfd_cell tps65912s[] = {
	{
		.name = "tps65912-pmic",
	},
};

static struct platform_device tps65912_key_device = {
	.name = "tps65912_key",
	.id = -1,
	.dev.platform_data = NULL,
};

int tps65912_set_bits(struct tps65912 *tps65912, u8 reg, u8 mask)
{
	u8 data;
	int err;

	mutex_lock(&tps65912->io_mutex);

	err = tps65912->read(tps65912, reg, 1, &data);
	if (err) {
		dev_err(tps65912->dev, "Read from reg 0x%x failed\n", reg);
		goto out;
	}

	data |= mask;
	err = tps65912->write(tps65912, reg, 1, &data);
	if (err)
		dev_err(tps65912->dev, "Write to reg 0x%x failed\n", reg);

out:
	mutex_unlock(&tps65912->io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(tps65912_set_bits);

int tps65912_clear_bits(struct tps65912 *tps65912, u8 reg, u8 mask)
{
	u8 data;
	int err;

	mutex_lock(&tps65912->io_mutex);
	err = tps65912->read(tps65912, reg, 1, &data);
	if (err) {
		dev_err(tps65912->dev, "Read from reg 0x%x failed\n", reg);
		goto out;
	}

	data &= ~mask;
	err = tps65912->write(tps65912, reg, 1, &data);
	if (err)
		dev_err(tps65912->dev, "Write to reg 0x%x failed\n", reg);

out:
	mutex_unlock(&tps65912->io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(tps65912_clear_bits);

static inline int tps65912_read(struct tps65912 *tps65912, u8 reg)
{
	u8 val;
	int err;

	err = tps65912->read(tps65912, reg, 1, &val);
	if (err < 0)
		return err;

	return val;
}

static inline int tps65912_write(struct tps65912 *tps65912, u8 reg, u8 val)
{
	return tps65912->write(tps65912, reg, 1, &val);
}

int tps65912_reg_read(struct tps65912 *tps65912, u8 reg)
{
	int data;

	mutex_lock(&tps65912->io_mutex);

	data = tps65912_read(tps65912, reg);
	if (data < 0)
		dev_err(tps65912->dev, "Read from reg 0x%x failed\n", reg);

	mutex_unlock(&tps65912->io_mutex);
	return data;
}
EXPORT_SYMBOL_GPL(tps65912_reg_read);

int tps65912_reg_write(struct tps65912 *tps65912, u8 reg, u8 val)
{
	int err;

	mutex_lock(&tps65912->io_mutex);

	err = tps65912_write(tps65912, reg, val);
	if (err < 0)
		dev_err(tps65912->dev, "Write for reg 0x%x failed\n", reg);

	mutex_unlock(&tps65912->io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(tps65912_reg_write);

#ifdef CONFIG_OF

static struct tps65912_register_init_data *
tps65912_get_register_init_data(struct device *dev, int *num_init_data)
{
	struct device_node *np = dev->of_node;
	const __be32 *property;
	static struct tps65912_register_init_data *init_data;
	int i, lenp, num_cells, init_data_size;

	property = of_get_property(np, "register-init-data", &lenp);

	if (!property || lenp <= 0)
		return NULL;

	/*
	 * Check data validity and whether number of cells is even
	 */
	if (lenp % sizeof(*property)) {
		dev_err(dev, "regulator-init-data has invalid data\n");
		return NULL;
	}

	num_cells = lenp / sizeof(*property);
	if (num_cells % 2) {
		dev_err(dev, "regulator-init-data must have even "
			     " number of cells\n");
		return NULL;
	}

	init_data_size = sizeof(struct tps65912_register_init_data) *
			 (num_cells / 2);
	init_data = (struct tps65912_register_init_data *)
		    kmalloc(init_data_size, GFP_KERNEL);

	if (init_data) {
		for (i = 0; i < num_cells / 2; i++) {
			init_data[i].addr = be32_to_cpu(property[2 * i]);
			init_data[i].data = be32_to_cpu(property[2 * i + 1]);
		}
	}
	*num_init_data = num_cells / 2;

	return init_data;
}

static struct tps65912_board *tps65912_parse_dt(struct tps65912 *tps65912)
{
	struct device_node *np = tps65912->dev->of_node;
	struct tps65912_board *board_info;
	int tps_irq_gpio, ret;

	board_info = kzalloc(sizeof(struct tps65912_board), GFP_KERNEL);
	if (!board_info) {
		pr_info("kzalloc failed in tps65912_parse_dt\n");
		return NULL;
	}

	if (of_find_property(np, "dcdc1_avs", NULL)) {
		board_info->is_dcdc1_avs = 1;
		pr_info("dcdc1_avs is 1\n");
	}

	if (of_find_property(np, "dcdc2_avs", NULL)) {
		board_info->is_dcdc2_avs = 1;
		pr_info("dcdc2_avs is 1\n");
	}

	if (of_find_property(np, "dcdc3_avs", NULL)) {
		board_info->is_dcdc3_avs = 1;
		pr_info("dcdc3_avs is 1\n");
	}

	if (of_find_property(np, "dcdc4_avs", NULL)) {
		board_info->is_dcdc4_avs = 1;
		pr_info("dcdc4_avs is 1\n");
	}

	if (!of_property_read_u32(np, "powerkey_up_irq",
				  &tps65912->powerkey_up_irq)) {
		pr_info("powerkey_up_irq:%d\n", tps65912->powerkey_up_irq);
	} else {
		tps65912->powerkey_up_irq = TPS65912_IRQ_PWRHOLD_R;
		pr_info("powerkey_up_irq:defaulting to %d\n",
			tps65912->powerkey_up_irq);
	}

	if (!of_property_read_u32(np, "powerkey_down_irq",
				  &tps65912->powerkey_down_irq)) {
		pr_info("powerkey_down_irq:%d\n", tps65912->powerkey_down_irq);
	} else {
		tps65912->powerkey_up_irq = TPS65912_IRQ_PWRHOLD_F;
		pr_info("powerkey_up_irq:defaulting to %d\n",
			tps65912->powerkey_up_irq);
	}

	if (!of_property_read_u32(np, "powerkey_code",
				  &tps65912->powerkey_code)) {
		pr_info("powerkey_code:%d\n", tps65912->powerkey_code);
	} else {
		tps65912->powerkey_code = KEY_POWER;
		pr_info("powerkey_code:defaulting to %d\n",
			tps65912->powerkey_code);
	}

	if (!of_property_read_u32(np, "tps_irq_gpio", &tps_irq_gpio)) {
		ret = gpio_request(tps_irq_gpio, "tps65912-irq");
		if (ret)
			goto err;
		ret = gpio_direction_input(tps_irq_gpio);
		if (ret) {
			gpio_free(tps_irq_gpio);
			goto err;
		}

		board_info->irq = __gpio_to_irq(tps_irq_gpio);
		pr_info("tps_irq_gpio:%d irq:%d\n", tps_irq_gpio, board_info->irq);
	}
	board_info->irq_base = irq_alloc_descs(-1, 0, TPS65912_NUM_IRQ, 0);
	pr_info("irq_base:%d\n", board_info->irq_base);

	board_info->register_init_data =
		tps65912_get_register_init_data(tps65912->dev,
						&board_info->
						num_init_registers);
	return board_info;
err:
	kfree(board_info);
	return NULL;
}
#else
static inline
struct tps65912_board *tps65912_parse_dt(struct tps65912 *tps65912)
{
	return NULL;
}
#endif

int tps65912_device_init(struct tps65912 *tps65912)
{
	struct tps65912_board *pmic_plat_data;
	struct tps65912_platform_data *init_data;
	int i, ret, dcdc_avs, value;

	init_data = kzalloc(sizeof(struct tps65912_platform_data), GFP_KERNEL);
	if (init_data == NULL)
		return -ENOMEM;

	if (tps65912->dev->of_node)
		pmic_plat_data = tps65912_parse_dt(tps65912);
	else
		pmic_plat_data = dev_get_platdata(tps65912->dev);

	if (!pmic_plat_data) {
		pr_info("Platform data not found\n");
		kfree(init_data);
		return -EINVAL;
	}

	mutex_init(&tps65912->io_mutex);
	dev_set_drvdata(tps65912->dev, tps65912);

	for (i = 0; i < pmic_plat_data->num_init_registers; i++) {
		tps65912_write(tps65912,
			       pmic_plat_data->register_init_data[i].addr,
			       pmic_plat_data->register_init_data[i].data);
	}

	dcdc_avs = (pmic_plat_data->is_dcdc1_avs << 0 |
			pmic_plat_data->is_dcdc2_avs  << 1 |
				pmic_plat_data->is_dcdc3_avs << 2 |
					pmic_plat_data->is_dcdc4_avs << 3);

	if (dcdc_avs) {
		tps65912->read(tps65912, TPS65912_I2C_SPI_CFG, 1, &value);
		dcdc_avs |= value;
		tps65912->write(tps65912, TPS65912_I2C_SPI_CFG, 1, &dcdc_avs);
	}

	ret = mfd_add_devices(tps65912->dev, -1,
			      tps65912s, ARRAY_SIZE(tps65912s),
			      NULL, 0, NULL);
	if (ret < 0)
		goto err;

	tps65912_key_device.dev.platform_data = tps65912;
	ret = platform_device_register(&tps65912_key_device);
	if (ret < 0)
		goto err;

	init_data->irq = pmic_plat_data->irq;
	init_data->irq_base = pmic_plat_data->irq_base;
	ret = tps65912_irq_init(tps65912, init_data->irq, init_data);
	if (ret < 0)
		goto err_irq;

#ifdef CONFIG_MFD_TPS65912_DEBUGFS
	ret = tps65912_debugfs_create(tps65912);
	if (ret < 0)
		goto err_debugfs;
#endif

	if (tps65912->dev->of_node) {
		kfree(pmic_plat_data->register_init_data);
		kfree(pmic_plat_data);
	}
	kfree(init_data);
	return ret;

#ifdef CONFIG_MFD_TPS65912_DEBUGFS
err_debugfs:
#endif
	tps65912_irq_exit(tps65912);
err_irq:
	platform_device_unregister(&tps65912_key_device);
err:
	kfree(init_data);
	if (tps65912->dev->of_node) {
		kfree(pmic_plat_data->register_init_data);
		kfree(pmic_plat_data);
	}
	mfd_remove_devices(tps65912->dev);
	kfree(tps65912);
	return ret;
}

void tps65912_device_exit(struct tps65912 *tps65912)
{
#ifdef CONFIG_MFD_TPS65912_DEBUGFS
	tps65912_debugfs_remove(tps65912);
#endif
	mfd_remove_devices(tps65912->dev);
	tps65912_irq_exit(tps65912);
	kfree(tps65912);
}

MODULE_AUTHOR("Margarita Olaya	<magi@slimlogic.co.uk>");
MODULE_DESCRIPTION("TPS65912x chip family multi-function driver");
MODULE_LICENSE("GPL");
