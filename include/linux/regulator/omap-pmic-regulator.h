/*
 * OMAP Generic PMIC Regulator interfaces
 *
 * Idea based on arch/arm/mach-omap2/omap_twl.c and
 * arch/arm/mach-omap2/voltage.h
 * Copyright (C) 2010 Texas Instruments Incorporated.
 * Thara Gopinath
 * Copyright (C) 2009 Texas Instruments Incorporated.
 * Nishanth Menon
 * Copyright (C) 2009 Nokia Corporation
 * Paul Walmsley
 *
 * Copyright (C) 2013 Texas Instruments Incorporated.
 * Grygorii Strashko
 * Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __POWER_OMAP_PMIC_H
#define __POWER_OMAP_PMIC_H

struct omap_pmic;

/**
 * struct omap_pmic_setup_commands - setup commands over voltage controller
 * @reg:	device's i2c register address
 * @cmd_val:	command to send.
 */
struct omap_pmic_setup_commands {
	u8 reg;
	u8 cmd_val;
};

/**
 * struct omap_pmic_ops - Conversion routines for voltage controller/processor
 * @vsel_to_uv: convert voltage selector to micro-volts
 * @uv_to_vsel: convert micro-volts to voltage selector
 *
 * voltage controller/processor drivers SHOULD NOT do modifications on vsel or
 * make any assumptions about vsel. Instead, they may operate on micro-volts
 * and request vsel conversion once they are ready to do hardware operations.
 *
 * This is provided over the omap_pmic structure.
 */
struct omap_pmic_ops {
	int (*vsel_to_uv) (struct omap_pmic *pmic, u8 vsel, u32 *uv);
	int (*uv_to_vsel) (struct omap_pmic *pmic, u32 uv, u8 *vsel);
};

/**
 * struct omap_pmic_controller_ops - regulator operations implemented
 * @devm_pmic_register: managed registration of an PMIC device with a specific
 *			voltage processor. Voltage processor provides an device
 *			handle which is remaining operations.
 *			NOTE:
 *			- This will be first interface to be invoked by
 *			omap_pmic regulator.
 *			- if the underlying layers are not ready, this is
 *			expected to return -EPROBE_DEFER
 *			- if failure, appropriate error code is expected.
 *			- This is expected to be a managed device to avoid
 *			explicit cleanup operations
 *			- Once this succeeds, this returns the pointer to
 *			the controller device and all other operations are
 *			expected to be ready for functionality.
 * @voltage_set:	set the voltage - expected to be synchronous.
 * @voltage_get:	get the current voltage in micro-volts set on PMIC.
 * @voltage_get_range:	Get minimum and maxium permissible operational voltage
 *			range for the device - used to set initial regulator
 *			constraints.
 *
 * These voltage processor interfaces are registered by voltage processor driver
 * using omap_pmic_register_controller_ops. This allows the omap_pmic driver to
 * operate with a specific voltage processor driver.
 */
struct omap_pmic_controller_ops {
	struct device *(*devm_pmic_register) (struct device *dev,
					      struct omap_pmic *pmic);
	int (*voltage_set) (struct device *control_dev, u32 uv);
	int (*voltage_get) (struct device *control_dev, u32 *uv);
	int (*voltage_get_range) (struct device *control_dev, u32 *min_uv,
				  u32 *max_uv);
};

/**
 * struct omap_pmic_info - PMIC information
 *
 * @slave_addr:		7 bit address representing I2C slave address.
 * @voltage_reg_addr:	I2C register address for setting voltage
 * @cmd_reg_addr:	I2C register address for low power transition commands
 * @i2c_timeout_us:	worst case latency for I2C operations for the device
 * @slew_rate_uV:	Slew rate in uV/uSeconds for voltage transitions
 * @step_size_uV:	Step size in uV for one vsel increment.
 * @min_uV:		represents the minimum step_sized incremental voltage
 * @max_uV:		represents the maximum step_sized incremental voltage
 * @voltage_selector_setbits:  what bits to set permenantly for the PMIC
 *			voltage selector - this may have PMIC specific meaning.
 * @voltage_selector_mask: what mask to use for the vsel value - this is useful
 *			for PMICs where the vsel has to be applied at an offset.
 * @voltage_selector_offset: what offset to apply to conversion routine when
 *			operating on vsel.
 * @voltage_selector_zero: Special case handling if 0 value does NOT indicates
 *			power-off for PMIC.
 * @setup_command_list:	array of setup commands for PMIC to operate
 * @setup_num_commands:	number of setup commands for PMIC to operate
 */
struct omap_pmic_info {
	u8 slave_addr;
	u8 voltage_reg_addr;
	u8 cmd_reg_addr;
	u32 i2c_timeout_us;

	u32 slew_rate_uV;
	u32 step_size_uV;
	u32 min_uV;
	u32 max_uV;

	u8 voltage_selector_offset;
	u8 voltage_selector_mask;
	u8 voltage_selector_setbits;
	bool voltage_selector_zero;

	const struct omap_pmic_setup_commands *setup_command_list;
	u8 setup_num_commands;
};

/**
 * struct omap_pmic - represents the OMAP PMIC device.
 * @ops:		PMIC conversion operations.
 *
 * NOTE: the fields marked Private are meant for PMIC driver.
 */
struct omap_pmic {
	const struct omap_pmic_info *info;
	u32 boot_voltage_uV;

	struct omap_pmic_ops *ops;

	struct device *dev;
	struct device *v_dev;
};

#if IS_ENABLED(CONFIG_REGULATOR_TI_OMAP_PMIC)
int omap_pmic_register_controller_ops(struct omap_pmic_controller_ops *cops);
#else
static inline int omap_pmic_register_controller_ops(struct
						    omap_pmic_controller_ops
						    *cops)
{
	return -EINVAL;
}
#endif

#endif				/* __POWER_OMAP_PMIC_H */
