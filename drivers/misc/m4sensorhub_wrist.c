/*
 * Copyright (C) 2013 Motorola Mobility, Inc.
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

#include "m4sensorhub_wrist.h"
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/m4sensorhub.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/m4sensorhub_gpio.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_gpio.h>

static struct platform_driver m4wrist_client_driver;
static int m4wrist_probe(struct platform_device *pdev);
static int m4wrist_remove(struct platform_device *pdev);
static int m4wrist_init(void);
static void m4wrist_exit(void);
static void m4wrist_free(struct m4wrist_driver_data *dd);
static int m4wrist_gpio_init(struct m4wrist_driver_data *dd);
static int m4wrist_request_firmware(struct m4wrist_driver_data *dd);
static int m4wrist_request_irq(struct m4wrist_driver_data *dd);
static void m4wrist_firmware_callback(const struct firmware *fw,
		void *context);
static void m4wrist_irq(enum m4sensorhub_irqs event, void *context);
static int m4wrist_gpio_control(struct m4wrist_driver_data *dd);
static int m4wrist_gpio_release(struct m4wrist_driver_data *dd);
static int m4wrist_reflash_ic(struct m4wrist_driver_data *dd);
static int m4wrist_enter_reset_mode(struct m4wrist_driver_data *dd);
static int m4wrist_erase_flash(struct m4wrist_driver_data *dd);
static int m4wrist_program_image(struct m4wrist_driver_data *dd);
static void m4wrist_send_bitstream(struct m4wrist_driver_data *dd,
		uint8_t *stream, uint32_t bits);
static void m4wrist_toggle_clock(struct m4wrist_driver_data *dd, int cycles);
static int m4wrist_wait_poll(struct m4wrist_driver_data *dd);
static int m4wrist_read_id_word(struct m4wrist_driver_data *dd, uint8_t *data);


static struct of_device_id m4wrist_match_tbl[] = {
    { .compatible = "mot,m4wrist" },
    {},
};

static struct platform_driver m4wrist_client_driver = {
	.probe		= m4wrist_probe,
	.remove		= m4wrist_remove,
	.driver		= {
		.name	= "m4sensorhub_wrist",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(m4wrist_match_tbl),
	},
};

static int m4wrist_probe(struct platform_device *pdev)
{
	int err = 0;
	struct m4wrist_driver_data *dd = NULL;
	struct device_node *node = pdev->dev.of_node;

	if (!node) {
		pr_warn("devtree node not present!\n");
		return -ENODEV;
	}
	dd = kzalloc(sizeof(struct m4wrist_driver_data), GFP_KERNEL);
	if (dd == NULL) {
		printk(KERN_ERR "%s: Unable to create driver data.\n",
			__func__);
		err = -ENOMEM;
		goto m4wrist_probe_fail;
	}

	dd->pdev = pdev;
	platform_set_drvdata(pdev, dd);

	dd->mutex = kzalloc(sizeof(struct mutex), GFP_KERNEL);
	if (dd->mutex == NULL) {
		printk(KERN_ERR "%s: Unable to create mutex lock.\n",
			__func__);
		err = -ENOMEM;
		goto m4wrist_probe_fail;
	}
	mutex_init(dd->mutex);

	dd->gpio_xres = of_get_named_gpio_flags(node,
			"mot,wrist_xres", 0, NULL);
	if (dd->gpio_xres <= 0) {
		printk(KERN_ERR "%s: gpio_xres is invalid.\n", __func__);
		err = -EINVAL;
		goto m4wrist_probe_fail;
	}
	dd->gpio_clk = of_get_named_gpio_flags(node, "mot,wrist_clk", 0, NULL);
	if (dd->gpio_clk <= 0) {
		printk(KERN_ERR "%s: gpio_clk is invalid.\n", __func__);
		err = -EINVAL;
		goto m4wrist_probe_fail;
	}
	dd->gpio_data = of_get_named_gpio_flags(node,
			"mot,wrist_data", 0, NULL);
	if (dd->gpio_data <= 0) {
		printk(KERN_ERR "%s: gpio_data is invalid.\n", __func__);
		err = -EINVAL;
		goto m4wrist_probe_fail;
	}

	err = m4wrist_gpio_init(dd);
	if (err < 0)
		goto m4wrist_probe_fail;

	err = m4wrist_request_firmware(dd);
	if (err < 0)
		goto m4wrist_probe_fail;

	err = m4wrist_request_irq(dd);
	if (err < 0)
		goto m4wrist_probe_fail;

	goto m4wrist_probe_pass;

m4wrist_probe_fail:
	m4wrist_free(dd);
	printk(KERN_ERR "%s: Probe failed with error code %d.\n",
		__func__, err);
	return err;

m4wrist_probe_pass:
	return 0;
}

static void m4wrist_free(struct m4wrist_driver_data *dd)
{
	if (dd != NULL) {
		kfree(dd->img);

		if (dd->client != NULL) {
			m4sensorhub_irq_disable(dd->client,
				M4SH_IRQ_WRIST_READY);
			m4sensorhub_irq_unregister(dd->client,
				M4SH_IRQ_WRIST_READY);
		}

		kfree(dd->mutex);
		platform_set_drvdata(dd->pdev, NULL);
		kfree(dd);
	}

	return;
}

static int m4wrist_gpio_init(struct m4wrist_driver_data *dd)
{
	int err = 0;
	int i = 0;
	int gpio_nums[3] = {dd->gpio_xres, dd->gpio_clk, dd->gpio_data};
	char *gpio_names[3] = {"wrist_xres", "wrist_clk", "wrist_data"};

	for (i = 0; i < 3; i++) {
		err = gpio_request(gpio_nums[i], gpio_names[i]);
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to request %s.\n",
				__func__, gpio_names[i]);
			i--;
			goto m4wrist_gpio_init_fail;
		}

		err = gpio_direction_input(gpio_nums[i]);
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to make %s an input.\n",
				__func__, gpio_names[i]);
			gpio_free(gpio_nums[i]);
			i--;
			goto m4wrist_gpio_init_fail;
		}
	}

	goto m4wrist_gpio_init_exit;

m4wrist_gpio_init_fail:
	while (i >= 0) {
		gpio_free(gpio_nums[i]);
		i--;
	}

m4wrist_gpio_init_exit:
	return err;
}

static int m4wrist_request_firmware(struct m4wrist_driver_data *dd)
{
	int err = 0;
	const struct firmware *fw = NULL;

	err = request_firmware(&fw,
		"m4sensorhub_wrist.bin", &(dd->pdev->dev));
	if (err < 0) {
		printk(KERN_ERR "%s: Firmware request failed.\n", __func__);
		goto m4wrist_request_firmware_fail;
	}

	m4wrist_firmware_callback(fw, dd);

m4wrist_request_firmware_fail:
	return err;
}

static int m4wrist_request_irq(struct m4wrist_driver_data *dd)
{
	int err = 0;

	dd->client = m4sensorhub_client_get_drvdata();
	if (dd->client == NULL) {
		printk(KERN_ERR "%s: No client data retrieved.\n",
			__func__);
		err = -ENODATA;
		goto m4wrist_request_irq_fail;
	}

	err = m4sensorhub_irq_register(dd->client,
		M4SH_IRQ_WRIST_READY, m4wrist_irq, dd);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to register IRQ.\n", __func__);
		dd->client = NULL;
		goto m4wrist_request_irq_fail;
	}

	err = m4sensorhub_irq_enable(dd->client, M4SH_IRQ_WRIST_READY);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to enable IRQ.\n", __func__);
		m4sensorhub_irq_unregister(dd->client, M4SH_IRQ_WRIST_READY);
		dd->client = NULL;
		goto m4wrist_request_irq_fail;
	}

m4wrist_request_irq_fail:
	return err;
}

static void m4wrist_firmware_callback(const struct firmware *fw,
		void *context)
{
	struct m4wrist_driver_data *dd = context;

	mutex_lock(dd->mutex);

	if (fw == NULL) {
		printk(KERN_ERR "%s: No firmware received.\n", __func__);
		goto m4wrist_firmware_callback_fail;
	} else if (fw->data == NULL || fw->size == 0) {
		printk(KERN_ERR "%s: No data found.\n", __func__);
		goto m4wrist_firmware_callback_fail;
	} else if (fw->data[0] < 4) {
		printk(KERN_ERR "%s: Firmware header is too small.\n",
			__func__);
		goto m4wrist_firmware_callback_fail;
	} else if (fw->data[0] >= fw->size) {
		printk(KERN_ERR "%s: Firmware data is missing.\n", __func__);
		goto m4wrist_firmware_callback_fail;
	}

	dd->size = fw->size - (fw->data[0] + 1);
	dd->img = kzalloc(dd->size * sizeof(uint8_t), GFP_KERNEL);
	if (dd->img == NULL) {
		printk(KERN_ERR "%s: Failed to allocate memory for firmware.\n",
			__func__);
		goto m4wrist_firmware_callback_fail;
	}
	memcpy(dd->img, &(fw->data[fw->data[0] + 1]), dd->size);
	dd->si_id[0] = fw->data[1];
	dd->si_id[1] = fw->data[2];
	dd->fw_ver[0] = fw->data[3];
	dd->fw_ver[1] = fw->data[4];

m4wrist_firmware_callback_fail:
	release_firmware(fw);
	mutex_unlock(dd->mutex);
	return;
}

static int m4wrist_remove(struct platform_device *pdev)
{
	struct m4wrist_driver_data *dd = NULL;

	dd = platform_get_drvdata(pdev);
	if (dd != NULL) {
		gpio_free(dd->gpio_xres);
		gpio_free(dd->gpio_clk);
		gpio_free(dd->gpio_data);
		m4wrist_free(dd);
	}

	return 0;
}

static int m4wrist_init(void)
{
	return platform_driver_register(&m4wrist_client_driver);
}

static void m4wrist_exit(void)
{
	platform_driver_unregister(&m4wrist_client_driver);
}

module_init(m4wrist_init);
module_exit(m4wrist_exit);

static void m4wrist_irq(enum m4sensorhub_irqs event, void *context)
{
	struct m4wrist_driver_data *dd = context;
	int err = 0;
	uint8_t irq_reason = 0x00;
	uint8_t val[2] = {0x00, 0x00};
	uint8_t mask[2] = {0xFF, 0xFF};
	mutex_lock(dd->mutex);

	if (dd->img == NULL) {
		printk(KERN_ERR "%s: Firmware image is missing--%s.\n",
			__func__, "unable to respond to interrupts");
		err = -ENODATA;
		goto m4wrist_irq_fail;
	}

	err = m4sensorhub_reg_read(dd->client,
		M4SH_REG_WRIST_INTERRUPTREASON, &irq_reason);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to read interrupt reason.\n",
			__func__);
		goto m4wrist_irq_fail;
	} else if (err < 1) {
		printk(KERN_ERR "%s: Read %d bytes instead of 1.\n",
			__func__, err);
		err = -EINVAL;
		goto m4wrist_irq_fail;
	}

	switch (irq_reason) {
	case 0x00:
		err = m4wrist_gpio_control(dd);
		if (err < 0) {
			printk(KERN_ERR
				"%s: Failed to take control of GPIO lines.\n",
				__func__);
			goto m4wrist_irq_fail;
		}

		err = m4wrist_reflash_ic(dd);
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to reflash IC.\n",
				__func__);
			goto m4wrist_irq_fail;
		}

		err = m4wrist_gpio_release(dd);
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to release GPIO lines.\n",
				__func__);
			goto m4wrist_irq_fail;
		}

		val[0] = 0x01;
		err = m4sensorhub_reg_write(dd->client,
			M4SH_REG_WRIST_HOSTRESPONSE, &(val[0]), &(mask[0]));
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to write host response.\n",
				__func__);
			goto m4wrist_irq_fail;
		}  else if (err < 1) {
			printk(KERN_ERR "%s: Wrote %d bytes instead of 1.\n",
				__func__, err);
			err = -EINVAL;
			goto m4wrist_irq_fail;
		}
		break;

	case 0x01:
		val[0] = dd->fw_ver[0];
		val[1] = dd->fw_ver[1];
		err = m4sensorhub_reg_write(dd->client,
			M4SH_REG_WRIST_FMONFILE, &(val[0]), &(mask[0]));
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to write firmware version.\n",
				__func__);
			goto m4wrist_irq_fail;
		}  else if (err < 2) {
			printk(KERN_ERR "%s: Wrote %d bytes instead of 2.\n",
				__func__, err);
			err = -EINVAL;
			goto m4wrist_irq_fail;
		}
		break;

	default:
		printk(KERN_ERR "%s: Unexpected interrupt 0x%02X received.\n",
			__func__, irq_reason);
		err = -EINVAL;
		goto m4wrist_irq_fail;
		break;
	}

	goto m4wrist_irq_pass;

m4wrist_irq_fail:
	printk(KERN_ERR "%s: IRQ handler failed with error code %d.\n",
		__func__, err);

m4wrist_irq_pass:
	mutex_unlock(dd->mutex);
	return;
}

static int m4wrist_gpio_control(struct m4wrist_driver_data *dd)
{
	int err = 0;
	int i = 0;
	int gpio_nums[3] = {dd->gpio_xres, dd->gpio_clk, dd->gpio_data};
	char *gpio_names[3] = {"wrist_xres", "wrist_clk", "wrist_data"};

	for (i = 0; i < 3; i++) {
		err = gpio_direction_output(gpio_nums[i], 0);
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to take control of %s.\n",
				__func__, gpio_names[i]);
			goto m4wrist_gpio_control_fail;
		}
	}

m4wrist_gpio_control_fail:
	return err;
}

static int m4wrist_gpio_release(struct m4wrist_driver_data *dd)
{
	int err = 0;
	int i = 0;
	int gpio_nums[3] = {dd->gpio_xres, dd->gpio_clk, dd->gpio_data};
	char *gpio_names[3] = {"wrist_xres", "wrist_clk", "wrist_data"};

	for (i = 0; i < 3; i++) {
		err = gpio_direction_input(gpio_nums[i]);
		if (err < 0) {
			printk(KERN_ERR "%s: Failed to release %s.\n",
				__func__, gpio_names[i]);
			goto m4wrist_gpio_release_fail;
		}
	}

m4wrist_gpio_release_fail:
	return err;
}

static int m4wrist_reflash_ic(struct m4wrist_driver_data *dd)
{
	int err = 0;

	err = m4wrist_enter_reset_mode(dd);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to enter programming mode.\n",
			__func__);
		goto m4wrist_reflash_ic_fail;
	}

	err = m4wrist_erase_flash(dd);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to erase IC flash.\n", __func__);
		goto m4wrist_reflash_ic_fail;
	}

	err = m4wrist_program_image(dd);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to program firmware image",
			__func__);
		goto m4wrist_reflash_ic_fail;
	}

	gpio_set_value(dd->gpio_xres, 1);
	udelay(100);
	gpio_set_value(dd->gpio_xres, 0);

m4wrist_reflash_ic_fail:
	return err;
}

static int m4wrist_enter_reset_mode(struct m4wrist_driver_data *dd)
{
	int err = 0;
	uint8_t silicon_id[2] = {0x00, 0x00};

	msleep(20);
	gpio_set_value(dd->gpio_xres, 1);
	udelay(400);
	gpio_set_value(dd->gpio_xres, 0);
	udelay(1);

	m4wrist_send_bitstream(dd, &(m4wrist_id_setup_1[0]),
		M4WRIST_ID_SETUP_1_BITS);

	err = m4wrist_wait_poll(dd);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to wait-and-poll 1.\n", __func__);
		goto m4wrist_enter_reset_mode_fail;
	}

	m4wrist_send_bitstream(dd, &(m4wrist_id_setup_2[0]),
		M4WRIST_ID_SETUP_2_BITS);

	err = m4wrist_wait_poll(dd);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to wait-and-poll 2.\n", __func__);
		goto m4wrist_enter_reset_mode_fail;
	}

	m4wrist_send_bitstream(dd, &(m4wrist_sync_enable[0]),
		M4WRIST_SYNC_ENABLE_BITS);

	err = m4wrist_read_id_word(dd, &(silicon_id[0]));
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to read silicon ID\n", __func__);
		goto m4wrist_enter_reset_mode_fail;
	}

	m4wrist_send_bitstream(dd, &(m4wrist_sync_disable[0]),
		M4WRIST_SYNC_DISABLE_BITS);

	if (silicon_id[0] != dd->si_id[0] || silicon_id[1] != dd->si_id[1]) {
		printk(KERN_ERR "%s: Silicon ID mismatch (read 0x%02X%02X).\n",
			__func__, silicon_id[0], silicon_id[1]);
		err = -EINVAL;
		goto m4wrist_enter_reset_mode_fail;
	}

m4wrist_enter_reset_mode_fail:
	return err;
}

static int m4wrist_erase_flash(struct m4wrist_driver_data *dd)
{
	int err = 0;

	m4wrist_send_bitstream(dd, &(m4wrist_erase[0]),
		M4WRIST_ERASE_BITS);

	err = m4wrist_wait_poll(dd);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to wait-and-poll.\n", __func__);
		goto m4wrist_erase_flash_fail;
	}

m4wrist_erase_flash_fail:
	return err;
}

static int m4wrist_program_image(struct m4wrist_driver_data *dd)
{
	int err = 0;
	uint32_t iter = 0;
	int i = 0;
	uint8_t blk_num = 0x00;
	uint8_t cur_addr = 0x00;
	uint8_t cur_vector[3] = {0x00, 0x00, 0x00};
	uint8_t cur_blk_data[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t status[2] = {0x00, 0x00};

	if (((dd->size % 128) != 0) || (dd->size == 0)) {
		printk(KERN_ERR "%s: Firmware has invalid size of %u.\n",
			__func__, dd->size);
		err = -EINVAL;
		goto m4wrist_program_image_fail;
	}

	printk(KERN_INFO "%s: Flashing version 0x%02X 0x%02X...\n", __func__,
		dd->fw_ver[0], dd->fw_ver[1]);

	while (iter < (dd->size - 1)) {
		m4wrist_send_bitstream(dd, &(m4wrist_sync_enable[0]),
			M4WRIST_SYNC_ENABLE_BITS);

		m4wrist_send_bitstream(dd, &(m4wrist_read_write_setup[0]),
			M4WRIST_READ_WRITE_SETUP_BITS);

		cur_addr = 0x00;
		for (i = 0; i < 128; i++) {
			cur_vector[0] = 0x90 | ((cur_addr & 0x78) >> 3);
			cur_vector[1] = ((cur_addr & 0x07) << 5) |
				((dd->img[iter] & 0xF8) >> 3);
			cur_vector[2] = ((dd->img[iter] & 0x07) << 5) | 0x1C;
			m4wrist_send_bitstream(dd, &(cur_vector[0]), 22);

			cur_addr++;
			iter++;
		}

		m4wrist_send_bitstream(dd, &(m4wrist_sync_enable[0]),
			M4WRIST_SYNC_ENABLE_BITS);

		cur_blk_data[0] = 0xDE;
		cur_blk_data[1] = 0xE0;
		cur_blk_data[2] = 0x1E;
		cur_blk_data[3] = 0x7D;
		cur_blk_data[4] = (blk_num & 0xFE) >> 1;
		cur_blk_data[5] = ((blk_num & 0x01) << 7) | 0x70;
		m4wrist_send_bitstream(dd, &(cur_blk_data[0]), 44);

		m4wrist_send_bitstream(dd, &(m4wrist_sync_disable[0]),
			M4WRIST_SYNC_DISABLE_BITS);

		m4wrist_send_bitstream(dd, &(m4wrist_program_and_verify[0]),
			M4WRIST_PROGRAM_AND_VERIFY_BITS);

		err = m4wrist_wait_poll(dd);
		if (err < 0) {
			printk(KERN_ERR "%s: %s for block 0x%02X.\n",
				__func__, "Failed to wait-and-poll", blk_num);
			goto m4wrist_program_image_fail;
		}

		m4wrist_send_bitstream(dd, &(m4wrist_sync_enable[0]),
			M4WRIST_SYNC_ENABLE_BITS);

		err = m4wrist_read_id_word(dd, &(status[0]));
		if (err < 0) {
			printk(KERN_ERR "%s: %s for block 0x%02X.\n",
				__func__, "Failed to read status", blk_num);
		}

		m4wrist_send_bitstream(dd, &(m4wrist_sync_disable[0]),
			M4WRIST_SYNC_DISABLE_BITS);

		if (status[0] != 0x00) {
			printk(KERN_ERR "%s: %s 0x%02X %s 0x%02X.\n", __func__,
				"Programming block", blk_num,
				"failed with error code", status[0]);
			err = -EINVAL;
			goto m4wrist_program_image_fail;
		}

		blk_num++;
	}

m4wrist_program_image_fail:
	return err;
}

static void m4wrist_send_bitstream(struct m4wrist_driver_data *dd,
		uint8_t *stream, uint32_t bits)
{
	int i = 0;
	int j = 0;
	uint32_t bits_sent = 0;

	j = 7;
	while (bits_sent < bits) {
		if (stream[i] & (0x01 << j))
			gpio_set_value(dd->gpio_data, 1);
		else
			gpio_set_value(dd->gpio_data, 0);

		gpio_set_value(dd->gpio_clk, 1);
		gpio_set_value(dd->gpio_clk, 0);

		bits_sent++;
		j--;
		if (j < 0) {
			j = 7;
			i++;
		}
	}

	return;
}

static void m4wrist_toggle_clock(struct m4wrist_driver_data *dd, int cycles)
{
	int i = 0;

	for (i = 0; i < cycles; i++) {
		gpio_set_value(dd->gpio_clk, 1);
		gpio_set_value(dd->gpio_clk, 0);
	}

	return;
}

static int m4wrist_wait_poll(struct m4wrist_driver_data *dd)
{
	int err = 0;
	int i = 0;
	bool saw_event = false;
	uint8_t clear[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

	err = gpio_direction_input(dd->gpio_data);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to release data line.\n", __func__);
		goto m4wrist_wait_poll_fail;
	}

	udelay(1);

	for (i = 0; i < 200000; i++) {
		if (gpio_get_value(dd->gpio_data) == 1) {
			saw_event = true;
			break;
		} else {
			m4wrist_toggle_clock(dd, 1);
		}
	}

	if (!saw_event) {
		printk(KERN_ERR "%s: Timeout waiting for data high.\n",
			__func__);
		err = -ETIME;
		goto m4wrist_wait_poll_fail;
	}

	saw_event = false;
	for (i = 0; i < 200000; i++) {
		if (gpio_get_value(dd->gpio_data) == 0) {
			saw_event = true;
			break;
		} else {
			udelay(1);
		}
	}

	if (!saw_event) {
		printk(KERN_ERR "%s: Timeout waiting for data low.\n",
			__func__);
		err = -ETIME;
		goto m4wrist_wait_poll_fail;
	}

	err = gpio_direction_output(dd->gpio_data, 0);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to acquire data line.\n",
			__func__);
		goto m4wrist_wait_poll_fail;
	}

	m4wrist_send_bitstream(dd, &(clear[0]), 40);

m4wrist_wait_poll_fail:
	return err;
}

static int m4wrist_read_id_word(struct m4wrist_driver_data *dd, uint8_t *data)
{
	int err = 0;
	int i = 0;
	int bit = 0;
	uint8_t stream1[2] = {0xBF, 0x00};
	uint8_t stream2[2] = {0xDF, 0x90};

	m4wrist_send_bitstream(dd, &(stream1[0]), 11);

	err = gpio_direction_input(dd->gpio_data);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to release initial data line.\n",
			__func__);
		goto m4wrist_read_id_word_fail;
	}

	m4wrist_toggle_clock(dd, 2);

	data[0] = 0x00;
	for (i = 0; i <= 7; i++) {
		gpio_set_value(dd->gpio_clk, 1);
		data[0] = (data[0] << 1);
		bit = gpio_get_value(dd->gpio_data);
		if (bit == 1)
			(data[0])++;

		gpio_set_value(dd->gpio_clk, 0);
	}

	gpio_set_value(dd->gpio_clk, 1);

	err = gpio_direction_output(dd->gpio_data, 1);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to acquire initial data line.\n",
			__func__);
		goto m4wrist_read_id_word_fail;
	}

	m4wrist_send_bitstream(dd, &(stream2[0]), 12);

	err = gpio_direction_input(dd->gpio_data);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to release final data line.\n",
			__func__);
		goto m4wrist_read_id_word_fail;
	}

	m4wrist_toggle_clock(dd, 2);

	data[1] = 0x00;
	for (i = 0; i <= 7; i++) {
		gpio_set_value(dd->gpio_clk, 1);
		data[1] = (data[1] << 1);
		bit = gpio_get_value(dd->gpio_data);
		if (bit == 1)
			(data[1])++;

		gpio_set_value(dd->gpio_clk, 0);
	}

	m4wrist_toggle_clock(dd, 1);

	err = gpio_direction_output(dd->gpio_data, 0);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed to acquire final data line.\n",
			__func__);
		goto m4wrist_read_id_word_fail;
	}

	m4wrist_toggle_clock(dd, 1);

m4wrist_read_id_word_fail:
	return err;
}

MODULE_LICENSE("GPL");
