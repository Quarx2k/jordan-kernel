/*
 * arch/arm/mach-omap2/board-mapphone-keypad.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2010 Motorola, Inc.
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
 */

#include <linux/platform_device.h>
#include <media/mt9p012.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>
#include <linux/mm.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio_mapping.h>
#include <linux/gpio.h>
#include <plat/omap-pm.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#include "dt_path.h"
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <mach/board-mapphone.h>

#define MT9P012_XCLK_48MHZ	48000000
#define CAM_MAX_REGS		5
#define CAM_MAX_REG_NAME_LEN	8
#define SENSOR_POWER_OFF	0
#define SENSOR_POWER_STANDBY	1
static enum v4l2_power mt9p012_previous_power = V4L2_POWER_OFF;
static int  mapphone_camera_reg_power(bool);
static char regulator_list[CAM_MAX_REGS][CAM_MAX_REG_NAME_LEN];
static int cam_reset_gpio   = -1;
static int cam_standby_gpio = -1;

void __init mapphone_camera_init(void)
{
	cam_standby_gpio = get_gpio_by_name("gpio_cam_pwdn");
	cam_reset_gpio = get_gpio_by_name("gpio_cam_reset");
}

static struct omap34xxcam_sensor_config cam_hwc = {
    .sensor_isp = 0,
    .capture_mem = PAGE_ALIGN(2592 * 1944 * 2) * 4,
    .ival_default    = { 1, 10 },
};
 
static int mt9p012_sensor_set_prv_data(struct v4l2_int_device *s, void *priv)
{
    struct omap34xxcam_hw_config *hwc = priv;
 
    hwc->u.sensor.sensor_isp = cam_hwc.sensor_isp;
    hwc->u.sensor.capture_mem = cam_hwc.capture_mem;
    hwc->dev_index = 0;
    hwc->dev_minor = 0;
    hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
    return 0;
}
 
static struct isp_interface_config mt9p012_if_config = {
    .ccdc_par_ser	= ISP_PARLL,
    .dataline_shift	= 0x1,
    .hsvs_syncdetect	= ISPCTRL_SYNC_DETECT_VSRISE,
    .strobe		= 0x0,
    .prestrobe		= 0x0,
    .shutter		= 0x0,
    .wenlog		 = ISPCCDC_CFG_WENLOG_OR,
    .wait_hs_vs		= 2,
    .cam_mclk		= 144000000,
    .u.par.par_bridge	= 0x0,
    .u.par.par_clk_pol	= 0x0,
};

static u32 mt9p012_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
    struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
 
    return isp_set_xclk(vdev->cam->isp, xclkfreq, OMAP34XXCAM_XCLK_A);
}

int mapphone_camera_reg_power(bool enable)
{
	static struct regulator *regulator[CAM_MAX_REGS];
	static bool reg_resource_acquired;
	int i, error;

	error = 0;
	printk("enter to mapphone_camera_reg_power\n");
	if (reg_resource_acquired == false && enable) {
		/* get list of regulators and enable*/
		for (i = 0; i < CAM_MAX_REGS && \
			regulator_list[i][0] != 0; i++) {
			printk(KERN_INFO "%s - enable %s\n",\
				__func__,\
				regulator_list[i]);
			regulator[i] = regulator_get(NULL, regulator_list[i]);
			if (IS_ERR(regulator[i])) {
				pr_err("%s: Cannot get %s "\
					"regulator, err=%ld\n",\
					__func__, regulator_list[i],
					PTR_ERR(regulator[i]));
				error = PTR_ERR(regulator[i]);
				regulator[i] = NULL;
				break;
			}
			if (regulator_enable(regulator[i]) != 0) {
				pr_err("%s: Cannot enable regulator: %s \n",
					__func__, regulator_list[i]);
				error = -EIO;
				regulator_put(regulator[i]);
				regulator[i] = NULL;
				break;
			}
		}

		if (error != 0 && i > 0) {
			/* return all acquired regulator resources if error */
			while (--i && regulator[i]) {
				regulator_disable(regulator[i]);
				regulator_put(regulator[i]);
				regulator[i] = NULL;
			}
		} else
			reg_resource_acquired = true;

	} else if (reg_resource_acquired && !enable) {
		/* get list of regulators and disable*/
		for (i = 0; i < CAM_MAX_REGS && \
			regulator_list[i][0] != 0; i++) {
			printk(KERN_INFO "%s - disable %s\n",\
					 __func__,\
					 regulator_list[i]);
			if (regulator[i]) {
				regulator_disable(regulator[i]);
				regulator_put(regulator[i]);
				regulator[i] = NULL;
			}
		}

		reg_resource_acquired = false;
	} else {
		pr_err("%s: Invalid regulator state\n", __func__);
		error = -EIO;
    }

    return error;

}

static int mt9p012_sensor_power_set(struct v4l2_int_device *s,
				    enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);

	int error = 0;
	printk("enter to mt9p012_sensor_power_set\n");

	switch (power) {

	case V4L2_POWER_OFF:
		printk("V4L2_POWER_OFF\n");
		/* Turn off power */
		error = mapphone_camera_reg_power(false);
		if (error != 0) {
			pr_err("%s: Failed to power off regulators\n",
				__func__);
		}

		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 0);

		if (mt9p012_previous_power == V4L2_POWER_ON)
			isp_disable_mclk(isp);

	break;
	case V4L2_POWER_ON:
		printk("V4L2_POWER_ON\n");
		/* Through-put requirement:
		 * 2592 x 1944 x 2Bpp x 11fps x 3 memory ops = 324770 KByte/s
		 */
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 324770);

		/* Power Up Sequence */
		isp_configure_interface(vdev->cam->isp, &mt9p012_if_config);

		/* set to output mode */
		gpio_direction_output(cam_standby_gpio, true);
		/* set to output mode */
		gpio_direction_output(cam_reset_gpio, true);

		/* STANDBY_GPIO is active HIGH for set LOW to release */
		gpio_set_value(cam_standby_gpio, 1);

		/* nRESET is active LOW. set HIGH to release reset */
		gpio_set_value(cam_reset_gpio, 1);

		/* turn on digital power */
		//enable_fpga_vio_1v8(1);

		/* turn on analog power */
		error = mapphone_camera_reg_power(true);
		if (error != 0) {
			pr_err("%s: Failed to power on regulators\n",
				__func__);
		}

		/* out of standby */
		gpio_set_value(cam_standby_gpio, 0);
		udelay(1000);

		/* have to put sensor to reset to guarantee detection */
		gpio_set_value(cam_reset_gpio, 0);

		udelay(1500);

		/* nRESET is active LOW. set HIGH to release reset */
		gpio_set_value(cam_reset_gpio, 1);
		/* give sensor sometime to get out of the reset.
		 * Datasheet says 2400 xclks. At 6 MHz, 400 usec is
		 * enough
		 */
		udelay(300);
		break;
	case V4L2_POWER_STANDBY:
		/* stand by */
		gpio_set_value(cam_standby_gpio, 1);
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 0);
		if (mt9p012_previous_power == V4L2_POWER_ON)
			isp_disable_mclk(isp);
		break;
	}
	/* Save powerstate to know what was before calling POWER_ON. */
	mt9p012_previous_power = power;
	return 0;
}


struct mt9p012_platform_data mapphone_mt9p012_platform_data = {
	.power_set = mt9p012_sensor_power_set,
	.priv_data_set  = mt9p012_sensor_set_prv_data,
	.set_xclk = mt9p012_sensor_set_xclk,
};
