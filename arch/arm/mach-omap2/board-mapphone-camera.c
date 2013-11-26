/*
 * arch/arm/mach-omap2/board-mapphone-camera.c
 *
 * Copyright (C) 2009 Google, Inc.
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

#ifdef CONFIG_VIDEO_MT9P012
#include <media/mt9p012.h>
#define MT9P012_XCLK_48MHZ		48000000
#endif
#ifdef CONFIG_VIDEO_CAM_ISE
#include <media/camise.h>
#define CAMISE_XCLK_24MHZ		24000000
#endif
#define CAM_MAX_REGS		5
#define CAM_MAX_REG_NAME_LEN	8
#define SENSOR_POWER_OFF	0
#define SENSOR_POWER_STANDBY	1
static enum v4l2_power mt9p012_previous_power = V4L2_POWER_OFF;
static int cam_reset_gpio   = -1;
static int cam_standby_gpio = -1;

void __init mapphone_camera_init(void)
{
	cam_standby_gpio = get_gpio_by_name("gpio_cam_pwdn");
	cam_reset_gpio = get_gpio_by_name("gpio_cam_reset");
}
#ifdef CONFIG_VIDEO_MT9P012
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

static int mt9p012_sensor_power_set(struct v4l2_int_device *s,
				    enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_device *isp = dev_get_drvdata(vdev->cam->isp);

	switch (power) {

	case V4L2_POWER_OFF:
		printk("V4L2_POWER_OFF\n");
		/* Power Down Sequence */
		if (cam_reset_gpio >= 0) {
			gpio_set_value(cam_reset_gpio, 0);
			/* free reset GPIO */
			gpio_free(cam_reset_gpio);
			cam_reset_gpio = -1;
		}


		/* Release pm constraints */
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 0);
		//omap_pm_set_max_mpu_wakeup_lat(dev, -1);
		/* mt9p012 autofocus module needs the standby
		   put the standby to high after safe mode */
		if (cam_standby_gpio >= 0) {
			gpio_set_value(cam_standby_gpio, 1);
			gpio_free(cam_standby_gpio);
			cam_standby_gpio = -1;
		}
	break;
	case V4L2_POWER_ON:
		printk("V4L2_POWER_ON\n");
		if (mt9p012_previous_power == V4L2_POWER_OFF) {

			/* mt9p012 autofocus module needs the standby
			  put the standby to high after functional mode */
			cam_standby_gpio = get_gpio_by_name("gpio_cam_pwdn");

			/* Set min throughput to:
			 *  2592 x 1944 x 2bpp x 30fps x 3 L3 accesses */
			omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 885735);
			/* Hold a constraint to keep MPU in C1 */
			//omap_pm_set_max_mpu_wakeup_lat(dev, MPU_LATENCY_C1);

			/* Configure ISP */
			isp_configure_interface(vdev->cam->isp, &mt9p012_if_config);

			msleep(5);
		}
		mt9p012_sensor_set_xclk(s, MT9P012_XCLK_48MHZ);
		msleep(3);

		if (mt9p012_previous_power == V4L2_POWER_OFF) {
			/* request for the GPIO's */
			cam_reset_gpio = get_gpio_by_name("gpio_cam_reset");
			if (cam_reset_gpio >= 0) {
				printk(KERN_INFO "cam_reset_gpio %d\n",
					cam_reset_gpio);
				if (gpio_request(cam_reset_gpio,
					"camera reset") != 0) {
					printk(KERN_ERR "Failed to req cam reset\n");
				}
				gpio_direction_output(cam_reset_gpio, 0);
			}
			if (cam_standby_gpio >= 0) {
				if (gpio_request(cam_standby_gpio,
						"camera standby") != 0) {
					printk(KERN_ERR "Failed to req cam standby\n");
				}
				gpio_direction_output(cam_standby_gpio, 0);
				gpio_set_value(cam_standby_gpio, 1);

			}

			/* RESET is active LOW. set HIGH to release reset */
			if (cam_reset_gpio >= 0)
				gpio_set_value(cam_reset_gpio, 1);

			/* give sensor sometime to get out of the reset.
			 * Datasheet says 2400 xclks.
			 */
			msleep(3);
		}
		break;

	case V4L2_POWER_STANDBY:
		printk("V4L2_POWER_STANDBY\n");
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
#endif

#ifdef CONFIG_VIDEO_CAM_ISE

struct camise_capture_size {
	unsigned long width;
	unsigned long height;
};
const static struct camise_capture_size camise_sizes_1[] = {
	{  176, 144 }, /* QCIF for Video Recording */
	{  320, 240 }, /* QVGA for Preview and Video Recording */
	{  352, 288 }, /* CIF for Video Recording */
	{  480, 360 }, /* Bigger Viewfinder */
	{  640, 480 }, /* 4X BINNING */
	{  512, 1024},
	/*{  512, 2048},*/ /* Jpeg Capture Resolution */
	{  640, 2048}, /* JPEG Catpure Resolution */
	{  848, 480 }, /* Support for WVGA preview */
};
const static struct camise_capture_size camise_sizes_2[] = {
	{  176, 144 }, /* QCIF for Video Recording */
	{  320, 240 }, /* QVGA for Preview and Video Recording */
	{  352, 288 }, /* CIF for Video Recording */
	{  480, 360 }, /* Bigger Viewfinder */
	{  640, 480 }, /* 4X BINNING */
	{  512, 1024},
	{  768, 1024}, /* JPEG Catpure Resolution */
};

static struct omap34xxcam_sensor_config camise_cam_hwc = {
	.sensor_isp = 1,
	.capture_mem = PAGE_ALIGN(2048 * 1536 * 2) * 4,
};

static struct isp_interface_config camise_if_config = {
	.ccdc_par_ser = ISP_PARLL,
	.dataline_shift = 0x2, /* 8bit sensor using D11 to D4. */
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.wenlog = ISPCCDC_CFG_WENLOG_AND,
	.cam_mclk = 216000000,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_RG_GB,
	.u.par.par_bridge = 0x3,
	.u.par.par_clk_pol = 0x0,
};

static int camise_get_capture_size(int index,
				unsigned long *w,
				unsigned long *h)
{

	int max_size = ARRAY_SIZE(camise_sizes_1);

	if (index < max_size) {
		*h = camise_sizes_1[index].height;
		*w = camise_sizes_1[index].width;
	}

	return max_size;
}
static int camise_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;
	hwc->u.sensor.sensor_isp = camise_cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = camise_cam_hwc.capture_mem;
	hwc->dev_index = 1;
	hwc->dev_minor = 1;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static void camise_if_configure(struct v4l2_int_device *s)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	isp_configure_interface(vdev->cam->isp, &camise_if_config);
}

static int camise_sensor_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	switch (power) {
	case V4L2_POWER_OFF:
		if (previous_power != V4L2_POWER_OFF) {
			printk(KERN_DEBUG "%s: power off\n", __func__);
			/* Power Down Sequence
			Need to free gpios since other drivers may request them
			*/
			if (cam_standby_gpio >= 0) {
				gpio_set_value(cam_standby_gpio, 1);
				gpio_free(cam_standby_gpio);
				cam_standby_gpio = -1;
			}

			if (cam_reset_gpio >= 0) {
				gpio_set_value(cam_reset_gpio, 0);
				gpio_free(cam_reset_gpio);
				cam_reset_gpio = -1;
			}

			/* turn off ISP clock */
			isp_set_xclk(vdev->cam->isp, 0, OMAP34XXCAM_XCLK_A);
		}
		break;

	case V4L2_POWER_ON:
		printk(KERN_DEBUG "%s: power on\n", __func__);

		if (previous_power == V4L2_POWER_OFF) {

			/* request for the GPIO's */
			if (cam_reset_gpio >= 0) {
				printk(KERN_INFO "cam_reset_gpio %d\n",
					cam_reset_gpio);
				if (gpio_request(cam_reset_gpio,
					"camera reset") != 0) {
					printk(KERN_ERR "Failed to req cam reset\n");
				}
				gpio_direction_output(cam_reset_gpio, 0);
			}

			if (cam_standby_gpio >= 0) {
				printk(KERN_INFO "cam_standby_gpio %d\n",
					cam_standby_gpio);
				if (gpio_request(cam_standby_gpio,
					"camera standby") != 0) {
					printk(KERN_ERR "Failed to req cam standby\n");
				}
				gpio_direction_output(cam_standby_gpio, 0);
				gpio_set_value(cam_standby_gpio, 1);
			}

			/* turn on ISP clock */
			isp_set_xclk(vdev->cam->isp, CAMISE_XCLK_24MHZ, OMAP34XXCAM_XCLK_A);
			/* Power Up Sequence */
			camise_if_configure(s);
			msleep(10);

			if (cam_standby_gpio >= 0) {
				/* Bring camera out of standby */
				gpio_set_value(cam_standby_gpio, 0);
				msleep(10);
			}

			if (cam_reset_gpio >= 0) {
				/* release reset */
				gpio_set_value(cam_reset_gpio, 0);
				msleep(10);
				gpio_set_value(cam_reset_gpio, 1);
				msleep(10);
			}

			/* Wait 20ms per OVT recommendation */
			msleep(20);
		}
		break;
	default:
		break;
	}

	previous_power = power;
	return 0;
}


struct camise_platform_data mapphone_camise_platform_data = {
    .power_set      = camise_sensor_power_set,
    .priv_data_set  = camise_sensor_set_prv_data,
    .if_config      = camise_if_configure,
    .get_size       = camise_get_capture_size,
};

#endif /*CONFIG_VIDEO_CAM_ISE*/
