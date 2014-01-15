/*
 * linux/arch/arm/mach-omap2/board-mapphone-camera.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * Derived from mach-omap3/board-3430sdp.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <plat/mux.h>
#include <plat/board-mapphone.h>
#include <plat/omap-pm.h>
#include <plat/control.h>
#include <linux/string.h>
#include <linux/gpio_mapping.h>
#include <plat/resource.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#if defined(CONFIG_VIDEO_OMAP3)
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#include <../drivers/media/video/isp/isp.h>
#include <../drivers/media/video/isp/ispcsi2.h>
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#include <media/mt9p012.h>
#define MT9P012_XCLK_48MHZ		48000000
#endif
#ifdef CONFIG_VIDEO_CAM_ISE
#include <media/camise.h>
#define CAMISE_XCLK_24MHZ		24000000
#endif

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
#include <../drivers/media/video/hplens.h>
#endif
#endif

#define CAM_MAX_REGS 5
#define CAM_MAX_REG_NAME_LEN 8
#define SENSOR_POWER_OFF     0
#define SENSOR_POWER_STANDBY 1

/* devtree flash */
static enum v4l2_power previous_power = V4L2_POWER_OFF;
static int cam_reset_gpio   = -1;
static int cam_standby_gpio = -1;
static u8 cam_flags;

static void mapphone_lock_cpufreq(int lock)
{
	static struct device ov_dev;
	static int flag;

	if (lock == 1) {
		resource_request("vdd1_opp",
			&ov_dev, omap_pm_get_max_vdd1_opp());
		flag = 1;
	} else {
		if (flag == 1) {
			resource_release("vdd1_opp", &ov_dev);
			flag = 0;
		}
	}
}


#ifdef CONFIG_VIDEO_OMAP3_HPLENS
static int hplens_power_set(enum v4l2_power power)
{
	(void)power;

	return 0;
}

static int hplens_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_LENS;

	return 0;
}

struct hplens_platform_data mapphone_hplens_platform_data = {
	.power_set = hplens_power_set,
	.priv_data_set = hplens_set_prv_data,
};
#endif

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
static struct omap34xxcam_sensor_config mt9p012_cam_hwc = {
	.sensor_isp = 0,
	.xclk = OMAP34XXCAM_XCLK_A,
	.capture_mem = PAGE_ALIGN(2592 * 1944 * 2) * 4,
};

static int mt9p012_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.xclk = mt9p012_cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = mt9p012_cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = mt9p012_cam_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static struct isp_interface_config mt9p012_if_config = {
	.ccdc_par_ser = ISP_PARLL,
	.dataline_shift = 0x1,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.wenlog = ISPCCDC_CFG_WENLOG_OR,
	.wait_bayer_frame = 0,
	.wait_yuv_frame = 1,
	.dcsub = 42,
	.cam_mclk = 144000000,
	.cam_mclk_src_div = 6,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_GR_BG,
	.u.par.par_bridge = 0x0,
	.u.par.par_clk_pol = 0x0,
};

u32 mt9p012_set_xclk(u32 xclkfreq)
{
	return isp_set_xclk(xclkfreq, OMAP34XXCAM_XCLK_A);
}

static int mt9p012_sensor_power_set(struct device* dev, enum v4l2_power power)
{
	switch (power) {
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		if (cam_reset_gpio >= 0) {
			gpio_set_value(cam_reset_gpio, 0);
			/* free reset GPIO */
			gpio_free(cam_reset_gpio);
			cam_reset_gpio = -1;
		}

		mt9p012_set_xclk(0);

		msleep(1);

		/* Release pm constraints */
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);
		omap_pm_set_max_mpu_wakeup_lat(dev, -1);
		/* mt9p012 autofocus module needs the standby
		   put the standby to high after safe mode */
		if (cam_standby_gpio >= 0) {
			gpio_set_value(cam_standby_gpio, 1);
			gpio_free(cam_standby_gpio);
			cam_standby_gpio = -1;
		}
	break;
	case V4L2_POWER_ON:
		if (previous_power == V4L2_POWER_OFF) {

			/* mt9p012 autofocus module needs the standby
			  put the standby to high after functional mode */
			cam_standby_gpio = get_gpio_by_name("gpio_cam_pwdn");

			/* Set min throughput to:
			 *  2592 x 1944 x 2bpp x 30fps x 3 L3 accesses */
			omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 885735);
			/* Hold a constraint to keep MPU in C1 */
			omap_pm_set_max_mpu_wakeup_lat(dev, MPU_LATENCY_C1);

			/* Configure ISP */
			isp_configure_interface(&mt9p012_if_config);
		}

		mt9p012_set_xclk(MT9P012_XCLK_48MHZ);
		msleep(3);

		if (previous_power == V4L2_POWER_OFF) {
			/* request for the GPIO's */
			cam_reset_gpio = get_gpio_by_name("gpio_cam_reset");
			if (cam_reset_gpio >= 0) {
				printk(KERN_INFO "cam_reset_gpio %d\n",
					cam_reset_gpio);
				if (gpio_request(cam_reset_gpio,
					"camera reset") != 0) {
					printk(KERN_ERR "Failed to req cam reset\n");
					goto failed_cam_reset_gpio;
				}
				gpio_direction_output(cam_reset_gpio, 0);
			}
			if (cam_standby_gpio >= 0) {
				if (gpio_request(cam_standby_gpio,
						"camera standby") != 0) {
					printk(KERN_ERR "Failed to req cam standby\n");
					goto failed_cam_standby_gpio;
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
failed_cam_standby_gpio:
		if (cam_standby_gpio >= 0) {
			gpio_free(cam_standby_gpio);
			cam_standby_gpio = -1;
		}
failed_cam_reset_gpio:
		if (cam_reset_gpio >= 0) {
			gpio_free(cam_reset_gpio);
			cam_reset_gpio = -1;
		}
        case V4L2_POWER_STANDBY:
                /* Stand By Sequence */
                mt9p012_set_xclk(0);
                break;
        }
        /* Save powerstate to know what was before calling POWER_ON. */
        previous_power = power;

	return 0;
}


static u8 mapphone_get_config_flags(void)
{
	return cam_flags;
}

struct mt9p012_platform_data mapphone_mt9p012_platform_data = {
	.power_set = mt9p012_sensor_power_set,
	.priv_data_set = mt9p012_sensor_set_prv_data,
	.lock_cpufreq = mapphone_lock_cpufreq,
	.get_config_flags = mapphone_get_config_flags,
	.csi2_lane_count = isp_csi2_complexio_lanes_count,
	.csi2_cfg_vp_out_ctrl = isp_csi2_ctrl_config_vp_out_ctrl,
	.csi2_ctrl_update = isp_csi2_ctrl_update,
	.csi2_cfg_virtual_id = isp_csi2_ctx_config_virtual_id,
	.csi2_ctx_update = isp_csi2_ctx_update,
	.csi2_calc_phy_cfg0  = isp_csi2_calc_phy_cfg0,
};

#endif /* #ifdef CONFIG_VIDEO_MT9P012 || CONFIG_VIDEO_MT9P012_MODULE */

#ifdef CONFIG_VIDEO_CAM_ISE

struct camise_capture_size {
	unsigned long width;
	unsigned long height;
};

const static struct camise_capture_size defy_camise_sizes_1[] = {
	{  176, 144 }, /* QCIF for Video Recording */
	{  320, 240 }, /* QVGA for Preview and Video Recording */
	{  352, 288 }, /* CIF for Video Recording */
	{  480, 360 }, /* Bigger Viewfinder */
	{  640, 480 }, /* 4X BINNING */
	{  512, 1024},
	{  640, 2048}, /* JPEG Catpure Resolution */
	{  848, 480 }, /* Support for WVGA preview */
};

const static struct camise_capture_size kobe_camise_sizes_1[] = {
	{  176, 144 }, /* QCIF for Video Recording */
	{  320, 240 }, /* QVGA for Preview and Video Recording */
	{  352, 288 }, /* CIF for Video Recording */
	{  480, 360 }, /* Bigger Viewfinder */
	{  512, 1024}, /* Jpeg Capture Resolution */
	{  640, 480 },	 /* 4X BINNING */
	{  768, 1024},	 /* JPEG Catpure Resolution */
};

static struct omap34xxcam_sensor_config camise_cam_hwc = {
	.sensor_isp = 1,
	.xclk = OMAP34XXCAM_XCLK_A,
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
	.dcsub = 0,	 /* Disabling DCSubtract function */
	/*.raw_fmt_in = ISPCCDC_INPUT_FMT_GR_BG,*/
	.wait_bayer_frame = 1,
	.wait_yuv_frame = 1,
	/*.cam_mclk = 216000000,
	.cam_mclk_src_div = OMAP_MCAM_SRC_DIV_36xx, */
	.cam_mclk = 216000000,
	.cam_mclk_src_div = OMAP_MCAM_SRC_DIV,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_RG_GB,
	.u.par.par_bridge = 0x3,
	.u.par.par_clk_pol = 0x0,
};

static int camise_get_capture_size(int index, unsigned long *w,	unsigned long *h)
{
	struct device_node *node;
	const void *prop;
	int max_size = 0;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		return max_size;
	}

	prop = of_get_property(node, DT_PROP_CHOSEN_USB_PROD_NAME, NULL);
	if (prop) {
		if (!strcmp(prop, "MB526")) {
			max_size =  ARRAY_SIZE(defy_camise_sizes_1);
			if (index < max_size) {
				*h = defy_camise_sizes_1[index].height;
				*w = defy_camise_sizes_1[index].width;
			}
		} else if (!strcmp(prop, "MB520")) {
			max_size =  ARRAY_SIZE(kobe_camise_sizes_1);
			if (index < max_size) {
				*h = kobe_camise_sizes_1[index].height;
				*w = kobe_camise_sizes_1[index].width;
			}
		}
	} else {
		pr_err("Read property %s error!\n",
		       DT_PROP_CHOSEN_USB_PROD_NAME);
	}

	of_node_put(node);

	return max_size;
}
static int camise_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;
	hwc->u.sensor.xclk = camise_cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = camise_cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = camise_cam_hwc.capture_mem;
	hwc->dev_index = 1;
	hwc->dev_minor = CAM_DEVICE_SOC;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static void camise_if_configure(void)
{
	isp_configure_interface(&camise_if_config);
}

static int camise_sensor_power_set(struct device *dev, enum v4l2_power power)
{
	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	int error = 0;
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
			isp_set_xclk(0, OMAP34XXCAM_XCLK_A);
			msleep(1);
		}
		break;

	case V4L2_POWER_ON:
		printk(KERN_DEBUG "%s: power on\n", __func__);

		if (previous_power == V4L2_POWER_OFF) {

			/* request for the GPIO's */
			cam_reset_gpio = get_gpio_by_name("gpio_cam_reset");
			if (cam_reset_gpio >= 0) {
				printk(KERN_INFO "cam_reset_gpio %d\n",
					cam_reset_gpio);
				if (gpio_request(cam_reset_gpio,
					"camera reset") != 0) {
					printk(KERN_ERR "Failed to req cam reset\n");
					goto failed_cam_reset_gpio;
				}
				gpio_direction_output(cam_reset_gpio, 0);
			}

			cam_standby_gpio = get_gpio_by_name("gpio_cam_pwdn");
			if (cam_standby_gpio >= 0) {
				printk(KERN_INFO "cam_standby_gpio %d\n",
					cam_standby_gpio);
				if (gpio_request(cam_standby_gpio,
					"camera standby") != 0) {
					printk(KERN_ERR "Failed to req cam standby\n");
					goto failed_cam_standby_gpio;
				}
				gpio_direction_output(cam_standby_gpio, 0);
				gpio_set_value(cam_standby_gpio, 1);
			}

			/* turn on ISP clock */
			isp_set_xclk(CAMISE_XCLK_24MHZ, OMAP34XXCAM_XCLK_A);
			/* Power Up Sequence */
			camise_if_configure();
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
	return 0;;
failed_cam_standby_gpio:
	if (cam_standby_gpio >= 0) {
		gpio_free(cam_standby_gpio);
		cam_standby_gpio = -1;
	}
failed_cam_reset_gpio:
	if (cam_reset_gpio >= 0) {
		gpio_free(cam_reset_gpio);
		cam_reset_gpio = -1;
	}
	return error;
}


struct camise_platform_data mapphone_camise_platform_data = {
    .power_set      = camise_sensor_power_set,
    .priv_data_set  = camise_sensor_set_prv_data,
    .lock_cpufreq   = mapphone_lock_cpufreq,
    .if_config      = camise_if_configure,
    .get_size       = camise_get_capture_size,
};

#endif /*CONFIG_VIDEO_CAM_ISE*/

