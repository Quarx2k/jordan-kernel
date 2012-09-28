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
#if defined(CONFIG_VIDEO_OV8810) || defined(CONFIG_VIDEO_OV8810_MODULE)
#include <media/ov8810.h>
#if defined(CONFIG_LEDS_FLASH_RESET)
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#endif
#define OV8810_CSI2_CLOCK_POLARITY	0	/* +/- pin order */
#define OV8810_CSI2_DATA0_POLARITY	0	/* +/- pin order */
#define OV8810_CSI2_DATA1_POLARITY	0	/* +/- pin order */
#define OV8810_CSI2_CLOCK_LANE		1	 /* Clock lane position: 1 */
#define OV8810_CSI2_DATA0_LANE		2	 /* Data0 lane position: 2 */
#define OV8810_CSI2_DATA1_LANE		3	 /* Data1 lane position: 3 */
#define OV8810_CSI2_PHY_THS_TERM	1  /* GVH */
#define OV8810_CSI2_PHY_THS_SETTLE	21  /* GVH */
#define OV8810_CSI2_PHY_TCLK_TERM	0
#define OV8810_CSI2_PHY_TCLK_MISS	1
#define OV8810_CSI2_PHY_TCLK_SETTLE	14
#define OV8810_XCLK_27MHZ			27000000
#endif

#if defined(CONFIG_VIDEO_OV5650) || defined(CONFIG_VIDEO_OV5650_MODULE)
#include <media/ov5650.h>
#define OV5650_CSI2_CLOCK_POLARITY	0	/* +/- pin order */
#define OV5650_CSI2_DATA0_POLARITY	0	/* +/- pin order */
#define OV5650_CSI2_DATA1_POLARITY	0	/* +/- pin order */
#define OV5650_CSI2_CLOCK_LANE		1	 /* Clock lane position: 1 */
#define OV5650_CSI2_DATA0_LANE		2	 /* Data0 lane position: 2 */
#define OV5650_CSI2_DATA1_LANE		3	 /* Data1 lane position: 3 */
#define OV5650_CSI2_PHY_THS_TERM	1
#define OV5650_CSI2_PHY_THS_SETTLE	21
#define OV5650_CSI2_PHY_TCLK_TERM	0
#define OV5650_CSI2_PHY_TCLK_MISS	1
#define OV5650_CSI2_PHY_TCLK_SETTLE	14
#define OV5650_XCLK_27MHZ		27000000
#endif

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
#include <../drivers/media/video/hplens.h>
#endif
#endif

#define CAM_IOMUX_SAFE_MODE (OMAP343X_PADCONF_PULL_UP | \
				OMAP343X_PADCONF_PUD_ENABLED | \
				OMAP343X_PADCONF_MUXMODE7)
#define CAM_IOMUX_SAFE_MODE_INPUT (OMAP343X_PADCONF_INPUT_ENABLED | \
				OMAP343X_PADCONF_PULL_UP | \
				OMAP343X_PADCONF_PUD_ENABLED | \
				OMAP343X_PADCONF_MUXMODE7)
#define CAM_IOMUX_FUNC_MODE (OMAP343X_PADCONF_INPUT_ENABLED | \
				OMAP343X_PADCONF_MUXMODE0)
#define CAM_IOMUX_SAFE_MODE_DOWN (OMAP343X_PADCONF_PULL_DOWN | \
				OMAP343X_PADCONF_PUD_ENABLED | \
				OMAP343X_PADCONF_MUXMODE7)

#define CAM_IOMUX_FUNC_MODE4 (OMAP343X_PADCONF_PULL_UP | \
				OMAP343X_PADCONF_PUD_ENABLED | \
				OMAP343X_PADCONF_MUXMODE4)

#define CAM_MAX_REGS 5
#define CAM_MAX_REG_NAME_LEN 8
#define SENSOR_POWER_OFF     0
#define SENSOR_POWER_STANDBY 1

static void mapphone_camera_lines_safe_mode(void);
static void mapphone_camera_lines_func_mode(void);
static void mapphone_camera_mipi_lines_safe_mode(void);
static void mapphone_camera_mipi_lines_func_mode(void);
static int  mapphone_camera_reg_power(bool);
static void mapphone_init_reg_list(void);
static void mapphone_init_flash_list(void);
/* devtree regulator support */
static char regulator_list[CAM_MAX_REGS][CAM_MAX_REG_NAME_LEN];
/* devtree flash */
static u8 bd7885_available;
static enum v4l2_power previous_power = V4L2_POWER_OFF;
static int avdd_en_gpio     = -1;
static int cam_reset_gpio   = -1;
static int cam_standby_gpio = -1;
static int sensor_power_config = SENSOR_POWER_STANDBY;
static u8 cam_flags;

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
	int error = 0;

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

		/* Turn off power */
		error = mapphone_camera_reg_power(false);
		if (error != 0) {
			pr_err("%s: Failed to power off regulators\n",
				__func__);
		}

		/* Release pm constraints */
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);
		omap_pm_set_max_mpu_wakeup_lat(dev, -1);
		mapphone_camera_lines_safe_mode();
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

			/* Power Up Sequence */
			mapphone_camera_lines_func_mode();
			/* Set min throughput to:
			 *  2592 x 1944 x 2bpp x 30fps x 3 L3 accesses */
			omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 885735);
			/* Hold a constraint to keep MPU in C1 */
			omap_pm_set_max_mpu_wakeup_lat(dev, MPU_LATENCY_C1);

			/* Configure ISP */
			isp_configure_interface(&mt9p012_if_config);

			/* turn on digital power */
			error = mapphone_camera_reg_power(true);
			if (error != 0) {
				pr_err("%s: Failed to power on regulators\n",
					__func__);
				goto out;
			}
			msleep(5);
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
out:
		mt9p012_set_xclk(0);
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);
		omap_pm_set_max_mpu_wakeup_lat(dev, -1);
		mapphone_camera_lines_safe_mode();
		return error;
	case V4L2_POWER_STANDBY:
		/* Stand By Sequence */
		mt9p012_set_xclk(0);
		break;
	}
	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return 0;
}

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


#if defined(CONFIG_VIDEO_OV8810)

static struct omap34xxcam_sensor_config ov8810_cam_hwc = {
	.sensor_isp = 0,
	.xclk = OMAP34XXCAM_XCLK_A,
	.capture_mem = PAGE_ALIGN(3264 * 2448 * 2) * 4,
};

static int ov8810_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.xclk = ov8810_cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = ov8810_cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = ov8810_cam_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;

	return 0;
}

static struct isp_interface_config ov8810_if_config = {
	.ccdc_par_ser = ISP_CSIA,
	.dataline_shift = 0x0,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.wenlog = ISPCCDC_CFG_WENLOG_OR,
	.wait_bayer_frame = 0,
	.wait_yuv_frame = 1,
	.dcsub = 8,
	.cam_mclk = 216000000,
	.cam_mclk_src_div = OMAP_MCAM_SRC_DIV_MIPI,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_BG_GR,
	.u.csi.crc = 0x0,
	.u.csi.mode = 0x0,
	.u.csi.edge = 0x0,
	.u.csi.signalling = 0x0,
	.u.csi.strobe_clock_inv = 0x0,
	.u.csi.vs_edge = 0x0,
	.u.csi.channel = 0x1,
	.u.csi.vpclk = 0x1,
	.u.csi.data_start = 0x0,
	.u.csi.data_size = 0x0,
	.u.csi.format = V4L2_PIX_FMT_SGRBG10,
};

static int ov8810_sensor_power_set(struct device *dev, enum v4l2_power power)
{

	struct isp_csi2_lanes_cfg lanecfg;
	struct isp_csi2_phy_cfg phyconfig;
	/*Basic turn on operation is will be first one time executed.*/
	int error = 0;
	static int cam_first_poweron = 1;

	switch (power) {
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "%s: power off\n", __func__);
		/* Power Down Sequence */
		isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);
		/* Release pm constraints */
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);
		omap_pm_set_max_mpu_wakeup_lat(dev, -1);

		gpio_set_value(GPIO_OV8810_STANDBY, 1);

		/* Wait 1ms per OVT recommendation */
		msleep(1);

		isp_set_xclk(0, OMAP34XXCAM_XCLK_A);
#if defined(CONFIG_LEDS_FLASH_RESET)
		if (bd7885_available)
			bd7885_device_disable();
#endif
		mapphone_camera_mipi_lines_safe_mode();
	break;
	case V4L2_POWER_ON:
		printk(KERN_DEBUG "%s: power on\n", __func__);

		if (previous_power == V4L2_POWER_OFF) {
			mapphone_camera_mipi_lines_func_mode();

			/* Set min throughput to:
			*  2592 x 1944 x 2bpp x 30fps x 3 L3 accesses */
			omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 885735);
			/* Hold a constraint to keep MPU in C1 */
			omap_pm_set_max_mpu_wakeup_lat(dev, MPU_LATENCY_C1);

			isp_csi2_reset();

			lanecfg.clk.pol = OV8810_CSI2_CLOCK_POLARITY;
			lanecfg.clk.pos = OV8810_CSI2_CLOCK_LANE;
			lanecfg.data[0].pol = OV8810_CSI2_DATA0_POLARITY;
			lanecfg.data[0].pos = OV8810_CSI2_DATA0_LANE;
			lanecfg.data[1].pol = 0;
			lanecfg.data[1].pos = 0;
			lanecfg.data[2].pol = 0;
			lanecfg.data[2].pos = 0;
			lanecfg.data[3].pol = 0;
			lanecfg.data[3].pos = 0;
			isp_csi2_complexio_lanes_config(&lanecfg);
			isp_csi2_complexio_lanes_update(true);

			phyconfig.ths_term = OV8810_CSI2_PHY_THS_TERM;
			phyconfig.ths_settle = OV8810_CSI2_PHY_THS_SETTLE;
			phyconfig.tclk_term = OV8810_CSI2_PHY_TCLK_TERM;
			phyconfig.tclk_miss = OV8810_CSI2_PHY_TCLK_MISS;
			phyconfig.tclk_settle = OV8810_CSI2_PHY_TCLK_SETTLE;
			isp_csi2_phy_config(&phyconfig);
			isp_csi2_phy_update(true);

			isp_configure_interface(&ov8810_if_config);

			/* Only call the following if this is not the
			 * first power-on. We need to make sure that
			 * standby is de-asserted in the correct sequence.
			 */
			if (cam_first_poweron == 0) {
				isp_set_xclk(OV8810_XCLK_27MHZ,
					OMAP34XXCAM_XCLK_A);

				/* Wait 5ms per OVT recommendation */
				msleep(5);

				/* Bring camera out of standby */
				gpio_set_value(GPIO_OV8810_STANDBY, 0);

				/* Give sensor some time to get out of the
				 * reset. Datasheet says 2400 xclks. At 6 MHz,
				 * 400 usec should be enough.  SKT waits 10ms
				 * here. Why?
				 */
				msleep(20);
			}

#if defined(CONFIG_LEDS_FLASH_RESET)
			if (bd7885_available)
				bd7885_device_enable();
#endif

		}

		if (cam_first_poweron) {
			/* Request and configure gpio pins */
			if (gpio_request(GPIO_OV8810_STANDBY,
					"ov8810 camera standby") != 0) {
				pr_err("%s: Failed to request " \
					"GPIO_OV8810_STANDBY\n", __func__);
			}
			/* Assert camera standby */
			gpio_direction_output(GPIO_OV8810_STANDBY, 1);

			if (gpio_request(GPIO_OV8810_RESET,
					"ov8810 camera reset") != 0) {
				pr_err("%s: Failed to request " \
					"GPIO_OV8810_RESET\n", __func__);
			}
			/* Assert reset */
			gpio_direction_output(GPIO_OV8810_RESET, 0);

			/* Turn off OMAP CSI2 RX for initial power-up */
			isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);

			/* Turn on power */
			error = mapphone_camera_reg_power(true);
			if (error != 0) {
				pr_err("%s: Failed to power on regulators\n",
					__func__);
				goto out;
			}

			/* Let power supplies settle.  Some hardware have large
			 * filter caps on the VCAM rail.
			*/
			msleep(10);

			isp_set_xclk(OV8810_XCLK_27MHZ, OMAP34XXCAM_XCLK_A);

			/* Wait 5ms per OVT recommendation */
			msleep(5);

			/* Bring camera out of standby */
			gpio_set_value(GPIO_OV8810_STANDBY, 0);

			/* Wait 5ms per OVT recommendation */
			msleep(5);

			/* Release reset */
			gpio_set_value(GPIO_OV8810_RESET, 1);

			/* Wait 20ms per OVT recommendation */
			msleep(20);

			cam_first_poweron = 0;
		}

		break;
out:
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);
		omap_pm_set_max_mpu_wakeup_lat(dev, -1);
		mapphone_camera_mipi_lines_safe_mode();
		return error;
	case V4L2_POWER_STANDBY:
		/* stand by */
		break;
	}
	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return error;
}

struct ov8810_platform_data mapphone_ov8810_platform_data = {
	.power_set      = ov8810_sensor_power_set,
	.priv_data_set  = ov8810_sensor_set_prv_data,
	.lock_cpufreq   = mapphone_lock_cpufreq,
	.default_regs   = NULL,
};

#endif  /* #ifdef CONFIG_VIDEO_OV8810*/


#if defined(CONFIG_VIDEO_OV5650)

static struct omap34xxcam_sensor_config ov5650_cam_hwc = {
	.sensor_isp = 0,
	.xclk = OMAP34XXCAM_XCLK_A,
	.capture_mem = PAGE_ALIGN(2592 * 1944 * 2) * 9,
};

static int ov5650_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.xclk = ov5650_cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = ov5650_cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = ov5650_cam_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;

	return 0;
}

static struct isp_interface_config ov5650_if_config = {
	.ccdc_par_ser = ISP_CSIA,
	.dataline_shift = 0x0,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.wenlog = ISPCCDC_CFG_WENLOG_OR,
	.wait_bayer_frame = 1,
	.wait_yuv_frame = 1,
	.dcsub = 16,
	.cam_mclk = 216000000,
	.cam_mclk_src_div = OMAP_MCAM_SRC_DIV_MIPI,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_BG_GR,
	.u.csi.crc = 0x0,
	.u.csi.mode = 0x0,
	.u.csi.edge = 0x0,
	.u.csi.signalling = 0x0,
	.u.csi.strobe_clock_inv = 0x0,
	.u.csi.vs_edge = 0x0,
	.u.csi.channel = 0x1,
	.u.csi.vpclk = 0x1,
	.u.csi.data_start = 0x0,
	.u.csi.data_size = 0x0,
	.u.csi.format = V4L2_PIX_FMT_SGRBG10,
};

static int ov5650_configure_gpio(void)
{
	int error = 0;

	/* request for the GPIO's */
	cam_reset_gpio = get_gpio_by_name("gpio_cam_reset");
	if (cam_reset_gpio >= 0) {
		printk(KERN_INFO "cam_reset_gpio %d\n",
			cam_reset_gpio);
			if (gpio_request(cam_reset_gpio,
			       "camera reset") != 0) {
				printk(KERN_ERR "Failed to req cam reset\n");
				error = -EINVAL;
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
				error = -EINVAL;
				goto failed_cam_standby_gpio;
		       }
		       gpio_direction_output(cam_standby_gpio, 0);
		       gpio_set_value(cam_standby_gpio, 1);
	}

	return error;
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
/*
 * ov5650_sensor_power_set_reset - Place the sensor in power off (reset) mode
 */
static int ov5650_sensor_power_set_reset(enum v4l2_power power)
{
	static bool cam_first_poweron = true;
	int error = 0;

	switch (power) {
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "%s: power off\n", __func__);
		/* turn off VADD regulator */
		if (avdd_en_gpio >= 0)
			gpio_set_value(avdd_en_gpio, 0);

		if (cam_standby_gpio >= 0)
			gpio_set_value(cam_standby_gpio, 1);

		if (cam_reset_gpio >= 0)
			gpio_set_value(cam_reset_gpio, 0);

		error = mapphone_camera_reg_power(false);
		if (error != 0) {
			pr_err("%s: Failed to power off regulators\n",
				__func__);
		}

		/* Wait 1ms per OVT recommendation */
		msleep(1);

		isp_set_xclk(0, OMAP34XXCAM_XCLK_A);
	break;

	case V4L2_POWER_ON:
		printk(KERN_DEBUG "%s: power on\n", __func__);
		if (previous_power == V4L2_POWER_OFF) {

			if (cam_first_poweron) {
				error = ov5650_configure_gpio();
				if (error != 0)
					return error;
				/*
					* Turn off OMAP CSI2 RX
					* for initial power-up
					*/
				isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);

				cam_first_poweron = false;
			}

			/* Turn on power */
			error = mapphone_camera_reg_power(true);
			if (error != 0) {
				pr_err("%s: Failed to power on regulators\n",
					__func__);
			}

			if (avdd_en_gpio >= 0)
				gpio_set_value(avdd_en_gpio, 1);

			/* Let power supplies settle.  Some hardware have large
			* filter caps on the VCAM rail.
			*/
			msleep(10);

			isp_set_xclk(OV5650_XCLK_27MHZ, OMAP34XXCAM_XCLK_A);

			/* Wait 5ms per OVT recommendation */
			msleep(5);

			/* Bring camera out of standby */
			if (cam_standby_gpio >= 0)
				gpio_set_value(cam_standby_gpio, 0);

			/* Wait 5ms per OVT recommendation */
			msleep(5);

			/* Release reset */
			if (cam_reset_gpio >= 0)
				gpio_set_value(cam_reset_gpio, 1);

			/* Wait 20ms per OVT recommendation */
			msleep(20);

		}
	break;

	case V4L2_POWER_STANDBY:
	/* stand by */
	break;
	}

	return error;
}

/*
*ov5650_sensor_power_set_standby - Place the sensor in standby mode
*/
int ov5650_sensor_power_set_standby(enum v4l2_power power)
{
	/*Basic turn on operation is will be first one time executed.*/
	static bool cam_first_poweron = true;
	int error = 0;

	switch (power) {
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "%s: power off\n", __func__);
		/* place sensor in standby mode */
		if (cam_standby_gpio >= 0)
			gpio_set_value(cam_standby_gpio, 1);

		/* Wait 1ms per OVT recommendation */
		msleep(1);

		isp_set_xclk(0, OMAP34XXCAM_XCLK_A);
	break;

	case V4L2_POWER_ON:
		printk(KERN_DEBUG "%s: power on\n", __func__);
		if (previous_power == V4L2_POWER_OFF) {
			/* Only call the following if this is not the
			* first power-on. We need to make sure that
			* standby is de-asserted in the correct sequence.
			*/
			if (cam_first_poweron == false) {
				if (avdd_en_gpio >= 0)
					gpio_set_value(avdd_en_gpio,
							1);


				isp_set_xclk(OV5650_XCLK_27MHZ,
						OMAP34XXCAM_XCLK_A);
				/* Wait 5ms per OVT recommendation */
				msleep(5);

				/* Bring camera out of standby */
				if (cam_standby_gpio >= 0)
					gpio_set_value(cam_standby_gpio,
							0);

				msleep(20);
			}

			else {

				error = ov5650_configure_gpio();
				if (error != 0)
					return error;
				/* Turn off OMAP CSI2 RX
					* for initial power-up
					*/
				isp_csi2_complexio_power(
					ISP_CSI2_POWER_OFF);

				/* Turn on power */
				error = mapphone_camera_reg_power(true);
				if (error != 0) {
					pr_err("%s: Failed: Power on regs\n",
					__func__);
				}

				if (avdd_en_gpio >= 0)
					gpio_set_value(avdd_en_gpio,
							1);

				/* Let power supplies settle.
				*  Some hardware have large
				*  filter caps on the VCAM rail.
				*/
				msleep(10);

				isp_set_xclk(OV5650_XCLK_27MHZ,
						OMAP34XXCAM_XCLK_A);

				/* Wait 5ms per OVT recommendation */
				msleep(5);

				/* Bring camera out of standby */
				if (cam_standby_gpio >= 0)
					gpio_set_value(cam_standby_gpio,
							0);

				/* Wait 5ms per OVT recommendation */
				msleep(5);

				/* Release reset */
				if (cam_reset_gpio >= 0)
					gpio_set_value(cam_reset_gpio,
							1);

				/* Wait 20ms per OVT recommendation */
				msleep(20);

				cam_first_poweron = false;
			}
		}
	break;

	case V4L2_POWER_STANDBY:
	/* stand by */
	break;
	}

	return error;

}
static int ov5650_sensor_power_set(struct device *dev, enum v4l2_power power)
{

	struct isp_csi2_lanes_cfg lanecfg;
	struct isp_csi2_phy_cfg phyconfig;

	int error = 0;

	switch (power) {
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);

		if (sensor_power_config == SENSOR_POWER_OFF)
			error = ov5650_sensor_power_set_reset(power);
		else
			error = ov5650_sensor_power_set_standby(power);

		mapphone_camera_mipi_lines_safe_mode();

		/* Release pm constraints */
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);
		omap_pm_set_max_mpu_wakeup_lat(dev, -1);
	break;
	case V4L2_POWER_ON:
		printk(KERN_DEBUG "%s: power on\n", __func__);

		if (previous_power == V4L2_POWER_OFF) {
			mapphone_camera_mipi_lines_func_mode();

			/* Set min throughput to:
			*  2592 x 1944 x 2bpp x 30fps x 3 L3 accesses */
			omap_pm_set_min_bus_tput(dev,
						OCP_INITIATOR_AGENT, 885735);
			/* Hold a constraint to keep MPU in C1 */
			omap_pm_set_max_mpu_wakeup_lat(dev, MPU_LATENCY_C1);

			isp_csi2_reset();

			lanecfg.clk.pol = OV5650_CSI2_CLOCK_POLARITY;
			lanecfg.clk.pos = OV5650_CSI2_CLOCK_LANE;
			lanecfg.data[0].pol = OV5650_CSI2_DATA0_POLARITY;
			lanecfg.data[0].pos = OV5650_CSI2_DATA0_LANE;
			lanecfg.data[1].pol = OV5650_CSI2_DATA1_POLARITY;
			lanecfg.data[1].pos = OV5650_CSI2_DATA1_LANE;
			lanecfg.data[2].pol = 0;
			lanecfg.data[2].pos = 0;
			lanecfg.data[3].pol = 0;
			lanecfg.data[3].pos = 0;
			isp_csi2_complexio_lanes_config(&lanecfg);
			isp_csi2_complexio_lanes_update(true);

			phyconfig.ths_term = OV5650_CSI2_PHY_THS_TERM;
			phyconfig.ths_settle = OV5650_CSI2_PHY_THS_SETTLE;
			phyconfig.tclk_term = OV5650_CSI2_PHY_TCLK_TERM;
			phyconfig.tclk_miss = OV5650_CSI2_PHY_TCLK_MISS;
			phyconfig.tclk_settle = OV5650_CSI2_PHY_TCLK_SETTLE*1.5;
			isp_csi2_phy_config(&phyconfig);
			isp_csi2_phy_update(true);

			isp_configure_interface(&ov5650_if_config);

		}

		if (sensor_power_config == SENSOR_POWER_OFF)
			error = ov5650_sensor_power_set_reset(power);
		else
			error = ov5650_sensor_power_set_standby(power);

		if (error != 0) {
			omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);
			omap_pm_set_max_mpu_wakeup_lat(dev, -1);
			mapphone_camera_mipi_lines_safe_mode();
		}
		break;


	case V4L2_POWER_STANDBY:
		/* stand by */
		break;
	}
	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return error;
}

struct ov5650_platform_data mapphone_ov5650_platform_data = {
	.power_set      = ov5650_sensor_power_set,
	.priv_data_set  = ov5650_sensor_set_prv_data,
	.lock_cpufreq   = mapphone_lock_cpufreq,
	.default_regs   = NULL,
};

#endif  /* #ifdef CONFIG_VIDEO_OV5650*/


int mapphone_camera_reg_power(bool enable)
{
	static struct regulator *regulator[CAM_MAX_REGS];
	static bool reg_resource_acquired;
	int i, error;

	error = 0;

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

/* We can't change the IOMUX config after bootup
 * with the current pad configuration architecture,
 * the next two functions are hack to configure the
 * camera pads at runtime to save power in standby.
 * For phones don't have MIPI camera support, like
 * Ruth, Tablet P2,P3 */

void mapphone_camera_lines_safe_mode(void)
{
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x011a);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x011c);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x011e);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x0120);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0122);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0124);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0126);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0128);
	if (cam_standby_gpio >= 0)
		omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_DOWN, 0x20D0);
}

void mapphone_camera_lines_func_mode(void)
{
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x011a);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x011c);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x011e);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0120);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0122);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0124);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0126);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0128);
	if (cam_standby_gpio >= 0)
		omap_ctrl_writew(CAM_IOMUX_FUNC_MODE4, 0x20D0);
}

/* the next two functions are for Phones have MIPI
 * camera support, like Tablet P2A */

void mapphone_camera_mipi_lines_safe_mode(void)
{
	/* CONTROL_PADCONF_GPMC_WAIT2 */
	omap_writew(CAM_IOMUX_SAFE_MODE, 0x480020D0);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0122);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0124);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0126);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0128);
}

void mapphone_camera_mipi_lines_func_mode(void)
{
	omap_writew(0x061C, 0x480020D0);	/* CONTROL_PADCONF_GPMC_WAIT2 */
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0122);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0124);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0126);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0128);
}

void mapphone_init_reg_list()
{
#ifdef CONFIG_ARM_OF
	struct device_node *feat_node;
	const void *feat_prop;
	char *prop_name;
	char reg_name[CAM_MAX_REG_NAME_LEN];
	int reg_entry;
	int feature_name_len, i, j;

	j = 0;
	reg_entry = 0;

	/* clear the regulator list */
	memset(regulator_list, 0x0, sizeof(regulator_list));

	/* get regulator info for this device */
	feat_node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (NULL == feat_node)
		return;

	feat_prop = of_get_property(feat_node,
				"feature_cam_regulators", NULL);
	if (NULL != feat_prop) {
		prop_name = (char *)feat_prop;
		printk(KERN_INFO \
			"Regulators for device: %s\n", prop_name);
		feature_name_len = strlen(prop_name);

		memset(reg_name, 0x0, CAM_MAX_REG_NAME_LEN);

		for (i = 0; i < feature_name_len; i++) {

			if (prop_name[i] != '\0' && prop_name[i] != ',')
				reg_name[j++] = prop_name[i];

			if (prop_name[i] == ',' ||\
				 (i == feature_name_len-1)) {
				printk(KERN_INFO \
					"Adding %s to camera \
						regulator list\n",\
					reg_name);
				if (reg_entry < CAM_MAX_REGS) {
					strncpy(\
						regulator_list[reg_entry++],\
						reg_name,\
						CAM_MAX_REG_NAME_LEN);
					memset(reg_name, 0x0, \
						CAM_MAX_REG_NAME_LEN);
					j = 0;
				} else {
					break;
				}
			}

		}
	}
#endif
    return;
}

static void mapphone_init_flash_list(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *node;
	int len = 0;
	const uint32_t *val;

	node = of_find_node_by_path(DT_PATH_BD7885);
	if (node != NULL) {
		val =
			of_get_property(node, "device_available", &len);
		if (val && len)
			bd7885_available =  *(u8 *)val;
	}
#endif
}

void __init mapphone_camera_init(void)
{
	struct device_node *feat_node;
	const void *feat_prop;
	unsigned int is_mipi_cam = 0;
	unsigned int is_smart_cam = 0;
	int flash_ready = 0;

#ifdef CONFIG_ARM_OF
	/* Check sensor Type */
	feat_node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (NULL != feat_node) {
		feat_prop = of_get_property(feat_node,
					"feature_mipi_cam", NULL);
		if (NULL != feat_prop) {
			is_mipi_cam = *(u8 *)feat_prop;
			printk(KERN_INFO "feature_mipi_cam %d\n", is_mipi_cam) ;
		}

		feat_prop = of_get_property(feat_node,
					"feature_smart_cam", NULL);
		if (NULL != feat_prop) {
			is_smart_cam = *(u8 *)feat_prop;
			printk(KERN_INFO "feature_smart_cam %d\n", is_smart_cam) ;
		}

		cam_flags = 0;
		feat_prop = of_get_property(feat_node,
					"feature_cam_flags", NULL);
		if (NULL != feat_prop) {
			cam_flags = *(u8 *)feat_prop;
			printk(KERN_INFO "feature_cam_flags %d\n", cam_flags) ;
		}
	}

	avdd_en_gpio = get_gpio_by_name("cam_avdd_en");
	if (avdd_en_gpio >= 0) {
		printk(KERN_INFO "cam_avdd_en %d\n", avdd_en_gpio);
		if (gpio_request(avdd_en_gpio, "camera vadd en") != 0) {
			printk(KERN_ERR "Failed vadd en\n");
			goto failed_avdd_en_gpio;
		}
		gpio_direction_output(avdd_en_gpio, 0);

		sensor_power_config = SENSOR_POWER_OFF;
	}
#endif /*CONFIG_ARM_OF */

	mapphone_init_reg_list();
	mapphone_init_flash_list();

	if (is_mipi_cam) {
		printk(KERN_INFO "mapphone_camera_init: MIPI camera\n");
		omap_cfg_reg(AD17_34XX_CSI2_DX0);
		omap_cfg_reg(AE18_34XX_CSI2_DY0);
		omap_cfg_reg(AD16_34XX_CSI2_DX1);
		omap_cfg_reg(AE17_34XX_CSI2_DY1);
		omap_cfg_reg(C25_34XX_CAM_XCLKA);
		omap_cfg_reg(C23_34XX_CAM_FLD);
		omap_cfg_reg(AG17_34XX_CAM_D0_ST);
		omap_cfg_reg(AH17_34XX_CAM_D1_ST);
		omap_cfg_reg(H2_34XX_GPMC_A3);
		/*Initialize F_RDY_N pin for Xenon flash control only.*/
		flash_ready = get_gpio_by_name("flash_ready");
		if (flash_ready >= 0) {
			if (gpio_request(flash_ready,
					"xenon flash ready pin") != 0)
				pr_err("%s: Failed to request flash ready\n",
					__func__);
			else
				gpio_direction_input(flash_ready);
		}
		mapphone_camera_mipi_lines_safe_mode();
	} else if (is_smart_cam) {
		printk(KERN_INFO "mapphone_camera_init: smart camera\n");
		omap_cfg_reg(A24_34XX_CAM_HS);
		omap_cfg_reg(A23_34XX_CAM_VS);
		omap_cfg_reg(C27_34XX_CAM_PCLK);
		omap_cfg_reg(B24_34XX_CAM_D2);
		omap_cfg_reg(C24_34XX_CAM_D3);
		omap_cfg_reg(D24_34XX_CAM_D4);
		omap_cfg_reg(A25_34XX_CAM_D5);
		omap_cfg_reg(K28_34XX_CAM_D6);
		omap_cfg_reg(L28_34XX_CAM_D7);
		omap_cfg_reg(K27_34XX_CAM_D8);
		omap_cfg_reg(L27_34XX_CAM_D9);
		omap_cfg_reg(C25_34XX_CAM_XCLKA);
		omap_cfg_reg(K8_34XX_GPMC_WAIT2);
		omap_cfg_reg(C23_34XX_CAM_FLD);

	} else {
		printk(KERN_INFO "mapphone_camera_init: conventional camera\n");

		mapphone_camera_lines_safe_mode();
	}

	return;

failed_avdd_en_gpio:
	if (avdd_en_gpio >= 0)
		gpio_free(avdd_en_gpio);

}
