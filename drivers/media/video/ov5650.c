/*
 * drivers/media/video/ov5650.c
 *
 * ov5650 sensor driver
 *
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Leverage ov8810.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>
#include <plat/resource.h>
#include <media/ov5650.h>
#include "omap34xxcam.h"
#include "isp/isp.h"
#include "isp/ispcsi2.h"
#include "isp/ispccdc.h"
#include "ov5650_reg.h"

#define OV5650_DRIVER_NAME  "ov5650"
#define MOD_NAME "OV5650: "

#define I2C_M_WR 0
#define VDD1_LOCK_VAL 0x5

#define USE_SYNCHRONOUS_LED_FLASH_CONTROL 0  /* 0=asynch or manual LED flash
						ctrl, 1=sync LED flash ctrl */

/* OV5650 clock related parameters */
struct ov5650_clk_freqs {
	u32 xclk;
	u32 vcoclk;
	u32 pclk;
	u32 mipiclk;
};

struct ov5650_sensor_id {
	u16 revision;
	u16 model;
	u16 mfr;
};

/**
 * struct ov5650_sensor_params
 */
struct ov5650_sensor_params {
	u32 line_time;  /* usec, q8 */
	u16 gain_frame_delay;
	u16 exp_time_frame_delay;
	u16 total_vertical_size;
	u16 line_length_clocks;
	u16 h_output_size;
	u16 v_output_size;
	u16 binning_sensitivity;
};

struct ov5650_flash_params {
	u16 flash_time;
	u8 flash_type;
	u8 shutter_type;
};

struct ov5650_exp_params {
	u32 exp_time;
	u32 line_time;
	u16 coarse_int_tm;
	u16 analog_gain;
	u16 digital_gain;
	u16 min_exp_time;
	u32 fps_max_exp_time;
	u32 abs_max_exp_time;
	u16 min_linear_gain;
	u16 max_linear_gain;
};

struct ov5650_min_readout_params {
	u8   enable_min_size;
	u16 requested_width;
	u16 requested_height;
};

enum ioctl_op {
	IOCTL_RD = 0,
	IOCTL_WR
};

enum sensor_op {
	SENSOR_REG_REQ = 0,
	CALIBRATION_ADJ,
	SENSOR_ID_REQ,
	COLOR_BAR,
	FLASH_NEXT_FRAME,
	ORIENTATION,
	LENS_CORRECTION,
	SENSOR_PARAMS_REQ,
	START_MECH_SHUTTER_CAPTURE,
	SET_SHUTTER_PARAMS,
	SENSOR_OTP_REQ,
	DEFECT_PIXEL_CORRECTION,
	FLICKER_DETECT_REQ,
	CROPPED_READOUT_REQ,
	USE_MIN_SIZE_READOUT
};

struct camera_params_control {
	enum ioctl_op xaction;
	enum sensor_op op;
	u32 data_in;
	u32 data_out;
	u16 data_in_size;
	u16 data_out_size;
};

/* Private IOCTLs  now in the ov5650_regs.h file */
#define V4L2_CID_PRIVATE_S_PARAMS	(V4L2_CID_PRIVATE_BASE + 22)
#define V4L2_CID_PRIVATE_G_PARAMS	(V4L2_CID_PRIVATE_BASE + 23)

/**
 * struct ov5650_sensor - main structure for storage of sensor information
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @i2c_client: iic client device structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @isize: base image size
 * @ver: ov5650 chip version
 * @width: configured width
 * @height: configuredheight
 * @vsize: vertical size for the image
 * @hsize: horizontal size for the image
 * @crop_rect: crop rectangle specifying the left,top and width and height
 * @state:
 * @frame: image frame parameters
*/
struct ov5650_sensor {
	struct device *dev;
	const struct ov5650_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	int isize;
	int fps;
	unsigned long width;
	unsigned long height;
	unsigned long vsize;
	unsigned long hsize;
	struct v4l2_rect crop_rect;
	int state;
	bool resuming;
	bool streaming;
	struct ov5650_clk_freqs freq;
	struct ov5650_sensor_id sensor_id;
	struct ov5650_flash_params flash;
	struct ov5650_exp_params exposure;
	enum ov5650_orientation orientation;
	struct ov5650_min_readout_params min_readout;
};

static struct ov5650_sensor ov5650 = {
	.timeperframe = {
		.numerator = 1,
		.denominator = 15,
	},
	.sensor_id = {
		.revision = 0,
		.model = 0,
		.mfr = 0
	},
	.state = SENSOR_NOT_DETECTED,
	.freq = {
		.xclk = OV5650_XCLK_MIN,
		.mipiclk = 378000000,
	},
	.orientation = OV5650_HORZ_FLIP_ONLY,
};

static struct i2c_driver ov5650sensor_i2c_driver;
static enum v4l2_power current_power_state = V4L2_POWER_OFF;
static bool ov5650_flicker_manual_mode = true;

#define BRIGHT_CENTER_COMP_GAIN (4*256)	/* linear gain threshold where bright
					   center compensation is used. */
static bool bright_center_compensation;

/* List of image formats supported by OV5650 sensor */
const static struct v4l2_fmtdesc ov5650_formats[] = {
	{
		.description	= "RAW10",
		.pixelformat	= V4L2_PIX_FMT_SGRBG10,
	}
};

#define NUM_CAPTURE_FORMATS (sizeof(ov5650_formats) / sizeof(ov5650_formats[0]))

/* register initialization tables for ov5650 */
#define OV5650_REG_TERM 0xFFFF	/* terminating list entry for reg */
#define OV5650_VAL_TERM 0xFF	/* terminating list entry for val */

#define OV5650_OTP_MEM_SIZE 256
static u8 ov5650_otp_data[OV5650_OTP_MEM_SIZE];

static struct ov5650_sensor_settings sensor_settings[] = {

	/* SIZE_80K */
		/* MIPI Sensor Raw QVGA 114fps with 27MHz EXCLK
		   2-lane, RAW 10, PCLK = 97.2MHz, MIPI_CLK = 114.8MHz
		   Blanking = 2.0msec */
	{
		.clk = {
			.pre_div_q1 = 4,   /* 2x actual */
			.div_p = 18,
			.div_s = 2,
			.div_l = 1,
			.div_m = 4,
			.seld2p5_q1 = 5,    /* 2x actual */
			.seld5 = 4,
			.out_blk_div = 2,
		},
		.frame = {
			.total_vert_size_min = 312,
			.total_horz_size_min = 2564,
			.man_hv_ref_start = true,
			.h_ref_start = 800,
			.h_ref_width = 1296,
			.v_ref_start = 8,
			.v_ref_width = 244,
			.h_output_size = 324,
			.v_output_size = 244,
			.h_subsample = 8,
			.v_subsample = 8,
			.h_binning = 1,
			.v_binning = 1,
			.binning_sensitivity = (u16)(4.0 * 256),
			.min_time_per_frame = {
				.numerator = 1,
				.denominator = 120,
			},
		},
		.mipi = {
			.hs_settle = 14,
		},
	},

	/* SIZE_315K */
		/* MIPI Sensor Raw VGA 30.06fps with 27MHz EXCLK
		   2-lane, RAW 10, PCLK = 75.6MHz, MIPI_CLK = 189.0MHz
		   V-Blanking = 20.0msec */
	{
		.clk = {
			.pre_div_q1 = 4,   /* 2x actual */
			.div_p = 14,
			.div_s = 2,
			.div_l = 1,
			.div_m = 2,
			.seld2p5_q1 = 5,    /* 2x actual */
			.seld5 = 4,
			.out_blk_div = 2,
		},
		.frame = {
			.total_vert_size_min = 1160,
			.total_horz_size_min = 2168,
			.man_hv_ref_start = true,
			.h_ref_start = 800,
			.h_ref_width = 1296,
			.v_ref_start = 8,
			.v_ref_width = 486,
			.h_output_size = 648,
			.v_output_size = 486,
			.h_subsample = 4,
			.v_subsample = 4,
			.h_binning = 1,
			.v_binning = 1,
			.binning_sensitivity = (u16)(4.0 * 256),
			.min_time_per_frame = {
				.numerator = 1,
				.denominator = 30,
			},
		},
		.mipi = {
			.hs_settle = 17,
		},
	},

	/* SIZE_1_25M */
		/* MIPI Sensor Raw 1296x972 30.06fps with 27MHz EXCLK
		   2-lane, RAW 10, PCLK = 75.6MHz, MIPI_CLK = 378.0MHz
		   V-Blanking = 6.0ms */
	{
		.clk = {
			.pre_div_q1 = 4,   /* 2x actual */
			.div_p = 14,
			.div_s = 2,
			.div_l = 1,
			.div_m = 1,
			.seld2p5_q1 = 5,    /* 2x actual */
			.seld5 = 4,
			.out_blk_div = 2,
		},
		.frame = {
			.total_vert_size_min = 1184,
			.total_horz_size_min = 2124,
			.man_hv_ref_start = false,
			.h_ref_start = 808,
			.h_ref_width = 1296,
			.v_ref_start = 12,
			.v_ref_width = 972,
			.h_output_size = 1296,
			.v_output_size = 972,
			.h_subsample = 2,
			.v_subsample = 2,
			.h_binning = 1,
			.v_binning = 1,
			.binning_sensitivity = (u16)(2.0 * 256),
			.min_time_per_frame = {
				.numerator = 1,
				.denominator = 30,
			},
		},
		.mipi = {
			.hs_settle = 27,
		},
	},

	/* SIZE_2M */
		/* MIPI Sensor Raw Cropped 1824x1216 30fps with 27MHz EXCLK
		   2-lane, RAW 10, PCLK = 97.2MHz, MIPI_CLK = 486.0MHz */
	{
		.clk = {
			.pre_div_q1 = 4,   /* 2x actual */
			.div_p = 18,
			.div_s = 2,
			.div_l = 1,
			.div_m = 1,
			.seld2p5_q1 = 5,    /* 2x actual */
			.seld5 = 4,
			.out_blk_div = 2,
		},
		.frame = {
			.total_vert_size_min = 1304,
			.total_horz_size_min = 2486,
			.man_hv_ref_start = false,
			.h_ref_start = 698,
			.h_ref_width = 1848,
			.v_ref_start = 18,
			.v_ref_width = 1232,
			.h_output_size = 1824,
			.v_output_size = 1216,
			.h_subsample = 1,
			.v_subsample = 1,
			.h_binning = 0,
			.v_binning = 0,
			.binning_sensitivity = (u16)(1.0 * 256),
			.min_time_per_frame = {
				.numerator = 1,
				.denominator = 30,
			},
		},
		.mipi = {
			.hs_settle = 27,
		},
	},

	/* SIZE_5M */
		/* MIPI Sensor Raw QSXGA 2592x1944 11.81fps with 27MHz EXCLK
		   2-lane, RAW 10, PCLK = 75.6MHz, MIPI_CLK = 378.0MHz */
	{
		.clk = {
			.pre_div_q1 = 4,   /* 2x actual */
			.div_p = 14,
			.div_s = 2,
			.div_l = 1,
			.div_m = 1,
			.seld2p5_q1 = 5,    /* 2x actual */
			.seld5 = 4,
			.out_blk_div = 2,
		},
		.frame = {
			.total_vert_size_min = 1968,
			.total_horz_size_min = 3252,
			.man_hv_ref_start = false,
			.h_ref_start = 596,
			.h_ref_width = 2592,
			.v_ref_start = 12,
			.v_ref_width = 1944,
			.h_output_size = 2592,
			.v_output_size = 1944,
			.h_subsample = 1,
			.v_subsample = 1,
			.h_binning = 0,
			.v_binning = 0,
			.binning_sensitivity = (u16)(1.0 * 256),
			.min_time_per_frame = {
				.numerator = 1,
				.denominator = 12,
			},
		},
		.mipi = {
			.hs_settle = 27,
		},
	},

};

/*
 * struct vcontrol - Video controls
 * @v4l2_queryctrl: V4L2 VIDIOC_QUERYCTRL ioctl structure
 * @current_value: current value of this control
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = 0,
			.maximum = -1,
			.step = EXPOSURE_STEP,
			.default_value = DEF_EXPOSURE,
		},
		.current_value = DEF_EXPOSURE,
	},
	{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Analog Gain",
			.minimum = OV5650_MIN_LINEAR_GAIN,
			.maximum = OV5650_MAX_LINEAR_GAIN,
			.step = LINEAR_GAIN_STEP,
			.default_value = DEF_LINEAR_GAIN,
		},
		.current_value = DEF_LINEAR_GAIN,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_S_PARAMS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Set Camera Params",
			.minimum = 0,
			.maximum = -1,
			.step = 0,
			.default_value = 0,
		},
		.current_value = 0,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_G_PARAMS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Get Camera Params",
			.minimum = 0,
			.maximum = -1,
			.step = 0,
			.default_value = 0,
		},
		.current_value = 0,
	},
};

struct private_vcontrol {
	int id;
	int type;
	char name[32];
	int minimum;
	int maximum;
	int step;
	int default_value;
	int current_value;
};

static struct private_vcontrol video_control_private[] = {
	{
		.id = SENSOR_REG_REQ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Sensor Register",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = SENSOR_ID_REQ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Sensor ID",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = COLOR_BAR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Color Bar",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = FLASH_NEXT_FRAME,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Flash On Next Frame",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = ORIENTATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Orientation",
		.minimum = OV5650_NO_HORZ_FLIP_OR_VERT_FLIP,
		.maximum = OV5650_HORZ_FLIP_AND_VERT_FLIP,
		.step = 0,
		.default_value = OV5650_HORZ_FLIP_ONLY,
		.current_value = OV5650_HORZ_FLIP_ONLY,
	},
	{
		.id = LENS_CORRECTION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Lens Correction",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = SENSOR_PARAMS_REQ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Sensor Params",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = START_MECH_SHUTTER_CAPTURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Start Mech Shutter Capture",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = SET_SHUTTER_PARAMS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Shutter Params",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = SENSOR_OTP_REQ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Sensor OTP",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = DEFECT_PIXEL_CORRECTION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Defect Pixel Correction",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = FLICKER_DETECT_REQ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Flicker Detect",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = CROPPED_READOUT_REQ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Cropped Readout",
		.minimum = 0,
		.maximum = 1,
		.step = 0,
		.default_value = 0,
		.current_value = 0,
	},
	{
		.id = USE_MIN_SIZE_READOUT,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Min Size Readout",
		.minimum = 0,
		.maximum = -1,
		.step = 0,
		.default_value = 0,
		.current_value = 0,
	},
};

struct i2c_client *ov5650_i2c_client;

/*
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array.
 *
 * Returns the index of the requested ID from the control structure array
 */
static int find_vctrl(int id)
{
	int i = 0;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

static int find_vctrl_private(int id)
{
	int i;

	if (id < 0)
		return -EDOM;

	for (i = 0; i < sizeof(video_control_private); i++)
		if (video_control_private[i].id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/*
 * Read a value from a register in ov5650 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov5650_read_reg(struct i2c_client *client, u16 data_length, u16 reg,
								u32 *val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;

	/* High byte goes out first */
	data[0] = (u8) (reg >> 8);
	data[1] = (u8) (reg & 0xff);

	err = i2c_transfer(client->adapter, msg, 1);
	if (err < 0) {
		msleep(5);
		err = i2c_transfer(client->adapter, msg, 1);
	}

	if (err >= 0) {
		mdelay(3);
		msg->flags = I2C_M_RD;
		msg->len = data_length;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = 0;
		/* High byte comes first */
		if (data_length == 1)
			*val = data[0];
		else if (data_length == 2)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
				(data[1] << 16) + (data[0] << 24);
		return 0;
	}
	printk(KERN_ERR "OV5650: read from offset 0x%x error %d\n", reg, err);
	return err;
}

/* Write a value to a register in ov5650 sensor device.
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 * Returns zero if successful, or non-zero otherwise.
 */
int ov5650_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[3];
	int retries = 5;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 3;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);
	data[1] = (u8) (reg & 0xff);
	data[2] = val;

	do {
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			udelay(50);
			return 0;
		}
		msleep(5);
	} while ((--retries) > 0);

	return err;
}

/*
 * Initialize a list of ov5650 registers.
 * The list of registers is terminated by the pair of values
 * {OV5650_REG_TERM, OV5650_VAL_TERM}.
 * @client: i2c driver client structure.
 * @reglist[]: List of address of the registers to write data.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ov5650_write_regs(struct i2c_client *client,
					const struct ov5650_reg reglist[])
{
	int err = 0;
	const struct ov5650_reg *next = reglist;

	while (!((next->reg == OV5650_REG_TERM)
		&& (next->val == OV5650_VAL_TERM))) {
		err = ov5650_write_reg(client, next->reg, next->val);
		udelay(100);
		if (err)
			return err;
		next++;
	}
	return 0;
}

/**
 * ov5650_set_exposure_time - sets exposure time per input value
 * @exp_time: exposure time to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 *
 * If the requested exposure time is not within the allowed limits, the
 * exposure time is forced to the limit value. The HW
 * is configured to use the new exposure time, and the
 * video_control[] array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov5650_set_exposure_time(u32 exp_time, struct v4l2_int_device *s,
				struct vcontrol *lvc, enum image_size_ov isize)
{
	/* Inputs exp_time in usec */
	u16 coarse_int_tm;
	int err = 0;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	u32 line_time_q8 = sensor->exposure.line_time;

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {

		if (exp_time < sensor->exposure.min_exp_time) {
			DPRINTK_OV5650("OV5650: Exposure time %dus is less " \
				"than regal limit %dus\n",
				exp_time, sensor->exposure.min_exp_time);

			exp_time = sensor->exposure.min_exp_time;
		}

		/* OV5650 cannot accept exposure time longer than frame time */
		if (exp_time > sensor->exposure.fps_max_exp_time) {
			DPRINTK_OV5650("OV5650: Exposure time %dus is " \
				"greater than legal limit %dus\n",
				exp_time, sensor->exposure.fps_max_exp_time);

			exp_time = sensor->exposure.fps_max_exp_time;
		}

		/* calc num 1/16 lines with rounding */
		coarse_int_tm = ((exp_time << 12) + (line_time_q8 >> 1)) /
			line_time_q8;
		/* OV5650 does not support m.n, only m.0 or 0.n */
		if (coarse_int_tm > 0xf)
			coarse_int_tm = (coarse_int_tm + 0x8) & 0xFFFF0;

		if (coarse_int_tm != sensor->exposure.coarse_int_tm) {
			/* write number of line times to EXPO_H/M/L registers */
			err = ov5650_write_reg(client, OV5650_LONG_EXPO_H,
				(coarse_int_tm >> 16) & 0x0F);
			err = ov5650_write_reg(client, OV5650_LONG_EXPO_M,
				(coarse_int_tm >> 8) & 0xFF);
			err |= ov5650_write_reg(client, OV5650_LONG_EXPO_L,
				coarse_int_tm & 0xFF);

			DPRINTK_OV5650("set_exposure_time = " \
				"%d usec, CoarseIntTime = %d, pclk=%d, " \
				"total_horz_size=%d clks, line_tm=%d/256 us\n",
				exp_time, coarse_int_tm, sensor->freq.pclk,
				sensor_settings[isize].frame.total_horz_size,
				line_time_q8);

			/* save results */
			sensor->exposure.exp_time = exp_time;
			sensor->exposure.coarse_int_tm = coarse_int_tm;
		}
	}

	if (err)
		printk(KERN_ERR "OV5650: Error setting exposure time...%d\n",
			err);
	else {
		if (lvc)
			lvc->current_value = exp_time;
	}

	return err;
}

/**
 * ov5650_set_gain - sets sensor analog & digital gain per input value
 * @lineargain: q8 analog gain value to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 analog gain entry in ov5650_video_control array
 *
 * If the requested analog gain is within the allowed limits, the HW
 * is configured to use the new gain value, and the ov5650_video_control
 * array is updated with the new current value.
 * Up to 2x digital gain will be used in addition to analog gain to achieve
 * the desired gain if necessary.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov5650_set_gain(u16 linear_gain_Q8, struct v4l2_int_device *s,
	struct vcontrol *lvc)
{
	/* Inputs linear Q8 gain */
	bool use_digital_2x_gain = false;
	u16 anlg_gain_stage_2x = 0;
	u16 shift_bits = 0;
	u16 anlg_gain_fraction = 0;
	u16 anlg_gain_register = 0, dgtl_gain_register = 0x01;
	int err = 0;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	if (linear_gain_Q8 < sensor->exposure.min_linear_gain) {
		DPRINTK_OV5650("OV5650: Gain %d less than legal limit %d\n",
			linear_gain_Q8, sensor->exposure.min_linear_gain);

		linear_gain_Q8 = sensor->exposure.min_linear_gain;
	}

	if (linear_gain_Q8 > sensor->exposure.max_linear_gain) {
		DPRINTK_OV5650("OV5650: Gain %d greater than legal limit %d\n",
			linear_gain_Q8, sensor->exposure.max_linear_gain);

		linear_gain_Q8 = sensor->exposure.max_linear_gain;
	}

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		if (linear_gain_Q8 >= 16*256) {
			use_digital_2x_gain = true;
			anlg_gain_stage_2x = 0x70;
			shift_bits = 4;
		} else if (linear_gain_Q8 >= 8*256) {
			anlg_gain_stage_2x = 0x70;
			shift_bits = 3;
		} else if (linear_gain_Q8 >= 4*256) {
			anlg_gain_stage_2x = 0x30;
			shift_bits = 2;
		} else if (linear_gain_Q8 >= 2*256) {
			anlg_gain_stage_2x = 0x10;
			shift_bits = 1;
		}

		anlg_gain_fraction = linear_gain_Q8 >> shift_bits;
		 /* subt 1 (Q8) and take upper 4 bits */
		anlg_gain_fraction = (anlg_gain_fraction - (1*256)) >> 4;
		if (anlg_gain_fraction > 0x0f)
			anlg_gain_fraction = 0x0f;

		anlg_gain_register = anlg_gain_stage_2x | anlg_gain_fraction;
		if (use_digital_2x_gain)
			dgtl_gain_register = 0x09;

		if (sensor->exposure.analog_gain != anlg_gain_register) {
			err = ov5650_write_reg(client, OV5650_AGC_ADJ,
				anlg_gain_register);

			DPRINTK_OV5650("gain =%d/256, " \
				"angl_gain reg = 0x%x\n",
				linear_gain_Q8, anlg_gain_register);

			 /* save results */
			 sensor->exposure.analog_gain = anlg_gain_register;
		}

		if (sensor->exposure.digital_gain != dgtl_gain_register) {
			err = ov5650_write_reg(client, OV5650_ISP_CTRL46,
				dgtl_gain_register);

			DPRINTK_OV5650("Use digital 2X gain = %d\n",
				(int)(use_digital_2x_gain));

			/* save results */
			sensor->exposure.digital_gain = dgtl_gain_register;
		}

		/* use bright center compensation at high gains */
		if (linear_gain_Q8 >= BRIGHT_CENTER_COMP_GAIN) {
			if (bright_center_compensation == false) {
				err = ov5650_write_regs(client,
					ov5650_enable_bright_center_comp_tbl);
				bright_center_compensation = true;
				DPRINTK_OV5650("Bright Center Comp Enabled\n");
			}
		} else {
			if (bright_center_compensation == true) {
				err = ov5650_write_regs(client,
					ov5650_disable_bright_center_comp_tbl);
				bright_center_compensation = false;
				DPRINTK_OV5650("Bright Center Comp Disabled\n");
			}
		}
	}

	if (err) {
		printk(KERN_ERR "OV5650: Error setting analog gain: %d\n", err);
		return err;
	} else {
		if (lvc)
			lvc->current_value = linear_gain_Q8;
	}

	return err;
}

static int ov5650_init_exposure_params(struct v4l2_int_device *s)
{
	struct ov5650_sensor *sensor = s->priv;

	/* flag current exp_time & gain values as invalid */
	sensor->exposure.analog_gain = 0;
	sensor->exposure.digital_gain = 0;
	sensor->exposure.coarse_int_tm = 0;

	return 0;
}

/**
 * ov5650_set_framerate - Sets framerate by adjusting total_vert_size reg.
 * @s: pointer to standard V4L2 device structure
 * @fper: frame period numerator and denominator in seconds
 *
 * The maximum exposure time is also updated since it is affected by the
 * frame rate.
 **/
static int ov5650_set_framerate(struct v4l2_int_device *s,
			struct v4l2_fract *fper, enum image_size_ov isize)
{
	u32 total_vertical_size, line_time_q8;
	int err = 0, i = 0;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct vcontrol *lvc = NULL;
	struct ov5650_sensor_settings *ss = &sensor_settings[isize];

	/* limit desired frame period to min frame period for this readout */
	if (((fper->numerator << 8) / fper->denominator) <
		((ss->frame.min_time_per_frame.numerator << 8) /
		  ss->frame.min_time_per_frame.denominator)) {
		fper->numerator = ss->frame.min_time_per_frame.numerator;
		fper->denominator = ss->frame.min_time_per_frame.denominator;
	}

	line_time_q8 = /* usec's (q8) */
		((((u32)ss->frame.total_horz_size * 1000) << 8) /
		(sensor->freq.pclk / 1000));

	total_vertical_size = (((u32)fper->numerator * 1000000 * 256 /
			       fper->denominator)) / line_time_q8;

	/* Range check total_vertical_size */
	if (total_vertical_size > OV5650_MAX_TOTAL_VERT_SIZE)
		total_vertical_size = OV5650_MAX_TOTAL_VERT_SIZE;
	else if (total_vertical_size < ss->frame.total_vert_size_min)
		total_vertical_size = ss->frame.total_vert_size_min;

	/* Write new frame length to sensor */
	ov5650_write_reg(client, OV5650_TOTAL_VERT_SIZE_H,
		total_vertical_size >> 8);
	ov5650_write_reg(client, OV5650_TOTAL_VERT_SIZE_L,
		total_vertical_size  & 0xFF);

	/* Save results */
	ss->frame.total_vert_size = total_vertical_size;
	sensor->exposure.line_time = line_time_q8;
	/* min line time is 1/16 of a line */
	sensor->exposure.min_exp_time = line_time_q8 >> 12;
	sensor->exposure.fps_max_exp_time = (line_time_q8 *
		(ss->frame.total_vert_size - 8)) >> 8;
	sensor->exposure.abs_max_exp_time = (line_time_q8 *
		(OV5650_MAX_TOTAL_VERT_SIZE - 8)) >> 8;

	/* Update Exposure Time */
	i = find_vctrl(V4L2_CID_EXPOSURE);
	if (i >= 0) {
		lvc = &video_control[i];
		/* Update min/max for query control */
		lvc->qc.minimum = sensor->exposure.min_exp_time;
		lvc->qc.maximum = sensor->exposure.fps_max_exp_time;

		ov5650_set_exposure_time(lvc->current_value, s, lvc, isize);
	}

	DPRINTK_OV5650("Set Framerate: fper=%d/%d, " \
		"total_vert_size=%d, fps_max_expT=%dus, " \
		"abs_max_expT=%dus, line_tm=%d/256\n",
		fper->numerator, fper->denominator, total_vertical_size,
		sensor->exposure.fps_max_exp_time,
		sensor->exposure.abs_max_exp_time, line_time_q8);

	return err;
}

/**
 * ov5650_set_color_bar_mode - puts sensor in color bar test mode
 * @enable: 0 = off, 1 = on
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 * This function should be called after the resolution is setup. The sensor
 * will stay in color bar mode until the next resolution is selected.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov5650_set_color_bar_mode(u16 enable, struct v4l2_int_device *s,
						struct private_vcontrol *pvc)
{
	int err = 0;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		if (enable) {
			err = ov5650_write_reg(client, OV5650_ISP_CTRL_3D,
				OV5650_ISP_CTRL_3D_TEST_PATT_EN_MASK);
		} else {
			err = ov5650_write_reg(client, OV5650_ISP_CTRL_3D,
				0x0);
		}
	}

	if (err)
		printk(KERN_ERR "OV5650: Error setting color bar mode\n");
	else {
		if (pvc)
			pvc->current_value = enable;
	}

	return err;
}

/**
 * ov5650_set_flash_next_frame - configures flash on for the next frame
 * @flash_params: flash type and time
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int ov5650_set_flash_next_frame(
			struct ov5650_flash_params *flash_params,
			struct v4l2_int_device *s, struct private_vcontrol *pvc)
{
	int err = 0, data = 0;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	u32 strb_pulse_width;
	u32 line_time_q8 = sensor->exposure.line_time;

	DPRINTK_OV5650("set_flash_next_frame: time=%dusec, " \
		"flash_type=%d, shutter_type=%d, power=%d\n",
		flash_params->flash_time, flash_params->flash_type,
		flash_params->shutter_type,
		(current_power_state == V4L2_POWER_ON) || sensor->resuming);

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {

		/* Enable strobe output pad */
		data = 1 << OV5650_PAD_OE_STRB_EN_SHIFT;
		err = ov5650_write_reg(c, OV5650_PAD_OUTPUT_ENBL_0, data);

		DPRINTK_OV5650("set_flash_next_frame:  " \
			"OV5650_PAD_OUTPUT_ENBL_0=0x%x\n", data);

		/* Set Strobe Ctrl Reg */
		data = 0;
		if (flash_params->flash_time != 0)
			data |= 1 << OV5650_STROBE_CTRL_STRB_EN_SHIFT;

		if (flash_params->flash_type == XENON_FLASH_TYPE) {
			/* use XENON mode - set XENON pulse width */
			strb_pulse_width = (flash_params->flash_time << 8) /
				line_time_q8;

			if (strb_pulse_width < 1)
				strb_pulse_width = 1;
			else if (strb_pulse_width > 4)
				strb_pulse_width = 4;

			data |= (strb_pulse_width - 1) <<
				OV5650_STROBE_CTRL_STRB_PLS_WIDTH_SHIFT;

			err |= ov5650_write_reg(c, OV5650_STROBE_CTRL, data);

			DPRINTK_OV5650("set_flash_next_frame:  " \
				"OV5650_STROBE_CTRL=0x%x\n", data);
		} else {

#if USE_SYNCHRONOUS_LED_FLASH_CONTROL

			/* use LED3 Mode Control */
			data |= 3 << OV5650_STROBE_CTRL_STRB_MODE_SHIFT;

			err |= ov5650_write_reg(c, OV5650_STROBE_CTRL, data);

			DPRINTK_OV5650("set_flash_next_frame:  " \
				"OV5650_STROBE_CTRL=0x%x\n", data);

			if (flash_params->shutter_type == MECH_SHUTTER_TYPE &&
					flash_params->flash_time != 0) {
				err = -EINVAL;
				dev_err(&c->dev, "Mechanical Shutter" \
					" with Flash not implemented.\n");
			}

#else 			/* use Asynch or Manual Flash Control */

			/* Set Strobe output to GPIO mode */
			data = 1 << OV5650_PAD_SELECT_0_IO_STRB_SEL_SHIFT;
			err |= ov5650_write_reg(c, OV5650_PAD_SELECT_0, data);
			DPRINTK_OV5650("%s:OV5650_PAD_SELECT_0=0x%x\n",
				__func__, data);

			/* Manually set strobe output */
			data = 0;
			if (flash_params->flash_time != 0)
				data = 1 << OV5650_PAD_OUTPUT_0_STROBE_SHIFT;

			err |= ov5650_write_reg(c, OV5650_PAD_OUTPUT_0, data);
			DPRINTK_OV5650("%s:OV5650_PAD_OUTPUT_0=0x%x\n",
				__func__, data);
#endif
		}

		/* Auto reset */
		flash_params->flash_time = 0;
	}

	if (err)
		dev_err(&c->dev, "Error setting flash registers\n");
	else {
		sensor->flash.flash_time = flash_params->flash_time;
		sensor->flash.flash_type = flash_params->flash_type;
		sensor->flash.shutter_type = flash_params->shutter_type;
	}
	return err;
}

/**
 * Sets the sensor orientation.
 */
static int ov5650_set_orientation(enum ov5650_orientation val,
	struct v4l2_int_device *s, struct private_vcontrol *pvc,
	enum image_size_ov isize)
{
	bool hmirror, vflip;
	int err = 0;
	u32 data;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	struct ov5650_sensor_settings *ss = &sensor_settings[isize];

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {

		hmirror = (val == OV5650_HORZ_FLIP_ONLY ||
		    val == OV5650_HORZ_FLIP_AND_VERT_FLIP);
		vflip = (val == OV5650_VERT_FLIP_ONLY ||
		    val == OV5650_HORZ_FLIP_AND_VERT_FLIP);

		/* Update OV5650_TIMING_TC_18 register */
		err = ov5650_read_reg(client, 1,
			OV5650_TIMING_TC_18, &data);
		if (hmirror)
			data |= OV5650_TIMING_TC_18_MIRROR_MASK;
		else
			data &= ~OV5650_TIMING_TC_18_MIRROR_MASK;
		if (vflip)
			data |= OV5650_TIMING_TC_18_VFLIP_MASK;
		else
			data &= ~OV5650_TIMING_TC_18_VFLIP_MASK;
		if (ss->frame.h_subsample == 1) {
			data &= ~(OV5650_TIMING_TC_18_VSUB2_MASK |
				OV5650_TIMING_TC_18_VSUB4_MASK);
		} else if (ss->frame.h_subsample == 2) {
			data |= OV5650_TIMING_TC_18_VSUB2_MASK;
			data &= ~OV5650_TIMING_TC_18_VSUB4_MASK;
		} else if (ss->frame.h_subsample == 4) {
			data &= ~OV5650_TIMING_TC_18_VSUB2_MASK;
			data |= OV5650_TIMING_TC_18_VSUB4_MASK;
		} else {
			data |= (OV5650_TIMING_TC_18_VSUB2_MASK |
				OV5650_TIMING_TC_18_VSUB4_MASK);
		}

		err |= ov5650_write_reg(client,
			OV5650_TIMING_TC_18, data);

		DPRINTK_OV5650("%s: sensor->orientation=%d, " \
			"OV5650_TIMING_TC_18=0x%x\n",
			__func__, val, data);

		/* Update OV5650_ARRAY_CONTROL_01 register */
		err = ov5650_read_reg(client, 1,
			OV5650_ARRAY_CTRL_01, &data);
		if (hmirror)
			data &= ~0x10;
		else
			data |= 0x10;
		err |= ov5650_write_reg(client,
			OV5650_ARRAY_CTRL_01, data);

		DPRINTK_OV5650("%s: OV5650_ARRAY_CTRL_01=0x%x\n",
			__func__, data);

		/* Update OV5650_RESERVED_505A/B register */
		if (hmirror)
			data = 0x0A2E;
		else
			data = 0x0012;
		err |= ov5650_write_reg(client,
			OV5650_RESERVED_505A, data >> 8);
		err |= ov5650_write_reg(client,
			OV5650_RESERVED_505B, data & 0xFF);

		DPRINTK_OV5650("%s: OV5650_RESERVED_505A/B=0x%x\n",
			__func__, data);

		/* Update VREF Start: Force Odd for vflip & vice versa */
		data = ss->frame.v_ref_start;
		if (ss->frame.man_hv_ref_start) {
			if ((vflip && (data & 0x1) == 0) ||
				(!vflip && (data & 0x1) == 1)) {
				data--;
				err |= ov5650_write_reg(client,
					OV5650_VREF_START_MAN_H,
					(data >> 8) & 0xFF);

				err |= ov5650_write_reg(client,
					OV5650_VREF_START_MAN_L,
					data & 0xFF);

				/* VREF_START regs may get used depending on
				   VFLIP */

				err |= ov5650_write_reg(client,
					OV5650_VREF_START_H,
					(data >> 8) & 0xFF);

				err |= ov5650_write_reg(client,
					OV5650_VREF_START_L,
					data & 0xFF);

				DPRINTK_OV5650("%s: OV5650_VREF_START_MAN=" \
					"0x%x\n", __func__, data);
			}
		} else {
			if ((vflip && (data & 0x1) == 0) ||
				(!vflip && (data & 0x1) == 1)) {
				data--;
				err |= ov5650_write_reg(client,
					OV5650_VREF_START_H,
					(data >> 8) & 0xFF);

				err |= ov5650_write_reg(client,
					OV5650_VREF_START_L, data & 0xFF);

				DPRINTK_OV5650("%s: OV5650_VREF_START_L=" \
					"0x%x\n", __func__, data);
			}
		}
	}

	if (err) {
		printk(KERN_ERR "OV5650: Error setting orientation.%d", err);
		return err;
	} else {
		pvc->current_value = (u32)val;
		sensor->orientation = val;
	}

	return err;
}

/*
 * Calculates the PClk.
 * 1) Read pclk related params
 * 2) Calc vcoclk
 *      vcoclk = xclk * PLL_multiplier * div45 / pll_pre_div
 * 3) Calc pclk
 *      Pclk = vcoclk / divs / divl / div2p5 / outblkdiv
 */
static int ov5650_calc_pclk(struct v4l2_int_device *s,
	enum image_size_ov isize)
{
	struct ov5650_sensor *sensor = s->priv;
	struct ov5650_sensor_settings *ss = &sensor_settings[isize];

	sensor->freq.vcoclk = (sensor->freq.xclk * ss->clk.div_p *
		ss->clk.seld5 * 2) / ss->clk.pre_div_q1;

	sensor->freq.pclk = (sensor->freq.vcoclk * 2) / ss->clk.div_l /
		ss->clk.div_s / ss->clk.out_blk_div / ss->clk.seld2p5_q1;

	DPRINTK_OV5650("ov5650_calc_pclk: div_s=%d, seld5=%d, "
		"div_l=%d, seld2p5_q1=%d(2x), pll_mult=%d, pre_div_q1=%d(2x), "
		"out_blk_div=%d, vcoclk=%d, pclk=%d, # lanes=%d\n",
		ss->clk.div_s, ss->clk.seld5,
		ss->clk.div_l, ss->clk.seld2p5_q1,
		ss->clk.div_p, ss->clk.pre_div_q1,
		ss->clk.out_blk_div, sensor->freq.vcoclk, sensor->freq.pclk,
		ss->clk.div_l+1);

	return 0;
}

/*
 * Set Lens Correction
 */
static int ov5650_set_lens_correction(u16 enable_lens_correction,
	struct v4l2_int_device *s, struct private_vcontrol *pvc,
	enum image_size_ov isize)
{
	int data;
	int err = 0;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		if (enable_lens_correction) {

			err = ov5650_write_regs(client, len_correction_tbl);

			/* enable isp[lenc] */
			err |= ov5650_read_reg(client, 1,
				OV5650_ISP_CTRL_00, &data);
			data |= OV5650_ISP_CTRL_00_LENC_EN_MASK;
			err |= ov5650_write_reg(client,
				OV5650_ISP_CTRL_00, data);

			DPRINTK_OV5650("OV5650: enabling lens correction\n");

		} else {  /* disable lens correction */
			err = ov5650_read_reg(client, 1,
				OV5650_ISP_CTRL_00, &data);
			data &= ~OV5650_ISP_CTRL_00_LENC_EN_MASK;
			err |= ov5650_write_reg(client,
				OV5650_ISP_CTRL_00, data);
		}
	}

	if (err)
		printk(KERN_ERR "OV5650: Error setting lens correction=%d.\n",
			enable_lens_correction);
	else {
		if (pvc)
			pvc->current_value = enable_lens_correction;
	}

	return err;
}

/*
 * Get OTP Data
 */
int ov5650_get_otp_data(struct v4l2_int_device *s, u8 *otp_data, u16 otp_size)
{
	bool orig_streaming_state;
	u32 val, val_sum;
	int err = 0, addr, i, j;
	static enum v4l2_power orig_power_state;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *c = to_i2c_client(sensor->dev);

	orig_power_state = current_power_state;
	if (current_power_state != V4L2_POWER_ON) {
		/* Turn Power & Clk On */
		err |= sensor->pdata->power_set(sensor->dev, V4L2_POWER_ON);
	}

	orig_streaming_state = sensor->streaming;
	if (sensor->streaming == false) {
		/* Take out of standby */
		err |= ov5650_write_reg(c, SYSTEM_CONTROL0, 0x2);
	}

	/* Wait till reading non-zero data from OTP */
	for (i = 0; i < 10; i++) {
		val_sum = 0;
		for (j = 0; j < 5; j++) {
			if (ov5650_write_reg(c, OV5650_OTP_SUB_ADDR, 0)) {
				printk(KERN_ERR "OV5650: Unable to write reg\n");
				sensor->pdata->power_set(sensor->dev,
							orig_power_state);
				return -ENODEV;
			}
			if (ov5650_read_reg(c, 1, OV5650_OTP_DATA, &val)) {
				printk(KERN_ERR "OV5650: Unable to read OTP reg\n");
				sensor->pdata->power_set(sensor->dev,
							orig_power_state);
				return -ENODEV;
			}
			val_sum += val;
			if (val_sum != 0)
				break;
			udelay(100);
		}
	}

	if (val_sum > 0) {

		/* Read OTP data */
		for (addr = 0; addr < otp_size; ++addr) {
			err |= ov5650_write_reg(c, OV5650_OTP_SUB_ADDR, addr);
			err |= ov5650_read_reg(c, 1, OV5650_OTP_DATA, &val);
			otp_data[addr] = val & 0xff;
		}

		if (orig_streaming_state == false) {
			/* Put back in standby */
			err |= ov5650_write_reg(c, SYSTEM_CONTROL0, 0x42);
		}

		if (orig_power_state != V4L2_POWER_ON) {
			/* Turn Power & Clk Off */
			err |= sensor->pdata->power_set(sensor->dev,
				orig_power_state);
		}

	} else {
		printk(KERN_ERR "OV5650: Cannot read OTP\n");
		err = -ENODEV;
	}

	return err;
}

/*
 * Get Flicker Detect
 *
 *  Flicker mode need to be initialized to manual (debug) mode and then
 *  switched to auto mode while streaming to get the default status to work.
 */
int ov5650_get_flicker_detect(struct v4l2_int_device *s, u32 *flicker)
{
	u32 val;
	int err = 0;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *c = to_i2c_client(sensor->dev);

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		/* Make sure flicker has been taken out of manual mode */
		if (ov5650_flicker_manual_mode) {
			err |= ov5650_read_reg(c, 1, OV5650_5060HZ_CTRL_01,
				&val);
			val &= ~OV5650_5060HZ_CTRL_01_DEBUG_MASK;
			err |= ov5650_write_reg(c, OV5650_5060HZ_CTRL_01,
				val);
			DPRINTK_OV5650("setting OV5650_5060HZ_CTRL_01=0x%x\n",
				val);
			ov5650_flicker_manual_mode = false;
		}

		/* Read Flicker result */
		err = ov5650_read_reg(c, 1, OV5650_5060HZ_CTRL_0C, flicker);
		*flicker &= OV5650_5060HZ_CTRL_0C_BAND50_MASK;
		/* DPRINTK_OV5650("Flicker status = %d\n", *flicker); */
	} else {
		printk(KERN_ERR "OV5650: Cannot read flicker result\n");
		err = -ENODEV;
	}

	return err;
}

/*
 * Set Defect Pixel Correction
 */
static int ov5650_set_defect_pixel_correction(u16 enbl_defect_pixel_corr,
	struct v4l2_int_device *s, struct private_vcontrol *pvc)
{
	int data, err = 0;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;

	DPRINTK_OV5650("setting defect pixel correction=%d, power=%d\n",
		enbl_defect_pixel_corr,
		(current_power_state == V4L2_POWER_ON) || sensor->resuming);

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		err |= ov5650_read_reg(c, 1, OV5650_ISP_CTRL_00, &data);
		if (enbl_defect_pixel_corr) {
			/* enable defect correction */
			data |= (OV5650_ISP_CTRL_00_BC_EN_MASK |
				OV5650_ISP_CTRL_00_WC_EN_MASK);
		} else {
			/* disable defect correction */
			data &= ~(OV5650_ISP_CTRL_00_BC_EN_MASK |
				OV5650_ISP_CTRL_00_WC_EN_MASK);
		}
		err |= ov5650_write_reg(c, OV5650_ISP_CTRL_00, data);

		DPRINTK_OV5650("%s: Setting OV5650_ISP_ENBL_0=0x%x\n",
			__func__, data);
	}

	if (err)
		dev_err(&c->dev, "Error setting defect pixel corr=%d.\n",
			enbl_defect_pixel_corr);
	else {
		if (pvc)
			pvc->current_value = enbl_defect_pixel_corr;
	}

	return err;
}

static int ov5650_param_handler(u32 ctrlval,
			struct v4l2_int_device *s)
{
	int err = 0, i;
	u32 flicker;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *c = to_i2c_client(sensor->dev);
	struct ov5650_sensor_params sensor_params;
	struct ov5650_sensor_regif sensor_reg;
	struct ov5650_flash_params flash_params;
	struct ov5650_min_readout_params min_readout;
	struct camera_params_control camctl;
	struct private_vcontrol *pvc;
	struct ov5650_sensor_settings *ss =
		&sensor_settings[sensor->isize];

	if (!ctrlval)
		return -EINVAL;

	if (copy_from_user(&camctl, (void *)ctrlval,
			   sizeof(struct camera_params_control))) {
		printk(KERN_ERR "ov5650: Failed copy_from_user\n");
		return -EINVAL;
	}

	i = find_vctrl_private(camctl.op);
	if (i < 0)
		return -EINVAL;
	pvc = &video_control_private[i];

	if (camctl.xaction == IOCTL_RD) {
		switch (camctl.op) {
		case SENSOR_REG_REQ:
			if ((current_power_state != V4L2_POWER_ON) &&
				!sensor->resuming) {
				printk(KERN_ERR "OV5650: I2C Read Err: " \
					"Power Off\n");
				err = -EINVAL;
				break;
			}
			if (copy_from_user(&sensor_reg,
				(struct ov5650_sensor_regif *)camctl.data_in,
				sizeof(struct ov5650_sensor_regif)) == 0) {
				err =  ov5650_read_reg(c, sensor_reg.len,
					sensor_reg.addr, &sensor_reg.val);
				DPRINTK_OV5650("SENSOR_REG_REQ IOCTL read-" \
					"%d: 0x%x=0x%x\n", sensor_reg.len,
					sensor_reg.addr, sensor_reg.val);
			}
			if (copy_to_user((void *)camctl.data_in, &(sensor_reg),
				sizeof(sensor_reg))) {
				printk(KERN_ERR "ov5650: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case SENSOR_ID_REQ:
			if (sizeof(sensor->sensor_id) > camctl.data_in_size) {
				dev_err(&c->dev,
					"V4L2_CID_PRIVATE_S_PARAMS IOCTL " \
					"DATA SIZE ERROR: src=%d dst=%d\n",
					sizeof(sensor->sensor_id),
					camctl.data_in_size);
				err = -EINVAL;
				break;
			}

			if (copy_to_user((void *)camctl.data_in,
					 &(sensor->sensor_id),
					 sizeof(sensor->sensor_id))) {
				printk(KERN_ERR "ov5650: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case COLOR_BAR:
			camctl.data_in = pvc->current_value;
			if (copy_to_user((void *)ctrlval, &camctl,
					sizeof(struct camera_params_control))) {
				printk(KERN_ERR "ov5650: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case FLASH_NEXT_FRAME:
			if (sizeof(sensor->flash) > camctl.data_in_size) {
				dev_err(&c->dev,
					"V4L2_CID_PRIVATE_S_PARAMS IOCTL " \
					"DATA SIZE ERROR: src=%d dst=%d\n",
					sizeof(sensor->flash),
					camctl.data_in_size);
				err = -EINVAL;
				break;
			}

			if (copy_to_user((void *)camctl.data_in,
					 &(sensor->flash),
					 sizeof(sensor->flash))) {
				dev_err(&c->dev, "ov5650: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case ORIENTATION:
			camctl.data_in = pvc->current_value;
			if (copy_to_user((void *)ctrlval, &camctl,
					sizeof(struct camera_params_control))) {
				printk(KERN_ERR "ov5650: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case SENSOR_PARAMS_REQ:
			sensor_params.line_time = sensor->exposure.line_time;
			sensor_params.gain_frame_delay =
				OV5650_GAIN_FRAME_DELAY;
			sensor_params.exp_time_frame_delay =
				OV5650_EXP_TIME_FRAME_DELAY;
			sensor_params.total_vertical_size =
				ss->frame.total_vert_size;
			sensor_params.line_length_clocks =
				ss->frame.total_horz_size;
			sensor_params.h_output_size =
				ss->frame.h_output_size;
			sensor_params.v_output_size =
				ss->frame.v_output_size;
			sensor_params.binning_sensitivity =
				ss->frame.binning_sensitivity;

			if (sizeof(sensor_params) >
					camctl.data_in_size) {
				dev_err(&c->dev,
					"V4L2_CID_PRIVATE_S_PARAMS IOCTL " \
					"DATA SIZE ERROR: src=%d dst=%d\n",
					sizeof(sensor_params),
					camctl.data_in_size);
				err = -EINVAL;
				break;
			}

			if (copy_to_user((void *)camctl.data_in,
					 &(sensor_params),
					 sizeof(sensor_params))) {
				printk(KERN_ERR "ov5650: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case SENSOR_OTP_REQ:
			if (sizeof(ov5650_otp_data) < camctl.data_in_size) {
				printk(KERN_ERR "Requested OTP data size " \
					" greater than available OTP data: %d" \
					" vrs %d\n", sizeof(ov5650_otp_data),
					camctl.data_in_size);
				err = -EINVAL;
				break;
			}
			if (ov5650_get_otp_data(s, ov5650_otp_data,
					camctl.data_in_size)) {
				printk(KERN_ERR "ov5650: Get OTP Data error\n");

				err = -EINVAL;
				break;
			}
			if (copy_to_user((void *)camctl.data_in,
				&(ov5650_otp_data), camctl.data_in_size)) {
				printk(KERN_ERR "ov5650: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case DEFECT_PIXEL_CORRECTION:
			camctl.data_in = pvc->current_value;
			if (copy_to_user((void *)ctrlval, &camctl,
					sizeof(struct camera_params_control))) {
				dev_err(&c->dev, "ov5650: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case FLICKER_DETECT_REQ:
			if (ov5650_get_flicker_detect(s, &flicker)) {
				printk(KERN_ERR "ov5650: Get flicker detect error\n");
				err = -EINVAL;
				break;
			}
			if (copy_to_user((void *)camctl.data_in,
				&flicker, camctl.data_in_size)) {
				printk(KERN_ERR "ov5650: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case CROPPED_READOUT_REQ:
			camctl.data_in = pvc->current_value;
			if (copy_to_user((void *)ctrlval, &camctl,
					sizeof(struct camera_params_control))) {
				dev_err(&c->dev, "ov5650: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case USE_MIN_SIZE_READOUT:
			if (sizeof(sensor->min_readout) > camctl.data_in_size) {
				dev_err(&c->dev,
					"V4L2_CID_PRIVATE_S_PARAMS IOCTL " \
					"DATA SIZE ERROR: src=%d dst=%d\n",
					sizeof(sensor->min_readout),
					camctl.data_in_size);
				err = -EINVAL;
				break;
			}

			if (copy_to_user((void *)camctl.data_in,
					 &(sensor->min_readout),
					 sizeof(sensor->min_readout))) {
				dev_err(&c->dev, "ov5650: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		default:
			dev_err(&c->dev, "Unrecognized op %d\n",
				camctl.op);
			break;
		}
	} else if (camctl.xaction == IOCTL_WR) {
		switch (camctl.op) {
		case SENSOR_REG_REQ:
			if ((current_power_state != V4L2_POWER_ON) &&
				!sensor->resuming) {
				printk(KERN_ERR "OV5650: Reg Write Err: Power Off\n");
				err = -EINVAL;
				break;
			}
			if (copy_from_user(&sensor_reg,
				(struct ov5650_sensor_regif *)camctl.data_out,
				sizeof(struct ov5650_sensor_regif)) == 0) {

				DPRINTK_OV5650("SENSOR_REG_REQ IOCTL write-" \
					"%d: 0x%x=0x%x\n", sensor_reg.len,
					sensor_reg.addr, sensor_reg.val);

				/* ov5650_write_reg only supports 1-byte writes
				*/
				if (sensor_reg.len == 1) {
					err = ov5650_write_reg(c,
						sensor_reg.addr,
						sensor_reg.val);
				} else if (sensor_reg.len == 2) {
					err = ov5650_write_reg(c,
						sensor_reg.addr,
						(sensor_reg.val & 0xff00) >> 8);
					err |= ov5650_write_reg(c,
						sensor_reg.addr + 1,
						sensor_reg.val & 0xff);
				} else if (sensor_reg.len == 4) {
					err = ov5650_write_reg(c,
						sensor_reg.addr,
						(sensor_reg.val & 0xff000000)
						>> 24);
					err |= ov5650_write_reg(c,
						sensor_reg.addr + 1,
						(sensor_reg.val & 0xff0000)
						>> 16);
					err |= ov5650_write_reg(c,
						sensor_reg.addr + 2,
						(sensor_reg.val & 0xff00)
						>> 8);
					err |= ov5650_write_reg(c,
						sensor_reg.addr + 3,
						sensor_reg.val & 0xff);
				} else {
					dev_err(&c->dev, "Error: " \
						"SENSOR_REG_REQ IOCTL " \
						"length must = 1, 2, or 4\n");
				}
			}
			break;
		case COLOR_BAR:
			err = ov5650_set_color_bar_mode(camctl.data_out,
				s, pvc);
		case FLASH_NEXT_FRAME:
			if (sizeof(flash_params) < camctl.data_out_size) {
				dev_err(&c->dev,
					"V4L2_CID_PRIVATE_S_PARAMS IOCTL " \
					"DATA SIZE ERROR: src=%d dst=%d\n",
					camctl.data_out_size,
					sizeof(flash_params));
				err = -EINVAL;
				break;
			}

			if (copy_from_user(&flash_params,
				(struct ov5650_flash_params *)camctl.data_out,
				sizeof(struct ov5650_flash_params)) == 0) {
				err = ov5650_set_flash_next_frame(&flash_params,
					s, pvc);
			}
			break;
		case ORIENTATION:
			err = ov5650_set_orientation(camctl.data_out, s,
				pvc, sensor->isize);
			break;
		case LENS_CORRECTION:
			err = ov5650_set_lens_correction(camctl.data_out, s,
				pvc, sensor->isize);
			break;
		case DEFECT_PIXEL_CORRECTION:
			err = ov5650_set_defect_pixel_correction(
				camctl.data_out, s, pvc);
			break;
		case CROPPED_READOUT_REQ:
			pvc->current_value = camctl.data_out;
			break;
		case USE_MIN_SIZE_READOUT:
			if (sizeof(min_readout) < camctl.data_out_size) {
				dev_err(&c->dev,
					"V4L2_CID_PRIVATE_S_PARAMS IOCTL " \
					"DATA SIZE ERROR: src=%d dst=%d\n",
					camctl.data_out_size,
					sizeof(min_readout));
				err = -EINVAL;
				break;
			}

			if (copy_from_user(&(sensor->min_readout),
			(struct ov5650_min_readout_params *)camctl.data_out,
			sizeof(struct ov5650_min_readout_params)) == 0) {
				pvc->current_value =
					sensor->min_readout.enable_min_size;
			}
			break;
		default:
			dev_err(&c->dev, "Unrecognized op %d\n",
				camctl.op);
			break;
		}
	} else {
		dev_err(&c->dev, "Unrecognized action %d\n",
			camctl.xaction);
		err = -EINVAL;
	}

	return err;
}

/* Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum image_size_ov
ov5650_find_size(struct v4l2_int_device *s, unsigned int width,
	unsigned int height)
{
	enum image_size_ov size;

	if ((width > ov5650_sizes[SIZE_2M].width) ||
		(height > ov5650_sizes[SIZE_2M].height))
		size = SIZE_5M;
	else if ((width > ov5650_sizes[SIZE_1_25M].width) ||
		(height > ov5650_sizes[SIZE_1_25M].height))
		size = SIZE_2M;
	else if ((width > ov5650_sizes[SIZE_315K].width) ||
		(height > ov5650_sizes[SIZE_315K].height))
		size = SIZE_1_25M;
	else if ((width > ov5650_sizes[SIZE_80K].width) ||
		(height > ov5650_sizes[SIZE_80K].height))
		size = SIZE_315K;
	else
		size = SIZE_80K;

	DPRINTK_OV5650("find_size: Req Width=%d, "
			"Find Size=%dx%d\n",
			width, (int)ov5650_sizes[size].width,
			(int)ov5650_sizes[size].height);

	return size;
}

/*
 * Set CSI2 Virtual ID.
 */
static int ov5650_set_virtual_id(struct i2c_client *client, u32 id)
{
	return ov5650_write_reg(client, OV5650_MIPI_CTRL14, (0x3 & id) << 6 |
									0x2A);
}

/*
 * Calculates the MIPIClk.
 */
static u32 ov5650_calc_mipiclk(struct v4l2_int_device *s,
	enum image_size_ov isize)
{
	struct ov5650_sensor *sensor = s->priv;
	struct ov5650_sensor_settings *ss = &sensor_settings[isize];

	sensor->freq.mipiclk = (sensor->freq.xclk * ss->clk.div_p *
		ss->clk.seld5 * 2) /
		ss->clk.div_s / ss->clk.div_m / ss->clk.pre_div_q1;

	DPRINTK_OV5650("mipiclk=%u  pre_divider=%u(2x)  seld5=%u " \
		"pll_mult=%u  r_divs=%u r_divm=%u\n",
		sensor->freq.mipiclk, ss->clk.pre_div_q1, ss->clk.seld5,
		ss->clk.div_p, ss->clk.div_s, ss->clk.div_m);

	return sensor->freq.mipiclk;
}

/**
 * ov5650_configure_frame - Setup the frame, clock and exposure parmas in the
 * sensor_settings array.
 *
 * @s: pointer to standard V4L2 device structure
 * @isize: current image size
 *
 * The sensor_settings is a common list used by all image sizes & frame
 * rates that is filled in by this routine.
 */
int ov5650_configure_frame(struct v4l2_int_device *s,
			    enum image_size_ov isize)
{
	bool hmirror = false, vflip = false;
	u8 lut1[6] = { 0, 1, 1, 1, 2, 3 };  /* SELD5   lut */
	u8 lut2[6] = { 0, 0, 1, 1, 2, 3 };  /* SELD2P5 lut - 2X input */
	u8 lut3[17] = { 0, 0, 0, 1, 2, 3,   /* PREDIV  lut - 2X input */
		4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7 };
	u32 data;
	int err = 0;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	struct ov5650_sensor_settings *ss = &sensor_settings[isize];

	data = (lut1[ss->clk.seld5] << OV5650_PLL_CTRL_00_R_SELD5_SHIFT) &
		OV5650_PLL_CTRL_00_R_SELD5_MASK;
	data |= ((ss->clk.div_l % 2) << OV5650_PLL_CTRL_00_R_DIVL_SHIFT) &
		OV5650_PLL_CTRL_00_R_DIVL_MASK;
	data |= (lut2[ss->clk.seld2p5_q1] << OV5650_PLL_CTRL_00_R_SELD2P5_SHIFT)
		& OV5650_PLL_CTRL_00_R_SELD2P5_MASK;
	data |= 0x08;
	err |= ov5650_write_reg(client, OV5650_PLL_CTRL_00, data);

	data = ((ss->clk.div_s - 1) << OV5650_PLL_CTRL_01_R_DIVS_SHIFT) &
		OV5650_PLL_CTRL_01_R_DIVS_MASK;
	data |= ((ss->clk.div_m - 1) << OV5650_PLL_CTRL_01_R_DIVM_SHIFT) &
		OV5650_PLL_CTRL_01_R_DIVM_MASK;
	err |= ov5650_write_reg(client, OV5650_PLL_CTRL_01, data);

	data = (ss->clk.div_p << OV5650_PLL_CTRL_02_R_DIVP_SHIFT) &
		OV5650_PLL_CTRL_02_R_DIVP_MASK;
	err |= ov5650_write_reg(client, OV5650_PLL_CTRL_02, data);

	data = (lut3[ss->clk.pre_div_q1] << OV5650_PLL_CTRL_03_R_PREDIV_SHIFT) &
		OV5650_PLL_CTRL_03_R_PREDIV_MASK;
	err |= ov5650_write_reg(client, OV5650_PLL_CTRL_03, data);

	err |= ov5650_write_reg(client, OV5650_H_OUTPUT_SIZE_H,
		 (ss->frame.h_output_size >> 8) & 0xFF);

	err |= ov5650_write_reg(client, OV5650_H_OUTPUT_SIZE_L,
		 ss->frame.h_output_size & 0xFF);

	err |= ov5650_write_reg(client, OV5650_V_OUTPUT_SIZE_H,
		 (ss->frame.v_output_size >> 8) & 0xFF);

	err |= ov5650_write_reg(client, OV5650_V_OUTPUT_SIZE_L,
		 ss->frame.v_output_size & 0xFF);

	if (ss->frame.man_hv_ref_start) {
		data = OV5650_HREF_START_MAN_H_HVS_MAN_MASK |
			((ss->frame.h_ref_start >> 8) &
			OV5650_HREF_START_MAN_H_MASK);
		err |= ov5650_write_reg(client, OV5650_HREF_START_MAN_H, data);

		err |= ov5650_write_reg(client, OV5650_HREF_START_MAN_L,
			 ss->frame.h_ref_start & 0xFF);

		err |= ov5650_write_reg(client, OV5650_VREF_START_MAN_H,
			 (ss->frame.v_ref_start >> 8) & 0xFF);

		err |= ov5650_write_reg(client, OV5650_VREF_START_MAN_L,
			 ss->frame.v_ref_start & 0xFF);

		/* VREF_START regs may get used depending on VFLIP */
		err |= ov5650_write_reg(client, OV5650_VREF_START_H,
			 (ss->frame.v_ref_start >> 8) & 0xFF);

		err |= ov5650_write_reg(client, OV5650_VREF_START_L,
			 ss->frame.v_ref_start & 0xFF);
	} else {
		err |= ov5650_write_reg(client, OV5650_HREF_START_H,
			 (ss->frame.h_ref_start >> 8) & 0xFF);

		err |= ov5650_write_reg(client, OV5650_HREF_START_L,
			 ss->frame.h_ref_start & 0xFF);

		err |= ov5650_write_reg(client, OV5650_VREF_START_H,
			 (ss->frame.v_ref_start >> 8) & 0xFF);

		err |= ov5650_write_reg(client, OV5650_VREF_START_L,
			 ss->frame.v_ref_start & 0xFF);
	}

	err |= ov5650_write_reg(client, OV5650_HREF_WIDTH_H,
		 (ss->frame.h_ref_width >> 8) & 0xFF);

	err |= ov5650_write_reg(client, OV5650_HREF_WIDTH_L,
		 ss->frame.h_ref_width & 0xFF);

	err |= ov5650_write_reg(client, OV5650_VREF_WIDTH_H,
		 (ss->frame.v_ref_width >> 8) & 0xFF);

	err |= ov5650_write_reg(client, OV5650_VREF_WIDTH_L,
		 ss->frame.v_ref_width & 0xFF);

	err |= ov5650_write_reg(client, OV5650_TOTAL_VERT_SIZE_H,
		 (ss->frame.total_vert_size_min >> 8) & 0xFF);

	err |= ov5650_write_reg(client, OV5650_TOTAL_VERT_SIZE_L,
		 ss->frame.total_vert_size_min & 0xFF);

	ss->frame.total_vert_size = ss->frame.total_vert_size_min;

	err |= ov5650_write_reg(client, OV5650_TOTAL_HORZ_SIZE_H,
		 (ss->frame.total_horz_size_min >> 8) & 0xFF);

	err |= ov5650_write_reg(client, OV5650_TOTAL_HORZ_SIZE_L,
		 ss->frame.total_horz_size_min & 0xFF);

	ss->frame.total_horz_size = ss->frame.total_horz_size_min;

	/*
	 * Update Mirror, Flip, Subsample, and Binning Control Registers
	 */
	DPRINTK_OV5650("%s: sensor->orientation = %d\n",
		__func__, sensor->orientation);

	if (sensor->orientation == OV5650_HORZ_FLIP_ONLY ||
	    sensor->orientation == OV5650_HORZ_FLIP_AND_VERT_FLIP)
		hmirror = true;
	if (sensor->orientation == OV5650_VERT_FLIP_ONLY ||
	    sensor->orientation == OV5650_HORZ_FLIP_AND_VERT_FLIP)
		vflip = true;

	/* Update OV5650_RESERVED_505A/B register */
	if (hmirror)
		data = 0x0A2E;
	else
		data = 0x0012;
	err |= ov5650_write_reg(client,
		OV5650_RESERVED_505A, data >> 8);
	err |= ov5650_write_reg(client,
		OV5650_RESERVED_505B, data & 0xFF);

	/* Update OV5650_TIMING_TC_18 register */
	data = 0x80;
	if (hmirror)
		data |= OV5650_TIMING_TC_18_MIRROR_MASK;
	if (vflip)
		data |= OV5650_TIMING_TC_18_VFLIP_MASK;
	if (ss->frame.v_subsample == 2)
		data |= OV5650_TIMING_TC_18_VSUB2_MASK;
	else if (ss->frame.v_subsample == 4)
		data |= OV5650_TIMING_TC_18_VSUB4_MASK;
	else if (ss->frame.v_subsample == 8)
		data |= (OV5650_TIMING_TC_18_VSUB2_MASK |
			OV5650_TIMING_TC_18_VSUB4_MASK);

	err |= ov5650_write_reg(client,
		OV5650_TIMING_TC_18, data);

	/* Update OV5650_ARRAY_CONTROL_01 register */
	data = 0x2F;
	if (hmirror)
		data &= ~0x10;
	else
		data |= 0x10;
	if (ss->frame.h_subsample > 1) {
		if (ss->frame.h_binning)
			data |= OV5650_ARRAY_CTRL_01_HORZ_BIN_SUB_EN_MASK;
		else {
			data |= OV5650_ARRAY_CTRL_01_HORZ_BIN_SUB_EN_MASK &
				OV5650_ARRAY_CTRL_01_HORZ_SUB_EN_MASK;
		}
	}
	err |= ov5650_write_reg(client,
		OV5650_ARRAY_CTRL_01, data);

	/* Update OV5650_ANALOG_CTRL_D register */
	if (ss->frame.h_subsample > 1) {
		data = 0x02;
		if (ss->frame.h_subsample == 8)
			data |= 0x80;
		if (ss->frame.v_binning == 1)
			data |= OV5650_ANALOG_CTRL_D_VBIN_EN_MASK;
		err |= ov5650_write_reg(client,
			OV5650_ANALOG_CTRL_D, data);
	}

	/* Update OV5650_ISP_CTRL_02 register */
	data = 0;
	if (ss->frame.h_subsample >= 4)
		data = OV5650_ISP_CTRL_02_VAP_EN_MASK;
	err |= ov5650_write_reg(client,
		OV5650_ISP_CTRL_02, data);

	/* Update OV5650_VAP_CTRL_01 register */
	data = (ss->frame.h_subsample / 4) <<
		OV5650_VAP_CTRL_01_HSUB_COEF_SHIFT;
	err |= ov5650_write_reg(client,
		OV5650_VAP_CTRL_01, data);

	sensor->isize = isize;
	if (err)
		return -EIO;
	else
		return 0;
}

/*
 * Configure the ov5650 for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the OV5650.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int ov5650_configure(struct v4l2_int_device *s)
{
	enum image_size_ov isize;
	/*
	u16 data = 0;
	*/
	int err = 0, i = 0;
	u32 mipiclk;
	enum pixel_format_ov pfmt = RAW10;
	struct ov5650_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &sensor->pix;
	struct i2c_client *client = sensor->i2c_client;
	struct vcontrol *lvc = NULL;
	struct private_vcontrol *pvc = NULL;

	switch (pix->pixelformat) {

	case V4L2_PIX_FMT_SGRBG10:
		pfmt = RAW10;
		break;
	}

	/* Set receivers virtual channel before sensor setup starts.
	 * Only set the sensors virtual channel after all other setup
	 * for the sensor is complete.
	 */
	isp_csi2_ctx_config_virtual_id(0, OV5650_CSI2_VIRTUAL_ID);
	isp_csi2_ctx_update(0, false);

	isize = ov5650_find_size(s, pix->width, pix->height);

	printk(KERN_INFO "ov5650_configure: isize=%d, Req Size=%dx%d, " \
		"Find Size = %dx%d, fps=%d/%d, fmt=%8.8x\n", \
		isize, pix->width, pix->height,
		(int)ov5650_sizes[isize].width,
		(int)ov5650_sizes[isize].height,
		sensor->timeperframe.denominator,
		sensor->timeperframe.numerator, pix->pixelformat);

	/* Reset ISP MIPI */
	isp_csi2_ctrl_config_if_enable(false);
	isp_csi2_ctrl_update(false);

	/* Reset Sensor & put in Standby */
	ov5650_write_reg(client, SYSTEM_CONTROL0, 0x82);
	mdelay(5);
	ov5650_write_reg(client, SYSTEM_CONTROL0, 0x42);
	mdelay(3);

	/* Set CSI2 common register settings */
	err = ov5650_write_regs(client, ov5650_common_csi2);
	if (err)
		return err;

	/* configure image size, pll, and pixel format */
	if (pix->pixelformat == V4L2_PIX_FMT_SGRBG10) {
		err = ov5650_write_regs(client, ov5650_common[isize]);
		if (err)
			return err;
	}

	/* Turn on 50-60 Hz Detection */
	if (isize != SIZE_5M) {
		err = ov5650_write_regs(client, ov5650_50_60_hz_detect_tbl);
		ov5650_flicker_manual_mode = true;
		if (err)
			return err;
	}

	/* Turn off bright center compensation */
	err = ov5650_write_regs(client, ov5650_disable_bright_center_comp_tbl);
	bright_center_compensation = false;
	if (err)
		return err;

	/* if the image size correspond to one of the base image sizes
		then we don't need to scale the image */
	sensor->hsize = pix->width;
	sensor->vsize = pix->height;

	/* Setup the ISP VP based on image format */
	if (pix->pixelformat == V4L2_PIX_FMT_SGRBG10) {
		isp_configure_interface_bridge(0x00);
		isp_csi2_ctrl_config_vp_out_ctrl(2);
		isp_csi2_ctrl_update(false);
	} else {
		isp_configure_interface_bridge(0x03);
		isp_csi2_ctrl_config_vp_out_ctrl(1);
		isp_csi2_ctrl_update(false);
	}

	/* Store image size */
	sensor->width = pix->width;
	sensor->height = pix->height;

	/* Update sensor clk, frame, & exposure params */
	ov5650_calc_pclk(s, isize);
	ov5650_init_exposure_params(s);
	err = ov5650_configure_frame(s, isize);
	if (err)
		return err;

	/* Setting of frame rate */
	err = ov5650_set_framerate(s, &sensor->timeperframe, isize);
	if (err)
		return err;

	mipiclk = ov5650_calc_mipiclk(s, isize);

	DPRINTK_OV5650("mipiclk = %d, hs_settle = %d\n",
		mipiclk, sensor_settings[isize].mipi.hs_settle);

	/* Send settings to ISP-CSI2 Receiver PHY */
	isp_csi2_calc_phy_cfg0(mipiclk,
		sensor_settings[isize].mipi.hs_settle,
		sensor_settings[isize].mipi.hs_settle);

	/* Set sensors virtual channel*/
	ov5650_set_virtual_id(client, OV5650_CSI2_VIRTUAL_ID);

	isp_csi2_ctrl_config_if_enable(true);
	isp_csi2_ctrl_update(false);

	/* Set initial exposure time */
	i = find_vctrl(V4L2_CID_EXPOSURE);
	if (i >= 0) {
		lvc = &video_control[i];
		ov5650_set_exposure_time(lvc->current_value,
			sensor->v4l2_int_device, lvc, isize);
	}

	/* Set initial gain */
	i = find_vctrl(V4L2_CID_GAIN);
	if (i >= 0) {
		lvc = &video_control[i];
		ov5650_set_gain(lvc->current_value,
			sensor->v4l2_int_device, lvc);
	}

	/* Set initial color bars */
	i = find_vctrl_private(COLOR_BAR);
	if (i >= 0) {
		pvc = &video_control_private[i];
		ov5650_set_color_bar_mode(pvc->current_value,
			sensor->v4l2_int_device, pvc);
	}

	/* Set initial flash mode */
	i = find_vctrl_private(FLASH_NEXT_FRAME);
	if (i >= 0) {
		pvc = &video_control_private[i];
		ov5650_set_flash_next_frame(&(sensor->flash),
			sensor->v4l2_int_device, pvc);
	}

	/* Set lens correction */
	i = find_vctrl_private(LENS_CORRECTION);
	if (i >= 0) {
		pvc = &video_control_private[i];
		ov5650_set_lens_correction(pvc->current_value,
			sensor->v4l2_int_device, pvc, isize);
	}

	/* Set orientation */
	i = find_vctrl_private(ORIENTATION);
	if (i >= 0) {
		pvc = &video_control_private[i];
		ov5650_set_orientation(pvc->current_value,
			sensor->v4l2_int_device, pvc, isize);
	}

	/* Set Defect Pixel Correction */
	i = find_vctrl_private(DEFECT_PIXEL_CORRECTION);
	if (i >= 0) {
		pvc = &video_control_private[i];
		ov5650_set_defect_pixel_correction(pvc->current_value,
			sensor->v4l2_int_device, pvc);
	}

	/* Take Sensor out of Standby */
	ov5650_write_reg(client, SYSTEM_CONTROL0, 0x02);
	sensor->streaming = true;

	DPRINTK_OV5650("OV5650: Starting streaming...\n");

	return err;
}


/* Detect if an ov5650 is present, returns a negative error number if no
 * device is detected, or pidl as version number if a device is detected.
 */
static int ov5650_detect(struct v4l2_int_device *s)
{
	u16 pid, rev = 0, rev1, rev2;
	u32 val, val_sum;
	int i, j;
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct ov5650_sensor_id *sensor_id = &(sensor->sensor_id);

	DPRINTK_OV5650("Entering %s:\n", __func__);

	if (!client)
		return -ENODEV;

	if (ov5650_read_reg(client, 1, OV5650_CHIP_ID_H, &val))
		return -ENODEV;
	pid = (val & 0xff) << 8;

	/* Read chip ID */
	if (ov5650_read_reg(client, 1, OV5650_CHIP_ID_L, &val))
		return -ENODEV;
	pid |= val & 0xf0;
	rev1 = val & 0xf;

	/* Check ID */
	if (pid != OV5650_PID) {
		/* We didn't read the pid we expected, so
		 * this must not be an OV5650.
		 */
		printk(KERN_ERR "OV5650: pid mismatch 0x%x\n", pid);

		return -ENODEV;
	}

	/* Take Sensor out of Standby */
	if (ov5650_write_reg(client, SYSTEM_CONTROL0, 0x02))
		return -ENODEV;

	/* Wait till reading non-zero data from OTP */
	for (i = 0; i < 10; i++) {
		val_sum = 0;
		for (j = 0; j < 5; j++) {
			if (ov5650_write_reg(client, OV5650_OTP_SUB_ADDR, 0))
				return -ENODEV;
			if (ov5650_read_reg(client, 1, OV5650_OTP_DATA, &val))
				return -ENODEV;
			val_sum += val;
			if (val_sum != 0)
				break;
			udelay(100);
		}
	}

	if (val_sum == 0) {
		printk(KERN_ERR "OV5650: Cannot read OTP\n");
		return -ENODEV;
	}

	if (ov5650_write_reg(client, OV5650_OTP_SUB_ADDR, 0xFD))
		return -ENODEV;
	if (ov5650_read_reg(client, 1, OV5650_OTP_DATA, &val))
		return -ENODEV;
	rev2 = val & 0x07;

	/* Put Sensor back in Standby */
	ov5650_write_reg(client, SYSTEM_CONTROL0, 0x42);

	/* Decode revision */
	if (rev1 == 0 && rev2 == 0)
		rev = 0x1A;
	else if (rev1 == 1 && rev2 == 0)
		rev = 0x1B;
	else if (rev1 == 1 && rev2 == 1)
		rev = 0x1C;
	else if (rev1 == 1 && rev2 == 2)
		rev = 0x1E;
	else if (rev1 == 1 && rev2 == 7)
		rev = 0x1D;

	/* Check ID & max supported rev */
	if (pid == OV5650_PID && rev >= 0x1A && rev <= 0x1E) {
		DPRINTK_OV5650("Detect success " \
			"(pid=0x%x rev=0x%x\n", pid, rev);

		sensor_id->model = pid;
		sensor_id->revision = rev;

		return 0;
	} else {
		/* We didn't read the values we expected, so
		 * this must not be an OV5650.
		 */
		printk(KERN_ERR "OV5650: pid mismatch 0x%x rev 0x%x\n",
			pid, rev);

		return -ENODEV;
	}
}

/*
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
						struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;
	return 0;
}

/*
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */

static int ioctl_g_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	struct vcontrol *lvc;
	int i;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case  V4L2_CID_EXPOSURE:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_GAIN:
		vc->value = lvc->current_value;
		break;
	}
	return 0;
}

/*
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	int err = -EINVAL;
	int i;
	struct ov5650_sensor *sensor = s->priv;
	struct vcontrol *lvc;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &video_control[i];

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		err = ov5650_set_exposure_time(vc->value, s, lvc,
			sensor->isize);
		break;
	case V4L2_CID_GAIN:
		err = ov5650_set_gain(vc->value, s, lvc);
		break;
	case V4L2_CID_PRIVATE_S_PARAMS:
		err = ov5650_param_handler(vc->value, s);
		break;
	case V4L2_CID_PRIVATE_G_PARAMS:
		break;
	}
	return err;
}

/*
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
 static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
				   struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
	break;
	default:
		return -EINVAL;
	}

	fmt->flags = ov5650_formats[index].flags;
	strlcpy(fmt->description, ov5650_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = ov5650_formats[index].pixelformat;

	return 0;
}


/*
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */

static int ioctl_try_fmt_cap(struct v4l2_int_device *s,
			     struct v4l2_format *f)
{
	int ifmt;
	enum image_size_ov isize;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (pix->width > ov5650_sizes[SIZE_5M].width)
		pix->width = ov5650_sizes[SIZE_5M].width;
	if (pix->height > ov5650_sizes[SIZE_5M].height)
		pix->height = ov5650_sizes[SIZE_5M].height;

	isize = ov5650_find_size(s, pix->width, pix->height);
	pix->width = ov5650_sizes[isize].width;
	pix->height = ov5650_sizes[isize].height;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == ov5650_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = ov5650_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width*2;
	pix->sizeimage = pix->bytesperline*pix->height;
	pix->priv = 0;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_SGRBG10:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}
	return 0;
}


/*
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
 static int ioctl_s_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct ov5650_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;

	rval = ioctl_try_fmt_cap(s, f);
	if (rval)
		return rval;

	sensor->pix = *pix;

	DPRINTK_OV5650("ioctl_s_fmt_cap: set width=%d, height=%d, fmt=%8.8x\n",
		sensor->pix.width, sensor->pix.height, sensor->pix.pixelformat);

	return 0;
}

/*
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct ov5650_sensor *sensor = s->priv;
	f->fmt.pix = sensor->pix;

	return 0;
}

/*
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	struct ov5650_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->timeperframe;

	return 0;
}

/*
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	int rval = 0;
	struct ov5650_sensor *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	struct v4l2_fract timeperframe_old;
	int desired_fps;
	timeperframe_old = sensor->timeperframe;
	sensor->timeperframe = *timeperframe;

	desired_fps = timeperframe->denominator / timeperframe->numerator;
	if ((desired_fps < OV5650_MIN_FPS) || (desired_fps > OV5650_MAX_FPS)) {
		sensor->timeperframe = timeperframe_old;
		printk(KERN_ERR "OV5650: Error setting FPS=%d/%d, " \
			"FPS must be between %d & %d,",
			timeperframe->denominator, timeperframe->numerator,
			OV5650_MIN_FPS, OV5650_MAX_FPS);
		rval = -EINVAL;
	} else {
		DPRINTK_OV5650("Setting FPS=%d\n", desired_fps);
		if ((current_power_state == V4L2_POWER_ON) ||
				sensor->resuming) {
			rval = ov5650_set_framerate(s, &sensor->timeperframe,
				sensor->isize);
		}
	}

	return rval;
}

/*
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct ov5650_sensor *sensor = s->priv;

	return sensor->pdata->priv_data_set(p);
}

/*
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
 static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct ov5650_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	struct omap34xxcam_hw_config hw_config;
	int rval, i;

	rval = ioctl_g_priv(s, &hw_config);
	if (rval) {
		printk(KERN_ERR "OV5650: Unable to get hw params\n");
		return rval;
	}

	rval = sensor->pdata->power_set(sensor->dev, on);
	if (rval < 0) {
		printk(KERN_ERR "OV5650: Unable to set the power state: "
			OV5650_DRIVER_NAME " sensor\n");
		return rval;
	}

	if (on == V4L2_POWER_ON) {
		/* set streaming off, standby */
		rval = ov5650_write_reg(c, SYSTEM_CONTROL0, 0x42);
		if (rval) {
			enum v4l2_power off = V4L2_POWER_OFF;
			printk(KERN_ERR "OV5650: Unable to write reg\n");
			sensor->pdata->power_set(sensor->dev, off);
			return rval;
		}
	} else {
		if (on == V4L2_POWER_OFF) {
			for (i = (ARRAY_SIZE(video_control_private) - 1); i >= 0; i--) {
				struct private_vcontrol *pvc = NULL;
				pvc = &video_control_private[i];
				pvc->current_value = pvc->default_value;
			}
		}
		sensor->streaming = false;
	}

	if ((current_power_state == V4L2_POWER_STANDBY) &&
			(on == V4L2_POWER_ON) &&
			(sensor->state == SENSOR_DETECTED)) {
		sensor->resuming = true;
		ov5650_configure(s);
	}

	if ((on == V4L2_POWER_ON) && (sensor->state == SENSOR_NOT_DETECTED)) {

		rval = ov5650_detect(s);
		if (rval < 0) {
			printk(KERN_ERR "OV5650: Unable to detect "
					OV5650_DRIVER_NAME " sensor\n");
			sensor->state = SENSOR_NOT_DETECTED;
			return rval;
		}
		sensor->state = SENSOR_DETECTED;
		pr_info(OV5650_DRIVER_NAME " Chip version 0x%02x detected\n",
			sensor->sensor_id.revision);
	}

	sensor->resuming = false;
	current_power_state = on;
	return 0;
}

/*
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call ov5650_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach.  The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.  Returns 0 if
 * ov5650 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct ov5650_sensor *sensor = s->priv;
	int err;

	err = ov5650_detect(s);
	if (err < 0) {
		printk(KERN_ERR "OV5650: Unable to detect " OV5650_DRIVER_NAME
			" sensor\n");
		return err;
	}

	pr_info(OV5650_DRIVER_NAME " chip version 0x%02x detected\n",
		sensor->sensor_id.revision);

	return 0;
}

/**
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 **/
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
					struct v4l2_frmsizeenum *frms)
{
	u16 use_cropped_readout = 0;
	int ifmt, i;
	struct private_vcontrol *pvc = NULL;

	i = find_vctrl_private(CROPPED_READOUT_REQ);
	if (i >= 0) {
		pvc = &video_control_private[i];
		use_cropped_readout = pvc->current_value;
	}

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frms->pixel_format == ov5650_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Have we already reached all discrete framesizes? */
	/* The SIZE_2M readout is a cropped readout and is the only readout
	   to be used when use_cropped_readout has been enabled */
	if (use_cropped_readout == 0) {
		/* Allow all but the SIZE_2M */
		if (frms->index >= (OV_NUM_IMAGE_SIZES - 1))
			return -EINVAL;
	} else {
		/* Allow only SIZE_2M */
		if (frms->index >= 1)
			return -EINVAL;
	}

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	if (use_cropped_readout == 0) {
		/* Allow all but the SIZE_2M */
		if (frms->index < SIZE_2M) {
			frms->discrete.width =
				ov5650_sizes[frms->index].width;
			frms->discrete.height =
				ov5650_sizes[frms->index].height;
		} else {
			frms->discrete.width =
				ov5650_sizes[frms->index + 1].width;
			frms->discrete.height =
				ov5650_sizes[frms->index + 1].height;
		}
	} else {
		/* Allow only SIZE_2M */
		frms->discrete.width = ov5650_sizes[SIZE_2M].width;
		frms->discrete.height = ov5650_sizes[SIZE_2M].height;
	}

	return 0;
}

const struct v4l2_fract ov5650_frameintervals[] = {
	{ .numerator = 3, .denominator = 3 },   /* 0 */
	{ .numerator = 3, .denominator = 6 },   /* 1 */
	{ .numerator = 3, .denominator = 9 },   /* 2 */
	{ .numerator = 3, .denominator = 12 },  /* 3 */
	{ .numerator = 3, .denominator = 15 },  /* 4 */
	{ .numerator = 3, .denominator = 18 },  /* 5 */
	{ .numerator = 3, .denominator = 21 },  /* 6 */
	{ .numerator = 3, .denominator = 24 },  /* 7 */
	{ .numerator = 1, .denominator = 10 },  /* 8 */
	{ .numerator = 1, .denominator = 12 },  /* 9 -  SIZE_5M max fps */
	{ .numerator = 1, .denominator = 15 },  /* 10 */
	{ .numerator = 1, .denominator = 16 },  /* 11 */
	{ .numerator = 1, .denominator = 20 },  /* 12 */
	{ .numerator = 1, .denominator = 21 },  /* 13 */
	{ .numerator = 1, .denominator = 25 },  /* 14 */
	{ .numerator = 1, .denominator = 26 },  /* 15 */
	{ .numerator = 1, .denominator = 30 },  /* 16 - SIZE_2M, SIZE_1_25M,
							SIZE_315K max fps */
	{ .numerator = 1, .denominator = 120 }, /* 17 - SIZE_80K max fps */
};

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					struct v4l2_frmivalenum *frmi)
{
	int ifmt, i;
	struct ov5650_sensor *sensor = s->priv;
	u16 use_min_readout = 0;
	struct private_vcontrol *pvc = NULL;

	i = find_vctrl_private(USE_MIN_SIZE_READOUT);
	if (i >= 0) {
		pvc = &video_control_private[i];
		use_min_readout = pvc->current_value;
	}

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frmi->pixel_format == ov5650_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Have we already reached all discrete framesizes? */

	if ((frmi->width == ov5650_sizes[SIZE_5M].width) &&
			(frmi->height == ov5650_sizes[SIZE_5M].height)) {
		/* The max framerate supported by SIZE_5M capture is 12 fps
		 */
		if (frmi->index > 9)
			return -EINVAL;
	} else if ((frmi->width == ov5650_sizes[SIZE_2M].width) &&
			(frmi->height == ov5650_sizes[SIZE_2M].height)) {
		/* The max framerate supported by SIZE_2M capture is 30 fps
		 */
		if (frmi->index > 16)
			return -EINVAL;
	} else if ((frmi->width == ov5650_sizes[SIZE_1_25M].width) &&
			(frmi->height == ov5650_sizes[SIZE_1_25M].height)) {
		/* The max framerate supported by SIZE_1_25M capture is 30 fps
		 */
		if (frmi->index > 16)
			return -EINVAL;
		/* If want to use min frame and there is a smaller size that
		   qualifies, do no allow this readout
		 */
		if ((use_min_readout == 1) &&
			 (sensor->min_readout.requested_width <=
				ov5650_sizes[SIZE_315K].width) &&
			 (sensor->min_readout.requested_height <=
				ov5650_sizes[SIZE_315K].height)) {
				DPRINTK_OV5650("ov5650: Readout 1.25M not " \
					"allowed when using Min Size\n");
				return -EINVAL;
		}
	} else if (((frmi->width == ov5650_sizes[SIZE_315K].width) &&
			(frmi->height == ov5650_sizes[SIZE_315K].height))) {
		/* The max framerate supported by SIZE_315K capture is 30 fps
		 */
		if (frmi->index > 16)
			return -EINVAL;
		/* If want to use min frame and there is a smaller size that
		   qualifies, do no allow this readout
		 */
		if ((use_min_readout == 1) &&
			 (sensor->min_readout.requested_width <=
				ov5650_sizes[SIZE_80K].width) &&
			 (sensor->min_readout.requested_height <=
				ov5650_sizes[SIZE_80K].height)) {
				DPRINTK_OV5650("ov5650:Readout 80K not "\
				 "allowed when using Min Size\n");
				return -EINVAL;
		}
	} else {
		if (frmi->index > 17)
			return -EINVAL;
	}

	frmi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	frmi->discrete.numerator =
				ov5650_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				ov5650_frameintervals[frmi->index].denominator;

	return 0;
}

static struct v4l2_int_ioctl_desc ov5650_ioctl_desc[] = {
	{vidioc_int_enum_framesizes_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
	{vidioc_int_dev_init_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_exit},
	{vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_priv_num,
	  (v4l2_int_ioctl_func *)ioctl_g_priv},
	{vidioc_int_init_num,
	  (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
	{vidioc_int_try_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_try_fmt_cap},
	{vidioc_int_g_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_s_parm},
	{vidioc_int_queryctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_queryctrl},
	{vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_s_ctrl},
};

static struct v4l2_int_slave ov5650_slave = {
	.ioctls		= ov5650_ioctl_desc,
	.num_ioctls	= ARRAY_SIZE(ov5650_ioctl_desc),
};

static struct v4l2_int_device ov5650_int_device = {
	.module	= THIS_MODULE,
	.name	= OV5650_DRIVER_NAME,
	.priv	= &ov5650,
	.type	= v4l2_int_type_slave,
	.u	= {
		.slave = &ov5650_slave,
	},
};

/*
 * ov5650_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int __init
ov5650_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ov5650_sensor *sensor = &ov5650;
	int err;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	sensor->pdata = client->dev.platform_data;

	if (!sensor->pdata) {
		printk(KERN_ERR "OV5650: No platform data?\n");
		return -ENODEV;
	}

	sensor->v4l2_int_device = &ov5650_int_device;
	sensor->i2c_client = client;
	sensor->dev = &client->dev;

	i2c_set_clientdata(client, sensor);

	/* Set sensor default values */
	sensor->pix.width = ov5650_sizes[SIZE_315K].width;
	sensor->pix.height = ov5650_sizes[SIZE_315K].height;
	sensor->pix.pixelformat = V4L2_PIX_FMT_SGRBG10;

	/* Set min/max limits */
	sensor->exposure.min_exp_time = OV5650_MIN_EXPOSURE;
	sensor->exposure.fps_max_exp_time = 33333;
	sensor->exposure.abs_max_exp_time = OV5650_MAX_EXPOSURE;
	sensor->exposure.min_linear_gain = OV5650_MIN_LINEAR_GAIN;
	sensor->exposure.max_linear_gain = OV5650_MAX_LINEAR_GAIN;

	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err)
		i2c_set_clientdata(client, NULL);

       ov5650_i2c_client = client;
	return 0;
}

/*
 * ov5650_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device. Complement of ov5650_probe().
 */
static int __exit
ov5650_remove(struct i2c_client *client)
{
	struct ov5650_sensor *sensor = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ov5650_id[] = {
	{ OV5650_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ov5650_id);

static struct i2c_driver ov5650sensor_i2c_driver = {
	.driver = {
		.name	= OV5650_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe	= ov5650_probe,
	.remove	= __exit_p(ov5650_remove),
	.id_table = ov5650_id,
};

/*
 * ov5650sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init ov5650sensor_init(void)
{
	int err;

	err = i2c_add_driver(&ov5650sensor_i2c_driver);
	if (err) {
		printk(KERN_ERR "OV5650: Failed to register" \
			OV5650_DRIVER_NAME ".\n");
		return err;
	}
	return 0;
}
late_initcall(ov5650sensor_init);

/*
 * ov5650sensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of ov5650sensor_init.
 */
static void __exit ov5650sensor_cleanup(void)
{
	i2c_del_driver(&ov5650sensor_i2c_driver);
}
module_exit(ov5650sensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OV5650 camera sensor driver");
