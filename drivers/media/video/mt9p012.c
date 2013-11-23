/*
 * mt9p012.c - mt9p012 sensor driver
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Contributors:
 *	Sameer Venkatraman <sameerv@ti.com>
 *	Sergio Aguirre <saaguirre@ti.com>
 *	Martinez Leonides
 *
 * Leverage OV9640.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>

#include <media/mt9p012.h>
#include "mt9p012_regs.h"

#include "omap34xxcam.h"

#define DRIVER_NAME  "mt9p012"

/* MT9P012 has 8/16/32 registers */
#define MT9P012_8BIT			1
#define MT9P012_16BIT			2
#define MT9P012_32BIT			4

/* terminating token for reg list */
#define MT9P012_TOK_TERM		0xFF

/* delay token for reg list */
#define MT9P012_TOK_DELAY		100

/* The ID values we are looking for */
#define MT9P012_MOD_ID			0x2800
#define MT9P012_MOD_ID_REV7		0x2801
#define MT9P013_MOD_ID			0x2803
#define MT9P012_MFR_ID			0x0006

/* FPS Capabilities */
#define MT9P012_MIN_FPS		11
#define MT9P012_DEF_FPS		15
#define MT9P012_MAX_FPS		30

#define MT9P012_I2C_RETRY_COUNT	5

#define CPU_CLK_LOCK    1
#define CPU_CLK_UNLOCK  0

#define MT9P012_XCLK_NOM_1 12000000
#define MT9P012_XCLK_NOM_2 24000000
#define MT9P012_XCLK_NOM_4 48000000

#define VIDEO_BIN_SUMMING 1

/* Still capture 5 MP */
#define MT9P012_IMAGE_WIDTH_MAX		2592
#define MT9P012_IMAGE_HEIGHT_MAX	1944
/* Still capture 3 MP and down to VGA, using ISP resizer */
#define MT9P012_IMAGE_WIDTH_MIN		2048
#define MT9P012_IMAGE_HEIGHT_MIN	1536

/* Video mode, for D1 NTSC, D1 PAL */
#define MT9P012_VIDEO_WIDTH_2X_BINN	1296
#define MT9P012_VIDEO_HEIGHT_2X_BINN	972

/* Sensor Video mode size for VGA, CIF, QVGA in 4x binning mode */
#define MT9P012_VIDEO_WIDTH_4X_BINN	648
#define MT9P012_VIDEO_HEIGHT_4X_BINN	486
/* To improve image quality in VGA */
#define MT9P012_CIF_PIXELS		(352 * 288)
#define MT9P012_QQVGA_PIXELS		(160 * 120)

/* Video mode, for QCIF, SQCIF */
#define MT9P012_VIDEO_WIDTH_4X_BINN_SCALED	216
#define MT9P012_VIDEO_HEIGHT_4X_BINN_SCALED	162

/* Default coarse integration times to get a good exposure */
#define MT9P012_COARSE_INT_TIME_216		 550
#define MT9P012_COARSE_INT_TIME_648		 550
#define MT9P012_COARSE_INT_TIME_216_30FPS	1350
#define MT9P012_COARSE_INT_TIME_648_30FPS	1350
#define MT9P012_COARSE_INT_TIME_1296		1000
#define MT9P012_COARSE_INT_TIME_3MP		1700
#define MT9P012_COARSE_INT_TIME_5MP		1700
#define MT9P012_COARSE_INT_TIME_INDEX		1
#define MT9P012_TST_PAT				0x0

/* Analog gain values */
#define MT9P012_MIN_ANALOG_GAIN			0x34
#define MT9P013_MIN_ANALOG_GAIN			0x2D
#define MT9P012_MAX_ANALOG_GAIN			0x1FF
#define MT9P012_GAIN_STEP			0x1
#define MT9P013_GAIN_STEP			0x1
#define MT9P012_DEF_LINEAR_GAIN			((u16)(2 * 256))
#define MT9P013_DEF_LINEAR_GAIN			((u16)(2 * 256))
#define MT9P012_MIN_LINEAR_GAIN			((u16)(2.0 * 256))
#define MT9P013_MIN_LINEAR_GAIN			((u16)(1.4 * 256))
#define MT9P012_MIN_LINEAR_GAIN_CAL_ADJ		((u16)(1.75 * 256))
#define MT9P013_MIN_LINEAR_GAIN_CAL_ADJ		((u16)(1.75 * 256))
#define MT9P012_MAX_LINEAR_GAIN			((u16)(31.75 * 256))
#define MT9P013_MAX_LINEAR_GAIN			((u16)(31.75 * 256))

/* Exposure time values (usecs)*/
#define MT9P012_DEF_MIN_EXPOSURE	100
#define MT9P012_DEF_MAX_EXPOSURE	1000000
#define MT9P012_DEF_EXPOSURE		20000
#define MT9P012_EXPOSURE_STEP		1
#define MT9P013_DEF_MIN_EXPOSURE	100
#define MT9P013_DEF_MAX_EXPOSURE	1000000
#define MT9P013_DEF_EXPOSURE		20000
#define MT9P013_EXPOSURE_STEP		1

/* Frame Delays */
#define MT9P012_GAIN_FRAME_DELAY 1
#define MT9P012_EXP_TIME_FRAME_DELAY 2

#define MT9P012_MAX_FRAME_LENGTH_LINES 0xFFFF
#define MT9P012_MAX_LINE_LENGTH_PCK 0xFFFE

#define RESET_REG_PLL_OFF_BP		5

#define MT9P012_LSC_SIZE 102

#define MT9P012_MIRROR_MODE_FLAG 0x1

/**
 * struct mt9p012_reg - mt9p012 register format
 * @length: length of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for MT9P012 register initialization values
 */
struct mt9p012_reg {
	u16 length;
	u16 reg;
	u32 val;
};

enum mt9p012_image_size {
	MT9P012_BIN4XSCALE = 0,
	MT9P012_BIN4X,
	MT9P012_BIN2X,
	MT9P012_FIVE_MP
};

#define MT9P012_NUM_IMAGE_SIZES		5
#define MT9P012_NUM_PIXEL_FORMATS	1
#define MT9P012_NUM_FPS			2	/* 2 ranges */

/**
 * struct capture_size - image capture size information
 * @width: image width in pixels
 * @height: image height in pixels
 */
struct mt9p012_capture_size {
	unsigned long width;
	unsigned long height;
};

/*
 * Array of image sizes supported by MT9P012.  These must be ordered from
 * smallest image size to largest.
 */
const static struct mt9p012_capture_size mt9p012_sizes[] = {
	{  216, 162 },	/* 4X BINNING+SCALING */
	{  648, 486 },	/* 4X BINNING */
	{ 1296, 972 },	/* 2X BINNING */
	{ 2592, 1944},	/* 5 MP */
};

enum mt9p012_frame_type {
	MT9P012_FRAME_5MP_10FPS = 0,
	MT9P012_FRAME_1296_30FPS,
	MT9P012_FRAME_648_30FPS,
	MT9P012_FRAME_216_30FPS,
};

enum mt9p012_orientation {
	MT9P012_NO_HORZ_FLIP_OR_VERT_FLIP = 0,
	MT9P012_HORZ_FLIP_ONLY,
	MT9P012_VERT_FLIP_ONLY,
	MT9P012_HORZ_FLIP_AND_VERT_FLIP
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

/* Private IOCTLs */

#define V4L2_CID_PRIVATE_S_PARAMS	(V4L2_CID_PRIVATE_BASE + 22)
#define V4L2_CID_PRIVATE_G_PARAMS	(V4L2_CID_PRIVATE_BASE + 23)

/* Debug functions */
static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug Enabled (0-1)");

static inline u32 mt9p012_ver(u16 model, u16 rev)
{
	return ((model & 0xffff) << 16) | (rev & 0xffff);
}


/* list of image formats supported by mt9p012 sensor */
const static struct v4l2_fmtdesc mt9p012_formats[] = {
	{
		.description	= "Bayer10 (GrR/BGb)",
		.pixelformat	= V4L2_PIX_FMT_SGRBG10,
	}
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(mt9p012_formats)

/* Enters soft standby, all settings are maintained */
const static struct mt9p012_reg stream_off_list[] = {
	{.length = MT9P012_8BIT, .reg = REG_MODE_SELECT, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0}
};

/* Exits soft standby */
const static struct mt9p012_reg stream_on_list[] = {
	{.length = MT9P012_8BIT, .reg = REG_MODE_SELECT, .val = 0x01},
	/* Sensor datasheet says we need 1 ms to allow PLL lock */
	{.length = MT9P012_TOK_DELAY, .reg = 0x00, .val = 1},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0}
};

/* Structure which will set the exposure time */
static struct mt9p012_reg set_exposure_time[] = {
	/* less than frame_lines-1 */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x01},
	{.length = MT9P012_16BIT, .reg = REG_COARSE_INT_TIME, .val = 500},
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0},
};

/*
 * Common MT9P012 register initialization for all image sizes, pixel formats,
 * and frame rates
 */
const static struct mt9p012_reg mt9p012_common_pre[] = {
	{MT9P012_16BIT, REG_RESET_REGISTER, 0x10C8},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_RESERVED_MFR_3064, 0x0805},
	{MT9P012_TOK_TERM, 0, 0}
};

const static struct mt9p012_reg mt9p012_common_post[] = {
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update all at once */
	{MT9P012_TOK_TERM, 0, 0}
};

const static struct mt9p012_reg mt9p012_common_rev1[] = {
	/* Recommended values for image quality, sensor Rev 1 */
	{MT9P012_16BIT, 0x3088, 0x6FFB},
	{MT9P012_16BIT, 0x308E, 0x2020},
	{MT9P012_16BIT, 0x309E, 0x4400},
	{MT9P012_16BIT, 0x30D4, 0x9080},
	{MT9P012_16BIT, 0x3126, 0x00FF},
	{MT9P012_16BIT, 0x3154, 0x1482},
	{MT9P012_16BIT, 0x3158, 0x97C7},
	{MT9P012_16BIT, 0x315A, 0x97C6},
	{MT9P012_16BIT, 0x3162, 0x074C},
	{MT9P012_16BIT, 0x3164, 0x0756},
	{MT9P012_16BIT, 0x3166, 0x0760},
	{MT9P012_16BIT, 0x316E, 0x8488},
	{MT9P012_16BIT, 0x3172, 0x0003},
	{MT9P012_16BIT, 0x30EA, 0x3F06},
	{MT9P012_TOK_TERM, 0, 0}
};

const static struct mt9p012_reg mt9p012_common_rev7[] = {
	/* Recommended values for image quality, sensor Rev 7-8 */
	{MT9P012_16BIT, 0x3088, 0x6FF6},
	{MT9P012_16BIT, 0x308E, 0xE060},
	{MT9P012_16BIT, 0x3092, 0x0A52},
	{MT9P012_16BIT, 0x3094, 0x4656},
	{MT9P012_16BIT, 0x3096, 0x5652},
	{MT9P012_16BIT, 0x309A, 0xA500},
	{MT9P012_16BIT, 0x30B0, 0x0001},
	{MT9P012_16BIT, 0x30CA, 0x8006},
	{MT9P012_16BIT, 0x312A, 0xDD02},
	{MT9P012_16BIT, 0x312C, 0x00E4},
	{MT9P012_16BIT, 0x3154, 0x0282},
	{MT9P012_16BIT, 0x3156, 0x0381},
	{MT9P012_16BIT, 0x3162, 0x01F1},
	{MT9P012_16BIT, 0x3170, 0x299A},
	{MT9P012_TOK_TERM, 0, 0}
};

const static struct mt9p012_reg mt9p013_common[] = {
	/* Recommended values for image quality */
	{MT9P012_16BIT, 0x3086, 0x2468},
	{MT9P012_16BIT, 0x3088, 0x6FFF},
	{MT9P012_16BIT, 0x309E, 0x5D00},
	{MT9P012_16BIT, 0x316C, 0xA4F0},
	{MT9P012_16BIT, 0x3094, 0x5056},
	{MT9P012_TOK_TERM, 0, 0}
};

/**
 * struct struct clk_settings - struct for storage of sensor
 * clock settings
 * @pre_pll_div: pre pll divider
 * @pll_mult: pll multiplier
 * @vt_pix_clk_div: video pixel clock divider
 * @vt_sys_clk_div: video system clock divider
 * @op_pix_clk_div: output pixel clock divider
 * @op_sys_clk_div: output system clock divider
 * @min_pll: minimum pll multiplier
 * @max_pll: maximum pll multiplier
 */
struct mt9p012_clk_settings {
	u16	pre_pll_div;
	u16	pll_mult;
	u16	vt_pix_clk_div;
	u16	vt_sys_clk_div;
	u16	op_pix_clk_div;
	u16	op_sys_clk_div;
	u16	min_pll;
	u16	max_pll;
};

/**
 * struct struct frame_settings - struct for storage of sensor
 * frame settings
 * @frame_len_lines: number of lines in frame
 * @line_len_pck_min: minimum number of pixels in line
 * @x_addr_start: horizontal start address
 * @x_addr_end: horizontal end address
 * @y_addr_start: vertical start address
 * @y_addr_end: vertical end address
 * @x_output_size: horizontal output size
 * @y_output_size: vertical output size
 * @x_odd_inc: X odd increment value
 * @y_odd_inc: Y odd increment value
 * @x_bin: X binning enable
 * @xy_bin: XY binning enable
 * @scale_m: scale factor = 16/M
 * @scale_mode: image resolution scaler mode
 */
struct mt9p012_frame_settings {
	u16	frame_len_lines;
	u16	line_len_pck_min;
	u16	line_len_pck;
	u16	x_addr_start;
	u16	x_addr_end;
	u16	y_addr_start;
	u16	y_addr_end;
	u16	x_output_size;
	u16	y_output_size;
	u16	x_odd_inc;
	u16	y_odd_inc;
	u16	x_bin;
	u16	xy_bin;
	u16	x_bin_summing;
	u16	scale_m;
	u16	scale_mode;
};

/**
 * struct struct exposure_settings - struct for storage of sensor
 * initial exposure settings
 * @coarse_int_tm: coarse integration time in lines
 * @fine_int_tm: fine integration time in pixels
 * @fine_correction: fine correction time in pixels
 * @analog_gain: global analog gain (smia)
 */
struct mt9p012_exposure_settings {
	u16	coarse_int_tm;
	u16	fine_int_tm;
	u16	fine_correction;
	u16	analog_gain;
};

/**
 * struct struct mt9p012_sensor_settings - struct for storage of
 * sensor settings.
 */
struct mt9p012_sensor_settings {
	struct mt9p012_clk_settings clk;
	struct mt9p012_frame_settings frame;
	struct mt9p012_exposure_settings exposure;
};

/**
 * struct mt9p012_sensor_id
 */
struct mt9p012_sensor_id {
	u16 revision;
	u16 model;
	u16 mfr;
};

/**
 * struct mt9p012_sensor_params
 */
struct mt9p012_sensor_params {
	u32 line_time;	/* usec, q8 */
	u16 gain_frame_delay;
	u16 exp_time_frame_delay;
	u16 frame_length_lines;
	u16 line_length_clocks;
	u16 x_output_size;
	u16 y_output_size;
	u16 binning_sensitivity;
};

/**
 * struct mt9p012_flash_params
 */
struct mt9p012_flash_params {
	u16 flash_time;
	u8 flash_type;
	u8 shutter_type;
};

/**
 * struct mt9p012_lsc_reg
 */
struct mt9p012_lsc_reg {
	u16 addr;
	u16 data;
};

/**
 * struct mt9p012_lsc_params
 */
struct mt9p012_lsc_params {
	u16 enable_lens_correction;
	u16 num_elements;
	struct mt9p012_lsc_reg lsc_tbl[MT9P012_LSC_SIZE];
};

/**
 * struct mt9p012_min_readout_params
 */
struct mt9p012_min_readout_params {
	u8   enable_min_size;
	u16 requested_width;
	u16 requested_height;
};

static struct mt9p012_sensor_settings sensor_settings[] = {

	/* FRAME_5MP */
	{
		.clk = {
			.pre_pll_div = 10,
			.pll_mult = 93,
			.vt_pix_clk_div = 4,
			.vt_sys_clk_div = 1,
			.op_pix_clk_div = 8,
			.op_sys_clk_div = 1,
		},
		.frame = {
			.frame_len_lines = 2056,
			.line_len_pck_min = 5372,
			.x_addr_start = 8,
			.x_addr_end = 2599,
			.y_addr_start = 8,
			.y_addr_end = 1951,
			.x_output_size = 2592,
			.y_output_size = 1944,
			.x_odd_inc = 1,
			.y_odd_inc = 1,
			.x_bin = 0,
			.xy_bin = 0,
			.x_bin_summing = 0,
			.scale_m = 0,
			.scale_mode = 0,
		},
		.exposure = {
			.coarse_int_tm = 1700,
			.fine_int_tm = 882,
			.fine_correction = 156,
			.analog_gain = 0x10C0
		}
	},

	/* FRAME_1296_30FPS */
	{
		.clk = {
			.pre_pll_div = 12,
			.pll_mult = 134,
			.vt_pix_clk_div = 5,
			.vt_sys_clk_div = 1,
			.op_pix_clk_div = 10,
			.op_sys_clk_div = 1,
		},
		.frame = {
			.frame_len_lines = 1063,
			.line_len_pck_min = 3360,
			.x_addr_start = 8,
			.x_addr_end = 2597,
			.y_addr_start = 8,
			.y_addr_end = 1949,
			.x_output_size = 1296,
			.y_output_size = 972,
			.x_odd_inc = 3,
			.y_odd_inc = 3,
			.x_bin = 0,
			.xy_bin = 1,
			.x_bin_summing = VIDEO_BIN_SUMMING,
			.scale_m = 0,
			.scale_mode = 0,
		},
		.exposure = {
			.coarse_int_tm = 1000,
			.fine_int_tm = 1794,
			.fine_correction = 348,
			.analog_gain = 0x10C0
		}
	},

	/* FRAME_648_30FPS */
	{
		.clk = {
			.pre_pll_div = 12,
			.pll_mult = 187,
			.vt_pix_clk_div = 5,
			.vt_sys_clk_div = 2,
			.op_pix_clk_div = 10,
			.op_sys_clk_div = 2,
		},
		.frame = {
			.frame_len_lines = 779,
			.line_len_pck_min = 3200,
			.x_addr_start = 0,
			.x_addr_end = 2601,
			.y_addr_start = 4,
			.y_addr_end = 1949,
			.x_output_size = 648,
			.y_output_size = 486,
			.x_odd_inc = 7,
			.y_odd_inc = 7,
			.x_bin = 0,
			.xy_bin = 1,
			.x_bin_summing = VIDEO_BIN_SUMMING,
			.scale_m = 0,
			.scale_mode = 0,
		},
		.exposure = {
			.coarse_int_tm = 750,
			.fine_int_tm = 1794,
			.fine_correction = 348,
			.analog_gain = 0x10C0
		}
	},

	/* FRAME_216_30FPS */
	{
		.clk = {
			.pre_pll_div = 6,
			.pll_mult = 94,
			.vt_pix_clk_div = 5,
			.vt_sys_clk_div = 2,
			.op_pix_clk_div = 10,
			.op_sys_clk_div = 2,
		},
		.frame = {
			.frame_len_lines = 783,
			.line_len_pck_min = 3200,
			.x_addr_start = 0,
			.x_addr_end = 2601,
			.y_addr_start = 4,
			.y_addr_end = 1949,
			.x_output_size = 216,
			.y_output_size = 162,
			.x_odd_inc = 7,
			.y_odd_inc = 7,
			.x_bin = 0,
			.xy_bin = 1,
			.x_bin_summing = VIDEO_BIN_SUMMING,
			.scale_m = 48,
			.scale_mode = 2,
		},
		.exposure = {
			.coarse_int_tm = 770,
			.fine_int_tm = 1794,
			.fine_correction = 348,
			.analog_gain = 0x11FD
		}
	}
};

/* Settings used if mirror mode is needed */
static struct mt9p012_sensor_settings mirror_sensor_settings[] = {

	/* FRAME_5MP */
	{
		.clk = {
			.pre_pll_div = 9,
			.pll_mult = 134,
			.vt_pix_clk_div = 7,
			.vt_sys_clk_div = 1,
			.op_pix_clk_div = 10,
			.op_sys_clk_div = 1,
		},
		.frame = {
			.frame_len_lines = 2056,
			.line_len_pck_min = 4915,
			.x_addr_start = 8,
			.x_addr_end = 2599,
			.y_addr_start = 8,
			.y_addr_end = 1951,
			.x_output_size = 2592,
			.y_output_size = 1944,
			.x_odd_inc = 1,
			.y_odd_inc = 1,
			.x_bin = 0,
			.xy_bin = 0,
			.x_bin_summing = 0,
			.scale_m = 0,
			.scale_mode = 0,
		},
		.exposure = {
			.coarse_int_tm = 1700,
			.fine_int_tm = 882,
			.fine_correction = 156,
			.analog_gain = 0x10C0
		}
	},

	/* FRAME_1296_30FPS */
	{
		.clk = {
			.pre_pll_div = 12,
			.pll_mult = 134,
			.vt_pix_clk_div = 6,
			.vt_sys_clk_div = 1,
			.op_pix_clk_div = 10,
			.op_sys_clk_div = 1,
		},
		.frame = {
			.frame_len_lines = 1063,
			.line_len_pck_min = 3430,
			.x_addr_start = 8,
			.x_addr_end = 2597,
			.y_addr_start = 8,
			.y_addr_end = 1949,
			.x_output_size = 1296,
			.y_output_size = 972,
			.x_odd_inc = 3,
			.y_odd_inc = 3,
			.x_bin = 0,
			.xy_bin = 1,
			.x_bin_summing = VIDEO_BIN_SUMMING,
			.scale_m = 0,
			.scale_mode = 0,
		},
		.exposure = {
			.coarse_int_tm = 1000,
			.fine_int_tm = 1794,
			.fine_correction = 348,
			.analog_gain = 0x10C0
		}
	}
};

/**
 * struct vcontrol - Video controls
 * @v4l2_queryctrl: V4L2 VIDIOC_QUERYCTRL ioctl structure
 * @current_value: current value of this control
 */
struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
};

static struct vcontrol video_control[] = {
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = MT9P013_DEF_MIN_EXPOSURE,
			.maximum = MT9P013_DEF_MAX_EXPOSURE,
			.step = MT9P013_EXPOSURE_STEP,
			.default_value = MT9P013_DEF_EXPOSURE,
		},
		.current_value = MT9P013_DEF_EXPOSURE,
	},
	{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Gain",
			.minimum = MT9P013_MIN_LINEAR_GAIN,
			.maximum = MT9P013_MAX_LINEAR_GAIN,
			.step = MT9P013_GAIN_STEP,
			.default_value = MT9P013_DEF_LINEAR_GAIN,
		},
		.current_value = MT9P013_DEF_LINEAR_GAIN,
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
		.id = ORIENTATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Orientation",
		.minimum = MT9P012_NO_HORZ_FLIP_OR_VERT_FLIP,
		.maximum = MT9P012_HORZ_FLIP_AND_VERT_FLIP,
		.step = 0,
		.default_value = MT9P012_NO_HORZ_FLIP_OR_VERT_FLIP,
		.current_value = MT9P012_NO_HORZ_FLIP_OR_VERT_FLIP,
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
		.id = CALIBRATION_ADJ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Calibration Adjust",
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


/**
 * struct mt9p012_sensor - main structure for storage of sensor information
 * @dev:
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @scaler:
 * @ver: mt9p012 chip version
 * @fps: frames per second value
 */
struct mt9p012_sensor {
	struct device *dev;
	struct mt9p012_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	int scaler;
	u32 ver;
	int fps;
	int detected;
	bool power_on;

	u32 x_clk;
	u32 pll_clk;
	u32 vt_pix_clk;
	u32 op_pix_clk;

	int current_iframe;

	u32 min_exposure_time;
	u32 fps_max_exposure_time;
	u32 abs_max_exposure_time;

	int min_linear_gain;
	int max_linear_gain;

	struct mt9p012_sensor_id sensor_id;
	struct mt9p012_flash_params flash;
	struct mt9p012_sensor_params sensor_params;
	struct mt9p012_lsc_params lsc;
	struct vcontrol *video_control;
	int n_video_control;
	struct mt9p012_min_readout_params min_readout;
};

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int find_vctrl(struct mt9p012_sensor *sensor, int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = sensor->n_video_control - 1; i >= 0; i--)
		if (sensor->video_control[i].qc.id == id)
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


/**
 * mt9p012_read_reg - Read a value from a register in an mt9p012 sensor device
 * @client: i2c driver client structure
 * @data_length: length of data to be read
 * @reg: register address / offset
 * @val: stores the value that gets read
 *
 * Read a value from a register in an mt9p012 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int mt9p012_read_reg(struct i2c_client *client, u16 data_length,
			    u16 reg, u32 *val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[4];
	printk("Enter to %s\n",__func__);
	if (!client->adapter)
		return -ENODEV;
	if (data_length != MT9P012_8BIT && data_length != MT9P012_16BIT
					&& data_length != MT9P012_32BIT)
		return -EINVAL;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);;
	data[1] = (u8) (reg & 0xff);
	err = i2c_transfer(client->adapter, msg, 1);
	if (err < 0) {
		msleep(3);
		err = i2c_transfer(client->adapter, msg, 1);
	}

	if (err >= 0) {
		msg->len = data_length;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = 0;
		/* high byte comes first */
		if (data_length == MT9P012_8BIT)
			*val = data[0];
		else if (data_length == MT9P012_16BIT)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
				(data[1] << 16) + (data[0] << 24);
		err = 0;
	}
	v4l_dbg(1, debug, client, "read from offset 0x%x error %d", reg, err);
	return err;
}
/**
 * mt9p012_write_reg - Write a value to a register in an mt9p012 sensor device
 * @client: i2c driver client structure
 * @data_length: length of data to be read
 * @reg: register address / offset
 * @val: value to be written to specified register
 *
 * Write a value to a register in an mt9p012 sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int mt9p012_write_reg(struct i2c_client *client, u16 data_length,
			     u16 reg, u32 val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[6];
	int retry = 0;
	struct mt9p012_sensor *sensor;
	printk("Enter to %s\n",__func__);
	if (!client->adapter)
		return -ENODEV;

	sensor = i2c_get_clientdata(client);

	if (data_length != MT9P012_8BIT && data_length != MT9P012_16BIT
					&& data_length != MT9P012_32BIT)
		return -EINVAL;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2 + data_length;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);;
	data[1] = (u8) (reg & 0xff);

	if (data_length == MT9P012_8BIT)
		data[2] = (u8) (val & 0xff);
	else if (data_length == MT9P012_16BIT) {
		data[2] = (u8) (val >> 8);
		data[3] = (u8) (val & 0xff);
	} else {
		data[2] = (u8) (val >> 24);
		data[3] = (u8) (val >> 16);
		data[4] = (u8) (val >> 8);
		data[5] = (u8) (val & 0xff);
	}

	do {
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			v4l_dbg(1, debug, client,
				"wrote 0x%x to offset 0x%x error %d",
				val, reg, err);
			return 0;
		}
		retry++;
		msleep(3);
	} while (retry <= MT9P012_I2C_RETRY_COUNT);

	return err;
}

/**
 * mt9p012_write_regs - Initializes a list of MT9P012 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * Initializes a list of MT9P012 registers. The list of registers is
 * terminated by MT9P012_TOK_TERM.
 */
static int mt9p012_write_regs(struct i2c_client *client,
			      const struct mt9p012_reg reglist[])
{
	int err;
	const struct mt9p012_reg *next = reglist;
	printk("Enter to %s\n",__func__);
	for (; next->length != MT9P012_TOK_TERM; next++) {
		if (next->length == MT9P012_TOK_DELAY) {
			mdelay(next->val);
			continue;
		}

		err = mt9p012_write_reg(client, next->length,
						next->reg, next->val);
		if (err)
			return err;
	}
	return 0;
}

/**
 * overwrite_with_mirror_settings - Overwrite mirror mode settings
 *
 * Overwrites a list of MT9P012 sensor settings.
 */
static void overwrite_with_mirror_settings(void)
{
	/* overwrite 5M settings */
	memcpy(&sensor_settings[MT9P012_FRAME_5MP_10FPS],
		&mirror_sensor_settings[MT9P012_FRAME_5MP_10FPS],
		sizeof(struct mt9p012_sensor_settings));

	/* overwrite liveview settings */
	memcpy(&sensor_settings[MT9P012_FRAME_1296_30FPS],
		&mirror_sensor_settings[MT9P012_FRAME_1296_30FPS],
		sizeof(struct mt9p012_sensor_settings));
}

/**
 * mt9p012_calc_size - Find the best match for a requested image capture size
 * @width: requested image width in pixels
 * @height: requested image height in pixels
 *
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum mt9p012_image_size mt9p012_calc_size(unsigned int width,
						 unsigned int height)
{
	enum mt9p012_image_size isize;
	unsigned long pixels = width * height;
	printk("Enter to %s\n",__func__);
	for (isize = MT9P012_BIN4XSCALE; isize <= MT9P012_FIVE_MP; isize++) {
		if (mt9p012_sizes[isize].height *
				mt9p012_sizes[isize].width >= pixels) {
			return isize;
		}
	}

	return MT9P012_FIVE_MP;
}

/**
 * mt9p012_find_isize - Find the best match for a requested image capture size
 * @width: requested image width in pixels
 * @height: requested image height in pixels
 *
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum mt9p012_image_size mt9p012_find_isize(unsigned int width)
{
	enum mt9p012_image_size isize;
	printk("Enter to %s\n",__func__);
	for (isize = MT9P012_BIN4XSCALE; isize <= MT9P012_FIVE_MP; isize++) {
		if (mt9p012_sizes[isize].width >= width)
			break;
	}

	return isize;
}

static enum
mt9p012_frame_type mt9p012_find_iframe(enum mt9p012_image_size isize)
{
	enum mt9p012_frame_type iframe = 0;
	printk("Enter to %s\n",__func__);
	if (isize == MT9P012_BIN4XSCALE)
		iframe = MT9P012_FRAME_216_30FPS;

	else if (isize == MT9P012_BIN4X)
		iframe = MT9P012_FRAME_648_30FPS;

	else if (isize == MT9P012_BIN2X)
		iframe = MT9P012_FRAME_1296_30FPS;

	else
		iframe = MT9P012_FRAME_5MP_10FPS;

	return iframe;
}

/**
 * mt9p012_set_exposure_time - sets exposure time per input value
 * @exp_time: exposure time to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 *
 * If the requested exposure time is within the allowed limits, the HW
 * is configured to use the new exposure time, and the video_controls
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
static int mt9p012_set_exposure_time(u32 exp_time, struct v4l2_int_device *s,
				    struct vcontrol *lvc)
{
	u16 scale_factor;
	u32 coarse_int_time = 0;
	int err = 0;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	struct mt9p012_sensor_settings *ss =
		&sensor_settings[sensor->current_iframe];
	printk("Enter to %s\n",__func__);
	if (exp_time < sensor->min_exposure_time) {
		dev_err(&client->dev, "Exposure time %d us too low.\n",
			exp_time);
		dev_err(&client->dev, "Min time %d us\n",
			sensor->min_exposure_time);
		exp_time = sensor->min_exposure_time;
	} else if (exp_time > sensor->abs_max_exposure_time) {
		dev_err(&client->dev, "Exposure time %d too high.\n",
			exp_time);
		dev_err(&client->dev, "Abs Max time %d us\n",
			sensor->abs_max_exposure_time);
		exp_time = sensor->abs_max_exposure_time;
	}

	if (!sensor->power_on)
		goto end;

	if (exp_time < 15000)
		scale_factor = 1;
	else if (exp_time < 150000)
		scale_factor = 10;
	else
		scale_factor = 100;

	coarse_int_time = ((((exp_time / scale_factor) *
		(sensor->vt_pix_clk / 1000)) / 1000) -
		(ss->exposure.fine_int_tm / scale_factor)) /
		(ss->frame.line_len_pck / scale_factor);

	if (coarse_int_time != ss->exposure.coarse_int_tm) {
		set_exposure_time[MT9P012_COARSE_INT_TIME_INDEX].val =
			coarse_int_time;
		err = mt9p012_write_regs(client, set_exposure_time);

		ss->exposure.coarse_int_tm = coarse_int_time;

		dev_info(&client->dev, "set_exp: expT=%dus " \
			"coarse_int_time=%d\n", \
			exp_time, coarse_int_time);
	}

end:
	if (err)
		dev_err(&client->dev, "Error setting exposure time %d\n",
			err);
	else
		lvc->current_value = exp_time;

	return err;
}

/**
 * mt9p012_set_gain - sets sensor analog gain per input value
 * @gain: analog linear gain Q8 value to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 analog gain entry in video_controls array
 *
 * If the requested analog gain is within the allowed limits, the HW
 * is configured to use the new gain value, and the video_controls
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
static int mt9p012_set_gain(u16 gain, struct v4l2_int_device *s,
			    struct vcontrol *lvc)
{
	u16 reg_gain = 0, digital_gain = 1;
	u16 shift_bits, gain_stage_2x, linear_gain_q5, digital_gain_bp;
	u16 analog_gain_code;
	u16 min_gain;
	int err = 0;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	struct mt9p012_sensor_settings *ss =
		&sensor_settings[sensor->current_iframe];
	printk("Enter to %s\n",__func__);
	if (gain < sensor->min_linear_gain) {
		dev_err(&client->dev, "Gain=%d out of legal range.\n",
			gain);
		dev_err(&client->dev, "Gain must be greater than %d \n",
			sensor->min_linear_gain);
		gain = sensor->min_linear_gain;
	}

	if (gain > sensor->max_linear_gain) {
		dev_err(&client->dev, "Gain=%d out of legal range.\n",
			gain);
		dev_err(&client->dev, "Gain must be less than %d \n",
			sensor->max_linear_gain);
		gain = sensor->max_linear_gain;
	}

	if (!sensor->power_on)
		goto end;

	/* Convert gain from linear to register value */
	linear_gain_q5 = (gain + (1<<2)) >> 3;

	if (sensor->ver >= mt9p012_ver(12, 7))
		digital_gain_bp = 12;
	else
		digital_gain_bp = 9;

	if (linear_gain_q5 >= 16*32) {
		gain_stage_2x = 0x180;	/* AG2X_1, AG2X_2 */
		shift_bits = 0x2;
		digital_gain = 2;
	} else if (linear_gain_q5 >= 8*32) {
		gain_stage_2x = 0x180;	/* AG2X_1, AG2X_2 */
		shift_bits = 0x2;
		digital_gain = 1;
	} else if (linear_gain_q5 >= 2*32) {
		gain_stage_2x = 0x080;	/* AG2X_1 */
		shift_bits = 0x1;
		digital_gain = 1;
	} else {
		gain_stage_2x = 0x000;
		shift_bits = 0x0;
		digital_gain = 1;
	}

	analog_gain_code = gain_stage_2x | (linear_gain_q5 >> shift_bits);

	if (sensor->ver >= mt9p012_ver(13, 0))
		min_gain = MT9P013_MIN_ANALOG_GAIN;
	else
		min_gain = MT9P012_MIN_ANALOG_GAIN;

	if (analog_gain_code < min_gain)
		analog_gain_code = min_gain;

	if (analog_gain_code > MT9P012_MAX_ANALOG_GAIN)
		analog_gain_code = MT9P012_MAX_ANALOG_GAIN;

	reg_gain = (digital_gain << digital_gain_bp) | analog_gain_code;

	if (reg_gain !=	ss->exposure.analog_gain) {
		err = mt9p012_write_reg(client, MT9P012_16BIT,
			REG_MANUF_GAIN_GLOBAL, reg_gain);
		ss->exposure.analog_gain = reg_gain;

		dev_info(&client->dev, "set_gain: lineargain=%d " \
			"reg_gain=0x%x sensor_ver=0x%x\n", \
			gain, reg_gain, sensor->ver);
	}

end:
	if (err) {
		dev_err(&client->dev, "Error setting gain.%d", err);
		return err;
	} else
		lvc->current_value = gain;

	return err;
}

/**
 * mt9p012_set_flash_next_frame - configures flash on for the next frame
 * @flash_params: flash type and time
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int mt9p012_set_flash_next_frame(struct mt9p012_flash_params *flash_params,
				 struct v4l2_int_device *s,
				 struct private_vcontrol *pvc)
{
	int err = 0;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *c = to_i2c_client(sensor->dev);
	u16 data;
	printk("Enter to %s\n",__func__);
	dev_info(&c->dev, "set_flash_next_frame: time=%dusec, " \
		"flash_type=%d, shutter_type=%d, power=%d\n",
		flash_params->flash_time, flash_params->flash_type,
		flash_params->shutter_type, sensor->power_on);

	if (sensor->power_on) {
		if (flash_params->flash_time != 0)
			data = 0x0100;
		else
			data = 0x0000;

		err = mt9p012_write_reg(c, MT9P012_16BIT, REG_FLASH, data);

		/* Register auto-resets */
		flash_params->flash_time = 0;
	}

	if (err)
		dev_err(&c->dev, "Error setting flash next frame.%d", err);
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
static int mt9p012_set_orientation(enum mt9p012_orientation val,
			struct v4l2_int_device *s, struct private_vcontrol *pvc)
{
	int err = 0;
	u8 orient;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	printk("Enter to %s\n",__func__);
	if (!sensor->power_on)
		goto end;

	switch (val) {
	case MT9P012_NO_HORZ_FLIP_OR_VERT_FLIP:
		orient = 0x0;
		break;
	case MT9P012_HORZ_FLIP_ONLY:
		orient = 0x1;
		break;
	case MT9P012_VERT_FLIP_ONLY:
		orient = 0x2;
		break;
	case MT9P012_HORZ_FLIP_AND_VERT_FLIP:
		orient = 0x3;
		break;
	default:
		orient = 0x0;
		break;
	}

	err = mt9p012_write_reg(client, MT9P012_8BIT,
				REG_IMAGE_ORIENTATION, orient);

end:
	if (err) {
		dev_err(&client->dev, "Error setting orientation.%d", err);
		return err;
	} else
		pvc->current_value = (u32)val;

	return err;
}

static int mt9p012_init_exposure_params(struct v4l2_int_device *s,
					enum mt9p012_frame_type iframe)
{
	int i = 0;
	struct mt9p012_sensor *sensor = s->priv;
	printk("Enter to %s\n",__func__);
	/* flag current exp_time & gain values as invalid */
	sensor_settings[iframe].exposure.analog_gain = 0;
	sensor_settings[iframe].exposure.coarse_int_tm = 0;

	/* init min/max gain params */
	if (sensor->ver >= mt9p012_ver(13, 0))
		sensor->min_linear_gain = MT9P013_MIN_LINEAR_GAIN;
	else
		sensor->min_linear_gain = MT9P012_MIN_LINEAR_GAIN;

	sensor->max_linear_gain = MT9P012_MAX_LINEAR_GAIN;

	i = find_vctrl(sensor, V4L2_CID_GAIN);
	if (i >= 0) {
		sensor->video_control[i].qc.minimum =
			sensor->min_linear_gain;
	}

	return 0;
}

static int mt9p012_calibration_adjust(int val, struct v4l2_int_device *s,
			struct private_vcontrol *pvc)
{
	int err = 0;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	struct vcontrol *lvc_gain;
	printk("Enter to %s\n",__func__);
	if (val != 1)
		return 0;

	if (sensor->power_on) {
		/* adj 0x308E to match calibration setting */
		err = mt9p012_write_reg(client, MT9P012_16BIT,
					REG_RESERVED_MFR_308E, 0xE060);
		dev_info(&client->dev, "mt9p013_cal_adj:setting " \
			"0x308E=0xE060\n");
	}

	if (err) {
		dev_err(&client->dev, "Error setting cal adj: %d", err);
		return err;
	} else {
		pvc->current_value = val;
	}

	/* adjust minimum gain */
	err = find_vctrl(sensor, V4L2_CID_GAIN);
	if (err >= 0) {
		sensor->min_linear_gain = MT9P012_MIN_LINEAR_GAIN_CAL_ADJ;
		lvc_gain = &sensor->video_control[err];
		lvc_gain->qc.minimum = MT9P012_MIN_LINEAR_GAIN_CAL_ADJ;
		dev_info(&client->dev, "mt9p013:setting min gain=%d\n",
			MT9P012_MIN_LINEAR_GAIN_CAL_ADJ);
	}

	return 0;
}

/**
 * mt9p012_set_framerate - Sets framerate by adjusting frame_length_lines reg.
 * @s: pointer to standard V4L2 device structure
 * @fper: frame period numerator and denominator in seconds
 * @iframe: enum value corresponding to frame type (size & fps)
 *
 * The maximum exposure time is also updated since it is affected by the
 * frame rate.
 **/
static int mt9p012_set_framerate(struct v4l2_int_device *s,
			struct v4l2_fract *fper, enum mt9p012_frame_type iframe)
{
	int err = 0, i = 0;
	u32 line_len_pck, line_time_q8;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	struct mt9p012_sensor_settings *ss;
	struct vcontrol *lvc = NULL;
	printk("Enter to %s\n",__func__);
	ss = &sensor_settings[iframe];

	/* calc desired line_time (usec's (q8)) */
	line_time_q8 = (((u32)fper->numerator * 1000000 * 256 /
		fper->denominator)) / ss->frame.frame_len_lines;

	line_len_pck = (((line_time_q8 * (sensor->vt_pix_clk / 10000)) /
		100) + 128) / 256;

	/* Range check line_len_pck */
	if (line_len_pck > MT9P012_MAX_LINE_LENGTH_PCK)
		line_len_pck = MT9P012_MAX_LINE_LENGTH_PCK;
	else if (line_len_pck < ss->frame.line_len_pck_min)
		line_len_pck = ss->frame.line_len_pck_min;

	/* recalculate line_time with actual line_len_pck that would be used */
	line_time_q8 = (((u32)line_len_pck * 1000 * 256) /
			(sensor->vt_pix_clk / 1000)); /* usec's (q8) */

	mt9p012_write_reg(client, MT9P012_16BIT,
			  REG_LINE_LEN_PCK, line_len_pck);

	sensor->sensor_params.line_time = line_time_q8;
	ss->frame.line_len_pck = line_len_pck;

	/* Update min/max exposure times */
	sensor->min_exposure_time = (ss->exposure.fine_int_tm * 1000000 /
				     (sensor->vt_pix_clk)) + 1;
	sensor->abs_max_exposure_time = (line_time_q8 *
				     (MT9P012_MAX_FRAME_LENGTH_LINES - 1)) >> 8;
	sensor->fps_max_exposure_time = (line_time_q8 *
				     (ss->frame.frame_len_lines - 1)) >> 8;

	/* Update Exposure Time */
	i = find_vctrl(sensor, V4L2_CID_EXPOSURE);
	if (i >= 0) {
		lvc = &sensor->video_control[i];
		/* Update min/max for query control */
		lvc->qc.minimum = sensor->min_exposure_time;
		lvc->qc.maximum = sensor->fps_max_exposure_time;

	}

	v4l_info(client, "MT9P012 Set Framerate: iframe=%d, fper=%d/%d, " \
		"line_len_pck=%d, fps_max_expT=%dus, " \
		"abs_max_expT=%dus\n",
		iframe, fper->numerator, fper->denominator,
		ss->frame.line_len_pck,
		sensor->fps_max_exposure_time,
		sensor->abs_max_exposure_time);

	return err;
}

/**
 * mt9p012_calc_xclk - Calculate the required xclk frequency
 * @c: i2c client driver structure
 *
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate and return the required xclk frequency
 */
static unsigned long mt9p012_calc_xclk(struct i2c_client *c)
{
	struct mt9p012_sensor *sensor = i2c_get_clientdata(c);
	struct v4l2_fract *timeperframe = &sensor->timeperframe;
	printk("Enter to %s\n",__func__);
	if (timeperframe->numerator == 0 ||
	    timeperframe->denominator == 0) {
		/* supply a default nominal_timeperframe */
		timeperframe->numerator = 1;
		timeperframe->denominator = MT9P012_DEF_FPS;
	}

	sensor->fps = timeperframe->denominator / timeperframe->numerator;
	if (sensor->fps < MT9P012_MIN_FPS)
		sensor->fps = MT9P012_MIN_FPS;
	else if (sensor->fps > MT9P012_MAX_FPS)
		sensor->fps = MT9P012_MAX_FPS;

	timeperframe->numerator = 1;
	timeperframe->denominator = sensor->fps;

	return MT9P012_XCLK_NOM_4;
}

/*
 * Set Lens Correction
 */
static int mt9p012_set_lens_correction(struct mt9p012_lsc_params *lsc,
	struct v4l2_int_device *s)
{
	int err = 0, i;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *c = to_i2c_client(sensor->dev);
	printk("Enter to %s\n",__func__);
	if (sensor->power_on) {
		if (lsc->enable_lens_correction) {
			/* Lock VDD1 to MAX for 720p mode */
			//sensor->pdata->lock_cpufreq(CPU_CLK_LOCK);
			err |= mt9p012_write_reg(c, MT9P012_16BIT,
				REG_SC_ENABLE, 0x0000);

			if (lsc->num_elements <= MT9P012_LSC_SIZE) {
				for (i = 0; i < lsc->num_elements; ++i) {
					err |= mt9p012_write_reg(c,
						MT9P012_16BIT,
						lsc->lsc_tbl[i].addr,
						lsc->lsc_tbl[i].data);
				}

			} else
				err = -EIO;

			if (!err) {
				err |= mt9p012_write_reg(c, MT9P012_16BIT,
					REG_SC_ENABLE, 0x8000);
			}

			dev_info(&c->dev, "Lens correction enabled.\n");

		} else {  /* disable lens correction */
			err |= mt9p012_write_reg(c, MT9P012_16BIT,
				REG_SC_ENABLE, 0x0000);
		}
	}

	if (err) {
		dev_err(&c->dev, "MT9P012: Error setting LSC=%d, size=%d.\n",
			lsc->enable_lens_correction, lsc->num_elements);
	}

	return err;
}

static int mt9p012_param_handler(u32 ctrlval,
			struct v4l2_int_device *s)
{
	int err = 0;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *c = to_i2c_client(sensor->dev);
	struct mt9p012_sensor_settings *ss =
		&sensor_settings[sensor->current_iframe];
	struct mt9p012_flash_params flash_params;
	struct camera_params_control camctl;
	struct mt9p012_min_readout_params min_readout;
	struct private_vcontrol *pvc;
	struct mt9p012_sensor_regif {
		u16 len;
		u32 addr;
		u32 val;
	} sensor_reg;
	int i;
	printk("Enter to %s\n",__func__);
	if (!ctrlval)
		return -EINVAL;

	if (copy_from_user(&camctl, (void *)ctrlval,
			   sizeof(struct camera_params_control))) {
		dev_err(&c->dev, "mt9p012: Failed copy_from_user\n");
		return -EINVAL;
	}

	i = find_vctrl_private(camctl.op);
	if (i < 0)
		return -EINVAL;
	pvc = &video_control_private[i];

	if (camctl.xaction == IOCTL_RD) {
		switch (camctl.op) {
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
				dev_err(&c->dev, "mt9p012: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case ORIENTATION:
			camctl.data_in = pvc->current_value;
			if (copy_to_user((void *)ctrlval, &camctl,
					sizeof(struct camera_params_control))) {
				dev_err(&c->dev, "mt9p012: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case LENS_CORRECTION:
			if (sizeof(sensor->lsc) > camctl.data_in_size) {
				dev_err(&c->dev,
					"V4L2_CID_PRIVATE_S_PARAMS IOCTL " \
					"DATA SIZE ERROR: src=%d dst=%d\n",
					sizeof(sensor->lsc),
					camctl.data_in_size);
				err = -EINVAL;
				break;
			}

			if (copy_to_user((void *)camctl.data_in,
					 &(sensor->lsc),
					 sizeof(sensor->lsc))) {
				dev_err(&c->dev, "mt9p012: Copy_to_user err\n");
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
				dev_err(&c->dev, "mt9p012: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case SENSOR_REG_REQ:
			if (!sensor->power_on) {
				dev_err(&c->dev, "mt9p012: I2C Read Err: " \
					"Power Off\n");
				err = -EINVAL;
				break;
			}
			if (copy_from_user(&sensor_reg,
				(struct mt9p012_sensor_regif *)camctl.data_in,
				sizeof(struct mt9p012_sensor_regif)) == 0) {
				err =  mt9p012_read_reg(c, sensor_reg.len,
					sensor_reg.addr, &sensor_reg.val);
				dev_info(&c->dev, "SENSOR_REG_REQ IOCTL read-" \
					"%d: 0x%x=0x%x\n", sensor_reg.len,
					sensor_reg.addr, sensor_reg.val);
			}
			if (copy_to_user((void *)camctl.data_in, &(sensor_reg),
				sizeof(sensor_reg))) {
				dev_err(&c->dev, "mt9p012: Copy_to_user err\n");
				err = -EINVAL;
			}
			break;
		case SENSOR_PARAMS_REQ:
			sensor->sensor_params.frame_length_lines =
				ss->frame.frame_len_lines;
			sensor->sensor_params.line_length_clocks =
				ss->frame.line_len_pck;
			sensor->sensor_params.x_output_size =
				ss->frame.x_output_size;
			sensor->sensor_params.y_output_size =
				ss->frame.y_output_size;
			if (ss->frame.x_bin_summing == 0) {
				sensor->sensor_params.binning_sensitivity =
					(u16)(1.0 * 256);
			} else {
				sensor->sensor_params.binning_sensitivity =
					(u16)(2.0 * 256);
			}

			if (sizeof(sensor->sensor_params) >
					camctl.data_in_size) {
				dev_err(&c->dev,
					"V4L2_CID_PRIVATE_S_PARAMS IOCTL " \
					"DATA SIZE ERROR: src=%d dst=%d\n",
					sizeof(sensor->sensor_params),
					camctl.data_in_size);
				err = -EINVAL;
				break;
			}

			if (copy_to_user((void *)camctl.data_in,
					 &(sensor->sensor_params),
					 sizeof(sensor->sensor_params))) {
				dev_err(&c->dev, "mt9p012: Copy_to_user err\n");
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
			if (!sensor->power_on) {
				dev_err(&c->dev, "Reg Write Err: Power Off\n");
				err = -EINVAL;
				break;
			}
			if (copy_from_user(&sensor_reg,
				(struct mt9p012_sensor_regif *)camctl.data_out,
				sizeof(struct mt9p012_sensor_regif)) == 0) {

				dev_info(&c->dev, "SENSOR_REG_REQ IOCTL write-" \
					"%d: 0x%x=0x%x\n", sensor_reg.len,
					sensor_reg.addr, sensor_reg.val);

				/* mt9p012_write_reg only supports 1, 2, or
				   4-byte writes
				*/
				if (sensor_reg.len == 1 ||
				    sensor_reg.len == 2 ||
				    sensor_reg.len == 4) {
					err = mt9p012_write_reg(c,
						sensor_reg.len,
						sensor_reg.addr,
						sensor_reg.val);
				} else {
					dev_err(&c->dev, "Error: " \
						"SENSOR_REG_REQ IOCTL " \
						"length must = 1, 2, or 4\n");
				}
			}
			break;
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
				(struct mt9p012_flash_params *)camctl.data_out,
				sizeof(struct mt9p012_flash_params)) == 0) {
				err = mt9p012_set_flash_next_frame(
					&flash_params, s, pvc);
			}
			break;
		case ORIENTATION:
			err = mt9p012_set_orientation(camctl.data_out,
						      s, pvc);
			break;
		case LENS_CORRECTION:
			if (sizeof(sensor->lsc) < camctl.data_out_size) {
				dev_err(&c->dev,
					"V4L2_CID_PRIVATE_S_PARAMS IOCTL " \
					"DATA SIZE ERROR: src=%d dst=%d\n",
					camctl.data_out_size,
					sizeof(sensor->lsc));
				err = -EINVAL;
				break;
			}

			err = copy_from_user(&(sensor->lsc),
				(struct mt9p012_lsc_params *)camctl.data_out,
				sizeof(struct mt9p012_lsc_params));
			err = mt9p012_set_lens_correction(&(sensor->lsc), s);
			dev_info(&c->dev, "LENS CORRECTION IOCTL write-"
					"enable=%d, size=%d\n",
					sensor->lsc.enable_lens_correction,
					sensor->lsc.num_elements);
			break;
		case CALIBRATION_ADJ:
			err = mt9p012_calibration_adjust(camctl.data_out,
							 s, pvc);
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
			(struct mt9p012_min_readout_params *)camctl.data_out,
			sizeof(struct mt9p012_min_readout_params)) == 0) {
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

/**
 * mt9p012_configure_frame - Setup the frame, clock and exposure parmas in the
 * config_frame_list array.
 *
 * @c: i2c client driver structure
 *
 * The config_frame_list is a common list used by all frame sizes & frame
 * rates that is filled in by this routine.
 */
int mt9p012_configure_frame(struct v4l2_int_device *s,
			    enum mt9p012_frame_type iframe)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	u16 data;
	int err = 0;
	printk("Enter to %s\n",__func__);
	 /* hold */
	err |= mt9p012_write_reg(client, MT9P012_8BIT, REG_GROUPED_PAR_HOLD,
			0x01);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_VT_PIX_CLK_DIV,
			 sensor_settings[iframe].clk.vt_pix_clk_div);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_VT_SYS_CLK_DIV,
			 sensor_settings[iframe].clk.vt_sys_clk_div);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_PRE_PLL_CLK_DIV,
			 sensor_settings[iframe].clk.pre_pll_div);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_PLL_MULTIPLIER,
			 sensor_settings[iframe].clk.pll_mult);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_OP_PIX_CLK_DIV,
			 sensor_settings[iframe].clk.op_pix_clk_div);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_OP_SYS_CLK_DIV,
			 sensor_settings[iframe].clk.op_sys_clk_div);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_X_OUTPUT_SIZE,
			 sensor_settings[iframe].frame.x_output_size);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_Y_OUTPUT_SIZE,
			 sensor_settings[iframe].frame.y_output_size);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_X_ADDR_START,
			 sensor_settings[iframe].frame.x_addr_start);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_Y_ADDR_START,
			 sensor_settings[iframe].frame.y_addr_start);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_X_ADDR_END,
			 sensor_settings[iframe].frame.x_addr_end);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_Y_ADDR_END,
			 sensor_settings[iframe].frame.y_addr_end);

	if (sensor->ver < mt9p012_ver(12, 7)) {
		data = (sensor_settings[iframe].frame.x_bin_summing & 1) << 12;
		data |= (sensor_settings[iframe].frame.x_bin & 1) << 11;
		data |= (sensor_settings[iframe].frame.xy_bin & 1) << 10;
		data |= (sensor_settings[iframe].frame.x_odd_inc & 7) << 5;
		data |= (sensor_settings[iframe].frame.y_odd_inc & 7) << 2;
	} else {
		data = (sensor_settings[iframe].frame.x_bin_summing & 1) << 12;
		data |= (sensor_settings[iframe].frame.x_bin & 1) << 11;
		data |= (sensor_settings[iframe].frame.xy_bin & 1) << 10;
		data |= (sensor_settings[iframe].frame.x_odd_inc & 7) << 6;
		data |= (sensor_settings[iframe].frame.y_odd_inc & 0x3F);
	}
	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_READ_MODE, data);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_FINE_INT_TIME,
			 sensor_settings[iframe].exposure.fine_int_tm);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_FRAME_LEN_LINES,
			 sensor_settings[iframe].frame.frame_len_lines);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_LINE_LEN_PCK,
			 sensor_settings[iframe].frame.line_len_pck_min);

	sensor_settings[iframe].frame.line_len_pck =
		sensor_settings[iframe].frame.line_len_pck_min;

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_SCALE_M,
			 sensor_settings[iframe].frame.scale_m);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_SCALING_MODE,
			 sensor_settings[iframe].frame.scale_mode);

	err |= mt9p012_write_reg(client, MT9P012_16BIT, REG_FINE_CORRECTION,
			sensor_settings[iframe].exposure.fine_correction);
	/* update */
	err |= mt9p012_write_reg(client, MT9P012_8BIT, REG_GROUPED_PAR_HOLD,
			0x00);

	sensor->current_iframe = iframe;
	if (err)
		return -EIO;
	else
		return 0;
}

/**
 * mt9p012_update_clocks - calcs sensor clocks based on sensor settings.
 * @xclk: current sensor input clock (xclk)
 * @isize: image size enum
 */
int mt9p012_update_clocks(struct v4l2_int_device *s, u32 xclk,
			  enum mt9p012_frame_type iframe)
{
	struct mt9p012_sensor *sensor = s->priv;
	printk("Enter to %s\n",__func__);
	sensor->x_clk = xclk;

	sensor->pll_clk =
			(xclk / sensor_settings[iframe].clk.pre_pll_div) *
			sensor_settings[iframe].clk.pll_mult;

	sensor->vt_pix_clk = sensor->pll_clk /
			(sensor_settings[iframe].clk.vt_pix_clk_div *
			sensor_settings[iframe].clk.vt_sys_clk_div);

	sensor->op_pix_clk = sensor->pll_clk /
			(sensor_settings[iframe].clk.op_pix_clk_div *
			sensor_settings[iframe].clk.op_sys_clk_div);

	dev_info(sensor->dev, "MT9P012: xclk=%u, pll_clk=%u, "
		  "vt_pix_clk=%u, op_pix_clk=%u\n", xclk, sensor->pll_clk,
		  sensor->vt_pix_clk, sensor->op_pix_clk);

	return 0;
}

/**
 * mt9p012_configure - Configure the mt9p012 for the specified image mode
 * @s: pointer to standard V4L2 device structure
 *
 * Configure the mt9p012 for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the mt9p012.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int mt9p012_configure(struct v4l2_int_device *s)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &sensor->pix;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	enum mt9p012_image_size isize;
	struct vcontrol *lvc = NULL;
	struct private_vcontrol *pvc = NULL;
	int iframe;
	int err;
	u32 xclk;
	int i;
	printk("Enter to %s\n",__func__);
	isize = mt9p012_find_isize(pix->width);

	/* common register initialization */
	err = mt9p012_write_regs(client, mt9p012_common_pre);
	if (err)
		return err;

	if (sensor->ver >= mt9p012_ver(13, 0))
		err = mt9p012_write_regs(client,
					 mt9p013_common);
	else if (sensor->ver >= mt9p012_ver(12, 7))
		err = mt9p012_write_regs(client,
					 mt9p012_common_rev7);
	else
		err = mt9p012_write_regs(client,
					 mt9p012_common_rev1);

	if (err)
		return err;

	err = mt9p012_write_regs(client, mt9p012_common_post);
	if (err)
		return err;

	/* configure frame rate */
	xclk = mt9p012_calc_xclk(client);

	iframe = mt9p012_find_iframe(isize);

	mt9p012_init_exposure_params(s, iframe);
	err = mt9p012_configure_frame(s, iframe);
	if (err)
		return err;

	mt9p012_update_clocks(s, xclk, iframe);

	/* configure frame rate */
	err = mt9p012_set_framerate(s, &sensor->timeperframe, iframe);
	if (err)
		return err;

	/* Set initial exposure time */
	i = find_vctrl(sensor, V4L2_CID_EXPOSURE);
	if (i >= 0) {
		lvc = &sensor->video_control[i];
		mt9p012_set_exposure_time(lvc->current_value,
			sensor->v4l2_int_device, lvc);
	}

	/* Set initial gain */
	i = find_vctrl(sensor, V4L2_CID_GAIN);
	if (i >= 0) {
		lvc = &sensor->video_control[i];
		mt9p012_set_gain(lvc->current_value,
			sensor->v4l2_int_device, lvc);
	}

	/* Set initial flash mode */
	i = find_vctrl_private(FLASH_NEXT_FRAME);
	if (i >= 0) {
		pvc = &video_control_private[i];
		mt9p012_set_flash_next_frame(&(sensor->flash),
			sensor->v4l2_int_device, pvc);
	}

	/* Set initial orientation */
	i = find_vctrl_private(ORIENTATION);
	if (i >= 0) {
		pvc = &video_control_private[i];
		mt9p012_set_orientation(pvc->current_value,
			sensor->v4l2_int_device, pvc);
	}

	/* Set cal adj */
	i = find_vctrl_private(CALIBRATION_ADJ);
	if (i >= 0) {
		pvc = &video_control_private[i];
		mt9p012_calibration_adjust(pvc->current_value,
			sensor->v4l2_int_device, pvc);
	}

	/* Set LSC */
	i = find_vctrl_private(LENS_CORRECTION);
	if (i >= 0) {
		pvc = &video_control_private[i];
		mt9p012_set_lens_correction(&(sensor->lsc),
			sensor->v4l2_int_device);
	}

	/* configure streaming ON */
	err = mt9p012_write_regs(client, stream_on_list);
	mdelay(1);

	return err;
}

/**
 * mt9p012_detect - Detect if an mt9p012 is present, and if so which revision
 * @client: pointer to the i2c client driver structure
 *
 * Detect if an mt9p012 is present, and if so which revision.
 * A device is considered to be detected if the manufacturer ID (MIDH and MIDL)
 * and the product ID (PID) registers match the expected values.
 * Any value of the version ID (VER) register is accepted.
 * Here are the version numbers we know about:
 *	0x48 --> mt9p012 Revision 1 or mt9p012 Revision 2
 *	0x49 --> mt9p012 Revision 3
 * Returns a negative error number if no device is detected, or the
 * non-negative value of the version ID register if a device is detected.
 */
static int mt9p012_detect(struct i2c_client *client)
{
	u32 model_id, mfr_id, rev, reset_data, data;
	struct mt9p012_sensor *sensor = i2c_get_clientdata(client);
	struct mt9p012_sensor_id *sensor_id = &(sensor->sensor_id);

	if (!client)
		return -ENODEV;

	if (mt9p012_read_reg(client, MT9P012_16BIT, REG_MODEL_ID, &model_id))
		return -ENODEV;
	if (mt9p012_read_reg(client, MT9P012_8BIT, REG_MANUFACTURER_ID,
				&mfr_id))
		return -ENODEV;

	/* Get sensor rev (reg 0x0002 is not valid) */
	if (mt9p012_read_reg(client, MT9P012_16BIT, REG_RESET_REGISTER,
				&reset_data))
		return -ENODEV;

	if (mt9p012_write_reg(client, MT9P012_16BIT, REG_RESET_REGISTER,
				reset_data | (1 << RESET_REG_PLL_OFF_BP)))
		return -ENODEV;

	if (mt9p012_read_reg(client, MT9P012_16BIT, REG_SENSOR_REVISION,
				&data))
		return -ENODEV;

	rev = data & 0xf;
	if (mt9p012_write_reg(client, MT9P012_16BIT, REG_RESET_REGISTER,
				reset_data))
		return -ENODEV;

	sensor_id->model = model_id;
	sensor_id->mfr = mfr_id;
	sensor_id->revision = rev;

	dev_info(&client->dev, "model id detected 0x%x mfr 0x%x\n", model_id,
								mfr_id);
	if ((mfr_id != MT9P012_MFR_ID)) {
		dev_warn(&client->dev, "mfgr id mismatch 0x%x\n", mfr_id);

		return -ENODEV;
	}

	if (model_id == MT9P012_MOD_ID) {
		rev = mt9p012_ver(12, 0);
	} else if (model_id == MT9P012_MOD_ID_REV7) {
		rev = mt9p012_ver(12, 7);
	} else if (model_id == MT9P013_MOD_ID) {
		rev = mt9p012_ver(13, 0);
	} else {
		/* We didn't read the values we expected, so
		 * this must not be an MT9P012.
		 */
		dev_warn(&client->dev, "model id mismatch 0x%x mfr 0x%x\n",
			model_id, mfr_id);

		return -ENODEV;
	}
	return rev;

}

/**
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qc)
{
	struct mt9p012_sensor *sensor = s->priv;
	int i;
	printk("Enter to %s\n",__func__);
	i = find_vctrl(sensor, qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = sensor->video_control[i].qc;
	return 0;
}

/**
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	struct mt9p012_sensor *sensor = s->priv;
	/*struct mt9p012_sensor_settings *ss =
		&sensor_settings[sensor->current_iframe];  */
	struct vcontrol *lvc;
	int i;
	printk("Enter to %s\n",__func__);
	i = find_vctrl(sensor, vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &sensor->video_control[i];

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_GAIN:
		vc->value = lvc->current_value;
		break;
	}

	return 0;
}

/**
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct vcontrol *lvc;
	int err = -EINVAL;
	int i;
	printk("Enter to %s\n",__func__);
	i = find_vctrl(sensor, vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &sensor->video_control[i];

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		err = mt9p012_set_exposure_time(vc->value, s, lvc);
		break;
	case V4L2_CID_GAIN:
		err = mt9p012_set_gain(vc->value, s, lvc);
		break;
	case V4L2_CID_PRIVATE_S_PARAMS:
		err = mt9p012_param_handler(vc->value, s);
		break;
	case V4L2_CID_PRIVATE_G_PARAMS:
		/* Not yet supported. */
		break;
	}

	return err;
}

/**
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
	printk("Enter to %s\n",__func__);
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

	fmt->flags = mt9p012_formats[index].flags;
	strlcpy(fmt->description, mt9p012_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = mt9p012_formats[index].pixelformat;

	return 0;
}

/**
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int ioctl_try_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	enum mt9p012_image_size isize;
	int ifmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct mt9p012_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix2 = &sensor->pix;
	printk("Enter to %s\n",__func__);
	isize = mt9p012_calc_size(pix->width, pix->height);

	pix->width = mt9p012_sizes[isize].width;
	pix->height = mt9p012_sizes[isize].height;
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == mt9p012_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = mt9p012_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
		printk("%s: Pixel format: V4L2_PIX_FMT_YUYV\n",__func__);
	case V4L2_PIX_FMT_UYVY:
		printk("%s: Pixel format: V4L2_PIX_FMT_UYVY\n",__func__);
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_PIX_FMT_RGB565:
		printk("%s: Pixel format: V4L2_PIX_FMT_RGB565\n",__func__);
	case V4L2_PIX_FMT_RGB565X:
		printk("%s: Pixel format: V4L2_PIX_FMT_RGB565X\n",__func__);
	case V4L2_PIX_FMT_RGB555:
		printk("%s: Pixel format: V4L2_PIX_FMT_RGB555\n",__func__);
	case V4L2_PIX_FMT_SGRBG10:
		printk("%s: Pixel format: V4L2_PIX_FMT_SGRBG10\n",__func__);
	case V4L2_PIX_FMT_RGB555X:
		printk("%s: Pixel format: V4L2_PIX_FMT_RGB555X\n",__func__);
	default:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}
	*pix2 = *pix;
	return 0;
}

/**
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;
	printk("Enter to %s\n",__func__);
	rval = ioctl_try_fmt_cap(s, f);
	if (!rval)
		sensor->pix = *pix;

	return rval;
}

/**
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct mt9p012_sensor *sensor = s->priv;
	printk("Enter to %s\n",__func__);
	f->fmt.pix = sensor->pix;

	return 0;
}

/**
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	printk("Enter to %s\n",__func__);
	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->timeperframe;

	return 0;
}

/**
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Framerate can be updated on the fly while streaming.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	u32 xclk;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	printk("Enter to %s\n",__func__);
	sensor->timeperframe = *timeperframe;

	if (sensor->power_on) {
		xclk = mt9p012_calc_xclk(client);
		mt9p012_update_clocks(s, xclk, sensor->current_iframe);
		mt9p012_set_framerate(s, &sensor->timeperframe,
			sensor->current_iframe);
	}

	*timeperframe = sensor->timeperframe;

	return 0;
}

/**
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct mt9p012_sensor *sensor = s->priv;
	printk("Enter to %s\n",__func__);
	if (sensor->pdata->priv_data_set)
		return sensor->pdata->priv_data_set(s, p);
	else
		return -EINVAL;
}

/**
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call mt9p012_configure())
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
 * mt9p012 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int err;
	printk("Enter to %s\n",__func__);
	err = mt9p012_detect(client);
	if (err < 0) {
		dev_err(&client->dev, "Unable to detect sensor\n");
		sensor->detected = 0;
		return err;
	}
	sensor->detected = 1;
	sensor->ver = err;
	dev_info(&client->dev, "Chip version 0x%02x detected\n", sensor->ver);

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
	int ifmt;
	printk("Enter to %s\n",__func__);
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frms->pixel_format == mt9p012_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index > MT9P012_FIVE_MP)
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = mt9p012_sizes[frms->index].width;
	frms->discrete.height = mt9p012_sizes[frms->index].height;

	return 0;
}

const struct v4l2_fract mt9p012_frameintervals[] = {
	{  .numerator = 1, .denominator = 11 },
	{  .numerator = 1, .denominator = 15 },
	{  .numerator = 1, .denominator = 20 },
	{  .numerator = 1, .denominator = 24 },
	{  .numerator = 1, .denominator = 30 },
};

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
				     struct v4l2_frmivalenum *frmi)
{
	int ifmt, i;
	struct mt9p012_sensor *sensor = s->priv;
	u16 use_min_readout = 0;
	struct private_vcontrol *pvc = NULL;
	printk("Enter to %s\n",__func__);
	i = find_vctrl_private(USE_MIN_SIZE_READOUT);
	if (i >= 0) {
		pvc = &video_control_private[i];
		use_min_readout = pvc->current_value;
	}

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frmi->pixel_format == mt9p012_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */

	if ((frmi->width == mt9p012_sizes[MT9P012_FIVE_MP].width) &&
		(frmi->height == mt9p012_sizes[MT9P012_FIVE_MP].height)) {
		/* The only frameinterval supported by 5MP
		 * capture sizes is 11 fps
		 */
		if (frmi->index != 0)
			return -EINVAL;
	} else if ((frmi->width == mt9p012_sizes[MT9P012_BIN2X].width) &&
			(frmi->height == mt9p012_sizes[MT9P012_BIN2X].height)) {
		/* The max framerate supported by MT9P012_BIN2X is 30 fps
		 */
		if (frmi->index > 4)
			return -EINVAL;
		/* If want to use min frame and there is a smaller size that
		   qualifies, do no allow this readout
		 */
		if ((use_min_readout == 1) &&
			(sensor->min_readout.requested_width <=
			mt9p012_sizes[MT9P012_BIN4X].width) &&
			(sensor->min_readout.requested_height <=
			mt9p012_sizes[MT9P012_BIN4X].height))
				return -EINVAL;
	} else {
		if (frmi->index > 4)
			return -EINVAL;
	}

	frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frmi->discrete.numerator =
				mt9p012_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				mt9p012_frameintervals[frmi->index].denominator;

	return 0;
}

/**
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power new_power)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *c = to_i2c_client(sensor->dev);
	int rval = 0;
	printk("Enter to %s\n",__func__);
	switch (new_power) {
	case V4L2_POWER_ON:
		rval = sensor->pdata->power_set(s, V4L2_POWER_ON);
		if (rval)
			break;
		sensor->power_on = true;
		if (sensor->detected)
			mt9p012_configure(s);
		else {
			rval = ioctl_dev_init(s);
			if (rval)
				goto err_on;
/*
		if (sensor->pdata->get_config_flags()
				& MT9P012_MIRROR_MODE_FLAG)
			overwrite_with_mirror_settings();
*/
		}

		break;
	case V4L2_POWER_OFF:
err_on:
		sensor->power_on = false;
		sensor->pdata->power_set(s, V4L2_POWER_OFF);
	//	sensor->pdata->lock_cpufreq(CPU_CLK_UNLOCK);
		break;
	case V4L2_POWER_STANDBY:
		if (sensor->detected)
			mt9p012_write_regs(c, stream_off_list);
		sensor->power_on = false;
		rval = sensor->pdata->power_set(s,
			V4L2_POWER_STANDBY);
		break;
	default:
		rval =  -EINVAL;
	}

	return rval;
}

static struct v4l2_int_ioctl_desc mt9p012_ioctl_desc[] = {
	{ .num = vidioc_int_enum_framesizes_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ .num = vidioc_int_enum_frameintervals_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
	{ .num = vidioc_int_dev_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ .num = vidioc_int_dev_exit_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_exit },
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_priv },
	{ .num = vidioc_int_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_init },
	{ .num = vidioc_int_enum_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ .num = vidioc_int_try_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_try_fmt_cap },
	{ .num = vidioc_int_g_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ .num = vidioc_int_s_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_fmt_cap },
	{ .num = vidioc_int_g_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ .num = vidioc_int_s_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_queryctrl },
	{ .num = vidioc_int_g_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_ctrl },
};

static struct v4l2_int_slave mt9p012_slave = {
	.ioctls = mt9p012_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9p012_ioctl_desc),
};

static struct v4l2_int_device mt9p012_int_device = {
	.module = THIS_MODULE,
	.name = DRIVER_NAME,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &mt9p012_slave,
	},
};

/**
 * mt9p012_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int mt9p012_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct mt9p012_sensor *sensor;
	struct mt9p012_platform_data *pdata;
	int err;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	/* Don't keep pointer to platform data, copy elements instead */
	sensor->pdata = kzalloc(sizeof(*sensor->pdata), GFP_KERNEL);
	if (!sensor->pdata) {
		err = -ENOMEM;
		goto on_err1;
	}

	sensor->video_control = kzalloc(sizeof(video_control), GFP_KERNEL);
	if (!sensor->video_control) {
		err = -ENOMEM;
		goto on_err2;
	}
	memcpy(sensor->video_control, video_control, sizeof(video_control));
	sensor->n_video_control = ARRAY_SIZE(video_control);

	sensor->pdata->power_set = pdata->power_set;
	sensor->pdata->priv_data_set = pdata->priv_data_set;
	//sensor->pdata->lock_cpufreq = pdata->lock_cpufreq;   /* 720p mode */
	//sensor->pdata->get_config_flags = pdata->get_config_flags;

	/* Set sensor default values */
	sensor->timeperframe.numerator = 1;
	sensor->timeperframe.denominator = 15;
	sensor->x_clk = MT9P012_XCLK_NOM_4;
	sensor->pix.width = MT9P012_VIDEO_WIDTH_4X_BINN_SCALED;
	sensor->pix.height = MT9P012_VIDEO_WIDTH_4X_BINN_SCALED;
	sensor->pix.pixelformat = V4L2_PIX_FMT_SGRBG10;
	sensor->power_on = false;

	/* Set min/max limits */
	sensor->min_exposure_time = MT9P012_DEF_MIN_EXPOSURE;
	sensor->fps_max_exposure_time = 33333;
	sensor->abs_max_exposure_time = MT9P012_DEF_MAX_EXPOSURE;
	sensor->min_linear_gain = MT9P012_MIN_LINEAR_GAIN;
	sensor->max_linear_gain = MT9P012_MAX_LINEAR_GAIN;

	/* Set Frame Delays */
	sensor->sensor_params.gain_frame_delay = MT9P012_GAIN_FRAME_DELAY;
	sensor->sensor_params.exp_time_frame_delay =
		MT9P012_EXP_TIME_FRAME_DELAY;

	sensor->v4l2_int_device = &mt9p012_int_device;
	sensor->v4l2_int_device->priv = sensor;
	sensor->dev = &client->dev;

	i2c_set_clientdata(client, sensor);

	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err)
		goto on_err3;

	return 0;

on_err3:
	i2c_set_clientdata(client, NULL);
	kfree(sensor->video_control);
on_err2:
	kfree(sensor->pdata);
on_err1:
	kfree(sensor);
	return err;
}

/**
 * mt9p012_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of mt9p012_probe().
 */
static int mt9p012_remove(struct i2c_client *client)
{
	struct mt9p012_sensor *sensor = i2c_get_clientdata(client);

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	i2c_set_clientdata(client, NULL);
	kfree(sensor->video_control);
	kfree(sensor->pdata);
	kfree(sensor);

	return 0;
}

static const struct i2c_device_id mt9p012_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mt9p012_id);

static struct i2c_driver mt9p012_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mt9p012_probe,
	.remove = mt9p012_remove,
	.id_table = mt9p012_id,
};

/**
 * mt9p012sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init mt9p012_init(void)
{
	pr_info("mt9p012 driver loading\n");
	return i2c_add_driver(&mt9p012_i2c_driver);
}
module_init(mt9p012_init);

/**
 * mt9p012sensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of mt9p012sensor_init.
 */
static void __exit mt9p012_cleanup(void)
{
	i2c_del_driver(&mt9p012_i2c_driver);
}
module_exit(mt9p012_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mt9p012 camera sensor driver");
