/*
 * mt9p012.c - mt9p012 sensor driver
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Contributors:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Sergio Aguirre <saaguirre@ti.com>
 * 	Martinez Leonides
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

#define DRIVER_NAME  "mt9p012"

/* MT9P012 has 8/16/32 registers */
#define MT9P012_8BIT			1
#define MT9P012_16BIT			2
#define MT9P012_32BIT			4

/* terminating token for reg list */
#define MT9P012_TOK_TERM 		0xFF

/* delay token for reg list */
#define MT9P012_TOK_DELAY		100

/* The ID values we are looking for */
#define MT9P012_MOD_ID			0x2800
#define MT9P012_MFR_ID			0x0006

/* FPS Capabilities */
#define MT9P012_MIN_FPS		11
#define MT9P012_DEF_FPS		15
#define MT9P012_MAX_FPS		30

#define MT9P012_I2C_RETRY_COUNT	5

#define MT9P012_XCLK_NOM_1 12000000
#define MT9P012_XCLK_NOM_2 24000000

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
#define MT9P012_VIDEO_WIDTH_4X_BINN_SCALED      216
#define MT9P012_VIDEO_HEIGHT_4X_BINN_SCALED     162

/* Default coarse integration times to get a good exposure */
#define MT9P012_COARSE_INT_TIME_216	         550
#define MT9P012_COARSE_INT_TIME_648	         550
#define MT9P012_COARSE_INT_TIME_216_30FPS	1350
#define MT9P012_COARSE_INT_TIME_648_30FPS	1350
#define MT9P012_COARSE_INT_TIME_1296		1000
#define MT9P012_COARSE_INT_TIME_3MP		1700
#define MT9P012_COARSE_INT_TIME_5MP		1700
#define MT9P012_COARSE_INT_TIME_INDEX	    	1
#define MT9P012_TST_PAT 			0x0

/* Analog gain values */
#define MT9P012_MIN_GAIN	0x08
#define MT9P012_MAX_GAIN	0x7F
#define MT9P012_DEF_GAIN	0x43
#define MT9P012_GAIN_STEP   	0x1

#define MT9P012_GAIN_INDEX	1

/* Exposure time values */
#define MT9P012_DEF_MIN_EXPOSURE	0x08
#define MT9P012_DEF_MAX_EXPOSURE	0x7F
#define MT9P012_DEF_EXPOSURE	    	0x43
#define MT9P012_EXPOSURE_STEP       	1

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
	MT9P012_BIN4XSCALE,
	MT9P012_BIN4X,
	MT9P012_BIN2X,
	MT9P012_THREE_MP,
	MT9P012_FIVE_MP
};

#define MT9P012_NUM_IMAGE_SIZES		5
#define MT9P012_NUM_PIXEL_FORMATS	1
#define MT9P012_NUM_FPS			2	/* 2 ranges */
#define MT9P012_FPS_LOW_RANGE		0
#define MT9P012_FPS_HIGH_RANGE		1

/**
 * struct capture_size - image capture size information
 * @width: image width in pixels
 * @height: image height in pixels
 */
struct mt9p012_capture_size {
	unsigned long width;
	unsigned long height;
};

/**
 * struct mt9p012_pll_settings - struct for storage of sensor pll values
 * @vt_pix_clk_div: vertical pixel clock divider
 * @vt_sys_clk_div: veritcal system clock divider
 * @pre_pll_div: pre pll divider
 * @fine_int_tm: fine resolution interval time
 * @frame_lines: number of lines in frame
 * @line_len: number of pixels in line
 * @min_pll: minimum pll multiplier
 * @max_pll: maximum pll multiplier
 */
struct mt9p012_pll_settings {
	u16 vt_pix_clk_div;
	u16 vt_sys_clk_div;
	u16 pre_pll_div;

	u16 fine_int_tm;
	u16 frame_lines;
	u16 line_len;

	u16 min_pll;
	u16 max_pll;
};

/*
 * Array of image sizes supported by MT9P012.  These must be ordered from
 * smallest image size to largest.
 */
const static struct mt9p012_capture_size mt9p012_sizes[] = {
	{  216, 162 },	/* 4X BINNING+SCALING */
	{  648, 486 },	/* 4X BINNING */
	{ 1296, 972 },	/* 2X BINNING */
	{ 2048, 1536},	/* 3 MP */
	{ 2592, 1944},	/* 5 MP */
};

/* PLL settings for MT9P012 */
enum mt9p012_pll_type {
	MT9P012_PLL_5MP = 0,
	MT9P012_PLL_3MP,
	MT9P012_PLL_1296_15FPS,
	MT9P012_PLL_1296_30FPS,
	MT9P012_PLL_648_15FPS,
	MT9P012_PLL_648_30FPS,
	MT9P012_PLL_216_15FPS,
	MT9P012_PLL_216_30FPS
};

/* Debug functions */
static int debug = 1;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug Enabled (0-1)");

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
	int ver;
	int fps;
	int detected;
	unsigned long xclk_current;
};

/* list of image formats supported by mt9p012 sensor */
const static struct v4l2_fmtdesc mt9p012_formats[] = {
	{
		.description    = "Bayer10 (GrR/BGb)",
		.pixelformat    = V4L2_PIX_FMT_SGRBG10,
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
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x01},
	/* less than frame_lines-1 */
	{.length = MT9P012_16BIT, .reg = REG_COARSE_INT_TIME, .val = 500},
	 /* updating */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0}
};

/* Structure to set analog gain */
static struct mt9p012_reg set_analog_gain[] = {
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x01},
	{.length = MT9P012_16BIT, .reg = REG_ANALOG_GAIN_GLOBAL,
		.val = MT9P012_MIN_GAIN},
	 /* updating */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0},
};

/*
 * Common MT9P012 register initialization for all image sizes, pixel formats,
 * and frame rates
 */
const static struct mt9p012_reg mt9p012_common[] = {
	{MT9P012_8BIT, REG_SOFTWARE_RESET, 0x01},
	{MT9P012_TOK_DELAY, 0x00, 5}, /* Delay = 5ms, min 2400 xcks */
	{MT9P012_16BIT, REG_RESET_REGISTER, 0x10C8},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_ANALOG_GAIN_GREENR, 0x0020},
	{MT9P012_16BIT, REG_ANALOG_GAIN_RED, 0x0020},
	{MT9P012_16BIT, REG_ANALOG_GAIN_BLUE, 0x0020},
	{MT9P012_16BIT, REG_ANALOG_GAIN_GREENB, 0x0020},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_GREENR, 0x0100},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_RED, 0x0100},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_BLUE, 0x0100},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_GREENB, 0x0100},
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
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update all at once */
	{MT9P012_TOK_TERM, 0, 0}
};

/*
 * mt9p012 register configuration for all combinations of pixel format and
 * image size
 */
	/* 4X BINNING+SCALING */
const static struct mt9p012_reg enter_video_216_15fps[] = {
	/* stream off */
	{.length = MT9P012_8BIT, .reg = REG_MODE_SELECT, .val = 0x00},
	{.length = MT9P012_TOK_DELAY, .reg = 0x00, .val = 100},
	 /* hold */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x01},
	{.length = MT9P012_16BIT, .reg = REG_VT_PIX_CLK_DIV, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_VT_SYS_CLK_DIV, .val = 2},
	{.length = MT9P012_16BIT, .reg = REG_PRE_PLL_CLK_DIV, .val = 2},
	{.length = MT9P012_16BIT, .reg = REG_PLL_MULTIPLIER, .val = 126},
	{.length = MT9P012_16BIT, .reg = REG_OP_PIX_CLK_DIV, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_OP_SYS_CLK_DIV, .val = 2},
	{.length = MT9P012_16BIT, .reg = REG_RESERVED_MFR_3064,
		.val = 0x0805},
	{.length = MT9P012_16BIT, .reg = REG_X_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_WIDTH_4X_BINN_SCALED},
	{.length = MT9P012_16BIT, .reg = REG_Y_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_HEIGHT_4X_BINN_SCALED},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_END, .val = 2593},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_END, .val = 1945},
	{.length = MT9P012_16BIT, .reg = REG_READ_MODE, .val = 0x04FC},
	{.length = MT9P012_16BIT, .reg = REG_FINE_INT_TIME, .val = 1794},
	{.length = MT9P012_16BIT, .reg = REG_FRAME_LEN_LINES, .val = 574},
	{.length = MT9P012_16BIT, .reg = REG_LINE_LEN_PCK, .val = 2712},
	 /* 0x10/0x30 = 0.3333 */
	{.length = MT9P012_16BIT, .reg = REG_SCALE_M, .val = 0x0030},
	/* enable scaler */
	{.length = MT9P012_16BIT, .reg = REG_SCALING_MODE, .val = 0x0002},
	{.length = MT9P012_16BIT, .reg = REG_COARSE_INT_TIME,
		.val = MT9P012_COARSE_INT_TIME_216},
	/* update */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0}
	};

	/* Video mode, 4x binning + scaling, range 16 - 30 fps */
const static struct mt9p012_reg enter_video_216_30fps[] = {
	/* stream off */
	{.length = MT9P012_8BIT, .reg = REG_MODE_SELECT, .val = 0x00},
	{.length = MT9P012_TOK_DELAY, .reg = 0x00, .val = 100},
	/* hold */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x01},
	{.length = MT9P012_16BIT, .reg = REG_VT_PIX_CLK_DIV, .val = 5},
	{.length = MT9P012_16BIT, .reg = REG_VT_SYS_CLK_DIV, .val = 2},
	{.length = MT9P012_16BIT, .reg = REG_PRE_PLL_CLK_DIV, .val = 3},
	{.length = MT9P012_16BIT, .reg = REG_PLL_MULTIPLIER, .val = 192},
	{.length = MT9P012_16BIT, .reg = REG_OP_PIX_CLK_DIV, .val = 10},
	{.length = MT9P012_16BIT, .reg = REG_OP_SYS_CLK_DIV, .val = 2},
	{.length = MT9P012_16BIT, .reg = REG_RESERVED_MFR_3064, .val = 0x0805},
	{.length = MT9P012_16BIT, .reg = REG_X_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_WIDTH_4X_BINN},
	{.length = MT9P012_16BIT, .reg = REG_Y_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_HEIGHT_4X_BINN},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_END, .val = 2593},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_END, .val = 1945},
	{.length = MT9P012_16BIT, .reg = REG_READ_MODE, .val = 0x04FC},
	{.length = MT9P012_16BIT, .reg = REG_FINE_INT_TIME, .val = 1794},
	{.length = MT9P012_16BIT, .reg = REG_FRAME_LEN_LINES, .val = 1374},
	{.length = MT9P012_16BIT, .reg = REG_LINE_LEN_PCK, .val = 3712},
	/* 0x10/0x30 = 0.3333 */
	{.length = MT9P012_16BIT, .reg = REG_SCALE_M, .val = 0x0030},
	/* enable scaler */
	{.length = MT9P012_16BIT, .reg = REG_SCALING_MODE, .val = 0x0002},
	{.length = MT9P012_16BIT, .reg = REG_COARSE_INT_TIME,
		.val = MT9P012_COARSE_INT_TIME_216_30FPS},
	/* update */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0}
	};


	/*Video mode, 4x binning: 648 x 486, range 8 - 15 fps*/
const static struct mt9p012_reg enter_video_648_15fps[] = {
	/* stream off */
	{.length = MT9P012_8BIT, .reg = REG_MODE_SELECT, .val = 0x00},
	{.length = MT9P012_TOK_DELAY, .reg = 0x00, .val = 100},
	/* hold */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x01},
	{.length = MT9P012_16BIT, .reg = REG_VT_PIX_CLK_DIV, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_VT_SYS_CLK_DIV, .val = 2},
	{.length = MT9P012_16BIT, .reg = REG_PRE_PLL_CLK_DIV, .val = 2},
	{.length = MT9P012_16BIT, .reg = REG_PLL_MULTIPLIER, .val = 126},
	{.length = MT9P012_16BIT, .reg = REG_OP_PIX_CLK_DIV, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_OP_SYS_CLK_DIV, .val = 2},
	{.length = MT9P012_16BIT, .reg = REG_RESERVED_MFR_3064, .val = 0x0805},
	{.length = MT9P012_16BIT, .reg = REG_X_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_WIDTH_4X_BINN},
	{.length = MT9P012_16BIT, .reg = REG_Y_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_HEIGHT_4X_BINN},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_END, .val = 2593},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_END, .val = 1945},
	{.length = MT9P012_16BIT, .reg = REG_READ_MODE, .val = 0x04FC},
	{.length = MT9P012_16BIT, .reg = REG_FINE_INT_TIME, .val = 1794},
	{.length = MT9P012_16BIT, .reg = REG_FRAME_LEN_LINES, .val = 574},
	{.length = MT9P012_16BIT, .reg = REG_LINE_LEN_PCK, .val = 2712},
	{.length = MT9P012_16BIT, .reg = REG_SCALING_MODE, .val = 0x0000},
	{.length = MT9P012_16BIT, .reg = REG_COARSE_INT_TIME,
		.val = MT9P012_COARSE_INT_TIME_648},
	/* update */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0}
};

	/* Video mode, 4x binning: 648 x 486, range 16 - 30 fps */
const static struct mt9p012_reg enter_video_648_30fps[] = {
	/* stream off */
	{.length = MT9P012_8BIT, .reg = REG_MODE_SELECT, .val = 0x00},
	{.length = MT9P012_TOK_DELAY, .reg = 0x00, .val = 100},
	/* hold */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x01},
	{.length = MT9P012_16BIT, .reg = REG_VT_PIX_CLK_DIV, .val = 5},
	{.length = MT9P012_16BIT, .reg = REG_VT_SYS_CLK_DIV, .val = 2},
	{.length = MT9P012_16BIT, .reg = REG_PRE_PLL_CLK_DIV, .val = 3},
	{.length = MT9P012_16BIT, .reg = REG_PLL_MULTIPLIER, .val = 192},
	{.length = MT9P012_16BIT, .reg = REG_OP_PIX_CLK_DIV, .val = 10},
	{.length = MT9P012_16BIT, .reg = REG_OP_SYS_CLK_DIV, .val = 2},
	{.length = MT9P012_16BIT, .reg = REG_RESERVED_MFR_3064, .val = 0x0805},
	{.length = MT9P012_16BIT, .reg = REG_X_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_WIDTH_4X_BINN},
	{.length = MT9P012_16BIT, .reg = REG_Y_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_HEIGHT_4X_BINN},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_END, .val = 2593},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_END, .val = 1945},
	{.length = MT9P012_16BIT, .reg = REG_READ_MODE, .val = 0x04FC},
	{.length = MT9P012_16BIT, .reg = REG_FINE_INT_TIME, .val = 1794},
	{.length = MT9P012_16BIT, .reg = REG_FRAME_LEN_LINES, .val = 1374},
	{.length = MT9P012_16BIT, .reg = REG_LINE_LEN_PCK, .val = 3712},
	{.length = MT9P012_16BIT, .reg = REG_SCALING_MODE, .val = 0x0000},
	{.length = MT9P012_16BIT, .reg = REG_COARSE_INT_TIME,
		.val = MT9P012_COARSE_INT_TIME_648_30FPS},
	/* update */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0}
};

	/* Video mode, scaler off: 1296 x 972, range  11 - 21 fps*/
const static struct mt9p012_reg enter_video_1296_15fps[] = {
	/* stream off */
	{.length = MT9P012_8BIT, .reg = REG_MODE_SELECT, .val = 0x00},
	{.length = MT9P012_TOK_DELAY, .reg = 0x00, .val = 100},
	/* hold */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x01},
	{.length = MT9P012_16BIT, .reg = REG_VT_PIX_CLK_DIV, .val = 5},
	{.length = MT9P012_16BIT, .reg = REG_VT_SYS_CLK_DIV, .val = 2},
	{.length = MT9P012_16BIT, .reg = REG_PRE_PLL_CLK_DIV, .val = 3},
	{.length = MT9P012_16BIT, .reg = REG_PLL_MULTIPLIER, .val = 134},
	{.length = MT9P012_16BIT, .reg = REG_OP_PIX_CLK_DIV, .val = 10},
	{.length = MT9P012_16BIT, .reg = REG_OP_SYS_CLK_DIV, .val = 1},
	{.length = MT9P012_16BIT, .reg = REG_RESERVED_MFR_3064, .val = 0x0805},
	{.length = MT9P012_16BIT, .reg = REG_X_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_WIDTH_2X_BINN},
	{.length = MT9P012_16BIT, .reg = REG_Y_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_HEIGHT_2X_BINN},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_END, .val = 2597},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_END, .val = 1949},
	{.length = MT9P012_16BIT, .reg = REG_READ_MODE, .val = 0x046C},
	{.length = MT9P012_16BIT, .reg = REG_FINE_INT_TIME, .val = 1794},
	{.length = MT9P012_16BIT, .reg = REG_FRAME_LEN_LINES, .val = 1061},
	{.length = MT9P012_16BIT, .reg = REG_LINE_LEN_PCK, .val = 3360},
	{.length = MT9P012_16BIT, .reg = REG_SCALING_MODE, .val = 0x0000},
	{.length = MT9P012_16BIT, .reg = REG_COARSE_INT_TIME,
		.val = MT9P012_COARSE_INT_TIME_1296},
	/* update */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0}
};

	/* YUV (YCbCr) VGA */
const static struct mt9p012_reg enter_video_1296_30fps[] = {
	/* stream off */
	{.length = MT9P012_8BIT, .reg = REG_MODE_SELECT, .val = 0x00},
	{.length = MT9P012_TOK_DELAY, .reg = 0x00, .val = 100},
	/* hold */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x01},
	{.length = MT9P012_16BIT, .reg = REG_VT_PIX_CLK_DIV, .val = 5},
	{.length = MT9P012_16BIT, .reg = REG_VT_SYS_CLK_DIV, .val = 1},
	{.length = MT9P012_16BIT, .reg = REG_PRE_PLL_CLK_DIV, .val = 3},
	{.length = MT9P012_16BIT, .reg = REG_PLL_MULTIPLIER, .val = 134},
	{.length = MT9P012_16BIT, .reg = REG_OP_PIX_CLK_DIV, .val = 10},
	{.length = MT9P012_16BIT, .reg = REG_OP_SYS_CLK_DIV, .val = 1},
	{.length = MT9P012_16BIT, .reg = REG_RESERVED_MFR_3064, .val = 0x0805},
	{.length = MT9P012_16BIT, .reg = REG_X_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_WIDTH_2X_BINN},
	{.length = MT9P012_16BIT, .reg = REG_Y_OUTPUT_SIZE,
		.val = MT9P012_VIDEO_HEIGHT_2X_BINN},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_END, .val = 2597},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_END, .val = 1949},
	{.length = MT9P012_16BIT, .reg = REG_READ_MODE, .val = 0x046C},
	{.length = MT9P012_16BIT, .reg = REG_FINE_INT_TIME, .val = 1794},
	{.length = MT9P012_16BIT, .reg = REG_FRAME_LEN_LINES, .val = 1061},
	{.length = MT9P012_16BIT, .reg = REG_LINE_LEN_PCK, .val = 3360},
	{.length = MT9P012_16BIT, .reg = REG_SCALING_MODE, .val = 0x0000},
	{.length = MT9P012_16BIT, .reg = REG_COARSE_INT_TIME,
		.val = MT9P012_COARSE_INT_TIME_1296},
	 /* update */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0}
};

const static struct mt9p012_reg enter_image_mode_3MP_10fps[] = {
	/* stream off */
	{.length = MT9P012_8BIT, .reg = REG_MODE_SELECT, .val = 0x00},
	{.length = MT9P012_TOK_DELAY, .reg = 0x00, .val = 100},
	/* hold */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x01},
	{.length = MT9P012_16BIT, .reg = REG_VT_PIX_CLK_DIV, .val = 4},
	{.length = MT9P012_16BIT, .reg = REG_VT_SYS_CLK_DIV, .val = 1},
	{.length = MT9P012_16BIT, .reg = REG_PRE_PLL_CLK_DIV, .val = 5},
	/* 10 fps */
	{.length = MT9P012_16BIT, .reg = REG_PLL_MULTIPLIER, .val = 184},
	{.length = MT9P012_16BIT, .reg = REG_OP_PIX_CLK_DIV, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_OP_SYS_CLK_DIV, .val = 1},
	{.length = MT9P012_16BIT, .reg = REG_RESERVED_MFR_3064, .val = 0x0805},
	{.length = MT9P012_16BIT, .reg = REG_X_OUTPUT_SIZE,
		.val = MT9P012_IMAGE_WIDTH_MIN},
	{.length = MT9P012_16BIT, .reg = REG_Y_OUTPUT_SIZE,
		.val = MT9P012_IMAGE_HEIGHT_MIN},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_END, .val = 2599},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_END, .val = 1951},
	{.length = MT9P012_16BIT, .reg = REG_READ_MODE, .val = 0x0024},
	{.length = MT9P012_16BIT, .reg = REG_FINE_INT_TIME, .val = 882},
	{.length = MT9P012_16BIT, .reg = REG_FRAME_LEN_LINES, .val = 2056},
	{.length = MT9P012_16BIT, .reg = REG_LINE_LEN_PCK, .val = 5372},
	/* 0x10/0x14 = 0.80 */
	{.length = MT9P012_16BIT, .reg = REG_SCALE_M, .val = 0x0014},
	/* enable scaler */
	{.length = MT9P012_16BIT, .reg = REG_SCALING_MODE, .val = 0x0002},
	{.length = MT9P012_16BIT, .reg = REG_TEST_PATTERN,
		.val = MT9P012_TST_PAT},
	{.length = MT9P012_16BIT, .reg = REG_COARSE_INT_TIME,
		.val = MT9P012_COARSE_INT_TIME_3MP},
	/* update */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0}
};

/* Image mode, 5 MP @ 10 fps */
const static struct mt9p012_reg enter_image_mode_5MP_10fps[] = {
	/* stream off */
	{.length = MT9P012_8BIT, .reg = REG_MODE_SELECT, .val = 0x00},
	{.length = MT9P012_TOK_DELAY, .reg = 0x00, .val = 100},
	/* hold */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x01},
	{.length = MT9P012_16BIT, .reg = REG_VT_PIX_CLK_DIV, .val = 4},
	{.length = MT9P012_16BIT, .reg = REG_VT_SYS_CLK_DIV, .val = 1},
	{.length = MT9P012_16BIT, .reg = REG_PRE_PLL_CLK_DIV, .val = 5},
	/* 10 fps */
	{.length = MT9P012_16BIT, .reg = REG_PLL_MULTIPLIER, .val = 184},
	{.length = MT9P012_16BIT, .reg = REG_OP_PIX_CLK_DIV, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_OP_SYS_CLK_DIV, .val = 1},
	{.length = MT9P012_16BIT, .reg = REG_RESERVED_MFR_3064, .val = 0x0805},
	{.length = MT9P012_16BIT, .reg = REG_X_OUTPUT_SIZE,
		.val = MT9P012_IMAGE_WIDTH_MAX},
	{.length = MT9P012_16BIT, .reg = REG_Y_OUTPUT_SIZE,
		.val = MT9P012_IMAGE_HEIGHT_MAX},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_START, .val = 8},
	{.length = MT9P012_16BIT, .reg = REG_X_ADDR_END, .val = 2599},
	{.length = MT9P012_16BIT, .reg = REG_Y_ADDR_END, .val = 1951},
	{.length = MT9P012_16BIT, .reg = REG_READ_MODE, .val = 0x0024},
	{.length = MT9P012_16BIT, .reg = REG_FINE_INT_TIME, .val = 882},
	{.length = MT9P012_16BIT, .reg = REG_FRAME_LEN_LINES, .val = 2056},
	{.length = MT9P012_16BIT, .reg = REG_LINE_LEN_PCK, .val = 5372},
	{.length = MT9P012_16BIT, .reg = REG_SCALE_M, .val = 0x0000},
	/* disable scaler */
	{.length = MT9P012_16BIT, .reg = REG_SCALING_MODE, .val = 0x0000},
	{.length = MT9P012_16BIT, .reg = REG_COARSE_INT_TIME,
		.val = MT9P012_COARSE_INT_TIME_5MP},
	/* update */
	{.length = MT9P012_8BIT, .reg = REG_GROUPED_PAR_HOLD, .val = 0x00},
	{.length = MT9P012_TOK_TERM, .reg = 0, .val = 0}
};

static u32 min_exposure_time;
static u32 max_exposure_time;
static u32 pix_clk_freq;

/* Structure to set frame rate */
static struct mt9p012_reg set_fps[2];

/**
 * struct mt9p012_pll_settings - struct for storage of sensor pll values
 * @vt_pix_clk_div: vertical pixel clock divider
 * @vt_sys_clk_div: veritcal system clock divider
 * @pre_pll_div: pre pll divider
 * @fine_int_tm: fine resolution interval time
 * @frame_lines: number of lines in frame
 * @line_len: number of pixels in line
 * @min_pll: minimum pll multiplier
 * @max_pll: maximum pll multiplier
 */
const static struct mt9p012_pll_settings all_pll_settings[] = {
	/* PLL_5MP */
	{.vt_pix_clk_div = 4, .vt_sys_clk_div = 1, .pre_pll_div = 5,
	.fine_int_tm = 882, .frame_lines = 2056, .line_len = 5372,
	.min_pll = 160, .max_pll = 200},
	/* PLL_3MP */
	{.vt_pix_clk_div = 4, .vt_sys_clk_div = 1, .pre_pll_div = 5,
	.fine_int_tm = 882, .frame_lines = 2056, .line_len = 5372,
	.min_pll = 160, .max_pll = 200},
	/* PLL_1296_15FPS */
	{.vt_pix_clk_div = 5, .vt_sys_clk_div = 2, .pre_pll_div = 3,
	.fine_int_tm = 1794, .frame_lines = 1061, .line_len = 3360,
	.min_pll = 96, .max_pll = 190},
	/* PLL_1296_30FPS */
	{.vt_pix_clk_div = 5, .vt_sys_clk_div = 1, .pre_pll_div = 3,
	.fine_int_tm = 1794, .frame_lines = 1061, .line_len = 3360,
	.min_pll = 96, .max_pll = 150},
	/* PLL_648_15FPS */
	{.vt_pix_clk_div = 8, .vt_sys_clk_div = 2, .pre_pll_div = 2,
	.fine_int_tm = 1794, .frame_lines = 574, .line_len = 2712,
	.min_pll = 92, .max_pll = 128},
	/* PLL_648_30FPS */
	{.vt_pix_clk_div = 5, .vt_sys_clk_div = 2, .pre_pll_div = 3,
	.fine_int_tm = 1794, .frame_lines = 1374, .line_len = 3712,
	.min_pll = 96, .max_pll = 192},
	/* PLL_216_15FPS */
	{.vt_pix_clk_div = 8, .vt_sys_clk_div = 2, .pre_pll_div = 2,
	.fine_int_tm = 1794,  .frame_lines = 574, .line_len = 2712,
	.min_pll = 92, .max_pll = 126},
	/* PLL_216_30FPS */
	{.vt_pix_clk_div = 5, .vt_sys_clk_div = 2, .pre_pll_div = 3,
	.fine_int_tm = 1794,  .frame_lines = 1374, .line_len = 3712,
	.min_pll = 96, .max_pll = 192}
};

static enum mt9p012_pll_type current_pll_video;

const static struct mt9p012_reg
		*mt9p012_reg_init[MT9P012_NUM_FPS][MT9P012_NUM_IMAGE_SIZES] = {
	{
		enter_video_216_15fps,
		enter_video_648_15fps,
		enter_video_1296_15fps,
		enter_image_mode_3MP_10fps,
		enter_image_mode_5MP_10fps
	},
	{
		enter_video_216_30fps,
		enter_video_648_30fps,
		enter_video_1296_30fps,
		enter_image_mode_3MP_10fps,
		enter_image_mode_5MP_10fps
	},
};

/**
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
			.minimum = MT9P012_DEF_MIN_EXPOSURE,
			.maximum = MT9P012_DEF_MAX_EXPOSURE,
			.step = MT9P012_EXPOSURE_STEP,
			.default_value = MT9P012_DEF_EXPOSURE,
		},
		.current_value = MT9P012_DEF_EXPOSURE,
	},
	{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Gain",
			.minimum = MT9P012_MIN_GAIN,
			.maximum = MT9P012_MAX_GAIN,
			.step = MT9P012_GAIN_STEP,
			.default_value = MT9P012_DEF_GAIN,
		},
		.current_value = MT9P012_DEF_GAIN,
	}
};

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
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
		return 0;
	}
	v4l_dbg(1, debug, client, "read from offset 0x%x error %d\n", reg, err);
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

	if (!client->adapter)
		return -ENODEV;

	if (data_length != MT9P012_8BIT && data_length != MT9P012_16BIT
					&& data_length != MT9P012_32BIT)
		return -EINVAL;

again:
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

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;

	v4l_dbg(1, debug, client, "wrote 0x%x to offset 0x%x error %d", val,
							reg, err);
	if (retry <= MT9P012_I2C_RETRY_COUNT) {
		v4l_warn(client, "retry ... %d", retry);
		retry++;
		mdelay(20);
		goto again;
	}
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
 * mt9p012_calc_pll - Calculate PLL settings based on input image size
 * @isize: enum value corresponding to image size
 * @xclk: xclk value (calculate by mt9p012sensor_calc_xclk())
 * @sensor: pointer to sensor device information structure
 *
 * Calculates sensor PLL related settings (scaler, fps, pll_multiplier,
 * pix_clk_freq, min_exposure_time, max_exposure_time) based on input
 * image size.  It then applies the fps register settings based on
 * these calculations.
 */
static int mt9p012_calc_pll(enum mt9p012_image_size isize, unsigned long xclk,
			    struct mt9p012_sensor *sensor)
{
	int err = 0, row = 1, i = 0;
	unsigned int vt_pix_clk;
	unsigned int pll_multiplier;
	unsigned int exposure_factor, pix_clk_scaled;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	struct vcontrol *lvc;

	/* Greater than 1296x972
	1. Scaler is 0
	2. fps is 10
	3. Apply image mode settings
	4. Turn Streaming ON.
	5. Exit
	*/
	if (isize > MT9P012_BIN2X) {
		/* Burst Mode */
		sensor->scaler = 0;
		sensor->fps = 10;
		current_pll_video = MT9P012_PLL_5MP;
		return 0;
	}

	/* Greater than 648X486 case
	1. Scaler is 0
	2. If fps>21 then choose PLL for 30
	3. If fps<21 then choose PLL for 15

	Greater than 216X162 case
	1. Scaler is 1
	2. If fps>15 then choose PLL for 30
	3. If fps<15 then choose PLL for 15

	Greater than 0 to 216x162
	1. Scaler is 2.
	2. If fps>15 then choose PLL for 30
	3. If fps<15 then choose PLL for 15
	*/

	if (isize > MT9P012_BIN4X) {
		sensor->scaler = 0;
		if (sensor->fps > 21)
			current_pll_video = MT9P012_PLL_1296_30FPS;
		else
			current_pll_video = MT9P012_PLL_1296_15FPS;
	} else if (isize > MT9P012_BIN4XSCALE) {
		sensor->scaler = 1;
		if (sensor->fps > 15)
			current_pll_video = MT9P012_PLL_648_30FPS;
		else
			current_pll_video = MT9P012_PLL_648_15FPS;
	} else {
		sensor->scaler = 2;
		if (sensor->fps > 15)
			current_pll_video = MT9P012_PLL_216_30FPS;
		else
			current_pll_video = MT9P012_PLL_216_15FPS;
	}

	/* Row adjustment */
	if (sensor->scaler && (sensor->fps < 16))
		row = 2; /* Adjustment when using 4x binning and 12 MHz clk */

	/* Calculate the PLL, set fps register */
	vt_pix_clk = sensor->fps *
		all_pll_settings[current_pll_video].frame_lines *
		all_pll_settings[current_pll_video].line_len;

	pll_multiplier =
		(((vt_pix_clk
		   * all_pll_settings[current_pll_video].vt_pix_clk_div
		   * all_pll_settings[current_pll_video].vt_sys_clk_div
		   * row) / xclk)
		   * all_pll_settings[current_pll_video].pre_pll_div) + 1;

	if (pll_multiplier < all_pll_settings[current_pll_video].min_pll)
		pll_multiplier = all_pll_settings[current_pll_video].min_pll;
	else if (pll_multiplier > all_pll_settings[current_pll_video].max_pll)
		pll_multiplier = all_pll_settings[current_pll_video].max_pll;

	pix_clk_freq = (xclk /
			(all_pll_settings[current_pll_video].pre_pll_div
			 * all_pll_settings[current_pll_video].vt_pix_clk_div
			 * all_pll_settings[current_pll_video].vt_sys_clk_div
			 * row)) * pll_multiplier;
	min_exposure_time = (all_pll_settings[current_pll_video].fine_int_tm
			     * 1000000 / pix_clk_freq) + 1;
	exposure_factor = (all_pll_settings[current_pll_video].frame_lines - 1)
				* all_pll_settings[current_pll_video].line_len;
	exposure_factor += all_pll_settings[current_pll_video].fine_int_tm;
	exposure_factor *= 100;
	pix_clk_scaled = pix_clk_freq / 100;
	max_exposure_time = (exposure_factor / pix_clk_scaled) * 100;

	/* Apply the fps settings */
	set_fps[0].length = MT9P012_16BIT;
	set_fps[0].reg = REG_PLL_MULTIPLIER;
	set_fps[0].val = pll_multiplier;
	set_fps[1].length = MT9P012_TOK_TERM;
	set_fps[1].reg = 0;
	set_fps[1].val = 0;

	/* Update min/max for query control */
	i = find_vctrl(V4L2_CID_EXPOSURE);
	if (i >= 0) {
		lvc = &video_control[i];
		lvc->qc.minimum = min_exposure_time;
		lvc->qc.maximum = max_exposure_time;
	}

	err = mt9p012_write_regs(client, set_fps);
	return err;
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

	for (isize = MT9P012_BIN4XSCALE; isize <= MT9P012_FIVE_MP; isize++) {
		if (mt9p012_sizes[isize].height *
					mt9p012_sizes[isize].width >= pixels) {
			/* To improve image quality in VGA */
			if (pixels > MT9P012_CIF_PIXELS &&
			    isize == MT9P012_BIN4X)
				isize = MT9P012_BIN2X;
			else {
				if ((pixels > MT9P012_QQVGA_PIXELS) &&
				    (isize == MT9P012_BIN4XSCALE))
					isize = MT9P012_BIN4X;
			}
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

	for (isize = MT9P012_BIN4XSCALE; isize <= MT9P012_FIVE_MP; isize++) {
		if (mt9p012_sizes[isize].width >= width)
			break;
	}

	return isize;
}
/**
 * mt9p012_find_fps_index - Find the best fps range match for a
 *  requested frame rate
 * @fps: desired frame rate
 * @isize: enum value corresponding to image size
 *
 * Find the best match for a requested frame rate.  The best match
 * is chosen between two fps ranges (11 - 15 and 16 - 30 fps) depending on
 * the image size. For image sizes larger than BIN2X, frame rate is fixed
 * at 10 fps.
 */
static unsigned int mt9p012_find_fps_index(unsigned int fps,
					   enum mt9p012_image_size isize)
{
	unsigned int index = MT9P012_FPS_LOW_RANGE;

	if (isize > MT9P012_BIN4X) {
		if (fps > 21)
			index = MT9P012_FPS_HIGH_RANGE;
	} else {
		if (fps > 15)
			index = MT9P012_FPS_HIGH_RANGE;
	}

	return index;
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
	struct v4l2_pix_format *pix = &sensor->pix;

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

	if ((pix->width <= MT9P012_VIDEO_WIDTH_4X_BINN) && (sensor->fps > 15))
		return MT9P012_XCLK_NOM_2;

	return MT9P012_XCLK_NOM_1;
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
	unsigned int fps_index;
	int err;

	isize = mt9p012_find_isize(pix->width);

	/* common register initialization */
	err = mt9p012_write_regs(client, mt9p012_common);
	if (err)
		return err;

	fps_index = mt9p012_find_fps_index(sensor->fps, isize);

	/* configure image size and pixel format */
	err = mt9p012_write_regs(client, mt9p012_reg_init[fps_index][isize]);
	if (err)
		return err;

	/* configure frame rate */
	err = mt9p012_calc_pll(isize, sensor->xclk_current, sensor);
	if (err)
		return err;

	/* configure streaming ON */
	err = mt9p012_write_regs(client, stream_on_list);

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
	u32 model_id, mfr_id, rev;

	if (!client) {
		printk("%s: i2c client error\n",__func__);
		return -ENODEV;
	}
	if (mt9p012_read_reg(client, MT9P012_16BIT, REG_MODEL_ID, &model_id)) {
		printk("%s: Read model id error\n",__func__);
		return -ENODEV;
	}
	if (mt9p012_read_reg(client, MT9P012_8BIT, REG_MANUFACTURER_ID,
				&mfr_id)) {
		printk("%s: Read manufcaturer id error\n",__func__);
		return -ENODEV;
	}
	if (mt9p012_read_reg(client, MT9P012_8BIT, REG_REVISION_NUMBER, &rev)) {
		printk("%s: Read revision number error\n",__func__);
		return -ENODEV;
	}
	dev_info(&client->dev, "model id detected 0x%x mfr 0x%x\n", model_id,
								mfr_id);
	if ((model_id != MT9P012_MOD_ID) || (mfr_id != MT9P012_MFR_ID)) {
		/* We didn't read the values we expected, so
		 * this must not be an MT9P012.
		 */
		dev_warn(&client->dev, "model id mismatch 0x%x mfr 0x%x\n",
			model_id, mfr_id);

		return -ENODEV;
	}
	return 0;

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
	int err;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	u32 coarse_int_time = 0;

	if ((exp_time < min_exposure_time) ||
			(exp_time > max_exposure_time)) {
		dev_err(&client->dev, "Exposure time not within the "
			"legal range.\n");
		dev_err(&client->dev, "Min time %d us Max time %d us",
			min_exposure_time, max_exposure_time);
		return -EINVAL;
	}
	coarse_int_time = ((((exp_time / 10) * (pix_clk_freq / 1000)) / 1000) -
		(all_pll_settings[current_pll_video].fine_int_tm / 10)) /
		(all_pll_settings[current_pll_video].line_len / 10);

	dev_dbg(&client->dev, "coarse_int_time calculated = %d\n",
						coarse_int_time);

	set_exposure_time[MT9P012_COARSE_INT_TIME_INDEX].val = coarse_int_time;
	err = mt9p012_write_regs(client, set_exposure_time);

	if (err)
		dev_err(&client->dev, "Error setting exposure time %d\n",
									err);
	else
		lvc->current_value = exp_time;

	return err;
}

/**
 * mt9p012_set_gain - sets sensor analog gain per input value
 * @gain: analog gain value to be set on device
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
	int err;
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);

	if ((gain < MT9P012_MIN_GAIN) || (gain > MT9P012_MAX_GAIN)) {
		dev_err(&client->dev, "Gain not within the legal range");
		return -EINVAL;
	}
	set_analog_gain[MT9P012_GAIN_INDEX].val = gain;
	err = mt9p012_write_regs(client, set_analog_gain);
	if (err) {
		dev_err(&client->dev, "Error setting gain.%d", err);
		return err;
	} else
		lvc->current_value = gain;

	return err;
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
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;
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
	int retval = -EINVAL;
	int i;
	struct vcontrol *lvc;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		retval = mt9p012_set_exposure_time(vc->value, s, lvc);
		break;
	case V4L2_CID_GAIN:
		retval = mt9p012_set_gain(vc->value, s, lvc);
		break;
	}

	return retval;
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
	case V4L2_PIX_FMT_UYVY:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_RGB555X:
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
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct mt9p012_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;

	sensor->timeperframe = *timeperframe;
	sensor->xclk_current = mt9p012_calc_xclk(client);
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

	return sensor->pdata->priv_data_set(s, p);
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

	err = mt9p012_detect(client);
	if (err < 0) {
		dev_err(&client->dev, "%s: Unable to detect sensor\n", __func__);
		sensor->detected = 0;
		return err;
	}
	sensor->detected = 1;
	sensor->ver = err;
	dev_dbg(&client->dev, "Chip version 0x%02x detected\n", sensor->ver);

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

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frms->pixel_format == mt9p012_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= 5)
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
	{  .numerator = 1, .denominator = 25 },
	{  .numerator = 1, .denominator = 30 },
};

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
				     struct v4l2_frmivalenum *frmi)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frmi->pixel_format == mt9p012_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */

	if (((frmi->width == mt9p012_sizes[4].width) &&
				(frmi->height == mt9p012_sizes[4].height)) ||
				((frmi->width == mt9p012_sizes[3].width) &&
				(frmi->height == mt9p012_sizes[3].height))) {
		/* FIXME: The only frameinterval supported by 5MP and 3MP
		 * capture sizes is 1/11 fps
		 */
		if (frmi->index != 0)
			return -EINVAL;
	} else {
		if (frmi->index >= 5)
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

	switch (new_power) {
	case V4L2_POWER_ON:
		printk("%s:  V4L2_POWER_ON\n",__func__);
		rval = sensor->pdata->set_xclk(s, sensor->xclk_current);
		if (rval == -EINVAL)
			break;
		rval = sensor->pdata->power_set(s, V4L2_POWER_ON);
		if (rval)
			break;

		if (sensor->detected)
			mt9p012_configure(s);
		else {
			rval = ioctl_dev_init(s);
			if (rval)
				goto err_on;
		}
		break;
	case V4L2_POWER_OFF:
err_on:
		rval = sensor->pdata->power_set(s, V4L2_POWER_OFF);
		sensor->pdata->set_xclk(s, 0);
		break;
	case V4L2_POWER_STANDBY:
		if (sensor->detected)
			mt9p012_write_regs(c, stream_off_list);
		rval = sensor->pdata->power_set(s, V4L2_POWER_STANDBY);
		sensor->pdata->set_xclk(s, 0);
		break;
	default:
		return -EINVAL;
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
	printk("enter to %s\n",__func__);

	if (i2c_get_clientdata(client))
		return -EBUSY;
	printk("%s: i2c_get_clientdata(client) OK \n",__func__);

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}
	printk("%s: client->dev.platform_data OK \n",__func__);

	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	printk("%s: kzalloc(sizeof(*sensor) OK \n",__func__);
	/* Don't keep pointer to platform data, copy elements instead */
	sensor->pdata = kzalloc(sizeof(*sensor->pdata), GFP_KERNEL);
	if (!sensor->pdata) {
		err = -ENOMEM;
		goto on_err1;
	}
	printk("%s: kzalloc(sizeof(*sensor->pdata) OK \n",__func__);

	printk("%s: power_set\n",__func__);
	sensor->pdata->power_set = pdata->power_set;
	printk("%s: set_xclk\n",__func__);
	sensor->pdata->set_xclk = pdata->set_xclk;
	printk("%s: priv_data_set\n",__func__);
	sensor->pdata->priv_data_set = pdata->priv_data_set;

	/* Set sensor default values */
	printk("%s: Set sensor default values\n",__func__);
	sensor->timeperframe.numerator = 1;
	sensor->timeperframe.denominator = 15;
	sensor->xclk_current = MT9P012_XCLK_NOM_1;
	sensor->pix.width = MT9P012_VIDEO_WIDTH_4X_BINN_SCALED;
	sensor->pix.height = MT9P012_VIDEO_WIDTH_4X_BINN_SCALED;
	sensor->pix.pixelformat = V4L2_PIX_FMT_SGRBG10;

	printk("%s: sensor->v4l2_int_device = &mt9p012_int_device\n",__func__);
	sensor->v4l2_int_device = &mt9p012_int_device;
	sensor->v4l2_int_device->priv = sensor;
	sensor->dev = &client->dev;
	printk("%s: i2c_set_clientdata\n",__func__);
	i2c_set_clientdata(client, sensor);

	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err) {
		printk("%s: error v4l2_int_device_register\n",__func__);
		goto on_err2;
	}

	return 0;
on_err2:
	i2c_set_clientdata(client, NULL);
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

