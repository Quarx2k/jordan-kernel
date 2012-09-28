/*
 * drivers/media/video/ov5650_reg.h
 *
 * Register definitions for the OV5650 CameraChip.
 *
 * Author: Pallavi Kulkarni (ti.com)
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/* #define SENSOR_OV5650_DEBUG */

#ifdef SENSOR_OV5650_DEBUG
#define DPRINTK_OV5650(format, ...)\
	printk(KERN_INFO "OV5650: " format, ## __VA_ARGS__)
#else
#define DPRINTK_OV5650(format, ...)
#endif

#ifndef OV5650_REG_H
#define OV5650_REG_H

/* Register initialization tables for ov5650 */
/* Terminating list entry for reg */
#define OV5650_REG_TERM			0xFFFF
/* Terminating list entry for val */
#define OV5650_VAL_TERM			0xFF

#define OV5650_USE_XCLKA		0
#define OV5650_USE_XCLKB		1

#define OV5650_CSI2_VIRTUAL_ID		0x1

#define DEBUG_BASE			0x08000000

#define VAUX_2_8_V			0x09
#define VAUX_1_8_V			0x05
#define VAUX_DEV_GRP_P1	0x20
#define VAUX_DEV_GRP_NONE		0x00

/* Sensor specific GPIO signals */
#define OV5650_RESET_GPIO		98
#define OV5650_STANDBY_GPIO		64

/* FPS Capabilities */
#define OV5650_MIN_FPS			1
#define OV5650_DEF_FPS			15
#define OV5650_MAX_FPS			120

/* Frame Delays */
#define OV5650_GAIN_FRAME_DELAY 	1
#define OV5650_EXP_TIME_FRAME_DELAY 	2

#define SENSOR_DETECTED			1
#define SENSOR_NOT_DETECTED		0

/* XCLK Frequency in Hz*/
#define OV5650_XCLK_MIN			27000000
#define OV5650_XCLK_MAX			27000000

#define OV5650_MAX_TOTAL_VERT_SIZE 	0xFFF8
#define OV5650_MAX_TOTAL_HORZ_SIZE 	0x1FF0

/* Gain Values (linear, Q8) */
#define OV5650_MIN_LINEAR_GAIN		((u16)(1.25 * 256))
#define OV5650_MAX_LINEAR_GAIN		((u16)(31.0 * 256))

/* Exposure time values (usecs)*/
#define OV5650_MIN_EXPOSURE		100
#define OV5650_MAX_EXPOSURE		1000000


/* Product ID */
#define OV5650_PID			0x5650
/* Max supported Rev */
#define OV5650_REV			1

/* ------------------ Register defines ------------------ */
/*
 * System Control Registers
 */
#define SYSTEM_CONTROL0				0x3008
#define OV5650_CHIP_ID_H			0x300A
#define OV5650_CHIP_ID_L			0x300B

#define OV5650_PLL_CTRL_00			0x300F
#define OV5650_PLL_CTRL_00_R_SELD5_SHIFT 6
#define OV5650_PLL_CTRL_00_R_SELD5_MASK (0x3 << \
	OV5650_PLL_CTRL_00_R_SELD5_SHIFT)
#define OV5650_PLL_CTRL_00_R_DIVL_SHIFT 2
#define OV5650_PLL_CTRL_00_R_DIVL_MASK (0x1 << \
	OV5650_PLL_CTRL_00_R_DIVL_SHIFT)
#define OV5650_PLL_CTRL_00_R_SELD2P5_SHIFT 0
#define OV5650_PLL_CTRL_00_R_SELD2P5_MASK (0x3 << \
	OV5650_PLL_CTRL_00_R_SELD2P5_SHIFT)

#define OV5650_PLL_CTRL_01			0x3010
#define OV5650_PLL_CTRL_01_R_DIVS_SHIFT 4
#define OV5650_PLL_CTRL_01_R_DIVS_MASK (0xF << \
	OV5650_PLL_CTRL_01_R_DIVS_SHIFT)
#define OV5650_PLL_CTRL_01_R_DIVM_SHIFT 0
#define OV5650_PLL_CTRL_01_R_DIVM_MASK (0xF << \
	OV5650_PLL_CTRL_01_R_DIVM_SHIFT)

#define OV5650_PLL_CTRL_02			0x3011
#define OV5650_PLL_CTRL_02_R_DIVP_SHIFT 0
#define OV5650_PLL_CTRL_02_R_DIVP_MASK (0x3F << \
	OV5650_PLL_CTRL_02_R_DIVP_SHIFT)

#define OV5650_PLL_CTRL_03			0x3012
#define OV5650_PLL_CTRL_03_R_PREDIV_SHIFT 0
#define OV5650_PLL_CTRL_03_R_PREDIV_MASK (0x7 << \
	OV5650_PLL_CTRL_03_R_PREDIV_SHIFT)

#define OV5650_PAD_OUTPUT_ENBL_0 		0x3016
#define OV5650_PAD_OE_STRB_EN_SHIFT 1

#define OV5650_PAD_OUTPUT_0	 		0x3019
#define OV5650_PAD_OUTPUT_0_STROBE_SHIFT 1

#define OV5650_PAD_SELECT_0	 		0x301C
#define OV5650_PAD_SELECT_0_IO_STRB_SEL_SHIFT 1

#define OV5650_LONG_EXPO_H			0x3500
#define OV5650_LONG_EXPO_M			0x3501
#define OV5650_LONG_EXPO_L			0x3502
#define OV5650_AGC_ADJ				0x350B

#define OV5650_ARRAY_CTRL_01			0x3621
#define OV5650_ARRAY_CTRL_01_HORZ_BIN_SUB_EN_SHIFT 7
#define OV5650_ARRAY_CTRL_01_HORZ_BIN_SUB_EN_MASK (0x1 << \
	OV5650_ARRAY_CTRL_01_HORZ_BIN_SUB_EN_SHIFT)
#define OV5650_ARRAY_CTRL_01_HORZ_SUB_EN_SHIFT 6
#define OV5650_ARRAY_CTRL_01_HORZ_SUB_EN_MASK (0x1 << \
	OV5650_ARRAY_CTRL_01_HORZ_SUB_EN_SHIFT)
#define OV5650_ANALOG_CTRL_D			0x370D
#define OV5650_ANALOG_CTRL_D_VBIN_EN_SHIFT 6
#define OV5650_ANALOG_CTRL_D_VBIN_EN_MASK (0x1 << \
	OV5650_ANALOG_CTRL_D_VBIN_EN_SHIFT)

#define OV5650_HREF_START_H			0x3800
#define OV5650_HREF_START_L			0x3801
#define OV5650_VREF_START_H			0x3802
#define OV5650_VREF_START_L			0x3803
#define OV5650_HREF_WIDTH_H			0x3804
#define OV5650_HREF_WIDTH_L			0x3805
#define OV5650_VREF_WIDTH_H			0x3806
#define OV5650_VREF_WIDTH_L			0x3807
#define OV5650_H_OUTPUT_SIZE_H			0x3808
#define OV5650_H_OUTPUT_SIZE_L			0x3809
#define OV5650_V_OUTPUT_SIZE_H			0x380A
#define OV5650_V_OUTPUT_SIZE_L			0x380B
#define OV5650_TOTAL_HORZ_SIZE_H		0x380C
#define OV5650_TOTAL_HORZ_SIZE_L		0x380D
#define OV5650_TOTAL_VERT_SIZE_H		0x380E
#define OV5650_TOTAL_VERT_SIZE_L		0x380F

#define OV5650_TIMING_TC_18			0x3818
#define OV5650_TIMING_TC_18_MIRROR_SHIFT  6
#define OV5650_TIMING_TC_18_MIRROR_MASK (0x1 << \
	OV5650_TIMING_TC_18_MIRROR_SHIFT)
#define OV5650_TIMING_TC_18_VFLIP_SHIFT  5
#define OV5650_TIMING_TC_18_VFLIP_MASK (0x1 << \
	OV5650_TIMING_TC_18_VFLIP_SHIFT)
#define OV5650_TIMING_TC_18_VSUB4_SHIFT  1
#define OV5650_TIMING_TC_18_VSUB4_MASK (0x1 << \
	OV5650_TIMING_TC_18_VSUB4_SHIFT)
#define OV5650_TIMING_TC_18_VSUB2_SHIFT  0
#define OV5650_TIMING_TC_18_VSUB2_MASK (0x1 << \
	OV5650_TIMING_TC_18_VSUB2_SHIFT)

#define OV5650_HREF_START_MAN_H			0x3824
#define OV5650_HREF_START_MAN_H_HVS_MAN_SHIFT 5
#define OV5650_HREF_START_MAN_H_HVS_MAN_MASK (0x1 << \
	OV5650_HREF_START_MAN_H_HVS_MAN_SHIFT)
#define OV5650_HREF_START_MAN_H_SHIFT 0
#define OV5650_HREF_START_MAN_H_MASK (0x1F << \
	OV5650_HREF_START_MAN_H_SHIFT)
#define OV5650_HREF_START_MAN_L			0x3825
#define OV5650_VREF_START_MAN_H			0x3826
#define OV5650_VREF_START_MAN_L			0x3827

#define OV5650_STROBE_CTRL			0x3B00
#define OV5650_STROBE_CTRL_STRB_EN_SHIFT  7
#define OV5650_STROBE_CTRL_STRB_PLS_WIDTH_SHIFT  2
#define OV5650_STROBE_CTRL_STRB_MODE_SHIFT  0

#define OV5650_5060HZ_CTRL_01			0x3C01
#define OV5650_5060HZ_CTRL_01_DEBUG_SHIFT  7
#define OV5650_5060HZ_CTRL_01_DEBUG_MASK (0x1 << \
	OV5650_5060HZ_CTRL_01_DEBUG_SHIFT)
#define OV5650_5060HZ_CTRL_0C			0x3C0C
#define OV5650_5060HZ_CTRL_0C_BAND50_SHIFT  0
#define OV5650_5060HZ_CTRL_0C_BAND50_MASK (0x1 << \
	OV5650_5060HZ_CTRL_0C_BAND50_SHIFT)
/*
 * END - System Control Registers
 */

/*
 * OTP Registers
 */
#define OV5650_OTP_SUB_ADDR			0x3D00
#define OV5650_OTP_DATA				0x3D04
/*
 * END - OTP Registers
 */

/*
 * MIPI Control Registers
 */
#define OV5650_MIPI_CTRL14			0x4814
/*
 * END - MIPI Control Registers
 */

/*
 * ISP Registers
 */
#define OV5650_ISP_CTRL_00			0x5000
#define OV5650_ISP_CTRL_00_LENC_EN_SHIFT  7
#define OV5650_ISP_CTRL_00_LENC_EN_MASK (0x1 << \
	OV5650_ISP_CTRL_00_LENC_EN_SHIFT)
#define OV5650_ISP_CTRL_00_BC_EN_SHIFT  2
#define OV5650_ISP_CTRL_00_BC_EN_MASK (0x1 << \
	OV5650_ISP_CTRL_00_BC_EN_SHIFT)
#define OV5650_ISP_CTRL_00_WC_EN_SHIFT  1
#define OV5650_ISP_CTRL_00_WC_EN_MASK (0x1 << \
	OV5650_ISP_CTRL_00_WC_EN_SHIFT)
#define OV5650_ISP_CTRL_02			0x5002
#define OV5650_ISP_CTRL_02_VAP_EN_SHIFT  1
#define OV5650_ISP_CTRL_02_VAP_EN_MASK (0x1 << \
	OV5650_ISP_CTRL_02_VAP_EN_SHIFT)
#define OV5650_ISP_CTRL_3D			0x503D
#define OV5650_ISP_CTRL_3D_TEST_PATT_EN_SHIFT  7
#define OV5650_ISP_CTRL_3D_TEST_PATT_EN_MASK (0x1 << \
	OV5650_ISP_CTRL_3D_TEST_PATT_EN_SHIFT)
#define OV5650_ISP_CTRL46			0x5046
#define OV5650_RESERVED_505A			0x505A
#define OV5650_RESERVED_505B			0x505B
/*
 * END - ISP Registers
 */

/*
 * VAP Registers
 */
#define OV5650_VAP_CTRL_01			0x5901
#define OV5650_VAP_CTRL_01_HSUB_COEF_SHIFT  2
#define OV5650_VAP_CTRL_01_HSUB_COEF_MASK (0x3 << \
	OV5650_VAP_CTRL_01_HSUB_COEF_SHIFT)
/*
 * END - VAP Registers
 */

/* ------------------------------------------------------ */

int ov5650_write_reg(struct i2c_client *client, u16 reg, u8 val);

/* Exposure time values */
#define DEF_MIN_EXPOSURE	250
#define DEF_MAX_EXPOSURE	128000
#define DEF_EXPOSURE	    	33000
#define EXPOSURE_STEP	    	50

#define DEF_LINEAR_GAIN		(2*256)
#define LINEAR_GAIN_STEP	0x1

struct ov5650_sensor_regif {
	u16 len;
	u32 addr;
	u32 val;
} ;

enum ov5650_orientation {
	OV5650_NO_HORZ_FLIP_OR_VERT_FLIP = 0,
	OV5650_HORZ_FLIP_ONLY,
	OV5650_VERT_FLIP_ONLY,
	OV5650_HORZ_FLIP_AND_VERT_FLIP
};

enum ov5650_shutter_type {
	ROLLING_SHUTTER_TYPE = 0,
	MECH_SHUTTER_TYPE
};

enum ov5650_flash_type {
	LED_FLASH_TYPE = 0,
	XENON_FLASH_TYPE
};

/* define a structure for ov5650 register initialization values */
struct ov5650_reg {
	unsigned int reg;
	unsigned char val;
};

struct capture_size_ov {
	unsigned long width;
	unsigned long height;
};

/*
 * struct ov5650_clk_settings - struct for storage of sensor
 * clock settings
 * @pre_div_q1: pre-pll divider (2X actual)
 * @div_p: pll feedback divider (pll_multiplier)
 * @div_s: R_DIVS divider
 * @div_l: # mipi lanes divider
 * @div_m: R_DIVM divider
 * @seld2p5_q1: R_SELD2P5 divider (2X actual)
 * @seld5: R_SELD divider
 * @out_blk_div: output block divider
 */
struct ov5650_clk_settings {
	u16	pre_div_q1;
	u16	div_p;
	u16	div_s;
	u16	div_l;
	u16	div_m;
	u16	seld2p5_q1;
	u16	seld5;
	u16	out_blk_div;
};

/*
 * struct ov5650_frame_settings - struct for storage of sensor
 * frame settings
 * @total_vert_size_min: min total number of lines in frame
 * @total_vert_size: total number of lines in frame
 * @total_horz_size_min: min total number of pixels in line
 * @total_horz_size: total number of pixels in line
 * @man_hv_ref_start; use manual HREF & VREF start offset
 * @h_ref_start: horizontal start reference in pixels
 * @h_ref_width: horizontal width in pixels
 * @v_ref_start: vertical start reference in lines
 * @v_ref_width: vertical width in lines
 * @h_output_size: horizontal output size in pixels
 * @v_output_size: vertical output size in lines
 * @h_subsample: horizontal subsample amount
 * @v_subsample: vertical subsample amount
 * @v_subsample: vertical subsample amount
 * @binning_sensitivity: sensitivity relative to no binning
 * @min_time_per_frame: fastest frame time possible
 */
struct ov5650_frame_settings {
	u16	total_vert_size_min;
	u16	total_vert_size;
	u16	total_horz_size_min;
	u16	total_horz_size;
	bool	man_hv_ref_start;
	u16	h_ref_start;
	u16	h_ref_width;
	u16	v_ref_start;
	u16	v_ref_width;
	u16	h_output_size;
	u16	v_output_size;
	u8	h_subsample;
	u8	v_subsample;
	u8	h_binning;
	u8	v_binning;
	u16	binning_sensitivity;
	struct v4l2_fract min_time_per_frame;
};

/*
 * struct ov5650_mipi_settings
 * @hs_settle: mipi high speed settle time (in DDR periods)
 */
struct ov5650_mipi_settings {
	u16	hs_settle;
};

/*
 * struct ov5650_sensor_settings - struct for storage of
 * sensor settings.
 */
struct ov5650_sensor_settings {
	struct	ov5650_clk_settings clk;
	struct	ov5650_frame_settings frame;
	struct	ov5650_mipi_settings mipi;
};

/* Array of image sizes supported by OV5650.  These must be ordered from
 * smallest image size to largest.
 */
const static struct capture_size_ov ov5650_sizes[] = {
	/* SIZE_80K */
	{ 324, 244 },
	/* SIZE_315K */
	{ 648, 486 },
	/* SIZE_1_25M */
	{ 1296, 972 },
	/* SIZE_2M (cropped) */
	{ 1824, 1216 },
	/* SIZE_5M */
	{ 2592, 1944 },
};

enum image_size_ov {
	SIZE_80K,
	SIZE_315K,
	SIZE_1_25M,
	SIZE_2M,
	SIZE_5M
};
enum pixel_format_ov {
	RAW10
};

#define OV_NUM_IMAGE_SIZES		5
#define OV_NUM_PIXEL_FORMATS		1

const static struct ov5650_reg ov5650_common[OV_NUM_IMAGE_SIZES][150] = {

	/* SIZE_324x244_Default settings */
	{
		/* MIPI Sensor Raw QVGA 94.5fps
		Two-lane 10Bit with 27MHz EXCLK and 75.6MHz */
		{0x3103, 0x93},
		{0x3b07, 0x0e},
		{0x3017, 0xff},
		{0x3018, 0xfc},
		{0x3613, 0xc4},
		{0x3703, 0x9a},
		{0x3605, 0x04},
		{0x3606, 0x3f},
		{0x3712, 0x13},
		{0x370b, 0x40},
		{0x3713, 0x92},
		{0x3714, 0x17},
		{0x3612, 0x1a},
		{0x3705, 0xdc},
		{0x370a, 0x83},
		{0x370c, 0xc8},
		{0x3710, 0x28},
		{0x3702, 0x3a},
		{0x3704, 0x18},
		{0x3a18, 0x00},
		{0x3a19, 0xf8},
		{0x3830, 0x50},
		{0x3a13, 0x54},
		{0x3815, 0x82},
		{0x5059, 0x80},
		{0x3a1a, 0x06},
		{0x3623, 0x01},
		{0x3633, 0x24},
		{0x4000, 0x05},
		{0x401d, 0x28},
		{0x4001, 0x02},
		{0x401c, 0x42},
		{0x5046, 0x01},
		{0x3810, 0x40},
		{0x3836, 0x41},
		{0x505f, 0x04},
		{0x5000, 0x00},
		{0x5001, 0x00},
		{0x585b, 0x2c},
		{0x585d, 0x93},
		{0x585f, 0x90},
		{0x5861, 0x0d},
		{0x5180, 0xc0},
		{0x5184, 0x00},
		{0x3603, 0xa7},
		{0x3615, 0x52},
		{0x3620, 0x56},
		{0x3631, 0x36},
		{0x3632, 0x5f},
		{0x3711, 0x24},
		{0x401f, 0x03},
		{0x3007, 0x3B},
		{0x300e, 0x0c},
		{0x3003, 0x01},
		{0x3503, 0x13},
		{0x3400, 0x08},
		{0x3402, 0x08},
		{0x3404, 0x08},
		{0x3406, 0x01},
		{0x4837, 0x57},  /* MIPI Pclk Period 2x */
		{OV5650_REG_TERM, OV5650_VAL_TERM},
	},

	/* SIZE_648x486_Default settings */
	{
		/* MIPI Sensor Raw VGA 70.02fps
		   Two-lane 10Bit with 27MHz EXCLK and 75.6MHz */
		{0x3103, 0x93},
		{0x3b07, 0x0e},
		{0x3017, 0xff},
		{0x3018, 0xfc},
		{0x3613, 0xc4},
		{0x3703, 0x9a},
		{0x3605, 0x04},
		{0x3606, 0x3f},
		{0x3712, 0x13},
		{0x370b, 0x40},
		{0x3713, 0x92},
		{0x3714, 0x17},
		{0x3612, 0x1a},
		{0x3705, 0xdc},
		{0x370a, 0x81},
		{0x370c, 0xc8},
		{0x3710, 0x28},
		{0x3702, 0x3a},
		{0x3704, 0x18},
		{0x3a18, 0x00},
		{0x3a19, 0xf8},
		{0x3830, 0x50},
		{0x3a13, 0x54},
		{0x3815, 0x82},
		{0x5059, 0x80},
		{0x3a1a, 0x06},
		{0x3623, 0x01},
		{0x3633, 0x24},
		{0x4000, 0x05},
		{0x401d, 0x28},
		{0x4001, 0x02},
		{0x401c, 0x42},
		{0x5046, 0x01},
		{0x3810, 0x40},
		{0x3836, 0x41},
		{0x505f, 0x04},
		{0x5000, 0x00},
		{0x5001, 0x00},
		{0x585b, 0x2c},
		{0x585d, 0x93},
		{0x585f, 0x90},
		{0x5861, 0x0d},
		{0x5180, 0xc0},
		{0x5184, 0x00},
		{0x3603, 0xa7},
		{0x3615, 0x50},
		{0x3620, 0x56},
		{0x3631, 0x36},
		{0x3632, 0x5f},
		{0x3711, 0x24},
		{0x401f, 0x03},
		{0x3007, 0x3B},
		{0x300e, 0x0c},
		{0x3003, 0x01},
		{0x3503, 0x13},
		{0x3400, 0x08},
		{0x3402, 0x08},
		{0x3404, 0x08},
		{0x3406, 0x01},
		{0x4837, 0x35},  /* MIPI Pclk Period 2x */
		{OV5650_REG_TERM, OV5650_VAL_TERM},
	},

	/* SIZE_1296x972 Default settings */
	{
		/* MIPI Sensor Raw 1296x972 36.03fps
		   Two-lane 10Bit with 27MHz EXCLK and 75.6MHz */
		{0x3103, 0x93},
		{0x3b07, 0x0e},
		{0x3017, 0x7f},
		{0x3018, 0xfc},
		{0x3703, 0x9a},
		{0x3605, 0x04},
		{0x3606, 0x3f},
		{0x3712, 0x13},
		{0x370b, 0x40},
		{0x3713, 0x92},
		{0x3714, 0x17},
		{0x3612, 0x1a},
		{0x3705, 0xdb},
		{0x370a, 0x81},
		{0x370c, 0x00},
		{0x3710, 0x28},
		{0x3702, 0x3a},
		{0x3704, 0x18},
		{0x3a18, 0x00},
		{0x3a19, 0xf8},
		{0x3830, 0x50},
		{0x3a13, 0x54},
		{0x3815, 0x82},
		{0x5059, 0x80},
		{0x3a1a, 0x06},
		{0x3623, 0x01},
		{0x3633, 0x24},
		{0x401c, 0x42},
		{0x4000, 0x05},
		{0x401d, 0x28},
		{0x4001, 0x02},
		{0x5046, 0x01},
		{0x3810, 0x40},
		{0x3836, 0x41},
		{0x5000, 0x00},
		{0x5001, 0x00},
		{0x585b, 0x2c},
		{0x585d, 0x93},
		{0x585f, 0x90},
		{0x5861, 0x0d},
		{0x5180, 0xc0},
		{0x5184, 0x00},
		{0x3603, 0xa7},
		{0x3615, 0x52},
		{0x3620, 0x56},
		{0x381a, 0x3c},
		{0x3631, 0x36},
		{0x3632, 0x5f},
		{0x3711, 0x24},
		{0x401f, 0x03},
		{0x3007, 0x3B},
		{0x300e, 0x0c},
		{0x3003, 0x01},
		{0x3503, 0x13},
		{0x3400, 0x08},
		{0x3402, 0x08},
		{0x3404, 0x08},
		{0x3406, 0x01},
		{0x4837, 0x1A},  /* MIPI Pclk Period 2x */
		{OV5650_REG_TERM, OV5650_VAL_TERM},
	},

	/* SIZE_1824x1216 Default settings - TRIMMED */
	{
		/* MIPI Sensor Raw 1824x1216 30fps
		   Two-lane 10Bit with 27MHz EXCLK and 97.2MHz */
		{0x3103, 0x93},
		{0x3007, 0x3B},
		{0x3706, 0x41},
		{0x3703, 0xe6},
		{0x3613, 0x44},
		{0x3605, 0x04},
		{0x3606, 0x3f},
		{0x3712, 0x13},
		{0x370b, 0x40},
		{0x3600, 0x54},
		{0x3601, 0x05},
		{0x3713, 0x22},
		{0x3714, 0x27},
		{0x3631, 0x36},
		{0x3612, 0x1a},
		{0x3604, 0x40},
		{0x3705, 0xda},
		{0x370c, 0x00},
		{0x3710, 0x28},
		{0x3702, 0x3a},
		{0x3704, 0x18},
		{0x3603, 0xa7},
		{0x3615, 0x52},
		{0x3632, 0x5f},
		{0x3711, 0x24},
		{0x3620, 0x56},
		{0x3621, 0x2f},
		{0x370A, 0x80},
		{0x3a1a, 0x06},
		{0x3a13, 0x54},
		{0x3830, 0x50},
		{0x3810, 0x40},
		{0x3836, 0x41},
		{0x401c, 0x46},
		{0x4000, 0x05},
		{0x4001, 0x02},
		{0x401d, 0x28},
		{0x5000, 0x00},
		{0x5001, 0x00},
		{0x5046, 0x01},
		{0x300e, 0x0c},
		{0x3815, 0x82},
		{0x381a, 0x1c},
		{0x381c, 0x31},
		{0x381d, 0x6c},
		{0x381e, 0x04},
		{0x381f, 0xd0},
		{0x3820, 0x03},
		{0x3821, 0x19},
		{0x3623, 0x01},
		{0x3633, 0x24},
		{0x3a18, 0x00},
		{0x3a19, 0xf8},
		{0x3b07, 0x0e},
		{0x300D, 0x21},
		{0x401f, 0x03},
		{0x3503, 0x13},
		{0x3003, 0x01},
		{0x4837, 0x1A},  /* MIPI Pclk Period 2x */
		{OV5650_REG_TERM, OV5650_VAL_TERM},
	},

	/* SIZE_2592x1944 Default settings */
	{
		/* MIPI Sensor Raw QSXGA 2592x1944 11.81fps
		   Two-lane 10Bit with 27MHz EXCLK, SYSCLK = 75.6MHz */
		{0x3103, 0x93},
		{0x3b07, 0x0e},
		{0x3017, 0xff},
		{0x3018, 0xfc},
		{0x3703, 0xeb},
		{0x3605, 0x04},
		{0x3606, 0x3f},
		{0x3712, 0x13},
		{0x370b, 0x40},
		{0x3713, 0x22},
		{0x3714, 0x27},
		{0x3612, 0x1a},
		{0x370c, 0x00},
		{0x3710, 0x28},
		{0x3702, 0x3a},
		{0x3704, 0x18},
		{0x3a18, 0x00},
		{0x3a19, 0xf8},
		{0x3830, 0x50},
		{0x3a13, 0x54},
		{0x3815, 0x82},
		{0x5059, 0x80},
		{0x3a1a, 0x06},
		{0x3623, 0x01},
		{0x3633, 0x24},
		{0x4000, 0x05},
		{0x401d, 0x28},
		{0x4001, 0x02},
		{0x401c, 0x46},
		{0x5046, 0x01},
		{0x3810, 0x40},
		{0x3836, 0x41},
		{0x505f, 0x04},
		{0x5000, 0x00},
		{0x5001, 0x00},
		{0x585b, 0x2c},
		{0x585d, 0x93},
		{0x585f, 0x90},
		{0x5861, 0x0d},
		{0x5180, 0xc0},
		{0x5184, 0x00},
		{0x3603, 0xa7},
		{0x3615, 0x50},
		{0x3620, 0x56},
		{0x381a, 0x3c},
		{0x3631, 0x36},
		{0x3632, 0x5f},
		{0x3711, 0x24},
		{0x401f, 0x03},
		{0x3007, 0x3B},
		{0x300e, 0x0c},
		{0x3003, 0x01},
		{0x3503, 0x13},
		{0x3400, 0x08},
		{0x3402, 0x08},
		{0x3404, 0x08},
		{0x3406, 0x01},
		{0x4837, 0x1A},  /* MIPI Pclk Period 2x */
		/*
		{0x401d, 0x08},  // TEMP - disable BLC frame averaging
		*/
		{OV5650_REG_TERM, OV5650_VAL_TERM},
	},
};

/* 50-60 Hz Detection settings */
const static struct ov5650_reg ov5650_50_60_hz_detect_tbl[] = {
	{0x3c01, 0xB4},  /* Start in manual, switch to auto after streaming */
	{0x3c04, 0x28},
	{0x3c05, 0x98},
	{0x3c07, 0x07},
	{0x3c09, 0xc2},
	{0x3c0a, 0xaf},  /* 27Mhz */
	{0x3c0b, 0xc8},  /* 27Mhz */
	/* read 3c0c[0], 1=50hz, 0=60hz */
	{OV5650_REG_TERM, OV5650_VAL_TERM},
};

/* Enable Bright Center Compensation settings */
const static struct ov5650_reg ov5650_enable_bright_center_comp_tbl[] = {
	{0x3212, 0x00},  /* Group 0 begins */
	{0x370e, 0x20},
	{0x3707, 0x62},
	{0x3630, 0x62},
	{0x3212, 0x10},  /* Group 0 ends */
	{0x3212, 0xa0},  /* Launch Group 0 */
	{OV5650_REG_TERM, OV5650_VAL_TERM},
};

/* Disable Bright Center Compensation settings */
const static struct ov5650_reg ov5650_disable_bright_center_comp_tbl[] = {
	{0x3212, 0x00},  /* Group 0 begins */
	{0x370e, 0x00},
	{0x3707, 0x22},
	{0x3630, 0x22},
	{0x3212, 0x10},  /* Group 0 ends */
	{0x3212, 0xa0},  /* Launch Group 0 */
	{OV5650_REG_TERM, OV5650_VAL_TERM},
};

/* Lens correction settings */
const static struct ov5650_reg len_correction_tbl[] = {
	/* G */
	{0x5800, 0x33},
	{0x5801, 0x1a},
	{0x5802, 0x14},
	{0x5803, 0x13},
	{0x5804, 0x1b},
	{0x5805, 0x38},
	{0x5806, 0x12},
	{0x5807, 0x0a},
	{0x5808, 0x07},
	{0x5809, 0x06},
	{0x580a, 0x0a},
	{0x580b, 0x10},
	{0x580c, 0x0c},
	{0x580d, 0x04},
	{0x580e, 0x00},
	{0x580f, 0x00},
	{0x5810, 0x04},
	{0x5811, 0x0b},
	{0x5812, 0x0d},
	{0x5813, 0x04},
	{0x5814, 0x00},
	{0x5815, 0x00},
	{0x5816, 0x05},
	{0x5817, 0x0c},
	{0x5818, 0x13},
	{0x5819, 0x0c},
	{0x581a, 0x08},
	{0x581b, 0x08},
	{0x581c, 0x0c},
	{0x581d, 0x12},
	{0x581e, 0x3a},
	{0x581f, 0x1a},
	{0x5820, 0x15},
	{0x5821, 0x15},
	{0x5822, 0x1c},
	{0x5823, 0x37},

	/* B */
	{0x5824, 0x0e},
	{0x5825, 0x0c},
	{0x5826, 0x0e},
	{0x5827, 0x0e},
	{0x5828, 0x0e},
	{0x5829, 0x0d},
	{0x582a, 0x0e},
	{0x582b, 0x0e},
	{0x582c, 0x0f},
	{0x582d, 0x09},
	{0x582e, 0x11},
	{0x582f, 0x10},
	{0x5830, 0x11},
	{0x5831, 0x10},
	{0x5832, 0x0b},
	{0x5833, 0x0b},
	{0x5834, 0x10},
	{0x5835, 0x10},
	{0x5836, 0x11},
	{0x5837, 0x0a},
	{0x5838, 0x0d},
	{0x5839, 0x09},
	{0x583a, 0x09},
	{0x583b, 0x09},
	{0x583c, 0x06},

	/* R */
	{0x583d, 0x1f},
	{0x583e, 0x1b},
	{0x583f, 0x1b},
	{0x5840, 0x1b},
	{0x5841, 0x18},
	{0x5842, 0x1a},
	{0x5843, 0x15},
	{0x5844, 0x12},
	{0x5845, 0x14},
	{0x5846, 0x1b},
	{0x5847, 0x17},
	{0x5848, 0x10},
	{0x5849, 0x0f},
	{0x584a, 0x10},
	{0x584b, 0x17},
	{0x584c, 0x19},
	{0x584d, 0x14},
	{0x584e, 0x12},
	{0x584f, 0x14},
	{0x5850, 0x1a},
	{0x5851, 0x1a},
	{0x5852, 0x1c},
	{0x5853, 0x1b},
	{0x5854, 0x1a},
	{0x5855, 0x18},
	{OV5650_REG_TERM, OV5650_VAL_TERM},
};

const static struct ov5650_reg ov5650_common_csi2[] = {
	{0x4801, 0x0f},
	{0x4803, 0x50},
	{OV5650_REG_TERM, OV5650_VAL_TERM},
};

#endif /* ifndef OV5650_REG_H */

