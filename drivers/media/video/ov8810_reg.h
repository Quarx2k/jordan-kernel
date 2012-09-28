/*
 * drivers/media/video/ov8810.h
 *
 * Register definitions for the OV8810 CameraChip.
 *
 * Author: Pallavi Kulkarni (ti.com)
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef OV8810_REG_H
#define OV8810_REG_H

/* #define SENSOR_OV8810_DEBUG */

#ifdef SENSOR_OV8810_DEBUG
#define DPRINTK_OV8810(format, ...)\
	printk(KERN_INFO "OV8810: " format, ## __VA_ARGS__)
#else
#define DPRINTK_OV8810(format, ...)
#endif

#define OV8810_I2C_ADDR		(0x6c >> 1)

/* ISP uses a 10-bit value, OV8810 uses a 12-bit value */
#define OV8810_BLACK_LEVEL_10BIT	8

/* Register initialization tables for ov8810 */
/* Terminating list entry for reg */
#define OV8810_REG_TERM		0xFFFF
/* Terminating list entry for val */
#define OV8810_VAL_TERM		0xFF

#define OV8810_USE_XCLKA	0
#define OV8810_USE_XCLKB	1

#define OV8810_CSI2_VIRTUAL_ID	0x1

#define DEBUG_BASE		0x08000000

#define VAUX_2_8_V		0x09
#define VAUX_1_8_V		0x05
#define VAUX_DEV_GRP_P1	0x20
#define VAUX_DEV_GRP_NONE	0x00

/* Sensor specific GPIO signals */
#define OV8810_RESET_GPIO	98
#define OV8810_STANDBY_GPIO	64

/* FPS Capabilities */
#define OV8810_MIN_FPS			3
#define OV8810_DEF_FPS			15
#define OV8810_MAX_FPS			116

/* Frame Delays */
#define OV8810_GAIN_FRAME_DELAY 1
#define OV8810_EXP_TIME_FRAME_DELAY 2

#define SENSOR_DETECTED		1
#define SENSOR_NOT_DETECTED	0

/* XCLK Frequency in Hz*/
#define OV8810_XCLK_MIN		27000000
#define OV8810_XCLK_MAX		27000000

#define OV8810_MAX_FRAME_LENGTH_LINES 0xFFF8
#define OV8810_MAX_LINE_LENGTH_PCK 0xFFF0

/* Gain Values (linear, Q8) */
#define OV8810_MIN_LINEAR_GAIN	((u16)(1.2 * 256))
#define OV8810_MAX_LINEAR_GAIN	((u16)(31.0 * 256))

/* Exposure time values (usecs)*/
#define OV8810_MIN_EXPOSURE	100
#define OV8810_MAX_EXPOSURE	1000000


/* Product ID */
#define OV8810_PID			0x8810
/* Max supported Rev */
#define OV8810_REV			3

/* ------------------ Register defines ------------------ */
/*
 * System Control Registers
 */
#define OV8810_AGCL				0x3000
#define OV8810_AECL_H				0x3002
#define OV8810_AECL_L				0x3003
#define OV8810_PIDH				0x300A
#define OV8810_PIDL				0x300B
#define OV8810_R_PLL1				0x300E
#define OV8810_R_PLL1_VT_SYS_DIV_SHIFT 4
#define OV8810_R_PLL1_VT_SYS_DIV_MASK (0xF << \
	OV8810_R_PLL1_VT_SYS_DIV_SHIFT)
#define OV8810_R_PLL1_DIV8_MASK 	0x7
#define OV8810_R_PLL2				0x300F
#define OV8810_R_PLL2_OP_SYS_DIV_SHIFT 4
#define OV8810_R_PLL2_OP_SYS_DIV_MASK (0xF << \
	OV8810_R_PLL2_OP_SYS_DIV_SHIFT)
#define OV8810_R_PLL2_OP_PIX_DIV_MASK 	0xF
#define OV8810_R_PLL3				0x3010
#define OV8810_R_PLL3_PLL_MULT_MASK 	0x7F
#define OV8810_R_PLL4				0x3011
#define OV8810_R_PLL4_PRE_DIV_MASK 	0xF

#define OV8810_SYS				0x3012
#define OV8810_ADDVS_H				0x301E
#define OV8810_ADDVS_L				0x301F
#define OV8810_FRM_LEN_LINES_H			0x3020
#define OV8810_FRM_LEN_LINES_L			0x3021
#define OV8810_LINE_LEN_PCK_H			0x3022
#define OV8810_LINE_LEN_PCK_L			0x3023
#define OV8810_X_ADDR_START_H			0x3024
#define OV8810_X_ADDR_START_L			0x3025
#define OV8810_Y_ADDR_START_H			0x3026
#define OV8810_Y_ADDR_START_L			0x3027
#define OV8810_X_ADDR_END_H			0x3028
#define OV8810_X_ADDR_END_L			0x3029
#define OV8810_Y_ADDR_END_H			0x302A
#define OV8810_Y_ADDR_END_L			0x302B
#define OV8810_X_OUTPUT_SIZE_H			0x302C
#define OV8810_X_OUTPUT_SIZE_L			0x302D
#define OV8810_Y_OUTPUT_SIZE_H			0x302E
#define OV8810_Y_OUTPUT_SIZE_L			0x302F

#define OV8810_5060HZ_CTRL			0x303D
#define OV8810_5060HZ_CTRL_BAND50_SHIFT  0
#define OV8810_5060HZ_CTRL_BAND50_MASK (0x1 << \
	OV8810_5060HZ_CTRL_BAND50_SHIFT)

#define OV8810_RESERVED_3058			0x3058
#define OV8810_IO_CTRL2				0x30B2
#define OV8810_DSIO0				0x30B3
#define OV8810_DSIO0_RPCLK_DIV_MASK 	0x3
#define OV8810_FRS0				0x30B7
#define OV8810_RESERVED_30E1			0x30E1
#define OV8810_FRS1				0x30E4
#define OV8810_FRS2				0x30E5
#define OV8810_FRS3				0x30E6
#define OV8810_FRS4				0x30E7
#define OV8810_FRS4_STRB_SOURCE_SEL_SHIFT 1
#define OV8810_FRS5				0x30E8
#define OV8810_FRS5_ROLLING_SHUT_STRB_EN_SHIFT 7
#define OV8810_FRS5_STRB_PLS_WIDTH_SHIFT 2
#define OV8810_FRS5_STROBE_MODE_SHIFT 0
#define OV8810_FRS6				0x30EA
#define OV8810_FRS7				0x30EB
#define OV8810_IMAGE_SYSTEM			0x30FA
#define OV8810_IMAGE_TRANSFORM			0x30F8
#define OV8810_IMAGE_TRANSFORM_HSUB_MASK (0x3)
#define OV8810_IMAGE_TRANSFORM_VSUB_SHIFT 2
#define OV8810_IMAGE_TRANSFORM_VSUB_MASK (0x3 << \
	OV8810_IMAGE_TRANSFORM_VSUB_SHIFT)
#define OV8810_IMAGE_TRANSFORM_HMIRROR_SHIFT 6
#define OV8810_IMAGE_TRANSFORM_HMIRROR_MASK (0x1 << \
	OV8810_IMAGE_TRANSFORM_HMIRROR_SHIFT)
#define OV8810_IMAGE_TRANSFORM_VFLIP_SHIFT 7
#define OV8810_IMAGE_TRANSFORM_VFLIP_MASK (0x1 << \
	OV8810_IMAGE_TRANSFORM_VFLIP_SHIFT)
#define OV8810_GROUP_WR				0x30FF
#define OV8810_ISP_ENBL_0			0x3300
#define OV8810_ISP_ENBL_0_BLC_EN_SHIFT 0
#define OV8810_ISP_ENBL_0_EVEN_ODD_EN_SHIFT 1
#define OV8810_ISP_ENBL_0_BC_EN_SHIFT 2
#define OV8810_ISP_ENBL_0_WC_EN_SHIFT 3
#define OV8810_ISP_ENBL_0_LENC_EN_SHIFT 4
#define OV8810_ISP_ENBL_0_AWB_GAIN_EN_SHIFT 5
#define OV8810_ISP_ENBL_0_AWB_STAT_EN_SHIFT 6
#define OV8810_ISP_ENBL_0_ISP_EN_SHIFT 7
#define OV8810_CBAR	 			0x3303
#define OV8810_DIG_GAIN 			0x3309
#define OV8810_SIZE_H0 				0x3316
#define OV8810_LENC				0x33E4
#define OV8810_DVP_CTRL08			0x3508
#define OV8810_DVP_CTRL0E			0x350E

/* len correction */
#define LENC_1_1_DOWNSAMPLING 0x02
#define LENC_2_1_DOWNSAMPLING 0x07
#define LENC_4_1_DOWNSAMPLING 0x0b
#define LENC_8_1_DOWNSAMPLING 0x0f

/*
 * END - System Control Registers
 */


/*
 * OUT_TOP Registers
 */

#define OV8810_MIPI_CTRL01			0x3601

#define OV8810_MIPI_CTRL02			0x3602
#define OV8810_MIPI_CTRL02_VIRTUALCH_ID_MASK	(0x3 << 6)


#define OV8810_MIPI_CTRL0A			0x360A
#define OV8810_MIPI_CTRL0B			0x360B
#define OV8810_MIPI_CTRL0B_DSBL_DATA_LANE_2_MASK  0x1
#define OV8810_MIPI_CTRL0B_DSBL_DATA_LANE_1_MASK  0x2

#define OV8810_MIPI_CTRL14			0x3614
#define OV8810_MIPI_CTRL14_MIN_HS_ZERO_NUI_SHIFT	2
#define OV8810_MIPI_CTRL14_MIN_HS_ZERO_NUI_MASK		(0x3F << \
				OV8810_MIPI_CTRL14_MIN_HS_ZERO_NUI_SHIFT)
#define OV8810_MIPI_CTRL14_MIN_HS_ZERO_H_MASK	0x3

#define OV8810_MIPI_CTRL15			0x3615
#define OV8810_MIPI_CTRL15_MIN_HS_ZERO_L_MASK	0xFF

#define OV8810_MIPI_CTRL16			0x3616
#define OV8810_MIPI_CTRL16_MIN_HS_TRAIL_NUI_SHIFT	2
#define OV8810_MIPI_CTRL16_MIN_HS_TRAIL_NUI_MASK	(0x3F << \
				OV8810_MIPI_CTRL16_MIN_HS_TRAIL_NUI_SHIFT)
#define OV8810_MIPI_CTRL16_MIN_HS_TRAIL_H_MASK	0x3

#define OV8810_MIPI_CTRL17			0x3617
#define OV8810_MIPI_CTRL17_MIN_HS_TRAIL_L_MASK	0xFF

#define OV8810_MIPI_CTRL18			0x3618
#define OV8810_MIPI_CTRL18_MIN_CLK_ZERO_NUI_SHIFT	2
#define OV8810_MIPI_CTRL18_MIN_CLK_ZERO_NUI_MASK	(0x3F << \
				OV8810_MIPI_CTRL18_MIN_CLK_ZERO_NUI_SHIFT)
#define OV8810_MIPI_CTRL18_MIN_CLK_ZERO_H_MASK	0x3

#define OV8810_MIPI_CTRL19			0x3619
#define OV8810_MIPI_CTRL19_MIN_CLK_ZERO_L_MASK	0xFF

#define OV8810_MIPI_CTRL1A			0x361A
#define OV8810_MIPI_CTRL1A_MIN_CLK_PREPARE_NUI_SHIFT	2
#define OV8810_MIPI_CTRL1A_MIN_CLK_PREPARE_NUI_MASK		(0x3F << \
				OV8810_MIPI_CTRL1A_MIN_CLK_PREPARE_NUI_SHIFT)
#define OV8810_MIPI_CTRL1A_MIN_CLK_PREPARE_H_MASK	0x3

#define OV8810_MIPI_CTRL1B			0x361B
#define OV8810_MIPI_CTRL1B_MIN_CLK_PREPARE_L_MASK	0xFF

#define OV8810_MIPI_CTRL1C			0x361C
#define OV8810_MIPI_CTRL1C_MAX_CLK_PREPARE_NUI_SHIFT	2
#define OV8810_MIPI_CTRL1C_MAX_CLK_PREPARE_NUI_MASK		(0x3F << \
				OV8810_MIPI_CTRL1C_MAX_CLK_PREPARE_NUI_SHIFT)
#define OV8810_MIPI_CTRL1C_MAX_CLK_PREPARE_H_MASK	0x3

#define OV8810_MIPI_CTRL1D			0x361D
#define OV8810_MIPI_CTRL1D_MAX_CLK_PREPARE_L_MASK	0xFF

#define OV8810_MIPI_CTRL1E			0x361E
#define OV8810_MIPI_CTRL1E_MIN_CLK_POST_NUI_SHIFT	2
#define OV8810_MIPI_CTRL1E_MIN_CLK_POST_NUI_MASK	(0x3F << \
				OV8810_MIPI_CTRL1E_MIN_CLK_POST_NUI_SHIFT)
#define OV8810_MIPI_CTRL1E_MIN_CLK_POST_H_MASK	0x3

#define OV8810_MIPI_CTRL1F			0x361F
#define OV8810_MIPI_CTRL1F_MIN_CLK_POST_L_MASK	0xFF

#define OV8810_MIPI_CTRL20			0x3620
#define OV8810_MIPI_CTRL20_MIN_CLK_TRAIL_NUI_SHIFT	2
#define OV8810_MIPI_CTRL20_MIN_CLK_TRAIL_NUI_MASK	(0x3F << \
				OV8810_MIPI_CTRL20_MIN_CLK_TRAIL_NUI_SHIFT)
#define OV8810_MIPI_CTRL20_MIN_CLK_TRAIL_H_MASK	0x3

#define OV8810_MIPI_CTRL21			0x3621
#define OV8810_MIPI_CTRL21_MIN_CLK_TRAIL_L_MASK	0xFF

#define OV8810_MIPI_CTRL22			0x3622
#define OV8810_MIPI_CTRL22_MIN_LPX_P_NUI_SHIFT	2
#define OV8810_MIPI_CTRL22_MIN_LPX_P_NUI_MASK	(0x3F << \
					OV8810_MIPI_CTRL22_MIN_LPX_P_NUI_SHIFT)
#define OV8810_MIPI_CTRL22_MIN_LPX_P_H_MASK	0x3

#define OV8810_MIPI_CTRL23			0x3623
#define OV8810_MIPI_CTRL23_MIN_LPX_P_L_MASK	0xFF

#define OV8810_MIPI_CTRL24			0x3624
#define OV8810_MIPI_CTRL24_MIN_HS_PREPARE_NUI_SHIFT	2
#define OV8810_MIPI_CTRL24_MIN_HS_PREPARE_NUI_MASK	(0x3F << \
				OV8810_MIPI_CTRL24_MIN_HS_PREPARE_NUI_SHIFT)
#define OV8810_MIPI_CTRL24_MIN_HS_PREPARE_H_MASK	0x3

#define OV8810_MIPI_CTRL25			0x3625
#define OV8810_MIPI_CTRL25_MIN_HS_PREPARE_L_MASK	0xFF

#define OV8810_MIPI_CTRL26			0x3626
#define OV8810_MIPI_CTRL26_MAX_HS_PREPARE_NUI_SHIFT	2
#define OV8810_MIPI_CTRL26_MAX_HS_PREPARE_NUI_MASK	(0x3F << \
				OV8810_MIPI_CTRL26_MAX_HS_PREPARE_NUI_SHIFT)
#define OV8810_MIPI_CTRL26_MAX_HS_PREPARE_H_MASK	0x3

#define OV8810_MIPI_CTRL27			0x3627
#define OV8810_MIPI_CTRL27_MAX_HS_PREPARE_L_MASK	0xFF

#define OV8810_MIPI_CTRL28			0x3628
#define OV8810_MIPI_CTRL28_MIN_HS_EXIT_NUI_SHIFT	2
#define OV8810_MIPI_CTRL28_MIN_HS_EXIT_NUI_MASK	(0x3F << \
				OV8810_MIPI_CTRL28_MIN_HS_EXIT_NUI_SHIFT)
#define OV8810_MIPI_CTRL28_MIN_HS_EXIT_H_MASK	0x3

#define OV8810_MIPI_CTRL29			0x3629
#define OV8810_MIPI_CTRL29_MIN_HS_EXIT_L_MASK	0xFF

/*
 * END - OUT_TOP Registers
 */
/* ------------------------------------------------------ */

/* Exposure time values */
#define DEF_MIN_EXPOSURE	250
#define DEF_MAX_EXPOSURE	128000
#define DEF_EXPOSURE	    	33000
#define EXPOSURE_STEP	    	50

#define DEF_LINEAR_GAIN		(2*256)
#define LINEAR_GAIN_STEP	0x1

struct ov8810_sensor_regif {
	u16 len;
	u32 addr;
	u32 val;
} ;

enum ov8810_orientation {
	OV8810_NO_HORZ_FLIP_OR_VERT_FLIP = 0,
	OV8810_HORZ_FLIP_ONLY,
	OV8810_VERT_FLIP_ONLY,
	OV8810_HORZ_FLIP_AND_VERT_FLIP
};

enum ov8810_shutter_type {
	ROLLING_SHUTTER_TYPE = 0,
	MECH_SHUTTER_TYPE
};

enum ov8810_flash_type {
	LED_FLASH_TYPE = 0,
	XENON_FLASH_TYPE
};

/* define a structure for ov8810 register initialization values */
struct ov8810_reg {
	unsigned int reg;
	unsigned char val;
};

struct capture_size_ov {
	unsigned long width;
	unsigned long height;
};

/*
 * struct ov8810_clk_settings - struct for storage of sensor
 * clock settings
 * @pll_mult: pll multiplier
 * @pll_pre_div: pre pll divider
 * @vt_sys_div: video system clock divider
 * @op_pix_div: output pixel clock divider
 * @op_sys_div: output system clock divider
 * @div8: pixel bit divider
 * @rp_clk_div: video pixel clock divider
 */
struct ov8810_clk_settings {
	u16 pll_mult;
	u16 pll_pre_div;
	u16 vt_sys_div;
	u16 op_sys_div;
	u16 op_pix_div;
	u16 div8;
	u16 rp_clk_div;
};

/*
 * struct ov8810_frame_settings - struct for storage of sensor
 * frame settings
 * @frame_len_lines: number of lines in frame
 * @line_len_pck: number of pixels in line
 */
struct ov8810_frame_settings {
	u16	frame_len_lines_min;
	u16	frame_len_lines;
	u16	line_len_pck_min;
	u16	line_len_pck;
	u16	x_addr_start;
	u16	x_addr_end;
	u16	y_addr_start;
	u16	y_addr_end;
	u16	x_output_size;
	u16	y_output_size;
	u8 	v_subsample;
	u8 	h_subsample;
	u16 	binning_sensitivity;
	struct v4l2_fract min_time_per_frame;
};

/*
 * struct ov8810_mipi_settings - struct for storage of sensor
 * initial exposure settings
 * @coarse_int_tm: coarse resolution interval time (line times)
 * @fine_int_tm: fine resolution interval time (pixel times)
 */
struct ov8810_mipi_settings {
	u16	hs_settle;
	u16	bit_depth;
};

/*
 * struct ov8810_sensor_settings - struct for storage of
 * sensor settings.
 */
struct ov8810_sensor_settings {
	struct ov8810_clk_settings clk;
	struct ov8810_frame_settings frame;
	struct ov8810_mipi_settings mipi;
};

/* Array of image sizes supported by OV8810.  These must be ordered from
 * smallest image size to largest.
 */
const static struct capture_size_ov ov8810_sizes[] = {
	/* SIZE_125K */
	{ 408, 306 },
	/* SIZE_500K */
	{ 816, 612 },
	/* SIZE_1_5M */
	{ 1632, 918 },
	/* SIZE_2M */
	{ 1632, 1224 },
	/* SIZE_8M */
	{ 3264, 2448 },
};

enum image_size_ov {
	SIZE_125K,
	SIZE_500K,
	SIZE_1_5M,
	SIZE_2M,
	SIZE_8M
};
enum pixel_format_ov {
	RAW10
};

#define OV_NUM_IMAGE_SIZES		5
#define OV_NUM_PIXEL_FORMATS		1

const static struct ov8810_reg ov8810_common[OV_NUM_IMAGE_SIZES][150] = {

	/* SIZE_408x306_Default settings */
	{
		{0x3100, 0x06},
		{0x3302, 0x20},
		{0x3099, 0x81},
		{0x309d, 0x64},
		{0x309e, 0x2d},
		{0x3321, 0x02},
		{0x3322, 0x04},
		{0x3328, 0x40},
		{0x3329, 0x00},
		{0x3306, 0x00},
		{0x3316, 0x03},
		{0x33e5, 0x02},
		{0x3058, 0x01},
		{0x3071, 0x50},
		{0x3300, 0xef},
		{0x3334, 0x02},
		{0x3331, 0x20},
		{0x3332, 0x20},
		{0x3301, 0x07},	/* enable dig_gain & vario_pixel */
		{0x3308, 0x3b},	/* set dig_gain manual mode */
		{0x3309, 0x00},	/* set dig_gain = 1x */
		{0x3082, 0x80},
		{0x331e, 0x94},
		{0x331f, 0x6e},
		{0x3092, 0x00},
		{0x30ab, 0x44},
		{0x3095, 0x0a},
		{0x308d, 0x00},
		{0x3082, 0x00},
		{0x3080, 0x40},
		{0x309f, 0x23},
		{0x3084, 0x44},
		{0x3016, 0x03},
		{0x308a, 0x02},
		{0x30e9, 0x09},
		{0x3087, 0x41},
		{0x3072, 0x0d},
		{0x3319, 0x02},
		{0x3300, 0xe3},
		{0x309e, 0x1b},
		{0x30e3, 0x0e},
		{0x30f0, 0x00},
		{0x30f2, 0x00},
		{0x30f4, 0x90},
		{0x309e, 0x09},
		{0x3347, 0x00},
		{0x3100, 0x88},
		{0x3101, 0x77},
		{0x3092, 0x00},
		{0x30f0, 0x10},
		{0x30f1, 0x56},
		{0x30fb, 0x50},
		{0x308d, 0x02},
		{0x3090, 0x2e},
		{0x3095, 0x0a},
		{0x3071, 0x40},
		{0x3013, 0x00},	/* AEC/AGC off */
		{0x3300, 0x81},	/* all ISP except BLC off */
		{0x3320, 0xc2},	/* AWB use manual 1x gain */
		{0x30e7, 0x41},	/* active lo FREX */
		{0x3610, 0x10},	/* MIPI Pclk Period 2x */
		{0x3409, 0x04},	/* fix for 0x3FC max data */
		{OV8810_REG_TERM, OV8810_VAL_TERM},
	},

	/* SIZE_816x612_Default settings */
	{
		{0x3100, 0x06},
		{0x3302, 0x20},
		{0x3099, 0x81},
		{0x309d, 0x64},
		{0x309e, 0x2d},
		{0x3321, 0x02},
		{0x3322, 0x04},
		{0x3328, 0x40},
		{0x3329, 0x00},
		{0x3306, 0x00},
		{0x3316, 0x03},
		{0x33e5, 0x01},
		{0x3058, 0x01},
		{0x3071, 0x50},
		{0x3300, 0xef},
		{0x3334, 0x02},
		{0x3331, 0x20},
		{0x3332, 0x20},
		{0x3301, 0x07},	/* enable dig_gain & vario_pixel */
		{0x3308, 0x3b},	/* set dig_gain manual mode */
		{0x3309, 0x00},	/* set dig_gain = 1x */
		{0x3082, 0x80},
		{0x331e, 0x94},
		{0x331f, 0x6e},
		{0x3092, 0x00},
		{0x30ab, 0x44},
		{0x3095, 0x0a},
		{0x308d, 0x00},
		{0x3082, 0x00},
		{0x3080, 0x40},
		{0x309f, 0x23},
		{0x3084, 0x44},
		{0x3016, 0x03},
		{0x308a, 0x01},
		{0x30e9, 0x09},
		{0x3087, 0x41},
		{0x3072, 0x0d},
		{0x3319, 0x02},
		{0x3300, 0xe3},
		{0x309e, 0x1b},
		{0x30e3, 0x0e},
		{0x30f0, 0x00},
		{0x30f2, 0x00},
		{0x30f4, 0x90},
		{0x309e, 0x09},
		{0x3347, 0x00},
		{0x3100, 0x88},
		{0x3101, 0x77},
		{0x3092, 0x00},
		{0x30f0, 0x10},
		{0x30f1, 0x56},
		{0x30fb, 0x50},
		{0x308d, 0x02},
		{0x3090, 0x2e},
		{0x3095, 0x0a},
		{0x3071, 0x40},
		{0x3013, 0x00},	/* AEC/AGC off */
		{0x3300, 0x81},	/* all ISP except BLC off */
		{0x3320, 0xc2},	/* AWB use manual 1x gain */
		{0x30e7, 0x41},	/* active lo FREX */
		{0x3610, 0x1b},	/* MIPI Pclk Period 2x */
		{0x3409, 0x04},	/* fix for 0x3FC max data */
		{OV8810_REG_TERM, OV8810_VAL_TERM},
	},
	/* SIZE_1632x918 Default settings */
	{
		{0x3100, 0x06},
		{0x3302, 0x20},
		{0x3099, 0x81},
		{0x309d, 0x64},
		{0x309e, 0x2d},
		{0x3321, 0x02},
		{0x3322, 0x04},
		{0x3328, 0x40},
		{0x3329, 0x00},
		{0x3306, 0x00},
		{0x3316, 0x03},
		{0x33e5, 0x00},
		{0x3058, 0x01},
		{0x3071, 0x50},
		{0x3300, 0xef},
		{0x3334, 0x02},
		{0x3331, 0x20},
		{0x3332, 0x20},
		{0x3301, 0x03}, /* enable dig_gain */
		{0x3308, 0x3b}, /* set dig_gain manual mode */
		{0x3309, 0x00}, /* set dig_gain = 1x */
		{0x3082, 0x80},
		{0x331e, 0x94},
		{0x331f, 0x6e},
		{0x3092, 0x00},
		{0x30ab, 0x44},
		{0x3095, 0x0a},
		{0x308d, 0x00},
		{0x3082, 0x00},
		{0x3080, 0x40},
		{0x309f, 0x23},
		{0x3084, 0x44},
		{0x3016, 0x03},
		{0x308a, 0x02},
		{0x30e9, 0x09},
		{0x3087, 0x41},
		{0x3072, 0x0d},
		{0x3319, 0x04},
		{0x3300, 0xe3},
		{0x309e, 0x1b},
		{0x30e3, 0x0e},
		{0x30f0, 0x00},
		{0x30f2, 0x00},
		{0x30f4, 0x90},
		{0x309e, 0x09},
		{0x3347, 0x00},
		{0x3100, 0x88},
		{0x3101, 0x77},
		{0x3092, 0x00},
		{0x30f0, 0x10},
		{0x30f1, 0x56},
		{0x30fb, 0x50},
		{0x308d, 0x02},
		{0x3090, 0x2e},
		{0x3095, 0x0a},
		{0x3071, 0x40},
		{0x3013, 0x00},	/* AEC/AGC off */
		{0x3300, 0x81},	/* all ISP except BLC off */
		{0x3320, 0xc2},	/* AWB use manual 1x gain */
		{0x30e7, 0x41},	/* active lo FREX */
		{0x3610, 0x15},	/* MIPI Pclk Period 2x */
		{0x3409, 0x04},	/* fix for 0x3FC max data */
		{OV8810_REG_TERM, OV8810_VAL_TERM},
	},
	/* SIZE_1632x1224 Default settings */
	{
		{0x3100, 0x06},
		{0x3302, 0x20},
		{0x3099, 0x81},
		{0x309d, 0x64},
		{0x309e, 0x2d},
		{0x3321, 0x02},
		{0x3322, 0x04},
		{0x3328, 0x40},
		{0x3329, 0x00},
		{0x3306, 0x00},
		{0x3316, 0x03},
		{0x33e5, 0x00},
		{0x3058, 0x01},
		{0x3071, 0x50},
		{0x3300, 0xef},
		{0x3334, 0x02},
		{0x3331, 0x20},
		{0x3332, 0x20},
		{0x3301, 0x03},	/* enable dig_gain */
		{0x3308, 0x3b},	/* set dig_gain manual mode */
		{0x3309, 0x00},	/* set dig_gain = 1x */
		{0x3082, 0x80},
		{0x331e, 0x94},
		{0x331f, 0x6e},
		{0x3092, 0x00},
		{0x30ab, 0x44},
		{0x3095, 0x0a},
		{0x308d, 0x00},
		{0x3082, 0x00},
		{0x3080, 0x40},
		{0x309f, 0x23},
		{0x3084, 0x44},
		{0x3016, 0x03},
		{0x308a, 0x02},
		{0x30e9, 0x09},
		{0x3087, 0x41},
		{0x3072, 0x0d},
		{0x3319, 0x04},
		{0x3300, 0xe3},
		{0x309e, 0x1b},
		{0x30e3, 0x0e},
		{0x30f0, 0x00},
		{0x30f2, 0x00},
		{0x30f4, 0x90},
		{0x309e, 0x09},
		{0x3347, 0x00},
		{0x3100, 0x88},
		{0x3101, 0x77},
		{0x3092, 0x00},
		{0x30f0, 0x10},
		{0x30f1, 0x56},
		{0x30fb, 0x50},
		{0x308d, 0x02},
		{0x3090, 0x2e},
		{0x3095, 0x0a},
		{0x3071, 0x40},
		{0x3013, 0x00},	/* AEC/AGC off */
		{0x3300, 0x81},	/* all ISP except BLC off */
		{0x3320, 0xc2},	/* AWB use manual 1x gain */
		{0x30e7, 0x41},	/* active lo FREX */
		{0x3610, 0x15},	/* MIPI Pclk Period 2x */
		{0x3409, 0x04},	/* fix for 0x3FC max data */
		{OV8810_REG_TERM, OV8810_VAL_TERM},
	},
	/* SIZE_3264x2448 Default settings */
	{
		{0x3100, 0x06},
		{0x3302, 0x20},
		{0x3099, 0x81},
		{0x309d, 0x64},
		{0x309e, 0x2d},
		{0x3321, 0x02},
		{0x3322, 0x04},
		{0x3328, 0x40},
		{0x3329, 0x00},
		{0x3306, 0x00},
		{0x3316, 0x03},
		{0x33e5, 0x00},
		{0x3058, 0x01},
		{0x3071, 0x40},
		{0x3300, 0xef},
		{0x3334, 0x02},
		{0x3331, 0x20},
		{0x3332, 0x20},
		{0x3301, 0x03},	/* enable dig_gain */
		{0x3308, 0x3b},	/* set dig_gain manual mode */
		{0x3309, 0x00},	/* set dig_gain = 1x */
		{0x3082, 0x80},
		{0x331e, 0x94},
		{0x331f, 0x6e},
		{0x331c, 0x28},
		{0x331d, 0x21},
		{0x3092, 0x00},
		{0x30ab, 0x44},
		{0x3095, 0x0a},
		{0x308d, 0x00},
		{0x3082, 0x00},
		{0x3080, 0x40},
		{0x309f, 0x23},	/* reset DSP */
		{0x3084, 0x44},
		{0x3016, 0x03},
		{0x308a, 0x01},
		{0x30e9, 0x09},
		{0x3087, 0x41},
		{0x3305, 0xa0},
		{0x3072, 0x01},
		{0x3319, 0x08},
		{0x3300, 0xe3},
		{0x309e, 0x1b},
		{0x30e3, 0x0e},
		{0x30f0, 0x00},
		{0x30f2, 0x00},
		{0x30f4, 0x90},
		{0x3347, 0x00},
		{0x3100, 0x88},
		{0x3101, 0x77},
		{0x3092, 0x00},
		{0x30f0, 0x10},
		{0x30f1, 0x56},
		{0x30fb, 0x50},
		{0x308d, 0x02},
		{0x3090, 0x2e},
		{0x3095, 0x0a},
		{0x3071, 0x40},
		{0x3013, 0x00},	/* AEC/AGC, off */
		{0x3300, 0x83},	/* all, ISP, except BLC, off */
		{0x3320, 0xc2},	/* AWB, use, manual, 1x, gain */
		{0x30e7, 0x41},	/* active lo FREX */
		{0x3610, 0x15},	/* MIPI Pclk Period 2x */
		{0x3409, 0x04},	/* fix for 0x3FC max data */
		{OV8810_REG_TERM, OV8810_VAL_TERM},
	},
};

/* 50-60 Hz Detection settings */
const static struct ov8810_reg ov8810_50_60_hz_detect_tbl[] = {
	{0x3014, 0x40},
	{0x304c, 0x0c},
	{0x30a4, 0x00},
	{0x30ab, 0x04},
	{0x30ad, 0x04},
	{0x3040, 0x00},
	{0x3041, 0x34},
	{0x3044, 0x28},
	{0x3045, 0x98},
	{0x3046, 0x00},
	{0x3047, 0x00},
	{0x3048, 0x01},
	{0x3049, 0xc2},
	{0x304e, 0x02},
	{0x304a, 0xaf},	/* 27Mhz */
	{0x304b, 0xc8},	/* 27Mhz */
	/* read 303d[0], 1=50hz, 0=60hz */
	{OV8810_REG_TERM, OV8810_VAL_TERM},
};

/* LDO 1.6V Register Settings */
const static struct ov8810_reg ov8810_ldo_1_6v[2][10] = {

	/* Settings used depend on 50-60 Hz Detection On/Off */

	/* 50-60 Hz Detection OFF */
	{
		{0x30a8, 0x2f},
		{0x30a9, 0x00},
		{0x30aa, 0x57},
		{0x30ab, 0x34},
		{0x30ac, 0x30},
		{0x30ad, 0x01},
		{0x30ae, 0x02},
		{OV8810_REG_TERM, OV8810_VAL_TERM},
	},

	/* 50-60 Hz Detection ON */
	{
		{0x30a8, 0x2f},
		{0x30a9, 0x00},
		{0x30aa, 0x57},
		{0x30ab, 0x04},
		{0x30ac, 0x30},
		{0x30ad, 0x03},
		{0x30ae, 0x02},
		{OV8810_REG_TERM, OV8810_VAL_TERM},
		},
	};

/* Lens correction settings */
const static struct ov8810_reg len_correction_tbl[] = {
	/* G */
	{0x3358, 0x28},
	{0x3359, 0x0d},
	{0x335a, 0x0c},
	{0x335b, 0x0a},
	{0x335c, 0x0a},
	{0x335d, 0x0b},
	{0x335e, 0x0d},
	{0x335f, 0x13},
	{0x3360, 0x0d},
	{0x3361, 0x0a},
	{0x3362, 0x07},
	{0x3363, 0x05},
	{0x3364, 0x05},
	{0x3365, 0x06},
	{0x3366, 0x08},
	{0x3367, 0x09},
	{0x3368, 0x0b},
	{0x3369, 0x06},
	{0x336a, 0x03},
	{0x336b, 0x02},
	{0x336c, 0x01},
	{0x336d, 0x02},
	{0x336e, 0x04},
	{0x336f, 0x06},
	{0x3370, 0x08},
	{0x3371, 0x04},
	{0x3372, 0x01},
	{0x3373, 0x00},
	{0x3374, 0x00},
	{0x3375, 0x00},
	{0x3376, 0x02},
	{0x3377, 0x04},
	{0x3378, 0x08},
	{0x3379, 0x04},
	{0x337a, 0x01},
	{0x337b, 0x00},
	{0x337c, 0x00},
	{0x337d, 0x00},
	{0x337e, 0x03},
	{0x337f, 0x04},
	{0x3380, 0x0b},
	{0x3381, 0x06},
	{0x3382, 0x04},
	{0x3383, 0x02},
	{0x3384, 0x02},
	{0x3385, 0x03},
	{0x3386, 0x05},
	{0x3387, 0x07},
	{0x3388, 0x0f},
	{0x3389, 0x0b},
	{0x338a, 0x08},
	{0x338b, 0x07},
	{0x338c, 0x07},
	{0x338d, 0x07},
	{0x338e, 0x0a},
	{0x338f, 0x0b},
	{0x3390, 0x26},
	{0x3391, 0x11},
	{0x3392, 0x0e},
	{0x3393, 0x0c},
	{0x3394, 0x0b},
	{0x3395, 0x0c},
	{0x3396, 0x0e},
	{0x3397, 0x11},

	/* B */
	{0x3398, 0x10},
	{0x3399, 0x10},
	{0x339a, 0x10},
	{0x339b, 0x10},
	{0x339c, 0x10},
	{0x339d, 0x10},
	{0x339e, 0x10},
	{0x339f, 0x10},
	{0x33a0, 0x10},
	{0x33a1, 0x10},
	{0x33a2, 0x10},
	{0x33a3, 0x10},
	{0x33a4, 0x10},
	{0x33a5, 0x10},
	{0x33a6, 0x10},
	{0x33a7, 0x10},
	{0x33a8, 0x10},
	{0x33a9, 0x10},
	{0x33aa, 0x10},
	{0x33ab, 0x10},
	{0x33ac, 0x10},
	{0x33ad, 0x10},
	{0x33ae, 0x10},
	{0x33af, 0x10},
	{0x33b0, 0x10},
	{0x33b1, 0x10},
	{0x33b2, 0x10},
	{0x33b3, 0x10},
	{0x33b4, 0x10},
	{0x33b5, 0x10},
	{0x33b6, 0x10},
	{0x33b7, 0x10},
	{0x33b8, 0x10},
	{0x33b9, 0x10},
	{0x33ba, 0x10},
	{0x33bb, 0x10},

	/* R */
	{0x33bc, 0x10},
	{0x33bd, 0x10},
	{0x33be, 0x10},
	{0x33bf, 0x10},
	{0x33c0, 0x10},
	{0x33c1, 0x10},
	{0x33c2, 0x10},
	{0x33c3, 0x10},
	{0x33c4, 0x10},
	{0x33c5, 0x10},
	{0x33c6, 0x10},
	{0x33c7, 0x10},
	{0x33c8, 0x10},
	{0x33c9, 0x10},
	{0x33ca, 0x10},
	{0x33cb, 0x10},
	{0x33cc, 0x10},
	{0x33cd, 0x10},
	{0x33ce, 0x10},
	{0x33cf, 0x10},
	{0x33d0, 0x10},
	{0x33d1, 0x10},
	{0x33d2, 0x10},
	{0x33d3, 0x10},
	{0x33d4, 0x10},
	{0x33d5, 0x10},
	{0x33d6, 0x10},
	{0x33d7, 0x10},
	{0x33d8, 0x10},
	{0x33d9, 0x10},
	{0x33da, 0x10},
	{0x33db, 0x10},
	{0x33dc, 0x10},
	{0x33dd, 0x10},
	{0x33de, 0x10},
	{0x33df, 0x10},
	{0x3350, 0x06},
	{0x3351, 0xab},
	{0x3352, 0x05},
	{0x3353, 0x00},
	{0x3354, 0x04},
	{0x3355, 0xf8},
	{0x3356, 0x07},
	{0x3357, 0x74},
	{OV8810_REG_TERM, OV8810_VAL_TERM},
};

const static struct ov8810_reg ov8810_common_csi2[] = {
	{OV8810_MIPI_CTRL0B, 0x0f},  /* disable MIPI output (enabled later) */
	{0x3601, 0x16},
	{0x30B8, 0x28},		/* disable low amp MIPI & pwr down MIPI PHY
				   when at sleep */
	{OV8810_REG_TERM, OV8810_VAL_TERM},
};

const static struct ov8810_reg ov8810_mech_shutter[] = {
	{0x30e1, 0xd0},	/* Set array reset control for frame mode */
	{0x350e, 0x44},	/* Use sensor native vsync for frame mode */
	{0x3058, 0x0c},	/* Set internal analog control for frame mode */
	{OV8810_REG_TERM, OV8810_VAL_TERM},
};

#endif /* ifndef OV8810_REG_H */

