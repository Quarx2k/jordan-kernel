/*
 * isp.h
 *
 * Top level public header file for ISP Control module in
 * TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments.
 * Copyright (C) 2009 Nokia.
 *
 * Contributors:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Mohit Jalori <mjalori@ti.com>
 * 	Sergio Aguirre <saaguirre@ti.com>
 * 	Sakari Ailus <sakari.ailus@nokia.com>
 * 	Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_TOP_H
#define OMAP_ISP_TOP_H
#include <plat/cpu.h>
#include <media/videobuf-dma-sg.h>
#include <linux/videodev2.h>
#define OMAP_ISP_CCDC		(1 << 0)
#define OMAP_ISP_PREVIEW	(1 << 1)
#define OMAP_ISP_RESIZER	(1 << 2)
#define OMAP_ISP_AEWB		(1 << 3)
#define OMAP_ISP_AF		(1 << 4)
#define OMAP_ISP_HIST		(1 << 5)

#define ISP_TOK_TERM		0xFFFFFFFF	/*
						 * terminating token for ISP
						 * modules reg list
						 */
#define NUM_BUFS		VIDEO_MAX_FRAME

#define ISP_BYTES_PER_PIXEL		2
#define NUM_ISP_CAPTURE_FORMATS 	(sizeof(isp_formats) /		\
					 sizeof(isp_formats[0]))

#define NR_PAGES(x, y)		((((y + x - 1) & PAGE_MASK) >> PAGE_SHIFT) - \
					((x & PAGE_MASK) >> PAGE_SHIFT) + 1)

#define ALIGN_TO(x, b)		(((unsigned long)x + (b - 1)) & ~(b - 1))
#define ALIGN_NEAR(x, b)	((unsigned long)x & ~(b-1))

#define ISP_LSC_MEMORY	(16*1024*1024)	/* 16MB LSC workaround memory */

typedef int (*isp_vbq_callback_ptr) (struct videobuf_buffer *vb);
typedef void (*isp_callback_t) (unsigned long status,
				isp_vbq_callback_ptr arg1, void *arg2);

enum isp_mem_resources {
	OMAP3_ISP_IOMEM_MAIN,
	OMAP3_ISP_IOMEM_CBUFF,
	OMAP3_ISP_IOMEM_CCP2,
	OMAP3_ISP_IOMEM_CCDC,
	OMAP3_ISP_IOMEM_HIST,
	OMAP3_ISP_IOMEM_H3A,
	OMAP3_ISP_IOMEM_PREV,
	OMAP3_ISP_IOMEM_RESZ,
	OMAP3_ISP_IOMEM_SBL,
	OMAP3_ISP_IOMEM_CSI2A,
	OMAP3_ISP_IOMEM_CSI2PHY
};

enum isp_running {
	ISP_STOPPED,
	ISP_RUNNING,
	ISP_STOPPING,
	ISP_FREERUNNING
};

struct isp_device {
	struct device *dev;
	u32 revision;

	/*** platform HW resources ***/
	unsigned int irq;

#define mmio_base_main mmio_base[OMAP3_ISP_IOMEM_MAIN]
#define mmio_cbuff_main mmio_base[OMAP3_ISP_IOMEM_CBUFF]
#define mmio_ccp2_main mmio_base[OMAP3_ISP_IOMEM_CCP2]
#define mmio_ccdc_main mmio_base[OMAP3_ISP_IOMEM_CCDC]
#define mmio_hist_main mmio_base[OMAP3_ISP_IOMEM_HIST]
#define mmio_h3a_main mmio_base[OMAP3_ISP_IOMEM_H3A]
#define mmio_prev_main mmio_base[OMAP3_ISP_IOMEM_PREV]
#define mmio_resz_main mmio_base[OMAP3_ISP_IOMEM_RESZ]
#define mmio_sbl_main mmio_base[OMAP3_ISP_IOMEM_SBL]
#define mmio_csi2_main mmio_base[OMAP3_ISP_IOMEM_CSI2A]
#define mmio_csi2phy_main mmio_base[OMAP3_ISP_IOMEM_CSI2PHY]
	unsigned long mmio_base[OMAP3_ISP_IOMEM_CSI2PHY + 1];
	unsigned long mmio_base_phys[OMAP3_ISP_IOMEM_CSI2PHY + 1];
	unsigned long mmio_size[OMAP3_ISP_IOMEM_CSI2PHY + 1];
};

enum isp_interface_type {
	ISP_PARLL = 1,
	ISP_CSIA = 2,
	ISP_CSIB = 4,
	ISP_NONE = 8 /* memory input to preview / resizer */
};

enum isp_irqevents {
	CSIA = 0x01,
	CSIB = 0x10,
	CCDC_VD0 = 0x100,
	CCDC_VD1 = 0x200,
	CCDC_VD2 = 0x400,
	CCDC_ERR = 0x800,
	H3A_AWB_DONE = 0x2000,
	H3A_AF_DONE = 0x1000,
	HIST_DONE = 0x10000,
	PREV_DONE = 0x100000,
	LSC_DONE = 0x20000,
	LSC_PRE_COMP = 0x40000,
	LSC_PRE_ERR = 0x80000,
	RESZ_DONE = 0x1000000,
	SBL_OVF = 0x2000000,
	MMU_ERR = 0x10000000,
	OCP_ERR = 0x20000000,
	HS_VS = 0x80000000
};

enum isp_callback_type {
	CBK_CCDC_VD0 = 0,
	CBK_CCDC_VD1,
	CBK_PREV_DONE,
	CBK_RESZ_DONE,
	CBK_MMU_ERR,
	CBK_H3A_AWB_DONE,
	CBK_HIST_DONE,
	CBK_HS_VS,
	CBK_H3A_AF_DONE,
	CBK_CSIA,
	CBK_CSIB,
	CBK_SBL_OVF,
	CBK_CATCHALL,
	CBK_END,
};

enum ispccdc_raw_fmt {
	ISPCCDC_INPUT_FMT_GR_BG,
	ISPCCDC_INPUT_FMT_RG_GB,
	ISPCCDC_INPUT_FMT_BG_GR,
	ISPCCDC_INPUT_FMT_GB_RG,
};

/**
 * struct isp_reg - Structure for ISP register values.
 * @reg: 32-bit Register address.
 * @val: 32-bit Register value.
 */
struct isp_reg {
	enum isp_mem_resources mmio_range;
	u32 reg;
	u32 val;
};

/**
 * struct isp_interface_config - ISP interface configuration.
 * @ccdc_par_ser: ISP interface type. 0 - Parallel, 1 - CSIA, 2 - CSIB to CCDC.
 * @par_bridge: CCDC Bridge input control. Parallel interface.
 *                  0 - Disable, 1 - Enable, first byte->cam_d(bits 7 to 0)
 *                  2 - Enable, first byte -> cam_d(bits 15 to 8)
 * @par_clk_pol: Pixel clock polarity on the parallel interface.
 *                    0 - Non Inverted, 1 - Inverted
 * @dataline_shift: Data lane shifter.
 *                      0 - No Shift, 1 - CAMEXT[13 to 2]->CAM[11 to 0]
 *                      2 - CAMEXT[13 to 4]->CAM[9 to 0]
 *                      3 - CAMEXT[13 to 6]->CAM[7 to 0]
 * @hsvs_syncdetect: HS or VS synchronization signal detection.
 *                       0 - HS Falling, 1 - HS rising
 *                       2 - VS falling, 3 - VS rising
 * @strobe: Strobe related parameter.
 * @prestrobe: PreStrobe related parameter.
 * @shutter: Shutter related parameter.
 * @hskip: Horizontal Start Pixel performed in Preview module.
 * @vskip: Vertical Start Line performed in Preview module.
 * @wenlog: Store the value for the sensor specific wenlog field.
 * @wait_bayer_frame: Skip this many frames before starting bayer capture.
 * @wait_yuv_frame: Skip this many frames before starting yuv capture.
 */
struct isp_interface_config {
	enum isp_interface_type ccdc_par_ser;
	u8 dataline_shift;
	u32 hsvs_syncdetect;
	int strobe;
	int prestrobe;
	int shutter;
	u32 wenlog;
	int wait_bayer_frame;
	int wait_yuv_frame;
	u32 dcsub;
	u32 cam_mclk;
	u32 cam_mclk_src_div;
	enum ispccdc_raw_fmt raw_fmt_in;
	union {
		struct par {
			unsigned par_bridge:2;
			unsigned par_clk_pol:1;
		} par;
		struct csi {
			unsigned crc:1;
			unsigned mode:1;
			unsigned edge:1;
			unsigned signalling:1;
			unsigned strobe_clock_inv:1;
			unsigned vs_edge:1;
			unsigned channel:3;
			unsigned vpclk:2;	/* Video port output clock */
			unsigned int data_start;
			unsigned int data_size;
			u32 format;		/* V4L2_PIX_FMT_* */
		} csi;
	} u;
};

u32 isp_reg_readl(enum isp_mem_resources isp_mmio_range, u32 reg_offset);

void isp_reg_writel(u32 reg_value, enum isp_mem_resources isp_mmio_range,
		    u32 reg_offset);

static inline void isp_reg_and(enum isp_mem_resources mmio_range, u32 reg,
			       u32 and_bits)
{
	u32 v = isp_reg_readl(mmio_range, reg);

	isp_reg_writel(v & and_bits, mmio_range, reg);
}

static inline void isp_reg_or(enum isp_mem_resources mmio_range, u32 reg,
			      u32 or_bits)
{
	u32 v = isp_reg_readl(mmio_range, reg);

	isp_reg_writel(v | or_bits, mmio_range, reg);
}

static inline void isp_reg_and_or(enum isp_mem_resources mmio_range, u32 reg,
				  u32 and_bits, u32 or_bits)
{
	u32 v = isp_reg_readl(mmio_range, reg);

	isp_reg_writel((v & and_bits) | or_bits, mmio_range, reg);
}

void isp_flush(void);

void isp_start(void);

void isp_stop(void);

int isp_buf_queue(struct videobuf_buffer *vb,
		  void (*complete)(struct videobuf_buffer *vb, void *priv),
		  void *priv);

int isp_vbq_setup(struct videobuf_queue *vbq, unsigned int *cnt,
		  unsigned int *size);

int isp_vbq_prepare(struct videobuf_queue *vbq, struct videobuf_buffer *vb,
		    enum v4l2_field field);

void isp_vbq_release(struct videobuf_queue *vbq, struct videobuf_buffer *vb);

int isp_set_callback(enum isp_callback_type type, isp_callback_t callback,
		     isp_vbq_callback_ptr arg1, void *arg2);

int isp_unset_callback(enum isp_callback_type type);

u32 isp_set_xclk(u32 xclk, u8 xclksel);

void isp_power_settings(int idle);

int isp_configure_interface(struct isp_interface_config *config);

int isp_configure_interface_bridge(u32 par_bridge);

int isp_get(void);

int isp_put(void);

int isp_queryctrl(struct v4l2_queryctrl *a);

int isp_querymenu(struct v4l2_querymenu *a);

int isp_g_ctrl(struct v4l2_control *a);

int isp_s_ctrl(struct v4l2_control *a);

int isp_enum_fmt_cap(struct v4l2_fmtdesc *f);

int isp_try_fmt_cap(struct v4l2_pix_format *pix_input,
		    struct v4l2_pix_format *pix_output);

void isp_g_fmt_cap(struct v4l2_pix_format *pix);

int isp_s_fmt_cap(struct v4l2_pix_format *pix_input,
		  struct v4l2_pix_format *pix_output);

int isp_g_crop(struct v4l2_crop *a);

int isp_s_crop(struct v4l2_crop *a, struct v4l2_pix_format *pix);

void isp_config_crop(struct v4l2_pix_format *pix);

int isp_try_fmt(struct v4l2_pix_format *pix_input,
		struct v4l2_pix_format *pix_output);

int isp_lsc_workaround_enabled(void);

int isp_handle_private(struct mutex *, int cmd, void *arg);

void isp_save_context(struct isp_reg *);

void isp_restore_context(struct isp_reg *);

void isp_print_status(void);

void isp_set_hs_vs(int);

unsigned long isp_get_buf_offset(void);

enum isp_running isp_state(void);

dma_addr_t isp_tmp_buf_addr(void);

int __init isp_ccdc_init(void);
int __init isp_hist_init(void);
int __init isph3a_aewb_init(void);
int __init isp_preview_init(void);
int __init isp_resizer_init(void);
int __init isp_af_init(void);
int __init isp_csi2_init(void);

void isp_ccdc_cleanup(void);
void isp_hist_cleanup(void);
void isph3a_aewb_cleanup(void);
void isp_preview_cleanup(void);
void isp_hist_cleanup(void);
void isp_resizer_cleanup(void);
void isp_af_exit(void);
void isp_csi2_cleanup(void);

#endif	/* OMAP_ISP_TOP_H */
