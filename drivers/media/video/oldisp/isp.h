/*
 * drivers/media/video/isp/isp.h
 *
 * Top level public header file for ISP Control module in
 * TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2008 Texas Instruments.
 * Copyright (C) 2008 Nokia.
 *
 * Contributors:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Mohit Jalori <mjalori@ti.com>
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
#include <media/videobuf-dma-sg.h>
#include <linux/videodev2.h>

#include <mach/oldisp_user.h>

#include "ispmmu.h"

#define OMAP_ISP_CCDC		(1 << 0)
#define OMAP_ISP_PREVIEW	(1 << 1)
#define OMAP_ISP_RESIZER	(1 << 2)
#define OMAP_ISP_AEWB		(1 << 3)
#define OMAP_ISP_AF		(1 << 4)
#define OMAP_ISP_HIST		(1 << 5)

/* Our ISP specific controls */
#define V4L2_CID_PRIVATE_ISP_COLOR_FX		(V4L2_CID_PRIVATE_BASE + 0)

#define ISP_TOK_TERM		0xFFFFFFFF	/*
						 * terminating token for ISP
						 * modules reg list
						 */
#define NUM_SG_DMA		(VIDEO_MAX_FRAME + 2)

#define ISP_BUF_INIT		0
#define ISP_FREE_RUNNING	1
#define ISP_BUF_TRAN		2

#ifndef CONFIG_ARCH_OMAP3410
#define USE_ISP_PREVIEW
#define USE_ISP_RESZ
#define is_isppreview_enabled()		1
#define is_ispresizer_enabled()		1
#else
#define is_isppreview_enabled()		0
#define is_ispresizer_enabled()		0
#endif

#ifdef OMAP_ISPCTRL_DEBUG
#define is_ispctrl_debug_enabled()		1
#else
#define is_ispctrl_debug_enabled()		0
#endif

#define ISP_XCLKA_DEFAULT		0x12
#define ISP_OUTPUT_WIDTH_DEFAULT	176
#define ISP_OUTPUT_HEIGHT_DEFAULT	144
#define ISP_BYTES_PER_PIXEL		2
#define NUM_ISP_CAPTURE_FORMATS 	(sizeof(isp_formats) /\
							sizeof(isp_formats[0]))
#define ISP_WORKAROUND 1
#define ISP_BUFFER_MAX_SIZE (1024 * 1024 * 10)
#define ISP_BUFFER_MAX_PAGES (ISP_BUFFER_MAX_SIZE / ISPMMU_PAGE_SIZE)

#define NR_PAGES(x, y)		((((y + x - 1) & PAGE_MASK) >> PAGE_SHIFT) - \
					((x & PAGE_MASK) >> PAGE_SHIFT) + 1)

#define ALIGN_TO(x, b)		(((unsigned long)x + (b - 1)) & ~(b - 1))
#define ALIGN_NEAR(x, b)	((unsigned long)x & ~(b-1))

typedef int (*isp_vbq_callback_ptr) (struct videobuf_buffer *vb);
typedef void (*isp_callback_t) (unsigned long status,
					isp_vbq_callback_ptr arg1, void *arg2);

enum isp_interface_type {
	ISP_PARLL = 1,
	ISP_CSIA = 2,
	ISP_CSIB = 4
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
	CBK_HS_VS,
	CBK_H3A_AF_DONE,
	CBK_HIST_DONE,
	CBK_LSC_ISR,
	CBK_CSIA,
	CBK_CSIB,
	CBK_MMU_ERR,
	CBK_H3A_AWB_DONE,
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
	u32 reg;
	u32 val;
};

/**
 * struct isp_sgdma_state - SG-DMA state for each videobuffer + 2 overlays
 * @isp_addr: ISP space address mapped by ISP MMU.
 * @status: DMA return code mapped by ISP MMU.
 * @callback: Pointer to ISP callback function.
 * @arg: Pointer to argument passed to the specified callback function.
 */
struct isp_sgdma_state {
	dma_addr_t isp_addr;
	u32 status;
	isp_callback_t callback;
	void *arg;
};

/**
 * struct isp_sgdma - ISP Scatter Gather DMA status.
 * @isp_addr_capture: Array of ISP space addresses mapped by the ISP MMU.
 * @lock: Spinlock used to check free_sgdma field.
 * @free_sgdma: Number of free SG-DMA slots.
 * @next_sgdma: Index of next SG-DMA slot to use.
 */
struct isp_sgdma {
	dma_addr_t isp_addr_capture[VIDEO_MAX_FRAME];
	spinlock_t lock;	/* For handling current buffer */
	int free_sgdma;
	int next_sgdma;
	struct isp_sgdma_state sg_state[NUM_SG_DMA];
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
 * @vdint0_timing: VD0 Interrupt timing.
 * @vdint1_timing: VD1 Interrupt timing.
 * @strobe: Strobe related parameter.
 * @prestrobe: PreStrobe related parameter.
 * @shutter: Shutter related parameter.
 * @hskip: Horizontal Start Pixel performed in Preview module.
 * @vskip: Vertical Start Line performed in Preview module.
 * @wenlog: Store the value for the sensor specific wenlog field.
 */
struct isp_interface_config {
	enum isp_interface_type ccdc_par_ser;
	u8 dataline_shift;
	u32 hsvs_syncdetect;
	u16 vdint0_timing;
	u16 vdint1_timing;
	int strobe;
	int prestrobe;
	int shutter;
	u32 wenlog;
	u32 dcsub;
	enum ispccdc_raw_fmt raw_fmt_in;
	struct ispprev_wbal wbal;
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

/**
 * struct isp_sysc - ISP Power switches to set.
 * @reset: Flag for setting ISP reset.
 * @idle_mode: Flag for setting ISP idle mode.
 */
struct isp_sysc {
	char reset;
	char idle_mode;
};

void isp_release_resources(void);

void isp_start(void);

void isp_stop(void);

void isp_sgdma_init(void);

void isp_sgdma_cancel(void);

void isp_vbq_done(unsigned long status, isp_vbq_callback_ptr arg1, void *arg2);

void isp_sgdma_process(struct isp_sgdma *sgdma, int irq, int *dma_notify,
						isp_vbq_callback_ptr func_ptr);

int isp_sgdma_queue(struct videobuf_dmabuf *vdma, struct videobuf_buffer *vb,
						int irq, int *dma_notify,
						isp_vbq_callback_ptr func_ptr);

int isp_vbq_prepare(struct videobuf_queue *vbq, struct videobuf_buffer *vb,
							enum v4l2_field field);

void isp_vbq_release(struct videobuf_queue *vbq, struct videobuf_buffer *vb);

int isp_set_callback(enum isp_callback_type type, isp_callback_t callback,
					isp_vbq_callback_ptr arg1, void *arg2);

void omapisp_unset_callback(void);

int isp_unset_callback(enum isp_callback_type type);

u32 isp_set_xclk(u32 xclk, u8 xclksel);

u32 isp_get_xclk(u8 xclksel);

int isp_request_interface(enum isp_interface_type if_t);

int isp_free_interface(enum isp_interface_type if_t);

void isp_power_settings(struct isp_sysc);

int isp_configure_interface(struct isp_interface_config *config);

void isp_CCDC_VD01_disable(void);

void isp_CCDC_VD01_enable(void);

int isp_get(void);

int isp_put(void);

void isp_set_pipeline(int soc_type);

void isp_config_pipeline(struct v4l2_pix_format *pix_input,
					struct v4l2_pix_format *pix_output);

int isp_queryctrl(struct v4l2_queryctrl *a);

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

int isp_try_size(struct v4l2_pix_format *pix_input,
					struct v4l2_pix_format *pix_output);

int isp_try_fmt(struct v4l2_pix_format *pix_input,
					struct v4l2_pix_format *pix_output);

int isp_handle_private(int cmd, void *arg);

void isp_save_context(struct isp_reg *);

void isp_restore_context(struct isp_reg *);

void isp_save_ctx(void);

void isp_restore_ctx(void);

/* Configure CCDC interface bridge*/
int isp_configure_interface_bridge(u32 par_bridge);

void isp_print_status(void);

dma_addr_t isp_buf_get(void);

int __init isp_ccdc_init(void);
int __init isp_hist_init(void);
int __init isph3a_aewb_init(void);
int __init ispmmu_init(void);
int __init isp_preview_init(void);
int __init isp_resizer_init(void);
int __init isp_af_init(void);

void __exit isp_ccdc_cleanup(void);
void __exit isp_hist_cleanup(void);
void __exit isph3a_aewb_cleanup(void);
void __exit ispmmu_cleanup(void);
void __exit isp_preview_cleanup(void);
void __exit isp_hist_cleanup(void);
void __exit isp_resizer_cleanup(void);
void __exit isp_af_exit(void);

struct page **map_user_memory_to_kernel(unsigned long addr, u32 size, u32 *nr_pages_mapped);
void unmap_user_memory_from_kernel(struct page **pages, int nr_pages);

int isp_run_resizer(void *userdata);
int isp_run_preview(void *userdata);

#endif	/* OMAP_ISP_TOP_H */
