/*
 * Minnow DSI command mode panel
 *
 * Copyright (C) 2013 Motorola Mobility LLC.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*#define DEBUG*/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>

#include <video/omapdss.h>
#include <video/omap-panel-data.h>
#include <video/mipi_display.h>

#include "../dss/dss.h"


/* DSI Virtual channel. Hardcoded for now. */
#define TCH 0

#define DCS_READ_NUM_ERRORS	0x05
#define DCS_BRIGHTNESS		0x51
#define DCS_CTRL_DISPLAY	0x53
#define DCS_WRITE_CABC		0x55
#define DCS_READ_CABC		0x56
#define DCS_GET_ID1		0xda
#define DCS_GET_ID2		0xdb
#define DCS_GET_ID3		0xdc

enum minnow_reset_gpios {
	MINNOW_RESET_PANEL,
	MINNOW_RESET_BRIDGE,
	MINNOW_RESET_MAX
};

enum minnow_panel_id {
	MINNOW_PANEL_CM_220X176,
	MINNOW_PANEL_CM_220X220,
	MINNOW_PANEL_CM_BRIDGE_320X320,
	MINNOW_PANEL_MAX
};

enum minnow_cmd_type {
	DCS_WRITE_SYNC,
	GENERIC_WRITE_SYNC,
	DCS_WRITE,
	GENERIC_WRITE,
	BTA_SYNC,
	WAIT_MS,
	CMD_TYPE_MAX
};


/* Panel Initialize DSI DCS command buffer description:
 * it uses compact DCS command buffer to store all DCS commands, the first
 * byte of each command is the command length in byte
 */
static u8 panel_init_220x176[] = {
/*n, type, data_0, data_1 ... data_n-1*/
1,  DCS_WRITE_SYNC, MIPI_DCS_EXIT_SLEEP_MODE,
1,  WAIT_MS, 5,
3,  DCS_WRITE_SYNC, 0xF0, 0x5A, 0x5A,
3,  DCS_WRITE_SYNC, 0xF1, 0x5A, 0x5A,
18, DCS_WRITE_SYNC, 0xF2, 0x16, 0xDC, 0x03, 0x28, 0x28, 0x10, 0x00, 0x60, 0xF8,
		    0x00, 0x07, 0x02, 0x00, 0x00, 0xDC, 0x28, 0x28,
15, DCS_WRITE_SYNC, 0xF4, 0x0A, 0x00, 0x00, 0x00, 0x77, 0x7F, 0x07, 0x22, 0x2A,
		    0x43, 0x07, 0x2A, 0x43, 0x07,
11, DCS_WRITE_SYNC, 0xF5, 0x00, 0x50, 0x28, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00,
		    0x00,
10, DCS_WRITE_SYNC, 0xF6, 0x07, 0x00, 0x07, 0x00, 0x0B, 0x04, 0x04, 0x04, 0x07,
5,  DCS_WRITE_SYNC, 0xF7, 0x00, 0x00, 0x00, 0x00,
3,  DCS_WRITE_SYNC, 0xF8, 0x44, 0x08,
2,  DCS_WRITE_SYNC, 0xF9, 0x04,
17, DCS_WRITE_SYNC, 0xFA, 0x0F, 0x0F, 0x1E, 0x23, 0x26, 0x2D, 0x21, 0x2B, 0x33,
		    0x32, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00,
17, DCS_WRITE_SYNC, 0xFB, 0x0F, 0x0F, 0x1E, 0x23, 0x26, 0x2D, 0x21, 0x2B, 0x33,
		    0x32, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00,
2,  DCS_WRITE_SYNC, 0xF9, 0x02,
17, DCS_WRITE_SYNC, 0xFA, 0x00, 0x00, 0x0A, 0x16, 0x1D, 0x27, 0x1C, 0x30, 0x38,
		    0x37, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
17, DCS_WRITE_SYNC, 0xFB, 0x00, 0x00, 0x0A, 0x16, 0x1D, 0x27, 0x1C, 0x30, 0x38,
		    0x37, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,
2,  DCS_WRITE_SYNC, 0xF9, 0x01,
17, DCS_WRITE_SYNC, 0xFA, 0x00, 0x00, 0x13, 0x14, 0x19, 0x24, 0x1A, 0x31, 0x39,
		    0x38, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00,
17, DCS_WRITE_SYNC, 0xFB, 0x00, 0x00, 0x13, 0x14, 0x19, 0x24, 0x1A, 0x31, 0x39,
		    0x38, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00,
3,  DCS_WRITE_SYNC, 0xF0, 0x00, 0x00,
3,  DCS_WRITE_SYNC, 0xF1, 0x00, 0x00,
2,  DCS_WRITE_SYNC, 0x36, 0xD8,
2,  DCS_WRITE_SYNC, 0x3A, 0x06,
0
};

static u8 panel_init_220x220[] = {
/*n, type, data_0, data_1 ... data_n-1*/
1,  DCS_WRITE_SYNC, MIPI_DCS_EXIT_SLEEP_MODE,
1,  WAIT_MS, 5,
3,  DCS_WRITE_SYNC, 0xF0, 0x5A, 0x5A,
3,  DCS_WRITE_SYNC, 0xF1, 0x5A, 0x5A,
18, DCS_WRITE_SYNC, 0xF2, 0x1C, 0xDC, 0x03, 0x28, 0x28, 0x10, 0x00, 0x60, 0xF8,
		    0x00, 0x07, 0x02, 0x00, 0x00, 0xDC, 0x28, 0x28,
15, DCS_WRITE_SYNC, 0xF4, 0x0A, 0x00, 0x00, 0x00, 0x77, 0x7F, 0x07, 0x22, 0x2A,
		    0x43, 0x07, 0x2A, 0x43, 0x07,
11, DCS_WRITE_SYNC, 0xF5, 0x00, 0x50, 0x28, 0x00, 0x00, 0x09, 0x00, 0x00, 0x01,
		    0x01,
10, DCS_WRITE_SYNC, 0xF6, 0x07, 0x00, 0x07, 0x00, 0x0B, 0x04, 0x04, 0x04, 0x07,
5,  DCS_WRITE_SYNC, 0xF7, 0x00, 0x00, 0x00, 0x00,
3,  DCS_WRITE_SYNC, 0xF8, 0x44, 0x02,
2,  DCS_WRITE_SYNC, 0xF9, 0x04,
17, DCS_WRITE_SYNC, 0xFA, 0x1E, 0x1E, 0x0D, 0x1D, 0x21, 0x2C, 0x23, 0x28, 0x2C,
		    0x28, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00,
17, DCS_WRITE_SYNC, 0xFB, 0x1E, 0x1E, 0x0D, 0x1D, 0x21, 0x2C, 0x23, 0x28, 0x2C,
		    0x28, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00,
2,  DCS_WRITE_SYNC, 0xF9, 0x02,
17, DCS_WRITE_SYNC, 0xFA, 0x19, 0x18, 0x08, 0x0F, 0x18, 0x26, 0x1E, 0x2C, 0x30,
		    0x2C, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00,
17, DCS_WRITE_SYNC, 0xFB, 0x19, 0x18, 0x08, 0x0F, 0x18, 0x26, 0x1E, 0x2C, 0x30,
		    0x2C, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00,
2,  DCS_WRITE_SYNC, 0xF9, 0x01,
17, DCS_WRITE_SYNC, 0xFA, 0x19, 0x19, 0x09, 0x0D, 0x12, 0x21, 0x1B, 0x2E, 0x31,
		    0x2E, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00,
17, DCS_WRITE_SYNC, 0xFB, 0x19, 0x19, 0x09, 0x0D, 0x12, 0x21, 0x1B, 0x2E, 0x31,
		    0x2E, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00,
3,  DCS_WRITE_SYNC, 0xF0, 0x00, 0x00,
3,  DCS_WRITE_SYNC, 0xF1, 0x00, 0x00,
2,  DCS_WRITE_SYNC, 0x36, 0xD8,
2,  DCS_WRITE_SYNC, 0x3A, 0x06,
0
};

static u8 panel_init_ssd2848_320x320[] = {
/*n, type, data_0, data_1 ... data_n-1*/
2, GENERIC_WRITE_SYNC, 0xFF, 0x00,
6, GENERIC_WRITE_SYNC, 0x00, 0x08, 0x01, 0xF4, 0x04, 0x29,
6, GENERIC_WRITE_SYNC, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x0B,
6, GENERIC_WRITE_SYNC, 0x00, 0x14, 0x0C, 0x01, 0x80, 0x0F,
1, DCS_WRITE_SYNC, MIPI_DCS_EXIT_SLEEP_MODE,
1, WAIT_MS, 1,
6, GENERIC_WRITE_SYNC, 0x10, 0x08, 0x01, 0x20, 0x01, 0x45,
6, GENERIC_WRITE_SYNC, 0x20, 0x0C, 0x00, 0x00, 0x00, 0x02,
6, GENERIC_WRITE_SYNC, 0x20, 0x10, 0x00, 0x1B, 0x00, 0x0C,
6, GENERIC_WRITE_SYNC, 0x20, 0x14, 0x01, 0x88, 0x00, 0x28,
6, GENERIC_WRITE_SYNC, 0x20, 0x18, 0x01, 0x4E, 0x00, 0x0A,
6, GENERIC_WRITE_SYNC, 0x20, 0x20, 0x01, 0x40, 0x01, 0x40,
6, GENERIC_WRITE_SYNC, 0x20, 0x24, 0x01, 0x40, 0x01, 0x40,
6, GENERIC_WRITE_SYNC, 0x20, 0x30, 0x00, 0x00, 0x00, 0x15,
6, GENERIC_WRITE_SYNC, 0x20, 0x34, 0x00, 0x00, 0x00, 0x00,
6, GENERIC_WRITE_SYNC, 0x20, 0x38, 0x01, 0x3F, 0x01, 0x3F,
6, GENERIC_WRITE_SYNC, 0x20, 0x3C, 0x01, 0x40, 0x01, 0x40,
6, GENERIC_WRITE_SYNC, 0x20, 0xA0, 0x00, 0x00, 0x05, 0x00,
5, DCS_WRITE_SYNC, 0x2A, 0x00, 0x00, 0x01, 0x3F,
5, DCS_WRITE_SYNC, 0x2B, 0x00, 0x00, 0x01, 0x3F,
6, GENERIC_WRITE_SYNC, 0x60, 0x08, 0x00, 0x02, 0x00, 0x0A,
6, GENERIC_WRITE_SYNC, 0x60, 0x0C, 0x0A, 0x2A, 0x02, 0x0A,
6, GENERIC_WRITE_SYNC, 0x60, 0x10, 0x01, 0x40, 0x02, 0x14,
6, GENERIC_WRITE_SYNC, 0x60, 0x14, 0x01, 0x00, 0x01, 0x00,
6, GENERIC_WRITE_SYNC, 0x60, 0x40, 0x3F, 0x0B, 0x23, 0x0A,
6, GENERIC_WRITE_SYNC, 0x60, 0x44, 0x1F, 0x1F, 0x07, 0x29,
6, GENERIC_WRITE_SYNC, 0x60, 0x84, 0x00, 0x00, 0x01, 0x40,
2, GENERIC_WRITE_SYNC, 0xFF, 0x01,
2, DCS_WRITE_SYNC, 0x36, 0x10,
3, DCS_WRITE_SYNC, 0xF0, 0x54, 0x47,
2, DCS_WRITE_SYNC, 0xA0, 0x00,
4, DCS_WRITE_SYNC, 0xBD, 0x00, 0x11, 0x31,
2, DCS_WRITE_SYNC, 0xE9, 0x46,
5, DCS_WRITE_SYNC, 0xBA, 0x07, 0x15, 0x3E, 0x01,
6, DCS_WRITE_SYNC, 0xB3, 0x02, 0x0A, 0x14, 0x2A, 0x2A,
5, DCS_WRITE_SYNC, 0xB5, 0x78, 0x78, 0x76, 0xF6,
18, DCS_WRITE_SYNC, 0xC0, 0x00, 0x03, 0x11, 0x12, 0x17, 0x25, 0x0D, 0x0C, 0x0A,
		    0x0E, 0x0C, 0x2F, 0x09, 0x0E, 0x3D, 0x3E, 0x3F,
18, DCS_WRITE_SYNC, 0xC1, 0x00, 0x03, 0x11, 0x12, 0x17, 0x25, 0x0D, 0x0C, 0x0A,
		    0x0E, 0x0C, 0x2F, 0x09, 0x0E, 0x3D, 0x3E, 0x3F,
18, DCS_WRITE_SYNC, 0xC2, 0x00, 0x03, 0x11, 0x12, 0x17, 0x25, 0x0D, 0x0C, 0x0A,
		    0x0E, 0x0C, 0x2F, 0x09, 0x0E, 0x3D, 0x3E, 0x3F,
18, DCS_WRITE_SYNC, 0xC3, 0x00, 0x03, 0x11, 0x12, 0x17, 0x25, 0x0D, 0x0C, 0x0A,
		    0x0E, 0x0C, 0x2F, 0x09, 0x0E, 0x3D, 0x3E, 0x3F,
18, DCS_WRITE_SYNC, 0xC4, 0x00, 0x03, 0x11, 0x12, 0x17, 0x25, 0x0D, 0x0C, 0x0A,
		    0x0E, 0x0C, 0x2F, 0x09, 0x0E, 0x3D, 0x3E, 0x3F,
18, DCS_WRITE_SYNC, 0xC5, 0x00, 0x03, 0x11, 0x12, 0x17, 0x25, 0x0D, 0x0C, 0x0A,
		    0x0E, 0x0C, 0x2F, 0x09, 0x0E, 0x3D, 0x3E, 0x3F,
1, DCS_WRITE_SYNC, MIPI_DCS_EXIT_SLEEP_MODE,
1, WAIT_MS, 120,
1, DCS_WRITE_SYNC, MIPI_DCS_SET_DISPLAY_ON,
2, GENERIC_WRITE_SYNC, 0xFF, 0x00,
0
};

struct minnow_panel_clk_range {
	int	min;
	int	max;
};

struct minnow_panel_hw_reset {
	int	active;
	int	reset_ms;
	int	wait_ms;
};

struct minnow_panel_attr {
	int	mode;
	int	xres;
	int	yres;
	int	pixel_clock;
	int	pixel_format;
	int	xoffset;
	int	yoffset;
	int	init_cmd_count;
	u8	*init_cmd;
	struct minnow_panel_clk_range hs;
	struct minnow_panel_clk_range lp;
	struct minnow_panel_hw_reset panel_reset;
	struct minnow_panel_hw_reset bridge_reset;
};

#define	INIT_CMD(buf) 	.init_cmd_count = sizeof(buf), .init_cmd = (buf)
static struct minnow_panel_attr panel_attr_table[MINNOW_PANEL_MAX] = {
	[MINNOW_PANEL_CM_220X176] = {
		.mode = OMAP_DSS_DSI_CMD_MODE,
		.xres = 220,
		.yres = 176,
		.pixel_clock = 4608,
		.pixel_format = OMAP_DSS_DSI_FMT_RGB666,
		.xoffset = 0x32,
		.yoffset = 0,
		INIT_CMD(panel_init_220x176),
		.hs = { 100000000, 150000000 },
		.lp = { 7000000, 9000000 },
		.panel_reset = { 0, 1, 5 },
		.bridge_reset = { 0, 0, 0 },
	},
	[MINNOW_PANEL_CM_220X220] = {
		.mode = OMAP_DSS_DSI_CMD_MODE,
		.xres = 220,
		.yres = 220,
		.pixel_clock = 4608,
		.pixel_format = OMAP_DSS_DSI_FMT_RGB666,
		.xoffset = 0x32,
		.yoffset = 0x4,
		INIT_CMD(panel_init_220x220),
		.hs = { 100000000, 150000000 },
		.lp = { 7000000, 9000000 },
		.panel_reset = { 0, 1, 5 },
		.bridge_reset = { 0, 0, 0 },
	},
	[MINNOW_PANEL_CM_BRIDGE_320X320] = {
		.mode = OMAP_DSS_DSI_CMD_MODE,
		.xres = 320,
		.yres = 290, /* actual display height is 290 */
		.pixel_clock = DIV_ROUND_UP(320 * 290 * 60, 1000),
		.pixel_format = OMAP_DSS_DSI_FMT_RGB888,
		.xoffset = 0,
		.yoffset = 0,
		INIT_CMD(panel_init_ssd2848_320x320),
		.hs = { 100000000, 150000000 },
		.lp = { 7000000, 9000000 },
		.panel_reset = { 0, 5, 10 },
		.bridge_reset = { 0, 20, 10 }
	},
};


static irqreturn_t minnow_panel_te_isr(int irq, void *data);
static void minnow_panel_te_timeout_work_callback(struct work_struct *work);
static int _minnow_panel_enable_te(struct omap_dss_device *dssdev, bool enable);

static int minnow_panel_reset(struct omap_dss_device *dssdev);


struct minnow_panel_data {
	struct mutex lock; /* mutex */

	struct backlight_device *bldev;

	unsigned long	hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long	hw_guard_wait;	/* max guard time in jiffies */

	struct omap_dss_device *dssdev;

	/* panel HW configuration from DT or platform data */
	int reset_gpio[MINNOW_RESET_MAX];
	int ext_te_gpio;
	int clk_en_gpio;
	struct minnow_panel_hw_reset hw_reset[MINNOW_RESET_MAX];
	struct minnow_panel_hw_reset bridge_reset;
	struct regulator *bridge_regulator;
	struct regulator *panel_regulator;

	bool use_dsi_backlight;

	struct omap_dsi_pin_config pin_config;
	struct omap_dss_dsi_config dsi_config;

	u8 *init_cmd_data;
	int init_cmd_count;
	int id_panel;
	int x_offset;
	int y_offset;

	/* runtime variables */
	bool enabled;

	bool te_enabled;

	atomic_t do_update;
	int channel;

	struct delayed_work te_timeout_work;

	unsigned cabc_mode;

	bool intro_printed;

	struct workqueue_struct *workqueue;

	struct delayed_work esd_work;
	unsigned esd_interval;

	bool ulps_enabled;
	unsigned ulps_timeout;
	struct delayed_work ulps_work;
};

#ifdef	CONFIG_OMAP2_DSS_DEBUGFS
static void minnow_panel_dump_regs(struct seq_file *s)
{
	static struct {char name[8]; int addr; int endreg; } regs[] = {
		{"SCM",		0x0000, 0x30},
		{"MIPIRX",	0x1000, 0x30},
		{"VTCM",	0x2000, 0xB4},
		{"VCU",		0x4000, 0x20},
		{"GPIO",	0x5000, 0x04},
		{"MIPITX",	0x6000, 0x54},
		{"TX-DSI0",	0x6080, 0x14},
	};
	struct omap_dss_output *out = omap_dss_get_output(OMAP_DSS_OUTPUT_DSI1);
	struct platform_device *dsidev = out->pdev;
	struct omap_dss_device *dssdev = out->device;
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int i, j, r;

	dsi_bus_lock(dssdev);

	if (dsi_runtime_get(dsidev)) {
		seq_puts(s, "dsi_runtime_get failed!");
		goto exit1;
	}

	if (mpd->dssdev != dssdev) {
		seq_puts(s, "dssdev mis-matched!");
		goto exit;
	}

	if (dsi_vc_set_max_rx_packet_size(dssdev, mpd->channel, 4)) {
		seq_puts(s, "failed set max rx_packet_size 4");
		goto exit;
	}

	for (i = 0; i < sizeof(regs)/sizeof(regs[0]); i++) {
		seq_printf(s, "%s Registers:\n", regs[i].name);
		for (j = 0; j <= regs[i].endreg; j += 4) {
			u8 reg[4];
			u16 addr = j + regs[i].addr;
			seq_printf(s, "  %04X: ", addr);
			r = dsi_vc_generic_read_2(dssdev, mpd->channel,
						addr>>8, addr&0xFF, reg, 4);
			if (r)
				seq_printf(s, "read failed ret = %d\n", r);
			else
				seq_printf(s, "%02X%02X%02X%02X\n",
					   reg[0], reg[1], reg[2], reg[3]);
		}
	}

	dsi_vc_set_max_rx_packet_size(dssdev, mpd->channel, 1);

exit:
	dsi_runtime_put(dsidev);
exit1:
	dsi_bus_unlock(dssdev);
}
#endif

static void minnow_panel_esd_work(struct work_struct *work);
static void minnow_panel_ulps_work(struct work_struct *work);

static void hw_guard_start(struct minnow_panel_data *mpd, int guard_msec)
{
	mpd->hw_guard_wait = msecs_to_jiffies(guard_msec);
	mpd->hw_guard_end = jiffies + mpd->hw_guard_wait;
}

static void hw_guard_wait(struct minnow_panel_data *mpd)
{
	unsigned long wait = mpd->hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= mpd->hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static int minnow_panel_dcs_read_1(struct minnow_panel_data *mpd, u8 dcs_cmd, u8 *data)
{
	int r;
	u8 buf[1];

	r = dsi_vc_dcs_read(mpd->dssdev, mpd->channel, dcs_cmd, buf, 1);

	if (r < 0)
		return r;

	*data = buf[0];

	return 0;
}

static int minnow_panel_dcs_write_0(struct minnow_panel_data *mpd, u8 dcs_cmd)
{
	return dsi_vc_dcs_write(mpd->dssdev, mpd->channel, &dcs_cmd, 1);
}

static int minnow_panel_dcs_write_1(struct minnow_panel_data *mpd, u8 dcs_cmd, u8 param)
{
	u8 buf[2];
	buf[0] = dcs_cmd;
	buf[1] = param;
	return dsi_vc_dcs_write(mpd->dssdev, mpd->channel, buf, 2);
}

static int minnow_panel_sleep_in(struct minnow_panel_data *mpd)

{
	u8 cmd;
	int r;

	hw_guard_wait(mpd);

	cmd = MIPI_DCS_ENTER_SLEEP_MODE;
	r = dsi_vc_dcs_write_nosync(mpd->dssdev, mpd->channel, &cmd, 1);
	if (r)
		return r;

	hw_guard_start(mpd, 120);

	msleep(10);

	return 0;
}

static int minnow_panel_sleep_out(struct minnow_panel_data *mpd)
{
	int r;

	hw_guard_wait(mpd);

	r = minnow_panel_dcs_write_0(mpd, MIPI_DCS_EXIT_SLEEP_MODE);
	if (r)
		return r;

	hw_guard_start(mpd, 120);

	msleep(10);

	return 0;
}

static int minnow_panel_get_bridge_rev(struct minnow_panel_data *mpd,
					u8 *id1, u8 *id2, u8 *id3)
{
	int r = dsi_vc_set_max_rx_packet_size(mpd->dssdev, mpd->channel, 4);

	if (!r) {
		u8 reg[4];
		r = dsi_vc_generic_read_2(mpd->dssdev,
						mpd->channel, 0, 0, reg, 4);
		if (!r) {
			*id1 = reg[0];
			*id2 = reg[1];
			*id3 = reg[3];
		}
	}
	dsi_vc_set_max_rx_packet_size(mpd->dssdev, mpd->channel, 1);

	return r;
}

static int minnow_panel_get_id(struct minnow_panel_data *mpd,
					u8 *id1, u8 *id2, u8 *id3)
{
	int r;

	if (mpd->id_panel == MINNOW_PANEL_CM_BRIDGE_320X320)
		return minnow_panel_get_bridge_rev(mpd, id1, id2, id3);

	r = minnow_panel_dcs_read_1(mpd, DCS_GET_ID1, id1);
	if (r)
		return r;
	r = minnow_panel_dcs_read_1(mpd, DCS_GET_ID2, id2);
	if (r)
		return r;
	r = minnow_panel_dcs_read_1(mpd, DCS_GET_ID3, id3);
	if (r)
		return r;


	return 0;
}

static int _minnow_panel_init(struct minnow_panel_data *mpd)
{
	u8 *data;
	int i, r;

	for (i = 0, data = mpd->init_cmd_data; *data; ) {
		i += ((u32)*data + 2);
		if (i >= mpd->init_cmd_count)
			break;
		if (data[1] >= CMD_TYPE_MAX)
			break;
		data += (*data + 2);
	}

	/* Init command data shall end with 0 */
	if (*data) {
		dev_err(&mpd->dssdev->dev, "Invalid panel initialize data\n");
		return -EINVAL;
	}

	for (i = 0, data = mpd->init_cmd_data; *data; i++) {
		int len = (u32)*data;
		u8 cmd = data[1];
		data += 2;
		switch (cmd) {
		case DCS_WRITE_SYNC:
			r = dsi_vc_dcs_write(mpd->dssdev,
					mpd->channel, data, len);
			break;
		case GENERIC_WRITE_SYNC:
			r = dsi_vc_generic_write(mpd->dssdev,
					mpd->channel, data, len);
			break;
		case DCS_WRITE:
			r = dsi_vc_dcs_write_nosync(mpd->dssdev,
					mpd->channel, data, len);
			break;
		case GENERIC_WRITE:
			r = dsi_vc_generic_write_nosync(mpd->dssdev,
					mpd->channel, data, len);
			break;
		case BTA_SYNC:
			r = dsi_vc_send_bta_sync(mpd->dssdev, mpd->channel);
			break;
		case WAIT_MS:
			msleep((len == 1 ? (u32)(*data) : *(u16 *)data));
			break;
		}
		if (r) {
			dev_err(&mpd->dssdev->dev, "Failed process initialize"
				"command[%d] len=%d type=%d ret=%d\n",
				i, len, cmd, r);
			break;
		}
		data += len;
	}

	return r;
}

static int minnow_panel_set_update_window(struct minnow_panel_data *mpd,
		u16 x, u16 y, u16 w, u16 h)
{
	int r;
	u16 x1 = x + mpd->x_offset;
	u16 x2 = x + mpd->x_offset + w - 1;
	u16 y1 = y + mpd->y_offset;
	u16 y2 = y + mpd->y_offset + h - 1;

	u8 buf[5];
	buf[0] = MIPI_DCS_SET_COLUMN_ADDRESS;
	buf[1] = (x1 >> 8) & 0xff;
	buf[2] = (x1 >> 0) & 0xff;
	buf[3] = (x2 >> 8) & 0xff;
	buf[4] = (x2 >> 0) & 0xff;

	r = dsi_vc_dcs_write_nosync(mpd->dssdev, mpd->channel, buf, sizeof(buf));
	if (r)
		return r;

	buf[0] = MIPI_DCS_SET_PAGE_ADDRESS;
	buf[1] = (y1 >> 8) & 0xff;
	buf[2] = (y1 >> 0) & 0xff;
	buf[3] = (y2 >> 8) & 0xff;
	buf[4] = (y2 >> 0) & 0xff;

	r = dsi_vc_dcs_write_nosync(mpd->dssdev, mpd->channel, buf, sizeof(buf));
	if (r)
		return r;

	dsi_vc_send_bta_sync(mpd->dssdev, mpd->channel);

	return r;
}

static void minnow_panel_queue_esd_work(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	if (mpd->esd_interval > 0)
		queue_delayed_work(mpd->workqueue, &mpd->esd_work,
				msecs_to_jiffies(mpd->esd_interval));
}

static void minnow_panel_cancel_esd_work(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	cancel_delayed_work(&mpd->esd_work);
}

static void minnow_panel_queue_ulps_work(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	if (mpd->ulps_timeout > 0)
		queue_delayed_work(mpd->workqueue, &mpd->ulps_work,
				msecs_to_jiffies(mpd->ulps_timeout));
}

static void minnow_panel_cancel_ulps_work(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	cancel_delayed_work(&mpd->ulps_work);
}

static int minnow_panel_enter_ulps(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int r;

	if (mpd->ulps_enabled)
		return 0;

	minnow_panel_cancel_ulps_work(dssdev);

	r = _minnow_panel_enable_te(dssdev, false);
	if (r)
		goto err;

	if (gpio_is_valid(mpd->ext_te_gpio))
		disable_irq(gpio_to_irq(mpd->ext_te_gpio));

	omapdss_dsi_display_disable(dssdev, false, true);

	mpd->ulps_enabled = true;

	return 0;

err:
	dev_err(&dssdev->dev, "enter ULPS failed");
	minnow_panel_reset(dssdev);

	mpd->ulps_enabled = false;

	minnow_panel_queue_ulps_work(dssdev);

	return r;
}

static int minnow_panel_exit_ulps(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int r;

	if (!mpd->ulps_enabled)
		return 0;

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err1;
	}

	omapdss_dsi_vc_enable_hs(dssdev, mpd->channel, true);

	r = _minnow_panel_enable_te(dssdev, true);
	if (r) {
		dev_err(&dssdev->dev, "failed to re-enable TE");
		goto err2;
	}

	if (gpio_is_valid(mpd->ext_te_gpio))
		enable_irq(gpio_to_irq(mpd->ext_te_gpio));

	minnow_panel_queue_ulps_work(dssdev);

	mpd->ulps_enabled = false;

	return 0;

err2:
	dev_err(&dssdev->dev, "failed to exit ULPS");

	r = minnow_panel_reset(dssdev);
	if (!r) {
		if (gpio_is_valid(mpd->ext_te_gpio))
			enable_irq(gpio_to_irq(mpd->ext_te_gpio));
		mpd->ulps_enabled = false;
	}
err1:
	minnow_panel_queue_ulps_work(dssdev);

	return r;
}

static int minnow_panel_wake_up(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	if (mpd->ulps_enabled)
		return minnow_panel_exit_ulps(dssdev);

	minnow_panel_cancel_ulps_work(dssdev);
	minnow_panel_queue_ulps_work(dssdev);
	return 0;
}

static int minnow_panel_bl_update_status(struct backlight_device *dev)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&dev->dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int r;
	int level;

	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		level = dev->props.brightness;
	else
		level = 0;

	dev_dbg(&dssdev->dev, "update brightness to %d\n", level);

	mutex_lock(&mpd->lock);

	if (mpd->enabled) {
		dsi_bus_lock(dssdev);

		r = minnow_panel_wake_up(dssdev);
		if (!r)
			r = minnow_panel_dcs_write_1(mpd, DCS_BRIGHTNESS, level);

		dsi_bus_unlock(dssdev);
	} else {
		r = 0;
	}

	mutex_unlock(&mpd->lock);

	return r;
}

static int minnow_panel_bl_get_intensity(struct backlight_device *dev)
{
	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		return dev->props.brightness;

	return 0;
}

static const struct backlight_ops minnow_panel_bl_ops = {
	.get_brightness = minnow_panel_bl_get_intensity,
	.update_status  = minnow_panel_bl_update_status,
};

static void minnow_panel_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static ssize_t minnow_panel_num_errors_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	u8 errors = 0;
	int r;

	mutex_lock(&mpd->lock);

	if (mpd->enabled) {
		dsi_bus_lock(dssdev);

		r = minnow_panel_wake_up(dssdev);
		if (!r)
			r = minnow_panel_dcs_read_1(mpd, DCS_READ_NUM_ERRORS, &errors);

		dsi_bus_unlock(dssdev);
	} else {
		r = -ENODEV;
	}

	mutex_unlock(&mpd->lock);

	if (r)
		return r;

	return snprintf(buf, PAGE_SIZE, "%d\n", errors);
}

static ssize_t minnow_panel_hw_revision_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	u8 id1, id2, id3;
	int r;

	mutex_lock(&mpd->lock);

	if (mpd->enabled) {
		dsi_bus_lock(dssdev);

		r = minnow_panel_wake_up(dssdev);
		if (!r)
			r = minnow_panel_get_id(mpd, &id1, &id2, &id3);

		dsi_bus_unlock(dssdev);
	} else {
		r = -ENODEV;
	}

	mutex_unlock(&mpd->lock);

	if (r)
		return r;

	return snprintf(buf, PAGE_SIZE, "%02x.%02x.%02x\n", id1, id2, id3);
}

static const char *cabc_modes[] = {
	"off",		/* used also always when CABC is not supported */
	"ui",
	"still-image",
	"moving-image",
};

static ssize_t show_cabc_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	const char *mode_str;
	int mode;
	int len;

	mode = mpd->cabc_mode;

	mode_str = "unknown";
	if (mode >= 0 && mode < ARRAY_SIZE(cabc_modes))
		mode_str = cabc_modes[mode];
	len = snprintf(buf, PAGE_SIZE, "%s\n", mode_str);

	return len < PAGE_SIZE - 1 ? len : PAGE_SIZE - 1;
}

static ssize_t store_cabc_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int i;
	int r;

	for (i = 0; i < ARRAY_SIZE(cabc_modes); i++) {
		if (sysfs_streq(cabc_modes[i], buf))
			break;
	}

	if (i == ARRAY_SIZE(cabc_modes))
		return -EINVAL;

	mutex_lock(&mpd->lock);

	if (mpd->enabled) {
		dsi_bus_lock(dssdev);

		r = minnow_panel_wake_up(dssdev);
		if (r)
			goto err;

		r = minnow_panel_dcs_write_1(mpd, DCS_WRITE_CABC, i);
		if (r)
			goto err;

		dsi_bus_unlock(dssdev);
	}

	mpd->cabc_mode = i;

	mutex_unlock(&mpd->lock);

	return count;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&mpd->lock);
	return r;
}

static ssize_t show_cabc_available_modes(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int len;
	int i;

	for (i = 0, len = 0;
	     len < PAGE_SIZE && i < ARRAY_SIZE(cabc_modes); i++)
		len += snprintf(&buf[len], PAGE_SIZE - len, "%s%s%s",
			i ? " " : "", cabc_modes[i],
			i == ARRAY_SIZE(cabc_modes) - 1 ? "\n" : "");

	return len < PAGE_SIZE ? len : PAGE_SIZE - 1;
}

static ssize_t minnow_panel_store_esd_interval(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&mpd->lock);
	minnow_panel_cancel_esd_work(dssdev);
	mpd->esd_interval = t;
	if (mpd->enabled)
		minnow_panel_queue_esd_work(dssdev);
	mutex_unlock(&mpd->lock);

	return count;
}

static ssize_t minnow_panel_show_esd_interval(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	unsigned t;

	mutex_lock(&mpd->lock);
	t = mpd->esd_interval;
	mutex_unlock(&mpd->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

static ssize_t minnow_panel_store_ulps(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&mpd->lock);

	if (mpd->enabled) {
		dsi_bus_lock(dssdev);

		if (t)
			r = minnow_panel_enter_ulps(dssdev);
		else
			r = minnow_panel_wake_up(dssdev);

		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&mpd->lock);

	if (r)
		return r;

	return count;
}

static ssize_t minnow_panel_show_ulps(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	unsigned t;

	mutex_lock(&mpd->lock);
	t = mpd->ulps_enabled;
	mutex_unlock(&mpd->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

static ssize_t minnow_panel_store_ulps_timeout(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&mpd->lock);
	mpd->ulps_timeout = t;

	if (mpd->enabled) {
		/* minnow_panel_wake_up will restart the timer */
		dsi_bus_lock(dssdev);
		r = minnow_panel_wake_up(dssdev);
		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&mpd->lock);

	if (r)
		return r;

	return count;
}

static ssize_t minnow_panel_show_ulps_timeout(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	unsigned t;

	mutex_lock(&mpd->lock);
	t = mpd->ulps_timeout;
	mutex_unlock(&mpd->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

static DEVICE_ATTR(num_dsi_errors, S_IRUGO, minnow_panel_num_errors_show, NULL);
static DEVICE_ATTR(hw_revision, S_IRUGO, minnow_panel_hw_revision_show, NULL);
static DEVICE_ATTR(cabc_mode, S_IRUGO | S_IWUSR,
		show_cabc_mode, store_cabc_mode);
static DEVICE_ATTR(cabc_available_modes, S_IRUGO,
		show_cabc_available_modes, NULL);
static DEVICE_ATTR(esd_interval, S_IRUGO | S_IWUSR,
		minnow_panel_show_esd_interval, minnow_panel_store_esd_interval);
static DEVICE_ATTR(ulps, S_IRUGO | S_IWUSR,
		minnow_panel_show_ulps, minnow_panel_store_ulps);
static DEVICE_ATTR(ulps_timeout, S_IRUGO | S_IWUSR,
		minnow_panel_show_ulps_timeout, minnow_panel_store_ulps_timeout);

static struct attribute *minnow_panel_attrs[] = {
	&dev_attr_num_dsi_errors.attr,
	&dev_attr_hw_revision.attr,
	&dev_attr_cabc_mode.attr,
	&dev_attr_cabc_available_modes.attr,
	&dev_attr_esd_interval.attr,
	&dev_attr_ulps.attr,
	&dev_attr_ulps_timeout.attr,
	NULL,
};

static struct attribute_group minnow_panel_attr_group = {
	.attrs = minnow_panel_attrs,
};

static void minnow_panel_hw_reset(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int i, ms_rst = -1, ms_rel = -1;

	for (i = 0; i < MINNOW_RESET_MAX; i++) {
		if (!gpio_is_valid(mpd->reset_gpio[i]))
			continue;
		gpio_set_value(mpd->reset_gpio[i],
			       mpd->hw_reset[i].active ? 0 : 1);
		if (ms_rst < mpd->hw_reset[i].reset_ms)
			ms_rst = mpd->hw_reset[i].reset_ms;
		if (ms_rel < mpd->hw_reset[i].wait_ms)
			ms_rel = mpd->hw_reset[i].wait_ms;
	}
	if (ms_rst == -1)
		return;
	msleep(5);

	/* reset the device */
	for (i = 0; i < MINNOW_RESET_MAX; i++) {
		if (!gpio_is_valid(mpd->reset_gpio[i]))
			continue;
		gpio_set_value(mpd->reset_gpio[i],
			       mpd->hw_reset[i].active ? 1 : 0);
	}
	/* wait device reset */
	msleep(ms_rst);
	/* assert reset */
	for (i = 0; i < MINNOW_RESET_MAX; i++) {
		if (!gpio_is_valid(mpd->reset_gpio[i]))
			continue;
		gpio_set_value(mpd->reset_gpio[i],
			       mpd->hw_reset[i].active ? 0 : 1);
	}
	/* wait after releasing reset */
	msleep(ms_rel);
}

#define	DEBUG_DT
#ifdef DEBUG_DT
#define DTINFO(fmt, ...) \
	printk(KERN_INFO "minnow-panel DT: " fmt, ## __VA_ARGS__)
#define DTINFO_PIXFMT(msg, pix) \
{ char *fmt[4] = {"RGB888", "RGB666", "RGB666_PACKED", "RGB565"};\
  DTINFO(msg"%s\n", fmt[pix]);\
}
#define DTINFO_ARRAY(msg, a, n, fmt, blen) \
{ int i; char str[blen], *p = str;\
	for (i = 0; i < n; i++) {\
		sprintf(p, fmt, a[i]);\
		p += strlen(p); \
	} \
	DTINFO(msg"%s\n", str);\
}
#else /* DEBUG_DT */
#define DTINFO(fmt, ...)
#define DTINFO_PIXFMT(msg, pix)
#define DTINFO_ARRAY(msg, a, n, fmt, blen)
#endif

static struct of_device_id minnow_panel_ids[] = {
	{ .compatible = "mot,minnow-panel-dsi-cm" },
	{ /*sentinel*/ }
};

static int minnow_panel_dt_init(struct minnow_panel_data *mpd)
{
	u32 range[2], value = 0;
	struct minnow_panel_attr *panel_attr;
	struct device_node *dt_node;

	dt_node = of_find_matching_node(NULL, minnow_panel_ids);
	if (dt_node == NULL) {
		dev_err(&mpd->dssdev->dev, "No dt_node found!\n");
		return -ENODEV;
	}

	/* Save the dt node entry to the device */
	mpd->dssdev->dev.of_node = dt_node;

	if (of_property_read_u32(dt_node, "id_panel", &value) \
		|| (value >= MINNOW_PANEL_MAX)) {
		dev_err(&mpd->dssdev->dev, \
			"Invalid id_panel = %u!\n", value);
		return -EINVAL;
	}
	mpd->id_panel = value;
	DTINFO("id_panel = %d\n", mpd->id_panel);

	panel_attr = &panel_attr_table[mpd->id_panel];
	mpd->init_cmd_data = panel_attr->init_cmd;
	mpd->init_cmd_count = panel_attr->init_cmd_count;
	mpd->dssdev->caps = OMAP_DSS_DISPLAY_CAP_MANUAL_UPDATE |
		OMAP_DSS_DISPLAY_CAP_TEAR_ELIM;
	mpd->dssdev->panel.timings.x_res = panel_attr->xres;
	mpd->dssdev->panel.timings.y_res = panel_attr->yres;
	mpd->dssdev->panel.timings.pixel_clock = panel_attr->pixel_clock;
	mpd->dssdev->panel.dsi_pix_fmt = panel_attr->pixel_format;
	mpd->dsi_config.mode = panel_attr->mode;
	mpd->dsi_config.pixel_format = panel_attr->pixel_format;
	mpd->dsi_config.hs_clk_min = panel_attr->hs.min;
	mpd->dsi_config.hs_clk_max = panel_attr->hs.max;
	mpd->dsi_config.lp_clk_min = panel_attr->lp.min;
	mpd->dsi_config.lp_clk_max = panel_attr->lp.max;
	mpd->x_offset = panel_attr->xoffset;
	mpd->y_offset = panel_attr->yoffset;

	mpd->hw_reset[MINNOW_RESET_PANEL] = panel_attr->panel_reset;
	mpd->hw_reset[MINNOW_RESET_BRIDGE] = panel_attr->bridge_reset;
	mpd->reset_gpio[MINNOW_RESET_PANEL] =
		of_get_named_gpio(dt_node, "gpio_panel_reset", 0);
	DTINFO("gpio_panel_reset = %d\n", mpd->reset_gpio[MINNOW_RESET_PANEL]);
	mpd->reset_gpio[MINNOW_RESET_BRIDGE] =
		of_get_named_gpio(dt_node, "gpio_bridge_reset", 0);
	DTINFO("gpio_bridge_reset = %d\n",
	       mpd->reset_gpio[MINNOW_RESET_BRIDGE]);
	mpd->ext_te_gpio = of_get_named_gpio(dt_node, "gpio_te", 0);
	DTINFO("gpio_te = %d\n", mpd->ext_te_gpio);
	mpd->clk_en_gpio = of_get_named_gpio(dt_node, "gpio_clk_en", 0);
	DTINFO("gpio_clk_en = %d\n", mpd->clk_en_gpio);

	mpd->esd_interval = 0;
	if (!of_property_read_u32(dt_node, "esd_interval", &value)) {
		mpd->esd_interval = value;
		DTINFO("esd_interval = %d\n", mpd->esd_interval);
	}
	mpd->ulps_timeout = 0;
	mpd->use_dsi_backlight = false;

	mpd->pin_config.num_pins = 4;
	mpd->pin_config.pins[0] = 0;
	mpd->pin_config.pins[1] = 1;
	mpd->pin_config.pins[2] = 2;
	mpd->pin_config.pins[3] = 3;
	if (of_get_property(dt_node, "pins", &value)) {
		u32 pins[OMAP_DSS_MAX_DSI_PINS];
		u32 num_pins = value / sizeof(u32);
		if (!num_pins || (num_pins > OMAP_DSS_MAX_DSI_PINS)) {
			dev_err(&mpd->dssdev->dev, \
				"Invalid DSI pins count = %u!\n", num_pins);
			return -EINVAL;
		}
		value = 0;
		if (!of_property_read_u32_array(dt_node, \
			"pins", pins, num_pins)) {
			for (; value < num_pins; value++) {
				if (pins[value] >= OMAP_DSS_MAX_DSI_PINS)
					break;
				mpd->pin_config.pins[value]\
					= pins[value];
			}
		}
		if (value < num_pins) {
			dev_err(&mpd->dssdev->dev, \
				"Invalid DSI pins config!\n");
			return -EINVAL;
		}
		mpd->pin_config.num_pins = num_pins;
		DTINFO("num_pins = %d\n", \
			mpd->pin_config.num_pins);
		DTINFO_ARRAY("pins =", mpd->pin_config.pins,\
			mpd->pin_config.num_pins, " %u", 64);
	}

	if (!of_property_read_u32(dt_node, "pixel_clock", &value)) {
		if (value < mpd->dssdev->panel.timings.pixel_clock) {
			dev_err(&mpd->dssdev->dev, \
				"Invalid pixel_clock = %u!\n", value);
			return -EINVAL;
		}
		mpd->dssdev->panel.timings.pixel_clock = value;
		DTINFO("pixel_clock = %u\n", \
			mpd->dssdev->panel.timings.pixel_clock);
	}

	if (!of_property_read_u32(dt_node, "pixel_format", &value)) {
		switch (value) {
		case OMAP_DSS_DSI_FMT_RGB888:
		case OMAP_DSS_DSI_FMT_RGB666:
		case OMAP_DSS_DSI_FMT_RGB666_PACKED:
		case OMAP_DSS_DSI_FMT_RGB565:
			break;
		default:
			dev_err(&mpd->dssdev->dev, \
				"Invalid pixel_format = %u!\n", value);
			return -EINVAL;
		}
		mpd->dssdev->panel.dsi_pix_fmt = \
			mpd->dsi_config.pixel_format = value;
		DTINFO_PIXFMT("pixel_format = ", \
			mpd->dssdev->panel.dsi_pix_fmt);
	}

	if (!of_property_read_u32_array(dt_node, "hs_clk", range, 2)) {
		mpd->dsi_config.hs_clk_min = range[0];
		mpd->dsi_config.hs_clk_max = range[1];
		DTINFO("hs_clk_min = %lu, hs_clk_max = %lu\n", \
			mpd->dsi_config.hs_clk_min, \
			mpd->dsi_config.hs_clk_max);
	}

	if (!of_property_read_u32_array(dt_node, "lp_clk", range, 2)) {
		mpd->dsi_config.lp_clk_min = range[0];
		mpd->dsi_config.lp_clk_max = range[1];
		DTINFO("lp_clk_min = %lu, lp_clk_max = %lu\n", \
			mpd->dsi_config.lp_clk_min, \
			mpd->dsi_config.lp_clk_max);
	}

	return 0;
}

static int minnow_panel_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct minnow_panel_data *mpd;
	struct backlight_device *bldev = NULL;
	int i, r;

	dev_dbg(&dssdev->dev, "probe\n");

	mpd = devm_kzalloc(&dssdev->dev, sizeof(*mpd), GFP_KERNEL);
	if (!mpd)
		return -ENOMEM;

	dev_set_drvdata(&dssdev->dev, mpd);
	mpd->dssdev = dssdev;

	r = minnow_panel_dt_init(mpd);
	if (r)
		return r;

	mutex_init(&mpd->lock);

	atomic_set(&mpd->do_update, 0);

	if (gpio_is_valid(mpd->clk_en_gpio)) {
		r = devm_gpio_request_one(&dssdev->dev, mpd->clk_en_gpio,
				GPIOF_OUT_INIT_HIGH, "minnow-panel clk_en");
		if (r) {
			dev_err(&dssdev->dev, "failed to request panel clk_en gpio\n");
			return r;
		}
	}

	for (i = 0; i < MINNOW_RESET_MAX; i++) {
		static const char * const name[MINNOW_RESET_MAX] = {
			"minnow-panel reset",
			"minnow-bridge reset"
		};
		if (!gpio_is_valid(mpd->reset_gpio[i]))
			continue;
		r = devm_gpio_request_one(&dssdev->dev, mpd->reset_gpio[i],
			mpd->hw_reset[i].active ? GPIOF_OUT_INIT_LOW
			: GPIOF_OUT_INIT_HIGH, name[i]);
		if (r) {
			dev_err(&dssdev->dev,
				"failed to request %s gpio\n", name[i]);
			return r;
		}
	}

	if (gpio_is_valid(mpd->ext_te_gpio)) {
		r = devm_gpio_request_one(&dssdev->dev, mpd->ext_te_gpio,
				GPIOF_IN, "minnow-panel irq");
		if (r) {
			dev_err(&dssdev->dev,
				"failed to request ext_te gpio\n");
			return r;
		}

		r = devm_request_irq(&dssdev->dev,
				gpio_to_irq(mpd->ext_te_gpio),
				minnow_panel_te_isr,
				IRQF_TRIGGER_RISING,
				"minnow-panel vsync", dssdev);

		if (r) {
			dev_err(&dssdev->dev, "IRQ request failed\n");
			return r;
		}

		INIT_DEFERRABLE_WORK(&mpd->te_timeout_work,
					minnow_panel_te_timeout_work_callback);

		dev_dbg(&dssdev->dev, "Using GPIO TE\n");
	}

	mpd->bridge_regulator = devm_regulator_get(&dssdev->dev, "bridge");
	if (IS_ERR(mpd->bridge_regulator)) {
		mpd->bridge_regulator = NULL;
		dev_info(&dssdev->dev, "Could not get bridge regulator\n");
	}

	mpd->panel_regulator = devm_regulator_get(&dssdev->dev, "panel");
	if (IS_ERR(mpd->panel_regulator)) {
		mpd->panel_regulator = NULL;
		dev_info(&dssdev->dev, "Could not get panel regulator\n");
	}

	mpd->workqueue = create_singlethread_workqueue("minnow_panel_esd");
	if (mpd->workqueue == NULL) {
		dev_err(&dssdev->dev, "can't create ESD workqueue\n");
		return -ENOMEM;
	}
	INIT_DEFERRABLE_WORK(&mpd->esd_work, minnow_panel_esd_work);
	INIT_DELAYED_WORK(&mpd->ulps_work, minnow_panel_ulps_work);

	if (mpd->use_dsi_backlight) {
		memset(&props, 0, sizeof(struct backlight_properties));
		props.max_brightness = 255;

		props.type = BACKLIGHT_RAW;
		bldev = backlight_device_register(dev_name(&dssdev->dev),
				&dssdev->dev, dssdev, &minnow_panel_bl_ops, &props);
		if (IS_ERR(bldev)) {
			r = PTR_ERR(bldev);
			goto err_bl;
		}

		mpd->bldev = bldev;

		bldev->props.fb_blank = FB_BLANK_UNBLANK;
		bldev->props.power = FB_BLANK_UNBLANK;
		bldev->props.brightness = 255;

		minnow_panel_bl_update_status(bldev);
	}

	r = omap_dsi_request_vc(dssdev, &mpd->channel);
	if (r) {
		dev_err(&dssdev->dev, "failed to get virtual channel\n");
		goto err_req_vc;
	}

	r = omap_dsi_set_vc_id(dssdev, mpd->channel, TCH);
	if (r) {
		dev_err(&dssdev->dev, "failed to set VC_ID\n");
		goto err_vc_id;
	}

	r = sysfs_create_group(&dssdev->dev.kobj, &minnow_panel_attr_group);
	if (r) {
		dev_err(&dssdev->dev, "failed to create sysfs files\n");
		goto err_vc_id;
	}

#ifdef	CONFIG_OMAP2_DSS_DEBUGFS
	if (mpd->id_panel == MINNOW_PANEL_CM_BRIDGE_320X320)
		dss_debugfs_create_file("panel_regs", minnow_panel_dump_regs);
#endif

	return 0;

err_vc_id:
	omap_dsi_release_vc(dssdev, mpd->channel);
err_req_vc:
	if (bldev != NULL)
		backlight_device_unregister(bldev);
err_bl:
	destroy_workqueue(mpd->workqueue);
	return r;
}

static void __exit minnow_panel_remove(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bldev;

	dev_dbg(&dssdev->dev, "remove\n");

	sysfs_remove_group(&dssdev->dev.kobj, &minnow_panel_attr_group);
	omap_dsi_release_vc(dssdev, mpd->channel);

	bldev = mpd->bldev;
	if (bldev != NULL) {
		bldev->props.power = FB_BLANK_POWERDOWN;
		minnow_panel_bl_update_status(bldev);
		backlight_device_unregister(bldev);
	}

	minnow_panel_cancel_ulps_work(dssdev);
	minnow_panel_cancel_esd_work(dssdev);
	destroy_workqueue(mpd->workqueue);

	/* reset, to be sure that the panel is in a valid state */
	minnow_panel_hw_reset(dssdev);
}

static int minnow_panel_power_on(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	u8 id1, id2, id3;
	int r;

	r = omapdss_dsi_configure_pins(dssdev, &mpd->pin_config);
	if (r) {
		dev_err(&dssdev->dev, "failed to configure DSI pins\n");
		goto err0;
	};

	mpd->dsi_config.timings = &dssdev->panel.timings;
	r = omapdss_dsi_set_config(dssdev, &mpd->dsi_config);
	if (r) {
		dev_err(&dssdev->dev, "failed to configure DSI\n");
		goto err0;
	}

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err0;
	}

#ifdef	CONFIG_OMAP2_DSS_RESET
	minnow_panel_hw_reset(dssdev);
#else
	/* Need reset display at first time as it's already enable on boot */
	if (mpd->intro_printed)
		minnow_panel_hw_reset(dssdev);
#endif

	omapdss_dsi_vc_enable_hs(dssdev, mpd->channel, false);

	r = _minnow_panel_init(mpd);
	if (r)
		goto err;

	r = minnow_panel_get_id(mpd, &id1, &id2, &id3);
	if (r)
		goto err;

	r = _minnow_panel_enable_te(dssdev, mpd->te_enabled);
	if (r)
		goto err;

	r = minnow_panel_dcs_write_0(mpd, MIPI_DCS_SET_DISPLAY_ON);
	if (r)
		goto err;

	omapdss_dsi_vc_enable_hs(dssdev, mpd->channel, true);

	r = dsi_enable_video_output(dssdev, mpd->channel);
	if (r)
		goto err;

	mpd->enabled = 1;

	if (!mpd->intro_printed) {
		dev_info(&dssdev->dev, "panel revision %02x.%02x.%02x\n",
			id1, id2, id3);
		mpd->intro_printed = true;
	}

	return 0;
err:
	dev_err(&dssdev->dev, "error while enabling panel, issuing HW reset\n");

	minnow_panel_hw_reset(dssdev);

	omapdss_dsi_display_disable(dssdev, true, false);
err0:
	return r;
}

static void minnow_panel_power_off(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int r;

	dsi_disable_video_output(dssdev, mpd->channel);

	r = minnow_panel_dcs_write_0(mpd, MIPI_DCS_SET_DISPLAY_OFF);
	if (!r)
		r = minnow_panel_sleep_in(mpd);

	if (r) {
		dev_err(&dssdev->dev,
				"error disabling panel, issuing HW reset\n");
		minnow_panel_hw_reset(dssdev);
	}

	omapdss_dsi_display_disable(dssdev, true, false);

	mpd->enabled = 0;
}

static int minnow_panel_reset(struct omap_dss_device *dssdev)
{
	dev_err(&dssdev->dev, "performing LCD reset\n");

	minnow_panel_power_off(dssdev);
	minnow_panel_hw_reset(dssdev);
	return minnow_panel_power_on(dssdev);
}

static int minnow_panel_enable(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_dbg(&dssdev->dev, "enable\n");

	mutex_lock(&mpd->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}

	if (mpd->bridge_regulator) {
		if (regulator_enable(mpd->bridge_regulator)) {
			r = -ENODEV;
			goto err;
		}
	}
	if (mpd->panel_regulator) {
		if (regulator_enable(mpd->panel_regulator)) {
			r = -ENODEV;
			goto err;
		}
	}

	dsi_bus_lock(dssdev);

	r = minnow_panel_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r)
		goto err;

	minnow_panel_queue_esd_work(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&mpd->lock);

	return 0;
err:
	dev_dbg(&dssdev->dev, "enable failed\n");
	mutex_unlock(&mpd->lock);
	return r;
}

static void minnow_panel_disable(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	mutex_lock(&mpd->lock);

	minnow_panel_cancel_ulps_work(dssdev);
	minnow_panel_cancel_esd_work(dssdev);

	dsi_bus_lock(dssdev);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		int r;

		r = minnow_panel_wake_up(dssdev);
		if (!r)
			minnow_panel_power_off(dssdev);
	}

	dsi_bus_unlock(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	if (mpd->panel_regulator)
		regulator_disable(mpd->panel_regulator);

	if (mpd->bridge_regulator)
		regulator_disable(mpd->bridge_regulator);

	mutex_unlock(&mpd->lock);
}

static void minnow_panel_framedone_cb(int err, void *data)
{
	struct omap_dss_device *dssdev = data;
	dev_dbg(&dssdev->dev, "framedone, err %d\n", err);
	dsi_bus_unlock(dssdev);
}

static irqreturn_t minnow_panel_te_isr(int irq, void *data)
{
	struct omap_dss_device *dssdev = data;
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int old;
	int r;

	old = atomic_cmpxchg(&mpd->do_update, 1, 0);

	if (old) {
		cancel_delayed_work(&mpd->te_timeout_work);
		r = omap_dsi_update(dssdev, mpd->channel, minnow_panel_framedone_cb,
				dssdev);
		if (r)
			goto err;
	}

	return IRQ_HANDLED;
err:
	dev_err(&dssdev->dev, "start update failed\n");
	dsi_bus_unlock(dssdev);
	return IRQ_HANDLED;
}

static void minnow_panel_te_timeout_work_callback(struct work_struct *work)
{
	struct minnow_panel_data *mpd = container_of(work, struct minnow_panel_data,
					te_timeout_work.work);
	struct omap_dss_device *dssdev = mpd->dssdev;

	dev_err(&dssdev->dev, "TE not received for 250ms!\n");

	atomic_set(&mpd->do_update, 0);
	dsi_bus_unlock(dssdev);
}

static int minnow_panel_update(struct omap_dss_device *dssdev,
				    u16 x, u16 y, u16 w, u16 h)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int r;

	/* for video mode, do not need manual update */
	if (mpd->dsi_config.mode == OMAP_DSS_DSI_VIDEO_MODE)
		return 0;

	dev_dbg(&dssdev->dev, "update %d, %d, %d x %d\n", x, y, w, h);

	mutex_lock(&mpd->lock);
	dsi_bus_lock(dssdev);

	r = minnow_panel_wake_up(dssdev);
	if (r)
		goto err;

	if (!mpd->enabled) {
		r = 0;
		goto err;
	}

	/* XXX no need to send this every frame, but dsi break if not done */
	r = minnow_panel_set_update_window(mpd, 0, 0,
			dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);
	if (r)
		goto err;

	if (mpd->te_enabled && gpio_is_valid(mpd->ext_te_gpio)) {
		schedule_delayed_work(&mpd->te_timeout_work,
				msecs_to_jiffies(250));
		atomic_set(&mpd->do_update, 1);
	} else {
		r = omap_dsi_update(dssdev, mpd->channel, minnow_panel_framedone_cb,
				dssdev);
		if (r)
			goto err;
	}

	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&mpd->lock);
	return 0;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&mpd->lock);
	return r;
}

static int minnow_panel_sync(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "sync\n");

	mutex_lock(&mpd->lock);
	dsi_bus_lock(dssdev);
	dsi_bus_unlock(dssdev);
	mutex_unlock(&mpd->lock);

	dev_dbg(&dssdev->dev, "sync done\n");

	return 0;
}

static int _minnow_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int r;

	if (enable)
		r = minnow_panel_dcs_write_1(mpd, MIPI_DCS_SET_TEAR_ON, 0);
	else
		r = minnow_panel_dcs_write_0(mpd, MIPI_DCS_SET_TEAR_OFF);

	if (!gpio_is_valid(mpd->ext_te_gpio))
		omapdss_dsi_enable_te(dssdev, enable);

	return r;
}

static int minnow_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&mpd->lock);

	if (mpd->te_enabled == enable)
		goto end;

	dsi_bus_lock(dssdev);

	if (mpd->enabled) {
		r = minnow_panel_wake_up(dssdev);
		if (r)
			goto err;

		r = _minnow_panel_enable_te(dssdev, enable);
		if (r)
			goto err;
	}

	mpd->te_enabled = enable;

	dsi_bus_unlock(dssdev);
end:
	mutex_unlock(&mpd->lock);

	return 0;
err:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&mpd->lock);

	return r;
}

static int minnow_panel_get_te(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&mpd->lock);
	r = mpd->te_enabled;
	mutex_unlock(&mpd->lock);

	return r;
}

static int minnow_panel_run_test(struct omap_dss_device *dssdev, int test_num)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	u8 id1, id2, id3;
	int r;

	mutex_lock(&mpd->lock);

	if (!mpd->enabled) {
		r = -ENODEV;
		goto err1;
	}

	dsi_bus_lock(dssdev);

	r = minnow_panel_wake_up(dssdev);
	if (r)
		goto err2;

	r = minnow_panel_dcs_read_1(mpd, DCS_GET_ID1, &id1);
	if (r)
		goto err2;
	r = minnow_panel_dcs_read_1(mpd, DCS_GET_ID2, &id2);
	if (r)
		goto err2;
	r = minnow_panel_dcs_read_1(mpd, DCS_GET_ID3, &id3);
	if (r)
		goto err2;

	dsi_bus_unlock(dssdev);
	mutex_unlock(&mpd->lock);
	return 0;
err2:
	dsi_bus_unlock(dssdev);
err1:
	mutex_unlock(&mpd->lock);
	return r;
}

static int minnow_panel_memory_read(struct omap_dss_device *dssdev,
		void *buf, size_t size,
		u16 x, u16 y, u16 w, u16 h)
{
	int r;
	int plen;
	u32 buf_used = 0;
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	u8 dcs_cmd = MIPI_DCS_READ_MEMORY_START;

	plen = dsi_get_pixel_size(mpd->dssdev->panel.dsi_pix_fmt);
	plen = DIV_ROUND_UP(plen, 8);
	if (size < w * h * plen)
		return -ENOMEM;

	mutex_lock(&mpd->lock);

	if (!mpd->enabled) {
		r = -ENODEV;
		goto err1;
	}

	size = min(w * h * plen,
			dssdev->panel.timings.x_res *
			dssdev->panel.timings.y_res * plen);

	dsi_bus_lock(dssdev);

	r = minnow_panel_wake_up(dssdev);
	if (r)
		goto err2;

	minnow_panel_set_update_window(mpd, x, y, w, h);

	/* each read 8 pixel as SSD2848 has minimum read pixels */
	plen *= 8;
	r = dsi_vc_set_max_rx_packet_size(dssdev, mpd->channel, plen);
	if (r)
		goto err2;

	size = size / plen * plen;
	for (; buf_used < size; dcs_cmd = MIPI_DCS_READ_MEMORY_CONTINUE) {
		r = dsi_vc_dcs_read(dssdev, mpd->channel, dcs_cmd,
				buf + buf_used, plen);
		if (r) {
			dev_err(&dssdev->dev,
				"read failed at %u err=%d\n", buf_used, r);
			goto err3;
		}
		buf_used += plen;
		if (signal_pending(current)) {
			dev_err(&dssdev->dev, "signal pending, "
					"aborting memory read\n");
			r = -ERESTARTSYS;
			goto err3;
		}
	}
	r = buf_used;

err3:
	dsi_vc_set_max_rx_packet_size(dssdev, mpd->channel, 1);
err2:
	dsi_bus_unlock(dssdev);
err1:
	mutex_unlock(&mpd->lock);
	return r;
}

static void minnow_panel_ulps_work(struct work_struct *work)
{
	struct minnow_panel_data *mpd = container_of(work, struct minnow_panel_data,
			ulps_work.work);
	struct omap_dss_device *dssdev = mpd->dssdev;

	mutex_lock(&mpd->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE || !mpd->enabled) {
		mutex_unlock(&mpd->lock);
		return;
	}

	dsi_bus_lock(dssdev);

	minnow_panel_enter_ulps(dssdev);

	dsi_bus_unlock(dssdev);
	mutex_unlock(&mpd->lock);
}

static void minnow_panel_esd_work(struct work_struct *work)
{
	struct minnow_panel_data *mpd = container_of(work, struct minnow_panel_data,
			esd_work.work);
	struct omap_dss_device *dssdev = mpd->dssdev;
	u8 state1, state2;
	int r;

	mutex_lock(&mpd->lock);

	if (!mpd->enabled) {
		mutex_unlock(&mpd->lock);
		return;
	}

	dsi_bus_lock(dssdev);

	r = minnow_panel_wake_up(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to exit ULPS\n");
		goto err;
	}

	r = minnow_panel_dcs_read_1(mpd, MIPI_DCS_GET_DIAGNOSTIC_RESULT, &state1);
	if (r) {
		dev_err(&dssdev->dev, "failed to read minnow-panel status\n");
		goto err;
	}

	/* Run self diagnostics */
	r = minnow_panel_sleep_out(mpd);
	if (r) {
		dev_err(&dssdev->dev, "failed to run minnow-panel self-diagnostics\n");
		goto err;
	}

	r = minnow_panel_dcs_read_1(mpd, MIPI_DCS_GET_DIAGNOSTIC_RESULT, &state2);
	if (r) {
		dev_err(&dssdev->dev, "failed to read minnow-panel status\n");
		goto err;
	}

	/* Each sleep out command will trigger a self diagnostic and flip
	 * Bit6 if the test passes.
	 */
	if (!((state1 ^ state2) & (1 << 6))) {
		dev_err(&dssdev->dev, "LCD self diagnostics failed\n");
		goto err;
	}
	/* Self-diagnostics result is also shown on TE GPIO line. We need
	 * to re-enable TE after self diagnostics */
	if (mpd->te_enabled && gpio_is_valid(mpd->ext_te_gpio)) {
		r = minnow_panel_dcs_write_1(mpd, MIPI_DCS_SET_TEAR_ON, 0);
		if (r)
			goto err;
	}

	dsi_bus_unlock(dssdev);

	minnow_panel_queue_esd_work(dssdev);

	mutex_unlock(&mpd->lock);

	return;
err:
	dev_err(&dssdev->dev, "performing LCD reset\n");

	minnow_panel_reset(dssdev);

	dsi_bus_unlock(dssdev);

	minnow_panel_queue_esd_work(dssdev);

	mutex_unlock(&mpd->lock);
}

static struct omap_dss_driver minnow_panel_driver = {
	.probe		= minnow_panel_probe,
	.remove		= __exit_p(minnow_panel_remove),

	.enable		= minnow_panel_enable,
	.disable	= minnow_panel_disable,

	.update		= minnow_panel_update,
	.sync		= minnow_panel_sync,

	.get_resolution	= minnow_panel_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.enable_te	= minnow_panel_enable_te,
	.get_te		= minnow_panel_get_te,

	.run_test	= minnow_panel_run_test,
	.memory_read	= minnow_panel_memory_read,

	.driver         = {
		.name   = "minnow-panel",
		.owner  = THIS_MODULE,
	},
};

static int __init minnow_panel_init(void)
{
	omap_dss_register_driver(&minnow_panel_driver);

	return 0;
}

static void __exit minnow_panel_exit(void)
{
	omap_dss_unregister_driver(&minnow_panel_driver);
}

module_init(minnow_panel_init);
module_exit(minnow_panel_exit);

MODULE_DESCRIPTION("Minnow Panel DSI Driver");
MODULE_LICENSE("GPL");
