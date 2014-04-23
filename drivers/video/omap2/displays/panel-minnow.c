/*
 * Minnow DSI command mode panel
 *
 * Copyright (C) 2013-2014 Motorola Mobility LLC.
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
#include <linux/clk.h>

#include <video/omapdss.h>
#include <video/omap-panel-data.h>
#include <video/mipi_display.h>

#include "../dss/dss.h"

/* panel parameter to indicate if it needs skip first time initialize */
static bool	def_skip_first_init;
module_param_named(skip_first_init, def_skip_first_init, bool, 0);

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

enum minnow_panel_component {
	MINNOW_PANEL,
	MINNOW_BRIDGE,
	MINNOW_COMPONENT_MAX
};

enum minnow_panel_id {
	MINNOW_PANEL_CM_220X176,
	MINNOW_PANEL_CM_220X220,
	MINNOW_PANEL_CM_BRIDGE_320X320,
	MINNOW_PANEL_MAX
};

/* Panel initialize command type description:
 * DCS_WRITE_SYNC, DCS_WRITE:
 *   standard DCS type command with/without BTA sync
 * GENERIC_WRITE_SYNC, GENERIC_WRITE:
 *   standard Generic type command with/without BTA sync
 * BTA_SYNC:
 *   standard BTA sync command
 * WAIT_MS:
 *   sleep for given milliseconds
 * CHECK_MS:
 *   same as WAIT_MS, but it also use this time to read back register to
 *   verify it.
 * SSD2848_CMD:
 *   special command for SSD2848 bridge register, it has fixed 6 bytes format,
 *   the fist 2 bytes is the register address, the last is 32 bits register
 * OTM3201_CMD:
 *   special command for OTM3201 register, it has flexible register length,
 *   the fist byte is the register address
 */
enum minnow_cmd_type {
	DCS_WRITE_SYNC,
	GENERIC_WRITE_SYNC,
	DCS_WRITE,
	GENERIC_WRITE,
	BTA_SYNC,
	WAIT_MS,
	CHECK_MS,
	SSD2848_CMD,
	OTM3201_CMD,
	CMD_TYPE_MAX
};

/* Panel initialize command buffer description:
 * it uses compact buffer to store all initialize commands, the first
 * byte of each command is the command length in byte
 *
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

#define	BRIDGE_WIDTH		320
#define	BRIDGE_HEIGHT		320
#define	PANEL_WIDTH		BRIDGE_WIDTH
#define	PANEL_HEIGHT		290
#define	UNUSED_LINES		(BRIDGE_HEIGHT-PANEL_HEIGHT)

#define	SWITCH_TO_PANEL		2, SSD2848_CMD, 0xFF, 0x01
#define	SWITCH_TO_BRIDGE	2, SSD2848_CMD, 0xFF, 0x00
static u8 panel_init_ssd2848_320x320[] = {
/*n, type, data_0, data_1 ... data_n-1*/
SWITCH_TO_BRIDGE,
6, SSD2848_CMD, 0x00, 0x08, 0x01, 0xF4, 0x04, 0x29,
6, SSD2848_CMD, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x0B,
6, SSD2848_CMD, 0x00, 0x14, 0x0C, 0x07, 0x80, 0x0F,
1, DCS_WRITE_SYNC, MIPI_DCS_EXIT_SLEEP_MODE,
1, WAIT_MS, 1,
6, SSD2848_CMD, 0x10, 0x08, 0x01, 0x20, 0x01, 0x45,
6, SSD2848_CMD, 0x20, 0x0C, 0x00, 0x00, 0x00, 0x02,
6, SSD2848_CMD, 0x20, 0x10, 0x00, 0x1B, 0x00, 0x0C,
6, SSD2848_CMD, 0x20, 0x14, 0x01, 0x88, 0x00, 0x28,
6, SSD2848_CMD, 0x20, 0x18, 0x01, 0x4E, 0x00, 0x0A,
6, SSD2848_CMD, 0x20, 0x20, 0x01, 0x40, 0x01, 0x40,
6, SSD2848_CMD, 0x20, 0x24, 0x01, 0x40, 0x01, 0x40,
6, SSD2848_CMD, 0x20, 0x30, 0x00, 0x00, 0x00, 0x15,
6, SSD2848_CMD, 0x20, 0x34, 0x00, 0x00, 0x00, 0x00,
6, SSD2848_CMD, 0x20, 0x38, 0x01, 0x3F, 0x01, 0x3F,
6, SSD2848_CMD, 0x20, 0x3C, 0x01, 0x40, 0x01, 0x40,
6, SSD2848_CMD, 0x20, 0xA0, 0x00, 0x00, 0x05, 0x00,
5, DCS_WRITE_SYNC, 0x2A, 0x00, 0x00, 0x01, 0x3F,
5, DCS_WRITE_SYNC, 0x2B, 0x00, 0x00, 0x01, 0x3F,
6, SSD2848_CMD, 0x60, 0x08, 0x00, 0x02, 0x00, 0x0A,
6, SSD2848_CMD, 0x60, 0x0C, 0x0A, 0x2A, 0x02, 0x0A,
6, SSD2848_CMD, 0x60, 0x10, 0x01, 0x40, 0x02, 0x14,
6, SSD2848_CMD, 0x60, 0x14, 0x01, 0x00, 0x01, 0x00,
6, SSD2848_CMD, 0x60, 0x40, 0x13, 0x01, 0x0A, 0x01,
6, SSD2848_CMD, 0x60, 0x84, 0x00, 0x00, 0x01, 0x40,
SWITCH_TO_PANEL,
2, DCS_WRITE_SYNC, 0x36, 0x10,
3, OTM3201_CMD, 0xF0, 0x54, 0x47,
2, OTM3201_CMD, 0xA0, 0x00,
4, OTM3201_CMD, 0xBD, 0x00, 0x11, 0x31,
2, OTM3201_CMD, 0xE9, 0x46,
5, OTM3201_CMD, 0xBA, 0x06, 0x15, 0x2B, 0x01,
6, OTM3201_CMD, 0xB3, 0x02, 0x0A, 0x14, 0x2A, 0x2A,
5, OTM3201_CMD, 0xB5, 0x78, 0x78, 0x76, 0xF6,
18, OTM3201_CMD, 0xC0, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
18, OTM3201_CMD, 0xC1, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
18, OTM3201_CMD, 0xC2, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
18, OTM3201_CMD, 0xC3, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
18, OTM3201_CMD, 0xC4, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
18, OTM3201_CMD, 0xC5, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
1, DCS_WRITE_SYNC, MIPI_DCS_EXIT_SLEEP_MODE,
1, CHECK_MS, 120,
/* Needed switch back as it changed by function to clear bottom lines */
SWITCH_TO_PANEL,
1, DCS_WRITE_SYNC, MIPI_DCS_SET_DISPLAY_ON,
SWITCH_TO_BRIDGE,
0
};

static u8 panel_off_ssd2848_320x320[] = {
/*n, type, data_0, data_1 ... data_n-1*/
SWITCH_TO_PANEL,
1, DCS_WRITE_SYNC, MIPI_DCS_SET_DISPLAY_OFF,
1, DCS_WRITE, MIPI_DCS_ENTER_SLEEP_MODE,
SWITCH_TO_BRIDGE,
1, DCS_WRITE_SYNC, MIPI_DCS_SET_DISPLAY_OFF,
1, WAIT_MS, 50,
1, DCS_WRITE, MIPI_DCS_ENTER_SLEEP_MODE,
1, WAIT_MS, 20,
6, GENERIC_WRITE, 0x10, 0x28, 0x00, 0x00, 0x00, 0x01, /*power cut enabled*/
0
};

static u8 panel_off_common[] = {
/*n, type, data_0, data_1 ... data_n-1*/
1, DCS_WRITE_SYNC, MIPI_DCS_SET_DISPLAY_OFF,
1, DCS_WRITE, MIPI_DCS_ENTER_SLEEP_MODE,
1, WAIT_MS, 20,
0
};

enum minnow_panel_dummy_type {
	DUMMY_NONE = 0,
	DUMMY_ENABLED,
	DUMMY_DISABLED
};

struct minnow_panel_clk_range {
	int	min;
	int	max;
};

enum minnow_panel_active_level {
	ACTIVE_LOW = 0,
	ACTIVE_HIGH,
	ACTIVE_MAX
};
struct minnow_panel_hw_reset {
	enum minnow_panel_active_level	active;
	int	reset_ms;
	int	wait_ms;
};

struct minnow_panel_cmd_buf {
	int	count;
	u8	*cmdbuf;
};

struct minnow_panel_attr {
	int	mode;
	int	xres;
	int	yres;
	int	pixel_clock;
	int	pixel_format;
	int	xoffset;
	int	yoffset;
	struct minnow_panel_cmd_buf power_on;
	struct minnow_panel_cmd_buf power_off;
	struct minnow_panel_clk_range hs;
	struct minnow_panel_clk_range lp;
	struct minnow_panel_hw_reset panel_reset;
	struct minnow_panel_hw_reset bridge_reset;
};

#define INIT_CMD_BUF(type, buf)		.power_##type = {\
					.count = sizeof(buf), .cmdbuf = (buf) }
static struct minnow_panel_attr panel_attr_table[MINNOW_PANEL_MAX] = {
	[MINNOW_PANEL_CM_220X176] = {
		.mode = OMAP_DSS_DSI_CMD_MODE,
		.xres = 220,
		.yres = 176,
		.pixel_clock = 4608,
		.pixel_format = OMAP_DSS_DSI_FMT_RGB666,
		.xoffset = 0x32,
		.yoffset = 0,
		INIT_CMD_BUF(on, panel_init_220x176),
		INIT_CMD_BUF(off, panel_off_common),
		.hs = { 100000000, 150000000 },
		.lp = { 7000000, 9000000 },
		.panel_reset = { ACTIVE_LOW, 1, 5 },
		.bridge_reset = { ACTIVE_LOW, 0, 0 },
	},
	[MINNOW_PANEL_CM_220X220] = {
		.mode = OMAP_DSS_DSI_CMD_MODE,
		.xres = 220,
		.yres = 220,
		.pixel_clock = 4608,
		.pixel_format = OMAP_DSS_DSI_FMT_RGB666,
		.xoffset = 0x32,
		.yoffset = 0x4,
		INIT_CMD_BUF(on, panel_init_220x220),
		INIT_CMD_BUF(off, panel_off_common),
		.hs = { 100000000, 150000000 },
		.lp = { 7000000, 9000000 },
		.panel_reset = { ACTIVE_LOW, 1, 5 },
		.bridge_reset = { ACTIVE_LOW, 0, 0 },
	},
	[MINNOW_PANEL_CM_BRIDGE_320X320] = {
		.mode = OMAP_DSS_DSI_CMD_MODE,
		.xres = PANEL_WIDTH,
		.yres = PANEL_HEIGHT,
		.pixel_clock = DIV_ROUND_UP(PANEL_WIDTH *
					    PANEL_HEIGHT * 60, 1000),
		.pixel_format = OMAP_DSS_DSI_FMT_RGB888,
		.xoffset = 0,
		.yoffset = 0,
		INIT_CMD_BUF(on, panel_init_ssd2848_320x320),
		INIT_CMD_BUF(off, panel_off_ssd2848_320x320),
		.hs = { 100000000, 150000000 },
		.lp = { 7000000, 9000000 },
		.panel_reset = { ACTIVE_LOW, 5, 10 },
		.bridge_reset = { ACTIVE_LOW, 20, 10 }
	},
};

static irqreturn_t minnow_panel_te_isr(int irq, void *data);
static void minnow_panel_te_timeout_work_callback(struct work_struct *work);
static int _minnow_panel_enable_te(struct omap_dss_device *dssdev, bool enable);

static int minnow_panel_reset(struct omap_dss_device *dssdev);
static int minnow_panel_update(struct omap_dss_device *dssdev,
			       u16 x, u16 y, u16 w, u16 h);


struct minnow_panel_data {
	struct mutex lock; /* mutex */

	struct backlight_device *bldev;

	unsigned long	hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long	hw_guard_wait;	/* max guard time in jiffies */

	struct omap_dss_device *dssdev;

	/* panel HW configuration from DT or platform data */
	int reset_gpio[MINNOW_COMPONENT_MAX];
	int ext_te_gpio;
	int vio_en_gpio;
	struct minnow_panel_hw_reset hw_reset[MINNOW_COMPONENT_MAX];
	struct regulator *regulators[MINNOW_COMPONENT_MAX];
	struct clk *clk_in;
	bool clk_in_en;

	bool use_dsi_backlight;

	struct omap_dsi_pin_config pin_config;
	struct omap_dss_dsi_config dsi_config;
	struct minnow_panel_cmd_buf power_on;
	struct minnow_panel_cmd_buf power_off;
	u8 *last_init_data;

	int id_panel;
	int x_offset;
	int y_offset;
	int reset_ms;
	int release_ms;

	/* runtime variables */
	bool enabled;
	bool interactive;
	enum minnow_panel_dummy_type dummy_panel;

	bool te_enabled;

	atomic_t do_update;
	int channel;

	struct delayed_work te_timeout_work;

	unsigned cabc_mode;

	bool first_enable;

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

static int set_bridge_retrans(struct minnow_panel_data *mpd, int enable)
{
	u8 data[2] = {0xFF, enable ? 0x01 : 0x00};
	return dsi_vc_generic_write(mpd->dssdev,
					mpd->channel, data, sizeof(data));
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

	r = dsi_vc_dcs_write_nosync(mpd->dssdev, mpd->channel,
				    buf, sizeof(buf));
	if (r)
		return r;

	buf[0] = MIPI_DCS_SET_PAGE_ADDRESS;
	buf[1] = (y1 >> 8) & 0xff;
	buf[2] = (y1 >> 0) & 0xff;
	buf[3] = (y2 >> 8) & 0xff;
	buf[4] = (y2 >> 0) & 0xff;

	r = dsi_vc_dcs_write_nosync(mpd->dssdev, mpd->channel,
				    buf, sizeof(buf));
	if (r)
		return r;

	dsi_vc_send_bta_sync(mpd->dssdev, mpd->channel);

	return r;
}

/* since SSD2848 frame buffer is 320 x 320, but the actual panel is 320 x 290
 * it needs clear the unused bottom 30 lines for saving power
 */
static int minnow_panel_clear_bottom_line(struct minnow_panel_data *mpd)
{
	int r = 0;
#if	UNUSED_LINES
	int plen, total;
	u8 buf[124]; /* maximum packet size */

	memset(buf, 0, sizeof(buf));
	buf[0] = MIPI_DCS_WRITE_MEMORY_START;

	r = set_bridge_retrans(mpd, false);
	if (r)
		return r;

	omapdss_dsi_vc_enable_hs(mpd->dssdev, mpd->channel, true);

	plen = dsi_get_pixel_size(mpd->dssdev->panel.dsi_pix_fmt);
	plen = DIV_ROUND_UP(plen, 8);
	total = PANEL_WIDTH * UNUSED_LINES * plen;
	plen = (sizeof(buf) - 1) / plen * plen;
	r = minnow_panel_set_update_window(mpd, 0, PANEL_HEIGHT,
					   PANEL_WIDTH, UNUSED_LINES);
	for (; total && !r; buf[0] = MIPI_DCS_WRITE_MEMORY_CONTINUE) {
		if (plen > total)
			plen = total;
		total -= plen;
		r = dsi_vc_dcs_write(mpd->dssdev, mpd->channel, buf, plen+1);
	}

	if (!r)
		r = minnow_panel_set_update_window(mpd, 0, 0,
					mpd->dssdev->panel.timings.x_res,
					mpd->dssdev->panel.timings.y_res);

	omapdss_dsi_vc_enable_hs(mpd->dssdev, mpd->channel, false);
#endif
	return r;
}

static int minnow_panel_process_cmd(struct minnow_panel_data *mpd,
				    u8 cmd, u8 *data, int len)
{
	int r = -EINVAL;

	switch (cmd) {
	case DCS_WRITE_SYNC:
	case OTM3201_CMD:
		r = dsi_vc_dcs_write(mpd->dssdev,
				mpd->channel, data, len);
		break;
	case GENERIC_WRITE_SYNC:
	case SSD2848_CMD:
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
	case CHECK_MS:
		msleep((len == 1 ? (u32)(*data) : *(u16 *)data));
		r = 0;
		break;
	}

	return r;
}

static int minnow_panel_verify_ssd2848(struct minnow_panel_data *mpd, u8 *data)
{
	u8 reg[4];
	int r = -EINVAL;
	if (data[0] != 6)
		return r;

	r = dsi_vc_set_max_rx_packet_size(mpd->dssdev, mpd->channel, 4);
	if (r)
		return r;

	r = dsi_vc_generic_read_2(mpd->dssdev, mpd->channel,
				  data[2], data[3], reg, 4);
	if (!r && (*(u32 *)(data+4) != *(u32 *)reg)) {
		dev_err(&mpd->dssdev->dev, "Failed verify ssd2848 reg"
			"[%02x%02x] %02x %02x %02x %02x\n", data[2], data[3],
			reg[0], reg[1], reg[2], reg[3]);
		r = -EINVAL;
	}

	return r;
}

static int minnow_panel_verify_otm3201(struct minnow_panel_data *mpd, u8 *data)
{
	static u8 reg_wo[] = { 0xA0, 0xF0, 0xB5 }; /* write only registers */
	int i, len, r = 0;
	u8 reg[18];

	for (i = r = 0; i < (sizeof(reg_wo)/sizeof(reg_wo[0])); i++) {
		if (data[2] == reg_wo[i]) {
			if (data[2] == 0xA0) {
				/* set OTM3201 read mode */
				reg[0] = 0xA0;
				reg[1] = 0x80;
				r = dsi_vc_dcs_write(mpd->dssdev,
						     mpd->channel, reg, 2);
			}
			/* do not read back for write only register */
			return r;
		}
	}

	len = data[0] - 1;
	if ((u32)(len) > sizeof(reg))
		return -EINVAL;

	r = dsi_vc_set_max_rx_packet_size(mpd->dssdev, mpd->channel, len);
	if (r)
		return r;

	r = dsi_vc_dcs_read(mpd->dssdev, mpd->channel, data[2], reg, len);
	if (r)
		return r;

	for (i = 0; i < len; i++) {
		if (reg[i] != data[3+i])
			break;
	}
	if (i < len) {
		char errstr[88], *pstr;
		sprintf(errstr, "Failed verify otm3201 reg[%02x]", data[2]);
		pstr = errstr + strlen(errstr);
		for (i = 0; i < len; i++, pstr += 3)
			sprintf(pstr, " %02x", reg[i]);
		strcpy(pstr, "\n");
		dev_err(&mpd->dssdev->dev, errstr);
		r = -EINVAL;
	}

	return r;
}

static int minnow_panel_dummy_check(struct minnow_panel_data *mpd)
{
	int r = set_bridge_retrans(mpd, true);
	if (!r) {
		u8 id1;
		r = minnow_panel_dcs_read_1(mpd, DCS_GET_ID1, &id1);
		if (r) {
			dev_err(&mpd->dssdev->dev, "Failed get panel id, "
				"enable dummy panel !!!");
			mpd->dummy_panel = DUMMY_ENABLED;
		} else {
			mpd->dummy_panel = DUMMY_DISABLED;
			r = set_bridge_retrans(mpd, false);
		}
	}
	return r;
}

static int minnow_panel_verify_cmdbuf(struct minnow_panel_data *mpd,
				      struct minnow_panel_cmd_buf *cmd_buf)
{
	u8 *data;
	int i, r = 0;
	unsigned long start_jiffies = jiffies;
	unsigned int delay_ms;

	/* it check dummy panel only at first time power on */
	if (mpd->dummy_panel == DUMMY_NONE) {
		r = minnow_panel_dummy_check(mpd);
		if (r)
			return r;
	}

	data = cmd_buf->cmdbuf;
	for (i = 0; *data && !r; i++, data += *data+2) {
		switch (data[1]) {
		case SSD2848_CMD:
			if (data[0] == 2) {
				/* set bridge retransmission mode */
				r = dsi_vc_generic_write(mpd->dssdev,
							 mpd->channel,
							 data+2, 2);
				continue;
			}
			r = minnow_panel_verify_ssd2848(mpd, data);
			continue;
		case OTM3201_CMD:
			if (mpd->dummy_panel == DUMMY_DISABLED)
				r = minnow_panel_verify_otm3201(mpd, data);
			continue;
		}
		if (data[1] == CHECK_MS) {
			delay_ms = (unsigned int)(data[0] == 1
					? (u16)(data[2]) : *(u16 *)(data+2));
			break;
		}
	}
	dsi_vc_set_max_rx_packet_size(mpd->dssdev, mpd->channel, 1);

	if (!r)
		r = minnow_panel_clear_bottom_line(mpd);
	if (!r) {
		unsigned int elapse_ms;
		elapse_ms = jiffies_to_msecs(jiffies - start_jiffies);
		if (elapse_ms < delay_ms)
			msleep(delay_ms - elapse_ms);
	}

	return r;
}

static int minnow_panel_check_cmdbuf(struct minnow_panel_data *mpd,
				     u8 *data, int count)
{
	int i, r = 0;

	for (i = count; *data && (i > 0); ) {
		if (data[1] >= CMD_TYPE_MAX)
			break;
		i -= ((u32)*data + 2);
		data += (*data + 2);
	}

	/* command data shall end with 0 */
	if (*data || (i != 1)) {
		dev_err(&mpd->dssdev->dev, "Invalid command data(0x%02x) "
			"found at offset %d", *data, count - i);
		r = -EINVAL;
	}

	return r;
}

static int minnow_panel_process_cmdbuf(struct minnow_panel_data *mpd,
				       struct minnow_panel_cmd_buf *cmd_buf)
{
	u8 *data;
	int i, r;

	/* be safe to check command data every time before sent to driver */
	r = minnow_panel_check_cmdbuf(mpd, cmd_buf->cmdbuf, cmd_buf->count);
	if (r)
		return r;

	for (i = 0, data = cmd_buf->cmdbuf; *data; i++, data += *data+2) {
		if (data[1] == CHECK_MS)
			r = minnow_panel_verify_cmdbuf(mpd, cmd_buf);
		else
			r = minnow_panel_process_cmd(mpd, data[1],
						     data+2, data[0]);
		if (r) {
			dev_err(&mpd->dssdev->dev, "Failed process initialize"
				" command[%d] len=%d type=%d ret=%d\n",
				i, data[0], data[1], r);
			break;
		}
	}

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
	dev_info(&dssdev->dev, "entered ULPS mode\n");

	return 0;

err:
	dev_err(&dssdev->dev, "enter ULPS failed\n");
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

	dev_info(&dssdev->dev, "exited ULPS mode\n");

	return 0;

err2:
	dev_err(&dssdev->dev, "failed to exit ULPS\n");

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

#ifdef	DEBUG
static ssize_t minnow_panel_store_init_data(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	u8 *data;
	int r;

	r = minnow_panel_check_cmdbuf(mpd, (u8 *)buf, count);
	if (r)
		return r;

	data = devm_kzalloc(&dssdev->dev, count, GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	memcpy(data, buf, count);

	mutex_lock(&mpd->lock);
	mpd->power_on.count = count;
	mpd->power_on.cmdbuf = data;
	if (mpd->last_init_data)
		devm_kfree(&dssdev->dev, mpd->last_init_data);
	mpd->last_init_data = data;
	mutex_unlock(&mpd->lock);

	return count;
}

static ssize_t minnow_panel_show_init_data(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int i, j;
	u8 *data;

	mutex_lock(&mpd->lock);
	data = mpd->power_on.cmdbuf;
	mutex_unlock(&mpd->lock);

	for (i = 0; i < PAGE_SIZE && *data; ) {
		snprintf(buf+i, PAGE_SIZE-i, "%02d %02d:", data[0], data[1]);
		i += 6;
		for (j = 0; j < *data && i < PAGE_SIZE; j++) {
			snprintf(buf+i, PAGE_SIZE-i, " %02X", data[2+j]);
			i += 3;
		}
		snprintf(buf+i, PAGE_SIZE-i, "\n");
		i++;
		data += *data + 2;
	}

	return i < PAGE_SIZE ? i : PAGE_SIZE;
}
#endif

static ssize_t minnow_panel_show_interactivemode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	unsigned t;

	mutex_lock(&mpd->lock);
	t = mpd->interactive;
	mutex_unlock(&mpd->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

static ssize_t minnow_panel_store_interactivemode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (!r) {
		bool enable = !!t;
		/* Temporary force DSI goto ULPS mode when interactive mode
		 * turns OFF, it will add support to switch bridge/panel to
		 * low refresh mode later
		 */
		mutex_lock(&mpd->lock);
		if (mpd->enabled && (mpd->interactive != enable)) {
			mpd->interactive = enable;
			dsi_bus_lock(dssdev);
			if (enable)
				r = minnow_panel_wake_up(dssdev);
			else
				r = minnow_panel_enter_ulps(dssdev);
			dsi_bus_unlock(dssdev);
		}
		mutex_unlock(&mpd->lock);
		dev_info(&dssdev->dev, "%s interactive mode%s\n",
			 enable ? "enable" : "disable", r ? " failed" : "");
	}

	return r ? r : count;
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
#ifdef	DEBUG
static DEVICE_ATTR(init_data, S_IRUGO | S_IWUSR,
		minnow_panel_show_init_data, minnow_panel_store_init_data);
#endif
static DEVICE_ATTR(interactivemode, S_IRUGO | S_IWUSR,
		minnow_panel_show_interactivemode, minnow_panel_store_interactivemode);

static struct attribute *minnow_panel_attrs[] = {
	&dev_attr_num_dsi_errors.attr,
	&dev_attr_hw_revision.attr,
	&dev_attr_cabc_mode.attr,
	&dev_attr_cabc_available_modes.attr,
	&dev_attr_esd_interval.attr,
	&dev_attr_ulps.attr,
	&dev_attr_ulps_timeout.attr,
#ifdef	DEBUG
	&dev_attr_init_data.attr,
#endif
	&dev_attr_interactivemode.attr,
	NULL,
};

static struct attribute_group minnow_panel_attr_group = {
	.attrs = minnow_panel_attrs,
};

static void _minnow_panel_hw_active_reset(struct minnow_panel_data *mpd)
{
	int i;
	if (mpd->reset_ms < 0)
		return;

	/* reset the device */
	for (i = 0; i < MINNOW_COMPONENT_MAX; i++) {
		if (!gpio_is_valid(mpd->reset_gpio[i]))
			continue;
		gpio_set_value(mpd->reset_gpio[i],
			       mpd->hw_reset[i].active ? 1 : 0);
	}

	/* wait device reset */
	msleep(mpd->reset_ms);
}

static void _minnow_panel_hw_reset(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int i;

	_minnow_panel_hw_active_reset(mpd);

	/* assert reset */
	for (i = 0; i < MINNOW_COMPONENT_MAX; i++) {
		if (!gpio_is_valid(mpd->reset_gpio[i]))
			continue;
		gpio_set_value(mpd->reset_gpio[i],
			       mpd->hw_reset[i].active ? 0 : 1);
	}

	/* wait after releasing reset */
	if (mpd->release_ms > 0)
		msleep(mpd->release_ms);
}

static int minnow_panel_set_regulators(struct minnow_panel_data *mpd,
	int (*func)(struct regulator *regulator))
{
	int i;

	for (i = 0; i < MINNOW_COMPONENT_MAX; i++) {
		if (!mpd->regulators[i])
			continue;
		if (func(mpd->regulators[i]))
			return -ENODEV;
	}

	return 0;
}

static void minnow_panel_enable_vio(struct minnow_panel_data *mpd, bool enable)
{
	if (gpio_is_valid(mpd->vio_en_gpio))
		gpio_set_value(mpd->vio_en_gpio, enable ? 0 : 1);
}

static int minnow_panel_enable_clkin(struct minnow_panel_data *mpd,
				     bool enable)
{
	int r = 0;
	if ((mpd->clk_in_en != enable) && (mpd->clk_in != NULL)) {
		if (enable)
			r = clk_prepare_enable(mpd->clk_in);
		else
			clk_disable_unprepare(mpd->clk_in);
		if (!r)
			mpd->clk_in_en = enable;
	}
	return r;
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
	char *clkin;

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
	mpd->power_on = panel_attr->power_on;
	mpd->power_off = panel_attr->power_off;
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

	mpd->hw_reset[MINNOW_PANEL] = panel_attr->panel_reset;
	mpd->hw_reset[MINNOW_BRIDGE] = panel_attr->bridge_reset;
	mpd->reset_gpio[MINNOW_PANEL] =
		of_get_named_gpio(dt_node, "gpio_panel_reset", 0);
	DTINFO("gpio_panel_reset = %d\n", mpd->reset_gpio[MINNOW_PANEL]);
	mpd->reset_gpio[MINNOW_BRIDGE] =
		of_get_named_gpio(dt_node, "gpio_bridge_reset", 0);
	DTINFO("gpio_bridge_reset = %d\n",
	       mpd->reset_gpio[MINNOW_BRIDGE]);
	mpd->ext_te_gpio = of_get_named_gpio(dt_node, "gpio_te", 0);
	DTINFO("gpio_te = %d\n", mpd->ext_te_gpio);
	mpd->vio_en_gpio = of_get_named_gpio(dt_node, "gpio_vio_en", 0);
	DTINFO("gpio_vio_en = %d\n", mpd->vio_en_gpio);
	clkin = (char *)of_get_property(dt_node, "clk_in", NULL);
	if (clkin) {
		mpd->clk_in = clk_get(NULL, clkin);
		if (IS_ERR(mpd->clk_in)) {
			int r = PTR_ERR(mpd->clk_in);
			dev_err(&mpd->dssdev->dev,
				"Failed get external clock %s!\n", clkin);
			mpd->clk_in = NULL;
			return r;
		}
	}
	mpd->esd_interval = 0;
	if (!of_property_read_u32(dt_node, "esd_interval", &value)) {
		mpd->esd_interval = value;
		DTINFO("esd_interval = %d\n", mpd->esd_interval);
	}
	/* automatically go to ULPS mode for none-update within 250ms */
	mpd->ulps_timeout = 250;
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
	mpd->first_enable = true;

	r = minnow_panel_dt_init(mpd);
	if (r)
		return r;

	mutex_init(&mpd->lock);

	atomic_set(&mpd->do_update, 0);

	dev_info(&dssdev->dev, "skip first time initialization is %s\n",
		 def_skip_first_init ? "enabled" : "disabled");

	if (gpio_is_valid(mpd->vio_en_gpio)) {
		r = devm_gpio_request_one(&dssdev->dev, mpd->vio_en_gpio,
					  def_skip_first_init
					  ? GPIOF_OUT_INIT_LOW
					  : GPIOF_OUT_INIT_HIGH,
					  "minnow-panel vio_en");
		if (r) {
			dev_err(&dssdev->dev,
				"failed to request panel vio_en gpio\n");
			return r;
		}
	}

	mpd->reset_ms = -1;
	mpd->release_ms = -1;
	for (i = 0; i < MINNOW_COMPONENT_MAX; i++) {
		static const char * const name[MINNOW_COMPONENT_MAX] = {
			"minnow-panel reset",
			"minnow-bridge reset"
		};
		if (!gpio_is_valid(mpd->reset_gpio[i]))
			continue;
		r = devm_gpio_request_one(&dssdev->dev, mpd->reset_gpio[i],
					  def_skip_first_init
					  ? (mpd->hw_reset[i].active
					     ? GPIOF_OUT_INIT_LOW
					     : GPIOF_OUT_INIT_HIGH)
					  : (mpd->hw_reset[i].active
					     ? GPIOF_OUT_INIT_HIGH
					     : GPIOF_OUT_INIT_LOW),
					  name[i]);
		if (r) {
			dev_err(&dssdev->dev,
				"failed to request %s gpio\n", name[i]);
			return r;
		}
		if (mpd->reset_ms < mpd->hw_reset[i].reset_ms)
			mpd->reset_ms = mpd->hw_reset[i].reset_ms;
		if (mpd->release_ms < mpd->hw_reset[i].wait_ms)
			mpd->release_ms = mpd->hw_reset[i].wait_ms;
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

	for (i = 0; i < MINNOW_COMPONENT_MAX; i++) {
		static const char * const name[MINNOW_COMPONENT_MAX] = {
			"panel",
			"bridge"
		};
		struct regulator *rt;
		rt = devm_regulator_get(&dssdev->dev, name[i]);
		if (IS_ERR(rt)) {
			rt = NULL;
			dev_info(&dssdev->dev, "Could not get %s regulator\n",
				 name[i]);
		}
		mpd->regulators[i] = rt;
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
	_minnow_panel_hw_reset(dssdev);
}

#define	DCS_POWER_MODE_NORMAL		0x1C
static int minnow_panel_check_panel_status(struct minnow_panel_data *mpd)
{
	u8 mode;
	int r;

	if (mpd->id_panel != MINNOW_PANEL_CM_BRIDGE_320X320)
		return 0;

	r = set_bridge_retrans(mpd, true);
	if (!r) {
		r = dsi_vc_dcs_read(mpd->dssdev, mpd->channel,
					MIPI_DCS_GET_POWER_MODE, &mode, 1);
		set_bridge_retrans(mpd, false);
	}
	if (r)
		return r;
	if (mode != DCS_POWER_MODE_NORMAL) {
		dev_err(&mpd->dssdev->dev,
			"panel is not On, power mode is 0x%02x\n", mode);
		return -ENODEV;
	}

	return 0;
}

#define	POWER_ON_RETRY_TIMES	3
static int minnow_panel_power_on(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	u8 id1, id2, id3;
	int r, retry = POWER_ON_RETRY_TIMES;

init_start:
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

	omapdss_dsi_vc_enable_hs(dssdev, mpd->channel, false);

	/* for the first time power on, do not reset h/w to keep logo on */
	if (mpd->first_enable && def_skip_first_init) {
		dsi_vc_send_bta_sync(dssdev, mpd->channel);
	} else {
		_minnow_panel_hw_reset(dssdev);
		r = minnow_panel_process_cmdbuf(mpd, &mpd->power_on);
		if (!r) {
			if (mpd->dummy_panel == DUMMY_DISABLED)
				r = minnow_panel_check_panel_status(mpd);
		}
		if (r) {
			if (!retry--)
				goto err;
			dev_err(&dssdev->dev, "Reset hardware to retry ...\n");
			omapdss_dsi_display_disable(dssdev, true, false);
			goto init_start;
		}
	}

	/* it needed enable TE to force update after display enabled */
	mpd->te_enabled = true;
	r = _minnow_panel_enable_te(dssdev, mpd->te_enabled);
	if (r)
		goto err;

	if (mpd->first_enable) {
		r = minnow_panel_get_id(mpd, &id1, &id2, &id3);
		if (r)
			goto err;
	}

	r = minnow_panel_dcs_write_0(mpd, MIPI_DCS_SET_DISPLAY_ON);
	if (r)
		goto err;

	omapdss_dsi_vc_enable_hs(dssdev, mpd->channel, true);

	r = dsi_enable_video_output(dssdev, mpd->channel);
	if (r)
		goto err;

	mpd->enabled = true;
	mpd->interactive = true;

	if (mpd->first_enable)
		dev_info(&dssdev->dev, "panel revision %02x.%02x.%02x\n",
			id1, id2, id3);

	return 0;
err:
	dev_err(&dssdev->dev, "error while enabling panel, issuing HW reset\n");

	_minnow_panel_hw_reset(dssdev);

	omapdss_dsi_display_disable(dssdev, true, false);
err0:
	return r;
}

static void minnow_panel_power_off(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int r;

	dsi_disable_video_output(dssdev, mpd->channel);

	r = minnow_panel_process_cmdbuf(mpd, &mpd->power_off);
	if (r)
		dev_err(&dssdev->dev,
			"error disabling panel, return %d\n", r);

	omapdss_dsi_display_disable(dssdev, true, false);

	mpd->enabled = false;
	mpd->interactive = false;
}

static int minnow_panel_reset(struct omap_dss_device *dssdev)
{
	dev_err(&dssdev->dev, "performing LCD reset\n");

	minnow_panel_power_off(dssdev);
	_minnow_panel_hw_reset(dssdev);
	return minnow_panel_power_on(dssdev);
}

static int minnow_panel_enable(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int r;

	dev_info(&dssdev->dev, "%s: current state = %d\n",
		 __func__, dssdev->state);

	mutex_lock(&mpd->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		r = -EINVAL;
		goto err;
	}

	r = minnow_panel_enable_clkin(mpd, true);
	if (r)
		goto err;
	r = minnow_panel_set_regulators(mpd, regulator_enable);
	if (r)
		goto err;
	minnow_panel_enable_vio(mpd, true);

	dsi_bus_lock(dssdev);

	r = minnow_panel_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r)
		goto err;

	minnow_panel_queue_esd_work(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	mutex_unlock(&mpd->lock);

	/* do not force update at first time if it needs keep logo on */
	if (!(mpd->first_enable && def_skip_first_init))
		r = minnow_panel_update(dssdev, 0, 0,
					dssdev->panel.timings.x_res,
					dssdev->panel.timings.y_res);

	dev_info(&dssdev->dev, "Display enabled, manual update ret = %d\n", r);
	mpd->first_enable = false;

	return 0;
err:
	dev_err(&dssdev->dev, "Display enable failed, err = %d\n", r);
	mutex_unlock(&mpd->lock);
	return r;
}

static void minnow_panel_disable(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	dev_info(&dssdev->dev, "%s: current state = %d\n",
		 __func__, dssdev->state);

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

	_minnow_panel_hw_active_reset(mpd);
	minnow_panel_enable_vio(mpd, false);
	minnow_panel_set_regulators(mpd, regulator_disable);
	minnow_panel_enable_clkin(mpd, false);

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
	int r = 0;

	/* for video mode, do not need manual update */
	if (mpd->dsi_config.mode == OMAP_DSS_DSI_VIDEO_MODE)
		return r;

	dev_dbg(&dssdev->dev, "update %d, %d, %d x %d\n", x, y, w, h);

	mutex_lock(&mpd->lock);
	dsi_bus_lock(dssdev);

	if (!mpd->enabled)
		goto err;

	r = minnow_panel_wake_up(dssdev);
	if (r)
		goto err;

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

	r = minnow_panel_check_panel_status(mpd);
	if (r) {
		dev_err(&dssdev->dev, "failed to read minnow-panel status\n");
		goto err;
	}

	if (!mpd->interactive)
		minnow_panel_enter_ulps(dssdev);

	dsi_bus_unlock(dssdev);

	minnow_panel_queue_esd_work(dssdev);
	mutex_unlock(&mpd->lock);

	return;
err:
	dev_err(&dssdev->dev, "performing LCD reset\n");

	dsi_bus_unlock(dssdev);
	mutex_unlock(&mpd->lock);

	minnow_panel_disable(dssdev);
	minnow_panel_enable(dssdev);
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
