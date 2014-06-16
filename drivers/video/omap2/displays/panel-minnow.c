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

/*#define PANEL_DEBUG*/
#define PANEL_PERF_TIME

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
#include <linux/wakelock.h>

#include <video/omapdss.h>
#include <video/omap-panel-data.h>
#include <video/mipi_display.h>

#include "../dss/dss.h"

#include "panel-minnow-common.h"

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

static u8 panel_off_common[] = {
/*n, type, data_0, data_1 ... data_n-1*/
1, DCS_WRITE_SYNC, MIPI_DCS_SET_DISPLAY_OFF,
1, DCS_WRITE, MIPI_DCS_ENTER_SLEEP_MODE,
1, WAIT_MS, 20,
0
};

#define	BRIDGE_WIDTH		320
#define	BRIDGE_HEIGHT		320
#define	PANEL_WIDTH		BRIDGE_WIDTH
#define	PANEL_HEIGHT		290
#define	UNUSED_LINES		(BRIDGE_HEIGHT-PANEL_HEIGHT)

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
		.hs = { 104000000, 150000000 },
		.lp = { 7000000, 9000000 },
		.panel_reset = { ACTIVE_LOW, 5, 10 },
		.bridge_reset = { ACTIVE_LOW, 20, 10 }
	},
};

#ifdef	PANEL_PERF_TIME
#define GET_ELAPSE_TIME(last)	jiffies_to_msecs((unsigned long)jiffies-last)
#endif
struct minnow_panel_data {
	struct mutex lock;		/* mutex */
	struct wake_lock wake_lock;	/* wake_lock */

	struct omap_dss_device *dssdev;

	/* panel HW configuration from DT or platform data */
	int reset_gpio[MINNOW_COMPONENT_MAX];
	int ext_te_gpio;
	int vio_en_gpio;
	int mem_en_gpio;
	struct minnow_panel_hw_reset hw_reset[MINNOW_COMPONENT_MAX];
	struct regulator *regulators[MINNOW_COMPONENT_MAX];
	struct clk *clk_in;
	bool clk_in_en;

#ifdef	CONFIG_PANEL_BACKLIGHT
	bool use_dsi_backlight;
	struct backlight_device *bldev;
#endif

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
	bool output_enabled;
	enum minnow_panel_type panel_type;

	bool te_enabled;

	atomic_t do_update;
	int channel;

	struct delayed_work te_timeout_work;

#ifdef	PANEL_DEBUG
	unsigned cabc_mode;
#endif

	bool first_enable;
	bool skip_first_init;
	int panel_retry_count;
	int esd_errors;

	struct workqueue_struct *workqueue;

	struct delayed_work esd_work;
	unsigned esd_interval;

	bool ulps_enabled;
	unsigned ulps_timeout;
	struct delayed_work ulps_work;

	int total_update;
	int total_error;
	int total_esd_reset;
#ifdef	PANEL_PERF_TIME
	unsigned long time_power_on;
	unsigned long time_ulps;
	unsigned long time_update;
	unsigned long time_update_min;
	unsigned long time_update_max;
	unsigned long last_power_on;
	unsigned long last_ulps;
	unsigned long last_update;
#endif
};

/* panel parameter passed from boot-loader */
static char *def_panel_param;
module_param_named(panel_param, def_panel_param, charp, 0);

static irqreturn_t minnow_panel_te_isr(int irq, void *data);
static void minnow_panel_te_timeout_work_callback(struct work_struct *work);
static int _minnow_panel_enable_te(struct minnow_panel_data *mpd, bool enable);

static int minnow_panel_wake_up_locked(struct minnow_panel_data *mpd);
static void minnow_panel_framedone_cb(int err, void *data);
static int minnow_panel_enable_locked(struct minnow_panel_data *mpd);
static void minnow_panel_disable_locked(struct minnow_panel_data *mpd,
					bool fast_power_off);
static int minnow_panel_update_locked(struct minnow_panel_data *mpd);

static void minnow_panel_esd_work(struct work_struct *work);
static void minnow_panel_ulps_work(struct work_struct *work);


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
	struct omap_dss_device *dssdev = out->device;
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int i, j, r;

	mutex_lock(&mpd->lock);
	if (!mpd->enabled) {
		seq_puts(s, "display is disabled!");
		goto exit1;
	}

	dsi_bus_lock(dssdev);
	r = minnow_panel_wake_up_locked(mpd);
	if (r) {
		seq_printf(s, "display wake up failed(%d)!\n", r);
		goto exit;
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
	dsi_bus_unlock(dssdev);
exit1:
	mutex_unlock(&mpd->lock);
}
#endif

static void minnow_panel_delay(int delay_ms)
{
	if (delay_ms > 5)
		msleep(delay_ms);
	else
		usleep_range(1000 * delay_ms, 1000 * delay_ms + 100);
}

static int panel_ssd2848_set_retransmit(struct minnow_panel_data *mpd,
					int enable)
{
	u8 data[2] = {0xFF, enable ? 0x01 : 0x00};
	return dsi_vc_generic_write(mpd->dssdev, mpd->channel, data, 2);
}

static int panel_ssd2848_read_reg(struct minnow_panel_data *mpd,
				  u16 addr, u8 *read)
{
	int r;
	dsi_vc_set_max_rx_packet_size(mpd->dssdev, mpd->channel, 4);
	r = dsi_vc_generic_read_2(mpd->dssdev, mpd->channel,
				  addr>>8, addr&0xFF, read, 4);
	dsi_vc_set_max_rx_packet_size(mpd->dssdev, mpd->channel, 1);
	return r;
}

static int panel_otm3201_read_reg(struct minnow_panel_data *mpd,
				   u8 addr, u8 *read, u8 len)
{
	int r;
	dsi_vc_set_max_rx_packet_size(mpd->dssdev, mpd->channel, len);
	r = dsi_vc_dcs_read(mpd->dssdev, mpd->channel, addr, read, len);
	dsi_vc_set_max_rx_packet_size(mpd->dssdev, mpd->channel, 1);
	return r;
}

#define WRITE_OTM3201(mpd, cmd)	\
	dsi_vc_dcs_write(mpd->dssdev, mpd->channel, cmd, sizeof(cmd))
static int panel_otm3201_rewrite_reg(struct minnow_panel_data *mpd,
				     u8 *data, u8 len, u8 *read)
{
	/* retry to write only when it could read back */
	int r, retry;

	for (retry = 3; retry--; ) {
		r = WRITE_OTM3201(mpd, otm3201_eng_mode);
		if (r)
			break;
		r = WRITE_OTM3201(mpd, otm3201_write_mode);
		if (r)
			break;
		r = dsi_vc_dcs_write(mpd->dssdev, mpd->channel, data, len);
		if (r)
			break;
		r = WRITE_OTM3201(mpd, otm3201_read_mode);
		if (r)
			break;
		r = panel_otm3201_read_reg(mpd, data[0], read, len-1);
		if (r)
			break;
		/* special register B5h */
		if (data[0] == 0xB5)
			read[3] |= data[4]&0x80;
		if (!memcmp(read, data+1, len-1))
			break;
	}

	return r;
}

static int minnow_panel_dcs_read_1(struct minnow_panel_data *mpd,
				   u8 dcs_cmd, u8 *data)
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

static int minnow_panel_dcs_write_1(struct minnow_panel_data *mpd,
				    u8 dcs_cmd, u8 param)
{
	u8 buf[2];
	buf[0] = dcs_cmd;
	buf[1] = param;
	return dsi_vc_dcs_write(mpd->dssdev, mpd->channel, buf, 2);
}

static int minnow_panel_get_id(struct minnow_panel_data *mpd,
			       u8 *id1, u8 *id2, u8 *id3)
{
	int r;

	if (mpd->id_panel == MINNOW_PANEL_CM_BRIDGE_320X320) {
		u8 data[4];
		r = panel_ssd2848_read_reg(mpd, 0, data);
		if (!r) {
			*id1 = data[0];
			*id2 = data[1];
			*id3 = mpd->panel_type;
		}
		return r;
	}

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
	if (!r) {
		buf[0] = MIPI_DCS_SET_PAGE_ADDRESS;
		buf[1] = (y1 >> 8) & 0xff;
		buf[2] = (y1 >> 0) & 0xff;
		buf[3] = (y2 >> 8) & 0xff;
		buf[4] = (y2 >> 0) & 0xff;

		r = dsi_vc_dcs_write_nosync(mpd->dssdev, mpd->channel,
					    buf, sizeof(buf));
		if (!r)
			r = dsi_vc_send_bta_sync(mpd->dssdev, mpd->channel);
	}

	return r;
}

/* since SSD2848 frame buffer is 320 x 320, but the actual panel is 320 x 290
 * it needs clear the unused bottom 30 lines for saving power
 */
static int minnow_panel_clear_bottom_line(struct minnow_panel_data *mpd,
					  unsigned int delay_ms)
{
	int r = 0;
#if	UNUSED_LINES
	unsigned int last_ms = jiffies_to_msecs(jiffies);
	int plen, total;
	u8 buf[124]; /* maximum packet size */

	memset(buf, 0, sizeof(buf));
	buf[0] = MIPI_DCS_WRITE_MEMORY_START;

	r = panel_ssd2848_set_retransmit(mpd, false);
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

	if (r) {
		/* waiting for the reset of time */
		last_ms = jiffies_to_msecs(jiffies) - last_ms;
		if (last_ms < delay_ms)
			minnow_panel_delay(delay_ms - last_ms);
	}
#else
	minnow_panel_delay(delay_ms);
#endif
	return r;
}

static int minnow_panel_process_cmd(struct minnow_panel_data *mpd,
				    u8 cmd, u8 *data, int len, bool retrans)
{
	int r = -EINVAL;

	if (retrans) {
		r = panel_ssd2848_set_retransmit(mpd, true);
		if (r)
			return r;
	}

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
		r = (int)(len == 1 ? (u32)(*data) : *(u16 *)data);
		/* It needs to clean bottom line for Solomon+Orise,
		 * and to save time it uses the waiting time(120 ms) of
		 * Orise's initialization process
		 */
		if ((r > 100) &&
		    (mpd->id_panel == MINNOW_PANEL_CM_BRIDGE_320X320))
			r = minnow_panel_clear_bottom_line(mpd, r);
		else {
			minnow_panel_delay(r);
			r = 0;
		}
		break;
	default:
		r = -EINVAL;
	}

	if (retrans && !r)
		r = panel_ssd2848_set_retransmit(mpd, false);

	return r;
}

static int minnow_panel_detect_type(struct minnow_panel_data *mpd)
{
	u8 rev;
	int r = panel_otm3201_read_reg(mpd, 0xB0, &rev, sizeof(rev));
	if (!r) {
		switch (rev) {
		case 0x12:
			mpd->panel_type = OTM3201_1_0;
			break;
		case 0x1A:
			mpd->panel_type = OTM3201_2_0;
			break;
		case 0x1C:
		default:
			mpd->panel_type = OTM3201_2_1;
			break;
		}
	}
	return r;
}

static void print_reg_mismatch(struct device *dev, u8 *src, u8 *read, int len)
{
	char *pstr, str[100];
	int i;
	for (i = 0, pstr = str; i < len; i++, pstr += 3)
		sprintf(pstr, " %02X", src[i]);
	dev_dbg(dev, "  Init:%s\n", str);
	for (i = 0, pstr = str; i < len; i++, pstr += 3)
		sprintf(pstr, " %02X", read[i]);
	dev_dbg(dev, "  Read:%s\n", str);
}

static int minnow_panel_verify_ssd2848(struct minnow_panel_data *mpd,
				       u8 *cmdbuf)
{
	u8 *data, read[4];
	int r = panel_ssd2848_set_retransmit(mpd, false);
	for (data = cmdbuf; *data && !r; data += *data+2) {
		u16 addr;
		if (data[1] != SSD2848_CMD)
			continue;
		if (data[2] == CMD_VERIFY_REG)
			break;
		addr = ((u16)data[2]<<8) | data[3];
		r = panel_ssd2848_read_reg(mpd, addr, read);
		if (r)
			break;
		if (!memcmp(&read, data+4, 4))
			continue;
		dev_err(&mpd->dssdev->dev, "Failed verify ssd2848"
			" register: %04X\n", addr);
		print_reg_mismatch(&mpd->dssdev->dev, data+4, read, 4);
		break;
	}

	return r;
}

static bool _minnow_panel_replace_cmdbuf(u8 *cmdbuf, u8 *data)
{
	for (; *cmdbuf; cmdbuf += *cmdbuf+2) {
		if ((data[0] != cmdbuf[0]) || (data[1] != cmdbuf[1]))
			continue;
		if (data[1] == SSD2848_CMD) {
			if ((data[2] != cmdbuf[2]) || (data[3] != cmdbuf[3]))
				continue;
			memcpy(cmdbuf+4, data+4, 4);
			return true;
		} else if (data[1] == OTM3201_CMD) {
			if (data[2] != cmdbuf[2])
				continue;
			memcpy(cmdbuf+3, data+3, cmdbuf[0]-1);
			return true;
		}
	}
	return false;
}

static bool minnow_panel_replace_cmdbuf(u8 *cmdbuf, u8 *data)
{
	bool ret = false;
	for (; *data; data += *data+2) {
		if (_minnow_panel_replace_cmdbuf(cmdbuf, data))
			ret = true;
	}
	return ret;
}

static int minnow_panel_verify_otm3201(struct minnow_panel_data *mpd,
				       u8 *cmdbuf)
{
	u8 *data, addr, read[20];
	int r = panel_ssd2848_set_retransmit(mpd, true);
	for (data = cmdbuf; *data && !r; data += *data+2) {
		if (data[1] != OTM3201_CMD)
			continue;
		addr = data[2];
		if (addr == CMD_VERIFY_REG) {
			break;
		} else if (addr == 0xF0) {
			/* resend command to make sure enable engineer mode */
			r = dsi_vc_dcs_write(mpd->dssdev, mpd->channel,
					     data+2, data[0]);
			continue;
		} else if (addr == 0xA0) {
			/* for initialization, it needs set write mode,
			 * but for verification, it needs set read mode
			 */
			r = WRITE_OTM3201(mpd, otm3201_read_mode);
			if (!r && (mpd->panel_type == PANEL_INIT)) {
				r = minnow_panel_detect_type(mpd);
				if (r)
					break;
				if (mpd->panel_type != OTM3201_1_0)
					continue;
				/* turn of ESD for panel 1.0 */
				mpd->esd_interval = 0;
				/* it needs replace some settings for 1.0 */
				if (minnow_panel_replace_cmdbuf
				    (cmdbuf, panel_init_ssd2848_320x320_1)) {
					dev_info(&mpd->dssdev->dev,
						 "Force reset for new settings"
						 " of panel 1.0!\n");
					mpd->panel_retry_count = 0;
					mpd->total_error--;
					r = -EIO;
					break;
				}
			}
			continue;
		}
		r = panel_otm3201_read_reg(mpd, addr, read, data[0]-1);
		if (r)
			break;
		/* special register B5h */
		if (addr == 0xB5)
			read[3] |= data[6]&0x80;
		if (!memcmp(&read, data+3, data[0]-1))
			continue;
		/* trying to rewrite register */
		r = panel_otm3201_rewrite_reg(mpd, data+2, data[0], read);
		if (r) {
			print_reg_mismatch(&mpd->dssdev->dev,
					   data+3, read, data[0]-1);
			break;
		}
	}
	if (r)
		dev_err(&mpd->dssdev->dev, "Failed verify otm3201"
			" register: %02X\n", addr);
	else
		r = panel_ssd2848_set_retransmit(mpd, false);

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
				       struct minnow_panel_cmd_buf *cmd_buf,
				       bool verify)
{
	u8 *data;
	int i, r;
	bool retrans = false;

	/* be safe to check command data every time before sent to driver */
	r = minnow_panel_check_cmdbuf(mpd, cmd_buf->cmdbuf, cmd_buf->count);
	if (r)
		return r;

	for (i = 0, data = cmd_buf->cmdbuf; *data; i++, data += *data+2) {
		if ((data[1] == SSD2848_CMD) &&
		    (data[0] == 1) && (data[2] == CMD_VERIFY_REG)) {
			if (!verify)
				continue;
			r = minnow_panel_verify_ssd2848(mpd, cmd_buf->cmdbuf);
		} else if ((data[1] == OTM3201_CMD) && (data[0] == 1) &&
			   (data[2] == CMD_VERIFY_REG)) {
			if (!verify)
				continue;
			r = minnow_panel_verify_otm3201(mpd, cmd_buf->cmdbuf);
		} else if (data[1] == SWITCH_TO_PANEL) {
			retrans = !!data[2];
		} else
			r = minnow_panel_process_cmd(mpd, data[1], data+2,
						     data[0], retrans);
		if (r) {
			dev_err(&mpd->dssdev->dev, "Failed process initialize"
				" command[%d] len=%d type=%d ret=%d\n",
				i, data[0], data[1], r);
			break;
		}
	}

	return r;
}

static void minnow_panel_queue_esd_work(struct minnow_panel_data *mpd)
{
	if (!mpd->esd_interval)
		return;
	queue_delayed_work(mpd->workqueue, &mpd->esd_work,
			   msecs_to_jiffies(mpd->esd_interval));
}

static void minnow_panel_cancel_esd_work(struct minnow_panel_data *mpd)
{
	cancel_delayed_work(&mpd->esd_work);
}

static void minnow_panel_queue_ulps_work(struct minnow_panel_data *mpd)
{
	if (!mpd->ulps_timeout)
		return;
	queue_delayed_work(mpd->workqueue, &mpd->ulps_work,
			   msecs_to_jiffies(mpd->ulps_timeout));
}

static void minnow_panel_cancel_ulps_work(struct minnow_panel_data *mpd)
{
	cancel_delayed_work(&mpd->ulps_work);
}

static int minnow_panel_dsi_recovery_locked(struct minnow_panel_data *mpd)
{
	struct omap_dss_device *dssdev = mpd->dssdev;
	int r;

	/* true/true for fast disable dsi */
	omapdss_dsi_display_disable(mpd->dssdev, true, true);
	mpd->ulps_enabled = false;
	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "DSI recovery failed to enable DSI\n");
		goto _ret_r_;
	}
	omapdss_dsi_vc_enable_hs(dssdev, mpd->channel, true);
	r = _minnow_panel_enable_te(mpd, mpd->te_enabled);
	/* for some reason, reset DSI may occur "false control" with bridge
	 * that will cause first command failed, so work around to try resend
	 * that check if it's still failed or not
	 */
	if (r)
		r = _minnow_panel_enable_te(mpd, mpd->te_enabled);
	if (r)
		dev_err(&dssdev->dev, "DSI recovery failed to re-init TE");
_ret_r_:
	return r;
}

static int minnow_panel_recovery_locked(struct minnow_panel_data *mpd)
{
	int r;
	dev_err(&mpd->dssdev->dev, "performing LCD reset\n");
	mpd->total_error++;
	mpd->total_esd_reset++;
	minnow_panel_disable_locked(mpd, true);
	msleep(20);
	r = minnow_panel_enable_locked(mpd);
	dev_err(&mpd->dssdev->dev, "LCD reset done(%d)\n", r);
	return r;
}

static int minnow_panel_enter_ulps_locked(struct minnow_panel_data *mpd)
{
	int r;

	if (mpd->ulps_enabled)
		return 0;

	minnow_panel_cancel_ulps_work(mpd);

	r = _minnow_panel_enable_te(mpd, false);
	if (r) {
		/* try once to recovery DSI */
		r = minnow_panel_dsi_recovery_locked(mpd);
		if (r)
			goto err;
		r = _minnow_panel_enable_te(mpd, false);
		if (r)
			goto err;
	}

	if (gpio_is_valid(mpd->ext_te_gpio))
		disable_irq(gpio_to_irq(mpd->ext_te_gpio));

	omapdss_dsi_display_disable(mpd->dssdev, false, true);

	mpd->ulps_enabled = true;
	dev_dbg(&mpd->dssdev->dev, "entered ULPS mode\n");
#ifdef	PANEL_PERF_TIME
	mpd->last_ulps = jiffies;
#endif

	return 0;

err:
	dev_err(&mpd->dssdev->dev, "enter ULPS failed\n");

	mpd->ulps_enabled = false;
	minnow_panel_queue_ulps_work(mpd);

	return r;
}

static int minnow_panel_exit_ulps_locked(struct minnow_panel_data *mpd)
{
	struct omap_dss_device *dssdev = mpd->dssdev;
	int r;

	if (!mpd->ulps_enabled)
		return 0;

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		/* try once to recovery DSI */
		r = minnow_panel_dsi_recovery_locked(mpd);
		if (!r)
			goto next;
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err;
	}

	omapdss_dsi_vc_enable_hs(dssdev, mpd->channel, true);

	r = _minnow_panel_enable_te(mpd, mpd->te_enabled);
	if (r) {
		/* try once to recovery DSI */
		r = minnow_panel_dsi_recovery_locked(mpd);
		if (r)
			goto err;
	}

next:
	if (gpio_is_valid(mpd->ext_te_gpio))
		enable_irq(gpio_to_irq(mpd->ext_te_gpio));

	minnow_panel_queue_ulps_work(mpd);

	mpd->ulps_enabled = false;

	dev_dbg(&dssdev->dev, "exited ULPS mode\n");

#ifdef	PANEL_PERF_TIME
	mpd->time_ulps += GET_ELAPSE_TIME(mpd->last_ulps);
#endif
	return 0;

err:
	dev_err(&dssdev->dev, "failed to exit ULPS\n");
	return r;
}

static int minnow_panel_wake_up_locked(struct minnow_panel_data *mpd)
{
	if (mpd->ulps_enabled)
		return minnow_panel_exit_ulps_locked(mpd);

	minnow_panel_cancel_ulps_work(mpd);
	minnow_panel_queue_ulps_work(mpd);
	return 0;
}

#ifdef	CONFIG_PANEL_BACKLIGHT
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

		r = minnow_panel_wake_up_locked(mpd);
		if (!r)
			r = minnow_panel_dcs_write_1(mpd, DCS_BRIGHTNESS,
						     level);

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
#endif

static void minnow_panel_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static ssize_t minnow_panel_errors_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	static struct { u16 addr; const char *name; } dump[] = {
		{0x1018, "MIPIRX-Phy Error"},
		{0x102C, "MIPIRX-DSI Error"},
		{0x1030, "MIPIRX-DSI Error Count"},
		{0x608C, "MIPITX-DSI0 Status"},
	};
	u8 read[6];
	int i, r, len = 0;

	mutex_lock(&mpd->lock);
	len += snprintf(buf+len, PAGE_SIZE-len, "Updates: %d\n",
			mpd->total_update);
	len += snprintf(buf+len, PAGE_SIZE-len, "Errors:  %d\n",
			mpd->total_error);
	len += snprintf(buf+len, PAGE_SIZE-len, "ESD RST: %d\n",
			mpd->total_esd_reset);
	dsi_bus_lock(dssdev);

	if (!mpd->enabled || (mpd->id_panel != MINNOW_PANEL_CM_BRIDGE_320X320))
		goto _ret_;

	r = minnow_panel_wake_up_locked(mpd);
	if (r) {
		len += snprintf(buf+len, PAGE_SIZE-len, "Failed to wakeup!\n");
		goto _ret_;
	}

	for (i = 0; i < sizeof(dump)/sizeof(dump[0]); i++) {
		r = panel_ssd2848_read_reg(mpd, dump[i].addr, read+2);
		if (r)
			len += snprintf(buf+len, PAGE_SIZE-len,
					"Failed read register %s\n",
					dump[i].name);
		else
			len += snprintf(buf+len, PAGE_SIZE-len,
					"%s:\t%02X%02X%02X%02X\n",
					dump[i].name, read[2], read[3],
					read[4], read[5]);
		if (dump[i].addr != 0x608C)
			continue;
		/* Cleaning MIPITX-DSI0 Status */
		read[0] = 0x60;
		read[1] = 0x8C;
		dsi_vc_generic_write(mpd->dssdev, mpd->channel, read, 6);
	}

_ret_:
	dsi_bus_unlock(dssdev);
	mutex_unlock(&mpd->lock);

	return len;
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

		r = minnow_panel_wake_up_locked(mpd);
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

#ifdef	PANEL_DEBUG
static const char *cabc_modes[] = {
	"off",		/* used also always when CABC is not supported */
	"ui",
	"still-image",
	"moving-image",
};

static ssize_t show_cabc_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	const char *mode_str;
	int mode;

	mode = mpd->cabc_mode;

	mode_str = "unknown";
	if (mode >= 0 && mode < ARRAY_SIZE(cabc_modes))
		mode_str = cabc_modes[mode];

	return snprintf(buf, PAGE_SIZE, "%s\n", mode_str);
}

static ssize_t store_cabc_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
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

		r = minnow_panel_wake_up_locked(mpd);
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
	struct device_attribute *attr, char *buf)
{
	int i, len;

	for (i = 0, len = 0;
	     len < PAGE_SIZE && i < ARRAY_SIZE(cabc_modes); i++)
		len += snprintf(&buf[len], PAGE_SIZE-len, "%s%s%s",
			i ? " " : "", cabc_modes[i],
			i == ARRAY_SIZE(cabc_modes) - 1 ? "\n" : "");

	return len;
}
#endif

static ssize_t minnow_panel_store_esd_interval(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	unsigned long t;
	int r;

	r = strict_strtoul(buf, 10, &t);
	if (r)
		return r;

	mutex_lock(&mpd->lock);
	minnow_panel_cancel_esd_work(mpd);
	/* special settings for test purpose */
	switch (t) {
	case 1: case 2:
		/* active panel/bridge reset to force ESD */
		t--;
		dev_info(&mpd->dssdev->dev, "ESD test to force %s reset\n",
			 t == MINNOW_PANEL ? "panel" : "bridge");
		gpio_set_value(mpd->reset_gpio[t],
			       mpd->hw_reset[t].active ? 1 : 0);
		break;
	case 3:
		dsi_bus_lock(dssdev);
		dev_info(&mpd->dssdev->dev, "ESD test for DSI recovery\n");
		r = minnow_panel_exit_ulps_locked(mpd);
		dev_info(&mpd->dssdev->dev,
			 "minnow_panel_exit_ulps_locked = %d\n", r);
		r = minnow_panel_dsi_recovery_locked(mpd);
		dev_info(&mpd->dssdev->dev,
			 "minnow_panel_dsi_recovery_locked = %d\n", r);
		r = minnow_panel_enter_ulps_locked(mpd);
		dev_info(&mpd->dssdev->dev,
			 "minnow_panel_enter_ulps_locked = %d\n", r);
		r = minnow_panel_dsi_recovery_locked(mpd);
		dev_info(&mpd->dssdev->dev,
			 "minnow_panel_dsi_recovery_locked = %d\n", r);
		dsi_bus_unlock(dssdev);
		break;
	case 4:
		dsi_bus_lock(dssdev);
		dev_info(&mpd->dssdev->dev, "ESD test for panel recovery\n");
		minnow_panel_disable_locked(mpd, true);
		dev_info(&mpd->dssdev->dev,
			 "minnow_panel_disable_locked done\n");
		msleep(20);
		dev_info(&mpd->dssdev->dev,
			 "minnow_panel_enable_locked start\n");
		r = minnow_panel_enable_locked(mpd);
		dev_info(&mpd->dssdev->dev,
			 "minnow_panel_enable_locked = %d\n", r);
		if (!r) {
			r = minnow_panel_update_locked(mpd);
			/* no dsi_bus_unlock when update start successfully */
			if (!r)
				break;
		}
		dsi_bus_unlock(dssdev);
	default:
		mpd->esd_interval = t;
		break;
	}
	if (mpd->enabled)
		minnow_panel_queue_esd_work(mpd);
	mutex_unlock(&mpd->lock);

	return count;
}

static ssize_t minnow_panel_show_esd_interval(struct device *dev,
	struct device_attribute *attr, char *buf)
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
	struct device_attribute *attr, const char *buf, size_t count)
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
			r = minnow_panel_enter_ulps_locked(mpd);
		else
			r = minnow_panel_wake_up_locked(mpd);

		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&mpd->lock);

	return r ? r : count;
}

static ssize_t minnow_panel_show_ulps(struct device *dev,
	struct device_attribute *attr, char *buf)
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
	struct device_attribute *attr, const char *buf, size_t count)
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
		/* minnow_panel_wake_up_locked will restart the timer */
		dsi_bus_lock(dssdev);
		r = minnow_panel_wake_up_locked(mpd);
		dsi_bus_unlock(dssdev);
	}

	mutex_unlock(&mpd->lock);

	return r ? r : count;
}

static ssize_t minnow_panel_show_ulps_timeout(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	unsigned t;

	mutex_lock(&mpd->lock);
	t = mpd->ulps_timeout;
	mutex_unlock(&mpd->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", t);
}

#ifdef	PANEL_DEBUG
static ssize_t minnow_panel_store_init_data(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
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
	struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int i, j;
	u8 *data;

	mutex_lock(&mpd->lock);
	data = mpd->power_on.cmdbuf;
	mutex_unlock(&mpd->lock);

	for (i = 0; i < PAGE_SIZE && *data; ) {
		i += snprintf(buf+i, PAGE_SIZE-i,
			      "%02d %02d:", data[0], data[1]);
		for (j = 0; j < *data && i < PAGE_SIZE; j++) {
			i + snprintf(buf+i, PAGE_SIZE-i, " %02X", data[2+j]);
		}
		snprintf(buf+i, PAGE_SIZE-i, "\n");
		i++;
		data += *data + 2;
	}

	return i;
}
#endif

static ssize_t minnow_panel_show_interactivemode(struct device *dev,
	struct device_attribute *attr, char *buf)
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
	struct device_attribute *attr, const char *buf, size_t count)
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
				r = minnow_panel_wake_up_locked(mpd);
			else
				r = minnow_panel_enter_ulps_locked(mpd);
			dsi_bus_unlock(dssdev);
		}
		mutex_unlock(&mpd->lock);
		dev_info(&dssdev->dev, "%s interactive mode%s\n",
			 enable ? "enable" : "disable", r ? " failed" : "");
	}

	return r ? r : count;
}

#ifdef	PANEL_PERF_TIME
static ssize_t minnow_panel_perftime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	int len = 0;

	mutex_lock(&mpd->lock);
	if (mpd->enabled) {
		mpd->time_power_on += GET_ELAPSE_TIME(mpd->last_power_on);
		mpd->last_power_on = jiffies;
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "Power On:     %lu ms\n",
			mpd->time_power_on);
	len += snprintf(buf+len, PAGE_SIZE-len, "Enter ULPS:   %lu ms\n",
			mpd->time_ulps);
	len += snprintf(buf+len, PAGE_SIZE-len, "Update Frame: %lu ms\n",
			mpd->time_update);
	len += snprintf(buf+len, PAGE_SIZE-len, "  Frame Min:  %lu ms\n",
			mpd->time_update_min);
	len += snprintf(buf+len, PAGE_SIZE-len, "  Frame Max:  %lu ms\n",
			mpd->time_update_max);
	len += snprintf(buf+len, PAGE_SIZE-len, "  Frame Avg:  %lu ms\n",
			mpd->time_update / mpd->total_update);
	mutex_unlock(&mpd->lock);

	return len;
}
#endif

static DEVICE_ATTR(errors, S_IRUGO, minnow_panel_errors_show, NULL);
static DEVICE_ATTR(hw_revision, S_IRUGO, minnow_panel_hw_revision_show, NULL);
static DEVICE_ATTR(esd_interval, S_IRUGO | S_IWUSR,
		   minnow_panel_show_esd_interval,
		   minnow_panel_store_esd_interval);
static DEVICE_ATTR(ulps, S_IRUGO | S_IWUSR,
		   minnow_panel_show_ulps, minnow_panel_store_ulps);
static DEVICE_ATTR(ulps_timeout, S_IRUGO | S_IWUSR,
		   minnow_panel_show_ulps_timeout,
		   minnow_panel_store_ulps_timeout);
#ifdef	PANEL_DEBUG
static DEVICE_ATTR(cabc_mode, S_IRUGO | S_IWUSR,
		   show_cabc_mode, store_cabc_mode);
static DEVICE_ATTR(cabc_available_modes, S_IRUGO,
		   show_cabc_available_modes, NULL);
static DEVICE_ATTR(init_data, S_IRUGO | S_IWUSR,
		   minnow_panel_show_init_data,
		   minnow_panel_store_init_data);
#endif
static DEVICE_ATTR(interactivemode, S_IRUGO | S_IWUSR,
		   minnow_panel_show_interactivemode,
		   minnow_panel_store_interactivemode);
#ifdef	PANEL_PERF_TIME
static DEVICE_ATTR(perftime, S_IRUGO, minnow_panel_perftime_show, NULL);
#endif

static struct attribute *minnow_panel_attrs[] = {
	&dev_attr_errors.attr,
	&dev_attr_hw_revision.attr,
	&dev_attr_esd_interval.attr,
	&dev_attr_ulps.attr,
	&dev_attr_ulps_timeout.attr,
#ifdef	PANEL_DEBUG
	&dev_attr_cabc_mode.attr,
	&dev_attr_cabc_available_modes.attr,
	&dev_attr_init_data.attr,
#endif
	&dev_attr_interactivemode.attr,
#ifdef	PANEL_PERF_TIME
	&dev_attr_perftime.attr,
#endif
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
	minnow_panel_delay(mpd->reset_ms);
}

static void _minnow_panel_hw_reset(struct minnow_panel_data *mpd)
{
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
		minnow_panel_delay(mpd->release_ms);
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
	if (enable) {
		if (gpio_is_valid(mpd->vio_en_gpio))
			gpio_set_value(mpd->vio_en_gpio, 0);
		if (gpio_is_valid(mpd->mem_en_gpio))
			gpio_set_value(mpd->mem_en_gpio, 1);
	} else {
		if (gpio_is_valid(mpd->mem_en_gpio))
			gpio_set_value(mpd->mem_en_gpio, 0);
		if (gpio_is_valid(mpd->vio_en_gpio))
			gpio_set_value(mpd->vio_en_gpio, 1);
	}
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
	mpd->mem_en_gpio = of_get_named_gpio(dt_node, "gpio_mem_en", 0);
	DTINFO("gpio_mem_en = %d\n", mpd->mem_en_gpio);
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
#ifdef	CONFIG_PANEL_BACKLIGHT
	mpd->use_dsi_backlight = false;
#endif

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

static int minnow_panel_parse_panel_param(char *param, int *ptype, int *pver)
{
	char *p, *start = (char *)param;
	int ver, type;

	if (!param)
		return -EINVAL;
	type = simple_strtoul(start, &p, 10);
	if (start == p)
		return -EINVAL;
	if (*p != '#')
		return -EINVAL;
	start = p + 1;
	ver = simple_strtoul(start, &p, 10);
	if (start == p)
		return -EINVAL;
	if (*p != '\0')
		return -EINVAL;
	*ptype = type;
	*pver = ver;
	return 0;
}

static int minnow_panel_probe(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd;
#ifdef	CONFIG_PANEL_BACKLIGHT
	struct backlight_device *bldev = NULL;
#endif
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
#ifdef	PANEL_PERF_TIME
	mpd->time_update_min = (unsigned long)(-1);
	mpd->time_update_max = 0;
#endif
	/* it will reset bridge/panel if boot-loader does not initialize it */
	mpd->skip_first_init = false;
	if (minnow_panel_parse_panel_param(def_panel_param, &i, &r)) {
		dev_err(&dssdev->dev, "wrong panel parameter %s\n",
			def_panel_param);
		i = PANEL_INIT;
	}
	mpd->panel_type = i;
	switch (mpd->panel_type) {
	case PANEL_INIT:
	case PANEL_DUMMY:
		dev_info(&dssdev->dev,
			 "There is not panel id coming from boot-loader\n");
		break;
	case OTM3201_1_0:
		/* turn of ESD for panel 1.0 */
		mpd->esd_interval = 0;
		/* it needs replace the settings for panel 1.0 */
		if (minnow_panel_replace_cmdbuf(mpd->power_on.cmdbuf,
						panel_init_ssd2848_320x320_1)){
			dev_info(&dssdev->dev, "Replaced for the settings "
				 "of panel 1.0!\n");
		}
	case OTM3201_2_0:
	case OTM3201_2_1:
		if (r == INIT_DATA_VERSION)
			mpd->skip_first_init = true;
		else
			dev_err(&dssdev->dev, "Initialize version mismatch"
				" (%d-%d)!\n", r, INIT_DATA_VERSION);
		break;
	default:
		dev_err(&dssdev->dev,
			"Wrong panel id(%d) got from boot-loader\n",
			mpd->panel_type);
		break;
	}
	dev_info(&dssdev->dev, "skip first time initialization is %s\n",
		 mpd->skip_first_init ? "enabled" : "disabled");

	if (gpio_is_valid(mpd->vio_en_gpio)) {
		r = devm_gpio_request_one(&dssdev->dev, mpd->vio_en_gpio,
					  mpd->skip_first_init
					  ? GPIOF_OUT_INIT_LOW
					  : GPIOF_OUT_INIT_HIGH,
					  "minnow-panel vio_en");
		if (r) {
			dev_err(&dssdev->dev,
				"failed to request panel vio_en gpio\n");
			return r;
		}
	}

	if (gpio_is_valid(mpd->mem_en_gpio)) {
		r = devm_gpio_request_one(&dssdev->dev, mpd->mem_en_gpio,
					  mpd->skip_first_init
					  ? GPIOF_OUT_INIT_HIGH
					  : GPIOF_OUT_INIT_LOW,
					  "minnow-panel mem_en");
		if (r) {
			dev_err(&dssdev->dev,
				"failed to request panel mem_en gpio %d\n",
				mpd->mem_en_gpio);
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
		bool active_low = (mpd->hw_reset[i].active == ACTIVE_LOW);
		if (!gpio_is_valid(mpd->reset_gpio[i]))
			continue;
		/* skip_first_init  hw_reset.active  gpio_init
		 *      Y             ACTIVE_LOW     INIT_HIGH
		 *      Y             ACTIVE_HIGH    INIT_LOW
		 *      N             ACTIVE_LOW     INIT_LOW
		 *      N             ACTIVE_HIGH    INIT_HIGH
		 */
		r = devm_gpio_request_one(&dssdev->dev, mpd->reset_gpio[i],
					  (mpd->skip_first_init ^ active_low)
					  ? GPIOF_OUT_INIT_LOW
					  : GPIOF_OUT_INIT_HIGH,
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
		if (!mpd->skip_first_init) {
			/* toggle enable/disable to force disable */
			r = regulator_enable(rt);
			r = regulator_disable(rt);
		}
	}

	mpd->workqueue = create_singlethread_workqueue("minnow_panel_esd");
	if (mpd->workqueue == NULL) {
		dev_err(&dssdev->dev, "can't create ESD workqueue\n");
		return -ENOMEM;
	}
	INIT_DEFERRABLE_WORK(&mpd->esd_work, minnow_panel_esd_work);
	INIT_DELAYED_WORK(&mpd->ulps_work, minnow_panel_ulps_work);

#ifdef	CONFIG_PANEL_BACKLIGHT
	if (mpd->use_dsi_backlight) {
		struct backlight_properties props;
		memset(&props, 0, sizeof(struct backlight_properties));
		props.max_brightness = 255;

		props.type = BACKLIGHT_RAW;
		bldev = backlight_device_register(dev_name(&dssdev->dev),
						  &dssdev->dev, dssdev,
						  &minnow_panel_bl_ops,
						  &props);
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
#endif

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
	wake_lock_init(&mpd->wake_lock, WAKE_LOCK_SUSPEND, "minnow-panel");

	return 0;

err_vc_id:
	omap_dsi_release_vc(dssdev, mpd->channel);
err_req_vc:
#ifdef	CONFIG_PANEL_BACKLIGHT
	if (bldev != NULL)
		backlight_device_unregister(bldev);
err_bl:
#endif
	destroy_workqueue(mpd->workqueue);
	return r;
}

static void __exit minnow_panel_remove(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "remove\n");

	sysfs_remove_group(&dssdev->dev.kobj, &minnow_panel_attr_group);
	omap_dsi_release_vc(dssdev, mpd->channel);

#ifdef	CONFIG_PANEL_BACKLIGHT
	if (mpd->bldev != NULL) {
		struct backlight_device *bldev = mpd->bldev;
		bldev->props.power = FB_BLANK_POWERDOWN;
		minnow_panel_bl_update_status(bldev);
		backlight_device_unregister(bldev);
	}
#endif

	minnow_panel_cancel_ulps_work(mpd);
	minnow_panel_cancel_esd_work(mpd);
	destroy_workqueue(mpd->workqueue);

	/* reset, to be sure that the panel is in a valid state */
	_minnow_panel_hw_reset(mpd);
}

#define	DCS_POWER_MODE_NORMAL		0x1C
#define	ERROR_UNEXPECT_MODE(mode)	(0x12345600 | (mode))
/* panel 1.0 has timing issue that may occur missing command or ECC error
 * then it will receive 00 from Solomon, so for this case, retry to read
 * might be better
 */
#define IS_ERR_UNEXPECT_MODE_00(err)	((err) == 0x12345600)
static int minnow_panel_check_panel_status(struct minnow_panel_data *mpd)
{
	u8 mode;
	int r;

	if (mpd->id_panel != MINNOW_PANEL_CM_BRIDGE_320X320)
		return 0;

	r =  panel_ssd2848_set_retransmit(mpd, true);
	if (!r) {
		r = panel_otm3201_read_reg(mpd, MIPI_DCS_GET_POWER_MODE,
					   &mode, 1);
		if (!r)
			r = panel_ssd2848_set_retransmit(mpd, false);
	}

	if (r)
		dev_dbg(&mpd->dssdev->dev, "unable to get panel power mode\n");
	else if (mode != DCS_POWER_MODE_NORMAL) {
		dev_err(&mpd->dssdev->dev,
			"panel is not On, power mode is 0x%02x\n", mode);
		r = ERROR_UNEXPECT_MODE(mode);
	}

	return r;
}

#define	POWER_ON_RETRY_TIMES	3
static int minnow_panel_power_on(struct minnow_panel_data *mpd)
{
	struct omap_dss_device *dssdev = mpd->dssdev;
	u8 id1, id2, id3;
	int r;
	bool need_verify = true;

	mpd->panel_retry_count = 0;
	mpd->esd_errors = 0;

init_start:
	mpd->panel_retry_count++;
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
	if (mpd->output_enabled) {
		dsi_disable_video_output(mpd->dssdev, mpd->channel);
		mpd->output_enabled = false;
	}

	omapdss_dsi_vc_enable_hs(dssdev, mpd->channel, false);

	/* for the first time power on, do not reset h/w to keep logo on */
	if (mpd->first_enable && mpd->skip_first_init) {
		dsi_vc_send_bta_sync(dssdev, mpd->channel);
	} else {
		_minnow_panel_hw_reset(mpd);
		r = minnow_panel_process_cmdbuf(mpd, &mpd->power_on,
						need_verify);
		if (!r && (mpd->panel_type != PANEL_DUMMY)) {
			/* check if panel power on correctly, it may
			 * get power on mode with 0 sometimes, but it
			 * could not prove panel does not work, needs
			 * retry to check it again.
			 */
			int retry = 3;
			do {
				r = minnow_panel_check_panel_status(mpd);
			} while (retry-- && IS_ERR_UNEXPECT_MODE_00(r));
		}
		if (r) {
			if (mpd->panel_retry_count >= POWER_ON_RETRY_TIMES) {
				if (mpd->panel_type == PANEL_INIT) {
					dev_err(&dssdev->dev,
						"No panel to be detected, "
						"using dummy panel instead\n");
					mpd->panel_type = PANEL_DUMMY;
				} else {
					if (!need_verify)
						goto err;
				}
				/* try without read back check */
				need_verify = false;
			}
			mpd->total_error++;
			dev_err(&dssdev->dev, "Reset hardware to retry ...\n");
			/* true/true for fast disable dsi */
			omapdss_dsi_display_disable(dssdev, true, true);
			goto init_start;
		}
	}

	/* it needed enable TE to force update after display enabled */
	mpd->te_enabled = true;
	r = _minnow_panel_enable_te(mpd, mpd->te_enabled);
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
	mpd->output_enabled = true;

	if (mpd->first_enable)
		dev_info(&dssdev->dev, "panel revision %02x.%02x.%02x\n",
			 id1, id2, id3);
#ifdef	PANEL_PERF_TIME
	mpd->last_power_on = jiffies;
#endif
	return 0;
err:
	mpd->total_error++;
	dev_err(&dssdev->dev,
		"error while enabling panel, issuing HW reset\n");
	_minnow_panel_hw_active_reset(mpd);

	omapdss_dsi_display_disable(dssdev, true, false);
err0:
	return r;
}

static void minnow_panel_power_off(struct minnow_panel_data *mpd,
				   bool fast_power_off)
{
	int r;

#ifdef	PANEL_PERF_TIME
	if (mpd->enabled)
		mpd->time_power_on += GET_ELAPSE_TIME(mpd->last_power_on);
#endif
	if (!fast_power_off) {
		dsi_disable_video_output(mpd->dssdev, mpd->channel);
		mpd->output_enabled = false;

		r = minnow_panel_process_cmdbuf(mpd, &mpd->power_off, false);
		if (r)
			dev_err(&mpd->dssdev->dev,
				"error disabling panel, return %d\n", r);
	}
	/* true/true for fast disable DSI */
	omapdss_dsi_display_disable(mpd->dssdev, true, fast_power_off);

	mpd->enabled = false;
	mpd->interactive = false;
	mpd->ulps_enabled = false;
}

static void minnow_panel_disable_locked(struct minnow_panel_data *mpd,
					bool fast_power_off)
{
	minnow_panel_cancel_ulps_work(mpd);
	minnow_panel_power_off(mpd, fast_power_off);
	mpd->dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	_minnow_panel_hw_active_reset(mpd);
	minnow_panel_enable_clkin(mpd, false);
	minnow_panel_enable_vio(mpd, false);
	minnow_panel_set_regulators(mpd, regulator_disable);
}

static int minnow_panel_enable_locked(struct minnow_panel_data *mpd)
{
	int r;

	r = minnow_panel_set_regulators(mpd, regulator_enable);
	if (r)
		goto err;
	minnow_panel_enable_vio(mpd, true);
	r = minnow_panel_enable_clkin(mpd, true);
	if (r)
		goto err;

	r = minnow_panel_power_on(mpd);
	if (r)
		goto err;

	minnow_panel_queue_esd_work(mpd);
	mpd->dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;

err:
	dev_err(&mpd->dssdev->dev, "Display enable failed, err = %d\n", r);

	/* clean up clk/power */
	minnow_panel_disable_locked(mpd, true);
	return r;
}

static int minnow_panel_update_locked(struct minnow_panel_data *mpd)
{
	int r = 0;

	/* hold wake_lock to avoid kernel suspend */
	wake_lock(&mpd->wake_lock);

	/* XXX no need to send this every frame, but dsi break if not done */
	r = minnow_panel_set_update_window(mpd, 0, 0, 
					   mpd->dssdev->panel.timings.x_res,
					   mpd->dssdev->panel.timings.y_res);
	if (r)
		goto err;

	if (mpd->te_enabled && gpio_is_valid(mpd->ext_te_gpio)) {
		schedule_delayed_work(&mpd->te_timeout_work,
				      msecs_to_jiffies(250));
		atomic_set(&mpd->do_update, 1);
	} else {
		r = omap_dsi_update(mpd->dssdev, mpd->channel,
				    minnow_panel_framedone_cb, mpd->dssdev);
		if (r)
			goto err;
	}

#ifdef	PANEL_PERF_TIME
	mpd->last_update = jiffies;
#endif
	/* No wake_unlock here, unlock will be done in framedone_cb */
	return r;
err:
	wake_unlock(&mpd->wake_lock);
	return r;
}

static int minnow_panel_enable(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	bool update;
	int r = -EINVAL;

	dev_info(&dssdev->dev, "%s: current state = %d\n",
		 __func__, dssdev->state);

	if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED) {
		mutex_lock(&mpd->lock);
		minnow_panel_cancel_ulps_work(mpd);
		minnow_panel_cancel_esd_work(mpd);

		dsi_bus_lock(dssdev);
		r = minnow_panel_enable_locked(mpd);
		/* do not force update at first time to keep boot logo on */
		update = !r && !(mpd->first_enable && mpd->skip_first_init);
		mpd->first_enable = false;
		if (update)
			update = !minnow_panel_update_locked(mpd);
		/* it will release dsi_bus_unlock in frame done callback when
		 *   update start successfully
		 */
		if (!update)
			dsi_bus_unlock(dssdev);
		if (!r) {
			minnow_panel_queue_ulps_work(mpd);
			minnow_panel_queue_esd_work(mpd);
			dev_info(&dssdev->dev, "Display enabled successfully "
				 "%s update!\n", update ? "with" : "without");
		}
		mutex_unlock(&mpd->lock);
	}

	return r;
}

static void minnow_panel_disable(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);

	dev_info(&dssdev->dev, "%s: current state = %d\n",
		 __func__, dssdev->state);

	mutex_lock(&mpd->lock);
	minnow_panel_cancel_ulps_work(mpd);
	minnow_panel_cancel_esd_work(mpd);

	dsi_bus_lock(dssdev);
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		/* if it can not wakeup, do fast disable */
		bool fast = minnow_panel_wake_up_locked(mpd) != 0;
		minnow_panel_disable_locked(mpd, fast);
	}
	dsi_bus_unlock(dssdev);

	mutex_unlock(&mpd->lock);
}

#if defined(CONFIG_HAS_AMBIENTMODE)
static int minnow_panel_suspend(struct omap_dss_device *dssdev)
{
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	dev_dbg(&dssdev->dev, "%s: current state = %d, wake_lock:%d\n",
		__func__, dssdev->state, wake_lock_active(&mpd->wake_lock));

	mutex_lock(&mpd->lock);
	if (mpd->enabled) {
		dsi_bus_lock(dssdev);
		minnow_panel_enter_ulps_locked(mpd);
		dsi_bus_unlock(dssdev);
	}
	mutex_unlock(&mpd->lock);
	return 0;
}
#endif

static void minnow_panel_framedone_cb(int err, void *data)
{
	struct omap_dss_device *dssdev = data;
	struct minnow_panel_data *mpd = dev_get_drvdata(&dssdev->dev);
	dev_dbg(&dssdev->dev, "framedone, err %d\n", err);
	mpd->total_update++;
#ifdef	PANEL_PERF_TIME
{
	unsigned long ms = GET_ELAPSE_TIME(mpd->last_update);
	if (ms < mpd->time_update_min)
		mpd->time_update_min = ms;
	if (ms > mpd->time_update_max)
		mpd->time_update_max = ms;
	mpd->time_update += ms;
}
#endif
	dsi_bus_unlock(dssdev);
	wake_unlock(&mpd->wake_lock);
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

	/* if driver is disabled ot it's video mode, do not manual update */
	mutex_lock(&mpd->lock);
	if (mpd->enabled && (mpd->dsi_config.mode!=OMAP_DSS_DSI_VIDEO_MODE)) {
		int recovered = 0;
		dev_dbg(&dssdev->dev, "update %d, %d, %d-%d\n", x, y, w, h);

		dsi_bus_lock(dssdev);
		r = minnow_panel_wake_up_locked(mpd);
		if (!r)
			goto _update_;
	_recovery_:
		/* try recovery panel if it can't wake up */
		r = minnow_panel_recovery_locked(mpd);
		if (r)
			goto _dsi_unlock_;
	_update_:
		r = minnow_panel_update_locked(mpd);
		/* no dsi_bus_unlock when update start successfully */
		if (!r)
			goto _mutex_unlock_;
		/* try if it need recovery once */
		if (recovered++)
			goto _dsi_unlock_;
		/* be safety, check the panel status to make sure it's stuck now */
		r = minnow_panel_check_panel_status(mpd);
		if (r)
			goto _recovery_;
	_dsi_unlock_:
		dev_err(&dssdev->dev, "update %d, %d, %d-%d failed(%d)\n", x, y, w, h, r);
		dsi_bus_unlock(dssdev);
	}
_mutex_unlock_:
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

static int _minnow_panel_enable_te(struct minnow_panel_data *mpd, bool enable)
{
	int r;

	if (enable)
		r = minnow_panel_dcs_write_1(mpd, MIPI_DCS_SET_TEAR_ON, 0);
	else
		r = minnow_panel_dcs_write_0(mpd, MIPI_DCS_SET_TEAR_OFF);

	if (!gpio_is_valid(mpd->ext_te_gpio))
		omapdss_dsi_enable_te(mpd->dssdev, enable);

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
		r = minnow_panel_wake_up_locked(mpd);
		if (r)
			goto err;

		r = _minnow_panel_enable_te(mpd, enable);
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

#ifdef	PANEL_DEBUG
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

	r = minnow_panel_wake_up_locked(mpd);
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
	void *buf, size_t size, u16 x, u16 y, u16 w, u16 h)
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

	r = minnow_panel_wake_up_locked(mpd);
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
#endif

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
	minnow_panel_enter_ulps_locked(mpd);
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

	r = minnow_panel_wake_up_locked(mpd);
	if (r) {
		dev_err(&dssdev->dev, "failed to exit ULPS\n");
		goto _reset_;
	}

	omapdss_dsi_vc_enable_hs(dssdev, mpd->channel, false);
	r = minnow_panel_check_panel_status(mpd);
	if (r) {
		if (IS_ERR_UNEXPECT_MODE_00(r)) {
			/* workaround to assume panel is good till it gets
			 * power state 00 for 3 continue times
			 */
			if (++mpd->esd_errors < 3)
				goto _next_;
		}
		dev_err(&dssdev->dev, "failed to read minnow-panel status\n");
		goto _reset_;
	}
	mpd->esd_errors = 0;

_next_:
	omapdss_dsi_vc_enable_hs(dssdev, mpd->channel, true);
	if (!mpd->interactive)
		minnow_panel_enter_ulps_locked(mpd);
	else
		minnow_panel_queue_ulps_work(mpd);

	dsi_bus_unlock(dssdev);
	mutex_unlock(&mpd->lock);
	return;

_reset_:
	r = minnow_panel_recovery_locked(mpd);
	if (!r) {
		r = minnow_panel_update_locked(mpd);
		/* it will release dsi_bus_unlock in frame done callback when
		 *   update start successfully
		 */
		if (!r)
			goto _munlock_;
		if (!mpd->interactive)
			minnow_panel_enter_ulps_locked(mpd);
		else
			minnow_panel_queue_ulps_work(mpd);
	}
	dsi_bus_unlock(dssdev);
_munlock_:
	mutex_unlock(&mpd->lock);
}

static struct omap_dss_driver minnow_panel_driver = {
	.probe		= minnow_panel_probe,
	.remove		= __exit_p(minnow_panel_remove),

	.enable		= minnow_panel_enable,
	.disable	= minnow_panel_disable,
#if defined(CONFIG_HAS_AMBIENTMODE)
	.suspend	= minnow_panel_suspend,
#endif

	.update		= minnow_panel_update,
	.sync		= minnow_panel_sync,

	.get_resolution	= minnow_panel_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.enable_te	= minnow_panel_enable_te,
	.get_te		= minnow_panel_get_te,

#ifdef	PANEL_DEBUG
	.run_test	= minnow_panel_run_test,
	.memory_read	= minnow_panel_memory_read,
#endif

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
