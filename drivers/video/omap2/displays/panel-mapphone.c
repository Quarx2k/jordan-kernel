#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>

#include <video/omapdss.h>
#include <plat/dma.h>
#include <linux/atomic.h>
#include <plat/clock.h>
#include <plat/gpio.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>

#include <asm/prom.h>

#include <video/panel.h>
#include <video/omap-panel-mapphone-dsi.h>
//#include "panel-mapphone-d2l.h"
#include "../dss/dss.h"

/* Define this Macro for factory board level test only */
/* #define DEBUG 1 */
/*
 * NOTE: DEBUG may be defined in dss.h already
 * (when CONFIG_OMAP2_DSS_DEBUG_SUPPORT is enabled).
 */

/* Display Control Hook */
extern u8 display_brightness(void);

static unsigned int panel_debug;
#define DBG(format, ...) \
	if (panel_debug) \
		printk(KERN_DEBUG "mapphone-panel: " format, ## __VA_ARGS__); \

#define EDISCO_CMD_SOFT_RESET		0x01
#define EDISCO_CMD_GET_POWER_MODE       0x0A
#define EDISCO_CMD_READ_DISPLAY_IMAGE_MODE	0x0D
#define EDISCO_CMD_ENTER_SLEEP_MODE	0x10
#define EDISCO_CMD_EXIT_SLEEP_MODE	0x11
#define EDISCO_CMD_SET_INVERSION_OFF	0x20
#define EDISCO_CMD_SET_INVERSION_ON	0x21
#define EDISCO_CMD_SET_DISPLAY_ON	0x29
#define EDISCO_CMD_SET_DISPLAY_OFF	0x28
#define EDISCO_CMD_SET_COLUMN_ADDRESS	0x2A
#define EDISCO_CMD_SET_PAGE_ADDRESS	0x2B
#define EDISCO_CMD_SET_TEAR_OFF		0x34
#define EDISCO_CMD_SET_TEAR_ON		0x35
#define EDISCO_CMD_SET_TEAR_SCANLINE	0x44
#define EDISCO_CMD_SET_CABC		0x55
#define EDISCO_CMD_READ_CABC		0x56
#define EDISCO_CMD_READ_DDB_START	0xA1
#define EDISCO_CMD_SET_MCS		0xB2
#define EDISCO_CMD_SET_DISPLAY_MODE     0xB3
#define EDISCO_CMD_SET_BCKLGHT_PWM	0xB4
#define EDISCO_CMD_DATA_LANE_CONFIG	0xB5
#define EDISCO_CMD_READ_DA		0xDA
#define EDISCO_CMD_READ_DB		0xDB
#define EDISCO_CMD_READ_DC		0xDC
#define EDISCO_CMD_RDDSDR		0x0F

#define EDISCO_CMD_DATA_LANE_ONE	0x0
#define EDISCO_CMD_DATA_LANE_TWO	0x01
#define EDISCO_CMD_SLEEP_MODE_OUT	0x10

#define EDISCO_CMD_MCS_ON		0x3
#define EDISCO_CMD_MCS_OFF		0x0

#define EDISCO_LONG_WRITE		0x29
#define EDISCO_SHORT_WRITE_1		0x23
#define EDISCO_SHORT_WRITE_0		0x13

#define DCS_CMD_RETRY_MAX 10

#define PANEL_OFF     0x0
#define PANEL_ON      0x1

#define CABC_MIN_VAL	0x01
#define CABC_MAX_VAL	0x04

/* Todo:  should NOT show vendor name, replace it with panel id later */
/* DDB Controller Supplier IDs used for run_test(1) */
#define CTL_SUPPLIER_ID_LEN	2
#define CTL_SUPPLIER_ID_AUO	0x0186
#define CTL_SUPPLIER_ID_AUO_43	0x0126	/* Shadow AUO panel reports 0x126 */
#define CTL_SUPPLIER_ID_TMD	0x0126
#define CTL_SUPPLIER_ID_SMD	0x010B

#define INVALID_VALUE	0xFFFF

#ifdef CONFIG_MACH_OMAP_MAPPHONE_DEFY
/**
 * RDDPM (Read Display Power Mode) - response to EDISCO_CMD_GET_POWER_MODE.
 * D7: Step up circuit Voltage Status (0 = off or fault, 1 = on/working)
 * D6: Idle Mode (0 = off, 1 = on)
 * D5: Partial Mode (0 = off, 1 = on)
 * D4: Wake/Sleep Mode (0 = "sleep in"/sleeping, 1 = "sleep out"/awake)
 * D3: Display Normal Mode (0 = off, 1 = on)
 * D2: Display Status (0 = off, 1 = on)
 * D1: Reserved - always 0
 * D0: Reserved - always 0
 */
#define REG_DPM_STEP_UP_CIRCUIT	(1 << 7)
#define REG_DPM_IDLE_MODE	(1 << 6)
#define REG_DPM_PARTIAL_MODE	(1 << 5)
#define REG_DPM_WAKE_MODE	(1 << 4)
#define REG_DPM_NORMAL_MODE	(1 << 3)
#define REG_DPM_DISPLAY_STATUS	(1 << 2)
#endif

/*
 *This must match with schema.xml section "device-id-value"
 *Name convention:
 *MOT_DISP_PROTOCOL_MODE_SIZE_RESOLUTION{_PANEL_MANUFACTURE_ORDER}
 *PROTOCOL: MIPI
 *MODE: CM for cmd mode,VM for video mode
 *SIZE: panel physical size
 *RESOLUTION: width * height in pixel
 *PANEL_MANUFACTURE_ORDER: for multiple panel manufacture only
 *value are 1,2,3...starting from default value 1
 */

#define MOT_DISP_MIPI_CM_480_854		0x000a0001
#define MOT_DISP_MIPI_CM_430_480_854		0x001a0000
#define MOT_DISP_MIPI_CM_370_480_854		0x001a0001
#define MOT_DISP_MIPI_CM_430_540_960_3		0x001a0002
#define MOT_DISP_MIPI_CM_450_720_1280		0x001a0003
#define MOT_DISP_MIPI_CM_400_540_960		0x00090004
#define MOT_DISP_MIPI_CM_430_540_960		0x00090005
#define MOT_DISP_MIPI_CM_430_540_960_AMOLED	0x00090006
#define MOT_DISP_MIPI_VM_248_320_240		0x00090002
#define MOT_DISP_MIPI_VM_280_320_240		0x00090003
#define MOT_DISP_MIPI_CM_310_320_480_1		0x001f0000
#define MOT_DISP_MIPI_CM_310_320_480_2		0x000a0003
#define MOT_DISP_LVDS_MIPI_VM_1007_1280_800	0x001a0004
/*TODO: ESD */
/*ESD spec require 10s, select 8s */
#define MAPPHONE_ESD_CHECK_PERIOD   msecs_to_jiffies(8000)
#define MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX 255

/* LVDS Panel EDID Manufacture ID Registers */
#define LVDS_MANUFACTURE_ID_ADDR1		0x08
#define LVDS_MANUFACTURE_ID_ADDR2		0x09

struct panel_regulator {
	struct regulator *regulator;
	const char *name;
	int min_uV;
	int max_uV;
};

static u8 dsi_vc_cmd;
static u8 dsi_vc_video;

static int column_address_offset;
static enum mapphone_panel_init_state panel_init_state =
			 MAPPHONE_PANEL_UNDETERMINE;
enum mapphone_panel_init_state get_panel_state(void)
{
	return panel_init_state;
}

static void free_regulators(struct panel_regulator *regulators, int n)
{
}


static int init_regulators(struct omap_dss_device *dssdev,
				struct panel_regulator *regulators, int n)
{
	return 0;
}

/**
 * struct panel_config - panel configuration
 * @name: panel name
 * @type: panel type
 * @timings: panel resolution
 * @sleep: various panel specific delays, passed to msleep() if non-zero
 * @reset_sequence: reset sequence timings, passed to udelay() if non-zero
 * @regulators: array of panel regulators
 * @num_regulators: number of regulators in the array
 */
struct panel_config {
	const char *name;
	int type;

	struct {
		unsigned int sleep_in;
		unsigned int sleep_out;
		unsigned int hw_reset;
		unsigned int enable_te;
	} sleep;

	struct {
		unsigned int high;
		unsigned int low;
	} reset_sequence;

	struct panel_regulator *regulators;
	int num_regulators;
};

enum {
	PANEL_MAPPHONE,
};

static struct panel_config panel_configs[] = {
	{
		.name           = "mapphone",
		.type           = PANEL_MAPPHONE,
		.sleep          = {
			.sleep_in       = 5,
			.sleep_out      = 5,
			.hw_reset       = 5,
			.enable_te      = 100, /* possible panel bug */
		},
		.reset_sequence = {
			.high           = 10,
			.low            = 10,
		},
	},
};

struct mapphone_data {
	struct mutex lock;

	struct backlight_device *bldev;

	struct omap_dss_device *dssdev;

	bool enabled;
	bool som_enabled;
	u8 rotate;
	bool mirror;

	bool te_enabled;
	bool acl_enabled;

	atomic_t do_update;
	struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
	} update_region;

	int channel;

	struct delayed_work te_timeout_work;
	struct workqueue_struct *te_wq;
	struct work_struct te_framedone_work;

	bool use_dsi_bl;

	bool cabc_broken;
	unsigned cabc_mode;

	bool intro_printed;
	/* cmd mode force update flag:
	 * if true, automatically call panel_update()
	 * within display driver, use for test purpose.
	 */
	bool force_update;
	struct workqueue_struct *esd_wq;
	struct delayed_work esd_work;

	struct panel_config *panel_config;
};

struct lvds_panel {
	struct i2c_client *client;
} *lvds;

static int lvds_panel_read(struct i2c_client *client, int reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (uint8_t)ret;
	return 0;
}

static int __devinit lvds_panel_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	lvds = kzalloc(sizeof(struct lvds_panel), GFP_KERNEL);
	if (lvds == NULL) {
		printk(KERN_ERR "failed to probe lvds panel\n");
		return -ENOMEM;
	}

	/* store i2c_client pointer on private data structure */
	lvds->client = client;

	/* store private data structure pointer on i2c_client structure */
	i2c_set_clientdata(client, lvds);

	return 0;
}

/* driver remove function */
static int __devexit lvds_panel_remove(struct i2c_client *client)
{
	struct lvds_panel *lvds = i2c_get_clientdata(client);

	/* remove client data */
	i2c_set_clientdata(client, NULL);

	/* free private data memory */
	kfree(lvds);

	return 0;
}

static const struct i2c_device_id lvds_panel_idtable[] = {
	{"lvds_panel", 0},
	{},
};

static struct i2c_driver lvds_panel_driver = {
	.probe = lvds_panel_probe,
	.remove = __exit_p(lvds_panel_remove),
	.id_table = lvds_panel_idtable,
	.driver = {
		   .name = "lvds_panel_drv"},
};

static inline struct mapphone_dsi_panel_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct mapphone_dsi_panel_data *) dssdev->data;
}

static void mapphone_panel_power_off(struct omap_dss_device *dssdev,
				bool secret);
static int mapphone_panel_power_on(struct omap_dss_device *dssdev);

static void mapphone_esd_work(struct work_struct *work)
{
	struct mapphone_data *mp_data = container_of(work, struct mapphone_data,
			esd_work.work);
	struct omap_dss_device *dssdev = mp_data->dssdev;
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	u8 power_mode;
	u8 expected_mode;
	int r;
	struct omap_dss_driver *dssdrv = dssdev->driver;
	u16 w, h;

	DBG("%s()\n", __func__);

	mutex_lock(&mp_data->lock);

	if (!mp_data->enabled) {
		DBG("%s: ignoring because display is disabled\n", __func__);
		mutex_unlock(&mp_data->lock);
		return;
	}

	dsi_bus_lock(dssdev);
	dsi_from_dss_runtime_get(dssdev);

	r = dsi_vc_dcs_read(dssdev, dsi_vc_cmd, EDISCO_CMD_GET_POWER_MODE,
			&power_mode, 1);
	if (r != 1) {
		dev_err(&dssdev->dev, "Failed to get power mode, r = %d\n", r);
		goto err;
	}

#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	if (atomic_read(&panel_data->state) == PANEL_ON)
		/* REG_DPM_STEP_UP_CIRCUIT | REG_DPM_DISPLAY_STATUS | REG_DPM_NORMAL_MODE | REG_DPM_WAKE_MODE */
		expected_mode = 0x9c;
	else
		/* REG_DPM_STEP_UP_CIRCUIT | REG_DPM_NORMAL_MODE | REG_DPM_WAKE_MODE */
		expected_mode = 0x98;

	DBG("ESD Check - read mode = 0x%02x, expected = 0x%02x\n", power_mode,
		expected_mode);

	if (power_mode != expected_mode) {
		dev_err(&dssdev->dev,
			"Power mode in incorrect state, "
			"mode = 0x%02x, expected = 0x%02x\n",
			power_mode, expected_mode);
		goto err;
	}
#else
	/* Some Defy/Milestone displays don't have REG_DPM_STEP_UP_CIRCUIT set. */
	if (atomic_read(&panel_data->state) == PANEL_ON)
		/* 0x1c */
		expected_mode = REG_DPM_DISPLAY_STATUS | REG_DPM_NORMAL_MODE | REG_DPM_WAKE_MODE;
	else
		/* 0x18 */
		expected_mode = REG_DPM_NORMAL_MODE | REG_DPM_WAKE_MODE;

	DBG("ESD Check - read mode = 0x%02x, expected to match = 0x%02x\n",
	        power_mode, expected_mode);

	if (!(power_mode & expected_mode)) {
		dev_err(&dssdev->dev,
			"Power mode in incorrect state, "
			"mode = 0x%02x, expected to match = 0x%02x\n",
			power_mode, expected_mode);
		goto err;
	}
#endif

	dsi_from_dss_runtime_put(dssdev);
	dsi_bus_unlock(dssdev);

	queue_delayed_work(mp_data->esd_wq, &mp_data->esd_work,
			 MAPPHONE_ESD_CHECK_PERIOD);

	mutex_unlock(&mp_data->lock);
	return;
err:
	dev_err(&dssdev->dev, "ESD: performing LCD reset\n");
	printk(KERN_INFO"ESD: mapphone_panel_power_off.\n");
	mapphone_panel_power_off(dssdev, false);
	printk(KERN_INFO"ESD: mdelay 20ms.\n");
	mdelay(20);
	printk(KERN_INFO"ESD: mapphone_panel_power_on.\n");
	r = mapphone_panel_power_on(dssdev);
	/*
	 * dssdev->state and panel_data->state was set to DISABLED/OFF in
	 * mapphone_panel_power_off(), after power_on(), need to set
	 * dssdev->state and panel->state to ACTIVE/ON, otherwise it causes
	 * panel suspend/resume/update failure because of dis-ordered state.
	 */
	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	}

	dsi_from_dss_runtime_put(dssdev);
	dsi_bus_unlock(dssdev);

	queue_delayed_work(mp_data->esd_wq, &mp_data->esd_work,
			 MAPPHONE_ESD_CHECK_PERIOD);

	mutex_unlock(&mp_data->lock);

	/* call update after recovery */
	dssdrv->get_resolution(dssdev, &w, &h);
	dssdrv->update(dssdev, 0, 0, w, h);
}

static int mapphone_panel_display_on(struct omap_dss_device *dssdev);

static void mapphone_framedone_cb(int err, void *data)
{
	struct omap_dss_device *dssdev = data;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	DBG("%s()\n", __func__);

	/* Turn on display when framedone */
//	mapphone_panel_display_on(dssdev);

#ifdef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	dsi_from_dss_runtime_put(dssdev);
	dsi_bus_unlock(dssdev);
#endif

	if (mp_data->force_update)
		queue_work(mp_data->te_wq, &mp_data->te_framedone_work);
}

static int mapphone_set_update_window(struct omap_dss_device *dssdev,
					u16 x, u16 y, u16 w, u16 h)
{
	u8 buf[5];
	int ret;

	u16 x1 = x + column_address_offset;
	u16 x2 = x + column_address_offset + w - 1;
	u16 y1 = y;
	u16 y2 = y + h - 1;

	DBG("Setting update window %d %d %d %d\n", x1, x2, y1, y2);

	/*
	 * set page, column address cmd using dsi_vc_dcs_write()
	 * with BTA sync. without BTA sync, DSS_L3_ICLK is not gated
	 * when framedone CM_DSS_CLKSTCTRL = 0x00000f03, which result
	 * in DSS does not go into idle CM_DSS_DSS_CLKCTRL= 0x00050702
	 * work around is: send cmd with BTA sync. with it
	 * when framedone:CM_DSS_CLKSTCTRL = 0x00000e03
	 * CM_DSS_DSS_CLKCTRL= 0x00060702
	 */
	buf[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	buf[1] = (x1 >> 8) & 0xff;
	buf[2] = (x1 >> 0) & 0xff;
	buf[3] = (x2 >> 8) & 0xff;
	buf[4] = (x2 >> 0) & 0xff;
	ret = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 5);
	if (ret)
		goto err;

	buf[0] = EDISCO_CMD_SET_PAGE_ADDRESS;
	buf[1] = (y1 >> 8) & 0xff;
	buf[2] = (y1 >> 0) & 0xff;
	buf[3] = (y2 >> 8) & 0xff;
	buf[4] = (y2 >> 0) & 0xff;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 5);
	if (ret)
		goto err;

	return 0;
err:
	return ret;

}
/*TODO: remove this later*/
/*
struct seq_file *s;
extern void dsi1_dump_regs(struct seq_file *s);
extern void dispc_dump_regs(struct seq_file *s);
*/
static int mapphone_panel_update(struct omap_dss_device *dssdev,
					u16 x, u16 y, u16 w, u16 h)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r, rr = 0;

	WARN_ON(dssdev != mp_data->dssdev);

	DBG("update %d, %d, %d x %d\n", x, y, w, h);

	mutex_lock(&mp_data->lock);
	dsi_bus_lock(dssdev);
	dsi_from_dss_runtime_get(dssdev);

	if (!mp_data->enabled) {
		DBG("Got update for disabled panel\n");
		r = 0;
		goto err;
	}

	DBG("starting to prepare update...\n");

	r = omap_dsi_prepare_update(dssdev, &x, &y, &w, &h, true);
	if (r)
		goto err;

	if (dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_CMD_MODE) {
		/* Only command mode can do partial update */
		rr = mapphone_set_update_window(dssdev, x, y, w, h);
		if (rr)
			dev_err(&dssdev->dev,
				"mapphone_set_update_window failed:%d\n", rr);
	}

	if (mp_data->te_enabled && panel_data->use_ext_te &&
		(dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_CMD_MODE)) {
		mp_data->update_region.x = x;
		mp_data->update_region.y = y;
		mp_data->update_region.w = w;
		mp_data->update_region.h = h;
		barrier();
		schedule_delayed_work(&mp_data->te_timeout_work,
					msecs_to_jiffies(250));
		atomic_set(&mp_data->do_update, 1);
	} else {
		/* We use VC(1) for VideoPort Data and VC(0) for L4 data */
		if (cpu_is_omap44xx())
			r = omap_dsi_update(dssdev, dsi_vc_video, x, y, w, h,
					mapphone_framedone_cb, dssdev);
		else
			r = omap_dsi_update(dssdev, dsi_vc_cmd, x, y, w, h,
					mapphone_framedone_cb, dssdev);
		if (r)
			goto err;

		DBG("Update successfully triggered\n");
	}

	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&mp_data->lock);

	return rr;
err:
	dsi_from_dss_runtime_put(dssdev);
	dsi_bus_unlock(dssdev);
	mutex_unlock(&mp_data->lock);
	return r;
}
static int mapphone_panel_sync(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	DBG("%s()\n", __func__);

	mutex_lock(&mp_data->lock);
	dsi_bus_lock(dssdev);
	dsi_bus_unlock(dssdev);
	mutex_unlock(&mp_data->lock);

	return 0;
}

static void mapphone_panel_get_timings(struct omap_dss_device *dssdev,
				struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int mapphone_panel_check_timings(struct omap_dss_device *dssdev,
				struct omap_video_timings *timings)
{
	if (timings->x_res != dssdev->panel.timings.x_res ||
	    timings->y_res != dssdev->panel.timings.y_res)
		return -EINVAL;

	return 0;
}

static u8 mapphone_panel_get_rotate(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&mp_data->lock);
	r = mp_data->rotate;
	mutex_unlock(&mp_data->lock);

	return r;
}

static void mapphone_panel_get_resolution(struct omap_dss_device *dssdev,
						u16 *xres, u16 *yres)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	if (mp_data->rotate == 0 || mp_data->rotate == 2) {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	} else {
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}
}

static void mapphone_panel_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	dssdev->panel.timings.x_res = timings->x_res;
	dssdev->panel.timings.y_res = timings->y_res;
	dssdev->panel.timings.pixel_clock = timings->pixel_clock;
	dssdev->panel.timings.hsw = timings->hsw;
	dssdev->panel.timings.hfp = timings->hfp;
	dssdev->panel.timings.hbp = timings->hbp;
	dssdev->panel.timings.vsw = timings->vsw;
	dssdev->panel.timings.vfp = timings->vfp;
	dssdev->panel.timings.vbp = timings->vbp;
}

static int mapphone_panel_get_te(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&mp_data->lock);
	r = mp_data->te_enabled;
	mutex_unlock(&mp_data->lock);

	return r;
}

static void te_work_callback(struct work_struct *work)
{
	struct mapphone_data *mp_data = container_of(work, struct mapphone_data,
						te_framedone_work);
	struct omap_dss_device *dssdev = mp_data->dssdev;

	u16 x_res = dssdev->panel.timings.x_res;
	u16 y_res = dssdev->panel.timings.y_res;

	DBG("%s()\n", __func__);

	mapphone_panel_update(dssdev, 0, 0, x_res, y_res);
}

static int mapphone_panel_memory_read(struct omap_dss_device *dssdev,
						void *buf, size_t size,
						u16 x, u16 y, u16 w, u16 h)
{
	int r;
	int first = 1;
	int plen;
	unsigned buf_used = 0;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	WARN_ON(dssdev != mp_data->dssdev);

	DBG("%s()\n", __func__);

	if (size < w * h * 3)
		return -ENOMEM;

	mutex_lock(&mp_data->lock);

	if (!mp_data->enabled) {
		r = -ENODEV;
		goto err1;
	}

	size = min(w * h * 3,
	dssdev->panel.timings.x_res *
	dssdev->panel.timings.y_res * 3);

	dsi_bus_lock(dssdev);

	/* plen 1 or 2 goes into short packet. until checksum error is fixed,
	* use short packets. plen 32 works, but bigger packets seem to cause
	* an error. */
	if (size % 2)
		plen = 1;
	else
		plen = 2;

	mapphone_set_update_window(dssdev, x, y, w, h);

	r = dsi_vc_set_max_rx_packet_size(dssdev, dsi_vc_cmd, plen);
	if (r) {
		dev_err(&dssdev->dev, "%s: Setting dsi_vc_set_max_rx_packet_size"
				" to %d failed.\n", __func__, plen);
		goto err2;
	}

	while (buf_used < size) {
		u8 dcs_cmd = first ? 0x2e : 0x3e;
		first = 0;

		r = dsi_vc_dcs_read(dssdev, dsi_vc_cmd, dcs_cmd,
		buf + buf_used, size - buf_used);

		if (r < 0) {
			dev_err(&dssdev->dev, "read error\n");
			goto err3;
		}

		buf_used += r;

		if (r < plen) {
			dev_err(&dssdev->dev, "short read\n");
			break;
		}

		if (signal_pending(current)) {
			dev_err(&dssdev->dev, "signal pending, "
						"aborting memory read\n");
			r = -ERESTARTSYS;
			goto err3;
		}
	}

	r = buf_used;

err3:
	dsi_vc_set_max_rx_packet_size(dssdev, dsi_vc_cmd, 1);
err2:
	dsi_bus_unlock(dssdev);
err1:
	mutex_unlock(&mp_data->lock);
	return r;
}

static void mapphone_panel_disable_local(struct omap_dss_device *dssdev);

static int dsi_mipi_vm_panel_on(struct omap_dss_device *dssdev)
{
	u8 data = EDISCO_CMD_SET_DISPLAY_ON;
	int ret;
	/*TODO: video mode*/
	/*dsi_disable_vid_vc_enable_cmd_vc(lcd_ix);*/
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, &data, 1);
	/*dsi_disable_cmd_vc_enable_vid_vc(lcd_ix);*/

	return ret;
}

static int dsi_mipi_cm_panel_on(struct omap_dss_device *dssdev)
{
	u8 data = EDISCO_CMD_SET_DISPLAY_ON;

	/*
	 *Issue:
	 *See corrupted image on the end of last line of display panel
	 *sometimes when resume the phone. It happens more often
	 *when OMAP4 run at high freq such as 600M/800M/1G, so far locking
	 *arm at 300M does not see the issue.
	 *
	 *It is observed on omap4-based products built with SMD pentile panel
	 *controller. The corruption happens within the controller's
	 *framebuffer after receive the first update and 29h/disp on cmd,
	 *the interval between EOT and 29h is about 16us, it probably isn't
	 *long enough for Pentile processing to get the data into memory.
	 *The following update have no problem.
	 *
	 *Adding delay before sending 29h/disp on cmd can make the issue
	 *go away based on test, it is test value*2.
	 */
	switch (dssdev->panel.panel_id) {
	case MOT_DISP_MIPI_CM_400_540_960:
	case MOT_DISP_MIPI_CM_430_540_960:
	case MOT_DISP_MIPI_CM_430_540_960_AMOLED:
		udelay(24);
		break;
	}

	/* Called in interrupt context, send cmd without sync */
	return dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, &data, 1);
}


static int mapphone_panel_display_on(struct omap_dss_device *dssdev)
{
	int ret = 0;

	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if (atomic_cmpxchg(&panel_data->state, PANEL_OFF, PANEL_ON) ==
						PANEL_OFF) {
		switch (dssdev->panel.panel_id) {
		case MOT_DISP_MIPI_VM_248_320_240:
		case MOT_DISP_MIPI_VM_280_320_240:
			ret = dsi_mipi_vm_panel_on(dssdev);
			break;
		case MOT_DISP_MIPI_CM_480_854:
		case MOT_DISP_MIPI_CM_370_480_854:
		case MOT_DISP_MIPI_CM_430_480_854:
		case MOT_DISP_MIPI_CM_400_540_960:
		case MOT_DISP_MIPI_CM_430_540_960:
		case MOT_DISP_MIPI_CM_430_540_960_3:
		case MOT_DISP_MIPI_CM_310_320_480_1:
		case MOT_DISP_MIPI_CM_430_540_960_AMOLED:
		case MOT_DISP_MIPI_CM_450_720_1280:
			ret = dsi_mipi_cm_panel_on(dssdev);
			break;
		default:
			printk(KERN_ERR"unsupport panel =0x%lx\n",
			dssdev->panel.panel_id);
			ret = -EINVAL;
		}

		if (ret == 0)
			printk(KERN_INFO"Panel is turned on\n");
	}

	return ret;
}

static u16 read_panel_manufacture(struct omap_dss_device *dssdev)
{
	static u16 id = INVALID_VALUE;
	u8 data = 0;

	if (id != INVALID_VALUE)
		goto end;

	if (dsi_vc_dcs_read(dssdev, dsi_vc_cmd,
			EDISCO_CMD_READ_DA, &data, 1) != 1)
		printk(KERN_ERR"Mapphone panel: failed to read panel manufacturer ID\n");
	else {
		id = data >> 6;
		printk(KERN_INFO"Mapphone panel: panel manufacturer ID (DAh)=0x%x\n",
			 id);
	}
end:
	return id;
}

static u16 read_panel_controller_version(struct omap_dss_device *dssdev)
{
	static u16 id = INVALID_VALUE;
	u8 data = 0;

	if (id != INVALID_VALUE)
		goto end;

	if (dsi_vc_dcs_read(dssdev, dsi_vc_cmd,
			EDISCO_CMD_READ_DB, &data, 1) != 1)
		printk(KERN_ERR"Mapphone panel: failed to read controller version\n");
	else {
		id = data;
		printk(KERN_INFO"Mapphone panel: controller version (DBh)=0x%x\n",
			 id);
	}
end:
	return id;
}

static u16 read_panel_controller_driver_version(struct omap_dss_device *dssdev)
{
	static u16 id = INVALID_VALUE;
	u8 data = 0;

	if (id != INVALID_VALUE)
		goto end;

	if (dsi_vc_dcs_read(dssdev, dsi_vc_cmd,
			EDISCO_CMD_READ_DC, &data, 1) != 1)
		printk(KERN_ERR"Mapphone panel: failed to read controller driver ID\n");
	else {
		id = data;
		printk(KERN_INFO"Mapphone panel: controller driver ID(DCh)=0x%x\n",
			 id);
	}
end:
	return id;
}

static u16 read_supplier_id(struct omap_dss_device *dssdev)
{
	static u16 id = INVALID_VALUE;
	int r;
	u8 data[CTL_SUPPLIER_ID_LEN];

	if (id != INVALID_VALUE)
		goto end;

	r = dsi_vc_set_max_rx_packet_size(dssdev, dsi_vc_cmd, CTL_SUPPLIER_ID_LEN);
	if (r) {
		printk(KERN_ERR "Mapphone panel: failed to update dsi_vc_set_max_rx_packet_size: %d\n",
					r);
		goto end;
	}

#ifdef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	/* Wait a bit because dsi_vc_set_max_rx_packet_size is async. */
	msleep(20);
#endif

	r = dsi_vc_dcs_read(dssdev, dsi_vc_cmd,
			    EDISCO_CMD_READ_DDB_START, data, CTL_SUPPLIER_ID_LEN);
	if (r == CTL_SUPPLIER_ID_LEN) {
		id = (data[0] << 8) | data[1];
		printk(KERN_INFO "Mapphone panel: controller supplier id(A1h)=0x%x\n",
			 id);
	} else
		printk(KERN_ERR "Mapphone panel: failed to read controller supplier ID: %d\n",
					r);

	dsi_vc_set_max_rx_packet_size(dssdev, dsi_vc_cmd, 1);
end:
	return id;
}

static u16 read_lvds_panel_manufacture(void)
{
	static u16 id = INVALID_VALUE;
	u8 val_l, val_h;
	int ret = 0;

	if (id != INVALID_VALUE)
		goto end;

	if (lvds == NULL) {
		printk(KERN_ERR "lvds panel i2c is not registered\n");
		goto end;
	}

	ret = lvds_panel_read(lvds->client, LVDS_MANUFACTURE_ID_ADDR1, &val_h);
	if (ret < 0) {
		printk(KERN_ERR "failed to read lvds panel manufactur ID 1\n");
		goto end;
	}

	ret = lvds_panel_read(lvds->client, LVDS_MANUFACTURE_ID_ADDR2, &val_l);
	if (ret < 0) {
		printk(KERN_ERR "failed to read lvds panel manufactur ID 2\n");
		goto end;
	}

	id = (val_h << 8) | val_l;
	printk(KERN_INFO "lvds panel manufacturer ID = 0x%x\n", id);

end:
	return id;
}

static ssize_t mapphone_panel_supplier_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	static u16 supplier_id = INVALID_VALUE;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	if (supplier_id == INVALID_VALUE) {

		if (!mp_data->enabled)
			goto end;

		dsi_bus_lock(dssdev);
		dsi_from_dss_runtime_get(dssdev);

		switch (dssdev->panel.panel_id) {
		case MOT_DISP_MIPI_CM_480_854:
		case MOT_DISP_MIPI_CM_370_480_854:
			supplier_id = read_supplier_id(dssdev);
			break;
		case MOT_DISP_MIPI_CM_400_540_960:
		case MOT_DISP_MIPI_CM_430_540_960:
		case MOT_DISP_MIPI_CM_430_540_960_3:
		case MOT_DISP_MIPI_CM_430_540_960_AMOLED:
		case MOT_DISP_MIPI_CM_450_720_1280:
			supplier_id = read_panel_manufacture(dssdev);
			break;
		case MOT_DISP_LVDS_MIPI_VM_1007_1280_800:
			supplier_id = read_lvds_panel_manufacture();
			break;
		default:
			printk(KERN_ERR "Do not support supplier_id of the panel\n");
		}

		dsi_from_dss_runtime_put(dssdev);
		dsi_bus_unlock(dssdev);
	}
end:
	return sprintf(buf, "%d\n", supplier_id);
}

static DEVICE_ATTR(supplier_id, S_IRUGO,
			mapphone_panel_supplier_id_show, NULL);

static bool mapphone_cabc_support(struct omap_dss_device *dssdev)
{
	bool ret = true;

	switch (dssdev->panel.panel_id) {
	case MOT_DISP_MIPI_CM_430_540_960_AMOLED:
		ret = false;
		break;
	default:
		ret = true;
	}

	return ret;
}

static ssize_t panel_cabc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	u8 data = 0xff;

	mutex_lock(&mp_data->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		printk(KERN_ERR "mapphone panel is NOT active\n");
		data = -EAGAIN;
		goto err;
	}

	if (mapphone_cabc_support(dssdev) == false) {
		printk(KERN_ERR "mapphone panel doesn't support this op\n");
		data = -EPERM;
		goto err;
	}

	dsi_from_dss_runtime_get(dssdev);
	dsi_bus_lock(dssdev);

	if ((dsi_vc_dcs_read(dssdev, dsi_vc_cmd,
				EDISCO_CMD_READ_CABC, &data, 1) == 1) &&
			((data == CABC_MIN_VAL) || (data == CABC_MAX_VAL))) {
		printk(KERN_INFO "Mapphone panel: CABC_mode=0x%x\n", data);
	} else
		printk(KERN_ERR "Mapphone panel: fail to read CABC. "
				"data=0x%x\n", data);

	dsi_bus_unlock(dssdev);
	dsi_from_dss_runtime_put(dssdev);

err:
	mutex_unlock(&mp_data->lock);
	return sprintf(buf, "%d\n", data);
}

static int mapphone_panel_lp_cmd_wrt_sync(struct omap_dss_device *dssdev,
					bool dcs_cmd, int write_dt,
					u8 *write_data, int write_len,
					int read_cmd, int read_len,
					int chk_val, int chk_mask);

static ssize_t panel_cabc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	unsigned long cabc_val, r;
	u8 data[2];
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&mp_data->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		printk(KERN_ERR "mapphone panel is NOT active\n");
		r = -EAGAIN;
		goto err;
	}

	if (mapphone_cabc_support(dssdev) == false) {
		printk(KERN_ERR "mapphone panel doesn't support this op\n");
		r = -EPERM;
		goto err;
	}

	r = strict_strtoul(buf, 0, &cabc_val);
	if ((r) || ((cabc_val != CABC_MIN_VAL) && (cabc_val != CABC_MAX_VAL))) {
		printk(KERN_ERR "Invalid CABC=%lu\n", cabc_val);
		r = -EINVAL;
		goto err;
	}

	dsi_from_dss_runtime_get(dssdev);
	dsi_bus_lock(dssdev);

	data[0] = EDISCO_CMD_SET_CABC;
	data[1] = (u8)cabc_val;
	r = mapphone_panel_lp_cmd_wrt_sync(dssdev,
					true, 0x00,
					data, 2,
					EDISCO_CMD_READ_CABC, 1,
					(u8)cabc_val, 0x7);
	if (r) {
		printk(KERN_ERR "failed to set EDISCO_CMD_SET_CABC\n");
		r = -EIO;
	}

	dsi_bus_unlock(dssdev);
	dsi_from_dss_runtime_put(dssdev);
err:
	mutex_unlock(&mp_data->lock);
	return r ? r : count;
}

static DEVICE_ATTR(cabc_mode, S_IRUGO | S_IWGRP,
				panel_cabc_show, panel_cabc_store);

static ssize_t panel_secret_off_show(struct device *dev,
			struct device_attribute *attr, char *buf)

{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int state = 0;

	if (!panel_data->ftr_support.som) {
		printk(KERN_ERR "Secret off not supported\n");
		state = -EINVAL;
		goto end;
	}

	mutex_lock(&mp_data->lock);

	if (mp_data->som_enabled)
		state = 1;

	mutex_unlock(&mp_data->lock);

end:
	return sprintf(buf, "%d\n", state);
}

static void mapphone_panel_stop(struct omap_dss_device *dssdev, bool secret);
static int mapphone_panel_start(struct omap_dss_device *dssdev);

static ssize_t panel_secret_off_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	bool change_state = false;
	bool enable_so = false;
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r = 0;
	unsigned long val = -1;

	if (!panel_data->ftr_support.som) {
		printk(KERN_ERR "Secret off not supported\n");
		r = -EINVAL;
		goto err;
	}

	r = strict_strtoul(buf, 0, &val);
	if ((r) || ((val != 0) && (val != 1))) {
		printk(KERN_ERR "Invalid secret off value = %lu\n", val);
		goto err;
	}

	mutex_lock(&mp_data->lock);
	if ((val == 1) && (mp_data->enabled) && (!mp_data->som_enabled)) {
		change_state = true;
		enable_so = true;
	} else if ((val == 0) && (!mp_data->enabled) &&
		(mp_data->som_enabled))
		change_state = true;

	if (change_state) {
		if (enable_so) {
			/* Enable secret off */
			DBG("Secret off start\n");
			mapphone_panel_stop(dssdev, true);
			DBG("Secret off stop\n");
		} else {
			/* Disable secret off */
			DBG("Secret resume start\n");
			mapphone_panel_start(dssdev);
			DBG("Secret resume stop\n");
		}
	}

	mutex_unlock(&mp_data->lock);

err:
	return r ? r : count;
}

static DEVICE_ATTR(secret_off, S_IRUGO | S_IWGRP,
				panel_secret_off_show, panel_secret_off_store);

static bool mapphone_acl_support(struct omap_dss_device *dssdev)
{
	bool ret = false;

	switch (dssdev->panel.panel_id) {
	case MOT_DISP_MIPI_CM_430_540_960_AMOLED:
		ret = true;
		break;
	default:
		ret = false;
	}

	return ret;
}

static ssize_t panel_acl_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	u8 data = 0xff;

	if (mapphone_acl_support(dssdev) == false) {
		printk(KERN_ERR "mapphone panel doesn't support ACL\n");
		data = -EPERM;
		goto err;
	}

	mutex_lock(&mp_data->lock);
	if (mp_data->acl_enabled)
		data = 1;
	else
		data = 0;

	mutex_unlock(&mp_data->lock);

err:
	return sprintf(buf, "%d\n", data);
}

static int mapphone_panel_acl_enable_locked(bool enable,
	 struct mapphone_data *mp_data, struct omap_dss_device *dssdev);

static ssize_t panel_acl_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct omap_dss_device *dssdev = to_dss_device(dev);
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	unsigned long acl_val;
	unsigned long r = 0;

	mutex_lock(&mp_data->lock);

	if (mapphone_acl_support(dssdev) == false) {
		printk(KERN_ERR "mapphone panel doesn't support this ACL\n");
		r = -EPERM;
		goto end;
	}

	r = strict_strtoul(buf, 0, &acl_val);
	if ((r) || ((acl_val != 0) && (acl_val != 1))) {
		printk(KERN_ERR "Invalid ACL value = %lu\n", acl_val);
		r = -EINVAL;
		goto end;
	}

	/* If display is disabled, just save the desired ACL state */
	if (!mp_data->enabled) {
		mp_data->acl_enabled = acl_val;
		goto end;
	}

	if (mp_data->acl_enabled != acl_val) {
		dsi_from_dss_runtime_get(dssdev);
		dsi_bus_lock(dssdev);

		r = mapphone_panel_acl_enable_locked(acl_val, mp_data, dssdev);
		mp_data->acl_enabled = acl_val;

		dsi_bus_unlock(dssdev);
		dsi_from_dss_runtime_put(dssdev);
	}
end:
	mutex_unlock(&mp_data->lock);
	return r ? r : count;
}

static DEVICE_ATTR(acl_mode, S_IRUGO | S_IWGRP,
				panel_acl_show, panel_acl_store);


static int mapphone_panel_acl_enable_locked(bool enable,
	 struct mapphone_data *mp_data, struct omap_dss_device *dssdev)
{
	u8 buf[2] = {0xc0, 0x00};
	int r;

	DBG("Setting ACL, enable = %d\n", enable);

	if (enable)
		buf[1] = 0x01;

	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);

	return r;
}

static void mapphone_hw_reset(struct omap_dss_device *dssdev)
{
}

static void mapphone_panel_print_config(struct omap_dss_device *dssdev)
{
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	struct mapphone_dsi_panel_pwr_supply *supply = NULL;
	int i;

	printk(KERN_INFO "DSI: dsi_vc_cmd = %d, dsi_vc_video = %d\n",
		dsi_vc_cmd, dsi_vc_video);

	printk(KERN_INFO "DT: x_res=%d y_res=%d phy_w_mm=%d phy_h_mm=%d\n",
		dssdev->panel.timings.x_res, dssdev->panel.timings.y_res,
		dssdev->panel.width_in_mm, dssdev->panel.height_in_mm);

	printk(KERN_INFO "DT: hfp=%d hsw=%d hbp=%d vfp=%d vsw=%d vbp=%d\n",
		dssdev->panel.timings.hfp, dssdev->panel.timings.hsw,
		dssdev->panel.timings.hbp, dssdev->panel.timings.vfp,
		dssdev->panel.timings.vsw, dssdev->panel.timings.vbp);

	printk(KERN_INFO "DT: clk_lane=%d clk_pos=%d d1_lane=%d d1_pos=%d "
				"d2_lane=%d d2_pos=%d\n",
		dssdev->phy.dsi.clk_lane, dssdev->phy.dsi.clk_pol,
		dssdev->phy.dsi.data1_lane, dssdev->phy.dsi.data1_pol,
		dssdev->phy.dsi.data2_lane, dssdev->phy.dsi.data2_pol);

	printk(KERN_INFO "DT: d3_lane=%d d3_pos=%d d4_lane=%d d4_pos=%d\n",
		dssdev->phy.dsi.data3_lane, dssdev->phy.dsi.data3_pol,
		dssdev->phy.dsi.data4_lane, dssdev->phy.dsi.data4_pol);

	printk(KERN_INFO "DT: panel_id=0x%lx type= %d\n",
			dssdev->panel.panel_id, dssdev->phy.dsi.type);

	printk(KERN_INFO "DT: regn=%d regm=%d regm3=%d regm4=%d"
			" lp_clk_div=%d lck_div=%d pck_div=%d\n",
		dssdev->clocks.dsi.regn, dssdev->clocks.dsi.regm,
		dssdev->clocks.dsi.regm_dispc, dssdev->clocks.dsi.regm_dsi,
		dssdev->clocks.dsi.lp_clk_div,
		dssdev->clocks.dispc.channel.lck_div,
		dssdev->clocks.dispc.channel.pck_div);

	printk(KERN_INFO"DT: te_support=%d, te_type=%d\n",
		panel_data->te_support,  panel_data->te_type);

	printk(KERN_INFO"DT: number of power supplies=%d\n",
		panel_data->num_pwr_supply);
	for (i = 0; i < panel_data->num_pwr_supply; i++) {
		supply = &(panel_data->disp_vol_supply[i]);
		printk(KERN_INFO"DT: Supply #%d, reg name=%s, gpio=%d, "
			"gpio_en_val=%d\n", i, supply->name, supply->en_gpio,
			supply->en_gpio_value);
	}
}

static int dsi_mipi_cm_430_540_960_amoled_bl_probe(
	struct omap_dss_device *dssdev,	struct mapphone_data *mp_data);

static void set_vc_channels(struct omap_dss_device *dssdev);
static int mapphone_panel_probe(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data;
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	struct panel_config *panel_config = NULL;
	int r, i;

	DBG("probe\n");

	if (!panel_data || !panel_data->name) {
		r = -EINVAL;
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(panel_configs); i++) {
		if (strcmp(panel_data->name, panel_configs[i].name) == 0) {
			panel_config = &panel_configs[i];
			break;
		}
	}

	if (!panel_config) {
		r = -EINVAL;
		goto err;
	}

	mp_data = kmalloc(sizeof(struct mapphone_data), GFP_KERNEL);
	if (!mp_data) {
		r = -ENOMEM;
		goto err;
	}

	memset(mp_data, 0, sizeof(struct mapphone_data));

	dssdev->panel.config = OMAP_DSS_LCD_TFT;

	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	mp_data->dssdev = dssdev;
	mp_data->panel_config = panel_config;

	if (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_430_540_960_AMOLED) {
		r = dsi_mipi_cm_430_540_960_amoled_bl_probe(dssdev, mp_data);
		if (r)
			goto err_reg;
	}

	mutex_init(&mp_data->lock);

	atomic_set(&mp_data->do_update, 0);

	r = init_regulators(dssdev, panel_config->regulators,
				panel_config->num_regulators);
	if (r)
		goto err_reg;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err_wq;
	}

	if (mapphone_acl_support(dssdev))
		mp_data->acl_enabled = true;

	mp_data->esd_wq = create_singlethread_workqueue("mapphone_esd");
	if (mp_data->esd_wq == NULL) {
		dev_err(&dssdev->dev, "can't create ESD workqueue\n");
		r = -ENOMEM;
		goto err_wq;
	}
	INIT_DELAYED_WORK_DEFERRABLE(&mp_data->esd_work, mapphone_esd_work);

	dev_set_drvdata(&dssdev->dev, mp_data);
	/* Set it to true to enable force update in cmd mode for test */
	mp_data->force_update = false;

	if (mp_data->force_update)
		if (dssdev->channel == OMAP_DSS_CHANNEL_LCD) {
			mp_data->te_wq =
			create_singlethread_workqueue("mapphone_panel wq");
			INIT_WORK(&mp_data->te_framedone_work,
						te_work_callback);
		}
	/* Disable secret off mode support for ICS, it is not needed */
	panel_data->ftr_support.som = false;

	r = device_create_file(&dssdev->dev, &dev_attr_supplier_id);
	if (r < 0) {
		dev_err(&dssdev->dev, "controller supplier_id create"
			" failed:%d\n", r);
		goto err_wq;
	}

	r = device_create_file(&dssdev->dev, &dev_attr_cabc_mode);
	if (r < 0) {
		dev_err(&dssdev->dev, "Display CABC create"
			" failed:%d\n", r);
		goto removeattr_id;
	}

	r = device_create_file(&dssdev->dev, &dev_attr_secret_off);
	if (r < 0) {
		dev_err(&dssdev->dev, "Display secret off create"
			" failed:%d\n", r);
		goto removeattr_cabc;
	}

	r = device_create_file(&dssdev->dev, &dev_attr_acl_mode);
	if (r < 0) {
		dev_err(&dssdev->dev, "Display ACL create"
			" failed:%d\n", r);
		goto removeattr_secret_off;
	}

	set_vc_channels(dssdev);
	return 0;

removeattr_secret_off:
	device_remove_file(&dssdev->dev, &dev_attr_secret_off);
removeattr_cabc:
	device_remove_file(&dssdev->dev, &dev_attr_cabc_mode);
removeattr_id:
	device_remove_file(&dssdev->dev, &dev_attr_supplier_id);
err_wq:
	free_regulators(panel_config->regulators, panel_config->num_regulators);
err_reg:
	kfree(mp_data);
err:
	return r;
}

static void mapphone_panel_remove(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	DBG("remove\n");

	device_remove_file(&dssdev->dev, &dev_attr_supplier_id);
	device_remove_file(&dssdev->dev, &dev_attr_cabc_mode);
	device_remove_file(&dssdev->dev, &dev_attr_secret_off);
	device_remove_file(&dssdev->dev, &dev_attr_acl_mode);
	if (panel_data->use_ext_te) {
		int gpio = panel_data->ext_te_gpio;
		free_irq(gpio_to_irq(gpio), dssdev);
		gpio_free(gpio);
	}

	cancel_delayed_work(&mp_data->esd_work);
	destroy_workqueue(mp_data->esd_wq);
	if (mp_data->force_update)
		destroy_workqueue(mp_data->te_wq);
	/* reset, to be sure that the panel is in a valid state */
	mapphone_hw_reset(dssdev);

	free_regulators(mp_data->panel_config->regulators,
	mp_data->panel_config->num_regulators);

	kfree(mp_data);

	return;
}

/* - In the LP mode, some panels have problems to receive command correctly
 * so we will send command out and read it back to make sure the write
 * command is accepted
 * - if the dsi_vc_dcs_write() request, then we will not care about the
 * write_dt (data type) */
static int mapphone_panel_lp_cmd_wrt_sync(struct omap_dss_device *dssdev,
					bool dcs_cmd, int write_dt,
					u8 *write_data, int write_len,
					int read_cmd, int read_len,
					int chk_val, int chk_mask)
{
	int i, ret;
	u8 data[7] = {0};

	for (i = 0; i < DCS_CMD_RETRY_MAX; i++) {
		if (dcs_cmd == true) {
			ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd,
						write_data, write_len);
			DBG("call dsi_vc_dcs_write"
				"(len=0%d, p1/p2/p3/p4=0x%x/0x%x/0x%x/0x%x)\n",
				write_len, write_data[0],
				write_data[1], write_data[2], write_data[3]);
		} else {
			ret = dsi_vc_write(dssdev, dsi_vc_cmd, write_dt,
						write_data, write_len);
			DBG("call dsi_vc_write"
				"(dt=0x%x len=%d, p1/p2/p3/p4 = "
				"0x%x/0x%x/0x%x/0x%x)\n",
				write_dt, write_len, write_data[0],
				write_data[1], write_data[2], write_data[3]);
		}

		if (ret) {
			printk(KERN_ERR "failed to send cmd=0x%x\n",
							 write_data[0]);
			continue;
		}

		mdelay(1);

		/* TODO. Do not know how to handle and to check if more than
		 * 1 byte to read is requested*/
		if (read_len < 0 || read_len > 1) {
			printk(KERN_ERR "Invalid read_len=%d\n", read_len);
			return -1;
		}

		/* Read the data back to make sure write_command is working */
		data[0] = 0;
		ret = dsi_vc_dcs_read(dssdev, dsi_vc_cmd, read_cmd,
						&data[0], read_len);

		DBG("read_chk_cmd dcs_cmd=%d read_cmd=0x%x "
				"read_len=%d chk_val=0x%x chk_mask=0x%x "
				"read_val=0x%x\n",
				dcs_cmd, read_cmd, read_len, chk_val,
				chk_mask, data[0]);

		if (ret < 0)
			DBG("fail to read 0x%x cmd and "
					"will try it again\n", read_cmd);

		if ((data[0] & chk_mask) == chk_val) {
			/* break if read back the same writing value*/
			ret  = 0;
			break;
		}
	}

	if (i >= DCS_CMD_RETRY_MAX) {
		printk(KERN_ERR "failed to read dcs_cmd=%d read_cmd=0x%x "
				"read_len=%d chk_val=0x%x chk_mask=0x%x\n"
				"read_val=0x%x\n",
				dcs_cmd, read_cmd, read_len, chk_val,
				chk_mask, data[0]);
		ret = -1;
	}

	return ret;
}

/***************************************************************************
 * Start of panel enable API
 ****************************************************************************/
/*
 * pls follow panel enable API name convention when add new panels
 * for OMAP4-based products.
 *
 * API name is composed of:
 * PROTOCOL:	dsi mipi
 * MODE:	cm for cmd mode, vm for video mode
 * SIZE:	physical dimension(4''/4.3'/3.7''')
 * RESOLUTION: width * height in pixel
 * MANUFACTURE ID: identify panel manufacture staring from 1
 * CONTROLLER VERSION:	for mulitple version such as ES1,ES2,ES5
 * dsi_mipi_mode_size_resolution{_manufactureid_version}_panel_enable
 */
static int dsi_mipi_248_vm_320_240_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;

	DBG("dsi_mipi_248_vm_320_240_panel_enable()\n");

	/* turn it on */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 1);
	if (ret)
		goto error;

	mdelay(10);

	printk(KERN_INFO "done EDISCO CTRL ENABLE\n");
	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_280_vm_320_240_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[10];
	int ret;

	DBG("dsi_mipi_280_vm_320_240_panel_enable()\n");

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;
	ret = dsi_vc_write(dssdev, dsi_vc_cmd,
				EDISCO_SHORT_WRITE_1, data, 2);

	/* Internal display set up */
	data[0] = 0xC0;
	data[1] = 0x11;
	data[2] = 0x04;
	ret = dsi_vc_write(dssdev, dsi_vc_cmd, EDISCO_LONG_WRITE, data, 3);

	/* Internal voltage set up */
	data[0] = 0xD3;
	data[1] = 0x1F;
	data[2] = 0x01;
	data[3] = 0x02;
	data[4] = 0x15;
	ret = dsi_vc_write(dssdev, dsi_vc_cmd, EDISCO_LONG_WRITE, data, 5);

	/* Internal voltage set up */
	data[0] = 0xD4;
	data[1] = 0x62;
	data[2] = 0x1E;
	data[3] = 0x00;
	data[4] = 0xB7;
	ret = dsi_vc_write(dssdev, dsi_vc_cmd, EDISCO_LONG_WRITE, data, 5);

	/* Internal display set up */
	data[0] = 0xC5;
	data[1] = 0x01;
	ret = dsi_vc_write(dssdev, dsi_vc_cmd,
				EDISCO_SHORT_WRITE_1, data, 2);

	/* Load optimized red gamma (+) settings*/
	data[0] = 0xE9;
	data[1] = 0x01;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x0B;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 10);

	/* Load optimized red gamma (-) settings*/
	data[0] = 0xEA;
	data[1] = 0x04;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x08;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 10);

	/* Load optimized green gamma (+) settings*/
	data[0] = 0xEB;
	data[1] = 0x02;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x0B;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 10);

	/* Load optimized green gamma (-) settings*/
	data[0] = 0xEC;
	data[1] = 0x05;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x08;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 10);

	/* Load optimized blue gamma (+) settings*/
	data[0] = 0xED;
	data[1] = 0x04;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x0B;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 10);

	/* Load optimized blue gamma (-) settings*/
	data[0] = 0xEE;
	data[1] = 0x07;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x08;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 10);

	/* turn on mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x03;
	ret = dsi_vc_write(dssdev, dsi_vc_cmd,
					EDISCO_SHORT_WRITE_1, data, 2);

	/* turn it on */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 1);
	if (ret)
		goto error;

	mdelay(10);

	printk(KERN_INFO "done EDISCO CTRL ENABLE\n");
	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_cm_480_854_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;

	DBG("dsi_mipi_cm_480_854_panel_enable()\n");

	/* 120ms delay for internal stabilization */
	msleep(120);

	/* Check if the display we are using is actually a TMD display */
	if (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_370_480_854) {
		if (read_supplier_id(dssdev) ==  CTL_SUPPLIER_ID_TMD) {
			DBG("dsi_mipi_cm_480_854_panel_enable() - TMD panel\n");
			dssdev->panel.panel_id = MOT_DISP_MIPI_CM_480_854;
		}
	}

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = EDISCO_CMD_MCS_OFF;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
						false, EDISCO_SHORT_WRITE_1,
						data, 2,
						EDISCO_CMD_SET_MCS, 1,
						EDISCO_CMD_MCS_OFF, 0x3);
	if (ret)
		printk(KERN_ERR "failed to send SET_MCS OFF\n");


	msleep(10);

	/* enable lane setting and test registers*/
	data[0] = 0xef;
	data[1] = 0x01;
	data[2] = 0x01;
	data[3] = 0x00;
	ret = dsi_vc_write(dssdev, dsi_vc_cmd, EDISCO_LONG_WRITE, data, 4);
	DBG("/* enable lane setting and test registers: %d*/\n", ret);
	/* 2nd param 61 = 1 line; 63 = 2 lanes */
	data[0] = 0xef;
	data[1] = 0x60;
	data[2] = 0x63;
	data[3] = 0x00;
	DBG("/* 2nd param 61 = 1 line; 63 = 2 lanes */\n");
	if (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_480_854) {
		/* Reading lane_config and it will return
		* 0x63 or 2-lanes, 0x60 for 1-lane (1st source displ only)*/
		ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
					true, 0x00,
					data, 4,
					0xef, 1,
					0x63, 0x63);
		DBG("0x63 or 2-lanes, 0x60 for 1-lane (1st source displ only)*/\n");
	} else {
		/* Reading lane_config and it will return
		* 0x1 for 2-lanes, 0x0 for 1-lane (2nd source displ only)*/
		ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
					true, 0x00,
					data, 4,
					EDISCO_CMD_DATA_LANE_CONFIG, 1,
					0x1, 0x1);
		DBG("0x1 for 2-lanes, 0x0 for 1-lane (2nd source displ only)*/\n");
	}

	if (ret)
		printk(KERN_ERR "failed to send LANE_CONFIG\n");

	msleep(10);

	/* Forcing display inversion off for hardware issue
	 * on some phones (observed inverted color, ~1% of powerups fail)
	 */
	data[0] = EDISCO_CMD_SET_INVERSION_OFF;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
				true, 0,
				data, 1,
				EDISCO_CMD_READ_DISPLAY_IMAGE_MODE, 1,
				0x00, EDISCO_CMD_SET_INVERSION_OFF);
	if (ret)
		printk(KERN_ERR "failed to send EDISCO_CMD_SET_INVERSION_OFF \n");

	msleep(10);

	/* 2nd param 0 = WVGA; 1 = WQVGA */
	data[0] = EDISCO_CMD_SET_DISPLAY_MODE;
	data[1] = 0x00;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
					true, 0x00,
					data, 2,
					EDISCO_CMD_SET_DISPLAY_MODE, 1,
					data[1], 0x01);
	DBG("/* 2nd param 0 = WVGA; 1 = WQVGA */\n");

	if (ret)
		printk(KERN_ERR "failed to send SET_DISPLAY_MODE\n");

	msleep(10);

	/* Set dynamic backlight control and PWM; D[7:4] = PWM_DIV[3:0];*/
	/* D[3]=0 (PWM OFF);
	 * D[2]=0 (auto BL control OFF for 1st source display only);
	 * D[1]=0 (Grama correction On for 1st source display only);
	 * D[0]=0 (Enhanced Image Correction OFF) */
	data[0] = EDISCO_CMD_SET_BCKLGHT_PWM;
	/* AUO displays require a different setting */
	if (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_370_480_854) {
		printk("AUO displays require a different setting: 0x09\n");
		data[1] = display_brightness();
		printk("Set display to :%u\n", display_brightness());
	} else {
		printk("AUO displays require a different setting: 0x09: 0x1f\n");
		data[1] = display_brightness();
		printk("Set display to :%u\n", display_brightness());
	}

	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
					true, 0x00,
					data, 2,
					EDISCO_CMD_SET_BCKLGHT_PWM, 1,
					data[1], 0x1f);

	DBG("EDISCO_CMD_SET_BCKLGHT_PWM\n");
	if (ret)
		printk(KERN_ERR "failed to send CABC/PWM\n");

	msleep(10);

	/* turn on mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = EDISCO_CMD_MCS_ON;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
						false, EDISCO_SHORT_WRITE_1,
						data, 2,
						EDISCO_CMD_SET_MCS, 1,
						EDISCO_CMD_MCS_ON, 0x3);
	if (ret)
		printk(KERN_ERR "failed to send SET_MCS ON\n");

	msleep(10);

	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
			true, 0x0,
			data, 1,
			EDISCO_CMD_GET_POWER_MODE, 1,
			EDISCO_CMD_SLEEP_MODE_OUT, EDISCO_CMD_SLEEP_MODE_OUT);
	DBG("EDISCO_CMD_GET_POWER_MODE\n");
	if (ret) {
		printk(KERN_ERR "failed to send EXIT_SLEEP_MODE\n");
		goto error;
	}

	msleep(200);

	printk(KERN_INFO "done EDISCO CTRL ENABLE\n");

	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_430_cm_480_854_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;
	DBG("dsi_mipi_430_cm_480_854_panel_enable()\n");

	/* Exit sleep mode */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
			true, 0x0,
			data, 1,
			EDISCO_CMD_GET_POWER_MODE, 1,
			EDISCO_CMD_SLEEP_MODE_OUT, EDISCO_CMD_SLEEP_MODE_OUT);
	if (ret) {
		printk(KERN_ERR "failed to send EXIT_SLEEP_MODE\n");
		goto error;
	}

	/* 120ms delay for internal block stabilization */
	msleep(120);

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
				false, EDISCO_SHORT_WRITE_1,
				data, 2,
				EDISCO_CMD_SET_MCS, 1,
				EDISCO_CMD_MCS_OFF, 0x3);
	if (ret)
		printk(KERN_ERR "failed to send SET_MCS\n");


	/* Enable 2 data lanes */
	data[0] = EDISCO_CMD_DATA_LANE_CONFIG;
	data[1] = EDISCO_CMD_DATA_LANE_TWO;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
			false, EDISCO_SHORT_WRITE_1,
			data, 2,
			EDISCO_CMD_DATA_LANE_CONFIG, 1,
			EDISCO_CMD_DATA_LANE_TWO, EDISCO_CMD_DATA_LANE_TWO);
	if (ret)
		printk(KERN_ERR"failed to send DATA_LANE_CONFIG\n");

	msleep(10);

	/* Set dynamic backlight control and PWM; D[7:4] = PWM_DIV[3:0];*/
	/* D[3]=0 (PWM OFF);
	 * D[2]=0 ;
	 * D[1]=0 ;
	 * D[0]=0 (Enhanced Image Correction OFF) */
	data[0] = EDISCO_CMD_SET_BCKLGHT_PWM;
	data[1] = 0xd9;
	ret = mapphone_panel_lp_cmd_wrt_sync(dssdev,
				true, 0x00,
				data, 2,
				EDISCO_CMD_SET_BCKLGHT_PWM, 1,
				data[1], 0xff);
	if (ret)
		printk(KERN_ERR"failed to send CABC/PWM\n");

	mdelay(200);

	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_310_1_cm_320_480_panel_enable(
				struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;

	DBG("dsi_mipi_310_1_cm_320_480_panel_enable()\n");

	data[0] = EDISCO_CMD_SOFT_RESET;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 1);
	if (ret)
		goto error;

	mdelay(15);

	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 1);
	if (ret)
		goto error;

	mdelay(15);

	/*Dimming function setting */
	data[0] = 0x69;
	data[1] = 0x00;
	data[2] = 0xFF;
	data[3] = 0x00;
	data[4] = 0x14;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 5);
	if (ret)
		goto error;

	/*Setting display brightness */
	data[0] = 0x51;
	data[1] = 0xFF;
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 2);
	if (ret)
		goto error;

	/*Setting CABC mode */
	data[0] = EDISCO_CMD_SET_CABC;
	data[1] = 0x02;	/* 0x02 = On 0x00 = OFF */
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 2);
	if (ret)
		goto error;

	/* setting CABC */
	data[0] = 0x53;
	data[1] = 0x16;	/* Enable CABC. BCTRL=1, DD=1, BL=1*/
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 2);
	if (ret)
		goto error;

	/* setting PWM */
	data[0] = 0x6B;
	data[1] = 0x00;
	data[2] = 0x01;	/* 0x01 = 31.26kHz */
	ret = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 3);
	if (ret)
		goto error;

	return 0;

error:
	return -EINVAL;

}

static int dsi_mipi_cm_400_540_960_m1_v1_panel_enable(
		struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[20];

	DBG("%s\n", __func__);

	buf[0] = EDISCO_CMD_SOFT_RESET;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 1);
	if (r)
		goto err;

	msleep(10);

	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 1);
	if (r)
		goto err;

	msleep(10);

	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xF1;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xFC;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xD0;
	buf[1] = 0x8E;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;
	buf[0] = 0xD2;
	buf[1] = 0x04;
	buf[2] = 0x35;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xD2;
	buf[1] = 0x05;
	buf[2] = 0x35;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xF7;
	buf[1] = 0x00;
	buf[2] = 0xD0;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;
	buf[0] = 0xC1;
	buf[1] = 0x02;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;
	buf[0] = 0xC3;
	buf[1] = 0x00;
	buf[2] = 0x03;
	buf[3] = 0x4C;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 4);
	if (r)
		goto err;
	buf[0] = 0xF2;
	buf[1] = 0x0C;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0x06;
	buf[5] = 0x04;
	buf[6] = 0x50;
	buf[7] = 0xF0;
	buf[8] = 0x00;
	buf[9] = 0x00;
	buf[10] = 0x00;
	buf[11] = 0x00;
	buf[12] = 0x01;
	buf[13] = 0x00;
	buf[14] = 0x00;
	buf[15] = 0x00;
	buf[16] = 0x55;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 17);
	if (r)
		goto err;
	buf[0] = 0xF3;
	buf[1] = 0x01;
	buf[2] = 0x00;
	buf[3] = 0x00;
	buf[4] = 0xD0;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 5);
	if (r)
		goto err;
	buf[0] = 0xF4;
	buf[1] = 0x00;
	buf[2] = 0xBB;
	buf[3] = 0x46;
	buf[4] = 0x45;
	buf[5] = 0x0C;
	buf[6] = 0x49;
	buf[7] = 0x74;
	buf[8] = 0x29;
	buf[9] = 0x12;
	buf[10] = 0x15;
	buf[11] = 0x2B;
	buf[12] = 0x2B;
	buf[13] = 0x04;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 14);
	if (r)
		goto err;
	buf[0] = 0xF6;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x06;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x04;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 7);
	if (r)
		goto err;
	buf[0] = 0xF8;
	buf[1] = 0x0F;
	buf[2] = 0x0A;
	buf[3] = 0x20;
	buf[4] = 0x1E;
	buf[5] = 0x54;
	buf[6] = 0x54;
	buf[7] = 0x54;
	buf[8] = 0x54;
	buf[9] = 0x0A;
	buf[10] = 0x12;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 11);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR"\n****Failed to init the display****\n");
	return r;

}
static int dsi_mipi_cm_400_540_960_m1_v2_panel_enable(
		struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[20];

	DBG("%s\n", __func__);

	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xF1;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	/* MIPI TE Drop out fix */
	buf[0] = 0xFC;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xFE;
	buf[1] = 0x04;
	buf[2] = 0x08;
	buf[3] = 0x26;
	buf[4] = 0x00;
	buf[5] = 0x04;
	buf[6] = 0x1B;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xFC;
	buf[1] = 0x00;
	buf[2] = 0x00;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;
	/* MIPI TE Drop out fix done*/

	buf[0] = 0xD0;
	buf[1] = 0x8E;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = EDISCO_CMD_SET_CABC;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 1);
	if (r)
		goto err;

	msleep(120);

	buf[0] = 0xB5;
	buf[1] = 0x03;
	buf[2] = 0x7F;
	buf[3] = 0x00;
	buf[4] = 0x80;
	buf[5] = 0xC7;
	buf[6] = 0x00;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xB7;
	buf[1] = 0x66;
	buf[2] = 0xF6;
	buf[3] = 0x46;
	buf[4] = 0x9F;
	buf[5] = 0x90;
	buf[6] = 0x99;
	buf[7] = 0xFF;
	buf[8] = 0x80;
	buf[9] = 0x6D;
	buf[10] = 0x01;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 11);
	if (r)
		goto err;

	buf[0] = 0xF4;
	buf[1] = 0x00;
	buf[2] = 0xBB;
	buf[3] = 0x46;
	buf[4] = 0x53;
	buf[5] = 0x0C;
	buf[6] = 0x49;
	buf[7] = 0x74;
	buf[8] = 0x29;
	buf[9] = 0x12;
	buf[10] = 0x15;
	buf[11] = 0x2F;
	buf[12] = 0x2F;
	buf[13] = 0x04;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 14);
	if (r)
		goto err;

	buf[0] = 0xF8;
	buf[1] = 0x4B;
	buf[2] = 0x04;
	buf[3] = 0x10;
	buf[4] = 0x1A;
	buf[5] = 0x2C;
	buf[6] = 0x2C;
	buf[7] = 0x2C;
	buf[8] = 0x2C;
	buf[9] = 0x14;
	buf[10] = 0x12;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 11);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x04;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x30;
	buf[4] = 0x12;
	buf[5] = 0x0E;
	buf[6] = 0x0C;
	buf[7] = 0x22;
	buf[8] = 0x27;
	buf[9] = 0x31;
	buf[10] = 0x2E;
	buf[11] = 0x07;
	buf[12] = 0x0F;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x30;
	buf[4] = 0x12;
	buf[5] = 0x0E;
	buf[6] = 0x0C;
	buf[7] = 0x22;
	buf[8] = 0x27;
	buf[9] = 0x31;
	buf[10] = 0x2E;
	buf[11] = 0x07;
	buf[12] = 0x0F;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x02;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x37;
	buf[4] = 0x15;
	buf[5] = 0x15;
	buf[6] = 0x11;
	buf[7] = 0x1F;
	buf[8] = 0x25;
	buf[9] = 0x2D;
	buf[10] = 0x2A;
	buf[11] = 0x05;
	buf[12] = 0x0F;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x37;
	buf[4] = 0x15;
	buf[5] = 0x15;
	buf[6] = 0x11;
	buf[7] = 0x1F;
	buf[8] = 0x25;
	buf[9] = 0x2D;
	buf[10] = 0x2A;
	buf[11] = 0x05;
	buf[12] = 0x0F;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x3F;
	buf[4] = 0x16;
	buf[5] = 0x1F;
	buf[6] = 0x15;
	buf[7] = 0x1F;
	buf[8] = 0x25;
	buf[9] = 0x2D;
	buf[10] = 0x2B;
	buf[11] = 0x06;
	buf[12] = 0x0B;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x3F;
	buf[4] = 0x16;
	buf[5] = 0x1F;
	buf[6] = 0x15;
	buf[7] = 0x1F;
	buf[8] = 0x25;
	buf[9] = 0x2D;
	buf[10] = 0x2B;
	buf[11] = 0x06;
	buf[12] = 0x0B;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x20;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x34;
	buf[4] = 0x15;
	buf[5] = 0x1A;
	buf[6] = 0x11;
	buf[7] = 0x1F;
	buf[8] = 0x23;
	buf[9] = 0x2D;
	buf[10] = 0x29;
	buf[11] = 0x02;
	buf[12] = 0x08;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x00;
	buf[2] = 0x2F;
	buf[3] = 0x34;
	buf[4] = 0x15;
	buf[5] = 0x1A;
	buf[6] = 0x11;
	buf[7] = 0x1F;
	buf[8] = 0x23;
	buf[9] = 0x2D;
	buf[10] = 0x29;
	buf[11] = 0x02;
	buf[12] = 0x08;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0x53;
	buf[1] = 0x2C;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR "\n****Failed to init the display****\n");
	return r;

}

static int dsi_mipi_cm_400_540_960_m1_v4_panel_enable(
		struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[20];

	DBG("dsi_mipi_400_cm_540_960_panel_enable\n");

	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xF1;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	/* MIPI TE Drop out fix */
	buf[0] = 0xFC;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xFE;
	buf[1] = 0x04;
	buf[2] = 0x08;
	buf[3] = 0x26;
	buf[4] = 0x00;
	buf[5] = 0x04;
	buf[6] = 0x1B;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xFC;
	buf[1] = 0x00;
	buf[2] = 0x00;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;
	/* MIPI TE Drop out fix done*/

	buf[0] = 0xD0;
	buf[1] = 0x8E;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xD2;
	buf[1] = 0x04;
	buf[2] = 0x53;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xD2;
	buf[1] = 0x05;
	buf[2] = 0x53;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	/* DBLC MODE select, mode 2, normal mode set */
	buf[0] = EDISCO_CMD_SET_CABC;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 1);
	if (r)
		goto err;

	msleep(120);

	buf[0] = 0xB5;
	buf[1] = 0x03;
	buf[2] = 0x7F;
	buf[3] = 0x0A;
	buf[4] = 0x80;
	buf[5] = 0xFF;
	buf[6] = 0x00;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xB7;
	buf[1] = 0x7A;
	buf[2] = 0xF7;
	buf[3] = 0x4D;
	buf[4] = 0x91;
	buf[5] = 0x90;
	buf[6] = 0xB3;
	buf[7] = 0xFF;
	buf[8] = 0x80;
	buf[9] = 0x6D;
	buf[10] = 0x01;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 11);
	if (r)
		goto err;

	buf[0] = 0xF4;
	buf[1] = 0x00;
	buf[2] = 0xBB;
	buf[3] = 0x46;
	buf[4] = 0x53;
	buf[5] = 0x0C;
	buf[6] = 0x49;
	buf[7] = 0x74;
	buf[8] = 0x29;
	buf[9] = 0x12;
	buf[10] = 0x15;
	buf[11] = 0x37;
	buf[12] = 0x37;
	buf[13] = 0x04;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 14);
	if (r)
		goto err;

	buf[0] = 0xF8;
	buf[1] = 0x0A;
	buf[2] = 0x04;
	buf[3] = 0x10;
	buf[4] = 0x2A;
	buf[5] = 0x35;
	buf[6] = 0x35;
	buf[7] = 0x35;
	buf[8] = 0x35;
	buf[9] = 0x21;
	buf[10] = 0x1A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 11);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x04;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x08;
	buf[2] = 0x1C;
	buf[3] = 0x1B;
	buf[4] = 0x0F;
	buf[5] = 0x0F;
	buf[6] = 0x0A;
	buf[7] = 0x1E;
	buf[8] = 0x22;
	buf[9] = 0x27;
	buf[10] = 0x26;
	buf[11] = 0x07;
	buf[12] = 0x0D;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x08;
	buf[2] = 0x3C;
	buf[3] = 0x27;
	buf[4] = 0x0F;
	buf[5] = 0x0F;
	buf[6] = 0x0A;
	buf[7] = 0x1E;
	buf[8] = 0x26;
	buf[9] = 0x31;
	buf[10] = 0x2F;
	buf[11] = 0x07;
	buf[12] = 0x0B;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x02;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x30;
	buf[2] = 0x14;
	buf[3] = 0x0F;
	buf[4] = 0x00;
	buf[5] = 0x06;
	buf[6] = 0x02;
	buf[7] = 0x1E;
	buf[8] = 0x22;
	buf[9] = 0x27;
	buf[10] = 0x27;
	buf[11] = 0x08;
	buf[12] = 0x10;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x30;
	buf[2] = 0x35;
	buf[3] = 0x0F;
	buf[4] = 0x00;
	buf[5] = 0x0A;
	buf[6] = 0x02;
	buf[7] = 0x1C;
	buf[8] = 0x23;
	buf[9] = 0x31;
	buf[10] = 0x2F;
	buf[11] = 0x08;
	buf[12] = 0x0E;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x12;
	buf[2] = 0x1B;
	buf[3] = 0x26;
	buf[4] = 0x0E;
	buf[5] = 0x12;
	buf[6] = 0x0B;
	buf[7] = 0x1E;
	buf[8] = 0x22;
	buf[9] = 0x27;
	buf[10] = 0x27;
	buf[11] = 0x06;
	buf[12] = 0x0C;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x12;
	buf[2] = 0x3B;
	buf[3] = 0x2C;
	buf[4] = 0x12;
	buf[5] = 0x12;
	buf[6] = 0x0E;
	buf[7] = 0x1E;
	buf[8] = 0x26;
	buf[9] = 0x31;
	buf[10] = 0x2F;
	buf[11] = 0x06;
	buf[12] = 0x0D;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xF9;
	buf[1] = 0x20;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x37;
	buf[2] = 0x1B;
	buf[3] = 0x09;
	buf[4] = 0x01;
	buf[5] = 0x06;
	buf[6] = 0x04;
	buf[7] = 0x19;
	buf[8] = 0x19;
	buf[9] = 0x22;
	buf[10] = 0x24;
	buf[11] = 0x04;
	buf[12] = 0x15;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x37;
	buf[2] = 0x3B;
	buf[3] = 0x17;
	buf[4] = 0x01;
	buf[5] = 0x0A;
	buf[6] = 0x04;
	buf[7] = 0x19;
	buf[8] = 0x1D;
	buf[9] = 0x2C;
	buf[10] = 0x2C;
	buf[11] = 0x04;
	buf[12] = 0x13;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	/*  PWM output control */
	buf[0] = 0x53;
	buf[1] = 0x2C;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR "\n****Failed to init the display****\n");
	return r;

}


static int dsi_mipi_cm_400_540_960_m2_v1_panel_enable(
		struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[20];

	DBG("%s\n", __func__);

	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	buf[0] = EDISCO_CMD_SET_CABC;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	/* MIPI TE Drop out fix */
	buf[0] = 0xFC;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xFE;
	buf[1] = 0x04;
	buf[2] = 0x08;
	buf[3] = 0x26;
	buf[4] = 0x00;
	buf[5] = 0x04;
	buf[6] = 0x1B;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xFC;
	buf[1] = 0x00;
	buf[2] = 0x00;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;
	/* MIPI TE Drop out fix done*/

	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 1);
	if (r)
		goto err;

	msleep(120);

	buf[0] = 0xB5;
	buf[1] = 0x03;
	buf[2] = 0x7F;
	buf[3] = 0x00;
	buf[4] = 0x80;
	buf[5] = 0xFF;
	buf[6] = 0x00;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xB7;
	buf[1] = 0xBA;
	buf[2] = 0xFE;
	buf[3] = 0x4E;
	buf[4] = 0xA5;
	buf[5] = 0x90;
	buf[6] = 0xB0;
	buf[7] = 0xFF;
	buf[8] = 0x80;
	buf[9] = 0x6D;
	buf[10] = 0x01;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 11);
	if (r)
		goto err;
	/* change to 0x7A to resolve the flicker and blackout issues */
	buf[0] = 0xF4;
	buf[1] = 0x00;
	buf[2] = 0x7A;
	buf[3] = 0x46;
	buf[4] = 0x20;
	buf[5] = 0x00;
	buf[6] = 0x49;
	buf[7] = 0x74;
	buf[8] = 0x29;
	buf[9] = 0x12;
	buf[10] = 0x14;
	buf[11] = 0x2F;
	buf[12] = 0x2F;
	buf[13] = 0x01;
	buf[14] = 0x00;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 15);
	if (r)
		goto err;

	buf[0] = 0xF6;
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x06;
	buf[4] = 0x00;
	buf[5] = 0x00;
	buf[6] = 0x0C;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 7);
	if (r)
		goto err;

	buf[0] = 0xF8;
	buf[1] = 0x15;
	buf[2] = 0x1C;
	buf[3] = 0x00;
	buf[4] = 0x18;
	buf[5] = 0x49;
	buf[6] = 0x49;
	buf[7] = 0x49;
	buf[8] = 0X49;
	buf[9] = 0X14;
	buf[10] = 0X16;
	buf[11] = 0X01;
	buf[12] = 0X64;
	buf[13] = 0X64;
	buf[14] = 0X02;
	buf[15] = 0X24;
	buf[16] = 0X64;
	buf[17] = 0X00;
	buf[18] = 0X00;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 19);
	if (r)
		goto err;

	/* Red */
	buf[0] = 0xF9;
	buf[1] = 0x04;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x0C;
	buf[2] = 0x2C;
	buf[3] = 0x29;
	buf[4] = 0x11;
	buf[5] = 0x03;
	buf[6] = 0x02;
	buf[7] = 0x20;
	buf[8] = 0x24;
	buf[9] = 0x2D;
	buf[10] = 0x23;
	buf[11] = 0x05;
	buf[12] = 0x0A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x0C;
	buf[2] = 0x2C;
	buf[3] = 0x29;
	buf[4] = 0x11;
	buf[5] = 0x03;
	buf[6] = 0x02;
	buf[7] = 0x20;
	buf[8] = 0x24;
	buf[9] = 0x2D;
	buf[10] = 0x23;
	buf[11] = 0x05;
	buf[12] = 0x0A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	/* Green */
	buf[0] = 0xF9;
	buf[1] = 0x02;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x10;
	buf[2] = 0x24;
	buf[3] = 0x2B;
	buf[4] = 0x0F;
	buf[5] = 0x08;
	buf[6] = 0x02;
	buf[7] = 0x27;
	buf[8] = 0x22;
	buf[9] = 0x32;
	buf[10] = 0x2E;
	buf[11] = 0x07;
	buf[12] = 0x0E;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;
	buf[0] = 0xFB;
	buf[1] = 0x10;
	buf[2] = 0x24;
	buf[3] = 0x2B;
	buf[4] = 0x0F;
	buf[5] = 0x08;
	buf[6] = 0x02;
	buf[7] = 0x27;
	buf[8] = 0x22;
	buf[9] = 0x32;
	buf[10] = 0x2E;
	buf[11] = 0x07;
	buf[12] = 0x0E;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	/* Blue */
	buf[0] = 0xF9;
	buf[1] = 0x01;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x12;
	buf[2] = 0x2B;
	buf[3] = 0x2D;
	buf[4] = 0x0A;
	buf[5] = 0x12;
	buf[6] = 0x0C;
	buf[7] = 0x21;
	buf[8] = 0x24;
	buf[9] = 0x2C;
	buf[10] = 0x21;
	buf[11] = 0x06;
	buf[12] = 0x06;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x12;
	buf[2] = 0x2B;
	buf[3] = 0x2D;
	buf[4] = 0x0A;
	buf[5] = 0x12;
	buf[6] = 0x0C;
	buf[7] = 0x21;
	buf[8] = 0x24;
	buf[9] = 0x2C;
	buf[10] = 0x21;
	buf[11] = 0x06;
	buf[12] = 0x06;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;
	/* White */
	buf[0] = 0xF9;
	buf[1] = 0x20;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xFA;
	buf[1] = 0x3B;
	buf[2] = 0x1F;
	buf[3] = 0x0E;
	buf[4] = 0x01;
	buf[5] = 0x08;
	buf[6] = 0x04;
	buf[7] = 0x19;
	buf[8] = 0x1B;
	buf[9] = 0x27;
	buf[10] = 0x28;
	buf[11] = 0x04;
	buf[12] = 0x14;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;

	buf[0] = 0xFB;
	buf[1] = 0x3B;
	buf[2] = 0x1F;
	buf[3] = 0x0E;
	buf[4] = 0x01;
	buf[5] = 0x08;
	buf[6] = 0x04;
	buf[7] = 0x19;
	buf[8] = 0x1B;
	buf[9] = 0x27;
	buf[10] = 0x28;
	buf[11] = 0x04;
	buf[12] = 0x14;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 13);
	if (r)
		goto err;
	buf[0] = 0x53;
	buf[1] = 0x2C;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR "\n****Failed to init the display****\n");
	return r;
}
/*TODO: port over later*/
void reconfigure_dsi_pll(struct omap_dss_device *dssdev)
{
}

static int dsi_mipi_cm_430_540_960_m3_v1_panel_enable(
		struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[20];
	static u16 manufacture_id = INVALID_VALUE;
	static u16 controller_ver = INVALID_VALUE;
	static u16 driver_ver = INVALID_VALUE;

	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	DBG("%s\n", __func__);

	if (controller_ver == INVALID_VALUE) {
		read_supplier_id(dssdev);
		manufacture_id = read_panel_manufacture(dssdev);
		controller_ver = read_panel_controller_version(dssdev);
		driver_ver = read_panel_controller_driver_version(dssdev);
		if (manufacture_id == INVALID_VALUE ||
			controller_ver == INVALID_VALUE ||
			driver_ver == INVALID_VALUE) {
			printk(KERN_ERR "Failed to detect panel, manufacture_id"
				" =%d, controller_ver =%d, driver_ver = %d\n ",
				manufacture_id, controller_ver, driver_ver);
			return -EINVAL;
		}
	}

	/* Read revision,  ES1 or ES2 panel, ES1 can not run high mipi clk */
	if (driver_ver == 1) {
		reconfigure_dsi_pll(dssdev);
		/* Don't want to run TE in ES1 panel */
		panel_data->te_support = false;

		buf[0] = EDISCO_CMD_SOFT_RESET;
		r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 1);
		if (r)
			goto err;

		msleep(10);
	}

	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 1);
	if (r)
		goto err;

	msleep(120);

	buf[0] = 0x51;
	buf[1] = 0xFF;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = EDISCO_CMD_SET_CABC;
	buf[1] = 0x03;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0x53;
	buf[1] = 0x2C;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;
	return 0;
err:
	printk(KERN_ERR "\n****Failed to init the display****\n");
	return r;

}

static int mapphone_panel_enable_te_locked(struct omap_dss_device *dssdev,
				bool enable);

static int dsi_mipi_cm_430_540_960_amoled_bl_set_locked(
	struct omap_dss_device *dssdev, int level);

static int dsi_mipi_430_cm_540_960_amoled_panel_enable(
						struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	int r;
	u8 buf[30];
	static u16 controller_ver = INVALID_VALUE;

	if (controller_ver == INVALID_VALUE) {
		controller_ver = read_panel_controller_version(dssdev);
		if (controller_ver == INVALID_VALUE) {
			printk(KERN_ERR "Failed to detect panel, controller_ver =%d\n",
					controller_ver);
			return -EINVAL;
		}
	}

	DBG("dsi_mipi_430_cm_540_960_amoled_panel_enable\n");

	column_address_offset = 30;

	/*
	 * Issue: on some spyder SMD amoled panel, when unlocked phone see weird
	 * hue of colors very rarely and randomly. The issue is caused by wrong
	 * FDh value, which is controller internal booster voltage.
	 * when send panel init seq, FDh is programmed to 0x1F, but sometimes
	 * read back 0x0F. The mismatch only happens in LP mode and only happen
	 * to that FDh register. Suspect controller has problem in LP mode.
	 * Workaround: send panel init seq in HS mode.
	 */
	omapdss_dsi_vc_enable_hs(dssdev, dsi_vc_cmd, true);

	/* ETC condition set 1 */
	buf[0] = 0xF0;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);

	/*
	 * Work-around for power-up stability issue where the first DCS
	 * command sent after power-up fails due to glitches on MIPI bus. If
	 * the command fails, retry once
	 */
	if (r) {
		r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
		if (r)
			goto err;
	}

	buf[0] = 0xF1;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	buf[0] = 0xFC;
	buf[1] = 0x5A;
	buf[2] = 0x5A;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	/* Gamma condition set */
	r = dsi_mipi_cm_430_540_960_amoled_bl_set_locked(dssdev,
			mp_data->bldev->props.brightness);
	if (r)
		goto err;

	/* Panel Condition set */
	buf[0]  = 0xF8;
	buf[1]  = 0x27;
	buf[2]  = 0x27;
	buf[3]  = 0x08;
	buf[4]  = 0x08;
	buf[5]  = 0x4E;
	buf[6]  = 0xAA;
	buf[7]  = 0x5E;
	buf[8]  = 0x8A;
	buf[9]  = 0x10;
	buf[10] = 0x3F;
	buf[11] = 0x10;
	buf[12] = 0x10;
	buf[13] = 0x00;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 14);
	if (r)
		goto err;

	/* ETC condition set 2 */
	buf[0]  = 0xF6;
	buf[1]  = 0x00;
	buf[2]  = 0x84;
	buf[3]  = 0x09;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 4);
	if (r)
		goto err;

	/* Set the starting address of the parameter of next coming command */
	buf[0]  = 0xB0;
	buf[1]  = 0x09;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xD5;
	buf[1]  = 0x64;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xb0;
	buf[1]  = 0x0b;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xd5;
	buf[1]  = 0xa4;
	buf[2]  = 0x7e;
	buf[3]  = 0x20;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 4);
	if (r)
		goto err;

	buf[0]  = 0xB0;
	buf[1]  = 0x08;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xFD;
	buf[1]  = 0xF8;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xB0;
	buf[1]  = 0x04;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xF2;
	buf[1]  = 0x4D;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xB0;
	buf[1]  = 0x05;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xFD;
	buf[1]  = 0x1F;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0]  = 0xB1;
	buf[1]  = 0x01;
	buf[2]  = 0x00;
	buf[3]  = 0x16;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 4);
	if (r)
		goto err;

	buf[0]  = 0xb2;
	buf[1]  = 0x06;
	buf[2]  = 0x06;
	buf[3]  = 0x06;
	buf[4]  = 0x06;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 5);
	if (r)
		goto err;

	/* Exit sleep mode */
	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 1);
	if (r)
		goto err;

	msleep(120);

	/* Memory window setting 2 */
	/* The 35h command is handled automatically by common
	 * display code, leave it out here.
	 */

	buf[0]  = 0x2A;
	buf[1]  = 0x00;
	buf[2]  = 0x1e;
	buf[3]  = 0x02;
	buf[4]  = 0x39;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 5);
	if (r)
		goto err;

	buf[0]  = 0x2B;
	buf[1]  = 0x00;
	buf[2]  = 0x00;
	buf[3]  = 0x03;
	buf[4]  = 0xbf;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 5);
	if (r)
		goto err;

	buf[0]  = 0xD1;
	buf[1]  = 0x8a;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	r = mapphone_panel_acl_enable_locked(mp_data->acl_enabled, mp_data,
					dssdev);
	if (r)
		goto err;

	/* 70% ACL */
	buf[0]  = 0xc1;
	buf[1]  = 0x47;
	buf[2]  = 0x53;
	buf[3]  = 0x13;
	buf[4]  = 0x53;
	buf[5]  = 0x00;
	buf[6]  = 0x00;
	buf[7]  = 0x01;
	buf[8]  = 0xdf;
	buf[9]  = 0x00;
	buf[10] = 0x00;
	buf[11] = 0x03;
	buf[12] = 0x1f;
	buf[13] = 0x00;
	buf[14] = 0x00;
	buf[15] = 0x00;
	buf[16] = 0x00;
	buf[17] = 0x00;
	buf[18] = 0x01;
	buf[19] = 0x02;
	buf[20] = 0x03;
	buf[21] = 0x07;
	buf[22] = 0x0e;
	buf[23] = 0x14;
	buf[24] = 0x1c;
	buf[25] = 0x24;
	buf[26] = 0x2d;
	buf[27] = 0x2d;
	buf[28] = 0x00;
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, 29);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR "\n****Failed to init the display****\n");
	return r;
}

static int mipi_cm_720_1280_freq_adjust_recap(struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[5];

	DBG("%s\n", __func__);

	buf[0] = 0x00;
	buf[1] = 0x83;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	/*
	 * 0x44 will change the display refresh to 60-62fps. Default is at
	 * 0x0A which the display will refresh at 68 fps
	 */
	buf[0] = 0xC0;
	buf[1] = 0x44;
	buf[2] = 0x44;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	return r;
err:
	printk(KERN_ERR "Mapphone panel: failed to send freq adjust recap\n");
	return r;
}

static int mipi_cm_720_1280_get_ta_go_value(struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[5];

	DBG("%s\n", __func__);

	buf[0] = 0x00;
	buf[1] = 0x82;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xff;
	if (dsi_vc_dcs_read(dssdev, dsi_vc_cmd, 0xb0, buf, 1) != 1)
		printk(KERN_ERR "Mapphone panel: failed to read panel TA_GO\n");

	return buf[0];
err:
	printk(KERN_ERR "Mapphone panel: failed to set index for 720_1280\n");
	return INVALID_VALUE;
}

static int mipi_cm_720_1280_level_2_enable(struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[5];

	DBG("%s\n", __func__);

	/* Enable command 2 */
	buf[0] = 0xff;
	buf[1] = 0x12;
	buf[2] = 0x80;
	buf[3] = 0x01;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 4);
	if (r)
		goto err;

	/* Enable the display manufacture mode */
	buf[0] = 0x00;
	buf[1] = 0x80;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	buf[0] = 0xff;
	buf[1] = 0x12;
	buf[2] = 0x80;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 3);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR "Mapphone panel: "
		"failed to enable level_2 mode for 720x1280 panel\n");
	return r;

}

/*TODO: port it over later*/
void omapdss_dsi_mod_phy_reg1(struct omap_dss_device *dssdev, int channel,
				u32 val, u16 start, u16 end)
{
}
static int mipi_cm_720_1280_detection(struct omap_dss_device *dssdev)
{
	static int panel_info = INVALID_VALUE;
	int r;
	/*
	 * There is a issue of the p1 display that cause ERRCONTENTIONLP1_2,
	 * This issue is casued by the wrong programmed TA_GO value
	 */
	const int good_ta_go_panel = 0x30;

	DBG("%s\n", __func__);

	if (panel_info != INVALID_VALUE) {
		if (panel_info != good_ta_go_panel)
			/* change TTAGET in DSI_PHY_REGISTER1 to 6 (9cycles) */
			omapdss_dsi_mod_phy_reg1(dssdev, dsi_vc_cmd, 0x06,
								26, 24);
		goto freq_adj_recap;
	}

	/*
	 * There is an issue of the p1 display that cause ERRCONTENTIONLP1_2
	 * To get around this we need to increase the TTAGET from 2 (5cycle)
	 * to 6 (9 cycles). From testing, we still observe problem at 4.
	 */
	omapdss_dsi_mod_phy_reg1(dssdev, dsi_vc_cmd, 0x06, 26, 24);

	r = mipi_cm_720_1280_level_2_enable(dssdev);
	if (r)
		goto err;

	r = mipi_cm_720_1280_get_ta_go_value(dssdev);
	if (r == 0xff)
		printk(KERN_ERR "Mapphone panel: failed to read panel TA_GO\n");
	else
		panel_info = r;

	/*
	 * This is a good display, reset DSI_PHY_REGISTER1 back to default
	 * value
	 */
	if (panel_info == good_ta_go_panel)
		omapdss_dsi_mod_phy_reg1(dssdev, dsi_vc_cmd, 0x02, 26, 24);
	else
		/* This is bad display */
		printk(KERN_WARNING "panel mapphone WARNING: This is a bad "
			"display and it should not be used for any "
			"performance measurement\n");

	r = mipi_cm_720_1280_freq_adjust_recap(dssdev);
	if (r)
		goto err;

	return panel_info;

freq_adj_recap:
	r = mipi_cm_720_1280_level_2_enable(dssdev);
	if (r)
		goto err;

	r = mipi_cm_720_1280_freq_adjust_recap(dssdev);
	if (r)
		goto err;

	return panel_info;
err:
	printk(KERN_ERR "Mapphone panel: failed to detect 720_1280 panel\n");
	return INVALID_VALUE;

}

static int dsi_mipi_cm_450_720_1280_panel_enable(struct omap_dss_device *dssdev)
{
	int r;
	u8 buf[20];
	static u16 manufacture_id = INVALID_VALUE;
	static u16 controller_ver = INVALID_VALUE;

	DBG("%s\n", __func__);

	mipi_cm_720_1280_detection(dssdev);

	if (controller_ver == INVALID_VALUE) {
		read_supplier_id(dssdev);
		manufacture_id = read_panel_manufacture(dssdev);
		controller_ver = read_panel_controller_version(dssdev);
		read_panel_controller_driver_version(dssdev);
		if (manufacture_id == INVALID_VALUE ||
			controller_ver == INVALID_VALUE) {
			printk(KERN_ERR "Failed to detect panel, manufacture_id"
				" =%d, controller_ver =%d.\n ", manufacture_id,
				controller_ver);
			return -EINVAL;
		}
	}


	buf[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 1);
	if (r)
		goto err;

	msleep(120);

	buf[0] = 0x51;
	buf[1] = 0xFF;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	/* BL Control */
	buf[0] = 0x53;
	buf[1] = 0x2C;
	r = dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, buf, 2);
	if (r)
		goto err;

	return 0;
err:
	printk(KERN_ERR"\n****Failed to init the display****\n");
	return r;
}

static int dsi_lvds_mipi_vm_1007_1280_800_panel_enable(
						struct omap_dss_device *dssdev)
{
	return 0;//mapphone_panel_d2l_on(dssdev);
}

/****************************************************************************
 *End of panel enable API
 ****************************************************************************/

static int mapphone_panel_MIPI_540_960_detect(struct omap_dss_device *dssdev)
{
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int ret;
	static u16 manufacture_id = INVALID_VALUE;
	static u16 controller_ver = INVALID_VALUE;

	if (manufacture_id  ==  INVALID_VALUE) {
		read_supplier_id(dssdev);
		manufacture_id = read_panel_manufacture(dssdev);
		controller_ver = read_panel_controller_version(dssdev);
		read_panel_controller_driver_version(dssdev);
		if (manufacture_id == INVALID_VALUE ||
			controller_ver == INVALID_VALUE) {
			printk(KERN_ERR "Failed to detect panel, manufacture_id"
				" =%d, controller_ver =%d.\n ", manufacture_id,
				controller_ver);
			return -EINVAL;
		}
	}
	/*
	 * Issue: see qHD panel dim screen sometimes, there is
	 * no PWM output, signal stays low, it happens randomly.
	 * The reason is CABC setting registers 0x53 or 0x55 are
	 * not programmed correctly in LP mode,0x54/0x56 read back
	 * 0x0 when they are supposed to be 0x2C and 0x01. qHD
	 * panel have communication problems sometimes in LP mode.
	 *
	 * To work it around, send qHD panel power up sequence
	 * in HS mode. ES2.0 and later support both LP and HS mode.
	 */
	omapdss_dsi_vc_enable_hs(dssdev, dsi_vc_cmd, true);

	switch (manufacture_id) {
	case 00: /* 1st panel manufacture */
		switch (controller_ver) {
		case 00: /*ES1*/
			omapdss_dsi_vc_enable_hs(dssdev, dsi_vc_cmd, false);

			reconfigure_dsi_pll(dssdev);
			/* Don't want to run TE in ES1 panel */
			panel_data->te_support = false;

			ret = dsi_mipi_cm_400_540_960_m1_v1_panel_enable(
				dssdev);
			break;
		case 01: /*ES2,3*/
			ret = dsi_mipi_cm_400_540_960_m1_v2_panel_enable(
				dssdev);
			break;
		case 02: /*ES4*/
		case 03: /*ES5*/
			ret = dsi_mipi_cm_400_540_960_m1_v4_panel_enable(
				dssdev);
			break;
		default:
			ret = dsi_mipi_cm_400_540_960_m1_v4_panel_enable(
				dssdev);
			printk(KERN_ERR " Not support controller version = %d,"
				" use E4/5 panel init sequence for now.\n",
				 controller_ver);
		}
		break;
	case 01: /* 2nd panel manufacture */
		switch (controller_ver) {
		case 00:
		case 01: /*ES1*/
		case 02: /*ES2*/
			ret = dsi_mipi_cm_400_540_960_m2_v1_panel_enable(
				dssdev);
			break;
		default:
			ret = dsi_mipi_cm_400_540_960_m2_v1_panel_enable(
				dssdev);
			printk(KERN_ERR " Not support controller version = %d,"
				" use ES2 panel init sequence for now.\n",
				 controller_ver);
		}
		break;
	default: /* 3rd and 4th panel manufacture */
		printk(KERN_ERR " Not support this panel manufacture = %d\n",
				manufacture_id);
		ret = -EINVAL;
	}

	return ret;
}
static void check_mapphone_panel_present(struct omap_dss_device *dssdev)
{
	int r, retry;

	if (panel_init_state != MAPPHONE_PANEL_UNDETERMINE)
		return;

	/*
	 * By checking if Ack from the peripheral for successful cmd VC BTA
	 * is received to detect if a panel is present or not. This is
	 * applicable to the smart phone only, and we don't know how to
	 * handle the other panels for now.
	 *
	 * This must be the first command to be sent to the panel before
	 * any init seqs to avoid interfering of the no-display support
	 * for factory. dsi bus should be locked and clk should be
	 * enabled in caller.
	 */
	r = dsi_vc_send_bta_sync(dssdev, dsi_vc_cmd);
	if (!r) {
		panel_init_state = MAPPHONE_PANEL_PRESENT;
	} else {
		for (retry = 0; retry < 10; retry++) {
			r = dsi_vc_send_bta_sync(dssdev, dsi_vc_cmd);
			if (!r) {
				r = dsi_vc_send_bta_sync(dssdev, dsi_vc_cmd);
				if (!r) {
					panel_init_state =
						MAPPHONE_PANEL_PRESENT;
					break;
				}
			}
		}
		if (retry == 10) {
			panel_init_state = MAPPHONE_PANEL_NOT_PRESENT;
			printk(KERN_ERR "%s: Failed to receive BTA ACK \
				 for 10 times.\n",  __func__);
		}
	}
}

static void set_default_panel_init_state(struct omap_dss_device *dssdev)
{
	/* Todo. Don't know how to handle for non-smart panel, set panel state
	 * to be present. For smart phones, set to undetermine to figure out
	 * later by communicating with panel.
	 */
	switch (dssdev->panel.panel_id) {
	case MOT_DISP_LVDS_MIPI_VM_1007_1280_800:
		panel_init_state = MAPPHONE_PANEL_PRESENT;
		break;
	default:
		panel_init_state = MAPPHONE_PANEL_UNDETERMINE;
	}
}
static int mapphone_panel_power_on(struct omap_dss_device *dssdev)
{
	static bool first_boot = true;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	int ret;
	u8 power_mode = 0;
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if (!first_boot && dssdev->phy.dsi.d2l_use_ulps) {
		if (dssdev->platform_enable) {
			ret = dssdev->platform_enable(dssdev);
			if (ret)
				goto err0;
		}
	}

	set_default_panel_init_state(dssdev);

	/*
	 * If 'secret off' is still enabled, DSI is still enabled.  Disable
	 * it initially
	 */
	if (mp_data->som_enabled)
		omapdss_dsi_display_disable(dssdev, true, false);

	ret = omapdss_dsi_display_enable(dssdev);
	if (ret) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err0;
	}

#ifdef CONFIG_FB_OMAP_BOOTLOADER_INIT
	if (first_boot) {
		/*
		 * At this point the panel mapphone code has taken over the
		 * ownership of the dss clock states and the dss_fck and
		 * dss_sys_clk enable count can be decremented by calling
		 * clk_disable.  These clocks were left enabled during boot
		 * since the ENABLE_ON_INIT flag is set for these clocks
		 * when CONFIG_FB_OMAP_BOOTLOADER_INIT is defined.
		 */
		struct clk *clk = omap_clk_get_by_name("dss_fck");
		if (IS_ERR(clk))
			printk(KERN_ERR "can't get clock dss_fck\n");
		else
			clk_disable(clk);

		clk = omap_clk_get_by_name("dss_sys_clk");
		if (IS_ERR(clk))
			printk(KERN_ERR "can't get clock dss_sys_clk\n");
		else
			clk_disable(clk);

		clk = omap_clk_get_by_name("dss_dss_clk");
		if (IS_ERR(clk))
			printk(KERN_ERR "can't get clock dss_dss_clk\n");
		else
			clk_disable(clk);
	}
#endif

	if (!first_boot && !dssdev->phy.dsi.d2l_use_ulps) {
		if (dssdev->platform_enable) {
			ret = dssdev->platform_enable(dssdev);
			if (ret)
				goto err0;
		}
	}

	mapphone_hw_reset(dssdev);

	omapdss_dsi_vc_enable_hs(dssdev, dsi_vc_cmd, false);

	if (dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_VIDEO_MODE &&
	    !dssdev->skip_init) {
		//do extra job to match kozio registers
		dsi_videomode_panel_preinit(dssdev);
		//Need to wait a certain time - Toshiba Bridge Constraint
		msleep(1);
	}

	check_mapphone_panel_present(dssdev);
	if (panel_init_state == MAPPHONE_PANEL_NOT_PRESENT) {
		printk(KERN_WARNING "%s:Panel is not attached or \
				 failed to send BTA.\n", __func__);
		ret = -EINVAL;
		goto err0;
	}

	switch (dssdev->panel.panel_id) {
	case MOT_DISP_MIPI_CM_480_854:
	case MOT_DISP_MIPI_CM_370_480_854:
		ret = dsi_mipi_cm_480_854_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_CM_430_480_854:
		ret = dsi_mipi_430_cm_480_854_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_CM_310_320_480_1:
		ret = dsi_mipi_310_1_cm_320_480_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_CM_400_540_960:
	case MOT_DISP_MIPI_CM_430_540_960:
		ret = mapphone_panel_MIPI_540_960_detect(dssdev);
		break;
	case MOT_DISP_MIPI_CM_430_540_960_3:
		ret = dsi_mipi_cm_430_540_960_m3_v1_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_CM_430_540_960_AMOLED:
		ret = dsi_mipi_430_cm_540_960_amoled_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_VM_248_320_240:
		ret = dsi_mipi_248_vm_320_240_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_VM_280_320_240:
		ret = dsi_mipi_280_vm_320_240_panel_enable(dssdev);
		break;
	case MOT_DISP_MIPI_CM_450_720_1280:
		ret = dsi_mipi_cm_450_720_1280_panel_enable(dssdev);
		break;
	case MOT_DISP_LVDS_MIPI_VM_1007_1280_800:
		if (!dssdev->skip_init)
			ret = dsi_lvds_mipi_vm_1007_1280_800_panel_enable(dssdev);
		break;
	default:
		printk(KERN_ERR "unsupport panel =0x%lx\n",
						dssdev->panel.panel_id);
		ret = -EINVAL;
		goto err;
	}

	if (ret)
		goto err;

	panel_init_state = MAPPHONE_PANEL_INIT_DONE;

	/* The first time we enable the display, read the current power mode.
	 * This is done to ensure AP + display are in sync for the display's
	 * state, as the bootloader can change the display's initial state.
	 * This is needed to ensure ESD check is done correctly.
	 */
	if (first_boot) {
		if (panel_data->use_esd_check) {
			if (dsi_vc_dcs_read(dssdev, dsi_vc_cmd,
					EDISCO_CMD_GET_POWER_MODE,
					&power_mode, 1) != 1) {
				printk(KERN_ERR "Failed to read "
					"'get_power_mode' first time\n");
				ret = -EINVAL;
				goto err;
			}

			/*Set the panel state to on if the display reports
			 * it is already on
			 */
			if (power_mode & 0x04)
				atomic_set(&panel_data->state, PANEL_ON);
		}
		first_boot = false;
	}

	mp_data->enabled = true;
	mp_data->som_enabled = false;

	/*moved this out of interrupt*/
	mapphone_panel_display_on(dssdev);

	omapdss_dsi_vc_enable_hs(dssdev, dsi_vc_cmd, true);



	if (dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_VIDEO_MODE) {
		if (!dssdev->skip_init)
			dsi_video_mode_enable(dssdev, 0x3E);
		else
			dssdev->skip_init = false;
	} else {
		ret = mapphone_panel_enable_te_locked(dssdev,
							mp_data->te_enabled);
		if (ret)
			goto err;

		/*
		 * This is for command mode only, and is used so that the DSS
		 * clock reference count is 0 at the end of this function.
		 * This allows the DSS power domain to be OFF when not
		 * communicating with the display.
		 */
		dsi_from_dss_runtime_put(dssdev);
	}

	printk(KERN_INFO "Mapphone Display is ENABLE\n");
	return 0;
err:
	dev_err(&dssdev->dev, "error while enabling panel, issuing HW reset\n");

	panel_init_state = MAPPHONE_PANEL_INIT_FAILED;
	mapphone_hw_reset(dssdev);

	omapdss_dsi_display_disable(dssdev, true, dssdev->phy.dsi.d2l_use_ulps);
	/* clk is already disabled above, skip dsi_runtime_put() */
	return ret;
err0:
	if (panel_init_state == MAPPHONE_PANEL_UNDETERMINE)
		panel_init_state = MAPPHONE_PANEL_NOT_PRESENT;

	dsi_from_dss_runtime_put(dssdev);
	return ret;
}

static void set_vc_channels(struct omap_dss_device *dssdev)
{
	if ((dsi_vc_cmd == 0) && (dsi_vc_video == 0)) {
		switch (dssdev->panel.panel_id) {
#ifdef CONFIG_MACH_OMAP_MAPPHONE_DEFY
		// TODO Martin / Quarx: are the first two cases required?
		case MOT_DISP_MIPI_CM_480_854:
		case MOT_DISP_MIPI_CM_370_480_854:
#endif
		case MOT_DISP_LVDS_MIPI_VM_1007_1280_800:
			dsi_vc_cmd = 1;
			dsi_vc_video = 0;
			break;
		default:
			dsi_vc_cmd = 0;
			dsi_vc_video = 1;
			break;
		}

		DBG("dsi_vc_cmd = %d, dsi_vc_video = %d\n", dsi_vc_cmd, dsi_vc_video);
	}
}

#if 0
/* see mapphone_panel_driver struct declaration. */
static int mapphone_get_vc_channels(struct omap_dss_device *dssdev,
		u8 *dsi_vc_cmd_chnl, u8 *dsi_vc_video_chnl)
{
	if (dsi_vc_cmd_chnl == NULL || dsi_vc_video_chnl == NULL)
		return -1;

	if ((dsi_vc_cmd == 0) && (dsi_vc_video == 0))
		set_vc_channels(dssdev);

	*dsi_vc_cmd_chnl = dsi_vc_cmd;
	*dsi_vc_video_chnl = dsi_vc_video;

	return 0;
}
#endif

static int mapphone_panel_start(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;

	dsi_bus_lock(dssdev);

	r = mapphone_panel_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		if (panel_data->use_esd_check)
			queue_delayed_work(mp_data->esd_wq, &mp_data->esd_work,
				MAPPHONE_ESD_CHECK_PERIOD);
	}

	return r;
}

static int mapphone_panel_enable(struct omap_dss_device *dssdev)
{
	int r;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	DBG("enable\n");

	mutex_lock(&mp_data->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		printk(KERN_WARNING "panel state is not disabled, returns\n");
		r = -EINVAL;
		goto end;
	}

	mapphone_panel_print_config(dssdev);

	r = mapphone_panel_start(dssdev);
end:
	mutex_unlock(&mp_data->lock);
	return r;
}

static void mapphone_panel_disable_local(struct omap_dss_device *dssdev)
{
	u8 data[1];
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	atomic_set(&panel_data->state, PANEL_OFF);

	/*
	 * Change panel power down sequence to be aligned with spec.
	 * The cmd order has to be 10h  and then 28h, inverting them may
	 * cause exessive current according to spec.
	 * Note: might have sticking image, but did not see it yet.
	 */

	data[0] = EDISCO_CMD_SET_DISPLAY_OFF;
	dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, data, 1);

	data[0] = EDISCO_CMD_ENTER_SLEEP_MODE;
	dsi_vc_dcs_write_nosync(dssdev, dsi_vc_cmd, data, 1);

	msleep(120);
}

static void mapphone_panel_power_off(struct omap_dss_device *dssdev,
				bool secret)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	DBG("mapphone_panel_disable\n");

	if (dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_CMD_MODE)
		/*
		 * This is only needed for non video mode panels, as the
		 * clocks should already be on at this point for video mode.
		 * For video mode, the DSS clocks are disabled in the
		 * omapdss_dsi_display_disable() call below.
		 */
		dsi_from_dss_runtime_get(dssdev);

	if (mp_data->enabled)
		mapphone_panel_disable_local(dssdev);

	if (!secret)
		/*
		 * clk will be released in below after context
		 * is saved before enter RET
		 */
		omapdss_dsi_display_disable(dssdev, true,
			dssdev->phy.dsi.d2l_use_ulps);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	mp_data->enabled = false;
	mp_data->som_enabled = secret;

}

static void mapphone_panel_stop(struct omap_dss_device *dssdev, bool secret)
{

	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if (panel_data->use_esd_check)
		cancel_delayed_work(&mp_data->esd_work);

	dsi_bus_lock(dssdev);

	mapphone_panel_power_off(dssdev, secret);

	dsi_bus_unlock(dssdev);

}
static void mapphone_panel_disable(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	DBG("disable\n");

	mutex_lock(&mp_data->lock);
	/*TODO: clean up dssdev->state*/
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		mapphone_panel_stop(dssdev, false);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	mutex_unlock(&mp_data->lock);
}

static bool mapphone_panel_support_te(struct omap_dss_device *dssdev)
{
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if (!panel_data->te_support)
		return false;
	else
		return true;
}

static int amoled_cmoste_wr(struct omap_dss_device *dssdev)
{
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r = 0;
	u8 data[6];

	if ((panel_data->te_type == OMAP_DSI_TE_CMOS_TE_0) &&
		(panel_data->cmoste_wr == true)) {
		/*
		 *Issue: SMD amoled panel controller external TE pulse is too
		 *long(1.26ms) and not programmable, which cause tearing.
		 *This issue will be resolved by reducing TE pulse width
		 *in future phone HW.
		 *For unmodified HW, use below SW workaround to adjust amoled
		 *CMD controller TE pad output generating ~125us pulse from
		 *line240 to 255 by set TE_AP_OPT = 0,TE_AP_SRT = 240,
		 *TE_AP_END = 255,*VFP = 77
		 */
		data[0] = 0xf2;
		data[1]	= 0x00;
		data[2] = 0x06;
		data[3] = 0xf0;
		data[4] = 0xff;
		data[5] = 0x4d;
		r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 6);
	}

	return r;
}
static int mapphone_panel_enable_te_locked(struct omap_dss_device *dssdev,
					bool enable)
{
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	int r;
	u8 data[3];

	DBG("mapphone_panel_enable_te_locked: %d\n", enable);

	if (enable) {
		data[0] = EDISCO_CMD_SET_TEAR_ON;
		data[1] = 0x00;
		r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 2);
		if (r) {
			printk(KERN_ERR "Failed to send EDISCO_CMD_SET_TEAR_ON\n");
			goto error;
		}

		data[0] = EDISCO_CMD_SET_TEAR_SCANLINE;
		data[1] = (panel_data->te_scan_line & 0xff00) >> 8;
		data[2] = (panel_data->te_scan_line & 0xff);
		r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 3);
		if (r) {
			printk(KERN_ERR "Failed to send EDISCO_CMD_SET_TEAR_SCANLINE\n");
			goto error;
		}

		if (dssdev->panel.panel_id ==
			MOT_DISP_MIPI_CM_430_540_960_AMOLED) {
			r = amoled_cmoste_wr(dssdev);
			if (r)
				goto error;
		}

	} else {
		data[0] = EDISCO_CMD_SET_TEAR_OFF;
		data[1] = 0x00; // the 2.6.32 kernel has this 2 bytes long.
		r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, data, 2);

		if (r) {
			printk(KERN_ERR "Failed to send EDISCO_CMD_SET_TEAR_OFF: %d\n", r);
			goto error;
		}
	}

#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	r = omapdss_dsi_enable_te(dssdev, enable);
#else
	/* On 2.6.32 TE was always disabled. */
	r = omapdss_dsi_enable_te(dssdev, false);
#endif

error:
	return r;
}

static int mapphone_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	int r = 0;
	struct mapphone_data *map_data = dev_get_drvdata(&dssdev->dev);

	DBG("%s()\n", __func__);

	if (dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_CMD_MODE) {
		mutex_lock(&map_data->lock);

		if (!mapphone_panel_support_te(dssdev))
			enable = false;

		if (map_data->te_enabled != enable) {
			dsi_bus_lock(dssdev);
			dsi_from_dss_runtime_get(dssdev);

			if (map_data->som_enabled) {
				/*If in secret off, just save the new TE state*/
				map_data->te_enabled = enable;
				DBG("Changed TE state in SO,TE=%d\n", enable);
			} else {
				r = mapphone_panel_enable_te_locked(dssdev,
					 enable);
				if (!r) {
					map_data->te_enabled = enable;
					DBG("Changed TE state,TE=%d\n", enable);
				}
			}

			dsi_from_dss_runtime_put(dssdev);
			dsi_bus_unlock(dssdev);
		}

		mutex_unlock(&map_data->lock);
	}

	return r;
}

#if 0
/* see mapphone_panel_driver struct declaration. */
static bool mapphone_panel_manual_te_trigger(struct omap_dss_device *dssdev)
{
	struct mapphone_dsi_panel_data *panel_data = get_panel_data(dssdev);
	return panel_data->manual_te_trigger;
}
#endif

static int mapphone_panel_rotate(struct omap_dss_device *display, u8 rotate)
{
	return 0;
}

static int mapphone_panel_mirror(struct omap_dss_device *display, bool enable)
{
	return 0;
}

/* Todo:  should NOT show Vendor name, replace it with panel id later */
static int mapphone_panel_run_test(struct omap_dss_device *dssdev,
					int test_num)
{
	int r = -1; /* Returns -1 if no dssdev or test isn't supported */
		/* Returns 0 on success, or the test_num on failure */
	u8 data[CTL_SUPPLIER_ID_LEN];
	u16 id = 0xFFFF;

	if (!dssdev)
		return r;

	if (test_num == 1)	{
		if ((dssdev->panel.panel_id == MOT_DISP_MIPI_CM_430_480_854) ||
		    (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_370_480_854) ||
		    (dssdev->panel.panel_id == MOT_DISP_MIPI_CM_480_854)) {
			r = test_num;
			/* Status check to ensure communication */
			dsi_vc_set_max_rx_packet_size(dssdev, dsi_vc_cmd,
					CTL_SUPPLIER_ID_LEN);
			if (dsi_vc_dcs_read(dssdev, dsi_vc_cmd,
					EDISCO_CMD_READ_DDB_START,
					data, CTL_SUPPLIER_ID_LEN)
					> 0) {
				id = (data[0] << 8) | data[1];

				switch (dssdev->panel.panel_id) {
				case MOT_DISP_MIPI_CM_430_480_854:
					if (id == CTL_SUPPLIER_ID_AUO_43)
						r = 0;
					break;
				case MOT_DISP_MIPI_CM_370_480_854:
					if (id == CTL_SUPPLIER_ID_AUO)
						r = 0;
					break;
				case MOT_DISP_MIPI_CM_480_854:
					if (id == CTL_SUPPLIER_ID_TMD)
						r = 0;
					break;
				default:
					r = 0;
					break;
				}
			}
			dsi_vc_set_max_rx_packet_size(dssdev, dsi_vc_cmd, 1);
		} else {
			/* If check not supported, return success */
			r = 0;
		}
	}
	return r;
}

static int mapphone_panel_suspend(struct omap_dss_device *dssdev)
{
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	DBG("suspend\n");

	mutex_lock(&mp_data->lock);
	/*Todo: clean up dssdev state*/
	if ((dssdev->state == OMAP_DSS_DISPLAY_ACTIVE))
		mapphone_panel_stop(dssdev, false);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	mutex_unlock(&mp_data->lock);

	return 0;
}
static int mapphone_panel_resume(struct omap_dss_device *dssdev)
{
	int r;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&mp_data->lock);

	printk(KERN_INFO "panel resume.\n");
	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		printk(KERN_WARNING "panel state is not suspended, returns\n");
		r = -EINVAL;
		goto end;
	}

	r = mapphone_panel_start(dssdev);
	if (r)
		printk(KERN_ERR "mapphone_panel_start returns err = %d.\n" , r);
end:
	mutex_unlock(&mp_data->lock);
	return r;
}

static int mapphone_panel_set_update_mode(struct omap_dss_device *dssdev,
		enum omap_dss_update_mode mode)
{
	/* ToDo: this API is supposed to set update mode to auto or cmd mode */
	if (dssdev->phy.dsi.type == OMAP_DSS_DSI_TYPE_VIDEO_MODE) {
		/* auto mode is supported in video mode */
		if (mode == OMAP_DSS_UPDATE_AUTO)
			return 0;
	}
	/* since only cmd mode is supported in smart phone, it just returns */
	if (mode != OMAP_DSS_UPDATE_MANUAL)
		return -EINVAL;

	return 0;
}

static enum omap_dss_update_mode mapphone_panel_get_update_mode(
		struct omap_dss_device *dssdev)
{
	if (dssdev->panel.panel_id == MOT_DISP_LVDS_MIPI_VM_1007_1280_800)
		return OMAP_DSS_UPDATE_AUTO;
	else
		return OMAP_DSS_UPDATE_MANUAL;
}

#if 0
/* see mapphone_panel_driver struct declaration. */
static int mapphone_panel_reg_read(struct omap_dss_device *dssdev,
				u8 address, u16 size, u8 *buf,
				u8 use_hs_mode)
{
	int r = -1;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	DBG("read reg, address = 0x%X, size = %d\n", address, size);
	mutex_lock(&mp_data->lock);
	dsi_bus_lock(dssdev);
	dsi_from_dss_runtime_get(dssdev);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EIO;
		goto end;
	}

	if (use_hs_mode)
		omapdss_dsi_vc_enable_hs(dssdev, dsi_vc_cmd, true);
	else
		omapdss_dsi_vc_enable_hs(dssdev, dsi_vc_cmd, false);

	r = dsi_vc_set_max_rx_packet_size(dssdev, dsi_vc_cmd, size);
	if (r != 0)
		goto end;

	r = dsi_vc_dcs_read(dssdev, dsi_vc_cmd, address, buf, size);
	if (r != size)
		goto end;

	r = dsi_vc_set_max_rx_packet_size(dssdev, dsi_vc_cmd, 1);
	if (r != 0)
		goto end;

	r = 0;

end:
	dsi_from_dss_runtime_put(dssdev);
	dsi_bus_unlock(dssdev);
	mutex_unlock(&mp_data->lock);
	DBG("read reg done, r = %d\n", r);
	return r;
}

static int mapphone_panel_reg_write(struct omap_dss_device *dssdev,
				u16 size, u8 *buf, u8 use_hs_mode)
{
	int r = -1;
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	DBG("write reg, size = %d\n", size);
	mutex_lock(&mp_data->lock);
	dsi_bus_lock(dssdev);
	dsi_from_dss_runtime_get(dssdev);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EIO;
		goto end;
	}

	if (use_hs_mode)
		omapdss_dsi_vc_enable_hs(dssdev, dsi_vc_cmd, true);
	else
		omapdss_dsi_vc_enable_hs(dssdev, dsi_vc_cmd, false);
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, size);

end:
	dsi_from_dss_runtime_put(dssdev);
	dsi_bus_unlock(dssdev);
	mutex_unlock(&mp_data->lock);
	DBG("write reg done, r = %d\n", r);
	return r;
}
#endif

static u8 amoled_bl_data_ws[][26] = {
	{0xfa, 0x02, 0x20, 0x00, 0x20, 0xa0, 0x00, 0xa0, 0xd2, 0xa0,
	 0xd2, 0xd9, 0xd3, 0xd4, 0xb7, 0xbe, 0xb1, 0xc8, 0xd2, 0xc6,
	 0x00, 0x69, 0x00, 0x55, 0x00, 0x7a}, /*70 */
	{0xfa, 0x02, 0x20, 0x00, 0x20, 0xba, 0x00, 0xc7, 0xcc, 0xc0,
	 0xc8, 0xd4, 0xd8, 0xd2, 0xb3, 0xbd, 0xad, 0xc3, 0xcd, 0xc0,
	 0x00, 0x88, 0x00, 0x71, 0x00, 0xa0}, /* 150 */
	{0xfa, 0x02, 0x20, 0x00, 0x20, 0xca, 0x00, 0xca, 0xcb, 0xc8,
	 0xc8, 0xd1, 0xd7, 0xce, 0xae, 0xba, 0xa9, 0xc0, 0xca, 0xbc,
	 0x00, 0x9d, 0x00, 0x83, 0x00, 0xb8}, /* 220 */
	{0xfa, 0x02, 0x20, 0x20, 0x20, 0xda, 0x9c, 0xd9, 0xc4, 0xbb,
	 0xc0, 0xd1, 0xcf, 0xcf, 0xaa, 0xa9, 0xa5, 0xbe, 0xbf, 0xba,
	 0x00, 0xaf, 0x00, 0x93, 0x00, 0xcf} /* 300 */
};

static u8 amoled_bl_data_cs_03[][26] = {
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0x9E, 0xEE, 0xD5, 0x93,
	 0xC7, 0xE3, 0xC1, 0xDE, 0xB9, 0x96, 0xC1, 0xD9, 0xCB, 0xDE,
	 0x00, 0x44, 0x00, 0x2C, 0x00, 0x4D}, /* 10 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0x9E, 0xEE, 0xD5, 0x93,
	 0xC7, 0xE3, 0xC1, 0xDE, 0xB9, 0x9E, 0xC1, 0xD7, 0xCF, 0xDA,
	 0x00, 0x51, 0x00, 0x39, 0x00, 0x5A}, /* 20 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD5, 0x97,
	 0xC7, 0xE3, 0xC1, 0xDE, 0xB9, 0xA5, 0xC3, 0xD5, 0xD0, 0xD4,
	 0x00, 0x5B, 0x00, 0x42, 0x00, 0x64}, /* 30 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD5, 0x97,
	 0xC7, 0xE3, 0xD1, 0xDE, 0xB8, 0xAC, 0xC3, 0xD3, 0xD0, 0xCF,
	 0x00, 0x6A, 0x00, 0x50, 0x00, 0x76}, /* 50 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD6, 0xA6,
	 0xC7, 0xE4, 0xD6, 0xE1, 0xB6, 0xAF, 0xBF, 0xD1, 0xD0, 0xCD,
	 0x00, 0x74, 0x00, 0x59, 0x00, 0x83}, /* 70 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD7, 0xB3,
	 0xC8, 0xE4, 0xD7, 0xE4, 0xB6, 0xB0, 0xBC, 0xCF, 0xCF, 0xCC,
	 0x00, 0x7A, 0x00, 0x5E, 0x00, 0x89}, /* 80 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD7, 0xB6,
	 0xC9, 0xE3, 0xD7, 0xE5, 0xB5, 0xB2, 0xBB, 0xCE, 0xCE, 0xCB,
	 0x00, 0x7F, 0x00, 0x62, 0x00, 0x8E}, /* 90 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD4, 0xB5,
	 0xC6, 0xE4, 0xD8, 0xE5, 0xB4, 0xB1, 0xB9, 0xCD, 0xCD, 0xC9,
	 0x00, 0x83, 0x00, 0x65, 0x00, 0x93}, /* 100 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD5, 0xB9,
	 0xC7, 0xE4, 0xD9, 0xE6, 0xB2, 0xB2, 0xB7, 0xCD, 0xCC, 0xC7,
	 0x00, 0x87, 0x00, 0x68, 0x00, 0x98}, /* 110 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD5, 0xBB,
	 0xC7, 0xE4, 0xD9, 0xE6, 0xB2, 0xB1, 0xB6, 0xCD, 0xCC, 0xC7,
	 0x00, 0x8A, 0x00, 0x6C, 0x00, 0x9C}, /* 120 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD5, 0xBC,
	 0xCB, 0xE4, 0xD9, 0xE8, 0xB2, 0xB1, 0xB2, 0xCC, 0xCC, 0xC5,
	 0x00, 0x8E, 0x00, 0x6F, 0x00, 0xA2}, /* 130 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD6, 0xBC,
	 0xCD, 0xE2, 0xDB, 0xE5, 0xB2, 0xB1, 0xB2, 0xCC, 0xCB, 0xC5,
	 0x00, 0x91, 0x00, 0x72, 0x00, 0xA6}, /* 140 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD4, 0xBF,
	 0xCC, 0xE3, 0xDB, 0xE5, 0xB1, 0xB1, 0xB1, 0xCA, 0xCB, 0xC5,
	 0x00, 0x95, 0x00, 0x74, 0x00, 0xA9}, /* 150 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD5, 0xC1,
	 0xCC, 0xE3, 0xDC, 0xE5, 0xB1, 0xB0, 0xB0, 0xC9, 0xCB, 0xC3,
	 0x00, 0x98, 0x00, 0x77, 0x00, 0xAE}, /* 160 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD6, 0xC2,
	 0xCF, 0xE2, 0xDB, 0xE4, 0xB0, 0xB0, 0xAF, 0xC9, 0xCA, 0xC2,
	 0x00, 0x9B, 0x00, 0x7A, 0x00, 0xB2}, /* 170 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD5, 0xC2,
	 0xCD, 0xE2, 0xDC, 0xE4, 0xB0, 0xB0, 0xAE, 0xC8, 0xCA, 0xC2,
	 0x00, 0x9E, 0x00, 0x7C, 0x00, 0xB5}, /* 180 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD4, 0xC3,
	 0xCF, 0xE3, 0xDB, 0xE4, 0xAF, 0xB0, 0xAD, 0xC9, 0xC9, 0xC2,
	 0x00, 0xA0, 0x00, 0x7F, 0x00, 0xB8}, /* 190 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD4, 0xC7,
	 0xD1, 0xE2, 0xDB, 0xE2, 0xAE, 0xB1, 0xAD, 0xC8, 0xC8, 0xC0,
	 0x00, 0xA3, 0x00, 0x81, 0x00, 0xBC}, /* 200 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD3, 0xC4,
	 0xD0, 0xE3, 0xDD, 0xE3, 0xAE, 0xB0, 0xAC, 0xC7, 0xC8, 0xBE,
	 0x00, 0xA6, 0x00, 0x83, 0x00, 0xC0}, /* 210 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD3, 0xC5,
	 0xD2, 0xE2, 0xDD, 0xE2, 0xAE, 0xAF, 0xA9, 0xC5, 0xC8, 0xBE,
	 0x00, 0xAC, 0x00, 0x87, 0x00, 0xC7}, /* 230 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD3, 0xC7,
	 0xD2, 0xE1, 0xDD, 0xE1, 0xAE, 0xAF, 0xA9, 0xC5, 0xC6, 0xBE,
	 0x00, 0xAF, 0x00, 0x8B, 0x00, 0xCB}, /* 250 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD2, 0xC8,
	 0xD2, 0xE1, 0xDE, 0xE0, 0xAC, 0xAF, 0xA8, 0xC5, 0xC5, 0xBB,
	 0x00, 0xB4, 0x00, 0x8F, 0x00, 0xD3}, /* 270 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD3, 0xC9,
	 0xD3, 0xE1, 0xDD, 0xDF, 0xAC, 0xAF, 0xA7, 0xC3, 0xC6, 0xBB,
	 0x00, 0xB8, 0x00, 0x91, 0x00, 0xD7}, /* 290 nits */
	{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB4, 0xEE, 0xD4, 0xC9,
	 0xD6, 0xE1, 0xDE, 0xDE, 0xAC, 0xAF, 0xA7, 0xC3, 0xC5, 0xBB,
	 0x00, 0xB9, 0x00, 0x93, 0x00, 0xD9} /* 300 nits */
};

static int dsi_mipi_cm_430_540_960_amoled_bl_set_locked(
	struct omap_dss_device *dssdev, int level)
{
	int r = 0;
	u8 buf[2] = {0xfa, 0x03};
	u8 (*bl_data)[26];
	int index;
	int range;
	int num_bl_steps;
	static u16 controller_ver = INVALID_VALUE;

	if (controller_ver == INVALID_VALUE) {
		controller_ver = read_panel_controller_version(dssdev);
		if (controller_ver == INVALID_VALUE) {
			printk(KERN_ERR "Failed to detect panel, controller_ver =%d\n",
					controller_ver);
			return -EINVAL;
		}
	}

	if (controller_ver == 0x00) {
		bl_data = amoled_bl_data_ws;
		num_bl_steps = (sizeof(amoled_bl_data_ws) /
				sizeof(amoled_bl_data_ws[0]));
	} else {
		/* Re-use all of the p1c gamma settings */
		bl_data = amoled_bl_data_cs_03;
		num_bl_steps = (sizeof(amoled_bl_data_cs_03) /
				sizeof(amoled_bl_data_cs_03[0]));
	}

	if (level < 0)
		level = 0;
	else if (level > MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX)
		level = MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX;

	range = (MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX /
		num_bl_steps) + 1;

	index = level / range;

	if (index >= num_bl_steps)
		index = num_bl_steps - 1;

	DBG("Set 430_540_960_amoled bl, level = %d, index = %d\n",
		level, index);
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, bl_data[index],
			sizeof(bl_data[index]));
	if (r)
		goto err;

	/* Gamma set update enable */
	r = dsi_vc_dcs_write(dssdev, dsi_vc_cmd, buf, sizeof(buf));
	if (r)
		goto err;

	return 0;

err:
	printk(KERN_ERR "Failed to set AMOLED BL index = %d, r = %d\n",
		index, r);
	return r;
}

static int dsi_mipi_cm_430_540_960_amoled_bl_get_brightness(
	struct backlight_device *bl)
{
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	/* Return 0 if backlight is off */
	return 0;
}

static int dsi_mipi_cm_430_540_960_amoled_bl_update_status(
	struct backlight_device *bl)
{
	int r = -1;
	int level;
	struct omap_dss_device *dssdev = dev_get_drvdata(&bl->dev);
	struct mapphone_data *mp_data = dev_get_drvdata(&dssdev->dev);

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	mutex_lock(&mp_data->lock);
	if (!mp_data->enabled) {
		r = 0;
	} else {
		dsi_bus_lock(dssdev);
		dsi_from_dss_runtime_get(dssdev);
		r = dsi_mipi_cm_430_540_960_amoled_bl_set_locked(dssdev,
								level);
		dsi_from_dss_runtime_put(dssdev);
		dsi_bus_unlock(dssdev);
	}
	mutex_unlock(&mp_data->lock);

	return r;
}

static const struct backlight_ops dsi_mipi_cm_430_540_960_amoled_bl_ops = {
	.get_brightness = dsi_mipi_cm_430_540_960_amoled_bl_get_brightness,
	.update_status = dsi_mipi_cm_430_540_960_amoled_bl_update_status,
};

static int dsi_mipi_cm_430_540_960_amoled_bl_probe(
	struct omap_dss_device *dssdev, struct mapphone_data *mp_data)
{
	struct backlight_properties bl_props;
	struct backlight_device *bl;
	int r = 0;

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.max_brightness =
		MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX;
	bl_props.type = BACKLIGHT_RAW;
	bl = backlight_device_register("430_540_960_amoled_bl",
				&dssdev->dev, dssdev,
				&dsi_mipi_cm_430_540_960_amoled_bl_ops,
				&bl_props);
	if (IS_ERR(bl)) {
		r = PTR_ERR(bl);
		return r;
	}
	mp_data->bldev = bl;
	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.brightness =
		MOT_DSIP_MIPI_CM_430_540_960_AMOLED_BL_MAX;
	return r;
}


static struct omap_dss_driver mapphone_panel_driver = {
	.probe		= mapphone_panel_probe,
	.remove		= mapphone_panel_remove,

	.enable		= mapphone_panel_enable,
	.disable	= mapphone_panel_disable,
	.suspend	= mapphone_panel_suspend,
	.resume		= mapphone_panel_resume,

	.set_update_mode	= mapphone_panel_set_update_mode,
	.get_update_mode	= mapphone_panel_get_update_mode,

	.update		= mapphone_panel_update,
	.sync		= mapphone_panel_sync,

	.get_resolution		= mapphone_panel_get_resolution,
	.get_recommended_bpp	= omapdss_default_get_recommended_bpp,

	/*.hs_mode_timing		= mapphone_panel_get_hs_mode_timing,*/
	.enable_te		= mapphone_panel_enable_te,
	.get_te			= mapphone_panel_get_te,

	.set_rotate		= mapphone_panel_rotate,
	.get_rotate		= mapphone_panel_get_rotate,
	.set_mirror		= mapphone_panel_mirror,
	.run_test		= mapphone_panel_run_test,
	.memory_read		= mapphone_panel_memory_read,
	.get_timings		= mapphone_panel_get_timings,
	.set_timings		= mapphone_panel_set_timings,
	.check_timings		= mapphone_panel_check_timings,

#if 0
	/*
	 * These are not supported in the DSS version in 3.0.8,
	 * but they might be interesting later ;-).
	 * Then you may also need to remove the #ifdef
	 * in mapphone_framedone_cb!
	 */
	.framedone		= mapphone_panel_display_on,
	.support_te		= mapphone_panel_support_te,
	.manual_te_trigger	= mapphone_panel_manual_te_trigger,
	.reg_read		= mapphone_panel_reg_read,
	.reg_write		= mapphone_panel_reg_write,
	.get_dsi_vc_chnls	= mapphone_get_vc_channels,
#endif

	.driver = {
		.name = "mapphone-panel",
		.owner = THIS_MODULE,
	},
};


static int __init mapphone_panel_init(void)
{
	DBG("mapphone_panel_init\n");

	omap_dss_register_driver(&mapphone_panel_driver);

	return i2c_add_driver(&lvds_panel_driver);
}

static void __exit mapphone_panel_exit(void)
{
	DBG("mapphone_panel_exit\n");
	omap_dss_unregister_driver(&mapphone_panel_driver);
	i2c_del_driver(&lvds_panel_driver);
}

module_init(mapphone_panel_init);
module_exit(mapphone_panel_exit);

MODULE_AUTHOR("Rebecca Schultz Zavin <rebecca@android.com>");
MODULE_DESCRIPTION("Sholes Panel Driver");
MODULE_LICENSE("GPL");
