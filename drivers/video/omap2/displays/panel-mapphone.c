#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/sched.h>

#include <plat/display.h>
#include <plat/dma.h>
#include <asm/atomic.h>

#include <mach/dt_path.h>
#include <asm/prom.h>

#include <plat/panel.h>

#ifndef CONFIG_ARM_OF
#error CONFIG_ARM_OF must be defined for Mapphone to compile
#endif
#ifndef CONFIG_USER_PANEL_DRIVER
#error CONFIG_USER_PANEL_DRIVER must be defined for Mapphone to compile
#endif

#ifdef DEBUG
#define DBG(format, ...) (printk(KERN_DEBUG "mapphone-panel: " format, \
				## __VA_ARGS__))
#else
#define DBG(format, ...)
#endif

#define EDISCO_CMD_SOFT_RESET		0x01
#define EDISCO_CMD_GET_POWER_MODE       0x0A
#define EDISCO_CMD_ENTER_SLEEP_MODE	0x10
#define EDISCO_CMD_EXIT_SLEEP_MODE	0x11
#define EDISCO_CMD_SET_DISPLAY_ON	0x29
#define EDISCO_CMD_SET_DISPLAY_OFF	0x28
#define EDISCO_CMD_SET_COLUMN_ADDRESS	0x2A
#define EDISCO_CMD_SET_PAGE_ADDRESS	0x2B
#define EDISCO_CMD_SET_TEAR_OFF		0x34
#define EDISCO_CMD_SET_TEAR_ON		0x35
#define EDISCO_CMD_SET_TEAR_SCANLINE	0x44
#define EDISCO_CMD_READ_SCANLINE	0x45
#define EDISCO_CMD_READ_DDB_START	0xA1
#define EDISCO_CMD_SET_MCS		0xB2
#define EDISCO_CMD_SET_DISPLAY_MODE     0xB3
#define EDISCO_CMD_SET_BCKLGHT_PWM	0xB4
#define EDISCO_CMD_DATA_LANE_CONFIG	0xB5
#define EDISCO_CMD_SET_INVERSION_OFF	0x20
#define EDISCO_CMD_SET_INVERSION_ON	0x21
#define EDISCO_CMD_READ_DISPLAY_IMAGE_MODE	0x0D

#define EDISCO_CMD_DATA_LANE_ONE	0x0
#define EDISCO_CMD_DATA_LANE_TWO	0x1
#define EDISCO_CMD_SLEEP_MODE_OUT       0x10

#define EDISCO_CMD_MCS_ON              0x3
#define EDISCO_CMD_MCS_OFF             0x0

#define EDISCO_LONG_WRITE	0x29
#define EDISCO_SHORT_WRITE_1	0x23
#define EDISCO_SHORT_WRITE_0	0x13

#define EDISCO_CMD_VC   1
#define EDISCO_VIDEO_VC 0

#define DCS_CMD_RETRY_MAX 10

#define PANEL_OFF     0x0
#define PANEL_ON      0x1

/* DDB Supplier IDs used for run_test(1) */
#define SUPPLIER_ID_LEN	2
#define SUPPLIER_ID_AUO 0x0186
#define SUPPLIER_ID_AUO_43 0x0126	/* Shadow AUO panel reports 0x126 */
#define SUPPLIER_ID_TMD 0x0126
#define SUPPLIER_ID_INVALID 0xFFFF

#define DISP_480_854_CM_TE_SCANLINE    0x80;

/* this must be match with schema.xml section "device-id-value" */
#define MOT_DISP_MIPI_480_854_CM   	0x000a0001
#define MOT_DISP_430_MIPI_480_854_CM	0x001a0000
#define MOT_DISP_370_MIPI_480_854_CM	0x001a0001
#define MOT_DISP_248_MIPI_320_240_VM	0x00090002
#define MOT_DISP_280_MIPI_320_240_VM	0x00090003
#define MOT_DISP_310_1_MIPI_320_480_CM	0x001f0000
#define MOT_DISP_310_2_MIPI_320_480_CM	0x000a0003

static bool mapphone_panel_device_read_dt;

/* Display Control Hook */
extern u8 display_brightness(void);

/* these enum must be matched with MOT DT */
enum omap_dss_device_disp_pxl_fmt {
	OMAP_DSS_DISP_PXL_FMT_RGB565	= 1,
	OMAP_DSS_DISP_PXL_FMT_RGB888	= 5
};

static struct omap_video_timings mapphone_panel_timings = {
	.x_res          = 480,
	.y_res          = 854,
	/*.pixel_clock  = 25000,*/
	.dsi1_pll_fclk	= 100000,
	.dsi2_pll_fclk  = 100000,
	.hfp            = 0,
	.hsw            = 2,
	.hbp            = 2,
	.vfp            = 0,
	.vsw            = 1,
	.vbp            = 1,
	/* defaults for dpi. overide in device_tree.xml only if required on product basis */
	.w		= 46,
	.h		= 82,
};

struct mapphone_data {
	struct omap_dss_device *dssdev;
	atomic_t state;
	void *panel_handle;

	bool manual_te_trigger;
	u32 te_scan_line;
	unsigned long disp_init_delay;
};

static void mapphone_panel_disable_local(struct omap_dss_device *dssdev);

static bool mapphone_panel_sw_te_sup(struct omap_dss_device *dssdev)
{
	bool sw_te = false;

	switch (dssdev->panel.panel_id) {
	case MOT_DISP_310_1_MIPI_320_480_CM:
		sw_te = true;
		break;
	default:
		sw_te = false;
		break;
	}

	return sw_te;
}

static int mapphone_panel_get_scl_setting(struct omap_dss_device *dssdev)
{
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;
	return map_data->te_scan_line;
}

static int mapphone_panel_read_scl(struct omap_dss_device *dssdev)
{
	int scan_line = 0xffff;
	u8 data[2];

	data[0] = 0x0;
	data[1] = 0x0;

	if (dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC, 2))
		goto end;

	if (dsi_vc_dcs_read(EDISCO_CMD_VC,
			EDISCO_CMD_READ_SCANLINE, data, 2) == 2)
		scan_line = (data[0] << 8) | data[1];
	else
		printk(KERN_ERR "failed to read scan_line \n");

	dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC, 1);
end:
	return scan_line;
}

static void set_delay_timer(struct omap_dss_device *dssdev, unsigned long delay)
{
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;
	map_data->disp_init_delay = jiffies + msecs_to_jiffies(delay);
}

static void check_delay_timer(struct omap_dss_device *dssdev)
{
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;
	unsigned long jiff_time = 0;

	/*
	 * Delay if necessary, before calling EDISCO commands after
	 * EDISCO_CMD_EXIT_SLEEP
	 */
	if (map_data->disp_init_delay) {
		jiff_time = jiffies;
		if (map_data->disp_init_delay > jiff_time)
			mdelay(jiffies_to_msecs(map_data->disp_init_delay -
					jiff_time));
		map_data->disp_init_delay = 0;
	}
}

static int dsi_mipi_vm_panel_on(struct omap_dss_device *dssdev)
{
	u8 data = EDISCO_CMD_SET_DISPLAY_ON;
	int ret;

	dsi_disable_vid_vc_enable_cmd_vc();
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, &data, 1);
	dsi_disable_cmd_vc_enable_vid_vc();

	return ret;
}

static int dsi_mipi_cm_panel_on(struct omap_dss_device *dssdev)
{
	u8 data = EDISCO_CMD_SET_DISPLAY_ON;

	check_delay_timer(dssdev);

	return dsi_vc_dcs_write(EDISCO_CMD_VC, &data, 1);
}


static int mapphone_panel_display_on(struct omap_dss_device *dssdev)
{
	int ret = 0;

	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;

	if (atomic_cmpxchg(&map_data->state, PANEL_OFF, PANEL_ON) ==
						PANEL_OFF) {
		switch (dssdev->panel.panel_id) {
		case MOT_DISP_248_MIPI_320_240_VM:
		case MOT_DISP_280_MIPI_320_240_VM:
			ret = dsi_mipi_vm_panel_on(dssdev);
			break;
		case MOT_DISP_MIPI_480_854_CM:
		case MOT_DISP_370_MIPI_480_854_CM:
		case MOT_DISP_430_MIPI_480_854_CM:
		case MOT_DISP_310_1_MIPI_320_480_CM:
		case MOT_DISP_310_2_MIPI_320_480_CM:
			ret = dsi_mipi_cm_panel_on(dssdev);
			break;
		default:
			printk(KERN_ERR "unsupport panel =0x%lx \n",
			dssdev->panel.panel_id);
			ret = -EINVAL;
		}

		if (ret == 0)
			printk(KERN_INFO "Panel is turned on \n");
	}

	return ret;
}

static int mapphone_panel_dt_panel_probe(struct omap_dss_device *dssdev,
						int *pixel_size)
{
	struct device_node *panel_node;
	const void *panel_prop;
	int panel_pixel_fmt;
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;

	if (mapphone_panel_device_read_dt == true)
		printk("\nmapphone_panel_device_read_dt =true");

	DBG("dt_panel_probe\n");

	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node != NULL) {
		/* Retrieve the panel DSI timing */
		panel_prop = of_get_property(panel_node, "width", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.x_res = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node, "height", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.y_res = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hfp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hfp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hsw", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hsw = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hbp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hbp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vfp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vfp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vsw", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vsw = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vbp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vbp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"te_scan_line", NULL);
		if (panel_prop != NULL)
			map_data->te_scan_line = *(u32 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"phy_width_mm", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.w = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"phy_height_mm", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.h = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node, "pixel_fmt", NULL);
		if (panel_prop != NULL) {
			panel_pixel_fmt = *(u32 *)panel_prop;
			if (panel_pixel_fmt == OMAP_DSS_DISP_PXL_FMT_RGB888)
				*pixel_size = 24;
			else if (panel_pixel_fmt ==
					OMAP_DSS_DISP_PXL_FMT_RGB565)
				*pixel_size = 16;
			else {
				printk(KERN_ERR " Invalid panel_pxl_fmt=%d",
						panel_pixel_fmt);
				return -ENODEV;
			}
		}

		of_node_put(panel_node);

		mapphone_panel_device_read_dt = true;
	}

	return panel_node ? 0 : -ENODEV;

}


static void panel_print_dt(struct omap_dss_device *dssdev)
{
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;

	printk(KERN_DEBUG "DT: width= %d height= %d\n",
		mapphone_panel_timings.x_res, mapphone_panel_timings.y_res);

	printk(KERN_DEBUG "DT: hfp=%d hsw=%d hbp=%d vfp=%d vsw=%d vbp=%d\n",
		mapphone_panel_timings.hfp, mapphone_panel_timings.hsw,
		mapphone_panel_timings.hbp, mapphone_panel_timings.vfp,
		mapphone_panel_timings.vsw, mapphone_panel_timings.vbp);

	printk(KERN_DEBUG "DT: clk_lane=%d clk_pos=%d  "
			"data1_lane=%d data1_pos=%d\n",
		dssdev->phy.dsi.clk_lane, dssdev->phy.dsi.clk_pol,
		dssdev->phy.dsi.data1_lane, dssdev->phy.dsi.data1_pol);

	printk(KERN_DEBUG "DT: data2_lane= %d data2_pos= %d xfer_mode= %d \n",
		dssdev->phy.dsi.data2_lane, dssdev->phy.dsi.data2_pol,
		dssdev->phy.dsi.xfer_mode);

	printk(KERN_DEBUG "DT: gpio_reset=%d panel_id=0x%lx"
				" te_scan_line=0x%x\n",
		dssdev->reset_gpio, dssdev->panel.panel_id,
		map_data->te_scan_line);

	printk(KERN_DEBUG "DT: regn=%d regm=%d regm3=%d regm4=%d"
		" lp_clk_div=%d lck_div=%d pck_div=%d \n",
		dssdev->phy.dsi.div.regn, dssdev->phy.dsi.div.regm,
		dssdev->phy.dsi.div.regm3, dssdev->phy.dsi.div.regm4,
		dssdev->phy.dsi.div.lp_clk_div, dssdev->phy.dsi.div.lck_div,
		dssdev->phy.dsi.div.pck_div);
}

static u16 mapphone_panel_read_supplier_id(void)
{
	static u16 id = SUPPLIER_ID_INVALID;
	u8 data[2];

	if (id != SUPPLIER_ID_INVALID)
		goto end;

	if (dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC, 2))
		goto end;

	if (dsi_vc_dcs_read(EDISCO_CMD_VC,
			    EDISCO_CMD_READ_DDB_START, data, 2) == 2)
		id = (data[0] << 8) | data[1];

	dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC, 1);

end:
	DBG("dsi_read_supplier_id() - supplier id [%hu]\n", id);

	return id;
}

static ssize_t mapphone_panel_supplier_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	char *supplier_str = "INVALID";

	dsi_bus_lock();

	switch (mapphone_panel_read_supplier_id()) {
	case SUPPLIER_ID_AUO:
		supplier_str = "AUO";
		break;
	case SUPPLIER_ID_TMD:
		supplier_str = "TMD";
		break;
	case SUPPLIER_ID_INVALID:
	default:
		break;
	}

	dsi_bus_unlock();

	return sprintf(buf, "%s\n", supplier_str);
}

static DEVICE_ATTR(supplier_name, S_IRUGO, mapphone_panel_supplier_name_show, NULL);

static ssize_t mapphone_panel_supplier_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;

	dsi_bus_lock();
	ret = sprintf(buf, "%d\n", mapphone_panel_read_supplier_id());
	dsi_bus_unlock();
	return ret;
}

static DEVICE_ATTR(supplier_id, S_IRUGO, mapphone_panel_supplier_id_show, NULL);

static int mapphone_panel_probe(struct omap_dss_device *dssdev)
{
	int error;
	int pixel_size = 24;
	struct mapphone_data *data;
	struct omap_panel_device panel_dev;

	DBG("probe\n");

	data = kmalloc(sizeof(struct mapphone_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memset(data, 0, sizeof(data));

	data->te_scan_line = 0x300;  /* default value for WVGA display */
	data->dssdev = dssdev;
	dssdev->data = data;

	if (mapphone_panel_dt_panel_probe(dssdev, &pixel_size))
		printk(KERN_INFO "panel: using non-dt configuration\n");

	panel_print_dt(dssdev);
	dssdev->ctrl.pixel_size = pixel_size;
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = mapphone_panel_timings;

	strncpy(panel_dev.name, dssdev->name, OMAP_PANEL_MAX_NAME_SIZE);
	panel_dev.fod_disable = mapphone_panel_disable_local;
	panel_dev.dssdev = dssdev;
	data->panel_handle = omap_panel_register(&panel_dev);
	if (data->panel_handle == NULL) {
		printk(KERN_ERR "Panel Register Failed\n");
		goto freedata;
	}

	error = device_create_file(&panel_dev.dssdev->dev,
		&dev_attr_supplier_id);
	if (error < 0) {
		pr_err("%s: device supplier_id create failed: %d\n",
			__func__, error);
		goto unregister;
	}

	error = device_create_file(&panel_dev.dssdev->dev,
		&dev_attr_supplier_name);
	if (error < 0) {
		pr_err("%s: device supplier_name create failed: %d\n",
			__func__, error);
		goto removeattr;
	}

	atomic_set(&data->state, PANEL_OFF);

	return 0;

removeattr:
	device_remove_file(&panel_dev.dssdev->dev, &dev_attr_supplier_id);
unregister:
	omap_panel_unregister(data->panel_handle);
freedata:
	kfree(data);
	data = NULL;

	return -ENODEV;
}

static void mapphone_panel_remove(struct omap_dss_device *dssdev)
{
	void *handle;
	struct mapphone_data *data = (struct mapphone_data *) dssdev->data;

	handle = data->panel_handle;
	omap_panel_unregister(handle);

	kfree(dssdev->data);
	return;
}

/* - In the LP mode, some panels have problems to receive command correctly
 * so we will send command out and read it back to make sure the write
 * command is accepted
 * - if the dsi_vc_dcs_write() request, then we will not care about the
 * write_dt (data type) */
static int mapphone_panel_lp_cmd_wrt_sync(bool dcs_cmd, int write_dt,
					u8 *write_data, int write_len,
					int read_cmd, int read_len,
					int chk_val, int chk_mask)
{
	int i, ret;
	u8 data[7];

	for (i = 0; i < DCS_CMD_RETRY_MAX; i++) {
		if (dcs_cmd == true) {
			ret = dsi_vc_dcs_write(EDISCO_CMD_VC,
						write_data, write_len);
			DBG("call dsi_vc_dcs_write"
				"(len=0%d, p1/p2/p3/p4=0x%x/0x%x/0x%x/0x%x)\n",
				write_len, write_data[0],
				write_data[1], write_data[2], write_data[3]);
		} else {
			ret = dsi_vc_write(EDISCO_CMD_VC, write_dt,
						write_data, write_len);
			DBG("call dsi_vc_write"
				"(dt=0x%x len=%d, p1/p2/p3/p4 = "
				"0x%x/0x%x/0x%x/0x%x)\n",
				write_dt, write_len, write_data[0],
				write_data[1], write_data[2], write_data[3]);
		}

		if (ret) {
			printk(KERN_ERR "failed to send cmd=0x%x \n",
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
		ret = dsi_vc_dcs_read(EDISCO_CMD_VC, read_cmd,
						&data[0], read_len);

		DBG("read_chk_cmd dcs_cmd=%d read_cmd=0x%x "
				"read_len=%d chk_val=0x%x chk_mask=0x%x "
				"read_val=0x%x \n",
				dcs_cmd, read_cmd, read_len, chk_val,
				chk_mask, data[0]);

		if (ret < 0)
			DBG("fail to read 0x%x cmd and "
					"will try it again \n", read_cmd);

		if ((data[0] & chk_mask) == chk_val) {
			/* break if read back the same writing value*/
			ret  = 0;
			break;
		}
	}

	if (i >= DCS_CMD_RETRY_MAX) {
		printk(KERN_ERR "failed to read dcs_cmd=%d read_cmd=0x%x "
				"read_len=%d chk_val=0x%x chk_mask=0x%x\n"
				"read_val=0x%x \n",
				dcs_cmd, read_cmd, read_len, chk_val,
				chk_mask, data[0]);
		ret = -1;
	}

	return ret;
}

static int dsi_mipi_248_vm_320_240_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;

	DBG(" dsi_mipi_248_vm_320_240_panel_enable() \n");

	/* turn it on */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);
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

	DBG(" dsi_mipi_280_vm_320_240_panel_enable() \n");

	/* turn off mcs register access protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1, data, 2);

	/* Internal display set up */
	data[0] = 0xC0;
	data[1] = 0x11;
	data[2] = 0x04;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 3);

	/* Internal voltage set up */
	data[0] = 0xD3;
	data[1] = 0x1F;
	data[2] = 0x01;
	data[3] = 0x02;
	data[4] = 0x15;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 5);

	/* Internal voltage set up */
	data[0] = 0xD4;
	data[1] = 0x62;
	data[2] = 0x1E;
	data[3] = 0x00;
	data[4] = 0xB7;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 5);

	/* Internal display set up */
	data[0] = 0xC5;
	data[1] = 0x01;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1, data, 2);

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
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

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
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

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
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

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
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

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
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

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
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

	/* turn on mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x03;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1, data, 2);

	/* turn it on */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);
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
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;

	DBG("dsi_mipi_cm_480_854_panel_enable() \n");

	/* Check if the display we are using is actually a TMD display */
	if (dssdev->panel.panel_id == MOT_DISP_370_MIPI_480_854_CM) {
		printk(KERN_INFO "te_scan_line is set = %d \n",
				map_data->te_scan_line);
		if (mapphone_panel_read_supplier_id() == SUPPLIER_ID_TMD) {
			DBG("dsi_mipi_cm_480_854_panel_enable() - TMD panel\n");
			dssdev->panel.panel_id = MOT_DISP_MIPI_480_854_CM;
			map_data->te_scan_line = DISP_480_854_CM_TE_SCANLINE;
			printk(KERN_INFO "Overwrite te_scan_line=%d\n",
				map_data->te_scan_line);
		}
	}

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;
	ret = mapphone_panel_lp_cmd_wrt_sync(false, EDISCO_SHORT_WRITE_1,
						data, 2,
						EDISCO_CMD_SET_MCS, 1,
						EDISCO_CMD_MCS_OFF, 0x3);
	if (ret)
		printk(KERN_ERR "failed to send SET_MCS \n");


	/* enable lane setting and test registers*/
	data[0] = 0xef;
	data[1] = 0x01;
	data[2] = 0x01;
	data[3] = 0x00;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 4);

	/* 2nd param 61 = 1 line; 63 = 2 lanes */
	data[0] = 0xef;
	data[1] = 0x60;
	data[2] = 0x63;
	data[3] = 0x00;
	if (dssdev->panel.panel_id == MOT_DISP_MIPI_480_854_CM)
		/* Reading lane_config and it will return
		* 0x63 or 2-lanes, 0x60 for 1-lane (1st source displ only)*/
		ret = mapphone_panel_lp_cmd_wrt_sync(true, 0x00,
					data, 4,
					0xef, 1,
					0x63, 0x63);
	else
		/* Reading lane_config and it will return
		* 0x1 for 2-lanes, 0x0 for 1-lane (2nd source displ only)*/
		ret = mapphone_panel_lp_cmd_wrt_sync(true, 0x00,
					data, 4,
					EDISCO_CMD_DATA_LANE_CONFIG, 1,
					0x1, 0x1);

	if (ret)
		printk(KERN_ERR "failed to send LANE_CONFIG \n");

	/* Forcing display inversion off for hardware issue
	 * on some phones (observed inverted color, ~1% of powerups fail)
	 */
	data[0] = EDISCO_CMD_SET_INVERSION_OFF;

	ret = mapphone_panel_lp_cmd_wrt_sync(true, 0,
			data, 1,
			EDISCO_CMD_READ_DISPLAY_IMAGE_MODE, 1,
			0x00, EDISCO_CMD_SET_INVERSION_OFF);
	if (ret)
		printk(KERN_ERR "failed to send EDISCO_CMD_SET_INVERSION_OFF \n");

	/* 2nd param 0 = WVGA; 1 = WQVGA */
	data[0] = EDISCO_CMD_SET_DISPLAY_MODE;
	data[1] = 0x00;
	ret = mapphone_panel_lp_cmd_wrt_sync(true, 0x00,
					data, 2,
					EDISCO_CMD_SET_DISPLAY_MODE, 1,
					data[1], 0x01);
	if (ret)
		printk(KERN_ERR "failed to send SET_DISPLAY_MODE \n");

	/* Set dynamic backlight control and PWM; D[7:4] = PWM_DIV[3:0];*/
	/* D[3]=0 (PWM OFF);
	 * D[2]=0 (auto BL control OFF for 1st source display only);
	 * D[1]=0 (Grama correction On for 1st source display only);
	 * D[0]=0 (Enhanced Image Correction OFF) */
	data[0] = EDISCO_CMD_SET_BCKLGHT_PWM;
	/* AUO displays require a different setting */
        if (dssdev->panel.panel_id == MOT_DISP_370_MIPI_480_854_CM) {
                data[1] = display_brightness();
                printk("Set display to :%u\n", display_brightness());
        } else {
                data[1] = display_brightness();
                printk("Set display to :%u\n", display_brightness());
        }
	ret = mapphone_panel_lp_cmd_wrt_sync(true, 0x00,
					data, 2,
					EDISCO_CMD_SET_BCKLGHT_PWM, 1,
					data[1], 0x1f);

	if (ret)
		printk(KERN_ERR "failed to send CABC/PWM \n");

	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = mapphone_panel_lp_cmd_wrt_sync(true, 0x0,
			data, 1,
			EDISCO_CMD_GET_POWER_MODE, 1,
			EDISCO_CMD_SLEEP_MODE_OUT, EDISCO_CMD_SLEEP_MODE_OUT);
	if (ret) {
		printk(KERN_ERR "failed to send EXIT_SLEEP_MODE \n");
		goto error;
	}

	/*
	 * 200ms delay for internal block stabilization required before panel
	 * turns on after EDISCO_CMD_SLEEP_MODE_OUT command
	 */
	set_delay_timer(dssdev, 200);

	/*
	 * Allow the OTP setting to  load
	 */
	msleep(5);

	printk(KERN_INFO "done EDISCO CTRL ENABLE\n");

	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_430_cm_480_854_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;

	DBG("dsi_mipi_430_cm_480_854_panel_enable() \n");

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;
	ret = mapphone_panel_lp_cmd_wrt_sync(false, EDISCO_SHORT_WRITE_1,
				data, 2,
				EDISCO_CMD_SET_MCS, 1,
				EDISCO_CMD_MCS_OFF, 0x3);
	if (ret)
		printk(KERN_ERR "failed to send SET_MCS \n");


	/* Enable 2 data lanes */
	data[0] = EDISCO_CMD_DATA_LANE_CONFIG;
	data[1] = EDISCO_CMD_DATA_LANE_TWO;
	ret = mapphone_panel_lp_cmd_wrt_sync(false, EDISCO_SHORT_WRITE_1,
			data, 2,
			EDISCO_CMD_DATA_LANE_CONFIG, 1,
			EDISCO_CMD_DATA_LANE_TWO, EDISCO_CMD_DATA_LANE_TWO);
	if (ret)
		printk(KERN_ERR "failed to send DATA_LANE_CONFIG \n");


	/* Forcing display inversion off for hardware issue
	 * on some phones (observed inverted color, ~1% of powerups fail)
	 */
	data[0] = EDISCO_CMD_SET_INVERSION_OFF;

	ret = mapphone_panel_lp_cmd_wrt_sync(true, 0,
			data, 1,
			EDISCO_CMD_READ_DISPLAY_IMAGE_MODE, 1,
			0x00, EDISCO_CMD_SET_INVERSION_OFF);
	if (ret)
		printk(KERN_ERR "failed to send EDISCO_CMD_SET_INVERSION_OFF \n");

	msleep(10);

	/* Set dynamic backlight control and PWM; D[7:4] = PWM_DIV[3:0];*/
	/* D[3]=0 (PWM OFF);
	 * D[2]=0 ;
	 * D[1]=0 ;
	 * D[0]=0 (Enhanced Image Correction OFF) */
	data[0] = EDISCO_CMD_SET_BCKLGHT_PWM;
	data[1] = 0xd9;
	ret = mapphone_panel_lp_cmd_wrt_sync(true, 0x00,
				data, 2,
				EDISCO_CMD_SET_BCKLGHT_PWM, 1,
				data[1], 0xff);
	if (ret)
		printk(KERN_ERR "failed to send CABC/PWM \n");


	/* Exit sleep mode */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = mapphone_panel_lp_cmd_wrt_sync(true, 0x0,
			data, 1,
			EDISCO_CMD_GET_POWER_MODE, 1,
			EDISCO_CMD_SLEEP_MODE_OUT, EDISCO_CMD_SLEEP_MODE_OUT);
	if (ret) {
		printk(KERN_ERR "failed to send EXIT_SLEEP_MODE \n");
		goto error;
	}

	/*
	 * 120ms delay for internal block stabilization required before panel
	 * turns on after EDISCO_CMD_SLEEP_MODE_OUT command
	 */
	set_delay_timer(dssdev, 120);

	/*
	 * Allow the OTP setting to  load
	 */
	msleep(10);

	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_310_1_cm_320_480_panel_enable(
				struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;

	DBG("dsi_mipi_310_1_cm_320_480_panel_enable() \n");

	mdelay(15);

	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);
	if (ret)
		goto error;

	mdelay(15);

	/*Dimming function setting */
	data[0] = 0x69;
	data[1] = 0x00;
	data[2] = 0xFF;
	data[3] = 0x00;
	data[4] = 0x14;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		goto error;

	/*Setting display brightness */
	data[0] = 0x51;
	data[1] = 0xFF;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
	if (ret)
		goto error;

	/*Setting CABC mode */
	data[0] = 0x55;
	data[1] = 0x02;	/* 0x02 = On 0x00 = OFF */
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
	if (ret)
		goto error;

	/* setting CABC */
	data[0] = 0x53;
	data[1] = 0x16;	/* Enable CABC. BCTRL=1, DD=1, BL=1*/
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
	if (ret)
		goto error;

	/* setting PWM */
	data[0] = 0x6B;
	data[1] = 0x00;
	data[2] = 0x01;	/* 0x01 = 31.26kHz */
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 3);
	if (ret)
		goto error;

	if (dssdev->ctrl.pixel_size == 16) {
		/* setting pixel Format to 16 */
		data[0] = 0x3A;
		data[1] = 0x65;
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
		if (ret)
			goto error;

		dispc_enable_spatial_dithering(true);
	}

	return 0;

error:
	return -EINVAL;

}


static int dsi_mipi_310_2_cm_320_480_panel_enable(
					struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret = 0;

	DBG("dsi_mipi_310_2_cm_320_480_panel_enable \n");

	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);
	data[0] = 0;

	mdelay(10);

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1, data, 2);

	ret = dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC, 6);
	if (ret)
		printk(KERN_ERR "failed to set max_rx__packet_size\n");

	memset(data, 0, sizeof(data));

	ret = dsi_vc_dcs_read(EDISCO_CMD_VC, EDISCO_CMD_READ_DDB_START,
					data, 6);
	printk(KERN_INFO "Supplier id return=0x%x%x, "
			"Manufacturer Version=0x%x%x, Revision=0x%x%x\n",
			data[0], data[1], data[2], data[3],
			data[4], data[5]);

	dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC, 1);

	data[0] = 0x3a;
	if (dssdev->ctrl.pixel_size == 16)
		data[1] = 0x05;
	else if (dssdev->ctrl.pixel_size != 18)
		data[1] = 0x06;
	else {
		printk(KERN_ERR "Invalied format pixel_size =%d\n",
			dssdev->ctrl.pixel_size);
		goto error;
	}

	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);

	/* Enable maximum brightness */
	data[0] = 0x51;
	data[1] = 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
	memset(data, 0, sizeof(data));

	/* Enable CABC Output  */
	data[0] = 0x53;
	data[1] = 0x2c;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);

	printk(KERN_INFO "done EDISCO CTRL ENABLE\n");

	return 0;
error:
	return -EINVAL;
}

static void mapphone_panel_set_man_te_trigger(struct omap_dss_device *dssdev);
static int mapphone_panel_enable(struct omap_dss_device *dssdev)
{
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;
	int ret;
	void *handle;

	DBG("mapphone_panel_enable\n");
	if (dssdev->platform_enable) {
		ret = dssdev->platform_enable(dssdev);
		if (ret)
			return ret;
	}

	handle = map_data->panel_handle;
	if (omap_panel_fod_enabled(handle)) {
		atomic_set(&map_data->state, PANEL_OFF);
	}
	omap_panel_fod_dss_state(handle, 1);
	omap_panel_fod_panel_state(handle, 1);

	switch (dssdev->panel.panel_id) {
	case MOT_DISP_MIPI_480_854_CM:
	case MOT_DISP_370_MIPI_480_854_CM:
		ret = dsi_mipi_cm_480_854_panel_enable(dssdev);
		break;
	case MOT_DISP_430_MIPI_480_854_CM:
		ret = dsi_mipi_430_cm_480_854_panel_enable(dssdev);
		break;
	case MOT_DISP_310_1_MIPI_320_480_CM:
		ret = dsi_mipi_310_1_cm_320_480_panel_enable(dssdev);
		break;
	case MOT_DISP_310_2_MIPI_320_480_CM:
		ret = dsi_mipi_310_2_cm_320_480_panel_enable(dssdev);
		break;
	case MOT_DISP_248_MIPI_320_240_VM:
		ret = dsi_mipi_248_vm_320_240_panel_enable(dssdev) ;
		break;
	case MOT_DISP_280_MIPI_320_240_VM:
		ret = dsi_mipi_280_vm_320_240_panel_enable(dssdev) ;
		break;
	default:
		printk(KERN_ERR "unsupport panel =0x%lx \n",
			dssdev->panel.panel_id);
		goto error;
	}

	mapphone_panel_set_man_te_trigger(dssdev);

	if (ret)
		goto error;

	return 0;
error:
	return -EINVAL;
}

static bool mapphone_panel_deep_sleep_mode(struct omap_dss_device *dssdev)
{
	bool ret = false;
	u8 data[2];

	switch (dssdev->panel.panel_id) {
	case MOT_DISP_310_1_MIPI_320_480_CM:
		data[0] = 0x3B; /* ENTER_DSTB_MODE command */
		data[1] = 0x01;
		ret = true;
		break;
	case MOT_DISP_310_2_MIPI_320_480_CM:
		data[0] = 0x4F; /* ENTER_DSTB_MODE command */
		data[1] = 0x01;
		ret = true;
		break;
	default:
		ret = false;
	}

	if (ret == true) {
		if (dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 2))
			printk(KERN_ERR "fail to send ENTER_DSTB_MODE =0x%x\n",
				data[0]);
	}

	return ret;
}

static void mapphone_panel_disable_local(struct omap_dss_device *dssdev)
{
	u8 data[1];
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;

	atomic_set(&map_data->state, PANEL_OFF);

	data[0] = EDISCO_CMD_SET_DISPLAY_OFF;
	dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 1);

	data[0] = EDISCO_CMD_ENTER_SLEEP_MODE;
	dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 1);

	msleep(100);

	/*
	 * mapphone_panel_deep_sleep_mode(0 will return false if the panel
	 * doesn't support depp_sleep_mode
	 */
	if (mapphone_panel_deep_sleep_mode(dssdev) == false) {
		if (dssdev->platform_disable)
			dssdev->platform_disable(dssdev);
	}
}

static void mapphone_panel_disable(struct omap_dss_device *dssdev)
{
	void *handle;

	DBG("mapphone_panel_disable\n");

	handle = ((struct mapphone_data *)dssdev->data)->panel_handle;
	omap_panel_fod_dss_state(handle, 0);
	if (omap_panel_fod_enabled(handle)) {
		DBG("Freezing the last frame on the display\n");
		return;
	}

	omap_panel_fod_panel_state(handle, 0);

	mapphone_panel_disable_local(dssdev);
}

static void mapphone_panel_setup_update(struct omap_dss_device *dssdev,
				      u16 x, u16 y, u16 w, u16 h)
{
	u8 data[5];
	int ret;

	/* set page, column address */
	data[0] = EDISCO_CMD_SET_PAGE_ADDRESS;
	data[1] = y >> 8;
	data[2] = y & 0xff;
	data[3] = (y + h - 1) >> 8;
	data[4] = (y + h - 1) & 0xff;
	ret = dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 5);
	if (ret)
		return;

	data[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	data[1] = x >> 8;
	data[2] = x & 0xff;
	data[3] = (x + w - 1) >> 8;
	data[4] = (x + w - 1) & 0xff;
	ret = dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 5);
	if (ret)
		return;
}

static int mapphone_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	u8 data[3];
	int ret;
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;

	if (enable == true) {
		data[0] = EDISCO_CMD_SET_TEAR_ON;
		data[1] = 0x00;
		ret = dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 2);
		if (ret)
			goto error;

		data[0] = EDISCO_CMD_SET_TEAR_SCANLINE;
		data[1] = (map_data->te_scan_line & 0xff00) >> 8;
		data[2] = (map_data->te_scan_line & 0xff);
		ret = dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 3);
		if (ret)
			goto error;
	} else {
		data[0] = EDISCO_CMD_SET_TEAR_OFF;
		data[1] = 0x00;
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
		if (ret)
			goto error;
	}

	DBG(" edisco_ctrl_enable_te(%d) \n", enable);
	return 0;

error:
	return -EINVAL;
}

static int mapphone_panel_get_hs_mode_timing(struct omap_dss_device *dssdev)
{
	/* The following time values are required for MIPI timing
	per OMAP spec */
	dssdev->phy.dsi.hs_timing.ths_prepare = 70;
	dssdev->phy.dsi.hs_timing.ths_prepare_ths_zero = 175;
	dssdev->phy.dsi.hs_timing.ths_trail = 60;
	dssdev->phy.dsi.hs_timing.ths_exit = 145;
	dssdev->phy.dsi.hs_timing.tlpx_half = 25;
	dssdev->phy.dsi.hs_timing.tclk_trail = 60;
	dssdev->phy.dsi.hs_timing.tclk_prepare = 65;
	dssdev->phy.dsi.hs_timing.tclk_zero = 260;

	/* These values are required for the following spec panels */
	if ((dssdev->panel.panel_id == MOT_DISP_430_MIPI_480_854_CM) ||
		(dssdev->panel.panel_id == MOT_DISP_370_MIPI_480_854_CM)) {
		dssdev->phy.dsi.hs_timing.ths_prepare_ths_zero = 454;
	}

	DBG("Programmed values: ths_prepare=%u ths_prepare_ths_zero=%u\n"
		" ths_trail=%u ths_exit=%u tlpx_half=%u \n"
		" tclk_trail =%u tclk_prepare=%u tclk_zero=%u\n",
		dssdev->phy.dsi.hs_timing.ths_prepare,
		dssdev->phy.dsi.hs_timing.ths_prepare_ths_zero,
		dssdev->phy.dsi.hs_timing.ths_trail,
		dssdev->phy.dsi.hs_timing.ths_exit,
		dssdev->phy.dsi.hs_timing.tlpx_half,
		dssdev->phy.dsi.hs_timing.tclk_trail,
		dssdev->phy.dsi.hs_timing.tclk_prepare,
		dssdev->phy.dsi.hs_timing.tclk_zero);

	return 0;
}

static void mapphone_panel_set_man_te_trigger(struct omap_dss_device *dssdev)
{
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;

	if ((dssdev->panel.panel_id == MOT_DISP_370_MIPI_480_854_CM) ||
		(dssdev->panel.panel_id == MOT_DISP_430_MIPI_480_854_CM))
			map_data->manual_te_trigger = true;
	else
		map_data->manual_te_trigger = false;
}

static bool mapphone_panel_manual_te_trigger(struct omap_dss_device *dssdev)
{
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;
	return map_data->manual_te_trigger;
}

static int mapphone_panel_rotate(struct omap_dss_device *display, u8 rotate)
{
	return 0;
}

static int mapphone_panel_mirror(struct omap_dss_device *display, bool enable)
{
	return 0;
}

static int mapphone_panel_run_test(struct omap_dss_device *display,
					int test_num)
{
	int r = -1; /* Returns -1 if no dssdev or test isn't supported */
		/* Returns 0 on success, or the test_num on failure */
	u8 data[SUPPLIER_ID_LEN];
	u16 id = 0xFFFF;

	if (!display)
		return r;

	if (test_num == 1)	{
		if ((display->panel.panel_id == MOT_DISP_430_MIPI_480_854_CM) ||
		    (display->panel.panel_id == MOT_DISP_370_MIPI_480_854_CM) ||
		    (display->panel.panel_id == MOT_DISP_MIPI_480_854_CM)) {
			r = test_num;
			/* Status check to ensure communication */
			dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC,
					SUPPLIER_ID_LEN);
			if (dsi_vc_dcs_read(EDISCO_CMD_VC,
					EDISCO_CMD_READ_DDB_START,
					data, SUPPLIER_ID_LEN)
					> 0) {
				id = (data[0] << 8) | data[1];

				switch (display->panel.panel_id) {
				case MOT_DISP_430_MIPI_480_854_CM:
					/* Shadow AUO panels report either */
					if ((id == SUPPLIER_ID_AUO) ||
					    (id == SUPPLIER_ID_AUO_43))
						r = 0;
					break;
				case MOT_DISP_370_MIPI_480_854_CM:
					if (id == SUPPLIER_ID_AUO)
						r = 0;
					break;
				case MOT_DISP_MIPI_480_854_CM:
					if (id == SUPPLIER_ID_TMD)
						r = 0;
					break;
				default:
					r = 0;
					break;
				}
			}
			dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC, 1);
		} else {
			/* If check not supported, return success */
			r = 0;
		}
	}
	return r;
}

static int mapphone_panel_suspend(struct omap_dss_device *dssdev)
{
	mapphone_panel_disable(dssdev);
	return 0;
}

static int mapphone_panel_resume(struct omap_dss_device *dssdev)
{
	return mapphone_panel_enable(dssdev);
}

static struct omap_dss_driver mapphone_panel_driver = {
	.probe			= mapphone_panel_probe,
	.remove			= mapphone_panel_remove,

	.enable			= mapphone_panel_enable,
	.framedone		= mapphone_panel_display_on,
	.disable		= mapphone_panel_disable,
	.suspend		= mapphone_panel_suspend,
	.resume			= mapphone_panel_resume,
	.setup_update		= mapphone_panel_setup_update,
	.hs_mode_timing		= mapphone_panel_get_hs_mode_timing,
	.enable_te		= mapphone_panel_enable_te,
	.manual_te_trigger 	= mapphone_panel_manual_te_trigger,
	.set_rotate		= mapphone_panel_rotate,
	.set_mirror		= mapphone_panel_mirror,
	.run_test		= mapphone_panel_run_test,
	.get_scl_setting 	= mapphone_panel_get_scl_setting,
	.read_scl		= mapphone_panel_read_scl,
	.sw_te_sup		= mapphone_panel_sw_te_sup,
	.deep_sleep_mode	= mapphone_panel_deep_sleep_mode,

	.driver = {
		.name = "mapphone-panel",
		.owner = THIS_MODULE,
	},
};


static int __init mapphone_panel_init(void)
{
	DBG("mapphone_panel_init\n");
	omap_dss_register_driver(&mapphone_panel_driver);
	mapphone_panel_device_read_dt = false;
	return 0;
}

static void __exit mapphone_panel_exit(void)
{
	DBG("mapphone_panel_exit\n");

	omap_dss_unregister_driver(&mapphone_panel_driver);
}

module_init(mapphone_panel_init);
module_exit(mapphone_panel_exit);

MODULE_AUTHOR("Rebecca Schultz Zavin <rebecca@android.com>");
MODULE_DESCRIPTION("Sholes Panel Driver");
MODULE_LICENSE("GPL");
