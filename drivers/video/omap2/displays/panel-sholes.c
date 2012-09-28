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

#define DEBUG
#ifdef DEBUG
#define DBG(format, ...) (printk(KERN_DEBUG "sholes-panel: " format, ## __VA_ARGS__))
#else
#define DBG(format, ...)
#endif

#define EDISCO_CMD_SOFT_RESET		0x01
#define EDISCO_CMD_ENTER_SLEEP_MODE	0x10
#define EDISCO_CMD_EXIT_SLEEP_MODE	0x11
#define EDISCO_CMD_SET_DISPLAY_ON	0x29
#define EDISCO_CMD_SET_DISPLAY_OFF	0x28
#define EDISCO_CMD_SET_COLUMN_ADDRESS	0x2A
#define EDISCO_CMD_SET_PAGE_ADDRESS	0x2B
#define EDISCO_CMD_SET_TEAR_ON		0x35
#define EDISCO_CMD_SET_TEAR_SCANLINE	0x44

#define EDISCO_CMD_VC   0
#define EDISCO_VIDEO_VC 1

#define EDISCO_LONG_WRITE	0x29
#define EDISCO_SHORT_WRITE_1	0x23
#define EDISCO_SHORT_WRITE_0	0x13

#define PANEL_OFF	0x0
#define PANEL_ON	0x1


static struct omap_video_timings sholes_panel_timings = {
	.x_res		= 480,
	.y_res		= 854,
	.hfp		= 44,
	.hsw		= 2,
	.hbp		= 38,
	.vfp		= 1,
	.vsw		= 1,
	.vbp		= 1,
	.w		= 46,
	.h 		= 82,
};

atomic_t state;

static int sholes_panel_probe(struct omap_dss_device *dssdev)
{
	DBG("probe\n");
	dssdev->ctrl.pixel_size = 24;
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = sholes_panel_timings;
	atomic_set(&state, PANEL_ON);
	return 0;
}

static void sholes_panel_remove(struct omap_dss_device *dssdev)
{
	return;
}

static int sholes_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;

	DBG("enable\n");
	if (dssdev->platform_enable) {
		ret = dssdev->platform_enable(dssdev);
		if (ret)
			return ret;
	}

	/* turn of mcs register acces protection */
	data[0] = 0xb2;
	data[1] = 0x00;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1, data, 2);

	/* enable lane setting and test registers*/
	data[0] = 0xef;
	data[1] = 0x01;
	data[2] = 0x01;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 3);

	/* 2nd param 61 = 1 line; 63 = 2 lanes */
	data[0] = 0xef;
	data[1] = 0x60;
	data[2] = 0x63;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 3);

	/* 2nd param 0 = WVGA; 1 = WQVGA */
	data[0] = 0xb3;
	data[1] = 0x00;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);

	/* Set dynamic backlight control and PWM; D[7:4] = PWM_DIV[3:0];*/
	/* D[3]=0 (PWM OFF);
	 * D[2]=0 (auto BL control OFF);
	 * D[1]=0 (Grama correction On);
	 * D[0]=0 (Enhanced Image Correction OFF) */
	data[0] = 0xb4;
	data[1] = 0x1f;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);

	/* set page, column address */
	data[0] = EDISCO_CMD_SET_PAGE_ADDRESS;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = (dssdev->panel.timings.y_res - 1) >> 8;
	data[4] = (dssdev->panel.timings.y_res - 1) & 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		goto error;

	data[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = (dssdev->panel.timings.x_res - 1) >> 8;
	data[4] = (dssdev->panel.timings.x_res - 1) & 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		goto error;

	/* turn it on */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	//ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_0, data, 1);
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);

	mdelay(200);

	return 0;
error:
	return -EINVAL;
}

static void sholes_panel_disable(struct omap_dss_device *dssdev)
{
	u8 data[1];

	DBG("sholes_panel_ctrl_disable\n");

	data[0] = EDISCO_CMD_SET_DISPLAY_OFF;
	dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);

	data[0] = EDISCO_CMD_ENTER_SLEEP_MODE;
	dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);
	msleep(120);

	atomic_set(&state, PANEL_OFF);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

}

static int sholes_panel_display_on(struct omap_dss_device *dssdev)
{
	u8 data = EDISCO_CMD_SET_DISPLAY_ON;

	if (atomic_cmpxchg(&state, PANEL_OFF, PANEL_ON) ==
	    PANEL_OFF) {
		return dsi_vc_dcs_write(EDISCO_CMD_VC, &data, 1);
	}
	return 0;
}

static void sholes_panel_setup_update(struct omap_dss_device *dssdev,
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
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		return;

	data[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	data[1] = x >> 8;
	data[2] = x & 0xff;
	data[3] = (x + w - 1) >> 8;
	data[4] = (x + w - 1) & 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		return;
}

static int sholes_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	u8 data[3];
	int ret;

	data[0] = EDISCO_CMD_SET_TEAR_ON;
	data[1] = 0x00;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
	if (ret)
		goto error;

	data[0] = EDISCO_CMD_SET_TEAR_SCANLINE;
	data[1] = 0x03;
	data[2] = 0x00;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 3);
	if (ret)
		goto error;

	DBG("edisco_ctrl_enable_te \n");
	return 0;

error:
	return -EINVAL;
}

static int sholes_panel_rotate(struct omap_dss_device *display, u8 rotate)
{
	return 0;
}

static int sholes_panel_mirror(struct omap_dss_device *display, bool enable)
{
	return 0;
}

static int sholes_panel_run_test(struct omap_dss_device *display, int test_num)
{
	return 0;
}

static int sholes_panel_suspend(struct omap_dss_device *dssdev)
{
	sholes_panel_disable(dssdev);
	return 0;
}

static int sholes_panel_resume(struct omap_dss_device *dssdev)
{
	return sholes_panel_enable(dssdev);
}

static struct omap_dss_driver sholes_panel_driver = {
	.probe = sholes_panel_probe,
	.remove = sholes_panel_remove,

	.enable = sholes_panel_enable,
	.framedone = sholes_panel_display_on,
	.disable = sholes_panel_disable,
	.suspend = sholes_panel_suspend,
	.resume = sholes_panel_resume,
	.setup_update = sholes_panel_setup_update,
	.enable_te = sholes_panel_enable_te,
	.set_rotate = sholes_panel_rotate,
	.set_mirror = sholes_panel_mirror,
	.run_test = sholes_panel_run_test,

	.driver = {
		.name = "sholes-panel",
		.owner = THIS_MODULE,
	},
};


static int __init sholes_panel_init(void)
{
	DBG("sholes_panel_init\n");
	omap_dss_register_driver(&sholes_panel_driver);
	return 0;
}

static void __exit sholes_panel_exit(void)
{
	DBG("sholes_panel_exit\n");

	omap_dss_unregister_driver(&sholes_panel_driver);
}

module_init(sholes_panel_init);
module_exit(sholes_panel_exit);

MODULE_AUTHOR("Rebecca Schultz Zavin <rebecca@android.com>");
MODULE_DESCRIPTION("Sholes Panel Driver");
MODULE_LICENSE("GPL");
