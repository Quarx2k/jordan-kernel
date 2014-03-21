#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <plat/display.h>
#include <plat/dma.h>
#include <plat/omap-pm.h>

/* #define DEBUG */
#ifdef DEBUG
#define DBG(format, ...) (\
	printk(KERN_DEBUG "hdtv-panel: " format, ## __VA_ARGS__))
#else
#define DBG(format, ...)
#endif

/* Default panel timings (720p) */
const struct omap_video_timings mapphone_hdtv_panel_timings_default = {
	.x_res		= 1280,
	.y_res		= 720,
	.pixel_clock	= 74250,
	.hsw		= 40,
	.hfp		= 110,
	.hbp		= 220,
	.vsw		= 5,
	.vfp		= 5,
	.vbp		= 20,
};

static int mapphone_hdtv_panel_probe(struct omap_dss_device *dssdev)
{
	DBG("mapphone_hdtv_panel_probe\n");

	dssdev->panel.timings = mapphone_hdtv_panel_timings_default;

	return 0;
}

static void mapphone_hdtv_panel_remove(struct omap_dss_device *dssdev)
{
	DBG("mapphone_hdtv_panel_remove\n");
}

static int mapphone_hdtv_panel_enable(struct omap_dss_device *dssdev)
{
	int ret;

	DBG("mapphone_hdtv_panel_enable\n");

	/* Set min throughput to:
	 * (1280 x 720 x 2bpp x 60fps x 3 L3 accesses) + (200M client overhead)
	 */
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 531776);

	if (dssdev->platform_enable)
		ret = dssdev->platform_enable(dssdev);
	else
		ret = 0;

	return ret;
}

static void mapphone_hdtv_panel_disable(struct omap_dss_device *dssdev)
{
	DBG("mapphone_hdtv_panel_disable\n");

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* Remove throughput requirement */
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
}

static void mapphone_hdtv_panel_setup_update(struct omap_dss_device *dssdev,
				      u16 x, u16 y, u16 w, u16 h)
{
}

static int mapphone_hdtv_panel_enable_te(struct omap_dss_device *dssdev,
								bool enable)
{
	return 0;
}

static int mapphone_hdtv_panel_rotate(struct omap_dss_device *display,
								u8 rotate)
{
	return 0;
}

static int mapphone_hdtv_panel_mirror(struct omap_dss_device *display,
								bool enable)
{
	return 0;
}

static int mapphone_hdtv_panel_run_test(struct omap_dss_device *display,
								int test_num)
{
	return 0;
}

static int mapphone_hdtv_panel_suspend(struct omap_dss_device *dssdev)
{
	DBG("mapphone_hdtv_panel_suspend\n");

	mapphone_hdtv_panel_disable(dssdev);

	return 0;
}

static int mapphone_hdtv_panel_resume(struct omap_dss_device *dssdev)
{
	DBG("mapphone_hdtv_panel_resume\n");

	return mapphone_hdtv_panel_enable(dssdev);
}

static struct omap_dss_driver mapphone_hdtv_panel_driver = {
	.probe		= mapphone_hdtv_panel_probe,
	.remove		= mapphone_hdtv_panel_remove,
	.enable		= mapphone_hdtv_panel_enable,
	.disable	= mapphone_hdtv_panel_disable,
	.suspend	= mapphone_hdtv_panel_suspend,
	.resume		= mapphone_hdtv_panel_resume,
	.setup_update	= mapphone_hdtv_panel_setup_update,
	.enable_te	= mapphone_hdtv_panel_enable_te,
	.set_rotate	= mapphone_hdtv_panel_rotate,
	.set_mirror	= mapphone_hdtv_panel_mirror,
	.run_test	= mapphone_hdtv_panel_run_test,
	.driver = {
		.name	= "hdtv-panel",
		.owner	= THIS_MODULE,
	},
};

static int __init mapphone_hdtv_panel_init(void)
{
	DBG("mapphone_hdtv_panel_init\n");

	omap_dss_register_driver(&mapphone_hdtv_panel_driver);

	return 0;
}

static void __exit mapphone_hdtv_panel_exit(void)
{
	DBG("mapphone_hdtv_panel_exit\n");

	omap_dss_unregister_driver(&mapphone_hdtv_panel_driver);
}

module_init(mapphone_hdtv_panel_init);
module_exit(mapphone_hdtv_panel_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("Mapphone HDTV Panel Driver");
MODULE_LICENSE("GPL");

