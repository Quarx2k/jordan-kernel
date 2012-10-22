/*
 * Defines for zoom boards
 */
#include <video/omapdss.h>

#define ZOOM_NAND_CS    0

void __init mapphone_gpio_mapping_init(void);
int __init mapphone_hsmmc_init(void);
void __init mapphone_panel_init(void);

#if (defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)) && \
	defined(CONFIG_VIDEO_OMAP3)
#include <media/imx046.h>
extern struct imx046_platform_data zoom2_imx046_platform_data;
#endif

#ifdef CONFIG_VIDEO_OMAP3
extern void zoom2_cam_init(void);
#else
#define zoom2_cam_init()	NULL
#endif

#if (defined(CONFIG_VIDEO_LV8093) || defined(CONFIG_VIDEO_LV8093_MODULE)) && \
	defined(CONFIG_VIDEO_OMAP3)
#include <media/lv8093.h>
extern struct lv8093_platform_data zoom2_lv8093_platform_data;
#define LV8093_PS_GPIO		7
/* GPIO7 is connected to lens PS pin through inverter */
#define LV8093_PWR_OFF		1
#define LV8093_PWR_ON		(!LV8093_PWR_OFF)
#endif

#define ZOOM2_HEADSET_EXTMUTE_GPIO	153
