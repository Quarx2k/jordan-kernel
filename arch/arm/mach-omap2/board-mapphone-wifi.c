/* linux/arch/arm/mach-omap2/board-mapphone-wifi.c
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/wifi_tiwlan.h>
#include <plat/board-mapphone.h>
#include <linux/wl12xx.h>

#include <linux/debugfs.h>

#define MAPPHONE_WIFI_PMENA_GPIO	186
#define MAPPHONE_WIFI_IRQ_GPIO	65


static struct wl12xx_platform_data mapphone_wlan_data __initdata = {
	.irq = -1, /* OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),*/
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
};


static void mapphone_wifi_init(void)
{
	int ret;

	printk("%s: start\n", __func__);
	ret = gpio_request(MAPPHONE_WIFI_IRQ_GPIO, "wifi_irq");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			MAPPHONE_WIFI_IRQ_GPIO);
	}
	ret = gpio_request(MAPPHONE_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			MAPPHONE_WIFI_PMENA_GPIO);
		gpio_free(MAPPHONE_WIFI_IRQ_GPIO);
	}
	gpio_direction_input(MAPPHONE_WIFI_IRQ_GPIO);
	gpio_direction_output(MAPPHONE_WIFI_PMENA_GPIO, 0);

	mapphone_wlan_data.irq = OMAP_GPIO_IRQ(MAPPHONE_WIFI_IRQ_GPIO);

	if (wl12xx_set_platform_data(&mapphone_wlan_data))
	{
		pr_err("Error setting wl12xx data\n");
	}
	printk("Wifi init done\n");
}


device_initcall(mapphone_wifi_init);
