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
#include <plat/board-mapphone.h>
#include <linux/wl12xx.h>

static struct wl12xx_platform_data mapphone_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(MAPPHONE_WIFI_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_26,
};


static int mapphone_wifi_init(void)
{
	int ret;

	ret = gpio_request(MAPPHONE_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			MAPPHONE_WIFI_PMENA_GPIO);
		gpio_free(MAPPHONE_WIFI_PMENA_GPIO);
	}
	gpio_direction_output(MAPPHONE_WIFI_PMENA_GPIO, 0);

	if (wl12xx_set_platform_data(&mapphone_wlan_data))
	{
		pr_err("Error setting wl12xx data\n");
	}
	printk("Wifi init done\n");

	return 0;
}


device_initcall(mapphone_wifi_init);
