/*
 * Defines for mapphone board
 */

int __init mapphone_hsmmc_init(void);
void __init mapphone_gpio_mapping_init(void);
void __init mapphone_panel_init(void);
void __init mapphone_cpcap_client_init(void);
void __init mapphone_spi_init(void);
void __init mapphone_als_init(void);
void __init mapphone_i2c_init(void);

#define BOOT_MODE_MAX_LEN 30
#define I2C_BUS_MAX_DEVICES 5
#define I2C_MAX_DEV_NAME_LEN 16
#define I2C_BUS_PROP_NAME_LEN 12
