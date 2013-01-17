/*
 * Defines for mapphone board
 */

extern int __init mapphone_hsmmc_init(void);
extern void __init mapphone_gpio_mapping_init(void);
extern void __init mapphone_panel_init(void);
extern void __init mapphone_cpcap_client_init(void);
extern void __init mapphone_spi_init(void);
extern void __init mapphone_als_init(void);
extern void __init mapphone_i2c_init(void);
extern void __init mapphone_padconf_init(void);
void __init mapphone_touch_panel_init(struct i2c_board_info *i2c_info);
void __init mapphone_touch_btn_init(struct i2c_board_info *i2c_info);

#define BOOT_MODE_MAX_LEN 30
#define I2C_BUS_MAX_DEVICES 5
#define I2C_MAX_DEV_NAME_LEN 16
#define I2C_BUS_PROP_NAME_LEN 12
