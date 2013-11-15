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
extern void __init mapphone_usbhost_init(void);
extern void __init mapphone_gadget_init(void);
extern void __init mapphone_camera_init(void);

extern struct attribute_group *mapphone_touch_vkey_prop_attr_group;

#define BOOT_MODE_MAX_LEN 30
#define I2C_BUS_MAX_DEVICES 5
#define I2C_MAX_DEV_NAME_LEN 16
#define I2C_BUS_PROP_NAME_LEN 12
