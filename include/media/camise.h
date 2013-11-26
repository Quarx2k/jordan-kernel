/*
 * drivers/media/video/camise.h
 *
 */

#ifndef CAMISE_H
#define CAMISE_H

#define CAMISE_I2C_ADDR 0x3C
 /**
 * struct camise_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @default_regs: Default registers written after power-on or reset.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct camise_platform_data {
        int (*power_set)(struct v4l2_int_device *s, enum v4l2_power power);
	int (*ifparm)(struct v4l2_ifparm *p);
	int (*priv_data_set)(void *);
	void (*lock_cpufreq)(int lock);
	void (*if_config)(struct v4l2_int_device *s);
	int (*get_size)(int idx, unsigned long *w, unsigned long *h);
};

#endif /* ifndef CAMISE_H */
