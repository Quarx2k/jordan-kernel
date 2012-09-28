/*
 * linux/arch/arm/mach-omap2/board-mapphone-sensors.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/airc.h>
#include <linux/akm8973_akmd.h>
#include <linux/akm8975.h>
#include <linux/bu52014hfv.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio_mapping.h>
#include <linux/i2c/adp8870.h>
#include <linux/input.h>
#include <linux/isl29030.h>
#include <linux/kxtf9.h>
#include <linux/leds.h>
#include <linux/lis331dlh.h>
#include <linux/regulator/consumer.h>
#include <linux/sfh7743.h>
#include <linux/vib-gpio.h>
#include <linux/vib-pwm.h>
#include <linux/interrupt.h>

#include <plat/dmtimer.h>
#include <plat/gpio.h>
#include <plat/keypad.h>
#include <plat/mux.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#define MAPPHONE_PROX_INT_GPIO		180
#define MAPPHONE_VIBRATOR_GPIO		181

#define PROXIMITY_SFH7743		0
#define PROXIMITY_ISL29030		1

static u8 prox_sensor_type = PROXIMITY_SFH7743;

/*
 * Vibe
 */

static int vib_pwm_period;
static int vib_pwm_duty;
static unsigned long load_reg, cmp_reg;
static int vib_pwm_enable_gpio;
static int linear_vib_only;

static struct regulator *mapphone_vibrator_regulator;
static int mapphone_vibrator_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vvib");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_vibrator_regulator = reg;
	return 0;
}

static void mapphone_vibrator_exit(void)
{
	regulator_put(mapphone_vibrator_regulator);
}

static int mapphone_vibrator_power_on(void)
{
	regulator_set_voltage(mapphone_vibrator_regulator, 3000000, 3000000);
	return regulator_enable(mapphone_vibrator_regulator);
}

static int mapphone_vibrator_power_off(void)
{
	if (mapphone_vibrator_regulator)
		return regulator_disable(mapphone_vibrator_regulator);
	return 0;
}

static struct vib_gpio_platform_data mapphone_vib_gpio_data = {
	.gpio = MAPPHONE_VIBRATOR_GPIO,
	.max_timeout = 15000,
	.active_low = 0,
	.initial_vibrate = 0,

	.init = mapphone_vibrator_initialization,
	.exit = mapphone_vibrator_exit,
	.power_on = mapphone_vibrator_power_on,
	.power_off = mapphone_vibrator_power_off,
};

static struct platform_device mapphone_vib_gpio = {
	.name           = "vib-gpio",
	.id             = -1,
	.dev            = {
		.platform_data  = &mapphone_vib_gpio_data,
	},
};

static void mapphone_vibrator_init(void)
{
	int vibrator_gpio = MAPPHONE_VIBRATOR_GPIO;
#ifdef CONFIG_ARM_OF
	vibrator_gpio = get_gpio_by_name("vib_control_en");
	if (vibrator_gpio < 0) {
		printk(KERN_DEBUG
			"cannot retrieve vib_control_en from device tree\n");
		vibrator_gpio = MAPPHONE_VIBRATOR_GPIO;
	}
	mapphone_vib_gpio_data.gpio = vibrator_gpio;
#endif
	if (gpio_request(mapphone_vib_gpio_data.gpio, "vib_ctrl_en")) {
		printk(KERN_ERR "vib_control_en GPIO request failed!\n");
		return;
	}

	gpio_direction_output(vibrator_gpio, 0);
	omap_cfg_reg(Y4_34XX_GPIO181);
}

/*
 * Lvibe
 */

static struct omap_dm_timer *vib_pwm_timer;

#ifdef CONFIG_VIB_PWM_SWEEP

struct omap_pwm_time {
	uint32_t	period;
};

struct omap_pwm_reg {
	int		type;
	uint32_t	load;
	int		time;
};

#define PWM_PATTERN_LST	1
#define PWM_PATTERN_SQR	2
#define PWM_PATTERN_OFF	3

struct omap_pwm_pattern {
	struct omap_pwm_reg *head;
	int size;
	struct omap_pwm_reg *this;
	struct hrtimer hrtimer;
	int timer_running;
	int int_enabled;
};

static struct omap_pwm_pattern *omap_pwm_pat_sweep;
static DEFINE_SPINLOCK(pwm_pat_lock);

static const struct omap_pwm_time omap_pwm_sweep[] = {
	{3448}, {3448}, {3448}, {3448}, {3425}, {3425}, {3425}, {3425},
	{3401}, {3401}, {3401}, {3401}, {3378}, {3378}, {3378}, {3378},
	{3356}, {3356}, {3356}, {3356}, {3333}, {3333}, {3333}, {3333},
	{3311}, {3311}, {3311}, {3311}, {3289}, {3289}, {3289}, {3289},
	{3268}, {3268}, {3268}, {3268}, {3247}, {3247}, {3247}, {3247},
	{3226}, {3226}, {3226}, {3226}, {3205}, {3205}, {3205}, {3205},
	{3185}, {3185}, {3185}, {3185}, {3165}, {3165}, {3165}, {3165},
	{3145}, {3145}, {3145}, {3145},
};

static const struct omap_pwm_time omap_pwm_sqr = { 3125 };

#define OMAP_PWM_PATTERN_SQR_TIME	800

static int lvib_pattern_load_next(void)
{
	struct omap_pwm_pattern *pwm_pat;
	struct omap_pwm_reg *preg;
	unsigned long flags;

	spin_lock_irqsave(&pwm_pat_lock, flags);
	if (!omap_pwm_pat_sweep) {
		spin_unlock_irqrestore(&pwm_pat_lock, flags);
		printk(KERN_ERR "%s omap_pwm_pattern is NULL\n", __func__);
		return -1;
	}
	pwm_pat = omap_pwm_pat_sweep;
	if (pwm_pat->this < pwm_pat->head ||
		(pwm_pat->this >= pwm_pat->head + pwm_pat->size - 1))
		pwm_pat->this = pwm_pat->head;
	else
		pwm_pat->this++;
	preg = pwm_pat->this;

	switch (preg->type) {
	case PWM_PATTERN_LST:
		if (!pwm_pat->timer_running) {
			gpio_set_value(vib_pwm_enable_gpio, 1);
			omap_dm_timer_set_int_enable(vib_pwm_timer,
						OMAP_TIMER_INT_OVERFLOW);
			pwm_pat->int_enabled = 1;
			omap_dm_timer_set_load_start(vib_pwm_timer, 1,
				-preg->load);
			pwm_pat->timer_running = 1;
		} else {
			if (!pwm_pat->int_enabled) {
				omap_dm_timer_set_int_enable(vib_pwm_timer,
						OMAP_TIMER_INT_OVERFLOW);
				pwm_pat->int_enabled = 1;
			}
			omap_dm_timer_update_load(vib_pwm_timer, -preg->load);
		}
		break;
	case PWM_PATTERN_SQR:
		omap_dm_timer_set_int_enable(vib_pwm_timer, 0);
		pwm_pat->int_enabled = 0;
		if (!pwm_pat->timer_running) {
			gpio_set_value(vib_pwm_enable_gpio, 1);
			omap_dm_timer_set_load_start(vib_pwm_timer, 1,
				-preg->load);
			pwm_pat->timer_running = 1;
		} else
			omap_dm_timer_update_load(vib_pwm_timer, -preg->load);
		hrtimer_start(&pwm_pat->hrtimer,
			ktime_set(pwm_pat->this->time / 1000,
				(pwm_pat->this->time % 1000) * 1000000),
			      HRTIMER_MODE_REL);
		break;
	case PWM_PATTERN_OFF:
		gpio_set_value(vib_pwm_enable_gpio, 0);
		omap_dm_timer_set_int_enable(vib_pwm_timer, 0);
		pwm_pat->int_enabled = 0;
		omap_dm_timer_stop(vib_pwm_timer);
		pwm_pat->timer_running = 0;
		hrtimer_start(&pwm_pat->hrtimer,
			ktime_set(pwm_pat->this->time / 1000,
				(pwm_pat->this->time % 1000) * 1000000),
			      HRTIMER_MODE_REL);
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&pwm_pat_lock, flags);
	return 0;
}

static irqreturn_t omap_pwm_timer_intr(int irq, void *data)
{
	uint32_t status = omap_dm_timer_read_status(vib_pwm_timer);
	omap_dm_timer_write_status(vib_pwm_timer, status);
	if (status & OMAP_TIMER_INT_OVERFLOW)
		lvib_pattern_load_next();
	return IRQ_HANDLED;
}

static enum hrtimer_restart pwm_pat_hrtimer_func(struct hrtimer *hrtimer)
{
	lvib_pattern_load_next();
	return HRTIMER_NORESTART;
}

/* vib_pwm_timer already requested */
static int omap_timed_vib_pwm_init(void)
{
	struct omap_pwm_pattern *pwm_pat;
	const struct omap_pwm_time *ptime;
	struct omap_pwm_reg *preg;
	int i, ret;
	uint32_t rate;

	if (omap_pwm_pat_sweep) {
		printk(KERN_ERR "%s: %p already initialized\n",
			__func__, omap_pwm_pat_sweep);
		return 0;
	}
	pwm_pat = kzalloc(sizeof(struct omap_pwm_pattern), GFP_KERNEL);
	if (!pwm_pat) {
		printk(KERN_ERR "%s: alloc fail %d\n", __func__, __LINE__);
		return -1;
	}
	pwm_pat->head = kzalloc(sizeof(struct omap_pwm_reg)
		* (ARRAY_SIZE(omap_pwm_sweep) + 1), GFP_KERNEL);
	if (!pwm_pat->head) {
		printk(KERN_ERR "%s: alloc fail %d\n", __func__, __LINE__);
		kfree(pwm_pat);
		return -1;
	}
	pwm_pat->size = ARRAY_SIZE(omap_pwm_sweep) + 1;
	rate = clk_get_rate(omap_dm_timer_get_fclk(vib_pwm_timer));
	hrtimer_init(&pwm_pat->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pwm_pat->hrtimer.function = pwm_pat_hrtimer_func;
	pwm_pat->this = 0;
	pwm_pat->timer_running = 0;
	pwm_pat->int_enabled = 0;

	for (i = 0, ptime = omap_pwm_sweep, preg = pwm_pat->head;
		i < ARRAY_SIZE(omap_pwm_sweep); i++, ptime++, preg++) {
		preg->type = PWM_PATTERN_LST;
		preg->load = rate * ptime->period / 1000000;
	}
	ptime = &omap_pwm_sqr;
	preg->type = PWM_PATTERN_SQR;
	preg->load = rate * ptime->period / 1000000;
	preg->time = OMAP_PWM_PATTERN_SQR_TIME;

	ret = request_irq(omap_dm_timer_get_irq(vib_pwm_timer),
			 omap_pwm_timer_intr,
			 IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
			 "vib_pwm", pwm_pat);
	omap_dm_timer_set_int_enable(vib_pwm_timer, 0);

	omap_pwm_pat_sweep = pwm_pat;
	return ret;
}

static void omap_timed_vib_pwm_exit(void)
{
	unsigned long flags;
	spin_lock_irqsave(&pwm_pat_lock, flags);
	if (omap_pwm_pat_sweep) {
		kfree(omap_pwm_pat_sweep->head);
		kfree(omap_pwm_pat_sweep);
		omap_pwm_pat_sweep = NULL;
	}
	spin_unlock_irqrestore(&pwm_pat_lock, flags);
}

static void mapphone_lvibrator_pattern_on(int pattern)
{
	omap_dm_timer_enable(vib_pwm_timer);
	omap_dm_timer_stop(vib_pwm_timer);
	gpio_set_value(vib_pwm_enable_gpio, 0);
	omap_dm_timer_set_pwm(vib_pwm_timer, 0, 1,
		OMAP_TIMER_TRIGGER_OVERFLOW);
	lvib_pattern_load_next();
}

static void mapphone_lvibrator_pattern_off(void)
{
	unsigned long flags;
	spin_lock_irqsave(&pwm_pat_lock, flags);
	if (!omap_pwm_pat_sweep) {
		spin_unlock_irqrestore(&pwm_pat_lock, flags);
		printk(KERN_ERR "%s omap_pwm_pattern is NULL\n", __func__);
		return;
	}
	omap_dm_timer_set_int_enable(vib_pwm_timer, 0);
	hrtimer_cancel(&omap_pwm_pat_sweep->hrtimer);
	omap_pwm_pat_sweep->this = 0;
	omap_pwm_pat_sweep->timer_running = 0;
	omap_pwm_pat_sweep->int_enabled = 0;
	spin_unlock_irqrestore(&pwm_pat_lock, flags);
}

#else /* CONFIG_VIB_PWM_SWEEP */

static inline int omap_timed_vib_pwm_init(void) { return 0; }
static inline void mapphone_lvibrator_pattern_off(void) {}
static inline void omap_timed_vib_pwm_exit(void) {}

#endif /* CONFIG_VIB_PWM_SWEEP */

static int mapphone_lvibrator_initialization(void)
{
	uint32_t timer_rate = 0;
	int ret = 0;
	vib_pwm_timer = omap_dm_timer_request_specific(11);
	if (vib_pwm_timer == NULL) {
		ret = -ENODEV;
		return ret;
	}
	timer_rate = clk_get_rate(omap_dm_timer_get_fclk(vib_pwm_timer));
	load_reg = timer_rate * vib_pwm_period / 1000000;
	cmp_reg = timer_rate * (vib_pwm_period -
				vib_pwm_duty) / 1000000;

	omap_dm_timer_set_source(vib_pwm_timer, OMAP_TIMER_SRC_32_KHZ);
	ret = omap_timed_vib_pwm_init();
	if (ret)
		return ret;

	ret = gpio_request(vib_pwm_enable_gpio, "glohap_en_omap");
	if (ret) {
		printk(KERN_ERR "Vibrator GPIO request error %d\n", ret);
		return ret;
	}
	gpio_direction_output(vib_pwm_enable_gpio, 1);

	return 0;
}

static void mapphone_lvibrator_exit(void)
{
	mapphone_lvibrator_pattern_off();
	omap_timed_vib_pwm_exit();
	omap_dm_timer_stop(vib_pwm_timer);
	omap_dm_timer_disable(vib_pwm_timer);
	gpio_set_value(vib_pwm_enable_gpio, 0);
}

static void mapphone_lvibrator_power_on(void)
{
	gpio_set_value(vib_pwm_enable_gpio, 1);
	omap_dm_timer_enable(vib_pwm_timer);
	omap_dm_timer_set_int_enable(vib_pwm_timer, 0);
	omap_dm_timer_set_load(vib_pwm_timer, 1, -load_reg);
	omap_dm_timer_set_match(vib_pwm_timer, 1, -cmp_reg);
	omap_dm_timer_set_pwm(vib_pwm_timer, 0, 1,
		       OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_write_counter(vib_pwm_timer, -2);
	omap_dm_timer_start(vib_pwm_timer);
}

static void mapphone_lvibrator_power_off(void)
{
	mapphone_lvibrator_pattern_off();
	omap_dm_timer_stop(vib_pwm_timer);
	omap_dm_timer_disable(vib_pwm_timer);
	gpio_set_value(vib_pwm_enable_gpio, 0);
}

static struct vib_pwm_platform_data vib_pwm_data = {
	.initial_vibrate = 500,
	.init = mapphone_lvibrator_initialization,
	.exit = mapphone_lvibrator_exit,
	.power_on = mapphone_lvibrator_power_on,
	.power_off = mapphone_lvibrator_power_off,
#ifdef CONFIG_VIB_PWM_SWEEP
	.pattern = mapphone_lvibrator_pattern_on,
#endif /* CONFIG_VIB_PWM_SWEEP */
	.device_name = "lvibrator",
};

static struct platform_device mapphone_vib_pwm = {
	.name = VIB_PWM_NAME,
	.id = -1,
	.dev = {
		.platform_data = &vib_pwm_data,
	},
};

static int mapphone_lvibrator_devtree(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *vib_pwm_node;
	const void *vib_pwm_prop = NULL;
	int len = 0;
#endif
	u8 lvibrator_in_device = 0;

	vib_pwm_enable_gpio = 9;
	vib_pwm_period = 5714;
	vib_pwm_duty = 2857;

#ifdef CONFIG_ARM_OF
	vib_pwm_node = of_find_node_by_path("/System@0/LinearVibrator@0");
	if (vib_pwm_node != NULL) {
		lvibrator_in_device = 1;

		vib_pwm_prop = of_get_property(vib_pwm_node, "supported", &len);
		if (vib_pwm_prop != NULL)
			lvibrator_in_device = *((u8 *)vib_pwm_prop);

		if (lvibrator_in_device != 0) {
			vib_pwm_prop = of_get_property(vib_pwm_node,
					"period", &len);
			if (vib_pwm_prop != NULL)
				vib_pwm_period = *((int *)vib_pwm_prop);

			vib_pwm_prop = of_get_property(vib_pwm_node,
					"cycle", &len);
			if (vib_pwm_prop != NULL)
				vib_pwm_duty = *((int *)vib_pwm_prop);

			vib_pwm_prop = of_get_property(vib_pwm_node,
					"gpio", &len);
			if (vib_pwm_prop != NULL)
				vib_pwm_enable_gpio = *((int *)vib_pwm_prop);

			vib_pwm_prop = of_get_property(vib_pwm_node,
					"linear_vib_only", NULL);
			if (vib_pwm_prop != NULL)
				linear_vib_only = *((int *)vib_pwm_prop);
			else
				pr_err("Read property linear_vib_only "
						"error!\n");
		}

		of_node_put(vib_pwm_node);
	}
#endif
	return lvibrator_in_device;
}

/*
 * SFH7743
 */

static char *prox_get_pwr_supply(void)
{
	char *prox_regulator = "vsdio";
#ifdef CONFIG_ARM_OF
	const void *pwr_supply_prop;
	struct device_node *prox_node =
		of_find_node_by_path("/System@0/PROX@0");

	if (prox_node) {
		pwr_supply_prop = of_get_property(prox_node,
				 "pwr_supply", NULL);
		if (pwr_supply_prop) {
			if (*(u32 *)pwr_supply_prop == 0x0)
				prox_regulator = "vrf1";
			else if	 (*(u32 *)pwr_supply_prop == 0x1)
				prox_regulator = "vsdio";
		}
		of_node_put(prox_node);
	}

#endif
		return prox_regulator;
}

static struct regulator *mapphone_sfh7743_regulator;
static int mapphone_sfh7743_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, prox_get_pwr_supply());
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_sfh7743_regulator = reg;
	return 0;
}

static void mapphone_sfh7743_exit(void)
{
	regulator_put(mapphone_sfh7743_regulator);
}

static int mapphone_sfh7743_power_on(void)
{
	return regulator_enable(mapphone_sfh7743_regulator);
}

static int mapphone_sfh7743_power_off(void)
{
	if (mapphone_sfh7743_regulator)
		return regulator_disable(mapphone_sfh7743_regulator);
	return 0;
}

static struct sfh7743_platform_data mapphone_sfh7743_data = {
	.init = mapphone_sfh7743_initialization,
	.exit = mapphone_sfh7743_exit,
	.power_on = mapphone_sfh7743_power_on,
	.power_off = mapphone_sfh7743_power_off,

	.gpio = MAPPHONE_PROX_INT_GPIO,
};

static void __init mapphone_sfh7743_init(void)
{
	int proximity_gpio = MAPPHONE_PROX_INT_GPIO;
#ifdef CONFIG_ARM_OF
	proximity_gpio = get_gpio_by_name("proximity_int");
	if (proximity_gpio < 0) {
		printk(KERN_DEBUG
		"cannot retrieve proximity_int from device tree\n");
		proximity_gpio = MAPPHONE_PROX_INT_GPIO;
	}
	mapphone_sfh7743_data.gpio = proximity_gpio;
#endif
	gpio_request(proximity_gpio, "sfh7743 proximity int");
	gpio_direction_input(proximity_gpio);
	omap_cfg_reg(Y3_34XX_GPIO180);
}

struct platform_device sfh7743_platform_device = {
	.name = "sfh7743",
	.id = -1,
	.dev = {
		.platform_data = &mapphone_sfh7743_data,
	},
};

/*
 * AIRC
 */

static struct regulator *mapphone_airc_regulator;
static int mapphone_airc_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vsdio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_airc_regulator = reg;
	return 0;
}

static void mapphone_airc_exit(void)
{
	regulator_put(mapphone_airc_regulator);
}

static int mapphone_airc_power_on(void)
{
	return regulator_enable(mapphone_airc_regulator);
}

static int mapphone_airc_power_off(void)
{
	if (mapphone_airc_regulator)
		return regulator_disable(mapphone_airc_regulator);
	return 0;
}

struct airc_platform_data mapphone_airc_data = {
	.init = mapphone_airc_initialization,
	.exit = mapphone_airc_exit,
	.power_on = mapphone_airc_power_on,
	.power_off = mapphone_airc_power_off,

	.gpio = MAPPHONE_PROX_INT_GPIO,
};

#ifdef CONFIG_SENSORS_AIRC
static void __init mapphone_airc_init(void)
{
	gpio_request(MAPPHONE_PROX_INT_GPIO, "airc proximity int");
	gpio_direction_input(MAPPHONE_PROX_INT_GPIO);
	omap_cfg_reg(Y3_34XX_GPIO180);
}
#endif

/*
 * LIS331DLH
 */

static struct regulator *mapphone_lis331dlh_regulator;
static int mapphone_lis331dlh_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_lis331dlh_regulator = reg;
	return 0;
}

static void mapphone_lis331dlh_exit(void)
{
	regulator_put(mapphone_lis331dlh_regulator);
}

static int mapphone_lis331dlh_power_on(void)
{
	return regulator_enable(mapphone_lis331dlh_regulator);
}

static int mapphone_lis331dlh_power_off(void)
{
	if (mapphone_lis331dlh_regulator)
		return regulator_disable(mapphone_lis331dlh_regulator);
	return 0;
}

struct lis331dlh_platform_data mapphone_lis331dlh_data = {
	.init = mapphone_lis331dlh_initialization,
	.exit = mapphone_lis331dlh_exit,
	.power_on = mapphone_lis331dlh_power_on,
	.power_off = mapphone_lis331dlh_power_off,

	.min_interval	= 1,
	.poll_interval	= 200,

	.g_range	= LIS331DLH_G_8G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,
};

/*
 * KXTF9
 */

#define MAPPHONE_KXTF9_INT_GPIO		22

static struct regulator *mapphone_kxtf9_regulator;
static int mapphone_kxtf9_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_kxtf9_regulator = reg;
	return 0;
}

static void mapphone_kxtf9_exit(void)
{
	regulator_put(mapphone_kxtf9_regulator);
}

static int mapphone_kxtf9_power_on(void)
{
	return regulator_enable(mapphone_kxtf9_regulator);
}

static int mapphone_kxtf9_power_off(void)
{
	if (mapphone_kxtf9_regulator)
		return regulator_disable(mapphone_kxtf9_regulator);
	return 0;
}

struct kxtf9_platform_data mapphone_kxtf9_data = {
	.init = mapphone_kxtf9_initialization,
	.exit = mapphone_kxtf9_exit,
	.power_on = mapphone_kxtf9_power_on,
	.power_off = mapphone_kxtf9_power_off,

	.min_interval	= 2,
	.poll_interval	= 200,

	.g_range	= KXTF9_G_8G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,

	.data_odr_init		= ODR12_5,
	.ctrl_reg1_init		= RES_12BIT | KXTF9_G_2G | TPE | WUFE | TDTE,
	.int_ctrl_init		= IEA | IEN,
	.tilt_timer_init	= 0x03,
	.engine_odr_init	= OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init		= 0x0A,
	.wuf_thresh_init	= 0x20,
	.tdt_timer_init		= 0x78,
	.tdt_h_thresh_init	= 0xB6,
	.tdt_l_thresh_init	= 0x1A,
	.tdt_tap_timer_init	= 0xA2,
	.tdt_total_timer_init	= 0x24,
	.tdt_latency_timer_init	= 0x28,
	.tdt_window_timer_init	= 0xA0,

	.gpio = MAPPHONE_KXTF9_INT_GPIO,
	.gesture = 0,
	.sensitivity_low = {
		0x50, 0xFF, 0xB8, 0x32, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_medium = {
		0x50, 0xFF, 0x68, 0x32, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_high = {
		0x78, 0xB6, 0x1A, 0xA2, 0x24, 0x28, 0xA0,
	},
};

static ssize_t kxtf9_sysfs_show(struct class *dev, char *buf)
{
	int data;
	char *str = buf;
	ssize_t count;

	data = gpio_get_value(MAPPHONE_KXTF9_INT_GPIO);
	str += sprintf(str, "%d\n", data);
	count = (ssize_t) (str - buf);
	return count;
}

static CLASS_ATTR(kxtf9, S_IRUGO, kxtf9_sysfs_show, NULL);

static void __init mapphone_kxtf9_init(void)
{
	struct class *class;

#ifdef CONFIG_ARM_OF
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_path(DT_PATH_ACCELEROMETER);
	if (node) {
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_X, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_x = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_Y, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_y = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_Z, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_z = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_X, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_x = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_Y, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_y = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_Z, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_z = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_LOW, &len);
		if (prop && len)
				memcpy(mapphone_kxtf9_data.sensitivity_low,
						(u8 *)prop, len);
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_MEDIUM, &len);
		if (prop && len)
				memcpy(mapphone_kxtf9_data.sensitivity_medium,
						(u8 *)prop, len);
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_HIGH, &len);
		if (prop && len)
				memcpy(mapphone_kxtf9_data.sensitivity_high,
						(u8 *)prop, len);
		of_node_put(node);
	}
#endif
	gpio_request(MAPPHONE_KXTF9_INT_GPIO, "kxtf9 accelerometer int");
	gpio_direction_input(MAPPHONE_KXTF9_INT_GPIO);
	omap_cfg_reg(AF9_34XX_GPIO22_DOWN);

	class = class_create(THIS_MODULE, "kxtf9");
	if (IS_ERR(class))
		printk(KERN_ERR "kxtf9 can't register class\n");
	else if (class_create_file(class, &class_attr_kxtf9)) {
		printk(KERN_ERR "kxtf9: can't create sysfs\n");
		class_destroy(class);
	}

}

/*
 * AKM8975
 */

static struct regulator *mapphone_akm8975_regulator;

static int mapphone_akm8975_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_akm8975_regulator = reg;
	return 0;
}

static void mapphone_akm8975_exit(void)
{
	if (mapphone_akm8975_regulator)
		regulator_put(mapphone_akm8975_regulator);
}

static int mapphone_akm8975_power_on(void)
{
	if (mapphone_akm8975_regulator)
		return regulator_enable(mapphone_akm8975_regulator);
	return 0;
}

static int mapphone_akm8975_power_off(void)
{
	if (mapphone_akm8975_regulator)
		return regulator_disable(mapphone_akm8975_regulator);
	return 0;
}

struct akm8975_platform_data mapphone_akm8975_pdata = {
	.init = mapphone_akm8975_initialization,
	.exit = mapphone_akm8975_exit,
	.power_on = mapphone_akm8975_power_on,
	.power_off = mapphone_akm8975_power_off,
};

static void __init mapphone_akm8975_init(void)
{
	int akm8975_reset_gpio = -1;
	int akm8975_int_gpio = -1;

	akm8975_reset_gpio = get_gpio_by_name("akm8975_reset");
	if (akm8975_reset_gpio < 0) {
		printk(KERN_DEBUG "cannot retrieve akm8975_reset from device tree\n");
		return;
	}

	akm8975_int_gpio = get_gpio_by_name("akm8975_int");
	if (akm8975_int_gpio < 0) {
		printk(KERN_DEBUG "cannot retrieve akm8975_int from device tree\n");
		return;
	}

	gpio_request(akm8975_reset_gpio, "akm8975 reset");
	gpio_direction_output(akm8975_reset_gpio, 1);

	gpio_request(akm8975_int_gpio, "akm8975 irq");
	gpio_direction_input(akm8975_int_gpio);
}

/*
 * AKM8973
 */

#define MAPPHONE_AKM8973_INT_GPIO	175
#define MAPPHONE_AKM8973_RESET_GPIO	28

static void __init mapphone_akm8973_init(void)
{
	gpio_request(MAPPHONE_AKM8973_RESET_GPIO, "akm8973 reset");
	gpio_direction_output(MAPPHONE_AKM8973_RESET_GPIO, 1);
	omap_cfg_reg(AB10_34XX_GPIO28_OUT);

	gpio_request(MAPPHONE_AKM8973_INT_GPIO, "akm8973 irq");
	gpio_direction_input(MAPPHONE_AKM8973_INT_GPIO);
	omap_cfg_reg(AC3_34XX_GPIO175);
}

/*
 * ISL29030
 */

struct isl29030_platform_data isl29030_pdata = {
	.configure = 0x62,
	.interrupt_cntrl = 0x20,
	.prox_lower_threshold = 0x1e,
	.prox_higher_threshold = 0x32,
	.crosstalk_vs_covered_threshold = 0x30,
	.default_prox_noise_floor = 0x30,
	.num_samples_for_noise_floor = 0x05,
	.lens_percent_t = 20,
	.regulator_name = {0},
};

static ssize_t isl29030_sysfs_show(struct class *dev, char *buf)
{
	int data;
	char *str = buf;
	ssize_t count;
	int gpio = get_gpio_by_name("als_int");

	if (gpio < 0) {
		printk(KERN_DEBUG "can't retrieve als_int.\n");
		return -ENODEV;
	}

	data = gpio_get_value(gpio);
	str += sprintf(str, "%d\n", data);
	count = (ssize_t) (str - buf);
	return count;
}

static CLASS_ATTR(isl29030, S_IRUGO, isl29030_sysfs_show, NULL);

static int mapphone_isl29030_init(void)
{
	int err = 0;
#ifdef CONFIG_ARM_OF
	struct device_node *prox_node = NULL;
	const void *prop = NULL;
	const char *prop_str = NULL;
	int len = 0;
	int i;
	int gpio = 0;
	struct class *class;
	prox_node = of_find_node_by_path(DT_PATH_PROX);
	if (prox_node != NULL) {
		prox_sensor_type = PROXIMITY_ISL29030;
		pr_err("%s - opened node %s from device tree\n",
			__func__, DT_PATH_PROX);

		prop_str = DT_PROP_ISL29030_CONF;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.configure = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_INT_CNTL;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
					 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.interrupt_cntrl = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_PROX_LOW_TH;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.prox_lower_threshold = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_PROX_HIGH_TH;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.prox_higher_threshold = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_XTALK_V_COV_TH;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.crosstalk_vs_covered_threshold =
				*(u8 *)prop;

		prop_str = DT_PROP_ISL29030_DEF_PROX_NOISE;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.default_prox_noise_floor = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_NUM_SAMP_NOISE;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.num_samples_for_noise_floor =
				*(u8 *)prop;

		prop_str = DT_PROP_ISL29030_LENS_PERCENT;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_err("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
			err = -1;
		} else
			isl29030_pdata.lens_percent_t = *(u8 *)prop;

		prop_str = DT_PROP_ISL29030_REGULATOR;
		prop = of_get_property(prox_node, prop_str, &len);
		if ((prop == NULL) || (len == 0)) {
			pr_info("%s - unable to read %s from device tree\n",
				 __func__, prop_str);
		} else {
			if (len > ISL29030_REGULATOR_NAME_LENGTH) {
				pr_err("%s - %s entry %s too long\n",
					__func__, prop_str, (char *)prop);
				/* Truncation should cause driver to err out,
				 * unless truncated name is valid */
				len = ISL29030_REGULATOR_NAME_LENGTH - 1;
			}
			for (i = 0; i < len; i++)
				isl29030_pdata.regulator_name[i] =
					((char *)prop)[i];
		}

		of_node_put(prox_node);
	} else {
		pr_err("%s - unable to read %s node from device tree.\n",
			__func__, DT_PATH_PROX);
		err = -1;
	}

	gpio = get_gpio_by_name("als_int");
	if (gpio >= 0) {
		isl29030_pdata.irq = gpio_to_irq(gpio);
		gpio_request(gpio, "isl29030 proximity int");
		gpio_direction_input(gpio);
		class = class_create(THIS_MODULE, "isl29030");
		if (IS_ERR(class))
			printk(KERN_ERR "isl29030 can't register class\n");
		else if (class_create_file(class, &class_attr_isl29030) != 0) {
			printk(KERN_ERR "isl29030: can't create sysfs\n");
			class_destroy(class);
		}
	}
#endif /* CONFIG_ARM_OF */
	return err;
}

/*
 * BU52014HFV
 */

#define MAPPHONE_HF_NORTH_GPIO		10
#define MAPPHONE_HF_SOUTH_GPIO		111

static struct bu52014hfv_platform_data bu52014hfv_platform_data = {
	.docked_north_gpio = MAPPHONE_HF_NORTH_GPIO,
	.docked_south_gpio = MAPPHONE_HF_SOUTH_GPIO,
	.north_is_desk = 1,
};

static void mapphone_bu52014hfv_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_HALLEFFECT_DOCK);

	if (node == NULL)
		return;

	prop = of_get_property(node, DT_PROP_DEV_NORTH_IS_DESK, NULL);

	if (prop)
		bu52014hfv_platform_data.north_is_desk = *(u8 *)prop;

	of_node_put(node);
#endif

	gpio_request(MAPPHONE_HF_NORTH_GPIO, "mapphone dock north");
	gpio_direction_input(MAPPHONE_HF_NORTH_GPIO);
	omap_cfg_reg(AG25_34XX_GPIO10);

	gpio_request(MAPPHONE_HF_SOUTH_GPIO, "mapphone dock south");
	gpio_direction_input(MAPPHONE_HF_SOUTH_GPIO);
	omap_cfg_reg(B26_34XX_GPIO111);
}

static struct platform_device omap3430_hall_effect_dock = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52014hfv_platform_data,
	},
};

/*
 * ADP8870
 */

static void __init mapphone_adp8870_init(void)
{
#ifdef CONFIG_ARM_OF
	int adp8870_reset_gpio = get_gpio_by_name("adp8870_reset");

	if (adp8870_reset_gpio < 0) {
		pr_err("adp8870_reset not in device_tree\n");
		return;
	}

	gpio_request(adp8870_reset_gpio, "adp8870 reset");
	gpio_direction_output(adp8870_reset_gpio, 1);
#endif
}

static struct led_info adp8870_leds[] = {
	{
		.name = "adp8870-led7",
		.default_trigger = "none",
		.flags = ADP8870_LED_D7 | ADP8870_LED_DIS_BLINK,
	},
};

struct adp8870_backlight_platform_data adp8870_pdata = {
	.bl_led_assign = ADP8870_BL_D1 | ADP8870_BL_D2 | ADP8870_BL_D3 |
		ADP8870_BL_D4 | ADP8870_BL_D5 | ADP8870_BL_D6,
	.pwm_assign = 0,

	.bl_fade_in = ADP8870_FADE_T_DIS,
	.bl_fade_out = ADP8870_FADE_T_DIS,
	.bl_fade_law = ADP8870_FADE_LAW_LINEAR,

	.en_ambl_sens = 0,
	.abml_filt = ADP8870_BL_AMBL_FILT_320ms,

	.l1_daylight_max = ADP8870_BL_CUR_mA(18),
	.l1_daylight_dim = ADP8870_BL_CUR_mA(0),
	.l2_bright_max = ADP8870_BL_CUR_mA(14),
	.l2_bright_dim = ADP8870_BL_CUR_mA(0),
	.l3_office_max = ADP8870_BL_CUR_mA(6),
	.l3_office_dim = ADP8870_BL_CUR_mA(0),
	.l4_indoor_max = ADP8870_BL_CUR_mA(3),
	.l4_indor_dim = ADP8870_BL_CUR_mA(0),
	.l5_dark_max = ADP8870_BL_CUR_mA(2),
	.l5_dark_dim = ADP8870_BL_CUR_mA(0),

	.l2_trip = ADP8870_L2_COMP_CURR_uA(710),
	.l2_hyst = ADP8870_L2_COMP_CURR_uA(73),
	.l3_trip = ADP8870_L3_COMP_CURR_uA(389),
	.l3_hyst = ADP8870_L3_COMP_CURR_uA(54),
	.l4_trip = ADP8870_L4_COMP_CURR_uA(167),
	.l4_hyst = ADP8870_L4_COMP_CURR_uA(16),
	.l5_trip = ADP8870_L5_COMP_CURR_uA(43),
	.l5_hyst = ADP8870_L5_COMP_CURR_uA(11),

	.leds = adp8870_leds,
	.num_leds = ARRAY_SIZE(adp8870_leds),
	.led_fade_law = ADP8870_FADE_LAW_LINEAR,
	.led_fade_in = ADP8870_FADE_T_DIS,
	.led_fade_out = ADP8870_FADE_T_DIS,
	.led_on_time = ADP8870_LED_ONT_200ms,
};

/*
 * Sensors
 */

void __init mapphone_sensors_init(void)
{
	mapphone_kxtf9_init();

	mapphone_isl29030_init();
	if (prox_sensor_type == PROXIMITY_SFH7743) {
		mapphone_sfh7743_init();
		platform_device_register(&sfh7743_platform_device);
	}

	mapphone_bu52014hfv_init();

	mapphone_akm8973_init();
	mapphone_akm8975_init();
	mapphone_adp8870_init();

#ifdef CONFIG_SENSORS_AIRC
	mapphone_airc_init();
#endif
	platform_device_register(&omap3430_hall_effect_dock);

	linear_vib_only = 0;
	if (mapphone_lvibrator_devtree()) {
		if (linear_vib_only == 1)
			vib_pwm_data.device_name = "vibrator";
	if (mapphone_lvibrator_devtree())
		platform_device_register(&mapphone_vib_pwm);
	}
	if (linear_vib_only == 0) {
		mapphone_vibrator_init();
		platform_device_register(&mapphone_vib_gpio);
	}
}
