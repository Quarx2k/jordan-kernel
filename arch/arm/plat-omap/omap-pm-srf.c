/*
 * omap-pm-srf.c - OMAP power management interface implemented
 * using Shared resource framework
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 * Copyright (C) 2008-2009 Nokia Corporation
 * Rajendra Nayak
 *
 * This code is based on plat-omap/omap-pm-noop.c.
 *
 * Interface developed by (in alphabetical order):
 * Karthik Dasu, Tony Lindgren, Rajendra Nayak, Sakari Poussa, Veeramanikandan
 * Raju, Anand Sawant, Igor Stoppa, Paul Walmsley, Richard Woodruff
 */

#undef DEBUG

#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/module.h>

#include <plat/omap-pm.h>
#include <plat/powerdomain.h>
#include <plat/resource.h>
#include <plat/omap_device.h>
#include <plat/omap34xx.h>

struct omap_opp *dsp_opps;
struct omap_opp *mpu_opps;
struct omap_opp *l3_opps;

#define LAT_RES_POSTAMBLE "_latency"
#define MAX_LATENCY_RES_NAME 30

/**
 * get_lat_res_name - gets the latency resource name given a power domain name
 * @pwrdm_name: Name of the power domain.
 * @lat_name: Buffer in which latency resource name is populated
 * @size: Max size of the latency resource name
 *
 * Returns the latency resource name populated in lat_name.
 */
void get_lat_res_name(const char *pwrdm_name, char **lat_name, int size)
{
	strcpy(*lat_name, "");
	WARN_ON(strlen(pwrdm_name) + strlen(LAT_RES_POSTAMBLE) > size);
	strcpy(*lat_name, pwrdm_name);
	strcat(*lat_name, LAT_RES_POSTAMBLE);
	return;
}

/*
 * Device-driver-originated constraints (via board-*.c files)
 */

void omap_pm_set_max_mpu_wakeup_lat(struct device *dev, long t)
{
	if (!dev || t < -1) {
		WARN_ON(1);
		return;
	};

	if (t == -1) {
		pr_debug("OMAP PM: remove max MPU wakeup latency constraint: "
			 "dev %s\n", dev_name(dev));
		resource_release("mpu_latency", dev);
	} else {
		pr_debug("OMAP PM: add max MPU wakeup latency constraint: "
			 "dev %s, t = %ld usec\n", dev_name(dev), t);
		resource_request("mpu_latency", dev, t);
	}
}

void omap_pm_set_min_bus_tput(struct device *dev, u8 agent_id, unsigned long r)
{
	if (!dev || (agent_id != OCP_INITIATOR_AGENT &&
	    agent_id != OCP_TARGET_AGENT)) {
		WARN_ON(1);
		return;
	};

	if (r == 0) {
		pr_debug("OMAP PM: remove min bus tput constraint: "
			 "dev %s for agent_id %d\n", dev_name(dev), agent_id);
		resource_release("vdd2_opp", dev);
	} else {
		pr_debug("OMAP PM: add min bus tput constraint: "
			 "dev %s for agent_id %d: rate %ld KiB\n",
			 dev_name(dev), agent_id, r);
		resource_request("vdd2_opp", dev, r);
	}
}
EXPORT_SYMBOL(omap_pm_set_min_bus_tput);

void omap_pm_set_max_dev_wakeup_lat(struct device *dev, long t)
{
	struct omap_device *odev;
	struct powerdomain *pwrdm_dev;
	struct platform_device *pdev;
	char *lat_res_name;

	if (!dev || t < -1) {
		WARN_ON(1);
		return;
	};
	/* Look for the devices Power Domain */
	/*
	 * WARNING! If device is not a platform device, container_of will
	 * return a pointer to unknown memory!
	 * TODO: Either change omap-pm interface to support only platform
	 * devices, or change the underlying omapdev implementation to
	 * support normal devices.
	 */
	pdev = container_of(dev, struct platform_device, dev);

	/* Try to catch non platform devices. */
	if (pdev->name == NULL) {
		printk(KERN_ERR "OMAP-PM: Error: platform device not valid\n");
		return;
	}

	odev = to_omap_device(pdev);
	if (odev) {
		pwrdm_dev = omap_device_get_pwrdm(odev);
	} else {
		printk(KERN_ERR "OMAP-PM: Error: Could not find omap_device "
						"for %s\n", pdev->name);
		return;
	}

	lat_res_name = kmalloc(MAX_LATENCY_RES_NAME, GFP_KERNEL);
	if (!lat_res_name) {
		printk(KERN_ERR "OMAP-PM: FATAL ERROR: kmalloc failed\n");
		return;
	}
	get_lat_res_name(pwrdm_dev->name, &lat_res_name, MAX_LATENCY_RES_NAME);

	if (t == -1) {
		pr_debug("OMAP PM: remove max device latency constraint: "
			 "dev %s\n", dev_name(dev));
		resource_release(lat_res_name, dev);
	} else {
		pr_debug("OMAP PM: add max device latency constraint: "
			 "dev %s, t = %ld usec\n", dev_name(dev), t);
		resource_request(lat_res_name, dev, t);
	}

	kfree(lat_res_name);
	return;
}

void omap_pm_set_max_sdma_lat(struct device *dev, long t)
{
	if (!dev || t < -1) {
		WARN_ON(1);
		return;
	};

	if (t == -1) {
		pr_debug("OMAP PM: remove max DMA latency constraint: "
			 "dev %s\n", dev_name(dev));
		resource_release("core_latency", dev);
	} else {
		pr_debug("OMAP PM: add max DMA latency constraint: "
			 "dev %s, t = %ld usec\n", dev_name(dev), t);
		resource_request("core_latency", dev, t);
	}
}

static struct device dummy_dsp_dev;

/*
 * DSP Bridge-specific constraints
 */
const struct omap_opp *omap_pm_dsp_get_opp_table(void)
{
	pr_debug("OMAP PM: DSP request for OPP table\n");

	/*
	 * Return DSP frequency table here:  The final item in the
	 * array should have .rate = .opp_id = 0.
	 */

	return dsp_opps;
}

static struct device dummy_vdd1_dev;
void omap_pm_vdd1_set_max_opp(u8 opp_id)
{
	pr_debug("OMAP PM: requests constraint for max OPP ID\n");

	if (opp_id != 0)
		resource_request("vdd1_max", &dummy_vdd1_dev, opp_id);
	 else
		resource_request("vdd1_max", &dummy_vdd1_dev, MAX_VDD1_OPP);
}
EXPORT_SYMBOL(omap_pm_vdd1_set_max_opp);

static bool vdd1_max_opp;

void omap_pm_dsp_set_min_opp(struct device *dev, unsigned long f)
{
	u8 opp_id;

	if (!dev) {
		WARN_ON(1);
		return;
	};

	pr_debug("OMAP PM: DSP requests minimum VDD1 opp, \
			 dsp freq requested is %lu\n", f);

	/* DSP uses KHz clock. */
	f *= 1000;
	if (cpu_is_omap3630() && (omap_rev_id() == OMAP_3630_1200)) {

		unsigned long min_dsp_freq = dsp_opps[MIN_VDD1_OPP].rate;
		int dsp_target;

		/* Set Vdd1 Max Constraint according to dsp freq*/

		/*
		 * check if dsp freq requested is above 65MHz, if yes set
		 * max opp to 4, which limits scaling to max VDD1 1G only.
		 * if dsp freq drops below opp1 260M, release constraint,
		 * max vdd1 1.2G can be reached.
		 */
		if ((f > min_dsp_freq) && !vdd1_max_opp) {
			vdd1_max_opp = 1;
			omap_pm_vdd1_set_max_opp(MAX_VDD1_OPP - 1);
		} else if ((f <= min_dsp_freq) && vdd1_max_opp) {
			omap_pm_vdd1_set_max_opp(0);
			vdd1_max_opp = 0;
		}
		/*
		 * DSP table has 65MHz as OPP5, give OPP1-260MHz
		 * when DSP request 65MHz.
		 */
		if (f == min_dsp_freq) {
			dsp_target =  get_opp_from_target_level(dsp_opps,
							 VDD1_OPP1);
			f = dsp_opps[dsp_target].rate;
		}
	}

	/* Get opp id to set VDD1 constraint*/
	opp_id = get_opp_id(dsp_opps + MAX_VDD1_OPP, f);
	/*
	 * For now pass a dummy_dev struct for SRF to identify the caller.
	 * Maybe its good to have DSP pass this as an argument
	 */
	resource_request("vdd1_opp", &dummy_dsp_dev, opp_id);
	return;
}

u8 omap_pm_dsp_get_opp(void)
{
	pr_debug("OMAP PM: DSP requests current DSP OPP ID\n");
	return resource_get_level("vdd1_opp");
}

u8 omap_pm_vdd1_get_opp(void)
{
	pr_debug("OMAP PM: User requests current VDD1 OPP\n");
	return resource_get_level("vdd1_opp");
}

u8 omap_pm_vdd2_get_opp(void)
{
	pr_debug("OMAP PM: User requests current VDD2 OPP\n");
	return resource_get_level("vdd2_opp");
}

/*
 * CPUFreq-originated constraint
 *
 * In the future, this should be handled by custom OPP clocktype
 * functions.
 */

struct cpufreq_frequency_table **omap_pm_cpu_get_freq_table(void)
{
	pr_debug("OMAP PM: CPUFreq request for frequency table\n");

	/*
	 * Return CPUFreq frequency table here: loop over
	 * all VDD1 clkrates, pull out the mpu_ck frequencies, build
	 * table
	 */

	return NULL;
}

static struct device dummy_cpufreq_dev;

void omap_pm_cpu_set_freq(unsigned long f)
{
	if (f == 0) {
		WARN_ON(1);
		return;
	}

	pr_debug("OMAP PM: CPUFreq requests CPU frequency to be set to %lu\n",
		 f);

	resource_request("mpu_freq", &dummy_cpufreq_dev, f);
	return;
}

void omap_pm_set_min_mpu_freq(struct device *dev, unsigned long f)
{
	if (f < 0) {
		WARN_ON(1);
		return;
	}

	if (f == 0) {
		pr_debug("OMAP PM: remove CPU frequency minimal request\n");
		resource_release("mpu_freq", dev);
	} else {
		pr_debug("OMAP PM: add CPU frequency minimal request to %lu\n",
			f);
		resource_request("mpu_freq", dev, f);
	}
	return;
}
EXPORT_SYMBOL(omap_pm_set_min_mpu_freq);

unsigned long omap_pm_cpu_get_freq(void)
{
	pr_debug("OMAP PM: CPUFreq requests current CPU frequency\n");
	return resource_get_level("mpu_freq");
}

/*
 * Device context loss tracking
 */

int omap_pm_get_dev_context_loss_count(struct device *dev)
{
	struct platform_device *pdev;
	struct omap_device *odev;
	struct powerdomain *pwrdm;

	if (!dev) {
		WARN_ON(1);
		return -EINVAL;
	};

	pr_debug("OMAP PM: returning context loss count for dev %s\n",
		 dev_name(dev));

	/*
	 * Map the device to the powerdomain.  Return the powerdomain
	 * off counter.
	 */
	pdev = to_platform_device(dev);
	odev = to_omap_device(pdev);

	if (odev) {
		pwrdm = omap_device_get_pwrdm(odev);
		if (pwrdm)
			return pwrdm->state_counter[0];
	}
	return 0;
}

/*
 * Must be called before clk framework init
 */
int __init omap_pm_if_early_init(struct omap_opp *mpu_opp_table,
				 struct omap_opp *dsp_opp_table,
				 struct omap_opp *l3_opp_table)
{
	mpu_opps = mpu_opp_table;
	dsp_opps = dsp_opp_table;
	l3_opps = l3_opp_table;
	return 0;
}

/* Must be called after clock framework is initialized */
int __init omap_pm_if_init(void)
{
	resource_init(resources_omap);
	return 0;
}

void omap_pm_if_exit(void)
{
	/* Deallocate CPUFreq frequency table here */
}

u8 omap_pm_get_max_vdd1_opp()
{
	if (cpu_is_omap3630()) {
		switch (omap_rev_id()) {
		case OMAP_3630:
		default:
			return VDD1_OPP4;
		case OMAP_3630_0800:
			return VDD1_OPP3;
		case OMAP_3630_1000:
			return VDD1_OPP4;
		case OMAP_3630_1200:
			return VDD1_OPP5;
		}
	} else {
		if (omap_rev() < OMAP3430_REV_ES3_1)
			return VDD1_OPP5;
		else {
			switch (omap_rev_id()) {
			case OMAP_3420:
			case OMAP_3430:
				return VDD1_OPP5;
			case OMAP_3440:
				return VDD1_OPP6;
			default:
				return VDD1_OPP5;
			}
		}
	}
}
EXPORT_SYMBOL(omap_pm_get_max_vdd1_opp);

u8 omap_pm_get_min_vdd1_opp(void)
{
	return VDD1_OPP1;
}
EXPORT_SYMBOL(omap_pm_get_min_vdd1_opp);


u8 omap_pm_get_max_vdd2_opp(void)
{
	if (cpu_is_omap3630())
		return VDD2_OPP2;
	else
		return VDD2_OPP3;
}
EXPORT_SYMBOL(omap_pm_get_max_vdd2_opp);

u8 omap_pm_get_min_vdd2_opp(void)
{
	if (cpu_is_omap3630())
		return VDD2_OPP1;
	else
		return VDD2_OPP2;
}
EXPORT_SYMBOL(omap_pm_get_min_vdd2_opp);

struct omap_opp *omap_get_mpu_rate_table()
{
	return mpu_opps;
}
EXPORT_SYMBOL(omap_get_mpu_rate_table);

struct omap_opp *omap_get_dsp_rate_table()
{
	return dsp_opps;
}
EXPORT_SYMBOL(omap_get_dsp_rate_table);
