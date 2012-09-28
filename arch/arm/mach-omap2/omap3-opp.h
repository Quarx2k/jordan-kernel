#ifndef __OMAP3_OPP_H_
#define __OMAP3_OPP_H_

#include <plat/omap-pm.h>

/* MPU speeds */
#define S1200M	1200000000
#define S1000M	1000000000
#define S800M	800000000
#define S720M   720000000
#define S600M   600000000
#define S550M   550000000
#define S520M   520000000
#define S500M   500000000
#define S300M	300000000
#define S250M   250000000
#define S150M	150000000
#define S125M   125000000

/* DSP speeds */
#define S875M   875000000
#define S760M   760000000
#define S660M   660000000
#define S520M	520000000
#define S430M   430000000
#define S400M   400000000
#define S360M   360000000
#define S260M	260000000
#define S180M   180000000
#define S130M	130000000
#define S90M    90000000
#define S65M    65000000

/* L3 speeds */
#define S50M	50000000
#define S83M    83000000
#define S100M	100000000
#define S166M   166000000
#define S200M	200000000

static struct omap_opp omap3630_mpu_rate_table[] = {
	{0, 0, 0},
	/*OPP1 (OPP50) - 0.93mV*/
	{S300M, VDD1_OPP1, 0x1b},
	/*OPP2 (OPP100) - 1.1V*/
	{S600M, VDD1_OPP2, 0x28},
	/*OPP3 (OPP130) - 1.26V*/
	{S800M, VDD1_OPP3, 0x35},
	/*OPP4 (OPP-1G) - 1.35V*/
	{S1000M, VDD1_OPP4, 0x3C},

};

static struct omap_opp omap3630_l3_rate_table[] = {
	{0, 0, 0},
	/*OPP1 (OPP50) - 0.93V*/
	{S100M, VDD2_OPP1, 0x1b},
	/*OPP2 (OPP100) - 1.1375V*/
	{S200M, VDD2_OPP2, 0x2b},
};

static struct omap_opp omap3630_dsp_rate_table[] = {
	{0, 0, 0},
	/*OPP1 (OPP50) - 0.93V*/
	{S260M, VDD1_OPP1, 0x1b},
	/*OPP2 (OPP100) - 1.1V*/
	{S520M, VDD1_OPP2, 0x28},
	/*OPP3 (OPP130) - 1.26V*/
	{S660M, VDD1_OPP3, 0x35},
	/*OPP4 (OPP-1G) - 1.35V*/
	{S800M, VDD1_OPP4, 0x3C},
};

static struct omap_opp omap3_mpu_rate_table[] = {
	{0, 0, 0},
	/*OPP1*/
	{S125M, VDD1_OPP1, 0x1E},
	/*OPP2*/
	{S250M, VDD1_OPP2, 0x26},
	/*OPP3*/
	{S500M, VDD1_OPP3, 0x30},
	/*OPP4*/
	{S550M, VDD1_OPP4, 0x36},
	/*OPP5*/
	{S600M, VDD1_OPP5, 0x3C},
	/*OPP6*/
	{S720M, VDD1_OPP6, 0x3C},
};

static struct omap_opp omap3_l3_rate_table[] = {
	{0, 0, 0},
	/*OPP1*/
	{0, VDD2_OPP1, 0x1E},
	/*OPP2*/
	{S83M, VDD2_OPP2, 0x24},
	/*OPP3*/
	{S166M, VDD2_OPP3, 0x2C},
};

/* iva rate table for 3420 */
static struct omap_opp omap3_dsp_rate_table_3420[] = {
	{0, 0, 0},
	/*OPP1*/
	{S90M, VDD1_OPP1, 0x1E},
	/*OPP2*/
	{S180M, VDD1_OPP2, 0x26},
	/*OPP3*/
	{S360M, VDD1_OPP3, 0x30},
	/*OPP4*/
	{S360M, VDD1_OPP4, 0x36},
	/*OPP5*/
	{S360M, VDD1_OPP5, 0x36},
};

/* iva rate table for 3430 */
static struct omap_opp omap3_dsp_rate_table[] = {
	{0, 0, 0},
	/*OPP1*/
	{S90M, VDD1_OPP1, 0x1E},
	/*OPP2*/
	{S180M, VDD1_OPP2, 0x26},
	/*OPP3*/
	{S360M, VDD1_OPP3, 0x30},
	/*OPP4*/
	{S430M, VDD1_OPP4, 0x36},
	/*OPP5*/
	{S430M, VDD1_OPP5, 0x3C},
};

/* iva rate table for 3430 */
static struct omap_opp omap3_dsp_rate_table_3440[] = {
	{0, 0, 0},
	/*OPP1*/
	{S90M, VDD1_OPP1, 0x1E},
	/*OPP2*/
	{S180M, VDD1_OPP2, 0x26},
	/*OPP3*/
	{S360M, VDD1_OPP3, 0x30},
	/*OPP4*/
	{S430M, VDD1_OPP4, 0x36},
	/*OPP5*/
	{S430M, VDD1_OPP5, 0x3C},
	/*OPP6*/
	{S520M, VDD1_OPP6, 0x3C},
};

#endif
