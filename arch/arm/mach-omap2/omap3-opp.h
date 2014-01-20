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
        {0, 0, 0, 0},
        /*Add headroom for CPCAP IR drop*/
        /*OPP1,CPCAP 1.0125v*/
        {S300M, VDD1_OPP1, 0x21, 0x0},
        /*OPP2,CPCAP 1.2v*/
        {S600M, VDD1_OPP2, 0x30, 0x0},
        /*OPP3,CPCAP 1.325v*/
        {S800M, VDD1_OPP3, 0x3A, 0x0},
        /*OPP4,CPCAP 1.375v*/
        {S1000M, VDD1_OPP4, 0x3E, 0x0},
        /*OPP5,CPCAP 1.375v*/
        {S1200M, VDD1_OPP5, 0x3E, 0x0},
};

static struct omap_opp omap3630_l3_rate_table[] = {
        {0, 0, 0, 0},
        /*OPP1*/
        {S100M, VDD2_OPP1, 0x20, 0x0},
        /*OPP2*/
        {S200M, VDD2_OPP2, 0x30, 0x0},
};

static struct omap_opp omap3630_dsp_rate_table[] = {
        {0, 0, 0, 0},
        /*OPP1,CPCAP 1.0125v*/
        {S260M, VDD1_OPP1, 0x21, 0x0},
        /*OPP2,CPCAP 1.2v*/
        {S520M, VDD1_OPP2, 0x30, 0x0},
        /*OPP3,CPCAP 1.325v*/
        {S660M, VDD1_OPP3, 0x3A, 0x0},
        /*OPP4,CPCAP 1.375v*/
        {S800M, VDD1_OPP4, 0x3E, 0x0},
        /*OPP5,CPCAP 1.375v*/
        {S65M, VDD1_OPP5, 0x3E, 0x0},
};

#endif
