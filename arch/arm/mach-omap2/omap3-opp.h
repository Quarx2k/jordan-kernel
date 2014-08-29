#ifndef __OMAP3_OPP_H_
#define __OMAP3_OPP_H_

#include <plat/omap-pm.h>

static struct omap_opp omap3630_mpu_rate_table[] = {
        {0, 0, 0, 0},
        /*Add headroom for CPCAP IR drop*/
        /*OPP1,CPCAP 1.0125v*/
        {S300M, VDD1_OPP1, 0x22, 0x0},
        /*OPP2,CPCAP 1.2v*/
        {S600M, VDD1_OPP2, 0x30, 0x0},
        /*OPP3,CPCAP 1.325v*/
        {S800M, VDD1_OPP3, 0x3E, 0x0},
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
        {S660M, VDD1_OPP3, 0x3F, 0x0},
        /*OPP4,CPCAP 1.375v*/
        {S875M, VDD1_OPP4, 0x3F, 0x0},
        /*OPP5,CPCAP 1.375v*/
        {S65M, VDD1_OPP5, 0x3E, 0x0},
};

#endif
