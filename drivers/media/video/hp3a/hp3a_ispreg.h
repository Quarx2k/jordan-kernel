/*
 * drivers/media/video/hp3a/hp3a_ispreg.h
 *
 * HP Imaging/3A ISP register definitions.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
 *
 * Contributors:
 *		Tanvir Islam <tanvir.islam@hp.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#ifndef	__HP3A_ISP_REG_H_INCLUDED
#define	__HP3A_ISP_REG_H_INCLUDED

#define ISP_32B_BOUNDARY_BUF            0xFFFFFFE0
#define ISP_32B_BOUNDARY_OFFSET         0x0000FFE0

#define ISP_REG_BASE                    0x480BC000
#define ISP_REG_SIZE                    0x00001600

#define ISPCCDC_REG_OFFSET              0x00000600
#define ISPCCDC_REG_BASE                0x480BC600

#define ISPSCMP_REG_OFFSET              0x00000800
#define ISPSCMP_REG_BASE                0x480BC800

#define ISPHIST_REG_OFFSET              0x00000A00
#define ISPHIST_REG_BASE                0x480BCA00
#define ISPHIST_REG(offset)             (ISPHIST_REG_BASE + (offset))

#define ISPH3A_REG_OFFSET               0x00000C00
#define ISPH3A_REG_BASE                         0x480BCC00
#define ISPH3A_REG(offset)              (ISPH3A_REG_BASE + (offset))

/* ISP module register offset */

#define ISP_REVISION                    0x480BC000
#define ISP_SYSCONFIG                   0x480BC004
#define ISP_SYSSTATUS                   0x480BC008
#define ISP_IRQ0ENABLE                  0x480BC00C
#define ISP_IRQ0STATUS                  0x480BC010
#define ISP_IRQ1ENABLE                  0x480BC014
#define ISP_IRQ1STATUS                  0x480BC018
#define ISP_TCTRL_GRESET_LENGTH         0x480BC030
#define ISP_TCTRL_PSTRB_REPLAY          0x480BC034
#define ISP_CTRL                        0x480BC040
#define ISP_SECURE                      0x480BC044
#define ISP_TCTRL_CTRL                  0x480BC050
#define ISP_TCTRL_FRAME                 0x480BC054
#define ISP_TCTRL_PSTRB_DELAY           0x480BC058
#define ISP_TCTRL_STRB_DELAY            0x480BC05C
#define ISP_TCTRL_SHUT_DELAY            0x480BC060
#define ISP_TCTRL_PSTRB_LENGTH          0x480BC064
#define ISP_TCTRL_STRB_LENGTH           0x480BC068
#define ISP_TCTRL_SHUT_LENGTH           0x480BC06C
#define ISP_PING_PONG_ADDR              0x480BC070
#define ISP_PING_PONG_MEM_RANGE         0x480BC074
#define ISP_PING_PONG_BUF_SIZE          0x480BC078

/* CCDC module register offset */
#define ISPCCDC_PID                     0x480BC600
#define ISPCCDC_PCR                     0x480BC604

/* CCDC module register offset */

#define ISPCCDC_PID                     0x480BC600
#define ISPCCDC_PCR                     0x480BC604
#define ISPCCDC_SYN_MODE                0x480BC608
#define ISPCCDC_HD_VD_WID               0x480BC60C
#define ISPCCDC_PIX_LINES               0x480BC610
#define ISPCCDC_HORZ_INFO               0x480BC614
#define ISPCCDC_VERT_START              0x480BC618
#define ISPCCDC_VERT_LINES              0x480BC61C
#define ISPCCDC_CULLING                 0x480BC620
#define ISPCCDC_HSIZE_OFF               0x480BC624
#define ISPCCDC_SDOFST                  0x480BC628
#define ISPCCDC_SDR_ADDR                0x480BC62C
#define ISPCCDC_CLAMP                   0x480BC630
#define ISPCCDC_DCSUB                   0x480BC634
#define ISPCCDC_COLPTN                  0x480BC638
#define ISPCCDC_BLKCMP                  0x480BC63C
#define ISPCCDC_FPC                     0x480BC640
#define ISPCCDC_FPC_ADDR                0x480BC644
#define ISPCCDC_VDINT                   0x480BC648
#define ISPCCDC_ALAW                    0x480BC64C
#define ISPCCDC_REC656IF                0x480BC650
#define ISPCCDC_CFG                     0x480BC654
#define ISPCCDC_FMTCFG                  0x480BC658
#define ISPCCDC_FMT_HORZ                0x480BC65C
#define ISPCCDC_FMT_VERT                0x480BC660
#define ISPCCDC_FMT_ADDR0               0x480BC664
#define ISPCCDC_FMT_ADDR1               0x480BC668
#define ISPCCDC_FMT_ADDR2               0x480BC66C
#define ISPCCDC_FMT_ADDR3               0x480BC670
#define ISPCCDC_FMT_ADDR4               0x480BC674
#define ISPCCDC_FMT_ADDR5               0x480BC678
#define ISPCCDC_FMT_ADDR6               0x480BC67C
#define ISPCCDC_FMT_ADDR7               0x480BC680
#define ISPCCDC_PRGEVEN0                0x480BC684
#define ISPCCDC_PRGEVEN1                0x480BC688
#define ISPCCDC_PRGODD0                 0x480BC68C
#define ISPCCDC_PRGODD1                 0x480BC690
#define ISPCCDC_VP_OUT                  0x480BC694
#define ISPCCDC_CFG_RESV_MASK           0x0000B9E0

#define ISPCCDC_LSC_CONFIG              0x480BC698
#define ISPCCDC_LSC_INITIAL             0x480BC69C
#define ISPCCDC_LSC_TABLE_BASE          0x480BC6A0
#define ISPCCDC_LSC_TABLE_OFFSET        0x480BC6A4

#define ISPCCDC_PCR_EN                  1
#define ISPCCDC_PCR_BUSY                (1 << 1)

#define ISPCCDC_SYN_MODE_VDHDOUT        0x1
#define ISPCCDC_SYN_MODE_FLDOUT         (1 << 1)
#define ISPCCDC_SYN_MODE_VDPOL          (1 << 2)
#define ISPCCDC_SYN_MODE_HDPOL          (1 << 3)
#define ISPCCDC_SYN_MODE_FLDPOL         (1 << 4)
#define ISPCCDC_SYN_MODE_EXWEN          (1 << 5)
#define ISPCCDC_SYN_MODE_DATAPOL        (1 << 6)
#define ISPCCDC_SYN_MODE_FLDMODE        (1 << 7)
#define ISPCCDC_SYN_MODE_DATSIZ_MASK    0xFFFFF8FF
#define ISPCCDC_SYN_MODE_DATSIZ_8_16    (0x0 << 8)
#define ISPCCDC_SYN_MODE_DATSIZ_12      (0x4 << 8)
#define ISPCCDC_SYN_MODE_DATSIZ_11      (0x5 << 8)
#define ISPCCDC_SYN_MODE_DATSIZ_10      (0x6 << 8)
#define ISPCCDC_SYN_MODE_DATSIZ_8       (0x7 << 8)
#define ISPCCDC_SYN_MODE_PACK8          (1 << 11)
#define ISPCCDC_SYN_MODE_INPMOD_MASK    0xFFFFCFFF
#define ISPCCDC_SYN_MODE_INPMOD_RAW     (0 << 12)
#define ISPCCDC_SYN_MODE_INPMOD_YCBCR16 (1 << 12)
#define ISPCCDC_SYN_MODE_INPMOD_YCBCR8  (2 << 12)
#define ISPCCDC_SYN_MODE_LPF            (1 << 14)
#define ISPCCDC_SYN_MODE_FLDSTAT        (1 << 15)
#define ISPCCDC_SYN_MODE_VDHDEN         (1 << 16)
#define ISPCCDC_SYN_MODE_WEN            (1 << 17)
#define ISPCCDC_SYN_MODE_VP2SDR         (1 << 18)
#define ISPCCDC_SYN_MODE_SDR2RSZ        (1 << 19)

#define ISPCCDC_HD_VD_WID_VDW_SHIFT     0
#define ISPCCDC_HD_VD_WID_HDW_SHIFT     16

#define ISPCCDC_PIX_LINES_HLPRF_SHIFT   0
#define ISPCCDC_PIX_LINES_PPLN_SHIFT    16

#define ISPCCDC_HORZ_INFO_NPH_SHIFT     0
#define ISPCCDC_HORZ_INFO_NPH_MASK      0xFFFF8000
#define ISPCCDC_HORZ_INFO_SPH_MASK      0x1000FFFF
#define ISPCCDC_HORZ_INFO_SPH_SHIFT     16

#define ISPCCDC_VERT_START_SLV0_SHIFT   16
#define ISPCCDC_VERT_START_SLV0_MASK    0x1000FFFF
#define ISPCCDC_VERT_START_SLV1_SHIFT   0

#define ISPCCDC_VERT_LINES_NLV_MASK     0xFFFF8000
#define ISPCCDC_VERT_LINES_NLV_SHIFT    0

#define ISPCCDC_CULLING_CULV_SHIFT      0
#define ISPCCDC_CULLING_CULHODD_SHIFT   16
#define ISPCCDC_CULLING_CULHEVN_SHIFT   24

#define ISPCCDC_HSIZE_OFF_SHIFT         0

#define ISPCCDC_SDOFST_FINV             (1 << 14)
#define ISPCCDC_SDOFST_FOFST_1L         (~(3 << 12))
#define ISPCCDC_SDOFST_FOFST_4L         (3 << 12)
#define ISPCCDC_SDOFST_LOFST3_SHIFT     0
#define ISPCCDC_SDOFST_LOFST2_SHIFT     3
#define ISPCCDC_SDOFST_LOFST1_SHIFT     6
#define ISPCCDC_SDOFST_LOFST0_SHIFT     9
#define EVENEVEN                        1
#define ODDEVEN                         2
#define EVENODD                         3
#define ODDODD                          4

#define ISPCCDC_CFG_BW656               (1 << 5)
#define ISPCCDC_CFG_FIDMD_SHIFT         6
#define ISPCCDC_CFG_WENLOG              (1 << 8)
#define ISPCCDC_CFG_WENLOG_AND          (0 << 8)
#define ISPCCDC_CFG_WENLOG_OR           (1 << 8)
#define ISPCCDC_CFG_Y8POS               (1 << 11)
#define ISPCCDC_CFG_BSWD                (1 << 12)
#define ISPCCDC_CFG_MSBINVI             (1 << 13)
#define ISPCCDC_CFG_VDLC                (1 << 15)

#define ISPCCDC_FMT_HORZ_FMTLNH_SHIFT   0
#define ISPCCDC_FMT_HORZ_FMTSPH_SHIFT   16

#define ISPCCDC_FMT_VERT_FMTLNV_SHIFT   0
#define ISPCCDC_FMT_VERT_FMTSLV_SHIFT   16

#define ISPCCDC_FMT_HORZ_FMTSPH_MASK    0x1FFF0000
#define ISPCCDC_FMT_HORZ_FMTLNH_MASK    0x1FFF

#define ISPCCDC_FMT_VERT_FMTSLV_MASK    0x1FFF0000
#define ISPCCDC_FMT_VERT_FMTLNV_MASK    0x1FFF

#define ISPCCDC_VP_OUT_HORZ_ST_SHIFT    0
#define ISPCCDC_VP_OUT_HORZ_NUM_SHIFT   4
#define ISPCCDC_VP_OUT_VERT_NUM_SHIFT   17

/* Histogram registers */
#define ISPHIST_PID                     ISPHIST_REG(0x000)
#define ISPHIST_PCR                     ISPHIST_REG(0x004)
#define ISPHIST_CNT                     ISPHIST_REG(0x008)
#define ISPHIST_WB_GAIN                 ISPHIST_REG(0x00C)
#define ISPHIST_R0_HORZ                 ISPHIST_REG(0x010)
#define ISPHIST_R0_VERT                 ISPHIST_REG(0x014)
#define ISPHIST_R1_HORZ                 ISPHIST_REG(0x018)
#define ISPHIST_R1_VERT                 ISPHIST_REG(0x01C)
#define ISPHIST_R2_HORZ                 ISPHIST_REG(0x020)
#define ISPHIST_R2_VERT                 ISPHIST_REG(0x024)
#define ISPHIST_R3_HORZ                 ISPHIST_REG(0x028)
#define ISPHIST_R3_VERT                 ISPHIST_REG(0x02C)
#define ISPHIST_ADDR                    ISPHIST_REG(0x030)
#define ISPHIST_DATA                    ISPHIST_REG(0x034)
#define ISPHIST_RADD                    ISPHIST_REG(0x038)
#define ISPHIST_RADD_OFF                ISPHIST_REG(0x03C)
#define ISPHIST_H_V_INFO                ISPHIST_REG(0x040)

#define ISPHIST_PCR_ENABLE_SHIFT        0
#define ISPHIST_PCR_ENABLE_MASK         0x01
#define ISPHIST_PCR_BUSY_SHIFT          1
#define ISPHIST_PCR_BUSY_MASK           0x02

#define ISPHIST_CNT_DATASIZE_SHIFT      8
#define ISPHIST_CNT_DATASIZE_MASK       0x0100
#define ISPHIST_CNT_CLEAR_SHIFT         7
#define ISPHIST_CNT_CLEAR_MASK          0x080
#define ISPHIST_CNT_CFA_SHIFT           6
#define ISPHIST_CNT_CFA_MASK            0x040
#define ISPHIST_CNT_BINS_SHIFT          4
#define ISPHIST_CNT_BINS_MASK           0x030
#define ISPHIST_CNT_SOURCE_SHIFT        3
#define ISPHIST_CNT_SOURCE_MASK         0x08
#define ISPHIST_CNT_SHIFT_SHIFT         0
#define ISPHIST_CNT_SHIFT_MASK          0x07

#define ISPHIST_WB_GAIN_WG00_SHIFT      24
#define ISPHIST_WB_GAIN_WG00_MASK       0xFF000000
#define ISPHIST_WB_GAIN_WG01_SHIFT      16
#define ISPHIST_WB_GAIN_WG01_MASK       0xFF0000
#define ISPHIST_WB_GAIN_WG02_SHIFT      8
#define ISPHIST_WB_GAIN_WG02_MASK       0xFF00
#define ISPHIST_WB_GAIN_WG03_SHIFT      0
#define ISPHIST_WB_GAIN_WG03_MASK       0xFF

/*
 * REGION 0 to 3 HORZ
 */
#define ISPHIST_REGHORIZ_HSTART_SHIFT	16
#define ISPHIST_REGHORIZ_HSTART_MASK	0x3FFF0000
#define ISPHIST_REGHORIZ_HEND_SHIFT	0
#define ISPHIST_REGHORIZ_HEND_MASK	0x3FFF
#define ISPHIST_REGVERT_VSTART_SHIFT	16
#define ISPHIST_REGVERT_VSTART_MASK	0x3FFF0000
#define ISPHIST_REGVERT_VEND_SHIFT	0
#define ISPHIST_REGVERT_VEND_MASK       0x3FFF

#define ISPHIST_REGHORIZ_MASK           0x3FFF3FFF
#define ISPHIST_REGVERT_MASK            0x3FFF3FFF

#define ISPHIST_ADDR_SHIFT              0
#define ISPHIST_ADDR_MASK               0x3FF

#define ISPHIST_DATA_SHIFT              0
#define ISPHIST_DATA_MASK               0xFFFFF

#define ISPHIST_RADD_SHIFT              0
#define ISPHIST_RADD_MASK               0xFFFFFFFF

#define ISPHIST_RADD_OFF_SHIFT          0
#define ISPHIST_RADD_OFF_MASK           0xFFFF

#define ISPHIST_HV_INFO_HSIZE_SHIFT     16
#define ISPHIST_HV_INFO_HSIZE_MASK      0x3FFF0000
#define ISPHIST_HV_INFO_VSIZE_SHIFT     0
#define ISPHIST_HV_INFO_VSIZE_MASK      0x3FFF
#define ISPHIST_HV_INFO_MASK            0x3FFF3FFF

/* H3A module registers */
#define ISPH3A_PID                      ISPH3A_REG(0x000)
#define ISPH3A_PCR                      ISPH3A_REG(0x004)
#define ISPH3A_AEWWIN1                  ISPH3A_REG(0x04C)
#define ISPH3A_AEWINSTART               ISPH3A_REG(0x050)
#define ISPH3A_AEWINBLK                 ISPH3A_REG(0x054)
#define ISPH3A_AEWSUBWIN                ISPH3A_REG(0x058)
#define ISPH3A_AEWBUFST                 ISPH3A_REG(0x05C)
#define ISPH3A_AFPAX1                   ISPH3A_REG(0x008)
#define ISPH3A_AFPAX2                   ISPH3A_REG(0x00C)
#define ISPH3A_AFPAXSTART               ISPH3A_REG(0x010)
#define ISPH3A_AFIIRSH                  ISPH3A_REG(0x014)
#define ISPH3A_AFBUFST                  ISPH3A_REG(0x018)
#define ISPH3A_AFCOEF010                ISPH3A_REG(0x01C)
#define ISPH3A_AFCOEF032                ISPH3A_REG(0x020)
#define ISPH3A_AFCOEF054                ISPH3A_REG(0x024)
#define ISPH3A_AFCOEF076                ISPH3A_REG(0x028)
#define ISPH3A_AFCOEF098                ISPH3A_REG(0x02C)
#define ISPH3A_AFCOEF0010               ISPH3A_REG(0x030)
#define ISPH3A_AFCOEF110                ISPH3A_REG(0x034)
#define ISPH3A_AFCOEF132                ISPH3A_REG(0x038)
#define ISPH3A_AFCOEF154                ISPH3A_REG(0x03C)
#define ISPH3A_AFCOEF176                ISPH3A_REG(0x040)
#define ISPH3A_AFCOEF198                ISPH3A_REG(0x044)
#define ISPH3A_AFCOEF1010               ISPH3A_REG(0x048)

#define ISPH3A_PCR_AEW_ALAW_EN_SHIFT    1
#define ISPH3A_PCR_AF_MED_TH_SHIFT      3
#define ISPH3A_PCR_AF_RGBPOS_SHIFT      11
#define ISPH3A_PCR_AEW_AVE2LMT_SHIFT    22
#define ISPH3A_PCR_AEW_AVE2LMT_MASK     0xFFC00000

#define ISPH3A_PCR_AF_BUSY_MASK		(1<<15)
#define ISPH3A_PCR_AF_EN_MASK		(1<<0)

#define ISPH3A_AEWWIN1_WINHC_SHIFT      0
#define ISPH3A_AEWWIN1_WINHC_MASK       0x3F
#define ISPH3A_AEWWIN1_WINVC_SHIFT      6
#define ISPH3A_AEWWIN1_WINVC_MASK       0x1FC0
#define ISPH3A_AEWWIN1_WINW_SHIFT       13
#define ISPH3A_AEWWIN1_WINW_MASK        0xFE000
#define ISPH3A_AEWWIN1_WINH_SHIFT       24
#define ISPH3A_AEWWIN1_WINH_MASK        0x7F000000

/* Preview module registers */
#define ISPPRV_PCR                      0x480BCE04
#define ISPPRV_HORZ_INFO                0x480BCE08
#define ISPPRV_VERT_INFO                0x480BCE0C
#define ISPPRV_RSDR_ADDR                0x480BCE10
#define ISPPRV_RADR_OFFSET              0x480BCE14
#define ISPPRV_DSDR_ADDR                0x480BCE18
#define ISPPRV_DRKF_OFFSET              0x480BCE1C
#define ISPPRV_WSDR_ADDR                0x480BCE20
#define ISPPRV_WADD_OFFSET              0x480BCE24
#define ISPPRV_AVE                      0x480BCE28
#define ISPPRV_HMED                     0x480BCE2C
#define ISPPRV_NF                       0x480BCE30
#define ISPPRV_WB_DGAIN                 0x480BCE34
#define ISPPRV_WBGAIN                   0x480BCE38
#define ISPPRV_WBSEL                    0x480BCE3C
#define ISPPRV_CFA                      0x480BCE40
#define ISPPRV_BLKADJOFF                0x480BCE44
#define ISPPRV_RGB_MAT1                 0x480BCE48
#define ISPPRV_RGB_MAT2                 0x480BCE4C
#define ISPPRV_RGB_MAT3                 0x480BCE50
#define ISPPRV_RGB_MAT4                 0x480BCE54
#define ISPPRV_RGB_MAT5                 0x480BCE58
#define ISPPRV_RGB_OFF1                 0x480BCE5C
#define ISPPRV_RGB_OFF2                 0x480BCE60
#define ISPPRV_CSC0                     0x480BCE64
#define ISPPRV_CSC1                     0x480BCE68
#define ISPPRV_CSC2                     0x480BCE6C
#define ISPPRV_CSC_OFFSET               0x480BCE70
#define ISPPRV_CNT_BRT                  0x480BCE74
#define ISPPRV_CSUP                     0x480BCE78
#define ISPPRV_SETUP_YC                 0x480BCE7C
#define ISPPRV_SET_TBL_ADDR             0x480BCE80
#define ISPPRV_SET_TBL_DATA             0x480BCE84
#define ISPPRV_CDC_THR0                 0x480BCE90
#define ISPPRV_CDC_THR1                 (ISPPRV_CDC_THR0 + (0x4))
#define ISPPRV_CDC_THR2                 (ISPPRV_CDC_THR0 + (0x4) * 2)
#define ISPPRV_CDC_THR3                 (ISPPRV_CDC_THR0 + (0x4) * 3)

#define ISP_INT_CLR                     0xFF113F11
#define ISPPRV_PCR_EN                   1
#define ISPPRV_PCR_BUSY                 (1 << 1)
#define ISPPRV_PCR_SOURCE               (1 << 2)
#define ISPPRV_PCR_ONESHOT              (1 << 3)
#define ISPPRV_PCR_WIDTH                (1 << 4)
#define ISPPRV_PCR_INVALAW              (1 << 5)
#define ISPPRV_PCR_DRKFEN               (1 << 6)
#define ISPPRV_PCR_DRKFCAP              (1 << 7)
#define ISPPRV_PCR_HMEDEN               (1 << 8)
#define ISPPRV_PCR_NFEN                 (1 << 9)
#define ISPPRV_PCR_CFAEN                (1 << 10)
#define ISPPRV_PCR_CFAFMT_SHIFT         11
#define ISPPRV_PCR_CFAFMT_MASK          0x7800
#define ISPPRV_PCR_CFAFMT_BAYER         (0 << 11)
#define ISPPRV_PCR_CFAFMT_SONYVGA       (1 << 11)
#define ISPPRV_PCR_CFAFMT_RGBFOVEON     (2 << 11)
#define ISPPRV_PCR_CFAFMT_DNSPL         (3 << 11)
#define ISPPRV_PCR_CFAFMT_HONEYCOMB     (4 << 11)
#define ISPPRV_PCR_CFAFMT_RRGGBBFOVEON  (5 << 11)
#define ISPPRV_PCR_YNENHEN              (1 << 15)
#define ISPPRV_PCR_SUPEN                (1 << 16)
#define ISPPRV_PCR_YCPOS_SHIFT          17
#define ISPPRV_PCR_YCPOS_YCrYCb         (0 << 17)
#define ISPPRV_PCR_YCPOS_YCbYCr         (1 << 17)
#define ISPPRV_PCR_YCPOS_CbYCrY         (2 << 17)
#define ISPPRV_PCR_YCPOS_CrYCbY         (3 << 17)
#define ISPPRV_PCR_RSZPORT              (1 << 19)
#define ISPPRV_PCR_SDRPORT              (1 << 20)
#define ISPPRV_PCR_SCOMP_EN             (1 << 21)
#define ISPPRV_PCR_SCOMP_SFT_SHIFT      (22)
#define ISPPRV_PCR_SCOMP_SFT_MASK       (~(7 << 22))
#define ISPPRV_PCR_GAMMA_BYPASS         (1 << 26)
#define ISPPRV_PCR_DCOREN               (1 << 27)
#define ISPPRV_PCR_DCCOUP               (1 << 28)
#define ISPPRV_PCR_DRK_FAIL             (1 << 31)

#define ISPPRV_WBGAIN_COEF0_SHIFT       0
#define ISPPRV_WBGAIN_COEF1_SHIFT       8
#define ISPPRV_WBGAIN_COEF2_SHIFT       16
#define ISPPRV_WBGAIN_COEF3_SHIFT       24

#define ISPPRV_WBSEL_COEF0              0x0
#define ISPPRV_WBSEL_COEF1              0x1
#define ISPPRV_WBSEL_COEF2              0x2
#define ISPPRV_WBSEL_COEF3              0x3

#define ISPPRV_WBSEL_N0_0_SHIFT         0
#define ISPPRV_WBSEL_N0_1_SHIFT         2
#define ISPPRV_WBSEL_N0_2_SHIFT         4
#define ISPPRV_WBSEL_N0_3_SHIFT         6
#define ISPPRV_WBSEL_N1_0_SHIFT         8
#define ISPPRV_WBSEL_N1_1_SHIFT         10
#define ISPPRV_WBSEL_N1_2_SHIFT         12
#define ISPPRV_WBSEL_N1_3_SHIFT         14
#define ISPPRV_WBSEL_N2_0_SHIFT         16
#define ISPPRV_WBSEL_N2_1_SHIFT         18
#define ISPPRV_WBSEL_N2_2_SHIFT         20
#define ISPPRV_WBSEL_N2_3_SHIFT         22
#define ISPPRV_WBSEL_N3_0_SHIFT         24
#define ISPPRV_WBSEL_N3_1_SHIFT         26
#define ISPPRV_WBSEL_N3_2_SHIFT         28
#define ISPPRV_WBSEL_N3_3_SHIFT         30

#define ISPPRV_CFA_GRADTH_HOR_SHIFT     0
#define ISPPRV_CFA_GRADTH_VER_SHIFT     8

#define ISPPRV_BLKADJOFF_B_SHIFT        0
#define ISPPRV_BLKADJOFF_G_SHIFT        8
#define ISPPRV_BLKADJOFF_R_SHIFT        16

#define ISPPRV_RGB_MAT1_MTX_RR_SHIFT    0
#define ISPPRV_RGB_MAT1_MTX_GR_SHIFT    16

#define ISPPRV_RGB_MAT2_MTX_BR_SHIFT    0
#define ISPPRV_RGB_MAT2_MTX_RG_SHIFT    16

#define ISPPRV_RGB_MAT3_MTX_GG_SHIFT    0
#define ISPPRV_RGB_MAT3_MTX_BG_SHIFT    16

#define ISPPRV_RGB_MAT4_MTX_RB_SHIFT    0
#define ISPPRV_RGB_MAT4_MTX_GB_SHIFT    16

#define ISPPRV_RGB_MAT5_MTX_BB_SHIFT    0

#define ISPPRV_RGB_OFF1_MTX_OFFG_SHIFT  0
#define ISPPRV_RGB_OFF1_MTX_OFFR_SHIFT  16

#define ISPPRV_RGB_OFF2_MTX_OFFB_SHIFT  0

#define ISPPRV_CSC0_RY_SHIFT            0
#define ISPPRV_CSC0_GY_SHIFT            10
#define ISPPRV_CSC0_BY_SHIFT            20

#define ISPPRV_CSC1_RCB_SHIFT           0
#define ISPPRV_CSC1_GCB_SHIFT           10
#define ISPPRV_CSC1_BCB_SHIFT           20

#define ISPPRV_CSC2_RCR_SHIFT           0
#define ISPPRV_CSC2_GCR_SHIFT           10
#define ISPPRV_CSC2_BCR_SHIFT           20

#define ISPPRV_CSC_OFFSET_CR_SHIFT      0
#define ISPPRV_CSC_OFFSET_CB_SHIFT      8
#define ISPPRV_CSC_OFFSET_Y_SHIFT       16

#define ISPPRV_CNT_BRT_BRT_SHIFT        0
#define ISPPRV_CNT_BRT_CNT_SHIFT        8

/* ISP modules IRQ */
#define IRQ0ENABLE_CSIA_IRQ                     1
#define IRQ0ENABLE_CSIA_LC1_IRQ                 (1 << 1)
#define IRQ0ENABLE_CSIA_LC2_IRQ                 (1 << 2)
#define IRQ0ENABLE_CSIA_LC3_IRQ                 (1 << 3)
#define IRQ0ENABLE_CSIB_IRQ                     (1 << 4)
#define IRQ0ENABLE_CSIB_LC1_IRQ                 (1 << 5)
#define IRQ0ENABLE_CSIB_LC2_IRQ                 (1 << 6)
#define IRQ0ENABLE_CSIB_LC3_IRQ                 (1 << 7)
#define IRQ0ENABLE_CCDC_VD0_IRQ                 (1 << 8)
#define IRQ0ENABLE_CCDC_VD1_IRQ                 (1 << 9)
#define IRQ0ENABLE_CCDC_VD2_IRQ                 (1 << 10)
#define IRQ0ENABLE_CCDC_ERR_IRQ                 (1 << 11)
#define IRQ0ENABLE_H3A_AF_DONE_IRQ              (1 << 12)
#define IRQ0ENABLE_H3A_AWB_DONE_IRQ             (1 << 13)
#define IRQ0ENABLE_HIST_DONE_IRQ                (1 << 16)
#define IRQ0ENABLE_CCDC_LSC_DONE_IRQ            (1 << 17)
#define IRQ0ENABLE_CCDC_LSC_PREF_COMP_IRQ       (1 << 18)
#define IRQ0ENABLE_CCDC_LSC_PREF_ERR_IRQ        (1 << 19)
#define IRQ0ENABLE_PRV_DONE_IRQ                 (1 << 20)
#define IRQ0ENABLE_RSZ_DONE_IRQ                 (1 << 24)
#define IRQ0ENABLE_OVF_IRQ                      (1 << 25)
#define IRQ0ENABLE_PING_IRQ                     (1 << 26)
#define IRQ0ENABLE_PONG_IRQ                     (1 << 27)
#define IRQ0ENABLE_MMU_ERR_IRQ                  (1 << 28)
#define IRQ0ENABLE_OCP_ERR_IRQ                  (1 << 29)
#define IRQ0ENABLE_SEC_ERR_IRQ                  (1 << 30)
#define IRQ0ENABLE_HS_VS_IRQ                    (1 << 31)


#define IRQ0STATUS_CSIA_IRQ                     1
#define IRQ0STATUS_CSIA_LC1_IRQ                 (1 << 1)
#define IRQ0STATUS_CSIA_LC2_IRQ                 (1 << 2)
#define IRQ0STATUS_CSIA_LC3_IRQ                 (1 << 3)
#define IRQ0STATUS_CSIB_IRQ                     (1 << 4)
#define IRQ0STATUS_CSIB_LC1_IRQ                 (1 << 5)
#define IRQ0STATUS_CSIB_LC2_IRQ                 (1 << 6)
#define IRQ0STATUS_CSIB_LC3_IRQ                 (1 << 7)
#define IRQ0STATUS_CCDC_VD0_IRQ                 (1 << 8)
#define IRQ0STATUS_CCDC_VD1_IRQ                 (1 << 9)
#define IRQ0STATUS_CCDC_VD2_IRQ                 (1 << 10)
#define IRQ0STATUS_CCDC_ERR_IRQ                 (1 << 11)
#define IRQ0STATUS_H3A_AF_DONE_IRQ              (1 << 12)
#define IRQ0STATUS_H3A_AWB_DONE_IRQ             (1 << 13)
#define IRQ0STATUS_HIST_DONE_IRQ                (1 << 16)
#define IRQ0STATUS_PRV_DONE_IRQ                 (1 << 20)
#define IRQ0STATUS_RSZ_DONE_IRQ                 (1 << 24)
#define IRQ0STATUS_OVF_IRQ                      (1 << 25)
#define IRQ0STATUS_PING_IRQ                     (1 << 26)
#define IRQ0STATUS_PONG_IRQ                     (1 << 27)
#define IRQ0STATUS_MMU_ERR_IRQ                  (1 << 28)
#define IRQ0STATUS_OCP_ERR_IRQ                  (1 << 29)
#define IRQ0STATUS_SEC_ERR_IRQ                  (1 << 30)
#define IRQ0STATUS_HS_VS_IRQ                    (1 << 31)

#endif /* #define	__HP3A_ISP_REG_H_INCLUDED */
