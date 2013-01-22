/*
 * Copyright (C) 2007-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 * Motorola 2009-Oct-29 - Support for low current drain wakeups
 * Motorola 2009-Jan-06 - Update for TI Kernel 25.3
 * Motorola 2008-Mar-05 - Update for OMAP
 * Motorola 2007-Sep-14 - Initial Creation
 */
 
#ifndef __SMART_CARD_KERNEL_H__
#define __SMART_CARD_KERNEL_H__

#ifdef __KERNEL__

#include <mach/hardware.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>

/************** STRUCTURES, ENUMS, AND TYPEDEFS *******************************/
typedef signed char INT8;
typedef unsigned char UINT8;
typedef signed short int INT16;
typedef unsigned short int UINT16;
typedef signed int INT32;
typedef unsigned int UINT32;
typedef unsigned char BOOL;
typedef unsigned char BOOLEAN;

typedef struct
{
    UINT32 revision;              /* 0x00 */
    UINT32 ident;                 /* 0x04 */
    UINT32 unused1;               /* 0x08 */
    UINT32 unused2;               /* 0x0C */
    UINT32 sysconfig;             /* 0x10 */
    UINT32 sysstatus;             /* 0x14 */
    UINT32 irqstatus;             /* 0x18 */
    UINT32 irqenable;             /* 0x1C */
    UINT32 wakeupen;              /* 0x20 */
    UINT32 usimcmd;               /* 0x24 */
    UINT32 usimstat;              /* 0x28 */
    UINT32 usimconf1;             /* 0x2C */
    UINT32 usimconf2;             /* 0x30 */
    UINT32 usimconf3;             /* 0x34 */
    UINT32 usim_drx;              /* 0x38 */
    UINT32 usim_dtx;              /* 0x3C */
    UINT32 usim_fifos;            /* 0x40 */
    UINT32 usim_cgt;              /* 0x44 */
    UINT32 usim_cwt;              /* 0x48 */
    UINT32 usim_bwt;              /* 0x4C */
    UINT32 debug_reg;             /* 0x50 */
    UINT32 conf_sam1_div;         /* 0x54 */
    UINT32 conf4_reg;             /* 0x58 */
    UINT32 atr_clk_prd_nbs;       /* 0x5C */
    UINT32 conf_etu_div;          /* 0x60 */
    UINT32 conf5_reg;             /* 0x64 */
    UINT32 tc_guard_time_add_reg; /* 0x68 */
} SIM_MODULE_REGISTER_BANK;


/*******************************************************************************************
 * Constants
 *******************************************************************************************/

#define INT_SIM_GENERAL                    35 

#define SIM_DEV_NAME                       "sim"

#define TRUE                                1
#define FALSE                               0

#define SIM_MODULE_RX_FIFO_SIZE          16
#define SIM_MODULE_TX_FIFO_SIZE          16

#define SIM_MODULE_MAX_NULL_PROCEDURE_BYTES   500
#define SIM_MODULE_MAX_DATA                  (260 + SIM_MODULE_MAX_NULL_PROCEDURE_BYTES)

#define SIM_NUM                       230

#define SIM1_BASE_ADDR (L4_WK_34XX_BASE + 0xe000) /* 0x4830E000 */

/* revision register */
#define REV_MASK 0x00FF

/* ident */
#define VENDOR_CODE_MASK 0xFFFF

/* sysconfig */
#define CLOCK_ACTIVITY_MASK 0x0300
#define EMUFREE_MASK        0x0020
#define IDLEMODE_MASK       0x0018
#define ENAWAKEUP_MASK      0x0004
#define SOFTRESET_MASK      0x0002
#define AUTOIDLE_MASK       0x0001

/* sysstatus */
#define RESETDONE_MASK 0x0001

/* irqstatus */
#define USIM_STOP_CLK_MASK              0x1000
#define IT_EMV_ATR_LENGTH_TIME_OUT_MASK 0x0800
#define TS_ERROR_MASK                   0x0400
#define USIM_RESENT_MASK                0x0200
#define USIM_TOB_MASK                   0x0100
#define USIM_TOC_MASK                   0x0080
#define USIM_EOB_MASK                   0x0040
#define USIM_CD_MASK                    0x0020
#define USIM_RX_MASK                    0x0010
#define USIM_TX_MASK                    0x0008
#define USIM_RXFULL_MASK                0x0004
#define USIM_WT_MASK                    0x0002
#define USIM_NATR_MASK                  0x0001

/* irqenable */
#define USIM_IRQEN_MASK_ALL                0x1FFF
#define USIM_STOP_CLK_EN_MASK              0x1000
#define IT_EMV_ATR_LENGTH_TIME_OUT_EN_MASK 0x0800
#define TS_ERROR_EN_MASK                   0x0400
#define USIM_RESENT_EN_MASK                0x0200
#define USIM_TOB_EN_MASK                   0x0100
#define USIM_TOC_EN_MASK                   0x0080
#define USIM_EOB_EN_MASK                   0x0040
#define USIM_CD_EN_MASK                    0x0020
#define USIM_RX_EN_MASK                    0x0010
#define USIM_TX_EN_MASK                    0x0008
#define USIM_RXFULL_EN_MASK                0x0004
#define USIM_WT_EN_MASK                    0x0002
#define USIM_NATR_EN_MASK                  0x0001

/* wakeupen */
#define USIM_STOP_CLK_WK_EN_MASK              0x1000
#define IT_EMV_ATR_LENGTH_TIME_OUT_WK_EN_MASK 0x0800
#define TS_ERROR_WK_EN_MASK                   0x0400
#define USIM_RESENT_WK_EN_MASK                0x0200
#define USIM_TOB_WK_EN_MASK                   0x0100
#define USIM_TOC_WK_EN_MASK                   0x0080
#define USIM_EOB_WK_EN_MASK                   0x0040
#define USIM_CD_WK_EN_MASK                    0x0020
#define USIM_RX_WK_EN_MASK                    0x0010
#define USIM_TX_WK_EN_MASK                    0x0008
#define USIM_RXFULL_WK_EN_MASK                0x0004
#define USIM_WT_WK_EN_MASK                    0x0002
#define USIM_NATR_WK_EN_MASK                  0x0001

/* usimcmd */
#define STOP_EMV_ATR_LENGTH_TIMER_MASK 0x0040
#define CMD_CLOCK_STOP_MASK            0x0020
#define CMD_WARM_RST_MASK              0x0010
#define MODULE_CLK_EN_MASK             0x0008
#define CMDSTART_MASK                  0x0004
#define CMDSTOP_MASK                   0x0002

/* usimstat */
#define X_MODE_MASK      0x0010
#define CONFCODCONV_MASK 0x0008
#define STATLRC_MASK     0x0004
#define STATTXPAR_MASK   0x0002
#define STATNOCARD_MASK  0x0001

/* usimconf1 */
#define EMV_CONF_MASK     0x0040
#define CONF_SCLK_EN_MASK 0x0020
#define SRSTLEV_MASK      0x0010
#define SVCCLEV_MASK      0x0008
#define CONFBYPASS_MASK   0x0004
#define CONFSIOLOW_MASK   0x0002
#define SCLKLEV_MASK      0x0001

/* usimconf2 */
#define DEBOUNCE_EN_MASK     0x2000
#define HW_DEACTIV_EN_MASK   0x1000
#define CARD_POLARITY_MASK   0x0800
#define CONFRESET_MASK       0x0700
#define CONFLRCCHECK_MASK    0x0080
#define CONFEDC_MASK         0x0040
#define CONFPROTOCOL_MASK    0x0020
#define ATR_ASYN_BYPASS_MASK 0x0010
#define CONFSCLKDIV_MASK     0x000C
#define TXNRX_MASK           0x0002
#define CONFCHKPAR_MASK      0x0001

/* usimconf3 */
#define TDUSIM_MASK 0x00F0
#define TFUSIM_MASK 0x000F

/* usim_drx */
#define STATRXPAR_MASK 0x0100
#define USIMDRX_MASK   0x00FF

/* usim_dtx */
#define DTX_MASK 0x00FF

/* usim_fifos */
#define FIFORX_FULL_MASK     0x8000
#define FIFORX_EMPTY_MASK    0x4000
#define FIFORX_RESET_MASK    0x2000
#define FIFO_RX_TRIGGER_MASK 0x1e00
#define FIFO_RX_TRIGGER_SHIFT 9
#define FIFOTX_FULL_MASK     0x0100
#define FIFOTX_EMPTY_MASK    0x0080
#define FIFOTX_RESET_MASK    0x0040
#define FIFO_TX_TRIGGER_MASK 0x003c
#define FIFO_TX_TRIGGER_SHIFT 2
#define FIFO_ENABLE_MASK     0x0002
#define SIM_DMA_MODE_MASK        0x0001

/* usim_cgt */
#define CGT_MASK 0x00FF

/* usim_cwt */
#define CWT_MASK 0xFFFF

/* usim_bwt */
#define BWT_MASK 0x007FFFFF

/* debug_reg */
#define TXDMA_MASK            0x4000
#define RXDMA_MASK            0x2000
#define RXFIFO_PEAK_MASK      0x1F00
#define RX_STATE_MACHINE_MASK 0x00C0
#define TX_STATE_MACHINE_MASK 0x0030
#define MAIN_STATE_DEBUG      0x000F

/* conf_sam1_div */
#define SAM1_DIV_MASK 0x0FFF

/* conf4_reg */
#define CONFWAITI_MASK 0xFFFF

/* atr_clk_prd_nbs */
#define CLOCK_NUMBER_BEFORE_ATR_MASK 0xFFFF

/* conf_etu_div */
#define ETU_DIV_MASK 0xFFFF

/* conf5_reg */
#define SOFT_NHARD_FIDI_PROG_MASK 0x0100
#define CONFFI_MASK               0x00F0
#define DI_MASK                   0x000F

/* tc_guard_time_add_reg */
#define SOFT_TC_GUARD_TIME_ADD_EN_MASK 0x2000
#define SOFT_TC_GUARD_TIME_ADD_MASK    0x1FFF

#endif /* __SMART_CARD_KERNEL_H__ */
#endif 
