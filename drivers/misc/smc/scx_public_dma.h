/*
 *Copyright (c)2006-2008 Trusted Logic S.A.
 *All Rights Reserved.
 *
 *This program is free software; you can redistribute it and/or
 *modify it under the terms of the GNU General Public License
 *version 2 as published by the Free Software Foundation.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *You should have received a copy of the GNU General Public License
 *along with this program; if not, write to the Free Software
 *Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 *MA 02111-1307 USA
 */

#ifndef __SCX_PUBLIC_DMA_H
#define __SCX_PUBLIC_DMA_H

#include "scx_public_crypto.h"

/*---------------------------------------------------------------------------
 *Cache management (implemented in the assembler file)
 *-------------------------------------------------------------------------- */

u32 v7_dma_flush_range(u32 nVAStart, u32 nVAEnd);
u32 v7_dma_inv_range(u32 nVAStart, u32 nVAEnd);

/*-------------------------------------------------------------------------- */
/*
 *Public DMA API
 */

/*
 *The DMA channels reserved for the SMC are 0x18--> 0x1F
 *> Secure World: 0x18 --> 0x1D
 *> Public World: 0x1E and 0x1F
 *
 *The DMA channels allocated for the SE are defined in the PPA:
 *--> ppa_init_lib.c / Dma_BootAllocatedChannels
 *
 *Note that the Secure World uses the IRQ Line 3.
 *This line is therefore not available in the Normal World.
 *
 *We only use 2 channels shared between all
 *public crypto operations.
 */
#define OMAP3XXX_SMC_DMA_CH_0	 0x1E
#define OMAP3XXX_SMC_DMA_CH_1	 0x1F
#define OMAP3XXX_SMC_DMA_CH_2	 0x1D
#define OMAP3XXX_SMC_DMA_CH_3	 0x1B
#define OMAP3XXX_SMC_DMA_CH_4	 0x1C

#define OMAP3XXX_SMC_DMA_CH_NB	2

#define OMAP3XXX_SMC_DMA_ALL	((1 << OMAP3XXX_SMC_DMA_CH_0) | \
					(1 << OMAP3XXX_SMC_DMA_CH_1) | \
					(1 << OMAP3XXX_SMC_DMA_CH_2) | \
					(1 << OMAP3XXX_SMC_DMA_CH_3) | \
					(1 << OMAP3XXX_SMC_DMA_CH_4))

#define OMAP_DMA_RES_IRQ		(1 << 0)
#define OMAP_DMA_DROP_IRQ		(1 << 1)
#define OMAP_DMA_HALF_IRQ		(1 << 2)
#define OMAP_DMA_FRAME_IRQ		(1 << 3)
#define OMAP_DMA_LAST_IRQ		(1 << 4)
#define OMAP_DMA_BLOCK_IRQ		(1 << 5)

/*
 *CCR Masks
 */
#define DMA_CCR_Mask_Channel(index)((((index) & 0x60) << 14) | \
					((index) & 0x1F))

#define DMA_CCR_Channel_Mem2AES1		0x0A
#define DMA_CCR_Channel_AES12Mem		0x09
#define DMA_CCR_Channel_Mem2AES2		0x42
#define DMA_CCR_Channel_AES22Mem		0x41

#define DMA_CCR_Channel_Mem2DES1		0x0C	/*DES1 is the trigger*/
#define DMA_CCR_Channel_DES12Mem		0x0B	/*	"	*/
#define DMA_CCR_Channel_Mem2DES2		0x44	/*DES2 is the trigger*/
#define DMA_CCR_Channel_DES22Mem		0x43	/*	"	*/
#define DMA_CCR_Channel_Mem2SHA			0x0D

/*
 * Add +1 to the value of the spec for these constants
 * (comes from CCRi register)
 */
#define DMA_CCR_Channel_Mem2AES		0x0A
#define DMA_CCR_Channel_AES2Mem		0x09
/*Value for DES2. For DES1 = 0x0C */
#define DMA_CCR_Channel_Mem2DES		0x44
/*Value for DES2. For DES1 = 0x0B */
#define DMA_CCR_Channel_DES2Mem		0x43
#define DMA_CCR_Channel_Mem2SHA		0x0D

/*
 *CSDP Masks
 */
#define DMA_CSDP_Srce_Endian_big	(1<<21)
#define DMA_CSDP_Srce_Endian_little	(0<<21)

#define DMA_CSDP_Srce_Endian_Lock_on	(1<<20)
#define DMA_CSDP_Srce_Endian_Lock_off	(0<<20)

#define DMA_CSDP_Dest_Endian_big	(1<<19)
#define DMA_CSDP_Dest_Endian_little	(0<<19)

#define DMA_CSDP_Dest_Endian_Lock_on	(1<<18)
#define DMA_CSDP_Dest_Endian_Lock_off	(0<<18)

#define DMA_CSDP_Write_Mode_none_posted		((0<<17) | (0<<16))
#define DMA_CSDP_Write_Mode_posted		((0<<17) | (1<<16))
#define DMA_CSDP_Write_Mode_posted_except_last	((1<<17) | (0<<16))
#define DMA_CSDP_Write_Mode_undefined		((1<<17) | (1<<16))

#define DMA_CSDP_Dest_Burst_off			((0<<15) | (0<<14))
#define DMA_CSDP_Dest_Burst_16B			((0<<15) | (1<<14))
#define DMA_CSDP_Dest_Burst_32B			((1<<15) | (0<<14))
#define DMA_CSDP_Dest_Burst_64B			((1<<15) | (1<<14))

#define DMA_CSDP_Dest_packed_on			(1<<13)
#define DMA_CSDP_Dest_packed_off		(0<<13)

#define DMA_CSDP_WR_Add_Trslt			(3<<9)

#define DMA_CSDP_Src_Burst_off			((0<<8) | (0<<7))
#define DMA_CSDP_Src_Burst_16B			((0<<8) | (1<<7))
#define DMA_CSDP_Src_Burst_32B			((1<<8) | (0<<7))
#define DMA_CSDP_Src_Burst_64B			((1<<8) | (1<<7))

#define DMA_CSDP_Src_packed_on			(1<<6)
#define DMA_CSDP_Src_packed_off			(0<<6)

#define DMA_CSDP_RD_Add_Trslt			(3<<2)

#define DMA_CSDP_Data_8b			((0<<1) | (0<<0))
#define DMA_CSDP_Data_16b			((0<<1) | (1<<0))
#define DMA_CSDP_Data_32b			((1<<1) | (0<<0))
#define DMA_CSDP_Data_undefined			((1<<1) | (1<<0))

/*
 *CEN Masks
 */
#define DMA_CEN_Elts_per_Frame_AES		4
#define DMA_CEN_Elts_per_Frame_DES		2
#define DMA_CEN_Elts_per_Frame_SHA		16

/*
 *Other Masks
 */
#define DMA_CSEI_Default	 0
#define DMA_CDEI_Default	 0
#define DMA_CSFI_Default	 0
#define DMA_CDFI_Default	 0

typedef struct {
	int data_type;		/*data type 8, 16, 32 */
	int elem_count;		/*number of elements in a frame */
	int frame_count;	/*number of frames in a element */

	/*constant , post increment, indexed , double indexed */
	int src_amode;
	int src_start;		/*source address : physical */
	int src_ei;		/*source element index */
	int src_fi;		/*source frame index */

	/*constant , post increment, indexed , double indexed */
	int dst_amode;
	int dst_start;		/*source address : physical */
	int dst_ei;		/*source element index */
	int dst_fi;		/*source frame index */

	int trigger;	/*trigger attached if the channel is synchronized */
	int sync_mode;		/*sycn on element, frame , block or packet */
	int src_or_dst_synch;	/*source synch(1)or destination synch(0) */

	int ie;				/*interrupt enabled */

} SCX_DMA_CHANNEL_PARAM;

/**
 *Registers the ISR for the DMA (System DMA interrupt request)
 */
u32 scxPublicDMAInit(void);

/**
 *Unregisters the ISR for the DMA (System DMA interrupt request)
 */
void scxPublicDMATerminate(void);

/*
 *Request a DMA channel
 *(between OMAP3XXX_SMC_DMA_CH_0 and OMAP3XXX_SMC_DMA_CH_3)
 */
u32 scxPublicDMARequest(int lch);

/*
 *Release a DMA channel
 *(between OMAP3XXX_SMC_DMA_CH_0 and OMAP3XXX_SMC_DMA_CH_3)
 */
u32 scxPublicDMARelease(int lch);

/**
 *This function polls the dma transfer completion.
 *
 *lch	DMA channel ID.
 */
void scxPublicDMAPoll(int lch);

/**
 *This function waits for the DMA IRQ.
 *
 */
void scxPublicDMAWait(void);

/*
 *This function starts a DMA operation.
 *
 *lch			DMA channel ID.
 *
 *interruptMask		Configures the Channel Interrupt Control Register
 *			should always contains OMAP_DMA_DROP_IRQ for enabling
 *			the event drop interrupt and might contains
 *			OMAP_DMA_BLOCK_IRQ for enabling the end of
 *			block interrupt.
 */
void scxPublicDMAStart(int lch, int interruptMask);

void scxPublicSetDMAChannelCommonParams(SCX_DMA_CHANNEL_PARAM *pDMAChannel,
				u32 nbBlocks, u32 nbElements, u32 nDstStart,
				u32 nSrcStart, u32 nTriggerID);
void scxPublicDMASetParams(int lch, SCX_DMA_CHANNEL_PARAM *pParams);
void scxPublicDMADisableChannel(int lch);
void scxPublicDMAClearChannel(int lch);

void scxPublicDMAClearL3IRQ(void);
void scxPublicDMAEnableL3IRQ(void);
void scxPublicDMADisableL3IRQ(void);

#endif /*__SCX_PUBLIC_DMA_H */
