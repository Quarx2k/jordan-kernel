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

#include "scxlnx_defs.h"
#include "scxlnx_util.h"
#include "scx_public_crypto.h"
#include "scx_public_dma.h"

/*
 *AES Hardware Accelerator: Base address
 */
#define AES1_REGS_HW_ADDR		0x480A6000

/*
 *CTRL register Masks
 */
#define AES_CTRL_OUTPUT_READY_BIT	(1<<0)
#define AES_CTRL_INPUT_READY_BIT		(1<<1)

#define AES_CTRL_GET_DIRECTION(x)	(x&4)
#define AES_CTRL_DIRECTION_DECRYPT	0
#define AES_CTRL_DIRECTION_ENCRYPT	(1<<2)

#define AES_CTRL_GET_KEY_SIZE(x)		(x&0x18)
#define AES_CTRL_KEY_SIZE_128			0x08
#define AES_CTRL_KEY_SIZE_192			0x10
#define AES_CTRL_KEY_SIZE_256			0x18

#define AES_CTRL_GET_MODE(x)			((x&0x60) >> 5)
#define AES_CTRL_IS_MODE_CBC(x)		(AES_CTRL_GET_MODE(x) == 1)
#define AES_CTRL_IS_MODE_ECB(x)		(AES_CTRL_GET_MODE(x) == 0)
#define AES_CTRL_IS_MODE_CTR(x)		((AES_CTRL_GET_MODE(x) == 2) || \
					(AES_CTRL_GET_MODE(x) == 3))
#define AES_CTRL_MODE_CBC_BIT			0x20
#define AES_CTRL_MODE_ECB_BIT			0
#define AES_CTRL_MODE_CTR_BIT			0x40

#define AES_CTRL_GET_CTR_WIDTH(x)	(x&0x180)
#define AES_CTRL_CTR_WIDTH_32			0
#define AES_CTRL_CTR_WIDTH_64			0x80
#define AES_CTRL_CTR_WIDTH_96			0x100
#define AES_CTRL_CTR_WIDTH_128		0x180

/*
 *MASK register masks
 */
#define AES_MASK_AUTOIDLE_BIT			(1<<0)
#define AES_MASK_SOFTRESET_BIT		(1<<1)
#define AES_MASK_DMA_REQ_IN_EN_BIT	(1<<2)
#define AES_MASK_DMA_REQ_OUT_EN_BIT	(1<<3)
#define AES_MASK_DIRECT_BUS_EN_BIT	(1<<4)
#define AES_MASK_START_BIT				(1<<5)

#define AES_MASK_GET_SIDLE(x)			((x&0xC0) >> 6)
#define AES_MASK_SIDLE_FORCE_IDLE	0
#define AES_MASK_SIDLE_NO_IDLE		1
#define AES_MASK_SIDLE_SMART_IDLE	2

/*
 *SYSTATUS register masks
 */
#define AES_SYSTATUS_RESET_DONE		(1<<0)

/*----------------------------------------------------------------------*/
/*			 AES Context					*/
/*----------------------------------------------------------------------*/
/**
 *This structure contains the registers of the AES HW accelerator.
 */
typedef struct {
	VU32 AES_KEY4_L;	/*0x00 */
	VU32 AES_KEY4_H;	/*0x04 */
	VU32 AES_KEY3_L;	/*0x08 */
	VU32 AES_KEY3_H;	/*0x0C */
	VU32 AES_KEY2_L;	/*0x10 */
	VU32 AES_KEY2_H;	/*0x14 */
	VU32 AES_KEY1_L;	/*0x18 */
	VU32 AES_KEY1_H;	/*0x1c */

	VU32 AES_IV_1;		/*0x20 */
	VU32 AES_IV_2;		/*0x24 */
	VU32 AES_IV_3;		/*0x28 */
	VU32 AES_IV_4;		/*0x2C */

	VU32 AES_CTRL;		/*0x30 */

	VU32 AES_DATA_1;	/*0x34 */
	VU32 AES_DATA_2;	/*0x38 */
	VU32 AES_DATA_3;	/*0x3C */
	VU32 AES_DATA_4;	/*0x40 */

	VU32 AES_REV;		/*0x44 */
	VU32 AES_MASK;		/*0x48 */

	VU32 AES_SYSSTATUS;	/*0x4C */

} AESReg_t;


/*---------------------------------------------------------------------------
 *Forward declarations
 *------------------------------------------------------------------------- */

static void PDrvCryptoUpdateAESWithDMA(u8 *pSrc, u8 *pDest,
					u32 nbBlocks, u32 dmaUse);

/*----------------------------------------------------------------------------
 *Save HWA registers into the specified operation state structure
 *--------------------------------------------------------------------------*/
static void PDrvCryptoSaveAESRegisters(
			u32 AES_CTRL,
			PUBLIC_CRYPTO_AES_OPERATION_STATE *pAESState)
{
	AESReg_t *pAESReg_t = (AESReg_t *)IO_ADDRESS(AES1_REGS_HW_ADDR);

	dprintk(KERN_INFO "PDrvCryptoSaveAESRegisters: \
		pAESState(%p) <- pAESReg_t(%p): CTRL=0x%08x\n",
		pAESState, pAESReg_t, AES_CTRL);

	/*Save the IV if we are in CBC or CTR mode (not required for ECB) */
	if (!AES_CTRL_IS_MODE_ECB(AES_CTRL)) {
		pAESState->AES_IV_1 = INREG32(&pAESReg_t->AES_IV_1);
		pAESState->AES_IV_2 = INREG32(&pAESReg_t->AES_IV_2);
		pAESState->AES_IV_3 = INREG32(&pAESReg_t->AES_IV_3);
		pAESState->AES_IV_4 = INREG32(&pAESReg_t->AES_IV_4);
	}
}

/*----------------------------------------------------------------------------
 *Restore the HWA registers from the operation state structure
 *---------------------------------------------------------------------------*/
void PDrvCryptoRestoreAESRegisters(
				u32 AES_CTRL,
				PUBLIC_CRYPTO_AES_OPERATION_STATE *pAESState)
{
	AESReg_t *pAESReg_t = (AESReg_t *)IO_ADDRESS(AES1_REGS_HW_ADDR);

	dprintk(KERN_INFO "PDrvCryptoRestoreAESRegisters: \
		pAESReg_t(%p) <- pAESState(%p): CTRL=0x%08x\n",
		pAESReg_t, pAESState, AES_CTRL);

	/*
	 * Restore the IV first if we are in CBC or CTR mode
	 * (not required for ECB)
	 */
	if (!AES_CTRL_IS_MODE_ECB(AES_CTRL)) {
		OUTREG32(&pAESReg_t->AES_IV_1, pAESState->AES_IV_1);
		OUTREG32(&pAESReg_t->AES_IV_2, pAESState->AES_IV_2);
		OUTREG32(&pAESReg_t->AES_IV_3, pAESState->AES_IV_3);
		OUTREG32(&pAESReg_t->AES_IV_4, pAESState->AES_IV_4);
	}

	/*
	 * Then set the CTRL register:
	 * Overwrite the CTRL only when needed, because unconditionally
	 * doing it leads to break the HWA process
	 * (observed by experimentation)
	 */
	AES_CTRL = (INREG32(&pAESReg_t->AES_CTRL) & (3 << 3)) /* key size */
		| (AES_CTRL & ((1 << 2) | (1 << 5) | (1 << 6)))
		/* CTR, CBC, DIRECTION */
		| (0x3 << 7)
		/* Always set CTR_WIDTH to 128-bit */;

	if ((AES_CTRL & 0x1FC) != (INREG32(&pAESReg_t->AES_CTRL) & 0x1FC))
		OUTREG32(&pAESReg_t->AES_CTRL, AES_CTRL & 0x1FC);

	/*Set the MASK register to 0 */
	OUTREG32(&pAESReg_t->AES_MASK, 0);
}

/*-------------------------------------------------------------------------- */

void PDrvCryptoUpdateAES(u32 AES_CTRL,
			PUBLIC_CRYPTO_AES_OPERATION_STATE *pAESState,
			u8 *pSrc, u8 *pDest, u32 nbBlocks)
{
	AESReg_t *pAESReg_t = (AESReg_t *)IO_ADDRESS(AES1_REGS_HW_ADDR);

	u32 nbr_of_blocks;
	u32 vTemp;
	u8 *pProcessSrc = pSrc;
	u8 *pProcessDest = pDest;
	u32 dmaUse = PUBLIC_CRYPTO_DMA_USE_NONE;

	/*
	 *Choice of the processing type
	 */
	if (nbBlocks * AES_BLOCK_SIZE >= DMA_TRIGGER_POLLING_AES) {
		if (nbBlocks * AES_BLOCK_SIZE >= DMA_TRIGGER_IRQ_AES)
			dmaUse = PUBLIC_CRYPTO_DMA_USE_IRQ;
		else
			dmaUse = PUBLIC_CRYPTO_DMA_USE_POLLING;
	}

	dprintk(KERN_INFO "PDrvCryptoUpdateAES: \
		pSrc=0x%08x, pDest=0x%08x, nbBlocks=0x%08x, dmaUse=0x%08x\n",
		(unsigned int)pSrc,
		(unsigned int)pDest,
		(unsigned int)nbBlocks,
		(unsigned int)dmaUse);

	if (nbBlocks == 0) {
		dprintk(KERN_INFO "PDrvCryptoUpdateAES: Nothing to process\n");
		return;
	}

	/*Restore the registers of the accelerator from the operation state */
	PDrvCryptoRestoreAESRegisters(AES_CTRL, pAESState);

	if ((dmaUse == PUBLIC_CRYPTO_DMA_USE_POLLING)
		 || (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ)) {

		/*perform the update with DMA */
		PDrvCryptoUpdateAESWithDMA(pProcessSrc,
				pProcessDest, nbBlocks, dmaUse);

	} else {
		for (nbr_of_blocks = 0;
			nbr_of_blocks < nbBlocks; nbr_of_blocks++) {

			/*We wait for the input ready */

			/*Crash the system as this should never occur */
			if (scxPublicCryptoWaitForReadyBit(
				(VU32 *)&pAESReg_t->AES_CTRL,
				AES_CTRL_INPUT_READY_BIT) !=
					PUBLIC_CRYPTO_OPERATION_SUCCESS)
					panic("Wait too long for AES hardware \
					accelerator Input data to be ready\n");

			/*We copy the 16 bytes of data src->reg */
			vTemp = (u32) BYTES_TO_LONG(pProcessSrc);
			OUTREG32(&pAESReg_t->AES_DATA_1, vTemp);
			pProcessSrc += 4;
			vTemp = (u32) BYTES_TO_LONG(pProcessSrc);
			OUTREG32(&pAESReg_t->AES_DATA_2, vTemp);
			pProcessSrc += 4;
			vTemp = (u32) BYTES_TO_LONG(pProcessSrc);
			OUTREG32(&pAESReg_t->AES_DATA_3, vTemp);
			pProcessSrc += 4;
			vTemp = (u32) BYTES_TO_LONG(pProcessSrc);
			OUTREG32(&pAESReg_t->AES_DATA_4, vTemp);
			pProcessSrc += 4;

			/*We wait for the output ready */
			scxPublicCryptoWaitForReadyBitInfinitely(
					(VU32 *)&pAESReg_t->AES_CTRL,
					AES_CTRL_OUTPUT_READY_BIT);

			/*We copy the 16 bytes of data reg->dest */
			vTemp = INREG32(&pAESReg_t->AES_DATA_1);
			LONG_TO_BYTE(pProcessDest, vTemp);
			pProcessDest += 4;
			vTemp = INREG32(&pAESReg_t->AES_DATA_2);
			LONG_TO_BYTE(pProcessDest, vTemp);
			pProcessDest += 4;
			vTemp = INREG32(&pAESReg_t->AES_DATA_3);
			LONG_TO_BYTE(pProcessDest, vTemp);
			pProcessDest += 4;
			vTemp = INREG32(&pAESReg_t->AES_DATA_4);
			LONG_TO_BYTE(pProcessDest, vTemp);
			pProcessDest += 4;
		}
	}

	/*Save the accelerator registers into the operation state */
	PDrvCryptoSaveAESRegisters(AES_CTRL, pAESState);

	dprintk(KERN_INFO "PDrvCryptoUpdateAES: Done\n");
}

/*-------------------------------------------------------------------------- */
/*
 *Static function, perform AES encryption/decryption using the DMA for data
 *transfer.
 *
 *inputs: pSrc : pointer of the input data to process
 *        nbBlocks : number of block to process
 *        dmaUse : PUBLIC_CRYPTO_DMA_USE_IRQ (use irq to monitor end of DMA)
 *                     | PUBLIC_CRYPTO_DMA_USE_POLLING (poll the end of DMA)
 *output: pDest : pointer of the output data (can be eq to pSrc)
 */
static void PDrvCryptoUpdateAESWithDMA(u8 *pSrc, u8 *pDest,
					u32 nbBlocks, u32 dmaUse)
{
	AESReg_t *pAESReg_t = (AESReg_t *)IO_ADDRESS(AES1_REGS_HW_ADDR);

	/*
	 *Note: The DMA only sees physical addresses !
	 */

	u32 dma_ch0 = OMAP3XXX_SMC_DMA_CH_0;
	u32 dma_ch1 = OMAP3XXX_SMC_DMA_CH_1;
	SCX_DMA_CHANNEL_PARAM ch0_parameters;
	SCX_DMA_CHANNEL_PARAM ch1_parameters;
	u32 nLength = nbBlocks * AES_BLOCK_SIZE;
	u32 nLengthLoop = 0;
	u32 nbBlocksLoop = 0;
	u32 bufInOut_phys = virt_to_phys(g_SCXLNXDeviceMonitor.pDMABuffer);

	dprintk(KERN_INFO
		"PDrvCryptoUpdateAESWithDMA: In=0x%08x, Out=0x%08x, Len=%u\n",
		(unsigned int)pSrc,
		(unsigned int)pDest,
		(unsigned int)nLength);

	/*lock the DMA */
	down(&g_SCXLNXDeviceMonitor.sm.sDMALock);

	while (nLength > 0) {

		/*
		 * At this time, we are sure that the DMAchannels
		 *are available and not used by other public crypto operation
		 */

		/*disable the channels before configuration */
		scxPublicDMADisableChannel(dma_ch0);
		scxPublicDMADisableChannel(dma_ch1);

		/*
		 *Reset DMA int (DMA CTRL) -
		 *The DMA int (INT CTRL)is reset by the OS
		 */
		if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
			scxPublicDMADisableL3IRQ();
			scxPublicDMAClearL3IRQ();
		}

		/*DMA used for Input and Output */
		OUTREG32(&pAESReg_t->AES_MASK, INREG32(&pAESReg_t->AES_MASK)
			| AES_MASK_DMA_REQ_OUT_EN_BIT
			| AES_MASK_DMA_REQ_IN_EN_BIT);

		/*check length */
		if (nLength <= g_SCXLNXDeviceMonitor.nDMABufferLength)
			nLengthLoop = nLength;
		else
			nLengthLoop = g_SCXLNXDeviceMonitor.nDMABufferLength;

		/*The length is always a multiple of the block size */
		nbBlocksLoop = nLengthLoop / AES_BLOCK_SIZE;

		/*
		 *Copy the data from the input buffer into a preallocated
		 *buffer which is aligned on the beginning of a page.
		 *This may prevent potential issues when flushing/invalidating
		 *the buffer as the cache lines are 64 bytes long.
		 */
		memcpy(g_SCXLNXDeviceMonitor.pDMABuffer, pSrc, nLengthLoop);

		/*
		 *The input data may be in the cache only,
		 *and the DMA is only working with physical addresses.
		 *So flush the cache to have data coherency.
		 */
		v7_dma_flush_range((u32)g_SCXLNXDeviceMonitor.pDMABuffer,
			(u32)g_SCXLNXDeviceMonitor.pDMABuffer + nLengthLoop);

		/*DMA1: Mem -> AES */
		scxPublicSetDMAChannelCommonParams(&ch0_parameters,
						nbBlocksLoop,
						DMA_CEN_Elts_per_Frame_AES,
						AES1_REGS_HW_ADDR + 0x34,
						bufInOut_phys,
						DMA_CCR_Channel_Mem2AES1);

		/*specific data_type field for AES */
		ch0_parameters.data_type |= DMA_CSDP_Dest_Burst_16B
						| DMA_CSDP_Src_Burst_16B;

		/*specific for Mem -> HWA */
		ch0_parameters.src_amode = 1;	/*post increment */
		ch0_parameters.dst_amode = 0;	/*const */
		/*Transfert is triggered by the Dest */
		ch0_parameters.src_or_dst_synch = 0;

		dprintk(KERN_INFO "PDrvCryptoUpdateAESWithDMA: \
				scxPublicDMASetParams(ch0)\n");
		scxPublicDMASetParams(dma_ch0, &ch0_parameters);

		dprintk(KERN_INFO
			"PDrvCryptoUpdateAESWithDMA: Start DMA channel %d\n",
			(unsigned int)dma_ch0);
		scxPublicDMAStart(dma_ch0, OMAP_DMA_DROP_IRQ);

		/*DMA2: AES -> Mem */
		scxPublicSetDMAChannelCommonParams(&ch1_parameters,
						nbBlocksLoop,
						DMA_CEN_Elts_per_Frame_AES,
						bufInOut_phys,
						AES1_REGS_HW_ADDR + 0x34,
						DMA_CCR_Channel_AES12Mem);

		/*specific data_type field for AES */
		ch1_parameters.data_type |= DMA_CSDP_Dest_Burst_16B
						| DMA_CSDP_Src_Burst_16B;

		/*specific for HWA -> mem */
		ch1_parameters.src_amode = 0;	/*const */
		ch1_parameters.dst_amode = 1;	/*post increment */
		/*Transfert is triggered by the Src */
		ch1_parameters.src_or_dst_synch = 1;

		dprintk(KERN_INFO "PDrvCryptoUpdateAESWithDMA: \
			scxPublicDMASetParams(ch1)\n");
		scxPublicDMASetParams(dma_ch1, &ch1_parameters);

		if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ)
			scxPublicDMAEnableL3IRQ();

		dprintk(KERN_INFO "PDrvCryptoUpdateAESWithDMA: \
			Start DMA channel %d\n",
			(unsigned int)dma_ch1);
		scxPublicDMAStart(dma_ch1, OMAP_DMA_DROP_IRQ
						| OMAP_DMA_BLOCK_IRQ);

		/*Start operation */
		OUTREG32(&pAESReg_t->AES_MASK,
			INREG32(&pAESReg_t->AES_MASK) | AES_MASK_START_BIT);

		if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
			/*Suspends the process until the DMA IRQ occurs */
			dprintk(KERN_INFO
			  "PDrvCryptoUpdateAESWithDMA: Waiting for IRQ\n");
			scxPublicDMAWait();
		} else {
			dprintk(KERN_INFO
				"PDrvCryptoUpdateAESWithDMA: Polling DMA\n");
			scxPublicDMAPoll(dma_ch1);
		}

		if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
			/*Acknowledge DMA interrupt */
			scxPublicDMADisableL3IRQ();
		}

		scxPublicDMAClearChannel(dma_ch0);
		scxPublicDMAClearChannel(dma_ch1);

		/*
		 *The dma transfer is complete
		 */

		/*Stop clocks */
		OUTREG32(&pAESReg_t->AES_MASK, INREG32(&pAESReg_t->AES_MASK)
			& (~AES_MASK_START_BIT));

		/*Unset DMA synchronisation requests */
		OUTREG32(&pAESReg_t->AES_MASK, INREG32(&pAESReg_t->AES_MASK)
			& (~AES_MASK_DMA_REQ_OUT_EN_BIT)
			& (~AES_MASK_DMA_REQ_IN_EN_BIT));

		/*
		 *The output data are in the physical memory.
		 *So invalidate the cache to have data coherency.
		 */
		v7_dma_inv_range((u32)g_SCXLNXDeviceMonitor.pDMABuffer,
			(u32)g_SCXLNXDeviceMonitor.pDMABuffer + nLengthLoop);

		/*The DMA output is in the preallocated aligned buffer
		 *and needs to be copied to the output buffer.*/
		memcpy(pDest, g_SCXLNXDeviceMonitor.pDMABuffer, nLengthLoop);

		pSrc += nLengthLoop;
		pDest += nLengthLoop;
		nLength -= nLengthLoop;
	}

	/*For safety reasons, let's clean the working buffer */
	memset(g_SCXLNXDeviceMonitor.pDMABuffer, 0, nLengthLoop);

	/*release the DMA */
	up(&g_SCXLNXDeviceMonitor.sm.sDMALock);

	dprintk(KERN_INFO "PDrvCryptoUpdateAESWithDMA: Success\n");
}
