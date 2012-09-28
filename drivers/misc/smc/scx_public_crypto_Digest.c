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
 *SHA2/MD5 Hardware Accelerator: Base address
 *This is referenced as the SHAM1 module in the Crypto TRM
 */
#define DIGEST1_REGS_HW_ADDR					0x480A4000

/*
 *IRQSTAT register Masks
 */
#define DIGEST_IRQSTAT_OUTPUT_READY_BIT	(1<<0)
#define DIGEST_IRQSTAT_INPUT_READY_BIT		(1<<1)
#define DIGEST_IRQSTAT_PARTHASH_READY		(1<<2)

/*
 *CTRL register Masks
 */
#define DIGEST_CTRL_GET_ALGO(x)			((x&0x6) >> 1)
#define DIGEST_CTRL_SET_ALGO(x, a)		((a<<1) | (x&0xFFFFFFF9))

#define DIGEST_CTRL_ALGO_CONST_BIT		(1<<3)
#define DIGEST_CTRL_CLOSE_HASH_BIT		(1<<4)

#define DIGEST_CTRL_GET_LENGTH(x)	((x>>5) & 0x07FFFFFF)
#define DIGEST_CTRL_SET_LENGTH(x, l) ((x&0x0000001F) | ((l<<5) & 0xFFFFFFE0))

/*
 *MASK register masks
 */
#define DIGEST_MASK_AUTOIDLE_BIT		(1<<0)
#define DIGEST_MASK_SOFTRESET_BIT		(1<<1)
#define DIGEST_MASK_IT_EN_BIT			(1<<2)
#define DIGEST_MASK_DMA_EN_BIT			(1<<3)

#define DIGEST_MASK_GET_SIDLE(x)		((x&0x20) >> 4)
#define DIGEST_MASK_SET_SIDLE(x, s)		((s<<4) | (x&0xFFFFFFCF))
#define DIGEST_MASK_SIDLE_FORCE_IDLE		0
#define DIGEST_MASK_SIDLE_NO_IDLE		1
#define DIGEST_MASK_SIDLE_SMART_IDLE		2

#define DIGEST_MASK_CONTEXT_BIT			(1<<6)

/*
 *SYSTATUS register masks
 */
#define DIGEST_SYSTATUS_RESET_DONE		(1<<0)


/*-------------------------------------------------------------------------*/
/*				 Digest Context				*/
/*-------------------------------------------------------------------------*/
/**
 *This structure contains the registers of the SHA1/MD5 HW accelerator.
 */
typedef struct {
	VU32 DIGEST_A;		/*0x00 Digest A      */
	VU32 DIGEST_B;		/*0x04 Digest B      */
	VU32 DIGEST_C;		/*0x08 Digest C      */
	VU32 DIGEST_D;		/*0x0C Digest D      */
	VU32 DIGEST_E;		/*0x10 Digest E      */
	VU32 DIGEST_F;		/*0x14 Digest F      */
	VU32 DIGEST_G;		/*0x18 Digest G      */
	VU32 DIGEST_H;		/*0x1C Digest H      */
	VU32 DIGCNT;		/*0x20 Digest count  */
	VU32 BYTE;			/*0x24 Byte          */
	VU32 IRQSTAT;		/*0x28 Irq status    */
	VU32 CTRL;			/*0x2C Control       */
	VU32 DIN_0;			/*0x30 Data 0        */
	VU32 DIN_1;			/*0x34 Data 1        */
	VU32 DIN_2;			/*0x38 Data 2        */
	VU32 DIN_3;			/*0x3C Data 3        */
	VU32 DIN_4;			/*0x40 Data 4        */
	VU32 DIN_5;			/*0x44 Data 5        */
	VU32 DIN_6;			/*0x48 Data 6        */
	VU32 DIN_7;			/*0x4C Data 7        */
	VU32 DIN_8;			/*0x50 Data 8        */
	VU32 DIN_9;			/*0x54 Data 9        */
	VU32 DIN_10;		/*0x58 Data 10       */
	VU32 DIN_11;		/*0x5C Data 11       */
	VU32 DIN_12;		/*0x60 Data 12       */
	VU32 DIN_13;		/*0x64 Data 13       */
	VU32 DIN_14;		/*0x68 Data 14       */
	VU32 DIN_15;		/*0x6C Data 15       */
	VU32 REV;			/*0x70 Revision      */
	VU32 MASK;			/*0x74 Mask          */
	VU32 SYSSTATUS;	/*0x78 System status */
} Sha1Md5Reg_t;


/*------------------------------------------------------------------------
 *Forward declarations
 *------------------------------------------------------------------------- */

static void static_Hash_HwPerform64bDigest(u32 *pData,
				u32 nAlgo, u32 nBytesProcessed);
static void static_Hash_HwPerformDmaDigest(u8 *pData, u32 nDataLength,
				u32 nAlgo, u32 nBytesProcessed, u32 dmaUse);

static void PDrvCryptoUpdateHashWithDMA(u32 SHA_CTRL,
				PUBLIC_CRYPTO_SHA_OPERATION_STATE *pSHAState,
				u8 *pData, u32 dataLength, u32 dmaUse);


/*-------------------------------------------------------------------------
 *Save HWA registers into the specified operation state structure
 *------------------------------------------------------------------------*/
static void PDrvCryptoSaveHashRegisters(
			PUBLIC_CRYPTO_SHA_OPERATION_STATE *pSHAState)
{
	Sha1Md5Reg_t *pSha1Md5Reg_t =
		(Sha1Md5Reg_t *)IO_ADDRESS(DIGEST1_REGS_HW_ADDR);

	dprintk(KERN_INFO "PDrvCryptoSaveHashRegisters: State=%p\n",
								pSHAState);

	pSHAState->SHA_DIGEST_A = INREG32(&pSha1Md5Reg_t->DIGEST_A);
	pSHAState->SHA_DIGEST_B = INREG32(&pSha1Md5Reg_t->DIGEST_B);
	pSHAState->SHA_DIGEST_C = INREG32(&pSha1Md5Reg_t->DIGEST_C);
	pSHAState->SHA_DIGEST_D = INREG32(&pSha1Md5Reg_t->DIGEST_D);
	pSHAState->SHA_DIGEST_E = INREG32(&pSha1Md5Reg_t->DIGEST_E);
	pSHAState->SHA_DIGEST_F = INREG32(&pSha1Md5Reg_t->DIGEST_F);
	pSHAState->SHA_DIGEST_G = INREG32(&pSha1Md5Reg_t->DIGEST_G);
	pSHAState->SHA_DIGEST_H = INREG32(&pSha1Md5Reg_t->DIGEST_H);
}

/*-------------------------------------------------------------------------
 *Restore the HWA registers from the operation state structure
 *-------------------------------------------------------------------------*/
static void PDrvCryptoRestoreHashRegisters(
			PUBLIC_CRYPTO_SHA_OPERATION_STATE *pSHAState)
{
	Sha1Md5Reg_t *pSha1Md5Reg_t =
			(Sha1Md5Reg_t *)IO_ADDRESS(DIGEST1_REGS_HW_ADDR);

	dprintk(KERN_INFO "PDrvCryptoRestoreHashRegisters: State=%p\n",
		pSHAState);

	if (pSHAState->nBytesProcessed != 0) {
		/*Some bytes were already processed. Initialize previous
		 *digest */
		OUTREG32(&pSha1Md5Reg_t->DIGEST_A, pSHAState->SHA_DIGEST_A);
		OUTREG32(&pSha1Md5Reg_t->DIGEST_B, pSHAState->SHA_DIGEST_B);
		OUTREG32(&pSha1Md5Reg_t->DIGEST_C, pSHAState->SHA_DIGEST_C);
		OUTREG32(&pSha1Md5Reg_t->DIGEST_D, pSHAState->SHA_DIGEST_D);
		OUTREG32(&pSha1Md5Reg_t->DIGEST_E, pSHAState->SHA_DIGEST_E);
		OUTREG32(&pSha1Md5Reg_t->DIGEST_F, pSHAState->SHA_DIGEST_F);
		OUTREG32(&pSha1Md5Reg_t->DIGEST_G, pSHAState->SHA_DIGEST_G);
		OUTREG32(&pSha1Md5Reg_t->DIGEST_H, pSHAState->SHA_DIGEST_H);
	}

	OUTREG32(&pSha1Md5Reg_t->MASK, 0);
}

/*------------------------------------------------------------------------- */

void PDrvCryptoUpdateHash(
			u32 SHA_CTRL,
			PUBLIC_CRYPTO_SHA_OPERATION_STATE *pSHAState,
			u8 *pData, u32 dataLength)
{
	u32 dmaUse = PUBLIC_CRYPTO_DMA_USE_NONE;

	/*
	 *Choice of the processing type
	 */
	if (dataLength >= DMA_TRIGGER_POLLING_DIGEST) {
		if (dataLength >= DMA_TRIGGER_IRQ_DIGEST)
			dmaUse = PUBLIC_CRYPTO_DMA_USE_IRQ;
		else
			dmaUse = PUBLIC_CRYPTO_DMA_USE_POLLING;
	}

	dprintk(KERN_INFO "PDrvCryptoUpdateHash : \
		Data=0x%08x/%u, Chunck=%u, Processed=%u, dmaUse=0x%08x\n",
		(u32)pData, (u32)dataLength,
		pSHAState->nChunkLength, pSHAState->nBytesProcessed,
		dmaUse);

	if (dataLength == 0) {
		dprintk(KERN_INFO "PDrvCryptoUpdateHash: \
				Nothing to process\n");
		return;
	}

	/*Restore the registers of the accelerator from the operation state */
	PDrvCryptoRestoreHashRegisters(pSHAState);

	if (dmaUse != PUBLIC_CRYPTO_DMA_USE_NONE) {

		/*perform the updates with DMA */
		PDrvCryptoUpdateHashWithDMA(SHA_CTRL,
				pSHAState, pData, dataLength, dmaUse);

	} else {
		/*Non-DMA transfer */

		/*(1)We take the chunk buffer wich contains the last saved
		 *data that could not be yet processed because we had not
		 *enough data to make a 64B buffer. Then we try to make a
		 *64B buffer by concatenating it with the new passed data
		 */

		/*Is there any data in the chunk? If yes is it possible to
		 *make a 64B buffer with the new data passed ? */
		if ((pSHAState->nChunkLength != 0)
			&& (pSHAState->nChunkLength + dataLength >=
						HASH_BLOCK_BYTES_LENGTH)) {

			u8 vLengthToComplete =
			HASH_BLOCK_BYTES_LENGTH - pSHAState->nChunkLength;

			/*So we fill the chunk buffer with the new data to
			 *complete to 64B */
			memcpy(pSHAState->pChunkBuffer + pSHAState->
				nChunkLength, pData, vLengthToComplete);

			if (pSHAState->nChunkLength + dataLength ==
				HASH_BLOCK_BYTES_LENGTH) {
				/*We'll keep some data for the final */
				pSHAState->nChunkLength =
					HASH_BLOCK_BYTES_LENGTH;
				dprintk(KERN_INFO "PDrvCryptoUpdateHash: \
					Done: Chunck=%u; Processed=%u\n",
					pSHAState->nChunkLength,
					pSHAState->nBytesProcessed);
				return;
			}

			/*Then we send this buffer to the HWA */
			static_Hash_HwPerform64bDigest(
				(u32 *)pSHAState->pChunkBuffer, SHA_CTRL,
				pSHAState->nBytesProcessed);

			pSHAState->nBytesProcessed += HASH_BLOCK_BYTES_LENGTH;

			/*We have flushed the chunk so it is empty now */
			pSHAState->nChunkLength = 0;

			/*Then we have less data to process */
			pData += vLengthToComplete;
			dataLength -= vLengthToComplete;
		}

		/*(2)We process all the 64B buffer that we can */
		if (pSHAState->nChunkLength + dataLength >=
					HASH_BLOCK_BYTES_LENGTH) {

			while (dataLength > HASH_BLOCK_BYTES_LENGTH) {
				u8 pTempAlignedBuffer[HASH_BLOCK_BYTES_LENGTH];

				/*
				 *We process a 64B buffer
				 */
				/*We copy the data to process to an aligned
				 *buffer */
				memcpy(pTempAlignedBuffer, pData,
					HASH_BLOCK_BYTES_LENGTH);

				/*Then we send this buffer to the hash
				 *hardware */
				static_Hash_HwPerform64bDigest(
					(u32 *)pTempAlignedBuffer, SHA_CTRL,
						pSHAState->nBytesProcessed);

				pSHAState->nBytesProcessed +=
					HASH_BLOCK_BYTES_LENGTH;

				/*Then we decrease the remaining data of 64B */
				pData += HASH_BLOCK_BYTES_LENGTH;
				dataLength -= HASH_BLOCK_BYTES_LENGTH;
			}
		}

		/*(3)We look if we have some data that could not be processed
		 *yet because it is not large enough to fill a buffer of 64B */
		if (dataLength > 0) {
			if (pSHAState->nChunkLength + dataLength >
					HASH_BLOCK_BYTES_LENGTH) {
				/*Should never be in this case !!! */
			panic("PDrvCryptoUpdateHash: nChunkLength + \
				dataLength > HASH_BLOCK_BYTES_LENGTH\n");
			}

			/*So we fill the chunk buffer with the new data to
			 *complete to 64B */
			memcpy(pSHAState->pChunkBuffer + pSHAState->
				nChunkLength, pData, dataLength);
			pSHAState->nChunkLength += dataLength;
		}
	}

	/*Save the accelerator registers into the operation state */
	PDrvCryptoSaveHashRegisters(pSHAState);

	dprintk(KERN_INFO "PDrvCryptoUpdateHash: Done: \
		Chunck=%u; Processed=%u\n",
		pSHAState->nChunkLength, pSHAState->nBytesProcessed);
}

/*------------------------------------------------------------------------- */

static void static_Hash_HwPerform64bDigest(u32 *pData,
					u32 nAlgo, u32 nBytesProcessed)
{
	Sha1Md5Reg_t *pSha1Md5Reg_t =
		(Sha1Md5Reg_t *)IO_ADDRESS(DIGEST1_REGS_HW_ADDR);

	u32 nAlgoConstant = 0;

	/*
	 *Set DIGCNT to 0 (DIGCNT is used only for the final hash)
	 */
	OUTREG32(&pSha1Md5Reg_t->DIGCNT, 0);

	if (nBytesProcessed == 0) {
		/*No bytes processed so far. Will use the algo constant instead
			of previous digest */
		nAlgoConstant = 1 << 3;
	}

	OUTREG32(&pSha1Md5Reg_t->CTRL,
	  nAlgoConstant | (nAlgo & 0x6) | (HASH_BLOCK_BYTES_LENGTH << 5));

	if (scxPublicCryptoWaitForReadyBit(
		(VU32 *)&pSha1Md5Reg_t->IRQSTAT,
		DIGEST_IRQSTAT_INPUT_READY_BIT) !=
			PUBLIC_CRYPTO_OPERATION_SUCCESS) {
		/*Crash the system as this should never occur */
		panic("Wait too long for DIGEST HW accelerator \
			Input data to be ready\n");
	}

	/*
	 *The pData buffer is a buffer of 64 bytes.
	 */
	OUTREG32(&pSha1Md5Reg_t->DIN_0, pData[0]);
	OUTREG32(&pSha1Md5Reg_t->DIN_1, pData[1]);
	OUTREG32(&pSha1Md5Reg_t->DIN_2, pData[2]);
	OUTREG32(&pSha1Md5Reg_t->DIN_3, pData[3]);
	OUTREG32(&pSha1Md5Reg_t->DIN_4, pData[4]);
	OUTREG32(&pSha1Md5Reg_t->DIN_5, pData[5]);
	OUTREG32(&pSha1Md5Reg_t->DIN_6, pData[6]);
	OUTREG32(&pSha1Md5Reg_t->DIN_7, pData[7]);
	OUTREG32(&pSha1Md5Reg_t->DIN_8, pData[8]);
	OUTREG32(&pSha1Md5Reg_t->DIN_9, pData[9]);
	OUTREG32(&pSha1Md5Reg_t->DIN_10, pData[10]);
	OUTREG32(&pSha1Md5Reg_t->DIN_11, pData[11]);
	OUTREG32(&pSha1Md5Reg_t->DIN_12, pData[12]);
	OUTREG32(&pSha1Md5Reg_t->DIN_13, pData[13]);
	OUTREG32(&pSha1Md5Reg_t->DIN_14, pData[14]);
	OUTREG32(&pSha1Md5Reg_t->DIN_15, pData[15]);

	/*
	 *Wait until the hash operation is finished.
	 */
	scxPublicCryptoWaitForReadyBitInfinitely((VU32 *)&pSha1Md5Reg_t->
					IRQSTAT,
					DIGEST_IRQSTAT_OUTPUT_READY_BIT);
}

/*------------------------------------------------------------------------- */

static void static_Hash_HwPerformDmaDigest(u8 *pData, u32 nDataLength,
			u32 nAlgo, u32 nBytesProcessed, u32 dmaUse)
{
	Sha1Md5Reg_t *pSha1Md5Reg_t =
		(Sha1Md5Reg_t *)IO_ADDRESS(DIGEST1_REGS_HW_ADDR);

	/*
	 *Note: The DMA only sees physical addresses !
	 */

	u32 dma_ch0 = OMAP3XXX_SMC_DMA_CH_0;	/*Hard Coded Tx Data */
	SCX_DMA_CHANNEL_PARAM ch0_parameters;
	u32 nLengthLoop = 0;
	u32 nAlgoConstant;
	u32 bufIn_phys = virt_to_phys(g_SCXLNXDeviceMonitor.pDMABuffer);

	dprintk(KERN_INFO
		"static_Hash_HwPerformDmaDigest: Buffer=0x%08x/%u\n",
		(u32)pData, (u32)nDataLength);

	/*lock the DMA */
	down(&g_SCXLNXDeviceMonitor.sm.sDMALock);
	/*at this time, we are sure that the DMAchannels are available and not
	 *used by other public crypto operation */

	while (nDataLength > 0) {

		nAlgoConstant = 0;
		if (nBytesProcessed == 0) {
			/*No bytes processed so far. Will use the algo
			 *constant instead of previous digest */
			nAlgoConstant = 1 << 3;
		}

		/*check length */
		if (nDataLength <= g_SCXLNXDeviceMonitor.nDMABufferLength)
			nLengthLoop = nDataLength;
		else
			nLengthLoop = g_SCXLNXDeviceMonitor.nDMABufferLength;

		/*
		 *Copy the data from the input buffer into a preallocated
		 *buffer which is aligned on the beginning of a page.
		 *This may prevent potential issues when flushing/invalidating
		 *the buffer as the cache lines are 64 bytes long.
		 */
		memcpy(g_SCXLNXDeviceMonitor.pDMABuffer, pData, nLengthLoop);

		/*
		 *The input data may be in the cache only,
		 *and the DMA is only working with physical addresses.
		 *So flush the cache to have data coherency.
		 */
		v7_dma_flush_range((u32)g_SCXLNXDeviceMonitor.pDMABuffer,
			(u32)g_SCXLNXDeviceMonitor.pDMABuffer + nLengthLoop);

		/*disable the channels before configuration */
		scxPublicDMADisableChannel(dma_ch0);

		if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
			/*Reset DMA int (DMA CTRL) -
			 *he DMA int (INT CTRL)is reset by the OS */
			scxPublicDMADisableL3IRQ();
			scxPublicDMAClearL3IRQ();
		}

		/*DMA1: Mem -> HASH */
		scxPublicSetDMAChannelCommonParams(&ch0_parameters,
						nLengthLoop /
						HASH_BLOCK_BYTES_LENGTH,
						DMA_CEN_Elts_per_Frame_SHA,
						DIGEST1_REGS_HW_ADDR + 0x30,
						bufIn_phys,
						DMA_CCR_Channel_Mem2SHA);

		/*specific data_type field for SHA */
		ch0_parameters.data_type |=
			DMA_CSDP_Dest_Burst_64B | DMA_CSDP_Src_Burst_64B;

		/*specific for Mem -> HWA */
		ch0_parameters.src_amode = 1;	/*post increment */
		ch0_parameters.dst_amode = 0;	/*const */
		/*Transfert is triggered by the Dest */
		ch0_parameters.src_or_dst_synch = 0;

		scxPublicDMASetParams(dma_ch0, &ch0_parameters);

		/*
		 *Set the DIGCNT register to 0
		 *(this register is used for final hash only)
		 */
		OUTREG32(&pSha1Md5Reg_t->DIGCNT, 0);

		OUTREG32(&pSha1Md5Reg_t->CTRL,
			nAlgoConstant | (nAlgo & 0x6) | (nLengthLoop << 5));

		if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ)
			scxPublicDMAEnableL3IRQ();

		/*Start operation */
		/*Triggers operation - Interrupt, Free Running + GO (DMA on) */
		OUTREG32(&pSha1Md5Reg_t->MASK, INREG32(&pSha1Md5Reg_t->MASK) |
			DIGEST_MASK_DMA_EN_BIT | DIGEST_MASK_IT_EN_BIT);

		scxPublicDMAStart(
			dma_ch0, OMAP_DMA_DROP_IRQ | OMAP_DMA_BLOCK_IRQ);

		if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
			/*Suspends the process until the DMA IRQ occurs */
			scxPublicDMAWait();
		} else {
			scxPublicDMAPoll(dma_ch0);
		}

		if (dmaUse == PUBLIC_CRYPTO_DMA_USE_IRQ) {
			/*Acknoledge DMA interrupt */
			scxPublicDMADisableL3IRQ();
		}

		scxPublicDMAClearChannel(dma_ch0);

		pData += nLengthLoop;
		nDataLength -= nLengthLoop;
		nBytesProcessed += nLengthLoop;
	}

	/*For safety reasons, let's clean the working buffer */
	memset(g_SCXLNXDeviceMonitor.pDMABuffer, 0, nLengthLoop);

	/*release the DMA */
	up(&g_SCXLNXDeviceMonitor.sm.sDMALock);

	/*The dma transfert is finished, now wait until the hash operation is
	 * finished. */
	scxPublicCryptoWaitForReadyBitInfinitely((VU32 *)&pSha1Md5Reg_t->
					IRQSTAT,
					DIGEST_IRQSTAT_OUTPUT_READY_BIT);

	/*Stop clocks */
	OUTREG32(&pSha1Md5Reg_t->MASK, 0);
}

/*------------------------------------------------------------------------- */
/*
 *Static function, perform data digest using the DMA for data transfer.
 *
 *inputs:
 *        pData : pointer of the input data to process
 *        dataLength : number of byte to process
 *        dmaUse : PUBLIC_CRYPTO_DMA_USE_IRQ (use irq to monitor end of DMA)
 *                     | PUBLIC_CRYPTO_DMA_USE_POLLING (poll the end of DMA)
 */
static void PDrvCryptoUpdateHashWithDMA(u32 nSHA_CTRL,
				PUBLIC_CRYPTO_SHA_OPERATION_STATE *pSHAState,
				u8 *pData, u32 dataLength,
				u32 dmaUse)
{
	dprintk(KERN_INFO "PDrvCryptoUpdateHashWithDMA\n");

	if (pSHAState->nChunkLength != 0) {

		u32 vLengthToComplete;

		/*Fill the chunk first */
		if (pSHAState->
			nChunkLength + dataLength <= HASH_BLOCK_BYTES_LENGTH) {

			/*So we fill the chunk buffer with the new data */
			memcpy(pSHAState->
				pChunkBuffer + pSHAState->nChunkLength,
				pData, dataLength);
			pSHAState->nChunkLength += dataLength;

			/*We'll keep some data for the final */
			return;
		}

		vLengthToComplete = HASH_BLOCK_BYTES_LENGTH - pSHAState->
								nChunkLength;

		if (vLengthToComplete != 0) {
			/*So we fill the chunk buffer with the new data to
			 *complete to 64B */
			memcpy(pSHAState->pChunkBuffer + pSHAState->
				nChunkLength, pData, vLengthToComplete);
		}

		/*Then we send this buffer to the HWA (no DMA) */
		static_Hash_HwPerform64bDigest(
			(u32 *)pSHAState->pChunkBuffer, nSHA_CTRL,
					pSHAState->nBytesProcessed);

		pSHAState->nBytesProcessed += HASH_BLOCK_BYTES_LENGTH;

		/*We have flushed the chunk so it is empty now */
		pSHAState->nChunkLength = 0;

		/*Update the data buffer depending of the data already
		 *processed */
		pData += vLengthToComplete;
		dataLength -= vLengthToComplete;
	}

	if (dataLength > HASH_BLOCK_BYTES_LENGTH) {

		/*DMA only manages data length that is multiple of 64b */
		u32 vDmaProcessSize = dataLength & 0xFFFFFFC0;

		if (vDmaProcessSize == dataLength) {
			/*We keep one block for the final */
			vDmaProcessSize -= HASH_BLOCK_BYTES_LENGTH;
		}

		static_Hash_HwPerformDmaDigest(pData, vDmaProcessSize,
			nSHA_CTRL, pSHAState->nBytesProcessed, dmaUse);

		pSHAState->nBytesProcessed += vDmaProcessSize;
		pData += vDmaProcessSize;
		dataLength -= vDmaProcessSize;
	}

	/*At that point, there is less than 64b left to process*/
	if ((dataLength == 0) || (dataLength > HASH_BLOCK_BYTES_LENGTH)) {
		/*Should never be in this case !!! */
		panic("PDrvCryptoUpdateHASHWithDMA: \
			Remaining dataLength=%u\n", dataLength);
	}

	/*We now fill the chunk buffer with the remaining data */
	memcpy(pSHAState->pChunkBuffer, pData, dataLength);
	pSHAState->nChunkLength = dataLength;
}
