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

#include <linux/init.h>		/*For arch_initcall */
#include <linux/interrupt.h>	/*For irqreturn_t */
#include <asm/mach/irq.h>
#include <asm/irq.h>		/*For irqaction */
#include <asm/dma.h>

#include "scxlnx_defs.h"
#include "scxlnx_util.h"
#include "scx_public_dma.h"

#define OMAP34XX_DMA_BASE			0x48056000
#define OMAP34XX_IC_BASE			0x48200000

#define OMAP_DMA4_CCR_EN			(1 << 7)
#define OMAP_DMA4_CCR_BUF_DIS		(1 << 25)
#define INT_SDMA_IRQ3				(15)

/*
 *CLNK_CTRL Masks
 */
#define DMA_CLNK_CTRL_MASK_Link_yes		(1<<15)

/*
 *CSR Masks
 */
#define DMA_CSR_Is_Finished		(1<<5)


typedef volatile struct {
	VU32 CCR;	/*offset 0x0, chnl ctrl */
	VU32 CLNK_CTRL;/*offset 0x4, chnl link ctrl */
	VU32 CICR;	/*offset 0x8, chnl intr ctrl */
	VU32 CSR;	/*offset 0xC, chnl status */
	VU32 CSDP;	/*offset 0x10, chnl src, dest params */
	VU32 CEN;	/*offset 0x14, chnl element number */
	VU32 CFN;	/*offset 0x18, chnl frame number */
	VU32 CSSA;	/*offset 0x1C, chnl src start addr */
	VU32 CDSA;	/*offset 0x20, chnl dest start addr */
	VU32 CSEI;	/*offset 0x24, chnl src element index */
	VU32 CSFI;	/*offset 0x28, chnl src frame index */
	VU32 CDEI;	/*offset 0x2C, chnl destination element index */
	VU32 CDFI;	/*offset 0x30, chnl destination frame index */
	VU32 CSAC;	/*offset 0x34, chnl src address counter */
	VU32 CDAC;	/*offset 0x38, chnl dest address counter */
	VU32 CCEN;	/*offset 0x3C, chnl cur trans element no */
	VU32 CCFN;	/*offset 0x40, chnl cur trans frame no */
	VU32 COLOR;	/*offset 0x44, chnl DMA color key/solid color */
	VU32 ulRESERVED_1[6];	/*48-60 Reserved */
} OMAP_DMA_LC_REGS;

typedef volatile struct {
	VU32 DMA4_REVISION;		/*offset 0x0, Revision code */
	VU32 ulRESERVED_0x04;
	VU32 DMA4_IRQSTATUS_L0;	/*offset 0x08, intr status over line L0 */
	VU32 DMA4_IRQSTATUS_L1;	/*offset 0x0C, intr status over line L1 */
	VU32 DMA4_IRQSTATUS_L2;	/*offset 0x10, intr status over line L2 */
	VU32 DMA4_IRQSTATUS_L3;	/*offset 0x14, intr status over line L3 */
	VU32 DMA4_IRQENABLE_L0;	/*offset 0x18, intr enable over line L0 */
	VU32 DMA4_IRQENABLE_L1;	/*offset 0x1C, intr enable over line L1 */
	VU32 DMA4_IRQENABLE_L2;	/*offset 0x20, intr enable over line L2 */
	VU32 DMA4_IRQENABLE_L3;	/*offset 0x24, intr enable over line L3 */
	VU32 DMA4_SYSSTATUS;		/*offset 0x28, module status */
	VU32 DMA4_OCP_SYSCONFIG;/*offset 0x2C, OCP i/f params control */
	VU32 ulRESERVED_0x30[13];
	VU32 DMA4_CAPS_0;	/*offset 0x64, DMA capabilities reg 0 LSW */
	VU32 ulRESERVED_0x68;
	VU32 DMA4_CAPS_2;	/*offset 0x6C, DMA capabilities reg 2 */
	VU32 DMA4_CAPS_3;	/*offset 0x70, DMA capabilities reg 3 */
	VU32 DMA4_CAPS_4;	/*offset 0x74, DMA capabilities reg 4 */
	VU32 DMA4_GCR;		/*offset 0x78, chnl DMA global register */
	VU32 ulRESERVED_0x7C;
	OMAP_DMA_LC_REGS CHNL_CTRL[32];	/*offset 0x80-0x920, chnl */
} OMAP3XXX_SDMA_REGS;

typedef volatile struct {
	VU32 ulINTC_REVISION;	/*offset 0x0, IP revision code */
	VU32 ulRESERVED_04;
	VU32 ulRESERVED_08;
	VU32 ulRESERVED_0C;
	VU32 ulINTC_SYSCONFIG;	/*offset 0x10, OCP i/f params ctrl */
	VU32 ulINTC_SYSSTATUS;	/*offset 0x14, status info of mod */
	VU32 ulRESERVED_18;
	VU32 ulRESERVED_1C;
	VU32 ulRESERVED_20;
	VU32 ulRESERVED_24;
	VU32 ulRESERVED_28;
	VU32 ulRESERVED_2C;
	VU32 ulRESERVED_30;
	VU32 ulRESERVED_34;
	VU32 ulRESERVED_38;
	VU32 ulRESERVED_3C;
	VU32 ulINTC_SIR_IRQ;	/*offset 0x40, supplies active IRQ */
	/*intr number */
	VU32 ulINTC_SIR_FIQ;	/*offset 0x44, supplies active FIQ */
	/*intr number */
	VU32 ulINTC_CONTROL;	/*offset 0x48, global mask and new */
	/*intr agreement bits */
	VU32 ulINTC_PROTECTION;	/*offset 0x4C, ctrls protection of */
	/*other registers */
	VU32 ulINTC_IDLE;	/*offset 0x50, func clock auto-idle */
	/*disable register */
	VU32 ulRESERVED_3[11];
	VU32 ulINTC_ITR0;	/*offset 0x80, raw intr i/p status */
	/*before masking */
	VU32 ulINTC_MIR0;	/*offset 0x84, interrupt mask register */
	VU32 ulINTC_MIR_CLEAR0;	/*offset 0x88, clear intr mask bits */
	VU32 ulINTC_MIR_SET0;	/*offset 0x8C, set intr mask bits */
	VU32 ulINTC_ISR_SET0;	/*offset 0x90, set SW intr bits */
	VU32 ulINTC_ISR_CLEAR0;	/*offset 0x94, clear SW intr bits */
	VU32 ulINTC_PENDING_IRQ0;	/*offset 0x98, IRQ stat aft masking */
	VU32 ulINTC_PENDING_FIQ0;	/*offset 0x9C, FIQ stat aft masking */
	VU32 ulINTC_ITR1;		/*offset 0xA0, raw intr i/p status */
	/*before masking */
	VU32 ulINTC_MIR1;	/*offset 0xA4, intr mask reg */
	VU32 ulINTC_MIR_CLEAR1;	/*offset 0xA8, clr intr mask bits */
	VU32 ulINTC_MIR_SET1;	/*offset 0xAC, set intr mask bits */
	VU32 ulINTC_ISR_SET1;	/*offset 0xB0, set SW intr bits */
	VU32 ulINTC_ISR_CLEAR1;	/*offset 0xB4, clear SW intr bits */
	VU32 ulINTC_PENDING_IRQ1;	/*offset 0xB8, IRQ stat aft masking */
	VU32 ulINTC_PENDING_FIQ1;	/*offset 0xBC, FIQ stat aft masking */
	VU32 ulINTC_ITR2;		/*offset 0xC0, raw intr i/p status */
	/*before masking */
	VU32 ulINTC_MIR2;	/*offset 0xC4, intr mask register */
	VU32 ulINTC_MIR_CLEAR2;	/*offset 0xC8, clr intr mask bits */
	VU32 ulINTC_MIR_SET2;	/*offset 0xCC, set intr mask bits */
	VU32 ulINTC_ISR_SET2;	/*offset 0xD0, set SW intr bits */
	VU32 ulINTC_ISR_CLEAR2;	/*offset 0xD4, clr SW intr bits */
	VU32 ulINTC_PENDING_IRQ2;	/*offset 0xD8, IRQ status after */
	/*masking */
	VU32 ulINTC_PENDING_FIQ2;	/*offset 0xDC, FIQ status after */
	/*masking */
	VU32 ulRESERVED_4[8];
	VU32 ulINTC_ILR[96];	/*offset 0x100, IRQ/FIQ priority 0-96 */
} OMAP_MPUIC_REGS;

OMAP3XXX_SDMA_REGS *g_pDmaController;
OMAP_MPUIC_REGS *g_pIntController;

static volatile uint g_dmaEventFlag;
static DECLARE_WAIT_QUEUE_HEAD(g_dmaWaitQueue);

/*-------------------------------------------------------------------------- */
/*
 *Internal functions
 */

#ifdef SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ

static irqreturn_t scxPublicDMAIRQHandler(int irq, void *dev_id)
{
	/*DMA HANDLING
		- Masks all dma irq (line L3)
		- Wake up the suspended driver and return
		Note : It is not possible to determine the origin
		of the dma irq (in public)because the dma registers are secure.
	 */

	/*Masks of all the secure dma (on the line L3) */
	OUTREG32(&(g_pDmaController->DMA4_IRQENABLE_L3), 0x00000000);

	g_dmaEventFlag++;

	/*Awake process */
	wake_up(&g_dmaWaitQueue);

	return IRQ_HANDLED;
}

#endif				/*SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ */

#ifdef DEBUG

 void scxPublicDMAPrintRegister(int lch)
{
	dprintk(KERN_INFO " DMA[%d] CCR			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR)));
	dprintk(KERN_INFO " DMA[%d] CLNK_CTRL	0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CLNK_CTRL)));
	dprintk(KERN_INFO " DMA[%d] CICR			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CICR)));
	dprintk(KERN_INFO " DMA[%d] CSR			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CSR)));
	dprintk(KERN_INFO " DMA[%d] CSDP			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CSDP)));
	dprintk(KERN_INFO " DMA[%d] CEN			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CEN)));
	dprintk(KERN_INFO " DMA[%d] CFN			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CFN)));
	dprintk(KERN_INFO " DMA[%d] CSSA			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CSSA)));
	dprintk(KERN_INFO " DMA[%d] CDSA			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CDSA)));
	dprintk(KERN_INFO " DMA[%d] CSEI			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CSEI)));
	dprintk(KERN_INFO " DMA[%d] CSFI			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CSFI)));
	dprintk(KERN_INFO " DMA[%d] CDEI			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CDEI)));
	dprintk(KERN_INFO " DMA[%d] CDFI			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CDFI)));
	dprintk(KERN_INFO " DMA[%d] CSAC			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CSAC)));
	dprintk(KERN_INFO " DMA[%d] CDAC			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CDAC)));
	dprintk(KERN_INFO " DMA[%d] CCEN			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CCEN)));
	dprintk(KERN_INFO " DMA[%d] CCFN			0x%08x\n",
		lch, INREG32(&(g_pDmaController->CHNL_CTRL[lch].CCFN)));
}

void scxPublicDMAPrintGeneralRegister(void)
{
	printk(KERN_INFO "DMA4_IRQSTATUS_L3	0x%08x\n",
			INREG32(&(g_pDmaController->DMA4_IRQSTATUS_L3)));
	printk(KERN_INFO "DMA4_IRQENABLE_L3	0x%08x\n",
			INREG32(&(g_pDmaController->DMA4_IRQENABLE_L3)));
}

#endif	/*defined(DEBUG) */

/*-------------------------------------------------------------------------- */
/*
 *Public DMA API
 */

/*
 *Registers the ISR for the DMA (System DMA interrupt request 3)
 *Maps the DMA registers
 */
u32 scxPublicDMAInit(void)
{
	dprintk(KERN_INFO "scxPublicDMAInit\n");

	g_pDmaController = (OMAP3XXX_SDMA_REGS *)IO_ADDRESS(OMAP34XX_DMA_BASE);
	g_pIntController = (OMAP_MPUIC_REGS *) IO_ADDRESS(OMAP34XX_IC_BASE);

#ifdef SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ

	/*Disables all irq on L3 */
	OUTREG32(&g_pDmaController->DMA4_IRQENABLE_L3, 0x00000000);

	/*Reset dma interrupt status on L3 */
	OUTREG32(&g_pDmaController->DMA4_IRQSTATUS_L3, OMAP3XXX_SMC_DMA_ALL);

	{
		int result;

		result = request_irq(INT_SDMA_IRQ3,
					scxPublicDMAIRQHandler,
					IRQF_DISABLED, "SMC DMA IRQ", NULL);
		if (result) {
			dprintk(KERN_ERR "scxPublicDMAInit: \
				Cannot get assigned IRQ %d [%d]\n",
				INT_SDMA_IRQ3, result);
			return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
		}
	}

#endif	/*SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ */

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*-------------------------------------------------------------------------- */
/*
 *Unregisters the ISR for the DMA (System DMA interrupt request 3)
 *Destroy DMA Interrupt Thread.
 */
void scxPublicDMATerminate(void)
{
	dprintk(KERN_INFO "scxPublicDMATerminate\n");

#ifdef SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ

	free_irq(INT_SDMA_IRQ3, NULL);

#endif	/*SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ */
}

/*-------------------------------------------------------------------------- */
/*
 *Request a DMA channel
 *(between OMAP3XXX_SMC_DMA_CH_0 and OMAP3XXX_SMC_DMA_CH_3)
 */
u32 scxPublicDMARequest(int lch)
{
	if ((lch < OMAP3XXX_SMC_DMA_CH_0)
		 || (lch >= OMAP3XXX_SMC_DMA_CH_0 + OMAP3XXX_SMC_DMA_CH_NB)) {
		dprintk(KERN_ERR "scxPublicDMARequest: Bad DMA channel [%d]\n",
			lch);
		return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
	}
#ifdef SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ

	/*Enables irq to be generated by dma channel id 'lch' on irq line L3 */
	/*NOT NECESSARY */
	SETREG32(&g_pDmaController->DMA4_IRQENABLE_L3, (1 << lch));
	/*Makes sure no interrupt will occurs on the L0 line which is the line
	 *used by the other DMA
	 *Disables irq to be generated by dma channel id 'lch'
	 *on irq line L0 */
	/*Global mask of dma interrupts */
	CLRREG32(&g_pDmaController->DMA4_IRQENABLE_L0, (1 << lch));

	/*Clear the IRQ status register */
	OUTREG32(&g_pDmaController->DMA4_IRQSTATUS_L3, OMAP3XXX_SMC_DMA_ALL);

#endif	/*SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ */

	/*Cleans all registers of the dma channel id lch while this
		is possible that is before the dma channel is made secure and
		no more accessible in public. */
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CLNK_CTRL), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CICR), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSR), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSDP), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CEN), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CFN), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSSA), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CDSA), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSEI), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSFI), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CDEI), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CDFI), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSAC), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CDAC), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CCEN), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CCFN), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].COLOR), 0x00000000);

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*-------------------------------------------------------------------------- */
/*
 *Release a DMA channel
 *(between OMAP3XXX_SMC_DMA_CH_0 and OMAP3XXX_SMC_DMA_CH_3)
 */
u32 scxPublicDMARelease(int lch)
{
	if ((lch < OMAP3XXX_SMC_DMA_CH_0)
		 || (lch >= OMAP3XXX_SMC_DMA_CH_0 + OMAP3XXX_SMC_DMA_CH_NB)) {
		dprintk(KERN_ERR "scxPublicDMARelease: Bad DMA channel [%d]\n",
			lch);
		return PUBLIC_CRYPTO_ERR_BAD_PARAMETERS;
	}
#ifdef SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ

	/*disable interrupts *//*NOT NECESSARY */

	/*Global mask of dma interrupts */
	CLRREG32(&g_pDmaController->DMA4_IRQENABLE_L3, (1 << lch));

	/*Clear the IRQ status register */
	OUTREG32(&g_pDmaController->DMA4_IRQSTATUS_L3, (1 << lch));

#endif	/*SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ */

	/*
	 *Cleans all registers of the dma channel id lch.
	 *     CICR: Disable all DMA interrupts for the channel.
	 *     CCR: Make sure the DMA transfer is stopped.
	 */
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CLNK_CTRL), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CICR), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSR), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSDP), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CEN), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CFN), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSSA), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CDSA), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSEI), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSFI), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CDEI), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CDFI), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSAC), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CDAC), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CCEN), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CCFN), 0x00000000);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].COLOR), 0x00000000);

	return PUBLIC_CRYPTO_OPERATION_SUCCESS;
}

/*-------------------------------------------------------------------------- */

void scxPublicDMASetParams(int lch, SCX_DMA_CHANNEL_PARAM *pParams)
{
	u32 w;

	/*Set transfert params */
	w = pParams->data_type;
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSDP), w);

	if (pParams->trigger) {
		w = INREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR));

		if (pParams->trigger > 63)
			w |= 1 << 20;
		else
			w &= ~(1 << 20);

		if (pParams->trigger > 31)
			w |= 1 << 19;
		else
			w &= ~(1 << 19);

		w |= (pParams->trigger & 0x1f);

		if (pParams->sync_mode & 0x1)
			w |= 1 << 18;
		else
			w &= ~(1 << 18);

		if (pParams->sync_mode & 0x2)
			w |= 1 << 5;
		else
			w &= ~(1 << 5);

		if (pParams->src_or_dst_synch)
			w |= 1 << 24;	/*source synch */
		else
			w &= ~(1 << 24);	/*dest synch */

		OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR), w);
	}

	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CEN), pParams->elem_count);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CFN),
		pParams->frame_count);

	/*Set src params */
	w = INREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR));
	w &= ~(0x03 << 12);
	w |= pParams->src_amode << 12;
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR), w);

	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSSA), pParams->src_start);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSEI), pParams->src_ei);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSFI), pParams->src_fi);

	/*Set dest params */
	w = INREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR));
	w &= ~(0x03 << 14);
	w |= pParams->dst_amode << 14;
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR), w);

	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CDSA), pParams->dst_start);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CDEI), pParams->dst_ei);
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CDFI), pParams->dst_fi);
}

/*-------------------------------------------------------------------------- */

void scxPublicDMAStart(int lch, int interruptMask)
{
	u32 w;

	/*DMA Errata - need to write lch to NEXTLCH_ID
	 *even if we are not using chaining!
	 */
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CLNK_CTRL), lch);

	/*Read CSR to make sure it's cleared. */
	w = INREG32(&(g_pDmaController->CHNL_CTRL[lch].CSR));

	/*Enable some nice interrupts. */
	/*w = INREG32(&(g_pDmaController->CHNL_CTRL[lch].CICR)); */
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CICR), interruptMask);

	w = INREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR));
	w |= OMAP_DMA4_CCR_EN;

	/*DMA Errata: for 24XX only: the BUFFERING Disable must be set to
	 *handle multiple failure scenarios
	 *Only for SRC sync(Bit 24)
	 */
	if ((0 == (w & (1 << 24))))
		w &= ~(OMAP_DMA4_CCR_BUF_DIS);
	else
		w |= OMAP_DMA4_CCR_BUF_DIS;

	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR), w);
}

/*-------------------------------------------------------------------------- */

void scxPublicDMADisableChannel(int lch)
{
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CCR), 0x0);
}

/*-------------------------------------------------------------------------- */

void scxPublicDMAClearChannel(int lch)
{
	/*Reset 'End of Block' event */
	OUTREG32(&(g_pDmaController->CHNL_CTRL[lch].CSR), DMA_CSR_Is_Finished);

#ifdef SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ

	{
		/*Reset Interrupt status bit */
		u32 w;
		w = INREG32(&(g_pDmaController->DMA4_IRQSTATUS_L3));
	}

	/*ch in this function is from 0-31 while in register it is 1-32 */
	OUTREG32(&(g_pDmaController->DMA4_IRQSTATUS_L3), 1 << (lch));

#endif	/*SMODULE_SMC_OMAP3XXX_PUBLIC_DMA_IRQ */
}

/*-------------------------------------------------------------------------- */

void scxPublicDMAClearL3IRQ(void)
{
	g_dmaEventFlag = 0;
	OUTREG32(&(g_pDmaController->DMA4_IRQSTATUS_L3), 0xFFFFFFFF);
}

/*-------------------------------------------------------------------------- */

void scxPublicDMAEnableL3IRQ(void)
{
	/*UnMasks of all the dma irq (on the line L3) */

	/*Active ALL dma irq */
	OUTREG32(&(g_pDmaController->DMA4_IRQENABLE_L3), OMAP3XXX_SMC_DMA_ALL);

	/*Interrupt controller */
	OUTREG32(&g_pIntController->ulINTC_MIR_CLEAR0, 1 << INT_SDMA_IRQ3);
}

/*-------------------------------------------------------------------------- */

void scxPublicDMADisableL3IRQ(void)
{
	OUTREG32(&(g_pDmaController->DMA4_IRQENABLE_L3), 0x00000000);
}

/*-------------------------------------------------------------------------- */

void scxPublicDMAPoll(int lch)
{
	while (!
			 (INREG32(&(g_pDmaController->CHNL_CTRL[lch].CSR)) &
		DMA_CSR_Is_Finished)) {
		/*Reschedule another thread (This is an active polling) */
		if (need_resched())
			schedule();
	}
}

/*-------------------------------------------------------------------------- */

void scxPublicDMAWait(void)
{
	wait_event(g_dmaWaitQueue, (g_dmaEventFlag != 0));
}

/*-------------------------------------------------------------------------- */
/*
 *perform common DMA channel setup, used to factorize the code
 *
 *output: SCX_DMA_CHANNEL_PARAM *pDMAChannel
 *Inputs: u32 nbBlocks    Number of block of the transfer
 *        u32 nbElements  Number of elements of the transfer
 *        u32 nDstStart   Destination address
 *        u32 nSrcStart   Source address
 *        u32 nTriggerID  Trigger ID
 *
 */
void scxPublicSetDMAChannelCommonParams(SCX_DMA_CHANNEL_PARAM *pDMAChannel,
					u32 nbBlocks,
					u32 nbElements,
					u32 nDstStart,
					u32 nSrcStart, u32 nTriggerID)
{
	pDMAChannel->data_type = DMA_CSDP_Srce_Endian_little |
		DMA_CSDP_Srce_Endian_Lock_off |
		DMA_CSDP_Dest_Endian_little |
		DMA_CSDP_Dest_Endian_Lock_off |
		DMA_CSDP_Write_Mode_none_posted |
		DMA_CSDP_Dest_Burst_off |
		DMA_CSDP_Dest_packed_off |
		DMA_CSDP_WR_Add_Trslt |
		DMA_CSDP_Src_Burst_off |
		DMA_CSDP_Src_packed_off |
		DMA_CSDP_RD_Add_Trslt |
		DMA_CSDP_Data_32b;
	pDMAChannel->elem_count = nbElements;
	pDMAChannel->frame_count = nbBlocks;
	pDMAChannel->src_ei = DMA_CSEI_Default;
	pDMAChannel->src_fi = DMA_CSFI_Default;
	pDMAChannel->dst_ei = DMA_CDEI_Default;
	pDMAChannel->dst_fi = DMA_CDFI_Default;
	pDMAChannel->sync_mode = 0x2;	/*FS =1, BS=0 => An entire frame is
				transferred once a DMA request is made */
	pDMAChannel->src_start = nSrcStart;
	pDMAChannel->dst_start = nDstStart;
	pDMAChannel->trigger = DMA_CCR_Mask_Channel(nTriggerID);
}
