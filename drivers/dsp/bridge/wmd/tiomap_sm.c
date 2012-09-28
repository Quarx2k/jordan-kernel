/*
 * tiomap_sm.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

#include <dspbridge/cfg.h>
#include <dspbridge/drv.h>
#include <dspbridge/dev.h>

#include <dspbridge/dbg.h>

#include "_tiomap.h"
#include "_tiomap_pwr.h"

#define MAILBOX_FIFOSTATUS(m) (0x80 + 4 * (m))

static inline unsigned int fifo_full(void __iomem *mbox_base, int mbox_id)
{
	return __raw_readl(mbox_base + MAILBOX_FIFOSTATUS(mbox_id)) & 0x1;
}

DSP_STATUS CHNLSM_EnableInterrupt(struct WMD_DEV_CONTEXT *pDevContext)
{
	DSP_STATUS status = DSP_SOK;
	u32 numMbxMsg;
	u32 mbxValue;
	u32 devType;
	struct IO_MGR *hIOMgr;

	DBG_Trace(DBG_ENTER, "CHNLSM_EnableInterrupt(0x%x)\n", pDevContext);

	/* Read the messages in the mailbox until the message queue is empty */

	DEV_GetDevType(pDevContext->hDevObject, &devType);
	status = DEV_GetIOMgr(pDevContext->hDevObject, &hIOMgr);
	if (devType == DSP_UNIT) {
		HW_MBOX_NumMsgGet(pDevContext->dwMailBoxBase,
				  MBOX_DSP2ARM, &numMbxMsg);
		while (numMbxMsg != 0) {
			HW_MBOX_MsgRead(pDevContext->dwMailBoxBase,
					MBOX_DSP2ARM,
					&mbxValue);
			numMbxMsg--;
		}
		/* clear the DSP mailbox as well...*/
		HW_MBOX_NumMsgGet(pDevContext->dwMailBoxBase,
				  MBOX_ARM2DSP, &numMbxMsg);
		while (numMbxMsg != 0) {
			HW_MBOX_MsgRead(pDevContext->dwMailBoxBase,
					MBOX_ARM2DSP, &mbxValue);
			numMbxMsg--;
			udelay(10);

			HW_MBOX_EventAck(pDevContext->dwMailBoxBase, MBOX_ARM2DSP,
					 HW_MBOX_U1_DSP1,
					 HW_MBOX_INT_NEW_MSG);
		}
		/* Enable the new message events on this IRQ line */
		HW_MBOX_EventEnable(pDevContext->dwMailBoxBase,
				    MBOX_DSP2ARM,
				    MBOX_ARM,
				    HW_MBOX_INT_NEW_MSG);
	}

	return status;
}

DSP_STATUS CHNLSM_DisableInterrupt(struct WMD_DEV_CONTEXT *pDevContext)
{
	DBG_Trace(DBG_ENTER, "CHNLSM_DisableInterrupt(0x%x)\n", pDevContext);

	HW_MBOX_EventDisable(pDevContext->dwMailBoxBase, MBOX_DSP2ARM,
			     MBOX_ARM, HW_MBOX_INT_NEW_MSG);
	return DSP_SOK;
}

DSP_STATUS CHNLSM_InterruptDSP2(struct WMD_DEV_CONTEXT *pDevContext,
				u16 wMbVal)
{
	DSP_STATUS status = DSP_SOK;
	unsigned long timeout;
	u32 temp;
	u32 cnt = 0;
	bool time_expired = true;


	if (DSP_FAILED(status))
		return DSP_EFAIL;
#ifdef CONFIG_BRIDGE_DVFS
	if (pDevContext->dwBrdState == BRD_DSP_HIBERNATION ||
	    pDevContext->dwBrdState == BRD_HIBERNATION) {
		if (!in_atomic())
			tiomap3430_bump_dsp_opp_level();
	}
#endif

	if (pDevContext->dwBrdState == BRD_DSP_HIBERNATION ||
	    pDevContext->dwBrdState == BRD_HIBERNATION) {
		/* Restart the peripheral clocks */
		DSP_PeripheralClocks_Enable(pDevContext, NULL);

		/* Restore mailbox settings */
		/* Enabling Dpll in lock mode*/
		temp = (u32) *((REG_UWORD32 *)
				((u32) (pDevContext->cmbase) + 0x34));
		temp = (temp & 0xFFFFFFFE) | 0x1;
		*((REG_UWORD32 *) ((u32) (pDevContext->cmbase) + 0x34)) =
			(u32) temp;
		temp = (u32) *((REG_UWORD32 *)
				((u32) (pDevContext->cmbase) + 0x4));
		temp = (temp & 0xFFFFFC8) | 0x37;

		*((REG_UWORD32 *) ((u32) (pDevContext->cmbase) + 0x4)) =
			(u32) temp;
		HW_MBOX_restoreSettings(pDevContext->dwMailBoxBase);

		/*  Access MMU SYS CONFIG register to generate a short wakeup */
		temp = (u32) *((REG_UWORD32 *) ((u32)
						(pDevContext->dwDSPMmuBase) + 0x10));

		pDevContext->dwBrdState = BRD_RUNNING;
	} else if (pDevContext->dwBrdState == BRD_RETENTION)
		/* Restart the peripheral clocks */
		DSP_PeripheralClocks_Enable(pDevContext, NULL);

	/* Check the mailbox every 1 msec 10 times before giving up */
	for (cnt = 0; (cnt < 10) && (time_expired); cnt++) {
		timeout = jiffies + msecs_to_jiffies(1);
		time_expired = false;
		while (fifo_full((void __iomem *)pDevContext->dwMailBoxBase, 0)) {
			if (time_after(jiffies, timeout)) {
				time_expired = true;
				printk(KERN_WARNING "Mailbox full trying again count %d \n", cnt);
				break;
			}
		}
	}
	if ((cnt == 10) && (time_expired)) {
		printk(KERN_ERR "dspbridge: Mailbox was not empty after %d trials , no message written !!!\n", cnt);
		return WMD_E_TIMEOUT;
	}
	DBG_Trace(DBG_LEVEL3, "writing %x to Mailbox\n",
		  wMbVal);

	HW_MBOX_MsgWrite(pDevContext->dwMailBoxBase, MBOX_ARM2DSP,
			 wMbVal);
	return DSP_SOK;
}

bool CHNLSM_ISR(struct WMD_DEV_CONTEXT *pDevContext, bool *pfSchedDPC,
		u16 *pwIntrVal)
{
	u32 numMbxMsg;
	u32 mbxValue;

	DBG_Trace(DBG_ENTER, "CHNLSM_ISR(0x%x)\n", pDevContext);

	HW_MBOX_NumMsgGet(pDevContext->dwMailBoxBase, MBOX_DSP2ARM, &numMbxMsg);

	if (numMbxMsg > 0) {
		HW_MBOX_MsgRead(pDevContext->dwMailBoxBase, MBOX_DSP2ARM, &mbxValue);

		HW_MBOX_EventAck(pDevContext->dwMailBoxBase, MBOX_DSP2ARM,
				 HW_MBOX_U0_ARM, HW_MBOX_INT_NEW_MSG);

		DBG_Trace(DBG_LEVEL3, "Read %x from Mailbox\n", mbxValue);
		*pwIntrVal = (u16) mbxValue;
	}
	/* Set *pfSchedDPC to true; */
	*pfSchedDPC = true;
	return true;
}
