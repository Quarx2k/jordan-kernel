/*
 * reg.h
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


/*
 *  ======== reg.h ========
 *  Purpose:
 *      Provide registry functions.
 *
 *  Public Functions:
 *      REG_DeleteValue
 *      REG_EnumValue
 *      REG_Exit
 *      REG_GetValue
 *      REG_Init
 *      REG_SetValue
 *
 *! Revision History:
 *! =================
 *! 30-Oct-2000 kc: Updated REG_SetValue & REG_GetValue; renamed
 *!                 REG_DeleteEntry to REG_DeleteValue.
 *! 29-Sep-2000 kc: Updated a REG functions for code review.
 *! 12-Aug-2000 kc: Renamed REG_EnumValue to REG_EnumKey. Re-implemented
 *!                 REG_EnumValue.
 *! 03-Feb-2000 rr: REG_EnumValue Fxn Added
 *! 13-Dec-1999 rr: windows.h removed
 *! 02-Dec-1999 rr: windows.h included for retail build
 *! 22-Nov-1999 kc: Changes from code review.
 *! 29-Dec-1997 cr: Changes from code review.
 *! 27-Oct-1997 cr: Added REG_DeleteValue.
 *! 20-Oct-1997 cr: Added ability to pass bValue = NULL to REG_GetValue
 *!                 and return size of reg entry in pdwValueSize.
 *! 29-Sep-1997 cr: Added REG_SetValue
 *! 29-Aug-1997 cr: Created.
 */

#ifndef _REG_H
#define _REG_H

#include <linux/types.h>

#define REG_MAXREGPATHLENGTH    255

/*
 *  ======== REG_DeleteValue ========
 *  Purpose:
 *      Deletes a registry entry. NOTE: A registry entry is not the same as
 *      a registry key.
 *  Parameters:
 *      pstrValue:  Name of entry to delete.
 *  Returns:
 *      DSP_SOK:    Success.
 *      DSP_EFAIL:  General failure.
 *  Requires:
 *      - REG initialized.
 *      - pstrValue is non-NULL value.
 *      - length of pstrValue < REG_MAXREGPATHLENGTH.
 *  Ensures:
 *  Details:
 */
extern DSP_STATUS REG_DeleteValue(IN CONST char *pstrValue);

/*
 *  ======== REG_EnumValue ========
 *  Purpose:
 *      Enumerates values of a specified key. Retrieves each value name and
 *      the data associated with the value.
 *  Parameters:
 *      dwIndex:        Specifies the index of the value to retrieve.
 *      pstrKey:        The name of the registry key to be enumerated.
 *      pstrValue:      Pointer to buffer that receives the name of the value.
 *      pdwValueSize:   Specifies bytes of memory pstrValue points to on input,
 *                      On output, specifies actual memory bytes written into.
 *                      If there is no value, pdwValueSize returns NULL
 *      pstrData:       Pointer to buffer that receives the data of a value.
 *      pdwDataSize:    Specifies bytes of memory in pstrData on input and
 *                      bytes of memory written into pstrData on output.
 *                      If there is no data, pdwDataSize returns NULL.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      General failure.
 *  Requires:
 *      REG initialized.
 *      pstrKey is a non-NULL value.
 *      pstrValue, pstrData, pdwValueSize and pdwDataSize are valid pointers.
 *      Length of pstrKey is less than REG_MAXREGPATHLENGTH.
 *  Ensures:
 */
extern DSP_STATUS REG_EnumValue(IN u32 dwIndex, IN CONST char *pstrKey,
				IN OUT char *pstrValue,
				IN OUT u32 *pdwValueSize,
				IN OUT char *pstrData,
				IN OUT u32 *pdwDataSize);

/*
 *  ======== REG_Exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      REG initialized.
 *  Ensures:
 *      Resources used by module are freed when cRef reaches zero.
 */
	extern void REG_Exit(void);

/*
 *  ======== REG_GetValue ========
 *  Purpose:
 *      Retrieve a value from the registry.
 *  Parameters:
 *      pstrEntry:      Name of entry to retrieve.
 *      pbValue:        Upon return, points to retrieved value.
 *      pdwValueSize:   Specifies bytes of memory pbValue points to on input,
 *                      on output, specifies actual memory bytes written into.
 *                      If pbValue is NULL, pdwValueSize reports the size of
 *                      the entry in pstrEntry.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      General failure.
 *  Requires:
 *      - REG initialized.
 *      - pstrEntry is non-NULL value.
 *      - pbValue is a valid pointer.
 *      - length of pstrEntry < REG_MAXREGPATHLENGTH.
 *  Ensures:
 */
extern DSP_STATUS REG_GetValue(IN CONST char *pstrEntry, OUT u8 *pbValue,
				IN OUT u32 *pdwValueSize);

/*
 *  ======== REG_Init ========
 *  Purpose:
 *      Initializes private state of REG module.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occured.
 *  Requires:
 *  Ensures:
 *      REG initialized.
 */
	extern bool REG_Init(void);

/*
 *  ======== REG_SetValue ========
 *  Purpose:
 *      Set a value in the registry.
 *  Parameters:
 *      pstrEntry:      Name of entry to set.
 *      pbValue:        Points to buffer containing new data.
 *      dwValueSize:    Specifies bytes of memory bValue points to.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EFAIL:      General failure.
 *  Requires:
 *      - REG initialized.
 *      - pstrEntry is non-NULL value.
 *      - pbValue is a valid pointer.
 *      - dwValuSize > 0.
 *      - length of pstrEntry < REG_MAXREGPATHLENGTH.
 *  Ensures:
 */
extern DSP_STATUS REG_SetValue(IN CONST char *pstrEntry, IN u8 *pbValue,
				IN u32 dwValueSize);

#endif				/* _REG_H */
