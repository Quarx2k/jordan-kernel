/******************************************************************************
 * NetMUX debug.c                                                             *
 *                                                                            *
 * Copyright (C) 2006-2010 Motorola, Inc.                                     *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions are     *
 * met:                                                                       *
 *                                                                            *
 * o Redistributions of source code must retain the above copyright notice,   *
 *   this list of conditions and the following disclaimer.                    *
 * o Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * o Neither the name of Motorola nor the names of its contributors may be    *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS    *
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,  *
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR     *
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR           *
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,      *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,        *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR         *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF     *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING       *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS         *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.               *
 *                                                                            *
 ******************************************************************************/
/*   DATE        OWNER       COMMENT                                          *
 *   ----------  ----------  -----------------------------------------------  *
 *   2006/09/28  Motorola    Initial version                                  *
 *   2007/05/01  Motorola    Change codes to ensure "shared" netmux           *
 *                           code is identical between AP and BP.             *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* debug.c provides functions that only get used when logging or debugging    */
/* are turned on.  The defined functions output data via the provided output  */
/* function declared in utility.*.                                            */

#include "debug.h"


/*
 * debug_error provides a mechanism to record the result of an error.
 * This function is generally called at the return of a function to
 * assist in debugging any function failures.
 *
 * Params:
 * function -- the name of the function the error occurred in
 * file -- the file the function resides in
 * line -- the line number the error was reported on
 * code -- the error code to be reported
 *
 * Returns:
 * int32 -- the same value as code
 */
int32 debug_error(sint8 *function, sint8 *file, int32 line, int32 code)
{
	switch (code) {
	case ERROR_NONE:
		{
			DEBUG
			("%s returned SUCCESS [%lu] at %s:%lu\n",
			function, code, file, line);
		}
		break;

	case ERROR_INVALIDPARAMETER:
		{
			DEBUG
			("%s returned ERROR_INVALIDPARAMETER [%lu] at %s:%lu\n",
			function, code, file, line);
		}
		break;

	case ERROR_MEMORY:
		{
			DEBUG
			("%s returned ERROR_MEMORY [%lu] at %s:%lu\n",
			function, code, file, line);
		}
		break;

	case ERROR_OPERATIONFAILED:
		{
			DEBUG
			("%s returned ERROR_OPERATIONFAILED [%lu] at %s:%lu\n",
			function, code, file, line);
		}
		break;

	case ERROR_OPERATIONRESTRICTED:
		{
			DEBUG
			("%s returned ERROR_OPERATIONRESTRICTED  \
			[%lu] at %s:%lu\n", function, code, file, line);
		}
		break;

	case ERROR_INCOMPLETE:
		{
			DEBUG
			("%s returned ERROR_INCOMPLETE [%lu] at %s:%lu\n",
			function, code, file, line);
		}
		break;

	default:
		{
			DEBUG
			("%s returned UNKNOWN ERROR [%lu] at %s:%lu\n",
			function, code, file, line);
		}
		break;
	}

	return code;
}

/*
 * debug_ldp provides a mechanism to record the result of an error in
 * the linkdriver protocol. This function is generally called at the
 * return of a function to assist in debugging any function failures.
 *
 * Params:
 * function -- the name of the function the error occurred in
 * file -- the file the function resides in
 * line -- the line number the error was reported on
 * code -- the error code to be reported
 *
 * Returns:
 * int32 -- the same value as code
 */
int32 debug_ldp(sint8 *function, sint8 *file, int32 line, int32 code)
{
	switch (code) {
	case LDP_ERROR_NONE:
		{
			DEBUG
			("%s returned SUCCESS [%lu] at %s:%lu\n",
			function, code, file, line);
		}
		break;

	case LDP_ERROR_FATAL:
		{
			DEBUG
			("%s returned LDP_ERROR_FATAL [%lu] at %s:%lu\n",
			function, code, file, line);
		}
		break;

	case LDP_ERROR_RECOVERABLE:
		{
			DEBUG
			("%s returned LDP_ERROR_RECOVERABLE [%lu] at %s:%lu\n",
			function, code, file, line);
		}
		break;

	default:
		{
			DEBUG
			("%s returned UNKNOWN ERROR [%lu] at %s:%lu\n",
			function, code, file, line);
		}
		break;
	}

	return code;
}
