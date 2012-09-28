/******************************************************************************
 * NetMUX errorcodes.h                                                        *
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
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/

/* errorcodes.h defines a list of common return values that have a unique     */
/* mapping to a certain meaning. Generally, any function returning an int32   */
/* is returning one of the following errors.                                  */

#ifndef _NETMUX_ERRORCODES_H_
#define _NETMUX_ERRORCODES_H_


/*
 * ERROR_NONE specifies no error occured
 */
#define ERROR_NONE 0

/*
 * ERROR_INVALIDPARAMETER specifies a value to a function is not
 * valid
 */
#define ERROR_INVALIDPARAMETER 1

/*
 * ERROR_MEMORY states an error involving memory occured, usually
 * allocating it
 */
#define ERROR_MEMORY 2

/*
 * ERROR_OPERATIONFAILED occurs if a something went wrong within
 * the function
 */
#define ERROR_OPERATIONFAILED 3

 /*
  * ERROR_OPERATIONRESTRICTED occurs if the function couldn't
  * proceed
  */
#define ERROR_OPERATIONRESTRICTED 4

 /*
  * ERROR_INCOMPLETE occurs if the function needs to be called again
  */
#define ERROR_INCOMPLETE 5


#endif
