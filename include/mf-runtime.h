#ifndef _LINUX_MF_RUNTIME
#define _LINUX_MF_RUNTIME
/*
 * Copyright (C) 2009 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */

#include<linux/kernel.h>
#include<asm/mf-runtime.h>

/* Codes to describe the type of access to check */
#define __MF_CHECK_READ 0
#define __MF_CHECK_WRITE 1

typedef void *__mf_ptr_t;
typedef unsigned int __mf_uintptr_t;
struct __mf_cache { __mf_uintptr_t low; __mf_uintptr_t high; };

void __mf_check(void *ptr, unsigned int sz, int type, const char *location);
void __mf_register(void *ptr, size_t sz, int type, const char *name);
void __mf_init(void);
void __mf_unregister(void *ptr, size_t sz, int type);

#ifdef _MUDFLAP
#pragma redefine_extname memcpy kmudflap_memcpy
#pragma redefine_extname memset kmudflap_memset
#pragma redefine_extname strcpy kmudflap_strcpy
#pragma redefine_extname strncpy kmudflap_strncpy
#endif /* _MUDFLAP */

#endif
