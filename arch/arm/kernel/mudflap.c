/*
 * kernel/mudflap.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include<mf-runtime.h>
#include<asm/string.h>
#include<linux/module.h>

void kmudflap_memzero(void *ptr, __kernel_size_t n)
{
	__mf_check(ptr, n, __MF_CHECK_WRITE, "memzero dest");
	return __memzero(ptr, n);
}
EXPORT_SYMBOL(kmudflap_memzero);
