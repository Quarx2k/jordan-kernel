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

#include<linux/kernel.h>
#include<linux/slab.h>
#include<linux/module.h>
#include<linux/mm.h>
#include<mf-runtime.h>

#define LOOKUP_CACHE_MASK_DFL 1023
#define LOOKUP_CACHE_SIZE_MAX 65536 /* Allows max CACHE_MASK 0xFFFF */
#define LOOKUP_CACHE_SHIFT_DFL 2

struct __mf_cache __mf_lookup_cache[LOOKUP_CACHE_SIZE_MAX];
EXPORT_SYMBOL(__mf_lookup_cache);

uintptr_t __mf_lc_mask = LOOKUP_CACHE_MASK_DFL;
EXPORT_SYMBOL(__mf_lc_mask);

unsigned char __mf_lc_shift = LOOKUP_CACHE_SHIFT_DFL;
EXPORT_SYMBOL(__mf_lc_shift);

static inline int verify_ptr(unsigned long ptr)
{
	if (ptr < PAGE_OFFSET ||
		(ptr > (unsigned long)high_memory && high_memory != 0))
		return -EFAULT;

	return 0;
}

void __mf_check(void *ptr, unsigned int sz, int type, const char *location)
{
	if (!slab_is_available())
		return;
	if (verify_ptr((unsigned long)ptr))
		return;
	if (type) /* write */
		slab_check_write(ptr, sz, location);
}
EXPORT_SYMBOL(__mf_check);

void *kmudflap_memset(void *s, int c, size_t count)
{
	__mf_check(s, count, __MF_CHECK_WRITE, "memset dest");
	return memset(s, c, count);
}
EXPORT_SYMBOL(kmudflap_memset);

void *kmudflap_memcpy(void *dest, const void *src, size_t count)
{
	__mf_check(dest, count, __MF_CHECK_WRITE, "memcpy dest");
	return memcpy(dest, src, count);
}
EXPORT_SYMBOL(kmudflap_memcpy);

char *kmudflap_strcpy(char *dest, const char *src)
{
	size_t n = strlen(src);
	 __mf_check(dest, n, __MF_CHECK_WRITE, "strcpy dest");
	return strcpy(dest, src);
}
EXPORT_SYMBOL(kmudflap_strcpy);

void *kmudflap_strncpy(char *dest, const char *src, size_t count)
{
	size_t len = strnlen(src, count);
	 __mf_check(dest, len, __MF_CHECK_WRITE, "strncpy dest");
	return strncpy(dest, src, count);
}
EXPORT_SYMBOL(kmudflap_strncpy);

void
__mf_register(void *ptr, size_t sz, int type, const char *name)
{
}
EXPORT_SYMBOL(__mf_register);

void
__mf_unregister(void *ptr, size_t sz, int type)
{
}
EXPORT_SYMBOL(__mf_unregister);

void __mf_init(void)
{
}
EXPORT_SYMBOL(__mf_init);

int
__mf_set_options(const char *optstr)
{
	return 0;
}
EXPORT_SYMBOL(__mf_set_options);
