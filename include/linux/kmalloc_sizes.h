/* CONFIG_DEBUG_SLAB adds overhead to each object in a slab.
 * The allocation sizes below take the overhead into account to
 * ensure that each slab uses a single page.
 */
#ifdef CONFIG_DEBUG_SLAB_KMALLOC_SINGLE_PAGE
#define dbg_slab(X) (((X/L1_CACHE_BYTES)*L1_CACHE_BYTES) \
		    - L1_CACHE_BYTES - 2 * sizeof(unsigned long long))
#define dbg_1024 dbg_slab(1024)
#define dbg_1365 dbg_slab(1365)
#define dbg_2048 dbg_slab(2048)
#endif

#if (PAGE_SIZE == 4096)
	CACHE(32)
#endif
	CACHE(64)
#if L1_CACHE_BYTES < 64
	CACHE(96)
#endif
	CACHE(128)
#if L1_CACHE_BYTES < 128
	CACHE(192)
#endif
	CACHE(256)
	CACHE(512)
#ifdef CONFIG_DEBUG_SLAB_KMALLOC_SINGLE_PAGE
	CACHE(dbg_1024)
	CACHE(dbg_1365)
	CACHE(dbg_2048)
#else
	CACHE(1024)
	CACHE(2048)
#endif
	CACHE(4096)
	CACHE(8192)
	CACHE(16384)
	CACHE(32768)
	CACHE(65536)
	CACHE(131072)
#if KMALLOC_MAX_SIZE >= 262144
	CACHE(262144)
#endif
#if KMALLOC_MAX_SIZE >= 524288
	CACHE(524288)
#endif
#if KMALLOC_MAX_SIZE >= 1048576
	CACHE(1048576)
#endif
#if KMALLOC_MAX_SIZE >= 2097152
	CACHE(2097152)
#endif
#if KMALLOC_MAX_SIZE >= 4194304
	CACHE(4194304)
#endif
#if KMALLOC_MAX_SIZE >= 8388608
	CACHE(8388608)
#endif
#if KMALLOC_MAX_SIZE >= 16777216
	CACHE(16777216)
#endif
#if KMALLOC_MAX_SIZE >= 33554432
	CACHE(33554432)
#endif

#ifdef CONFIG_DEBUG_SLAB_KMALLOC_SINGLE_PAGE
#undef dbg_slab
#undef dbg_1024
#undef dbg_1365
#undef dbg_2048
#endif
