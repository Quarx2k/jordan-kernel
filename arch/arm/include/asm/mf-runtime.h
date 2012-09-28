#ifndef _LINUX_ARM_MF_RUNTIME
#define _LINUX_ARM_MF_RUNTIME
#ifdef _MUDFLAP
#pragma redefine_extname __memzero kmudflap_memzero
#endif /* _MUDFLAP */
#endif
