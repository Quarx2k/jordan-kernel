/*
 * Copyright (c) 2006-2008 Trusted Logic S.A.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

.text

#define SMICODEPUB_IRQ_END   0xFE
#define SMICODEPUB_FIQ_END   0xFD
#define SMICODEPUB_RPC_END   0xFC

pub2sec_bridge_entry:
   .global pub2sec_bridge_entry

      PUSH     {R4-R12, LR}
      // Copy the Secure Service ID in R12
      MOV      R6, #0xFF
      MOV      R12, R0   // not needed on ES_2_0

      MCR      p15, 0, R0, c7, c5, 4   // Prefetch Buffer flush
      MCR      p15, 0, R0, c7, c10, 4  // Data Synchro Barrier

      SMC      1
      B        service_end
      NOP
      BL       v7_flush_kern_cache_all
      MOV      R12, #SMICODEPUB_IRQ_END
      SMC      1

service_end:
      POP      {R4-R12, LR}
      BX       LR


//rpc_handler:
 //   .global rpc_handler
  // CPSIE i   //enable IRQs
     // BL       SCXLNXSMCommRPCHandler
     // MOV      R12, #SMICODEPUB_RPC_END
     //` SMC      1


//----------------------------------------------------------------------------
// The following functions have been extracted from the kernel in:
//    > arch\arm\mm\cache-v7.S
//----------------------------------------------------------------------------

//
// v7_flush_dcache_all()
//
// Flush the whole D-cache.
//
// Corrupted registers: r0-r5, r7, r9-r11
//
// - mm    - mm_struct describing address space
//
v7_flush_dcache_all:
   .global v7_flush_dcache_all
   mrc   p15, 1, r0, c0, c0, 1   // read clidr
   ands  r3, r0, #0x7000000      // extract loc from clidr
   mov   r3, r3, lsr #23         // left align loc bit field
   beq   finished                // if loc is 0, then no need to clean
   mov   r10, #0                 // start clean at cache level 0
loop1:
   add   r2, r10, r10, lsr #1    // work out 3x current cache level
   mov   r1, r0, lsr r2          // extract cache type bits from clidr
   and   r1, r1, #7              // mask of the bits for current cache only
   cmp   r1, #2                  // see what cache we have at this level
   blt   skip                    // skip if no cache, or just i-cache
   mcr   p15, 2, r10, c0, c0, 0  // select current cache level in cssr
   isb                           // isb to sych the new cssr&csidr
   mrc   p15, 1, r1, c0, c0, 0   // read the new csidr
   and   r2, r1, #7              // extract the length of the cache lines
   add   r2, r2, #4              // add 4 (line length offset)
   ldr   r4, =0x3ff
   ands  r4, r4, r1, lsr #3      // find maximum number on the way size
   clz   r5, r4                  // find bit position of way size increment
   ldr   r7, =0x7fff
   ands  r7, r7, r1, lsr #13     // extract max number of the index size
loop2:
   mov   r9, r4                  // create working copy of max way size
loop3:
   orr   r11, r10, r9, lsl r5    // factor way and cache number into r11
   orr   r11, r11, r7, lsl r2    // factor index number into r11
   mcr   p15, 0, r11, c7, c14, 2 // clean & invalidate by set/way
   subs  r9, r9, #1              // decrement the way
   bge   loop3
   subs  r7, r7, #1              // decrement the index
   bge   loop2
skip:
   add   r10, r10, #2            // increment cache number
   cmp   r3, r10
   bgt   loop1
finished:
   mov   r10, #0                 // swith back to cache level 0
   mcr   p15, 2, r10, c0, c0, 0  // select current cache level in cssr
   isb
   mov   pc, lr

// v7_flush_cache_all()
//
// Flush the entire cache system.
//  The data cache flush is now achieved using atomic clean / invalidates
//  working outwards from L1 cache. This is done using Set/Way based cache
//  maintainance instructions.
//  The instruction cache can still be invalidated back to the point of
//  unification in a single instruction.
//
v7_flush_kern_cache_all:
   .global v7_flush_kern_cache_all
   stmfd sp!, {r4-r5, r7, r9-r11, lr}
   bl v7_flush_dcache_all
   mov   r0, #0
   mcr   p15, 0, r0, c7, c5, 0         // I+BTB cache invalidate
   ldmfd sp!, {r4-r5, r7, r9-r11, lr}
   mov   pc, lr


/*
 * cache_line_size - get the cache line size from the CSIDR register
 * (available on ARMv7+). It assumes that the CSSR register was configured
 * to access the L1 data cache CSIDR.
 */
   .macro   dcache_line_size, reg, tmp
   mrc   p15, 1, \tmp, c0, c0, 0    @ read CSIDR
   and   \tmp, \tmp, #7       @ cache line size encoding
   mov   \reg, #16         @ size offset
   mov   \reg, \reg, lsl \tmp    @ actual cache line size
   .endm


/*
 * v7_dma_flush_range(start,end)
 * - start   - virtual start address of region
 * - end     - virtual end address of region
 */
v7_dma_flush_range:
   .global v7_dma_flush_range
   dcache_line_size r2, r3
   sub   r3, r2, #1
   bic   r0, r0, r3
1:
   mcr   p15, 0, r0, c7, c14, 1     @ clean & invalidate D / U line
   add   r0, r0, r2
   cmp   r0, r1
   blo   1b
   dsb
   mov   pc, lr


/*
 * v7_dma_inv_range(start,end)
 *
 * Invalidate the data cache within the specified region; we will
 * be performing a DMA operation in this region and we want to
 * purge old data in the cache.
 *
 * - start   - virtual start address of region
 * - end     - virtual end address of region
 */
v7_dma_inv_range:
   .global v7_dma_inv_range
   dcache_line_size r2, r3
   sub   r3, r2, #1
   tst   r0, r3
   bic   r0, r0, r3
   mcrne p15, 0, r0, c7, c14, 1     @ clean & invalidate D / U line

   tst   r1, r3
   bic   r1, r1, r3
   mcrne p15, 0, r1, c7, c14, 1     @ clean & invalidate D / U line
1:
   mcr   p15, 0, r0, c7, c6, 1      @ invalidate D / U line
   add   r0, r0, r2
   cmp   r0, r1
   blo   1b
   dsb
   mov   pc, lr
