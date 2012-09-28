/******************************************************************************
 * NetMUX utility_host.h                                                      *
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
 *   2007/04/11  Motorola    Modified commbuff_add_* to take header pointer   *
 *   2007/11/02  Motorola    Safely disable and enable interrupts             *
 *   2007/12/05  Motorola    Change initialize_task as INIT_WOKR changes in   *
 *                           kernel                                           *
 *   2008/10/25  Motorola    update  kernel to TI 25.1                        *
 *   2009/10/02  Motorola    replace down_interruptible() with down()         *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/


/* utility_host.h is responsible for defining a platform independent API to   */
/* the NetMUX. The API is responsible for all common OS functions such as     */
/* memory allocation and task scheduling.                                     */

#ifndef _NETMUX_UTILITY_HOST_H_
#define _NETMUX_UTILITY_HOST_H_


#include "types.h"
#include "errorcodes.h"
#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <asm/irq.h>


void initialize_utilities(void);
void shutdown_utilities(void);

/*
 * PANIC macro to halt everything
 */
#define PANIC(id, text) \
  {                                                 \
    char panicStr[128];                             \
    snprintf(panicStr, 128, "0x%x %s\n", id, text); \
    panic(panicStr);                                \
  }

/*
 * Macros to align data to 1 int8. These ease compiling the NetMUX with
 * different compilers
 */
#define START_PACKED_STRUCT(name) typedef struct name {
#define END_PACKED_STRUCT(name)   } __attribute__((packed)) name;
#define PACKED_MEMBER(member)     member

/*
 * Used to convert a value to little endian
 */
#define convert_word(value)  {}
#define convert_dword(value) {}

/*
 * Methods for allocating and freeing kernel space memory
 */
#define free_mem(ptr)       kfree(ptr)

void *int_alloc_mem(int32 size);
void *alloc_mem(int32 size);

/*
 * The NetMUX refers to a data type called a 'COMMBUFF' for transmitting and
 * receiving data
 */
typedef struct sk_buff COMMBUFF;

typedef struct COMMBUFFTAG {
	COMMBUFF *commbuff;

	int32 channel;
	int32 size;
	void *param;

	void (*CommBuffRelease) (int32, int32, void *);

	struct COMMBUFFTAG *next_tag;
} COMMBUFFTAG;

/*
 * The following declare routines for manipulating data inside a COMMBUFF
 */
COMMBUFF *int_alloc_commbuff(int32);
COMMBUFF *alloc_commbuff(int32, int32);

#define free_commbuff(ptr)                   kfree_skb(ptr)
#define ref_commbuff(ptr)                    skb_get(ptr)
#define deref_commbuff(ptr)                  kfree_skb(ptr)
#define commbuff_length(ptr)                 ((ptr)->len)
#define commbuff_data(ptr)                   ((ptr)->data)
#define commbuff_copyout(dst, cb, off, len)  memcpy(dst, ((cb)->data)+off, len)
#define commbuff_remove_front(ptr, amount)   skb_pull(ptr, amount)
/* the memcopys work because the skbuff is in contiguous memory */
#define commbuff_add_header(ptr, hdr, amount) \
	{ 			\
		skb_push(ptr, amount);\
		memcpy((ptr)->data, hdr, amount);\
	}
#define commbuff_copyin(cb, off, src, len)   memcpy((cb)->data+(off), src, len);
#define commbuff_copyin_byte(ptr, off, val)  (*((ptr)->data+off) = val)
#define commbuff_copyin_word(ptr, off, val)  (*(int16 *)((ptr)->data+off) = val)
#define commbuff_copyin_dword(ptr, off, val) (*(int32 *)((ptr)->data+off) = val)
#define commbuff_data_pullup(ptr, len)

COMMBUFF *commbuff_split(COMMBUFF *, int32);
COMMBUFF *commbuff_merge(COMMBUFF *, COMMBUFF *);
void tag_commbuff(COMMBUFF *, int32, void *,
		  void (*__cdecl) (int32, int32, void *));
void detag_commbuff(COMMBUFF *);

/*
 * The NetMUX uses a data type called a COMMBUFFQUEUE to store COMMBUFF's
 */
typedef struct sk_buff_head COMMBUFFQUEUE;

/*
 * The following are routines to manipulate COMMBUFFQUEUE's
 */
#define initialize_commbuff_queue(queue) skb_queue_head_init(queue)
#define destroy_commbuff_queue(queue)    {}
#define queue_length(queue)              skb_queue_len(queue)
#define queue_frontbuff(queue) (queue->next)

void queue_commbuff(COMMBUFF *, COMMBUFFQUEUE *);
void queuefront_commbuff(COMMBUFF *, COMMBUFFQUEUE *);
COMMBUFF *dequeue_commbuff(COMMBUFFQUEUE *);
void empty_commbuff_queue(COMMBUFFQUEUE *);

/*
 * TASKDATA defines a routine that can be manipulated like a Linux tasklet
 */
typedef struct TASKDATA {
	struct work_struct work;
	int8 state;
} TASKDATA;

#define initialize_task(task, function) { 				\
					unsigned long flags; 		\
					local_irq_save(flags); 		\
					INIT_WORK(&(task)->work, function);\
					(task)->state = 0; 		\
					local_irq_restore(flags); 	\
					}

#define destroy_task(task)        {}

#define enable_task(task)	{ 					\
					unsigned long flags;		\
					local_irq_save(flags); 		\
					(task)->state &= ~0x01; 	\
					if ((task)->state) { 		\
						schedule_work(&(task)->work);\
						(task)->state = 0;	\
					} 				\
					local_irq_restore(flags); 	\
				}

#define disable_task(task)	{ 					\
					unsigned long flags; 		\
					local_irq_save(flags); 		\
					(task)->state |= 0x01; 		\
					local_irq_restore(flags); 	\
				}

#define task_schedule(task)	{ 					\
					unsigned long flags; 		\
					local_irq_save(flags); 		\
					if (!(task)->state) 		\
						schedule_work(&(task)->work);\
					else 				\
						(task)->state |= 0x02; 	\
					local_irq_restore(flags); 	\
				}

#define global_tasklet() {}

/*
 * CRITICALSECTION defines a locking mechanism for critical sections of code
 */
typedef struct semaphore CRITICALSECTION;

/*
 * The following are routines to control a critical section
 */
#define initialize_criticalsection_lock(lock) init_MUTEX(lock)
#define destroy_criticalsection_lock(lock)    {}

#define enter_read_criticalsection(lock)      down(lock)

/* Keep this implementation for further investigation;
 *#define enter_read_criticalsection(lock)     if(down_interruptible(lock)){ \
 *                  panic("kernel error in netmux:down_interruptible\n");}
 */

#define exit_read_criticalsection(lock)       up(lock)

#define enter_write_criticalsection(lock)     down(lock)

/* Keep this implementation for further investigation;
 *#define enter_write_criticalsection(lock)     if(down_interruptible(lock)){ \
 *                     panic("kernel erro in netmux: down_interruptible\n");}
 */

#define exit_write_criticalsection(lock)      up(lock)

typedef unsigned long INTERRUPT_STATE;

#define disable_interrupts(state) local_irq_save(state)
#define enable_interrupts(state)  local_irq_restore(state)

#endif
