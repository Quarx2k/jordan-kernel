/******************************************************************************
 * NetMUX utility.c                                                           *
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
 *   2006/12/19  Motorola    Combined header and data into one transfer       *
 *   2007/02/25  Motorola    Renamed kfree to kfree_skb inside commbuff_merge *
 *   2007/05/01  Motorola    Change codes to ensure "shared" netmux           *
 *                           code is identical between AP and BP.             *
 *   2007/12/05  Motorola    Change codes as INIT_WORK changes in kernel      *
 *   2008/04/10  Motorola    Add AP Debug log re-work                         *
 *   2010/04/28  Motorola    Format cleanup                                   *
 ******************************************************************************/


/* utility.c is responsible for defining a platform independent API to the    */
/* NetMUX.  The API is responsible for all common OS functions such as memory */
/* allocation and task scheduling.                                            */

#include "utility.h"
#include "packet.h"
#include "debug.h"


#define COMMBUFF_TAG_INDEX_COUNT         4
#define GET_COMMBUFF_TAG_INDEX(commbuff)  \
		(((int32)(commbuff)>>5)&(COMMBUFF_TAG_INDEX_COUNT-1))

#define LOG_COMMAND_ALL_WORK 49
#define LOG_COMMAND_BUFFER 50
#define LOG_COMMAND_FUNCTION 51

typedef struct LOCALTASK {
	TASKDATA releasetask;
	void *released_tagged_commbuffs;
} LOCALTASK;

static COMMBUFFTAG *commbuff_tags[COMMBUFF_TAG_INDEX_COUNT] = { 0 };

static COMMBUFFTAG *released_tagged_commbuffs = 0;

static LOCALTASK tagged_commbuff_release_task;

void ProcessReleasedCommBuffs(struct work_struct *);

extern char *NetmuxLogState;

void initialize_utilities(void)
{
	tagged_commbuff_release_task.released_tagged_commbuffs =
	    (void *) (&released_tagged_commbuffs);
	initialize_task(&tagged_commbuff_release_task.releasetask,
			&ProcessReleasedCommBuffs);
}

void shutdown_utilities(void)
{
	destroy_task(&tagged_commbuff_release_task.releasetask);
}

/*
 * int_alloc_mem allocates memory inside of an interrupt context
 * size is the size of the memory to allocate
 */
void *int_alloc_mem(int32 size)
{
	void *ret;

	ret = kmalloc(size, GFP_ATOMIC);
	if (!ret)
		PANIC(netmuxPanicMemFail1,
		      "NetMUX PANIC: Unable to allocate memory\n");

	return ret;
}

/*
 * alloc_mem allocates memory
 * size is the size of the memory to allocate
 */
void *alloc_mem(int32 size)
{
	void *ret;

	ret = kmalloc(size, GFP_KERNEL);
	if (!ret)
		PANIC(netmuxPanicMemFail2,
		      "NetMUX PANIC: Unable to allocate memory\n");

	return ret;
}

void ProcessReleasedCommBuffs(struct work_struct *work)
{
	INTERRUPT_STATE state;
	COMMBUFFTAG *taginfo;
	COMMBUFFTAG *freetag;
	LOCALTASK *localtask;

	disable_interrupts(state);

	/* grab a local copy of released_tagged_commbuffs */
	/* and clear the global list                      */
	localtask = container_of(work, LOCALTASK, releasetask.work);
	taginfo = *((COMMBUFFTAG **) localtask->released_tagged_commbuffs);
	*((COMMBUFFTAG **) localtask->released_tagged_commbuffs) = NULL;

	enable_interrupts(state);

	/* then process the local list */
	while (taginfo) {
		taginfo->CommBuffRelease(taginfo->channel,
					 taginfo->size, taginfo->param);

		freetag = taginfo;
		taginfo = taginfo->next_tag;

		free_mem(freetag);
	}
}

void FreeCommBuffData(COMMBUFF *commbuff)
{
	INTERRUPT_STATE state;
	COMMBUFFTAG **tags;
	COMMBUFFTAG *freetag;
	int32 index;

	index = GET_COMMBUFF_TAG_INDEX(commbuff);
	tags = &commbuff_tags[index];

	disable_interrupts(state);

	while (*tags) {
		if ((*tags)->commbuff == commbuff) {
			freetag = *tags;
			*tags = freetag->next_tag;

			freetag->next_tag = released_tagged_commbuffs;
			released_tagged_commbuffs = freetag;

			enable_interrupts(state);

			task_schedule(&tagged_commbuff_release_task.
				      releasetask);

			return;
		}

		tags = &(*tags)->next_tag;
	}

	enable_interrupts(state);
}

/*
 * int_alloc_commbuff allocates a commbuff inside of an interrupt context
 * size is the size of the commbuff to allocate
 */
COMMBUFF *int_alloc_commbuff(int32 size)
{
	COMMBUFF *newcommbuff;

	newcommbuff = alloc_skb(size, GFP_ATOMIC);
	if (!newcommbuff)
		PANIC(netmuxPanicSKBFail1,
		      "NetMUX PANIC: Unable to allocate commbuff\n");

	skb_put(newcommbuff, size);

	return newcommbuff;
}

/*
 * alloc_commbuff allocates a commbuff outside of interrupt context
 * size is the data size needed in the commbuff
 * headroom is the size needed to add a header in front of the data
 */
COMMBUFF *alloc_commbuff(int32 size, int32 headroom)
{
	COMMBUFF *newcommbuff;
	int32 totalSize = size + headroom;

	newcommbuff = alloc_skb(totalSize, GFP_KERNEL);
	if (!newcommbuff)
		PANIC(netmuxPanicSKBFail2,
		      "NetMUX PANIC: Unable to allocate commbuff\n");

	skb_reserve(newcommbuff, headroom);

	skb_put(newcommbuff, size);

	return newcommbuff;
}


/*
 * commbuff_split takes a commbuff and splits it at an offset
 * orig is the commbuff to be split
 * off is the offset at which the split is to occur
 * split is the resultant 'end partition' of the split
 */
COMMBUFF *commbuff_split(COMMBUFF *orig, int32 off)
{
	COMMBUFF *splitbuff;

	if (!off || off >= orig->len)
		return 0;

	splitbuff = skb_clone(orig, GFP_ATOMIC);
	if (!splitbuff)
		PANIC(netmuxPanicSKBFail3,
		      "NetMUX PANIC: Unable to allocate commbuff\n");

	skb_trim(orig, off);
	skb_pull(splitbuff, off);

	return splitbuff;
}

/*
 * commbuff_merge merges two commbuffs together
 * focus is where source is being merged to
 * source is what is being merged into focus
 */
COMMBUFF *commbuff_merge(COMMBUFF *focus, COMMBUFF *source)
{
	COMMBUFF *newcommbuff;
	sint8 *data;

	if (skb_tailroom(focus) >= source->len) {
		data = skb_put(focus, source->len);

		memcpy(data, source->data, source->len);
		kfree_skb(source);

		return focus;
	}

	newcommbuff = alloc_skb(focus->len + source->len, GFP_ATOMIC);
	if (!newcommbuff)
		PANIC(netmuxPanicSKBFail4,
		      "NetMUX PANIC: Unable to allocate commbuff\n");

	data = skb_put(newcommbuff, focus->len);
	memcpy(data, focus->data, focus->len);

	data = skb_put(newcommbuff, source->len);
	memcpy(data, source->data, source->len);

	kfree_skb(focus);
	kfree_skb(source);

	return newcommbuff;
}

void tag_commbuff(COMMBUFF *commbuff, int32 channel, void *param,
		  void (*release) (int32, int32, void*))
{
	INTERRUPT_STATE state;
	COMMBUFFTAG *newtag;
	int32 index;

	index = GET_COMMBUFF_TAG_INDEX(commbuff);

	newtag = (COMMBUFFTAG *) int_alloc_mem(sizeof(COMMBUFFTAG));

	newtag->channel = channel;
	newtag->commbuff = commbuff;
	newtag->param = param;
	newtag->size = commbuff_length(commbuff);
	newtag->CommBuffRelease = release;

	commbuff->destructor = &FreeCommBuffData;

	disable_interrupts(state);

	newtag->next_tag = commbuff_tags[index];
	commbuff_tags[index] = newtag;

	enable_interrupts(state);
}

void detag_commbuff(COMMBUFF *commbuff)
{
	FreeCommBuffData(commbuff);

	commbuff->destructor = 0;
}

void queue_commbuff(COMMBUFF *commbuff, COMMBUFFQUEUE *queue)
{
	INTERRUPT_STATE state;

	disable_interrupts(state);
	skb_queue_tail(queue, commbuff);
	enable_interrupts(state);
}

void queuefront_commbuff(COMMBUFF *commbuff, COMMBUFFQUEUE *queue)
{
	INTERRUPT_STATE state;

	disable_interrupts(state);
	skb_queue_head(queue, commbuff);
	enable_interrupts(state);
}

COMMBUFF *dequeue_commbuff(COMMBUFFQUEUE *queue)
{
	INTERRUPT_STATE state;
	COMMBUFF *ret;

	disable_interrupts(state);
	ret = skb_dequeue(queue);
	enable_interrupts(state);

	return ret;
}

void empty_commbuff_queue(COMMBUFFQUEUE *queue)
{
	INTERRUPT_STATE state;

	disable_interrupts(state);
	skb_queue_purge(queue);
	enable_interrupts(state);
}

void output(SU_PORT_HANDLE port, int32 msgid, sint8 *fmt, ...)
{
#if defined(_DEBUG_) || defined(_LOG_)
	va_list args;
	va_start(args, fmt);
	vprintk(fmt, args);
	va_end(args);
#endif				/* _DEBUG_ || _LOG_ */
	if (NetmuxLogState) {
		if ((NetmuxLogState[0] == LOG_COMMAND_ALL_WORK)
		    || (NetmuxLogState[0] == LOG_COMMAND_FUNCTION)) {
			va_list args;
			va_start(args, fmt);
			vprintk(fmt, args);
			va_end(args);
		}
	}
}

/*
 * log_commbuff logs all important data from a commbuff. This includes
 * the contents of the commbuff, the address of the commbuff, and the
 * length of the content.
 *
 * Params:
 * msgid -- A 4 byte message id, used for logging purposes
 * header -- A prefix to output prior to logging of the commbuff
 * cb -- a pointer to the commbuff to be logged
 * length -- the length of the commbuff
 */
void log_commbuff(int32 msgid, sint8 *header, COMMBUFF *cb, int32 length)
{
#ifdef _LOG_
	static sint8 text[256];
	int32 index;
	int32 value;

	value = 0;

	sprintf(text, "%s: Commbuff: 0x%p   Length: %lu   Data:", header,
		cb, length);
	printk(text);

	for (index = 0; index < length; index++) {
		if (!(index & 0x0F))
			printk("\nNETMUX: ");

		commbuff_copyout(&value, cb, index, sizeof(int8));

		sprintf(text, " 0x%02x", (unsigned int) value);
		printk(text);
	}

	printk("\n");
#endif				/* _LOG_ */
	if ((NetmuxLogState)
	    && ((NetmuxLogState[0] == LOG_COMMAND_ALL_WORK)
		|| (NetmuxLogState[0] == LOG_COMMAND_BUFFER))) {
		static sint8 text[256];
		int32 index;
		int32 value;

		value = 0;

		sprintf(text, "%s: Commbuff: 0x%p   Length: %lu   Data:",
			header, cb, length);
		printk(text);

		for (index = 0; index < length; index++) {
			if (!(index & 0x0F))
				printk("\nNETMUX: ");

			commbuff_copyout(&value, cb, index, sizeof(int8));

			sprintf(text, " 0x%02x", (unsigned int) value);
			printk(text);
		}

		printk("\n");
	}
}
