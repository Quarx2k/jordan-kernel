/*
 * drivers/media/video/hp3a/hp3a_common.h
 *
 * HP Imaging/3A library common functions and definitions.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co..
 *
 * Contributors:
 *		Tanvir Islam <tanvir.islam@hp.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef	__HP3A_QUEUE_H_INCLUDED
#define	__HP3A_QUEUE_H_INCLUDED

#define	QUEUE_SIZE(q)	(q.queue_size)
#define	QUEUE_COUNT(q)	(q.queue_count)

/**
 * struct hp3a_queue - Data structure for managing a queue.
 * @queue_lock: Lock synchronizaing thread/process access.
 * @write_index: Queue write index..
 * @read_index: Queue read index..
 * @queue_count: Element count in queue..
 * @queue_size:  Queue size.
 * @element_size: Element size.
 * @data: Pointer to the queue data.
 **/
struct hp3a_queue{
	spinlock_t queue_lock;
	int write_index;
	int read_index;
	int queue_count;
	int queue_size;
	unsigned int element_size;
	void *data;
};

/**
 * Queue functions.
 **/
int hp3a_initialize_queue(struct hp3a_queue *queue, int queue_size,
				unsigned int element_size);
void hp3a_deinitialize_queue(struct hp3a_queue *queue);
int hp3a_enqueue(struct hp3a_queue *queue, void *element);
int hp3a_enqueue_irqsave(struct hp3a_queue *queue, void *element);
int hp3a_dequeue(struct hp3a_queue *queue, void *element);
int hp3a_dequeue_irqsave(struct hp3a_queue *queue, void *element);
void hp3a_flush_queue(struct hp3a_queue *queue);
void hp3a_flush_queue_irqsave(struct hp3a_queue *queue);

#endif	/* __HP3A_QUEUE_H_INCLUDED */
