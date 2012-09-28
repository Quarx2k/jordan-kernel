/*
 * drivers/media/video/hp3a/hp3a_queue.c
 *
 * HP Imaging/3A Driver : hp3a queue functionality implementation.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
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
 *
 */

#include "hp3a_common.h"
#include "hp3a_queue.h"

/**
 * hp3a_initialize_queue - Initializes the quere for use.
 * @queue: Pointer to queue structure to be initialized.
 * @queue_size: size of queue.
 * @element_size: size of queue elements.
 *
 * No return value.
 **/
int hp3a_initialize_queue(struct hp3a_queue *queue,
		int queue_size, unsigned int element_size)
{
	if (queue_size > 0 && element_size > 0) {
		queue->data = kmalloc(queue_size * element_size,  GFP_KERNEL);
		if (queue->data != NULL) {
			queue->write_index = 0;
			queue->read_index = 0;
			queue->queue_count = 0;
			queue->queue_size = queue_size;
			queue->element_size = element_size;

			spin_lock_init(&queue->queue_lock);

			return 0;
		} else {
			printk(KERN_ERR "hp3a: Error allocating memory!\n");
		}
	}

	return -1;
}

/**
 * hp3a_deinitialize_queue - Deinitializes the quere.
 * @queue: Pointer to queue structure to be deinitialized.
 *
 * No return value.
 **/
void hp3a_deinitialize_queue(struct hp3a_queue *queue)
{
	spin_lock(&queue->queue_lock);

	if (queue->queue_size > 0) {
		queue->write_index = 0;
		queue->read_index = 0;
		queue->queue_count = 0;
		queue->queue_size = 0;

		kfree(queue->data);
		queue->data = NULL;
	}

	spin_unlock(&queue->queue_lock);
}

/**
 * hp3a_enqueue - Queues one element into the queue.
 * @queue: Pointer to queue structure to equeued to.
 * @element: Pointer to a element to be queued.
 *
 * Return 0 on success, -1 otherwise.
 **/
int hp3a_enqueue(struct hp3a_queue *queue, void *element)
{
	int ret = -1;

	spin_lock(&queue->queue_lock);

	if (queue->queue_count < queue->queue_size) {
		u32 offset =  queue->write_index * queue->element_size;
		void *item = (queue->data + offset);

		queue->queue_count += 1;
		queue->write_index += 1;
		queue->write_index = queue->write_index % queue->queue_size;

		/* copy data from element to queue. */
		memcpy(item, element, queue->element_size);
		ret = 0;
	}

	spin_unlock(&queue->queue_lock);

	return ret;
}

/**
 * hp3a_enqueue_irqsave - Queues one element into the queue.
 * @queue: Pointer to queue structure to equeued to.
 * @element: Pointer to a element to be queued.
 *
 * Return 0 on success, -1 otherwise.
 **/
int hp3a_enqueue_irqsave(struct hp3a_queue *queue, void *element)
{
	unsigned long irqflags = 0;
	int ret = -1;

	spin_lock_irqsave(&queue->queue_lock, irqflags);

	if (queue->queue_count < queue->queue_size) {
		u32 offset =  queue->write_index * queue->element_size;
		void *item = (queue->data + offset);

		queue->queue_count += 1;
		queue->write_index += 1;
		queue->write_index = queue->write_index % queue->queue_size;

		/* copy data from element to queue. */
		memcpy(item, element, queue->element_size);
		ret = 0;
	}

	spin_unlock_irqrestore(&queue->queue_lock, irqflags);

	return ret;
}

/**
 * hp3a_dequeue - Dequeues one element from the queue.
 * @queue: Pointer to queue structure to dequeued from.
 * @element: Pointer to a pointer of type element to copy
 *						dequeued element.
 *
 * Return 0 on success, -1 otherwise.
 **/
int hp3a_dequeue(struct hp3a_queue *queue, void *element)
{
	int ret = -1;

	spin_lock(&queue->queue_lock);

	if (queue->queue_count > 0) {
		u32 offset =  queue->read_index * queue->element_size;
		void *item = (queue->data + offset);

		queue->queue_count -= 1;
		queue->read_index += 1;
		queue->read_index = queue->read_index % queue->queue_size;

		/* copy data from queue to element. */
		memcpy(element, item, queue->element_size);
		ret = 0;
	}

	spin_unlock(&queue->queue_lock);

	return ret;
}

/**
 * hp3a_dequeue_irqsave - Dequeues one element from the queue.
 * @queue: Pointer to queue structure to dequeued from.
 * @element: Pointer to a pointer of type element to copy
 *						dequeued element.
 *
 * Return 0 on success, -1 otherwise.
 **/
int hp3a_dequeue_irqsave(struct hp3a_queue *queue, void *element)
{
	unsigned long irqflags = 0;
	int ret = -1;

	spin_lock_irqsave(&queue->queue_lock, irqflags);

	if (queue->queue_count > 0) {
		u32 offset =  queue->read_index * queue->element_size;
		void *item = (queue->data + offset);

		queue->queue_count -= 1;
		queue->read_index += 1;
		queue->read_index = queue->read_index % queue->queue_size;

		/* copy data from queue to element. */
		memcpy(element, item, queue->element_size);
		ret = 0;
	}

	spin_unlock_irqrestore(&queue->queue_lock, irqflags);

	return ret;
}

/**
 * hp3a_flush_queue - Flushes all elements from queue.
 * @queue: Pointer to queue structure to be flushed.
 *
 * No return value.
 **/
void hp3a_flush_queue(struct hp3a_queue *queue)
{
	spin_lock(&queue->queue_lock);

	if (queue->queue_count > 0) {
		queue->write_index = 0;
		queue->read_index = 0;
		queue->queue_count = 0;
	}

	spin_unlock(&queue->queue_lock);
}

/**
 * hp3a_flush_queue_irqsave - Flushes all elements from queue.
 * @queue: Pointer to queue structure to be flushed.
 *
 * No return value.
 **/
void hp3a_flush_queue_irqsave(struct hp3a_queue *queue)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&queue->queue_lock, irqflags);

	if (queue->queue_count > 0) {
		queue->write_index = 0;
		queue->read_index = 0;
		queue->queue_count = 0;
	}

	spin_unlock_irqrestore(&queue->queue_lock, irqflags);
}
