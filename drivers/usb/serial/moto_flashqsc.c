/*
 * Motorola Modem flash mode driver
 *
 * Copyright (C) 2008 Greg Kroah-Hartman <greg@kroah.com>
 * Copyright (C) 2009 Motorola, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <mach/io.h>

static struct usb_device_id id_table[] = {
	{USB_DEVICE_AND_INTERFACE_INFO(0x22b8, 0x2a63, 0x0a, 0, 0)},
	{USB_DEVICE_AND_INTERFACE_INFO(0x22b8, 0x4281, 0x0a, 0, 0xfc)},
	{USB_DEVICE_AND_INTERFACE_INFO(0x22b8, 0x2db4, 0x0a, 0, 0xfc)},
	{USB_DEVICE(0x22b8, 0x4260)},
	{USB_DEVICE(0x22b8, 0x426D)},
	{},
};

MODULE_DEVICE_TABLE(usb, id_table);

#define MOTO_FLASHQSC_BULKOUT_SIZE	(1024*8)


static int moto_flashqsc_attach(struct usb_serial *serial)
{
	int j;
	struct usb_serial_port *port = serial->port[0];

	if (port->bulk_out_size >= MOTO_FLASHQSC_BULKOUT_SIZE) {
		dev_info(&serial->dev->dev,
			 "bulk_out_size %d\n", port->bulk_out_size);
		return 0;
	}

	kfree(port->bulk_out_buffer);
	port->bulk_out_size = MOTO_FLASHQSC_BULKOUT_SIZE;
	port->bulk_out_buffer = kmalloc(port->bulk_out_size, GFP_KERNEL);
	if (!port->bulk_out_buffer) {
		dev_err(&serial->dev->dev,
			"Couldn't allocate bulk_out_buffer\n");
		return -ENOMEM;
	}
	usb_fill_bulk_urb(port->write_urb, serial->dev,
			  usb_sndbulkpipe(serial->dev,
					  port->bulk_out_endpointAddress),
			  port->bulk_out_buffer, port->bulk_out_size,
			  usb_serial_generic_write_bulk_callback, port);


	for (j = 0; j < ARRAY_SIZE(port->write_urbs); ++j) {
		kfree(port->bulk_out_buffers[j]);
		port->bulk_out_buffers[j] = kmalloc(port->bulk_out_size,
						GFP_KERNEL);
		if (!port->bulk_out_buffers[j]) {
			dev_err(&serial->dev->dev,
				"Couldn't allocate bulk_out_buffer\n");
			return -ENOMEM;
		}

		usb_fill_bulk_urb(port->write_urbs[j], serial->dev,
			usb_sndbulkpipe(serial->dev,
					port->bulk_out_endpointAddress),
			port->bulk_out_buffers[j], port->bulk_out_size,
			usb_serial_generic_write_bulk_callback, port);
	}

	return 0;
}

static struct usb_driver moto_flashqsc_driver = {
	.name = "moto-flashqsc",
	.probe = usb_serial_probe,
	.disconnect = usb_serial_disconnect,
	.id_table = id_table,
	.no_dynamic_id = 1,

};

static struct usb_serial_driver moto_flashqsc_device = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "moto-flashqsc",
		   },
	.usb_driver = &moto_flashqsc_driver,
	.id_table = id_table,
	.num_ports = 1,
	.attach = moto_flashqsc_attach,
};

static int __init moto_flashqsc_init(void)
{
	int retval;

	retval = usb_serial_register(&moto_flashqsc_device);
	if (retval)
		return retval;
	retval = usb_register(&moto_flashqsc_driver);
	if (retval)
		usb_serial_deregister(&moto_flashqsc_device);
	return retval;
}

static void __exit moto_flashqsc_exit(void)
{
	usb_deregister(&moto_flashqsc_driver);
	usb_serial_deregister(&moto_flashqsc_device);
}

module_init(moto_flashqsc_init);
module_exit(moto_flashqsc_exit);
MODULE_LICENSE("GPL");
