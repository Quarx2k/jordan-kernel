/*
 * viatel-cbp.c -- USB driver for CBP
 *
 * Copyright (C) 2009 Karfield Chen <kfchen@via-telecom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This driver exists because the "normal" serial driver doesn't work too well
 * with GSM modems. Issues:
 * - data loss -- one single Receive URB is not nearly enough
 * - nonstandard flow (Option devices) control
 * - controlling the baud rate doesn't make sense
 *
 * Some of the "one port" devices actually exhibit multiple USB instances
 * on the USB bus. This is not a bug, these ports are used for different
 * device features.
 */

/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#include <linux/usb/cdc.h>
#include <linux/bypass.h>

/* Function prototypes */
static int cbp_open(struct tty_struct *tty, struct usb_serial_port *port);
static void cbp_close(struct usb_serial_port *port);
static int cbp_startup(struct usb_serial *serial);
static void cbp_release(struct usb_serial *serial);
static int cbp_write_room(struct tty_struct *tty);

static void cbp_instat_callback(struct urb *urb);

static int cbp_write(struct tty_struct *tty, struct usb_serial_port *port,
		     const unsigned char *buf, int count);
static int cbp_chars_in_buffer(struct tty_struct *tty);
static void cbp_set_termios(struct tty_struct *tty,
			    struct usb_serial_port *port, struct ktermios *old);
static int cbp_tiocmget(struct tty_struct *tty, struct file *file);
static int cbp_tiocmset(struct tty_struct *tty, struct file *file,
			unsigned int set, unsigned int clear);
static int cbp_send_setup(struct usb_serial_port *port);
#ifdef CONFIG_PM
static int cbp_suspend(struct usb_serial *serial, pm_message_t message);
static int cbp_resume(struct usb_serial *serial);
#endif
static int usb_serial_cbp_probe(struct usb_interface *interface,
				const struct usb_device_id *id);
static void usb_serial_cbp_disconnect(struct usb_interface *interface);

static void cbp_disconnect(struct usb_serial *serial);

/* Vendor and product IDs */
/* VIA-Telecom */
#define VIATELECOM_VENDOR_ID			0x15eb
#define VIATELECOM_PRODUCT_ID			0x0001

/*The number of ttyUSB[X] port used to data call*/
#define VIA_DATA_CALL_NUM (0)

static struct usb_device_id cbp_ids[] = {
	{USB_DEVICE(VIATELECOM_VENDOR_ID, VIATELECOM_PRODUCT_ID)},
	{}			/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, cbp_ids);

static struct usb_driver cbp_driver = {
	.name = "via-cbp",
	.probe = usb_serial_cbp_probe,
	.disconnect = usb_serial_cbp_disconnect,
#ifdef CONFIG_PM
	.suspend = usb_serial_suspend,
	.resume = usb_serial_resume,
	.supports_autosuspend = 0,
#endif
	.id_table = cbp_ids,
	.no_dynamic_id = 1,
};

/* The card has three separate interfaces, which the serial driver
 * recognizes separately, thus num_port=1.
 */

static struct usb_serial_driver cbp_1port_device = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "cbp",
		   },
	.description = "VIA-Telecom AP/CP USB Connection",
	.usb_driver = &cbp_driver,
	.id_table = cbp_ids,
	.num_ports = 1,
	.open = cbp_open,
	.close = cbp_close,
	.write = cbp_write,
	.write_room = cbp_write_room,
	.chars_in_buffer = cbp_chars_in_buffer,
	.set_termios = cbp_set_termios,
	.tiocmget = cbp_tiocmget,
	.tiocmset = cbp_tiocmset,
	.attach = cbp_startup,
	.disconnect = cbp_disconnect,
	.release = cbp_release,
	.read_int_callback = cbp_instat_callback,
#ifdef CONFIG_PM
	.suspend = cbp_suspend,
	.resume = cbp_resume,
#endif
};

static int debug;

/* per port private data */

#define N_IN_URB 4
#define N_OUT_URB 4
#define IN_BUFLEN 4096
#define OUT_BUFLEN 4096

struct cbp_port_private {
	/* Input endpoints and buffer for this port */
	struct urb *in_urbs[N_IN_URB];
	u8 *in_buffer[N_IN_URB];
	/* Output endpoints and buffer for this port */
	struct urb *out_urbs[N_OUT_URB];
	u8 *out_buffer[N_OUT_URB];
	unsigned long out_busy;	/* Bit vector of URBs in use */

	/* Settings for the port */
	int rts_state;		/* Handshaking pins (outputs) */
	int dtr_state;
	int cts_state;		/* Handshaking pins (inputs) */
	int dsr_state;
	int dcd_state;
	int ri_state;

	unsigned long tx_start_time[N_OUT_URB];
};

static int bypass_write(int port_num, const unsigned char *buf, int count)
{
	struct bypass *bypass = bypass_get();
	struct usb_serial_port *port;

	if (bypass == NULL) {
		printk(KERN_ERR "%s - bypass_get error\n", __func__);
		return -1;
	}

	switch (port_num) {
	case 0:
		port = bypass->h_modem_port; break;
	case 1:
		port = bypass->h_ets_port; break;
	case 2:
		port = bypass->h_atc_port; break;
	case 3:
		port = bypass->h_pcv_port; break;
	case 4:
		port = bypass->h_gps_port; break;
	default:
		port = NULL;
	}

	if (port == NULL) {
		printk(KERN_ERR "can NOT find port (num %d)\n", port_num);
		return -EIO;
	}

	return port->serial->type->write(NULL, port, buf, count);
}

static struct bypass_ops h_write_ops = {
	.h_write = bypass_write,
};

/* Functions used by new usb-serial code. */
static int __init cbp_init(void)
{
	int retval;
	retval = usb_serial_register(&cbp_1port_device);
	if (retval)
		goto failed_1port_device_register;
	retval = usb_register(&cbp_driver);
	if (retval)
		goto failed_driver_register;

	bypass_register(&h_write_ops);

	return 0;

failed_driver_register:
	usb_serial_deregister(&cbp_1port_device);
failed_1port_device_register:
	return retval;
}

static void __exit cbp_exit(void)
{
	usb_deregister(&cbp_driver);
	usb_serial_deregister(&cbp_1port_device);
	bypass_unregister(1);
}

module_init(cbp_init);
module_exit(cbp_exit);

static void cbp_set_termios(struct tty_struct *tty,
			    struct usb_serial_port *port,
			    struct ktermios *old_termios)
{
	dbg("%s", __func__);
	/* Doesn't support option setting */
	tty_termios_copy_hw(tty->termios, old_termios);
	cbp_send_setup(port);
}

static int cbp_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct usb_serial_port *port = tty->driver_data;
	unsigned int value;
	struct cbp_port_private *portdata;

	portdata = usb_get_serial_port_data(port);

	value = ((portdata->rts_state) ? TIOCM_RTS : 0) |
	    ((portdata->dtr_state) ? TIOCM_DTR : 0) |
	    ((portdata->cts_state) ? TIOCM_CTS : 0) |
	    ((portdata->dsr_state) ? TIOCM_DSR : 0) |
	    ((portdata->dcd_state) ? TIOCM_CAR : 0) |
	    ((portdata->ri_state) ? TIOCM_RNG : 0);

	return value;
}

static int cbp_tiocmset(struct tty_struct *tty, struct file *file,
			unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;
	struct cbp_port_private *portdata;

	portdata = usb_get_serial_port_data(port);

	/* FIXME: what locks portdata fields ? */
	if (set & TIOCM_RTS)
		portdata->rts_state = 1;
	if (set & TIOCM_DTR)
		portdata->dtr_state = 1;

	if (clear & TIOCM_RTS)
		portdata->rts_state = 0;
	if (clear & TIOCM_DTR)
		portdata->dtr_state = 0;
	return cbp_send_setup(port);
}

/* Write */
static int cbp_write(struct tty_struct *tty, struct usb_serial_port *port,
		     const unsigned char *buf, int count)
{
	struct cbp_port_private *portdata;
	int i;
	int left, todo;
	struct urb *this_urb = NULL;	/* spurious */
	int err;
	struct usb_serial *serial = port->serial;

	portdata = usb_get_serial_port_data(port);

	dbg("%s: write port %d (%d chars)", __func__, port->number, count);

/*when debug auto pm, check the code again*/
/*
#if CONFIG_PM
	if ((serial->suspend_count)
	    && (VIA_DATA_CALL_NUM !=
		(serial->interface->cur_altsetting->desc).bInterfaceNumber)) {
		err = usb_autopm_get_interface(serial->interface);
		if (err) {
			printk(KERN_ERR "%s-autopm get error, ret=%d\n",
			       __func__, err);
			return 0;
		}
		usb_autopm_put_interface(port->serial->interface);
	} else {
		usb_mark_last_busy(interface_to_usbdev(serial->interface));
	}
#endif
*/
	i = 0;
	left = count;
	for (i = 0; left > 0 && i < N_OUT_URB; i++) {
		todo = left;
		if (todo > OUT_BUFLEN)
			todo = OUT_BUFLEN;

		this_urb = portdata->out_urbs[i];
		if (test_and_set_bit(i, &portdata->out_busy)) {
			if (time_before(jiffies,
					portdata->tx_start_time[i] + 10 * HZ))
				continue;
			usb_unlink_urb(this_urb);
			continue;
		}
		dbg("%s: endpoint %d buf %d", __func__,
		    usb_pipeendpoint(this_urb->pipe), i);

		/* send the data */
		memcpy(this_urb->transfer_buffer, buf, todo);
		this_urb->transfer_buffer_length = todo;

		this_urb->dev = port->serial->dev;
		err = usb_submit_urb(this_urb, GFP_ATOMIC);
		if (err) {
			dbg("usb_submit_urb %p (write bulk) failed "
			    "(%d)", this_urb, err);
			clear_bit(i, &portdata->out_busy);
			continue;
		}
		portdata->tx_start_time[i] = jiffies;
		buf += todo;
		left -= todo;
	}

	count -= left;
	dbg("%s: wrote (did %d)", __func__, count);
	return count;
}

static void cbp_indat_callback(struct urb *urb)
{
	int err;
	int endpoint;
	struct usb_serial_port *port;
	struct tty_struct *tty;
	unsigned char *data = urb->transfer_buffer;
	int status = urb->status;

	dbg("%s: %p", __func__, urb);

	endpoint = usb_pipeendpoint(urb->pipe);
	port = urb->context;

	if (status) {
		dbg("%s: nonzero status: %d on endpoint %02x.",
		    __func__, status, endpoint);
	} else {
		/* here we steal data to bypass but never push to tty */
		struct bypass *bypass = bypass_get();
		if ((bypass != NULL) && (urb->actual_length)) {
			if ((port->number == 0)
			    && (bypass->modem_status == 1)) {
				if (bypass->ops->g_modem_write) {
					bypass->ops->
					    g_modem_write(urb->transfer_buffer,
							  urb->actual_length);
				}
				goto RESUBMIT_URB;
			} else if (((port->number == 1)
				    && (bypass->ets_status == 1))
				   || ((port->number == 2)
				       && (bypass->at_status == 1))
				   || ((port->number == 3)
				       && (bypass->pcv_status == 1))
				   || ((port->number == 4)
				       && (bypass->gps_status == 1))) {
				if (bypass->ops->g_write) {
					bypass->ops->g_write(port->number,
							     urb->
								transfer_buffer,
							     urb->
								actual_length);
				}
				goto RESUBMIT_URB;
			}
		}
		tty = tty_port_tty_get(&port->port);
		if (urb->actual_length) {
			tty_buffer_request_room(tty, urb->actual_length);
			tty_insert_flip_string(tty, data, urb->actual_length);
			tty_flip_buffer_push(tty);
		} else
			dbg("%s: empty read urb received", __func__);
		tty_kref_put(tty);


RESUBMIT_URB:
		/* Resubmit urb so we continue receiving */
		if (port->port.count && status != -ESHUTDOWN) {
			/* keep read alive in default timeout(3*HZ) */
			usb_mark_last_busy(interface_to_usbdev
					   (port->serial->interface));

			err = usb_submit_urb(urb, GFP_ATOMIC);
			if (err)
				printk(KERN_ERR
				       "%s: resubmit read urb failed. "
				       "(%d)", __func__, err);
		}
	}
	return;
}

static void cbp_outdat_callback(struct urb *urb)
{
	struct usb_serial_port *port;
	struct cbp_port_private *portdata;
	int i;

	dbg("%s", __func__);

	port = urb->context;

	usb_serial_port_softint(port);

	portdata = usb_get_serial_port_data(port);
	for (i = 0; i < N_OUT_URB; ++i) {
		if (portdata->out_urbs[i] == urb) {
			smp_mb__before_clear_bit();
			clear_bit(i, &portdata->out_busy);
			break;
		}
	}
}

static void cbp_instat_callback(struct urb *urb)
{
	int err;
	int status = urb->status;
	struct usb_serial_port *port = urb->context;
	struct cbp_port_private *portdata = usb_get_serial_port_data(port);
	struct usb_serial *serial = port->serial;

	dbg("%s: urb %p port %p has data %p", __func__, urb, port, portdata);

	if (status == 0) {
		struct usb_ctrlrequest *req_pkt =
		    (struct usb_ctrlrequest *)urb->transfer_buffer;

		if (!req_pkt) {
			dbg("%s: NULL req_pkt\n", __func__);
			return;
		}
		if ((req_pkt->bRequestType == 0xA1) &&
		    (req_pkt->bRequest == 0x20)) {
			int old_dcd_state;
			unsigned char signals = *((unsigned char *)
						  urb->transfer_buffer +
						  sizeof(struct
							 usb_ctrlrequest));

			dbg("%s: signal x%x", __func__, signals);

			old_dcd_state = portdata->dcd_state;
			portdata->cts_state = 1;
			portdata->dcd_state = ((signals & 0x01) ? 1 : 0);
			portdata->dsr_state = ((signals & 0x02) ? 1 : 0);
			portdata->ri_state = ((signals & 0x08) ? 1 : 0);

			if (old_dcd_state && !portdata->dcd_state) {
				struct tty_struct *tty =
				    tty_port_tty_get(&port->port);
				if (tty && !C_CLOCAL(tty))
					tty_hangup(tty);
				tty_kref_put(tty);
			}
		} else {
			dbg("%s: type %x req %x", __func__,
			    req_pkt->bRequestType, req_pkt->bRequest);
		}
	} else
		err("%s: error %d", __func__, status);

	/* Resubmit urb so we continue receiving IRQ data */
	if (status != -ESHUTDOWN && status != -ENOENT) {
		urb->dev = serial->dev;
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err)
			dbg("%s: resubmit intr urb failed. (%d)",
			    __func__, err);
	}
}

static int cbp_write_room(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct cbp_port_private *portdata;
	int i;
	int data_len = 0;
	struct urb *this_urb;

	portdata = usb_get_serial_port_data(port);

	for (i = 0; i < N_OUT_URB; i++) {
		this_urb = portdata->out_urbs[i];
		if (this_urb && !test_bit(i, &portdata->out_busy))
			data_len += OUT_BUFLEN;
	}

	dbg("%s: %d", __func__, data_len);
	return data_len;
}

static int cbp_chars_in_buffer(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct cbp_port_private *portdata;
	int i;
	int data_len = 0;
	struct urb *this_urb;

	portdata = usb_get_serial_port_data(port);

	for (i = 0; i < N_OUT_URB; i++) {
		this_urb = portdata->out_urbs[i];
		/* FIXME: This locking is insufficient as this_urb may
		   go unused during the test */
		if (this_urb && test_bit(i, &portdata->out_busy))
			data_len += this_urb->transfer_buffer_length;
	}
	dbg("%s: %d", __func__, data_len);
	return data_len;
}

static int cbp_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct cbp_port_private *portdata;
	struct usb_serial *serial = port->serial;
	int i, err;
	struct urb *urb;

	portdata = usb_get_serial_port_data(port);

	/* Set some sane defaults */
	portdata->rts_state = 1;
	portdata->dtr_state = 1;

	/* Reset low level data toggle and start reading from endpoints */
	for (i = 0; i < N_IN_URB; i++) {
		urb = portdata->in_urbs[i];
		if (!urb)
			continue;
		if (urb->dev != serial->dev) {
			dbg("%s: dev %p != %p", __func__,
			    urb->dev, serial->dev);
			continue;
		}

		/*
		 * make sure endpoint data toggle is synchronized with the
		 * device
		 */
		usb_clear_halt(urb->dev, urb->pipe);

		err = usb_submit_urb(urb, GFP_KERNEL);
		if (err) {
			dbg("%s: submit urb %d failed (%d) %d",
			    __func__, i, err, urb->transfer_buffer_length);
		}
	}

	/* Reset low level data toggle on out endpoints */
	for (i = 0; i < N_OUT_URB; i++) {
		urb = portdata->out_urbs[i];
		if (!urb)
			continue;
		urb->dev = serial->dev;
		/* usb_settoggle(urb->dev, usb_pipeendpoint(urb->pipe),
		   usb_pipeout(urb->pipe), 0); */
	}

	cbp_send_setup(port);
/*
#if CONFIG_PM
	if (VIA_DATA_CALL_NUM ==
	    (serial->interface->cur_altsetting->desc).bInterfaceNumber) {
		usb_autopm_get_interface(serial->interface);
	}
#endif
*/

	return 0;
}

static void cbp_close(struct usb_serial_port *port)
{
	int i;
	struct usb_serial *serial = port->serial;
	struct cbp_port_private *portdata;

	struct bypass *bypass = bypass_get();
	if (bypass && bypass->ets_status)
		return;
	portdata = usb_get_serial_port_data(port);

	portdata->rts_state = 0;
	portdata->dtr_state = 0;

/* when debug auto pm, check the code again */
/*
#if CONFIG_PM
	if (VIA_DATA_CALL_NUM ==
	    (serial->interface->cur_altsetting->desc).bInterfaceNumber) {
		usb_autopm_put_interface(serial->interface);
	}
#endif
*/

	if (serial->dev) {
		mutex_lock(&serial->disc_mutex);
		if (!serial->disconnected) {
			cbp_send_setup(port);
			/* Stop reading/writing urbs */
			for (i = 0; i < N_IN_URB; i++)
				usb_kill_urb(portdata->in_urbs[i]);
			for (i = 0; i < N_OUT_URB; i++)
				usb_kill_urb(portdata->out_urbs[i]);
		}
		mutex_unlock(&serial->disc_mutex);
	}
	tty_port_tty_set(&port->port, NULL);

}

/* Helper functions used by cbp_setup_urbs */
static struct urb *cbp_setup_urb(struct usb_serial *serial, int endpoint,
				 int dir, void *ctx, char *buf, int len,
				 void (*callback) (struct urb *))
{
	struct urb *urb;

	if (endpoint == -1)
		return NULL;	/* endpoint not needed */

	urb = usb_alloc_urb(0, GFP_KERNEL);	/* No ISO */
	if (urb == NULL) {
		dbg("%s: alloc for endpoint %d failed.", __func__, endpoint);
		return NULL;
	}

	/* Fill URB using supplied data. */
	usb_fill_bulk_urb(urb, serial->dev,
			  usb_sndbulkpipe(serial->dev, endpoint) | dir,
			  buf, len, callback, ctx);

	return urb;
}

/* Setup urbs */
static void cbp_setup_urbs(struct usb_serial *serial)
{
	int i, j;
	struct usb_serial_port *port;
	struct cbp_port_private *portdata;

	dbg("%s", __func__);

	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
		portdata = usb_get_serial_port_data(port);

		/* Do indat endpoints first */
		for (j = 0; j < N_IN_URB; ++j) {
			portdata->in_urbs[j] = cbp_setup_urb
						(serial,
						 port->bulk_in_endpointAddress,
						 USB_DIR_IN,
						 port,
						 portdata->in_buffer
						 [j], IN_BUFLEN,
						 cbp_indat_callback);
		}

		/* outdat endpoints */
		for (j = 0; j < N_OUT_URB; ++j) {
			portdata->out_urbs[j] = cbp_setup_urb
						(serial,
						 port->bulk_out_endpointAddress,
						 USB_DIR_OUT,
						 port,
						 portdata->out_buffer
						 [j],
						 OUT_BUFLEN,
						 cbp_outdat_callback);
		}
	}
}

/** send RTS/DTR state to the port.
 *
 * This is exactly the same as SET_CONTROL_LINE_STATE from the PSTN
 * CDC.
 */
static int cbp_send_setup(struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;
	struct cbp_port_private *portdata;
	int ifNum = serial->interface->cur_altsetting->desc.bInterfaceNumber;
	dbg("%s", __func__);

	portdata = usb_get_serial_port_data(port);

	/* VIA-Telecom CBP DTR format */
	return usb_control_msg(serial->dev,
			       usb_rcvctrlpipe(serial->dev, 0),
			       0x01, 0x40, portdata->dtr_state ? 1 : 0,
			       ifNum, NULL, 0, USB_CTRL_SET_TIMEOUT);
}

static int cbp_startup(struct usb_serial *serial)
{
	int i, j, err;
	struct usb_serial_port *port;
	struct cbp_port_private *portdata;
	u8 *buffer;


	dbg("%s", __func__);

	/* Now setup per port private data */
	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
		portdata = kzalloc(sizeof(*portdata), GFP_KERNEL);
		if (!portdata) {
			dbg("%s: kmalloc for cbp_port_private (%d) failed!.",
			    __func__, i);
			return 1;
		}

		for (j = 0; j < N_IN_URB; j++) {
			buffer = (u8 *) __get_free_page(GFP_KERNEL);
			if (!buffer)
				goto bail_out_error;
			portdata->in_buffer[j] = buffer;
		}

		for (j = 0; j < N_OUT_URB; j++) {
			buffer = kmalloc(OUT_BUFLEN, GFP_KERNEL);
			if (!buffer)
				goto bail_out_error2;
			portdata->out_buffer[j] = buffer;
		}

		usb_set_serial_port_data(port, portdata);

		if (!port->interrupt_in_urb)
			continue;
		err = usb_submit_urb(port->interrupt_in_urb, GFP_KERNEL);
		if (err)
			dbg("%s: submit irq_in urb failed %d", __func__, err);
	}
	cbp_setup_urbs(serial);

	return 0;

bail_out_error2:
	for (j = 0; j < N_OUT_URB; j++)
		kfree(portdata->out_buffer[j]);
bail_out_error:
	for (j = 0; j < N_IN_URB; j++)
		if (portdata->in_buffer[j])
			free_page((unsigned long)portdata->in_buffer[j]);
	kfree(portdata);
	return 1;
}

static void stop_read_write_urbs(struct usb_serial *serial)
{
	int i, j;
	struct usb_serial_port *port;
	struct cbp_port_private *portdata;

	/* Stop reading/writing urbs */
	for (i = 0; i < serial->num_ports; ++i) {
		port = serial->port[i];
		portdata = usb_get_serial_port_data(port);
		for (j = 0; j < N_IN_URB; j++)
			usb_kill_urb(portdata->in_urbs[j]);
		for (j = 0; j < N_OUT_URB; j++)
			usb_kill_urb(portdata->out_urbs[j]);
	}
}

static void cbp_release(struct usb_serial *serial)
{
	int i, j;
	struct usb_serial_port *port;
	struct cbp_port_private *portdata;


	dbg("%s", __func__);

	stop_read_write_urbs(serial);

	/* Now free them */
	for (i = 0; i < serial->num_ports; ++i) {
		port = serial->port[i];
		portdata = usb_get_serial_port_data(port);

		for (j = 0; j < N_IN_URB; j++) {
			usb_free_urb(portdata->in_urbs[j]);
			free_page((unsigned long)
				  portdata->in_buffer[j]);
			portdata->in_urbs[j] = NULL;
		}
		for (j = 0; j < N_OUT_URB; j++) {
			usb_free_urb(portdata->out_urbs[j]);
			kfree(portdata->out_buffer[j]);
			portdata->out_urbs[j] = NULL;
		}
	}

	/* Now free per port private data */
	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
		kfree(usb_get_serial_port_data(port));
	}
}

static void cbp_disconnect(struct usb_serial *serial)
{
	dbg("%s entered", __func__);
	stop_read_write_urbs(serial);
}

#ifdef CONFIG_PM
static int cbp_suspend(struct usb_serial *serial, pm_message_t message)
{
	dbg("%s entered", __func__);
	stop_read_write_urbs(serial);

	return 0;
}

static int cbp_resume(struct usb_serial *serial)
{
	int err, i, j;
	struct usb_serial_port *port;
	struct urb *urb;
	struct cbp_port_private *portdata;

	dbg("%s entered", __func__);

	/* get the interrupt URBs resubmitted unconditionally */
	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
		if (!port->interrupt_in_urb) {
			dbg("%s: No interrupt URB for port %d\n", __func__, i);
			continue;
		}
		port->interrupt_in_urb->dev = serial->dev;
		err = usb_submit_urb(port->interrupt_in_urb, GFP_NOIO);
		dbg("Submitted interrupt URB for port %d (result %d)", i, err);
		if (err < 0) {
			err("%s: Error %d for interrupt URB of port%d",
			    __func__, err, i);
			return err;
		}
	}

	for (i = 0; i < serial->num_ports; i++) {
		/* walk all ports */
		port = serial->port[i];
		portdata = usb_get_serial_port_data(port);
		mutex_lock(&port->mutex);

		/* skip closed ports */
		if (!port->port.count) {
			mutex_unlock(&port->mutex);
			continue;
		}

		for (j = 0; j < N_IN_URB; j++) {
			urb = portdata->in_urbs[j];
			err = usb_submit_urb(urb, GFP_NOIO);
			if (err < 0) {
				mutex_unlock(&port->mutex);
				err("%s: Error %d for bulk URB %d",
				    __func__, err, i);
				return err;
			}
		}
		mutex_unlock(&port->mutex);
	}
	return 0;
}
#endif

static int usb_serial_cbp_probe(struct usb_interface *interface,
				const struct usb_device_id *id)
{
	int ret;
	struct usb_device *udev = interface_to_usbdev(interface);
	int i;
	struct usb_serial_port *port;
	struct usb_serial *serial;
	struct bypass *bypass;

	if (!udev)
		return -ENODEV;
	ret = usb_serial_probe(interface, id);
	if (ret) {
		printk(KERN_ERR "%s - probe error\n", __func__);
		return ret;
	}
	serial = usb_get_intfdata(interface);
	bypass = bypass_get();
	if (bypass == NULL || serial == NULL) {
		printk(KERN_ERR "%s - bypass_get or serial error\n", __func__);
		return -ENODEV;
	}

	for (i = 0; i < serial->num_ports; ++i) {
		port = serial->port[i];

		if (port->number == 1) {
			bypass->h_ets_port = port;
			/* after jump to loader, there only exist ets intf */
			if (bypass->ets_jump_flag == 1)
				usb_serial_port_open(bypass->h_ets_port);
			else if (bypass->ets_status)
				usb_serial_port_open(bypass->h_ets_port);
		}
	}

	/* you can filter interface here */

/* FIXME if autopm still need enable, uncomment the below code*/
/*
#if CONFIG_PM
	usb_autopm_enable(interface);
	interface->needs_remote_wakeup = 0;
	udev->autosuspend_disabled = 0;
	udev->autosuspend_delay = 3 * HZ;
#endif
*/

	return ret;
}

void usb_serial_cbp_disconnect(struct usb_interface *interface)
{
	int i;
	struct usb_serial_port *port;
	struct usb_serial *serial = usb_get_intfdata(interface);
	struct bypass *bypass = bypass_get();
	/* if disable bypass function after cp disconnect
	 * can set corresponding flag of port to be zero */
	if (bypass && bypass->ets_status) {
		for (i = 0; i < serial->num_ports; ++i) {
			port = serial->port[i];
			if (port->number == 1) {
				usb_serial_port_close(port);
				bypass->h_ets_port = NULL;
			}
		}
	}
	usb_serial_disconnect(interface);
}

int usb_serial_port_close(struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;

	if (!port)
		return -EINVAL;

	if (port->port.count == 1) {
		stop_read_write_urbs(serial);
/*
#if CONFIG_PM
		usb_autopm_put_interface(serial->interface);
#endif
*/
	}
	--port->port.count;
	return port->port.count;
}
EXPORT_SYMBOL_GPL(usb_serial_port_close);

int usb_serial_port_open(struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;
	int retval;
	unsigned long flags;

	printk("%s port_num %d count %d\n", __func__, port->number,
	       port->port.count);
	/*
	 * force low_latency on so that our tty_push actually forces the data
	 * through, otherwise it is scheduled, and with high data rates (like
	 * with OHCI) data can get lost.
	 */

	/* clear the throttle flags */
	spin_lock_irqsave(&port->lock, flags);
	++port->port.count;
	spin_unlock_irqrestore(&port->lock, flags);

	if (port->port.count != 1) {
		dbg("port already open, do nothing\n");
		return -ENODEV;
	}


	/*
	 * WARNING:
	 *  do NOT lock port->mutex before usb_autopm_get_interface(), 'cause
	 *  this will make a dead lock between autopm lock and port mutex
	 */

/*
#if CONFIG_PM
	retval = usb_autopm_get_interface(serial->interface);
	if (retval)
		goto exit;
#endif
*/

	if (mutex_lock_interruptible(&port->mutex)) {
		/*
#if CONFIG_PM
		usb_autopm_put_interface(serial->interface);
#endif
		*/
		retval = -ERESTARTSYS;
		goto exit;
	}

	retval = cbp_open(NULL, port);

	mutex_unlock(&port->mutex);
	if (retval) {
		/*
#if CONFIG_PM
		usb_autopm_put_interface(serial->interface);
#endif
		*/
		goto exit;
	}

exit:
	if (retval)
		port->port.count--;

	return retval;
}
EXPORT_SYMBOL_GPL(usb_serial_port_open);


module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug messages");
