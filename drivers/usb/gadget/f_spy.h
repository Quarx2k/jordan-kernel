 /*
  *  f_spy.h  --  SPY tracer Gadget driver
  *
  *
  *  This program is free software; you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation; either version 2 of the License, or
  *  (at your option) any later version.
  *
  */

#ifndef _F_SPY_H_
#define _F_SPY_H_


int spy_ctrlrequest(struct usb_composite_dev *cdev,
		const struct usb_ctrlrequest *ctrl);
int spy_bind_config(struct usb_configuration *c, u8 port_num);
int spy_init(struct usb_composite_dev *cdev);
void spy_cleanup(void);

#endif /* _F_SPY_H_ */
