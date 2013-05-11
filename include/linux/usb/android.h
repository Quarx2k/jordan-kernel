#ifndef __LINUX_USB_ANDROID_H
#define __LINUX_USB_ANDROID_H

struct android_usb_platform_data {
        int (*update_pid_and_serial_num)(uint32_t, const char *);
};

#endif  /* __LINUX_USB_ANDROID_H */

