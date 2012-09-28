#ifndef __MIPI_DLI_H
#define __MIPI_DLI_H

#include <linux/ioctl.h>

#define MIPI_DLI_DRIVER_NAME 	     "mipi_dli_tester"
#define MIPI_DLI_DEVICE_NAME 	     "mipi_dli_tester"

#define MIPI_DLI_IOCTL_BASE 88

/* MIPI DLI Frame count.*/
#define MIPI_DLI_IOCTL_FRAME_COUNT  _IOWR(MIPI_DLI_IOCTL_BASE, 0, unsigned long)
/* MIPI DLI ECC count.*/
#define MIPI_DLI_IOCTL_ECC_COUNT  _IOWR(MIPI_DLI_IOCTL_BASE, 1, unsigned long)
/* MIPI DLI CRC count.*/
#define MIPI_DLI_IOCTL_CRC_COUNT  _IOWR(MIPI_DLI_IOCTL_BASE, 2, unsigned long)

#endif /*__MIPI_DLI_H*/
