/*
 * include/leds-bu9847.h
 *
 * Configuration for BU9847 Flash driver
 */

#ifndef __LEDS_BU9847_H
#define __LEDS_BU9847_H

#include <linux/ioctl.h>

/*This definition for Xenon flash driver.*/

#define E_OK 0

/* BU9847 indicator */
#define BU9847_DRIVER_NAME 	     "bu9847"
#define BU9847_DEVICE_NAME            "bu9847"

/*#define BU9847_SLAVE_ADDR   (0x50)*/ /*For P1 unit.*/
#define BU9847_SLAVE_ADDR   (0x54) /*For P2 unit.*/

#define MAX_RETRY_CNT          (10)
#define BU9847_REG_ADDR_LEN    (1)    /* 1 BYTE */
#define BU9847_REG_DATA_LEN    (1)    /* 1 BYTE */
#define BU9847_REG_ADDR_DATA_LEN    (2)    /* 2 BYTE */

#define BU9847_MAX_RW_SIZE      (1)    /* 1 BYTE */

#define BU9847_REG_ADDR_POS 0   /*Address position is first of array.*/
#define BU9847_REG_DATA_POS 1   /*Data position is second of array.*/
/* BU9847 control register map */

#define BU9847_REG_TERM 0xFF
#define BU9847_VAL_TERM 0xFF

#define BU9847_HW_ID_REG	    0x00
#define BU9847_PRID1_REG         0x01
#define BU9847_PRID2_REG         0x02
#define BU9847_PRID3_REG         0x03

#define BU9847_QCHC1_REG         0x04
#define BU9847_QCHC2_REG         0x05
#define BU9847_QCHC3_REG         0x06
#define BU9847_QCHC4_REG         0x07
#define BU9847_QCHC5_REG         0x08
#define BU9847_QCHC6_REG         0x09
#define BU9847_QCHC7_REG         0x0A
#define BU9847_QCHC8_REG         0x0B
#define BU9847_QCHC9_REG         0x0C
#define BU9847_QCHC10_REG         0x0D
#define BU9847_QCHC11_REG         0x0E
#define BU9847_QCHC12_REG         0x0F

#define BU9847_VCHGC_REG           0x10
#define BU9847_PRLDT_REG           0x11

/* ****************************************************************************
 *   Command ID
 * ************************************************************************** */

#define BU9847_IOCTL_BASE 42

/*Below direct register control is for exceptional case for HAL.*/
#define BU9847_IOCTL_INIT _IO(BU9847_IOCTL_BASE, 20) /*init for BU9847 device.*/

#endif /* __LEDS_BU9847_H */
