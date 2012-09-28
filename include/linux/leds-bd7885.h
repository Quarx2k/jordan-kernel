/*
 * include/leds-bd7885.h
 *
 * Configuration for BD7885 Flash driver
 */

#ifndef __LEDS_BD7885_H
#define __LEDS_BD7885_H

#include <linux/ioctl.h>

/*This definition for Xenon flash driver.*/

#define E_OK 0

/* BD7885 indicator */
#define BD7885_DRIVER_NAME 	     "bd7885"
#define BD7885_DEVICE_NAME       "bd7885"

#define BD7885_SLAVE_ADDR   (0x51)

#define F_RDY_N_GPIO             36
#define MAX_RETRY_CNT          (10)
#define BD7885_REG_ADDR_LEN    (1)    /* 1 BYTE */
#define BD7885_REG_DATA_LEN    (1)    /* 1 BYTE */
#define BD7885_REG_ADDR_DATA_LEN    (2)    /* 2 BYTE */

#define BD7885_MAX_RW_SIZE      (1)    /* 1 BYTE */

#define BD7885_REG_ADDR_POS 0   /*Address position is first of array.*/
#define BD7885_REG_DATA_POS 1   /*Data position is second of array.*/
/* BD7885 control register map */

/* BD7885 POWER STATUS */
#define BD7885_INIT_STATUS	0
#define BD7885_CHARGE_ENABLE_STATUS	1
#define BD7885_CHARGE_DISABLE_STATUS 	2

#define BD7885_REG_TERM 0xFF
#define BD7885_VAL_TERM 0xFF

#define BD7885_HW_ID_REG		0x00
#define BD7885_HW_ID_VAL		0x01

#define BD7885_DRVCNT_REG         0x01
#define BD7885_PCNT_EN_MASK 0x1
#define BD7885_CHG_EN_MASK 0x2

#define BD7885_RECHG_REG           0x02
#define BD7885_RECHG_EN_MASK 0x10

#define BD7885_IPEAKADJ_REG      0x03

#define BD7885_FULLADJ_REG        0x04
#define BD7885_FULL_ADJ_LVL_MASK 0x0F

#define BD7885_QUENCHCNT_REG   0x05

#define BD7885_CAPDIS_EN_MASK  0x2
#define BD7885_QUENCH_EN_MASK 0x1

#define BD7885_QUENCH_ENABLE    0x1
#define BD7885_QUENCH_DISABLE   0x0

#define BD7885_DELAYADJ_REG      0x06

#define BD7885_QUENCHADJ_REG   0x07
#define BD7885_VSTOPADJ_SHIFT 0x3

#define BD7885_STATUS_REG          0x08
#define BD7885_STATUS_FULL_CHG_MASK 0x1
#define BD7885_STATUS_QMON_MASK 0x2

#define BD7885_STROBE_OFF         0
#define BD7885_STROBE_QUENCH_MODE 1
#define BD7885_STROBE_MANUAL_MODE 2

#define BD7885_CHARGE_DISABLE     0
#define BD7885_CHARGE_ENABLE      1

typedef struct {
	unsigned char reg;
	unsigned char	   data;
} bd7885_cfg;

/* ****************************************************************************
 *   Command ID
 * ************************************************************************** */

#define BD7885_IOCTL_BASE 42
/** \def BD7885_IOCTL_*
 *  \brief The following define the IOCTL command values via the ioctl macros
 */
/*Below direct register control is for exceptional case for HAL.*/
#define BD7885_IOCTL_GET_REGISTER  _IOR(BD7885_IOCTL_BASE, 0, bd7885_cfg)
		/*Register value read out from Xenon flash module.*/
#define BD7885_IOCTL_SET_REGISTER  _IOW(BD7885_IOCTL_BASE, 1, bd7885_cfg)
		/*Register value write down to Xenon flash module.*/
#define BD7885_IOCTL_SET_CHARGING  _IOW(BD7885_IOCTL_BASE, 2, unsigned char)
		/*Set charging status.*/

#define BD7885_IOCTL_SET_CHARGE_LEVEL _IOW(BD7885_IOCTL_BASE, 3, unsigned char)
		/*Set FULLADJ value for charging.*/
#define BD7885_IOCTL_SET_MODE  _IOW(BD7885_IOCTL_BASE, 4, unsigned char)
		/* 0=Off, 1= Quench mode, 2=Manual Mode */
#define BD7885_IOCTL_SET_QUENCH_THRESHOLD \
			_IOW(BD7885_IOCTL_BASE, 5, unsigned char)
		/*Sets VSTOPADJ parameter.*/
#define BD7885_IOCTL_GET_STATUS _IOR(BD7885_IOCTL_BASE, 6, unsigned char)
		/*Charged status get.*/

#define BD7885_IOCTL_READY_STROBE_MANUAL \
			_IOW(BD7885_IOCTL_BASE, 7, unsigned char)
		/* test ready strobe manually.*/
#define BD7885_IOCTL_FIRE_STROBE_MANUAL \
		_IOW(BD7885_IOCTL_BASE, 8, unsigned char)
		/* test fire strobe manually.*/

#endif /* __LEDS_BD7885_H */
