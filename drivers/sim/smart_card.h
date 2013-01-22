/*
 * Copyright (C) 2005-2010 Motorola, Inc.
 * This program is licensed under a BSD license with the following terms:
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this list of conditions
 *   and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of
 *   conditions and the following disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * - Neither the name of Motorola nor the names of its contributors may be used to endorse or
 *   promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * Motorola 2010-Jul-19 - Add start stop clock procedure IOCTL
 * Motorola 2009-Nov-06 - Add support for local timers
 * Motorola 2009-Oct-10 - Add support for low current drain wakeups
 * Motorola 2009-Mar-16 - Support the GIT build environment
 * Motorola 2008-Dec-06 - Added a Sleep IOCTL
 * Motorola 2008-Mar-05 - OMAP support
 * Motorola 2007-May-23 - Add new IOCTLs
 * Motorola 2006-Aug-31 - Remove some functionality from the driver
 * Motorola 2006-Aug-21 - Add support for all peripheral clock frequencies
 * Motorola 2006-Jul-25 - More MVL upmerge changes
 * Motorola 2006-Jul-14 - MVL upmerge
 * Motorola 2006-May-24 - Fix GPL issues
 * Motorola 2005-Oct-04 - Initial Creation
 */
 
#ifndef __SMART_CARD_H__
#define __SMART_CARD_H__

#include <linux/ioctl.h>

#ifdef __KERNEL__
#include "smart_card_kernel.h"
#endif

enum
{
    ISR_NO_ERROR = 0,
    ISR_FATAL_ERROR,
    ISR_NACK_LIMIT_ERROR
};
typedef UINT8 SIM_MODULE_ISR_ERROR;

enum
{
    SIM_MODULE_VOLTAGE_OFF = 0,
    SIM_MODULE_VOLTAGE_LOW,
    SIM_MODULE_VOLTAGE_HIGH
};
typedef UINT8 SIM_MODULE_VOLTAGE_LEVEL;

enum
{
    SIM_MODULE_RX_MODE,
    SIM_MODULE_TX_MODE,
    SIM_MODULE_RESET_DETECT_MODE,
    SIM_MODULE_MODE_NONE
};
typedef UINT8 SIM_MODULE_INTERRUPT_MODE;

enum
{
    SIM_MODULE_1 = 0,
    NUM_SIM_MODULES
};
typedef UINT8 SIM_MODULE_ID;

enum
{
    SIM_MODULE_CLOCK_0,
    SIM_MODULE_CLOCK_4,
    SIM_MODULE_CLOCK_5
};
typedef UINT8 SIM_MODULE_CLOCK_RATE;

/* List of values for manipulating the mutex */
enum
{
    KERNEL_MUTEX_ID     = 0x01,
    SCSM_MUTEX_ID       = 0x02,
    SCPC_MUTEX_ID       = 0x03,
};
typedef UINT8 SIM_MUTEX_ID;

#define MUTEX_UNLOCK_MASK      0x10
#define MUTEX_REQUESTER_MASK   0x0F
#define KERNEL_LOCK_MUTEX      KERNEL_MUTEX_ID
#define KERNEL_UNLOCK_MUTEX    0x11

/* SIM module A events */
#define SIM_MODULE_EVENT_NONE                   0x00000000
#define SIM_MODULE_EVENT_RX_A                   0x00000001
#define SIM_MODULE_EVENT_SIMPD_BOUNCE           0x00000002
#define SIM_MODULE_EVENT_SIMPD_INSERTION        0x00000004      
#define SIM_MODULE_EVENT_SIMPD_REMOVAL          0x00000008
#define SIM_MODULE_EVENT_PARITY_ERROR           0x00000010
#define SIM_MODULE_EVENT_BUFFER_INDEX           0x00000020
#define SIM_MODULE_EVENT_ERROR_FLAG             0x00000040
#define SIM_MODULE_EVENT_NO_ATR_FLAG            0x00000080
#define SIM_MODULE_EVENT_WWT_VIOLATION          0x00000100
#define SIM_MODULE_EVENT_FIFO_OVERFLOW          0x00000200
#define SIM_MODULE_EVENT_INCOMPLETE_SLIM_STATUS 0x00000400
#define SIM_MODULE_EVENT_MUTEX_FREE             0x00000800
#define SIM_MODULE_EVENT_NULL_BYTE_OVERFLOW     0x00001000
#define SIM_MODULE_EVENT_NO_DMA_CN_AVB          0x00002000
#define SIM_MODULE_EVENT_TIMER_EXP              0x00004000
#define SIM_MODULE_EVENT_SPURIOUS_DATA          0x00008000


#define SIM_NUM 230
/*******************************************************************************************
 * Ioctl commands
 *
 *    These are the ioctl commands for the SIM interface module driver. These will not be used directly
 *    from user-space, but are used to construct the request parameters used in ioctl() 
 *    calls to the driver. 
 ******************************************************************************************/

#define SIM_IOC_CMD_CORE_CONFIGURE_GPIO                (0x00)
#define SIM_IOC_CMD_CORE_INTERRUPT_INIT                (0x01)
#define SIM_IOC_CMD_CORE_INTERRUPT_MODE                (0x02)
#define SIM_IOC_CMD_CORE_READ_BUFFER_INDEX             (0x03)
#define SIM_IOC_CMD_CORE_READ_DATA_BUFFER              (0x04)
#define SIM_IOC_CMD_CORE_READ_RX_EVENT                 (0x05)
#define SIM_IOC_CMD_CORE_READ_SIM_REG_DATA             (0x06)
#define SIM_IOC_CMD_CORE_RESET_CARD_READER_DATA        (0x07)
#define SIM_IOC_CMD_CORE_SET_CWTCNT                    (0x08)
#define SIM_IOC_CMD_CORE_SET_GPCNT                     (0x09)
#define SIM_IOC_CMD_CORE_SET_VOLTAGE_LEVEL             (0x0A)
#define SIM_IOC_CMD_CORE_UPDATE_DATA_BUFFER            (0x0B)
#define SIM_IOC_CMD_CORE_UPDATE_BUFFER_INDEX           (0x0C)
#define SIM_IOC_CMD_CORE_UPDATE_RX_LAST_INDEX          (0x0D)
#define SIM_IOC_CMD_CORE_WRITE_SIM_REG_BIT             (0x0E)
#define SIM_IOC_CMD_CORE_WRITE_SIM_REG_DATA            (0x0F)
#define SIM_IOC_CMD_CORE_ALL_TX_DATA_SENT              (0x10)
#define SIM_IOC_CMD_CORE_RESET_ALL_TX_DATA_SENT        (0x11)
#define SIM_IOC_CMD_CORE_SET_SIM_CLOCK_RATE            (0x12)
#define SIM_IOC_CMD_CORE_DATA_TX                       (0x13)
#define SIM_IOC_CMD_CORE_GET_SIM_CLOCK_FREQ            (0x14)
#define SIM_IOC_CMD_CORE_LOW_POWER_STATE               (0x15)
#define SIM_IOC_CMD_CORE_SLEEP                         (0x16)
#define SIM_IOC_CMD_CORE_CARD_TYPE                     (0x17)
#define SIM_IOC_CMD_CORE_MUTEX_UPDATE                  (0x18)
#define SIM_IOC_CMD_CORE_POLL_INTERVAL                 (0x19)
#define SIM_IOC_CMD_CORE_CALL_STATUS                   (0x1A)
#define SIM_IOC_CMD_CORE_START_TIMER                   (0x1B)
#define SIM_IOC_CMD_CORE_STOP_TIMER                    (0x1C)
#define SIM_IOC_CMD_CORE_START_STOP_CLK_PROCEDURE      (0x1D)
#define SIM_IOC_CMD_CORE_LAST_CMD                         \
				SIM_IOC_CMD_CORE_START_STOP_CLK_PROCEDURE

/*******************************************************************************************
 * Ioctl requests
 *
 *    These are the requests that can be passed to ioctl() to request operations on the
 *    SIM interface module driver.
 ******************************************************************************************/
#define SIM_IOCTL_CONFIGURE_GPIO \
        _IOW(SIM_NUM, SIM_IOC_CMD_CORE_CONFIGURE_GPIO, UINT8)
#define SIM_IOCTL_INTERRUPT_INIT \
        _IOW(SIM_NUM,  SIM_IOC_CMD_CORE_INTERRUPT_INIT, UINT32 *)
#define SIM_IOCTL_INTERRUPT_MODE \
        _IOW(SIM_NUM, SIM_IOC_CMD_CORE_INTERRUPT_MODE, UINT8)
#define SIM_IOCTL_READ_BUFFER_INDEX \
        _IOR(SIM_NUM, SIM_IOC_CMD_CORE_READ_BUFFER_INDEX, UINT32 *)
#define SIM_IOCTL_READ_DATA_BUFFER \
        _IOR(SIM_NUM, SIM_IOC_CMD_CORE_READ_DATA_BUFFER, UINT32 *)
#define SIM_IOCTL_READ_RX_EVENT \
        _IOR(SIM_NUM, SIM_IOC_CMD_CORE_READ_RX_EVENT, UINT32 *)
#define SIM_IOCTL_READ_SIM_REG_DATA \
        _IOR(SIM_NUM, SIM_IOC_CMD_CORE_READ_SIM_REG_DATA, UINT32 *)
#define SIM_IOCTL_RESET_CARD_READER_DATA \
        _IOW(SIM_NUM, SIM_IOC_CMD_CORE_RESET_CARD_READER_DATA, UINT32 *)
#define SIM_IOCTL_SET_VOLTAGE_LEVEL \
        _IOW(SIM_NUM, SIM_IOC_CMD_CORE_SET_VOLTAGE_LEVEL, UINT32 *)
#define SIM_IOCTL_UPDATE_DATA_BUFFER \
        _IOW(SIM_NUM, SIM_IOC_CMD_CORE_UPDATE_DATA_BUFFER, UINT8 *)
#define SIM_IOCTL_UPDATE_BUFFER_INDEX \
        _IOW(SIM_NUM, SIM_IOC_CMD_CORE_UPDATE_BUFFER_INDEX, UINT32 *)
#define SIM_IOCTL_UPDATE_RX_LAST_INDEX \
        _IOW(SIM_NUM, SIM_IOC_CMD_CORE_UPDATE_RX_LAST_INDEX, UINT32 *)
#define SIM_IOCTL_WRITE_SIM_REG_BIT \
        _IOW(SIM_NUM, SIM_IOC_CMD_CORE_WRITE_SIM_REG_BIT, UINT32 *)
#define SIM_IOCTL_WRITE_SIM_REG_DATA \
        _IOW(SIM_NUM, SIM_IOC_CMD_CORE_WRITE_SIM_REG_DATA, UINT32 *)
#define SIM_IOCTL_ALL_TX_DATA_SENT \
       _IOR(SIM_NUM, SIM_IOC_CMD_CORE_ALL_TX_DATA_SENT, BOOLEAN *)
#define SIM_IOCTL_RESET_ALL_TX_DATA_SENT \
       _IO(SIM_NUM, SIM_IOC_CMD_CORE_RESET_ALL_TX_DATA_SENT)
#define SIM_IOCTL_SET_SIM_CLOCK_RATE \
       _IOW(SIM_NUM, SIM_IOC_CMD_CORE_SET_SIM_CLOCK_RATE, UINT32)
#define SIM_IOCTL_DATA_TX \
       _IOW(SIM_NUM, SIM_IOC_CMD_CORE_DATA_TX, UINT32 *)
#define SIM_IOCTL_GET_SIM_CLOCK_FREQ \
       _IOR(SIM_NUM, SIM_IOC_CMD_CORE_GET_SIM_CLOCK_FREQ, UINT32 *)
#define SIM_IOCTL_LOW_POWER_STATE \
       _IOW(SIM_NUM, SIM_IOC_CMD_CORE_LOW_POWER_STATE, UINT32 *)
#define SIM_IOCTL_SLEEP \
       _IO(SIM_NUM, SIM_IOC_CMD_CORE_SLEEP)
#define SIM_IOCTL_CARD_TYPE \
       _IOW(SIM_NUM, SIM_IOC_CMD_CORE_CARD_TYPE, UINT32 *)
#define SIM_IOCTL_MUTEX_UPDATE \
       _IOW(SIM_NUM, SIM_IOC_CMD_CORE_MUTEX_UPDATE, UINT32 *)
#define SIM_IOCTL_POLL_INTERVAL \
       _IOW(SIM_NUM, SIM_IOC_CMD_CORE_POLL_INTERVAL, UINT32 *)
#define SIM_IOCTL_CALL_STATUS \
       _IOW(SIM_NUM, SIM_IOC_CMD_CORE_CALL_STATUS, UINT32 *)
#define SIM_IOCTL_START_TIMER \
       _IO(SIM_NUM, SIM_IOC_CMD_CORE_START_TIMER)
#define SIM_IOCTL_STOP_TIMER \
       _IO(SIM_NUM, SIM_IOC_CMD_CORE_STOP_TIMER)
#define SIM_IOCTL_START_STOP_CLK_PROCEDURE \
       _IOW(SIM_NUM, SIM_IOC_CMD_CORE_START_STOP_CLK_PROCEDURE, UINT32 *)

#endif /* __SMART_CARD_H__ */
