/*
 * Copyright (C) 2012 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/m4sensorhub.h>
#include <linux/slab.h>

#include "m4sensorhub-reg.h"    /* auto-generated header defining registers */

#define I2C_RETRY_DELAY              5
#define I2C_RETRIES                  5

#define DEBUG_LINE_LENGTH           80

/* --------------- Global Declarations -------------- */

/* ------------ Local Function Prototypes ----------- */
static int m4sensorhub_mapsize(enum m4sensorhub_reg reg);

/* --------------- Local Declarations -------------- */
static DEFINE_MUTEX(reg_access);

/* -------------- Local Data Structures ------------- */

/* -------------- Global Functions ----------------- */

/* m4sensorhub_reg_init()

   Initialized register access data structures.

   Returns 0 on success or negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
*/

int m4sensorhub_reg_init(struct m4sensorhub_data *m4sensorhub)
{
	return 0;
}
EXPORT_SYMBOL_GPL(m4sensorhub_reg_init);

/* m4sensorhub_reg_shutdown()

   Clean up register subsystem on driver removal

   Returns 0 on success or negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
*/

int m4sensorhub_reg_shutdown(struct m4sensorhub_data *m4sensorhub)
{
	return 0;
}
EXPORT_SYMBOL_GPL(m4sensorhub_reg_shutdown);

/* m4sensorhub_reg_read_n()

   Read a n bytes from the M4 sensor hub starting at 'register'.  Use
   m4sensorhub_reg_read() instead where possible;

   Returns number of bytes read on success.
   Returns negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
     reg - Register to be read
     value - array to return data.  Needs to be at least register's size
     num - number of bytes to read
*/

int m4sensorhub_reg_read_n(struct m4sensorhub_data *m4sensorhub,
			   enum m4sensorhub_reg reg, unsigned char *value,
			   short num)
{
	int ret = -EINVAL;
	u8 stack_buf[M4SH_MAX_STACK_BUF_SIZE];

	if (!m4sensorhub || !value || !num) {
		KDEBUG(M4SH_ERROR, "%s() invalid parameter\n", __func__);
		return ret;
	}

	if ((reg < M4SH_REG__NUM) && num <= M4SH_MAX_REG_SIZE &&\
	     register_info_tbl[reg].offset + num <=
	     m4sensorhub_mapsize(reg)) {
		u8 *buf = (num > (M4SH_MAX_STACK_BUF_SIZE-2))\
			 ? kmalloc(num+2, GFP_KERNEL) : stack_buf;
		if (!buf) {
			KDEBUG(M4SH_ERROR, "%s() Failed alloc %d memeory\n"\
					, __func__, num+2);
			return -ENOMEM;
		}
		buf[0] = register_info_tbl[reg].type;
		buf[1] = register_info_tbl[reg].offset;

		mutex_lock(&reg_access);
		ret = m4sensorhub_i2c_write_read(m4sensorhub, buf, 2, num);
		mutex_unlock(&reg_access);

		if (ret != num)
			KDEBUG(M4SH_ERROR, "%s() read failure\n", __func__);
		else
			memcpy(value, buf, num);
		if (buf != stack_buf)
			kfree(buf);
	} else {
		KDEBUG(M4SH_ERROR, "%s() invalid register access reg=%d "
			"maxreg=%d size=%d maxsze=%d mapsize=%d\n", __func__,
			reg, M4SH_REG__NUM, num, M4SH_MAX_REG_SIZE,
			m4sensorhub_mapsize(reg));
	}
	return ret;
}
EXPORT_SYMBOL_GPL(m4sensorhub_reg_read_n);

/* m4sensorhub_reg_write()

   Write data to a register in the M4 sensor hub.  Use
   m4sensorhub_reg_write() instead where possible;

   Returns number of bytes written on success.
   Returns negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
     reg - Register to be written to
     value - array of data to write.  Needs to be at least register's size
     mask - mask representing which bits to change in register.  If all bits
	    are to be changed, then &m4sh_no_mask can be passed here.
     num - number of bytes to write
*/

int m4sensorhub_reg_write_n(struct m4sensorhub_data *m4sensorhub,
			    enum m4sensorhub_reg reg, unsigned char *value,
			    unsigned char *mask, short num)
{
	int i, ret = -EINVAL;
	u8 stack_buf[M4SH_MAX_STACK_BUF_SIZE];

	if (!m4sensorhub || !value || !num) {
		KDEBUG(M4SH_ERROR, "%s() invalid parameter\n", __func__);
		return ret;
	}

	if ((reg < M4SH_REG__NUM) && num <= M4SH_MAX_REG_SIZE &&\
	     register_info_tbl[reg].offset + num <=
	     m4sensorhub_mapsize(reg)) {
		u8 *buf = (num > (M4SH_MAX_STACK_BUF_SIZE-2))\
			? kmalloc(num+2, GFP_KERNEL) : stack_buf;
		if (!buf) {
			KDEBUG(M4SH_ERROR, "%s() Failed alloc %d memeory\n"\
					, __func__, num+2);
			return -ENOMEM;
		}

		buf[0] = register_info_tbl[reg].type;
		buf[1] = register_info_tbl[reg].offset;

		mutex_lock(&reg_access);
		if (mask) {
			ret = m4sensorhub_i2c_write_read(m4sensorhub, buf,
							 2, num);
			if (ret != num) {
				KDEBUG(M4SH_ERROR, "%s() register read"
						   "failure\n", __func__);
				goto error;
			}
			/* move data right 2 positions and apply mask and
			   new data to prepare for writeback */
			for (i = num-1; i >= 0; i--) {
				buf[i+2] = (buf[i] & ~mask[i]) |
						 (value[i] & mask[i]);
			}
			buf[0] = register_info_tbl[reg].type;
			buf[1] = register_info_tbl[reg].offset;
		} else
			memcpy(&buf[2], value, num);

		ret = m4sensorhub_i2c_write_read(m4sensorhub, buf,
						 num + 2, 0);
		if (ret != num + 2) {
			KDEBUG(M4SH_ERROR, "%s() register write failure\n",
			      __func__);
			ret = -EINVAL;
		} else
			ret -= 2;

error:		mutex_unlock(&reg_access);
		if (buf != stack_buf)
			kfree(buf);
	} else {
		KDEBUG(M4SH_ERROR, "%s() invalid register access reg=%d"
			" maxreg=%d num=%d maxsze=%d mapsize=%d\n", __func__,
			reg, M4SH_REG__NUM, num, M4SH_MAX_REG_SIZE,
			m4sensorhub_mapsize(reg));
	}

	return ret;
}
EXPORT_SYMBOL_GPL(m4sensorhub_reg_write_n);

/* m4sensorhub_reg_write_1byte()

   Write data to a 1 byte register in the M4 sensor hub.  Avoids need to pass
   data and mask by reference

   Returns number of bytes written on success.
   Returns negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
     reg - Register to be written to
     value - byte of data to write
     mask - mask representing which bits to change in register.
*/

int m4sensorhub_reg_write_1byte(struct m4sensorhub_data *m4sensorhub,
				enum m4sensorhub_reg reg, unsigned char value,
				unsigned char mask)
{
	if (register_info_tbl[reg].size == 1) {
		if (mask == 0xFF) {
			return m4sensorhub_reg_write(
					m4sensorhub, reg, &value, NULL
					);
		} else {
			return m4sensorhub_reg_write(
				m4sensorhub, reg, &value, &mask
				);
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(m4sensorhub_reg_write_1byte);

/* m4sensorhub_reg_getsize()

   Get the size of an M4 register

   Returns size of register on success
   Returns negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
     reg - Register to get size of
*/

int m4sensorhub_reg_getsize(struct m4sensorhub_data *m4sensorhub,
			    enum m4sensorhub_reg reg)
{
	if (reg < M4SH_REG__NUM)
		return register_info_tbl[reg].size;

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(m4sensorhub_reg_getsize);

/* m4sensorhub_reg_access_lock()

   Lock reg access to avoid broken I2C transmit process

*/

void m4sensorhub_reg_access_lock(void)
{
	mutex_lock(&reg_access);
}
EXPORT_SYMBOL_GPL(m4sensorhub_reg_access_lock);

/* m4sensorhub_reg_access_unlock()

   Unlock reg access to wake up blocked I2C transmit process

*/

void m4sensorhub_reg_access_unlock(void)
{
	mutex_unlock(&reg_access);
}
EXPORT_SYMBOL_GPL(m4sensorhub_reg_access_unlock);

/* m4sensorhub_i2c_write_read()

   Directly I2C access to communicate with the M4 sensor hub.
   It always read after write if both write and read length are non-zero

   Returns number of bytes write on success if readlen is zero
   Returns number of bytes read on success if readlen is non-zero
   Returns negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
     buf - buffer to be used for write to and read from
     writelen - number of bytes write to
     readlen - number of bytes read from
*/

int m4sensorhub_i2c_write_read(struct m4sensorhub_data *m4sensorhub,
				      u8 *buf, int writelen, int readlen)
{
	int i, msglen, msgstart, err, tries = 0;
	char buffer[DEBUG_LINE_LENGTH];
	struct i2c_msg msgs[] = {
		{
		.addr = m4sensorhub->i2c_client->addr,
		.flags = m4sensorhub->i2c_client->flags,
		.len = writelen,
		.buf = buf,
		},
		{
		.addr = m4sensorhub->i2c_client->addr,
		.flags = m4sensorhub->i2c_client->flags | I2C_M_RD,
		.len = readlen,
		.buf = buf,
		},
	};

	if (buf == NULL || (writelen == 0 && readlen == 0))
		return -EFAULT;

	/* Offset and size in msgs array depending on msg type */
	msglen = (writelen && readlen) ? 2 : 1;
	msgstart = writelen ? 0 : 1;

	if (m4sensorhub_debug >= M4SH_VERBOSE_DEBUG && writelen) {
		sprintf(buffer, "Writing to M4:");
		for (i = 0; i < writelen; i++) {
			if (strlen(buffer) >= DEBUG_LINE_LENGTH-5) {
				KDEBUG(M4SH_VERBOSE_DEBUG, "%s\n", buffer);
				buffer[0] = '\0';
			}
			sprintf(&buffer[strlen(buffer)], " 0x%02x", buf[i]);
		}
		KDEBUG(M4SH_VERBOSE_DEBUG, "%s\n", buffer);
	}

	do {
		err = i2c_transfer(m4sensorhub->i2c_client->adapter,
				   &msgs[msgstart], msglen);
		if (err != msglen)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != msglen) && (++tries < I2C_RETRIES));
	if (err != msglen) {
		dev_err(&m4sensorhub->i2c_client->dev, "i2c transfer error; "
			"type=%d offset=%d\n", buf[0], buf[1]);
		err = -EIO;
	} else {
		err = (readlen ? readlen : writelen);

		if (m4sensorhub_debug >= M4SH_VERBOSE_DEBUG && readlen) {
			sprintf(buffer, "Read from M4:");
			for (i = 0; i < readlen; i++) {
				if (strlen(buffer) >= DEBUG_LINE_LENGTH-5) {
					KDEBUG(M4SH_VERBOSE_DEBUG, "%s\n",
					       buffer);
					buffer[0] = '\0';
				}
				sprintf(&buffer[strlen(buffer)], " 0x%02x",
					buf[i]);
			}
			KDEBUG(M4SH_VERBOSE_DEBUG, "%s\n", buffer);
		}
	}
	return err;
}
EXPORT_SYMBOL_GPL(m4sensorhub_i2c_write_read);

/* -------------- Local Functions ----------------- */

static int m4sensorhub_mapsize(enum m4sensorhub_reg reg)
{
	int retval = -EINVAL;

	if (reg < M4SH_REG__NUM)
		retval = bank_size_tbl[register_info_tbl[reg].type];

	return retval;
}
