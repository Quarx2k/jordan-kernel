/*
 * drivers/i2c/chips/leds-bu9847.c
 *
 * I2C slave driver for BU9847 device
 *
 * Copyright (C) 2008-2009 Motorola, Inc.
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
 * Revision History:
 *
 * Date          Author    Comment
 * -----------   --------  -------------------
 * Jun-24-2008   Motorola  Initial version
 *
 */

#include <linux/kernel.h>	/* printk() */
#include <linux/fs.h>		/* everything... */
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#include "linux/leds-bu9847.h"

static int bu9847_open(struct inode *inode, struct file *file);
static int bu9847_release(struct inode *inode, struct file *file);
static int bu9847_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg);

struct bu9847_chip {
	struct mutex lock; /* checkpatch need some comments here! */
	struct i2c_client *client;
	int status;               /* flag indicate chip current power state */
};

const struct file_operations bu9847_fops = {
	.owner   = THIS_MODULE,
	.open    = bu9847_open,
	.release = bu9847_release,
	.ioctl   = bu9847_ioctl,
};

static struct miscdevice bu9847_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = BU9847_DEVICE_NAME,
  .fops = &bu9847_fops,
};

/*for register value keep here.*/
static char bu9847_reg_info_tbl[18];

/* for ioctl */
struct i2c_client *bu9847_i2c_client;

/* device major number */
int bu9847_major_num;
int bu9847_minor_num;

#define BU9847_I2C_RETRY_MAX 5

/* *****************************************************

	Low level interface routine.

* *****************************************************/
static int bu9847_reg_read(struct i2c_client *client, u8 reg_addr, u8 *buf)
{
    /** \var error
     *  \brief Return error code, initialized to no error
     */
    int error = E_OK;
    int ret = 0;
    /** \var i
     *  \brief Retry counter
     */
    int i = 0;
    unsigned char value[BU9847_REG_DATA_LEN];

    /* If number of bytes is out of range or
	buffer is incorrect, report, exit */
    if ((buf == NULL) || (reg_addr > BU9847_PRLDT_REG)) {
	printk(KERN_ERR "invalid read parameter(s)\n");
	return -EINVAL;
    }
    /* If I2C client doesn't exist */
    if (client == NULL) {
	printk(KERN_ERR "bu9847 null i2c read client\n");
	return -EUNATCH;
    }

	/* Do read */
    do {
	/* Establish read address
	*  i2c_master_send returns either the number of bytes
	* sent or negative error from i2c_transfer
	*/
	/* Copy the data into a buffer for correct format */
	value[0] = reg_addr & 0xFF;
	ret = i2c_master_send(client, value , BU9847_REG_DATA_LEN);
	/* Read data if send address succeeded
	*  (i2c_master_send returns the number
	*  of bytes transferred) */
	if (ret == BU9847_REG_DATA_LEN) {
		/* Retreive data
		*  i2c_master_recv returns either the number of bytes
		* received or negative error from i2c_transfer
		*/
		ret = i2c_master_recv(client, buf, BU9847_REG_DATA_LEN);
	} else {
		printk(KERN_ERR "bu9847 write of address %d failed: %d\n",
						value[0], ret);
	}

	/* On failure (error will contain num_bytes if succeeded),
		print error code and delay before trying again */
	if (ret != BU9847_REG_DATA_LEN) {
		printk(KERN_ERR "bu9847 read[%i] failed: %d\n", i, ret);
		msleep(10);
	}
    /* Do while an error exists and we haven't exceeded retry attempts */
    } while ((ret != BU9847_REG_DATA_LEN) && ((++i) < BU9847_I2C_RETRY_MAX));

    /* On success, set error to E_OK (i2c_master_recv returns
	the number of bytes transferred) */
    if (ret == BU9847_REG_DATA_LEN)
	error = E_OK;
    else
	error = ret;

    return error;
}

int bu9847_fetch_regs(void)
{
    unsigned char reg_cnt = 0;
    unsigned int ret = 0;

    /*Read out status register information.*/
    for (reg_cnt = BU9847_HW_ID_REG; reg_cnt <= BU9847_PRLDT_REG; reg_cnt++) {
	ret = bu9847_reg_read(bu9847_i2c_client,
			reg_cnt, &bu9847_reg_info_tbl[reg_cnt]);
	if (ret != 0) {
		printk(KERN_INFO "bu9847_reg_read_filed. reg_addr = %d\n",
					reg_cnt);
		return ret;
	}
    }

    return 0;
}

/* *****************************************************

	Probe, Remove

* ******************************************************/
static int bu9847_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bu9847_chip *chip;
	int ret = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		ret  = -ENODEV;
		goto exit_check_functionality_failed;
	}

       bu9847_i2c_client = client;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	printk(KERN_INFO "BU9847 driver: bu9847_probe()\n");

	chip->client = client;
	strncpy(client->name, BU9847_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, chip);

       ret = misc_register(&bu9847_device);
	/*
       if (!ret)
	    printk(KERN_INFO "misc_register failed\n");
	*/

	mutex_init(&chip->lock);

	printk(KERN_INFO "BU9847 chip probe is finished.\n");

	return ret;

exit_check_functionality_failed:
	return ret;
}

static int bu9847_remove(struct i2c_client *client)
{
	struct bu9847_chip *chip = i2c_get_clientdata(client);

	printk(KERN_INFO "bu9847_remove() is called!\n");
	kfree(chip);

	return 0;
}

static const struct i2c_device_id bu9847_id[] = {
	{ BU9847_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bu9847_id);

static struct i2c_driver bu9847_driver = {
	.driver = {
		.name = BU9847_DRIVER_NAME,
	},
	.probe	  = bu9847_probe,
	.remove	  = __devexit_p(bu9847_remove),
	.id_table = bu9847_id,
};

static int bu9847_open(struct inode *inode, struct file *file)
{
  printk(KERN_INFO "%s is called.\n", __func__);
  return nonseekable_open(inode, file);
}

static int bu9847_release(struct inode *inode, struct file *file)
{
  printk(KERN_INFO "%s is called.\n", __func__);
  return 0;
}
static int bu9847_ioctl(
	struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    /* Do the handling. */
    switch (cmd) {
    case BU9847_IOCTL_INIT:
    /*EEPROM data fetch from HW.*/
	    ret = bu9847_fetch_regs();
	    break;

    default:
	    printk(KERN_INFO "%s's default called. %d\n", __func__, cmd);
	    break;
    }

    return ret;
}
/*
 *  BU9847 voltage regulator init
 */
static int __init bu9847_init(void)
{
    int ret = -1;

    ret = i2c_add_driver(&bu9847_driver);
    if (ret != 0)
	printk(KERN_INFO "BU9847 init failed.\n");

     printk(KERN_INFO "BU9847 i2c driver : init is completed.\n");

    return ret;
}

static void __exit bu9847_exit(void)
{
	i2c_del_driver(&bu9847_driver);

	printk(KERN_INFO "BU9847  flash driver: exit\n");
}

module_init(bu9847_init);
module_exit(bu9847_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("BU9847 flash regulator driver");
MODULE_LICENSE("GPL");
