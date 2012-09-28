/*
 * Copyright (C) 2010 Motorola, Inc.
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

#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/time.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>
#include <linux/pm_dbg.h>

#define PM_DBG_RECORD_ADDR_ST 		0x134
#define PM_DBG_RECORD_LENGTH		12
#define PM_DBG_RECORD_ADDR_TI 		0x9108
#define CC_LSB_TI			91501	/* uAmS per LSB */
#define CC_LSB_ST			95374	/* uAmS per LSB */
#define CC_DUR				250	/* integration duration */

enum update_mode {
	ACTIVE_START,
	ACTIVE_UPDATE,
	ACTIVE_STOP,
	IDLE_START,
	IDLE_STOP,
	IDLE_MACRO_START,
	IDLE_MACRO_STOP,
	PROFILE_START,
	PROFILE_SUSPEND,
	PROFILE_RESUME,
};

struct pm_dbg_currmeas_data {
	unsigned long time_start;
	unsigned long time_stop;
	long acc_start;
	long sample_start;
	long long acc_active;
	long sample_active;
	unsigned long time_susp;
	unsigned long time_res;
	long acc_start_susp;
	long sample_start_susp;
	long long acc_susp;
	long sample_susp;
	unsigned long time_susp_mc;
	unsigned long time_res_mc;
	unsigned long time_susp_rpt;
	unsigned long time_res_rpt;
	int avg_curr_susp;
	int min_curr_susp;
	long acc_pf_res;
	long sample_pf_res;
	long acc_pf_susp;
	long sample_pf_susp;
	int pf_avg;
	int pf_act;
	int pf_susp;
};

struct pm_dbg_data {
	struct cpcap_device *cpcap;
	struct delayed_work work;
	struct mutex lock;
	unsigned char suspend_meas_enable;
	unsigned char active_meas_enable;
	unsigned char macro_meas_enable;
	unsigned char profile_enable;
	unsigned char is_supported;
	unsigned short cd_factor;
};

struct pm_reg {
	short offset;
	long sample;
	long accumulator;
};

static struct pm_dbg_currmeas_data currmeas_data;
static struct pm_dbg_data *pm_dbg_info;

static int fops_open(struct inode *inode, struct file *file);
static int fops_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg);
static ssize_t fops_write(struct file *file, const char *buf,
			  size_t count, loff_t *ppos);
static ssize_t fops_read(struct file *file, char *buf,
			 size_t count, loff_t *ppos);


static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.ioctl = fops_ioctl,
	.open = fops_open,
	.read = fops_read,
	.write = fops_write,
};

static struct miscdevice pm_dbg_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pm_dbg",
	.fops = &fops,
};

static void data_read_regs(struct pm_dbg_data *data, struct pm_reg *regs)
{
	unsigned short temp1 = 0;
	unsigned short temp2 = 0;
	unsigned long temp = 0;

	cpcap_regacc_read(data->cpcap, CPCAP_REG_CCM, &temp1);
	if (temp1 < 0x200)
		regs->offset = temp1;
	else
		regs->offset = (short)(temp1 | 0xFC00);

	cpcap_regacc_read(data->cpcap, CPCAP_REG_CCS2, &temp2);
	cpcap_regacc_read(data->cpcap, CPCAP_REG_CCS1, &temp1);
	temp = (temp1 | (temp2 << 16)) & 0x0FFFFFF;
	regs->sample = (signed long)temp;

	cpcap_regacc_read(data->cpcap, CPCAP_REG_CCA2, &temp2);
	cpcap_regacc_read(data->cpcap, CPCAP_REG_CCA1, &temp1);
	temp = temp1 | (temp2 << 16);
	regs->accumulator = (signed long)temp;
}

static unsigned long data_get_time_ms(void)
{
	unsigned long long time;
	time = sched_clock();
	do_div(time, NSEC_PER_MSEC);
	return (unsigned long)time;
}

static int data_get_avg_curr_ua(struct pm_dbg_data *data,
			long long acc, long sample)
{
	unsigned long long temp;
	int avg_current;
	unsigned long int cc_lsb;
	unsigned short temp1;
	short offset;

	if (sample == 0) {
		cpcap_regacc_read(data->cpcap, CPCAP_REG_CCI, &temp1);
		if (data->cpcap->vendor == CPCAP_VENDOR_TI) {
			if (temp1 > 0x2000)
				temp1 = temp1 | 0xC000;
			sample = 1;
		} else
			sample = 4;
		acc = (short)temp1;
		cpcap_regacc_read(data->cpcap, CPCAP_REG_CCM, &temp1);
		if (temp1 < 0x200)
			offset = temp1;
		else
			offset = (short)(temp1 | 0xFC00);
		acc = acc - sample * offset;
	}

	if (data->cpcap->vendor == CPCAP_VENDOR_ST) {
		cc_lsb = CC_LSB_ST;
		cc_lsb = (cc_lsb * data->cd_factor)/1000;
	} else {
		cc_lsb = CC_LSB_TI;
		cc_lsb = (cc_lsb * data->cd_factor)/1000;
	}

	if (acc >= 0) {
		temp = acc;
		temp = temp * cc_lsb;
		do_div(temp, (sample * CC_DUR));
		avg_current = (long)temp;
	} else {
		temp = (unsigned long long)(-acc);
		temp = temp * cc_lsb;
		do_div(temp, (sample * CC_DUR));
		avg_current = -(long)temp;
	}
	return -avg_current;
}

static void data_update(struct pm_dbg_data *data, enum update_mode mode)
{
	short offset;
	unsigned short temp1;
	long long acc;
	long accumulator;
	long sample;
	unsigned short result[PM_DBG_RECORD_LENGTH] = {0};
	short cc_int;
	int retval;
	int test = 0;
	int test1 = 0;
	struct pm_reg pm_reg;
	static unsigned char start_ind;

	switch (mode) {
	case ACTIVE_START:
		currmeas_data.time_start = data_get_time_ms();

		data_read_regs(data, &pm_reg);

		currmeas_data.sample_start = pm_reg.sample;
		currmeas_data.acc_start = pm_reg.accumulator;

		currmeas_data.acc_active = 0;
		currmeas_data.sample_active = 0;

		currmeas_data.time_susp_rpt = 0;
		currmeas_data.time_res_rpt = 0;
		currmeas_data.avg_curr_susp = 0;
		currmeas_data.min_curr_susp = 0;

		currmeas_data.acc_susp = 0;

		start_ind = 1;
		break;

	case ACTIVE_STOP:
		currmeas_data.time_stop = data_get_time_ms();

	case ACTIVE_UPDATE:
		data_read_regs(data, &pm_reg);
		if (pm_reg.sample < currmeas_data.sample_start) {
			acc = pm_reg.accumulator -
				(pm_reg.sample * pm_reg.offset);
		} else {
			acc = (pm_reg.accumulator - currmeas_data.acc_start) -
				((pm_reg.sample - currmeas_data.sample_start)
				 * pm_reg.offset);
		}
		currmeas_data.acc_start = pm_reg.accumulator;

		currmeas_data.acc_active += acc;
		currmeas_data.sample_active += (pm_reg.sample -
			 currmeas_data.sample_start);

		currmeas_data.sample_start = pm_reg.sample;

		break;

	case IDLE_START:
		if (start_ind != 0) {
			currmeas_data.time_susp = data_get_time_ms();
			data_read_regs(data, &pm_reg);
			currmeas_data.sample_susp = 0;
			currmeas_data.acc_susp = 0;
			currmeas_data.sample_start_susp = pm_reg.sample;
			currmeas_data.acc_start_susp = pm_reg.accumulator;
			start_ind = 0;
		}
		break;

	case IDLE_STOP:
		currmeas_data.time_res = data_get_time_ms();
		data_read_regs(data, &pm_reg);
		acc = (pm_reg.accumulator - currmeas_data.acc_start_susp) -
			((pm_reg.sample - currmeas_data.sample_start_susp)
			 * pm_reg.offset);
		currmeas_data.acc_susp += acc;
		currmeas_data.sample_susp += (pm_reg.sample -
			 currmeas_data.sample_start_susp);
		currmeas_data.acc_start_susp = pm_reg.accumulator;
		currmeas_data.sample_start_susp = pm_reg.sample;
		break;
	case IDLE_MACRO_START:
		currmeas_data.time_susp_mc = data_get_time_ms();

		if (data->cpcap->vendor == CPCAP_VENDOR_ST)
			retval = cpcap_regacc_write(data->cpcap, CPCAP_REG_MI2,
					(1 << CPCAP_MACRO_14),
					(1 << CPCAP_MACRO_14));
		else
			retval = cpcap_regacc_write(data->cpcap, CPCAP_REG_MIM1,
						    0, (1 << CPCAP_MACRO_14));
		if (retval != 0)
			printk(KERN_INFO "PM_DBG Enable PM Macro Failed\n");

		break;

	case IDLE_MACRO_STOP:
		currmeas_data.time_res_mc = data_get_time_ms();
		cpcap_regacc_read(data->cpcap, CPCAP_REG_CCM, &temp1);
		if (temp1 < 0x200)
			offset = temp1;
		else
			offset = (short)(temp1 | 0xFC00);

		if (data->cpcap->vendor == CPCAP_VENDOR_ST) {
			retval = cpcap_regacc_write(data->cpcap, CPCAP_REG_MI2,
					0, (1 << CPCAP_MACRO_14));
			retval |= cpcap_uc_ram_read(data->cpcap,
				PM_DBG_RECORD_ADDR_ST, PM_DBG_RECORD_LENGTH,
					&result[0]);
		} else {
			retval = cpcap_regacc_write(data->cpcap, CPCAP_REG_MIM1,
						    (1 << CPCAP_MACRO_14),
							(1 << CPCAP_MACRO_14));
			retval |= cpcap_uc_ram_read(data->cpcap,
				PM_DBG_RECORD_ADDR_TI, PM_DBG_RECORD_LENGTH,
					&result[0]);
		}

		if (retval != 0)
			printk(KERN_INFO "PM_DBG Suspend Macro Read Fails\n");
		else if ((currmeas_data.time_res_mc -
				currmeas_data.time_susp_mc) <= 1000)
			printk(KERN_INFO "PM_DBG Suspend too short 4 macro\n");
		else {
			if (data->cpcap->vendor == CPCAP_VENDOR_ST) {
				sample = result[2]<<16 | result[3];
				accumulator = (long)(result[0]<<16 | result[1]);

				accumulator = accumulator - offset * sample;
				test = data_get_avg_curr_ua(data, accumulator,
								sample);
				cc_int = (short)result[5];
				test1 = data_get_avg_curr_ua(data,
					(cc_int - offset * 4), 4);
			} else {
				sample = result[3]<<16 | result[2];
				accumulator = (long)(result[1]<<16 | result[0]);
				accumulator = accumulator - offset * sample;

				test = data_get_avg_curr_ua(data, accumulator,
					sample);
				if (result[5] > 0x2000)
					cc_int = (short)(result[5] | 0xC000);
				else
					cc_int = (short)result[5];
				test1 = data_get_avg_curr_ua(data,
					(cc_int - offset), 1);
			}

			if ((currmeas_data.time_res_rpt -
				currmeas_data.time_susp_rpt) <=
			    (currmeas_data.time_res_mc -
				currmeas_data.time_susp_mc)) {
				currmeas_data.time_res_rpt =
					currmeas_data.time_res_mc;
				currmeas_data.time_susp_rpt =
					currmeas_data.time_susp_mc;
				currmeas_data.avg_curr_susp = test;
				currmeas_data.min_curr_susp = test1;
			}
			printk(KERN_INFO
				"PM_DBG cd-susp-uc avg[%d],min[%d]\n",
				test, test1);
		}
		break;
	case PROFILE_START:
		data_read_regs(data, &pm_reg);
		currmeas_data.acc_pf_res = pm_reg.accumulator;
		currmeas_data.sample_pf_res = pm_reg.sample;
		break;
	case PROFILE_SUSPEND:
		data_read_regs(data, &pm_reg);
		sample = pm_reg.sample - currmeas_data.sample_pf_res;
		accumulator =
			(pm_reg.accumulator - currmeas_data.acc_pf_res) -
			(sample * pm_reg.offset);
		currmeas_data.pf_act =
			data_get_avg_curr_ua(data, accumulator, sample);
		currmeas_data.sample_pf_susp = pm_reg.sample;
		currmeas_data.acc_pf_susp = pm_reg.accumulator;
		break;
	case PROFILE_RESUME:
		data_read_regs(data, &pm_reg);
		sample = pm_reg.sample - currmeas_data.sample_pf_res;
		accumulator =
			(pm_reg.accumulator - currmeas_data.acc_pf_res) -
			(sample * pm_reg.offset);
		currmeas_data.pf_avg =
			data_get_avg_curr_ua(data, accumulator, sample);
		sample = pm_reg.sample - currmeas_data.sample_pf_susp;
		accumulator =
			(pm_reg.accumulator - currmeas_data.acc_pf_susp) -
			(sample * pm_reg.offset);
		currmeas_data.pf_susp =
			data_get_avg_curr_ua(data, accumulator, sample);
		currmeas_data.acc_pf_res = pm_reg.accumulator;
		currmeas_data.sample_pf_res = pm_reg.sample;

		printk(KERN_INFO
			"PM_DBG cd-prof act[%d],susp[%d],avg[%d]\n",
			currmeas_data.pf_act,
			currmeas_data.pf_susp,
			currmeas_data.pf_avg);
		break;

	default:
		break;
	}
}

static void pm_dbg_work(struct work_struct *work)
{
	struct pm_dbg_data *data =
		container_of(work, struct pm_dbg_data, work.work);
}

static ssize_t fops_write(struct file *file, const char *buf,
			  size_t count, loff_t *ppos)
{
	ssize_t retval = -EINVAL;
	return retval;
}

static ssize_t fops_read(struct file *file, char *buf,
			 size_t count, loff_t *ppos)
{
	ssize_t retval = -EFAULT;
	return retval;
}

static int fops_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	int retval = -ENOTTY;
	struct pm_dbg_data *data = file->private_data;
	struct pm_dbg_currmeas_req act_req;
	struct pm_dbg_currmeas_suspend_sts_req suspend_req;
	unsigned char usrcmd;

	switch (cmd) {
	case PM_DBG_IOCTL_CURRMEAS_START:
		if (data->active_meas_enable != 0) {
			act_req.time_start = currmeas_data.time_start;
			data_update(data, ACTIVE_UPDATE);

			act_req.curr_drain =
				data_get_avg_curr_ua(data,
					currmeas_data.acc_active,
					currmeas_data.sample_active);

			data_update(data, ACTIVE_START);
			act_req.time_stop = currmeas_data.time_start;

			retval = copy_to_user((struct pm_dbg_currmeas_req *)arg,
					&(act_req),
					sizeof(struct pm_dbg_currmeas_req));
		}
		break;

	case PM_DBG_IOCTL_CURRMEAS_STOP:
		if (data->active_meas_enable != 0) {
			data_update(data, ACTIVE_STOP);
			act_req.time_start = currmeas_data.time_start;
			act_req.time_stop = currmeas_data.time_stop;

			act_req.curr_drain =
				data_get_avg_curr_ua(data,
					currmeas_data.acc_active,
					currmeas_data.sample_active);

			retval = copy_to_user((struct pm_dbg_currmeas_req *)arg,
					&(act_req),
					sizeof(struct pm_dbg_currmeas_req));
		}

		break;

	case PM_DBG_IOCTL_CURRMEAS_CMD:
		if (copy_from_user((void *) &usrcmd, (void *) arg,
				   sizeof(usrcmd)))
			return -EFAULT;

		if (usrcmd & 0x01) {
			if (data->active_meas_enable == 0) {
				data->active_meas_enable = 1;
				data_update(data, ACTIVE_START);
				cpcap_irq_unmask(data->cpcap,
					 CPCAP_IRQ_UC_PRIMACRO_14);
			}
		}
		if (usrcmd & 0x02)
			data->suspend_meas_enable = 1;
		if (usrcmd & 0x04)
			data->macro_meas_enable = 1;
		if (usrcmd & 0x08) {
			data->profile_enable = 1;
			data_update(data, PROFILE_START);
		}
		if (usrcmd == 0) {
			data->suspend_meas_enable = 0;
			data->active_meas_enable = 0;
			data->macro_meas_enable = 0;
			data->profile_enable = 0;
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_14);
		}
		retval = 0;
		break;

	case PM_DBG_IOCTL_CURRMEAS_SUSPEND_STS:
		if (data->suspend_meas_enable != 0) {
			suspend_req.time_suspend = currmeas_data.time_susp;
			suspend_req.time_resume = currmeas_data.time_res;
			suspend_req.avg_curr_drain =
				data_get_avg_curr_ua(data,
					currmeas_data.acc_susp,
					currmeas_data.sample_susp);
			suspend_req.min_curr_drain =
				currmeas_data.avg_curr_susp;
			suspend_req.retention_failure = 0;
			suspend_req.time_retention = 0;

			retval = copy_to_user(
				(struct pm_dbg_currmeas_suspend_sts_req *)arg,
				&(suspend_req),
				sizeof(struct pm_dbg_currmeas_suspend_sts_req));
		}

		break;

	default:
		break;
	}

	return retval;
}

static int fops_open(struct inode *inode, struct file *file)
{
	int retval = -ENOTTY;

	if (pm_dbg_info->is_supported)
		retval = 0;

	file->private_data = pm_dbg_info;
	dev_info(&pm_dbg_info->cpcap->spi->dev, "PM DBG: open status:%d\n",
		 retval);

	return retval;
}

static void pm_dbg_macro_isr(enum cpcap_irqs irq, void *data)
{
	struct pm_dbg_data *pm_data = data;

	if (irq != CPCAP_IRQ_UC_PRIMACRO_14)
		return;
	if (pm_data->active_meas_enable != 0) {
		data_update(pm_data, ACTIVE_UPDATE);
		cpcap_irq_unmask(pm_data->cpcap, CPCAP_IRQ_UC_PRIMACRO_14);
	}
}

static ssize_t
pm_dbg_control(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct pm_dbg_data *data = dev_get_drvdata(dev);
	char ctrl = *buf;

	switch (ctrl) {
	case '1':
		data->active_meas_enable = 1;
		cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_14);
		break;
	case '2':
		data->suspend_meas_enable = 1;
		break;
	case '3':
		data->macro_meas_enable = 1;
		break;
	case '4':
		data->profile_enable = 1;
		data_update(data, PROFILE_START);
		break;
	case '0':
		data->suspend_meas_enable = 0;
		data->active_meas_enable = 0;
		data->macro_meas_enable = 0;
		data->profile_enable = 0;
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_14);
		break;
	default:
		break;
	}
	return 0;
}

static ssize_t
pm_dbg_control_show(struct device *dev, struct device_attribute *attr,
	      char *buf)
{
	struct pm_dbg_data *data = dev_get_drvdata(dev);

	return sprintf(buf,
			"1-active = %s\n"
			"2-suspend = %s\n"
			"3-macro = %s\n"
			"4-profile = %s\n",
			(data->active_meas_enable) ? "enabled" : "disabled",
			(data->suspend_meas_enable) ? "enabled" : "disabled",
			(data->macro_meas_enable) ? "enabled" : "disabled",
			(data->profile_enable) ? "enabled" : "disabled");
}

static DEVICE_ATTR(control, 0644, pm_dbg_control_show, pm_dbg_control);

static ssize_t
pm_dbg_factor_show(struct device *dev, struct device_attribute *attr,
	      char *buf)
{
	struct pm_dbg_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->cd_factor);
}

static DEVICE_ATTR(factor, 0644, pm_dbg_factor_show, NULL);

static int pm_dbg_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct pm_dbg_data *data;
	struct pm_dbg_drvdata *drvdata = platform_get_drvdata(pdev);

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cpcap = pdev->dev.platform_data;
	data->is_supported = 0;
	data->suspend_meas_enable = 0;
	data->active_meas_enable = 0;
	data->macro_meas_enable = 0;
	data->profile_enable = 0;

	data->cd_factor = drvdata->pm_cd_factor;

	INIT_DELAYED_WORK(&data->work, pm_dbg_work);

	mutex_init(&data->lock);
	platform_set_drvdata(pdev, data);
	pm_dbg_info = data;

	retval = cpcap_irq_register(data->cpcap,
				    CPCAP_IRQ_UC_PRIMACRO_14,
				    pm_dbg_macro_isr, data);
	cpcap_irq_mask(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_14);

	if (retval)
		goto err_irq;

	retval = misc_register(&pm_dbg_dev);
	if (retval)
		goto err_dev;

	retval = device_create_file(&pdev->dev,
				 &dev_attr_control);
	if (retval)
		goto err_devfile;
	retval = device_create_file(&pdev->dev,
				 &dev_attr_factor);
	if (retval)
		goto err_devfile_factor;
	dev_set_drvdata(&pdev->dev, data);

	data->is_supported = 1;
	return retval;

err_devfile_factor:
	device_remove_file(&pdev->dev, &dev_attr_factor);
err_devfile:
	device_remove_file(&pdev->dev, &dev_attr_control);
err_dev:
	misc_deregister(&pm_dbg_dev);
err_irq:
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_14);

	kfree(data);

	return retval;
}

static int __exit pm_dbg_remove(struct platform_device *pdev)
{
	struct pm_dbg_data *data = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_factor);
	device_remove_file(&pdev->dev, &dev_attr_control);
	misc_deregister(&pm_dbg_dev);
	pm_dbg_info = NULL;
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_UC_PRIMACRO_14);

	kfree(data);
	return 0;
}

static int pm_dbg_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct pm_dbg_data *data = platform_get_drvdata(pdev);

	if (data->macro_meas_enable != 0)
		data_update(data, IDLE_MACRO_START);
	if (data->suspend_meas_enable != 0)
		data_update(data, IDLE_START);
	if (data->profile_enable != 0)
		data_update(data, PROFILE_SUSPEND);

	cpcap_irq_pm_dbg_suspend();
	return 0;
}

static int pm_dbg_resume(struct platform_device *pdev)
{
	struct pm_dbg_data *data = platform_get_drvdata(pdev);

	if (data->suspend_meas_enable != 0)
		data_update(data, IDLE_STOP);
	if (data->macro_meas_enable != 0)
		data_update(data, IDLE_MACRO_STOP);
	if (data->profile_enable != 0)
		data_update(data, PROFILE_RESUME);

	cpcap_irq_pm_dbg_resume();
	return 0;
}


static struct platform_driver pm_dbg_driver = {
	.probe		= pm_dbg_probe,
	.remove		= __exit_p(pm_dbg_remove),
	.suspend	= pm_dbg_suspend,
	.resume		= pm_dbg_resume,
	.driver		= {
		.name	= "cpcap_pm_dbg",
		.owner	= THIS_MODULE,
	},
};

static int __init pm_dbg_init(void)
{
	return platform_driver_register(&pm_dbg_driver);
}
subsys_initcall(pm_dbg_init);

static void __exit pm_dbg_exit(void)
{
	platform_driver_unregister(&pm_dbg_driver);
}
module_exit(pm_dbg_exit);

MODULE_ALIAS("platform:cpcap_pm_dbg");
MODULE_DESCRIPTION("PM Debug driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

