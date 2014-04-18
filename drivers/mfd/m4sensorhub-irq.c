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

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <linux/m4sensorhub.h>

#ifdef CONFIG_PM_DEEPSLEEP
#include <linux/suspend.h>
#endif

#define NUM_INT_REGS      3
#define NUM_INTS_PER_REG  8
#define NUM_INTS_LAST_REG (((M4SH_IRQ__NUM-1)%NUM_INTS_PER_REG)+1)
#define INTR_VALID_BITS(n) (unsigned char)((1 << (n)) - 1)

#define EVENT_MASK(event) (1 << ((event) % NUM_INTS_PER_REG))

#define DBG_BUF_LINE_LEN  80

/* --------------- Global Declarations -------------- */

/* ------------ Local Function Prototypes ----------- */
static int m4sensorhub_irq_disable_all(struct m4sensorhub_data *m4sensorhub);
static unsigned short get_enable_reg(enum m4sensorhub_irqs event);
static void irq_work_func(struct work_struct *work);
#ifdef CONFIG_DEBUG_FS
static int m4sensorhub_dbg_irq_open(struct inode *inode, struct file *file);
#endif
static void m4sensorhub_irq_restore(struct m4sensorhub_data *m4sensorhub,\
					void *data);

/* ---------------- Local Declarations -------------- */

static const char *irq_name[] = {
	[M4SH_IRQ_TMP_DATA_READY]         = "TMP_DATA_READY",
	[M4SH_IRQ_PRESSURE_DATA_READY]	  = "PRES_DATA_READY",
	[M4SH_IRQ_GYRO_DATA_READY]	  = "GYRO_DATA_READY",
	[M4SH_IRQ_PEDOMETER_DATA_READY]   = "PEDO_DATA_READY",
	[M4SH_IRQ_COMPASS_DATA_READY]	  = "COMPASS_DATA_READY",
	[M4SH_IRQ_FUSION_DATA_READY]	  = "FUSION_DATA_READY",
	[M4SH_IRQ_ACCEL_DATA_READY]	  = "ACCEL_DATA_READY",
	[M4SH_IRQ_GESTURE_DETECTED]       = "GESTURE_DETECTED",
	[M4SH_IRQ_STILL_DETECTED]         = "STILL_DETECTED",
	[M4SH_IRQ_MOTION_DETECTED]        = "MOTION_DETECTED",
	[M4SH_IRQ_ACTIVITY_CHANGE]        = "ACTIVITY_CHANGE",
	[M4SH_IRQ_DLCMD_RESP_READY]       = "DLCMD_RESP_READY",
	[M4SH_IRQ_MIC_DATA_READY]         = "MIC_DATA_READY",
	[M4SH_IRQ_WRIST_READY]            = "WRIST_READY",
	[M4SH_IRQ_PASSIVE_BUFFER_FULL]    = "PASSIVE_BUFFER_FULL",
	[M4SH_IRQ_LIGHTSENSOR_DATA_READY] = "ALS_DATA_READY",
	[M4SH_IRQ_HRSENSOR_DATA_READY]    = "HR_DATA_READY",
	[M4SH_IRQ_AP_ALARM_EXPIRED]       = "AP_ALARM_EXPIRED",
};

/* -------------- Local Data Structures ------------- */

struct m4sensorhub_event_handler {
	void (*func)(enum m4sensorhub_irqs, void *);
	void *data;
};

struct m4sensorhub_irq_info {
	uint8_t registered;
	uint8_t enabled;
	uint32_t ena_fired;
	uint32_t disa_fired;
};

struct m4sensorhub_irqdata {
	struct mutex lock;           /* lock event handlers and data */
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct m4sensorhub_data *m4sensorhub;
	struct m4sensorhub_event_handler event_handler[M4SH_IRQ__NUM];
	struct m4sensorhub_irq_info irq_info[M4SH_IRQ__NUM];
	struct wake_lock wake_lock;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs;
#endif
};


static const struct {
	enum m4sensorhub_reg status_reg;
	enum m4sensorhub_reg enable_reg;
	unsigned char valid_bits;
} int_registers[NUM_INT_REGS] = {
	{M4SH_REG_GENERAL_INTERRUPT0STATUS,
	 M4SH_REG_GENERAL_INTERRUPT0ENABLE,
	 INTR_VALID_BITS(NUM_INTS_PER_REG)},
	{M4SH_REG_GENERAL_INTERRUPT1STATUS,
	M4SH_REG_GENERAL_INTERRUPT1ENABLE,
	INTR_VALID_BITS(NUM_INTS_PER_REG)},
	{M4SH_REG_GENERAL_INTERRUPT2STATUS,
	M4SH_REG_GENERAL_INTERRUPT2ENABLE,
	INTR_VALID_BITS(NUM_INTS_LAST_REG)},
};

static irqreturn_t event_isr(int irq, void *data)
{
	/* Interrupts are left enabled; if multiple interrupts arrive, there
	 * will be multiple jobs in the workqueue.  In this case, the first
	 * job in the workqueue may service multple interrupts and
	 * susbsequent jobs will have no interrupts left to service.
	 */
	struct m4sensorhub_irqdata *irq_data = data;
	wake_lock(&irq_data->wake_lock);
	queue_work(irq_data->workqueue, &irq_data->work);

	return IRQ_HANDLED;
}

static struct mrsensorhub_irq_dbg {
	unsigned short en_ints[NUM_INT_REGS];
	unsigned char suspend;
	unsigned char wakeup;
} irq_dbg_info;

#ifdef CONFIG_DEBUG_FS
static const struct file_operations debug_fops = {
	.open    = m4sensorhub_dbg_irq_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};
#endif

/* -------------- Global Functions ----------------- */

/* m4sensorhub_irq_init()

   Intialize M4 sensor hub IRQ subsystem

   Returns 0 on success.  Returns negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
*/

int m4sensorhub_irq_init(struct m4sensorhub_data *m4sensorhub)
{
	int retval;
	struct i2c_client *i2c = m4sensorhub->i2c_client;
	struct m4sensorhub_irqdata *data;

	data = kzalloc(sizeof(struct m4sensorhub_irqdata), GFP_KERNEL);
	if (!data) {
		KDEBUG(M4SH_ERROR, "m4sensorhub: Memory error in irq_init\n");
		retval = -ENOMEM;
		goto done;
	}
	KDEBUG(
		M4SH_INFO,
		"m4sensorhub: %u IRQs with valid_bits %02X%02X%02X\n",
		M4SH_IRQ__NUM, int_registers[2].valid_bits,
		int_registers[1].valid_bits, int_registers[0].valid_bits
		);
	retval = m4sensorhub_irq_disable_all(m4sensorhub);
	if (retval) {
		KDEBUG(M4SH_ERROR, "m4sensorhub: Failed disable all irqs\n");
		goto err_free;
	}

	data->workqueue = create_workqueue("m4sensorhub_irq");
	if (data->workqueue == NULL) {
		KDEBUG(M4SH_ERROR, "m4sensorhub: IRQ Workqueue init failure\n");
		retval = -ENOMEM;
		goto err_free;
	}
	INIT_WORK(&data->work, irq_work_func);

	mutex_init(&data->lock);

	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "m4sensorhub-irq");

	retval = request_irq(i2c->irq, event_isr, IRQF_DISABLED |
				IRQF_TRIGGER_RISING, "m4sensorhub-irq", data);
	if (retval) {
		KDEBUG(M4SH_ERROR, "m4sensorhub: Failed requesting irq.\n");
		goto err_destroy_wq;
	}

	data->m4sensorhub = m4sensorhub;
	m4sensorhub->irqdata = data;

	retval = enable_irq_wake(i2c->irq);
	if (retval) {
		KDEBUG(M4SH_ERROR, "m4sensorhub: Failed enabling irq wake.\n");
		goto err_free_irq;
	}

#ifdef CONFIG_DEBUG_FS
	data->debugfs = debugfs_create_file("m4sensorhub-irq", S_IRUGO, NULL,
					    data, &debug_fops);
	if (data->debugfs == NULL) {
		KDEBUG(M4SH_ERROR, "m4sensorhub: Error creating debufs\n");
		retval = -EINVAL;
		goto err_disa_irq;
	}
#endif
	m4sensorhub_panic_register(m4sensorhub, PANICHDL_IRQ_RESTORE,\
					m4sensorhub_irq_restore, data);
	KDEBUG(M4SH_INFO, "m4sensorhub IRQ subsystem initialized\n");
	retval = 0;
	goto done;

#ifdef CONFIG_DEBUG_FS
err_disa_irq:
#endif
	disable_irq_wake(i2c->irq);
err_free_irq:
	free_irq(i2c->irq, data);
	m4sensorhub->irqdata = NULL;
	data->m4sensorhub = NULL;
err_destroy_wq:
	wake_lock_destroy(&data->wake_lock);
	mutex_destroy(&data->lock);
	destroy_workqueue(data->workqueue);
err_free:
	kfree(data);
done:
	return retval;
}
EXPORT_SYMBOL_GPL(m4sensorhub_irq_init);

/* m4sensorhub_irq_shutdown()

   Shutdown the M4 sensor hub IRQ subsystem

     m4sensorhub - pointer to the main m4sensorhub data struct
*/
void m4sensorhub_irq_shutdown(struct m4sensorhub_data *m4sensorhub)
{
	struct i2c_client *i2c = m4sensorhub->i2c_client;
	struct m4sensorhub_irqdata *data = m4sensorhub->irqdata;

	KDEBUG(M4SH_INFO, "shutdown m4sensorhub IRQ subsystem\n");

	m4sensorhub_panic_unregister(m4sensorhub, PANICHDL_IRQ_RESTORE);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(data->debugfs);
#endif

	disable_irq_wake(i2c->irq);
	free_irq(i2c->irq, data);

	m4sensorhub->irqdata = NULL;
	data->m4sensorhub = NULL;

	if (wake_lock_active(&data->wake_lock))
		wake_unlock(&data->wake_lock);
	wake_lock_destroy(&data->wake_lock);

	if (mutex_is_locked(&data->lock))
		mutex_unlock(&data->lock);
	mutex_destroy(&data->lock);

	cancel_work_sync(&data->work);
	destroy_workqueue(data->workqueue);

	kfree(data);
}
EXPORT_SYMBOL_GPL(m4sensorhub_irq_shutdown);

/* m4sensorhub_irq_register()

   Register an interupt handler in the M4 Sensor Hub IRQ subsystem.
   This does not enable the IRQ, that needs to be done by caller
   with m4sensorhub_irq_enable()

   Returns 0 on success.  Returns negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
     irq - M4 Sensor Hub interupt to resiter for
     cb_func - IRQ handler function to execute on inturrupt
     data - pointer to data for IRQ handler function
*/

int m4sensorhub_irq_register(struct m4sensorhub_data *m4sensorhub,
			     enum m4sensorhub_irqs irq,
			     void (*cb_func) (enum m4sensorhub_irqs, void *),
			     void *data)
{
	struct m4sensorhub_irqdata *irqdata;
	int retval = 0;

	if ((!m4sensorhub) || (irq >= M4SH_IRQ__NUM) || (!cb_func))
		return -EINVAL;

	irqdata = m4sensorhub->irqdata;

	if (irqdata == NULL) {
		KDEBUG(M4SH_ERROR, "irqdata null for caller = %d\n", irq);
		return -EINVAL;
	}

	mutex_lock(&irqdata->lock);

	if (irqdata->event_handler[irq].func == NULL) {
		irqdata->irq_info[irq].registered = 1;
		irqdata->event_handler[irq].func = cb_func;
		irqdata->event_handler[irq].data = data;
		KDEBUG(M4SH_NOTICE, "m4sensorhub: %s IRQ registered\n",
			irq_name[irq]);
	} else {
		KDEBUG(M4SH_ERROR, "m4sensorhub: %s IRQ registration failed\n",
			irq_name[irq]);
		retval = -EPERM;
	}

	mutex_unlock(&irqdata->lock);

	return retval;
}
EXPORT_SYMBOL_GPL(m4sensorhub_irq_register);

/* m4sensorhub_irq_unregister()

   Unregister an interupt handler in the M4 Sensor Hub IRQ subsystem

   Returns 0 on success.  Returns negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
     irq - M4 Sensor Hub interupt to unresiter for
*/
int m4sensorhub_irq_unregister(struct m4sensorhub_data *m4sensorhub,
			       enum m4sensorhub_irqs irq)
{
	struct m4sensorhub_irqdata *data = m4sensorhub->irqdata;
	int retval;

	if ((irq >= M4SH_IRQ__NUM) || (m4sensorhub == NULL) ||
			(m4sensorhub->irqdata == NULL))
		return -EINVAL;

	retval = m4sensorhub_irq_disable(m4sensorhub, irq);

	mutex_lock(&data->lock);
	data->event_handler[irq].func = NULL;
	data->event_handler[irq].data = NULL;
	data->irq_info[irq].registered = 0;
	mutex_unlock(&data->lock);

	KDEBUG(M4SH_NOTICE, "m4sensorhub: %s IRQ un-registered\n",
			    irq_name[irq]);

	return retval;
}
EXPORT_SYMBOL_GPL(m4sensorhub_irq_unregister);

/* m4sensorhub_irq_enable_get()

   Check if an IRQ is enabled

   Returns 1 if enabled, 0 if disabled.
   Returns negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
     irq - M4 Sensor Hub interupt to check
*/

int m4sensorhub_irq_enable_get(struct m4sensorhub_data *m4sensorhub,
		       enum m4sensorhub_irqs irq)
{
	struct m4sensorhub_irqdata *data = m4sensorhub->irqdata;
	int retval = -EINVAL;

	if (irq < M4SH_IRQ__NUM)
		return data->irq_info[irq].enabled;

	return retval;
}
EXPORT_SYMBOL_GPL(m4sensorhub_irq_enable_get);

/* m4sensorhub_irq_disable()

   Disable M4 Sensor Hub subsystem IRQ

   Returns 0 on success.  Returns negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
     irq - M4 Sensor Hub interupt to disable
*/

int m4sensorhub_irq_disable(struct m4sensorhub_data *m4sensorhub,
		   enum m4sensorhub_irqs irq)
{
	struct m4sensorhub_irqdata *data = m4sensorhub->irqdata;
	int retval = -EINVAL;

	if (irq < M4SH_IRQ__NUM) {
		mutex_lock(&data->lock);
		data->irq_info[irq].enabled = 0;
		mutex_unlock(&data->lock);
		retval = m4sensorhub_reg_write_1byte(m4sensorhub,
				get_enable_reg(irq), 0, EVENT_MASK(irq));
		retval = CHECK_REG_ACCESS_RETVAL(m4sensorhub, retval,
						 get_enable_reg(irq));
	}

	return retval;
}
EXPORT_SYMBOL_GPL(m4sensorhub_irq_disable);

/* m4sensorhub_irq_enable()

   Enable M4 Sensor Hub subsystem IRQ

   Returns 0 on success.  Returns negative error code on failure

     m4sensorhub - pointer to the main m4sensorhub data struct
     irq - M4 Sensor Hub interupt to enable
*/

int m4sensorhub_irq_enable(struct m4sensorhub_data *m4sensorhub,
		     enum m4sensorhub_irqs irq)
{
	struct m4sensorhub_irqdata *data = m4sensorhub->irqdata;
	int retval = -EINVAL;

	if (irq < M4SH_IRQ__NUM) {
		mutex_lock(&data->lock);
		data->irq_info[irq].enabled = 1;
		mutex_unlock(&data->lock);
		retval = m4sensorhub_reg_write_1byte(m4sensorhub,
				get_enable_reg(irq), EVENT_MASK(irq),
				EVENT_MASK(irq));
		retval = CHECK_REG_ACCESS_RETVAL(m4sensorhub, retval,
						 get_enable_reg(irq));
	}

	return retval;
}
EXPORT_SYMBOL_GPL(m4sensorhub_irq_enable);

/* m4sensorhub_irq_pm_suspend()

   Called by core to track suspend state and wakeup cause

*/

void m4sensorhub_irq_pm_dbg_suspend(void)
{
	irq_dbg_info.suspend = 1;
	irq_dbg_info.wakeup = 0;
}
EXPORT_SYMBOL_GPL(m4sensorhub_irq_pm_dbg_suspend);

/* m4sensorhub_irq_pm_resume()

   Called by core to print interupt source on M4SH wakeup

*/

void m4sensorhub_irq_pm_dbg_resume(void)
{
	char buffer[DBG_BUF_LINE_LEN];
	int i;

	irq_dbg_info.suspend = 0;
	if ((irq_dbg_info.wakeup != 0) && (m4sensorhub_debug >= M4SH_NOTICE)) {
		strcpy(buffer, "M4 Sensor Hub IRQ registers:");
		for (i = 0; (i < NUM_INT_REGS) &&
			    (strlen(buffer) < DBG_BUF_LINE_LEN-5); ++i) {
			sprintf(&buffer[strlen(buffer)], " 0x%02x",
				irq_dbg_info.en_ints[i]);
		}

		KDEBUG(M4SH_NOTICE, "newbuf: %s\n", buffer);

		/* Decode the bits */
		KDEBUG(M4SH_NOTICE, "M4 Sensor Hub IRQ sources:\n");
		for (i = 0; i < NUM_INT_REGS; ++i) {
			unsigned char index;

			while (irq_dbg_info.en_ints[i] > 0) {
				/* find the first set bit */
				index = (unsigned char)
					(ffs(irq_dbg_info.en_ints[i]) - 1);
				if (index >= M4SH_IRQ__NUM)
					goto error;

				/* clear the bit */
				irq_dbg_info.en_ints[i] &= ~(1 << index);
				/* find the event that occurred */
				index += M4SH_IRQ__START +
					 (i * NUM_INTS_PER_REG);
				if (index >= M4SH_IRQ__NUM)
					goto error;

				KDEBUG(M4SH_NOTICE, "\t%s\n", irq_name[index]);
			}
		}
	}
error:
	return;
}
EXPORT_SYMBOL_GPL(m4sensorhub_irq_pm_dbg_resume);

/* --------------- Local Functions ----------------- */

static unsigned short get_enable_reg(enum m4sensorhub_irqs event)
{
	unsigned short ret;

	if ((event) >= M4SH_IRQ__NUM)
		ret = M4SH_REG__INVALID;
	else if ((event) >= M4SH_IRQ_INT2_INDEX)
		ret = M4SH_REG_GENERAL_INTERRUPT2ENABLE;
	else if ((event) >= M4SH_IRQ_INT1_INDEX)
		ret = M4SH_REG_GENERAL_INTERRUPT1ENABLE;
	else if ((event) >= M4SH_IRQ_INT0_INDEX)
		ret = M4SH_REG_GENERAL_INTERRUPT0ENABLE;
	else
		ret = M4SH_REG__INVALID;

	return ret;
}

static int m4sensorhub_irq_disable_all(struct m4sensorhub_data *m4sensorhub)
{
	int i;

	for (i = 0; i < NUM_INT_REGS; i++) {
		if (1 != m4sensorhub_reg_write_1byte(m4sensorhub,
				int_registers[i].enable_reg, 0,
				int_registers[i].valid_bits)) {
			KDEBUG(M4SH_ERROR, "m4sensorhub_irq: "
					"Failed disabling INT%d\n", i);
			return -EFAULT;
		}
	}
	return 0;
}

static void irq_work_func(struct work_struct *work)
{
	unsigned short en_ints[NUM_INT_REGS] = { 0 };
	int i;
	struct m4sensorhub_irqdata *data;
	struct m4sensorhub_data *m4sensorhub;
	struct i2c_client *i2c;
	unsigned char value, is_irq_set = 0;

	data = container_of(work, struct m4sensorhub_irqdata, work);
	m4sensorhub = data->m4sensorhub;
	i2c = m4sensorhub->i2c_client;

	for (i = 0; i < NUM_INT_REGS; ++i) {
		/* M4 is expected to clear these bits when read */
		if (1 != m4sensorhub_reg_read(m4sensorhub,
			int_registers[i].status_reg, &value)) {
			dev_err(&m4sensorhub->i2c_client->dev,
				"Error reading INT%d\n", i);
			goto error;
		}
		en_ints[i] = value;
		is_irq_set |= value;
	}

	if (!is_irq_set) {
		/* Got the checkpoint to check if M4 panicked */
		m4sensorhub_panic_process(m4sensorhub);
		goto error;
	}

	if ((irq_dbg_info.suspend != 0) && (irq_dbg_info.wakeup == 0)) {
		for (i = 0; i < NUM_INT_REGS; ++i)
			irq_dbg_info.en_ints[i] = en_ints[i];
		irq_dbg_info.wakeup = 1;
	}

	for (i = 0; i < NUM_INT_REGS; ++i) {
		unsigned char index;

		while (en_ints[i] > 0) {
			struct m4sensorhub_event_handler *event_handler;

			/* find the first set bit */
			index = (unsigned char)(ffs(en_ints[i]) - 1);
			if (index >= M4SH_IRQ__NUM)
				goto error;
			/* clear the bit */
			en_ints[i] &= ~(1 << index);
			/* find the event that occurred */
			index += M4SH_IRQ__START + (i * NUM_INTS_PER_REG);
			if (index >= M4SH_IRQ__NUM)
				goto error;

			if (data->irq_info[index].enabled) {
				event_handler = &data->event_handler[index];

				if (event_handler && event_handler->func)
					event_handler->func(index,
							   event_handler->data);

				mutex_lock(&data->lock);
				data->irq_info[index].ena_fired++;
				mutex_unlock(&data->lock);
			} else {
				mutex_lock(&data->lock);
				data->irq_info[index].disa_fired++;
				mutex_unlock(&data->lock);
			}
		}
	}
error:
	wake_unlock(&data->wake_lock);
}

#ifdef CONFIG_DEBUG_FS
static int m4sensorhub_dbg_irq_show(struct seq_file *s, void *data)
{
	unsigned int i;
	struct m4sensorhub_irqdata *irqdata = s->private;

	seq_printf(s, "%21s%9s%12s%15s%16s\n",
		   "M4SENSORHUB IRQ", "Enabled", "Registered",
		   "Fired Enabled", "Fired Disabled");

	for (i = 0; i < M4SH_IRQ__NUM; i++) {
		seq_printf(s, "%21s%9d%12d%15d%16d\n",
			   irq_name[i],
			   irqdata->irq_info[i].enabled,
			   irqdata->irq_info[i].registered,
			   irqdata->irq_info[i].ena_fired,
			   irqdata->irq_info[i].disa_fired);
	}
	return 0;
}

static int m4sensorhub_dbg_irq_open(struct inode *inode, struct file *file)
{
	return single_open(file, m4sensorhub_dbg_irq_show, inode->i_private);
}
#endif

/* m4sensorhub_irq_restore()

   Callback Handler is called by Panic after M4 has been restarted

*/
static void m4sensorhub_irq_restore(\
	struct m4sensorhub_data *m4sensorhub, void *data)
{
	int i;
	unsigned short en_ints[NUM_INT_REGS] = {0};

	mutex_lock(&((struct m4sensorhub_irqdata *)data)->lock);
	for (i = 0; i < M4SH_IRQ__NUM; i++) {
		if (!((struct m4sensorhub_irqdata *)data)->irq_info[i].enabled)
			continue;
		en_ints[i/NUM_INTS_PER_REG] |= EVENT_MASK(i);
	}
	mutex_unlock(&((struct m4sensorhub_irqdata *)data)->lock);

	for (i = 0; i < NUM_INT_REGS; i++) {
		KDEBUG(M4SH_INFO, "m4sensorhub_irq: Reseting INT%d-%02X\n",\
					i, en_ints[i]);
		if (1 != m4sensorhub_reg_write_1byte(m4sensorhub,
				int_registers[i].enable_reg, en_ints[i],
				int_registers[i].valid_bits)) {
			KDEBUG(M4SH_ERROR, "m4sensorhub_irq: "
					"Failed reseting INT%d\n", i);
		}
	}
}
