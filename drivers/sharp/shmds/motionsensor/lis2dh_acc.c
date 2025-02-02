/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
 *
 * File Name          : lis2dh_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Denis Ciocca (carmine.iascone@st.com)
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.12
 * Date               : 2012/Feb/29
 * Description        : LIS2DH accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 ******************************************************************************
 Revision 1.0.6 15/11/2010
  first revision
  supports sysfs;
  no more support for ioctl;
 Revision 1.0.7 26/11/2010
  checks for availability of interrupts pins
  correction on FUZZ and FLAT values;
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7
 Revision 1.0.9: 2011/May/23
  update_odr func correction;
 Revision 1.0.10: 2011/Aug/16
  introduces default_platform_data, i2c_read and i2c_write function rewritten,
  manages smbus beside i2c
 Revision 1.0.11: 2012/Jan/09
  moved under input/misc
 Revision 1.0.12: 2012/Feb/29
  moved use_smbus inside status struct; modified:-update_fs_range;-set_range
  input format; allows gpio_intX to be passed as parameter at insmod time;
 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include	<linux/kernel.h>
#include	<linux/device.h>
#include	<linux/module.h>
#include	<linux/moduleparam.h>

#include	<sharp/lis2dh.h>
#ifdef CONFIG_SHTERM
#include <sharp/shterm_k.h>
#endif /* CONFIG_SHTERM */

/* SHMDS_ID_0107_01 add S */
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
/* SHMDS_ID_0107_01 add E */
#include <linux/pm_qos.h>    /* SHMDS_ID_0205_01 add */

#define	DEBUG		1 

#define	G_MAX		16000

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/

/* Accelerometer Sensor Operating Mode */
#define LIS2DH_ACC_ENABLE	(0x01)
#define LIS2DH_ACC_DISABLE	(0x00)

#define	HIGH_RESOLUTION		(0x08)

#define	AXISDATA_REG		(0x28)
#define WHOAMI_LIS2DH_ACC	(0x33)	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		(0x0F)	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		(0x1F)	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		(0x20)	/*	control reg 1		*/
#define	CTRL_REG2		(0x21)	/*	control reg 2		*/
#define	CTRL_REG3		(0x22)	/*	control reg 3		*/
#define	CTRL_REG4		(0x23)	/*	control reg 4		*/
#define	CTRL_REG5		(0x24)	/*	control reg 5		*/
#define	CTRL_REG6		(0x25)	/*	control reg 6		*/
#define REFERENCE		(0x26)	/*	REFERENCE reg		*/

#define	FIFO_CTRL_REG		(0x2E)	/*	FiFo control reg	*/

#define	INT_CFG1		(0x30)	/*	interrupt 1 config	*/
#define	INT_SRC1		(0x31)	/*	interrupt 1 source	*/
#define	INT_THS1		(0x32)	/*	interrupt 1 threshold	*/
#define	INT_DUR1		(0x33)	/*	interrupt 1 duration	*/

#define	TT_CFG			(0x38)	/*	tap config		*/
#define	TT_SRC			(0x39)	/*	tap source		*/
#define	TT_THS			(0x3A)	/*	tap threshold		*/
#define	TT_LIM			(0x3B)	/*	tap time limit		*/
#define	TT_TLAT			(0x3C)	/*	tap time latency	*/
#define	TT_TW			(0x3D)	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/

#define ENABLE_HIGH_RESOLUTION	1
#define ALL_ZEROES		(0x00)

#define LIS2DH_ACC_PM_OFF		(0x00)
#define LIS2DH_ACC_ENABLE_ALL_AXES	(0x07)
#define LIS2DH_ACC_ENABLE_IS1_HPF	(0x31)


#define PMODE_MASK		(0x08)
#define ODR_MASK		(0XF0)

#define LIS2DH_ACC_ODR1	(0x10)  /* 1Hz output data rate */
#define LIS2DH_ACC_ODR10	(0x20)  /* 10Hz output data rate */
#define LIS2DH_ACC_ODR25	(0x30)  /* 25Hz output data rate */
#define LIS2DH_ACC_ODR50	(0x40)  /* 50Hz output data rate */
#define LIS2DH_ACC_ODR100	(0x50)  /* 100Hz output data rate */
#define LIS2DH_ACC_ODR200	(0x60)  /* 200Hz output data rate */
#define LIS2DH_ACC_ODR400	(0x70)  /* 400Hz output data rate */
#define LIS2DH_ACC_ODR1250	(0x90)  /* 1250Hz output data rate */

#define	IA			(0x40)
#define	ZH			(0x20)
#define	ZL			(0x10)
#define	YH			(0x08)
#define	YL			(0x04)
#define	XH			(0x02)
#define	XL			(0x01)
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	(0x40)
#define	CTRL_REG4_BDU_ENABLE	(0x80)
#define	CTRL_REG4_BDU_MASK	(0x80)
#define	CTRL_REG6_I2_TAPEN	(0x80)
#define	CTRL_REG6_HLACTIVE	(0x02)
/* */
#define NO_MASK			(0xFF)
#define INT1_DURATION_MASK	(0x7F)
#define	INT1_THRESHOLD_MASK	(0x7F)
#define TAP_CFG_MASK		(0x3F)
#define	TAP_THS_MASK		(0x7F)
#define	TAP_TLIM_MASK		(0x7F)
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			(0x20)
#define	STAP			(0x10)
#define	SIGNTAP			(0x08)
#define	ZTAP			(0x04)
#define	YTAP			(0x02)
#define	XTAZ			(0x01)


#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	(0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define	RESUME_ENTRIES		17
/* end RESUME STATE INDICES */

/* SHMDS_ID_0107_01 add S */
#define SHMDS_GPIO_INT0_NAME  "shmds_hostif_int0"
#define SHMDS_RC_OK           (0)
/* SHMDS_ID_0107_01 add E */

#ifdef SHMDS_DETECT
int shmds_ths_value = 6;
int shmds_duration_value = 0x01;
static unsigned long shmds_count = 0;
static int shmds_buf_size = 34;
int shmds_buf_size_temp = 34;
int shmds_md_thresh = 100;
static int shmds_detect_switch = DETECT_OFF;
static signed short *shmds_detect_buf[3];
static signed short shmds_economize_buf[3] = {0, 0, 0};
atomic_t shmds_detect_mode_flg = ATOMIC_INIT(SHMDS_DETECT_NORMAL);
int shmds_full_buffer_flg = 0;
static signed short shmds_tmp_buf[3][SHMDS_TMP_BUFSIZE];
static signed short center_data[3];
unsigned int shmds_normal_interval = 200;
unsigned int shmds_economize_interval = 5000;
#endif /* SHMDS_DETECT */

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis2dh_acc_odr_table[] = {
		{    1, LIS2DH_ACC_ODR1250 },
		{    3, LIS2DH_ACC_ODR400  },
		{    5, LIS2DH_ACC_ODR200  },
		{   10, LIS2DH_ACC_ODR100  },
		{   20, LIS2DH_ACC_ODR50   },
		{   40, LIS2DH_ACC_ODR25   },
		{  100, LIS2DH_ACC_ODR10   },
		{ 1000, LIS2DH_ACC_ODR1    },
};

static int int1_gpio = LIS2DH_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = LIS2DH_ACC_DEFAULT_INT2_GPIO;
module_param(int1_gpio, int, S_IRUGO);
module_param(int2_gpio, int, S_IRUGO);

/* SHMDS_ID_0107_01 add S */
static struct i2c_client *client_mcu;
static unsigned int shmds_gpio_no_int0 = 0;
/* SHMDS_ID_0107_01 add E */

struct lis2dh_acc_status {
	struct i2c_client *client;
	struct lis2dh_acc_platform_data *pdata;

	struct mutex lock;
//	struct delayed_work input_work;
	struct work_struct input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	int use_smbus;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	struct hrtimer acc_timer;
#ifdef DEBUG
	u8 reg_addr;
#endif
};

/* SHMDS_ID_0205_01 add S */
#define SHMDS_PM_QOS_LATENCY_VALUE 1
static void shmds_qos_init(void);
static void shmds_qos_destroy(void);
void shmds_qos_start(void);
void shmds_qos_end(void);
static struct pm_qos_request shmds_qos_cpu_dma_latency;
static DEFINE_MUTEX(qosMutex);
static int shmds_qos_num = 0;
/* SHMDS_ID_0205_01 add E */

static struct lis2dh_acc_platform_data default_lis2dh_acc_pdata = {
	
	.fs_range = LIS2DH_ACC_G_4G,  /* SHMDS_ID_0111_01 mod ( 2G->4G ) */
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 1,
	.negate_y = 0,
	.negate_z = 1,
	.poll_interval = 20,	//.poll_interval = 100,
	.min_interval = LIS2DH_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = LIS2DH_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = LIS2DH_ACC_DEFAULT_INT2_GPIO,
};

static int lis2dh_acc_i2c_read(struct lis2dh_acc_status *stat, u8 *buf,
									int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;
        unsigned int ii;
/*
	if (len > sizeof(buf))
			dev_err(&stat->client->dev,
				"read error insufficient buffer length: "
				"len:%d, buf size=%d\n",
				len, sizeof(buf));
*/
	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | reg);
	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(stat->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
				"command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(stat->client,
								cmd, len, buf);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
				"command=0x%02x, ",
				ret, len, cmd);
			for (ii = 0; ii < len; ii++)
				printk(KERN_DEBUG "buf[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
		} else
			ret = -1;

		if (ret < 0) {
			dev_err(&stat->client->dev,
				"read transfer error: len:%d, command=0x%02x\n",
				len, cmd);
			return 0; /* failure */
		}
		return len; /* success */
	}

	ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(stat->client, buf, len);
}

static int lis2dh_acc_i2c_write(struct lis2dh_acc_status *stat, u8 *buf,
									int len)
{
	int ret;
	u8 reg, value;
        unsigned int ii;

	if (len > 1)
		buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

	reg = buf[0];
	value = buf[1];

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(stat->client,
								reg, value);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(stat->client,
							reg, len, buf + 1);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_i2c_block_data: ret=%d, "
				"len:%d, command=0x%02x, ",
				ret, len, reg);

			for (ii = 0; ii < (len + 1); ii++)
				printk(KERN_DEBUG "value[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
			return ret;
		}
	}

	ret = i2c_master_send(stat->client, buf, len+1);
	return (ret == len+1) ? 0 : ret;
}

static int lis2dh_acc_hw_init(struct lis2dh_acc_status *stat)
{
	int err = -1;
	u8 buf[7];

//	pr_info("%s: hw init start\n", LIS2DH_ACC_DEV_NAME);    /* SHMDS_ID_0202_01 del */

	buf[0] = WHO_AM_I;
	err = lis2dh_acc_i2c_read(stat, buf, 1);
	if (err < 0) {
		dev_warn(&stat->client->dev, "Error reading WHO_AM_I:"
				" is device available/working?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf[0] != WHOAMI_LIS2DH_ACC) {
		dev_err(&stat->client->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n",
			WHOAMI_LIS2DH_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}


	buf[0] = CTRL_REG1;
	buf[1] = stat->resume_state[RES_CTRL_REG1];
	err = lis2dh_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TEMP_CFG_REG;
	buf[1] = stat->resume_state[RES_TEMP_CFG_REG];
	err = lis2dh_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = FIFO_CTRL_REG;
	buf[1] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = lis2dh_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TT_THS;
	buf[1] = stat->resume_state[RES_TT_THS];
	buf[2] = stat->resume_state[RES_TT_LIM];
	buf[3] = stat->resume_state[RES_TT_TLAT];
	buf[4] = stat->resume_state[RES_TT_TW];
	err = lis2dh_acc_i2c_write(stat, buf, 4);
	if (err < 0)
		goto err_resume_state;
	buf[0] = TT_CFG;
	buf[1] = stat->resume_state[RES_TT_CFG];
	err = lis2dh_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = INT_THS1;
	buf[1] = stat->resume_state[RES_INT_THS1];
	buf[2] = stat->resume_state[RES_INT_DUR1];
	err = lis2dh_acc_i2c_write(stat, buf, 2);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = stat->resume_state[RES_INT_CFG1];
	err = lis2dh_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = CTRL_REG2;
	buf[1] = stat->resume_state[RES_CTRL_REG2];
	buf[2] = stat->resume_state[RES_CTRL_REG3];
	buf[3] = stat->resume_state[RES_CTRL_REG4];
	buf[4] = stat->resume_state[RES_CTRL_REG5];
	buf[5] = stat->resume_state[RES_CTRL_REG6];
	err = lis2dh_acc_i2c_write(stat, buf, 5);
	if (err < 0)
		goto err_resume_state;

	stat->hw_initialized = 1;
//	pr_info("%s: hw init done\n", LIS2DH_ACC_DEV_NAME);     /* SHMDS_ID_0202_01 del */
	return 0;

err_firstread:
	stat->hw_working = 0;
err_unknown_device:
err_resume_state:
	stat->hw_initialized = 0;
	dev_err(&stat->client->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

/* SHMDS_ID_0112_01 add S */
static int lis2dh_acc_device_low_power(struct lis2dh_acc_status *stat)
{
	int err = -1;
	u8 buf[7];

	buf[0] = 0x1E;
	buf[1] = 0x90;
	err = lis2dh_acc_i2c_write(stat, buf, 1);
	if (err < 0)
	{
		return err;
	}

	return 0;
}
/* SHMDS_ID_0112_01 add E */

static void lis2dh_acc_device_power_off(struct lis2dh_acc_status *stat)
{
	int err;
	u8 buf[2] = { CTRL_REG1, LIS2DH_ACC_PM_OFF };

	err = lis2dh_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "soft power off failed: %d\n", err);

	buf[0] = CTRL_REG3;
	buf[1] = 0x00;
	err = lis2dh_acc_i2c_write(stat, buf, 1);
	if (err) {
		pr_err("%s|%d returning %d\n", __func__, __LINE__, err);
	}

	if (stat->pdata->power_off) {
		if (stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			disable_irq_nosync(stat->irq2);
		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}
	if (stat->hw_initialized) {
		if (stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			disable_irq_nosync(stat->irq2);
		stat->hw_initialized = 0;
	}

}

static int lis2dh_acc_device_power_on(struct lis2dh_acc_status *stat)
{
	int err = -1;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0) {
			dev_err(&stat->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
		if (stat->pdata->gpio_int1 >= 0)
			enable_irq(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			enable_irq(stat->irq2);
	}

	if (!stat->hw_initialized) {
		err = lis2dh_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lis2dh_acc_device_power_off(stat);
			return err;
		}
	}

	if (stat->hw_initialized) {
		if (stat->pdata->gpio_int1 >= 0)
			enable_irq(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			enable_irq(stat->irq2);
	}
	return 0;
}

static void lis2dh_acc_regset_md_disable(struct lis2dh_acc_status *stat)
{
	int result = 0;
	u8 buf[2];

	buf[0] = CTRL_REG3;
	buf[1] = 0x00;
	result = lis2dh_acc_i2c_write(stat, buf, 1);
	if (result) {
		pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
	}

	buf[0] = INT_CFG1;
	buf[1] = 0x00;
	result = lis2dh_acc_i2c_write(stat, buf, 1);
	if (result) {
		pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
	}
}

static irqreturn_t lis2dh_acc_isr1(int irq, void *dev)
{
	struct lis2dh_acc_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq1_work_queue, &stat->irq1_work);
	pr_debug("%s: isr1 queued\n", LIS2DH_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t lis2dh_acc_isr2(int irq, void *dev)
{
	struct lis2dh_acc_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq2_work_queue, &stat->irq2_work);
	pr_debug("%s: isr2 queued\n", LIS2DH_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void lis2dh_acc_irq1_work_func(struct work_struct *work)
{

	struct lis2dh_acc_status *stat =
	container_of(work, struct lis2dh_acc_status, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lis2dh_acc_get_int1_source(stat); */

//	cancel_delayed_work_sync(&stat->input_work);
//	schedule_delayed_work(&stat->input_work,
//			msecs_to_jiffies(0));

	stat->pdata->poll_interval = shmds_normal_interval;
	hrtimer_cancel(&stat->acc_timer);
	cancel_work_sync(&stat->input_work);
	schedule_work(&stat->input_work);
	hrtimer_start(&stat->acc_timer, ns_to_ktime((u64)stat->pdata->poll_interval * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	pr_debug("%s: IRQ1 triggered\n", LIS2DH_ACC_DEV_NAME);
/* exit: */
	enable_irq(stat->irq1);
}

static void lis2dh_acc_irq2_work_func(struct work_struct *work)
{

	struct lis2dh_acc_status *stat =
	container_of(work, struct lis2dh_acc_status, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lis2dh_acc_get_tap_source(stat); */

	/* ; */

	pr_debug("%s: IRQ2 triggered\n", LIS2DH_ACC_DEV_NAME);
/* exit: */
	enable_irq(stat->irq2);
}

static enum hrtimer_restart acc_timer_func(struct hrtimer *timer)
{
	struct lis2dh_acc_status *stat = container_of(timer,
		struct lis2dh_acc_status, acc_timer);

	schedule_work(&stat->input_work);

	hrtimer_forward_now(timer, ns_to_ktime((u64)stat->pdata->poll_interval * NSEC_PER_MSEC));

	return HRTIMER_RESTART;
}

static int lis2dh_acc_update_fs_range(struct lis2dh_acc_status *stat,
							u8 new_fs_range)
{
	int err = -1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LIS2DH_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_fs_range) {
	case LIS2DH_ACC_G_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case LIS2DH_ACC_G_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case LIS2DH_ACC_G_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	case LIS2DH_ACC_G_16G:

		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&stat->client->dev, "invalid fs range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}


	/* Updates configuration register 4,
	* which contains fs range setting */
	buf[0] = CTRL_REG4;
	err = lis2dh_acc_i2c_read(stat, buf, 1);
	if (err < 0)
		goto error;
	init_val = buf[0];
	stat->resume_state[RES_CTRL_REG4] = init_val;
	new_val = new_fs_range | HIGH_RESOLUTION;
	updated_val = ((mask & new_val) | ((~mask) & init_val));
	buf[1] = updated_val;
	buf[0] = CTRL_REG4;
	err = lis2dh_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;
	stat->resume_state[RES_CTRL_REG4] = updated_val;
	stat->sensitivity = sensitivity;

	return err;
error:
	dev_err(&stat->client->dev,
			"update fs range failed 0x%02x,0x%02x: %d\n",
			buf[0], buf[1], err);

	return err;
}

static int lis2dh_acc_update_odr(struct lis2dh_acc_status *stat,
							int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis2dh_acc_odr_table) - 1; i >= 0; i--) {
		if ((lis2dh_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
								|| (i == 0))
			break;
	}
	config[1] = lis2dh_acc_odr_table[i].mask;

	config[1] |= LIS2DH_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		config[0] = CTRL_REG1;
		err = lis2dh_acc_i2c_write(stat, config, 1);
		if (err < 0)
			goto error;
		stat->resume_state[RES_CTRL_REG1] = config[1];
	}

	return err;

error:
	dev_err(&stat->client->dev, "update odr failed 0x%02x,0x%02x: %d\n",
			config[0], config[1], err);

	return err;
}



static int lis2dh_acc_register_write(struct lis2dh_acc_status *stat,
					u8 *buf, u8 reg_address, u8 new_value)
{
	int err = -1;

		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = lis2dh_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;
	return err;
}

/*
static int lis2dh_acc_register_read(struct lis2dh_acc_status *stat,
							u8 *buf, u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = lis2dh_acc_i2c_read(stat, buf, 1);
	return err;
}
*/

/*
static int lis2dh_acc_register_update(struct lis2dh_acc_status *stat,
		u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = lis2dh_acc_register_read(stat, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = lis2dh_acc_register_write(stat, buf, reg_address,
				updated_val);
	}
	return err;
}
*/

static void shmds_get_center_data( signed short *c_data )
{
	int				i,j,k;
	signed short	CustodyData;
	signed short	buff[3][SHMDS_TMP_BUFSIZE];

	memcpy( buff, shmds_tmp_buf, sizeof(shmds_tmp_buf) );

	for ( i=0; i<3; i++ ){
		for ( j=0; j<SHMDS_TMP_BUFSIZE-1; j++ ){
			for ( k=0; k<( SHMDS_TMP_BUFSIZE-1-j ); k++ ){
				if ( buff[i][k] > buff[i][k+1] ){
					CustodyData = buff[i][k];
					buff[i][k] = buff[i][k+1];
					buff[i][k+1] = CustodyData;
				}
			}
		}
	}

	for ( i=0; i<3; i++ ){
		c_data[i] = buff[i][SHMDS_TMP_BUFSIZE/2];
	}
	return;
}

static int lis2dh_acc_get_acceleration_data(
				struct lis2dh_acc_status *stat, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };
	
	struct timespec ts;    /* SHMDS_ID_0205_02 add */

	acc_data[0] = (AXISDATA_REG);
	err = lis2dh_acc_i2c_read(stat, acc_data, 6);
	if (err < 0)
		return err;
	
	ktime_get_ts(&ts);            /* SHMDS_ID_0205_02 add */
    monotonic_to_bootbased(&ts);  /* SHMDS_ID_0205_02 add */

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] = hw_d[0] * stat->sensitivity;
	hw_d[1] = hw_d[1] * stat->sensitivity;
	hw_d[2] = hw_d[2] * stat->sensitivity;

	xyz[0] = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
		   : (hw_d[stat->pdata->axis_map_x]));
	xyz[1] = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
		   : (hw_d[stat->pdata->axis_map_y]));
	xyz[2] = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
		   : (hw_d[stat->pdata->axis_map_z]));
	xyz[3] = ts.tv_sec;    /* SHMDS_ID_0205_02 add */
	xyz[4] = ts.tv_nsec;   /* SHMDS_ID_0205_02 add */

#ifdef DEBUG
/*
	pr_debug("%s read x=%d, y=%d, z=%d\n",
			LIS2DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
*/
#endif
	return err;
}

static void lis2dh_acc_report_values(struct lis2dh_acc_status *stat,
					int *xyz)
{
#ifdef SHMDS_DETECT
	int i,j;
	signed long sub[3] = {0,0,0};
	signed short max, min;
	short acclx = 0, accly = 0, acclz = 0;
	int result;
	u8 buf[7];
#endif /* SHMDS_DETECT */

/* SHMDS_ID_0301_01 add S */
	if(xyz[0] == 0){
		stat->input_dev->absinfo[ABS_X].value = 1;
	}else{
		stat->input_dev->absinfo[ABS_X].value = 0;
	}
/* SHMDS_ID_0301_01 add E */
	input_report_abs(stat->input_dev, ABS_X, xyz[0]);
	input_report_abs(stat->input_dev, ABS_Y, xyz[1]);
	input_report_abs(stat->input_dev, ABS_Z, xyz[2]);
	input_report_abs(stat->input_dev, ABS_MISC, xyz[3]);    /* SHMDS_ID_0205_02 add */
    input_report_abs(stat->input_dev, ABS_VOLUME, xyz[4]);  /* SHMDS_ID_0205_02 add */
	input_sync(stat->input_dev);

#ifdef SHMDS_DETECT
	acclx = xyz[0];
	accly = xyz[1];
	acclz = xyz[2];

	if (shmds_buf_size != shmds_buf_size_temp){
		for (i = 0; i < 3; i++) {
			kfree(shmds_detect_buf[i]);
			shmds_detect_buf[i] = NULL;
		}

		for (i = 0; i < 3; i++) {
			shmds_detect_buf[i] = (signed short *)kmalloc(shmds_buf_size_temp * sizeof(signed short), GFP_KERNEL);
			if (NULL == shmds_detect_buf[i]) {
				for (j = 0; j < i; j ++) {
					kfree(shmds_detect_buf[j]);
				}
				pr_err("shmds_detect_buf is NULL");
			}
			memset(shmds_detect_buf[i], 0, shmds_buf_size_temp * sizeof(signed short));
		}
		shmds_buf_size = shmds_buf_size_temp;
	}

	if (shmds_detect_switch == DETECT_ON) {

		if ( atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_NORMAL ) {
			if (( !shmds_count ) && ( !shmds_full_buffer_flg )){
				for ( i=0; i<SHMDS_TMP_BUFSIZE; i++ ){
					shmds_tmp_buf[0][i] = acclx;
					shmds_tmp_buf[1][i] = accly;
					shmds_tmp_buf[2][i] = acclz;
				}
			} else {
				for ( i=0; i<( SHMDS_TMP_BUFSIZE-1 ); i++ ){
					shmds_tmp_buf[0][i] = shmds_tmp_buf[0][i+1];
					shmds_tmp_buf[1][i] = shmds_tmp_buf[1][i+1];
					shmds_tmp_buf[2][i] = shmds_tmp_buf[2][i+1];
				}
				shmds_tmp_buf[0][SHMDS_TMP_BUFSIZE-1] = acclx;
				shmds_tmp_buf[1][SHMDS_TMP_BUFSIZE-1] = accly;
				shmds_tmp_buf[2][SHMDS_TMP_BUFSIZE-1] = acclz;
			}

			shmds_get_center_data( center_data );

			shmds_detect_buf[0][shmds_count] = center_data[0];
			shmds_detect_buf[1][shmds_count] = center_data[1];
			shmds_detect_buf[2][shmds_count] = center_data[2];

			if ( (shmds_count+1) == shmds_buf_size ) {
				shmds_full_buffer_flg = 1;
			}

			if ( shmds_full_buffer_flg ) {

				for (i = 0; i < 3; i++) {
					max = -32768;
					min = 32767;

					for (j = 0; j < shmds_buf_size; j++) {
						if (shmds_detect_buf[i][j] > max)
							max = shmds_detect_buf[i][j];
						if (shmds_detect_buf[i][j] < min)
							min = shmds_detect_buf[i][j];
					}
					sub[i] = max - min;
				}
				if ((sub[0] <= shmds_md_thresh) && (sub[1] <= shmds_md_thresh) && (sub[2] <= shmds_md_thresh)) {

					buf[0] = INT_THS1;
					buf[1] = shmds_ths_value;
					result = lis2dh_acc_i2c_write(stat, buf, 1);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
					}

					buf[0] = INT_CFG1;
					buf[1] = (XH | YH | ZH);
					result = lis2dh_acc_i2c_write(stat, buf, 1);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
					}

					buf[0] = CTRL_REG3;
					buf[1] = CTRL_REG3_I1_AOI1;
					result = lis2dh_acc_i2c_write(stat, buf, 1);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
					}

					buf[0] = INT_DUR1;
					buf[1] = shmds_duration_value;
					result = lis2dh_acc_i2c_write(stat, buf, 1);
					if (result) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
					}

					buf[0] = REFERENCE;
					result = lis2dh_acc_i2c_read(stat, buf, 1);
					if (result < 0) {
						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
					}

					atomic_set(&shmds_detect_mode_flg, SHMDS_DETECT_ECONOMIZE);
					shmds_economize_buf[0] = shmds_detect_buf[0][shmds_count];
					shmds_economize_buf[1] = shmds_detect_buf[1][shmds_count];
					shmds_economize_buf[2] = shmds_detect_buf[2][shmds_count];
					shmds_count = 0;
					shmds_full_buffer_flg = 0;
					stat->pdata->poll_interval = shmds_economize_interval;
					for (i = 0; i < 3; i++) {
						memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
					}
				}
			}
			shmds_count = (shmds_count+1) % shmds_buf_size;

		} else if (atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE) {

			buf[0] = INT_SRC1;
			result = lis2dh_acc_i2c_read(stat, buf, 1);
			if (result < 0) {
				pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
			}

			if( (((buf[0] & IA) != 0) && (buf[0] & (XH | YH | ZH)) != 0) ) {
//				buf[0] = CTRL_REG3;
//				buf[1] = 0x00;
//				result = lis2dh_acc_i2c_write(stat, buf, 1);
//				if (result) {
//					pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
//				}

//				buf[0] = INT_CFG1;
//				buf[1] = 0x00;
//				result = lis2dh_acc_i2c_write(stat, buf, 1);
//				if (result) {
//					pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
//				}

				lis2dh_acc_regset_md_disable(stat);
				stat->pdata->poll_interval = shmds_normal_interval;
				atomic_set(&shmds_detect_mode_flg,SHMDS_DETECT_NORMAL);
//				hrtimer_start(&stat->acc_timer, ns_to_ktime((u64)stat->pdata->poll_interval * NSEC_PER_MSEC), HRTIMER_MODE_REL);

			} else {

				sub[0] = acclx;
				sub[1] = accly;
				sub[2] = acclz;

				for (i = 0; i < 3; i++) {
					sub[i] = abs(sub[i] - shmds_economize_buf[i]);
				}
				if ((sub[0] > shmds_md_thresh) || (sub[1] > shmds_md_thresh) || (sub[2] > shmds_md_thresh)) {

//					buf[0] = CTRL_REG3;
//					buf[1] = 0x00;
//					result = lis2dh_acc_i2c_write(stat, buf, 1);
//					if (result) {
//						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
//					}

//					buf[0] = INT_CFG1;
//					buf[1] = 0x00;
//					result = lis2dh_acc_i2c_write(stat, buf, 1);
//					if (result) {
//						pr_err("%s|%d returning %d\n", __func__, __LINE__, result);
//					}

					lis2dh_acc_regset_md_disable(stat);
					stat->pdata->poll_interval = shmds_normal_interval;
					atomic_set(&shmds_detect_mode_flg,SHMDS_DETECT_NORMAL);
					hrtimer_cancel(&stat->acc_timer);
					hrtimer_start(&stat->acc_timer, ns_to_ktime((u64)stat->pdata->poll_interval * NSEC_PER_MSEC), HRTIMER_MODE_REL);
				}
			}
		}
	}
#endif /* SHMDS_DETECT */
}

static int lis2dh_acc_enable(struct lis2dh_acc_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = lis2dh_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
#ifdef CONFIG_SHTERM
		shterm_k_set_info(SHTERM_INFO_ACCELE, 1);
#endif
//		schedule_delayed_work(&stat->input_work,
//			msecs_to_jiffies(20));
		hrtimer_start(&stat->acc_timer, ns_to_ktime((u64)(20 * NSEC_PER_MSEC)), HRTIMER_MODE_REL);
	}

	return 0;
}

static int lis2dh_acc_disable(struct lis2dh_acc_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
//		cancel_delayed_work_sync(&stat->input_work);
		hrtimer_cancel(&stat->acc_timer);
		cancel_work_sync(&stat->input_work);
		lis2dh_acc_device_power_off(stat);
//		stat->pdata->poll_interval = 20;
#ifdef CONFIG_SHTERM
		shterm_k_set_info(SHTERM_INFO_ACCELE, 0);
#endif
	}

	return 0;
}

static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = lis2dh_acc_i2c_read(stat, &data, 1);
	if (err < 0)
		return err;
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;

}

static int write_reg(struct device *dev, const char *buf, u8 reg,
		u8 mask, int resumeIndex)
{
	int err = -1;
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lis2dh_acc_register_write(stat, x, reg, new_val);
	if (err < 0)
		return err;
	stat->resume_state[resumeIndex] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;
#ifdef SHMDS_DETECT
	int i;
//	int i, interval_val;
#endif /* SHMDS_DETECT */

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;

#ifdef SHMDS_DETECT
	if (atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE) {
		lis2dh_acc_regset_md_disable(stat);
	}
//	interval_val = 1000 / interval_ms;
//	if( interval_val == 1 || interval_val == 2 ) {
	if( interval_ms == 1000 || interval_ms == 500 ) {

//		if( interval_val == 1 ) {
		if( interval_ms == 1000 ) {
			interval_ms = SHMDS_DETECT_INTERVAL1;
			shmds_buf_size_temp = SHMDS_DETECT_BUFSIZE1;
		} else {
			interval_ms = SHMDS_DETECT_INTERVAL2;
			shmds_buf_size_temp = SHMDS_DETECT_BUFSIZE2;
		}
		shmds_detect_switch		= DETECT_ON;
		shmds_count				= 0;
		shmds_full_buffer_flg	= 0;
		shmds_normal_interval	= interval_ms;
		atomic_set(&shmds_detect_mode_flg, SHMDS_DETECT_NORMAL);
		if ( shmds_buf_size == shmds_buf_size_temp ) {
			for (i = 0; i < 3; i++) {
				memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
			}
		}
	} else {
//		if (atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE) {
//			lis2dh_acc_regset_md_disable(stat);
//		}
		shmds_detect_switch = DETECT_OFF;
		hrtimer_cancel(&stat->acc_timer);
		cancel_work_sync(&stat->input_work);
		hrtimer_start(&stat->acc_timer, ns_to_ktime((u64)(20 * NSEC_PER_MSEC)), HRTIMER_MODE_REL);
//		cancel_delayed_work_sync(&stat->input_work);
//		stat->pdata->poll_interval = 20;
//		schedule_delayed_work(&stat->input_work,
//				msecs_to_jiffies(0));
	}
#endif /* SHMDS_DETECT */

	interval_ms = max((unsigned int)interval_ms, stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	lis2dh_acc_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	char val;
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	char range = 2;
	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range ;
	switch (val) {
	case LIS2DH_ACC_G_2G:
		range = 2;
		break;
	case LIS2DH_ACC_G_4G:
		range = 4;
		break;
	case LIS2DH_ACC_G_8G:
		range = 8;
		break;
	case LIS2DH_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 2:
		range = LIS2DH_ACC_G_2G;
		break;
	case 4:
		range = LIS2DH_ACC_G_4G;
		break;
	case 8:
		range = LIS2DH_ACC_G_8G;
		break;
	case 16:
		range = LIS2DH_ACC_G_16G;
		break;
	default:
		dev_err(&stat->client->dev, "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = lis2dh_acc_update_fs_range(stat, range);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_info(&stat->client->dev, "range set to: %lu g\n", val);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lis2dh_acc_enable(stat);
	else
		lis2dh_acc_disable(stat);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_thresh1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS1, INT1_THRESHOLD_MASK, RES_INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}

static ssize_t attr_set_click_cfg(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_CFG, TAP_CFG_MASK, RES_TT_CFG);
}

static ssize_t attr_get_click_cfg(struct device *dev,
		struct device_attribute *attr,	char *buf)
{

	return read_single_reg(dev, buf, TT_CFG);
}

static ssize_t attr_get_click_source(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_SRC);
}

static ssize_t attr_set_click_ths(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_THS, TAP_THS_MASK, RES_TT_THS);
}

static ssize_t attr_get_click_ths(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_THS);
}

static ssize_t attr_set_click_tlim(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_LIM, TAP_TLIM_MASK, RES_TT_LIM);
}

static ssize_t attr_get_click_tlim(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_LIM);
}

static ssize_t attr_set_click_tlat(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TLAT_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tlat(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

static ssize_t attr_set_click_tw(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TW_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tw(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}
//----------------
s64 get_time_ns(void)
{
	struct timespec ts;
	ktime_get_ts(&ts);
	return timespec_to_ns(&ts);
}

static ssize_t attr_get_raw_accl(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	int xyz[3] = { 0 };
	int ret;

	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	ret = lis2dh_acc_get_acceleration_data(stat, xyz);
	if (ret < 0)
		dev_err(&stat->client->dev, "get_acceleration_data failed\n");
	else
		lis2dh_acc_report_values(stat, xyz);

	ret = sprintf(buf, "%d %d %d %lld\n",
			(signed short)xyz[0],
			(signed short)xyz[1],
			(signed short)xyz[2],
			get_time_ns());

	mutex_unlock(&stat->lock);
	printk("jiaolining: x=%d, y=%d, z=%d\n", xyz[0], xyz[1], xyz[2]);
	printk("jiaolining: raw_accl-->[%s]\n", buf);

	return ret;
}

//---------------


#ifdef DEBUG
/* PAY ATTENTION: These DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = val;
	rc = lis2dh_acc_i2c_write(stat, x, 1);
/* SHMDS_ID_0110_01 add S */
	if (rc < 0){
	    return rc;
	}
/* SHMDS_ID_0110_01 add E */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = lis2dh_acc_i2c_read(stat, &data, 1);
/* SHMDS_ID_0110_01 add S */
	if (rc < 0){
	    return rc;
	}
/* SHMDS_ID_0110_01 add E */
/* SHMDS_ID_0110_01 mod S */
//	ret = sprintf(buf, "0x%02x\n", data);
	ret = sprintf(buf, "%02x", data);
/* SHMDS_ID_0110_01 mod E */
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lis2dh_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	stat->reg_addr = val;
	mutex_unlock(&stat->lock);
	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0664, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0664, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(click_config, 0664, attr_get_click_cfg, attr_set_click_cfg),
	__ATTR(click_source, 0444, attr_get_click_source, NULL),
	__ATTR(click_threshold, 0664, attr_get_click_ths, attr_set_click_ths),
	__ATTR(click_timelimit, 0664, attr_get_click_tlim, attr_set_click_tlim),
	__ATTR(click_timelatency, 0664, attr_get_click_tlat,
							attr_set_click_tlat),
	__ATTR(click_timewindow, 0664, attr_get_click_tw, attr_set_click_tw),
//-------------------------
	__ATTR(diag_raw_accl, 0444, attr_get_raw_accl, NULL),
//-------------------------

#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}


static void lis2dh_acc_input_work_func(struct work_struct *work)
{
	struct lis2dh_acc_status *stat;

	int xyz[5] = { 0 };  /* SHMDS_ID_0205_02 mod */
	int err;

//	stat = container_of((struct delayed_work *)work,
//			struct lis2dh_acc_status, input_work);
	stat = container_of(work,
			struct lis2dh_acc_status, input_work);

	mutex_lock(&stat->lock);
	shmds_qos_start();    /* SHMDS_ID_0205_01 add */
	err = lis2dh_acc_get_acceleration_data(stat, xyz);
	if (err < 0)
		dev_err(&stat->client->dev, "get_acceleration_data failed\n");
	else
		lis2dh_acc_report_values(stat, xyz);

//	schedule_delayed_work(&stat->input_work, msecs_to_jiffies(
//			stat->pdata->poll_interval));
	shmds_qos_end();    /* SHMDS_ID_0205_01 add */
	mutex_unlock(&stat->lock);
}

int lis2dh_acc_input_open(struct input_dev *input)
{
/* SHMDS_ID_0204_01 mod S */
//	struct lis2dh_acc_status *stat = input_get_drvdata(input);

//	return lis2dh_acc_enable(stat);
	return 0;
/* SHMDS_ID_0204_01 mod E */
}

void lis2dh_acc_input_close(struct input_dev *dev)
{
	struct lis2dh_acc_status *stat = input_get_drvdata(dev);

	lis2dh_acc_disable(stat);
}

static int lis2dh_acc_validate_pdata(struct lis2dh_acc_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int)LIS2DH_ACC_MIN_POLL_PERIOD_MS,
						stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
			stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 ||
		stat->pdata->axis_map_y > 2 ||
		 stat->pdata->axis_map_z > 2) {
		dev_err(&stat->client->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", stat->pdata->axis_map_x,
					stat->pdata->axis_map_y,
						stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 || stat->pdata->negate_y > 1
			|| stat->pdata->negate_z > 1) {
		dev_err(&stat->client->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", stat->pdata->negate_x,
				stat->pdata->negate_y, stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(&stat->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lis2dh_acc_input_init(struct lis2dh_acc_status *stat)
{
	int err;

//	INIT_DELAYED_WORK(&stat->input_work, lis2dh_acc_input_work_func);
	INIT_WORK(&stat->input_work, lis2dh_acc_input_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev, "input device allocation failed\n");
		goto err0;
	}

	stat->input_dev->open = lis2dh_acc_input_open;
	stat->input_dev->close = lis2dh_acc_input_close;
	stat->input_dev->name = LIS2DH_ACC_DEV_NAME;
	/* stat->input_dev->name = "accelerometer"; */
	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, stat->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, stat->input_dev->absbit);

	input_set_abs_params(stat->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_WHEEL, INT_MIN,
								INT_MAX, 0, 0);
	input_set_abs_params(stat->input_dev, ABS_MISC, 0, 0xFFFFFFFF, 0, 0);     /* SHMDS_ID_0205_02 add */
    input_set_abs_params(stat->input_dev, ABS_VOLUME, 0, 0xFFFFFFFF, 0, 0);   /* SHMDS_ID_0205_02 add */

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(&stat->client->dev,
				"unable to register input device %s\n",
				stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);
err0:
	return err;
}

static void lis2dh_acc_input_cleanup(struct lis2dh_acc_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

static int lis2dh_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct lis2dh_acc_status *stat;

	u32 smbus_func = (I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK);

	int err = -1;
#ifdef SHMDS_DETECT
	int i, j;
#endif

	dev_info(&client->dev, "probe start.\n");

	stat = kzalloc(sizeof(struct lis2dh_acc_status), GFP_KERNEL);
	if (stat == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}

	/* Support for both I2C and SMBUS adapter interfaces. */
	stat->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			goto exit_check_functionality_failed;
		}
	}

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	stat->client = client;
	i2c_set_clientdata(client, stat);

	shmds_set_gpio_no(client); /* SHMDS_ID_0107_01 add */
	
	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}

	if (client->dev.platform_data == NULL) {
		default_lis2dh_acc_pdata.gpio_int1 = int1_gpio;
		default_lis2dh_acc_pdata.gpio_int2 = int2_gpio;
		memcpy(stat->pdata, &default_lis2dh_acc_pdata,
							sizeof(*stat->pdata));
		dev_info(&client->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, client->dev.platform_data,
							sizeof(*stat->pdata));
	}

	err = lis2dh_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}


	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

/* SHMDS_ID_0107_01 del S */
//	if (stat->pdata->gpio_int1 >= 0) {
//		stat->irq1 = gpio_to_irq(stat->pdata->gpio_int1);
//		pr_info("%s: %s has set irq1 to irq: %d, "
//							"mapped on gpio:%d\n",
//			LIS2DH_ACC_DEV_NAME, __func__, stat->irq1,
//							stat->pdata->gpio_int1);
//	}
/* SHMDS_ID_0107_01 del E */

	if (stat->pdata->gpio_int2 >= 0) {
		stat->irq2 = gpio_to_irq(stat->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d, "
							"mapped on gpio:%d\n",
			LIS2DH_ACC_DEV_NAME, __func__, stat->irq2,
							stat->pdata->gpio_int2);
	}
	
/* SHMDS_ID_0107_01 add S */
	shmds_gpio_init();
	stat->irq1 = g_nIntIrqNo;
/* SHMDS_ID_0107_01 add E */

#ifdef SHMDS_DETECT
	for (i = 0; i < 3; i++) {
		shmds_detect_buf[i] = (signed short *)kmalloc(shmds_buf_size * sizeof(signed short), GFP_KERNEL);
		if (NULL == shmds_detect_buf[i]) {
			for (j = 0; j < i; j ++) {
				kfree(shmds_detect_buf[j]);
			}
			return -1;
		}
		memset(shmds_detect_buf[i], 0, shmds_buf_size * sizeof(signed short));
	}
#endif /* SHMDS_DETECT */

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL_REG1] = (ALL_ZEROES |
					LIS2DH_ACC_ENABLE_ALL_AXES);
	stat->resume_state[RES_CTRL_REG4] = (ALL_ZEROES | CTRL_REG4_BDU_ENABLE);

	stat->resume_state[RES_CTRL_REG2] = (ALL_ZEROES |
				 	LIS2DH_ACC_ENABLE_IS1_HPF);
/*
	stat->resume_state[RES_CTRL_REG3] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG4] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG5] = ALL_ZEROES;
	stat->resume_state[RES_CTRL_REG6] = ALL_ZEROES;

	stat->resume_state[RES_TEMP_CFG_REG] = ALL_ZEROES;
	stat->resume_state[RES_FIFO_CTRL_REG] = ALL_ZEROES;
	stat->resume_state[RES_INT_CFG1] = ALL_ZEROES;
	stat->resume_state[RES_INT_THS1] = ALL_ZEROES;
	stat->resume_state[RES_INT_DUR1] = ALL_ZEROES;

	stat->resume_state[RES_TT_CFG] = ALL_ZEROES;
	stat->resume_state[RES_TT_THS] = ALL_ZEROES;
	stat->resume_state[RES_TT_LIM] = ALL_ZEROES;
	stat->resume_state[RES_TT_TLAT] = ALL_ZEROES;
	stat->resume_state[RES_TT_TW] = ALL_ZEROES;
*/

	err = lis2dh_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&stat->enabled, 1);

/* SHMDS_ID_0112_01 add S */
	err = lis2dh_acc_device_low_power(stat);
	if (err < 0) {
		dev_err(&client->dev, "low power failed: %d\n", err);
		goto err_power_off;
	}
/* SHMDS_ID_0112_01 add E */

	err = lis2dh_acc_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lis2dh_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lis2dh_acc_input_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
		   "device LIS2DH_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lis2dh_acc_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	hrtimer_init(&stat->acc_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->acc_timer.function = acc_timer_func;

	if (stat->pdata->gpio_int1 >= 0) {
		INIT_WORK(&stat->irq1_work, lis2dh_acc_irq1_work_func);
		stat->irq1_work_queue =
			create_singlethread_workqueue("lis2dh_acc_wq1");
		if (!stat->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(stat->irq1, lis2dh_acc_isr1,
			IRQF_TRIGGER_RISING, "lis2dh_acc_irq1", stat);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(stat->irq1);
	}

	if (stat->pdata->gpio_int2 >= 0) {
		INIT_WORK(&stat->irq2_work, lis2dh_acc_irq2_work_func);
		stat->irq2_work_queue =
			create_singlethread_workqueue("lis2dh_acc_wq2");
		if (!stat->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(stat->irq2, lis2dh_acc_isr2,
			IRQF_TRIGGER_RISING, "lis2dh_acc_irq2", stat);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(stat->irq2);
	}

	mutex_unlock(&stat->lock);

	dev_info(&client->dev, "%s: probed\n", LIS2DH_ACC_DEV_NAME);

	return 0;

err_destoyworkqueue2:
	if (stat->pdata->gpio_int2 >= 0)
		destroy_workqueue(stat->irq2_work_queue);
err_free_irq1:
	free_irq(stat->irq1, stat);
err_destoyworkqueue1:
	if (stat->pdata->gpio_int1 >= 0)
		destroy_workqueue(stat->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
	lis2dh_acc_input_cleanup(stat);
err_power_off:
	lis2dh_acc_device_power_off(stat);
err_pdata_init:
	if (stat->pdata->exit)
		stat->pdata->exit();
exit_kfree_pdata:
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);
/* err_freedata: */
	kfree(stat);
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LIS2DH_ACC_DEV_NAME);
	return err;
}

/* SHMDS_ID_0106_01 mod S */
//static int __devexit lis2dh_acc_remove(struct i2c_client *client)
static int lis2dh_acc_remove(struct i2c_client *client)
/* SHMDS_ID_0106_01 mod E */
{

	struct lis2dh_acc_status *stat = i2c_get_clientdata(client);
#ifdef SHMDS_DETECT
	int i;

	for (i = 0; i < 3; i++) {
		kfree(shmds_detect_buf[i]);
		shmds_detect_buf[i] = NULL;
	}
#endif /* SHMDS_DETECT */

	if (stat->pdata->gpio_int1 >= 0) {
		free_irq(stat->irq1, stat);
/* SHMDS_ID_0107_01 mod S */
//		gpio_free(stat->pdata->gpio_int1);
		shmds_gpio_free();
/* SHMDS_ID_0107_01 mod E */
		destroy_workqueue(stat->irq1_work_queue);
	}

	if (stat->pdata->gpio_int2 >= 0) {
		free_irq(stat->irq2, stat);
		gpio_free(stat->pdata->gpio_int2);
		destroy_workqueue(stat->irq2_work_queue);
	}

	lis2dh_acc_input_cleanup(stat);
	lis2dh_acc_device_power_off(stat);
	remove_sysfs_interfaces(&client->dev);

	if (stat->pdata->exit)
		stat->pdata->exit();
	kfree(stat->pdata);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM
static int lis2dh_acc_resume(struct i2c_client *client)
{
	struct lis2dh_acc_status *stat = i2c_get_clientdata(client);
#ifdef SHMDS_DETECT
	int i;
	if ((shmds_detect_switch == DETECT_ON) && (atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE)){
		stat->pdata->poll_interval = shmds_normal_interval;
		atomic_set(&shmds_detect_mode_flg,SHMDS_DETECT_NORMAL);
		shmds_count = 0;
		for (i = 0; i < 3; i++)
		{
			memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
		}
		memset(shmds_economize_buf, 0, sizeof(shmds_economize_buf));
	}
#endif /* SHMDS_DETECT */

	if (stat->on_before_suspend)
		return lis2dh_acc_enable(stat);
	return 0;
}

static int lis2dh_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lis2dh_acc_status *stat = i2c_get_clientdata(client);
#ifdef SHMDS_DETECT
	int i;
	if ((shmds_detect_switch == DETECT_ON) && (atomic_read(&shmds_detect_mode_flg) == SHMDS_DETECT_ECONOMIZE)){
		stat->pdata->poll_interval = shmds_normal_interval;
		atomic_set(&shmds_detect_mode_flg,SHMDS_DETECT_NORMAL);
		shmds_count = 0;
		for (i = 0; i < 3; i++)
		{
			memset(shmds_detect_buf[i], 0, (shmds_buf_size * sizeof(signed short)));
		}
		memset(shmds_economize_buf, 0, sizeof(shmds_economize_buf));
	}
#endif /* SHMDS_DETECT */

	stat->on_before_suspend = atomic_read(&stat->enabled);
	return lis2dh_acc_disable(stat);
}
#else
#define lis2dh_acc_suspend	NULL
#define lis2dh_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lis2dh_acc_id[]
		= { { LIS2DH_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, lis2dh_acc_id);

/* SHMDS_ID_0107_01 add S */
#ifdef CONFIG_OF
static const struct of_device_id shmds_dev_dt_match[] = {
	{ .compatible = "st,lis2dh_acc",},
	{}
};
#else
#define shmds_dev_dt_match NULL;
#endif /* CONFIG_OF */
/* SHMDS_ID_0107_01 add E */

static struct i2c_driver lis2dh_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LIS2DH_ACC_DEV_NAME,
			.of_match_table = shmds_dev_dt_match, /* SHMDS_ID_0107_01 add */
		  },
	.probe = lis2dh_acc_probe,
/* SHMDS_ID_0106_01 mod S */
//	.remove = __devexit_p(lis2dh_acc_remove),
	.remove = lis2dh_acc_remove,
/* SHMDS_ID_0106_01 mod E */
	.suspend = lis2dh_acc_suspend,
	.resume = lis2dh_acc_resume,
	.id_table = lis2dh_acc_id,
};

static int __init lis2dh_acc_init(void)
{
	pr_info("%s accelerometer driver: init\n",
						LIS2DH_ACC_DEV_NAME);

	shmds_qos_init();           /* SHMDS_ID_0205_01 add */
	return i2c_add_driver(&lis2dh_acc_driver);
}

static void __exit lis2dh_acc_exit(void)
{

	pr_info("%s accelerometer driver exit\n",
						LIS2DH_ACC_DEV_NAME);

	shmds_qos_destroy();        /* SHMDS_ID_0205_01 add */	
	i2c_del_driver(&lis2dh_acc_driver);
	return;
}

/* SHMDS_ID_0107_01 add S */
int shmds_set_gpio_no( struct i2c_client *client )
{
    struct device_node *np = client->dev.of_node;
    
    client_mcu = client;
    shmds_gpio_no_int0  = of_get_named_gpio(np, "qcom,shmds-gpio-int0",  0);
    printk("[shmds]shmds_set_gpio_no: %ld\n", (long)shmds_gpio_no_int0);

    return 0;
}

int shmds_get_gpio_no(void)
{
    int gpio_no = -1;

    gpio_no = shmds_gpio_no_int0;

    return gpio_no;
}

static int32_t shmds_gpio_init(void)
{
    int32_t ret;

    g_nIntIrqNo = shmds_gpio_to_irq();
	
    ret = shmds_gpio_request();
    if (ret < 0){
        printk("failed to gpio_request ret=%d\n", ret);
        goto ERROR;
    }
	
    ret = shmds_gpio_direction_input();
    if (ret < 0){
        printk("failed to gpio_direction_input ret=%d\n", ret);
        goto ERROR;
    }

    return SHMDS_RC_OK;

ERROR:
    shmds_gpio_free();

    return -ENODEV;
}

int shmds_gpio_request(void)
{
    struct pinctrl_state *pin_state = NULL;
    struct pinctrl *pin;
    int ret = 0;
	int gpio_no;
    
    gpio_no = shmds_get_gpio_no();
    pin = devm_pinctrl_get(&client_mcu->dev);
    
    ret = gpio_request(gpio_no, SHMDS_GPIO_INT0_NAME);
    if (ret < 0){
        printk("[shmds]shmds_gpio_request: gpio_request error(ret=%d)\n", ret);
        return -1;
    }
    
	pin_state = pinctrl_lookup_state(pin, "shmds_int0_active");
    if(pin_state == NULL){
        printk("[shmds]shmds_gpio_request: Null error\n");
        return -1;
    }
    
    ret = pinctrl_select_state(pin, pin_state);
    if(ret){
        printk("[shmds]shmds_gpio_request: error(ret=%d)\n", ret);
        return ret;
    }
	
    return 0;
}

int shmds_gpio_free(void)
{
    struct pinctrl_state *pin_state = NULL;
    struct pinctrl *pin;
    int ret;
    int gpio_no;
    
    gpio_no = shmds_get_gpio_no();
    pin = devm_pinctrl_get(&client_mcu->dev);
    
    pin_state = pinctrl_lookup_state(pin, "shmds_int0_suspend");
    if(pin_state == NULL){
        printk("[shmds]shmds_gpio_free: Null error\n");
        return -1;
    }
	
    gpio_free(gpio_no);
	
    ret = pinctrl_select_state(pin, pin_state);
    if(ret){
        printk("[shmds]shmds_gpio_free: error(ret=%d)\n", ret);
        return ret;
    }
	
    return 0;
}

int shmds_gpio_direction_output(int data)
{
    int gpio_no;
    
    gpio_no = shmds_get_gpio_no();
    gpio_direction_output(gpio_no, data);
    return 0;
}

int shmds_gpio_direction_input(void)
{
    int gpio_no;
    
    gpio_no = shmds_get_gpio_no();
    gpio_direction_input(gpio_no);
    return 0;
}

int shmds_gpio_to_irq(void)
{
    int gpio_no = 0;
    int ret  = 0;
    
    gpio_no = shmds_get_gpio_no();
    ret = gpio_to_irq(gpio_no);
    return ret;
}
/* SHMDS_ID_0107_01 add E */

/* SHMDS_ID_0205_01 add S */
static void shmds_qos_init(void)
{
    shmds_qos_num = 0;
    mutex_init(&qosMutex);
    shmds_qos_cpu_dma_latency.type = PM_QOS_REQ_AFFINE_CORES;
    shmds_qos_cpu_dma_latency.cpus_affine.bits[0] = 0x0f; /* little cluster */
    pm_qos_add_request(&shmds_qos_cpu_dma_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
    return;
}

static void shmds_qos_destroy(void)
{
    shmds_qos_num = 0;
    pm_qos_remove_request(&shmds_qos_cpu_dma_latency);
    return;
}

void shmds_qos_start(void)
{
	mutex_lock(&qosMutex);
    if (!shmds_qos_num) {
    	pm_qos_update_request(&shmds_qos_cpu_dma_latency, SHMDS_PM_QOS_LATENCY_VALUE);
    }
    shmds_qos_num++;
    mutex_unlock(&qosMutex);

    return;
}

void shmds_qos_end(void)
{
	mutex_lock(&qosMutex);
    shmds_qos_num--;
    if (!shmds_qos_num) {
    	pm_qos_update_request(&shmds_qos_cpu_dma_latency, PM_QOS_DEFAULT_VALUE);
    }
    if (shmds_qos_num < 0) {
        shmds_qos_num = 0;
    }
    mutex_unlock(&qosMutex);

    return;
}
/* SHMDS_ID_0205_01 add E */

module_init(lis2dh_acc_init);
module_exit(lis2dh_acc_exit);

MODULE_DESCRIPTION("lis2dh accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");

