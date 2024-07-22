/* drivers/sharp/shbatt/shbatt_kerl.c
 *
 * Copyright (C) 2014 SHARP CORPORATION All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*+-----------------------------------------------------------------------------+*/
/*| @ DEFINE COMPILE SWITCH :                                                   |*/
/*+-----------------------------------------------------------------------------+*/
#define SHBATT_ENABLE_LIMIT_LOCK
/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/
#define pr_fmt(fmt) "SHBATT:%s: " fmt, __func__

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/alarmtimer.h>	/* Timer */
#include <linux/time.h>	/* Timer */
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/qpnp/qpnp-adc.h>

#include "sharp/shbatt_kerl.h"
#include "shbatt_type.h"
#include "sharp/shpwr_log.h"
#include "sharp/sh_smem.h"
#include "sharp/shdiag_smd.h"
#include "sharp/shterm_k.h"
#ifdef SHBATT_ENABLE_LIMIT_LOCK
#include <linux/cpufreq.h>
#endif /*SHBATT_ENABLE_LIMIT_LOCK*/
#include <linux/usb/msm_hsusb.h>

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL MACRO DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

#define SHBATT_ERROR(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,x)
#define SHBATT_INFO(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_INFO,SHPWR_LOG_TYPE_BATT,x)
#define SHBATT_TRACE(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,x)

#define SHBATT_DEV_NAME						"shbatt"
#define SHBATT_OF_DEV_NAME					"sharp,shbatt"
#define SHBATT_ATTR_ARRAY_END_NAME			"END_NULL"

#define SHBATT_WAKE_CTL(x)										\
{																\
	do															\
	{															\
		if(x == 0)												\
		{														\
			SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(0) call shbatt_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wake_lock_num));	\
			if(atomic_dec_return(&shbatt_wake_lock_num) == 0)	\
			{													\
				wake_unlock(&shbatt_wake_lock); 				\
				SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(0) done shbatt_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wake_lock_num));	\
			}													\
		}														\
		else													\
		{														\
			SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(1) call shbatt_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wake_lock_num));	\
			if(atomic_inc_return(&shbatt_wake_lock_num) == 1)	\
			{													\
				SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(1) done shbatt_wake_lock_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wake_lock_num));	\
				wake_lock(&shbatt_wake_lock);					\
			}													\
		}														\
	} while(0); 												\
}

#define SHBATT_FG_ATTR(_name)									\
{																\
	.attr	=													\
	{															\
		.name = #_name,											\
		.mode = S_IRUGO | S_IWUSR | S_IWGRP						\
	},															\
	.show	= shbatt_drv_show_fuelgauge_property,				\
	.store	= shbatt_drv_store_fuelgauge_property,				\
}

#define SHBATT_ADC_ATTR(_name)									\
{																\
	.attr	=													\
	{															\
		.name = #_name,											\
		.mode = S_IRUGO | S_IWUSR | S_IWGRP						\
	},															\
	.show	= shbatt_drv_show_adc_property,						\
	.store	= shbatt_drv_store_property,						\
}
#define SHBATT_ATTR_END											\
{																\
	.attr  =													\
	{															\
		.name = SHBATT_ATTR_ARRAY_END_NAME,						\
		.mode = S_IRUGO | S_IWUSR | S_IWGRP						\
	},															\
	.show  = NULL,												\
	.store = NULL,												\
}

#ifndef MAX
#define MAX(a,b)	(((a) > (b)) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a,b)	(((a) < (b)) ? (a) : (b))
#endif

#define SHBATT_DEBUG_PRINT(_flg, _str, _val) \
	if (_flg) printk(KERN_ERR "%s: %s = %d\n", __FUNCTION__, #_str, _val)

#define DEBUG_CAPACITY_IS_ENABLE() \
		((debug_capacity >= 0) && (debug_capacity <= 100))

#define SHPWR_LOG_INFO(fmt, ...) { \
     shpwr_add_dbg_log(pr_fmt(fmt), ##__VA_ARGS__); \
     pr_info(fmt, ##__VA_ARGS__); \
}
#define SHPWR_DUMP_REG_INFO(fmt, ...) shpwr_add_dump_reg(false, fmt, ##__VA_ARGS__)
#define SHPWR_DUMP_REG_INFO_AND_FORCESAVE(fmt, ...) shpwr_add_dump_reg(true, fmt, ##__VA_ARGS__)

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/
#define GISPRODUCT_F_MASK					0x00000080
#define GISSOFTUP_F_MASK					0x00000010

/* for calc base battery capacity */
#define SHBATT_INVALID_CALC_BASE_CAPACITY	(-1)
#define SHBATT_INVALID_CALC_BASE_BAT_THERM	(-255)

/* Timer */
#define SHBATT_TIMER_FG_PERIOD_NS			(10LL * NSEC_PER_SEC)
#define SHBATT_TIMER_MSEC_TO_NSEC(x)		(x * NSEC_PER_MSEC)
#define SHBATT_TIMER_SEC_TO_NSEC(x)			(x * NSEC_PER_SEC)

/* wake_lock */
#define SHBATT_LOCK_FUNC_LEN				64	/* SH_PWR_DEBUG T.B.D */

#define SHBATT_FAIL_SAFE_INPUT_EVENT		0
#define SHBATT_FAIL_SAFE_RESUME				1
#define SHBATT_NON_DISP_CHAR				(-128) 

#define SHBATT_LOW_BATTERY_CHECK_INTERVAL_MS	500
#define	SHBATT_FATAL_BATTERY_CHECK_INTERVAL_MS	20

#define	SHBATT_LOW_BATTERY_THRESH_VBAT		3350
#define	SHBATT_FATAL_BATTERY_THRESH_VBAT	3100

#define	SHBATT_LOW_BATTERY_RECOVERY_THRESH_VBAT	3550

#define SHBATT_LOW_BATTERY_CHECK_COUNT		5
#define SHBATT_FATAL_BATTERY_CHECK_COUNT	5

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/
typedef enum
{
	SHBATT_CALIBRATE_BATTERY_VOLTAGE_LOCK,
	SHBATT_TASK_COMMAND_LOCK,
	SHBATT_MUTEX_TYPE_NUM
}shbatt_kernel_mutex_type;

typedef enum
{
	SHBATT_TIMER_TYPE_HRTIMER,
	SHBATT_TIMER_TYPE_ALARMTIMER,
	SHBATT_TIMER_TYPE_NUM
}shbatt_timer_type;

#ifdef SHBATT_ENABLE_LIMIT_LOCK
typedef enum shbatt_limit_lock_level_tag
{
	SHBATT_LIMIT_LOCK_LEVEL1 = 800000,
	SHBATT_LIMIT_LOCK_LEVEL2 = 800000,
	SHBATT_LIMIT_LOCK_LEVEL3 = 800000,
	SHBATT_LIMIT_LOCK_LEVEL4 = 800000,
	SHBATT_LIMIT_LOCK_LEVEL5 = 800000,
	SHBATT_LIMIT_LOCK_LEVEL6 = 800000,
	SHBATT_LIMIT_LOCK_LEVEL7 = 800000,
	SHBATT_LIMIT_LOCK_LEVEL8 = 800000,
} shbatt_limit_lock_level_t;
#endif /* SHBATT_ENABLE_LIMIT_LOCK */

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/
/* Timer */
typedef struct shbatt_timer_tag
{
	union
	{
		struct alarm			alarm_timer;
		struct hrtimer			hr_timer;
		
	}alm;
	union
	{
		enum alarmtimer_restart	(*alarm_cb)(struct alarm *, ktime_t);
		enum hrtimer_restart	(*hrtimer_cb)(struct hrtimer *);
	}cb_func;
	shbatt_timer_type			timer_type;
	enum alarmtimer_type		alarm_type;
	int							prm;
} shbatt_timer_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ STATIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

static bool						shbatt_task_is_initialized = false;
static struct wake_lock			shbatt_wake_lock;
static atomic_t					shbatt_wake_lock_num;

static enum power_supply_property	shbatt_ps_props[] =
{
	POWER_SUPPLY_PROP_TYPE,
};

static dev_t					shbatt_dev;
static int						shbatt_major;
static int						shbatt_minor;
static struct cdev				shbatt_cdev;
static struct class*			shbatt_dev_class;
static atomic_t					shbatt_usse_op_cnt;
struct timespec					shbatt_last_hrtimer_expire_time;
struct timespec					shbatt_last_timer_expire_time;
static bool						shbatt_timer_restarted = false;

/* Timer */
static spinlock_t				shbatt_pkt_lock;
static struct mutex				shbatt_task_lock;
static struct workqueue_struct*	shbatt_task_workqueue_p;
static wait_queue_head_t		shbatt_usse_wait;
static struct completion		shbatt_usse_cmp;
static shbatt_packet_t			shbatt_pkt[16];
static shbatt_usse_packet_t		shbatt_usse_pkt;

/* wake_lock */
static struct timespec			shbatt_lock_time[SHBATT_MUTEX_TYPE_NUM];
static struct timespec			shbatt_unlock_time[SHBATT_MUTEX_TYPE_NUM];
static char						shbatt_lock_func[SHBATT_MUTEX_TYPE_NUM][SHBATT_LOCK_FUNC_LEN];

/* sysfs */
/* for cpu_temp */
static int						debug_cpu_temp = 0;
module_param_named(debug_cpu_tmp, debug_cpu_temp, int, S_IRUSR | S_IWUSR);

/* for lcd_temp */
static int						debug_lcd_temp = 0;
module_param_named(debug_lcd_tmp, debug_lcd_temp, int, S_IRUSR | S_IWUSR);

/* for pa0_temp */
static int						debug_pa0_temp = 0;
module_param_named(debug_pa0_tmp, debug_pa0_temp, int, S_IRUSR | S_IWUSR);

/* for cam_temp */
static int						debug_cam_temp = 0;
module_param_named(debug_cam_tmp, debug_cam_temp, int, S_IRUSR | S_IWUSR);

#ifdef CONFIG_PM_SUPPORT_BATT_TRACEABILITY
static char						pack_name[] = "0000";
module_param_string(pack_name, pack_name, sizeof(pack_name), 0600);

static char						cell_maker_name[] = "0000000000000000";
module_param_string(cell_maker_name, cell_maker_name, sizeof(cell_maker_name), 0600);

static char						maker_name[] = "0000000000000000";
module_param_string(maker_name, maker_name, sizeof(maker_name), 0600);

static char						cell_date[] = "00000000";
module_param_string(cell_date, cell_date, sizeof(cell_date), 0644);

static char						cell_line[] = "00";
module_param_string(cell_line, cell_line, sizeof(cell_line), 0644);

static char						pack_date[] = "00000000";
module_param_string(pack_date, pack_date, sizeof(pack_date), 0644);

static char						pack_line[] = "00";
module_param_string(pack_line, pack_line, sizeof(pack_line), 0644);

static char						pack_manu_num[] = "0000";
module_param_string(pack_manu_num, pack_manu_num, sizeof(pack_manu_num), 0644);

#endif /* CONFIG_PM_SUPPORT_BATT_TRACEABILITY */

#ifdef CONFIG_PM_SUPPORT_BATT_AUTH
static int						batt_auth = -1;
module_param(batt_auth, int, 0644);
#else /* CONFIG_PM_SUPPORT_BATT_AUTH */
#ifdef CONFIG_PM_SUPPORT_BATT_TRACEABILITY
static int						batt_auth = 1;
module_param(batt_auth, int, 0644);
#endif /* CONFIG_PM_SUPPORT_BATT_TRACEABILITY */
#endif /*CONFIG_PM_SUPPORT_BATT_AUTH*/

static shbatt_flg_t  shbatt_flg = {false, false, false, false, false};
module_param_named(disable_usb_charging,    shbatt_flg.disable_usb_charging,    bool, S_IRUSR | S_IWUSR);
module_param_named(disable_thermal_control, shbatt_flg.disable_thermal_control, bool, S_IRUSR | S_IWUSR);
module_param_named(disable_shutdown,        shbatt_flg.disable_shutdown,        bool, S_IRUSR | S_IWUSR);
module_param_named(factory_full_charge,     shbatt_flg.factory_full_charge,     bool, S_IRUSR | S_IWUSR);
module_param_named(disable_soc_poll,        shbatt_flg.disable_soc_poll,        bool, S_IRUSR | S_IWUSR);

/* fg_calibration_data[0]:adjmin */
/* fg_calibration_data[1]:adjmax */
/* fg_calibration_data[2]:curmin */
/* fg_calibration_data[3]:curmax */
static int						fg_calibration_data[4] = {0,0,0,0};
module_param_array(fg_calibration_data, int, NULL, 0644);

/* bat_calibration_data[0]:adjmin */
/* bat_calibration_data[1]:adjmax */
/* bat_calibration_data[2]:curmin */
/* bat_calibration_data[3]:curmax */
static int						bat_calibration_data[4] = {0,0,0,0};
module_param_array(bat_calibration_data, int, NULL, S_IRUSR | S_IWUSR);

static int						log_event = SHBATTLOG_EVENT_CAPACITY_FAILSAFE;

static int						shbatt_low_battery_count = 0;
static int						shbatt_fatal_battery_count = 0;
static shbatt_low_battery_state_t	low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE1;

#ifdef SHBATT_ENABLE_LIMIT_LOCK
static int shbatt_cur_capacity = 80;
static shbatt_limit_lock_level_t shbatt_limit_lock_level_table[8] =
{
	SHBATT_LIMIT_LOCK_LEVEL1,
	SHBATT_LIMIT_LOCK_LEVEL2,
	SHBATT_LIMIT_LOCK_LEVEL3,
	SHBATT_LIMIT_LOCK_LEVEL4,
	SHBATT_LIMIT_LOCK_LEVEL5,
	SHBATT_LIMIT_LOCK_LEVEL6,
	SHBATT_LIMIT_LOCK_LEVEL7,
	SHBATT_LIMIT_LOCK_LEVEL8,
};
static int shbatt_limit_lock_level_table_size = sizeof(shbatt_limit_lock_level_table) / sizeof(shbatt_limit_lock_level_table[0]);
static int shbatt_limit_lock_count = 0;
#endif /* SHBATT_ENABLE_LIMIT_LOCK */
/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/
#ifdef SHBATT_ENABLE_LIMIT_LOCK
static int shbatt_api_cpu_clock_limit_lock( shbatt_limit_lock_level_t level );
static int shbatt_api_cpu_clock_limit_unlock( void );

static int shbatt_limit_lock_callback(struct notifier_block *nb, unsigned long event, void *data);
static struct notifier_block shbatt_limit_lock_notifier =
{
	.notifier_call = shbatt_limit_lock_callback,
};
static unsigned int shbatt_limit_freq = UINT_MAX;
#endif /* SHBATT_ENABLE_LIMIT_LOCK */

static int shbatt_drv_create_device( void );

/* task */
/* task(Timer) */
static void shbatt_task(
	struct work_struct*			work_p );

static void shbatt_task_cmd_invalid(
	shbatt_packet_t*			pkt_p );

static void shbatt_task_cmd_exec_fuelgauge_soc_poll_sequence(
	shbatt_packet_t*			pkt_p );

/* low batt */
static void shbatt_task_cmd_exec_low_battery_check_sequence(
	shbatt_packet_t*			pkt_p );

static void shbatt_task_cmd_battlog_event(
	shbatt_packet_t*			pkt_p );

/* seq */
static shbatt_result_t shbatt_seq_initialize( void );

static shbatt_result_t shbatt_seq_exec_fuelgauge_soc_poll_sequence(
	shbatt_soc_t				soc );

/* low batt */
static void shbatt_seq_exec_low_battery_check_sequence_phase1(
	int							evt );

static void shbatt_seq_exec_low_battery_check_sequence_phase2(
	int							evt );

static void shbatt_seq_exec_low_battery_check_sequence_phase3(
	int							evt );

static void shbatt_seq_exec_low_battery_check_sequence_phase4(
	int							evt );

static enum hrtimer_restart shbatt_seq_fuelgauge_soc_poll_hrtimer_expire_cb(
	struct hrtimer*				hrtimer_p );

static enum hrtimer_restart shbatt_seq_fuelgauge_soc_poll_hrtimer_expire_multi_cb(
	struct hrtimer*				hrtimer_p );

static enum alarmtimer_restart shbatt_seq_fuelgauge_soc_poll_timer_expire_cb(
	struct alarm*				alm_p,
	ktime_t						kerl_time );

static enum alarmtimer_restart shbatt_seq_fuelgauge_soc_poll_timer_expire_multi_cb(
	struct alarm*				alm_p,
	ktime_t						kerl_time );

static enum alarmtimer_restart shbatt_seq_low_battery_poll_timer_expire_cb(
	struct alarm*				alm_p,
	ktime_t						kerl_time );

static enum alarmtimer_restart shbatt_seq_fatal_battery_poll_timer_expire_cb(
	struct alarm*				alm_p,
	ktime_t						kerl_time );

static shbatt_result_t shbatt_seq_battlog_event(
	int							evt );

/* api */
static shbatt_result_t shbatt_api_initialize( void );

static shbatt_result_t shbatt_api_get_fuelgauge_current(
	int*						cur_p );

static shbatt_result_t shbatt_api_exec_fuelgauge_soc_poll_sequence(
	shbatt_soc_t				soc );

static shbatt_result_t shbatt_api_exec_low_battery_check_sequence(
	int							evt );


static shbatt_result_t shbatt_api_initialize_fuelgauge_calibration_data( void );

/* from ioctrl. */
/* Timer */
static int shbatt_drv_ioctl_cmd_initialize(
	struct file*				fi_p,
	unsigned					long arg );

static int shbatt_drv_ioctl_cmd_pull_usse_packet(
	struct file*				fi_p,
	unsigned long				arg );

static int shbatt_drv_ioctl_cmd_done_usse_packet(
	struct file*				fi_p,
	unsigned long				arg );

static int shbatt_drv_ioctl_cmd_set_timer(
	struct file*				fi_p,
	unsigned long				arg );

static int shbatt_drv_ioctl_cmd_clr_timer(
	struct file*				fi_p,
	unsigned long				arg );

static int shbatt_drv_ioctl_cmd_get_boot_time(
	struct file*				fi_p,
	unsigned long				arg );

static int shbatt_drv_ioctl_cmd_get_shbatt_log_info(
	struct file*				fi_p,
	unsigned long				arg );

/* from driver */
static int shbatt_drv_create_ps_device_file(
	struct device*				dev_p,
	struct device_attribute*	attr_p );

static int shbatt_drv_create_ps_attrs(
	struct platform_device*		dev_p );

/* from attribute */

/* from property */
static int shbatt_ps_battery_set_property(
	struct power_supply*		psy,
	enum power_supply_property	psp,
	const union power_supply_propval*	val );

static int shbatt_drv_get_ps_property(
	struct power_supply*		psy_p,
	enum power_supply_property	psp,
	union power_supply_propval*	val_p );

static int shbatt_drv_set_ps_property(
	struct power_supply*		psy_p,
	enum power_supply_property	psp,
	const union power_supply_propval*	val_p );

static shbatt_result_t shbatt_api_initialize_bat_calibration_data( void );

static void shbatt_api_set_timer(
	shbatt_poll_timer_info_t*	pti );

static shbatt_result_t shbatt_api_low_battery_low_threshold_check(
	int							vbat );

static shbatt_result_t shbatt_low_battery_recovery_check(
	int							vbat );

/* Timer */
static shbatt_packet_t* shbatt_task_get_packet( void );

static void shbatt_task_free_packet(
	shbatt_packet_t*			pkt );

static shbatt_result_t shbatt_seq_call_user_space_sequence_executor( void );

/* wake_lock */
static void shbatt_seq_lock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func );

static void shbatt_seq_unlock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func );

/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE PROTO TYPE DECLARE :                               |*/
/*+-----------------------------------------------------------------------------+*/
/* attribute store */
static ssize_t shbatt_drv_store_property(
	struct device*				dev_p,
	struct device_attribute*	attr_p,
	const char*					buf_p,
	size_t						size );

static ssize_t shbatt_drv_store_fuelgauge_property(
	struct device*				dev_p,
	struct device_attribute*	attr_p,
	const char*					buf_p,
	size_t						size );

/* attribute show */
static ssize_t shbatt_drv_show_fuelgauge_property(
	struct device*				dev_p,
	struct device_attribute*	attr_p,
	char*						buf_p );

static ssize_t shbatt_drv_show_adc_property(
	struct device*				dev_p,
	struct device_attribute*	attr_p,
	char*						buf_p );

/* driver I/F */
static int shbatt_drv_open(
	struct inode*				in_p,
	struct file*				fi_p );

static int shbatt_drv_release(
	struct inode*				in_p,
	struct file*				fi_p );

static unsigned int shbatt_drv_poll(
	struct file*				fi_p,
	poll_table*					wait_p );

static long shbatt_drv_ioctl(
	struct file*				fi_p,
	unsigned int				cmd,
	unsigned long				arg );

static int shbatt_drv_probe(
	struct platform_device*		dev_p );

static int shbatt_drv_remove(
	struct platform_device*		dev_p );

static void shbatt_drv_shutdown(
	struct platform_device*		dev_p );

static int shbatt_drv_resume(
	struct platform_device*		dev_p);

static int shbatt_drv_suspend(
	struct platform_device*		dev_p,
	pm_message_t				state );

static int __init shbatt_drv_module_init( void );
static void __exit shbatt_drv_module_exit( void );

bool is_shbatt_prs_launched( void );

/*+-----------------------------------------------------------------------------+*/
/*| @ FUNCTION TABLE PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/
static struct device_attribute		shbatt_fuelgauge_attributes[] =
{
	SHBATT_FG_ATTR(current),						/* SHBATT_PS_PROPERTY_CURRENT */
	SHBATT_FG_ATTR(log_debug),						/* SHBATT_PS_PROPERTY_LOG_DEBUG */
	SHBATT_ATTR_END
};

static struct device_attribute		shbatt_adc_attributes[] =
{
	SHBATT_ADC_ATTR(cpu_temp),						/* SHBATT_PS_PROPERTY_CPU_TEMP */
	SHBATT_ADC_ATTR(lcd_temp),						/* SHBATT_PS_PROPERTY_LCD_TEMP */
	SHBATT_ADC_ATTR(pa0_temp),						/* SHBATT_PS_PROPERTY_PA0_TEMP */
	SHBATT_ADC_ATTR(cam_temp),						/* SHBATT_PS_PROPERTY_CAM_TEMP */
	SHBATT_ATTR_END
};

static shbatt_psy_attr_info_t		shbatt_power_supplies[] =
{
	{
		.psy =
		{
			.name = "shbatt_fuelgauge",
			.type = POWER_SUPPLY_TYPE_BATTERY,
			.properties = shbatt_ps_props,
			.num_properties = ARRAY_SIZE(shbatt_ps_props),
			.get_property = shbatt_drv_get_ps_property,
			.set_property = shbatt_drv_set_ps_property,
		},
		.attr_array = shbatt_fuelgauge_attributes
	},

	{
		.psy =
		{
			.name = "shbatt_adc",
			.type = POWER_SUPPLY_TYPE_BATTERY,
			.properties = shbatt_ps_props,
			.num_properties = ARRAY_SIZE(shbatt_ps_props),
			.get_property = shbatt_drv_get_ps_property,
			.set_property = shbatt_drv_set_ps_property,
		},
		.attr_array = shbatt_adc_attributes
	},

};

static struct file_operations shbatt_fops =
{
	.owner			= THIS_MODULE,
	.open			= shbatt_drv_open,
	.release		= shbatt_drv_release,
	.poll			= shbatt_drv_poll,
	.unlocked_ioctl	= shbatt_drv_ioctl,
	.compat_ioctl	= shbatt_drv_ioctl,
};

#ifdef CONFIG_OF
static struct of_device_id shbatt_match_table[] = {
	{ .compatible = SHBATT_OF_DEV_NAME },
	{}
};
#else  /* CONFIG_OF */
#define shbatt_match_table NULL;
#endif /* CONFIG_OF */

static struct platform_driver shbatt_platform_driver = {
	.probe		= shbatt_drv_probe,
	.remove		= shbatt_drv_remove,
	.shutdown	= shbatt_drv_shutdown,
	.driver		= {
		.name	= SHBATT_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = shbatt_match_table,
	},
	.resume		= shbatt_drv_resume,
	.suspend	= shbatt_drv_suspend,
};

/* Timer */
static void (*const shbatt_task_cmd_func[])( shbatt_packet_t* pkt_p ) =
{
	shbatt_task_cmd_invalid,									/* SHBATT_TASK_CMD_INVALID */
	shbatt_task_cmd_exec_fuelgauge_soc_poll_sequence,			/* SHBATT_TASK_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE */
	shbatt_task_cmd_exec_low_battery_check_sequence,			/* SHBATT_TASK_CMD_EXEC_LOW_BATTERY_CHECK_SEQUENCE */
	shbatt_task_cmd_battlog_event,								/* SHBATT_TASK_CMD_BATTLOG_EVENT */
};

static shbatt_timer_t			shbatt_poll_timer[NUM_SHBATT_POLL_TIMER_TYPE] =
{
	{
		.cb_func =
		{
			.alarm_cb = shbatt_seq_fuelgauge_soc_poll_timer_expire_cb,
		},
		.alarm_type	= ALARM_BOOTTIME,
		.timer_type	= SHBATT_TIMER_TYPE_ALARMTIMER,
	},
	{
		.cb_func =
		{
			.hrtimer_cb = shbatt_seq_fuelgauge_soc_poll_hrtimer_expire_cb,
		},
		.alarm_type	= ALARM_BOOTTIME,
		.timer_type	= SHBATT_TIMER_TYPE_HRTIMER,
	},
	{
		.cb_func =
		{
			.alarm_cb = shbatt_seq_fuelgauge_soc_poll_timer_expire_multi_cb,
		},
		.alarm_type	= ALARM_BOOTTIME,
		.timer_type	= SHBATT_TIMER_TYPE_ALARMTIMER,
	},
	{
		.cb_func =
		{
			.hrtimer_cb = shbatt_seq_fuelgauge_soc_poll_hrtimer_expire_multi_cb,
		},
		.alarm_type	= ALARM_BOOTTIME,
		.timer_type	= SHBATT_TIMER_TYPE_HRTIMER,
	},
	/* low batt */
	{
		.cb_func =
		{
			.alarm_cb = shbatt_seq_low_battery_poll_timer_expire_cb,
		},
		.alarm_type	= ALARM_BOOTTIME,
		.timer_type	= SHBATT_TIMER_TYPE_ALARMTIMER,
	},
	{
		.cb_func =
		{
			.alarm_cb = shbatt_seq_fatal_battery_poll_timer_expire_cb,
		},
		.alarm_type	= ALARM_BOOTTIME,
		.timer_type	= SHBATT_TIMER_TYPE_ALARMTIMER,
	},
};

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION'S CODE AREA :                                             |*/
/*+-----------------------------------------------------------------------------+*/
void shbatt_api_notify_low_batt_alarm( void )
{
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_api_exec_low_battery_check_sequence(SHBATT_LOW_BATTERY_EVENT_LOW_INTERRUPT);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__);
}
EXPORT_SYMBOL_GPL(shbatt_api_notify_low_batt_alarm);

void shbatt_api_notify_fatal_batt_alarm( void )
{
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_api_exec_low_battery_check_sequence(SHBATT_LOW_BATTERY_EVENT_FATAL_INTERRUPT);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__);
}
EXPORT_SYMBOL_GPL(shbatt_api_notify_fatal_batt_alarm);

shbatt_result_t shbatt_api_get_bat_calibration_data(
	shbatt_bat_calibration_data_t*	cal
){
	shbatt_result_t				result = SHBATT_RESULT_SUCCESS;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);
	
	cal->min  = bat_calibration_data[0];
	cal->max  = bat_calibration_data[1];
	cal->vmin = bat_calibration_data[2];
	cal->vmax = bat_calibration_data[3];

	SHBATT_TRACE("[bat_calibration min]  %d \n", cal->min);
	SHBATT_TRACE("[bat_calibration max]  %d \n", cal->max);
	SHBATT_TRACE("[bat_calibration vmin] %d \n", cal->vmin);
	SHBATT_TRACE("[bat_calibration vmax] %d \n", cal->vmax);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

bool shbatt_api_is_disable_usb_charging( void )
{
	static bool					call_once = false;
	sharp_smem_common_type*		p_smem = 0;
	int							softupdate_flg = 0;

	if(!call_once) {
	
		p_smem = sh_smem_get_common_address();
		if(p_smem != 0) {
			softupdate_flg = (p_smem->shdiag_FlagData & GISSOFTUP_F_MASK) ? 1 : 0;
			if((p_smem->shdiag_BootMode == D_SHDIAG_BOOT_FUNC && softupdate_flg == 0) ||
			   (p_smem->shdiag_BootMode == D_SHDIAG_BOOT_HW   && softupdate_flg == 0) ||
			   (p_smem->shusb_usb_charge_ena_flag == 0)) {
				shbatt_flg.disable_usb_charging = true;
			}
		} else {
			SHBATT_ERROR("invalid pointer shbatt_api_is_disable_usb_charging\n");
		}
		call_once = true;
	}
	
	return shbatt_flg.disable_usb_charging;
}

bool shbatt_api_is_disable_thermal_control( void )
{
	static						bool call_once = false;
	sharp_smem_common_type*		p_smem = 0;
	int							product_flg = 0;
	
	if(!call_once) {
		p_smem = sh_smem_get_common_address();
		if(p_smem != 0) {
			product_flg = (p_smem->shdiag_FlagData & GISPRODUCT_F_MASK)  ? 1 : 0;
			if(product_flg)
				shbatt_flg.disable_thermal_control = true;
		}else{
			SHBATT_ERROR("invalid pointer shbatt_api_is_disable_thermal_control\n");
		}
		call_once = true;
	}
	return shbatt_flg.disable_thermal_control;
}

bool shbatt_api_is_disable_shutdown( void )
{
	static bool					call_once = false;
	sharp_smem_common_type*		p_smem = 0;
	int							product_flg = 0;
	int							softupdate_flg = 0;

	if(!call_once) {
	
		p_smem = sh_smem_get_common_address();
		if(p_smem != 0) {
			product_flg	   = (p_smem->shdiag_FlagData & GISPRODUCT_F_MASK)  ? 1 : 0;
			softupdate_flg = (p_smem->shdiag_FlagData & GISSOFTUP_F_MASK)   ? 1 : 0;
			/* SH_BOOT_NORMAL(0xFFFF) OFF_CHARGE(0x20) */
			if( ((p_smem->sh_boot_mode != 0xFFFF) && ( p_smem->sh_boot_mode != 0x20 )) || 
				(product_flg == 1) || (softupdate_flg == 1)){
				shbatt_flg.disable_shutdown = true;
			}
		} else {
			SHBATT_ERROR("invalid pointer shbatt_api_is_disable_shutdown\n");
		}
		call_once = true;
	}
	return shbatt_flg.disable_shutdown;
}

bool shbatt_api_is_factory_full_charge( void )
{
	return shbatt_flg.factory_full_charge;
}

bool shbatt_api_is_disable_soc_poll( void )
{
	static bool					call_once = false;
	sharp_smem_common_type*		p_smem = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__ );
				
	if(!call_once) {
		p_smem = sh_smem_get_common_address();
		if(p_smem != 0) {
			/* SH_BOOT_MODE_DIAG(0x40) or SH_BOOT_MODE_FORCE_FUNC(0x44) */
			if( p_smem->sh_boot_mode == 0x40 || p_smem->sh_boot_mode == 0x44 ){
				SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
						"[P] %s() : shbatt_flg.disable_soc_poll TRUE .\n", __FUNCTION__ );
				shbatt_flg.disable_soc_poll = true;
			}
		} else {
			SHBATT_ERROR("invalid pointer shbatt_api_is_disable_soc_poll\n");
		}
		call_once = true;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return shbatt_flg.disable_soc_poll;
}

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION'S CODE AREA :                                              |*/
/*+-----------------------------------------------------------------------------+*/

static int shbatt_drv_create_device( void )
{
	struct device*				dev_p;
	int							ret;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	ret = alloc_chrdev_region(&shbatt_dev,0,1,SHBATT_DEV_NAME);

	if(ret < 0)
	{
		SHBATT_ERROR("%s : alloc_chrdev_region failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_0;
	}

	shbatt_major = MAJOR(shbatt_dev);
	shbatt_minor = MINOR(shbatt_dev);

	cdev_init(&shbatt_cdev,&shbatt_fops);

	shbatt_cdev.owner = THIS_MODULE;

	ret = cdev_add(&shbatt_cdev,shbatt_dev,1);

	if(ret < 0)
	{
		SHBATT_ERROR("%s : cdev_add failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_1;
	}

	shbatt_dev_class = class_create(THIS_MODULE,SHBATT_DEV_NAME);

	if(IS_ERR(shbatt_dev_class))
	{
		ret = PTR_ERR(shbatt_dev_class);
		SHBATT_ERROR("%s : class_create failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_2;
	}

	dev_p = device_create(shbatt_dev_class,NULL,shbatt_dev,&shbatt_cdev,SHBATT_DEV_NAME);

	if(IS_ERR(dev_p))
	{
		ret = PTR_ERR(dev_p);
		SHBATT_ERROR("%s : device_create failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_3;
	}

	atomic_set(&shbatt_usse_op_cnt,0);

	/* Timer */
	init_waitqueue_head(&shbatt_usse_wait);
	init_completion(&shbatt_usse_cmp);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return ret;

create_device_exit_3:
	class_destroy(shbatt_dev_class);

create_device_exit_2:
	cdev_del(&shbatt_cdev);

create_device_exit_1:
	unregister_chrdev_region(shbatt_dev,1);

create_device_exit_0:

	return ret;
}

static void shbatt_task(
	struct work_struct*			work_p
){
	shbatt_packet_t*			pkt_p;

	shbatt_seq_lock_task_mutex( SHBATT_TASK_COMMAND_LOCK,
								__FUNCTION__ );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	
	pkt_p = (shbatt_packet_t*)work_p;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"shbatt_task %d \n",pkt_p->hdr.cmd );

	if(pkt_p->hdr.cmd < NUM_SHBATT_TASK_CMD)
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"shbatt_task OK \n");
		shbatt_task_cmd_func[pkt_p->hdr.cmd](pkt_p);
	}
	else
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"shbatt_task NG \n");
		shbatt_task_cmd_invalid(pkt_p);
	}

	SHBATT_WAKE_CTL(0);

	shbatt_task_free_packet(pkt_p);

	shbatt_seq_unlock_task_mutex( SHBATT_TASK_COMMAND_LOCK,
								__FUNCTION__ );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static void shbatt_task_cmd_invalid(
	shbatt_packet_t*			pkt_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static void shbatt_task_cmd_exec_fuelgauge_soc_poll_sequence(
	shbatt_packet_t*			pkt_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_seq_exec_fuelgauge_soc_poll_sequence(pkt_p->prm.soc);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

/* low batt */
static void shbatt_task_cmd_exec_low_battery_check_sequence(
	shbatt_packet_t*			pkt_p
){
	static void (*const seq_func[NUM_SHBATT_LOW_BATTERY_SEQUENCE])( int event ) = 
	{
		shbatt_seq_exec_low_battery_check_sequence_phase1,
		shbatt_seq_exec_low_battery_check_sequence_phase2,
		shbatt_seq_exec_low_battery_check_sequence_phase3,
		shbatt_seq_exec_low_battery_check_sequence_phase4,
	};
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	seq_func[low_batt_phase](pkt_p->prm.evt);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static void shbatt_task_cmd_battlog_event(
	shbatt_packet_t*			pkt_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_seq_battlog_event(pkt_p->prm.evt);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static shbatt_packet_t* shbatt_task_get_packet( void )
{
	int							idx;
	unsigned long				flags;
	shbatt_packet_t*			ret = NULL;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	spin_lock_irqsave( &shbatt_pkt_lock, flags );

	for( idx = 0; idx < 16; idx++ )
	{
		if( shbatt_pkt[idx].is_used == false )
		{
			shbatt_pkt[idx].is_used = true;

			ret = &shbatt_pkt[idx];

			break;
		}
	}

	spin_unlock_irqrestore( &shbatt_pkt_lock, flags );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return ret;
}

static void shbatt_task_free_packet(
	shbatt_packet_t*			pkt
){
	unsigned long				flags;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	spin_lock_irqsave( &shbatt_pkt_lock, flags );

	pkt->is_used = false;

	spin_unlock_irqrestore( &shbatt_pkt_lock, flags );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return;
}

static shbatt_result_t shbatt_seq_initialize( void )
{
	shbatt_result_t				result = SHBATT_RESULT_SUCCESS;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	shbatt_task_is_initialized = true;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return result;
}

static shbatt_result_t shbatt_seq_exec_fuelgauge_soc_poll_sequence(
	shbatt_soc_t				soc
){
	shbatt_result_t				result;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	shbatt_usse_pkt.hdr.cmd = SHBATT_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE;
	shbatt_usse_pkt.hdr.ret = SHBATT_RESULT_FAIL;
	shbatt_usse_pkt.prm.soc = soc;

	result = shbatt_seq_call_user_space_sequence_executor();

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return result;
}

static void shbatt_seq_exec_low_battery_check_sequence_phase1(
	int							evt
){
	shbatt_low_battery_state_t	prev_low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE1;
	shbatt_poll_timer_info_t	pti;
#ifdef SHBATT_ENABLE_LIMIT_LOCK
	struct power_supply*		batt_psy = NULL;
	union power_supply_propval	val = {0,};
	int	ret = 0;
#endif /*SHBATT_ENABLE_LIMIT_LOCK*/

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s() evt:%d\n",__FUNCTION__, evt );

	switch( evt )
	{
	case SHBATT_LOW_BATTERY_EVENT_LOW_INTERRUPT:
#ifdef SHBATT_ENABLE_LIMIT_LOCK
		batt_psy = power_supply_get_by_name("battery");
		if( batt_psy == NULL )
		{
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
						"[E] %s() batt_psy is NULL.\n", __FUNCTION__ );
			return;
		}

		ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, &val );
		if( ret < 0 )
		{
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
						"[E] %s() ret:%d POWER_SUPPLY_PROP_CAPACITY is failed.\n",__FUNCTION__, ret );
			return;
		}

		shbatt_cur_capacity = val.intval;
		if(shbatt_cur_capacity < 10)
		{
			shbatt_api_cpu_clock_limit_lock(shbatt_limit_lock_level_table[shbatt_limit_lock_level_table_size - 1]);
			shbatt_limit_lock_count = shbatt_limit_lock_level_table_size;
			SHBATT_TRACE("[P] %s limit_lock_level= MAX \n",__FUNCTION__);
		}
		else
		{
			shbatt_api_cpu_clock_limit_lock(shbatt_limit_lock_level_table[0]);
			shbatt_limit_lock_count = 1;
			SHBATT_TRACE("[P] %s limit_lock_level= MIN \n",__FUNCTION__);
		}
#endif /*SHBATT_ENABLE_LIMIT_LOCK*/

		memset( &pti, 0x00, sizeof( pti ) );
		pti.ptt	= SHBATT_POLL_TIMER_TYPE_LOW_BATTERY;
		pti.ms	= SHBATT_LOW_BATTERY_CHECK_INTERVAL_MS;
		pti.prm	= 0;
		shbatt_api_set_timer( &pti );
		shbatt_low_battery_count = 0;
		prev_low_batt_phase = low_batt_phase;
		low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE2;
		SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE, SHPWR_LOG_TYPE_BATT,
					"[P] %s() 3.4V. next sequence:%d->%d\n", __FUNCTION__, prev_low_batt_phase, low_batt_phase );
		break;

	default:
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[P] %s() other event:%d.\n",__FUNCTION__, evt );
		break;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return;
}

static void shbatt_seq_exec_low_battery_check_sequence_phase2(
	int							evt
){
	shbatt_result_t				ret = SHBATT_RESULT_SUCCESS;
	shbatt_low_battery_state_t	prev_low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE1;
	shbatt_poll_timer_info_t	pti;
	int							now_voltage = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s() evt:%d\n",__FUNCTION__, evt );

	switch( evt )
	{
	case SHBATT_LOW_BATTERY_EVENT_FATAL_INTERRUPT:
		memset( &pti, 0x00, sizeof( pti ) );
		pti.ptt	= SHBATT_POLL_TIMER_TYPE_FATAL_BATTERY;
		pti.ms	= SHBATT_FATAL_BATTERY_CHECK_INTERVAL_MS;
		pti.prm	= 0;
		shbatt_api_set_timer( &pti );
		shbatt_low_battery_count = 0;
		prev_low_batt_phase = low_batt_phase;
		low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE3;
		SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE, SHPWR_LOG_TYPE_BATT,
					"[P] %s() 3.1V. next sequence:%d->%d\n", __FUNCTION__, prev_low_batt_phase, low_batt_phase );
		break;

	case SHBATT_LOW_BATTERY_EVENT_LOW_TIMER:
		now_voltage		= sh_shbatt_smb1360_get_prp_voltage_now() / 1000;
		ret = shbatt_api_low_battery_low_threshold_check( now_voltage );
		if( ret == SHBATT_RESULT_FAIL )
		{
			break;
		}
		ret = shbatt_low_battery_recovery_check( now_voltage );
		if( ret == SHBATT_RESULT_SUCCESS )
		{

#ifdef SHBATT_ENABLE_LIMIT_LOCK
			SHBATT_TRACE("[P] %s limit_unlock \n",__FUNCTION__);
			shbatt_api_cpu_clock_limit_unlock();
			shbatt_limit_lock_count = 0;
#endif /*SHBATT_ENABLE_LIMIT_LOCK*/

			prev_low_batt_phase = low_batt_phase;
			low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE1;
			SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE, SHPWR_LOG_TYPE_BATT,
						"[P] %s() recovery check. next sequence:%d->%d\n", __FUNCTION__,
						prev_low_batt_phase, low_batt_phase );
			break;
		}
		memset( &pti, 0x00, sizeof( pti ) );
		pti.ptt	= SHBATT_POLL_TIMER_TYPE_LOW_BATTERY;
		pti.ms	= SHBATT_LOW_BATTERY_CHECK_INTERVAL_MS;
		pti.prm	= 0;
		shbatt_api_set_timer( &pti );
		break;

	default:
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[P] %s() other event:%d.\n",__FUNCTION__, evt );
		break;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return;
}

static void shbatt_seq_exec_low_battery_check_sequence_phase3(
	int							evt
){
	shbatt_result_t				ret = SHBATT_RESULT_SUCCESS;
	shbatt_low_battery_state_t	prev_low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE1;
	shbatt_poll_timer_info_t	pti;
	int							now_voltage = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s() evt:%d\n",__FUNCTION__, evt );

	switch( evt )
	{
	case SHBATT_LOW_BATTERY_EVENT_LOW_TIMER:
		now_voltage		= sh_shbatt_smb1360_get_prp_voltage_now() / 1000;
		ret = shbatt_api_low_battery_low_threshold_check( now_voltage );
		if( ret == SHBATT_RESULT_FAIL )
		{
			break;
		}
		ret = shbatt_low_battery_recovery_check( now_voltage );
		if( ret == SHBATT_RESULT_SUCCESS )
		{

#ifdef SHBATT_ENABLE_LIMIT_LOCK
			SHBATT_TRACE("[P] %s limit_unlock \n",__FUNCTION__);
			shbatt_api_cpu_clock_limit_unlock();
			shbatt_limit_lock_count = 0;
#endif /*SHBATT_ENABLE_LIMIT_LOCK*/

			prev_low_batt_phase = low_batt_phase;
			low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE2;
			shbatt_low_battery_count = 0;
			SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE, SHPWR_LOG_TYPE_BATT,
						"[P] %s() recovery check. next sequence:%d->%d\n", __FUNCTION__,
						prev_low_batt_phase, low_batt_phase );
		}
		memset( &pti, 0x00, sizeof( pti ) );
		pti.ptt	= SHBATT_POLL_TIMER_TYPE_LOW_BATTERY;
		pti.ms	= SHBATT_LOW_BATTERY_CHECK_INTERVAL_MS;
		pti.prm	= 0;
		shbatt_api_set_timer( &pti );
		break;

	case SHBATT_LOW_BATTERY_EVENT_FATAL_TIMER:
		now_voltage		= sh_shbatt_smb1360_get_prp_voltage_now() / 1000;
		if( now_voltage < SHBATT_FATAL_BATTERY_THRESH_VBAT )
		{
			shbatt_fatal_battery_count++;
			if( shbatt_fatal_battery_count >= SHBATT_FATAL_BATTERY_CHECK_COUNT )
			{
				sh_shbatt_smb1360_notify_low_batt();
				prev_low_batt_phase = low_batt_phase;
				low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE4;
				shbatt_fatal_battery_count = 0;
				SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE, SHPWR_LOG_TYPE_BATT,
							"[P] %s() fatal threshold check. next sequence:%d->%d\n", __FUNCTION__,
							prev_low_batt_phase, low_batt_phase );
				break;
			}
		}
		ret = shbatt_low_battery_recovery_check( now_voltage );
		if( ret == SHBATT_RESULT_SUCCESS )
		{
			prev_low_batt_phase = low_batt_phase;
			low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE2;
			shbatt_low_battery_count = 0;
			memset( &pti, 0x00, sizeof( pti ) );
			pti.ptt	= SHBATT_POLL_TIMER_TYPE_LOW_BATTERY;
			pti.ms	= SHBATT_LOW_BATTERY_CHECK_INTERVAL_MS;
			pti.prm	= 0;
			shbatt_api_set_timer( &pti );
			SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE, SHPWR_LOG_TYPE_BATT,
						"[P] %s() recovery check. next sequence:%d->%d\n", __FUNCTION__,
						prev_low_batt_phase, low_batt_phase );
			break;
		}
		memset( &pti, 0x00, sizeof( pti ) );
		pti.ptt	= SHBATT_POLL_TIMER_TYPE_FATAL_BATTERY;
		pti.ms	= SHBATT_FATAL_BATTERY_CHECK_INTERVAL_MS;
		pti.prm	= 0;
		shbatt_api_set_timer( &pti );
		break;

	default:
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[P] %s() other event:%d.\n",__FUNCTION__, evt );
		break;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return;
}

static void shbatt_seq_exec_low_battery_check_sequence_phase4(
	int							evt
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s() evt:%d\n",__FUNCTION__, evt );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return;
}

static enum hrtimer_restart shbatt_seq_fuelgauge_soc_poll_hrtimer_expire_cb(
	struct hrtimer*				hrtimer_p
){
	shbatt_soc_t				soc;
	struct timespec				now_time;
	struct timespec				sleep_time;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	soc.type	= SHBATT_TIMER_TYPE_0;
	soc.sleep	= SHBATT_TIMER_TYPE_SLEEP;

	shbatt_api_exec_fuelgauge_soc_poll_sequence(soc);

	memset( &now_time, 0x00, sizeof(now_time) );
	memset( &sleep_time, 0x00, sizeof(sleep_time) );
	get_monotonic_boottime( &now_time );
	monotonic_to_bootbased( &sleep_time );
	shbatt_last_hrtimer_expire_time = timespec_sub(now_time, sleep_time);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[P] %s():shbatt_last_hrtimer_expire_time=%010lu.%09lu\n",__FUNCTION__,
				 shbatt_last_hrtimer_expire_time.tv_sec, shbatt_last_hrtimer_expire_time.tv_nsec);

	if(shbatt_timer_restarted == true)
	{
		shbatt_timer_restarted = false;
	}


	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart shbatt_seq_fuelgauge_soc_poll_hrtimer_expire_multi_cb(
	struct hrtimer*				hrtimer_p
){
	shbatt_soc_t				soc;
	struct timespec				now_time;
	struct timespec				sleep_time;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	soc.type	= SHBATT_TIMER_TYPE_1;
	soc.sleep	= SHBATT_TIMER_TYPE_SLEEP;

	shbatt_api_exec_fuelgauge_soc_poll_sequence(soc);

	memset( &now_time, 0x00, sizeof(now_time) );
	memset( &sleep_time, 0x00, sizeof(sleep_time) );
	get_monotonic_boottime( &now_time );
	monotonic_to_bootbased( &sleep_time );
	shbatt_last_hrtimer_expire_time = timespec_sub(now_time, sleep_time);


	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[P] %s():shbatt_last_hrtimer_expire_time=%010lu.%09lu\n",__FUNCTION__,
				 shbatt_last_hrtimer_expire_time.tv_sec, shbatt_last_hrtimer_expire_time.tv_nsec);

	if(shbatt_timer_restarted == true)
	{
		shbatt_timer_restarted = false;
	}


	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return HRTIMER_NORESTART;
}

static enum alarmtimer_restart shbatt_seq_fuelgauge_soc_poll_timer_expire_cb(
	struct alarm*				alm_p,
	ktime_t						kerl_time
){
	shbatt_soc_t				soc;
	enum alarmtimer_restart		ret = ALARMTIMER_NORESTART;
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);
	
	soc.type	= SHBATT_TIMER_TYPE_0;
	soc.sleep	= SHBATT_TIMER_TYPE_WAKEUP;

	shbatt_api_exec_fuelgauge_soc_poll_sequence(soc);

	get_monotonic_boottime( &shbatt_last_timer_expire_time );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[P] %s shbatt_last_timer_expire_time.tv_sec=%010lu shbatt_last_timer_expire_time.tv_nsec=%09lu\n",__FUNCTION__,
				 shbatt_last_timer_expire_time.tv_sec, shbatt_last_timer_expire_time.tv_nsec);

	if(shbatt_timer_restarted == true)
	{
		shbatt_timer_restarted = false;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);
	return ret;
}

static enum alarmtimer_restart shbatt_seq_fuelgauge_soc_poll_timer_expire_multi_cb(
	struct alarm*				alm_p,
	ktime_t						kerl_time
){
	shbatt_soc_t				soc;
	enum alarmtimer_restart		ret = ALARMTIMER_NORESTART;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	soc.type = SHBATT_TIMER_TYPE_1;
	soc.sleep	= SHBATT_TIMER_TYPE_WAKEUP;

	shbatt_api_exec_fuelgauge_soc_poll_sequence(soc);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static enum alarmtimer_restart shbatt_seq_low_battery_poll_timer_expire_cb(
	struct alarm*				alm_p,
	ktime_t						kerl_time
){
	enum alarmtimer_restart		ret = ALARMTIMER_NORESTART;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	shbatt_api_exec_low_battery_check_sequence(SHBATT_LOW_BATTERY_EVENT_LOW_TIMER);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static enum alarmtimer_restart shbatt_seq_fatal_battery_poll_timer_expire_cb(
	struct alarm*				alm_p,
	ktime_t						kerl_time
){
	enum alarmtimer_restart		ret = ALARMTIMER_NORESTART;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	shbatt_api_exec_low_battery_check_sequence(SHBATT_LOW_BATTERY_EVENT_FATAL_TIMER);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static shbatt_result_t shbatt_seq_call_user_space_sequence_executor( void )
{
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	INIT_COMPLETION(shbatt_usse_cmp);
	atomic_inc(&shbatt_usse_op_cnt);
	wake_up_interruptible(&shbatt_usse_wait);
	wait_for_completion_killable(&shbatt_usse_cmp);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );


	return shbatt_usse_pkt.hdr.ret;
}

/* wake_lock */
static void shbatt_seq_lock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	strncpy( &shbatt_lock_func[type][0], func, SHBATT_LOCK_FUNC_LEN - 1 );
	get_monotonic_boottime( &shbatt_lock_time[type] );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG, SHPWR_LOG_TYPE_BATT,
				"[P] %s() lock start\n", &shbatt_lock_func[type][0] );

	mutex_lock(&shbatt_task_lock);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);
}

static void shbatt_seq_unlock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func
){
	struct timespec						diff;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	mutex_unlock(&shbatt_task_lock);
	get_monotonic_boottime( &shbatt_unlock_time[type] );

	memset(&diff, 0x00, sizeof( diff ) );
	diff = timespec_sub( shbatt_unlock_time[type], shbatt_lock_time[type] );

	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE, SHPWR_LOG_TYPE_BATT,
				"[P] %s() locktime:%lu.%09lu\n", &shbatt_lock_func[type][0], diff.tv_sec, diff.tv_nsec );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);
}

/* api(Timer) */
static shbatt_result_t shbatt_api_initialize( void )
{
	shbatt_result_t				result = SHBATT_RESULT_SUCCESS;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(shbatt_task_is_initialized == true)
	{
		shbatt_task_is_initialized = false;
	}
	result = shbatt_seq_initialize();

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return result;
}

static shbatt_result_t shbatt_api_exec_fuelgauge_soc_poll_sequence(
	shbatt_soc_t				soc
){
	shbatt_packet_t*			pkt_p;
	ktime_t						set_time;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(shbatt_task_is_initialized == false)
	{
		memset( &set_time,0x00, sizeof( set_time ) );
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"[P] %s() timer restart\n",__FUNCTION__);
		set_time.tv64 = SHBATT_TIMER_FG_PERIOD_NS;
		alarm_start_relative(&(shbatt_poll_timer[SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC].alm.alarm_timer),
					set_time );
		return SHBATT_RESULT_REJECTED;
	}

	pkt_p = shbatt_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHBATT_RESULT_REJECTED;
	}

	SHBATT_WAKE_CTL(1);

	pkt_p->hdr.cmd		= SHBATT_TASK_CMD_EXEC_FUELGAUGE_SOC_POLL_SEQUENCE;
	pkt_p->hdr.cmp_p	= NULL;
	pkt_p->hdr.ret_p	= NULL;
	pkt_p->prm.soc		= soc;

	INIT_WORK((struct work_struct*)pkt_p,shbatt_task);

	queue_work(shbatt_task_workqueue_p,(struct work_struct*)pkt_p);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

static shbatt_result_t shbatt_api_exec_low_battery_check_sequence(
	int							evt
){
	shbatt_packet_t*			pkt_p;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(shbatt_task_is_initialized == false)
	{
		return SHBATT_RESULT_REJECTED;
	}

	pkt_p = shbatt_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHBATT_RESULT_REJECTED;
	}

	SHBATT_WAKE_CTL(1);

	pkt_p->hdr.cmd		= SHBATT_TASK_CMD_EXEC_LOW_BATTERY_CHECK_SEQUENCE;
	pkt_p->hdr.cmp_p	= NULL;
	pkt_p->hdr.ret_p	= NULL;
	pkt_p->prm.evt		= evt;

	INIT_WORK((struct work_struct*)pkt_p,shbatt_task);

	queue_work(shbatt_task_workqueue_p,(struct work_struct*)pkt_p);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_event(
	shbattlog_event_num			evt
){
	shbatt_packet_t*			pkt_p;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(shbatt_task_is_initialized == false)
	{
		return SHBATT_RESULT_REJECTED;
	}

	pkt_p = shbatt_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHBATT_RESULT_REJECTED;
	}

	SHBATT_WAKE_CTL(1);

	pkt_p->hdr.cmd		= SHBATT_TASK_CMD_BATTLOG_EVENT;
	pkt_p->hdr.cmp_p	= NULL;
	pkt_p->hdr.ret_p	= NULL;
	pkt_p->prm.evt		= evt;

	INIT_WORK((struct work_struct*)pkt_p,shbatt_task);

	queue_work(shbatt_task_workqueue_p,(struct work_struct*)pkt_p);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_charge_status(
	int			status
){
	int charge_status_event = SHBATTLOG_EVENT_NONE;
	static int pre_charge_status_event = SHBATTLOG_EVENT_NONE;

	switch (status) {
	case POWER_SUPPLY_STATUS_CHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_START;
		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_END;
		break;
	case POWER_SUPPLY_STATUS_FULL:
		charge_status_event = SHBATTLOG_EVENT_CHG_COMP;
		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_ERROR;
		break;
	default:
		charge_status_event = SHBATTLOG_EVENT_NONE;
	}

	if(charge_status_event != SHBATTLOG_EVENT_NONE && charge_status_event != pre_charge_status_event) {
		SHPWR_LOG_INFO("pre_charge_status_event = %d, charge_status_event = %d\n", pre_charge_status_event, charge_status_event);

		shbatt_api_battlog_event(charge_status_event);
		pre_charge_status_event = charge_status_event;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_charge_error(
	int			charge_error_event
){
	static int pre_charge_error_event = SHBATTLOG_EVENT_NONE;

	if(charge_error_event != SHBATTLOG_EVENT_NONE && charge_error_event != pre_charge_error_event) {
		SHPWR_LOG_INFO("pre_charge_error_event = %d, charge_error_event = %d\n", pre_charge_error_event, charge_error_event);

		shbatt_api_battlog_event(charge_error_event);
		pre_charge_error_event = charge_error_event;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_jeita_status(
	int			jeita_cur_status
){
	static int jeita_pre_status = POWER_SUPPLY_HEALTH_UNKNOWN;

	if (jeita_cur_status  != jeita_pre_status) {
		SHPWR_LOG_INFO("jeita_pre_status = %d,jeita_cur_status  = %d\n", jeita_pre_status , jeita_cur_status );

		switch (jeita_cur_status){
		case POWER_SUPPLY_HEALTH_OVERHEAT:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_HOT_STOP_ST);
			break;
		case POWER_SUPPLY_HEALTH_COLD:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_COLD_STOP_ST);
			break;
		case POWER_SUPPLY_HEALTH_WARM:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_HOT_FAST_ST);
			break;
		case POWER_SUPPLY_HEALTH_COOL:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_COLD_FAST_ST);
			break;
		case POWER_SUPPLY_HEALTH_GOOD:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_FAST_ST);
			break;
		default:
			break;
		}
	}
	jeita_pre_status = jeita_cur_status;

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_capacity(
	int			cur_capacity
){
	static int pre_capacity = 80;
	static const shbattlog_event_num event_tbl[10] =
	{
		SHBATTLOG_EVENT_FGIC_EX10,
		SHBATTLOG_EVENT_FGIC_EX20,
		SHBATTLOG_EVENT_FGIC_EX30,
		SHBATTLOG_EVENT_FGIC_EX40,
		SHBATTLOG_EVENT_FGIC_EX50,
		SHBATTLOG_EVENT_FGIC_EX60,
		SHBATTLOG_EVENT_FGIC_EX70,
		SHBATTLOG_EVENT_FGIC_EX80,
		SHBATTLOG_EVENT_FGIC_EX90,
		SHBATTLOG_EVENT_FGIC_EX100
	};

	if( pre_capacity != cur_capacity ){
		pr_debug("pre_capacity = %d cur_capacity  = %d\n", pre_capacity, cur_capacity );
		
		if(cur_capacity == 0)
		{
			shbatt_api_battlog_event(SHBATTLOG_EVENT_INDICATER_0);
			shbatt_api_battlog_event(SHBATTLOG_EVENT_FATAL_BATT);
		}
		else
		{
			if(cur_capacity % 10 == 0)
			{
				pr_debug("event cur_cap  = %d tbl:%d\n", cur_capacity, event_tbl[(cur_capacity/10)-1] );
				shbatt_api_battlog_event(event_tbl[(cur_capacity/10)-1]);
			}
		}
		pre_capacity = cur_capacity;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_usb_type(
	int			usb_type
){
	int usb_type_event = SHBATTLOG_EVENT_NONE;
	static int pre_usb_type = USB_INVALID_CHARGER;
	struct power_supply*	dc_psy = NULL;
	union power_supply_propval property_dc = {0,};
	static bool dc_present = false;

	pr_debug("usb_type = %d, pre_usb_type = %d\n", usb_type, pre_usb_type);

	if (usb_type == USB_INVALID_CHARGER) {
		pre_usb_type = USB_INVALID_CHARGER;
		dc_present = false;
		return SHBATT_RESULT_SUCCESS;
	}

	if (pre_usb_type == usb_type) {
		return SHBATT_RESULT_SUCCESS;
	}
	
	dc_psy = power_supply_get_by_name("dc");
	if (dc_psy == NULL) {
		return SHBATT_RESULT_SUCCESS;
	}
	dc_psy->get_property(dc_psy, POWER_SUPPLY_PROP_PRESENT, &property_dc);
	pr_info("dc_present = %d\n", property_dc.intval);
	if (property_dc.intval) {
		dc_present = true;
	}

	if (dc_present) {
		return SHBATT_RESULT_SUCCESS;
	}
	
	pre_usb_type = usb_type;

	switch (usb_type) {
	case USB_SDP_CHARGER:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_SDP;
		break;
	case USB_CDP_CHARGER:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_CDP;
		break;
	case USB_DCP_CHARGER:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_DCP;
		break;
	default:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_OTHER;
	}

	SHPWR_LOG_INFO("usb_type = %d, usb_type_event = %d\n", usb_type, usb_type_event);

	shbatt_api_battlog_event(usb_type_event);

	return SHBATT_RESULT_SUCCESS;
}

static shbatt_result_t shbatt_api_get_fuelgauge_current(
	int*						cur_p
){

	shbatt_result_t				result = SHBATT_RESULT_SUCCESS;
	int temp = 0;
	int rc = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(shbatt_api_is_disable_usb_charging()){
		rc = sh_shbatt_smb1360_enable_fg_access();
		if (rc) {
			pr_err("Couldn't request SHDW_FG_CURR_NOW access rc=%d\n", rc);
			return rc;
		}
	}

	temp = sh_shbatt_smb1360_get_prp_current_now();

	*cur_p = -temp;

	if(shbatt_api_is_disable_usb_charging()){
		rc = sh_shbatt_smb1360_disable_fg_access();
		if (rc) {
			pr_err("Couldn't disable SHDW_FG_CURR_NOW access rc=%d\n", rc);
			return rc;
		}
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return result;
}

static shbatt_result_t shbatt_api_initialize_fuelgauge_calibration_data( void )
{
	shbatt_result_t				result = SHBATT_RESULT_SUCCESS;
	sharp_smem_common_type*		p_smem = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	p_smem = sh_smem_get_common_address();

	if( p_smem ){
		memcpy(fg_calibration_data, p_smem->shpwr_fuel_data, sizeof(fg_calibration_data));
	}
	else
	{
		result = SHBATT_RESULT_FAIL;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return result;
}

shbatt_result_t shbatt_api_get_battery_log_info(
	shbatt_batt_log_info_t*		bli_p
){
	int							ret;
	union power_supply_propval	val = {0,};
	struct power_supply*		batt_psy = NULL;
	struct qpnp_vadc_result		vadc_val;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(shbatt_task_is_initialized == false)
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() shbatt_task_is_initialized is false. \n", __FUNCTION__ );
		return SHBATT_RESULT_REJECTED;
	}
	batt_psy = power_supply_get_by_name("battery");
	if( batt_psy == NULL )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() batt_psy is NULL.\n", __FUNCTION__ );
		return SHBATT_RESULT_REJECTED;
	}
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s() power_supply_get_by_name()\n",__FUNCTION__);

	ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d POWER_SUPPLY_PROP_VOLTAGE_NOW is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->bat_vol = val.intval / 1000;
	bli_p->avg_vol = val.intval / 1000;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()val.intval:%d. bat_vol:%d.avg_vol:%d.\n",__FUNCTION__,
				val.intval, bli_p->bat_vol, bli_p->avg_vol );
	val.intval = 0;

	ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_TEMP, &val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d POWER_SUPPLY_PROP_TEMP is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->bat_temp = val.intval / 10;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()val.intval:%d. bat_temp:%d.\n",__FUNCTION__,
				val.intval, bli_p->bat_temp );
	val.intval = 0;

	if ((bli_p->event_num == SHBATTLOG_EVENT_INDICATER_0)
		|| (bli_p->event_num == SHBATTLOG_EVENT_FATAL_BATT)) {
		val.intval = 0;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX10) {
		val.intval = 10;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX20) {
		val.intval = 20;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX30) {
		val.intval = 30;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX40) {
		val.intval = 40;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX50) {
		val.intval = 50;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX60) {
		val.intval = 60;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX70) {
		val.intval = 70;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX80) {
		val.intval = 80;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX90) {
		val.intval = 90;
	} else if (bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX100) {
		val.intval = 100;
	} else {	
		ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, &val );
		if( ret < 0 )
		{
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
						"[E] %s() ret:%d POWER_SUPPLY_PROP_CAPACITY is failed.\n",__FUNCTION__, ret );
			return SHBATT_RESULT_FAIL;
		}
	}
	bli_p->vol_per = val.intval;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()val.intval:%d. vol_per:%d.\n",__FUNCTION__,
				val.intval, bli_p->vol_per );
	val.intval = 0;

	memset( &vadc_val, 0x00, sizeof( vadc_val ) );
	ret = shbatt_vadc_channel_read( USBIN, &vadc_val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d shbatt_vadc_channel_read() USBIN is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->chg_vol = (int)vadc_val.physical / 1000;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()vadc_val.physical:%ld. chg_vol:%d.\n",__FUNCTION__,
				(long)vadc_val.physical, bli_p->chg_vol );

	memset( &vadc_val, 0x00, sizeof( vadc_val ) );
	ret = shbatt_vadc_channel_read( P_MUX2_1_1, &vadc_val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d shbatt_vadc_channel_read() P_MUX2_1_1 is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->cpu_temp = (int)vadc_val.physical;
	bli_p->chg_temp = (int)vadc_val.physical;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()vadc_val.physical:%ld. cpu_temp:%d. chg_temp:%d.\n",__FUNCTION__,
				(long)vadc_val.physical, bli_p->cpu_temp, bli_p->chg_temp );

	memset( &vadc_val, 0x00, sizeof( vadc_val ) );
	ret = shbatt_vadc_channel_read( DIE_TEMP, &vadc_val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d shbatt_vadc_channel_read() DIE_TEMP is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->pmic_temp = (int)vadc_val.physical / 1000;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()vadc_val.physical:%ld. pmic_temp:%d.\n",__FUNCTION__,
				(long)vadc_val.physical, bli_p->pmic_temp );

	memset( &vadc_val, 0x00, sizeof( vadc_val ) );
	ret = shbatt_vadc_channel_read( LR_MUX7_HW_ID, &vadc_val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d shbatt_vadc_channel_read() LR_MUX7_HW_ID is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->pa_temp = (int)vadc_val.physical;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()vadc_val.physical:%ld. pa_temp:%d.\n",__FUNCTION__,
				(long)vadc_val.physical, bli_p->pa_temp );

	val.intval = 0;

	ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d POWER_SUPPLY_PROP_CURRENT_NOW is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->latest_cur = -val.intval / 1000;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()val.intval:%d. latest_cur:%d.\n",__FUNCTION__,
				-val.intval, bli_p->latest_cur );

	if ((bli_p->event_num == SHBATTLOG_EVENT_NONE)	// SHBATTLOG_EVENT_BATT_REPORT_NORM or SHBATTLOG_EVENT_BATT_REPORT_CHG
		||(bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX10)
		||(bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX20)
		||(bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX30)
		||(bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX40)
		||(bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX50)
		||(bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX60)
		||(bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX70)
		||(bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX80)
		||(bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX90)
		||(bli_p->event_num == SHBATTLOG_EVENT_FGIC_EX100)
		||(bli_p->event_num == SHBATTLOG_EVENT_CHG_COLD_FAST_ST)) {

		val.intval = 0;

		ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_CURRENT_AVG, &val );
		if( ret < 0 )
		{
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
						"[E] %s() ret:%d POWER_SUPPLY_PROP_CURRENT_AVG is failed.\n",__FUNCTION__, ret );
			return SHBATT_RESULT_FAIL;
		}
		bli_p->avg_cur = -val.intval / 1000;
		SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
					"[P] %s()val.intval:%d. avg_cur:%d.\n",__FUNCTION__,
					-val.intval, bli_p->avg_cur );
		val.intval = 0;
		batt_psy->set_property(batt_psy, POWER_SUPPLY_PROP_CURRENT_AVG, &val );
	}

	val.intval = 0;

	ret = batt_psy->get_property(batt_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, &val );
	if( ret < 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[E] %s() ret:%d POWER_SUPPLY_PROP_INPUT_CURRENT_MAX is failed.\n",__FUNCTION__, ret );
		return SHBATT_RESULT_FAIL;
	}
	bli_p->chg_cur = val.intval;
	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE,SHPWR_LOG_TYPE_BATT,
				"[P] %s()val.intval:%d. chg_cur:%d.\n",__FUNCTION__,
				val.intval, bli_p->chg_cur );
	val.intval = 0;

	bli_p->cam_temp = 0;
	bli_p->lcd_temp = 0;
	bli_p->acc_cur = 0;
	bli_p->cur_dep_per = 0;
	bli_p->avg_dep_per = 0;

	if (bli_p->event_num == SHBATTLOG_EVENT_NONE)	// SHBATTLOG_EVENT_BATT_REPORT_NORM or SHBATTLOG_EVENT_BATT_REPORT_CHG
		sh_dump_regs();

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

static shbatt_result_t shbatt_seq_battlog_event(
	int		evt
) {
	shbattlog_info_t			shterm_bli;
	shbatt_batt_log_info_t		bli;
	
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	memset(&shterm_bli, 0, sizeof(shterm_bli));
	shterm_bli.event_num = evt;

	memset(&bli, 0, sizeof(bli));
	bli.event_num = evt;

	switch( shterm_bli.event_num )
	{
	case SHBATTLOG_EVENT_CHG_INSERT_USB:
	case SHBATTLOG_EVENT_CHG_REMOVE_USB:
	case SHBATTLOG_EVENT_CHG_REMOVE_CRADLE:
	case SHBATTLOG_EVENT_CHG_PUT_CRADLE:
	case SHBATTLOG_EVENT_CHG_START:
	case SHBATTLOG_EVENT_CHG_END:
	case SHBATTLOG_EVENT_CHG_ERROR:
	case SHBATTLOG_EVENT_CHG_COMP:
	case SHBATTLOG_EVENT_CHG_ERR_BD_BAT_UNUSUAL_ST:
	case SHBATTLOG_EVENT_CHG_ERR_BD_CHG_UNUSUAL_ST:
	case SHBATTLOG_EVENT_CHG_ERR_CHG_POWER_SHORTAGE_ST:
	case SHBATTLOG_EVENT_CHG_FAST_ST:
	case SHBATTLOG_EVENT_CHG_HOT_FAST_ST:
	case SHBATTLOG_EVENT_CHG_HOT_STOP_ST:
	case SHBATTLOG_EVENT_CHG_COLD_STOP_ST:
	case SHBATTLOG_EVENT_CHG_COUNT_OVER_STOP_ST:
	case SHBATTLOG_EVENT_INDICATER_0:
	case SHBATTLOG_EVENT_FATAL_BATT:
	case SHBATTLOG_EVENT_FGIC_EX10:
	case SHBATTLOG_EVENT_FGIC_EX20:
	case SHBATTLOG_EVENT_FGIC_EX30:
	case SHBATTLOG_EVENT_FGIC_EX40:
	case SHBATTLOG_EVENT_FGIC_EX50:
	case SHBATTLOG_EVENT_FGIC_EX60:
	case SHBATTLOG_EVENT_FGIC_EX70:
	case SHBATTLOG_EVENT_FGIC_EX80:
	case SHBATTLOG_EVENT_FGIC_EX90:
	case SHBATTLOG_EVENT_FGIC_EX100:
	case SHBATTLOG_EVENT_CHG_TYPE_SDP:
	case SHBATTLOG_EVENT_CHG_TYPE_CDP:
	case SHBATTLOG_EVENT_CHG_TYPE_DCP:
	case SHBATTLOG_EVENT_CHG_TYPE_HVDCP:
	case SHBATTLOG_EVENT_CHG_TYPE_OTHER:
		if( shbatt_api_get_battery_log_info(&bli) == SHBATT_RESULT_SUCCESS )
		{
			shterm_bli.bat_vol     = bli.bat_vol;
			shterm_bli.chg_vol     = bli.chg_vol;
			shterm_bli.chg_cur     = bli.chg_cur;
			shterm_bli.bat_temp    = bli.bat_temp;
			shterm_bli.cpu_temp    = bli.cpu_temp;
			shterm_bli.chg_temp    = bli.chg_temp;
			shterm_bli.cam_temp    = bli.cam_temp;
			shterm_bli.pmic_temp   = bli.pmic_temp;
			shterm_bli.pa_temp     = bli.pa_temp;
			shterm_bli.lcd_temp    = bli.lcd_temp;
			shterm_bli.avg_cur     = bli.avg_cur;
			shterm_bli.avg_vol     = bli.avg_vol;
			shterm_bli.latest_cur  = bli.latest_cur;
			shterm_bli.acc_cur     = bli.acc_cur;
			shterm_bli.vol_per     = bli.vol_per;
			shterm_bli.cur_dep_per = bli.cur_dep_per;
			shterm_bli.avg_dep_per = bli.avg_dep_per;
		}
		else
		{
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
						"[P] %s() event:%d. shbatt_api_get_battery_log_info() failed.\n", __FUNCTION__,
						shterm_bli.event_num );
			// data error
			memset(&shterm_bli, 0, sizeof(shterm_bli));
			shterm_bli.event_num = evt;
		}
		break;

	default:
		memset(&shterm_bli, 0, sizeof(shterm_bli));
		shterm_bli.event_num = evt;
		break;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:chg_vol    = %d\n",shterm_bli.chg_vol     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:chg_cur    = %d\n",shterm_bli.chg_cur     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:bat_temp   = %d\n",shterm_bli.bat_temp    );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:cpu_temp   = %d\n",shterm_bli.cpu_temp    );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:chg_temp   = %d\n",shterm_bli.chg_temp    );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:cam_temp   = %d\n",shterm_bli.cam_temp    );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:pmic_temp  = %d\n",shterm_bli.pmic_temp   );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:pa_temp    = %d\n",shterm_bli.pa_temp     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:lcd_temp   = %d\n",shterm_bli.lcd_temp    );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:avg_cur    = %d\n",shterm_bli.avg_cur     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:avg_vol    = %d\n",shterm_bli.avg_vol     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:latest_cur = %d\n",shterm_bli.latest_cur  );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:acc_cur    = %d\n",shterm_bli.acc_cur     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:vol_per    = %d\n",shterm_bli.vol_per     );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:cur_dep_per= %d\n",shterm_bli.cur_dep_per );
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,"#37196_E:avg_dep_per= %d\n",shterm_bli.avg_dep_per );

	shterm_k_set_event(&shterm_bli);


	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

static shbatt_result_t shbatt_api_initialize_bat_calibration_data( void )
{
	shbatt_result_t				result = SHBATT_RESULT_SUCCESS;
	sharp_smem_common_type*		p_smem = 0;
	
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);
	
	p_smem = sh_smem_get_common_address();
	
	if( p_smem )
	{
		memcpy(bat_calibration_data, p_smem->shpwr_vbat_data, sizeof(bat_calibration_data));
	}
	else
	{
		result = SHBATT_RESULT_FAIL;
	}
	
	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
	
	return result;
}
static void shbatt_api_set_timer(
	shbatt_poll_timer_info_t*	pti
){
	ktime_t						set_time;
	struct timespec				now_time;
	struct timespec				sleep_time;
	struct timespec				boot_time;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	switch( pti->ptt )
	{
	case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_SLEEP:
	case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_SLEEP_MULTI:
		memset( &set_time, 0x00, sizeof( set_time ) );
		memset( &now_time, 0x00, sizeof( now_time ) );
		memset( &sleep_time, 0x00, sizeof( sleep_time ) );
		memset( &boot_time, 0x00, sizeof( boot_time ) );

		get_monotonic_boottime( &now_time );
		monotonic_to_bootbased( &sleep_time );

		boot_time = timespec_sub( now_time, sleep_time );

		set_time = timespec_to_ktime( boot_time );
		set_time = ktime_add_ns( set_time, SHBATT_TIMER_MSEC_TO_NSEC( pti->ms ) );
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"[P] %s():type:%d expire_time=%010lu.%09lu + %d\n",__FUNCTION__, pti->ptt,
					boot_time.tv_sec, boot_time.tv_nsec, pti->ms );

		shbatt_poll_timer[pti->ptt].prm = pti->prm;
		hrtimer_cancel( &(shbatt_poll_timer[pti->ptt].alm.hr_timer) );
		hrtimer_start( &(shbatt_poll_timer[pti->ptt].alm.hr_timer),
					set_time, HRTIMER_MODE_ABS );
		break;
	case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC:
	case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_MULTI:
	case SHBATT_POLL_TIMER_TYPE_LOW_BATTERY:
	case SHBATT_POLL_TIMER_TYPE_FATAL_BATTERY:

		memset( &now_time, 0x00, sizeof( now_time ) );
		get_monotonic_boottime( &now_time );
		SHPWR_LOG(SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT, "[debug]%s():Now time sec=%010lu,nsec=%09lu, ms=%d\n", 
					__FUNCTION__, now_time.tv_sec, now_time.tv_nsec, pti->ms );
		if(pti->ms >= 1000)
		{
			now_time.tv_sec += (pti->ms / 1000);
			now_time.tv_nsec += SHBATT_TIMER_MSEC_TO_NSEC(pti->ms % 1000);
		}
		else
		{
			now_time.tv_nsec += SHBATT_TIMER_MSEC_TO_NSEC(pti->ms);
		}
		
		SHPWR_LOG(SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT, "[debug]%s():Set time sec=%010lu,nsec=%09lu.\n", 
					__FUNCTION__, now_time.tv_sec, now_time.tv_nsec );


		shbatt_poll_timer[pti->ptt].prm = pti->prm;
		alarm_cancel( &(shbatt_poll_timer[pti->ptt].alm.alarm_timer) );

		alarm_start( &(shbatt_poll_timer[pti->ptt].alm.alarm_timer), timespec_to_ktime(now_time) );
		break;

	default:
		SHPWR_LOG(SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
				"[P]%s() : timer type invalid:%d.\n", __FUNCTION__, pti->ptt );
		break;
	}
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return;
}

static shbatt_result_t shbatt_api_low_battery_low_threshold_check(
	int							vbat
){
	shbatt_result_t				ret = SHBATT_RESULT_SUCCESS;
	shbatt_low_battery_state_t	prev_low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE1;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);
	if( vbat < SHBATT_LOW_BATTERY_THRESH_VBAT )
	{
		shbatt_low_battery_count++;

#ifdef SHBATT_ENABLE_LIMIT_LOCK
		if(shbatt_limit_lock_count < shbatt_limit_lock_level_table_size)
		{
			shbatt_limit_lock_count++;
			shbatt_api_cpu_clock_limit_lock(shbatt_limit_lock_level_table[shbatt_limit_lock_count-1]);
			SHBATT_TRACE("[P] %s limit_lock_level=%d \n",__FUNCTION__, shbatt_limit_lock_count);
		}
#endif /*SHBATT_ENABLE_LIMIT_LOCK*/

		if( shbatt_low_battery_count >= SHBATT_LOW_BATTERY_CHECK_COUNT )
		{
			sh_shbatt_smb1360_notify_low_batt();
			prev_low_batt_phase = low_batt_phase;
			low_batt_phase = SHBATT_LOW_BATTERY_SEQUENCE_PHASE4;
			shbatt_low_battery_count = 0;
			ret = SHBATT_RESULT_FAIL;
			SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE, SHPWR_LOG_TYPE_BATT,
						"[P] %s() low threshold check. next sequence:%d->%d\n", __FUNCTION__,
						prev_low_batt_phase, low_batt_phase );
		}
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s() result:%d\n", __FUNCTION__, ret );

	return ret;
}

static shbatt_result_t shbatt_low_battery_recovery_check(
	int							vbat
){
	shbatt_result_t				ret = SHBATT_RESULT_SUCCESS;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if( vbat > SHBATT_LOW_BATTERY_RECOVERY_THRESH_VBAT )
	{
		ret = SHBATT_RESULT_SUCCESS;
	}
	else
	{
		ret = SHBATT_RESULT_FAIL;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s() result:%d\n", __FUNCTION__, ret );

	return ret;
}

/*from ioctl.*/
static int shbatt_drv_ioctl_cmd_initialize(
	struct file*				fi_p,
	unsigned					long arg
){
	int							ret;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	ret = shbatt_api_initialize();

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static int shbatt_drv_ioctl_cmd_pull_usse_packet(
	struct file*				fi_p,
	unsigned long				arg
){
	shbatt_usse_packet_t*		pkt_p;
	int							ret = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	pkt_p = &shbatt_usse_pkt;

	if(copy_to_user((shbatt_usse_packet_t*)arg,pkt_p,sizeof(shbatt_usse_packet_t)) != 0)
	{
		SHBATT_ERROR("%s : copy_to_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static int shbatt_drv_ioctl_cmd_done_usse_packet(
	struct file*				fi_p,
	unsigned long				arg
){
	shbatt_usse_packet_t*		pkt_p;
	int							ret = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	pkt_p = &shbatt_usse_pkt;

	if(copy_from_user(pkt_p,(shbatt_usse_packet_t*)arg,sizeof(shbatt_usse_packet_t)) != 0)
	{
		SHBATT_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static int shbatt_drv_ioctl_cmd_set_timer(
	struct file*				fi_p,
	unsigned long				arg
){
	shbatt_poll_timer_info_t	pti;
	int							ret = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if( copy_from_user(&pti,(shbatt_poll_timer_info_t*)arg,sizeof(shbatt_poll_timer_info_t)) != 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR, SHPWR_LOG_TYPE_BATT,
					"[P] %s() : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		switch( pti.ptt )
		{
		case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_SLEEP:
		case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_SLEEP_MULTI:
		case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC:
		case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_MULTI:
		case SHBATT_POLL_TIMER_TYPE_LOW_BATTERY:
		case SHBATT_POLL_TIMER_TYPE_FATAL_BATTERY:
			shbatt_api_set_timer( &pti );
			break;

		default:
			SHPWR_LOG(SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
					"[P] %s() : timer type invalid.\n", __FUNCTION__ );
			ret = -EINVAL;
			break;
		}
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s() ret:%d\n",__FUNCTION__, ret );

	return ret;
}

static int shbatt_drv_ioctl_cmd_clr_timer(
	struct file*				fi_p,
	unsigned long				arg
){
	shbatt_poll_timer_info_t	pti;
	int							ret = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	if(copy_from_user(&pti,(shbatt_poll_timer_info_t*)arg,sizeof(shbatt_poll_timer_info_t)) != 0)
	{
		SHBATT_ERROR("%s : copy_from_user failed.\n",__FUNCTION__);
		ret = -EPERM;
	}
	else
	{
		switch( pti.ptt ) {
		case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_SLEEP:
		case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_SLEEP_MULTI:
			hrtimer_cancel( &(shbatt_poll_timer[pti.ptt].alm.hr_timer) );
			break;

		case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC:
		case SHBATT_POLL_TIMER_TYPE_FUELGAUGE_SOC_MULTI:
		/* low batt */
		case SHBATT_POLL_TIMER_TYPE_LOW_BATTERY:
		case SHBATT_POLL_TIMER_TYPE_FATAL_BATTERY:
			alarm_cancel( &(shbatt_poll_timer[pti.ptt].alm.alarm_timer) );
			break;

		default:
			SHBATT_ERROR("%s : timer type invalid.\n",__FUNCTION__);
			ret = -EPERM;
			break;
		}
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static int shbatt_drv_ioctl_cmd_get_boot_time(
	struct file*				fi_p,
	unsigned long				arg
){
	int							result = 0;
	struct timespec				ts;
	shbatt_boottime_info_t		time_t;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	memset( &ts, 0x00, sizeof( ts ) );
	get_monotonic_boottime( &ts );

	time_t.boot_sec	= ts.tv_sec;
	time_t.boot_nsec= ts.tv_nsec;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
			"[P] %s(): tv_sec=%lu tv_nsec=%lu sec=%lld nsec=%lld\n",__FUNCTION__,
			ts.tv_sec, ts.tv_nsec, time_t.boot_sec, time_t.boot_nsec );

	result = copy_to_user( (shbatt_boottime_info_t __user *)arg, &time_t, sizeof( shbatt_boottime_info_t ) );
	if( result != 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
			"[E] %s(): copy_to_user failed.\n",__FUNCTION__);
		return -EPERM;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return 0;
}

static int shbatt_drv_ioctl_cmd_get_shbatt_log_info(
	struct file*				fi_p,
	unsigned long				arg
){
	int							result = 0;
	shbatt_log_info_t info;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	result = smb1360_get_log_info(&info);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT, "[P] %s(): result=%d\n",__FUNCTION__,result);

	result = copy_to_user( (shbatt_log_info_t __user *)arg, &info, sizeof( shbatt_log_info_t ) );
	if( result != 0 )
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
			"[E] %s(): copy_to_user failed.\n",__FUNCTION__);
		return -EPERM;
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return 0;
}

static int shbatt_drv_create_ps_device_file(
	struct device*				dev_p,
	struct device_attribute*	attr_p
){
	int							ret;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	ret = device_create_file(dev_p, attr_p);

	if (ret == -EEXIST)
	{
		SHBATT_ERROR("%s : already exists %s property register\n", __FUNCTION__, attr_p->attr.name);
		ret = 0;
	}
	else
	if (ret < 0)
	{
		device_remove_file(dev_p, attr_p);

		SHBATT_ERROR("%s : failed to %s property register = %d\n", __FUNCTION__, attr_p->attr.name, ret);
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return ret;
}

static int shbatt_drv_create_ps_attrs(
	struct platform_device*		dev_p
){
	struct device_attribute*	tmp_attr;
	int							idx;
	int							attr_cntr;
	int							ret;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);
	
	for(idx = 0; idx < ARRAY_SIZE(shbatt_power_supplies); idx++)
	{
		ret = power_supply_register( &dev_p->dev, &(shbatt_power_supplies[idx].psy) );
		if( ret < 0 )
		{
			SHBATT_ERROR("%s : failed to power supply register = %d\n", __FUNCTION__, ret);
			return ret;
		}

		tmp_attr = shbatt_power_supplies[idx].attr_array;
		for( attr_cntr = 0; strcmp(tmp_attr->attr.name,SHBATT_ATTR_ARRAY_END_NAME ) != 0; attr_cntr++ ) {
			ret = shbatt_drv_create_ps_device_file( shbatt_power_supplies[idx].psy.dev, tmp_attr );
			if( ret < 0 )
			{
				SHBATT_ERROR( "%s : failed to attribute register = %d, idx=%d, attr=%d\n",
								__FUNCTION__, ret, idx, attr_cntr );
				return ret;
			}
			tmp_attr++;
		}
	}

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE CODE AREA :                                        |*/
/*+-----------------------------------------------------------------------------+*/
static ssize_t shbatt_drv_store_property(
	struct device*				dev_p,
	struct device_attribute*	attr_p,
	const char*					buf_p,
	size_t						size
){
	SHBATT_INFO("%s,dev_p=0x%p,attr_p=0x%p,buf_p=%s,size=0x%x", __FUNCTION__, dev_p, attr_p, buf_p, size);
	return size;
}

static ssize_t shbatt_drv_store_fuelgauge_property(
	struct device*				dev_p,
	struct device_attribute*	attr_p,
	const char*					buf_p,
	size_t						size
){
	
	
	const ptrdiff_t				property = attr_p - shbatt_fuelgauge_attributes;
	int							value;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s() attr_p:0x%p fg_attr_p:0x%p attr_size:0x%x property:%d\n", __FUNCTION__,
				attr_p, shbatt_fuelgauge_attributes, sizeof( struct device_attribute ), property );

	switch( property )
	{
	case SHBATT_PS_PROPERTY_LOG_DEBUG:
		value = simple_strtoul( buf_p, NULL, 10 );
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"[P] %s() /shbatt_fuelgauge/log_debug = %d\n", __FUNCTION__, value);
		log_event = value;
		break;

	default:
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
					"[P]%s() unknown property:%d.\n", __FUNCTION__, property );
		size = -1;
		break;
	}
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
			"[E] %s()\n", __FUNCTION__ );

	return size;
}

static ssize_t shbatt_drv_show_fuelgauge_property(
	struct device*				dev_p,
	struct device_attribute*	attr_p,
	char*						buf_p
){
	const ptrdiff_t				property = attr_p - shbatt_fuelgauge_attributes;
	ssize_t						size = -1;
	shbatt_result_t				ret;
	shbatt_batt_log_info_t		log_info;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s() attr_p:0x%p fg_attr_p:0x%p attr_size:0x%x property:%d\n", __FUNCTION__,
				attr_p, shbatt_fuelgauge_attributes, sizeof( struct device_attribute ), property );

	switch(property)
	{
	case SHBATT_PS_PROPERTY_CURRENT:
		{
			int cur = 0;
			
			ret = shbatt_api_get_fuelgauge_current(&cur);
			if( ret == SHBATT_RESULT_SUCCESS )
			{
				SHBATT_TRACE("/shbatt_fuelgauge/current = %d\n", cur);
				size = snprintf(buf_p, PAGE_SIZE, "%d\n",cur);
			}
		}
		break;

	case SHBATT_PS_PROPERTY_LOG_DEBUG:
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"cat /shbatt_fuelgauge/log_debug = %d\n", log_event);
		memset( &log_info, 0x00, sizeof( log_info ) );
		log_info.event_num = log_event;
		ret = shbatt_api_get_battery_log_info( &log_info );
		if( ret == SHBATT_RESULT_SUCCESS )
		{
			size = snprintf( buf_p, PAGE_SIZE,
							"batvol:%d\nchg_vol:%d\nchg_cur:%d\nbat_temp:%d\ncpu_temp:%d\nchg_temp:%d\ncam_temp:%d\n"
							"pmic_temp:%d\npa_temp:%d\nlcd_temp:%d\navg_cur:%d\navg_vol:%d\n"
							"latest_cur:%d\nacc_cur:%d\nvol_per:%d\ncur_dep_per:%d\navg_dep_per:%d\n",
							log_info.bat_vol, log_info.chg_vol, log_info.chg_cur, log_info.bat_temp, log_info.cpu_temp, log_info.chg_temp,
							log_info.cam_temp, log_info.pmic_temp, log_info.pa_temp, log_info.lcd_temp, log_info.avg_cur, log_info.avg_vol,
							log_info.latest_cur, log_info.acc_cur, log_info.vol_per, log_info.cur_dep_per, log_info.avg_dep_per );
		}
		break;

	default:
		SHBATT_ERROR("[P] %s attr = %d\n",__FUNCTION__,property);
		break;
	}
	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
	return size;
}
static ssize_t shbatt_drv_show_adc_property(
	struct device*				dev_p,
	struct device_attribute*	attr_p,
	char*						buf_p
){
	const ptrdiff_t				property = attr_p - shbatt_adc_attributes;
	ssize_t						size = -1;
	int							temp = 10;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);
	switch(property)
	{
	case SHBATT_PS_PROPERTY_CPU_TEMP:
		if (debug_cpu_temp)
		{
			SHBATT_TRACE("/adc/cpu_temp = %d\n", debug_cpu_temp);
			size = snprintf(buf_p, PAGE_SIZE, "%d\n", debug_cpu_temp);
			break;
		}
		SHBATT_ERROR("/adc/cpu_temp error\n");
		break;

	case SHBATT_PS_PROPERTY_LCD_TEMP:
		if (debug_lcd_temp)
		{
			SHBATT_TRACE("/adc/lcd_temp = %d\n", debug_lcd_temp);
			size = snprintf(buf_p, PAGE_SIZE, "%d\n", debug_lcd_temp);
		}
		else
		{
			SHBATT_TRACE("/adc/lcd_temp = %d\n", temp);
			size = snprintf(buf_p, PAGE_SIZE, "%d\n", temp);
		}
		break;

	case SHBATT_PS_PROPERTY_PA0_TEMP:
		if (debug_pa0_temp)
		{
			SHBATT_TRACE("/adc/debug_pa0_temp = %d\n", debug_pa0_temp);
			size = snprintf(buf_p, PAGE_SIZE, "%d\n", debug_pa0_temp);
		}
		else
		{
			SHBATT_TRACE("/adc/pa0_temp = %d\n", temp);
			size = snprintf(buf_p, PAGE_SIZE, "%d\n", temp);
		}
		break;

	case SHBATT_PS_PROPERTY_CAM_TEMP:
		if (debug_cam_temp)
		{
			SHBATT_TRACE("/adc/debug_cam_temp = %d\n", debug_cam_temp);
			size = snprintf(buf_p, PAGE_SIZE, "%d\n", debug_cam_temp);
		} else {
			SHBATT_TRACE("/adc/cam_temp = %d\n", temp);
			size = snprintf(buf_p, PAGE_SIZE, "%d\n", temp);
		}
		break;

	default:
		SHBATT_ERROR("[P] %s attr = %d\n",__FUNCTION__,property);
		break;
	}

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
	return size;
}

static int shbatt_drv_open(
	struct inode*				in_p,
	struct file*				fi_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static int shbatt_drv_release(
	struct inode*				in_p,
	struct file*				fi_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static unsigned int shbatt_drv_poll(
	struct file*				fi_p,
	poll_table*					wait_p
){
	unsigned					int mask = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	if(atomic_read(&shbatt_usse_op_cnt) > 0)
	{
		atomic_dec(&shbatt_usse_op_cnt);

		mask = POLLIN;
	}
	else
	{
		poll_wait(fi_p,&shbatt_usse_wait,wait_p);
		complete(&shbatt_usse_cmp);
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return mask;
}

static long shbatt_drv_ioctl(
	struct file*				fi_p,
	unsigned int				cmd,
	unsigned long				arg
){
	int							ret = -EPERM;

	SHBATT_TRACE("[S] %s() cmd=0x%x\n",__FUNCTION__,cmd);

	switch(cmd) {
	case SHBATT_DRV_IOCTL_CMD_INITIALIZE:
		ret = shbatt_drv_ioctl_cmd_initialize( fi_p, arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_PULL_USSE_PACKET:
		ret = shbatt_drv_ioctl_cmd_pull_usse_packet( fi_p, arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_DONE_USSE_PACKET:
		ret = shbatt_drv_ioctl_cmd_done_usse_packet( fi_p, arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_SET_TIMER:
		ret = shbatt_drv_ioctl_cmd_set_timer( fi_p,arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_CLR_TIMER:
		ret = shbatt_drv_ioctl_cmd_clr_timer( fi_p,arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_GET_BOOT_TIME:
		ret = shbatt_drv_ioctl_cmd_get_boot_time( fi_p,arg );
		break;

	case SHBATT_DRV_IOCTL_CMD_GET_SHBATT_LOG_INFO:
		ret = shbatt_drv_ioctl_cmd_get_shbatt_log_info( fi_p,arg );
		break;

	default:
		SHBATT_ERROR( "[P] %s(): bad cmd. 0x%x\n", __FUNCTION__, cmd );
		ret = -EINVAL;
		break;
	}

	SHBATT_TRACE("[E] %s() ret=%d \n",__FUNCTION__,ret);

	return ret;
}

static int shbatt_drv_probe(
	struct platform_device*		dev_p
){
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	if(shbatt_drv_create_ps_attrs(dev_p) < 0)
	{
		SHBATT_ERROR("%s : create attrs failed.\n",__FUNCTION__);
		return -EPERM;
	}

	if(shbatt_drv_create_device() < 0)
	{
		SHBATT_ERROR("%s : create device failed.\n",__FUNCTION__);
		return -EPERM;
	}

	shbatt_api_initialize_bat_calibration_data();

	shbatt_api_initialize_fuelgauge_calibration_data();

	shbatt_task_is_initialized = true;

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static int shbatt_drv_remove(
	struct platform_device*		dev_p
){
	int							idx;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	for(idx = 0; idx < ARRAY_SIZE(shbatt_power_supplies); idx++)
	{
		power_supply_unregister( &(shbatt_power_supplies[idx].psy) );
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static void shbatt_drv_shutdown(
	struct platform_device*		dev_p
){
	int							alm_cnt;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_task_is_initialized = false;

	for( alm_cnt = 0; alm_cnt < NUM_SHBATT_POLL_TIMER_TYPE; alm_cnt++ )
	{
		switch(shbatt_poll_timer[alm_cnt].timer_type)
		{
		case SHBATT_TIMER_TYPE_HRTIMER:
			hrtimer_cancel(&(shbatt_poll_timer[alm_cnt].alm.hr_timer));
			break;

		case SHBATT_TIMER_TYPE_ALARMTIMER:
			alarm_cancel( &(shbatt_poll_timer[alm_cnt].alm.alarm_timer) );
			break;

		default:
		SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
					"[E] %s(): alarm type err. default=%d \n",__FUNCTION__, shbatt_poll_timer[alm_cnt].timer_type);
			break;
		}
	}

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static int shbatt_drv_resume(
	struct platform_device*		dev_p
){
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
	
	return 0;
}

static int shbatt_drv_suspend(
	struct platform_device*		dev_p,
	pm_message_t				state
){
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
	return 0;

}

static int shbatt_drv_get_ps_property(
	struct power_supply*		psy_p,
	enum power_supply_property	psp,
	union power_supply_propval*	val_p
){
	int							ret = 0;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shbatt_drv_set_ps_property(
	struct power_supply*		psy_p,
	enum power_supply_property	psp,
	const union power_supply_propval*	val_p
){
	int							ret = 0;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	ret = shbatt_ps_battery_set_property(psy_p, psp, val_p);
	if( ret < 0 )
	{
		SHBATT_ERROR("[P] %s ret = %d\n", __FUNCTION__, ret );
		ret = -EINVAL;
	}

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shbatt_ps_battery_set_property(
	struct power_supply*		psy,
	enum power_supply_property	psp,
	const union power_supply_propval*	val
){
	int							ret = 0;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

#ifdef SHBATT_ENABLE_LIMIT_LOCK
static int shbatt_api_cpu_clock_limit_lock( shbatt_limit_lock_level_t level )
{
	int ret = 0;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	if(shbatt_limit_freq > level)
	{
		shbatt_limit_freq = level;
		sh_cpufreq_update_policy_try();
	}

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}

static int shbatt_api_cpu_clock_limit_unlock( void )
{
	int ret = 0;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	if(shbatt_limit_freq != UINT_MAX)
	{
		shbatt_limit_freq = UINT_MAX;
		sh_cpufreq_update_policy_try();
	}

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return ret;
}
#endif /* SHBATT_ENABLE_LIMIT_LOCK */

#ifdef SHBATT_ENABLE_LIMIT_LOCK
static int shbatt_limit_lock_callback(struct notifier_block *nb, unsigned long event, void *data)
{
	struct cpufreq_policy *policy = (struct cpufreq_policy*)data;
	SHBATT_TRACE("[S]%s event=%lu\n", __FUNCTION__, event);

	SHBATT_INFO("[P]%s shbatt_limit_freq=%u\n", __FUNCTION__, shbatt_limit_freq);

	switch(event)
	{
		case CPUFREQ_INCOMPATIBLE:
			cpufreq_verify_within_limits(policy, 0, shbatt_limit_freq);
			break;
		default:
			/* nothing todo.*/
			break;
	}

	SHBATT_TRACE("[E]%s \n", __FUNCTION__);

	return NOTIFY_OK;
}

#endif /* SHBATT_ENABLE_LIMIT_LOCK */

static int __init shbatt_drv_module_init( void )
{
	int							alm_cnt;
	bool						disable_soc_poll = false;

	SHBATT_TRACE( "[S] %s \n", __FUNCTION__ );

#ifdef SHBATT_ENABLE_LIMIT_LOCK
	if(cpufreq_register_notifier(&shbatt_limit_lock_notifier, CPUFREQ_POLICY_NOTIFIER) != 0)
	{
		SHBATT_ERROR("cpufreq_register_notifier failed.\n");
	}
	else
	{
		SHBATT_TRACE("cpufreq_register_notifier succeeded.\n");
	}
#endif /* SHBATT_ENABLE_LIMIT_LOCK */

	shbatt_task_workqueue_p = create_singlethread_workqueue("shbatt_task");

	mutex_init(&shbatt_task_lock);

	spin_lock_init(&shbatt_pkt_lock);

	wake_lock_init( &shbatt_wake_lock, WAKE_LOCK_SUSPEND, "shbatt_wake" );

	atomic_set( &shbatt_wake_lock_num, 0 );

	for( alm_cnt = 0; alm_cnt < NUM_SHBATT_POLL_TIMER_TYPE; alm_cnt++ )
	{
		switch(shbatt_poll_timer[alm_cnt].timer_type)
		{
		case SHBATT_TIMER_TYPE_HRTIMER:
			memset( &(shbatt_poll_timer[alm_cnt].alm.hr_timer), 0x00, sizeof(struct hrtimer) );
			hrtimer_init( &(shbatt_poll_timer[alm_cnt].alm.hr_timer),
						shbatt_poll_timer[alm_cnt].alarm_type, HRTIMER_MODE_ABS );
			shbatt_poll_timer[alm_cnt].alm.hr_timer.function = shbatt_poll_timer[alm_cnt].cb_func.hrtimer_cb;
			break;

		case SHBATT_TIMER_TYPE_ALARMTIMER:
			memset( &(shbatt_poll_timer[alm_cnt].alm.alarm_timer), 0x00, sizeof(struct alarm) );
			alarm_init( &(shbatt_poll_timer[alm_cnt].alm.alarm_timer),
						shbatt_poll_timer[alm_cnt].alarm_type,
						shbatt_poll_timer[alm_cnt].cb_func.alarm_cb );
			break;

		default:
			SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
					"[E] %s(): alarm type err. default=%d \n",__FUNCTION__, shbatt_poll_timer[alm_cnt].timer_type);
			break;
		}
	}

	platform_driver_register( &shbatt_platform_driver );

	disable_soc_poll = shbatt_api_is_disable_soc_poll();

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[P] %s() : disable_soc_poll:%d\n", __FUNCTION__, disable_soc_poll );

	/* wake_lock */
	memset( &shbatt_lock_time, 0x00, sizeof(shbatt_lock_time) );
	memset( &shbatt_unlock_time, 0x00, sizeof(shbatt_unlock_time) );
	memset( &shbatt_lock_func, 0x00, sizeof(shbatt_lock_func) );

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static void __exit shbatt_drv_module_exit( void )
{
	
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	platform_driver_unregister(&shbatt_platform_driver);

#ifdef SHBATT_ENABLE_LIMIT_LOCK
	if(cpufreq_unregister_notifier(&shbatt_limit_lock_notifier, CPUFREQ_POLICY_NOTIFIER) != 0)
		pr_err("%s: cpufreq_unregister_notifier failed.\n", __func__);
#endif /* SHBATT_ENABLE_LIMIT_LOCK */

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
}

bool is_shbatt_prs_launched( void ) {
	return shbatt_task_is_initialized;
}

module_init(shbatt_drv_module_init);
module_exit(shbatt_drv_module_exit);

MODULE_DESCRIPTION("SH Battery Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/
