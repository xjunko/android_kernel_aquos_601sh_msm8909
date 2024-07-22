/*
 * Copyright (C) 2011 SHARP CORPORATION All rights reserved.
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

#ifndef SHBATT_KERL_H
#define SHBATT_KERL_H

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/
#include <sharp/shterm_k.h>

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

typedef enum shbatt_result_tag
{
	SHBATT_RESULT_SUCCESS,
	SHBATT_RESULT_FAIL,
	SHBATT_RESULT_REJECTED,
	NUM_SHBATT_RESULT

} shbatt_result_t;

typedef enum shbatt_voltage_alarm_type_tag
{
	SHBATT_VOLTAGE_ALARM_TYPE_LOW_BATTERY,
	SHBATT_VOLTAGE_ALARM_TYPE_FATAL_BATTERY,
	NUM_SHBATT_VOLTAGE_ALARM_TYPE
} shbatt_voltage_alarm_type_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/
typedef struct shbatt_bat_calibration_data_tag
{
	int						min;
	int						max;
	int						vmin;
	int						vmax;
} shbatt_bat_calibration_data_t;

typedef struct shbatt_batt_log_info_tag
{
	int						event_num;
	int						bat_vol;
	int						chg_vol;
	int						chg_cur;
	int						bat_temp;
	int						cpu_temp;
	int						chg_temp;
	int						cam_temp;
	int						pmic_temp;
	int						pa_temp;
	int						lcd_temp;
	int						avg_cur;
	int						avg_vol;
	int						latest_cur;
	int						acc_cur;
	int						vol_per;
	int						cur_dep_per;
	int						avg_dep_per;
} shbatt_batt_log_info_t;

typedef struct shbatt_smem_info_tag
{
	unsigned char			traceability_info[22];
} shbatt_smem_info_t;

typedef struct shbatt_boottime_info_tag
{
	int64_t					boot_sec;
	int64_t					boot_nsec;
} shbatt_boottime_info_t;

typedef struct shbatt_log_info_tag
{
	int latest_batt_info;	/* Latest_battery_info */
	int latest_msys_soc;	/* Latest_Msys_SOC     */
	int batt_cap;			/* Battery_capacity    */
	int rslow_drop;			/* Rslow_drop          */
	int latest_soc;			/* Latest_SOC          */
	int latest_cutoff_soc;	/* Latest_Cutoff_SOC   */
	int latest_full_soc;	/* Latest_full_SOC     */
	int vol_shadow;			/* Voltage_shadow      */
	int cur_shadow;			/* Current_shadow      */
	int latest_tmp;			/* Latest_temperature  */
	int latest_sys_sbits;	/* Latest_system_sbits */
} shbatt_log_info_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/
bool shbatt_api_is_disable_usb_charging( void );

bool shbatt_api_is_disable_thermal_control( void );

bool shbatt_api_is_disable_shutdown( void );

bool shbatt_api_is_factory_full_charge( void );

bool shbatt_api_is_disable_soc_poll( void );

shbatt_result_t shbatt_api_get_bat_calibration_data(
	shbatt_bat_calibration_data_t*	cal );

void shbatt_api_notify_low_batt_alarm( void );

void shbatt_api_notify_fatal_batt_alarm( void );

shbatt_result_t shbatt_api_get_battery_log_info(
	shbatt_batt_log_info_t*		bli_p );

int smb1360_get_log_info(shbatt_log_info_t *info);

shbatt_result_t shbatt_api_battlog_event(
	shbattlog_event_num			evt);

shbatt_result_t shbatt_api_battlog_charge_status(
	int			status);

shbatt_result_t shbatt_api_battlog_charge_error(
	int			charge_error_event);

shbatt_result_t shbatt_api_battlog_jeita_status(
	int			jeita_cur_status);

shbatt_result_t shbatt_api_battlog_capacity(
	int			cur_capacity);

shbatt_result_t shbatt_api_battlog_usb_type(
	int			usb_type);

bool is_shbatt_prs_launched( void );

void sh_dump_regs(void);

/*+-----------------------------------------------------------------------------+*/
/*| @ PRIVATE FUNCTION PROTO TYPE DECLARE :                                     |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/

#endif /* SHBATT_KERL_H */
