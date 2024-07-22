/* include/sharp/shdisp_ioctl.h  (Display Driver)
 *
 * Copyright (C) 2015 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

#ifndef SHDISP_IOCTL_H
#define SHDISP_IOCTL_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include "shdisp_context_def.h"

/* ------------------------------------------------------------------------- */
/* IOCTL                                                                     */
/* ------------------------------------------------------------------------- */

#define SHDISP_IOC_MAGIC 's'
#define SHDISP_IOCTL_GET_CONTEXT                        _IOR  (SHDISP_IOC_MAGIC,  0, struct shdisp_to_user_context)
#define SHDISP_IOCTL_SET_HOST_GPIO                      _IOW  (SHDISP_IOC_MAGIC,  1, struct shdisp_host_gpio)
#define SHDISP_IOCTL_TRI_LED_SET_COLOR                  _IOW  (SHDISP_IOC_MAGIC,  2, struct shdisp_tri_led)
#define SHDISP_IOCTL_BDIC_WRITE_REG                     _IOW  (SHDISP_IOC_MAGIC,  3, struct shdisp_diag_bdic_reg)
#define SHDISP_IOCTL_BDIC_READ_REG                      _IOWR (SHDISP_IOC_MAGIC,  4, struct shdisp_diag_bdic_reg)
#define SHDISP_IOCTL_GET_LUX                            _IOWR (SHDISP_IOC_MAGIC,  5, struct shdisp_photo_sensor_val)
#define SHDISP_IOCTL_PHOTO_SENSOR_POW_CTL               _IOW  (SHDISP_IOC_MAGIC,  6, struct shdisp_photo_sensor_power_ctl)
#define SHDISP_IOCTL_LCDDR_WRITE_REG                    _IOW  (SHDISP_IOC_MAGIC,  7, struct shdisp_lcddr_reg)
#define SHDISP_IOCTL_LCDDR_READ_REG                     _IOWR (SHDISP_IOC_MAGIC,  8, struct shdisp_lcddr_reg)
#define SHDISP_IOCTL_SET_FLICKER_PARAM                  _IOW  (SHDISP_IOC_MAGIC,  9, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_GET_FLICKER_PARAM                  _IOWR (SHDISP_IOC_MAGIC, 10, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_BKL_SET_AUTO_MODE                  _IOW  (SHDISP_IOC_MAGIC, 11, struct shdisp_main_bkl_auto)
#define SHDISP_IOCTL_BDIC_MULTI_READ_REG                _IOWR (SHDISP_IOC_MAGIC, 12, struct shdisp_diag_bdic_reg_multi)
#define SHDISP_IOCTL_BKL_SET_DTV_MODE                   _IOW  (SHDISP_IOC_MAGIC, 13, int)
#define SHDISP_IOCTL_BKL_SET_EMG_MODE                   _IOW  (SHDISP_IOC_MAGIC, 14, int)
#define SHDISP_IOCTL_LEDC_POWER_ON                      _IO   (SHDISP_IOC_MAGIC, 15)
#define SHDISP_IOCTL_LEDC_POWER_OFF                     _IO   (SHDISP_IOC_MAGIC, 16)
#define SHDISP_IOCTL_LEDC_SET_RGB                       _IOW  (SHDISP_IOC_MAGIC, 17, struct shdisp_ledc_rgb)
#define SHDISP_IOCTL_LEDC_SET_COLOR                     _IOW  (SHDISP_IOC_MAGIC, 18, struct shdisp_ledc_req)
#define SHDISP_IOCTL_LEDC_WRITE_REG                     _IOW  (SHDISP_IOC_MAGIC, 19, struct shdisp_diag_ledc_reg)
#define SHDISP_IOCTL_LEDC_READ_REG                      _IOWR (SHDISP_IOC_MAGIC, 20, struct shdisp_diag_ledc_reg)
#define SHDISP_IOCTL_LUX_CHANGE_IND                     _IOWR (SHDISP_IOC_MAGIC, 21, struct shdisp_photo_sensor_val)
#define SHDISP_IOCTL_SET_CABC                           _IOWR (SHDISP_IOC_MAGIC, 23, struct shdisp_main_dbc)
#define SHDISP_IOCTL_BKL_SET_CHG_MODE                   _IOW  (SHDISP_IOC_MAGIC, 24, int)
#define SHDISP_IOCTL_GET_FLICKER_LOW_PARAM              _IOWR (SHDISP_IOC_MAGIC, 25, struct shdisp_diag_flicker_param)
#define SHDISP_IOCTL_LCDC_SET_DRIVE_FREQ                _IOW  (SHDISP_IOC_MAGIC, 26, struct shdisp_main_drive_freq)
#define SHDISP_IOCTL_SET_GAMMA_INFO                     _IOW  (SHDISP_IOC_MAGIC, 27, struct shdisp_diag_gamma_info)
#define SHDISP_IOCTL_GET_GAMMA_INFO                     _IOWR (SHDISP_IOC_MAGIC, 28, struct shdisp_diag_gamma_info)
#define SHDISP_IOCTL_SET_GAMMA                          _IOW  (SHDISP_IOC_MAGIC, 29, struct shdisp_diag_gamma)
#define SHDISP_IOCTL_GET_AVE_ADO                        _IOWR (SHDISP_IOC_MAGIC, 30, struct shdisp_ave_ado)
#define SHDISP_IOCTL_PSALS_READ_REG                     _IOWR (SHDISP_IOC_MAGIC, 31, struct shdisp_diag_psals_reg)
#define SHDISP_IOCTL_PSALS_WRITE_REG                    _IOWR (SHDISP_IOC_MAGIC, 32, struct shdisp_diag_psals_reg)
#define SHDISP_IOCTL_GET_ALS                            _IOWR (SHDISP_IOC_MAGIC, 33, struct shdisp_photo_sensor_raw_val)
#ifdef CONFIG_SHDISP_PANEL_SUBDISPLAY
#define SHDISP_IOCTL_SUB_DISP_ON                        _IO   (SHDISP_IOC_MAGIC, 34)
#define SHDISP_IOCTL_SUB_DISP_OFF                       _IO   (SHDISP_IOC_MAGIC, 35)
#define SHDISP_IOCTL_SUB_DISP_UPDATE                    _IOWR (SHDISP_IOC_MAGIC, 36, struct shdisp_sub_update)
#endif
#define SHDISP_IOCTL_SET_IRQ_MASK                       _IOWR (SHDISP_IOC_MAGIC, 37, int)
#endif /* SHDISP_IOCTL_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
