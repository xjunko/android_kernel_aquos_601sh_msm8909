/* drivers/sharp/shdisp/shdisp_bd6118gu.c  (Display Driver)
 *
 * Copyright (C) 2014 SHARP CORPORATION
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

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/consumer.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <sharp/shdisp_kerl.h>
#include "shdisp_system.h"
#include "shdisp_type.h"
#include "shdisp_bdic.h"
#include "shdisp_dbg.h"

#include "./data/shdisp_bd6118gu_data.h"

#include "shdisp_pm.h"

#include "data/shdisp_bd6118gu_ctrl.h"


/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_BDIC_REGSET(x)               (shdisp_bdic_API_seq_regset(x, ARRAY_SIZE(x)))
#ifdef SHDISP_NOT_SUPPORT_NO_OS
 #define SHDISP_BDIC_HW_INIT
#endif /* SHDISP_NOT_SUPPORT_NO_OS */

#define SHDISP_BDIC_BKL_MODE_OFF                (0)
#define SHDISP_BDIC_BKL_MODE_FIX                (1)
#define SHDISP_BDIC_BKL_MODE_AUTO               (2)
#define SHDISP_BDIC_TRI_LED_MODE_OFF           (-1)
#define SHDISP_BDIC_TRI_LED_MODE_NORMAL         (0)
#define SHDISP_BDIC_TRI_LED_MODE_BLINK          (1)
#define SHDISP_BDIC_TRI_LED_MODE_FIREFLY        (2)
#define SHDISP_BDIC_TRI_LED_MODE_HISPEED        (3)
#define SHDISP_BDIC_TRI_LED_MODE_STANDARD       (4)
#define SHDISP_BDIC_TRI_LED_MODE_BREATH         (5)
#define SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH    (6)
#define SHDISP_BDIC_TRI_LED_MODE_WAVE           (7)
#define SHDISP_BDIC_TRI_LED_MODE_FLASH          (8)
#define SHDISP_BDIC_TRI_LED_MODE_AURORA         (9)
#define SHDISP_BDIC_TRI_LED_MODE_RAINBOW       (10)

#define SHDISP_BDIC_BKL_EMG_OFF                 (0)
#define SHDISP_BDIC_BKL_EMG_ON                  (1)

static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status);
static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status);
static void shdisp_bdic_PD_BKL_set_led_value(void);
static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param);
static void shdisp_bdic_seq_backlight_off(void);
static void shdisp_bdic_seq_backlight_fix_on(int param);
static int  shdisp_bdic_seq_led_off(void);
static int  shdisp_bdic_seq_led_normal_on(unsigned char color);
static void shdisp_bdic_seq_led_blink_on(unsigned char color, int ontime, int interval, int count);
static void shdisp_bdic_LD_set_led_fix_on_table(int clr_vari, int color);
static void shdisp_bdic_PD_BKL_control(unsigned char request, int param);
static void shdisp_bdic_seq_bdic_active_for_led(int);
static void shdisp_bdic_seq_bdic_standby_for_led(int);
static void shdisp_bdic_LD_LCD_BKL_emg_on(void);
static void shdisp_bdic_LD_LCD_BKL_emg_off(void);
static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode);

static void shdisp_bdic_LD_LCD_BKL_get_fix_param(int mode, int level, unsigned char *value);



/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static struct shdisp_bdic_state_str s_state_str;
static int shdisp_bdic_bkl_mode;
static int shdisp_bdic_tri_led_before_mode;
static int shdisp_bdic_bkl_param;
static int shdisp_bdic_bkl_before_mode;
static unsigned char shdisp_bdic_tri_led_color;
static int shdisp_bdic_tri_led_mode;
static int shdisp_bdic_tri_led_ontime;
static int shdisp_bdic_tri_led_interval;
static int shdisp_bdic_tri_led_count;
static struct shdisp_main_bkl_ctl shdisp_bkl_priority_table[NUM_SHDISP_MAIN_BKL_DEV_TYPE] = {
    { SHDISP_MAIN_BKL_MODE_OFF      , SHDISP_MAIN_BKL_PARAM_OFF },
    { SHDISP_MAIN_BKL_MODE_AUTO     , SHDISP_MAIN_BKL_PARAM_OFF }
};
static int shdisp_bdic_irq_fac = 0;

static int shdisp_bdic_emg;
extern int bdic_irq_gpio;



/* ------------------------------------------------------------------------- */
/* UNSUPPORT API                                                             */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_get_bdic_chipver(int* chipver) {}
void shdisp_bdic_API_LCD_power_on(void) {}
void shdisp_bdic_API_LCD_power_off(void) {}
void shdisp_bdic_API_LCD_m_power_on(void) {}
void shdisp_bdic_API_LCD_m_power_off(void) {}
void shdisp_bdic_API_LCD_BKL_auto_on(int param) {}
void shdisp_bdic_API_LCD_BKL_dtv_on(void) {}
void shdisp_bdic_API_LCD_BKL_dtv_off(void) {}
void shdisp_bdic_API_LCD_BKL_eco_on(void) {}
void shdisp_bdic_API_LCD_BKL_eco_off(void) {}
void shdisp_bdic_API_LCD_BKL_chg_on(void) {}
void shdisp_bdic_API_LCD_BKL_chg_off(void) {}

void shdisp_bdic_API_TRI_LED_firefly_on(unsigned char color, int ontime, int interval, int count) {}
int shdisp_bdic_API_i2c_transfer(struct shdisp_bdic_i2c_msg *msg) { return 1; }
unsigned char shdisp_bdic_API_I2C_start_judge(void) { return 1; }
void shdisp_bdic_API_I2C_start_ctl(int flg) {}

int  shdisp_bdic_API_RECOVERY_check_bdic_practical(void) { return 1; }
#if defined (CONFIG_ANDROID_ENGINEERING)
void shdisp_bdic_API_OPT_INFO_output(void) {}
#ifndef SHDISP_NOT_SUPPORT_PSALS
void shdisp_psals_API_DBG_INFO_output(void) {}
#endif
#endif /* CONFIG_ANDROID_ENGINEERING */

void shdisp_bdic_API_IRQ_Clear(void) {}
void shdisp_bdic_API_IRQ_i2c_error_Clear(void) {}
void shdisp_bdic_API_IRQ_det_fac_Clear(void) {}
int  shdisp_bdic_API_IRQ_check_I2C_ERR(void) { return SHDISP_BDIC_IRQ_TYPE_NONE; }
void shdisp_bdic_API_IRQ_dbg_Clear_All(void) {}
void shdisp_bdic_API_IRQ_dbg_set_fac(unsigned int nGFAC) {shdisp_bdic_irq_fac = nGFAC;}
void shdisp_bdic_API_set_device_code(void) {}
#ifndef SHDISP_NOT_SUPPORT_PSALS
int  shdisp_bdic_API_PHOTO_SENSOR_get_lux(unsigned short *value, unsigned long *lux) { return 1; }
int  shdisp_bdic_API_PHOTO_SENSOR_get_raw_als(unsigned short *clear, unsigned short *ir) { return 1; }
int  shdisp_bdic_API_PHOTO_SENSOR_lux_change_ind(int *mode) { return 1; }
void shdisp_bdic_API_IRQ_dbg_photo_param( int level1, int level2) {}
int  shdisp_bdic_API_als_sensor_pow_ctl(int dev_type, int power_mode) { return 1; }
int  shdisp_bdic_API_psals_power_on(void) { return 1; }
int  shdisp_bdic_API_psals_power_off(void) { return 1; }
int  shdisp_bdic_API_psals_ps_init_als_off(void) { return 1; }
int  shdisp_bdic_API_psals_ps_init_als_on(void) { return 1; }
int  shdisp_bdic_API_psals_ps_deinit_als_off(void) { return 1; }
int  shdisp_bdic_API_psals_ps_deinit_als_on(void) { return 1; }
int  shdisp_bdic_API_psals_als_init_ps_off(void) { return 1; }
int  shdisp_bdic_API_psals_als_init_ps_on(void) { return 1; }
int  shdisp_bdic_API_psals_als_deinit_ps_off(void) { return 1; }
int  shdisp_bdic_API_psals_als_deinit_ps_on(void) { return 1; }
int  shdisp_bdic_API_psals_is_recovery_successful(void) { return 1; }
void shdisp_bdic_API_set_prox_sensor_param( struct shdisp_prox_params *prox_params) {}
int  shdisp_bdic_API_get_lux_data(void) { return 1; }
void shdisp_bdic_API_set_bkl_mode(unsigned char bkl_mode, unsigned char data, unsigned char msk) {}
void shdisp_bdic_API_set_lux_mode(unsigned char lux_mode, unsigned char data, unsigned char msk) {}
void shdisp_bdic_API_set_lux_mode_modify(unsigned char data, unsigned char msk) {}
int  shdisp_bdic_API_get_sensor_state(void) { return 1; }
void shdisp_bdic_API_RECOVERY_lux_data_backup(void) {}
void shdisp_bdic_API_RECOVERY_lux_data_restore(void) {}
void shdisp_bdic_API_psals_active(unsigned long dev_type) {}
void shdisp_bdic_API_psals_standby(unsigned long dev_type) {}
void shdisp_bdic_API_ps_background(unsigned long state) {}
void shdisp_bdic_API_check_sensor_param(struct shdisp_photo_sensor_adj *adj_in, struct shdisp_photo_sensor_adj *adj_out) {}
void shdisp_bdic_API_als_sensor_adjust(struct shdisp_photo_sensor_adj *adj) {}
int  shdisp_bdic_API_get_ave_ado(struct shdisp_ave_ado *ave_ado) { return 1; }
#endif
/* ------------------------------------------------------------------------- */
/* SUPPORT API                                                               */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_initialize                                                */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_initialize(struct shdisp_bdic_state_str* state_str)
{
    SHDISP_TRACE("in\n");
    shdisp_bdic_bkl_mode        = SHDISP_BDIC_BKL_MODE_OFF;
    shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
    shdisp_bdic_bkl_param       = SHDISP_MAIN_BKL_PARAM_OFF;
    shdisp_bdic_emg             = SHDISP_BDIC_BKL_EMG_OFF;

    s_state_str.bdic_is_exist = state_str->bdic_is_exist;
    s_state_str.bdic_clrvari_index = state_str->bdic_clrvari_index;
#ifdef SHDISP_BDIC_HW_INIT
    if (s_state_str.bdic_is_exist == SHDISP_BDIC_IS_EXIST) {
            (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_ON);
            (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_INIT, SHDISP_DEV_STATE_OFF);
    }
#endif /* SHDISP_BDIC_HW_INIT */
    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_off                                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_off(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_seq_backlight_off();
    SHDISP_TRACE("out\n")
    return;
}



/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_fix_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_fix_on(int param)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_seq_backlight_fix_on(param);
    SHDISP_TRACE("out\n")
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_set_request                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_set_request(int type, struct shdisp_main_bkl_ctl *tmp)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;

    shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
    shdisp_bdic_bkl_mode   = tmp->mode;
    shdisp_bdic_bkl_param  = tmp->param;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_set_request                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_set_request(struct shdisp_tri_led *tmp)
{
    int color = 0x00;

    color = (tmp->blue << 2) | (tmp->green << 1) | tmp->red;

    shdisp_bdic_tri_led_mode        = tmp->led_mode;
    shdisp_bdic_tri_led_before_mode = tmp->led_mode;
    shdisp_bdic_tri_led_color       = color;
    shdisp_bdic_tri_led_ontime      = tmp->ontime;
    shdisp_bdic_tri_led_interval    = tmp->interval;
    shdisp_bdic_tri_led_count       = tmp->count;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_request                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_get_request(int type, struct shdisp_main_bkl_ctl *tmp, struct shdisp_main_bkl_ctl *req)
{
    shdisp_bkl_priority_table[type].mode  = tmp->mode;
    shdisp_bkl_priority_table[type].param = tmp->param;


    SHDISP_TRACE("tmp->mode %d, tmp->param %d \n", tmp->mode, tmp->param)

    if (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_OFF) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    } else if ((shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode == SHDISP_MAIN_BKL_MODE_FIX) &&
               (shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param == SHDISP_MAIN_BKL_PARAM_WEAK)) {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    } else {
        req->mode  = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].mode;
        req->param = shdisp_bkl_priority_table[SHDISP_MAIN_BKL_DEV_TYPE_APP].param;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_release_hw_reset                                      */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_release_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_HIGH);
    shdisp_SYS_API_delay_us(10);
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_LOW);
    shdisp_SYS_API_delay_us(10);
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_HIGH);
    shdisp_SYS_API_delay_us(20000);

    return;

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_set_hw_reset                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_set_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_LOW);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_release_deep_standby                                  */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_release_deep_standby(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_LOW);
    shdisp_SYS_API_delay_us(30000);
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_COG_RESET, SHDISP_BDIC_GPIO_HIGH);
    shdisp_SYS_API_delay_us(60000);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_subdisplay_release_hw_reset                               */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_subdisplay_release_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_SUBDISPLAY_RESET, SHDISP_BDIC_GPIO_HIGH);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_subdisplay_set_hw_reset                                   */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_subdisplay_set_hw_reset(void)
{
    shdisp_bdic_LD_GPIO_control(SHDISP_BDIC_GPIO_SUBDISPLAY_RESET, SHDISP_BDIC_GPIO_LOW);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_write_reg                                            */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_write_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    ret = shdisp_bdic_API_IO_write_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_read_reg                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = 0;

    ret = shdisp_bdic_API_IO_read_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DIAG_multi_read_reg                                       */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_DIAG_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret = 0;

    ret = shdisp_bdic_API_IO_multi_read_reg(reg, val, size);
    return ret;
}

#if defined (CONFIG_ANDROID_ENGINEERING)
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_DBG_INFO_output                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_DBG_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;
    unsigned char   shdisp_log_lv_bk;

    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_STATE_ON);
    pbuf = kzalloc(9, GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d\n", 9);
        return;
    }

    shdisp_log_lv_bk = shdisp_log_lv;
    shdisp_log_lv = SHDISP_LOG_LV_ERR;

    p = pbuf;
    for (idx = 0x00; idx <= 0x08; idx++ ) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }
    shdisp_log_lv = shdisp_log_lv_bk;

    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_DEBUG, SHDISP_DEV_STATE_OFF);

    printk("[SHDISP] BDIC INFO ->>\n");
    printk("[SHDISP] s_state_str.bdic_is_exist              = %d.\n", s_state_str.bdic_is_exist);
    printk("[SHDISP] s_state_str.bdic_chipver               = %d.\n", s_state_str.bdic_chipver);
    printk("[SHDISP] s_state_str.bdic_main_bkl_opt_mode_output = %d. (0:low/1:high)\n",
                                                                      s_state_str.bdic_main_bkl_opt_mode_output);
    printk("[SHDISP] s_state_str.bdic_main_bkl_opt_mode_ado = %d. (0:low/1:high)\n",
                                                                      s_state_str.bdic_main_bkl_opt_mode_ado);
#ifndef SHDISP_NOT_SUPPORT_PSALS
    printk("[SHDISP] s_state_str.shdisp_lux_change_level1   = %d.\n", s_state_str.shdisp_lux_change_level1);
    printk("[SHDISP] s_state_str.shdisp_lux_change_level2   = %d.\n", s_state_str.shdisp_lux_change_level2);
#endif
    printk("[SHDISP] shdisp_bdic_bkl_mode                   = %d.\n", shdisp_bdic_bkl_mode);
    printk("[SHDISP] shdisp_bdic_bkl_before_mode            = %d.\n", shdisp_bdic_bkl_before_mode);
    printk("[SHDISP] shdisp_bdic_bkl_param                  = %d.\n", shdisp_bdic_bkl_param);
    printk("[SHDISP] shdisp_bdic_tri_led_color              = %d.\n", (int)shdisp_bdic_tri_led_color);
    printk("[SHDISP] shdisp_bdic_tri_led_mode               = %d.\n", shdisp_bdic_tri_led_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_before_mode        = %d.\n", shdisp_bdic_tri_led_before_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_ontime             = %d.\n", shdisp_bdic_tri_led_ontime);
    printk("[SHDISP] shdisp_bdic_tri_led_interval           = %d.\n", shdisp_bdic_tri_led_interval);
    printk("[SHDISP] shdisp_bdic_tri_led_count              = %d.\n", shdisp_bdic_tri_led_count);
    printk("[SHDISP] bdic_clrvari_index                     = %d.\n", s_state_str.bdic_clrvari_index );


    p = pbuf;
    printk("[SHDISP] BDIC_REG 0x%02X: %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                                idx, *p, *(p+1), *(p+2), *(p+3), *(p+4), *(p+5), *(p+6), *(p+7),*(p+8));
    printk("[SHDISP] BDIC INFO <<-\n");
    kfree(pbuf);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_INFO_output                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_INFO_output(void)
{
    int idx;
    unsigned char   *p;
    unsigned char   *pbuf;

    pbuf = (unsigned char*)kzalloc((BDIC_REG_LEDCNTG - BDIC_REG_GPOUT + 1), GFP_KERNEL);
    if (!pbuf) {
        SHDISP_ERR("kzalloc failed. size=%d\n",(BDIC_REG_LEDCNTG - BDIC_REG_GPOUT + 1));
        return;
    }
    p = pbuf;
    for (idx = BDIC_REG_GPOUT; idx <= BDIC_REG_LEDCNTG; idx++ ) {
        *p = 0x00;
        shdisp_bdic_API_DIAG_read_reg(idx, p);
        p++;
    }

    printk("[SHDISP] TRI-LED INFO ->>\n");
    printk("[SHDISP] s_state_str.bdic_is_exist      = %d.\n", s_state_str.bdic_is_exist);
    printk("[SHDISP] shdisp_bdic_tri_led_color      = %d.\n", (int)shdisp_bdic_tri_led_color);
    printk("[SHDISP] shdisp_bdic_tri_led_mode       = %d.\n", shdisp_bdic_tri_led_mode);
    printk("[SHDISP] shdisp_bdic_tri_led_ontime     = %d.\n", shdisp_bdic_tri_led_ontime);
    printk("[SHDISP] shdisp_bdic_tri_led_interval   = %d.\n", shdisp_bdic_tri_led_interval);
    printk("[SHDISP] shdisp_bdic_tri_led_count      = %d.\n", shdisp_bdic_tri_led_count);
    printk("[SHDISP] shdisp_bdic_clrvari_index      = %d.\n", s_state_str.bdic_clrvari_index);

    p = pbuf;
    printk("[SHDISP] BDIC_REG_GPOUT 0x%02X: %02x\n", BDIC_REG_GPOUT, *p);
    p += 1;
    printk("[SHDISP] BDIC_REG_LEDDRVCNT   0x%02X: %02x\n", BDIC_REG_LEDDRVCNT, *p);
    p += 2;
    printk("[SHDISP] BDIC_REG_LED_CURRENT   0x%02X: %02x %02x %02x\n",
                            BDIC_REG_LEDCNTB, *p, *(p+1), *(p+2));

    kfree(pbuf);

    printk("[SHDISP] TRI-LED INFO <<-\n");
    return;
}
#endif /* CONFIG_ANDROID_ENGINEERING */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_det_irq_ctrl                                          */
/* ------------------------------------------------------------------------- */
void shdisp_bdic_API_IRQ_det_irq_ctrl(int ctrl)
{
    if (ctrl) {
        shdisp_bdic_API_IO_set_bit_reg(BDIC_REG_GPOUT, 0x80);
    } else {
        shdisp_bdic_API_IO_clr_bit_reg(BDIC_REG_GPOUT, 0x80);
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_RECOVERY_check_restoration                                */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_RECOVERY_check_restoration(void)
{
    int value = 0;

    value = gpio_get_value_cansleep(bdic_irq_gpio);

    return value;
}

void shdisp_bdic_API_IRQ_save_fac(void)
{
    shdisp_bdic_irq_fac = shdisp_bdic_API_RECOVERY_check_restoration();

    SHDISP_DEBUG("shdisp_bdic_irq_fac[%d]\n", shdisp_bdic_irq_fac);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_DET                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_DET(void)
{
    if (shdisp_bdic_irq_fac) {
        return SHDISP_BDIC_IRQ_TYPE_DET;
    } else {
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_fac                                             */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_fac(void)
{
    if (shdisp_bdic_irq_fac == 0) {
        return SHDISP_RESULT_FAILURE;
    } else {
        return SHDISP_RESULT_SUCCESS;
    }

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_get_fac                                               */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_get_fac(int iQueFac)
{
    if ((shdisp_bdic_irq_fac != 0) && iQueFac == 0) {
        return SHDISP_BDIC_IRQ_TYPE_DET;
    } else {
        return SHDISP_BDIC_IRQ_TYPE_NONE;
    }
}
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_off                                               */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_TRI_LED_off(void)
{
    int ret;
    ret = shdisp_bdic_seq_led_off();
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_get_color_index_and_reedit                        */
/* ------------------------------------------------------------------------- */

unsigned char shdisp_bdic_API_TRI_LED_get_color_index_and_reedit(struct shdisp_tri_led *tri_led )
{
    int i;
    unsigned char color = 0xFF;

    for (i = 0; i < ARRAY_SIZE(shdisp_triple_led_color_index_tbl); i++) {
        if (shdisp_triple_led_color_index_tbl[i].red   == tri_led->red     &&
            shdisp_triple_led_color_index_tbl[i].green == tri_led->green   &&
            shdisp_triple_led_color_index_tbl[i].blue  == tri_led->blue){
            color = shdisp_triple_led_color_index_tbl[i].color;
            break;
        }
    }

    if (color == 0xFF) {
        if (tri_led->red > 1) {
            tri_led->red = 1;
        }
        if (tri_led->green > 1) {
            tri_led->green = 1;
        }
        if (tri_led->blue > 1) {
            tri_led->blue = 1;
        }
        for (i = 0; i < ARRAY_SIZE(shdisp_triple_led_color_index_tbl); i++) {
            if (shdisp_triple_led_color_index_tbl[i].red   == tri_led->red     &&
                shdisp_triple_led_color_index_tbl[i].green == tri_led->green   &&
                shdisp_triple_led_color_index_tbl[i].blue  == tri_led->blue){
                color = shdisp_triple_led_color_index_tbl[i].color;
                break;
            }
        }
        if (color == 0xFF) {
            color = 0;
        }
    }
    return color;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_normal_on                                         */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_TRI_LED_normal_on(unsigned char color)
{
    int ret;
    ret = shdisp_bdic_seq_led_normal_on(color);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_blink_on                                          */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_TRI_LED_blink_on(unsigned char color, int ontime, int interval, int count)
{
    shdisp_bdic_seq_led_blink_on(color, ontime, interval, count);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_TRI_LED_get_clrvari_index                                 */
/* ------------------------------------------------------------------------- */

int  shdisp_bdic_API_TRI_LED_get_clrvari_index( int clrvari )
{
    int i = 0;

    for (i = 0; i < SHDISP_COL_VARI_KIND; i++) {
        if ((int)shdisp_clrvari_index[i] == clrvari) {
            break;
        }
    }
    if (i >= SHDISP_COL_VARI_KIND) {
        i = 0;
    }
    return i;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_subdisplay_power_on                                       */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_subdisplay_power_on(void)
{
    SHDISP_BDIC_REGSET(shdisp_bdic_subdisplay_power_on);
	return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_subdisplay_power_off                                      */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_subdisplay_power_off(void)
{
    SHDISP_BDIC_REGSET(shdisp_bdic_subdisplay_power_off);
	return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_off                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_backlight_off(void)
{

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_OFF, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_STOP, 0);

    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_STATE_OFF);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_backlight_fix_on                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_backlight_fix_on(int param)
{
    SHDISP_TRACE("in param:%d\n", param);

    (void)shdisp_pm_bdic_power_manager(SHDISP_DEV_TYPE_BKL, SHDISP_DEV_STATE_ON);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0020 START\n");

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_MODE_FIX, param);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_SET_LED_VALUE, 0);
    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_ON, 0);

    SHDISP_PERFORMANCE("RESUME BDIC TURN-ON BACKLIGHT 0020 END\n");


    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_off                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_led_off(void)
{
    SHDISP_TRACE("in\n");

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_STOP, 0);
    shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_normal_on                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_seq_led_normal_on(unsigned char color)
{
    SHDISP_TRACE("in color:%d\n", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL, color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);

    SHDISP_TRACE("out\n");
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_led_blink_on                                              */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_led_blink_on(unsigned char color, int ontime, int interval, int count)
{
    SHDISP_TRACE("in color:%d\n", color);

    shdisp_bdic_seq_bdic_active_for_led(SHDISP_DEV_TYPE_LED);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK,   color);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME,       ontime);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL,     interval);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_TRI_LED_SET_COUNT,        count);
    shdisp_bdic_PD_TRI_LED_control(SHDISP_BDIC_REQ_START, 0);

    if (s_state_str.bdic_chipver >= SHDISP_BDIC_CHIPVER_1) {
        shdisp_bdic_seq_bdic_standby_for_led(SHDISP_DEV_TYPE_LED);
    }
    SHDISP_TRACE("out\n");
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_set_led_fix_on_table                                       */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_set_led_fix_on_table(int clr_vari, int color)
{
    unsigned char *pTriLed;

    SHDISP_TRACE("in\n");
    pTriLed = (unsigned char*)(&(shdisp_triple_led_tbl[clr_vari][color]));

    shdisp_bdic_led_fix_on[0].data = *(pTriLed + 0);
    shdisp_bdic_led_fix_on[1].data = *(pTriLed + 1);
    shdisp_bdic_led_fix_on[2].data = *(pTriLed + 2);
    shdisp_bdic_led_enable[0].data = *(pTriLed + 3);
    SHDISP_TRACE("out\n");
}



/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_fix_param                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_fix_param(int mode, int level, unsigned char *param)
{
    unsigned char value;

    if (param == NULL) {
        return;
    }

    value = shdisp_main_bkl_tbl[level];

    switch (mode) {
    case SHDISP_BKL_TBL_MODE_NORMAL:
    case SHDISP_BKL_TBL_MODE_ECO:
        break;
    case SHDISP_BKL_TBL_MODE_EMERGENCY:
        if (value > SHDISP_BKL_EMERGENCY_LIMIT_FIX) {
            value = SHDISP_BKL_EMERGENCY_LIMIT_FIX;
        }
        break;
    case SHDISP_BKL_TBL_MODE_CHARGE:
    default:
        break;
    }

    *param = value;
    return;
}
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_get_param                                         */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_get_param(unsigned long int* param)
{
    int mode = 0;
    unsigned char value;

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_FIX:
        shdisp_bdic_LD_LCD_BKL_get_mode(&mode);
        shdisp_bdic_LD_LCD_BKL_get_fix_param(mode, shdisp_bdic_bkl_param, &value);
        *param = value;
        break;

    case SHDISP_BDIC_BKL_MODE_AUTO:
        *param = 0x100;
        break;

    default:
        *param = 0;
        break;
    }
    return;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_on                                            */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_emg_on(void)
{
    shdisp_bdic_LD_LCD_BKL_emg_on();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_LCD_BKL_emg_off                                           */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_LCD_BKL_emg_off(void)
{
    shdisp_bdic_LD_LCD_BKL_emg_off();

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_control                                                */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_BKL_control(unsigned char request, int param)
{

    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_STANDBY:
        break;

    case SHDISP_BDIC_REQ_BKL_ON:
        if (shdisp_bdic_bkl_before_mode == SHDISP_BDIC_BKL_MODE_OFF) {
            SHDISP_BDIC_REGSET(shdisp_bdic_bkl_on);
        }
        break;

    case SHDISP_BDIC_REQ_BKL_SET_LED_VALUE:
        if (shdisp_bdic_bkl_mode  == SHDISP_BDIC_BKL_MODE_FIX) {
            shdisp_bdic_PD_BKL_set_led_value();
        }
        break;

    case SHDISP_BDIC_REQ_START:
        if (shdisp_bdic_bkl_mode  == SHDISP_BDIC_BKL_MODE_FIX) {
           shdisp_bdic_PD_BKL_set_led_value();
        }
        break;

    case SHDISP_BDIC_REQ_STOP:
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = SHDISP_MAIN_BKL_PARAM_OFF;
        SHDISP_BDIC_REGSET(shdisp_bdic_bkl_off);
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_OFF;
        shdisp_bdic_bkl_param = param;
        break;

    case SHDISP_BDIC_REQ_BKL_SET_MODE_FIX:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_bkl_mode  = SHDISP_BDIC_BKL_MODE_FIX;
        shdisp_bdic_bkl_param = param;
        break;
    case SHDISP_BDIC_REQ_BKL_EMG_OFF:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_emg = SHDISP_BDIC_BKL_EMG_OFF;
        break;

    case SHDISP_BDIC_REQ_BKL_EMG_ON:
        shdisp_bdic_bkl_before_mode = shdisp_bdic_bkl_mode;
        shdisp_bdic_emg = SHDISP_BDIC_BKL_EMG_ON;
        break;

    default:
        break;
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_BKL_set_led_value                                          */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_BKL_set_led_value()
{
    int  mode = 0;
    if (shdisp_bdic_bkl_mode == SHDISP_BDIC_BKL_MODE_OFF) {
        return;
    }

    shdisp_bdic_LD_LCD_BKL_get_mode(&mode);

    switch (shdisp_bdic_bkl_mode) {
    case SHDISP_BDIC_BKL_MODE_FIX:
        shdisp_bdic_LD_LCD_BKL_get_fix_param(mode, shdisp_bdic_bkl_param, &shdisp_bdic_bkl_led_value[0].data );
        break;
    case SHDISP_BDIC_BKL_MODE_AUTO:
        break;
    default:

        return;
    }

    SHDISP_BDIC_REGSET(shdisp_bdic_bkl_led_value);
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_TRI_LED_control                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_TRI_LED_control(unsigned char request, int param)
{
    switch (request) {
    case SHDISP_BDIC_REQ_ACTIVE:
        break;

    case SHDISP_BDIC_REQ_STANDBY:
        break;

    case SHDISP_BDIC_REQ_START:
        SHDISP_DEBUG("SHDISP_BDIC_REQ_START.tri_led_mode=%d.led_before_mode=%d.\n"
                       ,shdisp_bdic_tri_led_mode, shdisp_bdic_tri_led_before_mode);
        SHDISP_DEBUG("SHDISP_BDIC_REQ_START.s_state_str.bdic_clrvari_index=%d\n"
                       ,s_state_str.bdic_clrvari_index);
        switch (shdisp_bdic_tri_led_before_mode) {
        case SHDISP_BDIC_TRI_LED_MODE_OFF:
            break;
        case SHDISP_BDIC_TRI_LED_MODE_NORMAL:
            if (shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
                break;
            }
        /* FALLTHROUGH */
        case SHDISP_BDIC_TRI_LED_MODE_BLINK:
        case SHDISP_BDIC_TRI_LED_MODE_FIREFLY:
        case SHDISP_BDIC_TRI_LED_MODE_HISPEED:
        case SHDISP_BDIC_TRI_LED_MODE_STANDARD:
        case SHDISP_BDIC_TRI_LED_MODE_BREATH:
        case SHDISP_BDIC_TRI_LED_MODE_LONG_BREATH:
        case SHDISP_BDIC_TRI_LED_MODE_WAVE:
        case SHDISP_BDIC_TRI_LED_MODE_FLASH:
        case SHDISP_BDIC_TRI_LED_MODE_AURORA:
        case SHDISP_BDIC_TRI_LED_MODE_RAINBOW:
        default:
            SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
            break;
        }

        if (shdisp_bdic_tri_led_mode == SHDISP_BDIC_TRI_LED_MODE_NORMAL) {
            shdisp_bdic_LD_set_led_fix_on_table(s_state_str.bdic_clrvari_index, shdisp_bdic_tri_led_color);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_fix_on);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_disable);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_ani_off);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_enable);
        } else {
            shdisp_bdic_LD_set_led_fix_on_table(s_state_str.bdic_clrvari_index, shdisp_bdic_tri_led_color);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_fix_on);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_lposc_enable);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_enable);
            SHDISP_BDIC_REGSET(shdisp_bdic_led_ani_on);


        }
        shdisp_bdic_tri_led_before_mode = shdisp_bdic_tri_led_mode;
        break;

    case SHDISP_BDIC_REQ_STOP:
        SHDISP_BDIC_REGSET(shdisp_bdic_led_off);
        shdisp_bdic_tri_led_mode        = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_before_mode = SHDISP_BDIC_TRI_LED_MODE_OFF;
        shdisp_bdic_tri_led_color       = 0;
        shdisp_bdic_tri_led_ontime      = 0;
        shdisp_bdic_tri_led_interval    = 0;
        shdisp_bdic_tri_led_count       = 0;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_NORMAL:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_NORMAL;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_BLINK:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_BLINK;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_MODE_FIREFLY:
        shdisp_bdic_tri_led_mode  = SHDISP_BDIC_TRI_LED_MODE_FIREFLY;
        shdisp_bdic_tri_led_color = (unsigned char)param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_ONTIME:
        shdisp_bdic_tri_led_ontime = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_INTERVAL:
        shdisp_bdic_tri_led_interval = param;
        break;

    case SHDISP_BDIC_REQ_TRI_LED_SET_COUNT:
        shdisp_bdic_tri_led_count = param;
        break;

    default:
        break;
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_GPIO_control(unsigned char symbol, unsigned char status)
{
    unsigned char port;

    switch (symbol) {
    case SHDISP_BDIC_GPIO_COG_RESET:
        port = SHDISP_BDIC_GPIO_GPOD0;
        break;

    case SHDISP_BDIC_GPIO_SUBDISPLAY_RESET:
        port = SHDISP_BDIC_GPIO_GPOD2;
        break;
    default:
        return;
    }

    shdisp_bdic_PD_GPIO_control(port, status);

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_GPIO_control                                               */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_GPIO_control(unsigned char port, unsigned char status)
{
    unsigned char    reg;
    unsigned char    bit;

    switch (port) {
    case SHDISP_BDIC_GPIO_GPOD0:
        reg = BDIC_REG_GPOUT;
        bit = 0x01;
        break;
    case SHDISP_BDIC_GPIO_GPOD1:
        reg = BDIC_REG_GPOUT;
        bit = 0x02;
        break;
    case SHDISP_BDIC_GPIO_GPOD2:
        reg = BDIC_REG_GPOUT;
        bit = 0x04;
        break;
    case SHDISP_BDIC_GPIO_GPOD3:
        reg = BDIC_REG_GPOUT;
        bit = 0x08;
        break;
    case SHDISP_BDIC_GPIO_GPOD4:
        reg = BDIC_REG_GPOUT;
        bit = 0x10;
        break;
    default:
        return;
    }
    if (status == SHDISP_BDIC_GPIO_HIGH) {
        shdisp_bdic_API_IO_set_bit_reg(reg, bit);
    } else {
        shdisp_bdic_API_IO_clr_bit_reg(reg, bit);
    }

    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_bdic_active_for_led                                       */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_bdic_active_for_led(int dev_type)
{
    SHDISP_TRACE("in dev_type:%d\n", dev_type);
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_ON);
    SHDISP_TRACE("out\n");
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_seq_bdic_standby_for_led                                      */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_seq_bdic_standby_for_led(int dev_type)
{
    (void)shdisp_pm_bdic_power_manager(dev_type, SHDISP_DEV_STATE_OFF);
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IRQ_check_type                                            */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_IRQ_check_type(int irq_type)
{
    if (irq_type != SHDISP_IRQ_TYPE_DET && irq_type != SHDISP_IRQ_TYPE_ALS)
           return SHDISP_RESULT_FAILURE;

    return SHDISP_RESULT_SUCCESS;
}
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_emg_on                                             */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_emg_on(void)
{
    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_EMG_ON, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_emg_off                                            */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_emg_off(void)
{
    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_OFF) {
        return;
    }

    shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_BKL_EMG_OFF, 0);

    if (shdisp_bdic_bkl_mode != SHDISP_BDIC_BKL_MODE_OFF) {
        shdisp_bdic_PD_BKL_control(SHDISP_BDIC_REQ_START, 0);
    }
    return;
}
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_LCD_BKL_get_mode                                           */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_LD_LCD_BKL_get_mode(int *mode)
{

    if (shdisp_bdic_emg == SHDISP_BDIC_BKL_EMG_ON) {
        *mode = SHDISP_BKL_TBL_MODE_EMERGENCY;
    } else {
        *mode = SHDISP_BKL_TBL_MODE_NORMAL;
    }

    if (*mode >= NUM_SHDISP_BKL_TBL_MODE) {
        *mode = SHDISP_BKL_TBL_MODE_NORMAL;
    }

    return;
}




