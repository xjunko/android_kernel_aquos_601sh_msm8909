/* include/sharp/shdisp_kerl.h  (Display Driver)
 *
 * Copyright (C) 2013 SHARP CORPORATION
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

#ifndef SHDISP_KERN_H
#define SHDISP_KERN_H


/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include "shdisp_context_def.h"
#include "shdisp_define.h"
#include "shdisp_ioctl.h"


/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define DEV_NAME    ("shdisp")
#define SHDISP_DEV    DEV_NAME

#define SHDISP_NOT_ADJUST_VCOM_VAL  0xFFFF


/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

enum {
    SHDISP_BDIC_I2C_M_W,
    SHDISP_BDIC_I2C_M_R,
    SHDISP_BDIC_I2C_M_R_MODE1,
    SHDISP_BDIC_I2C_M_R_MODE2,
    SHDISP_BDIC_I2C_M_R_MODE3,
    NUM_SHDISP_BDIC_I2C_M
};


#ifndef SHDISP_NOT_SUPPORT_PSALS
enum {
    SHDISP_PROX_SENSOR_POWER_OFF,
    SHDISP_PROX_SENSOR_POWER_ON,
    NUM_SHDISP_PROX_SENSOR_POWER
};
#endif


enum {
    SHDISP_IRQ_TYPE_ALS,
    SHDISP_IRQ_TYPE_PS,
    SHDISP_IRQ_TYPE_DET,
    SHDISP_IRQ_TYPE_I2CERR,
    NUM_SHDISP_IRQ_TYPE
};



enum {
    SHDISP_LEDC_PWR_STATUS_OFF,
    SHDISP_LEDC_PWR_STATUS_ON,
    NUM_SHDISP_LEDC_PWR_STATUS
};


enum {
    SHDISP_RESULT_SUCCESS,
    SHDISP_RESULT_FAILURE,
    SHDISP_RESULT_FAILURE_I2C_TMO,
    NUM_SHDISP_RESULT
};

enum {
    SHDISP_MAIN_BKL_CHG_OFF,
    SHDISP_MAIN_BKL_CHG_ON,
    NUM_SHDISP_MAIN_BKL_CHG
};

enum {
    SHDISP_LEDC_RGB_MODE_NORMAL,
    SHDISP_LEDC_RGB_MODE_PATTERN1,
    SHDISP_LEDC_RGB_MODE_PATTERN2,
    SHDISP_LEDC_RGB_MODE_PATTERN3,
    SHDISP_LEDC_RGB_MODE_PATTERN4,
    SHDISP_LEDC_RGB_MODE_PATTERN5,
    SHDISP_LEDC_RGB_MODE_PATTERN6,
    SHDISP_LEDC_RGB_MODE_ANIMATION1,
    SHDISP_LEDC_RGB_MODE_ANIMATION2,
    SHDISP_LEDC_RGB_MODE_ANIMATION3,
    SHDISP_LEDC_RGB_MODE_ANIMATION4,
    SHDISP_LEDC_RGB_MODE_ANIMATION5,
    SHDISP_LEDC_RGB_MODE_ANIMATION6,
    SHDISP_LEDC_RGB_MODE_ANIMATION7,
    SHDISP_LEDC_RGB_MODE_ANIMATION8,
    SHDISP_LEDC_RGB_MODE_ANIMATION9,
    SHDISP_LEDC_RGB_MODE_ANIMATION10,
    NUM_SHDISP_LEDC_RGB_MODE
};

enum {
    SHDISP_MAIN_DISP_CABC_MODE_OFF,
    SHDISP_MAIN_DISP_CABC_MODE_DBC,
    SHDISP_MAIN_DISP_CABC_MODE_ACC,
    SHDISP_MAIN_DISP_CABC_MODE_DBC_ACC,
    NUM_SHDISP_CABC_MODE
};

enum {
    SHDISP_MAIN_DISP_CABC_LUT0,
    SHDISP_MAIN_DISP_CABC_LUT1,
    SHDISP_MAIN_DISP_CABC_LUT2,
    SHDISP_MAIN_DISP_CABC_LUT3,
    SHDISP_MAIN_DISP_CABC_LUT4,
    SHDISP_MAIN_DISP_CABC_LUT5,
    NUM_SHDISP_CABC_LUT
};

enum {
    SHDISP_BLACKSCREEN_TYPE_T10,
    SHDISP_BLACKSCREEN_TYPE_T30,
    NUM_SHDISP_BLACKSCREEN_TYPE
};

#define SHDISP_REG_WRITE        (0x01)
#define SHDISP_SAVE_VALUE       (0x02)
#define SHDISP_SAVE_VALUE_LOW   (0x04)
#define SHDISP_RESET_VALUE      (0x08)

struct shdisp_procfs {
    int id;
    int par[4];
};

struct dma_abl_lut_gamma {
    struct dma_abl_color abl_color[256];
    unsigned long chksum;
};

struct dma_argc_lut_gamma{
    unsigned short red[16][4];
    unsigned long red_chksum;
    unsigned short green[16][4];
    unsigned long green_chksum;
    unsigned short blue[16][4];
    unsigned long blue_chksum;
};

struct shdisp_bdic_i2c_msg {
    unsigned short addr;
    unsigned char mode;
    unsigned char wlen;
    unsigned char rlen;
    const unsigned char *wbuf;
    unsigned char *rbuf;
};

struct shdisp_panel_param_str{
    unsigned short vcom_alpha;
    unsigned short vcom_alpha_low;
    unsigned int   shdisp_lcd;
};

struct shdisp_main_update {
    void *buf;
    unsigned short src_width;
    unsigned short src_xps;
    unsigned short src_yps;
    unsigned short upd_xsz;
    unsigned short upd_ysz;
    unsigned short dst_xps;
    unsigned short dst_yps;
};

struct shdisp_main_clear {
    unsigned long color;
    unsigned short start_xps;
    unsigned short start_yps;
    unsigned short end_xps;
    unsigned short end_yps;
};


struct shdisp_subscribe {
    int   irq_type;
    void (*callback)(void);
};



struct shdisp_prox_params {
    unsigned int threshold_low;
    unsigned int threshold_high;
};



struct shdisp_main_dbc {
    int mode;
    int auto_mode;
};



struct shdisp_check_cabc_val {
    int old_mode;
    int old_lut;
    int mode;
    int lut;
    int change;
};



/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

int shdisp_api_main_lcd_power_on(void);
int shdisp_api_main_lcd_power_off(void);
int shdisp_api_main_lcd_disp_on(void);
int shdisp_api_main_lcd_disp_off(void);
int shdisp_api_main_lcd_start_display(void);
int shdisp_api_main_lcd_post_video_start(void);
int shdisp_api_main_bkl_on(struct shdisp_main_bkl_ctl *bkl);
int shdisp_api_main_bkl_off(void);
int shdisp_api_shutdown(void);
#ifndef SHDISP_NOT_SUPPORT_PSALS
int shdisp_api_write_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
int shdisp_api_read_bdic_i2c(struct shdisp_bdic_i2c_msg *i2c_msg);
int shdisp_api_prox_sensor_pow_ctl(int power_mode, struct shdisp_prox_params *prox_params);
#endif
unsigned short shdisp_api_get_vcom(void);
unsigned short shdisp_api_get_vcom_low(void);
unsigned short shdisp_api_get_vcom_nvram(void);
struct shdisp_lcddr_phy_gamma_reg* shdisp_api_get_lcddr_phy_gamma(void);
struct shdisp_gamma_lut* shdisp_api_get_gamma_lut(void);
int shdisp_API_get_bdic_is_exist(void);
int shdisp_api_check_recovery (void);
int shdisp_api_event_subscribe(struct shdisp_subscribe *subscribe);
int shdisp_api_event_unsubscribe(int irq_type);

#ifndef SHDISP_NOT_SUPPORT_PSALS
int shdisp_api_photo_sensor_pow_ctl(void);
#endif
int shdisp_api_get_main_disp_status(void);
void shdisp_api_get_boot_context(void);
int shdisp_api_get_boot_disp_status(void);
int shdisp_api_display_done(void);
int shdisp_api_do_lcd_det_recovery(void);



#include "shdisp_boot_context.h"
#include "shdisp_kerl_context.h"


#endif /* SHDISP_KERN_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
