/* drivers/sharp/shdisp/shdisp_bd6118gu_main.h  (Display Driver)
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

#ifndef SHDISP_BD6118GU_MAIN_H
#define SHDISP_BD6118GU_MAIN_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <sharp/shdisp_kerl.h>

/* ------------------------------------------------------------------------- */
/* TYPES                                                                     */
/* ------------------------------------------------------------------------- */

enum {
    SHDISP_BDIC_STR,
    SHDISP_BDIC_SET,
    SHDISP_BDIC_CLR,
    SHDISP_BDIC_RMW,
    SHDISP_BDIC_STRM,
    SHDISP_BDIC_BANK,
    SHDISP_BDIC_WAIT,
#ifndef SHDISP_NOT_SUPPORT_PSALS
    SHDISP_ALS_STR,
    SHDISP_ALS_RMW,
    SHDISP_ALS_STRM,
    SHDISP_ALS_STRMS
#endif
};

typedef struct {
    unsigned char   addr;
    unsigned char   flg;
    unsigned char   data;
    unsigned char   mask;
    unsigned long   wait;
} shdisp_bdicRegSetting_t;

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */
int  shdisp_bdic_API_boot_init( void ) ;
void shdisp_bdic_API_shutdown(void);
int  shdisp_bdic_API_set_active(int power_status) ;
int  shdisp_bdic_API_set_standby(void) ;
int shdisp_bdic_API_seq_regset(const shdisp_bdicRegSetting_t *regtable, int size);
int shdisp_bdic_API_IO_write_reg(unsigned char reg, unsigned char val);
int shdisp_bdic_API_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size);
int shdisp_bdic_API_IO_read_reg(unsigned char reg, unsigned char *val);
int shdisp_bdic_API_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size);
int shdisp_bdic_API_IO_set_bit_reg(unsigned char reg, unsigned char val);
int shdisp_bdic_API_IO_clr_bit_reg(unsigned char reg, unsigned char val);
int shdisp_bdic_API_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk);
#endif
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
