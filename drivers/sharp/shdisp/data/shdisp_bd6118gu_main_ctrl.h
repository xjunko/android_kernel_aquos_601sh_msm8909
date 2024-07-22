/* drivers/sharp/shdisp/data/shdisp_bd6118gu_main_ctrl.h  (Display Driver)
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

#ifndef SHDISP_BD6118GU_MAIN_CTRL_H
#define SHDISP_BD6118GU_MAIN_CTRL_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include "../shdisp_bd6118gu.h"
#include "../shdisp_bd6118gu_main.h"

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static const shdisp_bdicRegSetting_t shdisp_bdic_init[] = {
     {BDIC_REG_PSCONT,              SHDISP_BDIC_STR,    0x24,                       0xFF,  10000}
    ,{BDIC_REG_GPOUT,               SHDISP_BDIC_CLR,    0x00,                       0xFF,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_active[] = {
     {BDIC_REG_PSCONT,              SHDISP_BDIC_CLR,    0x00,                       0x04,      0}
};

static const shdisp_bdicRegSetting_t shdisp_bdic_standby[] = {
     {BDIC_REG_PSCONT,              SHDISP_BDIC_SET,    0x04,                       0x04,      0}
};
#endif /* SHDISP_BD6118GU_MAIN_CTRL_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
