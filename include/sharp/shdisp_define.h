/* include/sharp/shdisp_define.h  (Display Driver)
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

#ifndef SHDISP_DEFINE_H
#define SHDISP_DEFINE_H

/* ------------------------------------------------------------------------- */
/* NOT SUPPORT FUNCTION                                                      */
/* ------------------------------------------------------------------------- */
#if defined(CONFIG_MACH_PG20) || defined(CONFIG_MACH_PG40) || defined(CONFIG_MACH_LYNX_DF40) || defined(CONFIG_MACH_DECKARD_KF40) || defined(CONFIG_MACH_NF1)
#define SHDISP_NOT_SUPPORT_DET_I2CERR
#define SHDISP_NOT_SUPPORT_PSALS
#if defined(SHDISP_FACTORY_MODE_ENABLE)
#define SHDISP_NOT_SUPPORT_DET
#endif
#else
#define SHDISP_NOT_SUPPORT_DET
#define SHDISP_NOT_SUPPORT_DET_I2CERR
#endif

#endif /* SHDISP_DEFINE_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
