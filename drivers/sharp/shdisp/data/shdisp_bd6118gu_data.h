/* drivers/sharp/shdisp/data/shdisp_bd6118gu_data.h  (Display Driver)
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

#ifndef SHDISP_BD6118GU_DATA_DEFAULT_H
#define SHDISP_BD6118GU_DATA_DEFAULT_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */


/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
#define SHDISP_BKL_FIX_TBL_NUM                  (256)
#define SHDISP_TRI_LED_COLOR_TBL_NUM            (8)
#if defined(CONFIG_MACH_PG40)
#define SHDISP_COL_VARI_KIND                    (5)
#else /* CONFIG_MACH_PG40 */
#define SHDISP_COL_VARI_KIND                    (3)
#endif /* CONFIG_MACH_PG40 */
#define SHDISP_HANDSET_COLOR_NONE               (0x00)
#define SHDISP_HANDSET_COLOR_WHITE              (0x01)
#define SHDISP_HANDSET_COLOR_PINK               (0x02)
#define SHDISP_HANDSET_COLOR_RED                (0x03)
#define SHDISP_HANDSET_COLOR_TURQUOISE          (0x05)
#define SHDISP_HANDSET_COLOR_BLACK              (0x06)
#define SHDISP_HANDSET_COLOR_GOLD               (0x07)
#define SHDISP_HANDSET_COLOR_PURPLE             (0x0A)
#define SHDISP_HANDSET_COLOR_NAVY               (0x0D)
#define SHDISP_BKL_EMERGENCY_LIMIT_FIX          (0x0E)

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

#if defined(CONFIG_MACH_PG40) || defined(CONFIG_MACH_PG42)
static const unsigned char shdisp_main_bkl_tbl[SHDISP_BKL_FIX_TBL_NUM] = {
    0x00,
    0x01,
    0x02,
    0x03,
    0x04,
    0x05,
    0x05,
    0x05,
    0x05,
    0x05,
    0x05,
    0x05,
    0x06,
    0x06,
    0x06,
    0x06,
    0x06,
    0x06,
    0x07,
    0x07,
    0x07,
    0x07,
    0x07,
    0x07,
    0x08,
    0x08,
    0x08,
    0x08,
    0x08,
    0x08,
    0x09,
    0x09,
    0x09,
    0x09,
    0x09,
    0x09,
    0x0A,
    0x0A,
    0x0A,
    0x0A,
    0x0A,
    0x0A,
    0x0B,
    0x0B,
    0x0B,
    0x0B,
    0x0B,
    0x0B,
    0x0C,
    0x0C,
    0x0C,
    0x0C,
    0x0C,
    0x0C,
    0x0D,
    0x0D,
    0x0D,
    0x0D,
    0x0D,
    0x0D,
    0x0E,
    0x0E,
    0x0E,
    0x0E,
    0x0E,
    0x0E,
    0x0F,
    0x0F,
    0x0F,
    0x0F,
    0x0F,
    0x10,
    0x10,
    0x10,
    0x10,
    0x11,
    0x11,
    0x11,
    0x11,
    0x12,
    0x12,
    0x12,
    0x12,
    0x13,
    0x13,
    0x13,
    0x13,
    0x14,
    0x14,
    0x14,
    0x14,
    0x15,
    0x15,
    0x15,
    0x15,
    0x16,
    0x16,
    0x16,
    0x16,
    0x17,
    0x17,
    0x17,
    0x17,
    0x18,
    0x18,
    0x18,
    0x18,
    0x19,
    0x19,
    0x19,
    0x19,
    0x1A,
    0x1A,
    0x1A,
    0x1A,
    0x1B,
    0x1B,
    0x1B,
    0x1B,
    0x1C,
    0x1C,
    0x1C,
    0x1C,
    0x1D,
    0x1D,
    0x1D,
    0x1D,
    0x1E,
    0x1E,
    0x1E,
    0x1F,
    0x1F,
    0x1F,
    0x20,
    0x20,
    0x20,
    0x21,
    0x21,
    0x21,
    0x22,
    0x22,
    0x22,
    0x23,
    0x23,
    0x23,
    0x24,
    0x24,
    0x24,
    0x25,
    0x25,
    0x25,
    0x26,
    0x26,
    0x26,
    0x27,
    0x27,
    0x27,
    0x28,
    0x28,
    0x28,
    0x29,
    0x29,
    0x29,
    0x2A,
    0x2A,
    0x2A,
    0x2B,
    0x2B,
    0x2B,
    0x2C,
    0x2C,
    0x2C,
    0x2D,
    0x2D,
    0x2D,
    0x2E,
    0x2E,
    0x2E,
    0x2F,
    0x2F,
    0x2F,
    0x30,
    0x30,
    0x31,
    0x31,
    0x32,
    0x32,
    0x33,
    0x33,
    0x34,
    0x34,
    0x35,
    0x35,
    0x36,
    0x37,
    0x37,
    0x38,
    0x38,
    0x39,
    0x39,
    0x3A,
    0x3A,
    0x3B,
    0x3B,
    0x3C,
    0x3C,
    0x3D,
    0x3D,
    0x3E,
    0x3E,
    0x3F,
    0x3F,
    0x40,
    0x40,
    0x41,
    0x41,
    0x42,
    0x42,
    0x43,
    0x43,
    0x44,
    0x44,
    0x45,
    0x45,
    0x46,
    0x46,
    0x47,
    0x47,
    0x48,
    0x48,
    0x49,
    0x49,
    0x4A,
    0x4A,
    0x4B,
    0x4B,
    0x4C,
    0x4C,
    0x4D,
    0x4D,
    0x4E,
    0x4E,
    0x4F,
    0x4F,
    0x50,
    0x50,
    0x51,
    0x51,
    0x52,
    0x52,
    0x53,
    0x53,
    0x54,
    0x54,
    0x55,
    0x55,
};
#else /* CONFIG_MACH_PG40 || CONFIG_MACH_PG42 */
static const unsigned char shdisp_main_bkl_tbl[SHDISP_BKL_FIX_TBL_NUM] = {
    0x00,
    0x01,
    0x02,
    0x03,
    0x04,
    0x05,
    0x05,
    0x05,
    0x05,
    0x05,
    0x05,
    0x05,
    0x06,
    0x06,
    0x06,
    0x06,
    0x06,
    0x06,
    0x07,
    0x07,
    0x07,
    0x07,
    0x07,
    0x07,
    0x08,
    0x08,
    0x08,
    0x08,
    0x08,
    0x08,
    0x09,
    0x09,
    0x09,
    0x09,
    0x09,
    0x09,
    0x0A,
    0x0A,
    0x0A,
    0x0A,
    0x0A,
    0x0A,
    0x0B,
    0x0B,
    0x0B,
    0x0B,
    0x0B,
    0x0B,
    0x0C,
    0x0C,
    0x0C,
    0x0C,
    0x0C,
    0x0C,
    0x0D,
    0x0D,
    0x0D,
    0x0D,
    0x0D,
    0x0D,
    0x0E,
    0x0E,
    0x0E,
    0x0E,
    0x0E,
    0x0E,
    0x0E,
    0x0F,
    0x0F,
    0x0F,
    0x10,
    0x10,
    0x11,
    0x11,
    0x12,
    0x12,
    0x12,
    0x13,
    0x13,
    0x14,
    0x14,
    0x14,
    0x15,
    0x15,
    0x16,
    0x16,
    0x17,
    0x17,
    0x17,
    0x18,
    0x18,
    0x19,
    0x19,
    0x19,
    0x1A,
    0x1A,
    0x1B,
    0x1B,
    0x1C,
    0x1C,
    0x1C,
    0x1D,
    0x1D,
    0x1E,
    0x1E,
    0x1E,
    0x1F,
    0x1F,
    0x20,
    0x20,
    0x20,
    0x21,
    0x21,
    0x22,
    0x22,
    0x23,
    0x23,
    0x23,
    0x24,
    0x24,
    0x25,
    0x25,
    0x25,
    0x26,
    0x26,
    0x27,
    0x27,
    0x27,
    0x28,
    0x28,
    0x28,
    0x29,
    0x29,
    0x29,
    0x2A,
    0x2A,
    0x2A,
    0x2B,
    0x2B,
    0x2B,
    0x2C,
    0x2C,
    0x2C,
    0x2D,
    0x2D,
    0x2D,
    0x2E,
    0x2E,
    0x2E,
    0x2F,
    0x2F,
    0x2F,
    0x30,
    0x30,
    0x30,
    0x31,
    0x31,
    0x31,
    0x32,
    0x32,
    0x32,
    0x33,
    0x33,
    0x33,
    0x34,
    0x34,
    0x34,
    0x35,
    0x35,
    0x35,
    0x36,
    0x36,
    0x36,
    0x37,
    0x37,
    0x37,
    0x38,
    0x38,
    0x38,
    0x39,
    0x39,
    0x39,
    0x3A,
    0x3A,
    0x3A,
    0x3B,
    0x3B,
    0x3B,
    0x3C,
    0x3C,
    0x3D,
    0x3D,
    0x3D,
    0x3E,
    0x3E,
    0x3E,
    0x3F,
    0x3F,
    0x40,
    0x40,
    0x40,
    0x41,
    0x41,
    0x41,
    0x42,
    0x42,
    0x43,
    0x43,
    0x43,
    0x44,
    0x44,
    0x44,
    0x45,
    0x45,
    0x46,
    0x46,
    0x46,
    0x47,
    0x47,
    0x47,
    0x48,
    0x48,
    0x48,
    0x49,
    0x49,
    0x4A,
    0x4A,
    0x4A,
    0x4B,
    0x4B,
    0x4B,
    0x4C,
    0x4C,
    0x4D,
    0x4D,
    0x4D,
    0x4E,
    0x4E,
    0x4E,
    0x4F,
    0x4F,
    0x50,
    0x50,
    0x50,
    0x51,
    0x51,
    0x51,
    0x52,
    0x52,
    0x53,
    0x53,
    0x53,
    0x54,
    0x54,
    0x54,
    0x55,
};
#endif /* CONFIG_MACH_PG40 || CONFIG_MACH_PG42 */

static const struct shdisp_bdic_led_color_index shdisp_triple_led_color_index_tbl[SHDISP_TRI_LED_COLOR_TBL_NUM] = {
    {0, 0, 0,  0}
   ,{1, 0, 0,  1}
   ,{0, 1, 0,  2}
   ,{1, 1, 0,  3}
   ,{0, 0, 1,  4}
   ,{1, 0, 1,  5}
   ,{0, 1, 1,  6}
   ,{1, 1, 1,  7}
};

static const unsigned char shdisp_clrvari_index[SHDISP_COL_VARI_KIND] = {
#if  defined(CONFIG_MACH_PG40)
    SHDISP_HANDSET_COLOR_BLACK,
    SHDISP_HANDSET_COLOR_PINK,
    SHDISP_HANDSET_COLOR_GOLD,
    SHDISP_HANDSET_COLOR_WHITE,
    SHDISP_HANDSET_COLOR_TURQUOISE,
#elif defined (CONFIG_MACH_NF1)
    SHDISP_HANDSET_COLOR_BLACK,
    SHDISP_HANDSET_COLOR_WHITE,
    SHDISP_HANDSET_COLOR_RED,
#else /* CONFIG_MACH_PG40 CONFIG_MACH_NF1 */
    SHDISP_HANDSET_COLOR_BLACK,
    SHDISP_HANDSET_COLOR_PINK,
    SHDISP_HANDSET_COLOR_GOLD,
#endif /* CONFIG_MACH_PG40 CONFIG_MACH_NF1 */
};

static const unsigned char shdisp_triple_led_tbl[SHDISP_COL_VARI_KIND][SHDISP_TRI_LED_COLOR_TBL_NUM][4] = {
#if  defined(CONFIG_MACH_PG40)
  { /* default */
    { 0x00, 0x00, 0x00, 0x00 },
    { 0x26, 0x00, 0x00, 0x02 },
    { 0x00, 0x0F, 0x00, 0x04 },
    { 0x14, 0x10, 0x00, 0x06 },
    { 0x00, 0x00, 0x3F, 0x01 },
    { 0x18, 0x00, 0x14, 0x03 },
    { 0x00, 0x14, 0x10, 0x05 },
    { 0x16, 0x14, 0x16, 0x07 }
  },
  {
    { 0x00, 0x00, 0x00, 0x00 },
    { 0x14, 0x00, 0x00, 0x02 },
    { 0x00, 0x08, 0x00, 0x04 },
    { 0x14, 0x10, 0x00, 0x06 },
    { 0x00, 0x00, 0x30, 0x01 },
    { 0x10, 0x00, 0x10, 0x03 },
    { 0x00, 0x10, 0x10, 0x05 },
    { 0x0A, 0x08, 0x0A, 0x07 }
  },
  {
    { 0x00, 0x00, 0x00, 0x00 },
    { 0x14, 0x00, 0x00, 0x02 },
    { 0x00, 0x08, 0x00, 0x04 },
    { 0x10, 0x08, 0x00, 0x06 },
    { 0x00, 0x00, 0x30, 0x01 },
    { 0x08, 0x00, 0x08, 0x03 },
    { 0x00, 0x10, 0x10, 0x05 },
    { 0x0A, 0x08, 0x0A, 0x07 }
  },
  {
    { 0x00, 0x00, 0x00, 0x00 },
    { 0x14, 0x00, 0x00, 0x02 },
    { 0x00, 0x08, 0x00, 0x04 },
    { 0x10, 0x08, 0x00, 0x06 },
    { 0x00, 0x00, 0x26, 0x01 },
    { 0x10, 0x00, 0x08, 0x03 },
    { 0x00, 0x08, 0x08, 0x05 },
    { 0x0A, 0x08, 0x0A, 0x07 }
  },
  {
    { 0x00, 0x00, 0x00, 0x00 },
    { 0x16, 0x00, 0x00, 0x02 },
    { 0x00, 0x06, 0x00, 0x04 },
    { 0x12, 0x08, 0x00, 0x06 },
    { 0x00, 0x00, 0x3A, 0x01 },
    { 0x10, 0x00, 0x10, 0x03 },
    { 0x00, 0x10, 0x10, 0x05 },
    { 0x12, 0x08, 0x12, 0x07 }
  }
#elif defined (CONFIG_MACH_NF1)
  { /* default */
    /* R     G     B  */
    { 0x00, 0x00, 0x00, 0x00 },
    { 0x10, 0x00, 0x00, 0x02 },
    { 0x00, 0x08, 0x00, 0x04 },
    { 0x10, 0x05, 0x00, 0x06 },
    { 0x00, 0x00, 0x28, 0x01 },
    { 0x10, 0x00, 0x12, 0x03 },
    { 0x00, 0x06, 0x06, 0x05 },
    { 0x08, 0x04, 0x05, 0x07 }
  },
  {
    { 0x00, 0x00, 0x00, 0x00 },
    { 0x10, 0x00, 0x00, 0x02 },
    { 0x00, 0x28, 0x00, 0x04 },
    { 0x08, 0x03, 0x00, 0x06 },
    { 0x00, 0x00, 0x28, 0x01 },
    { 0x09, 0x00, 0x10, 0x03 },
    { 0x00, 0x06, 0x10, 0x05 },
    { 0x06, 0x03, 0x05, 0x07 }
  },
  {
    { 0x00, 0x00, 0x00, 0x00 },
    { 0x10, 0x00, 0x00, 0x02 },
    { 0x00, 0x28, 0x00, 0x04 },
    { 0x03, 0x28, 0x00, 0x06 },
    { 0x00, 0x00, 0x28, 0x01 },
    { 0x05, 0x00, 0x20, 0x03 },
    { 0x00, 0x28, 0x05, 0x05 },
    { 0x02, 0x28, 0x08, 0x07 }
  },
#else /* CONFIG_MACH_PG40 CONFIG_MACH_NF1 */
  { /* default */
    { 0x00, 0x00, 0x00, 0x00 },
    { 0x26, 0x00, 0x00, 0x02 },
    { 0x00, 0x0F, 0x00, 0x04 },
    { 0x20, 0x10, 0x00, 0x06 },
    { 0x00, 0x00, 0x3F, 0x01 },
    { 0x18, 0x00, 0x14, 0x03 },
    { 0x00, 0x14, 0x10, 0x05 },
    { 0x16, 0x14, 0x16, 0x07 }
  },
  {
    { 0x00, 0x00, 0x00, 0x00 },
    { 0x14, 0x00, 0x00, 0x02 },
    { 0x00, 0x08, 0x00, 0x04 },
    { 0x12, 0x10, 0x00, 0x06 },
    { 0x00, 0x00, 0x30, 0x01 },
    { 0x10, 0x00, 0x10, 0x03 },
    { 0x00, 0x10, 0x10, 0x05 },
    { 0x0A, 0x08, 0x0A, 0x07 }
  },
  {
    { 0x00, 0x00, 0x00, 0x00 },
    { 0x14, 0x00, 0x00, 0x02 },
    { 0x00, 0x08, 0x00, 0x04 },
    { 0x14, 0x10, 0x00, 0x06 },
    { 0x00, 0x00, 0x30, 0x01 },
    { 0x08, 0x00, 0x08, 0x03 },
    { 0x00, 0x10, 0x10, 0x05 },
    { 0x0A, 0x08, 0x0A, 0x07 }
  },
#endif /* CONFIG_MACH_PG40 CONFIG_MACH_NF1 */
};
#endif /* SHDISP_BD6118GU_DATA_DEFAULT_H */

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
