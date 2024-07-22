/* drivers/sharp/shtrpd/shtrpd_debug_data.h  (Trackpad driver)
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

enum {
    SHTRPD_DBG_TYPE_REATI           = 0,
    SHTRPD_DBG_TYPE_NRST,
    SHTRPD_DBG_TYPE_MAX
};

enum {
    SHTRPD_DBG_CODE_NONE_STR        = 0,
    SHTRPD_DBG_CODE_ZILLIGAL,
    SHTRPD_DBG_CODE_WEAKCLINGREJECT,
    SHTRPD_DBG_CODE_INITIALSETUP,
    SHTRPD_DBG_CODE_INTTIMING,
    SHTRPD_DBG_CODE_POWERKEY,
    SHTRPD_DBG_CODE_INITIALATI,
    SHTRPD_DBG_CODE_SHOWRESET,
    SHTRPD_DBG_CODE_FLIPOPEN,
    SHTRPD_DBG_CODE_FLIPCLOSE,
    SHTRPD_DBG_CODE_PROBE,
    SHTRPD_DBG_CODE_MAX
};

enum {
    SHTRPD_DBG_SUBCODE_0            = 0,
    SHTRPD_DBG_SUBCODE_MAX
};

struct shtrpd_dbg_error_code {
    unsigned char type;
    unsigned char code;
    unsigned char subcode;
    char linestring[24];
};

extern char shtrpd_linestring_default[];
