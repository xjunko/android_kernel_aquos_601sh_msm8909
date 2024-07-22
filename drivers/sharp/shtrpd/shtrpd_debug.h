/* drivers/sharp/shtrpd/shtrpd_debug.h  (Trackpad driver)
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
//#define SHTRPD_DEBUG_LOG_ENABLE
#define SHTRPD_SET_LOG_LV(lv)
#define SHTRPD_PRINTK(lv, fmt, args...) \
        if ((lv & (SHTRPD_LOG_LV_DEBUG | SHTRPD_LOG_LV_TRACE | SHTRPD_LOG_LV_WARN | SHTRPD_LOG_LV_ERR)) != 0) { \
            printk(fmt, ## args); \
        }

#define SHTRPD_LOG_LV_ERR       0x01
#define SHTRPD_LOG_LV_WARN      0x02
#define SHTRPD_LOG_LV_DEBUG     0x04
#define SHTRPD_LOG_LV_TRACE     0x08
#define SHTRPD_LOG_LV_INFO      0x10
#define SHTRPD_LOG_LV_PERFORM   0x20
#define SHTRPD_LOG_LV_PERFORM_DEBUG   0x40
#define SHTRPD_LOG_LV_I2CLOG    0x80

#if defined (SHTRPD_DEBUG_LOG_ENABLE)
#define SHTRPD_ERROR(fmt, args...) \
        SHTRPD_PRINTK(SHTRPD_LOG_LV_ERR, KERN_ERR "[SHTRPD_ERROR][%s] " fmt, __func__, ## args)

#define SHTRPD_TRACE(fmt, args...) \
        SHTRPD_PRINTK(SHTRPD_LOG_LV_TRACE, KERN_INFO "[SHTRPD_TRACE][%s] " fmt, __func__, ## args)

#define SHTRPD_DEBUG(fmt, args...) \
        SHTRPD_PRINTK(SHTRPD_LOG_LV_DEBUG, KERN_DEBUG "[SHTRPD_DEBUG][%s] " fmt, __func__, ## args)
#else
	#define SHTRPD_ERROR(fmt, args...)
	#define SHTRPD_TRACE(fmt, args...)
	#define SHTRPD_DEBUG(fmt, args...)
#endif

#define LOG_BUF_SIZE              (8192)
#define LINE_BUF_SIZE             (1024)

#define SHTRPD_DBG_KERNEL_FILE_CHECK_POINTER_ALWAYS_OK ((void *)1)

