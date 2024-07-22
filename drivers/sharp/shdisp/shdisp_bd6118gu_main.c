/* drivers/sharp/shdisp/shdisp_bd6118gu_main.c  (Display Driver)
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

#include "data/shdisp_bd6118gu_main_ctrl.h"


/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */

#define SHDISP_BDIC_REGSET(x)               (shdisp_bdic_seq_regset(x, ARRAY_SIZE(x)))
#ifdef SHDISP_NOT_SUPPORT_NO_OS
 #define SHDISP_BDIC_HW_INIT
#endif /* SHDISP_NOT_SUPPORT_NO_OS */

static int shdisp_bdic_register_driver(void);
#ifdef SHDISP_BDIC_HW_INIT
static int  shdisp_bdic_LD_hw_init(void);
static void shdisp_bdic_PD_hw_reset(void);
#endif /* SHDISP_BDIC_HW_INIT */
static int  shdisp_bdic_PD_set_active(int power_status);
static int  shdisp_bdic_PD_set_standby(void);
static int  shdisp_bdic_seq_regset(const shdisp_bdicRegSetting_t* regtable, int size);
static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size);
static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_read_check_reg(unsigned char reg, unsigned char *val);
static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size);
static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val);
static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk);
/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

static int rst_gpio = SHDISP_GPIO_NUM_BL_RST_N;
int bdic_irq_gpio = SHDISP_BDIC_INT_GPIO;
struct regulator *vddio_vreg;

/* ------------------------------------------------------------------------- */
/* SUPPORT API                                                               */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_boot_init                                                 */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_boot_init( void )
{
#ifdef SHDISP_BDIC_HW_INIT
    int ret;
#endif /* SHDISP_BDIC_HW_INIT */

    SHDISP_TRACE("in\n");

    shdisp_bdic_register_driver();

#ifdef SHDISP_BDIC_HW_INIT
    pin_config_set("1000000.pinctrl", "gp-30", PIN_CONFIG_OUTPUT);
    shdisp_SYS_Host_API_gpio_request(rst_gpio, "BL_RST_N");
    ret = shdisp_bdic_LD_hw_init();
    if (ret != SHDISP_RESULT_SUCCESS) {
        shdisp_SYS_API_set_Host_gpio(rst_gpio, SHDISP_GPIO_CTL_LOW);
        return SHDISP_BDIC_IS_NOT_EXIST;
    }
#endif /* SHDISP_BDIC_HW_INIT */

    SHDISP_TRACE("out\n");
    return SHDISP_BDIC_IS_EXIST;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_shutdown                                                  */
/* ------------------------------------------------------------------------- */

void shdisp_bdic_API_shutdown(void)
{
    shdisp_SYS_API_set_Host_gpio(rst_gpio, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_API_delay_us(6000);

}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_active                                                */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_set_active(int power_status)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret =  shdisp_bdic_PD_set_active(power_status);
    SHDISP_TRACE("out\n");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_set_standby                                               */
/* ------------------------------------------------------------------------- */

int shdisp_bdic_API_set_standby(void)
{
    int ret;

    SHDISP_TRACE("in\n");
    ret = shdisp_bdic_PD_set_standby();
    SHDISP_TRACE("out\n");

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_seq_regset                                                */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_seq_regset(const shdisp_bdicRegSetting_t *regtable, int size)
{
    int ret = 0;

    ret = shdisp_bdic_seq_regset(regtable, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_write_reg                                              */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_write_reg(reg,val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_multi_write_reg                                        */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size)
{
    int ret = 0;

    ret = shdisp_bdic_IO_multi_write_reg(reg, wval, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_read_reg                                               */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_read_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_multi_read_reg                                         */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret = 0;

    ret = shdisp_bdic_IO_multi_read_reg(reg, val, size);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_set_bit_reg                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_set_bit_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_set_bit_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_clr_bit_reg                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_clr_bit_reg(unsigned char reg, unsigned char val)
{
    int ret = 0;

    ret = shdisp_bdic_IO_clr_bit_reg(reg, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_API_IO_msk_bit_reg                                            */
/* ------------------------------------------------------------------------- */
int shdisp_bdic_API_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk)
{
    int ret = 0;

    ret = shdisp_bdic_IO_msk_bit_reg(reg, val, msk);
    return ret;
}

#ifdef SHDISP_BDIC_HW_INIT

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_LD_hw_init                                                    */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_LD_hw_init(void)
{
    SHDISP_TRACE("in\n")
    shdisp_bdic_PD_hw_reset();
    SHDISP_TRACE("out\n")
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_hw_reset                                                   */
/* ------------------------------------------------------------------------- */

static void shdisp_bdic_PD_hw_reset(void)
{
    SHDISP_TRACE("in\n");
    shdisp_SYS_API_set_Host_gpio(rst_gpio, SHDISP_GPIO_CTL_LOW);
    shdisp_SYS_API_delay_us(8000);
    shdisp_SYS_API_set_Host_gpio(rst_gpio, SHDISP_GPIO_CTL_HIGH);
    shdisp_SYS_API_delay_us(8000);
    SHDISP_TRACE("out\n");
}
#endif /* SHDISP_BDIC_HW_INIT */

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_set_active                                                 */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_PD_set_active(int power_status)
{
    SHDISP_TRACE("in\n");

    if (power_status == SHDISP_DEV_STATE_NOINIT) {
        SHDISP_BDIC_REGSET(shdisp_bdic_init);
    }
    SHDISP_BDIC_REGSET(shdisp_bdic_active);
    SHDISP_TRACE("out\n");

    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_PD_set_standby                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_PD_set_standby(void)
{
    SHDISP_TRACE("in\n");
    SHDISP_BDIC_REGSET(shdisp_bdic_standby);
    SHDISP_TRACE("out\n");

    return SHDISP_RESULT_SUCCESS;
}

/*---------------------------------------------------------------------------*/
/* shdisp_bdic_seq_regset                                                    */
/*---------------------------------------------------------------------------*/

static int shdisp_bdic_seq_regset(const shdisp_bdicRegSetting_t* regtable, int size)
{
    int i, cnt_bdic;
    int ret = SHDISP_RESULT_SUCCESS;
    shdisp_bdicRegSetting_t* tbl;
    unsigned char top_addr_bdic;
    unsigned char bBuf_bdic[16];

    cnt_bdic = 0;
    top_addr_bdic = 0x00;

    tbl = (shdisp_bdicRegSetting_t*)regtable;
    for (i = 0; i < size; i++) {
        if (((cnt_bdic > 0) && (tbl->flg != SHDISP_BDIC_STRM)) || (cnt_bdic == sizeof(bBuf_bdic))) {
            ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
            cnt_bdic = 0;
            top_addr_bdic = 0x00;
        }
        switch(tbl->flg) {
        case SHDISP_BDIC_STR:
            ret = shdisp_bdic_IO_write_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_SET:
            ret = shdisp_bdic_IO_set_bit_reg(tbl->addr, tbl->data);
            break;
        case SHDISP_BDIC_CLR:
            ret = shdisp_bdic_IO_clr_bit_reg(tbl->addr, tbl->mask);
            break;
        case SHDISP_BDIC_RMW:
            ret = shdisp_bdic_IO_msk_bit_reg(tbl->addr, tbl->data, tbl->mask);
            break;
        case SHDISP_BDIC_STRM:
            if (cnt_bdic == 0) {
                top_addr_bdic = tbl->addr;
            }
            bBuf_bdic[cnt_bdic] = tbl->data;
            cnt_bdic++;
            if ((i + 1) == size) {
                ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
                cnt_bdic = 0;
                top_addr_bdic = 0x00;
            }
            break;
        case SHDISP_BDIC_WAIT:
            shdisp_SYS_API_delay_us(tbl->wait);
            ret = SHDISP_RESULT_SUCCESS;
            break;
        default:
            break;
        }
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("bdic R/W Error addr=%02X, data=%02X, mask=%02X\n", tbl->addr, tbl->data, tbl->mask);
            continue;
        }
        if (tbl->wait > 0) {
            if ((cnt_bdic > 0) && (tbl->flg == SHDISP_BDIC_STRM)) {
                ret = shdisp_bdic_IO_multi_write_reg(top_addr_bdic, bBuf_bdic, cnt_bdic);
                cnt_bdic = 0;
                top_addr_bdic = 0x00;
            }
            shdisp_SYS_API_delay_us(tbl->wait);
        }
        tbl++;
    }
    return SHDISP_RESULT_SUCCESS;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_write_reg                                                  */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_write_reg(unsigned char reg, unsigned char val)
{
    int ret;

    if (shdisp_API_get_bdic_is_exist() != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> \n");
        return SHDISP_RESULT_SUCCESS;
    }
    ret = shdisp_SYS_API_bdic_i2c_write(reg, val);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_API_bdic_i2c_write.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_API_bdic_i2c_write.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_multi_write_reg                                            */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_multi_write_reg(unsigned char reg, unsigned char *wval, unsigned char size)
{
    int ret;

    if (shdisp_API_get_bdic_is_exist() != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> \n");
        return SHDISP_RESULT_SUCCESS;
    }

    ret = shdisp_SYS_API_bdic_i2c_multi_write(reg, wval, size);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_multi_write.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_multi_write.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_reg                                                   */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_reg(unsigned char reg, unsigned char *val)
{
    int ret;

    if (val == NULL) {
        return SHDISP_RESULT_FAILURE;
    }

    if (shdisp_API_get_bdic_is_exist() != SHDISP_BDIC_IS_EXIST) {
        SHDISP_ERR("<ACCESS_ERR> \n");
        return SHDISP_RESULT_SUCCESS;
    }

        ret = shdisp_bdic_IO_read_check_reg(reg, val);

    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_read_check_reg                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_read_check_reg(unsigned char reg, unsigned char *val)
{
    int ret;
    int retry = 0;
    unsigned char try_1st, try_2nd;
#ifdef SHDISP_RESET_LOG
    struct shdisp_dbg_error_code err_code;
#endif /* SHDISP_RESET_LOG */

    try_1st = 0;
    try_2nd = 0;

    for (retry = 0; retry < 3; retry++) {
        ret = shdisp_SYS_API_bdic_i2c_read(reg, &try_1st);

        if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
            SHDISP_ERR("<TIME_OUT> shdisp_SYS_API_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        } else if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_API_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE;
        }

        ret = shdisp_SYS_API_bdic_i2c_read(reg, &try_2nd);

        if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
            SHDISP_ERR("<TIME_OUT> shdisp_SYS_API_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE_I2C_TMO;
        } else if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_API_bdic_i2c_read.\n");
            return SHDISP_RESULT_FAILURE;
        }

        if (try_1st == try_2nd) {
            *val = try_1st;
            return SHDISP_RESULT_SUCCESS;
        } else if (retry == 2) {
            SHDISP_ERR("<OTHER> i2c read retry over! addr:0x%02X val:(1st:0x%02X, 2nd:0x%02X).\n",
                                                                            reg, try_1st, try_2nd);
#ifdef SHDISP_RESET_LOG
            err_code.mode = SHDISP_DBG_MODE_LINUX;
            err_code.type = SHDISP_DBG_TYPE_BDIC;
            err_code.code = SHDISP_DBG_CODE_RETRY_OVER;
            err_code.subcode = SHDISP_DBG_SUBCODE_I2C_READ;
            shdisp_dbg_api_err_output(&err_code, 0);
#endif /* SHDISP_RESET_LOG */
            *val = try_1st;
            return SHDISP_RESULT_SUCCESS;
        } else {
            SHDISP_WARN("<OTHER> i2c read retry (%d)! addr:0x%02X val:(1st:0x%02X, 2nd:0x%02X).\n",
                                                                            retry, reg, try_1st, try_2nd);
        }
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_API_bdic_i2c_read.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_multi_read_reg                                             */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_multi_read_reg(unsigned char reg, unsigned char *val, int size)
{
    int ret;
    int maxreg;

    if (val == NULL) {
        SHDISP_ERR("<NULL_POINTER> val.\n");
        return SHDISP_RESULT_FAILURE;
    }

    if ((size < 1) || (size > 8)) {
        SHDISP_ERR("<INVALID_VALUE> size(%d).\n", size);
        return SHDISP_RESULT_FAILURE;
    }

    maxreg = (int)reg + (size - 1);
    if (maxreg > 0x08) {
        SHDISP_ERR("<OTHER> register address overflow.\n");
        return SHDISP_RESULT_FAILURE;
    }

    ret = shdisp_SYS_API_bdic_i2c_multi_read(reg, val, size);

    if (ret == SHDISP_RESULT_SUCCESS) {
        return SHDISP_RESULT_SUCCESS;
    } else if (ret == SHDISP_RESULT_FAILURE_I2C_TMO) {
        SHDISP_ERR("<TIME_OUT> shdisp_SYS_bdic_i2c_multi_read.\n");
        return SHDISP_RESULT_FAILURE_I2C_TMO;
    }

    SHDISP_ERR("<RESULT_FAILURE> shdisp_SYS_bdic_i2c_multi_read.\n");
    return SHDISP_RESULT_FAILURE;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_set_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_set_bit_reg(unsigned char reg, unsigned char val)
{
    int ret;

    ret = shdisp_SYS_API_bdic_i2c_mask_write(reg, val, val);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_clr_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_clr_bit_reg(unsigned char reg, unsigned char val)
{
    int ret;

    ret = shdisp_SYS_API_bdic_i2c_mask_write(reg, 0x00, val);
    return ret;
}


/* ------------------------------------------------------------------------- */
/* shdisp_bdic_IO_msk_bit_reg                                                */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_IO_msk_bit_reg(unsigned char reg, unsigned char val, unsigned char msk)
{
    int ret;

    ret = shdisp_SYS_API_bdic_i2c_mask_write(reg, val, msk);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* Logical Driver                                                            */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* shdisp_bdic_remove                                                        */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_remove(struct platform_device *pdev)
{
    int ret = 0;
    struct pinctrl *pin;
    struct pinctrl_state *pin_state;
    pin = devm_pinctrl_get(&(pdev->dev));
    pin_state = pinctrl_lookup_state(pin, "bdic_irq_gpio_suspend");
    ret = pinctrl_select_state(pin, pin_state);
    if(ret){
         SHDISP_ERR("bdic_irq_gpio pinctrl_select_state()\n");
    }
    regulator_disable(vddio_vreg);
    regulator_put(vddio_vreg);
    return 0;
}

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_probe                                                         */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_probe(struct platform_device *pdev)
{
#ifdef CONFIG_OF
    struct resource *res;
    struct pinctrl *pin;
    struct pinctrl_state *pin_state;
    int rc = 0;

    SHDISP_TRACE("in pdev = 0x%p\n", pdev );

    if (pdev) {
        res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
        if (!res) {
            SHDISP_ERR("irq resouce err!!\n");
            rc = 0;
            goto probe_done;
        } else {
            shdisp_SYS_API_set_irq_port(res->start, pdev);
        }
        vddio_vreg = regulator_get(&(pdev->dev),"vddio");
        rc = PTR_RET(vddio_vreg);
        if(rc){
            SHDISP_ERR("%s: vddio get failed. rc=%d\n", __func__,rc);
            vddio_vreg = NULL;
        }
        if(vddio_vreg != NULL) {
            rc = regulator_enable(vddio_vreg);
            if (rc < 0) {
                SHDISP_ERR("vddio_vreg regulator_enable failed!\n");
            }
            shdisp_SYS_API_delay_us(150);
        }
        if (&(pdev->dev) != NULL) {
            rst_gpio = of_get_named_gpio(pdev->dev.of_node, "bdic_rst_gpio", 0);
            if (!gpio_is_valid(rst_gpio)) {
                SHDISP_ERR("rst gpio not specified");
            } else {
                SHDISP_DEBUG("rst gpio succusess!");
            }
            bdic_irq_gpio = of_get_named_gpio(pdev->dev.of_node, "bdic_irq_gpio", 0);
            if (!gpio_is_valid(bdic_irq_gpio)) {
                SHDISP_ERR("bdic_irq gpio not specified");
            } else {
                SHDISP_DEBUG("bdic_irq gpio succusess!");
            }
            pin = devm_pinctrl_get(&(pdev->dev));
            pin_state = pinctrl_lookup_state(pin, "bdic_irq_gpio_active");
            rc = pinctrl_select_state(pin, pin_state);
            if(rc){
                SHDISP_ERR("bdic_irq_gpio pinctrl_select_state()\n");
            }
        } else {
            SHDISP_ERR("pdev->dev is NULL");
        }
    }else {
        SHDISP_ERR("pdev is NULL");
    }

probe_done:
    SHDISP_TRACE("out rc = %d\n", rc );

    return rc;
#else
    return 0;
#endif /* CONFIG_OF */
}

#ifdef CONFIG_OF
static const struct of_device_id shdisp_bdic_dt_match[] = {
    { .compatible = "sharp,shdisp_bdic",},
    {}
};
#else
#define shdisp_bdic_dt_match NULL
#endif /* CONFIG_OF */

static struct platform_driver shdisp_bdic_driver = {
    .probe = shdisp_bdic_probe,
    .remove = shdisp_bdic_remove,
    .shutdown = NULL,
    .driver = {
        /*
         * Driver name must match the device name added in
         * platform.c.
         */
        .name = "shdisp_bdic",
        .of_match_table = shdisp_bdic_dt_match,
    },
};

/* ------------------------------------------------------------------------- */
/* shdisp_bdic_register_driver                                               */
/* ------------------------------------------------------------------------- */

static int shdisp_bdic_register_driver(void)
{
    return platform_driver_register(&shdisp_bdic_driver);
}
