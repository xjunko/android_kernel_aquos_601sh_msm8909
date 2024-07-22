/*
 *  shub-i2c.c - Linux kernel modules for Sensor Hub 
 *
 *  Copyright (C) 2014 LAPIS SEMICONDUCTOR CO., LTD.
 *
 *  This file is based on :
 *    ml610q792.c - Linux kernel modules for acceleration sensor
 *    http://android-dev.kyocera.co.jp/source/versionSelect_URBANO.html
 *    (C) 2012 KYOCERA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/moduleparam.h>
#include <linux/input-polldev.h>

#include <asm/uaccess.h> 
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
//#include <linux/delay.h>
#include <linux/io.h>
#include <linux/poll.h>
#include <asm/gpio.h>
#include <linux/types.h>
#include <linux/sched.h>
//#include <linux/earlysuspend.h> /* SHMDS_HUB_0111_01 del */
#include <linux/miscdevice.h>
#include "ml630q790.h"

static int32_t i2c_probe( struct i2c_client *client, const struct i2c_device_id *id);
static int32_t i2c_remove( struct i2c_client *client );
static int32_t i2c_suspend( struct i2c_client *client, pm_message_t mesg );
static int32_t i2c_resume( struct i2c_client *client );
static void    i2c_shutdown( struct i2c_client *client );

#ifdef CONFIG_OF
static struct of_device_id shub_match_table[] = {
	{.name = "sensorhub",},
	{ .compatible = "lapis,sensorhub", },
	{ },
};
#else
#define shub_match_table NULL;
#endif /* CONFIG_OF */

static const struct i2c_device_id i2c_id[] = {
	{ SENOSR_HUB_DRIVER_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, shub_id);

static struct i2c_driver interface_driver = {
    .driver = {
        .name    = SENOSR_HUB_DRIVER_NAME,
        .owner   = THIS_MODULE,
        .of_match_table = shub_match_table,
    },
    .probe       = i2c_probe,
    .remove      = i2c_remove,
    .resume      = i2c_resume,
    .suspend     = i2c_suspend,
    .shutdown    = i2c_shutdown,
    .id_table    = i2c_id,
};

struct i2c_client *this_client=NULL;

/* SHMDS_HUB_0109_02 add S */
static int shub_acc_axis_val = 0;
static int shub_gyro_axis_val = 0;
static int shub_mag_axis_val = 0;
/* SHMDS_HUB_0109_02 add E */

static int32_t i2c_remove( struct i2c_client *client )
{
	printk(KERN_INFO "i2c_remove \n");

    return 0;
}

static int32_t i2c_suspend( struct i2c_client *client, pm_message_t mesg )
{
	printk(KERN_INFO "i2c_suspend \n");

    shub_suspend(client, mesg);
    return 0;
}

static int32_t i2c_resume( struct i2c_client *client )
{
	printk(KERN_INFO "i2c_resume \n");

    shub_resume(client);
    return 0;
}

static void i2c_shutdown( struct i2c_client *client )
{
	printk(KERN_INFO "i2c_shutdown \n");
}

static int32_t i2c_probe( struct i2c_client *client, const struct i2c_device_id *id )
{
/* SHMDS_HUB_0109_02 add S */
    int rc;
    struct device_node *np = client->dev.of_node;
    u32 temp_val;
    rc = of_property_read_u32(np, "shub,shub_acc_axis_val", &temp_val);
    if (rc) {
        printk("[shub]Unable to read. shub,shub_acc_axis_val\n");
        shub_acc_axis_val = 0;
    }
    else {
        shub_acc_axis_val = temp_val;
    }

    rc = of_property_read_u32(np, "shub,shub_gyro_axis_val", &temp_val);
    if (rc) {
        printk("[shub]Unable to read. shub,shub_gyro_axis_val\n");
        shub_gyro_axis_val = 0;
    }
    else {
        shub_gyro_axis_val = temp_val;
    }

    rc = of_property_read_u32(np, "shub,shub_mag_axis_val", &temp_val);
    if (rc) {
        printk("[shub]Unable to read. shub,shub_mag_axis_val\n");
        shub_mag_axis_val = 0;
    }
    else {
        shub_mag_axis_val = temp_val;
    }
/* SHMDS_HUB_0109_02 add E */
	printk(KERN_INFO "i2c_probe \n");

    shub_set_gpio_no(client); /* SHMDS_HUB_0110_01 add */
	this_client = client;
    return shub_probe();
}

int32_t init_i2c(void)
{
    int32_t ret;

    ret = i2c_add_driver(&interface_driver);
    if(ret != 0){
		printk(KERN_INFO "can't regist i2c driver \n");
    }

    return ret;
}
EXPORT_SYMBOL(init_i2c);
MODULE_ALIAS("i2c:" SENOSR_HUB_DRIVER_NAME);

int32_t hostif_write_proc(uint8_t adr, const uint8_t *data, uint8_t size)
{
/* SHMDS_HUB_0114_03 mod S */
//    uint8_t send_data[256];
    uint8_t send_data[512+128];
/* SHMDS_HUB_0114_03 mod E */
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags =0,
			.len = size+1,
			.buf = send_data,
		},
	};

#if 0
	printk(KERN_INFO "[shub-i2c]hostif_write_proc adr = 0x%02X size = %d\n", adr, size);
	{
		int len1,len2,ofs;
		char work1[256];
		char work2[8];
		for( len1=0 ; len1<(size/16) ; len1++ ){
			printk(KERN_INFO "%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X",
					*(data+len1*16+0),
					*(data+len1*16+1),
					*(data+len1*16+2),
					*(data+len1*16+3),
					*(data+len1*16+4),
					*(data+len1*16+5),
					*(data+len1*16+6),
					*(data+len1*16+7),
					*(data+len1*16+8),
					*(data+len1*16+9),
					*(data+len1*16+10),
					*(data+len1*16+11),
					*(data+len1*16+12),
					*(data+len1*16+13),
					*(data+len1*16+14),
					*(data+len1*16+15)
					);
		}
		if( size%16 ){
			memset(work1, 0, sizeof(work1));
			ofs=0;
			for(len2=0;len2<(size%16);len2++){
				memset(work2, 0, sizeof(work2));
				sprintf(work2, "%X ", *(data+len2));
				strcat(work1, work2);
			}
			printk(KERN_INFO "%s", work1);
		}
	}
#endif

    send_data[0] = adr;
    memcpy(&send_data[1], data, size);

	if (i2c_transfer(this_client->adapter, msgs, 1) < 0) {
		pr_err("shub-i2c_write: transfer error\n");
		return -EIO;
	} else
		return 0;
}
EXPORT_SYMBOL(hostif_write_proc);

int32_t hostif_read_proc(uint8_t adr, uint8_t *data, uint16_t size)
{
    uint8_t send_data[FIFO_SIZE+1];     /* SHMDS_HUB_0323_01 mod (512 -> FIFO_SIZE+1) */
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = 1,
			.buf = send_data,
		},
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = data,
		},
	};

/* SHMDS_HUB_0323_01 add S */
    if (size > FIFO_SIZE){
        pr_err("%s: Invalid argument, size=%d\n", __FUNCTION__, size);
        return -EINVAL;
    }
/* SHMDS_HUB_0323_01 add E */
    send_data[0] = adr;
    memcpy(&send_data[1], data, size);

//	printk(KERN_INFO "[shub-i2c]hostif_read_proc adr = 0x%02X size = %d\n", adr, size);

	if (i2c_transfer(this_client->adapter, msgs, 2) < 0) {
		pr_err("shub-i2c_read: transfer error\n");
		return -EIO;
	} else {

#if 0
	{
		int len1,len2,ofs;
		char work1[512];
		char work2[8];

		printk(KERN_INFO "[shub-i2c]hostif_read_proc readData = \n");

		for( len1=0 ; len1<(size/16) ; len1++ ){
			printk(KERN_INFO "%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X",
					*(data+len1*16+0),
					*(data+len1*16+1),
					*(data+len1*16+2),
					*(data+len1*16+3),
					*(data+len1*16+4),
					*(data+len1*16+5),
					*(data+len1*16+6),
					*(data+len1*16+7),
					*(data+len1*16+8),
					*(data+len1*16+9),
					*(data+len1*16+10),
					*(data+len1*16+11),
					*(data+len1*16+12),
					*(data+len1*16+13),
					*(data+len1*16+14),
					*(data+len1*16+15)
					);
		}
		if( size%16 ){
			memset(work1, 0, sizeof(work1));
			ofs=0;
			for(len2=0;len2<(size%16);len2++){
				memset(work2, 0, sizeof(work2));
				sprintf(work2, "%X ", *(data+len2));
				strcat(work1, work2);
			}
			printk(KERN_INFO "%s", work1);
		}
	}
#endif

		return 0;
	}
}
EXPORT_SYMBOL(hostif_read_proc);

/* SHMDS_HUB_0109_02 add S */
int shub_get_acc_axis_val(void)
{
    printk("[shub]acc_axis_val=%d\n", shub_acc_axis_val);
    return shub_acc_axis_val;
}

int shub_get_gyro_axis_val(void)
{
    printk("[shub]gyro_axis_val=%d\n", shub_gyro_axis_val);
    return shub_gyro_axis_val;
}

int shub_get_mag_axis_val(void)
{
    printk("[shub]mag_axis_val=%d\n", shub_mag_axis_val);
    return shub_mag_axis_val;
}
/* SHMDS_HUB_0109_02 add E */

