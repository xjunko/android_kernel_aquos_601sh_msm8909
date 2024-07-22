 /* drivers/sharp/shkeyled/shkeyled_kerl.c  (Key LED Driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

static struct regulator *reg_p;
#define SHKEYLED_PMIC_PORT_NAME "8916_l15"

#define SHKEYLED_LOG_TAG "SHKEYLEDkerl"

int shkeyled_err_log  = 1;
int shkeyled_dbg_log  = 0;

#if defined (CONFIG_ANDROID_ENGINEERING)
module_param(shkeyled_err_log,  int, 0600);
module_param(shkeyled_dbg_log,  int, 0600);
#endif /* CONFIG_ANDROID_ENGINEERING */

#define SHKEYLED_DEBUG_LOG(fmt, args...)\
		if(shkeyled_dbg_log == 1) { \
			printk(KERN_INFO "[%s][%s(%d)] " fmt"\n", SHKEYLED_LOG_TAG, __func__, __LINE__, ## args);\
		}


#define SHKEYLED_ERR_LOG(fmt, args...)\
		if(shkeyled_err_log == 1) { \
			printk(KERN_ERR "[%s][%s(%d)] " fmt"\n", SHKEYLED_LOG_TAG, __func__, __LINE__, ## args);\
		}


/* LED trigger */
//DEFINE_LED_TRIGGER(buttonbacklight_trigger);

static void shkeyled_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	int ret = 0;
	
	if(value == LED_OFF)
	{
		SHKEYLED_DEBUG_LOG("KEY_BL_OFF: value = %d", value);
		if (regulator_is_enabled(reg_p)) {
			ret = regulator_disable(reg_p);
			if(ret != 0){
				SHKEYLED_ERR_LOG("regulator_disble ret:%d",ret);
			}
		}
	}
	else if(value == LED_FULL)
	{
		SHKEYLED_DEBUG_LOG("KEY_BL_ON : value = %d", value);
		if (!regulator_is_enabled(reg_p)) {
			ret = regulator_enable(reg_p);
			if(ret != 0){
				SHKEYLED_ERR_LOG("regulator_enable ret:%d",ret);
			}
		}
	}
}

static struct led_classdev shkeyled_dev =
{
	.name			= "keyboard-backlight",
	.brightness_set	= shkeyled_set,
	.brightness		= LED_OFF,
};

static struct of_device_id key_backlight_of_match[] = {
	{ .compatible = "keyboard_backlight", },
	{ },
};
MODULE_DEVICE_TABLE(of, key_backlight_of_match);

static int shkeyled_probe(struct platform_device *pdev)
{
	int error = 0;
	struct device *dev = &pdev->dev;
	
	error = led_classdev_register(&pdev->dev, &shkeyled_dev);
	if (error)
	{
		SHKEYLED_ERR_LOG("led_classdev_register Error");
		return error;
	}
	
	reg_p = regulator_get(dev, SHKEYLED_PMIC_PORT_NAME);
	if (IS_ERR(reg_p)) {
		SHKEYLED_ERR_LOG("Unable to get %s regulator\n", SHKEYLED_PMIC_PORT_NAME);
		return -1;
	}
	
	shkeyled_set(&shkeyled_dev, LED_OFF);

	return 0;
}

static int shkeyled_remove(struct platform_device *pdev)
{
	shkeyled_set(&shkeyled_dev, LED_OFF);
	led_classdev_unregister(&shkeyled_dev);
	regulator_put(reg_p);
	
	return 0;
}

static struct platform_driver shkeyled_driver = {
	.probe		= shkeyled_probe,
	.remove		= shkeyled_remove,
	.driver		= {
		.name	= "shkeyled",
		.owner	= THIS_MODULE,
		.of_match_table = key_backlight_of_match,
	},
};

static int __init shkeyled_init(void)
{
	return platform_driver_register(&shkeyled_driver);
}

static void __exit shkeyled_exit(void)
{
	platform_driver_unregister(&shkeyled_driver);
}

module_exit(shkeyled_exit);
module_init(shkeyled_init);

MODULE_DESCRIPTION("SHARP KEYLED DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
