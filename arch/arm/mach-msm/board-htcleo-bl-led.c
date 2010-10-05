/* linux/arch/arm/mach-msm/board-htcleo-bkl.c
 *
 * Copyright (c) 2010 Cotulla
 * Edited to Common Structure by Markinus
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/leds.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/msm_fb.h>
#include <linux/gpio.h>
#include <mach/msm_iomap.h>
#include <mach/atmega_microp.h>


//#define DEBUG_LCM
#ifdef DEBUG_LCM
#define LCMDBG(fmt, arg...) printk(fmt, ## arg)
#else
#define LCMDBG(fmt, arg...) {}
#endif

static struct led_trigger *htcleo_lcd_backlight;

int htcleo_brightness_onoff_bkl(int enable)
{
	int ret;
	uint8_t data[1];

	data[0] = enable ? 1 : 0;
	ret = microp_i2c_write(MICROP_I2C_WCMD_BL_EN, data, 1);
	if (ret != 0)
		pr_err("%s: set failed\n", __func__);
	return 0;
}

int htcleo_brightness_set_bkl(uint8_t value)
{
	int ret;
	uint8_t cmd[2], data[2];

	printk("microp_set_bkl(%d)\n", value);

	if (value > 9)
	{
		value = 9;
	}
	// disable autobrigtness
// CotullaTEST: Lsensor test, add 0x100
//	data[0] = 0;
	data[0] = 1;
	data[1] = 0;
	ret = microp_i2c_write(MICROP_I2C_WCMD_AUTO_BL_CTL, data, 2); // 23
	if (ret != 0)
		pr_err("%s: set auto light sensor fail\n", __func__);

	// setvalue
	cmd[0] = value << 4;
//	printk("22LEVEL %02X\n", cmd[0]);
	ret = microp_i2c_write(MICROP_I2C_WCMD_LCM_BL_MANU_CTL, cmd, 1); // 22
	if (ret < 0)
	{
		pr_err("%s: request adc fail\n", __func__);
		return -EIO;
	}

	return 0;
}

void htcleo_brightness_set(struct led_classdev *led_cdev, enum led_brightness val)
{
	led_cdev->brightness = val;

	// set brigtness level via MicroP
	LCMDBG("htcleo_brightness_set: %d\n", val);
	if (val > 255) val = 255;
	if (val < 30)
	{
		htcleo_brightness_onoff_bkl(0);
	}
	else
	{
		htcleo_brightness_onoff_bkl(1);
		htcleo_brightness_set_bkl((val - 30) / 23);
	}
}

static struct led_classdev htcleo_backlight_led = 
{
	.name = "lcd-backlight",
	.brightness = LED_FULL,
	.brightness_set = htcleo_brightness_set,
};

static int  htcleo_backlight_probe(struct platform_device *pdev)
{
	int rc;
	printk(KERN_INFO "%s: HTCLeo Backlight connect with microP: "
			"Probe\n", __func__);
	
	led_trigger_register_simple("lcd-backlight-gate", &htcleo_lcd_backlight);
	rc = led_classdev_register(&pdev->dev, &htcleo_backlight_led);
	if (rc)
	      LCMDBG("HTCLeo Backlight: failure on register led_classdev\n");
	return 0;

}

static int htcleo_backlight_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver htcleo_backlight_driver = {
	.probe		= htcleo_backlight_probe,
	.remove		= htcleo_backlight_remove,
	.driver		= {
		.name   = "htcleo-backlight",
		.owner  = THIS_MODULE,
	},
};

static int __init htcleo_backlight_init(void)
{
	return platform_driver_register(&htcleo_backlight_driver);

}

static void __exit htcleo_backlight_exit(void)
{
	platform_driver_unregister(&htcleo_backlight_driver);
}

module_init(htcleo_backlight_init);
module_exit(htcleo_backlight_exit);

MODULE_DESCRIPTION("BMA150 G-sensor driver");
MODULE_LICENSE("GPL");
