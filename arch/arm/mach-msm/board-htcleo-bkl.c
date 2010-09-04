/* linux/arch/arm/mach-msm/board-htcleo-bkl.c
 *
 * Copyright (c) 2010 Cotulla
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


//#define DEBUG_LCM
#ifdef DEBUG_LCM
#define LCMDBG(fmt, arg...) printk(fmt, ## arg)
#else
#define LCMDBG(fmt, arg...) {}
#endif



extern int microp_set_bkl(uint8_t value);
extern int micorp_onoff_bkl(int enable);

static struct led_trigger *htcleo_lcd_backlight;


void htcleo_brightness_set(struct led_classdev *led_cdev, enum led_brightness val)
{
    led_cdev->brightness = val;

    // set brigtness level via MicroP
    LCMDBG("htcleo_brightness_set: %d\n", val);
    if (val > 255) val = 255;
    if (val < 30)
    {
        micorp_onoff_bkl(0);
    }
    else
    {
        micorp_onoff_bkl(1);
        microp_set_bkl((val - 30) / 23);
    }
}

static struct led_classdev htcleo_backlight_led = 
{
    .name = "lcd-backlight",
    .brightness = LED_FULL,
    .brightness_set = htcleo_brightness_set,
};




static int htcleo_backlight_probe(struct platform_device *pdev)
{
    int rc;

    led_trigger_register_simple("lcd-backlight-gate", &htcleo_lcd_backlight);
    rc = led_classdev_register(&pdev->dev, &htcleo_backlight_led);
    if (rc)
        LCMDBG("backlight: failure on register led_classdev\n");
    return 0;
}


static struct platform_driver htcleo_backlight_driver = 
{
    .probe      = htcleo_backlight_probe,
    .driver     = 
    {
        .name   = "htcleo-backlight",
        .owner  = THIS_MODULE,
    },
};


static int __init htcleo_backlight_init(void)
{
    return platform_driver_register(&htcleo_backlight_driver);
}

module_init(htcleo_backlight_init);

// END OF FILE
