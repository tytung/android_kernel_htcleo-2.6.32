/* board-htcleo-hds.c
*
* Copyright (C) 2010 Cotulla
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>
#include <mach/htc_35mm_jack.h>

#include "board-htcleo.h"
#include "gpio_chip.h"


int microp_get_remote_adc(uint32_t *val);
int microp_set_adc_req(uint8_t value, int enable);


static int headset_has_mic(void);
static int enable_headset_plug_event(void);
static int enable_key_event(void);
static int disable_key_event(void);

static struct h35mm_platform_data htcleo_h35mm_data = 
{
    .plug_event_enable = enable_headset_plug_event,
    .headset_has_mic   = headset_has_mic,
    .key_event_enable  = enable_key_event,
    .key_event_disable = disable_key_event,
};

static struct platform_device htcleo_h35mm = 
{
    .name           = "htc_headset",
    .id             = -1,
    .dev            = 
    {
        .platform_data  = &htcleo_h35mm_data,
    },
};

/////////////////////////////////////////////////////////////////////////////////////////////

struct hds_data 
{
    int inited;
    int gpio_mic;
    int gpio_det;
    int irq_mic;
    int irq_det;
    int headset_is_in;
    int is_hpin_pin_stable;
    int last_pressed_key;
    struct work_struct work_det;
    struct work_struct work_mic;
    struct delayed_work hpin_debounce_work;
};

static struct hds_data hds;
/////////////////////////////////////////////////////////////////////////////////////////////

static int headset_detect_mic(void)
{
    return !gpio_get_value(hds.gpio_mic);
}

static int headset_is_in(void)
{
    return !gpio_get_value(hds.gpio_det);
}

static int get_remote_keycode(int *keycode)
{
    uint32_t val;
    uint32_t btn = 0;

    microp_set_adc_req(1, 1);
    if (microp_get_remote_adc(&val))
    {
        // failed. who know why? ignore
        *keycode = 0;
        return 1;
    }

    if((val >= 0) && (val <= 33))
    {
        btn = 1;
    }
    else if((val >= 38) && (val <= 82))
    {
        btn = 2;
    }
    else if((val >= 95) && (val <= 200))
    {
        btn = 3;
    }
    else if(val > 200)
    {   
        // check previous key
        if (hds.last_pressed_key)
        {
            *keycode = hds.last_pressed_key | 0x80;
            hds.last_pressed_key = 0;
            return 0;
        }
        *keycode = 0;
        return 1;
    }

    hds.last_pressed_key = btn;
    *keycode = btn;
    return 0;
}

static int headset_has_mic(void)
{
    int mic1 = -1;
    int mic2 = -1;
    int count = 0;

    mic2 = headset_detect_mic();

    /* debounce the detection wait until 2 consecutive read are equal */
    while ((mic1 != mic2) && (count < 10)) 
    {
        mic1 = mic2;
        msleep(600);
        mic2 = headset_detect_mic();
        count++;
    }

    printk("%s: microphone (%d) %s\n", __func__, count, mic1 ? "present" : "not present");

    return mic1;
}

static int enable_headset_plug_event(void)
{
    uint16_t stat;

    enable_irq(hds.irq_det);

    // see if headset state has changed
    stat = headset_is_in();
    printk("headsetisin: old %d new %d\n", hds.headset_is_in, stat);
    if (hds.headset_is_in != stat) 
    {
        hds.headset_is_in = stat;
        printk("Headset state changed\n");
        htc_35mm_jack_plug_event(stat, &hds.is_hpin_pin_stable);
    } 
    return 1;
}

static int enable_key_event(void)
{
    printk("enable_key_event\n");
    enable_irq(hds.irq_mic);
    return 0;
}

static int disable_key_event(void)
{
    printk("disable_key_event\n");
    disable_irq(hds.irq_mic);
    return 0;
}

static void hpin_debounce_do_work(struct work_struct *work)
{
    int insert = 0;

    insert = headset_is_in();
    printk("debonce new %d old %d\n", insert, hds.headset_is_in);
    if (insert != hds.headset_is_in) 
    {
        // clear keypress state
        hds.last_pressed_key = 0;

        hds.headset_is_in = insert;
        printk("headset %s\n", insert ? "inserted" : "removed");
        htc_35mm_jack_plug_event(hds.headset_is_in, &hds.is_hpin_pin_stable);
    }	
}


static void det_intr_work_func(struct work_struct *work)
{
    int value1;

    printk("det_intr_work_func\n");
    hds.is_hpin_pin_stable = 0;
    // TODO:
    //	wake_lock_timeout(&microp_i2c_wakelock, 3 * HZ);
    if (!hds.headset_is_in)
        schedule_delayed_work(&hds.hpin_debounce_work, msecs_to_jiffies(500));
    else
        schedule_delayed_work(&hds.hpin_debounce_work, msecs_to_jiffies(300));
}

static void mic_intr_work_func(struct work_struct *work)
{
    int keycode = 0;
    int value1;

    printk("mic_intr_work_func\n");
    if ((get_remote_keycode(&keycode) == 0) && (hds.is_hpin_pin_stable)) 
    {
        printk("keycode %d\n", keycode);
        htc_35mm_key_event(keycode, &hds.is_hpin_pin_stable);
    }
}


static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
    int value1, value2;
    int retry_limit = 10;

    do 
    {
        value1 = gpio_get_value(hds.gpio_det);
        set_irq_type(hds.irq_det, value1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
        value2 = gpio_get_value(hds.gpio_det);

    } while (value1 != value2 && retry_limit-- > 0);

    schedule_work(&hds.work_det);
    return IRQ_HANDLED;
}

static irqreturn_t mic_irq_handler(int irq, void *dev_id)
{
    int value1, value2;
    int retry_limit = 10;

    do 
    {
        value1 = gpio_get_value(hds.gpio_mic);
        set_irq_type(hds.irq_mic, value1 ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
        value2 = gpio_get_value(hds.gpio_mic);

    } while (value1 != value2 && retry_limit-- > 0);

    schedule_work(&hds.work_mic);
    return IRQ_HANDLED;
}

/////////////////////////////////////////////////////////////////////////////////////////////

static int htcleo_hds_probe(struct platform_device *pdev)
{
    int rc = 0;

    printk("htcleo_hds_probe()\n");

    // allow run it only once
    if (hds.inited != 0)
    {
        return -1;
    }
    hds.last_pressed_key = 0;
    hds.inited = 1;
    hds.gpio_mic = HTCLEO_GPIO_HDS_MIC;
    hds.gpio_det = HTCLEO_GPIO_HDS_DET;

    hds.irq_mic = gpio_to_irq(hds.gpio_mic); 
    hds.irq_det = gpio_to_irq(hds.gpio_det);

    gpio_request(hds.gpio_det, "hds_detect");
    gpio_request(hds.gpio_mic, "hds_mic");

    gpio_direction_input(hds.gpio_det);
    gpio_direction_input(hds.gpio_mic);

    hds.headset_is_in = 0;
    hds.is_hpin_pin_stable = 1;
    INIT_WORK(&hds.work_det, det_intr_work_func);
    INIT_WORK(&hds.work_mic, mic_intr_work_func);
    INIT_DELAYED_WORK(&hds.hpin_debounce_work, hpin_debounce_do_work);


    rc = request_irq(hds.irq_det, detect_irq_handler, IRQF_DISABLED | IRQF_TRIGGER_LOW, "hds_detect_intr", 0);
    rc = request_irq(hds.irq_mic, mic_irq_handler, IRQF_DISABLED | IRQF_TRIGGER_LOW, "hds_mic_intr", 0);
    disable_irq(hds.irq_mic);

    platform_device_register(&htcleo_h35mm);

    return rc;
}

static int htcleo_hds_remove(struct platform_device *dev)
{
    platform_device_unregister(&htcleo_h35mm);

    free_irq(hds.irq_det, 0);
    free_irq(hds.irq_mic, 0);

    gpio_free(hds.gpio_det);
    gpio_free(hds.gpio_mic);

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////


static struct platform_driver htcleo_hds_driver = 
{
    .probe = htcleo_hds_probe,
    .remove = htcleo_hds_remove,
    .driver = 
    {
        .name = "htcleo_hds",
        .owner = THIS_MODULE,
    },
};

static int __init htcleo_hds_init(void)
{
    return platform_driver_register(&htcleo_hds_driver);
}

static void __exit htcleo_hds_exit(void)
{
    platform_driver_unregister(&htcleo_hds_driver);
}

module_init(htcleo_hds_init);
module_exit(htcleo_hds_exit);

MODULE_AUTHOR("Cotulla");
MODULE_DESCRIPTION("HTC LEO headset driver");
MODULE_LICENSE("GPL");
