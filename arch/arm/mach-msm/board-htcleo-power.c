/* arch/arm/mach-msm/board-htcleo-power.c
*
* Copyright (C) 2010 Cotulla
* Copyright (C) 2008 HTC Corporation.
* Copyright (C) 2008 Google, Inc.
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

//
// calls seq:
//
// notify_vbus_change_intr -> vbus_work_func -> msm_hsusb_set_vbus_state -> USB -> notify_usb_connected
//

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <mach/msm_fast_timer.h>
#include <mach/msm_rpcrouter.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>

#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/switch.h>

#include "board-htcleo.h"

static char *supply_list[] =
{
    "battery",
};

static int inited;
static int vbus_present;
static int usb_status;
static struct work_struct vbus_work;

#ifdef CONFIG_USB_ANDROID
extern void notify_usb_connected(int status);
static struct t_usb_status_notifier usb_status_notifier = {
	.name = "htc_battery",
	.func = notify_usb_connected,
};
#endif

static int power_get_property(struct power_supply *psy,
                              enum power_supply_property psp,
                              union power_supply_propval *val)
{
    if (psp != POWER_SUPPLY_PROP_ONLINE)
    return -EINVAL;

    if (psy->type == POWER_SUPPLY_TYPE_MAINS)
    {
        val->intval = (usb_status == 2);
    }
    else
    {
        val->intval = vbus_present;
    }
    return 0;
}

static enum power_supply_property power_properties[] =
{
    POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply ac_supply =
{
    .name = "ac",
    .type = POWER_SUPPLY_TYPE_MAINS,
    .supplied_to = supply_list,
    .num_supplicants = ARRAY_SIZE(supply_list),
    .properties = power_properties,
    .num_properties = ARRAY_SIZE(power_properties),
    .get_property = power_get_property,
};

static struct power_supply usb_supply =
{
    .name = "usb",
    .type = POWER_SUPPLY_TYPE_USB,
    .supplied_to = supply_list,
    .num_supplicants = ARRAY_SIZE(supply_list),
    .properties = power_properties,
    .num_properties = ARRAY_SIZE(power_properties),
    .get_property = power_get_property,
};

static int get_vbus_state(void)
{
    if (readl(MSM_SHARED_RAM_BASE + 0xEF20C))
        return 1;
    else
        return 0;
}

void notify_cable_status(int status)
{
    pr_info("### notify_cable_status(%d) ###\n", status);
    vbus_present = status;
    msm_hsusb_set_vbus_state(vbus_present);
    power_supply_changed(&ac_supply);
    power_supply_changed(&usb_supply);
}

// called from USB driver
void notify_usb_connected(int status)
{
    if (!inited)    return;
    pr_info("### notify_usb_connected(%d) ###\n", status);
    usb_status = status;
    power_supply_changed(&ac_supply);
    power_supply_changed(&usb_supply);
}

// called from DEX intrrupt
void notify_vbus_change_intr(void)
{
    if (!inited)    return;
    schedule_work(&vbus_work);
}

// used by battery driver
int is_ac_power_supplied(void)
{
    return (usb_status == 2);
}

static void vbus_work_func(struct work_struct *work)
{
    int vbus = get_vbus_state();
    printk("  new vbus = %d\n", vbus);
#ifdef CONFIG_USB_EHCI_HCD
    if (vbus)
	return 0;
    else
	return 0;
#else
     if (vbus)
	gpio_set_value(HTCLEO_GPIO_POWER_USB, 0);
     else
	gpio_set_value(HTCLEO_GPIO_POWER_USB, 1);
#endif
     notify_cable_status(vbus);
}


static int htcleo_power_probe(struct platform_device *pdev)
{
    printk("$$$ htcleo_power_probe $$$\n");

    INIT_WORK(&vbus_work, vbus_work_func);

    gpio_request(HTCLEO_GPIO_POWER_USB, "power_usb");
    gpio_direction_output(HTCLEO_GPIO_POWER_USB, 0);

    power_supply_register(&pdev->dev, &ac_supply);
    power_supply_register(&pdev->dev, &usb_supply);

    inited = 1;
    // init VBUS state
    notify_vbus_change_intr();
    return 0;
}

//#define APP_BATT_PDEV_NAME		"rs30100001:00000000"
#define APP_BATT_PDEV_NAME		"htcleo_power"

static struct platform_driver htcleo_power_driver =
{
    .probe	= htcleo_power_probe,
    .driver	=
    {
        .name	= APP_BATT_PDEV_NAME,
        .owner	= THIS_MODULE,
    },
};

static int __init htcleo_power_init(void)
{
    printk("htcleo_power_init\n");
    platform_driver_register(&htcleo_power_driver);
#ifdef CONFIG_USB_ANDROID
    usb_register_notifier(&usb_status_notifier);
#endif
    return 0;
}

module_init(htcleo_power_init);
//later_init(htcleo_power_init);
MODULE_DESCRIPTION("HTCLEO Power Driver");
MODULE_LICENSE("GPL");
