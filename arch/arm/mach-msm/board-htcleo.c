/* linux/arch/arm/mach-msm/board-htcleo.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Author: Dima Zavin <dima@android.com>
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/bootmem.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/regulator/machine.h>
#include <linux/akm8973.h>
#include <../../../drivers/staging/android/timed_gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/msm_iomap.h>
#include <mach/perflock.h>
#include <mach/htc_usb.h>
#include <mach/msm_flashlight.h>


#include "board-htcleo.h"
#include "board-htcleo-ts.h"
#include "devices.h"
#include "proc_comm.h"
#include "dex_comm.h"

extern int __init htcleo_init_mmc(unsigned debug_uart);

///////////////////////////////////////////////////////////////////////
// SPI
///////////////////////////////////////////////////////////////////////

static struct platform_device qsd_device_spi =
{
    .name           = "spi_qsd",
    .id             = 0,
};

///////////////////////////////////////////////////////////////////////
// Regulator
///////////////////////////////////////////////////////////////////////

static struct regulator_consumer_supply tps65023_dcdc1_supplies[] =
{
    {
        .supply = "acpu_vcore",
    },
};

static struct regulator_init_data tps65023_data[5] =
{
    {
        .constraints =
		{
            .name = "dcdc1", /* VREG_MSMC2_1V29 */
            .min_uV = 1000000,
            .max_uV = 1300000,
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
        },
        .consumer_supplies = tps65023_dcdc1_supplies,
        .num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_supplies),
    },
    /* dummy values for unused regulators to not crash driver: */
    {
        .constraints = {
            .name = "dcdc2", /* VREG_MSMC1_1V26 */
            .min_uV = 1260000,
            .max_uV = 1260000,
        },
    },
    {
        .constraints = {
            .name = "dcdc3", /* unused */
            .min_uV = 800000,
            .max_uV = 3300000,
        },
    },
    {
        .constraints = {
            .name = "ldo1", /* unused */
            .min_uV = 1000000,
            .max_uV = 3150000,
        },
    },
    {
        .constraints = {
            .name = "ldo2", /* V_USBPHY_3V3 */
            .min_uV = 3300000,
            .max_uV = 3300000,
        },
    },
};

// Compass
static struct akm8973_platform_data compass_platform_data =
{
	.layouts = HTCLEO_LAYOUTS,
	.project_name = HTCLEO_PROJECT_NAME,
	.reset = HTCLEO_GPIO_COMPASS_RST_N,
	.intr = HTCLEO_GPIO_COMPASS_INT_N,
};

static struct i2c_board_info base_i2c_devices[] =
{
  	{
		I2C_BOARD_INFO("htcleo-battery", 0x26),
	},
	{
		I2C_BOARD_INFO(LEO_TOUCH_DRV_NAME, 0),
	},
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.platform_data = tps65023_data,
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(HTCLEO_GPIO_COMPASS_INT_N),
	},
};

///////////////////////////////////////////////////////////////////////
// USB 

///////////////////////////////////////////////////////////////////////

static uint32_t usb_phy_3v3_table[] =
{
    PCOM_GPIO_CFG(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA)
};

static int htcleo_phy_init_seq[] ={0x0C, 0x31, 0x30, 0x32, 0x1D, 0x0D, 0x1D, 0x10, -1};

#ifdef CONFIG_USB_ANDROID
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq		= htcleo_phy_init_seq,
	.phy_reset		= msm_hsusb_8x50_phy_reset,
	.accessory_detect = 0, /* detect by ID pin gpio */
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "Android Phone",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c02,
	.version	= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
static void htcleo_add_usb_devices(void)
{
	android_usb_pdata.products[0].product_id =
		android_usb_pdata.product_id;
	android_usb_pdata.serial_number = board_serialno();
	msm_hsusb_pdata.serial_number = board_serialno();
	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	config_gpio_table(usb_phy_3v3_table, ARRAY_SIZE(usb_phy_3v3_table));
	gpio_set_value(HTCLEO_GPIO_USBPHY_3V3_ENABLE, 1);
	platform_device_register(&msm_device_hsusb);
	platform_device_register(&usb_mass_storage_device);
	platform_device_register(&android_usb_device);
}

unsigned htcleo_get_vbus_state(void)
{
	if(readl(MSM_SHARED_RAM_BASE+0xef20c))
		return 1;
	else
		return 0;
}

#endif

///////////////////////////////////////////////////////////////////////
// Flashlight
///////////////////////////////////////////////////////////////////////

static uint32_t flashlight_gpio_table[] =
{
	PCOM_GPIO_CFG(HTCLEO_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static int config_htcleo_flashlight_gpios(void)
{
	config_gpio_table(flashlight_gpio_table, ARRAY_SIZE(flashlight_gpio_table));
	return 0;
}

static struct flashlight_platform_data htcleo_flashlight_data =
{
	.gpio_init  = config_htcleo_flashlight_gpios,
	.torch = HTCLEO_GPIO_FLASHLIGHT_TORCH,
	.flash = HTCLEO_GPIO_FLASHLIGHT_FLASH,
	.flash_duration_ms = 600
};

static struct platform_device htcleo_flashlight_device =
{
	.name = "flashlight",
	.dev =
	{
		.platform_data  = &htcleo_flashlight_data,
	},
};

///////////////////////////////////////////////////////////////////////
// KGSL (HW3D support)#include <linux/android_pmem.h>

///////////////////////////////////////////////////////////////////////

static struct resource msm_kgsl_resources[] =
{
	{
		.name	= "kgsl_reg_memory",
		.start	= MSM_GPU_REG_PHYS,
		.end	= MSM_GPU_REG_PHYS + MSM_GPU_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "kgsl_phys_memory",
		.start	= MSM_GPU_PHYS_BASE,
		.end	= MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
	},
};

static int htcleo_kgsl_power_rail_mode(int follow_clk)
{
	int mode = follow_clk ? 0 : 1;
	int rail_id = 0;
	return msm_proc_comm(PCOM_CLK_REGIME_SEC_RAIL_CONTROL, &rail_id, &mode);
}

static int htcleo_kgsl_power(bool on)
{
	int cmd;
	int rail_id = 0;

    	cmd = on ? PCOM_CLK_REGIME_SEC_RAIL_ENABLE : PCOM_CLK_REGIME_SEC_RAIL_DISABLE;
    	return msm_proc_comm(cmd, &rail_id, 0);
}

static struct platform_device msm_kgsl_device =
{
	.name		= "kgsl",
	.id		= -1,
	.resource	= msm_kgsl_resources,
	.num_resources	= ARRAY_SIZE(msm_kgsl_resources),
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name		= "pmem",
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name		= "pmem_adsp",
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name		= "pmem_camera",
        .start = MSM_PMEM_CAMERA_BASE,
	.size = MSM_PMEM_CAMERA_SIZE,
	.no_allocator	= 1,
	.cached		= 1,
};

static struct platform_device android_pmem_device = {
	.name		= "android_pmem",
	.id		= 0,
	.dev		= {
		.platform_data = &android_pmem_pdata
	},
};

static struct platform_device android_pmem_adsp_device = {
	.name		= "android_pmem",
	.id		= 1,
	.dev		= {
		.platform_data = &android_pmem_adsp_pdata,
	},
};

static struct platform_device android_pmem_camera_device = {
	.name		= "android_pmem",
	.id		= 2,
	.dev		= {
		.platform_data = &android_pmem_camera_pdata,
	},
};

static struct resource ram_console_resources[] = {
	{
		.start	= MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};


/* Battery */
static struct platform_device htcleo_power  =
{
	.name = "htcleo_power",
	.id = -1,
};

static struct platform_device *devices[] __initdata =
{
	&ram_console_device,
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
//	&bcm_bt_lpm_device,
//	&msm_device_uart_dm1,
	&msm_device_smd,
//	&htcleo_rfkill,
//	&msm_audio_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_camera_device,
	&msm_device_i2c,
//	&htcleo_backlight,
//	&htcleo_headset,
//	&msm_kgsl_device,
//	&capella_cm3602,
//	&msm_camera_sensor_s5k3e2fx,
	&htcleo_flashlight_device,
	&htcleo_power,
	&qsd_device_spi,

};

/* Vibrator */
static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = HTCLEO_GPIO_VIBRATOR_ON,
		.max_timeout = 15000,
	},
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device htcleo_timed_gpios = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &timed_gpio_data,
	},
};

static struct msm_acpu_clock_platform_data htcleo_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 245000,
	.wait_for_irq_khz	= 245000,
//	.wait_for_irq_khz	= 19200,   // TCXO
};

static void htcleo_reset(void)
{
	// 25 - 16 = 9
	while (1)
	{
	        writel(readl(MSM_GPIOCFG2_BASE + 0x504) | (1 << 9), MSM_GPIOCFG2_BASE + 0x504);// owner
		gpio_set_value(HTCLEO_GPIO_PS_HOLD, 0);
	}
}



static void do_grp_reset(void)
{
   	writel(0x20000, MSM_CLK_CTL_BASE + 0x214);
}

static void __init htcleo_init(void)
{
	volatile unsigned *bank6_in, *bank6_out;
	printk("htcleo_init()\n");

	msm_hw_reset_hook = htcleo_reset;

	do_grp_reset();

	msm_acpu_clock_init(&htcleo_clock_data);
	
	init_dex_comm();
	
	/* set the gpu power rail to manual mode so clk en/dis will not
	* turn off gpu power, and hang it on resume */
/*
	htcleo_kgsl_power_rail_mode(0);
	htcleo_kgsl_power(false);
	mdelay(100);
	htcleo_kgsl_power(true);
*/	
	platform_add_devices(devices, ARRAY_SIZE(devices));

	i2c_register_board_info(0, base_i2c_devices, ARRAY_SIZE(base_i2c_devices));

#ifdef CONFIG_USB_ANDROID
	htcleo_add_usb_devices();
#endif
	
	htcleo_init_mmc(0);
	platform_device_register(&htcleo_timed_gpios);
	
#ifdef CONFIG_USB_ANDROID
	msm_hsusb_set_vbus_state(htcleo_get_vbus_state());
#endif

	/* Blink the camera LED shortly to show that we're alive! */

	bank6_in = (unsigned int*)(MSM_GPIO1_BASE + 0x0864);
	bank6_out = (unsigned int*)(MSM_GPIO1_BASE + 0x0814);
	*bank6_out = *bank6_in ^ 0x200000;
	mdelay(50);
	*bank6_out = *bank6_in | 0x200000;
	mdelay(200);

}

static void __init htcleo_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = MSM_EBI1_BANK0_BASE;
	mi->bank[0].node = PHYS_TO_NID(MSM_EBI1_BANK0_BASE);
	mi->bank[0].size = MSM_EBI1_BANK0_SIZE;
}

#if defined(CONFIG_VERY_EARLY_CONSOLE)
#if defined(CONFIG_HTC_FB_CONSOLE)
int __init htc_fb_console_init(void);
#endif
#if defined(CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT)
int __init ram_console_early_init(void);
#endif
#endif

static void __init htcleo_map_io(void)
{
	msm_map_common_io();
	msm_clock_init();
	
#if defined(CONFIG_VERY_EARLY_CONSOLE)
// Init our consoles _really_ early
#if defined(CONFIG_HTC_FB_CONSOLE)
	htc_fb_console_init();
#endif
#if defined(CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT)
	ram_console_early_init();
#endif
#endif

}

extern struct sys_timer msm_timer;

MACHINE_START(HTCLEO, "htcleo")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= (CONFIG_PHYS_OFFSET + 0x00000100),
	.fixup		= htcleo_fixup,
	.map_io		= htcleo_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= htcleo_init,
	.timer		= &msm_timer,
MACHINE_END
