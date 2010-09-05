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
#include <linux/i2c-msm.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/bootmem.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/regulator/machine.h>
#include <linux/leds.h>
#ifdef CONFIG_SENSORS_BMA150_SPI
#include <linux/bma150.h>
#endif
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
#include <mach/msm_serial_hs.h>
#include <mach/perflock.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>

#include <mach/board-htcleo-microp.h>

#include "board-htcleo.h"
#include "board-htcleo-ts.h"
#include "devices.h"
#include "proc_comm.h"
#include "dex_comm.h"

extern int __init htcleo_init_mmc(unsigned debug_uart);
extern void __init htcleo_audio_init(void);

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
///////////////////////////////////////////////////////////////////////
// Headset
///////////////////////////////////////////////////////////////////////

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
};

static struct platform_device htc_headset_mgr = {
	.name	= "HTC_HEADSET_MGR",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_mgr_data,
	},
};

static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio		= HTCLEO_GPIO_HDS_DET,
	.mic_detect_gpio	= HTCLEO_GPIO_HDS_MIC,
	.microp_channel		= 1,
	.key_enable_gpio	= NULL,
	.mic_select_gpio	= NULL,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};



///////////////////////////////////////////////////////////////////////
// Compass
///////////////////////////////////////////////////////////////////////
static struct akm8973_platform_data compass_platform_data =
{
	.layouts = HTCLEO_LAYOUTS,
	.project_name = HTCLEO_PROJECT_NAME,
	.reset = HTCLEO_GPIO_COMPASS_RST_N,
	.intr = HTCLEO_GPIO_COMPASS_INT_N,
};

///////////////////////////////////////////////////////////////////////
// Microp
///////////////////////////////////////////////////////////////////////

static struct bma150_platform_data htcleo_g_sensor_pdata = {
	.microp_new_cmd = 0,
};

static struct platform_device microp_devices[] = {
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &htcleo_g_sensor_pdata,
		},
	},
	{
		.name = "htcleo-backlight",
		.id = -1,
	},
	{
		.name = "htcleo-proximity",
		.id = -1,
	},
	{
		.name = "htcleo-leds",
		.id = -1,
	},
	{
		.name = "htcleo-lsensor",
		.id = -1,
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = HTCLEO_GPIO_UP_RESET_N,
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
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(HTCLEO_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(HTCLEO_GPIO_COMPASS_INT_N),
	},
	{
	        I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
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
// Camera
///////////////////////////////////////////////////////////////////////

static uint32_t camera_off_gpio_table[] =
{
	PCOM_GPIO_CFG(0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] =
{
	PCOM_GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* MCLK */
};

void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table, ARRAY_SIZE(camera_on_gpio_table));
}

void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table, ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] =
{
	{
		.start	= MSM_VFE_PHYS,
		.end	= MSM_VFE_PHYS + MSM_VFE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		 INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data =
{
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static int flashlight_control(int mode)
{
	return aat1271_flashlight_control(mode);
}

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data =
{
	.sensor_name = "s5k3e2fx",
	.sensor_reset = 144,
	/* CAM1_PWDN, enabled in a9 */
	//.sensor_pwd = 143,
	/* CAM1_VCM_EN, enabled in a9 */
	//.vcm_pwd = 31,
	.pdata = &msm_camera_device_data,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.flash_cfg = &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_s5k3e2fx =
{
	.name     = "msm_camera_s5k3e2fx",
	.dev      = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};

///////////////////////////////////////////////////////////////////////
// bluetooth
///////////////////////////////////////////////////////////////////////

static uint32_t bt_gpio_table[] =
{
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_CTS, 2, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_RX, 2, GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_CHIP_WAKE, 0, GPIO_OUTPUT,GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(HTCLEO_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA),
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata =
{
	/* Chip to Device */
	.rx_wakeup_irq = MSM_GPIO_TO_INT(HTCLEO_GPIO_BT_HOST_WAKE),
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 0,

	/* for bcm */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin   = HTCLEO_GPIO_BT_CHIP_WAKE,
	.host_wakeup_pin = HTCLEO_GPIO_BT_HOST_WAKE,

};
#endif

static void __init htcleo_bt_init(void)
{
	config_gpio_table(bt_gpio_table, ARRAY_SIZE(bt_gpio_table));
}


static struct platform_device htcleo_rfkill =
{
	.name = "htcleo_rfkill",
	.id = -1,
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

///////////////////////////////////////////////////////////////////////
// Memory 
///////////////////////////////////////////////////////////////////////

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
        .start		= MSM_PMEM_CAMERA_BASE,
	.size 		= MSM_PMEM_CAMERA_SIZE,
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
	.id		= 4,
	.dev		= {
		.platform_data = &android_pmem_adsp_pdata,
	},
};

static struct platform_device android_pmem_camera_device = {
	.name		= "android_pmem",
	.id		= 6,
	.dev		= {
		.platform_data = &android_pmem_camera_pdata,
	},
};
///////////////////////////////////////////////////////////////////////
// RAM-Console
///////////////////////////////////////////////////////////////////////

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
///////////////////////////////////////////////////////////////////////
// Power/Battery
///////////////////////////////////////////////////////////////////////

static struct platform_device htcleo_power  =
{
	.name = "htcleo_power",
	.id = -1,
};
///////////////////////////////////////////////////////////////////////
// Real Time Clock
///////////////////////////////////////////////////////////////////////

struct platform_device msm_device_rtc = {
	.name = "msm_rtc",
	.id = -1,
};
///////////////////////////////////////////////////////////////////////
// Platform Devices
///////////////////////////////////////////////////////////////////////


static struct platform_device *devices[] __initdata =
{
	&ram_console_device,
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
	&msm_device_smd,
	&htcleo_rfkill,
	&msm_device_rtc,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_camera_device,
	&msm_device_i2c,
	&msm_kgsl_device,
	&msm_camera_sensor_s5k3e2fx,
	&htcleo_flashlight_device,
	&htcleo_power,
	&qsd_device_spi,
	&htc_headset_mgr,
	&htc_headset_gpio,
};
///////////////////////////////////////////////////////////////////////
// Vibrator
///////////////////////////////////////////////////////////////////////

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
///////////////////////////////////////////////////////////////////////
// I2C
///////////////////////////////////////////////////////////////////////

static struct msm_i2c_device_platform_data msm_i2c_pdata = {
	.i2c_clock = 100000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_8MA,
};

static void __init msm_device_i2c_init(void)
{
	msm_i2c_gpio_init();
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}
///////////////////////////////////////////////////////////////////////
// Clocks
///////////////////////////////////////////////////////////////////////

static struct msm_acpu_clock_platform_data htcleo_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 245000,
	.wait_for_irq_khz	= 245000,
//	.wait_for_irq_khz	= 19200,   // TCXO
};

static unsigned htcleo_perf_acpu_table[] = {
	245000000,
	576000000,
	998400000,
};

static struct perflock_platform_data htcleo_perflock_data = {
	.perf_acpu_table = htcleo_perf_acpu_table,
	.table_size = ARRAY_SIZE(htcleo_perf_acpu_table),
};
///////////////////////////////////////////////////////////////////////
// Reset
///////////////////////////////////////////////////////////////////////

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
///////////////////////////////////////////////////////////////////////
// Init
///////////////////////////////////////////////////////////////////////

static void __init htcleo_init(void)
{
	volatile unsigned *bank6_in, *bank6_out;
	printk("htcleo_init()\n");

	msm_hw_reset_hook = htcleo_reset;

	do_grp_reset();

	msm_acpu_clock_init(&htcleo_clock_data);
	
	perflock_init(&htcleo_perflock_data);

	init_dex_comm();
	
	/* set the gpu power rail to manual mode so clk en/dis will not
	* turn off gpu power, and hang it on resume */

	htcleo_kgsl_power_rail_mode(0);
	htcleo_kgsl_power(false);
	mdelay(100);
	htcleo_kgsl_power(true);
	
#ifdef CONFIG_MICROP_COMMON
	htcleo_microp_init();
#endif
	msm_device_i2c_init();

	platform_add_devices(devices, ARRAY_SIZE(devices));

#ifdef CONFIG_SERIAL_MSM_HS
  	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_device_uart_dm1.name = "msm_serial_hs_bcm"; /* for bcm */
    	msm_device_uart_dm1.resource[3].end = 6;
#endif

	i2c_register_board_info(0, base_i2c_devices, ARRAY_SIZE(base_i2c_devices));

#ifdef CONFIG_USB_ANDROID
	htcleo_add_usb_devices();
#endif
	
	htcleo_init_mmc(0);
	platform_device_register(&htcleo_timed_gpios);

	htcleo_bt_init();
	htcleo_audio_init();
	
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

///////////////////////////////////////////////////////////////////////
// Bootfunctions
///////////////////////////////////////////////////////////////////////

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

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static void __init pmem_sf_size_setup(char **p)
{
	pmem_sf_size = memparse(*p, p);
}
__early_param("pmem_sf_size=", pmem_sf_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
	pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);

static void __init htcleo_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_sf_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}
}

static void __init htcleo_map_io(void)
{
	msm_map_common_io();
	htcleo_allocate_memory_regions();
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
