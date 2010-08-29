/* arch/arm/mach-msm/htc_wifi_nvs.c
 *
 * Code to extract WiFi calibration information from ATAG set up 
 * by the bootloader.
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Dmitry Shmidt <dimitrysh@google.com>
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
#include <linux/proc_fs.h>

#include <asm/setup.h>


static unsigned char *hardcoded_nvs = 
  "macaddr=00:11:22:33:44:55\n"\
  "sromrev=3\n"\
  "vendid=0x14e4\n"\
  "devid=0x432f\n"\
  "boardtype=0x4b9\n"\
  "boardrev=0x32\n"\
  "boardflags=0x200\n"\
  "xtalfreq=37400\n"\
  "aa2g=1\n"\
  "aa5g=0\n"\
  "ag0=255\n"\
  "pa0b0=6003\n"\
  "pa0b1=64086\n"\
  "pa0b2=65195\n"\
  "pa0itssit=62\n"\
  "pa0maxpwr=68\n"\
  "opo=0\n"\
  "mcs2gpo0=0x6666\n"\
  "mcs2gpo1=0x6666\n"\
  "rssismf2g=0xa\n"\
  "rssismc2g=0xb\n"\
  "rssisav2g=0x3\n"\
  "bxa2g=0\n"\
  "ccode=ALL\n"\
  "cctl=0x0\n"\
  "cckdigfilttype=0\n"\
  "ofdmdigfilttype=1\n"\
  "rxpo2g=2\n"\
  "boardnum=1\n"\
  "nocrc=1\n"\
  "otpimagesize=182\n"\
  "hwhdr=0x05ffff031030031003100000\n"\
  "RAW1=80 32 fe 21 02 0c 00 22 2a 01 01 00 00 c5 0 e6 00 00 00 00 00 40 00 00 ff ff 80 00 00 00 00 00 00 00 00 00 00 c8 00 00 00 00 00 00 00 00 00 00 00 00 00 ff 20 04 D0 2 29 43 21 02 0c 00 22 04 00 20 00 5A\n"\
  "sd_gpout=0\n"\
  "sd_oobonly=1\n";
 


static struct proc_dir_entry *wifi_calibration;


static int wifi_calibration_read_proc(char *page, char **start, off_t off,
					int count, int *eof, void *data)
{
	unsigned len;

	len = min(strlen(hardcoded_nvs),  (unsigned)count);
	memcpy(page, hardcoded_nvs, len);
	return len;
}

static int __init wifi_nvs_init(void)
{
	wifi_calibration = create_proc_entry("calibration", 0444, NULL);
	if (wifi_calibration != NULL) {
	        wifi_calibration->size = strlen(hardcoded_nvs);
		wifi_calibration->read_proc = wifi_calibration_read_proc;
		wifi_calibration->write_proc = NULL;
	}
	return 0;
}

device_initcall(wifi_nvs_init);
