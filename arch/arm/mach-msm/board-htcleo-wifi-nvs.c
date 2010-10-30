/* arch/arm/mach-msm/htc_wifi_nvs.c
 *
 * Code to extract WiFi calibration information from ATAG set up 
 * by the bootloader.
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Dmitry Shmidt <dimitrysh@google.com>
 * Changed for nand read for Leo by Markinus
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
#include <linux/mtd/mtd.h>
#include <linux/mtd/blktrans.h>

#define NVS_MAX_SIZE		0x800U
#define NVS_MACADDR_SIZE	0x1AU
#define WLAN_SKB_BUF_NUM	16
//---------PATCH for mac address------------
#define MAC_ADDRESS_LEN_C   17
#define ETHER_ADDR_LEN		6
//------------------------------------------


static unsigned char wifi_nvs_ram[NVS_MAX_SIZE];
static struct proc_dir_entry *wifi_calibration;
static unsigned char *nvs_mac_addr = "macaddr=00:11:22:33:44:55\n";
static unsigned char *hardcoded_nvs = 
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

#include <asm/setup.h>

static struct proc_dir_entry *wifi_calibration;

//---------PATCH for mac address-----------------
static char htcleo_mac_address_c[MAC_ADDRESS_LEN_C+1];
int user_mac_address = 0; 
static int __init htcleo_macaddress_setup(char *bootconfig) 
{
	int ret;
	unsigned int tmp[ETHER_ADDR_LEN];

	printk("%s: cmdline mac config=%s | %s\n",__FUNCTION__, bootconfig, __FILE__);
	
	strncpy(htcleo_mac_address_c, bootconfig, MAC_ADDRESS_LEN_C);
	ret = sscanf(htcleo_mac_address_c, "%2x:%2x:%2x:%2x:%2x:%2x", tmp, tmp+1, tmp+2, tmp+3, tmp+4, tmp+5);
	if (ret==ETHER_ADDR_LEN)
	{
		strcpy(nvs_mac_addr, "macaddr=");
		strcat(nvs_mac_addr, htcleo_mac_address_c);
		strcat(nvs_mac_addr, "\n");
		printk("%s parsed macaddr=%s | %s\n",__FUNCTION__,nvs_mac_addr, __FILE__);
		user_mac_address = 1;
	}
	printk("%s parsed mac_address=%2x:%2x:%2x:%2x:%2x:%2x | %s\n",__FUNCTION__,
	       tmp[0], tmp[1], tmp[2],
	       tmp[3], tmp[4], tmp[5], __FILE__);
	
    return 1;
}
__setup("wifi.mac=", htcleo_macaddress_setup);
//------------------------------------------



unsigned char *get_wifi_nvs_ram( void )
{
	return hardcoded_nvs;
}
EXPORT_SYMBOL(get_wifi_nvs_ram);

static int parse_tag_msm_wifi(void)
{
	unsigned size;
#ifdef NVS_MSM_WIFI_DEBUG
	unsigned i;
#endif
	int devnum = 0;
	int ret = 0;
	char* maddr=wifi_nvs_ram;
	struct mtd_info *mtd;

	DEBUG(MTD_DEBUG_LEVEL0, "MTD_open\n");
	mtd = get_mtd_device(NULL, devnum);

	if (IS_ERR(mtd)) {
		ret = PTR_ERR(mtd);
		goto out;
	}

	if (MTD_ABSENT == mtd->type) {
		put_mtd_device(mtd);
		ret = -ENODEV;
		goto out;
	}

	if(mtd->read(mtd, (0x7e40 * NVS_MAX_SIZE), NVS_MAX_SIZE, &size, wifi_nvs_ram)) {
		put_mtd_device(mtd);
		ret = 1;
		goto out;
	}
	
	put_mtd_device(mtd);
	while(memcmp(maddr, "macaddr=", 8)!=0) { 
		if((char*)++maddr>(char*)(wifi_nvs_ram+NVS_MAX_SIZE-NVS_MACADDR_SIZE))
			break;
		else
			maddr++;
	}
	if (!user_mac_address)
		if((char*)maddr<(char*)(wifi_nvs_ram+NVS_MAX_SIZE-NVS_MACADDR_SIZE)) nvs_mac_addr = maddr;
	
	
#ifdef NVS_MSM_WIFI_DEBUG
	printk("WiFi Data size = %d \n", size);
	for(i=0;( i < size );i++) {
		printk("%02x ", wifi_nvs_ram[i]);
	}
#endif	
	return 0;
out:
	return ret;
}

static unsigned wifi_get_nvs_size( void )
{
	unsigned len;

	len = strlen(hardcoded_nvs)+NVS_MACADDR_SIZE;
	return len;
}

int wifi_calibration_size_set(void)
{
	if (wifi_calibration != NULL)
		wifi_calibration->size = wifi_get_nvs_size();
	return 0;
}

static int wifi_calibration_read_proc(char *page, char **start, off_t off,
					int count, int *eof, void *data)
{
	unsigned char *ptr;
	unsigned len;
#ifdef NVS_MSM_WIFI_DEBUG
	unsigned i;
#endif

	memcpy(page, nvs_mac_addr, NVS_MACADDR_SIZE);
	ptr = get_wifi_nvs_ram();
	len = min(wifi_get_nvs_size(), (unsigned)count);
	memcpy(page+NVS_MACADDR_SIZE, ptr, strlen(hardcoded_nvs));
#ifdef NVS_MSM_WIFI_DEBUG
	printk("WiFi Data len = %d \n", len);
	for(i=0;( i < len );i++) {
		printk("%c", *page++);
	}
#endif	
	return len;
}

static int __init wifi_nvs_init(void)
{
	pr_info("%s\n", __func__);
	parse_tag_msm_wifi();
	wifi_calibration = create_proc_entry("calibration", 0444, NULL);
	if (wifi_calibration != NULL) {
		wifi_calibration->size = wifi_get_nvs_size();
		wifi_calibration->read_proc = wifi_calibration_read_proc;
		wifi_calibration->write_proc = NULL;
	}
	return 0;
}

late_initcall(wifi_nvs_init);
