/* arch/arm/mach-msm/board-htcleo-battery.c
*
* Copyright (C) 2010 Cotulla
* Copyright (C) 2009 HTC Corporation
* Copyright (C) 2009 Google, Inc.
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

/*

actually this file called "board-htcleo-battery.c", so it's HTC LEO specific only
most values are hard coded here really.
this driver designed for android.

proper ds2745 driver located at "driver/i2c/chips/ds2745.c"
I don't want and have not resources to support any "proper" linux drivers coding.
("proper" for them, not for me ofcourse)

my primary target was to make high quality battery support in Android for HTC LEO. kefir with us!

*/
//#define DEBUG

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <asm/gpio.h>

#include <linux/android_alarm.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/wakelock.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/i2c.h>

#include "gpio_chip.h"
#include "board-htcleo.h"


////////////////////////////////////////////////////////////////////////////////////////


#define LEO_BATTERY_CAPACITY    1230
#define LEO_BATTERY_EMPTY       500


/* Known commands to the DS2784 chip */
#define W1_DS2784_SWAP			0xAA
#define W1_DS2784_READ_DATA		0x69
#define W1_DS2784_WRITE_DATA		0x6C
#define W1_DS2784_COPY_DATA		0x48
#define W1_DS2784_RECALL_DATA		0xB8
#define W1_DS2784_LOCK			0x6A

/* Number of valid register addresses */
#define DS2784_DATA_SIZE		0x80

#define DS2784_EEPROM_BLOCK0		0x20
#define DS2784_ACTIVE_FULL		0x20
#define DS2784_EEPROM_BLOCK1		0x30
#define DS2784_RATED_CAPACITY		0x32
#define DS2784_CURRENT_OFFSET_BIAS	0x33
#define DS2784_ACTIVE_EMPTY		0x3b

/**
* The DS2482 registers - there are 3 registers that are addressed by a read
* pointer. The read pointer is set by the last command executed.
*
* To read the data, issue a register read for any address
*/
#define DS2482_CMD_RESET		0xF0	/* No param */
#define DS2482_CMD_SET_READ_PTR		0xE1	/* Param: DS2482_PTR_CODE_xxx */
#define DS2482_CMD_CHANNEL_SELECT	0xC3
#define DS2482_CMD_WRITE_CONFIG		0xD2	/* Param: Config byte */
#define DS2482_CMD_1WIRE_RESET		0xB4	/* Param: None */
#define DS2482_CMD_1WIRE_SINGLE_BIT	0x87	/* Param: Bit byte (bit7) */
#define DS2482_CMD_1WIRE_WRITE_BYTE	0xA5	/* Param: Data byte */
#define DS2482_CMD_1WIRE_READ_BYTE	0x96	/* Param: None */
/* Note to read the byte, Set the ReadPtr to Data then read (any addr) */
#define DS2482_CMD_1WIRE_TRIPLET	0x78	/* Param: Dir byte (bit7) */

/* Values for DS2482_CMD_SET_READ_PTR */
#define DS2482_PTR_CODE_STATUS		0xF0
#define DS2482_PTR_CODE_DATA		0xE1
#define DS2482_PTR_CODE_CHANNEL		0xD2	/* DS2482-800 only */
#define DS2482_PTR_CODE_CONFIG		0xC3


#define RAW_DATA_SIZE       10


////////////////////////////////////////////////////////////////////////////////////////


// from board-htcleo-power.c
extern int is_ac_power_supplied(void);

// from board-htcleo-log.c
extern double log(double x);


struct battery_status
{
    int timestamp;

    int voltage_uV;		/* units of uV */
    int current_uA;		/* units of uA */
    int current_avg_uA;
    int charge_uAh;

    s16 temp_C;		/* units of 0.1 C */

    u8 percentage;		/* battery percentage */
    u8 charge_source;
    u8 status_reg;
    u8 battery_full;	/* battery full (don't charge) */

    u8 cooldown;		/* was overtemp */
    u8 charge_mode;
} __attribute__((packed));


#define SOURCE_NONE	0
#define SOURCE_USB	1
#define SOURCE_AC	2

#define CHARGE_OFF	0
#define CHARGE_SLOW	1
#define CHARGE_FAST	2
#define CHARGE_BATT_DISABLE	3 /* disable charging at battery */

#define TEMP_CRITICAL	600 /* no charging at all */
#define TEMP_HOT	500 /* no fast charge, no charge > 4.1v */
#define TEMP_WARM	450 /* no fast charge above this */

#define TEMP_HOT_MAX_MV	4100 /* stop charging here when hot */
#define TEMP_HOT_MIN_MV	3800 /* resume charging here when hot */
#define CE_DISABLE_MIN_MV 4100

#define BATTERY_LOG_MAX 1024
#define BATTERY_LOG_MASK (BATTERY_LOG_MAX - 1)

/* When we're awake or running on wall power, sample the battery
* gauge every FAST_POLL seconds.  If we're asleep and on battery
* power, sample every SLOW_POLL seconds
*/
#define FAST_POLL	(1 * 60)
#define SLOW_POLL	(10 * 60)

static DEFINE_MUTEX(battery_log_lock);
static struct battery_status battery_log[BATTERY_LOG_MAX];
static unsigned battery_log_head;
static unsigned battery_log_tail;

void battery_log_status(struct battery_status *s)
{
    unsigned n;
    mutex_lock(&battery_log_lock);
    n = battery_log_head;
    memcpy(battery_log + n, s, sizeof(struct battery_status));
    n = (n + 1) & BATTERY_LOG_MASK;
    if (n == battery_log_tail)
        battery_log_tail = (battery_log_tail + 1) & BATTERY_LOG_MASK;
    battery_log_head = n;
    mutex_unlock(&battery_log_lock);
}

static const char *battery_source[3] = { "none", " usb", "  ac" };
static const char *battery_mode[4] = { " off", "slow", "fast", "full" };

static int battery_log_print(struct seq_file *sf, void *private)
{
    unsigned n;
    mutex_lock(&battery_log_lock);
    seq_printf(sf, "timestamp    mV     mA avg mA      uAh   dC   %%   src  mode   reg full\n");
    for (n = battery_log_tail; n != battery_log_head; n = (n + 1) & BATTERY_LOG_MASK)
    {
        struct battery_status *s = battery_log + n;
        seq_printf(sf, "%9d %5d %6d %6d %8d %4d %3d  %s  %s  0x%02x %d\n",
            s->timestamp, s->voltage_uV / 1000,
            s->current_uA / 1000, s->current_avg_uA / 1000,
            s->charge_uAh, s->temp_C,
            s->percentage,
            battery_source[s->charge_source],
            battery_mode[s->charge_mode],
            s->status_reg, s->battery_full);
    }
    mutex_unlock(&battery_log_lock);
    return 0;
}


struct htcleo_device_info
{
    struct device *dev;

    /* DS2784 data, valid after calling htcleo_battery_read_status() */
    char raw[RAW_DATA_SIZE];	/* raw HTCLEO data */
    uint32_t raw_status;

    struct battery_status status;

    struct power_supply bat;
    struct workqueue_struct *monitor_wqueue;
    struct work_struct monitor_work;
    struct alarm alarm;
    struct wake_lock work_wake_lock;

    //    int (*charge)(int on, int fast);
    //    struct w1_slave *w1_slave;
    struct i2c_client *client;

    u8 dummy; /* dummy battery flag */
    u8 last_charge_mode; /* previous charger state */
    u8 slow_poll;

    ktime_t last_poll;
    ktime_t last_charge_seen;
};

#define psy_to_dev_info(x) container_of((x), struct htcleo_device_info, bat)

static struct wake_lock vbus_wake_lock;

#define BATT_RSNSP			(67)	/*Passion battery source 1*/

static enum power_supply_property battery_properties[] =
{
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CURRENT_AVG,
    POWER_SUPPLY_PROP_CHARGE_COUNTER,
};

static int battery_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val);
static void htcleo_program_alarm(struct htcleo_device_info *di, int seconds);
static void battery_ext_power_changed(struct power_supply *psy);

#define to_htcleo_device_info(x) container_of((x), struct htcleo_device_info, bat);


//////////////////////////////////////////////////////////////////////////

static int I2C_Read_Status(struct htcleo_device_info *di)
{
    uint8_t i2c_msg[1];
    uint8_t i2c_data[2];

    i2c_msg[0] = 0x01; //status reg

    i2c_master_send(di->client, i2c_msg, 1);
    i2c_master_recv(di->client, i2c_data, 2);

    dev_dbg(&di->client->dev, "I2C_Read_Status() = %08X!\n", i2c_data[0]);
    di->raw_status = i2c_data[0];
    return 0;
}

static int I2C_Read_Data(struct htcleo_device_info *di)
{
    uint8_t i2c_msg[1];
    uint8_t i2c_data[10];

    i2c_msg[0] = 0x08; // AUX0 AUX1 VOLTAGE CURRENT ACR

    i2c_master_send(di->client, i2c_msg, 1);
    i2c_master_recv(di->client, i2c_data, 10);

    dev_dbg(&di->client->dev, "I2C_Read_Data() %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X!\n",
                i2c_data[0], i2c_data[1], i2c_data[2], i2c_data[3], i2c_data[4],
                i2c_data[5], i2c_data[6], i2c_data[7], i2c_data[8], i2c_data[9]);
    memcpy(di->raw, i2c_data, 10);
    return 0;
}

static int I2C_Write_ACR(struct htcleo_device_info *di, uint16_t val)
{
    uint8_t i2c_msg[3];

    i2c_msg[0] = 0x10;
    i2c_msg[1] = (val >> 8) & 0xFF;
    i2c_msg[2] = (val >> 0) & 0xFF;

 //   dev_dbg(&di->client->dev, "I2C_Write_ACR() = %04X!\n", val);
    i2c_master_send(di->client, i2c_msg, 3);
    return 0;
}

//////////////////////////////////////////////////////////////////////////

static int htcleo_charge(int on, int fast)
{
    gpio_direction_output(HTCLEO_GPIO_BATTERY_CHARGER_CURRENT, !!fast);
    gpio_direction_output(HTCLEO_GPIO_BATTERY_CHARGER_ENABLE, !on);
    return 0;
}

static void htcleo_parse_data(uint32_t raw_status, u8 *raw, struct battery_status *s)
{
    short n;
    uint32_t n32;
    uint32_t FL, ACR, ACR_EMPTY;

    /* Get status reg */
    s->status_reg = raw_status;

    /* Get Level */
    // TODO: FL too wrong (?)
    ACR = ((raw[8] << 8) | raw[9]);
    FL = (LEO_BATTERY_CAPACITY * 1570) / 625;
    ACR_EMPTY = (LEO_BATTERY_EMPTY * 1570) / 625;
    s->percentage = (100 * (ACR - ACR_EMPTY)) / FL;

    s->charge_uAh = 1000 * (((ACR - ACR_EMPTY) * 625) / 1570);
    printk("ACR=%d FL=%d RAAC=%d\n", ACR, ACR_EMPTY, s->percentage);

    if (s->percentage < 0 ) s->percentage = 0;
    if (s->percentage > 100 ) s->percentage = 100;


    /* Get Voltage */
    n32 = (((raw[4] << 8) |  raw[5]) / 16);
    //s->voltage_uV = (n32 * 244) / 100;
    s->voltage_uV = 1000 * ((n32 * 312) >> 7); // div to 128

    /* Get Current */
    n = ((raw[6]) << 8) | raw[7];
    s->current_uA = 1000 * (((n / 4) * 625) / 1570);

    // average current not supported by DS2746
    s->current_avg_uA = s->current_uA;

    /* Get Temperature */
    n = ((raw[0] << 8) | (raw[1]));
    n /= 16;

//    printk("temp = %x\n", n);
    if (n > 2047 || n == 0)
    {
        s->temp_C = 250;
    }
    else
    {
        double v = 0.021277 * (300.0 / (2047.0 / n - 1.0));
        s->temp_C = 10 * (1.0 / ((log(v) * 0.000290698) + 0.003354016) - 273.15);
    }
    if (s->temp_C < -250)
    {
        s->temp_C = -250;
    }
}

static int htcleo_battery_read_status(struct htcleo_device_info *di)
{
    // read status
    I2C_Read_Status(di);
    I2C_Read_Data(di);

 //   printk("status = %04X\n", di->raw_status);
    htcleo_parse_data(di->raw_status, di->raw, &di->status);

    dev_dbg(&di->client->dev, "batt: %3d%%, %d mV, %d mA (%d avg), %d.%d C, %d mAh\n",
        di->status.percentage,
        di->status.voltage_uV / 1000, di->status.current_uA / 1000,
        di->status.current_avg_uA / 1000,
        di->status.temp_C / 10, di->status.temp_C % 10,
        di->status.charge_uAh / 1000);

    return 0;
}

//////////////////////////////////////////////////////////////////////////

static int htcleo_set_cc(struct htcleo_device_info *di, bool enable)
{
    // not supported for DS2745, there no CE bit
    return 0;
}


static int battery_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
    struct htcleo_device_info *di = psy_to_dev_info(psy);

    switch (psp)
    {
    case POWER_SUPPLY_PROP_STATUS:
        switch (di->status.charge_source)
        {
        case CHARGE_OFF:
            val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
            break;
        case CHARGE_FAST:
        case CHARGE_SLOW:
            if (di->status.battery_full)
                val->intval = POWER_SUPPLY_STATUS_FULL;
            else if (di->status.charge_mode == CHARGE_OFF ||
                di->status.charge_mode == CHARGE_BATT_DISABLE)
                val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
            else
                val->intval = POWER_SUPPLY_STATUS_CHARGING;
            break;
        default:
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
            break;
        }
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        if (di->status.temp_C >= TEMP_HOT)
            val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
        else
            val->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        /* XXX todo */
        val->intval = 1;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        if (di->dummy)
            val->intval = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        else
            val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        if (di->dummy)
            val->intval = 75;
        else
            val->intval = di->status.percentage;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = di->status.voltage_uV;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = di->status.temp_C;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = di->status.current_uA;
        break;
    case POWER_SUPPLY_PROP_CURRENT_AVG:
        val->intval = di->status.current_avg_uA;
        break;
    case POWER_SUPPLY_PROP_CHARGE_COUNTER:
        val->intval = di->status.charge_uAh;
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static void htcleo_battery_update_status(struct htcleo_device_info *di)
{
    u8 last_level;
    last_level = di->status.percentage;

    htcleo_battery_read_status(di);

// CotullaTODO: add ACR = FL at top point here

    if ((last_level != di->status.percentage) || (di->status.temp_C > 450))
    {
        power_supply_changed(&di->bat);
    }
}

static DEFINE_MUTEX(charge_state_lock);

static bool check_timeout(ktime_t now, ktime_t last, int seconds)
{
    ktime_t timeout = ktime_add(last, ktime_set(seconds, 0));
    return ktime_sub(timeout, now).tv64 < 0;
}

static int battery_adjust_charge_state(struct htcleo_device_info *di)
{
    unsigned source;
    int rc = 0;
    int temp, volt;
	int ovp;
    u8 charge_mode;
    bool charge_timeout = false;

    mutex_lock(&charge_state_lock);

    temp = di->status.temp_C;
    volt = di->status.voltage_uV / 1000;

    source = di->status.charge_source;

    /* initially our charge mode matches our source:
    * NONE:OFF, USB:SLOW, AC:FAST
    */
    charge_mode = source;

    /* shut off charger when full:
    * - CHGTF flag is set
    */
#if 1
    /*if (di->status.status_reg & 0x80)
    {
        di->status.battery_full = 1;
        charge_mode = CHARGE_BATT_DISABLE;
    }
    else
    {
        di->status.battery_full = 0;
    }*/
       if(di->status.percentage >= 99)
       {
               di->status.battery_full = 1;
        charge_mode = CHARGE_BATT_DISABLE;
       }
       else
    {
        di->status.battery_full = 0;
    }
#else
    // CotullaTODO: add DS274X check code here
    di->status.battery_full = 0;
#endif

	ovp = gpio_get_value(HTCLEO_GPIO_BATTERY_OVER_CHG);
	if (ovp)
	{
	//	printk("OVERPOWER!!!! FACK!\n");
	}

    if (temp >= TEMP_HOT || ovp)
    {
        if (temp >= TEMP_CRITICAL)
        {
            charge_mode = CHARGE_BATT_DISABLE;
        }

        /* once we charge to max voltage when hot, disable
        * charging until the temp drops or the voltage drops
        */
        if (volt >= TEMP_HOT_MAX_MV)
        {
            di->status.cooldown = 1;
        }
    }

    /* when the battery is warm, only charge in slow charge mode */
    if ((temp >= TEMP_WARM) && (charge_mode == CHARGE_FAST))
        charge_mode = CHARGE_SLOW;

    if (di->status.cooldown)
    {
        if ((temp < TEMP_WARM) || (volt <= TEMP_HOT_MIN_MV))
            di->status.cooldown = 0;
        else
            charge_mode = CHARGE_BATT_DISABLE;
    }

    if (di->status.current_uA > 1024)
    {
        di->last_charge_seen = di->last_poll;
    }
    else if (di->last_charge_mode != CHARGE_OFF && check_timeout(di->last_poll, di->last_charge_seen, 60 * 60))
    {
        if (di->last_charge_mode == CHARGE_BATT_DISABLE)
        {
            /* The charger is only powering the phone. Toggle the
            * enable line periodically to prevent auto shutdown.
            */
            di->last_charge_seen = di->last_poll;
            pr_info("batt: charging POKE CHARGER\n");
            htcleo_charge(0, 0);
            udelay(10);
            htcleo_charge(1, source == CHARGE_FAST);
        }
        else
        {
            /* The charger has probably stopped charging. Turn it
            * off until the next sample period.
            */
            charge_timeout = true;
            charge_mode = CHARGE_OFF;
        }
    }

    if (source == CHARGE_OFF)
        charge_mode = CHARGE_OFF;

    /* Don't use CHARGE_BATT_DISABLE unless the voltage is high since the
    * voltage drop over the discharge-path diode can cause a shutdown.
    */
    if (charge_mode == CHARGE_BATT_DISABLE && volt < CE_DISABLE_MIN_MV)
        charge_mode = CHARGE_OFF;

    if (di->last_charge_mode == charge_mode)
        goto done;

    di->last_charge_mode = charge_mode;
    di->status.charge_mode = charge_mode;

    switch (charge_mode)
    {
    case CHARGE_OFF:
        htcleo_charge(0, 0);
        htcleo_set_cc(di, true);
        if (temp >= TEMP_CRITICAL)
            pr_info("batt: charging OFF [OVERTEMP]\n");
        else if (di->status.cooldown)
            pr_info("batt: charging OFF [COOLDOWN]\n");
        else if (di->status.battery_full)
            pr_info("batt: charging OFF [FULL]\n");
        else if (charge_timeout)
            pr_info("batt: charging OFF [TIMEOUT]\n");
        else
            pr_info("batt: charging OFF\n");
        break;
    case CHARGE_BATT_DISABLE:
        di->last_charge_seen = di->last_poll;
        htcleo_set_cc(di, false);
        htcleo_charge(1, source == CHARGE_FAST);
        if (temp >= TEMP_CRITICAL)
            pr_info("batt: charging BATTOFF [OVERTEMP]\n");
        else if (di->status.cooldown)
            pr_info("batt: charging BATTOFF [COOLDOWN]\n");
        else if (di->status.battery_full)
            pr_info("batt: charging BATTOFF [FULL]\n");
        else
            pr_info("batt: charging BATTOFF [UNKNOWN]\n");
        break;
    case CHARGE_SLOW:
        di->last_charge_seen = di->last_poll;
        htcleo_set_cc(di, true);
        htcleo_charge(1, 0);
        pr_info("batt: charging SLOW\n");
        break;
    case CHARGE_FAST:
        di->last_charge_seen = di->last_poll;
        htcleo_set_cc(di, true);
        htcleo_charge(1, 1);
        pr_info("batt: charging FAST\n");
        break;
    }
    rc = 1;
done:
    mutex_unlock(&charge_state_lock);
    return rc;
}



static void htcleo_battery_work(struct work_struct *work)
{
    struct htcleo_device_info *di = container_of(work, struct htcleo_device_info, monitor_work);
    struct timespec ts;
    unsigned long flags;

    htcleo_battery_update_status(di);

    di->last_poll = alarm_get_elapsed_realtime();

    if (battery_adjust_charge_state(di))
    {
        power_supply_changed(&di->bat);
    }

    ts = ktime_to_timespec(di->last_poll);
    di->status.timestamp = ts.tv_sec;
    battery_log_status(&di->status);

    /* prevent suspend before starting the alarm */
    local_irq_save(flags);
    wake_unlock(&di->work_wake_lock);
    htcleo_program_alarm(di, FAST_POLL);
    local_irq_restore(flags);
}

//////////////////////////////////////////////////////////////////////////

static void htcleo_program_alarm(struct htcleo_device_info *di, int seconds)
{
    ktime_t low_interval = ktime_set(seconds - 10, 0);
    ktime_t slack = ktime_set(20, 0);
    ktime_t next;

    next = ktime_add(di->last_poll, low_interval);

    alarm_start_range(&di->alarm, next, ktime_add(next, slack));
}

static void htcleo_battery_alarm(struct alarm *alarm)
{
    struct htcleo_device_info *di = container_of(alarm, struct htcleo_device_info, alarm);
    wake_lock(&di->work_wake_lock);
    queue_work(di->monitor_wqueue, &di->monitor_work);
}

static void battery_ext_power_changed(struct power_supply *psy)
{
    struct htcleo_device_info *di;
    int got_power;

    di = psy_to_dev_info(psy);
    got_power = power_supply_am_i_supplied(psy);

    if (got_power)
    {
        if (is_ac_power_supplied())
            di->status.charge_source = SOURCE_AC;
        else
            di->status.charge_source = SOURCE_USB;
        wake_lock(&vbus_wake_lock);
    }
    else
    {
        di->status.charge_source = SOURCE_NONE;
        /* give userspace some time to see the uevent and update
        * LED state or whatnot...
        */
        wake_lock_timeout(&vbus_wake_lock, HZ / 2);
    }
    battery_adjust_charge_state(di);
    power_supply_changed(psy);
}

static int htcleo_battery_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc;
    struct htcleo_device_info *di;

    printk("$$$ htcleo_battery_probe $$$\n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE))
    {
        return -EIO;
    }

    di = kzalloc(sizeof(*di), GFP_KERNEL);
    if (!di)
    {
        return -ENOMEM;
    }

    di->client = client;

    i2c_set_clientdata(client, di);

    gpio_request(HTCLEO_GPIO_BATTERY_CHARGER_CURRENT, "charger_current");
    gpio_request(HTCLEO_GPIO_BATTERY_CHARGER_ENABLE, "charger_enable");
    gpio_request(HTCLEO_GPIO_BATTERY_OVER_CHG, "charger_over_chg");

    gpio_direction_output(HTCLEO_GPIO_BATTERY_CHARGER_CURRENT, 1);
    gpio_direction_output(HTCLEO_GPIO_BATTERY_CHARGER_ENABLE, 1);
    gpio_direction_input(HTCLEO_GPIO_BATTERY_OVER_CHG);


    di->bat.name = "battery";
    di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
    di->bat.properties = battery_properties;
    di->bat.num_properties = ARRAY_SIZE(battery_properties);
    di->bat.external_power_changed = battery_ext_power_changed;
    di->bat.get_property = battery_get_property;
    di->last_charge_mode = 0xff;

    rc = power_supply_register(&client->dev, &di->bat);
    if (rc)
    {
        goto fail_register;
    }

    INIT_WORK(&di->monitor_work, htcleo_battery_work);
    di->monitor_wqueue = create_freezeable_workqueue(dev_name(&client->dev));

    /* init to something sane */
    di->last_poll = alarm_get_elapsed_realtime();

    if (!di->monitor_wqueue)
    {
        rc = -ESRCH;
        goto fail_workqueue;
    }
    wake_lock_init(&di->work_wake_lock, WAKE_LOCK_SUSPEND, "htcleo-battery");
    printk("alarm init\n");
    alarm_init(&di->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP, htcleo_battery_alarm);
    wake_lock(&di->work_wake_lock);
    queue_work(di->monitor_wqueue, &di->monitor_work);
    printk("probe exit!\n");
    return 0;

fail_workqueue:
    power_supply_unregister(&di->bat);
fail_register:


    gpio_free(HTCLEO_GPIO_BATTERY_CHARGER_CURRENT);
    gpio_free(HTCLEO_GPIO_BATTERY_CHARGER_ENABLE);
    gpio_free(HTCLEO_GPIO_BATTERY_OVER_CHG);

    kfree(di);
    return rc;
}

static int htcleo_suspend(struct device *dev)
{
    struct htcleo_device_info *di = dev_get_drvdata(dev);

    /* If we are on battery, reduce our update rate until
    * we next resume.
    */
    if (di->status.charge_source == SOURCE_NONE)
    {
        htcleo_program_alarm(di, SLOW_POLL);
        di->slow_poll = 1;
    }
    return 0;
}

static int htcleo_resume(struct device *dev)
{
    struct htcleo_device_info *di = dev_get_drvdata(dev);

    /* We might be on a slow sample cycle.  If we're
    * resuming we should resample the battery state
    * if it's been over a minute since we last did
    * so, and move back to sampling every minute until
    * we suspend again.
    */
    if (di->slow_poll)
    {
        htcleo_program_alarm(di, FAST_POLL);
        di->slow_poll = 0;
    }
    return 0;
}

static struct dev_pm_ops htcleo_pm_ops =
{
    .suspend	= htcleo_suspend,
    .resume	= htcleo_resume,
};


static const struct i2c_device_id htcleo_battery_id[] =
{
    { "htcleo-battery", 0 },
    { }
};


static struct i2c_driver htcleo_battery_driver =
{
    .driver =
    {
        .name = "htcleo-battery",
        .owner = THIS_MODULE,
        .pm = &htcleo_pm_ops,
    },
    .id_table   = htcleo_battery_id,
    .probe = htcleo_battery_probe,
#ifndef CONFIG_HAS_EARLYSUSPEND
 /*   .suspend = htcleo_suspend,
    .resume = htcleo_resume,*/
#endif
};

static int battery_log_open(struct inode *inode, struct file *file)
{
    return single_open(file, battery_log_print, NULL);
}

static struct file_operations battery_log_fops =
{
    .open = battery_log_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static int __init htcleo_battery_init(void)
{
    debugfs_create_file("battery_log", 0444, NULL, NULL, &battery_log_fops);
    wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
    return i2c_add_driver(&htcleo_battery_driver);
}

//module_init(htcleo_battery_init);
late_initcall(htcleo_battery_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cotulla");
MODULE_DESCRIPTION("htcleo battery driver");
// END OF FILE
