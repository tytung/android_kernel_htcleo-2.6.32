#ifndef __LEO_BATTERY__
#define __LEO_BATTERY__

#include <linux/android_alarm.h>

#define LEO_BATTERY_CAPACITY    1230
#define LEO_BATTERY_EMPTY       500

// from board-htcleo-power.c
extern int is_ac_power_supplied(void);

// from board-htcleo-log.c
extern double log(double x);

#define RAW_DATA_SIZE       10


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

#define TEMP_CRITICAL	600 /* no charging at all */
#define TEMP_HOT	500 /* no fast charge, no charge > 4.1v */
#define TEMP_WARM	450 /* no fast charge above this */

#define TEMP_HOT_MAX_MV	4100 /* stop charging here when hot */
#define TEMP_HOT_MIN_MV	3800 /* resume charging here when hot */
#define CE_DISABLE_MIN_MV 4100


/* When we're awake or running on wall power, sample the battery
* gauge every FAST_POLL seconds.  If we're asleep and on battery
* power, sample every SLOW_POLL seconds
*/
#define FAST_POLL	(1 * 60)
#define SLOW_POLL	(10 * 60)


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

static struct htcleo_device_info * mydi = NULL;

#define psy_to_dev_info(x) container_of((x), struct htcleo_device_info, bat)

static struct wake_lock vbus_wake_lock;


#define to_htcleo_device_info(x) container_of((x), struct htcleo_device_info, bat);

#endif

