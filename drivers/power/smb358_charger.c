/****************************************************
description:smb358 charger driver
author:jiangsm jsm@oppo.com jsmssb@163.com
author:shijianchao@oppo.com
date:2013-6-9
*****************************************************/
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include "smb358_regs.h"
#include <linux/power/smb358_charger.h>
#include <linux/pcb_version.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/pcb_version.h>

/*macores defined begin*/
#define IRQ_POWER_SUPPLY_UPDATE_DELAY	50//ms
#define CHG_POWER_UPPLY_UPDATE_DELAY 	50//ms
#define POWER_SUPPLY_UPDATE_PERIOD		3000//5000//ms
#define START_CHG_DELAY					500//1000//ms

#define BAT_TEMP__INVALID			-273
#define BAT_TEMP__TEN_BELOW_ZERO	-10
#define BAT_TEMP__ZERO				0
#define BAT_TEMP__TEN				10
#define BAT_TEMP__FORTY_FIVE		45
#define BAT_TEMP__FIFTY_FIVE		55
#define BAT_TEMP__BOUNCE			3

/*initial charger and battery data*/
#define CHARGER_VOL__DEFAULT		5000
#define BAT_VOL__DEFAULT			3800//MV
#define BAT_TEMP__DEFAULT			25
#define BAT_TEMP_REGION__DEFAULT	BATTERY_TEMP_REGION__NORMAL
#define BAT_CAPACITY__DEFAULT		30

/*charging current set*/
#define STANDARD__PRE_CHG__LITTLE_COLD_CURRNET		PRE_CHG_CURRENT__450MA
#define STANDARD__PRE_CHG__COOL_CURRNET				PRE_CHG_CURRENT__450MA
#define STANDARD__PRE_CHG__NORMAL_CURRNET			PRE_CHG_CURRENT__450MA
#define STANDARD__PRE_CHG__WARM_CURRNET			PRE_CHG_CURRENT__450MA
#define STANDARD__FAST_CHG__LITTLE_COLD_CURRNET		FAST_CHG_CURRENT__200MA
#define STANDARD__FAST_CHG__COOL_CURRNET			FAST_CHG_CURRENT__900MA
#define STANDARD__FAST_CHG__NORMAL_CURRNET			FAST_CHG_CURRENT__2000MA
#define STANDARD__FAST_CHG__NORMAL_CURRNET_AICL	FAST_CHG_CURRENT__1800MA//sjc1003 for current aicl
#define STANDARD__FAST_CHG__WARM_CURRNET			FAST_CHG_CURRENT__900MA
#define STANDARD__TAPER_CHG__LITTLE_COLD_CURRNET	FAST_CHG_CURRENT__1800MA//sjc0927//FAST_CHG_CURRENT__2000MA
#define STANDARD__TAPER_CHG__COOL_CURRNET			FAST_CHG_CURRENT__1800MA//sjc0927//FAST_CHG_CURRENT__2000MA
#define STANDARD__TAPER_CHG__NORMAL_CURRNET		FAST_CHG_CURRENT__1800MA//sjc0927//FAST_CHG_CURRENT__2000MA
#define STANDARD__TAPER_CHG__WARM_CURRNET			FAST_CHG_CURRENT__1800MA//sjc0927//FAST_CHG_CURRENT__2000MA

#define NON_STANDARD__PRE_CHG__LITTLE_COLD_CURRNET		PRE_CHG_CURRENT__450MA
#define NON_STANDARD__PRE_CHG__COOL_CURRNET			PRE_CHG_CURRENT__450MA
#define NON_STANDARD__PRE_CHG__NORMAL_CURRNET			PRE_CHG_CURRENT__450MA
#define NON_STANDARD__PRE_CHG__WARM_CURRNET			PRE_CHG_CURRENT__450MA
#define NON_STANDARD__FAST_CHG__LITTLE_COLD_CURRNET	FAST_CHG_CURRENT__200MA
#define NON_STANDARD__FAST_CHG__COOL_CURRNET			FAST_CHG_CURRENT__600MA
#define NON_STANDARD__FAST_CHG__NORMAL_CURRNET		FAST_CHG_CURRENT__600MA
#define NON_STANDARD__FAST_CHG__WARM_CURRNET			FAST_CHG_CURRENT__600MA
#define NON_STANDARD__TAPER_CHG__LITTLE_COLD_CURRNET	FAST_CHG_CURRENT__600MA
#define NON_STANDARD__TAPER_CHG__COOL_CURRNET			FAST_CHG_CURRENT__600MA
#define NON_STANDARD__TAPER_CHG__NORMAL_CURRNET		FAST_CHG_CURRENT__600MA
#define NON_STANDARD__TAPER_CHG__WARM_CURRNET			FAST_CHG_CURRENT__600MA

#define USB__PRE_CHG__LITTLE_COLD_CURRNET		PRE_CHG_CURRENT__450MA
#define USB__PRE_CHG__COOL_CURRNET				PRE_CHG_CURRENT__450MA
#define USB__PRE_CHG__NORMAL_CURRNET			PRE_CHG_CURRENT__450MA
#define USB__PRE_CHG__WARM_CURRNET				PRE_CHG_CURRENT__450MA
#define USB__FAST_CHG__LITTLE_COLD_CURRNET		FAST_CHG_CURRENT__200MA
#define USB__FAST_CHG__COOL_CURRNET				FAST_CHG_CURRENT__600MA
#define USB__FAST_CHG__NORMAL_CURRNET			FAST_CHG_CURRENT__600MA
#define USB__FAST_CHG__WARM_CURRNET				FAST_CHG_CURRENT__600MA
#define USB__TAPER_CHG__LITTLE_COLD_CURRNET		FAST_CHG_CURRENT__600MA
#define USB__TAPER_CHG__COOL_CURRNET				FAST_CHG_CURRENT__600MA
#define USB__TAPER_CHG__NORMAL_CURRNET			FAST_CHG_CURRENT__600MA
#define USB__TAPER_CHG__WARM_CURRNET			FAST_CHG_CURRENT__600MA

#define MAX_INPUT_CURRENT_LIMIT__STANDARD		MAX_CHG_CURRENT__1800MA//sjc0927//MAX_CHG_CURRENT__2000MA
#define MAX_INPUT_CURRENT_LIMIT__STANDARD_SUS	MAX_CHG_CURRENT__1500MA//sjc0927
#define MAX_INPUT_CURRENT_LIMIT__NON_STANDARD	MAX_CHG_CURRENT__500MA
#define MAX_INPUT_CURRENT_LIMIT__USB				MAX_CHG_CURRENT__500MA

/*irq flags bgein*/
#define IRQ_FLAGS__BAT_MISSING				1
#define IRQ_FLAGS__CHG_COMPLETE			2
#define IRQ_FLAGS__CHG_RESUME				3
#define IRQ_FLAGS__NO_CHG					4
#define IRQ_FLAGS__CHG_MODE_PRE			5
#define IRQ_FLAGS__CHG_MODE_FAST			6
#define IRQ_FLAGS__CHG_MODE_TAPER		7
#define IRQ_FLAGS__CHG_TYPE__INVALID		8
#define IRQ_FLAGS__CHG_TYPE__SDP			9
#define IRQ_FLAGS__CHG_TYPE__DCP			10
#define IRQ_FLAGS__CHG_TYPE__NON_DCP		11
#define IRQ_FLAGS__CHG_TIMEOUT			12//sjc0820
/*irq flags end*/

/*charging complete  begin*/
#define CHG_COMPLETE_CHECK_COUNT			6//12
#define CHG_COMPLETE_CHECK_COUNT_MAX	20
#define CHG_COMPLETE_CHECK__CURRENT		100//ma

#define CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__LITTLE_COLD		4000//mv
#define CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__COOL			4300//4200//mv
#define CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__NORMAL			4350//mv
#define CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__WARM			4100//mv

#define CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__LITTLE_COLD	4000//mv
#define CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__COOL			4300//4200//mv
#define CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__NORMAL		4350//mv
#define CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__WARM			4100//mv

#define CHG_COMPLETE_CHECK_VOLTAGE__USB__LITTLE_COLD			4000//mv
#define CHG_COMPLETE_CHECK_VOLTAGE__USB__COOL					4300//4200//mv
#define CHG_COMPLETE_CHECK_VOLTAGE__USB__NORMAL				4350//mv
#define CHG_COMPLETE_CHECK_VOLTAGE__USB__WARM					4100//mv
/*charging complete end*/

/*chargign resume begin*/
#define CHG_RESUME_CHECK_COUNT	12

#define CHG_RESUME_CHECK_VOLTAGE__STANDARD__LITTLE_COLD		3800//mv
#define CHG_RESUME_CHECK_VOLTAGE__STANDARD__COOL				4100//4000//mv
#define CHG_RESUME_CHECK_VOLTAGE__STANDARD__NORMAL			4200//mv
#define CHG_RESUME_CHECK_VOLTAGE__STANDARD__WARM				3900//mv

#define CHG_RESUME_CHECK_VOLTAGE__NON_STANDARD__LITTLE_COLD	3800//mv
#define CHG_RESUME_CHECK_VOLTAGE__NON_STANDARD__COOL			4100//4000//mv
#define CHG_RESUME_CHECK_VOLTAGE__NON_STANDARD__NORMAL		4200//mv
#define CHG_RESUME_CHECK_VOLTAGE__NON_STANDARD__WARM		3900//mv

#define CHG_RESUME_CHECK_VOLTAGE__USB__LITTLE_COLD				3800//mv
#define CHG_RESUME_CHECK_VOLTAGE__USB__COOL					4100//4000//mv
#define CHG_RESUME_CHECK_VOLTAGE__USB__NORMAL					4200//mv
#define CHG_RESUME_CHECK_VOLTAGE__USB__WARM					3900//mv
/*charging resume end*/

/*charger and battery u/ovp begin*/
#define CHARGER_OVP_CHECK_COUNT	3
#define CHARGER_UVP_CHECK_COUNT	3	
#define CHARGER_OVP_VOLTAGE		5800//6800//mv
#define CHARGER_UVP_VOLTAGE		4300//4500//mv
#define CHARGER_VOLTAGE_READ_TIMES	2

#define BATTERY_OVP_CHECK_COUNT	3
#define BATTERY_OVP_VOLTAGE		4500//mv
#define BATTERY_UVP_VOLTAGE		3000//mv

#define BAT_MISSING_CHECK_COUNT 	3
#define BAT_MISSING_VOLTAGE		2500//2000//mv

#define BAT_MAX_DESIGNED_VOLTAGE 	4500
#define BAT_MIN_DESIGNED_VOLTAGE	2000//mv
/*charger and battery u/ovp end*/

#define CHG_CURRENT_AICL_COUNT	3//sjc1003 for current aicl
#define CHG_CURRENT_AICL_VOL		4000//mv sjc1003 for current aicl

/*charging timeout*/
#define CHG_TIME_OUT
#define SECONDS_PER_MINUTE 		60;
#define MAX_CHARGING_TIMEOUT_DCP	6*60*60//6 hours
#define MAX_CHARGING_TIMEOUT_SDP	10*60*60//sjc1004 8->10//8 hours
/*macores defined end*/

typedef struct {
   s32 x;
   s32 y;
} voltage2capacity;

static const voltage2capacity v2c_map[] =
{
   //13005 provide by HW Team 2012-07-25
   { 3400,     0 },	//@0	
   { 3500,     1 },	//@1
   { 3571,     5 },	//@2	
   { 3596,     9 },	//@3	
   { 3622,    13 },	//@4
   { 3640,    17 },	//@5 -- red
   { 3653,    21 },	//@6 -- green
   { 3664,    25 },	//@7
   { 3677,    29 },	//@8
   { 3689,    33 },	//@9
   { 3702,    37 },	//@10	
   { 3716,    41 },	//@11
   { 3731,    45 },	//@12	
   { 3748,    49 },	//@13
   { 3766,    53 },	//@14
   { 3787,    57 },	//@15
   { 3813,    61 },	//@16
   { 3839,    65 },	//@17
   { 3873,    69 },	//@18
   { 3911,    73 },	//@19
   { 3949,    77 },	//@20
   { 3990,    81 },	//@21
   { 4032,    85 },	//@22	
   { 4074,    89 },	//@23
   { 4117,    93 },	//@24
   { 4163,    97 },	//@25
   { 4176,    98 },  //@25
   { 4190,    99 },  //@25
   { 4200,   100 }  //FULL
};
static int get_charger_voltage(void);
static int get_battery_capacity(void);
static int get_battery_voltage(void);
static int get_battery_temperature(void);
static int get_battery_soh(void);
static int get_battery_fcc(void);
static int get_charging_current(void);
static int set_battery_capacity_full(void);

static smb358_charger_type startup_charger_type = CHARGER_TYPE__INVALID;
static char *charger_type_str[] = {"INVALID", "SDP(usb charger)", "DCP(standard charger)", 
	"NON_DCP(non-standard)", "HDMI","CDP", "ACA", "OTG"
};
static char *charger_status_str[] = {"invalid", "good", "uvp", "ovp"};
static char *charging_status_str[] = {"invalid", "charging", "discharging", "not charging", "full"};
static char *charging_mode_str[] = {"not charging", "trick-charging","pre-charging", "fast-charging", "taper-charging"};
//static char *battery_status_str[] = {"invalid", "good", "uvp", "ovp"};//sjc0806
static char *battery_status_str[] = {"invalid", "good", "overheat", "dead", "ovp", "unspec_failure", "cold", "uvp"};
static char *battery_temp_region_str[] = {"invalid", "cold", "little cold", "cool", "normal", "warm", "hot"};
static char *battery_missing_status_str[] = {"invalid", "missed", "good"};

static int current_aicl_counts = 0;//sjc1003 for current aicl
static int ovp_counts = 0;//sjc0806
static int uvp_counts = 0;
static bool in_ovp_status = false;
static bool in_uvp_status = false;
static struct smb358_charger *the_smb358_charger = NULL;
static struct oppo_battery_fuelgauge *batt_fuelgauge = NULL;

extern struct mutex i2c_bus_mutex;//sjc1024 for GSBI1_I2C err
extern int get_boot_mode(void);//sjc0823 for FTM/RF/WLAN

enum {
	MSM_BOOT_MODE__NORMAL,
	MSM_BOOT_MODE__FASTBOOT,
	MSM_BOOT_MODE__RECOVERY,
	MSM_BOOT_MODE__FACTORY,
	MSM_BOOT_MODE__RF,
	MSM_BOOT_MODE__WLAN,
	MSM_BOOT_MODE__CHARGE,
};

static bool is_ftm_mode(void)
{
	int boot_mode = 0;
	boot_mode = get_boot_mode();
	if (boot_mode == MSM_BOOT_MODE__FACTORY || boot_mode == MSM_BOOT_MODE__RF
			|| boot_mode == MSM_BOOT_MODE__WLAN)
		return true;
	else
		return false;
}

/*for debug info begin*/
static bool smb358_debug_enabled_get(struct smb358_charger * smb358_chg);
static void smb358_debug_enabled_set(struct smb358_charger * smb358_chg, bool val);
static smb358_charger_type smb358_debug_charger_type_get(struct smb358_charger * smb358_chg);
static void smb358_debug_charger_type_set(struct smb358_charger * smb358_chg, smb358_charger_type chg_type);
static int smb358_debug_charger_voltage_get(struct smb358_charger * smb358_chg);
static void smb358_debug_charger_voltage_set(struct smb358_charger * smb358_chg, int vol);
static int smb358_debug_battery_voltage_get(struct smb358_charger * smb358_chg);
static void smb358_debug_battery_voltage_set(struct smb358_charger * smb358_chg, int vol);
static int smb358_debug_battery_temperature_get(struct smb358_charger * smb358_chg);
static void smb358_debug_battery_temperature_set(struct smb358_charger * smb358_chg, int vol);
static int smb358_debug_battery_capacity_get(struct smb358_charger * smb358_chg);
static void smb358_debug_battery_capacity_set(struct smb358_charger * smb358_chg, int vol);
static int smb358_debug_charging_current_get(struct smb358_charger * smb358_chg);
static void smb358_debug_charger_type_set(struct smb358_charger * smb358_chg, smb358_charger_type chg_type);
static bool smb358_debug_printk_enabled_get(struct smb358_charger * smb358_chg);
static void smb358_debug_printk_enabled_set(struct smb358_charger * smb358_chg, bool val);
static bool smb358_debug_dump_registers_enabled_get(struct smb358_charger * smb358_chg);
static void smb358_debug_dump_registers_enabled_set(struct smb358_charger * smb358_chg, bool val);
static bool printk_enabled(void)
{
	if (the_smb358_charger)
		return smb358_debug_printk_enabled_get(the_smb358_charger);
	else
		return false;
}
#define CHG_DBG(fmt, ...) \
	if (printk_enabled()) \
		printk(KERN_ERR  pr_fmt(fmt), ##__VA_ARGS__); 
#define CHG_ERR(fmt, ...) printk(KERN_ERR  pr_fmt(fmt), ##__VA_ARGS__)
/*for debug info end*/

/*function declare begin*/
static bool smb358_hardware_init(struct smb358_charger * smb358_chg);
static smb358_charger_type smb358_charger_type_get(struct smb358_charger *smb358_chg);
static void smb358_charger_type_set(struct smb358_charger * smb358_chg, smb358_charger_type chg_type);
static smb358_charging_mode smb358_charging_mode_get(struct smb358_charger *smb358_chg);
static void smb358_charging_mode_set(struct smb358_charger * smb358_chg, smb358_charging_mode chg_mode);
static bool smb358_suspended_get(struct smb358_charger * smb358_chg);
static void smb358_suspend_charger(struct smb358_charger *smb358_chg);
static void smb358_unsuspend_charger(struct smb358_charger *smb358_chg);
static void smb358_suspended_set(struct smb358_charger * smb358_chg, bool value);
static bool smb358_charging_current_set(struct smb358_charger * smb358_chg);
static int smb358_charging_current_get(struct smb358_charger * smb358_chg);
static int smb358_charger_voltage_get(struct smb358_charger *smb358_chg);
static void smb358_charger_voltage_set(struct smb358_charger *smb358_chg, int vol);
static smb358_charger_status smb358_charger_status_get(struct smb358_charger * smb358_chg);
static void smb358_charger_status_set(struct smb358_charger *smb358_chg, smb358_charger_status  chg_stat);
static int smb358_battery_voltage_get(struct smb358_charger *smb358_chg);
static void smb358_battery_voltage_set(struct smb358_charger *smb358_chg, int vol);
static void smb358_battery_missing_status_set(struct smb358_charger *smb358_chg, smb358_battery_missing_status bat_missing_stat);
static smb358_battery_missing_status smb358_battery_missing_status_get(struct smb358_charger *smb358_chg);
static smb358_battery_status smb358_battery_status_get(struct smb358_charger * smb358_chg);
static void smb358_battery_status_set(struct smb358_charger *smb358_chg, smb358_battery_status bat_stat);
static int smb358_battery_temperature_get(struct smb358_charger *smb358_chg);
static void smb358_battery_temperature_set(struct smb358_charger *smb358_chg, int bat_temp);
static smb358_battery_temperature_region smb358_battery_temperature_region_get(struct smb358_charger * smb358_chg);
static void smb358_battery_temperature_region_set(struct smb358_charger * smb358_chg, smb358_battery_temperature_region bat_temp_region);
static int smb358_cold_to_little_cold_critical_temperature_get(struct smb358_charger * smb358_chg);
static void smb358_cold_to_little_cold_critical_temperature_set(struct smb358_charger * smb358_chg, int val);
static int smb358_little_cold_to_cool_critical_temperature_get(struct smb358_charger * smb358_chg);
static void smb358_little_cold_to_cool_critical_temperature_set(struct smb358_charger * smb358_chg, int val);
static int smb358_cool_to_normal_critical_temperature_get(struct smb358_charger * smb358_chg);
static void smb358_cool_to_normal_critical_temperature_set(struct smb358_charger * smb358_chg, int val);
static int smb358_normal_to_warm_critical_temperature_get(struct smb358_charger * smb358_chg);
static void smb358_normal_to_warm_critical_temperature_set(struct smb358_charger * smb358_chg, int val);
static int smb358_warm_to_hot_critical_temperature_get(struct smb358_charger * smb358_chg);
static void smb358_warm_to_hot_critical_temperature_set(struct smb358_charger * smb358_chg, int val);
static int smb358_battery_capacity_get(struct smb358_charger *smb358_chg);
static void smb358_battery_capacity_set(struct smb358_charger *smb358_chg, int bat_cap);
static bool smb358_charging_complete_request_get(struct smb358_charger *smb358_chg);
static void smb358_charging_complete_request_set(struct smb358_charger *smb358_chg, bool value);
static bool smb358_charging_resume_request_get(struct smb358_charger *smb358_chg);
static void smb358_charging_resume_request_set(struct smb358_charger *smb358_chg, bool value);
static smb358_charging_status smb358_charging_status_get(struct smb358_charger *smb358_chg);
static void smb358_charging_status_set(struct smb358_charger *smb358_chg, smb358_charging_status chg_stat);
static void smb358_bit_set(struct smb358_charger *smb358_chg, int bit);
static bool smb358_bit_test(struct smb358_charger *smb358_chg, int bit);
static void smb358_bit_clear(struct smb358_charger *smb358_chg, int bit);
static void smb358_power_supply_update(struct smb358_charger * smb358_chg);
static bool smb358_debug_charging_timeout_get(struct smb358_charger *smb358_chg);
static void smb358_debug_charging_timeout_set(struct smb358_charger * smb358_chg, bool val);
static bool smb358_charging_timeout_get(struct smb358_charger * smb358_chg);
static void smb358_charging_timeout_set(struct smb358_charger * smb358_chg, bool val);
static bool smb358_float_voltage_write(struct smb358_charger *smb358_chg, int flt_vol);
static bool smb358_otg_status_get(struct smb358_charger *smb358_chg);//sjc0808otg
static void smb358_otg_status_set(struct smb358_charger *smb358_chg, bool value);
static void smb358_otg_enable(struct smb358_charger *smb358_chg);
static void smb358_otg_disable(struct smb358_charger *smb358_chg);
static void smb358_charger_voltage_handle(struct smb358_charger *smb358_chg);
static bool smb358_input_current_limit_standard_set(struct smb358_charger *smb358_chg);
static void smb358_charging_timeout_status_set(struct smb358_charger *smb358_chg, bool val);//sjc0820
static bool smb358_charging_timeout_status_get(struct smb358_charger *smb358_ch);
static void smb358_start_chg_timeout_timer(struct smb358_charger *smb358_chg);
static void smb358_stop_chg_timeout_timer(struct smb358_charger *smb358_chg);
static void smb358_aicl_status_set(struct smb358_charger *smb358_chg, bool val);//sjc0824
static bool smb358_aicl_status_get(struct smb358_charger *smb358_chg);
static void smb358_aicl_handle(struct smb358_charger *smb358_chg);
static int smb358_battery_soh_get(struct smb358_charger *smb358_chg);//sjc0826
static void smb358_battery_soh_set(struct smb358_charger *smb358_chg, int val);
static int smb358_battery_fcc_get(struct smb358_charger *smb358_chg);
static void smb358_battery_fcc_set(struct smb358_charger *smb358_chg, int val);
static bool smb358_early_suspend_status_get(struct smb358_charger *smb358_chg);//sjc0927
static void smb358_charging_current_aicl_status_set(struct smb358_charger *smb358_chg, bool val);//sjc1003
static bool smb358_charging_current_aicl_status_get(struct smb358_charger *smb358_chg);
static void smb358_charging_current_aicl_handle(struct smb358_charger *smb358_chg);
/*function declare begin*/

/*for i2c data read write funcs begin*/
static void smb358_bit_set(struct smb358_charger *smb358_chg, int bit)
{
	set_bit(bit, &smb358_chg->irq_flags);
}

static bool smb358_bit_test(struct smb358_charger *smb358_chg, int bit)
{
	return test_bit(bit, &smb358_chg->irq_flags);
}

static void smb358_bit_clear(struct smb358_charger *smb358_chg, int bit)
{
	clear_bit(bit, &smb358_chg->irq_flags);
}

static void smb358_reschedule_update_work(struct smb358_charger *smb358_chg)
{
	queue_delayed_work(smb358_chg->work_queue, &smb358_chg->update_work, 0);
}

static bool smb358_register_read(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;
	struct smb358_charger *smb358_chg;

	smb358_chg = i2c_get_clientdata(client);
	mutex_lock(&i2c_bus_mutex);//sjc1024 for GSBI1_I2C err
	ret = i2c_smbus_read_byte_data(smb358_chg->client, reg);
	mutex_unlock(&i2c_bus_mutex);//sjc1024 for GSBI1_I2C err
	if (ret < 0) {
		CHG_ERR("i2c read fail: can't read from reg addr=%02x: error=%d\n", reg, ret);
		return false;
	} else {
		*val = ret;
	}
	return true;
}

static bool smb358_register_write(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;
	struct smb358_charger *smb358_chg;

	smb358_chg = i2c_get_clientdata(client);
	mutex_lock(&i2c_bus_mutex);//sjc1024 for GSBI1_I2C err
	ret = i2c_smbus_write_byte_data(smb358_chg->client, reg, val);
	mutex_unlock(&i2c_bus_mutex);//sjc1024 for GSBI1_I2C err
	if (ret < 0) {
		CHG_ERR("i2c write fail: can't write val=%2x to reg addr=%2x: error=%d\n", val, reg, ret);
		return false;
	}
	return true;
}

static bool smb358_register_masked_write(struct i2c_client *client, int reg, u8 mask, u8 val)
{
	bool ret = false;
	u8 temp;
	ret = smb358_register_read(client, reg, &temp);
	if (ret){//read success
		temp &= ~mask;
		temp |= val & mask;
		ret = smb358_register_write(client, reg, temp);
		if (!ret ) {//write failed
			CHG_ERR("smb358_register_write failed: reg=%03X ,value to be write=%x\n", reg, temp);
		}
	}
	return ret;
}
/*for i2c data read write funcs end*/

static bool smb358_enable_charging(struct smb358_charger *smb358_chg)
{
	int ret = -1;
	
	ret = smb358_register_masked_write(smb358_chg->client, CMD_REG_A_REGISTER_ADDR, CHG_CONTROL__MASK, CHG_CONTROL__ENABLE);
	if (ret)
		CHG_ERR("%s:enable charging success\n", __func__);
	else
		CHG_ERR("%s:enable charging failed\n", __func__);
	return ret;		
}

static bool smb358_disable_charging(struct smb358_charger *smb358_chg)
{
	int ret = -1;
	
	ret = smb358_register_masked_write(smb358_chg->client, CMD_REG_A_REGISTER_ADDR, CHG_CONTROL__MASK, CHG_CONTROL__DISABLE);
	if (ret)
		CHG_ERR("%s:disable charging success\n", __func__);
	else
		CHG_ERR("%s:disable charging failed\n", __func__);
	return ret;		
}

static bool smb358_charging_current_write_pre(struct smb358_charger *smb358_chg, int chg_cur)
{
	int ret = -1;
	u8 value = chg_cur &0xff;
	ret = smb358_register_masked_write(smb358_chg->client, CHG_CURRENT_REGISTER_ADDR, CHG_CURRENT__PRE_CHG_MASK, value);
	if (false == ret)
		CHG_ERR("%s:set pre-charging current failed\n", __func__);
	return ret;		
}

static bool smb358_charging_current_write_fast(struct smb358_charger *smb358_chg, int chg_cur)
{
	int ret = -1;
	u8 value = chg_cur &0xff;
	ret = smb358_register_masked_write(smb358_chg->client, CHG_CURRENT_REGISTER_ADDR, CHG_CURRENT__FAST_CHG_MASK, value);
	if (false == ret)
		CHG_ERR("%s:set fast-charging current failed\n", __func__);
	return ret;		
}

static bool smb358_charging_current_write_taper(struct smb358_charger *smb358_chg, int chg_cur)
{
	int ret = -1;
	u8 value = chg_cur &0xff;
	ret = smb358_register_masked_write(smb358_chg->client, CHG_CURRENT_REGISTER_ADDR, CHG_CURRENT__FAST_CHG_MASK, value);
	if (false == ret)
		CHG_ERR("%s:set taper-charging current failed\n", __func__);
	return ret;		
}

static bool smb358_charging_current_write(struct smb358_charger *smb358_chg, int chg_cur)
{
	smb358_charging_mode chg_mode = smb358_charging_mode_get(smb358_chg);
	if (CHARGING_MODE__PRE_CHG == chg_mode)
		return smb358_charging_current_write_pre(smb358_chg, chg_cur);
	else if(chg_mode == CHARGING_MODE__FAST_CHG )
		return smb358_charging_current_write_fast(smb358_chg, chg_cur);
	else if (chg_mode == CHARGING_MODE__TAPER_CHG){
		return smb358_charging_current_write_taper(smb358_chg, chg_cur);
	}else{
		CHG_DBG("%s:not charging\n", __func__);
		return true;
	}
}

static bool smb358_is_charger_in(struct smb358_charger *smb358_chg)//sjc1023
{
	if (smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__SDP || smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__DCP
			|| smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__NON_DCP)
		return true;
	else
		return false;
}

static bool smb358_start_charging(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	int batt_vol;
	int batt_temp;
	
	if (CHARGER_TYPE__INVALID == smb358_charger_type_get(smb358_chg) || CHARGER_TYPE__OTG == smb358_charger_type_get(smb358_chg))//sjc1014
		return false;
	if (is_ftm_mode()) {//sjc0823
		smb358_suspend_charger(smb358_chg);
		smb358_suspended_set(smb358_chg, false);//do not use it in FTM
		return false;
	}
/*if charger suspended do nothing*/
	if (true == smb358_suspended_get(smb358_chg)){
		CHG_ERR("%s:in suspend mode ,no charging !\n", __func__);
		return false;
	}
	smb358_hardware_init(smb358_chg);

	batt_vol = smb358_battery_voltage_get(smb358_chg);
	batt_temp = smb358_battery_temperature_get(smb358_chg);
	
	if (batt_temp < BAT_TEMP__TEN_BELOW_ZERO || batt_temp > BAT_TEMP__FIFTY_FIVE) {
		power_supply_changed(smb358_chg->power_supplies.battery);//sjc8010
		return false;
	}
	else if (batt_temp < BAT_TEMP__ZERO) {
		if (batt_vol < CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__LITTLE_COLD)
			smb358_charging_mode_set(smb358_chg, CHARGING_MODE__FAST_CHG);
		else
			smb358_charging_mode_set(smb358_chg, CHARGING_MODE__TAPER_CHG);
	}
	else if (batt_temp < BAT_TEMP__TEN) {
		if (batt_vol < CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__COOL)
			smb358_charging_mode_set(smb358_chg, CHARGING_MODE__FAST_CHG);
		else
			smb358_charging_mode_set(smb358_chg, CHARGING_MODE__TAPER_CHG);
	}
	else if (batt_temp < BAT_TEMP__FORTY_FIVE) {
		if (batt_vol < CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__NORMAL)
			smb358_charging_mode_set(smb358_chg, CHARGING_MODE__FAST_CHG);
		else
			smb358_charging_mode_set(smb358_chg, CHARGING_MODE__TAPER_CHG);
	}
	else if (batt_temp < BAT_TEMP__FIFTY_FIVE) {
		if (batt_vol < CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__WARM)
			smb358_charging_mode_set(smb358_chg, CHARGING_MODE__FAST_CHG);
		else
			smb358_charging_mode_set(smb358_chg, CHARGING_MODE__TAPER_CHG);
	}

	if (smb358_charging_current_set(smb358_chg))
	{
		ret = smb358_enable_charging(smb358_chg);
		if (ret) {
			if (true != smb358_charging_resume_request_get(smb358_chg))//sjc0813 resume charging status
				smb358_charging_status_set(smb358_chg, CHARGING_STATUS__CHARGING);
			smb358_reschedule_update_work(smb358_chg);
			return ret;
		} else {
			smb358_charging_mode_set(smb358_chg, CHARGING_MODE__INVALID);
		}
	}
	return ret;
}

static bool smb358_stop_charging(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	ret = smb358_disable_charging(smb358_chg);
	if (ret){
		smb358_charging_timeout_set(smb358_chg, false);
		smb358_charging_status_set(smb358_chg, CHARGING_STATUS__NOT_CHARGING);
		if (CHARGER_TYPE__INVALID == smb358_charger_type_get(smb358_chg))
			smb358_charging_status_set(smb358_chg, CHARGING_STATUS__DISCHARGING);
		smb358_reschedule_update_work(smb358_chg);
	}
	return ret;
}

static bool smb358_charging_current_set__standard_trick(struct smb358_charger *smb358_chg)
{
	CHG_ERR("%s:trick charging no need to do anything for software \n", __func__);
	return true;
}

static bool smb358_charging_current_set__standard_pre_litte_cold(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, STANDARD__PRE_CHG__LITTLE_COLD_CURRNET);
}

static bool smb358_charging_current_set__standard_pre_cool(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, STANDARD__PRE_CHG__COOL_CURRNET);
}

static bool smb358_charging_current_set__standard_pre_normal(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, STANDARD__PRE_CHG__NORMAL_CURRNET);
}

static bool smb358_charging_current_set__standard_pre_warm(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, STANDARD__PRE_CHG__WARM_CURRNET);
}

static bool smb358_charging_current_set__standard_pre(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__INVALID:
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_charging_current_set__standard_pre_litte_cold(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_charging_current_set__standard_pre_cool(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_charging_current_set__standard_pre_normal(smb358_chg);		
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_charging_current_set__standard_pre_warm(smb358_chg);		
			break;
		default:
			break;
	}
	return ret;
}

static bool smb358_charging_current_set__standard_fast_litte_cold(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, STANDARD__FAST_CHG__LITTLE_COLD_CURRNET);
}

static bool smb358_charging_current_set__standard_fast_cool(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, STANDARD__FAST_CHG__COOL_CURRNET);
}

static bool smb358_charging_current_set__standard_fast_normal(struct smb358_charger *smb358_chg)
{
	if (smb358_battery_voltage_get(smb358_chg) > CHG_CURRENT_AICL_VOL) {//sjc1003
		smb358_charging_current_aicl_status_set(smb358_chg, true);
		return smb358_charging_current_write(smb358_chg, STANDARD__FAST_CHG__NORMAL_CURRNET_AICL);
	}
	return smb358_charging_current_write(smb358_chg, STANDARD__FAST_CHG__NORMAL_CURRNET);
}

static bool smb358_charging_current_set__standard_fast_warm(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, STANDARD__FAST_CHG__WARM_CURRNET);
}


static bool smb358_charging_current_set__standard_fast(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__INVALID:
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_charging_current_set__standard_fast_litte_cold(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_charging_current_set__standard_fast_cool(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_charging_current_set__standard_fast_normal(smb358_chg);		
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_charging_current_set__standard_fast_warm(smb358_chg);		
			break;
		default:
			break;
	}
	return ret;
}

static bool smb358_charging_current_set__standard_taper_litte_cold(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, STANDARD__TAPER_CHG__LITTLE_COLD_CURRNET);
}

static bool smb358_charging_current_set__standard_taper_cool(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, STANDARD__TAPER_CHG__COOL_CURRNET);
}

static bool smb358_charging_current_set__standard_taper_normal(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, STANDARD__TAPER_CHG__NORMAL_CURRNET);
}

static bool smb358_charging_current_set__standard_taper_warm(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, STANDARD__TAPER_CHG__WARM_CURRNET);
}

static bool smb358_charging_current_set__standard_taper(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__INVALID:
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_charging_current_set__standard_taper_litte_cold(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_charging_current_set__standard_taper_cool(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_charging_current_set__standard_taper_normal(smb358_chg);		
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_charging_current_set__standard_taper_warm(smb358_chg);		
			break;
		default:
			break;
	}
	return ret;

};

static bool smb358_charging_current_set__non_standard_pre_litte_cold(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__PRE_CHG__LITTLE_COLD_CURRNET);
}

static bool smb358_charging_current_set__non_standard_pre_cool(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__PRE_CHG__COOL_CURRNET);
}

static bool smb358_charging_current_set__non_standard_pre_normal(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__PRE_CHG__NORMAL_CURRNET);
}

static bool smb358_charging_current_set__non_standard_pre_warm(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__PRE_CHG__WARM_CURRNET);
}

static bool smb358_charging_current_set__non_standard_trick(struct smb358_charger *smb358_chg)
{
	CHG_ERR("%s:trick charging no need to do anything for software \n", __func__);
	return true;
}

static bool smb358_charging_current_set__non_standard_pre(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__INVALID:
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_charging_current_set__non_standard_pre_litte_cold(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_charging_current_set__non_standard_pre_cool(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_charging_current_set__non_standard_pre_normal(smb358_chg);		
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_charging_current_set__non_standard_pre_warm(smb358_chg);		
			break;
		default:
			break;
	}
	return ret;

}

static bool smb358_charging_current_set__non_standard_fast_litte_cold(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__FAST_CHG__LITTLE_COLD_CURRNET);
}

static bool smb358_charging_current_set__non_standard_fast_cool(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__FAST_CHG__COOL_CURRNET);
}

static bool smb358_charging_current_set__non_standard_fast_normal(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__FAST_CHG__NORMAL_CURRNET);
}

static bool smb358_charging_current_set__non_standard_fast_warm(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__FAST_CHG__WARM_CURRNET);

}

static bool smb358_charging_current_set__non_standard_fast(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__INVALID:
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_charging_current_set__non_standard_fast_litte_cold(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_charging_current_set__non_standard_fast_cool(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_charging_current_set__non_standard_fast_normal(smb358_chg);		
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_charging_current_set__non_standard_fast_warm(smb358_chg);		
			break;
		default:
			break;
	}
	return ret;
}

static bool smb358_charging_current_set__non_standard_taper_litte_cold(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__TAPER_CHG__LITTLE_COLD_CURRNET);
}

static bool smb358_charging_current_set__non_standard_taper_cool(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__TAPER_CHG__COOL_CURRNET);
}

static bool smb358_charging_current_set__non_standard_taper_normal(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__TAPER_CHG__NORMAL_CURRNET);
}

static bool smb358_charging_current_set__non_standard_taper_warm(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, NON_STANDARD__TAPER_CHG__WARM_CURRNET);
}

static bool smb358_charging_current_set__non_standard_taper(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__INVALID:
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_charging_current_set__non_standard_taper_litte_cold(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_charging_current_set__non_standard_taper_cool(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_charging_current_set__non_standard_taper_normal(smb358_chg);		
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_charging_current_set__non_standard_taper_warm(smb358_chg);		
			break;
		default:
			break;
	}
	return ret;
}

static bool smb358_charging_current_set__usb_trick(struct smb358_charger *smb358_chg)
{
	CHG_ERR("%s:trick charging no need to do anything for software \n", __func__);
	return true;
}

static bool smb358_charging_current_set__usb_pre_litte_cold(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__PRE_CHG__LITTLE_COLD_CURRNET);
}

static bool smb358_charging_current_set__usb_pre_cool(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__PRE_CHG__COOL_CURRNET);
}

static bool smb358_charging_current_set__usb_pre_normal(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__PRE_CHG__NORMAL_CURRNET);
}

static bool smb358_charging_current_set__usb_pre_warm(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__PRE_CHG__WARM_CURRNET);
}

static bool smb358_charging_current_set__usb_pre(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__INVALID:
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_charging_current_set__usb_pre_litte_cold(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_charging_current_set__usb_pre_cool(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_charging_current_set__usb_pre_normal(smb358_chg);		
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_charging_current_set__usb_pre_warm(smb358_chg);		
			break;
		default:
			break;
	}
	return ret;

}

static bool smb358_charging_current_set__usb_fast_litte_cold(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__FAST_CHG__LITTLE_COLD_CURRNET);
}

static bool smb358_charging_current_set__usb_fast_cool(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__FAST_CHG__COOL_CURRNET);
}

static bool smb358_charging_current_set__usb_fast_normal(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__FAST_CHG__NORMAL_CURRNET);
}

static bool smb358_charging_current_set__usb_fast_warm(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__FAST_CHG__WARM_CURRNET);
}

static bool smb358_charging_current_set__usb_fast(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__INVALID:
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_charging_current_set__usb_fast_litte_cold(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_charging_current_set__usb_fast_cool(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_charging_current_set__usb_fast_normal(smb358_chg);		
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_charging_current_set__usb_fast_warm(smb358_chg);		
			break;
		default:
			break;
	}
	return ret;

}

static bool smb358_charging_current_set__usb_taper_litte_cold(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__TAPER_CHG__LITTLE_COLD_CURRNET);
}

static bool smb358_charging_current_set__usb_taper_cool(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__TAPER_CHG__COOL_CURRNET);
}

static bool smb358_charging_current_set__usb_taper_normal(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__TAPER_CHG__NORMAL_CURRNET);
}

static bool smb358_charging_current_set__usb_taper_warm(struct smb358_charger *smb358_chg)
{
	return smb358_charging_current_write(smb358_chg, USB__TAPER_CHG__WARM_CURRNET);
}

static bool smb358_charging_current_set__usb_taper(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__INVALID:
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_charging_current_set__usb_taper_litte_cold(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_charging_current_set__usb_taper_cool(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_charging_current_set__usb_taper_normal(smb358_chg);		
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_charging_current_set__usb_taper_warm(smb358_chg);		
			break;
		default:
			break;
	}
	return ret;
}

static bool smb358_charging_current_set__standard(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_charging_mode_get(smb358_chg)){
		case CHARGING_MODE__INVALID:
			break;
		case CHARGING_MODE__TRICK_CHG:
			ret = smb358_charging_current_set__standard_trick(smb358_chg);
			break;
		case CHARGING_MODE__PRE_CHG:
			ret = smb358_charging_current_set__standard_pre(smb358_chg);
			break;
		case CHARGING_MODE__FAST_CHG:
			ret = smb358_charging_current_set__standard_fast(smb358_chg);
			break;
		case CHARGING_MODE__TAPER_CHG:
			ret = smb358_charging_current_set__standard_taper(smb358_chg);
			break;
		default:
			break;
	}
	return ret;
}

static bool smb358_charging_current_set__non_standard(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_charging_mode_get(smb358_chg)){
		case CHARGING_MODE__INVALID:
			break;
		case CHARGING_MODE__TRICK_CHG:
			ret = smb358_charging_current_set__non_standard_trick(smb358_chg);
			break;
		case CHARGING_MODE__PRE_CHG:
			ret = smb358_charging_current_set__non_standard_pre(smb358_chg);
			break;
		case CHARGING_MODE__FAST_CHG:
			ret = smb358_charging_current_set__non_standard_fast(smb358_chg);
			break;
		case CHARGING_MODE__TAPER_CHG:
			ret = smb358_charging_current_set__non_standard_taper(smb358_chg);
			break;
		default:
			break;
	}
	return ret;
}

static bool smb358_charging_current_set__usb(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_charging_mode_get(smb358_chg)){
		case CHARGING_MODE__INVALID:
			break;
		case CHARGING_MODE__TRICK_CHG:
			ret = smb358_charging_current_set__usb_trick(smb358_chg);
			break;
		case CHARGING_MODE__PRE_CHG:
			ret = smb358_charging_current_set__usb_pre(smb358_chg);
			break;
		case CHARGING_MODE__FAST_CHG:
			ret = smb358_charging_current_set__usb_fast(smb358_chg);
			break;
		case CHARGING_MODE__TAPER_CHG:
			ret = smb358_charging_current_set__usb_taper(smb358_chg);
			break;
		default:
			break;
	}
	return ret;
}

static void smb358_irq_registers_read(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	u8 value= 0;
/*STAT_REG_C_REGISTER_ADDR*/
	ret = smb358_register_read(smb358_chg->client, STAT_REG_C_REGISTER_ADDR, &value);
	if(ret){	//for charging mode read
		switch((value & CHG_MODE_STAT__MASK) >> 1){
			case CHG_MODE_STAT__NO_CHG:
				smb358_bit_set(smb358_chg, IRQ_FLAGS__NO_CHG);
				break;
			case CHG_MODE_STAT__PRE_CHG:
				smb358_bit_set(smb358_chg, IRQ_FLAGS__CHG_MODE_PRE);
				break;
			case CHG_MODE_STAT__FAST_CHG:
				smb358_bit_set(smb358_chg, IRQ_FLAGS__CHG_MODE_FAST);
				break;
			case CHG_MODE_STAT__TAPER_CHG:
				smb358_bit_set(smb358_chg, IRQ_FLAGS__CHG_MODE_TAPER);
				break;
			default:
				break;
		}	
	}else{
		CHG_ERR("%s:status register C read failed\n", __func__);
	}
/*STAT_REG_D_REGISTER_ADDR*/
	ret = smb358_register_read(smb358_chg->client, STAT_REG_D_REGISTER_ADDR, &value);
/*INTERRUPTE REGISETER B*/
	ret = smb358_register_read(smb358_chg->client, INTERRUPT_STAT_REG_B_REGISTER_ADDR, &value);
	if (ret){
		if( value & BAT_MISSING_STAT) {//for battery missing
			smb358_bit_set(smb358_chg, IRQ_FLAGS__BAT_MISSING);
			CHG_ERR("===%s:battery missing IRQ!===\n", __func__);}
		
	}else{
		CHG_ERR("%s:interrupte stat B register value read failed\n", __func__);
	}
/*INTERRUPT REGISTER C*/
	ret = smb358_register_read(smb358_chg->client, INTERRUPT_STAT_REG_C_REGISTER_ADDR, &value);
	if (ret){
		if( value & RECHG_BAT_THRESHOLD_STAT)//for chg resume
			smb358_bit_set(smb358_chg, IRQ_FLAGS__CHG_RESUME);
		if( value & TERM_CHG_CURRENT_STAT) {//for chg complete
			smb358_bit_set(smb358_chg, IRQ_FLAGS__CHG_COMPLETE);
			CHG_ERR("===%s:battery charging complete IRQ!===\n", __func__);
		}
	}else{
		CHG_ERR("%s:interrupte stat C register value read failed\n", __func__);
	}

	ret = smb358_register_read(smb358_chg->client, FAULT_INTERRUPT_REGISTER_ADDR, &value);
	if (ret) {
		;//add here
	}else{
		CHG_ERR("%s:interrupte 0xc register value read failed\n", __func__);
	}
	ret = smb358_register_read(smb358_chg->client, STAT_INTERRUPT_REGISTER_ADDR, &value);
	if (ret) {
		;//add here
	}else{
		CHG_ERR("%s:interrupte 0xd register value read failed\n", __func__);
	}
	ret = smb358_register_read(smb358_chg->client, INTERRUPT_STAT_REG_A_REGISTER_ADDR, &value);
	if (ret) {
		;//add here
	}else{
		CHG_ERR("%s:interrupte 0x35 register value read failed\n", __func__);
	}
	ret = smb358_register_read(smb358_chg->client, INTERRUPT_STAT_REG_D_REGISTER_ADDR, &value);
	if (ret) {
		if (value & COMPLETE_CHG_TIMEOUT_STAT)//for chg timeout
			smb358_bit_set(smb358_chg, IRQ_FLAGS__CHG_TIMEOUT);//sjc0820
	}else{
		CHG_ERR("%s:interrupte 0x38 register value read failed\n", __func__);
	}
	ret = smb358_register_read(smb358_chg->client, INTERRUPT_STAT_REG_E_REGISTER_ADDR, &value);
	if (ret) {
		;//add here
	}else{
		CHG_ERR("%s:interrupte 0x39 register value read failed\n", __func__);
	}
	ret = smb358_register_read(smb358_chg->client, INTERRUPT_STAT_F_REGISTER_ADDR, &value);
	if (ret) {
		;//add here
	}else{
		CHG_ERR("%s:interrupte 0x3a register value read failed\n", __func__);
	}
}

/*for wokrs define begin*/
static void smb358_irq_work(struct work_struct  *work)
{
	struct smb358_charger *smb358_chg = container_of(work, struct smb358_charger, irq_work);
/**/
	smb358_irq_registers_read(smb358_chg);
/**/
	smb358_reschedule_update_work(smb358_chg);
}

static void smb358_start_charging_work(struct work_struct *work)
{
	struct delayed_work *start_chg_work = to_delayed_work(work);
	struct smb358_charger *smb358_chg = container_of(start_chg_work, struct smb358_charger, start_charging_work);
	
	//sjc0810
	/*OVP or UVP when plug in charger*/
	int charger_vol = get_charger_voltage();
	int counts = 0;
	
	//sjc1023 begin
	if (charger_vol > CHARGER_OVP_VOLTAGE || charger_vol < CHARGER_UVP_VOLTAGE) {
		while (counts < CHARGER_VOLTAGE_READ_TIMES) {
			msleep(100);
			if (smb358_is_charger_in(smb358_chg)) {
				charger_vol = get_charger_voltage();
				if (charger_vol > CHARGER_OVP_VOLTAGE || charger_vol < CHARGER_UVP_VOLTAGE) {
					counts++;
					continue;
				}
				else
					break;
			}
			else
				break;
		}
	}
	smb358_charger_voltage_set(smb358_chg, charger_vol);
	if (charger_vol > CHARGER_OVP_VOLTAGE || charger_vol < CHARGER_UVP_VOLTAGE)
		CHG_ERR("%s:charger_vol=%d\n", __func__, charger_vol);
	if (smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__DCP || smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__NON_DCP)
		power_supply_changed(smb358_chg->power_supplies.mains);
	else if (smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__SDP)
		power_supply_changed(smb358_chg->power_supplies.usb);
	//sjc1023 end
	
	smb358_charger_status_set(smb358_chg, CHARGER_STATUS__GOOD);//suppose that the status is good before plug in OVP/UVP charger
	if (charger_vol > CHARGER_OVP_VOLTAGE) {
		in_ovp_status = true;
		ovp_counts = 1;
		smb358_charger_voltage_handle(smb358_chg);
		return;
	} 
	else if (charger_vol < CHARGER_UVP_VOLTAGE) {
		in_uvp_status = true;
		uvp_counts = 1;
		smb358_charger_voltage_handle(smb358_chg);
		return;
	}
	//sjc1014
	/*there maybe an uvp when DCP was pulled out last time, if disconnect process was finished
	 *before suspend operation, the unsuspend operation would not be execute. so the following
	 */
	if (smb358_suspended_get(the_smb358_charger)) {
		smb358_unsuspend_charger(the_smb358_charger);
		CHG_ERR("%s:smb358 don't unsuspend when charger pulled out last time\n", __func__);
	}
	//sjc0810
	if (smb358_start_charging(smb358_chg)) {
		//CHG_ERR("===%s: enter===\n", __func__);
		smb358_start_chg_timeout_timer(smb358_chg);//sjc0820
		//smb358_max_chg_timeout_flag_set(smb358_chg, true);
	}
}

//read charger vol,battery vol,battery temp,charging current capacity and so on, only in this func
static void smb358_charging_current_read(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	u8 value= 0;
	int chg_cur = 0;
	return;//DO NOT use
	ret = smb358_register_read(smb358_chg->client, STAT_REG_B_REGISTER_ADDR, &value);
	if (ret){
		CHG_DBG("%s:value read is=%x\n", __func__, value);
		if ((value & CHG_CURRENT_AFTER_COMPENSATION_2__MASK) == 0){
			switch (value & CHG_CURRENT_AFTER_COMPENSATION_1__MASK){
				case CHG_CURRENT_AFTER_COMPENSATION_1__100MA:
					chg_cur = 100; 
					break;
				case CHG_CURRENT_AFTER_COMPENSATION_1__150MA:
					chg_cur = 150; 
					break;
				case CHG_CURRENT_AFTER_COMPENSATION_1__200MA: 
					chg_cur = 200;
					break;
				case CHG_CURRENT_AFTER_COMPENSATION_1__250MA: 
					chg_cur = 250;
					break;
				default:
					return;
			};
		}else{
			switch(value & CHG_CURRENT_AFTER_COMPENSATION_3__MASK){
				case CHG_CURRENT_AFTER_COMPENSATION_3__100MA:
					chg_cur = 100;
					break;
				case CHG_CURRENT_AFTER_COMPENSATION_3__200MA:
					chg_cur = 200;
					break;
				case CHG_CURRENT_AFTER_COMPENSATION_3__450MA:
					chg_cur = 450;
					break;
				case CHG_CURRENT_AFTER_COMPENSATION_3__600MA:
					chg_cur = 600;
					break;
				case CHG_CURRENT_AFTER_COMPENSATION_3__900MA:
					chg_cur = 900;
					break;
				case CHG_CURRENT_AFTER_COMPENSATION_3__1300MA:
					chg_cur = 1300;
					break;
				case CHG_CURRENT_AFTER_COMPENSATION_3__1500MA:
					chg_cur = 1500;
					break;
				case CHG_CURRENT_AFTER_COMPENSATION_3__1800MA:
					chg_cur = 1800;
					break;
				default:
					return;
			};
		}		
		smb358_chg->power_supply_data.charging_current = chg_cur;
	}
	else
		CHG_ERR("%s:read charging current failed\n", __func__);
}

static  void smb358_battery_voltage_read(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	u8 value = 0;
	int bat_vol = 0;
	return;//DO NOT use
	ret = smb358_register_read(smb358_chg->client, ADC_STAT_REGISTER_ADDR, &value);
	if (ret){
		CHG_DBG("%s: data read=0x%x\n", __func__, value);
		bat_vol =((value - BAT_VOLTAGE__BASE) * BAT_VOLTAGE__STEP_UV) / 1000;
		bat_vol = bat_vol > BAT_VOLTAGE__MAX? BAT_VOLTAGE__BASE: bat_vol;
		smb358_battery_voltage_set(smb358_chg, bat_vol);
	}else{
		CHG_ERR("%s:read battery voltage register failed\n", __func__);
	}
}

extern int get_charger_voltage_from_adc(void);
extern int get_battery_temperature_from_adc(void);
extern int get_charging_current_from_adc(void);
extern int get_battery_voltage_from_adc(void);

static void smb358_power_supply_data_read(struct smb358_charger *smb358_chg)
{
	//user auto test 
	if (smb358_debug_enabled_get(smb358_chg)){
		smb358_charger_type_set(smb358_chg, smb358_debug_charger_type_get(smb358_chg));
		smb358_charger_voltage_set(smb358_chg, smb358_debug_charger_voltage_get(smb358_chg));
		smb358_battery_voltage_set(smb358_chg, smb358_debug_battery_voltage_get(smb358_chg));
		smb358_battery_temperature_set(smb358_chg, smb358_debug_battery_temperature_get(smb358_chg));
		smb358_battery_capacity_set(smb358_chg, smb358_debug_battery_capacity_get(smb358_chg));
		smb358_battery_soh_set(smb358_chg, get_battery_soh());
		smb358_battery_fcc_set(smb358_chg, get_battery_fcc());
		smb358_chg->power_supply_data.charging_current = smb358_debug_charging_current_get(smb358_chg);
		smb358_chg->power_supply_data.charging_current = get_charging_current();
		smb358_charging_timeout_set(smb358_chg, smb358_debug_charging_timeout_get(smb358_chg));
	}else{
		smb358_charger_voltage_set(smb358_chg, get_charger_voltage());
		smb358_battery_voltage_read(smb358_chg);
		smb358_battery_voltage_set(smb358_chg, get_battery_voltage());
		smb358_battery_temperature_set(smb358_chg, get_battery_temperature() / 10);
		smb358_battery_capacity_set(smb358_chg, get_battery_capacity());//30);
		smb358_battery_soh_set(smb358_chg, get_battery_soh());
		smb358_battery_fcc_set(smb358_chg, get_battery_fcc());
		smb358_charging_current_read(smb358_chg);
		smb358_chg->power_supply_data.charging_current = get_charging_current();
//		smb358_charger_voltage_set(smb358_chg, 5000);
//		smb358_battery_voltage_read(smb358_chg);
//		smb358_battery_temperature_set(smb358_chg, 30);
//		smb358_battery_capacity_set(smb358_chg, 30);
//		smb358_charging_current_read(smb358_chg);
	}
}

/*return true means good, return false means uovp*/
static bool smb358_charger_ovp_check(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int times = 0;
	int chg_vol = smb358_charger_voltage_get(smb358_chg);
	
	if (chg_vol > CHARGER_OVP_VOLTAGE) {//sjc1023
		while (times < CHARGER_VOLTAGE_READ_TIMES) {
			msleep(100);
			if (smb358_is_charger_in(the_smb358_charger)) {
				chg_vol = get_charger_voltage();
				if (chg_vol > CHARGER_OVP_VOLTAGE) {
					times++;
					continue;
				}
				else
					break;
			}
			else
				break;
		}
		smb358_charger_voltage_set(smb358_chg, chg_vol);//update data
	}
	
	if (chg_vol > CHARGER_OVP_VOLTAGE) {
		//count++;
		if (!in_ovp_status) {
			ovp_counts++;//sjc0806
			in_ovp_status = true;
			//CHG_ERR("===%s:ovp_counts=%d, in_ovp_status=%d===\n", __func__, ovp_counts, in_ovp_status);
		}
		return false;//sjc0806
	}else{
		//count = 0;
		in_ovp_status = false;
		//CHG_ERR("===%s:ovp_counts=%d, in_ovp_status=%d===\n", __func__, ovp_counts, in_ovp_status);
		return true;//sjc0806
	}
	
	CHG_DBG("%s:count=%d[%d],cahrger voltge=%d\n", __func__, count, CHARGER_OVP_CHECK_COUNT, chg_vol);
	if (count >= CHARGER_OVP_CHECK_COUNT){
		count = 0;
		return false;
	}else{
		return true;	
	}
}

static bool smb358_charger_uvp_check(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int times = 0;
	int chg_vol = smb358_charger_voltage_get(smb358_chg);

	if (chg_vol < CHARGER_UVP_VOLTAGE) {//sjc1023
		while (times < CHARGER_VOLTAGE_READ_TIMES) {
			msleep(100);
			if (smb358_is_charger_in(the_smb358_charger)) {
				chg_vol = get_charger_voltage();
				if (chg_vol < CHARGER_UVP_VOLTAGE) {
					times++;
					continue;
				}
				else
					break;
			}
			else
				break;
		}
		smb358_charger_voltage_set(smb358_chg, chg_vol);//update data
	}

	if (chg_vol < CHARGER_UVP_VOLTAGE) {
		//count++;
		if (!in_uvp_status) {
			uvp_counts++;//sjc0806
			in_uvp_status = true;
			//CHG_ERR("===%s:uvp_counts=%d, in_uvp_status=%d===\n", __func__, uvp_counts, in_uvp_status);
		}
		return false;//sjc0806
	}
	else
	{
		//count = 0;
		in_uvp_status = false;
		//CHG_ERR("===%s:uvp_counts=%d, in_uvp_status=%d===\n", __func__, uvp_counts, in_uvp_status);
		return true;//sjc0806
	}
	
	CHG_DBG("%s:count=%d[%d],charger voltage=%d\n", __func__, count, CHARGER_UVP_CHECK_COUNT, chg_vol);
	if (count >= CHARGER_UVP_CHECK_COUNT){
		count = 0;
		return false;
	}else{
		return true;	
	}
}

/*true means charger status is good*/
static void smb358_charger_uovp_check(struct smb358_charger *smb358_chg)
{
	int charger_vol= smb358_charger_voltage_get(smb358_chg);
	if (false == smb358_charger_ovp_check(smb358_chg)){
		smb358_charger_status_set(smb358_chg, CHARGER_STATUS__OVP);
		return;
	}else if (false == smb358_charger_uvp_check(smb358_chg)){
		smb358_charger_status_set(smb358_chg, CHARGER_STATUS__UVP);
		return;
	}else{
		if (CHARGER_OVP_VOLTAGE >= charger_vol && CHARGER_UVP_VOLTAGE <= charger_vol)
			smb358_charger_status_set(smb358_chg, CHARGER_STATUS__GOOD);
		return;
	}
}

/*do this only when charger is exist*/
static void smb358_charger_voltage_handle(struct smb358_charger *smb358_chg)
{
	smb358_charger_status charger_stat_pre = smb358_charger_status_get(smb358_chg);
	bool ret = false;
	smb358_charger_status charger_stat_now = CHARGER_STATUS__INVALID;
	
	if (smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__INVALID || smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__OTG)//sjc0806//1010
		return;
	
	smb358_charger_uovp_check(smb358_chg);
	
	charger_stat_now = smb358_charger_status_get(smb358_chg);
	CHG_DBG("%s:charger status pre=%s, now=%s\n", __func__, charger_status_str[charger_stat_pre], charger_status_str[charger_stat_now]);
	if (charger_stat_pre == charger_stat_now)
		return;
	
	if (CHARGER_TYPE__INVALID == smb358_charger_type_get(smb358_chg))
		return;
	
	if ((CHARGER_STATUS__OVP == charger_stat_pre || CHARGER_STATUS__UVP == charger_stat_pre) 
		&& (CHARGER_STATUS__GOOD == charger_stat_now)){
		if (ovp_counts >= CHARGER_OVP_CHECK_COUNT || uvp_counts >= CHARGER_UVP_CHECK_COUNT)//sjc0806
			return;
		
		CHG_ERR("%s:charger voltage resume to normal ,start to charging\n", __func__);
		
		if (CHARGER_STATUS__UVP == charger_stat_pre) {
			smb358_unsuspend_charger(smb358_chg);//sjc0806
			smb358_charging_status_set(smb358_chg, CHARGING_STATUS__CHARGING);//sjc0814
		}
			
		ret = smb358_start_charging(smb358_chg);
		if (false == ret)
			CHG_ERR("%s:trying to start charging failed\n", __func__);
	}else{
		if (CHARGER_STATUS__GOOD == charger_stat_pre && charger_stat_now == CHARGER_STATUS__OVP) {//sjc0806
			CHG_ERR("%s:charger is ovp,starting to stop charging \n", __func__);
			smb358_unsuspend_charger(smb358_chg);//sjc0806
			smb358_stop_charging(smb358_chg);//sjc0806
			return;
		}//sjc0806
		if (CHARGER_STATUS__GOOD == charger_stat_pre && charger_stat_now == CHARGER_STATUS__UVP) {//sjc0806
			CHG_ERR("%s:charger is uvp, starting to stop charging\n", __func__);
			//ret = smb358_stop_charging(smb358_chg);
			smb358_suspend_charger(smb358_chg);//sjc0806
			smb358_charging_status_set(smb358_chg, CHARGING_STATUS__DISCHARGING);//sjc0814
			return;
		}//sjc0806
		if (false == ret)
			CHG_ERR("%s:trying to stop charging failed\n", __func__);
	}
}

static void smb358_battery_missing_check(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	static int count_in_charger = 0;
	int bat_vol = -1;
	int i = 0;
	int j = 0;
	/*
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < BAT_MISSING_VOLTAGE)
		count++;
	else{
		smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__GOOD);
		count = 0;
	}
	CHG_DBG("%s:count=%d[%d], battery voltage=%d\n", __func__, count, BAT_MISSING_CHECK_COUNT, bat_vol);
	if (count >= BAT_MISSING_CHECK_COUNT){
		count = 0;
		smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__MISSED);
	}
	*/
	if (get_pcb_version() >= PCB_VERSION_EVT3_N1F) {//for EVT3 and later...
#if 0//sjc1019 to avoid the IC bug(sometime it cannot be read in a short time)
		if (batt_fuelgauge && batt_fuelgauge->get_battery_mvolts)
			bat_vol = batt_fuelgauge->get_battery_mvolts();
		if (bat_vol == -1 || get_battery_voltage_from_adc() < BAT_MISSING_VOLTAGE) {
			for (i = 1; i <= BAT_MISSING_CHECK_COUNT; i++) {
				mdelay(200);
				if (batt_fuelgauge && batt_fuelgauge->get_battery_mvolts)
					bat_vol = batt_fuelgauge->get_battery_mvolts();
				if (bat_vol == -1 || get_battery_voltage_from_adc() < BAT_MISSING_VOLTAGE)
					continue;
				else
					break;
			}
			if (i >= BAT_MISSING_CHECK_COUNT) {
				smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__MISSED);
				CHG_ERR("===%s:count=%d[%d], battery voltage=%d===\n", __func__, i, BAT_MISSING_CHECK_COUNT, bat_vol);
			}
			else
				smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__GOOD);
		}
		else
			smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__GOOD);
//#else
		if (batt_fuelgauge && batt_fuelgauge->get_battery_mvolts)
			bat_vol = batt_fuelgauge->get_battery_mvolts();
		if (bat_vol == -1 || get_battery_voltage_from_adc() < BAT_MISSING_VOLTAGE) {
			count++;
			CHG_ERR("===%s:battery missing maybe take place[%d]===\n", __func__, count);
		} else {
			count = 0;
			smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__GOOD);
		}
		if (count >= BAT_MISSING_CHECK_COUNT) {
			smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__MISSED);
			CHG_ERR("===%s:count=%d[%d], battery voltage=%d===\n", __func__, count, BAT_MISSING_CHECK_COUNT, bat_vol);
			count = 0;
		}
#endif
		/* sjc1024 begin
		** if charger is in Vadc=Vcharger, we only use batt_fuelgauge to detect
		** if charger is not in Vadc=Vbattery, we use batt_fuelgauge and ADC to detect
		**/
		if (smb358_is_charger_in(smb358_chg) && batt_fuelgauge && batt_fuelgauge->get_battery_mvolts) {
			if (batt_fuelgauge->get_battery_mvolts() == -1) {
				for (i = 1; i < BAT_MISSING_CHECK_COUNT; i++) {
					msleep(200);
					if (smb358_is_charger_in(smb358_chg) && batt_fuelgauge && batt_fuelgauge->get_battery_mvolts) {
						if (batt_fuelgauge->get_battery_mvolts() == -1) {
							count_in_charger++;
							if (count_in_charger > 10)
								count_in_charger = 10;
							continue;
						} else {//battery is good
							count_in_charger = 0;
							count = 0;
							break;
						}
					}
				}
			} else {//battery is good
				count_in_charger = 0;
				count = 0;
			}
		} else {//charger not in
			if ((bat_vol = get_battery_voltage()) < BAT_MISSING_VOLTAGE) {
				for (j = 1; j < BAT_MISSING_CHECK_COUNT; j++) {
					msleep(200);
					if (!smb358_is_charger_in(smb358_chg)) {
						if ((bat_vol = get_battery_voltage()) < BAT_MISSING_VOLTAGE) {
							count++;
							if (count > 10)
								count = 10;
							continue;
						} else {//battery is good
							count = 0;
							count_in_charger = 0;
							break;
						}
					}
				}
			} else {//battery is good
				count = 0;
				count_in_charger = 0;
			}
		}
		
		if (count_in_charger > BAT_MISSING_CHECK_COUNT || count > BAT_MISSING_CHECK_COUNT) {
			smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__MISSED);
			CHG_ERR("===%s:count=[%d], count_in_charger=[%d], battery voltage=%d===\n", __func__, count, count_in_charger, bat_vol);
		} else if ((count_in_charger >= 1 && count >= 2) || (count >= 1 && count_in_charger >= 2)) {
			smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__MISSED);
			CHG_ERR("===%s:count=[%d], count_in_charger=[%d], battery voltage=%d===\n", __func__, count, count_in_charger, bat_vol);
		} else {
			smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__GOOD);
		}
		//sjc1024end
	} else {//for EVT1 and EVT2
		bat_vol = get_battery_voltage();
		if (bat_vol < BAT_MISSING_VOLTAGE) {
			for (i = 1; i <= BAT_MISSING_CHECK_COUNT; i++) {
				mdelay(200);
				bat_vol = get_battery_voltage();
				if (bat_vol > BAT_MISSING_VOLTAGE)
					break;
			}
			if (i >= BAT_MISSING_CHECK_COUNT) {
				smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__MISSED);
				CHG_ERR("===%s:count=%d[%d], battery voltage=%d===\n", __func__, i, BAT_MISSING_CHECK_COUNT, bat_vol);
			}
			else
				smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__GOOD);
		}
		else
			smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__GOOD);
	}
}

static void smb358_battery_missing_handle(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	smb358_battery_missing_status  bat_missing_pre = smb358_battery_missing_status_get(smb358_chg);
	smb358_battery_missing_status  bat_missing_now = BATTERY_MISSING_STATUS__INVALID;
	if (BATTERY_MISSING_STATUS__MISSED == bat_missing_pre){
		if (CHARGING_STATUS__CHARGING != smb358_charging_status_get(smb358_chg))
			goto out;
		CHG_ERR("%s:battery missed ,starting to stop charging\n", __func__);
		ret = smb358_stop_charging(smb358_chg);
		if (false == ret)
			CHG_ERR("%s:trying to sotp charging failed\n", __func__);	
	}
out:
	smb358_battery_missing_check(smb358_chg);
	bat_missing_now = smb358_battery_missing_status_get(smb358_chg);
	if (BATTERY_MISSING_STATUS__MISSED == bat_missing_pre && 
		BATTERY_MISSING_STATUS__GOOD == bat_missing_now){
		CHG_ERR("%s:battery re-inserted ,starting to enable charging\n", __func__);
		ret = smb358_start_charging(smb358_chg);
		if (false == ret)
			CHG_ERR("%s:trying to starting charging failed\n", __func__);	
	}
		
}

static bool smb358_battery_ovp_check(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int bat_vol = smb358_battery_voltage_get(smb358_chg) ;
	if (bat_vol > BATTERY_OVP_VOLTAGE)
		count++;
	else
		count = 0;
	CHG_DBG("%s:count=%d[%d], battery voltage=%d\n", __func__, count, BATTERY_OVP_CHECK_COUNT, bat_vol);	
	if (count >= BATTERY_OVP_CHECK_COUNT){
		count = 0;
		return false;
	}else{
		return true;	
	}
}

static void smb358_battery_voltage_check(struct smb358_charger *smb358_chg)
{
	int bat_vol = smb358_battery_voltage_get(smb358_chg);//smb358_battery_status_get(smb358_chg);
	CHG_DBG("%s:battery voltage=%d\n", __func__, bat_vol);//smb358_battery_voltage_get(smb358_chg));
	if (false == smb358_battery_ovp_check(smb358_chg)){
		smb358_battery_status_set(smb358_chg, BATTERY_STATUS__OVP);
		return;
	}
	/*if we need to handle battery missing ,do here*/
	if (BATTERY_OVP_VOLTAGE >= bat_vol){
		smb358_battery_status_set(smb358_chg, BATTERY_STATUS__GOOD);
	}
	return;
}

/*do this only when battery is exist*/
static void smb358_battery_voltage_handle(struct smb358_charger *smb358_chg)
{
	smb358_battery_status bat_stat_pre = smb358_battery_status_get(smb358_chg);
	bool ret = false; 
	smb358_battery_status bat_stat_now = BATTERY_STATUS__INVALID;
	
	smb358_battery_voltage_check(smb358_chg);
	bat_stat_now = smb358_battery_status_get(smb358_chg);
	CHG_DBG("%s:battery status pre=%s, now=%s\n", __func__, battery_status_str[bat_stat_pre], battery_status_str[bat_stat_now]);
	if (BATTERY_MISSING_STATUS__MISSED == smb358_battery_missing_status_get(smb358_chg))
		return;
	if (bat_stat_pre == bat_stat_now)
		return;
	if (BATTERY_STATUS__OVP == bat_stat_pre && BATTERY_STATUS__GOOD == bat_stat_now){
		CHG_ERR("%s:battery voltage resume from ovp\n", __func__);
	}
	if (BATTERY_STATUS__GOOD == bat_stat_pre && BATTERY_STATUS__OVP == bat_stat_now){
		CHG_ERR("%s:battery is ovp try to stop charging\n", __func__);
		if (smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__INVALID || smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__OTG)//sjc1014
			return;
		ret = smb358_stop_charging(smb358_chg);
		if (false == ret)
			CHG_ERR("%s:trying to stop charging failed\n", __func__);
	}
}

static void smb358_battery_temperature_check(struct smb358_charger *smb358_chg)
{
	int bat_temp = smb358_battery_temperature_get(smb358_chg);
	smb358_battery_temperature_region bat_temp_region_pre = smb358_battery_temperature_region_get(smb358_chg);
	smb358_battery_temperature_region bat_temp_region = BATTERY_TEMP_REGION__INVALID;
	
	if (bat_temp < smb358_cold_to_little_cold_critical_temperature_get(smb358_chg)) {
		bat_temp_region = BATTERY_TEMP_REGION__COLD;
		if (smb358_charger_type_get(smb358_chg) != CHARGER_TYPE__INVALID)//sjc0813 only in charging
			smb358_battery_status_set(smb358_chg, BATTERY_STATUS__COLD);//sjc0806
	}
	else if (bat_temp < smb358_little_cold_to_cool_critical_temperature_get(smb358_chg))
		bat_temp_region = BATTERY_TEMP_REGION__LITTLE_COLD;
	else if (bat_temp < smb358_cool_to_normal_critical_temperature_get(smb358_chg))
		bat_temp_region = BATTERY_TEMP_REGION__COOL;
	else if (bat_temp < smb358_normal_to_warm_critical_temperature_get(smb358_chg))
		bat_temp_region = BATTERY_TEMP_REGION__NORMAL;
	else if (bat_temp < smb358_warm_to_hot_critical_temperature_get(smb358_chg))
		bat_temp_region = BATTERY_TEMP_REGION__WARM;
	else {
		bat_temp_region = BATTERY_TEMP_REGION__HOT;
		if (smb358_charger_type_get(smb358_chg) != CHARGER_TYPE__INVALID)//sjc0813 only in charging
			smb358_battery_status_set(smb358_chg, BATTERY_STATUS__OVERHEAT);//sjc0806
	}
	smb358_battery_temperature_region_set(smb358_chg, bat_temp_region);
	if (bat_temp_region == bat_temp_region_pre)
		return;
	smb358_cold_to_little_cold_critical_temperature_set(smb358_chg, BAT_TEMP__TEN_BELOW_ZERO);
	smb358_little_cold_to_cool_critical_temperature_set(smb358_chg, BAT_TEMP__ZERO);
	smb358_cool_to_normal_critical_temperature_set(smb358_chg, BAT_TEMP__TEN);
	smb358_normal_to_warm_critical_temperature_set(smb358_chg, BAT_TEMP__FORTY_FIVE);
	smb358_warm_to_hot_critical_temperature_set(smb358_chg, BAT_TEMP__FIFTY_FIVE);

	switch(bat_temp_region){
		case BATTERY_TEMP_REGION__COLD:
			if (BATTERY_TEMP_REGION__LITTLE_COLD == bat_temp_region_pre)
				smb358_cold_to_little_cold_critical_temperature_set(smb358_chg,
					smb358_cold_to_little_cold_critical_temperature_get(smb358_chg) + BAT_TEMP__BOUNCE);
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			if (BATTERY_TEMP_REGION__COLD == bat_temp_region_pre)
				smb358_cold_to_little_cold_critical_temperature_set(smb358_chg,
					smb358_cold_to_little_cold_critical_temperature_get(smb358_chg));// - BAT_TEMP__BOUNCE);//sjc0806
			if (BATTERY_TEMP_REGION__COOL == bat_temp_region_pre)
				smb358_little_cold_to_cool_critical_temperature_set(smb358_chg,
					smb358_little_cold_to_cool_critical_temperature_get(smb358_chg) + BAT_TEMP__BOUNCE);
			break;
		case BATTERY_TEMP_REGION__COOL:
			if (BATTERY_TEMP_REGION__LITTLE_COLD == bat_temp_region_pre)
				smb358_little_cold_to_cool_critical_temperature_set(smb358_chg,
					smb358_little_cold_to_cool_critical_temperature_get(smb358_chg) -BAT_TEMP__BOUNCE);
			if (BATTERY_TEMP_REGION__NORMAL == bat_temp_region_pre)
				smb358_cool_to_normal_critical_temperature_set(smb358_chg,
					smb358_cool_to_normal_critical_temperature_get(smb358_chg) + BAT_TEMP__BOUNCE);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			if (BATTERY_TEMP_REGION__COOL == bat_temp_region_pre)
				smb358_cool_to_normal_critical_temperature_set(smb358_chg,
					smb358_cool_to_normal_critical_temperature_get(smb358_chg) - BAT_TEMP__BOUNCE);
			if (BATTERY_TEMP_REGION__WARM == bat_temp_region_pre)
				smb358_normal_to_warm_critical_temperature_set(smb358_chg,
					smb358_normal_to_warm_critical_temperature_get(smb358_chg) + BAT_TEMP__BOUNCE);
			break;
		case BATTERY_TEMP_REGION__WARM:
			if (BATTERY_TEMP_REGION__NORMAL == bat_temp_region_pre)
				smb358_normal_to_warm_critical_temperature_set(smb358_chg,
					smb358_normal_to_warm_critical_temperature_get(smb358_chg) - BAT_TEMP__BOUNCE);
			if (BATTERY_TEMP_REGION__HOT == bat_temp_region_pre)
				smb358_warm_to_hot_critical_temperature_set(smb358_chg,
					smb358_warm_to_hot_critical_temperature_get(smb358_chg) + BAT_TEMP__BOUNCE);
			break;
		case BATTERY_TEMP_REGION__HOT:
			if (BATTERY_TEMP_REGION__WARM == bat_temp_region_pre)
				smb358_warm_to_hot_critical_temperature_set(smb358_chg,
					smb358_warm_to_hot_critical_temperature_get(smb358_chg) - BAT_TEMP__BOUNCE);
			break;
		default:
			break;
	}
}

static bool smb358_float_voltage_standard_little_cold_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__LITTLE_COLD);
}

static bool smb358_float_voltage_standard_cool_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__COOL);
}

static bool smb358_float_voltage_standard_normal_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__NORMAL);
}

static bool smb358_float_voltage_standard_warm_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__WARM);
}

static bool smb358_float_voltage_standard_set(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_float_voltage_standard_little_cold_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_float_voltage_standard_cool_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_float_voltage_standard_normal_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_float_voltage_standard_warm_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
		default:
			break;
	}
	return ret;
}

static bool smb358_float_voltage_non_standard_little_cold_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__LITTLE_COLD);
}

static bool smb358_float_voltage_non_standard_cool_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__COOL);
}

static bool smb358_float_voltage_non_standard_normal_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__NORMAL);
}

static bool smb358_float_voltage_non_standard_warm_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__WARM);
}

static bool smb358_float_voltage_non_standard_set(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_float_voltage_non_standard_little_cold_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_float_voltage_non_standard_cool_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_float_voltage_non_standard_normal_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_float_voltage_non_standard_warm_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
		default:
			break;
	}
	return ret;
}

static bool smb358_float_voltage_usb_little_cold_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__USB__LITTLE_COLD);
}

static bool smb358_float_voltage_usb_cool_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__USB__COOL);
}

static bool smb358_float_voltage_usb_normal_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__USB__NORMAL);
}

static bool smb358_float_voltage_usb_warm_set(struct smb358_charger *smb358_chg)
{
	return smb358_float_voltage_write(smb358_chg, CHG_COMPLETE_CHECK_VOLTAGE__USB__WARM);
}

static bool smb358_float_voltage_usb_set(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			ret = smb358_float_voltage_usb_little_cold_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			ret = smb358_float_voltage_usb_cool_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			ret = smb358_float_voltage_usb_normal_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__WARM:
			ret = smb358_float_voltage_usb_warm_set(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
		default:
			break;
	}
	return ret;
}

static bool smb358_float_voltage_set(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch (smb358_charger_type_get(smb358_chg)){
		case CHARGER_TYPE__SDP:
			ret = smb358_float_voltage_usb_set(smb358_chg);
			break;
		case CHARGER_TYPE__DCP:
			ret = smb358_float_voltage_standard_set(smb358_chg);
			break;
		case CHARGER_TYPE__NON_DCP:
			ret = smb358_float_voltage_non_standard_set(smb358_chg);
			 break;
		case CHARGER_TYPE__INVALID:
		default:
			ret = true;
			break;
	}
	return ret;
}

static void smb358_battery_temperature_handle(struct smb358_charger *smb358_chg)
{
	bool ret = true;
	smb358_battery_temperature_region bat_temp_region_pre = smb358_battery_temperature_region_get(smb358_chg);
	smb358_battery_temperature_region bat_temp_region_now = BATTERY_TEMP_REGION__INVALID;
	if (BATTERY_MISSING_STATUS__MISSED == smb358_battery_missing_status_get(smb358_chg))//sjc1104
		return;
	smb358_battery_temperature_check(smb358_chg);	
	bat_temp_region_now = smb358_battery_temperature_region_get(smb358_chg);
	CHG_DBG("%s:temp region pre=%s, temp region now=%s\n",
		__func__, battery_temp_region_str[bat_temp_region_pre], battery_temp_region_str[bat_temp_region_now]);
	if (bat_temp_region_pre == bat_temp_region_now)
		return;
	
	switch (bat_temp_region_now){
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			if (smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__SDP || smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__DCP
					|| smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__NON_DCP)//sjc1014
				ret = smb358_stop_charging(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
		case BATTERY_TEMP_REGION__COOL:
		case BATTERY_TEMP_REGION__NORMAL:
		case BATTERY_TEMP_REGION__WARM:
			if (smb358_charger_status_get(smb358_chg) != CHARGER_STATUS__UVP && smb358_charger_status_get(smb358_chg) != CHARGER_STATUS__OVP)//sjc1104
				ret = smb358_start_charging(smb358_chg);
			break;
		default:
			break;
	}
	if ((false == ret) && (CHARGER_TYPE__INVALID != smb358_charger_type_get(smb358_chg)) && (CHARGER_TYPE__OTG != smb358_charger_type_get(smb358_chg))){//sjc0809//1014
		CHG_ERR("%s:start or stop charging failed,restore temp region to pre,so that next update we can do this again\n", __func__);
		smb358_battery_temperature_region_set(smb358_chg, bat_temp_region_pre);		
	} else {//sjc0809
		smb358_battery_temperature_region_set(smb358_chg, bat_temp_region_now);	
	}
}

static void smb358_charging_mode_check_and_set(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	u8 value = 0;
	
	return;
	ret = smb358_register_read(smb358_chg->client, STAT_REG_C_REGISTER_ADDR, &value);
	if (ret) {	//for charging mode read
		CHG_ERR("===%s:0x3d-0x%x===\n", __func__, value);
		switch (value & CHG_MODE_STAT__MASK) {
			case CHG_MODE_STAT__NO_CHG:
				//smb358_bit_set(smb358_chg, IRQ_FLAGS__NO_CHG);
				smb358_charging_mode_set(smb358_chg, CHARGING_MODE__INVALID);
				//CHG_ERR("===%s:charging mode=%s===\n", __func__, charging_mode_str[smb358_charging_mode_get(smb358_chg)]);
				break;
			case 2://CHG_MODE_STAT__PRE_CHG:
				//smb358_bit_set(smb358_chg, IRQ_FLAGS__CHG_MODE_PRE);
				smb358_charging_mode_set(smb358_chg, CHARGING_MODE__PRE_CHG);
				//CHG_ERR("===%s:charging mode=%s===\n", __func__, charging_mode_str[smb358_charging_mode_get(smb358_chg)]);
				break;
			case 4://CHG_MODE_STAT__FAST_CHG:
				//smb358_bit_set(smb358_chg, IRQ_FLAGS__CHG_MODE_FAST);
				smb358_charging_mode_set(smb358_chg, CHARGING_MODE__FAST_CHG);
				//CHG_ERR("===%s:charging mode=%s===\n", __func__, charging_mode_str[smb358_charging_mode_get(smb358_chg)]);
				break;
			case 5://CHG_MODE_STAT__TAPER_CHG:
				//smb358_bit_set(smb358_chg, IRQ_FLAGS__CHG_MODE_TAPER);
				smb358_charging_mode_set(smb358_chg, CHARGING_MODE__TAPER_CHG);
				//CHG_ERR("===%s:charging mode=%s===\n", __func__, charging_mode_str[smb358_charging_mode_get(smb358_chg)]);
				break;
			default:
				break;
		}
	} else {
		CHG_ERR("%s:status register C read failed\n", __func__);
	}	
}

static void smb358_charging_complete_check_standard_little_cold(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__LITTLE_COLD;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}
}

static void smb358_charging_complete_check_non_standard_little_cold(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__LITTLE_COLD;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}

}

static void smb358_charging_complete_check_usb_little_cold(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__USB__LITTLE_COLD;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}
}

static void smb358_charging_complete_check_little_cold(struct smb358_charger *smb358_chg)
{
	switch (smb358_charger_type_get(smb358_chg)){
		case CHARGER_TYPE__INVALID:
			break;
		case CHARGER_TYPE__SDP:
			smb358_charging_complete_check_usb_little_cold(smb358_chg);
			break;
		case CHARGER_TYPE__DCP:
			smb358_charging_complete_check_standard_little_cold(smb358_chg);
			break;
		case CHARGER_TYPE__NON_DCP:
			smb358_charging_complete_check_non_standard_little_cold(smb358_chg);
			break;
		default:
			CHG_ERR("%s:unsupported charger type!\n", __func__);

	}
}

static void smb358_charging_complete_check_standard_cool(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__COOL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}
}

static void smb358_charging_complete_check_non_standard_cool(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__COOL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}
}

static void smb358_charging_complete_check_usb_cool(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__USB__COOL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}
}

static void smb358_charging_complete_check_cool(struct smb358_charger *smb358_chg)
{
	switch (smb358_charger_type_get(smb358_chg)){
		case CHARGER_TYPE__INVALID:
			break;
		case CHARGER_TYPE__SDP:
			smb358_charging_complete_check_usb_cool(smb358_chg);
			break;
		case CHARGER_TYPE__DCP:
			smb358_charging_complete_check_standard_cool(smb358_chg);
			break;
		case CHARGER_TYPE__NON_DCP:
			smb358_charging_complete_check_non_standard_cool(smb358_chg);
			break;
		default:
			CHG_ERR("%s:unsupported charger type!\n", __func__);

	}
}

static void smb358_charging_complete_check_standard_normal(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__NORMAL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}
}

static void smb358_charging_complete_check_non_standard_normal(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__NORMAL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}
}

static void smb358_charging_complete_check_usb_normal(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__USB__NORMAL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}
}

static void smb358_charging_complete_check_normal(struct smb358_charger *smb358_chg)
{
	switch (smb358_charger_type_get(smb358_chg)){
		case CHARGER_TYPE__INVALID:
			break;
		case CHARGER_TYPE__SDP:
			smb358_charging_complete_check_usb_normal(smb358_chg);
			break;
		case CHARGER_TYPE__DCP:
			smb358_charging_complete_check_standard_normal(smb358_chg);
			break;
		case CHARGER_TYPE__NON_DCP:
			smb358_charging_complete_check_non_standard_normal(smb358_chg);
			break;
		default:
			CHG_ERR("%s:unsupported charger type!\n", __func__);

	}
}

static void smb358_charging_complete_check_standard_warm(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__WARM;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}
}

static void smb358_charging_complete_check_non_standard_warm(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__NON_STANDARD__WARM;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}
}

static void smb358_charging_complete_check_usb_warm(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_complete_chk_vol = CHG_COMPLETE_CHECK_VOLTAGE__USB__WARM;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	
	if (bat_vol > chg_complete_chk_vol)//sjc0821
		smb358_charging_complete_request_set(smb358_chg, true);
	return;
	
	if (bat_vol > chg_complete_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_COMPLETE_CHECK_COUNT){
		count = 0;
		smb358_charging_complete_request_set(smb358_chg, true);
	}
}

static void smb358_charging_complete_check_warm(struct smb358_charger *smb358_chg)
{
	switch (smb358_charger_type_get(smb358_chg)){
		case CHARGER_TYPE__INVALID:
			break;
		case CHARGER_TYPE__SDP:
			smb358_charging_complete_check_usb_warm(smb358_chg);
			break;
		case CHARGER_TYPE__DCP:
			smb358_charging_complete_check_standard_warm(smb358_chg);
			break;
		case CHARGER_TYPE__NON_DCP:
			smb358_charging_complete_check_non_standard_warm(smb358_chg);
			break;
		default:
			CHG_ERR("%s:unsupported charger type!\n", __func__);
	}
}

static void smb358_charging_complete_check(struct smb358_charger *smb358_chg)
{
	//sjc0821begin
	static int counts = 0;
	int i_chg = 0;
	
	i_chg = smb358_charging_current_get(smb358_chg);
	if ((smb358_battery_temperature_region_get(smb358_chg) == BATTERY_TEMP_REGION__WARM) && (i_chg < CHG_COMPLETE_CHECK__CURRENT))
		counts++;
	else if (i_chg < CHG_COMPLETE_CHECK__CURRENT - 20)//use 80mA for more full
		counts++;
	else
		counts = 0;
	if (counts < CHG_COMPLETE_CHECK_COUNT)
		return;
	if (counts > CHG_COMPLETE_CHECK_COUNT_MAX) {//reset counts
		if (smb358_battery_capacity_get(smb358_chg) == 100)
			smb358_charging_complete_request_set(smb358_chg, true);
		counts = 0;
	}
	//sjc0821end
	switch (smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			smb358_charging_complete_check_little_cold(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			smb358_charging_complete_check_cool(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			smb358_charging_complete_check_normal(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__WARM:
			smb358_charging_complete_check_warm(smb358_chg);
			break;
		default:
			return;
	}
}

static void smb358_charging_complete_handle(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	CHG_DBG("%s:battery voltage=%d\n", __func__, smb358_battery_voltage_get(smb358_chg));
	if (CHARGING_STATUS__CHARGING != smb358_charging_status_get(smb358_chg))
		return;
	if (true == smb358_charging_complete_request_get(smb358_chg)){
		ret = smb358_stop_charging(smb358_chg);
		if (ret){
			smb358_charging_status_set(smb358_chg, CHARGING_STATUS__FULL);
			smb358_charging_complete_request_set(smb358_chg, false);
		}
		smb358_stop_chg_timeout_timer(smb358_chg);//sjc1118
		return;
	}else
		smb358_charging_complete_check(smb358_chg);
}

static void smb358_charging_resume_check_standard_little_cold(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__STANDARD__LITTLE_COLD;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_non_standard_little_cold(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__NON_STANDARD__LITTLE_COLD;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_usb_little_cold(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__USB__LITTLE_COLD;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_little_cold(struct smb358_charger *smb358_chg)
{
	switch (smb358_charger_type_get(smb358_chg)){
		case CHARGER_TYPE__INVALID:
			break;
		case CHARGER_TYPE__SDP:
			smb358_charging_resume_check_usb_little_cold(smb358_chg);
			break;
		case CHARGER_TYPE__DCP:
			smb358_charging_resume_check_standard_little_cold(smb358_chg);
			break;
		case CHARGER_TYPE__NON_DCP:
			smb358_charging_resume_check_non_standard_little_cold(smb358_chg);
			break;
		default:
			CHG_ERR("%s:unsupported charger type!\n", __func__);
	}
}

static void smb358_charging_resume_check_standard_cool(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__STANDARD__COOL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_non_standard_cool(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__NON_STANDARD__COOL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_usb_cool(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__USB__COOL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_cool(struct smb358_charger *smb358_chg)
{
	switch (smb358_charger_type_get(smb358_chg)){
	case CHARGER_TYPE__INVALID:
		break;
	case CHARGER_TYPE__SDP:
		smb358_charging_resume_check_usb_cool(smb358_chg);
		break;
	case CHARGER_TYPE__DCP:
		smb358_charging_resume_check_standard_cool(smb358_chg);
		break;
	case CHARGER_TYPE__NON_DCP:
		smb358_charging_resume_check_non_standard_cool(smb358_chg);
		break;
	default:
		CHG_ERR("%s:unsupported charger type!\n", __func__);
	}
}

static void smb358_charging_resume_check_standard_normal(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__STANDARD__NORMAL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_non_standard_normal(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__NON_STANDARD__NORMAL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_usb_normal(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__USB__NORMAL;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_normal(struct smb358_charger *smb358_chg)
{
	switch (smb358_charger_type_get(smb358_chg)){
	case CHARGER_TYPE__INVALID:
		break;
	case CHARGER_TYPE__SDP:
		smb358_charging_resume_check_usb_normal(smb358_chg);
		break;
	case CHARGER_TYPE__DCP:
		smb358_charging_resume_check_standard_normal(smb358_chg);
		break;
	case CHARGER_TYPE__NON_DCP:
		smb358_charging_resume_check_non_standard_normal(smb358_chg);
		break;
	default:
		CHG_ERR("%s:unsupported charger type!\n", __func__);
	}
}

static void smb358_charging_resume_check_standard_warm(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__STANDARD__WARM;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_non_standard_warm(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__NON_STANDARD__WARM;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_usb_warm(struct smb358_charger *smb358_chg)
{
	static int count = 0;
	int chg_resume_chk_vol = CHG_RESUME_CHECK_VOLTAGE__USB__WARM;
	int bat_vol = smb358_battery_voltage_get(smb358_chg);
	if (bat_vol < chg_resume_chk_vol)
		count++;
	else
		count = 0;
	if (count >= CHG_RESUME_CHECK_COUNT){
		count = 0;
		smb358_charging_resume_request_set(smb358_chg, true);
	}
}

static void smb358_charging_resume_check_warm(struct smb358_charger *smb358_chg)
{
	switch (smb358_charger_type_get(smb358_chg)){
	case CHARGER_TYPE__INVALID:
		break;
	case CHARGER_TYPE__SDP:
		smb358_charging_resume_check_usb_warm(smb358_chg);
		break;
	case CHARGER_TYPE__DCP:
		smb358_charging_resume_check_standard_warm(smb358_chg);
		break;
	case CHARGER_TYPE__NON_DCP:
		smb358_charging_resume_check_non_standard_warm(smb358_chg);
		break;
	default:
		CHG_ERR("%s:unsupported charger type!\n", __func__);
	}

}

static void smb358_charging_resume_check(struct smb358_charger *smb358_chg)
{
	CHG_DBG("%s:battery voltage =%d\n", __func__, smb358_battery_voltage_get(smb358_chg));
	if (CHARGING_STATUS__FULL != smb358_charging_status_get(smb358_chg))
		return;
	switch (smb358_battery_temperature_region_get(smb358_chg)){
		case BATTERY_TEMP_REGION__COLD:
		case BATTERY_TEMP_REGION__HOT:
			break;
		case BATTERY_TEMP_REGION__LITTLE_COLD:
			smb358_charging_resume_check_little_cold(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__COOL:
			smb358_charging_resume_check_cool(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__NORMAL:
			smb358_charging_resume_check_normal(smb358_chg);
			break;
		case BATTERY_TEMP_REGION__WARM:
			smb358_charging_resume_check_warm(smb358_chg);
			break;
		default:
			return;
	}
}

static void smb358_charging_resume_handle(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	if (smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__INVALID || smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__OTG)//sjc1010
		return;
	if (CHARGING_STATUS__CHARGING == smb358_charging_status_get(smb358_chg))
		return;
	if (true == smb358_charging_resume_request_get(smb358_chg)){
		ret = smb358_start_charging(smb358_chg);
		if (ret){
			smb358_charging_resume_request_set(smb358_chg, false);
		}
		return;
	}else
		smb358_charging_resume_check(smb358_chg);

}

static void smb358_charging_timeout_handle(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	if (true == smb358_charging_timeout_get(smb358_chg)){
		ret = smb358_stop_charging(smb358_chg);
		if (false == ret )
			CHG_ERR("%s:trying to stop charging failed\n", __func__);
		smb358_charging_timeout_status_set(smb358_chg, true);//sjc0820
	}
}

static void sm3b58_irq_flags_handle(struct smb358_charger *smb358_chg)
{
/*battery missing set*/
	if (smb358_bit_test(smb358_chg, IRQ_FLAGS__BAT_MISSING)){
		smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__MISSED);
		smb358_bit_clear(smb358_chg, IRQ_FLAGS__BAT_MISSING);
	}
/*charging complete set*/
	if (smb358_bit_test(smb358_chg, IRQ_FLAGS__CHG_COMPLETE)){
		if (smb358_charging_current_get(smb358_chg) < CHG_COMPLETE_CHECK__CURRENT + 50) {//sjc0930 to avoid the condition that press power key for 7s
			if ((smb358_battery_temperature_get(smb358_chg) > BAT_TEMP__ZERO
					&& smb358_battery_temperature_get(smb358_chg) < BAT_TEMP__TEN
					&& smb358_battery_voltage_get(smb358_chg) > CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__COOL - 30)
					|| (smb358_battery_temperature_get(smb358_chg) >= BAT_TEMP__TEN
					&& smb358_battery_temperature_get(smb358_chg) < BAT_TEMP__FORTY_FIVE
					&& smb358_battery_voltage_get(smb358_chg) > CHG_COMPLETE_CHECK_VOLTAGE__STANDARD__NORMAL - 30))
				smb358_charging_complete_request_set(smb358_chg, true);
		}
		//smb358_charging_complete_request_set(smb358_chg, true);
		CHG_DBG("===%s:charging complete..., Ichg=%dmA, Vbatt=%dmV===\n", __func__, smb358_charging_current_get(smb358_chg), smb358_battery_voltage_get(smb358_chg));
		smb358_bit_clear(smb358_chg, IRQ_FLAGS__CHG_COMPLETE);
	}
/*charging resume set*/
	if (smb358_bit_test(smb358_chg, IRQ_FLAGS__CHG_RESUME)){
		smb358_charging_resume_request_set(smb358_chg, true);
		smb358_bit_clear(smb358_chg, IRQ_FLAGS__CHG_RESUME);
	}
/*charging mode set*/
	if (smb358_bit_test(smb358_chg, IRQ_FLAGS__NO_CHG)){
		smb358_charging_mode_set(smb358_chg, CHARGING_MODE__INVALID);
		smb358_bit_clear(smb358_chg, IRQ_FLAGS__NO_CHG);
	}else if (smb358_bit_test(smb358_chg, IRQ_FLAGS__CHG_MODE_PRE)){
		smb358_charging_mode_set(smb358_chg, CHARGING_MODE__PRE_CHG);
		smb358_bit_clear(smb358_chg, IRQ_FLAGS__CHG_MODE_PRE);
	}
	else if (smb358_bit_test(smb358_chg, IRQ_FLAGS__CHG_MODE_FAST)){
		smb358_charging_mode_set(smb358_chg, CHARGING_MODE__FAST_CHG);
		smb358_bit_clear(smb358_chg, IRQ_FLAGS__CHG_MODE_FAST);
	}
	else if (smb358_bit_test(smb358_chg, IRQ_FLAGS__CHG_MODE_TAPER)){
		smb358_charging_mode_set(smb358_chg, CHARGING_MODE__TAPER_CHG);
		smb358_bit_clear(smb358_chg, IRQ_FLAGS__CHG_MODE_TAPER);
	}
/*charging timeout set*/
	if (smb358_bit_test(smb358_chg, IRQ_FLAGS__CHG_TIMEOUT)) {//sjc0820
		smb358_charging_timeout_set(smb358_chg, true);
		smb358_bit_clear(smb358_chg, IRQ_FLAGS__CHG_TIMEOUT);
	}

	//more code add here.	
}

static void smb358_power_supply_notify(struct smb358_charger *smb358_chg)
{
	switch(smb358_charger_type_get(smb358_chg)){
		case CHARGER_TYPE__NON_DCP:
		case CHARGER_TYPE__DCP:
			power_supply_changed(smb358_chg->power_supplies.mains);
			power_supply_changed(smb358_chg->power_supplies.battery);
			break;
		case CHARGER_TYPE__SDP:
			power_supply_changed(smb358_chg->power_supplies.usb);
			power_supply_changed(smb358_chg->power_supplies.battery);
			break;
		case CHARGER_TYPE__INVALID:
		default:
			power_supply_changed(smb358_chg->power_supplies.battery);
			break;
	}
}

static void smb358_dump_registers(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	u8 value = 0;
	int addr = 0;

	if (false == smb358_debug_dump_registers_enabled_get(smb358_chg))
		return;
	for (addr = 0; addr <= 0xf; addr++){
		ret = smb358_register_read(smb358_chg->client, addr, &value);
		if (ret){
			CHG_ERR("%s:register addr=0x%x, value=0x%x\n", __func__, addr, value);
		}else{
			CHG_ERR("%s:register addr=0x%x read failed\n", __func__, addr);
		}
	}
	for (addr = 0x30; addr <= 0x3f; addr++){
		ret = smb358_register_read(smb358_chg->client, addr, &value);
		if (ret){
			CHG_ERR("%s:register addr=0x%x, value=0x%x\n", __func__, addr, value);
		}else{
			CHG_ERR("%s:register addr=0x%x read failed\n", __func__, addr);
		}
	}
	CHG_ERR("\n");
}

static void smb358_dump_status(struct smb358_charger *smb358_chg)
{
	CHG_DBG("===============dump===============\n");
	CHG_DBG("%s:charger type=%s\n", __func__, charger_type_str[smb358_charger_type_get(smb358_chg)]);
	CHG_DBG("%s:charger status=%s\n", __func__, charger_status_str[smb358_charger_status_get(smb358_chg)]);
	CHG_DBG("%s:charger voltage=%d\n", __func__, smb358_charger_voltage_get(smb358_chg));
	CHG_DBG("%s:charging status=%s\n", __func__, charging_status_str[smb358_charging_status_get(smb358_chg)]);
	CHG_DBG("%s:charging mode=%s\n", __func__, charging_mode_str[smb358_charging_mode_get(smb358_chg)]);
	
	CHG_DBG("\n");
	CHG_DBG("%s:battery voltage=%d\n", __func__, smb358_battery_voltage_get(smb358_chg));
	CHG_DBG("%s:battery status=%s\n", __func__, battery_status_str[smb358_battery_status_get(smb358_chg)]);
	CHG_DBG("%s:battery temperature=%d\n", __func__, smb358_battery_temperature_get(smb358_chg));
	CHG_DBG("%s:battery temperature region=%s\n", __func__, battery_temp_region_str[smb358_battery_temperature_region_get(smb358_chg)]);
	CHG_DBG("%s:battery critical tempearture:(cold<%d), [%d<=little cold<%d), [%d<=cool<%d), [%d<=normal<%d), [%d<=warm<%d), (hot>%d)\n", 
		__func__, smb358_cold_to_little_cold_critical_temperature_get(smb358_chg),
		smb358_cold_to_little_cold_critical_temperature_get(smb358_chg), smb358_little_cold_to_cool_critical_temperature_get(smb358_chg),
		smb358_little_cold_to_cool_critical_temperature_get(smb358_chg), smb358_cool_to_normal_critical_temperature_get(smb358_chg),
		smb358_cool_to_normal_critical_temperature_get(smb358_chg), smb358_normal_to_warm_critical_temperature_get(smb358_chg),
		smb358_normal_to_warm_critical_temperature_get(smb358_chg), smb358_warm_to_hot_critical_temperature_get(smb358_chg),
		smb358_warm_to_hot_critical_temperature_get(smb358_chg));
	CHG_DBG("%s:battery missing status=%s\n", __func__, battery_missing_status_str[smb358_battery_missing_status_get(smb358_chg)]);

	CHG_DBG("\n");
	CHG_DBG("%s:charger suspended:%s\n", __func__, smb358_suspended_get(smb358_chg)? "true": "false");
	CHG_DBG("%s:battery charging current=%d\n", __func__, smb358_charging_current_get(smb358_chg));
	CHG_DBG("%s:battery capacity=%d%%\n", __func__, smb358_battery_capacity_get(smb358_chg));
	CHG_DBG("\n");

	//CHG_DBG("===%s:get_battery_capacity=%d===\n", __func__, batt_fuelgauge->get_battery_capacity());	
	//CHG_DBG("===%s:get_battery_mvolts=%d===\n", __func__, batt_fuelgauge->get_battery_mvolts());
	//CHG_DBG("===%s:get_battery_temperature=%d===\n", __func__, batt_fuelgauge->get_battery_temperature()/10);
	//CHG_DBG("===%s:get_ichg_current=%d===\n", __func__, batt_fuelgauge->get_ichg_current());
	//CHG_DBG("===\n\n");
}

static void smb358_dumps(struct smb358_charger *smb358_chg)
{
	smb358_dump_status(smb358_chg);
	smb358_dump_registers(smb358_chg);
}

static void smb358_power_supply_update(struct smb358_charger *smb358_chg)
{
	mutex_lock(&smb358_chg->lock);
/*check charging mode fisrt*/
	smb358_charging_mode_check_and_set(smb358_chg);
/*data read only this place*/
	smb358_power_supply_data_read(smb358_chg);
/*irq data handle*/
	sm3b58_irq_flags_handle(smb358_chg);
/*charger voltage handle*/
	smb358_charger_voltage_handle(smb358_chg);
/*battery missing check*/
	smb358_battery_missing_handle(smb358_chg);
/*battery voltage handle*/
	smb358_battery_voltage_handle(smb358_chg);
/*battery temp handle*/
	smb358_battery_temperature_handle(smb358_chg);
/*software charging done handle*/
	smb358_charging_complete_handle(smb358_chg);
/*charging resume handle*/
	smb358_charging_resume_handle(smb358_chg);
/*charging timeout*/
	smb358_charging_timeout_handle(smb358_chg);
/*charger aicl handle*/
	smb358_aicl_handle(smb358_chg);//sjc0824
/*charging current aicl handle*/
	smb358_charging_current_aicl_handle(smb358_chg);//sjc1003
/*nofify system*/
	smb358_power_supply_notify(smb358_chg);
/*dump status.*/
	smb358_dumps(smb358_chg);

	mutex_unlock(&smb358_chg->lock);
}

static void smb358_update_work(struct work_struct *work)
{
	struct smb358_charger *smb358_chg = container_of(to_delayed_work(work), struct smb358_charger, update_work);
	smb358_power_supply_update(smb358_chg);
	queue_delayed_work(smb358_chg->work_queue,&smb358_chg->update_work, round_jiffies_relative(msecs_to_jiffies(POWER_SUPPLY_UPDATE_PERIOD)));
}
/*for works define end*/

/*for  charger status  functons begin*/
static smb358_charger_status smb358_charger_status_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.charger_status;
}

static void smb358_charger_status_set(struct smb358_charger *smb358_chg, smb358_charger_status  chg_stat)
{
	smb358_chg->power_supply_data.charger_status = chg_stat;
}

static smb358_charger_type smb358_charger_type_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.charger_type;
}

static void smb358_charger_type_set(struct smb358_charger *smb358_chg, smb358_charger_type chg_type)
{
	smb358_chg->power_supply_data.charger_type = chg_type;
}

static int smb358_charger_voltage_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.charger_voltage;
}

static void smb358_charger_voltage_set(struct smb358_charger *smb358_chg, int chg_vol)
{
	smb358_chg->power_supply_data.charger_voltage = chg_vol;
}

static bool smb358_usb_online_get(struct smb358_charger *smb358_chg)
{
	smb358_charger_type chg_type = smb358_charger_type_get(smb358_chg);
	if (CHARGER_TYPE__SDP == chg_type)
		return true;
	else
		return false;
}

static bool smb358_ac_online_get(struct smb358_charger *smb358_chg)
{
	smb358_charger_type chg_type = smb358_charger_type_get(smb358_chg);
	if (CHARGER_TYPE__DCP ==  chg_type || CHARGER_TYPE__NON_DCP == chg_type)
		return true;
	else
		return false;
}

static smb358_charging_status smb358_charging_status_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.charging_status;
}

static void smb358_charging_status_set(struct smb358_charger *smb358_chg, smb358_charging_status chg_stat)
{
	 smb358_chg->power_supply_data.charging_status = chg_stat;
}

static int smb358_charging_current_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.charging_current;
}

static bool smb358_charging_current_set(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_charger_type_get(smb358_chg)){
		case CHARGER_TYPE__SDP:
			ret = smb358_charging_current_set__usb(smb358_chg);
			break;
		case CHARGER_TYPE__DCP:
			ret = smb358_charging_current_set__standard(smb358_chg);
			break;
		case CHARGER_TYPE__NON_DCP:
			ret = smb358_charging_current_set__non_standard(smb358_chg);
			break;
		case CHARGER_TYPE__CDP:
		case CHARGER_TYPE__ACA:
			CHG_ERR("%s:not supportted yet\n", __func__);
			break;
		case CHARGER_TYPE__INVALID:
			CHG_ERR("%s:invalid charger type\n", __func__);
			break;
		default:
			break;
	}
	return ret;
}

static smb358_charging_mode smb358_charging_mode_get(struct smb358_charger *smb358_chg)
{	
	return smb358_chg->power_supply_data.charging_mode;
}

static void smb358_charging_mode_set(struct smb358_charger *smb358_chg, smb358_charging_mode chg_mode)
{
	smb358_chg->power_supply_data.charging_mode = chg_mode;
}

static bool smb358_charging_complete_request_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.charging_complete_request;
}

static void smb358_charging_complete_request_set(struct smb358_charger *smb358_chg, bool value)
{
	smb358_chg->power_supply_data.charging_complete_request = value;
}

static bool smb358_charging_resume_request_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.charging_resume_request;
}

static void smb358_charging_resume_request_set(struct smb358_charger *smb358_chg, bool value)
{
	smb358_chg->power_supply_data.charging_resume_request = value;
}

static bool smb358_charging_timeout_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.charging_timeout;
}

static void smb358_charging_timeout_set(struct smb358_charger *smb358_chg, bool val)
{
	smb358_chg->power_supply_data.charging_timeout = val;
}

static int smb358_cold_to_little_cold_critical_temperature_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.critical_temperature_cold_to_little_cold;
}

static void smb358_cold_to_little_cold_critical_temperature_set(struct smb358_charger *smb358_chg, int val)
{
	smb358_chg->power_supply_data.critical_temperature_cold_to_little_cold= val;
}

static int smb358_little_cold_to_cool_critical_temperature_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.critical_temperature_little_cold_to_cool;
}

static void smb358_little_cold_to_cool_critical_temperature_set(struct smb358_charger *smb358_chg, int val)
{
	smb358_chg->power_supply_data.critical_temperature_little_cold_to_cool= val;
}

static int smb358_cool_to_normal_critical_temperature_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.critical_temperature_cool_to_normal;
}

static void smb358_cool_to_normal_critical_temperature_set(struct smb358_charger *smb358_chg, int val)
{
	smb358_chg->power_supply_data.critical_temperature_cool_to_normal= val;
}

static int smb358_normal_to_warm_critical_temperature_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.critical_temperature_normal_to_warm;
}

static void smb358_normal_to_warm_critical_temperature_set(struct smb358_charger *smb358_chg, int val)
{
	smb358_chg->power_supply_data.critical_temperature_normal_to_warm = val;
}

static int smb358_warm_to_hot_critical_temperature_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.critical_temperature_warm_to_hot;
}

static void smb358_warm_to_hot_critical_temperature_set(struct smb358_charger *smb358_chg, int val)
{
	smb358_chg->power_supply_data.critical_temperature_warm_to_hot = val;
}
/*charger status get set funcs end*/

/*battery status get set funcs begin)*/
static smb358_battery_status smb358_battery_status_get(struct smb358_charger *smb358_chg)
{
	return 	smb358_chg->power_supply_data.battery_status;
}

static void smb358_battery_status_set(struct smb358_charger *smb358_chg, smb358_battery_status bat_stat)
{
	smb358_chg->power_supply_data.battery_status = bat_stat;
}

static smb358_battery_missing_status smb358_battery_missing_status_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.battery_missing_status;
}

static void smb358_battery_missing_status_set(struct smb358_charger *smb358_chg, smb358_battery_missing_status bat_missing_stat)
{
	smb358_chg->power_supply_data.battery_missing_status = bat_missing_stat;
}

static bool smb358_battery_present_get(struct smb358_charger *smb358_chg)
{
	if (BATTERY_MISSING_STATUS__MISSED == smb358_battery_missing_status_get(smb358_chg))
		return false;
	else
		return true;
}

static int smb358_battery_voltage_max_designed_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->battery_platform_data.battery_max_designed_voltage;
}

static int smb358_battery_voltage_min_designed_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->battery_platform_data.battery_min_designed_voltage;
}

static int smb358_battery_voltage_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.battery_voltage;
}

static void smb358_battery_voltage_set(struct smb358_charger *smb358_chg, int bat_vol)
{
	smb358_chg->power_supply_data.battery_voltage = bat_vol;
}

static int smb358_battery_capacity_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.battery_capacity;
}

static void smb358_battery_capacity_set(struct smb358_charger *smb358_chg, int bat_cap)
{
	smb358_chg->power_supply_data.battery_capacity = bat_cap;
}

static int smb358_battery_temperature_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.battery_temperature;
}

static void smb358_battery_temperature_set(struct smb358_charger *smb358_chg, int bat_temp)
{
	smb358_chg->power_supply_data.battery_temperature = bat_temp;
}

static smb358_battery_temperature_region smb358_battery_temperature_region_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.battery_temperature_region;
}

static void smb358_battery_temperature_region_set(struct smb358_charger *smb358_chg, smb358_battery_temperature_region bat_temp_region)
{
	smb358_chg->power_supply_data.battery_temperature_region = bat_temp_region;
}
/*battery status get set funcs end*/

 void smb358_charger_connected(smb358_charger_type chg_type)
{
	smb358_charger_type pre_chg_type;
	
	CHG_ERR("%s:charger type is:%s\n", __func__, charger_type_str[chg_type]);

	if (NULL == the_smb358_charger){
		CHG_ERR("%s:smb358 charger is not initialized yet\n", __func__);
		startup_charger_type = chg_type;
		return;
	}
	pre_chg_type = smb358_charger_type_get(the_smb358_charger);
	if (pre_chg_type == chg_type)//sjc1104
		return;
	smb358_charger_type_set(the_smb358_charger, chg_type);
	if (chg_type == CHARGER_TYPE__INVALID) {
		if (pre_chg_type == CHARGER_TYPE__DCP || pre_chg_type == CHARGER_TYPE__NON_DCP)
			power_supply_changed(the_smb358_charger->power_supplies.mains);
		else if (pre_chg_type == CHARGER_TYPE__SDP)
			power_supply_changed(the_smb358_charger->power_supplies.usb);
	}
	if (CHARGER_TYPE__OTG == smb358_charger_type_get(the_smb358_charger)) {//sjc0808otg
		smb358_otg_enable(the_smb358_charger);
		wake_lock(&the_smb358_charger->wlock);
		return;
	}
	if (CHARGER_TYPE__INVALID == smb358_charger_type_get(the_smb358_charger)){
		if (smb358_otg_status_get(the_smb358_charger)) {//sjc0808otg
			smb358_otg_disable(the_smb358_charger);
			wake_unlock(&the_smb358_charger->wlock);
			return;
		}
		if (is_ftm_mode())//sjc0823
			return;
		cancel_delayed_work_sync(&the_smb358_charger->start_charging_work);
		if (smb358_suspended_get(the_smb358_charger))//sjc0806
			smb358_unsuspend_charger(the_smb358_charger);
		smb358_stop_charging(the_smb358_charger);
		smb358_stop_chg_timeout_timer(the_smb358_charger);//sjc0820
		if (smb358_charging_timeout_status_get(the_smb358_charger))//sjc0820
			smb358_charging_timeout_status_set(the_smb358_charger, false);
		smb358_aicl_status_set(the_smb358_charger, false);//sjc0824
		smb358_charging_current_aicl_status_set(the_smb358_charger, false);//sjc1003
		smb358_input_current_limit_standard_set(the_smb358_charger);//sjc0812 for AICL
		wake_lock_timeout(&the_smb358_charger->prevent_wlock, HZ);//sjc0823
		wake_unlock(&the_smb358_charger->wlock);
	}else{
		wake_lock(&the_smb358_charger->wlock);
		current_aicl_counts = 0;//sjc1003
		ovp_counts = 0;//sjc0806
		uvp_counts = 0;
		in_ovp_status = false;
		in_uvp_status = false;
		queue_delayed_work(the_smb358_charger->work_queue, &the_smb358_charger->start_charging_work, 0);//msecs_to_jiffies(START_CHG_DELAY));//sjc1023
	}
}

/*for power supplies*/
static enum power_supply_property smb358_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *smb358_power_supplied_to[] = {
	"battery",
};

static int smb358_power_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = smb358_ac_online_get(the_smb358_charger) ? 1 : 0;
		}
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = smb358_usb_online_get(the_smb358_charger) ? 1 : 0;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply smb358_power_supply_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = smb358_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(smb358_power_supplied_to),
	.properties = smb358_power_props,
	.num_properties = ARRAY_SIZE(smb358_power_props),
	.get_property = smb358_power_get_property,
};

static struct power_supply smb358_power_supply_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = smb358_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(smb358_power_supplied_to),
	.properties = smb358_power_props,
	.num_properties = ARRAY_SIZE(smb358_power_props),
	.get_property = smb358_power_get_property,
};

static enum power_supply_property smb358_battery_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_TIMEOUT,
};

static int smb358_battery_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb358_charging_status_get(the_smb358_charger);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (!smb358_battery_present_get(the_smb358_charger))//sjc0814
			val->intval = BATTERY_STATUS__INVALID;
		else
			val->intval = smb358_battery_status_get(the_smb358_charger);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb358_battery_present_get(the_smb358_charger);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = smb358_battery_voltage_max_designed_get(the_smb358_charger);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = smb358_battery_voltage_min_designed_get(the_smb358_charger);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb358_battery_voltage_get(the_smb358_charger) * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (is_ftm_mode()) {//sjc0823
			val->intval = 30;
			break;
		}
		if ((smb358_battery_temperature_get(the_smb358_charger) > BAT_TEMP__ZERO)//sjc0813
			&& (smb358_battery_temperature_get(the_smb358_charger) < BAT_TEMP__FORTY_FIVE)) {
			if (smb358_charging_status_get(the_smb358_charger) == CHARGING_STATUS__FULL) {
				set_battery_capacity_full();
				val->intval = 100;
			}
			val->intval = smb358_battery_capacity_get(the_smb358_charger);
		} else
			val->intval = smb358_battery_capacity_get(the_smb358_charger);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = smb358_charger_voltage_get(the_smb358_charger);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = smb358_charging_current_get(the_smb358_charger);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb358_battery_temperature_get(the_smb358_charger) * 10;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TIMEOUT://sjc0820
		val->intval = smb358_charging_timeout_status_get(the_smb358_charger);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply smb358_power_supply_battery = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = smb358_battery_power_props,
	.num_properties = ARRAY_SIZE(smb358_battery_power_props),
	.get_property = smb358_battery_power_get_property,
};

/*for init functions begin*/
static void smb358_power_supply_data_init(struct smb358_charger *smb358_chg)
{
	smb358_charger_type_set(smb358_chg, CHARGER_TYPE__INVALID);
	smb358_charger_status_set(smb358_chg, CHARGER_STATUS__INVALID);
	smb358_charging_status_set(smb358_chg, CHARGING_STATUS__DISCHARGING);//CHARGING_STATUS__INVALID);

	smb358_battery_status_set(smb358_chg, BATTERY_STATUS__INVALID);
	smb358_battery_missing_status_set(smb358_chg, BATTERY_MISSING_STATUS__INVALID);
	smb358_battery_temperature_region_set(smb358_chg, BAT_TEMP_REGION__DEFAULT);
	smb358_battery_temperature_set(smb358_chg, BAT_TEMP__DEFAULT);

	smb358_charging_complete_request_set(smb358_chg, false);
	smb358_charging_resume_request_set(smb358_chg, false);
	smb358_charging_mode_set(smb358_chg, CHARGING_MODE__FAST_CHG);
	//add code here to initialize charger voltage,battery voltage, 
	smb358_charger_voltage_set(smb358_chg, CHARGER_VOL__DEFAULT);
	smb358_battery_voltage_set(smb358_chg, BAT_VOL__DEFAULT);
	smb358_charging_timeout_set(smb358_chg, false);
	smb358_charging_timeout_status_set(smb358_chg, false);//sjc0820
	//smb358_max_chg_timeout_flag_set(smb358_chg, false);
	smb358_aicl_status_set(smb358_chg, false);//sjc0824
	smb358_charging_current_aicl_status_set(smb358_chg, false);//sjc1003
	smb358_otg_status_set(smb358_chg, false);//sjc1023

	smb358_cold_to_little_cold_critical_temperature_set(smb358_chg, BAT_TEMP__TEN_BELOW_ZERO);
	smb358_little_cold_to_cool_critical_temperature_set(smb358_chg, BAT_TEMP__ZERO);
	smb358_cool_to_normal_critical_temperature_set(smb358_chg, BAT_TEMP__TEN);
	smb358_normal_to_warm_critical_temperature_set(smb358_chg, BAT_TEMP__FORTY_FIVE);
	smb358_warm_to_hot_critical_temperature_set(smb358_chg, BAT_TEMP__FIFTY_FIVE);
}

static bool smb358_power_supplies_init(struct smb358_charger *smb358_chg)
{
	int rc=-1;

	smb358_chg->power_supplies.mains = &smb358_power_supply_ac;
	smb358_chg->power_supplies.usb = &smb358_power_supply_usb;
	smb358_chg->power_supplies.battery = &smb358_power_supply_battery;

	rc = power_supply_register(&smb358_chg->client->dev, smb358_chg->power_supplies.mains);
	if (rc){
		CHG_ERR("register power supply mains failed\n");
		return false;
	}
	rc = power_supply_register(&smb358_chg->client->dev, smb358_chg->power_supplies.usb);
	if (rc){
		CHG_ERR("register power supply usb failed\n");
		power_supply_unregister(smb358_chg->power_supplies.mains);
		return false;
	}
	rc = power_supply_register(&smb358_chg->client->dev, smb358_chg->power_supplies.battery);
	if (rc){
		CHG_ERR("register power supply battery failed\n");
		power_supply_unregister(smb358_chg->power_supplies.mains);
		power_supply_unregister(smb358_chg->power_supplies.usb);
		return false;
	}
	return true;
}

static void smb358_power_supplies_deinit(struct smb358_charger *smb358_chg)
{
	power_supply_unregister(smb358_chg->power_supplies.mains);
	power_supply_unregister(smb358_chg->power_supplies.usb);
	power_supply_unregister(smb358_chg->power_supplies.battery);
}

static void smb358_charger_platform_data_init(struct smb358_charger *smb358_chg, const struct smb358_charger_platform_data *pdata)
{
	smb358_chg->charger_platform_data.irq_gpio = pdata->irq_gpio;
	smb358_chg->charger_platform_data.max_charge_current = pdata->max_charge_current;
//add code here
}

static void smb358_charger_platform_data_deinit(struct smb358_charger *smb358_chg)
{
	smb358_chg->charger_platform_data.irq_gpio = 0;
	smb358_chg->charger_platform_data.max_charge_current = 0;
}

void smb358_battery_platform_data_init(struct smb358_charger *smb358_chg)
{
	smb358_chg->battery_platform_data.battery_max_designed_voltage = BAT_MAX_DESIGNED_VOLTAGE;
	smb358_chg->battery_platform_data.battery_min_designed_voltage = BAT_MIN_DESIGNED_VOLTAGE;
//if any data,add code here
}

static void smb358_battery_platform_data_deinit(struct smb358_charger *smb358_chg)
{
	return;
}

static bool smb358_float_voltage_write(struct smb358_charger *smb358_chg, int flt_vol)
{
	bool rc = false;
	u8 val = 0;
	val = (flt_vol - FLOAT_VOLTAGE__BASE_VOLTAGE_MV) / FLOAT_VOLTAGE__STEP_MV;
	CHG_DBG("%s:float voltage to be set is %d,register value is %d\n", __func__, flt_vol, val);
	rc = smb358_register_masked_write(smb358_chg->client,FLOAT_VOLTAGE_REGISTER_ADDR,
		FLOAT_VOLTAGE__MASK, val);
	if (false == rc){
		CHG_ERR("%s:set float voltage failed\n", __func__);
	}
	return rc;
}

static bool smb358_input_current_limit_write(struct smb358_charger *smb358_chg,  u8 value)
{
	bool ret = false;
	if (value <= 0)
		value = MAX_CHG_CURRENT__500MA;
	if (value > MAX_CHG_CURRENT__2000MA)
		value = MAX_CHG_CURRENT__2000MA;
	ret = smb358_register_masked_write(smb358_chg->client, INPUT_CURRENT_LIMIT_REGISTER_ADDR,
		INPUT_CURRENT_LIMIT__MAX_CHG_CURERNT_MASK, value);
	if (false == ret){
		CHG_ERR("%s:set input current limit failed\n", __func__);
	}
	return ret;
}

static bool smb358_input_current_limit_standard_set(struct smb358_charger *smb358_chg)
{
	if (!smb358_early_suspend_status_get(smb358_chg))//sjc0927
		return smb358_input_current_limit_write(smb358_chg, MAX_INPUT_CURRENT_LIMIT__STANDARD_SUS);
	return smb358_input_current_limit_write(smb358_chg, MAX_INPUT_CURRENT_LIMIT__STANDARD);//sjc0927//MAX_CHG_CURRENT__2000MA);
}

static bool smb358_input_current_limit_non_standard_set(struct smb358_charger *smb358_chg)
{
	return smb358_input_current_limit_write(smb358_chg, MAX_CHG_CURRENT__500MA);
}

static bool smb358_input_current_limit_usb_set(struct smb358_charger *smb358_chg)
{
	return smb358_input_current_limit_write(smb358_chg, MAX_CHG_CURRENT__500MA);
}

static bool smb358_input_current_limit_set(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	switch(smb358_charger_type_get(smb358_chg)){
		case CHARGER_TYPE__SDP:
			ret = smb358_input_current_limit_usb_set(smb358_chg);
			break;
		case CHARGER_TYPE__DCP:
			ret = smb358_input_current_limit_standard_set(smb358_chg);
			break;
		case CHARGER_TYPE__NON_DCP:
			ret = smb358_input_current_limit_non_standard_set(smb358_chg);
			break;
		case CHARGER_TYPE__INVALID:
		default:
			ret = true;
			break;
	}
	return ret;
}

static bool smb358_hardware_init(struct smb358_charger *smb358_chg)
{
	bool rc = false;
	u8 value = 0;
/* 0x30h: enable write permission to config registers*/
	rc = smb358_register_masked_write(smb358_chg->client, CMD_REG_A_REGISTER_ADDR, 
		CONFIG_REG_WRITE_PERMISSION__MASK, CONFIG_REG_WRITE_PERMISSION__ALLOW);
	if(false == rc){
		CHG_ERR("%s:enable write permission failed\n", __func__);
		return false;
	}	
/* 0x30h: set command A register*/
	value = AD_CONVERTER_CONTROL__ENABLE | FAST_CHG_SETTING__ALOW_FAST_CHG_CURRENT_SETTINGS
		| SUSPEND_MODE_CONTROL__DISABLE;
	rc = smb358_register_masked_write(smb358_chg->client, CMD_REG_A_REGISTER_ADDR, 
		FAST_CHG_SETTING__MASK | AD_CONVERTER_CONTROL___MASK | SUSPEND_MODE_CONTROL__MASK , value);
	if(false == rc){
		CHG_ERR("%s:set command a register failed\n", __func__);
		return false;
	}
/* 0x31: command register B */
	value = USB_HC_MODE_SELECT__HC;
	rc = smb358_register_masked_write(smb358_chg->client, CMD_REG_B_REGISTER_ADDR,
		USB_HC_MODE_SELECT__MASK, value);
	if (false == rc){
		CHG_ERR("%s: set command register B failed\n", __func__);
	}
/* 0x00h: set pre charging and term charging current*/
	value = PRE_CHG_CURRENT__450MA | TERM_CHG_CURRENT__100MA;//sjc1003 100MA->150MA
	rc = smb358_register_masked_write(smb358_chg->client, CHG_CURRENT_REGISTER_ADDR,
		CHG_CURRENT__PRE_CHG_MASK |CHG_CURRENT__TERM_CHG_MASK , value);
	if (false == rc){
		CHG_ERR("%s: set pre-charging and term charging current failed\n", __func__);
	}
/* 0x01h: max input current limit*/
	rc = smb358_input_current_limit_set(smb358_chg);
	if (false == rc){
		CHG_ERR("%s:set input current limit failed\n", __func__);
	}
/* 0x02h: Various functions*/
	value = SUSPEND_ON_OFF_CONTROL__BY_REGISTER;//sjc0806
	rc = smb358_register_masked_write(smb358_chg->client, VARIOUS_FUNCTIONS_REGISTER_ADDR,
		SUSPEND_ON_OFF_CONTROL__MASK , value);
	if (false == rc){
		CHG_ERR("%s: set suspend register control failed\n", __func__);
	}
/* 0x03h: float voltage set */
	rc = smb358_float_voltage_set(smb358_chg);
	if (false == rc){
		CHG_ERR("%s:set float voltage failed\n", __func__);
	}
/* 0x04h: set charge control register*/
	value = AUTO_RECHARGE_CONTROL__ENABLE | CURRENT_TERM_CONTROl__ALLOW_TO_END_CHG
		| BATTERY_MISSING_DETECT__VIA_INTERNAL_BDM_ALGORITHM | BATTERYGOOD_SYSTEMOK_INOK_OUTPUT__PUSH_PULL
		| AUTO_POWER_SOURCE_DETECT_CONTROL__ENABLE | AICL_BEHAVIOR_CONTROL__CHG_DISABLE_INPUT_FET_OPEN
		| AICL_RISING_EDGE_GLITH_FILTER_DURATION__20MS;
	rc = smb358_register_masked_write(smb358_chg->client, CHG_CONTROL_REGISTER_ADDR,
		CHG_CONTROL_REG__MASK , value);
	if (false == rc){
		CHG_ERR("%s: set charge control register failed\n", __func__);
	}
/* 0x05h: set status and timer control register*/
	value = STAT_OUTPUT_POLARITY__ACTIVE_LOW | STAT_OUTPUT_MODE__INDICATE_CHG_STAT
	| STAT_OUTPUT__DISABLE | OTHER_CHARGER_INPUT_CURRENT_LIMIT__HC | COMPLETE_CHG_TIMEOUT__764_MINUTES
	| PRE_CHG_TIMEOUT__48_MINUTES;
	rc = smb358_register_masked_write(smb358_chg->client, STAT_AND_TIMER_CONTROL_REGISTER_ADDR,
		STAT_AND_TIMER_CONTROL__MASK , value);
	if (false == rc){
		CHG_ERR("%s:set status and timer control register failed\n", __func__);
	}
/* 0x06h: set pin and enable control register*/
	value = LED_BLINKING_FUNCTION__DISABLE | ENABLE_PIN_CONTROL__I2C_CONTROL_DISABLE_CHG
	| USB5_1_HC_OR_USB9_1P5_HC_CONTROL__REGISTER_CONTROL | USB5_1_HC_INPUT_STAT__TRI_STAT
	| CHG_ERROR_CONTROL__TRIGGER_IRQ | APSD_DONE_CONTROL__TRIGGER_IRQ | USBIN_INPUT_VOLTAGE_BIAS__ENABLE;
	rc = smb358_register_masked_write(smb358_chg->client, PIN_AND_ENABLE_CONTROL_REGISTER_ADDR,
		PIN_AND_ENABLE_CONTROL__MASK , value);
	if (false == rc){
		CHG_ERR("%s:set pin and enable control register failed\n", __func__);
	}
/* 0x07h: set therm and system control a register*/
	value = SWITCH_FREQ_CONTROL__3M | MIN_SYSTEM_VOLTAGE__3600_OR_3750_MV
	| THERM_MONITOR_SELECT__VDDCAP | THERMISTOR_MONITOR_CONTROL__DISABLE
	| SOFT_COLD_TEMP_LIMIT_BEHAVIOR__NO_RESPONSE |SOFT_HOT_TEMP_LIMIT_BEHAVIOR__NO_RESPONSE;
	rc = smb358_register_masked_write(smb358_chg->client, THERM_AND_SYSTEM_CONTROL_A_REGISTER_ADDR,
		THERM_AND_SYSTEM_CONTROL_A__MASK , value);
	if (false == rc){
		CHG_ERR("%s:set therm and system control a register failed\n", __func__);
	}
/* 0x08h: set sysok and usb3.0 selection register */
	value = SYSOK_AND_USB3P0_OPERATION__INOK | USB2P0_AND_USB3P0_INPUT_CURRENT_LIMIT__USB2P0
	| FLOAT_VOLTAGE_COMPENSATION__VFLT_MINUS_120MV | HARD_TEMP_LIMIT_BEHAVIOR__SUSPEND_CHG_WHEN_BEYOND_LIMIT
	| PRE_CHG_TO_FAST_CHG_THRESHOLD_CONTROL__ENABLE | INOK_AND_BATGOOD_POLARITY__ACTIVE_HIGH;
	rc = smb358_register_masked_write(smb358_chg->client, SYSOK_AND_USB3P0_REGISTER_ADDR,
		SYSOK_AND_USB3P0__MASK , value);
	if (false == rc){
		CHG_ERR("%s:set sysok and usb3.0 selection register failed\n", __func__);
	}
/* 0x09h: set other control A register*/
	value = OTG_ID_PIN_CONTROL__RID_DISABLE_AND_OTG_I2C_CONTROL | OTG_PIN_POLARITY__ACTIVE_LOW
	| MIN_SYS_VOLTAGE__3P45_OR_3P6 |LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__3350MV;
	rc = smb358_register_masked_write(smb358_chg->client, OTHER_CONTROL_A_REGISTER_ADDR,
		OTHER_CONTROL_A__MASK , value);
	if (false == rc){
		CHG_ERR("%s:set other control a register failed\n", __func__);
	}
/* 0x0a: set otg tlim therm control register*/
	value = CHG_CURRENT_COMPENSATION__600MA | DIGITAL_THERM_REGULATION_TEMP_THRESHOLD__130C
	| OTG_CURRENT_LIMIT_AT_USB_IN__900MA | OTG_BAT_UVLO_THRESHOLD__3300MV;
	rc = smb358_register_masked_write(smb358_chg->client, OTG_TLIM_THERM_CONTROL_REGISTER_ADDR,
		OTG_TLIM_THERM_CONTROL__MASK , value);
	if (false == rc){
		CHG_ERR("%s:set otg tlim therm control register failed\n", __func__);
	}
/* 0x0b: set hard /soft limit cell temperature monitor register*/
	value = HARD_COLD_TEMP_ALARM_TRIP_POINT__10C | HARD_HOT_TEMP_ALARM_TRIP_POINT__55C
	| SOFT_COLD_TEMP_ALARM_TRIP_POINT__15C | SOFT_HOT_TEMP_ALARM_TRIP_POINT__55C;
	rc = smb358_register_masked_write(smb358_chg->client, HARD_SOFT_LIMIT_TEMP_MONITOR_ADDR,
		HARD_SOFT_LIMIT_TEMP_MONITOR__MASK , value);
	if (false == rc){
		CHG_ERR("%s:set hard /soft limit cell temperature monitor register failed\n", __func__);
	}
/* 0x0c: set fault interrupt register*/
	value = FAULT_INTERRUPT__TEMP_BEYOND_HARD_LIMIT | FAULT_INTERRUPT__TEMP_BEYOND_SOFT_LIMIT
	| FAULT_INTERRUPT__OTG_BAT_FAIL | FAULT_INTERRUPT__OTG_OVER_CURRENT_LIMIT
	| FAULT_INTERRUPT__USB_INPUT_OVER_VOLTAGE | FAULT_INTERRUPT__USB_INPUT_UNDER_VOLTAGE
	| FAULT_INTERRUPT__ACIL_COMPLETE | FAULT_INTERRUPT__INTERNAL_OVER_TEMP;
	rc = smb358_register_masked_write(smb358_chg->client, FAULT_INTERRUPT_REGISTER_ADDR,
		FAULT_INTERRUPT__MASK, value);
	if (false == rc){
		CHG_ERR("%s: set fault interrupt register failed\n", __func__);
	}
/* 0x0d: set status interrupt register*/
	value = STAT_INTERRUPT__CHG_TIME_OUT | STAT_INTERRUPT__BAT_OVER_VOLTAGE
		| STAT_INTERRUPT__FAST_CHG_TERM_OR_TAPPER_CHG | STAT_INTERRUPT__INOK
		| STAT_INTERRUPT__MISSING_BAT | STAT_INTERRUPT__LOW_BAT;	//sjc test
	rc = smb358_register_masked_write(smb358_chg->client, STAT_INTERRUPT_REGISTER_ADDR,
		STAT_INTERRUPT__MASK, value);
	if (false == rc){
		CHG_ERR("%s: set status interrupt register failed\n", __func__);
	}

	return true;
}

static irqreturn_t smb358_irq( int irq, void *dev_id )
{
	//disable_irq(irq);
	if(the_smb358_charger != NULL) {
		CHG_DBG("%s, SMB358 Charging chip IRQ coming\n", __func__);
		//CHG_ERR("\n========%s: smb358 Charging chip IRQ coming========\n\n",__func__);
		//smb358_irq_registers_read(the_smb358_charger);
		queue_work(the_smb358_charger->work_queue, &the_smb358_charger->irq_work);
	}else{
	       CHG_ERR("%s: the_smb358_charger is NULL\n",__func__);
	}
	//enable_irq(irq);
	
	return IRQ_HANDLED;
}

static void smb358_irq_init(struct smb358_charger *smb358_chg)
{
	int ret = -1;
	ret = gpio_request(smb358_chg->charger_platform_data.irq_gpio, "smb358_irq_gpio");
	if (ret) {
		CHG_ERR("%s: smb358 charger irq gpio_request failed, error %d\n", __func__, ret);
	}
    	ret = gpio_direction_input(smb358_chg->charger_platform_data.irq_gpio);
	if (ret < 0) {
		CHG_ERR("Failed to configure input direction for GPIO %d, error %d\n",smb358_chg->charger_platform_data.irq_gpio, ret);
	    	if (smb358_chg->charger_platform_data.irq_gpio)
			gpio_free(smb358_chg->charger_platform_data.irq_gpio);
	}
	smb358_chg->irq = gpio_to_irq(smb358_chg->charger_platform_data.irq_gpio);
	//CHG_ERR("%s: smb358_chg->irq=%d\n", __func__, smb358_chg->irq);
		
	ret = request_threaded_irq(smb358_chg->irq, smb358_irq,
				   NULL,
				   IRQF_DISABLED | IRQF_TRIGGER_FALLING,
				   smb358_chg->client->name, smb358_chg);

	//ret = request_irq(smb358_chg->irq, smb358_irq, IRQF_TRIGGER_FALLING, smb358_chg->client->name, smb358_chg);
	
	//enable_irq(smb358_chg->irq);
	
	if (ret) {
		CHG_ERR("%s request_threaded_irq failed for %d ret=%d\n", __func__, smb358_chg->irq, ret);
		free_irq(smb358_chg->irq, NULL);
	}
	//disable_irq_nosync(smb358_chg->irq);
}

static void smb358_locks_init(struct smb358_charger *smb358_chg)
{
	mutex_init(&smb358_chg->lock);
	wake_lock_init(&smb358_chg->wlock, WAKE_LOCK_SUSPEND, SMB358_CHARGER_NAME);
	wake_lock_init(&smb358_chg->prevent_wlock, WAKE_LOCK_SUSPEND, "smb358_charger_prevent_wlock");//sjc0823
}

static void smb358_works_init(struct smb358_charger *smb358_chg)
{
	INIT_WORK(&smb358_chg->irq_work, smb358_irq_work);
	INIT_DELAYED_WORK(&smb358_chg->start_charging_work, smb358_start_charging_work);
	INIT_DELAYED_WORK(&smb358_chg->update_work, smb358_update_work);
	smb358_chg->work_queue = create_workqueue(SMB358_CHARGER_NAME);
}

//sjc0820timer begin
static void smb358_charging_timeout_status_set(struct smb358_charger *smb358_chg, bool val)
{
	smb358_chg->power_supply_data.charging_timeout_status = val;
}

static bool smb358_charging_timeout_status_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.charging_timeout_status;
}
/*
static void smb358_max_chg_timeout_flag_set(struct smb358_charger *smb358_chg, bool val)
{
	smb358_chg->max_chg_timeout_timer_flag = val;
}

static bool smb358_max_chg_timeout_flag_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->max_chg_timeout_timer_flag;
}
*/
static void smb358_start_chg_timeout_timer(struct smb358_charger *smb358_chg)
{

	CHG_DBG("===%s: enter===\n", __func__);
	if (smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__DCP)
		hrtimer_start(&smb358_chg->max_chg_timeout_timer, ktime_set(MAX_CHARGING_TIMEOUT_DCP, 0), HRTIMER_MODE_REL);
	else if (smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__SDP || smb358_charger_type_get(smb358_chg) == CHARGER_TYPE__NON_DCP)
		hrtimer_start(&smb358_chg->max_chg_timeout_timer, ktime_set(MAX_CHARGING_TIMEOUT_SDP, 0), HRTIMER_MODE_REL);
	//smb358_max_chg_timeout_flag_set(smb358_chg, true);
}

static void smb358_stop_chg_timeout_timer(struct smb358_charger *smb358_chg)
{
	CHG_DBG("===%s: enter===\n", __func__);
	hrtimer_cancel(&smb358_chg->max_chg_timeout_timer);
	//smb358_max_chg_timeout_flag_set(smb358_chg, false);
}

static enum hrtimer_restart smb358_max_chg_timeout_timer_func(struct hrtimer *timer)
{
	struct smb358_charger *smb358_chg = container_of(timer, struct smb358_charger, max_chg_timeout_timer);

	CHG_ERR("===%s: safe timer expired===\n", __func__);

	smb358_charging_timeout_set(smb358_chg, true);
	
	return HRTIMER_NORESTART;	
}

static void smb358_timer_init(struct smb358_charger *smb358_chg)
{
	hrtimer_init(&smb358_chg->max_chg_timeout_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	smb358_chg->max_chg_timeout_timer.function = smb358_max_chg_timeout_timer_func;
}
//sjc0820timer end

/*this part for sysfs debug begin*/
static bool smb358_debug_enabled_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->debug_data.debug_enabled;
}

static void smb358_debug_enabled_set(struct smb358_charger *smb358_chg, bool val)
{
	smb358_chg->debug_data.debug_enabled= val;
}

static bool smb358_debug_printk_enabled_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->debug_data.printk_enabled;
}

static void smb358_debug_printk_enabled_set(struct smb358_charger *smb358_chg, bool val)
{
	smb358_chg->debug_data.printk_enabled = val;
}

static bool smb358_debug_dump_registers_enabled_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->debug_data.dump_registers_enabled;
}

static void smb358_debug_dump_registers_enabled_set(struct smb358_charger *smb358_chg, bool val)
{
	smb358_chg->debug_data.dump_registers_enabled = val;
}

static bool smb358_debug_charging_timeout_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->debug_data.charging_timeout;
}

static void smb358_debug_charging_timeout_set(struct smb358_charger *smb358_chg, bool val)
{
	smb358_chg->debug_data.charging_timeout = val;
}

static smb358_charger_type smb358_debug_charger_type_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->debug_data.charger_type;
}

static void smb358_debug_charger_type_set(struct smb358_charger *smb358_chg, smb358_charger_type chg_type)
{
	smb358_chg->debug_data.charger_type = chg_type;
}

static int smb358_debug_charger_voltage_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->debug_data.charger_voltage;
}

static void smb358_debug_charger_voltage_set(struct smb358_charger *smb358_chg, int  vol)
{
	smb358_chg->debug_data.charger_voltage = vol;
}

static int smb358_debug_battery_voltage_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->debug_data.battery_voltage;
}

static void smb358_debug_battery_voltage_set(struct smb358_charger *smb358_chg, int  vol)
{
	smb358_chg->debug_data.battery_voltage= vol;
}

static int smb358_debug_battery_temperature_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->debug_data.battery_temperature;
}

static void smb358_debug_battery_temperature_set(struct smb358_charger *smb358_chg, int  vol)
{
	smb358_chg->debug_data.battery_temperature = vol;
}

static int smb358_debug_battery_capacity_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->debug_data.battery_capacity;
}

static void smb358_debug_battery_capacity_set(struct smb358_charger *smb358_chg, int  vol)
{
	smb358_chg->debug_data.battery_capacity= vol;
}

static int smb358_debug_charging_current_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->debug_data.charging_current;
}

static void smb358_debug_charging_current_set(struct smb358_charger *smb358_chg, int  vol)
{
	smb358_chg->debug_data.charging_current= vol;
}

static ssize_t smb358_debug_enabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d\n", smb358_debug_enabled_get(smb358_chg));
}

static ssize_t smb358_debug_enabled_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	val = !!val;
	smb358_debug_enabled_set(smb358_chg, val);
	return size;
}

static ssize_t smb358_debug_charger_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d:%s\n", smb358_debug_charger_type_get(smb358_chg), charger_type_str[smb358_debug_charger_type_get(smb358_chg)]);
}

static ssize_t smb358_debug_charger_type_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	if (false == smb358_debug_enabled_get(smb358_chg))
		return size;
	if (val < 0 || val > CHARGER_TYPE__ACA){
		CHG_ERR("%s:unkown charger type\n", __func__);
		return size;
	}
	smb358_debug_charger_type_set(smb358_chg, val);
	smb358_charger_connected(smb358_debug_charger_type_get(smb358_chg));
	return size;
}

static ssize_t smb358_debug_charger_voltage_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d\n", smb358_debug_charger_voltage_get(smb358_chg));
}

static ssize_t smb358_debug_charger_voltage_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	if (false == smb358_debug_enabled_get(smb358_chg))
		return size;
	smb358_debug_charger_voltage_set(smb358_chg, val);
	return size;
}

static ssize_t smb358_debug_battery_voltage_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d\n", smb358_debug_battery_voltage_get(smb358_chg));
}

static ssize_t smb358_debug_battery_voltage_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	if (false == smb358_debug_enabled_get(smb358_chg))
		return size;
	smb358_debug_battery_voltage_set(smb358_chg, val);
	return size;
}

static ssize_t smb358_debug_battery_temperature_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d\n", smb358_debug_battery_temperature_get(smb358_chg));
}

static ssize_t smb358_debug_battery_temperature_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	 long val = simple_strtol(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	if (false == smb358_debug_enabled_get(smb358_chg))
		return size;
	smb358_debug_battery_temperature_set(smb358_chg, val);
	return size;
}

static ssize_t smb358_debug_battery_capacity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d\n", smb358_debug_battery_capacity_get(smb358_chg));
}

static ssize_t smb358_debug_battery_capacity_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	if (false == smb358_debug_enabled_get(smb358_chg))
		return size;
	smb358_debug_battery_capacity_set(smb358_chg, val);
	return size;
}

static ssize_t smb358_debug_charging_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d\n", smb358_debug_charging_current_get(smb358_chg));
}

static ssize_t smb358_debug_charging_current_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	long val = simple_strtol(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	if (false == smb358_debug_enabled_get(smb358_chg))
		return size;
	smb358_debug_charging_current_set(smb358_chg, val);
	return size;
}

static ssize_t smb358_debug_enable_charging_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t smb358_debug_enable_charging_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	if(0 != val) {
		smb358_start_charging(smb358_chg);
	}
	else{
		smb358_stop_charging(smb358_chg);
	}
	return size;
}

static ssize_t smb358_suspend_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d\n", smb358_suspended_get(smb358_chg));
}

static ssize_t smb358_suspend_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	if(0 != val) {
		CHG_ERR("%s:charger to be suspened \n", __func__);
		smb358_suspend_charger(smb358_chg);
	}
	else{
		CHG_ERR("%s:charger to be unsuspened \n", __func__);
		smb358_unsuspend_charger(smb358_chg);
	}
	return size;
}

static ssize_t smb358_debug_printk_enabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d\n", smb358_debug_printk_enabled_get(smb358_chg));
}

static ssize_t smb358_debug_printk_enabled_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	val = !!val;
	smb358_debug_printk_enabled_set(smb358_chg, val);
	return size;
}

static ssize_t smb358_debug_dump_registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d\n", smb358_debug_dump_registers_enabled_get(smb358_chg));
}

static ssize_t smb358_debug_dump_registers_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	val = !!val;
	smb358_debug_dump_registers_enabled_set(smb358_chg, val);
	return size;
}

static ssize_t smb358_debug_charging_timeout_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d\n", smb358_debug_charging_timeout_get(smb358_chg));
}

static ssize_t smb358_debug_charging_timeout_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) 
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	struct smb358_charger * smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg){
		CHG_ERR("%s:smb358 charger is null\n", __func__);
		return size;
	}
	val = !!val;
	smb358_debug_charging_timeout_set(smb358_chg, val);
	return size;
}

//sjc0826
static ssize_t smb358_debug_battery_soh_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger *smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%d%%\n", smb358_battery_soh_get(smb358_chg));
}

static ssize_t smb358_debug_battery_fcc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct smb358_charger *smb358_chg = dev_get_drvdata(dev);
	if (NULL == smb358_chg)
		return 0;
	return sprintf(buf, "%dmAh\n", smb358_battery_fcc_get(smb358_chg));
}

static DEVICE_ATTR(debug_enabled, 0660, smb358_debug_enabled_show, smb358_debug_enabled_store);
static DEVICE_ATTR(debug_charger_type, 0660, smb358_debug_charger_type_show, smb358_debug_charger_type_store);
static DEVICE_ATTR(debug_charger_voltage, 0660, smb358_debug_charger_voltage_show, smb358_debug_charger_voltage_store);
static DEVICE_ATTR(debug_battery_voltage, 0660, smb358_debug_battery_voltage_show, smb358_debug_battery_voltage_store);
static DEVICE_ATTR(debug_battery_temperature, 0660, smb358_debug_battery_temperature_show, smb358_debug_battery_temperature_store);
static DEVICE_ATTR(debug_battery_capacity, 0660, smb358_debug_battery_capacity_show, smb358_debug_battery_capacity_store);
static DEVICE_ATTR(debug_charging_current, 0660, smb358_debug_charging_current_show, smb358_debug_charging_current_store);
static DEVICE_ATTR(suspend, 0660, smb358_suspend_show, smb358_suspend_store);
static DEVICE_ATTR(debug_printk_enabled, 0660, smb358_debug_printk_enabled_show, smb358_debug_printk_enabled_store);
static DEVICE_ATTR(debug_enable_charging, 0660, smb358_debug_enable_charging_show, smb358_debug_enable_charging_store);
static DEVICE_ATTR(debug_dump_registers, 0660, smb358_debug_dump_registers_show, smb358_debug_dump_registers_store);
static DEVICE_ATTR(debug_charging_timeout, 0660, smb358_debug_charging_timeout_show, smb358_debug_charging_timeout_store);
static DEVICE_ATTR(batt_soh, 0444, smb358_debug_battery_soh_show, NULL);//sjc0826
static DEVICE_ATTR(batt_fcc, 0444, smb358_debug_battery_fcc_show, NULL);

static void smb358_debug_data_init(struct smb358_charger *smb358_chg)
{
	int ret = -1;
	smb358_debug_enabled_set(smb358_chg, false);
	smb358_debug_charger_type_set(smb358_chg, CHARGER_TYPE__INVALID);
	smb358_debug_charger_voltage_set(smb358_chg, CHARGER_VOL__DEFAULT);
	smb358_debug_battery_voltage_set(smb358_chg, BAT_VOL__DEFAULT);
	smb358_debug_battery_temperature_set(smb358_chg, BAT_TEMP__DEFAULT);
	smb358_debug_battery_capacity_set(smb358_chg, BAT_CAPACITY__DEFAULT);
	smb358_debug_charging_current_set(smb358_chg, 0);
	smb358_debug_printk_enabled_set(smb358_chg, false);
	smb358_debug_dump_registers_enabled_set(smb358_chg, false);
	smb358_debug_charging_timeout_set(smb358_chg, false);
	
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_debug_enabled);
	if (ret){
		CHG_ERR("%s:create debug file debug_enabled failed\n", __func__);
		goto out1;
	}
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_debug_charger_type);
	if (ret){
		CHG_ERR("%s:create debug file debug_charger_type failed\n", __func__);
		goto out2;
	}

	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_debug_charger_voltage);
	if (ret){
		CHG_ERR("%s:create debug file debug_charger_voltage failed\n", __func__);
		goto out3;
	}
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_debug_battery_voltage);
	if (ret){
		CHG_ERR("%s:create debug file debug_battery_voltage failed\n", __func__);
		goto out4;
	}
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_debug_battery_temperature);
	if (ret){
		CHG_ERR("%s:create debug file debug_battery_temperature failed\n", __func__);
		goto out5;
	}
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_debug_battery_capacity);
	if (ret){
		CHG_ERR("%s:create debug file debug_battery_capacity failed\n", __func__);
		goto out6;
	}

	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_debug_charging_current);
	if (ret){
		CHG_ERR("%s:create debug file debug_charging_current failed\n", __func__);
		goto out7;
	}
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_suspend);
	if (ret){
		CHG_ERR("%s:create  file suspend failed\n", __func__);
		goto out8;
	}
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_debug_printk_enabled);
	if (ret){
		CHG_ERR("%s:create  file debug_printk_enabled failed\n", __func__);
		goto out9;
	}
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_debug_enable_charging);
	if (ret){
		CHG_ERR("%s:create  file debug_enable_charging failed\n", __func__);
		goto out10;
	}
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_debug_dump_registers);
	if (ret){
		CHG_ERR("%s:create  file debug_dump_registers failed\n", __func__);
		goto out11;
	}
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_debug_charging_timeout);
	if (ret){
		CHG_ERR("%s:create  file debug_charging_timeout failed\n", __func__);
		goto out12;
	}
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_batt_soh);
	if (ret){
		CHG_ERR("%s:create  file debug_charging_timeout failed\n", __func__);
		goto out13;
	}
	ret = device_create_file(&smb358_chg->client->dev, &dev_attr_batt_fcc);
	if (ret){
		CHG_ERR("%s:create  file debug_charging_timeout failed\n", __func__);
		goto out14;
	}
	return;

out14:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_batt_soh);
out13:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_debug_charging_timeout);
out12:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_debug_dump_registers);
out11:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_debug_enable_charging);
out10:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_debug_printk_enabled);
out9:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_suspend);
out8:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_debug_charging_current);
out7:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_debug_battery_capacity);
out6:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_debug_battery_temperature);
out5:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_debug_battery_voltage);
out4:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_debug_charger_voltage);
out3:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_debug_charger_type);
out2:
	device_remove_file(&smb358_chg->client->dev, &dev_attr_debug_enabled);
out1:
	return;
}

//sjc1003 charging current aicl
static bool smb358_charging_current_aicl_status_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->charging_current_aicl_status;
}

static void smb358_charging_current_aicl_status_set(struct smb358_charger *smb358_chg, bool val)
{
	smb358_chg->charging_current_aicl_status = val;
}

static void smb358_charging_current_aicl_check_and_set(struct smb358_charger *smb358_chg)
{
	//static int counts = 0;
	
	if (smb358_battery_voltage_get(smb358_chg) > CHG_CURRENT_AICL_VOL)
		current_aicl_counts++;
	else
		current_aicl_counts = 0;

	if (current_aicl_counts > CHG_CURRENT_AICL_COUNT) {
		smb358_charging_current_write(smb358_chg, STANDARD__FAST_CHG__NORMAL_CURRNET_AICL);
		//smb358_charging_current_write_fast(smb358_chg, STANDARD__FAST_CHG__NORMAL_CURRNET_AICL);
		smb358_charging_current_aicl_status_set(smb358_chg, true);
		CHG_DBG("===%s:set charging current to 1800mA===\n", __func__);
	}	
}

static void smb358_charging_current_aicl_handle(struct smb358_charger *smb358_chg)
{
	if (smb358_charger_type_get(smb358_chg) != CHARGER_TYPE__DCP || smb358_charging_status_get(smb358_chg) != CHARGING_STATUS__CHARGING
			|| smb358_battery_temperature_region_get(smb358_chg) != BATTERY_TEMP_REGION__NORMAL)
		return;

	if (smb358_charging_current_aicl_status_get(smb358_chg) == true)
		return;

	smb358_charging_current_aicl_check_and_set(smb358_chg);
}

//sjc0927
static bool smb358_early_suspend_status_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->early_suspend_status;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void smb358_early_suspend(struct early_suspend *handler)
{
	struct smb358_charger *smb358_chg = container_of(handler, struct smb358_charger, early_suspend);
	
	CHG_DBG("%s\n", __func__);
	if (smb358_chg) {
		smb358_chg->early_suspend_status = true;
		if (CHARGER_TYPE__DCP == smb358_charger_type_get(smb358_chg) && smb358_chg->aicl_result > 1100) {//sjc1010 only use in charger>1.1A
			if (!smb358_input_current_limit_write(smb358_chg, MAX_INPUT_CURRENT_LIMIT__STANDARD)) {
				CHG_ERR("%s:input current limit write failed\n", __func__);
			} else {
				if (smb358_register_masked_write(smb358_chg->client, VARIOUS_FUNCTIONS_REGISTER_ADDR,
						AUTO_INPUT_CURRENT_LIMIT_CONTROL__MASK, AUTO_INPUT_CURRENT_LIMINT_CONTROL__DISABLE)) {
					if (!smb358_register_masked_write(smb358_chg->client, VARIOUS_FUNCTIONS_REGISTER_ADDR,
							AUTO_INPUT_CURRENT_LIMIT_CONTROL__MASK, AUTO_INPUT_CURRENT_LIMINT_CONTROL__ENABLE))
						CHG_ERR("%s:enable failed\n", __func__);
				} else {
					CHG_ERR("%s:disable failed\n", __func__);
				}
			}
		}
	}
}

static void smb358_late_resume(struct early_suspend *handler)
{
	struct smb358_charger *smb358_chg = container_of(handler, struct smb358_charger, early_suspend);
	
	CHG_DBG("%s\n", __func__);
	if (smb358_chg) {
		smb358_chg->early_suspend_status = false;
		if (CHARGER_TYPE__DCP == smb358_charger_type_get(smb358_chg) && smb358_chg->aicl_result > 1100) {//sjc1010 only use in charger>1.1A
			if (!smb358_input_current_limit_write(smb358_chg, MAX_INPUT_CURRENT_LIMIT__STANDARD_SUS))
				CHG_ERR("%s:input current limit write failed\n", __func__);
		}
	}
}

#endif //defined(CONFIG_HAS_EARLYSUSPEND)
static void smb358_suspend_int(struct smb358_charger *smb358_chg)
{
	smb358_chg->early_suspend_status = false;
#ifdef CONFIG_HAS_EARLYSUSPEND
	smb358_chg->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	smb358_chg->early_suspend.suspend = smb358_early_suspend;
	smb358_chg->early_suspend.resume = smb358_late_resume;
	register_early_suspend(&smb358_chg->early_suspend);
#endif
}

static void smb358_suspend_deinit(struct smb358_charger *smb358_chg)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&smb358_chg->early_suspend);
#endif
}

//sjc0826
static int smb358_battery_soh_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.battery_soh;
}

static void smb358_battery_soh_set(struct smb358_charger *smb358_chg, int val)
{
	smb358_chg->power_supply_data.battery_soh = val;
}

static int smb358_battery_fcc_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->power_supply_data.battery_fcc;
}

static void smb358_battery_fcc_set(struct smb358_charger *smb358_chg, int val)
{
	smb358_chg->power_supply_data.battery_fcc = val;
}

//sjc0824aicl
static bool smb358_aicl_status_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->aicl_status;
}

static void smb358_aicl_status_set(struct smb358_charger *smb358_chg, bool val)
{
	smb358_chg->aicl_status = val;
	if (val == false)
		smb358_chg->aicl_result = 0;//sjc1010
}

static void smb358_aicl_check(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	u8 value = 0;
	
	ret = smb358_register_read(smb358_chg->client, STAT_REG_E_REGISTER_ADDR, &value);
	if (ret) {
		switch (value & AICL_STAT__MASK) {
			case AICL_STAT__COMPLETED:
				smb358_aicl_status_set(smb358_chg, true);
				break;
			case AICL_STAT__NOT_COMPLETED:
				smb358_aicl_status_set(smb358_chg, false);
				break;
			default:
				break;
		}	
	} else
		CHG_ERR("%s:status register E read failed\n", __func__);
}

static int smb358_aicl_results(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	u8 value = 0;
	int result = 0;

	ret = smb358_register_read(smb358_chg->client, STAT_REG_E_REGISTER_ADDR, &value);
	if (ret) {
		switch (value & AICL_RESULT__MASK) {
			case AICL_RESULT__300MA:
				result = 300;
				break;
			case AICL_RESULT__500MA:
				result = 500;
				break;
			case AICL_RESULT__700MA:
				result = 700;
				break;
			case AICL_RESULT__900MA:
				result = 900;
				break;
			case AICL_RESULT__1100MA:
				result = 1100;
				break;
			case AICL_RESULT__1300MA:
				result = 1300;
				break;
			case AICL_RESULT__1500MA:
				result = 1500;
				break;
			case AICL_RESULT__2000MA:
				result = 2000;
				break;
			default:
				break;
		}
	}
	smb358_chg->aicl_result = result;//sjc1010
	return result;
}

static void smb358_aicl_handle(struct smb358_charger *smb358_chg)
{
	bool aicl_pre = false;
	bool aicl_now = false;
	int aicl_result = 0;
	//int aicl_set = 0;
	
	if (smb358_charger_type_get(smb358_chg) != CHARGER_TYPE__DCP)
		return;

	if (smb358_charging_status_get(smb358_chg) != CHARGING_STATUS__CHARGING)
		return;
	
	aicl_pre = smb358_aicl_status_get(smb358_chg);
	if (aicl_pre == true)//already complete, retrun
		return;
	
	smb358_aicl_check(smb358_chg);
	
	aicl_now = smb358_aicl_status_get(smb358_chg);
	if ((aicl_pre == false) && (aicl_now == true)) {
		aicl_result = smb358_aicl_results(smb358_chg);
		if (aicl_result > 1500) {//2000mA
			CHG_DBG("===%s:AICL=2000mA(default)===\n", __func__);//default
		}
		else if (aicl_result > 1100) {//1500mA
			if (!smb358_input_current_limit_write(smb358_chg, MAX_CHG_CURRENT__1500MA))
				CHG_ERR("%s:AICL=1500mA failed\n", __func__);
			else
				CHG_DBG("===%s:AICL=1500mA OK===\n", __func__);
		}
		else if (aicl_result >= 900) {//1000mA
			if (!smb358_input_current_limit_write(smb358_chg, MAX_CHG_CURRENT__1000MA))
				CHG_ERR("%s:AICL=1000mA failed\n", __func__);
			else
				CHG_DBG("===%s:AICL=1000mA OK===\n", __func__);
		}
		else
			if (!smb358_input_current_limit_write(smb358_chg, MAX_CHG_CURRENT__700MA))
				CHG_ERR("%s:AICL=700mA failed\n", __func__);
			else
				CHG_DBG("===%s:AICL=700mA OK===\n", __func__);
	}
}
//sjc0824

static bool smb358_suspended_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->suspended;
}

static void smb358_suspended_set(struct smb358_charger *smb358_chg, bool value)
{
	smb358_chg->suspended = value;
}

#if 0 //sjc0806
static void smb358_suspend_charger(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	ret = smb358_stop_charging(smb358_chg);
	if (ret){
		ret = smb358_register_masked_write(smb358_chg->client, CMD_REG_A_REGISTER_ADDR,
			SUSPEND_MODE_CONTROL__MASK, SUSPEND_MODE_CONTROL__ENABLE);
		if (ret){
			CHG_ERR("%s:suspend charger success\n", __func__);
			smb358_charging_status_set(smb358_chg, CHARGING_STATUS__DISCHARGING);
			smb358_suspended_set(smb358_chg, true);
		}
		else
			CHG_ERR("%s:suspend charger failed\n", __func__);
	}else
		CHG_ERR("%s:stop charging failed\n", __func__);
}
#else
static void smb358_suspend_charger(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	ret = smb358_register_masked_write(smb358_chg->client, CMD_REG_A_REGISTER_ADDR,
			SUSPEND_MODE_CONTROL__MASK, SUSPEND_MODE_CONTROL__ENABLE);
	if (ret) {
		CHG_ERR("%s:suspend charger success\n", __func__);
		//smb358_charging_status_set(smb358_chg, CHARGING_STATUS__DISCHARGING);
		smb358_suspended_set(smb358_chg, true);
	}
	else
		CHG_ERR("%s:suspend charger failed\n", __func__);
}
#endif

#if 0 //sjc0806
static void smb358_unsuspend_charger(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	ret = smb358_register_masked_write(smb358_chg->client, CMD_REG_A_REGISTER_ADDR,
		SUSPEND_MODE_CONTROL__MASK, SUSPEND_MODE_CONTROL__DISABLE);
	if (ret){
		CHG_ERR("%s:unsuspend charger success\n", __func__);
		smb358_suspended_set(smb358_chg, false);
		ret = smb358_start_charging(smb358_chg);
		if (ret){
			CHG_ERR("%s:start charging success\n", __func__);
		}else
			CHG_ERR("%s:start charging failed\n", __func__);
	}
	else
		CHG_ERR("%s:unsuspend charger failed\n", __func__);
}
#else
static void smb358_unsuspend_charger(struct smb358_charger *smb358_chg)
{
	bool ret = false;
	ret = smb358_register_masked_write(smb358_chg->client, CMD_REG_A_REGISTER_ADDR,
			SUSPEND_MODE_CONTROL__MASK, SUSPEND_MODE_CONTROL__DISABLE);
	if (ret) {
		CHG_ERR("%s:unsuspend charger success\n", __func__);
		smb358_suspended_set(smb358_chg, false);
		//ret = smb358_start_charging(smb358_chg);
		//if (ret){
		//	CHG_ERR("%s:start charging success\n", __func__);
		//}else
		//	CHG_ERR("%s:start charging failed\n", __func__);
	}
	else
		CHG_ERR("%s:unsuspend charger failed\n", __func__);
}
#endif
/*for init functions end*/

//sjc0808otg begin
static bool smb358_otg_status_get(struct smb358_charger *smb358_chg)
{
	return smb358_chg->otg_enabled;
}

static void smb358_otg_status_set(struct smb358_charger *smb358_chg, bool value)
{
	smb358_chg->otg_enabled = value;
}

static void smb358_otg_enable(struct smb358_charger *smb358_chg)
{
	bool ret = false;

	ret = smb358_register_masked_write(smb358_chg->client, CMD_REG_A_REGISTER_ADDR,
			OTG_SWITCH__MASK, OTG_SWITCH__ENABLE);
	if (ret) {
		CHG_ERR("%s:OTG enable success\n", __func__);
		smb358_otg_status_set(smb358_chg, true);
	}
	else
		CHG_ERR("%s:OTG enable failed\n", __func__);
}

static void smb358_otg_disable(struct smb358_charger *smb358_chg)
{
	bool ret = false;

	ret = smb358_register_masked_write(smb358_chg->client, CMD_REG_A_REGISTER_ADDR,
			OTG_SWITCH__MASK, OTG_SWITCH__DISABLE);
	if (ret) {
		CHG_ERR("%s:OTG disable success\n", __func__);
		smb358_otg_status_set(smb358_chg, false);
	}
	else
		CHG_ERR("%s:OTG disable failed\n", __func__);
}
//sjc0808otg end

bool get_charging_status(void)
{
	if (the_smb358_charger == NULL) {
		CHG_ERR("%s:smb358 charger is not initialized yet\n", __func__);
		return false;
	}
	
	if (smb358_charging_status_get(the_smb358_charger) == CHARGING_STATUS__CHARGING || 
		smb358_charging_status_get(the_smb358_charger) == CHARGING_STATUS__FULL)
		return true;
	else
		return false;
}

void smb358_battery_fuelgauge_register(struct oppo_battery_fuelgauge *fuelgauge)
{
	batt_fuelgauge = fuelgauge;
}
EXPORT_SYMBOL(smb358_battery_fuelgauge_register);

void smb358_battery_fuelgauge_unregister(struct oppo_battery_fuelgauge *fuelgauge)
{
	batt_fuelgauge = NULL;
}
EXPORT_SYMBOL(smb358_battery_fuelgauge_unregister);

bool is_show_gague_log(void)
{
	return smb358_debug_enabled_get(the_smb358_charger);
}

/**
 *  From 12001 (by wangjc)
 * DESCRIPTION: - map liner integer to another integer 
 * @Param: voltage2capacity, nTableSize, input, output
 * @Return: int
 */ 
static int map_liner_int32_to_int32(const voltage2capacity *paPts, u32 nTableSize, s32 input, s32 *output)
{
	bool bDescending = true;
	u32 nSearchIdx = 0;

	if ((paPts == NULL) || (output == NULL))
	{
		return -1;//-EINVAL;
	}

	/* Check if table is descending or ascending */
	if (nTableSize > 1)
	{
		if (paPts[0].x < paPts[1].x)
		{
			bDescending = false;
		}
	}

	while (nSearchIdx < nTableSize)
	{
		if ( (bDescending == true) && (paPts[nSearchIdx].x < input) )
		{
			/* table entry is less than measured value and table is descending, stop */
			break;
		}
		else if ( (bDescending == false) && (paPts[nSearchIdx].x > input) )
		{
			/* table entry is greater than measured value and table is ascending, stop */
			break;
		}
		else
		{
			nSearchIdx++;
		}
	}

	if (nSearchIdx == 0)
	{
		*output = paPts[0].y;
	}
	else if (nSearchIdx == nTableSize)
	{
		*output = paPts[nTableSize-1].y;
	}
	else
	{
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (
		       ( (s32)
		           (
		            (paPts[nSearchIdx].y - paPts[nSearchIdx-1].y)
		             * (input - paPts[nSearchIdx-1].x)
		           )
		           / (paPts[nSearchIdx].x - paPts[nSearchIdx-1].x)
		       )
		       + paPts[nSearchIdx-1].y
		     );
	}
   return 0;
}

/** 
 * @Param: voltage_now
 * @Return: battery capacity or error (-1)
 */ 
static u32 get_batt_capacity_by_batt_voltage(u32 voltage_now)
{
	u32 capacity;
	int ret;

	ret = map_liner_int32_to_int32(v2c_map, sizeof(v2c_map)/sizeof(v2c_map[0]), voltage_now, &capacity);
	if(ret)
		return ret;	//error -1

	return capacity;
}

static int get_charger_voltage(void)
{
	return get_charger_voltage_from_adc();
}

static int get_battery_capacity(void)
{
	int batt_capacity = -1;
	
	if (batt_fuelgauge && batt_fuelgauge->get_battery_capacity) {
		batt_capacity = batt_fuelgauge->get_battery_capacity();
		if (batt_capacity != -1) 
			return batt_capacity;
	}

	if (get_pcb_version() < PCB_VERSION_EVT3_N1F) {//EVT3 and later not use...
		batt_capacity = get_batt_capacity_by_batt_voltage(get_battery_voltage());
		if (batt_capacity != -1)
			return batt_capacity;
	}

	return 30;
}

static int get_battery_voltage(void)
{
	int batt_voltage = -1;
	
	if (batt_fuelgauge && batt_fuelgauge->get_battery_mvolts) {
		batt_voltage = batt_fuelgauge->get_battery_mvolts();
		if (batt_voltage != -1)
			return batt_voltage;
	}

	return get_battery_voltage_from_adc();
}

static int get_battery_temperature(void)
{
	int batt_temp = -1000;

	if (batt_fuelgauge && batt_fuelgauge->get_battery_temperature) {
		batt_temp = 	batt_fuelgauge->get_battery_temperature();
		if (batt_temp != -1000)
			return batt_temp;
	}

	return get_battery_temperature_from_adc();
}

static int get_battery_soh(void)//sjc0826
{	
	if (batt_fuelgauge && batt_fuelgauge->get_battery_soh) {
		return batt_fuelgauge->get_battery_soh();
	}
	
	return -1;
}

static int get_battery_fcc(void)//sjc0826
{
	if (batt_fuelgauge && batt_fuelgauge->get_battery_fcc) {
		return batt_fuelgauge->get_battery_fcc();
	}
	
	return -1;
}

static int get_charging_current(void)
{
	int ichg = -10000;

	if (batt_fuelgauge && batt_fuelgauge->get_ichg_current) {
		ichg = batt_fuelgauge->get_ichg_current();
		if (ichg != -10000) {
			if (ichg & 0x8000) {
				ichg = -((~(ichg - 1))&0x0000FFFF);
			}
			return ichg;
		}
	}

	return get_charging_current_from_adc();
}

static int set_battery_capacity_full(void)
{
	if (batt_fuelgauge && batt_fuelgauge->set_full_capacity) {
		return batt_fuelgauge->set_full_capacity();
	} else {
		pr_err("%s:Warning: batt_fuelgauge not ready!\n",__func__);
		return -ENODEV;
	}
}

static int __devinit smb358_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	const struct smb358_charger_platform_data *pdata;
	struct smb358_charger *smb358_chg;
	CHG_ERR("%s:\n", __func__);

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "%s no platform data\n", __func__);
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		CHG_ERR("%s:smb358 i2c_probe I2C_FUNC_SMBUS_BYTE_DATA Failed\n", __func__);		
		return -EIO;
	}
	smb358_chg = (struct smb358_charger *)kzalloc(sizeof(struct smb358_charger), GFP_KERNEL);
	if (!smb358_chg) {
		CHG_ERR("%s:allocat memroy for smb358_charger failed\n", __func__);
		return -ENOMEM;
	}
	the_smb358_charger = smb358_chg;
	smb358_chg->client = client;
	i2c_set_clientdata(client, smb358_chg);

/*do some init works*/
	smb358_charger_platform_data_init(smb358_chg, pdata);
	smb358_battery_platform_data_init(smb358_chg);
	smb358_power_supply_data_init(smb358_chg);

	smb358_power_supplies_init(smb358_chg);

	//smb358_irq_init(smb358_chg);
	smb358_hardware_init(smb358_chg);
	
	smb358_debug_data_init(smb358_chg);

	smb358_locks_init(smb358_chg);
	smb358_works_init(smb358_chg);
	smb358_timer_init(smb358_chg);//sjc0820
	smb358_suspend_int(smb358_chg);//sjc0927

	smb358_irq_init(smb358_chg);
	
/*start chg if possible*/
	if (CHARGER_TYPE__INVALID != startup_charger_type)
		smb358_charger_connected(smb358_charger_type_get(smb358_chg));
	//schedule_delayed_work(&smb358_chg->update_work, msecs_to_jiffies(POWER_SUPPLY_UPDATE_PERIOD));
	schedule_delayed_work(&smb358_chg->update_work, round_jiffies_relative(msecs_to_jiffies(POWER_SUPPLY_UPDATE_PERIOD)));

	//enable_irq(smb358_chg->irq);
	CHG_ERR("%s:probe end\n", __func__);
	return 0;
}

static int __devexit smb358_remove(struct i2c_client *client)
{
	struct smb358_charger *smb358_chg = i2c_get_clientdata(client);

	if (smb358_otg_status_get(smb358_chg))//sjc1023
		smb358_otg_disable(smb358_chg);
	flush_delayed_work(&smb358_chg->start_charging_work);
	flush_work(&smb358_chg->irq_work);
	flush_delayed_work(&smb358_chg->update_work);
	destroy_workqueue(smb358_chg->work_queue);
	
	smb358_charger_platform_data_deinit(smb358_chg);
	smb358_battery_platform_data_deinit(smb358_chg);
	smb358_power_supplies_deinit(smb358_chg);
	
	free_irq(smb358_chg->irq, NULL);
	gpio_free(smb358_chg->charger_platform_data.irq_gpio);

	smb358_suspend_deinit(smb358_chg);//sjc0927
	wake_lock_destroy(&smb358_chg->wlock);//sjc0823
	wake_lock_destroy(&smb358_chg->prevent_wlock);
	kfree(smb358_chg);
	
	return 0;
}

static int smb358_suspend(struct device *dev)
{
	CHG_DBG("%s\n", __func__);
	return 0;
}

static int smb358_resume(struct device *dev)
{
	CHG_DBG("%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops smb358_pm_ops = {
	.suspend	= smb358_suspend,
	.resume	= smb358_resume,
};

static const struct i2c_device_id smb358_charger_id[] = {
	{SMB358_CHARGER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, smb349_id);

static struct i2c_driver smb358_driver = {
	.driver = {
		   .name	= SMB358_CHARGER_NAME,
		   .owner	= THIS_MODULE,
		   .pm	= &smb358_pm_ops,
	},
	.probe		= smb358_probe,
	.remove		= __devexit_p(smb358_remove),
	.id_table		= smb358_charger_id,
};

static int __init smb358_init(void)
{
	if (get_pcb_version() < PCB_VERSION_EVT_N1) {
		CHG_ERR("===%s===\n", __func__);
		return 0;
	}

	return i2c_add_driver(&smb358_driver);
}
module_init(smb358_init);

static void __exit smb358_exit(void)
{
	if (get_pcb_version() < PCB_VERSION_EVT_N1)
		return;

	return i2c_del_driver(&smb358_driver);
}
module_exit(smb358_exit);

MODULE_DESCRIPTION("Driver for SMB358 charger chip");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:" SMB358_CHARGER_NAME);

