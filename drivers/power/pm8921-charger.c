/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/mfd/pm8xxx/pm8921-bms.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/mfd/pm8xxx/ccadc.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/batt-alarm.h>
#include <linux/ratelimit.h>

#include <mach/msm_xo.h>
/* OPPO 2012-08-10 chendx Modify begin for 12025 charge*/
#ifndef CONFIG_VENDOR_EDIT
#include <mach/msm_hsusb.h>
#else
#include <linux/usb/msm_hsusb.h>	
#include <linux/mfd/pm8xxx/batt-alarm.h>
#include <linux/input.h>
#include <linux/reboot.h>
#endif
/* OPPO 2012-08-10 chendx Modify end */

#define CHG_BUCK_CLOCK_CTRL	0x14
#define CHG_BUCK_CLOCK_CTRL_8038	0xD

#define PBL_ACCESS1		0x04
#define PBL_ACCESS2		0x05
#define SYS_CONFIG_1		0x06
#define SYS_CONFIG_2		0x07
#define CHG_CNTRL		0x204
#define CHG_IBAT_MAX		0x205
#define CHG_TEST		0x206
#define CHG_BUCK_CTRL_TEST1	0x207
#define CHG_BUCK_CTRL_TEST2	0x208
#define CHG_BUCK_CTRL_TEST3	0x209
#define COMPARATOR_OVERRIDE	0x20A
#define PSI_TXRX_SAMPLE_DATA_0	0x20B
#define PSI_TXRX_SAMPLE_DATA_1	0x20C
#define PSI_TXRX_SAMPLE_DATA_2	0x20D
#define PSI_TXRX_SAMPLE_DATA_3	0x20E
#define PSI_CONFIG_STATUS	0x20F
#define CHG_IBAT_SAFE		0x210
#define CHG_ITRICKLE		0x211
#define CHG_CNTRL_2		0x212
#define CHG_VBAT_DET		0x213
#define CHG_VTRICKLE		0x214
#define CHG_ITERM		0x215
#define CHG_CNTRL_3		0x216
#define CHG_VIN_MIN		0x217
#define CHG_TWDOG		0x218
#define CHG_TTRKL_MAX		0x219
#define CHG_TEMP_THRESH		0x21A
#define CHG_TCHG_MAX		0x21B
#define USB_OVP_CONTROL		0x21C
#define DC_OVP_CONTROL		0x21D
#define USB_OVP_TEST		0x21E
#define DC_OVP_TEST		0x21F
#define CHG_VDD_MAX		0x220
#define CHG_VDD_SAFE		0x221
#define CHG_VBAT_BOOT_THRESH	0x222
#define USB_OVP_TRIM		0x355
#define BUCK_CONTROL_TRIM1	0x356
#define BUCK_CONTROL_TRIM2	0x357
#define BUCK_CONTROL_TRIM3	0x358
#define BUCK_CONTROL_TRIM4	0x359
#define CHG_DEFAULTS_TRIM	0x35A
#define CHG_ITRIM		0x35B
#define CHG_TTRIM		0x35C
#define CHG_COMP_OVR		0x20A
#define IUSB_FINE_RES		0x2B6
#define OVP_USB_UVD		0x2B7

/* check EOC every 10 seconds */
#define EOC_CHECK_PERIOD_MS	10000
/* check for USB unplug every 200 msecs */
#define UNPLUG_CHECK_WAIT_PERIOD_MS 200
#define UNPLUG_CHECK_RAMP_MS 25
#define USB_TRIM_ENTRIES 16

/* OPPO 2012-08-10 chendx Modify begin for 12025 charge*/
#ifdef CONFIG_VENDOR_EDIT
/* delay 500msecs to begin charge*/
#define CHARGE_ENABLE_DELAY	500
/* recharging  monitor*/
#define RECHARGING_MONITOR_MS 1000*60*1
/* bellow this is concern with temperature iuse */
#define BATT_REMOVE_TEMP  -350 //0.1C
#define BATT_REMOVE_TEMP_C -35 //C


/* batt capacity algorithm */
#ifndef min
#define min(_a, _b)     (((_a) < (_b)) ? (_a) : (_b))
#endif
#ifndef max
#define max(_a, _b)     (((_a) > (_b)) ? (_a) : (_b))
#endif

/*batt capacity calibrate*/
#define BATT_CAPACITY_CALIB

/* Add begin for FTM CHARGER MODE */
#define FTM_CHARGE_MODE_FEATURE
#ifdef FTM_CHARGE_MODE_FEATURE
#define FTM_LOW_CAPACITY_LEVEL 60
#define FTM_HIGH_CAPACITY_LEVEL 80
#endif

/*Add begin for temperature charge test */
#define DEFAULT_BATT_CAPACITY 30

#define TEMPERATURE_CHARGE_TEST_FEATURE
#ifdef TEMPERATURE_CHARGE_TEST_FEATURE
#define DEFAULT_COLD_TEMP -13
#define DEFAULT_LITTLE_COLD_TEMP -5
#define DEFAULT_COOL_TEMP 5
#define DEFAULT_NORMAL_TEMP 23
#define DEFAULT_WARM_TEMP 50
#define DEFAULT_HOT_TEMP 60
#endif

/* Define for btm battery temperate get from BTM gague precison 1C*/
#define AUTO_CHARGING_BATT_TEMP_T0                           -10 
#define AUTO_CHARGING_BATT_TEMP_T1                            0    
#define AUTO_CHARGING_BATT_TEMP_T2                            10  
#define AUTO_CHARGING_BATT_TEMP_T3                            45  
#define AUTO_CHARGING_BATT_TEMP_T4                            55  
#define AUTO_CHARGING_BATT_REMOVE_TEMP                        -35 
#define AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_HOT_TO_WARM      3
#define AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_WARM_TO_NORMAL   1
#define AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COOL_TO_NORMAL   1
#define AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COLD_TO_COOL     3

/* charger voltage uovp check */
#define MAX_CHARGER_CHECK_COUNT			3
#define CHARGER_SOFT_OVP_VOLTAGE		5800
#define CHARGER_SOFT_UVP_VOLTAGE		4400
#define BATTERY_SOFT_OVP_VOLTAGE		4500  
#define CHARGER_SOFT_OVP_VOLTAGE_UI		5800
#define CHARGER_SOFT_UVP_VOLTAGE_UI		4400 

/* bad charger or battery count */
#define BAD_STATE_COUNT		3
#define CHARGER_VOLTAGE_NORMAL		5000

/*  Add for R-sense set when hw init */
#define PM8921_CHG_RSENSE_10MOHM	0x0<<6
#define PM8921_CHG_RSENSE_20MOHM	0x2<<6
#define PM8921_CHG_SAFE_MASK	0xff
#define PM8921_CHG_RSENSE_MASK	0xc0

/* Set the max iusb to 1100mA */
#define PM8921_IUSB_MAX 0x5

/* stanard dongle detect*/
struct completion stanard_mhl_wait;

/*enable to debug with charge*/
int debug_feature = 0;

#define AUTO_TEST_FEATURE
#ifdef AUTO_TEST_FEATURE
int chg_auto_test_feature = 0;
#endif
static int logo_level  = 1;
#define DEBUG_ERROR     1
#define DEBUG_INFO           2
#define DEBUG_TRACE          3
#define print_pm8921(level, ...) \
	do { \
		if (logo_level  >= (level)) \
			printk(__VA_ARGS__); \
   } while (0) 
#endif /*CONFIG_VENDOR_EDIT*/
/* OPPO 2012-08-22 chendx Add end */

enum chg_fsm_state {
	FSM_STATE_OFF_0 = 0,
	FSM_STATE_BATFETDET_START_12 = 12,
	FSM_STATE_BATFETDET_END_16 = 16,
	FSM_STATE_ON_CHG_HIGHI_1 = 1,
	FSM_STATE_ATC_2A = 2,
	FSM_STATE_ATC_2B = 18,
	FSM_STATE_ON_BAT_3 = 3,
	FSM_STATE_ATC_FAIL_4 = 4 ,
	FSM_STATE_DELAY_5 = 5,
	FSM_STATE_ON_CHG_AND_BAT_6 = 6,
	FSM_STATE_FAST_CHG_7 = 7,
	FSM_STATE_TRKL_CHG_8 = 8,
	FSM_STATE_CHG_FAIL_9 = 9,
	FSM_STATE_EOC_10 = 10,
	FSM_STATE_ON_CHG_VREGOK_11 = 11,
	FSM_STATE_ATC_PAUSE_13 = 13,
	FSM_STATE_FAST_CHG_PAUSE_14 = 14,
	FSM_STATE_TRKL_CHG_PAUSE_15 = 15,
	FSM_STATE_START_BOOT = 20,
	FSM_STATE_FLCB_VREGOK = 21,
	FSM_STATE_FLCB = 22,
};

struct fsm_state_to_batt_status {
	enum chg_fsm_state	fsm_state;
	int			batt_state;
};

static struct fsm_state_to_batt_status map[] = {
	{FSM_STATE_OFF_0, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_START_12, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_END_16, POWER_SUPPLY_STATUS_UNKNOWN},
	/*
	 * for CHG_HIGHI_1 report NOT_CHARGING if battery missing,
	 * too hot/cold, charger too hot
	 */
	{FSM_STATE_ON_CHG_HIGHI_1, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ATC_2A, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ATC_2B, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ON_BAT_3, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_ATC_FAIL_4, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_DELAY_5, POWER_SUPPLY_STATUS_UNKNOWN },
	{FSM_STATE_ON_CHG_AND_BAT_6, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_FAST_CHG_7, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_TRKL_CHG_8, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_CHG_FAIL_9, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_EOC_10, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ON_CHG_VREGOK_11, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_ATC_PAUSE_13, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FAST_CHG_PAUSE_14, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_TRKL_CHG_PAUSE_15, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_START_BOOT, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB_VREGOK, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB, POWER_SUPPLY_STATUS_NOT_CHARGING},
};

enum chg_regulation_loop {
	VDD_LOOP = BIT(3),
	BAT_CURRENT_LOOP = BIT(2),
	INPUT_CURRENT_LOOP = BIT(1),
	INPUT_VOLTAGE_LOOP = BIT(0),
	CHG_ALL_LOOPS = VDD_LOOP | BAT_CURRENT_LOOP
			| INPUT_CURRENT_LOOP | INPUT_VOLTAGE_LOOP,
};

enum pmic_chg_interrupts {
	USBIN_VALID_IRQ = 0,
	USBIN_OV_IRQ,
	BATT_INSERTED_IRQ,
	VBATDET_LOW_IRQ,
	USBIN_UV_IRQ,
	VBAT_OV_IRQ,
	CHGWDOG_IRQ,
	VCP_IRQ,
	ATCDONE_IRQ,
	ATCFAIL_IRQ,
	CHGDONE_IRQ,
	CHGFAIL_IRQ,
	CHGSTATE_IRQ,
	LOOP_CHANGE_IRQ,
	FASTCHG_IRQ,
	TRKLCHG_IRQ,
	BATT_REMOVED_IRQ,
	BATTTEMP_HOT_IRQ,
	CHGHOT_IRQ,
	BATTTEMP_COLD_IRQ,
	CHG_GONE_IRQ,
	BAT_TEMP_OK_IRQ,
	COARSE_DET_LOW_IRQ,
	VDD_LOOP_IRQ,
	VREG_OV_IRQ,
	VBATDET_IRQ,
	BATFET_IRQ,
	PSI_IRQ,
	DCIN_VALID_IRQ,
	DCIN_OV_IRQ,
	DCIN_UV_IRQ,
	PM_CHG_MAX_INTS,
};

struct bms_notify {
	int			is_battery_full;
	int			is_charging;
	struct	work_struct	work;
};

/**
 * struct pm8921_chg_chip -device information
 * @dev:			device pointer to access the parent
 * @usb_present:		present status of usb
 * @dc_present:			present status of dc
 * @usb_charger_current:	usb current to charge the battery with used when
 *				the usb path is enabled or charging is resumed
 * @safety_time:		max time for which charging will happen
 * @update_time:		how frequently the userland needs to be updated
 * @max_voltage_mv:		the max volts the batt should be charged up to
 * @min_voltage_mv:		the min battery voltage before turning the FETon
 * @uvd_voltage_mv:		(PM8917 only) the falling UVD threshold voltage
 * @alarm_low_mv:		the battery alarm voltage low
 * @alarm_high_mv:		the battery alarm voltage high
 * @cool_temp_dc:		the cool temp threshold in deciCelcius
 * @warm_temp_dc:		the warm temp threshold in deciCelcius
 * @resume_voltage_delta:	the voltage delta from vdd max at which the
 *				battery should resume charging
 * @term_current:		The charging based term current
 * @charge_is_finished: 	charge is finished flag,only reset when 
 *                                charger connect or disconnect
 * @ftm_test_mode: RF WIFI AT FTM TEST MODE 
 */
struct pm8921_chg_chip {
	struct device			*dev;
	unsigned int			usb_present;
	unsigned int			dc_present;
	unsigned int			usb_charger_current;
	unsigned int			max_bat_chg_current;
	unsigned int			pmic_chg_irq[PM_CHG_MAX_INTS];
	unsigned int			ttrkl_time;
	unsigned int			update_time;
	unsigned int			max_voltage_mv;
	unsigned int			min_voltage_mv;
	unsigned int			uvd_voltage_mv;
	unsigned int			safe_current_ma;
	unsigned int			alarm_low_mv;
	unsigned int			alarm_high_mv;
	int				cool_temp_dc;
	int				warm_temp_dc;
	unsigned int			temp_check_period;
	/* OPPO 2012-08-07 chendx Add begin for oppo charge */
	#ifdef CONFIG_VENDOR_EDIT
	
	unsigned int			little_cold_bat_chg_current;
	unsigned int			normal_dcp_chg_current;
	unsigned int			normal_sdp_chg_current;
	unsigned int			little_cold_bat_voltage;
	unsigned int			normal_bat_voltage;
	unsigned int			normal_resume_voltage_delta;
	/*add for BTM */
	#ifdef FTM_CHARGE_MODE_FEATURE
	bool 	low_charge_mode; //60% stop charge
    bool 	high_charge_mode; //80% stop charge
    #endif
	bool	ftm_test_mode;
	bool chg_temp_test_mode;
	/* battery temp relative value */
	chg_cv_battery_temp_region_type mBatteryTempRegion;
	short 	mBatteryTempBoundT0;
	short 	mBatteryTempBoundT1;
	short 	mBatteryTempBoundT2;
	short 	mBatteryTempBoundT3;
	short 	mBatteryTempBoundT4;
	int batt_health;
	struct delayed_work		charger_valid_work;
	enum usb_chg_type pm8921_chg_type;
	struct delayed_work		recharge_monitor_work;
    bool	charge_is_finished;
	int battery_health;
	int battery_voltage; /* in millie volts */
	int charger_voltage; /*in millie volts*/
	int charge_current; /*in millie volts*/
	int mChargerCheckCounter;
	int mBadChargerCounter;
	int mBadBatteryCounter;
	chg_charger_status charger_status;
	chg_battery_status battery_status;
	/* for chg voltage */
	unsigned int			chg_voltage_channel;

	int recharging_counter;
	int vbatdet_mv;/*in millie volts*/

	int mhl_chg_current;
	int nonstanard_mhl_chg_current;
	bool stanard_mhl_chg;

	#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock pm8921_wake_lock;
	struct wake_lock prevent_suspend_lock;
	#endif

	#ifdef TEMPERATURE_CHARGE_TEST_FEATURE
	int chg_test_temp;  /* in celsius */
    #endif
	
	int soc_charge_counter;
	int soc_fall_status;
	int ocv_shutdown_counter;
	int soc_fall_counter;

	int eoc_vbatt_counter;
	int cv_long_counter;

	int batt_temp_high_counter;

    #ifdef BATT_CAPACITY_CALIB
    /*use for battery capacity calibrate*/
	int batt_capacity_pre;
	int report_calib_soc; /* in percentage */
	int capacity_saltate_counter; 
	#endif

	#ifdef AUTO_TEST_FEATURE
	//add by chendx for auto test system
	int		auto_test_timeout;
	int		auto_test_chargervol_high;
	int		auto_test_chargervol_low;
	int		auto_test_batvol_high;
	int		auto_test_bat_disconnect;
	int		auto_test_cool_charge_finished;
	int		auto_test_warm_charge_finished;
	#endif

	/* sw charge teoc handle */
	struct delayed_work 	teoc_work;
    bool			  safe_charge_teoc;
	unsigned int			r_sense;
	int battery_temp;  /* in celsius */
	int                     bad_charger_check_time;
	/*OPPO,Jiangsm add begin for bad charger type detecting logic,2013-1-23*/

	struct work_struct		cancel_charge_det;

/*OPPO,Jiangsm add end*/
	#endif /*CONFIG_VENDOR_EDIT*/
	/* OPPO 2012-08-07 chendx Add end */
	unsigned int			cool_bat_chg_current;
	unsigned int			warm_bat_chg_current;
	unsigned int			cool_bat_voltage;
	unsigned int			warm_bat_voltage;
	unsigned int			is_bat_cool;
	unsigned int			is_bat_warm;
	unsigned int			resume_voltage_delta;
	int				resume_charge_percent;
	unsigned int			term_current;
	unsigned int			vbat_channel;
	unsigned int			batt_temp_channel;
	unsigned int			batt_id_channel;
	struct power_supply		usb_psy;
	struct power_supply		dc_psy;
	struct power_supply		*ext_psy;
	struct power_supply		batt_psy;
	struct dentry			*dent;
	struct bms_notify		bms_notify;
	int				*usb_trim_table;
	bool				ext_charging;
	bool				ext_charge_done;
	bool				iusb_fine_res;
	DECLARE_BITMAP(enabled_irqs, PM_CHG_MAX_INTS);
	struct work_struct		battery_id_valid_work;
	int64_t				batt_id_min;
	int64_t				batt_id_max;
	int				trkl_voltage;
	int				weak_voltage;
	int				trkl_current;
	int				weak_current;
	int				vin_min;
	unsigned int			*thermal_mitigation;
	int				thermal_levels;
	struct delayed_work		update_heartbeat_work;
	struct delayed_work		eoc_work;
	struct delayed_work		unplug_check_work;
	struct delayed_work		vin_collapse_check_work;
	struct delayed_work		btc_override_work;
	struct wake_lock		eoc_wake_lock;
	enum pm8921_chg_cold_thr	cold_thr;
	enum pm8921_chg_hot_thr		hot_thr;
	int				rconn_mohm;
	enum pm8921_chg_led_src_config	led_src_config;
	bool				host_mode;
	bool				has_dc_supply;
	u8				active_path;
	int				recent_reported_soc;
	int				battery_less_hardware;
	int				ibatmax_max_adj_ma;
	int				btc_override;
	int				btc_override_cold_decidegc;
	int				btc_override_hot_decidegc;
	int				btc_delay_ms;
	bool				btc_panic_if_cant_stop_chg;
	int				stop_chg_upon_expiry;
	bool				disable_aicl;
	int				usb_type;
	bool				disable_chg_rmvl_wrkarnd;
};

/* user space parameter to limit usb current */
static unsigned int usb_max_current;
/*
 * usb_target_ma is used for wall charger
 * adaptive input current limiting only. Use
 * pm_iusbmax_get() to get current maximum usb current setting.
 */
static int usb_target_ma;
static int charging_disabled;
static int thermal_mitigation;

static struct pm8921_chg_chip *the_chip;

#define LPM_ENABLE_BIT	BIT(2)
static int pm8921_chg_set_lpm(struct pm8921_chg_chip *chip, int enable)
{
	int rc;
	u8 reg;

	rc = pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &reg);
	if (rc) {
		pr_err("pm8xxx_readb failed: addr=%03X, rc=%d\n",
				CHG_CNTRL, rc);
		return rc;
	}
	reg &= ~LPM_ENABLE_BIT;
	reg |= (enable ? LPM_ENABLE_BIT : 0);

	rc = pm8xxx_writeb(chip->dev->parent, CHG_CNTRL, reg);
	if (rc) {
		pr_err("pm_chg_write failed: addr=%03X, rc=%d\n",
				CHG_CNTRL, rc);
		return rc;
	}

	return rc;
}

static int pm_chg_write(struct pm8921_chg_chip *chip, u16 addr, u8 reg)
{
	int rc;

	rc = pm8xxx_writeb(chip->dev->parent, addr, reg);
	if (rc)
		pr_err("failed: addr=%03X, rc=%d\n", addr, rc);

	return rc;
}
/* OPPO 2012-08-08 chendx Add begin for 12025 charge */
#ifdef CONFIG_VENDOR_EDIT
static int get_prop_batt_status(struct pm8921_chg_chip *chip);
static int Pm8921_battery_set_normal_params(struct pm8921_chg_chip *chip);
static int pm8921_chg_temp_state_reset(struct pm8921_chg_chip *chip);
static int pm8921_battery_temp_handle(struct pm8921_chg_chip *chip);
static int pm8921_set_charge_mhl(struct pm8921_chg_chip *chip);
static int pm8921_chg_state_reset(struct pm8921_chg_chip *chip);
static void pm8921_check_charger_uovp(struct pm8921_chg_chip *chip,bool plugin);
static void pm8921_check_battery_uovp(struct pm8921_chg_chip *chip);
static void pm8921_battery_status_set(struct pm8921_chg_chip *chip,
											  chg_battery_status battery_status);
static void pm8921_charger_status_set(struct pm8921_chg_chip *chip,
											chg_charger_status charger_status);
static chg_charger_status pm8921_charger_status_get(struct pm8921_chg_chip *chip);
static chg_battery_status pm8921_battery_status_get(struct pm8921_chg_chip *chip);

static chg_charger_status pm8921_charger_status_get(struct pm8921_chg_chip *chip);

static chg_cv_battery_temp_region_type Pm8921_battery_temp_region_get(struct pm8921_chg_chip *chip);

static int teoc_work_schedule(bool on,struct pm8921_chg_chip *chip);
int pm8921_chg_connected(enum usb_chg_type chg_type);
static bool is_capable_of_charging(struct pm8921_chg_chip *chip);
static int get_prop_batt_health(struct pm8921_chg_chip *chip);

static int is_usb_chg_plugged_in(struct pm8921_chg_chip *chip);
static int is_dc_chg_plugged_in(struct pm8921_chg_chip *chip);


/* add  for batt capacity algorithm */
static bool is_charger_connect(struct pm8921_chg_chip *chip)
{
   enum usb_chg_type cur_chg_type;

   cur_chg_type = chip->pm8921_chg_type;

   	if(cur_chg_type == USB_SDP_CHARGER ||
		cur_chg_type == USB_DCP_CHARGER ||
		cur_chg_type == USB_NON_DCP_CHARGER
		)
		return true;
	else
		return false;
}

/* add for batt capacity algorithm */
bool is_charger_plugin(void)
{
   enum usb_chg_type cur_chg_type;

   if(the_chip == NULL){
      pr_err("the_chip not init\n");
	  return false;  
   }

   cur_chg_type = the_chip->pm8921_chg_type;

   	if(cur_chg_type == USB_SDP_CHARGER ||
		cur_chg_type == USB_DCP_CHARGER ||
		cur_chg_type == USB_NON_DCP_CHARGER
		)
		return true; 
	else
		return false; 
}
EXPORT_SYMBOL(is_charger_plugin);

/*get battery for oppo fuelgauger */
int oppo_get_battery_uvolts(u32 *batt_voltage)
{
	int rc=0;
	struct pm8xxx_adc_chan_result result;

	if(the_chip ==  NULL){
		pr_err("get battery uvolts failed\n");
		return -1;
	}

	rc = pm8xxx_adc_read(the_chip->vbat_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					the_chip->vbat_channel, rc);
		return rc;
	}

	*batt_voltage = (int)result.physical/1000;
	return rc;
}
EXPORT_SYMBOL(oppo_get_battery_uvolts);

/*function for get charge long time error status*/
static int get_prop_teoc_status(struct pm8921_chg_chip *chip)
{

/* OPPO 2013-04-08 chendx Add begin for auto test system */
 #ifdef AUTO_TEST_FEATURE
   if(chg_auto_test_feature && chip->auto_test_timeout == 1){
   	   chip->safe_charge_teoc = true;
   	   return 1;
   }
 #endif
/* OPPO 2013-04-08 chendx Add end */
   
   if(chip->safe_charge_teoc)
   	   return 1;
   else 
   	   return 0;
}

/*charge and system current is large,soc drop continue with 5minutes,waning user*/
static int get_prop_soc_fall_status(struct pm8921_chg_chip *chip)
{
   if(chip->soc_fall_status)
   	   return 1;
   else 
   	   return 0;
}



/*function for get charger voltage prop */
static int get_prop_chg_voltage(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->chg_voltage_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->chg_voltage_channel, rc);
		return rc;
	}
	
	pr_debug("chg voltage phy = %lld meas = 0x%llx,=%d\n", result.physical,
						result.measurement,chip->chg_voltage_channel);
    
	chip->charger_voltage = (int)result.physical/1000;

	/*  for charger voltage bad */
	if(pm8921_charger_status_get(chip) == CHARGER_STATUS_WEAK){
		chip->charger_voltage =  CHARGER_SOFT_UVP_VOLTAGE_UI - 30;//4370mv
	}else if(pm8921_charger_status_get(chip) == CHARGER_STATUS_OVER){
	    chip->charger_voltage =  CHARGER_SOFT_OVP_VOLTAGE_UI + 30;//5830mv
	}else{
	    if(is_charger_connect(chip)){
			/*good charger status and charger connect*/
			if(chip->charger_voltage >= CHARGER_SOFT_OVP_VOLTAGE)
				chip->charger_voltage =  CHARGER_SOFT_OVP_VOLTAGE_UI - 30;//5770mv
		    else if(chip->charger_voltage <= CHARGER_SOFT_UVP_VOLTAGE)
		    	chip->charger_voltage =  CHARGER_SOFT_UVP_VOLTAGE_UI + 30;//4430mv
		}
	}
	
	return chip->charger_voltage;
}

static int get_chg_voltage(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	/* OPPO 2013-04-08 chendx Add begin for auto test charger voltage high and low */
	#ifdef AUTO_TEST_FEATURE
	if(chg_auto_test_feature && chip->auto_test_chargervol_high)
		return CHARGER_SOFT_OVP_VOLTAGE_UI + 30;
	else if(chg_auto_test_feature && chip->auto_test_chargervol_low)
		return CHARGER_SOFT_UVP_VOLTAGE_UI - 30;
	#endif
    /* OPPO 2013-04-08 chendx Add end */

	rc = pm8xxx_adc_read(chip->chg_voltage_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->chg_voltage_channel, rc);
		return rc;
	}
	
	pr_debug("chg voltage phy = %lld meas = 0x%llx,=%d\n", result.physical,
						result.measurement,chip->chg_voltage_channel);
    
	chip->charger_voltage = (int)result.physical/1000;
	return chip->charger_voltage;
}

/* add for chg debug log*/
static int __dump_pm8921_regs(struct pm8921_chg_chip *chip)
{
	u8 temp;
	
	if(!chip){
		pr_err("chip is not init before\n");
		return -1;
	}

	/*charger status*/
	print_pm8921(DEBUG_ERROR,"BATT info####,capacity=%d%%,%dC,Batt voltage=%dmv,chg_type=%d\n",
			chip->recent_reported_soc,chip->battery_temp,chip->battery_voltage,chip->pm8921_chg_type);
	print_pm8921(DEBUG_ERROR,"Charge info,Charger voltage=%dmv,chg_current=%dmA,Batt health=%d\n",
			chip->charger_voltage,chip->charge_current,chip->batt_health);
	
	pm8xxx_readb(chip->dev->parent, CHG_IBAT_SAFE, &temp);
	print_pm8921(DEBUG_ERROR,"CHG_IBAT_SAFE 0x%0x\n",temp);
	pm8xxx_readb(chip->dev->parent, PBL_ACCESS2, &temp);
	print_pm8921(DEBUG_ERROR,"PBL_ACCESS2(0x05) 0x%0x\n",temp);
	pm8xxx_readb(chip->dev->parent, CHG_IBAT_MAX, &temp);
	print_pm8921(DEBUG_ERROR,"CHG_IBAT_MAX(0x205) 0x%0x\n",temp);
	pm8xxx_readb(chip->dev->parent, CHG_ITERM, &temp);
	print_pm8921(DEBUG_ERROR,"CHG_ITERM 0x%0x\n",temp);
	pm8xxx_readb(chip->dev->parent, CHG_VIN_MIN, &temp);
	print_pm8921(DEBUG_ERROR,"CHG_VIN_MIN 0x%0x\n",temp);

	return 0;

}

/* add begin for  batt capacity algorithm */
#ifdef BATT_CAPACITY_CALIB
#define CAPACITY_SALTATE_CHANGE_MVOLTS 3750
/*battery high than 3600mv,set the capacity saltate to 60s*/
#define POLL_SALTATE_COUNTER 15//20
/*battery low than 3600mv,set the capacity saltate to 30s*/
#define POLL_SALTATE_COUNTER_LOW 5
#define BATT_CAPACITY_FULL 100
#define BATT_CAPACITY_INVALID_LEVEL 2
#define BATT_VOL_VERY_LOW 3200
#define BATT_VOL_VERY_LOW_POLL_TIME 3
#define VOL_VERY_LOW_SHUTDWON_POLL 5 //36s
#define VOL_VERY_LOW_FORCE_SHUTDWON_POLL 8 //60s

static bool vol_very_low_check(struct pm8921_chg_chip *chip)
{

	static int poll_shutdown_counter=0;
	static bool vol_very_low = false;

	/*
	* XXXX: vol very low handle*/
	if(chip->battery_voltage < BATT_VOL_VERY_LOW){
		pr_info("warning,vol low %d\n",poll_shutdown_counter);
		poll_shutdown_counter++;
	}else{
		/*reset status very low voltage*/
		poll_shutdown_counter=0;
		vol_very_low = false;
	}

	if(poll_shutdown_counter >= BATT_VOL_VERY_LOW_POLL_TIME){
		vol_very_low = true;
	}

	if(poll_shutdown_counter >= VOL_VERY_LOW_SHUTDWON_POLL){
		pr_info("warning,force shutdown when vol very low!!!");
		/*will shutdown quickly*/
		if(chip->batt_capacity_pre >=2)
			chip->batt_capacity_pre = 2;
		vol_very_low = true;
	}

	if(poll_shutdown_counter >= VOL_VERY_LOW_FORCE_SHUTDWON_POLL)
	{
	   pr_warning("vbatt low force shutdown\n");
	   kernel_power_off();
	}
	
	return vol_very_low;

}

#define OCV_VBATT_LOW 3600
#define VBATT_LOW 3200
#define OCV_VBATT_SHUTDOWN_COUNTER 5
#define SOC_REDUCE_FORCE_CURRENT 100
static int vbatt_ocv_low_shutdown_check(struct pm8921_chg_chip *chip)
{
    int ocv_vbatt=0;

	if (!(is_usb_chg_plugged_in(chip)
				&& !(is_dc_chg_plugged_in(chip)))) {
	    /* No charger active */
		ocv_vbatt = get_bms_ocv_vbatt(chip->charge_current*1000,chip->battery_voltage*1000);
		if(ocv_vbatt <= OCV_VBATT_LOW || chip->battery_voltage <= VBATT_LOW)
	   		chip->ocv_shutdown_counter ++;
		else
			chip->ocv_shutdown_counter = 0;

		if(chip->ocv_shutdown_counter >= (OCV_VBATT_SHUTDOWN_COUNTER *6)){	
			pr_warning("warning,OCV LOW force calling kernel_power_off\n");
			chip->ocv_shutdown_counter = 0;
			kernel_power_off();
			return 0;
		}else if(chip->ocv_shutdown_counter >= (OCV_VBATT_SHUTDOWN_COUNTER *2)){
		    pr_warning("warning,OCV LOW force soc to 0\n");
		    chip->batt_capacity_pre = 0;
			return 1;
		}else if(chip->ocv_shutdown_counter >= (OCV_VBATT_SHUTDOWN_COUNTER *1)){
		    pr_warning("warning,OCV LOW force soc to 1\n");
		    chip->batt_capacity_pre = 1;
			return 1;
		}
		
	}else{
		ocv_vbatt = get_bms_ocv_vbatt(chip->charge_current*1000,chip->battery_voltage*1000);
		if(ocv_vbatt <= OCV_VBATT_LOW)
	   		chip->ocv_shutdown_counter ++;
		else
			chip->ocv_shutdown_counter = 0;

		if(chip->ocv_shutdown_counter >= OCV_VBATT_SHUTDOWN_COUNTER){
			chip->ocv_shutdown_counter = 0;
			if(chip->recent_reported_soc == 0 &&
			   chip->charge_current >= SOC_REDUCE_FORCE_CURRENT &&
			   chip->soc_fall_status){
			        pr_warning("charger plugin,OCV LOW force shutdown\n");
					kernel_power_off();
			}
			return 0;
		}
	}
	return 0;
}

/*charge and system current is large,soc drop continue with 5minutes,waning user*/
#define SOC_FALL_SHUTDOWN_COUNTER 20 //2minutes
#define SOC_FALL_CURRENT 150
#define SOC_SHUTDOWN_CURRENT 100
#define SOC_FALL_COUNTER 20 //2minutes

#define SOC_CHARGE_LEVEL__0 98
#define SOC_CHARGE_LEVEL__1 93
#define SOC_CHARGE_LEVEL__2 80

#define SOC_CHARGE_TIME__1 60
#define SOC_CHARGE_TIME__0 100
#define SOC_CHARGE_TIME__2 15//20
static int soc_check_with_charge(struct pm8921_chg_chip *chip,
												int percent_soc, 
												int *charge_calib_soc)
{  
   chip->soc_charge_counter++;

   /*soc fall shutdown system current high with charge*/
	if(chip->report_calib_soc == 0 &&
		 chip->charge_current >= SOC_SHUTDOWN_CURRENT)
	{
		 chip->soc_fall_counter++;
	}else{
	     chip->soc_fall_counter = 0;
		 chip->soc_fall_status = false;
	}

	if( chip->soc_fall_counter >=SOC_FALL_SHUTDOWN_COUNTER){	 
		  pr_warning("warning,Soc fall shutdown!!\n");
	      /*soc drop waring*/
	      chip->soc_fall_status = true;
		  chip->soc_fall_counter = 0;
	}

   /*
    * XXXX: check soc increase with charge*/
   if(percent_soc >= chip->batt_capacity_pre){

	    /*charge level control fo charge time*/
		if(chip->report_calib_soc >= SOC_CHARGE_LEVEL__0)
			chip->capacity_saltate_counter = chip->capacity_saltate_counter%(SOC_CHARGE_TIME__0);
	    else if(chip->report_calib_soc >= SOC_CHARGE_LEVEL__1)
			chip->capacity_saltate_counter = chip->capacity_saltate_counter%(SOC_CHARGE_TIME__1);
		else if(chip->report_calib_soc >= SOC_CHARGE_LEVEL__2)
			chip->capacity_saltate_counter = chip->capacity_saltate_counter%(SOC_CHARGE_TIME__2);
		else			
  			chip->capacity_saltate_counter = chip->capacity_saltate_counter%POLL_SALTATE_COUNTER;
			
		if(chip->capacity_saltate_counter == 0){
		    if(percent_soc > chip->batt_capacity_pre){
				/*30seconds update capacity 1%*/
				*charge_calib_soc = chip->batt_capacity_pre + 1;
			}else{
				*charge_calib_soc = chip->batt_capacity_pre;
			}
				
		}else{
			 /*Charging status capacity can not reduce*/
			 *charge_calib_soc = chip->batt_capacity_pre;
		}
		return 0;
   }else{ 
	   if(chip->soc_charge_counter%SOC_FALL_COUNTER)
	   {
	      /*if continute to check fall 30s,soc fall 1%*/
		  if(chip->charge_current > 0 && chip->charge_current >= SOC_FALL_CURRENT){
	         pr_warning("warning,Soc reduce when system current very large\n");
		     /*
		  	  * XXXX: check soc drop with charge*/
		 	  *charge_calib_soc = chip->batt_capacity_pre - 1;
		  }else{
		      *charge_calib_soc = chip->batt_capacity_pre;
		  }
		  
		  chip->soc_charge_counter = 0;   
		  return 0;
	   }
	   
	   *charge_calib_soc = chip->batt_capacity_pre;
   }
   
   return 0;
}

/*Soc low shutdown handle,system will shutdown 60s later when soc is 1%*/
#define SHUTDWON_SOC_LOW 1
#define SHUTDWON_LOW_COUNTER 5

int soc_low_shutdown(int charge_now,struct pm8921_chg_chip *chip)
{
    static int soc_low_shutdown_counter = 0;
    if(chip->report_calib_soc == SHUTDWON_SOC_LOW && !charge_now)
    {
       soc_low_shutdown_counter ++;
       if(soc_low_shutdown_counter>=SHUTDWON_LOW_COUNTER){
	   	   pr_info("Soc low shutdown!!\n");
		   soc_low_shutdown_counter= 0;
	   	   chip->batt_capacity_pre = 0;
		   chip->report_calib_soc = 0;
		   power_supply_changed(&chip->batt_psy);
		   return 1;
       }   	  
    }else{
    	soc_low_shutdown_counter= 0;
    }

	return 0;
}

/*Soc force full handle*/
static bool soc_force_full_capable(struct pm8921_chg_chip *chip)
{
    bool soc_force_full = false;

	if(Pm8921_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__NORMAL)
		/* 10 C <Tbatt <= 45C*/
		soc_force_full = true;
	else if(Pm8921_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__COOL)
	    /* 0 C <Tbatt <= 10C*/
		soc_force_full = true;
	#if 0
	else if(Pm8921_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__WARM)
	    /* 45 C <Tbatt <= 55C*/
		soc_force_full = true;
	#endif

	return soc_force_full;

}

/*discharge rate adjust*/
#define SOC_FULL_FALL_COUNTER 14*10
#define SOC_FULL_FALL_MIDDLE_COUNTER 6*10
#define SOC_FULL_FALL_LOW_RATE_5 (5*10)
#define SOC_FULL_FALL_LOW_RATE_4 (4*10)
#define SOC_FULL_FALL_LOW_RATE_3 (3*10)
#define SOC_FULL_FALL_LOW_RATE_2 (2*10)
#define SOC_FULL_FALL_LOW_RATE_1 (1*10)

#define SOC_FULL_FALL_LOW_RATE_0 5

static int get_rate_with_discharge_current(int current_now)	
{
   if(current_now>0 && current_now <= 200)
   	  return 0;
   else if(current_now>200 && current_now <= 400)
   	  return 1;
   else if(current_now>400 && current_now <= 600)
   	  return 2;
   else if(current_now>600 && current_now <= 800)
   	  return 3;
   else if(current_now>800 && current_now <= 1600)
   	  return 4;
   else if(current_now>1600)
   	  return 5;
   else 
   	  return -1;
}

#define SOC_COMPENSATE_LEVEL 70
#define SOC_COMPENSATE_VALUE 3
static int soc_level_compensate_algorithm(int percent_soc)
{
    int compensate_soc=0;

	compensate_soc = percent_soc;

	if(compensate_soc >= SOC_COMPENSATE_LEVEL)
		compensate_soc = compensate_soc + SOC_COMPENSATE_VALUE;
	else if(compensate_soc >= (SOC_COMPENSATE_LEVEL - 10))
		compensate_soc = compensate_soc + SOC_COMPENSATE_VALUE - 1;
	else if(compensate_soc >= (SOC_COMPENSATE_LEVEL - 20))
		compensate_soc = compensate_soc + SOC_COMPENSATE_VALUE - 2;

	/* XXXX add Soc valid range protect */
	if(compensate_soc >= 100)
		compensate_soc = 100;
	else if(compensate_soc <= 0)
		compensate_soc = 0;

	return compensate_soc;
	
}

#define SOC_FORCE_CALIBRATE_COUNTER (22*10)
static int soc_force_calibrate_of_time(struct pm8921_chg_chip *chip,
															int percent_soc)
{
      static int backup_soc=0;
	  static int soc_force_counter=0;
	  int discharge_rate=-1;
	  int soc_now = percent_soc;

	  if(chip->charge_current <= 0){
	  		soc_force_counter = 0;
	  	    /*not need calibrate of charge*/
	  		return soc_now;
	  }

	  if(backup_soc != percent_soc){
	      /*reset soc counter*/
	      soc_force_counter = 0;
		  backup_soc = percent_soc;
	  }else{
	        if(chip->charge_current > 0)
	  			discharge_rate = get_rate_with_discharge_current(chip->charge_current);

			soc_force_counter = soc_force_counter + discharge_rate + 1;
			if(soc_force_counter >= SOC_FORCE_CALIBRATE_COUNTER)
			{
			    pr_info("Soc long time not change!force of time soc_now =%d%%,%d",
					soc_now,soc_force_counter);
				soc_force_counter = 0;
				if(soc_now > 0)
					soc_now --;
				else
					soc_now = 0;
				return soc_now;
			}	
	  }

	  return soc_now;

}

static int batt_report_capacity_calib(struct pm8921_chg_chip *chip,
															int percent_soc)
{
	#ifdef BATT_CAPACITY_CALIB
	int charging_now=0;
	int full_discharge_rate=0;
    int capacity_calib;
	int charge_calib_soc;
	int rc=0;

	if(chip == NULL){
		pr_err("chip is NULL\n");
		return chip->batt_capacity_pre;
	}

	capacity_calib = percent_soc;
	/* XXXX add Soc valid range protect */
	if(capacity_calib >= 100)
		capacity_calib = 100;
	else if(capacity_calib <= 0)
		capacity_calib = 0;

    #if 0
	/*delete 2012-3-5*/
	if(capacity_calib < 13)
		capacity_calib = 10*capacity_calib/12;
	else if(capacity_calib < 88)
		capacity_calib = (80*capacity_calib - 200)/76;
	else
		capacity_calib = (10*capacity_calib + 200)/12;
	#endif
	
	charging_now = is_charger_connect(chip);

    /*soc compensate algorithm of discharge*/
	capacity_calib = soc_level_compensate_algorithm(capacity_calib);
	
    if(debug_feature){
		pr_info("chip->soc_charge_counter =%d\n",chip->soc_charge_counter);
		pr_info("charging status =%d,bms capacity  =[%d/%d],counter=%d\n",
				charging_now,percent_soc,capacity_calib, chip->capacity_saltate_counter); 
		pr_info("recent_reported_soc =%d%%,chip->report_calib_soc =%d%%,chip->batt_capacity_pre=%d%%\n",
			     chip->recent_reported_soc,chip->report_calib_soc,chip->batt_capacity_pre); 
    }

	/*
	 * Charge finished handle, BMS Soc not report 100% when charge finished,
	 * then force report 100% with high T > 0 C
	 */
	if(chip->charge_is_finished
		&& charging_now 
		&& soc_force_full_capable(chip))
	{
	    /*force report 100% capacity if charge finished*/
		if(debug_feature){
			 pr_info("force report BATT_CAPACITY_FULL\n");
		}
		
		capacity_calib = BATT_CAPACITY_FULL;
		goto soc_calib;
	}

    /* XXXX: OCV LOW shutdown*/
	rc = vbatt_ocv_low_shutdown_check(chip);
    if(rc)
	{
	    capacity_calib = chip->batt_capacity_pre;
		goto soc_calib;
	}

	/*
	* XXXX: vol very low handle*/
	if(vol_very_low_check(chip)){
		pr_info("warning,vol very low %d\n",chip->batt_capacity_pre);
		if(chip->batt_capacity_pre > 0)
			capacity_calib = chip->batt_capacity_pre - 1;
		goto soc_calib;
	}
		
	chip->capacity_saltate_counter++;

    /*1% soc shutdown handle*/
	rc = soc_low_shutdown(charging_now,chip);
	if(rc)
	{
		goto soc_calib;
	}

	/*fliter error capacity when read gague capacity error*/
	if(charging_now){
		soc_check_with_charge(chip,capacity_calib,&charge_calib_soc);
		capacity_calib = charge_calib_soc;
		goto soc_calib;
		
	}else{
	    if(chip->battery_voltage >= CAPACITY_SALTATE_CHANGE_MVOLTS){
			full_discharge_rate = get_rate_with_discharge_current(chip->charge_current);
			chip->capacity_saltate_counter = chip->capacity_saltate_counter + full_discharge_rate;
			if(chip->report_calib_soc >= 95){
				if(chip->capacity_saltate_counter >= SOC_FULL_FALL_COUNTER)
					chip->capacity_saltate_counter = 0;
			}else if(chip->report_calib_soc >= 90){
				if(chip->capacity_saltate_counter >= SOC_FULL_FALL_MIDDLE_COUNTER)
					chip->capacity_saltate_counter = 0;
			}else if(chip->report_calib_soc >= 75){
				if(chip->capacity_saltate_counter >= SOC_FULL_FALL_LOW_RATE_3)
					chip->capacity_saltate_counter = 0;
			}else{
				if(chip->capacity_saltate_counter >= SOC_FULL_FALL_MIDDLE_COUNTER)
					chip->capacity_saltate_counter = 0;
			}
		}else{
		    if((chip->battery_voltage <= 3450 && chip->charge_current >= 500) ||
				(chip->battery_voltage <= 3500 && chip->charge_current <= 500) ||
				(chip->battery_voltage <= 3550 && chip->charge_current <= 300)){
				if(chip->capacity_saltate_counter >= SOC_FULL_FALL_LOW_RATE_0)
					chip->capacity_saltate_counter = 0;
			}else{
				full_discharge_rate = get_rate_with_discharge_current(chip->charge_current);
				chip->capacity_saltate_counter = chip->capacity_saltate_counter + full_discharge_rate;
				if(chip->capacity_saltate_counter >= SOC_FULL_FALL_LOW_RATE_4)
					chip->capacity_saltate_counter = 0;
			}
		}
	
	    if(chip->capacity_saltate_counter == 0)
	    {
	        /*Discharge status battery soc reduce 1% 60s*/
			if(capacity_calib < chip->batt_capacity_pre)
				capacity_calib = chip->batt_capacity_pre - 1;
			else
				capacity_calib = chip->batt_capacity_pre;
		}else{
			/*DisCharging status capacity can not increase*/
			capacity_calib = chip->batt_capacity_pre;
		}		
	}
	
soc_calib:	
	
	/* XXXX add Soc valid range protect */
	if(capacity_calib >= 100)
		capacity_calib = 100;
	else if(capacity_calib <= 0)
		capacity_calib = 0;

    /*XXX : calibrate soc with time*/
	capacity_calib = soc_force_calibrate_of_time(chip,capacity_calib);

	if(!charging_now && capacity_calib > chip->report_calib_soc){
			if (!(is_usb_chg_plugged_in(chip)
				&& !(is_dc_chg_plugged_in(chip)))) {
			/* XXX: capacity_calib can not up when discharge*/
			capacity_calib = chip->report_calib_soc;
		}
	}
		
	chip->batt_capacity_pre = capacity_calib;
	
	return capacity_calib;
	#else
		return percent_soc;
	#endif
}

/*10minutes*/
#define SOC_FALL_RATE 10
#define MINUTES_TIMES 60
int update_presoc_long_time_sleep(int bms_soc,unsigned long sleep_time)
{
    int soc_delta;
	static unsigned long sleep_time_all=0;
    if(the_chip == NULL){
		pr_err("the_chip not init ready\n");
		return -1;
    }
	
    pr_info("resume ,bms_soc =%d%%,%d%%,%d%%\n",
			bms_soc,the_chip->batt_capacity_pre,the_chip->report_calib_soc);
	
	if(bms_soc <= the_chip->report_calib_soc){
		sleep_time_all = sleep_time_all + sleep_time;
		/*sleep soc fall rate 10minutes max*/
		soc_delta = sleep_time_all/(SOC_FALL_RATE*MINUTES_TIMES);
		sleep_time_all = sleep_time_all%(SOC_FALL_RATE*MINUTES_TIMES);
		the_chip->report_calib_soc = the_chip->report_calib_soc - soc_delta;

		if(the_chip->report_calib_soc >= 100)
			the_chip->report_calib_soc = 100;
		else if(the_chip->report_calib_soc <= 0)
			the_chip->report_calib_soc  = 0;
		
		if(the_chip->report_calib_soc <= bms_soc)
			the_chip->report_calib_soc = bms_soc;
			
		
		
		if(the_chip->report_calib_soc >= the_chip->batt_capacity_pre)
			/*report calib soc can not change more*/
			the_chip->report_calib_soc = the_chip->batt_capacity_pre;
			
		the_chip->batt_capacity_pre = the_chip->report_calib_soc;
		
		pr_info("resume calibrate ,%d%%,%d%%\n",
			the_chip->batt_capacity_pre,the_chip->report_calib_soc);
		
		power_supply_changed(&the_chip->batt_psy);
	}

	return 0;

}
EXPORT_SYMBOL(update_presoc_long_time_sleep);
#endif

#endif
/* OPPO 2012-08-06 chendx Add end */

static int pm_chg_masked_write(struct pm8921_chg_chip *chip, u16 addr,
							u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = pm8xxx_readb(chip->dev->parent, addr, &reg);
	if (rc) {
		pr_err("pm8xxx_readb failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	reg &= ~mask;
	reg |= val & mask;
	rc = pm_chg_write(chip, addr, reg);
	if (rc) {
		pr_err("pm_chg_write failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	return 0;
}

static int pm_chg_get_rt_status(struct pm8921_chg_chip *chip, int irq_id)
{
	return pm8xxx_read_irq_stat(chip->dev->parent,
					chip->pmic_chg_irq[irq_id]);
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_usb_chg_plugged_in(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_dc_chg_plugged_in(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
}

static int is_batfet_closed(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, BATFET_IRQ);
}
#define CAPTURE_FSM_STATE_CMD	0xC2
#define READ_BANK_7		0x70
#define READ_BANK_4		0x40
static int pm_chg_get_fsm_state(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int err = 0, ret = 0;

	temp = CAPTURE_FSM_STATE_CMD;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	temp = READ_BANK_7;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		goto err_out;
	}
	/* get the lower 4 bits */
	ret = temp & 0xF;

	temp = READ_BANK_4;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		goto err_out;
	}
	/* get the upper 1 bit */
	ret |= (temp & 0x1) << 4;

err_out:
	if (err)
		return err;

	return  ret;
}

#define READ_BANK_6		0x60
static int pm_chg_get_regulation_loop(struct pm8921_chg_chip *chip)
{
	u8 temp, data;
	int err = 0;

	temp = READ_BANK_6;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &data);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		goto err_out;
	}

err_out:
	if (err)
		return err;

	/* return the lower 4 bits */
	return data & CHG_ALL_LOOPS;
}

#define CHG_USB_SUSPEND_BIT  BIT(2)
static int pm_chg_usb_suspend_enable(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_USB_SUSPEND_BIT,
			enable ? CHG_USB_SUSPEND_BIT : 0);
}

#define CHG_EN_BIT	BIT(7)
static int pm_chg_auto_enable(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_EN_BIT,
				enable ? CHG_EN_BIT : 0);
}

#define CHG_FAILED_CLEAR	BIT(0)
#define ATC_FAILED_CLEAR	BIT(1)
static int pm_chg_failed_clear(struct pm8921_chg_chip *chip, int clear)
{
	int rc;

	rc = pm_chg_masked_write(chip, CHG_CNTRL_3, ATC_FAILED_CLEAR,
				clear ? ATC_FAILED_CLEAR : 0);
	rc |= pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_FAILED_CLEAR,
				clear ? CHG_FAILED_CLEAR : 0);
	return rc;
}

#define CHG_CHARGE_DIS_BIT	BIT(1)
static int pm_chg_charge_dis(struct pm8921_chg_chip *chip, int disable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL, CHG_CHARGE_DIS_BIT,
				disable ? CHG_CHARGE_DIS_BIT : 0);
}

static int pm_is_chg_charge_dis(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	return  temp & CHG_CHARGE_DIS_BIT;
}
#define PM8921_CHG_V_MIN_MV	3240
#define PM8921_CHG_V_STEP_MV	20
#define PM8921_CHG_V_STEP_10MV_OFFSET_BIT	BIT(7)
#define PM8921_CHG_VDDMAX_MAX	4500
#define PM8921_CHG_VDDMAX_MIN	3400
#define PM8921_CHG_V_MASK	0x7F
static int __pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int remainder;
	u8 temp = 0;

	if (voltage < PM8921_CHG_VDDMAX_MIN
			|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;

	remainder = voltage % 20;
	if (remainder >= 10) {
		temp |= PM8921_CHG_V_STEP_10MV_OFFSET_BIT;
	}

	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_write(chip, CHG_VDD_MAX, temp);
}

static int pm_chg_vddmax_get(struct pm8921_chg_chip *chip, int *voltage)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VDD_MAX, &temp);
	if (rc) {
		pr_err("rc = %d while reading vdd max\n", rc);
		*voltage = 0;
		return rc;
	}
	*voltage = (int)(temp & PM8921_CHG_V_MASK) * PM8921_CHG_V_STEP_MV
							+ PM8921_CHG_V_MIN_MV;
	if (temp & PM8921_CHG_V_STEP_10MV_OFFSET_BIT)
		*voltage =  *voltage + 10;
	return 0;
}

static int pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int current_mv, ret, steps, i;
	bool increase;

	ret = 0;

	if (voltage < PM8921_CHG_VDDMAX_MIN
		|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	ret = pm_chg_vddmax_get(chip, &current_mv);
	if (ret) {
		pr_err("Failed to read vddmax rc=%d\n", ret);
		return -EINVAL;
	}
	if (current_mv == voltage)
		return 0;

	/* Only change in increments when USB is present */
	if (is_usb_chg_plugged_in(chip)) {
		if (current_mv < voltage) {
			steps = (voltage - current_mv) / PM8921_CHG_V_STEP_MV;
			increase = true;
		} else {
			steps = (current_mv - voltage) / PM8921_CHG_V_STEP_MV;
			increase = false;
		}
		for (i = 0; i < steps; i++) {
			if (increase)
				current_mv += PM8921_CHG_V_STEP_MV;
			else
				current_mv -= PM8921_CHG_V_STEP_MV;
			ret |= __pm_chg_vddmax_set(chip, current_mv);
		}
	}
	ret |= __pm_chg_vddmax_set(chip, voltage);
	return ret;
}

#define PM8921_CHG_VDDSAFE_MIN	3400
#define PM8921_CHG_VDDSAFE_MAX	4500
static int pm_chg_vddsafe_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VDDSAFE_MIN
			|| voltage > PM8921_CHG_VDDSAFE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VDD_SAFE, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VBATDET_MIN	3240
#define PM8921_CHG_VBATDET_MAX	5780
static int pm_chg_vbatdet_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VBATDET_MIN
			|| voltage > PM8921_CHG_VBATDET_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	
	/* OPPO 2012-08-20 chendx Add begin for recharging */
	#ifdef CONFIG_VENDOR_EDIT
	chip->vbatdet_mv = voltage;
	#endif
	/* OPPO 2012-08-20 chendx Add end */
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VBAT_DET, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VINMIN_MIN_MV	3800
#define PM8921_CHG_VINMIN_STEP_MV	100
#define PM8921_CHG_VINMIN_USABLE_MAX	6500
#define PM8921_CHG_VINMIN_USABLE_MIN	4300
#define PM8921_CHG_VINMIN_MASK		0x1F
static int pm_chg_vinmin_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VINMIN_USABLE_MIN
			|| voltage > PM8921_CHG_VINMIN_USABLE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_VINMIN_MIN_MV) / PM8921_CHG_VINMIN_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VIN_MIN, PM8921_CHG_VINMIN_MASK,
									temp);
}

static int pm_chg_vinmin_get(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc, voltage_mv;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VIN_MIN, &temp);
	temp &= PM8921_CHG_VINMIN_MASK;

	voltage_mv = PM8921_CHG_VINMIN_MIN_MV +
			(int)temp * PM8921_CHG_VINMIN_STEP_MV;

	return voltage_mv;
}

#define PM8917_USB_UVD_MIN_MV	3850
#define PM8917_USB_UVD_MAX_MV	4350
#define PM8917_USB_UVD_STEP_MV	100
#define PM8917_USB_UVD_MASK	0x7
static int pm_chg_uvd_threshold_set(struct pm8921_chg_chip *chip, int thresh_mv)
{
	u8 temp;

	if (thresh_mv < PM8917_USB_UVD_MIN_MV
			|| thresh_mv > PM8917_USB_UVD_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", thresh_mv);
		return -EINVAL;
	}
	temp = (thresh_mv - PM8917_USB_UVD_MIN_MV) / PM8917_USB_UVD_STEP_MV;
	return pm_chg_masked_write(chip, OVP_USB_UVD,
				PM8917_USB_UVD_MASK, temp);
}

/* OPPO 2012-08-09 chendx Modify begin for ibat value */
#ifndef CONFIG_VENDOR_EDIT
#define PM8921_CHG_IBATMAX_MIN	325
#else
#define PM8921_CHG_IBATMAX_MIN	225	
#endif
/* OPPO 2012-08-09 chendx Modify end */
#define PM8921_CHG_IBATMAX_MAX	3025
#define PM8921_CHG_I_MIN_MA	225
#define PM8921_CHG_I_STEP_MA	50
#define PM8921_CHG_I_MASK	0x3F
static int pm_chg_ibatmax_get(struct pm8921_chg_chip *chip, int *ibat_ma)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_IBAT_MAX, &temp);
	if (rc) {
		pr_err("rc = %d while reading ibat max\n", rc);
		*ibat_ma = 0;
		return rc;
	}
	*ibat_ma = (int)(temp & PM8921_CHG_I_MASK) * PM8921_CHG_I_STEP_MA
							+ PM8921_CHG_I_MIN_MA;
	return 0;
}

static int pm_chg_ibatmax_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_IBATMAX_MIN
			|| chg_current > PM8921_CHG_IBATMAX_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	return pm_chg_masked_write(chip, CHG_IBAT_MAX, PM8921_CHG_I_MASK, temp);
}

#define PM8921_CHG_IBATSAFE_MIN	225
#define PM8921_CHG_IBATSAFE_MAX	3375
static int pm_chg_ibatsafe_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_IBATSAFE_MIN
			|| chg_current > PM8921_CHG_IBATSAFE_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	
	/* OPPO 2012-08-07 chendx Modify begin for r-sense */
	#ifdef CONFIG_VENDOR_EDIT
	/*12025 r-sense 20mohm,qualcomm default is 10mohm*/
	if(chip->r_sense == 20){
		temp &= ~PM8921_CHG_RSENSE_MASK;
		temp |= PM8921_CHG_RSENSE_20MOHM;
	}else{
		temp &= ~PM8921_CHG_RSENSE_MASK;
		temp |= PM8921_CHG_RSENSE_10MOHM;
	}
	return pm_chg_masked_write(chip, CHG_IBAT_SAFE,
					PM8921_CHG_SAFE_MASK, temp);
	#else	
	return pm_chg_masked_write(chip, CHG_IBAT_SAFE,
						PM8921_CHG_I_MASK, temp);
	#endif
	/* OPPO 2012-08-07 chendx Modify end */
}

#define PM8921_CHG_ITERM_MIN_MA		50
#define PM8921_CHG_ITERM_MAX_MA		200
#define PM8921_CHG_ITERM_STEP_MA	10
#define PM8921_CHG_ITERM_MASK		0xF
static int pm_chg_iterm_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_ITERM_MIN_MA
			|| chg_current > PM8921_CHG_ITERM_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}

	temp = (chg_current - PM8921_CHG_ITERM_MIN_MA)
				/ PM8921_CHG_ITERM_STEP_MA;
	return pm_chg_masked_write(chip, CHG_ITERM, PM8921_CHG_ITERM_MASK,
					 temp);
}

static int pm_chg_iterm_get(struct pm8921_chg_chip *chip, int *chg_current)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_ITERM, &temp);
	if (rc) {
		pr_err("err=%d reading CHG_ITEM\n", rc);
		*chg_current = 0;
		return rc;
	}
	temp &= PM8921_CHG_ITERM_MASK;
	*chg_current = (int)temp * PM8921_CHG_ITERM_STEP_MA
					+ PM8921_CHG_ITERM_MIN_MA;
	return 0;
}

struct usb_ma_limit_entry {
	int	usb_ma;
	u8	value;
};

/* USB Trim tables */
static int usb_trim_8038_table[USB_TRIM_ENTRIES] = {
	0x0,
	0x0,
	-0x9,
	0x0,
	-0xD,
	0x0,
	-0x10,
	-0x11,
	0x0,
	0x0,
	-0x25,
	0x0,
	-0x28,
	0x0,
	-0x32,
	0x0
};

static int usb_trim_8917_table[USB_TRIM_ENTRIES] = {
	0x0,
	0x0,
	0xA,
	0xC,
	0x10,
	0x10,
	0x13,
	0x14,
	0x13,
	0x3,
	0x1A,
	0x1D,
	0x1D,
	0x21,
	0x24,
	0x26
};

/* Maximum USB  setting table */
static struct usb_ma_limit_entry usb_ma_table[] = {
	{100, 0x0},
	{200, 0x1},
	{500, 0x2},
	{600, 0x3},
	{700, 0x4},
	{800, 0x5},
	{850, 0x6},
	{900, 0x8},
	{950, 0x7},
	{1000, 0x9},
	{1100, 0xA},
	{1200, 0xB},
	{1300, 0xC},
	{1400, 0xD},
	{1500, 0xE},
	{1600, 0xF},
};

#define REG_SBI_CONFIG			0x04F
#define PAGE3_ENABLE_MASK		0x6
#define USB_OVP_TRIM_MASK		0x3F
#define USB_OVP_TRIM_PM8917_MASK	0x7F
#define USB_OVP_TRIM_MIN		0x00
#define REG_USB_OVP_TRIM_ORIG_LSB	0x10A
#define REG_USB_OVP_TRIM_ORIG_MSB	0x09C
#define REG_USB_OVP_TRIM_PM8917		0x2B5
#define REG_USB_OVP_TRIM_PM8917_BIT	BIT(0)
static int pm_chg_usb_trim(struct pm8921_chg_chip *chip, int index)
{
	u8 temp, sbi_config, msb, lsb, mask;
	s8 trim;
	int rc = 0;
	static u8 usb_trim_reg_orig = 0xFF;

	/* No trim data for PM8921 */
	if (!chip->usb_trim_table)
		return 0;

	if (usb_trim_reg_orig == 0xFF) {
		rc = pm8xxx_readb(chip->dev->parent,
				REG_USB_OVP_TRIM_ORIG_MSB, &msb);
		if (rc) {
			pr_err("error = %d reading sbi config reg\n", rc);
			return rc;
		}

		rc = pm8xxx_readb(chip->dev->parent,
				REG_USB_OVP_TRIM_ORIG_LSB, &lsb);
		if (rc) {
			pr_err("error = %d reading sbi config reg\n", rc);
			return rc;
		}

		msb = msb >> 5;
		lsb = lsb >> 5;
		usb_trim_reg_orig = msb << 3 | lsb;

		if (pm8xxx_get_version(chip->dev->parent)
				== PM8XXX_VERSION_8917) {
			rc = pm8xxx_readb(chip->dev->parent,
					REG_USB_OVP_TRIM_PM8917, &msb);
			if (rc) {
				pr_err("error = %d reading config reg\n", rc);
				return rc;
			}

			msb = msb & REG_USB_OVP_TRIM_PM8917_BIT;
			usb_trim_reg_orig |= msb << 6;
		}
	}

	/* use the original trim value */
	trim = usb_trim_reg_orig;

	trim += chip->usb_trim_table[index];
	if (trim < 0)
		trim = 0;

	pr_debug("trim_orig %d write 0x%x index=%d value 0x%x to USB_OVP_TRIM\n",
		usb_trim_reg_orig, trim, index, chip->usb_trim_table[index]);

	rc = pm8xxx_readb(chip->dev->parent, REG_SBI_CONFIG, &sbi_config);
	if (rc) {
		pr_err("error = %d reading sbi config reg\n", rc);
		return rc;
	}

	temp = sbi_config | PAGE3_ENABLE_MASK;
	rc = pm_chg_write(chip, REG_SBI_CONFIG, temp);
	if (rc) {
		pr_err("error = %d writing sbi config reg\n", rc);
		return rc;
	}

	mask = USB_OVP_TRIM_MASK;

	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917)
		mask = USB_OVP_TRIM_PM8917_MASK;

	rc = pm_chg_masked_write(chip, USB_OVP_TRIM, mask, trim);
	if (rc) {
		pr_err("error = %d writing USB_OVP_TRIM\n", rc);
		return rc;
	}

	rc = pm_chg_write(chip, REG_SBI_CONFIG, sbi_config);
	if (rc) {
		pr_err("error = %d writing sbi config reg\n", rc);
		return rc;
	}
	return rc;
}

#define PM8921_CHG_IUSB_MASK 0x1C
#define PM8921_CHG_IUSB_SHIFT 2
#define PM8921_CHG_IUSB_MAX  7
#define PM8921_CHG_IUSB_MIN  0
#define PM8917_IUSB_FINE_RES BIT(0)
static int pm_chg_iusbmax_set(struct pm8921_chg_chip *chip, int index)
{
	u8 temp, fineres, reg_val;
	int rc;

	reg_val = usb_ma_table[index].value >> 1;
	fineres = PM8917_IUSB_FINE_RES & usb_ma_table[index].value;

	if (reg_val < PM8921_CHG_IUSB_MIN || reg_val > PM8921_CHG_IUSB_MAX) {
		pr_err("bad mA=%d asked to set\n", reg_val);
		return -EINVAL;
	}
	
	/* OPPO 2012-11-02 chendx Add begin for Set the max iusb to 1100mA */
	#ifdef CONFIG_VENDOR_EDIT
	if(reg_val > PM8921_IUSB_MAX)
		reg_val = PM8921_IUSB_MAX;
	#endif
	/* OPPO 2012-11-02 chendx Modify end */
	temp = reg_val << PM8921_CHG_IUSB_SHIFT;

	/* IUSB_FINE_RES */
	if (chip->iusb_fine_res) {
		/* Clear IUSB_FINE_RES bit to avoid overshoot */
		rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
			PM8917_IUSB_FINE_RES, 0);

		rc |= pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);

		if (rc) {
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
			return rc;
		}

		if (fineres) {
			rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
				PM8917_IUSB_FINE_RES, fineres);
			if (rc) {
				pr_err("Failed to write ISUB_FINE_RES rc=%d\n",
					rc);
				return rc;
			}
		}
	} else {
		rc = pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);
		if (rc) {
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
			return rc;
		}
	}

	rc = pm_chg_usb_trim(chip, index);
	if (rc)
			pr_err("unable to set usb trim rc = %d\n", rc);

	return rc;
}

static int pm_chg_iusbmax_get(struct pm8921_chg_chip *chip, int *mA)
{
	u8 temp, fineres;
	int rc, i;

	fineres = 0;
	*mA = 0;
	rc = pm8xxx_readb(chip->dev->parent, PBL_ACCESS2, &temp);
	if (rc) {
		pr_err("err=%d reading PBL_ACCESS2\n", rc);
		return rc;
	}

	if (chip->iusb_fine_res) {
		rc = pm8xxx_readb(chip->dev->parent, IUSB_FINE_RES, &fineres);
		if (rc) {
			pr_err("err=%d reading IUSB_FINE_RES\n", rc);
			return rc;
		}
	}
	temp &= PM8921_CHG_IUSB_MASK;
	temp = temp >> PM8921_CHG_IUSB_SHIFT;

	temp = (temp << 1) | (fineres & PM8917_IUSB_FINE_RES);
	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (usb_ma_table[i].value == temp)
			break;
	}

	if (i < 0) {
		pr_err("can't find %d in usb_ma_table. Use min.\n", temp);
		i = 0;
	}

	*mA = usb_ma_table[i].usb_ma;

	return rc;
}

#define PM8921_CHG_WD_MASK 0x1F
static int pm_chg_disable_wd(struct pm8921_chg_chip *chip)
{
	/* writing 0 to the wd timer disables it */
	return pm_chg_masked_write(chip, CHG_TWDOG, PM8921_CHG_WD_MASK, 0);
}

#define PM8921_CHG_TCHG_MASK	0x7F
#define PM8921_CHG_TCHG_MIN	4
#define PM8921_CHG_TCHG_MAX	512
#define PM8921_CHG_TCHG_STEP	4
static int pm_chg_tchg_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	if (minutes < PM8921_CHG_TCHG_MIN || minutes > PM8921_CHG_TCHG_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = (minutes - 1)/PM8921_CHG_TCHG_STEP;
	return pm_chg_masked_write(chip, CHG_TCHG_MAX, PM8921_CHG_TCHG_MASK,
					 temp);
}

#define PM8921_CHG_TTRKL_MASK	0x3F
#define PM8921_CHG_TTRKL_MIN	1
#define PM8921_CHG_TTRKL_MAX	64
static int pm_chg_ttrkl_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	if (minutes < PM8921_CHG_TTRKL_MIN || minutes > PM8921_CHG_TTRKL_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = minutes - 1;
	return pm_chg_masked_write(chip, CHG_TTRKL_MAX, PM8921_CHG_TTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VTRKL_MIN_MV		2050
#define PM8921_CHG_VTRKL_MAX_MV		2800
#define PM8921_CHG_VTRKL_STEP_MV	50
#define PM8921_CHG_VTRKL_SHIFT		4
#define PM8921_CHG_VTRKL_MASK		0xF0
static int pm_chg_vtrkl_low_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	if (millivolts < PM8921_CHG_VTRKL_MIN_MV
			|| millivolts > PM8921_CHG_VTRKL_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VTRKL_MIN_MV)/PM8921_CHG_VTRKL_STEP_MV;
	temp = temp << PM8921_CHG_VTRKL_SHIFT;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VWEAK_MIN_MV		2100
#define PM8921_CHG_VWEAK_MAX_MV		3600
#define PM8921_CHG_VWEAK_STEP_MV	100
#define PM8921_CHG_VWEAK_MASK		0x0F
static int pm_chg_vweak_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	if (millivolts < PM8921_CHG_VWEAK_MIN_MV
			|| millivolts > PM8921_CHG_VWEAK_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VWEAK_MIN_MV)/PM8921_CHG_VWEAK_STEP_MV;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VWEAK_MASK,
					 temp);
}

#define PM8921_CHG_ITRKL_MIN_MA		50
#define PM8921_CHG_ITRKL_MAX_MA		200
#define PM8921_CHG_ITRKL_MASK		0x0F
#define PM8921_CHG_ITRKL_STEP_MA	10
static int pm_chg_itrkl_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	if (milliamps < PM8921_CHG_ITRKL_MIN_MA
		|| milliamps > PM8921_CHG_ITRKL_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	temp = (milliamps - PM8921_CHG_ITRKL_MIN_MA)/PM8921_CHG_ITRKL_STEP_MA;

	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_ITRKL_MASK,
					 temp);
}

#define PM8921_CHG_IWEAK_MIN_MA		325
#define PM8921_CHG_IWEAK_MAX_MA		525
#define PM8921_CHG_IWEAK_SHIFT		7
#define PM8921_CHG_IWEAK_MASK		0x80
static int pm_chg_iweak_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	if (milliamps < PM8921_CHG_IWEAK_MIN_MA
		|| milliamps > PM8921_CHG_IWEAK_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	if (milliamps < PM8921_CHG_IWEAK_MAX_MA)
		temp = 0;
	else
		temp = 1;

	temp = temp << PM8921_CHG_IWEAK_SHIFT;
	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_IWEAK_MASK,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_COLD	BIT(1)
#define PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT	1
static int pm_chg_batt_cold_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_cold_thr cold_thr)
{
	u8 temp;

	temp = cold_thr << PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_COLD;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_COLD,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_HOT		BIT(0)
#define PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT	0
static int pm_chg_batt_hot_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_hot_thr hot_thr)
{
	u8 temp;

	temp = hot_thr << PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_HOT;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_HOT,
					 temp);
}

#define PM8921_CHG_LED_SRC_CONFIG_SHIFT	4
#define PM8921_CHG_LED_SRC_CONFIG_MASK	0x30
static int pm_chg_led_src_config(struct pm8921_chg_chip *chip,
				enum pm8921_chg_led_src_config led_src_config)
{
	u8 temp;

	if (led_src_config < LED_SRC_GND ||
			led_src_config > LED_SRC_BYPASS)
		return -EINVAL;

	if (led_src_config == LED_SRC_BYPASS)
		return 0;

	temp = led_src_config << PM8921_CHG_LED_SRC_CONFIG_SHIFT;

	return pm_chg_masked_write(chip, CHG_CNTRL_3,
					PM8921_CHG_LED_SRC_CONFIG_MASK, temp);
}


static int64_t read_battery_id(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->batt_id_channel, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_id phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	return result.physical;
}

static int is_battery_valid(struct pm8921_chg_chip *chip)
{
	int64_t rc;

	if (chip->batt_id_min == 0 && chip->batt_id_max == 0)
		return 1;

	rc = read_battery_id(chip);
	if (rc < 0) {
		pr_err("error reading batt id channel = %d, rc = %lld\n",
					chip->vbat_channel, rc);
		/* assume battery id is valid when adc error happens */
		return 1;
	}

	if (rc < chip->batt_id_min || rc > chip->batt_id_max) {
		pr_err("batt_id phy =%lld is not valid\n", rc);
		return 0;
	}
	return 1;
}

static void check_battery_valid(struct pm8921_chg_chip *chip)
{
	if (is_battery_valid(chip) == 0) {
		pr_err("batt_id not valid, disbling charging\n");
		pm_chg_auto_enable(chip, 0);
	} else {
		pm_chg_auto_enable(chip, !charging_disabled);
	}
}

static void battery_id_valid(struct work_struct *work)
{
	struct pm8921_chg_chip *chip = container_of(work,
				struct pm8921_chg_chip, battery_id_valid_work);

	check_battery_valid(chip);
}

static void pm8921_chg_enable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (!__test_and_set_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		enable_irq(chip->pmic_chg_irq[interrupt]);
	}
}

static void pm8921_chg_disable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (__test_and_clear_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		disable_irq_nosync(chip->pmic_chg_irq[interrupt]);
	}
}

static int pm8921_chg_is_enabled(struct pm8921_chg_chip *chip, int interrupt)
{
	return test_bit(interrupt, chip->enabled_irqs);
}

static bool is_ext_charging(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval > POWER_SUPPLY_CHARGE_TYPE_NONE)
		return ret.intval;

	return false;
}

static bool is_ext_trickle_charging(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval == POWER_SUPPLY_CHARGE_TYPE_TRICKLE)
		return true;

	return false;
}

static int is_battery_charging(int fsm_state)
{
	if (is_ext_charging(the_chip))
		return 1;

	switch (fsm_state) {
	case FSM_STATE_ATC_2A:
	case FSM_STATE_ATC_2B:
	case FSM_STATE_ON_CHG_AND_BAT_6:
	case FSM_STATE_FAST_CHG_7:
	case FSM_STATE_TRKL_CHG_8:
		return 1;
	}
	return 0;
}

static void bms_notify(struct work_struct *work)
{
	struct bms_notify *n = container_of(work, struct bms_notify, work);

	if (n->is_charging) {
		pm8921_bms_charging_began();
	} else {
		pm8921_bms_charging_end(n->is_battery_full);
		n->is_battery_full = 0;
	}
}

static void bms_notify_check(struct pm8921_chg_chip *chip)
{
	int fsm_state, new_is_charging;

	/* OPPO 2013-02-28 chendx Add begin for do nothing with bms 
   	 XXX ,notify bms through function bms_notify_is_charging_check */
	return;
	/* OPPO 2013-02-28 chendx Add end */

	fsm_state = pm_chg_get_fsm_state(chip);
	new_is_charging = is_battery_charging(fsm_state);
	
	if (chip->bms_notify.is_charging ^ new_is_charging) {		
		/* OPPO 2013-02-23 chendx Add begin for debug log */
		pr_info("new_is_charging =%d,chip->bms_notify.is_charging=%d\n",
					new_is_charging,chip->bms_notify.is_charging);
		/* OPPO 2013-02-23 chendx Add end */
		chip->bms_notify.is_charging = new_is_charging;
		schedule_work(&(chip->bms_notify.work));
	}
}

/* OPPO 2013-02-28 chendx Add begin for oppo begin and end bms charge */
static void bms_notify_is_charging_check(bool new_is_chaging,struct pm8921_chg_chip *chip)
{
	pr_info("new_is_charging =%d,chip->bms_notify.is_charging=%d\n",
			new_is_chaging,chip->bms_notify.is_charging);

	if(new_is_chaging)
	    chip->bms_notify.is_charging = 1;
	else
		chip->bms_notify.is_charging = 0;
	
	schedule_work(&(chip->bms_notify.work));

}
/* OPPO 2013-02-28 chendx Add end */


static enum power_supply_property pm_power_props_usb[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property pm_power_props_mains[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static char *pm_power_supplied_to[] = {
	"battery",
};

#define USB_WALL_THRESHOLD_MA	500
static int pm_power_get_property_mains(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	/* OPPO 2012-12-04 chendx Modify begin for reason */
	#ifndef CONFIG_VENDOR_EDIT
	int type;
	#endif
	/* OPPO 2012-12-04 chendx Modify end */


	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
/* OPPO 2012-12-04 chendx Modify begin for reason */
#ifndef CONFIG_VENDOR_EDIT

		val->intval = 0;

		if (the_chip->has_dc_supply) {
			val->intval = 1;
			return 0;
		}

		if (the_chip->dc_present) {
			val->intval = 1;
			return 0;
		}

		type = the_chip->usb_type;
		if (type == POWER_SUPPLY_TYPE_USB_DCP ||
			type == POWER_SUPPLY_TYPE_USB_ACA ||
			type == POWER_SUPPLY_TYPE_USB_CDP)
			val->intval = is_usb_chg_plugged_in(the_chip);

#else
	   if(the_chip->pm8921_chg_type == USB_DCP_CHARGER ||
	   	  the_chip->pm8921_chg_type == USB_NON_DCP_CHARGER)
	   	    val->intval = 1;
	   else
	   	    val->intval = 0;
#endif
/* OPPO 2012-12-04 chendx Modify end */
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int disable_aicl(int disable)
{
	if (disable != POWER_SUPPLY_HEALTH_UNKNOWN
		&& disable != POWER_SUPPLY_HEALTH_GOOD) {
		pr_err("called with invalid param :%d\n", disable);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}

	pr_debug("Disable AICL = %d\n", disable);
	the_chip->disable_aicl = disable;
	return 0;
}

static int switch_usb_to_charge_mode(struct pm8921_chg_chip *chip)
{
	int rc;

	if (!chip->host_mode)
		return 0;

	/* enable usbin valid comparator and remove force usb ovp fet off */
	rc = pm_chg_write(chip, USB_OVP_TEST, 0xB2);
	if (rc < 0) {
		pr_err("Failed to write 0xB2 to USB_OVP_TEST rc = %d\n", rc);
		return rc;
	}

	chip->host_mode = 0;

	return 0;
}

static int switch_usb_to_host_mode(struct pm8921_chg_chip *chip)
{
	int rc;

	if (chip->host_mode)
		return 0;

	/* disable usbin valid comparator and force usb ovp fet off */
	rc = pm_chg_write(chip, USB_OVP_TEST, 0xB3);
	if (rc < 0) {
		pr_err("Failed to write 0xB3 to USB_OVP_TEST rc = %d\n", rc);
		return rc;
	}

	chip->host_mode = 1;

	return 0;
}

static int pm_power_set_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_SCOPE:
		if (val->intval == POWER_SUPPLY_SCOPE_SYSTEM)
			return switch_usb_to_host_mode(the_chip);
		if (val->intval == POWER_SUPPLY_SCOPE_DEVICE)
			return switch_usb_to_charge_mode(the_chip);
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		return pm8921_set_usb_power_supply_type(val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		/* UNKNOWN(0) means enable aicl, GOOD(1) means disable aicl */
		return disable_aicl(val->intval);
	default:
		return -EINVAL;
	}
	return 0;
}

static int usb_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
		return 1;
	default:
		break;
	}

	return 0;
}

static int pm_power_get_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int current_max;

	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (pm_is_chg_charge_dis(the_chip)) {
			val->intval = 0;
		} else {
			pm_chg_iusbmax_get(the_chip, &current_max);
			val->intval = current_max;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
/* OPPO 2012-12-04 chendx Modify begin for reason */
#ifndef CONFIG_VENDOR_EDIT
		val->intval = 0;

		if (the_chip->usb_type == POWER_SUPPLY_TYPE_USB)
			val->intval = is_usb_chg_plugged_in(the_chip);

#else
		if(the_chip->pm8921_chg_type == USB_SDP_CHARGER)
			val->intval = 1;
		else
			val->intval = 0;
#endif
/* OPPO 2012-12-04 chendx Modify end */
		break;

	case POWER_SUPPLY_PROP_SCOPE:
		if (the_chip->host_mode)
			val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		else
			val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		/* UNKNOWN(0) means enable aicl, GOOD(1) means disable aicl */
		val->intval = the_chip->disable_aicl;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	/* OPPO 2012-08-06 chendx Add begin for charge current */
	#ifdef CONFIG_VENDOR_EDIT
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_TIMEOUT,
	POWER_SUPPLY_PROP_CHARGE_SOC_FALL,
	#endif
	/* OPPO 2012-08-06 chendx Add end */
};

static int get_prop_battery_uvolts(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

/* OPPO 2013-04-08 chendx Add begin for auto test */
	#ifdef AUTO_TEST_FEATURE
	if(chg_auto_test_feature && chip->auto_test_batvol_high){
		chip->battery_voltage = BATTERY_SOFT_OVP_VOLTAGE + 100;
		return (BATTERY_SOFT_OVP_VOLTAGE + 100) * 1000;
	}
	#endif
/* OPPO 2013-04-08 chendx Add end */

	rc = pm8xxx_adc_read(chip->vbat_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
    /* OPPO 2012-08-08 chendx Add begin for reason */
    #ifdef CONFIG_VENDOR_EDIT
	chip->battery_voltage = (int)result.physical/1000;
    #endif
    /* OPPO 2012-08-08 chendx Add end */
	pr_debug("mvolts phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	return (int)result.physical;
}

static int voltage_based_capacity(struct pm8921_chg_chip *chip)
{
/* OPPO 2012-10-26 chendx Modify begin for  battery is remove,Default Soc is 30% */
#ifndef CONFIG_VENDOR_EDIT

	unsigned int current_voltage_uv = get_prop_battery_uvolts(chip);
	unsigned int current_voltage_mv = current_voltage_uv / 1000;
	unsigned int low_voltage = chip->min_voltage_mv;
	unsigned int high_voltage = chip->max_voltage_mv;

	if (current_voltage_uv < 0) {
		pr_err("Error reading current voltage\n");
		return -EIO;
	}

	if (current_voltage_mv <= low_voltage)
		return 0;
	else if (current_voltage_mv >= high_voltage)
		return 100;
	else
		return (current_voltage_mv - low_voltage) * 100
		    / (high_voltage - low_voltage);
#else
	return DEFAULT_BATT_CAPACITY;
#endif
/* OPPO 2012-10-26 chendx Modify end */
}

static int get_prop_batt_present(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
}

static int get_prop_batt_status(struct pm8921_chg_chip *chip)
{
	int batt_state = POWER_SUPPLY_STATUS_DISCHARGING;
	int fsm_state = pm_chg_get_fsm_state(chip);
	int i;

	/* OPPO 2012-11-28 chendx Add begin for not charge when not capable charge now */
	#ifdef CONFIG_VENDOR_EDIT
	
		/* OPPO 2013-04-08 chendx Add begin for auto test */
		#ifdef AUTO_TEST_FEATURE
		if(chg_auto_test_feature && (chip->auto_test_cool_charge_finished ||
			chip->auto_test_warm_charge_finished) )
			return POWER_SUPPLY_STATUS_FULL;
		#endif
		/* OPPO 2013-04-08 chendx Add end */ 
	
		if(chip->charge_is_finished)
			return POWER_SUPPLY_STATUS_FULL;
	
		if(!is_capable_of_charging(chip))
		  return POWER_SUPPLY_STATUS_NOT_CHARGING;
	
		if(is_charger_connect(chip))
			/*charger connect*/
			return POWER_SUPPLY_STATUS_CHARGING;
		else
			return POWER_SUPPLY_STATUS_DISCHARGING; 	
	#endif /*CONFIG_VENDOR_EDIT*/
	/* OPPO 2012-11-28 chendx Add end */

	if (chip->ext_psy) {
		if (chip->ext_charge_done)
			return POWER_SUPPLY_STATUS_FULL;
		if (chip->ext_charging)
			return POWER_SUPPLY_STATUS_CHARGING;
	}

	for (i = 0; i < ARRAY_SIZE(map); i++)
		if (map[i].fsm_state == fsm_state)
			batt_state = map[i].batt_state;

	if (fsm_state == FSM_STATE_ON_CHG_HIGHI_1) {
		if (!pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			|| !pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			|| pm_chg_get_rt_status(chip, CHGHOT_IRQ)
			|| pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ))

			batt_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	return batt_state;
}

static int get_prop_batt_capacity(struct pm8921_chg_chip *chip)
{
	int percent_soc;
	/* OPPO 2012-10-26 chendx add begin for  batt capacity algorithm */
#ifdef CONFIG_VENDOR_EDIT
	static int reported_soc_init=0;
#endif
/* OPPO 2012-11-26 chendx Add end */

	if (chip->battery_less_hardware)
		return 100;

	if (!get_prop_batt_present(chip))
		percent_soc = voltage_based_capacity(chip);
	else
		percent_soc = pm8921_bms_get_percent_charge();

	if (percent_soc == -ENXIO)
		percent_soc = voltage_based_capacity(chip);

/* OPPO 2012-10-26 chendx Modify begin for  batt capacity algorithm */
#ifndef CONFIG_VENDOR_EDIT
	if (percent_soc <= 10)
		pr_warn_ratelimited("low battery charge = %d%%\n",
						percent_soc);

	if (percent_soc <= chip->resume_charge_percent
		&& get_prop_batt_status(chip) == POWER_SUPPLY_STATUS_FULL) {
		pr_debug("soc fell below %d. charging enabled.\n",
						chip->resume_charge_percent);
		if (chip->is_bat_warm)
			pr_warn_ratelimited("battery is warm = %d, do not resume charging at %d%%.\n",
					chip->is_bat_warm,
					chip->resume_charge_percent);
		else if (chip->is_bat_cool)
			pr_warn_ratelimited("battery is cool = %d, do not resume charging at %d%%.\n",
					chip->is_bat_cool,
					chip->resume_charge_percent);
		else
			pm_chg_vbatdet_set(the_chip, PM8921_CHG_VBATDET_MAX);
	}

fail_voltage:
	chip->recent_reported_soc = percent_soc;
	return percent_soc;
#else
	if(chip->battery_temp <= BATT_REMOVE_TEMP_C &&
		pm_chg_get_rt_status(chip, BATT_REMOVED_IRQ) &&
		percent_soc <= 0){
		/*report battery capacity 30 when battery is not connect*/
		pr_err("Batt is not connect!!\n");
		return DEFAULT_BATT_CAPACITY;
	}

	/* Add begin for RF WIFI AT factory TEST */
	if(chip->ftm_test_mode == true){
		percent_soc = DEFAULT_BATT_CAPACITY;
		chip->recent_reported_soc = percent_soc;
		return percent_soc;
	}

	/*first report soc init
	XXXX init chip->recent_reported_soc*/
	if(!reported_soc_init){
		percent_soc = read_calib_soc();
		if(percent_soc<0){
			    pr_err("Read calib soc failed!\n");
				return 30;
		}
			
		pr_err("Warning,recent_reported_soc need init first=%d\n",percent_soc);
		chip->recent_reported_soc = percent_soc;
		chip->batt_capacity_pre = percent_soc;
		chip->report_calib_soc = percent_soc;
		reported_soc_init = 1;
	}
	
	chip->recent_reported_soc = percent_soc;

	return chip->report_calib_soc;

#endif
/* OPPO 2012-11-26 chendx Modify end */
}

static int get_prop_batt_current_max(struct pm8921_chg_chip *chip, int *curr)
{
	*curr = 0;
	*curr = pm8921_bms_get_current_max();
	if (*curr == -EINVAL)
		return -EINVAL;

	return 0;
}

static int get_prop_batt_current(struct pm8921_chg_chip *chip, int *curr)
{
	int rc;

	*curr = 0;
	rc = pm8921_bms_get_battery_current(curr);
	if (rc == -ENXIO) {
		rc = pm8xxx_ccadc_get_battery_current(curr);
	}
	if (rc) {
		pr_err("unable to get batt current rc = %d\n", rc);

	/* OPPO 2012-08-09 chendx Modify begin for error handle */
	#ifndef CONFIG_VENDOR_EDIT
		return rc;
	#else
		return chip->charge_current;
	#endif
	/* OPPO 2012-08-09 chendx Modify end */
	} else {
	/* OPPO 2012-08-07 chendx Modify begin for battery current(uA) */
	#ifdef CONFIG_VENDOR_EDIT
	    chip->charge_current = *curr/1000;
		
		return rc;
	#else
		return rc;
	#endif
	/* OPPO 2012-08-07 chendx Modify end */
	}
}

static int get_prop_batt_fcc(struct pm8921_chg_chip *chip)
{
	int rc;

	rc = pm8921_bms_get_fcc();
	if (rc < 0)
		pr_err("unable to get batt fcc rc = %d\n", rc);
	return rc;
}
#ifndef CONFIG_VENDOR_EDIT
static int get_prop_batt_charge_now(struct pm8921_chg_chip *chip, int *cc_uah)
{
	int rc;

	*cc_uah = 0;
	rc = pm8921_bms_cc_uah(cc_uah);
	if (rc)
		pr_err("unable to get batt fcc rc = %d\n", rc);

	return rc;
}
#endif

static int get_prop_batt_health(struct pm8921_chg_chip *chip)
{
	/* OPPO 2012-08-09 chendx Modify begin for reason */
	#ifndef CONFIG_VENDOR_EDIT
	int temp;

	temp = pm_chg_get_rt_status(chip, BATTTEMP_HOT_IRQ);
	if (temp)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	temp = pm_chg_get_rt_status(chip, BATTTEMP_COLD_IRQ);
	if (temp)
		return POWER_SUPPLY_HEALTH_COLD;

	return POWER_SUPPLY_HEALTH_GOOD;
	#else
	
/* OPPO 2013-04-08 chendx Add begin for auto test */
	#ifdef AUTO_TEST_FEATURE
	if(chg_auto_test_feature && chip->auto_test_bat_disconnect)
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	#endif
/* OPPO 2013-04-08 chendx Add end */
	
	print_pm8921(DEBUG_TRACE, "%s batt_health = %d\n", __func__,chip->batt_health);
	return chip->batt_health;
	#endif
	/* OPPO 2012-08-09 chendx Modify end */
}

static int get_prop_charge_type(struct pm8921_chg_chip *chip)
{
	int temp;

	if (!get_prop_batt_present(chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	if (is_ext_trickle_charging(chip))
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	if (is_ext_charging(chip))
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	temp = pm_chg_get_rt_status(chip, TRKLCHG_IRQ);
	if (temp)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	temp = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (temp)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

#define MAX_TOLERABLE_BATT_TEMP_DDC	680
#define MAX_BATT_TEMP_HIGH_COUNTER	15
static int get_prop_batt_temp(struct pm8921_chg_chip *chip, int *temp)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	if (chip->battery_less_hardware) {
		*temp = 300;
		return 0;
	}

	rc = pm8xxx_adc_read(chip->batt_temp_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
/* OPPO 2012-10-19 chendx Modify begin for battery temperature */
#ifndef CONFIG_VENDOR_EDIT

	pr_debug("batt_temp phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	if (result.physical > MAX_TOLERABLE_BATT_TEMP_DDC)
		pr_err("BATT_TEMP= %d > 68degC, device will be shutdown\n",
							(int) result.physical);

	*temp = (int)result.physical;

	return rc;
#else
    #ifdef TEMPERATURE_CHARGE_TEST_FEATURE
    /*Use to  charge temperature test*/
    if(chip->chg_temp_test_mode == true)
    {
       chip->battery_temp = chip->chg_test_temp;
	   *temp = chip->chg_test_temp *10;
       return rc;
    }
	#endif
	
	pr_debug("batt_temp phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	
    if (result.physical > MAX_TOLERABLE_BATT_TEMP_DDC){
		chip->batt_temp_high_counter ++;
		if(chip->batt_temp_high_counter >= MAX_BATT_TEMP_HIGH_COUNTER){
			pr_err("BATT_TEMP= %ddegC > 68degC, device will be shutdown,counter=%d\n",
							(int) result.physical/10,chip->batt_temp_high_counter);
			chip->batt_temp_high_counter = 0;
		}else{
		    chip->battery_temp = MAX_TOLERABLE_BATT_TEMP_DDC/10;
			  /*report 68C when continute to read temp high than MAX_TOLERABLE_BATT_TEMP_DDC 
			   and shutdown system quikly*/
			*temp = MAX_TOLERABLE_BATT_TEMP_DDC;
			return 0;
		}
    }else{
        if(chip->batt_temp_high_counter)
    		chip->batt_temp_high_counter = 0;
    }
	
	chip->battery_temp = (int)result.physical/10;
	*temp  = (int)result.physical;
    return rc;
#endif
/* OPPO 2012-10-19 chendx Modify end */
}

static int pm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	int rc = 0;
	int value;
	struct pm8921_chg_chip *chip = container_of(psy, struct pm8921_chg_chip,
								batt_psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		rc = get_prop_batt_present(chip);
		if (rc >= 0) {
			val->intval = rc;
			rc = 0;
		}
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->max_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->min_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = get_prop_battery_uvolts(chip);
		if (rc >= 0) {
			val->intval = rc;
			rc = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = get_prop_batt_capacity(chip);
		if (rc >= 0) {
			val->intval = rc;
			rc = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = get_prop_batt_current(chip, &value);
		if (!rc)
			/* OPPO 2012-10-26 chendx Modify begin for current (mA) */
#ifndef CONFIG_VENDOR_EDIT  //liuhd add 

			val->intval = value;
#else
  			val->intval = value/1000;
#endif			
/* OPPO 2012-10-26 chendx Modify end */
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = get_prop_batt_current_max(chip, &value);
		if (!rc)
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		rc = get_prop_batt_temp(chip, &value);
		if (!rc)
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		rc = get_prop_batt_fcc(chip);
		if (rc >= 0) {
			val->intval = rc;
			rc = 0;
		}
		break;
#ifndef CONFIG_VENDOR_EDIT
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		rc = get_prop_batt_charge_now(chip, &value);
		if (!rc) {
			val->intval = value;
			rc = 0;
		}
		break;
#endif
	/* OPPO 2012-08-06 chendx Add begin for get charger voltage */
	#ifdef CONFIG_VENDOR_EDIT
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = get_prop_chg_voltage(chip);
		break;
    case POWER_SUPPLY_PROP_CHARGE_TIMEOUT:
		/*charge too long error full,stantard 6 hours and USB and nonstantard 10 hours*/
		val->intval = get_prop_teoc_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_SOC_FALL:
		/*continue to check soc fall within 5minutes*/
		val->intval = get_prop_soc_fall_status(chip);
	break;
	
	#endif
	/* OPPO 2012-08-06 chendx Add end */
	default:
		rc = -EINVAL;
	}

	return rc;
}

static void (*notify_vbus_state_func_ptr)(int);
static int usb_chg_current;

int pm8921_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = callback;
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_charger_register_vbus_sn);

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void pm8921_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = NULL;
}
EXPORT_SYMBOL_GPL(pm8921_charger_unregister_vbus_sn);

static void notify_usb_of_the_plugin_event(int plugin)
{
	plugin = !!plugin;
	if (notify_vbus_state_func_ptr) {
		pr_debug("notifying plugin\n");
		(*notify_vbus_state_func_ptr) (plugin);
	} else {
		pr_debug("unable to notify plugin\n");
	}
}

static void __pm8921_charger_vbus_draw(unsigned int mA)
{
	int i, rc;
	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	if (usb_max_current && mA > usb_max_current) {
		pr_debug("restricting usb current to %d instead of %d\n",
					usb_max_current, mA);
		mA = usb_max_current;
	}

	if (mA <= 2) {
		usb_chg_current = 0;
		rc = pm_chg_iusbmax_set(the_chip, 0);
		if (rc) {
			pr_err("unable to set iusb to %d rc = %d\n", 0, rc);
		}
		rc = pm_chg_usb_suspend_enable(the_chip, 1);
		if (rc)
			pr_err("fail to set suspend bit rc=%d\n", rc);
	} else {
		rc = pm_chg_usb_suspend_enable(the_chip, 0);
		if (rc)
			pr_err("fail to reset suspend bit rc=%d\n", rc);
		for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
			if (usb_ma_table[i].usb_ma <= mA)
				break;
		}

		if (i < 0) {
			pr_err("can't find %dmA in usb_ma_table. Use min.\n",
			       mA);
			i = 0;
		}

		/* Check if IUSB_FINE_RES is available */
		while ((usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
				&& !the_chip->iusb_fine_res)
			i--;
		if (i < 0)
			i = 0;
		rc = pm_chg_iusbmax_set(the_chip, i);
		if (rc)
			pr_err("unable to set iusb to %d rc = %d\n", i, rc);
	}
}

/* USB calls these to tell us how much max usb current the system can draw */
void pm8921_charger_vbus_draw(unsigned int mA)
{
	int set_usb_now_ma;

	pr_debug("Enter charge=%d\n", mA);

	/*
	 * Reject VBUS requests if USB connection is the only available
	 * power source. This makes sure that if booting without
	 * battery the iusb_max value is not decreased avoiding potential
	 * brown_outs.
	 *
	 * This would also apply when the battery has been
	 * removed from the running system.
	 */
	if (mA == 0 && the_chip && !get_prop_batt_present(the_chip)
		&& !is_dc_chg_plugged_in(the_chip)) {
		if (!the_chip->has_dc_supply) {
			pr_err("rejected: no other power source mA = %d\n", mA);
			return;
		}
	}

	if (usb_max_current && mA > usb_max_current) {
		pr_warn("restricting usb current to %d instead of %d\n",
					usb_max_current, mA);
		mA = usb_max_current;
	}
	if (usb_target_ma == 0 && mA > USB_WALL_THRESHOLD_MA)
		usb_target_ma = mA;

	if (usb_target_ma)
		usb_target_ma = mA;


	if (mA > USB_WALL_THRESHOLD_MA)
		set_usb_now_ma = USB_WALL_THRESHOLD_MA;
	else
		set_usb_now_ma = mA;

	if (the_chip && the_chip->disable_aicl)
		set_usb_now_ma = mA;

	if (the_chip)
		__pm8921_charger_vbus_draw(set_usb_now_ma);
	else
		/*
		 * called before pmic initialized,
		 * save this value and use it at probe
		 */
		usb_chg_current = set_usb_now_ma;
}
EXPORT_SYMBOL_GPL(pm8921_charger_vbus_draw);

//add by chendx
int pm8921_charger_enable(bool enable)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

/* OPPO 2013-02-28 chendx Add begin for notify with bms */
	bms_notify_is_charging_check(enable,the_chip);
/* OPPO 2013-02-28 chendx Add end */
	
	enable = !!enable;
	rc = pm_chg_auto_enable(the_chip, enable);
	if (rc)
		pr_err("Failed rc=%d\n", rc);
	return rc;
}
EXPORT_SYMBOL(pm8921_charger_enable);
//end by chendx

int pm8921_is_usb_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_usb_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_usb_chg_plugged_in);

int pm8921_is_dc_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_dc_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_dc_chg_plugged_in);

int pm8921_is_battery_present(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return get_prop_batt_present(the_chip);
}
EXPORT_SYMBOL(pm8921_is_battery_present);

int pm8921_is_batfet_closed(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_batfet_closed(the_chip);
}
EXPORT_SYMBOL(pm8921_is_batfet_closed);
/*
 * Disabling the charge current limit causes current
 * current limits to have no monitoring. An adequate charger
 * capable of supplying high current while sustaining VIN_MIN
 * is required if the limiting is disabled.
 */
int pm8921_disable_input_current_limit(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable) {
		pr_warn("Disabling input current limit!\n");

		return pm_chg_write(the_chip, CHG_BUCK_CTRL_TEST3, 0xF2);
	}
	return 0;
}
EXPORT_SYMBOL(pm8921_disable_input_current_limit);

int pm8917_set_under_voltage_detection_threshold(int mv)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return pm_chg_uvd_threshold_set(the_chip, mv);
}
EXPORT_SYMBOL(pm8917_set_under_voltage_detection_threshold);

int pm8921_set_max_battery_charge_current(int ma)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return pm_chg_ibatmax_set(the_chip, ma);
}
EXPORT_SYMBOL(pm8921_set_max_battery_charge_current);

int pm8921_disable_source_current(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable)
		pr_warn("current drawn from chg=0, battery provides current\n");

	pm_chg_usb_suspend_enable(the_chip, disable);

	return pm_chg_charge_dis(the_chip, disable);
}
EXPORT_SYMBOL(pm8921_disable_source_current);

int pm8921_regulate_input_voltage(int voltage)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm_chg_vinmin_set(the_chip, voltage);

	if (rc == 0)
		the_chip->vin_min = voltage;

	return rc;
}

#define USB_OV_THRESHOLD_MASK  0x60
#define USB_OV_THRESHOLD_SHIFT  5
int pm8921_usb_ovp_set_threshold(enum pm8921_usb_ov_threshold ov)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ov > PM_USB_OV_7V) {
		pr_err("limiting to over voltage threshold to 7volts\n");
		ov = PM_USB_OV_7V;
	}

	temp = USB_OV_THRESHOLD_MASK & (ov << USB_OV_THRESHOLD_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OV_THRESHOLD_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_threshold);

#define USB_DEBOUNCE_TIME_MASK	0x06
#define USB_DEBOUNCE_TIME_SHIFT 1
int pm8921_usb_ovp_set_hystersis(enum pm8921_usb_debounce_time ms)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ms > PM_USB_DEBOUNCE_80P5MS) {
		pr_err("limiting debounce to 80.5ms\n");
		ms = PM_USB_DEBOUNCE_80P5MS;
	}

	temp = USB_DEBOUNCE_TIME_MASK & (ms << USB_DEBOUNCE_TIME_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_DEBOUNCE_TIME_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_hystersis);

#define USB_OVP_DISABLE_MASK	0x80
int pm8921_usb_ovp_disable(int disable)
{
	u8 temp = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (disable)
		temp = USB_OVP_DISABLE_MASK;

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OVP_DISABLE_MASK, temp);
}

bool pm8921_is_battery_charging(int *source)
{
	int fsm_state, is_charging, dc_present, usb_present;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	fsm_state = pm_chg_get_fsm_state(the_chip);
	is_charging = is_battery_charging(fsm_state);
	if (is_charging == 0) {
		*source = PM8921_CHG_SRC_NONE;
		return is_charging;
	}

	if (source == NULL)
		return is_charging;

	/* the battery is charging, the source is requested, find it */
	dc_present = is_dc_chg_plugged_in(the_chip);
	usb_present = is_usb_chg_plugged_in(the_chip);

	if (dc_present && !usb_present)
		*source = PM8921_CHG_SRC_DC;

	if (usb_present && !dc_present)
		*source = PM8921_CHG_SRC_USB;

	if (usb_present && dc_present)
		/*
		 * The system always chooses dc for charging since it has
		 * higher priority.
		 */
		*source = PM8921_CHG_SRC_DC;

	return is_charging;
}
EXPORT_SYMBOL(pm8921_is_battery_charging);

int pm8921_set_usb_power_supply_type(enum power_supply_type type)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (type < POWER_SUPPLY_TYPE_USB && type > POWER_SUPPLY_TYPE_BATTERY)
		return -EINVAL;

#ifdef CONFIG_VENDOR_EDIT
/* OPPO 2012-12-06 chendx Modify begin for usb_psy only support POWER_SUPPLY_TYPE_USB type */
   //XXX we report support from pm8921-charger.c
	if(type != POWER_SUPPLY_TYPE_USB)
		type = POWER_SUPPLY_TYPE_USB; 
/* OPPO 2012-12-06 chendx Add end */
#endif
	the_chip->usb_type = type;
	power_supply_changed(&the_chip->usb_psy);
	power_supply_changed(&the_chip->dc_psy);
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_set_usb_power_supply_type);

int pm8921_batt_temperature(void)
{
	int temp = 0, rc = 0;
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = get_prop_batt_temp(the_chip, &temp);
	if (rc) {
		pr_err("Unable to read temperature");
		return rc;
	}
	return temp;
}

static void handle_usb_insertion_removal(struct pm8921_chg_chip *chip)
{
	int usb_present;

	pm_chg_failed_clear(chip, 1);
	usb_present = is_usb_chg_plugged_in(chip);
	if (chip->usb_present ^ usb_present) {
		notify_usb_of_the_plugin_event(usb_present);
		chip->usb_present = usb_present;
/* OPPO 2012-12-04 chendx Delete begin for not use oppo charge */
#ifndef CONFIG_VENDOR_EDIT

		power_supply_changed(&chip->usb_psy);
		power_supply_changed(&chip->batt_psy);
#endif
/* OPPO 2012-12-04 chendx Delete end */
		pm8921_bms_calibrate_hkadc();
	}
	if (usb_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_RAMP_MS));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
/* OPPO 2013-02-28 chendx Add begin for notify with boms */
		bms_notify_is_charging_check(1,chip);
/* OPPO 2013-02-28 chendx Add end */
	} else {
		/* USB unplugged reset target current */
		usb_target_ma = 0;
		pm8921_chg_disable_irq(chip, CHG_GONE_IRQ);
	/* OPPO 2012-08-09 chendx Add begin for USBIN charger remove from otg driver */
	#ifdef CONFIG_VENDOR_EDIT
		//pm8921_chg_connected(USB_INVALID_CHARGER);
	#endif
	
/* OPPO 2013-02-28 chendx Add begin for notify with bms */
		bms_notify_is_charging_check(0,chip);
/* OPPO 2013-02-28 chendx Add end */
	}
/* OPPO 2013-02-23 chendx Delete begin for usb insertion removal notify */
#if 0
	bms_notify_check(chip);
#endif
/* OPPO 2013-02-23 chendx Delete end */
}

static void handle_stop_ext_chg(struct pm8921_chg_chip *chip)
{
	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}

	if (!chip->ext_charging) {
		pr_debug("already not charging.\n");
		return;
	}

	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_NONE);
	pm8921_disable_source_current(false); /* release BATFET */
	power_supply_changed(&chip->dc_psy);
	chip->ext_charging = false;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

static void handle_start_ext_chg(struct pm8921_chg_chip *chip)
{
	int dc_present;
	int batt_present;
	int batt_temp_ok;
	unsigned long delay =
		round_jiffies_relative(msecs_to_jiffies(EOC_CHECK_PERIOD_MS));

	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}

	if (chip->ext_charging) {
		pr_debug("already charging.\n");
		return;
	}

	dc_present = is_dc_chg_plugged_in(chip);
	batt_present = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
	batt_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	if (!dc_present) {
		pr_warn("%s. dc not present.\n", __func__);
		return;
	}
	if (!batt_present) {
		pr_warn("%s. battery not present.\n", __func__);
		return;
	}
	if (!batt_temp_ok) {
		pr_warn("%s. battery temperature not ok.\n", __func__);
		return;
	}

	/* Force BATFET=ON */
	pm8921_disable_source_current(true);

	schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_RAMP_MS));

	power_supply_set_online(chip->ext_psy, dc_present);
	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_FAST);
	chip->ext_charging = true;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/*
	 * since we wont get a fastchg irq from external charger
	 * use eoc worker to detect end of charging
	 */
	schedule_delayed_work(&chip->eoc_work, delay);
	wake_lock(&chip->eoc_wake_lock);
	if (chip->btc_override)
		schedule_delayed_work(&chip->btc_override_work,
				round_jiffies_relative(msecs_to_jiffies
					(chip->btc_delay_ms)));
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

static void turn_off_ovp_fet(struct pm8921_chg_chip *chip, u16 ovptestreg)
{
	u8 temp;
	int rc;

	rc = pm_chg_write(chip, ovptestreg, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to ovptestreg rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, ovptestreg, &temp);
	if (rc) {
		pr_err("Failed to read from ovptestreg rc = %d\n", rc);
		return;
	}
	/* set ovp fet disable bit and the write bit */
	temp |= 0x81;
	rc = pm_chg_write(chip, ovptestreg, temp);
	if (rc) {
		pr_err("Failed to write 0x%x ovptestreg rc=%d\n", temp, rc);
		return;
	}
}

static void turn_on_ovp_fet(struct pm8921_chg_chip *chip, u16 ovptestreg)
{
	u8 temp;
	int rc;

	rc = pm_chg_write(chip, ovptestreg, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to OVP_TEST rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, ovptestreg, &temp);
	if (rc) {
		pr_err("Failed to read from OVP_TEST rc = %d\n", rc);
		return;
	}
	/* unset ovp fet disable bit and set the write bit */
	temp &= 0xFE;
	temp |= 0x80;
	rc = pm_chg_write(chip, ovptestreg, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to OVP_TEST rc = %d\n",
								temp, rc);
		return;
	}
}

static int param_open_ovp_counter = 10;
module_param(param_open_ovp_counter, int, 0644);

#define USB_ACTIVE_BIT BIT(5)
#define DC_ACTIVE_BIT BIT(6)
static int is_active_chg_plugged_in(struct pm8921_chg_chip *chip,
						u8 active_chg_mask)
{
	if (active_chg_mask & USB_ACTIVE_BIT)
		return pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
	else if (active_chg_mask & DC_ACTIVE_BIT)
		return pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
	else
		return 0;
}

#define WRITE_BANK_4		0xC0
#define OVP_DEBOUNCE_TIME 0x06
static void unplug_ovp_fet_open(struct pm8921_chg_chip *chip)
{
	int chg_gone = 0, active_chg_plugged_in = 0;
	int count = 0;
	u8 active_mask = 0;
	u16 ovpreg, ovptestreg;

	if (is_usb_chg_plugged_in(chip) &&
		(chip->active_path & USB_ACTIVE_BIT)) {
		ovpreg = USB_OVP_CONTROL;
		ovptestreg = USB_OVP_TEST;
		active_mask = USB_ACTIVE_BIT;
	} else if (is_dc_chg_plugged_in(chip) &&
		(chip->active_path & DC_ACTIVE_BIT)) {
		ovpreg = DC_OVP_CONTROL;
		ovptestreg = DC_OVP_TEST;
		active_mask = DC_ACTIVE_BIT;
	} else {
		return;
	}

	while (count++ < param_open_ovp_counter) {
		pm_chg_masked_write(chip, ovpreg, OVP_DEBOUNCE_TIME, 0x0);
		usleep(10);
		active_chg_plugged_in
			= is_active_chg_plugged_in(chip, active_mask);
		chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);
		pr_debug("OVP FET count = %d chg_gone=%d, active_valid = %d\n",
					count, chg_gone, active_chg_plugged_in);

		/* note usb_chg_plugged_in=0 => chg_gone=1 */
		if (chg_gone == 1 && active_chg_plugged_in == 1) {
			pr_debug("since chg_gone = 1 dis ovp_fet for 20msec\n");
			turn_off_ovp_fet(chip, ovptestreg);

			msleep(20);

			turn_on_ovp_fet(chip, ovptestreg);
		} else {
			break;
		}
	}
	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917)
		pm_chg_masked_write(chip, ovpreg, OVP_DEBOUNCE_TIME, 0x6);
	else
		pm_chg_masked_write(chip, ovpreg, OVP_DEBOUNCE_TIME, 0x2);

	pr_debug("Exit count=%d chg_gone=%d, active_valid=%d\n",
		count, chg_gone, active_chg_plugged_in);
	return;
}

static int find_usb_ma_value(int value)
{
	int i;

	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (value >= usb_ma_table[i].usb_ma)
			break;
	}

	return i;
}

static void decrease_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);
		if (i > 0)
			i--;
		while (!the_chip->iusb_fine_res && i > 0
			&& (usb_ma_table[i].value & PM8917_IUSB_FINE_RES))
			i--;

		if (i < 0) {
			pr_err("can't find %dmA in usb_ma_table. Use min.\n",
			       *value);
			i = 0;
		}

		*value = usb_ma_table[i].usb_ma;
	}
}

static void increase_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);

		if (i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;
		/* Get next correct entry if IUSB_FINE_RES is not available */
		while (!the_chip->iusb_fine_res
			&& (usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
			&& i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;

		*value = usb_ma_table[i].usb_ma;
	}
}

static void vin_collapse_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
			struct pm8921_chg_chip, vin_collapse_check_work);

	/*
	 * AICL only for wall-chargers. If the charger appears to be plugged
	 * back in now, the corresponding unplug must have been because of we
	 * were trying to draw more current than the charger can support. In
	 * such a case reset usb current to 500mA and decrease the target.
	 * The AICL algorithm will step up the current from 500mA to target
	 */
	if (is_usb_chg_plugged_in(chip)
		&& usb_target_ma > USB_WALL_THRESHOLD_MA
		&& !chip->disable_aicl) {
		/* decrease usb_target_ma */
		decrease_usb_ma_value(&usb_target_ma);
		/* reset here, increase in unplug_check_worker */
		__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
		pr_debug("usb_now=%d, usb_target = %d\n",
				USB_WALL_THRESHOLD_MA, usb_target_ma);
		if (!delayed_work_pending(&chip->unplug_check_work))
			schedule_delayed_work(&chip->unplug_check_work,
				      msecs_to_jiffies
						(UNPLUG_CHECK_WAIT_PERIOD_MS));
	} else {
		handle_usb_insertion_removal(chip);
	}
}

/*OPPO,Jiangsm add begin for bad charger type detecting logic,2013-1-23*/
#ifdef CONFIG_VENDOR_EDIT
bool is_usb_dc_plugged_in(void)
{
	struct pm8921_chg_chip *chip= the_chip;
	return is_usb_chg_plugged_in(chip) || is_dc_chg_plugged_in(chip);
}
extern void cancel_charger_type_detect_work(void);
static void cancel_charge_det_worker(struct work_struct *work)
{
	cancel_charger_type_detect_work();
}
#endif
/*OPPO,Jiangsm add end*/
#define VIN_MIN_COLLAPSE_CHECK_MS	50
static irqreturn_t usbin_valid_irq_handler(int irq, void *data)
{
	pr_err("%s:!!!!!!!!,usb_target_ma=%d\n", __func__, usb_target_ma);
/*OPPO,Jiangsm add begin for bad charger type detecting logic,2013-1-22*/
#ifdef CONFIG_VENDOR_EDIT
	schedule_work(&the_chip->cancel_charge_det);
#endif
/*OPPO,Jiangsm add end*/
	if (usb_target_ma)
		schedule_delayed_work(&the_chip->vin_collapse_check_work,
				      round_jiffies_relative(msecs_to_jiffies
						(VIN_MIN_COLLAPSE_CHECK_MS)));
	else
	    handle_usb_insertion_removal(data);
	return IRQ_HANDLED;
}

static irqreturn_t batt_inserted_irq_handler(int irq, void *data)
{
/* OPPO 2012-11-30 chendx Delete begin for oppo battery remove check */
#ifndef CONFIG_VENDOR_EDIT

	struct pm8921_chg_chip *chip = data;
	int status;

	status = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
	schedule_work(&chip->battery_id_valid_work);
	handle_start_ext_chg(chip);
	pr_debug("battery present=%d", status);
	power_supply_changed(&chip->batt_psy);
#endif
/* OPPO 2012-11-30 chendx Delete end */
	return IRQ_HANDLED;
}

/*
 * this interrupt used to restart charging a battery.
 *
 * Note: When DC-inserted the VBAT can't go low.
 * VPH_PWR is provided by the ext-charger.
 * After End-Of-Charging from DC, charging can be resumed only
 * if DC is removed and then inserted after the battery was in use.
 * Therefore the handle_start_ext_chg() is not called.
 */
static irqreturn_t vbatdet_low_irq_handler(int irq, void *data)
{
/* OPPO 2012-08-20 chendx Modify begin for recharging not use vbatdet irq */
#ifndef CONFIG_VENDOR_EDIT

	struct pm8921_chg_chip *chip = data;
	int high_transition;

	high_transition = pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ);

	if (high_transition) {
		/* enable auto charging */
		pm_chg_auto_enable(chip, !charging_disabled);
		pr_info("batt fell below resume voltage %s\n",
			charging_disabled ? "" : "charger enabled");
	}
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

	return IRQ_HANDLED;
#else
	return IRQ_HANDLED;
#endif
/* OPPO 2012-08-20 chendx Modify end */
	return IRQ_HANDLED;
}

static irqreturn_t chgwdog_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vcp_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcdone_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcfail_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t chgdone_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));

	handle_stop_ext_chg(chip);

	/* OPPO 2012-08-09 chendx Add begin for reason */
	#ifdef CONFIG_VENDOR_EDIT
	pm8921_charger_enable(false);
	#endif
	/* OPPO 2012-08-09 chendx Add end */
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

	bms_notify_check(chip);

	return IRQ_HANDLED;
}

static irqreturn_t chgfail_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int ret;

	if (!chip->stop_chg_upon_expiry) {
		ret = pm_chg_failed_clear(chip, 1);
		if (ret)
			pr_err("Failed to write CHG_FAILED_CLEAR bit\n");
	}

	pr_err("batt_present = %d, batt_temp_ok = %d, state_changed_to=%d\n",
			get_prop_batt_present(chip),
			pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ),
			pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chgstate_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);

	bms_notify_check(chip);

	return IRQ_HANDLED;
}

enum {
	PON_TIME_25NS	= 0x04,
	PON_TIME_50NS	= 0x08,
	PON_TIME_100NS	= 0x0C,
};

static void set_min_pon_time(struct pm8921_chg_chip *chip, int pon_time_ns)
{
	u8 temp;
	int rc;

	rc = pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0x40);
	if (rc) {
		pr_err("Failed to write 0x70 to CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, &temp);
	if (rc) {
		pr_err("Failed to read CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	/* clear the min pon time select bit */
	temp &= 0xF3;
	/* set the pon time */
	temp |= (u8)pon_time_ns;
	/* write enable bank 4 */
	temp |= 0x80;
	rc = pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to CTRL_TEST3 rc=%d\n", temp, rc);
		return;
	}
}

static void attempt_reverse_boost_fix(struct pm8921_chg_chip *chip)
{
	pr_debug("Start\n");
	set_min_pon_time(chip, PON_TIME_100NS);
	pm_chg_vinmin_set(chip, chip->vin_min + 200);
	msleep(250);
	pm_chg_vinmin_set(chip, chip->vin_min);
	set_min_pon_time(chip, PON_TIME_25NS);
	pr_debug("End\n");
}

#define VIN_ACTIVE_BIT BIT(0)
#define UNPLUG_WRKARND_RESTORE_WAIT_PERIOD_US	200
#define VIN_MIN_INCREASE_MV	100
static void unplug_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, unplug_check_work);
	u8 reg_loop = 0, active_path;
	int rc, ibat, active_chg_plugged_in, usb_ma;
	int chg_gone = 0;
	bool ramp = false;

	rc = pm8xxx_readb(chip->dev->parent, PBL_ACCESS1, &active_path);
	if (rc) {
		pr_err("Failed to read PBL_ACCESS1 rc=%d\n", rc);
		return;
	}

	chip->active_path = active_path;
	active_chg_plugged_in = is_active_chg_plugged_in(chip, active_path);
	pr_debug("active_path = 0x%x, active_chg_plugged_in = %d\n",
			active_path, active_chg_plugged_in);
	if (active_path & USB_ACTIVE_BIT) {
		pr_debug("USB charger active\n");

		pm_chg_iusbmax_get(chip, &usb_ma);

		if (usb_ma <= 100) {
			pr_debug(
				"Unenumerated or suspended usb_ma = %d skip\n",
				usb_ma);
			goto check_again_later;
		}
	} else if (active_path & DC_ACTIVE_BIT) {
		pr_debug("DC charger active\n");
	} else {
		/* No charger active */
		if (!(is_usb_chg_plugged_in(chip)
				&& !(is_dc_chg_plugged_in(chip)))) {
			get_prop_batt_current(chip, &ibat);
			pr_debug(
			"Stop: chg removed reg_loop = %d, fsm = %d ibat = %d\n",
				pm_chg_get_regulation_loop(chip),
				pm_chg_get_fsm_state(chip), ibat);
			return;
		} else {
			goto check_again_later;
		}
	}

	/* AICL only for usb wall charger */
	if ((active_path & USB_ACTIVE_BIT) && usb_target_ma > 0 &&
		!chip->disable_aicl) {
		reg_loop = pm_chg_get_regulation_loop(chip);
		pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);
		if ((reg_loop & VIN_ACTIVE_BIT) &&
			(usb_ma > USB_WALL_THRESHOLD_MA)
			&& !charging_disabled) {
			decrease_usb_ma_value(&usb_ma);
			usb_target_ma = usb_ma;
			/* end AICL here */
			__pm8921_charger_vbus_draw(usb_ma);
			pr_debug("usb_now=%d, usb_target = %d\n",
				usb_ma, usb_target_ma);
		}
	}

	reg_loop = pm_chg_get_regulation_loop(chip);
	pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);

	rc = get_prop_batt_current(chip, &ibat);
	if ((reg_loop & VIN_ACTIVE_BIT) && !chip->disable_chg_rmvl_wrkarnd) {
		if (ibat > 0 && !rc) {
			pr_debug("revboost ibat = %d fsm = %d loop = 0x%x\n",
				ibat, pm_chg_get_fsm_state(chip), reg_loop);
			attempt_reverse_boost_fix(chip);
			/* after reverse boost fix check if the active
			 * charger was detected as removed */
			active_chg_plugged_in
				= is_active_chg_plugged_in(chip,
					active_path);
			pr_debug("revboost post: active_chg_plugged_in = %d\n",
					active_chg_plugged_in);
		}
	}

	active_chg_plugged_in = is_active_chg_plugged_in(chip, active_path);
	pr_debug("active_path = 0x%x, active_chg = %d\n",
			active_path, active_chg_plugged_in);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	if (chg_gone == 1  && active_chg_plugged_in == 1 &&
					!chip->disable_chg_rmvl_wrkarnd) {
		pr_debug("chg_gone=%d, active_chg_plugged_in = %d\n",
					chg_gone, active_chg_plugged_in);
		unplug_ovp_fet_open(chip);
	}

	/* AICL only for usb wall charger */
	if (!(reg_loop & VIN_ACTIVE_BIT) && (active_path & USB_ACTIVE_BIT)
		&& usb_target_ma > 0
		&& !charging_disabled
		&& !chip->disable_aicl) {
		/* only increase iusb_max if vin loop not active */
		if (usb_ma < usb_target_ma) {
			increase_usb_ma_value(&usb_ma);
			if (usb_ma > usb_target_ma)
				usb_ma = usb_target_ma;
			__pm8921_charger_vbus_draw(usb_ma);
			pr_debug("usb_now=%d, usb_target = %d\n",
					usb_ma, usb_target_ma);
			ramp = true;
		} else {
			usb_target_ma = usb_ma;
		}
	}
check_again_later:
	pr_debug("ramp: %d\n", ramp);
	/* schedule to check again later */
	if (ramp)
		schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_RAMP_MS));
	else
		schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_WAIT_PERIOD_MS));
}

static irqreturn_t loop_change_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("fsm_state=%d reg_loop=0x%x\n",
		pm_chg_get_fsm_state(data),
		pm_chg_get_regulation_loop(data));
	schedule_work(&chip->unplug_check_work.work);
	return IRQ_HANDLED;
}

struct ibatmax_max_adj_entry {
	int ibat_max_ma;
	int max_adj_ma;
};

static struct ibatmax_max_adj_entry ibatmax_adj_table[] = {
	{975, 300},
	{1475, 150},
	{1975, 200},
	{2475, 250},
};

static int find_ibat_max_adj_ma(int ibat_target_ma)
{
	int i = 0;

	for (i = ARRAY_SIZE(ibatmax_adj_table); i > 0; i--) {
		if (ibat_target_ma >= ibatmax_adj_table[i - 1].ibat_max_ma)
			break;
	}

	if (i > 0)
		i--;

	return ibatmax_adj_table[i].max_adj_ma;
}

static irqreturn_t fastchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;

	high_transition = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (high_transition && !delayed_work_pending(&chip->eoc_work)) {
		wake_lock(&chip->eoc_wake_lock);
		schedule_delayed_work(&chip->eoc_work,
				      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
	}
	if (high_transition
		&& chip->btc_override
		&& !delayed_work_pending(&chip->btc_override_work)) {
		schedule_delayed_work(&chip->btc_override_work,
					round_jiffies_relative(msecs_to_jiffies
						(chip->btc_delay_ms)));
	}
	power_supply_changed(&chip->batt_psy);
	bms_notify_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t trklchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batt_removed_irq_handler(int irq, void *data)
{
	/* OPPO 2012-11-30 chendx Delete begin for temp check batt remove */
	#ifndef CONFIG_VENDOR_EDIT
	struct pm8921_chg_chip *chip = data;
	int status;

	status = pm_chg_get_rt_status(chip, BATT_REMOVED_IRQ);
	pr_info("battery present=%d state=%d", !status,
					 pm_chg_get_fsm_state(data));

	handle_stop_ext_chg(chip);
	power_supply_changed(&chip->batt_psy);
	#endif
	/* OPPO 2012-11-30 chendx Delete end */
	return IRQ_HANDLED;
}

static irqreturn_t batttemp_hot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chghot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("Chg hot fsm_state=%d\n", pm_chg_get_fsm_state(data));
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	handle_stop_ext_chg(chip);
	return IRQ_HANDLED;
}

static irqreturn_t batttemp_cold_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("Batt cold fsm_state=%d\n", pm_chg_get_fsm_state(data));
	handle_stop_ext_chg(chip);

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chg_gone_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int chg_gone, usb_chg_plugged_in;

	usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	pr_debug("chg_gone=%d, usb_valid = %d\n", chg_gone, usb_chg_plugged_in);
	pr_debug("Chg gone fsm_state=%d\n", pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	return IRQ_HANDLED;
}
/*
 *
 * bat_temp_ok_irq_handler - is edge triggered, hence it will
 * fire for two cases:
 *
 * If the interrupt line switches to high temperature is okay
 * and thus charging begins.
 * If bat_temp_ok is low this means the temperature is now
 * too hot or cold, so charging is stopped.
 *
 */
static irqreturn_t bat_temp_ok_irq_handler(int irq, void *data)
{
/* OPPO 2012-12-04 chendx Delete begin for not use oppo charge */
#ifndef CONFIG_VENDOR_EDIT
	int bat_temp_ok;
	struct pm8921_chg_chip *chip = data;

	bat_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	pr_debug("batt_temp_ok = %d fsm_state%d\n",
			 bat_temp_ok, pm_chg_get_fsm_state(data));

	if (bat_temp_ok)
		handle_start_ext_chg(chip);
	else
		handle_stop_ext_chg(chip);

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	bms_notify_check(chip);
#endif
/* OPPO 2012-12-04 chendx Delete end */
	return IRQ_HANDLED;
}

static irqreturn_t coarse_det_low_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vdd_loop_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vreg_ov_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vbatdet_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t batfet_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("vreg ov\n");
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_valid_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int dc_present;

	pm_chg_failed_clear(chip, 1);
	dc_present = pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);

	if (chip->dc_present ^ dc_present)
		pm8921_bms_calibrate_hkadc();

	if (dc_present)
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	else
		pm8921_chg_disable_irq(chip, CHG_GONE_IRQ);

	chip->dc_present = dc_present;

	if (chip->ext_psy) {
		if (dc_present)
			handle_start_ext_chg(chip);
		else
			handle_stop_ext_chg(chip);
	} else {
		if (dc_present)
			schedule_delayed_work(&chip->unplug_check_work,
				msecs_to_jiffies(UNPLUG_CHECK_WAIT_PERIOD_MS));
		power_supply_changed(&chip->dc_psy);
	}

	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_ov_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_uv_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);

	return IRQ_HANDLED;
}

static int __pm_batt_external_power_changed_work(struct device *dev, void *data)
{
	struct power_supply *psy = &the_chip->batt_psy;
	struct power_supply *epsy = dev_get_drvdata(dev);
	int i, dcin_irq;

	/* Only search for external supply if none is registered */
	if (!the_chip->ext_psy) {
		dcin_irq = the_chip->pmic_chg_irq[DCIN_VALID_IRQ];
		for (i = 0; i < epsy->num_supplicants; i++) {
			if (!strncmp(epsy->supplied_to[i], psy->name, 7)) {
				if (!strncmp(epsy->name, "dc", 2)) {
					the_chip->ext_psy = epsy;
					dcin_valid_irq_handler(dcin_irq,
							the_chip);
				}
			}
		}
	}
	return 0;
}

static void pm_batt_external_power_changed(struct power_supply *psy)
{
	if (!the_chip)
		return;

	/* Only look for an external supply if it hasn't been registered */
	if (!the_chip->ext_psy)
		class_for_each_device(power_supply_class, NULL, psy,
					 __pm_batt_external_power_changed_work);
}
/* OPPO 2012-12-04 chendx add begin for oppo charge */
#ifdef CONFIG_VENDOR_EDIT
/*add by chedx  Add begin for batt health */
static int set_prop_batt_health(struct pm8921_chg_chip *chip, int batt_health)
{
	chip->batt_health = batt_health;
	return 0;
}

/* Add begin for oppo charge status capable */
static bool is_capable_of_charging(struct pm8921_chg_chip *chip)
{
   	if( pm8921_charger_status_get(chip) == CHARGER_STATUS_WEAK ||
		pm8921_charger_status_get(chip) == CHARGER_STATUS_OVER ||
		pm8921_battery_status_get(chip) == BATTERY_STATUS_BAD ||
		get_prop_batt_health(chip) == POWER_SUPPLY_HEALTH_UNKNOWN ||
		get_prop_batt_health(chip) == POWER_SUPPLY_HEALTH_OVERHEAT ||
		get_prop_batt_health(chip) == POWER_SUPPLY_HEALTH_COLD ||
		chip->safe_charge_teoc == true)
		return 0;
	else 
		return 1;
}

/*
*  @pm8921_chg_start_charging()
 * this function used to start charging a battery.
 * start charging with Battery temperate
 * start charing with battery reconnect
 */
static int pm8921_chg_start_charging(struct pm8921_chg_chip *chip)
{
    if(!chip){
		pr_err("chip is not init\n");
	    return -1;
    }
	print_pm8921(DEBUG_INFO,"start charing!!!\n");
	
	pm8921_chg_temp_state_reset(the_chip);
	pm8921_charger_enable(true);

	/*start charging from Battery temperate*/
	pm8921_battery_temp_handle(the_chip);

	return 0;
}

/*
*  @pm8921_chg_set_input_current_max()
* set input current with charger type
*/
static int pm8921_chg_set_input_current_max(struct pm8921_chg_chip *chip)
{
    int rc=0;
    if(!chip){
		pr_err("%s,chip is not init\n",__func__);
	    return -1;
    }
	
	switch (chip->pm8921_chg_type)
	{
		case USB_DCP_CHARGER:
		{
			__pm8921_charger_vbus_draw(IDEV_CHG_MAX);
			break;
		}
		case USB_SDP_CHARGER:
		case USB_NON_DCP_CHARGER:
		case USB_HDMI_CHARGER:
		{
			__pm8921_charger_vbus_draw(IDEV_CHG_MIN);
			break;
		}
		default:
		{
			rc = -1;
			pr_err("invalid charger type!\n");
			break;
		}
	}

	return rc;
}


/*
*  @pm8921_chg_stop_charging()
 * this function used to stop charging a battery.
 * stop charging with Battery temperate
 * stop charing with battery remove
 */

static int pm8921_chg_stop_charging(struct pm8921_chg_chip *chip)
{
    if(!chip){
		pr_err("chip is not init\n");
	    return -1;
    }

	print_pm8921(DEBUG_INFO,"%s: stop charing!\n",__func__);
	pm8921_charger_enable(false);
    if (delayed_work_pending(&chip->recharge_monitor_work)){
			cancel_delayed_work(&chip->recharge_monitor_work);
	}
	return 0;
}

int pm8921_chg_connected(enum usb_chg_type chg_type)
{
	char *chg_types[] = {
			"CHARGER REMOVE",
		    "STD DOWNSTREAM PORT(USB Charger)",
			"DEDICATED CHARGER(STANDARD Charger)",
			"NON-STANDARD Charger",
			"HDMI Charger",			
			"USB CDP CHARGER",
		    "USB ACA A CHARGER",
			"USB ACA B CHARGER",
			"USB ACA C CHARGER",
			"USB ACA DOCK CHARGER"};

	pr_info("Charger Type: #### :%s\n",chg_types[chg_type]);

	if (!the_chip) {
		pr_err("%s:called before init\n",__func__);
		return -EINVAL;
	}

	the_chip->pm8921_chg_type = chg_type;

	/*reset pm8921 charger state when charger connect or disconnect*/
	pm8921_chg_state_reset(the_chip);
	
	if(chg_type == USB_SDP_CHARGER ||
		chg_type == USB_DCP_CHARGER ||
		chg_type == USB_NON_DCP_CHARGER ||
		chg_type == USB_HDMI_CHARGER
		)
	{
	    
		/*reset health status when plugin*/
		if(get_prop_batt_health(the_chip) == POWER_SUPPLY_HEALTH_UNKNOWN)
			set_prop_batt_health(the_chip,POWER_SUPPLY_HEALTH_GOOD);
	
	    if(chg_type == USB_SDP_CHARGER){
			the_chip->usb_psy.type = POWER_SUPPLY_TYPE_USB;
			power_supply_changed(&the_chip->usb_psy);
		}else if(chg_type == USB_DCP_CHARGER ||
			chg_type == USB_NON_DCP_CHARGER ){
			the_chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS;
			power_supply_changed(&the_chip->dc_psy);
		}
			
	    schedule_delayed_work(&the_chip->charger_valid_work,
			      round_jiffies_relative(msecs_to_jiffies
						     	(CHARGE_ENABLE_DELAY)));	
	    /*set the wake lock, do not let sytem sleep. */
		#ifdef CONFIG_HAS_WAKELOCK
		wake_lock(&the_chip->pm8921_wake_lock);
		#endif	
		
	}else if(chg_type == USB_INVALID_CHARGER){
	    power_supply_changed(&the_chip->batt_psy);
			
	    /* set the wake lock, do not let sytem sleep. */
		#ifdef CONFIG_HAS_WAKELOCK
		/*add timeout wake lock to prevent suspend quickly ,make sure system resume*/
		wake_lock_timeout(&the_chip->prevent_suspend_lock, HZ);
		wake_unlock(&the_chip->pm8921_wake_lock);
		#endif
	    pm8921_chg_stop_charging(the_chip);
		/**
             **XXX start teoc work
             */
	    teoc_work_schedule(false,the_chip);
	}	
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_chg_connected);

/*Add begin for BTM */
 static chg_cv_battery_temp_region_type Pm8921_battery_temp_region_get(struct pm8921_chg_chip *chip)
 {
	 return chip->mBatteryTempRegion;
 }
 static void Pm8921_battery_temp_region_set(struct pm8921_chg_chip *chip, 
 													chg_cv_battery_temp_region_type batt_temp_region)
 {
	 chip->mBatteryTempRegion = batt_temp_region;
 }
 static int Pm8921_battery_set_litle_cold_params(struct pm8921_chg_chip *chip)
 {
	pm8921_set_max_battery_charge_current(chip->little_cold_bat_chg_current);
	pm_chg_vddmax_set(chip, chip->little_cold_bat_voltage);
	pm_chg_vbatdet_set(chip,
		chip->little_cold_bat_voltage
		- chip->resume_voltage_delta);

	return 0;
 }
 
 static int Pm8921_battery_set_cool_params(struct pm8921_chg_chip *chip)
 {
    int charge_cool_current = 750;
	
    /*set ibat with new OPPO charge standard*/
	if(chip->pm8921_chg_type == USB_DCP_CHARGER){
		pm8921_set_max_battery_charge_current(charge_cool_current);
	}else
		pm8921_set_max_battery_charge_current(chip->cool_bat_chg_current);
	
	pm_chg_vddmax_set(chip, chip->cool_bat_voltage);
	pm_chg_vbatdet_set(chip,
		chip->cool_bat_voltage
		- chip->resume_voltage_delta);

	return 0;
 }
 
 static int Pm8921_battery_set_normal_params(struct pm8921_chg_chip *chip)
 {
    if(the_chip->pm8921_chg_type == USB_SDP_CHARGER)
    {
		pm8921_set_max_battery_charge_current(chip->normal_sdp_chg_current);
    }else if(the_chip->pm8921_chg_type == USB_DCP_CHARGER){
    	pm8921_set_max_battery_charge_current(chip->normal_dcp_chg_current);
    }else if(the_chip->pm8921_chg_type == USB_HDMI_CHARGER &&
      chip->stanard_mhl_chg == true){
        pm8921_set_max_battery_charge_current(chip->mhl_chg_current);
    }else{
   	 	pm8921_set_max_battery_charge_current(chip->normal_sdp_chg_current);
    }
	pm_chg_vddmax_set(chip, chip->normal_bat_voltage);
	pm_chg_vbatdet_set(chip,
		chip->normal_bat_voltage
		- chip->normal_resume_voltage_delta);

	return 0;
 }

static int Pm8921_battery_set_warm_params(struct pm8921_chg_chip *chip)
{
	int charge_warm_current = 750;
	
    /*set ibat with new OPPO charge standard*/
	if(chip->pm8921_chg_type == USB_DCP_CHARGER){
		pm8921_set_max_battery_charge_current(charge_warm_current);
	}else
		pm8921_set_max_battery_charge_current(chip->warm_bat_chg_current);
	
	pm_chg_vddmax_set(chip, chip->warm_bat_voltage);
	pm_chg_vbatdet_set(chip,
		chip->warm_bat_voltage
		- chip->resume_voltage_delta);

	return 0;
}


/*Tbatt <-10C*/
static int handle_batt_temp_cold(struct pm8921_chg_chip *chip)
{
	if (Pm8921_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__COLD)
	{
		print_pm8921(DEBUG_INFO, "%s\n", __func__);

        /*set to cool params to fix cold status change to little_cold failed sometimes*/
		Pm8921_battery_set_cool_params(chip);
		
		/* Update battery temp region */
		Pm8921_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__COLD);

		
		/*temp cold sotp charge*/
		pm8921_charger_enable(false);
		
		power_supply_changed(&chip->batt_psy);
		//power_supply_changed(&chip->usb_psy);
		//power_supply_changed(&chip->dc_psy);
		bms_notify_check(chip);
		
		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0 + AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COLD_TO_COOL;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1 + AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COLD_TO_COOL;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2 + AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COOL_TO_NORMAL;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;

		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_COLD);
	}
	return 0;
}

/* -10 C <=Tbatt <= 0C*/
static int handle_batt_temp_little_cold(struct pm8921_chg_chip *chip)
{
	chg_cv_battery_temp_region_type batt_temp_region_pre;

	if (Pm8921_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION_LITTLE__COLD)
	{
		/*if temp from cold or hot to cool then start charging again*/
		batt_temp_region_pre = Pm8921_battery_temp_region_get(chip);
		
		print_pm8921(DEBUG_INFO, "%s,pre_region=%d\n", __func__,batt_temp_region_pre);
		if(batt_temp_region_pre == CV_BATTERY_TEMP_REGION__COLD || 
				batt_temp_region_pre == CV_BATTERY_TEMP_REGION__HOT){
			pm8921_charger_enable(true);
		}

		/* Update battery temp region */
		Pm8921_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION_LITTLE__COLD);
		
		/*set little_cold params here*/
		Pm8921_battery_set_litle_cold_params(chip);
		
		power_supply_changed(&chip->batt_psy);
		//power_supply_changed(&chip->usb_psy);
		//power_supply_changed(&chip->dc_psy);
		bms_notify_check(chip);
		
		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1 + AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COOL_TO_NORMAL;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2 + AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COOL_TO_NORMAL;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;

		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);
	}
	return 0;
}
 
/* 0 C <Tbatt <= 10C*/
static int handle_batt_temp_cool(struct pm8921_chg_chip *chip)
{
	chg_cv_battery_temp_region_type batt_temp_region_pre;

	if (Pm8921_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__COOL)
	{
       	print_pm8921(DEBUG_INFO, "%s\n", __func__);
		//if temp from cold or hot to cool then start charging again
		batt_temp_region_pre = Pm8921_battery_temp_region_get(chip);
		if(batt_temp_region_pre == CV_BATTERY_TEMP_REGION__COLD || 
				batt_temp_region_pre == CV_BATTERY_TEMP_REGION__HOT){
			pm8921_charger_enable(true);
		}

		/* Update battery temp region */
		Pm8921_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__COOL);

		//set cool params here
		Pm8921_battery_set_cool_params(chip);
		
		power_supply_changed(&chip->batt_psy);
		//power_supply_changed(&chip->usb_psy);
		//power_supply_changed(&chip->dc_psy);
		bms_notify_check(chip);
		
		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2 + AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COOL_TO_NORMAL;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;

		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);

	}
	return 0;
}
 
/* 10 C <Tbatt <45C*/
static int handle_batt_temp_normal(struct pm8921_chg_chip *chip)
{
	chg_cv_battery_temp_region_type batt_temp_region_pre;

	if (Pm8921_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__NORMAL)
	{
		print_pm8921(DEBUG_INFO, "%s\n", __func__);
		//if temp from cold or hot to cool then start charging again
		batt_temp_region_pre = Pm8921_battery_temp_region_get(chip);
		if (batt_temp_region_pre == CV_BATTERY_TEMP_REGION__COLD ||
				batt_temp_region_pre == CV_BATTERY_TEMP_REGION__HOT){
			//handle_start_ext_chg(chip);
			pm8921_charger_enable(true);
		}

		/* Update battery temp region */
		Pm8921_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__NORMAL);
		
		//set normal params here
		Pm8921_battery_set_normal_params(chip);
		
		power_supply_changed(&chip->batt_psy);
		//power_supply_changed(&chip->usb_psy);
		//power_supply_changed(&chip->dc_psy);
		bms_notify_check(chip);

		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;

		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);

	}
	return 0;
}
 
/* 45C <=Tbatt <=55C*/
static int handle_batt_temp_warm(struct pm8921_chg_chip *chip)
{
	chg_cv_battery_temp_region_type batt_temp_region_pre;
	
	if(Pm8921_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__WARM)
	{
	    
		print_pm8921(DEBUG_INFO, "%s\n", __func__);
		//if temp from cold or hot to cool then start charging again
		batt_temp_region_pre = Pm8921_battery_temp_region_get(chip);
		if (batt_temp_region_pre == CV_BATTERY_TEMP_REGION__COLD ||
				batt_temp_region_pre == CV_BATTERY_TEMP_REGION__HOT){
			//handle_start_ext_chg(chip);
			pm8921_charger_enable(true);
		}

		/* Update battery temp region */
		Pm8921_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__WARM);
		
		//set warm params here
		Pm8921_battery_set_warm_params(chip);
		
		power_supply_changed(&chip->batt_psy);
		//power_supply_changed(&chip->usb_psy);
		//power_supply_changed(&chip->dc_psy);
		bms_notify_check(chip);

		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3 - AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_WARM_TO_NORMAL;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;

		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);

	}
	return 0;	
}
 
/* 55C <Tbatt*/
static int handle_batt_temp_hot(struct pm8921_chg_chip *chip)
{
	if(Pm8921_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__HOT)
	{
	
		print_pm8921(DEBUG_INFO, "%s\n", __func__);
		/* Update battery temp region */
		Pm8921_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__HOT);

	    /*temp hot sotp charge*/
		//handle_stop_ext_chg(chip);
		pm8921_charger_enable(false);
		
		power_supply_changed(&chip->batt_psy);
		//power_supply_changed(&chip->usb_psy);
		//power_supply_changed(&chip->dc_psy);
		bms_notify_check(chip);

		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3 - AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_WARM_TO_NORMAL;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4 - AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_HOT_TO_WARM;

		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_OVERHEAT);
	}
	return 0;
}


static int pm8921_battery_temp_handle(struct pm8921_chg_chip *chip)
{
	int rc = -1;
	int temperature = chip->battery_temp;

	print_pm8921(DEBUG_TRACE, "%s: temperature =%d, region =%d\n", 
	 		__func__, temperature, Pm8921_battery_temp_region_get(chip));
	
    if(temperature < chip->mBatteryTempBoundT0 &&
		 temperature > AUTO_CHARGING_BATT_REMOVE_TEMP) /* battery is cold */
    {
            rc = handle_batt_temp_cold(chip);
    } 
 	else if( (temperature >=  chip->mBatteryTempBoundT0) && 
             (temperature <= chip->mBatteryTempBoundT1) ) /* battery is more cool */
    {
            rc = handle_batt_temp_little_cold(chip);
    }
    else if( (temperature >=  chip->mBatteryTempBoundT1) && 
             (temperature <= chip->mBatteryTempBoundT2) ) /* battery is cool */
    {
            rc = handle_batt_temp_cool(chip);
    }
    else if( (temperature > chip->mBatteryTempBoundT2) && 
             (temperature < chip->mBatteryTempBoundT3) ) /* battery is normal */
    {
            rc = handle_batt_temp_normal(chip);
    }
    else if( (temperature >= chip->mBatteryTempBoundT3) && 
             (temperature <=  chip->mBatteryTempBoundT4) ) /* battery is warm */
    {
            rc = handle_batt_temp_warm(chip);
    }
    else if(temperature > chip->mBatteryTempBoundT4)/* battery is hot */
    {
            rc = handle_batt_temp_hot(chip);
    }
		
	return rc;
}
static int pm8921_chg_temp_state_reset(struct pm8921_chg_chip *chip)
{

	Pm8921_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__INVALID);
	chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
	chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1; 
	chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2;
	chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
	chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;

	return 0;
}

/**
*  reset pm8921 charger state to init state,
*  xxxx only reset when charger connect and disconnect
*/
static int pm8921_chg_state_reset(struct pm8921_chg_chip *chip)
{

    /*
        *XXX:only reset when charger connect and disconnect
	  */
    pm8921_charger_status_set(chip, CHARGER_STATUS_GOOD);
    pm8921_battery_status_set(chip, BATTERY_STATUS_GOOD);
	set_prop_batt_health(chip,POWER_SUPPLY_HEALTH_GOOD);
	pm8921_chg_temp_state_reset(chip);
	chip->stanard_mhl_chg = false;
	chip->recharging_counter = 0;
	chip->charge_is_finished = false;
	chip->safe_charge_teoc = false;
	chip->bad_charger_check_time = BAD_STATE_COUNT;
	chip->soc_fall_status = false;
	chip->soc_charge_counter = 0;
	chip->capacity_saltate_counter = 0;

	chip->eoc_vbatt_counter = 0;
	chip->cv_long_counter = 0;

	return 0;
}

#ifndef FEATURE_AUTO_RECHARGE
/* Add begin for recharging  monitor*/
#define RECHARGING_COUNTER 5
#define AUTO_CHARGING_RESUME_CHARG_VOLT_WARM	3900 
#define AUTO_CHARGING_RESUME_CHARG_VOLT_COOL 	4200  
#define AUTO_CHARGING_RESUME_CHARG_VOLT_LITTLECOLD 3800  
#define AUTO_CHARGING_RESUME_CHARG_VOLT	4200
static int pm8921_chg_recharging_monitor(struct pm8921_chg_chip *chip,bool *recharging_disable)
{
	int vbatt_resume=0;

    /*set vbatt resume voltage*/
	if (Pm8921_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__NORMAL){
		vbatt_resume = AUTO_CHARGING_RESUME_CHARG_VOLT;
	}else if (Pm8921_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__WARM){
		vbatt_resume = AUTO_CHARGING_RESUME_CHARG_VOLT_WARM;
	}else if (Pm8921_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__COOL){
		vbatt_resume = AUTO_CHARGING_RESUME_CHARG_VOLT_COOL;
	}else if (Pm8921_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION_LITTLE__COLD){
		vbatt_resume = AUTO_CHARGING_RESUME_CHARG_VOLT_LITTLECOLD;
	}
	
	if(chip->battery_voltage < vbatt_resume)
          chip->recharging_counter++;
	else
		  chip->recharging_counter = 0;
    
	print_pm8921(DEBUG_INFO,"%s,VBAT=%dmv,%dmv,counter=%d\n",
			__func__,chip->battery_voltage,vbatt_resume,chip->recharging_counter);
	
	if(chip->recharging_counter >= RECHARGING_COUNTER)
	{
	   	print_pm8921(DEBUG_INFO,"%s,vbat fell below resume voltage,recharging!\n",__func__);
		/* enable auto charging */
		pm_chg_auto_enable(chip, 1);
		chip->recharging_counter = 0;
		*recharging_disable = true;
	}else{
        *recharging_disable = false;
		power_supply_changed(&chip->batt_psy);
		//power_supply_changed(&chip->usb_psy);
		//power_supply_changed(&chip->dc_psy);
	}

	return 0;

}

static void recharging_monitor_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
		struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, recharge_monitor_work);
		
     bool recharging_disable=false;
	 
     if(!chip)
     {
        pr_err("chip is NULL!\n");
		return;
     }
	 
	 pm8921_chg_recharging_monitor(chip,&recharging_disable);
	 
     if(!recharging_disable){
	 	schedule_delayed_work(&chip->recharge_monitor_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (RECHARGING_MONITOR_MS)));	
     }else{
         print_pm8921(DEBUG_INFO,"%s:Recharging monitor complete ,=%d\n",
		 				__func__,recharging_disable);
     }

	 return;
}
#endif

/* Add begin for mhl charge*/
int mhl_stanard_charge(void)
{
	complete(&stanard_mhl_wait);
    pr_info("%s: stanard mhl charge set!\n",__func__);
	return 0;
}
EXPORT_SYMBOL(mhl_stanard_charge);

#define STANARD_MHL_MA 1000
#define NONSTANARD_MHL_MA 500
static int pm8921_set_mhl__stanard(struct pm8921_chg_chip *chip)
{
	if(!chip)
		pr_err("chip is NULL\n");

	__pm8921_charger_vbus_draw(STANARD_MHL_MA);
	pm8921_set_max_battery_charge_current(chip->mhl_chg_current);
	chip->stanard_mhl_chg = true;
	
	pr_info("%s\n", __func__);
	return 0;	
}

static int pm8921_set_mhl__nonstanard(struct pm8921_chg_chip *chip)
{
	if(!chip)
		pr_err("chip is NULL\n");
	
    __pm8921_charger_vbus_draw(NONSTANARD_MHL_MA);
	pm8921_set_max_battery_charge_current(chip->nonstanard_mhl_chg_current);
	chip->stanard_mhl_chg = false;
	pr_info("%s\n", __func__);
	
	return 0;	
}

static int pm8921_set_charge_mhl(struct pm8921_chg_chip *chip)
{
	int rc = 0;	
	pr_info("%s\n", __func__);

	if(!chip)
		pr_err("chip is NULL\n");
	
	/*wait 1500ms for stanard mhl device */
    if (!wait_for_completion_timeout(&stanard_mhl_wait,
		 msecs_to_jiffies(1500))){
		 rc = pm8921_set_mhl__nonstanard(chip);
	     if(rc)
			   pr_err("nonstanard set failed!\n");
		 pr_info("timer out nonstandard mhl\n");
      }else{
		  rc = pm8921_set_mhl__stanard(chip);
		  if(rc)
				pr_err("stanard set failed!\n");
    }
	return rc;
}

/* Add begin for charger and battery uovp  */
static chg_battery_status pm8921_battery_status_get(struct pm8921_chg_chip *chip)
{
	return chip->battery_status;
}
static void pm8921_battery_status_set(struct pm8921_chg_chip *chip,
											  chg_battery_status battery_status)
{
	chip->battery_status = battery_status;
}

static int pm8921_handle_battery_uovp(struct pm8921_chg_chip *chip)
{
	print_pm8921(DEBUG_INFO,"%s\n", __func__);

	/* in battery voltage err , stop charging first */
	pm8921_chg_stop_charging(chip);
	set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_OVERVOLTAGE);
	
	return 0;
}

static int pm8921_handle_battery_restore_from_uovp(struct pm8921_chg_chip *chip)
{
	print_pm8921(DEBUG_INFO,"%s\n", __func__);

	/*restore charging form battery ovp*/
	pm8921_chg_start_charging(chip);
	set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);
	
	return 0;
}

static void pm8921_check_battery_uovp(struct pm8921_chg_chip *chip)
{
	int count = BAD_STATE_COUNT;
	int battery_voltage=0;
	chg_battery_status battery_status_pre;

    if(!chip){
		print_pm8921(DEBUG_ERROR,"%s:chip is null,error\n",__func__);
		return;
    }
	
	battery_status_pre = pm8921_battery_status_get(chip);	
	while (count--)
	{
	    battery_voltage = get_prop_battery_uvolts(chip);
		if (chip->battery_voltage > BATTERY_SOFT_OVP_VOLTAGE) { 
			print_pm8921(DEBUG_INFO,"%s:Warning, battery is high more than 4500mv,=%d\n",
												__func__,chip->mBadBatteryCounter);
			chip->mBadBatteryCounter++;
		}
		else {
			chip->mBadBatteryCounter = 0;
		}

		if ( !chip->mBadBatteryCounter) {
			print_pm8921(DEBUG_TRACE,"%s:battery voltage status is ok\n", __func__);
			pm8921_battery_status_set(chip, BATTERY_STATUS_GOOD);
			return;
		}
		msleep(20);
	}
	
	if (chip->mBadBatteryCounter == BAD_STATE_COUNT){
		print_pm8921(DEBUG_ERROR,"%s:battery mvolts is ovp\n", __func__);
		pm8921_battery_status_set(chip, BATTERY_STATUS_BAD);
		chip->mBadBatteryCounter = 0;
	}else{
		pm8921_battery_status_set(chip, BATTERY_STATUS_GOOD);
		chip->mBadBatteryCounter = 0;
	}

	/*handle battery uovp*/
	if (battery_status_pre == BATTERY_STATUS_GOOD){
		if (pm8921_battery_status_get(chip) == BATTERY_STATUS_BAD){
			print_pm8921(DEBUG_INFO,"%s:stop charing for battery uovp\n", __func__);
			pm8921_handle_battery_uovp(chip);
		}	
	}else if (battery_status_pre == BATTERY_STATUS_BAD){
		if (pm8921_battery_status_get(chip) == BATTERY_STATUS_GOOD){
			print_pm8921(DEBUG_INFO,"%s:restore charing from battery uovp\n", __func__);
			pm8921_handle_battery_restore_from_uovp(chip);
		}
	}

	return;
}

static bool is_battery_present(int batt_temp)
{
    //temperate <= -35C,we consider battery is remove
    if(batt_temp <= BATT_REMOVE_TEMP)
		return false;
	else
		return true;
}

static int handle_battery_removed(struct pm8921_chg_chip *chip)
{
	pr_info("battery remove@@\n");
	pm_chg_auto_enable(chip, 0);
	set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_UNKNOWN);
	power_supply_changed(&chip->batt_psy);
	return 0;

}

static int handle_battery_inserted(struct pm8921_chg_chip *chip)
{

	pm_chg_auto_enable(chip, 1);
	set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);
	pm8921_battery_status_set(chip, BATTERY_STATUS_GOOD);
	pm8921_charger_status_set(chip, CHARGER_STATUS_GOOD);
	Pm8921_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__INVALID);
	pr_info("battery inserted@@\n");
	power_supply_changed(&chip->batt_psy);
	return 0;
}

static void pm8921_check_battery_connect(struct pm8921_chg_chip *chip)
{
   
	int count = BAD_STATE_COUNT;
	int batt_temp=30;
	int status;

	/**
	* XXX: check battery is connect through battery temperate,read battery high
	*  BAD_STATE_COUNT time and irq status
 	*/

	if(!chip){
		print_pm8921(DEBUG_ERROR,"%s:chip is null,error\n",__func__);
		return;
	}

	while (count--)
	{

		get_prop_batt_temp(chip, &batt_temp);
		if(is_battery_present(batt_temp)){
			if(get_prop_batt_health(chip) == POWER_SUPPLY_HEALTH_UNKNOWN){
				handle_battery_inserted(chip);
			}
			return;
		}
		mdelay(20);
	}

	/*Read battery temp remove more than BAD_STATE_COUNT*/
	status = pm_chg_get_rt_status(chip, BATT_REMOVED_IRQ);
    if(status)
    { 
      if(get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_UNKNOWN){
	       /*check battery is remove irq*/
		   handle_battery_removed(chip);
      }
    }
	return;

}


static chg_charger_status pm8921_charger_status_get(struct pm8921_chg_chip *chip)
{
	return	chip->charger_status;
}

static void pm8921_charger_status_set(struct pm8921_chg_chip *chip,
											chg_charger_status charger_status)
{
	chip->charger_status = charger_status;
}

#define CHARGER_VALID_VOLTAGE 3850
/*fix bug:remove and connect charger quickly ,the read vchg voltage is 8mv
*  so that we check charger voltage bat voltage must when charge connect and voltage is valid*/
static bool is_charger_voltage_valid(struct pm8921_chg_chip *chip,
                                           int vchg_mv)
{
   if(is_usb_chg_plugged_in(chip) && vchg_mv >= CHARGER_VALID_VOLTAGE)
   	  return true;
   else
   	  return false;
}
/**
*  @pm8921_check_charger_uovp
*  check charger uovp when plugin true
*  check charger ovp when plugin false
*/
static int pm8921_vchg_compensate(int vchg_vol,struct pm8921_chg_chip *chip)
{

    int vchg_mv;

	vchg_mv = vchg_vol; 

	if(vchg_mv < CHARGER_VOLTAGE_NORMAL)
		return vchg_mv;


	/*pm8921 default input current is 500mA,When set input current to100mA the ACI is error
	  * so that we not set input current to 100mA to read charger voltage, compensate to voltage
	*/

	if(chip->bad_charger_check_time < BAD_STATE_COUNT){
		pr_info("bad charger check!! check time=%d\n",chip->bad_charger_check_time);
		if(chip->pm8921_chg_type == USB_DCP_CHARGER)
			vchg_mv = vchg_mv + 350;//350mv
		else if(chip->pm8921_chg_type == USB_NON_DCP_CHARGER)
			vchg_mv = vchg_mv + 150;//150mv
	}else{
		if(chip->pm8921_chg_type == USB_DCP_CHARGER)
			vchg_mv = vchg_mv + 150;//150mv
		else if(chip->pm8921_chg_type == USB_NON_DCP_CHARGER)
			vchg_mv = vchg_mv + 50;//100mv
	}

	return vchg_mv;

}
static void pm8921_check_charger_uovp(struct pm8921_chg_chip *chip,
												bool plugin)
{
	int count = BAD_STATE_COUNT;
	int vchg_mv = CHARGER_VOLTAGE_NORMAL;
	chg_charger_status charger_status_pre = CHARGER_STATUS_GOOD;

	charger_status_pre = pm8921_charger_status_get(chip);
	print_pm8921(DEBUG_TRACE,"%s:mChargerCheckCounter=%d,=%d,=%d\n",
				__func__,chip->mChargerCheckCounter,charger_status_pre,plugin);
	chip->mBadChargerCounter = 0;
	   
	while (count--)
	{
	    /*get vchg mvolts 3times*/
		vchg_mv = get_chg_voltage(chip);
		//pr_info("%s vchg_mv =%dmv\n", __func__,vchg_mv);
		vchg_mv = pm8921_vchg_compensate(vchg_mv,chip);
		if ((vchg_mv > CHARGER_SOFT_OVP_VOLTAGE || 
		     vchg_mv < CHARGER_SOFT_UVP_VOLTAGE) &&
			is_charger_voltage_valid(chip,vchg_mv)){ 
			chip->mBadChargerCounter++;
		}else {
			chip->mBadChargerCounter = 0;
			break;
		}
		
		msleep(100);
	}
	
	chip->charger_voltage = vchg_mv;
	
    if (charger_status_pre == CHARGER_STATUS_GOOD){
		if (chip->mBadChargerCounter >= BAD_STATE_COUNT){
			pr_info("stop charing for charger uovp vchg=%dmv\n",vchg_mv);
			pm8921_charger_enable(false);
			
			__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
			
			if(vchg_mv > CHARGER_SOFT_OVP_VOLTAGE){
			    pm8921_charger_status_set(chip, CHARGER_STATUS_OVER);
			}else{
				pm8921_charger_status_set(chip, CHARGER_STATUS_WEAK);
			}
		}	
	}else if (charger_status_pre == CHARGER_STATUS_OVER ||
	          charger_status_pre == CHARGER_STATUS_WEAK ){     
		/*bad charger restore 3times*/
	    if(chip->bad_charger_check_time){
		    pr_info("restore charing for charger ovp=%d,=%d,=%dmv\n",
						chip->bad_charger_check_time,chip->mBadChargerCounter,vchg_mv);
			
			if (chip->mBadChargerCounter < BAD_STATE_COUNT){
				pm8921_charger_enable(true);
				pm8921_charger_status_set(chip, CHARGER_STATUS_GOOD);
			    chip->bad_charger_check_time--;
			}
	    }
	}
	
	return;
}

/* sw charge teoc handle */
/*safe timeout charge SDP10 hours/DCP 6hours,pmic charge safetime 480minutes */
#define TEOC_CHECK_PERIOD_MS_DCP	1000*3600*6
#define TEOC_CHECK_PERIOD_MS_SDP	1000*3600*10
static int teoc_work_schedule(bool on,struct pm8921_chg_chip *chip)
{
   if(on){
   	    if(chip->pm8921_chg_type == USB_DCP_CHARGER )
   			schedule_delayed_work(&chip->teoc_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (TEOC_CHECK_PERIOD_MS_DCP)));
		else if(chip->pm8921_chg_type == USB_SDP_CHARGER ||
			chip->pm8921_chg_type == USB_NON_DCP_CHARGER)
			schedule_delayed_work(&chip->teoc_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (TEOC_CHECK_PERIOD_MS_SDP)));
   }else{
   	   cancel_delayed_work_sync(&chip->teoc_work);
   }

   return 0;
   	
}
/**
** XXX: charge time out ,
*/
static void teoc_worker(struct work_struct *work)
{
   struct delayed_work *dwork = to_delayed_work(work);
   struct pm8921_chg_chip *chip = container_of(dwork,
			   struct pm8921_chg_chip, teoc_work);

   pr_err("safety timer work expired,stop charge!!\n");
   pm_chg_auto_enable(chip, 0);
   chip->safe_charge_teoc = true;
   return;
}

static void usbin_charger_valid(struct work_struct *work)
{
	enum usb_chg_type chg_type;
    if(!the_chip){
		pr_err("the chip not inited!\n");
	    return;
    }
	
	chg_type = the_chip->pm8921_chg_type;
	
	if(chg_type == USB_HDMI_CHARGER){
		/*mhl charger */
		pm8921_set_charge_mhl(the_chip);
	}
	
	the_chip->mChargerCheckCounter = MAX_CHARGER_CHECK_COUNT;

    /*disable charging first to  read charger voltage accurate*/
	pm8921_charger_enable(false);
	mdelay(20);

	/* check battery is remove when charger plugin */
	pm8921_check_battery_connect(the_chip);
	
	/* check is charger OVP and UVP only when charger plugin */
	pm8921_check_charger_uovp(the_chip,true);
	
	if (pm8921_charger_status_get(the_chip) == CHARGER_STATUS_WEAK ||
		pm8921_charger_status_get(the_chip) == CHARGER_STATUS_OVER){
		print_pm8921(DEBUG_INFO,"bad charger state will not start charging\n"); 
		pm8921_charger_enable(false);

	}else{
	    /*Set charge input current to max*/
		pm8921_chg_set_input_current_max(the_chip);
		pm8921_chg_start_charging(the_chip);
		/**
             **XXX start teoc work
             */
	    teoc_work_schedule(true,the_chip);
	}


}
#endif
/* OPPO 2012-08-07 chendx Add end */

/* OPPO 2013-01-18 chendx Add begin for charge eoc with not begin with fastchged */
#define VDDMAX_NORMAL__TEMP 4350 
#define VDDMAX_COOL__TEMP 4200 
#define VDDMAX_WARM__TEMP 4100 
#define VDDMAX_LITTLE_COLD__TEMP 4000 
#define EOC_FINISHED_COUNTER 10
#define EOC_FINISHED_WITH_VBATT_DELTA 100
/*2 hours CV CHG LONG*/
#define CV_LONG_COUNTER 10*60*2 
static int eoc_chg_done_hanlde(struct pm8921_chg_chip *chip)
{

    if(!chip->charge_is_finished || chip->bms_notify.is_charging){
	    pr_info("End of charging!\n");
		pm_chg_auto_enable(chip, 0);
		chip->charge_is_finished = true;
		/* OPPO 2013-02-17 chendx Add begin for warm charge not full */
		#ifdef CONFIG_VENDOR_EDIT
		/* adjust vdd_max only in normal temperature zone */
		if (Pm8921_battery_temp_region_get(the_chip) == CV_BATTERY_TEMP_REGION_LITTLE__COLD ||
			Pm8921_battery_temp_region_get(the_chip) == CV_BATTERY_TEMP_REGION__WARM){
			pr_info("not charge full chg standard!!\n");
			chip->bms_notify.is_battery_full = 0;
		}else{
			chip->bms_notify.is_battery_full = 1;
		}
		#endif
		/* OPPO 2013-02-17 chendx Add end */
		
		/**
		 **XXX stop teoc work
		 */
		teoc_work_schedule(false,the_chip);
		
		/*Add begin for recharging monitor for normal charge */
		if(chip->low_charge_mode == false && chip->high_charge_mode == false){
		
			schedule_delayed_work(&chip->recharge_monitor_work,
					  round_jiffies_relative(msecs_to_jiffies
								 (RECHARGING_MONITOR_MS)));
		}
    }

	return 0;

}

static int eoc_check_with_vbatt(struct pm8921_chg_chip *chip)
{
     int chg_status = 0;
	 chg_cv_battery_temp_region_type temp_region = CV_BATTERY_TEMP_REGION__NORMAL;
	 
	 temp_region = Pm8921_battery_temp_region_get(chip);
     if(temp_region == CV_BATTERY_TEMP_REGION__NORMAL
	 	&& chip->battery_voltage >= (chip->normal_bat_voltage - EOC_FINISHED_WITH_VBATT_DELTA)){
	 	    /*CV CHG more than 2Hours ,force complete*/
	 	    chip->cv_long_counter++;
			if(chip->cv_long_counter >= CV_LONG_COUNTER)
	 			chip->eoc_vbatt_counter++;
			else
				chip->eoc_vbatt_counter = 0;
     }else if(temp_region == CV_BATTERY_TEMP_REGION__COOL
        && chip->battery_voltage >= (chip->cool_bat_voltage - EOC_FINISHED_WITH_VBATT_DELTA)){
		    /*CV CHG more than 2Hours ,force complete*/
	 	    chip->cv_long_counter++;
			if(chip->charge_current >= 0)
				/*discharge current */
				chip->eoc_vbatt_counter++;
			else if(chip->charge_current * -1 < 80)
				chip->eoc_vbatt_counter++;
			else
				chip->eoc_vbatt_counter = 0;
		 	
     }else if(temp_region == CV_BATTERY_TEMP_REGION__WARM
        && chip->battery_voltage >= (chip->warm_bat_voltage - EOC_FINISHED_WITH_VBATT_DELTA)){
			 /*CV CHG more than 2Hours ,force complete*/
	 	     chip->cv_long_counter++;
			 if(chip->charge_current >= 0)
				 /*discharge current */
				 chip->eoc_vbatt_counter++;
			 else if(chip->charge_current * -1 < 80)
				 chip->eoc_vbatt_counter++;
			 else
				 chip->eoc_vbatt_counter = 0;

        
     }else if(temp_region == CV_BATTERY_TEMP_REGION_LITTLE__COLD
        && chip->battery_voltage >= (chip->little_cold_bat_voltage - EOC_FINISHED_WITH_VBATT_DELTA)){
			/*CV CHG more than 2Hours ,force complete*/
	 	    chip->cv_long_counter++;
			
			if(chip->charge_current >= 0)
				/*discharge current */
				chip->eoc_vbatt_counter++;
			else if(chip->charge_current * -1 < 80)
				chip->eoc_vbatt_counter++;
			else
				chip->eoc_vbatt_counter = 0;

     }else{
     	 chip->eoc_vbatt_counter=0;
		 chip->cv_long_counter = 0;
     }


     /*EOC check 1Minutes later*/
     if(chip->eoc_vbatt_counter >= EOC_FINISHED_COUNTER ||
	 	chip->cv_long_counter >= CV_LONG_COUNTER){
	 	chg_status=1;
	    /*charge eoc with vbatt*/
		pr_info("End of Charging vbatt\n");
		chip->eoc_vbatt_counter=0;
		chip->cv_long_counter = 0;
		eoc_chg_done_hanlde(chip);
	 }

	 return chg_status;
	 	 
}
/* OPPO 2013-01-18 chendx Add end */

/**
 * update_heartbeat - internal function to update userspace
 *		per update_time minutes
 *
 */
#define LOW_SOC_HEARTBEAT_MS	20000
static int boot_time = 10; //System boottime 60s
static void update_heartbeat(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, update_heartbeat_work);
/* OPPO 2012-10-19 chendx Add begin for battery health */
#ifdef CONFIG_VENDOR_EDIT
    static int soc_backup = 0;
	char *batt_health[] = {
			"HEALTH_UNKNOWN",
		    "HEALTH_GOOD",
			"HEALTH_OVERHEAT",
			"HEALTH_DEAD",
			"HEALTH_OVERVOLTAGE",			
			"HEALTH_UNSPEC_FAILURE",
		    "HEALTH_COLD"};
#endif
/* OPPO 2012-10-19 chendx Add end */

	pm_chg_failed_clear(chip, 1);
	//power_supply_changed(&chip->batt_psy);

	/* OPPO 2012-08-07 chendx Add begin for reason */
	#ifdef CONFIG_VENDOR_EDIT
	if(boot_time)
		boot_time--;

	if(boot_time == 0)
		/*calibrate battery report Soc,boot_time later ,chip->recent_reported_soc get from BMS system*/
		chip->report_calib_soc = batt_report_capacity_calib(chip,chip->recent_reported_soc);
	
	power_supply_changed(&chip->batt_psy);
	
	if(chip->report_calib_soc != soc_backup){

        /*xxxx:backup report_calib_soc*/
        backup_calib_soc(chip->report_calib_soc);
		
		/*when capacity change dump the charge log*/
		print_pm8921(DEBUG_ERROR,"BATT info####,capacity=%d%%,%dC,Batt voltage=%dmv\n",
					chip->report_calib_soc,chip->battery_temp,chip->battery_voltage);
		print_pm8921(DEBUG_ERROR,"Charge info,Charger voltage=%dmv,chg_current=%dmA,Batt health=%s\n",
					chip->charger_voltage,chip->charge_current,batt_health[chip->batt_health]);
		soc_backup = chip->report_calib_soc ;
	}
	
	/* check is battery connect when charger plugin */
	pm8921_check_battery_connect(chip);
	
	if (is_usb_chg_plugged_in(chip)){
		/*charge eoc with not begin with fastchged*/
		if(boot_time == 0 && (!chip->charge_is_finished || chip->bms_notify.is_charging))
			eoc_check_with_vbatt(chip);
		
		/* check is battery OVP only when charger plugin */
		pm8921_check_battery_uovp(chip);
		
	    /* check is charger OVP and UVP only when charger plugin */
		//if(pm8921_charger_status_get(chip) == CHARGER_STATUS_GOOD)
			pm8921_check_charger_uovp(chip,false);

		if (pm8921_charger_status_get(chip) == CHARGER_STATUS_GOOD &&
				pm8921_battery_status_get(chip) == BATTERY_STATUS_GOOD &&
				get_prop_batt_present(chip)){
		    /*update temp handle when charger status ok,battery status ok,battery present*/
			pm8921_battery_temp_handle(chip);
		}
	}
    /*update time 6s*/
	schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (chip->update_time)));
	#else
	
	power_supply_changed(&chip->batt_psy);
	if (chip->recent_reported_soc <= 20)
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (LOW_SOC_HEARTBEAT_MS)));
	else
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (chip->update_time)));
	#endif
	/* OPPO 2012-08-07 chendx Add end */
}
#define VDD_LOOP_ACTIVE_BIT	BIT(3)
#define VDD_MAX_INCREASE_MV	400
static int vdd_max_increase_mv = VDD_MAX_INCREASE_MV;
module_param(vdd_max_increase_mv, int, 0644);

static int ichg_threshold_ua = -400000;
module_param(ichg_threshold_ua, int, 0644);

#define PM8921_CHG_VDDMAX_RES_MV	10
static void adjust_vdd_max_for_fastchg(struct pm8921_chg_chip *chip,
						int vbat_batt_terminal_uv)
{
	int adj_vdd_max_mv, programmed_vdd_max;
	int vbat_batt_terminal_mv;
	int reg_loop;
	int delta_mv = 0;

	if (chip->rconn_mohm == 0) {
		pr_debug("Exiting as rconn_mohm is 0\n");
		return;
	}
	/* adjust vdd_max only in normal temperature zone */
	if (chip->is_bat_cool || chip->is_bat_warm) {
		pr_debug("Exiting is_bat_cool = %d is_batt_warm = %d\n",
				chip->is_bat_cool, chip->is_bat_warm);
		return;
	}

/* OPPO 2013-02-17 chendx Add begin for reason */
#ifdef CONFIG_VENDOR_EDIT
	/* adjust vdd_max only in normal temperature zone */
	if (Pm8921_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION_LITTLE__COLD ||
		Pm8921_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__WARM){
		pr_info("Exiting is_bat_cool,is_batt_warm\n");
		return;
	}
#endif
/* OPPO 2013-02-17 chendx Add end */
	reg_loop = pm_chg_get_regulation_loop(chip);
	if (!(reg_loop & VDD_LOOP_ACTIVE_BIT)) {
		pr_debug("Exiting Vdd loop is not active reg loop=0x%x\n",
			reg_loop);
		return;
	}
	vbat_batt_terminal_mv = vbat_batt_terminal_uv/1000;
	pm_chg_vddmax_get(the_chip, &programmed_vdd_max);

	delta_mv =  chip->max_voltage_mv - vbat_batt_terminal_mv;

	adj_vdd_max_mv = programmed_vdd_max + delta_mv;
	pr_debug("vdd_max needs to be changed by %d mv from %d to %d\n",
			delta_mv,
			programmed_vdd_max,
			adj_vdd_max_mv);

	if (adj_vdd_max_mv < chip->max_voltage_mv) {
		pr_debug("adj vdd_max lower than default max voltage\n");
		return;
	}

	adj_vdd_max_mv = DIV_ROUND_UP(adj_vdd_max_mv, PM8921_CHG_VDDMAX_RES_MV)
					* PM8921_CHG_VDDMAX_RES_MV;

	if (adj_vdd_max_mv > (chip->max_voltage_mv + vdd_max_increase_mv))
		adj_vdd_max_mv = chip->max_voltage_mv + vdd_max_increase_mv;
	pr_debug("adjusting vdd_max_mv to %d to make "
		"vbat_batt_termial_uv = %d to %d\n",
		adj_vdd_max_mv, vbat_batt_terminal_uv, chip->max_voltage_mv);
	pm_chg_vddmax_set(chip, adj_vdd_max_mv);
}

static void set_appropriate_vbatdet(struct pm8921_chg_chip *chip)
{
	if (chip->is_bat_cool)
		pm_chg_vbatdet_set(the_chip,
			the_chip->cool_bat_voltage
			- the_chip->resume_voltage_delta);
	else if (chip->is_bat_warm)
		pm_chg_vbatdet_set(the_chip,
			the_chip->warm_bat_voltage
			- the_chip->resume_voltage_delta);
	else
		pm_chg_vbatdet_set(the_chip,
			the_chip->max_voltage_mv
			- the_chip->resume_voltage_delta);
}

static void set_appropriate_battery_current(struct pm8921_chg_chip *chip)
{
	unsigned int chg_current = chip->max_bat_chg_current;

	if (chip->is_bat_cool)
		chg_current = min(chg_current, chip->cool_bat_chg_current);

	if (chip->is_bat_warm)
		chg_current = min(chg_current, chip->warm_bat_chg_current);

	if (thermal_mitigation != 0 && chip->thermal_mitigation)
		chg_current = min(chg_current,
				chip->thermal_mitigation[thermal_mitigation]);

	pm_chg_ibatmax_set(the_chip, chg_current);
}

#define TEMP_HYSTERISIS_DECIDEGC 20
static void battery_cool(bool enter)
{
	pr_debug("enter = %d\n", enter);
	if (enter == the_chip->is_bat_cool)
		return;
	the_chip->is_bat_cool = enter;
	if (enter)
		pm_chg_vddmax_set(the_chip, the_chip->cool_bat_voltage);
	else
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);
	set_appropriate_battery_current(the_chip);
	set_appropriate_vbatdet(the_chip);
}

static void battery_warm(bool enter)
{
	pr_debug("enter = %d\n", enter);
	if (enter == the_chip->is_bat_warm)
		return;
	the_chip->is_bat_warm = enter;
	if (enter)
		pm_chg_vddmax_set(the_chip, the_chip->warm_bat_voltage);
	else
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);

	set_appropriate_battery_current(the_chip);
	set_appropriate_vbatdet(the_chip);
}

static void check_temp_thresholds(struct pm8921_chg_chip *chip)
{
	int temp = 0, rc;

/* OPPO 2013-04-09 chendx Add begin for delete qualcomm temp check thresholds*/
	return;
/* OPPO 2013-04-09 chendx Add end */
	rc = get_prop_batt_temp(chip, &temp);
	pr_debug("temp = %d, warm_thr_temp = %d, cool_thr_temp = %d\n",
			temp, chip->warm_temp_dc,
			chip->cool_temp_dc);

	if (chip->warm_temp_dc != INT_MIN) {
		if (chip->is_bat_warm
			&& temp < chip->warm_temp_dc - TEMP_HYSTERISIS_DECIDEGC)
			battery_warm(false);
		else if (!chip->is_bat_warm && temp >= chip->warm_temp_dc)
			battery_warm(true);
	}

	if (chip->cool_temp_dc != INT_MIN) {
		if (chip->is_bat_cool
			&& temp > chip->cool_temp_dc + TEMP_HYSTERISIS_DECIDEGC)
			battery_cool(false);
		else if (!chip->is_bat_cool && temp <= chip->cool_temp_dc)
			battery_cool(true);
	}
}

enum {
	CHG_IN_PROGRESS,
	CHG_NOT_IN_PROGRESS,
	CHG_FINISHED,
};

#define VBAT_TOLERANCE_MV	70
#define CHG_DISABLE_MSLEEP	100
static int is_charging_finished(struct pm8921_chg_chip *chip,
			int vbat_batt_terminal_uv, int ichg_meas_ma)
{
	int vbat_programmed, iterm_programmed, vbat_intended;
	int regulation_loop, fast_chg, vcp;
	int rc;
	static int last_vbat_programmed = -EINVAL;

	if (!is_ext_charging(chip)) {
		/* return if the battery is not being fastcharged */
		fast_chg = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
		pr_debug("fast_chg = %d\n", fast_chg);
		if (fast_chg == 0)
			return CHG_NOT_IN_PROGRESS;

		vcp = pm_chg_get_rt_status(chip, VCP_IRQ);
		pr_debug("vcp = %d\n", vcp);
		if (vcp == 1)
			return CHG_IN_PROGRESS;

		/* reset count if battery is hot/cold */
		rc = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);
		pr_debug("batt_temp_ok = %d\n", rc);
		if (rc == 0)
			return CHG_IN_PROGRESS;

		rc = pm_chg_vddmax_get(chip, &vbat_programmed);
		if (rc) {
			pr_err("couldnt read vddmax rc = %d\n", rc);
			return CHG_IN_PROGRESS;
		}
		pr_debug("vddmax = %d vbat_batt_terminal_uv=%d\n",
			 vbat_programmed, vbat_batt_terminal_uv);

		if (last_vbat_programmed == -EINVAL)
			last_vbat_programmed = vbat_programmed;
		if (last_vbat_programmed !=  vbat_programmed) {
			/* vddmax changed, reset and check again */
			pr_debug("vddmax = %d last_vdd_max=%d\n",
				 vbat_programmed, last_vbat_programmed);
			last_vbat_programmed = vbat_programmed;
			return CHG_IN_PROGRESS;
		}

		if (chip->is_bat_cool)
			vbat_intended = chip->cool_bat_voltage;
		else if (chip->is_bat_warm)
			vbat_intended = chip->warm_bat_voltage;
		else
			vbat_intended = chip->max_voltage_mv;

		if (vbat_batt_terminal_uv / 1000 < vbat_intended) {
			pr_debug("terminal_uv:%d < vbat_intended:%d.\n",
							vbat_batt_terminal_uv,
							vbat_intended);
			return CHG_IN_PROGRESS;
		}

		regulation_loop = pm_chg_get_regulation_loop(chip);
		if (regulation_loop < 0) {
			pr_err("couldnt read the regulation loop err=%d\n",
				regulation_loop);
			return CHG_IN_PROGRESS;
		}
		pr_debug("regulation_loop=%d\n", regulation_loop);

		if (regulation_loop != 0 && regulation_loop != VDD_LOOP)
			return CHG_IN_PROGRESS;
	} /* !is_ext_charging */

	/* reset count if battery chg current is more than iterm */
	rc = pm_chg_iterm_get(chip, &iterm_programmed);
	if (rc) {
		pr_err("couldnt read iterm rc = %d\n", rc);
		return CHG_IN_PROGRESS;
	}

	pr_debug("iterm_programmed = %d ichg_meas_ma=%d\n",
				iterm_programmed, ichg_meas_ma);
	/*
	 * ichg_meas_ma < 0 means battery is drawing current
	 * ichg_meas_ma > 0 means battery is providing current
	 */
	if (ichg_meas_ma > 0)
		return CHG_IN_PROGRESS;

	if (ichg_meas_ma * -1 > iterm_programmed)
		return CHG_IN_PROGRESS;

	return CHG_FINISHED;
}

#define COMP_OVERRIDE_HOT_BANK	6
#define COMP_OVERRIDE_COLD_BANK	7
#define COMP_OVERRIDE_BIT  BIT(1)
static int pm_chg_override_cold(struct pm8921_chg_chip *chip, int flag)
{
	u8 val;
	int rc = 0;

	val = 0x80 | COMP_OVERRIDE_COLD_BANK << 2 | COMP_OVERRIDE_BIT;

	if (flag)
		val |= 0x01;

	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0)
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);

	pr_debug("btc cold = %d val = 0x%x\n", flag, val);
	return rc;
}

static int pm_chg_override_hot(struct pm8921_chg_chip *chip, int flag)
{
	u8 val;
	int rc = 0;

	val = 0x80 | COMP_OVERRIDE_HOT_BANK << 2 | COMP_OVERRIDE_BIT;

	if (flag)
		val |= 0x01;

	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0)
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);

	pr_debug("btc hot = %d val = 0x%x\n", flag, val);
	return rc;
}

static void __devinit pm8921_chg_btc_override_init(struct pm8921_chg_chip *chip)
{
	int rc = 0;
	u8 reg;
	u8 val;

	val = COMP_OVERRIDE_HOT_BANK << 2;
	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0) {
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);
		goto cold_init;
	}
	rc = pm8xxx_readb(chip->dev->parent, COMPARATOR_OVERRIDE, &reg);
	if (rc < 0) {
		pr_err("Could not read bank %d of override rc = %d\n",
				COMP_OVERRIDE_HOT_BANK, rc);
		goto cold_init;
	}
	if ((reg & COMP_OVERRIDE_BIT) != COMP_OVERRIDE_BIT) {
		/* for now override it as not hot */
		rc = pm_chg_override_hot(chip, 0);
		if (rc < 0)
			pr_err("Could not override hot rc = %d\n", rc);
	}

cold_init:
	val = COMP_OVERRIDE_COLD_BANK << 2;
	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0) {
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, COMPARATOR_OVERRIDE, &reg);
	if (rc < 0) {
		pr_err("Could not read bank %d of override rc = %d\n",
				COMP_OVERRIDE_COLD_BANK, rc);
		return;
	}
	if ((reg & COMP_OVERRIDE_BIT) != COMP_OVERRIDE_BIT) {
		/* for now override it as not cold */
		rc = pm_chg_override_cold(chip, 0);
		if (rc < 0)
			pr_err("Could not override cold rc = %d\n", rc);
	}
}

static void btc_override_worker(struct work_struct *work)
{
	int decidegc;
	int temp;
	int rc = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, btc_override_work);

	if (!chip->btc_override) {
		pr_err("called when not enabled\n");
		return;
	}

	rc = get_prop_batt_temp(chip, &decidegc);
	if (rc) {
		pr_info("Failed to read temperature\n");
		goto fail_btc_temp;
	}

	pr_debug("temp=%d\n", decidegc);

	temp = pm_chg_get_rt_status(chip, BATTTEMP_HOT_IRQ);
	if (temp) {
		if (decidegc < chip->btc_override_hot_decidegc)
			/* stop forcing batt hot */
			rc = pm_chg_override_hot(chip, 0);
			if (rc)
				pr_err("Couldnt write 0 to hot comp\n");
	} else {
		if (decidegc >= chip->btc_override_hot_decidegc)
			/* start forcing batt hot */
			rc = pm_chg_override_hot(chip, 1);
			if (rc && chip->btc_panic_if_cant_stop_chg)
				panic("Couldnt override comps to stop chg\n");
	}

	temp = pm_chg_get_rt_status(chip, BATTTEMP_COLD_IRQ);
	if (temp) {
		if (decidegc > chip->btc_override_cold_decidegc)
			/* stop forcing batt cold */
			rc = pm_chg_override_cold(chip, 0);
			if (rc)
				pr_err("Couldnt write 0 to cold comp\n");
	} else {
		if (decidegc <= chip->btc_override_cold_decidegc)
			/* start forcing batt cold */
			rc = pm_chg_override_cold(chip, 1);
			if (rc && chip->btc_panic_if_cant_stop_chg)
				panic("Couldnt override comps to stop chg\n");
	}

	if ((is_dc_chg_plugged_in(the_chip) || is_usb_chg_plugged_in(the_chip))
		&& get_prop_batt_status(chip) != POWER_SUPPLY_STATUS_FULL) {
		schedule_delayed_work(&chip->btc_override_work,
					round_jiffies_relative(msecs_to_jiffies
						(chip->btc_delay_ms)));
		return;
	}

fail_btc_temp:
	rc = pm_chg_override_hot(chip, 0);
	if (rc)
		pr_err("Couldnt write 0 to hot comp\n");
	rc = pm_chg_override_cold(chip, 0);
	if (rc)
		pr_err("Couldnt write 0 to cold comp\n");
}

/**
 * eoc_worker - internal function to check if battery EOC
 *		has happened
 *
 * If all conditions favouring, if the charge current is
 * less than the term current for three consecutive times
 * an EOC has happened.
 * The wakelock is released if there is no need to reshedule
 * - this happens when the battery is removed or EOC has
 * happened
 */
#define CONSECUTIVE_COUNT	3
static void eoc_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, eoc_work);
	static int count;
	int end;
	int vbat_meas_uv, vbat_meas_mv;
	int ichg_meas_ua, ichg_meas_ma;
	int vbat_batt_terminal_uv;

	pm8921_bms_get_simultaneous_battery_voltage_and_current(
					&ichg_meas_ua,	&vbat_meas_uv);
	vbat_meas_mv = vbat_meas_uv / 1000;
	/* rconn_mohm is in milliOhms */
	ichg_meas_ma = ichg_meas_ua / 1000;
	vbat_batt_terminal_uv = vbat_meas_uv
					+ ichg_meas_ma
					* the_chip->rconn_mohm;

	end = is_charging_finished(chip, vbat_batt_terminal_uv, ichg_meas_ma);

	if (end == CHG_NOT_IN_PROGRESS) {
		count = 0;
		goto eoc_worker_stop;
	}

	if (end == CHG_FINISHED) {
		count++;
	} else {
		count = 0;
	}

	/* OPPO 2012-10-19 chendx Add begin for FTM CHARGER MODE */
	#ifdef CONFIG_VENDOR_EDIT
    #ifdef FTM_CHARGE_MODE_FEATURE
	if(chip->low_charge_mode == true && chip->recent_reported_soc >= FTM_LOW_CAPACITY_LEVEL){
		pr_info("%s: Warning charge on FTM mode, charge to 60 finish charge,=%d\n",
				__func__,chip->low_charge_mode);
		count = CONSECUTIVE_COUNT;		
	}else if(chip->high_charge_mode == true && chip->recent_reported_soc >= FTM_HIGH_CAPACITY_LEVEL){
	    pr_info("%s: Warning charge on FTM mode, charge to 80 finish charge,=%d\n",
				__func__,chip->high_charge_mode);
		count = CONSECUTIVE_COUNT;
	}
	#endif
	#endif
	/* OPPO 2012-10-19 chendx Add end */
	if (count == CONSECUTIVE_COUNT) {
		count = 0;
		pr_info("End of Charging\n");

		pm_chg_auto_enable(chip, 0);

		if (is_ext_charging(chip))
			chip->ext_charge_done = true;

/* OPPO 2012-11-28 chendx Add begin for charge is finished */
#ifdef CONFIG_VENDOR_EDIT
		chip->charge_is_finished = true;

		/**
		 **XXX stop teoc work
		 */
		teoc_work_schedule(false,the_chip);

/*Add begin for recharging monitor for normal charge */
	if(chip->low_charge_mode == false && chip->high_charge_mode == false){

	    schedule_delayed_work(&chip->recharge_monitor_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (RECHARGING_MONITOR_MS)));
	}
#endif
/* OPPO 2012-08-22 chendx Add end */

		/* OPPO 2013-02-17 chendx Add begin for warm charge not full */
		#ifdef CONFIG_VENDOR_EDIT
		/* adjust vdd_max only in normal temperature zone */
		if (Pm8921_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION_LITTLE__COLD ||
			Pm8921_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__WARM){
			pr_info("not charge full chg standard!!\n");
			chip->bms_notify.is_battery_full = 0;
		}else{
			chip->bms_notify.is_battery_full = 1;
		}
		#else
		if (chip->is_bat_warm || chip->is_bat_cool)
			chip->bms_notify.is_battery_full = 0;
		else
			chip->bms_notify.is_battery_full = 1;
		#endif
		/* OPPO 2013-02-17 chendx Add end */
		/* declare end of charging by invoking chgdone interrupt */
		chgdone_irq_handler(chip->pmic_chg_irq[CHGDONE_IRQ], chip);
	} else {
		check_temp_thresholds(chip);
		adjust_vdd_max_for_fastchg(chip, vbat_batt_terminal_uv);
		pr_debug("EOC count = %d\n", count);
		schedule_delayed_work(&chip->eoc_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
		return;
	}

eoc_worker_stop:
	wake_unlock(&chip->eoc_wake_lock);
	/* set the vbatdet back, in case it was changed to trigger charging */
	set_appropriate_vbatdet(chip);
}

/**
 * set_disable_status_param -
 *
 * Internal function to disable battery charging and also disable drawing
 * any current from the source. The device is forced to run on a battery
 * after this.
 */
static int set_disable_status_param(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	pr_info("factory set disable param to %d\n", charging_disabled);
	if (chip) {
		pm_chg_auto_enable(chip, !charging_disabled);
		pm_chg_charge_dis(chip, charging_disabled);
	}
	return 0;
}
module_param_call(disabled, set_disable_status_param, param_get_uint,
					&charging_disabled, 0644);

static int rconn_mohm;
static int set_rconn_mohm(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip)
		chip->rconn_mohm = rconn_mohm;
	return 0;
}
module_param_call(rconn_mohm, set_rconn_mohm, param_get_uint,
					&rconn_mohm, 0644);
/**
 * set_thermal_mitigation_level -
 *
 * Internal function to control battery charging current to reduce
 * temperature
 */
static int set_therm_mitigation_level(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (!chip->thermal_mitigation) {
		pr_err("no thermal mitigation\n");
		return -EINVAL;
	}

	if (thermal_mitigation < 0
		|| thermal_mitigation >= chip->thermal_levels) {
		pr_err("out of bound level selected\n");
		return -EINVAL;
	}

	set_appropriate_battery_current(chip);
	return ret;
}
module_param_call(thermal_mitigation, set_therm_mitigation_level,
					param_get_uint,
					&thermal_mitigation, 0644);

static int set_usb_max_current(const char *val, struct kernel_param *kp)
{
	int ret, mA;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip) {
		pr_warn("setting current max to %d\n", usb_max_current);
		pm_chg_iusbmax_get(chip, &mA);
		if (mA > usb_max_current)
			pm8921_charger_vbus_draw(usb_max_current);
		return 0;
	}
	return -EINVAL;
}
module_param_call(usb_max_current, set_usb_max_current,
	param_get_uint, &usb_max_current, 0644);

static void free_irqs(struct pm8921_chg_chip *chip)
{
	int i;

	for (i = 0; i < PM_CHG_MAX_INTS; i++)
		if (chip->pmic_chg_irq[i]) {
			free_irq(chip->pmic_chg_irq[i], chip);
			chip->pmic_chg_irq[i] = 0;
		}
}

/* determines the initial present states */
static void __devinit determine_initial_state(struct pm8921_chg_chip *chip)
{
	int fsm_state;
	int is_fast_chg;

	chip->dc_present = !!is_dc_chg_plugged_in(chip);
	chip->usb_present = !!is_usb_chg_plugged_in(chip);

	notify_usb_of_the_plugin_event(chip->usb_present);
	if (chip->usb_present || chip->dc_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_WAIT_PERIOD_MS));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	}

	pm8921_chg_enable_irq(chip, DCIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, USBIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, BATT_REMOVED_IRQ);
	pm8921_chg_enable_irq(chip, BATT_INSERTED_IRQ);
	pm8921_chg_enable_irq(chip, DCIN_OV_IRQ);
	pm8921_chg_enable_irq(chip, DCIN_UV_IRQ);
	pm8921_chg_enable_irq(chip, CHGFAIL_IRQ);
	pm8921_chg_enable_irq(chip, FASTCHG_IRQ);
	pm8921_chg_enable_irq(chip, VBATDET_LOW_IRQ);
	pm8921_chg_enable_irq(chip, BAT_TEMP_OK_IRQ);

	if (get_prop_batt_present(the_chip) || is_dc_chg_plugged_in(the_chip))
		if (usb_chg_current)
			/*
			 * Reissue a vbus draw call only if a battery
			 * or DC is present. We don't want to brown out the
			 * device if usb is its only source
			 */
			__pm8921_charger_vbus_draw(usb_chg_current);
	usb_chg_current = 0;

	/*
	 * The bootloader could have started charging, a fastchg interrupt
	 * might not happen. Check the real time status and if it is fast
	 * charging invoke the handler so that the eoc worker could be
	 * started
	 */
	is_fast_chg = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (is_fast_chg)
		fastchg_irq_handler(chip->pmic_chg_irq[FASTCHG_IRQ], chip);

	fsm_state = pm_chg_get_fsm_state(chip);
	if (is_battery_charging(fsm_state)) {
		chip->bms_notify.is_charging = 1;
		pm8921_bms_charging_began();
	}

	check_battery_valid(chip);

	pr_debug("usb = %d, dc = %d  batt = %d state=%d\n",
			chip->usb_present,
			chip->dc_present,
			get_prop_batt_present(chip),
			fsm_state);

	/* Determine which USB trim column to use */
	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917)
		chip->usb_trim_table = usb_trim_8917_table;
	else if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8038)
		chip->usb_trim_table = usb_trim_8038_table;
}

struct pm_chg_irq_init_data {
	unsigned int	irq_id;
	char		*name;
	unsigned long	flags;
	irqreturn_t	(*handler)(int, void *);
};

#define CHG_IRQ(_id, _flags, _handler) \
{ \
	.irq_id		= _id, \
	.name		= #_id, \
	.flags		= _flags, \
	.handler	= _handler, \
}
struct pm_chg_irq_init_data chg_irq_data[] = {
	CHG_IRQ(USBIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						usbin_valid_irq_handler),
	CHG_IRQ(BATT_INSERTED_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batt_inserted_irq_handler),
	CHG_IRQ(VBATDET_LOW_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						vbatdet_low_irq_handler),
	CHG_IRQ(CHGWDOG_IRQ, IRQF_TRIGGER_RISING, chgwdog_irq_handler),
	CHG_IRQ(VCP_IRQ, IRQF_TRIGGER_RISING, vcp_irq_handler),
	CHG_IRQ(ATCDONE_IRQ, IRQF_TRIGGER_RISING, atcdone_irq_handler),
	CHG_IRQ(ATCFAIL_IRQ, IRQF_TRIGGER_RISING, atcfail_irq_handler),
	CHG_IRQ(CHGDONE_IRQ, IRQF_TRIGGER_RISING, chgdone_irq_handler),
	CHG_IRQ(CHGFAIL_IRQ, IRQF_TRIGGER_RISING, chgfail_irq_handler),
	CHG_IRQ(CHGSTATE_IRQ, IRQF_TRIGGER_RISING, chgstate_irq_handler),
	CHG_IRQ(LOOP_CHANGE_IRQ, IRQF_TRIGGER_RISING, loop_change_irq_handler),
	CHG_IRQ(FASTCHG_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						fastchg_irq_handler),
	CHG_IRQ(TRKLCHG_IRQ, IRQF_TRIGGER_RISING, trklchg_irq_handler),
	CHG_IRQ(BATT_REMOVED_IRQ, IRQF_TRIGGER_RISING,
						batt_removed_irq_handler),
	CHG_IRQ(BATTTEMP_HOT_IRQ, IRQF_TRIGGER_RISING,
						batttemp_hot_irq_handler),
	CHG_IRQ(CHGHOT_IRQ, IRQF_TRIGGER_RISING, chghot_irq_handler),
	CHG_IRQ(BATTTEMP_COLD_IRQ, IRQF_TRIGGER_RISING,
						batttemp_cold_irq_handler),
	CHG_IRQ(CHG_GONE_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						chg_gone_irq_handler),
	CHG_IRQ(BAT_TEMP_OK_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						bat_temp_ok_irq_handler),
	CHG_IRQ(COARSE_DET_LOW_IRQ, IRQF_TRIGGER_RISING,
						coarse_det_low_irq_handler),
	CHG_IRQ(VDD_LOOP_IRQ, IRQF_TRIGGER_RISING, vdd_loop_irq_handler),
	CHG_IRQ(VREG_OV_IRQ, IRQF_TRIGGER_RISING, vreg_ov_irq_handler),
	CHG_IRQ(VBATDET_IRQ, IRQF_TRIGGER_RISING, vbatdet_irq_handler),
	CHG_IRQ(BATFET_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batfet_irq_handler),
	CHG_IRQ(DCIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_valid_irq_handler),
	CHG_IRQ(DCIN_OV_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_ov_irq_handler),
	CHG_IRQ(DCIN_UV_IRQ, IRQF_TRIGGER_RISING, dcin_uv_irq_handler),
};

static int __devinit request_irqs(struct pm8921_chg_chip *chip,
					struct platform_device *pdev)
{
	struct resource *res;
	int ret, i;

	ret = 0;
	bitmap_fill(chip->enabled_irqs, PM_CHG_MAX_INTS);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
				chg_irq_data[i].name);
		if (res == NULL) {
			pr_err("couldn't find %s\n", chg_irq_data[i].name);
			goto err_out;
		}
		chip->pmic_chg_irq[chg_irq_data[i].irq_id] = res->start;
		ret = request_irq(res->start, chg_irq_data[i].handler,
			chg_irq_data[i].flags,
			chg_irq_data[i].name, chip);
		if (ret < 0) {
			pr_err("couldn't request %d (%s) %d\n", res->start,
					chg_irq_data[i].name, ret);
			chip->pmic_chg_irq[chg_irq_data[i].irq_id] = 0;
			goto err_out;
		}
		pm8921_chg_disable_irq(chip, chg_irq_data[i].irq_id);
	}
	return 0;

err_out:
	free_irqs(chip);
	return -EINVAL;
}

static void pm8921_chg_force_19p2mhz_clk(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;

	temp  = 0xD1;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD1;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD5;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	udelay(183);

	temp  = 0xD1;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD0;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
	udelay(32);

	temp  = 0xD1;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}

static void pm8921_chg_set_hw_clk_switching(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;

	temp  = 0xD1;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD0;
	err = pm_chg_write(chip, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}

#define VREF_BATT_THERM_FORCE_ON	BIT(7)
static void detect_battery_removal(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	pr_debug("upon restart CHG_CNTRL = 0x%x\n",  temp);

	if (!(temp & VREF_BATT_THERM_FORCE_ON))
		/*
		 * batt therm force on bit is battery backed and is default 0
		 * The charger sets this bit at init time. If this bit is found
		 * 0 that means the battery was removed. Tell the bms about it
		 */
		pm8921_bms_invalidate_shutdown_soc();
}

#define ENUM_TIMER_STOP_BIT	BIT(1)
#define BOOT_DONE_BIT		BIT(6)
#define CHG_BATFET_ON_BIT	BIT(3)
#define CHG_VCP_EN		BIT(0)
#define CHG_BAT_TEMP_DIS_BIT	BIT(2)
#define SAFE_CURRENT_MA		1500
#define PM_SUB_REV		0x001
#define MIN_CHARGE_CURRENT_MA	350
#define DEFAULT_SAFETY_MINUTES	500
static int __devinit pm8921_chg_hw_init(struct pm8921_chg_chip *chip)
{
	u8 subrev;
	int rc, vdd_safe, fcc_uah, safety_time = DEFAULT_SAFETY_MINUTES;

	/* forcing 19p2mhz before accessing any charger registers */
	pm8921_chg_force_19p2mhz_clk(chip);

	detect_battery_removal(chip);

	rc = pm_chg_masked_write(chip, SYS_CONFIG_2,
					BOOT_DONE_BIT, BOOT_DONE_BIT);
	if (rc) {
		pr_err("Failed to set BOOT_DONE_BIT rc=%d\n", rc);
		return rc;
	}

	vdd_safe = chip->max_voltage_mv + VDD_MAX_INCREASE_MV;

	if (vdd_safe > PM8921_CHG_VDDSAFE_MAX)
		vdd_safe = PM8921_CHG_VDDSAFE_MAX;

	rc = pm_chg_vddsafe_set(chip, vdd_safe);

	if (rc) {
		pr_err("Failed to set safe voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}
	rc = pm_chg_vbatdet_set(chip,
				chip->max_voltage_mv
				- chip->resume_voltage_delta);
	if (rc) {
		pr_err("Failed to set vbatdet comprator voltage to %d rc=%d\n",
			chip->max_voltage_mv - chip->resume_voltage_delta, rc);
		return rc;
	}

	rc = pm_chg_vddmax_set(chip, chip->max_voltage_mv);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}

	if (chip->safe_current_ma == 0)
		chip->safe_current_ma = SAFE_CURRENT_MA;

	rc = pm_chg_ibatsafe_set(chip, chip->safe_current_ma);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						SAFE_CURRENT_MA, rc);
		return rc;
	}

	rc = pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);
	if (rc) {
		pr_err("Failed to set max current to 400 rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_iterm_set(chip, chip->term_current);
	if (rc) {
		pr_err("Failed to set term current to %d rc=%d\n",
						chip->term_current, rc);
		return rc;
	}

	/* Disable the ENUM TIMER */
	rc = pm_chg_masked_write(chip, PBL_ACCESS2, ENUM_TIMER_STOP_BIT,
			ENUM_TIMER_STOP_BIT);
	if (rc) {
		pr_err("Failed to set enum timer stop rc=%d\n", rc);
		return rc;
	}

	fcc_uah = pm8921_bms_get_fcc();
	if (fcc_uah > 0) {
		safety_time = div_s64((s64)fcc_uah * 60,
						1000 * MIN_CHARGE_CURRENT_MA);
		/* add 20 minutes of buffer time */
		safety_time += 20;

		/* make sure we do not exceed the maximum programmable time */
		if (safety_time > PM8921_CHG_TCHG_MAX)
			safety_time = PM8921_CHG_TCHG_MAX;
	}

	rc = pm_chg_tchg_max_set(chip, safety_time);
	if (rc) {
		pr_err("Failed to set max time to %d minutes rc=%d\n",
						safety_time, rc);
		return rc;
	}

	if (chip->ttrkl_time != 0) {
		rc = pm_chg_ttrkl_max_set(chip, chip->ttrkl_time);
		if (rc) {
			pr_err("Failed to set trkl time to %d minutes rc=%d\n",
							chip->ttrkl_time, rc);
			return rc;
		}
	}

	if (chip->vin_min != 0) {
		rc = pm_chg_vinmin_set(chip, chip->vin_min);
		if (rc) {
			pr_err("Failed to set vin min to %d mV rc=%d\n",
							chip->vin_min, rc);
			return rc;
		}
	} else {
		chip->vin_min = pm_chg_vinmin_get(chip);
	}

	rc = pm_chg_disable_wd(chip);
	if (rc) {
		pr_err("Failed to disable wd rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_masked_write(chip, CHG_CNTRL_2,
				CHG_BAT_TEMP_DIS_BIT, 0);
	if (rc) {
		pr_err("Failed to enable temp control chg rc=%d\n", rc);
		return rc;
	}
	/* switch to a 3.2Mhz for the buck */
	if (pm8xxx_get_revision(chip->dev->parent) >= PM8XXX_REVISION_8038_1p0)
		rc = pm_chg_write(chip,
			CHG_BUCK_CLOCK_CTRL_8038, 0x15);
	else
		rc = pm_chg_write(chip,
			CHG_BUCK_CLOCK_CTRL, 0x15);

	if (rc) {
		pr_err("Failed to switch buck clk rc=%d\n", rc);
		return rc;
	}

	if (chip->trkl_voltage != 0) {
		rc = pm_chg_vtrkl_low_set(chip, chip->trkl_voltage);
		if (rc) {
			pr_err("Failed to set trkl voltage to %dmv  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_voltage != 0) {
		rc = pm_chg_vweak_set(chip, chip->weak_voltage);
		if (rc) {
			pr_err("Failed to set weak voltage to %dmv  rc=%d\n",
							chip->weak_voltage, rc);
			return rc;
		}
	}

	if (chip->trkl_current != 0) {
		rc = pm_chg_itrkl_set(chip, chip->trkl_current);
		if (rc) {
			pr_err("Failed to set trkl current to %dmA  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_current != 0) {
		rc = pm_chg_iweak_set(chip, chip->weak_current);
		if (rc) {
			pr_err("Failed to set weak current to %dmA  rc=%d\n",
							chip->weak_current, rc);
			return rc;
		}
	}

	rc = pm_chg_batt_cold_temp_config(chip, chip->cold_thr);
	if (rc) {
		pr_err("Failed to set cold config %d  rc=%d\n",
						chip->cold_thr, rc);
	}

	rc = pm_chg_batt_hot_temp_config(chip, chip->hot_thr);
	if (rc) {
		pr_err("Failed to set hot config %d  rc=%d\n",
						chip->hot_thr, rc);
	}

	rc = pm_chg_led_src_config(chip, chip->led_src_config);
	if (rc) {
		pr_err("Failed to set charger LED src config %d  rc=%d\n",
						chip->led_src_config, rc);
	}

	/* Workarounds for die 3.0 */
	if (pm8xxx_get_revision(chip->dev->parent) == PM8XXX_REVISION_8921_3p0
	&& pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8921) {
		rc = pm8xxx_readb(chip->dev->parent, PM_SUB_REV, &subrev);
		if (rc) {
			pr_err("read failed: addr=%03X, rc=%d\n",
				PM_SUB_REV, rc);
			return rc;
		}
		/* Check if die 3.0.1 is present */
		if (subrev & 0x1)
			pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0xA4);
		else
			pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0xAC);
	}

	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917) {
		/* Set PM8917 USB_OVP debounce time to 15 ms */
		rc = pm_chg_masked_write(chip, USB_OVP_CONTROL,
			OVP_DEBOUNCE_TIME, 0x6);
		if (rc) {
			pr_err("Failed to set USB OVP db rc=%d\n", rc);
			return rc;
		}

		/* Enable isub_fine resolution AICL for PM8917 */
		chip->iusb_fine_res = true;
		if (chip->uvd_voltage_mv) {
			rc = pm_chg_uvd_threshold_set(chip,
					chip->uvd_voltage_mv);
			if (rc) {
				pr_err("Failed to set UVD threshold %drc=%d\n",
						chip->uvd_voltage_mv, rc);
				return rc;
			}
		}
	}

	pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0xD9);

	/* Disable EOC FSM processing */
	pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0x91);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm rc=%d\n", rc);

	rc = pm_chg_charge_dis(chip, charging_disabled);
	if (rc) {
		pr_err("Failed to disable CHG_CHARGE_DIS bit rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_auto_enable(chip, !charging_disabled);
	if (rc) {
		pr_err("Failed to enable charging rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int get_rt_status(void *data, u64 * val)
{
	int i = (int)data;
	int ret;

	/* global irq number is passed in via data */
	ret = pm_chg_get_rt_status(the_chip, i);
	*val = ret;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rt_fops, get_rt_status, NULL, "%llu\n");

static int get_fsm_status(void *data, u64 * val)
{
	u8 temp;

	temp = pm_chg_get_fsm_state(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fsm_fops, get_fsm_status, NULL, "%llu\n");

static int get_reg_loop(void *data, u64 * val)
{
	u8 temp;

	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	temp = pm_chg_get_regulation_loop(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_loop_fops, get_reg_loop, NULL, "0x%02llx\n");

static int get_reg(void *data, u64 * val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	ret = pm8xxx_readb(the_chip->dev->parent, addr, &temp);
	if (ret) {
		pr_err("pm8xxx_readb to %x value =%d errored = %d\n",
			addr, temp, ret);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	temp = (u8) val;
	ret = pm_chg_write(the_chip, addr, temp);
	if (ret) {
		pr_err("pm_chg_write to %x value =%d errored = %d\n",
			addr, temp, ret);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");

static int reg_loop;
#define MAX_REG_LOOP_CHAR	10
static int get_reg_loop_param(char *buf, struct kernel_param *kp)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	temp = pm_chg_get_regulation_loop(the_chip);
	return snprintf(buf, MAX_REG_LOOP_CHAR, "%d", temp);
}
module_param_call(reg_loop, NULL, get_reg_loop_param,
					&reg_loop, 0644);

static int max_chg_ma;
#define MAX_MA_CHAR	10
static int get_max_chg_ma_param(char *buf, struct kernel_param *kp)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return snprintf(buf, MAX_MA_CHAR, "%d", the_chip->max_bat_chg_current);
}
module_param_call(max_chg_ma, NULL, get_max_chg_ma_param,
					&max_chg_ma, 0644);
static int ibatmax_ma;
static int set_ibat_max(const char *val, struct kernel_param *kp)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("error setting value %d\n", rc);
		return rc;
	}

	if (abs(ibatmax_ma - the_chip->max_bat_chg_current)
				<= the_chip->ibatmax_max_adj_ma) {
		rc = pm_chg_ibatmax_set(the_chip, ibatmax_ma);
		if (rc) {
			pr_err("Failed to set ibatmax rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}
static int get_ibat_max(char *buf, struct kernel_param *kp)
{
	int ibat_ma;
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = pm_chg_ibatmax_get(the_chip, &ibat_ma);
	if (rc) {
		pr_err("ibatmax_get error = %d\n", rc);
		return rc;
	}

	return snprintf(buf, MAX_MA_CHAR, "%d", ibat_ma);
}
module_param_call(ibatmax_ma, set_ibat_max, get_ibat_max,
					&ibatmax_ma, 0644);
enum {
	BAT_WARM_ZONE,
	BAT_COOL_ZONE,
};
static int get_warm_cool(void *data, u64 * val)
{
	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	if ((int)data == BAT_WARM_ZONE)
		*val = the_chip->is_bat_warm;
	if ((int)data == BAT_COOL_ZONE)
		*val = the_chip->is_bat_cool;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(warm_cool_fops, get_warm_cool, NULL, "0x%lld\n");

static void create_debugfs_entries(struct pm8921_chg_chip *chip)
{
	int i;

	chip->dent = debugfs_create_dir("pm8921_chg", NULL);

	if (IS_ERR(chip->dent)) {
		pr_err("pmic charger couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("CHG_CNTRL", 0644, chip->dent,
			    (void *)CHG_CNTRL, &reg_fops);
	debugfs_create_file("CHG_CNTRL_2", 0644, chip->dent,
			    (void *)CHG_CNTRL_2, &reg_fops);
	debugfs_create_file("CHG_CNTRL_3", 0644, chip->dent,
			    (void *)CHG_CNTRL_3, &reg_fops);
	debugfs_create_file("PBL_ACCESS1", 0644, chip->dent,
			    (void *)PBL_ACCESS1, &reg_fops);
	debugfs_create_file("PBL_ACCESS2", 0644, chip->dent,
			    (void *)PBL_ACCESS2, &reg_fops);
	debugfs_create_file("SYS_CONFIG_1", 0644, chip->dent,
			    (void *)SYS_CONFIG_1, &reg_fops);
	debugfs_create_file("SYS_CONFIG_2", 0644, chip->dent,
			    (void *)SYS_CONFIG_2, &reg_fops);
	debugfs_create_file("CHG_VDD_MAX", 0644, chip->dent,
			    (void *)CHG_VDD_MAX, &reg_fops);
	debugfs_create_file("CHG_VDD_SAFE", 0644, chip->dent,
			    (void *)CHG_VDD_SAFE, &reg_fops);
	debugfs_create_file("CHG_VBAT_DET", 0644, chip->dent,
			    (void *)CHG_VBAT_DET, &reg_fops);
	debugfs_create_file("CHG_IBAT_MAX", 0644, chip->dent,
			    (void *)CHG_IBAT_MAX, &reg_fops);
	debugfs_create_file("CHG_IBAT_SAFE", 0644, chip->dent,
			    (void *)CHG_IBAT_SAFE, &reg_fops);
	debugfs_create_file("CHG_VIN_MIN", 0644, chip->dent,
			    (void *)CHG_VIN_MIN, &reg_fops);
	debugfs_create_file("CHG_VTRICKLE", 0644, chip->dent,
			    (void *)CHG_VTRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITRICKLE", 0644, chip->dent,
			    (void *)CHG_ITRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITERM", 0644, chip->dent,
			    (void *)CHG_ITERM, &reg_fops);
	debugfs_create_file("CHG_TCHG_MAX", 0644, chip->dent,
			    (void *)CHG_TCHG_MAX, &reg_fops);
	debugfs_create_file("CHG_TWDOG", 0644, chip->dent,
			    (void *)CHG_TWDOG, &reg_fops);
	debugfs_create_file("CHG_TEMP_THRESH", 0644, chip->dent,
			    (void *)CHG_TEMP_THRESH, &reg_fops);
	debugfs_create_file("CHG_COMP_OVR", 0644, chip->dent,
			    (void *)CHG_COMP_OVR, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST1", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST1, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST2", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST2, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST3", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST3, &reg_fops);
	debugfs_create_file("CHG_TEST", 0644, chip->dent,
			    (void *)CHG_TEST, &reg_fops);

	debugfs_create_file("FSM_STATE", 0644, chip->dent, NULL,
			    &fsm_fops);

	debugfs_create_file("REGULATION_LOOP_CONTROL", 0644, chip->dent, NULL,
			    &reg_loop_fops);

	debugfs_create_file("BAT_WARM_ZONE", 0644, chip->dent,
				(void *)BAT_WARM_ZONE, &warm_cool_fops);
	debugfs_create_file("BAT_COOL_ZONE", 0644, chip->dent,
				(void *)BAT_COOL_ZONE, &warm_cool_fops);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		if (chip->pmic_chg_irq[chg_irq_data[i].irq_id])
			debugfs_create_file(chg_irq_data[i].name, 0444,
				chip->dent,
				(void *)chg_irq_data[i].irq_id,
				&rt_fops);
	}
}

static int pm8921_charger_suspend_noirq(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON, 0);
	if (rc)
		pr_err("Failed to Force Vref therm off rc=%d\n", rc);

	rc = pm8921_chg_set_lpm(chip, 1);
	if (rc)
		pr_err("Failed to set lpm rc=%d\n", rc);

	pm8921_chg_set_hw_clk_switching(chip);

	return 0;
}

static int pm8921_charger_resume_noirq(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	pm8921_chg_force_19p2mhz_clk(chip);

	rc = pm8921_chg_set_lpm(chip, 0);
	if (rc)
		pr_err("Failed to set lpm rc=%d\n", rc);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm on rc=%d\n", rc);
	return 0;
}

static int pm8921_charger_resume(struct device *dev)
{
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	if (pm8921_chg_is_enabled(chip, LOOP_CHANGE_IRQ)) {
		disable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
		pm8921_chg_disable_irq(chip, LOOP_CHANGE_IRQ);
	}

	if (chip->btc_override && (is_dc_chg_plugged_in(the_chip) ||
					is_usb_chg_plugged_in(the_chip)))
		schedule_delayed_work(&chip->btc_override_work, 0);

	return 0;
}

static int pm8921_charger_suspend(struct device *dev)
{
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	if (chip->btc_override)
		cancel_delayed_work_sync(&chip->btc_override_work);

	if (is_usb_chg_plugged_in(chip)) {
		pm8921_chg_enable_irq(chip, LOOP_CHANGE_IRQ);
		enable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
	}

	return 0;
}

/* OPPO 2012-09-12 chendx Add begin for pm8921 chg charge suspend mode */
#ifdef CONFIG_VENDOR_EDIT
/**
* @path :/sys/bus/platform/devices/pm8921-charger/pm8921_suspend_mode
*/
static ssize_t pm8921_suspend_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val = simple_strtoul(buf, NULL, 10);
	
	print_pm8921(DEBUG_INFO,"writes %ld to pm8921 chg suspend mode\n",val);
	if(val == 1) {
		print_pm8921(DEBUG_INFO,"pm8921 charger suspend mode enter!\n");
		pm8921_disable_source_current(true); /* Force BATFET=ON */
	}else if(val == 0) {
		print_pm8921(DEBUG_INFO,"pm8921 charger suspend mode exist!\n");
		pm8921_disable_source_current(false); /* release BATFET */
	}
	
	return size;
}
static DEVICE_ATTR(pm8921_suspend_mode, S_IRUGO | S_IWUSR, NULL, pm8921_suspend_mode_store);

/*Add begin for FTM CHARGER MODE */
/**
* @path :/sys/bus/platform/devices/pm8921-charger/pm8921_chg
*/
#ifdef FTM_CHARGE_MODE_FEATURE
static ssize_t pm8921_chg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if(the_chip == NULL){
		pr_err("%s:the chip is not init\n",__func__);
		return size;
	}
	
    /*set FTM charge mode,1. 60% stop charge,2.80% stop charge 3.normal charge*/
	if(val == 60){
	   pr_info("FTM CHARGER MODE: charge to capacity 60 stop mode =%ld\n",val);
	   /*Dump pm8921 register*/
	   __dump_pm8921_regs(the_chip);
	   the_chip->low_charge_mode = true;
	   return size;
	}else if(val == 80){
	   pr_info("FTM CHARGER MODE: charge to capacity 80 stop mode =%ld\n",val); 
	   the_chip->high_charge_mode = true;
	   return size;
	}else if(val == 11){
	   /* FTM charge mode :RF WIFI AT*/
	   pr_info("FTM TEST MODE,RF AT,WIFI TEST =%ld\n",val); 
	   the_chip->ftm_test_mode = true;
	   return size;
	}else if(val == 100){
	   pr_info("FTM CHARGER MODE: charge to normal 100 stop mode =%ld\n",val); 
	   the_chip->low_charge_mode = false;
	   the_chip->high_charge_mode = false;
	   debug_feature = 1;
	   return size;
	}

	if(val == 55){
	   debug_feature = 1;
	}

   /* OPPO 2013-04-08 chendx Add begin for auto test system */
   #ifdef AUTO_TEST_FEATURE
	if(val == 1100){
		chg_auto_test_feature = 1;
	}else if(val == 0011){
	    chg_auto_test_feature = 0;
	}

    /*charge timeout test*/
	if(val == 1122){
		chg_auto_test_feature = 1;
		the_chip->safe_charge_teoc = true;
		the_chip->auto_test_timeout = 1;
	}else if(val == 2211){
	    chg_auto_test_feature = 0;
		the_chip->safe_charge_teoc = false;
		the_chip->auto_test_timeout = 0;
	}

	/*charge voltage high test*/
	if(val == 1133){
		chg_auto_test_feature = 1;
		the_chip->auto_test_chargervol_high = 1;
	}else if(val == 3311){
	    chg_auto_test_feature = 0;
		the_chip->auto_test_chargervol_high = 0;
	}

	/*charger voltage low test*/
	if(val == 1144){
		chg_auto_test_feature = 1;
		the_chip->auto_test_chargervol_low = 1;
	}else if(val == 4411){
	    chg_auto_test_feature = 0;
		the_chip->auto_test_chargervol_low = 0;
	}

	/*battery voltage high test*/
	if(val == 1155){
		chg_auto_test_feature = 1;
		the_chip->auto_test_batvol_high = 1;
	}else if(val == 5511){
	    chg_auto_test_feature = 0;
		the_chip->auto_test_batvol_high = 0;
	}

	/*battery disconncet test*/
	if(val == 1166){
		chg_auto_test_feature = 1;
		the_chip->auto_test_bat_disconnect = 1;
	}else if(val == 6611){
	    chg_auto_test_feature = 0;
		the_chip->auto_test_bat_disconnect = 0;
	}

	/*cool temp charge finished test*/
	if(val == 1177){
		chg_auto_test_feature = 1;
		the_chip->chg_temp_test_mode= true;
		the_chip->chg_test_temp = DEFAULT_LITTLE_COLD_TEMP;
		the_chip->auto_test_cool_charge_finished = 1;
	}else if(val == 7711){
	    chg_auto_test_feature = 0;
		the_chip->chg_temp_test_mode= false;
		the_chip->chg_test_temp = DEFAULT_NORMAL_TEMP;
		the_chip->auto_test_cool_charge_finished = 0;
	}

	/*warm temp charge finished test*/
	if(val == 1188){
		chg_auto_test_feature = 1;
		the_chip->chg_temp_test_mode= true;
		the_chip->chg_test_temp = DEFAULT_WARM_TEMP;
		the_chip->auto_test_warm_charge_finished = 1;
	}else if(val == 8811){
	    chg_auto_test_feature = 0;
		the_chip->chg_temp_test_mode= false;
		the_chip->chg_test_temp = DEFAULT_NORMAL_TEMP;
		the_chip->auto_test_warm_charge_finished = 0;
	}

	/*hot temp test*/
	if(val == 1199){
		chg_auto_test_feature = 1;
		the_chip->chg_temp_test_mode= true;
		the_chip->chg_test_temp = DEFAULT_HOT_TEMP;
	}else if(val == 9911){
	    chg_auto_test_feature = 0;
		the_chip->chg_temp_test_mode= false;
		the_chip->chg_test_temp = DEFAULT_NORMAL_TEMP;
	}

	/*cold temp test*/
	if(val == 2200){
		chg_auto_test_feature = 1;
		the_chip->chg_temp_test_mode= true;
		the_chip->chg_test_temp = DEFAULT_COLD_TEMP;
	}else if(val == 0022){
	    chg_auto_test_feature = 0;
		the_chip->chg_temp_test_mode= false;
		the_chip->chg_test_temp = DEFAULT_NORMAL_TEMP;
	}
	#endif
    /* OPPO 2013-04-08 chendx Add end */
	
    /*  Add begin for charge temperature test */
	#ifdef TEMPERATURE_CHARGE_TEST_FEATURE
	if(val == 19){
		pr_info("Temperature charge test MODE open!!! =%ld\n",val);
		the_chip->chg_temp_test_mode= true;
	}else if(val == 20){
		pr_info("Temperature charge test MODE close!!! =%ld\n",val);
		the_chip->chg_temp_test_mode= false;
	}else if(val == 21){
		pr_info("Temperature charge test MODE cold!!! =%ld\n",val);
		the_chip->chg_test_temp = DEFAULT_COLD_TEMP;
	}else if(val == 22){
		pr_info("Temperature charge test MODE little_cold!!! =%ld\n",val);
		the_chip->chg_test_temp = DEFAULT_LITTLE_COLD_TEMP;
	}else if(val == 23){
		pr_info("Temperature charge test MODE cool!!! =%ld\n",val);
		the_chip->chg_test_temp = DEFAULT_COOL_TEMP;
	}else if(val == 24){
		pr_info("Temperature charge test MODE normal!!! =%ld\n",val);
		the_chip->chg_test_temp = DEFAULT_NORMAL_TEMP;
	}else if(val == 25){
		pr_info("Temperature charge test MODE warm!!! =%ld\n",val);
		the_chip->chg_test_temp = DEFAULT_WARM_TEMP;
	}else if(val == 26){
		pr_info("Temperature charge test MODE hot!!! =%ld\n",val);
		the_chip->chg_test_temp = DEFAULT_HOT_TEMP;
	}
	#endif
	
	return size;
}
static DEVICE_ATTR(pm8921_chg, S_IRUGO | S_IWUSR, NULL, pm8921_chg_store);
#endif
#endif
/* OPPO 2012-10-19 chendx Add end */
static int __devinit pm8921_charger_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct pm8921_chg_chip *chip;
	const struct pm8921_charger_platform_data *pdata
				= pdev->dev.platform_data;

	if (!pdata) {
		pr_err("missing platform data\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct pm8921_chg_chip),
					GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate pm_chg_chip\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	chip->ttrkl_time = pdata->ttrkl_time;
	chip->update_time = pdata->update_time;
	chip->max_voltage_mv = pdata->max_voltage;
	chip->alarm_low_mv = pdata->alarm_low_mv;
	chip->alarm_high_mv = pdata->alarm_high_mv;
	chip->min_voltage_mv = pdata->min_voltage;
	chip->safe_current_ma = pdata->safe_current_ma;
	chip->uvd_voltage_mv = pdata->uvd_thresh_voltage;
	chip->resume_voltage_delta = pdata->resume_voltage_delta;
	chip->resume_charge_percent = pdata->resume_charge_percent;
	chip->term_current = pdata->term_current;
	chip->vbat_channel = pdata->charger_cdata.vbat_channel;
	chip->batt_temp_channel = pdata->charger_cdata.batt_temp_channel;
	chip->batt_id_channel = pdata->charger_cdata.batt_id_channel;
	
	/* OPPO 2012-08-06 chendx Add begin for oppo charge */
	#ifdef CONFIG_VENDOR_EDIT
	chip->chg_voltage_channel = pdata->charger_cdata.chg_voltage_channel;
	/* Add begin for BTM */
	chip->normal_resume_voltage_delta = pdata->normal_resume_voltage_delta;
	chip->little_cold_bat_chg_current = pdata->little_cold_bat_chg_current;
	chip->normal_dcp_chg_current = pdata->normal_dcp_chg_current;
	chip->normal_sdp_chg_current = pdata->normal_sdp_chg_current;
	chip->little_cold_bat_voltage = pdata->little_cold_bat_voltage;
	chip->normal_bat_voltage = pdata->normal_bat_voltage;
	chip->pm8921_chg_type = USB_INVALID_CHARGER;
	chip->mhl_chg_current = pdata->mhl_chg_current;
	chip->nonstanard_mhl_chg_current = pdata->nonstanard_mhl_chg_current;
	chip->stanard_mhl_chg = false; 
	chip->mChargerCheckCounter = MAX_CHARGER_CHECK_COUNT; 
	chip->mBadChargerCounter = 0; 
	chip->mBadBatteryCounter = 0; 
	chip->charge_is_finished = false;
	chip->safe_charge_teoc = false;
	/*  Add begin for rsense */
	chip->r_sense = pdata->r_sense;
	chip->battery_temp = 23;
	/*Add begin for charge temperature test */
	#ifdef TEMPERATURE_CHARGE_TEST_FEATURE
	chip->chg_test_temp = 23;
	#endif
	chip->bad_charger_check_time = BAD_STATE_COUNT;
	chip->battery_voltage = 3500;
	chip->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	chip->charger_voltage = 0;
	chip->charge_current = 0;
	chip->charger_status = CHARGER_STATUS_GOOD;
	chip->battery_status = BATTERY_STATUS_GOOD;
	/*  Add begin for FTM CHARGE MODE */
	#ifdef FTM_CHARGE_MODE_FEATURE
	chip->low_charge_mode = false;
	chip->high_charge_mode = false;
	#endif
	/* Add begin for RF WIFI AT factory test */
	chip->ftm_test_mode= false;

    /* OPPO 2013-04-08 chendx Add begin for auto test system */
	#ifdef AUTO_TEST_FEATURE
	chip->auto_test_timeout = 0;
	chip->auto_test_chargervol_high = 0;
	chip->auto_test_chargervol_low = 0;
	chip->auto_test_batvol_high = 0;
	chip->auto_test_bat_disconnect = 0;
	chip->auto_test_cool_charge_finished = 0;
	chip->auto_test_warm_charge_finished = 0;
	#endif
   /* OPPO 2013-04-08 chendx Add end */

	chip->batt_temp_high_counter = 0;

	chip->soc_charge_counter = 0;
	chip->ocv_shutdown_counter = 0;
	chip->soc_fall_status = false;
	chip->soc_fall_counter = 0;

	chip->eoc_vbatt_counter = 0;
	chip->cv_long_counter = 0;
	
	/* Add begin for Temprate charger test */
	#ifdef TEMPERATURE_CHARGE_TEST_FEATURE
	chip->chg_temp_test_mode= false;
	#endif
	/* for charge done and recharging */
	chip->recharging_counter = 0;
	chip->vbatdet_mv = pdata->normal_bat_voltage;
	/* Add begin for BTM */
	Pm8921_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__NORMAL);
	chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
	chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1;	
	chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2;
	chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
	chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;
	#endif
	/* OPPO 2012-08-22 chendx Add end */
	chip->batt_id_min = pdata->batt_id_min;
	chip->batt_id_max = pdata->batt_id_max;
	if (pdata->cool_temp != INT_MIN)
		chip->cool_temp_dc = pdata->cool_temp * 10;
	else
		chip->cool_temp_dc = INT_MIN;

	if (pdata->warm_temp != INT_MIN)
		chip->warm_temp_dc = pdata->warm_temp * 10;
	else
		chip->warm_temp_dc = INT_MIN;

	chip->temp_check_period = pdata->temp_check_period;
	chip->max_bat_chg_current = pdata->max_bat_chg_current;
	/* Assign to corresponding module parameter */
	usb_max_current = pdata->usb_max_current;
	chip->cool_bat_chg_current = pdata->cool_bat_chg_current;
	chip->warm_bat_chg_current = pdata->warm_bat_chg_current;
	chip->cool_bat_voltage = pdata->cool_bat_voltage;
	chip->warm_bat_voltage = pdata->warm_bat_voltage;
	chip->trkl_voltage = pdata->trkl_voltage;
	chip->weak_voltage = pdata->weak_voltage;
	chip->trkl_current = pdata->trkl_current;
	chip->weak_current = pdata->weak_current;
	chip->vin_min = pdata->vin_min;
	chip->thermal_mitigation = pdata->thermal_mitigation;
	chip->thermal_levels = pdata->thermal_levels;
	chip->disable_chg_rmvl_wrkarnd = pdata->disable_chg_rmvl_wrkarnd;

	chip->cold_thr = pdata->cold_thr;
	chip->hot_thr = pdata->hot_thr;
	chip->rconn_mohm = pdata->rconn_mohm;
	chip->led_src_config = pdata->led_src_config;
	chip->has_dc_supply = pdata->has_dc_supply;
	chip->battery_less_hardware = pdata->battery_less_hardware;
	chip->btc_override = pdata->btc_override;
	if (chip->btc_override) {
		chip->btc_delay_ms = pdata->btc_delay_ms;
		chip->btc_override_cold_decidegc
			= pdata->btc_override_cold_degc * 10;
		chip->btc_override_hot_decidegc
			= pdata->btc_override_hot_degc * 10;
		chip->btc_panic_if_cant_stop_chg
			= pdata->btc_panic_if_cant_stop_chg;
	}

	if (chip->battery_less_hardware)
		charging_disabled = 1;

	chip->ibatmax_max_adj_ma = find_ibat_max_adj_ma(
					chip->max_bat_chg_current);

	rc = pm8921_chg_hw_init(chip);
	if (rc) {
		pr_err("couldn't init hardware rc=%d\n", rc);
		goto free_chip;
	}

	if (chip->btc_override)
		pm8921_chg_btc_override_init(chip);

	chip->stop_chg_upon_expiry = pdata->stop_chg_upon_expiry;
	chip->usb_type = POWER_SUPPLY_TYPE_UNKNOWN;

	chip->usb_psy.name = "usb";
	chip->usb_psy.type = POWER_SUPPLY_TYPE_USB;
	chip->usb_psy.supplied_to = pm_power_supplied_to;
	chip->usb_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
	chip->usb_psy.properties = pm_power_props_usb;
	chip->usb_psy.num_properties = ARRAY_SIZE(pm_power_props_usb);
	chip->usb_psy.get_property = pm_power_get_property_usb;
	chip->usb_psy.set_property = pm_power_set_property_usb;
	chip->usb_psy.property_is_writeable = usb_property_is_writeable;

	chip->dc_psy.name = "pm8921-dc";
	chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chip->dc_psy.supplied_to = pm_power_supplied_to;
	chip->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
	chip->dc_psy.properties = pm_power_props_mains;
	chip->dc_psy.num_properties = ARRAY_SIZE(pm_power_props_mains);
	chip->dc_psy.get_property = pm_power_get_property_mains;

	chip->batt_psy.name = "battery";
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.properties = msm_batt_power_props;
	chip->batt_psy.num_properties = ARRAY_SIZE(msm_batt_power_props);
	chip->batt_psy.get_property = pm_batt_power_get_property;
	chip->batt_psy.external_power_changed = pm_batt_external_power_changed;
	rc = power_supply_register(chip->dev, &chip->usb_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
		goto free_chip;
	}

	rc = power_supply_register(chip->dev, &chip->dc_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
		goto unregister_usb;
	}

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		pr_err("power_supply_register batt failed rc = %d\n", rc);
		goto unregister_dc;
	}

	platform_set_drvdata(pdev, chip);
	the_chip = chip;
/* OPPO 2012-08-06 chendx Add begin for oppo charge */
#ifdef CONFIG_VENDOR_EDIT
/*Add begin for charge suspend mode */
	rc = device_create_file(chip->dev, &dev_attr_pm8921_suspend_mode);
	if (rc < 0) {
		print_pm8921(DEBUG_INFO,"creat pm8921 chg suspend mode failed\n");
		device_remove_file(chip->dev, &dev_attr_pm8921_suspend_mode);
	}

/* Add begin for FTM CHARGE MODE */
#ifdef FTM_CHARGE_MODE_FEATURE
	rc = device_create_file(chip->dev, &dev_attr_pm8921_chg);
	if (rc < 0) {
		pr_err("%s: creat pm8921 ftm mode file failed ret = %d\n", 
					__func__, rc);
		device_remove_file(chip->dev, &dev_attr_pm8921_chg);
	}
#endif
#endif
/* OPPO 2012-10-19 chendx Add end */

	/* set initial state of the USB charger type to UNKNOWN */
	power_supply_set_supply_type(&chip->usb_psy, POWER_SUPPLY_TYPE_UNKNOWN);

	wake_lock_init(&chip->eoc_wake_lock, WAKE_LOCK_SUSPEND, "pm8921_eoc");
	INIT_DELAYED_WORK(&chip->eoc_work, eoc_worker);
	INIT_DELAYED_WORK(&chip->vin_collapse_check_work,
						vin_collapse_check_worker);
	INIT_DELAYED_WORK(&chip->unplug_check_work, unplug_check_worker);
/*OPPO,Jiangsm add begin for bad charger type detecting logic,2013-1-23*/
#ifdef CONFIG_VENDOR_EDIT
	INIT_WORK(&chip->cancel_charge_det, cancel_charge_det_worker);
#endif
/*OPPO,Jiangsm add end*/
	/* OPPO 2012-08-06 chendx Add begin for oppo charge */
	#ifdef CONFIG_VENDOR_EDIT
	/* Add begin for recharging */
	INIT_DELAYED_WORK(&chip->recharge_monitor_work, recharging_monitor_worker);
    /* Add begin for time out charge */
	INIT_DELAYED_WORK(&chip->teoc_work, teoc_worker);
	/*Add begin for charger valid */
	INIT_DELAYED_WORK(&chip->charger_valid_work,
							usbin_charger_valid);
	#endif
	/* OPPO 2012-08-15 chendx Add end */
	INIT_WORK(&chip->bms_notify.work, bms_notify);
	INIT_WORK(&chip->battery_id_valid_work, battery_id_valid);

	INIT_DELAYED_WORK(&chip->update_heartbeat_work, update_heartbeat);
	INIT_DELAYED_WORK(&chip->btc_override_work, btc_override_worker);

	rc = request_irqs(chip, pdev);
	if (rc) {
		pr_err("couldn't register interrupts rc=%d\n", rc);
		goto unregister_batt;
	}

	enable_irq_wake(chip->pmic_chg_irq[USBIN_VALID_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[DCIN_VALID_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[VBATDET_LOW_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[FASTCHG_IRQ]);

	create_debugfs_entries(chip);

	/* OPPO 2012-08-13 chendx Add begin for oppo charge */
	#ifdef CONFIG_VENDOR_EDIT
	init_completion(&stanard_mhl_wait);

	/* charger wakelock */
	/* init mutex*/
	#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&chip->pm8921_wake_lock, WAKE_LOCK_SUSPEND, "pm8921-charger");
	wake_lock_init(&chip->prevent_suspend_lock, WAKE_LOCK_SUSPEND, "pm8921-prevent-lock");
	#endif
	#endif
	/* OPPO 2012-08-31 chendx Add end */
	/* determine what state the charger is in */
	determine_initial_state(chip);

	if (chip->update_time)
		schedule_delayed_work(&chip->update_heartbeat_work,
				      round_jiffies_relative(msecs_to_jiffies
							(chip->update_time)));
	return 0;

unregister_batt:
	wake_lock_destroy(&chip->eoc_wake_lock);
	power_supply_unregister(&chip->batt_psy);
unregister_dc:
	power_supply_unregister(&chip->dc_psy);
unregister_usb:
	power_supply_unregister(&chip->usb_psy);
free_chip:
	kfree(chip);
	return rc;
}

static int __devexit pm8921_charger_remove(struct platform_device *pdev)
{
	struct pm8921_chg_chip *chip = platform_get_drvdata(pdev);

	//regulator_put(chip->vreg_xoadc);
	free_irqs(chip);
/* OPPO 2012-08-31 chendx Add begin for charger wakelock */
#ifdef CONFIG_VENDOR_EDIT
	/* destroy mutex */
	#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&chip->pm8921_wake_lock);
	wake_lock_destroy(&chip->prevent_suspend_lock);
	#endif


	/*charge suspend mode */
	device_remove_file(chip->dev, &dev_attr_pm8921_suspend_mode);	

	/*FTM CHARGE TEST MODE */
#ifdef FTM_CHARGE_MODE_FEATURE
	device_remove_file(chip->dev, &dev_attr_pm8921_chg);
#endif
#endif /*CONFIG_VENDOR_EDIT*/
/* OPPO 2012-10-19 chendx Add end */

	platform_set_drvdata(pdev, NULL);
	the_chip = NULL;
	kfree(chip);
	return 0;
}
static const struct dev_pm_ops pm8921_pm_ops = {
	.suspend	= pm8921_charger_suspend,
	.suspend_noirq  = pm8921_charger_suspend_noirq,
	.resume_noirq   = pm8921_charger_resume_noirq,
	.resume		= pm8921_charger_resume,
};
static struct platform_driver pm8921_charger_driver = {
	.probe		= pm8921_charger_probe,
	.remove		= __devexit_p(pm8921_charger_remove),
	.driver		= {
			.name	= PM8921_CHARGER_DEV_NAME,
			.owner	= THIS_MODULE,
			.pm	= &pm8921_pm_ops,
	},
};

static int __init pm8921_charger_init(void)
{
	return platform_driver_register(&pm8921_charger_driver);
}

static void __exit pm8921_charger_exit(void)
{
	platform_driver_unregister(&pm8921_charger_driver);
}

late_initcall(pm8921_charger_init);
module_exit(pm8921_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8921 charger/battery driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:" PM8921_CHARGER_DEV_NAME);
