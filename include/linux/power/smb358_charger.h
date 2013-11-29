#ifndef __SMB358_CHARGER_H__
#define __SMB358_CHARGER_H__
#include <linux/power_supply.h>
#include <linux/earlysuspend.h>

#define SMB358_CHARGER_NAME "smb358_charger"

struct smb358_charger_platform_data {
	struct power_supply_info battery_info;
	
	unsigned int	max_charge_current;
	unsigned int	max_charge_voltage;
	unsigned int	pre_charge_current;
	unsigned int	termination_current;
	
	unsigned int	pre_to_fast_voltage;
	unsigned int	mains_current_limit;
	unsigned int	usb_hc_current_limit;
	unsigned int	chip_temp_threshold;
	int			soft_cold_temp_limit;
	int			soft_hot_temp_limit;
	int			hard_cold_temp_limit;
	int			hard_hot_temp_limit;
	bool			suspend_on_hard_temp_limit;
	unsigned int	soft_temp_limit_compensation;
	unsigned int	charge_current_compensation;
	
	int			irq_gpio;
};

struct smb358_battery_platform_data {
	int	battery_min_designed_voltage;
	int	battery_max_designed_voltage;
};

struct smb358_power_supply {
	struct power_supply	*mains;
	struct power_supply	*usb;
	struct power_supply	*battery;
};

typedef enum {
	CHARGER_STATUS__INVALID,
	CHARGER_STATUS__GOOD,
	CHARGER_STATUS__UVP,//under voltage
	CHARGER_STATUS__OVP//over voltage
} smb358_charger_status;

/*sjc0806
typedef enum {
	BATTERY_STATUS__INVALID,
	BATTERY_STATUS__GOOD,
	BATTERY_STATUS__UVP,
	BATTERY_STATUS__OVP,
} smb358_battery_status;
*/
typedef enum {
	BATTERY_STATUS__INVALID,
	BATTERY_STATUS__GOOD,
	BATTERY_STATUS__OVERHEAT,
	BATTERY_STATUS__DEAD,
	BATTERY_STATUS__OVP,
	BATTERY_STATUS__UNSPEC_FAILURE,
	BATTERY_STATUS__COLD,
	BATTERY_STATUS__UVP,
} smb358_battery_status;

typedef enum {
	BATTERY_MISSING_STATUS__INVALID,
	BATTERY_MISSING_STATUS__MISSED,
	BATTERY_MISSING_STATUS__GOOD
} smb358_battery_missing_status;

typedef enum {
	CHARGING_STATUS__INVALID,
	CHARGING_STATUS__CHARGING,
	CHARGING_STATUS__DISCHARGING,
	CHARGING_STATUS__NOT_CHARGING,
	CHARGING_STATUS__FULL
} smb358_charging_status;

typedef enum {
	CHARGER_TYPE__INVALID,
	CHARGER_TYPE__SDP,//usb
	CHARGER_TYPE__DCP,//standard
	CHARGER_TYPE__NON_DCP,//non standard
	CHARGER_TYPE__HDMI,//
	CHARGER_TYPE__CDP,//charge down port
	CHARGER_TYPE__ACA,
	CHARGER_TYPE__OTG//sjc0808otg
} smb358_charger_type;

typedef enum {
	BATTERY_TEMP_REGION__INVALID,
	BATTERY_TEMP_REGION__COLD,
	BATTERY_TEMP_REGION__LITTLE_COLD,
	BATTERY_TEMP_REGION__COOL,
	BATTERY_TEMP_REGION__NORMAL,
	BATTERY_TEMP_REGION__WARM,
	BATTERY_TEMP_REGION__HOT
} smb358_battery_temperature_region;

typedef enum {
	CHARGING_MODE__INVALID,
	CHARGING_MODE__TRICK_CHG,
	CHARGING_MODE__PRE_CHG,
	CHARGING_MODE__FAST_CHG,
	CHARGING_MODE__TAPER_CHG
} smb358_charging_mode;

struct smb358_power_supply_data {
	unsigned int	charger_voltage;
	unsigned int 	battery_voltage;
	int			charging_current;
	int			battery_temperature;
	int			battery_capacity;
	int			battery_soh;
	int			battery_fcc;
	smb358_charger_type				charger_type;
	smb358_charger_status				charger_status;
	smb358_battery_status				battery_status;
	smb358_battery_missing_status 		battery_missing_status;
	smb358_charging_status			charging_status;
	smb358_charging_mode				charging_mode;
	smb358_battery_temperature_region	battery_temperature_region;
	int 			critical_temperature_cold_to_little_cold;
	int			critical_temperature_little_cold_to_cool;
	int 			critical_temperature_cool_to_normal;
	int 			critical_temperature_normal_to_warm;
	int 			critical_temperature_warm_to_hot;
	bool			charging_complete_request;
	bool 		charging_resume_request;
	bool			charging_timeout;
	bool			charging_timeout_status;//sjc0820
};

struct smb358_debug_data {
	bool			debug_enabled;
	bool			printk_enabled;
	bool 		dump_registers_enabled;
	smb358_charger_type	charger_type;
	int			charger_voltage;
	int  			battery_voltage;
	int 			battery_temperature;
	int  			battery_capacity;
	int 			charging_current;

	bool	 		charging_timeout;//
};

struct smb358_charger {
	struct mutex		lock;
	struct wake_lock	wlock;
	struct wake_lock	prevent_wlock;
	struct i2c_client	*client;
	struct device		dev;
	bool				suspended;
	int				irq;
	unsigned long		irq_flags;
	struct work_struct			irq_work;
	struct delayed_work		start_charging_work;//starting chg in 1s
	struct delayed_work		update_work;//update powersupply data very 5s
	struct workqueue_struct	*work_queue;
/*for debug*/
	struct smb358_debug_data				debug_data;
	struct smb358_power_supply_data		power_supply_data;
	struct smb358_power_supply 			power_supplies;
	struct smb358_charger_platform_data	charger_platform_data;
	struct smb358_battery_platform_data		battery_platform_data;
	bool				user_debug_enabled;
	bool				otg_enabled;//sjc0808otg
	struct hrtimer	max_chg_timeout_timer;//sjc0820
	bool				aicl_status;//sjc0824
	int				aicl_result;//sjc1010
	struct early_suspend	early_suspend;//sjc0927
	bool				early_suspend_status;//sjc0927
	bool				charging_current_aicl_status;//sjc1003
};

void smb358_charger_connected(smb358_charger_type chg_type);

struct oppo_battery_fuelgauge {
	int (*get_battery_capacity) (void);
	int (*get_battery_mvolts) (void);
	int (*get_battery_temperature) (void);
	int (*get_battery_soh) (void);
	int (*get_battery_fcc) (void);
	//int (*get_vchg_mvolts) (void);
	int (*get_ichg_current) (void);
	int (*set_full_capacity) (void);
};
bool get_charging_status(void);
void smb358_battery_fuelgauge_register(struct oppo_battery_fuelgauge *fuelgauge);
void smb358_battery_fuelgauge_unregister(struct oppo_battery_fuelgauge *fuelgauge);
bool is_show_gague_log(void);
#endif
