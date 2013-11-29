/*
 * BQ27541 battery gauge driver
 *
 * Copyright (C) 2012-2013 OPPO Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <mach/mpp.h>
#include <linux/pmic8058-xoadc.h>
#include <linux/msm_adc.h>
#include <linux/power/smb358_charger.h>
#include <linux/pcb_version.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/mpp.h>
#include <mach/board-msm8660.h>

#include <linux/rtc.h>


#define gague_debug(fmt, ...) \
	({ if (is_show_gague_log()) printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__); 0; })
/*
//#undef pr_info
//#define pr_info(fmt, ...) \
//	printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
*/
	
#define DRIVER_VERSION			"1.1.0"

#define INVALID_ERROR		-1

#define NONFUELGAGUE_SUPPORT_FEATRUE

#define ACHG_PIN	XOADC_MPP_4
#define VCHG_PIN	XOADC_MPP_5
#define BATT_ID		XOADC_MPP_8

#define BQ27541_REG_CNTL		0x00
#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_FCC		0x12
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_TTE		0x16
#define BQ27541_REG_TTF		0x18
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_SOC		0x2c
#define BQ27541_REG_SOH		0x2e

#define BQ27541_CHEM_ID_1		0x136
#define BQ27541_CHEM_ID_2		0x106
#define BQ27541_CHEM_ID_3		0x354
#define BQ27541_CHEM_ID_4		0x3141

/* Control subcommands */
#define BQ27541_SUBCMD_CTNL_STATUS  0x0000
#define BQ27541_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27541_SUBCMD_FW_VER  0x0002
#define BQ27541_SUBCMD_HW_VER  0x0003
#define BQ27541_SUBCMD_DF_CSUM  0x0004
#define BQ27541_SUBCMD_PREV_MACW   0x0007
#define BQ27541_SUBCMD_CHEM_ID   0x0008
#define BQ27541_SUBCMD_BD_OFFSET   0x0009
#define BQ27541_SUBCMD_INT_OFFSET  0x000a
#define BQ27541_SUBCMD_CC_VER   0x000b
#define BQ27541_SUBCMD_OCV  0x000c
#define BQ27541_SUBCMD_BAT_INS   0x000d
#define BQ27541_SUBCMD_BAT_REM   0x000e
#define BQ27541_SUBCMD_SET_HIB   0x0011
#define BQ27541_SUBCMD_CLR_HIB   0x0012
#define BQ27541_SUBCMD_SET_SLP   0x0013
#define BQ27541_SUBCMD_CLR_SLP   0x0014
#define BQ27541_SUBCMD_FCT_RES   0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27541_SUBCMD_SEALED   0x0020
#define BQ27541_SUBCMD_ENABLE_IT    0x0021
#define BQ27541_SUBCMD_DISABLE_IT   0x0023
#define BQ27541_SUBCMD_CAL_MODE  0x0040
#define BQ27541_SUBCMD_RESET   0x0041
#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   (-2731)
#define BQ27541_INIT_DELAY   ((HZ)*1)

static bool bq27541_present_flag = true;

#define BQ27000_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27000_FLAG_CHGS		BIT(7)

#define BQ27500_REG_SOC		0x2c
#define BQ27500_FLAG_DSC		BIT(0)
#define BQ27500_FLAG_FC		BIT(9)

struct bq27541_device_info;
struct bq27541_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27541_device_info *di);
};
#define BATTERY_COLD_TEMP			-10
#define BATTERY_COOL_TEMP			5
#define BATTERY_WARM_TEMP			45
#define BATTERY_HOT_TEMP			55
#define BATTERY_REMOVE_TEMP		-40
#define BATTERY_SHUTDOWN_TEMP	680

#define BATTERY_DEFAULT_CAPACITY	0

#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   (-2731)

#define BATTERY_DEFAULT_TEMP 230 //23 C
#define VCHG_DEFAULT_VOLTAGE 5000
#define BOOT_TIME 20//sjc 12->20 //60S
enum bq27541_chip { BQ27541, BQ27500 };/*use to support more bq chip*/

#ifndef GAGUE_MIN
#define GAGUE_MIN(_a, _b)     (((_a) < (_b)) ? (_a) : (_b))
#endif

#ifndef GAGUE_MAX
#define GAGUE_MAX(_a, _b)     (((_a) > (_b)) ? (_a) : (_b))
#endif
#if 0//sjc
static struct pm8xxx_mpp_config_data mpp_data_enable = 
{
    .type = PM_MPP_TYPE_A_INPUT,
	.level = PM_MPP_AIN_AMUX_CH8,
	.control = PM_MPP_AOUT_CTL_ENABLE,
};
static struct pm8xxx_mpp_config_data mpp_data_disable = 
{
    .type = PM_MPP_TYPE_SINK,
	.level = PM_MPP_CS_OUT_5MA,
	.control = PM_MPP_CS_CTL_DISABLE,
};
//#else
static struct pm8xxx_mpp_config_data pm8xxx_adc_mpp_config = {
	.type	= PM8XXX_MPP_TYPE_A_INPUT,
	/* AMUX6 is dedicated to be used for apps processor */
	.level	= PM8XXX_MPP_AIN_AMUX_CH6,
	.control	= PM8XXX_MPP_AOUT_CTRL_DISABLE,
};
/* MPP Configuration for default settings */
static struct pm8xxx_mpp_config_data pm8xxx_adc_mpp_unconfig = {
	.type	= PM8XXX_MPP_TYPE_SINK,
	.level	= PM8XXX_MPP_AIN_AMUX_CH5,
	.control	= PM8XXX_MPP_AOUT_CTRL_DISABLE,
};
#endif

struct bq27541_device_info {
	struct device 		*dev;
	int			id;
	struct  bq27541_access_methods	*bus;
	enum bq27541_chip	chip;

	struct i2c_client	*client;
	
	//add reason batt present check counter 3 times(one time will take 5s)
	u8 batt_present_counter;  
	u8 batt_is_present;  
	unsigned int batt_capacity_pre;
	unsigned int vbatt_mvolts;//get from fulegague really
	u16 batt_temperate_pre;
    u8  capacity_saltate_counter;
	u8 HighTempShutdownCount;
	unsigned long rtc_resume_time;
	unsigned long rtc_suspend_time;
	struct rtc_device *rtc;
	int temp_pre;//sjc1024
	int soh_pre;
	int fcc_pre;
	unsigned int batt_cur_pre;
	atomic_t suspended;
};
static struct bq27541_device_info *bq27541_info;
#if 0 //sjc
/* add adc read interface 012-03-15 */
static int batt_read_adc(int channel, int *mv_reading)
{
	int ret;
	void *h;
	struct adc_chan_result adc_chan_result;
	struct completion  conv_complete_evt;

	pr_debug("%s: called for %d\n", __func__, channel);
	ret = adc_channel_open(channel, &h);
	if (ret) {
		pr_err("%s: couldnt open channel %d ret=%d\n",
					__func__, channel, ret);
		goto out;
	}
	init_completion(&conv_complete_evt);
	ret = adc_channel_request_conv(h, &conv_complete_evt);
	if (ret) {
		pr_err("%s: couldnt request conv channel %d ret=%d\n",
						__func__, channel, ret);
		goto out;
	}
	wait_for_completion(&conv_complete_evt);
	ret = adc_channel_read_result(h, &adc_chan_result);
	if (ret) {
		pr_err("%s: couldnt read result channel %d ret=%d\n",
						__func__, channel, ret);
		goto out;
	}
	ret = adc_channel_close(h);
	if (ret) {
		pr_err("%s: couldnt close channel %d ret=%d\n",
						__func__, channel, ret);
	}
	if (mv_reading)
		*mv_reading = adc_chan_result.measurement;

	pr_debug("%s: done for %d\n", __func__, channel);
	return adc_chan_result.physical;
out:
	pr_debug("%s: done for %d\n", __func__, channel);
	return -EINVAL;

}

/* add for read vchg voltage from xo mpp05
*/
static int  free_amux8(int pin)
{
	int rc=0;
	rc = pm8xxx_mpp_config(PM8058_MPP_PM_TO_SYS(pin),&mpp_data_disable);
	if (rc)
		dev_info(bq27541_info->dev, "%s free AMUX channel fail,pin: %d,rt:%d\n",__func__,pin,rc);
	return rc;
}

static int read_mpp_from_amux8(int pin,int *mv)
{
	int i=0,j=0,rt=0,mv_reading=0,rmv=0;

    /*free mpp08 first to avoid mpp voltage error*/
	free_amux8(BATT_ID);

	rt = pm8xxx_mpp_config(PM8058_MPP_PM_TO_SYS(pin),&mpp_data_enable);
	if(rt) {
		dev_info(bq27541_info->dev, "%s config MPP fail,pin: %d,rt:%d\n",__func__,pin,rt);
		goto OUT;
	}
	
	for(i=0;i<10;i++){
		mdelay(5);//delay 5ms
		rt = batt_read_adc(CHANNEL_ADC_AMUX8, &mv_reading);
		//dev_info(bq27541_info->dev, "%s read vchg voltage =%d,pin: %d,rt:%d\n",__func__,mv_reading,pin,rt);
		if(rt>=0)
		{	
			rmv += mv_reading;
			j++;
		}
		else
		{
		      pr_err("%s read_mpp_from_amux8 rt:%d\n",__func__,rt);
		}
		if(j>=3)	break;
	}
	OUT:
	if(j>0)
		{
		*mv = rmv/j;
		return	0;
		}
	else{
		return 	1;
		}
}
/* end adc read interface 012-03-15 */
#define ADC_ICHG_CURRENT__MAX 1200 //1200mA
#define ADC_ICHG_CURRENT__MIN 0
static int adc_get_chg_current(void)
{
	int chg_current=0;
	int rt=0;   
	int i;

	for(i = 0; i < 3; i++)
	{
       	rt = read_mpp_from_amux8(ACHG_PIN,&chg_current);
		if(!rt)
		{
			free_amux8(ACHG_PIN);
			chg_current = (chg_current*10)/8;
			if(chg_current > ADC_ICHG_CURRENT__MAX ||chg_current < ADC_ICHG_CURRENT__MIN)
			{
			       chg_current = 0;
			}
			pr_info("%s:chg_current = %dmA\n",__func__,chg_current);
			return chg_current;
		}
	}
	
	//read mpp from amux8 error
	pr_err("%s read chg_current failed\n",__func__);
	return 0;//0ma
}

static int adc_get_battery_mvolts(void)
{
	int vbatt_mv;

	vbatt_mv = batt_read_adc(CHANNEL_ADC_VBATT, NULL);
	pr_debug("%s: vbatt_mv is %dmv\n", __func__, vbatt_mv);
	if (vbatt_mv > 0){
		return vbatt_mv;
	}
	/*
	 * return 3500 to tell the upper layers
	 * we couldnt read the battery voltage
	 */
	return 3500;
}
#endif //sjc
/*
 * sjc 2013-07-31 modify
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27541_get_ichg_current(void)
{
	int ret;
	u8 buffer[2];
	//s8 buffer[2];
	unsigned int batt_cur;
	//s16 batt_cur;
	int i;

	if (atomic_read(&bq27541_info->suspended) == 1) {//sjc1024
		pr_info("%s:sus, use fulegague pre current=[%d]\n", __func__, bq27541_info->batt_cur_pre);
		return bq27541_info->batt_cur_pre;
	}

	for(i = 0; i < 3; i++)
	{
		ret = i2c_smbus_read_i2c_block_data(bq27541_info->client,BQ27541_REG_AI,0x02,buffer);
		if(ret >= 0){
			batt_cur = ((buffer[1] << 8) & 0xFF00);
			batt_cur = ((unsigned int)(batt_cur + buffer[0]) & 0x0000FFFF);
			//batt_cur = ((batt_cur + buffer[0]) & 0x0000FFFF); 
			gague_debug("%s:ichg current is =%dmA\n",__func__,batt_cur);
			bq27541_info->batt_cur_pre = batt_cur;//sjc1024
			return batt_cur ;
		}
	}
	//read i2c block data error
	dev_err(bq27541_info->dev,"bq27541_get_charger_current fail\n");
	return 0;
}

#define DISCHARGING_CURRENT_VALUE_MAX 64336 //-1200mA
#define CHARGING_CURRENT_VALUE_MAX 1200 //1200mA

/**
    get the BQ27541_REG_AI value to check charging status
*/
/* OPPO 2012-06-07 wangjc Delete begin for not used. */
#if 0
static bool bq27541_batt_charging_status(void)
{
	int ichg_value;

	ichg_value = bq27541_get_ichg_current();
	if(ichg_value > DISCHARGING_CURRENT_VALUE_MAX){
		return false;
	}else{
		return true;
	}		  
}
#endif
/* OPPO 2012-06-07 wangjc Delete end */

/*
 * sjc 2013-07-31 modify
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
int bq27541_get_battery_temperature(void)
{
	int ret;
	u8 buffer[2];
	//s8 buffer[2];
	unsigned int batt_tp;
	int temp;
	int i;
	
	if (atomic_read(&bq27541_info->suspended) == 1) {//sjc1024
		pr_info("%s:sus, use fulegague pre temperature=[%d]\n", __func__, bq27541_info->temp_pre);
		return bq27541_info->temp_pre;
	}
	
	for(i = 0; i < 3; i++)
	{
		ret = i2c_smbus_read_i2c_block_data(bq27541_info->client,BQ27541_REG_TEMP,0x02,buffer);
		if(ret >= 0){
			batt_tp = ((buffer[1] << 8) & 0xFF00);
			batt_tp = ((unsigned int)(batt_tp + buffer[0]) & 0x0000FFFF);
			temp = (batt_tp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN);
			gague_debug("%s:temperature value =%d C\n",__func__,temp);
			bq27541_info->temp_pre = temp;//sjc1024
			return temp;
		}
	}
	//read i2c block data error
	dev_err(bq27541_info->dev," bq27541_get_battery_temperature fail \n");
	return BATTERY_DEFAULT_TEMP;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
#define BATTERY_DEFAULT_VOL 3500
static int bq27541_get_battery_voltage(void)
{
	int ret;
	u8 buffer[2];
	unsigned int vbatt_mv;
	int i;

	if (atomic_read(&bq27541_info->suspended) == 1) {//sjc1024
		pr_info("%s:sus, use fulegague pre voltage=[%d]\n", __func__, bq27541_info->vbatt_mvolts);
		return bq27541_info->vbatt_mvolts;
	}
	
	for(i = 0; i < 3; i++)
	{
		ret = i2c_smbus_read_i2c_block_data(bq27541_info->client,BQ27541_REG_VOLT,0x02,buffer);
		if(ret >= 0)
		{
			vbatt_mv = ((buffer[1] << 8) & 0xFF00);
			vbatt_mv = ((unsigned int)(vbatt_mv + buffer[0]) & 0x0000FFFF); 
			/*back vbatt mvolts*/
			bq27541_info->vbatt_mvolts = vbatt_mv;
			gague_debug("%s:voltage value =%dmv\n",__func__,vbatt_mv);
			return vbatt_mv;
		}
	}
	//read i2c block data error
	dev_err(bq27541_info->dev,"bq27541_get_battery_voltage fail!\n");
	//return BATTERY_DEFAULT_VOL;
	return -1;//sjc0814
}
#if 0//sjc
/*  get vchg voltage in mV */
int adc_get_vchg_mvolts(void)
{
    int vchg_mv;
	int rt=0;   

    rt = read_mpp_from_amux8(VCHG_PIN,&vchg_mv);
	if(rt){
		 dev_err(bq27541_info->dev, "%s read vchg voltage failed\n",__func__);
		 return 0;//0mv
	}
	
	free_amux8(VCHG_PIN);
	vchg_mv = (vchg_mv*32)/10;
    gague_debug("%s:vchg voltage is %dmv\n",__func__,vchg_mv);
	return vchg_mv;
}
#endif
/*
 * Common code for BQ27541 devices
 */
static int bq27541_read(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}
#if 0
/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27541_get_battery_temperature(void)
{
	int ret;
	u8 buffer[2];
	unsigned int batt_tp;
	int i;

	return BATTERY_DEFAULT_TEMP;
	
	for(i = 0; i < 3; i++)
	{
		ret = i2c_smbus_read_i2c_block_data(bq27541_info->client,BQ27541_REG_TEMP,0x02,buffer);
		if(ret >= 0)
		{
			batt_tp = ((buffer[1] << 8) & 0xFF00);
			batt_tp = ((unsigned int)(batt_tp + buffer[0]) & 0x0000FFFF);  
			printk(KERN_INFO "%s:temperature value =%d\n",__func__,batt_tp);
			return batt_tp /100 ;
		}
	}
	//read i2c block data error
	dev_err(bq27541_info->dev," bq27541_get_battery_temperature fail \n");
	return BATTERY_DEFAULT_TEMP;
}

#define BATT_PRESENT_COUNT 2
static int bq27541_is_battery_present(void)
{	
	int batt_temp = 0;

	/*check battery present */   
	batt_temp = bq27541_get_battery_temperature();

	 /* battery temp <= -40 degree C,we will consider battery remove*/
	 if(batt_temp <= BATTERY_REMOVE_TEMP){
	 	    if(bq27541_info->batt_is_present)
	 	    {
		 	        bq27541_info->batt_present_counter ++;
				 if(bq27541_info->batt_present_counter >= 2)
				 {
				        /*reset batt present counter*/
	                            bq27541_info->batt_present_counter = 0;
				        bq27541_info->batt_is_present = 0;
				        pr_info("%s:Warning batt is remove suceessfully!\n",__func__);		
					 return 0;
				}else{

				       return 1;	
				}
	 	     }else{
	 	          /*battery is remove*/
		          return 0;	
	 	     }
	 }
	 
	 /*reset batt present counter*/
        bq27541_info->batt_present_counter = 0;

	 /*battery is present*/
	 bq27541_info->batt_is_present = 1;
	 
	 return 1;
}

static int bq27541_is_battery_temp_within_range(void)
{

	/*
	*  Todo battery is always within range by chendexiang
	*/
	return 1;
}

static int bq27541_is_battery_id_valid(void)
{

	/*
	 * The readings are not in range
	 * assume battery is present for now
	 */
	return 1;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
 #define BATTERY_DEFAULT_VOL 3500
 static int bq27541_get_battery_voltage(void)
{
	int ret;
	u8 buffer[2];
	unsigned int vbatt_mv;
	int i;

	return BATTERY_DEFAULT_VOL;

	for(i = 0; i < 3; i++)
	{
		ret = i2c_smbus_read_i2c_block_data(bq27541_info->client,BQ27541_REG_VOLT,0x02,buffer);
		if(ret >= 0)
		{
			vbatt_mv = ((buffer[1] << 8) & 0xFF00);
			vbatt_mv = ((unsigned int)(vbatt_mv + buffer[0]) & 0x0000FFFF);  
			printk(KERN_INFO "%s:voltage value =%d\n",__func__,vbatt_mv);
			return vbatt_mv;
		}
	}
	//read i2c block data error
	dev_err(bq27541_info->dev,"bq27541_get_battery_voltage fail!\n");
	return BATTERY_DEFAULT_VOL;
}


/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */

int bq27541_get_battery_current(void)
{
	int ret;
	u8 buffer[2];
	unsigned int batt_cur;
	int i;

	return 0;

	for(i = 0; i < 3; i++)
	{
		ret = i2c_smbus_read_i2c_block_data(bq27541_info->client,BQ27541_REG_AI,0x02,buffer);
		if(ret >= 0)
		{
			batt_cur = ((buffer[1] << 8) & 0xFF00);
			batt_cur = ((unsigned int)(batt_cur + buffer[0]) & 0x0000FFFF);  
			printk(KERN_INFO "%s:current value =%d\n",__func__,batt_cur);
			return batt_cur ;
		}
	}
	//read i2c block data error
	dev_err(bq27541_info->dev,"bq27541_get_battery_current fail\n");
	return 0;
}

int bq27541_get_battery_status(void)
{
	int flags = 0;
	int status;/*battery status*/
	int ret;

	ret = bq27541_read(BQ27541_REG_FLAGS, &flags, 0, bq27541_info);
	if(ret < 0){
		dev_err(bq27541_info->dev, "error reading flags\n");
		return ret;
	}

	if (bq27541_info->chip == BQ27500) {
		if (flags & BQ27500_FLAG_FC)
			status = POWER_SUPPLY_STATUS_FULL;
		else if (flags & BQ27500_FLAG_DSC)
			status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		if (flags & BQ27000_FLAG_CHGS)
			status = POWER_SUPPLY_STATUS_CHARGING;
		else
			status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	return status;
}
#endif
/*
 * Read a time register.
 * Return < 0 if something fails.
 * 1. timetoempty 0x16/0x17
 * 2. timetofull 0x18/0x19
 * 3. AtRateTimeToEmpty 0x04/0x05
 * 4. StandbyTimeToEmpty 0x1c/0x1d
 */
#if 0
int bq27541_get_battery_time(int reg)
{
      int ret;
	u8 buffer[2];
	unsigned int batt_time;

	ret = i2c_smbus_read_i2c_block_data(bq27541_info->client,reg,0x02,buffer);
	if(ret != 0x02){
	    dev_err(bq27541_info->dev,"i2c_smbus_read_i2c_block_data err\n");
	}  
      batt_time = ((buffer[1] << 8) & 0xFF00);
      batt_time = ((unsigned int)(batt_time + buffer[0]) & 0x0000FFFF);  
	
      //printk(KERN_INFO "%s:capacity_lv value =%d\n",__func__,batt_time);

	return batt_time ;
}
#endif

/**
*   calibrate batt capacity which from gague
*/
#define BATT_CAPACITY_CALIB 
#define CAPACITY_SALTATE_CHANGE_MVOLTS 3650
#define CAPACITY_SALTATE_CHANGE_MVOLTS_LOW 3500
/*battery high than 3600mv,set the capacity saltate to 60s*/
#define CAPACITY_SALTATE_COUNTER 20//sjc 12->20
/*battery low than 3650,set the capacity saltate to 30s*/
#define CAPACITY_SALTATE_COUNTER_MID 10//sjc 6->10
/*battery low than 3500,set the capacity saltate to 30s*/
#define CAPACITY_SALTATE_COUNTER_LOW 5//sjc 3->5 


#define BATT_CAPACITY_FULL 100
#define BATT_CAPACITY_INVALID_LEVEL 2
extern bool get_charging_status(void);

static unsigned int bq27541_capacity_calibrate(unsigned int batt_capacity)
{
	#ifdef BATT_CAPACITY_CALIB
	int charging_now=0;
    unsigned int capacity_calib;

	capacity_calib = batt_capacity;

    //charging_now = bq27541_batt_charging_status();
	charging_now = get_charging_status();
	
	/**
	*   check capacity if abnormally now
	*   if we continute get battery capacity saltate more than 3times then we consider capacity saltate now
	*/
	if(abs(batt_capacity - bq27541_info->batt_capacity_pre ) >= BATT_CAPACITY_INVALID_LEVEL){
		bq27541_info->capacity_saltate_counter  ++;
		pr_info("%s:!Warning battery capacity skip, fulegague capacity=%d%%,current=%d%%,%d,charging =%d,%dmv\n",
			__func__,batt_capacity,bq27541_info->batt_capacity_pre,bq27541_info->capacity_saltate_counter,charging_now,bq27541_info->vbatt_mvolts);

        if(bq27541_info->vbatt_mvolts > CAPACITY_SALTATE_CHANGE_MVOLTS){    
			if(bq27541_info->capacity_saltate_counter < CAPACITY_SALTATE_COUNTER){
				return bq27541_info->batt_capacity_pre;
			}else{
				bq27541_info->capacity_saltate_counter = 0;
			}
        }else if(bq27541_info->vbatt_mvolts > CAPACITY_SALTATE_CHANGE_MVOLTS_LOW){
        	if(bq27541_info->capacity_saltate_counter < CAPACITY_SALTATE_COUNTER_MID){
				return bq27541_info->batt_capacity_pre;
			}else{
				bq27541_info->capacity_saltate_counter = 0;
			}
        }else {
			if(bq27541_info->capacity_saltate_counter < CAPACITY_SALTATE_COUNTER_LOW){
				return bq27541_info->batt_capacity_pre;
			}else{
				bq27541_info->capacity_saltate_counter = 0;
			}
		}
	}else{
	       bq27541_info->capacity_saltate_counter = 0;
	}
	
	/*fliter error capacity when read gague capacity error*/
	if(charging_now){
		if(batt_capacity > bq27541_info->batt_capacity_pre){
			/*50seconds update capacity 1%*/
			capacity_calib = bq27541_info->batt_capacity_pre + 1;
		}else{
			/*Charging status capacity can not reduce*/
			capacity_calib = GAGUE_MAX(batt_capacity, bq27541_info->batt_capacity_pre);
		}
	}else{
		if(batt_capacity < bq27541_info->batt_capacity_pre){
			capacity_calib = bq27541_info->batt_capacity_pre - 1;
		}else{
			/*Charging status capacity can not reduce*/
			capacity_calib = GAGUE_MIN(batt_capacity, bq27541_info->batt_capacity_pre);
		}
	}
		
	if(capacity_calib>=BATT_CAPACITY_FULL)
		capacity_calib = BATT_CAPACITY_FULL;
	
	gague_debug("%s: charging status =%d,capacity origin =[%d%%],calib capacity =[%d%%]\n",
		__func__,charging_now,batt_capacity,capacity_calib);  

	return capacity_calib;
	#else
		return batt_capacity;
	#endif
}

#define BATT_CAPACITY_VALID_MAX 100
#define BATT_CAPACITY_VALID_MIN 0
#define DEBUG_LOG_TIME 20//sjc 12->20
static int bq27541_get_battery_capacity(void)
{
	int ret;
	u8 buffer[2];
	unsigned int capacity_lv;
	int i;
	static int log_counter=0;

	log_counter++;
    /*dump fulegague capactiy 120s every time*/
	log_counter = log_counter%DEBUG_LOG_TIME;
	
	if (atomic_read(&bq27541_info->suspended) == 1) {//sjc1024
		pr_info("%s:sus, use fulegague pre capacity=[%d%%]\n", __func__, bq27541_info->batt_capacity_pre);
		return bq27541_info->batt_capacity_pre;
	}

	for(i = 0; i < 3; i++)
	{
		ret = i2c_smbus_read_i2c_block_data(bq27541_info->client,BQ27541_REG_SOC,0x02,buffer);
		if(ret >= 0)
		{
			capacity_lv = ((buffer[1] << 8) & 0xFF00);
			capacity_lv = ((unsigned int)(capacity_lv + buffer[0]) & 0x0000FFFF); 
			if(capacity_lv > BATT_CAPACITY_VALID_MAX || capacity_lv < BATT_CAPACITY_VALID_MIN){
				pr_err("%s: error invalid battery capacity\n",__func__);
				return INVALID_ERROR;
			}
			
			if(log_counter == 0)
				pr_info("%s:fulegague really capacity =[%d%%]\n",__func__,capacity_lv);
			
			return capacity_lv;
		}
	}
	//read i2c block data error
	dev_err(bq27541_info->dev,"bq27541_get_battery_capacity fail\n");
	return bq27541_info->batt_capacity_pre;
}

static int bq27541_get_battery_capacity_boot(void)
{
	unsigned int capacity_lv;
	int i;

	for(i = 0; i < 3; i++){		
		capacity_lv = bq27541_get_battery_capacity();
	
		if(capacity_lv != INVALID_ERROR){
			printk(KERN_INFO"%s:capacity_lv =[%d%%]\n",__func__,capacity_lv);
			return capacity_lv;
		}
	}
	capacity_lv = BATTERY_DEFAULT_CAPACITY;
	
	printk(KERN_ERR"%s:Fail to get boot capacity!\n",__func__);
	return capacity_lv;
}

/*
 *  i2c specific code
 */
static int bq27541_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

/*
 * i2c specific code
 */
static int bq27541_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27541_device_info *di)
{
	struct i2c_msg msg;
	unsigned char data[3];
	int ret;

	if (!di->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;

	msg.addr = di->client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	ret = i2c_transfer(di->client->adapter, &msg, 1);
	if (ret < 0)
		return -EIO;

	return 0;
}

static void bq27541_cntl_cmd(struct bq27541_device_info *di,
				int subcmd)
{
	bq27541_i2c_txsubcmd(BQ27541_REG_CNTL, subcmd, di);
}

static int bq27541_id_check(struct bq27541_device_info *di)
{
	int i;
	int ret=0;
	int temp = 0;
	int gague_counter=0;
	
	if(di){
		for(i = 0; i < 5; i++)
		{
			bq27541_cntl_cmd(di, BQ27541_SUBCMD_CHEM_ID); /* Retrieve Chip status */
			udelay(66);
			ret = bq27541_read(BQ27541_REG_CNTL, &temp, 0, di);

			pr_info("%s: chem id is=0x%0x\n",__func__,temp);

			if(ret >=0){
				if(temp == BQ27541_CHEM_ID_1 || temp == BQ27541_CHEM_ID_2 
						|| temp == BQ27541_CHEM_ID_3 || temp == BQ27541_CHEM_ID_4){
					gague_counter ++;

					if(gague_counter >=3){
						bq27541_present_flag = true;
						pr_info("%s:BQ27541 chem id present\n",__func__);
						return 0;
					}
				}
			}
		}
	}
	
	#ifdef NONFUELGAGUE_SUPPORT_FEATRUE   
	bq27541_present_flag = false;
	pr_info("%s:BQ27541 id not present\n",__func__);	   
	#else
	//12001 project only suppory fuelgague battery
	bq27541_present_flag = true;
	#endif

	return 0;
}

void fuelgague_is_present_check(void)
{
	bq27541_id_check(bq27541_info);
}

 bool fuelgague_is_present(void)
{
      return bq27541_present_flag;
}

bool fuelgague_battery_is_present(void)
{
  	int ret;
	u8 buffer[2];
	int i;

	if(fuelgague_is_present()){
	       //Read i2c failed more than 5 times,then think battery is remove
		for(i = 0; i < 5; i++)
		{
			ret = i2c_smbus_read_i2c_block_data(bq27541_info->client,BQ27541_REG_VOLT,0x02,buffer);
			if(ret >= 0){
			       return true;
			}
		}

		pr_err("%s: Warning fuelgague battery is not present\n",__func__);
		return false;
	}else{
	       return true;
	}
}

 /**
 * sjc 2013-07-26 modify 
 * return battery voltage or error(-1)
 **/
static int get_battery_voltage_fuelgague(void)
{
       int batt_vol;
	if(fuelgague_is_present()){
		batt_vol = bq27541_get_battery_voltage();
	}else{
	      batt_vol = -1;//adc_get_battery_mvolts();
	}
	gague_debug("%s:voltage value =%dmv\n",__func__, batt_vol);

	return batt_vol;
}

/**
 * sjc 2013-07-26 modify 
 * return battery temperature or error(-1000)
 **/
static int get_battery_temperature_fuelgague(void)
{
	int vbatt_temp;
	
	if(fuelgague_is_present()){
		vbatt_temp = bq27541_get_battery_temperature();
		
		/* check battery temperature too high to shut down */
		if(vbatt_temp > BATTERY_SHUTDOWN_TEMP){
			pr_info("%s:Warning battery temp too high shutdown  TEMP =%d C\n",__func__,vbatt_temp);
			bq27541_info->HighTempShutdownCount ++;
			/*25seconds  battery temp check*/
			if(bq27541_info->HighTempShutdownCount < 5)
			/*we only report high temp shutdown when 25s(5times) to check battery temperature > 68 degress C*/
				vbatt_temp = BATTERY_SHUTDOWN_TEMP -30;
			else
				bq27541_info->HighTempShutdownCount = 0;
			
			pr_info("%s:msm_chg.HighTempShutdownCount = %d\n",__func__,bq27541_info->HighTempShutdownCount);
		}else{
			bq27541_info->HighTempShutdownCount = 0;
		}
	}else{
	       vbatt_temp = -1000;//BATTERY_DEFAULT_TEMP;
	}

	return vbatt_temp;
}

/**
 * sjc 2013-07-26 modify 
 * return charging current or error(-10000)
 **/
static int get_ichg_current_fuelgague(void)
{
     	int ichg_mvolts;
	
	if(fuelgague_is_present()){
		ichg_mvolts = bq27541_get_ichg_current();
	}else{
	       ichg_mvolts = -10000;//adc_get_chg_current();
	}
	gague_debug("%s:ichg current is =%dmA\n",__func__,ichg_mvolts);
	
	return ichg_mvolts;	
}

#define VCHG_VOLTAGE_MAX 6000
#define VCHG_VOLTAGE_MIN 0
#if 0//sjc
static int get_vchg_voltage_fuelgague(void)
{
     int vchg_mvolts;

	 vchg_mvolts = adc_get_vchg_mvolts();
	 if(vchg_mvolts<VCHG_VOLTAGE_MIN)
	 	vchg_mvolts = VCHG_VOLTAGE_MIN;
	 else if(vchg_mvolts > VCHG_VOLTAGE_MAX)
	 	vchg_mvolts = VCHG_VOLTAGE_MAX;
        gague_debug("%s:vchg_mvolts =%dmv\n",__func__,vchg_mvolts);
	 return vchg_mvolts;
	
}
#endif
#define BATTERY_CAPACITY_CALIB
#ifdef BATTERY_CAPACITY_CALIB
#define BATTERY_SHUTDOWN_MVOLTS 3450
static bool battery_shutdown_mvolts_condition(void)
{
	int i=0;	
	int vbatt=3500;
	
	for(i=0;i<3;i++){
           vbatt = get_battery_voltage_fuelgague();
	     if(vbatt >= BATTERY_SHUTDOWN_MVOLTS)
		 	return false;
	}

	//check battery mvolts low than 3450mv
	return true;
}
#endif

/**
 * sjc 2013-07-26 modify 
 * return battery capacity or error(-1)
 **/
static int get_battery_capacity_fuelgague(void)
{
    int batt_capacity;
	int batt_capacity_calib;
	#ifdef BATTERY_CAPACITY_CALIB
	bool shutdown_condition=false;
	#endif	

	if(fuelgague_is_present()){
		batt_capacity = bq27541_get_battery_capacity();

		if(batt_capacity == INVALID_ERROR){
			return bq27541_info->batt_capacity_pre;
		}

		batt_capacity_calib = bq27541_capacity_calibrate(batt_capacity);
		//pr_debug("%s:calibrate capacity =[%d%%]\n",__func__,batt_capacity_calib);
		
		#ifdef BATTERY_CAPACITY_CALIB
		/*shutdown condition :capacity 0% and the battery must low than 3450mv,make sure bq27541 gague 0% capacity battery 
		   voltage low than 3450mv
		*/
		if(batt_capacity_calib == 0){
			shutdown_condition = battery_shutdown_mvolts_condition();
			if(!shutdown_condition){
				/*vbatt not satisfy shutdown condition vbatt>=3450mv*/
				pr_info("%s:vbatt not satisfy shutdown condition!!,capacity_pre=%d%%\n",__func__,bq27541_info->batt_capacity_pre);
				return bq27541_info->batt_capacity_pre;
			}
		}
		#endif
		
		bq27541_info->batt_capacity_pre = batt_capacity_calib;
	}else{
	       return -1;//return 30;
	}

	return bq27541_info->batt_capacity_pre;
	
}

static int set_battery_full_capacity(void)
{
	if(fuelgague_is_present()){
		bq27541_info->batt_capacity_pre = BATT_CAPACITY_FULL;
		return bq27541_info->batt_capacity_pre;
	}
	else {
		return -ENODEV;
	}
}

/**
 * sjc 2013-08-26 add for New Demand
 * return battery soh or error(-1)
 **/
static int bq27541_get_battery_soh(void)
{
	int i;
	int ret = 0;
	int soh = 0;

	if (atomic_read(&bq27541_info->suspended) == 1) {//sjc1024
		pr_info("%s:sus, use fulegague pre soh=[%d]\n", __func__, bq27541_info->soh_pre);
		return bq27541_info->soh_pre;
	}

	for (i = 0; i < 3; i++) {
		ret = bq27541_read(BQ27541_REG_SOH, &soh, 0, bq27541_info);
		if (ret == 0) {
			bq27541_info->soh_pre = soh;//sjc1024
			return soh;
		}
	}
	
	dev_err(bq27541_info->dev, "%s:reading SOH error, ret=%d\n", __func__, ret);
	
	return INVALID_ERROR;
}

/**
 * sjc 2013-08-26 add for New Demand
 * return battery fcc or error(-1)
 **/
static int bq27541_get_battery_fcc(void)
{
	int i;
	int ret = 0;
	int fcc = 0;

	if (atomic_read(&bq27541_info->suspended) == 1) {//sjc1024
		pr_info("%s:sus, use fulegague pre fcc=[%d]\n", __func__, bq27541_info->fcc_pre);
		return bq27541_info->fcc_pre;
	}

	for (i = 0; i < 3; i++) {
		ret = bq27541_read(BQ27541_REG_FCC, &fcc, 0, bq27541_info);
		if (ret == 0) {
			bq27541_info->fcc_pre = fcc;//sjc1024
			return fcc;
		}
	}

	dev_err(bq27541_info->dev, "%s:reading FCC error, ret=%d\n", __func__, ret);
	
	return INVALID_ERROR;
}

/**
 * sjc 2013-08-26 add for New Demand
 * return battery soh or error(-1)
 **/
static int get_battery_soh_fuelgague(void)
{
	int batt_soh;
	
	if (fuelgague_is_present()) {
		batt_soh = bq27541_get_battery_soh();
	} else {
		batt_soh = INVALID_ERROR;
	}
	gague_debug("%s:battery SOH=%d\n",__func__, batt_soh);
	
	return batt_soh;
}

/**
 * sjc 2013-08-26 add for New Demand
 * return battery fcc or error(-1)
 **/
static int get_battery_fcc_fuelgague(void)
{
	int batt_fcc;
	
	if (fuelgague_is_present()) {
		batt_fcc = bq27541_get_battery_fcc();
	} else {
		batt_fcc = INVALID_ERROR;
	}
	gague_debug("%s:battery FCC=%d\n",__func__, batt_fcc);
	
	return batt_fcc;
}

/*fuelgague function*/
static struct oppo_battery_fuelgauge bq27541_batt_fuelgauge = {
	.get_battery_capacity		= get_battery_capacity_fuelgague,
	.get_battery_mvolts		= get_battery_voltage_fuelgague,
	.get_battery_temperature	= get_battery_temperature_fuelgague,
	.get_battery_soh			= get_battery_soh_fuelgague,
	.get_battery_fcc			= get_battery_fcc_fuelgague,
	//sjc .get_vchg_mvolts		= get_vchg_voltage_fuelgague,
	.get_ichg_current			= get_ichg_current_fuelgague,
	.set_full_capacity			= set_battery_full_capacity,
};

static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{

	struct bq27541_access_methods *bus;
	int err = 0;
	int retval = 0;

	printk(KERN_INFO "%s\n", __func__);
	//printk(KERN_INFO "%s, i2c_client address=0x%x\n", __func__, client->addr);
	//return 0;

	if (i2c_check_functionality(client->adapter, I2C_FUNC_I2C) == 0) {
		dev_err(bq27541_info->dev, "BQ27541 i2c_probe I2C_FUNC_I2C Failed\n");
		err = -ENODEV;
		return err;
      }

	bq27541_info = kzalloc(sizeof(*bq27541_info), GFP_KERNEL);
	if (!bq27541_info) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	i2c_set_clientdata(client, bq27541_info);
	
	bq27541_info->dev = &client->dev;
	bus->read = &bq27541_read_i2c;
	bq27541_info->bus = bus;
	bq27541_info->client = client;

	bq27541_info->batt_present_counter = 0;
	bq27541_info->batt_is_present = 1;
	bq27541_info->capacity_saltate_counter = 0;
	bq27541_info->HighTempShutdownCount = 0;
	
	bq27541_info->batt_capacity_pre = BATTERY_DEFAULT_CAPACITY;
	bq27541_info->batt_temperate_pre = BATTERY_DEFAULT_TEMP;

	bq27541_info->batt_capacity_pre = bq27541_get_battery_capacity_boot();
	/*update vbatt mvolts*/
	bq27541_get_battery_voltage();
	
	//check bq27541 gague if present now
	bq27541_id_check(bq27541_info);

	smb358_battery_fuelgauge_register(&bq27541_batt_fuelgauge);

	bq27541_info->temp_pre = BATTERY_DEFAULT_TEMP;//sjc1024
	bq27541_info->soh_pre = 100;
	bq27541_info->fcc_pre = 0;
	bq27541_info->batt_cur_pre = 0;
	atomic_set(&bq27541_info->suspended, 0);

	pr_info("%s:update vchg mvolts boot stegs\n",__func__);	
	//sjc smb346_vchg_mvolts_update();
	//pr_info("====%s:sjc test oppo_battery_fuelgauge inferface begin====\n\n",__func__);
	//get_battery_capacity_fuelgague();
	//get_battery_voltage_fuelgague();
	//get_battery_temperature_fuelgague();
	//get_ichg_current_fuelgague();
	//pr_info("\n====%s:sjc test oppo_battery_fuelgauge inferface end====\n",__func__);
	//printk(KERN_INFO "\n===%s:FCC=%dmAh===\n\n", __func__, get_battery_fcc_fuelgague());
	//printk(KERN_INFO "\n===%s:SOH=%d%%===\n\n", __func__, get_battery_soh_fuelgague());
	return 0;

batt_failed_2:
	kfree(bq27541_info);
	kfree(bus);
	return retval;
batt_failed_1:
	return retval;

}

static int bq27541_battery_remove(struct i2c_client *client)
{
	pr_info("%s: battery gague driver remove\n",__func__);
	kfree(bq27541_info->bus);
	kfree(bq27541_info);
	/*unregister batt gauge to msm charger*/
	smb358_battery_fuelgauge_unregister(&bq27541_batt_fuelgauge);
	return 0;
}

static int bq27541_update_capacity_pre_resume(void)
{
    int i=0;
	int capacity=0;
	
	for(i=0;i<3;i++){
		capacity = bq27541_get_battery_capacity(); 
		if(capacity == INVALID_ERROR){
			pr_err("%s:read capacity failed!\n",__func__);
			continue;
	    }
		
		if(capacity <= bq27541_info->batt_capacity_pre)
		{
		    /*update batt_capacity_pre*/
		    bq27541_info->batt_capacity_pre = capacity;
	        pr_info("read the pre_capacity=[%d%%]\n",capacity);
			return 0;
			
		}else{
		    pr_err("%s: err resume read fuelgague capacity failed!%d%%,%d%%\n",
				 __func__, bq27541_info->batt_capacity_pre,capacity);
			continue;
		}
	}
	
	return 0;

}

extern int msmrtc_alarm_read_time(struct rtc_time *tm);
static int bq27541_battery_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	int ret=0;
	struct rtc_time     rtc_suspend_rtc_time;
	
	atomic_set(&bq27541_info->suspended, 1);//sjc1024
	msmrtc_alarm_read_time(&rtc_suspend_rtc_time);
	if (ret < 0) {
		pr_err("%s: Failed to read RTC time\n", __func__);
		return 0;
	}
	rtc_tm_to_time(&rtc_suspend_rtc_time, &bq27541_info->rtc_suspend_time);

    //pr_info("bq27541_battery_i2c_suspend %lds\n",bq27541_info->rtc_suspend_time);
	
	return 0;
}

/*1 minute*/
#define RESUME_TIME  1*60 
static int bq27541_battery_i2c_resume(struct i2c_client *client)
{
	int ret=0;
	struct rtc_time     rtc_resume_rtc_time;
		
	//sjc pr_info("enter:bq27541_resume\n");
	
	atomic_set(&bq27541_info->suspended, 0);//sjc1024
	msmrtc_alarm_read_time(&rtc_resume_rtc_time);
	if (ret < 0) {
		pr_err("%s: Failed to read RTC time\n", __func__);
		return 0;
	}
	rtc_tm_to_time(&rtc_resume_rtc_time, &bq27541_info->rtc_resume_time);

	//sjc pr_info("exist:bq27541_resume %ld,%ld\n",bq27541_info->rtc_suspend_time,bq27541_info->rtc_resume_time);
	
	if((bq27541_info->rtc_resume_time - bq27541_info->rtc_suspend_time)>= RESUME_TIME){
		/*update pre capacity when sleep time more than 1minutes*/
		bq27541_update_capacity_pre_resume();
	}
		
	//sjc updata_batt_params_fulegague_resume();

	return 0;
}

/*
 * Module stuff
 */
static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541-gauge", 0 },
	{},
};

static struct i2c_driver bq27541_battery_driver = {
	.driver = {
		.name = "bq27541-gauge",
		.owner	= THIS_MODULE,
	},
	.probe = bq27541_battery_probe,
	.remove = bq27541_battery_remove,
	.suspend = bq27541_battery_i2c_suspend ,
	.resume  = bq27541_battery_i2c_resume,
	.id_table = bq27541_id,
};

static int __init bq27541_battery_init(void)
{
	int ret;

	if (get_pcb_version() < PCB_VERSION_EVT_N1)
		return 0;
	
	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27541 gauge driver\n");
	return ret;
}

module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	if (get_pcb_version() < PCB_VERSION_EVT_N1)
		return;
	
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_AUTHOR("OPPO");
MODULE_DESCRIPTION("BQ27541 battery fuelgauge driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1.0");
