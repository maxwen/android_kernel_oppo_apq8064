/************************************************************************************
** drivers/input/touchscreen/synaptics_s3202_i2c_rmi.c
**
** Copyright (C), 2000-2012, OPPO Mobile Comm Corp., Ltd
** All rights reserved.
** 
** VENDOR_EDIT
** 
** Description: -
** 
** from
** drivers/input/touchscreen/synaptics_i2c_rmi.c
**
** Copyright (C) 2007 Google, Inc.
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
** 
** 
** --------------------------- Revision History: --------------------------------
** <author>		                      <data> 	<version >  <desc>
** ------------------------------------------------------------------------------
** LiuJun@OnlineRD.Driver.TouchScreen  2012/11/14   1.0	    create file
** max.weninger@gmail.com			   2013/06/22   1.0     added S2W
** max.weninger@gmail.com			   2013/07/26   1.0     added DT2W
** ------------------------------------------------------------------------------
** 
************************************************************************************/


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/wakelock.h>
#include <mach/board.h>

/******************* tp function switch **************************/
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_ENABLE_FW_FLASH
#define TP_UPDATE_FIRMWARE  1
#else
#define TP_UPDATE_FIRMWARE  0
#endif
#define SUPPORT_DOUBLE_TAP  0

#define TOUCHSCREEN_SYNAPTICS_WINTEK_FW Syna_Firmware_Data_Wintek
#define TOUCHSCREEN_SYNAPTICS_WINTEK_FW_OLD Syna_Firmware_Data_Wintek_Old
/*****************************************************************/

#include "synaptics_firmware_truly.h"
#include "synaptics_firmware_wintek.h"
#include "synaptics_firmware_wintek-old.h"
#include "synaptics_firmware_tpk.h"
#define TP_UPDATE_RLE_FILE	"tpupdate.rle"
extern int display_rle_file(char *filename);

#include "synaptics_test_rawdata.h"

#define MAX_FINGERS		(10)
#define FINGER_PACKAGE_SIZE		(5)

#define MASK_16BIT		0xFFFF
#define MASK_8BIT		0xFF
#define MASK_5BIT		0x1F
#define MASK_4BIT		0x0F
#define MASK_3BIT		0x07
#define MASK_1BIT		0x01
#define MAX_RETRY_COUNT		5

/*************** tp register definition **************************/
//#define REG_MAP_START	0x00DD
//#define REG_MAP_END		0x00EF
//#define REG_MAP_NUM		(REG_MAP_END - REG_MAP_START)
#define PDT_START_SCAN_LOCATION (0x00E9)
#define PDT_END_SCAN_LOCATION	(0x000A)
#define PDT_ENTRY_SIZE			(0x0006)

#define PAGE_SELECT		0x00ff
//#define PAGE_LEN		2

#define FN01_QUERY_INFO_BUF_LEN		21
// define offset to F01 query base address
#define OFFSET_MANUFACTURER_ID		(0)
#define OFFSET_PRODUCT_PROPERTY		(1)
#define OFFSET_FAMILY				(2)
#define OFFSET_FIRMWARE_REVISION	(3)
#define OFFSET_FIRMWARE_DATE		(4)
#define OFFSET_PRODUCT_ID			(11)

// For F01 Ctrl
#define F01_CTRL_BASE_ADDR		(syna_ts_data->fn01_desc.ctrl_base_addr)
#define F01_CTRL_DEVICE_CONTROL	(F01_CTRL_BASE_ADDR + 0)
#define F01_CTRL_INT_ENABLE		(F01_CTRL_BASE_ADDR + 1)

// For F01 Cmd
#define F01_CMD_BASE_ADDR		(syna_ts_data->fn01_desc.cmd_base_addr)
#define F01_CMD_DEVICE_CMD		(F01_CMD_BASE_ADDR + 0)

// For F01 Data
#define F01_DATA_BASE_ADDR		(syna_ts_data->fn01_desc.data_base_addr)
#define F01_DATA_DEVICE_STATUS	(F01_DATA_BASE_ADDR + 0)
#define F01_DATA_INT_STATUS		(F01_DATA_BASE_ADDR + 1)

// For F11 Ctrl
#define F11_CTRL_BASE_ADDR		(syna_ts_data->fn11_desc.ctrl_base_addr)
#define F11_CTRL_REPORT_MODE	(F11_CTRL_BASE_ADDR + 0)
#define F11_CTRL_DELTA_X_THRESH	(F11_CTRL_BASE_ADDR + 2)
#define F11_CTRL_DELTA_Y_THRESH	(F11_CTRL_BASE_ADDR + 3)
#define F11_CTRL_MAX_X			(F11_CTRL_BASE_ADDR + 6)
#define F11_CTRL_MAX_Y			(F11_CTRL_BASE_ADDR + 8)
#define F11_CTRL_32_00		(F11_CTRL_BASE_ADDR + 15)
#define F11_CTRL_32_01			(F11_CTRL_BASE_ADDR + 16)
#define F11_CTRL_58			(F11_CTRL_BASE_ADDR + 41)

// For F11 Data
#define F11_DATA_BASE_ADDR		(syna_ts_data->fn11_desc.data_base_addr)
#define F11_DATA_LPWG_STATUS	(F11_DATA_BASE_ADDR + 67)	//data38

// For F34 Ctrl
#define F34_CTRL_BASE_ADDR		(syna_ts_data->fn34_desc.ctrl_base_addr)
#define F34_CTRL_CUSTOMER_ID_0	(F34_CTRL_BASE_ADDR + 0)
#define F34_CTRL_CUSTOMER_ID_1	(F34_CTRL_BASE_ADDR + 1)
#define F34_CTRL_CUSTOMER_ID_2	(F34_CTRL_BASE_ADDR + 2)
#define F34_CTRL_CUSTOMER_ID_3	(F34_CTRL_BASE_ADDR + 3)

// For F54
#define F54_CTRL_BASE_ADDR		(syna_ts_data->fn54_desc.ctrl_base_addr)
#define F54_CMD_BASE_ADDR		(syna_ts_data->fn54_desc.cmd_base_addr)
#define F54_DATA_BASE_ADDR		(syna_ts_data->fn54_desc.data_base_addr)

#define F54_CTRL_02_00	(F54_CTRL_BASE_ADDR + 2)
#define F54_CTRL_02_01	(F54_CTRL_BASE_ADDR + 3)

/*****************************************************************/
/*************** log definition **********************************/
#define TS_ERROR   1
#define TS_WARNING 2
#define TS_INFO    3
#define TS_DEBUG   4
#define TS_TRACE   5
static int syna_log_level = TS_WARNING;
#define print_ts(level, ...) \
	do { \
		if (syna_log_level >= (level)) \
			printk("[syna] " __VA_ARGS__); \
	} while (0) 
/*****************************************************************/
/*************** tp vendor id definition *************************/
#define TP_VENDOR_WINTEK	1	//胜华
#define TP_VENDOR_TPK		2	//TPK
#define TP_VENDOR_TRULY		3	//信利
/*****************************************************************/
/*************** tp command definition *************************/
#define TP_CMD_DO_NOTHING	0
#define TP_CMD_RESET		1
#define TP_CMD_REZERO		2
#define TP_CMD_FORCE_UPDATE	90
#define TP_CMD_UPDATE_FROM_FILE	91
/*****************************************************************/

/* Filter the first point, added by Eric for solve the bug: click action will change to glide action */
static int input_point_num = 0;

static struct workqueue_struct *synaptics_wq;

struct synaptics_rmi4_fn_desc {
	unsigned short	query_base_addr;
	unsigned short	cmd_base_addr;
	unsigned short	ctrl_base_addr;
	unsigned short	data_base_addr;
	unsigned char	intr_src_count;
	unsigned char	fn_number;
};

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	bool has_relative_report;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	uint32_t flags;
	int reported_finger_count;
	int8_t sensitivity_adjust;
	uint8_t cmd;
	uint8_t is_tp_suspended;
	int (*power)(int on);
	struct early_suspend early_suspend;

	uint8_t	need_hardware_reset;
	uint8_t report_mode;
	uint8_t deltx;
	uint8_t delty;
	uint8_t finger_data[((MAX_FINGERS+3)/4)+(MAX_FINGERS*5)+1];
	uint16_t vendor_id;
	uint8_t version[4];
	uint16_t snap_left;
	uint16_t snap_right;
	uint16_t snap_top;
	uint32_t virtual_key_height;
	uint32_t vk_prop_center_y;
	uint32_t vk_prop_height;
	uint32_t vk_prop_width;
	struct delayed_work delay_work;
	struct kobject *properties_kobj;
	struct mutex mutex_set_page;
	int  current_page;
	struct synaptics_rmi4_fn_desc fn01_desc;
	struct synaptics_rmi4_fn_desc fn11_desc;
	struct synaptics_rmi4_fn_desc fn34_desc;
	struct synaptics_rmi4_fn_desc fn54_desc;
	wait_queue_head_t  wait_i2c_ready;
	int i2c_ready;
	struct wake_lock        double_wake_lock;
	struct early_suspend early_suspend_power;
#if SUPPORT_DOUBLE_TAP
	atomic_t double_tap_number;
	atomic_t double_tap_enable;
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
	bool s2w_enabled;
	unsigned int s2w_register_threshold;
	unsigned int s2w_min_distance;
	bool s2w_allow_stroke;
	unsigned int s2w_barrier_y;
	bool dt2w_enabled;
	unsigned int dt2w_duration;
	unsigned int dt2w_threshold;
	unsigned int dt2w_barrier_y;
#endif
};

static struct synaptics_ts_data   *syna_ts_data;
static DEFINE_SEMAPHORE(synaptics_sem);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
static void synaptics_ts_late_resume_power(struct early_suspend *h);
#endif

extern void CompleteReflash(struct i2c_client *client, const unsigned char* firmware_data);
static int synaptics_i2c_block_read(struct synaptics_ts_data *ts,
					unsigned short address, int size, unsigned char *buf);
static int synaptics_i2c_byte_write(struct synaptics_ts_data *ts,
						unsigned short address,	unsigned char data);
static int synaptics_init_panel(struct synaptics_ts_data *ts);
static int synaptics_set_int_mask(struct synaptics_ts_data *ts, int enable);
static int synaptics_set_report_mode(struct synaptics_ts_data *ts, uint8_t set_mode);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
static bool s2w_barrier_reached = false;
static bool s2w_exec_power_press = true;
// -1 = not touched; -2 = touched on screen; >=0 = touched on button panel
static unsigned int s2w_down_x = -1;
static u64 dt2w_double_tap_start;

static struct input_dev * s2w_pwrdev = NULL;
static DEFINE_MUTEX(pwrkeyworklock);

static struct kobject *android_touch_kobj;

void synaptics_s2w_setdev(struct input_dev * input_device) 
{
	print_ts(TS_INFO, KERN_INFO "set s2w_pwrdev=%s\n", input_device->name);
	s2w_pwrdev = input_device;
}

static void s2w_presspwr(struct work_struct * s2w_presspwr_work) 
{
	if (s2w_pwrdev == NULL){
		print_ts(TS_ERROR, KERN_ERR "s2w_pwrdev not set\n");
		return;
	}

	if (!mutex_trylock(&pwrkeyworklock)){
        return;
    }
	
	print_ts(TS_INFO, KERN_INFO "simulate power key pressed\n");

	input_event(s2w_pwrdev, EV_KEY, KEY_POWER, 1);
	input_event(s2w_pwrdev, EV_SYN, 0, 0);
	msleep(100);
	input_event(s2w_pwrdev, EV_KEY, KEY_POWER, 0);
	input_event(s2w_pwrdev, EV_SYN, 0, 0);
	msleep(100);
    mutex_unlock(&pwrkeyworklock);
}

static DECLARE_WORK(s2w_presspwr_work, s2w_presspwr);

static void simulate_power_press(void)
{
	schedule_work(&s2w_presspwr_work);
}

static bool synaptics_s2w_handle_move(struct synaptics_ts_data* ts, int finger_data_x)
{
	// lock panel to s2w after this distance
	if(abs(s2w_down_x - finger_data_x) > ts->s2w_register_threshold)
    	s2w_barrier_reached = true;
                      
	// handle after distance travelled
	if(abs(s2w_down_x - finger_data_x) > ts->s2w_min_distance)
		return true;

	return false;
}

static bool input_wakeup_active(struct synaptics_ts_data* ts)
{
	return ts->s2w_enabled || ts->dt2w_enabled;
}

#endif

/***** For device sysfs attributs interfaces begin ********/
static int syna_test_max_err_count = 10;
static inline void wait_test_cmd_finished(void)
{
	uint8_t data = 0;
	do {
		mdelay(1); //wait 1ms
		synaptics_i2c_block_read(syna_ts_data, F54_CMD_BASE_ADDR, 1, &data);
	} while (data != 0x00);
}
static ssize_t tp_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int x, y, i;
	int16_t read_data;
	uint8_t tmp_old = 0;
	uint8_t tmp_new = 0;
	ssize_t num_read_chars = 0;
	uint tx_num = 0;
	uint rx_num = 0;
	uint rx2rx_lower_limit = 0;
	uint rx2rx_upper_limit = 0;
	const int16_t *raw_cap_data = NULL;
//	int16_t Rxdata[RX_NUM][RX_NUM];
	int16_t *Rxdata = NULL;
	int error_count = 0;
	struct i2c_client* client = syna_ts_data->client;

	down(&synaptics_sem);
	if (syna_ts_data->is_tp_suspended)
	{
		up(&synaptics_sem);
		return sprintf(&(buf[num_read_chars]), "-1 tp suspended.\n");
	}

	disable_irq_nosync(client->irq);

	if (syna_ts_data->vendor_id == TP_VENDOR_TRULY)
	{
		tx_num = TX_NUM_TRULY;
		rx_num = RX_NUM_TRULY;
		rx2rx_lower_limit = DiagonalLowerLimit_TRULY;
		rx2rx_upper_limit = DiagonalUpperLimit_TRULY;
		raw_cap_data = (const int16_t *)raw_cap_data_truly_3035;
	}
	else if (syna_ts_data->vendor_id == TP_VENDOR_WINTEK)
	{
		tx_num = TX_NUM_WINTEK;
		rx_num = RX_NUM_WINTEK;
		rx2rx_lower_limit = DiagonalLowerLimit_WINTEK;
		rx2rx_upper_limit = DiagonalUpperLimit_WINTEK;
		raw_cap_data = (const int16_t *)raw_cap_data_wintek_9093;
	}
	else if (syna_ts_data->vendor_id == TP_VENDOR_TPK)
	{
		tx_num = TX_NUM_TPK;
		rx_num = RX_NUM_TPK;
		rx2rx_lower_limit = DiagonalLowerLimit_TPK;
		rx2rx_upper_limit = DiagonalUpperLimit_TPK;
		raw_cap_data = (const int16_t *)raw_cap_data_tpk;
	}
	if (tx_num == 0 || rx_num == 0 || raw_cap_data == NULL
		|| rx2rx_lower_limit == 0 || rx2rx_upper_limit == 0 || tx_num > rx_num)
	{
		num_read_chars += sprintf(&(buf[num_read_chars]), "%2d++ NO appropriate data for current tp.\n", ++error_count);
		goto END_TP_TEST;
	}

	print_ts(TS_DEBUG, "alloc test memory.\n");
	Rxdata = kmalloc(sizeof(int16_t)*rx_num*rx_num, GFP_KERNEL);
	if (Rxdata == NULL)
	{
		num_read_chars += sprintf(&(buf[num_read_chars]), "%2d++ alloc memory failed.\n", ++error_count);
		goto END_TP_TEST;
	}
	memset(Rxdata, 0, sizeof(Rxdata));

	synaptics_set_int_mask(syna_ts_data, 0);
	//ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x01);

//step 1:check raw capacitance
	print_ts(TS_DEBUG, "Step 1 : Raw Capacitance Report \n");
	//set report type
	//ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x03);
	synaptics_i2c_byte_write(syna_ts_data, F54_DATA_BASE_ADDR, 0x03); //set report type 0x03

	// forbid CBC
	synaptics_i2c_block_read(syna_ts_data, F54_CTRL_BASE_ADDR +8, 1, &tmp_old);
	//ret = i2c_smbus_read_byte_data(ts_g->client, F54_CTRL_BASE_ADDR +8);
	//tmp_old = ret & 0xff;
	tmp_new = tmp_old & 0xef;
	print_ts(TS_DEBUG, "tmp_old =0x%x ,tmp_new = 0x%x\n", tmp_old, tmp_new);
	//ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,tmp_new);
	//ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	synaptics_i2c_byte_write(syna_ts_data, F54_CTRL_BASE_ADDR +8, tmp_new);
	synaptics_i2c_byte_write(syna_ts_data, F54_CMD_BASE_ADDR, 0x04); // force update
	wait_test_cmd_finished();
	print_ts(TS_DEBUG, "Forbid CBC oK\n");
	// Forbid NoiseMitigation
	//ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE + 81,0X01);  //Forbid NoiseMitigation F54_ANALOG_CTRL41
	//ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X04);//force F54_ANALOG_CMD00
	synaptics_i2c_byte_write(syna_ts_data, F54_CTRL_BASE_ADDR +81, 0x01);
	synaptics_i2c_byte_write(syna_ts_data, F54_CMD_BASE_ADDR, 0x04); //force update
	wait_test_cmd_finished();
	print_ts(TS_DEBUG, "Forbid NoiseMitigation oK\n");
	//ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);//Force Cal, F54_ANALOG_CMD00
	synaptics_i2c_byte_write(syna_ts_data, F54_CMD_BASE_ADDR, 0x02);
	wait_test_cmd_finished();
	print_ts(TS_DEBUG, "Force Cal oK\n");

	ret = i2c_smbus_write_word_data(client, F54_DATA_BASE_ADDR+1,0x00);//set fifo 00
	//ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
	synaptics_i2c_byte_write(syna_ts_data, F54_CMD_BASE_ADDR, 0x01);
	wait_test_cmd_finished();

	for(x = 0;x < tx_num; x++)
	{
		i = 0;
		for(y = 0; y < rx_num; y++)
		{
			ret = i2c_smbus_read_word_data(client,F54_DATA_BASE_ADDR+3);
			read_data = ret & 0xffff;

			print_ts(TS_DEBUG, "Raw[%d][%d] = %d,  ", x, y, read_data);
			print_ts(TS_DEBUG, "range:[%d ~ %d], ", raw_cap_data[x*rx_num*2+i], raw_cap_data[x*rx_num*2+i+1]);
			if (read_data >= raw_cap_data[x*rx_num*2+i] && read_data <= raw_cap_data[x*rx_num*2+i+1])
			{
				print_ts(TS_DEBUG, "pass.\n");
			}
			else
			{
				print_ts(TS_DEBUG, "NOT in range!!\n");
				num_read_chars += sprintf(&(buf[num_read_chars]), "%2d++ raw_cap[%02d][%02d]=%4d is not in range[%04d~%04d].\n",
									++error_count,x,y,read_data,raw_cap_data[x*rx_num*2+i],raw_cap_data[x*rx_num*2+i+1]);
				if (error_count >= syna_test_max_err_count)
				{
					num_read_chars += sprintf(&(buf[num_read_chars]), "  == Reach max error count (%d), stop test.\n", syna_test_max_err_count);
					goto END_TP_TEST;
				}
			}
			i+=2;
		}
		print_ts(TS_DEBUG, "------------------------------\n");
	}
	ret = i2c_smbus_write_byte_data(client, F54_CTRL_BASE_ADDR+8, tmp_old);
	ret = i2c_smbus_write_byte_data(client, F54_CMD_BASE_ADDR, 0x04); // force update
	wait_test_cmd_finished();

//step 2 :check tx-to-tx and tx-to-vdd
	print_ts(TS_DEBUG, "Step 2 : Check tx-to-tx  \n");
	ret = i2c_smbus_write_byte_data(client,F54_DATA_BASE_ADDR,0x05);//select report type 0x05
	ret = i2c_smbus_write_byte_data(client,F54_CMD_BASE_ADDR,0X01);//get report
	wait_test_cmd_finished();
	ret = i2c_smbus_write_word_data(client,F54_DATA_BASE_ADDR+1,0);
	ret = i2c_smbus_read_word_data(client,F54_DATA_BASE_ADDR+3);
	if( (ret&0x7ff) != 0)
	{
		print_ts(TS_DEBUG, "Step 2 error.\n");
		num_read_chars += sprintf(&(buf[num_read_chars]), "%2d++ tx-tx-short or tx-vdd-short data wrong.\n",++error_count);
		goto END_TP_TEST;
	}

//step 3 :check rx-to-rx
	print_ts(TS_DEBUG, "Step 3 : Read rx-rx  \n");
	ret = i2c_smbus_write_byte_data(client,F54_DATA_BASE_ADDR,7);//select report type 0x07
	ret = i2c_smbus_write_byte_data(client,F54_CMD_BASE_ADDR,0X01);//get report
	wait_test_cmd_finished();
	ret = i2c_smbus_write_word_data(client,F54_DATA_BASE_ADDR+1,0);
	for(x = 0;x < tx_num; x++)
	{
		for(y = 0; y < rx_num; y++)
		{
			//Rxdata[x][y] = i2c_smbus_read_word_data(client,F54_DATA_BASE_ADDR+3);
			Rxdata[x*rx_num+y] = i2c_smbus_read_word_data(client,F54_DATA_BASE_ADDR+3);     
		}
	}
	ret = i2c_smbus_write_byte_data(client,F54_DATA_BASE_ADDR,17);//select report type 0x17 
	ret = i2c_smbus_write_word_data(client,F54_DATA_BASE_ADDR+1,0);
	ret = i2c_smbus_write_byte_data(client,F54_CMD_BASE_ADDR,0X01);//get report
	wait_test_cmd_finished();
	for(x = 0; x < rx_num - tx_num; x++)
	{
		for(y = 0; y < rx_num; y++)
		{
			//Rxdata[ x + TX_NUM][y] = i2c_smbus_read_word_data(client,F54_DATA_BASE_ADDR+3);
			Rxdata[(x+tx_num)*rx_num+y] = i2c_smbus_read_word_data(client,F54_DATA_BASE_ADDR+3);
		}
	}
	
//step 4:check rx-rx short
	print_ts(TS_DEBUG, "Step 4: Check rx-rx \n rx-rx range:[%d ~ %d]\n", rx2rx_lower_limit, rx2rx_upper_limit);
	for(x = 0; x < rx_num; ++x)
	{
		read_data = Rxdata[x*rx_num+x];
		print_ts(TS_DEBUG, "rx[%d] = %d, ", x, read_data);
		if(read_data < rx2rx_lower_limit || read_data > rx2rx_upper_limit)
		{
			print_ts(TS_DEBUG, "Not in range!!\n");
			num_read_chars += sprintf(&(buf[num_read_chars]), "%2d++ rx-to-rx short or rx-to-vdd short rx[%d][%d] = %d\n",++error_count,x,x,read_data);
			if (error_count >= syna_test_max_err_count)
			{
				num_read_chars += sprintf(&(buf[num_read_chars]), "  == Reach max error count (%d), stop test.\n", syna_test_max_err_count);
				goto END_TP_TEST;
			}
		}
		else
		{
			print_ts(TS_DEBUG, "pass.\n");
		}
	}

//step 5:reset touchpanel and reconfig the device
END_TP_TEST:
	if (Rxdata)
	{
		print_ts(TS_DEBUG, "Release test memory.\n");
		kfree(Rxdata);
	}
	print_ts(TS_DEBUG, "Tp test finish!\n");

	num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
	//ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00); 
	//ret = i2c_smbus_write_byte_data(ts_g->client,F01_RMI_CMD_BASE,0X01);
	synaptics_i2c_byte_write(syna_ts_data, F01_CMD_BASE_ADDR, 0x01);

	msleep(150);
	synaptics_init_panel(syna_ts_data);
	synaptics_set_int_mask(syna_ts_data, 1);

	enable_irq(client->irq);
	up(&synaptics_sem);
	return num_read_chars;
}
static ssize_t synaptics_attr_errorlimit_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", syna_test_max_err_count);
}
static ssize_t synaptics_attr_errorlimit_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	sscanf(buf, "%d", &syna_test_max_err_count);
	return count;
}

static ssize_t synaptics_attr_loglevel_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", syna_log_level);
}
static ssize_t synaptics_attr_loglevel_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = 0;
	sscanf(buf, "%d", &val);
	syna_log_level = val;
	printk("[synaptics] set log level : %d\n", val);
	return count;
}
static ssize_t synaptics_attr_deltx_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", syna_ts_data->deltx);
}
static ssize_t synaptics_attr_deltx_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = 0;
	sscanf(buf, "%d", &val);
	if (val != 0 && !syna_ts_data->is_tp_suspended)
	{
		print_ts(TS_INFO, "set deltx : 0x%02x\n", val);
		syna_ts_data->deltx = val;
		synaptics_i2c_byte_write(syna_ts_data, F11_CTRL_DELTA_X_THRESH, syna_ts_data->deltx);
	}
	return count;
}
static ssize_t synaptics_attr_delty_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", syna_ts_data->delty);
}
static ssize_t synaptics_attr_delty_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = 0;
	sscanf(buf, "%d", &val);
	if (val != 0 && !syna_ts_data->is_tp_suspended)
	{
		print_ts(TS_INFO, "set delty : 0x%02x\n", val);
		syna_ts_data->delty = val;
		synaptics_i2c_byte_write(syna_ts_data, F11_CTRL_DELTA_Y_THRESH, syna_ts_data->delty);
	}
	return count;
}
static ssize_t synaptics_attr_report_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%02x\n", syna_ts_data->report_mode & MASK_3BIT);
}
static ssize_t synaptics_attr_report_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = 0;
	if (syna_ts_data->is_tp_suspended == 0)
	{
		sscanf(buf, "%d", &val);
		if (val)
			syna_ts_data->report_mode = 0x01;
		else
			syna_ts_data->report_mode = 0x00;
		print_ts(TS_INFO, "set report mode : 0x%02x\n", syna_ts_data->report_mode);
		synaptics_set_report_mode(syna_ts_data, syna_ts_data->report_mode);
	}
	return count;
}
static inline ssize_t print_fn_address(char *buf, struct synaptics_rmi4_fn_desc* desc)
{
	return sprintf(buf, "fn%02x: query[%04x], cmd[%04x], ctrl[%04x], data[%04x]\n",
				desc->fn_number, desc->query_base_addr, desc->cmd_base_addr,
				desc->ctrl_base_addr,  desc->data_base_addr);
}
static ssize_t synaptics_attr_basic_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = syna_ts_data;
	ssize_t count = 0;
	count += sprintf(buf, "fw:%02x%02x%02x%02x  max:%d,%d  active:%d,%d\n",
				ts->version[0], ts->version[1], ts->version[2], ts->version[3],
				ts->max[0], ts->max[1],
				ts->max[0] - ts->snap_left - ts->snap_right,
				ts->max[1] - ts->snap_top - ts->virtual_key_height);
	count += print_fn_address(buf+count, &ts->fn01_desc);
	count += print_fn_address(buf+count, &ts->fn11_desc);
	count += print_fn_address(buf+count, &ts->fn34_desc);
	count += print_fn_address(buf+count, &ts->fn54_desc);
	return count;
}
static ssize_t synaptics_attr_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", syna_ts_data->vendor_id); 
}

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
static ssize_t synaptics_s2w_min_distance_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", syna_ts_data->s2w_min_distance);

	return count;
}

static ssize_t synaptics_s2w_min_distance_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
    int ret = 0;

	ret = strict_strtoul(buf, 10, &value);
    if (ret < 0) {
        return count;
    }
	syna_ts_data->s2w_min_distance = (unsigned int)value;
	print_ts(TS_INFO, KERN_INFO "s2w_min_distance=%d\n", syna_ts_data->s2w_min_distance);
	return count;
}

static DEVICE_ATTR(s2w_min_distance, (S_IRUGO|S_IWUGO), 
    synaptics_s2w_min_distance_show, synaptics_s2w_min_distance_dump);

static ssize_t synaptics_s2w_register_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", syna_ts_data->s2w_register_threshold);

	return count;
}

static ssize_t synaptics_s2w_register_threshold_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
    int ret = 0;

	ret = strict_strtoul(buf, 10, &value);
    if (ret < 0) {
        return count;
    }
	syna_ts_data->s2w_register_threshold = (unsigned int)value;
	print_ts(TS_INFO, KERN_INFO "s2w_register_threshold=%d\n", syna_ts_data->s2w_register_threshold);
	return count;
}

static DEVICE_ATTR(s2w_register_threshold, (S_IRUGO|S_IWUGO),
	synaptics_s2w_register_threshold_show, synaptics_s2w_register_threshold_dump);


static ssize_t synaptics_s2w_allow_stroke_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", syna_ts_data->s2w_allow_stroke);

	return count;
}

static ssize_t synaptics_s2w_allow_stroke_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
    int ret = 0;

	ret = strict_strtoul(buf, 10, &value);
    if (ret < 0) {
        return count;
    }
    if (value == 0 || value == 1) {
	    syna_ts_data->s2w_allow_stroke = (bool)value;
	    print_ts(TS_INFO, KERN_INFO "s2w_allow_stroke=%d\n", syna_ts_data->s2w_allow_stroke);
    }
	return count;
}

static DEVICE_ATTR(s2w_allow_stroke, (S_IRUGO|S_IWUGO),
	synaptics_s2w_allow_stroke_show, synaptics_s2w_allow_stroke_dump);

static ssize_t synaptics_s2w_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", syna_ts_data->s2w_enabled);

	return count;
}

static ssize_t synaptics_s2w_enabled_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
    int ret = 0;

	ret = strict_strtoul(buf, 10, &value);
    if (ret < 0) {
        return count;
    }
    if (value == 0 || value == 1) {
        syna_ts_data->s2w_enabled = (bool)value;
    	print_ts(TS_INFO, KERN_INFO "s2w_enabled=%d\n", syna_ts_data->s2w_enabled);
    }
	return count;
}

static DEVICE_ATTR(s2w_enabled, (S_IRUGO|S_IWUGO),
	synaptics_s2w_enabled_show, synaptics_s2w_enabled_dump);

static ssize_t synaptics_dt2w_enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", syna_ts_data->dt2w_enabled);

	return count;
}

static ssize_t synaptics_dt2w_enabled_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
    int ret = 0;

	ret = strict_strtoul(buf, 10, &value);
    if (ret < 0) {
        return count;
    }
    if (value == 0 || value == 1) {
        syna_ts_data->dt2w_enabled = (bool)value;
    	print_ts(TS_INFO, KERN_INFO "dt2w_enabled=%d\n", syna_ts_data->dt2w_enabled);
    }
	return count;
}

static DEVICE_ATTR(dt2w_enabled, (S_IRUGO|S_IWUGO),
	synaptics_dt2w_enabled_show, synaptics_dt2w_enabled_dump);

static ssize_t synaptics_s2w_barrier_y_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", syna_ts_data->s2w_barrier_y);

	return count;
}

static ssize_t synaptics_s2w_barrier_y_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
    int ret = 0;

	ret = strict_strtoul(buf, 10, &value);
    if (ret < 0) {
        return count;
    }
	syna_ts_data->s2w_barrier_y = (unsigned int)value;
	print_ts(TS_INFO, KERN_INFO "s2w_barrier_y=%d\n", syna_ts_data->s2w_barrier_y);
	return count;
}

static DEVICE_ATTR(s2w_barrier_y, (S_IRUGO|S_IWUGO),
	synaptics_s2w_barrier_y_show, synaptics_s2w_barrier_y_dump);
	
static ssize_t synaptics_dt2w_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", syna_ts_data->dt2w_duration);

	return count;
}

static ssize_t synaptics_dt2w_duration_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
    int ret = 0;

	ret = strict_strtoul(buf, 10, &value);
    if (ret < 0) {
        return count;
    }
	syna_ts_data->dt2w_duration = (unsigned int)value;
	print_ts(TS_INFO, KERN_INFO "dt2w_duration=%d\n", syna_ts_data->dt2w_duration);
	return count;
}

static DEVICE_ATTR(dt2w_duration, (S_IRUGO|S_IWUGO),
	synaptics_dt2w_duration_show, synaptics_dt2w_duration_dump);

static ssize_t synaptics_dt2w_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", syna_ts_data->dt2w_threshold);

	return count;
}

static ssize_t synaptics_dt2w_threshold_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
    int ret = 0;

	ret = strict_strtoul(buf, 10, &value);
    if (ret < 0) {
        return count;
    }
	syna_ts_data->dt2w_threshold = (unsigned int)value;
	print_ts(TS_INFO, KERN_INFO "dt2w_threshold=%d\n", syna_ts_data->dt2w_threshold);
	return count;
}

static DEVICE_ATTR(dt2w_threshold, (S_IRUGO|S_IWUGO),
	synaptics_dt2w_threshold_show, synaptics_dt2w_threshold_dump);

static ssize_t synaptics_dt2w_barrier_y_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;

	count += sprintf(buf, "%d\n", syna_ts_data->dt2w_barrier_y);

	return count;
}

static ssize_t synaptics_dt2w_barrier_y_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
    int ret = 0;

	ret = strict_strtoul(buf, 10, &value);
    if (ret < 0) {
        return count;
    }
	syna_ts_data->dt2w_barrier_y = (unsigned int)value;
	print_ts(TS_INFO, KERN_INFO "dt2w_barrier_y=%d\n", syna_ts_data->dt2w_barrier_y);
	return count;
}

static DEVICE_ATTR(dt2w_barrier_y, (S_IRUGO|S_IWUGO),
	synaptics_dt2w_barrier_y_show, synaptics_dt2w_barrier_y_dump);
#endif

static DEVICE_ATTR(log_level, S_IRUGO|S_IWUSR, synaptics_attr_loglevel_show, synaptics_attr_loglevel_store);
static DEVICE_ATTR(deltx, S_IRUGO|S_IWUSR, synaptics_attr_deltx_show, synaptics_attr_deltx_store);
static DEVICE_ATTR(delty, S_IRUGO|S_IWUSR, synaptics_attr_delty_show, synaptics_attr_delty_store);
static DEVICE_ATTR(report_mode, S_IRUGO|S_IWUSR, synaptics_attr_report_mode_show, synaptics_attr_report_mode_store);
static DEVICE_ATTR(info, S_IRUGO, synaptics_attr_basic_info_show, NULL);
static DEVICE_ATTR(baseline_test, S_IRUGO, tp_test_show, NULL);
static DEVICE_ATTR(test_max_error, S_IRUGO|S_IWUSR, synaptics_attr_errorlimit_show, synaptics_attr_errorlimit_store);
static DEVICE_ATTR(vendor_id, S_IRUGO, synaptics_attr_vendor_show, NULL);

static struct attribute * attr_debug_interfaces[] = {
	&dev_attr_log_level.attr,
	&dev_attr_deltx.attr,
	&dev_attr_delty.attr,
	&dev_attr_report_mode.attr,
	&dev_attr_info.attr,
	&dev_attr_test_max_error.attr,
	NULL,
};

static struct attribute_group syna_attr_group = {
	.name = "syna_attr",
	.attrs = attr_debug_interfaces,
};
static const struct attribute * attr_output_interfaces[] = {
	&dev_attr_vendor_id.attr,
	&dev_attr_baseline_test.attr,
	NULL,
};
static int synaptics_ts_sysfs_init(struct input_dev *dev)
{
	int ret;

	ret = sysfs_create_files(&dev->dev.kobj, attr_output_interfaces);
	if (ret)
		print_ts(TS_ERROR, KERN_ERR "[syna]%s: sysfs_create_files failed\n", __func__);

	ret = sysfs_create_group(&dev->dev.kobj, &syna_attr_group);
	if (ret)
		print_ts(TS_ERROR, KERN_ERR "[syna]%s: sysfs_create_group failed\n", __func__);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		print_ts(TS_ERROR, KERN_ERR "[syna]%s: create android_touch failed\n", __func__);
		ret = -ENOMEM;
	}
	else
		if (sysfs_create_file(android_touch_kobj, &dev_attr_s2w_enabled.attr) ||
        	sysfs_create_file(android_touch_kobj, &dev_attr_s2w_allow_stroke.attr) ||
        	sysfs_create_file(android_touch_kobj, &dev_attr_s2w_register_threshold.attr) ||
        	sysfs_create_file(android_touch_kobj, &dev_attr_s2w_min_distance.attr) ||
        	sysfs_create_file(android_touch_kobj, &dev_attr_s2w_barrier_y.attr) ||
        	sysfs_create_file(android_touch_kobj, &dev_attr_dt2w_enabled.attr) ||
        	sysfs_create_file(android_touch_kobj, &dev_attr_dt2w_duration.attr) ||
        	sysfs_create_file(android_touch_kobj, &dev_attr_dt2w_threshold.attr) ||
        	sysfs_create_file(android_touch_kobj, &dev_attr_dt2w_barrier_y.attr)) {
			print_ts(TS_ERROR, KERN_ERR "[syna]%s: create android_touch files failed\n", __func__);
			ret = -ENOMEM;
		}
#endif
	return ret;
}

static void synaptics_ts_sysfs_deinit(struct input_dev *dev)
{
	sysfs_remove_files(&dev->dev.kobj, attr_output_interfaces);
	sysfs_remove_group(&dev->dev.kobj, &syna_attr_group);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
	sysfs_remove_file(android_touch_kobj, &dev_attr_s2w_enabled.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_s2w_allow_stroke.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_s2w_register_threshold.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_s2w_min_distance.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_s2w_barrier_y.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_dt2w_enabled.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_dt2w_duration.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_dt2w_threshold.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_dt2w_barrier_y.attr);
	kobject_del(android_touch_kobj);
#endif

}
/***** For device sysfs interfaces end ********************/

/***** For virtual key definition begin *******************/
enum tp_vkey_enum
{
	TP_VKEY_MENU,
	TP_VKEY_HOME,
	TP_VKEY_BACK,

	TP_VKEY_NONE,
	TP_VKEY_COUNT = TP_VKEY_NONE,
};
static struct tp_vkey_button
{
	int x;
	int y;
	int width;
	int height;
}vkey_buttons[TP_VKEY_COUNT];

#define LCD_MAX_X  (1080)
#define LCD_MAX_Y  (1920)
#define VK_LCD_WIDTH  (LCD_MAX_X/TP_VKEY_COUNT)   // 3 keys
static void vk_calculate_area(void)  //added by liujun
{
	int i;
	int tp_max_x = syna_ts_data->max[0];
	int tp_max_y = syna_ts_data->max[1];
	int vk_height = syna_ts_data->virtual_key_height;
	int vk_width = tp_max_x/TP_VKEY_COUNT;
	int margin_x = 110;		// 每个按键左右各留的坐标点(相对于tp坐标)

	syna_ts_data->vk_prop_width = VK_LCD_WIDTH - 2* margin_x * LCD_MAX_X/tp_max_x + 30;
	syna_ts_data->vk_prop_center_y = LCD_MAX_Y/2 + tp_max_y * LCD_MAX_Y/((tp_max_y - vk_height)*2);
	syna_ts_data->vk_prop_height = tp_max_y*LCD_MAX_Y/(tp_max_y - vk_height) - LCD_MAX_Y;

	for (i = 0; i < TP_VKEY_COUNT; ++i)
	{
		vkey_buttons[i].width = vk_width - margin_x*2;
		vkey_buttons[i].height = vk_height - 10;
		vkey_buttons[i].x = vk_width*i + margin_x;
		vkey_buttons[i].y = tp_max_y - vkey_buttons[i].height;
	}
	vkey_buttons[TP_VKEY_MENU].x += 40;
	vkey_buttons[TP_VKEY_BACK].x -= 40;
}
static ssize_t vk_syna_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = syna_ts_data;
	return sprintf(buf,
		//	__stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":180:1997:350:154"
		//":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":540:1997:350:154"
		//":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":900:1997:350:154" "\n",
			__stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":%d:%d:%d:%d"
		":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)  ":%d:%d:%d:%d"
		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":%d:%d:%d:%d" "\n",
		VK_LCD_WIDTH/2 + 20,   ts->vk_prop_center_y, ts->vk_prop_width, ts->vk_prop_height,
		VK_LCD_WIDTH*3/2, ts->vk_prop_center_y, ts->vk_prop_width, ts->vk_prop_height,
		VK_LCD_WIDTH*5/2 - 20, ts->vk_prop_center_y, ts->vk_prop_width, ts->vk_prop_height);
}

static struct kobj_attribute vk_syna_attr = {
	.attr = {
		.name = "virtualkeys."SYNAPTICS_I2C_RMI_NAME,
		.mode = S_IRUGO,
	},
	.show = &vk_syna_show,
};

static struct attribute *syna_properties_attrs[] = {
	&vk_syna_attr.attr,
	NULL
};

static struct attribute_group syna_properties_attr_group = {
	.attrs = syna_properties_attrs,
};
static int synaptics_ts_init_virtual_key(struct synaptics_ts_data *ts )
{
	int ret = 0;
	vk_calculate_area();
	/* virtual keys */
	ts->properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (ts->properties_kobj)
		ret = sysfs_create_group(ts->properties_kobj,
			&syna_properties_attr_group);
	
	if (!ts->properties_kobj || ret)
		print_ts(TS_ERROR, KERN_ERR"%s: failed to create board_properties\n", __func__);
	/* virtual keys */
	return ret;
}
/***** For virtual key definition end *********************/

#define WAKEUP_GPIO  (14)
#define TP_ID_GPIO   (15)
static void synaptics_read_vendor_id(void)
{
	int vendor_id = 0;
	vendor_id = gpio_get_value(WAKEUP_GPIO);
	vendor_id = (vendor_id << 1) | gpio_get_value(TP_ID_GPIO);
	syna_ts_data->vendor_id = vendor_id;
	print_ts(TS_INFO, KERN_ERR "get vendor id: %x\n", vendor_id);
}

/**
 * synaptics_set_page() - sets the page
 * @pdata: pointer to synaptics_ts_data structure
 * @address: set the address of the page
 *
 * This function is used to set the page and returns integer.
 */
static int synaptics_set_page(struct synaptics_ts_data *ts,
					unsigned int address)
{
	int ret = 0;
	unsigned int page = ((address >> 8) & MASK_8BIT);
	if (page != ts->current_page)
	{
		ret = i2c_smbus_write_byte_data(ts->client, PAGE_SELECT, page);
		if (ret)
			dev_err(&ts->client->dev, "%s:failed:%d\n", __func__, ret);
		else
			ts->current_page = page;
	}
	return ret;
}

/**
 * synaptics_i2c_block_read() - read the block of data
 * @ts: pointer to synaptics_ts_data structure
 * @address: read the block of data from this offset
 * @size: number of bytes to read
 * @buf: pointer to a buffer containing the data to be read
 *
 * This function is to read the block of data. success return 0.
 * WARNING: Max read size is 32 bit. If the size is more than 32 bit,
 *         do not use this function.
 */
static int synaptics_i2c_block_read(struct synaptics_ts_data *ts,
					unsigned short address, int size, unsigned char *buf)
{
	int ret = 0;
	int retry_count = 0;
	int index;
	struct i2c_client *i2c = ts->client;

	mutex_lock(&ts->mutex_set_page);
	ret = synaptics_set_page(ts, address);
	if (ret)
		goto exit_block_read;

	index = address & MASK_8BIT;
retry_block_read:
	ret = i2c_smbus_read_i2c_block_data(i2c, index, size, buf);
	if (ret != size)
	{
		if (++retry_count == MAX_RETRY_COUNT)
		{
			dev_err(&i2c->dev, "%s: address 0x%04x size %d failed %d retry_count %d\n",
					__func__, address, size, ret, retry_count);
			ret = -1;
		}
		else
			goto retry_block_read;
	}
	else
	{
		ret = 0;
	}
exit_block_read:
	mutex_unlock(&ts->mutex_set_page);
	return ret;
}

/**
 * synaptics_i2c_byte_write() - write the single byte data
 * @pdata: pointer to synaptics_ts_data structure
 * @address: write the block of data from this offset
 * @data: data to be write
 *
 * This function is to write the single byte data and returns integer.
 */
static int synaptics_i2c_byte_write(struct synaptics_ts_data *ts,
						unsigned short address,
						unsigned char data)
{
	int ret = 0;
	struct i2c_client *i2c = ts->client;

	/* Can't have anyone else changing the page behind our backs */
	mutex_lock(&(ts->mutex_set_page));

	ret = synaptics_set_page(ts, address);
	if (ret)
		goto exit_byte_write;

	ret = i2c_smbus_write_byte_data(i2c, address & MASK_8BIT, data);
	if (ret)
		dev_err(&i2c->dev, "%s: address 0x%04x failed:%d\n",
					__func__, address, ret);

exit_byte_write:
	mutex_unlock(&(ts->mutex_set_page));
	return ret;
}

static void synaptics_set_fn_address(struct synaptics_rmi4_fn_desc* desc,
						uint8_t* pdt_data, int pdt_page)
{
	if (desc)
	{
		desc->query_base_addr = pdt_data[0] | (pdt_page<<8);
	 	desc->cmd_base_addr   = pdt_data[1] | (pdt_page<<8);
	 	desc->ctrl_base_addr  = pdt_data[2] | (pdt_page<<8);
	 	desc->data_base_addr  = pdt_data[3] | (pdt_page<<8);
	 	desc->intr_src_count  = pdt_data[4];
	 	desc->fn_number       = pdt_data[5];

		print_ts(TS_DEBUG, KERN_ERR "[SYNAPTICS] Find fn%02x: query:%04x, cmd:%04x, ctrl:%04x, data:%04x \n",
				desc->fn_number, desc->query_base_addr, desc->cmd_base_addr,
				desc->ctrl_base_addr,  desc->data_base_addr);
	}
}

static int synaptics_scan_pdt_table(struct synaptics_ts_data *ts)
{
	uint8_t pdt_buf[PDT_ENTRY_SIZE];
	int ret;
	int i;

	int j;
	int pdt_start_location = 0;
	int pdt_end_location = 0;

	ts->current_page = MASK_16BIT;
	for (j = 0; j < 5; ++j)
	{
		pdt_start_location = (j<<8) | PDT_START_SCAN_LOCATION;
		pdt_end_location = (j<<8) | PDT_END_SCAN_LOCATION;
		for (i = pdt_start_location; i > pdt_end_location; i -= PDT_ENTRY_SIZE)
		{
			ret = synaptics_i2c_block_read(ts, i, PDT_ENTRY_SIZE, pdt_buf);
			if (ret)
			{
				/* failed to read next PDT entry */
				dev_err(&ts->client->dev, "%s: read pdt table error, address=0x%x.\n", __func__, i);
				return -EIO;
			}
			if (pdt_buf[PDT_ENTRY_SIZE - 1] == 0)
			{
				/* A zero in the function number signals the end of the PDT */
				dev_dbg(&ts->client->dev, "%s:end of PDT\n", __func__);
				break;
			}
			switch (pdt_buf[PDT_ENTRY_SIZE - 1])
			{
				case 0x01:
					synaptics_set_fn_address(&ts->fn01_desc, pdt_buf, j);
					break;
				case 0x11:
					synaptics_set_fn_address(&ts->fn11_desc, pdt_buf, j);
					break;
				case 0x34:
					synaptics_set_fn_address(&ts->fn34_desc, pdt_buf, j);
					break;
				case 0x54:
					synaptics_set_fn_address(&ts->fn54_desc, pdt_buf, j);
					break;
			}
		}
	}

	return 0;
}

static int synaptics_scan_param(struct synaptics_ts_data *ts)
{
	int ret = 0;

	uint8_t  fn01_query_info[FN01_QUERY_INFO_BUF_LEN];
	uint8_t  fn11_buf[2];

	ts->current_page = MASK_16BIT;
	// scan pdt table
	ret = synaptics_scan_pdt_table(ts);
	if (ret)
		return ret;

	// read F01 QUERY content
	memset(fn01_query_info, 0, sizeof(fn01_query_info));
	ret = synaptics_i2c_block_read(ts, ts->fn01_desc.query_base_addr,
		sizeof(fn01_query_info), fn01_query_info);
	if (ret)
	{
		dev_err(&ts->client->dev, "%s: get info failed\n", __func__);
		return -1;
	}

	// read F34 customer id content
	memset(ts->version, 0, sizeof(ts->version));
	ret = synaptics_i2c_block_read(ts, F34_CTRL_CUSTOMER_ID_0,
		sizeof(ts->version), ts->version);
	if (ret)
	{
		dev_err(&ts->client->dev, "%s: get customer id failed\n", __func__);
		return -1;
	}
	//ts->version = (fn34_customer_id[2]<<8) | fn34_customer_id[3];

	if (1)
	{
		int year, month, day;
		char product_id[11];
		memset(product_id, 0, sizeof(product_id));
		memcpy(product_id, fn01_query_info + OFFSET_PRODUCT_ID, sizeof(product_id)-1);
		year = fn01_query_info[OFFSET_FIRMWARE_DATE] & MASK_5BIT;
		month = (fn01_query_info[OFFSET_FIRMWARE_DATE] >> 5) & MASK_3BIT;
		month |= (fn01_query_info[OFFSET_FIRMWARE_DATE+1] & MASK_1BIT) << 3;
		day = (fn01_query_info[OFFSET_FIRMWARE_DATE+1] >> 1) & MASK_5BIT;
		print_ts(TS_INFO, KERN_ERR "Synaptics %s manufactur:%x product:%x family:%x firmware:%x %d-%d-%d \n",
			product_id,//fn01_query_info + OFFSET_PRODUCT_ID,
			fn01_query_info[OFFSET_MANUFACTURER_ID],
			fn01_query_info[OFFSET_PRODUCT_PROPERTY],
			fn01_query_info[OFFSET_FAMILY],
			fn01_query_info[OFFSET_FIRMWARE_REVISION],
			year, month, day);
		//print_ts(TS_INFO, KERN_ERR "Synaptics customer id: %02x%02x%02x%02x \n",
		//	ts->version[0], ts->version[1], ts->version[2], ts->version[3]);
	}


	memset(fn11_buf, 0, sizeof(fn11_buf));
	// read max_x
	ret = synaptics_i2c_block_read(ts, F11_CTRL_MAX_X, sizeof(fn11_buf), fn11_buf);
	if (ret)
	{
		dev_err(&ts->client->dev, "%s: get max x failed\n", __func__);
		return -1;
	}
	ts->max[0] = (fn11_buf[0]&MASK_8BIT) | ((fn11_buf[1]&MASK_4BIT) << 8);

	// read max_y
	ret = synaptics_i2c_block_read(ts, F11_CTRL_MAX_Y, sizeof(fn11_buf), fn11_buf);
	if (ret)
	{
		dev_err(&ts->client->dev, "%s: get max y failed\n", __func__);
		return -1;
	}
	ts->max[1] = (fn11_buf[0]&MASK_8BIT) | ((fn11_buf[1]&MASK_4BIT) << 8);

	return 0;

}

static int synaptics_set_report_mode(struct synaptics_ts_data *ts, uint8_t set_mode)
{
	int ret;

	uint8_t report_reg;
	uint8_t status_int;

	ret = synaptics_i2c_block_read(ts, F11_CTRL_REPORT_MODE, 1, &report_reg);  //read report mode reg
	if (ret)
	{
		print_ts(TS_ERROR, KERN_ERR "%s: Read report mode reg failed.\n", __func__);
		return ret;
	}

	synaptics_i2c_block_read(ts, F01_DATA_INT_STATUS, 1, &status_int);  //clear the interrupt

	report_reg = (report_reg & 0xF8) | (set_mode & MASK_3BIT);
	ret = synaptics_i2c_byte_write(ts, F11_CTRL_REPORT_MODE, report_reg);
	if (ret)
	{
		print_ts(TS_ERROR, KERN_ERR "%s: Set report mode failed.\n", __func__);
		return ret;
	}

	ret |= synaptics_i2c_byte_write(ts, F11_CTRL_DELTA_X_THRESH, ts->deltx);
	ret |= synaptics_i2c_byte_write(ts, F11_CTRL_DELTA_Y_THRESH, ts->delty);

	return ret;
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;
	ts->current_page = MASK_16BIT;
	ts->need_hardware_reset = 0;

	ret = synaptics_i2c_byte_write(ts, F01_CTRL_DEVICE_CONTROL, 0x80);

	if (ret < 0)
		dev_err(&ts->client->dev, "%s: i2c_smbus_write_byte_data failed\n", __func__);
	return ret;
}

static int synaptics_set_int_mask(struct synaptics_ts_data *ts, int enable)
{
	uint8_t int_msk;
	int ret;
	uint8_t buf[1];

	if(enable)
		int_msk = 0x07;
	else 
		int_msk = 0x00;

	synaptics_i2c_block_read(ts, F01_DATA_INT_STATUS, 1, buf);  //clear the interrupt
	ret = synaptics_i2c_byte_write(ts, F01_CTRL_INT_ENABLE, int_msk);
	if(ret < 0) 
	{
		print_ts(TS_ERROR, KERN_ERR "%s: enable or disable abs interrupt failed, int_msk = 0x%x\n",
				__func__, int_msk);
	}
	return ret;	
}

static void synaptics_hardware_reset(struct synaptics_ts_data *ts)
{
	int ret = 0;

	print_ts(TS_DEBUG, "synaptics do hardware reset\n");
	/*if hardware reset is used, hold the I2C bus to avoid I2C err*/
	rt_mutex_lock(&ts->client->adapter->bus_lock);
	if(ts->power)
		ret = ts->power(0xEF);//i2c_smbus_write_byte_data(ts->client, REG_RESET, 0x01);
		
	if(ret < 0)
		print_ts(TS_ERROR, "synaptics reset failed!\n");
	
	rt_mutex_unlock(&ts->client->adapter->bus_lock);

	//msleep(200);
	synaptics_init_panel(ts);
	synaptics_set_int_mask( ts, 1);

}

static void synaptics_software_reset(struct synaptics_ts_data *ts)
{
	print_ts(TS_DEBUG, "synaptics do software reset\n");

	synaptics_i2c_byte_write(ts, F01_CMD_DEVICE_CMD, 0x01);
	msleep(30);
	synaptics_init_panel(ts);
	synaptics_set_int_mask( ts, 1);
}

static int get_virtual_key_button(int x, int y)
{
	int i;
	for (i = 0; i < TP_VKEY_NONE; ++i)
	{
		struct tp_vkey_button* button = &vkey_buttons[i];
		if ((x >= button->x) && (x <= button->x + button->width)
			&& (y >= button->y) && (y <= button->y + button->height))
		{
			// In this button area.
			break;
		}
	}
	return i;
}

static void offset_correction(struct synaptics_ts_data *ts, int *pf0_x, int *pf0_y)
{
	int f0_x, f0_y;

	f0_x = *pf0_x;
	f0_y = *pf0_y;

	//prtkTs("[%s]before f0_x = %2d, f0_y=%2d\n", __func__, f0_x, f0_y);
	/*x轴位置偏移校正*/
	if(f0_x <= syna_ts_data->snap_left)		/*解决从触摸屏的最左边往右边滑动的时候没有反应的问题*/
	{
		f0_x = syna_ts_data->snap_left + 1;
	}
	else if(f0_x >= ts->max[0]-syna_ts_data->snap_right)	/*解决从触摸屏的最右边往左边滑动的时候没有反应的问题*/
	{
		f0_x = ts->max[0] - syna_ts_data->snap_right - 1;
	}
	f0_x -= syna_ts_data->snap_left;

	/*y轴位置偏移校正*/
	if(f0_y <= syna_ts_data->snap_top)
	{
		f0_y = syna_ts_data->snap_top + 1;
	}
	else if (f0_y >= ts->max[1])
	{
		f0_y = ts->max[1] - 1;
	}
	f0_y -= syna_ts_data->snap_top;

	//prtkTs("[%s]after f0_x = %2d, f0_y=%2d\n", __func__, f0_x, f0_y);

	*pf0_x = f0_x;
	*pf0_y = f0_y;
}

static void synaptics_ts_work_func(struct work_struct *work)
{
	int i;
	int ret;
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	int data_offset = (MAX_FINGERS + 3)/4;
	int index;
	static uint32_t f0_x = 0, f0_y = 0, f0_z = 0;
	static uint8_t f0_wx, f0_wy;
	int finger_pressed = 0;
	uint8_t * buf = ts->finger_data;
	uint8_t buf_status[2];
	struct i2c_msg msg[2];
	uint8_t data_start_addr = ts->fn11_desc.data_base_addr & MASK_8BIT;
	unsigned char double_tap = 0;
	bool input_wakeup_event = false;
	bool virtual_key_pressed = false;

	//printk("[SYNAPTICS]%s enter.\n", __func__);
	down(&synaptics_sem);
	
	if (ts->is_tp_suspended)
	{
#if SUPPORT_DOUBLE_TAP
		if (1 == atomic_read(&ts->double_tap_enable)) {
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
		if (input_wakeup_active(ts)){
#endif
			ret= wait_event_timeout(ts->wait_i2c_ready,
					ts->i2c_ready,
					msecs_to_jiffies(1000));
			if (ret == 0) {
				print_ts(TS_WARNING, "wait i2c ready timeout\n");
				goto work_func_end;
			}
				
			//mdelay(50);
		} else {
			goto work_func_end;
		}
	}

	ret = synaptics_i2c_block_read(ts, F01_DATA_DEVICE_STATUS, 2, buf_status);

	if (ret)
	{
		print_ts(TS_ERROR, "%s: read device status failed. rc=%d\n", __func__, ret);
		ts->need_hardware_reset++;
		if(ts->need_hardware_reset > MAX_RETRY_COUNT) {
			print_ts(TS_WARNING, "synaptics tp do hardware reset forced\n");
			synaptics_hardware_reset(ts);
/* OPPO 2013-05-02 huanggd Add begin for double tap*/			
			if (ts->is_tp_suspended){
#if SUPPORT_DOUBLE_TAP			
				if (atomic_read(&ts->double_tap_enable)) {
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
				if (input_wakeup_active(ts)){
#endif
					synaptics_set_int_mask(ts, 0);
#if SUPPORT_DOUBLE_TAP
					synaptics_set_report_mode(ts, 0x04);
#endif
					enable_irq_wake(ts->client->irq);
					synaptics_set_int_mask(ts, 1);
					synaptics_i2c_byte_write(ts, F01_CTRL_DEVICE_CONTROL, 0x80);
				}
			}
/* OPPO 2013-05-02 huanggd Add end*/			
		}
		goto work_func_end;
	}

	if (buf_status[1] & 0x02) /*0x02: status bit*/
	{
		if(buf_status[0] != 0)
		{
#if 0
			print_ts(TS_WARNING, "TP interrupt register status unnormal , software reset !\n");
			synaptics_software_reset(ts);
#else
			print_ts(TS_WARNING, "TP interrupt register status unnormal , hardware reset !\n");
			synaptics_hardware_reset(ts);    
#endif

/* OPPO 2013-05-02 huanggd Add begin for double tap*/			
			if (ts->is_tp_suspended){
#if SUPPORT_DOUBLE_TAP			
				if (atomic_read(&ts->double_tap_enable)) {
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
				if (input_wakeup_active(ts)){
#endif
					print_ts(TS_WARNING, "reinit after hardware reset !\n");
					synaptics_set_int_mask(ts, 0);
#if SUPPORT_DOUBLE_TAP
					synaptics_set_report_mode(ts, 0x04);
#endif
					enable_irq_wake(ts->client->irq);
					synaptics_set_int_mask(ts, 1);
					synaptics_i2c_byte_write(ts, F01_CTRL_DEVICE_CONTROL, 0x80);
				}
			}
/* OPPO 2013-05-02 huanggd Add end*/
		} 
	}
	else if (buf_status[1] & 0x04) /*0x04: Abs0 bit*/
	{
		msg[0].addr = ts->client->addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &data_start_addr;
		msg[1].addr = ts->client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = sizeof(ts->finger_data);
		msg[1].buf = ts->finger_data;

		ret = i2c_transfer(ts->client->adapter, msg, 2);
		if (ret < 0) 
		//ret = synaptics_i2c_block_read(ts, ts->fn11_desc.data_base_addr, sizeof(ts->finger_data), ts->finger_data);
		//if (ret != sizeof(ts->finger_data))
		{
			print_ts(TS_ERROR, KERN_ERR "[%s]: read finger data failed.\n", __func__);
		}
		else
		{
			index = data_offset;
			//finger_pressed = 0;
			finger_pressed = buf[0] | buf[1] | (buf[2]&0x0F);

			if (finger_pressed)
			{
				for (i = 0; i < MAX_FINGERS; ++i, index += FINGER_PACKAGE_SIZE)
				{
					f0_z = buf[index + 4];
					if(f0_z) 
					{

						f0_x = (buf[index + 0] <<4) | (buf[index + 2] & 0x0f);
						f0_y = (buf[index + 1] <<4) | ((buf[index + 2] & 0xf0) >>4);
						f0_wx = (buf[index + 3] & 0x0f);
						f0_wy = ((buf[index + 3] & 0xf0) >>4);

						offset_correction(ts, &f0_x, &f0_y);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
						if (input_wakeup_active(ts) && ts->s2w_enabled)
						{
							if (i == 0 && s2w_down_x != -2)
							{
								int finger_y = f0_y;
								int finger_x = f0_x;

								if (finger_y > ts->s2w_barrier_y)
								{
									if(s2w_down_x == -1)
									{
										print_ts(TS_TRACE, KERN_ERR "down at %d %d\n", finger_x, finger_y);
										s2w_down_x = finger_x;
									} 
									else 
									{
										if (ts->s2w_allow_stroke)
										{
											// stroke2wake - any direction activates
											if (synaptics_s2w_handle_move(ts, finger_x))
											{
												if (s2w_exec_power_press)
												{
													simulate_power_press();
													s2w_exec_power_press = false;
													input_wakeup_event = true;
													goto work_func_end;
												}
											}
										} 
										else 
										{
											// Free swipe - single direction activation
											//left->right	
											if (ts->is_tp_suspended)
											{
												if(finger_x > s2w_down_x)
												{
													if (synaptics_s2w_handle_move(ts, finger_x))
													{
														if (s2w_exec_power_press)
														{
															simulate_power_press();
															s2w_exec_power_press = false;
															input_wakeup_event = true;
															goto work_func_end;
														}
													}
												}
											//right->left
											} 
											else 
											{
												if(finger_x < s2w_down_x)
												{
													if (synaptics_s2w_handle_move(ts, finger_x))
													{
														if (s2w_exec_power_press)
														{
															simulate_power_press();
															s2w_exec_power_press = false;
															input_wakeup_event = true;
															goto work_func_end;
														}
													}
												}
											}
										}
									}
								} else {
									print_ts(TS_TRACE, KERN_ERR "left buttons at %d\n", finger_y);
									// this prevents swipes originating on screen and then
									// entering button panel to affect s2w (gaming etc.)
									s2w_down_x = -2; 
								}
							}
							// we are in a swipe - dont report anything
							if (s2w_barrier_reached)
								goto work_func_end;
						}

#endif
						if (f0_y > ts->max[1] - ts->snap_top - ts->virtual_key_height)
						{
							int pressed_vkey = get_virtual_key_button(f0_x, f0_y);
							if (pressed_vkey == TP_VKEY_NONE)
							{
								input_mt_sync(ts->input_dev);
								print_ts(TS_TRACE, KERN_ERR"[%s]:finger(%d): (x,y)=(%4d,%4d), not in available area\n",
										__func__, i, f0_x, f0_y);
								continue;
							}
							virtual_key_pressed = true;
						}
						//finger_pressed ++;
						input_point_num++;

						// dont do that when pressing a virtual key!
						if (!virtual_key_pressed && 1 == input_point_num)
						{
							continue;
						}

						input_report_abs(ts->input_dev, ABS_MT_POSITION_X, f0_x);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, f0_y);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE, f0_z);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, f0_z);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, f0_wx);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MINOR, f0_wy);
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
						input_mt_sync(ts->input_dev);

						print_ts(TS_TRACE, KERN_ERR"[%s]:finger(%d): (x,y)=(%4d,%4d), z=%3d, f0_wx=%2d, f0_wy=%2d\n",
							__func__, i, f0_x, f0_y, f0_z, f0_wx, f0_wy);
					}

				}
			}
			else//if (finger_pressed == 0)
			{
				input_point_num = 0;
				input_mt_sync(ts->input_dev);
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
				if (input_wakeup_active(ts)){
					print_ts(TS_TRACE, KERN_ERR "up at %d %d\n", f0_x, f0_y);
										
					s2w_exec_power_press = true;
					s2w_barrier_reached = false;
					s2w_down_x = -1;

					if (ts->is_tp_suspended && ts->dt2w_enabled && f0_y > ts->dt2w_barrier_y){
						u64 now = ktime_to_ms(ktime_get());
						u64 diff = now - dt2w_double_tap_start;
						u64 tapTime = ts->dt2w_duration;
						u64 tooLongTime = tapTime + ts->dt2w_threshold;

						print_ts(TS_TRACE, KERN_ERR "dt2w x=%d y=%d\n", f0_x, f0_y);
						print_ts(TS_TRACE, KERN_ERR "dt2w diff=%lld\n", diff);

						dt2w_double_tap_start = now;
					
						if (diff > tapTime && diff < tooLongTime){
							simulate_power_press();
							input_wakeup_event = true;
							goto work_func_end;
						}
					}
				}
#endif
				print_ts(TS_TRACE, KERN_ERR"[%s]: all finger up\n", __func__);
			}

			input_sync(ts->input_dev);

#if SUPPORT_DOUBLE_TAP
			if (ts->is_tp_suspended && 1 == atomic_read(&ts->double_tap_enable))
			{
				
				synaptics_i2c_block_read(ts, F11_DATA_LPWG_STATUS, 1, &double_tap);
				print_ts(TS_DEBUG, KERN_INFO "[%s] get LPWG Status value = 0x%x \n", __func__, double_tap);
				if (double_tap & 0x1)
				{
					//input_report_key(ts->input_dev, KEY_POWER, 1);
					input_report_key(ts->input_dev, KEY_DOUBLE_TAP, 1);//sjc  for double tap wakeup detect with proxy
					input_sync(ts->input_dev);
					//input_report_key(ts->input_dev, KEY_POWER, 0);
					input_report_key(ts->input_dev, KEY_DOUBLE_TAP, 0);//sjc  for double tap wakeup detect with proxy
					input_sync(ts->input_dev);
					atomic_inc(&ts->double_tap_number);
				}
			}
#endif
		}
	}


work_func_end:
	
	if (ts->use_irq)
		enable_irq(ts->client->irq);
/* OPPO 2013-05-02 huanggd Add begin for double tap*/	

	if (ts->is_tp_suspended){
#if SUPPORT_DOUBLE_TAP	
		if (atomic_read(&ts->double_tap_enable)) {
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
		if (input_wakeup_active(ts)){
#endif
			if (double_tap || input_wakeup_event)
				wake_lock_timeout(&ts->double_wake_lock, HZ);
			else
				wake_unlock(&ts->double_wake_lock);
		}
	}
	
/* OPPO 2013-05-02 huanggd Add end*/

	up(&synaptics_sem);
}

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	/* printk("synaptics_ts_timer_func\n"); */

	queue_work(synaptics_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	/* printk("synaptics_ts_irq_handler\n"); */
	disable_irq_nosync(ts->client->irq);
	queue_work(synaptics_wq, &ts->work);
/* OPPO 2013-05-02 huanggd Add begin for double tap*/	

	if (ts->is_tp_suspended){
#if SUPPORT_DOUBLE_TAP	
		if (atomic_read(&ts->double_tap_enable)) {
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
		if (input_wakeup_active(ts)){
#endif
			wake_lock_timeout(&ts->double_wake_lock, HZ);
			//print_ts(TS_DEBUG, KERN_INFO "[%s]  \n", __func__);
		}
	}

/* OPPO 2013-05-02 huanggd Add end*/
	return IRQ_HANDLED;
}

static void synaptics_update_fw_from_file(const char *filename)
{
	int fd = 0;
	int count = 0;
	unsigned char* filedata = NULL;
	if (filename)
	{
		int fd = sys_open(filename, O_RDONLY, 0);
		if (fd < 0) {
			printk(KERN_WARNING "%s: Can not open %s\n", __func__, filename);
			return ;
		}
		count = sys_lseek(fd, (off_t)0, 2);
		if (count <= 0) {
			goto err_load_fw_close_file;
		}

		sys_lseek(fd, (off_t)0, 0);
		filedata = kmalloc(count, GFP_KERNEL);
		if (!filedata) {
			printk(KERN_WARNING "%s: Can not alloc data\n", __func__);
			goto err_load_fw_close_file;
		}
		if (sys_read(fd, (char *)filedata, count) != count) {
			goto err_load_fw_free_data;
		}

		if (strncmp(filedata+0x10, "S3202", 5)) {
			printk(KERN_WARNING "%s: Not correct fw file.\n", __func__);
			goto err_load_fw_free_data;
		}

		disable_irq_nosync(syna_ts_data->client->irq);
		synaptics_set_int_mask(syna_ts_data, 0);

		CompleteReflash(syna_ts_data->client, filedata);

		synaptics_scan_param(syna_ts_data);
		synaptics_init_panel(syna_ts_data);
		synaptics_set_report_mode(syna_ts_data, syna_ts_data->report_mode);
		synaptics_set_int_mask(syna_ts_data, 1);
		enable_irq(syna_ts_data->client->irq);
	}
	else
	{
		printk("no firmware file.\n");
		return;
	}

err_load_fw_free_data:
	kfree(filedata);
err_load_fw_close_file:
	sys_close(fd);
	return;
}

static char proc_syna_data[64];  
static char *fw_file_path = NULL;
static void synaptics_ts_delay_work(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, delay_work.work);
	//int ret;
	down(&synaptics_sem);
	if (ts->is_tp_suspended)
		goto end_syna_delay_work;

	if(ts->cmd == TP_CMD_RESET)
	{
		ts->cmd = TP_CMD_DO_NOTHING;
		printk(KERN_INFO"synaptics do reset now.\n");
		/*clear the touchpanel data*/
		input_mt_sync(ts->input_dev);
		input_sync(ts->input_dev);

		synaptics_software_reset(ts);
	}
	else if( ts->cmd == TP_CMD_REZERO)
	{
		ts->cmd = TP_CMD_DO_NOTHING;
		printk(KERN_INFO"synaptics do rezero now.\n");
		synaptics_i2c_byte_write(ts, ts->fn11_desc.cmd_base_addr, 0x01);
	}
	else if (ts->cmd == TP_CMD_FORCE_UPDATE)
	{
		ts->cmd = TP_CMD_DO_NOTHING;

		printk(KERN_INFO "synaptics do force-update!\n");
		disable_irq_nosync(ts->client->irq);
		synaptics_set_int_mask(ts, 0);

		if (ts->vendor_id == TP_VENDOR_TRULY)
		{
			CompleteReflash(ts->client, Syna_Firmware_Data_Truly);
		}
		else if (ts->vendor_id == TP_VENDOR_WINTEK)
		{
			CompleteReflash(ts->client, TOUCHSCREEN_SYNAPTICS_WINTEK_FW);
		}
		else if (ts->vendor_id == TP_VENDOR_TPK)
		{
			CompleteReflash(ts->client, Syna_Firmware_Data_TPK);
		}
		synaptics_scan_param(ts);
		synaptics_init_panel(ts);
		synaptics_set_report_mode(ts, ts->report_mode);
		synaptics_set_int_mask(ts, 1);
		enable_irq(ts->client->irq);

	}
	else if (ts->cmd == TP_CMD_UPDATE_FROM_FILE)
	{
		ts->cmd = TP_CMD_DO_NOTHING;
		printk(KERN_INFO "synaptics do update firmware!\n");
		if (fw_file_path)
		{
			int len = strlen(fw_file_path);
			if (fw_file_path[len-1] == '\n')
				fw_file_path[len-1] = 0;
			synaptics_update_fw_from_file(fw_file_path);
			kfree(fw_file_path);
			fw_file_path = NULL;
		}
	}

end_syna_delay_work:
	up(&synaptics_sem);

}

static ssize_t synaptics_proc_write( struct file *filp, const char __user *buff,
                        unsigned long len, void *data )
{
	int copy_len = len;
	struct synaptics_ts_data *ts = data;
	if (copy_len >= sizeof(proc_syna_data))
		copy_len = sizeof(proc_syna_data) - 1;

	if (copy_from_user( &proc_syna_data, buff, copy_len )) {
		printk(KERN_INFO "synaptics read proc input error.\n");
		return -EFAULT;
	}
	proc_syna_data[copy_len] = 0;
	//printk("synaptics write: (%s) len=%d\n", proc_syna_data, copy_len);

	if (proc_syna_data[0] == '1') {
		print_ts(TS_DEBUG, KERN_INFO "synaptics proc get reset cmd.\n");
		ts->cmd = TP_CMD_RESET;
		cancel_delayed_work_sync(&ts->delay_work);
		schedule_delayed_work(&ts->delay_work, msecs_to_jiffies(20));
	} else if(proc_syna_data[0] == '2') {
		ts->cmd = TP_CMD_REZERO;
		cancel_delayed_work_sync(&ts->delay_work);
		schedule_delayed_work(&ts->delay_work, msecs_to_jiffies(20));
	} else if (!strcmp(proc_syna_data, "3\n")) {
		int irq_status = gpio_get_value(6);  // tp_int_gpio = 6
		printk(KERN_INFO "synaptics device irq(%d)!\n", irq_status);
	} else if (!strcmp(proc_syna_data, "force-update tp-fw\n")) {
		ts->cmd = TP_CMD_FORCE_UPDATE;
		//printk(KERN_INFO "synaptics update force now!\n");
		cancel_delayed_work_sync(&ts->delay_work);
		schedule_delayed_work(&ts->delay_work, msecs_to_jiffies(20));
	} else if (!strncmp(proc_syna_data, "update-fw use ", 14)) {
		ts->cmd = TP_CMD_UPDATE_FROM_FILE;
		if (!fw_file_path) {
			fw_file_path = kmalloc(len-14+1, GFP_KERNEL);
			if (fw_file_path && !copy_from_user(fw_file_path, buff+14, len-14)) {
				fw_file_path[len-14] = 0; //end string with NULL char
				cancel_delayed_work_sync(&ts->delay_work);
				schedule_delayed_work(&ts->delay_work, msecs_to_jiffies(20));
			}
		}
	} else if (!strcmp(proc_syna_data, "power on\n")) {
		if (ts->power)
			ts->power(1);
	} else if (!strcmp(proc_syna_data, "power off\n")) {
		if (ts->power)
			ts->power(0);
	} else if (!strcmp(proc_syna_data, "set tp_suspend\n")) {
		ts->is_tp_suspended = 1;
	} else if (!strcmp(proc_syna_data, "set tp_resume\n")) {
		ts->is_tp_suspended = 0;
	} else if (!strcmp(proc_syna_data, "set irq_enable\n")) {
		if (ts->use_irq)
			enable_irq(ts->client->irq);
	} else if (!strcmp(proc_syna_data, "set irq_disable\n")) {
		if (ts->use_irq)
			disable_irq(ts->client->irq);
	}

	return len;
}

#if SUPPORT_DOUBLE_TAP
#define BUFFER_LEN (10)
static int double_tap_counter_proc_write( struct file *filp, const char __user *buff,
                        unsigned long len, void *data )
{
	unsigned int val = 0;
	struct synaptics_ts_data *ts = data;
	char buf[BUFFER_LEN];

	if (len > BUFFER_LEN) 
		return len;
	
	if (copy_from_user( buf, buff, len )) {
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return len;
	}
	
	sscanf(buf, "%d", &val);
 	if (val >= 0) {
		atomic_set(&ts->double_tap_number, val);
		printk(KERN_INFO"%s: set double tap count : %d\n", __func__, val);
	}
	return len;
}

static int double_tap_enable_proc_write( struct file *filp, const char __user *buff,
                        unsigned long len, void *data )
{
	unsigned int val = 0;
	struct synaptics_ts_data *ts = data;
	char buf[BUFFER_LEN];

	if (len > BUFFER_LEN) 
		return len;
	
	if (copy_from_user( buf, buff, len )) {
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return len;
	}
	
	sscanf(buf, "%d", &val);
	val = (val == 0 ? 0:1);
	
	if (ts->is_tp_suspended == 0) {
		atomic_set(&ts->double_tap_enable, val);
		printk(KERN_INFO"%s: set double tap enable : %d\n", __func__, val);
	} else {
		if ((val == 1) && atomic_read(&ts->double_tap_enable) == 0) {
			wake_lock(&ts->double_wake_lock);
			down(&synaptics_sem);

			if (ts->power) {
				ts->power(1);
			}
			
			synaptics_set_int_mask(ts, 0);
			synaptics_set_report_mode(ts, 0x04);
			enable_irq(ts->client->irq);
			enable_irq_wake(ts->client->irq);
			synaptics_set_int_mask(ts, 1);
			synaptics_i2c_byte_write(ts, F01_CTRL_DEVICE_CONTROL, 0x80);
			atomic_set(&ts->double_tap_enable, val);
			
			up(&synaptics_sem);
			wake_unlock(&ts->double_wake_lock);
			printk(KERN_INFO"%s: set double tap enable %d while suspend\n", __func__, val);
		} else if ((val == 0) && atomic_read(&ts->double_tap_enable) == 1) {
			wake_lock(&ts->double_wake_lock);
			down(&synaptics_sem);

			disable_irq(ts->client->irq);
			synaptics_set_int_mask(ts, 0); /* disable interrupt */
			/* deep sleep */
			synaptics_i2c_byte_write(ts, F01_CTRL_DEVICE_CONTROL, 0x01); 		
			if (ts->power) {
				ts->power(0);
			}
			atomic_set(&ts->double_tap_enable, val);
			
			up(&synaptics_sem);
			wake_unlock(&ts->double_wake_lock);
			printk(KERN_INFO"%s: set double tap enable %d while suspend\n", __func__, val);
	
		}

	}
	return len;
}

static int double_tap_counter_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	struct synaptics_ts_data *ts = data;
	
	return sprintf(page, "%d\n", atomic_read(&ts->double_tap_number));
}

static int double_tap_enable_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	struct synaptics_ts_data *ts = data;
	
	return sprintf(page, "%d\n", atomic_read(&ts->double_tap_enable));
}
#endif

extern struct proc_dir_entry proc_root;
static int init_synaptics_proc(struct synaptics_ts_data *ts)
{
	int ret=0;
#if SUPPORT_DOUBLE_TAP	
	struct proc_dir_entry *prcdir;
#endif
	struct proc_dir_entry *proc_entry = create_proc_entry( "syna_proc_write", 0666, NULL );
	proc_entry->data = ts;

	if (proc_entry == NULL)
	{
		ret = -ENOMEM;
	  	printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
	else
	{
		proc_entry->write_proc = synaptics_proc_write;
	}
#if SUPPORT_DOUBLE_TAP
	prcdir = proc_mkdir("touchpanel", &proc_root);
	if (prcdir == NULL) {
		printk(KERN_ERR "%s: can't create /proc/touchpanel\n", __func__);
		return ret;
	}
	proc_entry = create_proc_entry("double_tap_counter", 0666, prcdir);
	if (proc_entry) {
		proc_entry->write_proc = double_tap_counter_proc_write;
		proc_entry->read_proc = double_tap_counter_proc_read;
		proc_entry->data = ts;
	}
	proc_entry = create_proc_entry("double_tap_enable", 0666, prcdir);
	if (proc_entry) {
		proc_entry->write_proc = double_tap_enable_proc_write;
		proc_entry->read_proc = double_tap_enable_proc_read;
		proc_entry->data = ts;
	}
#endif	
	return ret;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	uint16_t max_x, max_y;
	int fuzz_x, fuzz_y, fuzz_p, fuzz_w;
	struct synaptics_i2c_rmi_platform_data *pdata;
	unsigned long irqflags;
	int force_update;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		print_ts(TS_ERROR, KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	syna_ts_data = ts;

	INIT_WORK(&ts->work, synaptics_ts_work_func);
	INIT_DELAYED_WORK(&ts->delay_work, synaptics_ts_delay_work);
/* OPPO 2013-05-02 huanggd Add begin for double tap*/		
	wake_lock_init(&ts->double_wake_lock, WAKE_LOCK_SUSPEND, "touchpanel");
/* OPPO 2013-05-02 huanggd Add end*/	
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata)
		ts->power = pdata->power;
	/*if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "synaptics_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}*/

	ts->is_tp_suspended = 0;
	ts->report_mode = 0x00;
	ts->deltx = 5;
	ts->delty = 5;
	ts->current_page = MASK_16BIT;
	mutex_init(&(ts->mutex_set_page));

	synaptics_read_vendor_id();

#if TP_UPDATE_FIRMWARE
detect_device:
#endif
	// detect device
	ret = synaptics_scan_param(ts);
	if (ret < 0)
		goto err_detect_failed;

	print_ts(TS_ERROR, "vendor: %d version: %02x%02x\n",
			ts->vendor_id, ts->version[2], ts->version[3]);

	force_update = 0;

firmware_update:
	//Firmware update
#if TP_UPDATE_FIRMWARE
	if (MSM_BOOT_MODE__RECOVERY != get_boot_mode())
	{
		unsigned int fw_update_version = 0;
		const unsigned char* fw_update_data = NULL;
		if (ts->vendor_id == TP_VENDOR_TRULY)
		{
			fw_update_version = FIRMWARE_TRULY_VERSION;
			fw_update_data = Syna_Firmware_Data_Truly;
		}
		else if (ts->vendor_id == TP_VENDOR_WINTEK)
		{
			fw_update_version = FIRMWARE_WINTEK_VERSION;
			fw_update_data = TOUCHSCREEN_SYNAPTICS_WINTEK_FW;
		}
		else if (ts->vendor_id == TP_VENDOR_TPK)
		{
			fw_update_version = FIRMWARE_TPK_VERSION;
			fw_update_data = Syna_Firmware_Data_TPK;
		}
		if (fw_update_version != 0 && fw_update_data != NULL && (force_update
			|| (ts->version[2] != ((fw_update_version>>8)&0xFF)
				|| ts->version[3] < (fw_update_version&0xFF))))
		{
			display_rle_file(TP_UPDATE_RLE_FILE);
			CompleteReflash(client, fw_update_data);
			goto detect_device;
		}
	}
#endif
	if (ts->max[0] == 0 || ts->max[1] == 0)
	{
		force_update ++;
		if (force_update < 3)
		{
			print_ts(TS_ERROR, "[SYNAP]detect invalid max_x or max_y, update firmware force!\n");
			goto firmware_update;
		}
		else
		{
			print_ts(TS_ERROR, "[SYNAP]detect invalid max_x or max_y, exit init!\n");
			goto err_detect_failed;
		}
	}

	ret = synaptics_set_int_mask(ts, 0); /* disable interrupt */
	if (ret < 0) {
		print_ts(TS_ERROR, KERN_ERR "%s: disable interrupt failed\n", __func__);
		goto err_detect_failed;
	}

	if (pdata) {
		//while (pdata->version > ts->version)
		//	pdata++;
		ts->flags = pdata->flags;
		ts->sensitivity_adjust = pdata->sensitivity_adjust;
		irqflags = pdata->irqflags;
		fuzz_x = pdata->fuzz_x;
		fuzz_y = pdata->fuzz_y;
		fuzz_p = pdata->fuzz_p;
		fuzz_w = pdata->fuzz_w;
	} else {
		irqflags = 0;
		fuzz_x = 0;
		fuzz_y = 0;
		fuzz_p = 0;
		fuzz_w = 0;
	}


	ret = synaptics_init_panel(ts);
	ret |= synaptics_set_report_mode(ts, ts->report_mode);
	if (ret < 0) {
		print_ts(TS_ERROR, KERN_ERR "synaptics_init_panel failed\n");
		goto err_detect_failed;
	}

//huanggd tmp
#if 0
	{
		uint8_t tttemp;
		
		synaptics_i2c_block_read(ts, F11_CTRL_32_00, 1, &tttemp);  
		print_ts(TS_ERROR, KERN_ERR "F11_CTRL_32_00=0x%x\n", tttemp);
		synaptics_i2c_block_read(ts, F11_CTRL_32_01, 1, &tttemp);  
		print_ts(TS_ERROR, KERN_ERR "F11_CTRL_32_01=0x%x\n", tttemp);
		synaptics_i2c_block_read(ts, F11_CTRL_58, 1, &tttemp);  
		print_ts(TS_ERROR, KERN_ERR "F11_CTRL_58_01=0x%x\n", tttemp);
		synaptics_i2c_block_read(ts, F54_CTRL_02_00, 1, &tttemp); 
		print_ts(TS_ERROR, KERN_ERR "F54_CTRL_02_00=0x%x\n", tttemp);
		synaptics_i2c_block_read(ts, F54_CTRL_02_01, 1, &tttemp); 
		print_ts(TS_ERROR, KERN_ERR "F54_CTRL_02_01=0x%x\n", tttemp);
	}
#endif
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		print_ts(TS_ERROR, KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = SYNAPTICS_I2C_RMI_NAME;//"synaptics touchscreen s3202";

	max_x = ts->max[0];
	max_y = ts->max[1];
	if (ts->flags & SYNAPTICS_SWAP_XY)
		swap(max_x, max_y);

	if (ts->vendor_id == TP_VENDOR_TRULY)
	{
		// TP height:117.8mm, LCD:110.6mm
		ts->virtual_key_height = max_y * 72/1178;
		ts->snap_top = 0;
		ts->snap_left = 0;
		// TP width:62.52,  LCD width:61.88
		ts->snap_right = max_x * 64/6252;
	}
	else if (ts->vendor_id == TP_VENDOR_WINTEK)
	{
		// TP height:117.2mm, LCD:110.6mm
		ts->virtual_key_height = max_y * 66/1172;
		ts->snap_top = 0;
		ts->snap_left = 0;
		// TP width:63.12,  LCD width:61.88
		ts->snap_right = max_x * 124/6312;
	}
	else //if (ts->vendor_id == TP_VENDOR_TPK)
	{
		// TP height:117.74mm, LCD:110.6mm
		ts->virtual_key_height = max_y * 71/1177;
		ts->snap_top = 0;
		ts->snap_left = 0;
		// TP width:62.92,  LCD width:61.88
		ts->snap_right = max_x * 104/6292;
	}

	fuzz_x = fuzz_x * max_x / 0x10000;
	fuzz_y = fuzz_y * max_y / 0x10000;
	print_ts(TS_INFO, KERN_INFO "synaptics_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
	print_ts(TS_INFO, KERN_INFO "synaptics_ts_probe: inactive_x %d %d, inactive_y %d %d\n",
	       ts->snap_left, ts->snap_right,
	       ts->snap_top,  ts->virtual_key_height);

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOMEPAGE, ts->input_dev->keybit);

#if SUPPORT_DOUBLE_TAP
	//set_bit(KEY_POWER, ts->input_dev->keybit);
	set_bit(KEY_DOUBLE_TAP, ts->input_dev->keybit);//sjc  for double tap wakeup detect with proxy
	atomic_set(&ts->double_tap_number, 0);
	atomic_set(&ts->double_tap_enable, 0);
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
	ts->s2w_enabled = true;
	ts->s2w_register_threshold = 9;
	ts->s2w_min_distance = 325;
	ts->s2w_allow_stroke = true;
	
	// button area begins at this y position
	ts->s2w_barrier_y = max_y - 150;
	print_ts(TS_INFO, KERN_INFO "%s: s2w_enabled=%d  s2w_register_threshold=%d s2w_min_distance=%d s2w_allow_stroke=%d s2w_barrier_y=%d\n", __func__, ts->s2w_enabled, ts->s2w_register_threshold, ts->s2w_min_distance, ts->s2w_allow_stroke, ts->s2w_barrier_y);	
	
	ts->dt2w_barrier_y = ts->s2w_barrier_y;
	ts->dt2w_enabled = true;
	ts->dt2w_duration = 150;
	ts->dt2w_threshold = 200;

	print_ts(TS_INFO, KERN_INFO "%s: dt2w_enabled=%d  dt2w_duration=%d dt2w_threshold=%d dt2w_barrier_y=%d\n", __func__, ts->dt2w_enabled, ts->dt2w_duration, ts->dt2w_threshold, ts->dt2w_barrier_y);	
	
#endif
	// set device type as touchscreen
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, ts->snap_left, max_x - ts->snap_right, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 
							ts->snap_top, max_y - ts->virtual_key_height, fuzz_y, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,    0, 255, fuzz_p, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, fuzz_p, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, fuzz_w, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MAX_FINGERS, 0, 0);

	synaptics_ts_init_virtual_key(ts);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		print_ts(TS_ERROR, KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	if (client->irq) {
		ret = request_irq(client->irq, synaptics_ts_irq_handler, irqflags, client->name, ts);
		if (ret == 0) {
			ret = synaptics_set_int_mask(ts, 1); /* enable abs int */
			if (ret)
			{
				print_ts(TS_ERROR, KERN_ERR "%s: failed to enable synaptics interrupt!\n",__func__);
				free_irq(client->irq, ts);
				goto err_input_register_device_failed;
			}
		}
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
/* OPPO 2013-05-02 huanggd Add begin for double tap*/	
	ts->early_suspend_power.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	ts->early_suspend_power.suspend = NULL;
	ts->early_suspend_power.resume = synaptics_ts_late_resume_power;
	register_early_suspend(&ts->early_suspend_power);

	init_waitqueue_head(&ts->wait_i2c_ready);
	ts->i2c_ready = 1;
/* OPPO 2013-05-02 huanggd Add end*/	
#endif

	synaptics_ts_sysfs_init(ts->input_dev);
	init_synaptics_proc(ts);

	print_ts(TS_INFO, KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	if (ts->power)
		ts->power(0);
	wake_lock_destroy(&ts->double_wake_lock);
//err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	synaptics_ts_sysfs_deinit(ts->input_dev);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	down(&synaptics_sem);
	ts->is_tp_suspended = 1;

#if SUPPORT_DOUBLE_TAP
	if (1 == atomic_read(&ts->double_tap_enable))
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
	if (input_wakeup_active(ts))
#endif
	{
		print_ts(TS_INFO, KERN_INFO "%s: input_wakeup_active\n", __func__);
		synaptics_set_int_mask(ts, 0);
#if SUPPORT_DOUBLE_TAP
		synaptics_set_report_mode(ts, 0x04);
#endif
		enable_irq_wake(client->irq);
		synaptics_set_int_mask(ts, 1);
		synaptics_i2c_byte_write(ts, F01_CTRL_DEVICE_CONTROL, 0x80);
		up(&synaptics_sem);
		return 0;
	}

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	//ret = cancel_work_sync(&ts->work);
	//if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	//	enable_irq(client->irq);
	ret = synaptics_set_int_mask(ts, 0); /* disable interrupt */
	if (ret < 0)
		print_ts(TS_ERROR, KERN_ERR "%s: can not disable interrupt\n", __func__);

	/* deep sleep */
	ret = synaptics_i2c_byte_write(ts, F01_CTRL_DEVICE_CONTROL, 0x01); 
	if (ret < 0)
		print_ts(TS_ERROR, KERN_ERR "synaptics_ts_suspend: i2c_smbus_write_byte_data failed\n");
	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			print_ts(TS_ERROR, KERN_ERR "synaptics_ts_resume power off failed\n");
	}

	up(&synaptics_sem);
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	down(&synaptics_sem);

#if SUPPORT_DOUBLE_TAP
	if (1 == atomic_read(&ts->double_tap_enable))
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
	if (input_wakeup_active(ts))
#endif
	{
		print_ts(TS_INFO, KERN_INFO "%s: input_wakeup_active\n", __func__);
/* OPPO 2013-05-02 huanggd Add begin for double tap*/		
		if (ts->power) {
			ret = ts->power(1);
			if (ret < 0)
				print_ts(TS_ERROR, KERN_ERR "synaptics_ts_resume power on failed\n");
		}
/* OPPO 2013-05-02 huanggd Add end*/			
		synaptics_set_int_mask(ts, 0);
		synaptics_init_panel(ts);
		synaptics_set_report_mode(ts, ts->report_mode);

		ts->is_tp_suspended = 0;
		disable_irq_wake(client->irq);
/* OPPO 2013-05-02 huanggd Add begin for double tap*/			
		enable_irq(client->irq);
/* OPPO 2013-05-02 huanggd Add end*/	
		synaptics_set_int_mask(ts, 1);
		up(&synaptics_sem);

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W		
		// else those are not reset cause we dont get
		// a finger up event when using S2W
		s2w_exec_power_press = true;
		s2w_barrier_reached = false;
		s2w_down_x = -1;
#endif
		return 0;
	}

	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0)
			print_ts(TS_ERROR, KERN_ERR "synaptics_ts_resume power on failed\n");
	}

	synaptics_init_panel(ts);
	synaptics_set_report_mode(ts, ts->report_mode);

	ts->is_tp_suspended = 0;
	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	else
		synaptics_set_int_mask(ts, 1); /* enable abs int */

	up(&synaptics_sem);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
/* OPPO 2013-05-02 huanggd Add begin for double tap*/	
static void synaptics_ts_late_resume_power(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	int ret;
	ts = container_of(h, struct synaptics_ts_data, early_suspend_power);

#if SUPPORT_DOUBLE_TAP
	if (0 == atomic_read(&ts->double_tap_enable))
		return;
#endif
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_S2W
	if (!input_wakeup_active(ts))
		return;
#endif
	disable_irq(ts->client->irq);
	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			print_ts(TS_ERROR, KERN_ERR "synaptics_ts_late_resume_power power off failed\n");
	}
}

static int synaptics_ts_suspend_input_event(struct i2c_client *client, pm_message_t mesg)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	ts->i2c_ready = 0;
	//printk("%s\n", __func__);
	return 0;
}

static int synaptics_ts_resume_input_event(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	ts->i2c_ready = 1;
	wake_up(&ts->wait_i2c_ready);
	//printk("%s\n", __func__);
	return 0;
}
/* OPPO 2013-05-02 huanggd Add end*/	
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ SYNAPTICS_I2C_RMI_NAME, 0 },
	{ }
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#else
	.suspend	= synaptics_ts_suspend_input_event,
	.resume		= synaptics_ts_resume_input_event,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= SYNAPTICS_I2C_RMI_NAME,
	},
};

#include <linux/pcb_version.h>

static int __devinit synaptics_ts_init(void)
{
	print_ts(TS_INFO, KERN_INFO "[TSP] pcb version value: %d\n", get_pcb_version());
	if (get_pcb_version() < PCB_VERSION_EVT)
		return -EINVAL;
	print_ts(TS_INFO, KERN_INFO "[syna] synaptics s3202 init.\n");
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if (!synaptics_wq)
		return -ENOMEM;
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics S3202 Touchscreen Driver");
MODULE_LICENSE("GPL");
