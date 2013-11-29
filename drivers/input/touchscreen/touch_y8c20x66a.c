/************************************************************************************
** drivers/input/touchscreen/y8c20x66a_s3202_i2c_rmi.c
**
** Copyright (C), 2000-2012, OPPO Mobile Comm Corp., Ltd
** All rights reserved.
** 
** VENDOR_EDIT
** 
** Description: -
** 
** from
** drivers/input/touchscreen/y8c20x66a_i2c_rmi.c
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
** YangHai@OnlineRD.Driver.TouchScreen  2013/05/19   1.0	    create file
** ranfei@OnlineRD.Driver.TouchScreen   2013/09/23   1.1	    shut down touchpad after probe
** ranfei@OnlineRD.Driver.TouchSCreen   2013/10/10   1.2        当使能的时候，由控制复位变成控制电源
** ranfei@OnlineRD.Driver.TouchSCreen   2013/10/15   1.3        修改睡眠唤醒时断电与上电的bug
** ranfei@OnlineRD.Driver.TouchSCreen   2013/10/28   1.4        在断电之后把中断处理给取消掉
** ranfei@OnlineRD.Driver.TouchSCreen   2013/10/28   1.5        在检测到大面积的时候压力上报0，减少误操作
** ranfei@OnlineRD.Driver.TouchSCreen   2013/11/15   1.6        增加设备厂家信息
** ranfei@OnlineRD.Driver.TouchSCreen   2013/11/19   1.7        modify the /proc/devinfo/back_tp version info
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
#include <linux/y8c20x66a_i2c_rmi.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <mach/board.h>
#include "touch_y8c20x66a_microchip.h"
#include "Cypress.h"
#include <mach/device_info.h>

/*****************************************************************/

#define MAX_FINGERS		(1)

/*************** log definition **********************************/
#define TS_ERROR   1
#define TS_WARNING 2
#define TS_INFO    3
#define TS_DEBUG   4
#define TS_TRACE   5
static int syna_log_level = TS_TRACE;
#define print_ts(level, ...) \
	do { \
		if (syna_log_level >= (level)) \
			printk(__VA_ARGS__); \
	} while (0) 
/*****************************************************************/

#define MAX_READ_RETRY_COUNT		2
#define MAX_WRITE_RETRY_COUNT		1

#define CMD_MICROCHIP_ERASEAPP         1
#define CMD_MICROCHIP_WRITEAPP         2
#define CMD_MICROCHIP_READAPP          4
#define CMD_MICROCHIP_UPDATEFW         (CMD_MICROCHIP_ERASEAPP|CMD_MICROCHIP_WRITEAPP)

extern int get_boot_mode(void);

static struct workqueue_struct *y8c20x66a_wq;


struct y8c20x66a_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	bool has_relative_report;
	struct hrtimer timer;
	struct work_struct  work;
	struct mutex mutex_set_page;
	int reported_finger_count;
	uint8_t is_tp_suspended;
	int (*power)(int on);
	struct early_suspend early_suspend;
	#ifdef CYPRESS_CHIP
	uint8_t finger_data[6];
    uint8_t version2str[6];
	#endif
	#ifdef MICROCHIP_CHIP
	uint8_t finger_data[7];
	#endif
    atomic_t touch_enable;
    int reset_gpio;
};

//DEFINE_MUTEX(i2c_bus_mutex);//for GSBI1_I2C err
extern struct mutex i2c_bus_mutex;  //for GSBI1_I2C err

static struct y8c20x66a_ts_data   *syna_ts_data;
static DEFINE_SEMAPHORE(y8c20x66a_sem);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void y8c20x66a_ts_early_suspend(struct early_suspend *h);
static void y8c20x66a_ts_late_resume(struct early_suspend *h);
#endif

#ifdef CYPRESS_CHIP
ssize_t updatefw(unsigned char is_blmode);
#else
ssize_t programapp(unsigned int cmd,char *buf, size_t count);
#endif
static int writeI2C(unsigned char address, unsigned char *data, unsigned char size);
static int readI2C(unsigned char address, unsigned char *buffer, unsigned char size);

extern int display_rle_file(char *filename);

#ifdef CYPRESS_CHIP
unsigned int g_cypress_id = 0;
unsigned int g_cypress_ver = 0;
#else
unsigned int g_microchip_id =0;
unsigned int g_microchip_ver=0;
#endif

static ssize_t updatefw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    unsigned char buffer[2] ;
    
#ifdef CYPRESS_CHIP
    if(readI2C(0x04, &buffer[0], 2) == 2)
        g_cypress_ver = (buffer[0] << 8) | buffer[1];
    
    return sprintf(buf, "version=0x%x,id=0x%x\n", g_cypress_ver,g_cypress_id);
#endif
#ifdef MICROCHIP_CHIP
	return sprintf(buf, "version=0x%x,id=0x%x\n", g_microchip_ver,g_microchip_id);
#endif
}
static ssize_t updatefw_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = 0;
	if(count > 0)
	{
	   if(buf[0] >= 0x31 && buf[0] <= 0x39 ) {
	   	   val = buf[0] - 0x30 ;
#ifdef CYPRESS_CHIP
           updatefw(0);
#else
	       programapp(val,0,0) ;
#endif
	   } 
	}
	return count;
}

static DEVICE_ATTR(updatefw, S_IRUGO|S_IWUSR, updatefw_show, updatefw_store);

static struct attribute * attr_updatefw_interfaces[] = {
	&dev_attr_updatefw.attr,
	NULL,
};
static struct attribute_group y8c20x66_attr_group = {
	.name = "y8c_attr",
	.attrs = attr_updatefw_interfaces,
};

static int y8c20x66a_init_panel(struct y8c20x66a_ts_data *ts){
    int ret;
    
    ret = gpio_request(ts->reset_gpio, "TOUCH_BK_RST");
	if (ret) {
		pr_err("touch pad reset gpio[%d] request faild: %d\n", ts->reset_gpio, ret);
        return ret;
	}
	gpio_direction_output(ts->reset_gpio, 0);

	return 0;
}

static int y8c20x66a_init_params(struct y8c20x66a_ts_data *ts)
{
    int retry_count = 0;
	unsigned char data[10];
    unsigned char boot_mode;
	int ret = -1 ;
    int force_update = 0;

    print_ts(TS_ERROR, KERN_ERR "%s enter\n", __func__);
	if(sysfs_create_group(&ts->client->dev.kobj, &y8c20x66_attr_group))
		print_ts(TS_ERROR, KERN_ERR "[syna]%s: sysfs_create_group failed\n", __func__);

#ifdef CYPRESS_CHIP
retry_block:
    if(readI2C(0x00, &data[0], 10) != 10) {
        printk(KERN_ERR "read cypress device 10 bytes faild\n");
        return ret;
    }        
	g_cypress_id  = (data[8] << 8) | data[9];
    g_cypress_ver = (data[4] << 8) | data[5];
    boot_mode = !!data[0];
    if(g_cypress_ver >= 0x108 && g_cypress_ver <= 0x117)  //ranfei modify
        boot_mode = 1;
    print_ts(TS_INFO, KERN_ERR"cypress chip id is [0x%x], version is [0x%x], boot mode is [%d]\n",
        g_cypress_id, g_cypress_ver, boot_mode);

    if(g_cypress_ver != CYPRESS_FIRMWARE_VERSION || force_update){
        if(updatefw(boot_mode) != 0) {
            if (++retry_count == 3) {
    		    printk(KERN_ERR "update Cypress firmware faild\n");
                return ret;
		    } else {
		        ts->power(0);
                msleep(50);
                ts->power(1);
                msleep(50);
			    goto retry_block;
		    }
        }
        g_cypress_ver = CYPRESS_FIRMWARE_VERSION;
        return 0;
    }
    ret = 0;
#else
    if(readI2C(0x00, &buffer[0], 4) == 4) {

	    g_microchip_ver = (buffer[0] << 8) | buffer[1];
		g_microchip_id = (buffer[2] << 8) | buffer[3] ;
		print_ts(TS_INFO, KERN_ERR "y8c2:version=0x%x,id=0x%x\n",g_microchip_ver,g_microchip_id);

		if(g_microchip_ver < MICROCHIP_FIRMWARE_VERSION || g_microchip_id < MICROCHIP_FIRMWARE_ID) {
			programapp(CMD_MICROCHIP_UPDATEFW,0,0);
		}

        buffer[0] = 0x70 ;
		writeI2C(0x25, &buffer[0], 1);  //ranfei
		
		ret = 0 ;
    }
#endif

	return ret;
}
/*
static int inside(uint8_t org, uint8_t target, uint8_t tolerance) {

    if(org <= tolerance) {
        if(target >= 0 && target <= org + tolerance)
            return 1;
        else 
            return 0;
    } else if(255 - tolerance <= org) {
        if(target <= 255 && target >= org - tolerance)
            return 1;
        else 
            return 0;
    } else {
        if(target >= org - tolerance && target <= org + tolerance)
            return 1;
        else 
            return 0;
    }
}
*/
static void y8c20x66a_ts_work_func(struct work_struct *work)
{
	int ret;
    uint8_t buffer;
	struct y8c20x66a_ts_data *ts = container_of(work, struct y8c20x66a_ts_data, work);

	int finger_pressed = 0;
	struct i2c_msg msg[2];
	#ifdef CYPRESS_CHIP
	uint8_t data_start_addr = 0x01;
	#endif
	#ifdef MICROCHIP_CHIP
	uint8_t data_start_addr = 0x10;
	#endif

	down(&y8c20x66a_sem);

	if (ts->is_tp_suspended)
		goto work_func_end;
/* OPPO 2013-09-09 ranfei Add begin for reason */
    if(readI2C(0x06, &buffer, 1) != 1) {
        printk(KERN_ERR "read cypress device id faild\n");
        return;
    }
/* OPPO 2013-09-09 ranfei Add end */
   
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
	{
		print_ts(TS_ERROR, KERN_ERR "[%s]: gesture read failed.\n", __func__);
	}
	else
	{
#ifdef CYPRESS_CHIP
		finger_pressed = ts->finger_data[2] ;
#endif
#ifdef MICROCHIP_CHIP
		finger_pressed = ts->finger_data[0] &0x1;
#endif

		if (finger_pressed)
		{
#ifdef CYPRESS_CHIP
			input_report_abs(ts->input_dev, ABS_X, ts->finger_data[0]);
//            input_report_abs(ts->input_dev, ABS_X, 100);
			input_report_abs(ts->input_dev, ABS_Y, ts->finger_data[1]); 
            input_report_abs(ts->input_dev, ABS_PRESSURE, 1);
            input_report_key(ts->input_dev, BTN_TOUCH, 1); 
                
            print_ts(TS_TRACE, KERN_ERR"[%d]finger: (x,y)=(%4d,%4d)\n",buffer,
							ts->finger_data[0],  ts->finger_data[1]);			
#endif
#ifdef MICROCHIP_CHIP
			input_report_abs(ts->input_dev, ABS_X, (ts->finger_data[1]<<4)|(ts->finger_data[3]&0xf));  //ranfei
			input_report_abs(ts->input_dev, ABS_Y, (ts->finger_data[2]<<4)|((ts->finger_data[3]&0xf0)>>4));
            input_report_abs(ts->input_dev, ABS_PRESSURE, 1);
            input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif

		} else {
			print_ts(TS_TRACE, KERN_ERR"[%d]finger up^^^^^^^^^\n", buffer);
            if(ts->finger_data[5]) {
                input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
                input_sync(ts->input_dev);
            }
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
	    }
        input_sync(ts->input_dev);			
    }
#if 0
/* OPPO 2013-08-20 ranfei Add begin for reason */
{
    uint8_t readreg = 0x00;
    uint8_t data[21];
        
    msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &readreg;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(data);
	msg[1].buf = data;

	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) 
	{
		print_ts(TS_ERROR, KERN_ERR "[%s]: gesture read failed.\n", __func__);
	}
	else
	{
        printk(KERN_INFO "[%d] %d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d",
                data[6], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20]);
	}
}
/* OPPO 2013-08-20 ranfei Add end */
#endif
work_func_end:
	up(&y8c20x66a_sem);
	if (ts->use_irq)
		enable_irq(ts->client->irq);
}

static enum hrtimer_restart y8c20x66a_ts_timer_func(struct hrtimer *timer)
{
	struct y8c20x66a_ts_data *ts = container_of(timer, struct y8c20x66a_ts_data, timer);
	queue_work(y8c20x66a_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 25000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t y8c20x66a_ts_irq_handler(int irq, void *dev_id)
{
	struct y8c20x66a_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(y8c20x66a_wq, &ts->work);
	return IRQ_HANDLED;
}


static int readI2C(unsigned char address, unsigned char *buffer, unsigned char size)
{
	int ret = 0;
	int retry_count = 0;
	unsigned char i2cbuf[100] ;
	struct i2c_client *client ;
	struct i2c_msg msg[2];

	if(!syna_ts_data)
		return -1 ;

	client = syna_ts_data->client;

	mutex_lock(&syna_ts_data->mutex_set_page);
//	ret = synaptics_set_page(ts, address);
//	if (ret)
//		goto exit_byte_read;

    i2cbuf[0] = address;
	msg[0].addr = client->addr;
	msg[0].flags = 0 ;
	msg[0].len = 1;
	msg[0].buf = i2cbuf;
	
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD ;
	msg[1].len = size;
	msg[1].buf = buffer;

retry_block_read:
	if(i2c_transfer(client->adapter,msg,2)<0)
	{
		if (++retry_count == MAX_READ_RETRY_COUNT)
		{
			dev_err(&client->dev, "%s: read address 0x%04x size %d failed:%d\n",
					__func__, address, size, ret);
			ret = -1;
		}
		else
			goto retry_block_read;
	}
	else
	{
		ret = size;
	}
//exit_byte_read:
	mutex_unlock(&syna_ts_data->mutex_set_page);
	return ret;
}
static int writeI2C(unsigned char address, unsigned char *data, unsigned char size)
{
	int ret = 0;
	int retry_count = 0;
	unsigned char i2cbuf[100] ;
	struct i2c_client *client ;
	struct i2c_msg msg;
	
	if(!syna_ts_data)
		return -1 ;

	client = syna_ts_data->client; ;

	mutex_lock(&syna_ts_data->mutex_set_page);
//	ret = synaptics_set_page(ts, address);
//	if (ret)
//		goto exit_byte_write;


	memcpy(i2cbuf+1,data,size);
    i2cbuf[0] = address;
	msg.addr = client->addr;
	msg.flags = 0 ;
	msg.len = size+1;
	msg.buf = i2cbuf;

retry_block_write:
	if(i2c_transfer(client->adapter,&msg,1)<0)
	{
		if (++retry_count == MAX_WRITE_RETRY_COUNT)
		{
			dev_err(&client->dev, "%s: write address 0x%04x size %d failed:%d\n",
					__func__, address, size, ret);
			ret = -1;
		}
		else
			goto retry_block_write;
	}
	else
	{
		ret = size;
	}
//exit_byte_write:
	mutex_unlock(&syna_ts_data->mutex_set_page);
	return ret;
}

#ifdef CYPRESS_CHIP
/**
* boot_mode: when IC in bootloader mode, the boot_mode is 0, then 1;
*/
ssize_t updatefw(unsigned char boot_mode)
{
    unsigned int return_value;
	unsigned char buffer[3];
    unsigned int line;
    unsigned char key[] = {0x00, 0xFF, 0x38, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    unsigned char last_key[] = {0x00, 0xFF, 0x3B, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	
    disable_irq_nosync(syna_ts_data->client->irq);

	print_ts(TS_INFO, KERN_INFO "Entering Bootloader...\n");

	//enter bootloader mode
	if(boot_mode) {
        buffer[0] = 0x88;
        writeI2C(0x00, &buffer[0], 1);
        msleep(100);
    }   

    //write command
    writeI2C(0x00, &key[0], 11);
    msleep(300);

    //if device in bootloaer mode, no check the return data
    readI2C(0x00, &buffer[0], 3);
    return_value = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
    if(boot_mode && return_value != 0xff38) {
        printk(KERN_ERR "Cypress can not enter bootloader mode, the value is 0x%x\n", return_value);
        return -EPERM;
    }

    //write the bin
    for(line = 0; line < 92; line++) {
       writeI2C(0x00, &touch_y8c20x66a_cypress_data[line][0], 17); 
       msleep(15);
       writeI2C(0x00, &touch_y8c20x66a_cypress_data[line][17], 17);
       msleep(15);
       writeI2C(0x00, &touch_y8c20x66a_cypress_data[line][34], 17);
       msleep(15);
       writeI2C(0x00, &touch_y8c20x66a_cypress_data[line][51], 17);
       msleep(15);
       writeI2C(0x00, &touch_y8c20x66a_cypress_data[line][68], 15);
       msleep(100);
       readI2C(0x00, &buffer[0], 3);
       print_ts(TS_INFO, KERN_INFO"the return data is 0x%x, 0x%x, 0x%x, line = %d\n", buffer[0], buffer[1], buffer[2], line);
       return_value = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
       if(return_value != 0x1020) {
            printk(KERN_ERR "load the %d block faild, the return value is 0x%x\n", line, return_value);
            return -EPERM;
       }
    }

    //finish
    writeI2C(0x00, &last_key[0], 11);
    msleep(300);
	
	enable_irq(syna_ts_data->client->irq);
	
	print_ts(TS_INFO, KERN_INFO "y8c2:the update firmware had finished.\n");

	return 0 ;
}
#else
ssize_t programapp(unsigned int cmd,char *buf, size_t count)
{
	unsigned char buffer[100];
	unsigned int x ;
	//unsigned short *pp ;
	
    disable_irq_nosync(syna_ts_data->client->irq);

	print_ts(TS_INFO, KERN_INFO "Entering Bootloader[%d]...\n",cmd);

	//prepare
    buffer[0] = 0xAA;
    buffer[1] = 0x55;
    writeI2C(0x0A, &buffer[0], 2);

    buffer[0] = 0x10;
    writeI2C(0x08, &buffer[0], 1);
	msleep(250);

    readI2C(0x00, &buffer[0], 4);
	print_ts(TS_INFO, KERN_INFO "Bootloader Version: 0x%x,ID:0x%x\n", \
		(buffer[0]<<8)|buffer[1],(buffer[2]<<8)|buffer[3]);
	
	//erase
	if(cmd & CMD_MICROCHIP_ERASEAPP) {
		buffer[0] = 0xe0;
		buffer[1] = 0x3f; 
	    writeI2C(0x11, &buffer[0], 2);
	    buffer[0] = 0x22;
	    writeI2C(0x10, &buffer[0], 1);
		
		for(x=0x800;x<0x3fe0;x+=0x20)    {       
	        buffer[0] = x&0x00ff;
	        buffer[1] = x>>8;
	        writeI2C(0x11, &buffer[0], 2);

	        buffer[0] = 0x22;
	        writeI2C(0x10, &buffer[0], 1);
		}
		print_ts(TS_INFO, KERN_INFO "Erase has done...\n");
	}

	//program app
	if(cmd & CMD_MICROCHIP_WRITEAPP) {
		for( x = 0 ; x < sizeof(touch_y8c20x66a_microchip_data)/34; x++) {
	        writeI2C(0x11, (unsigned char*)touch_y8c20x66a_microchip_data+x*34, 34);

	        buffer[0] = 0x21;
	        writeI2C(0x10, &buffer[0], 1);
		}
		print_ts(TS_INFO, KERN_INFO "Programing has done...\n");

		//erase force boot flag - dont do for now, safer
	    msleep(1) ;
	    buffer[0] = 0xC0;
	    buffer[1] = 0x3F;
	    writeI2C(0x11, &buffer[0], 2);
	    msleep(1);
	    buffer[0] = 0x22;
	    writeI2C(0x10, &buffer[0], 1);
	    msleep(1);

	}

	
	enable_irq(syna_ts_data->client->irq);
	
	print_ts(TS_INFO, KERN_INFO "y8c2:the update firmware had finished.\n");

	return 0 ;
}
#endif

static int touch_pad_enable_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	struct y8c20x66a_ts_data *ts = data;
	
	return sprintf(page, "%d\n", atomic_read(&ts->touch_enable));
}

static int touch_pad_enable_proc_write( struct file *filp, const char __user *buff,
                        unsigned long len, void *data )
{
	unsigned int val = 0;
	struct y8c20x66a_ts_data *ts = data;
	char buf[10];

	if (len > 10) 
		return len;
	
	if (copy_from_user( buf, buff, len )) {
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return len;
	}
	
	sscanf(buf, "%d", &val);
	val = (val == 0 ? 0:1);
	
	if ((val == 1) && atomic_read(&ts->touch_enable) == 0) {
        mutex_lock(&i2c_bus_mutex);//for GSBI1_I2C err
        ts->power(1);
		atomic_set(&ts->touch_enable, val);
        mutex_unlock(&i2c_bus_mutex);//for GSBI1_I2C err
        if (ts->use_irq)
		    enable_irq(ts->client->irq);

	    if (!ts->use_irq)
		    hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        
		print_ts(TS_INFO, KERN_INFO "%s: touch pad enable \n", __func__);
	} else if ((val == 0) && atomic_read(&ts->touch_enable) == 1) {
	    if (ts->use_irq)
		    disable_irq(ts->client->irq);
	    else
		    hrtimer_cancel(&ts->timer);
        ts->power(0);
		atomic_set(&ts->touch_enable, val);

		print_ts(TS_INFO, KERN_INFO "%s: set touch pad disable\n", __func__);
	}
	return len;
}

static int touch_pad_register_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
    unsigned char buf[21];

    if(readI2C(0x00, buf, 21) != 21){
        printk(KERN_ERR "read reg[0x00] 20 bytes faild\n");
    }
   
	return sprintf(page, "reg[0-9]: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\nreg[10-20]: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
                   buf[0],  buf[1],  buf[2],  buf[3],  buf[4],  buf[5],  buf[6],  buf[7],  buf[8],  buf[9], 
                   buf[10],  buf[11],  buf[12],  buf[13],  buf[14],  buf[15],  buf[16],  buf[17],  buf[18],  buf[19],  buf[20]);
}

static int touch_pad_rawdata_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
    int i, j;
    int rawdata[11] = {0};
    unsigned char buf[22];
    unsigned char reg = 0x15;

    for(i = 0; i < 8; i++) {
        if(readI2C(reg, buf, 22) != 22){
            printk(KERN_ERR "read reg[%d] 22 bytes faild\n", reg);
        }

        for(j = 0; j < 11; j++){
            rawdata[j] += buf[j << 1] << 8 | buf[(j << 1) + 1];
        }

        msleep(15);
    }

    for(j = 0; j < 11; j++) {
        rawdata[j] = rawdata[j] >> 3;
    }
   
	return sprintf(page, "%d*%d*%d*%d*%d*%d*%d*%d*%d*%d*%d\n",
                   rawdata[0],  rawdata[1],  rawdata[2],  rawdata[3],  rawdata[4],  rawdata[5],  rawdata[6],
                   rawdata[7],  rawdata[8],  rawdata[9],  rawdata[10]);
}

static int touch_pad_log_level_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	return sprintf(page, "%d\n", syna_log_level);
}

static int touch_pad_log_level_proc_write( struct file *filp, const char __user *buff,
                        unsigned long len, void *data )
{
	unsigned int val = 0;
	char buf[10];

	if (len > 10) 
		return len;
	
	if (copy_from_user( buf, buff, len )) {
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return len;
	}
	
	sscanf(buf, "%d", &val);	
	syna_log_level = val;
    
	return len;
}

extern struct proc_dir_entry proc_root;

static int init_touch_proc(struct y8c20x66a_ts_data *ts)
{
	int ret = 0;
    
	struct proc_dir_entry *prcdir;
    struct proc_dir_entry *proc_entry;

	prcdir = proc_mkdir("touchpad", &proc_root);
	if (prcdir == NULL) {
		printk(KERN_ERR "%s: can't create /proc/touchpad\n", __func__);
		return -ENOMEM;
	}
    
	proc_entry = create_proc_entry("enable", 0666, prcdir);
	if (proc_entry) {
		proc_entry->write_proc = touch_pad_enable_proc_write;
		proc_entry->read_proc = touch_pad_enable_proc_read;
		proc_entry->data = ts;
	}

    proc_entry = create_proc_entry("register", 0444, prcdir);
	if (proc_entry) {
		proc_entry->read_proc = touch_pad_register_proc_read;
		proc_entry->data = ts;
	}

    proc_entry = create_proc_entry("rawdata", 0444, prcdir);
	if (proc_entry) {
		proc_entry->read_proc = touch_pad_rawdata_proc_read;
		proc_entry->data = ts;
	}

    proc_entry = create_proc_entry("log_level", 0666, prcdir);
	if (proc_entry) {
		proc_entry->read_proc = touch_pad_log_level_proc_read;
        proc_entry->write_proc = touch_pad_log_level_proc_write;
		proc_entry->data = ts;
	}

    return ret;
}

static int y8c20x66a_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct y8c20x66a_ts_data *ts;
	int ret = 0;
	struct y8c20x66a_i2c_rmi_platform_data *pdata;
	unsigned long irqflags=0;
	
#ifdef CONFIG_VENDOR_EDIT
/*OPPO 2013.09.28 hewei modify begin for recovery mode do not use touchscreen*/
    if(get_boot_mode() == MSM_BOOT_MODE__RECOVERY)
        return -EPERM;
/*OPPO 2013.09.28 hewei modify end for recovery mode do not use touchscreen*/
#endif

	print_ts(TS_INFO, KERN_ERR "y8c20x66a_ts_probe+++\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		print_ts(TS_ERROR, KERN_ERR "y8c20x66a_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	syna_ts_data = ts;

	INIT_WORK(&ts->work, y8c20x66a_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata) {
		ts->power = pdata->power;
        ts->reset_gpio = pdata->reset_gpio;
	}

	ts->is_tp_suspended = 0;
	mutex_init(&(ts->mutex_set_page));

	if (pdata) {
		irqflags = pdata->irqflags;
	} 

	if(y8c20x66a_init_panel(ts) != 0) 
        goto err_input_dev_alloc_failed ;
    
	if(y8c20x66a_init_params(ts) != 0)
		goto err_input_dev_alloc_failed ;

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		print_ts(TS_ERROR, KERN_ERR "y8c20x66a_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = y8c20x66a_I2C_RMI_NAME;//"y8c20x66a touchscreen s3202";

    set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
    set_bit(EV_KEY, ts->input_dev->evbit);

    __set_bit(ABS_X, ts->input_dev->absbit);
	__set_bit(ABS_Y, ts->input_dev->absbit);
	__set_bit(ABS_PRESSURE, ts->input_dev->absbit);

    ts->input_dev->keybit[BIT_WORD(BTN_DIGI)] = BIT_MASK(BTN_TOUCH);
    
	#ifdef CYPRESS_CHIP
    sprintf(ts->version2str, "0x%03x", g_cypress_ver);
    register_device_proc("back_tp", ts->version2str, "Cypress");
	input_set_abs_params(ts->input_dev, ABS_X, 0, 330, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, 480, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 512, 0, 0);
	#endif
	#ifdef MICROCHIP_CHIP
    register_device_proc("back_tp", "PIC16F", "Microchip");
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 384, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 320, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 512, 0, 0);
	#endif

	ret = input_register_device(ts->input_dev);
	if (ret) {
		print_ts(TS_ERROR, KERN_ERR "y8c20x66a_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	if (client->irq) {
		ret = request_irq(client->irq, y8c20x66a_ts_irq_handler, irqflags, client->name, ts);

		if (ret == 0) {
			ts->use_irq = 1;
		}
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = y8c20x66a_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 2;
	ts->early_suspend.suspend = y8c20x66a_ts_early_suspend;
	ts->early_suspend.resume = y8c20x66a_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

    init_touch_proc(ts);

    atomic_set(&ts->touch_enable, 0);
    if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer); 
    if (ts->power)
		ts->power(0);

	syna_log_level = TS_INFO;

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	if (ts->power)
		ts->power(0);
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int y8c20x66a_ts_remove(struct i2c_client *client)
{
	struct y8c20x66a_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int y8c20x66a_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret=0;
	struct y8c20x66a_ts_data *ts = i2c_get_clientdata(client);
	down(&y8c20x66a_sem);
    
	ts->is_tp_suspended = 1;
    
	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);

	if (ret < 0)
		print_ts(TS_ERROR, KERN_ERR "%s: can not disable interrupt\n", __func__);

    if(atomic_read(&ts->touch_enable) == 1) {
	    if (ts->power) {
		    ret = ts->power(0);
		    if (ret < 0)
			    print_ts(TS_ERROR, KERN_ERR "y8c20x66a_ts_resume power off failed\n");
	    }
	}
    
	up(&y8c20x66a_sem);

	return 0;
}

static int y8c20x66a_ts_resume(struct i2c_client *client)
{
	int ret;
	struct y8c20x66a_ts_data *ts = i2c_get_clientdata(client);

	down(&y8c20x66a_sem);

    if(atomic_read(&ts->touch_enable) == 1) {
	    if (ts->power) {
            mutex_lock(&i2c_bus_mutex);//for GSBI1_I2C err
		    ret = ts->power(1);
            mutex_unlock(&i2c_bus_mutex);//for GSBI1_I2C err
		    if (ret < 0)
			    print_ts(TS_ERROR, KERN_ERR "y8c20x66a_ts_resume power on failed\n");
	    }
	}

	ts->is_tp_suspended = 0;
    
	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	up(&y8c20x66a_sem);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void y8c20x66a_ts_early_suspend(struct early_suspend *h)
{
	struct y8c20x66a_ts_data *ts;
	ts = container_of(h, struct y8c20x66a_ts_data, early_suspend);
	y8c20x66a_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void y8c20x66a_ts_late_resume(struct early_suspend *h)
{
	struct y8c20x66a_ts_data *ts;
	ts = container_of(h, struct y8c20x66a_ts_data, early_suspend);
	y8c20x66a_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id y8c20x66a_ts_id[] = {
	{ y8c20x66a_I2C_RMI_NAME, 0 },
	{ }
};

static struct i2c_driver y8c20x66a_ts_driver = {
	.probe		= y8c20x66a_ts_probe,
	.remove		= y8c20x66a_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= y8c20x66a_ts_suspend,
	.resume		= y8c20x66a_ts_resume,
#endif
	.id_table	= y8c20x66a_ts_id,
	.driver = {
		.name	= y8c20x66a_I2C_RMI_NAME,
	},
};


static int __devinit y8c20x66a_ts_init(void)
{

	y8c20x66a_wq = create_singlethread_workqueue("y8c20x66a_wq");
	if (!y8c20x66a_wq)
		return -ENOMEM;
	
	return i2c_add_driver(&y8c20x66a_ts_driver);
}

static void __exit y8c20x66a_ts_exit(void)
{
	i2c_del_driver(&y8c20x66a_ts_driver);
	if (y8c20x66a_wq)
		destroy_workqueue(y8c20x66a_wq);
}

module_init(y8c20x66a_ts_init);
module_exit(y8c20x66a_ts_exit);

MODULE_DESCRIPTION("y8c20x66a S3202 Touchscreen Driver");
MODULE_LICENSE("GPL");

