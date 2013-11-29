/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#ifndef M9MO_V412_H
#define M9MO_V412_H
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <mach/camera.h>
#include <media/msm_camera.h>
#include <media/v4l2-subdev.h>
#include "msm_camera_i2c.h"
#include "msm_camera_eeprom.h"

#include <mach/vreg.h>
#include <linux/spi/spi.h>
#include "msm_camera_i2c_mux.h"
#include "msm.h"
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h> 
#include <linux/pcb_version.h>
#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include <asm/unistd.h>
#include <linux/ioctl.h>

/*gpio key include*/
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/time.h>

/*max proc cmd*/
#define MAX_CMD_INDEX				(16)

/*msm sensor info&tags*/
#define SENSOR_NAME "m9mo"
#define PLATFORM_DRIVER_NAME "msm_camera_m9mo"
#define m9mo_obj m9mo_##obj

/*kernel log switch*/
#define m9mo_debug(fmt, arg...) printk(fmt, ##arg)

/*RAW capture for IQ process*/
#define M9MO_CAPTURE_RAW

/*zsl enable*/
#define ZSL_ENABLE

char m9mo_proc_data[256] = {0x00};
char m9mo_cmd[16] = {0x00};
char m9mo_category = 0x00;
char m9mo_byte = 0x00;
char m9mo_read_data[16] = {0x00};

enum m9mo_focus_state{
	FOCUSED_SUCCESS = 0,
	FOCUSING,
	FOCUSED_FAILED,
	FOCUS_MAX,
};
enum m9mo_focus_state current_status;
enum m9mo_focus_state last_status = FOCUSING;

typedef enum
{
	M9MO_ACTION_INIT,
	M9MO_ACTION_START,
	M9MO_ACTION_STOP,
	M9MO_ACTION_MAX
}m9mo_action_state;

struct m9mo_action_struct{
	m9mo_action_state state;
	void (*init)(struct msm_sensor_ctrl_t *s_ctrl);
	void (*start)(struct msm_sensor_ctrl_t *s_ctrl);
	void (*output)(struct msm_sensor_ctrl_t *s_ctrl);
	void (*stop)(struct msm_sensor_ctrl_t *s_ctrl);
};

/*struct of gpio key for camera rotate*/
struct gpio_button_data {
	const struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	unsigned int timer_debounce;	/* in msecs */
	unsigned int irq;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct mutex disable_lock;
	unsigned int n_buttons;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	struct gpio_button_data data[0];
};

struct exif_info
{
	uint32_t exposure;
	uint32_t ISO;
	uint32_t focus_value;
	uint32_t flash_fired;
	uint32_t shutterSpeed;
	uint32_t gain;
};

struct parameter_config
{
	int8_t brightness;
	int8_t wb;
	int8_t flash_mode;
	int8_t contrast;
	int8_t antibanding;
	int8_t saturation;
	int8_t sharpness;
	int8_t ev;
	int8_t iso;
	int8_t effect;
	int8_t slow_shutter;
	int8_t ae_lock;
	int8_t awb_lock;
	int8_t zoom;
	int8_t scene;
	int8_t flip_hint;
	bool hdr_enable;
	bool asd_enable;
	bool need_update;
};

#endif

