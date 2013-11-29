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
#ifndef OPPO_FEATURE_H
#define OPPO_FEATURE_H
#include "msm_sensor.h"


enum OPPO_FEATURE{
    OPPO_CAF = 0,
    OPPO_ASD,
    MAX_FEATURE,
};

typedef enum
{
	FORCE_CAF_START,
	FORCE_CAF_STOP,
}m9mo_caf_result;

typedef enum
{
	M9MO_AE_MODE_AUTO,
	M9MO_AE_MODE_TOUCH,
	M9MO_AE_MODE_FACE,
	M9MO_AE_MODE_MAX
}m9mo_ae_mode;

struct frame_info_t
{
	int16_t brightness;
	u_int32_t wave_detect;
	u_int32_t exp_time;
	u_int32_t shutter;
	u_int16_t wb;
	u_int8_t  af_position;
	u_int16_t gain;
	bool capture_start;
	bool single_af;
	bool wd_valid;
	bool ae_stable;
	bool focus_done;
	m9mo_caf_result caf_result;
	bool start_moving;
	bool has_faces;
	m9mo_ae_mode ae_mode;
};

#define WDV_CAF_LOW_THR 				(2300)
#define ANTI_WDV_SHAKE_DELAY 			(15)
#define DO_AF_DELAY 					(0)

#define CAM_START_DELAY_FRAME			(6)
#define WD_VALID_DELAY_FRAME			(30)
#define CAP_DELAY_FRAME					(20)
#define SINGLE_FOCUS_DETECT_FRAME		(30)

struct oppo_interface{
   void (*notify)(struct msm_sensor_ctrl_t*,  int32_t);
   void (*process)(struct msm_sensor_ctrl_t*, struct frame_info_t*);
};

int32_t oppo_scene_detect_init(struct oppo_interface *interface);
int32_t oppo_caf_init(struct oppo_interface *interface);

#endif

