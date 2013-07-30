/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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
#include "msm_sensor.h"
#define SENSOR_NAME "s5k6a3yx"
#define PLATFORM_DRIVER_NAME "msm_camera_s5k6a3yx"
#define s5k6a3yx_obj s5k6a3yx_##obj

DEFINE_MUTEX(s5k6a3yx_mut);
static struct msm_sensor_ctrl_t s5k6a3yx_s_ctrl;

static struct msm_camera_i2c_reg_conf s5k6a3yx_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf s5k6a3yx_stop_settings[] = {
	{0x0100, 0x00},
};

static struct msm_camera_i2c_reg_conf s5k6a3yx_groupon_settings[] = {
	{0x0104, 0x01},
};

static struct msm_camera_i2c_reg_conf s5k6a3yx_groupoff_settings[] = {
	{0x0104, 0x00},
};

static struct msm_camera_i2c_reg_conf s5k6a3yx_prev_settings[] = {
/* OPPO 2012-11-22 yxq Modify begin for full-size preview */
#if 0
        //####################################################
        //#  name:  S5K6A3YX EVT2 setfile
        //#  v0.54 Temporary version
        //#  MSR setting could be changed
        //#  Ext CLK : 9.6Mhz , 60fps, size 1412x796
        //####################################################

        //# Streaming off
        //{0x0100, 0x00}, //# mode_select				(default)

        //$MIPI[width:1412, height:796, lane:1, format:raw10, datarate:796]

        //analog setting(Line length pck : 1602d)
        {0x3061, 0x55},   
        {0x3062, 0x54},   
        {0x5703, 0x07},   
        {0x5704, 0x07},   
        {0x305e, 0x0D},   
        {0x305f, 0x2E},   
        {0x3052, 0x01},   
        {0x300B, 0x28},   
        {0x300C, 0x2E},   
        {0x3004, 0x0a},   
        {0x5700, 0x08},   
        {0x3005, 0x3D},   
        {0x3008, 0x1E},   
        {0x3025, 0x40},   
        {0x3023, 0x20},   
        {0x3029, 0xFF}, 
        {0x302A, 0xFF}, 
        {0x3505, 0x41}, 
        {0x3095, 0x15}, 
        {0x3506, 0x00}, 
        {0x3521, 0x01}, 
        {0x3522, 0x01}, 
        {0x3D20, 0x63},
        {0x3110, 0x01},	
        {0x3111, 0x62},
        {0x3112, 0x0E},
        {0x3113, 0xBC},
        {0x311D, 0x30},
        {0x311F, 0x40},
        {0x0138, 0x00},
        {0x3009, 0x1E}, 

        //# PLL setting for 796MHz, Frame rate 60fps
        {0x0305, 0x06}, //# pre_pll_clk_div			(default)
        {0x0306, 0x00}, //# pll_multiplier MSB			(default)
        {0x0307, 0xC7}, //# pll_multiplier LSB

        {0x0301, 0x0a}, //# vt_pix_clk_div			(default)
        {0x0303, 0x01}, //# vt_sys_clk_div			(default)
        {0x0309, 0x0a}, //# op_pix_clk_div			(default)
        {0x030b, 0x01}, //# op_sys_clk_div			(default)

        {0x0820, 0x03}, //# requested_link_bit_rate_mbps MSB MSB
        {0x0821, 0x1c}, //# requested_link_bit_rate_mbps MSB LSB
        {0x0822, 0x00}, //# requested_link_bit_rate_mbps LSB MSB
        {0x0823, 0x00}, //# requested_link_bit_rate_mbps LSB LSB

        {0x0101, 0x00}, //# image_orientation			(default)
        //{0x0101, 0x01}, //# image_orientation

        //# Select output mode  0:CCP2(Data/Clock), 1:CCP2(Data/Strobe), 2:CSI2
        {0x0111, 0x02}, //# CSI_signaling_mode			(default)

        //# Output data type : Raw10
        {0x0112, 0x0a}, //# CSI_data_format MSB			(default)
        {0x0113, 0x0a}, //# CSI_data_format LSB			(default)

        {0x0136, 0x18}, //# extclk_frequency_mhz MSB		(default)
        {0x0137, 0x00}, //# extclk_frequency_mhz LSB		(default)

        //# Fine integration time : 467d
        {0x0200, 0x01}, //# fine_integration_time MSB		
        {0x0201, 0xD3}, //# fine_integration_time LSB

        //# Coarse integration time : 522d
        {0x0202, 0x02}, //# coarse_integration_time MSB		(default)
        {0x0203, 0x0a}, //# coarse_integration_time LSB		(default)

        //# Analog gain x1
        {0x0204, 0x00}, //# analogue_gain_code_global MSB		(default)
        {0x0205, 0x20}, //# analogue_gain_code_global LSB		(default)

        //# Frame length lines : 828d
        {0x0340, 0x03}, //# frame_length_lines MSB
        {0x0341, 0x3c}, //# frame_length_lines LSB

        //# Line length pck : 1602d
        {0x0342, 0x06}, //# line_length_pck MSB			(default)
        {0x0343, 0x42}, //# line_length_pck LSB			(default)

        //# Size setting : 1412x796
        {0x0344, 0x00}, //# x_addr_start MSB			(default)
        {0x0345, 0x00}, //# x_addr_start LSB			(default)
        {0x0348, 0x05}, //# x_addr_end MSB			(default)
        {0x0349, 0x83}, //# x_addr_end LSB			(default)
        {0x034c, 0x05}, //# x_output_size MSB			(default)
        {0x034d, 0x84}, //# x_output_size LSB			(default)

        {0x0346, 0x01}, //# y_addr_start MSB
        {0x0347, 0x34}, //# y_addr_start LSB
        {0x034a, 0x04}, //# y_addr_end MSB
        {0x034b, 0x4f}, //# y_addr_end LSB
        {0x034e, 0x03}, //# y_output_size MSB
        {0x034f, 0x1c}, //# y_output_size LSB

        //# Sub-sampling
        {0x0381, 0x01}, //# x_even_inc				(default)
        {0x0383, 0x01}, //# x_odd_inc				(default)
        {0x0385, 0x01}, //# y_even_inc				(default)
        {0x0387, 0x01}, //# y_odd_inc				(default)

        //# Digital crop setting
        {0x0408, 0x00}, //# digital_crop_x_offset MSB		(default)
        {0x0409, 0x00}, //# digital_crop_x_offset LSB		(default)
        {0x040a, 0x00}, //# digital_crop_y_offset MSB		(default)
        {0x040b, 0x00}, //# digital_crop_y_offset LSB		(default)
        {0x040c, 0x05}, //# digital_crop_image_width MSB		(default)
        {0x040d, 0x84}, //# digital_crop_image_width LSB		(default)
        {0x040e, 0x03}, //# digital_crop_image_height MSB
        {0x040f, 0x1c}, //# digital_crop_image_height LSB

        //# Analog Binning
        {0x0900, 0x00}, //# binning_mode				(default)

        //# Streaming on
        //{0x0100, 0x01}, //# mode_select
#else
		//####################################################
		//#  name:	S5K6A3YX EVT2 setfile
		//#  v0.54 Temporary version
		//#  MSR setting could be changed
		//#  Ext CLK : 9.6Mhz , 60fps, size 1412x796
		//####################################################

		//# Streaming off
		//{0x0100, 0x00}, //# mode_select 			(default)

		//$MIPI[width:1412, height:1412, lane:1, format:raw10, datarate:796]

		//analog setting(Line length pck : 1602d)
		{0x3061, 0x55},   
		{0x3062, 0x54},   
		{0x5703, 0x07},   
		{0x5704, 0x07},   
		{0x305e, 0x0D},   
		{0x305f, 0x2E},   
		{0x3052, 0x01},   
		{0x300B, 0x28},   
		{0x300C, 0x2E},   
		{0x3004, 0x0a},   
		{0x5700, 0x08},   
		{0x3005, 0x3D},   
		{0x3008, 0x1E},   
		{0x3025, 0x40},   
		{0x3023, 0x20},   
		{0x3029, 0xFF}, 
		{0x302A, 0xFF}, 
		{0x3505, 0x41}, 
		{0x3095, 0x15}, 
		{0x3506, 0x00}, 
		{0x3521, 0x01}, 
		{0x3522, 0x01}, 
		{0x3D20, 0x63},
		{0x3110, 0x01}, 
		{0x3111, 0x62},
		{0x3112, 0x0E},
		{0x3113, 0xBC},
		{0x311D, 0x30},
		{0x311F, 0x40},
		{0x0138, 0x00},
		{0x3009, 0x1E}, 

		//# PLL setting for 796MHz, Frame rate 60fps
		{0x0305, 0x06}, //# pre_pll_clk_div 		(default)
		{0x0306, 0x00}, //# pll_multiplier MSB			(default)
		{0x0307, 0xC7}, //# pll_multiplier LSB

		{0x0301, 0x0a}, //# vt_pix_clk_div			(default)
		{0x0303, 0x01}, //# vt_sys_clk_div			(default)
		{0x0309, 0x0a}, //# op_pix_clk_div			(default)
		{0x030b, 0x01}, //# op_sys_clk_div			(default)

		{0x0820, 0x03}, //# requested_link_bit_rate_mbps MSB MSB
		{0x0821, 0x1c}, //# requested_link_bit_rate_mbps MSB LSB
		{0x0822, 0x00}, //# requested_link_bit_rate_mbps LSB MSB
		{0x0823, 0x00}, //# requested_link_bit_rate_mbps LSB LSB

		{0x0101, 0x00}, //# image_orientation			(default)

		//# Select output mode	0:CCP2(Data/Clock), 1:CCP2(Data/Strobe), 2:CSI2
		{0x0111, 0x02}, //# CSI_signaling_mode			(default)

		//# Output data type : Raw10
		{0x0112, 0x0a}, //# CSI_data_format MSB 		(default)
		{0x0113, 0x0a}, //# CSI_data_format LSB 		(default)

		{0x0136, 0x18}, //# extclk_frequency_mhz MSB		(default)
		{0x0137, 0x00}, //# extclk_frequency_mhz LSB		(default)

		//# Fine integration time : 467d
		{0x0200, 0x01}, //# fine_integration_time MSB		
		{0x0201, 0xD3}, //# fine_integration_time LSB

		//# Coarse integration time : 522d
		{0x0202, 0x02}, //# coarse_integration_time MSB 	(default)
		{0x0203, 0x0a}, //# coarse_integration_time LSB 	(default)

		//# Analog gain x1
		{0x0204, 0x00}, //# analogue_gain_code_global MSB		(default)
		{0x0205, 0x20}, //# analogue_gain_code_global LSB		(default)

		//# Frame length lines : 828d
		{0x0340, 0x05}, //# frame_length_lines MSB
		{0x0341, 0xA4}, //# frame_length_lines LSB

		//# Line length pck : 1602d
		{0x0342, 0x06}, //# line_length_pck MSB 		(default)
		{0x0343, 0x42}, //# line_length_pck LSB 		(default)

		//# Size setting : 1412x796
		{0x0344, 0x00}, //# x_addr_start MSB			(default)
		{0x0345, 0x00}, //# x_addr_start LSB			(default)
		{0x0348, 0x05}, //# x_addr_end MSB			(default)
		{0x0349, 0x83}, //# x_addr_end LSB			(default)
		{0x034c, 0x05}, //# x_output_size MSB			(default)
		{0x034d, 0x84}, //# x_output_size LSB			(default)

		{0x0346, 0x00}, //# y_addr_start MSB
		{0x0347, 0x00}, //# y_addr_start LSB
		{0x034a, 0x05}, //# y_addr_end MSB
		{0x034b, 0x83}, //# y_addr_end LSB
		{0x034e, 0x05}, //# y_output_size MSB
		{0x034f, 0x84}, //# y_output_size LSB

		//# Sub-sampling
		{0x0381, 0x01}, //# x_even_inc				(default)
		{0x0383, 0x01}, //# x_odd_inc				(default)
		{0x0385, 0x01}, //# y_even_inc				(default)
		{0x0387, 0x01}, //# y_odd_inc				(default)

		//# Digital crop setting
		{0x0408, 0x00}, //# digital_crop_x_offset MSB		(default)
		{0x0409, 0x00}, //# digital_crop_x_offset LSB		(default)
		{0x040a, 0x00}, //# digital_crop_y_offset MSB		(default)
		{0x040b, 0x00}, //# digital_crop_y_offset LSB		(default)
		{0x040c, 0x05}, //# digital_crop_image_width MSB		(default)
		{0x040d, 0x84}, //# digital_crop_image_width LSB		(default)
		{0x040e, 0x05}, //# digital_crop_image_height MSB
		{0x040f, 0x84}, //# digital_crop_image_height LSB

		//# Analog Binning
		{0x0900, 0x00}, //# binning_mode				(default)

		//# Streaming on
		//{0x0100, 0x01}, //# mode_select
#endif
/* OPPO 2012-11-22 yxq Modify end */
};

static struct msm_camera_i2c_reg_conf s5k6a3yx_snap_settings[] = {
/* OPPO 2012-10-25 yxq modified begin for full size */
#if 0
        //####################################################
        //#  name:  S5K6A3YX EVT2 setfile
        //#  v0.54 Temporary version
        //#  MSR setting could be changed
        //#  Ext CLK : 9.6Mhz , 60fps, size 1412x796
        //####################################################

        //# Streaming off
        //{0x0100, 0x00}, //# mode_select				(default)

        //$MIPI[width:1412, height:796, lane:1, format:raw10, datarate:796]

        //analog setting(Line length pck : 1602d)
        {0x3061, 0x55},   
        {0x3062, 0x54},   
        {0x5703, 0x07},   
        {0x5704, 0x07},   
        {0x305e, 0x0D},   
        {0x305f, 0x2E},   
        {0x3052, 0x01},   
        {0x300B, 0x28},   
        {0x300C, 0x2E},   
        {0x3004, 0x0a},   
        {0x5700, 0x08},   
        {0x3005, 0x3D},   
        {0x3008, 0x1E},   
        {0x3025, 0x40},   
        {0x3023, 0x20},   
        {0x3029, 0xFF}, 
        {0x302A, 0xFF}, 
        {0x3505, 0x41}, 
        {0x3095, 0x15}, 
        {0x3506, 0x00}, 
        {0x3521, 0x01}, 
        {0x3522, 0x01}, 
        {0x3D20, 0x63},
        {0x3110, 0x01},	
        {0x3111, 0x62},
        {0x3112, 0x0E},
        {0x3113, 0xBC},
        {0x311D, 0x30},
        {0x311F, 0x40},
        {0x0138, 0x00},
        {0x3009, 0x1E}, 

        //# PLL setting for 796MHz, Frame rate 60fps
        {0x0305, 0x06}, //# pre_pll_clk_div			(default)
        {0x0306, 0x00}, //# pll_multiplier MSB			(default)
        {0x0307, 0xC7}, //# pll_multiplier LSB

        {0x0301, 0x0a}, //# vt_pix_clk_div			(default)
        {0x0303, 0x01}, //# vt_sys_clk_div			(default)
        {0x0309, 0x0a}, //# op_pix_clk_div			(default)
        {0x030b, 0x01}, //# op_sys_clk_div			(default)

        //{0x0820, 0x04}, //# requested_link_bit_rate_mbps MSB MSB
        //{0x0821, 0x24}, //# requested_link_bit_rate_mbps MSB LSB
	{0x0820, 0x03}, //# requested_link_bit_rate_mbps MSB MSB
        {0x0821, 0x1C}, //# requested_link_bit_rate_mbps MSB LSB
        {0x0822, 0x00}, //# requested_link_bit_rate_mbps LSB MSB
        {0x0823, 0x00}, //# requested_link_bit_rate_mbps LSB LSB

        //{0x0101, 0x00}, //# image_orientation			(default)
	{0x0101, 0x00}, //# image_orientation

        //# Select output mode  0:CCP2(Data/Clock), 1:CCP2(Data/Strobe), 2:CSI2
        {0x0111, 0x02}, //# CSI_signaling_mode			(default)

        //# Output data type : Raw10
        {0x0112, 0x0a}, //# CSI_data_format MSB			(default)
        {0x0113, 0x0a}, //# CSI_data_format LSB			(default)

        {0x0136, 0x18}, //# extclk_frequency_mhz MSB		(default)
        {0x0137, 0x00}, //# extclk_frequency_mhz LSB		(default)

        //# Fine integration time : 467d
        {0x0200, 0x01}, //# fine_integration_time MSB		
        {0x0201, 0xD3}, //# fine_integration_time LSB

        //# Coarse integration time : 522d
        {0x0202, 0x02}, //# coarse_integration_time MSB		(default)
        {0x0203, 0x0a}, //# coarse_integration_time LSB		(default)

        //# Analog gain x1
        {0x0204, 0x00}, //# analogue_gain_code_global MSB		(default)
        {0x0205, 0x20}, //# analogue_gain_code_global LSB		(default)

        //# Frame length lines : 828d
        {0x0340, 0x04}, //# frame_length_lines MSB
        {0x0341, 0x44}, //# frame_length_lines LSB

        //# Line length pck : 1602d
        {0x0342, 0x06}, //# line_length_pck MSB			(default)
        {0x0343, 0x42}, //# line_length_pck LSB			(default)

        //# Size setting : 1412x796
        {0x0344, 0x00}, //# x_addr_start MSB			(default)
        {0x0345, 0x00}, //# x_addr_start LSB			(default)
        {0x0348, 0x05}, //# x_addr_end MSB			(default)
        {0x0349, 0x83}, //# x_addr_end LSB			(default)
        {0x034c, 0x05}, //# x_output_size MSB			(default)
        {0x034d, 0x84}, //# x_output_size LSB			(default)

        {0x0346, 0x00}, //# y_addr_start MSB
        {0x0347, 0xB0}, //# y_addr_start LSB
        {0x034a, 0x04}, //# y_addr_end MSB
        {0x034b, 0xD3}, //# y_addr_end LSB
        {0x034e, 0x04}, //# y_output_size MSB
        {0x034f, 0x24}, //# y_output_size LSB

        //# Sub-sampling
        {0x0381, 0x01}, //# x_even_inc				(default)
        {0x0383, 0x01}, //# x_odd_inc				(default)
        {0x0385, 0x01}, //# y_even_inc				(default)
        {0x0387, 0x01}, //# y_odd_inc				(default)

        //# Digital crop setting
        {0x0408, 0x00}, //# digital_crop_x_offset MSB		(default)
        {0x0409, 0x00}, //# digital_crop_x_offset LSB		(default)
        {0x040a, 0x00}, //# digital_crop_y_offset MSB		(default)
        {0x040b, 0x00}, //# digital_crop_y_offset LSB		(default)
        {0x040c, 0x05}, //# digital_crop_image_width MSB		(default)
        {0x040d, 0x84}, //# digital_crop_image_width LSB		(default)
        //{0x040e, 0x03}, //# digital_crop_image_height MSB
        //{0x040f, 0x1c}, //# digital_crop_image_height LSB
	{0x040e, 0x04}, //# digital_crop_image_height MSB
        {0x040f, 0x24}, //# digital_crop_image_height LSB

        //# Analog Binning
        {0x0900, 0x00}, //# binning_mode				(default)

        //# Streaming on
        //{0x0100, 0x01}, //# mode_select
#else
		//####################################################
		//#  name:	S5K6A3YX EVT2 setfile
		//#  v0.54 Temporary version
		//#  MSR setting could be changed
		//#  Ext CLK : 9.6Mhz , 60fps, size 1412x796
		//####################################################

		//# Streaming off
		//{0x0100, 0x00}, //# mode_select 			(default)

		//$MIPI[width:1412, height:1412, lane:1, format:raw10, datarate:796]

		//analog setting(Line length pck : 1602d)
		{0x3061, 0x55},   
		{0x3062, 0x54},   
		{0x5703, 0x07},   
		{0x5704, 0x07},   
		{0x305e, 0x0D},   
		{0x305f, 0x2E},   
		{0x3052, 0x01},   
		{0x300B, 0x28},   
		{0x300C, 0x2E},   
		{0x3004, 0x0a},   
		{0x5700, 0x08},   
		{0x3005, 0x3D},   
		{0x3008, 0x1E},   
		{0x3025, 0x40},   
		{0x3023, 0x20},   
		{0x3029, 0xFF}, 
		{0x302A, 0xFF}, 
		{0x3505, 0x41}, 
		{0x3095, 0x15}, 
		{0x3506, 0x00}, 
		{0x3521, 0x01}, 
		{0x3522, 0x01}, 
		{0x3D20, 0x63},
		{0x3110, 0x01}, 
		{0x3111, 0x62},
		{0x3112, 0x0E},
		{0x3113, 0xBC},
		{0x311D, 0x30},
		{0x311F, 0x40},
		{0x0138, 0x00},
		{0x3009, 0x1E}, 

		//# PLL setting for 796MHz, Frame rate 60fps
		{0x0305, 0x06}, //# pre_pll_clk_div 		(default)
		{0x0306, 0x00}, //# pll_multiplier MSB			(default)
		{0x0307, 0xC7}, //# pll_multiplier LSB

		{0x0301, 0x0a}, //# vt_pix_clk_div			(default)
		{0x0303, 0x01}, //# vt_sys_clk_div			(default)
		{0x0309, 0x0a}, //# op_pix_clk_div			(default)
		{0x030b, 0x01}, //# op_sys_clk_div			(default)

		{0x0820, 0x03}, //# requested_link_bit_rate_mbps MSB MSB
		{0x0821, 0x1c}, //# requested_link_bit_rate_mbps MSB LSB
		{0x0822, 0x00}, //# requested_link_bit_rate_mbps LSB MSB
		{0x0823, 0x00}, //# requested_link_bit_rate_mbps LSB LSB

		{0x0101, 0x00}, //# image_orientation			(default)

		//# Select output mode	0:CCP2(Data/Clock), 1:CCP2(Data/Strobe), 2:CSI2
		{0x0111, 0x02}, //# CSI_signaling_mode			(default)

		//# Output data type : Raw10
		{0x0112, 0x0a}, //# CSI_data_format MSB 		(default)
		{0x0113, 0x0a}, //# CSI_data_format LSB 		(default)

		{0x0136, 0x18}, //# extclk_frequency_mhz MSB		(default)
		{0x0137, 0x00}, //# extclk_frequency_mhz LSB		(default)

		//# Fine integration time : 467d
		{0x0200, 0x01}, //# fine_integration_time MSB		
		{0x0201, 0xD3}, //# fine_integration_time LSB

		//# Coarse integration time : 522d
		{0x0202, 0x02}, //# coarse_integration_time MSB 	(default)
		{0x0203, 0x0a}, //# coarse_integration_time LSB 	(default)

		//# Analog gain x1
		{0x0204, 0x00}, //# analogue_gain_code_global MSB		(default)
		{0x0205, 0x20}, //# analogue_gain_code_global LSB		(default)

		//# Frame length lines : 828d
		{0x0340, 0x05}, //# frame_length_lines MSB
		{0x0341, 0xA4}, //# frame_length_lines LSB

		//# Line length pck : 1602d
		{0x0342, 0x06}, //# line_length_pck MSB 		(default)
		{0x0343, 0x42}, //# line_length_pck LSB 		(default)

		//# Size setting : 1412x796
		{0x0344, 0x00}, //# x_addr_start MSB			(default)
		{0x0345, 0x00}, //# x_addr_start LSB			(default)
		{0x0348, 0x05}, //# x_addr_end MSB			(default)
		{0x0349, 0x83}, //# x_addr_end LSB			(default)
		{0x034c, 0x05}, //# x_output_size MSB			(default)
		{0x034d, 0x84}, //# x_output_size LSB			(default)

		{0x0346, 0x00}, //# y_addr_start MSB
		{0x0347, 0x00}, //# y_addr_start LSB
		{0x034a, 0x05}, //# y_addr_end MSB
		{0x034b, 0x83}, //# y_addr_end LSB
		{0x034e, 0x05}, //# y_output_size MSB
		{0x034f, 0x84}, //# y_output_size LSB

		//# Sub-sampling
		{0x0381, 0x01}, //# x_even_inc				(default)
		{0x0383, 0x01}, //# x_odd_inc				(default)
		{0x0385, 0x01}, //# y_even_inc				(default)
		{0x0387, 0x01}, //# y_odd_inc				(default)

		//# Digital crop setting
		{0x0408, 0x00}, //# digital_crop_x_offset MSB		(default)
		{0x0409, 0x00}, //# digital_crop_x_offset LSB		(default)
		{0x040a, 0x00}, //# digital_crop_y_offset MSB		(default)
		{0x040b, 0x00}, //# digital_crop_y_offset LSB		(default)
		{0x040c, 0x05}, //# digital_crop_image_width MSB		(default)
		{0x040d, 0x84}, //# digital_crop_image_width LSB		(default)
		{0x040e, 0x05}, //# digital_crop_image_height MSB
		{0x040f, 0x84}, //# digital_crop_image_height LSB

		//# Analog Binning
		{0x0900, 0x00}, //# binning_mode				(default)

		//# Streaming on
		//{0x0100, 0x01}, //# mode_select
#endif
/* OPPO 2012-10-25 yxq modified end */
};

static struct msm_camera_i2c_reg_conf s5k6a3yx_recommend_settings[] = {
        //####################################################
        //#  name:  S5K6A3YX EVT2 setfile
        //#  v0.54 Temporary version
        //#  MSR setting could be changed
        //#  Ext CLK : 9.6Mhz , 60fps, size 1412x796
        //####################################################

        //# Streaming off
        //{0x0100, 0x00}, //# mode_select				(default)

        //$MIPI[width:1412, height:796, lane:1, format:raw10, datarate:796]

        //analog setting(Line length pck : 1602d)
        {0x3061, 0x55},   
        {0x3062, 0x54},   
        {0x5703, 0x07},   
        {0x5704, 0x07},   
        {0x305e, 0x0D},   
        {0x305f, 0x2E},   
        {0x3052, 0x01},   
        {0x300B, 0x28},   
        {0x300C, 0x2E},   
        {0x3004, 0x0a},   
        {0x5700, 0x08},   
        {0x3005, 0x3D},   
        {0x3008, 0x1E},   
        {0x3025, 0x40},   
        {0x3023, 0x20},   
        {0x3029, 0xFF}, 
        {0x302A, 0xFF}, 
        {0x3505, 0x41}, 
        {0x3095, 0x15}, 
        {0x3506, 0x00}, 
        {0x3521, 0x01}, 
        {0x3522, 0x01}, 
        {0x3D20, 0x63},
        {0x3110, 0x01},	
        {0x3111, 0x62},
        {0x3112, 0x0E},
        {0x3113, 0xBC},
        {0x311D, 0x30},
        {0x311F, 0x40},
        {0x0138, 0x00},
        {0x3009, 0x1E}, 

        //# PLL setting for 796MHz, Frame rate 60fps
        {0x0305, 0x06}, //# pre_pll_clk_div			(default)
        {0x0306, 0x00}, //# pll_multiplier MSB			(default)
        {0x0307, 0xC7}, //# pll_multiplier LSB

        {0x0301, 0x0a}, //# vt_pix_clk_div			(default)
        {0x0303, 0x01}, //# vt_sys_clk_div			(default)
        {0x0309, 0x0a}, //# op_pix_clk_div			(default)
        {0x030b, 0x01}, //# op_sys_clk_div			(default)

        {0x0820, 0x03}, //# requested_link_bit_rate_mbps MSB MSB
        {0x0821, 0x1c}, //# requested_link_bit_rate_mbps MSB LSB
        {0x0822, 0x00}, //# requested_link_bit_rate_mbps LSB MSB
        {0x0823, 0x00}, //# requested_link_bit_rate_mbps LSB LSB

        {0x0101, 0x00}, //# image_orientation			(default)

        //# Select output mode  0:CCP2(Data/Clock), 1:CCP2(Data/Strobe), 2:CSI2
        {0x0111, 0x02}, //# CSI_signaling_mode			(default)

        //# Output data type : Raw10
        {0x0112, 0x0a}, //# CSI_data_format MSB			(default)
        {0x0113, 0x0a}, //# CSI_data_format LSB			(default)

        {0x0136, 0x18}, //# extclk_frequency_mhz MSB		(default)
        {0x0137, 0x00}, //# extclk_frequency_mhz LSB		(default)

        //# Fine integration time : 467d
        {0x0200, 0x01}, //# fine_integration_time MSB		
        {0x0201, 0xD3}, //# fine_integration_time LSB

        //# Coarse integration time : 522d
        {0x0202, 0x02}, //# coarse_integration_time MSB		(default)
        {0x0203, 0x0a}, //# coarse_integration_time LSB		(default)

        //# Analog gain x1
        {0x0204, 0x00}, //# analogue_gain_code_global MSB		(default)
        {0x0205, 0x20}, //# analogue_gain_code_global LSB		(default)

        //# Frame length lines : 828d
        {0x0340, 0x03}, //# frame_length_lines MSB
        {0x0341, 0x3c}, //# frame_length_lines LSB

        //# Line length pck : 1602d
        {0x0342, 0x06}, //# line_length_pck MSB			(default)
        {0x0343, 0x42}, //# line_length_pck LSB			(default)

        //# Size setting : 1412x796
        {0x0344, 0x00}, //# x_addr_start MSB			(default)
        {0x0345, 0x00}, //# x_addr_start LSB			(default)
        {0x0348, 0x05}, //# x_addr_end MSB			(default)
        {0x0349, 0x83}, //# x_addr_end LSB			(default)
        {0x034c, 0x05}, //# x_output_size MSB			(default)
        {0x034d, 0x84}, //# x_output_size LSB			(default)

        {0x0346, 0x01}, //# y_addr_start MSB
        {0x0347, 0x34}, //# y_addr_start LSB
        {0x034a, 0x04}, //# y_addr_end MSB
        {0x034b, 0x4f}, //# y_addr_end LSB
        {0x034e, 0x03}, //# y_output_size MSB
        {0x034f, 0x1c}, //# y_output_size LSB

        //# Sub-sampling
        {0x0381, 0x01}, //# x_even_inc				(default)
        {0x0383, 0x01}, //# x_odd_inc				(default)
        {0x0385, 0x01}, //# y_even_inc				(default)
        {0x0387, 0x01}, //# y_odd_inc				(default)

        //# Digital crop setting
        {0x0408, 0x00}, //# digital_crop_x_offset MSB		(default)
        {0x0409, 0x00}, //# digital_crop_x_offset LSB		(default)
        {0x040a, 0x00}, //# digital_crop_y_offset MSB		(default)
        {0x040b, 0x00}, //# digital_crop_y_offset LSB		(default)
        {0x040c, 0x05}, //# digital_crop_image_width MSB		(default)
        {0x040d, 0x84}, //# digital_crop_image_width LSB		(default)
        {0x040e, 0x03}, //# digital_crop_image_height MSB
        {0x040f, 0x1c}, //# digital_crop_image_height LSB

        //# Analog Binning
        {0x0900, 0x00}, //# binning_mode				(default)

        //# Streaming on
        //{0x0100, 0x01}, //# mode_select
};

static struct v4l2_subdev_info s5k6a3yx_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array s5k6a3yx_init_conf[] = {
	{&s5k6a3yx_recommend_settings[0],
	ARRAY_SIZE(s5k6a3yx_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array s5k6a3yx_confs[] = {
	{&s5k6a3yx_snap_settings[0],
	ARRAY_SIZE(s5k6a3yx_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&s5k6a3yx_prev_settings[0],
	ARRAY_SIZE(s5k6a3yx_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_sensor_output_info_t s5k6a3yx_dimensions[] = {
	/* snap */
	{
/* OPPO 2012-10-25 yxq modified begin for full size */
#if 0
		.x_output = 0x0584,  // 1412
		.y_output = 0x0424,  // 1060
		.line_length_pclk = 0x0642,  // 1602
		.frame_length_lines = 0x0444,  // 1092
		.vt_pixel_clk = 105000000,
		.op_pixel_clk = 105000000,
		.binning_factor = 1,
#else
		.x_output = 0x0584,  // 1412
		.y_output = 0x0584,  // 1412
		.line_length_pclk = 0x0642,  // 1602
		.frame_length_lines = 0x05A4,  // 1444
		.vt_pixel_clk = 79600000,
		.op_pixel_clk = 79600000,
		.binning_factor = 1,
#endif
/* OPPO 2012-10-25 yxq modified end */
	},
	/* preview */
	{
/* OPPO 2012-11-22 yxq Modify begin for reason */
#if 0
		.x_output = 0x0584,  // 1412
		.y_output = 0x031C,  // 796
		.line_length_pclk = 0x0642,  // 1602
		.frame_length_lines = 0x033C,  // 828
		.vt_pixel_clk = 79600000,
		.op_pixel_clk = 79600000,
		.binning_factor = 1,
#else
        .x_output = 0x0584,  // 1412
        .y_output = 0x0584,  // 1412
        .line_length_pclk = 0x0642,  // 1602
        .frame_length_lines = 0x05A4,  // 1444
        .vt_pixel_clk = 79600000,
        .op_pixel_clk = 79600000,
        .binning_factor = 1,
#endif
/* OPPO 2012-11-22 yxq Modify end */
	},
};

/* OPPO 2013-02-04 kangjian delete begin for s5k6a3yx */
#if 0
static struct msm_camera_csid_vc_cfg s5k6a3yx_cid_cfg[] = {
	{0, CSI_RAW10, CSI_DECODE_10BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
	{2, CSI_RESERVED_DATA_0, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params s5k6a3yx_csi_params = {
	.csid_params = {
		.lane_cnt = 1,
		.lut_params = {
			.num_cid = ARRAY_SIZE(s5k6a3yx_cid_cfg),
			.vc_cfg = s5k6a3yx_cid_cfg,
		},
	},
	.csiphy_params = {
		.lane_cnt = 1,
		.settle_cnt = 0x1B,
	},
};

static struct msm_camera_csi2_params *s5k6a3yx_csi_params_array[] = {
	&s5k6a3yx_csi_params,
	&s5k6a3yx_csi_params,
	&s5k6a3yx_csi_params,
};
#endif
/* OPPO 2013-02-04 kangjian delete end for s5k6a3yx */

static struct msm_sensor_output_reg_addr_t s5k6a3yx_reg_addr = {
	.x_output = 0x034C,
	.y_output = 0x034E,
	.line_length_pclk = 0x0342,
	.frame_length_lines = 0x0340,
};

static struct msm_sensor_id_info_t s5k6a3yx_id_info = {
	.sensor_id_reg_addr = 0x0000,
	.sensor_id = 0000,
};

static struct msm_sensor_exp_gain_info_t s5k6a3yx_exp_gain_info = {
	.coarse_int_time_addr = 0x0202,
	.global_gain_addr = 0x0204,
	.vert_offset = 6,
};

static const struct i2c_device_id s5k6a3yx_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&s5k6a3yx_s_ctrl},
	{ }
};

static struct i2c_driver s5k6a3yx_i2c_driver = {
	.id_table = s5k6a3yx_i2c_id,
	.probe  = msm_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client s5k6a3yx_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};


static int __init s5k6a3yx_sensor_init_module(void)
{
	return i2c_add_driver(&s5k6a3yx_i2c_driver);
}

static struct v4l2_subdev_core_ops s5k6a3yx_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops s5k6a3yx_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops s5k6a3yx_subdev_ops = {
	.core = &s5k6a3yx_subdev_core_ops,
	.video  = &s5k6a3yx_subdev_video_ops,
};

static struct msm_sensor_fn_t s5k6a3yx_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_setting = msm_sensor_setting,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
#if 0
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
#else
	.sensor_power_up = s5k6a3yx_sensor_power_up,
	.sensor_power_down = s5k6a3yx_sensor_power_down,
#endif
	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines1,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
};

static struct msm_sensor_reg_t s5k6a3yx_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = s5k6a3yx_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(s5k6a3yx_start_settings),
	.stop_stream_conf = s5k6a3yx_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(s5k6a3yx_stop_settings),
	.group_hold_on_conf = s5k6a3yx_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(s5k6a3yx_groupon_settings),
	.group_hold_off_conf = s5k6a3yx_groupoff_settings,
	.group_hold_off_conf_size = ARRAY_SIZE(s5k6a3yx_groupoff_settings),
	.init_settings = &s5k6a3yx_init_conf[0],
	.init_size = ARRAY_SIZE(s5k6a3yx_init_conf),
	.mode_settings = &s5k6a3yx_confs[0],
	.output_settings = &s5k6a3yx_dimensions[0],
	.num_conf = ARRAY_SIZE(s5k6a3yx_confs),
};

static struct msm_sensor_ctrl_t s5k6a3yx_s_ctrl = {
	.msm_sensor_reg = &s5k6a3yx_regs,
	.sensor_i2c_client = &s5k6a3yx_sensor_i2c_client,
	.sensor_i2c_addr = 0x20,
	.sensor_output_reg_addr = &s5k6a3yx_reg_addr,
	.sensor_id_info = &s5k6a3yx_id_info,
	.sensor_exp_gain_info = &s5k6a3yx_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	//.csi_params = &s5k6a3yx_csi_params_array[0],
	.msm_sensor_mutex = &s5k6a3yx_mut,
	.sensor_i2c_driver = &s5k6a3yx_i2c_driver,
	.sensor_v4l2_subdev_info = s5k6a3yx_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(s5k6a3yx_subdev_info),
	.sensor_v4l2_subdev_ops = &s5k6a3yx_subdev_ops,
	.func_tbl = &s5k6a3yx_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(s5k6a3yx_sensor_init_module);
MODULE_DESCRIPTION("SONY 13MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
