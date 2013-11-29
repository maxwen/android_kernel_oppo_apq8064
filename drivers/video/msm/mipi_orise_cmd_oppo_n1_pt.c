/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_orise.h"
#include <linux/pcb_version.h>
#include <mach/device_info.h>


extern int get_pcb_version(void);

static struct msm_panel_info pinfo;
static char *DEVICE_VERSION = "64317";
static char *DEVICE_MANUFACUTRE	= "jdi";


static struct mipi_dsi_phy_ctrl dsi_cmd_mode_phy_db_1080p =
{

    /* 1920*1200, RGB888, 4 Lane 60 fps cmd mode */
    /* regulator */
    {0x09, 0x08, 0x05, 0x00, 0x20},
    /* timing */
    {
        0xe9, 0x5e, 0x2c, 0x00, 0x7b, 0x7c, 0x30, 0x61,
        0x42, 0x03, 0x04, 0xa0
    },
    /* phy ctrl */
    {0x5f, 0x00, 0x00, 0x10},
    /* strength */
    {0xff, 0x00, 0x06, 0x00},
    /* pll control */
    {
        0x0, 0x49, 0x30, 0xc4, 0x00, 0x20, 0x07, 0x62,
        0x41, 0x0f, 0x01,
        0x00, 0x14, 0x03, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01
    },
};

static int __init mipi_cmd_orise_oppo_n1_pt_init(void)
{
    int ret;

/* OPPO 2013-11-13 gousj Add begin for device information */
#ifdef CONFIG_VENDOR_EDIT
	register_device_proc("lcd", DEVICE_VERSION, DEVICE_MANUFACUTRE);
#endif
/* OPPO 2013-11-13 gousj Add end */
	
    if (msm_fb_detect_client("mipi_cmd_orise_oppo_n1"))
        return 0;
    pr_info("%s:\n", __func__);
    pinfo.xres = 1080;
    pinfo.yres = 1920;
    pinfo.type = MIPI_CMD_PANEL;
    pinfo.pdest = DISPLAY_1;
    pinfo.wait_cycle = 0;
    pinfo.bpp = 24;
    pinfo.mipi.data_lane2 = TRUE;

    pinfo.lcdc.h_back_porch = 120;
    //pinfo.lcdc.h_front_porch = 100;
	//pinfo.lcdc.h_back_porch = 101;
    pinfo.lcdc.h_front_porch = 100;
    pinfo.lcdc.h_pulse_width = 8;
    pinfo.lcdc.v_back_porch = 11;
    pinfo.lcdc.v_front_porch = 10;
    pinfo.lcdc.v_pulse_width = 5;
    pinfo.lcdc.border_clr = 0;  //blk

    pinfo.lcdc.underflow_clr = 0xff;    /* blue */
    pinfo.lcdc.hsync_skew = 0;
    pinfo.bl_max = 255;
    pinfo.bl_min = 0;
    pinfo.fb_num = 2;
    //pinfo.clk_rate = 550000000;
    //pinfo.clk_rate = 499000000;
    pinfo.clk_rate = 900000000;
    pinfo.mipi.esc_byte_ratio = 4;
    pinfo.lcd.vsync_enable = TRUE;
    pinfo.lcd.hw_vsync_mode = TRUE;
    pinfo.lcd.refx100 = 6300; /* adjust refx100 to prevent tearing */
	//pinfo.lcd.refx100 = 6204;
    //pinfo.lcd.refx100 = 6105;
    //pinfo.lcd.refx100 = 6205;
    pinfo.mipi.mode = DSI_CMD_MODE;
    pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;
    pinfo.mipi.vc = 0;
    pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
    pinfo.mipi.data_lane0 = TRUE;
    pinfo.mipi.data_lane1 = TRUE;
    pinfo.mipi.data_lane2 = TRUE;
    pinfo.mipi.data_lane3 = TRUE;
    //pinfo.mipi.t_clk_post = 0x04;
    //pinfo.mipi.t_clk_pre = 0x1d;
	pinfo.mipi.t_clk_post = 0x20;
	pinfo.mipi.t_clk_pre = 0x2F;
    pinfo.mipi.stream = 0; /* dma_p */
    pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
    pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	//pinfo.mipi.traffic_mode = DSI_BURST_MODE;
	//pinfo.mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
	//pinfo.mipi.dsi_pclk_rate = ;
	//pinfo.mipi.frame_rate = 60;
    pinfo.mipi.te_sel = 1; /* TE from vsync gpio */
    pinfo.mipi.interleave_max = 1;
    pinfo.mipi.insert_dcs_cmd = TRUE;
    pinfo.mipi.wr_mem_continue = 0x3c;
    pinfo.mipi.wr_mem_start = 0x2c;
    pinfo.mipi.dsi_phy_db = &dsi_cmd_mode_phy_db_1080p;
    pinfo.mipi.tx_eot_append = 0x01;
    pinfo.mipi.rx_eot_ignore = 0x0;


    ret = mipi_orise_device_register(&pinfo, MIPI_DSI_PRIM,
                                     MIPI_DSI_PANEL_720P_PT);
    if (ret)
        pr_err("%s: failed to register device!\n", __func__);

    return ret;
}


module_init(mipi_cmd_orise_oppo_n1_pt_init);
