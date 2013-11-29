#include "msm_sensor.h"
#include "oppo_feature.h"

enum{
    THR_CASE = 0,
    STABLE_CASE
};
#define m9mo_caf_debug(fmt, arg...) //printk(fmt, ##arg);

static u_int32_t pre_wd;
static u_int32_t pre_wb;
static u_int32_t pre_bright;

static struct oppo_interface* caf_i;

static u_int8_t getOrder(u_int32_t wd_value)
{
    u_int8_t order = 0;
	
    while(wd_value != 0){
       wd_value = wd_value/10;
       order++;
    }
    return order; 
}
static bool m9mo_check_if_do_caf(struct frame_info_t *frame_info, u_int32_t mode)
{
	bool result = false;

	u_int32_t wd_value = 0;
	int32_t wd_diff  = 0;  
	int32_t thread_high = 0;
	int32_t thread_low = 0;
	int32_t bright = 0;
	int32_t wb = 0;
	int32_t wb_diff = 0;
	int32_t bright_diff = 0;

	static bool moving = false;
	
	wd_value = frame_info->wave_detect;
	wb = frame_info->wb;
	bright = frame_info->brightness;

	if(getOrder(wd_value) < 4 || getOrder(wd_value) > 7)
	{
		m9mo_caf_debug("wd_value[%u] is not valid, not use it \r\n", wd_value);
		return false;
	}

	wd_diff = abs(pre_wd - wd_value);
	wb_diff = abs(wb - pre_wb);
	bright_diff = abs(bright - pre_bright);

	m9mo_caf_debug("bright_diff---[%d], bright----[%d] \r\n", bright_diff, bright);
	m9mo_caf_debug("wb_diff---[%d], wb----[%d] \r\n", wb_diff, wb);
		
	switch(mode)
	{
		case THR_CASE:
		{
			m9mo_caf_debug("THR_CASE-----start \r\n");
			if (getOrder(pre_wd) == 5)
			{
				thread_high = pre_wd/50;
			}
			else if (getOrder(pre_wd) == 6)
			{
				thread_high = pre_wd/300;
			}

			m9mo_caf_debug("wd_diff = %d , thread_high = %d \r\n", wd_diff, thread_high);
			
			if(wd_diff > thread_high)
			{
				m9mo_caf_debug("THR_CASE  true \n");
				result = true;
			}
			else 
			{
				result = false;
			}
			
			//avoid focus continous at some scenes
			if (result == true)
			{
				m9mo_caf_debug("check bright/wb/gyro for avoiding shaking foucs \r\n");
				if ((bright_diff <5 && bright > 0)
					&& (wb_diff < 5 && wb > 0)
					&& frame_info->caf_result != FORCE_CAF_STOP)
				{
					m9mo_caf_debug("brightness or wb is stable, not trigger high tread \r\n");
					result = false;
				}
				
				if (frame_info->caf_result == FORCE_CAF_START)
					result = false;
			}
			else
			{
				if ((bright_diff*100> pre_bright*10 && pre_bright) ||
					(wb_diff*100> pre_wb*30 && pre_wb && wb_diff>50))
				{
					m9mo_caf_debug("bright or wb vares, trigger high_thread \r\n");
					result = true;
				}
			}
			
			if (frame_info->caf_result == FORCE_CAF_STOP)
			{
				m9mo_caf_debug("Now start moving \n");
				result = true;
				moving = true;
				frame_info->start_moving = false;
			}
			break;
        }
        case STABLE_CASE:
		{
			m9mo_caf_debug("STABLE_CASE-----start \r\n");
			
			/* detect by wd start */
			if (getOrder(pre_wd) == 5)
			{
				thread_low = pre_wd/50;
			}
			else if (getOrder(pre_wd) == 6)
			{
				thread_low = pre_wd/200;
			}

			if(WDV_CAF_LOW_THR <= pre_wd/200) 
			{
				thread_low = pre_wd/300;
			}
			else
			{
				thread_low = WDV_CAF_LOW_THR;
			}
			
			m9mo_caf_debug("wd_diff = %d , thread_low = %d, pre_wd = %d \r\n", wd_diff, thread_low, pre_wd);
			if(wd_diff > thread_low)
			{
				result = false;
			} 
			else 
			{
				m9mo_caf_debug("STABLE_CASE true ^^^^^^^^^^^^^^^\n");
				result = true;
			}

			m9mo_caf_debug("[stable] bright_diff---[%d], bright----[%d] \r\n", bright_diff, bright);
			m9mo_caf_debug("[stable] wb_diff---[%d], wb----[%d] \r\n", wb_diff, wb);
			//avoid focus when moving sometimes
			if (result == false)
			{
				m9mo_caf_debug("check the brightness and wb-----stable\r\n");
				
				if ((bright_diff < 10 && bright > 0) && 
					(wb_diff < 30 && wb > 0))
				{
					m9mo_caf_debug("the brightness and WB is stable, do focus \r\n");
					result = true;
				}
			}
			
			if (frame_info->caf_result == FORCE_CAF_START)
			{
				if (moving)
				{
					m9mo_caf_debug("moving trigger----phone stop moving, can do focus now \r\n");
					result = true;
					moving = false;
				}
				else
				{
					result = false;
				}
			}
			else
			{
				result = false;
			}
			break;
		}
        default:
            break;
    }
 
	pre_wd = wd_value;
	pre_wb = wb;
	pre_bright = bright;
	
    return result;
}

static bool m9mo_detect_caf_first_focus(struct frame_info_t *frame_info)
{
	bool trigger = false;
	
	u_int32_t bright = 0;
	u_int32_t wb = 0;

	static u_int32_t bright_pre = 0;
	static u_int32_t wb_pre = 0;

	bright = abs(frame_info->brightness - bright_pre);
	wb = abs(frame_info->wb - wb_pre);

	if ((bright*100 > bright_pre*10 && bright_pre != 0) ||
			(wb*100 > wb_pre*5 && wb_pre != 0))
	{
		trigger = true;
		bright_pre = 0;
		wb_pre = 0;
	}
	else
	{
		trigger = false;
	}

	bright_pre = frame_info->brightness;
	wb_pre = frame_info->wb;
	
	return trigger;
}

static void m9mo_do_caf(struct msm_sensor_ctrl_t *s_ctrl,
	struct frame_info_t *frame_info)
{
	static bool caf_flag = false;
	static bool caf_start = false;

	static int32_t shake_delay_frames = 0;
	static int32_t af_delay_frames = 0;
	static int32_t start_delay_frames = 0;
	static int32_t wd_delay_cnt = 0;
	static int32_t cap_detect_time = 0;
	static int32_t single_af_delay = 0;

	bool detect_after_delay = false;

	m9mo_caf_debug("[%s] Enter\r\n", __func__);

	do
	{
		//when start monitor, must do one time focus,the WD value can valid
		if (!frame_info->wd_valid)
		{
			m9mo_caf_debug("[%s] after camera start, must do one focus for wd valid\r\n", __func__);
			m9mo_caf_debug("[%s] start_delay_frames[%d]\r\n", __func__, start_delay_frames);
			if (start_delay_frames++>CAM_START_DELAY_FRAME && 
				frame_info->focus_done)
			{
				caf_i->notify(s_ctrl, 0);
				frame_info->wd_valid = true;
				start_delay_frames = 0;
				wd_delay_cnt = WD_VALID_DELAY_FRAME;
			}
			break;
		}
		
		//after first focus, delay some frames
		if (frame_info->wd_valid && wd_delay_cnt-->0)
		{
			m9mo_caf_debug("[%s] after wd valid, delay frames\r\n", __func__);
			caf_start = false;
			caf_flag = false;
			break;
		}
		
		//lock when do single focus
		if (frame_info->single_af && frame_info->wd_valid)
		{
			m9mo_caf_debug("[%s]: Not do caf when single focus \r\n", __func__);
			single_af_delay = SINGLE_FOCUS_DETECT_FRAME;
			caf_start = false;
			caf_flag = false;
			break;
		}
		//after single af, need delay some frames
		if (single_af_delay>0 && !frame_info->single_af)
		{
			m9mo_caf_debug("After single af, delay some frames\r\n");
			single_af_delay--;
			if (single_af_delay == 0)
				detect_after_delay = true;
			break;
		}

		if (detect_after_delay)
		{
			if (m9mo_detect_caf_first_focus(frame_info) && 
				frame_info->focus_done)
			{
				caf_i->notify(s_ctrl, 0);
				detect_after_delay = false;
				caf_start = false;
				caf_flag = false;
			}
			break;
		}
		
		//after capture need delay some frames lock
		if (frame_info->capture_start)
		{
			m9mo_caf_debug("After capture, detect caf \r\n");
			if (m9mo_detect_caf_first_focus(frame_info) && 
				frame_info->focus_done)
			{
				caf_i->notify(s_ctrl, 0);
				frame_info->capture_start = false;
				caf_start = false;
				caf_flag = false;
			}
			
			if (cap_detect_time++ > CAP_DELAY_FRAME)
			{
				cap_detect_time = 0;
				frame_info->capture_start = false;
				caf_start = false;
				caf_flag = false;
			}
			break;
		}
		
		if(!caf_start)
		{
			caf_flag = m9mo_check_if_do_caf(frame_info, THR_CASE);
			if(caf_flag)
			{
				af_delay_frames = DO_AF_DELAY;
				caf_start = true;
			}
		}
		
		if(caf_flag)
		{
			m9mo_caf_debug("af_delay_frames = %d \r\n", af_delay_frames);
			if (frame_info->start_moving)
			{
				caf_flag = false;
				shake_delay_frames = 0;
			}
			if((af_delay_frames < 1) && 
				m9mo_check_if_do_caf(frame_info, STABLE_CASE) &&
				frame_info->focus_done) 
			{
				caf_i->notify(s_ctrl, 0);
				caf_flag = false;
				shake_delay_frames = ANTI_WDV_SHAKE_DELAY;
			}	
			af_delay_frames--;
		}
		
		if(shake_delay_frames > 0)
		{
			shake_delay_frames--;
		} 
		else if(shake_delay_frames == 0)
		{
			caf_start = false;
			shake_delay_frames--;
		}
	}while (0);

	pre_wd = frame_info->wave_detect;
	pre_bright = frame_info->brightness;
	pre_wb = frame_info->wb;
	
	m9mo_caf_debug("[%s] Exit \r\n", __func__);
	
}
int32_t oppo_caf_init(struct oppo_interface *interface)
{
    caf_i =  interface;
    //reg process
	if(caf_i)
	  caf_i->process = m9mo_do_caf;

    return 0; 
}