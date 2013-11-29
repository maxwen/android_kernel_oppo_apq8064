#include "msm_sensor.h"
#include "oppo_feature.h"

#include "msm.h"

#define COUNT_AVER_CNT 5
#define oppo_asd_debug(fmt, arg...) //printk(fmt, ##arg)
typedef enum {
	    SCENE_BEGIN    =   0, 
        SCENE_OFF      =   SCENE_BEGIN,   // Disable scene mode equal Auto mode
        SCENE_NORMAL,                          // Normal mode
        SCENE_ACTION,                          // Action mode
        SCENE_PORTRAIT, /*3*/                  // Portrait mode
        SCENE_LANDSCAPE,                       // Landscape
        SCENE_NIGHTSCENE,                      // Night Scene
        SCENE_NIGHTPORTRAIT,                   // Night Portrait
        SCENE_THEATRE,                         // Theatre mode
        SCENE_BEACH,                           // Beach mode
        SCENE_SNOW,                            // Snow mode
        SCENE_SUNSET,                          // Sunset mode
        SCENE_STEADYPHOTO,                     // Steady photo mode
        SCENE_FIREWORKS,                       // Fireworks mode
        SCENE_SPORTS, /*13*/                   // Sports mode
        SCENE_PARTY,                           // Party mode
        SCENE_CANDLELIGHT,                     // Candle light mode
        //  (Unsupported legacy symbol; don't use this symbol if possible)
        SCENE_ISO_ANTI_SHAKE,                  // ISO Anti Shake mode
        //  (Unsupported legacy symbol; don't use this symbol if possible)
        SCENE_BRACKET_AE,                      // Bracket AE
        SCENE_INDOOR,  /*18*/                  //new add for indoor
        SCENE_MIX_ILLUMINANT,                  //new add for mix
        SCENE_MACRO,                           //new add for macro
        SCENE_HDR,
        SCENE_NUM,
	}SCENE;

typedef struct {
    SCENE scene;
    char  scene_name[16];
} scene_mode;
scene_mode scene_mode_define[8] = {
        {SCENE_PORTRAIT, "portrait"},
        {SCENE_LANDSCAPE, "landscape"},
        {SCENE_INDOOR, "indoor"},
        {SCENE_MIX_ILLUMINANT, "mix_illuminant"},
        {SCENE_NIGHTSCENE, "nightscene"},
        {SCENE_SPORTS, "sports"},
        {SCENE_MACRO, "macro"},
        {SCENE_NORMAL, "normal/auto"}
};

static u_int32_t pre_fv;
static u_int32_t pre_br;
static u_int32_t pre_wb;

static int32_t count_flag;
static int32_t stable_cnt;
static int32_t change_cnt_wb;
static int32_t change_cnt_br;
static u_int32_t averWB_10frame;
static u_int32_t averBr_10frame;
static int32_t averWB_10frame_cnt;

static int32_t af_position;
static int32_t af_focused;
//static u_int32_t cur_time;

static int32_t count_change_cnt;
static int32_t delay_to_stable;
static int32_t delay_frame;
static int32_t sporting_cnt;
static int32_t sporting_stop_cnt;	

static int32_t scene_indoor;
static int32_t scene_landscape;
static int32_t scene_mix_illuminant;
static int32_t scene_night;
static int32_t scene_sports;
static int32_t scene_portrait;
static int32_t scene_macro;

static int32_t scene_cur;
static int32_t scene_pre;
static u_int32_t PreScene_to_Auto;

typedef struct {
    u_int32_t indoor_min_br;
    u_int32_t indoor_max_br;
    u_int32_t outdoor_min_br;
    u_int32_t outdoor_min_wb;
    u_int32_t night_max_wb;
    u_int32_t night_max_br;
    u_int32_t mix_max_wb;
    u_int32_t mix_min_wb;
} scene_detect_para;

scene_detect_para scene_para;
static struct oppo_interface* asd_i = NULL;

/*oppo 2013-09-04 guanjindian add for N1 asd */
#if 0
static void scene_detect_done(struct msm_sensor_ctrl_t *s_ctrl)
{
	oppo_asd_debug("[%s] enter\n", __func__);
	if(scene_pre != scene_cur)
	{
		int rc = 0;
		struct msm_ctrl_cmd ctrlcmd;
		ctrlcmd.type = MSM_V4L2_SCENE_CMD;
		ctrlcmd.timeout_ms = 2000;
		ctrlcmd.length = 0;
		ctrlcmd.value = NULL;
		ctrlcmd.vnode_id = 0;
		ctrlcmd.queue_idx = 1;//queue_id >= 0 ?queue_id:1;
		ctrlcmd.config_ident = scene_cur;
		rc = msm_server_send_ctrl(&ctrlcmd, MSM_CAM_RESP_V4L2);
		oppo_asd_debug("[%s] callback\n", __func__);
	}
}
#endif

#if 1
static void m9mo_confirm_scene(struct msm_sensor_ctrl_t *s_ctrl)
{
	
	oppo_asd_debug("[%s] enter\n", __func__);
	
	if(scene_indoor == 1 || scene_landscape == 1 || scene_mix_illuminant == 1 ||
        scene_night == 1 || scene_sports == 1 || scene_portrait == 1 || scene_macro == 1) {
        count_change_cnt = 0;
    }

    if (scene_portrait == 1){
		scene_cur = SCENE_PORTRAIT;
		 oppo_asd_debug("#####scene is portrait\n");
		
		goto out ;
	}
	else if(scene_macro == 1 ){
		scene_cur = SCENE_MACRO;
		oppo_asd_debug("#####scene is macro\n");
		
	 	goto out ;
	}
	else if(scene_night == 1 ){
		scene_cur = SCENE_NIGHTSCENE;
		oppo_asd_debug("#####scene is night\n");
		
	 	goto out ;
	}
	else if(scene_sports == 1 ){
		scene_cur = SCENE_SPORTS;
		oppo_asd_debug("#####scene is sports\n");
		
	 	goto out ;	
	}
	else if(scene_landscape == 1){
		scene_cur = SCENE_LANDSCAPE;
		oppo_asd_debug("#####scene is landscape\n");
		
		goto out ;
	}
	else if(scene_indoor == 1 ){
	    if(averWB_10frame > scene_para.mix_min_wb &&
	        averWB_10frame < scene_para.mix_max_wb) {
            scene_cur = SCENE_MIX_ILLUMINANT;
			oppo_asd_debug("#####scene is mix_illuminant\n");
			
	    } else {
		    scene_cur = SCENE_INDOOR;
			oppo_asd_debug("#####scene is indoor\n");
			
	    }
		goto out ;
	}
	else if(scene_mix_illuminant == 1 ){
		scene_cur = SCENE_MIX_ILLUMINANT;
		oppo_asd_debug("#####scene is mix_illuminant\n");
		
		goto out ;
	}
	else if(count_change_cnt >= 10) {
	    scene_cur = SCENE_NORMAL;
		oppo_asd_debug("#####scene is normal\n");
		
    }
	
    out:
       
        oppo_asd_debug("#####current scene is %d\n",scene_cur);
	    // scene_detect_done(s_ctrl);
		asd_i->notify(s_ctrl,scene_cur );
        
    scene_indoor = 0;
    scene_landscape = 0;
    scene_mix_illuminant = 0;
    scene_night = 0;
    scene_sports = 0;
    scene_portrait = 0;
    scene_macro = 0;

    scene_pre = scene_cur;
}
#endif
/*oppo 2013-09-04 add end*/
void m9mo_scene_detect(struct msm_sensor_ctrl_t *s_ctrl, struct frame_info_t *frame_info)
{
    int32_t cur_br = frame_info->brightness;
    //u_int32_t cur_fv = frame_info->wave_detect;
    //u_int32_t cur_fv = 0;
    u_int32_t cur_wb = frame_info->wb;
    u_int32_t cur_ae_gain = frame_info->gain;
	u_int32_t cur_exp_time =  frame_info->exp_time;
	af_position = frame_info->af_position;

	oppo_asd_debug("[%s] enter\n", __func__);
    
    if(cur_wb == 0) {
        count_flag = 0;
        stable_cnt=0;
        averWB_10frame = 0;
        averBr_10frame = 0;
        averWB_10frame_cnt = 0;
    } else {           
        if(count_flag) {
            oppo_asd_debug("#####test:detect change\n");
            if(abs(averWB_10frame - cur_wb) > 200) {
                change_cnt_wb++;
                oppo_asd_debug("#####test:scene change for wb_cnt=%d\n",change_cnt_wb);
            } else {
                change_cnt_wb = 0;
            }
            
            if(abs(averBr_10frame - cur_br) > 100) {
                change_cnt_br++;
                oppo_asd_debug("#####test:scene change for br_cnt=%d\n",change_cnt_br);
            } else {
                change_cnt_br = 0;
            }

            if(change_cnt_wb >= 6 || change_cnt_br >= 8) {
                //recalculate aver br and wb
                count_flag = 0;
                stable_cnt=0;
                count_change_cnt = 0;
                averWB_10frame = 0;
                averBr_10frame = 0;
                averWB_10frame_cnt = 0;
                
                scene_cur = scene_pre;
            }
    
        }

       
        if(!count_flag/* && stable_cnt>2*/) {
            if(++averWB_10frame_cnt <= COUNT_AVER_CNT) {
                averWB_10frame += cur_wb;
                averBr_10frame += cur_br;
            }
            else {
                averWB_10frame = averWB_10frame/COUNT_AVER_CNT;
                averBr_10frame = averBr_10frame/COUNT_AVER_CNT;
                //oppo_asd_debug("#####averWB_10frame=%d averBr_10frame=%d\n",averWB_10frame,averBr_10frame);
                oppo_asd_debug("#####averWB_10frame=%d averBr_10frame=%d\n",averWB_10frame,averBr_10frame);
                
                count_flag = 1;
            }
        }

        if(count_flag == 1) {
            oppo_asd_debug("####test:count_flag = 1\n");
            count_change_cnt++;


            if(averBr_10frame == 0 && cur_ae_gain >= 230/*200*/&&cur_exp_time>=1400/*1200*/) {
                scene_night = 1;
                oppo_asd_debug("####scene is night at br==0\n");
            } else if(averBr_10frame < scene_para.night_max_br&& cur_ae_gain >= 250/*220*/) {
                if(averWB_10frame < scene_para.night_max_wb/* && (cur_time > 18 && cur_time < 7)*/) {
                    scene_night = 1;
                    oppo_asd_debug("####scene is night when night\n");
                }
            }

            if(averBr_10frame > scene_para.indoor_min_br && averBr_10frame <= 700 && af_position != 1
                && averWB_10frame < 6400) {
                scene_indoor = 1;
                oppo_asd_debug("####scene is indoor mid br\n");
            }
		#if 0	
		if(cur_ae_gain<240&&cur_ae_gain>50&&cur_exp_time<1150&&cur_exp_time>800){
			scene_indoor = 1;
                oppo_asd_debug("####scene is indoor by gain time\n");
			}
		#endif
            if(averBr_10frame > 50 && averBr_10frame <= scene_para.indoor_max_br && af_position != 1
                /*&& averWB_10frame < 6200*/&&cur_exp_time<1150&&cur_exp_time>800) {
                
                    scene_indoor = 1;
                    oppo_asd_debug("####scene is indoor normal br\n");
           
            }

            if(averWB_10frame > 5000 && averBr_10frame > 1000/*700*/&&cur_ae_gain< 10
				&&cur_exp_time<=100/*200*/&&af_position == 3) {
                oppo_asd_debug("####scene is landscape low br\n");
                scene_landscape = 1;
            }
		/*
            if(cur_ae_gain< 10&&cur_exp_time<=400){
				scene_landscape = 1;
            	}
            	*/
        /*    
            if(averBr_10frame >= scene_para.outdoor_min_br &&
                averWB_10frame >= scene_para.outdoor_min_wb &&
                af_position == 3) {
                oppo_asd_debug("####scene is landscape\n");
                scene_landscape = 1;
            }*/
        }
    }

    if(af_position == 1) {
        scene_macro = 1;
        oppo_asd_debug("####scene is macro\n");
    }

   // pre_fv = cur_fv;
    pre_br = cur_br;
    pre_wb = cur_wb;
      m9mo_confirm_scene(s_ctrl);
  
}



int32_t oppo_scene_detect_init(struct oppo_interface *interface)
{
	
	pre_fv = 0;
    pre_br = 0;
    pre_wb = 0;

    af_position = 0;
    af_focused = 1;
    //portrait_detect_flag = 0;     

    count_change_cnt = 0;
    delay_to_stable = 0;
    delay_frame = 30;
    sporting_cnt = 0;
    sporting_stop_cnt = 0;
    PreScene_to_Auto = 0;

    count_flag = 0;
    stable_cnt = 0;
    change_cnt_wb = 0;
    change_cnt_br = 0;
    averWB_10frame = 0;
    averBr_10frame = 0;
	averWB_10frame_cnt = 0;

    scene_indoor = 0;
    scene_landscape = 0;
    scene_mix_illuminant = 0;
    scene_night = 0;
    scene_sports = 0;
    scene_portrait = 0;
    scene_macro = 0;

    scene_cur = 0;
    scene_pre = 0;

    //get system time

    scene_para.indoor_min_br = 100/*10*//*300*/;
    scene_para.indoor_max_br = 690/*400*//*1200*/;
    scene_para.outdoor_min_br = 700/*500*//*2200*/;
    scene_para.outdoor_min_wb = 5200/*4800*/;    
    scene_para.night_max_wb = 5000;
    scene_para.night_max_br = 30/*50*/;
    scene_para.mix_max_wb = 4100/*4500*/;
    scene_para.mix_min_wb = 3300/*3200*/;
	asd_i =  interface;
	//reg process
	if(asd_i)
	  asd_i->process = m9mo_scene_detect;
	  
	return 0;
}



