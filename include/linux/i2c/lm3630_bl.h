/**************************************************************
* Copyright (c)  200X- 2012  Oppo Mobile communication Corp.ltd.£¬
* VENDOR_EDIT
*File       : lm3528.h
* Description: Source file for backlight IC LM3528.
*           To control backlight brightness.
* Version   : 1.0
* Date      : 2012-10-14
* Author    : zhengzekai@oppo.com
* ---------------------------------- Revision History: ----------------------------------
*  	<version>	<date>		< author >			<desc>
* 	Revision 1.0  2012-10-14   zhengzekai@oppo.com	creat
****************************************************************/

#ifndef _LM3630_H_

#define _LM3630_H_

int lm3630_bkl_control(unsigned char bkl_level);
int lm3630_bkl_readout(void);
int set_backlight_pwm(int state);


#endif /* __LINUX_LM3630_H */
