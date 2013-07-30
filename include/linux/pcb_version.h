/************************************************************ 
** Copyright (C), 2010-2013, OPPO Mobile Comm Corp., Ltd
** All rights reserved. 
************************************************************/
#ifndef _PCB_VERSION_H
#define _PCB_VERSION_H

enum {
	PCB_VERSION_EVB,
	PCB_VERSION_EVT,
	PCB_VERSION_DVT,
	PCB_VERSION_PVT,
	PCB_VERSION_EVB_TD,
	PCB_VERSION_EVT_TD,
	PCB_VERSION_DVT_TD,
	PCB_VERSION_PVT_TD,
	PCB_VERSION_PVT2_TD,
	PCB_VERSION_PVT3_TD,

	PCB_VERSION_UNKNOWN,
};

extern int get_pcb_version(void);

#endif /* _PCB_VERSION_H */


