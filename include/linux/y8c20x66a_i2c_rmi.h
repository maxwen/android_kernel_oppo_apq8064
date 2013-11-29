/*
 * include/linux/synaptics_i2c_rmi.h - platform data structure for f75375s sensor
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_Y8C_I2C_RMI_H
#define _LINUX_Y8C_I2C_RMI_H


#define y8c20x66a_I2C_RMI_NAME "y8c20x66a-rmi-ts"

#define CYPRESS_CHIP
//#define MICROCHIP_CHIP

struct y8c20x66a_i2c_rmi_platform_data {
	int (*power)(int on);	/* Only valid in first array entry */
	unsigned long irqflags;
    unsigned int reset_gpio;
};

#endif /* _LINUX_Y8C_I2C_RMI_H */
