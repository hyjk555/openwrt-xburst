/*
 * linux/drivers/misc/jz_cim.h -- Ingenic Jz4750 On-Chip CIM driver
 *
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __JZ_SENSOR_H__
#define __JZ_SENSOR_H__

#include "jz_cim.h"


#define IN_YUV422	1		/*Sensor output YUV422*/

/*
 * Define the Max Image Size Sensor Support, Should less than CIM MAX
 */
#if defined(CONFIG_OV3640)
#define MAX_SENSOR_WIDTH	2048
#define MAX_SENSOR_HEIGHT	1536
#define MAX_SENSOR_BPP		16
#define SENSOR_PRE_WIDTH	320
#define SENSOR_PRE_HEIGHT	240
#elif defined(CONFIG_OV2640)
#define MAX_SENSOR_WIDTH	1600
#define MAX_SENSOR_HEIGHT	1200
#define MAX_SENSOR_BPP		16
#define SENSOR_PRE_WIDTH	320
#define SENSOR_PRE_HEIGHT	240
#elif defined(CONFIG_OV9650)
#define MAX_SENSOR_WIDTH	1280
#define MAX_SENSOR_HEIGHT	1024
#define MAX_SENSOR_BPP		16
#define SENSOR_PRE_WIDTH	320
#define SENSOR_PRE_HEIGHT	240
#else 
#define MAX_SENSOR_WIDTH	1280
#define MAX_SENSOR_HEIGHT	1024
#define MAX_SENSOR_BPP		16
#define SENSOR_PRE_WIDTH	320
#define SENSOR_PRE_HEIGHT	240
#endif

#if defined(CONFIG_OV3640) || defined(CONFIG_OV2640)
//#define JPEG_OUTPUT_SUPPORT	
#endif

#if defined(JPEG_OUTPUT_SUPPORT)
#define MAX_PICTURE_SIZE	(6*1024*1024) /* for 2048*1536*2 */
#else
#define MAX_PICTURE_SIZE	(MAX_SENSOR_WIDTH * MAX_SENSOR_HEIGHT * MAX_IMAGE_BPP >> 3)
#endif


#if defined(CONFIG_OV9650) || defined(CONFIG_OV2640)
#define __sensor_gpio_init()	\
do {\
	__gpio_as_output(GPIO_CAMERA_RST);	\
	__gpio_set_pin(GPIO_CAMERA_RST); \
	mdelay(50); \
	__gpio_clear_pin(GPIO_CAMERA_RST);\
} while(0)
#elif defined(CONFIG_OV3640)
#define __sensor_gpio_init()	\
do {\
	__gpio_as_output(GPIO_CAMERA_RST);	\
	__gpio_clear_pin(GPIO_CAMERA_RST);\
	mdelay(50); \
	__gpio_set_pin(GPIO_CAMERA_RST); \
} while(0)

#endif


extern unsigned int i2c_addr;
extern unsigned int i2c_clk;

/* I2C APP */

extern int i2c_write_16(unsigned char device, unsigned char *buf, unsigned short address, int count);
extern int i2c_read_16(unsigned char device, unsigned char *buf, unsigned short address, int count);


extern void sensor_write_reg(unsigned char reg, unsigned char val);
extern int sensor_write_reg16(unsigned short reg, unsigned char val);
extern unsigned char sensor_read_reg(unsigned char reg);
extern unsigned char sensor_read_reg16(unsigned short reg);

#endif
