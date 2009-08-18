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

#ifndef __JZ_CIM_H__
#define __JZ_CIM_H__

/* use ext clock as mclk */
#define CIM_EXTCLK	24000000


/* If use default 16M mem, enable it*/
//#define USE_DEFAULT_MEM

/* Camera Preview buffer number */
#define CIM_BUF_NUM	3
#define INVALID_PIC_BUF	0

//#define IMEM_MAX_ORDER 12		/* max 2^12 * 4096 = 16MB */

/*
 * Define the Max Image Size CIM Support
 */
#define MAX_IMAGE_WIDTH		4096
#define MAX_IMAGE_HEIGHT	4096
#define MAX_PRE_WIDTH		640
#define MAX_PRE_HEIGHT		480
#define MAX_IMAGE_BPP		16
#define MAX_PRE_SIZE		(MAX_PRE_WIDTH * MAX_PRE_HEIGHT * MAX_IMAGE_BPP >> 3) 
#define MAX_FRAME_SIZE		(MAX_IMAGE_WIDTH * MAX_IMAGE_HEIGHT * MAX_IMAGE_BPP >> 3)

/*
 * IOCTL_XXX commands
 */
#define IOCTL_SET_I2C_ADDR	0x0
#define IOCTL_SET_I2C_CLK	0x1
#define IOCTL_WRITE_I2C_REG	0x2
#define IOCTL_READ_I2C_REG	0x3
#define IOCTL_WRITE_I2C_REG16	0x4
#define IOCTL_READ_I2C_REG16	0x5
#define IOCTL_SET_MEM		0x6
#define IOCTL_START_CIM		0x7       // arg type: void
#define IOCTL_STOP_CIM		0x8       // arg type: void
#define IOCTL_GET_PREVIEW_PARAM	0x9	// arg type: preview param *
#define IOCTL_SET_PREVIEW_PARAM	0xA	// arg type: preview param *
#define IOCTL_GET_PICTURE_PARAM	0xB	// arg type: img_param_t *
#define IOCTL_SET_PICTURE_PARAM	0xC	// arg type: img_param_t *
#define IOCTL_GET_CIM_CONFIG	0xD	// arg type: cim_config_t *
#define IOCTL_SET_CIM_CONFIG	0xE	// arg type: cim_config_t *
#define IOCTL_PRINT_REGS	0xF	// NULL
#define IOCTL_TAKE_PICTURE	0x10
#define IOCTL_GET_CURRENT_BUF_ID 0x11

/* gpio init */
#if defined(CONFIG_JZ4750_APUS) || defined(CONFIG_JZ4750D_FUWA1) /* board pavo */
#define GPIO_CAMERA_RST 	(32*4+8) /* CIM_MCLK as reset */
#else
#error "driver/misc/jz_cim.h, please define camera for your board."
#endif


/* preview-format=rgb565|yuv422 */
typedef struct{
	unsigned int width;
	unsigned int height;
	unsigned int bpp;
	char format[20];
	unsigned int framebuf[CIM_BUF_NUM];
} preview_param_t;

/* picture-format=yuv422|jpeg */
typedef struct{
	unsigned int width;
	unsigned int height;
	unsigned int bpp;
//	const char *format;
	char format[20];
	unsigned int framebuf[CIM_BUF_NUM];
} picture_param_t;

typedef struct{
	unsigned int cfg;
	unsigned int ctrl;
	unsigned int mclk;
	unsigned int size;
	unsigned int offs;
} cim_config_t;


struct jz_camera_device_platform_data {
	int gpio_reset;
	void (*config_gpio_on) (void);
	void (*config_gpio_off)(void);
};


#endif /*__JZ_CIM_H__*/
