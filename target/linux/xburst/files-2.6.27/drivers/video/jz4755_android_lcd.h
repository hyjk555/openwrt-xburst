
/*
 * linux/drivers/video/jz4755_android_lcd.h -- Ingenic Jz4755 On-Chip LCD frame buffer device
 *
 * Copyright (C) 2005-2009, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __JZ4755_LCD_H__
#define __JZ4755_LCD_H__

/* Please define next in lcd panel header file
 * __lcd_special_pin_init()
 * __lcd_special_on()
 * __lcd_special_off()
 */

#if defined(CONFIG_JZ4755_ANDROID_LCD_AUO_A043FL01V2)
#include "jz_auo_a043fl01v2.h"
#endif

#if defined(CONFIG_JZ4755_ANDROID_LCD_TOPPOLY_TD043MGEB1)
#include "jz_toppoly_td043mgeb1.h"
#endif

//#include <asm/io.h>


#define NR_PALETTE	256
#define PALETTE_SIZE	(NR_PALETTE*2)


/* use new descriptor(8 words) */
struct jz4755_lcd_dma_desc {
	unsigned int next_desc; 	/* LCDDAx */
	unsigned int databuf;   	/* LCDSAx */
	unsigned int frame_id;  	/* LCDFIDx */ 
	unsigned int cmd; 		/* LCDCMDx */
	unsigned int offsize;       	/* Stride Offsize(in word) */
	unsigned int page_width; 	/* Stride Pagewidth(in word) */
	unsigned int cmd_num; 		/* Command Number(for SLCD) */
	unsigned int desc_size; 	/* Foreground Size */
};

struct jz4755lcd_panel_t {
	unsigned int cfg;	/* panel mode and pin usage etc. */
	unsigned int slcd_cfg;	/* Smart lcd configurations */
	unsigned int ctrl;	/* lcd controll register */
	unsigned int w;		/* Panel Width(in pixel) */
	unsigned int h;		/* Panel Height(in line) */
	unsigned int fclk;	/* frame clk */
	unsigned int hsw;	/* hsync width, in pclk */
	unsigned int vsw;	/* vsync width, in line count */
	unsigned int elw;	/* end of line, in pclk */
	unsigned int blw;	/* begin of line, in pclk */
	unsigned int efw;	/* end of frame, in line count */
	unsigned int bfw;	/* begin of frame, in line count */
};


struct jz4755lcd_fg_t {
	int bpp;	/* foreground bpp */
	int x;		/* foreground start position x */
	int y;		/* foreground start position y */
	int w;		/* foreground width */
	int h;		/* foreground height */
};

struct jz4755lcd_osd_t {
	unsigned int osd_cfg;	/* OSDEN, ALHPAEN, F0EN, F1EN, etc */
	unsigned int osd_ctrl;	/* IPUEN, OSDBPP, etc */
	unsigned int rgb_ctrl;	/* RGB Dummy, RGB sequence, RGB to YUV */
	unsigned int bgcolor;	/* background color(RGB888) */
	unsigned int colorkey0;	/* foreground0's Colorkey enable, Colorkey value */
	unsigned int colorkey1; /* foreground1's Colorkey enable, Colorkey value */
	unsigned int alpha;	/* ALPHAEN, alpha value */
	unsigned int ipu_restart; /* IPU Restart enable, ipu restart interval time */

#define FG_NOCHANGE 		0x0000
#define FG0_CHANGE_SIZE 	0x0001
#define FG0_CHANGE_POSITION 	0x0002
#define FG1_CHANGE_SIZE 	0x0010
#define FG1_CHANGE_POSITION 	0x0020
#define FG_CHANGE_ALL 		( FG0_CHANGE_SIZE | FG0_CHANGE_POSITION | \
				  FG1_CHANGE_SIZE | FG1_CHANGE_POSITION )
	int fg_change;
	struct jz4755lcd_fg_t fg0;	/* foreground 0 */
	struct jz4755lcd_fg_t fg1;	/* foreground 1 */
};

struct jz4755lcd_info {
	struct jz4755lcd_panel_t panel;
	struct jz4755lcd_osd_t osd;
};


/***********************Emily****************************/

#define JZ_ANDROID_PANELNUM 2
struct jz_android_din_t{
	unsigned int w;
	unsigned int h;
	unsigned int index;
};

struct android_display_info_t {
	unsigned int flag;
	unsigned int fg0_number; /**/
	unsigned int fg0_index;
	unsigned int fg0_alpha; /* */
	unsigned int fg0_colorkey;/**/
	unsigned int fg0_enable;/**/
	unsigned int fg0_x;        /*fg0 start position x*/
	unsigned int fg0_y;        /*fg0 start position y*/
	unsigned int fg0_w;        /*the weight of fg0*/
	unsigned int fg0_h;        /*the height of fg0*/
	unsigned int fg1_x;        /*fg1 start position x*/
	unsigned int fg1_y;        /*fg1 start position y*/
	unsigned int fg1_w;        /*the weight of fg1*/
	unsigned int fg1_h;        /*the height of fg1*/
	unsigned int fg1_enable;   /*start or stop fg1*/
	unsigned int fg1_short_cut;/*IPU direct*/
};

#define FBIO_ANDROID_CTL                        0xad10

#define ANDROID_GET_DISPLAY_NUM 		0x00000001 
#define ANDROID_GET_DISPLAY_INFO 		0x00000002 
#define ANDROID_SET_DISPLAY_INDEX		0x00000004 
#define ANDROID_SET_FG0_ALPHA   		0x00000008
#define ANDROID_SET_FG0_COLORKEY  		0x00000010
#define ANDROID_SET_FG0_ENABLE    		0x00000020
#define ANDROID_SET_FG1_POS       		0x00000040
#define ANDROID_SET_FG1_SIZE       		0x00000080
#define ANDROID_SET_FG1_ENABLE      		0x00000100
#define ANDROID_SET_FG1_IPU_DIRECT      	0x00000200
#define ANDROID_GET_PANEL_SIZE                  0x00000400

#define BARRIER __asm__ __volatile__(".set noreorder\n\t" \
				     "nop; nop; nop; nop; nop; nop;\n\t" \
				     ".set reorder\n\t")
extern void local_flush_tlb_all(void);

/******************************Emily**************************************/

/* Jz LCDFB supported I/O controls. */
#define FBIOSETBACKLIGHT	0x4688 /* set back light level */
#define FBIODISPON		0x4689 /* display on */
#define FBIODISPOFF		0x468a /* display off */
#define FBIORESET		0x468b /* lcd reset */
#define FBIOPRINT_REG		0x468c /* print lcd registers(debug) */
#define FBIOROTATE		0x46a0 /* rotated fb */
#define FBIOGETBUFADDRS		0x46a1 /* get buffers addresses */
#define FBIO_GET_MODE		0x46a2 /* get lcd info */
#define FBIO_SET_MODE		0x46a3 /* set osd mode */
#define FBIO_DEEP_SET_MODE	0x46a4 /* set panel and osd mode */
#define FBIO_MODE_SWITCH	0x46a5 /* switch mode between LCD and TVE */
#define FBIO_GET_TVE_MODE	0x46a6 /* get tve info */
#define FBIO_SET_TVE_MODE	0x46a7 /* set tve mode */
#define FBIODISON_FG		0x46a8 /* FG display on */
#define FBIODISOFF_FG		0x46a9 /* FG display on */
#define FBIO_SET_LCD_TO_TVE	0x46b0 /* set lcd to tve mode */
#define FBIO_SET_FRM_TO_LCD	0x46b1 /* set framebuffer to lcd */
#define FBIO_SET_IPU_TO_LCD	0x46b2 /* set ipu to lcd directly */
#define FBIO_CHANGE_SIZE	0x46b3 /* change FG size */
#define FBIO_CHANGE_POSITION	0x46b4 /* change FG starts position */
#define FBIO_SET_BG_COLOR	0x46b5 /* set background color */
#define FBIO_SET_IPU_RESTART_VAL	0x46b6 /* set ipu restart value */
#define FBIO_SET_IPU_RESTART_ON	0x46b7 /* set ipu restart on */
#define FBIO_SET_IPU_RESTART_OFF	0x46b8 /* set ipu restart off */
#define FBIO_ALPHA_ON		0x46b9 /* enable alpha */
#define FBIO_ALPHA_OFF		0x46c0 /* disable alpha */
#define FBIO_SET_ALPHA_VAL	0x46c1 /* set alpha value */

/*
 * Platform specific definition
 */
#if defined(CONFIG_SOC_JZ4755) || defined(CONFIG_SOC_JZ4755D)

#if defined(CONFIG_JZ4755_APUS) /* board apus */
#define __lcd_display_pin_init() \
do { \
	__gpio_as_output(GPIO_LCD_VCC_EN_N);	 \
	__lcd_special_pin_init();	   \
} while (0)
#define __lcd_display_on() \
do { \
	__gpio_clear_pin(GPIO_LCD_VCC_EN_N);	\
	__lcd_special_on();			\
} while (0)

#define __lcd_display_off() \
do { \
	__lcd_special_off();	 \
} while (0)

#elif defined(CONFIG_JZ4755D_CETUS)/* board apus */

#define __lcd_display_pin_init() \
do { \
	__gpio_as_output(GPIO_LCD_VCC_EN_N);	 \
	__lcd_special_pin_init();	   \
} while (0)
#define __lcd_display_on() \
do { \
	__gpio_set_pin(GPIO_LCD_VCC_EN_N);	\
	__lcd_special_on();			\
} while (0)

#define __lcd_display_off() \
do { \
	__lcd_special_off();	 \
} while (0)

#else /* other boards */

#define __lcd_display_pin_init() \
do { \
	__lcd_special_pin_init();	   \
} while (0)
#define __lcd_display_on() \
do { \
	__lcd_special_on();			\
} while (0)

#define __lcd_display_off() \
do { \
	__lcd_special_off();	 \
} while (0)
#endif /* APUS */
#endif /* CONFIG_SOC_JZ4755 */


/*****************************************************************************
 * LCD display pin dummy macros
 *****************************************************************************/
#ifndef __lcd_special_pin_init
#define __lcd_special_pin_init()
#endif
#ifndef __lcd_special_on
#define __lcd_special_on()
#endif
#ifndef __lcd_special_off
#define __lcd_special_off()
#endif

#ifndef __lcd_display_pin_init
#define __lcd_display_pin_init()
#endif
#ifndef __lcd_slcd_special_on
#define __lcd_slcd_special_on()
#endif
#ifndef __lcd_display_on
#define __lcd_display_on()
#endif
#ifndef __lcd_display_off
#define __lcd_display_off()
#endif
#ifndef __lcd_set_backlight_level
#define __lcd_set_backlight_level(n)
#endif

#endif /* __JZ4755_LCD_H__ */

