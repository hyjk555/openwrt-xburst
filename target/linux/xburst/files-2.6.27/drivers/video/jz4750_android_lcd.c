
/*
 * linux/drivers/video/jz4750_android_lcd.c -- Ingenic Jz4750 LCD frame buffer device
 *
 * Copyright (C) 2005-2009, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */
/*
 * --------------------------------
 * NOTE:
 * This LCD driver support TFT16 TFT32 LCD, not support STN and Special TFT LCD 
 * now.
 * 	It seems not necessory to support STN and Special TFT.
 * 	If it's necessary, update this driver in the future.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/earlysuspend.h>
#include <linux/pm.h>
#include <linux/leds.h>
#include <linux/wait.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>

#include "console/fbcon.h"
#include "jz4750_android_lcd.h"
#include "jz4750_tve.h"

#define DRIVER_NAME	"jz-lcd"

MODULE_DESCRIPTION("Jz4750 LCD Controller driver");
MODULE_AUTHOR("Wolfgang Wang <lgwang@ingenic.cn>, Lemon Liu <zyliu@ingenic.cn>, Emily Feng <chlfeng@ingenic.cn>");
MODULE_LICENSE("GPL");

#define LCD_DEBUG
//#undef LCD_DEBUG

#ifdef CONFIG_JZSOC_BOOT_LOGO
extern int load_565_image(char *filename);
#ifdef CONFIG_FB_565RLE_LOGO
#define INIT_IMAGE_FILE "/logo.rle"
#endif
#ifdef CONFIG_FB_565RGB_LOGO
#define INIT_IMAGE_FILE "/logo.rgb565"
#endif
#endif /* CONFIG_JZSOC_BOOT_LOGO */

#ifdef LCD_DEBUG
#define dprintk(x...)	printk(x)
#define print_dbg(f, arg...) printk("dbg::" __FILE__ ",LINE(%d): " f "\n", __LINE__, ## arg)
#else
#define dprintk(x...)
#define print_dbg(f, arg...) do {} while (0)
#endif

#define print_err(f, arg...) printk(KERN_ERR DRIVER_NAME ": " f "\n", ## arg)
#define print_warn(f, arg...) printk(KERN_WARNING DRIVER_NAME ": " f "\n", ## arg)
#define print_info(f, arg...) printk(KERN_INFO DRIVER_NAME ": " f "\n", ## arg)

#define JZ_LCD_ID "jz-lcd"
#define ANDROID_NUMBER_OF_BUFFERS 2

struct lcd_cfb_info {
	struct fb_info		fb0;	/* foreground 0 */
	struct fb_info		fb;	/* foreground 1 */
	struct display_switch	*dispsw;
	signed int		currcon;
	int			func_use_count;
	spinlock_t	update_lock;
	unsigned 	frame_requested;
	unsigned 	frame_done;
	wait_queue_head_t frame_wq;
	struct early_suspend earlier_suspend;
	struct early_suspend early_suspend;
	struct {
		u16 red, green, blue;
	} palette[NR_PALETTE];
};

static struct lcd_cfb_info *jz4750fb_info;
static int current_dma0_id, current_dma1_id;
static struct jz4750_lcd_dma_desc *dma_desc_base;
static struct jz4750_lcd_dma_desc *dma0_desc_palette, *dma0_desc0, *dma0_desc1, *dma1_desc0, *dma1_desc1;
static struct jz4750_lcd_dma_desc *dma0_desc0_change, *dma1_desc0_change, *dma0_desc1_change, *dma1_desc1_change;

#define DMA_DESC_NUM 		9
//#define IMEM_MAX_ORDER         12 /*16M*/  

static unsigned char *lcd_palette;
static unsigned char *lcd_frame0;
static unsigned char *lcd_frame;

/* APP */
static void jz4750fb_set_mode(struct jz4750lcd_osd_t * lcd_osd_info );
static void jz4750fb_deep_set_mode( struct jz4750lcd_info * lcd_info );
static void print_lcdc_registers(void);

static void jz4750lcd_info_switch_to_TVE(int mode);
static void jz4750lcd_info_switch_to_lcd(void);

static int jz4750fb0_foreground_resize(struct jz4750lcd_osd_t *lcd_osd_info);
static int jz4750fb0_foreground_move(struct jz4750lcd_osd_t *lcd_osd_info);

static int jz4750fb_foreground_resize(struct jz4750lcd_osd_t *lcd_osd_info);
static int jz4750fb_foreground_move(struct jz4750lcd_osd_t *lcd_osd_info);

static struct jz4750lcd_info jz4750_lcd_panel = {
#if defined(CONFIG_JZ4750_ANDROID_LCD_AUO_A043FL01V2)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */ 
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_24BIT | 	/* output 18bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		480, 272, 60, 41, 10, 8, 4, 4, 2,
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN | LCD_OSDC_F0EN| LCD_OSDC_F1EN|
		LCD_OSDC_ALPHAEN,// | /* Use OSD mode */
		.osd_ctrl = 0,		/* disable ipu,  */
		.rgb_ctrl = 0,
		.bgcolor = 0x0000ff, /* set background color Black */
		.colorkey0 = 0, /* disable colorkey */
		.colorkey1 = 0, /* disable colorkey */
		.alpha = 0xff,	/* alpha value */
		.ipu_restart = 0x80001000, /* ipu restart */
		.fg_change = FG_CHANGE_ALL, /* change all initially */
		.fg0 = {16, 0, 0, 704, 573}, /* bpp, x, y, w, h */
		.fg1 = {16, 0, 0, 480, 272}, /* bpp, x, y, w, h */
	},
#elif defined(CONFIG_JZ4750_ANDROID_LCD_TOPPOLY_TD043MGEB1)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */ 
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* General TFT panel */
		LCD_CFG_MODE_TFT_24BIT | 	/* output 18bpp */
		LCD_CFG_HSP | 	/* Hsync polarity: active low */
		LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_BST_16,	/* 16words burst, enable out FIFO underrun irq */
		800, 480, 45, 1, 1, 40, 215, 10, 34,
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN | LCD_OSDC_F0EN| LCD_OSDC_F1EN|
		LCD_OSDC_ALPHAEN,// | /* Use OSD mode */
		.osd_ctrl = 0,		/* disable ipu,  */
		.rgb_ctrl = 0,
		.bgcolor = 0x0000ff, /* set background color Black */
		.colorkey0 = 0, /* disable colorkey */
		.colorkey1 = 0, /* disable colorkey */
		.alpha = 0xff,	/* alpha value */
		.ipu_restart = 0x80001000, /* ipu restart */
		.fg_change = FG_CHANGE_ALL, /* change all initially */
		.fg0 = {16, 0, 0, 800, 573}, /* bpp, x, y, w, h */
		.fg1 = {16, 0, 0, 800, 480}, /* bpp, x, y, w, h */
	 },
#endif
};

static struct jz4750lcd_info jz4750_info_tve = {
	.panel = {
		.cfg = LCD_CFG_TVEN | /* output to tve */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_RECOVER | /* underrun protect */
		LCD_CFG_MODE_INTER_CCIR656, /* Interlace CCIR656 mode */
		.ctrl =  LCD_CTRL_BST_16,//LCD_CTRL_OFUM |	/* 16words burst */
		TVE_WIDTH_PAL16, TVE_HEIGHT_PAL16, TVE_FREQ_PAL, 0, 0, 0, 0, 0, 0,
	},
	.osd = {
		 .osd_cfg = LCD_OSDC_OSDEN | /* Use OSD mode */
		 LCD_OSDC_ALPHAEN | /* enable alpha */
		 LCD_OSDC_F0EN |LCD_OSDC_F1EN,	/* enable Foreground0,Foreground1 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = LCD_RGBC_YCC, /* enable RGB => YUV */
		 .bgcolor = 0x000000ff, /* set background color Black */
		 .colorkey0 = 0x80000000, /* enable colorkey */
		 //.colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80000100, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {16,},	/*  */
		 .fg1 = {16,},
	},
};

static struct jz_android_din_t jz_panel[JZ_ANDROID_PANELNUM];
static int jz_panel_num = JZ_ANDROID_PANELNUM;
static int jz_panel_index = 0;

static struct android_display_info_t adi={0};

static struct jz4750lcd_info *jz4750_lcd_info = &jz4750_lcd_panel; /* default output to lcd panel */


/*********************************************TVE***********************************************/
struct jz4750tve_info jz4750_tve_info_PAL = {
	.ctrl = (4 << TVE_CTRL_YCDLY_BIT) | TVE_CTRL_SYNCT | TVE_CTRL_PAL | TVE_CTRL_SWRST,	/* PAL, SVIDEO */
	.frcfg = (23 << TVE_FRCFG_L1ST_BIT) | (625 << TVE_FRCFG_NLINE_BIT),
	.slcfg1 = (800<<TVE_SLCFG1_WHITEL_BIT) | (282<<TVE_SLCFG1_BLACKL_BIT),
	.slcfg2 = (296<<TVE_SLCFG2_VBLANKL_BIT) | (240<<TVE_SLCFG2_BLANKL_BIT),
	.slcfg3 = (72 <<TVE_SLCFG3_SYNCL_BIT),
	.ltcfg1 = (20<<TVE_LTCFG1_FRONTP_BIT) | (63<<TVE_LTCFG1_HSYNCW_BIT) | (78<<TVE_LTCFG1_BACKP_BIT),
	.ltcfg2 = ((TVE_WIDTH_PAL*2) << TVE_LTCFG2_ACTLIN_BIT) | (16 << TVE_LTCFG2_PREBW_BIT) | (61 << TVE_LTCFG2_BURSTW_BIT),
	.cfreq = 0x2a098acb,
	.cphase = (0 << TVE_CPHASE_INITPH_BIT) | (0 << TVE_CPHASE_ACTPH_BIT) | (1 << TVE_CPHASE_CCRSTP_BIT),
	.cbcrcfg = (59<<TVE_CBCRCFG_CBBA_BIT) | (59<<TVE_CBCRCFG_CRBA_BIT) | (137<<TVE_CBCRCFG_CBGAIN_BIT) | (137<<TVE_CBCRCFG_CRGAIN_BIT), /* CBGAIN CRGAIN??? */
	.wsscr = 0x00000070,	/* default value */
	.wsscfg1 = 0x0,
	.wsscfg2 = 0x0,
	.wsscfg3 = 0x0,
};

struct jz4750tve_info jz4750_tve_info_NTSC = {
	.ctrl = (4 << TVE_CTRL_YCDLY_BIT) | TVE_CTRL_SWRST,	/* NTSC, SVIDEO */
	.frcfg = (21 << TVE_FRCFG_L1ST_BIT) | (525 << TVE_FRCFG_NLINE_BIT),
	.slcfg1 = (800<<TVE_SLCFG1_WHITEL_BIT) | (282<<TVE_SLCFG1_BLACKL_BIT),
	.slcfg2 = (296<<TVE_SLCFG2_VBLANKL_BIT) | (240<<TVE_SLCFG2_BLANKL_BIT),
	.slcfg3 = (72 <<TVE_SLCFG3_SYNCL_BIT),
	.ltcfg1 = (16<<TVE_LTCFG1_FRONTP_BIT) | (63<<TVE_LTCFG1_HSYNCW_BIT) | (59<<TVE_LTCFG1_BACKP_BIT),
	.ltcfg2 = (1440 << TVE_LTCFG2_ACTLIN_BIT) | (22 << TVE_LTCFG2_PREBW_BIT) | (68 << TVE_LTCFG2_BURSTW_BIT),
	.cfreq = 0x21f07c1f,
	.cphase = (0x17 << TVE_CPHASE_INITPH_BIT) | (0 << TVE_CPHASE_ACTPH_BIT) | (1 << TVE_CPHASE_CCRSTP_BIT),
	.cbcrcfg = (59<<TVE_CBCRCFG_CBBA_BIT) | (0<<TVE_CBCRCFG_CRBA_BIT) | (137<<TVE_CBCRCFG_CBGAIN_BIT) | (137<<TVE_CBCRCFG_CRGAIN_BIT),
	.wsscr = 0x00000070,	/* default value */
	.wsscfg1 = 0x0,
	.wsscfg2 = 0x0,
	.wsscfg3 = 0x0,
};

struct jz4750tve_info *jz4750_tve_info = &jz4750_tve_info_PAL; /* default as PAL mode */
void jz4750tve_enable_tve(void)
{
	/* enable tve controller, enable DACn??? */
	jz4750_tve_info->ctrl = (jz4750_tve_info->ctrl | TVE_CTRL_DAPD) & ( ~( TVE_CTRL_DAPD1 | TVE_CTRL_DAPD2));
	jz4750_tve_info->ctrl &= ~TVE_CTRL_SWRST;
	REG_TVE_CTRL = jz4750_tve_info->ctrl;
}

/* turn off TVE, turn off DACn... */
void jz4750tve_disable_tve(void)
{
	jz4750_tve_info->ctrl &= ~TVE_CTRL_DAPD;/* DACn disabled??? */
	jz4750_tve_info->ctrl |= TVE_CTRL_SWRST;/* DACn disabled??? */
	REG_TVE_CTRL = jz4750_tve_info->ctrl;
}

void jz4750tve_set_tve_mode(struct jz4750tve_info *tve)
{
	REG_TVE_CTRL 		= tve->ctrl;
	REG_TVE_FRCFG 		= tve->frcfg;
	REG_TVE_SLCFG1 		= tve->slcfg1;
	REG_TVE_SLCFG2 		= tve->slcfg2;
	REG_TVE_SLCFG3 		= tve->slcfg3;
	REG_TVE_LTCFG1 		= tve->ltcfg1;
	REG_TVE_LTCFG2 		= tve->ltcfg2;
	REG_TVE_CFREQ 		= tve->cfreq;
	REG_TVE_CPHASE 		= tve->cphase;
	REG_TVE_CBCRCFG 	= tve->cbcrcfg;
	REG_TVE_WSSCR 		= tve->wsscr;
	REG_TVE_WSSCFG1 	= tve->wsscfg1;
	REG_TVE_WSSCFG2 	= tve->wsscfg2;
	REG_TVE_WSSCFG3 	= tve->wsscfg3;
}

void jz4750tve_init( int tve_mode )
{
	switch ( tve_mode ) {
	case PANEL_MODE_TVE_PAL:
		jz4750_tve_info = &jz4750_tve_info_PAL;
		break;
	case PANEL_MODE_TVE_NTSC:
		jz4750_tve_info = &jz4750_tve_info_NTSC;
		break;
	}

	jz4750tve_set_tve_mode( jz4750_tve_info );
}


void jz4750tve_outfmt_init(unsigned int outfmt)
{
	switch (outfmt) {
#ifdef CONFIG_SOC_JZ4750D
	case PANEL_OUT_FMT_YCBCR:
 		jz4750_tve_info_PAL.ctrl |= TVE_CTRL_EYCBCR;
		break;
#endif
	case PANEL_OUT_FMT_CVBS:
#ifdef CONFIG_SOC_JZ4750D
		/* Disable YCbCr */
 		jz4750_tve_info_PAL.ctrl &= ~TVE_CTRL_EYCBCR;
#endif
		jz4750_tve_info_PAL.ctrl |= TVE_CTRL_ECVBS;
		break;
	case PANEL_OUT_FMT_SVIDEO:
	default:
#ifdef CONFIG_SOC_JZ4750D
		/* Disable YCbCr */
 		jz4750_tve_info_PAL.ctrl &= ~TVE_CTRL_EYCBCR;
#endif
		jz4750_tve_info_PAL.ctrl &= ~TVE_CTRL_ECVBS;
		break;
	}
}

void jzfb_get_panel_size(unsigned int *w, unsigned *h)
{
	*w = jz4750_lcd_info->panel.w;
	*h = jz4750_lcd_info->panel.h;
}

static inline void lcd_display_on(void)
{
	__lcd_display_on();                     /* Turn on panel */
	__lcd_set_ena();	                /* Enable LCD Controller */
	schedule_timeout_uninterruptible(HZ/5); /* Wait 200ms */
	__lcd_set_backlight_level(255);         /* Turn of backlight */
}

static inline void lcd_display_off(void)
{
	__lcd_close_backlight();
	__lcd_clr_ena(); /* Quick Disable */
	__lcd_display_off();
}

/********************************************************************************************/
		
/************************************
 *      Jz475X Framebuffer ops
 ************************************/
static int jz4750fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			  u_int transp, struct fb_info *info)
{
	struct fb_info *fb = info;
	if (regno >= NR_PALETTE)
		return 1;
	if (fb->var.bits_per_pixel <= 16) {
		red	>>= 8;
		green	>>= 8;
		blue	>>= 8;

		red	&= 0xff;
		green	&= 0xff;
		blue	&= 0xff;
	}

	switch (fb->var.bits_per_pixel) {
	case 15:
		if (regno < 16)
			((u32 *)fb->pseudo_palette)[regno] =
				((red >> 3) << 10) | 
				((green >> 3) << 5) |
				(blue >> 3);
		break;
	case 16:
		if (regno < 16) {
			((u32 *)fb->pseudo_palette)[regno] =
				((red >> 3) << 11) | 
				((green >> 2) << 5) |
				(blue >> 3); 
		}
		break;
	case 17 ... 32:
		if (regno < 16)
			((u32 *)fb->pseudo_palette)[regno] =
				(red << 16) | 
				(green << 8) |
				(blue << 0); 
		break;
	}
	return 0;
}

static int jz4750fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int ret = 0, nret = -1;
        void __user *argp = (void __user *)arg;
	struct jz4750lcd_fg_t fg1;

	struct jz4750tve_mode jz4750_tve_mode;

	switch (cmd) {
	case FBIOSETBACKLIGHT:
		__lcd_set_backlight_level(arg);	/* We support 8 levels here. */
		break;
	case FBIODISPON:
		REG_LCD_STATE = 0; /* clear lcdc status */
		__lcd_slcd_special_on();
		REG_LCD_DA1 = virt_to_phys(dma1_desc0);
		__lcd_clr_dis();
		__lcd_set_ena(); /* enable lcdc */
		__lcd_display_on();
		break;
	case FBIODISPOFF:
		__lcd_display_off();
		if ( jz4750_lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD || 
			jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) /*  */
			__lcd_clr_ena(); /* Smart lcd and TVE mode only support quick disable */
		else
			__lcd_set_dis(); /* regular disable */
		break;
	case FBIOPRINT_REG:
		print_lcdc_registers();
		break;
	case FBIO_GET_MODE:
		print_dbg("fbio get mode\n");
		if (copy_to_user(argp, jz4750_lcd_info, sizeof(struct jz4750lcd_info)))
			return -EFAULT;
		break;
	case FBIO_SET_MODE:
		print_dbg("fbio set mode\n");
		if (copy_from_user(jz4750_lcd_info, argp, sizeof(struct jz4750lcd_info)))
			return -EFAULT;
		/* set mode */
		jz4750fb_set_mode(&jz4750_lcd_info->osd);
		break;
	case FBIO_DEEP_SET_MODE:
		print_dbg("fbio deep set mode\n");
		if (copy_from_user(jz4750_lcd_info, argp, sizeof(struct jz4750lcd_info)))
			return -EFAULT;
		jz4750fb_deep_set_mode(jz4750_lcd_info);
		break;

	case FBIO_MODE_SWITCH:
		print_dbg("lcd mode switch between tve and lcd, arg=%lu\n", arg);
		switch ( arg ) {
		case PANEL_MODE_TVE_PAL: 	/* switch to TVE_PAL mode */
		case PANEL_MODE_TVE_NTSC: 	/* switch to TVE_NTSC mode */
			jz4750lcd_info_switch_to_TVE(arg);
			jz4750tve_init(arg); /* tve controller init */
			jz4750tve_enable_tve();
			/* turn off lcd backlight */
			__lcd_display_off();
			break;
		case PANEL_MODE_LCD_PANEL: 	/* switch to LCD mode */
		default :
			/* turn off TVE, turn off DACn... */
			jz4750tve_disable_tve();
			jz4750_lcd_info = &jz4750_lcd_panel;
			/* turn on lcd backlight */
			__lcd_display_on();
			break;
		}
		jz4750fb_deep_set_mode(jz4750_lcd_info);
		break;
	case FBIO_GET_TVE_MODE:
		print_dbg("fbio get TVE mode\n");
		if (copy_to_user(argp, jz4750_tve_info, sizeof(struct jz4750tve_info)))
			return -EFAULT;
		break;
	case FBIO_SET_TVE_MODE:
		print_dbg("fbio set TVE mode\n");
		if (copy_from_user(jz4750_tve_info, argp, sizeof(struct jz4750tve_info)))
			return -EFAULT;
		/* set tve mode */
		jz4750tve_set_tve_mode(jz4750_tve_info);
		break;
	case FBIODISON_FG: //pass
		/*lcdc_enable_fg1();*/
		print_dbg("lcdc_disable_fg1()\n");
		jz4750_lcd_info->osd.osd_cfg |= LCD_OSDC_F1EN;
		__lcd_enable_f1();
		break;
	case FBIODISOFF_FG://pass
		/*lcdc_disable_fg1();*/
		jz4750_lcd_info->osd.osd_cfg &= ~LCD_OSDC_F1EN;
		__lcd_disable_f1();
		break;

	case FBIO_SET_LCD_TO_TVE:
		/*tve PAL NTSC, LCD, */
		//jz4750_tve_info
		/* 1. display off 2. enable ipu_restart 3. use ipu as input 4. after run_ipu, display on lcd*/
		if (copy_from_user(&jz4750_tve_mode, argp, sizeof(struct jz4750tve_mode)))
			return -EFAULT;
		if (jz4750_tve_mode.mode == PANEL_MODE_LCD_PANEL) {
			jz4750tve_disable_tve();
			jz4750lcd_info_switch_to_lcd();
			__lcd_display_on();
		}
		else {
			jz4750lcd_info_switch_to_TVE(jz4750_tve_mode.mode);
			jz4750tve_outfmt_init(jz4750_tve_mode.out_fmt);
			jz4750tve_init(jz4750_tve_mode.mode);
			jz4750tve_enable_tve();
			__lcd_display_off();
		}
		jz4750fb_deep_set_mode(jz4750_lcd_info);
		break;

	case FBIO_SET_IPU_TO_LCD:
		/* 1. display off 2. enable ipu_restart 3. use ipu as input 4. after run_ipu, display on lcd*/
		__lcd_set_dis();
		__lcd_fg1_use_ipu();
		REG_LCD_BGC = jz4750_lcd_info->osd.ipu_restart;
		jz4750fb_deep_set_mode(jz4750_lcd_info);
		break;
	case FBIO_SET_FRM_TO_LCD:
		__lcd_fg1_use_dma_chan1();
		jz4750fb_deep_set_mode(jz4750_lcd_info);
		break;
	case FBIO_CHANGE_SIZE:
		/*fg1_change_size();*/
		if(!(REG_LCD_OSDC & LCD_OSDC_F1EN))
			return -EFAULT;
		if (copy_from_user(&fg1, argp, sizeof(struct jz4750lcd_fg_t)))
			return -EFAULT;
		jz4750_lcd_info->osd.fg1.w = fg1.w;
		jz4750_lcd_info->osd.fg1.h = fg1.h;
		jz4750fb_foreground_resize(&jz4750_lcd_info->osd);
		break;
	case FBIO_CHANGE_POSITION:
		if (copy_from_user(&fg1, argp, sizeof(struct jz4750lcd_fg_t)))
			return -EFAULT;
		jz4750_lcd_info->osd.fg1.x = fg1.x;
		jz4750_lcd_info->osd.fg1.y = fg1.y;
		jz4750fb_foreground_move(&jz4750_lcd_info->osd);
		break;
	case FBIO_SET_BG_COLOR://pass
		jz4750_lcd_info->osd.bgcolor = arg;
		/*lcdc_set_bgcolor(arg);*/
		REG_LCD_BGC = jz4750_lcd_info->osd.bgcolor;
		break;
	case FBIO_SET_IPU_RESTART_VAL:
		/*lcdc_set_ipu_restart_val(arg);*/
		jz4750_lcd_info->osd.ipu_restart &= ~LCD_IPUR_IPURMASK;
		jz4750_lcd_info->osd.ipu_restart |= (arg & LCD_IPUR_IPURMASK);
		__lcd_set_ipu_restart_triger(arg);
		break;
	case FBIO_SET_IPU_RESTART_ON:
		/*lcdc_enable_ipu_restart();*/
		__lcd_enable_ipu_restart();
		break;
	case FBIO_SET_IPU_RESTART_OFF:
		/*lcdc_disable_ipu_restart();*/
		__lcd_disable_ipu_restart();
		break;
	case FBIO_ANDROID_CTL: 
		print_dbg("ANDROID supply!\n");
		if (copy_from_user(&adi, argp, sizeof(struct android_display_info_t)))
			return -EFAULT;
		switch (adi.flag) {
		case ANDROID_SET_FG0_ALPHA: //pass
			/*lcdc_set_alpha(arg);*/
			if (adi.fg0_alpha > 0xFF){
				/*lcdc_disable_alpha();*/
				printk("alpha value is to large,so shut down the alpha");
				jz4750_lcd_info->osd.osd_cfg &= ~LCD_OSDC_ALPHAEN;
				__lcd_disable_alpha();	
			}
			else{
				/*lcdc_enable_alpha();*/
				jz4750_lcd_info->osd.osd_cfg |= LCD_OSDC_ALPHAEN;
				jz4750_lcd_info->osd.alpha = adi.fg0_alpha;
				REG_LCD_ALPHA = adi.fg0_alpha;
				__lcd_enable_alpha();
			}
			break;
		case ANDROID_SET_FG0_COLORKEY:	
			printk("============fg0_colorkey=%x\n",adi.fg0_colorkey);
			if (adi.fg0_colorkey > 0xFFFFFF){
				/*lcdc_disable_colorkey0;*/
				jz4750_lcd_info->osd.colorkey0 = 0;
				__lcd_disable_colorkey0();
			}
			else{
				if ( jz4750_lcd_info->osd.fg0.bpp == 16 ) { // 16bpp
					adi.fg0_colorkey &= 0x00f8fcf8;
					jz4750_lcd_info->osd.colorkey0 = adi.fg0_colorkey;
				}
				else if ( jz4750_lcd_info->osd.fg0.bpp == 15 ) { // 15bpp
					adi.fg0_colorkey &= 0x00f8f8f8;
					jz4750_lcd_info->osd.colorkey0 = adi.fg0_colorkey;
				}
				else { // > 16bpp
					jz4750_lcd_info->osd.colorkey0 = adi.fg0_colorkey;
				}
				__lcd_set_colorkey0(jz4750_lcd_info->osd.colorkey0);
				__lcd_enable_colorkey0();
			}
			break;
		case  ANDROID_SET_FG0_ENABLE://pass
			if( adi.fg0_enable ){ 
				print_dbg("lcdc_enable_fg0()\n");
				jz4750_lcd_info->osd.osd_cfg |= LCD_OSDC_F0EN;
				__lcd_enable_f0();
			}
			else{
				print_dbg("lcdc_disable_fg0()\n");
				jz4750_lcd_info->osd.osd_cfg &= ~LCD_OSDC_F0EN;
				__lcd_disable_f0();

			}
			break;
			
		case  ANDROID_SET_FG1_POS://pass
			print_dbg("fg1 pos set\n");
			__lcd_clr_ena();
			if (jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN){
				jz4750_lcd_info->osd.fg1.x = adi.fg1_x+8;
				jz4750_lcd_info->osd.fg1.y = adi.fg1_y+10;	
			}
			else{
				jz4750_lcd_info->osd.fg1.x = adi.fg1_x;
				jz4750_lcd_info->osd.fg1.y = adi.fg1_y;
			}
			jz4750fb_foreground_move(&jz4750_lcd_info->osd);
			__lcd_set_ena();
			break;	
		case  ANDROID_SET_FG1_SIZE:
			__lcd_clr_ena();	
			if ( REG_LCD_OSDCTRL & LCD_OSDCTRL_IPU){
				printk("fg1_ipu\n");
				if(!(REG_LCD_OSDC & LCD_OSDC_F1EN))
					return -EFAULT;
				//	__lcd_clr_ena();
				jz4750_lcd_info->osd.fg1.w = ((adi.fg1_w+3)<<2)>>2;
				jz4750_lcd_info->osd.fg1.h = adi.fg1_h;
				jz4750fb_foreground_resize(&jz4750_lcd_info->osd);
				__lcd_enable_ipu_restart();
				jz4750fb_deep_set_mode(jz4750_lcd_info);	
			}
			else{
				printk("fg1_dma1\n");
				if(!(REG_LCD_OSDC & LCD_OSDC_F1EN))
					return -EFAULT;
				jz4750_lcd_info->osd.fg1.w = ((adi.fg1_w+3)>>2)<<2;
				jz4750_lcd_info->osd.fg1.h = adi.fg1_h;
				jz4750fb_foreground_resize(&jz4750_lcd_info->osd);	
			}
			__lcd_set_ena();
			break;
		case ANDROID_SET_FG1_ENABLE://pass
			if( adi.fg1_enable ){
				print_dbg("lcdc_enable_fg1()\n");
				jz4750_lcd_info->osd.osd_cfg |= LCD_OSDC_F1EN;
				__lcd_enable_f1();
			}
			else{
				print_dbg("lcdc_disable_fg1()\n");
				jz4750_lcd_info->osd.osd_cfg &= ~LCD_OSDC_F1EN;
				__lcd_disable_f1();
			}
			break;
		case ANDROID_SET_FG1_IPU_DIRECT:
			print_dbg("fg1 use ipu or dma1\n");
			if( adi.fg1_short_cut){
				printk("fg1 use ipu\n");
				//__lcd_set_dis();
				__lcd_clr_ena();
				jz4750_lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_IPU;
				jz4750fb_deep_set_mode(jz4750_lcd_info);
			}
			else{	printk("fg1 use dma1\n");
				__lcd_set_dis();
				//__lcd_clr_ena();
				jz4750_lcd_info->osd.osd_ctrl &= ~LCD_OSDCTRL_IPU;
				jz4750fb_deep_set_mode(jz4750_lcd_info);
			}
			break;
		case ANDROID_GET_PANEL_SIZE:
			printk("+++++++++++++++++++++++++++++jz_panel_index = %d\n",jz_panel_index);
			adi.fg1_w = jz_panel[jz_panel_index].w;
			adi.fg1_h = jz_panel[jz_panel_index].h;
			if (copy_to_user(argp, &adi, sizeof(struct android_display_info_t)))
				return -EFAULT;
			break;
		default:
			printk("FG1:%s, unknown android command(0x%x)", __FILE__, adi.flag);
			return nret;
		}
		break;
	default:
		printk("%s, unknown command(0x%x)", __FILE__, cmd);
		return nret;
	}

	return ret;
}

/* Use mmap /dev/fb can only get a non-cacheable Virtual Address. */
static int jz4750fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{

	struct fb_info *fb = info;
	unsigned long start;
	unsigned long off;
	u32 len;
	off = vma->vm_pgoff << PAGE_SHIFT;
	//fb->fb_get_fix(&fix, PROC_CONSOLE(info), info);

	/* frame buffer memory */
	start = fb->fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + fb->fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */

 	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
 	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
//	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;	/* Write-Back */

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}

	return 0;
}
  /*end of Foreground 1 ops*/

static int jz4750fb0_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			  u_int transp, struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned short *ptr, ctmp;

	if (regno >= NR_PALETTE)
		return 1;

	cfb->palette[regno].red		= red ;
	cfb->palette[regno].green	= green;
	cfb->palette[regno].blue	= blue;
	if (cfb->fb0.var.bits_per_pixel <= 16) {
		red	>>= 8;
		green	>>= 8;
		blue	>>= 8;

		red	&= 0xff;
		green	&= 0xff;
		blue	&= 0xff;
	}
	switch (cfb->fb0.var.bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
		if (((jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_SINGLE_MSTN ) ||
		    ((jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_DUAL_MSTN )) {
			ctmp = (77L * red + 150L * green + 29L * blue) >> 8;
			ctmp = ((ctmp >> 3) << 11) | ((ctmp >> 2) << 5) |
				(ctmp >> 3);
		} else {
			/* RGB 565 */
			if (((red >> 3) == 0) && ((red >> 2) != 0))
			red = 1 << 3;
			if (((blue >> 3) == 0) && ((blue >> 2) != 0))
				blue = 1 << 3;
			ctmp = ((red >> 3) << 11) 
				| ((green >> 2) << 5) | (blue >> 3);
		}

		ptr = (unsigned short *)lcd_palette;
		ptr = (unsigned short *)(((u32)ptr)|0xa0000000);
		ptr[regno] = ctmp;

		break;
		
	case 15:
		if (regno < 16)
			((u32 *)cfb->fb0.pseudo_palette)[regno] =
				((red >> 3) << 10) | 
				((green >> 3) << 5) |
				(blue >> 3);
		break;
	case 16:
		if (regno < 16) {
			((u32 *)cfb->fb0.pseudo_palette)[regno] =
				((red >> 3) << 11) | 
				((green >> 2) << 5) |
				(blue >> 3); 
		}
		break;
	case 17 ... 32:
		if (regno < 16)
			((u32 *)cfb->fb0.pseudo_palette)[regno] =
				(red << 16) | 
				(green << 8) |
				(blue << 0); 

		break;
	}
	return 0;
}


static int jz4750fb0_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int ret = 0, nret = -1;
        void __user *argp = (void __user *)arg;
	struct jz4750lcd_fg_t fg0;

	switch (cmd) {
	case FBIOSETBACKLIGHT:
		__lcd_set_backlight_level(arg);	/* We support 8 levels here. */
		break;
	case FBIODISPON:
		REG_LCD_STATE = 0; /* clear lcdc status */
		__lcd_slcd_special_on();
		__lcd_clr_dis();
		__lcd_set_ena(); /* enable lcdc */
		__lcd_display_on();
		break;
	case FBIODISPOFF:
		__lcd_display_off();
		if ( jz4750_lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD || 
			jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) /*  */
			__lcd_clr_ena(); /* Smart lcd and TVE mode only support quick disable */
		else
			__lcd_set_dis(); /* regular disable */
		break;
	case FBIOPRINT_REG:
		print_lcdc_registers();
		break;
	case FBIO_GET_MODE:
		print_dbg("fbio get mode\n");
		if (copy_to_user(argp, jz4750_lcd_info, sizeof(struct jz4750lcd_info)))
//		if (copy_to_user(argp, &jz4750_lcd_info->osd, sizeof(struct jz4750lcd_osd_t)))
			return -EFAULT;
		break;
	case FBIO_SET_MODE:
		print_dbg("fbio set mode\n");
		if (copy_from_user(jz4750_lcd_info, argp, sizeof(struct jz4750lcd_info)))
//		if (copy_to_user(argp, &jz4750_lcd_info->osd, sizeof(struct jz4750lcd_osd_t)))
			return -EFAULT;
		/* set mode */
		jz4750fb_set_mode(&jz4750_lcd_info->osd);
//		jz4750fb_set_mode(jz4750_lcd_info);
		break;
	case FBIO_DEEP_SET_MODE:
		print_dbg("fbio deep set mode\n");
		if (copy_from_user(jz4750_lcd_info, argp, sizeof(struct jz4750lcd_info)))
			return -EFAULT;
		jz4750fb_deep_set_mode(jz4750_lcd_info);
		break;

	case FBIO_GET_TVE_MODE:
		print_dbg("fbio get TVE mode\n");
		if (copy_to_user(argp, jz4750_tve_info, sizeof(struct jz4750tve_info)))
			return -EFAULT;
		break;
	case FBIO_SET_TVE_MODE:
		print_dbg("fbio set TVE mode\n");
		if (copy_from_user(jz4750_tve_info, argp, sizeof(struct jz4750tve_info)))
			return -EFAULT;
		/* set tve mode */
		jz4750tve_set_tve_mode(jz4750_tve_info);
		break;

	case FBIODISON_FG: //pass
		/*lcdc_enable_fg0();*/
		jz4750_lcd_info->osd.osd_cfg |= LCD_OSDC_F0EN;
		__lcd_enable_f0();
		break;
	case FBIODISOFF_FG://pass
		/*lcdc_disable_fg0();*/
		print_dbg("lcdc_disable_fg0()\n");
		jz4750_lcd_info->osd.osd_cfg &= ~LCD_OSDC_F0EN;
		__lcd_disable_f0();
		break;
	case FBIO_CHANGE_SIZE:
		/*fg0_change_size();*/
		if(!(REG_LCD_OSDC & LCD_OSDC_F0EN))
			return -EFAULT;
		if (copy_from_user(&fg0, argp, sizeof(struct jz4750lcd_fg_t)))
			return -EFAULT;
		jz4750_lcd_info->osd.fg0.w = fg0.w;
		jz4750_lcd_info->osd.fg0.h = fg0.h;
		jz4750fb0_foreground_resize(&jz4750_lcd_info->osd);
		break;
	case FBIO_CHANGE_POSITION:
		if (copy_from_user(&fg0, argp, sizeof(struct jz4750lcd_fg_t)))
			return -EFAULT;
		jz4750_lcd_info->osd.fg0.x = fg0.x;
		jz4750_lcd_info->osd.fg0.y = fg0.y;
		jz4750fb0_foreground_move(&jz4750_lcd_info->osd);
		break;
	case FBIO_SET_BG_COLOR://pass
		jz4750_lcd_info->osd.bgcolor = arg;
		/*lcdc_set_bgcolor(arg);*/
		REG_LCD_BGC = jz4750_lcd_info->osd.bgcolor;
		break;
	case FBIO_ALPHA_ON://pass
		/*lcdc_enable_alpha();*/
		jz4750_lcd_info->osd.osd_cfg |= LCD_OSDC_ALPHAEN;
		__lcd_enable_alpha();
		break;
	case FBIO_ALPHA_OFF://pass
		/*lcdc_disable_alpha();*/
		jz4750_lcd_info->osd.osd_cfg &= ~LCD_OSDC_ALPHAEN;
		__lcd_disable_alpha();
		break;
	case FBIO_SET_ALPHA_VAL://pass
		jz4750_lcd_info->osd.alpha = arg;
		/*lcdc_set_alpha(arg);*/
		REG_LCD_ALPHA = jz4750_lcd_info->osd.alpha;
		break;
	case FBIO_ANDROID_CTL:  
		if (copy_from_user(&adi, argp, sizeof(struct android_display_info_t)))
			return -EFAULT;

		switch (adi.flag) {
		case ANDROID_GET_DISPLAY_NUM://pass
			adi.fg0_number = jz_panel_num;
			if (copy_to_user(argp, &adi, sizeof(struct android_display_info_t)))
				return -EFAULT;
			break;
		case ANDROID_GET_DISPLAY_INFO://pass
			adi.fg0_w = jz_panel[adi.fg0_index].w;
			adi.fg0_h = jz_panel[adi.fg0_index].h;
			if (copy_to_user(argp, &adi, sizeof(struct android_display_info_t)))
				return -EFAULT;
			break;
		case ANDROID_SET_DISPLAY_INDEX: //pass
			printk("fg0_index=%d\n",adi.fg0_index);
			switch (adi.fg0_index) {
			case PANEL_MODE_TVE_PAL: 	/* switch to TVE_PAL mode */
				__lcd_clr_ena();
				jz4750lcd_info_switch_to_TVE(adi.fg0_index);
				jz4750tve_init(adi.fg0_index); /* tve controller init */
				jz4750tve_enable_tve();
				/* turn off lcd backlight */
				__lcd_display_off();
				break;
			case PANEL_MODE_LCD_PANEL: 	/* switch to LCD mode */
				/* turn off TVE, turn off DACn... */
				jz4750tve_disable_tve();
				__lcd_clr_ena();
				jz4750_lcd_info = &jz4750_lcd_panel;
				/* turn on lcd backlight */
				__lcd_display_on();
				break;
			default :
				printk("FG0:%s, android has no this style panel(0x%x)", __FILE__, adi.fg0_index);
				return nret;
			}
			jz_panel_index = adi.fg0_index;
			jz4750fb_deep_set_mode(jz4750_lcd_info);
			//print_lcdc_registers();
			break;

		default:
			printk("FG0:%s, unknown android command(0x%x)", __FILE__, adi.flag);
			return nret;
		}
		break;
	default:
		printk("FG0:%s, unknown command(0x%x)", __FILE__, cmd);
		return nret;
	}

	return ret;
}


static int jz4750fb0_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned long start;
	unsigned long off;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;
	//fb->fb_get_fix(&fix, PROC_CONSOLE(info), info);

	/* frame buffer memory */
	start = cfb->fb0.fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + cfb->fb0.fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */

 	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
 	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
//	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;	/* Write-Back */

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}
  /* end of Foreground 0 ops*/


/* checks var and eventually tweaks it to something supported,
 * DO NOT MODIFY PAR */
static int jz4750fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
/*	if((var->rotate & 1) != (info->var.rotate & 1)) {
		if((var->xres != info->var.yres) ||
		   (var->yres != info->var.xres) ||
		   (var->xres_virtual != info->var.yres) ||
		   (var->yres_virtual > 
		    info->var.xres * ANDROID_NUMBER_OF_BUFFERS) ||
		   (var->yres_virtual < info->var.xres )) {
			return -EINVAL;
		}
	}
	else {
		if((var->xres != info->var.xres) ||
		   (var->yres != info->var.yres) ||
		   (var->xres_virtual != info->var.xres) ||
		   (var->yres_virtual > 
		    info->var.yres * ANDROID_NUMBER_OF_BUFFERS) ||
		   (var->yres_virtual < info->var.yres )) {
			return -EINVAL;
		}
	}
	if((var->xoffset != info->var.xoffset) ||
	   (var->bits_per_pixel != info->var.bits_per_pixel)) {// ||
//	   (var->grayscale != info->var.grayscale)) {
		return -EINVAL;
		}
*/
	return 0;
}


/* 
 * set the video mode according to info->var
 */
static int jz4750fb_set_par(struct fb_info *info)
{
	//dprintk("jz4750fb_set_par, not implemented\n");
	return 0;
}


/*
 * (Un)Blank the display.
 * Fix me: should we use VESA value?
 */
static int jz4750fb_blank(int blank_mode, struct fb_info *info)
{
	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		//case FB_BLANK_NORMAL:
			/* Turn on panel */
		__lcd_set_ena();
		__lcd_display_on();
		break;

	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		/* Turn off panel */
#if 0
		__lcd_display_off();
		__lcd_set_dis();
#endif
		break;
	default:
		break;

	}
	return 0;
}

/* 
 * pan display
 */
static int jz4750fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct fb_info *fb = info;
	struct jz4750lcd_info *lcd_info = jz4750_lcd_info;
	int dy;	
	int fg1_line_size;

	if (!var || !fb) {
		return -EINVAL;
	}

	if (var->xoffset - fb->var.xoffset) {
		/* No support for X panning for now! */
		return -EINVAL;
	}

	printk("pan frame rect %d %d %d %d\n",
	       var->reserved[1] & 0xffff,
	       var->reserved[1] >> 16, var->reserved[2] & 0xffff,
	       var->reserved[2] >> 16);

	/* TODO: Wait for current frame to finished */

	dy = var->yoffset;// - fb->var.yoffset; 
	fg1_line_size =  lcd_info->osd.fg1.w * lcd_info->osd.fg1.bpp/8 ;
	fg1_line_size = ((fg1_line_size+3)>>2)<<2; 
	if (dy) {
		dma1_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame + (fb->fix.line_length * dy));
		if ( lcd_info->panel.cfg & LCD_CFG_TVEN )  /* TVE mode */
			  dma1_desc1->databuf = (unsigned int)virt_to_phys((void *)(lcd_frame + fg1_line_size) + (fb->fix.line_length * dy));
		dma_cache_wback((unsigned int)(dma1_desc0), sizeof(struct jz4750_lcd_dma_desc));

	}
	else {
		dma1_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame);
		if ( lcd_info->panel.cfg & LCD_CFG_TVEN )  /* TVE mode */
			dma1_desc1->databuf = (unsigned int)virt_to_phys((void *)(lcd_frame + fg1_line_size));
		dma_cache_wback((unsigned int)(dma1_desc0), sizeof(struct jz4750_lcd_dma_desc));
	}

	return 0;
}

static int jz4750fb0_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	unsigned long irq_flags;
	struct fb_info *fb = info;
	struct jz4750lcd_info *lcd_info = jz4750_lcd_info;
	struct lcd_cfb_info *cfb = jz4750fb_info;
	int fg0_line_size;
	int dy;

	if (!(REG_LCD_OSDC & LCD_OSDC_F0EN))
		return 0;

	if (!var || !fb) {
		return -EINVAL;
	}
	if (var->xoffset - fb->var.xoffset) {
		/* No support for X panning for now! */
		return -EINVAL;
	}

	dy = var->yoffset;// - fb->var.yoffset; 
	fb->fix.line_length = ( jz4750_lcd_panel.osd.fg0.w * (lcd_info->osd.fg0.bpp) / 8);
	fg0_line_size = ( jz4750_lcd_panel.osd.fg0.w * (lcd_info->osd.fg0.bpp) / 8);
	fg0_line_size = ((fg0_line_size + 3) >> 2) << 2; 
	
	if (dy) {
		dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame0 + (fb->fix.line_length * dy));
		if ( lcd_info->panel.cfg & LCD_CFG_TVEN )  /* TVE mode */	
			dma0_desc1->databuf = (unsigned int)virt_to_phys((void *)(lcd_frame0 + fg0_line_size) + (fb->fix.line_length * dy));
		dma_cache_wback((unsigned int)(dma0_desc0), sizeof(struct jz4750_lcd_dma_desc));

	}
	else {
		dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame0);
		if ( lcd_info->panel.cfg & LCD_CFG_TVEN )  /* TVE mode */
			dma0_desc1->databuf = (unsigned int)virt_to_phys((void *)(lcd_frame0 + fg0_line_size ));
		dma_cache_wback((unsigned int)(dma0_desc0), sizeof(struct jz4750_lcd_dma_desc));

	}

	/* Wait for current frame to finished */

	spin_lock_irqsave(cfb->update_lock, irq_flags);

	REG_LCD_STATE &= ~LCD_STATE_EOF; // Clear previous EOF flag
	__lcd_enable_eof_intr();

	cfb->frame_requested++;

	spin_unlock_irqrestore(cfb->update_lock, irq_flags);

	if (cfb->frame_requested != cfb->frame_done)
		wait_event_interruptible_timeout(
			cfb->frame_wq, cfb->frame_done == cfb->frame_requested, HZ/50);

	__lcd_disable_eof_intr();

	return 0;
}

/* use default function cfb_fillrect, cfb_copyarea, cfb_imageblit */
static struct fb_ops jz4750fb_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg		= jz4750fb_setcolreg,
	.fb_check_var 		= jz4750fb_check_var,
	.fb_set_par 		= jz4750fb_set_par,
	.fb_blank		= jz4750fb_blank,
	.fb_pan_display		= jz4750fb_pan_display,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
	.fb_mmap		= jz4750fb_mmap,
	.fb_ioctl		= jz4750fb_ioctl,
};

/* use default function cfb_fillrect, cfb_copyarea, cfb_imageblit */
static struct fb_ops jz4750fb0_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg		= jz4750fb0_setcolreg,
	.fb_check_var 		= jz4750fb_check_var,
	.fb_set_par 		= jz4750fb_set_par,
	.fb_blank		= jz4750fb_blank,
	.fb_pan_display		= jz4750fb0_pan_display,
	.fb_fillrect		= cfb_fillrect,
	.fb_copyarea		= cfb_copyarea,
	.fb_imageblit		= cfb_imageblit,
	.fb_mmap		= jz4750fb0_mmap,
	.fb_ioctl		= jz4750fb0_ioctl,
};

/***************************************************************/
extern void show_tlb(void);

static void jz_lcd_add_wired_entry(unsigned long entrylo0, unsigned long entrylo1,
				unsigned long entryhi, unsigned long pagemask)
{
	unsigned long flags;
	unsigned long wired;
	unsigned long old_pagemask;
	unsigned long old_ctx;

	/* We will lock an 4MB page size entry to map the 4MB reserved IPU memory */
	entrylo0 = entrylo0 >> 6;
	entrylo0 |= 0x7 | (2 << 3);
	entrylo1 = entrylo1 >> 6;
	entrylo1 |= 0x7 | (2 << 3);

	local_irq_save(flags);
	/* Save old context and create impossible VPN2 value */
	old_ctx = read_c0_entryhi() & 0xff;
	old_pagemask = read_c0_pagemask();  
	wired = read_c0_wired();  
	write_c0_wired(wired + 1);
	write_c0_index(wired); 
	BARRIER;
//	entryhi &= ~0xff;	
//	entryhi |= g_asid;	
//	entryhi |= old_ctx;	
	write_c0_pagemask(pagemask);
	write_c0_entryhi(entryhi);
	write_c0_entrylo0(entrylo0);
	write_c0_entrylo1(entrylo1);
	BARRIER;
	tlb_write_indexed();
	BARRIER;

	write_c0_entryhi(old_ctx);
	BARRIER;
	write_c0_pagemask(old_pagemask);
	local_flush_tlb_all();
	local_irq_restore(flags);

#if defined LCD_DEBUG
	printk("\nold_ctx=%03ld\n", old_ctx);
	show_tlb();
#endif
}

static void jz_del_wired_entry( void )
{
	unsigned long flags;	
	unsigned long wired;	

	local_irq_save(flags);
	wired = read_c0_wired();
	if (wired) {
		write_c0_wired(0);
	}
	local_irq_restore(flags);
}

/***********************************************/

static int jz4750fb_set_var(struct fb_var_screeninfo *var, int con,
			struct fb_info *info)
{
	struct fb_info *fb = info;
	struct jz4750lcd_info *lcd_info = jz4750_lcd_info;
	int chgvar = 0;

	if (con == 0) {
		var->height	            = lcd_info->osd.fg0.h;	
		var->width	            = lcd_info->osd.fg0.w;
		var->bits_per_pixel	    = lcd_info->osd.fg0.bpp;
	}
	else {
		var->height	            = lcd_info->osd.fg1.h;
		var->width	            = lcd_info->osd.fg1.w;
		var->bits_per_pixel	    = lcd_info->osd.fg1.bpp;
	}

	var->vmode                  = FB_VMODE_NONINTERLACED;
//	var->vmode                  = FB_VMODE_DOUBLE
	var->activate               = fb->var.activate;
	var->xres                   = var->width;
	var->yres                   = var->height;
	var->xres_virtual           = var->width;
	var->yres_virtual           = var->height * ANDROID_NUMBER_OF_BUFFERS;
	var->xoffset                = 0;
	var->yoffset                = 0;
	var->pixclock               = KHZ2PICOS(jz_clocks.pixclk/1000);

	var->left_margin            = lcd_info->panel.elw;
	var->right_margin           = lcd_info->panel.blw;
	var->upper_margin           = lcd_info->panel.efw;
	var->lower_margin           = lcd_info->panel.bfw;
	var->hsync_len              = lcd_info->panel.hsw;
	var->vsync_len              = lcd_info->panel.vsw;
	var->sync                   = 0;
	var->activate               = FB_ACTIVATE_NOW;

	/*
	 * CONUPDATE and SMOOTH_XPAN are equal.  However,
	 * SMOOTH_XPAN is only used internally by fbcon.
	 */
	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = fb->var.xoffset;
		var->yoffset = fb->var.yoffset;
	}

	if (var->activate & FB_ACTIVATE_TEST)
		return 0;

	if ((var->activate & FB_ACTIVATE_MASK) != FB_ACTIVATE_NOW)
		return -EINVAL;

	if (fb->var.xres != var->xres)
		chgvar = 1;
	if (fb->var.yres != var->yres)
		chgvar = 1;
	if (fb->var.xres_virtual != var->xres_virtual)
		chgvar = 1;
	if (fb->var.yres_virtual != var->yres_virtual)
		chgvar = 1;
	if (fb->var.bits_per_pixel != var->bits_per_pixel)
		chgvar = 1;

	//display = fb_display + con;

	var->red.msb_right	= 0;
	var->green.msb_right	= 0;
	var->blue.msb_right	= 0;

	switch(var->bits_per_pixel){
	case 1:	/* Mono */
		fb->fix.visual	= FB_VISUAL_MONO01;
		fb->fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 2:	/* Mono */
		var->red.offset		= 0;
		var->red.length		= 2;
		var->green.offset	= 0;
		var->green.length	= 2;
		var->blue.offset	= 0;
		var->blue.length	= 2;

		fb->fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		fb->fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 4:	/* PSEUDOCOLOUR*/
		var->red.offset		= 0;
		var->red.length		= 4;
		var->green.offset	= 0;
		var->green.length	= 4;
		var->blue.offset	= 0;
		var->blue.length	= 4;

		fb->fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		fb->fix.line_length	= var->xres / 2;
		break;
	case 8:	/* PSEUDOCOLOUR, 256 */
		var->red.offset		= 0;
		var->red.length		= 8;
		var->green.offset	= 0;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;

		fb->fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		fb->fix.line_length	= var->xres ;
		break;
	case 15: /* DIRECTCOLOUR, 32k */
		var->bits_per_pixel	= 15;
		var->red.offset		= 10;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 5;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		fb->fix.visual	= FB_VISUAL_DIRECTCOLOR;
		fb->fix.line_length	= var->xres_virtual * 2;
		break;
	case 16: /* DIRECTCOLOUR, 64k */
		var->bits_per_pixel	= 16;
		var->red.offset		= 11;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 6;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		fb->fix.visual	= FB_VISUAL_TRUECOLOR;
		fb->fix.line_length	= var->xres_virtual * 2;
		break;
	case 17 ... 32:
		/* DIRECTCOLOUR, 256 */
		var->bits_per_pixel	= 32;

		var->red.offset		= 16;
		var->red.length		= 8;
		var->green.offset	= 8;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;
		var->transp.offset  	= 24;
		var->transp.length 	= 8;

		fb->fix.visual	= FB_VISUAL_TRUECOLOR;
		fb->fix.line_length	= var->xres_virtual * 4;
		break;

	default: /* in theory this should never happen */
		printk(KERN_WARNING "%s: don't support for %dbpp\n",
		       fb->fix.id, var->bits_per_pixel);
		break;
	}

	fb->var = *var;
	fb->var.activate &= ~FB_ACTIVATE_ALL;

	/*
	 * Update the old var.  The fbcon drivers still use this.
	 * Once they are using cfb->fb.var, this can be dropped.
	 *					--rmk
	 */
	//display->var = cfb->fb.var;
	/*
	 * If we are setting all the virtual consoles, also set the
	 * defaults used to create new consoles.
	 */
	fb_set_cmap(&fb->cmap, fb);
	return 0;
}

static struct lcd_cfb_info * jz4750fb_alloc_fb_info(void)
{
 	struct lcd_cfb_info *cfb;
	cfb = kmalloc(sizeof(struct lcd_cfb_info) + sizeof(u32) * 16, GFP_KERNEL);

	if (!cfb)
		return NULL;

	jz4750fb_info = cfb;

	memset(cfb, 0, sizeof(struct lcd_cfb_info) );

	cfb->currcon		= -1;

	/* Foreground 1 -- fb */
	strcpy(cfb->fb.fix.id, "jzlcd-fg1");
	cfb->fb.flags		= FBINFO_FLAG_DEFAULT;
	cfb->fb.fix.type	= FB_TYPE_PACKED_PIXELS;	
	cfb->fb.fix.type_aux	= 0;
	cfb->fb.fix.xpanstep	= 1;
	cfb->fb.fix.ypanstep	= 1;
	cfb->fb.fix.ywrapstep	= 0;
	cfb->fb.fix.accel	= FB_ACCEL_NONE;
	
	cfb->fb.var.nonstd	= 0;
	cfb->fb.var.activate	= FB_ACTIVATE_NOW;
	cfb->fb.var.height	= -1;
	cfb->fb.var.width	= -1;
	cfb->fb.var.accel_flags	= FB_ACCELF_TEXT;
	
	cfb->fb.fbops		= &jz4750fb_ops;
	cfb->fb.flags		= FBINFO_FLAG_DEFAULT;
		
	cfb->fb.pseudo_palette	= (void *)(cfb + 1);
	
	switch (jz4750_lcd_info->osd.fg1.bpp) {
	case 1:
		fb_alloc_cmap(&cfb->fb.cmap, 4, 0);
		break;
	case 2:
		fb_alloc_cmap(&cfb->fb.cmap, 8, 0);
		break;
	case 4:
		fb_alloc_cmap(&cfb->fb.cmap, 32, 0);
		break;
	case 8:
		
	default:
		fb_alloc_cmap(&cfb->fb.cmap, 256, 0);
		break;
	}

	/* Foreground 0 -- fb0 */
	strcpy(cfb->fb0.fix.id, "jzlcd-fg0");
	cfb->fb0.fix.type	= FB_TYPE_PACKED_PIXELS;	
	cfb->fb0.fix.type_aux	= 0;
	cfb->fb0.fix.xpanstep	= 1;
	cfb->fb0.fix.ypanstep	= 1;
	cfb->fb0.fix.ywrapstep	= 0;
	cfb->fb0.fix.accel	= FB_ACCEL_NONE;
	
	cfb->fb0.var.nonstd	= 0;
	cfb->fb0.var.activate	= FB_ACTIVATE_NOW;
	cfb->fb0.var.height	= -1;
	cfb->fb0.var.width	= -1;
	cfb->fb0.var.accel_flags	= FB_ACCELF_TEXT;
	
	cfb->fb0.fbops		= &jz4750fb0_ops;
	cfb->fb0.flags		= FBINFO_FLAG_DEFAULT;
		
	cfb->fb0.pseudo_palette	= (void *)(cfb + 1);
	
	switch (jz4750_lcd_info->osd.fg0.bpp) {
	case 1:
		fb_alloc_cmap(&cfb->fb0.cmap, 4, 0);
		break;
	case 2:
		fb_alloc_cmap(&cfb->fb0.cmap, 8, 0);
		break;
	case 4:
		fb_alloc_cmap(&cfb->fb0.cmap, 32, 0);
		break;
	case 8:
		
	default:
		fb_alloc_cmap(&cfb->fb0.cmap, 256, 0);
		break;
	}
	dprintk("fb_alloc_cmap, fb.cmap.len:%d, fb0.cmap.len:%d....\n", cfb->fb.cmap.len, cfb->fb0.cmap.len);
	return cfb;
}

/*
 * Map screen memory
 */
static int jz4750fb_map_smem(struct lcd_cfb_info *cfb)
{
	unsigned long page;
	unsigned int page_shift, needroom = 0, needroom1=0, bpp, w, h;
	unsigned char *fb_palette, *fb_frame;
	unsigned long pagemask = 0x3ff << 13;  /*4M */
	/* caculate the mem size of Foreground 0 */
	bpp = jz4750_lcd_info->osd.fg0.bpp;
	if (bpp == 18 || bpp == 24)
		bpp = 32;
	if (bpp == 15)
		bpp = 16;

	w = (jz4750_lcd_info->osd.fg0.w > TVE_WIDTH_PAL) ? jz4750_lcd_info->osd.fg0.w : TVE_WIDTH_PAL;
	h = (jz4750_lcd_info->osd.fg0.h > TVE_HEIGHT_PAL) ? jz4750_lcd_info->osd.fg0.h : TVE_HEIGHT_PAL;

	needroom1 = needroom = ((w * bpp + 7) >> 3) * h * ANDROID_NUMBER_OF_BUFFERS;
	/* end of alloc Foreground 0 mem */

	/* Caculate the mem size of Foreground 1 */
	bpp = jz4750_lcd_info->osd.fg1.bpp;
	if (bpp == 18 || bpp == 24)
		bpp = 32;
	if (bpp == 15)
		bpp = 16;

	/* The buffer size of FG1 should be large enough, so alloc memory depend of the fg0 args */
	w = (jz4750_lcd_info->osd.fg0.w > TVE_WIDTH_PAL) ? jz4750_lcd_info->osd.fg0.w : TVE_WIDTH_PAL;
	h = (jz4750_lcd_info->osd.fg0.h > TVE_HEIGHT_PAL) ? jz4750_lcd_info->osd.fg0.h : TVE_HEIGHT_PAL;
	needroom += ((w * bpp + 7) >> 3) * h;
	/* end of alloc Foreground 1 mem */

	/* Alloc memory */
	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;
	fb_palette = (unsigned char *)__get_free_pages(GFP_KERNEL, 0);
	fb_frame = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);
	if ((!fb_palette) || (!fb_frame))
		return -ENOMEM;
	memset((void *)fb_palette, 0, PAGE_SIZE);
	memset((void *)fb_frame, 0, PAGE_SIZE << page_shift);

	lcd_palette = fb_palette;
	dma_desc_base = (struct jz4750_lcd_dma_desc *)((void*)lcd_palette + ((PALETTE_SIZE+3)/4)*4);

	/*
	 * Set page reserved so that mmap will work. This is necessary
	 * since we'll be remapping normal memory.
	 */
	page = (unsigned long)lcd_palette;
	SetPageReserved(virt_to_page((void*)page));
	
	for (page = (unsigned long)fb_frame;
	     page < PAGE_ALIGN((unsigned long)fb_frame + (PAGE_SIZE<<page_shift));
	     page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

//	lcd_frame = fb_frame;
	lcd_frame = fb_frame + needroom1;
	cfb->fb.fix.smem_start = virt_to_phys((void *)lcd_frame);
	cfb->fb.fix.smem_len = needroom - needroom1; /* page_shift/2 ??? */
	cfb->fb.screen_base =
		(unsigned char *)(((unsigned int)lcd_frame&0x1fffffff) | 0xa0000000);
	if (!cfb->fb.screen_base) {
		printk("jz4750fb, %s: unable to map screen memory\n", cfb->fb.fix.id);
		return -ENOMEM;
	}

	lcd_frame0 = fb_frame;
	cfb->fb0.fix.smem_start = virt_to_phys((void *)lcd_frame0);
	cfb->fb0.fix.smem_len = needroom1; /* page_shift/2 ??? */
	cfb->fb0.screen_base =
		(unsigned char *)(((unsigned int)lcd_frame0&0x1fffffff) | 0xa0000000);
	if (!cfb->fb0.screen_base) {
		printk("jz4750fb0, %s: unable to map screen memory\n", cfb->fb0.fix.id);
		return -ENOMEM;
	}
	jz_lcd_add_wired_entry((unsigned long)cfb->fb0.fix.smem_start, 0, 0x50000000, pagemask);
	//jz_lcd_add_wired_entry((unsigned long)fb_frame, 0, 0x50000000, pagemask);
	return 0;
}

static void jz4750fb_free_fb_info(struct lcd_cfb_info *cfb)
{
	if (cfb) {
		fb_alloc_cmap(&cfb->fb.cmap, 0, 0);
		kfree(cfb);
	}
}

static void jz4750fb_unmap_smem(struct lcd_cfb_info *cfb)
{
	struct page * map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom, bpp, w, h;
	bpp = jz4750_lcd_info->osd.fg0.bpp;
	if ( bpp == 18 || bpp == 24)
		bpp = 32;
	if ( bpp == 15 )
		bpp = 16;
	w = jz4750_lcd_info->osd.fg0.w;
	h = jz4750_lcd_info->osd.fg0.h;
	needroom = ((w * bpp + 7) >> 3) * h;

	bpp = jz4750_lcd_info->osd.fg1.bpp;
	if ( bpp == 18 || bpp == 24)
		bpp = 32;
	if ( bpp == 15 )
		bpp = 16;
	w = jz4750_lcd_info->osd.fg1.w;
	h = jz4750_lcd_info->osd.fg1.h;
	needroom += ((w * bpp + 7) >> 3) * h;

	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	if (cfb && cfb->fb.screen_base) {
		iounmap(cfb->fb.screen_base);
		cfb->fb.screen_base = NULL;
		release_mem_region(cfb->fb.fix.smem_start,
				   cfb->fb.fix.smem_len);
	}

	if (lcd_palette) {
		map = virt_to_page(lcd_palette);
		clear_bit(PG_reserved, &map->flags);
		free_pages((int)lcd_palette, 0);
	}

	if (lcd_frame0) {
		for (tmp=(unsigned char *)lcd_frame0; 
		     tmp < lcd_frame0 + (PAGE_SIZE << page_shift); 
		     tmp += PAGE_SIZE) {
			map = virt_to_page(tmp);
			clear_bit(PG_reserved, &map->flags);
		}
		free_pages((int)lcd_frame0, page_shift);
	}
}

/************************************
 *      Jz475X Chipset OPS
 ************************************/

/* 
 * switch to tve mode from lcd mode
 * mode:
 * 	PANEL_MODE_TVE_PAL: switch to TVE_PAL mode
 * 	PANEL_MODE_TVE_NTSC: switch to TVE_NTSC mode
 */
static void print_lcdc_registers(void)	/* debug */
{
#ifdef  LCD_DEBUG
	/* LCD Controller Resgisters */
	printk("REG_LCD_CFG:\t0x%08x\n", REG_LCD_CFG);
	printk("REG_LCD_CTRL:\t0x%08x\n", REG_LCD_CTRL);
	printk("REG_LCD_STATE:\t0x%08x\n", REG_LCD_STATE);
	printk("REG_LCD_OSDC:\t0x%08x\n", REG_LCD_OSDC);
	printk("REG_LCD_OSDCTRL:\t0x%08x\n", REG_LCD_OSDCTRL);
	printk("REG_LCD_OSDS:\t0x%08x\n", REG_LCD_OSDS);
	printk("REG_LCD_BGC:\t0x%08x\n", REG_LCD_BGC);
	printk("REG_LCD_KEY0:\t0x%08x\n", REG_LCD_KEY0);
	printk("REG_LCD_KEY1:\t0x%08x\n", REG_LCD_KEY1);
	printk("REG_LCD_ALPHA:\t0x%08x\n", REG_LCD_ALPHA);
	printk("REG_LCD_IPUR:\t0x%08x\n", REG_LCD_IPUR);
	printk("REG_LCD_VAT:\t0x%08x\n", REG_LCD_VAT);
	printk("REG_LCD_DAH:\t0x%08x\n", REG_LCD_DAH);
	printk("REG_LCD_DAV:\t0x%08x\n", REG_LCD_DAV);
	printk("REG_LCD_XYP0:\t0x%08x\n", REG_LCD_XYP0);
	printk("REG_LCD_XYP1:\t0x%08x\n", REG_LCD_XYP1);
	printk("REG_LCD_SIZE0:\t0x%08x\n", REG_LCD_SIZE0);
	printk("REG_LCD_SIZE1:\t0x%08x\n", REG_LCD_SIZE1);
	printk("REG_LCD_RGBC\t0x%08x\n", REG_LCD_RGBC);
	printk("REG_LCD_VSYNC:\t0x%08x\n", REG_LCD_VSYNC);
	printk("REG_LCD_HSYNC:\t0x%08x\n", REG_LCD_HSYNC);
	printk("REG_LCD_PS:\t0x%08x\n", REG_LCD_PS);
	printk("REG_LCD_CLS:\t0x%08x\n", REG_LCD_CLS);
	printk("REG_LCD_SPL:\t0x%08x\n", REG_LCD_SPL);
	printk("REG_LCD_REV:\t0x%08x\n", REG_LCD_REV);
	printk("REG_LCD_IID:\t0x%08x\n", REG_LCD_IID);
	printk("REG_LCD_DA0:\t0x%08x\n", REG_LCD_DA0);
	printk("REG_LCD_SA0:\t0x%08x\n", REG_LCD_SA0);
	printk("REG_LCD_FID0:\t0x%08x\n", REG_LCD_FID0);
	printk("REG_LCD_CMD0:\t0x%08x\n", REG_LCD_CMD0);
	printk("REG_LCD_OFFS0:\t0x%08x\n", REG_LCD_OFFS0);
	printk("REG_LCD_PW0:\t0x%08x\n", REG_LCD_PW0);
	printk("REG_LCD_CNUM0:\t0x%08x\n", REG_LCD_CNUM0);
	printk("REG_LCD_DESSIZE0:\t0x%08x\n", REG_LCD_DESSIZE0);
	printk("REG_LCD_DA1:\t0x%08x\n", REG_LCD_DA1);
	printk("REG_LCD_SA1:\t0x%08x\n", REG_LCD_SA1);
	printk("REG_LCD_FID1:\t0x%08x\n", REG_LCD_FID1);
	printk("REG_LCD_CMD1:\t0x%08x\n", REG_LCD_CMD1);
	printk("REG_LCD_OFFS1:\t0x%08x\n", REG_LCD_OFFS1);
	printk("REG_LCD_PW1:\t0x%08x\n", REG_LCD_PW1);
	printk("REG_LCD_CNUM1:\t0x%08x\n", REG_LCD_CNUM1);
	printk("REG_LCD_DESSIZE1:\t0x%08x\n", REG_LCD_DESSIZE1);
	printk("==================================\n");
	printk("REG_LCD_VSYNC:\t%d:%d\n", REG_LCD_VSYNC>>16, REG_LCD_VSYNC&0xfff);
	printk("REG_LCD_HSYNC:\t%d:%d\n", REG_LCD_HSYNC>>16, REG_LCD_HSYNC&0xfff);
	printk("REG_LCD_VAT:\t%d:%d\n", REG_LCD_VAT>>16, REG_LCD_VAT&0xfff);
	printk("REG_LCD_DAH:\t%d:%d\n", REG_LCD_DAH>>16, REG_LCD_DAH&0xfff);
	printk("REG_LCD_DAV:\t%d:%d\n", REG_LCD_DAV>>16, REG_LCD_DAV&0xfff);
	printk("==================================\n");

	/* Smart LCD Controller Resgisters */
	printk("REG_SLCD_CFG:\t0x%08x\n", REG_SLCD_CFG);
	printk("REG_SLCD_CTRL:\t0x%08x\n", REG_SLCD_CTRL);
	printk("REG_SLCD_STATE:\t0x%08x\n", REG_SLCD_STATE);
	printk("==================================\n");

	/* TVE Controller Resgisters */
	printk("REG_TVE_CTRL:\t0x%08x\n", REG_TVE_CTRL);
	printk("REG_TVE_FRCFG:\t0x%08x\n", REG_TVE_FRCFG);
	printk("REG_TVE_SLCFG1:\t0x%08x\n", REG_TVE_SLCFG1);
	printk("REG_TVE_SLCFG2:\t0x%08x\n", REG_TVE_SLCFG2);
	printk("REG_TVE_SLCFG3:\t0x%08x\n", REG_TVE_SLCFG3);
	printk("REG_TVE_LTCFG1:\t0x%08x\n", REG_TVE_LTCFG1);
	printk("REG_TVE_LTCFG2:\t0x%08x\n", REG_TVE_LTCFG2);
	printk("REG_TVE_CFREQ:\t0x%08x\n", REG_TVE_CFREQ);
	printk("REG_TVE_CPHASE:\t0x%08x\n", REG_TVE_CPHASE);
	printk("REG_TVE_CBCRCFG:\t0x%08x\n", REG_TVE_CBCRCFG);
	printk("REG_TVE_WSSCR:\t0x%08x\n", REG_TVE_WSSCR);
	printk("REG_TVE_WSSCFG1:\t0x%08x\n", REG_TVE_WSSCFG1);
	printk("REG_TVE_WSSCFG2:\t0x%08x\n", REG_TVE_WSSCFG2);
	printk("REG_TVE_WSSCFG3:\t0x%08x\n", REG_TVE_WSSCFG3);

	printk("==================================\n");

	if ( 1 ) {
		unsigned int * pii = (unsigned int *)dma_desc_base;
		int i, j;
		for (j=0;j< DMA_DESC_NUM ; j++) {
			printk("dma_desc%d(0x%08x):\n", j, (unsigned int)pii);
			for (i =0; i<8; i++ ) {
				printk("\t\t0x%08x\n", *pii++);
			}
		}
	}
#endif
}

static void jz4750lcd_info_switch_to_TVE(int mode)
{
	struct jz4750lcd_info *info;
	struct jz4750lcd_osd_t *osd_lcd;
	int x, y, w, h;

	info = jz4750_lcd_info = &jz4750_info_tve;
	osd_lcd = &jz4750_lcd_panel.osd;

	switch ( mode ) {
	case PANEL_MODE_TVE_PAL:
		info->panel.cfg |= LCD_CFG_TVEPEH; /* TVE PAL enable extra halfline signal */
		info->panel.w = TVE_WIDTH_PAL;
		info->panel.h = TVE_HEIGHT_PAL;
		info->panel.fclk = TVE_FREQ_PAL;

		w = TVE_WIDTH_PAL - 16;
		h = TVE_HEIGHT_PAL - 20;
		x = (TVE_WIDTH_PAL - w) / 2;
		y = (TVE_HEIGHT_PAL - h ) / 2;

		info->osd.fg0.bpp = osd_lcd->fg0.bpp;
		info->osd.fg0.x = x;
		info->osd.fg0.y = y;
		info->osd.fg0.w = w;
		info->osd.fg0.h = h;

		w = TVE_WIDTH_PAL - 16;
		h = TVE_HEIGHT_PAL - 20;
		x = (TVE_WIDTH_PAL-w) / 2;
		y = (TVE_HEIGHT_PAL-h)/ 2;

		info->osd.fg1.bpp = 16;	/* use RGB888 in TVE mode*/
		info->osd.fg1.x = x;
		info->osd.fg1.y = y;
		info->osd.fg1.w = w;
		info->osd.fg1.h = h;
		break;
	case PANEL_MODE_TVE_NTSC:
		info->panel.cfg &= ~LCD_CFG_TVEPEH; /* TVE NTSC disable extra halfline signal */
		info->panel.w = TVE_WIDTH_NTSC;
		info->panel.h = TVE_HEIGHT_NTSC;
		info->panel.fclk = TVE_FREQ_NTSC;

		w = TVE_WIDTH_NTSC - 16;
		h = TVE_HEIGHT_NTSC - 12;
		x = (TVE_WIDTH_NTSC - w) / 2;
		y = (TVE_HEIGHT_NTSC - h) / 2;
		info->osd.fg0.bpp = osd_lcd->fg0.bpp;
		info->osd.fg0.x = x;
		info->osd.fg0.y = y;
		info->osd.fg0.w = w;
		info->osd.fg0.h = h;

		w = TVE_WIDTH_NTSC - 16;
		h = TVE_HEIGHT_NTSC - 12;
		x = (TVE_WIDTH_NTSC - w) / 2;
		y = (TVE_HEIGHT_NTSC - h) / 2;
		info->osd.fg1.bpp = 32;	/* use RGB888 int TVE mode */
		info->osd.fg1.x = x;
		info->osd.fg1.y = y;
		info->osd.fg1.w = w;
		info->osd.fg1.h = h;
		break;
	default:
		printk("%s, %s: Unknown tve mode\n", __FILE__, __FUNCTION__);
		break;
	}
}

/* 
 * switch to lcd mode from TVE mode
 */

static void jz4750lcd_info_switch_to_lcd(void)
{
	struct jz4750lcd_info *info;
	struct jz4750lcd_osd_t *osd_lcd;
	struct jz4750lcd_panel_t *panel_lcd;
	int x, y, w, h;

	info = &jz4750_lcd_panel;

	/* set to tve mode */
	info->panel.cfg	= jz4750_info_tve.panel.cfg;
	info->panel.cfg &= ~(LCD_CFG_TVEN | LCD_CFG_MODE_INTER_CCIR656); /* Interlace CCIR656 mode */
	info->panel.ctrl = jz4750_info_tve.panel.ctrl;
	info->osd.rgb_ctrl &= ~LCD_RGBC_YCC; /* enable YUV => RGB*/

	/*  */
	osd_lcd = &jz4750_info_tve.osd;
	panel_lcd = &jz4750_info_tve.panel;

	/* set Foreground 0 */
	w = osd_lcd->fg0.w / panel_lcd->w * info->panel.w;
	h = osd_lcd->fg0.h / panel_lcd->h * info->panel.h;
	x = osd_lcd->fg0.x / panel_lcd->w * info->panel.w;
	y = osd_lcd->fg0.y / panel_lcd->h * info->panel.h;
	info->osd.fg0.x = x;
	info->osd.fg0.y = y;
	info->osd.fg0.w = w;
	info->osd.fg0.h = h;

	/* set Foreground 1 */
	w = osd_lcd->fg1.w / panel_lcd->w * info->panel.w;
	h = osd_lcd->fg1.h / panel_lcd->h * info->panel.h;
	x = osd_lcd->fg1.x / panel_lcd->w * info->panel.w;
	y = osd_lcd->fg1.y / panel_lcd->h * info->panel.h;
	info->osd.fg1.x = x;
	info->osd.fg1.y = y;
	info->osd.fg1.w = w;
	info->osd.fg1.h = h;

	jz4750_lcd_info = &jz4750_lcd_panel;
}

/* initial dma descriptors */
static void jz4750fb_descriptor_init( struct jz4750lcd_info * lcd_info )
{
	unsigned int pal_size;
	int fg0_line_size, fg0_frm_size, fg1_line_size, fg1_frm_size;
	int buffer_line_size, buffer_frm_size;
	int panel_line_size, panel_frm_size;
	int size0, size1;


	switch ( lcd_info->osd.fg0.bpp ) {
	case 1:
		pal_size = 4;
		break;
	case 2:
		pal_size = 8;
		break;
	case 4:
		pal_size = 32;
		break;
	case 8:
	default:
		pal_size = 512;
	}

	pal_size /= 4;

	/*
	 * Normal TFT panel's DMA Chan0: 
	 *	TO LCD Panel: 	
	 * 		no palette:	dma0_desc0 <<==>> dma0_desc0 
	 * 		palette :	dma0_desc_palette <<==>> dma0_desc0
	 *	TO TV Encoder:
	 * 		no palette:	dma0_desc0 <<==>> dma0_desc1
	 * 		palette:	dma0_desc_palette --> dma0_desc0 
	 * 				--> dma0_desc1 --> dma0_desc_palette --> ...
	 * DMA Chan1:
	 *	TO LCD Panel:
	 * 		dma1_desc0 <<==>> dma1_desc0 
	 *	TO TV Encoder:
	 * 		dma1_desc0 <<==>> dma1_desc1
	 */

	dma0_desc_palette 	= dma_desc_base + 0;
	dma0_desc0 		= dma_desc_base + 1;
	dma0_desc1 		= dma_desc_base + 2;
	dma0_desc0_change	= dma_desc_base + 3;
	dma0_desc1_change	= dma_desc_base + 4;

	/* Foreground 0, caculate size */
	if ( lcd_info->osd.fg0.x >= lcd_info->panel.w )
		lcd_info->osd.fg0.x = lcd_info->panel.w - 1;
	if ( lcd_info->osd.fg0.y >= lcd_info->panel.h )
		lcd_info->osd.fg0.y = lcd_info->panel.h - 1;

	if (lcd_info->panel.cfg & LCD_CFG_TVEN  ) {
		//	if ( lcd_info->osd.fg0.x + lcd_info->osd.fg0.w > lcd_info->panel.w ) 
		lcd_info->osd.fg0.w = lcd_info->panel.w - 2 * lcd_info->osd.fg0.x;
			//	if ( lcd_info->osd.fg0.y + lcd_info->osd.fg0.h > lcd_info->panel.h ) 
		lcd_info->osd.fg0.h = lcd_info->panel.h - 2 * lcd_info->osd.fg0.y;
	}
/*	else{
		if ( lcd_info->osd.fg0.x + lcd_info->osd.fg0.w > lcd_info->panel.w ) 
			lcd_info->osd.fg0.w = lcd_info->panel.w - 2*lcd_info->osd.fg0.x;
		if ( lcd_info->osd.fg0.y + lcd_info->osd.fg0.h > lcd_info->panel.h ) 
			lcd_info->osd.fg0.h = lcd_info->panel.h - 2*lcd_info->osd.fg0.y;
       }
*/	
	size0 = lcd_info->osd.fg0.h << 16 | lcd_info->osd.fg0.w;
	//size0 = lcd_info->panel.h << 16 | lcd_info->panel.w;
	/* lcd display area */
	panel_line_size = (lcd_info->panel.w - 2 * lcd_info->osd.fg0.x) * lcd_info->osd.fg0.bpp / 8;
	panel_line_size = ((panel_line_size + 3) >> 2) << 2; /* word aligned */
	panel_frm_size= panel_line_size * (lcd_info->panel.h - 2 * lcd_info->osd.fg0.y);//lcd_info->panel.h;//

	/* total fg0 buffer area */
	fg0_line_size = (lcd_info->osd.fg0.w * (lcd_info->osd.fg0.bpp) / 8);
	fg0_line_size = ((fg0_line_size + 3) >> 2) << 2; /* word aligned */
	fg0_frm_size = fg0_line_size * lcd_info->osd.fg0.h;

#if 1
	/*total buffer size*/
	buffer_line_size = jz4750_lcd_panel.osd.fg0.w * lcd_info->osd.fg0.bpp / 8;
	buffer_line_size = ((buffer_line_size + 3) >> 2) << 2; /* word aligned */
	buffer_frm_size= buffer_line_size * jz4750_lcd_panel.panel.h;
#endif
	/* Palette Descriptor */
	dma0_desc_palette->next_desc 	= (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_palette->databuf 	= (unsigned int)virt_to_phys((void *)lcd_palette);
	dma0_desc_palette->frame_id 	= (unsigned int)0xaaaaaaaa;
	dma0_desc_palette->cmd 		= LCD_CMD_PAL | pal_size; /* Palette Descriptor */
	/* DMA0 Descriptor */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* TVE mode */
		/* Next */
		dma0_desc0->next_desc 	= (unsigned int)virt_to_phys(dma0_desc1);
		if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup */
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_palette);
		else
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
		dma0_desc0_change->next_desc = (unsigned int)virt_to_phys(dma0_desc1_change);
		dma0_desc1_change->next_desc = (unsigned int)virt_to_phys(dma0_desc0_change);

		/* frame phys addr */
		dma0_desc0->databuf = dma0_desc0_change->databuf = virt_to_phys((void *)lcd_frame0);
		dma0_desc1->databuf = virt_to_phys((void *)(lcd_frame0 + fg0_line_size));

		/* frame id */
		dma0_desc0->frame_id = (unsigned int)0x0da00000; /* DMA0'0 */
		dma0_desc1->frame_id = (unsigned int)0x0da00001; /* DMA0'1 */
		dma0_desc0_change->frame_id = (unsigned int)0x0da000c0; /* DMA0'2 */
		dma0_desc1_change->frame_id = (unsigned int)0x0da000c1;

		/* others */
		//dma0_desc0->cmd = dma0_desc1->cmd = LCD_CMD_EOFINT | ((panel_frm_size)/4)/2;
		dma0_desc0->cmd = LCD_CMD_EOFINT | ((panel_frm_size+panel_line_size)/4)/2;
		dma0_desc1->cmd = LCD_CMD_EOFINT | ((panel_frm_size-panel_line_size)/4)/2;
		dma0_desc0->offsize = dma0_desc1->offsize =(buffer_line_size + buffer_line_size - panel_line_size)/4;
		dma0_desc0->page_width = dma0_desc1->page_width = panel_line_size/4;
		dma0_desc0->desc_size = dma0_desc1->desc_size = size0;

	}
	else {			/* Normal TFT LCD */
		/* next */
		dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
		dma0_desc0_change->next_desc = (unsigned int)virt_to_phys(dma0_desc0_change);

		/* frame phys addr */
		dma0_desc0->databuf = dma0_desc0_change->databuf = virt_to_phys((void *)lcd_frame0);

		/* frame id */
		dma0_desc0->frame_id = (unsigned int)0x0000da00; /* DMA0'0 */
		dma0_desc0_change->frame_id = (unsigned int)0x0da000c0; /* DMA0'2 */

		/* others */
		dma0_desc0->cmd = LCD_CMD_EOFINT | panel_frm_size/4;
//		dma0_desc0->cmd = panel_frm_size/4;
		dma0_desc0->offsize = (fg0_line_size - panel_line_size)/4;
		dma0_desc0->page_width = panel_line_size/4;

		dma0_desc0->desc_size = size0;
	}

	if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup */
		REG_LCD_DA0 = virt_to_phys(dma0_desc_palette);
	else
		REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
	REG_LCD_SIZE0 = size0;
	current_dma0_id = 0;//dma0_desc0;

	dma1_desc0 		= dma_desc_base + 5;
	dma1_desc1 		= dma_desc_base + 6;
	dma1_desc0_change	= dma_desc_base + 7;
	dma1_desc1_change	= dma_desc_base + 8;

#if 1	/* Foreground 1, caculate size */
	if ( lcd_info->osd.fg1.x >= lcd_info->panel.w )
		lcd_info->osd.fg1.x = lcd_info->panel.w - 1;
	if ( lcd_info->osd.fg1.y >= lcd_info->panel.h )
		lcd_info->osd.fg1.y = lcd_info->panel.h - 1;

	if (lcd_info->panel.cfg & LCD_CFG_TVEN  ) {
		if ( lcd_info->osd.fg1.x + lcd_info->osd.fg1.w > lcd_info->panel.w ) 
			lcd_info->osd.fg1.w = lcd_info->panel.w - 2 *lcd_info->osd.fg1.x;
		if ( lcd_info->osd.fg1.y + lcd_info->osd.fg1.h > lcd_info->panel.h ) 
			lcd_info->osd.fg1.h = lcd_info->panel.h - 2 * lcd_info->osd.fg1.y;
	}
/*	else{
	       	if ( lcd_info->osd.fg1.x + lcd_info->osd.fg1.w > lcd_info->panel.w ) 
       			lcd_info->osd.fg1.w = lcd_info->panel.w - 2*lcd_info->osd.fg1.x;
	       	if ( lcd_info->osd.fg1.y + lcd_info->osd.fg1.h > lcd_info->panel.h ) 
			lcd_info->osd.fg1.h = lcd_info->panel.h - 2*lcd_info->osd.fg1.y;
       }
*/	
#endif
	size1 = lcd_info->osd.fg1.h << 16 | lcd_info->osd.fg1.w;
	/* lcd panel display area */
	panel_line_size = lcd_info->panel.w * lcd_info->osd.fg1.bpp / 8;
	panel_line_size = ((panel_line_size + 3) >> 2) << 2; /* word aligned */
	panel_frm_size= panel_line_size * lcd_info->panel.h;
#if 0
	/*total buffer size*/
	buffer_line_size =  TVE_WIDTH_PAL * lcd_info->osd.fg1.bpp / 8;
	buffer_line_size = ((buffer_line_size + 3) >> 2) << 2; /* word aligned */
	buffer_frm_size= buffer_line_size * TVE_HEIGHT_PAL;
#endif
	/* lcd display area */
	fg1_line_size = lcd_info->osd.fg1.w*lcd_info->osd.fg1.bpp/8;
	fg1_line_size = ((fg1_line_size+3)>>2)<<2; /* word aligned */
	fg1_frm_size = fg1_line_size * lcd_info->osd.fg1.h;

	/* DMA1 Descriptor */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) {/* TVE mode */
		/* Next */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc1);
		dma1_desc1->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
		dma1_desc0_change->next_desc = (unsigned int)virt_to_phys(dma1_desc1_change);
		dma1_desc1_change->next_desc = (unsigned int)virt_to_phys(dma1_desc0_change);

		/* frame phys addr */
		dma1_desc0->databuf = dma1_desc0_change->databuf = virt_to_phys((void *)lcd_frame);
		dma1_desc1->databuf = virt_to_phys((void *)(lcd_frame + fg1_line_size));

		/* frame id */
		dma1_desc0->frame_id = (unsigned int)0x0da10000; /* DMA1'0 */
		dma1_desc1->frame_id = (unsigned int)0x0da10001; /* DMA1'1 */
		dma1_desc0_change->frame_id = (unsigned int)0x0da100c0; /* DMA1'C0 */
		dma1_desc1_change->frame_id = (unsigned int)0x0da100c1; /* DMA1'C1 */

		/* other*/
		if(jz4750_lcd_info->osd.fg1.h % 2 == 0){
			dma1_desc0->cmd = LCD_CMD_EOFINT | ((fg1_frm_size)/4)/2;
			dma1_desc1->cmd = LCD_CMD_EOFINT | ((fg1_frm_size)/4)/2;
		}
		else{
			dma1_desc0->cmd = LCD_CMD_EOFINT | ((fg1_frm_size + fg1_line_size)/4)/2;
			dma1_desc1->cmd = LCD_CMD_EOFINT | ((fg1_frm_size - fg1_line_size)/4)/2;
		}
			dma1_desc0->offsize = dma1_desc1->offsize = fg1_line_size/4;
			dma1_desc0->page_width = dma1_desc1->page_width = fg1_line_size/4;
					
		dma1_desc0->desc_size = dma1_desc1->desc_size = size1;
	}
	else {			/* Normal TFT LCD */
		/* Next */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
		dma1_desc0_change->next_desc = (unsigned int)virt_to_phys(dma1_desc0_change);

		/* frame phys addr */
		dma1_desc0->databuf = dma1_desc0_change->databuf = virt_to_phys((void *)lcd_frame);

		/* frame id */
		dma1_desc0->frame_id = (unsigned int)0x0da10000; /* DMA1'0 */
		dma1_desc0_change->frame_id = (unsigned int)0x0da100c0; /* DMA1'C0 */

		/* other */
		if (panel_line_size >= fg1_line_size){
			dma1_desc0->cmd = LCD_CMD_EOFINT | fg1_frm_size /4;
			dma1_desc0->offsize = 0;//(buffer_line_size - fg1_line_size )/4;
			dma1_desc0->page_width = 0;//fg1_line_size /4;
		}	
		else{
			dma1_desc0->cmd = LCD_CMD_EOFINT | panel_frm_size/4;
			dma1_desc0->offsize = 0;//(fg1_line_size - panel_line_size)/4;
			dma1_desc0->page_width = 0;//panel_line_size/4;
		}
		dma1_desc0->desc_size = size1;
	}

	REG_LCD_DA1 = virt_to_phys(dma1_desc0);	/* set Dma-chan1's Descripter Addrress */
	REG_LCD_SIZE1 = size1;
	current_dma1_id = 0;//dma1_desc0;

	dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));
}

static void jz4750fb_set_panel_mode(struct jz4750lcd_info * lcd_info)
{
	struct jz4750lcd_panel_t *panel = &lcd_info->panel;

	/* set bpp */
	lcd_info->panel.ctrl &= ~LCD_CTRL_BPP_MASK;
	if ( lcd_info->osd.fg0.bpp == 1 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_1;
	else if ( lcd_info->osd.fg0.bpp == 2 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_2;
	else if ( lcd_info->osd.fg0.bpp == 4 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_4;
	else if ( lcd_info->osd.fg0.bpp == 8 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_8;
	else if ( lcd_info->osd.fg0.bpp == 15 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB555;
	else if ( lcd_info->osd.fg0.bpp == 16 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB565;
	else if ( lcd_info->osd.fg0.bpp > 16 && lcd_info->osd.fg0.bpp < 32+1 ) {
		lcd_info->osd.fg0.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	}
	else {
		printk("The BPP %d is not supported\n", lcd_info->osd.fg0.bpp);
		lcd_info->osd.fg0.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	}

	lcd_info->panel.cfg |= LCD_CFG_NEWDES; /* use 8words descriptor always */
	REG_LCD_CTRL = lcd_info->panel.ctrl; /* LCDC Controll Register */

	REG_LCD_CFG = lcd_info->panel.cfg; /* LCDC Configure Register */

	switch ( lcd_info->panel.cfg & LCD_CFG_MODE_MASK ) {
	case LCD_CFG_MODE_GENERIC_TFT:
	case LCD_CFG_MODE_INTER_CCIR656:
	case LCD_CFG_MODE_NONINTER_CCIR656:
	case LCD_CFG_MODE_SLCD:
	default:		/* only support TFT16 TFT32, not support STN and Special TFT by now(10-06-2008)*/
		REG_LCD_VAT = (((panel->blw + panel->w + panel->elw + panel->hsw)) << 16) | (panel->vsw + panel->bfw + panel->h + panel->efw);
		REG_LCD_DAH = ((panel->hsw + panel->blw) << 16) | (panel->hsw + panel->blw + panel->w);
		REG_LCD_DAV = ((panel->vsw + panel->bfw) << 16) | (panel->vsw + panel->bfw + panel->h);
		REG_LCD_HSYNC = (0 << 16) | panel->hsw;
		REG_LCD_VSYNC = (0 << 16) | panel->vsw;
		break;
	}
}

static void jz4750fb_set_osd_mode(struct jz4750lcd_osd_t *lcd_osd_info)
{
	lcd_osd_info->osd_ctrl &= ~(LCD_OSDCTRL_OSDBPP_MASK);
	if ( lcd_osd_info->fg1.bpp == 15 )
		lcd_osd_info->osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16|LCD_OSDCTRL_RGB555;
	else if ( lcd_osd_info->fg1.bpp == 16 )
		lcd_osd_info->osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16|LCD_OSDCTRL_RGB565;
	else {
		lcd_osd_info->fg1.bpp = 32;
		lcd_osd_info->osd_ctrl |= LCD_OSDCTRL_OSDBPP_18_24;
	}

	REG_LCD_OSDC 	= lcd_osd_info->osd_cfg; /* F0, F1, alpha, */

	REG_LCD_OSDCTRL = lcd_osd_info->osd_ctrl; /* IPUEN, bpp */
	REG_LCD_RGBC  	= lcd_osd_info->rgb_ctrl;
	REG_LCD_BGC  	= lcd_osd_info->bgcolor;
	REG_LCD_KEY0 	= lcd_osd_info->colorkey0;
	REG_LCD_KEY1 	= lcd_osd_info->colorkey1;
	REG_LCD_ALPHA 	= lcd_osd_info->alpha;
	REG_LCD_IPUR 	= lcd_osd_info->ipu_restart;
	REG_LCD_XYP0 	= lcd_osd_info->fg0.y << 16 | lcd_osd_info->fg0.x ;
	REG_LCD_XYP1 	= lcd_osd_info->fg1.y << 16 | lcd_osd_info->fg1.x;
}


/* Change Position of Foreground 0 */
static int jz4750fb0_foreground_move(struct jz4750lcd_osd_t *lcd_osd_info)
{
	int pos;
#if defined(CONFIG_SOC_JZ4750D)
	int j, count = 100000;
#endif
	/* 
	 * Foreground, only one of the following can be change at one time:
	 * 	1. F0 size, 2. F0 position, 3. F1 size, 4. F1 position
	 *
	 * The rules of fg0 position:
	 * 	fg0.x + fg0.w <= panel.w;
	 * 	fg0.y + fg0.h <= panel.h;
	 *
	 * When output is LCD panel, fg.y can be odd number or even number.
	 * When output is TVE, as the TVE has odd frame and even frame,
	 * to simplified operation, fg.y should be even number always.
	 *
	 */

	/* Foreground 0  */
	if (lcd_osd_info->fg0.x + lcd_osd_info->fg0.w > jz4750_lcd_info->panel.w)
		lcd_osd_info->fg0.x = jz4750_lcd_info->panel.w - lcd_osd_info->fg0.w;
	if (lcd_osd_info->fg0.y + lcd_osd_info->fg0.h > jz4750_lcd_info->panel.h)
		lcd_osd_info->fg0.y = jz4750_lcd_info->panel.h - lcd_osd_info->fg0.h;

	if (lcd_osd_info->fg0.x >= jz4750_lcd_info->panel.w)
		lcd_osd_info->fg0.x = jz4750_lcd_info->panel.w - 1;
	if (lcd_osd_info->fg0.y >= jz4750_lcd_info->panel.h)
		lcd_osd_info->fg0.y = jz4750_lcd_info->panel.h - 1;

	pos = lcd_osd_info->fg0.y << 16 | lcd_osd_info->fg0.x;
	if (REG_LCD_XYP0 == pos){
		printk("FG0: same position\n");
		return 0;
	}

#if defined(CONFIG_SOC_JZ4750D)
	REG_LCD_XYP0 = pos;
	REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
	while(!(REG_LCD_OSDS & LCD_OSDS_READY));
	j = count;
	msleep(40);
	while((REG_LCD_OSDCTRL & LCD_OSDCTRL_CHANGES) && j--);
	if(j == 0) {
		printk("Error FG0 Position: Wait change fail.\n");
		return -EFAULT;
	}

#else
	/****jz4750****/
//	REG_LCD_OSDC &= ~LCD_OSDC_F0EN;
	REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
	REG_LCD_XYP0 = pos;
//	REG_LCD_OSDC |= LCD_OSDC_F0EN;
	/*********************************************/

#endif
	return 0;
}
/* Change Window size of Foreground 0 */
static int jz4750fb0_foreground_resize(struct jz4750lcd_osd_t *lcd_osd_info)
{
	struct lcd_cfb_info *cfb = jz4750fb_info;
	int size, fg0_line_size, fg0_frm_size;
//	int desc_len = sizeof(struct jz4750_lcd_dma_desc);
	/* 
	 * NOTE: 
	 * Foreground change sequence: 
	 * 	1. Change Position Registers -> LCD_OSDCTL.Change;
	 * 	2. LCD_OSDCTRL.Change -> descripter->Size
	 * Foreground, only one of the following can be change at one time:
	 * 	1. F0 size;
	 *	2. F0 position
	 * 	3. F1 size
	 *	4. F1 position
	 */
	
	/* 
	 * The rules of f0, f1's position:
	 * 	f0.x + f0.w <= panel.w;
	 * 	f0.y + f0.h <= panel.h;
	 *
	 * When output is LCD panel, fg.y and fg.h can be odd number or even number.
	 * When output is TVE, as the TVE has odd frame and even frame,
	 * to simplified operation, fg.y and fg.h should be even number always.
	 *
	 */
	/* Foreground 0  */
	if (lcd_osd_info->fg0.x + lcd_osd_info->fg0.w > jz4750_lcd_info->panel.w)
		lcd_osd_info->fg0.w = jz4750_lcd_info->panel.w - lcd_osd_info->fg0.x;
	if (lcd_osd_info->fg0.y + lcd_osd_info->fg0.h > jz4750_lcd_info->panel.h)
		lcd_osd_info->fg0.h = jz4750_lcd_info->panel.h - lcd_osd_info->fg0.y;

	size = lcd_osd_info->fg0.h << 16 | lcd_osd_info->fg0.w;

	if (REG_LCD_SIZE0 == size) {
		printk("FG0: same size\n");
		return 0;
	}

	fg0_line_size = lcd_osd_info->fg0.w * lcd_osd_info->fg0.bpp / 8;
	fg0_line_size = ((fg0_line_size + 3) >> 2) << 2; /* word aligned */
	fg0_frm_size = fg0_line_size * lcd_osd_info->fg0.h;

	REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
#if 0 // For 4750d
	if (current_dma0_id == 0) {
		printk("Change to dma0_desc0_change\n");
//		REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
		if (jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) {
			dma0_desc0_change->cmd = dma0_desc1_change->cmd = (fg0_frm_size/4)/2;
			dma0_desc0_change->offsize = dma0_desc1_change->offsize 
				= fg0_line_size/4;
			dma0_desc0_change->page_width = dma0_desc1_change->page_width 
				= fg0_line_size/4;
			dma0_desc1_change->databuf = virt_to_phys((void *)(lcd_frame0 + fg0_line_size));
			dma0_desc0_change->desc_size = dma0_desc1->desc_size = size;
			dma_cache_wback_inv((unsigned int)(dma0_desc0_change), desc_len);
			dma_cache_wback_inv((unsigned int)(dma0_desc1_change), desc_len);
		}
		else {
			dma0_desc0_change->cmd = fg0_frm_size/4;
			dma0_desc0_change->offsize = 0;
			dma0_desc0_change->page_width = 0;
			dma0_desc0_change->desc_size = size;
			dma0_desc0_change->next_desc = (unsigned int)virt_to_phys(dma0_desc0_change);
			dma_cache_wback_inv((unsigned int)(dma0_desc0_change),desc_len);
		}
		REG_LCD_SIZE0 = size;	
		if (jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) {
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc0_change);
			dma_cache_wback_inv((unsigned int)(dma0_desc1_change), desc_len);
		}
		else {
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc0_change);
			dma_cache_wback_inv((unsigned int)(dma0_desc0_change), desc_len);
		}
		current_dma0_id = 1;//dma0_desc0_change;
	}
	else {
		printk("Change to dma0_desc0\n");
		if (jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) {
			dma0_desc0->cmd = dma0_desc1->cmd = (fg0_frm_size/4)/2;
			dma0_desc0->offsize = dma0_desc1->offsize 
				= fg0_line_size/4;
			dma0_desc0->page_width = dma0_desc1->page_width 
				= fg0_line_size/4;
			dma0_desc1->databuf = virt_to_phys((void *)(lcd_frame0 + fg0_line_size));
			dma0_desc0->desc_size = dma0_desc1->desc_size = size;
		}
		else {
			dma0_desc0->cmd = fg0_frm_size/4;
			dma0_desc0->offsize =0;
			dma0_desc0->page_width = 0;
			dma0_desc0->desc_size = size;
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
			dma_cache_wback_inv((unsigned int)(dma0_desc0), desc_len);
		}
		REG_LCD_SIZE0 = size;
		if (jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) {
			dma0_desc1_change->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
			dma_cache_wback_inv((unsigned int)(dma0_desc1_change), desc_len);
		}
		else {
			dma0_desc0_change->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
			dma_cache_wback_inv((unsigned int)(dma0_desc0_change), desc_len);
		}
		current_dma0_id = 0;//dma0_desc0;
	}

#else
	/* set change bit */
	REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;

	if (jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* output to TV */
		dma0_desc0->cmd = dma0_desc1->cmd = LCD_CMD_EOFINT | (fg0_frm_size/4)/2;
		dma0_desc0->offsize = dma0_desc1->offsize 
			= fg0_line_size/4;
		dma0_desc0->page_width = dma0_desc1->page_width 
			= fg0_line_size/4;
		dma0_desc1->databuf = virt_to_phys((void *)(lcd_frame0 + fg0_line_size));
	}
	else {
		dma0_desc0->cmd = dma0_desc1->cmd = LCD_CMD_EOFINT | fg0_frm_size/4;
		dma0_desc0->offsize = dma0_desc1->offsize =0;
		dma0_desc0->page_width = dma0_desc1->page_width = 0;
		}
	
	dma0_desc0->desc_size = dma0_desc1->desc_size = size;
//			= lcd_osd_info->fg0.h << 16 | lcd_osd_info->fg0.w;
	REG_LCD_SIZE0 = size;
//	REG_LCD_SIZE0 = (lcd_osd_info->fg0.h << 16) | lcd_osd_info->fg0.w;

	dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));
#endif
	jz4750fb_set_var(&cfb->fb0.var, 0, &cfb->fb0);
	return 0;
}


/* Change Position of Foreground 1 */
static int jz4750fb_foreground_move(struct jz4750lcd_osd_t *lcd_osd_info)
{
	int pos;
#if defined(CONFIG_SOC_JZ4750D)
	int j, count = 100000;
#endif
	/* 
	 * Foreground, only one of the following can be change at one time:
	 * 	1. F0 size, 2. F0 position, 3. F1 size, 4. F1 position
	 *
	 * The rules of fg1 position:
	 * 	fg1.x + fg1.w <= panel.w;
	 * 	fg1.y + fg1.h <= panel.h;
	 *
	 * When output is LCD panel, fg.y can be odd number or even number.
	 * When output is TVE, as the TVE has odd frame and even frame,
	 * to simplified operation, fg.y should be even number always.
	 *
	 */

	/* Foreground 0  */
	if (lcd_osd_info->fg1.x + lcd_osd_info->fg1.w > jz4750_lcd_info->panel.w)
		lcd_osd_info->fg1.x = jz4750_lcd_info->panel.w - lcd_osd_info->fg1.w;
	if (lcd_osd_info->fg1.y + lcd_osd_info->fg1.h > jz4750_lcd_info->panel.h)
		lcd_osd_info->fg1.y = jz4750_lcd_info->panel.h - lcd_osd_info->fg1.h;

	if (lcd_osd_info->fg1.x >= jz4750_lcd_info->panel.w)
		lcd_osd_info->fg1.x = jz4750_lcd_info->panel.w - 1;
	if (lcd_osd_info->fg1.y >= jz4750_lcd_info->panel.h)
		lcd_osd_info->fg1.y = jz4750_lcd_info->panel.h - 1;

	pos = lcd_osd_info->fg1.y << 16 | lcd_osd_info->fg1.x;

	if (REG_LCD_XYP1 == pos){
		printk("FG1: same position\n");
		//return 0;
	}

#if defined(CONFIG_SOC_JZ4750)
	/****jz4750****/
//	REG_LCD_OSDC &= ~LCD_OSDC_F0EN;
	REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
	REG_LCD_XYP1 = pos;
//	REG_LCD_OSDC |= LCD_OSDC_F0EN;
	/*********************************************/
#elif defined(CONFIG_SOC_JZ4750D)
	REG_LCD_XYP1 = pos;
	REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
	while(!(REG_LCD_OSDS & LCD_OSDS_READY));
	j = count;
	msleep(40);
	while((REG_LCD_OSDCTRL & LCD_OSDCTRL_CHANGES) && j--);
	if(j == 0) {
		printk("Error FG1 Position: Wait change fail.\n");
		return -EFAULT;

	}
#endif
	return 0;
}

/* Change window size of Foreground 1 */
static int jz4750fb_foreground_resize(struct jz4750lcd_osd_t *lcd_osd_info)
{
	struct lcd_cfb_info *cfb = jz4750fb_info;
	int size, fg1_line_size, fg1_frm_size;
//	int desc_len = sizeof(struct jz4750_lcd_dma_desc);
	/* 
	 * NOTE: 
	 * Foreground change sequence: 
	 * 	1. Change Position Registers -> LCD_OSDCTL.Change;
	 * 	2. LCD_OSDCTRL.Change -> descripter->Size
	 * Foreground, only one of the following can be change at one time:
	 * 	1. F0 size;
	 *	2. F0 position
	 * 	3. F1 size
	 *	4. F1 position
	 */
	
	/* 
	 * The rules of f0, f1's position:
	 * 	f0.x + f0.w <= panel.w;
	 * 	f0.y + f0.h <= panel.h;
	 *
	 * When output is LCD panel, fg.y and fg.h can be odd number or even number.
	 * When output is TVE, as the TVE has odd frame and even frame,
	 * to simplified operation, fg.y and fg.h should be even number always.
	 *
	 */

	/* Foreground 1  */

/*	if (lcd_osd_info->fg1.x + lcd_osd_info->fg1.w > jz4750_lcd_info->panel.w)
		lcd_osd_info->fg1.w = jz4750_lcd_info->panel.w - lcd_osd_info->fg1.x;
	if (lcd_osd_info->fg1.y + lcd_osd_info->fg1.h > jz4750_lcd_info->panel.h)
		lcd_osd_info->fg1.h = jz4750_lcd_info->panel.h - lcd_osd_info->fg1.y;
*/
//	size = lcd_info->osd.fg1.h << 16|lcd_info->osd.fg1.w;

	size = lcd_osd_info->fg1.h << 16|lcd_osd_info->fg1.w;
	if (REG_LCD_SIZE1 == size) {
		printk("FG1: same size\n");
		return 0;// -EFAULT;
	}
//	fg1_line_size = lcd_osd_info->fg1.w * ((lcd_osd_info->fg1.bpp + 7) / 8);
	fg1_line_size = lcd_osd_info->fg1.w * lcd_osd_info->fg1.bpp / 8;
	fg1_line_size = ((fg1_line_size + 3) >> 2) << 2; /* word aligned */
	fg1_frm_size = fg1_line_size * lcd_osd_info->fg1.h;

#if 0
	if (current_dma1_id == 0) {
		if (jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) {
			dma1_desc0_change->cmd = dma1_desc1_change->cmd = (fg1_frm_size/4)/2;
			dma1_desc0_change->offsize = dma1_desc1_change->offsize 
				= fg1_line_size/4;
			dma1_desc0_change->page_width = dma1_desc1_change->page_width 
				= fg1_line_size/4;
			dma1_desc1_change->databuf = virt_to_phys((void *)(lcd_frame + fg1_line_size));
			dma1_desc0_change->desc_size = dma1_desc1->desc_size = size;
			dma_cache_wback_inv((unsigned int)(dma1_desc0_change), desc_len);
			dma_cache_wback_inv((unsigned int)(dma1_desc1_change), desc_len);
		}
		else {
			dma1_desc0_change->cmd = fg1_frm_size/4;
			dma1_desc0_change->offsize = 0;
			dma1_desc0_change->page_width = 0;
			dma1_desc0_change->desc_size = size;
			dma1_desc0_change->next_desc = (unsigned int)virt_to_phys(dma1_desc0_change);
			dma_cache_wback_inv((unsigned int)(dma1_desc0_change),desc_len);
		}

		REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
		if (jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) {
			dma1_desc1->next_desc = (unsigned int)virt_to_phys(dma1_desc0_change);
			dma_cache_wback_inv((unsigned int)(dma1_desc1_change), desc_len);
		}
		else {
			dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc0_change);
			dma_cache_wback_inv((unsigned int)(dma1_desc0_change), desc_len);
		}
		REG_LCD_SIZE1 = size;
		current_dma1_id = 1;//dma1_desc0_change;

	}
	else {
		if (jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) {
			dma1_desc0->cmd = dma1_desc1->cmd = (fg1_frm_size/4)/2;
			dma1_desc0->offsize = dma1_desc1->offsize 
				= fg1_line_size/4;
			dma1_desc0->page_width = dma1_desc1->page_width 
				= fg1_line_size/4;
			dma1_desc1->databuf = virt_to_phys((void *)(lcd_frame + fg1_line_size));
			dma1_desc0->desc_size = dma1_desc1->desc_size = size;
		}
		else {
			dma1_desc0->cmd = fg1_frm_size/4;
			dma1_desc0->offsize =0;
			dma1_desc0->page_width = 0;
			dma1_desc0->desc_size = size;
			dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
			dma_cache_wback_inv((unsigned int)(dma1_desc0), desc_len);
		}
		if (jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) {
			dma1_desc1_change->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
			dma_cache_wback_inv((unsigned int)(dma1_desc1_change), desc_len);
		}
		else {
			dma1_desc0_change->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
			dma_cache_wback_inv((unsigned int)(dma1_desc0_change), desc_len);
		}
		REG_LCD_SIZE1 = size;
		current_dma1_id = 0;//dma1_desc0_change;
	}


#else
	/* set change bit */
	REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;
	if ( jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* output to TV */
		if(jz4750_lcd_info->osd.fg1.h % 2 == 0){
			dma1_desc0->cmd = LCD_CMD_EOFINT | (fg1_frm_size/4)/2;
			dma1_desc1->cmd = LCD_CMD_EOFINT | (fg1_frm_size/4)/2;
		}
		else{
			dma1_desc0->cmd = LCD_CMD_EOFINT | (fg1_frm_size/4 + fg1_line_size/4)/2;
			dma1_desc1->cmd = LCD_CMD_EOFINT | (fg1_frm_size/4 - fg1_line_size/4)/2;
		}
		dma1_desc0->offsize = dma1_desc1->offsize = fg1_line_size/4;
		dma1_desc0->page_width = dma1_desc1->page_width = fg1_line_size/4;
		dma1_desc1->databuf = virt_to_phys((void *)(lcd_frame + fg1_line_size));
	}
	else {
		dma1_desc0->cmd = dma1_desc1->cmd = LCD_CMD_EOFINT | fg1_frm_size/4;
		dma1_desc0->offsize = dma1_desc1->offsize = 0;
		dma1_desc0->page_width = dma1_desc1->page_width = 0;//fg1_line_size;   	
	}

	dma1_desc0->desc_size = dma1_desc1->desc_size = size;
	REG_LCD_SIZE1 = size;

	dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));
#endif
	jz4750fb_set_var(&cfb->fb.var, 1, &cfb->fb);
	return 0;
}



/*
 * Set lcd pixel clock 
 */
static void jz4750fb_change_clock( struct jz4750lcd_info * lcd_info )
{
	unsigned int val = 0;
	unsigned int pclk;

	/* Timing setting */
	__cpm_stop_lcd();

	val = lcd_info->panel.fclk; /* frame clk */

	if ( (lcd_info->panel.cfg & LCD_CFG_MODE_MASK) != LCD_CFG_MODE_SERIAL_TFT) {
		pclk = val * (lcd_info->panel.w + lcd_info->panel.hsw + lcd_info->panel.elw + lcd_info->panel.blw) * (lcd_info->panel.h + lcd_info->panel.vsw + lcd_info->panel.efw + lcd_info->panel.bfw); /* Pixclk */
	}
	else {
		/* serial mode: Hsync period = 3*Width_Pixel */
		pclk = val * (lcd_info->panel.w*3 + lcd_info->panel.hsw + lcd_info->panel.elw + lcd_info->panel.blw) * (lcd_info->panel.h + lcd_info->panel.vsw + lcd_info->panel.efw + lcd_info->panel.bfw); /* Pixclk */
	}

	/********* In TVE mode PCLK = 27MHz ***********/
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { 		/* LCDC output to TVE */
		pclk = 27000000;
		__cpm_select_pixclk_tve();
	}
	else { 		/* LCDC output to  LCD panel */
		__cpm_select_pixclk_lcd();
	}
	val = __cpm_get_pllout2() / pclk; /* pclk */
	val--;
	dprintk("ratio: val = %d\n", val);
	if ( val > 0x7ff ) {
		printk("pixel clock divid is too large, set it to 0x7ff\n");
		val = 0x7ff;
	}
	
	__cpm_set_pixdiv(val);
	
	dprintk("REG_CPM_LPCDR = 0x%08x\n", REG_CPM_LPCDR);
#if defined(CONFIG_SOC_JZ4750) /* Jz4750D don't use LCLK */
	val = pclk * 3 ;	/* LCDClock > 2.5*Pixclock */
	val =__cpm_get_pllout2() / val;
	if ( val > 0x1f ) {
		printk("lcd clock divide is too large, set it to 0x1f\n");
		val = 0x1f;
	}
	__cpm_set_ldiv( val );
#endif
	__cpm_enable_pll_change();

	dprintk("REG_CPM_LPCDR=0x%08x\n", REG_CPM_LPCDR);
	dprintk("REG_CPM_CPCCR=0x%08x\n", REG_CPM_CPCCR);

	jz_clocks.pixclk = __cpm_get_pixclk();
	printk("LCDC: PixClock:%d\n", jz_clocks.pixclk);

#if defined(CONFIG_SOC_JZ4750) /* Jz4750D don't use LCLK */
	jz_clocks.lcdclk = __cpm_get_lcdclk();
	printk("LCDC: LcdClock:%d\n", jz_clocks.lcdclk);
#endif
	__cpm_start_lcd();
	udelay(1000);

}


/* 
 * jz4750fb_set_mode(), set osd configure, resize foreground
 *
 */
static void jz4750fb_set_mode(struct jz4750lcd_osd_t * lcd_osd_info)
{
	jz4750fb_set_osd_mode(lcd_osd_info);
	jz4750fb_foreground_resize(lcd_osd_info);
	jz4750fb0_foreground_resize(lcd_osd_info);
}

/* 
 * jz4750fb_deep_set_mode, 
 *
 */
static void jz4750fb_deep_set_mode( struct jz4750lcd_info * lcd_info )
{
	/* configurate sequence:
	 * 1. disable lcdc.
	 * 2. init frame descriptor.
	 * 3. set panel mode
	 * 4. set osd mode
	 * 5. start lcd clock in CPM
	 * 6. enable lcdc.
	 */
	struct lcd_cfb_info *cfb = jz4750fb_info;

	__lcd_clr_ena();	/* Quick Disable */
	lcd_info->osd.fg_change = FG_CHANGE_ALL; /* change FG0, FG1 size, postion??? */
	jz4750fb_set_osd_mode(&lcd_info->osd);
	jz4750fb_set_panel_mode(lcd_info);
	jz4750fb_descriptor_init(lcd_info);
	jz4750fb_change_clock(lcd_info);

	jz4750fb_set_var(&cfb->fb0.var, 0, &cfb->fb0);
	jz4750fb_set_var(&cfb->fb.var, 1, &cfb->fb);
	__lcd_set_ena();	/* enable lcdc */
}


static irqreturn_t jz4750fb_interrupt_handler(int irq, void *dev_id)
{
	unsigned long irq_flags;
	unsigned int state;//, osdstate;
	struct lcd_cfb_info *cfb = jz4750fb_info;
	static int irqcnt = 0;

	spin_lock_irqsave(cfb->update_lock, irq_flags);
//	dprintk("Lcd irq, state=0x%08x, osdstate=0x%08x\n", state, osdstate);
	state = REG_LCD_STATE;
//	osdstate = REG_LCD_OSDS;
	if (state & LCD_STATE_EOF) {/* End of frame */
		REG_LCD_STATE = state & ~LCD_STATE_EOF;
//		dprintk("lcd dma eof interrupt\n");
	}
/****************************************************************/
#if 0
	if (state & LCD_STATE_IFU0) {
		printk("%s, InFiFo0 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU0;
	}

	if (state & LCD_STATE_IFU1) {
		printk("%s, InFiFo1 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU1;
	}

#endif
	if (state & LCD_STATE_OFU) {
		REG_LCD_STATE = state & ~LCD_STATE_OFU;
		if ( irqcnt++ > 100 ) {
			__lcd_disable_ofu_intr();
			printk("disable Out FiFo underrun irq.\n");
		}
		printk("%s, Out FiFo underrun.\n", __FUNCTION__);
	}
/****************************************************************/
	cfb->frame_done = cfb->frame_requested;
	spin_unlock_irqrestore(cfb->update_lock, irq_flags);
	wake_up(&cfb->frame_wq);

	return IRQ_HANDLED;
}


#ifdef CONFIG_HAS_EARLYSUSPEND

static void jz4750fb_earlier_suspend(struct early_suspend *h)
{
	lcd_display_off();
	__cpm_stop_lcd();
}

static void jz4750fb_earlier_resume(struct early_suspend *h)
{
	__cpm_start_lcd();
	lcd_display_on();
}

#endif /* CONFIG_HAS_EARLYSUSPEND */

/* The following routine is only for test */

static void jz4750_lcd_gpio_init(void)
{
	/* gpio init __gpio_as_lcd */
	if (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_16BIT)
		__gpio_as_lcd_16bit();
	else if (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_24BIT)
		__gpio_as_lcd_24bit();
	else
 		__gpio_as_lcd_18bit();

	/* Configure SLCD module for setting smart lcd control registers */
#if defined(CONFIG_FB_JZ4750_SLCD)
	__lcd_as_smart_lcd();
	__slcd_disable_dma();
	__init_slcd_bus();	/* Note: modify this depend on you lcd */

#endif	
	__lcd_display_pin_init();
}

static void jz4750_lcd_init_cfg(void)
{

	jz4750_lcd_info->osd.osd_cfg |= LCD_OSDC_F0EN; /* only open fg0 */
//	jz4750_lcd_info->osd.osd_cfg |= LCD_OSDC_F1EN; /* only open fg1 */

	/* In special mode, we only need init special pin, 
	 * as general lcd pin has init in uboot */
#if defined(CONFIG_SOC_JZ4750)
	switch (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) {
	case LCD_CFG_MODE_SPECIAL_TFT_1:
	case LCD_CFG_MODE_SPECIAL_TFT_2:
	case LCD_CFG_MODE_SPECIAL_TFT_3:
		__gpio_as_lcd_special();
		break;
	default:
		break;
	}
#endif
	/* Foreground 0 support bpp = 1, 2, 4, 8, 15, 16, 18, 24 */
	switch ( jz4750_lcd_info->osd.fg0.bpp ) {
	case 17 ... 32:
		jz4750_lcd_info->osd.fg1.bpp = 32;
		break;
	default:
		break;
	}

	/* Foreground 1 support bpp = 15, 16, 18, 24 */
	switch ( jz4750_lcd_info->osd.fg1.bpp ) {
	case 15:
	case 16:
		break;
	case 17 ... 32:
		jz4750_lcd_info->osd.fg1.bpp = 32;
		break;
	default:
		printk("jz4750fb fg1 not support bpp(%d), force to 32bpp\n", 
		       jz4750_lcd_info->osd.fg1.bpp);
		jz4750_lcd_info->osd.fg1.bpp = 32;
	}
}

#ifdef CONFIG_LEDS_CLASS
static void lcd_set_backlight_level(struct led_classdev *led_cdev, enum led_brightness value)
{
	__lcd_set_backlight_level((int)value);
}

static struct led_classdev lcd_backlight_led = {
	.name			= "lcd-backlight",
	.brightness_set		= lcd_set_backlight_level,
};
#endif

static int __init jz4750fb_probe(struct platform_device *pdev)
{
	struct lcd_cfb_info *cfb;
	int err = 0;

	jz_panel[0].w = jz4750_lcd_panel.panel.w;
	jz_panel[0].h = jz4750_lcd_panel.panel.h;
	jz_panel[0].index = 0;

	jz_panel[1].w = jz4750_info_tve.panel.w;
	jz_panel[1].h = jz4750_info_tve.panel.h;
	jz_panel[1].index = 1;

	if (!pdev)
		return -EINVAL;

	__lcd_close_backlight();
	jz4750_lcd_gpio_init();		/* gpio init */
	jz4750_lcd_init_cfg();		/* first config of lcd */

	__lcd_clr_dis();
	__lcd_clr_ena();

	/* init clock */
	__lcd_slcd_special_on();

	cfb = jz4750fb_alloc_fb_info();
	if (!cfb)
		goto failed;

	err = jz4750fb_map_smem(cfb);
	if (err)
		goto failed;

	spin_lock_init(&cfb->update_lock);
	init_waitqueue_head(&cfb->frame_wq);
	cfb->frame_requested = cfb->frame_done = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
	cfb->earlier_suspend.suspend = jz4750fb_earlier_suspend;
	cfb->earlier_suspend.resume = jz4750fb_earlier_resume;
	cfb->earlier_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&cfb->earlier_suspend);
#endif

	jz4750fb_deep_set_mode( jz4750_lcd_info ); 

	/* registers frame buffer devices */
	/* register fg0 */
	err = register_framebuffer(&cfb->fb0);
	if (err < 0) {
		dprintk("jz4750fb_init(): register framebuffer err.\n");
		goto failed;
	}
	printk("fb%d: %s frame buffer device, using %dK of video memory\n",
	       cfb->fb0.node, cfb->fb0.fix.id, cfb->fb0.fix.smem_len>>10);

	/* register fg1 */
	err = register_framebuffer(&cfb->fb);
	if (err < 0) {
		dprintk("jz4750fb_init(): register framebuffer err.\n");
		goto failed;
	}
	printk("fb%d: %s frame buffer device, using %dK of video memory\n",
	       cfb->fb.node, cfb->fb.fix.id, cfb->fb.fix.smem_len>>10);
#ifdef CONFIG_JZSOC_BOOT_LOGO
	load_565_image(INIT_IMAGE_FILE);
#endif
	if (request_irq(IRQ_LCD, jz4750fb_interrupt_handler, IRQF_DISABLED,
			"lcd", 0)) {
		err = -EBUSY;
		goto failed;
	}

	 
#ifdef CONFIG_LEDS_CLASS
	err = led_classdev_register(&pdev->dev, &lcd_backlight_led);
	if (err < 0)
		goto failed;
#endif

	lcd_display_on();
	print_lcdc_registers();


	return 0;

failed:
	print_dbg();
	jz_del_wired_entry();
	jz4750fb_unmap_smem(cfb);
	jz4750fb_free_fb_info(cfb);

	return err;
}

static int jz4750fb_remove(struct platform_device *pdev)
{
	struct lcd_cfb_info *cfb = platform_get_drvdata(pdev);

	jz_del_wired_entry();
	jz4750fb_unmap_smem(cfb);
	jz4750fb_free_fb_info(cfb);
	return 0;
}

static struct platform_driver jz_lcd_driver = {
	.probe = jz4750fb_probe,
	.remove = jz4750fb_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   },
};

static int __init jz4750fb_init(void)
{
	return platform_driver_register(&jz_lcd_driver);
}

static void __exit jz4750fb_cleanup(void)
{
	platform_driver_unregister(&jz_lcd_driver);
}

module_init(jz4750fb_init);
module_exit(jz4750fb_cleanup);
