#ifndef __JZ4750_TVE_H__
#define __JZ4750_TVE_H__


#define PANEL_MODE_TVE_NTSC	2
#define PANEL_MODE_TVE_PAL	1
#define PANEL_MODE_LCD_PANEL	0

#define PANEL_OUT_FMT_YCBCR	2
#define PANEL_OUT_FMT_SVIDEO	1
#define PANEL_OUT_FMT_CVBS	0

/* TV parameter */
#define TVE_WIDTH_PAL 		720
#define TVE_WIDTH_PAL16 	704
#define TVE_HEIGHT_PAL 		573
#define TVE_HEIGHT_PAL16	553
#define TVE_FREQ_PAL 		50
#define TVE_WIDTH_NTSC 		720
#define TVE_WIDTH_NTSC16	704
#define TVE_HEIGHT_NTSC 	482
#define TVE_FREQ_NTSC 		60



/* Structure for TVE */
struct jz4750tve_info {
	unsigned int ctrl;
	unsigned int frcfg;
	unsigned int slcfg1;
	unsigned int slcfg2;
	unsigned int slcfg3;
	unsigned int ltcfg1;
	unsigned int ltcfg2;
	unsigned int cfreq;
	unsigned int cphase;
	unsigned int cbcrcfg;
	unsigned int wsscr;
	unsigned int wsscfg1;
	unsigned int wsscfg2;
	unsigned int wsscfg3;
};

struct jz4750tve_mode {
	unsigned int mode; /* PAL or NTSC mode, lcd mode*/
	unsigned int out_fmt; /* CVBS, S-video, YCbCr(Jz4750 didn't support)*/
};

extern struct jz4750tve_info *jz4750_tve_info;

extern void jz4750tve_enable_tve(void);
extern void jz4750tve_disable_tve(void);

extern void jz4750tve_set_tve_mode( struct jz4750tve_info *tve );
extern void jz4750tve_init( int tve_mode );

extern void jz4750tve_outfmt_init(unsigned int outfmt);

#endif	/* __JZ4750_TVE_H__ */
