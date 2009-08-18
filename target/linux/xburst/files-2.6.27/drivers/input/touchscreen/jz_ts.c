/*
 * JZ Touch Screen Driver
 *
 * Copyright (c) 2005 - 2009  Ingenic Semiconductor Inc.
 *
 * Author: Jason <xwang@ingenic.cn> 20090219
 *         Regen <lhhuang@ingenic.cn> 20090324 add adkey
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/jzsoc.h>

#define TS_NAME "jz-ts"

#define KEY_SCAN_INTERVAL       5
#define TS_SCAN_INTERVAL	0

/* from qwerty.kl of android */
#define  DPAD_CENTER            232
#define  DPAD_DOWN              108
#define  DPAD_UP                103
#define  DPAD_LEFT              105
#define  DPAD_RIGHT             106

/* TS event status */
#define PENUP			0x00
#define PENDOWN			0x01

/* Sample times in one sample process	*/
#define SAMPLE_TIMES		3

/* Min pressure value. If less than it, filt the point.
 * Mask it if it is not useful for you
 */
//#define MIN_PRESSURE		0x100

/* Max delta x distance between current point and last point.	*/
#define MAX_DELTA_X_OF_2_POINTS	200
/* Max delta x distance between current point and last point.	*/
#define MAX_DELTA_Y_OF_2_POINTS	120

/* Max delta between points in one sample process
 * Verify method :
 * 	(diff value / min value) * 100 <= MAX_DELTA_OF_SAMPLING
 */
#define MAX_DELTA_OF_SAMPLING	20

#define DIFF(a,b)		(((a)>(b))?((a)-(b)):((b)-(a)))
#define MIN(a,b)		(((a)<(b))?(a):(b))



/*
 * TS deriver
 */
struct jz_ts_t {
	int pendown_irq;		// IRQ of pendown interrupt
	int pen_is_down;		// 1 = pen is down, 0 = pen is up
	int irq_enabled;
	struct timer_list acq_timer;	// Timer for triggering acquisitions
#ifdef CONFIG_JZ_ADKEY
	struct timer_list key_timer;	// for adkey
	int active_low;		// for adkey's interrupt pin
#endif
	wait_queue_head_t wait;		// read wait queue
	spinlock_t lock;

	/* Following 4 members use to pass arguments from u-boot to tell us the ts data.
	 * But in Android we do not use them.
	 */
/*
	int minx, miny, maxx, maxy;
*/
	int first_read;

	char	phys[32];
	struct input_dev *input_dev;
};

/*
 * TS Event type
 */
struct ts_event {
	u16 status;
        u16 x;
        u16 y;
        u16 pressure;
        u16 pad;
};

#ifdef CONFIG_JZ_ADKEY
struct ad_keys_button {
	int code;		/* input event code */
	int val;                /* the ad value of the key */
	int fuzz;               /* the error(+-fuzz) allowed of the ad value of the key */
};
static struct ad_keys_button ad_buttons[] = {
	{
		.code = DPAD_LEFT,
		.val = DPAD_LEFT_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_DOWN,
		.val = DPAD_DOWN_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_UP,
		.val = DPAD_UP_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_CENTER,
		.val = DPAD_CENTER_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_RIGHT,
		.val = DPAD_RIGHT_LEVEL,
		.fuzz = 40,
	},
};
#define KEY_NUM (sizeof(ad_buttons) / sizeof(struct ad_keys_button))
#endif

/************************************************************************/
/*	SAR ADC OPS							*/
/************************************************************************/

typedef struct datasource {
        u16 xbuf;
        u16 ybuf;
	u16 zbuf;
}datasource_t;

static datasource_t data_s;
static unsigned int p;

static DECLARE_WAIT_QUEUE_HEAD (sadc_wait_queue);

static int first_time = 0;
//static unsigned long last_x, last_y, last_p;
static unsigned int old_x, old_y;

extern unsigned int (*codec_read_battery)(void);
#if 0
static void reg_debug(void)
{
        printk("\t####CTRL####################################################\n");
        printk("\tPEND %s,   ", REG_SADC_CTRL & SADC_CTRL_PENDM ? "masked" : "enabled");
        printk("PENU %s,   ", REG_SADC_CTRL & SADC_CTRL_PENUM ? "masked" : "enabled");
        printk("TSRDY %s\n", REG_SADC_CTRL & SADC_CTRL_TSRDYM ? "masked" : "enabled");
        printk("\t----STATE---------------------------------------------------\n");
        printk("\tIRQ actived: %s,   %s,   %s\n",
               REG_SADC_STATE & SADC_STATE_PEND ? "pen down" : "        ",
               REG_SADC_STATE & SADC_STATE_PENU ? "pen up  " : "        ",
               REG_SADC_STATE & SADC_STATE_TSRDY ? "sample  " : "        ");
        printk("\t############################################################\n");
}
#endif
/* 
 * set adc clock to 24MHz/div. A/D works at freq between 500KHz to 8MHz.
 */
static void sadc_init_clock(int div)
{
	if (div < 2) div = 2;
	if (div > 23) div = 23;
#if defined(CONFIG_SOC_JZ4740)	
	REG_SADC_CFG &= ~SADC_CFG_CLKDIV_MASK;
	REG_SADC_CFG |= (div - 1) << SADC_CFG_CLKDIV_BIT;
#endif
#if defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D)
	REG_SADC_ADCLK &= ~SADC_ADCLK_CLKDIV_MASK;
	REG_SADC_ADCLK |= (div - 1) << SADC_ADCLK_CLKDIV_BIT;
	REG_SADC_ADCLK &= ~SADC_ADCLK_CLKDIV_BIT;
	REG_SADC_ADCLK |= 39 << SADC_ADCLK_CLKDIV_10_BIT;  /* if div ==3,here is 39 */
#endif
}

static inline void sadc_start_sadcin(void)
{
	REG_SADC_ENA |= SADC_ENA_SADCINEN;
}

static void sadc_start_pbat(void)
{
	if (CFG_PBAT_DIV == 1)
		REG_SADC_CFG |= SADC_CFG_PBAT_HIGH;   /* full baterry voltage >= 2.5V */
	else
		REG_SADC_CFG |= SADC_CFG_PBAT_LOW;    /* full baterry voltage < 2.5V */
  	REG_SADC_ENA |= SADC_ENA_PBATEN;      /* Enable pbat adc */
}

static inline void ts_enable_pendown_irq(void)
{
	REG_SADC_CTRL &= ~SADC_CTRL_PENDM;
}

static inline void ts_enable_penup_irq(void)
{
	REG_SADC_CTRL &= ~SADC_CTRL_PENUM;
}

static inline void ts_disable_pendown_irq(void)
{
	REG_SADC_CTRL |= SADC_CTRL_PENDM;
}

static inline void ts_disable_penup_irq(void)
{
	REG_SADC_CTRL |= SADC_CTRL_PENUM;
}

static inline void sadc_enable_ts(void)
{
	REG_SADC_ENA |= SADC_ENA_TSEN;
}

static inline void sadc_disable_ts(void)
{
	REG_SADC_ENA &= ~SADC_ENA_TSEN;
}

static inline void sadc_start_ts(void)
{
	REG_SADC_SAMETIME = 10;	/* about 0.1 ms,you can change it */
	REG_SADC_WAITTIME = 2;	/* about 0.02 ms,you can change it */

	REG_SADC_CFG &= ~(SADC_CFG_TS_DMA | SADC_CFG_XYZ_MASK | SADC_CFG_SNUM_MASK);
	REG_SADC_CFG |= (SADC_CFG_EXIN | SADC_CFG_XYZ | SADC_CFG_SNUM_3);

	REG_SADC_CTRL |= (SADC_CTRL_TSRDYM | SADC_CTRL_PBATRDYM | SADC_CTRL_PENUM |SADC_CTRL_SRDYM);

	ts_enable_pendown_irq();

	sadc_enable_ts();
}

/** 
 * Read the battery voltage
 */
unsigned int jz_read_battery(void)
{
	unsigned int v;
	unsigned int timeout = 0x3fff;
	u16 pbat;

	if(!(REG_SADC_STATE & SADC_STATE_PBATRDY) ==1)
		sadc_start_pbat();

	while(!(REG_SADC_STATE & SADC_STATE_PBATRDY) && --timeout)
		;

	pbat = REG_SADC_BATDAT;
	v = pbat & 0x0fff;
	REG_SADC_STATE = SADC_STATE_PBATRDY;
	return v;
}

#define TSMAXX		3920
#define TSMAXY		3700
#define TSMINX		150
#define TSMINY		270

#define SCREEN_MAXX	479
#define SCREEN_MAXY	271

static unsigned long transform_to_screen_x(struct jz_ts_t *ts, unsigned long x )
{
/* Now we don't need u-boot to tell us the ts data.	*/
/*
	if (ts->minx)
	{
		if (x < ts->minx) x = ts->minx;
		if (x > ts->maxx) x = ts->maxx;

		return (x - ts->minx) * SCREEN_MAXX / (ts->maxx - ts->minx);
	}
	else
	{
*/
	if (x < TSMINX) x = TSMINX;
	if (x > TSMAXX) x = TSMAXX;

	return (x - TSMINX) * SCREEN_MAXX / (TSMAXX - TSMINX);
/*
	}
*/
}

static unsigned long transform_to_screen_y(struct jz_ts_t *ts, unsigned long y)
{
/* Now we don't need u-boot to tell us the ts data.	*/
/*
	if (ts->miny)
	{
		if (y < ts->miny) y = ts->miny;
		if (y > ts->maxy) y = ts->maxy;

		return (ts->maxy - y) * SCREEN_MAXY / (ts->maxy - ts->miny);
	}
	else
	{
*/
	if (y < TSMINY) y = TSMINY;
	if (y > TSMAXY) y = TSMAXY;

	return (TSMAXY - y) * SCREEN_MAXY / (TSMAXY - TSMINY);
/*
	}
*/
}

static inline void ts_data_ready(void)
{
	REG_SADC_CTRL |= SADC_CTRL_TSRDYM;
}

static int adc_read(struct jz_ts_t *ts)
{
	struct datasource *ds = &data_s;
	u32 xybuf,z;

	while (!(REG_SADC_STATE & SADC_STATE_TSRDY)) {
		REG_SADC_CTRL &= ~SADC_CTRL_TSRDYM;
	}

	xybuf = REG_SADC_TSDAT;
	ds->xbuf = (xybuf>>16) & 0x0fff;
	ds->ybuf = (xybuf)& 0x0fff;
	z = REG_SADC_TSDAT;
	ds->zbuf = z& 0x0fff;
  	REG_SADC_STATE &= ~SADC_STATE_TSRDY;
	return 0;
}

/*
 * Acquire raw pen coodinate data and compute touch screen
 * pressure resistance. Hold spinlock when calling.
 */
int adc_acquire_event(struct jz_ts_t *ts, struct ts_event *event)
{
	unsigned int x_raw[SAMPLE_TIMES], y_raw[SAMPLE_TIMES], p_raw[SAMPLE_TIMES];
	int i;
	unsigned int avl_x, avl_y, avl_p, diff_x, diff_y;
	struct datasource *ds = &data_s;
	avl_x = avl_y = avl_p = 0;

	for (i = 0; i < SAMPLE_TIMES; i++) {
		if (adc_read(ts)) {
			goto _INVALID_POINT;
		}

		x_raw[i] = ds->ybuf;
		y_raw[i] = ds->xbuf;
		p_raw[i] = ds->zbuf;

#ifdef MIN_PRESSURE
		if (p_raw[i] < MIN_PRESSURE) {
			goto _INVALID_POINT;
		}
#endif
		avl_x += x_raw[i];
		avl_y += y_raw[i];
		avl_p += p_raw[i];
#if 0
		printk("x_raw = %u , y_raw = %u , z_raw = %u\n",x_raw[i],y_raw[i],p_raw[i]);
#endif

	}

	avl_x /= SAMPLE_TIMES;
	avl_y /= SAMPLE_TIMES;
	avl_p /= SAMPLE_TIMES;

	/* Verify delta data. */
	for (i = 1; i < SAMPLE_TIMES; i++)
	{
		if ( ((DIFF(x_raw[i],x_raw[i-1]) / MIN(x_raw[i],x_raw[i-1])) * 100) > MAX_DELTA_OF_SAMPLING )
			goto _INVALID_POINT;

		if ( ((DIFF(y_raw[i],y_raw[i-1]) / MIN(y_raw[i],y_raw[i-1])) * 100) > MAX_DELTA_OF_SAMPLING )
			goto _INVALID_POINT;

		if ( ((DIFF(p_raw[i],p_raw[i-1]) / MIN(p_raw[i],p_raw[i-1])) * 100) > MAX_DELTA_OF_SAMPLING )
			goto _INVALID_POINT;
	}

	/* Compare with last point. */
	if (ts->first_read) {
		ts->first_read = 0;
		old_x = avl_x;
		old_y = avl_y;
	}

	diff_x = DIFF(old_x, avl_x);
	diff_y = DIFF(old_y, avl_y);

	if (diff_x >= MAX_DELTA_X_OF_2_POINTS || diff_y >= MAX_DELTA_Y_OF_2_POINTS)
		goto _INVALID_POINT;

	old_x = avl_x;
	old_y = avl_y;
		

	/* Android need it ... transform the raw value to screen coordinate. */
	event->x = transform_to_screen_x(ts, avl_x);
	event->y = transform_to_screen_y(ts, avl_y);

	event->pressure = (u16)avl_p;
	event->status = PENDOWN;

	return 1;

_INVALID_POINT:

	return 0;
}


/*
 * Interrupt handler
 */
void ts_irq_callback(void)
{
	u32 state;

	state = REG_SADC_STATE;
	if (!(REG_SADC_CTRL&SADC_CTRL_PENDM)&&(REG_SADC_STATE & SADC_STATE_PEND)) {
		REG_SADC_STATE = SADC_STATE_PEND;
		REG_SADC_STATE = SADC_STATE_PENU;
		REG_SADC_CTRL |= SADC_CTRL_PENDM;
		REG_SADC_CTRL &= ~SADC_CTRL_PENUM;
		p = 1;
	} 

	if (!(REG_SADC_CTRL&SADC_CTRL_PENUM)&&(REG_SADC_STATE & SADC_STATE_PENU)) {		
		REG_SADC_STATE = SADC_STATE_PENU;
		REG_SADC_CTRL |= SADC_CTRL_PENUM;
		REG_SADC_CTRL &= ~SADC_CTRL_PENDM;
		p = 0;
	}

	first_time = 1; // first time to acquire sample
}

static inline int PenIsDown(void)
{
	return p;
}

#ifdef CONFIG_JZ_ADKEY
/** 
 * Read the battery voltage
 */
static unsigned int read_sadcin(void)
{
	unsigned int v;
	unsigned int timeout = 0x3ff;
	u16 val;

	if(!(REG_SADC_STATE & SADC_STATE_SRDY))
		sadc_start_sadcin();

	while(!(REG_SADC_STATE & SADC_STATE_SRDY) && --timeout)
		;

	val = REG_SADC_SADDAT;
	v = val & 0x0fff;
	REG_SADC_STATE = SADC_STATE_SRDY;
	return v;
}

static unsigned int key_scan(int ad_val)
{
	 int i;

	 for(i = 0; i<KEY_NUM; i++) {
		if((ad_buttons[i].val + ad_buttons[i].fuzz >= ad_val) && 
		   (ad_val >=ad_buttons[i].val - ad_buttons[i].fuzz)) {
			return ad_buttons[i].code;
		}
	 }
	 return -1;
}

static void key_timer_callback(unsigned long data)
{
	struct jz_ts_t *ts = (struct jz_ts_t *)data;
	int state;
	int active_low = ts->active_low;
	int ad_val, code;
	static int old_code;

	spin_lock(&ts->lock);

	state = __gpio_get_pin(GPIO_ADKEY_INT);
	ad_val = read_sadcin();

        if (active_low) {
		if (state == 0) {
			/* press down */
			code = key_scan(ad_val);
			old_code = code;
			input_report_key(ts->input_dev, code, 1);
			input_sync(ts->input_dev);
			mod_timer(&ts->key_timer, jiffies + KEY_SCAN_INTERVAL);
		} else {
			/* up */
			input_report_key(ts->input_dev, old_code, 0);
			input_sync(ts->input_dev);
			udelay(1000);
			__gpio_as_irq_fall_edge(GPIO_ADKEY_INT);
		}
	} else {
		if (state == 1) {
			/* press down */
			code = key_scan(ad_val);
			old_code = code;
			input_report_key(ts->input_dev, code, 1);
			input_sync(ts->input_dev);
			mod_timer(&ts->key_timer, jiffies + KEY_SCAN_INTERVAL);
		} else {
			/* up */
			input_report_key(ts->input_dev, old_code, 0);
			input_sync(ts->input_dev);
			udelay(1000);
			__gpio_as_irq_rise_edge(GPIO_ADKEY_INT);
		}
	}

	spin_unlock(&ts->lock);
}

static irqreturn_t key_interrupt(int irq, void * dev_id)
{
	struct jz_ts_t *ts = dev_id;

	spin_lock(&ts->lock);

	__gpio_ack_irq(GPIO_ADKEY_INT);
	__gpio_as_input(GPIO_ADKEY_INT);
	sadc_start_sadcin();
	mod_timer(&ts->key_timer, jiffies + KEY_SCAN_INTERVAL);

	spin_unlock(&ts->lock);

	return IRQ_HANDLED;
}
#endif

/************************************************************************/
/*	Touch Screen module						*/
/************************************************************************/

static int pen_is_down = 0;

static irqreturn_t pendown_interrupt(int irq, void * dev_id)
{
	struct jz_ts_t *ts = dev_id;

	spin_lock_irq(&ts->lock);

	if (ts->irq_enabled)
		ts->irq_enabled = 0;
	else
		ts->irq_enabled = 1;

	
	if (pen_is_down)
		pen_is_down = 0;
	else
		pen_is_down = 1;

	/* callback routine to clear irq status */
	ts_irq_callback();

	if ( (pen_is_down == 0)){
		del_timer(&ts->acq_timer);
		ts->first_read = 0;
		input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
		/* Android need it ... */
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_sync(ts->input_dev);

	} else { // pen_is_down == 1

		ts->acq_timer.expires = jiffies + TS_SCAN_INTERVAL;
		del_timer(&ts->acq_timer);
		ts->first_read = 1;
		add_timer(&ts->acq_timer);
	}

	spin_unlock_irq(&ts->lock);

	return IRQ_HANDLED;
}

/*
 * Raw X,Y,pressure acquisition timer function. It gets scheduled
 * only while pen is down. Its duration between calls is the polling
 * rate.
 */
static void
jz_acq_timer(unsigned long data)
{
	struct jz_ts_t *ts = (struct jz_ts_t *)data;
	struct ts_event event;
	int pen_was_down = ts->pen_is_down;

	spin_lock_irq(&ts->lock);

	if (PenIsDown()) {

		ts->pen_is_down = 1;

		if (adc_acquire_event(ts, &event)) {// check event is valid or not?
			input_report_abs(ts->input_dev, ABS_X, event.x);
			input_report_abs(ts->input_dev, ABS_Y, event.y);
			input_report_abs(ts->input_dev, ABS_PRESSURE, event.pressure);
			/* Android need it ... */
			input_report_key(ts->input_dev, BTN_TOUCH, 1);

			input_sync(ts->input_dev);
		}

		// schedule next acquire
		ts->acq_timer.expires = jiffies + TS_SCAN_INTERVAL;
		del_timer(&ts->acq_timer);
		add_timer(&ts->acq_timer);
	} else {

		if (!ts->irq_enabled) {
			ts->irq_enabled = 1;
		}
		ts->pen_is_down = 0;
		if (pen_was_down) {
			input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
			/* Android need it ... */
			input_report_key(ts->input_dev, BTN_TOUCH, 0);

			input_sync(ts->input_dev);
		}
	}

	spin_unlock_irq(&ts->lock);
}

static struct jz_ts_t *jz_ts;
static int __init jz_ts_init(void)
{
	struct input_dev	*input_dev;
	struct jz_ts_t		*ts;
	int	error;

	ts = jz_ts = kzalloc(sizeof(struct jz_ts_t), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev)
		return -ENOMEM;

	input_dev->name = "qwerty"; /* Set to 'qwerty' to load /system/usr/keychars/qwerty.kcm.bin by Android */
	input_dev->phys = ts->phys;

/*
 	old:
	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
*/

	/* For Android */
	set_bit(EV_ABS, input_dev->evbit);
        set_bit(ABS_X, input_dev->absbit);
        set_bit(ABS_Y, input_dev->absbit);
        set_bit(ABS_PRESSURE, input_dev->absbit);
        set_bit(EV_KEY, input_dev->evbit);
        set_bit(BTN_TOUCH, input_dev->keybit);

#ifdef CONFIG_JZ_ADKEY
	set_bit(DPAD_CENTER, input_dev->keybit);
	set_bit(DPAD_DOWN, input_dev->keybit);
	set_bit(DPAD_UP, input_dev->keybit);
	set_bit(DPAD_LEFT, input_dev->keybit);
	set_bit(DPAD_RIGHT, input_dev->keybit);
#endif
	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAXX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAXY, 0, 0);
        input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_drvdata(input_dev, ts);
	error = input_register_device(input_dev);

	strcpy(ts->phys, "input/ts0");
	spin_lock_init(&ts->lock);

	ts->input_dev = input_dev;

	// Init ts acquisition timer function
	init_timer(&ts->acq_timer);
	ts->acq_timer.function = jz_acq_timer;
	ts->acq_timer.data = (unsigned long)ts;
	ts->irq_enabled = 1;

	if (error) {
		printk("Input device register failed !\n");
		goto err_free_dev;
	}

        sadc_init_clock(6);
	ts_disable_pendown_irq();
	ts_disable_penup_irq();

	error = request_irq(IRQ_SADC, pendown_interrupt, IRQF_DISABLED, TS_NAME, ts);
	if (error) {
		pr_err("unable to get PenDown IRQ %d", IRQ_SADC);
		goto err_free_irq;
	}
#ifdef CONFIG_JZ_ADKEY
	// Init key acquisition timer function
	init_timer(&ts->key_timer);
	ts->key_timer.function = key_timer_callback;
	ts->key_timer.data = (unsigned long)ts;
	ts->active_low = ACTIVE_LOW_ADKEY;

	error = request_irq(IRQ_GPIO_0 + GPIO_ADKEY_INT, key_interrupt, IRQF_DISABLED, TS_NAME, ts);
	if (error) {
		pr_err("unable to get AD KEY IRQ %d", IRQ_GPIO_0 + GPIO_ADKEY_INT);
		goto err_free_irq;
	}

	__gpio_disable_pull(GPIO_ADKEY_INT);
	if(ts->active_low)
		__gpio_as_irq_fall_edge(GPIO_ADKEY_INT);
	else
		__gpio_as_irq_rise_edge(GPIO_ADKEY_INT);

#endif
	sadc_start_ts();

	printk("input: JZ Touch Screen registered.\n");

	return 0;

err_free_irq:
	free_irq(IRQ_SADC, ts);
#ifdef CONFIG_JZ_ADKEY
	free_irq(IRQ_GPIO_0 + GPIO_ADKEY_INT, ts);
#endif
err_free_dev:
	input_free_device(ts->input_dev);
	kfree(ts);
        return 0;
}

static void __exit jz_ts_exit(void)
{
        free_irq(IRQ_SADC, jz_ts);
        input_unregister_device(jz_ts->input_dev);

	ts_disable_pendown_irq();
	ts_disable_penup_irq();

	sadc_disable_ts();
}

module_init(jz_ts_init);
module_exit(jz_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JZ TouchScreen Driver");
MODULE_AUTHOR("Jason <xwang@ingenic.com>");
