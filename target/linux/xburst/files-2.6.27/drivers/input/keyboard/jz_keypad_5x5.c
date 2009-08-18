/*
 * JZ Keypad ( 5 x 5 ) Driver
 *
 * Copyright (c) 2005 - 2008  Ingenic Semiconductor Inc.
 *
 * Author: Jason <xwang@ingenic.cn> 20090210
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/version.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
//#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <asm/gpio.h>
#include <asm/jzsoc.h>

#define KB_ROWS			5
#define KB_COLS         	5
#define KB_COUNT		25


#define JZ_KEY_PRESSED		0x01
#define JZ_KEY_RELEASED		0x00

#define SCAN_INTERVAL      	10	/* jiffies */

#define ROW_KEYBIT_MASK		0xFFE0

#define SET_GPIOS_AS_INPUT()						\
do {									\
	unsigned short i;						\
									\
	for (i = 0; i < KB_ROWS; i++) {					\
		__gpio_as_input(jz_row_gpios[i]);			\
		__gpio_as_input(jz_col_gpios[i]);			\
	}								\
} while (0)


#define GET_ROW_GPIO_PINS()						\
({									\
	unsigned short _pins = 0, i;					\
	for (i = 0;							\
	     i < KB_ROWS;						\
	     _pins |= __gpio_get_pin(jz_row_gpios[i]) << i, i++)	\
		;							\
	_pins;								\
})

#define CHECK_IF_KEY_PRESSED(s)						\
({									\
	unsigned short i;						\
	for (i = 0; i < KB_COLS && s[i] == 0x1F ; i++)			\
		;							\
	i != KB_ROWS;							\
})

#define CLEAN_SCAN_RESULT(s)						\
do {									\
	unsigned short i;						\
	for (i = 0; i < KB_COLS; s[i++] = 0x1F)				\
		;							\
} while (0)


static const unsigned short jz_col_gpios[KB_ROWS] = {76, 75, 74, 73, 72};
static const unsigned short jz_row_gpios[KB_COLS] = {181, 182, 79, 78, 77};

static const unsigned int jz_kbd_keycode[KB_ROWS * KB_COLS] = {
	KEY_A, KEY_B, KEY_C, KEY_D, KEY_E,
	KEY_F, KEY_G, KEY_H, KEY_I, KEY_J,
	KEY_K, KEY_L, KEY_M, KEY_N, KEY_O,
	KEY_P, KEY_Q, KEY_R, KEY_S, KEY_T,
	KEY_LEFTSHIFT, KEY_LEFTCTRL, KEY_LEFTALT, KEY_BACKSPACE, KEY_ENTER
};

static unsigned short jz_kbd_status[KB_ROWS * KB_COLS] = {0};

struct jz_kbd {
	unsigned int keycode[ARRAY_SIZE(jz_kbd_keycode)];
	struct input_dev *input;
	char phys[32];

	spinlock_t lock;
	struct timer_list timer;

	unsigned int suspended;
	unsigned long suspend_jiffies;
};
static struct jz_kbd g_jz_kbd;

/* An element shows the scan result of a whole row. LSB --> column 0 */
static unsigned short scan_result[KB_ROWS];

/** 
 *  Scan keypad by reading GPIO pins.
 */
static inline void jz_do_scan(unsigned short *s)
{
	unsigned short i;

	if (!s)
		return ;

	for (i = 0; i < KB_COLS; i++) {

		SET_GPIOS_AS_INPUT();
		__gpio_clear_pin(jz_col_gpios[i]);
		__gpio_as_output(jz_col_gpios[i]);

		udelay(1000);

		s[i] = GET_ROW_GPIO_PINS();
	}
}

/** 
 *  Call scan function and handle 'GPIO event'(like key down, key up),
 *  and report it to upper layer of input subsystem ... if necessary
 */
static void jz_kbd_scan(struct jz_kbd *kbd_data)
{
	unsigned short row, col;
	unsigned long flags;

	if (kbd_data->suspended)
		return;

	spin_lock_irqsave(&kbd_data->lock, flags);

	jz_do_scan(scan_result);

	/* handle gpio event */
	for (row = 0; row < KB_ROWS; row++) {
		for (col = 0; col < KB_COLS; col++) {

			if ( !(scan_result[row] & 0x01) ) {
				/* oh! a key pressed ... may be it is not news ...*/
				input_report_key(kbd_data->input, kbd_data->keycode[row * KB_COLS + col], 1);
				input_sync(kbd_data->input);
				jz_kbd_status[row * KB_ROWS + col] = JZ_KEY_PRESSED;

			} else {
				/* if the key has been pressed ... release it */
				if (jz_kbd_status[row * KB_ROWS + col]) {
					input_report_key(kbd_data->input, kbd_data->keycode[row * KB_COLS + col], 0);
					input_sync(kbd_data->input);
					jz_kbd_status[row * KB_ROWS + col] = JZ_KEY_RELEASED;
				}
			}

			scan_result[row] >>= 1;
		}
	}

	spin_unlock_irqrestore(&kbd_data->lock, flags);

	return;
}

static void jz_kbd_timer_callback(unsigned long data)
{
	jz_kbd_scan(&g_jz_kbd);
	mod_timer(&g_jz_kbd.timer, jiffies + SCAN_INTERVAL);
}

static int __init jz_kbd_init(void)
{
	struct input_dev *input_dev;
	int i, error;

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	strcpy(g_jz_kbd.phys, "input/kbd0");

	spin_lock_init(&g_jz_kbd.lock);

	g_jz_kbd.suspend_jiffies = jiffies;
	g_jz_kbd.input = input_dev;

	input_dev->name = "JZ 5x5 Keypad";
	input_dev->phys = g_jz_kbd.phys;
	input_dev->id.bustype = BUS_PARPORT;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_REP) | BIT(EV_SYN);
	input_dev->keycode = g_jz_kbd.keycode;
	input_dev->keycodesize = sizeof(unsigned int);
	input_dev->keycodemax = ARRAY_SIZE(jz_kbd_keycode);

	memcpy(g_jz_kbd.keycode, jz_kbd_keycode, sizeof(g_jz_kbd.keycode));

	for ( i = 0; i < ARRAY_SIZE(jz_kbd_keycode); i++)
		set_bit(g_jz_kbd.keycode[i], input_dev->keybit);

	init_timer(&g_jz_kbd.timer);
	g_jz_kbd.timer.function = jz_kbd_timer_callback;
	g_jz_kbd.timer.data = (unsigned long)&g_jz_kbd;
	mod_timer(&g_jz_kbd.timer, jiffies + SCAN_INTERVAL);

	input_set_drvdata(input_dev, &g_jz_kbd);
	error = input_register_device(input_dev);
	if (error) {
		pr_err("gpio-keys: Unable to register input device,  "
		       "error: %d\n", error);
	}
	printk("input: JZ 5x5 Keypad Registered.\n");

	return 0;
}

static void __exit jz_kbd_exit(void)
{
	del_timer_sync(&g_jz_kbd.timer);

	SET_GPIOS_AS_INPUT();

	input_unregister_device(g_jz_kbd.input);
}

module_init(jz_kbd_init);
module_exit(jz_kbd_exit);

MODULE_AUTHOR("Jason <xwang@ingenic.cn>");
MODULE_DESCRIPTION("JZ 5x5 keypad driver");
MODULE_LICENSE("GPL");
