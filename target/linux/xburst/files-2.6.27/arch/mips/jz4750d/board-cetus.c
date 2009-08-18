/*
 * linux/arch/mips/jz4750d/board-cetus.c
 *
 * JZ4750D CETUS board setup routines.
 *
 * Copyright (c) 2006-2008  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>

/*********************************************************************************
 * Power management routines
 ********************************************************************************/

/* 
 * __gpio_as_sleep set all pins to pull-disable, and set all pins as input
 * except sdram and the pins which can be used as CS1_N to CS4_N for chip select. 
 */
#define __gpio_as_sleep()	              \
do {	                                      \
	REG_GPIO_PXFUNC(1) = ~0x03ff7fff;     \
	REG_GPIO_PXSELC(1) = ~0x03ff7fff;     \
	REG_GPIO_PXDIRC(1) = ~0x03ff7fff;     \
	REG_GPIO_PXPES(1)  = ~0x03ff7fff;     \
	REG_GPIO_PXFUNC(2) = ~0x01e00000;     \
	REG_GPIO_PXSELC(2) = ~0x01e00000;     \
	REG_GPIO_PXDIRC(2) = ~0x01e00000;     \
	REG_GPIO_PXPES(2)  = ~0x01e00000;     \
	REG_GPIO_PXFUNC(3) =  0xffffffff;     \
	REG_GPIO_PXSELC(3) =  0xffffffff;     \
	REG_GPIO_PXDIRC(3) =  0xffffffff;     \
	REG_GPIO_PXPES(3)  =  0xffffffff;     \
	REG_GPIO_PXFUNC(4) =  0xffffffff;     \
	REG_GPIO_PXSELC(4) =  0xffffffff;     \
	REG_GPIO_PXDIRC(4) =  0xffffffff;     \
	REG_GPIO_PXPES(4)  =  0xffffffff;     \
	REG_GPIO_PXFUNC(5) =  0xffffffff;     \
	REG_GPIO_PXSELC(5) =  0xffffffff;     \
	REG_GPIO_PXDIRC(5) =  0xffffffff;     \
	REG_GPIO_PXPES(5)  =  0xffffffff;     \
} while (0)

extern void (*jz_timer_callback)(void);

struct wakeup_key_s {
	int gpio;       /* gpio pin number */
	int active_low; /* the key interrupt pin is low voltage 
                           or fall edge acitve */
};

/* add wakeup keys here */
static struct wakeup_key_s wakeup_key[] = {
	{
		.gpio = GPIO_CALL,
		.active_low = ACTIVE_LOW_CALL,
	},
	{
		.gpio = GPIO_HOME,
		.active_low = ACTIVE_LOW_HOME,
	},
	{
		.gpio = GPIO_BACK,
		.active_low = ACTIVE_LOW_BACK,
	},
	{
		.gpio = GPIO_MENU,
		.active_low = ACTIVE_LOW_MENU,
	},
	{
		.gpio = GPIO_ENDCALL,
		.active_low = ACTIVE_LOW_ENDCALL,
	},
	{
		.gpio = GPIO_ADKEY_INT,
		.active_low = ACTIVE_LOW_ADKEY,
	},
};

static void wakeup_key_setup(void)
{
	int i;
	int num = sizeof(wakeup_key) / sizeof(wakeup_key[0]);
 
	for(i = 0; i < num; i++) {
#if 0
		if(wakeup_key[i].active_low)
			__gpio_as_irq_fall_edge(wakeup_key[i].gpio);
		else
			__gpio_as_irq_rise_edge(wakeup_key[i].gpio);
#endif
		__gpio_ack_irq(wakeup_key[i].gpio);
		__gpio_unmask_irq(wakeup_key[i].gpio);
		__intc_unmask_irq(IRQ_GPIO0 - (wakeup_key[i].gpio/32));  /* unmask IRQ_GPIOn */
	}
}

/* NOTES:
 * 1: Pins that are floated (NC) should be set as input and pull-enable.
 * 2: Pins that are pull-up or pull-down by outside should be set as input 
 *    and pull-disable.
 * 3: Pins that are connected to a chip except sdram and nand flash 
 *    should be set as input and pull-disable, too.
 */
void jz_board_do_sleep(unsigned long *ptr)
{
	unsigned char i;
#ifdef DEBUG  
	__intc_unmask_irq(IRQ_UART3);
        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		printk("run dat:%x pin:%x fun:%x sel:%x dir:%x pull:%x msk:%x trg:%x\n", \
			REG_GPIO_PXDAT(i),REG_GPIO_PXPIN(i),REG_GPIO_PXFUN(i),REG_GPIO_PXSEL(i), \
			REG_GPIO_PXDIR(i),REG_GPIO_PXPE(i),REG_GPIO_PXIM(i),REG_GPIO_PXTRG(i));
	}
#endif
        /* Save GPIO registers */
	for(i = 1; i < GPIO_PORT_NUM; i++) {
		*ptr++ = REG_GPIO_PXFUN(i);
		*ptr++ = REG_GPIO_PXSEL(i);
		*ptr++ = REG_GPIO_PXDIR(i);
		*ptr++ = REG_GPIO_PXPE(i);
	}

	/*
	 * Mask the ethernet irq
	 */
//	__gpio_mask_irq(GPIO_NET_INT);

        /*
         * Set all pins to pull-disable, and set all pins as input except 
         * sdram and the pins which can be used as CS1_N to CS4_N for chip select. 
         */
//	 __gpio_as_sleep();

        /*
	 * Set proper status for GPC21 to GPC24 which can be used as CS1_N to CS4_N.
	 * Keep the pins' function used for chip select(CS) here according to your   
         * system to avoid chip select crashing with sdram when resuming from sleep mode.
         */

        /* GPB25/CS1_N is used as chip select for nand flash, shouldn't be change. */ 

        /* GPB26/CS2_N is connected to nand flash, needn't be changed. */

        /* GPB28/CS3_N is used as cs8900's chip select, shouldn't be changed. */
 
	/* GPB27/CS4_N is used as NOR's chip select, shouldn't be changed. */ 

 	/* 
         * Enable pull for NC pins here according to your system 
	 */

	/* 
         * If you must set some GPIOs as output to high level or low level,  
         * you can set them here, using:
         * __gpio_as_output(n);
         * __gpio_set_pin(n); or  __gpio_clear_pin(n);
	 */

	/* AMPEN_N should be set to high to disable audio amplifier */
	__gpio_as_output(GPIO_AMPEN_N);
	__gpio_set_pin(GPIO_AMPEN_N);

#ifdef DEBUG
        /* Keep uart function for printing debug message */
	__gpio_as_uart0();
	__gpio_as_uart1();

        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		printk("sleep dat:%x pin:%x fun:%x sel:%x dir:%x pull:%x msk:%x trg:%x\n", \
			REG_GPIO_PXDAT(i),REG_GPIO_PXPIN(i),REG_GPIO_PXFUN(i),REG_GPIO_PXSEL(i), \
			REG_GPIO_PXDIR(i),REG_GPIO_PXPE(i),REG_GPIO_PXIM(i),REG_GPIO_PXTRG(i));
	}
	__intc_mask_irq(IRQ_UART1);
#endif

	/*
	 * Just allow following interrupts to wakeup the system.
	 */

#ifdef CONFIG_RTC_CLASS
	/* enable RTC alarm */
	__intc_unmask_irq(IRQ_RTC);
#if 0
        /* make system wake up after n seconds by RTC alarm */
	unsigned int v, n;
	n = 10;
	while (!__rtc_write_ready());
	__rtc_enable_alarm();
	while (!__rtc_write_ready());
	__rtc_enable_alarm_irq();
 	while (!__rtc_write_ready());
 	v = __rtc_get_second();
 	while (!__rtc_write_ready());
 	__rtc_set_alarm_second(v+n);
#endif
#endif
        /* setup wakeup keys before sleeping */
	wakeup_key_setup();
}

void jz_board_do_resume(unsigned long *ptr)
{
	unsigned char i;

	/* Restore GPIO registers */
	for(i = 1; i < GPIO_PORT_NUM; i++) {
		 REG_GPIO_PXFUNS(i) = *ptr;
		 REG_GPIO_PXFUNC(i) = ~(*ptr++);

		 REG_GPIO_PXSELS(i) = *ptr;
		 REG_GPIO_PXSELC(i) = ~(*ptr++);

		 REG_GPIO_PXDIRS(i) = *ptr;
		 REG_GPIO_PXDIRC(i) = ~(*ptr++);

		 REG_GPIO_PXPES(i) = *ptr;
		 REG_GPIO_PXPEC(i) = ~(*ptr++);
	}
#ifdef DEBUG 
        /* Print messages of GPIO registers for debug */
	for(i=0;i<GPIO_PORT_NUM;i++) {
		printk("resume dat:%x pin:%x fun:%x sel:%x dir:%x pull:%x msk:%x trg:%x\n", \
			REG_GPIO_PXDAT(i),REG_GPIO_PXPIN(i),REG_GPIO_PXFUN(i),REG_GPIO_PXSEL(i), \
			REG_GPIO_PXDIR(i),REG_GPIO_PXPE(i),REG_GPIO_PXIM(i),REG_GPIO_PXTRG(i));
	}
#endif
}

#if 0
static void dancing(void)
{
	static unsigned char slash[] = "\\|/-";
//	static volatile unsigned char *p = (unsigned char *)0xb6000058;
	static volatile unsigned char *p = (unsigned char *)0xb6000016;
	static unsigned int count = 0;
	*p = slash[count++];
	count &= 3;
}

static void cetus_timer_callback(void)
{
	static unsigned long count = 0;

	if ((++count) % 50 == 0) {
		dancing();
		count = 0;
	}
}
#endif
static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4750d/setup.c.
	 */
}

static void __init board_gpio_setup(void)
{
	/*
	 * Initialize SDRAM pins
	 */
	__lcd_close_backlight();
}

void __init jz_board_setup(void)
{
	printk("JZ4750D CETUS board setup\n");

	board_cpm_setup();
	board_gpio_setup();

	jz_timer_callback = NULL;//cetus_timer_callback;
}

/**
 * Called by arch/mips/kernel/proc.c when 'cat /proc/cpuinfo'.
 * Android requires the 'Hardware:' field in cpuinfo to setup the init.%hardware%.rc.
 */
const char *get_board_type(void)
{
	return "cetus";
}
