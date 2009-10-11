/*
 * linux/arch/mips/jz4740/board-qi_lb60.c
 *
 * QI_LB60 setup routines.
 *
 * Copyright (c) 2009 Qi Hardware inc.,
 * Author: Xiangfu Liu <xiangfu@qi-hardware.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>

#include <asm/mach-jz4740/board-qi_lb60.h>


static void __init board_gpio_setup(void)
{
	/* We only need to enable/disable pullup here for pins used in generic
	 * drivers. Everything else is done by the drivers themselfs. */
	jz_gpio_disable_pullup(GPIO_SD_VCC_EN_N);
	jz_gpio_disable_pullup(GPIO_SD_CD_N);
	jz_gpio_disable_pullup(GPIO_SD_WP);
}

void __init jz_board_setup(void)
{
	printk("Qi Hardware JZ4740 QI_LB60 setup\n");

	board_gpio_setup();

}
