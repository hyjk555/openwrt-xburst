/*
 * linux/drivers/misc/jz_sensor.c
 *
 * Virtual device driver with tricky appoach to manage TCSM 
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/major.h>
#include <linux/version.h>

#include <asm/cacheflush.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/thread_info.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>
#include "jz_cim.h"
#include "jz_sensor.h"

unsigned int i2c_addr = 0x60;
unsigned int i2c_clk = 100000;

/* I2C ops to init senser */
void sensor_write_reg(unsigned char reg, unsigned char val)
{
	i2c_open();
	i2c_setclk(i2c_clk);
	i2c_write((i2c_addr >> 1), &val, reg, 1);
	i2c_close();
}

int sensor_write_reg16(unsigned short reg, unsigned char val)
{ 
	int ret;
	i2c_open();
	i2c_setclk(i2c_clk);
	ret = i2c_write_16(i2c_addr >> 1, &val, reg, 1);
	i2c_close();
	return ret;
}

unsigned char sensor_read_reg(unsigned char reg)
{
	unsigned char val;

	i2c_open();
	i2c_setclk(i2c_clk);
	i2c_read((i2c_addr >> 1), &val, reg, 1);
	i2c_close();
	return val;
}

/*
 * Get sensor register through i2c bus
 */

unsigned char sensor_read_reg16(unsigned short reg)
{
	unsigned char val;
	i2c_open();
	i2c_setclk(i2c_clk);
	i2c_read_16(i2c_addr >> 1, &val, reg, 1);
	i2c_close();
	return val;
}

