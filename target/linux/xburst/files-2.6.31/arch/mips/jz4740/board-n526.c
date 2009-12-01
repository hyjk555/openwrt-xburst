/*
 *  Copyright (C) 2009, Lars-Peter Clausen <lars@metafoo.de>
 *  	N526 eBook reader support
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>

#include <asm/mach-jz4740/platform.h>

#include <linux/mtd/jz4740_nand.h>
#include <linux/jz4740_fb.h>
#include <linux/power_supply.h>
#include <linux/power/jz4740-battery.h>
#include <linux/mmc/jz4740_mmc.h>

/* NAND */
static struct nand_ecclayout n526_ecclayout = {
	.eccbytes = 36,
	.eccpos = {
		6,  7,  8,  9,  10, 11, 12, 13,
		14, 15, 16, 17, 18, 19, 20, 21,
		22, 23, 24, 25, 26, 27, 28, 29,
		30, 31, 32, 33, 34, 35, 36, 37,
		38, 39, 40, 41},
	.oobfree = {
		{.offset = 2,
		 .length = 4},
		{.offset = 42,
		 .length = 22}}
};

static struct mtd_partition n526_partitions[] = {
	{ .name = "NAND BOOT partition",
	  .offset = 0 * 0x100000,
	  .size = 4 * 0x100000,
 	},
	{ .name = "NAND KERNEL partition",
	  .offset = 4 * 0x100000,
	  .size = 4 * 0x100000,
 	},
	{ .name = "NAND ROOTFS partition",
	  .offset = 8 * 0x100000,
	  .size = 504 * 0x100000,
 	},
	{ .name = "NAND DATA partition",
	  .offset = 512 * 0x100000,
	  .size = 512 * 0x100000,
 	},
};

static struct jz_nand_platform_data n526_nand_pdata = {
	.ecc_layout = &n526_ecclayout,
	.partitions = n526_partitions,
	.num_partitions = ARRAY_SIZE(n526_partitions),
	.busy_gpio = 94,
};


/* Battery */
/*static struct jz_batt_info n526_battery_pdata = {
	.dc_dect_gpio	= GPIO_DC_DETE_N,
	.usb_dect_gpio	= GPIO_USB_DETE,
	.charg_stat_gpio  = GPIO_CHARG_STAT_N,

	.min_voltag	= 3600000,
	.max_voltag	= 4200000,
	.batt_tech	= POWER_SUPPLY_TECHNOLOGY_LIPO,
};*/


static struct jz4740_mmc_platform_data n526_mmc_pdata = {
	.gpio_card_detect	= JZ_GPIO_PORTD(7),
	.card_detect_active_low = 1,
	.gpio_read_only		= -1,
	.gpio_power		= JZ_GPIO_PORTD(17),
	.power_active_low = 1,
};

static struct gpio_led n526_leds[] = {
	{
		.name = "n526:blue:power",
		.gpio = JZ_GPIO_PORTD(28),
		.default_state = LEDS_GPIO_DEFSTATE_ON,
	}
};

static struct gpio_led_platform_data n526_leds_pdata = {
	.leds = n526_leds,
	.num_leds = ARRAY_SIZE(n526_leds),
};

static struct platform_device n526_leds_device = {
	.name = "leds-gpio",
	.id = -1,
	.dev = {
		.platform_data = &n526_leds_pdata,
	},
};

static struct platform_device *jz_platform_devices[] __initdata = {
	&jz4740_usb_ohci_device,
	&jz4740_usb_gdt_device,
	&jz4740_mmc_device,
	&jz4740_nand_device,
	&jz4740_i2s_device,
	&jz4740_codec_device,
	&jz4740_rtc_device,
	&n526_leds_device,
};

static void __init board_gpio_setup(void)
{
	/* We only need to enable/disable pullup here for pins used in generic
	 * drivers. Everything else is done by the drivers themselfs. */
	jz_gpio_disable_pullup(JZ_GPIO_PORTD(17));
	jz_gpio_enable_pullup(JZ_GPIO_PORTD(7));
}

static int __init n526_init_platform_devices(void)
{
	jz4740_nand_device.dev.platform_data = &n526_nand_pdata;
/*	jz4740_battery_device.dev.platform_data = &n526_battery_pdata;*/
	jz4740_mmc_device.dev.platform_data = &n526_mmc_pdata;

	return platform_add_devices(jz_platform_devices,
					ARRAY_SIZE(jz_platform_devices));

}
extern int jz_gpiolib_init(void);
extern int jz_init_clocks(unsigned long extal);

static int __init n526_board_setup(void)
{
	if (jz_gpiolib_init())
		panic("Failed to initalize jz gpio\n");
	jz_init_clocks(12000000);

	board_gpio_setup();

	if (n526_init_platform_devices())
		panic("Failed to initalize platform devices\n");

	return 0;
}

arch_initcall(n526_board_setup);
