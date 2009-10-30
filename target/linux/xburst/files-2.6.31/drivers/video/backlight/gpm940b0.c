/*
 *  Copyright (C) 2009, Lars-Peter Clausen <lars@metafoo.de>
 *  	JZ4720/JZ4740 SoC LCD framebuffer driver
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

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/lcd.h>
#include <linux/backlight.h>

struct gpm940b0 {
	struct spi_device *spi;
	struct lcd_device *lcd;
	struct backlight_device *bl;
};

static int gpm940b0_write_reg(struct spi_device *spi, uint8_t reg,
				uint8_t data)
{
	uint8_t buf[2];
	buf[0] = ((reg & 0x40) << 1) | (reg & 0x3f);
	buf[1] = data;

	return spi_write(spi, buf, sizeof(buf));
}

static int gpm940b0_set_power(struct lcd_device *lcd, int power)
{
	struct gpm940b0 *gpm940b0 = lcd_get_data(lcd);

	switch (power) {
	case FB_BLANK_UNBLANK:
	    gpm940b0_write_reg(gpm940b0->spi, 0x05, 0x5f);
	    break;
	default:
	    gpm940b0_write_reg(gpm940b0->spi, 0x05, 0x5e);
	    break;
	}
	return 0;
}

static int gpm940b0_set_contrast(struct lcd_device *lcd, int contrast)
{
	struct gpm940b0 *gpm940b0 = lcd_get_data(lcd);
	gpm940b0_write_reg(gpm940b0->spi, 0x0d, contrast);
	return 0;
}

static int gpm940b0_set_mode(struct lcd_device *lcd, struct fb_videomode *mode)
{
	if (mode->xres != 320 && mode->yres != 240)
		return -EINVAL;

	return 0;
}

int gpm940b0_bl_update_status(struct backlight_device *bl)
{
	struct gpm940b0 *gpm940b0 = bl_get_data(bl);

    /* The datasheet suggest that this is not possible on the giantplus module 
	gpm940b0_write_reg(gpm940b0->spi, 21, bl->props.brightness);*/

	return 0;
}

static size_t reg_write(struct device *dev, struct device_attribute *attr,
                        const char *buf, size_t count)
{
	char *buf2;
	uint32_t reg = simple_strtoul(buf, &buf2, 10);
	uint32_t val = simple_strtoul(buf2 + 1, NULL, 10);
	struct gpm940b0 *gpm940b0 = dev_get_drvdata(dev);

	if (reg < 0 || val < 0)
		return -EINVAL;

	gpm940b0_write_reg(gpm940b0->spi, reg, val);
	return count;
}

static DEVICE_ATTR(reg, 0644, NULL, reg_write);

static struct lcd_ops gpm940b0_lcd_ops = {
	.set_power = gpm940b0_set_power,
	.set_contrast = gpm940b0_set_contrast,
	.set_mode = gpm940b0_set_mode,
};

static struct backlight_ops gpm940b0_bl_ops = {
/*	.get_brightness	= gpm940b0_bl_get_brightness,*/
	.update_status	= gpm940b0_bl_update_status,
};

static int __devinit gpm940b0_probe(struct spi_device *spi)
{
	int ret;
	struct gpm940b0 *gpm940b0;

	gpm940b0 = kmalloc(sizeof(*gpm940b0), GFP_KERNEL);

	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "Failed to setup spi\n");
		goto err_free_gpm940b0;
	}

	gpm940b0->spi = spi;

	gpm940b0->lcd = lcd_device_register("gpm940b0-lcd", &spi->dev, gpm940b0,
					    &gpm940b0_lcd_ops);

	if (IS_ERR(gpm940b0->lcd)) {
		ret = PTR_ERR(gpm940b0->lcd);
		dev_err(&spi->dev, "Failed to register lcd device: %d\n", ret);
		goto err_free_gpm940b0;
	}

	gpm940b0->lcd->props.max_contrast = 255;

	gpm940b0->bl = backlight_device_register("gpm940b0-bl", &spi->dev, gpm940b0,
						 &gpm940b0_bl_ops);

	if (IS_ERR(gpm940b0->bl)) {
		ret = PTR_ERR(gpm940b0->bl);
		dev_err(&spi->dev, "Failed to register backlight device: %d\n", ret);
		gpm940b0->bl = NULL;
	} else {
		gpm940b0->bl->props.max_brightness = 15;
		gpm940b0->bl->props.brightness = 0;
		gpm940b0->bl->props.power = FB_BLANK_UNBLANK;
	}
	device_create_file(&spi->dev, &dev_attr_reg);

	dev_set_drvdata(&spi->dev, gpm940b0);

	gpm940b0_write_reg(spi, 0x13, 0x01);
	gpm940b0_write_reg(spi, 0x05, 0x5f);
	return 0;
err_free_gpm940b0:
	return ret;
}

static int __devexit gpm940b0_remove(struct spi_device *spi)
{
	struct gpm940b0 *gpm940b0 = spi_get_drvdata(spi);
	if (gpm940b0->bl)
		backlight_device_unregister(gpm940b0->bl);

	lcd_device_unregister(gpm940b0->lcd);

	spi_set_drvdata(spi, NULL);
	kfree(gpm940b0);
	return 0;
}

static struct spi_driver gpm940b0_driver = {
	.driver = {
		.name = "gpm940b0",
		.owner = THIS_MODULE,
	},
	.probe = gpm940b0_probe,
	.remove = __devexit_p(gpm940b0_remove),
};

static int __init gpm940b0_init(void)
{
	return spi_register_driver(&gpm940b0_driver);
}
module_init(gpm940b0_init);

static void __exit gpm940b0_exit(void)
{
	return spi_unregister_driver(&gpm940b0_driver);
}
module_exit(gpm940b0_exit)

MODULE_AUTHOR("Lars-Peter Clausen");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("LCD and backlight controll for Giantplus GPM940B0");
MODULE_ALIAS("i2c:gpm940b0");
