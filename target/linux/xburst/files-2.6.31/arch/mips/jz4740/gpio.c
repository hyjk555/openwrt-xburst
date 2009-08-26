/*
 *  Copyright (C) 2009, Lars-Peter Clausen <lars@metafoo.de>
 *  	JZ74xx platform GPIO support
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>

#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>

#include <asm/mach-jz4740/regs.h>

#define JZ_GPIO_BASE_A (32*0)
#define JZ_GPIO_BASE_B (32*1)
#define JZ_GPIO_BASE_C (32*2)
#define JZ_GPIO_BASE_D (32*3)

#define JZ_GPIO_NUM_A 32
#define JZ_GPIO_NUM_B 32
#define JZ_GPIO_NUM_C 31
#define JZ_GPIO_NUM_D 32

#define JZ_IRQ_GPIO_BASE_A JZ_IRQ_GPIO(0) + JZ_GPIO_BASE_A
#define JZ_IRQ_GPIO_BASE_B JZ_IRQ_GPIO(0) + JZ_GPIO_BASE_B
#define JZ_IRQ_GPIO_BASE_C JZ_IRQ_GPIO(0) + JZ_GPIO_BASE_C
#define JZ_IRQ_GPIO_BASE_D JZ_IRQ_GPIO(0) + JZ_GPIO_BASE_D

#define JZ_IRQ_GPIO_A(num) (num < JZ_GPIO_NUM_A ? JZ_IRQ_GPIO_BASE_A + num : -EINVAL)
#define JZ_IRQ_GPIO_B(num) (num < JZ_GPIO_NUM_B ? JZ_IRQ_GPIO_BASE_B + num : -EINVAL)
#define JZ_IRQ_GPIO_C(num) (num < JZ_GPIO_NUM_C ? JZ_IRQ_GPIO_BASE_C + num : -EINVAL)
#define JZ_IRQ_GPIO_D(num) (num < JZ_GPIO_NUM_D ? JZ_IRQ_GPIO_BASE_D + num : -EINVAL)


#define CHIP_TO_REG(chip, reg) (jz_gpio_base + (((chip)->base) << 3) + reg)
#define CHIP_TO_PIN_REG(chip)			CHIP_TO_REG(chip, 0x00)
#define CHIP_TO_DATA_REG(chip)			CHIP_TO_REG(chip, 0x10)
#define CHIP_TO_DATA_SET_REG(chip)		CHIP_TO_REG(chip, 0x14)
#define CHIP_TO_DATA_CLEAR_REG(chip)		CHIP_TO_REG(chip, 0x18)
#define CHIP_TO_PULL_REG(chip)			CHIP_TO_REG(chip, 0x30)
#define CHIP_TO_PULL_SET_REG(chip)		CHIP_TO_REG(chip, 0x34)
#define CHIP_TO_PULL_CLEAR_REG(chip)		CHIP_TO_REG(chip, 0x38)
#define CHIP_TO_DATA_SELECT_REG(chip)		CHIP_TO_REG(chip, 0x50)
#define CHIP_TO_DATA_SELECT_SET_REG(chip)	CHIP_TO_REG(chip, 0x54)
#define CHIP_TO_DATA_SELECT_CLEAR_REG(chip)	CHIP_TO_REG(chip, 0x58)
#define CHIP_TO_DATA_DIRECION_REG(chip)		CHIP_TO_REG(chip, 0x60)
#define CHIP_TO_DATA_DIRECTION_SET_REG(chip)	CHIP_TO_REG(chip, 0x64)
#define CHIP_TO_DATA_DIRECTION_CLEAR_REG(chip)	CHIP_TO_REG(chip, 0x68)

#define GPIO_TO_BIT(gpio) BIT(gpio & 0x1f)

#define GPIO_TO_REG(gpio, reg) (jz_gpio_base + ((gpio >> 5) << 8) + reg)
#define GPIO_TO_PULL_REG(gpio)		GPIO_TO_REG(gpio, 0x30)
#define GPIO_TO_PULL_SET_REG(gpio)	GPIO_TO_REG(gpio, 0x34)
#define GPIO_TO_PULL_CLEAR_REG(gpio)	GPIO_TO_REG(gpio, 0x38)
#define GPIO_TO_FUNC_REG(gpio)		GPIO_TO_REG(gpio, 0x40)
#define GPIO_TO_FUNC_SET_REG(gpio)	GPIO_TO_REG(gpio, 0x44)
#define GPIO_TO_FUNC_CLEAR_REG(gpio)	GPIO_TO_REG(gpio, 0x48)
#define GPIO_TO_SEL_REG(gpio)		GPIO_TO_REG(gpio, 0x50)
#define GPIO_TO_SEL_SET_REG(gpio)	GPIO_TO_REG(gpio, 0x54)
#define GPIO_TO_SEL_CLEAR_REG(gpio)	GPIO_TO_REG(gpio, 0x58)
#define GPIO_TO_TRIGGER_REG(gpio)	GPIO_TO_REG(gpio, 0x70)
#define GPIO_TO_TRIGGER_SET_REG(gpio)	GPIO_TO_REG(gpio, 0x74)
#define GPIO_TO_TRIGGER_CLEAR_REG(gpio)	GPIO_TO_REG(gpio, 0x78)




static void __iomem *jz_gpio_base;
static spinlock_t jz_gpio_lock;

struct jz_gpio_chip {
	unsigned int irq_base;
	struct gpio_chip gpio_chip;
	struct irq_chip irq_chip;
};

int jz_gpio_set_function(int gpio, enum jz_gpio_function function)
{
	if (function == JZ_GPIO_FUNC_NONE) {
		writew(GPIO_TO_BIT(gpio), GPIO_TO_FUNC_CLEAR_REG(gpio));
		writew(GPIO_TO_BIT(gpio), GPIO_TO_SEL_CLEAR_REG(gpio));
		writew(GPIO_TO_BIT(gpio), GPIO_TO_TRIGGER_CLEAR_REG(gpio));
	} else {
		writew(GPIO_TO_BIT(gpio), GPIO_TO_FUNC_SET_REG(gpio));
		switch (function) {
		case JZ_GPIO_FUNC1:
			writew(GPIO_TO_BIT(gpio), GPIO_TO_SEL_CLEAR_REG(gpio));
			break;
		case JZ_GPIO_FUNC3:
			writew(GPIO_TO_BIT(gpio), GPIO_TO_TRIGGER_SET_REG(gpio));
		case JZ_GPIO_FUNC2: /* Falltrough */
			writew(GPIO_TO_BIT(gpio), GPIO_TO_SEL_SET_REG(gpio));
			break;
		default:
		    BUG();
		    break;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(jz_gpio_set_function);

int jz_gpio_bulk_request(const struct jz_gpio_bulk_request *request, size_t num)
{
	size_t i;
	int ret;

	for (i = 0; i < num; ++i, ++request) {
		ret = gpio_request(request->gpio, request->name);
		if (ret)
			goto err;
		jz_gpio_set_function(request->gpio, request->function);
	}

	return 0;
err:
	for (--request; i > 0; --i, --request)
		gpio_free(request->gpio);

	return ret;
}
EXPORT_SYMBOL_GPL(jz_gpio_bulk_request);

void jz_gpio_bulk_free(const struct jz_gpio_bulk_request *request, size_t num)
{
	size_t i;

	for (i = 0; i < num; ++i, ++request) {
		gpio_free(request->gpio);
		jz_gpio_set_function(request->gpio, JZ_GPIO_FUNC_NONE);
	}

}
EXPORT_SYMBOL_GPL(jz_gpio_bulk_free);

void jz_gpio_enable_pullup(unsigned gpio)
{
	writel(GPIO_TO_BIT(gpio), GPIO_TO_PULL_CLEAR_REG(gpio));
}
EXPORT_SYMBOL_GPL(jz_gpio_enable_pullup);

void jz_gpio_disable_pullup(unsigned gpio)
{
	writel(GPIO_TO_BIT(gpio), GPIO_TO_PULL_SET_REG(gpio));
}
EXPORT_SYMBOL_GPL(jz_gpio_disable_pullup);

static int jz_gpio_get_value(struct gpio_chip *chip, unsigned gpio)
{
	return !!(readl(CHIP_TO_PIN_REG(chip)) & BIT(gpio));
}

static void jz_gpio_set_value(struct gpio_chip *chip, unsigned gpio, int value)
{
	uint32_t __iomem *reg = CHIP_TO_DATA_SET_REG(chip) + ((!value) << 2);
	writel(BIT(gpio), reg);
}

static int jz_gpio_direction_output(struct gpio_chip *chip, unsigned gpio, int value)
{
	writel(BIT(gpio), CHIP_TO_DATA_DIRECTION_SET_REG(chip));
	jz_gpio_set_value(chip, gpio, value);

	return 0;
}

static int jz_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	writel(BIT(gpio), CHIP_TO_DATA_DIRECTION_CLEAR_REG(chip));

	return 0;
}


#define IRQ_TO_GPIO(irq) (irq - JZ_IRQ_GPIO(0))
#define IRQ_TO_BIT(irq) BIT(IRQ_TO_GPIO(irq) & 0x1f)


#define IRQ_TO_REG(irq, reg)  GPIO_TO_REG(IRQ_TO_GPIO(irq), reg)
#define IRQ_TO_MASK_REG(irq)		IRQ_TO_REG(irq, 0x20)
#define IRQ_TO_MASK_SET_REG(irq)	IRQ_TO_REG(irq, 0x24)
#define IRQ_TO_MASK_CLEAR_REG(irq)	IRQ_TO_REG(irq, 0x28)
#define IRQ_TO_SELECT_REG(irq)		IRQ_TO_REG(irq, 0x50)
#define IRQ_TO_SELECT_SET_REG(irq)	IRQ_TO_REG(irq, 0x54)
#define IRQ_TO_SELECT_CLEAR_REG(irq)	IRQ_TO_REG(irq, 0x58)
#define IRQ_TO_DIRECTION_REG(irq)	IRQ_TO_REG(irq, 0x60)
#define IRQ_TO_DIRECTION_SET_REG(irq)	IRQ_TO_REG(irq, 0x64)
#define IRQ_TO_DIRECTION_CLEAR_REG(irq) IRQ_TO_REG(irq, 0x68)
#define IRQ_TO_TRIGGER_REG(irq)		IRQ_TO_REG(irq, 0x70)
#define IRQ_TO_TRIGGER_SET_REG(irq)	IRQ_TO_REG(irq, 0x74)
#define IRQ_TO_TRIGGER_CLEAR_REG(irq)	IRQ_TO_REG(irq, 0x78)
#define IRQ_TO_FLAG_REG(irq)		IRQ_TO_REG(irq, 0x80)
#define IRQ_TO_FLAG_CLEAR_REG(irq)	IRQ_TO_REG(irq, 0x14)


static void jz_gpio_irq_demux_handler(unsigned int irq, struct irq_desc *desc)
{
	uint32_t flag;
	unsigned int gpio_irq;
	unsigned int gpio_bank;

	gpio_bank = IRQ_GPIO0 - irq;

	flag = readl(jz_gpio_base + (gpio_bank << 8) + 0x80);

	gpio_irq = ffs(flag);

	gpio_irq += (gpio_bank << 5) + JZ_IRQ_GPIO(0) - 1;

	generic_handle_irq(gpio_irq);
};

/* TODO: Check if function is gpio */
static unsigned int jz_gpio_irq_startup(unsigned int irq)
{
	writel(IRQ_TO_BIT(irq), IRQ_TO_SELECT_SET_REG(irq));
	spin_lock(&jz_gpio_lock);
	writel(IRQ_TO_BIT(irq), IRQ_TO_MASK_CLEAR_REG(irq));
	spin_unlock(&jz_gpio_lock);
	return 0;
}

static void jz_gpio_irq_shutdown(unsigned int irq)
{
	spin_lock(&jz_gpio_lock);
	writel(IRQ_TO_BIT(irq), IRQ_TO_MASK_SET_REG(irq));
	spin_unlock(&jz_gpio_lock);
	/* Set direction to input */
	writel(IRQ_TO_BIT(irq), IRQ_TO_DIRECTION_CLEAR_REG(irq));
	writel(IRQ_TO_BIT(irq), IRQ_TO_SELECT_CLEAR_REG(irq));
}

static void jz_gpio_irq_mask(unsigned int irq)
{
	writel(IRQ_TO_BIT(irq), IRQ_TO_MASK_SET_REG(irq));
};

static void jz_gpio_irq_unmask(unsigned int irq)
{
	writel(IRQ_TO_BIT(irq), IRQ_TO_MASK_CLEAR_REG(irq));
};

static void jz_gpio_irq_ack(unsigned int irq)
{
	writel(IRQ_TO_BIT(irq), IRQ_TO_FLAG_CLEAR_REG(irq));
};

static int jz_gpio_irq_set_type(unsigned int irq, unsigned int flow_type)
{
	uint32_t mask;
	spin_lock(&jz_gpio_lock);

	mask = readl(IRQ_TO_MASK_REG(irq));

	writel(IRQ_TO_BIT(irq), IRQ_TO_MASK_CLEAR_REG(irq));

	switch(flow_type) {
	case IRQ_TYPE_EDGE_RISING:
		writel(IRQ_TO_BIT(irq), IRQ_TO_DIRECTION_SET_REG(irq));
		writel(IRQ_TO_BIT(irq), IRQ_TO_TRIGGER_SET_REG(irq));
		break;
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_EDGE_BOTH:
		writel(IRQ_TO_BIT(irq), IRQ_TO_DIRECTION_CLEAR_REG(irq));
		writel(IRQ_TO_BIT(irq), IRQ_TO_TRIGGER_SET_REG(irq));
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		writel(IRQ_TO_BIT(irq), IRQ_TO_DIRECTION_SET_REG(irq));
		writel(IRQ_TO_BIT(irq), IRQ_TO_TRIGGER_CLEAR_REG(irq));
		break;
	case IRQ_TYPE_LEVEL_LOW:
		writel(IRQ_TO_BIT(irq), IRQ_TO_DIRECTION_CLEAR_REG(irq));
		writel(IRQ_TO_BIT(irq), IRQ_TO_TRIGGER_CLEAR_REG(irq));
		break;
	default:
		return -EINVAL;
	}

	writel(mask, IRQ_TO_MASK_SET_REG(irq));

	spin_unlock(&jz_gpio_lock);

	return 0;
}

int gpio_to_irq(unsigned gpio)
{
	return JZ_IRQ_GPIO(0) + gpio;
}
EXPORT_SYMBOL_GPL(gpio_to_irq);

int irq_to_gpio(unsigned gpio)
{
    return IRQ_TO_GPIO(gpio);
}
EXPORT_SYMBOL_GPL(irq_to_gpio);

#define JZ_GPIO_CHIP(_bank) { \
	.irq_base = JZ_IRQ_GPIO_BASE_ ## _bank, \
	.gpio_chip = { \
		.label = "Bank " # _bank, \
		.owner = THIS_MODULE, \
		.set = jz_gpio_set_value, \
		.get = jz_gpio_get_value, \
		.direction_output = jz_gpio_direction_output, \
		.direction_input = jz_gpio_direction_input, \
		.base = JZ_GPIO_BASE_ ## _bank, \
		.ngpio = JZ_GPIO_NUM_ ## _bank, \
	}, \
	.irq_chip =  { \
		.name = "GPIO Bank " # _bank, \
		.mask = jz_gpio_irq_mask, \
		.unmask = jz_gpio_irq_unmask, \
		.ack = jz_gpio_irq_ack, \
		.startup = jz_gpio_irq_startup, \
		.shutdown = jz_gpio_irq_shutdown, \
		.set_type = jz_gpio_irq_set_type, \
	}, \
}

static struct jz_gpio_chip jz_gpio_chips[] = {
	JZ_GPIO_CHIP(A),
	JZ_GPIO_CHIP(B),
	JZ_GPIO_CHIP(C),
	JZ_GPIO_CHIP(D),
};

__init int jz_gpiolib_init(void)
{
	struct jz_gpio_chip *chip = jz_gpio_chips;
	int i, irq;

	jz_gpio_base = ioremap(0x10010000, 0x400);

	for (i = 0; i < ARRAY_SIZE(jz_gpio_chips); ++i, ++chip) {
		gpiochip_add(&chip->gpio_chip);
		set_irq_chained_handler(JZ_IRQ_INTC_GPIO(i), jz_gpio_irq_demux_handler);
		for (irq = chip->irq_base; irq < chip->irq_base + chip->gpio_chip.ngpio;
		++irq)
			set_irq_chip_and_handler(irq, &chip->irq_chip, handle_level_irq);
	}

	printk("JZ GPIO initalized\n");

	return 0;
}

arch_initcall(jz_gpiolib_init);

