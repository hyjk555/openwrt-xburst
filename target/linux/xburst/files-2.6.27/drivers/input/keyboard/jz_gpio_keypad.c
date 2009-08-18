/*
 * linux/drivers/input/keyboard/jz_gpio_keys.c
 *
 * Keypad driver based on GPIO pins for Jz4750 APUS board.
 *
 * User applications can access to this device via /dev/input/eventX.
 *
 * Copyright (c) 2005 - 2009  Ingenic Semiconductor Inc.
 *
 * Author: Richard <cjfeng@ingenic.cn>
 *         Regen   <lhhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <asm/gpio.h>
#include <asm/jzsoc.h>

#undef DEBUG
//#define DEBUG 
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

/* Device name */
#ifdef CONFIG_JZ4750_APUS
#define DEV_NAME "apus-keypad"
#else
#ifdef CONFIG_JZ4750D_CETUS
#define DEV_NAME "cetus-keypad"
#else
#define DEV_NAME "jz-keypad"
#endif
#endif

/* Key codes */
#define ANDROID_MENU        KEY_MENU
#define ANDROID_CALL        KEY_SEND
#define ANDROID_HOME        KEY_HOME
#define ANDROID_ENDCALL     KEY_END
#define ANDROID_BACK        KEY_BACK

/* Timer interval */
#define SCAN_INTERVAL       5

/*
 * GPIO Buttons
 */
static struct gpio_keys_button board_buttons[] = {
#if 0
	{
		.gpio		= GPIO_WAKEUP,
		.code        	= KEY_1,
		.desc		= "Button 0",
		.active_low	= 1,
	},
#endif
	{
		.gpio		= GPIO_CALL,
		.code   	= ANDROID_CALL,
		.desc		= "Button 1",
		.active_low	= ACTIVE_LOW_CALL,
	},
	{
		.gpio		= GPIO_HOME,
		.code   	= ANDROID_HOME,
		.desc		= "Button 2",
		.active_low	= ACTIVE_LOW_HOME,
	},
	{
		.gpio		= GPIO_BACK,
		.code   	= ANDROID_BACK,
		.desc		= "Button 3",
		.active_low	= ACTIVE_LOW_BACK,
	},
	{
		.gpio		= GPIO_MENU,
		.code   	= ANDROID_MENU,
		.desc		= "Button 4",
		.active_low	= ACTIVE_LOW_MENU,
	},
	{
		.gpio		= GPIO_ENDCALL,
		.code   	= ANDROID_ENDCALL,
		.desc		= "Button 5",
		.active_low	= ACTIVE_LOW_ENDCALL,
	},
#if 0
	{
		.gpio		= GPIO_SW7,
		.code   	= KEY_7,
		.desc		= "Button 6",
		.active_low	= 1,
	},
#endif
};

static struct timer_list button_timer;
static int button_no;

static struct gpio_keys_platform_data board_button_data = {
	.buttons	= board_buttons,
	.nbuttons	= ARRAY_SIZE(board_buttons),
};

static struct platform_device board_button_device = {
	.name		= DEV_NAME,
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &board_button_data,
	}
};

static void enable_gpio_irqs(struct gpio_keys_platform_data *pdata)
{
	int i;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];

		if (button->active_low)
			__gpio_as_irq_fall_edge(button->gpio);
		else
			__gpio_as_irq_rise_edge(button->gpio);
	}
}

static void button_timer_callback(unsigned long data)
{
	int gpio = board_buttons[button_no].gpio;
	int code = board_buttons[button_no].code;
	int active_low = board_buttons[button_no].active_low;
	struct platform_device *pdev = (struct platform_device *)data;
	struct input_dev *input = platform_get_drvdata(pdev);
	int state;
	static int button_pressed = 0;

	state = __gpio_get_pin(gpio);

        if (active_low) {
		if (state == 0) {
			/* button pressed */
			button_pressed = 1;
			input_report_key(input, code, 1);
			input_sync(input);
			mod_timer(&button_timer, jiffies + SCAN_INTERVAL);
			dprintk("gpio %d down, code:%d \n", gpio, code);
		} else {
			/* button released */
			if (button_pressed) {
				input_report_key(input, code, 0);
				input_sync(input);
				button_pressed = 0;
				dprintk("gpio %d up, code:%d \n", gpio, code);
			}
			__gpio_as_irq_fall_edge(gpio);
		}
	} else {
		if (state == 1) {
			/* button pressed */
			button_pressed = 1;
			input_report_key(input, code, 1);
			input_sync(input);
			mod_timer(&button_timer, jiffies + SCAN_INTERVAL);
			dprintk("gpio %d down code:%d \n", gpio, code);
		} else {
			/* button released */
			if (button_pressed) {
				input_report_key(input, code, 0);
				input_sync(input);
				dprintk("gpio %d up code:%d \n", gpio, code);
			}
			__gpio_as_irq_rise_edge(gpio);
		}
	}
}

static irqreturn_t jz_gpio_interrupt(int irq, void *dev_id)
{
	int i;
	struct platform_device *pdev = dev_id;
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;

	dprintk("--irq of gpio:%d\n", irq - IRQ_GPIO_0);

	__gpio_ack_irq(irq - IRQ_GPIO_0);  /* clear flag */

	if (!timer_pending(&button_timer)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			int gpio = button->gpio;
			
			if (irq == (gpio + IRQ_GPIO_0) ) {
				/* start timer */
				__gpio_as_input(gpio);
				button_no = i;
				mod_timer(&button_timer, jiffies + SCAN_INTERVAL);
				dprintk("--mod_timer for gpio:%d\n", gpio);
				break;
			}
		}
	}

	return IRQ_HANDLED;
}

static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	platform_set_drvdata(pdev, input);

	input->name = pdev->name;
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	input->evbit[0] = BIT(EV_KEY) | BIT(EV_SYN);

	set_bit(ANDROID_MENU, input->keybit);
	set_bit(ANDROID_HOME, input->keybit);
	set_bit(ANDROID_CALL, input->keybit);
	set_bit(ANDROID_BACK, input->keybit);
	set_bit(ANDROID_ENDCALL, input->keybit);

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		int irq;
		unsigned int type = button->type ?: EV_KEY;

		irq = IRQ_GPIO_0 + button->gpio;
		if (irq < 0) {
			error = irq;
			pr_err("%s: Unable to get irq number"
			       " for GPIO %d, error %d\n", DEV_NAME,
				button->gpio, error);
			goto fail;
		}

		error = request_irq(irq, jz_gpio_interrupt,
				    IRQF_SAMPLE_RANDOM | IRQF_DISABLED,
				    button->desc ? button->desc : "gpio_keys",
				    pdev);
		if (error) {
			pr_err("%s: Unable to claim irq %d; error %d\n",
			       DEV_NAME, irq, error);
			goto fail;
		}

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, type, button->code);
	}

	/* Enable all GPIO irqs */
	enable_gpio_irqs(pdata);

	/* Init timer */
	init_timer(&button_timer);
	button_timer.data = (unsigned long)&board_button_device;
	button_timer.function = button_timer_callback;

	error = input_register_device(input);
	if (error) {
		pr_err("%s: Unable to register input device, "
		       "error: %d\n", DEV_NAME, error);
		goto fail;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail:
	while (--i >= 0) {
		free_irq(pdata->buttons[i].gpio + IRQ_GPIO_0 , pdev);
	}

	platform_set_drvdata(pdev, NULL);
	input_free_device(input);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input = platform_get_drvdata(pdev);
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = pdata->buttons[i].gpio + IRQ_GPIO_0;
		free_irq(irq, pdev);
	}

	input_unregister_device(input);

	return 0;
}

#ifdef CONFIG_PM

static int gpio_keys_suspend(struct platform_device *pdev, pm_message_t state)
{
#if 0
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = button->gpio + IRQ_GPIO_0;
				enable_irq_wake(irq);
			}
		}
	}
#endif
	return 0;
}

static int gpio_keys_resume(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
#if 0
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {
				int irq = button->gpio + IRQ_GPIO_0;
				disable_irq_wake(irq);
			}
		}
	}
#endif

	/* Enable all GPIO irqs */
	enable_gpio_irqs(pdata);

	return 0;
}
#else
#define gpio_keys_suspend	NULL
#define gpio_keys_resume	NULL
#endif

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
	.driver		= {
		.name	= DEV_NAME,
	}
};

static int __init gpio_keys_init(void)
{
	int ret;

	platform_device_register(&board_button_device);
	ret = platform_driver_register(&gpio_keys_device_driver);

	return ret;
}

static void __exit gpio_keys_exit(void)
{
	platform_device_unregister(&board_button_device);
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Regen Huang <lhhuang@ingenic.cn>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
