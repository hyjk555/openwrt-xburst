/*
 * Battery measurement code for Ingenic JZ SOC.
 *
 * based on tosa_battery.c
 *
 * Copyright (C) 2008 Marek Vasut <marek.vasut@gmail.com>
 * Copyright (C) 2009 Jiejing Zhang <kzjeef@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <linux/power/jz4740-battery.h>
#include <linux/jz4740-adc.h>

/*struct jz_battery {*/
	static int bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
	static struct jz_batt_info *pdata = 0;

	struct mutex work_lock;

	struct workqueue_struct *monitor_wqueue;
	struct delayed_work bat_work;
/*};*/


/*********************************************************************
 *		Power
 *********************************************************************/

static int jz_get_power_prop(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int gpio = (psy->type == POWER_SUPPLY_TYPE_MAINS) ? pdata->dc_dect_gpio :
														pdata->usb_dect_gpio;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !gpio_get_value(gpio);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property jz_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply jz_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = jz_power_props,
	.num_properties = ARRAY_SIZE(jz_power_props),
	.get_property = jz_get_power_prop,
};

static struct power_supply jz_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = jz_power_props,
	.num_properties = ARRAY_SIZE(jz_power_props),
	.get_property = jz_get_power_prop,
};


/*********************************************************************
 *		Battery properties
 *********************************************************************/

static long jz_read_bat(struct power_supply *bat_ps)
{
	enum jz_adc_battery_scale scale;
	if (pdata->max_voltag > 2500000)
		scale = JZ_ADC_BATTERY_SCALE_7V5;
	else
		scale = JZ_ADC_BATTERY_SCALE_2V5;

	return jz4740_adc_read_battery_voltage(bat_ps->dev->parent->parent, scale);
}

static int jz_bat_get_capacity(struct power_supply *bat_ps)
{
	int ret;

	ret = jz_read_bat(bat_ps);

	if (ret < 0)
		return ret;

	ret = (ret - pdata->min_voltag) * 100
		/ (pdata->max_voltag - pdata->min_voltag);

	if (ret > 100)
		ret = 100;
	else if (ret < 0)
		ret = 0;

	return ret;
}

static int jz_bat_get_property(struct power_supply *bat_ps,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat_status;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->batt_tech;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if(jz_read_bat(bat_ps) < pdata->min_voltag) {
			dev_dbg(bat_ps->dev, "%s: battery is dead,"
				"voltage too low!\n", __func__);
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		} else {
			dev_dbg(bat_ps->dev, "%s: battery is good,"
				"voltage normal.\n", __func__);
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = jz_bat_get_capacity(bat_ps);
		dev_dbg(bat_ps->dev, "%s: battery_capacity = %d\n",
			__func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = jz_read_bat(bat_ps);
		if (val->intval < 0)
			return val->intval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = pdata->max_voltag;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = pdata->min_voltag;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void jz_bat_external_power_changed(struct power_supply *bat_ps)
{
	cancel_delayed_work(&bat_work);
	queue_delayed_work(monitor_wqueue, &bat_work, HZ / 8);
}

static char *status_text[] = {
	[POWER_SUPPLY_STATUS_UNKNOWN] =		"Unknown",
	[POWER_SUPPLY_STATUS_CHARGING] =	"Charging",
	[POWER_SUPPLY_STATUS_DISCHARGING] =	"Discharging",
	[POWER_SUPPLY_STATUS_NOT_CHARGING] =    "Not charging",
};

static void jz_bat_update(struct power_supply *bat_ps)
{
	int old_status = bat_status;
	static unsigned long old_batt_vol = 0;
	unsigned long batt_vol = jz_read_bat(bat_ps);
	mutex_lock(&work_lock);

	if(!gpio_get_value(pdata->charg_stat_gpio))
		bat_status = POWER_SUPPLY_STATUS_CHARGING;
	else
		bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	dev_dbg(bat_ps->dev, "%s: battery status=%s\n",
		__func__, status_text[bat_status]);

	if ((old_status != bat_status) ||
		(old_batt_vol - batt_vol > 50000)) {
		dev_dbg(bat_ps->dev, "%s %s -> %s\n",
			 bat_ps->name,
			 status_text[old_status],
			 status_text[bat_status]);

		power_supply_changed(bat_ps);
	}

	old_batt_vol = batt_vol;
	mutex_unlock(&work_lock);
}

static enum power_supply_property jz_bat_main_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY, /* in percents! */
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_PRESENT,
};

struct power_supply bat_ps = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= jz_bat_main_props,
	.num_properties		= ARRAY_SIZE(jz_bat_main_props),
	.get_property		= jz_bat_get_property,
	.external_power_changed = jz_bat_external_power_changed,
	.use_for_apm		= 1,
};

static void jz_bat_work(struct work_struct *work)
{
	/* query interval too small will increase system workload*/
	const int interval = HZ * 30;

	jz_bat_update(&bat_ps);
	queue_delayed_work(monitor_wqueue, &bat_work, interval);
}

#ifdef CONFIG_PM
static int jz_bat_suspend(struct platform_device *dev, pm_message_t state)
{
	bat_status =  POWER_SUPPLY_STATUS_UNKNOWN;

	return 0;
}

static int jz_bat_resume(struct platform_device *dev)
{
	bat_status =  POWER_SUPPLY_STATUS_UNKNOWN;
	cancel_delayed_work(&bat_work);
	queue_delayed_work(monitor_wqueue, &bat_work, HZ/10);

	return 0;
}
#else
#define jz_bat_suspend NULL
#define jz_bat_resume NULL
#endif

static int __devinit jz_bat_probe(struct platform_device *pdev)
{
	int ret = 0;

	printk("JZ battery init.\n");
	mutex_init(&work_lock);
	INIT_DELAYED_WORK(&bat_work, jz_bat_work);

	if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "Please set battery info\n");
		return -EINVAL;
	}

	pdata = pdev->dev.platform_data;

	if (gpio_is_valid(pdata->dc_dect_gpio)) {
		ret = gpio_request(pdata->dc_dect_gpio, "AC/DC DECT");
		if (ret) {
			dev_err(&pdev->dev, "ac/dc dect gpio request failed.\n");

			goto err_dc_gpio_request;
		}
		ret = gpio_direction_input(pdata->dc_dect_gpio);
		if (ret) {
			dev_err(&pdev->dev, "ac/dc dect gpio direction failed.\n");

			goto err_dc_gpio_direction;
		}
	}

	if (gpio_is_valid(pdata->usb_dect_gpio)) {
		ret = gpio_request(pdata->usb_dect_gpio, "USB DECT");
		if (ret) {
			dev_err(&pdev->dev, "usb dect gpio request failed.\n");

			goto err_usb_gpio_request;
		}
		ret = gpio_direction_input(pdata->usb_dect_gpio);
		if (ret) {
			dev_err(&pdev->dev, "usb dect gpio set direction failed.\n");
			goto err_usb_gpio_direction;
		}

		jz_gpio_disable_pullup(pdata->usb_dect_gpio);
		/* TODO: Use generic gpio is better */
	}

	if (gpio_is_valid(pdata->charg_stat_gpio)) {
		ret = gpio_request(pdata->charg_stat_gpio, "CHARG STAT");
		if (ret) {
			dev_err(&pdev->dev, "charger state gpio request failed.\n");
			goto err_charg_gpio_request;
		}
		ret = gpio_direction_input(pdata->charg_stat_gpio);
		if (ret) {
			dev_err(&pdev->dev, "charger state gpio set direction failed.\n");
			goto err_charg_gpio_direction;
		}
	}

	ret = power_supply_register(&pdev->dev, &jz_ac);
	if (ret) {
		dev_err(&pdev->dev, "power supply ac/dc register failed.\n");
		goto err_power_register_ac;
	}

	ret = power_supply_register(&pdev->dev, &jz_usb);
	if (ret) {
		dev_err(&pdev->dev, "power supply usb register failed.\n");
		goto err_power_register_usb;
	}

	ret = power_supply_register(&pdev->dev, &bat_ps);
	if (ret) {
		dev_err(&pdev->dev, "power supply battery register failed.\n");
		goto err_power_register_bat;
	}

	if (!ret) {
		monitor_wqueue = create_singlethread_workqueue("jz_battery");
		if (!monitor_wqueue) {
			return -ESRCH;
		}
		queue_delayed_work(monitor_wqueue, &bat_work, HZ * 1);
	}

	return ret;

err_power_register_bat:
	power_supply_unregister(&jz_usb);
err_power_register_usb:
	power_supply_unregister(&jz_ac);
err_power_register_ac:
err_charg_gpio_direction:
	gpio_free(pdata->charg_stat_gpio);
err_charg_gpio_request:
err_usb_gpio_direction:
	gpio_free(pdata->usb_dect_gpio);
err_usb_gpio_request:
err_dc_gpio_direction:
	gpio_free(pdata->dc_dect_gpio);
err_dc_gpio_request:
	return ret;
}

static int __devexit jz_bat_remove(struct platform_device *dev)
{
	if (pdata) {
		if (gpio_is_valid(pdata->dc_dect_gpio))
			gpio_free(pdata->dc_dect_gpio);
		if (gpio_is_valid(pdata->usb_dect_gpio))
			gpio_free(pdata->usb_dect_gpio);
		if (gpio_is_valid(pdata->charg_stat_gpio))
			gpio_free(pdata->charg_stat_gpio);
	}

	power_supply_unregister(&bat_ps);
	power_supply_unregister(&jz_ac);
	power_supply_unregister(&jz_usb);

	return 0;
}

static struct platform_driver jz_bat_driver = {
	.probe		= jz_bat_probe,
	.remove		= __devexit_p(jz_bat_remove),
	.suspend	= jz_bat_suspend,
	.resume		= jz_bat_resume,
	.driver = {
		.name = "jz4740-battery",
		.owner = THIS_MODULE,
	},
};

static int __init jz_bat_init(void)
{
	return platform_driver_register(&jz_bat_driver);
}
module_init(jz_bat_init);

static void __exit jz_bat_exit(void)
{
	platform_driver_unregister(&jz_bat_driver);
}
module_exit(jz_bat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jiejing Zhang <kzjeef@gmail.com>");
MODULE_DESCRIPTION("JZ4720/JZ4740 SoC battery driver");
