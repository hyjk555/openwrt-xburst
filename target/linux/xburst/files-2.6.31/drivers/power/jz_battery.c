/*
 * linux/drivers/power/jz_battery
 *
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
#include <linux/jz4740_batt.h>

#include <asm/jzsoc.h>

struct workqueue_struct *monitor_wqueue;
struct delayed_work bat_work;
struct mutex work_lock;

static int bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
static struct jz_batt_info *pdata = 0;

extern unsigned int jz_read_battery(void);

/*********************************************************************
 *		Power
 *********************************************************************/

static int jz_get_power_prop(struct power_supply *psy,
			   enum power_supply_property psp,
			   union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = !gpio_get_value(pdata->dc_dect_gpio);
		else
			val->intval = !!gpio_get_value(pdata->usb_dect_gpio);
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

static unsigned long jz_read_bat(struct power_supply *bat_ps)
{
	unsigned long val;
	if (CFG_PBAT_DIV == 1)
		val = (((unsigned long long)jz_read_battery() * 7500000L) >> 12) + 33000L;
	else
		val = ((unsigned long long)jz_read_battery() * CFG_PBAT_DIV * 2500000L) >> 12;
	dev_dbg(bat_ps->dev, "%s: raw_batter_vol = %d uV\n",__func__,val);
	return val;
}

static int jz_bat_get_capacity(struct power_supply *bat_ps)
{
	int ret;
	ret = (jz_read_bat(bat_ps) - pdata->min_voltag) * 100
		/ (pdata->max_voltag - pdata->min_voltag);
	if (ret > 100) {
		dev_warn(bat_ps->dev, "%s: capacity=%d which exceeds 100,"
			 "set to 100\n", __func__, ret);
		ret = 100;
	}
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
		if(jz_read_bat(bat_ps) < JZ_BAT_MIN_VOLTAGE) {
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
		dev_dbg(bat_ps->dev, "%s: battery_capacity = %d\%\n",
			__func__, val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = jz_read_bat(bat_ps);
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
	queue_delayed_work(monitor_wqueue, &bat_work, HZ/10);
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

	if(!gpio_get_value(pdata->charg_stat_pgio))
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

static int __devinit jz_bat_probe(struct platform_device *dev)
{
	int ret = 0;

	printk("JZ battery init.\n");
	mutex_init(&work_lock);
	INIT_DELAYED_WORK(&bat_work, jz_bat_work);

	if (!dev->dev->platform_data) {
		dev_error(&dev->dev, "Please set battery info\n");
		return -EINVAL;
	}

	pdata = dev->dev->platform_data;

	if (gpio_is_valid(pdata->dc_dect_gpio)) {
		ret = gpio_request(pdata->dc_dect_gpio, "AC/DC DECT");
		if (ret) {
			dev_err(dev->dev, "ac/dc dect gpio request failed.\n");

			goto err_dc_gpio_request;
		}
		ret = gpio_direction_input(pdata->dc_dect_gpio);
		if (ret) {
			dev_err(dev->dev, "ac/dc dect gpio direction failed.\n");

			goto err_dc_gpio_direction;
		}
	}

	if (gpio_is_valid(pdata->usb_dect_gpio)) {
		ret = gpio_request(pdata->usb_dect_gpio, "USB DECT");
		if (ret) {
			dev_err(dev->dev, "usb dect gpio request failed.\n");

			goto err_usb_gpio_request;
		}
		ret = gpio_direction_input(pdata->usb_dect_gpio);
		if (ret) {
			dev_err(dev->dev, "usb dect gpio set direction failed.\n");
			goto err_usb_gpio_direction;
		}

		jz_gpio_disable_pullup(pdata->usb_dect_gpio);
		/* TODO: Use generic gpio is better */
	}

	if (gpio_is_valid(pdata->charg_stat_gpio)) {
		ret = gpio_request(pdata->charg_stat_gpio, "CHARG STAT");
		if (ret) {
			dev_err(dev->dev, "charger state gpio request failed.\n");
			goto err_charg_gpio_request;
		}
		ret = gpio_direction_input(pdata->charg_stat_pgio);
		if (ret) {
			dev_err(dev->dev, "charger state gpio set direction failed.\n");
			goto err_charg_gpio_direction;
		}
	}

	ret = power_supply_register(&dev->dev, &jz_ac);
	if (ret) {
		dev_err(dev->dev, "power supply ac/dc register failed.\n");
		goto err_power_register_ac;
	}
	
	ret = power_supply_register(&dev->dev, &jz_usb);
	if (ret) {
		dev_err(dev->dev, "power supply usb register failed.\n");
		goto err_power_register_usb;
	}

	ret = power_supply_register(&dev->dev, &bat_ps);
	if (ret) {
		dev_err(dev->dev, "power supply battery register failed.\n");
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
	gpio_free(pdata->charg_stat_pgio);
err_charg_gpio_request:
err_usb_gpio_direction:
	gpio_free(pdata->usb_dect_gpio);
err_usb_gpio_request:
err_dc_gpio_direction:
	gpio_free(pdata->dc_dect_gpio);
err_err_dc_gpio_request:
	return ret;
}

static int __devexit jz_bat_remove(struct platform_device *dev)
{
	if (pdata) {
	    if (gpio_is_valid(pdata->dc_dct_gpio))
		    gpio_free(pdata->dc_dect_gpio);
	    if (gpio_is_valid(pdata->usb_dect_gpio))
		    gpio_free(pdata->usb_dect_pgio);
	    if (gpio_is_valid(pdata->charg_stat_gpio))
		    gpio_free(pdata->charg_stat_gpio);
	}

	power_supply_unregister(&bat_ps);
	power_supply_unregister(&jz_ac);
	power_supply_unregister(&jz_usb);
	return 0;
}

static struct platform_driver jz_bat_driver = {
	.driver.name	= "jz-battery",
	.driver.owner	= THIS_MODULE,
	.probe		= jz_bat_probe,
	.remove		= __devexit_p(jz_bat_remove),
	.suspend	= jz_bat_suspend,
	.resume		= jz_bat_resume,
};

static int __init jz_bat_init(void)
{
	platform_device_register_simple("jz-battery", 0, NULL, 0);
	return platform_driver_register(&jz_bat_driver);
}

static void __exit jz_bat_exit(void)
{
	platform_driver_unregister(&jz_bat_driver);
}

module_init(jz_bat_init);
module_exit(jz_bat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marek Vasut <marek.vasut@gmail.com>");
MODULE_DESCRIPTION("Palm T|X battery driver");
