/*
 * linux/drivers/power/jz_battery
 *
 * Battery measurement code for Ingenic JZ SOC.
 *
 * based on tosa_battery.c
 *
 * Copyright (C) 2008 Marek Vasut <marek.vasut@gmail.com>
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

#include <asm/jzsoc.h>

#ifdef CONFIG_POWER_SUPPLY_DEBUG
#define dprintk(x...) printk(x)
#else
#define dprintk(x...) while(0){}
#endif

#define JZ_BAT_MAX_VOLTAGE 4200000 // uV
#define JZ_BAT_MIN_VOLTAGE 3600000

static DEFINE_MUTEX(bat_lock);
struct workqueue_struct *monitor_wqueue;
struct delayed_work bat_work;
struct mutex work_lock;

int bat_status = POWER_SUPPLY_STATUS_DISCHARGING;

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
			val->intval = !__gpio_get_pin(GPIO_DC_DETE_N);
		else
			val->intval = __gpio_get_pin(GPIO_USB_DETE);
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
		val = (((unsigned long long)jz_read_battery() * 7500000)) >> 12;
	else
		val = (((unsigned long long)jz_read_battery() * CFG_PBAT_DIV * 2500000)) >> 12;
	dprintk("--raw_batter_vol=%d uV\n", val);
	return val;
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
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if(jz_read_bat(bat_ps) < 3600000) {
			dprintk("--battery dead\n");
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		} else {
			dprintk("--battery good\n");
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = (jz_read_bat(bat_ps) - 3600000) * 100 / (4200000 - 3600000);
		if (val->intval > 100)
			val->intval = 100;
		dprintk("--battery_capacity=%d\%\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = jz_read_bat(bat_ps);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = JZ_BAT_MAX_VOLTAGE;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = JZ_BAT_MIN_VOLTAGE;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_VOL:
		val->intval = 0; // reading TEMP and VOL aren't supported
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

	if(!__gpio_get_pin(GPIO_CHARG_STAT_N))
		bat_status = POWER_SUPPLY_STATUS_CHARGING;
	else {
		bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	dprintk("--battery status=%s\n", status_text[bat_status]);
	if ((old_status != bat_status) ||
	    (old_batt_vol - batt_vol > 50000)) {
		pr_debug("%s %s -> %s\n", bat_ps->name,
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
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOL,
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
	const int interval = HZ * 6;

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

	__gpio_disable_pull(GPIO_USB_DETE);

	power_supply_register(&dev->dev, &jz_ac);
	power_supply_register(&dev->dev, &jz_usb);

	ret = power_supply_register(&dev->dev, &bat_ps);
	if (!ret) {
		monitor_wqueue = create_singlethread_workqueue("jz_battery");
		if (!monitor_wqueue) {
			return -ESRCH;
		}
		queue_delayed_work(monitor_wqueue, &bat_work, HZ * 1);
	}

	return ret;
}

static int __devexit jz_bat_remove(struct platform_device *dev)
{
	power_supply_unregister(&bat_ps);
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
