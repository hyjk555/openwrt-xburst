/*
 * Real Time Clock interface for Jz4750/Jz4755.
 *
 * Copyright (C) 2005-2009, Ingenic Semiconductor Inc.
 *
 * Author: Richard Feng <cjfeng@ingenic.cn>
 *         Regen Huang <lhhuang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/bitops.h>

#include <asm/irq.h>
#include <asm/jzsoc.h>

#define TIMER_FREQ		CLOCK_TICK_RATE

/* The divider is decided by the RTC clock frequency. */
#define RTC_FREQ_DIVIDER	(32768 - 1)

/* Default time for the first-time power on */
static struct rtc_time default_tm = {
	.tm_year = (2009 - 1900), // year 2009
	.tm_mon = (10 - 1),       // month 10
	.tm_mday = 1,             // day 1
	.tm_hour = 12,
	.tm_min = 0,
	.tm_sec = 0
};

static unsigned long rtc_freq = 1024;
static struct rtc_time rtc_alarm;
static DEFINE_SPINLOCK(jz4750_rtc_lock);

static inline int rtc_periodic_alarm(struct rtc_time *tm)
{
	return  (tm->tm_year == -1) ||
		((unsigned)tm->tm_mon >= 12) ||
		((unsigned)(tm->tm_mday - 1) >= 31) ||
		((unsigned)tm->tm_hour > 23) ||
		((unsigned)tm->tm_min > 59) ||
		((unsigned)tm->tm_sec > 59);
}

/*
 * Calculate the next alarm time given the requested alarm time mask
 * and the current time.
 */
static void rtc_next_alarm_time(struct rtc_time *next, struct rtc_time *now, struct rtc_time *alrm)
{
	unsigned long next_time;
	unsigned long now_time;

	next->tm_year = now->tm_year;
	next->tm_mon = now->tm_mon;
	next->tm_mday = now->tm_mday;
	next->tm_hour = alrm->tm_hour;
	next->tm_min = alrm->tm_min;
	next->tm_sec = alrm->tm_sec;

	rtc_tm_to_time(now, &now_time);
	rtc_tm_to_time(next, &next_time);

	if (next_time < now_time) {
		/* Advance one day */
		next_time += 60 * 60 * 24;
		rtc_time_to_tm(next_time, next);
	}
}

static int rtc_update_alarm(struct rtc_time *alrm)
{
	struct rtc_time alarm_tm, now_tm;
	unsigned long now, time;
	int ret;

	do {
		now = REG_RTC_RSR;
		rtc_time_to_tm(now, &now_tm);
		rtc_next_alarm_time(&alarm_tm, &now_tm, alrm);
		ret = rtc_tm_to_time(&alarm_tm, &time);
		if (ret != 0)
			break;
		while ( !__rtc_write_ready());
		REG_RTC_RCR = REG_RTC_RCR & ~( RTC_RCR_1HZIE | RTC_RCR_1HZ | RTC_RCR_AIE | RTC_RCR_AF | RTC_RCR_AE);
		while ( !__rtc_write_ready());
		REG_RTC_RSAR = time;
		while ( !__rtc_write_ready());
	} while (now != REG_RTC_RSR);
	return ret;
}

static irqreturn_t jz4750_rtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = to_platform_device(dev_id);
	struct rtc_device *rtc = platform_get_drvdata(pdev);
	unsigned int rtsr;
	unsigned long events = 0;

	spin_lock(&jz4750_rtc_lock);
	rtsr = REG_RTC_RCR;

	if ((rtsr & (RTC_RCR_1HZIE | RTC_RCR_AE | RTC_RCR_AIE)) == (RTC_RCR_1HZIE | RTC_RCR_AE | RTC_RCR_AIE)) {
		//printk("1Hz&alarm!\n");
		while ( !__rtc_write_ready());
		REG_RTC_RCR = rtsr & ~(RTC_RCR_1HZ | RTC_RCR_1HZIE | RTC_RCR_AF | RTC_RCR_AIE);
		while ( !__rtc_write_ready());
		if (rtsr & RTC_RCR_AF) {
			rtsr &= ~RTC_RCR_AIE;
			while ( !__rtc_write_ready());
			__rtc_disable_alarm_irq();
			while ( !__rtc_write_ready());
			__rtc_clear_alarm_flag();
			while ( !__rtc_write_ready());
			__rtc_disable_alarm();
		}
       
		/* update irq data & counter */
		if (rtsr & RTC_RCR_AF)
			events |= RTC_AF | RTC_IRQF;
		if (rtsr & RTC_RCR_1HZ)
			events |= RTC_UF | RTC_IRQF;
		rtc_update_irq(rtc, 1, events);
		if ((rtsr & RTC_RCR_AF) && rtc_periodic_alarm(&rtc_alarm))
			rtc_update_alarm(&rtc_alarm);
		if (rtsr & RTC_RCR_1HZ) {
			if ((rtsr & RTC_RCR_AF) == 0) {
				while ( !__rtc_write_ready());
				__rtc_enable_alarm_irq();
			}
			while ( !__rtc_write_ready());
			__rtc_enable_1Hz_irq();
			while ( !__rtc_write_ready());
		}

	} else if ((rtsr & (RTC_RCR_1HZ | RTC_RCR_1HZIE)) == (RTC_RCR_1HZ | RTC_RCR_1HZIE)) {
		//printk("1Hz!\n");
		while ( !__rtc_write_ready());
		REG_RTC_RCR = rtsr & ~(RTC_RCR_1HZ | RTC_RCR_1HZIE | RTC_RCR_AF | RTC_RCR_AIE);
		while ( !__rtc_write_ready());
		REG_RTC_RCR |= RTC_RCR_1HZIE;
		if (rtsr & RTC_RCR_1HZ)
			events |= RTC_UF | RTC_IRQF;
		rtc_update_irq(rtc, 1, events);
	} else if ((rtsr & (RTC_RCR_AE | RTC_RCR_AIE | RTC_RCR_AF)) == (RTC_RCR_AE | RTC_RCR_AIE | RTC_RCR_AF)) {
		//printk("alarm!\n");
		while ( !__rtc_write_ready());
		REG_RTC_RCR = rtsr & ~(RTC_RCR_1HZ | RTC_RCR_1HZIE | RTC_RCR_AF | RTC_RCR_AIE);
		/* clear alarm interrupt if it has occurred */
		rtsr &= ~RTC_RCR_AIE;
		events |= RTC_AF | RTC_IRQF;
		rtc_update_irq(rtc, 1, events);
		if (rtsr & RTC_RCR_AF && rtc_periodic_alarm(&rtc_alarm))
			rtc_update_alarm(&rtc_alarm);
	}
	spin_unlock(&jz4750_rtc_lock);

	return IRQ_HANDLED;
}

#if 0
static int rtc_timer1_count;
static irqreturn_t timer1_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = to_platform_device(dev_id);
	struct rtc_device *rtc = platform_get_drvdata(pdev);

	/*
	 * If we match for the first time, rtc_timer1_count will be 1.
	 * Otherwise, we wrapped around (very unlikely but
	 * still possible) so compute the amount of missed periods.
	 * The match reg is updated only when the data is actually retrieved
	 * to avoid unnecessary interrupts.
	 */
	OSSR = OSSR_M1;	/* clear match on timer1 */

	rtc_update_irq(rtc, rtc_timer1_count, RTC_PF | RTC_IRQF);

	if (rtc_timer1_count == 1)
		rtc_timer1_count = (rtc_freq * ((1<<30)/(TIMER_FREQ>>2)));

	return IRQ_HANDLED;
}
#endif

#if 0
static int jz4750_rtc_read_callback(struct device *dev, int data)
{
	if (data & RTC_PF) {
		/* interpolate missed periods and set match for the next */
		unsigned long period = TIMER_FREQ/rtc_freq;
		unsigned long oscr = OSCR;
		unsigned long osmr1 = OSMR1;
		unsigned long missed = (oscr - osmr1)/period;
		data += missed << 8;
		OSSR = OSSR_M1;	/* clear match on timer 1 */
		OSMR1 = osmr1 + (missed + 1)*period;
		/* Ensure we didn't miss another match in the mean time.
		 * Here we compare (match - OSCR) 8 instead of 0 --
		 * see comment in pxa_timer_interrupt() for explanation.
		 */
		while( (signed long)((osmr1 = OSMR1) - OSCR) <= 8 ) {
			data += 0x100;
			OSSR = OSSR_M1;	/* clear match on timer 1 */
			OSMR1 = osmr1 + period;
		}
	}
	return data;
}
#endif

static int jz4750_rtc_open(struct device *dev)
{
	int ret;

	ret = request_irq(IRQ_RTC, jz4750_rtc_interrupt, IRQF_DISABLED,
				"rtc 1Hz and alarm", dev);
	if (ret) {
		dev_err(dev, "IRQ %d already in use.\n", IRQ_RTC);
		goto fail_ui;
	}

	/*ret = request_irq(IRQ_OST1, timer1_interrupt, IRQF_DISABLED,
				"rtc timer", dev);
	if (ret) {
		dev_err(dev, "IRQ %d already in use.\n", IRQ_OST1);
		goto fail_pi;
		}*/

	return 0;

 fail_ui:
	free_irq(IRQ_RTC, dev);
	return ret;
}

static void jz4750_rtc_release(struct device *dev)
{
	spin_lock_irq(&jz4750_rtc_lock);
	
	spin_unlock_irq(&jz4750_rtc_lock);
	//free_irq(IRQ_OST1, dev);
	free_irq(IRQ_RTC, dev);
}


static int jz4750_rtc_ioctl(struct device *dev, unsigned int cmd,
		unsigned long arg)
{
	switch(cmd) {
	case RTC_AIE_OFF:
		spin_lock_irq(&jz4750_rtc_lock);
		while ( !__rtc_write_ready());
		__rtc_disable_alarm_irq();
		while ( !__rtc_write_ready());
		__rtc_disable_alarm();
		while ( !__rtc_write_ready());
		spin_unlock_irq(&jz4750_rtc_lock);
		return 0;
	case RTC_AIE_ON:
		spin_lock_irq(&jz4750_rtc_lock);
		while ( !__rtc_write_ready());
		__rtc_enable_alarm();
		while ( !__rtc_write_ready());
		__rtc_enable_alarm_irq();
		while ( !__rtc_write_ready());
		spin_unlock_irq(&jz4750_rtc_lock);
		return 0;
	case RTC_UIE_OFF:
		spin_lock_irq(&jz4750_rtc_lock);
		while ( !__rtc_write_ready());
		__rtc_disable_1Hz_irq();
		while ( !__rtc_write_ready());
		spin_unlock_irq(&jz4750_rtc_lock);
		return 0;
	case RTC_UIE_ON:
		spin_lock_irq(&jz4750_rtc_lock);
		while ( !__rtc_write_ready());
		__rtc_clear_1Hz_flag();
		while ( !__rtc_write_ready());
		__rtc_clear_alarm_flag();
		while ( !__rtc_write_ready());
		__rtc_enable_1Hz_irq();
		while ( !__rtc_write_ready());
		spin_unlock_irq(&jz4750_rtc_lock);
		return 0;
	case RTC_PIE_OFF:
		spin_lock_irq(&jz4750_rtc_lock);
		printk("no implement!\n");
		spin_unlock_irq(&jz4750_rtc_lock);
		return 0;
	case RTC_PIE_ON:
		spin_lock_irq(&jz4750_rtc_lock);
		printk("no implement!\n");
		spin_unlock_irq(&jz4750_rtc_lock);
		return 0;
	case RTC_IRQP_READ:
		return put_user(rtc_freq, (unsigned long *)arg);
	case RTC_IRQP_SET:
		if (arg < 1 || arg > TIMER_FREQ)
			return -EINVAL;
		rtc_freq = arg;
		return 0;
	}
	return -ENOIOCTLCMD;
}

static int jz4750_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long time;
	int ret;

	ret = rtc_tm_to_time(tm, &time);
	if (ret == 0) {
		while ( !__rtc_write_ready());
		REG_RTC_RSR = time;
		while ( !__rtc_write_ready());
	}
	return ret;
}

static int jz4750_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	rtc_time_to_tm(REG_RTC_RSR, tm);

	if (rtc_valid_tm(tm) < 0) {
                /* Set the default time */
		jz4750_rtc_set_time(dev, &default_tm);

		rtc_time_to_tm(REG_RTC_RSR, tm);
	}

	return 0;
}

static int jz4750_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	u32     rtc_rcr;

	rtc_time_to_tm(REG_RTC_RSAR, &rtc_alarm);
	memcpy(&alrm->time, &rtc_alarm, sizeof(struct rtc_time));
	rtc_rcr = REG_RTC_RCR;

	alrm->enabled = (rtc_rcr & RTC_RCR_AIE) ? 1 : 0;
	alrm->pending = (rtc_rcr & RTC_RCR_AF) ? 1 : 0;
	return 0;
}

static int jz4750_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	int ret;

	spin_lock_irq(&jz4750_rtc_lock);
	ret = rtc_update_alarm(&alrm->time);
	if (ret == 0) {
		if (alrm->enabled) {
			while ( !__rtc_write_ready());
			__rtc_enable_alarm();
			while ( !__rtc_write_ready());
			__rtc_enable_alarm_irq();
			while ( !__rtc_write_ready());
		} else {
			while ( !__rtc_write_ready());
			__rtc_disable_alarm_irq();
			while ( !__rtc_write_ready());
			__rtc_disable_alarm();
			while ( !__rtc_write_ready());
		}
	}
	spin_unlock_irq(&jz4750_rtc_lock);

	return ret;
}

static int jz4750_rtc_proc(struct device *dev, struct seq_file *seq)
{
	seq_printf(seq, "RTC regulator\t: 0x%08x\n", (u32) REG_RTC_RGR);
	seq_printf(seq, "update_IRQ\t: %s\n",
			(REG_RTC_RCR & RTC_RCR_1HZIE) ? "yes" : "no");
	/*seq_printf(seq, "periodic_IRQ\t: %s\n",
	  (OIER & OIER_E1) ? "yes" : "no");*/
	seq_printf(seq, "periodic_freq\t: %ld\n", rtc_freq);

	return 0;
}

static const struct rtc_class_ops jz4750_rtc_ops = {
	.open = jz4750_rtc_open,
	//.read_callback = jz4750_rtc_read_callback,
	.release = jz4750_rtc_release,
	.ioctl = jz4750_rtc_ioctl,
	.read_time = jz4750_rtc_read_time,
	.set_time = jz4750_rtc_set_time,
	.read_alarm = jz4750_rtc_read_alarm,
	.set_alarm = jz4750_rtc_set_alarm,
	.proc = jz4750_rtc_proc,
};

static int jz4750_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;

	/*
	 * When we are powered on for the first time, init the rtc and reset time.
	 *
	 * For other situations, we remain the rtc status unchanged.
	 */
	if (__rtc_status_ppr_reset_occur()) {
		/* We are powered on for the first time !!! */

		printk("jz4750-rtc: rtc status reset by power-on\n");

		/* select external 32K crystal as RTC clock */
		__cpm_select_rtcclk_rtc();

		/* init rtc status */
		while ( !__rtc_write_ready());
		__rtc_disable_1Hz_irq();
		while ( !__rtc_write_ready());
		__rtc_disable_alarm_irq();
		while ( !__rtc_write_ready());
		__rtc_clear_alarm_flag();
		while ( !__rtc_write_ready());
		__rtc_clear_1Hz_flag();
		while ( !__rtc_write_ready());
		__rtc_disable_alarm();
		while ( !__rtc_write_ready());

		/* Set 32768 rtc clocks per seconds */
		REG_RTC_RGR  = RTC_FREQ_DIVIDER;
		while ( !__rtc_write_ready());

		/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */
		REG_RTC_HWFCR = (100 << RTC_HWFCR_BIT);
		while ( !__rtc_write_ready());
	
		/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
		REG_RTC_HRCR = (60 << RTC_HRCR_BIT);
		while ( !__rtc_write_ready());

                /* Reset to the default time */
		jz4750_rtc_set_time(NULL, &default_tm);
		while ( !__rtc_write_ready());

		/* start rtc */
		__rtc_enabled();
		while ( !__rtc_write_ready());
	}

	/* clear all rtc flags */
	__rtc_clear_hib_stat_all();
	while ( !__rtc_write_ready());

	device_init_wakeup(&pdev->dev, 1);

	rtc = rtc_device_register(pdev->name, &pdev->dev, &jz4750_rtc_ops,
				THIS_MODULE);

	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	platform_set_drvdata(pdev, rtc);

	return 0;
}

static int jz4750_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);

	while ( !__rtc_write_ready());
	__rtc_disable_1Hz_irq();
	while ( !__rtc_write_ready());
	__rtc_disable_alarm_irq();
	while ( !__rtc_write_ready());
	__rtc_disabled();

 	if (rtc)
		rtc_device_unregister(rtc);

	return 0;
}

#ifdef CONFIG_PM
static int jz4750_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
//	if (device_may_wakeup(&pdev->dev))
//		enable_irq_wake(IRQ_RTC);
	return 0;
}

static int jz4750_rtc_resume(struct platform_device *pdev)
{
//	if (device_may_wakeup(&pdev->dev))
//		disable_irq_wake(IRQ_RTC);
	return 0;
}
#else
#define jz4750_rtc_suspend	NULL
#define jz4750_rtc_resume	NULL
#endif

static struct platform_driver jz4750_rtc_driver = {
	.probe		= jz4750_rtc_probe,
	.remove		= jz4750_rtc_remove,
	.suspend	= jz4750_rtc_suspend,
	.resume		= jz4750_rtc_resume,
	.driver		= {
		.name		= "jz4750-rtc",
	},
};

static int __init jz4750_rtc_init(void)
{
	return platform_driver_register(&jz4750_rtc_driver);
}

static void __exit jz4750_rtc_exit(void)
{
	platform_driver_unregister(&jz4750_rtc_driver);
}

module_init(jz4750_rtc_init);
module_exit(jz4750_rtc_exit);

MODULE_AUTHOR("Richard Feng <cjfeng@ingenic.cn>");
MODULE_DESCRIPTION("JZ4750 Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz4750-rtc");
