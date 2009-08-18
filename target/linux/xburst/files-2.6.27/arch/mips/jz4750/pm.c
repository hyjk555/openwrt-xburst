/*
 * linux/arch/mips/jz4750/common/pm.c
 * 
 * JZ4750 Power Management Routines
 * 
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 * 
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 * 
 */

#include <linux/init.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h> 
#include <linux/sysctl.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <asm/cacheops.h>
#include <asm/jzsoc.h>

extern void jz_board_do_sleep(unsigned long *ptr);
extern void jz_board_do_resume(unsigned long *ptr);

static void jz_pm_do_hibernate(void)
{
	printk("Put CPU into hibernate mode.\n");

	/* Mask all interrupts */
	REG_INTC_IMSR = 0xffffffff;

	/* 
	 * RTC Wakeup or 1Hz interrupt can be enabled or disabled 
	 * through  RTC driver's ioctl (linux/driver/char/rtc_jz.c).
	 */

	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HWFCR = (100 << RTC_HWFCR_BIT);

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HRCR = (60 << RTC_HRCR_BIT); /* 60 ms */

	/* Scratch pad register to be reserved */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HSPR = 0x12345678;

	/* clear wakeup status register */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HWRSR = 0x0;

	/* Put CPU to power down mode */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HCR = RTC_HCR_PD;

	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	while(1);

	/* We can't get here */
}

static int jz_pm_do_sleep(void)
{ 
	unsigned long delta;
	unsigned long nfcsr = REG_EMC_NFCSR;
	unsigned long opcr = REG_CPM_OPCR;
	unsigned long imr = REG_INTC_IMR;
	unsigned long sadc = REG_SADC_ENA;
	unsigned long sleep_gpio_save[4*(GPIO_PORT_NUM-1)];

	printk("Put CPU into sleep mode.\n");

	/* Preserve current time */
	delta = xtime.tv_sec - REG_RTC_RSR;

        /* Disable nand flash */
	REG_EMC_NFCSR = ~0xff;

        /* stop sadc */
	REG_SADC_ENA &= ~0x7;
	while((REG_SADC_ENA & 0x7) != 0);
 	udelay(100);

        /*stop udc and usb*/
	__cpm_suspend_uhcphy();
	__cpm_suspend_udcphy();

	/* Mask all interrupts */
	REG_INTC_IMSR = 0xffffffff;

	/* Sleep on-board modules and setup wake event */
	jz_board_do_sleep(sleep_gpio_save);

	/* disable externel clock Oscillator in sleep mode */
	__cpm_disable_osc_in_sleep();
	/* select 32K crystal as RTC clock in sleep mode */
	__cpm_select_rtcclk_rtc();

 	/* Enter SLEEP mode */
	REG_CPM_LCR &= ~CPM_LCR_LPM_MASK;
	REG_CPM_LCR |= CPM_LCR_LPM_SLEEP;
	__asm__(".set\tmips3\n\t"
		"wait\n\t"
		".set\tmips0");

	/* Restore to IDLE mode */
	REG_CPM_LCR &= ~CPM_LCR_LPM_MASK;
	REG_CPM_LCR |= CPM_LCR_LPM_IDLE;

        /* Restore nand flash control register */
	REG_EMC_NFCSR = nfcsr;

	/* Restore interrupts */
	REG_INTC_IMSR = imr;
	REG_INTC_IMCR = ~imr;
	
	/* Restore sadc */
	REG_SADC_ENA = sadc;
	
	/* Resume on-board modules */
	jz_board_do_resume(sleep_gpio_save);

	/* Restore Oscillator and Power Control Register */
	REG_CPM_OPCR = opcr;

	/* Restore current time */
	xtime.tv_sec = REG_RTC_RSR + delta;

	printk("Resume CPU from sleep mode.\n");

	return 0;
}

/* Put CPU to HIBERNATE mode 
 *----------------------------------------------------------------------------
 * Power Management sleep sysctl interface
 *
 * Write "mem" to /sys/power/state invokes this function 
 * which initiates a poweroff.
 */
void jz_pm_hibernate(void)
{
	jz_pm_do_hibernate();
}

/* Put CPU to SLEEP mode
 *----------------------------------------------------------------------------
 * Power Management sleep sysctl interface
 *
 * Write "standby" to /sys/power/state invokes this function 
 * which initiates a sleep.
 */

int jz_pm_sleep(void) 
{
	return jz_pm_do_sleep();
}

/*
 * valid states, only support standby(sleep) and mem(hibernate)
 */
static int jz4750_pm_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}

/*
 * Jz CPU enter save power mode
 */
static int jz4750_pm_enter(suspend_state_t state)
{
	return jz_pm_do_sleep();
}

static struct platform_suspend_ops jz4750_pm_ops = {
	.valid		= jz4750_pm_valid,
	.enter		= jz4750_pm_enter,
};

/*
 * Initialize power interface
 */
int __init jz_pm_init(void)
{
	printk("JZ4750 Power Management\n");

	suspend_set_ops(&jz4750_pm_ops);
	return 0;
}
