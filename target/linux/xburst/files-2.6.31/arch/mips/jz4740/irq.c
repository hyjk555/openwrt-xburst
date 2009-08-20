/*
 * linux/arch/mips/jz4740/irq.c
 *
 * JZ4740 interrupt routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <lhhuang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/timex.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/delay.h>
#include <linux/bitops.h>

#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/mipsregs.h>
#include <asm/system.h>
#include <asm/jzsoc.h>

static void __iomem *jz_intc_base;

#define JZ_REG_BASE_INTC 0x10001000

#define JZ_REG_INTC_STATUS	0x00
#define JZ_REG_INTC_MASK	0x04
#define JZ_REG_INTC_SET_MASK	0x08
#define JZ_REG_INTC_CLEAR_MASK	0x0c
#define JZ_REG_INTC_PENDING	0x10

/*
 * INTC irq type
 */

static void intc_irq_unmask(unsigned int irq)
{
	writel(BIT(irq), jz_intc_base + JZ_REG_INTC_CLEAR_MASK);
}

static void intc_irq_mask(unsigned int irq)
{
	writel(BIT(irq), jz_intc_base + JZ_REG_INTC_SET_MASK);
}

static void intc_irq_ack(unsigned int irq)
{
	writel(BIT(irq), jz_intc_base + JZ_REG_INTC_PENDING);
}

static void intc_irq_end(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS))) {
		intc_irq_unmask(irq);
	}
}

static struct irq_chip intc_irq_type = {
	.name =	    "INTC",
	.mask =	    intc_irq_mask,
	.unmask =   intc_irq_unmask,
	.ack =	    intc_irq_ack,
	.end =	    intc_irq_end,
};

/*
 * DMA irq type
 */

static void enable_dma_irq(unsigned int irq)
{
	__intc_unmask_irq(IRQ_DMAC);
	__dmac_channel_enable_irq(irq - IRQ_DMA_0);
}

static void disable_dma_irq(unsigned int irq)
{
	__dmac_channel_disable_irq(irq - IRQ_DMA_0);
}

static void mask_and_ack_dma_irq(unsigned int irq)
{
	__intc_ack_irq(IRQ_DMAC);
	__dmac_channel_disable_irq(irq - IRQ_DMA_0);
}

static void end_dma_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS))) {
		enable_dma_irq(irq);
	}
}

static struct irq_chip dma_irq_type = {
	.name = "DMA",
	.unmask = enable_dma_irq,
	.mask = disable_dma_irq,
	.ack = mask_and_ack_dma_irq,
	.end = end_dma_irq,
};

//----------------------------------------------------------------------

void __init arch_init_irq(void)
{
	int i;

	clear_c0_status(0xff04); /* clear ERL */
	set_c0_status(0x0400);   /* set IP2 */

	jz_intc_base = ioremap(JZ_REG_BASE_INTC, 0x14);

	for (i = 0; i < 32; i++) {
		intc_irq_mask(i);
		set_irq_chip_and_handler(i, &intc_irq_type, handle_level_irq);
	}


	/* Set up DMAC irq
	 */
	for (i = 0; i < NUM_DMA; i++) {
		disable_dma_irq(IRQ_DMA_0 + i);
        set_irq_chip_and_handler(IRQ_DMA_0 + i, &dma_irq_type, handle_level_irq);
	}

}

asmlinkage void plat_irq_dispatch(void)
{
	uint32_t irq_reg;

	irq_reg = readl(jz_intc_base + JZ_REG_INTC_PENDING);

	if (irq_reg)
	    do_IRQ(ffs(irq_reg) - 1);
}
