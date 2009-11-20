#ifndef __JZ4740_IRQ_H__
#define __JZ4740_IRQ_H__
/*
 *  JZ4740 irqs.
 *
 *  Copyright (C) 2009, Lars-Peter Clausen <lars@metafoo.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define JZ_IRQ_BASE 8

// 1st-level interrupts
#define JZ_IRQ(x)	(JZ_IRQ_BASE + (x))
#define JZ_IRQ_I2C	JZ_IRQ(1)
#define JZ_IRQ_UHC	JZ_IRQ(3)
#define JZ_IRQ_UART1	JZ_IRQ(8)
#define JZ_IRQ_UART0	JZ_IRQ(9)
#define JZ_IRQ_SADC	JZ_IRQ(12)
#define JZ_IRQ_MSC	JZ_IRQ(14)
#define JZ_IRQ_RTC	JZ_IRQ(15)
#define JZ_IRQ_SSI	JZ_IRQ(16)
#define JZ_IRQ_CIM	JZ_IRQ(17)
#define JZ_IRQ_AIC	JZ_IRQ(18)
#define JZ_IRQ_ETH	JZ_IRQ(19)
#define JZ_IRQ_DMAC	JZ_IRQ(20)
#define JZ_IRQ_TCU2	JZ_IRQ(21)
#define JZ_IRQ_TCU1	JZ_IRQ(22)
#define JZ_IRQ_TCU0	JZ_IRQ(23)
#define JZ_IRQ_UDC 	JZ_IRQ(24)
#define JZ_IRQ_GPIO3	JZ_IRQ(25)
#define JZ_IRQ_GPIO2	JZ_IRQ(26)
#define JZ_IRQ_GPIO1	JZ_IRQ(27)
#define JZ_IRQ_GPIO0	JZ_IRQ(28)
#define JZ_IRQ_IPU	JZ_IRQ(29)
#define JZ_IRQ_LCD	JZ_IRQ(30)

/* 2nd-level interrupts */
#define JZ_IRQ_DMA(x)	((x) + JZ_IRQ(32))  /* 32 to 37 for DMAC channel 0 to 5 */
#define JZ_IRQ_GPIO_0	JZ_IRQ(48)  /* 48 to 175 for GPIO pin 0 to 127 */

#define JZ_IRQ_INTC_GPIO(x)	(JZ_IRQ_GPIO0 - (x))
#define JZ_IRQ_GPIO(x)		((x) + JZ_IRQ(48)

#define NR_IRQS (JZ_IRQ_GPIO(127) + 1)

#endif
