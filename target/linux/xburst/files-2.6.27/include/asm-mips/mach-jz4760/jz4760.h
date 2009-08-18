/*
 *  linux/include/asm-mips/mach-jz4760/jz4760.h
 *
 *  JZ4760 common definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4760_H__
#define __ASM_JZ4760_H__

#include <asm/mach-jz4760/regs.h>
#include <asm/mach-jz4760/ops.h>
#include <asm/mach-jz4760/dma.h>
#include <asm/mach-jz4760/misc.h>

/*------------------------------------------------------------------
 * Platform definitions
 */
#ifdef CONFIG_JZ4760_F4760
#include <asm/mach-jz4760/board-f4760.h>
#endif

/* Add other platform definition here ... */


/*------------------------------------------------------------------
 * Follows are related to platform definitions
 */

#include <asm/mach-jz4760/clock.h>
#include <asm/mach-jz4760/serial.h>

#endif /* __ASM_JZ4760_H__ */
