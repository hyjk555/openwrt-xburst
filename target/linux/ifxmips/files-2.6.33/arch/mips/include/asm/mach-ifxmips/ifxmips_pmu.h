/*
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA.
 *
 *   Copyright (C) 2007 John Crispin <blogic@openwrt.org>
 */
#ifndef _IFXMIPS_PMU_H__
#define _IFXMIPS_PMU_H__


#define IFXMIPS_PMU_PWDCR_DMA		0x0020
#define IFXMIPS_PMU_PWDCR_USB		0x8041
#define IFXMIPS_PMU_PWDCR_LED		0x0800
#define IFXMIPS_PMU_PWDCR_GPT		0x1000
#define IFXMIPS_PMU_PWDCR_PPE		0x2000
#define IFXMIPS_PMU_PWDCR_FPI		0x4000

void ifxmips_pmu_enable(unsigned int module);
void ifxmips_pmu_disable(unsigned int module);

#endif
