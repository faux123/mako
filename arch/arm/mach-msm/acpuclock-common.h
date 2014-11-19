/* arch/arm/mach-msm/acpuclock-common.h
 *
 * MSM architecture cpu clock common definitions 
 *
 * Copyright (C) 2014 Paul Reioux (faux123)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __ARCH_ARM_MACH_MSM_ACPUCLOCK_COMMON_H
#define __ARCH_ARM_MACH_MSM_ACPUCLOCK_COMMON_H

#ifdef CONFIG_OC_ULTIMATE
#ifdef CONFIG_LOW_CPUCLOCKS
#define FREQ_TABLE_SIZE         41
#else
#define FREQ_TABLE_SIZE         37
#endif
#else
#ifdef CONFIG_LOW_CPUCLOCKS
#define FREQ_TABLE_SIZE         39
#else
#define FREQ_TABLE_SIZE         35
#endif
#endif

#endif  /* __ARCH_ARM_MACH_MSM_ACPUCLOCK_COMMON_H */
