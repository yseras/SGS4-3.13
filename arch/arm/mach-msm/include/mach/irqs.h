/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_IRQS_H
#define __ASM_ARCH_MSM_IRQS_H

#define MSM_IRQ_BIT(irq)     (1 << ((irq) & 31))

/* For now, use the maximum number of interrupts until a pending GIC issue
 * is sorted out */
#define NR_MSM_IRQS 288
#define NR_GPIO_IRQS 152
#define NR_PM8921_IRQS 256
#define NR_PM8821_IRQS 112
#define NR_WCD9XXX_IRQS 49
#define NR_TABLA_IRQS NR_WCD9XXX_IRQS
#define NR_GPIO_EXPANDER_IRQS 64
#ifdef CONFIG_MFD_MAX77693
#define MAX77693_IRQS 32
#endif
#ifdef CONFIG_PCI_MSI
#define NR_PCIE_MSI_IRQS 256
#define NR_BOARD_IRQS (NR_PM8921_IRQS + NR_PM8821_IRQS + \
		NR_WCD9XXX_IRQS + NR_GPIO_EXPANDER_IRQS + NR_PCIE_MSI_IRQS)
#else
#ifdef CONFIG_MFD_MAX77693
#define NR_BOARD_IRQS (NR_PM8921_IRQS + NR_PM8821_IRQS + \
		NR_WCD9XXX_IRQS + NR_GPIO_EXPANDER_IRQS + MAX77693_IRQS)
#else
#define NR_BOARD_IRQS (NR_PM8921_IRQS + NR_PM8821_IRQS + \
		NR_WCD9XXX_IRQS + NR_GPIO_EXPANDER_IRQS)
#endif
#endif
#define NR_TLMM_MSM_DIR_CONN_IRQ 8 /*Need to Verify this Count*/
#define NR_MSM_GPIOS NR_GPIO_IRQS

#if defined(CONFIG_ARCH_MSM7X30)
#include "irqs-7x30.h"
#elif defined(CONFIG_ARCH_QSD8X50)
#include "irqs-8x50.h"
#include "sirc.h"
#elif defined(CONFIG_ARCH_MSM_ARM11)
#include "irqs-7x00.h"
#elif defined (CONFIG_ARCH_MSM8960)
#include "irqs-8960.h"
#elif defined (CONFIG_ARCH_APQ8064)
#include "irqs-8064.h"
#else
#error "Unknown architecture specification"
#endif

#define NR_IRQS (NR_MSM_IRQS + NR_GPIO_IRQS + NR_BOARD_IRQS)
#define MSM_GPIO_TO_INT(n) (NR_MSM_IRQS + (n))
#define MSM_INT_TO_REG(base, irq) (base + irq / 32)

#endif
