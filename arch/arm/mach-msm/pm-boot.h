/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _ARCH_ARM_MACH_MSM_PM_BOOT_H
#define _ARCH_ARM_MACH_MSM_PM_BOOT_H

/* Added for backwards compatibility */
enum {
	MSM_PM_BOOT_CONFIG_TZ		     = 0,
	MSM_PM_BOOT_CONFIG_RESET_VECTOR_PHYS = 1,
	MSM_PM_BOOT_CONFIG_RESET_VECTOR_VIRT = 2,
	MSM_PM_BOOT_CONFIG_REMAP_BOOT_ADDR   = 3,
};

struct msm_pm_boot_platform_data {
	int mode;
	phys_addr_t  p_addr;
	void __iomem *v_addr;
};
/* ********************************* */

void msm_pm_boot_config_before_pc(unsigned int cpu, unsigned long entry);
void msm_pm_boot_config_after_pc(unsigned int cpu);

#endif
