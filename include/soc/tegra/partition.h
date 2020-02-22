/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __SOC_TEGRA_PARTITION_H__
#define __SOC_TEGRA_PARTITION_H__

#include <linux/types.h>

#ifdef CONFIG_TEGRA_PARTITION
void tegra_partition_table_setup(sector_t logical_sector_address,
				 sector_t logical_sectors_num);
#else
static inline void tegra_partition_table_setup(sector_t logical_sector_address,
					       sector_t logical_sectors_num)
{
}
#endif

#endif /* __SOC_TEGRA_PARTITION_H__ */
