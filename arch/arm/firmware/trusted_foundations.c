/*
 * Trusted Foundations support for ARM CPUs
 *
 * Copyright (c) 2013, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <asm/io.h>
#include <asm/firmware.h>
#include <asm/outercache.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/trusted_foundations.h>

#define TF_CACHE_MAINT           0xfffff100
#define TF_SET_CPU_BOOT_ADDR_SMC 0xfffff200

#define TF_CACHE_INIT		1
#define TF_CACHE_FLUSH		2
#define TF_CACHE_REENABLE	4

#define TF_CPU_PM		0xfffffffc
#define TF_CPU_PM_S3		0xffffffe3
#define TF_CPU_PM_S2		0xffffffe6
#define TF_CPU_PM_S2_NO_MC_CLK	0xffffffe5
#define TF_CPU_PM_S1		0xffffffe4
#define TF_CPU_PM_S1_NOFLUSH_L2	0xffffffe7

static unsigned long cpu_boot_addr;

static void tf_generic_smc(u32 type, u32 arg1, u32 arg2)
{
	register u32 r0 asm("r0") = type;
	register u32 r1 asm("r1") = arg1;
	register u32 r2 asm("r2") = arg2;

	asm volatile(
		".arch_extension	sec\n\t"
		"stmfd	sp!, {r4 - r11}\n\t"
		__asmeq("%0", "r0")
		__asmeq("%1", "r1")
		__asmeq("%2", "r2")
		"mov	r3, #0\n\t"
		"mov	r4, #0\n\t"
		"smc	#0\n\t"
		"ldmfd	sp!, {r4 - r11}\n\t"
		:
		: "r" (r0), "r" (r1), "r" (r2)
		: "memory", "r3", "r12", "lr");
}

static int tf_set_cpu_boot_addr(int cpu, unsigned long boot_addr)
{
	cpu_boot_addr = boot_addr;
	tf_generic_smc(TF_SET_CPU_BOOT_ADDR_SMC, cpu_boot_addr, 0);

	return 0;
}

static int tf_prepare_idle(void)
{
	tf_generic_smc(TF_CPU_PM, TF_CPU_PM_S1_NOFLUSH_L2, cpu_boot_addr);

	return 0;
}

#ifdef CONFIG_CACHE_L2X0
static void tf_write_sec(unsigned long val, unsigned reg)
{
	unsigned long cur = readl_relaxed(l2x0_base + reg);

	pr_warn("TF: ignoring write_sec[0x%x]: 0x%08lx -> 0x%08lx\n",
		reg, cur, val);
}

static void tf_disable_cache(void)
{
	tf_generic_smc(TF_CACHE_MAINT, TF_CACHE_FLUSH, l2x0_way_mask);
}

static void tf_resume_cache(void)
{
	unsigned long aux_val = readl_relaxed(l2x0_base + L2X0_AUX_CTRL);
	tf_generic_smc(TF_CACHE_MAINT, TF_CACHE_REENABLE, aux_val);
}

static void tf_configure_cache(const struct l2x0_regs *regs)
{
	outer_cache.disable = tf_disable_cache;
	outer_cache.resume = tf_resume_cache;
}

static int tf_init_cache(void)
{
	tf_generic_smc(TF_CACHE_MAINT, TF_CACHE_INIT, 0);

	outer_cache.write_sec = tf_write_sec;
	outer_cache.configure = tf_configure_cache;
	return 0;
}
#endif /* CONFIG_CACHE_L2X0 */

static const struct firmware_ops trusted_foundations_ops = {
	.set_cpu_boot_addr = tf_set_cpu_boot_addr,
	.prepare_idle = tf_prepare_idle,
#ifdef CONFIG_CACHE_L2X0
	.l2x0_init = tf_init_cache,
#endif
};

void register_trusted_foundations(struct trusted_foundations_platform_data *pd)
{
	/*
	 * we are not using version information for now since currently
	 * supported SMCs are compatible with all TF releases
	 */
	register_firmware_ops(&trusted_foundations_ops);
}

void of_register_trusted_foundations(void)
{
	struct device_node *node;
	struct trusted_foundations_platform_data pdata;
	int err;

	node = of_find_compatible_node(NULL, NULL, "tlm,trusted-foundations");
	if (!node)
		return;

	err = of_property_read_u32(node, "tlm,version-major",
				   &pdata.version_major);
	if (err != 0)
		panic("Trusted Foundation: missing version-major property\n");
	err = of_property_read_u32(node, "tlm,version-minor",
				   &pdata.version_minor);
	if (err != 0)
		panic("Trusted Foundation: missing version-minor property\n");
	register_trusted_foundations(&pdata);

	of_node_put(node);
}
