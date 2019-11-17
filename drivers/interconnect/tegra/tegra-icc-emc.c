// SPDX-License-Identifier: GPL-2.0+
/*
 * Author: Dmitry Osipenko <digetx@gmail.com>
 * Copyright (C) 2019 GRATE-DRIVER project
 */

#include <dt-bindings/interconnect/tegra-icc.h>

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interconnect-provider.h>
#include <linux/kernel.h>
#include <linux/of.h>

#include <soc/tegra/mc.h>

struct tegra_emc_provider {
	struct icc_provider provider;
	struct clk *clk;
	unsigned int dram_data_bus_width_bytes;
};

static inline struct tegra_emc_provider *
to_tegra_emc_provider(struct icc_provider *provider)
{
	return container_of(provider, struct tegra_emc_provider, provider);
}

static struct icc_node *
tegra_emc_of_icc_xlate_onecell(struct of_phandle_args *spec, void *data)
{
	struct icc_provider *provider = data;
	struct icc_node *node;

	list_for_each_entry(node, &provider->nodes, node_list) {
		if (node->id == spec->args[0])
			return node;
	}

	return ERR_PTR(-EINVAL);
}

static int tegra_emc_icc_set(struct icc_node *src, struct icc_node *dst)
{
	struct tegra_emc_provider *emc = to_tegra_emc_provider(dst->provider);
	unsigned long long rate = icc_units_to_bps(dst->avg_bw);
	unsigned int ddr = 2;
	int err;

	do_div(rate, ddr * emc->dram_data_bus_width_bytes);
	rate = min_t(u64, rate, U32_MAX);

	err = clk_set_min_rate(emc->clk, rate);
	if (err)
		return err;

	err = clk_set_rate(emc->clk, rate);
	if (err)
		return err;

	return 0;
}

static int tegra_emc_icc_aggregate(struct icc_node *node,
				   u32 tag, u32 avg_bw, u32 peak_bw,
				   u32 *agg_avg, u32 *agg_peak)
{
	*agg_avg = min((u64)avg_bw + (*agg_avg), (u64)U32_MAX);
	*agg_peak = max(*agg_peak, peak_bw);

	return 0;
}

int tegra_icc_emc_setup_interconnect(struct device *emc_dev,
				     unsigned int dram_data_bus_width_bytes)
{
	struct tegra_emc_provider *emc;
	struct icc_node *node, *tmp;
	int err;

	emc = devm_kzalloc(emc_dev, sizeof(*emc), GFP_KERNEL);
	if (!emc)
		return -ENOMEM;

	emc->clk = devm_clk_get(emc_dev, "emc");
	err = PTR_ERR_OR_ZERO(emc->clk);
	if (err)
		return err;

	emc->dram_data_bus_width_bytes = dram_data_bus_width_bytes;

	emc->provider.dev = emc_dev;
	emc->provider.set = tegra_emc_icc_set;
	emc->provider.data = &emc->provider;
	emc->provider.xlate = tegra_emc_of_icc_xlate_onecell;
	emc->provider.aggregate = tegra_emc_icc_aggregate;

	err = icc_provider_add(&emc->provider);
	if (err)
		return err;

	/* create External Memory Controller node */
	node = icc_node_create(TEGRA_ICC_EMC);
	err = PTR_ERR_OR_ZERO(node);
	if (err)
		goto del_provider;

	node->name = "EMC";
	icc_node_add(node, &emc->provider);

	/* link External Memory Controller with External Memory */
	err = icc_link_create(node, TEGRA_ICC_EMEM);
	if (err)
		goto destroy_nodes;

	/* create External Memory node */
	node = icc_node_create(TEGRA_ICC_EMEM);
	err = PTR_ERR_OR_ZERO(node);
	if (err)
		goto destroy_nodes;

	node->name = "EMEM";
	icc_node_add(node, &emc->provider);

	return 0;

destroy_nodes:
	list_for_each_entry_safe(node, tmp, &emc->provider.nodes, node_list) {
		icc_node_del(node);
		icc_node_destroy(node->id);
	}

del_provider:
	icc_provider_del(&emc->provider);

	return err;
}
