// SPDX-License-Identifier: GPL-2.0+
/*
 * Author: Dmitry Osipenko <digetx@gmail.com>
 * Copyright (C) 2019 GRATE-DRIVER project
 */

#include <dt-bindings/interconnect/tegra-icc.h>

#include <linux/device.h>
#include <linux/err.h>
#include <linux/interconnect-provider.h>
#include <linux/of.h>

#include <soc/tegra/mc.h>

static struct icc_node *
tegra_mc_of_icc_xlate_onecell(struct of_phandle_args *spec, void *data)
{
	struct icc_provider *provider = data;
	struct icc_node *node;

	list_for_each_entry(node, &provider->nodes, node_list) {
		if (node->id == spec->args[0])
			return node;
	}

	return ERR_PTR(-EINVAL);
}

static int tegra_mc_icc_set(struct icc_node *src, struct icc_node *dst)
{
	return 0;
}

static int tegra_mc_icc_aggregate(struct icc_node *node,
				  u32 tag, u32 avg_bw, u32 peak_bw,
				  u32 *agg_avg, u32 *agg_peak)
{
	*agg_avg = min((u64)avg_bw + (*agg_avg), (u64)U32_MAX);
	*agg_peak = max(*agg_peak, peak_bw);

	return 0;
}

/*
 * Memory Controller (MC) has few Memory Clients that are issuing memory
 * bandwidth allocation requests to the MC interconnect provider. The MC
 * provider aggregates the requests and then sends the aggregated request
 * up to the External Memory Controller (EMC) interconnect provider which
 * re-configures hardware interface to External Memory (EMEM) in accordance
 * to the required bandwidth.
 *
 * Memory interconnect topology:
 *
 *               +----+
 *   +-----+     |    |
 *   | GPU +---->+    |
 *   +-----+     |    |
 *               |    |     +-----+     +------+
 *    ...        | MC +---->+ EMC +---->+ EMEM |
 *               |    |     +-----+     +------+
 *  +------+     |    |
 *  | DISP +---->+    |
 *  +------+     |    |
 *               +----+
 */
int tegra_icc_mc_setup_interconnect(struct tegra_mc *mc)
{
	struct icc_provider *provider;
	struct icc_node *node, *tmp;
	unsigned int i;
	int err;

	provider = devm_kzalloc(mc->dev, sizeof(*provider), GFP_KERNEL);
	if (!provider)
		return -ENOMEM;

	provider->dev = mc->dev;
	provider->set = tegra_mc_icc_set;
	provider->data = provider;
	provider->xlate = tegra_mc_of_icc_xlate_onecell;
	provider->aggregate = tegra_mc_icc_aggregate;

	err = icc_provider_add(provider);
	if (err)
		return err;

	/* create Memory Controller node */
	node = icc_node_create(TEGRA_ICC_MC);
	err = PTR_ERR_OR_ZERO(node);
	if (err)
		goto del_provider;

	node->name = "MC";
	icc_node_add(node, provider);

	/* link Memory Controller with External Memory Controller */
	err = icc_link_create(node, TEGRA_ICC_EMC);
	if (err)
		goto destroy_nodes;

	for (i = 0; i < mc->soc->num_icc_nodes; i++) {
		/* create MC client node */
		node = icc_node_create(mc->soc->icc_nodes[i].id);
		err = PTR_ERR_OR_ZERO(node);
		if (err)
			goto destroy_nodes;

		node->name = mc->soc->icc_nodes[i].name;
		icc_node_add(node, provider);

		/* link Memory Client with Memory Controller */
		err = icc_link_create(node, TEGRA_ICC_MC);
		if (err)
			goto destroy_nodes;
	}

	return 0;

destroy_nodes:
	list_for_each_entry_safe(node, tmp, &provider->nodes, node_list) {
		icc_node_del(node);
		icc_node_destroy(node->id);
	}

del_provider:
	icc_provider_del(provider);

	return err;
}
