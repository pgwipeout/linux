// SPDX-License-Identifier: GPL-2.0
/*
 * dwc3-of-simple.c - OF glue layer for simple integrations
 *
 * Copyright (c) 2015 Texas Instruments Incorporated - https://www.ti.com
 *
 * Author: Felipe Balbi <balbi@ti.com>
 *
 * This is a combination of the old dwc3-qcom.c by Ivan T. Ivanov
 * <iivanov@mm-sol.com> and the original patch adding support for Xilinx' SoC
 * by Subbaraya Sundeep Bhatta <subbaraya.sundeep.bhatta@xilinx.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include <linux/workqueue.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/phy.h>

#include "core.h"
#include "../host/xhci.h"


struct dwc3_of_simple {
	struct device		*dev;
	struct clk_bulk_data	*clks;
	struct dwc3		*dwc;
	struct usb_phy		*phy;
	struct notifier_block	reset_nb;
	struct work_struct	reset_work;
	struct mutex		lock;
	int			num_clocks;
	struct reset_control	*resets;
	bool			need_reset;
	bool			need_host_reset;
};

static int dwc3_of_simple_host_reset_notifier(struct notifier_block *nb, unsigned long event, void *data)
{
	struct dwc3_of_simple	*simple = container_of(nb, struct dwc3_of_simple, reset_nb);

	schedule_work(&simple->reset_work);

	return NOTIFY_DONE;
}

static void dwc3_of_simple_host_reset_work(struct work_struct *work)
{
	struct dwc3_of_simple	*simple = container_of(work, struct dwc3_of_simple, reset_work);
	struct usb_hcd		*hcd = dev_get_drvdata(&simple->dwc->xhci->dev);
	struct usb_hcd		*shared_hcd = hcd->shared_hcd;
	struct xhci_hcd		*xhci = hcd_to_xhci(hcd);
	unsigned int		count = 0;

	mutex_lock(&simple->lock);

	if (hcd->state != HC_STATE_HALT) {
		usb_remove_hcd(shared_hcd);
		usb_remove_hcd(hcd);
	}

	if (simple->phy)
		usb_phy_shutdown(simple->phy);

	while (hcd->state != HC_STATE_HALT) {
		if (++count > 1000) {
			dev_err(simple->dev, "wait for HCD remove 1s timeout!\n");
			break;
		}
		usleep_range(1000, 1100);
	}

	if (hcd->state == HC_STATE_HALT) {
		xhci->shared_hcd = shared_hcd;
		usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
		usb_add_hcd(shared_hcd, hcd->irq, IRQF_SHARED);
	}

	if (simple->phy)
		usb_phy_init(simple->phy);

	mutex_unlock(&simple->lock);
	dev_dbg(simple->dev, "host reset complete\n");
}

static int dwc3_of_simple_probe(struct platform_device *pdev)
{
	struct dwc3_of_simple	*simple;
	struct device		*dev = &pdev->dev;
	struct device_node	*np = dev->of_node, *child, *node;
	struct platform_device	*child_pdev;

	int			ret;

	simple = devm_kzalloc(dev, sizeof(*simple), GFP_KERNEL);
	if (!simple)
		return -ENOMEM;

	platform_set_drvdata(pdev, simple);
	simple->dev = dev;

	/*
	 * Some controllers need to toggle the usb3-otg reset before trying to
	 * initialize the PHY, otherwise the PHY times out.
	 */
	if (of_device_is_compatible(np, "rockchip,rk3328-dwc3")) {
		simple->need_reset = true;
		simple->need_host_reset = true;
	}

	simple->resets = of_reset_control_array_get(np, false, true,
						    true);
	if (IS_ERR(simple->resets)) {
		ret = PTR_ERR(simple->resets);
		dev_err(dev, "failed to get device resets, err=%d\n", ret);
		return ret;
	}

	ret = reset_control_deassert(simple->resets);
	if (ret)
		goto err_resetc_put;

	ret = clk_bulk_get_all(simple->dev, &simple->clks);
	if (ret < 0)
		goto err_resetc_assert;

	simple->num_clocks = ret;
	ret = clk_bulk_prepare_enable(simple->num_clocks, simple->clks);
	if (ret)
		goto err_resetc_assert;

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret)
		goto err_clk_put;

	if (simple->need_host_reset) {
		child = of_get_child_by_name(np, "dwc3");
		if (!child) {
			dev_err(dev, "failed to find dwc3 core node\n");
			ret = -ENODEV;
			goto err_plat_depopulate;
		}

		child_pdev = of_find_device_by_node(child);
		if (!child_pdev) {
			dev_err(dev, "failed to get dwc3 core device\n");
			ret = -ENODEV;
			goto err_plat_depopulate;
		}

		simple->dwc = platform_get_drvdata(child_pdev);
		if (!simple->dwc || !simple->dwc->xhci) {
			ret = -EPROBE_DEFER;
			goto err_plat_depopulate;
		}

		node = of_parse_phandle(child, "usb-phy", 0);
		INIT_WORK(&simple->reset_work, dwc3_of_simple_host_reset_work);
		simple->reset_nb.notifier_call = dwc3_of_simple_host_reset_notifier;
		simple->phy = devm_usb_get_phy_by_node(dev, node, &simple->reset_nb);
		of_node_put(node);
		mutex_init(&simple->lock);
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	return 0;

err_plat_depopulate:
	of_platform_depopulate(dev);

err_clk_put:
	clk_bulk_disable_unprepare(simple->num_clocks, simple->clks);
	clk_bulk_put_all(simple->num_clocks, simple->clks);

err_resetc_assert:
	reset_control_assert(simple->resets);

err_resetc_put:
	reset_control_put(simple->resets);
	return ret;
}

static void __dwc3_of_simple_teardown(struct dwc3_of_simple *simple)
{
	of_platform_depopulate(simple->dev);

	clk_bulk_disable_unprepare(simple->num_clocks, simple->clks);
	clk_bulk_put_all(simple->num_clocks, simple->clks);
	simple->num_clocks = 0;

	reset_control_assert(simple->resets);

	reset_control_put(simple->resets);

	pm_runtime_disable(simple->dev);
	pm_runtime_put_noidle(simple->dev);
	pm_runtime_set_suspended(simple->dev);
}

static int dwc3_of_simple_remove(struct platform_device *pdev)
{
	struct dwc3_of_simple	*simple = platform_get_drvdata(pdev);

	__dwc3_of_simple_teardown(simple);

	return 0;
}

static void dwc3_of_simple_shutdown(struct platform_device *pdev)
{
	struct dwc3_of_simple	*simple = platform_get_drvdata(pdev);

	__dwc3_of_simple_teardown(simple);
}

static int __maybe_unused dwc3_of_simple_runtime_suspend(struct device *dev)
{
	struct dwc3_of_simple	*simple = dev_get_drvdata(dev);

	clk_bulk_disable(simple->num_clocks, simple->clks);

	return 0;
}

static int __maybe_unused dwc3_of_simple_runtime_resume(struct device *dev)
{
	struct dwc3_of_simple	*simple = dev_get_drvdata(dev);

	return clk_bulk_enable(simple->num_clocks, simple->clks);
}

static int __maybe_unused dwc3_of_simple_suspend(struct device *dev)
{
	struct dwc3_of_simple *simple = dev_get_drvdata(dev);

	if (simple->need_reset)
		reset_control_assert(simple->resets);

	return 0;
}

static int __maybe_unused dwc3_of_simple_resume(struct device *dev)
{
	struct dwc3_of_simple *simple = dev_get_drvdata(dev);

	if (simple->need_reset)
		reset_control_deassert(simple->resets);

	return 0;
}

static const struct dev_pm_ops dwc3_of_simple_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_of_simple_suspend, dwc3_of_simple_resume)
	SET_RUNTIME_PM_OPS(dwc3_of_simple_runtime_suspend,
			dwc3_of_simple_runtime_resume, NULL)
};

static const struct of_device_id of_dwc3_simple_match[] = {
	{ .compatible = "rockchip,rk3328-dwc3" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_dwc3_simple_match);

static struct platform_driver dwc3_of_simple_driver = {
	.probe		= dwc3_of_simple_probe,
	.remove		= dwc3_of_simple_remove,
	.shutdown	= dwc3_of_simple_shutdown,
	.driver		= {
		.name	= "dwc3-of-simple",
		.of_match_table = of_dwc3_simple_match,
		.pm	= &dwc3_of_simple_dev_pm_ops,
	},
};

module_platform_driver(dwc3_of_simple_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 OF Simple Glue Layer");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
