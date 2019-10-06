// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016, NVIDIA Corporation
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/chipidea.h>

#include "../host/ehci.h"
#include "ci.h"

#define TEGRA_USB_DMA_ALIGN 32

struct tegra_udc {
	struct ci_hdrc_platform_data data;
	struct platform_device *dev;

	struct usb_phy *phy;
	struct clk *clk;
};

struct tegra_udc_soc_info {
	unsigned long flags;
};

struct tegra_dma_aligned_buffer {
	void *kmalloc_ptr;
	void *old_xfer_buffer;
	u8 data[0];
};

static int tegra_ehci_internal_port_reset(struct ehci_hcd *ehci,
					u32 __iomem *portsc_reg)
{
	u32 saved_usbintr, temp;
	unsigned int i, tries;
	unsigned long flags;
	int retval = 0;

	spin_lock_irqsave(&ehci->lock, flags);
	saved_usbintr = ehci_readl(ehci, &ehci->regs->intr_enable);
	/* disable USB interrupt */
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	spin_unlock_irqrestore(&ehci->lock, flags);

	/*
	 * Here we have to do Port Reset at most twice for
	 * Port Enable bit to be set.
	 */
	for (i = 0; i < 2; i++) {
		temp = ehci_readl(ehci, portsc_reg);
		temp |= PORT_RESET;
		ehci_writel(ehci, temp, portsc_reg);
		mdelay(10);
		temp &= ~PORT_RESET;
		ehci_writel(ehci, temp, portsc_reg);
		mdelay(1);
		tries = 100;
		do {
			mdelay(1);
			/*
			 * Up to this point, Port Enable bit is
			 * expected to be set after 2 ms waiting.
			 * USB1 usually takes extra 45 ms, for safety,
			 * we take 100 ms as timeout.
			 */
			temp = ehci_readl(ehci, portsc_reg);
		} while (!(temp & PORT_PE) && tries--);
		if (temp & PORT_PE)
			break;
	}
	if (i == 2)
		retval = -ETIMEDOUT;

	/*
	 * Clear Connect Status Change bit if it's set.
	 * We can't clear PORT_PEC. It will also cause PORT_PE to be cleared.
	 */
	if (temp & PORT_CSC)
		ehci_writel(ehci, PORT_CSC, portsc_reg);

	/*
	 * Write to clear any interrupt status bits that might be set
	 * during port reset.
	 */
	temp = ehci_readl(ehci, &ehci->regs->status);
	ehci_writel(ehci, temp, &ehci->regs->status);

	/* restore original interrupt enable bits */
	ehci_writel(ehci, saved_usbintr, &ehci->regs->intr_enable);
	return retval;
}

static void tegra_free_dma_aligned_buffer(struct urb *urb)
{
	struct tegra_dma_aligned_buffer *temp;
	size_t length;

	if (!(urb->transfer_flags & URB_ALIGNED_TEMP_BUFFER))
		return;

	temp = container_of(urb->transfer_buffer,
		struct tegra_dma_aligned_buffer, data);

	if (usb_urb_dir_in(urb)) {
		if (usb_pipeisoc(urb->pipe))
			length = urb->transfer_buffer_length;
		else
			length = urb->actual_length;

		memcpy(temp->old_xfer_buffer, temp->data, length);
	}
	urb->transfer_buffer = temp->old_xfer_buffer;
	kfree(temp->kmalloc_ptr);

	urb->transfer_flags &= ~URB_ALIGNED_TEMP_BUFFER;
}

static int tegra_alloc_dma_aligned_buffer(struct urb *urb, gfp_t mem_flags)
{
	struct tegra_dma_aligned_buffer *temp, *kmalloc_ptr;
	size_t kmalloc_size;

	if (urb->num_sgs || urb->sg ||
		urb->transfer_buffer_length == 0 ||
		!((uintptr_t)urb->transfer_buffer & (TEGRA_USB_DMA_ALIGN - 1)))
		return 0;

	/* Allocate a buffer with enough padding for alignment */
	kmalloc_size = urb->transfer_buffer_length +
		sizeof(struct tegra_dma_aligned_buffer) + TEGRA_USB_DMA_ALIGN - 1;

	kmalloc_ptr = kmalloc(kmalloc_size, mem_flags);
	if (!kmalloc_ptr)
		return -ENOMEM;

	/* Position our struct dma_aligned_buffer such that data is aligned */
	temp = PTR_ALIGN(kmalloc_ptr + 1, TEGRA_USB_DMA_ALIGN) - 1;
	temp->kmalloc_ptr = kmalloc_ptr;
	temp->old_xfer_buffer = urb->transfer_buffer;
	if (usb_urb_dir_out(urb))
		memcpy(temp->data, urb->transfer_buffer,
			urb->transfer_buffer_length);
	urb->transfer_buffer = temp->data;

	urb->transfer_flags |= URB_ALIGNED_TEMP_BUFFER;

	return 0;
}

static int tegra_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb,
				      gfp_t mem_flags)
{
	int ret;

	ret = tegra_alloc_dma_aligned_buffer(urb, mem_flags);
	if (ret)
		return ret;

	ret = usb_hcd_map_urb_for_dma(hcd, urb, mem_flags);
	if (ret)
		tegra_free_dma_aligned_buffer(urb);

	return ret;
}

static void tegra_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{
	usb_hcd_unmap_urb_for_dma(hcd, urb);
	tegra_free_dma_aligned_buffer(urb);
}

static const struct tegra_udc_soc_info tegra_udc_soc_info = {
	.flags = CI_HDRC_REQUIRES_ALIGNED_DMA,
};

static const struct of_device_id tegra_udc_of_match[] = {
	{
		.compatible = "nvidia,tegra20-udc",
		.data = &tegra_udc_soc_info,
	}, {
		.compatible = "nvidia,tegra30-udc",
		.data = &tegra_udc_soc_info,
	}, {
		.compatible = "nvidia,tegra114-udc",
		.data = &tegra_udc_soc_info,
	}, {
		.compatible = "nvidia,tegra124-udc",
		.data = &tegra_udc_soc_info,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, tegra_udc_of_match);

static int tegra_udc_probe(struct platform_device *pdev)
{
	const struct tegra_udc_soc_info *soc;
	struct tegra_udc *udc;
	int err;

	udc = devm_kzalloc(&pdev->dev, sizeof(*udc), GFP_KERNEL);
	if (!udc)
		return -ENOMEM;

	soc = of_device_get_match_data(&pdev->dev);
	if (!soc) {
		dev_err(&pdev->dev, "failed to match OF data\n");
		return -EINVAL;
	}

	/* check the dual mode and warn about bad configurations */
	switch (usb_get_dr_mode(&pdev->dev)) {
	case USB_DR_MODE_HOST:
		dev_dbg(&pdev->dev, "dr_mode is set to host\n");
		udc->data.dr_mode = USB_DR_MODE_HOST;
		break;

	case USB_DR_MODE_UNKNOWN:
		dev_warn(&pdev->dev, "dr_mode is unset or unknown, setting host mode\n");
		udc->data.dr_mode = USB_DR_MODE_HOST;
		break;

	case USB_DR_MODE_PERIPHERAL:
		dev_dbg(&pdev->dev, "dr_mode is set to peripheral\n");
		udc->data.dr_mode = USB_DR_MODE_PERIPHERAL;
		break;

	case USB_DR_MODE_OTG:
		dev_err(&pdev->dev, "dr_mode is otg, tegra-udc does not support otg at this time\n");
		return -EINVAL;
	}

	udc->phy = devm_usb_get_phy_by_phandle(&pdev->dev, "nvidia,phy", 0);
	if (IS_ERR(udc->phy)) {
		err = PTR_ERR(udc->phy);
		dev_err(&pdev->dev, "failed to get PHY: %d\n", err);
		return err;
	}

	udc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(udc->clk)) {
		err = PTR_ERR(udc->clk);
		dev_err(&pdev->dev, "failed to get clock: %d\n", err);
		return err;
	}

	err = clk_prepare_enable(udc->clk);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to enable clock: %d\n", err);
		return err;
	}

	/*
	 * Tegra's USB PHY driver doesn't implement optional phy_init()
	 * hook, so we have to power on UDC controller before ChipIdea
	 * driver initialization kicks in.
	 */
	usb_phy_set_suspend(udc->phy, 0);

	/* setup and register ChipIdea HDRC device */
	udc->data.name = "tegra-udc";
	udc->data.flags = soc->flags;
	udc->data.usb_phy = udc->phy;
	udc->data.capoffset = DEF_CAPOFFSET;
	udc->data.map_urb_for_dma = tegra_map_urb_for_dma;
	udc->data.unmap_urb_for_dma = tegra_unmap_urb_for_dma;

	/* check the double reset flag */
	if (of_property_read_bool(pdev->dev.of_node,
				"nvidia,needs-double-reset"))
		udc->data.port_reset = tegra_ehci_internal_port_reset;

	udc->dev = ci_hdrc_add_device(&pdev->dev, pdev->resource,
				      pdev->num_resources, &udc->data);
	if (IS_ERR(udc->dev)) {
		err = PTR_ERR(udc->dev);
		dev_err(&pdev->dev, "failed to add HDRC device: %d\n", err);
		goto fail_power_off;
	}

	platform_set_drvdata(pdev, udc);

	return 0;

fail_power_off:
	usb_phy_set_suspend(udc->phy, 1);
	clk_disable_unprepare(udc->clk);
	return err;
}

static int tegra_udc_remove(struct platform_device *pdev)
{
	struct tegra_udc *udc = platform_get_drvdata(pdev);

	ci_hdrc_remove_device(udc->dev);
	usb_phy_set_suspend(udc->phy, 1);
	clk_disable_unprepare(udc->clk);

	return 0;
}

static struct platform_driver tegra_udc_driver = {
	.driver = {
		.name = "tegra-udc",
		.of_match_table = tegra_udc_of_match,
	},
	.probe = tegra_udc_probe,
	.remove = tegra_udc_remove,
};
module_platform_driver(tegra_udc_driver);

MODULE_DESCRIPTION("NVIDIA Tegra USB device mode driver");
MODULE_AUTHOR("Thierry Reding <treding@nvidia.com>");
MODULE_ALIAS("platform:tegra-udc");
MODULE_LICENSE("GPL v2");
