// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Rockchip USB 3.0 PHY with Innosilicon IP block driver
 *
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_clk.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/usb/phy.h>
#include <linux/uaccess.h>

#define USB3PHY_PORT_NUM	2
#define BIT_WRITEABLE_SHIFT	16
#define SCHEDULE_DELAY	(60 * HZ)

#define USB3PHY_APB_RST	BIT(0)
#define USB3PHY_POR_RST	BIT(1)
#define USB3PHY_MAC_RST	BIT(2)

#define DRIVER_NAME "phy-rockchip-inno-usb3"

struct rockchip_usb3phy;
struct rockchip_usb3phy_port;

enum rockchip_usb3phy_type {
	USB3PHY_TYPE_PIPE,
	USB3PHY_TYPE_UTMI,
};

enum rockchip_usb3phy_pipe_pwr {
	PIPE_PWR_P0	= 0,
	PIPE_PWR_P1	= 1,
	PIPE_PWR_P2	= 2,
	PIPE_PWR_P3	= 3,
	PIPE_PWR_MAX	= 4,
};

enum rockchip_usb3phy_rest_req {
	USB3_POR_RSTN	= 0,
	U2_POR_RSTN	= 1,
	PIPE_MAC_RSTN	= 2,
	UTMI_MAC_RSTN	= 3,
	PIPE_APB_RSTN	= 4,
	UTMI_APB_RSTN	= 5,
	USB3PHY_RESET_MAX	= 6,
};

enum rockchip_usb3phy_utmi_state {
	PHY_UTMI_HS_ONLINE	= 0,
	PHY_UTMI_DISCONNECT	= 1,
	PHY_UTMI_CONNECT	= 2,
	PHY_UTMI_FS_LS_ONLINE	= 4,
};

/*
 * @disable: disable value
 * @enable: enable value
 */
struct usb3phy_reg {
	unsigned int	offset;
	unsigned int	bitend;
	unsigned int	bitstart;
	unsigned int	disable;
	unsigned int	enable;
};

struct rockchip_usb3phy_grfcfg {
	struct usb3phy_reg	um_suspend;
	struct usb3phy_reg	ls_det_en;
	struct usb3phy_reg	ls_det_st;
	struct usb3phy_reg	um_ls;
	struct usb3phy_reg	um_hstdct;
	struct usb3phy_reg	usb3_disable;
	struct usb3phy_reg	pp_pwr_st;
	struct usb3phy_reg	pp_pwr_en[PIPE_PWR_MAX];
};

/**
 * struct rockchip_usb3phy_apbcfg: usb3-phy apb configuration.
 * @u2_pre_emp: usb2-phy pre-emphasis tuning.
 * @u2_pre_emp_sth: usb2-phy pre-emphasis strength tuning.
 * @u2_odt_tuning: usb2-phy odt 45ohm tuning.
 */
struct rockchip_usb3phy_apbcfg {
	unsigned int	u2_pre_emp;
	unsigned int	u2_pre_emp_sth;
	unsigned int	u2_odt_tuning;
};

struct rockchip_usb3phy_cfg {
	unsigned int reg;
	const struct rockchip_usb3phy_grfcfg grfcfg;

	int (*phy_pipe_power)(struct rockchip_usb3phy *usb3phy,
			      struct rockchip_usb3phy_port *usb3phy_port,
			      bool on);
	int (*phy_tuning)(struct rockchip_usb3phy *usb3phy,
			  struct rockchip_usb3phy_port *usb3phy_port,
			  struct device_node *child_np);
};

struct rockchip_usb3phy_port {
	struct phy	*phy;
	struct regmap	*regs;
	unsigned int	index;
	unsigned char	type;
	bool		suspended;
	bool		refclk_25m_quirk;
	struct mutex	mutex; /* mutex for updating register */
	struct delayed_work	um_sm_work;
};

struct rockchip_usb3phy {
	struct device *dev;
	struct regmap *grf;
	struct regmap *usb3phy_grf;
	int um_ls_irq;
	struct clk **clks;
	int num_clocks;
	struct dentry *root;
	struct regulator *vbus;
	struct reset_control *rsts[USB3PHY_RESET_MAX];
	struct rockchip_usb3phy_apbcfg apbcfg;
	const struct rockchip_usb3phy_cfg *cfgs;
	struct rockchip_usb3phy_port ports[USB3PHY_PORT_NUM];
	struct usb_phy usb_phy;
	bool vbus_enabled;
};

static inline int property_enable(struct regmap *base,
			      const struct usb3phy_reg *reg, bool en)
{
	unsigned int val, mask, tmp;

	tmp = en ? reg->enable : reg->disable;
	mask = GENMASK(reg->bitend, reg->bitstart);
	val = (tmp << reg->bitstart) | (mask << BIT_WRITEABLE_SHIFT);

	return regmap_write(base, reg->offset, val);
}

static inline bool param_exped(struct regmap *regs,
			       const struct usb3phy_reg *reg,
			       unsigned int value)
{
	int ret;
	unsigned int tmp, orig;
	unsigned int mask = GENMASK(reg->bitend, reg->bitstart);

	ret = regmap_read(regs, reg->offset, &orig);
	if (ret)
		return false;

	tmp = (orig & mask) >> reg->bitstart;
	return tmp == value;
}

static int rockchip_set_vbus_power(struct rockchip_usb3phy *usb3phy, bool en)
{
	int ret = 0;

	if (!usb3phy->vbus)
		return 0;

	if (en && !usb3phy->vbus_enabled) {
		ret = regulator_enable(usb3phy->vbus);
		if (ret)
			dev_err(usb3phy->dev,
				"Failed to enable VBUS supply\n");
	} else if (!en && usb3phy->vbus_enabled) {
		ret = regulator_disable(usb3phy->vbus);
	}

	if (ret == 0)
		usb3phy->vbus_enabled = en;

	return ret;
}

static const char *get_rest_name(enum rockchip_usb3phy_rest_req rst)
{
	switch (rst) {
	case U2_POR_RSTN:
		return "usb3phy-u2-por";
	case USB3_POR_RSTN:
		return "usb3phy-usb3-por";
	case PIPE_MAC_RSTN:
		return "usb3phy-pipe-mac";
	case UTMI_MAC_RSTN:
		return "usb3phy-utmi-mac";
	case UTMI_APB_RSTN:
		return "usb3phy-utmi-apb";
	case PIPE_APB_RSTN:
		return "usb3phy-pipe-apb";
	default:
		return "invalid";
	}
}

static void rockchip_usb3phy_rest_deassert(struct rockchip_usb3phy *usb3phy,
					 unsigned int flag)
{
	int rst;

	if (flag & USB3PHY_APB_RST) {
		dev_dbg(usb3phy->dev, "deassert APB bus interface reset\n");
		for (rst = PIPE_APB_RSTN; rst <= UTMI_APB_RSTN; rst++) {
			if (usb3phy->rsts[rst])
				reset_control_deassert(usb3phy->rsts[rst]);
		}
	}

	if (flag & USB3PHY_POR_RST) {
		usleep_range(12, 15);
		dev_dbg(usb3phy->dev, "deassert u2 and usb3 phy power on reset\n");
		for (rst = USB3_POR_RSTN; rst <= U2_POR_RSTN; rst++) {
			if (usb3phy->rsts[rst])
				reset_control_deassert(usb3phy->rsts[rst]);
		}
	}

	if (flag & USB3PHY_MAC_RST) {
		usleep_range(1200, 1500);
		dev_dbg(usb3phy->dev, "deassert pipe and utmi MAC reset\n");
		for (rst = PIPE_MAC_RSTN; rst <= UTMI_MAC_RSTN; rst++)
			if (usb3phy->rsts[rst])
				reset_control_deassert(usb3phy->rsts[rst]);
	}
}

static void rockchip_usb3phy_rest_assert(struct rockchip_usb3phy *usb3phy)
{
	int rst;

	dev_dbg(usb3phy->dev, "assert usb3phy reset\n");
	for (rst = 0; rst < USB3PHY_RESET_MAX; rst++)
		if (usb3phy->rsts[rst])
			reset_control_assert(usb3phy->rsts[rst]);
}

static int rockchip_usb3phy_clk_enable(struct rockchip_usb3phy *usb3phy)
{
	int ret, clk;

	for (clk = 0; clk < usb3phy->num_clocks && usb3phy->clks[clk]; clk++) {
		ret = clk_prepare_enable(usb3phy->clks[clk]);
		if (ret)
			goto err_disable_clks;
	}
	return 0;

err_disable_clks:
	while (--clk >= 0)
		clk_disable_unprepare(usb3phy->clks[clk]);
	return ret;
}

static void rockchip_usb3phy_clk_disable(struct rockchip_usb3phy *usb3phy)
{
	int clk;

	for (clk = usb3phy->num_clocks - 1; clk >= 0; clk--)
		if (usb3phy->clks[clk])
			clk_disable_unprepare(usb3phy->clks[clk]);
}

static int rockchip_usb3phy_init(struct phy *phy)
{
	return 0;
}

static int rockchip_usb3phy_exit(struct phy *phy)
{
	return 0;
}

static int rockchip_usb3phy_power_on(struct phy *phy)
{
	struct rockchip_usb3phy_port *usb3phy_port = phy_get_drvdata(phy);
	struct rockchip_usb3phy *usb3phy = dev_get_drvdata(phy->dev.parent);
	int ret;

	dev_info(&usb3phy_port->phy->dev, "usb3phy %s power on\n",
		 (usb3phy_port->type == USB3PHY_TYPE_UTMI) ? "u2" : "usb3");

	if (!usb3phy_port->suspended)
		return 0;

	ret = rockchip_usb3phy_clk_enable(usb3phy);
	if (ret)
		return ret;

	if (usb3phy_port->type == USB3PHY_TYPE_UTMI) {
		property_enable(usb3phy->usb3phy_grf,
			    &usb3phy->cfgs->grfcfg.um_suspend, false);
	} else {
		/* current in p2 ? */
		if (param_exped(usb3phy->usb3phy_grf,
				&usb3phy->cfgs->grfcfg.pp_pwr_st, PIPE_PWR_P2))
			goto done;

		if (usb3phy->cfgs->phy_pipe_power) {
			dev_dbg(usb3phy->dev, "do pipe power up\n");
			usb3phy->cfgs->phy_pipe_power(usb3phy, usb3phy_port, true);
		}

		/* exit to p0 */
		property_enable(usb3phy->usb3phy_grf,
			    &usb3phy->cfgs->grfcfg.pp_pwr_en[PIPE_PWR_P0], true);
		usleep_range(90, 100);

		/* enter to p2 from p0 */
		property_enable(usb3phy->usb3phy_grf,
			    &usb3phy->cfgs->grfcfg.pp_pwr_en[PIPE_PWR_P2],
			    false);
		udelay(3);
	}

done:
	rockchip_set_vbus_power(usb3phy, true);
	usb3phy_port->suspended = false;
	return 0;
}

static int rockchip_usb3phy_power_off(struct phy *phy)
{
	struct rockchip_usb3phy_port *usb3phy_port = phy_get_drvdata(phy);
	struct rockchip_usb3phy *usb3phy = dev_get_drvdata(phy->dev.parent);

	dev_info(&usb3phy_port->phy->dev, "usb3phy %s power off\n",
		 (usb3phy_port->type == USB3PHY_TYPE_UTMI) ? "u2" : "usb3");

	if (usb3phy_port->suspended)
		return 0;

	if (usb3phy_port->type == USB3PHY_TYPE_UTMI) {
		property_enable(usb3phy->usb3phy_grf,
			    &usb3phy->cfgs->grfcfg.um_suspend, true);
	} else {
		/* current in p3 ? */
		if (param_exped(usb3phy->usb3phy_grf,
				&usb3phy->cfgs->grfcfg.pp_pwr_st, PIPE_PWR_P3))
			goto done;

		/* exit to p0 */
		property_enable(usb3phy->usb3phy_grf,
			    &usb3phy->cfgs->grfcfg.pp_pwr_en[PIPE_PWR_P0], true);
		udelay(2);

		/* enter to p3 from p0 */
		property_enable(usb3phy->usb3phy_grf,
			    &usb3phy->cfgs->grfcfg.pp_pwr_en[PIPE_PWR_P3], true);
		udelay(6);

		if (usb3phy->cfgs->phy_pipe_power) {
			dev_dbg(usb3phy->dev, "do pipe power down\n");
			usb3phy->cfgs->phy_pipe_power(usb3phy, usb3phy_port, false);
		}
	}

done:
	rockchip_usb3phy_clk_disable(usb3phy);
	usb3phy_port->suspended = true;
	return 0;
}

static __maybe_unused
struct phy *rockchip_usb3phy_xlate(struct device *dev,
				 struct of_phandle_args *args)
{
	struct rockchip_usb3phy *usb3phy = dev_get_drvdata(dev);
	struct rockchip_usb3phy_port *usb3phy_port = NULL;
	struct device_node *phy_np = args->np;
	int index;

	if (args->args_count != 1) {
		dev_err(dev, "invalid number of cells in 'phy' property\n");
		return ERR_PTR(-EINVAL);
	}

	for (index = 0; index < USB3PHY_PORT_NUM; index++) {
		if (phy_np == usb3phy->ports[index].phy->dev.of_node) {
			usb3phy_port = &usb3phy->ports[index];
			break;
		}
	}

	if (!usb3phy_port) {
		dev_err(dev, "failed to find appropriate phy\n");
		return ERR_PTR(-EINVAL);
	}

	return usb3phy_port->phy;
}

static struct phy_ops rockchip_usb3phy_ops = {
	.init		= rockchip_usb3phy_init,
	.exit		= rockchip_usb3phy_exit,
	.power_on	= rockchip_usb3phy_power_on,
	.power_off	= rockchip_usb3phy_power_off,
	.owner		= THIS_MODULE,
};

/*
 * The function manage host-phy port state and suspend/resume phy port
 * to save power automatically.
 *
 * we rely on utmi_linestate and utmi_hostdisconnect to identify whether
 * devices is disconnect or not. Besides, we do not need care it is FS/LS
 * disconnected or HS disconnected, actually, we just only need get the
 * device is disconnected at last through rearm the delayed work,
 * to suspend the phy port in _PHY_STATE_DISCONNECT_ case.
 */
static void rockchip_usb3phy_um_sm_work(struct work_struct *work)
{
	struct rockchip_usb3phy_port *usb3phy_port =
		container_of(work, struct rockchip_usb3phy_port, um_sm_work.work);
	struct rockchip_usb3phy *usb3phy =
		dev_get_drvdata(usb3phy_port->phy->dev.parent);
	unsigned int sh = usb3phy->cfgs->grfcfg.um_hstdct.bitend -
			usb3phy->cfgs->grfcfg.um_hstdct.bitstart + 1;
	unsigned int ul, uhd, state;
	unsigned int ul_mask, uhd_mask;
	int ret;

	mutex_lock(&usb3phy_port->mutex);

	ret = regmap_read(usb3phy->usb3phy_grf,
			  usb3phy->cfgs->grfcfg.um_ls.offset, &ul);
	if (ret < 0)
		goto next_schedule;

	ret = regmap_read(usb3phy->usb3phy_grf,
			  usb3phy->cfgs->grfcfg.um_hstdct.offset, &uhd);
	if (ret < 0)
		goto next_schedule;

	uhd_mask = GENMASK(usb3phy->cfgs->grfcfg.um_hstdct.bitend,
			   usb3phy->cfgs->grfcfg.um_hstdct.bitstart);
	ul_mask = GENMASK(usb3phy->cfgs->grfcfg.um_ls.bitend,
			  usb3phy->cfgs->grfcfg.um_ls.bitstart);

	/* stitch on um_ls and um_hstdct as phy state */
	state = ((uhd & uhd_mask) >> usb3phy->cfgs->grfcfg.um_hstdct.bitstart) |
		(((ul & ul_mask) >> usb3phy->cfgs->grfcfg.um_ls.bitstart) << sh);

	switch (state) {
	case PHY_UTMI_HS_ONLINE:
		dev_dbg(&usb3phy_port->phy->dev, "HS online\n");
		break;
	case PHY_UTMI_FS_LS_ONLINE:
		/*
		 * For FS/LS device, the online state share with connect state
		 * from um_ls and um_hstdct register, so we distinguish
		 * them via suspended flag.
		 *
		 * Plus, there are two cases, one is D- Line pull-up, and D+
		 * line pull-down, the state is 4; another is D+ line pull-up,
		 * and D- line pull-down, the state is 2.
		 */
		if (!usb3phy_port->suspended) {
			/* D- line pull-up, D+ line pull-down */
			dev_dbg(&usb3phy_port->phy->dev, "FS/LS online\n");
			break;
		}
		/* fall through */
	case PHY_UTMI_CONNECT:
		if (usb3phy_port->suspended) {
			dev_dbg(&usb3phy_port->phy->dev, "Connected\n");
			rockchip_usb3phy_power_on(usb3phy_port->phy);
			usb3phy_port->suspended = false;
		} else {
			/* D+ line pull-up, D- line pull-down */
			dev_dbg(&usb3phy_port->phy->dev, "FS/LS online\n");
		}
		break;
	case PHY_UTMI_DISCONNECT:
		if (!usb3phy_port->suspended) {
			dev_dbg(&usb3phy_port->phy->dev, "Disconnected\n");
			rockchip_usb3phy_power_off(usb3phy_port->phy);
			usb3phy_port->suspended = true;
		}

		/*
		 * activate the linestate detection to get the next device
		 * plug-in irq.
		 */
		property_enable(usb3phy->usb3phy_grf,
			    &usb3phy->cfgs->grfcfg.ls_det_st, true);
		property_enable(usb3phy->usb3phy_grf,
			    &usb3phy->cfgs->grfcfg.ls_det_en, true);

		/*
		 * we don't need to rearm the delayed work when the phy port
		 * is suspended.
		 */
		mutex_unlock(&usb3phy_port->mutex);
		return;
	default:
		dev_dbg(&usb3phy_port->phy->dev, "unknown phy state\n");
		break;
	}

next_schedule:
	mutex_unlock(&usb3phy_port->mutex);
	schedule_delayed_work(&usb3phy_port->um_sm_work, SCHEDULE_DELAY);
}

static irqreturn_t rockchip_usb3phy_um_ls_irq(int irq, void *data)
{
	struct rockchip_usb3phy_port *usb3phy_port = data;
	struct rockchip_usb3phy *usb3phy =
		dev_get_drvdata(usb3phy_port->phy->dev.parent);

	if (!param_exped(usb3phy->usb3phy_grf,
			 &usb3phy->cfgs->grfcfg.ls_det_st,
			 usb3phy->cfgs->grfcfg.ls_det_st.enable))
		return IRQ_NONE;

	dev_dbg(usb3phy->dev, "utmi linestate interrupt\n");
	mutex_lock(&usb3phy_port->mutex);

	/* disable linestate detect irq and clear its status */
	property_enable(usb3phy->usb3phy_grf, &usb3phy->cfgs->grfcfg.ls_det_en, false);
	property_enable(usb3phy->usb3phy_grf, &usb3phy->cfgs->grfcfg.ls_det_st, true);

	mutex_unlock(&usb3phy_port->mutex);

	/*
	 * In this case for host phy, a new device is plugged in, meanwhile,
	 * if the phy port is suspended, we need rearm the work to resume it
	 * and mange its states; otherwise, we just return irq handled.
	 */
	if (usb3phy_port->suspended) {
		dev_dbg(usb3phy->dev, "schedule utmi sm work\n");
		rockchip_usb3phy_um_sm_work(&usb3phy_port->um_sm_work.work);
	}

	return IRQ_HANDLED;
}

static int rockchip_usb3phy_parse_dt(struct rockchip_usb3phy *usb3phy,
				   struct platform_device *pdev)

{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret, i, clk;

	usb3phy->um_ls_irq = platform_get_irq_byname(pdev, "linestate");
	if (usb3phy->um_ls_irq < 0) {
		dev_err(dev, "get utmi linestate irq failed\n");
		return -ENXIO;
	}

	/* Get Vbus regulators */
	usb3phy->vbus = devm_regulator_get_optional(dev, "vbus");
	if (IS_ERR(usb3phy->vbus)) {
		ret = PTR_ERR(usb3phy->vbus);
		if (ret == -EPROBE_DEFER)
			return ret;

		dev_warn(dev, "Failed to get VBUS supply regulator\n");
		usb3phy->vbus = NULL;
	}

	usb3phy->num_clocks = of_clk_get_parent_count(np);
	if (usb3phy->num_clocks == 0)
		dev_warn(&pdev->dev, "no clks found in dt\n");

	usb3phy->clks = devm_kcalloc(dev, usb3phy->num_clocks,
				  sizeof(struct clk *), GFP_KERNEL);

	for (clk = 0; clk < usb3phy->num_clocks; clk++) {
		usb3phy->clks[clk] = of_clk_get(np, clk);
		if (IS_ERR(usb3phy->clks[clk])) {
			ret = PTR_ERR(usb3phy->clks[clk]);
			if (ret == -EPROBE_DEFER)
				goto err_put_clks;
			dev_err(&pdev->dev, "failed to get clks, %i\n",
				ret);
			usb3phy->clks[clk] = NULL;
			break;
		}
	}

	for (i = 0; i < USB3PHY_RESET_MAX; i++) {
		usb3phy->rsts[i] = devm_reset_control_get(dev, get_rest_name(i));
		if (IS_ERR(usb3phy->rsts[i])) {
			dev_info(dev, "no %s reset control specified\n",
				 get_rest_name(i));
			usb3phy->rsts[i] = NULL;
		}
	}

	return 0;

err_put_clks:
	while (--clk >= 0)
		clk_put(usb3phy->clks[clk]);
	return ret;
}

static const struct regmap_config rockchip_usb3phy_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.name = DRIVER_NAME,
};

static int rockchip_usb3phy_port_init(struct rockchip_usb3phy *usb3phy,
				    struct rockchip_usb3phy_port *usb3phy_port,
				    struct device_node *child_np)
{
	struct resource res;
	struct phy *phy;
	void __iomem *regs;
	int ret;

	dev_dbg(usb3phy->dev, "usb3phy port initialize\n");

	mutex_init(&usb3phy_port->mutex);
	usb3phy_port->suspended = true; /* initial status */

	phy = devm_phy_create(usb3phy->dev, child_np, &rockchip_usb3phy_ops);
	if (IS_ERR(phy)) {
		dev_err(usb3phy->dev, "failed to create phy\n");
		return PTR_ERR(phy);
	}

	usb3phy_port->phy = phy;

	ret = of_address_to_resource(child_np, 0, &res);
	if (ret) {
		dev_err(usb3phy->dev, "failed to get address resource(np-%pOFn)\n",
			child_np);
		return ret;
	}

	regs = devm_ioremap_resource(&usb3phy_port->phy->dev, &res);
	if (IS_ERR(regs)) {
		dev_err(usb3phy->dev, "failed to remap phy regs\n");
		return PTR_ERR(regs);
	}
	
	usb3phy_port->regs = devm_regmap_init_mmio(&usb3phy_port->phy->dev, regs,
						 &rockchip_usb3phy_regmap_config);
	
	if (IS_ERR(usb3phy_port->regs)) {
		dev_err(usb3phy->dev, "failed to create phy regmap\n");
		return PTR_ERR(usb3phy_port->regs);
	}

	if (of_node_name_eq(child_np, "pipe")) {
		usb3phy_port->type = USB3PHY_TYPE_PIPE;
		usb3phy_port->refclk_25m_quirk =
			of_property_read_bool(child_np,
					      "rockchip,refclk-25m-quirk");
	} else {
		usb3phy_port->type = USB3PHY_TYPE_UTMI;
		INIT_DELAYED_WORK(&usb3phy_port->um_sm_work,
				  rockchip_usb3phy_um_sm_work);

		ret = devm_request_threaded_irq(usb3phy->dev, usb3phy->um_ls_irq,
						NULL, rockchip_usb3phy_um_ls_irq,
						IRQF_ONESHOT, "rockchip_usb3phy",
						usb3phy_port);
		if (ret) {
			dev_err(usb3phy->dev, "failed to request utmi linestate irq handle\n");
			return ret;
		}
	}

	if (usb3phy->cfgs->phy_tuning) {
		dev_dbg(usb3phy->dev, "do usb3phy tuning\n");
		ret = usb3phy->cfgs->phy_tuning(usb3phy, usb3phy_port, child_np);
		if (ret)
			return ret;
	}

	phy_set_drvdata(usb3phy_port->phy, usb3phy_port);
	return 0;
}

static int rockchip_usb3phy_on_init(struct usb_phy *usb_phy)
{
	struct rockchip_usb3phy *usb3phy =
		container_of(usb_phy, struct rockchip_usb3phy, usb_phy);

	rockchip_usb3phy_rest_deassert(usb3phy, USB3PHY_POR_RST | USB3PHY_MAC_RST);
	return 0;
}

static void rockchip_usb3phy_on_shutdown(struct usb_phy *usb_phy)
{
	struct rockchip_usb3phy *usb3phy =
		container_of(usb_phy, struct rockchip_usb3phy, usb_phy);
	int rst;

	for (rst = 0; rst < USB3PHY_RESET_MAX; rst++)
		if (usb3phy->rsts[rst] && rst != UTMI_APB_RSTN &&
		    rst != PIPE_APB_RSTN)
			reset_control_assert(usb3phy->rsts[rst]);
	udelay(1);
}

static int rockchip_usb3phy_on_disconnect(struct usb_phy *usb_phy,
					enum usb_device_speed speed)
{
	struct rockchip_usb3phy *usb3phy =
		container_of(usb_phy, struct rockchip_usb3phy, usb_phy);

	dev_info(usb3phy->dev, "%s device has disconnected\n",
		 (speed == USB_SPEED_SUPER) ? "USB3" : "UW/U2/U1.1/U1");

	if (speed == USB_SPEED_SUPER)
		atomic_notifier_call_chain(&usb_phy->notifier, 0, NULL);

	return 0;
}

static int rockchip_usb3phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *child_np;
	struct phy_provider *provider;
	struct rockchip_usb3phy *usb3phy;
	const struct rockchip_usb3phy_cfg *phy_cfgs;
	const struct of_device_id *match;
	unsigned int reg[2];
	int index, ret;

	match = of_match_device(dev->driver->of_match_table, dev);
	if (!match || !match->data) {
		dev_err(dev, "phy-cfgs are not assigned!\n");
		return -EINVAL;
	}

	usb3phy = devm_kzalloc(dev, sizeof(*usb3phy), GFP_KERNEL);
	if (!usb3phy)
		return -ENOMEM;

	usb3phy->usb3phy_grf =
		syscon_regmap_lookup_by_phandle(np, "rockchip,usb3phygrf");
	if (IS_ERR(usb3phy->usb3phy_grf))
		return PTR_ERR(usb3phy->usb3phy_grf);

	usb3phy->grf =
		syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(usb3phy->grf)) {
		dev_err(dev, "Missing rockchip,grf property\n");
		return PTR_ERR(usb3phy->grf);
	}

	if (of_property_read_usb32_array(np, "reg", reg, 2)) {
		dev_err(dev, "the reg property is not assigned in %pOFn node\n",
			np);
		return -EINVAL;
	}

	usb3phy->dev = dev;
	usb3phy->vbus_enabled = false;
	phy_cfgs = match->data;
	platform_set_drvdata(pdev, usb3phy);

	/* find out a proper config which can be matched with dt. */
	index = 0;
	while (phy_cfgs[index].reg) {
		if (phy_cfgs[index].reg == reg[1]) {
			usb3phy->cfgs = &phy_cfgs[index];
			break;
		}

		++index;
	}

	if (!usb3phy->cfgs) {
		dev_err(dev, "no phy-cfgs can be matched with %pOFn node\n",
			np);
		return -EINVAL;
	}

	ret = rockchip_usb3phy_parse_dt(usb3phy, pdev);
	if (ret) {
		dev_err(dev, "parse dt failed, ret(%d)\n", ret);
		return ret;
	}

	ret = rockchip_usb3phy_clk_enable(usb3phy);
	if (ret) {
		dev_err(dev, "clk enable failed, ret(%d)\n", ret);
		return ret;
	}

	rockchip_usb3phy_rest_assert(usb3phy);
	rockchip_usb3phy_rest_deassert(usb3phy, USB3PHY_APB_RST | USB3PHY_POR_RST);

	index = 0;
	for_each_available_child_of_node(np, child_np) {
		struct rockchip_usb3phy_port *usb3phy_port = &usb3phy->ports[index];

		usb3phy_port->index = index;
		ret = rockchip_usb3phy_port_init(usb3phy, usb3phy_port, child_np);
		if (ret) {
			dev_err(dev, "usb3phy port init failed,ret(%d)\n", ret);
			goto put_child;
		}

		/* to prevent out of boundary */
		if (++index >= USB3PHY_PORT_NUM)
			break;
	}

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR_OR_NULL(provider))
		goto put_child;

	rockchip_usb3phy_rest_deassert(usb3phy, USB3PHY_MAC_RST);
	rockchip_usb3phy_clk_disable(usb3phy);

	usb3phy->usb_phy.dev = dev;
	usb3phy->usb_phy.init = rockchip_usb3phy_on_init;
	usb3phy->usb_phy.shutdown = rockchip_usb3phy_on_shutdown;
	usb3phy->usb_phy.notify_disconnect = rockchip_usb3phy_on_disconnect;
	usb_add_phy(&usb3phy->usb_phy, USB_PHY_TYPE_USB3);
	ATOMIC_INIT_NOTIFIER_HEAD(&usb3phy->usb_phy.notifier);

	rockchip_usb3phy_debugfs_init(usb3phy);

	dev_info(dev, "Rockchip usb3phy initialized successfully\n");
	return 0;

put_child:
	of_node_put(child_np);
	return ret;
}

static int rk3328_usb3phy_pipe_power(struct rockchip_usb3phy *usb3phy,
				   struct rockchip_usb3phy_port *usb3phy_port,
				   bool on)
{
	unsigned int reg = 0;

	if (on) {
		regmap_read(usb3phy_port->regs, 0x1a8, &reg);
		/* reg = readl(usb3phy_port->base + 0x1a8); */
		reg &= ~BIT(4); /* ldo power up */
		regmap_write(usb3phy_port->regs, 0x1a8, reg);
		/* writel(reg, usb3phy_port->base + 0x1a8); */

		regmap_read(usb3phy_port->regs, 0x44, &reg);
		/* reg = readl(usb3phy_port->base + 0x044); */
		reg &= ~BIT(4); /* bg power on */
		regmap_write(usb3phy_port->regs, 0x44, reg);
		/* writel(reg, usb3phy_port->base + 0x044); */

		regmap_read(usb3phy_port->regs, 0x150, &reg);
		/* reg = readl(usb3phy_port->base + 0x150); */
		reg |= BIT(6); /* tx bias enable */
		regmap_write(usb3phy_port->regs, 0x150, reg);
		/* writel(reg, usb3phy_port->base + 0x150); */

		regmap_read(usb3phy_port->regs, 0x80, &reg);
		/* reg = readl(usb3phy_port->base + 0x080); */
		reg &= ~BIT(2); /* tx cm power up */
		regmap_write(usb3phy_port->regs, 0x80, reg);
		/* writel(reg, usb3phy_port->base + 0x080); */

		regmap_read(usb3phy_port->regs, 0xc0, &reg);
		/* reg = readl(usb3phy_port->base + 0x0c0); */
		/* tx obs enable and rx cm enable */
		reg |= (BIT(3) | BIT(4));
		regmap_write(usb3phy_port->regs, 0xc0, reg);
		/* writel(reg, usb3phy_port->base + 0x0c0); */

		udelay(1);
	} else {
		regmap_read(usb3phy_port->regs, 0x1a8, &reg);
		/* reg = readl(usb3phy_port->base + 0x1a8); */
		reg |= BIT(4); /* ldo power down */
		regmap_write(usb3phy_port->regs, 0x1a8, reg);
		/* writel(reg, usb3phy_port->base + 0x1a8); */

		regmap_read(usb3phy_port->regs, 0x44, &reg);
		/* reg = readl(usb3phy_port->base + 0x044); */
		reg |= BIT(4); /* bg power down */
		regmap_write(usb3phy_port->regs, 0x44, reg);
		/* writel(reg, usb3phy_port->base + 0x044); */

		regmap_read(usb3phy_port->regs, 0x150, &reg);
		/* reg = readl(usb3phy_port->base + 0x150); */
		reg &= ~BIT(6); /* tx bias disable */
		regmap_write(usb3phy_port->regs, 0x150, reg);
		/* writel(reg, usb3phy_port->base + 0x150); */

		regmap_read(usb3phy_port->regs, 0x80, &reg);
		/* reg = readl(usb3phy_port->base + 0x080); */
		reg |= BIT(2); /* tx cm power down */
		regmap_write(usb3phy_port->regs, 0x80, reg);
		/* writel(reg, usb3phy_port->base + 0x080); */

		regmap_read(usb3phy_port->regs, 0xc0, &reg);
		/* reg = readl(usb3phy_port->base + 0x0c0); */
		/* tx obs disable and rx cm disable */
		reg &= ~(BIT(3) | BIT(4));
		regmap_write(usb3phy_port->regs, 0xc0, reg);
		/* writel(reg, usb3phy_port->base + 0x0c0); */
	}

	return 0;
}

static int rk3328_usb3phy_tuning(struct rockchip_usb3phy *usb3phy,
			       struct rockchip_usb3phy_port *usb3phy_port,
			       struct device_node *child_np)
{
	if (usb3phy_port->type == USB3PHY_TYPE_UTMI) {
		/*
		 * For rk3328 SoC, pre-emphasis and pre-emphasis strength must
		 * be written as one fixed value as below.
		 *
		 * Dissimilarly, the odt 45ohm value should be flexibly tuninged
		 * for the different boards to adjust HS eye height, so its
		 * value can be assigned in DT in code design.
		 */

		/* {bits[2:0]=111}: always enable pre-emphasis */
		usb3phy->apbcfg.u2_pre_emp = 0x0f;

		/* {bits[5:3]=000}: pre-emphasis strength as the weakest */
		usb3phy->apbcfg.u2_pre_emp_sth = 0x41;

		/* {bits[4:0]=10101}: odt 45ohm tuning */
		usb3phy->apbcfg.u2_odt_tuning = 0xb5;
		/* optional override of the odt 45ohm tuning */
		of_property_read_usb32(child_np, "rockchip,odt-val-tuning",
				     &usb3phy->apbcfg.u2_odt_tuning);

		regmap_write(usb3phy_port->regs, 0x30, usb3phy->apbcfg.u2_pre_emp);
		regmap_write(usb3phy_port->regs, 0x40, usb3phy->apbcfg.u2_pre_emp_sth);
		regmap_write(usb3phy_port->regs, 0x11c, usb3phy->apbcfg.u2_odt_tuning);
		/* writel(usb3phy->apbcfg.u2_pre_emp, usb3phy_port->base + 0x030); */
		/* writel(usb3phy->apbcfg.u2_pre_emp_sth, usb3phy_port->base + 0x040); */
		/* writel(usb3phy->apbcfg.u2_odt_tuning, usb3phy_port->base + 0x11c); */
	} else if (usb3phy_port->type == USB3PHY_TYPE_PIPE) {
		if (usb3phy_port->refclk_25m_quirk) {
			dev_dbg(usb3phy->dev, "switch to 25m refclk\n");
			/* ref clk switch to 25M */
			regmap_write(usb3phy_port->regs, 0x11c, 0x64);
			regmap_write(usb3phy_port->regs, 0x028, 0x64);
			regmap_write(usb3phy_port->regs, 0x020, 0x01);
			regmap_write(usb3phy_port->regs, 0x030, 0x21);
			regmap_write(usb3phy_port->regs, 0x108, 0x06);
			regmap_write(usb3phy_port->regs, 0x118, 0x00);
			/* writel(0x64, usb3phy_port->base + 0x11c); */
			/* writel(0x64, usb3phy_port->base + 0x028); */
			/* writel(0x01, usb3phy_port->base + 0x020); */
			/* writel(0x21, usb3phy_port->base + 0x030); */
			/* writel(0x06, usb3phy_port->base + 0x108); */
			/* writel(0x00, usb3phy_port->base + 0x118); */
		} else {
			/* configure for 24M ref clk */
			regmap_write(usb3phy_port->regs, 0x10c, 0x80);
			regmap_write(usb3phy_port->regs, 0x118, 0x01);
			regmap_write(usb3phy_port->regs, 0x11c, 0x38);
			regmap_write(usb3phy_port->regs, 0x020, 0x83);
			regmap_write(usb3phy_port->regs, 0x108, 0x02);
			/* writel(0x80, usb3phy_port->base + 0x10c); */
			/* writel(0x01, usb3phy_port->base + 0x118); */
			/* writel(0x38, usb3phy_port->base + 0x11c); */
			/* writel(0x83, usb3phy_port->base + 0x020); */
			/* writel(0x02, usb3phy_port->base + 0x108); */
		}

		/* Enable SSC */
		udelay(3);
		regmap_write(usb3phy_port->regs, 0x000, 0x08);
		regmap_write(usb3phy_port->regs, 0x000, 0x0c);
		/* writel(0x08, usb3phy_port->base + 0x000); */
		/* writel(0x0c, usb3phy_port->base + 0x120); */

		/* Tuning Rx for compliance RJTL test */
		regmap_write(usb3phy_port->regs, 0x150, 0x70);
		regmap_write(usb3phy_port->regs, 0x0c8, 0x12);
		regmap_write(usb3phy_port->regs, 0x148, 0x05);
		regmap_write(usb3phy_port->regs, 0x068, 0x08);
		regmap_write(usb3phy_port->regs, 0x1c4, 0xf0);
		regmap_write(usb3phy_port->regs, 0x070, 0xff);
		regmap_write(usb3phy_port->regs, 0x06c, 0x0f);
		regmap_write(usb3phy_port->regs, 0x060, 0xe0);
		/* writel(0x70, usb3phy_port->base + 0x150); */
		/* writel(0x12, usb3phy_port->base + 0x0c8); */
		/* writel(0x05, usb3phy_port->base + 0x148); */
		/* writel(0x08, usb3phy_port->base + 0x068); */
		/* writel(0xf0, usb3phy_port->base + 0x1c4); */
		/* writel(0xff, usb3phy_port->base + 0x070); */
		/* writel(0x0f, usb3phy_port->base + 0x06c); */
		/* writel(0xe0, usb3phy_port->base + 0x060); */

		/*
		 * Tuning Tx to increase the bias current
		 * used in TX driver and RX EQ, it can
		 * also increase the voltage of LFPS.
		 */
		regmap_write(usb3phy_port->regs, 0x180, 0x08);
		/* writel(0x08, usb3phy_port->base + 0x180); */
	} else {
		dev_err(usb3phy->dev, "invalid usb3phy port type\n");
		return -EINVAL;
	}

	return 0;
}

static const struct rockchip_usb3phy_cfg rk3328_usb3phy_cfgs[] = {
	{
		.reg		= 0xff470000,
		.grfcfg		= {
			.um_suspend	= { 0x0004, 15, 0, 0x1452, 0x15d1 },
			.um_ls		= { 0x0030, 5, 4, 0, 1 },
			.um_hstdct	= { 0x0030, 7, 7, 0, 1 },
			.ls_det_en	= { 0x0040, 0, 0, 0, 1 },
			.ls_det_st	= { 0x0044, 0, 0, 0, 1 },
			.pp_pwr_st	= { 0x0034, 14, 13, 0, 0},
			.pp_pwr_en	= { {0x0020, 14, 0, 0x0014, 0x0005},
					    {0x0020, 14, 0, 0x0014, 0x000d},
					    {0x0020, 14, 0, 0x0014, 0x0015},
					    {0x0020, 14, 0, 0x0014, 0x001d} },
			.usb3_disable	= { 0x04c4, 15, 0, 0x1100, 0x101},
		},
		.phy_pipe_power	= rk3328_usb3phy_pipe_power,
		.phy_tuning	= rk3328_usb3phy_tuning,
	},
	{ /* sentinel */ }
};

static const struct of_device_id rockchip_usb3phy_dt_match[] = {
	{ .compatible = "rockchip,rk3328-usb3phy", .data = &rk3328_usb3phy_cfgs },
	{}
};
MODULE_DEVICE_TABLE(of, rockchip_usb3phy_dt_match);

static struct platform_driver rockchip_usb3phy_driver = {
	.probe		= rockchip_usb3phy_probe,
	.driver		= {
		.name	= "rockchip-usb3phy",
		.of_match_table = rockchip_usb3phy_dt_match,
	},
};
module_platform_driver(rockchip_usb3phy_driver);

MODULE_AUTHOR("Frank Wang <frank.wang@rock-chips.com>");
MODULE_AUTHOR("William Wu <william.wu@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip USB 3.0 PHY driver");
MODULE_LICENSE("GPL v2");
