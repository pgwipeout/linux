/*
 *
 * Author:
 * 	Peter Geis <pgwipeout@gmail.com>
 * 	Based on drivers/cpufreq/tegra20-cpufreq.c, (C) 2010 Google, Inc.
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

#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>

static struct cpufreq_frequency_table freq_table[] = {
	{ .frequency = 408000 },
	{ .frequency = 612000 },
	{ .frequency = 760000 },
	{ .frequency = 816000 },
	{ .frequency = 910000 },
	{ .frequency = 1000000 },
	{ .frequency = 1100000 },
	{ .frequency = 1200000 },
	{ .frequency = 1300000 },
	{ .frequency = 1400000 },
	{ .frequency = 1500000 },
	{ .frequency = 1600000 },
	{ .frequency = CPUFREQ_TABLE_END },
};

struct tegra30_cpufreq {
	struct device *dev;
	struct cpufreq_driver driver;
	struct clk *cpu_clk;
	struct clk *pll_x_clk;
	struct clk *pll_p_cclkg_clk;
	struct clk *clk_32k;
	struct regulator *vdd_cpu;
	bool pll_x_prepared;
};

static unsigned int tegra_get_intermediate(struct cpufreq_policy *policy,
					   unsigned int index)
{
	struct tegra30_cpufreq *cpufreq = cpufreq_get_driver_data();
	unsigned int ifreq = clk_get_rate(cpufreq->pll_p_cclkg_clk) / 1000;

	/*
	 * Don't switch to intermediate freq if:
	 * - we are already at it, i.e. policy->cur == ifreq
	 * - index corresponds to ifreq
	 */
	if ((freq_table[index].frequency == ifreq) || (policy->cur == ifreq)) 
		return 0;

	return ifreq;
}

static int tegra_target_intermediate(struct cpufreq_policy *policy,
				     unsigned int index)
{
	struct tegra30_cpufreq *cpufreq = cpufreq_get_driver_data();
	int ret;

	/*
	 * Take an extra reference to the main pll so it doesn't turn
	 * off when we move the cpu off of it as enabling it again while we
	 * switch to it from tegra_target() would take additional time.
	 *
	 * When target-freq is equal to intermediate freq we don't need to
	 * switch to an intermediate freq and so this routine isn't called.
	 * Also, we wouldn't be using pll_x anymore and must not take extra
	 * reference to it, as it can be disabled now to save some power.
	 */
	clk_prepare_enable(cpufreq->pll_x_clk);

	ret = clk_set_parent(cpufreq->cpu_clk, cpufreq->pll_p_cclkg_clk);
	if (ret)
		clk_disable_unprepare(cpufreq->pll_x_clk);
	else
		cpufreq->pll_x_prepared = true;

	return ret;
}

static int tegra_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct tegra30_cpufreq *cpufreq = cpufreq_get_driver_data();
	unsigned long rate = freq_table[index].frequency;
	unsigned long reg_volt = freq_table[index].frequency / 4 + 875000;
	unsigned int ifreq = clk_get_rate(cpufreq->pll_p_cclkg_clk) / 1000;
	int ret;

	/*
	 * target freq == pll_p_cclkg, don't need to take extra reference to pll_x_clk
	 * as it isn't used anymore.
	 */
	if (rate == ifreq)
		return clk_set_parent(cpufreq->cpu_clk, cpufreq->pll_p_cclkg_clk);

	ret = clk_set_rate(cpufreq->pll_x_clk, rate * 1000);
	/* Restore to earlier frequency on error, i.e. pll_x */
	if (ret)
		dev_err(cpufreq->dev, "Failed to change pll_x to %lu\n", rate);

	/* 
	 * Set the regulator voltage based off the current frequency before switching
	 * back to it.
	 */

	ret = regulator_set_voltage(cpufreq->vdd_cpu, reg_volt, reg_volt);
	if (ret)
		pr_err("Failed to change regulator to %lu\n", reg_volt);

	ret = clk_set_parent(cpufreq->cpu_clk, cpufreq->pll_x_clk);
	/* This shouldn't fail while changing or restoring */
	WARN_ON(ret);

	/*
	 * Drop count to pll_x clock only if we switched to intermediate freq
	 * earlier while transitioning to a target frequency.
	 */
	if (cpufreq->pll_x_prepared) {
		clk_disable_unprepare(cpufreq->pll_x_clk);
		cpufreq->pll_x_prepared = false;
	}

}

static int tegra_cpu_init(struct cpufreq_policy *policy)
{
	struct tegra30_cpufreq *cpufreq = cpufreq_get_driver_data();
	unsigned int trans_time = clk_get_rate(cpufreq->clk_32k) * 2;
	int ret;

	clk_prepare_enable(cpufreq->cpu_clk);

	/* 
	 * The transition time should be two clock periods of the main clock.
	 * The main clock should be 32768, so we could hard code this to 65536.
	 * But just in case, we grab the clock anyways. 
	 */
	ret = cpufreq_generic_init(policy, freq_table, trans_time);
	if (ret) {
		clk_disable_unprepare(cpufreq->cpu_clk);
		return ret;
	}

	policy->clk = cpufreq->cpu_clk;
	policy->suspend_freq = freq_table[0].frequency;
	return 0;
}

static int tegra_cpu_exit(struct cpufreq_policy *policy)
{
	struct tegra30_cpufreq *cpufreq = cpufreq_get_driver_data();

	clk_disable_unprepare(cpufreq->cpu_clk);
	return 0;
}

static struct cpufreq_driver tegra_cpufreq_driver = {
	.flags			= CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify			= cpufreq_generic_frequency_table_verify,
	.get_intermediate	= tegra_get_intermediate,
	.target_intermediate	= tegra_target_intermediate,
	.target_index		= tegra_target,
	.get			= cpufreq_generic_get,
	.init			= tegra_cpu_init,
	.exit			= tegra_cpu_exit,
	.name			= "tegra",
	.attr			= cpufreq_generic_attr,
	.suspend		= cpufreq_generic_suspend,
};

static int tegra30_cpufreq_probe(struct platform_device *pdev)
{
	struct tegra30_cpufreq *cpufreq;
	int err;

	cpufreq = devm_kzalloc(&pdev->dev, sizeof(*cpufreq), GFP_KERNEL);
	if (!cpufreq)
		return -ENODEV;

	cpufreq->cpu_clk = clk_get_sys(NULL, "cclk_g");
	if (IS_ERR(cpufreq->cpu_clk)) 
		return PTR_ERR(cpufreq->cpu_clk);

	cpufreq->pll_x_clk = clk_get_sys(NULL, "pll_x");
	if (IS_ERR(cpufreq->pll_x_clk)) {
		err = PTR_ERR(cpufreq->pll_x_clk);
		goto put_cpu;
	}

	cpufreq->pll_p_cclkg_clk = clk_get_sys(NULL, "pll_p_cclkg");
	if (IS_ERR(cpufreq->pll_p_cclkg_clk)) {
		err = PTR_ERR(cpufreq->pll_p_cclkg_clk);
		goto put_pll_x;
	}

	cpufreq->vdd_cpu = regulator_get(NULL, "vdd_cpu,vdd_sys");
	if (IS_ERR(cpufreq->vdd_cpu)) {
		pr_warn("Failed to get cpu regulator.");
		return PTR_ERR(cpufreq->vdd_cpu);
	}

	cpufreq->vdd_cpu = regulator_get(NULL, "vdd_cpu,vdd_sys");
	if (IS_ERR(cpufreq->vdd_cpu)) {
		pr_warn("Failed to get cpu regulator.");
		return PTR_ERR(cpufreq->vdd_cpu);
	}

	cpufreq->dev = &pdev->dev;
	cpufreq->driver.get = cpufreq_generic_get;
	cpufreq->driver.attr = cpufreq_generic_attr;
	cpufreq->driver.init = tegra_cpu_init;
	cpufreq->driver.exit = tegra_cpu_exit;
	cpufreq->driver.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK;
	cpufreq->driver.verify = cpufreq_generic_frequency_table_verify;
	cpufreq->driver.suspend = cpufreq_generic_suspend;
	cpufreq->driver.driver_data = cpufreq;
	cpufreq->driver.target_index = tegra_target;
	cpufreq->driver.get_intermediate = tegra_get_intermediate;
	cpufreq->driver.target_intermediate = tegra_target_intermediate;
	snprintf(cpufreq->driver.name, CPUFREQ_NAME_LEN, "tegra");

	err = cpufreq_register_driver(&cpufreq->driver);
	if (err)
		goto put_pll_p_cclkg;

	platform_set_drvdata(pdev, cpufreq);

	return 0;

put_pll_p_cclkg:
	clk_put(cpufreq->pll_p_cclkg_clk);
put_pll_x:
	clk_put(cpufreq->pll_x_clk);
put_cpu:
	clk_put(cpufreq->cpu_clk);

	return cpufreq_register_driver(&tegra_cpufreq_driver);
	return err;
}

static int tegra30_cpufreq_remove(struct platform_device *pdev)
{
	struct tegra30_cpufreq *cpufreq = platform_get_drvdata(pdev);

	cpufreq_unregister_driver(&cpufreq->driver);

	regulator_put(cpufreq->vdd_cpu);
	clk_put(cpufreq->pll_p_cclkg_clk);
	clk_put(cpufreq->pll_x_clk);
	clk_put(cpufreq->cpu_clk);

	return 0;
}

static struct platform_driver tegra30_cpufreq_driver = {
	.probe		= tegra30_cpufreq_probe,
	.remove		= tegra30_cpufreq_remove,
	.driver		= {
		.name	= "tegra30-cpufreq",
	},
};
module_platform_driver(tegra30_cpufreq_driver);

MODULE_AUTHOR("Peter Geis <pgwipeout@gmail.com>");
MODULE_ALIAS("platform:tegra30-cpufreq");
MODULE_DESCRIPTION("NVIDIA Tegra30 cpufreq driver");
MODULE_LICENSE("GPL");

