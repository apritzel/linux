// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 Arm Ltd.
 *
 * Covers the two CPU PLLs: CPU_L for the "little" cluster, covering
 * cores 0-3, CPU_B for the "big" cluster, covering cores 4-7.
 * There is also the DSU PLL.
 */

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "ccu_common.h"
#include "ccu_reset.h"

#include "ccu_div.h"
#include "ccu_gate.h"
#include "ccu_mp.h"
#include "ccu_nm.h"

//#include "ccu-sun50i-h6-r.h"

/*
 * The PLLs are input * N / P / (M0 * M1). Model them as NKMP with no K,
 * and with M0 fixed to 1.
 * The actual enable bit is bit 31, which we set once in probe, along with
 * some other control bits, as the manual recommends to not touch them
 * during runtime.
 */
#define SUN55I_A523_PLL_CPU_L_REG		0x04
static struct ccu_nkmp pll_cpu_l_clk = {
	.enable		= BIT(27),
	.lock		= BIT(28),
	.n		= _SUNXI_CCU_MULT_MIN(8, 8, 20),
	.m		= _SUNXI_CCU_DIV(0, 3),		/* M1 */
	.p		= _SUNXI_CCU_DIV(16, 4),	/* prediv */
	.common		= {
		.reg		= 0x04,
		.hw.init	= CLK_HW_INIT_PARENTS_DATA("pll-cpu-l", osc24M,
							   &ccu_nkmp_ops,
							   CLK_SET_RATE_UNGATE),
	},
};
static const struct clk_hw *pll_cpu_l_hws[] = {
	&pll_cpu_l_clk.common.hw
};

#define SUN55I_A523_PLL_CPU_DSU_REG		0x08
static struct ccu_nkmp pll_cpu_dsu_clk = {
	.enable		= BIT(27),
	.lock		= BIT(28),
	.n		= _SUNXI_CCU_MULT(8, 8),
	.m		= _SUNXI_CCU_DIV(0, 3),		/* M1 */
	.p		= _SUNXI_CCU_DIV(16, 4),	/* prediv */
	.common		= {
		.reg		= 0x08,
		.hw.init	= CLK_HW_INIT_PARENTS_DATA("pll-cpu-dsu", osc24M,
							   &ccu_nkmp_ops,
							   CLK_SET_RATE_UNGATE),
	},
};

#define SUN55I_A523_PLL_CPU_B_REG		0x0c
static struct ccu_nkmp pll_cpu_b_clk = {
	.enable		= BIT(27),
	.lock		= BIT(28),
	.n		= _SUNXI_CCU_MULT_MIN(8, 8, 20),
	.m		= _SUNXI_CCU_DIV(0, 3),		/* M1 */
	.p		= _SUNXI_CCU_DIV(16, 4),	/* prediv */
	.common		= {
		.reg		= 0x0c,
		.hw.init	= CLK_HW_INIT_PARENTS_DATA("pll-cpu-b", osc24M,
							   &ccu_nkmp_ops,
							   CLK_SET_RATE_UNGATE),
	},
};
static const struct clk_hw *pll_cpu_b_hws[] = {
	&pll_cpu_b_clk.common.hw
};

/*
 * TODO: perfect chaos in the manual: There is CPU_L_PLL and CPU_B_PLL (ok),
 * but also CPUA_CLK and CPUB_CLK, and references to CPU[013]PLL. Sigh.
 * Also unclear which divider affects which clock exactly.
 * And ... the divider is actually a shift (P). Model with MP and M:=1 ?
 */
static SUNXI_CCU_M_HWS(pll_cpu_l_div_clk, "pll-cpu-l-div",
		       pll_cpu_l_hws, 0x064, 16, 2, 0);
static SUNXI_CCU_M_HWS(pll_cpu_b_div_clk, "pll-cpu-b-div",
		       pll_cpu_b_hws, 0x060, 16, 2, 0);

static const struct clk_parent_data cpu_a_parents[] = {
	{ .fw_name = "hosc" },
	{ .fw_name = "losc" },
	{ .fw_name = "iosc" },
	{ .hw = &pll_cpu_b_div_clk.common.hw },
	{ .fw_name = "pll-periph" },
	{ .hw = &pll_cpu_l_clk.common.hw },
};
static SUNXI_CCU_MUX_DATA(cpu_a_clk, "cpu-a", cpu_a_parents, 0x60,
			  24, 3,		/* mux */
			  0);

static const struct clk_parent_data cpu_b_parents[] = {
	{ .fw_name = "hosc" },
	{ .fw_name = "losc" },
	{ .fw_name = "iosc" },
	{ .hw = &pll_cpu_l_div_clk.common.hw },
	{ .fw_name = "pll-periph" },
	{ .hw = &pll_cpu_b_clk.common.hw },
};
static SUNXI_CCU_MUX_DATA(cpu_b_clk, "cpu-b", cpu_b_parents, 0x64,
			  24, 3,		/* mux */
			  0);

/*
 * TODO: Do we want to expose these gate clocks, really?
 * Turning off the cluster clock might be more of a PSCI firmware job?
 * Also we need to write the magic 0x16aa into the upper half for that.
 */
static SUNXI_CCU_GATE_HWS(cpu_a_gate_clk, "cpu-a-gate", &cpu_a_clk.common.hw,
			  0x68, BIT(0), 0);
static SUNXI_CCU_GATE_HWS(cpu_b_gate_clk, "cpu-b-gate", &cpu_b_clk.common.hw,
			  0x68, BIT(1), 0);
static SUNXI_CCU_GATE_HWS(cpu_dsu_gate_clk, "cpu-dsu-gate", &cpu_dsu_clk.common.hw,
			  0x68, BIT(2), 0);

static struct ccu_common *sun55i_a523_cpu_ccu_clks[] = {
	&pll_cpu_l_clk.common,
	&pll_cpu_b_clk.common,
	&pll_cpu_dsu_clk.common,
	&cpu_a_clk.common,
	&cpu_b_clk.common,
	&cpu_dsu_clk.common,
	/* &cpu_a_gate_clk.common, */
};

static struct clk_hw_onecell_data sun55i_a523_cpu_hw_clks = {
	.hws	= {
		[CLK_CPU_A]		= &cpu_a_clk.common.hw,
		[CLK_CPU_B]		= &cpu_b_clk.common.hw,
		[CLK_CPU_DSU]		= &cpu_dsu_clk.common.hw,
	},
	.num	= CLK_NUMBER,
};

static const struct sunxi_ccu_desc sun55i_a523_cpu_ccu_desc = {
	.ccu_clks	= sun55i_a523_cpu_ccu_clks,
	.num_ccu_clks	= ARRAY_SIZE(sun55i_a523_cpu_ccu_clks),

	.hw_clks	= &sun55i_a523_cpu_hw_clks,
};

static const u32 pll_regs[] = {
	SUN55I_A523_PLL_CPU_L_REG,
	SUN55I_A523_PLL_CPU_DSU_REG,
	SUN55I_A523_PLL_CPU_B_REG,
};

static int sun55i_a523_cpu_ccu_probe(struct platform_device *pdev)
{
	const struct sunxi_ccu_desc *desc;
	void __iomem *reg;

	desc = of_device_get_match_data(&pdev->dev);
	if (!desc)
		return -EINVAL;

	reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reg))
		return PTR_ERR(reg);

	/* Enable the enable, LDO, and lock bits on all PLLs, clear M0 */
	for (i = 0; i < ARRAY_SIZE(pll_regs); i++) {
		val = readl(reg + pll_regs[i]);
		val |= BIT(31) | BIT(30) | BIT(29);
		val &= ~GENMASK(21, 20);
		writel(val, reg + pll_regs[i]);
	}

	return devm_sunxi_ccu_probe(&pdev->dev, reg, desc);
}

static const struct of_device_id sun55i_a523_cpu_ccu_ids[] = {
	{
		.compatible = "allwinner,sun55i-a523-cpu-ccu",
		.data = &sun55i_a523_cpu_ccu_desc,
	},
	{ }
};

static struct platform_driver sun55i_a523_cpu_ccu_driver = {
	.probe	= sun55i_a523_cpu_ccu_probe,
	.driver	= {
		.name			= "sun55i-a523-cpu-ccu",
		.suppress_bind_attrs	= true,
		.of_match_table		= sun55i_a523_cpu_ccu_ids,
	},
};
module_platform_driver(sun55i_a523_cpu_ccu_driver);

MODULE_IMPORT_NS(SUNXI_CCU);
MODULE_LICENSE("GPL");
