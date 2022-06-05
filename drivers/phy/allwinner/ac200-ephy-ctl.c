// SPDX-License-Identifier: GPL-2.0+
/**
 * syscon driver to control and configure AC200 Ethernet PHY
 * Copyright (c) 2020 Jernej Skrabec <jernej.skrabec@siol.net>
 *
 * TODO's and questions:
 * =========================
 * - This driver is something like a syscon driver, as it controls various
 *   bits and registers that affect other devices (the actual PHY). It's
 *   unclear where it should live, though:
 *   - it could be integrated into the MFD driver, but this looks messy
 *   - it could live at the current location (drivers/phy/allwinner), but that
 *     sounds wrong
 *   - it could be a separate file, but in drivers/mfd
 *   - something else
 *
 * - TODO: AC200 I2C register 0x14 contains a clock gate, that apparently
 *   supplies the PHY logic (not the PHY control logic). This should be
 *   modelled as a CCF gate clock, and referenced by the PHY. Only that the
 *   generic PHY driver needs to learn first how to deal with a clock.
 *   But we might need that anyway, for board with GBit PHYs without crystals.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/ac200.h>
#include <linux/nvmem-consumer.h>
#include <linux/reset-controller.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <dt-bindings/gpio/gpio.h>

/* macros for system ephy control 0 register */
#define AC200_EPHY_RESET_INVALID	BIT(0)
#define AC200_EPHY_SYSCLK_GATING	BIT(1)

/* macros for system ephy control 1 register */
#define AC200_EPHY_E_EPHY_MII_IO_EN	BIT(0)
#define AC200_EPHY_E_LNK_LED_IO_EN	BIT(1)
#define AC200_EPHY_E_SPD_LED_IO_EN	BIT(2)
#define AC200_EPHY_E_DPX_LED_IO_EN	BIT(3)

/* macros for ephy control register */
#define AC200_EPHY_SHUTDOWN		BIT(0)
#define AC200_EPHY_LED_POL		BIT(1)
#define AC200_EPHY_CLK_SEL		BIT(2)
#define AC200_EPHY_ADDR(x)		(((x) & 0x1F) << 4)
#define AC200_EPHY_XMII_SEL		BIT(11)
#define AC200_EPHY_CALIB(x)		(((x) & 0xF) << 12)

struct ac200_ephy_ctl_dev {
	struct reset_controller_dev	rcdev;
	struct clk_hw			gate_clk;
	struct regmap			*regmap;
};

static struct ac200_ephy_ctl_dev *to_phy_dev(struct reset_controller_dev *rcdev)
{
	return container_of(rcdev, struct ac200_ephy_ctl_dev, rcdev);
}

static int ephy_ctl_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct ac200_ephy_ctl_dev *ac200 = to_phy_dev(rcdev);

	regmap_clear_bits(ac200->regmap, AC200_SYS_EPHY_CTL0,
			  AC200_EPHY_RESET_INVALID);
	/* This is going via I2C, so there is plenty of built-in delay. */
	return regmap_set_bits(ac200->regmap, AC200_SYS_EPHY_CTL0,
			       AC200_EPHY_RESET_INVALID);
}

static int ephy_ctl_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct ac200_ephy_ctl_dev *ac200 = to_phy_dev(rcdev);

	return regmap_clear_bits(ac200->regmap, AC200_SYS_EPHY_CTL0,
				 AC200_EPHY_RESET_INVALID);
}

static int ephy_ctl_deassert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct ac200_ephy_ctl_dev *ac200 = to_phy_dev(rcdev);

	/*
	 * TODO: hack for now(?): also enable the clock gate.
	 * We can model the clock gate (see below for the beginnings),
	 * but the existing Ethernet PHY binding does not support a "clocks"
	 * property, so the PHY driver would never enable that (atm).
	 * This can be added, but the semantic of that clock is not very
	 * clear (there are already *MII clocks and the MDIO clock).
	 * Maybe we keep this clock gate always on, and only toggle it
	 * on probe()/remove()?
	 */
	return regmap_set_bits(ac200->regmap, AC200_SYS_EPHY_CTL0,
			       AC200_EPHY_RESET_INVALID |
			       AC200_EPHY_SYSCLK_GATING);
}

static int ephy_ctl_status(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct ac200_ephy_ctl_dev *ac200 = to_phy_dev(rcdev);

	return regmap_test_bits(ac200->regmap, AC200_SYS_EPHY_CTL0,
				AC200_EPHY_RESET_INVALID);
}

static int ephy_ctl_reset_of_xlate(struct reset_controller_dev *rcdev,
				   const struct of_phandle_args *reset_spec)
{
	if (WARN_ON(reset_spec->args_count != 0))
		return -EINVAL;

	return 0;
}

const struct reset_control_ops ephy_ctl_reset_ops = {
	.assert		= ephy_ctl_assert,
	.deassert	= ephy_ctl_deassert,
	.reset		= ephy_ctl_reset,
	.status		= ephy_ctl_status,
};

static struct ac200_ephy_ctl_dev *to_gate_clk(struct clk_hw *hw)
{
	return container_of(hw, struct ac200_ephy_ctl_dev, gate_clk);
}

static int ephy_ctl_gate_enable(struct clk_hw *hw)
{
	struct ac200_ephy_ctl_dev *ac200 = to_gate_clk(hw);

	return regmap_set_bits(ac200->regmap, AC200_SYS_EPHY_CTL0,
			       AC200_EPHY_SYSCLK_GATING);
}

static void ephy_ctl_gate_disable(struct clk_hw *hw)
{
	struct ac200_ephy_ctl_dev *ac200 = to_gate_clk(hw);

	regmap_clear_bits(ac200->regmap, AC200_SYS_EPHY_CTL0,
			  AC200_EPHY_SYSCLK_GATING);
}

static int ephy_ctl_gate_is_enabled(struct clk_hw *hw)
{
	struct ac200_ephy_ctl_dev *ac200 = to_gate_clk(hw);

	return regmap_test_bits(ac200->regmap, AC200_SYS_EPHY_CTL0,
				AC200_EPHY_SYSCLK_GATING);
}

const struct clk_ops ephy_ctl_clk_gate_ops = {
	.enable		= ephy_ctl_gate_enable,
	.disable	= ephy_ctl_gate_disable,
	.is_enabled	= ephy_ctl_gate_is_enabled,
};

static struct clk_hw *ephy_gate_get(struct of_phandle_args *clkspec, void *data)
{
	return data;
}

static int ac200_ephy_ctl_probe(struct platform_device *pdev)
{
	struct ac200_dev *ac200 = dev_get_drvdata(pdev->dev.parent);
	phy_interface_t phy_if;
	struct device *dev = &pdev->dev;
	struct ac200_ephy_ctl_dev *priv;
	struct nvmem_cell *calcell;
	u16 *caldata, ephy_ctl;
	u32 value;
	size_t callen;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	calcell = devm_nvmem_cell_get(dev, "ephy_calib");
	if (IS_ERR(calcell)) {
		dev_err(dev, "Unable to find calibration data!\n");
		return PTR_ERR(calcell);
	}

	caldata = nvmem_cell_read(calcell, &callen);
	if (IS_ERR(caldata)) {
		dev_err(dev, "Unable to read calibration data!\n");
		return PTR_ERR(caldata);
	}

	if (callen != 2) {
		dev_err(dev, "Calibration data has wrong length: 2 != %zu\n",
			callen);
		kfree(caldata);
		return -EINVAL;
	}

	ephy_ctl = AC200_EPHY_CALIB(*caldata + 3);
	kfree(caldata);

	ret = of_get_phy_mode(dev->of_node, &phy_if);
	if (ret) {
		dev_err(dev, "Unable to read PHY connection mode\n");
		return ret;
	}
	switch (phy_if) {
	case PHY_INTERFACE_MODE_MII:
		break;
	case PHY_INTERFACE_MODE_RMII:
		ephy_ctl |= AC200_EPHY_XMII_SEL;
		break;
	default:
		dev_err(dev, "Illegal PHY connection mode (%d), only RMII or MII supported\n",
			phy_if);
		return -EINVAL;
	}

	ret = of_property_read_u32(dev->of_node, "x-powers,led-polarity",
				   &value);
	if (ret) {
		dev_err(dev, "Unable to read LED polarity setting\n");
		return ret;
	}
	if (value == GPIO_ACTIVE_LOW)
		ephy_ctl |= AC200_EPHY_LED_POL;

	ret = of_property_read_u32(dev->of_node, "phy-address", &value);
	if (ret) {
		dev_err(dev, "Unable to read PHY address value\n");
		return ret;
	}
	ephy_ctl |= AC200_EPHY_ADDR(value);

	priv->regmap = ac200->regmap;
	platform_set_drvdata(pdev, priv);

	/* Assert reset and gate clock, to disable PHY for now */
	ret = regmap_write(ac200->regmap, AC200_SYS_EPHY_CTL0,
			   AC200_EPHY_RESET_INVALID |
			   AC200_EPHY_SYSCLK_GATING);
	if (ret)
		return ret;

	ret = regmap_write(ac200->regmap, AC200_SYS_EPHY_CTL1,
			   AC200_EPHY_E_EPHY_MII_IO_EN |
			   AC200_EPHY_E_LNK_LED_IO_EN |
			   AC200_EPHY_E_SPD_LED_IO_EN |
			   AC200_EPHY_E_DPX_LED_IO_EN);
	if (ret)
		return ret;

	ret = regmap_write(ac200->regmap, AC200_EPHY_CTL,
			   ephy_ctl | AC200_EPHY_CLK_SEL);
	if (ret)
		return ret;

	/* TODO: check reset controller setup, THIS_MODULE or dev? */
	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.nr_resets = 1;
	priv->rcdev.ops = &ephy_ctl_reset_ops;
	priv->rcdev.of_node = dev->of_node;
	priv->rcdev.of_reset_n_cells = 0;
	priv->rcdev.of_xlate = ephy_ctl_reset_of_xlate;
	ret = devm_reset_controller_register(dev, &priv->rcdev);
	if (ret) {
		dev_err(dev, "Unable to register reset controller\n");
		return ret;
	}


#if 0
	/* TODO: clock gate */
	ret = devm_clk_hw_register(dev, &priv->gate_clk);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Unable to register clock gate: %d\n",
				ret);
		return ret;
	}
	ret = of_clk_add_hw_provider(dev->of_node, ephy_gate_get,
				     &priv->gate_clk);
	if (ret) {
		dev_err(dev, "Unable to register clock gate: %d\n", ret);
		return ret;
	}
#endif

	return 0;
}

static int ac200_ephy_ctl_remove(struct platform_device *pdev)
{
	struct ac200_ephy_ctl_dev *priv = platform_get_drvdata(pdev);

	reset_controller_unregister(&priv->rcdev);

	regmap_write(priv->regmap, AC200_EPHY_CTL, AC200_EPHY_SHUTDOWN);
	regmap_write(priv->regmap, AC200_SYS_EPHY_CTL1, 0);
	regmap_write(priv->regmap, AC200_SYS_EPHY_CTL0, 0);

	return 0;
}

static const struct of_device_id ac200_ephy_ctl_match[] = {
	{ .compatible = "x-powers,ac200-ephy-ctl" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ac200_ephy_match);

static struct platform_driver ac200_ephy_ctl_driver = {
	.probe		= ac200_ephy_ctl_probe,
	.remove		= ac200_ephy_ctl_remove,
	.driver		= {
		.name		= "ac200-ephy-ctl",
		.of_match_table	= ac200_ephy_ctl_match,
	},
};
module_platform_driver(ac200_ephy_ctl_driver);

MODULE_AUTHOR("Jernej Skrabec <jernej.skrabec@siol.net>");
MODULE_DESCRIPTION("AC200 Ethernet PHY control driver");
MODULE_LICENSE("GPL");
