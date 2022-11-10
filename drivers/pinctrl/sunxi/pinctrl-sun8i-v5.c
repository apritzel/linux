// SPDX-License-Identifier: GPL-2.0
/*
 * Allwinner V5 SoC pinctrl driver.
 *
 * Copyright (C) 2022 Arm Ltd.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>

#include "pinctrl-sunxi.h"

static const unsigned int v5_irq_bank_map[] = { 0, 1, 5, 6, 7 };

static const u8 v5_irq_bank_muxes[SUNXI_PINCTRL_MAX_BANKS] =
/*       PA PB PC PD PE PF PG PH */
        { 6, 6, 0, 0, 0, 6, 6, 6 };

static const u8 v5_nr_bank_pins[SUNXI_PINCTRL_MAX_BANKS] =
/*        PA  PB  PC  PD  PE  PF  PG  PH  PI  PJ */
        { 16, 11, 17, 23, 18,  7, 14, 16, 17, 18 };

static struct sunxi_pinctrl_desc v5_pinctrl_data = {
	.irq_banks = ARRAY_SIZE(v5_irq_bank_map),
	.irq_bank_map = v5_irq_bank_map,
	.irq_read_needs_mux = true,
	.io_bias_cfg_variant = BIAS_VOLTAGE_PIO_POW_MODE_SEL,
};

static int v5_pinctrl_probe(struct platform_device *pdev)
{
	return sunxi_pinctrl_dt_table_init(pdev, v5_nr_bank_pins,
					   v5_irq_bank_muxes,
					   &v5_pinctrl_data);
}

static const struct of_device_id v5_pinctrl_match[] = {
	{ .compatible = "allwinner,sun8i-v5-pinctrl", },
	{}
};

static struct platform_driver v5_pinctrl_driver = {
	.probe	= v5_pinctrl_probe,
	.driver	= {
		.name		= "sun8i-v5-pinctrl",
		.of_match_table	= v5_pinctrl_match,
	},
};
builtin_platform_driver(v5_pinctrl_driver);
