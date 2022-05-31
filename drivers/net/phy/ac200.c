// SPDX-License-Identifier: GPL-2.0+
/**
 * Driver for AC200 Ethernet PHY
 *
 * Copyright (c) 2019 Jernej Skrabec <jernej.skrabec@siol.net>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/phy.h>

#define AC200_EPHY_ID			0x00441400
#define AC200_EPHY_ID_MASK		0x0ffffff0

static void disable_intelligent_ieee(struct phy_device *phydev)
{
	unsigned int value;

	phy_write(phydev, 0x1f, 0x0100);	/* switch to page 1 */
	value = phy_read(phydev, 0x17);
	value &= ~BIT(3);			/* disable IEEE */
	phy_write(phydev, 0x17, value);
	phy_write(phydev, 0x1f, 0x0000);	/* switch to page 0 */
}

static void disable_802_3az_ieee(struct phy_device *phydev)
{
	unsigned int value;

	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x3c);
	phy_write(phydev, 0xd, BIT(14) | 0x7);
	value = phy_read(phydev, 0xe);
	value &= ~BIT(1);
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x3c);
	phy_write(phydev, 0xd, BIT(14) | 0x7);
	phy_write(phydev, 0xe, value);

	phy_write(phydev, 0x1f, 0x0200);	/* switch to page 2 */
	phy_write(phydev, 0x18, 0x0000);
}

static int ac200_ephy_config_init(struct phy_device *phydev)
{
	phy_write(phydev, 0x1f, 0x0100);	/* Switch to Page 1 */
	phy_write(phydev, 0x12, 0x4824);	/* Disable APS */

	phy_write(phydev, 0x1f, 0x0200);	/* Switch to Page 2 */
	phy_write(phydev, 0x18, 0x0000);	/* PHYAFE TRX optimization */

	phy_write(phydev, 0x1f, 0x0600);	/* Switch to Page 6 */
	phy_write(phydev, 0x14, 0x708f);	/* PHYAFE TX optimization */
	phy_write(phydev, 0x13, 0xF000);	/* PHYAFE RX optimization */
	phy_write(phydev, 0x15, 0x1530);

	phy_write(phydev, 0x1f, 0x0800);	/* Switch to Page 6 */
	phy_write(phydev, 0x18, 0x00bc);	/* PHYAFE TRX optimization */

	disable_intelligent_ieee(phydev);	/* Disable Intelligent IEEE */
	disable_802_3az_ieee(phydev);		/* Disable 802.3az IEEE */
	phy_write(phydev, 0x1f, 0x0000);	/* Switch to Page 0 */

	/* FIXME: This is probably H6 specific */
	phy_set_bits(phydev, 0x13, BIT(12));

	return 0;
}

static struct phy_driver ac200_ephy_driver[] = {
{
	.phy_id		= AC200_EPHY_ID,
	.phy_id_mask	= AC200_EPHY_ID_MASK,
	.name		= "Allwinner AC200 EPHY",
	.config_init	= ac200_ephy_config_init,
	.soft_reset	= genphy_soft_reset,
	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
} };

module_phy_driver(ac200_ephy_driver);

MODULE_AUTHOR("Jernej Skrabec <jernej.skrabec@siol.net>");
MODULE_DESCRIPTION("AC200 Ethernet PHY driver");
MODULE_LICENSE("GPL");

static const struct mdio_device_id __maybe_unused ac200_ephy_phy_tbl[] = {
	{ AC200_EPHY_ID, AC200_EPHY_ID_MASK },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(mdio, ac200_ephy_phy_tbl);
