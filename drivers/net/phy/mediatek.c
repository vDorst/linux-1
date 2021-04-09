// SPDX-License-Identifier: GPL-2.0+
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/version.h>

#define MTK_EXT_PAGE_ACCESS		0x1f
#define MTK_PHY_PAGE_STANDARD		0x0000
#define MTK_PHY_PAGE_EXTENDED		0x0001
#define MTK_PHY_PAGE_EXTENDED_2		0x0002
#define MTK_PHY_PAGE_EXTENDED_3		0x0003
#define MTK_PHY_PAGE_EXTENDED_2A30	0x2a30
#define MTK_PHY_PAGE_EXTENDED_52B5	0x52b5

#define MT7531_INTR_STATUS			0x1a
#define MT7531_INTR_ENABLE			0x19

#define  MT7531_INTR_ENABLE_AUTONEG_ERR 	BIT(11)
#define  MT7531_INTR_ENABLE_SPEED_CHANGED	BIT(14)
#define  MT7531_INTR_ENABLE_DUPLEX_CHANGED	BIT(12)
#define  MT7531_INTR_ENABLE_LINK_FAIL		BIT(3)
#define  MT7531_INTR_ENABLE_LINK_SUCCESS	BIT(10)


static int mtk_phy_read_page(struct phy_device *phydev)
{
	return __phy_read(phydev, MTK_EXT_PAGE_ACCESS);
}

static int mtk_phy_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, MTK_EXT_PAGE_ACCESS, page);
}

static void mtk_phy_config_init(struct phy_device *phydev)
{
	/* Disable EEE */
	phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV, 0);

	/* Enable HW auto downshift */
	phy_modify_paged(phydev, MTK_PHY_PAGE_EXTENDED, 0x14, 0, BIT(4));

	/* Increase SlvDPSready time */
	phy_select_page(phydev, MTK_PHY_PAGE_EXTENDED_52B5);
	__phy_write(phydev, 0x10, 0xafae);
	__phy_write(phydev, 0x12, 0x2f);
	__phy_write(phydev, 0x10, 0x8fae);
	phy_restore_page(phydev, MTK_PHY_PAGE_STANDARD, 0);

	/* Adjust 100_mse_threshold */
	phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x123, 0xffff);

	/* Disable mcc */
	phy_write_mmd(phydev, MDIO_MMD_VEND1, 0xa6, 0x300);
}

static int mt7530_phy_config_init(struct phy_device *phydev)
{
	mtk_phy_config_init(phydev);

	/* Increase post_update_timer */
	phy_write_paged(phydev, MTK_PHY_PAGE_EXTENDED_3, 0x11, 0x4b);

	return 0;
}

static int mt7531_phy_config_init(struct phy_device *phydev)
{
	if (phydev->interface != PHY_INTERFACE_MODE_INTERNAL)
		return -EINVAL;

	mtk_phy_config_init(phydev);

	/* PHY link down power saving enable */
	phy_set_bits(phydev, 0x17, BIT(4));
	phy_clear_bits_mmd(phydev, MDIO_MMD_VEND1, 0xc6, 0x300);

	/* Set TX Pair delay selection */
	phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x13, 0x404);
	phy_write_mmd(phydev, MDIO_MMD_VEND1, 0x14, 0x404);


	/* enable interrupt */
	phy_write(phydev, MII_BMCR, BMCR_ANENABLE);

	return 0;
}

static int mt7531_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, MT7531_INTR_STATUS);

	return (err < 0) ? err : 0;
}

static int mt753x_config_intr(struct phy_device *phydev)
{
	int err;
	int value;

	value = phy_read(phydev, MT7531_INTR_ENABLE);

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		/* Clear any pending interrupts */
		err = mt7531_ack_interrupt(phydev);
		if (err)
			return err;

		value = MT7531_INTR_ENABLE_AUTONEG_ERR;
		value |= MT7531_INTR_ENABLE_SPEED_CHANGED;
		value |= MT7531_INTR_ENABLE_DUPLEX_CHANGED;
		value |= MT7531_INTR_ENABLE_LINK_FAIL;
		value |= MT7531_INTR_ENABLE_LINK_SUCCESS;
		value |= BIT(15);

		err = phy_write(phydev, MT7531_INTR_ENABLE, value);
		
		phydev_info(phydev, "Set INT: %x", value);
	} else {
		err = phy_write(phydev, MT7531_INTR_ENABLE, 0);
		if (err)
			return err;

		/* Clear any pending interrupts */
		err = mt7531_ack_interrupt(phydev);

		phydev_info(phydev, "No INT.");
	}

	return err;
}

static irqreturn_t mt753x_handle_interrupt(struct phy_device *phydev)
{
	int irq_status, int_enabled;

	irq_status = phy_read(phydev, MT7531_INTR_STATUS);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}

	/* Read the current enabled interrupts */
	int_enabled = phy_read(phydev, MT7531_INTR_ENABLE);
	if (int_enabled < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}

	/* See if this was one of our enabled interrupts */
	if (!(irq_status & int_enabled))
		return IRQ_NONE;

	phydev_info(phydev, "INT: S%x E%x", irq_status, int_enabled);

	phy_trigger_machine(phydev);

	return IRQ_HANDLED;
}

static struct phy_driver mtk_phy_driver[] = {
	{
		PHY_ID_MATCH_EXACT(0x03a29412),
		.name		= "MediaTek MT7530 PHY",
		.config_init	= mt7530_phy_config_init,
		/* Interrupts are handled by the switch, not the PHY
		 * itself.
		 */
		.config_intr	= mt753x_config_intr,
		.handle_interrupt = mt753x_handle_interrupt,
		.read_page	= mtk_phy_read_page,
		.write_page	= mtk_phy_write_page,
	},
	{
		PHY_ID_MATCH_EXACT(0x03a29441),
		.name		= "MediaTek MT7531 PHY",
		.config_init	= mt7531_phy_config_init,
		/* Interrupts are handled by the switch, not the PHY
		 * itself.
		 */
		.config_intr	= mt753x_config_intr,
		.handle_interrupt = mt753x_handle_interrupt,
		.read_page	= mtk_phy_read_page,
		.write_page	= mtk_phy_write_page,
	},
};

module_phy_driver(mtk_phy_driver);

static struct mdio_device_id __maybe_unused mtk_phy_tbl[] = {
	{ PHY_ID_MATCH_VENDOR(0x03a29400) },
	{ }
};

MODULE_DESCRIPTION("MediaTek switch integrated PHY driver");
MODULE_AUTHOR("DENG, Qingfang <dqfext@gmail.com>");
MODULE_LICENSE("GPL");

MODULE_DEVICE_TABLE(mdio, mtk_phy_tbl);
