/*
 * drivers/net/phy/at803x.c
 *
 * Driver for Atheros 803x PHY
 *
 * Author: Matus Ujhelyi <ujhelyi.m@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/of_net.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of_mdio.h>
#include <linux/property.h>
#include <linux/sfp.h>
#include <linux/phylink.h>

#define AT803X_INTR_ENABLE			0x12
#define AT803X_INTR_ENABLE_AUTONEG_ERR		BIT(15)
#define AT803X_INTR_ENABLE_SPEED_CHANGED	BIT(14)
#define AT803X_INTR_ENABLE_DUPLEX_CHANGED	BIT(13)
#define AT803X_INTR_ENABLE_PAGE_RECEIVED	BIT(12)
#define AT803X_INTR_ENABLE_LINK_FAIL		BIT(11)
#define AT803X_INTR_ENABLE_LINK_SUCCESS		BIT(10)
#define AT803X_INTR_ENABLE_WIRESPEED_DOWNGRADE	BIT(5)
#define AT803X_INTR_ENABLE_POLARITY_CHANGED	BIT(1)
#define AT803X_INTR_ENABLE_WOL			BIT(0)

#define AT803X_INTR_STATUS			0x13

#define AT803X_SMART_SPEED			0x14
#define AT803X_LED_CONTROL			0x18

#define AT803X_DEVICE_ADDR			0x03
#define AT803X_LOC_MAC_ADDR_0_15_OFFSET		0x804C
#define AT803X_LOC_MAC_ADDR_16_31_OFFSET	0x804B
#define AT803X_LOC_MAC_ADDR_32_47_OFFSET	0x804A
#define AT803X_MMD_ACCESS_CONTROL		0x0D
#define AT803X_MMD_ACCESS_CONTROL_DATA		0x0E
#define AT803X_FUNC_DATA			0x4003
#define AT803X_REG_CHIP_CONFIG			0x1f
#define AT803X_BT_BX_REG_SEL			0x8000
#define AT803X_SGMII_ANEG_EN			0x1000

#define AT803X_PCS_SMART_EEE_CTRL3			0x805D
#define AT803X_SMART_EEE_CTRL3_LPI_TX_DELAY_SEL_MASK	0x3
#define AT803X_SMART_EEE_CTRL3_LPI_TX_DELAY_SEL_SHIFT	12
#define AT803X_SMART_EEE_CTRL3_LPI_EN			BIT(8)

#define AT803X_DEBUG_ADDR			0x1D
#define AT803X_DEBUG_DATA			0x1E

#define AT803X_MODE_CFG_MASK			0x0F
#define AT803X_MODE_CFG_SGMII			0x01
#define AT803X_MODE_CFG_BX1000_RGMII_50		0x02
#define AT803X_MODE_CFG_BX1000_RGMII_75		0x03
#define AT803X_MODE_FIBER			0x01
#define AT803X_MODE_COPPER			0x00


#define AT803X_PSSR			0x11	/*PHY-Specific Status Register*/
#define AT803X_PSSR_MR_AN_COMPLETE	0x0200

#define AT803X_DEBUG_REG_0			0x00
#define AT803X_DEBUG_RX_CLK_DLY_EN		BIT(15)

#define AT803X_DEBUG_REG_5			0x05
#define AT803X_DEBUG_TX_CLK_DLY_EN		BIT(8)

#define ATH8030_PHY_ID 0x004dd076
#define ATH8031_PHY_ID 0x004dd074
#define ATH8035_PHY_ID 0x004dd072
#define AT803X_PHY_ID_MASK			0xffffffef

MODULE_DESCRIPTION("Atheros 803x PHY driver");
MODULE_AUTHOR("Matus Ujhelyi");
MODULE_LICENSE("GPL");

struct at803x_phy_hw_stat {
	const char *string;
	u8 reg;
	u8 bits;
};

static struct at803x_phy_hw_stat at803x_hw_stats[] = {
	{ "phy_idle_errors", 10, 8 },
};

struct at803x_priv {
	struct fwnode_handle *sfp_fwnode;
	struct sfp_bus *sfp_bus;
	enum phy_state state;
	bool running;
	bool phy_reset:1;
};

struct at803x_context {
	u16 bmcr;
	u16 advertise;
	u16 control1000;
	u16 int_enable;
	u16 smart_speed;
	u16 led_control;
};

static int at803x_sfp_insert(void *upstream, const struct sfp_eeprom_id *id)
{
	struct phy_device *phydev = upstream;
	struct at803x_priv *priv = phydev->priv;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(support) = { 0, };
	phy_interface_t iface;

	sfp_parse_support(priv->sfp_bus, id, support);
	iface = sfp_select_interface(priv->sfp_bus, id, support);

	pr_warn("at803x_sfp_insert: Mode: %s\n", phy_modes(iface));

	if (iface == PHY_INTERFACE_MODE_SGMII) {
		iface = PHY_INTERFACE_MODE_1000BASEX;
		dev_err(&phydev->mdio.dev, "sgmii not supported but try 1000basex any way!\n");
	}

	if (iface != PHY_INTERFACE_MODE_1000BASEX) {
		dev_err(&phydev->mdio.dev, "incompatible SFP module inserted\n");
		return -EINVAL;
	}

	return 0;
}

static const struct sfp_upstream_ops at803x_sfp_ops = {
	.module_insert = at803x_sfp_insert,
};

static int at803x_phy_get_sset_count(struct phy_device *phydev)
{
	return ARRAY_SIZE(at803x_hw_stats);
}

static void at803x_phy_get_strings(struct phy_device *phydev, u8 *data)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(at803x_hw_stats); i++)
		memcpy(data + i * ETH_GSTRING_LEN,
		       at803x_hw_stats[i].string, ETH_GSTRING_LEN);
}

static void at803x_phy_get_stats(struct phy_device *phydev,
		       struct ethtool_stats *stats, u64 *data)
{
	unsigned int i;
	int val;
	struct at803x_phy_hw_stat stat;
	u64 ret;

	for (i = 0; i < ARRAY_SIZE(at803x_hw_stats); i++) {
		stat = at803x_hw_stats[i];
		val = phy_read(phydev, stat.reg);
		if (val < 0) {
			ret = U64_MAX;
		} else {
			ret = val & ((1 << stat.bits) - 1);
		}
		data[i] = ret;
	}
}

static int at803x_debug_reg_read(struct phy_device *phydev, u16 reg)
{
	int ret;

	ret = phy_write(phydev, AT803X_DEBUG_ADDR, reg);
	if (ret < 0)
		return ret;

	return phy_read(phydev, AT803X_DEBUG_DATA);
}

static int at803x_debug_reg_mask(struct phy_device *phydev, u16 reg,
				 u16 clear, u16 set)
{
	u16 val;
	int ret;

	ret = at803x_debug_reg_read(phydev, reg);
	if (ret < 0)
		return ret;

	val = ret & 0xffff;
	val &= ~clear;
	val |= set;

	return phy_write(phydev, AT803X_DEBUG_DATA, val);
}

static inline int at803x_enable_rx_delay(struct phy_device *phydev)
{
	return at803x_debug_reg_mask(phydev, AT803X_DEBUG_REG_0, 0,
					AT803X_DEBUG_RX_CLK_DLY_EN);
}

static inline int at803x_enable_tx_delay(struct phy_device *phydev)
{
	return at803x_debug_reg_mask(phydev, AT803X_DEBUG_REG_5, 0,
				     AT803X_DEBUG_TX_CLK_DLY_EN);
}

static int at803x_disable_rx_delay(struct phy_device *phydev)
{
	return at803x_debug_reg_mask(phydev, AT803X_DEBUG_REG_0,
				     AT803X_DEBUG_RX_CLK_DLY_EN, 0);
}

static int at803x_disable_tx_delay(struct phy_device *phydev)
{
	return at803x_debug_reg_mask(phydev, AT803X_DEBUG_REG_5,
				     AT803X_DEBUG_TX_CLK_DLY_EN, 0);
}

/* save relevant PHY registers to private copy */
static void at803x_context_save(struct phy_device *phydev,
				struct at803x_context *context)
{
	context->bmcr = phy_read(phydev, MII_BMCR);
	context->advertise = phy_read(phydev, MII_ADVERTISE);
	context->control1000 = phy_read(phydev, MII_CTRL1000);
	context->int_enable = phy_read(phydev, AT803X_INTR_ENABLE);
	context->smart_speed = phy_read(phydev, AT803X_SMART_SPEED);
	context->led_control = phy_read(phydev, AT803X_LED_CONTROL);
}

/* restore relevant PHY registers from private copy */
static void at803x_context_restore(struct phy_device *phydev,
				   const struct at803x_context *context)
{
	phy_write(phydev, MII_BMCR, context->bmcr);
	phy_write(phydev, MII_ADVERTISE, context->advertise);
	phy_write(phydev, MII_CTRL1000, context->control1000);
	phy_write(phydev, AT803X_INTR_ENABLE, context->int_enable);
	phy_write(phydev, AT803X_SMART_SPEED, context->smart_speed);
	phy_write(phydev, AT803X_LED_CONTROL, context->led_control);
}

static int at803x_set_wol(struct phy_device *phydev,
			  struct ethtool_wolinfo *wol)
{
	struct net_device *ndev = phydev->attached_dev;
	const u8 *mac;
	int ret;
	u32 value;
	unsigned int i, offsets[] = {
		AT803X_LOC_MAC_ADDR_32_47_OFFSET,
		AT803X_LOC_MAC_ADDR_16_31_OFFSET,
		AT803X_LOC_MAC_ADDR_0_15_OFFSET,
	};

	if (!ndev)
		return -ENODEV;

	if (wol->wolopts & WAKE_MAGIC) {
		mac = (const u8 *) ndev->dev_addr;

		if (!is_valid_ether_addr(mac))
			return -EINVAL;

		for (i = 0; i < 3; i++) {
			phy_write(phydev, AT803X_MMD_ACCESS_CONTROL,
				  AT803X_DEVICE_ADDR);
			phy_write(phydev, AT803X_MMD_ACCESS_CONTROL_DATA,
				  offsets[i]);
			phy_write(phydev, AT803X_MMD_ACCESS_CONTROL,
				  AT803X_FUNC_DATA);
			phy_write(phydev, AT803X_MMD_ACCESS_CONTROL_DATA,
				  mac[(i * 2) + 1] | (mac[(i * 2)] << 8));
		}

		value = phy_read(phydev, AT803X_INTR_ENABLE);
		value |= AT803X_INTR_ENABLE_WOL;
		ret = phy_write(phydev, AT803X_INTR_ENABLE, value);
		if (ret)
			return ret;
		value = phy_read(phydev, AT803X_INTR_STATUS);
	} else {
		value = phy_read(phydev, AT803X_INTR_ENABLE);
		value &= (~AT803X_INTR_ENABLE_WOL);
		ret = phy_write(phydev, AT803X_INTR_ENABLE, value);
		if (ret)
			return ret;
		value = phy_read(phydev, AT803X_INTR_STATUS);
	}

	return ret;
}

static void at803x_get_wol(struct phy_device *phydev,
			   struct ethtool_wolinfo *wol)
{
	u32 value;

	wol->supported = WAKE_MAGIC;
	wol->wolopts = 0;

	value = phy_read(phydev, AT803X_INTR_ENABLE);
	if (value & AT803X_INTR_ENABLE_WOL)
		wol->wolopts |= WAKE_MAGIC;
}

static int at803x_suspend(struct phy_device *phydev)
{
	int value;
	int wol_enabled;

	value = phy_read(phydev, AT803X_INTR_ENABLE);
	wol_enabled = value & AT803X_INTR_ENABLE_WOL;

	if (wol_enabled)
		value |= BMCR_ISOLATE;
	else
		value |= BMCR_PDOWN;

	phy_modify(phydev, MII_BMCR, 0, value);

	return 0;
}

static int at803x_resume(struct phy_device *phydev)
{
	return phy_modify(phydev, MII_BMCR, BMCR_PDOWN | BMCR_ISOLATE, 0);
}

static int at803x_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct at803x_priv *priv;

	pr_warn("at803x_probe: Mode: %s\n", phy_modes(phydev->interface));

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;

	if (phydev->mdio.dev.fwnode) {
		struct fwnode_reference_args ref;
		int ret;

		ret = fwnode_property_get_reference_args(phydev->mdio.dev.fwnode,
							 "sfp", NULL, 0, 0,
							 &ref);
		if (ret == 0)
			priv->sfp_fwnode = ref.fwnode;
	}

	return 0;
}

static void at803x_remove(struct phy_device *phydev)
{
	struct at803x_priv *priv = phydev->priv;

	if (priv->sfp_bus)
		sfp_unregister_upstream(priv->sfp_bus);

	fwnode_handle_put(priv->sfp_fwnode);
}


static int at803x_mode(struct phy_device *phydev)
{
	int mode;

	mode = phy_read(phydev, AT803X_REG_CHIP_CONFIG) & AT803X_MODE_CFG_MASK;

	if (mode == AT803X_MODE_CFG_BX1000_RGMII_50 ||
	    mode == AT803X_MODE_CFG_BX1000_RGMII_75)
		return AT803X_MODE_FIBER;
	return AT803X_MODE_COPPER;
}

static int at803x_config_init(struct phy_device *phydev)
{
	struct at803x_priv *priv = phydev->priv;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(supported) = { 0, };
	int ret;
	u32 v;
	int phy_mode;

	/* Check that the PHY interface type is compatible */
	if (!phy_interface_mode_is_rgmii(phydev->interface)) {
		pr_warn("at803x_config_init: %s not supported!\n", phy_modes(phydev->interface));
		return -ENODEV;
	}


	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_RXID ||
			phydev->interface == PHY_INTERFACE_MODE_RGMII_ID) {
		ret = at803x_enable_rx_delay(phydev);
		if (ret < 0)
			return ret;
	}

	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID ||
			phydev->interface == PHY_INTERFACE_MODE_RGMII_ID) {
		ret = at803x_enable_tx_delay(phydev);
		if (ret < 0)
			return ret;
	}
	
	pr_warn("at803x_config_init: Mode: %s\n", phy_modes(phydev->interface));

	__set_bit(ETHTOOL_LINK_MODE_Pause_BIT, supported);
	__set_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, supported);

	if ( (phydev->drv->phy_id == ATH8031_PHY_ID &&
		phydev->interface == PHY_INTERFACE_MODE_SGMII) ||
		(at803x_mode(phydev) == AT803X_MODE_FIBER) )
	{
		v = phy_read(phydev, AT803X_REG_CHIP_CONFIG);
		/* select SGMII/fiber page */

		ret = phy_write(phydev, AT803X_REG_CHIP_CONFIG,
						v & ~AT803X_BT_BX_REG_SEL);
		if (ret)
			return ret;

		/* enable SGMII autonegotiation */
		ret = phy_write(phydev, MII_BMCR, AT803X_SGMII_ANEG_EN);
		if (ret)
			return ret;

		/* select copper page */
		ret = phy_write(phydev, AT803X_REG_CHIP_CONFIG,
						v | AT803X_BT_BX_REG_SEL);
		if (ret)
			return ret;
		__set_bit(ETHTOOL_LINK_MODE_FIBRE_BIT, supported);
		__set_bit(ETHTOOL_LINK_MODE_1000baseX_Full_BIT, supported);
		__set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT, supported);
		__set_bit(ETHTOOL_LINK_MODE_MII_BIT, supported);
	} else {
		__set_bit(ETHTOOL_LINK_MODE_TP_BIT, supported);
		__set_bit(ETHTOOL_LINK_MODE_MII_BIT, supported);
			__set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
				  supported);			
			__set_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
				  supported);
			__set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT,
				  supported);
			__set_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT,
				  supported);
			__set_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT,
				  supported);
			__set_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT,
				  supported);
	}
	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;

	linkmode_copy(phydev->supported, supported);
	linkmode_and(phydev->advertising, phydev->advertising,
		     phydev->supported);

	/* Would be nice to do this in the probe function, but unfortunately,
	 * phylib doesn't have phydev->attached_dev set there.
	 */
	if (priv->sfp_fwnode && !priv->sfp_bus)
		priv->sfp_bus = sfp_register_upstream(priv->sfp_fwnode,
						      phydev->attached_dev,
						      phydev, &at803x_sfp_ops);

	return 0;
}

static int at803x_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, AT803X_INTR_STATUS);

	return (err < 0) ? err : 0;
}

static int at803x_config_intr(struct phy_device *phydev)
{
	int err;
	int value;

	value = phy_read(phydev, AT803X_INTR_ENABLE);

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		value |= AT803X_INTR_ENABLE_AUTONEG_ERR;
		value |= AT803X_INTR_ENABLE_SPEED_CHANGED;
		value |= AT803X_INTR_ENABLE_DUPLEX_CHANGED;
		value |= AT803X_INTR_ENABLE_LINK_FAIL;
		value |= AT803X_INTR_ENABLE_LINK_SUCCESS;

		err = phy_write(phydev, AT803X_INTR_ENABLE, value);
	}
	else
		err = phy_write(phydev, AT803X_INTR_ENABLE, 0);

	return err;
}

static void at803x_link_change_notify(struct phy_device *phydev)
{
	struct at803x_priv *priv = phydev->priv;
	enum phy_state state = phydev->state;
	bool running;

	//pr_warn("at803x_link_change: Mode: %s\n", phy_modes(phydev->interface));

	/*
	 * Conduct a hardware reset for AT8030 every time a link loss is
	 * signalled. This is necessary to circumvent a hardware bug that
	 * occurs when the cable is unplugged while TX packets are pending
	 * in the FIFO. In such cases, the FIFO enters an error mode it
	 * cannot recover from by software.
	 */
	if (phydev->state == PHY_NOLINK) {
		if (phydev->mdio.reset && !priv->phy_reset) {
			struct at803x_context context;

			at803x_context_save(phydev, &context);

			phy_device_reset(phydev, 1);
			msleep(1);
			phy_device_reset(phydev, 0);
			msleep(1);

			at803x_context_restore(phydev, &context);

			phydev_dbg(phydev, "%s(): phy was reset\n",
				   __func__);
			priv->phy_reset = true;
		}
	} else {
		priv->phy_reset = false;
	}

	if (priv->sfp_bus && priv->state != state) {
		priv->state = state;

		running = state >= PHY_UP && state < PHY_HALTED;
		if (priv->running != running) {
				priv->running = running;
				if (running)
					sfp_upstream_start(priv->sfp_bus);
				else
					sfp_upstream_stop(priv->sfp_bus);
		}
	}
}


static int at803x_read_status(struct phy_device *phydev) {
	int ccr, ret, val;
	int pssr;

	phydev->speed = SPEED_UNKNOWN;
	phydev->duplex = DUPLEX_UNKNOWN;
	linkmode_zero(phydev->lp_advertising);
	phydev->link = 0;
	phydev->pause = 0;
	phydev->asym_pause = 0;
	phydev->mdix = 0;

	/* Handle (Fiber) SGMII to RGMII mode */
	if (at803x_mode(phydev) == AT803X_MODE_FIBER) {

		ccr = phy_read(phydev, AT803X_REG_CHIP_CONFIG);
		/* select SGMII/fiber page */
		ret = phy_write(phydev, AT803X_REG_CHIP_CONFIG,
						ccr & ~AT803X_BT_BX_REG_SEL);
		if (ret)
			return ret;

		/* check if the SGMII link is OK. */
		pssr = phy_read(phydev, AT803X_PSSR);

		//pr_warn("803x_read_status: %x\n", pssr);

		if (pssr < 0)
		    return pssr;

		/* select Copper page */
		ret = phy_write(phydev, AT803X_REG_CHIP_CONFIG,
						ccr | AT803X_BT_BX_REG_SEL);
		if (ret)
			return ret;

		val = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_STAT1);
		if (val < 0)
			return val;

#if 0
		if (val & MDIO_AN_STAT1_COMPLETE) {
			/* Read the link partner's 1G advertisement */
			val = phy_read_mmd(phydev, MDIO_MMD_AN, MV_AN_STAT1000);
			if (val < 0)
				return val;

			mii_stat1000_mod_linkmode_lpa_t(phydev->lp_advertising, val);

			if (phydev->autoneg == AUTONEG_ENABLE)
				phy_resolve_aneg_linkmode(phydev);
		}
#endif

#define		PSSR_LINK      BIT(10)
#define		PSSR_SYNC_STATUS      BIT(8)

		if ((pssr & PSSR_SYNC_STATUS) && (pssr & PSSR_LINK))
			phydev->link = 1;

		phydev->speed = SPEED_1000;
		phydev->duplex = DUPLEX_FULL;


		return 0;
	}

	return genphy_read_status(phydev);
}

static int at803x_config_aneg(struct phy_device *phydev)
{
	pr_warn("at803x_config_aneg: Mode: %s\n", phy_modes(phydev->interface));
	return genphy_config_aneg(phydev);
}

static int at803x_aneg_done(struct phy_device *phydev)
{
	int ccr, ret;
	int aneg_done;

	pr_warn("803x_aneg_done: enter\n");

	if (at803x_mode(phydev) == AT803X_MODE_FIBER) {

		aneg_done = BMSR_ANEGCOMPLETE;

		ccr = phy_read(phydev, AT803X_REG_CHIP_CONFIG);
		/* select SGMII/fiber page */
		ret = phy_write(phydev, AT803X_REG_CHIP_CONFIG,
						ccr & ~AT803X_BT_BX_REG_SEL);
		if (ret)
			return ret;
		/* check if the SGMII link is OK. */
		if (!(phy_read(phydev, AT803X_PSSR) & AT803X_PSSR_MR_AN_COMPLETE)) {
			pr_warn("803x_aneg_done: SGMII link is not ok\n");
			aneg_done = 0;
		}

		/* select Copper page */
		ret = phy_write(phydev, AT803X_REG_CHIP_CONFIG,
						ccr | AT803X_BT_BX_REG_SEL);
		if (ret)
			return ret;

		return aneg_done;
	}

	aneg_done = genphy_aneg_done(phydev);
	if (aneg_done != BMSR_ANEGCOMPLETE)
		return aneg_done;

	/*
	 * in SGMII mode, if copper side autoneg is successful,
	 * also check SGMII side autoneg result
	 */
	ccr = phy_read(phydev, AT803X_REG_CHIP_CONFIG);
	if ((ccr & AT803X_MODE_CFG_MASK) != AT803X_MODE_CFG_SGMII)
		return aneg_done;

	/* switch to SGMII/fiber page */
	phy_write(phydev, AT803X_REG_CHIP_CONFIG, ccr & ~AT803X_BT_BX_REG_SEL);

	/* check if the SGMII link is OK. */
	if (!(phy_read(phydev, AT803X_PSSR) & AT803X_PSSR_MR_AN_COMPLETE)) {
		phydev_warn(phydev, "803x_aneg_done: SGMII link is not ok\n");
		aneg_done = 0;
	}
	/* switch back to copper page */
	phy_write(phydev, AT803X_REG_CHIP_CONFIG, ccr | AT803X_BT_BX_REG_SEL);

	return aneg_done;
}

static struct phy_driver at803x_driver[] = {
{
	/* ATHEROS 8035 */
	.phy_id			= ATH8035_PHY_ID,
	.name			= "Atheros 8035 ethernet",
	.phy_id_mask		= AT803X_PHY_ID_MASK,
	.probe			= at803x_probe,
	.config_init		= at803x_config_init,
	.set_wol		= at803x_set_wol,
	.get_wol		= at803x_get_wol,
	.suspend		= at803x_suspend,
	.resume			= at803x_resume,
	.features		= PHY_GBIT_FEATURES,
	.ack_interrupt		= at803x_ack_interrupt,
	.config_intr		= at803x_config_intr,
}, {
	/* ATHEROS 8030 */
	.phy_id			= ATH8030_PHY_ID,
	.name			= "Atheros 8030 ethernet",
	.phy_id_mask		= AT803X_PHY_ID_MASK,
	.probe			= at803x_probe,
	.config_init		= at803x_config_init,
	.link_change_notify	= at803x_link_change_notify,
	.set_wol		= at803x_set_wol,
	.get_wol		= at803x_get_wol,
	.suspend		= at803x_suspend,
	.resume			= at803x_resume,
	.features		= PHY_BASIC_FEATURES,
	.ack_interrupt		= at803x_ack_interrupt,
	.config_intr		= at803x_config_intr,
}, {
	/* ATHEROS 8031 */
	.phy_id			= ATH8031_PHY_ID,
	.name			= "Atheros 8031 ethernet",
	.phy_id_mask		= AT803X_PHY_ID_MASK,
	.probe			= at803x_probe,
	.remove			= at803x_remove,
	.config_init		= at803x_config_init,
	.set_wol		= at803x_set_wol,
	.get_wol		= at803x_get_wol,
	.suspend		= at803x_suspend,
	.resume			= at803x_resume,
	.features		= PHY_GBIT_FEATURES,
	.config_aneg		= at803x_config_aneg,
	.read_status		= at803x_read_status,
	.aneg_done		= at803x_aneg_done,
	.ack_interrupt		= at803x_ack_interrupt,
	.config_intr		= at803x_config_intr,
	.set_loopback		= genphy_loopback,
	.get_sset_count		= at803x_phy_get_sset_count,
	.get_strings		= at803x_phy_get_strings,
	.get_stats		= at803x_phy_get_stats,
	.link_change_notify	= at803x_link_change_notify,
} };

module_phy_driver(at803x_driver);

static struct mdio_device_id __maybe_unused atheros_tbl[] = {
	{ ATH8030_PHY_ID, AT803X_PHY_ID_MASK },
	{ ATH8031_PHY_ID, AT803X_PHY_ID_MASK },
	{ ATH8035_PHY_ID, AT803X_PHY_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, atheros_tbl);
