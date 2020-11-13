// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// created by gobftald

#ifndef SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_PHY_DEVICE_H_
#define SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_PHY_DEVICE_H_

static inline int phy_read(ddk::EthMacProtocolClient& eth_mac, uint32_t regnum) {
	uint32_t val;
	eth_mac.MdioRead(regnum, &val);
	return val;
}

static inline int phy_write(ddk::EthMacProtocolClient& eth_mac, uint32_t regnum, uint32_t val) {
  return eth_mac.MdioWrite(regnum, val);
  return unimac_mdio_write(const std::optional<ddk::MmioBuffer>& mdio, phy_addr_, reg, val);
}



int genphy_read_abilities(const std::optional<ddk::MmioBuffer>& mdio)
{
	int val;
	
	linkmode_set_bit_array(phy_basic_ports_array,
			       ARRAY_SIZE(phy_basic_ports_array),
			       phydev->supported);

	val = phy_read(phydev, MII_BMSR);
	lprintk("val = phy_read(phydev, MII_BMSR) = 0x%x\n", val);
	if (val < 0)
		return val;

	lprintk("ETHTOOL_LINK_MODE_Autoneg_BIT = 0x%x\n", val & BMSR_ANEGCAPABLE);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, phydev->supported,
			 val & BMSR_ANEGCAPABLE);

	lprintk("ETHTOOL_LINK_MODE_100baseT_Full_BIT = 0x%x\n", val & BMSR_100FULL);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, phydev->supported,
			 val & BMSR_100FULL);
	lprintk("ETHTOOL_LINK_MODE_100baseT_Half_BIT = 0x%x\n", val & BMSR_100HALF);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT, phydev->supported,
			 val & BMSR_100HALF);
	lprintk("ETHTOOL_LINK_MODE_10baseT_Full_BIT = 0x%x\n", val & BMSR_10FULL);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT, phydev->supported,
			 val & BMSR_10FULL);
	lprintk("ETHTOOL_LINK_MODE_10baseT_Half_BIT = 0x%x\n", val & BMSR_10HALF);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT, phydev->supported,
			 val & BMSR_10HALF);

	if (val & BMSR_ESTATEN) {
		lprintk("if (val & BMSR_ESTATEN)\n");
		val = phy_read(phydev, MII_ESTATUS);
		lprintk("val = phy_read(phydev, MII_ESTATUS) = 0x%x\n", val);
		if (val < 0)
			return val;

  	lprintk("ETHTOOL_LINK_MODE_1000baseT_Full_BIT = 0x%x\n", val & ESTATUS_1000_TFULL);
		linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
				 phydev->supported, val & ESTATUS_1000_TFULL);
  	lprintk("ETHTOOL_LINK_MODE_1000baseT_Half_BIT = 0x%x\n", val & ESTATUS_1000_THALF);
		linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
				 phydev->supported, val & ESTATUS_1000_THALF);
  	lprintk("ETHTOOL_LINK_MODE_1000baseX_Full_BIT = 0x%x\n", val & ESTATUS_1000_XFULL);
		linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseX_Full_BIT,
				 phydev->supported, val & ESTATUS_1000_XFULL);
	}

  lprintk_restore;
	return 0;
}
EXPORT_SYMBOL(genphy_read_abilities);

#endif  // SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_PHY_DEVICE_H_

