// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// created by gobftald

#ifndef SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_UNIMAC_PHY_H_
#define SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_UNIMAC_PHY_H_

#define PHY_MAX_ADDR            32                // max number of phys on mii bus
#define PHY_POLL_INTERVAL       1                 // PHY's poll cycle interval in sec


// helpers for managing our own advertisements

// for easy debugging it follows Linux bitmap
// but forced to fit into a single uint32_t
enum link_mode_bit_indices {
  LINK_MODE_10baseT_Half_BIT    = 0,
  LINK_MODE_10baseT_Full_BIT    = 1,
  LINK_MODE_100baseT_Half_BIT   = 2,
  LINK_MODE_100baseT_Full_BIT   = 3,
  LINK_MODE_1000baseT_Half_BIT  = 4,
  LINK_MODE_1000baseT_Full_BIT  = 5,
  LINK_MODE_Autoneg_BIT         = 6,
  LINK_MODE_TP_BIT              = 7,
  LINK_MODE_AUI_BIT             = 8,
  LINK_MODE_MII_BIT             = 9,

  LINK_MODE_1000baseX_Full_BIT  = 31,   // 41 in Linux
};

inline void linkmode_mod_bit(const int nr, uint32_t& addr, const uint32_t set) {
  if (set)
    addr |= 1u << nr;
  else
    addr &= ~(1u << nr);
}

inline int linkmode_test_bit(const int nr, const uint32_t& addr) {
  return (1u << nr) & addr ? 1 : 0;
}

// Translates linkmode advertisement settings to phy autonegotiation
// advertisements for the MII_ADVERTISE register.
inline uint32_t linkmode_adv_to_mii_adv_t(const uint32_t& advertising) {
  uint32_t result = 0;

  if (linkmode_test_bit(LINK_MODE_10baseT_Half_BIT, advertising))
    result |= ADVERTISE_10HALF;
  if (linkmode_test_bit(LINK_MODE_10baseT_Full_BIT, advertising))
    result |= ADVERTISE_10FULL;
  if (linkmode_test_bit(LINK_MODE_100baseT_Half_BIT, advertising))
    result |= ADVERTISE_100HALF;
  if (linkmode_test_bit(LINK_MODE_100baseT_Full_BIT, advertising))
    result |= ADVERTISE_100FULL;

  return result;
}

// Translates linkmode advertisement settings to phy autonegotiation
// advertisements for the MII_CTRL1000 register when in 1000T mode.
inline uint32_t linkmode_adv_to_mii_ctrl1000_t(const uint32_t& advertising) {
  uint32_t result = 0;

  if (linkmode_test_bit(LINK_MODE_1000baseT_Half_BIT, advertising))
    result |= ADVERTISE_1000HALF;
  if (linkmode_test_bit(LINK_MODE_1000baseT_Full_BIT, advertising))
    result |= ADVERTISE_1000FULL;

  return result;
}

// Translates MII_STAT1000 bits, when in 1000Base-T mode, to linkmode advertisement 
// settings. Other bits in advertising are not changes.
inline void mii_stat1000_mod_linkmode_lpa_t(uint32_t& advertising, uint32_t lpa) {
        linkmode_mod_bit(LINK_MODE_1000baseT_Half_BIT, advertising, lpa & LPA_1000HALF);
        linkmode_mod_bit(LINK_MODE_1000baseT_Full_BIT, advertising, lpa & LPA_1000FULL);
}

inline void mii_adv_mod_linkmode_adv_t(uint32_t& advertising, uint32_t adv) {
  linkmode_mod_bit(LINK_MODE_10baseT_Half_BIT, advertising, adv & ADVERTISE_10HALF);
  linkmode_mod_bit(LINK_MODE_10baseT_Full_BIT, advertising, adv & ADVERTISE_10FULL);
  linkmode_mod_bit(LINK_MODE_100baseT_Half_BIT, advertising, adv & ADVERTISE_100HALF);
  linkmode_mod_bit(LINK_MODE_100baseT_Full_BIT, advertising, adv & ADVERTISE_100FULL);
}

inline void mii_lpa_mod_linkmode_lpa_t(uint32_t& lp_advertising, uint32_t lpa) {
        mii_adv_mod_linkmode_adv_t(lp_advertising, lpa);
        linkmode_mod_bit(LINK_MODE_Autoneg_BIT, lp_advertising, lpa & LPA_LPACK);
}


// helpers for managing link partner's advertisements

struct phy_setting {
        uint32_t speed;
        uint8_t duplex;
        uint8_t bit;
};

#define DUPLEX_HALF             0
#define DUPLEX_FULL             1

#define SPEED_10                10
#define SPEED_100               100
#define SPEED_1000              1000

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
#define PHY_SETTING(s, d, b) { .speed = s, .duplex = DUPLEX_ ## d, \
                               .bit = LINK_MODE_ ## b ## _BIT}

const struct phy_setting settings[] = {
  // 1G
  PHY_SETTING(SPEED_1000, HALF, 1000baseT_Half ),
  PHY_SETTING(SPEED_1000, FULL, 1000baseT_Full ),
  PHY_SETTING(SPEED_1000, FULL, 1000baseX_Full ),
  // 100M
  PHY_SETTING( SPEED_100, FULL,  100baseT_Full ),
  PHY_SETTING( SPEED_100, HALF,  100baseT_Half ),
  // 10M
  PHY_SETTING(  SPEED_10, FULL,   10baseT_Full ),
  PHY_SETTING(  SPEED_10, HALF,   10baseT_Half ),
};

// helpers for PHY's print Satus and States

const char *phy_speed_to_str(int speed) {
  switch (speed) {
  case SPEED_10:    return "10Mbps";
  case SPEED_100:   return "100Mbps";
  case SPEED_1000:  return "1Gbps";
  default:
    return "Unsupported speed value";
  }
}

const char *phy_duplex_to_str(int duplex) {
  if (duplex == DUPLEX_HALF)
    return "Half";
  if (duplex == DUPLEX_FULL)
    return "Full";
  return "Unsupported duplex value";
}

const char *phy_state_to_str(int st) {
  switch (st) {
    case PHY_STARTED: return "STARTED";
    case PHY_READY:   return "READY";
    case PHY_RUNNING: return "RUNNING";
    case PHY_NOLINK:  return "NOLINK";
    case PHY_HALTED:  return "HALTED";
  }
  return NULL;
}

#endif  // SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_UNIMAC_PHY_H_
