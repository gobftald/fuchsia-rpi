// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// ported by gobftald from Linux: drivers/net/phy/mdio-bcm-unimac.c
//                                drivers/net/phy/phy_device.c
//                                drivers/net/phy/phy.c
//                                drivers/net/phy/phy-core.c                      

#include "gen-phy.h"
#include "phy.h"

#include <ddktl/device.h>
#include <lib/zx/clock.h>


namespace eth {

GenPhy::GenPhy(const ddk::MmioView& mdio, uint32_t addr, uint32_t id) : mdio_(mdio) {
  addr_ = addr;
  id_ = id;
}

zx_status_t GenPhy::Create(const ddk::MmioView& mdio, std::optional<GenPhy> *pphy) {
  // create instance  
  GenPhy phy(mdio);

  // Search for a responding phy (address) on MDIO bus.
  for (phy.addr_ = 0;
       phy.Read(MII_PHYSID1) == 0xffff && phy.addr_ < PHY_MAX_ADDR;
       phy.addr_++);
  if (phy.addr_ == PHY_MAX_ADDR) {
    zxlogf(ERROR, "GenPhy::Create: Could not find any phy device");
    return ZX_ERR_NOT_FOUND;
  }

  // Get PHY's HW id, which will be used for binding Zircon device driver.
  uint32_t reg = phy.Read(MII_PHYSID1);
  if (reg < 0) {
    return reg;
  }
  phy.id_ = reg << 16;
  if ((reg = phy.Read(MII_PHYSID2)) < 0) {
    return reg;
  }
  phy.id_ |= reg;

  pphy->emplace(mdio, phy.addr_, phy.id_);
  return ZX_OK;
}

inline void GenPhy::Start() {
  uint32_t reg = mdio_.Read32(MDIO_CMD);
  reg |= MDIO_START_BUSY;
  mdio_.Write32(reg, MDIO_CMD);
}

// Returns < 0 on error
// returns 16 bit read value >= 0
int GenPhy::Read(uint32_t reg) {
  // Prepare the read operation
  uint32_t cmd = MDIO_RD | (addr_ << MDIO_PMD_SHIFT) | (reg << MDIO_REG_SHIFT);
  mdio_.Write32(cmd, MDIO_CMD);

  // Start MDIO transaction
  Start();

  // Initial sleep, based on a max value of majority
  // MDIO interrupt driven method was 10-100 times slower
  zx_nanosleep(zx_deadline_after(ZX_NSEC(1400))); 
  // Scan and read value
  uint32_t val;
  zx_time_t deadline = zx_deadline_after(ZX_MSEC(1));
  do {
    if (!mdio_.ReadMasked32(MDIO_START_BUSY, MDIO_CMD)) {
      val = mdio_.Read32(MDIO_CMD) & 0xffff;
      //zxlogf(DEBUG, "GenPhy::Read(reg=0x%x) -> 0x%x", reg, val);
      return val;
    }
    zx_nanosleep(zx_deadline_after(ZX_NSEC(200))); 
  } while (zx_clock_get_monotonic() < deadline);

  return ZX_ERR_TIMED_OUT;
}

zx_status_t GenPhy::Write(uint32_t reg, uint32_t val) {
  // Prepare the write operation
  uint32_t cmd = MDIO_WR |
                 (addr_ << MDIO_PMD_SHIFT) | (reg << MDIO_REG_SHIFT) | (val & 0xffff);
  mdio_.Write32(cmd, MDIO_CMD);

  // Start MDIO transaction
  Start();

  // Initial sleep, based on a max value of majority
  // MDIO interrupt driven method was 10-500 times slower
  zx_nanosleep(zx_deadline_after(ZX_NSEC(1200))); 

  // Scan for readiness
  zx_time_t deadline = zx_deadline_after(ZX_MSEC(1));
  do {
    if (!mdio_.ReadMasked32(MDIO_START_BUSY, MDIO_CMD)) {
      //zxlogf(DEBUG, "GenPhy::Write(reg=0x%x, val=0x%x", reg, val);
      return ZX_OK;
    }
    zx_nanosleep(zx_deadline_after(ZX_NSEC(300))); 
  } while (zx_clock_get_monotonic() < deadline);

  return ZX_ERR_TIMED_OUT;
}

zx_status_t GenPhy::Init(bool (*callback)(uint32_t)) {
  // Callback to MAC, configuring Mii block and signalling link state change to the world
  SetupMii = callback;
  old_speed_ = -1;
  old_duplex_ = -1;
  old_link_ = -1;

  // Reads the PHY's abilities and populates supported_ accordingly.
  auto status = ReadAbilities();
  if (status != ZX_OK) {
    zxlogf(ERROR, "GenPhy::Create: could not read PHY's abilities");
    return status;
  }

  // Auto-negotiation is default
  if (!linkmode_test_bit(LINK_MODE_Autoneg_BIT, supported_)) {
    zxlogf(ERROR, "GenPhy::Create: Auto-negotiation is not supported");
    return ZX_ERR_NOT_SUPPORTED;
  }

  // Gigabit capability
  if (linkmode_test_bit(LINK_MODE_1000baseT_Half_BIT, supported_) ||
      linkmode_test_bit(LINK_MODE_1000baseT_Full_BIT, supported_)) {
    is_gigabit_capable_ = true;
  }

  // Advertise supported capabilities
  advertising_ = supported_;

  // Initial state of state-machine
  state_ = PHY_STARTED;

  // Launch state-machine thread
  auto thunk = [](void* arg) -> int { return reinterpret_cast<GenPhy*>(arg)->Thread(); };
  status = thrd_create_with_name(&thread_, thunk, this, "phy_thread");
  ZX_DEBUG_ASSERT(status == thrd_success);

  return ZX_OK;
}

// Reads PHY default abilities
zx_status_t GenPhy::ReadAbilities() {
  // Set default abilities
  linkmode_mod_bit(LINK_MODE_Autoneg_BIT, supported_, 1);
  linkmode_mod_bit(LINK_MODE_TP_BIT, supported_, 1);
  linkmode_mod_bit(LINK_MODE_MII_BIT, supported_, 1);

  uint32_t val = Read(MII_BMSR);
  if (val < 0) {
    return val;
  }

  linkmode_mod_bit(LINK_MODE_Autoneg_BIT, supported_, val & BMSR_ANEGCAPABLE);

  linkmode_mod_bit(LINK_MODE_100baseT_Full_BIT, supported_, val & BMSR_100FULL);
  linkmode_mod_bit(LINK_MODE_100baseT_Half_BIT, supported_, val & BMSR_100HALF);
  linkmode_mod_bit(LINK_MODE_10baseT_Full_BIT, supported_, val & BMSR_10FULL);
  linkmode_mod_bit(LINK_MODE_10baseT_Half_BIT, supported_, val & BMSR_10HALF);

  if (val & BMSR_ESTATEN) {
    if ((val = Read(MII_ESTATUS)) < 0) {
      return val;
    }

    linkmode_mod_bit(LINK_MODE_1000baseT_Full_BIT, supported_, val & ESTATUS_1000_TFULL);
    linkmode_mod_bit(LINK_MODE_1000baseT_Half_BIT, supported_, val & ESTATUS_1000_THALF);
    linkmode_mod_bit(LINK_MODE_1000baseX_Full_BIT, supported_, val & ESTATUS_1000_XFULL);
  }
  return ZX_OK;
}

// Polling and managing link state changes
int GenPhy::Thread() {
  enum phy_state old_state;
  int err = 0;

  while (true) {

    old_state = state_;

    switch (state_) {
    case PHY_STARTED:
      err = ConfigAneg();
      if ( err < 0 ) {
        break;
      }
      state_ = PHY_READY;
      // !!!! continue
    case PHY_READY:
      err = CheckLinkStatus();
      break;      
    case PHY_NOLINK:
    case PHY_RUNNING:
      err = CheckLinkStatus();
      break;
    case PHY_HALTED:
      if (link_) {
        link_ = 0;
        SetupLink();
      }
      break;
    }

    if (err < 0) {
      state_ = PHY_HALTED;
      continue;
    }

    // MAC initialisation has just finished, don't wait for another polling cycle
    if (state_ == PHY_READY && is_mac_ready_) {
      continue;
    }

    // If MAC is initialising check the end of that more frequently
    if (state_ == PHY_READY && link_ && !is_mac_ready_) {
      zx_nanosleep(zx_deadline_after(ZX_MSEC(10))); 
      continue;
    }

    // Log state changes
    if (old_state != state_ ) {
      zxlogf(DEBUG, "GenPhy::Thread: PHY state change %s -> %s\n",
             phy_state_to_str(old_state),
             phy_state_to_str(state_));
    }

    zx_nanosleep(zx_deadline_after(ZX_SEC(3)));   
  } 
  return 0;
}

// Config auto-negotiation
zx_status_t GenPhy::ConfigAneg() {
  int err, ctl, changed = false;

  if ((err = ConfigAdvert()) < 0) {
    return err;
  } else if (err > 0) {
    changed = true;
  }

  if (!changed) {
    if ((ctl = Read(MII_BMCR)) < 0) {
      return ctl;
    }

    if (!(ctl & BMCR_ANENABLE)) {
      changed = true;   // do restart aneg
    }
  }

  if (changed) {
    // Restart auto-negotiation
    if ((err = ModifyChanged(MII_BMCR, 0, BMCR_ANENABLE | BMCR_ANRESTART)) < 0) {
      return err;
    }
  }
  return ZX_OK;
}

// Check link and set state accordingly
zx_status_t GenPhy::CheckLinkStatus() {
  int err;

  if ((err = ReadStatus()) != ZX_OK) {
    return err;
  }

  // Run MAC callback until it is not ready
  if (state_ == PHY_READY) {
    if (!is_mac_ready_) {
      SetupLink();
      return ZX_OK;
    }
  }

  // Track the main state changes
  if (link_ && state_ != PHY_RUNNING) {
    state_ = PHY_RUNNING;
    SetupLink();
  } else if (!link_ && state_ != PHY_NOLINK) {
    state_ = PHY_NOLINK;
    SetupLink();
  }
  return ZX_OK;
}

// Writes MII_ADVERTISE with the appropriate values,
// make sure we only advertise what is supported.
// Returns < 0 on error,
// returns 0 if the PHY's advertisement hasn't changed,
// returns > 0 if it has changed.
int GenPhy::ConfigAdvert() {
  int err, bmsr, changed = 0;
  uint32_t adv;

  // Only allow advertising what this PHY supports
  advertising_ &= supported_;

  adv = linkmode_adv_to_mii_adv_t(advertising_);

  // Setup standard advertisement
  if ((err = ModifyChanged(MII_ADVERTISE, ADVERTISE_ALL, adv)) < 0) {
    return err;
  }

  if (err > 0) {
    changed = true;
  }

  if ((bmsr = Read(MII_BMSR)) < 0) {
    return bmsr;
  }

  // Per 802.3-2008, Section 22.2.4.2.16 Extended status
  // all 1000Mbits/sec capable PHYs shall have the BMSR_ESTATEN bit set to a logical 1.
  if (!(bmsr & BMSR_ESTATEN)) {
    return changed;
  }

  adv = linkmode_adv_to_mii_ctrl1000_t(advertising_);

  if ((err = ModifyChanged(MII_CTRL1000, ADVERTISE_1000FULL | ADVERTISE_1000HALF, adv)) < 0) {
    return err;
  }
  if (err > 0) {
    changed = 1;
  }
  return changed;
}

// Returns < 0 on error,
// returns 0 if no change
// returns 1 if value changed
int GenPhy::ModifyChanged(uint32_t reg, uint32_t mask, uint32_t set) {
  int ret;

  if ((ret = Read(reg)) < 0) {
    return ret;
  }

  int val = (ret & ~mask) | set;
  if (val == ret) {
    return 0;
  }

  ret = Write(reg, val);
  return ret < 0 ? ret : 1;
}

// Check the link, then figure out the current state by comparing
// what we advertise with what the link partner advertises.
zx_status_t GenPhy::ReadStatus() {
  int err, old_link = link_;

  // Update the link, but return if there was an error
  if ((err = UpdateLink()) != ZX_OK) {
    return err;
  }

  // why bother the PHY is up and not changed
  if (old_link && link_) {
    return ZX_OK;
  }

  speed_ = SPEED_UNKNOWN;
  duplex_ = DUPLEX_UNKNOWN;

  if ((err = ReadLpa()) != ZX_OK) {
    return err;
  }

  if (autoneg_complete_) {
    ResolveAnegLinkmode();
  }  
  return ZX_OK;
}

// Update the value in 'link_' to reflect the current link value. 
// return 0 -> update was OK,
// return < 0 -> reading error from MII_BMCR or MII_BMSR mdio registers
zx_status_t GenPhy::UpdateLink() {
  int status = 0, bmcr;

  bmcr = Read(MII_BMCR);
  if (bmcr < 0) {
    return bmcr;
  }

  // if autoneg is being started, disregard BMSR value
  // and report link as down.
  if (bmcr & BMCR_ANRESTART) {
    goto done;
  }

  // Read link and autonegotiation status
  if ((status = Read(MII_BMSR)) < 0) {
    return status;
  }

done:
  link_ = status & BMSR_LSTATUS ? true : false;
  autoneg_complete_ = status & BMSR_ANEGCOMPLETE ? true : false;

  // Consider the case that autoneg was started and "aneg complete" bit has been reset,
  // but "link up" bit not yet.
  if (!autoneg_complete_) {
    link_ = false;
  }
  return ZX_OK;
}

zx_status_t GenPhy::ReadLpa() {
  int lpa, lpagb;

  if (!autoneg_complete_) {
    mii_stat1000_mod_linkmode_lpa_t(lp_advertising_, 0);
    mii_lpa_mod_linkmode_lpa_t(lp_advertising_, 0);
    return 0;
  }

  if (is_gigabit_capable_) {
    if ((lpagb = Read(MII_STAT1000)) < 0) {
      return lpagb;
    }

    if (lpagb & LPA_1000MSFAIL) {
      int adv;
      if ((adv = Read(MII_CTRL1000)) < 0) {
        return adv;
      }
      if (adv & CTL1000_ENABLE_MASTER) {
        zxlogf(ERROR,
            "GenPhy::ReadLpa: Master/Slave resolution failed, maybe conflicting manual settings?");
      } else {
        zxlogf(ERROR, "GenPhy::ReadLpa: Master/Slave resolution failed");
      }
      return ZX_ERR_NOT_SUPPORTED;
    }

    mii_stat1000_mod_linkmode_lpa_t(lp_advertising_, lpagb);
  }

  if((lpa = Read(MII_LPA)) < 0) {
    return lpa;
  }

  mii_lpa_mod_linkmode_lpa_t(lp_advertising_, lpa);

  return ZX_OK;
}

// Resolve our and the link partner advertisements
// into their corresponding speed and duplex.
void GenPhy::ResolveAnegLinkmode() {
  // Select common abilities
  uint32_t common = lp_advertising_ & advertising_;

  // Set speed and duplex
  for (uint32_t i = 0; i < ARRAY_SIZE(settings); i++)
    if ((1u << settings[i].bit) & common) {
      speed_ = settings[i].speed;
      duplex_ = settings[i].duplex;
      break;
    }
}

// setup netdev link state and update
// UMAC and RGMII block when link up
void GenPhy::SetupLink() {
  uint32_t cmd_bits = 0;
  bool status_changed = false;

  if (old_link_ != link_) {
    status_changed = true;
    old_link_ = link_;
  }

  if (link_) {
    // check speed/duplex/pause changes
    if (old_speed_ != speed_) {
      status_changed = true;
      old_speed_ = speed_;
    }

    if (old_duplex_ != duplex_) {
      status_changed = true;
      old_duplex_ = duplex_;
    }

    // done if nothing has changed
    // except initial PHY_READY but MAC is not ready loop
    if (!PHY_READY && !status_changed) {
      return;
    }

    // speed
    if (speed_ == SPEED_1000) {
      cmd_bits = UMAC_SPEED_1000;
    } else if (speed_ == SPEED_100) {
      cmd_bits = UMAC_SPEED_100;
    } else {
      cmd_bits = UMAC_SPEED_10;
    }
    cmd_bits <<= CMD_SPEED_SHIFT;

    // duplex
    if (duplex_ != DUPLEX_FULL) {
      cmd_bits |= CMD_HD_EN;
    }

    // pause capability (default false)
    cmd_bits |= CMD_RX_PAUSE_IGNORE | CMD_TX_PAUSE_IGNORE;

    // Enable Mii communication
    is_mac_ready_ = SetupMii(cmd_bits);
  }

  else   { // link_ = false
    if (is_mac_ready_) {
      // Disable Mii communication    
      is_mac_ready_ = SetupMii(0);
    }
  }

  if (link_) {
    zxlogf(DEBUG, "GenPhy: Link is Up - %s/%s\n", phy_speed_to_str(speed_), phy_duplex_to_str(duplex_));
  } else {
    zxlogf(DEBUG, "GenPhy: Link is Down");
  }
}

}  // namespace eth
