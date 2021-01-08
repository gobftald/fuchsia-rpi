// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// created by gobftald

#ifndef SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_UNIMAC_GEN_PHY_H_
#define SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_UNIMAC_GEN_PHY_H_

#include "unimac.h"

#include <threads.h>
#include <lib/mmio/mmio.h>

#define SPEED_UNKNOWN           -1
#define DUPLEX_UNKNOWN          -1

enum phy_state {
        PHY_STARTED = 0,
        PHY_READY,
        PHY_RUNNING,
        PHY_NOLINK,
        PHY_HALTED,
};


namespace eth {

// Generic Broadcom PHY device
// implements PHY state-machine, and operations via MDIO bus
class GenPhy {
 public:
  GenPhy(const ddk::MmioView& mdio) : mdio_(mdio) {}
  GenPhy(const ddk::MmioView& mdio, uint32_t addr, uint32_t id);
  static zx_status_t Create(const ddk::MmioView& mdio, std::optional<GenPhy> *pphy);

  int Read(uint32_t reg);                         // MDIO read
  zx_status_t Write(uint32_t reg, uint32_t val);  // MDIO write
  uint32_t GetId() { return id_; }                // Get PHY physical ID
  zx_status_t Init(bool (*)(uint32_t));           // Init PHY state machine

private:
  thrd_t thread_;
  int Thread();                                   // Polling thread

  void Start();                                   // Start MDIO read/write operations
  zx_status_t ReadAbilities();                    // Reads the PHY's abilities
  zx_status_t ConfigAneg();                       // Config auto-negotiation
  zx_status_t CheckLinkStatus();                  // Check and set link state
  int ConfigAdvert();                             // Config advertised capabilites
  int ModifyChanged(uint32_t, uint32_t, uint32_t);    // Helper function for read/change/write
  zx_status_t ReadStatus();                       // Check link and link partner
  zx_status_t UpdateLink();                       // Reflect the curren link value
  zx_status_t ReadLpa();                          // Read link partner advertisement
  void ResolveAnegLinkmode();                     // Read our and LP advertisement
  void SetupLink();                               // Setup link when change
  bool (*SetupMii)(uint32_t);                     // SetupMii callback to MAC

  ddk::MmioView mdio_;                            // MDIO bus addresses

  uint32_t addr_;                                 // phy's address on the mdio bus
  uint32_t id_;                                   // phy's proudct id: PDEV_PID_BCM...

  int speed_ = SPEED_UNKNOWN;                     // Speed change status
  int old_speed_;
  int duplex_ = DUPLEX_UNKNOWN;                   // Duplex change status
  int old_duplex_;
  bool link_ = 0;                                 // Link actual state
  bool old_link_;

  uint32_t supported_ = 0;                        // Link Modes supported
  uint32_t advertising_;                          // Link Modes advertising
  uint32_t lp_advertising_;                       // Link partner advertisements
  bool is_gigabit_capable_;                       // Gigabit support
  bool autoneg_complete_;                         // Auto-negotiation status
  bool is_mac_ready_ = false;                     // Waiting for MAC initialisation

  enum phy_state state_;                          // Actual state of state machine
};

}  // namespace eth

#endif  // SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_UNIMAC_GEN_PHY_H_
