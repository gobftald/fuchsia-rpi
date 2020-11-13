// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// created by gobftald

#ifndef SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_MDIO_BCM_UNIMAC_H_
#define SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_MDIO_BCM_UNIMAC_H_

namespace eth {

int unimac_mdio_read(const std::optional<ddk::MmioBuffer>& mdio, int phy_addr, int reg);
int unimac_mdio_write(const std::optional<ddk::MmioBuffer>& mdio,
                      int phy_addr, int reg, uint32_t val);

}  // namespace eth

#endif  // SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_MDIO_BCM_UNIMAC_H_
