// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// ported by gobftald from Linux: drivers/net/phy/mdio-bcm-unimac.c

#include <lib/mmio/mmio.h>
#include <lib/zx/clock.h>
#include <sys/types.h>

#include <ddktl/device.h>

#include "bcmgenet_.h"
#include "mdio-bcm-unimac.h"

namespace eth {

static inline void unimac_mdio_start(const std::optional<ddk::MmioBuffer>& mdio) {
	uint32_t reg = mdio->Read32(MDIO_CMD);
	reg |= MDIO_START_BUSY;
	mdio->Write32(reg, MDIO_CMD);
}

int unimac_mdio_read(const std::optional<ddk::MmioBuffer>& mdio, int phy_addr, int reg) {
	// Prepare the read operation
	uint32_t cmd = MDIO_RD | (phy_addr << MDIO_PMD_SHIFT) | (reg << MDIO_REG_SHIFT);
	mdio->Write32(cmd, MDIO_CMD);

	// Start MDIO transaction
	unimac_mdio_start(mdio);

  // Scan and read value
	zx::time deadline = zx::deadline_after(zx::msec(3));
	do {
	  if (!mdio->ReadMasked32(MDIO_START_BUSY, MDIO_CMD)) {
	    printf("* unimac_mdio_read(reg = 0x%x) -> 0x%x\n", reg, mdio->Read32(MDIO_CMD) & 0xffff);
	    return mdio->Read32(MDIO_CMD) & 0xffff;
	  }
	  zx::nanosleep(zx::deadline_after(zx::usec(10)));
	} while (zx::clock::get_monotonic() < deadline);
  return ZX_ERR_TIMED_OUT;
}

int unimac_mdio_write(const std::optional<ddk::MmioBuffer>& mdio,
                             int phy_addr, int reg, uint32_t val) {
	// Prepare the write operation
	uint32_t cmd = MDIO_WR |
	               (phy_addr << MDIO_PMD_SHIFT) | (reg << MDIO_REG_SHIFT) | (val & 0xffff);
	mdio->Write32(cmd, MDIO_CMD);

	// Start MDIO transaction
	unimac_mdio_start(mdio);

  // Scan and read value
	zx::time deadline = zx::deadline_after(zx::msec(3));
	do {
	  if (!mdio->ReadMasked32(MDIO_START_BUSY, MDIO_CMD)) {
	    printf("* unimac_mdio_write(reg = 0x%x, val = 0x%x\n", reg, val);
	    return ZX_OK;
	  }
	  zx::nanosleep(zx::deadline_after(zx::usec(10)));
	} while (zx::clock::get_monotonic() < deadline);
  return ZX_ERR_TIMED_OUT;
}

}  // namespace eth
