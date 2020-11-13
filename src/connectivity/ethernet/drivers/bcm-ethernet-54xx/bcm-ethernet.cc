// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "bcm-ethernet.h"

#include <lib/device-protocol/i2c.h>
#include <lib/device-protocol/platform-device.h>
#include <stdio.h>
#include <string.h>
#include <zircon/compiler.h>

#include <iterator>

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/driver.h>
#include <ddk/metadata.h>
#include <ddk/platform-defs.h>
#include <ddk/protocol/composite.h>
#include <ddk/protocol/ethernet.h>
#include <ddk/protocol/platform/device.h>
#include <fbl/algorithm.h>
#include <fbl/alloc_checker.h>
#include <fbl/auto_call.h>
#include <fbl/auto_lock.h>
#include <hw/reg.h>
#include <soc/aml-s912/s912-hw.h>

#include "bcm-regs.h"

namespace eth {

#define MCU_I2C_REG_BOOT_EN_WOL 0x21
#define MCU_I2C_REG_BOOT_EN_WOL_RESET_ENABLE 0x03

zx_status_t BcmEthernet::EthBoardResetPhy() {
  if (has_reset_) {
    gpios_[PHY_RESET].Write(0);
    zx_nanosleep(zx_deadline_after(ZX_MSEC(100)));
    gpios_[PHY_RESET].Write(1);
    zx_nanosleep(zx_deadline_after(ZX_MSEC(100)));
  }
  return ZX_OK;
}

zx_status_t BcmEthernet::InitPdev() {
  composite_protocol_t composite;

  auto status = device_get_protocol(parent(), ZX_PROTOCOL_COMPOSITE, &composite);
  // parent() = ethernet_mac
  if (status != ZX_OK) {
    zxlogf(ERROR, "Could not get composite protocol");
    return status;
  }

  zx_device_t* fragments[FRAGMENT_COUNT];
  size_t actual;
  printf("# BcmEthernet::InitPdev: composite_get_fragments\n");
  composite_get_fragments(&composite, fragments, std::size(fragments), &actual);
  /*
  if (actual == std::size(fragments)) {
    has_reset_ = true;
  } else {
    if (actual == (std::size(fragments) - 1)) {
      has_reset_ = false;
    } else {
      zxlogf(ERROR, "could not get fragments");
      return ZX_ERR_NOT_SUPPORTED;
    }
  }
  */

  pdev_protocol_t pdev;
  printf("# BcmEthernet::InitPdev: device_get_protocol(fragments[FRAGMENT_PDEV], ZX_PROTOCOL_PDEV, &pdev)\n");
  status = device_get_protocol(fragments[FRAGMENT_PDEV], ZX_PROTOCOL_PDEV, &pdev);
  // fragments[FRAGMENT_PDEV] = fragment-proxy
  if (status != ZX_OK) {
    zxlogf(ERROR, "Could not get PDEV protocol");
    return status;
  }
  pdev_ = &pdev;

  /*
  i2c_protocol_t i2c;
  status = device_get_protocol(fragments[FRAGMENT_I2C], ZX_PROTOCOL_I2C, &i2c);
  if (status != ZX_OK) {
    zxlogf(ERROR, "Could not get I2C protocol");
    return status;
  }
  i2c_ = &i2c;

  gpio_protocol_t gpio;
  if (has_reset_) {
    printf("# device_get_protocol(fragments[FRAGMENT_RESET_GPIO], ZX_PROTOCOL_GPIO, &gpio)\n");
    status = device_get_protocol(fragments[FRAGMENT_RESET_GPIO], ZX_PROTOCOL_GPIO, &gpio);
    if (status != ZX_OK) {
      zxlogf(ERROR, "Could not get GPIO protocol");
      return status;
    }
    gpios_[PHY_RESET] = &gpio;
  }

  printf("# device_get_protocol(fragments[FRAGMENT_INTR_GPIO], ZX_PROTOCOL_GPIO, &gpio)\n");
  status = device_get_protocol(fragments[FRAGMENT_INTR_GPIO], ZX_PROTOCOL_GPIO, &gpio);
  if (status != ZX_OK) {
    zxlogf(ERROR, "Could not get GPIO protocol");
    return status;
  }
  gpios_[PHY_INTR] = &gpio;

  // Map broadcom peripheral control registers.
  printf("# pdev_.MapMmio(MMIO_PERIPH, &periph_mmio_)\n");
  status = pdev_.MapMmio(MMIO_PERIPH, &periph_mmio_);
  if (status != ZX_OK) {
    zxlogf(ERROR, "bcm-bcmac: could not map periph mmio: %d", status);
    return status;
  }

  // Map HHI regs (clocks and power domains).
  printf("# pdev_.MapMmio(MMIO_HHI, &hhi_mmio_)\n");
  status = pdev_.MapMmio(MMIO_HHI, &hhi_mmio_);
  if (status != ZX_OK) {
    zxlogf(ERROR, "bcm-bcmac: could not map hiu mmio: %d", status);
    return status;
  }
  */

  return status;
}

zx_status_t BcmEthernet::Bind() {
  /*
  // Set reset line to output if implemented
  if (has_reset_) {
    gpios_[PHY_RESET].ConfigOut(0);
  }

  // Initialize Broadcom peripheral registers associated with bcmac.
  // Sorry about the magic...rtfm
  periph_mmio_->Write32(0x1621, PER_ETH_REG0);
  */

  pdev_board_info_t board;
  //bool is_vim3 = false;
  printf("# BcmEthernet::Bind: pdev_.GetBoardInfo(&board)\n");
  zx_status_t status = pdev_.GetBoardInfo(&board);

  /*
  if (status == ZX_OK) {
    is_vim3 = ((board.vid == PDEV_VID_BROADCOM) &&
               (board.pid == PDEV_PID_VIM3));
  }

  if (!is_vim3) {
    periph_mmio_->Write32(0x20000, PER_ETH_REG1);
  }

  periph_mmio_->Write32(REG2_ETH_REG2_REVERSED | REG2_INTERNAL_PHY_ID, PER_ETH_REG2);

  periph_mmio_->Write32(REG3_CLK_IN_EN | REG3_ETH_REG3_19_RESVERD | REG3_CFG_PHY_ADDR |
                            REG3_CFG_MODE | REG3_CFG_EN_HIGH | REG3_ETH_REG3_2_RESERVED,
                        PER_ETH_REG3);

  // Enable clocks and power domain for dwmac
  hhi_mmio_->SetBits32(1 << 3, HHI_GCLK_MPEG1);
  hhi_mmio_->ClearBits32((1 << 3) | (1 << 2), HHI_MEM_PD_REG0);

  // WOL reset enable to MCU
  uint8_t write_buf[2] = {MCU_I2C_REG_BOOT_EN_WOL, MCU_I2C_REG_BOOT_EN_WOL_RESET_ENABLE};
  status = i2c_.WriteSync(write_buf, sizeof(write_buf));
  if (status) {
    zxlogf(ERROR, "bcm-ethernet: WOL reset enable to MCU failed: %d", status);
    return status;
  }
  */

  // Populate board specific information
  eth_dev_metadata_t mac_info;
  size_t actual;
  status = device_get_metadata(parent(), DEVICE_METADATA_ETH_MAC_DEVICE, &mac_info,
                               sizeof(eth_dev_metadata_t), &actual);
  if (status != ZX_OK || actual != sizeof(eth_dev_metadata_t)) {
    zxlogf(ERROR, "bcm-ethernet: Could not get MAC metadata %d", status);
    return status;
  }

  zx_device_prop_t props[] = {
      {BIND_PLATFORM_DEV_VID, 0, mac_info.vid},
      {BIND_PLATFORM_DEV_DID, 0, mac_info.did},
  };
  printf("# vid = %d, did = %d\n", mac_info.vid, mac_info.did);

  return DdkAdd(ddk::DeviceAddArgs("bcm-ethernet").set_props(props));
}

void BcmEthernet::DdkUnbindNew(ddk::UnbindTxn txn) { txn.Reply(); }

void BcmEthernet::DdkRelease() { delete this; }

zx_status_t BcmEthernet::Create(void* ctx, zx_device_t* parent) {
  zxlogf(INFO, "bcm-ethernet: adding driver");
  fbl::AllocChecker ac;
  auto eth_device = fbl::make_unique_checked<BcmEthernet>(&ac, parent);
  if (!ac.check()) {
    return ZX_ERR_NO_MEMORY;
  }

  zx_status_t status = eth_device->InitPdev();
  if (status != ZX_OK) {
    zxlogf(ERROR, "bcm-ethernet: failed to init platform device");
    return status;
  }

  status = eth_device->Bind();
  if (status != ZX_OK) {
    zxlogf(ERROR, "bcm-ethernet driver failed to get added: %d", status);
    return status;
  } else {
    zxlogf(INFO, "bcm-ethernet driver added");
  }

  // eth_device intentionally leaked as it is now held by DevMgr
  __UNUSED auto ptr = eth_device.release();

  return ZX_OK;
}

static constexpr zx_driver_ops_t driver_ops = []() {
  zx_driver_ops_t ops = {};
  ops.version = DRIVER_OPS_VERSION;
  ops.bind = BcmEthernet::Create;
  return ops;
}();

}  // namespace eth

// clang-format off
ZIRCON_DRIVER_BEGIN(bcm_eth, eth::driver_ops, "bcm-ethernet", "0.1", 4)
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_COMPOSITE),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_BROADCOM),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_DID, PDEV_DID_BCM_ETH),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_PID, PDEV_PID_BCM54xx),
ZIRCON_DRIVER_END(bcm_eth)
