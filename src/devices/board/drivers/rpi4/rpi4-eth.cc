// Copyright 2020 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// for the time being it's only a template copy from vim3

#include <limits.h>

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/metadata.h>
#include <ddk/platform-defs.h>
#include <ddk/protocol/ethernet.h>
#include <fbl/algorithm.h>
/*
#include <soc/aml-a311d/a311d-gpio.h>
#include <soc/aml-a311d/a311d-hw.h>
*/

#include "rpi4.h"
#include <soc/bcm2711/bcm2711-hw.h>

namespace rpi4 {

static const pbus_irq_t eth_mac_irqs[] = {
    {
        .irq = BCM2711_UMAC_IRQ_0,
        .mode = ZX_INTERRUPT_MODE_LEVEL_HIGH,
    },
    {
        .irq = BCM2711_UMAC_IRQ_1,
        .mode = ZX_INTERRUPT_MODE_LEVEL_HIGH,
    },
};

static const pbus_mmio_t eth_mac_mmios[] = {
    {
        .base = BCM2711_ETH_BASE,
        .length = BCM2711_ETH_LENGTH,
    },
};

static const pbus_bti_t eth_mac_btis[] = {
    {
        .iommu_index = 0,
        .bti_id = BTI_ETHERNET,
    },
};

static const pbus_boot_metadata_t eth_mac_metadata[] = {
    {
        .zbi_type = DEVICE_METADATA_MAC_ADDRESS,
        .zbi_extra = 0,
    },
};

static pbus_dev_t eth_dev = []() {
  pbus_dev_t dev = {};
  dev.name = "bcmgenet";
  dev.vid = PDEV_VID_BROADCOM;
  dev.did = PDEV_DID_BCM_UNIMAC;
  dev.mmio_list = eth_mac_mmios;
  dev.mmio_count = countof(eth_mac_mmios);
  dev.irq_list = eth_mac_irqs;
  dev.irq_count = countof(eth_mac_irqs);
  dev.bti_list = eth_mac_btis;
  dev.bti_count = countof(eth_mac_btis);
  dev.boot_metadata_list = eth_mac_metadata;
  dev.boot_metadata_count = countof(eth_mac_metadata);
  return dev;
}();

zx_status_t Rpi4::EthInit() {
  // setup pinmux for RGMII connections
  /*
  gpio_impl_.SetAltFunction(A311D_GPIOZ(0), A311D_GPIOZ_0_ETH_MDIO_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(1), A311D_GPIOZ_1_ETH_MDC_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(2), A311D_GPIOZ_2_ETH_RX_CLK_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(3), A311D_GPIOZ_3_ETH_RX_DV_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(4), A311D_GPIOZ_4_ETH_RXD0_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(5), A311D_GPIOZ_5_ETH_RXD1_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(6), A311D_GPIOZ_6_ETH_RXD2_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(7), A311D_GPIOZ_7_ETH_RXD3_FN);

  gpio_impl_.SetAltFunction(A311D_GPIOZ(8), A311D_GPIOZ_8_ETH_TX_CLK_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(9), A311D_GPIOZ_9_ETH_TX_EN_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(10), A311D_GPIOZ_10_ETH_TXD0_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(11), A311D_GPIOZ_11_ETH_TXD1_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(12), A311D_GPIOZ_12_ETH_TXD2_FN);
  gpio_impl_.SetAltFunction(A311D_GPIOZ(13), A311D_GPIOZ_13_ETH_TXD3_FN);
  */

  zx_status_t status = pbus_.DeviceAdd(&eth_dev);
  if (status != ZX_OK) {
    zxlogf(ERROR, "%s: DeviceAdd failed: %d", __func__, status);
    return status;
  }

  return ZX_OK;
}

}  // namespace rpi4
