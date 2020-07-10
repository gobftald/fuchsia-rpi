// Copyright 2020 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// for the time being it's only a template copy from vim3

#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/metadata.h>
#include <ddk/metadata/clock.h>
#include <ddk/platform-defs.h>
#include <ddk/protocol/platform/bus.h>
#include <soc/aml-a311d/a311d-hw.h>
#include <soc/aml-meson/g12b-clk.h>

#include "rpi4.h"

namespace rpi4 {
static const pbus_mmio_t clk_mmios[] = {
    {
        .base = A311D_HIU_BASE,
        .length = A311D_HIU_LENGTH,
    },
    {
        .base = A311D_DOS_BASE,
        .length = A311D_DOS_LENGTH,
    },
};

static const clock_id_t clock_ids[] = {
    {g12b_clk::G12B_CLK_SYS_PLL_DIV16},
    {g12b_clk::G12B_CLK_SYS_CPU_CLK_DIV16},
    {g12b_clk::G12B_CLK_SYS_PLLB_DIV16},
    {g12b_clk::G12B_CLK_SYS_CPUB_CLK_DIV16},
};

static const pbus_metadata_t clock_metadata[] = {
    {
        .type = DEVICE_METADATA_CLOCK_IDS,
        .data_buffer = &clock_ids,
        .data_size = sizeof(clock_ids),
    },
};

static pbus_dev_t clk_dev = []() {
  pbus_dev_t dev = {};
  dev.name = "rpi4-clk";
  dev.vid = PDEV_VID_AMLOGIC;
  dev.did = PDEV_DID_AMLOGIC_G12B_CLK;
  dev.mmio_list = clk_mmios;
  dev.mmio_count = countof(clk_mmios);
  dev.metadata_list = clock_metadata;
  dev.metadata_count = countof(clock_metadata);
  return dev;
}();

zx_status_t Rpi4::ClkInit() {
  zx_status_t status = pbus_.ProtocolDeviceAdd(ZX_PROTOCOL_CLOCK_IMPL, &clk_dev);
  if (status != ZX_OK) {
    zxlogf(ERROR, "ClkInit: DeviceAdd failed, st = %d", status);
    return status;
  }

  clk_impl_ = ddk::ClockImplProtocolClient(parent());
  if (!clk_impl_.is_valid()) {
    zxlogf(ERROR, "%s: ClockImplProtocolClient failed", __func__);
    return ZX_ERR_INTERNAL;
  }
  return ZX_OK;
}
}  // namespace rpi4
