// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// ported by gobftald from Linux: include/linux/phy.h
//                                drivers/net/phy/broadcom.c
//                                drivers/net/phy/bcm-phy-lib.c

#include "bcm54210e.h"
#include "brcmphy.h"

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/platform-defs.h>
#include <fbl/alloc_checker.h>

static inline int phy_read(ddk::EthMacProtocolClient& eth_mac, uint32_t regnum) {
	uint32_t val;
	eth_mac.MdioRead(regnum, &val);
	return val;
}

static inline int phy_write(ddk::EthMacProtocolClient& eth_mac, uint32_t regnum, uint32_t val) {
  return eth_mac.MdioWrite(regnum, val);
}

static int bcm54xx_auxctl_read(ddk::EthMacProtocolClient& eth_mac, uint32_t regnum) {
        /* The register must be written to both the Shadow Register Select
         * and the Shadow Read Register Selector
         */
        phy_write(eth_mac, MII_BCM54XX_AUX_CTL, MII_BCM54XX_AUXCTL_SHDWSEL_MASK | 
                           regnum << MII_BCM54XX_AUXCTL_SHDWSEL_READ_SHIFT);
        return phy_read(eth_mac, MII_BCM54XX_AUX_CTL);
}

static int bcm54xx_auxctl_write(ddk::EthMacProtocolClient& eth_mac, uint32_t regnum, uint32_t val) {
        return phy_write(eth_mac, MII_BCM54XX_AUX_CTL, regnum | val);
}

static int bcm_phy_read_shadow(ddk::EthMacProtocolClient& eth_mac, uint32_t shadow) {
        phy_write(eth_mac, MII_BCM54XX_SHD, MII_BCM54XX_SHD_VAL(shadow));
        return MII_BCM54XX_SHD_DATA(phy_read(eth_mac, MII_BCM54XX_SHD));
}

static int bcm_phy_write_shadow(ddk::EthMacProtocolClient& eth_mac, uint32_t shadow, uint32_t val) {
        return phy_write(eth_mac, MII_BCM54XX_SHD,
                         MII_BCM54XX_SHD_WRITE |
                         MII_BCM54XX_SHD_VAL(shadow) |
                         MII_BCM54XX_SHD_DATA(val));
}

static int bcm_phy_write_exp(ddk::EthMacProtocolClient& eth_mac, uint32_t reg, uint32_t val)
{
        int rc;
        rc = phy_write(eth_mac, MII_BCM54XX_EXP_SEL, reg);
        if (rc < 0)
                return rc;
        return phy_write(eth_mac, MII_BCM54XX_EXP_DATA, val);
}

static int bcm54xx_config_clock_delay(ddk::EthMacProtocolClient& eth_mac) {
  int rc, val;

  // handling PHY's internal RX clock delay
  val = bcm54xx_auxctl_read(eth_mac, MII_BCM54XX_AUXCTL_SHDWSEL_MISC);
  val |= MII_BCM54XX_AUXCTL_MISC_WREN;
  val |= MII_BCM54XX_AUXCTL_SHDWSEL_MISC_RGMII_SKEW_EN;
  rc = bcm54xx_auxctl_write(eth_mac, MII_BCM54XX_AUXCTL_SHDWSEL_MISC, val);
	if (rc < 0)
		return rc;

  // handling PHY's internal TX clock delay
  val = bcm_phy_read_shadow(eth_mac, BCM54810_SHD_CLK_CTL);
  val &= ~BCM54810_SHD_CLK_CTL_GTXCLK_EN;
  rc = bcm_phy_write_shadow(eth_mac, BCM54810_SHD_CLK_CTL, val);
  if (rc < 0)
		return rc;

  return ZX_OK;
}

static int bcm54210e_config_init(ddk::EthMacProtocolClient& eth_mac) {
  return bcm54xx_config_clock_delay(eth_mac);
}

namespace phy {

zx_status_t PhyDevice::ConfigPhy(const uint8_t mac[MAC_ARRAY_LENGTH]) {
 	int err, reg, val;

  reg = phy_read(eth_mac_, MII_BCM54XX_ECR);
 	if (reg < 0)
		return reg;
	printf("# PhyDevice::ConfigPhy: reg = phy_read(eth_mac_, MII_BCM54XX_ECR) = 0x%x\n", reg);

  // Mask interrupts globally.
  reg |= MII_BCM54XX_ECR_IM;
  printf("# PhyDevice::ConfigPhy: reg |= MII_BCM54XX_ECR_IM = 0x%x\n", reg);
  printf("# PhyDevice::ConfigPhy: phy_write(eth_mac_, MII_BCM54XX_ECR, reg)\n");
  err = phy_write(eth_mac_, MII_BCM54XX_ECR, reg);
 	if (err < 0)
		return err;

  // Unmask events we are interested in.
  reg = ~(MII_BCM54XX_INT_DUPLEX | MII_BCM54XX_INT_SPEED | MII_BCM54XX_INT_LINK);
  printf("# PhyDevice::ConfigPhy: reg = ~(MII_BCM54XX_INT_DUPLEX | MII_BCM54XX_INT_SPEED | MII_BCM54XX_INT_LINK) = 0x%x\n", reg);
  printf("# PhyDevice::ConfigPhy: phy_write(eth_mac_, MII_BCM54XX_IMR, reg)\n");
 	err = phy_write(eth_mac_, MII_BCM54XX_IMR, reg);
	if (err < 0)
		return err;

	// Enable interrup
	reg = phy_read(eth_mac_, MII_BCM54XX_ECR);
	printf("# PhyDevice::ConfigPhy: reg = phy_read(eth_mac_, MII_BCM54XX_ECR) = 0x%x\n", reg);
  reg &= ~MII_BCM54XX_ECR_IM;
  printf("# PhyDevice::ConfigPhy: reg &= ~MII_BCM54XX_ECR_IM = 0x%x\n", reg);
  printf("# PhyDevice::ConfigPhy: phy_write(eth_mac_, MII_BCM54XX_ECR, reg)\n");
  err = phy_write(eth_mac_, MII_BCM54XX_ECR, reg);
  
  err = bcm54210e_config_init(eth_mac_);
	if (err)
		return err;
		
  val = BCM5482_SHD_LEDS1_LED1(BCM_LED_SRC_MULTICOLOR1) |
        BCM5482_SHD_LEDS1_LED3(BCM_LED_SRC_MULTICOLOR1);
  bcm_phy_write_shadow(eth_mac_, BCM5482_SHD_LEDS1, val);
  
  val = BCM_LED_MULTICOLOR_IN_PHASE |
        BCM5482_SHD_LEDS1_LED1(BCM_LED_MULTICOLOR_LINK_ACT) |
        BCM5482_SHD_LEDS1_LED3(BCM_LED_MULTICOLOR_LINK);
  bcm_phy_write_exp(eth_mac_, BCM_EXP_MULTICOLOR, val);

  return ZX_OK;
}

void PhyDevice::DdkUnbindNew(ddk::UnbindTxn txn) { txn.Reply(); }

void PhyDevice::DdkRelease() { delete this; }

zx_status_t PhyDevice::Create(void* ctx, zx_device_t* device) {
  fbl::AllocChecker ac;
  printf("# PhyDevice::Create: phy_device = fbl::make_unique_checked<PhyDevice>(&ac, device)\n");
  auto phy_device = fbl::make_unique_checked<PhyDevice>(&ac, device);
  if (!ac.check()) {
    return ZX_ERR_NO_MEMORY;
  }
  
  // Get ETH_MAC protocol.
  printf("# PhyDevice::Create: 'Get ETH_MAC protocol'\n");
  printf("# PhyDevice::Create: phy_device->eth_mac_.is_valid()\n");
  if (!phy_device->eth_mac_.is_valid()) {
    zxlogf(ERROR, "bcm54210e: could not obtain ETH_MAC protocol");
    return ZX_ERR_NO_RESOURCES;
  }  
  
  printf("# PhyDevice::Create: phy_device->DdkAdd(\"phy_bcm54210e\", DEVICE_ADD_NON_BINDABLE)\n");
  zx_status_t status = phy_device->DdkAdd("phy_bcm54210e", DEVICE_ADD_NON_BINDABLE);
  if (status != ZX_OK) {
    zxlogf(ERROR, "bcm54210e: Could not create phy device: %d", status);
    return status;
  }
  
  // devmgr now owns device.
  printf("# PhyDevice::Create: dev = phy_device.release()\n");
  auto* dev = phy_device.release();
  
  printf("# PhyDevice::Create: eth_mac_callbacks_t cb\n");
  eth_mac_callbacks_t cb;
  printf("# PhyDevice::Create: cb.config_phy = [](void* ctx, const uint8_t* mac)\n");
  cb.config_phy = [](void* ctx, const uint8_t* mac) {
    return static_cast<PhyDevice*>(ctx)->ConfigPhy(mac);
  };
  printf("# PhyDevice::Create: cb.ctx = dev\n");
  cb.ctx = dev;
  
  printf("# PhyDevice::Create: dev->eth_mac_.RegisterCallbacks(&cb)\n");
  dev->eth_mac_.RegisterCallbacks(&cb);
  return status;
}

static constexpr zx_driver_ops_t driver_ops = []() {
  zx_driver_ops_t ops = {};
  ops.version = DRIVER_OPS_VERSION;
  ops.bind = PhyDevice::Create;
  return ops;
}();

}  // namespace phy

ZIRCON_DRIVER_BEGIN(bcm54210e, phy::driver_ops, "bcm54210e_phy", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_BROADCOM),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_PID, PDEV_PID_BCM54210E),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_ETH_PHY),
ZIRCON_DRIVER_END(bcm54210e)
