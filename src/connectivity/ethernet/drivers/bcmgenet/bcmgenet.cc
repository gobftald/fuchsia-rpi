// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <lib/operation/ethernet.h>
#include <lib/zircon-internal/align.h>
#include <lib/zx/clock.h>
#include <sys/types.h>

#include <ddk/binding.h>
#include <ddk/metadata.h>
#include <ddk/platform-defs.h>

#include "bcmgenet.h"
#include "bcmgenet_.h"
#include "mdio-bcm-unimac.h"
#include "bcm-genet-dma.h"


/*
// Array of GENET hardware parameters/characteristics
static const struct bcmgenet_hw_params hw_params = {
		.tdma_offset = 0x4000,
		.words_per_bd = 3,
};


// Tx/Rx DMA register offset, skip 256 descriptors
#define DMA_DESC_SIZE	(hw_params.words_per_bd * sizeof(uint32_t))
#define GENET_TDMA_REG_OFF (hw_params.tdma_offset + TOTAL_DESC * DMA_DESC_SIZE)

// GENET v4 and v5 supports 40-bits pointer addressing
// for obvious reasons the LO and HI word parts are
// contiguous, but this offsets the other registers.
static const uint8_t genet_dma_ring_regs [] {
};


// GENET v3plus DMA regs
static const uint8_t bcmgenet_dma_regs [] {
};
*/

static inline uint32_t bcmgenet_rbuf_ctrl_get(std::optional<ddk::MmioBuffer>& mmio) {
	return bcmgenet_sys_readl(mmio, SYS_RBUF_FLUSH_CTRL);
}

static inline void bcmgenet_rbuf_ctrl_set(std::optional<ddk::MmioBuffer>& mmio, 
                                          uint32_t val) {
	bcmgenet_sys_writel(mmio, val, SYS_RBUF_FLUSH_CTRL);
}

static void umac_enable_set(std::optional<ddk::MmioBuffer>& mmio,
                            uint32_t mask, bool enable) {
  uint32_t reg = bcmgenet_umac_readl(mmio, UMAC_CMD);
  if (enable) {
    reg |= mask;
  } else {
    reg &= ~mask;
  }
  bcmgenet_umac_writel(mmio, reg, UMAC_CMD);
  
  // UniMAC stops on a packet boundary,
	// wait for full-size packet to be processed
  if (enable == 0) {
    zx::nanosleep(zx::deadline_after(zx::usec(1000)));
  }
}

static void bcmgenet_mii_setup(std::optional<ddk::MmioBuffer>& mmio) {
  uint32_t cmd_bits = UMAC_SPEED_100;
  cmd_bits <<= CMD_SPEED_SHIFT;
  uint32_t reg = bcmgenet_ext_readl(mmio, EXT_RGMII_OOB_CTRL);
  reg &= ~OOB_DISABLE;
  reg |= RGMII_LINK;
  bcmgenet_ext_writel(mmio, reg, EXT_RGMII_OOB_CTRL);
  
  reg = bcmgenet_umac_readl(mmio, UMAC_CMD);
  reg &= ~((CMD_SPEED_MASK << CMD_SPEED_SHIFT) |
           CMD_HD_EN |
           CMD_RX_PAUSE_IGNORE | CMD_TX_PAUSE_IGNORE);
  reg |= cmd_bits;
  bcmgenet_umac_writel(mmio, reg, UMAC_CMD);
}
static void bcmgenet_mii_config(std::optional<ddk::MmioBuffer>& mmio) {
  // Configure phy-mode to rgmii-rxid (trivially external)
  bcmgenet_sys_writel(mmio, PORT_MODE_EXT_GPHY, SYS_PORT_CTRL);
  
  // Enable the RGMII block for the interface to work
  uint32_t reg = bcmgenet_ext_readl(mmio, EXT_RGMII_OOB_CTRL);
  reg &= ~ID_MODE_DIS;  // enable the RX delay
  reg |= RGMII_MODE_EN;
  bcmgenet_ext_writel(mmio, reg, EXT_RGMII_OOB_CTRL);
}

static void bcmgenet_link_intr_enable(std::optional<ddk::MmioBuffer>& mmio) {
	uint32_t int0_enable = 0;
		
	// Monitor cable plug/unplugged event
	int0_enable |= UMAC_IRQ_LINK_EVENT;
	int0_enable |= (1 << 9);
	bcmgenet_intrl2_0_writel(mmio, int0_enable, INTRL2_CPU_MASK_CLEAR);
}

static void bcmgenet_intr_disable(std::optional<ddk::MmioBuffer>& mmio) {
	/* Mask all interrupts.*/
	bcmgenet_intrl2_0_writel(mmio, 0xFFFFFFFF, INTRL2_CPU_MASK_SET);
	bcmgenet_intrl2_0_writel(mmio, 0xFFFFFFFF, INTRL2_CPU_CLEAR);
	bcmgenet_intrl2_1_writel(mmio, 0xFFFFFFFF, INTRL2_CPU_MASK_SET);
	bcmgenet_intrl2_1_writel(mmio, 0xFFFFFFFF, INTRL2_CPU_CLEAR);
}

static void bcmgenet_umac_reset(std::optional<ddk::MmioBuffer>& mmio) {
	uint32_t reg = bcmgenet_rbuf_ctrl_get(mmio);
	reg |= BIT(1);
	bcmgenet_rbuf_ctrl_set(mmio, reg);
	zx::nanosleep(zx::deadline_after(zx::usec(10)));

	reg &= ~BIT(1);
	bcmgenet_rbuf_ctrl_set(mmio, reg);
	zx::nanosleep(zx::deadline_after(zx::usec(10)));
}

/*
static inline void bcmgenet_tdma_ring_write(std::optional<ddk::MmioBuffer>& mmio,
                                            unsigned int ring, uint32_t val,
                                            enum dma_ring_reg r) {
  	mmio->Write32(val, GENET_TDMA_REG_OFF +
			                 (DMA_RING_SIZE * ring) +
			                 genet_dma_ring_regs[r]);
}
*/

namespace eth {

void BCMUniMacDevice::UpdateLinkStatus() {
  //bool temp = mmio_->ReadMasked32(GMAC_RGMII_STATUS_LNKSTS, DW_MAC_MAC_RGMIISTATUS);

  //if (temp != online_) {
  //  online_ = temp;
  //  if (ethernet_client_.is_valid()) {
  //    ethernet_client_.Status(online_ ? ETHERNET_STATUS_ONLINE : 0u);
  //  } else {
  //    zxlogf(ERROR, "BCMUniMacDevice: System not ready");
  //  }
  //}
  //if (online_) {
  //  mmio_->SetBits32((GMAC_CONF_TE | GMAC_CONF_RE), DW_MAC_MAC_CONF);
  //} else {
  //  mmio_->ClearBits32((GMAC_CONF_TE | GMAC_CONF_RE), DW_MAC_MAC_CONF);
  //}
  //zxlogf(INFO, "BCMUniMacDevice: Link is now %s", online_ ? "up" : "down");
}

void BCMUniMacDevice::EthernetImplGetBti(zx::bti* bti) { bti_.duplicate(ZX_RIGHT_SAME_RIGHTS, bti); }

zx_status_t BCMUniMacDevice::EthernetImplSetParam(uint32_t param, int32_t value, const void* data,
                                              size_t data_size) {
  zxlogf(INFO, "%s: SetParam called  %x  %x", __func__, param, value);
  return ZX_OK;
}

zx_status_t BCMUniMacDevice::EthernetImplQuery(uint32_t options, ethernet_info_t* info) {
  if (options)
    return ZX_ERR_INVALID_ARGS;

  memset(info, 0, sizeof(*info));
  info->features = ETHERNET_FEATURE_DMA;
  info->mtu = 1500;
  memcpy(info->mac, mac_, sizeof info->mac);
  info->netbuf_size = eth::BorrowedOperation<>::OperationSize(sizeof(ethernet_netbuf_t));

  return ZX_OK;
}

void BCMUniMacDevice::EthernetImplStop() {
}

void BCMUniMacDevice::EthernetImplQueueTx(uint32_t options, ethernet_netbuf_t* netbuf,
                            ethernet_impl_queue_tx_callback completion_cb, void* cookie) {
}

zx_status_t BCMUniMacDevice::EthernetImplStart(const ethernet_ifc_protocol_t* ifc) {
  fbl::AutoLock lock(&lock_);
  
  if (ethernet_client_.is_valid()) {
    zxlogf(ERROR, "BCMUniMacDevice:  Already bound!!!");
    return ZX_ERR_ALREADY_BOUND;
  } else {
    ethernet_client_ = ddk::EthernetIfcProtocolClient(ifc);
    UpdateLinkStatus();
    zxlogf(INFO, "BCMUniMacDevice: Started");
  } 
  return ZX_OK;
}

zx_status_t BCMUniMacDevice::EthMacRegisterCallbacks(const eth_mac_callbacks_t* cbs) {
  if (cbs == nullptr) {
    return ZX_ERR_INVALID_ARGS;
  }

  cbs_ = *cbs;

  sync_completion_signal(&cb_registered_signal_);
  return ZX_OK;
}

zx_status_t BCMUniMacDevice::EthMacMdioWrite(uint32_t reg, uint32_t val) {
  return unimac_mdio_write(mdio_, phy_addr_, reg, val);
}

zx_status_t BCMUniMacDevice::EthMacMdioRead(uint32_t reg, uint32_t* val) {
  int ret = unimac_mdio_read(mdio_, phy_addr_, reg);
  
  *val = ret;
  
  if (ret < 0)
    return ret;
  else
    return ZX_OK;
}

zx_status_t BCMUniMacDevice::PhyModify(uint32_t regnum, uint32_t mask, uint32_t set) {
  int ret = unimac_mdio_read(mdio_, phy_addr_, regnum);
  if (ret < 0 )
    return ret;
  
  int val = (ret & ~mask) | set;
  if (val == ret)
    return ZX_OK;
  
  ret = unimac_mdio_write(mdio_, phy_addr_, regnum, val);
  return ret < 0 ? ret : true;
}

zx_status_t BCMUniMacDevice::GenphyRestartAneg() {
  return PhyModify(MII_BMCR, BMCR_ISOLATE, BMCR_ANENABLE | BMCR_ANRESTART);
}

void BCMUniMacDevice::DdkRelease() {
  zxlogf(INFO, "Ethernet release...");
  delete this;
}

zx_status_t BCMUniMacDevice::ShutDown() {
  return ZX_OK;
}

void BCMUniMacDevice::DdkUnbindNew(ddk::UnbindTxn txn) {
  zxlogf(INFO, "Ethernet DdkUnbind");
  ShutDown();
  txn.Reply();
}

BCMUniMacDevice::BCMUniMacDevice(zx_device_t* device, pdev_protocol_t* pdev)
    : ddk::Device<BCMUniMacDevice, ddk::UnbindableNew>(device), pdev_(pdev) {}

int BCMUniMacDevice::Thread() {
  zxlogf(INFO, "%s: mac-thread started", __func__);
  while (true) {
    /*
    zx::nanosleep(zx::deadline_after(zx::msec(1000)));
    // Read irq status
    printf("* Read irq status\n");
    uint32_t v1, v2, status;
    v1 = bcmgenet_intrl2_0_readl(mmio_, INTRL2_CPU_STAT);
    v2 = bcmgenet_intrl2_0_readl(mmio_, INTRL2_CPU_MASK_STATUS);
    // Clear interrupts
    status = v1 & ~v2;
    bcmgenet_intrl2_0_writel(mmio_, status, INTRL2_CPU_CLEAR);
    */
    zx_port_packet_t packet;
    zx_status_t status = port_.wait(zx::time::infinite(), &packet);
    printf("get out port_.wait\n");
    if (status != ZX_OK) {
      zxlogf(ERROR, "%s: port wait failed: %d", __func__, status);
      return thrd_error;
    }
    if (packet.key > dma_irqs_.size()) {
      zxlogf(WARNING, "%s: received interrupt from invalid port", __func__);
      continue;
    }  
    if (packet.key == dma_irqs_.size()) {
      zxlogf(INFO, "%s thread terminating", __func__);
      return thrd_success;
    }
    if (packet.key == 0) {
      printf("# dma_irq_0 asserted\n");
    } else if (packet.key == 1) {
      printf("# dma_irq_1 asserted\n");
    } else {
      zxlogf(WARNING, "%s: received interrupt from invalid port", __func__);
    }
    dma_irqs_[packet.key].ack();
  }
  return ZX_OK;
}

zx_status_t BCMUniMacDevice::InitDevice() {
  // Create port.
  zx_status_t status = zx::port::create(ZX_PORT_BIND_TO_INTERRUPT, &port_);
  if (status != ZX_OK) {
    zxlogf(ERROR, "%s: zx_port_create failed %d", __func__, status);
    return status;
  }
  
  // Bind interrupts.
  uint32_t port_key = 0;
  for (const zx::interrupt& dma_irq : dma_irqs_) {
    status = dma_irq.bind(port_, port_key++, ZX_INTERRUPT_BIND);
    if (status != ZX_OK) {
      zxlogf(ERROR, "%s: zx_interrupt_bind failed %d", __func__, status);
      return status;
    }
  }
  // Config RGMII interface
  printf("# BCMUniMacDevice::InitDevice: bcmgenet_mii_config(mmio_)\n");
  bcmgenet_mii_config(mmio_);

  printf("# BCMUniMacDevice::InitDevice: umac_enable_set(priv, CMD_TX_EN | CMD_RX_EN, true)\n");
  umac_enable_set(mmio_, CMD_TX_EN | CMD_RX_EN | (1 << 9), true);
  
  // Enable monitoring link interrupts
  printf("# BCMUniMacDevice::InitDevice: bcmgenet_link_intr_enable(mmio_)\n");
  bcmgenet_link_intr_enable(mmio_);
  
  // Enable and Restart Autonegotiation
  printf("# GenphyRestartAneg()\n");
  GenphyRestartAneg();
  
  // setup link state and update UMAC and RGMII block
  printf("# bcmgenet_mii_setup(mmio_)\n");
  bcmgenet_mii_setup(mmio_);

  return ZX_OK;
}

int BCMUniMacDevice::WorkerThread() {
  // waiting for PHY to register its callbacks before proceeding further.
  sync_completion_wait(&cb_registered_signal_, ZX_TIME_INFINITE);
  
 	// take MAC out of reset.
	printf("# BCMUniMacDevice::InitDevice: bcmgenet_umac_reset(priv)\n");
	bcmgenet_umac_reset(mmio_);

  // Reset/disable all interrupts
  printf("# BCMUniMacDevice::InitDevice: bcmgenet_intr_disable(mmio_)\n");
  bcmgenet_intr_disable(mmio_);

  // Configure the phy.
  printf("# %s: cbs_.config_phy(cbs_.ctx, mac_)\n", __func__);
  cbs_.config_phy(cbs_.ctx, mac_);
  
  printf("# %s: InitDevice()\n", __func__);
  InitDevice();

  auto thunk = [](void* arg) -> int { return reinterpret_cast<BCMUniMacDevice*>(arg)->Thread(); };
  int ret = thrd_create_with_name(&thread_, thunk, this, "UniMAC_thread");
  ZX_DEBUG_ASSERT(ret == thrd_success);

  zx_status_t status = DdkAdd("Broadcom_UniMAC");
  if (status != ZX_OK) {
    zxlogf(ERROR, "%s: Could not create eth device: %d", __func__, status);
    return status;
  } else {
    zxlogf(INFO, "%s: Added Broadcom_UniMAC device", __func__);
  }
  return status;
}

zx_status_t BCMUniMacDevice::InitBuffers() {
  constexpr size_t kDescSize = ZX_ROUNDUP(2 * kNumDesc * sizeof(bcm_dmadescr_t), PAGE_SIZE);
  printf("# %s: kDescSize = 0x%x (%d)\n", __func__, (uint32_t)kDescSize, (uint32_t)kDescSize);
  constexpr size_t kBufSize = 2 * kNumDesc * kTxnBufSize;
  printf("# %s: kBufSize = 0x%x (%d)\n", __func__, (uint32_t)kBufSize, (uint32_t)kBufSize);

  printf("# %s: desc_buffer_ = PinnedBuffer::Create(kDescSize, bti_, ZX_CACHE_POLICY_UNCACHED)\n", __func__);
  desc_buffer_ = PinnedBuffer::Create(kDescSize, bti_, ZX_CACHE_POLICY_UNCACHED);
  printf("# %s: txn_buffer_ = PinnedBuffer::Create(kBufSize, bti_, ZX_CACHE_POLICY_CACHED)\n", __func__);
  txn_buffer_ = PinnedBuffer::Create(kBufSize, bti_, ZX_CACHE_POLICY_CACHED);

  tx_buffer_ = static_cast<uint8_t*>(txn_buffer_->GetBaseAddress());
  printf("# %s: tx_buffer_ = 0x%lx\n", __func__, (uint64_t)tx_buffer_);
  zx_cache_flush(tx_buffer_, kBufSize, ZX_CACHE_FLUSH_DATA | ZX_CACHE_FLUSH_INVALIDATE);
  // rx buffer right after tx
  rx_buffer_ = &tx_buffer_[kBufSize / 2];
  printf("# %s: rx_buffer_ = 0x%lx\n", __func__, (uint64_t)rx_buffer_);

  tx_descriptors_ = static_cast<bcm_dmadescr_t*>(desc_buffer_->GetBaseAddress());
  printf("# %s: tx_descriptors_ = 0x%lx\n", __func__, (uint64_t)tx_descriptors_);
  // rx descriptors right after tx
  rx_descriptors_ = &tx_descriptors_[kNumDesc];
  printf("# %s: rx_descriptors_ = 0x%lx\n", __func__, (uint64_t)rx_descriptors_);

  zx_paddr_t tmpaddr;

  // Initialize descriptors. Doing tx and rx all at once
  printf("# %s: 'Initialize descriptors'\n", __func__);
  for (uint i = 0; i < kNumDesc; i++) {
    desc_buffer_->LookupPhys(((i + 1) % kNumDesc) * sizeof(bcm_dmadescr_t), &tmpaddr);
    tx_descriptors_[i].dmamac_next = static_cast<uint32_t>(tmpaddr);

    txn_buffer_->LookupPhys(i * kTxnBufSize, &tmpaddr);
    tx_descriptors_[i].dmamac_addr = static_cast<uint32_t>(tmpaddr);
    tx_descriptors_[i].txrx_status = 0;
    tx_descriptors_[i].dmamac_cntl = DESC_TXCTRL_TXCHAIN;

    desc_buffer_->LookupPhys((((i + 1) % kNumDesc) + kNumDesc) * sizeof(bcm_dmadescr_t), &tmpaddr);
    rx_descriptors_[i].dmamac_next = static_cast<uint32_t>(tmpaddr);

    txn_buffer_->LookupPhys((i + kNumDesc) * kTxnBufSize, &tmpaddr);
    rx_descriptors_[i].dmamac_addr = static_cast<uint32_t>(tmpaddr);
    rx_descriptors_[i].dmamac_cntl =
        (MAC_MAX_FRAME_SZ & DESC_RXCTRL_SIZE1MASK) | DESC_RXCTRL_RXCHAIN;

    rx_descriptors_[i].txrx_status = DESC_RXSTS_OWNBYDMA;
  }

  return ZX_OK;
}

zx_status_t BCMUniMacDevice::GetPhyId() {
  // base address of mdio registers
  mdio_ = mmio_->View(GENET_UMAC_OFF + UMAC_MDIO_CMD);

  // search for a responding phy (address) on mdio bus
  for (phy_addr_ = 0;
       unimac_mdio_read(mdio_, phy_addr_, MII_PHYSID1) == 0xffff &&
       phy_addr_ < PHY_MAX_ADDR;
       phy_addr_++) {
    if (phy_addr_ == PHY_MAX_ADDR) {
      zxlogf(ERROR, "%s: Could not find any phy device", __func__);
      return ZX_ERR_NOT_FOUND;
    }
  }
 
  // get phy's hw id for binding device driver                 
  uint32_t phy_reg = unimac_mdio_read(mdio_, phy_addr_, MII_PHYSID1);
  phy_id_ = phy_reg << 16;
  phy_reg = unimac_mdio_read(mdio_, phy_addr_, MII_PHYSID2);
  phy_id_ |= phy_reg;
  
  return ZX_OK;
}
 
zx_status_t BCMUniMacDevice::CheckMACVersion() {
  uint32_t reg;
  uint8_t version;
    
  // Read GENET HW version
  reg = bcmgenet_sys_readl(mmio_, SYS_REV_CTRL);
  version = (reg >> 24 & 0x0f);
  if (version == 6)
		version = 5;
	
	// check for supported version
	if (version != GENET_V5) {
	  zxlogf(ERROR, "%s: GENET version mismatch, got: %d, configured for: %d",
	         __func__, version, GENET_V5);
	  return ZX_ERR_NOT_SUPPORTED;
	}
  printf("# %s: version = %d\n", __func__, version);
  return ZX_OK;
}
  
zx_status_t BCMUniMacDevice::GetMACAddress(zx_device_t* dev) {
  // look for MAC address device metadata
  // metadata is padded so we need buffer size > 6 bytes
  uint8_t buffer[16];
  size_t actual;
  
  printf("# %s: device_get_metadata(dev, DEVICE_METADATA_MAC_ADDRESS, buffer, sizeof(buffer), &actual)\n", __func__);
  zx_status_t status =
      device_get_metadata(dev, DEVICE_METADATA_MAC_ADDRESS, buffer, sizeof(buffer), &actual);
  if (status != ZX_OK || actual < 6) {
    zxlogf(ERROR, "%s: MAC address metadata load failed. Falling back on HW setting.", __func__);
    // read MAC address from hardware register
    //uint32_t hi = mmio_->Read32(0x00C);
    //uint32_t lo = mmio_->Read32(0x010);
    // in case of rpi4 we need either to read it somehow from fdt
    // or develop a mbox/vchi driver to get it from te video core system

    /* Extract the MAC address from the high and low words */
    //buffer[0] = static_cast<uint8_t>(lo & 0xff);
    //buffer[1] = static_cast<uint8_t>((lo >> 8) & 0xff);
    //buffer[2] = static_cast<uint8_t>((lo >> 16) & 0xff);
    //buffer[3] = static_cast<uint8_t>((lo >> 24) & 0xff);
    //buffer[4] = static_cast<uint8_t>(hi & 0xff);
    //buffer[5] = static_cast<uint8_t>((hi >> 8) & 0xff);
  }

  zxlogf(INFO, "%s: actual = %d, MAC address %02x:%02x:%02x:%02x:%02x:%02x", __func__,
         (int)actual, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
  memcpy(mac_, buffer, sizeof mac_);
  return ZX_OK;
}

zx_status_t BCMUniMacDevice::InitPdev() {
  // Map mac control registers and dma control registers. 
  auto status = pdev_.MapMmio(ETH_MAC_MMIO, &mmio_);
  if (status != ZX_OK) {
    zxlogf(ERROR, "%s: could not map mmio: %d", __func__, status);
    return status;
  }
  
  // Get and map DMA interrupts.
  pdev_device_info_t info;
  if ((status = pdev_.GetDeviceInfo(&info)) != ZX_OK) {
    zxlogf(ERROR, "%s: GetDeviceInfo failed: %d", __func__, status);
    return status;
  }
  if (info.irq_count != ETH_MAC_IRQ_NUM) {
    zxlogf(ERROR, "%s: number of IRQ must be %d", __func__, ETH_MAC_IRQ_NUM);
    return ZX_ERR_OUT_OF_RANGE;
  }
  fbl::AllocChecker ac;
  fbl::Array<zx::interrupt>
      dma_irqs_(new (&ac) zx::interrupt[info.irq_count], info.irq_count);
  if (!ac.check()) {
    zxlogf(ERROR, "%s: port interrupts alloc failed", __func__);
    return ZX_ERR_NO_RESOURCES;
  }
  for (uint32_t i = 0; i < dma_irqs_.size(); i++) {
    zx::interrupt interrupt;  
    if ((status = pdev_.GetInterrupt(i, &interrupt)) != ZX_OK) {
      zxlogf(ERROR, "%s: GetInterrupt failed: %d", __func__, status);
      return status;
    }
    dma_irqs_[i] = std::move(interrupt);
  }
  
  // Get our bti.
  if ((status = pdev_.GetBti(0, &bti_)) != ZX_OK) {
    zxlogf(ERROR, "%s: could not obtain bti: %d", __func__, status);
    return status;
  }
  return status;
}

zx_status_t BCMUniMacDevice::Create(void* ctx, zx_device_t* device) {
  pdev_protocol_t pdev;
  auto status = device_get_protocol(device, ZX_PROTOCOL_PDEV, &pdev);
  if (status != ZX_OK) {
    zxlogf(ERROR, "%s could not get ZX_PROTOCOL_PDEV", __func__);
    return status;
  }
  auto mac_device = std::make_unique<BCMUniMacDevice>(device, &pdev);

  status = mac_device->InitPdev();
  if (status != ZX_OK) {
    return status;
  }
  
  // Get and cache the MAC address.
  status = mac_device->GetMACAddress(device);
  if (status != ZX_OK) {
    return status;
  }  
  
  // Check UniMac device version
  // this MAC driver supports only GENET_V5
  status = mac_device->CheckMACVersion();
  if (status != ZX_OK) {
    return status;
  }  
  status = mac_device->InitBuffers();
  if (status != ZX_OK)
    return status;

  // Scan for phy device and get its hw ID
  // for binding appropriate device driver  
  status = mac_device->GetPhyId();
  if (status != ZX_OK) {
    return status;
  }
  
  zx_device_prop_t props[] = {
      {BIND_PLATFORM_DEV_VID, 0, PDEV_VID_BROADCOM},
      {BIND_PLATFORM_DEV_PID, 0, mac_device->phy_id_},
      {BIND_PLATFORM_DEV_DID, 0, PDEV_DID_ETH_PHY},
  };

  device_add_args_t phy_device_args = {};
  phy_device_args.version = DEVICE_ADD_ARGS_VERSION;
  phy_device_args.name = "ethernet_phy";
  phy_device_args.ops = &mac_device->ddk_device_proto_,
  phy_device_args.proto_id = ZX_PROTOCOL_ETH_MAC;
  phy_device_args.props = props;
  phy_device_args.prop_count = countof(props);
  phy_device_args.ctx = mac_device.get();
  phy_device_args.proto_ops = &mac_device->eth_mac_protocol_ops_;

  zx_device_t* dev;
  status = device_add(device, &phy_device_args, &dev);
  if (status != ZX_OK) {
    zxlogf(ERROR, "%s: Could not create phy device: %d", __func__, status);
    return status;
  }

  auto worker_thunk = [](void* arg) -> int {
    return reinterpret_cast<BCMUniMacDevice*>(arg)->WorkerThread();
  };

  int ret = thrd_create_with_name(&mac_device->worker_thread_, worker_thunk,
                                  reinterpret_cast<void*>(mac_device.get()), "mac_worker_thread");
  ZX_DEBUG_ASSERT(ret == thrd_success);

  __UNUSED auto ptr = mac_device.release();
  return ZX_OK;
}

static constexpr zx_driver_ops_t driver_ops = []() {
  zx_driver_ops_t ops = {};
  ops.version = DRIVER_OPS_VERSION;
  ops.bind = BCMUniMacDevice::Create;
  return ops;
}();

}  // namespace eth

ZIRCON_DRIVER_BEGIN(bcmgenet, eth::driver_ops, "broadcom_unimac", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_PDEV),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_BROADCOM),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_BCM_UNIMAC),
ZIRCON_DRIVER_END(bcmgenet)
