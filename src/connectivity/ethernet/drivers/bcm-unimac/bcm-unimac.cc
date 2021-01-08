// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// ported by gobftald from Linux: drivers/net/ethernet/broadcom/genet/bcmgenet.c
//                                drivers/net/ethernet/broadcom/genet/bcmmii.c

#include "bcm-unimac.h"

#include <lib/operation/ethernet.h>

#include <ddk/binding.h>
#include <ddk/metadata.h>
#include <ddk/platform-defs.h>

bool io_debug = 0;

namespace eth {

int BcmUniMAC::WorkerThread() {
  // Wait for zircon PHY device - added previously - to be bound and ready for configuration
  sync_completion_wait(&phy_signal_, ZX_TIME_INFINITE);
  sync_completion_reset(&phy_signal_);

  // Configure zircon PHY device by calling its callback config function
  phy_cbs_.config_phy(phy_cbs_.ctx, mac_addr_);

  // Set external PHY and appropriate RGMII mode
  ConfigMii();

  // Initialize PHY virtual device
  zx_status_t status = phy_->Init(SetupMii);
  if (status != ZX_OK) {
    zxlogf(ERROR, "BcmUniMAC: Could not initialize PHY device: %d", status);
    return thrd_error;
  }

  // Waiting until the Link is Up for the first time,
  // Adding UniMAC only after this event delays ethernet driver saving memory usage of both
  sync_completion_wait(&phy_signal_, ZX_TIME_INFINITE);

  // Take MAC out of reset.
  ResetUmac();

  // Clear Rx/Tx counters, init Rx registers
  InitUmac();

  // Set HW (MAC) address
  SetHWAddr(mac_addr_);

  // Disable Rx/Tx DMA and flush Tx queues
  // returns a reusable dma control register value
  auto dma_ctrl = DisableDMA();

  // Initialize RDMA and TDMA control registers
  InitDMA();

  // Enable Rx/Tx DMA for default queue (16)
  EnableDMA(dma_ctrl);

  // Request and bind interrupts
  status = RequestIrqs();
  if (status  != ZX_OK) {
    return thrd_error;
  }

  // Launch IRQ handling thread
  auto thunk = [](void* arg) -> int { return reinterpret_cast<BcmUniMAC*>(arg)->IRQThread(); };
  int ret = thrd_create_with_name(&thread_, thunk, this, "UniMAC_IRQ_thread");
  ZX_DEBUG_ASSERT(ret == thrd_success);

  // Set IPv6 address filters
  SetRxMode();

  // Enable Rx interrupts
  intrl2_0_writel(mmio_, UMAC_IRQ_RXDMA_DONE, INTRL2_CPU_MASK_CLEAR);

  // Enable Rx/Tx operation
  EnableUmac(CMD_TX_EN | CMD_RX_EN, true);

  // Enable Tx interrupts
  intrl2_0_writel(mmio_, UMAC_IRQ_TXDMA_DONE, INTRL2_CPU_MASK_CLEAR);

  // EOF MAC initialisation
  is_mac_ready_ = true;

  if ((status = DdkAdd("Broadcom_UniMAC")) != ZX_OK) {
    zxlogf(ERROR, "BcmUniMAC: Could not create Ethernet (MAC) device: %d", status);
    return thrd_error;
  } else {
    zxlogf(INFO, "BcmUniMAC: Adding Broadcom_UniMAC device");
  }
  return thrd_success;
}

zx_status_t BcmUniMAC::Create(void* ctx, zx_device_t* device) {
  pdev_protocol_t pdev;
  auto status = device_get_protocol(device, ZX_PROTOCOL_PDEV, &pdev);
  if (status != ZX_OK) {
    zxlogf(ERROR, "BcmUniMAC: could not get ZX_PROTOCOL_PDEV");
    return status;
  }
  auto mac_device = std::make_unique<BcmUniMAC>(device, &pdev);

  // Init platfrom device - MMIO, IRQs, BTI
  status = mac_device->InitPdev();
  if (status != ZX_OK) {
    return status;
  }

  // Get MAC address either from boot config or the video core system (mbox/vchi driver)
  status =  mac_device->GetMACAddress(device);
  if (status != ZX_OK) {
    return status;
  }

  // Currently only GENET_V5 supported, so only check HW version.
  status = mac_device->CheckMACVersion();
  if (status != ZX_OK) {
    return status;
  }

  sync_completion_reset(&mac_device->phy_signal_);

  // Get a 'General PHY' 'virtual device' represeting PHY's state
  // and implementing its control/operations via MDIO bus
  // Bus controller registers start at UniMAC offset + MDIO_CMD
  status = GenPhy::Create(mac_device->mmio_->View(GENET_UMAC_OFF + UMAC_MDIO_CMD),
                          &mac_device->phy_);
  if (status != ZX_OK) {
    return status;
  }

  // Create Zircon PHY device based on PHY's HW id 
  zx_device_prop_t props[] = {
      {BIND_PLATFORM_DEV_VID, 0, PDEV_VID_BROADCOM},
      {BIND_PLATFORM_DEV_PID, 0, mac_device->phy_->GetId()},
                                 // Use PHY's HW is read from the PHY device registers
      {BIND_PLATFORM_DEV_DID, 0, PDEV_DID_BCM_PHY},
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
  if ((status = device_add(device, &phy_device_args, &dev)) != ZX_OK) {
    zxlogf(ERROR, "BcmUniMAC: Could not create phy device: %d", status);
    return status;
  }

  // Create a worker thread to continue setup and initialization
  // but this thread finishes 'bind' returning OK to DevMgr.
  auto worker_thunk = [](void* arg) -> int {
    return reinterpret_cast<BcmUniMAC*>(arg)->WorkerThread();
  };
  int ret = thrd_create_with_name(&mac_device->worker_thread_, worker_thunk,
                                  reinterpret_cast<void*>(mac_device.get()), "unimac-worker-thread");
  ZX_DEBUG_ASSERT(ret == thrd_success);

  // mac_device is now held by DevMgr.
  __UNUSED auto ptr = mac_device.release();

  return ZX_OK;
}

void BcmUniMAC::DdkUnbindNew(ddk::UnbindTxn txn) {
  zxlogf(INFO, "BcmUniMAC::DdkUnbindNew");
  ShutDown();
  txn.Reply();
}

zx_status_t BcmUniMAC::ShutDown() {
  return ZX_OK;
}

void BcmUniMAC::DdkRelease() {
  zxlogf(INFO, "BcmUniMAC: Ethernet release...");
  delete this;
}

zx_status_t BcmUniMAC::EthernetImplStart(const ethernet_ifc_protocol_t* ifc) {
  fbl::AutoLock lock(&lock_);

  if (ethernet_client_.is_valid()) {
    zxlogf(ERROR, "BcmUniMAC: Already bound!!!");
    return ZX_ERR_ALREADY_BOUND;
  } else {
    ethernet_client_ = ddk::EthernetIfcProtocolClient(ifc);
    UpdateLinkStatus();
    zxlogf(INFO, "BcmUniMAC: Started");
  }
  return ZX_OK;
}

void BcmUniMAC::EthernetImplStop() {
  zxlogf(INFO, "BcmUniMAC: Stopping");
  fbl::AutoLock lock(&lock_);
  ethernet_client_.clear();
}

zx_status_t BcmUniMAC::EthernetImplQuery(uint32_t options, ethernet_info_t* info) {
  if (options) {
    return ZX_ERR_INVALID_ARGS;
  }

  memset(info, 0, sizeof(*info));
  info->features = ETHERNET_FEATURE_DMA;
  info->mtu = ETH_DATA_LEN;
  memcpy(info->mac, mac_addr_, sizeof info->mac);
  info->netbuf_size = eth::BorrowedOperation<>::OperationSize(sizeof(ethernet_netbuf_t));

  return ZX_OK;
}

zx_status_t BcmUniMAC::EthMacRegisterCallbacks(const eth_mac_callbacks_t* cbs) {
  if (cbs == nullptr) {
    return ZX_ERR_INVALID_ARGS;
  }
  phy_cbs_ = *cbs;

  sync_completion_signal(&phy_signal_);
  return ZX_OK;
}

zx_status_t BcmUniMAC::EthMacMdioWrite(uint32_t reg, uint32_t val) {
  return phy_->Write(reg, val);
}

zx_status_t BcmUniMAC::EthMacMdioRead(uint32_t reg, uint32_t* val) {
  int ret = phy_->Read(reg);

  *val = ret;

  if (ret < 0) {
    return ret;
  } else {
    return ZX_OK;
  }
}

zx_status_t BcmUniMAC::InitPdev() {
  // Map the full MMIO range - MAC, MDIO and DMA control registers.
  auto status = pdev_.MapMmio(GENET_MMIO, &mmio_);
  if (status != ZX_OK) {
    zxlogf(ERROR, "BcmUniMAC: could not map MMIO: %d", status);
    return status;
  }

  // Get interrupts
  pdev_device_info_t info;
  status = pdev_.GetDeviceInfo(&info);
  if (status != ZX_OK) {
    zxlogf(ERROR, "BcmUniMAC: GetDeviceInfo failed: %d", status);
    return status;
  }
  if (info.irq_count != GENET_IRQ_NUM) {
    zxlogf(ERROR, "BcmUniMAC: number of IRQ must be %d", GENET_IRQ_NUM);
    return ZX_ERR_OUT_OF_RANGE;
  }
  fbl::AllocChecker ac;
  fbl::Array<zx::interrupt>
      irqs(new (&ac) zx::interrupt[info.irq_count], info.irq_count);
  if (!ac.check()) {
    zxlogf(ERROR, "BcmUniMAC: interrupts allocation failed");
    return ZX_ERR_NO_RESOURCES;
  }
  for (uint32_t i = 0; i < irqs.size(); i++) {
    zx::interrupt interrupt;
    status = pdev_.GetInterrupt(i, &interrupt); 
    if (status != ZX_OK) {
      zxlogf(ERROR, "BcmUniMAC: GetInterrupt failed: %d", status);
      return status;
    }
    irqs[i] = std::move(interrupt);
  }
  mac_irqs_ = std::move(irqs);

  // Get Bti
  if ((status = pdev_.GetBti(0, &bti_)) != ZX_OK) {
    zxlogf(ERROR, "BcmUniMAC: could not obtain 'BTI': %d", status);
    return status;
  }

  return ZX_OK;
}

zx_status_t BcmUniMAC::GetMACAddress(zx_device_t* dev) {
  // Search for MAC address device metadata
  // metadata is padded so we need buffer size > 6 bytes
  uint8_t buffer[16];
  size_t actual;

  zx_status_t status =
      device_get_metadata(dev, DEVICE_METADATA_MAC_ADDRESS,
                          buffer, sizeof(buffer), &actual);
  if (status != ZX_OK || actual < MAC_ARRAY_LENGTH) {
    zxlogf(ERROR, "BcmUniMAC: Metadata load failed. Falling back on HW setting.");
  }

  zxlogf(INFO, "BcmUniMAC: MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
         buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
  memcpy(mac_addr_, buffer, sizeof mac_addr_);

  return ZX_OK;
}

zx_status_t BcmUniMAC::CheckMACVersion() {
  // Read GENET HW version
  uint32_t reg = sys_readl(mmio_, SYS_REV_CTRL);
  uint8_t version = (reg >> 24 & 0x0f);
  if (version == 6) {
    version = 5;
  }

  // Check for supported version
  if (version != GENET_V5) {
    zxlogf(ERROR, "BcmUniMAC: HW version mismatch, got: %d, supported: %d", version, GENET_V5);
    return ZX_ERR_NOT_SUPPORTED;
  }
  zxlogf(INFO, "BcmUniMAC: HW version = %d\n", version);
  return ZX_OK;
}

bool BcmUniMAC::SetupMii(uint32_t cmd_bits) {
  uint32_t reg;

  if (!mac_instance_->is_mac_ready_) {
    if (cmd_bits) {
      // Signal Link online status for the first time,
      // it triggers UniMAC final init and adding as device
      sync_completion_signal(&phy_signal_);
    }
    return mac_instance_->is_mac_ready_;    
  }

  if (cmd_bits) {
    if (!mac_instance_->online_) {
      // Program UMAC and RGMII block based on established link speed and duplex.
      // The speed set in UMAC_CMD tell RGMII block which clock
      // to use for transmit -- 25MHz(100Mbps) or 125MHz(1Gbps).
      // Receive clock is provided by the PHY.
      reg = ext_readl(mac_instance_->mmio_, EXT_RGMII_OOB_CTRL);
      reg &= ~OOB_DISABLE;
      reg |= RGMII_LINK;
      ext_writel(mac_instance_->mmio_, reg, EXT_RGMII_OOB_CTRL);

      reg = umac_readl(mac_instance_->mmio_, UMAC_CMD);
      reg &= ~((CMD_SPEED_MASK << CMD_SPEED_SHIFT) |
             CMD_HD_EN | CMD_RX_PAUSE_IGNORE | CMD_TX_PAUSE_IGNORE);
      reg |= cmd_bits;
      umac_writel(mac_instance_->mmio_, reg, UMAC_CMD);

      mac_instance_->online_ = true;

      fbl::AutoLock lock(&mac_instance_->lock_);
      mac_instance_->UpdateLinkStatus();
    }
  } else {
    // Program UMAC and RGMII block signalling lost link via interrupt 
    reg = ext_readl(mac_instance_->mmio_, EXT_RGMII_OOB_CTRL);
    reg |= OOB_DISABLE;
    reg &= ~RGMII_LINK;
    ext_writel(mac_instance_->mmio_, reg, EXT_RGMII_OOB_CTRL);

    mac_instance_->online_ = false;

    fbl::AutoLock lock(&mac_instance_->lock_);
    mac_instance_->UpdateLinkStatus();
  }
  return mac_instance_->is_mac_ready_;
}

void BcmUniMAC::UpdateLinkStatus() {
  static bool first_online_signal = true;

  if (ethernet_client_.is_valid()) {
    // Broadcast/signal link state to all active clients
    ethernet_client_.Status(online_ ? ETHERNET_STATUS_ONLINE : 0u);
  } else {
    // Since ethernet_client bind/start after MAC init,
    // MAC init started when link first goes online,
    // so this call most often precedes the client
    if (!first_online_signal) {
      zxlogf(ERROR, "BcmUniMAC: System not ready");
    } else {
      first_online_signal = false;
    }
  }
  zxlogf(INFO, "BcmUniMAC: Link is now %s", online_ ? "up" : "down");
}

inline void BcmUniMAC::ResetUmac() {
  uint32_t reg = sys_readl(mmio_, SYS_RBUF_FLUSH_CTRL);
  reg |= (1u << 1);
  sys_writel(mmio_, reg, SYS_RBUF_FLUSH_CTRL);
  zx::nanosleep(zx::deadline_after(zx::usec(10)));

  reg &= ~(1u << 1);
  sys_writel(mmio_, reg, SYS_RBUF_FLUSH_CTRL);
  zx::nanosleep(zx::deadline_after(zx::usec(10)));
}

inline void BcmUniMAC::InitUmac() {
  // clear tx/rx counter
  umac_writel(mmio_, MIB_RESET_RX | MIB_RESET_TX | MIB_RESET_RUNT, UMAC_MIB_CTRL);
  umac_writel(mmio_, 0, UMAC_MIB_CTRL);  

  umac_writel(mmio_, ENET_MAX_MTU_SIZE, UMAC_MAX_FRAME_LEN);

  // init rx registers, enable ip header optimization
  uint32_t reg = rbuf_readl(mmio_, RBUF_CTRL);
  reg |= RBUF_ALIGN_2B;
  rbuf_writel(mmio_, reg, RBUF_CTRL);

  rbuf_writel(mmio_, 1, RBUF_TBUF_SIZE_CTRL);

  DisableIntr();
}

void BcmUniMAC::DisableIntr() {
  // Mask all interrupts.
  intrl2_0_writel(mmio_, 0xFFFFFFFF, INTRL2_CPU_MASK_SET);
  intrl2_0_writel(mmio_, 0xFFFFFFFF, INTRL2_CPU_CLEAR);
  intrl2_1_writel(mmio_, 0xFFFFFFFF, INTRL2_CPU_MASK_SET);
  intrl2_1_writel(mmio_, 0xFFFFFFFF, INTRL2_CPU_CLEAR);
}

inline void BcmUniMAC::SetHWAddr(uint8_t* addr) {
  umac_writel(mmio_, (addr[0] << 24) | (addr[1] << 16) | 
                     (addr[2] << 8) | addr[3], UMAC_MAC0);
  umac_writel(mmio_, (addr[4] << 8) | addr[5], UMAC_MAC1);
}
         
inline uint32_t BcmUniMAC::DisableDMA() {
  uint32_t reg, dma_ctrl;

  // disable DMA
  dma_ctrl = 1 << (DESC_INDEX + DMA_RING_BUF_EN_SHIFT) | DMA_EN;
  reg = tdma_readl(mmio_, DMA_CTRL);
  reg &= ~dma_ctrl;
  tdma_writel(mmio_, reg, DMA_CTRL);

  reg = rdma_readl(mmio_, DMA_CTRL);
  reg &= ~dma_ctrl;
  rdma_writel(mmio_, reg, DMA_CTRL);

  umac_writel(mmio_, 1, UMAC_TX_FLUSH);
  zx_nanosleep(zx_deadline_after(ZX_USEC(10)));
  umac_writel(mmio_, 0, UMAC_TX_FLUSH);

  return dma_ctrl;
}

void BcmUniMAC::EnableDMA(uint32_t dma_ctrl) {
  uint32_t reg;

  reg = rdma_readl(mmio_, DMA_CTRL);
  reg |= dma_ctrl;
  rdma_writel(mmio_, reg, DMA_CTRL);

  reg = tdma_readl(mmio_, DMA_CTRL);
  reg |= dma_ctrl;
  tdma_writel(mmio_, reg, DMA_CTRL);
}

zx_status_t BcmUniMAC::InitDMA() {
  // Create Rx transaction buffers
  rxn_pbuf_ = PinnedBuffer::Create(TOTAL_DESC * RXTX_BUF_LENGTH, bti_, ZX_CACHE_POLICY_CACHED);
  if (!rxn_pbuf_) {
    return ZX_ERR_NO_MEMORY;
  }

  // Setup Rx buffers
  rx_buf_ = static_cast<uint8_t*>(rxn_pbuf_->GetBaseAddress());
  zx_cache_flush(rx_buf_, TOTAL_DESC * RXTX_BUF_LENGTH,
                 ZX_CACHE_FLUSH_DATA | ZX_CACHE_FLUSH_INVALIDATE);

  // Initialize Rx DMA descriptors and control blocks
  zx_paddr_t tmpaddr;
  for (int i = 0; i < TOTAL_DESC; i++) {
    rxn_pbuf_->LookupPhys(i * RXTX_BUF_LENGTH, &tmpaddr);
    rdma_desc_writel(mmio_, static_cast<uint32_t>(tmpaddr),
                     i * DMA_DESC_SIZE + DMA_DESC_ADDRESS_LO);
    rdma_desc_writel(mmio_, static_cast<uint32_t>(tmpaddr >> 32),
                     i * DMA_DESC_SIZE + DMA_DESC_ADDRESS_HI);
  }

  // Setup Rx/Tx DMA control registers
  rdma_writel(mmio_, DMA_MAX_BURST_LENGTH, DMA_SCB_BURST_SIZE);
  tdma_writel(mmio_, DMA_MAX_BURST_LENGTH, DMA_SCB_BURST_SIZE);

  rdma_ring_writel(mmio_, 0, RDMA_PROD_INDEX);
  tdma_ring_writel(mmio_, 0, TDMA_PROD_INDEX);

  rdma_ring_writel(mmio_, 0, RDMA_CONS_INDEX);
  tdma_ring_writel(mmio_, 0, TDMA_CONS_INDEX);

  rdma_ring_writel(mmio_, ((TOTAL_DESC << DMA_RING_SIZE_SHIFT) | RXTX_BUF_LENGTH),
                   DMA_RING_BUF_SIZE);
  tdma_ring_writel(mmio_, ((TOTAL_DESC << DMA_RING_SIZE_SHIFT) | RXTX_BUF_LENGTH),
                   DMA_RING_BUF_SIZE);

  rdma_ring_writel(mmio_, (DMA_FC_THRESH_LO << DMA_XOFF_THRESHOLD_SHIFT) | DMA_FC_THRESH_HI,
                   RDMA_XON_XOFF_THRESH);
  tdma_ring_writel(mmio_, 10, DMA_MBUF_DONE_THRESH);

  rdma_ring_writel(mmio_, 0, DMA_START_ADDR);
  tdma_ring_writel(mmio_, 0, DMA_START_ADDR);

  rdma_ring_writel(mmio_, 0, RDMA_READ_PTR);
  tdma_ring_writel(mmio_, 0, TDMA_READ_PTR);

  rdma_ring_writel(mmio_, 0, RDMA_WRITE_PTR);
  tdma_ring_writel(mmio_, 0, TDMA_WRITE_PTR);

  rdma_ring_writel(mmio_, TOTAL_DESC * WORDS_PER_BD - 1, DMA_END_ADDR);
  tdma_ring_writel(mmio_, TOTAL_DESC * WORDS_PER_BD - 1, DMA_END_ADDR);

  // Disable rate control for now 
  tdma_ring_writel(mmio_, 0, TDMA_FLOW_PERIOD);

  // Enable default Rx/Tx queue (16)
  rdma_writel(mmio_, 1 << DESC_INDEX, DMA_RING_CFG);
  tdma_writel(mmio_, 1 << DESC_INDEX, DMA_RING_CFG);

  return ZX_OK;
}

void BcmUniMAC::ConfigMii() {
  // Configure phy-mode to rgmii-rxid (trivially external)
  sys_writel(mmio_, PORT_MODE_EXT_GPHY, SYS_PORT_CTRL);

  // Enable the RGMII block for the interface to work
  uint32_t reg = ext_readl(mmio_, EXT_RGMII_OOB_CTRL);
  reg &= ~ID_MODE_DIS;  // enable the RX delay
  reg |= RGMII_MODE_EN;
  ext_writel(mmio_, reg, EXT_RGMII_OOB_CTRL);
}

void BcmUniMAC::SetRxMode() {
  // Convert MAC Address to IPv6 Solicit Neighbor Multicast Address
  uint8_t snm_mac_addr[] = { 0x33, 0x33, 0xff, 0x00, 0x00, 0x01 };
  snm_mac_addr[3] = mac_addr_[3];
  snm_mac_addr[4] = mac_addr_[4];
  snm_mac_addr[5] = mac_addr_[5];

  // Ethernet multicast address of 'all nodes on the local network segment'
  uint8_t multicast[] = { 0x33, 0x33, 0x00, 0x00, 0x00, 0x01 };

  // Turn off promicuous mode
  uint32_t reg = umac_readl(mmio_, UMAC_CMD);
  reg &= ~CMD_PROMISC;
  umac_writel(mmio_, reg, UMAC_CMD);

  // Update multicast filters
  int i  = 0, j = 0;
  set_mdf_addr(mmio_, mac_addr_, &j);
  i++;
  set_mdf_addr(mmio_, snm_mac_addr, &j);
  i++;
  set_mdf_addr(mmio_, multicast, &j);
  i++;

  // Enable filters
  reg = GENMASK(MAX_MDF_FILTER - 1, MAX_MDF_FILTER - i);
  umac_writel(mmio_, reg, UMAC_MDF_CTRL);
}

zx_status_t BcmUniMAC::RequestIrqs() {
  // Create port.
  zx_status_t status = zx::port::create(ZX_PORT_BIND_TO_INTERRUPT, &port_);
  if (status != ZX_OK) {
    zxlogf(ERROR, "BcmUniMAC: zx_port_create failed %d", status);
    return status;
  }

  // Bind interrupts.
  uint32_t port_key = 0;
  for (const zx::interrupt& mac_irq : mac_irqs_) {
    if ((status = mac_irq.bind(port_, port_key++, ZX_INTERRUPT_BIND)) != ZX_OK) {
      zxlogf(ERROR, "BcmUniMAC: zx_interrupt_bind failed %d", status);
      return status;
    }
  }
  return ZX_OK;
}

void BcmUniMAC::EnableUmac(uint32_t mask, bool enable) {
  uint32_t reg;

  reg = umac_readl(mmio_, UMAC_CMD);
  if (enable) {
    reg |= mask;
  } else {
    reg &= ~mask;
  }

  umac_writel(mmio_, reg, UMAC_CMD);

  // UniMAC stops on a packet boundary, wait for a full-size packet to be processed
  if (!enable) {
    zx_nanosleep(zx_deadline_after(ZX_MSEC(1)));
  }
}

int BcmUniMAC::IRQThread() {
  zx_status_t status, irq_status;
  zx_port_packet_t packet;

  while (true) {
    status = port_.wait(zx::time::infinite(), &packet);

    if (status != ZX_OK) {
      zxlogf(ERROR, "BcmUniMAC IRQ wait failed: %d", status);
      return thrd_error;
    }

    if (packet.key > mac_irqs_.size()) {
      zxlogf(WARNING, "Invalid BcmUniMAC IRQ received");
    }

    if (packet.key == mac_irqs_.size()) {
      zxlogf(INFO, "BcmUniMAC thread terminating");
      return thrd_success;
    }

    // Handle Rx and Tx default queues
    if (packet.key == 0) {
      // Read irq status
      irq_status =  intrl2_0_readl(mmio_, INTRL2_CPU_STAT) &
                    ~intrl2_0_readl(mmio_, INTRL2_CPU_MASK_STATUS);

      // RDMA interrupt
      if (irq_status & UMAC_IRQ_RXDMA_DONE) {
        rx_irqs_++;
        status = ProcRxBuffer();
        if (status != ZX_OK) {
          return thrd_error;
        } 
      }

      // TDMA interrupt
      if (irq_status & UMAC_IRQ_TXDMA_DONE) {
        tx_irqs_++;
        ProcTxBuffer();
      }
    }

    // Handle Rx and Tx priority queues
    else if (packet.key == 1) {
      zxlogf(ERROR, "BcmUniMAC::%s: false interrupt - no Rx/Tx priority queues", __func__);   
    } else { 
      zxlogf(WARNING, "BcmUniMAC::%s: received IRQ from invalid port", __func__);
    }

    // Clear interrupt register
    intrl2_0_writel(mmio_, irq_status, INTRL2_CPU_CLEAR);

    // Acknowledge interrupt
    mac_irqs_[packet.key].ack();

  } // EOF while
  
  return thrd_success;
}

void BcmUniMAC::EthernetImplQueueTx(uint32_t options, ethernet_netbuf_t* netbuf,
                            ethernet_impl_queue_tx_callback completion_cb, void* cookie) {
  eth::BorrowedOperation<> op(netbuf, completion_cb, cookie, sizeof(ethernet_netbuf_t));

  {  // Check to make sure we are ready to accept packets
    fbl::AutoLock lock(&lock_);
    if (!online_) {
      op.Complete(ZX_ERR_UNAVAILABLE);
      return;
    }
  }

  // Check for invalid buffer length
  if (op.operation()->data_size > RXTX_BUF_LENGTH) {
    op.Complete(ZX_ERR_INVALID_ARGS);
    return;
  }

  // Check for a free transaction control block, indicating/traking free DMA descriptor
  if (tx_cbs_[curr_tx_cbs_] == USED) {
    zxlogf(ERROR, "BcmUniMAC: TX buffer overrun@ %u", curr_tx_cbs_);
    op.Complete(ZX_ERR_UNAVAILABLE);
    return;
  }

  // Get the content of I/O buffer to the physical (DMAble) memory,
  // this is the point where we really grab the content of I/O buffer
  zx_cache_flush(netbuf->data_buffer, netbuf->data_size, ZX_CACHE_FLUSH_DATA);  

  // Initialize DMA descriptor with physical address of I/O buffer,
  // and  setup its 'length_status' register
  tdma_desc_writel(mmio_, static_cast<uint32_t>(op.operation()->phys),
                   curr_tx_cbs_ * DMA_DESC_SIZE + DMA_DESC_ADDRESS_LO);
  tdma_desc_writel(mmio_, static_cast<uint32_t>(op.operation()->phys  >> 32),
                   curr_tx_cbs_ * DMA_DESC_SIZE + DMA_DESC_ADDRESS_HI);

  uint32_t len_stat = static_cast<uint32_t>(op.operation()->data_size << DMA_BUFLENGTH_SHIFT);
  len_stat |= DMA_TX_APPEND_CRC | DMA_SOP | DMA_EOP;
  tdma_desc_writel(mmio_, len_stat, curr_tx_cbs_ * DMA_DESC_SIZE + DMA_DESC_LENGTH_STATUS);
  tx_cbs_[curr_tx_cbs_] = USED;

  // Update control block and DMA producer index
  curr_tx_cbs_ = ++curr_tx_cbs_ % TOTAL_DESC;
  tdma_ring_writel(mmio_, ++tx_p_index_ & DMA_P_INDEX_MASK, TDMA_PROD_INDEX);

  // We can safely give back the ownership of this I/O buffer, and don't need to wait until
  // the associated DMA operation finished in UMAC_IRQ_TXDMA_DONE, since the 'real grabbing'
  // of this I/O buffer will happen at its next zx_cache_flush operation above
  tx_bytes_compl_ += op.operation()->data_size;
  op.Complete(ZX_OK);
}

void BcmUniMAC::ProcTxBuffer() {
  uint32_t tmp_index = tdma_ring_readl(mmio_, TDMA_CONS_INDEX) & DMA_C_INDEX_MASK;

  // Manage rollover
  if (tx_c_index_ > tmp_index) {
    tmp_index += 0x10000;
  }

  // Reclaim transmitted buffers
  while (tx_c_index_ < tmp_index) {
    tx_cbs_[tx_c_index_ % TOTAL_DESC] = FREE;
    tx_c_index_++;
    tx_pkts_compl_++;
  }
  // Mask rollover effect 
  tx_c_index_ &= DMA_C_INDEX_MASK;
}

zx_status_t BcmUniMAC::ProcRxBuffer() {
  // Get discards and index of last packet received
  rx_p_index_ = rdma_ring_readl(mmio_, RDMA_PROD_INDEX);

  // Clear HW register when we reach 75% of maximum 0xFFFF discards
  rx_discards_ += (rx_p_index_ >> DMA_P_INDEX_DISCARD_CNT_SHIFT) & DMA_P_INDEX_DISCARD_CNT_MASK;
  if (rx_discards_ >= 0xC000) {
    rx_discards_ = 0;
    rdma_ring_writel(mmio_, 0, RDMA_PROD_INDEX);
  }

  // Get index to the last packet received
  rx_p_index_ &= DMA_P_INDEX_MASK;

  // Manage rollover
  if (rx_c_index_ > rx_p_index_) {
    rx_p_index_ += 0x10000;
  }

  // Check for buffer overrun
  if (rx_p_index_ - rx_c_index_ >= TOTAL_DESC) {
    zxlogf(ERROR, "BcmUniMAC: RX buffer overrun@ %u", rx_p_index_);
    return ZX_ERR_UNAVAILABLE;
  }

  // Process incoming packets
  uint8_t* temptr;
  uint32_t len;
  while (rx_c_index_ < rx_p_index_) {
    // Get DMA flags and length
    uint32_t dma_length_status = rdma_desc_readl(mmio_,
                               (rx_c_index_ % TOTAL_DESC) * DMA_DESC_SIZE + DMA_DESC_LENGTH_STATUS);

    // Check for fragment and DMA errors
    uint32_t dma_flag = dma_length_status & 0xffff;
    if (unlikely(!(dma_flag & DMA_EOP) || !(dma_flag & DMA_SOP))) {
      zxlogf(WARNING, "BcmUniMAC::%s: Dropping fragmented packet!", __func__);
      goto next;
    }
    if (unlikely(dma_flag & (DMA_RX_CRC_ERROR | DMA_RX_OV | DMA_RX_NO | DMA_RX_LG | DMA_RX_RXER))) {
      zxlogf(WARNING, "BcmUniMAC::%s: DMA error = 0x%x\n", __func__, dma_flag);
      goto next;
    }

    // Check for valid packet length
    len = dma_length_status >> DMA_BUFLENGTH_SHIFT;
    if (len > RXTX_BUF_LENGTH) {
      zxlogf(WARNING, "BcmUniMAC::%s: Unsupported packet size received", __func__);
      goto next;
    }

    // Get the I/O buffer
    temptr = &rx_buf_[(rx_c_index_ % TOTAL_DESC) * RXTX_BUF_LENGTH];

    // Sync it with physical (DMA) buffer content
    zx_cache_flush(temptr, RXTX_BUF_LENGTH, ZX_CACHE_FLUSH_DATA | ZX_CACHE_FLUSH_INVALIDATE);

    // remove hardware 2bytes added for IP alignment
    temptr += 2;
    len -= 2;

    {  // limit scope of autolock
      fbl::AutoLock lock(&lock_);
      if ((ethernet_client_.is_valid())) {
        // Send packet to the ethernet driver
        ethernet_client_.Recv(temptr, len, 0);
        rx_bytes_compl_ += len;
      } else {
        zxlogf(ERROR, "BcmUniMAC::%s: Ethernet client lost", __func__);
        return ZX_ERR_UNAVAILABLE;
      }
    };
next:
    rx_c_index_++;
    rx_pkts_compl_++;

  } // EOF while

  // Mask rollover effect 
  rx_p_index_ &= DMA_P_INDEX_MASK;
  rx_c_index_ &= DMA_C_INDEX_MASK;

  rdma_ring_writel(mmio_, rx_c_index_ & DMA_C_INDEX_MASK, RDMA_CONS_INDEX);

  return ZX_OK;
}

zx_status_t BcmUniMAC::EthernetImplSetParam(uint32_t param, int32_t value, const void* data,
                                              size_t data_size) {
  zx_status_t status = ZX_OK;

  fbl::AutoLock lock(&lock_);

  switch (param) {
    case ETHERNET_SETPARAM_PROMISC:
      SetPromisc(static_cast<bool>(value));
      break;
    case ETHERNET_SETPARAM_MULTICAST_PROMISC:
      SetMulticastPromisc(static_cast<bool>(value));
      break;
    case ETHERNET_SETPARAM_MULTICAST_FILTER:
      status = SetMulticastFilter(value, static_cast<const uint8_t*>(data), data_size);
      break;
    case ETHERNET_SETPARAM_DUMP_REGS:
      DumpRegs();
      break;
    default:
      status = ZX_ERR_NOT_SUPPORTED;
  }

  return status;
}

void BcmUniMAC::SetPromisc(bool on) {
  uint32_t reg = umac_readl(mmio_, UMAC_CMD);

  if (on ) {
    // Turn on promicuous mode
    reg |= CMD_PROMISC;
    umac_writel(mmio_, reg, UMAC_CMD);
  } 
  else {
    // Turn off promicuous mode
    reg &= ~CMD_PROMISC;
    umac_writel(mmio_, reg, UMAC_CMD);  
  }
}

void BcmUniMAC::SetMulticastPromisc(bool on) {  
  if (on) {
    // Enable filters
    umac_writel(mmio_, multicast_filter_, UMAC_MDF_CTRL);
  }
  else {
    // Disable filters
    umac_writel(mmio_, 0, UMAC_MDF_CTRL);
  }
}

zx_status_t BcmUniMAC::SetMulticastFilter(int32_t n_addr,
                                          const uint8_t* addr_bytes, size_t addr_size) {
  // Check overflow coming either from ethernet driver or from current HW limitation
  bool overflow = (n_addr == ETHERNET_MULTICAST_FILTER_OVERFLOW) || (n_addr > MAX_MDF_FILTER);

  if (overflow) {
    // There is nothing we can do but turn on Promiscuous mode
    SetPromisc(true);
    return ZX_OK;;
  }

  if (addr_size < n_addr * MAC_ARRAY_LENGTH)
    return ZX_ERR_OUT_OF_RANGE;

  // Set multicast filter registers
  for (int i = 0, j = 0; i < n_addr; i++) {
    set_mdf_addr(mmio_, addr_bytes + i * MAC_ARRAY_LENGTH, &j);
  }

  // Calculate multicast filter mask
  multicast_filter_ = GENMASK(MAX_MDF_FILTER - 1, MAX_MDF_FILTER - n_addr);

  return ZX_OK;
}

void BcmUniMAC::DumpRegs() {
  printf("\n\n");
  zxlogf(DEBUG, "BcmUniMAC: Number of RX buffers = %d", TOTAL_DESC);
  zxlogf(DEBUG, "BcmUniMAC: Number of TX buffers = %d", TOTAL_DESC);
  zxlogf(DEBUG, "BcmUniMAC: Size of RX buffers = %d", RXTX_BUF_LENGTH);
  zxlogf(DEBUG, "BcmUniMAC: Size of TX buffers = %d", RXTX_BUF_LENGTH);
  zxlogf(DEBUG, "BcmUniMAC: Current RX pointer = %d", rx_p_index_ % TOTAL_DESC);
  zxlogf(DEBUG, "BcmUniMAC: Current TX pointer = %d\n", tx_p_index_ % TOTAL_DESC);

  zxlogf(DEBUG, "BcmUniMAC: Number of RX IRQs = %lu", rx_irqs_);
  zxlogf(DEBUG, "BcmUniMAC: Number of TX IRQs = %lu", tx_irqs_);
  zxlogf(DEBUG, "BcmUniMAC: Number of RX packets completed = %lu", rx_pkts_compl_);
  zxlogf(DEBUG, "BcmUniMAC: Number of TX packets completed = %lu", tx_pkts_compl_);
  zxlogf(DEBUG, "BcmUniMAC: Number of RX bytes received = %lu", rx_bytes_compl_);
  zxlogf(DEBUG, "BcmUniMAC: Number of TX bytes trmitted = %lu", tx_bytes_compl_);
  zxlogf(DEBUG, "BcmUniMAC: Number of discarded packets = %d\n\n", rx_discards_);
}

static constexpr zx_driver_ops_t driver_ops = []() {
  zx_driver_ops_t ops = {};
  ops.version = DRIVER_OPS_VERSION;
  ops.bind = BcmUniMAC::Create;
  return ops;
}();

}  // namespace eth

ZIRCON_DRIVER_BEGIN(bcm_unimac, eth::driver_ops, "broadcom-unimac", "0.1", 4)
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_PDEV),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_BROADCOM),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_PID, PDEV_PID_BCM2711),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_BCM_UNIMAC),
ZIRCON_DRIVER_END(bcm_unimac)
