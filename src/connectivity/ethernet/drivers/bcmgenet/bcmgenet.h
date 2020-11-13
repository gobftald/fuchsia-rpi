// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// created by gobftald

#ifndef SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_BCMGENET_H_
#define SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_BCMGENET_H_

#include <lib/device-protocol/pdev.h>
#include <lib/mmio/mmio.h>
#include <lib/sync/completion.h>
#include <lib/zircon-internal/thread_annotations.h>
#include <threads.h>

#include <ddktl/device.h>
#include <ddktl/protocol/ethernet.h>
#include <ddktl/protocol/ethernet/mac.h>
#include <fbl/array.h>
#include <fbl/mutex.h>

#include "bcmgenet_.h"
#include "mdio-bcm-unimac.h"
#include "pinned-buffer.h"

// clang-format off
//DMA transaction descriptors
typedef volatile struct bcm_dmadescr {
    uint32_t txrx_status;
    uint32_t dmamac_cntl;
    uint32_t dmamac_addr;
    uint32_t dmamac_next;
} __ALIGNED(64) bcm_dmadescr_t;
// clang-format on

// MMIO Indexes.
enum {
  ETH_MAC_MMIO,
};

// Ethernet UMAC IRQs
enum {
  ETH_MAC_IRQ_0,
  ETH_MAC_IRQ_1,
  ETH_MAC_IRQ_NUM,
};

namespace eth {

class BCMUniMacDevice :
                    public ddk::Device<BCMUniMacDevice, ddk::UnbindableNew>,
                    public ddk::EthernetImplProtocol<BCMUniMacDevice, ddk::base_protocol>,
                    public ddk::EthMacProtocol<BCMUniMacDevice> {
 public:
  BCMUniMacDevice(zx_device_t* device, pdev_protocol_t* pdev);

  static zx_status_t Create(void* ctx, zx_device_t* device);
      
  void DdkRelease();
  void DdkUnbindNew(ddk::UnbindTxn txn);

  // ZX_PROTOCOL_ETHERNET_IMPL ops.
  zx_status_t EthernetImplStart(const ethernet_ifc_protocol_t* ifc) __TA_EXCLUDES(lock_);
  void EthernetImplQueueTx(uint32_t options, ethernet_netbuf_t* netbuf,
                           ethernet_impl_queue_tx_callback completion_cb, 
                           void* cookie) __TA_EXCLUDES(lock_);
  void EthernetImplStop() __TA_EXCLUDES(lock_);
  zx_status_t EthernetImplQuery(uint32_t options, ethernet_info_t* info);
  zx_status_t EthernetImplSetParam(uint32_t param, int32_t value, const void* data,
                                   size_t data_size);
  void EthernetImplGetBti(zx::bti* bti);
  
  // ZX_PROTOCOL_ETH_MAC ops.
  zx_status_t EthMacMdioRead(uint32_t reg, uint32_t* val);
  zx_status_t EthMacMdioWrite(uint32_t reg, uint32_t val);
  zx_status_t EthMacRegisterCallbacks(const eth_mac_callbacks_t* callbacks);

 private:
  zx_status_t InitPdev();
  zx_status_t CheckMACVersion();
  zx_status_t GetPhyId();
  zx_status_t InitDevice();
  zx_status_t PhyModify(uint32_t regnum, uint32_t mask, uint32_t set);
  zx_status_t GenphyRestartAneg();
  zx_status_t ShutDown() __TA_EXCLUDES(lock_);

  void UpdateLinkStatus() __TA_REQUIRES(lock_);

  int Thread() __TA_EXCLUDES(lock_);
  int WorkerThread();
  thrd_t thread_;
  thrd_t worker_thread_;

  zx_status_t GetMACAddress(zx_device_t* dev);
  uint8_t mac_[MAC_ARRAY_LENGTH] = {};            // MAC address
  
  zx_status_t InitBuffers();
  static constexpr uint32_t kNumDesc = 4096;      // Number of tx/rx transaction descrptrs
                                                  //       4096 buffers = ~48ms of packets
  static constexpr uint32_t kTxnBufSize = 4096;   // Size of each transaction buffer

  fbl::RefPtr<PinnedBuffer> desc_buffer_;  
  fbl::RefPtr<PinnedBuffer> txn_buffer_;

  bcm_dmadescr_t* tx_descriptors_ = nullptr;
  bcm_dmadescr_t* rx_descriptors_ = nullptr;

  uint8_t* tx_buffer_ = nullptr;
  //uint32_t curr_tx_buf_ = 0;
  uint8_t* rx_buffer_ = nullptr;
  //uint32_t curr_rx_buf_ = 0;


  ddk::PDev pdev_;
  std::optional<ddk::MmioBuffer> mmio_;           // base of the full ethernet mmio space
  std::optional<ddk::MmioBuffer> mdio_;           // MDIO's mmio view*/
  fbl::Array<zx::interrupt> dma_irqs_;            // DMA IRQs
  zx::port port_;
  zx::bti bti_;    

  uint32_t phy_addr_;                             // phy's address on the mii bus
  uint32_t phy_id_;                               // phy's proudct id: PDEV_PID_BCM...
    
  fbl::Mutex lock_;
  ddk::EthernetIfcProtocolClient ethernet_client_ __TA_GUARDED(lock_);

  // PHY callbacks.
  eth_mac_callbacks_t cbs_;

  // Callbacks registered signal.
  sync_completion_t cb_registered_signal_;
  
  // friend functions come from Linux following that naming convention
  
  
};

}  // namespace eth

#endif  // SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_BCMGENET_H_
