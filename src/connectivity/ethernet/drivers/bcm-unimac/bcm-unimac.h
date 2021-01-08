// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// created by gobftald

#ifndef SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_GENET_BCM_GENET_H_
#define SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_GENET_BCM_GENET_H_

#include "gen-phy.h"
#include "pinned-buffer.h"

#include <lib/device-protocol/pdev.h>
#include <lib/sync/completion.h>
#include <lib/zircon-internal/thread_annotations.h>

#include <ddktl/device.h>
#include <ddktl/protocol/ethernet.h>
#include <ddktl/protocol/ethernet/mac.h>

#include <fbl/array.h>
#include <fbl/auto_lock.h>


// Genet IRQs
enum {
  GENET_IRQ_0,
  GENET_IRQ_1,
  GENET_IRQ_NUM,
};

// MMIO Indexes.
enum {
  GENET_MMIO,
};

// control block status
enum cbs_stat: bool {
  FREE,
  USED,
};


namespace eth {

class BcmUniMAC : public ddk::Device<BcmUniMAC, ddk::UnbindableNew>,
                        public ddk::EthernetImplProtocol<BcmUniMAC, ddk::base_protocol>,
                        public ddk::EthMacProtocol<BcmUniMAC> {
 public:
  BcmUniMAC(zx_device_t* device, pdev_protocol_t* pdev)
      : ddk::Device<BcmUniMAC, ddk::UnbindableNew>(device), pdev_(pdev) { mac_instance_ = this; }

  static zx_status_t Create(void* ctx, zx_device_t* device);

  void DdkRelease();
  void DdkUnbindNew(ddk::UnbindTxn txn);

  // ZX_PROTOCOL_ETHERNET_IMPL ops.
  zx_status_t EthernetImplQuery(uint32_t options, ethernet_info_t* info);
  zx_status_t EthernetImplStart(const ethernet_ifc_protocol_t* ifc) __TA_EXCLUDES(lock_);
  void EthernetImplQueueTx(uint32_t options, ethernet_netbuf_t* netbuf,
                           ethernet_impl_queue_tx_callback completion_cb, 
                           void* cookie) __TA_EXCLUDES(lock_);
  void EthernetImplStop() __TA_EXCLUDES(lock_);
  zx_status_t EthernetImplSetParam(uint32_t param, int32_t value, const void* data,
                                   size_t data_size);
  void EthernetImplGetBti(zx::bti* bti) { bti_.duplicate(ZX_RIGHT_SAME_RIGHTS, bti); }

  // ZX_PROTOCOL_ETH_MAC ops.
  zx_status_t EthMacMdioWrite(uint32_t reg, uint32_t val);
  zx_status_t EthMacMdioRead(uint32_t reg, uint32_t* val);
  zx_status_t EthMacRegisterCallbacks(const eth_mac_callbacks_t* callbacks);

 private:
  zx_status_t ShutDown() __TA_EXCLUDES(lock_);

  zx_status_t InitPdev();
  zx_status_t GetMACAddress(zx_device_t*);
  zx_status_t CheckMACVersion();
  int WorkerThread();
  void ConfigMii();
  static bool SetupMii(uint32_t);
  void UpdateLinkStatus() __TA_REQUIRES(lock_);
  void ResetUmac();
  void InitUmac();
  void DisableIntr();
  void SetHWAddr(uint8_t *addr);
  uint32_t DisableDMA();
  zx_status_t InitDMA();
  void EnableDMA(uint32_t);
  zx_status_t RequestIrqs() ;
  int IRQThread();
  void SetRxMode();
  void EnableUmac(uint32_t, bool);
  void ProcTxBuffer();
  zx_status_t ProcRxBuffer();
  void SetPromisc(bool) TA_REQ(lock_);
  void SetMulticastPromisc(bool) TA_REQ(lock_);
  zx_status_t SetMulticastFilter(int32_t, const uint8_t*, size_t) TA_REQ(lock_);
  void DumpRegs();


  ddk::PDev pdev_;
  fbl::Array<zx::interrupt> mac_irqs_;
  zx::port port_;
  uint8_t mac_addr_[MAC_ARRAY_LENGTH] = {};
  std::optional<ddk::MmioBuffer> mmio_;
  zx::bti bti_;

  fbl::Mutex lock_;
  thrd_t worker_thread_;
  thrd_t thread_;
  std::optional<GenPhy> phy_;
  bool online_ = false;
  bool is_mac_ready_ = false;

  ddk::EthernetIfcProtocolClient ethernet_client_ __TA_GUARDED(lock_);

  static inline BcmUniMAC *mac_instance_;

  eth_mac_callbacks_t phy_cbs_;
  inline static sync_completion_t phy_signal_;    // signal for Zircon PHY and GenPhy devices

  cbs_stat tx_cbs_[TOTAL_DESC];
  uint32_t curr_tx_cbs_ = 0;
  uint32_t tx_p_index_ = 0;
  uint32_t tx_c_index_ = 0;
  uint64_t tx_irqs_ = 0;
  uint64_t tx_pkts_compl_ = 0;
  uint64_t tx_bytes_compl_ = 0;

  fbl::RefPtr<PinnedBuffer> rxn_pbuf_;
  uint8_t *rx_buf_ = nullptr;
  uint32_t rx_discards_ = 0;
  uint32_t rx_p_index_ = 0;
  uint32_t rx_c_index_ = 0;
  uint64_t rx_irqs_ = 0;
  uint64_t rx_pkts_compl_ = 0;
  uint64_t rx_bytes_compl_ = 0;

  uint32_t multicast_filter_ __TA_GUARDED(lock_) = 0;
};


}  // namespace eth

#endif  // SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_GENET_BCM_GENET_H_
