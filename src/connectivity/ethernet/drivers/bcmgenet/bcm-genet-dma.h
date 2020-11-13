// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_BCM_GENET_DMA_H_
#define SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_BCM_GENET_DMA_H_

// clang-format off

// rx status bits definitions
#define DESC_RXSTS_OWNBYDMA            (1 << 31)


// tx control bits definitions
#define DESC_TXCTRL_TXCHAIN            (1 << 24)


// rx control bits definitions
#define DESC_RXCTRL_RXCHAIN            (1 << 24)

#define DESC_RXCTRL_SIZE1MASK          (0x7FF << 0)


#define MAC_MAX_FRAME_SZ                (1600)

#endif  // SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_BCM_GENET_DMA_H_
