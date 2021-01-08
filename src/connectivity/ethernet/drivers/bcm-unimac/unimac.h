// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// created by gobftald

#ifndef SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_GENET_BCMGENET_H_
#define SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_GENET_BCMGENET_H_

#include <sys/types.h>
#include <lib/mmio/mmio.h>

// IEEE 802.3 Ethernet magic constants
#define ETH_DATA_LEN              1500   // Max. octets in payload
#define ETH_HLEN                  14     // Total octets in header
#define ETH_FCS_LEN               4      // Octets in the FCS

#define VLAN_HLEN                 4      // The additional bytes required by VLAN

// Body(1500) + EH_SIZE(14) + VLANTAG(4) + BRCMTAG(6) + FCS(4) = 1528.
// 1536 is multiple of 256 bytes
#define ENET_BRCM_TAG_LEN         6
#define ENET_PAD                  8
#define ENET_MAX_MTU_SIZE         (ETH_DATA_LEN + ETH_HLEN + VLAN_HLEN + \
                                   ENET_BRCM_TAG_LEN + ETH_FCS_LEN + ENET_PAD)

// SYS block offsets and register definitions
#define GENET_SYS_OFF             0x0000
#define  SYS_REV_CTRL             0x00
#define  SYS_PORT_CTRL            0x04
#define   PORT_MODE_EXT_GPHY      3
#define  SYS_RBUF_FLUSH_CTRL      0x08

// Ext block register offsets and definitions
#define GENET_EXT_OFF             0x0080
#define EXT_RGMII_OOB_CTRL        0x0C
#define  RGMII_LINK               (1 << 4)
#define  OOB_DISABLE              (1 << 5)
#define  RGMII_MODE_EN            (1 << 6)
#define  ID_MODE_DIS              (1 << 16)

// UniMac intrl2 registers
#define GENET_INTRL2_0_OFF        0x0200
#define  UMAC_IRQ_RXDMA_MBDONE    (1 << 13)
#define  UMAC_IRQ_RXDMA_DONE      UMAC_IRQ_RXDMA_MBDONE
#define   UMAC_IRQ_TXDMA_MBDONE   (1 << 16)
#define UMAC_IRQ_TXDMA_DONE       UMAC_IRQ_TXDMA_MBDONE
#define GENET_INTRL2_1_OFF        0x0240
#define  INTRL2_CPU_STAT          0x00
#define  INTRL2_CPU_CLEAR         0x08
#define  INTRL2_CPU_MASK_STATUS   0x0C
#define  INTRL2_CPU_MASK_SET      0x10
#define  INTRL2_CPU_MASK_CLEAR    0x14

#define GENET_RBUF_OFF            0x0300
#define  RBUF_CTRL                0x00
#define   RBUF_ALIGN_2B           (1 << 1)
#define  RBUF_TBUF_SIZE_CTRL      0xb4

#define UMAC_MDF_CTRL             0x650
#define UMAC_MDF_ADDR             0x654
#define  MAX_MDF_FILTER           17

// UMAC block offsets and register definitions
#define GENET_UMAC_OFF            0x0800
#define UMAC_CMD                  0x008
#define  CMD_TX_EN                (1 << 0)
#define  CMD_RX_EN                (1 << 1)
#define  UMAC_SPEED_10            0
#define  UMAC_SPEED_100           1
#define  UMAC_SPEED_1000          2
#define  CMD_SPEED_SHIFT          2
#define  CMD_SPEED_MASK           3
#define  CMD_PROMISC              (1 << 4)
#define  CMD_RX_PAUSE_IGNORE      (1 << 8)
#define  CMD_HD_EN                (1 << 10)
#define  CMD_TX_PAUSE_IGNORE      (1 << 28)
#define UMAC_MAC0                 0x00C
#define UMAC_MAC1                 0x010
#define UMAC_MAX_FRAME_LEN        0x014

#define UMAC_TX_FLUSH             0x334

#define UMAC_MIB_CTRL             0x580
#define  MIB_RESET_RX             (1 << 0)
#define  MIB_RESET_RUNT           (1 << 1)
#define  MIB_RESET_TX             (1 << 2)

#define UMAC_MDIO_CMD             0x614
#define  MDIO_CMD                 0x00
#define  MDIO_START_BUSY          (1 << 29)
#define  MDIO_RD                  (2 << 26)
#define  MDIO_WR                  (1 << 26)
#define  MDIO_PMD_SHIFT           21
#define  MDIO_REG_SHIFT           16

// MDIO registers
#define MII_BMCR                  0x00    // Basic mode control register
#define  BMCR_ANRESTART           0x0200  // Auto negotiation restart
#define  BMCR_ANENABLE            0x1000  // Enable auto negotiation
#define MII_BMSR                  0x01    // Basic mode status register
#define  BMSR_ERCAP               0x0001  // Ext-reg capability
#define  BMSR_LSTATUS             0x0004  // Link status
#define  BMSR_ANEGCAPABLE         0x0008  // Able to do auto-negotiation
#define  BMSR_ANEGCOMPLETE        0x0020  // Auto-negotiation complete
#define  BMSR_ESTATEN             0x0100  // Extended Status in R15
#define  BMSR_10HALF              0x0800  // Can do 10mbps, half-duplex
#define  BMSR_10FULL              0x1000  // Can do 10mbps, full-duplex
#define  BMSR_100HALF             0x2000  // Can do 100mbps, half-duplex
#define  BMSR_100FULL             0x4000  // Can do 100mbps, full-duplex
#define MII_PHYSID1               0x02    // high part of phy's id
#define MII_PHYSID2               0x03    // low  part of phy's id
#define MII_ADVERTISE             0x04    // Advertisement control register
#define  ADVERTISE_10HALF         0x0020  // Try for 10mbps half-duplex
#define  ADVERTISE_10FULL         0x0040  // Try for 10mbps full-duplex
#define  ADVERTISE_100HALF        0x0080  // Try for 100mbps half-duplex
#define  ADVERTISE_100FULL        0x0100  // Try for 100mbps full-duplex
#define  ADVERTISE_ALL            (ADVERTISE_10HALF | ADVERTISE_10FULL | \
                                  ADVERTISE_100HALF | ADVERTISE_100FULL)
#define MII_LPA                   0x05    // Link partner ability reg
#define  LPA_LPACK                0x4000  // Link partner acked us
#define MII_CTRL1000              0x09    // 1000BASE-T control
#define  ADVERTISE_1000HALF       0x0100  // Advertise 1000BASE-T half duplex
#define  ADVERTISE_1000FULL       0x0200  // Advertise 1000BASE-T full duplex 
#define  CTL1000_ENABLE_MASTER    0x1000                                 
#define MII_ESTATUS               0x0f    // Extended Status
#define  ESTATUS_1000_THALF       0x1000  // Can do 1000BT Half
#define  ESTATUS_1000_TFULL       0x2000  // Can do 1000BT Full
#define  ESTATUS_1000_XFULL       0x8000  // Can do 1000BaseX Full
#define MII_STAT1000              0x0a    // 1000BASE-T status
#define  LPA_1000HALF             0x0400  // Link partner 1000BASE-T half duplex
#define  LPA_1000FULL             0x0800  // Link partner 1000BASE-T full duplex
#define  LPA_1000MSFAIL           0x8000  // Master/Slave resolution failure


enum bcmgenet_version {
  GENET_V1 = 1,
  GENET_V2,
  GENET_V3,
  GENET_V4,
  GENET_V5
};

// BCMGENET V5 hardware parameters
#define BP_IN_EN_SHIFT            17
#define BP_IN_MASK                0x1ffff
#define HFB_FILTER_CNT            48
#define HFB_FILTER_SIZE           128
#define QTAG_MASK                 0x3F

#define TBUF_OFFSET               0x0600
#define RDMA_OFFSET               0x2000
#define TDMA_OFFSET               0x4000
#define HFB_OFFSET                0x8000
#define HFB_REG_OFFSET            0xfc00

// RX/TX DMA registers
#define DMA_RING_CFG              0x00
#define DMA_CTRL                  0x04            // DMA control register 
#define  DMA_EN                   (1 << 0)
#define  DMA_RING_BUF_EN_SHIFT    0x01
//#define DMA_STATUS              0x08
#define DMA_SCB_BURST_SIZE        0x0C
#define  DMA_MAX_BURST_LENGTH     0x08
#define DMA_ARB_CTRL              0x2C
#define  DMA_RING_BUF_PRIORITY_SHIFT  5
#define  DMA_PRIO_REG_INDEX(q)    ((q) / 6)
#define  DMA_PRIO_REG_SHIFT(q)    (((q) % 6) * DMA_RING_BUF_PRIORITY_SHIFT)
#define  GENET_Q0_PRIORITY        0               // Default highest priority queue

// DMA Descriptor - see WORDS_PER_BD above
#define DMA_DESC_LENGTH_STATUS    0x00            // in bytes of data in buffer
#define  DMA_BUFLENGTH_SHIFT      16
#define  DMA_EOP                  0x4000
#define  DMA_SOP                  0x2000
#define  DMA_TX_APPEND_CRC        0x0040
#define  DMA_RX_LG                0x0010
#define  DMA_RX_NO                0x0008
#define  DMA_RX_RXER              0x0004 
#define  DMA_RX_CRC_ERROR         0x0002
#define  DMA_RX_OV                0x0001
#define DMA_DESC_ADDRESS_LO       0x04            // lower bits of PA
#define DMA_DESC_ADDRESS_HI       0x08            // upper 32 bits of PA, GENETv4+

// total number of Buffer Descriptors, same for Rx/Tx
#define TOTAL_DESC        				256
#define WORDS_PER_BD              3               // see DMA Descriptor section above
#define DMA_DESC_SIZE             (WORDS_PER_BD * 4)

// Rx/Tx DMA register offset, skip 256 descriptors
#define GENET_RDMA_REG_OFF        (RDMA_OFFSET + TOTAL_DESC * DMA_DESC_SIZE)
#define GENET_TDMA_REG_OFF        (TDMA_OFFSET + TOTAL_DESC * DMA_DESC_SIZE)

// which ring is descriptor based
#define DESC_INDEX        				16

/* DMA rings size */
#define DMA_RING_SIZE             (0x40)
#define DMA_RINGS_SIZE            (DMA_RING_SIZE * (DESC_INDEX + 1))

// GENET v4+ supports 40-bits pointer addressing
// for obvious reasons the LO and HI word parts are contiguous
#define TDMA_READ_PTR             0x00
#define RDMA_WRITE_PTR            TDMA_READ_PTR
#define TDMA_READ_PTR_HI          0x04
#define RDMA_WRITE_PTR_HI         TDMA_READ_PTR_HI,
#define TDMA_CONS_INDEX           0x08
#define RDMA_PROD_INDEX           TDMA_CONS_INDEX
#define TDMA_PROD_INDEX           0x0C
#define RDMA_CONS_INDEX           TDMA_PROD_INDEX
#define  DMA_P_INDEX_MASK         0xFFFF
#define  DMA_C_INDEX_MASK         0xFFFF
#define  DMA_P_INDEX_DISCARD_CNT_MASK 0xFFFF
#define  DMA_P_INDEX_DISCARD_CNT_SHIFT  16
#define DMA_RING_BUF_SIZE         0x10
#define  DMA_RING_SIZE_SHIFT      16
#define  RXTX_BUF_LENGTH          2048
#define DMA_START_ADDR            0x14
#define DMA_START_ADDR_HI         0x18
#define DMA_END_ADDR              0x1C
#define DMA_END_ADDR_HI           0x20
#define DMA_MBUF_DONE_THRESH      0x24
#define TDMA_FLOW_PERIOD          0x28
#define RDMA_XON_XOFF_THRESH      TDMA_FLOW_PERIOD
#define  DMA_FC_THRESH_HI         (TOTAL_DESC >> 4)
#define  DMA_FC_THRESH_LO         5
#define  DMA_XOFF_THRESHOLD_SHIFT 16
#define TDMA_WRITE_PTR            0x2C
#define RDMA_READ_PTR             TDMA_WRITE_PTR
#define TDMA_WRITE_PTR_HI         0x30
#define RDMA_READ_PTR_HI          TDMA_WRITE_PTR_HI

extern bool io_debug;
#define GENET_IO_MACRO(name, offset)                                                      \
inline uint32_t name##_readl(std::optional<ddk::MmioBuffer>& mmio, uint32_t off) {        \
  if (io_debug) {                                                                         \
    zxlogf(DEBUG, "mmio->Read32(offset=0x%x + off=0x%x) -> 0x%x\n",                       \
           offset, off, mmio->Read32(offset + off));                                      \
  }                                                                                       \
  return mmio->Read32(offset + off);                                                      \
}                                                                                         \
inline void name##_writel(std::optional<ddk::MmioBuffer>& mmio,                           \
                                 uint32_t val, uint32_t off)  {                           \
  if (io_debug) {                                                                         \
    zxlogf(DEBUG, "mmio->Write32(val=0x%x, offset=0x%x + off=0x%x)\n", val, offset, off); \
  }                                                                                       \
  mmio->Write32(val, offset + off);                                                       \
}

GENET_IO_MACRO(sys, GENET_SYS_OFF)
GENET_IO_MACRO(ext, GENET_EXT_OFF)
GENET_IO_MACRO(intrl2_0, GENET_INTRL2_0_OFF)      // interrupt l2 registers accessors
GENET_IO_MACRO(intrl2_1, GENET_INTRL2_1_OFF)
GENET_IO_MACRO(rbuf, GENET_RBUF_OFF)              // RBUF register accessors
GENET_IO_MACRO(umac, GENET_UMAC_OFF)

GENET_IO_MACRO(rdma_desc, (GENET_SYS_OFF + RDMA_OFFSET))
GENET_IO_MACRO(tdma_desc, (GENET_SYS_OFF + TDMA_OFFSET))

GENET_IO_MACRO(rdma_ring, (GENET_RDMA_REG_OFF + (DMA_RING_SIZE * DESC_INDEX)))
GENET_IO_MACRO(tdma_ring, (GENET_TDMA_REG_OFF + (DMA_RING_SIZE * DESC_INDEX)))

GENET_IO_MACRO(rdma, (GENET_RDMA_REG_OFF + DMA_RINGS_SIZE))
GENET_IO_MACRO(tdma, (GENET_TDMA_REG_OFF + DMA_RINGS_SIZE))

inline void set_mdf_addr(std::optional<ddk::MmioBuffer>& mmio, const uint8_t *addr, int *i) {
  umac_writel(mmio, addr[0] << 8 | addr[1], UMAC_MDF_ADDR + (*i * 4));
  umac_writel(mmio, addr[2] << 24 | addr[3] << 16 | addr[4] << 8 | addr[5],
              UMAC_MDF_ADDR + ((*i + 1) * 4));
  *i += 2;
}

#define GENMASK(h, l) \
        (((~0u) - (1u << (l)) + 1) & (~0u >> (31 - (h))))

#endif  // SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_GENET_BCMGENET_H_
