// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// created by gobftald

#ifndef SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_BCMGENET__H_
#define SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_BCMGENET__H_

#define BIT(n) (1u << n)

enum bcmgenet_version {
	GENET_V5 = 5
};

// total number of Buffer Descriptors, same for Rx/Tx
#define TOTAL_DESC				        256


// SYS block offsets and register definitions
#define GENET_SYS_OFF			        0x0000
#define GENET_EXT_OFF			        0x0080
#define GENET_INTRL2_0_OFF		    0x0200
#define GENET_INTRL2_1_OFF		    0x0240


// SYS block offsets and register definitions
#define SYS_REV_CTRL			        0x00
#define SYS_PORT_CTRL			        0x04
#define  PORT_MODE_EXT_GPHY		    3
#define SYS_RBUF_FLUSH_CTRL		    0x08


// Ext block register offsets and definitions
#define EXT_RGMII_OOB_CTRL		    0x0C
#define   RGMII_LINK			        (1 << 4)
#define   OOB_DISABLE			        (1 << 5)
#define   RGMII_MODE_EN			      (1 << 6)
#define   ID_MODE_DIS			        (1 << 16)

// UMAC block offsets and register definitions
#define GENET_UMAC_OFF			      0x0800
#define   UMAC_CMD			          0x008
#define     CMD_TX_EN			        (1 << 0)
#define     CMD_RX_EN			        (1 << 1)
#define     UMAC_SPEED_10			    0
#define     UMAC_SPEED_100		    1
#define     UMAC_SPEED_1000		    2
#define     UMAC_SPEED_2500		    3
#define     CMD_SPEED_SHIFT		    2
#define     CMD_SPEED_MASK			  3
#define     CMD_RX_PAUSE_IGNORE		(1 << 8)
#define     CMD_HD_EN			        (1 << 10)
#define     CMD_TX_PAUSE_IGNORE		(1 << 28)

#define   UMAC_MDIO_CMD			      0x614
#define     MDIO_CMD		          0x00
#define       MDIO_START_BUSY		  (1 << 29)
#define       MDIO_RD			        (2 << 26)
#define       MDIO_WR			        (1 << 26)
#define       MDIO_PMD_SHIFT		  21
#define       MDIO_REG_SHIFT		  16

// uniMac intrl2 registers
#define INTRL2_CPU_STAT			      0x00
#define INTRL2_CPU_SET			      0x04
#define INTRL2_CPU_CLEAR		      0x08
#define INTRL2_CPU_MASK_STATUS	  0x0C
#define INTRL2_CPU_MASK_SET		    0x10
#define INTRL2_CPU_MASK_CLEAR		  0x14

// INTRL2 instance 0 definitions
#define UMAC_IRQ_LINK_UP		      (1 << 4)
#define UMAC_IRQ_LINK_DOWN		    (1 << 5)
#define UMAC_IRQ_LINK_EVENT		    (UMAC_IRQ_LINK_UP | UMAC_IRQ_LINK_DOWN)

#define MII_BMCR                  0x00    // Basic mode control register
#define   BMCR_ISOLATE            0x0400  // Isolate data paths from MII
#define   BMCR_ANRESTART          0x0200  // Auto negotiation restart
#define   BMCR_ANENABLE           0x1000  // Enable auto negotiation
#define MII_BMSR                  0x01    // Basic mode status register 
#define   BMSR_ESTATEN            0x0100  // Extended Status in R15
#define   BMSR_10HALF             0x0800  // Can do 10mbps, half-duplex
#define   BMSR_10FULL             0x1000  // Can do 10mbps, full-duplex
#define   BMSR_100HALF            0x2000  // Can do 100mbps, half-duplex
#define   BMSR_100FULL            0x4000  // Can do 100mbps, full-duplex
#define MII_PHYSID1               0x02    // high part of phy's id
#define MII_PHYSID2               0x03    // low  part of phy's id
#define PHY_MAX_ADDR	            32      // max number of phys on mii bus


/* DMA rings size */
#define DMA_RING_SIZE			        (0x40)
#define DMA_RINGS_SIZE			      (DMA_RING_SIZE * (DESC_INDEX + 1))


// BCMGENET hardware parameters, keep this structure nicely aligned
// since it is going to be used in hot paths
struct bcmgenet_hw_params {
	uint32_t tdma_offset;
	uint32_t words_per_bd;
};

// RX/TX DMA register accessors
enum dma_reg {
	DMA_RING_CFG = 0,
	DMA_CTRL,
	DMA_STATUS,
	DMA_SCB_BURST_SIZE,
	DMA_ARB_CTRL,
	DMA_PRIORITY_0,
	DMA_PRIORITY_1,
	DMA_PRIORITY_2,
	DMA_INDEX2RING_0,
	DMA_INDEX2RING_1,
	DMA_INDEX2RING_2,
	DMA_INDEX2RING_3,
	DMA_INDEX2RING_4,
	DMA_INDEX2RING_5,
	DMA_INDEX2RING_6,
	DMA_INDEX2RING_7,
	DMA_RING0_TIMEOUT,
	DMA_RING1_TIMEOUT,
	DMA_RING2_TIMEOUT,
	DMA_RING3_TIMEOUT,
	DMA_RING4_TIMEOUT,
	DMA_RING5_TIMEOUT,
	DMA_RING6_TIMEOUT,
	DMA_RING7_TIMEOUT,
	DMA_RING8_TIMEOUT,
	DMA_RING9_TIMEOUT,
	DMA_RING10_TIMEOUT,
	DMA_RING11_TIMEOUT,
	DMA_RING12_TIMEOUT,
	DMA_RING13_TIMEOUT,
	DMA_RING14_TIMEOUT,
	DMA_RING15_TIMEOUT,
	DMA_RING16_TIMEOUT,
};

// RDMA/TDMA ring registers and accessors
// we merge the common fields and just prefix with T/D the registers
// having different meaning depending on the direction
enum dma_ring_reg {
	TDMA_READ_PTR = 0,
	RDMA_WRITE_PTR = TDMA_READ_PTR,
	TDMA_READ_PTR_HI,
	RDMA_WRITE_PTR_HI = TDMA_READ_PTR_HI,
	TDMA_CONS_INDEX,
	RDMA_PROD_INDEX = TDMA_CONS_INDEX,
	TDMA_PROD_INDEX,
	RDMA_CONS_INDEX = TDMA_PROD_INDEX,
	DMA_RING_BUF_SIZE,
	DMA_START_ADDR,
	DMA_START_ADDR_HI,
	DMA_END_ADDR,
	DMA_END_ADDR_HI,
	DMA_MBUF_DONE_THRESH,
	TDMA_FLOW_PERIOD,
	RDMA_XON_XOFF_THRESH = TDMA_FLOW_PERIOD,
	TDMA_WRITE_PTR,
	RDMA_READ_PTR = TDMA_WRITE_PTR,
	TDMA_WRITE_PTR_HI,
	RDMA_READ_PTR_HI = TDMA_WRITE_PTR_HI
};


#define GENET_IO_MACRO(name, offset)					                   \
static inline uint32_t bcmgenet_##name##_readl(                  \
                           std::optional<ddk::MmioBuffer>& mmio, \
                           uint32_t off) {                       \
  printf("* mmio->Read32(offset=0x%x + off=0x%x) -> 0x%x\n", offset, off, mmio->Read32(offset + off)); \
  return mmio->Read32(offset + off);	                           \
}									                                               \
static inline void bcmgenet_##name##_writel(                     \
                       std::optional<ddk::MmioBuffer>& mmio,     \
                       uint32_t val, uint32_t off)	{	           \
  printf("* mmio->Write32(val=0x%x, offset=0x%x + off=0x%x)\n", val, offset, off); \
  mmio->Write32(val, offset + off);		                           \
}

GENET_IO_MACRO(sys, GENET_SYS_OFF)
GENET_IO_MACRO(ext, GENET_EXT_OFF)
GENET_IO_MACRO(umac, GENET_UMAC_OFF)

// interrupt l2 registers accessors
GENET_IO_MACRO(intrl2_0, GENET_INTRL2_0_OFF)
GENET_IO_MACRO(intrl2_1, GENET_INTRL2_1_OFF)

                                            
#endif  // SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCMGENET_BCMGENET__H_
