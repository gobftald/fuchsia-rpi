// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
//
// SPDX-License-Identifier: GPL-2.0 
//
// ported by gobftald from Linux: include/linux/brcmphy.h

#ifndef SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_54210E_BRCMPHY_H_
#define SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_54210E_BRCMPHY_H_


// Broadcom BCM54XX register definitions, common to most Broadcom PHYs

#define MII_BCM54XX_ECR		                    0x10	  // BCM54xx extended control register
#define MII_BCM54XX_ECR_IM	                  0x1000	// Interrupt mask

#define MII_BCM54XX_EXP_DATA	                0x15	  // Expansion register data

#define MII_BCM54XX_EXP_SEL	                  0x17	  // Expansion register select
#define MII_BCM54XX_EXP_SEL_ER	              0x0f00	// Expansion register select
#define MII_BCM54XX_EXP_SEL_ETC	              0x0d00	// Expansion register spare + 2k mem


// AUXILIARY CONTROL SHADOW ACCESS REGISTERS
#define MII_BCM54XX_AUX_CTL	                  0x18    // Auxiliary control register
#define MII_BCM54XX_AUXCTL_SHDWSEL_AUXCTL     0x00    //*
#define MII_BCM54XX_AUXCTL_ACTL_TX_6DB        0x0400  //*
#define MII_BCM54XX_AUXCTL_ACTL_SMDSP_ENA     0x0800  //*

#define MII_BCM54XX_AUXCTL_SHDWSEL_MISC			  0x07
#define MII_BCM54XX_AUXCTL_SHDWSEL_MISC_RGMII_SKEW_EN	\
                                              0x0100
#define MII_BCM54XX_AUXCTL_MISC_WREN			    0x8000

#define MII_BCM54XX_AUXCTL_SHDWSEL_READ_SHIFT	12
#define MII_BCM54XX_AUXCTL_SHDWSEL_MASK	      0x0007


#define MII_BCM54XX_IMR		                    0x1b	  // BCM54xx interrupt mask register
#define MII_BCM54XX_INT_LINK	                0x0002	// Link status changed
#define MII_BCM54XX_INT_SPEED	                0x0004	// Link speed change
#define MII_BCM54XX_INT_DUPLEX	              0x0008	// Duplex mode changed


#define MII_BCM54XX_SHD		                    0x1c	  // 0x1c shadow registers
#define MII_BCM54XX_SHD_WRITE	                0x8000
#define MII_BCM54XX_SHD_VAL(x)	              ((x & 0x1f) << 10)
#define MII_BCM54XX_SHD_DATA(x)	              ((x & 0x3ff) << 0)


// Broadcom LED source encodings.  These are used in
// BCM5461, BCM5481, BCM5482, and possibly some others.
#define BCM_LED_SRC_MULTICOLOR1	              0xa


// Broadcom Multicolor LED configurations (expansion register 4)
#define BCM_EXP_MULTICOLOR		                (MII_BCM54XX_EXP_SEL_ER + 0x04)
#define BCM_LED_MULTICOLOR_IN_PHASE	          (1 << 8)
#define BCM_LED_MULTICOLOR_LINK_ACT	          0x0
#define BCM_LED_MULTICOLOR_LINK		            0x8


// BCM5482: Shadow registers
// Shadow values go into bits [14:10] of register 0x1c to select a shadow
// register to access.

// 00101: Spare Control Register 3
#define BCM54XX_SHD_SCR3		                  0x05


#define BCM5482_SHD_LEDS1	                    0x0d	              // 01101: LED Selector 1
#define BCM5482_SHD_LEDS1_LED3(src)	          ((src & 0xf) << 4)  // ~LINKSPD[2] selector
#define BCM5482_SHD_LEDS1_LED1(src)	          ((src & 0xf) << 0)  // ~LINKSPD[1] selector


// BCM54810 Registers
#define BCM54810_SHD_CLK_CTL			            0x3
#define BCM54810_SHD_CLK_CTL_GTXCLK_EN		    (1 << 9)


#endif  // SRC_CONNECTIVITY_ETHERNET_DRIVERS_BCM_54210E_BRCMPHY_H_
