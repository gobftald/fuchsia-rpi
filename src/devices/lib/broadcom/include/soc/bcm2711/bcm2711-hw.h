// Copyright 2020 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef SRC_DEVICES_LIB_BROADCOM_INCLUDE_SOC_BCM2711_BCM2711_HW_H_
#define SRC_DEVICES_LIB_BROADCOM_INCLUDE_SOC_BCM2711_BCM2711_HW_H_

// Ethernet base
#define BCM2711_ETH_BASE        0xfd580000            // from DT (BCM54210E)
#define BCM2711_ETH_LENGTH      0x10000

// Ethernet UMAC IRQs
#define BCM2711_UMAC_IRQ_0      189                   // GIC_SPI 157
#define BCM2711_UMAC_IRQ_1      190                   // GIC_SPI 158

// GPIO base
#define BCM2711_GPIO_BASE       0xfe200000
#define BCM2711_GPIO_LENGTH          0x100

// GPIO IRQs
#define BCM2711_GPIO_IRQ_0      145                   // GIC_SPI 113
#define BCM2711_GPIO_IRQ_1      146                   // GIC_SPI 114

// GPIO bank boundaries
#define BCM2711_GPIO_MAX_PIN    45                    //  0 - 45
#define BCM2711_GPIO_BANK0_END  27                    //  0 - 27

#endif  // SRC_DEVICES_LIB_BROADCOM_INCLUDE_SOC_BCM2711_BCM2711_HW_H_
