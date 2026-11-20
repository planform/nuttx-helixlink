/****************************************************************************
 * arch/risc-v/src/hpm5300/hardware/hpm5361/hpm5361_gpio.h
 *
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Register definitions adapted from the HPM5300 SDK and HPM5361 SVD.
 * See hpm5300/REGISTER_SOURCES.txt for provenance and manual references.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_GPIO_H
#define __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hpm5361_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Hardware port indices, not consecutive software port numbers. */

#define HPM_GPIO_PORT_A  0
#define HPM_GPIO_PORT_B  1
#define HPM_GPIO_PORT_X  13
#define HPM_GPIO_PORT_Y  14

/* Register offsets. p is a hardware port index. */

#define HPM_GPIO_DI_OFFSET(p)         (0x0000 + 0x10 * (p))
#define HPM_GPIO_DO_OFFSET(p)         (0x0100 + 0x10 * (p))
#define HPM_GPIO_DO_SET_OFFSET(p)     (0x0104 + 0x10 * (p))
#define HPM_GPIO_DO_CLEAR_OFFSET(p)   (0x0108 + 0x10 * (p))
#define HPM_GPIO_DO_TOGGLE_OFFSET(p)  (0x010c + 0x10 * (p))
#define HPM_GPIO_OE_OFFSET(p)         (0x0200 + 0x10 * (p))
#define HPM_GPIO_OE_SET_OFFSET(p)     (0x0204 + 0x10 * (p))
#define HPM_GPIO_OE_CLEAR_OFFSET(p)   (0x0208 + 0x10 * (p))
#define HPM_GPIO_OE_TOGGLE_OFFSET(p)  (0x020c + 0x10 * (p))

/* Register addresses. b is the GPIO0, PGPIO or FGPIO controller base.
 * These formulas do not validate port availability or pad ownership.
 */

#define HPM_GPIO_DI(b, p)             ((b) + HPM_GPIO_DI_OFFSET(p))
#define HPM_GPIO_DO(b, p)             ((b) + HPM_GPIO_DO_OFFSET(p))
#define HPM_GPIO_DO_SET(b, p)         ((b) + HPM_GPIO_DO_SET_OFFSET(p))
#define HPM_GPIO_DO_CLEAR(b, p)       ((b) + HPM_GPIO_DO_CLEAR_OFFSET(p))
#define HPM_GPIO_DO_TOGGLE(b, p)      ((b) + HPM_GPIO_DO_TOGGLE_OFFSET(p))
#define HPM_GPIO_OE(b, p)             ((b) + HPM_GPIO_OE_OFFSET(p))
#define HPM_GPIO_OE_SET(b, p)         ((b) + HPM_GPIO_OE_SET_OFFSET(p))
#define HPM_GPIO_OE_CLEAR(b, p)       ((b) + HPM_GPIO_OE_CLEAR_OFFSET(p))
#define HPM_GPIO_OE_TOGGLE(b, p)      ((b) + HPM_GPIO_OE_TOGGLE_OFFSET(p))

#endif /* __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_GPIO_H */
