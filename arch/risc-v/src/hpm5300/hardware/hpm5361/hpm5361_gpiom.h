/****************************************************************************
 * arch/risc-v/src/hpm5300/hardware/hpm5361/hpm5361_gpiom.h
 *
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Register definitions adapted from the HPM5300 SDK and HPM5361 SVD.
 * See hpm5300/REGISTER_SOURCES.txt for provenance and manual references.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_GPIOM_H
#define __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_GPIOM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hpm5361_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* p is the hardware port index; n is the pin number within that port. */

#define HPM_GPIOM_ASSIGN_OFFSET(p, n) (0x80 * (p) + 0x04 * (n))

#define HPM_GPIOM_ASSIGN(p, n)        (HPM_GPIOM_BASE + HPM_GPIOM_ASSIGN_OFFSET(p, n))


#define HPM_GPIOM_SELECT_MASK         0x03U
#define HPM_GPIOM_SELECT_GPIO0        0x00U
#define HPM_GPIOM_SELECT_FGPIO        0x02U

#define HPM_GPIOM_HIDE_MASK           (0x0fU << 8)
#define HPM_GPIOM_LOCK                (1U << 31)

#endif /* __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_GPIOM_H */
