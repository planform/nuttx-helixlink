/****************************************************************************
 * arch/risc-v/src/hpm5300/hardware/hpm_spi.h
 *
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM_SPI_H
#define __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_HPM5361)
#  include "hpm5361/hpm5361_spi.h"
#else
#  error Unsupported HPM5300 chip
#endif

#endif /* __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM_SPI_H */
