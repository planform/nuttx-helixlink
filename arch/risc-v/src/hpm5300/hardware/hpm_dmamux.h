/****************************************************************************
 * arch/risc-v/src/hpm5300/hardware/hpm_dmamux.h
 *
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Chip selector for HPM5300 DMAMUX definitions adapted from the HPMicro SDK.
 * Register definitions and request-source IDs belong to the selected chip.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM_DMAMUX_H
#define __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_HPM5361)
#  include "hpm5361/hpm5361_dmamux.h"
#else
#  error Unsupported HPM5300 chip
#endif

#endif /* __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM_DMAMUX_H */
