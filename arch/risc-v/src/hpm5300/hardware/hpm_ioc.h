/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Register definitions adapted from the HPM5300 SDK and HPM5361 SVD.
 * See hpm5300/REGISTER_SOURCES.txt for provenance and manual references.
 */

#ifndef __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM_IOC_H
#define __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM_IOC_H

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_HPM5361)
#  include "hpm5361/hpm5361_ioc.h"
#else
#  error Unsupported HPM5300 chip
#endif

#endif /* __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM_IOC_H */
