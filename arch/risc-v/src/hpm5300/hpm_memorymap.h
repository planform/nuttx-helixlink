/*
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Register definitions adapted from the HPM5300 SDK and HPM5361 SVD.
 * See hpm5300/REGISTER_SOURCES.txt for provenance and manual references.
 */

#ifndef __ARCH_RISCV_SRC_HPM5300_HPM_MEMORYMAP_H
#define __ARCH_RISCV_SRC_HPM5300_HPM_MEMORYMAP_H

#include "riscv_common_memorymap.h"
#include "hardware/hpm_memorymap.h"

/* Idle thread stack starts at the end of .bss. */

#ifndef __ASSEMBLY__
#  define HPM_IDLESTACK_BASE  (uintptr_t)_ebss
#else
#  define HPM_IDLESTACK_BASE  _ebss
#endif

#define HPM_IDLESTACK_TOP  (HPM_IDLESTACK_BASE + SMP_STACK_SIZE)

#endif /* __ARCH_RISCV_SRC_HPM5300_HPM_MEMORYMAP_H */
