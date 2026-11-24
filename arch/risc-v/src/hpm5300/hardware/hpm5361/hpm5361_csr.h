/****************************************************************************
 * arch/risc-v/src/hpm5300/hardware/hpm5361/hpm5361_csr.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_CSR_H
#define __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_CSR_H

#define HPM_CSR_MMISC_CTL       0x7d0
#define HPM_MMISC_CTL_VEC_PLIC  (1 << 1)

/* HPM5361 SDK hpm_csr_regs.h and arch/riscv/l1c/hpm_l1c_drv.h. */

#define HPM_CSR_MCACHE_CTL           0x7ca
#define HPM_CSR_MCCTLCOMMAND         0x7cc
#define HPM_MCACHE_CTL_DC_EN         (1 << 1)
#define HPM_MCACHE_CTL_DPREF_EN      (1 << 10)
#define HPM_CCTL_L1D_WBINVAL_ALL     6

#endif /* __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_CSR_H */
