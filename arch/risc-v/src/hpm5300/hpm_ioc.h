/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_ioc.h
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

#ifndef __ARCH_RISCV_SRC_HPM5300_HPM_IOC_H
#define __ARCH_RISCV_SRC_HPM5300_HPM_IOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include "hardware/hpm_ioc.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum hpm_ioc_domain_e
{
  HPM_IOC_DOMAIN_MAIN = 0,
  HPM_IOC_DOMAIN_PMIC
};

/* The function field contains FUNC_CTL bits. pad_mask selects the PAD_CTL
 * fields to change; pad_value must be a subset of pad_mask. A zero pad_mask
 * leaves electrical settings unchanged. Reserved bits are always preserved.
 * Pin availability and alternate-function choice are board responsibilities.
 */

struct hpm_ioc_config_s
{
  enum hpm_ioc_domain_e domain;
  uint16_t pad;
  uint32_t function;
  uint32_t pad_mask;
  uint32_t pad_value;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Validate the descriptor without changing hardware. */

int hpm_ioc_validate(const struct hpm_ioc_config_s *config);

/* Configure one IOC/PIOC pad. The caller must first disable any peripheral
 * output that could glitch during remapping. GPIO output-enable control is
 * handled by the GPIO layer, not by this function.
 *
 * Main-domain functions on PY pads additionally require the corresponding
 * PIOC pad to select SOC_GPIO_Y. Configure that route explicitly through a
 * PMIC descriptor; selecting a main IOC function does not change ownership.
 *
 * These functions do not allocate memory. Concurrent configuration of the
 * same pad requires caller-level ownership.
 */

int hpm_ioc_configure(const struct hpm_ioc_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_HPM5300_HPM_IOC_H */
