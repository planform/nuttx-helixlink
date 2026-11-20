/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_clockconfig.h
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

#ifndef __ARCH_RISCV_SRC_HPM5300_HPM_CLOCKCONFIG_H
#define __ARCH_RISCV_SRC_HPM5300_HPM_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum hpm_clock_source_e
{
  HPM_CLOCK_SOURCE_OSC0 = 0,  /* CLK_24M: automatic XTAL/RC selection */
  HPM_CLOCK_SOURCE_PLL0_CLK0,
  HPM_CLOCK_SOURCE_PLL0_CLK1,
  HPM_CLOCK_SOURCE_PLL0_CLK2,
  HPM_CLOCK_SOURCE_PLL1_CLK0,
  HPM_CLOCK_SOURCE_PLL1_CLK1,
  HPM_CLOCK_SOURCE_PLL1_CLK2,
  HPM_CLOCK_SOURCE_PLL1_CLK3
};

struct hpm_clock_config_s
{
  enum hpm_clock_source_e source;
  uint32_t source_frequency;  /* Source frequency before division, in Hz */
  unsigned int divider;       /* Actual division ratio: 1 through 256 */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Configure only the specified UART clock, leaving other UARTs untouched.
 * Return the input frequency in Hz, or zero on failure.
 * The implementation belongs to the HPM5300 clock driver.
 */

#ifdef __cplusplus
extern "C"
{
#endif

int hpm_clockconfig(void);
uint32_t hpm_uart_clockconfig(uint32_t base);
uint32_t hpm_mchtmr_clockconfig(void);
uint32_t hpm_spi_clockconfig(uint32_t base);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_HPM5300_HPM_CLOCKCONFIG_H */
