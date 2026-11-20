/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_gpio.h
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

#ifndef __ARCH_RISCV_SRC_HPM5300_HPM_GPIO_H
#define __ARCH_RISCV_SRC_HPM5300_HPM_GPIO_H

#ifndef __ASSEMBLY__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

#include "hpm_ioc.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum hpm_gpio_mode_e
{
  HPM_GPIO_INPUT = 0,
  HPM_GPIO_OUTPUT,
  HPM_GPIO_PERIPHERAL
};

enum hpm_gpio_controller_e
{
  HPM_GPIO_CONTROLLER_GPIO0 = 0,
  HPM_GPIO_CONTROLLER_FGPIO,
  HPM_GPIO_CONTROLLER_PGPIO
};

struct hpm_gpio_config_s
{
  struct hpm_ioc_config_s ioc;
  enum hpm_gpio_controller_e controller;
  enum hpm_gpio_mode_e mode;
  bool output_high;   /* Initial output level; used only in output mode */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

int hpm_gpio_config(const struct hpm_gpio_config_s *config);

int hpm_gpio_write(const struct hpm_gpio_config_s *config, bool value);

int hpm_gpio_read(const struct hpm_gpio_config_s *config, bool *value);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_HPM5300_HPM_GPIO_H */
