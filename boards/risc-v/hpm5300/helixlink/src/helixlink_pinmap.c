/****************************************************************************
 * boards/risc-v/hpm5300/helixlink/src/helixlink_pinmap.c
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

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "hpm_gpio.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_HPM_UART3

const struct hpm_gpio_config_s g_helixlink_uart3_rx =
{
  .ioc =
  {
    .domain   = HPM_IOC_DOMAIN_MAIN,
    .pad      = HPM_IOC_PAD_PB14,
    .function = IOC_PB14_FUNC_CTL_UART3_RXD,
  },
  .controller = HPM_GPIO_CONTROLLER_GPIO0,
  .mode       = HPM_GPIO_PERIPHERAL,
};

const struct hpm_gpio_config_s g_helixlink_uart3_tx =
{
  .ioc =
  {
    .domain   = HPM_IOC_DOMAIN_MAIN,
    .pad      = HPM_IOC_PAD_PB15,
    .function = IOC_PB15_FUNC_CTL_UART3_TXD,
  },
  .controller = HPM_GPIO_CONTROLLER_GPIO0,
  .mode       = HPM_GPIO_PERIPHERAL,
};

#endif
