/****************************************************************************
 * boards/risc-v/hpm5300/helixlink/src/helixlink_clockconfig.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "hpm_clockconfig.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct hpm_clock_config_s g_helixlink_mchtmr_clock =
{
  .source           = HPM_CLOCK_SOURCE_OSC0,
  .source_frequency = BOARD_CLK24M_FREQUENCY,
  .divider          = 1,
};

#ifdef CONFIG_HPM_UART3

const struct hpm_clock_config_s g_helixlink_uart3_clock =
{
  .source           = HPM_CLOCK_SOURCE_OSC0,
  .source_frequency = BOARD_CLK24M_FREQUENCY,
  .divider          = 1,
};

#endif /* ifdef CONFIG_HPM_UART3 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Board clock policy is data-only; arch performs the hardware operations. */

#ifdef CONFIG_HPM_SPI1
const struct hpm_clock_config_s g_helixlink_spi1_clock =
{
  .source           = HPM_CLOCK_SOURCE_OSC0,
  .source_frequency = BOARD_CLK24M_FREQUENCY,
  .divider          = 1,
};
#endif
