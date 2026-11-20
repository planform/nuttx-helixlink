/****************************************************************************
 * boards/risc-v/hpm5300/helixlink/src/helixlink_boot.c
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

#include <nuttx/board.h>
#include <syslog.h>
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: helixlink_boardinitialize
 *
 * Description:
 *   Perform early HelixLink board initialization before nx_start().
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void helixlink_boardinitialize(void)
{
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   Initialize board devices after OS initialization and before
 *   starting the initial application.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
#ifdef CONFIG_HPM_SPI1
  int ret = helixlink_spi_initialize();

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI1: %d\n", ret);
      return;
    }

#ifdef CONFIG_HELIXLINK_SPIFLASH
  ret = helixlink_flash_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize external Flash: %d\n", ret);
    }
#endif
#endif
}
#endif
