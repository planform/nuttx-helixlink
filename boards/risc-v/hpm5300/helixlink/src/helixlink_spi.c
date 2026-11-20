/****************************************************************************
 * boards/risc-v/hpm5300/helixlink/src/helixlink_spi.c
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
#include <errno.h>
#include <arch/board/board.h>

#include "hpm_gpio.h"
#include "hpm_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HELIXLINK_SPI_PAD_MASK  (IOC_PAD_PAD_CTL_SR_MASK | \
                                 IOC_PAD_PAD_CTL_SPD_MASK)
#define HELIXLINK_SPI_PAD_VALUE (IOC_PAD_PAD_CTL_SR_SET(1) | \
                                 IOC_PAD_PAD_CTL_SPD_SET(3))

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct hpm_gpio_config_s g_helixlink_spi1_sclk =
{
  .ioc =
  {
    .domain    = HPM_IOC_DOMAIN_MAIN,
    .pad       = HPM_IOC_PAD_PA27,
    .function  = IOC_PA27_FUNC_CTL_SPI1_SCLK |
                IOC_PAD_FUNC_CTL_LOOP_BACK_MASK,
    .pad_mask  = HELIXLINK_SPI_PAD_MASK,
    .pad_value = HELIXLINK_SPI_PAD_VALUE,
  },
  .controller = HPM_GPIO_CONTROLLER_GPIO0,
  .mode       = HPM_GPIO_PERIPHERAL,
};

const struct hpm_gpio_config_s g_helixlink_spi1_miso =
{
  .ioc =
  {
    .domain    = HPM_IOC_DOMAIN_MAIN,
    .pad       = HPM_IOC_PAD_PA28,
    .function  = IOC_PA28_FUNC_CTL_SPI1_MISO,
    .pad_mask  = HELIXLINK_SPI_PAD_MASK,
    .pad_value = HELIXLINK_SPI_PAD_VALUE,
  },
  .controller = HPM_GPIO_CONTROLLER_GPIO0,
  .mode       = HPM_GPIO_PERIPHERAL,
};

const struct hpm_gpio_config_s g_helixlink_spi1_mosi =
{
  .ioc =
  {
    .domain    = HPM_IOC_DOMAIN_MAIN,
    .pad       = HPM_IOC_PAD_PA29,
    .function  = IOC_PA29_FUNC_CTL_SPI1_MOSI,
    .pad_mask  = HELIXLINK_SPI_PAD_MASK,
    .pad_value = HELIXLINK_SPI_PAD_VALUE,
  },
  .controller = HPM_GPIO_CONTROLLER_GPIO0,
  .mode       = HPM_GPIO_PERIPHERAL,
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* In ordinary SPI mode W25Q128 IO2/IO3 are /WP and /HOLD. Keep both high.
 * CS is also GPIO-controlled so it stays low across separate SPI calls.
 */

static const struct hpm_gpio_config_s g_flash_cs =
{
  .ioc =
  {
    .domain    = HPM_IOC_DOMAIN_MAIN,
    .pad       = HPM_IOC_PAD_PA26,
    .function  = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0),
    .pad_mask  = HELIXLINK_SPI_PAD_MASK | IOC_PAD_PAD_CTL_PE_MASK |
                 IOC_PAD_PAD_CTL_PS_MASK,
    .pad_value = HELIXLINK_SPI_PAD_VALUE | IOC_PAD_PAD_CTL_PE_MASK |
                 IOC_PAD_PAD_CTL_PS_MASK,
  },
  .controller  = HPM_GPIO_CONTROLLER_GPIO0,
  .mode        = HPM_GPIO_OUTPUT,
  .output_high = true,
};

static const struct hpm_gpio_config_s g_flash_wp =
{
  .ioc =
  {
    .domain    = HPM_IOC_DOMAIN_MAIN,
    .pad       = HPM_IOC_PAD_PA30,
    .function  = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0),
    .pad_mask  = HELIXLINK_SPI_PAD_MASK | IOC_PAD_PAD_CTL_PE_MASK |
                 IOC_PAD_PAD_CTL_PS_MASK,
    .pad_value = HELIXLINK_SPI_PAD_VALUE | IOC_PAD_PAD_CTL_PE_MASK |
                 IOC_PAD_PAD_CTL_PS_MASK,
  },
  .controller  = HPM_GPIO_CONTROLLER_GPIO0,
  .mode        = HPM_GPIO_OUTPUT,
  .output_high = true,
};

static const struct hpm_gpio_config_s g_flash_hold =
{
  .ioc =
  {
    .domain    = HPM_IOC_DOMAIN_MAIN,
    .pad       = HPM_IOC_PAD_PA31,
    .function  = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0),
    .pad_mask  = HELIXLINK_SPI_PAD_MASK | IOC_PAD_PAD_CTL_PE_MASK |
                 IOC_PAD_PAD_CTL_PS_MASK,
    .pad_value = HELIXLINK_SPI_PAD_VALUE | IOC_PAD_PAD_CTL_PE_MASK |
                 IOC_PAD_PAD_CTL_PS_MASK,
  },
  .controller  = HPM_GPIO_CONTROLLER_GPIO0,
  .mode        = HPM_GPIO_OUTPUT,
  .output_high = true,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Called once during board late initialization, before any SPI clients. */

int helixlink_spi_initialize(void)
{
  int ret;

  ret = hpm_gpio_config(&g_flash_cs);
  if (ret < 0)
    {
      return ret;
    }

  ret = hpm_gpio_config(&g_flash_wp);
  if (ret < 0)
    {
      return ret;
    }

  ret = hpm_gpio_config(&g_flash_hold);
  if (ret < 0)
    {
      return ret;
    }

  return hpm_spibus_initialize(1) != NULL ? OK : -ENODEV;
}

void hpm_spi1select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  if (devid == SPIDEV_FLASH(0))
    {
      hpm_gpio_write(&g_flash_cs, !selected);
    }
}

uint8_t hpm_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  /* A soldered device is physically present; JEDEC probing belongs to MTD. */

  return devid == SPIDEV_FLASH(0) ? SPI_STATUS_PRESENT : 0;
}

#ifdef CONFIG_SPI_CMDDATA
int hpm_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif
