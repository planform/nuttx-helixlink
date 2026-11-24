/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_clockconfig.c
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
#include <stddef.h>
#include <stdint.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "hpm_clockconfig.h"
#include "hardware/hpm_sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Software polling budget, not a time interval or hardware parameter. */

#define HPM_CLOCK_POLL_LIMIT    100000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hpm_uart_clock_map_s
{
  uint32_t base;
  unsigned int node;
  unsigned int resource;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct hpm_uart_clock_map_s g_uart_clock_map[] =
{
  {
    HPM_UART0_BASE,
    HPM_SYSCTL_CLOCK_CLK_TOP_URT0,
    HPM_SYSCTL_RESOURCE_URT0
  },
  {
    HPM_UART1_BASE,
    HPM_SYSCTL_CLOCK_CLK_TOP_URT1,
    HPM_SYSCTL_RESOURCE_URT1
  },
  {
    HPM_UART2_BASE,
    HPM_SYSCTL_CLOCK_CLK_TOP_URT2,
    HPM_SYSCTL_RESOURCE_URT2
  },
  {
    HPM_UART3_BASE,
    HPM_SYSCTL_CLOCK_CLK_TOP_URT3,
    HPM_SYSCTL_RESOURCE_URT3
  },
  {
    HPM_UART4_BASE,
    HPM_SYSCTL_CLOCK_CLK_TOP_URT4,
    HPM_SYSCTL_RESOURCE_URT4
  },
  {
    HPM_UART5_BASE,
    HPM_SYSCTL_CLOCK_CLK_TOP_URT5,
    HPM_SYSCTL_RESOURCE_URT5
  },
  {
    HPM_UART6_BASE,
    HPM_SYSCTL_CLOCK_CLK_TOP_URT6,
    HPM_SYSCTL_RESOURCE_URT6
  },
  {
    HPM_UART7_BASE,
    HPM_SYSCTL_CLOCK_CLK_TOP_URT7,
    HPM_SYSCTL_RESOURCE_URT7
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const struct hpm_uart_clock_map_s *
hpm_uart_clock_lookup(uint32_t base)
{
  size_t i;

  for (i = 0; i < sizeof(g_uart_clock_map) /
                  sizeof(g_uart_clock_map[0]); i++)
    {
      if (g_uart_clock_map[i].base == base)
        {
          return &g_uart_clock_map[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: hpm_clock_wait_ready
 *
 * Description:
 *   Wait until the specified busy bits clear. This bounded polling helper
 *   does not depend on the system timer or scheduler.
 *
 ****************************************************************************/

static int hpm_clock_wait_ready(uintptr_t address, uint32_t busy_mask)
{
  unsigned int count;

  for (count = 0; count < HPM_CLOCK_POLL_LIMIT; count++)
    {
      if ((getreg32(address) & busy_mask) == 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: hpm_clock_group0_enable
 *
 * Description:
 *   Add a linkable resource to Group0, connect Group0 to CPU0, and wait
 *   for the resource update to complete. Existing group bits are preserved.
 *
 ****************************************************************************/

static int hpm_clock_group0_enable(unsigned int resource)
{
  unsigned int relative;
  unsigned int index;
  uint32_t mask;
  uint32_t busy;
  int ret;

  if (resource < HPM_SYSCTL_RESOURCE_AHB0 ||
      resource >= HPM_SYSCTL_RESOURCE_COUNT)
    {
      return -EINVAL;
    }

  relative = resource - HPM_SYSCTL_RESOURCE_AHB0;
  index = relative / 32;
  mask = 1U << (relative % 32);

  if (index >= HPM_SYSCTL_GROUP0_COUNT)
    {
      return -EINVAL;
    }

  busy = HPM_SYSCTL_RESOURCE_LOC_BUSY_MASK |
         HPM_SYSCTL_RESOURCE_GLB_BUSY_MASK;

  ret = hpm_clock_wait_ready(HPM_SYSCTL_RESOURCE(resource), busy);
  if (ret < 0)
    {
      return ret;
    }

  /* Connect Group0 to CPU0, then add the requested resource. */

  putreg32(1U, HPM_SYSCTL_AFFILIATE_SET(HPM_SYSCTL_AFFILIATE_CPU0));
  putreg32(mask, HPM_SYSCTL_GROUP0_SET(index));

  if ((getreg32(HPM_SYSCTL_AFFILIATE_VALUE(
                 HPM_SYSCTL_AFFILIATE_CPU0)) & 1U) == 0 ||
      (getreg32(HPM_SYSCTL_GROUP0_VALUE(index)) & mask) == 0)
    {
      return -EIO;
    }

  return hpm_clock_wait_ready(HPM_SYSCTL_RESOURCE(resource), busy);
}

/****************************************************************************
 * Name: hpm_dma_clockconfig
 *
 * Description:
 *   Enable the HDMA/DMAMUX bus resource without changing CPU/PLL clocks.
 *
 ****************************************************************************/

int hpm_dma_clockconfig(void)
{
  return hpm_clock_group0_enable(HPM_SYSCTL_RESOURCE_HDMA);
}

/****************************************************************************
 * Name: hpm_clock_node_configure
 *
 * Description:
 *   Configure a CLOCK[] node. The source must already be available.
 *   divider is the actual division ratio, not the register encoding.
 *
 ****************************************************************************/

static int hpm_clock_node_configure(unsigned int node,
                                    unsigned int source,
                                    unsigned int divider)
{
  uintptr_t address;
  uint32_t fields;
  uint32_t value;
  uint32_t busy;
  int ret;

  if (node >= HPM_SYSCTL_CLOCK_COUNT ||
      source > (HPM_SYSCTL_CLOCK_MUX_MASK >>
                HPM_SYSCTL_CLOCK_MUX_SHIFT) ||
      divider == 0 ||
      divider > (HPM_SYSCTL_CLOCK_DIV_MASK >>
                 HPM_SYSCTL_CLOCK_DIV_SHIFT) + 1U)
    {
      return -EINVAL;
    }

  address = HPM_SYSCTL_CLOCK(node);
  fields = HPM_SYSCTL_CLOCK_MUX_MASK | HPM_SYSCTL_CLOCK_DIV_MASK;
  busy = HPM_SYSCTL_CLOCK_LOC_BUSY_MASK |
         HPM_SYSCTL_CLOCK_GLB_BUSY_MASK;

  ret = hpm_clock_wait_ready(address, busy);
  if (ret < 0)
    {
      return ret;
    }

  value = HPM_SYSCTL_CLOCK_MUX(source) |
          HPM_SYSCTL_CLOCK_DIV(divider - 1U);

  modifyreg32(address, fields, value);

  ret = hpm_clock_wait_ready(address, busy);
  if (ret < 0)
    {
      return ret;
    }

  if ((getreg32(address) & fields) != value)
    {
      return -EIO;
    }

  return OK;
}

static const struct hpm_clock_config_s *
hpm_uart_clock_policy(uint32_t base)
{
  switch (base)
    {
#ifdef CONFIG_HPM_UART0
      case HPM_UART0_BASE:
        return BOARD_UART0_CLOCK_CONFIG;
#endif
#ifdef CONFIG_HPM_UART1
      case HPM_UART1_BASE:
        return BOARD_UART1_CLOCK_CONFIG;
#endif
#ifdef CONFIG_HPM_UART2
      case HPM_UART2_BASE:
        return BOARD_UART2_CLOCK_CONFIG;
#endif
#ifdef CONFIG_HPM_UART3
      case HPM_UART3_BASE:
        return BOARD_UART3_CLOCK_CONFIG;
#endif
#ifdef CONFIG_HPM_UART4
      case HPM_UART4_BASE:
        return BOARD_UART4_CLOCK_CONFIG;
#endif
#ifdef CONFIG_HPM_UART5
      case HPM_UART5_BASE:
        return BOARD_UART5_CLOCK_CONFIG;
#endif
#ifdef CONFIG_HPM_UART6
      case HPM_UART6_BASE:
        return BOARD_UART6_CLOCK_CONFIG;
#endif
#ifdef CONFIG_HPM_UART7
      case HPM_UART7_BASE:
        return BOARD_UART7_CLOCK_CONFIG;
#endif
      default:
        return NULL;
    }
}

/****************************************************************************
 * Name: hpm_clock_apply
 ****************************************************************************/

static uint32_t hpm_clock_apply(unsigned int node, unsigned int resource,
                                const struct hpm_clock_config_s *config)
{
  uint32_t frequency;

  if (config == NULL || config->source_frequency == 0 ||
      config->divider == 0)
    {
      return 0;
    }

  frequency = config->source_frequency / config->divider;
  if (frequency == 0 ||
      hpm_clock_node_configure(node, config->source, config->divider) < 0 ||
      hpm_clock_group0_enable(resource) < 0)
    {
      return 0;
    }

  return frequency;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_clockconfig
 *
 * Description:
 *   Enable baseline bus and GPIO resources without changing CPU/PLL clocks.
 *   CLK_24M selects XTAL/RC automatically (HPM5300UM Rev1.2, section 10.2).
 ****************************************************************************/

int hpm_clockconfig(void)
{
  int ret = hpm_clock_group0_enable(HPM_SYSCTL_RESOURCE_AHB0);

  if (ret < 0)
    {
      return ret;
    }

  return hpm_clock_group0_enable(HPM_SYSCTL_RESOURCE_GPIO);
}

/****************************************************************************
 * Name: hpm_uart_clockconfig
 ****************************************************************************/

uint32_t hpm_uart_clockconfig(uint32_t base)
{
  const struct hpm_uart_clock_map_s *map = hpm_uart_clock_lookup(base);

  if (map == NULL)
    {
      return 0;
    }

  return hpm_clock_apply(map->node, map->resource,
                         hpm_uart_clock_policy(base));
}

/****************************************************************************
 * Name: hpm_mchtmr_clockconfig
 ****************************************************************************/

uint32_t hpm_mchtmr_clockconfig(void)
{
  return hpm_clock_apply(HPM_SYSCTL_CLOCK_CLK_TOP_MCT0,
                         HPM_SYSCTL_RESOURCE_MCT0,
                         BOARD_MCHTMR_CLOCK_CONFIG);
}

/****************************************************************************
 * Name: hpm_spi_clockconfig
 ****************************************************************************/

#ifdef CONFIG_HPM_SPI
uint32_t hpm_spi_clockconfig(uint32_t base)
{
  switch (base)
    {
#ifdef CONFIG_HPM_SPI0
      case HPM_SPI0_BASE:
        return hpm_clock_apply(HPM_SYSCTL_CLOCK_CLK_TOP_SPI0,
                               HPM_SYSCTL_RESOURCE_SPI0,
                               BOARD_SPI0_CLOCK_CONFIG);
#endif
#ifdef CONFIG_HPM_SPI1
      case HPM_SPI1_BASE:
        return hpm_clock_apply(HPM_SYSCTL_CLOCK_CLK_TOP_SPI1,
                               HPM_SYSCTL_RESOURCE_SPI1,
                               BOARD_SPI1_CLOCK_CONFIG);
#endif
#ifdef CONFIG_HPM_SPI2
      case HPM_SPI2_BASE:
        return hpm_clock_apply(HPM_SYSCTL_CLOCK_CLK_TOP_SPI2,
                               HPM_SYSCTL_RESOURCE_SPI2,
                               BOARD_SPI2_CLOCK_CONFIG);
#endif
#ifdef CONFIG_HPM_SPI3
      case HPM_SPI3_BASE:
        return hpm_clock_apply(HPM_SYSCTL_CLOCK_CLK_TOP_SPI3,
                               HPM_SYSCTL_RESOURCE_SPI3,
                               BOARD_SPI3_CLOCK_CONFIG);
#endif
      default:
        return 0;
    }
}
#endif
