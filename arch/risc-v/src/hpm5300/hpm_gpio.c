/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_gpio.c
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

#include <nuttx/irq.h>

#include "riscv_internal.h"
#include "hpm_gpio.h"
#include "hpm_ioc.h"
#include "hardware/hpm_gpio.h"
#include "hardware/hpm_gpiom.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_gpio_pinmap
 *
 * Description:
 *    Validate the IOC descriptor and translate its pad index into a hardware
 *    port index and a pin mask. This function does not access registers.
 *
 ****************************************************************************/

static int hpm_gpio_pinmap(const struct hpm_gpio_config_s *config,
                          unsigned int *port, uint32_t *mask)
{
  int ret;

  if (config == NULL || port == NULL || mask == NULL)
    {
      return -EINVAL;
    }

  ret = hpm_ioc_validate(&config->ioc);
  if (ret < 0)
    {
      return ret;
    }

  *port = config->ioc.pad / 32;
  *mask = 1U << (config->ioc.pad % 32);

  return OK;
}

/****************************************************************************
 * Name: hpm_gpio_controller_base
 *
 * Description:
 *   Check the controller/domain combination and resolve the controller
 *   base. This function does not change ownership or access registers.
 *
 ****************************************************************************/

static int hpm_gpio_controller_base(const struct hpm_gpio_config_s *config,
                                    uintptr_t *base)
{
  uintptr_t selected;

  if (config == NULL || base == NULL)
    {
      return -EINVAL;
    }

  switch (config->controller)
    {
      case HPM_GPIO_CONTROLLER_GPIO0:
      case HPM_GPIO_CONTROLLER_FGPIO:
        if (config->ioc.domain != HPM_IOC_DOMAIN_MAIN)
          {
            return -EINVAL;
          }

        selected = config->controller == HPM_GPIO_CONTROLLER_GPIO0 ?
                   HPM_GPIO0_BASE : HPM_FGPIO_BASE;
        break;

      case HPM_GPIO_CONTROLLER_PGPIO:
        if (config->ioc.domain != HPM_IOC_DOMAIN_PMIC)
          {
            return -EINVAL;
          }

        selected = HPM_PGPIO_BASE;
        break;

      default:
        return -EINVAL;
    }

  *base = selected;
  return OK;
}

/****************************************************************************
 * Name: hpm_gpio_assign
 *
 * Description:
 *    Assign a main-domain pin to GPIO0 or FGPIO. PGPIO selection is handled
 *    by PIOC and does not use GPIOM.
 *
 ****************************************************************************/

static int hpm_gpio_assign(unsigned int port, unsigned int pin,
                           enum hpm_gpio_controller_e controller)
{
  uintptr_t address;
  uint32_t selected;
  uint32_t value;
  irqstate_t flags;
  int ret = OK;

  if (port > HPM_GPIO_PORT_Y || pin >= 32 ||
      !HPM_IOC_PAD_VALID(port * 32 + pin))
    {
      return -EINVAL;
    }

  switch (controller)
    {
      case HPM_GPIO_CONTROLLER_GPIO0:
        selected = HPM_GPIOM_SELECT_GPIO0;
        break;

      case HPM_GPIO_CONTROLLER_FGPIO:
        selected = HPM_GPIOM_SELECT_FGPIO;
        break;

      case HPM_GPIO_CONTROLLER_PGPIO:
        return HPM_PIOC_PAD_VALID(port * 32 + pin) ? OK : -EINVAL;

      default:
        return -EINVAL;
    }

  address = HPM_GPIOM_ASSIGN(port, pin);
  flags = enter_critical_section();
  value = getreg32(address);

  if ((value & HPM_GPIOM_SELECT_MASK) != selected)
    {
      if ((value & HPM_GPIOM_LOCK) != 0)
        {
          ret = -EBUSY;
        }
      else
        {
          value = (value & ~HPM_GPIOM_SELECT_MASK) | selected;
          putreg32(value, address);

          if ((getreg32(address) & HPM_GPIOM_SELECT_MASK) != selected)
            {
              ret = -EIO;
            }
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: hpm_gpio_resolve
 *
 * Description:
 *    Validate a GPIO descriptor and resolve its register addressing.
 *    No registers are accessed. Outputs are updated only on success.
 *
 ****************************************************************************/

static int hpm_gpio_resolve(const struct hpm_gpio_config_s *config,
                           uintptr_t *base, unsigned int *port,
                           uint32_t *mask)
{
  uintptr_t selected_base;
  unsigned int selected_port;
  uint32_t selected_mask;
  int ret;

  if (base == NULL || port == NULL || mask == NULL)
    {
      return -EINVAL;
    }

  ret = hpm_gpio_pinmap(config, &selected_port, &selected_mask);
  if (ret < 0)
    {
      return ret;
    }

  ret = hpm_gpio_controller_base(config, &selected_base);
  if (ret < 0)
    {
      return ret;
    }

  switch (config->mode)
    {
      case HPM_GPIO_INPUT:
      case HPM_GPIO_OUTPUT:
        if ((config->ioc.function &
             (IOC_PAD_FUNC_CTL_ALT_SELECT_MASK |
              IOC_PAD_FUNC_CTL_ANALOG_MASK)) != 0)
          {
            return -EINVAL;
          }
        break;

      case HPM_GPIO_PERIPHERAL:
        break;

      default:
        return -EINVAL;
    }

  *base = selected_base;
  *port = selected_port;
  *mask = selected_mask;
  return OK;
}

/****************************************************************************
 * Name: hpm_gpio_config
 *
 * Description:
 *    Configure a caller-owned pad. Peripheral users must stop their output
 *    before remapping. On failure after hardware updates begin, GPIO outputs
 *    remain disabled; previous configuration is not restored.
 *
 ****************************************************************************/

int hpm_gpio_config(const struct hpm_gpio_config_s *config)
{
  uintptr_t base;
  unsigned int port;
  unsigned int pin;
  uint32_t mask;
  uint32_t selected;
  uint32_t value;
  irqstate_t flags;
  int ret;

  ret = hpm_gpio_resolve(config, &base, &port, &mask);
  if (ret < 0)
    {
      return ret;
    }

  pin = config->ioc.pad % 32;
  flags = enter_critical_section();

  /* Reject a locked ownership conflict before changing any outputs. */

  if (config->mode != HPM_GPIO_PERIPHERAL &&
      config->ioc.domain == HPM_IOC_DOMAIN_MAIN)
    {
      selected = config->controller == HPM_GPIO_CONTROLLER_GPIO0 ?
                 HPM_GPIOM_SELECT_GPIO0 : HPM_GPIOM_SELECT_FGPIO;

      value = getreg32(HPM_GPIOM_ASSIGN(port, pin));

      if ((value & HPM_GPIOM_LOCK) != 0 &&
          (value & HPM_GPIOM_SELECT_MASK) != selected)
        {
          ret = -EBUSY;
          goto out;
        }
    }

  /* Disable GPIO outputs that can reach this physical pad. */

  putreg32(mask, HPM_GPIO_OE_CLEAR(HPM_GPIO0_BASE, port));
  putreg32(mask, HPM_GPIO_OE_CLEAR(HPM_FGPIO_BASE, port));

  if (HPM_PIOC_PAD_VALID(config->ioc.pad))
    {
      putreg32(mask, HPM_GPIO_OE_CLEAR(HPM_PGPIO_BASE, port));
    }

  /* Prepare the output latch before enabling the output driver. */

  if (config->mode == HPM_GPIO_OUTPUT)
    {
      if (config->output_high)
        {
          putreg32(mask, HPM_GPIO_DO_SET(base, port));
        }
      else
        {
          putreg32(mask, HPM_GPIO_DO_CLEAR(base, port));
        }
    }

  if (config->mode != HPM_GPIO_PERIPHERAL)
    {
      ret = hpm_gpio_assign(port, pin, config->controller);
      if (ret < 0)
        {
          goto out;
        }
    }

  ret = hpm_ioc_configure(&config->ioc);
  if (ret < 0)
    {
      goto out;
    }

  /* Main-domain PY functions pass through the PMIC-domain mux. */

  if (config->ioc.domain == HPM_IOC_DOMAIN_MAIN &&
      HPM_PIOC_PAD_VALID(config->ioc.pad))
    {
      struct hpm_ioc_config_s route =
      {
        .domain   = HPM_IOC_DOMAIN_PMIC,
        .pad      = config->ioc.pad,
        .function = PIOC_PY00_FUNC_CTL_SOC_GPIO_Y_00,
      };

      ret = hpm_ioc_configure(&route);
      if (ret < 0)
        {
          goto out;
        }
    }

  if (config->mode == HPM_GPIO_OUTPUT)
    {
      putreg32(mask, HPM_GPIO_OE_SET(base, port));
    }

out:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: hpm_gpio_write
 *
 * Description:
 *   Update the output latch of a configured GPIO output.
 *   The caller must retain ownership of the configured pad.
 *
 ****************************************************************************/

int hpm_gpio_write(const struct hpm_gpio_config_s *config, bool value)
{
  uintptr_t base;
  unsigned int port;
  uint32_t mask;
  int ret;

  ret = hpm_gpio_resolve(config, &base, &port, &mask);
  if (ret < 0)
    {
      return ret;
    }

  if (config->mode != HPM_GPIO_OUTPUT)
    {
      return -EPERM;
    }

  if (value)
    {
      putreg32(mask, HPM_GPIO_DO_SET(base, port));
    }
  else
    {
      putreg32(mask, HPM_GPIO_DO_CLEAR(base, port));
    }

  return OK;
}

/****************************************************************************
 * Name: hpm_gpio_read
 *
 * Description:
 *   Read the GPIO input sample, not the output latch.
 *   The caller must retain ownership and ensure input visibility.
 *   Output-pad sampling may require IOC loopback to be enabled.
 *
 ****************************************************************************/

int hpm_gpio_read(const struct hpm_gpio_config_s *config, bool *value)
{
  uintptr_t base;
  unsigned int port;
  uint32_t mask;
  int ret;

  if (value == NULL)
    {
      return -EINVAL;
    }

  ret = hpm_gpio_resolve(config, &base, &port, &mask);
  if (ret < 0)
    {
      return ret;
    }

  if (config->mode == HPM_GPIO_PERIPHERAL)
    {
      return -EPERM;
    }

  *value = (getreg32(HPM_GPIO_DI(base, port)) & mask) != 0;
  return OK;
}
