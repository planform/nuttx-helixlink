/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_ioc.c
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

#include <nuttx/irq.h>

#include "riscv_internal.h"
#include "hpm_ioc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_ioc_validate
 ****************************************************************************/

int hpm_ioc_validate(const struct hpm_ioc_config_s *config)
{
  if (config == NULL)
    {
      return -EINVAL;
    }

  switch (config->domain)
    {
      case HPM_IOC_DOMAIN_MAIN:
        if (!HPM_IOC_PAD_VALID(config->pad))
          {
            return -EINVAL;
          }
        break;

      case HPM_IOC_DOMAIN_PMIC:
        if (!HPM_PIOC_PAD_VALID(config->pad))
          {
            return -EINVAL;
          }
        break;

      default:
        return -EINVAL;
    }

  if ((config->function & ~HPM_IOC_FUNCTION_MASK) != 0 ||
      (config->pad_mask & ~HPM_IOC_PAD_CONTROL_MASK) != 0 ||
      (config->pad_value & ~config->pad_mask) != 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: hpm_ioc_configure
 ****************************************************************************/

int hpm_ioc_configure(const struct hpm_ioc_config_s *config)
{
  uintptr_t base;
  uintptr_t offset;
  irqstate_t flags;
  int ret;

  ret = hpm_ioc_validate(config);
  if (ret < 0)
    {
      return ret;
    }

  base = config->domain == HPM_IOC_DOMAIN_MAIN ?
         HPM_IOC_BASE : HPM_PIOC_BASE;
  offset = HPM_IOC_PAD_FUNC_CTL_OFFSET(config->pad);

  /* Apply electrical settings before switching to the selected function. */

  flags = enter_critical_section();

  if (config->pad_mask != 0)
    {
      modifyreg32(base + HPM_IOC_PAD_PAD_CTL_OFFSET(config->pad),
                  config->pad_mask, config->pad_value);
    }

  modifyreg32(base + offset, HPM_IOC_FUNCTION_MASK, config->function);
  leave_critical_section(flags);
  return OK;
}
