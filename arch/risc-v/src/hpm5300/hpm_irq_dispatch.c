/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_irq_dispatch.c
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

#include <stdint.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "riscv_internal.h"
#include "chip.h"
#include "hpm.h"
#include "hardware/hpm_plic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * riscv_dispatch_irq
 ****************************************************************************/

void *riscv_dispatch_irq(uintreg_t vector, uintreg_t *regs)
{
  uint32_t claim = 0;
  uintreg_t interrupt = (uintreg_t)1 << 31;
  int irq = (int)(vector & ~interrupt);

  if ((vector & interrupt) != 0)
    {
      irq += RISCV_IRQ_ASYNC;
    }

  /* Firstly, check if the irq is machine external interrupt */

  if (RISCV_IRQ_MEXT == irq)
    {
      claim = getreg32(HPM_PLIC_CLAIM);

      /* No interrupt source was claimed */

      if (claim == 0)
        {
          return regs;
        }

      /* Add the value to nuttx irq which is offset to the mext */

      irq = claim + HPM_IRQ_PERI_START;
    }

  /* Acknowledge and dispatch the interrupt or exception */

  riscv_ack_irq(irq);
  regs = riscv_doirq(irq, regs);

  /* Complete the claimed interrupt */

  if (claim != 0)
    {
      putreg32(claim, HPM_PLIC_CLAIM);
    }

  return regs;
}
