/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_start.c
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

#include <nuttx/init.h>

#include <arch/board/board.h>

#include "hpm_memorymap.h"
#include "hpm_lowputc.h"
#include "hpm_clockconfig.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __hpm_start
 *
 * Called by the assembly entry after the initial stack is ready.
 *
 ****************************************************************************/

void __hpm_start(void)
{
  const uint32_t *src;
  volatile uint32_t *dest;

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  dest = (volatile uint32_t *)_sbss;
  while (dest < (volatile uint32_t *)_ebss)
    {
      *dest++ = 0;
    }

  /* Copy initialized data from its ILM load address into DLM. */

  src = (const uint32_t *)_eronly;
  dest = (volatile uint32_t *)_sdata;
  while (dest < (volatile uint32_t *)_edata)
    {
      *dest++ = *src++;
    }

  /* Enable baseline resources; preserve the existing CPU/PLL policy. */

  if (hpm_clockconfig() < 0)
    {
      /* Console hardware is not ready yet; stop for debugger inspection. */

      for (; ; )
        {
          __asm__ __volatile__("wfi");
        }
    }

  /* Configure the UART so we can get debug output */

  hpm_lowsetup();

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  /* Do board initialization */

  helixlink_boardinitialize();

  /* Call nx_start() */

  nx_start();

  /* Shouldn't get here */

  for (; ; )
    {
    }
}
