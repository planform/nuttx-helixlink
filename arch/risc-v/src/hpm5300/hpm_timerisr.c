/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_timerisr.c
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

#include <assert.h>
#include <stdint.h>

#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/oneshot.h>

#include "riscv_internal.h"
#include "riscv_mtimer.h"
#include "hpm_clockconfig.h"
#include "hardware/hpm_mchtmr.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Bind MCHTMR to NuttX's common RISC-V oneshot/alarm implementation.
 *   The common driver handles safe RV32 accesses to the 64-bit timer.
 ****************************************************************************/

void up_timer_initialize(void)
{
  struct oneshot_lowerhalf_s *lower;
  uint32_t frequency = hpm_mchtmr_clockconfig();

  if (frequency == 0)
    {
      PANIC();
    }

  lower = riscv_mtimer_initialize(HPM_MCHTMR_MTIME,
                                  HPM_MCHTMR_MTIMECMP,
                                  RISCV_IRQ_MTIMER, frequency);
  if (lower == NULL)
    {
      PANIC();
    }

  up_alarm_set_lowerhalf(lower);
}
