/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_lowputc.c
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
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "hpm_clockconfig.h"
#include "hpm_config.h"
#include "hpm_console.h"
#include "hpm_lowputc.h"
#include "hardware/hpm_uart.h"

#if defined(HAVE_UART_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
#  include "hpm_gpio.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(HAVE_UART_DEVICE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

/****************************************************************************
 * Name: hpm_uart_calculate_baudrate
 *
 * Description:
 *   Find an integer divisor and even oversampling rate within the permitted
 *   baud error. OSCR encodes an oversampling rate of 32 as zero.
 ****************************************************************************/

static bool hpm_uart_calculate_baudrate(uint32_t frequency, uint32_t baud,
                                      uint16_t *divisor, uint8_t *oscr)
{
  uint64_t best_error = UINT64_MAX;
  uint64_t best_target = 1;
  uint32_t best_divisor = 0;
  uint32_t best_rate = 0;
  uint32_t rate;

  if (frequency == 0 || baud < HPM_UART_MINIMUM_BAUDRATE)
    {
      return false;
    }

  for (rate = HPM_UART_OSC_MIN; rate <= HPM_UART_OSC_MAX; rate += 2)
    {
      uint64_t denominator = (uint64_t)baud * rate;
      uint64_t div = (frequency + denominator / 2) / denominator;
      uint64_t target;
      uint64_t error;

      if (div < HPM_UART_BAUDRATE_DIV_MIN || div > HPM_UART_BAUDRATE_DIV_MAX)
        {
          continue;
        }

      target = denominator * div;
      error = frequency > target ? frequency - target : target - frequency;

      if (error * 100 > target * HPM_UART_BAUDRATE_TOLERANCE)
        {
          continue;
        }

      if (best_divisor == 0 || error * best_target < best_error * target)
        {
          best_error = error;
          best_target = target;
          best_divisor = div;
          best_rate = rate;
        }
    }

  if (best_divisor == 0)
    {
      return false;
    }

  *divisor = best_divisor;
  *oscr = best_rate == HPM_UART_OSC_MAX ? 0 : best_rate;
  return true;
}
#endif

#if defined(HAVE_UART_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

/****************************************************************************
 * Name: hpm_console_setup
 *
 * Description:
 *   Configure board-selected pins, then the selected UART clock and format.
 *   CTS controls outbound flow; RTS controls inbound flow. Non-console UARTs
 *   use their lower-half setup path instead of this early entry point.
 ****************************************************************************/

static int hpm_console_setup(void)
{
  struct uart_config_s config =
  {
    .baud      = HPM_CONSOLE_BAUD,
    .parity    = HPM_CONSOLE_PARITY,
    .bits      = HPM_CONSOLE_BITS,
    .stopbits2 = HPM_CONSOLE_2STOP != 0,
#ifdef HPM_CONSOLE_CTS
    .usects    = true,
#endif
#ifdef HPM_CONSOLE_RTS
    .userts    = true,
#endif
  };

  const struct hpm_gpio_config_s *rx_pin = HPM_CONSOLE_RXD;
  const struct hpm_gpio_config_s *tx_pin = HPM_CONSOLE_TXD;
#ifdef HPM_CONSOLE_CTS
  const struct hpm_gpio_config_s *cts_pin = HPM_CONSOLE_CTS;
#endif
#ifdef HPM_CONSOLE_RTS
  const struct hpm_gpio_config_s *rts_pin = HPM_CONSOLE_RTS;
#endif
  int ret;

  if (rx_pin == NULL || tx_pin == NULL ||
      rx_pin->mode != HPM_GPIO_PERIPHERAL ||
      tx_pin->mode != HPM_GPIO_PERIPHERAL)
    {
      return -EINVAL;
    }

#ifdef HPM_CONSOLE_CTS
  if (cts_pin == NULL ||
      cts_pin->mode != HPM_GPIO_PERIPHERAL)
    {
      return -EINVAL;
    }
#endif

#ifdef HPM_CONSOLE_RTS
  if (rts_pin == NULL ||
      (rts_pin->mode != HPM_GPIO_PERIPHERAL &&
       rts_pin->mode != HPM_GPIO_OUTPUT))
    {
      return -EINVAL;
    }
#endif

  ret = hpm_gpio_config(HPM_CONSOLE_RXD);
  if (ret < 0)
    {
      return ret;
    }

  ret = hpm_gpio_config(HPM_CONSOLE_TXD);
  if (ret < 0)
    {
      return ret;
    }

#ifdef HPM_CONSOLE_CTS
  ret = hpm_gpio_config(HPM_CONSOLE_CTS);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef HPM_CONSOLE_RTS
  ret = hpm_gpio_config(HPM_CONSOLE_RTS);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef HPM_CONSOLE_RTS
  if (rts_pin->mode == HPM_GPIO_OUTPUT)
    {
      ret = hpm_gpio_write(HPM_CONSOLE_RTS, false);
      if (ret < 0)
        {
          return ret;
        }

      config.userts = false;
    }
#endif

  config.frequency = hpm_uart_clockconfig(HPM_CONSOLE_BASE);
  if (config.frequency == 0)
    {
      return -EINVAL;
    }

  return hpm_uart_configure(HPM_CONSOLE_BASE, &config);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_lowsetup
 *
 * Description:
 *   Prepare the configured serial console for early polled output. This
 *   function neither registers serial devices nor initializes other UARTs.
 *   CONFIG_SUPPRESS_UART_CONFIG preserves the bootloader's UART settings.
 ****************************************************************************/

void hpm_lowsetup(void)
{
#if defined(HAVE_UART_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  DEBUGVERIFY(hpm_console_setup());
#endif
}

/****************************************************************************
 * Name: hpm_uart_configure
 *
 * Description:
 *   Configure a UART using the caller-provided input frequency and format.
 *   With RTS alone the pin is asserted manually; the serial lower half must
 *   manage receive backpressure. AFE enables auto-CTS and, when RTS is set,
 *   auto-RTS. RS485 direction switching and inverted RTS return -ENOTSUP.
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
int hpm_uart_configure(uint32_t base, const struct uart_config_s *config)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  uint16_t div;
  uint8_t osc;
  uint32_t lcr;
  uint32_t mcr = 0;

  if (config == NULL || config->bits < 5 || config->bits > 8 ||
      config->parity > 2)
    {
      return -EINVAL;
    }

  if (config->users485 || config->invrts)
    {
      return -ENOTSUP;
    }

  if (!hpm_uart_calculate_baudrate(config->frequency, config->baud,
                                    &div, &osc))
    {
      return -EINVAL;
    }

  lcr = config->bits - 5;
  if (config->stopbits2)
    {
      lcr |= UART_LCR_STB;
    }

  if (config->parity != 0)
    {
      lcr |= UART_LCR_PEN;
      if (config->parity == 2)
        {
          lcr |= UART_LCR_EPS;
        }
    }

  /* Clear DLAB before accessing IER: IER and DLM share an address. */

  putreg32(lcr, base + HPM_UART_LCR_OFFSET);
  putreg32(0, base + HPM_UART_IER_OFFSET);
  putreg32(lcr | UART_LCR_DLAB, base + HPM_UART_LCR_OFFSET);
  putreg32(osc, base + HPM_UART_OSCR_OFFSET);
  putreg32(div & 0xff, base + HPM_UART_DLL_OFFSET);
  putreg32(div >> 8, base + HPM_UART_DLM_OFFSET);
  putreg32(lcr, base + HPM_UART_LCR_OFFSET);

  if (config->usects)
    {
      mcr |= UART_MCR_AFE;
    }

  if (config->userts)
    {
      mcr |= UART_MCR_RTS;
    }

  /* Enable FIFO operation for automatic CTS/RTS without clearing data. */

  putreg32(UART_FCR_FIFOE, base + HPM_UART_FCR_OFFSET);

  /* Do not alter unrelated modem-control bits such as loopback. */

  modifyreg32(base + HPM_UART_MCR_OFFSET,
              UART_MCR_AFE | UART_MCR_RTS, mcr);
#endif
  return OK;
}
#endif

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Output one character without buffering or newline translation.
 ****************************************************************************/

void riscv_lowputc(char ch)
{
#ifdef HAVE_UART_CONSOLE
  while ((getreg32(HPM_CONSOLE_BASE + HPM_UART_LSR_OFFSET) &
          UART_LSR_THRE) == 0)
    {
    }

  putreg32((uint8_t)ch, HPM_CONSOLE_BASE + HPM_UART_THR_OFFSET);
#endif
}

/****************************************************************************
 * Name: hpm_lowputc
 *
 * Description:
 *   Output a console character, adding a carriage return before a newline.
 ****************************************************************************/

#ifdef HAVE_UART_DEVICE
void hpm_lowputc(int ch)
{
#ifdef HAVE_UART_CONSOLE
  if (ch == '\n')
    {
      riscv_lowputc('\r');
    }

  riscv_lowputc((char)ch);
#endif
}
#endif
