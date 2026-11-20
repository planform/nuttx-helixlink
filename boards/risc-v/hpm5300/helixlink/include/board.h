/****************************************************************************
 * boards/risc-v/hpm5300/helixlink/include/board.h
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

#ifndef __BOARDS_RISCV_HPM5300_HELIXLINK_INCLUDE_BOARD_H
#define __BOARDS_RISCV_HPM5300_HELIXLINK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Nominal CLK_24M frequency; XTAL/RC selection is automatic in hardware. */

#define BOARD_CLK24M_FREQUENCY  24000000

/* LED definitions **********************************************************/

/* There are four LED status indicators located on the EVK Board.
 * The functions of these LEDs include:
 *
 *   - Main Power Supply(D3)
 *     Green: DC 5V main supply is normal.
 *     Red:   J2 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset RED LED(D15)
 *   - OpenSDA LED(D16)
 *   - USER LED(D18)
 *
 * Only a single LED, D18, is under software control.
 */

/* LED index values for use with board_userled() */

#if 0
#define BOARD_USERLED     0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_USERLED_BIT (1 << BOARD_USERLED)

/* This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/imxrt_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   -------------------- ----------------------------- ------
 *   SYMBOL                   Meaning                   LED
 *   -------------------- ----------------------------- ------
 */

#define LED_STARTED       0  /* NuttX has been started  OFF    */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated OFF    */
#define LED_IRQSENABLED   0  /* Interrupts enabled      OFF    */
#define LED_STACKCREATED  1  /* Idle stack created      ON     */
#define LED_INIRQ         2  /* In an interrupt         N/C    */
#define LED_SIGNAL        2  /* In a signal handler     N/C    */
#define LED_ASSERTION     2  /* An assertion failed     N/C    */
#define LED_PANIC         3  /* The system has crashed  FLASH  */
#undef  LED_IDLE             /* Not used                       */

/* Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* Button definitions *******************************************************/

/* The IMXRT board has one external user button
 *
 * 1. SW8 (IRQ88)   GPIO5-00
 */

#define BUTTON_SW8        0
#define NUM_BUTTONS       3

#define BUTTON_SW8_BIT    (1 << BUTTON_SW8)

/* UARTs */

#define GPIO_UART0_RXD  (GPIO_UART0_RXD4 | IOC_PAD_UART_DEFAULT)
#define GPIO_UART0_TXD  (GPIO_UART0_TXD4 | IOC_PAD_UART_DEFAULT)

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct hpm_clock_config_s;

EXTERN const struct hpm_clock_config_s g_helixlink_mchtmr_clock;

#define BOARD_MCHTMR_CLOCK_CONFIG  (&g_helixlink_mchtmr_clock)

#ifdef CONFIG_HPM_UART3

struct hpm_gpio_config_s;

EXTERN const struct hpm_clock_config_s g_helixlink_uart3_clock;
EXTERN const struct hpm_gpio_config_s g_helixlink_uart3_rx;
EXTERN const struct hpm_gpio_config_s g_helixlink_uart3_tx;

#define BOARD_UART3_CLOCK_CONFIG  (&g_helixlink_uart3_clock)
#define GPIO_UART3_RXD  (&g_helixlink_uart3_rx)
#define GPIO_UART3_TXD  (&g_helixlink_uart3_tx)

#endif /* CONFIG_HPM_UART3 */

#ifdef CONFIG_HPM_SPI1

struct hpm_gpio_config_s;

EXTERN const struct hpm_clock_config_s g_helixlink_spi1_clock;
EXTERN const struct hpm_gpio_config_s g_helixlink_spi1_sclk;
EXTERN const struct hpm_gpio_config_s g_helixlink_spi1_miso;
EXTERN const struct hpm_gpio_config_s g_helixlink_spi1_mosi;

#define BOARD_SPI1_CLOCK_CONFIG  (&g_helixlink_spi1_clock)
#define GPIO_SPI1_SCLK           (&g_helixlink_spi1_sclk)
#define GPIO_SPI1_MISO           (&g_helixlink_spi1_miso)
#define GPIO_SPI1_MOSI           (&g_helixlink_spi1_mosi)

int helixlink_spi_initialize(void);

#endif /* CONFIG_HPM_SPI1 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: helixlink_boardinitialize
 ****************************************************************************/

void helixlink_boardinitialize(void);

#ifdef CONFIG_HELIXLINK_SPIFLASH
int helixlink_flash_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_HPM5300_HELIXLINK_INCLUDE_BOARD_H */
