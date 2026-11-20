/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_console.h
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

#ifndef __ARCH_RISCV_SRC_HPM5300_HPM_CONSOLE_H
#define __ARCH_RISCV_SRC_HPM5300_HPM_CONSOLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hpm_config.h"
#include "hardware/hpm_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Only the selected console may reference board pin definitions. A board
 * supplies GPIO_UARTn_RXD/TXD and optional CTS/RTS when flow control
 * is selected. Enabling another UART does not configure its pins here.
 */

#ifdef HAVE_UART_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE     HPM_UART0_BASE
#    define HPM_CONSOLE_BAUD     CONFIG_UART0_BAUD
#    define HPM_CONSOLE_BITS     CONFIG_UART0_BITS
#    define HPM_CONSOLE_PARITY   CONFIG_UART0_PARITY
#    define HPM_CONSOLE_2STOP    CONFIG_UART0_2STOP
#    define HPM_CONSOLE_RXD      GPIO_UART0_RXD
#    define HPM_CONSOLE_TXD      GPIO_UART0_TXD
#    ifdef CONFIG_UART0_OFLOWCONTROL
#      define HPM_CONSOLE_CTS GPIO_UART0_CTS
#    endif
#    ifdef CONFIG_UART0_IFLOWCONTROL
#      define HPM_CONSOLE_RTS GPIO_UART0_RTS
#    endif
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE     HPM_UART1_BASE
#    define HPM_CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define HPM_CONSOLE_BITS     CONFIG_UART1_BITS
#    define HPM_CONSOLE_PARITY   CONFIG_UART1_PARITY
#    define HPM_CONSOLE_2STOP    CONFIG_UART1_2STOP
#    define HPM_CONSOLE_RXD      GPIO_UART1_RXD
#    define HPM_CONSOLE_TXD      GPIO_UART1_TXD
#    ifdef CONFIG_UART1_OFLOWCONTROL
#      define HPM_CONSOLE_CTS GPIO_UART1_CTS
#    endif
#    ifdef CONFIG_UART1_IFLOWCONTROL
#      define HPM_CONSOLE_RTS GPIO_UART1_RTS
#    endif
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE     HPM_UART2_BASE
#    define HPM_CONSOLE_BAUD     CONFIG_UART2_BAUD
#    define HPM_CONSOLE_BITS     CONFIG_UART2_BITS
#    define HPM_CONSOLE_PARITY   CONFIG_UART2_PARITY
#    define HPM_CONSOLE_2STOP    CONFIG_UART2_2STOP
#    define HPM_CONSOLE_RXD      GPIO_UART2_RXD
#    define HPM_CONSOLE_TXD      GPIO_UART2_TXD
#    ifdef CONFIG_UART2_OFLOWCONTROL
#      define HPM_CONSOLE_CTS GPIO_UART2_CTS
#    endif
#    ifdef CONFIG_UART2_IFLOWCONTROL
#      define HPM_CONSOLE_RTS GPIO_UART2_RTS
#    endif
#  elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE     HPM_UART3_BASE
#    define HPM_CONSOLE_BAUD     CONFIG_UART3_BAUD
#    define HPM_CONSOLE_BITS     CONFIG_UART3_BITS
#    define HPM_CONSOLE_PARITY   CONFIG_UART3_PARITY
#    define HPM_CONSOLE_2STOP    CONFIG_UART3_2STOP
#    define HPM_CONSOLE_RXD      GPIO_UART3_RXD
#    define HPM_CONSOLE_TXD      GPIO_UART3_TXD
#    ifdef CONFIG_UART3_OFLOWCONTROL
#      define HPM_CONSOLE_CTS GPIO_UART3_CTS
#    endif
#    ifdef CONFIG_UART3_IFLOWCONTROL
#      define HPM_CONSOLE_RTS GPIO_UART3_RTS
#    endif
#  elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE     HPM_UART4_BASE
#    define HPM_CONSOLE_BAUD     CONFIG_UART4_BAUD
#    define HPM_CONSOLE_BITS     CONFIG_UART4_BITS
#    define HPM_CONSOLE_PARITY   CONFIG_UART4_PARITY
#    define HPM_CONSOLE_2STOP    CONFIG_UART4_2STOP
#    define HPM_CONSOLE_RXD      GPIO_UART4_RXD
#    define HPM_CONSOLE_TXD      GPIO_UART4_TXD
#    ifdef CONFIG_UART4_OFLOWCONTROL
#      define HPM_CONSOLE_CTS GPIO_UART4_CTS
#    endif
#    ifdef CONFIG_UART4_IFLOWCONTROL
#      define HPM_CONSOLE_RTS GPIO_UART4_RTS
#    endif
#  elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE     HPM_UART5_BASE
#    define HPM_CONSOLE_BAUD     CONFIG_UART5_BAUD
#    define HPM_CONSOLE_BITS     CONFIG_UART5_BITS
#    define HPM_CONSOLE_PARITY   CONFIG_UART5_PARITY
#    define HPM_CONSOLE_2STOP    CONFIG_UART5_2STOP
#    define HPM_CONSOLE_RXD      GPIO_UART5_RXD
#    define HPM_CONSOLE_TXD      GPIO_UART5_TXD
#    ifdef CONFIG_UART5_OFLOWCONTROL
#      define HPM_CONSOLE_CTS GPIO_UART5_CTS
#    endif
#    ifdef CONFIG_UART5_IFLOWCONTROL
#      define HPM_CONSOLE_RTS GPIO_UART5_RTS
#    endif
#  elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE     HPM_UART6_BASE
#    define HPM_CONSOLE_BAUD     CONFIG_UART6_BAUD
#    define HPM_CONSOLE_BITS     CONFIG_UART6_BITS
#    define HPM_CONSOLE_PARITY   CONFIG_UART6_PARITY
#    define HPM_CONSOLE_2STOP    CONFIG_UART6_2STOP
#    define HPM_CONSOLE_RXD      GPIO_UART6_RXD
#    define HPM_CONSOLE_TXD      GPIO_UART6_TXD
#    ifdef CONFIG_UART6_OFLOWCONTROL
#      define HPM_CONSOLE_CTS GPIO_UART6_CTS
#    endif
#    ifdef CONFIG_UART6_IFLOWCONTROL
#      define HPM_CONSOLE_RTS GPIO_UART6_RTS
#    endif
#  elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#    define HPM_CONSOLE_BASE     HPM_UART7_BASE
#    define HPM_CONSOLE_BAUD     CONFIG_UART7_BAUD
#    define HPM_CONSOLE_BITS     CONFIG_UART7_BITS
#    define HPM_CONSOLE_PARITY   CONFIG_UART7_PARITY
#    define HPM_CONSOLE_2STOP    CONFIG_UART7_2STOP
#    define HPM_CONSOLE_RXD      GPIO_UART7_RXD
#    define HPM_CONSOLE_TXD      GPIO_UART7_TXD
#    ifdef CONFIG_UART7_OFLOWCONTROL
#      define HPM_CONSOLE_CTS GPIO_UART7_CTS
#    endif
#    ifdef CONFIG_UART7_IFLOWCONTROL
#      define HPM_CONSOLE_RTS GPIO_UART7_RTS
#    endif
#  else
#    error Invalid HPM UART console selection
#  endif
#endif /* HAVE_UART_CONSOLE */

#endif /* __ARCH_RISCV_SRC_HPM5300_HPM_CONSOLE_H */
