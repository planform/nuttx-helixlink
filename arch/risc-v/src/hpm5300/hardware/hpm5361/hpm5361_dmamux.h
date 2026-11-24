/****************************************************************************
 * arch/risc-v/src/hpm5300/hardware/hpm5361/hpm5361_dmamux.h
 *
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Definitions adapted from HPM5300 SDK hpm_dmamux_regs.h and HPM5361
 * hpm_dmamux_src.h. See HPM5300UM Rev1.2, chapter 29.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_DMAMUX_H
#define __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hpm5361_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MUXCFG register offsets and addresses. n selects a routing output, 0..31.
 * The caller must validate n before constructing the register address.
 */

#define HPM_DMAMUX_NCHANNELS          32

#define HPM_DMAMUX_MUXCFG_OFFSET(n)   (0x0000 + 0x0004 * (n))
#define HPM_DMAMUX_MUXCFG(n) \
  (HPM_DMAMUX_BASE + HPM_DMAMUX_MUXCFG_OFFSET(n))

/* MUXCFG is write-only. Write the complete configuration directly; do not
 * read it back or use read-modify-write. Stop the DMA channel before
 * changing
 * its routing. SOURCE_GET decodes a software value, not a hardware read.
 */

#define HPM_DMAMUX_MUXCFG_ENABLE      (1U << 31)

#define HPM_DMAMUX_MUXCFG_SOURCE_MASK 0x7fU
#define HPM_DMAMUX_MUXCFG_SOURCE_SET(v) \
  ((v) & HPM_DMAMUX_MUXCFG_SOURCE_MASK)
#define HPM_DMAMUX_MUXCFG_SOURCE_GET(v) \
  ((v) & HPM_DMAMUX_MUXCFG_SOURCE_MASK)

/* Peripheral request-source IDs. These are SOURCE encodings, not DMA
 * channel numbers or CHANREQCTRL handshake-port selectors. Zero is a valid
 * peripheral source ID and must not be used as a no-request sentinel.
 */

/* GPTMR, UART, I2C, SPI and MCAN request sources. */

#define HPM_DMA_SRC_GPTMR0_0       0x00U
#define HPM_DMA_SRC_GPTMR0_1       0x01U
#define HPM_DMA_SRC_GPTMR0_2       0x02U
#define HPM_DMA_SRC_GPTMR0_3       0x03U
#define HPM_DMA_SRC_GPTMR1_0       0x04U
#define HPM_DMA_SRC_GPTMR1_1       0x05U
#define HPM_DMA_SRC_GPTMR1_2       0x06U
#define HPM_DMA_SRC_GPTMR1_3       0x07U
#define HPM_DMA_SRC_GPTMR2_0       0x08U
#define HPM_DMA_SRC_GPTMR2_1       0x09U
#define HPM_DMA_SRC_GPTMR2_2       0x0aU
#define HPM_DMA_SRC_GPTMR2_3       0x0bU
#define HPM_DMA_SRC_GPTMR3_0       0x0cU
#define HPM_DMA_SRC_GPTMR3_1       0x0dU
#define HPM_DMA_SRC_GPTMR3_2       0x0eU
#define HPM_DMA_SRC_GPTMR3_3       0x0fU
#define HPM_DMA_SRC_UART0_RX       0x14U
#define HPM_DMA_SRC_UART0_TX       0x15U
#define HPM_DMA_SRC_UART1_RX       0x16U
#define HPM_DMA_SRC_UART1_TX       0x17U
#define HPM_DMA_SRC_UART2_RX       0x18U
#define HPM_DMA_SRC_UART2_TX       0x19U
#define HPM_DMA_SRC_UART3_RX       0x1aU
#define HPM_DMA_SRC_UART3_TX       0x1bU
#define HPM_DMA_SRC_UART4_RX       0x1cU
#define HPM_DMA_SRC_UART4_TX       0x1dU
#define HPM_DMA_SRC_UART5_RX       0x1eU
#define HPM_DMA_SRC_UART5_TX       0x1fU
#define HPM_DMA_SRC_UART6_RX       0x20U
#define HPM_DMA_SRC_UART6_TX       0x21U
#define HPM_DMA_SRC_UART7_RX       0x22U
#define HPM_DMA_SRC_UART7_TX       0x23U
#define HPM_DMA_SRC_I2C0           0x24U
#define HPM_DMA_SRC_I2C1           0x25U
#define HPM_DMA_SRC_I2C2           0x26U
#define HPM_DMA_SRC_I2C3           0x27U
#define HPM_DMA_SRC_SPI0_RX        0x28U
#define HPM_DMA_SRC_SPI0_TX        0x29U
#define HPM_DMA_SRC_SPI1_RX        0x2aU
#define HPM_DMA_SRC_SPI1_TX        0x2bU
#define HPM_DMA_SRC_SPI2_RX        0x2cU
#define HPM_DMA_SRC_SPI2_TX        0x2dU
#define HPM_DMA_SRC_SPI3_RX        0x2eU
#define HPM_DMA_SRC_SPI3_TX        0x2fU
#define HPM_DMA_SRC_MCAN0          0x30U
#define HPM_DMA_SRC_MCAN1          0x31U
#define HPM_DMA_SRC_MCAN2          0x32U
#define HPM_DMA_SRC_MCAN3          0x33U

/* MOT requests are routed through TRGM0. They aggregate requests from
 * PWM0, PWM1, QEI0, QEI1, MMC0, MMC1, SEI0 and SEI1.
 */

#define HPM_DMA_SRC_MOT_0          0x34U
#define HPM_DMA_SRC_MOT_1          0x35U
#define HPM_DMA_SRC_MOT_2          0x36U
#define HPM_DMA_SRC_MOT_3          0x37U
#define HPM_DMA_SRC_MOT_4          0x38U
#define HPM_DMA_SRC_MOT_5          0x39U
#define HPM_DMA_SRC_MOT_6          0x3aU
#define HPM_DMA_SRC_MOT_7          0x3bU

/* XPI, DAC and ACMP request sources. */

#define HPM_DMA_SRC_XPI0_RX        0x3cU
#define HPM_DMA_SRC_XPI0_TX        0x3dU
#define HPM_DMA_SRC_DAC0           0x3eU
#define HPM_DMA_SRC_DAC1           0x3fU
#define HPM_DMA_SRC_ACMP0          0x40U
#define HPM_DMA_SRC_ACMP1          0x41U

/* Chip-specific request-source validity. HPM5361 defines sources 0x00..0x0f
 * and 0x14..0x41; the gaps and higher SOURCE encodings are not valid
 * requests.
 * Keep this capability in the chip header so the common DMA driver does not
 * embed this chip's numbering. The argument must have no side effects.
 */

#define HPM_DMAMUX_SOURCE_VALID(s) \
  (((s) <= HPM_DMA_SRC_GPTMR3_3) || \
   ((s) >= HPM_DMA_SRC_UART0_RX && (s) <= HPM_DMA_SRC_ACMP1))

#endif /* __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_DMAMUX_H */
