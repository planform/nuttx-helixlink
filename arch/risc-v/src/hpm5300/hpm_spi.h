/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_spi.h
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

#ifndef __ARCH_RISCV_SRC_HPM5300_HPM_SPI_H
#define __ARCH_RISCV_SRC_HPM5300_HPM_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Task context only, with interrupts enabled. Repeated calls return the same
 * bus without resetting a device's settings. NULL means an invalid/disabled
 * bus or a pin, clock or controller initialization failure.
 *
 * Supports 1..16 bits per word (uint8_t buffers up to 8 bits, uint16_t
 * buffers above 8 bits), modes 0..3, and optional LSB-first order.
 *
 * Clients hold SPI_LOCK around select/configure/exchange/deselect. Board
 * code initializes GPIO chip selects before calling this function. The
 * board supplies GPIO_SPIx_SCLK/MISO/MOSI and BOARD_SPIx_CLOCK_CONFIG.
 */

struct spi_dev_s *hpm_spibus_initialize(int bus);

/* Board callbacks: all chip-select wiring, polarity and device IDs belong
 * to boards/. Status returns SPI_STATUS_* bits, not a negated errno.
 */

#ifdef CONFIG_HPM_SPI0
void hpm_spi0select(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t hpm_spi0status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int hpm_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_HPM_SPI1
void hpm_spi1select(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t hpm_spi1status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int hpm_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_HPM_SPI2
void hpm_spi2select(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t hpm_spi2status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int hpm_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef CONFIG_HPM_SPI3
void hpm_spi3select(struct spi_dev_s *dev, uint32_t devid, bool selected);
uint8_t hpm_spi3status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int hpm_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPM5300_HPM_SPI_H */
