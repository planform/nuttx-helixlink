/****************************************************************************
 * boards/risc-v/hpm5300/helixlink/src/helixlink_flash.c
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
#include <inttypes.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <arch/board/board.h>

#include "hpm_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HELIXLINK_FLASH_PATH          "/dev/mtd0"
#define HELIXLINK_FLASH_CAPACITY      (16U * 1024 * 1024)
#define HELIXLINK_FLASH_READY_TIMEOUT MSEC2TICK(3000)
#define W25_CMD_RELEASE               0xab
#define W25_CMD_JEDEC_ID              0x9f
#define W25_CMD_STATUS                0x05
#define W25_STATUS_BUSY               0x01
#define W25_JEDEC_WINBOND              0xef
#define W25_JEDEC_W25Q128JV            0x40
#define W25_JEDEC_128MBIT              0x18

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_flash_lock = NXMUTEX_INITIALIZER;
static struct mtd_dev_s *g_flash_mtd;
static bool g_flash_registered;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* The common W25 driver waits indefinitely for BUSY to clear, even before
 * reading JEDEC ID. Preflight with a deadline so an absent/unresponsive
 * device cannot trap normal board bring-up in that initial busy loop.
 * This does not add a timeout to later common-driver read/write operations.
 */

static int helixlink_flash_probe(struct spi_dev_s *spi)
{
  clock_t start;
  uint8_t id[3];
  uint8_t status;
  int ret;

  ret = SPI_LOCK(spi, true);
  if (ret < 0)
    {
      return ret;
    }

  SPI_SETMODE(spi, CONFIG_W25_SPIMODE);
  SPI_SETBITS(spi, 8);
  ret = SPI_HWFEATURES(spi, 0);
  if (ret < 0 || SPI_SETFREQUENCY(spi, CONFIG_W25_SPIFREQUENCY) == 0)
    {
      ret = -EINVAL;
      goto out;
    }

  /* Release deep power down, without changing status/protection bits. */

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);
  SPI_SEND(spi, W25_CMD_RELEASE);
  SPI_SELECT(spi, SPIDEV_FLASH(0), false);
  nxsched_usleep(20);

  start = clock_systime_ticks();
  do
    {
      SPI_SELECT(spi, SPIDEV_FLASH(0), true);
      SPI_SEND(spi, W25_CMD_STATUS);
      status = SPI_SEND(spi, 0xff);
      SPI_SELECT(spi, SPIDEV_FLASH(0), false);
      if ((status & W25_STATUS_BUSY) == 0)
        {
          break;
        }

      if (clock_systime_ticks() - start >= HELIXLINK_FLASH_READY_TIMEOUT)
        {
          ret = -ETIMEDOUT;
          goto out;
        }

      nxsched_usleep(1000);
    }
  while (true);

  SPI_SELECT(spi, SPIDEV_FLASH(0), true);
  SPI_SEND(spi, W25_CMD_JEDEC_ID);
  id[0] = SPI_SEND(spi, 0xff);
  id[1] = SPI_SEND(spi, 0xff);
  id[2] = SPI_SEND(spi, 0xff);
  SPI_SELECT(spi, SPIDEV_FLASH(0), false);

  syslog(LOG_INFO, "SPI1 Flash JEDEC ID: %02x %02x %02x\n",
         id[0], id[1], id[2]);

  /* Board BOM specifies W25Q128JVSIQ; reject a floating MISO line and a
   * different capacity before handing the device to the common driver.
   */

  ret = id[0] == W25_JEDEC_WINBOND && id[1] == W25_JEDEC_W25Q128JV &&
        id[2] == W25_JEDEC_128MBIT ? OK : -ENODEV;

out:
  SPI_LOCK(spi, false);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* SPI1's board pins/CS must already be initialized. Registration failures
 * retain the allocated MTD instance so a retry does not leak another one.
 */

int helixlink_flash_initialize(void)
{
  struct mtd_geometry_s geometry;
  struct spi_dev_s *spi;
  uint64_t capacity;
  int ret;

  ret = nxmutex_lock(&g_flash_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (g_flash_registered)
    {
      ret = OK;
      goto out;
    }

  if (g_flash_mtd == NULL)
    {
      spi = hpm_spibus_initialize(1);
      if (spi == NULL)
        {
          ret = -ENODEV;
          goto out;
        }

      ret = helixlink_flash_probe(spi);
      if (ret < 0)
        {
          goto out;
        }

      g_flash_mtd = w25_initialize(spi);
      if (g_flash_mtd == NULL)
        {
          ret = -ENODEV;
          goto out;
        }
    }

  ret = MTD_IOCTL(g_flash_mtd, MTDIOC_GEOMETRY,
                  (unsigned long)(uintptr_t)&geometry);
  if (ret < 0)
    {
      goto out;
    }

  capacity = (uint64_t)geometry.erasesize * geometry.neraseblocks;
  if (capacity != HELIXLINK_FLASH_CAPACITY || geometry.blocksize == 0)
    {
      syslog(LOG_ERR, "Unexpected Flash geometry: %" PRIu64 " bytes\n",
             capacity);
      ret = -EINVAL;
      goto out;
    }

#ifdef CONFIG_W25_READONLY
  ret = register_mtddriver(HELIXLINK_FLASH_PATH, g_flash_mtd, 0444, NULL);
#else
  ret = register_mtddriver(HELIXLINK_FLASH_PATH, g_flash_mtd, 0660, NULL);
#endif
  if (ret == OK)
    {
      g_flash_registered = true;
      syslog(LOG_INFO, "%s: %" PRIu64 " bytes, block %" PRIu32
             ", erase %" PRIu32 " x %" PRIu32 "\n",
             HELIXLINK_FLASH_PATH, capacity, geometry.blocksize,
             geometry.erasesize, geometry.neraseblocks);
    }

out:
  nxmutex_unlock(&g_flash_lock);
  return ret;
}
