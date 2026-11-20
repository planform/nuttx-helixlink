/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_spi.c
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
#include <string.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "hpm_clockconfig.h"
#include "hpm_gpio.h"
#include "hpm_spi.h"
#include "hardware/hpm_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HPM_SPI_RESET_MASK (HPM_SPI_CTRL_SPIRST_MASK | \
                            HPM_SPI_CTRL_TXFIFORST_MASK | \
                            HPM_SPI_CTRL_RXFIFORST_MASK)
#define HPM_SPI_INTR_MASK  0x3f
#define HPM_SPI_TIMEOUT    MSEC2TICK(CONFIG_HPM_SPI_TIMEOUT)

/* HPM5300DS Rev0.12, SPI master minimum SCK period: 12.5 ns. */

#define HPM_SPI_MAX_FREQUENCY 80000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hpm_spidev_s
{
  struct spi_dev_s dev;
  mutex_t lock;
  uintptr_t base;
  uint32_t input;
  uint32_t frequency;
  uint8_t nbits;
  bool initialized;
  int error;                       /* Abort the current CS transaction */
  const struct hpm_gpio_config_s *sclk;
  const struct hpm_gpio_config_s *miso;
  const struct hpm_gpio_config_s *mosi;
  void (*select)(struct spi_dev_s *dev, uint32_t devid, bool selected);
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hpm_spi_lock(struct spi_dev_s *dev, bool lock);
static void hpm_spi_select(struct spi_dev_s *dev, uint32_t devid,
                           bool selected);
static uint32_t hpm_spi_setfrequency(struct spi_dev_s *dev,
                                     uint32_t frequency);
static void hpm_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void hpm_spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t hpm_spi_send(struct spi_dev_s *dev, uint32_t word);
static void hpm_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                             void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void hpm_spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                             size_t nwords);
static void hpm_spi_recvblock(struct spi_dev_s *dev, void *buffer,
                              size_t nwords);
#endif
#ifdef CONFIG_SPI_HWFEATURES
static int hpm_spi_hwfeatures(struct spi_dev_s *dev,
                              spi_hwfeatures_t features);
#endif
#ifdef CONFIG_SPI_DELAY_CONTROL
static int hpm_spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                            uint32_t stopdelay, uint32_t csdelay,
                            uint32_t ifdelay);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_HPM_SPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock             = hpm_spi_lock,
  .select           = hpm_spi_select,
  .setfrequency     = hpm_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay         = hpm_spi_setdelay,
#endif
  .setmode          = hpm_spi_setmode,
  .setbits          = hpm_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = hpm_spi_hwfeatures,
#endif
  .status           = hpm_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = hpm_spi0cmddata,
#endif
  .send             = hpm_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = hpm_spi_exchange,
#else
  .sndblock         = hpm_spi_sndblock,
  .recvblock        = hpm_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = NULL,
#endif
  .registercallback = NULL,
};

static struct hpm_spidev_s g_spi0dev =
{
  .dev              =
  {
    .ops            = &g_spi0ops
  },
  .lock             = NXMUTEX_INITIALIZER,
  .base             = HPM_SPI0_BASE,
  .sclk             = GPIO_SPI0_SCLK,
  .miso             = GPIO_SPI0_MISO,
  .mosi             = GPIO_SPI0_MOSI,
  .select           = hpm_spi0select,
};
#endif

#ifdef CONFIG_HPM_SPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock             = hpm_spi_lock,
  .select           = hpm_spi_select,
  .setfrequency     = hpm_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay         = hpm_spi_setdelay,
#endif
  .setmode          = hpm_spi_setmode,
  .setbits          = hpm_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = hpm_spi_hwfeatures,
#endif
  .status           = hpm_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = hpm_spi1cmddata,
#endif
  .send             = hpm_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = hpm_spi_exchange,
#else
  .sndblock         = hpm_spi_sndblock,
  .recvblock        = hpm_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = NULL,
#endif
  .registercallback = NULL,
};

static struct hpm_spidev_s g_spi1dev =
{
  .dev              =
  {
    .ops            = &g_spi1ops
  },
  .lock             = NXMUTEX_INITIALIZER,
  .base             = HPM_SPI1_BASE,
  .sclk             = GPIO_SPI1_SCLK,
  .miso             = GPIO_SPI1_MISO,
  .mosi             = GPIO_SPI1_MOSI,
  .select           = hpm_spi1select,
};
#endif

#ifdef CONFIG_HPM_SPI2
static const struct spi_ops_s g_spi2ops =
{
  .lock             = hpm_spi_lock,
  .select           = hpm_spi_select,
  .setfrequency     = hpm_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay         = hpm_spi_setdelay,
#endif
  .setmode          = hpm_spi_setmode,
  .setbits          = hpm_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = hpm_spi_hwfeatures,
#endif
  .status           = hpm_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = hpm_spi2cmddata,
#endif
  .send             = hpm_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = hpm_spi_exchange,
#else
  .sndblock         = hpm_spi_sndblock,
  .recvblock        = hpm_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = NULL,
#endif
  .registercallback = NULL,
};

static struct hpm_spidev_s g_spi2dev =
{
  .dev              =
  {
    .ops            = &g_spi2ops
  },
  .lock             = NXMUTEX_INITIALIZER,
  .base             = HPM_SPI2_BASE,
  .sclk             = GPIO_SPI2_SCLK,
  .miso             = GPIO_SPI2_MISO,
  .mosi             = GPIO_SPI2_MOSI,
  .select           = hpm_spi2select,
};
#endif

#ifdef CONFIG_HPM_SPI3
static const struct spi_ops_s g_spi3ops =
{
  .lock             = hpm_spi_lock,
  .select           = hpm_spi_select,
  .setfrequency     = hpm_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay         = hpm_spi_setdelay,
#endif
  .setmode          = hpm_spi_setmode,
  .setbits          = hpm_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = hpm_spi_hwfeatures,
#endif
  .status           = hpm_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = hpm_spi3cmddata,
#endif
  .send             = hpm_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = hpm_spi_exchange,
#else
  .sndblock         = hpm_spi_sndblock,
  .recvblock        = hpm_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger          = NULL,
#endif
  .registercallback = NULL,
};

static struct hpm_spidev_s g_spi3dev =
{
  .dev              =
  {
    .ops            = &g_spi3ops
  },
  .lock             = NXMUTEX_INITIALIZER,
  .base             = HPM_SPI3_BASE,
  .sclk             = GPIO_SPI3_SCLK,
  .miso             = GPIO_SPI3_MISO,
  .mosi             = GPIO_SPI3_MOSI,
  .select           = hpm_spi3select,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int hpm_spi_wait(struct hpm_spidev_s *priv, unsigned int offset,
                         uint32_t mask)
{
  clock_t start = clock_systime_ticks();

  while ((getreg32(priv->base + offset) & mask) != 0)
    {
      if (clock_systime_ticks() - start >= HPM_SPI_TIMEOUT)
        {
          return -ETIMEDOUT;
        }
    }

  return OK;
}

/* Reset the transfer engine and FIFOs, not the format/timing registers.
 * Hardware CS outputs, DMA requests and interrupts are unused here.
 */

static int hpm_spi_reset(struct hpm_spidev_s *priv)
{
  int ret;

  putreg32(0, HPM_SPI_INTREN(priv->base));
  putreg32(HPM_SPI_RESET_MASK, HPM_SPI_CTRL(priv->base));
  ret = hpm_spi_wait(priv, HPM_SPI_CTRL_OFFSET, HPM_SPI_RESET_MASK);
  putreg32(HPM_SPI_INTR_MASK, HPM_SPI_INTRST(priv->base));
  return ret;
}

static int hpm_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;

  return lock ? nxmutex_lock(&priv->lock) : nxmutex_unlock(&priv->lock);
}

static void hpm_spi_select(struct spi_dev_s *dev, uint32_t devid,
                           bool selected)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;

  priv->select(dev, devid, selected);
  if (!selected)
    {
      /* After an exchange failure, suppress the remainder of this CS
       * transaction. A new transaction can retry after recovery.
       */

      priv->error = 0;
    }
}

static uint32_t hpm_spi_setfrequency(struct spi_dev_s *dev,
                                     uint32_t frequency)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;
  uint32_t divisor;
  uint32_t encoded;

  if (frequency == 0 || priv->input == 0)
    {
      return 0;
    }

  if (frequency > HPM_SPI_MAX_FREQUENCY)
    {
      frequency = HPM_SPI_MAX_FREQUENCY;
    }

  if (frequency < priv->input / 510 + (priv->input % 510 != 0))
    {
      spierr("SPI frequency %" PRIu32 " is below the divider range\n",
             frequency);
      return 0;
    }

  if (frequency >= priv->input)
    {
      divisor = 1;
      encoded = 0xff;   /* Special encoding: bypass the divider */
    }
  else
    {
      /* Round up to an even divisor so SCLK never exceeds the request.
       * Encodings 0..254 correspond to divisors 2..510; 255 is bypass.
       */

      divisor = priv->input / frequency;
      divisor += priv->input % frequency != 0;
      divisor = (divisor + 1U) & ~1U;
      encoded = divisor / 2 - 1;
    }

  modifyreg32(HPM_SPI_TIMING(priv->base), HPM_SPI_TIMING_SCLK_DIV_MASK,
              HPM_SPI_TIMING_SCLK_DIV_SET(encoded));
  priv->frequency = priv->input / divisor;
  return priv->frequency;
}

static void hpm_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;
  uint32_t bits = 0;

  switch (mode)
    {
      case SPIDEV_MODE0:
        break;
      case SPIDEV_MODE1:
        bits = HPM_SPI_TRANSFMT_CPHA_MASK;
        break;
      case SPIDEV_MODE2:
        bits = HPM_SPI_TRANSFMT_CPOL_MASK;
        break;
      case SPIDEV_MODE3:
        bits = HPM_SPI_TRANSFMT_CPOL_MASK | HPM_SPI_TRANSFMT_CPHA_MASK;
        break;
      default:
        spierr("Unsupported SPI mode: %d\n", mode);
        return;
    }

  modifyreg32(HPM_SPI_TRANSFMT(priv->base), HPM_SPI_TRANSFMT_CPOL_MASK |
              HPM_SPI_TRANSFMT_CPHA_MASK, bits);
}

static void hpm_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;

  /* Follow the NuttX SPI buffer convention: uint8_t for 1..8 bits and
   * uint16_t for 9..16 bits. SPI_SEND also truncates its argument to 16 bits
   * in NuttX 13.1, so do not advertise the hardware's 32-bit word extension.
   */

  if (nbits < 1 || nbits > 16)
    {
      spierr("Unsupported SPI word size: %d\n", nbits);
      return;
    }

  modifyreg32(HPM_SPI_TRANSFMT(priv->base), HPM_SPI_TRANSFMT_DATALEN_MASK,
              HPM_SPI_TRANSFMT_DATALEN_SET(nbits - 1U));
  priv->nbits = nbits;
}

#ifdef CONFIG_SPI_HWFEATURES
static int hpm_spi_hwfeatures(struct spi_dev_s *dev,
                              spi_hwfeatures_t features)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;
  uint32_t bits = 0;

#ifdef CONFIG_SPI_BITORDER
  if ((features & ~HWFEAT_LSBFIRST) != 0)
    {
      return -ENOSYS;
    }

  if ((features & HWFEAT_LSBFIRST) != 0)
    {
      bits = HPM_SPI_TRANSFMT_LSB_MASK;
    }
#else
  if (features != 0)
    {
      return -ENOSYS;
    }
#endif

  modifyreg32(HPM_SPI_TRANSFMT(priv->base), HPM_SPI_TRANSFMT_LSB_MASK, bits);
  return OK;
}
#endif

#ifdef CONFIG_SPI_DELAY_CONTROL
static int hpm_spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                            uint32_t stopdelay, uint32_t csdelay,
                            uint32_t ifdelay)
{
  /* GPIO chip-select timing and inter-frame delays are not implemented. */

  return -ENOSYS;
}
#endif

/* Little-endian, unaligned-safe access to one buffer word. */

static uint32_t hpm_spi_load(const uint8_t *buffer, unsigned int bytes)
{
  uint32_t word = UINT32_MAX;

  if (buffer != NULL)
    {
      word = 0;
      memcpy(&word, buffer, bytes);
    }

  return word;
}

static void hpm_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                             void *rxbuffer, size_t nwords)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;
  const uint8_t *tx = txbuffer;
  uint8_t *rx = rxbuffer;
  unsigned int bytes = priv->nbits > 8 ? 2 : 1;
  size_t sent = 0;
  size_t received = 0;
  clock_t start;
  uint32_t status;
  uint32_t word;
  bool progress;
  int ret;

  if (nwords == 0)
    {
      return;
    }

  ret = priv->error;
  if (ret < 0)
    {
      goto fill;
    }

  ret = hpm_spi_wait(priv, HPM_SPI_STATUS_OFFSET,
                     HPM_SPI_STATUS_SPIACTIVE_MASK);
  if (ret < 0)
    {
      goto fail;
    }

  /* Generic SPI always exchanges words, without command/address phases.
   * Reading with no TX buffer shifts all ones; writing with no RX buffer
   * still drains RX FIFO. Keep the board's GPIO CS unchanged throughout.
   */

  /* Populate the legacy fields as well, matching the vendor's extended
   * count setup. The independent 32-bit registers allow blocks >512 words.
   */

  putreg32(HPM_SPI_TRANSCTRL_WRTRANCNT_SET(nwords - 1) |
           HPM_SPI_TRANSCTRL_RDTRANCNT_SET(nwords - 1),
           HPM_SPI_TRANSCTRL(priv->base));
  putreg32(nwords - 1, HPM_SPI_WR_TRANS_CNT(priv->base));
  putreg32(nwords - 1, HPM_SPI_RD_TRANS_CNT(priv->base));
  ret = hpm_spi_reset(priv);
  if (ret < 0)
    {
      goto fail;
    }

  /* Preload one word before CMD starts the transfer (UM section 47.3.2). */

  putreg32(hpm_spi_load(tx, bytes), HPM_SPI_DATA(priv->base));
  if (tx != NULL)
    {
      tx += bytes;
    }

  sent = 1;
  putreg32(0xff, HPM_SPI_CMD(priv->base));
  start = clock_systime_ticks();

  while (received < nwords)
    {
      status = getreg32(HPM_SPI_STATUS(priv->base));
      progress = false;

      /* Drain first to avoid unnecessary RX FIFO backpressure. */

      if ((status & HPM_SPI_STATUS_RXEMPTY_MASK) == 0)
        {
          word = getreg32(HPM_SPI_DATA(priv->base)) &
                 (UINT32_MAX >> (32 - priv->nbits));
          if (rx != NULL)
            {
              memcpy(rx, &word, bytes);
              rx += bytes;
            }

          received++;
          progress = true;
        }

      if (sent < nwords && (status & HPM_SPI_STATUS_TXFULL_MASK) == 0)
        {
          putreg32(hpm_spi_load(tx, bytes), HPM_SPI_DATA(priv->base));
          if (tx != NULL)
            {
              tx += bytes;
            }

          sent++;
          progress = true;
        }

      if (progress)
        {
          start = clock_systime_ticks();
        }
      else if (clock_systime_ticks() - start >= HPM_SPI_TIMEOUT)
        {
          ret = -ETIMEDOUT;
          goto fail;
        }
    }

  /* FIFO data availability alone does not guarantee the last clock edge
   * has completed. Do not allow the client to release CS until idle.
   */

  ret = hpm_spi_wait(priv, HPM_SPI_STATUS_OFFSET,
                     HPM_SPI_STATUS_SPIACTIVE_MASK);
  if (ret == OK)
    {
      return;
    }

fail:
  spierr("SPI at %" PRIxPTR ": transfer failed: %d\n", priv->base, ret);
  hpm_spi_reset(priv);
  priv->error = ret;

fill:
  /* SPI_EXCHANGE has no error return. Supply deterministic dummy data for
   * unread words and log the failure; the upper protocol must validate its
   * response. Remaining exchanges under this CS are suppressed until the
   * caller deselects the device.
   */

  if (rx != NULL)
    {
      memset(rx, 0xff, (nwords - received) * bytes);
    }
}

static uint32_t hpm_spi_send(struct spi_dev_s *dev, uint32_t word)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;
  uint32_t received = 0;

  hpm_spi_exchange(dev, &word, &received, 1);
  return received & (UINT32_MAX >> (32 - priv->nbits));
}

#ifndef CONFIG_SPI_EXCHANGE
static void hpm_spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                             size_t nwords)
{
  hpm_spi_exchange(dev, buffer, NULL, nwords);
}

static void hpm_spi_recvblock(struct spi_dev_s *dev, void *buffer,
                              size_t nwords)
{
  hpm_spi_exchange(dev, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct spi_dev_s *hpm_spibus_initialize(int bus)
{
  struct hpm_spidev_s *priv;
  int ret;

  switch (bus)
    {
#ifdef CONFIG_HPM_SPI0
      case 0:
        priv = &g_spi0dev;
        break;
#endif
#ifdef CONFIG_HPM_SPI1
      case 1:
        priv = &g_spi1dev;
        break;
#endif
#ifdef CONFIG_HPM_SPI2
      case 2:
        priv = &g_spi2dev;
        break;
#endif
#ifdef CONFIG_HPM_SPI3
      case 3:
        priv = &g_spi3dev;
        break;
#endif
      default:
        return NULL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return NULL;
    }

  if (priv->initialized)
    {
      nxmutex_unlock(&priv->lock);
      return &priv->dev;
    }

  /* Reject malformed board descriptions before touching any hardware. */

  if (priv->sclk == NULL || priv->miso == NULL || priv->mosi == NULL ||
      priv->sclk->mode != HPM_GPIO_PERIPHERAL ||
      priv->miso->mode != HPM_GPIO_PERIPHERAL ||
      priv->mosi->mode != HPM_GPIO_PERIPHERAL ||
      hpm_ioc_validate(&priv->sclk->ioc) < 0 ||
      hpm_ioc_validate(&priv->miso->ioc) < 0 ||
      hpm_ioc_validate(&priv->mosi->ioc) < 0)
    {
      ret = -EINVAL;
      goto out;
    }

  priv->input = hpm_spi_clockconfig(priv->base);
  if (priv->input == 0)
    {
      ret = -EIO;
      goto out;
    }

  ret = hpm_spi_reset(priv);
  if (ret < 0)
    {
      goto out;
    }

  putreg32(0, HPM_SPI_DIRECTIO(priv->base));
  putreg32(HPM_SPI_TRANSFMT_DATALEN_SET(7), HPM_SPI_TRANSFMT(priv->base));
  putreg32(0, HPM_SPI_TRANSCTRL(priv->base));
  priv->nbits = 8;

  /* Configure SCLK's idle level before connecting the output to the pad. */

  ret = hpm_gpio_config(priv->sclk);
  if (ret < 0)
    {
      goto out;
    }

  ret = hpm_gpio_config(priv->miso);
  if (ret < 0)
    {
      goto out;
    }

  ret = hpm_gpio_config(priv->mosi);
  if (ret < 0)
    {
      goto out;
    }

  /* Lowest representable frequency if the board clock cannot reach 400k. */

  if (hpm_spi_setfrequency(&priv->dev, 400000) == 0)
    {
      uint32_t minimum = priv->input / 510 +
                         (priv->input % 510 != 0);

      if (hpm_spi_setfrequency(&priv->dev, minimum) == 0)
        {
          ret = -ERANGE;
          goto out;
        }
    }

  priv->error = 0;
  priv->initialized = true;

out:
  nxmutex_unlock(&priv->lock);
  if (ret < 0)
    {
      spierr("SPI%d initialization failed: %d\n", bus, ret);
      return NULL;
    }

  return &priv->dev;
}
