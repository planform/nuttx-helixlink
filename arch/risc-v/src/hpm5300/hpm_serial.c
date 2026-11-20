/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_serial.c
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

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/serial/serial.h>
#include <nuttx/spinlock.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "hpm_config.h"
#include "hpm_gpio.h"
#include "hpm_lowputc.h"
#include "hpm_clockconfig.h"
#include "hardware/hpm_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HPM_UART_ISR_LIMIT  256

#define HPM_UART_RX_ERRORS \
  (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | \
   UART_LSR_LBREAK | UART_LSR_ERRF)

#if (defined(CONFIG_HPM_UART0) && \
     (defined(CONFIG_UART0_RXDMA) || defined(CONFIG_UART0_TXDMA))) || \
    (defined(CONFIG_HPM_UART1) && \
     (defined(CONFIG_UART1_RXDMA) || defined(CONFIG_UART1_TXDMA))) || \
    (defined(CONFIG_HPM_UART2) && \
     (defined(CONFIG_UART2_RXDMA) || defined(CONFIG_UART2_TXDMA))) || \
    (defined(CONFIG_HPM_UART3) && \
     (defined(CONFIG_UART3_RXDMA) || defined(CONFIG_UART3_TXDMA))) || \
    (defined(CONFIG_HPM_UART4) && \
     (defined(CONFIG_UART4_RXDMA) || defined(CONFIG_UART4_TXDMA))) || \
    (defined(CONFIG_HPM_UART5) && \
     (defined(CONFIG_UART5_RXDMA) || defined(CONFIG_UART5_TXDMA))) || \
    (defined(CONFIG_HPM_UART6) && \
     (defined(CONFIG_UART6_RXDMA) || defined(CONFIG_UART6_TXDMA))) || \
    (defined(CONFIG_HPM_UART7) && \
     (defined(CONFIG_UART7_RXDMA) || defined(CONFIG_UART7_TXDMA)))
#  error HPM5300 UART DMA is not implemented
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(USE_SERIALDRIVER) && defined(HAVE_UART_DEVICE)

struct hpm_uart_s
{
  uint32_t uartbase;
  int irq;
  spinlock_t lock;
  uint32_t im;          /* Enabled UART interrupt mask */
  uint32_t rxstatus;    /* Cached receive error flags */

  /* Receive IRQ intent and temporary flow-control pause are independent. */

  bool rxenabled;
  bool rxpaused;

  struct uart_config_s config;

  const struct hpm_gpio_config_s *rx_pin;
  const struct hpm_gpio_config_s *tx_pin;
  const struct hpm_gpio_config_s *cts_pin;
  const struct hpm_gpio_config_s *rts_pin;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hpm_interrupt(int irq, void *context, void *arg);

static int hpm_setup(struct uart_dev_s *dev);
static void hpm_shutdown(struct uart_dev_s *dev);
static int hpm_attach(struct uart_dev_s *dev);
static void hpm_detach(struct uart_dev_s *dev);

static int hpm_ioctl(struct file *filep, int cmd, unsigned long arg);

static int hpm_receive(struct uart_dev_s *dev, unsigned int *status);
static void hpm_rxint(struct uart_dev_s *dev, bool enable);
static bool hpm_rxavailable(struct uart_dev_s *dev);

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool hpm_rxflowcontrol(struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper);
#endif

static void hpm_send(struct uart_dev_s *dev, int ch);
static void hpm_txint(struct uart_dev_s *dev, bool enable);
static bool hpm_txready(struct uart_dev_s *dev);
static bool hpm_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup       = hpm_setup,
  .shutdown    = hpm_shutdown,
  .attach      = hpm_attach,
  .detach      = hpm_detach,
  .ioctl       = hpm_ioctl,
  .receive     = hpm_receive,
  .rxint       = hpm_rxint,
  .rxavailable = hpm_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = hpm_rxflowcontrol,
#endif
  .send        = hpm_send,
  .txint       = hpm_txint,
  .txready     = hpm_txready,
  .txempty     = hpm_txempty,
};

#ifdef CONFIG_HPM_UART0
static char g_uart0_rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0_txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct hpm_uart_s g_uart0_priv =
{
  .uartbase = HPM_UART0_BASE,
  .irq      = HPM_IRQ_UART0,
  .lock     = SP_UNLOCKED,
  .config   =
  {
    .baud      = CONFIG_UART0_BAUD,
    .parity    = CONFIG_UART0_PARITY,
    .bits      = CONFIG_UART0_BITS,
    .stopbits2 = CONFIG_UART0_2STOP != 0,
#ifdef CONFIG_UART0_OFLOWCONTROL
    .usects    = true,
#endif
#ifdef CONFIG_UART0_IFLOWCONTROL
    .userts    = true,
#endif
  },
  .rx_pin = GPIO_UART0_RXD,
  .tx_pin = GPIO_UART0_TXD,
#ifdef CONFIG_UART0_OFLOWCONTROL
  .cts_pin = GPIO_UART0_CTS,
#endif
#ifdef CONFIG_UART0_IFLOWCONTROL
  .rts_pin = GPIO_UART0_RTS,
#endif
};

static struct uart_dev_s g_uart0_dev =
{
  .recv =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0_rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0_txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart0_priv,
};
#endif

#ifdef CONFIG_HPM_UART1
static char g_uart1_rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1_txbuffer[CONFIG_UART1_TXBUFSIZE];

static struct hpm_uart_s g_uart1_priv =
{
  .uartbase = HPM_UART1_BASE,
  .irq      = HPM_IRQ_UART1,
  .lock     = SP_UNLOCKED,
  .config   =
  {
    .baud      = CONFIG_UART1_BAUD,
    .parity    = CONFIG_UART1_PARITY,
    .bits      = CONFIG_UART1_BITS,
    .stopbits2 = CONFIG_UART1_2STOP != 0,
#ifdef CONFIG_UART1_OFLOWCONTROL
    .usects    = true,
#endif
#ifdef CONFIG_UART1_IFLOWCONTROL
    .userts    = true,
#endif
  },
  .rx_pin = GPIO_UART1_RXD,
  .tx_pin = GPIO_UART1_TXD,
#ifdef CONFIG_UART1_OFLOWCONTROL
  .cts_pin = GPIO_UART1_CTS,
#endif
#ifdef CONFIG_UART1_IFLOWCONTROL
  .rts_pin = GPIO_UART1_RTS,
#endif
};

static struct uart_dev_s g_uart1_dev =
{
  .recv =
  {
    .size   = CONFIG_UART1_RXBUFSIZE,
    .buffer = g_uart1_rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART1_TXBUFSIZE,
    .buffer = g_uart1_txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart1_priv,
};
#endif

#ifdef CONFIG_HPM_UART2
static char g_uart2_rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2_txbuffer[CONFIG_UART2_TXBUFSIZE];

static struct hpm_uart_s g_uart2_priv =
{
  .uartbase = HPM_UART2_BASE,
  .irq      = HPM_IRQ_UART2,
  .lock     = SP_UNLOCKED,
  .config   =
  {
    .baud      = CONFIG_UART2_BAUD,
    .parity    = CONFIG_UART2_PARITY,
    .bits      = CONFIG_UART2_BITS,
    .stopbits2 = CONFIG_UART2_2STOP != 0,
#ifdef CONFIG_UART2_OFLOWCONTROL
    .usects    = true,
#endif
#ifdef CONFIG_UART2_IFLOWCONTROL
    .userts    = true,
#endif
  },
  .rx_pin = GPIO_UART2_RXD,
  .tx_pin = GPIO_UART2_TXD,
#ifdef CONFIG_UART2_OFLOWCONTROL
  .cts_pin = GPIO_UART2_CTS,
#endif
#ifdef CONFIG_UART2_IFLOWCONTROL
  .rts_pin = GPIO_UART2_RTS,
#endif
};

static struct uart_dev_s g_uart2_dev =
{
  .recv =
  {
    .size   = CONFIG_UART2_RXBUFSIZE,
    .buffer = g_uart2_rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART2_TXBUFSIZE,
    .buffer = g_uart2_txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart2_priv,
};
#endif

#ifdef CONFIG_HPM_UART3
static char g_uart3_rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3_txbuffer[CONFIG_UART3_TXBUFSIZE];

static struct hpm_uart_s g_uart3_priv =
{
  .uartbase = HPM_UART3_BASE,
  .irq      = HPM_IRQ_UART3,
  .lock     = SP_UNLOCKED,
  .config   =
  {
    .baud      = CONFIG_UART3_BAUD,
    .parity    = CONFIG_UART3_PARITY,
    .bits      = CONFIG_UART3_BITS,
    .stopbits2 = CONFIG_UART3_2STOP != 0,
#ifdef CONFIG_UART3_OFLOWCONTROL
    .usects    = true,
#endif
#ifdef CONFIG_UART3_IFLOWCONTROL
    .userts    = true,
#endif
  },
  .rx_pin = GPIO_UART3_RXD,
  .tx_pin = GPIO_UART3_TXD,
#ifdef CONFIG_UART3_OFLOWCONTROL
  .cts_pin = GPIO_UART3_CTS,
#endif
#ifdef CONFIG_UART3_IFLOWCONTROL
  .rts_pin = GPIO_UART3_RTS,
#endif
};

static struct uart_dev_s g_uart3_dev =
{
  .recv =
  {
    .size   = CONFIG_UART3_RXBUFSIZE,
    .buffer = g_uart3_rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART3_TXBUFSIZE,
    .buffer = g_uart3_txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart3_priv,
};
#endif

#ifdef CONFIG_HPM_UART4
static char g_uart4_rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4_txbuffer[CONFIG_UART4_TXBUFSIZE];

static struct hpm_uart_s g_uart4_priv =
{
  .uartbase = HPM_UART4_BASE,
  .irq      = HPM_IRQ_UART4,
  .lock     = SP_UNLOCKED,
  .config   =
  {
    .baud      = CONFIG_UART4_BAUD,
    .parity    = CONFIG_UART4_PARITY,
    .bits      = CONFIG_UART4_BITS,
    .stopbits2 = CONFIG_UART4_2STOP != 0,
#ifdef CONFIG_UART4_OFLOWCONTROL
    .usects    = true,
#endif
#ifdef CONFIG_UART4_IFLOWCONTROL
    .userts    = true,
#endif
  },
  .rx_pin = GPIO_UART4_RXD,
  .tx_pin = GPIO_UART4_TXD,
#ifdef CONFIG_UART4_OFLOWCONTROL
  .cts_pin = GPIO_UART4_CTS,
#endif
#ifdef CONFIG_UART4_IFLOWCONTROL
  .rts_pin = GPIO_UART4_RTS,
#endif
};

static struct uart_dev_s g_uart4_dev =
{
  .recv =
  {
    .size   = CONFIG_UART4_RXBUFSIZE,
    .buffer = g_uart4_rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART4_TXBUFSIZE,
    .buffer = g_uart4_txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart4_priv,
};
#endif

#ifdef CONFIG_HPM_UART5
static char g_uart5_rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5_txbuffer[CONFIG_UART5_TXBUFSIZE];

static struct hpm_uart_s g_uart5_priv =
{
  .uartbase = HPM_UART5_BASE,
  .irq      = HPM_IRQ_UART5,
  .lock     = SP_UNLOCKED,
  .config   =
  {
    .baud      = CONFIG_UART5_BAUD,
    .parity    = CONFIG_UART5_PARITY,
    .bits      = CONFIG_UART5_BITS,
    .stopbits2 = CONFIG_UART5_2STOP != 0,
#ifdef CONFIG_UART5_OFLOWCONTROL
    .usects    = true,
#endif
#ifdef CONFIG_UART5_IFLOWCONTROL
    .userts    = true,
#endif
  },
  .rx_pin = GPIO_UART5_RXD,
  .tx_pin = GPIO_UART5_TXD,
#ifdef CONFIG_UART5_OFLOWCONTROL
  .cts_pin = GPIO_UART5_CTS,
#endif
#ifdef CONFIG_UART5_IFLOWCONTROL
  .rts_pin = GPIO_UART5_RTS,
#endif
};

static struct uart_dev_s g_uart5_dev =
{
  .recv =
  {
    .size   = CONFIG_UART5_RXBUFSIZE,
    .buffer = g_uart5_rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART5_TXBUFSIZE,
    .buffer = g_uart5_txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart5_priv,
};
#endif

#ifdef CONFIG_HPM_UART6
static char g_uart6_rxbuffer[CONFIG_UART6_RXBUFSIZE];
static char g_uart6_txbuffer[CONFIG_UART6_TXBUFSIZE];

static struct hpm_uart_s g_uart6_priv =
{
  .uartbase = HPM_UART6_BASE,
  .irq      = HPM_IRQ_UART6,
  .lock     = SP_UNLOCKED,
  .config   =
  {
    .baud      = CONFIG_UART6_BAUD,
    .parity    = CONFIG_UART6_PARITY,
    .bits      = CONFIG_UART6_BITS,
    .stopbits2 = CONFIG_UART6_2STOP != 0,
#ifdef CONFIG_UART6_OFLOWCONTROL
    .usects    = true,
#endif
#ifdef CONFIG_UART6_IFLOWCONTROL
    .userts    = true,
#endif
  },
  .rx_pin = GPIO_UART6_RXD,
  .tx_pin = GPIO_UART6_TXD,
#ifdef CONFIG_UART6_OFLOWCONTROL
  .cts_pin = GPIO_UART6_CTS,
#endif
#ifdef CONFIG_UART6_IFLOWCONTROL
  .rts_pin = GPIO_UART6_RTS,
#endif
};

static struct uart_dev_s g_uart6_dev =
{
  .recv =
  {
    .size   = CONFIG_UART6_RXBUFSIZE,
    .buffer = g_uart6_rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART6_TXBUFSIZE,
    .buffer = g_uart6_txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart6_priv,
};
#endif

#ifdef CONFIG_HPM_UART7
static char g_uart7_rxbuffer[CONFIG_UART7_RXBUFSIZE];
static char g_uart7_txbuffer[CONFIG_UART7_TXBUFSIZE];

static struct hpm_uart_s g_uart7_priv =
{
  .uartbase = HPM_UART7_BASE,
  .irq      = HPM_IRQ_UART7,
  .lock     = SP_UNLOCKED,
  .config   =
  {
    .baud      = CONFIG_UART7_BAUD,
    .parity    = CONFIG_UART7_PARITY,
    .bits      = CONFIG_UART7_BITS,
    .stopbits2 = CONFIG_UART7_2STOP != 0,
#ifdef CONFIG_UART7_OFLOWCONTROL
    .usects    = true,
#endif
#ifdef CONFIG_UART7_IFLOWCONTROL
    .userts    = true,
#endif
  },
  .rx_pin = GPIO_UART7_RXD,
  .tx_pin = GPIO_UART7_TXD,
#ifdef CONFIG_UART7_OFLOWCONTROL
  .cts_pin = GPIO_UART7_CTS,
#endif
#ifdef CONFIG_UART7_IFLOWCONTROL
  .rts_pin = GPIO_UART7_RTS,
#endif
};

static struct uart_dev_s g_uart7_dev =
{
  .recv =
  {
    .size   = CONFIG_UART7_RXBUFSIZE,
    .buffer = g_uart7_rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART7_TXBUFSIZE,
    .buffer = g_uart7_txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart7_priv,
};
#endif

static struct uart_dev_s * const g_uart_devices[8] =
{
#ifdef CONFIG_HPM_UART0
  [0] = &g_uart0_dev,
#endif
#ifdef CONFIG_HPM_UART1
  [1] = &g_uart1_dev,
#endif
#ifdef CONFIG_HPM_UART2
  [2] = &g_uart2_dev,
#endif
#ifdef CONFIG_HPM_UART3
  [3] = &g_uart3_dev,
#endif
#ifdef CONFIG_HPM_UART4
  [4] = &g_uart4_dev,
#endif
#ifdef CONFIG_HPM_UART5
  [5] = &g_uart5_dev,
#endif
#ifdef CONFIG_HPM_UART6
  [6] = &g_uart6_dev,
#endif
#ifdef CONFIG_HPM_UART7
  [7] = &g_uart7_dev,
#endif
};

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV (&g_uart0_dev)
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV (&g_uart1_dev)
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV (&g_uart2_dev)
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_DEV (&g_uart3_dev)
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  define CONSOLE_DEV (&g_uart4_dev)
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#  define CONSOLE_DEV (&g_uart5_dev)
#elif defined(CONFIG_UART6_SERIAL_CONSOLE)
#  define CONSOLE_DEV (&g_uart6_dev)
#elif defined(CONFIG_UART7_SERIAL_CONSOLE)
#  define CONSOLE_DEV (&g_uart7_dev)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_serialin
 ****************************************************************************/

static inline uint32_t hpm_serialin(struct hpm_uart_s *priv,
                                   uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: hpm_serialout
 ****************************************************************************/

static inline void hpm_serialout(struct hpm_uart_s *priv,
                                uint32_t offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: hpm_read_lsr
 *
 * Description:
 *   Read line status and preserve observed receive errors.
 *   The caller must not already hold priv->lock.
 *
 ****************************************************************************/

static uint32_t hpm_read_lsr(struct hpm_uart_s *priv)
{
  irqstate_t flags;
  uint32_t status;

  flags = spin_lock_irqsave(&priv->lock);
  status = hpm_serialin(priv, HPM_UART_LSR_OFFSET);
  priv->rxstatus |= status & HPM_UART_RX_ERRORS;
  spin_unlock_irqrestore(&priv->lock, flags);

  return status;
}

/****************************************************************************
 * Name: hpm_rxavailable
 ****************************************************************************/

static bool hpm_rxavailable(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = dev->priv;

  return (hpm_read_lsr(priv) & UART_LSR_DR) != 0;
}

/****************************************************************************
 * Name: hpm_txready
 ****************************************************************************/

static bool hpm_txready(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = dev->priv;

  return (hpm_read_lsr(priv) & UART_LSR_THRE) != 0;
}

/****************************************************************************
 * Name: hpm_txempty
 ****************************************************************************/

static bool hpm_txempty(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = dev->priv;

  return (hpm_read_lsr(priv) & UART_LSR_TEMT) != 0;
}

/****************************************************************************
 * Name: hpm_send
 *
 * Description:
 *   Send one byte after the serial framework has checked txready().
 *   Newline translation and polled waiting do not belong in this callback.
 *
 ****************************************************************************/

static void hpm_send(struct uart_dev_s *dev, int ch)
{
  struct hpm_uart_s *priv = dev->priv;

  hpm_serialout(priv, HPM_UART_THR_OFFSET, (uint8_t)ch);
}

/****************************************************************************
 * Name: hpm_receive
 *
 * Description:
 *   Receive one available byte and report the observed line status.
 *   The serial framework checks rxavailable() before calling this method.
 *
 ****************************************************************************/

static int hpm_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct hpm_uart_s *priv = dev->priv;
  irqstate_t flags;
  uint32_t observed;
  uint32_t value;

  flags = spin_lock_irqsave(&priv->lock);

  observed = hpm_serialin(priv, HPM_UART_LSR_OFFSET);
  observed |= priv->rxstatus;

  value = hpm_serialin(priv, HPM_UART_RBR_OFFSET);
  priv->rxstatus = 0;

  spin_unlock_irqrestore(&priv->lock, flags);

  if (status != NULL)
    {
      *status = observed;
    }

  return (int)(value & UART_RBR_RBR_MASK);
}

/****************************************************************************
 * Name: hpm_update_uart_interrupts
 *
 * Description:
 *   Update the software interrupt mask and IER under the same lock.
 *   The caller must not already hold priv->lock. DLAB must be clear.
 *
 ****************************************************************************/

static void hpm_update_uart_interrupts(struct hpm_uart_s *priv,
                                      uint32_t clearbits,
                                      uint32_t setbits)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  priv->im = (priv->im & ~clearbits) | setbits;
  hpm_serialout(priv, HPM_UART_IER_OFFSET, priv->im);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: hpm_rxint
 ****************************************************************************/

static void hpm_rxint(struct uart_dev_s *dev, bool enable)
{
  struct hpm_uart_s *priv = dev->priv;
  uint32_t bits = UART_IER_ERBI | UART_IER_ELSI;
  irqstate_t flags;

#ifdef CONFIG_SUPPRESS_SERIAL_INTS
  enable = false;
#endif

  flags = spin_lock_irqsave(&priv->lock);
  priv->rxenabled = enable;
  priv->im &= ~bits;
  if (enable && !priv->rxpaused)
    {
      priv->im |= bits;
    }

  hpm_serialout(priv, HPM_UART_IER_OFFSET, priv->im);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: hpm_txint
 ****************************************************************************/

static void hpm_txint(struct uart_dev_s *dev, bool enable)
{
  struct hpm_uart_s *priv = dev->priv;
  irqstate_t flags;

#ifdef CONFIG_SUPPRESS_SERIAL_INTS
  enable = false;
#endif

  /* Start transmission without holding the framework callbacks' lock. */

  flags = enter_critical_section();
  hpm_update_uart_interrupts(priv, UART_IER_ETHEI,
                            enable ? UART_IER_ETHEI : 0);
  if (enable)
    {
      uart_xmitchars(dev);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm_setup
 ****************************************************************************/

static int hpm_setup(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = dev->priv;
  irqstate_t flags;
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  const struct hpm_gpio_config_s *pins[4] =
  {
    priv->rx_pin, priv->tx_pin, priv->cts_pin, priv->rts_pin
  };

  struct uart_config_s config = priv->config;
  unsigned int i;
  int ret;

  if (priv->rx_pin == NULL || priv->tx_pin == NULL ||
      (config.usects && priv->cts_pin == NULL) ||
      (config.userts && priv->rts_pin == NULL))
    {
      return -EINVAL;
    }

  if (priv->rx_pin->mode != HPM_GPIO_PERIPHERAL ||
      priv->tx_pin->mode != HPM_GPIO_PERIPHERAL ||
      (config.usects && priv->cts_pin->mode != HPM_GPIO_PERIPHERAL) ||
      (config.userts && priv->rts_pin->mode != HPM_GPIO_PERIPHERAL &&
       priv->rts_pin->mode != HPM_GPIO_OUTPUT))
    {
      return -EINVAL;
    }

  for (i = 0; i < 4; i++)
    {
      if (pins[i] != NULL)
        {
          ret = hpm_gpio_config(pins[i]);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  config.frequency = hpm_uart_clockconfig(priv->uartbase);
  if (config.frequency == 0)
    {
      return -EIO;
    }

  /* GPIO-based RTS is managed by rxflowcontrol, not by the UART MCR. */

  if (config.userts && priv->rts_pin->mode == HPM_GPIO_OUTPUT)
    {
      config.userts = false;
    }

  ret = hpm_uart_configure(priv->uartbase, &config);
  if (ret < 0)
    {
      return ret;
    }

  priv->config.frequency = config.frequency;
  hpm_serialout(priv, HPM_UART_FCR_OFFSET,
                UART_FCR_FIFOE | UART_FCR_RXFIFORST | UART_FCR_TXFIFORST);

  if (priv->config.userts && priv->rts_pin->mode == HPM_GPIO_OUTPUT)
    {
      ret = hpm_gpio_write(priv->rts_pin, false);
      if (ret < 0)
        {
          return ret;
        }
    }
#endif

  flags = spin_lock_irqsave(&priv->lock);
  priv->im = 0;
  priv->rxstatus = 0;
  priv->rxenabled = false;
  priv->rxpaused = false;
  hpm_serialout(priv, HPM_UART_IER_OFFSET, 0);
  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: hpm_shutdown
 ****************************************************************************/

static void hpm_shutdown(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = dev->priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  priv->im = 0;
  priv->rxenabled = false;
  priv->rxpaused = false;
  hpm_serialout(priv, HPM_UART_IER_OFFSET, 0);
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  hpm_serialout(priv, HPM_UART_FCR_OFFSET,
                UART_FCR_RXFIFORST | UART_FCR_TXFIFORST);
  modifyreg32(priv->uartbase + HPM_UART_MCR_OFFSET,
              UART_MCR_AFE | UART_MCR_RTS, 0);
#endif
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: hpm_attach
 ****************************************************************************/

static int hpm_attach(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = dev->priv;
  int ret = irq_attach(priv->irq, hpm_interrupt, dev);

  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: hpm_detach
 ****************************************************************************/

static void hpm_detach(struct uart_dev_s *dev)
{
  struct hpm_uart_s *priv = dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: hpm_ioctl
 ****************************************************************************/

static int hpm_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  /* Dynamic termios support is not advertised by the HPM5300 Kconfig. */

  return -ENOTTY;
}

/****************************************************************************
 * Name: hpm_rxflowcontrol
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool hpm_rxflowcontrol(struct uart_dev_s *dev,
                             unsigned int nbuffered, bool upper)
{
  struct hpm_uart_s *priv = dev->priv;
  uint32_t bits = UART_IER_ERBI | UART_IER_ELSI;
  irqstate_t flags;

  if (!priv->config.userts || priv->rts_pin == NULL)
    {
      return false;
    }

  flags = spin_lock_irqsave(&priv->lock);
  if (priv->rts_pin->mode == HPM_GPIO_OUTPUT)
    {
      if (hpm_gpio_write(priv->rts_pin, upper) < 0)
        {
          spin_unlock_irqrestore(&priv->lock, flags);
          return false;
        }
    }
  else
    {
      modifyreg32(priv->uartbase + HPM_UART_MCR_OFFSET, UART_MCR_RTS,
                  upper ? 0 : UART_MCR_RTS);
    }

  /* Stop RX IRQs while the software ring is full, not just the peer. This
   * prevents a level-triggered interrupt storm from bytes still in the FIFO.
   */

  priv->rxpaused = upper;
  priv->im &= ~bits;
  if (priv->rxenabled && !priv->rxpaused)
    {
      priv->im |= bits;
    }

  hpm_serialout(priv, HPM_UART_IER_OFFSET, priv->im);
  spin_unlock_irqrestore(&priv->lock, flags);
  return upper;
}
#endif

/****************************************************************************
 * Name: hpm_interrupt
 ****************************************************************************/

static int hpm_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = arg;
  struct hpm_uart_s *priv = dev->priv;
  unsigned int pass;
  uint32_t reason;

  /* Framework callbacks acquire priv->lock. Do not hold it here. */

  for (pass = 0; pass < HPM_UART_ISR_LIMIT; pass++)
    {
      reason = hpm_serialin(priv, HPM_UART_IIR_OFFSET) &
               UART_IIR_INTRID_MASK;

      switch (reason)
        {
          case UART_IIR_INTRID_NONE:
            return OK;

          case UART_IIR_INTRID_RX_STATE:
            hpm_read_lsr(priv);
            uart_recvchars(dev);
            break;

          case UART_IIR_INTRID_RX_AVAILE:
          case UART_IIR_INTRID_RX_TIMEOUT:
            uart_recvchars(dev);
            break;

          case UART_IIR_INTRID_TX_AVAILE:
            uart_xmitchars(dev);
            break;

          case UART_IIR_INTRID_MODEM_STATE:
            hpm_serialin(priv, HPM_UART_MSR_OFFSET);
            break;

          default:

            /* Writes to the IIR address access FCR, not interrupt status. */

            hpm_update_uart_interrupts(priv, UINT32_MAX, 0);
            return -EIO;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_earlyserialinit
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void riscv_earlyserialinit(void)
{
#ifdef CONSOLE_DEV
  CONSOLE_DEV->isconsole = true;
  DEBUGVERIFY(hpm_setup(CONSOLE_DEV));
#endif
}
#endif

/****************************************************************************
 * Name: riscv_serialinit
 ****************************************************************************/

void riscv_serialinit(void)
{
  unsigned int i;
  char name[] = "/dev/ttyS0";

#ifdef CONSOLE_DEV
  DEBUGVERIFY(uart_register("/dev/console", CONSOLE_DEV));
#endif

  /* Preserve hardware port numbers, including gaps between enabled ports. */

  for (i = 0; i < 8; i++)
    {
      if (g_uart_devices[i] != NULL)
        {
          name[9] = '0' + i;
          DEBUGVERIFY(uart_register(name, g_uart_devices[i]));
        }
    }
}
#elif defined(USE_SERIALDRIVER)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* No hardware UARTs were selected. Other console classes may be registered
 * by their own drivers; there are no UART devices to register here.
 */

#ifdef USE_EARLYSERIALINIT
void riscv_earlyserialinit(void)
{
}
#endif

void riscv_serialinit(void)
{
}
#endif /* USE_SERIALDRIVER && HAVE_UART_DEVICE */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Poll the configured console without depending on the scheduler or UART
 *   interrupts. Preserve observed receive errors and restore the UART mask.
 ****************************************************************************/

void up_putc(int ch)
{
#if defined(USE_SERIALDRIVER) && defined(CONSOLE_DEV)
  struct hpm_uart_s *priv = CONSOLE_DEV->priv;
  irqstate_t flags;
  uint32_t saved;
  uint32_t status;
  unsigned int i;
  unsigned int count = ch == '\n' ? 2 : 1;

  flags = spin_lock_irqsave(&priv->lock);
  saved = hpm_serialin(priv, HPM_UART_IER_OFFSET);
  hpm_serialout(priv, HPM_UART_IER_OFFSET, 0);

  for (i = 0; i < count; i++)
    {
      do
        {
          status = hpm_serialin(priv, HPM_UART_LSR_OFFSET);
          priv->rxstatus |= status & HPM_UART_RX_ERRORS;
        }
      while ((status & UART_LSR_THRE) == 0);

      hpm_serialout(priv, HPM_UART_THR_OFFSET,
                    count == 2 && i == 0 ? '\r' : (uint8_t)ch);
    }

  hpm_serialout(priv, HPM_UART_IER_OFFSET, saved);
  spin_unlock_irqrestore(&priv->lock, flags);
#else
  hpm_lowputc(ch);
#endif
}
