/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_dma.c
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
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#ifdef CONFIG_PM
#  include <nuttx/power/pm.h>
#endif

#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>

#include <arch/barriers.h>

#include "hpm_dma.h"
#include "hpm_clockconfig.h"
#include "hardware/hpm_memorymap.h"
#include "riscv_internal.h"
#include "hardware/hpm_dma.h"
#include "hardware/hpm_dmamux.h"
#include "hardware/hpm_csr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CTRL interrupt-mask bits: writing one masks the corresponding event. These
 * are hardware control bits, not the public HPM_DMA_EVENT_* values.
 */

#define HPM_DMA_CTRL_IRQ_MASK \
  (HPM_DMA_CH_CTRL_INTHALFCNTMASK | HPM_DMA_CH_CTRL_INTABTMASK | \
   HPM_DMA_CH_CTRL_INTERRMASK | HPM_DMA_CH_CTRL_INTTCMASK)

#define HPM_DMA_MEMORY_SOURCE(d) \
  ((d) == HPM_DMA_MEM_TO_MEM || (d) == HPM_DMA_MEM_TO_PERIPH)
#define HPM_DMA_MEMORY_DESTINATION(d) \
  ((d) == HPM_DMA_MEM_TO_MEM || (d) == HPM_DMA_PERIPH_TO_MEM)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Channel lifecycle. FREE has no owner; ALLOCATED has an owner but needs
 * setup; CONFIGURED is ready to start; RUNNING has an active transfer.
 * STOPPING is cancellation in progress; FAULT requires a confirmed stop
 * before reuse. Completion consumes setup and returns to ALLOCATED.
 */

/* Inclusive byte-address span touched by one endpoint. first is the lowest
 * accessed address, which differs from the initial DMA address in decrement
 * mode. This describes numeric coverage only, not DMA accessibility.
 */

struct hpm_dma_span_s
{
  uintptr_t first;
  uintptr_t last;
};

/* A validated transfer independent of channel ownership. The same compiler
 * is used for direct, repeated and descriptor-based transfers.
 */

struct hpm_dma_transfer_s
{
  struct hpm_dma_config_s config;
  uintptr_t src_busaddr;
  uintptr_t dst_busaddr;
  uint32_t ctrl;
  uint32_t ntransfers;
  struct hpm_dma_span_s src_span;
  struct hpm_dma_span_s dst_span;
  struct hpm_dma_span_s dst_cache_span;
  bool cache_pending;
  bool source_ram;
  uint8_t ownership;
  uint8_t request_port;
  uint64_t sequence;
  size_t next_ready;
};

/* HPM5361 DMA V2 descriptor, not the older 64-bit-address layout shown in UM
 * table 161. Word 3 is CHANREQCTRL; words 5 and 7 are reserved on HPM5361.
 * Use the register layout and the SDK's dma_linked_descriptor_t. Alignment
 * and stride also ensure that no descriptor crosses a 4 KiB boundary.
 */

struct hpm_dma_desc_s
{
  uint32_t ctrl;
  uint32_t ntransfers;
  uint32_t src_addr;
  uint32_t request;
  uint32_t dst_addr;
  uint32_t reserved0;
  uint32_t next;
  uint32_t reserved1;
};

static_assert(sizeof(struct hpm_dma_desc_s) == 32,
              "DMA descriptor size must be 32 bytes");
static_assert(offsetof(struct hpm_dma_desc_s, ntransfers) == 0x04,
              "DMA descriptor count offset");
static_assert(offsetof(struct hpm_dma_desc_s, src_addr) == 0x08,
              "DMA descriptor source offset");
static_assert(offsetof(struct hpm_dma_desc_s, request) == 0x0c,
              "DMA descriptor request offset");
static_assert(offsetof(struct hpm_dma_desc_s, dst_addr) == 0x10,
              "DMA descriptor destination offset");
static_assert(offsetof(struct hpm_dma_desc_s, reserved0) == 0x14,
              "DMA descriptor reserved word offset");
static_assert(offsetof(struct hpm_dma_desc_s, next) == 0x18,
              "DMA descriptor link offset");
static_assert(offsetof(struct hpm_dma_desc_s, reserved1) == 0x1c,
              "DMA descriptor final word offset");

struct hpm_dma_routes_s
{
  uint32_t mask;
  uint8_t source[HPM_DMAMUX_NCHANNELS];
};

/* The driver owns both allocations. No descriptor or metadata is changed
 * while hardware/callbacks can reference it. Descriptors are DMA-readable;
 * software metadata need not be. Heap operations are task-context only.
 */

struct hpm_dma_list_s
{
  size_t count;
  size_t desc_bytes;
  uintptr_t desc_busaddr;
  struct hpm_dma_desc_s *desc;
  struct hpm_dma_span_s *sources;
  struct hpm_dma_span_s *destinations;
  struct hpm_dma_span_s *invalidations;
  size_t nsources;
  size_t ndestinations;
  size_t ninvalidations;
  bool cache_pending;
  struct hpm_dma_transfer_s items[];
};

enum hpm_dma_ownership_e
{
  HPM_DMA_BUFFER_QUEUED = 0,
  HPM_DMA_BUFFER_ACTIVE,
  HPM_DMA_BUFFER_READY,
  HPM_DMA_BUFFER_CPU,
  HPM_DMA_BUFFER_PREPARING
};

/* Lock order: channel mutex, then controller spinlock. The mutex serializes
 * task operations and pins the list during setup/start/stop/free. Deferred
 * work_active pins it through cache operations and callback return.
 * Do not hold the spinlock across heap/cache operations, waits or callbacks.
 */

struct hpm_dmach_s
{
  mutex_t lock;
  uint8_t channel;
  enum hpm_dma_state_e state;
  enum hpm_dma_mode_e mode;
  struct hpm_dma_transfer_s transfer;
  struct hpm_dma_list_s *list;
  hpm_dma_callback_t callback;
  void *arg;
  bool callback_active;
  struct work_s work;
  bool work_active;
  bool waiting_work;
  sem_t work_sem;
  bool configuration_valid;
  uint32_t pending_events;
  uint32_t last_events;
  int result;
  clock_t terminal_start;
  uint64_t generation;
  uint64_t observed_half;
  uint64_t observed_periods;
  uint64_t completed_periods;
  uint32_t stop_timeouts;
  uint32_t queue_errors;
  size_t stream_current;
  size_t loans;
  size_t ready_head;
  size_t ready_tail;
  size_t ready_count;
  struct hpm_dma_fault_s fault;
  struct hpm_dma_routes_s routes;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static struct hpm_dmach_s *hpm_dma_from_handle(hpm_dma_handle_t handle);

static void hpm_dma_clear_status(unsigned int channel);

static int hpm_dma_validate_basic(const struct hpm_dma_config_s *config);

static int hpm_dma_transfer_geometry(const struct hpm_dma_config_s *config,
                                     uint32_t *ntransfers);

static int hpm_dma_address_span(uintptr_t address, size_t nbytes,
                                enum hpm_dma_width_e width,
                                enum hpm_dma_addrmode_e mode,
                                struct hpm_dma_span_s *span);

static bool hpm_dma_span_in_region(const struct hpm_dma_span_s *span,
                                   uintptr_t base, size_t size);

static int hpm_dma_map_ram(uintptr_t address,
                           const struct hpm_dma_span_s *span,
                           uintptr_t *busaddr);

static void hpm_dma_finish_all(struct hpm_dmach_s *ch);
static void hpm_dma_worker(void *arg);
static int hpm_dma_task_context(void);

#ifdef CONFIG_PM
static int hpm_dma_pm_prepare(struct pm_callback_s *cb, int domain,
                              enum pm_state_e state);
static void hpm_dma_pm_notify(struct pm_callback_s *cb, int domain,
                              enum pm_state_e state);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Controller lock protects allocation, channel state and callback ownership
 * shared by task context and the interrupt handler. Use irqsave operations;
 * do not hold this lock during waits, cache maintenance or client callbacks.
 */

static spinlock_t g_dma_lock = SP_UNLOCKED;

/* Statically allocated channel pool. Initialization must assign channel
 * numbers and initialize each mutex before publishing controller readiness.
 * A channel allocation grants ownership of an existing object, not heap
 * memory. Zero-filled storage alone does not initialize the mutexes.
 */

static struct hpm_dmach_s g_dma_channels[HPM_DMA_NCHANNELS];

/* Publish readiness only after clocks, hardware state, all channel objects
 * and the interrupt handler have been initialized successfully. Read/write
 * this flag under the controller lock once concurrent access is possible.
 */

static bool g_dma_initialized;
/* Watched channels include COMPLETING until bus drain, so late errors are
 * not lost. Ports include both committed plans and in-flight reservations.
 */

static uint32_t g_dma_running;
static uint32_t g_dma_ports;
static mutex_t g_dma_queue_lock = NXMUTEX_INITIALIZER;
static struct kwork_wqueue_s *g_dma_wqueue;
static pid_t g_dma_worker_pid = -1;
static bool g_dma_allocating;
#ifdef CONFIG_PM
static bool g_dma_pm_blocked;
static bool g_dma_restore_needed;
static struct pm_callback_s g_dma_pm =
{
  .prepare = hpm_dma_pm_prepare,
  .notify = hpm_dma_pm_notify
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_dma_from_handle
 *
 * Description:
 *   Match a handle against the static channel pool without dereferencing the
 *   supplied pointer. Equality comparisons avoid relational comparisons or
 *   pointer subtraction involving unrelated objects.
 *
 *   This checks pool membership only. Initialization, allocation state and
 *   callback ownership must still be checked by callers under the proper
 *   locks. It does not establish ownership of a previously released handle.
 *
 * Returned Value:
 *   The matching channel object, or NULL if the handle is not a pool entry.
 *
 ****************************************************************************/

static struct hpm_dmach_s *hpm_dma_from_handle(hpm_dma_handle_t handle)
{
  unsigned int i;

  for (i = 0; i < HPM_DMA_NCHANNELS; i++)
    {
      if (handle == &g_dma_channels[i])
        {
          return &g_dma_channels[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: hpm_dma_clear_status
 *
 * Description:
 *   Clear residual status for one validated, quiescent channel before start
 *   or after confirmed stop. Write only that channel's bit to each register.
 *   Call while controller/channel ownership is protected.
 *
 *   Do not use this blanket clear for an active channel's ISR: acknowledge
 *   only the status actually observed, otherwise a newly arriving terminal
 *   event could be lost. No controller reset or other-channel clear occurs.
 *
 *   Follow the SDK's write-one clear for INTHALFSTS. The Rev1.2 manual
 * labels
 *   that field RW without a clear description; verify half-status clearing
 *   during hardware bring-up. Other terminal status fields document W1C.
 *
 ****************************************************************************/

static void hpm_dma_clear_status(unsigned int channel)
{
  uint32_t bit = HPM_DMA_CHANNEL_BIT(channel);

  putreg32(bit, HPM_DMA_INTHALFSTS);
  putreg32(bit, HPM_DMA_INTTCSTS);
  putreg32(bit, HPM_DMA_INTABORTSTS);
  putreg32(bit, HPM_DMA_INTERRSTS);
}

/****************************************************************************
 * Name: hpm_dma_validate_basic
 *
 * Description:
 *   Validate scalar configuration fields without accessing hardware or data
 *   buffers. Reject invalid enum values, zero lengths, unsupported burst
 *   counts and reserved peripheral request-source IDs. Memory-to-memory
 *   transfers ignore request as specified by the public interface.
 *
 *   This is only the first validation stage: address translation, DMA
 *   accessibility, alignment, range overflow and hardware transfer count
 *   limits must also pass before channel registers can be programmed.
 *
 * Returned Value:
 *   Zero on success, or -EINVAL for invalid basic configuration fields.
 *
 ****************************************************************************/

static int hpm_dma_validate_basic(const struct hpm_dma_config_s *config)
{
  if (config == NULL ||
      (unsigned int)config->direction > HPM_DMA_PERIPH_TO_PERIPH ||
      (unsigned int)config->src_width > HPM_DMA_WIDTH_32BIT ||
      (unsigned int)config->dst_width > HPM_DMA_WIDTH_32BIT ||
      (unsigned int)config->src_addrmode > HPM_DMA_ADDR_FIXED ||
      (unsigned int)config->dst_addrmode > HPM_DMA_ADDR_FIXED)
    {
      return -EINVAL;
    }

  if (config->nbytes == 0 ||
      config->burst == 0 ||
      config->burst > HPM_DMA_MAX_BURST ||
      (config->burst > 16 &&
       (config->burst & (config->burst - 1U)) != 0))
    {
      return -EINVAL;
    }

  if (config->direction != HPM_DMA_MEM_TO_MEM && !config->software_trigger &&
      !HPM_DMAMUX_SOURCE_VALID(config->request))
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: hpm_dma_transfer_geometry
 *
 * Description:
 *   Validate basic fields first, then endpoint alignment, byte-count
 *   alignment and the hardware source-transfer count. No hardware or buffer
 *   access occurs. A non-NULL count output is set to zero on failure.
 *
 *   Follow the vendor setup restriction that source width is not narrower
 *   than destination width. Other width-conversion combinations are not
 *   supported by this implementation without further hardware validation.
 *   Address span/overflow, DMA accessibility and bus-address translation
 *   remain separate checks required before programming a channel.
 *
 * Returned Value:
 *   Zero on success, -EINVAL for invalid fields/alignment, -ENOTSUP for an
 *   unsupported width combination, or -E2BIG for excessive transfer count.
 *
 ****************************************************************************/

static int hpm_dma_transfer_geometry(const struct hpm_dma_config_s *config,
                                     uint32_t *ntransfers)
{
  unsigned int src_bytes;
  unsigned int dst_bytes;
  size_t count;
  int ret;

  if (ntransfers == NULL)
    {
      return -EINVAL;
    }

  *ntransfers = 0;

  ret = hpm_dma_validate_basic(config);
  if (ret < 0)
    {
      return ret;
    }

  src_bytes = 1U << config->src_width;
  dst_bytes = 1U << config->dst_width;

  if (src_bytes < dst_bytes)
    {
      return -ENOTSUP;
    }

  if ((config->src_addr & (src_bytes - 1U)) != 0 ||
      (config->dst_addr & (dst_bytes - 1U)) != 0 ||
      (config->nbytes & (src_bytes - 1U)) != 0 ||
      (config->nbytes & (dst_bytes - 1U)) != 0)
    {
      return -EINVAL;
    }

  count = config->nbytes / src_bytes;
  if (count > HPM_DMA_CH_TRANSIZE_MASK)
    {
      return -E2BIG;
    }

  *ntransfers = (uint32_t)count;
  return OK;
}

/****************************************************************************
 * Name: hpm_dma_address_span
 *
 * Description:
 *   Compute the inclusive byte span touched by an incrementing, decrementing
 *   or fixed endpoint. Validate width, mode, alignment and integer overflow
 *   without touching memory or hardware. Clear a non-NULL output on failure.
 *   Fixed addressing covers only one transfer-width unit, regardless of the
 *   total byte count; decrementing addressing starts at the supplied address
 *   and steps down by one transfer-width unit after each access.
 *
 *   This does not validate a memory region or translate CPU addresses. Those
 *   checks remain required before programming the DMA controller.
 *
 * Returned Value:
 *   Zero on success, -EINVAL for invalid fields/alignment or -EOVERFLOW if
 *   the computed byte span would wrap around the address space.
 *
 ****************************************************************************/

static int hpm_dma_address_span(uintptr_t address, size_t nbytes,
                                enum hpm_dma_width_e width,
                                enum hpm_dma_addrmode_e mode,
                                struct hpm_dma_span_s *span)
{
  unsigned int bytes;
  uintptr_t first;
  size_t span_bytes;

  if (span == NULL)
    {
      return -EINVAL;
    }

  span->first = 0;
  span->last  = 0;

  if (nbytes == 0 ||
      (unsigned int)width > HPM_DMA_WIDTH_32BIT ||
      (unsigned int)mode > HPM_DMA_ADDR_FIXED)
    {
      return -EINVAL;
    }

  bytes = 1U << width;

  if ((address & (bytes - 1U)) != 0 ||
      (nbytes & (bytes - 1U)) != 0)
    {
      return -EINVAL;
    }

  first = address;
  span_bytes = mode == HPM_DMA_ADDR_FIXED ? bytes : nbytes;

  if (mode == HPM_DMA_ADDR_DECREMENT)
    {
      if (nbytes - bytes > address)
        {
          return -EOVERFLOW;
        }

      first = address - (nbytes - bytes);
    }

  if (span_bytes - 1 > UINTPTR_MAX - first)
    {
      return -EOVERFLOW;
    }

  span->first = first;
  span->last  = first + (span_bytes - 1);

  return OK;
}

/****************************************************************************
 * Name: hpm_dma_span_in_region
 *
 * Description:
 *   Check whether an inclusive span lies entirely in a nonempty region.
 *   Reject malformed spans and regions whose final byte would overflow.
 *   This checks numeric containment only, not DMA access permissions.
 *
 * Returned Value:
 *   True if the entire span is contained in the region; false otherwise.
 *
 ****************************************************************************/

static bool hpm_dma_span_in_region(const struct hpm_dma_span_s *span,
                                   uintptr_t base, size_t size)
{
  if (span == NULL || size == 0)
    {
      return false;
    }

  if (span->first > span->last ||
      size - 1 > UINTPTR_MAX - base)
    {
      return false;
    }

  return span->first >= base &&
         span->last <= base + (size - 1);
}

/****************************************************************************
 * Name: hpm_dma_map_ram
 *
 * Description:
 *   Validate that an endpoint and its complete inclusive span lie in one
 *   supported on-chip RAM window. HPM5361 HDMA uses the same system
 * addresses
 *   as the CPU for these windows; preserve the caller's selected alias.
 *   Peripheral and flash endpoints require separate validation policies.
 *   This does not perform cache maintenance or transfer buffer ownership.
 *
 * Returned Value:
 *   Zero on success, -EINVAL for invalid arguments or -EFAULT for a span
 *   outside the supported RAM windows. Clear a non-NULL output on failure.
 *
 ****************************************************************************/

static int hpm_dma_map_ram(uintptr_t address,
                           const struct hpm_dma_span_s *span,
                           uintptr_t *busaddr)
{
  bool valid;

  if (busaddr == NULL)
    {
      return -EINVAL;
    }

  *busaddr = 0;

  if (span == NULL || span->first > span->last ||
      address < span->first || address > span->last)
    {
      return -EINVAL;
    }

  valid =
    hpm_dma_span_in_region(span, HPM_ILM_BASE, HPM_ILM_SIZE) ||
    hpm_dma_span_in_region(span, HPM_ILM_ALIAS_BASE, HPM_ILM_SIZE) ||
    hpm_dma_span_in_region(span, HPM_ILM_SYSTEM_BASE, HPM_ILM_SIZE) ||
    hpm_dma_span_in_region(span, HPM_DLM_BASE, HPM_DLM_SIZE) ||
    hpm_dma_span_in_region(span, HPM_DLM_ALIAS_BASE, HPM_DLM_SIZE) ||
    hpm_dma_span_in_region(span, HPM_DLM_SYSTEM_BASE, HPM_DLM_SIZE) ||
    hpm_dma_span_in_region(span, HPM_HRAM_BASE, HPM_HRAM_SIZE);

  if (!valid)
    {
      return -EFAULT;
    }

  *busaddr = address;
  return OK;
}

/****************************************************************************
 * Name: hpm_dma_map_source
 *
 * Description:
 *   Accept readable RAM, ROM and the mapped XPI memory aperture.
 *
 ****************************************************************************/

static int hpm_dma_map_source(uintptr_t address,
                              const struct hpm_dma_span_s *span,
                              uintptr_t *busaddr)
{
  int ret = hpm_dma_map_ram(address, span, busaddr);

  if (ret == -EFAULT &&
      (hpm_dma_span_in_region(span, HPM_ROM_BASE, HPM_ROM_SIZE) ||
       hpm_dma_span_in_region(span, HPM_XPI0_MEMORY_BASE,
                             HPM_XPI0_MEMORY_SIZE)))
    {
      *busaddr = address;
      return OK;
    }

  return ret;
}

/* DMA-accessible peripheral windows. Request routing is independent of the
 * data endpoint: a timer request can, for example, update a GPIO register.
 * Keep the endpoint within one instance; register semantics and request
 * selection belong to the client. DMA/DMAMUX control registers are not
 * exposed as data endpoints owned by another client.
 */

/****************************************************************************
 * Name: hpm_dma_peripheral
 *
 * Description:
 *   Check that a peripheral span belongs to a DMA-capable instance.
 *
 ****************************************************************************/

static bool hpm_dma_peripheral(const struct hpm_dma_span_s *span)
{
  static const uintptr_t bases[] =
  {
    HPM_GPTMR0_BASE, HPM_GPTMR1_BASE, HPM_GPTMR2_BASE, HPM_GPTMR3_BASE,
    HPM_UART0_BASE, HPM_UART1_BASE, HPM_UART2_BASE, HPM_UART3_BASE,
    HPM_UART4_BASE, HPM_UART5_BASE, HPM_UART6_BASE, HPM_UART7_BASE,
    HPM_I2C0_BASE, HPM_I2C1_BASE, HPM_I2C2_BASE, HPM_I2C3_BASE,
    HPM_SPI0_BASE, HPM_SPI1_BASE, HPM_SPI2_BASE, HPM_SPI3_BASE,
    HPM_MCAN0_BASE, HPM_MCAN1_BASE, HPM_MCAN2_BASE, HPM_MCAN3_BASE,
    HPM_QEI0_BASE, HPM_QEI1_BASE, HPM_QEO0_BASE, HPM_QEO1_BASE,
    HPM_MMC0_BASE, HPM_MMC1_BASE, HPM_PWM0_BASE, HPM_PWM1_BASE,
    HPM_SEI_BASE, HPM_PLB_BASE, HPM_TRGM0_BASE,
    HPM_ADC0_BASE, HPM_ADC1_BASE, HPM_DAC0_BASE, HPM_DAC1_BASE,
    HPM_XPI0_BASE, HPM_ACMP_BASE,
    HPM_GPIO0_BASE, HPM_PGPIO_BASE, HPM_CRC_BASE, HPM_TSNS_BASE,
    HPM_MBX0A_BASE, HPM_MBX0B_BASE, HPM_PTMR_BASE, HPM_PUART_BASE
  };

  unsigned int i;

  for (i = 0; i < sizeof(bases) / sizeof(bases[0]); i++)
    {
      if (hpm_dma_span_in_region(span, bases[i], 0x4000))
        {
          return true;
        }
    }

  return false;
}

/* Normalize RAM aliases only for physical-overlap checks. DMA registers
 * retain the original addresses. DMA is not a memmove implementation.
 */

/****************************************************************************
 * Name: hpm_dma_physical
 *
 * Description:
 *   Resolve RAM aliases for overlap detection only.
 *
 ****************************************************************************/

static uintptr_t hpm_dma_physical(uintptr_t address)
{
  struct hpm_dma_span_s byte =
  {
    address, address
  };

  if (hpm_dma_span_in_region(&byte, HPM_ILM_ALIAS_BASE, HPM_ILM_SIZE))
    {
      return HPM_ILM_BASE + address - HPM_ILM_ALIAS_BASE;
    }

  if (hpm_dma_span_in_region(&byte, HPM_ILM_SYSTEM_BASE, HPM_ILM_SIZE))
    {
      return HPM_ILM_BASE + address - HPM_ILM_SYSTEM_BASE;
    }

  if (hpm_dma_span_in_region(&byte, HPM_DLM_ALIAS_BASE, HPM_DLM_SIZE))
    {
      return HPM_DLM_BASE + address - HPM_DLM_ALIAS_BASE;
    }

  if (hpm_dma_span_in_region(&byte, HPM_DLM_SYSTEM_BASE, HPM_DLM_SIZE))
    {
      return HPM_DLM_BASE + address - HPM_DLM_SYSTEM_BASE;
    }

  return address;
}

/* Cached DMA destinations must occupy whole cache lines. Otherwise final
 * invalidation could discard unrelated dirty bytes. Clients must not touch
 * any alias of a buffer while it is owned by DMA. Reject nonrepresentable
 * exclusive ends before calling the cache API.
 */

/****************************************************************************
 * Name: hpm_dma_cache_validate
 *
 * Description:
 *   Check cache-line ownership and cache API address limits.
 *
 ****************************************************************************/

static int hpm_dma_cache_validate(struct hpm_dma_transfer_s *transfer)
{
  transfer->dst_cache_span = transfer->dst_span;
#ifdef CONFIG_ARCH_DCACHE
  size_t line = up_get_dcache_linesize();

  if (line == 0 || (line & (line - 1)) != 0)
    {
      return -ENOTSUP;
    }

  if (HPM_DMA_MEMORY_DESTINATION(transfer->config.direction) &&
      !transfer->config.dst_cache_exclusive &&
      ((transfer->dst_span.first & (line - 1)) != 0 ||
       transfer->dst_span.last == UINTPTR_MAX ||
       ((transfer->dst_span.last + 1) & (line - 1)) != 0))
    {
      return -EINVAL;
    }

  if (HPM_DMA_MEMORY_DESTINATION(transfer->config.direction))
    {
      uintptr_t ignored;
      transfer->dst_cache_span.first &= ~(uintptr_t)(line - 1);
      transfer->dst_cache_span.last |= line - 1;
      if (transfer->dst_cache_span.last == UINTPTR_MAX ||
          hpm_dma_map_ram(transfer->dst_cache_span.first,
                         &transfer->dst_cache_span, &ignored) < 0)
        {
          return -EINVAL;
        }
    }

  if (HPM_DMA_MEMORY_SOURCE(transfer->config.direction) &&
      transfer->src_span.last == UINTPTR_MAX)
    {
      return -EINVAL;
    }

#endif

  return OK;
}

#ifdef CONFIG_ARCH_DCACHE
/* A destination may also be a later source through another CPU alias. Keep
 * every documented view coherent without an O(n^2) cross-descriptor scan.
 * The supplied view is authoritative; conflicting dirty aliases must have
 * been synchronized by the caller before DMA ownership begins.
 */

/****************************************************************************
 * Name: hpm_dma_invalidate_views
 *
 * Description:
 *   Invalidate documented CPU aliases of a written physical RAM span.
 *
 ****************************************************************************/

static void hpm_dma_invalidate_views(const struct hpm_dma_span_s *span,
                                     bool supplied)
{
  static const uintptr_t ilm[] =
  {
    HPM_ILM_BASE, HPM_ILM_ALIAS_BASE, HPM_ILM_SYSTEM_BASE
  };

  static const uintptr_t dlm[] =
  {
    HPM_DLM_BASE, HPM_DLM_ALIAS_BASE, HPM_DLM_SYSTEM_BASE
  };

  const uintptr_t *views = NULL;
  uintptr_t first = hpm_dma_physical(span->first);
  uintptr_t last = hpm_dma_physical(span->last);
  uintptr_t base = 0;
  uintptr_t address;
  unsigned int i;

  if (first < HPM_ILM_BASE + HPM_ILM_SIZE)
    {
      views = ilm;
      base = HPM_ILM_BASE;
    }
  else if (first >= HPM_DLM_BASE && first < HPM_DLM_BASE + HPM_DLM_SIZE)
    {
      views = dlm;
      base = HPM_DLM_BASE;
    }

  if (views != NULL)
    {
      for (i = 0; i < 3; i++)
        {
          address = views[i] + first - base;
          if (address != span->first)
            {
              up_invalidate_dcache(address, views[i] + last - base + 1);
            }
        }
    }

  if (supplied)
    {
      up_invalidate_dcache(span->first, span->last + 1);
    }
}

#endif

/****************************************************************************
 * Name: hpm_dma_cache_prepare
 *
 * Description:
 *   Clean source data and flush destination data before DMA starts.
 *
 ****************************************************************************/

static void hpm_dma_cache_prepare(struct hpm_dma_transfer_s *transfer)
{
  if (transfer->source_ram)
    {
      up_clean_dcache(transfer->src_span.first, transfer->src_span.last + 1);
    }

  if (HPM_DMA_MEMORY_DESTINATION(transfer->config.direction))
    {
      up_flush_dcache(transfer->dst_cache_span.first,
                      transfer->dst_cache_span.last + 1);
#ifdef CONFIG_ARCH_DCACHE
      hpm_dma_invalidate_views(&transfer->dst_cache_span, false);
#endif
    }

  transfer->cache_pending = true;
}

/****************************************************************************
 * Name: hpm_dma_cache_finish
 *
 * Description:
 *   Invalidate destination data after hardware has finished accessing it.
 *
 ****************************************************************************/

static void hpm_dma_cache_finish(struct hpm_dma_transfer_s *transfer)
{
  if (!transfer->cache_pending)
    {
      return;
    }

  UP_DSB();
  if (HPM_DMA_MEMORY_DESTINATION(transfer->config.direction))
    {
#ifdef CONFIG_ARCH_DCACHE
      hpm_dma_invalidate_views(&transfer->dst_cache_span, true);
#endif
    }

  transfer->cache_pending = false;
}

/* CHEN is the hardware's enable bitmap. Never read-modify-write live CTRL:
 * hardware can clear ENABLE between the read and write, restarting a
 * channel.
 */

/****************************************************************************
 * Name: hpm_dma_enabled
 *
 * Description:
 *   Read the hardware enable bit for one channel.
 *
 ****************************************************************************/

static bool hpm_dma_enabled(struct hpm_dmach_s *ch)
{
  return (getreg32(HPM_DMA_CHEN) & HPM_DMA_CHANNEL_BIT(ch->channel)) != 0;
}

/* A cleared enable bit alone does not prove that the last bus access has
 * drained. Wait until this channel is no longer the engine's active channel,
 * or the engine reports idle. Other channels may continue transferring.
 */

/****************************************************************************
 * Name: hpm_dma_quiescent
 *
 * Description:
 *   Check that a disabled channel has drained its bus accesses.
 *
 ****************************************************************************/

static bool hpm_dma_quiescent(struct hpm_dmach_s *ch)
{
  uint32_t status;

  if (hpm_dma_enabled(ch))
    {
      return false;
    }

  status = getreg32(HPM_DMA_IDMISC);
  return HPM_DMA_IDMISC_DMASTATE_GET(status) == 0 ||
         HPM_DMA_IDMISC_CURCHAN_GET(status) != ch->channel;
}

/****************************************************************************
 * Name: hpm_dma_idle
 *
 * Description:
 *   Mask interrupts and disconnect a quiescent channel.
 *
 ****************************************************************************/

static void hpm_dma_idle(struct hpm_dmach_s *ch)
{
  putreg32(HPM_DMA_CTRL_IRQ_MASK, HPM_DMA_CH_CTRL(ch->channel));
  uint32_t ports = ch->routes.mask;
  while (ports != 0)
    {
      unsigned int port = __builtin_ctz(ports);
      ports &= ~HPM_DMA_CHANNEL_BIT(port);
      putreg32(0, HPM_DMAMUX_MUXCFG(port));
    }

  putreg32(0, HPM_DMA_CH_LLPOINTER(ch->channel));
  hpm_dma_clear_status(ch->channel);
}

/****************************************************************************
 * Name: hpm_dma_task_context
 *
 * Description:
 *   Reject IRQ callers and blocking control from the DMA worker itself.
 *
 ****************************************************************************/

static int hpm_dma_task_context(void)
{
  irqstate_t flags;
  bool worker;

  if (up_interrupt_context())
    {
      return -EPERM;
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  worker = nxsched_gettid() == g_dma_worker_pid;
  spin_unlock_irqrestore(&g_dma_lock, flags);
  return worker ? -EDEADLK : OK;
}

/* The channel mutex permits one waiter. The condition is always rechecked;
 * a semaphore post racing its timeout can therefore never grant false idle.
 */

static void hpm_dma_work_idle(struct hpm_dmach_s *ch)
{
  ch->work_active = false;
  if (ch->waiting_work)
    {
      ch->waiting_work = false;
      nxsem_post(&ch->work_sem);
    }
}

static int hpm_dma_wait_work(struct hpm_dmach_s *ch, clock_t start)
{
  irqstate_t flags;
  clock_t elapsed;

  for (; ; )
    {
      flags = spin_lock_irqsave(&g_dma_lock);
      if (!ch->work_active)
        {
          spin_unlock_irqrestore(&g_dma_lock, flags);
          return OK;
        }

      elapsed = clock_systime_ticks() - start;
      if (elapsed >= MSEC2TICK(CONFIG_HPM_HDMA_STOP_TIMEOUT))
        {
          spin_unlock_irqrestore(&g_dma_lock, flags);
          return -ETIMEDOUT;
        }

      ch->waiting_work = true;
      spin_unlock_irqrestore(&g_dma_lock, flags);
      nxsem_tickwait_uninterruptible(&ch->work_sem,
                   MSEC2TICK(CONFIG_HPM_HDMA_STOP_TIMEOUT) - elapsed);
      flags = spin_lock_irqsave(&g_dma_lock);
      ch->waiting_work = false;
      spin_unlock_irqrestore(&g_dma_lock, flags);
    }
}

/****************************************************************************
 * Name: hpm_dma_capture_fault
 *
 * Description:
 *   Preserve the first hardware fault snapshot before resetting registers.
 *
 ****************************************************************************/

static void hpm_dma_capture_fault(struct hpm_dmach_s *ch, int result)
{
  ch->result = result;
  if (!ch->fault.valid)
    {
      ch->fault.valid = true;
      ch->fault.result = result;
      ch->fault.events = ch->pending_events |
                        (result == -ECANCELED ? HPM_DMA_EVENT_ABORT :
                                               HPM_DMA_EVENT_ERROR);
      ch->fault.generation = ch->generation;
      ch->fault.ctrl = getreg32(HPM_DMA_CH_CTRL(ch->channel));
      ch->fault.remaining = getreg32(HPM_DMA_CH_TRANSIZE(ch->channel));
      ch->fault.src = getreg32(HPM_DMA_CH_SRCADDR(ch->channel));
      ch->fault.dst = getreg32(HPM_DMA_CH_DSTADDR(ch->channel));
      ch->fault.next = getreg32(HPM_DMA_CH_LLPOINTER(ch->channel));
    }
}

/****************************************************************************
 * Name: hpm_dma_schedule
 *
 * Description:
 *   Queue preallocated work while retaining its channel/list lifetime pin.
 *
 ****************************************************************************/

static int hpm_dma_schedule(struct hpm_dmach_s *ch, clock_t delay)
{
  int ret;

  ch->work_active = true;
  ret = work_queue_wq(g_dma_wqueue, &ch->work, hpm_dma_worker, ch, delay);
  if (ret < 0)
    {
      hpm_dma_work_idle(ch);
      ch->queue_errors++;
      hpm_dma_capture_fault(ch, ret);
      ch->state = HPM_DMA_STATE_FAULT;
      g_dma_running &= ~HPM_DMA_CHANNEL_BIT(ch->channel);
      if (hpm_dma_enabled(ch))
        {
          putreg32(HPM_DMA_CHANNEL_BIT(ch->channel), HPM_DMA_CHABORT);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: hpm_dma_launch
 *
 * Description:
 *   Write a complete register image for a validated, quiescent channel.
 *
 ****************************************************************************/

static void hpm_dma_launch(struct hpm_dmach_s *ch, size_t index)
{
  unsigned int n = ch->channel;
  uint32_t ports;
  unsigned int port;
  uintptr_t next = 0;

  hpm_dma_idle(ch);
  if (ch->list != NULL)
    {
      ch->transfer = ch->list->items[index];
      if (ch->mode != HPM_DMA_MODE_STREAM)
        {
          next = ch->list->desc[index].next;
        }
    }

  putreg32(ch->transfer.src_busaddr, HPM_DMA_CH_SRCADDR(n));
  putreg32(ch->transfer.dst_busaddr, HPM_DMA_CH_DSTADDR(n));
  putreg32(ch->transfer.ntransfers, HPM_DMA_CH_TRANSIZE(n));
  putreg32(next, HPM_DMA_CH_LLPOINTER(n));
  putreg32(HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_SET(ch->transfer.request_port) |
           HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_SET(ch->transfer.request_port),
           HPM_DMA_CH_CHANREQCTRL(n));
  ports = ch->mode == HPM_DMA_MODE_STREAM ? 0 : ch->routes.mask;
  while (ports != 0)
    {
      port = __builtin_ctz(ports);
      ports &= ~HPM_DMA_CHANNEL_BIT(port);
      putreg32(HPM_DMAMUX_MUXCFG_ENABLE |
               HPM_DMAMUX_MUXCFG_SOURCE_SET(ch->routes.source[port]),
               HPM_DMAMUX_MUXCFG(port));
    }

  if (ch->mode == HPM_DMA_MODE_STREAM &&
      ch->transfer.config.direction != HPM_DMA_MEM_TO_MEM &&
      !ch->transfer.config.software_trigger)
    {
      putreg32(HPM_DMAMUX_MUXCFG_ENABLE |
               HPM_DMAMUX_MUXCFG_SOURCE_SET(ch->transfer.config.request),
               HPM_DMAMUX_MUXCFG(ch->transfer.request_port));
    }

  ch->state = HPM_DMA_STATE_RUNNING;
  g_dma_running |= HPM_DMA_CHANNEL_BIT(n);
  UP_DSB();
  putreg32(ch->transfer.ctrl | HPM_DMA_CH_CTRL_ENABLE, HPM_DMA_CH_CTRL(n));
}

/****************************************************************************
 * Name: hpm_dma_interrupt
 *
 * Description:
 *   Reserve observed events before unlocking, then defer cache and
 *   callbacks.
 *   Completing channels remain watched until their last bus access drains.
 *
 ****************************************************************************/

static int hpm_dma_interrupt(int irq, void *context, void *arg)
{
  struct hpm_dmach_s *ch;
  irqstate_t flags;
  uint32_t half;
  uint32_t complete;
  uint32_t abort;
  uint32_t error;
  uint32_t pending;
  uint32_t enabled;
  uint32_t bit;
  uint32_t events;
  unsigned int i;

  flags = spin_lock_irqsave(&g_dma_lock);
  half = getreg32(HPM_DMA_INTHALFSTS);
  complete = getreg32(HPM_DMA_INTTCSTS);
  abort = getreg32(HPM_DMA_INTABORTSTS);
  error = getreg32(HPM_DMA_INTERRSTS);
  if (half != 0)
    {
      putreg32(half, HPM_DMA_INTHALFSTS);
    }

  if (complete != 0)
    {
      putreg32(complete, HPM_DMA_INTTCSTS);
    }

  if (abort != 0)
    {
      putreg32(abort, HPM_DMA_INTABORTSTS);
    }

  if (error != 0)
    {
      putreg32(error, HPM_DMA_INTERRSTS);
    }

  pending = (half | complete | abort | error) & g_dma_running;
  enabled = pending != 0 ? getreg32(HPM_DMA_CHEN) : 0;

  /* Reserve every event before unlocking. Deferred callbacks cannot cause
   * an old global snapshot to be applied to a newly configured transfer.
   */

  while (pending != 0)
    {
      i = __builtin_ctz(pending);
      bit = HPM_DMA_CHANNEL_BIT(i);
      pending &= ~bit;
      ch = &g_dma_channels[i];
      if (ch->state == HPM_DMA_STATE_COMPLETING)
        {
          if ((error & bit) != 0)
            {
              ch->pending_events |= HPM_DMA_EVENT_ERROR;
            }

          if ((abort & bit) != 0)
            {
              ch->pending_events |= HPM_DMA_EVENT_ABORT;
            }

          if ((ch->pending_events & (HPM_DMA_EVENT_ERROR |
                                     HPM_DMA_EVENT_ABORT)) != 0)
            {
              hpm_dma_capture_fault(ch, (ch->pending_events &
                                    HPM_DMA_EVENT_ERROR) != 0 ? -EIO :
                                    -ECANCELED);
              if ((enabled & bit) != 0)
                {
                  putreg32(bit, HPM_DMA_CHABORT);
                }
            }

          continue;
        }

      events = 0;
      if ((half & bit) != 0 && ch->transfer.config.half_interrupt)
        {
          events |= HPM_DMA_EVENT_HALF;
          ch->observed_half++;
        }

      if ((complete & bit) != 0)
        {
          if (ch->mode == HPM_DMA_MODE_CYCLIC ||
              ch->mode == HPM_DMA_MODE_REPEAT)
            {
              events |= (enabled & bit) != 0 ? HPM_DMA_EVENT_PERIOD :
                                              HPM_DMA_EVENT_ERROR;
              ch->observed_periods++;
            }
          else if (ch->mode != HPM_DMA_MODE_CHAIN || (enabled & bit) == 0 ||
                   (getreg32(HPM_DMA_CH_LLPOINTER(i)) == 0 &&
                    getreg32(HPM_DMA_CH_TRANSIZE(i)) == 0))
            {
              events |= HPM_DMA_EVENT_COMPLETE;
              if (ch->mode == HPM_DMA_MODE_CHAIN &&
                  getreg32(HPM_DMA_CH_LLPOINTER(i)) != 0)
                {
                  events |= HPM_DMA_EVENT_ERROR;
                }
            }
        }

      if ((abort & bit) != 0)
        {
          events |= HPM_DMA_EVENT_ABORT;
        }

      if ((error & bit) != 0)
        {
          events |= HPM_DMA_EVENT_ERROR;
        }

      if (events == 0)
        {
          continue;
        }

      if ((events & HPM_DMA_EVENT_TERMINAL) != 0)
        {
          ch->state = HPM_DMA_STATE_COMPLETING;
          ch->terminal_start = clock_systime_ticks();
          if ((events & (HPM_DMA_EVENT_ERROR | HPM_DMA_EVENT_ABORT)) != 0)
            {
              ch->pending_events |= events;
              hpm_dma_capture_fault(ch, (events & HPM_DMA_EVENT_ERROR) != 0 ?
                                    -EIO : -ECANCELED);
              if ((enabled & bit) != 0)
                {
                  putreg32(bit, HPM_DMA_CHABORT);
                }
            }
        }

      ch->pending_events |= events;
      if (!ch->work_active)
        {
          hpm_dma_schedule(ch, 0);
        }
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: hpm_dma_compile
 *
 * Description:
 *   Validate a transfer and construct its disabled CTRL shadow.
 *
 ****************************************************************************/

static int hpm_dma_compile(const struct hpm_dma_config_s *config,
                           struct hpm_dma_transfer_s *prepared)
{
  unsigned int burst;
  unsigned int encoded = 0;
  int ret;
  uintptr_t ignored;

  memset(prepared, 0, sizeof(*prepared));
  ret = hpm_dma_transfer_geometry(config, &prepared->ntransfers);
  if (ret < 0)
    {
      return ret;
    }

  prepared->config = *config;
  ret = hpm_dma_address_span(config->src_addr, config->nbytes,
                             config->src_width, config->src_addrmode,
                             &prepared->src_span);
  if (ret < 0)
    {
      return ret;
    }

  ret = hpm_dma_address_span(config->dst_addr, config->nbytes,
                             config->dst_width, config->dst_addrmode,
                             &prepared->dst_span);
  if (ret < 0)
    {
      return ret;
    }

  if (!HPM_DMA_MEMORY_SOURCE(config->direction))
    {
      if (!hpm_dma_peripheral(&prepared->src_span))
        {
          ret = -EFAULT;
          return ret;
        }

      prepared->src_busaddr = config->src_addr;
    }
  else
    {
      ret = hpm_dma_map_source(config->src_addr, &prepared->src_span,
                            &prepared->src_busaddr);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (!HPM_DMA_MEMORY_DESTINATION(config->direction))
    {
      if (!hpm_dma_peripheral(&prepared->dst_span))
        {
          ret = -EFAULT;
          return ret;
        }

      prepared->dst_busaddr = config->dst_addr;
    }
  else
    {
      ret = hpm_dma_map_ram(config->dst_addr, &prepared->dst_span,
                            &prepared->dst_busaddr);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (config->direction == HPM_DMA_MEM_TO_MEM &&
      hpm_dma_physical(prepared->src_span.first) <=
      hpm_dma_physical(prepared->dst_span.last) &&
      hpm_dma_physical(prepared->dst_span.first) <=
      hpm_dma_physical(prepared->src_span.last))
    {
      ret = -EINVAL;
      return ret;
    }

  prepared->source_ram = HPM_DMA_MEMORY_SOURCE(config->direction) &&
                         hpm_dma_map_ram(config->src_addr,
                                         &prepared->src_span,
                                         &ignored) == OK;
  ret = hpm_dma_cache_validate(prepared);
  if (ret < 0)
    {
      return ret;
    }

  if ((config->burst & (config->burst - 1U)) != 0)
    {
      encoded = config->burst - 1;
      prepared->ctrl = HPM_DMA_CH_CTRL_BURSTOPT;
    }
  else
    {
      for (burst = config->burst; burst > 1; burst >>= 1)
        {
          encoded++;
        }
    }

  prepared->ctrl |= HPM_DMA_CH_CTRL_SRCBURSTSIZE_SET(encoded) |
                  HPM_DMA_CH_CTRL_SRCWIDTH_SET(config->src_width) |
                  HPM_DMA_CH_CTRL_DSTWIDTH_SET(config->dst_width) |
                  HPM_DMA_CH_CTRL_SRCADDRCTRL_SET(config->src_addrmode) |
                  HPM_DMA_CH_CTRL_DSTADDRCTRL_SET(config->dst_addrmode);
  if (config->direction != HPM_DMA_MEM_TO_MEM && !config->software_trigger)
    {
      prepared->ctrl |= (config->direction == HPM_DMA_PERIPH_TO_MEM ||
                        (config->direction == HPM_DMA_PERIPH_TO_PERIPH &&
                         !config->destination_request)) ?
                       HPM_DMA_CH_CTRL_SRCMODE : HPM_DMA_CH_CTRL_DSTMODE;
      if (config->handshake_block)
        {
          prepared->ctrl |= HPM_DMA_CH_CTRL_HANDSHAKEOPT;
        }
    }

  if (config->high_priority)
    {
      prepared->ctrl |= HPM_DMA_CH_CTRL_PRIORITY;
    }

  if (!config->half_interrupt)
    {
      prepared->ctrl |= HPM_DMA_CH_CTRL_INTHALFCNTMASK;
    }

  return OK;
}

/****************************************************************************
 * Name: hpm_dma_list_free
 *
 * Description:
 *   Release a list only from task context after it is no longer referenced.
 *
 ****************************************************************************/

static void hpm_dma_list_free(struct hpm_dma_list_s *list)
{
  if (list != NULL)
    {
      kmm_free(list->desc);
      kmm_free(list);
    }
}

/****************************************************************************
 * Name: hpm_dma_overlap
 *
 * Description:
 *   Compare physical RAM coverage, including different CPU aliases.
 *
 ****************************************************************************/

static bool hpm_dma_overlap(const struct hpm_dma_span_s *first,
                            const struct hpm_dma_span_s *second)
{
  return hpm_dma_physical(first->first) <= hpm_dma_physical(second->last) &&
         hpm_dma_physical(second->first) <= hpm_dma_physical(first->last);
}

struct hpm_dma_range_s
{
  struct hpm_dma_span_s span;
  size_t owner;
};

/****************************************************************************
 * Name: hpm_dma_range_compare
 *
 * Description:
 *   Sort physical cache coverage for disjoint streaming buffer ownership.
 *
 ****************************************************************************/

static int hpm_dma_range_compare(const void *first, const void *second)
{
  const struct hpm_dma_range_s *a = first;
  const struct hpm_dma_range_s *b = second;
  return a->span.first < b->span.first ? -1 :
         a->span.first > b->span.first ? 1 : 0;
}

/****************************************************************************
 * Name: hpm_dma_stream_validate
 *
 * Description:
 *   Reject physical/cache aliases between periods and driver metadata.
 *
 ****************************************************************************/

static int hpm_dma_stream_validate(struct hpm_dma_list_s *list,
                                    size_t metadata_bytes)
{
  struct hpm_dma_range_s *ranges;
  struct hpm_dma_transfer_s *item;
  struct hpm_dma_span_s span;
  uintptr_t ignored;
  uintptr_t last;
  size_t owner;
  size_t count = 1;
  size_t i;
  size_t j;
  size_t line = 1;
  int ret = OK;

#ifdef CONFIG_ARCH_DCACHE
  line = up_get_dcache_linesize();
#endif

  ranges = kmm_zalloc((2 * list->count + 1) * sizeof(*ranges));
  if (ranges == NULL)
    {
      return -ENOMEM;
    }

  ranges[0].span.first = hpm_dma_physical((uintptr_t)list);
  ranges[0].span.last = hpm_dma_physical((uintptr_t)list +
                                         metadata_bytes - 1);
  ranges[0].owner = SIZE_MAX;
  for (i = 0; i < list->count; i++)
    {
      item = &list->items[i];
      for (j = 0; j < 2; j++)
        {
          if (j == 0)
            {
              if (!HPM_DMA_MEMORY_SOURCE(item->config.direction))
                {
                  continue;
                }

              span = item->src_span;
              if (hpm_dma_map_ram(span.first, &span, &ignored) < 0)
                {
                  continue;
                }
            }
          else
            {
              if (!HPM_DMA_MEMORY_DESTINATION(item->config.direction))
                {
                  continue;
                }

              span = item->dst_cache_span;
            }

          span.first &= ~(uintptr_t)(line - 1);
          span.last |= line - 1;
          ranges[count].span.first = hpm_dma_physical(span.first);
          ranges[count].span.last = hpm_dma_physical(span.last);
          ranges[count++].owner = i;
        }
    }

  qsort(ranges, count, sizeof(*ranges), hpm_dma_range_compare);
  last = ranges[0].span.last;
  owner = ranges[0].owner;
  for (i = 1; i < count; i++)
    {
      if (ranges[i].span.first <= last && ranges[i].owner != owner)
        {
          ret = -EINVAL;
          break;
        }

      if (ranges[i].span.first > last || ranges[i].span.last > last)
        {
          last = ranges[i].span.last;
          owner = ranges[i].owner;
        }
    }

  kmm_free(ranges);
  return ret;
}

#ifdef CONFIG_ARCH_DCACHE
/* Cache plans are constructed once. Merge repeated/adjacent ranges while
 * preserving CPU-window boundaries; a DLM tail must not merge with an alias
 * head merely because their virtual addresses happen to be consecutive.
 */

/****************************************************************************
 * Name: hpm_dma_span_compare
 *
 * Description:
 *   Order cache ranges by their first CPU address.
 *
 ****************************************************************************/

static int hpm_dma_span_compare(const void *a, const void *b)
{
  const struct hpm_dma_span_s *first = a;
  const struct hpm_dma_span_s *second = b;
  return first->first < second->first ? -1 :
         first->first > second->first ? 1 : 0;
}

/****************************************************************************
 * Name: hpm_dma_merge_spans
 *
 * Description:
 *   Coalesce cache work without merging discontinuous alias windows.
 *
 ****************************************************************************/

static size_t hpm_dma_merge_spans(struct hpm_dma_span_s *spans, size_t count)
{
  size_t i;
  size_t n = 0;
  uintptr_t first;
  uintptr_t last;

  qsort(spans, count, sizeof(*spans), hpm_dma_span_compare);
  for (i = 0; i < count; i++)
    {
      if (n != 0)
        {
          first = hpm_dma_physical(spans[n - 1].first);
          last = hpm_dma_physical(spans[i].last);
          if (spans[i].first <= spans[n - 1].last + 1 &&
              last >= first && last - first ==
              spans[i].last - spans[n - 1].first)
            {
              if (spans[i].last > spans[n - 1].last)
                {
                  spans[n - 1].last = spans[i].last;
                }

              continue;
            }
        }

      spans[n++] = spans[i];
    }

  return n;
}

/****************************************************************************
 * Name: hpm_dma_cache_plan
 *
 * Description:
 *   Precompute reusable source, destination and physical cache ranges.
 *
 ****************************************************************************/

static void hpm_dma_cache_plan(struct hpm_dma_list_s *list)
{
  struct hpm_dma_transfer_s *item;
  struct hpm_dma_span_s span;
  size_t line = up_get_dcache_linesize();
  size_t i;

  list->sources = (struct hpm_dma_span_s *)(list->items + list->count);
  list->destinations = list->sources + list->count;
  list->invalidations = list->destinations + list->count;
  for (i = 0; i < list->count; i++)
    {
      item = &list->items[i];
      if (item->source_ram)
        {
          span.first = item->src_span.first & ~(uintptr_t)(line - 1);
          span.last = item->src_span.last | (line - 1);
          list->sources[list->nsources++] = span;
        }

      if (HPM_DMA_MEMORY_DESTINATION(item->config.direction))
        {
          span = item->dst_cache_span;
          list->destinations[list->ndestinations++] = span;
          span.first = hpm_dma_physical(span.first);
          span.last = hpm_dma_physical(span.last);
          list->invalidations[list->ninvalidations++] = span;
        }
    }

  list->nsources = hpm_dma_merge_spans(list->sources, list->nsources);
  list->ndestinations = hpm_dma_merge_spans(list->destinations,
                                          list->ndestinations);
  list->ninvalidations = hpm_dma_merge_spans(list->invalidations,
                                           list->ninvalidations);
}

#endif

/****************************************************************************
 * Name: hpm_dma_list_build
 *
 * Description:
 *   Copy and validate a hardware chain before replacing any active setup.
 *
 ****************************************************************************/

static int hpm_dma_list_build(const struct hpm_dma_config_s *configs,
                              size_t count,
                              enum hpm_dma_mode_e mode,
                              struct hpm_dma_list_s **result)
{
  struct hpm_dma_list_s *list;
  struct hpm_dma_transfer_s *item;
  struct hpm_dma_desc_s *desc;
  struct hpm_dma_span_s desc_span;
  struct hpm_dma_span_s metadata_span;
  size_t alignment = HPM_DMA_DESCRIPTOR_ALIGN;
  size_t bytes;
  size_t metadata_bytes;
  size_t i;
  int ret;

  *result = NULL;
  if (configs == NULL || count == 0)
    {
      return -EINVAL;
    }

  if (count > CONFIG_HPM_HDMA_MAX_DESCRIPTORS)
    {
      return -E2BIG;
    }

  if (mode != HPM_DMA_MODE_STREAM &&
      (getreg32(HPM_DMA_DMACFG) & HPM_DMA_DMACFG_CHAINXFR) == 0)
    {
      return -ENOTSUP;
    }

#ifdef CONFIG_ARCH_DCACHE
  alignment = up_get_dcache_linesize();
  if (alignment == 0 || (alignment & (alignment - 1)) != 0)
    {
      return -ENOTSUP;
    }

  if (alignment < HPM_DMA_DESCRIPTOR_ALIGN)
    {
      alignment = HPM_DMA_DESCRIPTOR_ALIGN;
    }

#endif

  bytes = count * sizeof(struct hpm_dma_desc_s);
  if (bytes > SIZE_MAX - (alignment - 1))
    {
      return -EOVERFLOW;
    }

  bytes = (bytes + alignment - 1) & ~(alignment - 1);
  metadata_bytes = sizeof(*list) + count * sizeof(list->items[0]);
#ifdef CONFIG_ARCH_DCACHE
  if (mode != HPM_DMA_MODE_STREAM)
    {
      metadata_bytes += 3 * count * sizeof(struct hpm_dma_span_s);
    }

#endif

  list = kmm_zalloc(metadata_bytes);
  if (list == NULL)
    {
      return -ENOMEM;
    }

  list->count = count;
  list->desc_bytes = bytes;
  for (i = 0; i < count; i++)
    {
      item = &list->items[i];
      ret = hpm_dma_compile(&configs[i], item);
      if (ret < 0)
        {
          goto fail;
        }

      if (mode != HPM_DMA_MODE_STREAM &&
          item->config.half_interrupt != configs[0].half_interrupt)
        {
          ret = -EINVAL;
          goto fail;
        }
    }

  if (mode == HPM_DMA_MODE_STREAM)
    {
      ret = hpm_dma_stream_validate(list, metadata_bytes);
      if (ret < 0)
        {
          goto fail;
        }

      *result = list;
      return OK;
    }

#ifdef CONFIG_ARCH_DCACHE
  hpm_dma_cache_plan(list);
#endif

  list->desc = kmm_memalign(alignment, bytes);
  if (list->desc == NULL)
    {
      ret = -ENOMEM;
      goto fail;
    }

  memset(list->desc, 0, bytes);
  ret = hpm_dma_address_span((uintptr_t)list->desc, bytes,
                             HPM_DMA_WIDTH_8BIT, HPM_DMA_ADDR_INCREMENT,
                             &desc_span);
  if (ret < 0)
    {
      goto fail;
    }

  ret = hpm_dma_map_ram((uintptr_t)list->desc, &desc_span,
                        &list->desc_busaddr);
  if (ret < 0)
    {
      goto fail;
    }

  if ((list->desc_busaddr & (HPM_DMA_DESCRIPTOR_ALIGN - 1)) != 0)
    {
      ret = -EINVAL;
      goto fail;
    }

  ret = hpm_dma_address_span((uintptr_t)list, metadata_bytes,
                             HPM_DMA_WIDTH_8BIT, HPM_DMA_ADDR_INCREMENT,
                             &metadata_span);
  if (ret < 0)
    {
      goto fail;
    }

  for (i = 0; i < count; i++)
    {
      item = &list->items[i];
      if (HPM_DMA_MEMORY_DESTINATION(item->config.direction) &&
          (hpm_dma_overlap(&item->dst_cache_span, &desc_span) ||
           hpm_dma_overlap(&item->dst_cache_span, &metadata_span)))
        {
          ret = -EINVAL;
          goto fail;
        }

      /* Finite chains interrupt only for final completion. Masked TC
       * status can still be observed during HALF; also check CHEN before
       * declaring the chain complete.
       */

      if (mode == HPM_DMA_MODE_CHAIN && i + 1 < count)
        {
          item->ctrl |= HPM_DMA_CH_CTRL_INTTCMASK;
        }

      desc = &list->desc[i];
      desc->ctrl = item->ctrl | HPM_DMA_CH_CTRL_ENABLE;
      desc->ntransfers = item->ntransfers;
      desc->src_addr = item->src_busaddr;
      desc->dst_addr = item->dst_busaddr;
      desc->request = 0;
      if (i + 1 < count)
        {
          desc->next = list->desc_busaddr +
                       (i + 1) * sizeof(struct hpm_dma_desc_s);
        }
      else if (mode == HPM_DMA_MODE_CYCLIC)
        {
          desc->next = list->desc_busaddr;
        }
    }

  *result = list;
  return OK;

fail:
  hpm_dma_list_free(list);
  return ret;
}

/****************************************************************************
 * Name: hpm_dma_prepare_all
 *
 * Description:
 *   Prepare every buffer and clean immutable hardware descriptors.
 *
 ****************************************************************************/

static void hpm_dma_prepare_all(struct hpm_dmach_s *ch)
{
  size_t i;
  if (ch->list == NULL)
    {
      hpm_dma_cache_prepare(&ch->transfer);
    }
  else if (ch->mode == HPM_DMA_MODE_STREAM)
    {
      for (i = 0; i < ch->list->count; i++)
        {
          hpm_dma_cache_prepare(&ch->list->items[i]);
        }
    }
  else
    {
#ifdef CONFIG_ARCH_DCACHE
      struct hpm_dma_span_s span;
      for (i = 0; i < ch->list->nsources; i++)
        {
          span = ch->list->sources[i];
          up_clean_dcache(span.first, span.last + 1);
        }

      for (i = 0; i < ch->list->ndestinations; i++)
        {
          span = ch->list->destinations[i];
          up_flush_dcache(span.first, span.last + 1);
          hpm_dma_invalidate_views(&span, false);
        }

      span.first = (uintptr_t)ch->list->desc;
      span.last = span.first + ch->list->desc_bytes - 1;
      up_clean_dcache(span.first, span.last + 1);
      hpm_dma_invalidate_views(&span, false);
#endif

      ch->list->cache_pending = true;
    }

  UP_DSB();
}

/****************************************************************************
 * Name: hpm_dma_finish_all
 *
 * Description:
 *   Return buffer cache ownership only after a terminal event or stop.
 *
 ****************************************************************************/

static void hpm_dma_finish_all(struct hpm_dmach_s *ch)
{
  size_t i;

  if (ch->list == NULL)
    {
      hpm_dma_cache_finish(&ch->transfer);
    }
  else if (ch->mode != HPM_DMA_MODE_STREAM)
    {
      if (!ch->list->cache_pending)
        {
          return;
        }

      UP_DSB();
#ifdef CONFIG_ARCH_DCACHE
      for (i = 0; i < ch->list->ninvalidations; i++)
        {
          hpm_dma_invalidate_views(&ch->list->invalidations[i], true);
        }

#endif

      ch->list->cache_pending = false;
    }
  else
    {
      for (i = 0; i < ch->list->count; i++)
        {
          bool finish = true;
          if (ch->mode == HPM_DMA_MODE_STREAM)
            {
              irqstate_t flags = spin_lock_irqsave(&g_dma_lock);
              uint8_t owner = ch->list->items[i].ownership;
              finish = owner == HPM_DMA_BUFFER_ACTIVE ||
                       owner == HPM_DMA_BUFFER_QUEUED;
              spin_unlock_irqrestore(&g_dma_lock, flags);
            }

          if (finish)
            {
              hpm_dma_cache_finish(&ch->list->items[i]);
            }
        }
    }
}

/****************************************************************************
 * Name: hpm_dma_worker
 *
 * Description:
 *   Drain final bus accesses, return caches and notify outside IRQ context.
 *
 ****************************************************************************/

static void hpm_dma_worker(void *arg)
{
  struct hpm_dmach_s *ch = arg;
  hpm_dma_callback_t callback = NULL;
  struct hpm_dma_transfer_s *finished = NULL;
  irqstate_t flags;
  uint32_t events;
  void *cbarg = NULL;
  size_t index = 0;
  size_t next;
  bool terminal;
  bool quiet = false;
  bool fault;

  flags = spin_lock_irqsave(&g_dma_lock);
  g_dma_worker_pid = nxsched_gettid();
  if (ch->state == HPM_DMA_STATE_WAITING && ch->list != NULL &&
      ch->list->items[ch->stream_current].ownership == HPM_DMA_BUFFER_QUEUED)
    {
      ch->list->items[ch->stream_current].ownership = HPM_DMA_BUFFER_ACTIVE;
      hpm_dma_launch(ch, ch->stream_current);
    }

  if (ch->state != HPM_DMA_STATE_RUNNING &&
      ch->state != HPM_DMA_STATE_COMPLETING)
    {
      ch->pending_events = 0;
      hpm_dma_work_idle(ch);
      spin_unlock_irqrestore(&g_dma_lock, flags);
      return;
    }

  if (ch->pending_events == 0)
    {
      hpm_dma_work_idle(ch);
      spin_unlock_irqrestore(&g_dma_lock, flags);
      return;
    }

  terminal = ch->state == HPM_DMA_STATE_COMPLETING;
  if (terminal)
    {
      quiet = hpm_dma_quiescent(ch);
      if (!quiet && clock_systime_ticks() - ch->terminal_start <
                    MSEC2TICK(CONFIG_HPM_HDMA_STOP_TIMEOUT))
        {
          hpm_dma_schedule(ch, 1);
          spin_unlock_irqrestore(&g_dma_lock, flags);
          return;
        }

      if (!quiet)
        {
          ch->pending_events |= HPM_DMA_EVENT_ERROR;
          hpm_dma_capture_fault(ch, -ETIMEDOUT);
          if (hpm_dma_enabled(ch))
            {
              putreg32(HPM_DMA_CHANNEL_BIT(ch->channel), HPM_DMA_CHABORT);
            }
        }
      else
        {
          uint32_t bit = HPM_DMA_CHANNEL_BIT(ch->channel);
          if ((getreg32(HPM_DMA_INTERRSTS) & bit) != 0)
            {
              ch->pending_events |= HPM_DMA_EVENT_ERROR;
              hpm_dma_capture_fault(ch, -EIO);
            }
          else if ((getreg32(HPM_DMA_INTABORTSTS) & bit) != 0)
            {
              ch->pending_events |= HPM_DMA_EVENT_ABORT;
              hpm_dma_capture_fault(ch, -ECANCELED);
            }

          hpm_dma_idle(ch);
        }

      g_dma_running &= ~HPM_DMA_CHANNEL_BIT(ch->channel);
    }

  events = ch->pending_events;
  ch->pending_events = 0;
  fault = (events & (HPM_DMA_EVENT_ERROR | HPM_DMA_EVENT_ABORT)) != 0;
  if (fault)
    {
      events &= ~(HPM_DMA_EVENT_COMPLETE | HPM_DMA_EVENT_HALF |
                  HPM_DMA_EVENT_PERIOD);
      if ((events & HPM_DMA_EVENT_ERROR) != 0)
        {
          events &= ~HPM_DMA_EVENT_ABORT;
        }

      ch->state = HPM_DMA_STATE_FAULT;
    }

  if (terminal && quiet && !fault && ch->mode == HPM_DMA_MODE_STREAM)
    {
      index = ch->stream_current;
      finished = &ch->list->items[index];
      next = (index + 1) % ch->list->count;
      ch->stream_current = next;
      events &= ~HPM_DMA_EVENT_COMPLETE;
      events |= HPM_DMA_EVENT_PERIOD;
      if (ch->list->items[next].ownership == HPM_DMA_BUFFER_QUEUED)
        {
          ch->list->items[next].ownership = HPM_DMA_BUFFER_ACTIVE;
          hpm_dma_launch(ch, next);
        }
      else
        {
          ch->state = HPM_DMA_STATE_WAITING;
          events |= HPM_DMA_EVENT_STARVED;
        }
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
  if (terminal && quiet)
    {
      if (finished != NULL)
        {
          hpm_dma_cache_finish(finished);
        }
      else
        {
          hpm_dma_finish_all(ch);
        }
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  if (finished != NULL)
    {
      if (ch->callback != NULL)
        {
          finished->sequence = ++ch->completed_periods;
          finished->ownership = HPM_DMA_BUFFER_READY;
          finished->next_ready = SIZE_MAX;
          if (ch->ready_tail == SIZE_MAX)
            {
              ch->ready_head = index;
            }
          else
            {
              ch->list->items[ch->ready_tail].next_ready = index;
            }

          ch->ready_tail = index;
          ch->ready_count++;
        }
      else
        {
          finished->ownership = HPM_DMA_BUFFER_QUEUED;
        }
    }

  /* Stop may have suppressed delivery while cache work ran. Do not overwrite
   * its STOPPING state or a later stream block's COMPLETING state.
   */

  if (ch->state != HPM_DMA_STATE_STOPPING && ch->callback != NULL)
    {
      if (terminal && quiet && !fault && finished == NULL)
        {
          ch->state = HPM_DMA_STATE_ALLOCATED;
        }

      ch->last_events = events;
      callback = ch->callback;
      cbarg = ch->arg;
      ch->callback_active = true;
      if ((events & HPM_DMA_EVENT_TERMINAL) != 0)
        {
          ch->callback = NULL;
          ch->arg = NULL;
        }
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
  if (callback != NULL)
    {
      callback(ch, events, cbarg);
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  ch->callback_active = false;
  if (ch->pending_events != 0 ||
      (ch->state == HPM_DMA_STATE_WAITING && ch->list != NULL &&
       ch->list->items[ch->stream_current].ownership ==
       HPM_DMA_BUFFER_QUEUED))
    {
      hpm_dma_schedule(ch, 0);
    }
  else
    {
      hpm_dma_work_idle(ch);
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
}

/* Route reservation is transactional. Old ports stay reserved until commit;
 * failures release only newly acquired ports. Stream mode can reuse one port
 * at its explicit quiescent boundaries; raw chains reserve distinct sources.
 */

/****************************************************************************
 * Name: hpm_dma_routes_build
 *
 * Description:
 *   Reserve request ports transactionally while preserving old ownership.
 *
 ****************************************************************************/

static int hpm_dma_routes_build(struct hpm_dmach_s *ch,
                                struct hpm_dma_transfer_s *single,
                                struct hpm_dma_list_s *list,
                                enum hpm_dma_mode_e mode,
                                struct hpm_dma_routes_s *routes)
{
  struct hpm_dma_transfer_s *item;
  uint32_t seen[4] =
  {
    0, 0, 0, 0
  };

  uint8_t sources[HPM_DMA_NCHANNELS];
  irqstate_t flags;
  uint32_t ports;
  uint32_t available;
  unsigned int port;
  unsigned int source;
  unsigned int count_sources = 0;
  size_t i;
  size_t count = list == NULL ? 1 : list->count;

  memset(routes, 0, sizeof(*routes));
  for (i = 0; i < count; i++)
    {
      item = list == NULL ? single : &list->items[i];
      if (item->config.direction == HPM_DMA_MEM_TO_MEM ||
          item->config.software_trigger)
        {
          continue;
        }

      source = item->config.request;
      if ((seen[source / 32] & (1U << (source % 32))) == 0)
        {
          if (count_sources == HPM_DMA_NCHANNELS)
            {
              return -ENOSPC;
            }

          sources[count_sources++] = source;
          seen[source / 32] |= 1U << (source % 32);
        }

      if (mode == HPM_DMA_MODE_STREAM)
        {
          break;
        }
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  for (i = 0; i < count_sources; i++)
    {
      available = (~g_dma_ports | ch->routes.mask) & ~routes->mask;
      if (available == 0)
        {
          g_dma_ports &= ~(routes->mask & ~ch->routes.mask);
          spin_unlock_irqrestore(&g_dma_lock, flags);
          return -EBUSY;
        }

      port = __builtin_ctz(available);
      routes->mask |= HPM_DMA_CHANNEL_BIT(port);
      routes->source[port] = sources[i];
      g_dma_ports |= HPM_DMA_CHANNEL_BIT(port);
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);

  for (i = 0; i < count; i++)
    {
      item = list == NULL ? single : &list->items[i];
      if (item->config.direction != HPM_DMA_MEM_TO_MEM &&
          !item->config.software_trigger)
        {
          ports = routes->mask;
          while (ports != 0)
            {
              port = __builtin_ctz(ports);
              ports &= ~HPM_DMA_CHANNEL_BIT(port);
              if (mode == HPM_DMA_MODE_STREAM ||
                  routes->source[port] == item->config.request)
                {
                  item->request_port = port;
                  break;
                }
            }
        }

      if (list != NULL && list->desc != NULL)
        {
          port = item->request_port;
          list->desc[i].request =
            HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_SET(port) |
            HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_SET(port);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: hpm_dma_configure
 *
 * Description:
 *   Atomically replace an idle setup after all validation succeeds.
 *
 ****************************************************************************/

static int hpm_dma_configure(hpm_dma_handle_t handle,
                             const struct hpm_dma_config_s *configs,
                             size_t count, enum hpm_dma_mode_e mode)
{
  struct hpm_dmach_s *ch = hpm_dma_from_handle(handle);
  struct hpm_dma_transfer_s prepared;
  struct hpm_dma_routes_s routes;
  struct hpm_dma_list_s *list = NULL;
  struct hpm_dma_list_s *old;
  irqstate_t flags;
  int ret;

  ret = hpm_dma_task_context();
  if (ret < 0)
    {
      return ret;
    }

  if (ch == NULL || !g_dma_initialized)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&ch->lock);
  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  if ((ch->state != HPM_DMA_STATE_ALLOCATED &&
       ch->state != HPM_DMA_STATE_CONFIGURED) ||
      ch->work_active || ch->callback_active || ch->loans != 0 ||
      hpm_dma_enabled(ch))
    {
      spin_unlock_irqrestore(&g_dma_lock, flags);
      nxmutex_unlock(&ch->lock);
      return -EBUSY;
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
  if (mode == HPM_DMA_MODE_CHAIN || mode == HPM_DMA_MODE_CYCLIC ||
      mode == HPM_DMA_MODE_STREAM)
    {
      ret = hpm_dma_list_build(configs, count, mode, &list);
      if (ret < 0)
        {
          goto out;
        }

      prepared = list->items[0];
    }
  else
    {
      ret = hpm_dma_compile(configs, &prepared);
      if (ret < 0)
        {
          goto out;
        }

      if (mode == HPM_DMA_MODE_REPEAT)
        {
          prepared.ctrl |= HPM_DMA_CH_CTRL_INFINITELOOP;
        }
    }

  ret = hpm_dma_routes_build(ch, &prepared, list, mode, &routes);
  if (ret < 0)
    {
      hpm_dma_list_free(list);
      goto out;
    }

  if (list != NULL)
    {
      prepared = list->items[0];
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  hpm_dma_idle(ch);
  g_dma_ports &= ~(ch->routes.mask & ~routes.mask);
  ch->routes = routes;
  old = ch->list;
  ch->list = list;
  ch->transfer = prepared;
  ch->mode = mode;
  ch->state = HPM_DMA_STATE_CONFIGURED;
  ch->configuration_valid = true;
  ch->ready_head = SIZE_MAX;
  ch->ready_tail = SIZE_MAX;
  ch->ready_count = 0;
  ch->generation++;
  spin_unlock_irqrestore(&g_dma_lock, flags);
  hpm_dma_list_free(old);
  ret = OK;

out:
  nxmutex_unlock(&ch->lock);
  return ret;
}

/* Power transitions may occur only with no channel owners. prepare closes
 * the allocation gate atomically; notify only requests lazy restoration.
 * No mutex wait, cache operation or DMA polling occurs in PM callbacks.
 */

#ifdef CONFIG_PM

/****************************************************************************
 * Name: hpm_dma_pm_prepare
 *
 * Description:
 *   Close the allocation gate before deep sleep when no owners remain.
 *
 ****************************************************************************/

static int hpm_dma_pm_prepare(struct pm_callback_s *cb, int domain,
                              enum pm_state_e state)
{
  irqstate_t flags;
  unsigned int i;
  int ret = OK;

  if (state < PM_STANDBY)
    {
      return OK;
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  if (g_dma_allocating)
    {
      ret = -EBUSY;
    }

  for (i = 0; ret == OK && i < HPM_DMA_NCHANNELS; i++)
    {
      if (g_dma_channels[i].state != HPM_DMA_STATE_FREE ||
          g_dma_channels[i].work_active)
        {
          ret = -EBUSY;
        }
    }

  if (ret == OK)
    {
      g_dma_pm_blocked = true;
      up_disable_irq(HPM_IRQ_HDMA);
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: hpm_dma_pm_notify
 *
 * Description:
 *   Defer clock and controller restoration to the next channel allocation.
 *
 ****************************************************************************/

static void hpm_dma_pm_notify(struct pm_callback_s *cb, int domain,
                              enum pm_state_e state)
{
  irqstate_t flags;
  if (state >= PM_STANDBY)
    {
      return;
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  if (g_dma_pm_blocked)
    {
      g_dma_restore_needed = true;
    }

  g_dma_pm_blocked = false;
  spin_unlock_irqrestore(&g_dma_lock, flags);
}

#endif

/* Only bootstrap, or lazy PM restoration with every channel FREE, may reset
 * the controller. Existing channel mutexes and generation numbers survive.
 */

/****************************************************************************
 * Name: hpm_dma_hardware_initialize
 *
 * Description:
 *   Reset hardware only during bootstrap or owner-free PM restoration.
 *
 ****************************************************************************/

static int hpm_dma_hardware_initialize(void)
{
  unsigned int i;
  int ret = hpm_dma_clockconfig();

  if (ret < 0)
    {
      return ret;
    }

  up_disable_irq(HPM_IRQ_HDMA);
  putreg32(HPM_DMA_DMACTRL_RESET, HPM_DMA_DMACTRL);
  for (i = 0; i < 1000; i++)
    {
      if ((getreg32(HPM_DMA_DMACTRL) & HPM_DMA_DMACTRL_RESET) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (i == 1000)
    {
      return -ETIMEDOUT;
    }

  for (i = 0; i < HPM_DMA_NCHANNELS; i++)
    {
      putreg32(HPM_DMA_CTRL_IRQ_MASK, HPM_DMA_CH_CTRL(i));
      putreg32(0, HPM_DMA_CH_LLPOINTER(i));
      putreg32(0, HPM_DMAMUX_MUXCFG(i));
    }

  putreg32(HPM_DMA_ALL_CHANNELS, HPM_DMA_INTHALFSTS);
  putreg32(HPM_DMA_ALL_CHANNELS, HPM_DMA_INTTCSTS);
  putreg32(HPM_DMA_ALL_CHANNELS, HPM_DMA_INTABORTSTS);
  putreg32(HPM_DMA_ALL_CHANNELS, HPM_DMA_INTERRSTS);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Called once during NuttX architecture initialization, before any clients.
 * Reset is permitted only here, when the controller has no channel owners.
 * An initialization failure leaves allocation disabled.
 */

/****************************************************************************
 * Name: riscv_dma_initialize
 *
 * Description:
 *   Initialize the DMA controller through the architecture boot hook.
 *
 ****************************************************************************/

void riscv_dma_initialize(void)
{
  unsigned int i;
  int ret;

  if (g_dma_initialized)
    {
      return;
    }

  ret = hpm_dma_hardware_initialize();
  if (ret < 0)
    {
      _err("ERROR: HDMA hardware initialization failed: %d\n", ret);
      return;
    }

  for (i = 0; i < HPM_DMA_NCHANNELS; i++)
    {
      g_dma_channels[i].channel = i;
      ret = nxsem_init(&g_dma_channels[i].work_sem, 0, 0);
      if (ret == OK)
        {
          ret = nxsem_set_protocol(&g_dma_channels[i].work_sem,
                                    SEM_PRIO_NONE);
        }

      if (ret < 0)
        {
          _err("ERROR: HDMA work semaphore init failed: %d\n", ret);
          return;
        }

      ret = nxmutex_init(&g_dma_channels[i].lock);
      if (ret < 0)
        {
          _err("ERROR: HDMA channel mutex initialization failed: %d\n", ret);
          return;
        }
    }

  ret = irq_attach(HPM_IRQ_HDMA, hpm_dma_interrupt, NULL);
  if (ret < 0)
    {
      _err("ERROR: HDMA interrupt attachment failed: %d\n", ret);
      return;
    }

#ifdef CONFIG_PM
  ret = pm_register(&g_dma_pm);
  if (ret < 0)
    {
      _err("ERROR: HDMA PM registration failed: %d\n", ret);
      return;
    }

#endif

  g_dma_initialized = true;
  up_enable_irq(HPM_IRQ_HDMA);
}

/****************************************************************************
 * Name: hpm_dma_alloc
 *
 * Description:
 *   Allocate an unused channel without waiting for a channel owner.
 *
 ****************************************************************************/

int hpm_dma_alloc(hpm_dma_handle_t *handle)
{
  irqstate_t flags;
  unsigned int i;
  int ret;

  if (handle == NULL)
    {
      return -EINVAL;
    }

  *handle = NULL;
  ret = hpm_dma_task_context();
  if (ret < 0)
    {
      return ret;
    }

  if (!g_dma_initialized)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&g_dma_queue_lock);
  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&g_dma_lock);
#ifdef CONFIG_PM
  if (g_dma_pm_blocked)
    {
      spin_unlock_irqrestore(&g_dma_lock, flags);
      nxmutex_unlock(&g_dma_queue_lock);
      return -EAGAIN;
    }

#endif

  g_dma_allocating = true;
  spin_unlock_irqrestore(&g_dma_lock, flags);
#ifdef CONFIG_PM
  if (g_dma_restore_needed)
    {
      ret = hpm_dma_hardware_initialize();
      if (ret < 0)
        {
          goto done;
        }

      flags = spin_lock_irqsave(&g_dma_lock);
      g_dma_restore_needed = false;
      spin_unlock_irqrestore(&g_dma_lock, flags);
      up_enable_irq(HPM_IRQ_HDMA);
    }

#endif

  if (g_dma_wqueue == NULL)
    {
      g_dma_wqueue = work_queue_create("hpm_dma",
                      CONFIG_HPM_HDMA_WORKPRIORITY, NULL,
                      CONFIG_HPM_HDMA_WORKSTACKSIZE, 1);
    }

  ret = g_dma_wqueue == NULL ? -ENOMEM : -EBUSY;
  flags = spin_lock_irqsave(&g_dma_lock);
  if (g_dma_wqueue != NULL)
    {
      for (i = 0; i < HPM_DMA_NCHANNELS; i++)
        {
          if (g_dma_channels[i].state == HPM_DMA_STATE_FREE)
            {
              g_dma_channels[i].state = HPM_DMA_STATE_ALLOCATED;
              g_dma_channels[i].mode = HPM_DMA_MODE_SINGLE;
              g_dma_channels[i].result = OK;
              g_dma_channels[i].last_events = 0;
              g_dma_channels[i].pending_events = 0;
              memset(&g_dma_channels[i].fault, 0,
                     sizeof(g_dma_channels[i].fault));
              *handle = &g_dma_channels[i];
              ret = OK;
              break;
            }
        }
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
#ifdef CONFIG_PM
done:
#endif

  flags = spin_lock_irqsave(&g_dma_lock);
  g_dma_allocating = false;
  spin_unlock_irqrestore(&g_dma_lock, flags);
  nxmutex_unlock(&g_dma_queue_lock);
  return ret;
}

/* Validate into a temporary object before replacing an existing setup. No
 * registers are modified on validation failure. Per-channel mutex ownership
 * prevents another task from starting/freeing the channel during validation.
 */

/****************************************************************************
 * Name: hpm_dma_setup
 *
 * Description:
 *   Validate and retain one transfer configuration without starting DMA.
 *
 ****************************************************************************/

int hpm_dma_setup(hpm_dma_handle_t handle,
                  const struct hpm_dma_config_s *config)
{
  return hpm_dma_configure(handle, config, 1, HPM_DMA_MODE_SINGLE);
}

/****************************************************************************
 * Name: hpm_dma_setup_chain
 *
 * Description:
 *   Prepare a finite descriptor chain or a circular descriptor ring.
 *
 ****************************************************************************/

int hpm_dma_setup_chain(hpm_dma_handle_t handle,
                        const struct hpm_dma_config_s *configs,
                        size_t count, bool cyclic)
{
  enum hpm_dma_mode_e mode = cyclic ? HPM_DMA_MODE_CYCLIC :
                                     HPM_DMA_MODE_CHAIN;

  return hpm_dma_configure(handle, configs, count, mode);
}

/****************************************************************************
 * Name: hpm_dma_setup_repeat
 *
 * Description:
 *   Prepare native hardware infinite repetition without a descriptor chain.
 *
 ****************************************************************************/

int hpm_dma_setup_repeat(hpm_dma_handle_t handle,
                         const struct hpm_dma_config_s *config)
{
  return hpm_dma_configure(handle, config, 1, HPM_DMA_MODE_REPEAT);
}

/****************************************************************************
 * Name: hpm_dma_start
 *
 * Description:
 *   Prepare caches and registers, then enable one asynchronous transfer.
 *
 ****************************************************************************/

int hpm_dma_start(hpm_dma_handle_t handle,
                  hpm_dma_callback_t callback, void *arg)
{
  struct hpm_dmach_s *ch = hpm_dma_from_handle(handle);
  irqstate_t flags;
  int ret;

  ret = hpm_dma_task_context();
  if (ret < 0)
    {
      return ret;
    }

  if (ch == NULL || callback == NULL || !g_dma_initialized)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&ch->lock);
  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  if (ch->state != HPM_DMA_STATE_CONFIGURED || ch->work_active ||
      ch->callback_active ||
      hpm_dma_enabled(ch))
    {
      spin_unlock_irqrestore(&g_dma_lock, flags);
      nxmutex_unlock(&ch->lock);
      return -EBUSY;
    }

#ifndef CONFIG_ARCH_DCACHE
  if ((READ_CSR(HPM_CSR_MCACHE_CTL) & HPM_MCACHE_CTL_DC_EN) != 0)
    {
      ch->result = -ENOTSUP;
      spin_unlock_irqrestore(&g_dma_lock, flags);
      nxmutex_unlock(&ch->lock);
      return -ENOTSUP;
    }

#endif

  spin_unlock_irqrestore(&g_dma_lock, flags);
  hpm_dma_prepare_all(ch);
  flags = spin_lock_irqsave(&g_dma_lock);
  ch->generation++;
  ch->pending_events = 0;
  ch->last_events = 0;
  ch->result = OK;
  ch->observed_half = 0;
  ch->observed_periods = 0;
  ch->completed_periods = 0;
  ch->stream_current = 0;
  ch->ready_head = SIZE_MAX;
  ch->ready_tail = SIZE_MAX;
  ch->ready_count = 0;
  memset(&ch->fault, 0, sizeof(ch->fault));
  ch->callback = callback;
  ch->arg = arg;
  if (ch->mode == HPM_DMA_MODE_STREAM)
    {
      ch->list->items[0].ownership = HPM_DMA_BUFFER_ACTIVE;
    }

  hpm_dma_launch(ch, 0);
  spin_unlock_irqrestore(&g_dma_lock, flags);
  nxmutex_unlock(&ch->lock);
  return OK;
}

/* Suppress notifications first, then request abort without rewriting CTRL.
 * Keep the channel mutex, but release the spinlock while waiting. A timeout
 * quarantines the channel; never reset the controller underneath other
 * users.
 */

/****************************************************************************
 * Name: hpm_dma_stop
 *
 * Description:
 *   Suppress notifications and cancel one transfer with a bounded wait.
 *
 ****************************************************************************/

int hpm_dma_stop(hpm_dma_handle_t handle)
{
  struct hpm_dmach_s *ch = hpm_dma_from_handle(handle);
  irqstate_t flags;
  clock_t start;
  bool work_active;
  bool quiet;
  int ret;

  ret = hpm_dma_task_context();
  if (ret < 0)
    {
      return ret;
    }

  if (ch == NULL || !g_dma_initialized)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&ch->lock);
  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  if (ch->state == HPM_DMA_STATE_FREE)
    {
      spin_unlock_irqrestore(&g_dma_lock, flags);
      nxmutex_unlock(&ch->lock);
      return -EINVAL;
    }

  g_dma_running &= ~HPM_DMA_CHANNEL_BIT(ch->channel);
  if (ch->mode != HPM_DMA_MODE_STREAM)
    {
      ch->generation++;
    }

  ch->state = HPM_DMA_STATE_STOPPING;
  ch->callback = NULL;
  ch->arg = NULL;
  ch->pending_events = 0;
  if (hpm_dma_enabled(ch))
    {
      putreg32(HPM_DMA_CHANNEL_BIT(ch->channel), HPM_DMA_CHABORT);
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
  start = clock_systime_ticks();
  for (; ; )
    {
      flags = spin_lock_irqsave(&g_dma_lock);
      work_active = ch->work_active;
      quiet = hpm_dma_quiescent(ch);
      if (quiet && !work_active)
        {
          hpm_dma_idle(ch);
          spin_unlock_irqrestore(&g_dma_lock, flags);
          break;
        }

      if (clock_systime_ticks() - start >=
          MSEC2TICK(CONFIG_HPM_HDMA_STOP_TIMEOUT))
        {
          ch->state = HPM_DMA_STATE_FAULT;
          ch->stop_timeouts++;
          hpm_dma_capture_fault(ch, -ETIMEDOUT);
          spin_unlock_irqrestore(&g_dma_lock, flags);
          nxmutex_unlock(&ch->lock);
          return -ETIMEDOUT;
        }

      spin_unlock_irqrestore(&g_dma_lock, flags);
      if (quiet)
        {
          hpm_dma_wait_work(ch, start);
        }
      else
        {
          nxsig_usleep(1000);
        }
    }

  hpm_dma_finish_all(ch);

  flags = spin_lock_irqsave(&g_dma_lock);
  ch->state = HPM_DMA_STATE_ALLOCATED;
  spin_unlock_irqrestore(&g_dma_lock, flags);
  nxmutex_unlock(&ch->lock);
  return OK;
}

/****************************************************************************
 * Name: hpm_dma_free
 *
 * Description:
 *   Release a channel only after hardware and callbacks are quiescent.
 *
 ****************************************************************************/

int hpm_dma_free(hpm_dma_handle_t handle)
{
  struct hpm_dmach_s *ch = hpm_dma_from_handle(handle);
  struct hpm_dma_list_s *list = NULL;
  irqstate_t flags;
  int ret;

  ret = hpm_dma_task_context();
  if (ret < 0)
    {
      return ret;
    }

  if (ch == NULL || !g_dma_initialized)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&ch->lock);
  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  if ((ch->state != HPM_DMA_STATE_ALLOCATED &&
       ch->state != HPM_DMA_STATE_CONFIGURED) ||
      ch->work_active || ch->callback_active || ch->loans != 0 ||
      hpm_dma_enabled(ch))
    {
      ret = -EBUSY;
    }
  else
    {
      hpm_dma_idle(ch);
      ch->callback = NULL;
      ch->arg = NULL;
      list = ch->list;
      ch->list = NULL;
      g_dma_ports &= ~ch->routes.mask;
      memset(&ch->routes, 0, sizeof(ch->routes));
      ch->configuration_valid = false;
      ch->generation++;
      ch->state = HPM_DMA_STATE_FREE;
      ret = OK;
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
  hpm_dma_list_free(list);
  nxmutex_unlock(&ch->lock);
  return ret;
}

/****************************************************************************
 * Name: hpm_dma_setup_stream
 *
 * Description:
 *   Prepare disjoint owned periods with hardware gating against overwrite.
 *
 ****************************************************************************/

int hpm_dma_setup_stream(hpm_dma_handle_t handle,
                         const struct hpm_dma_config_s *configs,
                         size_t count)
{
  return hpm_dma_configure(handle, configs, count, HPM_DMA_MODE_STREAM);
}

/****************************************************************************
 * Name: hpm_dma_rearm
 *
 * Description:
 *   Reuse immutable validated configuration without allocating or
 *   rebuilding.
 *
 ****************************************************************************/

int hpm_dma_rearm(hpm_dma_handle_t handle)
{
  struct hpm_dmach_s *ch = hpm_dma_from_handle(handle);
  irqstate_t flags;
  size_t i;
  int ret = hpm_dma_task_context();

  if (ret < 0)
    {
      return ret;
    }

  if (ch == NULL || !g_dma_initialized)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&ch->lock);
  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  if (ch->state == HPM_DMA_STATE_ALLOCATED && ch->work_active)
    {
      spin_unlock_irqrestore(&g_dma_lock, flags);
      ret = hpm_dma_wait_work(ch, clock_systime_ticks());
      if (ret < 0)
        {
          nxmutex_unlock(&ch->lock);
          return ret;
        }

      flags = spin_lock_irqsave(&g_dma_lock);
    }

  if (!ch->configuration_valid)
    {
      ret = -EINVAL;
    }
  else if ((ch->state != HPM_DMA_STATE_ALLOCATED &&
            ch->state != HPM_DMA_STATE_CONFIGURED) ||
           ch->work_active || ch->loans != 0 || !hpm_dma_quiescent(ch))
    {
      ret = -EBUSY;
    }
  else
    {
      if (ch->list != NULL)
        {
          for (i = 0; ch->mode == HPM_DMA_MODE_STREAM &&
                      i < ch->list->count; i++)
            {
              ch->list->items[i].ownership = HPM_DMA_BUFFER_QUEUED;
              ch->list->items[i].sequence = 0;
            }

          ch->transfer = ch->list->items[0];
        }

      ch->ready_head = SIZE_MAX;
      ch->ready_tail = SIZE_MAX;
      ch->ready_count = 0;
      ch->generation++;
      ch->state = HPM_DMA_STATE_CONFIGURED;
      ret = OK;
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
  nxmutex_unlock(&ch->lock);
  return ret;
}

/****************************************************************************
 * Name: hpm_dma_getstatus
 *
 * Description:
 *   Copy software state, live advisory registers and preserved fault
 *   evidence.
 *
 ****************************************************************************/

int hpm_dma_getstatus(hpm_dma_handle_t handle,
                      struct hpm_dma_status_s *status)
{
  struct hpm_dmach_s *ch = hpm_dma_from_handle(handle);
  irqstate_t flags;
  size_t offset;
  unsigned int width;

  if (status == NULL)
    {
      return -EINVAL;
    }

  memset(status, 0, sizeof(*status));
  if (ch == NULL || !g_dma_initialized)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  if (ch->state == HPM_DMA_STATE_FREE)
    {
      spin_unlock_irqrestore(&g_dma_lock, flags);
      return -EINVAL;
    }

  status->state = ch->state;
  status->mode = ch->mode;
  status->generation = ch->generation;
  status->pending_events = ch->pending_events;
  status->last_events = ch->last_events;
  status->result = ch->result;
  status->observed_half = ch->observed_half;
  status->observed_periods = ch->observed_periods;
  status->completed_periods = ch->completed_periods;
  status->cpu_owned_periods = ch->loans;
  status->ready_periods = ch->mode == HPM_DMA_MODE_STREAM ?
                          ch->ready_count : 0;
  status->stop_timeouts = ch->stop_timeouts;
  status->queue_errors = ch->queue_errors;
  status->work_pending = ch->work_active;
  status->callback_active = ch->callback_active;
  status->configuration_valid = ch->configuration_valid;
  status->segment_count = ch->list == NULL ? 1 : ch->list->count;
  status->fault = ch->fault;
  status->hardware_enabled = hpm_dma_enabled(ch);
  status->request_ports = ch->routes.mask;
  status->stream_next = ch->mode == HPM_DMA_MODE_STREAM ?
                        ch->stream_current : SIZE_MAX;
  status->ctrl = getreg32(HPM_DMA_CH_CTRL(ch->channel));
  status->remaining_units = HPM_DMA_CH_TRANSIZE_GET(
                           getreg32(HPM_DMA_CH_TRANSIZE(ch->channel)));
  status->src = getreg32(HPM_DMA_CH_SRCADDR(ch->channel));
  status->dst = getreg32(HPM_DMA_CH_DSTADDR(ch->channel));
  status->next = getreg32(HPM_DMA_CH_LLPOINTER(ch->channel));
  width = HPM_DMA_CH_CTRL_SRCWIDTH_GET(status->ctrl);
  status->remaining_valid = status->hardware_enabled &&
                            width <= HPM_DMA_WIDTH_32BIT;
  if (status->remaining_valid)
    {
      status->remaining_bytes = (size_t)status->remaining_units << width;
    }

  status->current_segment = SIZE_MAX;
  if (status->hardware_enabled)
    {
      if (ch->mode == HPM_DMA_MODE_STREAM)
        {
          status->current_segment = ch->stream_current;
        }
      else if (ch->list == NULL)
        {
          status->current_segment = 0;
        }
      else if (status->next == 0 && ch->mode == HPM_DMA_MODE_CHAIN)
        {
          status->current_segment = ch->list->count - 1;
        }
      else if (status->next >= ch->list->desc_busaddr)
        {
          offset = status->next - ch->list->desc_busaddr;
          if (offset % sizeof(struct hpm_dma_desc_s) == 0 &&
              offset / sizeof(struct hpm_dma_desc_s) < ch->list->count)
            {
              offset /= sizeof(struct hpm_dma_desc_s);
              status->current_segment = (offset + ch->list->count - 1) %
                                        ch->list->count;
            }
        }
    }

  status->position_valid = status->current_segment != SIZE_MAX;
  spin_unlock_irqrestore(&g_dma_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: hpm_dma_acquire_period
 *
 * Description:
 *   Transfer the oldest completed stream period from READY to CPU ownership.
 *
 ****************************************************************************/

int hpm_dma_acquire_period(hpm_dma_handle_t handle,
                           struct hpm_dma_period_s *period)
{
  struct hpm_dmach_s *ch = hpm_dma_from_handle(handle);
  struct hpm_dma_transfer_s *item;
  irqstate_t flags;
  size_t selected = SIZE_MAX;
  int ret = -EAGAIN;

  if (period == NULL)
    {
      return -EINVAL;
    }

  memset(period, 0, sizeof(*period));
  if (ch == NULL || !g_dma_initialized)
    {
      return -EINVAL;
    }

  memset(period, 0, sizeof(*period));
  flags = spin_lock_irqsave(&g_dma_lock);
  if (ch->mode != HPM_DMA_MODE_STREAM || ch->list == NULL ||
      (ch->state != HPM_DMA_STATE_RUNNING &&
       ch->state != HPM_DMA_STATE_COMPLETING &&
       ch->state != HPM_DMA_STATE_WAITING))
    {
      ret = -EINVAL;
    }
  else
    {
      selected = ch->ready_head;
      if (selected != SIZE_MAX)
        {
          item = &ch->list->items[selected];
          ch->ready_head = item->next_ready;
          if (ch->ready_head == SIZE_MAX)
            {
              ch->ready_tail = SIZE_MAX;
            }

          ch->ready_count--;
          item->ownership = HPM_DMA_BUFFER_CPU;
          ch->loans++;
          period->generation = ch->generation;
          period->sequence = item->sequence;
          period->index = selected;
          period->config = item->config;
          ret = OK;
        }
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: hpm_dma_release_period
 *
 * Description:
 *   Prepare one CPU-owned period before making it available to DMA again.
 *
 ****************************************************************************/

int hpm_dma_release_period(hpm_dma_handle_t handle,
                           const struct hpm_dma_period_s *period)
{
  struct hpm_dmach_s *ch = hpm_dma_from_handle(handle);
  struct hpm_dma_transfer_s *item;
  irqstate_t flags;
  int ret = hpm_dma_task_context();
  bool worker = ret == -EDEADLK;

  if (ret < 0 && !worker)
    {
      return ret;
    }

  if (ch == NULL || period == NULL || !g_dma_initialized)
    {
      return -EINVAL;
    }

  ret = worker ? nxmutex_trylock(&ch->lock) : nxmutex_lock(&ch->lock);
  if (ret < 0)
    {
      return worker && ret == -EBUSY ? -EAGAIN : ret;
    }

  flags = spin_lock_irqsave(&g_dma_lock);
  if (period->generation != ch->generation)
    {
      ret = -ESTALE;
    }
  else if (ch->mode != HPM_DMA_MODE_STREAM || ch->list == NULL ||
           period->index >= ch->list->count)
    {
      ret = -EINVAL;
    }
  else if (ch->state != HPM_DMA_STATE_RUNNING &&
           ch->state != HPM_DMA_STATE_COMPLETING &&
           ch->state != HPM_DMA_STATE_WAITING &&
           ch->state != HPM_DMA_STATE_ALLOCATED &&
           ch->state != HPM_DMA_STATE_FAULT)
    {
      ret = -EBUSY;
    }
  else
    {
      item = &ch->list->items[period->index];
      if (item->ownership != HPM_DMA_BUFFER_CPU ||
          item->sequence != period->sequence)
        {
          ret = -ESTALE;
        }
      else if (ch->state == HPM_DMA_STATE_ALLOCATED ||
               ch->state == HPM_DMA_STATE_FAULT)
        {
          item->ownership = HPM_DMA_BUFFER_QUEUED;
          ch->loans--;
          ret = OK;
        }
      else
        {
          item->ownership = HPM_DMA_BUFFER_PREPARING;
          spin_unlock_irqrestore(&g_dma_lock, flags);
          hpm_dma_cache_prepare(item);
          UP_DSB();
          flags = spin_lock_irqsave(&g_dma_lock);
          item->ownership = HPM_DMA_BUFFER_QUEUED;
          ch->loans--;
          ret = ch->state == HPM_DMA_STATE_FAULT ? -EIO : OK;
          if (ch->state == HPM_DMA_STATE_WAITING && !ch->work_active &&
              ch->stream_current == period->index)
            {
              ret = hpm_dma_schedule(ch, 0);
            }
        }
    }

  spin_unlock_irqrestore(&g_dma_lock, flags);
  nxmutex_unlock(&ch->lock);
  return ret;
}

/****************************************************************************
 * Name: hpm_dma_buffer_alloc
 *
 * Description:
 *   Allocate padded, DMA-readable RAM without changing transfer byte counts.
 *
 ****************************************************************************/

int hpm_dma_buffer_alloc(size_t nbytes, void **buffer, size_t *allocation)
{
  struct hpm_dma_span_s span;
  uintptr_t ignored;
  size_t alignment = HPM_DMA_DESCRIPTOR_ALIGN;
  size_t bytes;
  void *memory;
  int ret = hpm_dma_task_context();

  if (buffer == NULL || allocation == NULL)
    {
      return -EINVAL;
    }

  *buffer = NULL;
  *allocation = 0;
  if (ret < 0)
    {
      return ret;
    }

  if (nbytes == 0)
    {
      return -EINVAL;
    }

#ifdef CONFIG_ARCH_DCACHE
  alignment = up_get_dcache_linesize();
  if (alignment == 0 || (alignment & (alignment - 1)) != 0)
    {
      return -ENOTSUP;
    }

  if (alignment < HPM_DMA_DESCRIPTOR_ALIGN)
    {
      alignment = HPM_DMA_DESCRIPTOR_ALIGN;
    }

#endif

  if (nbytes > SIZE_MAX - (alignment - 1))
    {
      return -EOVERFLOW;
    }

  bytes = (nbytes + alignment - 1) & ~(alignment - 1);
  memory = kmm_memalign(alignment, bytes);
  if (memory == NULL)
    {
      return -ENOMEM;
    }

  ret = hpm_dma_address_span((uintptr_t)memory, bytes,
                             HPM_DMA_WIDTH_8BIT, HPM_DMA_ADDR_INCREMENT,
                             &span);
  if (ret == OK)
    {
      ret = hpm_dma_map_ram((uintptr_t)memory, &span, &ignored);
    }

  if (ret < 0)
    {
      kmm_free(memory);
    }
  else
    {
      *buffer = memory;
      *allocation = bytes;
    }

  return ret;
}
