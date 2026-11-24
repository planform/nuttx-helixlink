#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#

"""Build the actual HPM5300 DMA driver against a host register/OS model.

Run from any directory with Python 3 and a host C compiler (CC, default cc).
This Linux test maps the model DLM at its chip address with MAP_FIXED_NOREPLACE.
It checks driver behavior, not silicon timing, request wiring or cache HW.
"""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

MOCK = r"""
#ifndef HPM_DMA_TEST_MOCK_H
#define HPM_DMA_TEST_MOCK_H
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>
#define OK 0
#define _err(...) ((void)0)
#define HPM_IRQ_HDMA 50
#define CONFIG_ARCH_CHIP_HPM5361 1
#define CONFIG_HPM_HDMA 1
#define CONFIG_HPM_HDMA_STOP_TIMEOUT 100
#define CONFIG_HPM_HDMA_MAX_DESCRIPTORS 64
#define CONFIG_HPM_HDMA_WORKPRIORITY 224
#define CONFIG_HPM_HDMA_WORKSTACKSIZE 2048
#define NXMUTEX_INITIALIZER {false}
#ifdef CONFIG_PM
enum pm_state_e { PM_RESTORE = -1, PM_NORMAL, PM_IDLE, PM_STANDBY, PM_SLEEP };
struct pm_callback_s
{
  int (*prepare)(struct pm_callback_s *, int, enum pm_state_e);
  void (*notify)(struct pm_callback_s *, int, enum pm_state_e);
};
int pm_register(struct pm_callback_s *cb);
#endif
#define aligned_data(n) __attribute__((aligned(n)))
#define MSEC2TICK(n) (n)
typedef unsigned int irqstate_t;
typedef struct { bool locked; } mutex_t;
typedef int spinlock_t;
typedef struct { unsigned int count; } sem_t;
#define SEM_PRIO_NONE 0
int nxsem_init(sem_t *, int, unsigned int);
int nxsem_set_protocol(sem_t *, int);
int nxsem_post(sem_t *);
int nxsem_tickwait_uninterruptible(sem_t *, clock_t);
struct work_s { void (*worker)(void *); void *arg; clock_t due; bool queued; };
struct kwork_wqueue_s { int placeholder; };
struct kwork_wqueue_s *work_queue_create(const char *, int, void *, int, int);
int work_queue_wq(struct kwork_wqueue_s *, struct work_s *,
                  void (*)(void *), void *, clock_t);
pid_t nxsched_gettid(void);
uint32_t mock_read_csr(unsigned int csr);
#define READ_CSR(csr) mock_read_csr(csr)
#define SP_UNLOCKED 0
extern int lock_depth;
extern bool in_irq;
uint32_t getreg32(uintptr_t address);
void putreg32(uint32_t value, uintptr_t address);
int hpm_dma_clockconfig(void);
int irq_attach(int irq, int (*handler)(int, void *, void *), void *arg);
void up_disable_irq(int irq);
void up_enable_irq(int irq);
void up_udelay(unsigned int delay);
clock_t clock_systime_ticks(void);
int nxsig_usleep(unsigned int delay);
void mock_cache(unsigned int operation, uintptr_t first, uintptr_t end);
void *kmm_zalloc(size_t size);
void *kmm_memalign(size_t alignment, size_t size);
void kmm_free(void *ptr);
static inline bool up_interrupt_context(void) { return in_irq; }
static inline int nxmutex_init(mutex_t *m) { m->locked = false; return 0; }
static inline int nxmutex_lock(mutex_t *m)
{
  assert(!in_irq && !m->locked); m->locked = true; return 0;
}
static inline int nxmutex_trylock(mutex_t *m)
{
  if (m->locked) return -EBUSY;
  m->locked = true; return 0;
}
static inline int nxmutex_unlock(mutex_t *m)
{
  assert(m->locked); m->locked = false; return 0;
}
static inline irqstate_t spin_lock_irqsave(spinlock_t *lock)
{
  (void)lock; assert(lock_depth++ == 0); return 0;
}
static inline void spin_unlock_irqrestore(spinlock_t *lock, irqstate_t flags)
{
  (void)lock; (void)flags; assert(--lock_depth == 0);
}
#define UP_DSB() do {} while (0)
#ifdef CONFIG_ARCH_DCACHE
#define up_get_dcache_linesize() MOCK_CACHE_LINE_SIZE
#define up_clean_dcache(a,b) mock_cache(0,a,b)
#define up_flush_dcache(a,b) mock_cache(1,a,b)
#define up_invalidate_dcache(a,b) mock_cache(2,a,b)
#else
#define up_clean_dcache(a,b) ((void)(a), (void)(b))
#define up_flush_dcache(a,b) ((void)(a), (void)(b))
#define up_invalidate_dcache(a,b) ((void)(a), (void)(b))
#endif
#endif
"""

TEST = r"""
#include <stdio.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <string.h>
#include "mock.h"
#include "DRIVER_PATH"

int lock_depth;
bool in_irq;
static uint32_t regs[0x500 / 4];
static uint32_t mux[32];
static clock_t ticks;
static int init_failure;
static bool stuck;
static int drain_after;
static int irq_calls;
static int notifications;
static uint32_t last_events;
static int cache_calls[3];
static int checks;
static unsigned int mmio_reads;
static unsigned int mmio_writes;
static void (*cache_hook)(void);
static unsigned int invalidated_dlm_views;
#ifdef CONFIG_PM
static struct pm_callback_s *registered_pm;
#endif
static uintptr_t last_cache_first[3];
static uintptr_t last_cache_end[3];
static hpm_dma_handle_t restart_peer;
static pid_t mock_pid = 1;
static bool worker_executing;
static bool block_worker;
static bool queue_failure;
static bool create_failure;
static bool mock_cache_enabled;
static bool return_from_callback;
static int callback_release_result;
static struct hpm_dma_period_s callback_period;
static struct kwork_wqueue_s mock_queue;
static void run_work(void);
static bool chain_supported = true;
static int alloc_fail_after = -1;
static bool descriptor_outside_ram;
static unsigned int live_allocations;
static unsigned int descriptor_sizes[2048];
static unsigned int descriptor_hint;
static bool descriptor_used[2048];
#define CHECK(x) do { checks++; if (!(x)) {   fprintf(stderr, "line %d: %s\n", __LINE__, #x); abort(); } } while (0)

uint32_t getreg32(uintptr_t address)
{
  mmio_reads++;
  assert(address >= HPM_HDMA_BASE && address < HPM_HDMA_BASE + sizeof(regs));
  if (address == HPM_DMA_DMACFG)
    return chain_supported ? HPM_DMA_DMACFG_CHAINXFR : 0;
  return regs[(address - HPM_HDMA_BASE) / 4];
}

void putreg32(uint32_t value, uintptr_t address)
{
  unsigned int off;
  unsigned int ch;
  mmio_writes++;
  if (address >= HPM_DMAMUX_BASE && address < HPM_DMAMUX_BASE + sizeof(mux))
    {
      mux[(address - HPM_DMAMUX_BASE) / 4] = value;
      return;
    }

  assert(address >= HPM_HDMA_BASE && address < HPM_HDMA_BASE + sizeof(regs));
  off = address - HPM_HDMA_BASE;
  if (off == HPM_DMA_DMACTRL_OFFSET)
    {
      if (init_failure != 2)
        {
          memset(regs, 0, sizeof(regs));
        }
      else
        {
          regs[off / 4] = value;
        }
    }
  else if (off >= HPM_DMA_INTHALFSTS_OFFSET && off <= HPM_DMA_INTERRSTS_OFFSET)
    {
      regs[off / 4] &= ~value;
    }
  else if (off == HPM_DMA_CHABORT_OFFSET)
    {
      if (!stuck)
        {
          value &= regs[HPM_DMA_CHEN_OFFSET / 4];
          regs[HPM_DMA_CHEN_OFFSET / 4] &= ~value;
          regs[HPM_DMA_INTABORTSTS_OFFSET / 4] |= value;
        }
    }
  else
    {
      regs[off / 4] = value;
      if (off >= HPM_DMA_CH_BASE_OFFSET &&
          (off - HPM_DMA_CH_BASE_OFFSET) % HPM_DMA_CH_STRIDE == 0)
        {
          ch = (off - HPM_DMA_CH_BASE_OFFSET) / HPM_DMA_CH_STRIDE;
          if (value & HPM_DMA_CH_CTRL_ENABLE)
            regs[HPM_DMA_CHEN_OFFSET / 4] |= HPM_DMA_CHANNEL_BIT(ch);
          else
            regs[HPM_DMA_CHEN_OFFSET / 4] &= ~HPM_DMA_CHANNEL_BIT(ch);
        }
    }
}

int hpm_dma_clockconfig(void) { return init_failure == 1 ? -EIO : 0; }
int irq_attach(int irq, int (*handler)(int, void *, void *), void *arg)
{
  CHECK(irq == HPM_IRQ_HDMA && handler == hpm_dma_interrupt && arg == NULL);
  return init_failure == 3 ? -EIO : 0;
}
void up_disable_irq(int irq) { CHECK(irq == HPM_IRQ_HDMA); }
void up_enable_irq(int irq) { CHECK(irq == HPM_IRQ_HDMA); irq_calls++; }
void up_udelay(unsigned int delay) { (void)delay; }
clock_t clock_systime_ticks(void) { return ticks; }
int nxsig_usleep(unsigned int delay)
{
  CHECK(lock_depth == 0 && delay == 1000);
  ticks++;
  if (drain_after > 0 && --drain_after == 0)
    regs[HPM_DMA_IDMISC_OFFSET / 4] = 0;
  run_work();
  return 0;
}
void mock_cache(unsigned int operation, uintptr_t first, uintptr_t end)
{
  CHECK(!in_irq && lock_depth == 0 && first < end);
  cache_calls[operation]++;
  if (operation == 2)
    {
      if (first >= HPM_DLM_BASE && first < HPM_DLM_BASE + HPM_DLM_SIZE) invalidated_dlm_views |= 1;
      if (first >= HPM_DLM_ALIAS_BASE && first < HPM_DLM_ALIAS_BASE + HPM_DLM_SIZE) invalidated_dlm_views |= 2;
      if (first >= HPM_DLM_SYSTEM_BASE && first < HPM_DLM_SYSTEM_BASE + HPM_DLM_SIZE) invalidated_dlm_views |= 4;
    }
  last_cache_first[operation] = first;
  last_cache_end[operation] = end;
  if (cache_hook != NULL)
    {
      void (*hook)(void) = cache_hook;
      cache_hook = NULL;
      hook();
    }
}

static bool allocation_fails(void)
{
  CHECK(!in_irq && lock_depth == 0);
  if (alloc_fail_after < 0) return false;
  if (alloc_fail_after-- == 0) return true;
  return false;
}

void *kmm_zalloc(size_t size)
{
  void *ptr;
  if (allocation_fails()) return NULL;
  ptr = calloc(1, size);
  if (ptr != NULL) live_allocations++;
  return ptr;
}

void *kmm_memalign(size_t alignment, size_t size)
{
  unsigned int count = (size + 31) / 32;
  unsigned int i;
  unsigned int j;
  void *ptr;
  if (allocation_fails()) return NULL;
  if (descriptor_outside_ram)
    {
      ptr = aligned_alloc(alignment, size);
      if (ptr != NULL) live_allocations++;
      return ptr;
    }
  for (i = descriptor_hint; i + count <= 2048; i++)
    {
      uintptr_t address = 0x90000 + 32 * i;
      if ((address & (alignment - 1)) != 0) continue;
      for (j = 0; j < count && !descriptor_used[i + j]; j++);
      if (j != count) continue;
      for (j = 0; j < count; j++) descriptor_used[i + j] = true;
      descriptor_sizes[i] = count;
      live_allocations++;
      return (void *)address;
    }
  return NULL;
}

void kmm_free(void *ptr)
{
  uintptr_t address = (uintptr_t)ptr;
  unsigned int i;
  unsigned int count;
  if (ptr == NULL) return;
  CHECK(!in_irq && lock_depth == 0 && live_allocations > 0);
  live_allocations--;
  if (address >= 0x90000 && address < 0xa0000)
    {
      i = (address - 0x90000) / 32;
      count = descriptor_sizes[i];
      CHECK(count != 0);
      descriptor_sizes[i] = 0;
      for (; count > 0; count--, i++)
        {
          CHECK(descriptor_used[i]); descriptor_used[i] = false;
        }
    }
  else free(ptr);
}

#ifdef CONFIG_PM
int pm_register(struct pm_callback_s *cb)
{
  CHECK(cb != NULL && cb->prepare != NULL && cb->notify != NULL);
  registered_pm = cb;
  return 0;
}
#endif

uint32_t mock_read_csr(unsigned int csr)
{
  CHECK(csr == HPM_CSR_MCACHE_CTL);
  return mock_cache_enabled ? HPM_MCACHE_CTL_DC_EN : 0;
}

pid_t nxsched_gettid(void) { return mock_pid; }

struct kwork_wqueue_s *work_queue_create(const char *name, int priority,
                                        void *stack, int size, int threads)
{
  CHECK(!in_irq && lock_depth == 0 && threads == 1 && stack == NULL);
  CHECK(priority == CONFIG_HPM_HDMA_WORKPRIORITY &&
        size == CONFIG_HPM_HDMA_WORKSTACKSIZE && strcmp(name, "hpm_dma") == 0);
  return create_failure ? NULL : &mock_queue;
}

int work_queue_wq(struct kwork_wqueue_s *queue, struct work_s *work,
                  void (*worker)(void *), void *arg, clock_t delay)
{
  CHECK(queue == &mock_queue);
  if (queue_failure) return -EIO;
  CHECK(!work->queued);
  work->worker = worker;
  work->arg = arg;
  work->due = ticks + delay;
  work->queued = true;
  return 0;
}

static void run_work(void)
{
  unsigned int i;
  unsigned int budget = 100;
  pid_t pid = mock_pid;
  if (worker_executing || block_worker) return;
  worker_executing = true;
  mock_pid = 2;
  do
    {
      for (i = 0; i < HPM_DMA_NCHANNELS; i++)
        {
          struct work_s *work = &g_dma_channels[i].work;
          if (work->queued && work->due <= ticks)
            {
              void (*worker)(void *) = work->worker;
              void *arg = work->arg;
              CHECK(!in_irq && lock_depth == 0 && budget-- > 0);
              work->queued = false;
              work->worker = NULL;
              worker(arg);
              break;
            }
        }
    }
  while (i < HPM_DMA_NCHANNELS);
  mock_pid = pid;
  worker_executing = false;
}

int nxsem_init(sem_t *sem, int shared, unsigned int value)
{
  (void)shared; sem->count = value; return 0;
}
int nxsem_set_protocol(sem_t *sem, int protocol)
{
  (void)sem; CHECK(protocol == SEM_PRIO_NONE); return 0;
}
int nxsem_post(sem_t *sem) { sem->count++; return 0; }
int nxsem_tickwait_uninterruptible(sem_t *sem, clock_t timeout)
{
  CHECK(!in_irq && lock_depth == 0 && mock_pid != 2);
  run_work();
  if (sem->count == 0)
    {
      ticks += timeout;
      run_work();
    }
  if (sem->count == 0) return -ETIMEDOUT;
  sem->count--; return 0;
}

static void callback(hpm_dma_handle_t handle, uint32_t events, void *arg)
{
  hpm_dma_handle_t other = handle;
  struct hpm_dma_status_s status;
  CHECK(!in_irq && mock_pid == 2 && lock_depth == 0 && arg == &notifications);
  CHECK(hpm_dma_alloc(&other) == -EDEADLK && other == NULL);
  CHECK(hpm_dma_stop(handle) == -EDEADLK);
  CHECK(hpm_dma_getstatus(handle, &status) == 0 && status.work_pending);
  if (return_from_callback && (events & HPM_DMA_EVENT_PERIOD) != 0 &&
      handle->mode == HPM_DMA_MODE_STREAM)
    {
      CHECK(hpm_dma_acquire_period(handle, &callback_period) == 0);
      callback_release_result = hpm_dma_release_period(handle, &callback_period);
      CHECK(callback_release_result == 0 || callback_release_result == -EAGAIN);
    }
  notifications++;
  last_events = events;
  if (restart_peer != NULL)
    {
      hpm_dma_handle_t peer = restart_peer;
      irqstate_t flags;
      struct hpm_dma_config_s c = peer->transfer.config;
      restart_peer = NULL;
      mock_pid = 1; /* Model another CPU beginning cancellation. */
      CHECK(hpm_dma_setup(peer, &c) == -EBUSY);
      flags = spin_lock_irqsave(&g_dma_lock);
      peer->state = HPM_DMA_STATE_STOPPING;
      peer->callback = NULL;
      peer->pending_events = 0;
      g_dma_running &= ~HPM_DMA_CHANNEL_BIT(peer->channel);
      spin_unlock_irqrestore(&g_dma_lock, flags);
      mock_pid = 2;
    }
}

static void interrupt_channel(hpm_dma_handle_t ch, uint32_t events, bool enabled)
{
  uint32_t bit = HPM_DMA_CHANNEL_BIT(ch->channel);
  if (!enabled)
    regs[HPM_DMA_CHEN_OFFSET / 4] &= ~bit;
  if (events & HPM_DMA_EVENT_HALF) regs[HPM_DMA_INTHALFSTS_OFFSET / 4] |= bit;
  if (events & HPM_DMA_EVENT_COMPLETE) regs[HPM_DMA_INTTCSTS_OFFSET / 4] |= bit;
  if (events & HPM_DMA_EVENT_ERROR) regs[HPM_DMA_INTERRSTS_OFFSET / 4] |= bit;
  if (events & HPM_DMA_EVENT_ABORT) regs[HPM_DMA_INTABORTSTS_OFFSET / 4] |= bit;
  in_irq = true;
  CHECK(hpm_dma_interrupt(HPM_IRQ_HDMA, NULL, NULL) == 0);
  in_irq = false;
  run_work();
}

static struct hpm_dma_config_s config(void)
{
  struct hpm_dma_config_s c = {0};
  c.src_addr = HPM_DLM_BASE;
  c.dst_addr = HPM_DLM_BASE + 0x2000;
  c.nbytes = 64;
  c.src_width = HPM_DMA_WIDTH_32BIT;
  c.dst_width = HPM_DMA_WIDTH_32BIT;
  c.burst = 4;
  return c;
}

static void validation(void)
{
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c = config();
  struct hpm_dma_span_s span;
  uint32_t count;
  unsigned int i;
  unsigned int j;
  uintptr_t address;
  CHECK(hpm_dma_alloc(&ch) == 0);
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK(ch->transfer.ntransfers == 16);
  CHECK(HPM_DMA_CH_CTRL_SRCBURSTSIZE_GET(ch->transfer.ctrl) == 2);
  CHECK(!(ch->transfer.ctrl & HPM_DMA_CH_CTRL_ENABLE));
  CHECK(hpm_dma_setup(ch, NULL) == -EINVAL);
  CHECK(hpm_dma_setup(NULL, &c) == -EINVAL);
  CHECK(hpm_dma_setup((hpm_dma_handle_t)(uintptr_t)123, &c) == -EINVAL);
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      {
        c.src_width = i; c.dst_width = j;
        CHECK(hpm_dma_setup(ch, &c) == (i < j ? -ENOTSUP : 0));
      }
  c = config(); c.src_addr++;
  CHECK(hpm_dma_setup(ch, &c) == -EINVAL);
  c = config(); c.nbytes = 63;
  CHECK(hpm_dma_setup(ch, &c) == -EINVAL);
  c = config(); c.nbytes = 0;
  CHECK(hpm_dma_setup(ch, &c) == -EINVAL);
  c = config(); c.burst = 17;
  CHECK(hpm_dma_setup(ch, &c) == -EINVAL);
  c = config(); c.burst = 256;
  CHECK(hpm_dma_setup(ch, &c) == -EINVAL);
  for (i = 0; i <= 129; i++)
    {
      bool valid = i >= 1 && (i <= 16 || i == 32 || i == 64 || i == 128);
      c = config(); c.burst = i;
      CHECK(hpm_dma_setup(ch, &c) == (valid ? 0 : -EINVAL));
      if (valid)
        {
          bool linear = (i & (i - 1)) != 0;
          uint32_t field = HPM_DMA_CH_CTRL_SRCBURSTSIZE_GET(ch->transfer.ctrl);
          CHECK(((ch->transfer.ctrl & HPM_DMA_CH_CTRL_BURSTOPT) != 0) == linear);
          CHECK((linear ? field + 1 : 1U << field) == i);
        }
    }
  c = config(); c.src_width = -1;
  CHECK(hpm_dma_setup(ch, &c) == -EINVAL);
  c = config(); c.src_addr = HPM_DLM_BASE + HPM_DLM_SIZE - 32;
  CHECK(hpm_dma_setup(ch, &c) == -EFAULT);
  c = config(); c.src_addr = 0x20000000;
  CHECK(hpm_dma_setup(ch, &c) == 0);
  c = config(); c.dst_addr = HPM_DLM_ALIAS_BASE;
  CHECK(hpm_dma_setup(ch, &c) == -EINVAL);
  c = config(); c.src_addrmode = HPM_DMA_ADDR_DECREMENT;
  c.dst_addrmode = HPM_DMA_ADDR_DECREMENT;
  c.src_addr += 60; c.dst_addr += 60;
  CHECK(hpm_dma_setup(ch, &c) == 0 && ch->transfer.src_span.first == HPM_DLM_BASE);
  c = config(); c.src_addrmode = HPM_DMA_ADDR_FIXED;
  CHECK(hpm_dma_setup(ch, &c) == 0 && ch->transfer.src_span.last == c.src_addr + 3);
  c = config(); c.nbytes = (size_t)HPM_DMA_CH_TRANSIZE_MASK * 4;
  CHECK(hpm_dma_transfer_geometry(&c, &count) == 0 && count == HPM_DMA_CH_TRANSIZE_MASK);
  c.nbytes += 4;
  CHECK(hpm_dma_transfer_geometry(&c, &count) == -E2BIG && count == 0);
  CHECK(hpm_dma_address_span(0, 8, 2, 1, &span) == -EOVERFLOW);
  CHECK(hpm_dma_address_span(UINTPTR_MAX, 2, 0, 0, &span) == -EOVERFLOW);
  CHECK(hpm_dma_address_span(UINTPTR_MAX, 1, 0, 0, &span) == 0);
  CHECK(!hpm_dma_span_in_region(&span, UINTPTR_MAX, 2));
  CHECK(!hpm_dma_span_in_region(NULL, 0, 1));
  CHECK(hpm_dma_map_ram(HPM_DLM_BASE, NULL, &address) == -EINVAL && address == 0);
#ifdef CONFIG_ARCH_DCACHE
  c = config(); c.dst_addr += 4;
  CHECK(hpm_dma_setup(ch, &c) == -EINVAL);
  c = config(); c.nbytes = 4;
  CHECK(hpm_dma_setup(ch, &c) == -EINVAL);
#endif
  {
    static const uintptr_t windows[] =
    {
      HPM_ILM_BASE, HPM_ILM_ALIAS_BASE, HPM_ILM_SYSTEM_BASE,
      HPM_DLM_BASE, HPM_DLM_ALIAS_BASE, HPM_DLM_SYSTEM_BASE, HPM_HRAM_BASE
    };
    for (i = 0; i < sizeof(windows) / sizeof(windows[0]); i++)
      {
        c = config(); c.src_addr = windows[i];
        CHECK(hpm_dma_setup(ch, &c) == 0 && ch->transfer.src_busaddr == windows[i]);
      }
  }
  CHECK(hpm_dma_free(ch) == 0);
}

static void lifecycle(void)
{
  hpm_dma_handle_t ch;
  hpm_dma_handle_t all[HPM_DMA_NCHANNELS];
  hpm_dma_handle_t extra;
  struct hpm_dma_config_s c = config();
  unsigned int i;
  int notified;
  uint32_t bit;
  CHECK(hpm_dma_alloc(&ch) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == -EBUSY);
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK(hpm_dma_start(ch, NULL, NULL) == -EINVAL);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  CHECK(getreg32(HPM_DMA_CH_SRCADDR(ch->channel)) == c.src_addr);
  CHECK(getreg32(HPM_DMA_CH_DSTADDR(ch->channel)) == c.dst_addr);
  CHECK(getreg32(HPM_DMA_CH_TRANSIZE(ch->channel)) == 16);
  CHECK(getreg32(HPM_DMA_CH_LLPOINTER(ch->channel)) == 0);
  CHECK(mux[ch->channel] == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == -EBUSY);
  CHECK(hpm_dma_setup(ch, &c) == -EBUSY);
  CHECK(hpm_dma_free(ch) == -EBUSY);
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE, false);
  CHECK(last_events == HPM_DMA_EVENT_COMPLETE && ch->state == HPM_DMA_STATE_ALLOCATED);
  CHECK(hpm_dma_start(ch, callback, &notifications) == -EBUSY);
  c.half_interrupt = true;
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  interrupt_channel(ch, HPM_DMA_EVENT_HALF, true);
  CHECK(last_events == HPM_DMA_EVENT_HALF && ch->state == HPM_DMA_STATE_RUNNING);
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE | HPM_DMA_EVENT_ERROR, false);
  CHECK(last_events == HPM_DMA_EVENT_ERROR && ch->state == HPM_DMA_STATE_FAULT);
  CHECK(hpm_dma_free(ch) == -EBUSY);
  CHECK(hpm_dma_stop(ch) == 0);
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  notified = notifications;
  CHECK(hpm_dma_stop(ch) == 0 && notifications == notified);
  interrupt_channel(ch, HPM_DMA_EVENT_ABORT, false);
  CHECK(notifications == notified);
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  stuck = true;
  CHECK(hpm_dma_stop(ch) == -ETIMEDOUT);
  CHECK(ch->state == HPM_DMA_STATE_FAULT && hpm_dma_free(ch) == -EBUSY);
  CHECK(hpm_dma_setup(ch, &c) == -EBUSY);
  stuck = false;
  CHECK(hpm_dma_stop(ch) == 0 && notifications == notified);
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  regs[HPM_DMA_IDMISC_OFFSET / 4] = (4U << HPM_DMA_IDMISC_DMASTATE_SHIFT) |
                                  (ch->channel << HPM_DMA_IDMISC_CURCHAN_SHIFT);
  CHECK(hpm_dma_stop(ch) == -ETIMEDOUT);
  drain_after = 3;
  CHECK(hpm_dma_stop(ch) == 0);
  CHECK(hpm_dma_free(ch) == 0);
  for (i = 0; i < HPM_DMA_NCHANNELS; i++) CHECK(hpm_dma_alloc(&all[i]) == 0);
  extra = all[0];
  CHECK(hpm_dma_alloc(&extra) == -EBUSY && extra == NULL);
  CHECK(all[31]->channel == 31);
  ch = all[31]; bit = HPM_DMA_CHANNEL_BIT(31);
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  regs[HPM_DMA_INTTCSTS_OFFSET / 4] |= 1;
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE, false);
  CHECK((regs[HPM_DMA_INTTCSTS_OFFSET / 4] & bit) == 0);
  for (i = 0; i < HPM_DMA_NCHANNELS; i++) CHECK(hpm_dma_free(all[i]) == 0);
}

static void peripherals(void)
{
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c = config();
  unsigned int i;
  CHECK(hpm_dma_alloc(&ch) == 0);
  c.direction = HPM_DMA_MEM_TO_PERIPH;
  c.dst_addr = HPM_SPI1_BASE + 0x20;
  c.dst_addrmode = HPM_DMA_ADDR_FIXED;
  c.request = HPM_DMA_SRC_SPI1_TX;
  c.handshake_block = true;
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK((ch->transfer.ctrl & HPM_DMA_CH_CTRL_DSTMODE) != 0);
  CHECK((ch->transfer.ctrl & HPM_DMA_CH_CTRL_SRCMODE) == 0);
  CHECK((ch->transfer.ctrl & HPM_DMA_CH_CTRL_HANDSHAKEOPT) != 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  CHECK(mux[ch->channel] == (HPM_DMAMUX_MUXCFG_ENABLE | c.request));
  CHECK(HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_GET(getreg32(HPM_DMA_CH_CHANREQCTRL(ch->channel))) == ch->channel);
  CHECK(hpm_dma_stop(ch) == 0 && mux[ch->channel] == 0);
  c = config(); c.direction = HPM_DMA_PERIPH_TO_MEM;
  c.src_addr = HPM_UART3_BASE + 0x20;
  c.src_addrmode = HPM_DMA_ADDR_FIXED;
  c.request = HPM_DMA_SRC_UART3_RX;
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK((ch->transfer.ctrl & HPM_DMA_CH_CTRL_SRCMODE) != 0);
  for (i = 0; i < 128; i++)
    {
      bool valid = i <= 15 || (i >= 20 && i <= 65);
      c.request = i;
      CHECK(hpm_dma_setup(ch, &c) == (valid ? 0 : -EINVAL));
    }
  c.request = 0;
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  interrupt_channel(ch, HPM_DMA_EVENT_ABORT, false);
  CHECK(last_events == HPM_DMA_EVENT_ABORT);
  CHECK(hpm_dma_stop(ch) == 0);
  CHECK(hpm_dma_free(ch) == 0);
}


/* An old status must never be delivered to a new transfer after another
 * channel's callback permits concurrent stop/setup/start on the same handle.
 */

static void concurrent_channels(void)
{
  hpm_dma_handle_t first;
  hpm_dma_handle_t second;
  struct hpm_dma_config_s c = config();
  uint32_t second_bit;
  int notified;
  CHECK(hpm_dma_alloc(&first) == 0);
  CHECK(hpm_dma_alloc(&second) == 0);
  CHECK(hpm_dma_setup(first, &c) == 0);
  CHECK(hpm_dma_setup_chain(second, &c, 1, true) == 0);
  CHECK(hpm_dma_start(first, callback, &notifications) == 0);
  CHECK(hpm_dma_start(second, callback, &notifications) == 0);
  second_bit = HPM_DMA_CHANNEL_BIT(second->channel);
  regs[HPM_DMA_CHEN_OFFSET / 4] &= ~second_bit;
  regs[HPM_DMA_INTTCSTS_OFFSET / 4] |= second_bit;
  restart_peer = second;
  notified = notifications;
  interrupt_channel(first, HPM_DMA_EVENT_COMPLETE, false);
  CHECK(notifications == notified + 1);
  CHECK(hpm_dma_stop(second) == 0);
  CHECK(hpm_dma_setup(second, &c) == 0);
  CHECK(hpm_dma_start(second, callback, &notifications) == 0);
  CHECK(second->state == HPM_DMA_STATE_RUNNING && hpm_dma_enabled(second));
  interrupt_channel(second, HPM_DMA_EVENT_COMPLETE, false);
  CHECK(notifications == notified + 2);
  CHECK(hpm_dma_setup(first, &c) == 0);
  CHECK(hpm_dma_setup(second, &c) == 0);
  CHECK(hpm_dma_start(first, callback, &notifications) == 0);
  CHECK(hpm_dma_start(second, callback, &notifications) == 0);
  regs[HPM_DMA_CHEN_OFFSET / 4] &= ~second_bit;
  regs[HPM_DMA_INTTCSTS_OFFSET / 4] |= second_bit;
  CHECK(hpm_dma_stop(first) == 0);
  CHECK((regs[HPM_DMA_INTTCSTS_OFFSET / 4] & second_bit) != 0);
  interrupt_channel(second, 0, false);
  CHECK(notifications == notified + 3);
  CHECK(hpm_dma_free(first) == 0);
  CHECK(hpm_dma_free(second) == 0);
}

/* Consume the actual register image and descriptors. This is an independent
 * model of the SDK register layout, not a call to the driver's compiler.
 */

static void engine_step(hpm_dma_handle_t ch, bool deliver)
{
  unsigned int n = ch->channel;
  uint32_t ctrl = getreg32(HPM_DMA_CH_CTRL(n));
  unsigned int sw = 1U << HPM_DMA_CH_CTRL_SRCWIDTH_GET(ctrl);
  unsigned int dw = 1U << HPM_DMA_CH_CTRL_DSTWIDTH_GET(ctrl);
  unsigned int sm = HPM_DMA_CH_CTRL_SRCADDRCTRL_GET(ctrl);
  unsigned int dm = HPM_DMA_CH_CTRL_DSTADDRCTRL_GET(ctrl);
  uintptr_t src = getreg32(HPM_DMA_CH_SRCADDR(n));
  uintptr_t dst = getreg32(HPM_DMA_CH_DSTADDR(n));
  uint32_t count = getreg32(HPM_DMA_CH_TRANSIZE(n));
  uint32_t next = getreg32(HPM_DMA_CH_LLPOINTER(n));
  uint32_t bit = HPM_DMA_CHANNEL_BIT(n);
  unsigned int i;
  unsigned int j;
  unsigned char word[4];
  CHECK(hpm_dma_enabled(ch));
  for (i = 0; i < count; i++)
    {
      CHECK(src >= HPM_DLM_BASE && src + sw <= HPM_DLM_BASE + HPM_DLM_SIZE);
      memcpy(word, (void *)src, sw);
      for (j = 0; j < sw; j += dw)
        {
          CHECK(dst >= HPM_DLM_BASE && dst + dw <= HPM_DLM_BASE + HPM_DLM_SIZE);
          memcpy((void *)dst, word + j, dw);
          if (dm == HPM_DMA_ADDR_INCREMENT) dst += dw;
          else if (dm == HPM_DMA_ADDR_DECREMENT) dst -= dw;
        }
      if (sm == HPM_DMA_ADDR_INCREMENT) src += sw;
      else if (sm == HPM_DMA_ADDR_DECREMENT) src -= sw;
    }
  regs[HPM_DMA_INTTCSTS_OFFSET / 4] |= bit;
  if ((ctrl & HPM_DMA_CH_CTRL_INFINITELOOP) != 0)
    {
      CHECK(next == 0);
    }
  else if (next != 0)
    {
      const uint32_t *desc = (const uint32_t *)(uintptr_t)next;
      CHECK(next >= 0x90000 && next + 32 <= 0xa0000 && (next & 31) == 0);
      CHECK(desc[5] == 0 && desc[7] == 0);
      putreg32(desc[1], HPM_DMA_CH_TRANSIZE(n));
      putreg32(desc[2], HPM_DMA_CH_SRCADDR(n));
      putreg32(desc[3], HPM_DMA_CH_CHANREQCTRL(n));
      putreg32(desc[4], HPM_DMA_CH_DSTADDR(n));
      putreg32(desc[6], HPM_DMA_CH_LLPOINTER(n));
      putreg32(desc[0], HPM_DMA_CH_CTRL(n));
    }
  else
    {
      regs[HPM_DMA_CHEN_OFFSET / 4] &= ~bit;
      regs[HPM_DMA_CH_CTRL_OFFSET(n) / 4] &= ~HPM_DMA_CH_CTRL_ENABLE;
      regs[HPM_DMA_CH_TRANSIZE_OFFSET(n) / 4] = 0;
    }
  if (deliver &&
      (getreg32(HPM_DMA_CH_CTRL(n)) & HPM_DMA_CH_CTRL_INTTCMASK) == 0)
    {
      in_irq = true;
      CHECK(hpm_dma_interrupt(HPM_IRQ_HDMA, NULL, NULL) == 0);
      in_irq = false;
      run_work();
    }
}

static void chains(void)
{
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c[3];
  struct hpm_dma_list_s *old;
  unsigned char *src = (unsigned char *)(uintptr_t)HPM_DLM_BASE;
  unsigned char *dst = src + 0x2000;
  unsigned int live;
  unsigned int i;
  int notified;
  CHECK(hpm_dma_alloc(&ch) == 0);
  for (i = 0; i < 3; i++)
    {
      c[i] = config(); c[i].src_addr += 64 * i; c[i].dst_addr += 64 * i;
    }
  for (i = 0; i < 192; i++) src[i] = i ^ 0x5a;
  memset(dst, 0xcc, 224);
  descriptor_hint = 126; /* Place a node at the end of a 4 KiB page. */
  CHECK(hpm_dma_setup_chain(ch, c, 3, false) == 0);
  old = ch->list;
  CHECK(old != NULL && old->count == 3 && live_allocations == 2);
  CHECK(((uintptr_t)old->desc & 31) == 0);
  CHECK(((uintptr_t)old->desc & 0xfff) == 0xfc0);
  for (i = 0; i < 3; i++)
    {
      CHECK(old->desc[i].src_addr == c[i].src_addr);
      CHECK(old->desc[i].dst_addr == c[i].dst_addr);
      CHECK(old->desc[i].ntransfers == 16);
      CHECK((old->desc[i].ctrl & HPM_DMA_CH_CTRL_ENABLE) != 0);
      CHECK((old->desc[i].ctrl & HPM_DMA_CH_CTRL_INFINITELOOP) == 0);
      CHECK(((old->desc[i].ctrl & HPM_DMA_CH_CTRL_INTTCMASK) != 0) == (i < 2));
      CHECK(old->desc[i].reserved0 == 0 && old->desc[i].reserved1 == 0);
      CHECK(((uintptr_t)&old->desc[i] & 0xfff) <= 0xfe0);
      CHECK(old->desc[i].next == (i < 2 ? old->desc_busaddr + (i + 1) * 32 : 0));
    }
  descriptor_hint = 0;
  live = live_allocations;
  CHECK(hpm_dma_setup_chain(ch, NULL, 3, false) == -EINVAL && ch->list == old);
  CHECK(hpm_dma_setup_chain(ch, c, 0, false) == -EINVAL);
  CHECK(hpm_dma_setup_chain(ch, c, SIZE_MAX, false) == -E2BIG);
  chain_supported = false;
  CHECK(hpm_dma_setup_chain(ch, c, 3, false) == -ENOTSUP && ch->list == old);
  chain_supported = true;
  alloc_fail_after = 0;
  CHECK(hpm_dma_setup_chain(ch, c, 3, false) == -ENOMEM && ch->list == old);
  alloc_fail_after = 1;
  CHECK(hpm_dma_setup_chain(ch, c, 3, false) == -ENOMEM && ch->list == old);
  alloc_fail_after = -1;
  descriptor_outside_ram = true;
  CHECK(hpm_dma_setup_chain(ch, c, 3, false) == -EFAULT && ch->list == old);
  descriptor_outside_ram = false;
  c[1].burst = 17;
  CHECK(hpm_dma_setup_chain(ch, c, 3, false) == -EINVAL && ch->list == old);
  c[1].burst = 4; c[1].half_interrupt = true;
  CHECK(hpm_dma_setup_chain(ch, c, 3, false) == -EINVAL && ch->list == old);
  c[1].half_interrupt = false;
  c[0].dst_addr = HPM_DLM_ALIAS_BASE + 0x10000;
  CHECK(hpm_dma_setup_chain(ch, c, 3, false) == -EINVAL && ch->list == old);
  c[0] = config();
  CHECK(live_allocations == live);
  notified = notifications;
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  CHECK(getreg32(HPM_DMA_CH_LLPOINTER(ch->channel)) == old->desc_busaddr + 32);
  CHECK(hpm_dma_setup_repeat(ch, &c[0]) == -EBUSY);
  CHECK(hpm_dma_setup_chain(ch, c, 3, true) == -EBUSY);
  engine_step(ch, true);
  CHECK(memcmp(src, dst, 64) == 0 && dst[64] == 0xcc);
  CHECK(notifications == notified && ch->state == HPM_DMA_STATE_RUNNING);
  engine_step(ch, true); /* Tail loaded, old masked TC becomes visible. */
  CHECK(notifications == notified && ch->state == HPM_DMA_STATE_RUNNING);
  engine_step(ch, true);
  CHECK(notifications == notified + 1 && last_events == HPM_DMA_EVENT_COMPLETE);
  CHECK(memcmp(src, dst, 192) == 0 && dst[192] == 0xcc);
  CHECK(ch->state == HPM_DMA_STATE_ALLOCATED && live_allocations == live);
  CHECK(hpm_dma_setup_repeat(ch, &c[0]) == 0 && ch->list == NULL);
  CHECK(live_allocations == 0);
  CHECK(hpm_dma_free(ch) == 0);
}

static void cyclic_and_repeat(void)
{
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c[2];
  unsigned int live;
  unsigned int i;
  int notified;
  CHECK(hpm_dma_alloc(&ch) == 0);
  c[0] = config(); c[1] = config();
  c[1].src_addr += 64; c[1].dst_addr += 64;
  CHECK(hpm_dma_setup_chain(ch, c, 2, true) == 0);
  CHECK(ch->list->desc[1].next == ch->list->desc_busaddr);
  live = live_allocations;
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  notified = notifications;
  for (i = 0; i < 7; i++)
    {
      engine_step(ch, true);
      CHECK(last_events == HPM_DMA_EVENT_PERIOD && ch->state == HPM_DMA_STATE_RUNNING);
      CHECK(ch->callback == callback && live_allocations == live);
    }
  CHECK(notifications == notified + 7);
  CHECK(hpm_dma_free(ch) == -EBUSY);
  notified = notifications;
  for (i = 0; i < 5; i++) engine_step(ch, false);
  in_irq = true; CHECK(hpm_dma_interrupt(HPM_IRQ_HDMA, NULL, NULL) == 0); in_irq = false; run_work();
  CHECK(notifications == notified + 1 && last_events == HPM_DMA_EVENT_PERIOD);
  stuck = true;
  CHECK(hpm_dma_stop(ch) == -ETIMEDOUT && live_allocations == live);
  CHECK(hpm_dma_free(ch) == -EBUSY && ch->list != NULL);
  stuck = false;
  CHECK(hpm_dma_stop(ch) == 0 && live_allocations == live);
  CHECK(getreg32(HPM_DMA_CH_LLPOINTER(ch->channel)) == 0);
  CHECK(hpm_dma_setup_chain(ch, c, 1, true) == 0);
  CHECK(ch->list->desc[0].next == ch->list->desc_busaddr);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  engine_step(ch, true); engine_step(ch, true);
  CHECK(last_events == HPM_DMA_EVENT_PERIOD && hpm_dma_enabled(ch));
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE | HPM_DMA_EVENT_ERROR, false);
  CHECK(last_events == HPM_DMA_EVENT_ERROR && ch->state == HPM_DMA_STATE_FAULT);
  CHECK(hpm_dma_stop(ch) == 0);
  CHECK(hpm_dma_setup_repeat(ch, &c[0]) == 0 && live_allocations == 0);
  CHECK(ch->list == NULL && (ch->transfer.ctrl & HPM_DMA_CH_CTRL_INFINITELOOP) != 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  CHECK(getreg32(HPM_DMA_CH_LLPOINTER(ch->channel)) == 0);
  notified = notifications;
  for (i = 0; i < 5; i++) engine_step(ch, true);
  CHECK(notifications == notified + 5 && last_events == HPM_DMA_EVENT_PERIOD);
  CHECK(ch->state == HPM_DMA_STATE_RUNNING && hpm_dma_enabled(ch));
  CHECK(hpm_dma_stop(ch) == 0);
  CHECK(hpm_dma_setup(ch, &c[0]) == 0);
  CHECK((ch->transfer.ctrl & HPM_DMA_CH_CTRL_INFINITELOOP) == 0);
  CHECK(hpm_dma_free(ch) == 0 && live_allocations == 0);
}

static void chain_edges(void)
{
  hpm_dma_handle_t ch;
  hpm_dma_handle_t held[3];
  struct hpm_dma_config_s configs[CONFIG_HPM_HDMA_MAX_DESCRIPTORS];
  struct hpm_dma_list_s *old;
  unsigned int i;
  int notified;
  for (i = 0; i < 3; i++) CHECK(hpm_dma_alloc(&held[i]) == 0);
  CHECK(hpm_dma_alloc(&ch) == 0 && ch->channel == 3);
  configs[0] = config(); configs[1] = config();
  configs[0].direction = configs[1].direction = HPM_DMA_MEM_TO_PERIPH;
  configs[0].dst_addr = configs[1].dst_addr = HPM_SPI1_BASE + 0x20;
  configs[0].dst_addrmode = configs[1].dst_addrmode = HPM_DMA_ADDR_FIXED;
  configs[0].request = configs[1].request = HPM_DMA_SRC_SPI1_TX;
  configs[1].src_addr += 64;
  configs[1].src_width = HPM_DMA_WIDTH_16BIT;
  configs[1].dst_width = HPM_DMA_WIDTH_8BIT;
  configs[1].burst = 3;
  configs[1].high_priority = true;
  CHECK(hpm_dma_setup_chain(ch, configs, 2, false) == 0);
  old = ch->list;
  for (i = 0; i < 2; i++)
    {
      CHECK(HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_GET(old->desc[i].request) == old->items[i].request_port);
      CHECK(HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_GET(old->desc[i].request) == old->items[i].request_port);
      CHECK((old->desc[i].ctrl & HPM_DMA_CH_CTRL_DSTMODE) != 0);
      CHECK((old->desc[i].ctrl & HPM_DMA_CH_CTRL_SRCMODE) == 0);
    }
  CHECK(old->desc[1].ntransfers == 32);
  CHECK((old->desc[1].ctrl & HPM_DMA_CH_CTRL_PRIORITY) != 0);
  CHECK((old->desc[1].ctrl & HPM_DMA_CH_CTRL_BURSTOPT) != 0);
  configs[1].request = HPM_DMA_SRC_SPI2_TX;
  CHECK(hpm_dma_setup_chain(ch, configs, 2, false) == 0); old = ch->list;
  configs[1] = config();
  CHECK(hpm_dma_setup_chain(ch, configs, 2, false) == 0); old = ch->list;
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  CHECK(mux[ch->transfer.request_port] == (HPM_DMAMUX_MUXCFG_ENABLE | HPM_DMA_SRC_SPI1_TX));
  putreg32(0, HPM_DMA_CH_LLPOINTER(ch->channel));
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE, false);
  CHECK(last_events == HPM_DMA_EVENT_COMPLETE);
  configs[0] = config(); configs[1] = config();
  configs[0].half_interrupt = configs[1].half_interrupt = true;
  CHECK(hpm_dma_setup_chain(ch, configs, 2, false) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  notified = notifications;
  interrupt_channel(ch, HPM_DMA_EVENT_HALF | HPM_DMA_EVENT_COMPLETE, true);
  CHECK(notifications == notified + 1 && last_events == HPM_DMA_EVENT_HALF);
  CHECK(ch->state == HPM_DMA_STATE_RUNNING);
  CHECK(hpm_dma_stop(ch) == 0);
  CHECK(hpm_dma_setup_chain(ch, configs, 1, false) == 0);
  CHECK(ch->list->desc[0].next == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  engine_step(ch, true);
  CHECK(last_events == HPM_DMA_EVENT_COMPLETE);
  CHECK(hpm_dma_setup_chain(ch, configs, 2, true) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE, false);
  CHECK(last_events == HPM_DMA_EVENT_ERROR && ch->state == HPM_DMA_STATE_FAULT);
  CHECK(hpm_dma_stop(ch) == 0);
  CHECK(hpm_dma_setup_chain(ch, configs, 2, true) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE | HPM_DMA_EVENT_ERROR, true);
  CHECK(last_events == HPM_DMA_EVENT_ERROR && ch->state == HPM_DMA_STATE_FAULT);
  CHECK(hpm_dma_stop(ch) == 0);
  for (i = 0; i < CONFIG_HPM_HDMA_MAX_DESCRIPTORS; i++) configs[i] = config();
  CHECK(hpm_dma_setup_chain(ch, configs, CONFIG_HPM_HDMA_MAX_DESCRIPTORS, true) == 0);
  CHECK(ch->list->desc[CONFIG_HPM_HDMA_MAX_DESCRIPTORS - 1].next == ch->list->desc_busaddr);
  CHECK(hpm_dma_free(ch) == 0 && live_allocations == 0);
  for (i = 0; i < 3; i++) CHECK(hpm_dma_free(held[i]) == 0);
  CHECK(hpm_dma_alloc(&ch) == 0);
  configs[0] = config();
  configs[0].direction = HPM_DMA_MEM_TO_PERIPH;
  configs[0].dst_addr = HPM_GPIO0_BASE + 0x100;
  configs[0].dst_addrmode = HPM_DMA_ADDR_FIXED;
  configs[0].request = HPM_DMA_SRC_GPTMR0_0;
  CHECK(hpm_dma_setup_repeat(ch, &configs[0]) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  CHECK(mux[ch->channel] == HPM_DMAMUX_MUXCFG_ENABLE);
  CHECK(getreg32(HPM_DMA_CH_DSTADDR(ch->channel)) == HPM_GPIO0_BASE + 0x100);
  CHECK(hpm_dma_stop(ch) == 0);
  configs[0] = config();
  configs[0].src_addr = HPM_ROM_BASE;
  CHECK(hpm_dma_setup(ch, &configs[0]) == 0);
  configs[0].src_addr = HPM_XPI0_MEMORY_BASE;
  CHECK(hpm_dma_setup(ch, &configs[0]) == 0);
  configs[0].src_addr = HPM_XPI0_MEMORY_BASE + HPM_XPI0_MEMORY_SIZE - 32;
  CHECK(hpm_dma_setup(ch, &configs[0]) == -EFAULT);
  configs[0] = config(); configs[0].dst_addr = HPM_XPI0_MEMORY_BASE;
  CHECK(hpm_dma_setup(ch, &configs[0]) == -EFAULT);
  configs[0].dst_addr = HPM_ROM_BASE;
  CHECK(hpm_dma_setup(ch, &configs[0]) == -EFAULT);
  configs[0] = config();
  alloc_fail_after = 0;
  CHECK(hpm_dma_setup_repeat(ch, &configs[0]) == 0);
  alloc_fail_after = -1;
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE, false);
  CHECK(last_events == HPM_DMA_EVENT_ERROR && ch->state == HPM_DMA_STATE_FAULT);
  CHECK(hpm_dma_stop(ch) == 0);
  CHECK(hpm_dma_free(ch) == 0);
}

static void mixed_chain_payload(void)
{
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c[3];
  unsigned char *src = (unsigned char *)(uintptr_t)HPM_DLM_BASE;
  unsigned char *dst = src + 0x2000;
  unsigned char expected[192];
  unsigned int i;
  int notified;
  CHECK(hpm_dma_alloc(&ch) == 0);
  for (i = 0; i < 192; i++) src[i] = (i * 17) ^ 0xa5;
  memset(dst, 0x3c, 224);
  for (i = 0; i < 3; i++)
    {
      c[i] = config(); c[i].src_addr += 64 * i; c[i].dst_addr += 64 * i;
    }
  c[0].dst_width = HPM_DMA_WIDTH_8BIT;
  memcpy(expected, src, 64);
  c[1].src_width = c[1].dst_width = HPM_DMA_WIDTH_16BIT;
  c[1].src_addrmode = HPM_DMA_ADDR_DECREMENT;
  c[1].src_addr += 62;
  for (i = 0; i < 32; i++) memcpy(expected + 64 + i * 2, src + 126 - i * 2, 2);
  c[2].src_addrmode = HPM_DMA_ADDR_FIXED;
  c[2].dst_width = HPM_DMA_WIDTH_8BIT;
  for (i = 0; i < 64; i++) expected[128 + i] = src[128 + i % 4];
  CHECK(hpm_dma_setup_chain(ch, c, 3, false) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  notified = notifications;
  engine_step(ch, true); engine_step(ch, true); engine_step(ch, true);
  CHECK(notifications == notified + 1 && last_events == HPM_DMA_EVENT_COMPLETE);
  CHECK(memcmp(dst, expected, 192) == 0 && dst[192] == 0x3c);
  CHECK(hpm_dma_setup_chain(ch, c, 3, false) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  engine_step(ch, true);
  regs[HPM_DMA_IDMISC_OFFSET / 4] = (5U << HPM_DMA_IDMISC_DMASTATE_SHIFT) |
                                  (ch->channel << HPM_DMA_IDMISC_CURCHAN_SHIFT);
  drain_after = 3;
  notified = notifications;
  CHECK(hpm_dma_stop(ch) == 0 && notifications == notified);
  CHECK(ch->list != NULL && live_allocations == 2);
  CHECK(hpm_dma_free(ch) == 0 && live_allocations == 0);
}

static void irq_only(void)
{
  CHECK(!in_irq);
  in_irq = true;
  CHECK(hpm_dma_interrupt(HPM_IRQ_HDMA, NULL, NULL) == 0);
  in_irq = false;
}

static void deferred_and_status(void)
{
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c = config();
  struct hpm_dma_status_s status;
  uint64_t generation;
  unsigned int reads;
  unsigned int writes;
  int notified;
  int invalidated;
  uint32_t bit;
  CHECK(hpm_dma_alloc(&ch) == 0);
  CHECK(hpm_dma_rearm(ch) == -EINVAL);
  CHECK(hpm_dma_setup(ch, &c) == 0);
#ifndef CONFIG_ARCH_DCACHE
  mock_cache_enabled = true;
  CHECK(hpm_dma_start(ch, callback, &notifications) == -ENOTSUP);
  CHECK(!hpm_dma_enabled(ch) && ch->state == HPM_DMA_STATE_CONFIGURED);
  mock_cache_enabled = false;
#endif
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  CHECK(hpm_dma_getstatus(ch, &status) == 0);
  generation = status.generation;
  CHECK(status.state == HPM_DMA_STATE_RUNNING && status.position_valid &&
        status.remaining_bytes == 64 && status.current_segment == 0);
  notified = notifications; invalidated = cache_calls[2];
  bit = HPM_DMA_CHANNEL_BIT(ch->channel);
  regs[HPM_DMA_CHEN_OFFSET / 4] &= ~bit;
  regs[HPM_DMA_INTTCSTS_OFFSET / 4] |= bit;
  regs[HPM_DMA_IDMISC_OFFSET / 4] = (4U << HPM_DMA_IDMISC_DMASTATE_SHIFT) |
                                   (ch->channel << HPM_DMA_IDMISC_CURCHAN_SHIFT);
  reads = mmio_reads; writes = mmio_writes;
  irq_only();
  CHECK(mmio_reads - reads <= 8 && mmio_writes - writes <= 2);
  CHECK(ch->work_active && notifications == notified && cache_calls[2] == invalidated);
  run_work();
  CHECK(ch->state == HPM_DMA_STATE_COMPLETING && ch->work_active);
  CHECK(notifications == notified && cache_calls[2] == invalidated);
  CHECK(hpm_dma_free(ch) == -EBUSY && hpm_dma_rearm(ch) == -EBUSY);
  drain_after = 2;
  nxsig_usleep(1000); nxsig_usleep(1000);
  CHECK(ch->state == HPM_DMA_STATE_ALLOCATED && !ch->work_active);
  CHECK(notifications == notified + 1 && last_events == HPM_DMA_EVENT_COMPLETE);
  CHECK(hpm_dma_rearm(ch) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  CHECK(ch->generation > generation);
  block_worker = true;
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE, false);
  CHECK(ch->state == HPM_DMA_STATE_COMPLETING);
  interrupt_channel(ch, HPM_DMA_EVENT_ERROR | HPM_DMA_EVENT_ABORT, false);
  block_worker = false; run_work();
  CHECK(last_events == HPM_DMA_EVENT_ERROR && ch->state == HPM_DMA_STATE_FAULT);
  CHECK(hpm_dma_getstatus(ch, &status) == 0 && status.fault.valid);
  CHECK(status.result == -EIO && status.fault.src == c.src_addr);
  CHECK(hpm_dma_rearm(ch) == -EBUSY);
  CHECK(hpm_dma_stop(ch) == 0);
  CHECK(hpm_dma_rearm(ch) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  queue_failure = true;
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE, false);
  queue_failure = false;
  CHECK(hpm_dma_getstatus(ch, &status) == 0 && status.state == HPM_DMA_STATE_FAULT);
  CHECK(status.queue_errors == 1 && status.result == -EIO && !status.work_pending);
  CHECK(hpm_dma_free(ch) == -EBUSY && hpm_dma_stop(ch) == 0);
  CHECK(hpm_dma_rearm(ch) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  block_worker = true;
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE, false);
  notified = notifications;
  CHECK(hpm_dma_stop(ch) == -ETIMEDOUT);
  CHECK(ch->work_active && hpm_dma_free(ch) == -EBUSY);
  block_worker = false; run_work();
  CHECK(!ch->work_active && notifications == notified);
  CHECK(hpm_dma_stop(ch) == 0 && hpm_dma_free(ch) == 0);
}

static void rearm_and_buffers(void)
{
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c[2];
  struct hpm_dma_desc_s *desc;
  void *buffer;
  size_t allocated;
  unsigned int i;
  unsigned int j;
  CHECK(hpm_dma_alloc(&ch) == 0);
  c[0] = config(); c[1] = config();
  c[1].src_addr += 64; c[1].dst_addr += 64;
  CHECK(hpm_dma_setup_chain(ch, c, 2, false) == 0);
  desc = ch->list->desc;
  alloc_fail_after = 0;
  for (i = 0; i < 5; i++)
    {
      CHECK(hpm_dma_rearm(ch) == 0 && ch->list->desc == desc);
      CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
      for (j = 0; j < 2; j++) engine_step(ch, true);
      CHECK(last_events == HPM_DMA_EVENT_COMPLETE && live_allocations == 2);
    }
  CHECK(alloc_fail_after == 0);
  alloc_fail_after = -1;
  CHECK(hpm_dma_free(ch) == 0 && live_allocations == 0);
  CHECK(hpm_dma_buffer_alloc(71, &buffer, &allocated) == 0);
  CHECK(allocated >= 71 && (allocated & 31) == 0 && ((uintptr_t)buffer & 31) == 0);
  memset(buffer, 0x7c, allocated);
  CHECK(hpm_dma_alloc(&ch) == 0);
  c[0] = config(); c[0].src_width = c[0].dst_width = HPM_DMA_WIDTH_8BIT;
  c[0].nbytes = 71; c[0].dst_addr = (uintptr_t)buffer;
  c[0].dst_cache_exclusive = true;
  CHECK(hpm_dma_setup(ch, &c[0]) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  engine_step(ch, true);
  CHECK(((unsigned char *)buffer)[71] == 0x7c);
#ifdef CONFIG_ARCH_DCACHE
  CHECK(last_cache_first[2] == (uintptr_t)buffer);
  CHECK(last_cache_end[2] == (uintptr_t)buffer + allocated);
#endif
  CHECK(hpm_dma_free(ch) == 0);
  kmm_free(buffer);
  CHECK(hpm_dma_buffer_alloc(SIZE_MAX, &buffer, &allocated) == -EOVERFLOW);
  CHECK(buffer == NULL && allocated == 0);
  CHECK(live_allocations == 0);
}

static void streams(void)
{
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c[3];
  struct hpm_dma_period_s first;
  struct hpm_dma_period_s second;
  struct hpm_dma_period_s third;
  struct hpm_dma_period_s newer;
  struct hpm_dma_period_s empty;
  unsigned int i;
  int notified;
  int invalidated;
  CHECK(hpm_dma_alloc(&ch) == 0);
  for (i = 0; i < 3; i++)
    {
      c[i] = config(); c[i].src_addr += 64 * i; c[i].dst_addr += 64 * i;
    }
  CHECK(hpm_dma_setup_stream(ch, c, 3) == 0 && ch->list->desc == NULL);
  CHECK(live_allocations == 1);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  CHECK(hpm_dma_acquire_period(ch, &empty) == -EAGAIN);
  engine_step(ch, true);
  CHECK(last_events == HPM_DMA_EVENT_PERIOD && ch->stream_current == 1);
  CHECK(hpm_dma_acquire_period(ch, &first) == 0 && first.index == 0 && first.sequence == 1);
  memset((void *)first.config.src_addr, 0xb3, first.config.nbytes);
  memset((void *)first.config.dst_addr, 0xe9, first.config.nbytes);
  engine_step(ch, true);
  CHECK(hpm_dma_acquire_period(ch, &second) == 0 && second.index == 1 && second.sequence == 2);
  engine_step(ch, true);
  CHECK(ch->state == HPM_DMA_STATE_WAITING && !hpm_dma_enabled(ch));
  CHECK(last_events == (HPM_DMA_EVENT_PERIOD | HPM_DMA_EVENT_STARVED));
  CHECK(((unsigned char *)first.config.dst_addr)[0] == 0xe9);
  CHECK(hpm_dma_acquire_period(ch, &third) == 0 && third.index == 2 && third.sequence == 3);
  notified = notifications;
  CHECK(hpm_dma_release_period(ch, &second) == 0);
  run_work();
  CHECK(ch->state == HPM_DMA_STATE_WAITING);
  CHECK(hpm_dma_release_period(ch, &first) == 0);
  run_work();
  CHECK(ch->state == HPM_DMA_STATE_RUNNING && ch->stream_current == 0);
  CHECK(notifications == notified);
  CHECK(hpm_dma_release_period(ch, &first) == -ESTALE);
  engine_step(ch, true);
  CHECK(((unsigned char *)first.config.dst_addr)[0] == 0xb3);
  CHECK(hpm_dma_acquire_period(ch, &newer) == 0 && newer.index == 0 && newer.sequence == 4);
  CHECK(hpm_dma_release_period(ch, &first) == -ESTALE);
  CHECK(hpm_dma_release_period(ch, &newer) == 0);
  invalidated = cache_calls[2];
  CHECK(hpm_dma_stop(ch) == 0);
#ifdef CONFIG_ARCH_DCACHE
  CHECK(cache_calls[2] == invalidated + 6); /* CPU period 2 must not be invalidated. */
#else
  (void)invalidated;
#endif
  CHECK(hpm_dma_rearm(ch) == -EBUSY && hpm_dma_free(ch) == -EBUSY);
  CHECK(hpm_dma_release_period(ch, &third) == 0);
  CHECK(hpm_dma_rearm(ch) == 0 && hpm_dma_start(ch, callback, &notifications) == 0);
  CHECK(hpm_dma_release_period(ch, &newer) == -ESTALE);
  engine_step(ch, true);
  CHECK(hpm_dma_acquire_period(ch, &newer) == 0 && newer.sequence == 1);
  CHECK(hpm_dma_stop(ch) == 0 && hpm_dma_free(ch) == -EBUSY);
  CHECK(hpm_dma_release_period(ch, &newer) == 0);
  CHECK(hpm_dma_free(ch) == 0 && live_allocations == 0);
}

#ifdef CONFIG_ARCH_DCACHE
static hpm_dma_handle_t release_fault_channel;
static void fault_during_release(void)
{
  interrupt_channel(release_fault_channel, HPM_DMA_EVENT_ERROR, false);
}
#endif

static void stream_validation_and_faults(void)
{
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c[2];
  struct hpm_dma_period_s period;
  CHECK(hpm_dma_alloc(&ch) == 0);
  c[0] = config(); c[1] = config();
  CHECK(hpm_dma_setup_stream(ch, c, 2) == -EINVAL);
  c[1].src_addr += 64; c[1].dst_addr = HPM_DLM_ALIAS_BASE + 0x2000;
  CHECK(hpm_dma_setup_stream(ch, c, 2) == -EINVAL);
  c[1].dst_addr = HPM_DLM_BASE + 0x2040;
  c[0].nbytes = c[1].nbytes = 32;
  c[0].dst_cache_exclusive = c[1].dst_cache_exclusive = true;
  c[1].src_addr = HPM_DLM_BASE + 32;
#ifdef CONFIG_ARCH_DCACHE
  CHECK(hpm_dma_setup_stream(ch, c, 2) == (MOCK_CACHE_LINE_SIZE == 64 ? -EINVAL : 0));
#else
  CHECK(hpm_dma_setup_stream(ch, c, 2) == 0);
#endif
  c[0] = config(); c[1] = config(); c[1].src_addr += 64; c[1].dst_addr += 64;
  c[1].half_interrupt = true;
  CHECK(hpm_dma_setup_stream(ch, c, 2) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  engine_step(ch, true);
  CHECK(hpm_dma_acquire_period(ch, &period) == 0 && period.index == 0);
#ifdef CONFIG_ARCH_DCACHE
  release_fault_channel = ch;
  cache_hook = fault_during_release;
  CHECK(hpm_dma_release_period(ch, &period) == -EIO);
  CHECK(ch->state == HPM_DMA_STATE_FAULT && !ch->work_active);
#else
  interrupt_channel(ch, HPM_DMA_EVENT_ERROR, false);
  CHECK(hpm_dma_release_period(ch, &period) == 0);
#endif
  CHECK(hpm_dma_stop(ch) == 0 && hpm_dma_free(ch) == 0);
  CHECK(live_allocations == 0);
}

static void route_pool(void)
{
  hpm_dma_handle_t channels[32];
  struct hpm_dma_config_s c[33];
  struct hpm_dma_config_s old;
  unsigned int i;
  uint32_t used = 0;
  for (i = 0; i < 32; i++)
    {
      CHECK(hpm_dma_alloc(&channels[i]) == 0);
      c[0] = config(); c[0].direction = HPM_DMA_MEM_TO_PERIPH;
      c[0].dst_addr = HPM_GPIO0_BASE + 0x100;
      c[0].dst_addrmode = HPM_DMA_ADDR_FIXED;
      c[0].request = HPM_DMA_SRC_GPTMR0_0;
      CHECK(hpm_dma_setup(channels[i], &c[0]) == 0);
      CHECK((used & channels[i]->routes.mask) == 0);
      used |= channels[i]->routes.mask;
    }
  CHECK(used == UINT32_MAX && g_dma_ports == UINT32_MAX);
  old = channels[0]->transfer.config;
  c[1] = c[0]; c[1].request = HPM_DMA_SRC_GPTMR0_1;
  CHECK(hpm_dma_setup_chain(channels[0], c, 2, false) == -EBUSY);
  CHECK(g_dma_ports == UINT32_MAX && channels[0]->list == NULL);
  CHECK(channels[0]->transfer.config.request == old.request);
  CHECK(hpm_dma_free(channels[31]) == 0);
  CHECK(hpm_dma_setup_chain(channels[0], c, 2, false) == 0);
  CHECK(channels[0]->routes.mask == (1U | (1U << 31)));
  CHECK(hpm_dma_start(channels[0], callback, &notifications) == 0);
  CHECK(mux[0] == HPM_DMAMUX_MUXCFG_ENABLE &&
        mux[31] == (HPM_DMAMUX_MUXCFG_ENABLE | HPM_DMA_SRC_GPTMR0_1));
  CHECK(hpm_dma_alloc(&channels[31]) == 0);
  old = config();
  CHECK(hpm_dma_setup(channels[31], &old) == 0);
  CHECK(hpm_dma_setup(channels[31], &c[0]) == -EBUSY);
  CHECK(mux[31] == (HPM_DMAMUX_MUXCFG_ENABLE | HPM_DMA_SRC_GPTMR0_1));
  CHECK(hpm_dma_stop(channels[0]) == 0);
  for (i = 0; i < 32; i++) CHECK(hpm_dma_free(channels[i]) == 0);
  CHECK(g_dma_ports == 0);
  for (i = 0; i < 32; i++) CHECK(mux[i] == 0);
  CHECK(hpm_dma_alloc(&channels[0]) == 0);
  for (i = 0; i < 33; i++)
    {
      c[i] = c[0]; c[i].request = i < 16 ? i : i + 4;
    }
  CHECK(hpm_dma_setup_chain(channels[0], c, 33, false) == -ENOSPC);
  CHECK(g_dma_ports == 0 && live_allocations == 0);
  CHECK(hpm_dma_free(channels[0]) == 0);
}

static void cache_plan_regressions(void)
{
#ifdef CONFIG_ARCH_DCACHE
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c[8];
  struct hpm_dma_status_s status;
  unsigned int i;
  int before;
  CHECK(hpm_dma_alloc(&ch) == 0);
  for (i = 0; i < 8; i++) c[i] = config();
  CHECK(hpm_dma_setup_chain(ch, c, 8, false) == 0);
  CHECK(ch->list->nsources == 1 && ch->list->ndestinations == 1 &&
        ch->list->ninvalidations == 1);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  invalidated_dlm_views = 0;
  before = cache_calls[2];
  for (i = 0; i < 8; i++) engine_step(ch, true);
  CHECK(cache_calls[2] == before + 3 && invalidated_dlm_views == 7);
  CHECK(hpm_dma_getstatus(ch, &status) == 0 && !status.remaining_valid);
  CHECK(hpm_dma_free(ch) == 0);
  CHECK(hpm_dma_alloc(&ch) == 0);
  c[0] = config(); c[1] = config();
  c[0].src_addr = HPM_DLM_BASE + HPM_DLM_SIZE - 64;
  c[1].src_addr = HPM_DLM_ALIAS_BASE;
  CHECK(hpm_dma_setup_chain(ch, c, 2, false) == 0);
  CHECK(ch->list->nsources == 2); /* Adjacent addresses, different physical windows. */
  CHECK(hpm_dma_free(ch) == 0);
#endif
}

#ifdef CONFIG_PM
static void power_management(void)
{
  hpm_dma_handle_t ch;
  CHECK(registered_pm == &g_dma_pm);
  CHECK(hpm_dma_alloc(&ch) == 0);
  CHECK(registered_pm->prepare(registered_pm, 0, PM_STANDBY) == -EBUSY);
  CHECK(hpm_dma_free(ch) == 0);
  CHECK(registered_pm->prepare(registered_pm, 0, PM_STANDBY) == 0);
  CHECK(hpm_dma_alloc(&ch) == -EAGAIN && ch == NULL);
  registered_pm->notify(registered_pm, 0, PM_NORMAL);
  CHECK(g_dma_restore_needed);
  init_failure = 1;
  CHECK(hpm_dma_alloc(&ch) == -EIO && ch == NULL && !g_dma_allocating);
  init_failure = 0;
  CHECK(hpm_dma_alloc(&ch) == 0 && !g_dma_restore_needed);
  CHECK(hpm_dma_free(ch) == 0);
  CHECK(registered_pm->prepare(registered_pm, 0, PM_STANDBY) == 0);
  CHECK(registered_pm->prepare(registered_pm, 0, PM_NORMAL) == 0);
  CHECK(hpm_dma_alloc(&ch) == -EAGAIN);
  registered_pm->notify(registered_pm, 0, PM_NORMAL);
  CHECK(hpm_dma_alloc(&ch) == 0 && hpm_dma_free(ch) == 0);
  g_dma_allocating = true;
  CHECK(registered_pm->prepare(registered_pm, 0, PM_STANDBY) == -EBUSY);
  g_dma_allocating = false;
}
#endif

static void callback_handoff_and_wait(void)
{
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c = config();
  clock_t before;
  int notified;
  unsigned int i;
  CHECK(hpm_dma_alloc(&ch) == 0);
  CHECK(hpm_dma_setup_stream(ch, &c, 1) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  return_from_callback = true;
  notified = notifications;
  for (i = 0; i < 5; i++)
    {
      engine_step(ch, true);
      CHECK(callback_release_result == 0 && ch->loans == 0 && hpm_dma_enabled(ch));
    }
  CHECK(notifications == notified + 5 && ch->completed_periods == 5);
  ch->lock.locked = true; /* Another task owns the control mutex. */
  engine_step(ch, true);
  ch->lock.locked = false;
  CHECK(callback_release_result == -EAGAIN && ch->loans == 1);
  return_from_callback = false;
  CHECK(hpm_dma_stop(ch) == 0 && hpm_dma_free(ch) == -EBUSY);
  CHECK(hpm_dma_release_period(ch, &callback_period) == 0);
  CHECK(hpm_dma_free(ch) == 0);
  CHECK(hpm_dma_alloc(&ch) == 0);
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  block_worker = true;
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE, false);
  block_worker = false;
  before = ticks;
  CHECK(hpm_dma_stop(ch) == 0 && ticks == before); /* No polling sleep. */
  CHECK(hpm_dma_free(ch) == 0);
}

static void software_and_p2p(void)
{
  hpm_dma_handle_t ch;
  struct hpm_dma_config_s c = config();
  uint32_t modes;
  CHECK(hpm_dma_alloc(&ch) == 0);
  c.direction = HPM_DMA_MEM_TO_PERIPH;
  c.dst_addr = HPM_CRC_BASE;
  c.dst_addrmode = HPM_DMA_ADDR_FIXED;
  c.software_trigger = true;
  c.request = UINT32_MAX; /* Explicitly ignored in software mode. */
  CHECK(hpm_dma_setup(ch, &c) == 0 && ch->routes.mask == 0);
  modes = HPM_DMA_CH_CTRL_SRCMODE | HPM_DMA_CH_CTRL_DSTMODE;
  CHECK((ch->transfer.ctrl & modes) == 0);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE, false);
  CHECK(last_events == HPM_DMA_EVENT_COMPLETE);
  c.direction = HPM_DMA_PERIPH_TO_PERIPH;
  c.src_addr = HPM_SPI0_BASE + 0x20;
  c.src_addrmode = HPM_DMA_ADDR_FIXED;
  c.software_trigger = false;
  c.request = HPM_DMA_SRC_SPI0_RX;
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK((ch->transfer.ctrl & modes) == HPM_DMA_CH_CTRL_SRCMODE);
  c.destination_request = true;
  c.request = HPM_DMA_SRC_DAC0;
  c.dst_addr = HPM_DAC0_BASE;
  CHECK(hpm_dma_setup(ch, &c) == 0);
  CHECK((ch->transfer.ctrl & modes) == HPM_DMA_CH_CTRL_DSTMODE);
  CHECK(hpm_dma_start(ch, callback, &notifications) == 0);
  interrupt_channel(ch, HPM_DMA_EVENT_COMPLETE, false);
  CHECK(last_events == HPM_DMA_EVENT_COMPLETE);
  c.software_trigger = true;
  c.request = UINT32_MAX;
  CHECK(hpm_dma_setup(ch, &c) == 0 && ch->routes.mask == 0);
  CHECK((ch->transfer.ctrl & modes) == 0);
  CHECK(hpm_dma_free(ch) == 0 && g_dma_ports == 0);
}

int main(int argc, char **argv)
{
  hpm_dma_handle_t ch = NULL;
  CHECK(mmap((void *)(uintptr_t)HPM_DLM_BASE, HPM_DLM_SIZE,
             PROT_READ | PROT_WRITE,
             MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0) ==
        (void *)(uintptr_t)HPM_DLM_BASE);
  if (argc > 1) init_failure = atoi(argv[1]);
  CHECK(hpm_dma_alloc(&ch) == -ENODEV && ch == NULL);
  riscv_dma_initialize();
  if (init_failure)
    {
      CHECK(!g_dma_initialized && hpm_dma_alloc(&ch) == -ENODEV);
      printf("initialization failure %d: %d checks passed\n", init_failure, checks);
      return 0;
    }
  CHECK(g_dma_initialized && irq_calls == 1);
  riscv_dma_initialize();
  CHECK(irq_calls == 1);
  create_failure = true;
  CHECK(hpm_dma_alloc(&ch) == -ENOMEM && ch == NULL);
  create_failure = false;
  validation(); lifecycle(); peripherals(); concurrent_channels();
  chains(); cyclic_and_repeat(); chain_edges(); mixed_chain_payload();
  software_and_p2p(); deferred_and_status(); rearm_and_buffers(); streams(); callback_handoff_and_wait();
  stream_validation_and_faults(); route_pool(); cache_plan_regressions();
#ifdef CONFIG_PM
  power_management();
#endif
  CHECK(live_allocations == 0 && g_dma_ports == 0);
#ifdef CONFIG_ARCH_DCACHE
  CHECK(cache_calls[0] > 0 && cache_calls[1] > 0 && cache_calls[2] > 0);
#endif
  CHECK(lock_depth == 0);
  printf("DMA model: %d checks passed\n", checks);
  return 0;
}
"""


def main():
    driver = ROOT / "arch/risc-v/src/hpm5300/hpm_dma.c"
    with tempfile.TemporaryDirectory(prefix="hpm5300-dma-") as directory:
        tmp = Path(directory)
        (tmp / "mock.h").write_text(MOCK)
        for header in (
            "nuttx/config.h", "nuttx/arch.h", "nuttx/cache.h",
            "nuttx/clock.h", "nuttx/irq.h", "nuttx/signal.h",
            "nuttx/mutex.h", "nuttx/spinlock.h", "arch/barriers.h",
            "riscv_internal.h", "debug.h", "nuttx/compiler.h",
            "nuttx/kmalloc.h", "nuttx/sched.h", "nuttx/wqueue.h",
            "nuttx/semaphore.h",
            "nuttx/power/pm.h",
        ):
            path = tmp / header
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#include "mock.h"\n')
        (tmp / "test.c").write_text(TEST.replace("DRIVER_PATH", str(driver)))
        for cached, line_size, pm in ((False, 0, False), (True, 32, False),
                                      (True, 64, False), (True, 32, True)):
            output = tmp / ("cached" if cached else "uncached")
            command = shlex.split(os.environ.get("CC", "cc")) + [
                "-std=c11", "-D_GNU_SOURCE=1", "-Wall", "-Wextra", "-Werror",
                "-Wno-unused-parameter", "-I", str(tmp),
            ]
            if cached:
                command += ["-DCONFIG_ARCH_DCACHE=1",
                            f"-DMOCK_CACHE_LINE_SIZE={line_size}"]
            if pm:
                command.append("-DCONFIG_PM=1")
            command += [str(tmp / "test.c"), "-o", str(output)]
            subprocess.run(command, check=True)
            print((f"Cached ({line_size} bytes), PM={pm}:" if cached else
                   "Uncached:"), flush=True)
            for failure in (0, 1, 2, 3):
                subprocess.run([str(output), str(failure)], check=True)


if __name__ == "__main__":
    main()
