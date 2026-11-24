/****************************************************************************
 * arch/risc-v/src/hpm5300/hpm_dma.h
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

#ifndef __ARCH_RISCV_SRC_HPM5300_HPM_DMA_H
#define __ARCH_RISCV_SRC_HPM5300_HPM_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Client-visible event bits, independent of hardware register bit positions.
 * A notification can contain multiple events. Half-transfer is progress;
 * complete, error and abort are terminal events for a one-shot transfer. A
 * terminal event containing error or abort must not be treated as success.
 */

#define HPM_DMA_EVENT_COMPLETE    (1U << 0)
#define HPM_DMA_EVENT_HALF        (1U << 1)
#define HPM_DMA_EVENT_ERROR       (1U << 2)
#define HPM_DMA_EVENT_ABORT       (1U << 3)
#define HPM_DMA_EVENT_PERIOD      (1U << 4)
#define HPM_DMA_EVENT_STARVED     (1U << 5)

#define HPM_DMA_EVENT_TERMINAL \
  (HPM_DMA_EVENT_COMPLETE | HPM_DMA_EVENT_ERROR | HPM_DMA_EVENT_ABORT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Opaque channel handle. The channel object is owned by the DMA driver;
 * clients must not dereference or free it. A handle is valid only while the
 * client owns the allocated channel.
 */

struct hpm_dmach_s;

typedef struct hpm_dmach_s *hpm_dma_handle_t;

enum hpm_dma_state_e
{
  HPM_DMA_STATE_FREE = 0,
  HPM_DMA_STATE_ALLOCATED,
  HPM_DMA_STATE_CONFIGURED,
  HPM_DMA_STATE_RUNNING,
  HPM_DMA_STATE_STOPPING,
  HPM_DMA_STATE_FAULT,
  HPM_DMA_STATE_COMPLETING,
  HPM_DMA_STATE_WAITING
};

enum hpm_dma_mode_e
{
  HPM_DMA_MODE_SINGLE = 0,
  HPM_DMA_MODE_CHAIN,
  HPM_DMA_MODE_CYCLIC,
  HPM_DMA_MODE_REPEAT,
  HPM_DMA_MODE_STREAM
};

struct hpm_dma_fault_s
{
  bool valid;
  int result;
  uint32_t events;
  uint64_t generation;
  uint32_t ctrl;
  uint32_t remaining;
  uintptr_t src;
  uintptr_t dst;
  uintptr_t next;
};

struct hpm_dma_status_s
{
  enum hpm_dma_state_e state;
  enum hpm_dma_mode_e mode;
  uint64_t generation;
  uint32_t pending_events;
  uint32_t last_events;
  int result;
  uint64_t observed_half;
  uint64_t observed_periods;
  uint64_t completed_periods;
  size_t cpu_owned_periods;
  size_t ready_periods;
  uint32_t stop_timeouts;
  uint32_t queue_errors;
  size_t segment_count;
  size_t current_segment;
  size_t stream_next;
  size_t remaining_bytes;
  uint32_t ctrl;
  uint32_t remaining_units;
  uintptr_t src;
  uintptr_t dst;
  uintptr_t next;
  bool hardware_enabled;
  bool work_pending;
  bool callback_active;
  bool configuration_valid;
  bool position_valid;
  bool remaining_valid;
  uint32_t request_ports;
  struct hpm_dma_fault_s fault;
};

/* Transfer direction determines which endpoint uses a peripheral request.
 * Memory-to-memory transfers use software triggering. This hardware does not
 * permit both source and destination handshake modes simultaneously.
 */

enum hpm_dma_direction_e
{
  HPM_DMA_MEM_TO_MEM = 0,
  HPM_DMA_PERIPH_TO_MEM,
  HPM_DMA_MEM_TO_PERIPH,
  HPM_DMA_PERIPH_TO_PERIPH
};

/* Transfer width encodings: 0, 1 and 2 represent 1, 2 and 4 bytes per
 * transfer respectively. Source and destination widths are independent;
 * addresses and lengths must satisfy the required alignment.
 */

enum hpm_dma_width_e
{
  HPM_DMA_WIDTH_8BIT = 0,
  HPM_DMA_WIDTH_16BIT,
  HPM_DMA_WIDTH_32BIT
};

/* Address behavior is independent of transfer direction. Fixed addressing is
 * useful for peripheral registers and memory fill sources alike.
 */

enum hpm_dma_addrmode_e
{
  HPM_DMA_ADDR_INCREMENT = 0,
  HPM_DMA_ADDR_DECREMENT,
  HPM_DMA_ADDR_FIXED
};

/* Zero-initialize this configuration before filling one contiguous transfer.
 *
 * src_addr/dst_addr are CPU-visible addresses: memory-buffer addresses for
 * memory endpoints and MMIO addresses for peripheral endpoints. The driver
 * resolves DMA bus addresses and validates accessibility and alignment.
 * nbytes is the total byte count, not the hardware TRANSIZE field value.
 *
 * request is an HPM_DMA_SRC_* peripheral source ID, ignored for memory-to-
 * memory transfers. It is not a DMA channel or DMAMUX output-slot number.
 * Source ID zero is valid for peripheral transfers.
 *
 * burst is a source transfer count: any integer 1..16, or 32, 64 or 128.
 * The driver selects linear or power-of-two encoding as needed. It is not
 * a byte count or an encoded SRCBURSTSIZE field.
 * handshake_block selects one hardware request per complete block when true,
 * or per burst when false; it is ignored for memory-to-memory use.
 * high_priority selects the higher hardware priority level. half_interrupt
 * requests notification at the half-transfer point, in addition to the
 * terminal events managed by the driver.
 *
 * The client must keep buffers valid and must not access transferred memory
 * while DMA owns it. Width combinations, lengths and endpoint modes are
 * validated by setup. Memory sources accept on-chip RAM, ROM and the XPI
 * memory aperture; destinations accept on-chip RAM. Clients must configure
 * XPI and respect the attached device's actual capacity and availability.
 * Overlapping source/destination physical spans within one transfer,
 * including different RAM aliases, are rejected.
 *
 * Destinations occupy complete cache lines unless dst_cache_exclusive
 * explicitly grants the containing padding too. Dirty conflicting aliases
 * must be synchronized before start. Destination invalidation covers every
 * documented CPU alias. No CPU access is permitted to device-owned buffers;
 * STREAM CPU loans are the explicit exception.
 */

struct hpm_dma_config_s
{
  enum hpm_dma_direction_e direction;

  uintptr_t src_addr;
  uintptr_t dst_addr;
  size_t nbytes;

  enum hpm_dma_width_e src_width;
  enum hpm_dma_width_e dst_width;

  enum hpm_dma_addrmode_e src_addrmode;
  enum hpm_dma_addrmode_e dst_addrmode;

  unsigned int request;
  unsigned int burst;

  /* Peripheral endpoints may also use normal/software triggering. Request
   * and handshake_block are ignored then. P2P supports only one handshake:
   * destination_request chooses its side; the other peripheral must keep up.
   */

  bool software_trigger;
  bool destination_request;
  bool handshake_block;
  bool high_priority;
  bool half_interrupt;

  /* Promise exclusive ownership of the entire containing destination cache
   * lines, including padding. Allows an unaligned byte length; transfer
   * width alignment still applies. False requires whole-line destinations.
   */

  bool dst_cache_exclusive;
};

/* A stream loan is valid until successful release. Its config is a copied
 * description, not a way to change DMA addresses. Stop does not revoke CPU
 * loans; they must be returned before setup, rearm or free can replace them.
 */

struct hpm_dma_period_s
{
  uint64_t generation;
  uint64_t sequence;
  size_t index;
  struct hpm_dma_config_s config;
};

/* Notifications run on a DMA-private high-priority worker, never in the
 * hardware ISR. Cache maintenance and final bus draining precede terminal
 * notification. Callbacks must be short and nonblocking. Calling a mutating
 * DMA API from this worker returns -EDEADLK; getstatus and acquire_period
 * are
 * nonblocking exceptions. Queue application work or wake a separate task.
 *
 * HALF and raw CYCLIC/REPEAT PERIOD are advisory, can coalesce, have no
 * exact
 * descriptor count and release no buffers. STREAM PERIOD makes one completed
 * period available through acquire_period. STARVED indicates that a stream
 * paused at a CPU-owned/not-yet-returned period, without overwriting it.
 * ERROR/ABORT requires successful stop before reclaiming device-owned data.
 * Callback context remains valid until terminal notification has returned or
 * stop succeeds. A start can notify before its caller returns.
 */

typedef void (*hpm_dma_callback_t)(hpm_dma_handle_t handle,
                                   uint32_t events, void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: hpm_dma_alloc
 *
 * Description:
 *   Allocate an unused channel in task context. This does not wait for a
 *   busy channel to be released. The first allocation lazily creates a
 *   private completion worker and can fail with -ENOMEM. PM transitions can
 *   return -EAGAIN; restoration can return a hardware error. On failure, a
 *   non-NULL output pointer is set to NULL. The returned channel is
 *   exclusively owned by the caller.
 *
 * Returned Value:
 *   Zero on success, -EBUSY if no channel is available, or another negated
 *   errno on failure.
 *
 ****************************************************************************/

int hpm_dma_alloc(hpm_dma_handle_t *handle);

/****************************************************************************
 * Name: hpm_dma_setup
 *
 * Description:
 *   Validate and prepare one transfer without starting it. Called in task
 *   context. Copy the configuration; do not retain the caller's config
 *   pointer. The buffers themselves must remain valid until DMA finishes.
 *   Reject reconfiguration of an active channel.
 *
 * Returned Value:
 *   Zero on success or a negated errno on failure.
 *
 ****************************************************************************/

int hpm_dma_setup(hpm_dma_handle_t handle,
                  const struct hpm_dma_config_s *config);

/****************************************************************************
 * Name: hpm_dma_setup_chain
 *
 * Description:
 *   Copy and validate count transfers and construct an owned descriptor list
 *   in task context. count must be 1..CONFIG_HPM_HDMA_MAX_DESCRIPTORS. Every
 *   entry uses the same half_interrupt policy: sticky HALF status has no
 *   descriptor identity. Directions, requests, widths, addresses, lengths,
 *   burst size and priority may differ. Unique peripheral requests reserve
 *   DMAMUX ports transactionally; -EBUSY preserves the previous setup when
 *   those ports are busy. More than 32 distinct sources returns -ENOSPC.
 *   The config array is copied, not retained.
 *
 *   With cyclic false, only the final entry reports COMPLETE. With cyclic
 *   true, the last descriptor points to the first and TC reports PERIOD,
 *   continuing until stop or fault. HALF/PERIOD do not release buffers or
 *   identify a descriptor. Error/abort requires confirmed stop in all modes.
 *   A failed setup leaves the preceding configuration unchanged. Descriptors
 *   are retained until a successful replacement or channel free, even after
 *   completion/stop. All data buffers must remain valid while DMA owns them.
 *
 * Returned Value:
 *   Zero on success; -EINVAL for incompatible entries, -E2BIG for excess
 *   count, -ENOMEM for allocation failure, or another negated errno.
 *
 ****************************************************************************/

int hpm_dma_setup_chain(hpm_dma_handle_t handle,
                        const struct hpm_dma_config_s *configs,
                        size_t count, bool cyclic);

/****************************************************************************
 * Name: hpm_dma_setup_repeat
 *
 * Description:
 *   Prepare native INFINITELOOP repetition of one transfer in task context.
 *   Hardware reloads its initial addresses and length after each block.
 *   LLPOINTER remains zero: hardware forbids combining this mode with a
 *   descriptor chain. TC reports nonterminal PERIOD; stop is required before
 *   buffers can be reclaimed. No descriptor allocation is needed.
 *
 * Returned Value:
 *   Zero on success or a negated errno on failure.
 *
 ****************************************************************************/

int hpm_dma_setup_repeat(hpm_dma_handle_t handle,
                         const struct hpm_dma_config_s *config);

/****************************************************************************
 * Name: hpm_dma_setup_stream
 *
 * Description:
 *   Copy 1..CONFIG_HPM_HDMA_MAX_DESCRIPTORS independent periods. Their RAM
 *   and containing cache-line coverage must be disjoint, including aliases.
 *   Directions, requests and HALF policy may differ at every period. Only
 *   software metadata is allocated; hardware uses finite blocks and can
 *   reuse one reserved request port at quiescent period boundaries.
 *
 *   The worker launches the next queued block before handing the completed
 *   buffer to the CPU. If the next buffer has not been returned, DMA pauses
 *   in WAITING and reports STARVED, without overwriting a CPU-owned buffer.
 *   There is a software scheduling gap: peripherals require FIFO headroom,
 *   request pacing or flow control. Raw CYCLIC is the autonomous hardware
 *   ring; it does not provide this CPU ownership guarantee.
 *
 ****************************************************************************/

int hpm_dma_setup_stream(hpm_dma_handle_t handle,
                         const struct hpm_dma_config_s *configs,
                         size_t count);

/****************************************************************************
 * Name: hpm_dma_rearm
 *
 * Description:
 *   Reuse immutable validated configuration, ports, descriptors and cache
 *   plans without allocating or rebuilding. Buffers must still be valid.
 *   Stop must first recover FAULT, and all CPU loans must have been
 *   returned. It can wait for a terminal callback to return, using the stop
 *   timeout. A new run starts at entry zero, never an ambiguous aborted bus
 *   position.
 *
 ****************************************************************************/

int hpm_dma_rearm(hpm_dma_handle_t handle);

/****************************************************************************
 * Name: hpm_dma_getstatus
 *
 * Description:
 *   IRQ/callback-safe software state, live registers and first-fault data.
 *   Hardware fields are sequential advisory reads, not an atomic position or
 *   ownership guarantee. remaining_valid requires enabled hardware;
 *   position_valid only means the sampled link belongs to this setup.
 *   observed_periods counts observed TC bits, not raw-ring laps. STREAM
 *   completed_periods counts successful READY commits, not canceled blocks.
 *   Generation numbers distinguish runs; queue/stop timeout counters persist
 *   across allocations. The first-fault record survives stop until start.
 *
 ****************************************************************************/

int hpm_dma_getstatus(hpm_dma_handle_t handle,
                      struct hpm_dma_status_s *status);

/****************************************************************************
 * Name: hpm_dma_acquire_period
 *
 * Description:
 *   Borrow the oldest READY stream period without blocking; -EAGAIN if none.
 *   This is IRQ/callback-safe and returns a copied generation/sequence
 *   token. CPU may use that period's RAM until release. Stop leaves CPU
 *   loans valid and preserves their data. Free, rearm and reconfiguration
 *   reject active loans; return them even after stop/fault before replacing
 *   the setup.
 *
 ****************************************************************************/

int hpm_dma_acquire_period(hpm_dma_handle_t handle,
                           struct hpm_dma_period_s *period);

/****************************************************************************
 * Name: hpm_dma_release_period
 *
 * Description:
 *   Task-only return of a CPU loan. Running streams prepare caches before
 *   requeueing it; inactive/faulted streams merely return ownership to the
 *   channel owner without invalidating CPU-modified data. Duplicate/stale
 *   tokens return -ESTALE. Calls from the DMA callback use try-lock,
 *   returning -EAGAIN with the loan unchanged on contention. On queue/fault
 *   error after cache preparation, use stop before reusing the buffer. Never
 *   retain CPU access to a successfully returned period.
 *
 ****************************************************************************/

int hpm_dma_release_period(hpm_dma_handle_t handle,
                           const struct hpm_dma_period_s *period);

/****************************************************************************
 * Name: hpm_dma_buffer_alloc
 *
 * Description:
 *   Allocate cache-aligned on-chip RAM in task context. allocation reports
 *   padded storage size, not a different DMA byte count. dst_cache_exclusive
 *   permits partial-line transfers when this whole allocation is owned. Use
 *   kmm_free only after the CPU owns it and DMA cannot reuse it.
 *
 ****************************************************************************/

int hpm_dma_buffer_alloc(size_t nbytes, void **buffer, size_t *allocation);

/****************************************************************************
 * Name: hpm_dma_start
 *
 * Description:
 *   Start a prepared transfer asynchronously in task context. A non-NULL
 *   callback is required. Prepare its context before calling: an interrupt
 *   can notify completion before this function returns. Each run needs setup
 *   or rearm. Task APIs require a sleepable context with interrupts enabled.
 *   A cache-disabled build rejects a mismatched enabled hardware D-cache.
 *
 * Returned Value:
 *   Zero when started, or a negated errno if the transfer was not started.
 *   Later transfer events are reported through the callback.
 *
 ****************************************************************************/

int hpm_dma_start(hpm_dma_handle_t handle,
                  hpm_dma_callback_t callback, void *arg);

/****************************************************************************
 * Name: hpm_dma_stop
 *
 * Description:
 *   Cancel a transfer in task context, suppress further client callbacks for
 *   it and wait for hardware to become quiescent with a bounded wait. Do not
 *   synthesize a client callback from task context. Success confirms
 *   device-owned buffers/context can be reclaimed. CPU loans remain valid
 *   until released; they prevent rearm/setup/free. Setup or rearm is needed
 *   before restart. On failure, cancellation is unconfirmed: retain the
 *   channel and buffers.
 *
 * Returned Value:
 *   Zero on success or a negated errno, including -ETIMEDOUT, on failure.
 *
 ****************************************************************************/

int hpm_dma_stop(hpm_dma_handle_t handle);

/****************************************************************************
 * Name: hpm_dma_free
 *
 * Description:
 *   Release an inactive, quiescent channel in task context. Reject release
 *   while a transfer, queued work, callback or CPU loan is active. A failed
 *   stop must not be treated as permission to release a channel. The handle
 *   is invalid after success.
 *
 * Returned Value:
 *   Zero on success, -EBUSY if still in use, or another negated errno.
 *
 ****************************************************************************/

int hpm_dma_free(hpm_dma_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPM5300_HPM_DMA_H */
