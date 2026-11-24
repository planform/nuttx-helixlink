/****************************************************************************
 * arch/risc-v/src/hpm5300/hardware/hpm5361/hpm5361_dma.h
 *
 * Copyright (c) 2021-2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * HPM5361 HDMA uses the DMA V2 register layout. Register definitions are
 * adapted from the HPM5300 SDK hpm_dmav2_regs.h and HPM5361 SVD.
 * See HPM5300UM Rev1.2, chapter 28, for behavior and access semantics.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_DMA_H
#define __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hpm5361_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Controller-wide register offsets, relative to HPM_HDMA_BASE. */

/* Controller capacity. Maximum burst is in source transfers, not bytes. */

#define HPM_DMA_NCHANNELS           32
#define HPM_DMA_MAX_BURST           128U

#define HPM_DMA_IDMISC_OFFSET       0x0004
#define HPM_DMA_DMACFG_OFFSET       0x0010
#define HPM_DMA_DMACTRL_OFFSET      0x0014
#define HPM_DMA_CHABORT_OFFSET      0x0018
#define HPM_DMA_INTHALFSTS_OFFSET   0x0024
#define HPM_DMA_INTTCSTS_OFFSET     0x0028
#define HPM_DMA_INTABORTSTS_OFFSET  0x002c
#define HPM_DMA_INTERRSTS_OFFSET    0x0030
#define HPM_DMA_CHEN_OFFSET         0x0034

/* Channel register offsets. Channel indices are 0 through 31. */

#define HPM_DMA_CH_BASE_OFFSET      0x0040
#define HPM_DMA_CH_STRIDE           0x0020

#define HPM_DMA_CH_OFFSET(n) \
  (HPM_DMA_CH_BASE_OFFSET + HPM_DMA_CH_STRIDE * (n))

#define HPM_DMA_CH_CTRL_OFFSET(n) \
  (HPM_DMA_CH_OFFSET(n) + 0x0000)
#define HPM_DMA_CH_TRANSIZE_OFFSET(n) \
  (HPM_DMA_CH_OFFSET(n) + 0x0004)
#define HPM_DMA_CH_SRCADDR_OFFSET(n) \
  (HPM_DMA_CH_OFFSET(n) + 0x0008)
#define HPM_DMA_CH_CHANREQCTRL_OFFSET(n) \
  (HPM_DMA_CH_OFFSET(n) + 0x000c)
#define HPM_DMA_CH_DSTADDR_OFFSET(n) \
  (HPM_DMA_CH_OFFSET(n) + 0x0010)
#define HPM_DMA_CH_LLPOINTER_OFFSET(n) \
  (HPM_DMA_CH_OFFSET(n) + 0x0018)

/* Register addresses. */

#define HPM_DMA_IDMISC \
  (HPM_HDMA_BASE + HPM_DMA_IDMISC_OFFSET)
#define HPM_DMA_DMACFG \
  (HPM_HDMA_BASE + HPM_DMA_DMACFG_OFFSET)
#define HPM_DMA_DMACTRL \
  (HPM_HDMA_BASE + HPM_DMA_DMACTRL_OFFSET)
#define HPM_DMA_CHABORT \
  (HPM_HDMA_BASE + HPM_DMA_CHABORT_OFFSET)
#define HPM_DMA_INTHALFSTS \
  (HPM_HDMA_BASE + HPM_DMA_INTHALFSTS_OFFSET)
#define HPM_DMA_INTTCSTS \
  (HPM_HDMA_BASE + HPM_DMA_INTTCSTS_OFFSET)
#define HPM_DMA_INTABORTSTS \
  (HPM_HDMA_BASE + HPM_DMA_INTABORTSTS_OFFSET)
#define HPM_DMA_INTERRSTS \
  (HPM_HDMA_BASE + HPM_DMA_INTERRSTS_OFFSET)
#define HPM_DMA_CHEN \
  (HPM_HDMA_BASE + HPM_DMA_CHEN_OFFSET)

#define HPM_DMA_CH_CTRL(n) \
  (HPM_HDMA_BASE + HPM_DMA_CH_CTRL_OFFSET(n))
#define HPM_DMA_CH_TRANSIZE(n) \
  (HPM_HDMA_BASE + HPM_DMA_CH_TRANSIZE_OFFSET(n))
#define HPM_DMA_CH_SRCADDR(n) \
  (HPM_HDMA_BASE + HPM_DMA_CH_SRCADDR_OFFSET(n))
#define HPM_DMA_CH_CHANREQCTRL(n) \
  (HPM_HDMA_BASE + HPM_DMA_CH_CHANREQCTRL_OFFSET(n))
#define HPM_DMA_CH_DSTADDR(n) \
  (HPM_HDMA_BASE + HPM_DMA_CH_DSTADDR_OFFSET(n))
#define HPM_DMA_CH_LLPOINTER(n) \
  (HPM_HDMA_BASE + HPM_DMA_CH_LLPOINTER_OFFSET(n))

/* Register bit definitions. W1C and write-only registers must not be
 * modified using read-modify-write operations.
 */

/* IDMISC: read-only controller state and current channel. */

#define HPM_DMA_IDMISC_DMASTATE_SHIFT   13
#define HPM_DMA_IDMISC_DMASTATE_MASK \
  (7U << HPM_DMA_IDMISC_DMASTATE_SHIFT)
#define HPM_DMA_IDMISC_DMASTATE_GET(v) \
  (((v) & HPM_DMA_IDMISC_DMASTATE_MASK) >> \
   HPM_DMA_IDMISC_DMASTATE_SHIFT)

#define HPM_DMA_IDMISC_CURCHAN_SHIFT    8
#define HPM_DMA_IDMISC_CURCHAN_MASK \
  (31U << HPM_DMA_IDMISC_CURCHAN_SHIFT)
#define HPM_DMA_IDMISC_CURCHAN_GET(v) \
  (((v) & HPM_DMA_IDMISC_CURCHAN_MASK) >> \
   HPM_DMA_IDMISC_CURCHAN_SHIFT)

/* DMACFG: read-only hardware configuration fields. Field values are raw
 * encodings; they must not be assumed to be literal resource counts.
 */

#define HPM_DMA_DMACFG_CHAINXFR         (1U << 31)
#define HPM_DMA_DMACFG_REQSYNC          (1U << 30)

#define HPM_DMA_DMACFG_DATAWIDTH_SHIFT  24
#define HPM_DMA_DMACFG_DATAWIDTH_MASK \
  (3U << HPM_DMA_DMACFG_DATAWIDTH_SHIFT)
#define HPM_DMA_DMACFG_DATAWIDTH_GET(v) \
  (((v) & HPM_DMA_DMACFG_DATAWIDTH_MASK) >> \
   HPM_DMA_DMACFG_DATAWIDTH_SHIFT)

#define HPM_DMA_DMACFG_ADDRWIDTH_SHIFT  17
#define HPM_DMA_DMACFG_ADDRWIDTH_MASK \
  (127U << HPM_DMA_DMACFG_ADDRWIDTH_SHIFT)
#define HPM_DMA_DMACFG_ADDRWIDTH_GET(v) \
  (((v) & HPM_DMA_DMACFG_ADDRWIDTH_MASK) >> \
   HPM_DMA_DMACFG_ADDRWIDTH_SHIFT)

#define HPM_DMA_DMACFG_CORENUM_SHIFT     16
#define HPM_DMA_DMACFG_CORENUM_MASK \
  (1U << HPM_DMA_DMACFG_CORENUM_SHIFT)
#define HPM_DMA_DMACFG_CORENUM_GET(v) \
  (((v) & HPM_DMA_DMACFG_CORENUM_MASK) >> \
   HPM_DMA_DMACFG_CORENUM_SHIFT)

#define HPM_DMA_DMACFG_BUSNUM_SHIFT      15
#define HPM_DMA_DMACFG_BUSNUM_MASK \
  (1U << HPM_DMA_DMACFG_BUSNUM_SHIFT)
#define HPM_DMA_DMACFG_BUSNUM_GET(v) \
  (((v) & HPM_DMA_DMACFG_BUSNUM_MASK) >> \
   HPM_DMA_DMACFG_BUSNUM_SHIFT)

#define HPM_DMA_DMACFG_REQNUM_SHIFT      10
#define HPM_DMA_DMACFG_REQNUM_MASK \
  (31U << HPM_DMA_DMACFG_REQNUM_SHIFT)
#define HPM_DMA_DMACFG_REQNUM_GET(v) \
  (((v) & HPM_DMA_DMACFG_REQNUM_MASK) >> \
   HPM_DMA_DMACFG_REQNUM_SHIFT)

#define HPM_DMA_DMACFG_FIFODEPTH_SHIFT   4
#define HPM_DMA_DMACFG_FIFODEPTH_MASK \
  (63U << HPM_DMA_DMACFG_FIFODEPTH_SHIFT)
#define HPM_DMA_DMACFG_FIFODEPTH_GET(v) \
  (((v) & HPM_DMA_DMACFG_FIFODEPTH_MASK) >> \
   HPM_DMA_DMACFG_FIFODEPTH_SHIFT)

#define HPM_DMA_DMACFG_CHANNELNUM_SHIFT  0
#define HPM_DMA_DMACFG_CHANNELNUM_MASK \
  (15U << HPM_DMA_DMACFG_CHANNELNUM_SHIFT)
#define HPM_DMA_DMACFG_CHANNELNUM_GET(v) \
  (((v) & HPM_DMA_DMACFG_CHANNELNUM_MASK) >> \
   HPM_DMA_DMACFG_CHANNELNUM_SHIFT)

/* DMACTRL: controller-wide reset. This affects all channels and must not
 * be used to stop an individual client's transfer.
 */

#define HPM_DMA_DMACTRL_RESET             (1U << 0)

/* Channel bitmaps. The caller must validate the channel before shifting. */

#define HPM_DMA_CHANNEL_BIT(n)            (1U << (n))
#define HPM_DMA_ALL_CHANNELS              0xffffffffU

/* CHCTRL[n].CTRL: channel configuration. */

/* Channel options: infinite loop, handshake granularity, priority and
 * burst-size interpretation.
 */

#define HPM_DMA_CH_CTRL_INFINITELOOP    (1U << 31)
#define HPM_DMA_CH_CTRL_HANDSHAKEOPT    (1U << 30)
#define HPM_DMA_CH_CTRL_PRIORITY        (1U << 29)
#define HPM_DMA_CH_CTRL_BURSTOPT        (1U << 28)

/* Source burst size: encoded value, not a literal transfer count. */

#define HPM_DMA_CH_CTRL_SRCBURSTSIZE_SHIFT  24
#define HPM_DMA_CH_CTRL_SRCBURSTSIZE_MASK \
  (15U << HPM_DMA_CH_CTRL_SRCBURSTSIZE_SHIFT)
#define HPM_DMA_CH_CTRL_SRCBURSTSIZE_SET(v) \
  (((v) << HPM_DMA_CH_CTRL_SRCBURSTSIZE_SHIFT) & \
   HPM_DMA_CH_CTRL_SRCBURSTSIZE_MASK)
#define HPM_DMA_CH_CTRL_SRCBURSTSIZE_GET(v) \
  (((v) & HPM_DMA_CH_CTRL_SRCBURSTSIZE_MASK) >> \
   HPM_DMA_CH_CTRL_SRCBURSTSIZE_SHIFT)

/* Source and destination transfer widths: encoded values. Validate them
 * against this chip's supported widths before configuring a channel.
 */

#define HPM_DMA_CH_CTRL_SRCWIDTH_SHIFT  21
#define HPM_DMA_CH_CTRL_SRCWIDTH_MASK \
  (7U << HPM_DMA_CH_CTRL_SRCWIDTH_SHIFT)
#define HPM_DMA_CH_CTRL_SRCWIDTH_SET(v) \
  (((v) << HPM_DMA_CH_CTRL_SRCWIDTH_SHIFT) & \
   HPM_DMA_CH_CTRL_SRCWIDTH_MASK)
#define HPM_DMA_CH_CTRL_SRCWIDTH_GET(v) \
  (((v) & HPM_DMA_CH_CTRL_SRCWIDTH_MASK) >> \
   HPM_DMA_CH_CTRL_SRCWIDTH_SHIFT)

#define HPM_DMA_CH_CTRL_DSTWIDTH_SHIFT  18
#define HPM_DMA_CH_CTRL_DSTWIDTH_MASK \
  (7U << HPM_DMA_CH_CTRL_DSTWIDTH_SHIFT)
#define HPM_DMA_CH_CTRL_DSTWIDTH_SET(v) \
  (((v) << HPM_DMA_CH_CTRL_DSTWIDTH_SHIFT) & \
   HPM_DMA_CH_CTRL_DSTWIDTH_MASK)
#define HPM_DMA_CH_CTRL_DSTWIDTH_GET(v) \
  (((v) & HPM_DMA_CH_CTRL_DSTWIDTH_MASK) >> \
   HPM_DMA_CH_CTRL_DSTWIDTH_SHIFT)

/* Source/destination handshake modes. They must not both be enabled for
 * the same channel.
 */

#define HPM_DMA_CH_CTRL_SRCMODE          (1U << 17)
#define HPM_DMA_CH_CTRL_DSTMODE          (1U << 16)

/* Source/destination address control: increment, decrement or fixed.
 * Encoding 3 is reserved and must be rejected by channel configuration.
 */

#define HPM_DMA_CH_CTRL_SRCADDRCTRL_SHIFT  14
#define HPM_DMA_CH_CTRL_SRCADDRCTRL_MASK \
  (3U << HPM_DMA_CH_CTRL_SRCADDRCTRL_SHIFT)
#define HPM_DMA_CH_CTRL_SRCADDRCTRL_SET(v) \
  (((v) << HPM_DMA_CH_CTRL_SRCADDRCTRL_SHIFT) & \
   HPM_DMA_CH_CTRL_SRCADDRCTRL_MASK)
#define HPM_DMA_CH_CTRL_SRCADDRCTRL_GET(v) \
  (((v) & HPM_DMA_CH_CTRL_SRCADDRCTRL_MASK) >> \
   HPM_DMA_CH_CTRL_SRCADDRCTRL_SHIFT)

#define HPM_DMA_CH_CTRL_DSTADDRCTRL_SHIFT  12
#define HPM_DMA_CH_CTRL_DSTADDRCTRL_MASK \
  (3U << HPM_DMA_CH_CTRL_DSTADDRCTRL_SHIFT)
#define HPM_DMA_CH_CTRL_DSTADDRCTRL_SET(v) \
  (((v) << HPM_DMA_CH_CTRL_DSTADDRCTRL_SHIFT) & \
   HPM_DMA_CH_CTRL_DSTADDRCTRL_MASK)
#define HPM_DMA_CH_CTRL_DSTADDRCTRL_GET(v) \
  (((v) & HPM_DMA_CH_CTRL_DSTADDRCTRL_MASK) >> \
   HPM_DMA_CH_CTRL_DSTADDRCTRL_SHIFT)

/* Channel interrupt masks: one disables the interrupt, zero permits it. */

#define HPM_DMA_CH_CTRL_INTHALFCNTMASK    (1U << 4)
#define HPM_DMA_CH_CTRL_INTABTMASK        (1U << 3)
#define HPM_DMA_CH_CTRL_INTERRMASK        (1U << 2)
#define HPM_DMA_CH_CTRL_INTTCMASK         (1U << 1)

/* Channel enable. In handshake mode, a hardware request starts movement
 * after the channel is enabled; normal mode starts through software enable.
 */

#define HPM_DMA_CH_CTRL_ENABLE            (1U << 0)

/* CHCTRL[n].TRANSIZE: source transfer count, not a byte count or a
 * count-minus-one encoding. Zero is invalid for an enabled channel. Setup
 * routine must validate range and alignment before constructing the field.
 */

#define HPM_DMA_CH_TRANSIZE_MASK          0x0fffffffU

#define HPM_DMA_CH_TRANSIZE_SET(v) \
  ((v) & HPM_DMA_CH_TRANSIZE_MASK)

#define HPM_DMA_CH_TRANSIZE_GET(v) \
  ((v) & HPM_DMA_CH_TRANSIZE_MASK)

/* CHCTRL[n].CHANREQCTRL: select source/destination handshake ports from
 * DMAMUX outputs. These selectors are not peripheral request-source IDs;
 * peripheral source selection belongs to DMAMUX.MUXCFG.SOURCE.
 */

#define HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_SHIFT  24
#define HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_MASK \
  (31U << HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_SHIFT)
#define HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_SET(v) \
  (((v) << HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_SHIFT) & \
   HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_MASK)
#define HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_GET(v) \
  (((v) & HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_MASK) >> \
   HPM_DMA_CH_CHANREQCTRL_SRCREQSEL_SHIFT)

#define HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_SHIFT  16
#define HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_MASK \
  (31U << HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_SHIFT)
#define HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_SET(v) \
  (((v) << HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_SHIFT) & \
   HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_MASK)
#define HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_GET(v) \
  (((v) & HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_MASK) >> \
   HPM_DMA_CH_CHANREQCTRL_DSTREQSEL_SHIFT)

/* CHCTRL[n].SRCADDR/DSTADDR: 32-bit DMA bus addresses. Endpoint alignment
 * depends on the respective transfer widths. CPU-local address
 * translation and DMA accessibility checks belong to the driver layer.
 */

#define HPM_DMA_CH_SRCADDR_MASK          0xffffffffU
#define HPM_DMA_CH_SRCADDR_SET(v) \
  ((v) & HPM_DMA_CH_SRCADDR_MASK)
#define HPM_DMA_CH_SRCADDR_GET(v) \
  ((v) & HPM_DMA_CH_SRCADDR_MASK)

#define HPM_DMA_CH_DSTADDR_MASK          0xffffffffU
#define HPM_DMA_CH_DSTADDR_SET(v) \
  ((v) & HPM_DMA_CH_DSTADDR_MASK)
#define HPM_DMA_CH_DSTADDR_GET(v) \
  ((v) & HPM_DMA_CH_DSTADDR_MASK)

/* CHCTRL[n].LLPOINTER: descriptor bus address in bits 31..5. The address
 * helper accepts a complete address, not an address shifted right by five.
 * Validate 32-byte alignment before encoding; masking does not validate or
 * repair a misaligned descriptor. Zero terminates a descriptor chain.
 */

#define HPM_DMA_DESCRIPTOR_ALIGN         32U

#define HPM_DMA_CH_LLPOINTER_ADDR_MASK   0xffffffe0U
#define HPM_DMA_CH_LLPOINTER_ADDR(a) \
  ((a) & HPM_DMA_CH_LLPOINTER_ADDR_MASK)

#endif /* __ARCH_RISCV_SRC_HPM5300_HARDWARE_HPM5361_HPM5361_DMA_H */
