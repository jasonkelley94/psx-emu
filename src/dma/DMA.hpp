#pragma once

#include <array>
#include <cstdio>
#include "common/Types.hpp"
#include "mem/Ram.hpp"
#include "gpu/GPU.hpp"
#include "irq/IRQ.hpp"
#include "cdrom/CDRom.hpp"

// ── DMA Controller ────────────────────────────────────────────────────────────
// The PSX DMA has 7 channels, each with three 32-bit registers, plus two
// global registers (DPCR, DICR).  All DMA registers live in the I/O window:
//
//   Base = 0x1F80_1080  (offset 0x080 from IO_BASE)
//
//   Per-channel layout  (channel N starts at base + N*0x10)
//   ──────────────────────────────────────────────────────────────────────────
//   +0x0  D_MADR  — Base address (word-aligned, physical)
//   +0x4  D_BCR   — Block control
//            Mode 0 (burst):       [15:0]  = word count (0 → 0x10000)
//            Mode 1 (slice/sync):  [15:0]  = words/block  [31:16] = #blocks
//            Mode 2 (linked list): ignored
//   +0x8  D_CHCR  — Channel control
//            Bit  0      Transfer direction  0=RAM→dev, 1=dev→RAM
//            Bit  1      Address step        0=+4, 1=-4
//            Bit  8      Chopping enable
//            Bits[10:9]  Sync mode           0=burst, 1=slice, 2=linked-list
//            Bits[18:16] Chopping DMA window size (2^N words)
//            Bits[22:20] Chopping CPU window size (2^N cycles)
//            Bit 24      Start/Busy          1=start, cleared on completion
//            Bit 28      Start trigger       required for mode 0
//
//   Global registers
//   ──────────────────────────────────────────────────────────────────────────
//   +0xF0  D_DPCR  — DMA Priority Control
//            Each channel occupies 4 bits: [3:0]=ch0, [7:4]=ch1, …
//            Bit 3 of each nibble = master enable for that channel
//   +0xF4  D_DICR  — DMA Interrupt Control
//            Bits[14:0]  Force/misc (bit 15 = force-IRQ)
//            Bits[22:16] Per-channel IRQ enable (ch0=bit16, …, ch6=bit22)
//            Bit  23     IRQ master enable
//            Bits[30:24] Per-channel IRQ flags (ch0=bit24, …) — write-1-to-clear
//            Bit  31     IRQ master flag (read-only, set by hardware)
//
// Channel assignments
// ──────────────────────────────────────────────────────────────────────────
//   0  MDECin   1  MDECout   2  GPU   3  CD-ROM
//   4  SPU      5  PIO (EXT) 6  OTC (Ordering Table Clear)
//
// Implemented channels
//   ch2 GPU  — linked-list mode (mode 2, CPU→GPU GP0)
//            — block mode       (mode 0/1, CPU→GPU GP0)
//   ch6 OTC  — burst mode       fills reverse-linked list in RAM
// ──────────────────────────────────────────────────────────────────────────────
class DMA {
public:
    DMA(Ram& ram, GPU& gpu, IRQ& irq, CDRom& cdrom) noexcept
        : ram_(ram), gpu_(gpu), irq_(irq), cdrom_(cdrom) {}

    // ── Bus interface (offset from DMA base = IO_BASE + 0x080) ───────────────
    [[nodiscard]] u32  read (u32 off) const noexcept;
    void               write(u32 off, u32 value) noexcept;

private:
    // ── Per-channel state ─────────────────────────────────────────────────────
    struct Channel {
        u32 madr = 0;
        u32 bcr  = 0;
        u32 chcr = 0;
    };
    std::array<Channel, 7> ch_{};

    // ── Global registers ──────────────────────────────────────────────────────
    // DPCR default enables all channels with ascending priority (ch0 lowest).
    u32 dpcr_ = 0x07654321u;
    u32 dicr_ = 0u;

    // ── References — DMA does not own these ───────────────────────────────────
    Ram&   ram_;
    GPU&   gpu_;
    IRQ&   irq_;
    CDRom& cdrom_;

    // ── Helpers ───────────────────────────────────────────────────────────────
    [[nodiscard]] bool channel_enabled(u32 n) const noexcept {
        return (dpcr_ >> (n * 4 + 3)) & 1u;
    }

    void start_transfer(u32 n) noexcept;

    // Channel-specific transfer implementations
    void run_otc       (u32 n) noexcept;  // ch6: reverse ordering-table clear
    void run_gpu_ll    (u32 n) noexcept;  // ch2, mode 2: linked-list to GP0
    void run_gpu_block (u32 n) noexcept;  // ch2, mode 0/1: block to GP0
    void run_vram_read (u32 n) noexcept;  // ch2, mode 0/1: GPU→RAM VRAM readback
    void run_cdrom     (u32 n) noexcept;  // ch3: CD-ROM sector data → RAM

    // Called after any transfer completes — updates CHCR, DICR, and IRQ.
    void finish(u32 n) noexcept;
    void update_irq() noexcept;
};
