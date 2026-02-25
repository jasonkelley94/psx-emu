#include "DMA.hpp"

// ── Register read ─────────────────────────────────────────────────────────────
u32 DMA::read(u32 off) const noexcept {
    // Global registers
    if (off == 0x070) return dpcr_;  // offset from DMA_BASE (= IO_BASE+0x080)
    if (off == 0x074) return dicr_;  // Note: called with off = phys - DMA_BASE

    // Per-channel registers: offset 0x000–0x06F → channel 0–6
    // Each channel is 0x10 bytes wide; only offsets +0, +4, +8 are used.
    if (off < 0x070) {
        const u32 n   = off / 0x10;
        const u32 reg = (off % 0x10) / 4;
        if (n < 7) {
            switch (reg) {
                case 0: return ch_[n].madr;
                case 1: return ch_[n].bcr;
                case 2:  return ch_[n].chcr;
                default: break;
            }
        }
    }

    std::fprintf(stderr, "[DMA] unhandled read  off=0x%03X\n", off);
    return 0xFFFF'FFFFu;
}

// ── Register write ────────────────────────────────────────────────────────────
void DMA::write(u32 off, u32 value) noexcept {
    // Global registers
    if (off == 0x070) { dpcr_ = value; return; }
    if (off == 0x074) {
        // Bits [30:24] are IRQ flags — write-1-to-clear.
        // All other bits are plain R/W (preserve forced bits, update enables).
        const u32 flags_mask = 0x7F00'0000u;
        dicr_ &= ~(value & flags_mask);   // clear acknowledged flags
        dicr_  = (dicr_ & flags_mask)     // keep un-cleared flags
               | (value & ~flags_mask);   // overwrite non-flag bits
        // Re-evaluate master flag after any change to DICR.
        update_irq();
        return;
    }

    if (off >= 0x070) {
        std::fprintf(stderr, "[DMA] unhandled write off=0x%03X val=0x%08X\n", off, value);
        return;
    }

    // Per-channel write
    const u32 n   = off / 0x10;
    const u32 reg = (off % 0x10) / 4;
    if (n >= 7) { return; }

    switch (reg) {
    case 0:  // MADR — low 2 bits always 0 (word-aligned)
        ch_[n].madr = value & 0x00FF'FFFFu;
        break;
    case 1:  // BCR
        ch_[n].bcr = value;
        break;
    case 2: {  // CHCR — writing bit 24 (and optionally bit 28) starts transfer
        // ── Channel 6 (OTC) has hardware-fixed bits ──────────────────────────
        // Bit 1 (step direction) is hardwired to 1 (OTC always counts down).
        // Bits 28 and 24 are only stored when bit 24 (busy/start) is set; a
        // write without bit 24 clears them.  All other bits are hardwired to 0
        // (or set by hardware on completion — see finish()).
        if (n == 6u) {
            // Bits 30 and 28 are independently writable (not gated by bit 24).
            // Bit 24 (busy/start) is also writable.  All other bits are
            // hardwired to 0 except bit 1 (step direction) which is fixed to 1.
            const u32 storable = value & 0x51000000u;  // bits 30, 28, 24
            ch_[6u].chcr = storable | 0x00000002u;
        } else {
            ch_[n].chcr = value;
        }

        const u32  sync    = (value >> 9) & 3u;
        const bool busy    = value & (1u << 24);
        const bool trigger = value & (1u << 28);

        // Mode 0 (burst) requires both the busy bit AND the trigger bit.
        // Modes 1 and 2 start on busy alone (device sync'd or linked-list).
        // Exception: GPU→RAM (ch2, dir=1) uses hardware DREQ from the GPU which
        // is implicitly asserted when armed for VRAM readback — no bit 28 needed.
        const bool gpu_dreq = (n == 2u) && (ch_[n].chcr & 1u);
        const bool should_run = busy && (sync != 0 || trigger || gpu_dreq);

        if (should_run && channel_enabled(n)) {
            start_transfer(n);
        }
        break;
    }
    default:
        std::fprintf(stderr, "[DMA] write to padding ch=%u reg=%u val=0x%08X\n",
                     n, reg, value);
        break;
    }
}

// ── Transfer dispatch ─────────────────────────────────────────────────────────
void DMA::start_transfer(u32 n) noexcept {
    const u32  sync = (ch_[n].chcr >> 9) & 3u;
    const u32  dir  = ch_[n].chcr & 1u;   // 0=RAM→device, 1=device→RAM

    switch (n) {
    case 0:  // MDEC-in  (CPU/RAM → MDEC): accept data, MDEC decodes nothing.
    case 1:  // MDEC-out (MDEC → RAM):     no decoded data; RAM already zeroed.
        // MDEC is not implemented; both channels complete immediately.
        // DMA ch1 destination RAM is zero-initialized, so the BIOS reads black
        // pixels for the boot animation and continues without hanging.
        finish(n);
        break;

    case 2:  // GPU
        if (sync == 2) {
            run_gpu_ll(n);       // linked-list, always CPU→GPU
        } else {
            if (dir == 0) {
                run_gpu_block(n);  // block/burst CPU→GPU GP0
            } else {
                run_vram_read(n);
            }
        }
        break;

    case 3:  // CD-ROM → RAM
        run_cdrom(n);
        break;

    case 6:  // OTC — ordering table clear (always CPU→RAM via counter logic)
        run_otc(n);
        break;

    default:
        // Unimplemented channel — mark complete immediately so the BIOS
        // doesn't hang waiting for the busy bit to clear.
        std::fprintf(stderr, "[DMA] ch%u not implemented (CHCR=0x%08X, BCR=0x%08X)\n",
                     n, ch_[n].chcr, ch_[n].bcr);
        finish(n);
        break;
    }
}

// ── ch6: OTC (Ordering Table Clear) ──────────────────────────────────────────
// Fills a region of RAM with a reverse singly-linked list.
//
// The "ordering table" is a structure used by the GPU display-list renderer:
//   - Each 32-bit word is a pointer to the previous (lower-address) entry.
//   - The last entry (lowest address) holds the end-of-list sentinel 0xFFFFFF.
//   - Rendering sweeps from the *last* entry (highest address) toward entry 0,
//     so objects at the end of the list appear furthest from the camera.
//
// MADR   = address of the HIGHEST word (table "tail", start of the sweep)
// BCR[15:0] = number of words to fill  (0 → 0x10000)
// CHCR   = 0x1100_0002  (dir=RAM, step=-4, mode=burst, start+trigger set)
//
void DMA::run_otc(u32 n) noexcept {
    u32 addr  = ch_[n].madr & 0x00FF'FFFFu;  // 24-bit physical, word-aligned
    u32 count = ch_[n].bcr  & 0xFFFFu;
    if (count == 0) count = 0x1'0000u;        // 0 is treated as max (65536)

    // Fill from MADR downward.  Each entry points to the next-lower entry.
    while (count > 1) {
        const u32 prev = (addr - 4u) & 0x00FF'FFFFu;
        ram_.write<u32>(addr & (Ram::SIZE - 1u), prev);
        addr = prev;
        --count;
    }
    // Terminal entry — the GPU stops traversal here.
    ram_.write<u32>(addr & (Ram::SIZE - 1u), 0x00FF'FFFFu);

    finish(n);
}

// ── ch2: GPU linked-list mode ─────────────────────────────────────────────────
// Walks a singly-linked list in RAM and feeds each packet's data words to
// the GPU GP0 port.
//
// Packet layout (each node in the list):
//   Word 0 (header):  [31:24] = N  (number of following data words)
//                     [23:0]  = address of the NEXT packet
//                               (0x00FF_FFFF = end of list)
//   Words 1..N:       GP0 drawing commands
//
void DMA::run_gpu_ll(u32 n) noexcept {
    u32 addr = ch_[n].madr & 0x00FF'FFFFu;  // 24-bit MADR, consistent with OTC

    // Guard against runaway lists (shouldn't happen with valid data).
    constexpr u32 MAX_ITER = 1'000'000u;
    u32 iter = 0;

    while (iter++ < MAX_ITER) {
        const u32 header  = ram_.read<u32>(addr & (Ram::SIZE - 1u));
        const u32 n_words = header >> 24;

        // Send data words to GPU GP0
        for (u32 i = 0; i < n_words; ++i) {
            addr = (addr + 4u) & 0x00FF'FFFFu;
            gpu_.write(0, ram_.read<u32>(addr & (Ram::SIZE - 1u)));
        }

        // End-of-list sentinel check (bits [23:0] == 0xFFFFFF)
        if ((header & 0x00FF'FFFFu) == 0x00FF'FFFFu) break;

        // Advance to next packet (next pointer is a 24-bit physical address)
        addr = header & 0x00FF'FFFFu;
    }

    if (iter >= MAX_ITER) {
        // Looped / runaway chain — leave the channel "busy" (bit 24 stays set)
        // and do NOT fire the IRQ.  On real hardware the DMA would spin forever
        // until the CPU clears the busy bit; the channel appears to never finish.
        std::fprintf(stderr, "[DMA] ch2 linked-list runaway at 0x%08X — leaving busy\n", addr);
        return;
    }

    finish(n);
}

// ── ch2: GPU block mode (CPU→GPU) ────────────────────────────────────────────
// Mode 0 (burst):  BCR[15:0]        = total word count
// Mode 1 (slice):  BCR[15:0] × BCR[31:16] = total word count
//
// Both modes stream words from RAM to GP0 sequentially.
// When chopping is enabled (CHCR bit 8), the transfer is split into DMA-window
// bursts separated by CPU-window pauses.  The CPU cycles yielded per pause are
// accumulated in cpu_credit_ so Bus can advance the timers accordingly.
//
// Chopping fields in CHCR:
//   [8]     chop enable
//   [18:16] DMA window size = 2^N words  (used for burst/sync-0 mode)
//   [22:20] CPU window size = 2^N cycles (credited between every burst)
//
// In slice mode (sync=1) the natural burst unit is the block size (BCR[15:0])
// because the DMA already pauses between blocks waiting for the device signal.
void DMA::run_gpu_block(u32 n) noexcept {
    u32 addr = ch_[n].madr & 0x00FF'FFFFu;  // 24-bit MADR, consistent with OTC
    const u32 chcr = ch_[n].chcr;
    const u32 sync = (chcr >> 9) & 3u;
    const bool chop = (chcr >> 8) & 1u;
    const s32 step = (chcr & 2u) ? -4 : 4;  // CHCR bit 1

    u32 total;
    u32 burst;   // words per DMA burst (for chopping)
    if (sync == 1) {
        // Slice mode: block-size × block-count
        const u32 bs = ch_[n].bcr & 0xFFFFu;
        const u32 ba = (ch_[n].bcr >> 16) & 0xFFFFu;
        total = bs * ba;
        // In slice mode each block is the natural chopping unit.
        burst = (bs > 0u) ? bs : 1u;
    } else {
        // Burst mode: raw word count
        total = ch_[n].bcr & 0xFFFFu;
        if (total == 0) total = 0x1'0000u;
        // Chopping DMA window from CHCR[18:16]; without chopping, run all at once.
        burst = chop ? (1u << ((chcr >> 16) & 7u)) : total;
    }

    const u32 cpu_window = chop ? (1u << ((chcr >> 20) & 7u)) : 0u;

    u32 remaining = total;
    while (remaining > 0) {
        const u32 n_words = (burst < remaining) ? burst : remaining;
        for (u32 i = 0; i < n_words; ++i) {
            gpu_.write(0, ram_.read<u32>(addr & (Ram::SIZE - 1u)));
            addr = static_cast<u32>(static_cast<s32>(addr) + step);
        }
        remaining -= n_words;
        // Credit CPU cycles between bursts (not after the last burst).
        if (chop && remaining > 0u) {
            cpu_credit_ += 6u + cpu_window;  // bus overhead + CPU yield window
        }
    }

    // Credit DMA transfer time (1 cycle/word) + slice-mode inter-block overhead.
    cpu_credit_ += total;
    if (sync == 1u && !chop) {
        const u32 bs = ch_[n].bcr & 0xFFFFu;  // words per block
        cpu_credit_ += (bs > 0u ? total / bs : 0u) * 10u;  // 10 cycles/block DREQ
    }

    finish(n);
}

// ── ch2: GPU VRAM readback (GPU→RAM) ─────────────────────────────────────────
// The GPU must have already processed a GP0(0xC0) VRAM-to-CPU command, which
// arms the GPU's VTCR readback cursor.  Each read of GPUREAD (GPU offset 0)
// returns two packed 16bpp pixels and advances the cursor.
//
// Mode 0 (burst):  BCR[15:0]        = word count
// Mode 1 (slice):  BCR[15:0] × BCR[31:16] = word count
//
// Timing credits: we credit the timer with the approximate sys-clock cycles the
// real DMA controller would consume, so that timer-based chopping measurements
// reflect realistic timing rather than zero (our DMA runs synchronously).
//
//   Slice mode (sync=1): 1 cycle/word (GPU output rate) +
//                        ~10 cycles/block (DREQ re-assertion turnaround)
//   Burst no-chop (sync=0, bit8=0): 1 cycle/word
//   Burst with chop (sync=0, bit8=1): 1 cycle/word +
//                        (6 + cpu_window) cycles/burst  (bus overhead + yield)
void DMA::run_vram_read(u32 n) noexcept {
    u32 addr = ch_[n].madr & 0x00FF'FFFFu;
    const u32 chcr = ch_[n].chcr;
    const u32 sync = (chcr >> 9u) & 3u;
    const bool chop = (chcr >> 8u) & 1u;

    u32 total;
    u32 bs = 0;   // block size (slice mode)
    u32 ba = 0;   // block count (slice mode)
    if (sync == 1u) {
        bs = ch_[n].bcr & 0xFFFFu;
        ba = (ch_[n].bcr >> 16u) & 0xFFFFu;
        total = bs * ba;
    } else {
        total = ch_[n].bcr & 0xFFFFu;
        if (total == 0u) total = 0x1'0000u;
    }

    for (u32 i = 0; i < total; ++i) {
        ram_.write<u32>(addr & (Ram::SIZE - 1u), gpu_.read(0u));
        addr = (addr + 4u) & 0x00FF'FFFFu;
    }

    // ── Timing credits ────────────────────────────────────────────────────────
    // Credit the DMA transfer time (1 cycle/word) always.
    cpu_credit_ += total;

    if (sync == 1u) {
        // Slice mode: ~10 cycles per block for DREQ re-assertion between blocks.
        const u32 n_blocks = (bs > 0u) ? ba : 0u;
        cpu_credit_ += n_blocks * 10u;
    } else if (chop) {
        // Burst mode with chopping: 6 cycles bus overhead + cpu_window per burst.
        const u32 burst_size  = 1u << ((chcr >> 16u) & 7u);
        const u32 cpu_window  = 1u << ((chcr >> 20u) & 7u);
        const u32 n_bursts    = (burst_size > 0u) ? (total / burst_size) : total;
        cpu_credit_ += n_bursts * (6u + cpu_window);
    }

    finish(n);
}

// ── Completion ────────────────────────────────────────────────────────────────
void DMA::finish(u32 n) noexcept {
    // Clear busy (bit 24) and trigger (bit 28) bits for all channels.
    // For OTC (ch6), also clear bit 30 which is writable and reset on completion.
    u32 clear_mask = (1u << 24) | (1u << 28);
    if (n == 6u) clear_mask |= (1u << 30);
    ch_[n].chcr &= ~clear_mask;

    // Raise per-channel IRQ flag if the channel's IRQ is enabled in DICR.
    if ((dicr_ >> (16u + n)) & 1u) {
        dicr_ |= (1u << (24u + n));
    }

    update_irq();
}

// ── ch3: CD-ROM sector data → RAM ────────────────────────────────────────────
// Mode 0 (burst): BCR[15:0] = word count (0 → 0x10000).
// Drains the CDRom sector buffer word-by-word into RAM at MADR.
void DMA::run_cdrom(u32 n) noexcept {
    u32 addr  = ch_[n].madr & 0x00FF'FFFFu;
    u32 total = ch_[n].bcr  & 0xFFFFu;
    if (total == 0u) total = 0x1'0000u;

    for (u32 i = 0u; i < total; ++i) {
        ram_.write<u32>(addr & (Ram::SIZE - 1u), cdrom_.read_data_word());
        addr = (addr + 4u) & 0x00FF'FFFFu;
    }

    finish(n);
}

// ── DICR master IRQ flag ──────────────────────────────────────────────────────
// Bit 31 is set (and I_STAT.DMA is raised) when:
//   • bit 15 (force) is set, OR
//   • bit 23 (master enable) is set AND any enabled channel's flag is set.
void DMA::update_irq() noexcept {
    const bool force      = (dicr_ >> 15) & 1u;
    const bool master_en  = (dicr_ >> 23) & 1u;
    const u32  ch_enables = (dicr_ >> 16) & 0x7Fu;
    const u32  ch_flags   = (dicr_ >> 24) & 0x7Fu;
    const bool channels   = master_en && ((ch_enables & ch_flags) != 0u);

    if (force || channels) {
        dicr_ |=  (1u << 31);
        irq_.set(IRQSource::DMA);
    } else {
        dicr_ &= ~(1u << 31);
    }
}
