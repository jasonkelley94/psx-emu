#pragma once

#include <memory>
#include <cstdio>
#include "common/Types.hpp"
#include "mem/Ram.hpp"
#include "mem/Bios.hpp"
#include "gpu/GPU.hpp"
#include "irq/IRQ.hpp"
#include "dma/DMA.hpp"
#include "cdrom/CDRom.hpp"
#include "timers/Timers.hpp"

// ── Bus ───────────────────────────────────────────────────────────────────────
// Central interconnect.  Owns all memory-mapped components and routes CPU
// reads/writes to the correct device by physical address.
//
// Address translation (virtual → physical) is performed here using the MIPS
// region-mask table defined in Types.hpp; the CPU always hands us virtual
// addresses, letting the Bus strip KSEG0/KSEG1 prefixes transparently.
//
// PSX physical memory map summary
// ──────────────────────────────────────────────────────────────────────────────
// 0x0000_0000 – 0x001F_FFFF   RAM (2 MiB), mirrored ×4 up to 0x007F_FFFF
// 0x1F00_0000 – 0x1F7F_FFFF   Expansion 1 (open-bus / memory card)
// 0x1F80_0000 – 0x1F80_03FF   Scratchpad (D-cache used as 1 KiB fast RAM)
// 0x1F80_1000 – 0x1F80_2FFF   Hardware I/O (GPU, SPU, CD-ROM, DMA, timers…)
// 0x1FC0_0000 – 0x1FC7_FFFF   BIOS ROM (512 KiB)
// 0xFFFE_0000                  Cache-control register (write-only)
// ──────────────────────────────────────────────────────────────────────────────
class Bus {
public:
    // Construct with a pre-loaded BIOS; all other components are default-init.
    explicit Bus(std::unique_ptr<Bios> bios);

    // ── Typed read ────────────────────────────────────────────────────────────
    // T must be u8, u16, or u32.
    template<std::unsigned_integral T>
    [[nodiscard]] T read(u32 vaddr) const;

    // ── Typed write ───────────────────────────────────────────────────────────
    template<std::unsigned_integral T>
    void write(u32 vaddr, T value);

    // ── IRQ query (used by the CPU before each instruction fetch) ─────────────
    [[nodiscard]] bool irq_pending() const noexcept { return irq_->irq_pending(); }

    // ── TTY output ────────────────────────────────────────────────────────────
    // Shared by the SIO1 byte-write path (io_write8) and the BIOS A-function
    // intercept (CPU::bios_a_call).  Flushes a [TTY] line on '\n' or overflow.
    void tty_putchar(char c) noexcept;

    // ── Timing tick — call once per CPU instruction ───────────────────────────
    // Advances root counters and fires VBlank IRQ at ~60 Hz.
    void tick(u32 cycles = 1) noexcept;

    // ── PS-EXE sideloader ─────────────────────────────────────────────────────
    // Load a bare-metal PS-EXE directly into RAM, bypassing the BIOS boot.
    // Returns initial {PC, GP, SP} for the caller to push into the CPU.
    struct PsxExeInfo {
        u32  pc = 0;
        u32  gp = 0;   // r28
        u32  sp = 0;   // 0 → caller should use default 0x801FFFF0
        bool ok = false;
    };
    [[nodiscard]] PsxExeInfo sideload(const char* path) noexcept;

    // Direct component access for the debugger / test harness
    [[nodiscard]] Ram&    ram()   noexcept { return *ram_; }
    [[nodiscard]] GPU&    gpu()   noexcept { return *gpu_; }
    [[nodiscard]] IRQ&    irq()   noexcept { return *irq_; }
    [[nodiscard]] CDRom&  cdrom() noexcept { return *cdrom_; }
    [[nodiscard]] DMA&    dma()   noexcept { return *dma_; }

private:
    // ── Owned components ──────────────────────────────────────────────────────
    // Heap-allocated: RAM (2 MiB) + BIOS (512 KiB) cannot live on the stack.
    std::unique_ptr<Ram>   ram_;
    std::unique_ptr<Bios>  bios_;
    std::unique_ptr<GPU>   gpu_;
    std::unique_ptr<IRQ>   irq_;
    std::unique_ptr<CDRom>   cdrom_;    // after irq_ (holds irq_ reference)
    std::unique_ptr<Timers>  timers_;   // after irq_
    std::unique_ptr<DMA>     dma_;      // last (needs ram_, gpu_, irq_)

    // Scratchpad — 1 KiB fast SRAM (modelled as D-cache on real hardware)
    std::array<u8, PSX::SCRATCH_SIZE> scratch_{};

    // SPU register file — 0x280 bytes (0x1F801C00–0x1F801E7F), backed by a
    // byte array so that 16-bit and 32-bit writes are preserved on read-back.
    // This lets the cpu/code-in-io test write jr-$ra to a SPU voice register,
    // then execute from that address and return cleanly without a bus error.
    static constexpr u32 SPU_REG_SIZE = 0x280u;
    std::array<u8, SPU_REG_SIZE> spu_regs_{};

    // ── SIO0 / Joypad state ───────────────────────────────────────────────────
    // Implements the minimal digital-pad exchange protocol so BIOS pad detection
    // succeeds and ps1-tests input/pad.exe can read button state.
    //
    // Exchange sequence (host sends → controller responds):
    //   byte 0: 0x01 (addr)    → 0xFF
    //   byte 1: 0x42 (poll)   → 0x41 (digital pad, 1 halfword payload)
    //   byte 2: 0x00 (TAP)    → 0x5A (payload header)
    //   byte 3: 0x00          → buttons_lo  (active-low)
    //   byte 4: 0x00          → buttons_hi  (active-low)
    //
    // JOY_STAT layout:
    //   bit 0 TXRDY1: always 1  (TX FIFO has room)
    //   bit 1 RXFIFO: 1 when a response byte is waiting
    //   bit 2 TXRDY2: always 1
    //   bit 7 ACKINPUT: 0 (active) after each write (controller ACK), 1 otherwise
    struct JoyState {
        u16  buttons  = 0xFFFFu;   // active-low (0=pressed, 1=released)
        u8   rx_byte  = 0xFFu;
        bool rx_ready = false;
        u8   seq      = 0u;
    };
    mutable JoyState joy_{};

    void joy_data_write(u8 b) noexcept;

    // VBlank generation — fire IRQ0 every ~100,000 cycles.
    static constexpr u32 kVBlankPeriod    = 100'000u;
    // VBlank active duration: ~5% of frame (~10 scanlines of the 263 total).
    static constexpr u32 kVBlankDuration  = kVBlankPeriod * 10u / 263u;  // ~3802
    u32 vblank_cycles_  = 0;
    u32 vblank_active_  = 0;   // cycles since VBlank start; 0 = outside VBlank
    bool in_vblank_     = false;

    // HBlank generation — one pulse per scanline (263 per NTSC frame, ≈380 cycles).
    static constexpr u32 kHBlankPeriod   = kVBlankPeriod / 263u;  // ≈380 cycles
    // HBlank active duration: ~18% of scanline.
    static constexpr u32 kHBlankDuration = kHBlankPeriod * 18u / 100u;  // ≈68 cycles
    u32 hblank_cycles_  = 0;
    u32 hblank_active_  = 0;   // cycles since HBlank start; 0 = outside HBlank
    bool in_hblank_     = false;

    // ── MMIO helpers ─────────────────────────────────────────────────────────
    // Most I/O registers are 32-bit wide; the CD-ROM is the exception (8-bit).
    // io_read8 / io_write8 provide a byte-granular path used exclusively for
    // byte-sized CPU accesses to the I/O window.
    [[nodiscard]] u32  io_read32 (u32 phys) const noexcept;
    [[nodiscard]] u8   io_read8  (u32 phys) const noexcept;
    void               io_write32(u32 phys, u32 value) noexcept;
    void               io_write16(u32 phys, u16 value) noexcept;
    void               io_write8 (u32 phys, u8  value) noexcept;
};

// ── Template implementations ──────────────────────────────────────────────────

template<std::unsigned_integral T>
T Bus::read(u32 vaddr) const {
    const u32 phys = PSX::to_physical(vaddr);

    // ── RAM (2 MiB, mirrored ×4 → 8 MiB window) ──────────────────────────────
    if (phys < 0x0080'0000u) {
        return ram_->read<T>(phys & (Ram::SIZE - 1u));
    }

    // ── BIOS ROM ──────────────────────────────────────────────────────────────
    if (phys >= PSX::BIOS_BASE && phys < PSX::BIOS_BASE + Bios::SIZE) {
        if (!bios_) return static_cast<T>(0u);  // sideload mode: no BIOS
        return bios_->read<T>(phys - PSX::BIOS_BASE);
    }

    // ── Scratchpad ────────────────────────────────────────────────────────────
    if (phys >= PSX::SCRATCH_BASE && phys < PSX::SCRATCH_BASE + PSX::SCRATCH_SIZE) {
        const u32 off = phys - PSX::SCRATCH_BASE;
        T value{};
        std::memcpy(&value, scratch_.data() + off, sizeof(T));
        return value;
    }

    // ── Hardware I/O ──────────────────────────────────────────────────────────
    // Byte accesses use io_read8 directly — avoids consuming CD-ROM FIFO bytes
    // as a side-effect of packing a full 32-bit word.  Wider accesses use the
    // 32-bit path with lane extraction.
    if (phys >= PSX::IO_BASE && phys < PSX::IO_BASE + PSX::IO_SIZE) {
        if constexpr (sizeof(T) == 1) {
            return static_cast<T>(io_read8(phys));
        } else {
            const u32 word = io_read32(phys & ~0x3u);
            if constexpr (sizeof(T) == 4) return static_cast<T>(word);
            if constexpr (sizeof(T) == 2) return static_cast<T>(word >> ((phys & 2u) * 8u));
        }
    }

    std::fprintf(stderr, "[Bus] unhandled read%zu  phys=0x%08X (virt=0x%08X)\n",
                 sizeof(T) * 8u, phys, vaddr);
    return static_cast<T>(0xFFFF'FFFFu);
}

template<std::unsigned_integral T>
void Bus::write(u32 vaddr, T value) {
    const u32 phys = PSX::to_physical(vaddr);

    // ── RAM ───────────────────────────────────────────────────────────────────
    if (phys < 0x0080'0000u) {
        ram_->write<T>(phys & (Ram::SIZE - 1u), value);
        return;
    }

    // ── BIOS — writes are silently ignored (ROM) ──────────────────────────────
    if (phys >= PSX::BIOS_BASE && phys < PSX::BIOS_BASE + Bios::SIZE) {
        return;
    }

    // ── Scratchpad ────────────────────────────────────────────────────────────
    if (phys >= PSX::SCRATCH_BASE && phys < PSX::SCRATCH_BASE + PSX::SCRATCH_SIZE) {
        const u32 off = phys - PSX::SCRATCH_BASE;
        std::memcpy(scratch_.data() + off, &value, sizeof(T));
        return;
    }

    // ── Hardware I/O ──────────────────────────────────────────────────────────
    if (phys >= PSX::IO_BASE && phys < PSX::IO_BASE + PSX::IO_SIZE) {
        if constexpr (sizeof(T) == 4) {
            io_write32(phys, static_cast<u32>(value));
        } else if constexpr (sizeof(T) == 2) {
            io_write16(phys, static_cast<u16>(value));
        } else {
            io_write8(phys, static_cast<u8>(value));
        }
        return;
    }

    // ── Cache-control register ────────────────────────────────────────────────
    if (phys == PSX::CACHE_CTL) {
        // TODO: cache isolation / swap bits (currently handled in CPU via COP0.SR)
        return;
    }

    std::fprintf(stderr, "[Bus] unhandled write%zu phys=0x%08X val=0x%08X\n",
                 sizeof(T) * 8u, phys, static_cast<u32>(value));
}
