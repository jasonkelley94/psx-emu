#pragma once

#include <array>
#include "common/Types.hpp"
#include "irq/IRQ.hpp"

// ── Root Counters (Timers 0–2) ────────────────────────────────────────────────
// Three 16-bit free-running counters in the I/O window:
//
//   Base = IO_BASE + 0x100
//   Timer N starts at base + N * 0x10
//
//   +0x0  Counter value  (16-bit, read/write)
//   +0x4  Counter mode   (16-bit, writing it resets the counter)
//   +0x8  Target value   (16-bit, read/write)
//
// Selected mode bits:
//   [2:1] Sync mode      (meaning depends on timer)
//   [3]   Reset on target  (0 = free-run to 0xFFFF; 1 = reset to 0 on target)
//   [4]   IRQ on target reach
//   [5]   IRQ on counter overflow (0xFFFF → 0)
//   [6]   Repeat IRQ mode   (0 = one-shot, 1 = repeat)
//   [8]   Clock source select (Timer0: 0=sys,1=dotclock; Timer1: 0=sys,1=HBlank)
//   [9]   Clock source (Timer2: 0/1=sys, 1x=sys/8)
//   [11]  TargetReached    (set when counter == target; cleared on mode read/write)
//   [12]  OverflowReached  (set when counter overflows; cleared on mode read/write)
//
// Clock sources per timer:
//   Timer 0: mode[8]=0 → system clock, mode[8]=1 → dotclock (~sys/5 for 320x240)
//   Timer 1: mode[8]=0 → system clock, mode[8]=1 → HBlank pulse count
//   Timer 2: mode[9:8]=0x/0y → system clock, mode[9]=1 → system clock / 8
// ─────────────────────────────────────────────────────────────────────────────
class Timers {
public:
    explicit Timers(IRQ& irq) noexcept : irq_(irq) {}

    // ── Bus interface (offset relative to IO_BASE + 0x100) ───────────────────
    [[nodiscard]] u32 read (u32 off) const noexcept;
    void              write(u32 off, u32 value) noexcept;

    // ── Advance all counters by `cycles` system clock ticks ──────────────────
    void tick(u32 cycles) noexcept;

    // ── HBlank pulse — called once per scanline (~380 sys cycles in NTSC) ────
    // Timer1 in HBlank mode increments its counter by 1 here.
    void hblank_tick() noexcept;

    // ── VBlank pulse — called once per frame ─────────────────────────────────
    // Used for Timer1 sync modes (pause/reset on VBlank).
    void vblank_tick() noexcept;

private:
    IRQ& irq_;

    static constexpr IRQSource kSrc[3] = {
        IRQSource::TMR0, IRQSource::TMR1, IRQSource::TMR2
    };

    struct Timer {
        u32 counter = 0;
        // mode is mutable so that the const read() path can clear the
        // read-clear status bits [11] (targetReached) and [12] (overflowReached).
        mutable u32 mode = 0;
        u32 target  = 0;
        // Deferred-reset flag: when the counter reaches the target with mode[3]=1,
        // we keep counter == target for 1 clock so software can observe the value.
        // On the NEXT advance() call, the reset is applied before incrementing.
        bool pending_reset = false;
    };
    std::array<Timer, 3> tmr_{};

    // Fractional accumulators for sub-integer clock sources.
    // Fixed-point with kFracBits fractional bits.
    static constexpr u32 kFracBits = 8u;
    static constexpr u32 kFracOne  = 1u << kFracBits;  // 256

    // Timer0 dotclock: GPU pixel clock ≈ sys * 11 / 7 / dot_div
    // For 320x240 (dot_div=8): sys * 11/56 ≈ sys/5.09 → 198 dots per 1000 sys
    u32 dot_frac_ = 0;   // fractional accumulator (accumulated * 11 / 7 / dot_div)

    // Timer2 sys/8: 1 tick every 8 sys clocks
    u32 sys8_frac_ = 0;  // fractional accumulator

    // Advances a single timer counter by `ticks`, handling target/overflow IRQs.
    void advance(u32 n, u32 ticks) noexcept;
};
