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
//   [0]   Sync enable      (0 = free-run; 1 = sync mode active)
//   [2:1] Sync mode        (meaning depends on timer, see below)
//   [3]   Reset on target  (0 = free-run to 0xFFFF; 1 = reset to 0 on target)
//   [4]   IRQ on target reach
//   [5]   IRQ on counter overflow (0xFFFF → 0)
//   [6]   Repeat IRQ mode   (0 = one-shot, 1 = repeat)
//   [8]   Clock source select (Timer0: 0=sys,1=dotclock; Timer1: 0=sys,1=HBlank)
//   [9]   Clock source (Timer2: 0/1=sys, 1x=sys/8)
//   [11]  TargetReached    (set when counter == target; cleared on mode read/write)
//   [12]  OverflowReached  (set when counter overflows; cleared on mode read/write)
//
// Sync modes per timer:
//   Timer 0 (HBlank):
//     0 = Pause counter during HBlank
//     1 = Reset counter to 0 at HBlank start
//     2 = Reset to 0 at HBlank start AND pause outside HBlank
//     3 = Pause until first HBlank occurs, then free-run
//   Timer 1 (VBlank):
//     0 = Pause counter during VBlank
//     1 = Reset counter to 0 at VBlank start
//     2 = Reset to 0 at VBlank start AND pause outside VBlank
//     3 = Pause until first VBlank occurs, then free-run
//   Timer 2 (no H/V sync — uses internal gate only):
//     0 = Stop counter permanently
//     1 = Free run
//     2 = Free run
//     3 = Stop counter permanently
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

    // ── HBlank events — called by Bus at start/end of each HBlank ────────────
    void hblank_begin() noexcept;
    void hblank_end()   noexcept;

    // ── VBlank events — called by Bus at start/end of each VBlank ────────────
    void vblank_begin() noexcept;
    void vblank_end()   noexcept;

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
        // For sync mode 3 (T0/T1): pause until first blank, then free-run.
        // Set to true on first hblank_begin()/vblank_begin() after mode write.
        bool sync3_released = false;
    };
    std::array<Timer, 3> tmr_{};

    // Current blanking state — updated by hblank_begin/end and vblank_begin/end.
    bool in_hblank_ = false;
    bool in_vblank_ = false;

    // Fractional accumulators for sub-integer clock sources.
    u32 dot_frac_  = 0;   // Timer0 dotclock accumulator
    u32 sys8_frac_ = 0;   // Timer2 sys/8 accumulator

    // Helper: extract sync_enable (bit 0) and sync_mode (bits [2:1]).
    static bool sync_enable(u32 mode) noexcept { return mode & 1u; }
    static u32  sync_mode  (u32 mode) noexcept { return (mode >> 1u) & 3u; }

    // Advances a single timer counter by `ticks`, handling target/overflow IRQs.
    void advance(u32 n, u32 ticks) noexcept;
};
