#include "Timers.hpp"

// ── Bus interface ─────────────────────────────────────────────────────────────
// Offset layout within the timer window (IO_BASE + 0x100):
//   bits [5:4] of off → timer index (0, 1, 2)
//   bits [3:2] of off → register within timer (0=counter, 1=mode, 2=target)

u32 Timers::read(u32 off) const noexcept {
    const u32 n   = (off >> 4u) & 3u;
    const u32 reg = (off >> 2u) & 3u;
    if (n >= 3u) return 0;
    switch (reg) {
    case 0:  return tmr_[n].counter & 0xFFFFu;
    case 1: {
        // Mode register read: return current value then clear the read-clear
        // status bits [11] (targetReached) and [12] (overflowReached).
        const u32 val = tmr_[n].mode;
        tmr_[n].mode &= ~((1u << 11u) | (1u << 12u));  // clear after read
        return val;
    }
    case 2:  return tmr_[n].target  & 0xFFFFu;
    default: return 0;
    }
}

void Timers::write(u32 off, u32 value) noexcept {
    const u32 n   = (off >> 4u) & 3u;
    const u32 reg = (off >> 2u) & 3u;
    if (n >= 3u) return;
    switch (reg) {
    case 0:  tmr_[n].counter = value & 0xFFFFu; break;
    case 1:
        // Writing mode: accept new mode bits [10:0] from value, clear status
        // bits [12:11] (they are set by hardware and cleared on read or write).
        tmr_[n].mode         = value & 0x07FFu;  // keep only writable bits [10:0]
        tmr_[n].counter      = 0;                // writing mode resets counter
        tmr_[n].pending_reset = false;           // cancel any deferred reset
        // Also clear fractional accumulators when clock source changes.
        if (n == 0) dot_frac_  = 0;
        if (n == 2) sys8_frac_ = 0;
        break;
    case 2:  tmr_[n].target = value & 0xFFFFu;  break;
    default: break;
    }
}

// ── advance() ─────────────────────────────────────────────────────────────────
// Advance timer N's counter by `ticks`, firing IRQs on target-reach / overflow.
//
// PSX hardware behavior: when the counter reaches the target value, it stays AT
// the target for 1 clock cycle (observable by software), then resets to 0 at the
// START of the next clock.  We model this with a one-clock deferred reset:
//   • On the tick where counter == target: set pending_reset, leave counter at target.
//   • On the NEXT advance() call: apply the reset before any new increment.
void Timers::advance(u32 n, u32 ticks) noexcept {
    if (ticks == 0) return;
    auto& t = tmr_[n];
    const u32 tgt = t.target & 0xFFFFu;

    // ── Apply deferred reset from the previous tick ───────────────────────────
    // The reset replaces the normal increment for this first tick, so decrement
    // the remaining ticks budget by 1 to model the consumed clock edge.
    if (t.pending_reset) {
        t.pending_reset = false;
        t.counter = (tgt > 0u && t.counter >= tgt) ? (t.counter - tgt) : 0u;
        if (ticks == 1u) return;   // that tick was consumed by the reset
        ticks -= 1u;
    }

    const u32 old = t.counter;
    t.counter = (t.counter + ticks) & 0xFFFFu;

    const bool overflow = (t.counter < old) || (ticks > 0xFFFFu);

    // Target-match detection: did the counter cross tgt this tick?
    //   Case 1 (no wrap)  old < tgt <= new
    //   Case 2 (wrap)     old < tgt  (hit before wrapping)
    //                  OR new >= tgt (hit after wrapping)
    const bool target_hit = (tgt > 0u) && (
        (!overflow && old < tgt && t.counter >= tgt) ||
        ( overflow && (old < tgt || t.counter >= tgt)));

    // Deferred reset (mode[3]): snap counter to tgt and set the pending flag
    // so the actual reset to 0 happens on the next advance() call.
    // This makes counter == tgt observable for exactly 1 system clock cycle.
    if (target_hit && ((t.mode >> 3u) & 1u)) {
        t.counter      = tgt;        // hold at target value this clock
        t.pending_reset = true;      // reset to 0 at start of next clock
    }

    // Target-reach IRQ (mode[4]) and status flag (mode[11]).
    if (target_hit) {
        t.mode |= (1u << 11u);           // set targetReached status flag
        if ((t.mode >> 4u) & 1u) irq_.set(kSrc[n]);
    }

    // Overflow IRQ (mode[5]) and status flag (mode[12]).
    if (overflow) {
        t.mode |= (1u << 12u);           // set overflowReached status flag
        if ((t.mode >> 5u) & 1u) irq_.set(kSrc[n]);
    }
}

// ── tick() ────────────────────────────────────────────────────────────────────
// Advances timers based on their configured clock source.
//
// Timer 0 clock sources (mode[8]):
//   0 = System clock  — 1 tick per sys cycle
//   1 = Dot clock     — ≈1 tick per 5 sys cycles (320×240 NTSC 6.65 MHz dotclk)
//
// Timer 1 clock sources (mode[8]):
//   0 = System clock  — 1 tick per sys cycle
//   1 = HBlank        — 1 tick per HBlank pulse (via hblank_tick())
//
// Timer 2 clock sources (mode[9]):
//   0 = System clock  — 1 tick per sys cycle
//   1 = System / 8    — 1 tick per 8 sys cycles
void Timers::tick(u32 cycles) noexcept {
    // ── Timer 0 ───────────────────────────────────────────────────────────────
    if ((tmr_[0].mode >> 8u) & 1u) {
        // Dot clock: GPU pixel clock ≈ sys × 11 / (7 × 8) for 320×240.
        // Numerator: sys × 11 = accumulated in dot_frac_.
        // Denominator: 7 × 8 = 56 → 1 dot tick every 56/11 ≈ 5.09 sys cycles.
        dot_frac_ += cycles * 11u;
        const u32 dot_ticks = dot_frac_ / 56u;
        dot_frac_ %= 56u;
        advance(0, dot_ticks);
    } else {
        // System clock.
        advance(0, cycles);
    }

    // ── Timer 1 ───────────────────────────────────────────────────────────────
    // HBlank mode: counter is advanced by hblank_tick(), not here.
    if (!((tmr_[1].mode >> 8u) & 1u)) {
        advance(1, cycles);
    }

    // ── Timer 2 ───────────────────────────────────────────────────────────────
    if ((tmr_[2].mode >> 9u) & 1u) {
        // System clock / 8.
        sys8_frac_ += cycles;
        const u32 sys8_ticks = sys8_frac_ / 8u;
        sys8_frac_ %= 8u;
        advance(2, sys8_ticks);
    } else {
        advance(2, cycles);
    }
}

// ── hblank_tick() ─────────────────────────────────────────────────────────────
// Called once per HBlank pulse (~380 sys cycles in NTSC).
// Advances Timer1 counter when it is in HBlank clock-source mode.
void Timers::hblank_tick() noexcept {
    if ((tmr_[1].mode >> 8u) & 1u) {
        advance(1, 1u);
    }
}

// ── vblank_tick() ─────────────────────────────────────────────────────────────
// Called once per VBlank pulse (once per frame).
// Placeholder for Timer1 sync mode behavior (reset/pause at VBlank).
// Currently a no-op — sync modes are not yet implemented.
void Timers::vblank_tick() noexcept {
    // TODO: implement Timer1 sync modes 1/2 (reset counter at VBlank).
}
