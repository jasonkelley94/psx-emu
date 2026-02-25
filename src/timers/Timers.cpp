#include "Timers.hpp"
#include <cstdio>

// ── Bus interface ─────────────────────────────────────────────────────────────
u32 Timers::read(u32 off) const noexcept {
    const u32 n   = (off >> 4u) & 3u;
    const u32 reg = (off >> 2u) & 3u;
    if (n >= 3u) return 0;
    switch (reg) {
    case 0:  return tmr_[n].counter & 0xFFFFu;
    case 1: {
        const u32 val = tmr_[n].mode;
        tmr_[n].mode &= ~((1u << 11u) | (1u << 12u));
        return val;
    }
    case 2:  return tmr_[n].target & 0xFFFFu;
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
        tmr_[n].mode          = value & 0x07FFu;
        tmr_[n].counter       = 0;
        tmr_[n].pending_reset  = false;
        tmr_[n].sync3_released = false;
        if (n == 0) dot_frac_  = 0;
        if (n == 2) sys8_frac_ = 0;
        break;
    case 2:  tmr_[n].target = value & 0xFFFFu; break;
    default: break;
    }
}

// ── advance() ─────────────────────────────────────────────────────────────────
void Timers::advance(u32 n, u32 ticks) noexcept {
    if (ticks == 0) return;
    auto& t = tmr_[n];
    const u32 tgt = t.target & 0xFFFFu;

    if (t.pending_reset) {
        t.pending_reset = false;
        t.counter = (tgt > 0u && t.counter >= tgt) ? (t.counter - tgt) : 0u;
        if (ticks == 1u) return;
        ticks -= 1u;
    }

    const u32 old = t.counter;
    t.counter = (t.counter + ticks) & 0xFFFFu;

    const bool overflow   = (t.counter < old) || (ticks > 0xFFFFu);
    const bool target_hit = (tgt > 0u) && (
        (!overflow && old < tgt && t.counter >= tgt) ||
        ( overflow && (old < tgt || t.counter >= tgt)));

    if (target_hit && ((t.mode >> 3u) & 1u)) {
        t.counter       = tgt;
        t.pending_reset = true;
    }
    if (target_hit) {
        t.mode |= (1u << 11u);
        if ((t.mode >> 4u) & 1u) irq_.set(kSrc[n]);
    }
    if (overflow) {
        t.mode |= (1u << 12u);
        if ((t.mode >> 5u) & 1u) irq_.set(kSrc[n]);
    }
}

// ── tick() ────────────────────────────────────────────────────────────────────
void Timers::tick(u32 cycles) noexcept {
    // ── Timer 0 ───────────────────────────────────────────────────────────────
    {
        auto& t = tmr_[0];
        const bool se = sync_enable(t.mode);
        const u32  sm = sync_mode(t.mode);

        bool run = true;
        if (se) {
            switch (sm) {
            case 0: run = !in_hblank_;        break;  // pause during HBlank
            case 1: run = true;                break;  // always run
            case 2: run = in_hblank_;          break;  // run only during HBlank
            case 3: run = t.sync3_released;    break;  // pause until first HBlank
            }
        }

        if (run) {
            if ((t.mode >> 8u) & 1u) {
                // Dotclock source: one dot per (sys / divisor) system cycles.
                // Divisor is read from the GPU's current horizontal resolution.
                const u32 div = gpu_.dot_divisor();
                dot_frac_ += cycles;
                const u32 ticks = dot_frac_ / div;
                dot_frac_ %= div;
                advance(0, ticks);
            } else {
                advance(0, cycles);
            }
        }
    }

    // ── Timer 1 ───────────────────────────────────────────────────────────────
    {
        auto& t = tmr_[1];
        // HBlank clock-source: counter is only advanced in hblank_begin().
        if (!((t.mode >> 8u) & 1u)) {
            const bool se = sync_enable(t.mode);
            const u32  sm = sync_mode(t.mode);
            bool run = true;
            if (se) {
                switch (sm) {
                case 0: run = !in_vblank_;      break;
                case 1: run = true;              break;
                case 2: run = in_vblank_;        break;
                case 3: run = t.sync3_released;  break;
                }
            }
            if (run) advance(1, cycles);
        }
    }

    // ── Timer 2 ───────────────────────────────────────────────────────────────
    {
        auto& t = tmr_[2];
        const bool se = sync_enable(t.mode);
        const u32  sm = sync_mode(t.mode);
        // Modes 0 and 3 with sync_enable stop the counter permanently.
        if (se && (sm == 0u || sm == 3u)) return;

        if ((t.mode >> 9u) & 1u) {
            sys8_frac_ += cycles;
            const u32 ticks = sys8_frac_ / 8u;
            sys8_frac_ %= 8u;
            advance(2, ticks);
        } else {
            advance(2, cycles);
        }
    }
}

// ── hblank_begin() ────────────────────────────────────────────────────────────
void Timers::hblank_begin() noexcept {
    in_hblank_ = true;

    // Timer 1 HBlank clock-source: count one pulse (gated by VBlank sync mode).
    auto& t1 = tmr_[1];
    if ((t1.mode >> 8u) & 1u) {
        const bool se = sync_enable(t1.mode);
        const u32  sm = sync_mode(t1.mode);
        bool count = true;
        if (se) {
            switch (sm) {
            case 0: count = !in_vblank_;      break;
            case 1: count = true;              break;
            case 2: count = in_vblank_;        break;
            case 3: count = t1.sync3_released; break;
            }
        }
        if (count) advance(1, 1u);
    }

    // Timer 0 HBlank-triggered resets (modes 1 and 2).
    auto& t0 = tmr_[0];
    if (sync_enable(t0.mode)) {
        const u32 sm = sync_mode(t0.mode);
        if (sm == 1u || sm == 2u) {
            t0.counter       = 0;
            t0.pending_reset = false;
        }
        if (sm == 3u && !t0.sync3_released) {
            t0.sync3_released = true;
        }
    }
}

// ── hblank_end() ──────────────────────────────────────────────────────────────
void Timers::hblank_end() noexcept {
    in_hblank_ = false;
}

// ── vblank_begin() ────────────────────────────────────────────────────────────
void Timers::vblank_begin() noexcept {
    in_vblank_ = true;

    auto& t1 = tmr_[1];
    if (sync_enable(t1.mode)) {
        const u32 sm = sync_mode(t1.mode);
        if (sm == 1u || sm == 2u) {
            t1.counter       = 0;
            t1.pending_reset = false;
        }
        if (sm == 3u && !t1.sync3_released) {
            t1.sync3_released = true;
        }
    }
}

// ── vblank_end() ──────────────────────────────────────────────────────────────
void Timers::vblank_end() noexcept {
    in_vblank_ = false;
}
