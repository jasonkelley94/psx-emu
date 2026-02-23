#pragma once

#include "common/Types.hpp"

// ── Interrupt sources ─────────────────────────────────────────────────────────
// I_STAT / I_MASK bit positions (0–10).
enum class IRQSource : u32 {
    VBlank   = 0,
    GPU      = 1,
    CDROM    = 2,
    DMA      = 3,
    TMR0     = 4,
    TMR1     = 5,
    TMR2     = 6,
    Ctrl     = 7,   // Controller + Memory Card
    SIO      = 8,
    SPU      = 9,
    LightPen = 10,
};

// ── Interrupt Controller ───────────────────────────────────────────────────────
// Two 11-bit registers live at:
//   0x1F80_1070  I_STAT  (R=pending, W=acknowledge — write-0-to-clear)
//   0x1F80_1074  I_MASK  (R/W — simple mask, only low 11 bits used)
//
// An interrupt is presented to the CPU when (I_STAT & I_MASK) != 0.
// The CPU checks this via COP0 SR.IM[2] (hardware interrupt line 0).
class IRQ {
public:
    // ── Device interface ──────────────────────────────────────────────────────
    // Devices call set() to assert an interrupt line.
    void set(IRQSource src) noexcept {
        stat_ |= (1u << static_cast<u32>(src));
    }

    // ── Bus / CPU interface ───────────────────────────────────────────────────
    [[nodiscard]] u32  read_stat() const noexcept { return stat_; }
    [[nodiscard]] u32  read_mask() const noexcept { return mask_; }

    // I_STAT write: semantics are write-0-to-clear (AND with written value).
    void write_stat(u32 val) noexcept { stat_ &= val; }

    // I_MASK write: plain R/W, unused bits always read 0.
    void write_mask(u32 val) noexcept { mask_ = val & 0x7FFu; }

    // True when any enabled interrupt is pending — used by the CPU to decide
    // whether to take an external hardware interrupt (COP0 IP[2]).
    [[nodiscard]] bool irq_pending() const noexcept {
        return (stat_ & mask_) != 0;
    }

private:
    u32 stat_ = 0;   // I_STAT — set by hardware, cleared by software
    u32 mask_ = 0;   // I_MASK — written by software
};
