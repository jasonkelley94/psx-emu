#pragma once

#include <array>
#include <bit>
#include <algorithm>
#include <cstdio>
#include <limits>
#include <type_traits>
#include "common/Types.hpp"

// ── GTE (Geometry Transformation Engine) — COP2 ───────────────────────────────
//
// The GTE is a fixed-function math coprocessor wired as MIPS COP2.
// It accelerates the perspective projection, lighting, and color-blending
// operations needed for 3D rendering on the PSX.
//
// ── Fixed-point number system ─────────────────────────────────────────────────
//
// All GTE arithmetic uses a unified 12-bit fractional representation:
//
//   Format       C++ type  Range (real units)     Primary use
//   ──────────────────────────────────────────────────────────────────────────
//   1.3.12  s16  fp16      −8  … +7.9997          Matrix elements, IR regs
//   1.19.12 s32  fp32      −524288 … +524287.9997  MAC regs (sf=0 result)
//   1.31.12 s64  fp44      −2^43 … +2^43−1        44-bit accumulator
//
// The naming "1.N.12" means: 1 sign bit, N integer bits, 12 fractional bits.
// The least-significant bit of fp16 represents 2^−12 ≈ 0.000244 real units.
//
// Key arithmetic relations:
//   • product of two fp16: raw_a × raw_b  → 32-bit, 24 implicit frac. bits
//   • translation scaled:  (s64)TR_raw << 12  → aligns to fp44 space
//   • MAC with sf=1:        fp44 >> 12 → fp32 (back to integer world units)
//   • MAC with sf=0:        fp44 → fp32  (keeps 12 fractional bits)
//
// ── Accumulator overflow detection ────────────────────────────────────────────
//
// The PSX GTE uses 44-bit signed accumulators (MAC1/2/3) internally.
// Overflow is flagged when the s64 value falls outside [−2^43, 2^43−1].
// Because the host s64 has 63 bits, we can sum multiple s16×s16 products
// without host overflow and check the 44-bit range once at the end.
//
// ── Register file layout ──────────────────────────────────────────────────────
//
// CP2 Data registers (accessed by MFC2/MTC2):
//
//   0  VXY0   [15:0]=VX0   [31:16]=VY0    Vector 0 XY (s16 each)
//   1  VZ0    [15:0]=VZ0   [31:16]=pad    Vector 0 Z  (s16)
//   2  VXY1   [15:0]=VX1   [31:16]=VY1
//   3  VZ1    [15:0]=VZ1
//   4  VXY2   [15:0]=VX2   [31:16]=VY2
//   5  VZ2    [15:0]=VZ2
//   6  RGBC   R,G,B,CODE as u8 in [7:0],[15:8],[23:16],[31:24]
//   7  OTZ    [15:0]=OTZ   average Z (u16)
//   8  IR0    [15:0]=IR0   interpolation factor (fp16, [0,+0x1000])
//   9  IR1    [15:0]=IR1   result X (fp16)
//  10  IR2    [15:0]=IR2   result Y (fp16)
//  11  IR3    [15:0]=IR3   result Z (fp16)
//  12  SXY0   [15:0]=SX0   [31:16]=SY0    screen XY FIFO slot 0 (s16 each)
//  13  SXY1   [15:0]=SX1   [31:16]=SY1
//  14  SXY2   [15:0]=SX2   [31:16]=SY2   (newest; pushed on write to reg 15)
//  15  SXYP   mirror of SXY2 on read; write pushes FIFO: 0←1, 1←2, 2←val
//  16  SZ0    [15:0]=SZ0   screen Z FIFO slot 0 (u16)
//  17  SZ1    [15:0]=SZ1
//  18  SZ2    [15:0]=SZ2
//  19  SZ3    [15:0]=SZ3   (newest)
//  20  RGB0   R,G,B,CODE color FIFO slot 0 (u8 each)
//  21  RGB1
//  22  RGB2   (newest)
//  23  RES1   prohibited / garbage
//  24  MAC0   s32 accumulator (perspective / interpolation)
//  25  MAC1   s32 accumulator (X component)
//  26  MAC2   s32 accumulator (Y component)
//  27  MAC3   s32 accumulator (Z component)
//  28  IRGB   write-only: decodes 5-bit RGB → IR1,IR2,IR3
//  29  ORGB   read-only:  encodes IR1,IR2,IR3 → 5-bit RGB
//  30  LZCS   leading-zero-count source (write to set)
//  31  LZCR   leading-zero-count result (read-only)
//
// CP2 Control registers (accessed by CFC2/CTC2):
//
//   0  RT11RT12  [15:0]=RT11  [31:16]=RT12   Rotation matrix, row 1 (fp16 each)
//   1  RT13RT21  [15:0]=RT13  [31:16]=RT21
//   2  RT22RT23  [15:0]=RT22  [31:16]=RT23
//   3  RT31RT32  [15:0]=RT31  [31:16]=RT32
//   4  RT33      [15:0]=RT33
//   5  TRX       s32  Translation X (integer world units)
//   6  TRY       s32  Translation Y
//   7  TRZ       s32  Translation Z
//   8  L11L12    Light matrix row 1 (fp16 each)
//   9  L13L21
//  10  L22L23
//  11  L31L32
//  12  L33
//  13  RBK       s32  Background colour Red   (fp32, 1.19.12)
//  14  GBK       s32  Background colour Green
//  15  BBK       s32  Background colour Blue
//  16  LR1LR2    Colour matrix row 1 (fp16 each)
//  17  LR3LG1
//  18  LG2LG3
//  19  LB1LB2
//  20  LB3
//  21  RFC       s32  Far colour Red   (fp32, 1.19.12)
//  22  GFC       s32  Far colour Green
//  23  BFC       s32  Far colour Blue
//  24  OFX       s32  Screen offset X (fp32 — added to projected X)
//  25  OFY       s32  Screen offset Y
//  26  H         u16  Projection plane distance
//  27  DQA       s16  Depth-queing coefficient A
//  28  DQB       s32  Depth-queing coefficient B (fp32, 1.19.12)
//  29  ZSF3      s16  Z average scale factor for 3-point average
//  30  ZSF4      s16  Z average scale factor for 4-point average
//  31  FLAG      u32  Overflow/saturation flags (see namespace Flag below)

// ── GTE flag bits ─────────────────────────────────────────────────────────────
namespace Flag {
    // MAC1/2/3 44-bit overflow — positive (bit 30/29/28) and negative (27/26/25)
    inline constexpr u32 MAC1_POS  = 1u << 30;
    inline constexpr u32 MAC2_POS  = 1u << 29;
    inline constexpr u32 MAC3_POS  = 1u << 28;
    inline constexpr u32 MAC1_NEG  = 1u << 27;
    inline constexpr u32 MAC2_NEG  = 1u << 26;
    inline constexpr u32 MAC3_NEG  = 1u << 25;
    // IR saturation
    inline constexpr u32 IR1_SAT   = 1u << 24;
    inline constexpr u32 IR2_SAT   = 1u << 23;
    inline constexpr u32 IR3_SAT   = 1u << 22;  // NOT part of error summary
    // Colour FIFO saturation
    inline constexpr u32 RGB_R_SAT = 1u << 21;
    inline constexpr u32 RGB_G_SAT = 1u << 20;
    inline constexpr u32 RGB_B_SAT = 1u << 19;
    // Screen-Z / OTZ saturation (bit 18)
    inline constexpr u32 SZ3_SAT   = 1u << 18;  // also used for OTZ
    // Unreal-division overflow (bit 17)
    inline constexpr u32 DIV_OVF   = 1u << 17;
    // MAC0 31-bit overflow (bits 16/15)
    inline constexpr u32 MAC0_POS  = 1u << 16;
    inline constexpr u32 MAC0_NEG  = 1u << 15;
    // Screen-coordinate saturation (bits 14/13)
    inline constexpr u32 SX2_SAT   = 1u << 14;
    inline constexpr u32 SY2_SAT   = 1u << 13;
    // IR0 saturation (bit 12) — NOT part of error summary
    inline constexpr u32 IR0_SAT   = 1u << 12;
    // Bit 31 (error summary) = 1 if any bit in ERROR_MASK is set.
    // Hardware sets this automatically; emulation recomputes it after each cmd.
    // Included bits: 30–23 (MAC1/2/3 ovfl, IR1/IR2 sat) and 18–13.
    // Notably excluded: bit 22 (IR3_SAT), bits 21–19 (RGB_x_SAT), bit 12 (IR0_SAT).
    inline constexpr u32 ERROR_MASK = 0x7F87'E000u;
}

// ── Fixed-point type aliases ──────────────────────────────────────────────────
namespace gte {

using fp16 = s16;   // 1.3.12   — matrix elements, IR regs, OTZ, H, DQA, ZSFx
using fp32 = s32;   // 1.19.12  — MAC1/2/3 after sf=0, BK/FC/OFX/OFY/DQB
using fp44 = s64;   // 1.31.12  — 44-bit intermediate accumulator (in s64)

// Number of fractional bits shared by all GTE fixed-point formats.
inline constexpr int FP_SHIFT = 12;

// 44-bit signed range (host s64 is wider, so overflow is safely detectable).
inline constexpr fp44 FP44_MIN = -(s64{1} << 43);
inline constexpr fp44 FP44_MAX =  (s64{1} << 43) - 1;

// ── Compound types ────────────────────────────────────────────────────────────
// Row-major 3×3 matrix of fp16.  Accessed as M[row][col], rows 0..2.
using Mat3   = std::array<std::array<fp16, 3>, 3>;
// Three-element column vectors.
using Vec16  = std::array<fp16, 3>;
using Vec32  = std::array<s32,  3>;

// ── GTE command word decoder ──────────────────────────────────────────────────
// A COP2 instruction word with bit 25 set encodes a GTE command.
// Bits [24:0] of the instruction are the opaque 25-bit command word.
struct Cmd {
    u32 raw;

    // 6-bit function code — selects the specific GTE operation.
    [[nodiscard]] constexpr u32  func()  const noexcept { return raw & 0x3Fu; }
    // lm — "limit mode": if true, IR1/2/3 are clamped to [0, +0x7FFF] (no neg).
    [[nodiscard]] constexpr bool lm()    const noexcept { return (raw >> 10) & 1u; }
    // sf — "shift fraction": if true, right-shift accumulator by 12 before
    //       storing to MAC; if false, store the full 44-bit value truncated to 32.
    [[nodiscard]] constexpr bool sf()    const noexcept { return (raw >> 19) & 1u; }
    // Shift amount derived from sf flag.
    [[nodiscard]] constexpr int  shift() const noexcept { return sf() ? FP_SHIFT : 0; }
    // MVMVA operand selectors (only meaningful for the MVMVA command).
    [[nodiscard]] constexpr u32  mx()    const noexcept { return (raw >> 17) & 3u; }
    [[nodiscard]] constexpr u32  vx()    const noexcept { return (raw >> 15) & 3u; }
    [[nodiscard]] constexpr u32  tx()    const noexcept { return (raw >> 13) & 3u; }
};

// ── Free-function saturation helpers ─────────────────────────────────────────
//
// Each function clamps a value to a hardware-defined range and, on overflow,
// sets the corresponding FLAG bit(s) in the u32 passed by reference.
// Callers must call GTE::update_flag() after a complete command to refresh
// the error-summary bit (FLAG[31]).
//
// Naming convention:
//   sat_mac(v, n, flag)  — 44-bit → s32 for MAC1,MAC2,MAC3 (n=1,2,3)
//   sat_mac0(v, flag)    — s64    → s32 for MAC0 (31-bit check)
//   sat_ir(v, n, lm, f)  — s32    → fp16 for IR1,IR2,IR3 (n=1,2,3)
//   sat_ir0(v, flag)     — s32    → fp16 for IR0 ([0,+0x1000])
//   sat_sxy(v, bit, f)   — s32    → s16  for screen X or Y ([-0x400, +0x3FF])
//   sat_sz(v, flag)      — s32    → u16  for screen Z and OTZ ([0, 0xFFFF])
//   sat_rgb(v, bit, f)   — s32    → u8   for colour channel ([0, 0xFF])

// ── sat_mac: 44-bit intermediate → s32 (MAC1, MAC2, MAC3) ────────────────────
// Overflow is checked against the 44-bit range; result is clamped to s32.
// The hardware flags separate positive and negative overflow.
//
//   n=1: pos=bit30, neg=bit27
//   n=2: pos=bit29, neg=bit26
//   n=3: pos=bit28, neg=bit25
[[nodiscard]] inline s32 sat_mac(fp44 v, int n, u32& flag) noexcept {
    // First, detect and flag 44-bit overflow (n in [1,3]).
    const u32 pos_bit = 1u << (31u - static_cast<u32>(n));  // 30, 29, 28
    const u32 neg_bit = 1u << (28u - static_cast<u32>(n));  // 27, 26, 25
    if (v > FP44_MAX) { flag |= pos_bit; v = FP44_MAX; }
    if (v < FP44_MIN) { flag |= neg_bit; v = FP44_MIN; }
    // Then truncate to s32 (no additional clamping needed — FP44_MAX < INT32_MAX).
    return static_cast<s32>(v);
}

// ── sat_mac0: wide intermediate → s32 (MAC0) ─────────────────────────────────
// MAC0 is a 32-bit register.  The hardware detects overflow on the 32-bit
// boundary and sets flags, but stores the RAW low-32 bits (no clamping).
[[nodiscard]] inline s32 sat_mac0(s64 v, u32& flag) noexcept {
    constexpr s64 LO = std::numeric_limits<s32>::min();
    constexpr s64 HI = std::numeric_limits<s32>::max();
    if (v > HI) { flag |= Flag::MAC0_POS; }
    if (v < LO) { flag |= Flag::MAC0_NEG; }
    return static_cast<s32>(v);  // truncate to 32 bits, not clamped
}

// ── sat_ir: s32 → fp16 (IR1, IR2, IR3) ───────────────────────────────────────
// Clamps to [-0x8000, +0x7FFF] when lm=false, or [0, +0x7FFF] when lm=true.
// Both clamping directions set the same saturation FLAG bit for the channel.
//
//   n=1: flag bit 24
//   n=2: flag bit 23
//   n=3: flag bit 22
[[nodiscard]] inline fp16 sat_ir(s32 v, int n, bool lm, u32& flag) noexcept {
    // The saturation flag fires when the value is outside the clamping range.
    // The clamping range (and therefore the flag check) is lm-dependent:
    //   lm=0: [-0x8000, +0x7FFF]
    //   lm=1: [0,       +0x7FFF]
    constexpr s32 hi = 0x7FFF;
    const s32  lo    = lm ? 0 : -0x8000;
    const u32 flag_bit = 1u << (25u - static_cast<u32>(n));  // 24, 23, 22
    if (v > hi || v < lo) { flag |= flag_bit; }
    return static_cast<fp16>(std::clamp(v, lo, hi));
}

// ── sat_ir0: s32 → fp16 (IR0) ────────────────────────────────────────────────
// IR0 is the perspective/interpolation factor, always clamped to [0, +0x1000].
// Sets FLAG bit 12 (IR0_SAT) when the value is outside [0, +0x1000].
[[nodiscard]] inline fp16 sat_ir0(s32 v, u32& flag) noexcept {
    if (v < 0 || v > 0x1000) { flag |= Flag::IR0_SAT; }
    return static_cast<fp16>(std::clamp(v, 0, 0x1000));
}

// ── sat_sxy: s32 → s16 (screen X or Y) ───────────────────────────────────────
// Screen coordinates are clamped to [-0x400, +0x3FF] (10 signed bits).
// flag_bit must be Flag::SX2_SAT or Flag::SY2_SAT.
[[nodiscard]] inline s16 sat_sxy(s32 v, u32 flag_bit, u32& flag) noexcept {
    constexpr s32 LO = -0x400, HI = 0x3FF;
    if (v > HI || v < LO) { flag |= flag_bit; }
    return static_cast<s16>(std::clamp(v, LO, HI));
}

// ── sat_sz: s32 → u16 (screen Z, OTZ) ────────────────────────────────────────
// Unsigned 16-bit clamp.  Negative values saturate to 0.
// Both SZ3 (slot 3 of the Z FIFO) and OTZ share Flag::SZ3_SAT (bit 18).
[[nodiscard]] inline u16 sat_sz(s32 v, u32& flag) noexcept {
    constexpr s32 HI = 0xFFFF;
    if (v < 0)   { flag |= Flag::SZ3_SAT; return 0u; }
    if (v > HI)  { flag |= Flag::SZ3_SAT; return static_cast<u16>(HI); }
    return static_cast<u16>(v);
}

// ── sat_rgb: s32 → u8 (colour channel) ───────────────────────────────────────
// Clamps to [0, 0xFF].  flag_bit should be Flag::RGB_R_SAT/G/B.
[[nodiscard]] inline u8 sat_rgb(s32 v, u32 flag_bit, u32& flag) noexcept {
    if (v < 0)    { flag |= flag_bit; return 0u; }
    if (v > 0xFF) { flag |= flag_bit; return 0xFFu; }
    return static_cast<u8>(v);
}

// ── Arithmetic helpers ────────────────────────────────────────────────────────

// Scale a translation-vector component (s32 integer) into the fp44 space.
// TR components are in plain integer units; multiplying by 4096 (= 1<<12)
// aligns them with products of two fp16 values (which carry 24 raw bits but
// represent real values scaled by 4096^2 / 4096 = 4096).
[[nodiscard]] constexpr fp44 tr_to_fp44(s32 tr) noexcept {
    return static_cast<fp44>(tr) << FP_SHIFT;
}

// Product of two fp16 values — result is in the fp44 (1.31.12) space.
// No shift is applied here; the shift by FP_SHIFT is done once at the end
// (via the sf flag) when storing to the MAC register.
[[nodiscard]] constexpr fp44 fp16_mul(fp16 a, fp16 b) noexcept {
    return static_cast<fp44>(a) * static_cast<fp44>(b);
}

// Apply the sf shift to collapse a fp44 accumulator to a MAC s32 value.
// sf=true: right-shift by 12 (result is in integer world units)
// sf=false: truncate to s32 (result retains 12 fractional bits, i.e. fp32)
[[nodiscard]] constexpr s32 apply_shift(fp44 v, bool sf) noexcept {
    return static_cast<s32>(sf ? (v >> FP_SHIFT) : v);
}

// Sign-extend an intermediate matrix-multiplication accumulator from 44 bits.
// The GTE hardware treats intermediate sums in matrix dot-products as 44-bit
// signed values: if a partial sum goes outside [-2^43, 2^43), it wraps around
// modulo 2^44 (not clamped).  This is DuckStation's "SignExtendN<44>".
//
// Overflow checking (flag bits) is a separate caller responsibility.
[[nodiscard]] constexpr fp44 mac_sign_extend(fp44 v) noexcept {
    // Take the lower 44 bits as a signed 44-bit integer.
    // Equivalent to: (v << 20) >> 20 (arithmetic), but avoids potential
    // undefined-behaviour on exotic platforms by using the XOR-subtract trick.
    constexpr fp44 MASK = (fp44{1} << 44) - 1;
    constexpr fp44 SIGN = fp44{1} << 43;
    return ((v & MASK) ^ SIGN) - SIGN;
}

// Unsigned Newton-Raphson reciprocal approximation for RTPS divide.
// Computes floor((h * 0x20000) / sz3) clamped to [0, 0x1FFFF].
// This matches the PSX hardware's unreal-integer-division algorithm.
[[nodiscard]] u32 gte_divide(u16 h, u16 sz3, u32& flag) noexcept;

} // namespace gte

// ── GTE class ─────────────────────────────────────────────────────────────────
class GTE {
public:
    // ── State (trivially copyable — suitable for save-state snapshots) ─────────
    // The two 32-element arrays directly map to the CP2d and CP2c register
    // files.  They are the sole persistent state of the coprocessor.
    struct Registers {
        std::array<u32, 32> d{};  // CP2 data  registers (MFC2/MTC2)
        std::array<u32, 32> c{};  // CP2 control registers (CFC2/CTC2)
    };
    static_assert(std::is_trivially_copyable_v<Registers>,
                  "GTE::Registers must be trivially copyable for save states");

    // ── Save-state interface ───────────────────────────────────────────────────
    [[nodiscard]] Registers save()                const noexcept { return regs_; }
    void                    restore(const Registers& r) noexcept { regs_ = r;   }

    // ── COP2 register bus (used by MFC2/MTC2/CFC2/CTC2) ──────────────────────
    [[nodiscard]] u32 read_data (u32 idx) const noexcept;
    void              write_data(u32 idx, u32 val) noexcept;
    [[nodiscard]] u32 read_ctrl (u32 idx) const noexcept;
    void              write_ctrl(u32 idx, u32 val) noexcept;

    // ── Command execution ─────────────────────────────────────────────────────
    // Dispatches the 25-bit GTE command word (instruction bits [24:0]).
    // pc is the address of the COP2 instruction; used only for diagnostics.
    void execute(u32 cmd, u32 pc) noexcept;

    // ── Diagnostic output ─────────────────────────────────────────────────────
    // Prints all 32 data + 32 control registers in a format that can be
    // compared line-by-line against DuckStation's GTE debug panel.
    // pc is the instruction address that triggered the dump.
    // out defaults to stderr; pass stdout for capture-friendly output.
    void dump_gte_state(u32 pc, std::FILE* out = stderr) const noexcept;

private:
    Registers regs_{};

    // ── FLAG register reference ───────────────────────────────────────────────
    [[nodiscard]] u32& flag()       noexcept { return regs_.c[31]; }
    [[nodiscard]] u32  flag() const noexcept { return regs_.c[31]; }

    // Recompute FLAG[31] (error summary) from the other flag bits.
    // Must be called at the end of every GTE command.
    void update_flag() noexcept {
        const bool error = (regs_.c[31] & Flag::ERROR_MASK) != 0u;
        regs_.c[31] = (regs_.c[31] & ~(1u << 31)) | (error ? (1u << 31) : 0u);
    }

    // ── Typed data-register accessors ─────────────────────────────────────────

    // Input vectors V0, V1, V2  (data[0..5])
    // Returns {VXn, VYn, VZn} as three fp16 values.
    [[nodiscard]] gte::Vec16 vec(int n) const noexcept;

    // IR0 (data[8]) — interpolation factor
    [[nodiscard]] gte::fp16 ir0() const noexcept {
        return static_cast<gte::fp16>(regs_.d[8]);
    }
    void set_ir0(s32 v) noexcept {
        regs_.d[8] = static_cast<u32>(gte::sat_ir0(v, flag()));
    }

    // IR1, IR2, IR3 (data[9..11]) — result vector; n in {1,2,3}.
    [[nodiscard]] gte::fp16 ir(u32 n) const noexcept {
        return static_cast<gte::fp16>(regs_.d[8u + n]);
    }
    void set_ir(u32 n, s32 v, bool lm) noexcept {
        regs_.d[8u + n] = static_cast<u32>(gte::sat_ir(v, static_cast<int>(n), lm, flag()));
    }

    // OTZ (data[7])
    [[nodiscard]] u16 otz() const noexcept { return static_cast<u16>(regs_.d[7]); }
    void set_otz(s32 v) noexcept { regs_.d[7] = gte::sat_sz(v, flag()); }

    // Screen XY FIFO: SXY0(d12), SXY1(d13), SXY2(d14). Index 0..2.
    [[nodiscard]] s16 sx(u32 n) const noexcept {
        return static_cast<s16>(regs_.d[12u + n]);
    }
    [[nodiscard]] s16 sy(u32 n) const noexcept {
        return static_cast<s16>(regs_.d[12u + n] >> 16);
    }
    // Push new (x,y) into the FIFO: [0]←[1], [1]←[2], [2]←new.
    // Saturates x to SX2_SAT range and y to SY2_SAT range.
    void push_sxy(s32 x, s32 y) noexcept {
        regs_.d[12] = regs_.d[13];
        regs_.d[13] = regs_.d[14];
        const s16 sx = gte::sat_sxy(x, Flag::SX2_SAT, flag());
        const s16 sy = gte::sat_sxy(y, Flag::SY2_SAT, flag());
        regs_.d[14] = (static_cast<u32>(static_cast<u16>(sx)))
                    | (static_cast<u32>(static_cast<u16>(sy)) << 16);
    }

    // Screen Z FIFO: SZ0(d16), SZ1(d17), SZ2(d18), SZ3(d19). Index 0..3.
    [[nodiscard]] u16 sz(u32 n) const noexcept {
        return static_cast<u16>(regs_.d[16u + n]);
    }
    // Push new Z into the FIFO: [0]←[1], [1]←[2], [2]←[3], [3]←new.
    void push_sz(s32 v) noexcept {
        regs_.d[16] = regs_.d[17];
        regs_.d[17] = regs_.d[18];
        regs_.d[18] = regs_.d[19];
        regs_.d[19] = gte::sat_sz(v, flag());
    }

    // Colour FIFO: RGB0(d20), RGB1(d21), RGB2(d22). Index 0..2.
    [[nodiscard]] u8 rgb_r(u32 n) const noexcept { return static_cast<u8>(regs_.d[20u+n]); }
    [[nodiscard]] u8 rgb_g(u32 n) const noexcept { return static_cast<u8>(regs_.d[20u+n] >> 8); }
    [[nodiscard]] u8 rgb_b(u32 n) const noexcept { return static_cast<u8>(regs_.d[20u+n] >> 16); }
    [[nodiscard]] u8 rgb_c(u32 n) const noexcept { return static_cast<u8>(regs_.d[20u+n] >> 24); }
    // Push new colour into FIFO: [0]←[1], [1]←[2], [2]←new.
    // CODE byte comes from RGBC (data[6] bits [31:24]).
    void push_rgb(s32 r, s32 g, s32 b) noexcept {
        const u8 code = static_cast<u8>(regs_.d[6] >> 24);
        regs_.d[20] = regs_.d[21];
        regs_.d[21] = regs_.d[22];
        const u8 sr = gte::sat_rgb(r, Flag::RGB_R_SAT, flag());
        const u8 sg = gte::sat_rgb(g, Flag::RGB_G_SAT, flag());
        const u8 sb = gte::sat_rgb(b, Flag::RGB_B_SAT, flag());
        regs_.d[22] = static_cast<u32>(sr)
                    | (static_cast<u32>(sg) << 8)
                    | (static_cast<u32>(sb) << 16)
                    | (static_cast<u32>(code) << 24);
    }

    // MAC0 (data[24]) — perspective/interpolation accumulator
    [[nodiscard]] s32 mac0() const noexcept { return std::bit_cast<s32>(regs_.d[24]); }
    void set_mac0(s64 v) noexcept {
        regs_.d[24] = std::bit_cast<u32>(gte::sat_mac0(v, flag()));
    }

    // MAC1, MAC2, MAC3 (data[25..27]) — X/Y/Z accumulators; n in {1,2,3}.
    [[nodiscard]] s32 mac(u32 n) const noexcept {
        return std::bit_cast<s32>(regs_.d[24u + n]);
    }
    // Set MACn from a 44-bit accumulator, applying sf shift first.
    // Overflow is detected on the RAW fp44 value (before any shift).
    // The hardware stores the raw (non-clamped) computation result truncated
    // to 32 bits — it does NOT clamp the stored value on overflow.
    void set_mac(u32 n, gte::fp44 v, bool sf) noexcept {
        // n=1→pos bit30/neg bit27, n=2→pos bit29/neg bit26, n=3→pos bit28/neg bit25
        const u32 pos_bit = 1u << (31u - n);
        const u32 neg_bit = 1u << (28u - n);
        if (v > gte::FP44_MAX) { flag() |= pos_bit; }
        if (v < gte::FP44_MIN) { flag() |= neg_bit; }
        regs_.d[24u + n] = static_cast<u32>(sf ? (v >> gte::FP_SHIFT) : v);
    }

    // ── Typed control-register accessors ──────────────────────────────────────

    // Helper: extract two packed s16 values from a u32.
    // The low half is 'a' and the high half is 'b'.
    [[nodiscard]] static gte::fp16 lo16(u32 r) noexcept { return static_cast<s16>(r & 0xFFFFu); }
    [[nodiscard]] static gte::fp16 hi16(u32 r) noexcept { return static_cast<s16>(r >> 16); }

    // Rotation matrix RT (ctrl[0..4]).
    // RT[row][col]:  M[0]={RT11,RT12,RT13}, M[1]={RT21,RT22,RT23}, M[2]={RT31,RT32,RT33}
    [[nodiscard]] gte::Mat3 rt() const noexcept;

    // Light matrix L (ctrl[8..12]) — same packing as RT.
    [[nodiscard]] gte::Mat3 lm() const noexcept;

    // Light-colour matrix LC (ctrl[16..20]) — same packing.
    [[nodiscard]] gte::Mat3 lcm() const noexcept;

    // Read any of the three 3×3 matrices by index (0=RT, 1=L, 2=LC).
    [[nodiscard]] gte::Mat3 matrix(int idx) const noexcept;

    // Translation vector TR (ctrl[5..7]) — plain s32 integer units.
    [[nodiscard]] gte::Vec32 tr() const noexcept {
        return { std::bit_cast<s32>(regs_.c[5]),
                 std::bit_cast<s32>(regs_.c[6]),
                 std::bit_cast<s32>(regs_.c[7]) };
    }
    // Background colour BK (ctrl[13..15]) — fp32 (1.19.12).
    [[nodiscard]] gte::Vec32 bk() const noexcept {
        return { std::bit_cast<s32>(regs_.c[13]),
                 std::bit_cast<s32>(regs_.c[14]),
                 std::bit_cast<s32>(regs_.c[15]) };
    }
    // Far colour FC (ctrl[21..23]) — fp32 (1.19.12).
    [[nodiscard]] gte::Vec32 fc() const noexcept {
        return { std::bit_cast<s32>(regs_.c[21]),
                 std::bit_cast<s32>(regs_.c[22]),
                 std::bit_cast<s32>(regs_.c[23]) };
    }
    // Translation vector by tx selector (MVMVA): 0=TR, 1=BK, 2=FC, 3=zero.
    [[nodiscard]] gte::Vec32 translation(u32 tx) const noexcept;

    // Scalar controls
    [[nodiscard]] s32 ofx()  const noexcept { return std::bit_cast<s32>(regs_.c[24]); }
    [[nodiscard]] s32 ofy()  const noexcept { return std::bit_cast<s32>(regs_.c[25]); }
    [[nodiscard]] u16 h()    const noexcept { return static_cast<u16>(regs_.c[26]); }
    [[nodiscard]] s16 dqa()  const noexcept { return static_cast<s16>(regs_.c[27]); }
    [[nodiscard]] s32 dqb()  const noexcept { return std::bit_cast<s32>(regs_.c[28]); }
    [[nodiscard]] s16 zsf3() const noexcept { return static_cast<s16>(regs_.c[29]); }
    [[nodiscard]] s16 zsf4() const noexcept { return static_cast<s16>(regs_.c[30]); }

    // RGBC (data[6]) component accessors
    [[nodiscard]] u8 rgbc_r()    const noexcept { return static_cast<u8>(regs_.d[6]); }
    [[nodiscard]] u8 rgbc_g()    const noexcept { return static_cast<u8>(regs_.d[6] >> 8); }
    [[nodiscard]] u8 rgbc_b()    const noexcept { return static_cast<u8>(regs_.d[6] >> 16); }
    [[nodiscard]] u8 rgbc_code() const noexcept { return static_cast<u8>(regs_.d[6] >> 24); }

    // Apply 44-bit overflow flag check then sign-extend an intermediate accumulator.
    // Matches DuckStation's SignExtendMACResult(index, value):
    //   CheckMACOverflow sets pos/neg flag bits if value is outside [-2^43, 2^43).
    //   SignExtendN<44> wraps the value modulo 2^44 (not clamped).
    // Used between accumulation terms in matrix dot-products of the lighting pipeline.
    // n is the MAC channel (1, 2, or 3) whose flag bits are set on overflow.
    gte::fp44 mac_se(gte::fp44 v, u32 n) noexcept {
        const u32 pos_bit = 1u << (31u - n);  // n=1→30, n=2→29, n=3→28
        const u32 neg_bit = 1u << (28u - n);  // n=1→27, n=2→26, n=3→25
        if (v > gte::FP44_MAX) flag() |= pos_bit;
        if (v < gte::FP44_MIN) flag() |= neg_bit;
        return gte::mac_sign_extend(v);
    }

    // Check 44-bit overflow flags on an intermediate accumulator WITHOUT sign-extension.
    // Used in the rotation pipeline (RTPS/RTPT) where intermediate values must not wrap,
    // but flag detection still occurs at each accumulation step (matching hardware).
    // n is the MAC channel (1, 2, or 3).
    gte::fp44 check_mac_flag(gte::fp44 v, u32 n) noexcept {
        const u32 pos_bit = 1u << (31u - n);
        const u32 neg_bit = 1u << (28u - n);
        if (v > gte::FP44_MAX) flag() |= pos_bit;
        if (v < gte::FP44_MIN) flag() |= neg_bit;
        return v;  // no sign-extension
    }

    // ── GTE command helpers ────────────────────────────────────────────────────
    // Each helper implements one GTE operation (or a reusable sub-operation).
    // Kept below ~50 lines each so execute() stays a thin dispatch table.

    // MVMVA: M × V + T → MAC1/2/3, IR1/2/3.
    // M, V, T selectors come from the command word (mx/vx/tx fields).
    void compute_mvmva(gte::Cmd c) noexcept;

    // RTPS core: rotate-translate-perspective for vertex v_idx ∈ {0,1,2}.
    // Used directly by RTPS (v_idx=0) and called three times by RTPT.
    // Writes MAC1-3, IR1-3, SXY2 (push), SZ3 (push), and (if last=true) MAC0, IR0.
    // For RTPT, MAC0/IR0 are only updated for the last vertex (V2).
    void compute_rtps(u32 v_idx, gte::Cmd c, bool last = true) noexcept;

    // Lighting pipeline sub-operations — shared by NCS/NCT/NCCS/NCCT/NCDS/NCDT.
    //
    // compute_ncs_core: step 1 (LLM*V → IR) + step 2 (BK + LCM*IR → IR).
    // compute_ncs:      ncs_core  + push_rgb(MAC>>4).
    // compute_nccs:     ncs_core  + step 3 (RGBC*IR)             + push_rgb.
    // compute_ncds:     ncs_core  + step 3 + step 4 (far-colour) + push_rgb.
    // compute_dpcs:     step 1' (RGB*IR0) + step 4 (far-colour)  + push_rgb.
    void compute_ncs_core(u32 vi, gte::Cmd c) noexcept;
    void compute_ncs     (u32 vi, gte::Cmd c) noexcept;
    void compute_nccs    (u32 vi, gte::Cmd c) noexcept;
    void compute_ncds    (u32 vi, gte::Cmd c) noexcept;
    void compute_dpcs    (s32 r, s32 g, s32 b, gte::Cmd c) noexcept;
};

// ── Compile-time fixed-point safety assertions ─────────────────────────────
// These fire at build time if the fixed-point type definitions or arithmetic
// ranges are ever broken by a refactor.  They document invariants the execute
// loop relies on.

// 1. The fp44 host type (s64) is strictly wider than the 44-bit logical range,
//    so that overflow IS detectable (the s64 never wraps around itself).
static_assert(gte::FP44_MAX  <  std::numeric_limits<s64>::max(),
    "FP44_MAX must be < s64_max so host-side overflow detection is reliable");
static_assert(gte::FP44_MIN  >  std::numeric_limits<s64>::min(),
    "FP44_MIN must be > s64_min so host-side overflow detection is reliable");

// 2. The logical boundary values are exactly the 44-bit signed extremes.
static_assert(gte::FP44_MAX == (s64{1} << 43) - 1, "FP44_MAX definition mismatch");
static_assert(gte::FP44_MIN == -(s64{1} << 43),     "FP44_MIN definition mismatch");

// 3. A single fp16 × fp16 raw product fits inside the fp44 logical range.
//    If this fails, fp16_mul() would immediately produce a value that looks
//    overflowed even without adding the translation or other terms.
static_assert(
    static_cast<gte::fp44>(std::numeric_limits<gte::fp16>::max())
        * std::numeric_limits<gte::fp16>::max()
    <= gte::FP44_MAX,
    "max fp16 × fp16 product exceeds FP44_MAX — accumulator overflow on single multiply");

// 4. The worst-case accumulator (maximum TR<<12 plus three maximum products)
//    fits inside s64 without host integer overflow.  The GTE 44-bit range may
//    still be exceeded (and the overflow flags set), but the C++ computation
//    itself must not produce UB via s64 wraparound.
//    max TR<<12  ≈ (2^31-1)*4096  ≈ 2^43
//    3 × max product ≈ 3 × (2^15-1)^2 ≈ 3 × 2^30  ≈ 2^32
//    total ≈ 2^43 + 2^32  <<  2^63-1
static_assert(
    gte::FP44_MAX + 3 * (static_cast<gte::fp44>(std::numeric_limits<gte::fp16>::max())
                         * std::numeric_limits<gte::fp16>::max())
    < std::numeric_limits<s64>::max(),
    "worst-case positive GTE accumulator overflows the s64 host type");
static_assert(
    gte::FP44_MIN - 3 * (static_cast<gte::fp44>(std::numeric_limits<gte::fp16>::max())
                         * std::numeric_limits<gte::fp16>::max())
    > std::numeric_limits<s64>::min(),
    "worst-case negative GTE accumulator underflows the s64 host type");

// 5. After applying the sf=1 right-shift, the clamped fp44 result always fits
//    in s32 — this justifies the static_cast<u32> in set_mac().
static_assert(gte::FP44_MAX >> gte::FP_SHIFT <= std::numeric_limits<s32>::max(),
    "FP44_MAX >> FP_SHIFT overflows s32 — sf=1 result would be truncated");
static_assert(gte::FP44_MIN >> gte::FP_SHIFT >= std::numeric_limits<s32>::min(),
    "FP44_MIN >> FP_SHIFT underflows s32 — sf=1 result would be truncated");

// 6. IR0's upper saturation bound [0, 0x1000] fits in fp16 (s16).
static_assert(0x1000 <= std::numeric_limits<gte::fp16>::max(),
    "IR0 upper clamp (0x1000) exceeds fp16 range — sat_ir0 would produce garbage");

// 7. The register file size is exactly what a save-state slot expects.
static_assert(sizeof(GTE::Registers) == 2 * 32 * sizeof(u32),
    "GTE::Registers size changed — save-state format may be broken");
