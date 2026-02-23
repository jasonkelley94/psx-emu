#include "GTE.hpp"

#include <bit>
#include <cstdio>

// ── gte::gte_divide ───────────────────────────────────────────────────────────
// Computes the GTE perspective-divide result N = H / SZ3 * 0x20000, using
// the hardware UNR (Newton-Raphson reciprocal) algorithm.
//
// Result is clamped to [0, 0x1FFFF].  DIV_OVF is set only for the early-out
// overflow case (SZ3*2 <= H); the final min() clamp does NOT set DIV_OVF.
//
// Algorithm matches DuckStation (verified against real hardware):
//   1. Early overflow:  if SZ3*2 <= H → DIV_OVF, return 0x1FFFF
//   2. Normalize:  shift = clz16(SZ3);  lhs = H << shift;  rhs = SZ3 << shift
//   3. divisor = rhs | 0x8000   (forces bit 15; already set after normalization
//                                 for non-zero SZ3; guards the zero case)
//   4. x      = 0x101 + unr_table[((divisor & 0x7FFF) + 0x40) >> 7]
//   5. d      = (divisor * -x  + 0x80) >> 8
//   6. recip  = (x * (0x20000 + d) + 0x80) >> 8
//   7. result = min(0x1FFFF, (lhs * recip + 0x8000) >> 16)
namespace gte {

// 257-entry UNR reciprocal LUT (hardware-accurate, from DuckStation).
// Indexed by ((divisor & 0x7FFF) + 0x40) >> 7, where divisor is the
// normalised SZ3 (bit 15 forced set, so index 0 → divisor=0x8000,
// index 256 → divisor=0xFFFF).
static const u8 kUNR_TABLE[257] = {
    0xFF,0xFD,0xFB,0xF9,0xF7,0xF5,0xF3,0xF1,0xEF,0xEE,0xEC,0xEA,0xE8,0xE6,0xE4,0xE3,
    0xE1,0xDF,0xDD,0xDC,0xDA,0xD8,0xD6,0xD5,0xD3,0xD1,0xD0,0xCE,0xCD,0xCB,0xC9,0xC8,
    0xC6,0xC5,0xC3,0xC1,0xC0,0xBE,0xBD,0xBB,0xBA,0xB8,0xB7,0xB5,0xB4,0xB2,0xB1,0xB0,
    0xAE,0xAD,0xAB,0xAA,0xA9,0xA7,0xA6,0xA4,0xA3,0xA2,0xA0,0x9F,0x9E,0x9C,0x9B,0x9A,
    0x99,0x97,0x96,0x95,0x94,0x92,0x91,0x90,0x8F,0x8D,0x8C,0x8B,0x8A,0x89,0x87,0x86,
    0x85,0x84,0x83,0x82,0x81,0x7F,0x7E,0x7D,0x7C,0x7B,0x7A,0x79,0x78,0x77,0x75,0x74,
    0x73,0x72,0x71,0x70,0x6F,0x6E,0x6D,0x6C,0x6B,0x6A,0x69,0x68,0x67,0x66,0x65,0x64,
    0x63,0x62,0x61,0x60,0x5F,0x5E,0x5D,0x5D,0x5C,0x5B,0x5A,0x59,0x58,0x57,0x56,0x55,
    0x54,0x53,0x53,0x52,0x51,0x50,0x4F,0x4E,0x4D,0x4D,0x4C,0x4B,0x4A,0x49,0x48,0x48,
    0x47,0x46,0x45,0x44,0x43,0x43,0x42,0x41,0x40,0x3F,0x3F,0x3E,0x3D,0x3C,0x3C,0x3B,
    0x3A,0x39,0x39,0x38,0x37,0x36,0x36,0x35,0x34,0x33,0x33,0x32,0x31,0x31,0x30,0x2F,
    0x2E,0x2E,0x2D,0x2C,0x2C,0x2B,0x2A,0x2A,0x29,0x28,0x28,0x27,0x26,0x26,0x25,0x24,
    0x24,0x23,0x22,0x22,0x21,0x20,0x20,0x1F,0x1E,0x1E,0x1D,0x1D,0x1C,0x1B,0x1B,0x1A,
    0x19,0x19,0x18,0x18,0x17,0x16,0x16,0x15,0x15,0x14,0x14,0x13,0x12,0x12,0x11,0x11,
    0x10,0x0F,0x0F,0x0E,0x0E,0x0D,0x0D,0x0C,0x0C,0x0B,0x0A,0x0A,0x09,0x09,0x08,0x08,
    0x07,0x07,0x06,0x06,0x05,0x05,0x04,0x04,0x03,0x03,0x02,0x02,0x01,0x01,0x00,0x00,
    0x00  // entry 256
};

u32 gte_divide(u16 h, u16 sz3, u32& flag) noexcept {
    const u32 lhs = static_cast<u32>(h);
    const u32 rhs = static_cast<u32>(sz3);

    // Early overflow: hardware sets DIV_OVF and returns max.
    if (rhs * 2u <= lhs) {
        flag |= Flag::DIV_OVF;
        return 0x1FFFFu;
    }

    // Normalise: shift both so the divisor has bit 15 set (range [0x8000, 0xFFFF]).
    // For sz3==0: clz of a u16 zero is 16; lhs becomes 0x10000, rhs stays 0 →
    // divisor = 0 | 0x8000 = 0x8000.
    const u32 shift   = (sz3 == 0u) ? 16u
                                     : static_cast<u32>(std::countl_zero(static_cast<u16>(sz3)));
    const u32 lhs_n   = lhs << shift;
    const u32 rhs_n   = rhs << shift;
    const u32 divisor = rhs_n | 0x8000u;

    // UNR Newton-Raphson step.
    const s32 x     = static_cast<s32>(0x101u + static_cast<u32>(kUNR_TABLE[((divisor & 0x7FFFu) + 0x40u) >> 7]));
    const s32 d     = ((static_cast<s32>(divisor) * -x) + 0x80) >> 8;
    const u32 recip = static_cast<u32>(((x * (0x20000 + d)) + 0x80) >> 8);

    const u32 result = static_cast<u32>(
        (static_cast<u64>(lhs_n) * static_cast<u64>(recip) + 0x8000u) >> 16u);

    // Final clamp — does NOT set DIV_OVF (early check above handles that).
    return std::min<u32>(0x1FFFFu, result);
}

} // namespace gte

// ── GTE::read_data ────────────────────────────────────────────────────────────
// Most data registers are a straightforward raw register read.
// Special cases with hardware-defined read behaviour:
//   reg 1,3,5  (VZ0,VZ1,VZ2) — s16 in [15:0], sign-extended to 32 bits
//   reg 7      (OTZ)          — u16 in [15:0], zero-extended to 32 bits
//   reg 8-11   (IR0-IR3)      — s16 in [15:0], sign-extended to 32 bits
//   reg 15     (SXYP)         — aliases SXY2 (register 14)
//   reg 16-19  (SZ0-SZ3)      — u16 in [15:0], zero-extended to 32 bits
//   reg 28     (IRGB)         — reads same as ORGB (reg 29)
//   reg 29     (ORGB)         — read-only; encodes IR1/2/3 as 5-bit RGB
//   reg 31     (LZCR)         — leading-zeros of LZCS (register 30)
u32 GTE::read_data(u32 idx) const noexcept {
    idx &= 31u;
    switch (idx) {
    // s16 registers — sign-extend lo16 to 32 bits on read
    case 1:  // VZ0
    case 3:  // VZ1
    case 5:  // VZ2
    case 8:  // IR0
    case 9:  // IR1
    case 10: // IR2
    case 11: // IR3
        return static_cast<u32>(static_cast<s32>(static_cast<s16>(regs_.d[idx])));

    // u16 registers — zero-extend lo16 to 32 bits on read
    case 7:  // OTZ
    case 16: // SZ0
    case 17: // SZ1
    case 18: // SZ2
    case 19: // SZ3
        return regs_.d[idx] & 0xFFFFu;

    case 15:  // SXYP aliases SXY2
        return regs_.d[14];

    case 28:  // IRGB — reads same as ORGB on real hardware
    case 29: {  // ORGB — encode IR1[14:7], IR2[14:7], IR3[14:7] as 5-bit channels
        // Saturate each IR value / 0x80 to 5 bits [0,31].
        const auto clamp5 = [](s16 ir) -> u32 {
            const s32 v = static_cast<s32>(ir) >> 7;
            return static_cast<u32>(std::clamp(v, 0, 31));
        };
        return clamp5(static_cast<s16>(regs_.d[9]))
             | (clamp5(static_cast<s16>(regs_.d[10])) << 5)
             | (clamp5(static_cast<s16>(regs_.d[11])) << 10);
    }

    case 31: {  // LZCR — count leading zeros (or leading ones if LZCS negative)
        const u32 lzcs = regs_.d[30];
        return (lzcs & 0x8000'0000u)
            ? static_cast<u32>(std::countl_one(lzcs))
            : static_cast<u32>(std::countl_zero(lzcs));
    }

    default:
        return regs_.d[idx];
    }
}

// ── GTE::write_data ───────────────────────────────────────────────────────────
// Special cases:
//   reg 15 (SXYP)  — pushes FIFO: SXY0←SXY1, SXY1←SXY2, SXY2←val
//   reg 28 (IRGB)  — decode 5-bit RGB and write into IR1, IR2, IR3
//   reg 29 (ORGB)  — read-only; writes are ignored
//   reg 30 (LZCS)  — store input value; LZCR (reg 31) is derived on read
//   reg 31 (LZCR)  — read-only; writes are ignored
void GTE::write_data(u32 idx, u32 val) noexcept {
    idx &= 31u;
    switch (idx) {
    case 15:  // SXYP — push into the screen-XY FIFO
        regs_.d[12] = regs_.d[13];
        regs_.d[13] = regs_.d[14];
        regs_.d[14] = val;
        break;

    case 28: {  // IRGB — decode 5-bit RGB → IR1, IR2, IR3 (scale by 0x80)
        // Each 5-bit channel is placed in the upper bits of the s16 IR register
        // as if it were multiplied by 0x80 (= left-shift 7).
        const auto expand = [](u32 channel) -> u32 {
            return static_cast<u32>(static_cast<s16>(static_cast<u16>((channel & 0x1Fu) << 7)));
        };
        regs_.d[9]  = expand(val);
        regs_.d[10] = expand(val >> 5);
        regs_.d[11] = expand(val >> 10);
        break;
    }

    case 29:  // ORGB — read-only
    case 31:  // LZCR — read-only
        break;

    case 30:  // LZCS — store raw; LZCR computed on next read of reg 31
        regs_.d[30] = val;
        break;

    default:
        regs_.d[idx] = val;
        break;
    }
}

// ── GTE::read_ctrl ────────────────────────────────────────────────────────────
// Most control registers return their stored value directly.
// Registers that contain a single s16 field sign-extend lo16 to 32 bits:
//   c[4]  RT33  — last element of rotation matrix
//   c[12] L33   — last element of light matrix
//   c[20] LB3   — last element of light-colour matrix
//   c[26] H     — projection distance (u16 but hardware sign-extends, a HW bug)
//   c[27] DQA   — depth-cue coefficient A (s16)
//   c[29] ZSF3  — Z scale factor for 3-point average (s16)
//   c[30] ZSF4  — Z scale factor for 4-point average (s16)
u32 GTE::read_ctrl(u32 idx) const noexcept {
    idx &= 31u;
    switch (idx) {
    case 4:  // RT33 (s16)
    case 12: // L33  (s16)
    case 20: // LB3  (s16)
    case 26: // H    (u16, but sign-extended by hardware)
    case 27: // DQA  (s16)
    case 29: // ZSF3 (s16)
    case 30: // ZSF4 (s16)
        return static_cast<u32>(static_cast<s32>(static_cast<s16>(regs_.c[idx])));

    default:
        return regs_.c[idx];
    }
}

// ── GTE::write_ctrl ───────────────────────────────────────────────────────────
// Most control registers are plain writes.  FLAG (reg 31) has two constraints:
//   • bits[11:0] are always 0 (hardware ignores writes to them)
//   • bit 31 (error summary) is computed; recomputed after any write
void GTE::write_ctrl(u32 idx, u32 val) noexcept {
    idx &= 31u;
    if (idx == 31u) {
        regs_.c[31] = val & ~0xFFFu;  // bits[11:0] always zero in FLAG
        update_flag();                  // recompute error-summary bit 31
        return;
    }
    regs_.c[idx] = val;
}

// ── Named accessor implementations ───────────────────────────────────────────

gte::Vec16 GTE::vec(int n) const noexcept {
    // n in {0,1,2}: VXYn is at data[2n], VZn at data[2n+1]
    const u32 xy = regs_.d[static_cast<u32>(n) * 2u];
    const u32 z  = regs_.d[static_cast<u32>(n) * 2u + 1u];
    return { static_cast<gte::fp16>(xy),
             static_cast<gte::fp16>(xy >> 16),
             static_cast<gte::fp16>(z) };
}

// Helper: read a 3×3 matrix from 5 consecutive control registers starting at
// base_idx.  Packing: reg+0=[R00,R01], reg+1=[R02,R10], reg+2=[R11,R12],
//                     reg+3=[R20,R21], reg+4=[R22, pad].
static gte::Mat3 read_matrix(const std::array<u32, 32>& c, u32 base) noexcept {
    gte::Mat3 m{};
    m[0][0] = static_cast<gte::fp16>(c[base+0]);       // R00 (lo)
    m[0][1] = static_cast<gte::fp16>(c[base+0] >> 16); // R01 (hi)
    m[0][2] = static_cast<gte::fp16>(c[base+1]);       // R02 (lo)
    m[1][0] = static_cast<gte::fp16>(c[base+1] >> 16); // R10 (hi)
    m[1][1] = static_cast<gte::fp16>(c[base+2]);       // R11 (lo)
    m[1][2] = static_cast<gte::fp16>(c[base+2] >> 16); // R12 (hi)
    m[2][0] = static_cast<gte::fp16>(c[base+3]);       // R20 (lo)
    m[2][1] = static_cast<gte::fp16>(c[base+3] >> 16); // R21 (hi)
    m[2][2] = static_cast<gte::fp16>(c[base+4]);       // R22 (lo)
    return m;
}

gte::Mat3 GTE::rt()  const noexcept { return read_matrix(regs_.c,  0u); }
gte::Mat3 GTE::lm()  const noexcept { return read_matrix(regs_.c,  8u); }
gte::Mat3 GTE::lcm() const noexcept { return read_matrix(regs_.c, 16u); }

gte::Mat3 GTE::matrix(int idx) const noexcept {
    switch (idx) {
    case 0:  return rt();
    case 1:  return lm();
    case 2:  return lcm();
    default: {
        // mx=3 is "undefined" — hardware reads from wrong register banks, producing
        // a garbage matrix built from RGBC, IR0, RT13, and RT22.
        // Matches DuckStation's verified implementation:
        //   row0: { -(R<<4), +(R<<4), IR0 }  where R = RGBC red byte (u8)
        //   row1: { RT13, RT13, RT13 }        (lo16 of ctrl[1])
        //   row2: { RT22, RT22, RT22 }        (lo16 of ctrl[2])
        const gte::fp16 r_scaled = static_cast<gte::fp16>(
            static_cast<u16>(rgbc_r()) << 4);  // always fits: max R=255, 255<<4=4080
        const gte::fp16 rt13 = lo16(regs_.c[1]);
        const gte::fp16 rt22 = lo16(regs_.c[2]);
        return {{ { static_cast<gte::fp16>(-r_scaled), r_scaled, ir0() },
                  { rt13, rt13, rt13 },
                  { rt22, rt22, rt22 } }};
    }
    }
}

gte::Vec32 GTE::translation(u32 tx) const noexcept {
    switch (tx) {
    case 0: return tr();
    case 1: return bk();
    case 2: return fc();
    default: return {};  // tx=3 → zero vector
    }
}

// ── GTE::compute_mvmva ────────────────────────────────────────────────────────
// MVMVA — Matrix-Vector Multiply + Add.
//
// For each output row i ∈ {0,1,2}:
//   fp44 acc = (T[i] << 12)  +  M[i][0]*V[0]  +  M[i][1]*V[1]  +  M[i][2]*V[2]
//   MAC[i+1] = acc >> sf_shift  (44-bit overflow flagged before the shift)
//   IR[i+1]  = saturate(MAC[i+1], lm)
//
// Operand selectors are decoded from the command word:
//   mx=0 → RT,  mx=1 → Light,  mx=2 → LightColor,  mx=3 → (undefined)
//   vx=0 → V0,  vx=1 → V1,    vx=2 → V2,            vx=3 → {IR1,IR2,IR3}
//   tx=0 → TR,  tx=1 → BK,    tx=2 → FC,             tx=3 → zero
void GTE::compute_mvmva(gte::Cmd c) noexcept {
    const gte::Mat3  M = matrix(static_cast<int>(c.mx()));

    // vx=3 selects the IR1/IR2/IR3 result vector instead of an input vertex.
    gte::Vec16 V{};
    if (c.vx() < 3u) {
        V = vec(static_cast<int>(c.vx()));
    } else {
        V = { ir(1u), ir(2u), ir(3u) };
    }

    if (c.tx() == 2u) {
        // MVMVA tx=FC hardware bug — two-phase computation:
        //
        // Phase 1: (FC[i] << 12) + M[i][0]*V[0]
        //   Hardware performs this step and checks for 44-bit overflow (sets
        //   MAC1/2/3 flags), but the intermediate value is discarded — it is
        //   NOT stored in MAC and does NOT update IR saturation flags.
        //
        // Phase 2: M[i][1]*V[1] + M[i][2]*V[2]
        //   This is the actual result, stored in MAC and IR normally.
        const gte::Vec32 FC = fc();
        for (u32 i = 0; i < 3u; ++i) {
            const u32 n = i + 1u;
            // Phase 1: (FC[i]<<12) + M[i][0]*V[0].
            // Hardware checks 44-bit overflow (MAC flags) and then shifts to
            // get an intermediate IR — the IR saturation flags are set here
            // even though the final IR comes from phase 2.
            const gte::fp44 acc1 = mac_se(gte::tr_to_fp44(FC[i])
                                        + gte::fp16_mul(M[i][0], V[0]), n);
            // Do NOT clamp before truncation — use the wrapped acc1 bits, just
            // like the hardware: the overflow flag was already set by mac_se.
            const s32 ir1 = static_cast<s32>(c.sf() ? (acc1 >> gte::FP_SHIFT)
                                                     : acc1);
            set_ir(n, ir1, false);  // sets IR saturation flags if ir1 out of range

            // Phase 2: actual stored result — overwrites IR, flags accumulated.
            // Intermediate overflow check+SE between the two terms (DuckStation pattern).
            const gte::fp44 acc2 = mac_se(gte::fp16_mul(M[i][1], V[1]), n)
                                 + gte::fp16_mul(M[i][2], V[2]);
            set_mac(n, acc2, c.sf());
            set_ir (n, mac(n), c.lm());
        }
    } else {
        // Normal translation path (tx = TR, BK, or zero).
        // Two nested flag-check+SE calls match DuckStation's MulMatVec with translation.
        const gte::Vec32 T = translation(c.tx());
        for (u32 i = 0; i < 3u; ++i) {
            const u32 n = i + 1u;
            const gte::fp44 acc =
                mac_se(mac_se(gte::tr_to_fp44(T[i])
                            + gte::fp16_mul(M[i][0], V[0]), n)
                     + gte::fp16_mul(M[i][1], V[1]), n)
                + gte::fp16_mul(M[i][2], V[2]);
            set_mac(n, acc, c.sf());
            set_ir (n, mac(n), c.lm());
        }
    }
}

// ── GTE::compute_rtps ─────────────────────────────────────────────────────────
// Perspective transformation for one vertex (v_idx ∈ {0,1,2}).
//
// Step 1 — Rotate + translate (RT matrix, TR vector, lm=false):
//   fp44 accN = (TR[N] << 12) + RT[N][0]*V[0] + RT[N][1]*V[1] + RT[N][2]*V[2]
//   MAC1/2/3, IR1/2/3 set from acc0/1/2.
//
// Step 2 — Screen Z:
//   SZ3 ← Lim(acc3 >> 12, 0..FFFFh, SZ3_SAT)   [always >>12, independent of sf]
//
// Step 3 — Perspective divide:
//   N = gte_divide(H, SZ3)                        [≈ H/SZ3 * 131072]
//
// Step 4 — Screen XY:
//   SX2,SY2 ← Lim((N*IRn + OFn) >> 16, ±3FFh, SXn_SAT)
//
// Step 5 — Depth-cue coefficient:
//   MAC0 = N*DQA + DQB,   IR0 = Lim(MAC0 >> 12, 0..1000h)
void GTE::compute_rtps(u32 vi, gte::Cmd c, bool last) noexcept {
    const gte::Mat3  M  = rt();
    const gte::Vec32 T  = tr();
    const gte::Vec16 V  = vec(static_cast<int>(vi));
    const bool       sf = c.sf();

    // Step 1: rows 0 and 1 (X, Y components).
    // Intermediate mac_se: check overflow flags AND sign-extend, matching hardware behavior.
    for (u32 i = 0; i < 2u; ++i) {
        const u32 n = i + 1u;
        const gte::fp44 acc =
            mac_se(mac_se(gte::tr_to_fp44(T[i])
                        + gte::fp16_mul(M[i][0], V[0]), n)
                 + gte::fp16_mul(M[i][1], V[1]), n)
            + gte::fp16_mul(M[i][2], V[2]);
        set_mac(n, acc, sf);
        set_ir (n, mac(n), c.lm());
    }

    // Row 2 (Z component) — intermediate mac_se for flag detection and hardware accuracy.
    const gte::fp44 acc3 =
        mac_se(mac_se(gte::tr_to_fp44(T[2])
                    + gte::fp16_mul(M[2][0], V[0]), 3u)
             + gte::fp16_mul(M[2][1], V[1]), 3u)
        + gte::fp16_mul(M[2][2], V[2]);
    set_mac(3u, acc3, sf);

    // SZ3 always uses the >>12-shifted accumulator, independent of sf.
    const s32 acc3_shifted = static_cast<s32>(acc3 >> gte::FP_SHIFT);

    // IR3 in RTPS:
    //   FLAG check uses acc3>>12 (the depth-shifted value, independent of sf).
    //   Stored IR3 uses MAC3 (the stored value: acc3>>12 for sf=1, or bottom-32
    //   of acc3 for sf=0).  Saturation flag always checks lm=false range.
    //   IR3 storage is clamped with actual c.lm().
    {
        if (acc3_shifted < -0x8000 || acc3_shifted > 0x7FFF) { flag() |= Flag::IR3_SAT; }
        const s32 lo    = c.lm() ? 0 : -0x8000;
        const s32 ir3_v = mac(3u);  // sf-dependent stored value
        regs_.d[11] = static_cast<u32>(
            static_cast<gte::fp16>(std::clamp(ir3_v, lo, 0x7FFF)));
    }

    // Step 2: SZ3 always uses the >>12-shifted accumulator (independent of sf).
    push_sz(acc3_shifted);

    // Step 3: perspective divide N = floor((H * 0x20000) / SZ3).
    const u32 N = gte::gte_divide(h(), sz(3u), flag());

    // Step 4: screen XY = (N * IR + Offset) >> 16, clamped to 10-bit.
    // The hardware also checks MAC0 overflow on each of the two screen coords.
    const s64 sx64 = static_cast<s64>(N) * static_cast<s64>(ir(1u)) + static_cast<s64>(ofx());
    const s64 sy64 = static_cast<s64>(N) * static_cast<s64>(ir(2u)) + static_cast<s64>(ofy());
    (void)gte::sat_mac0(sx64, flag());
    (void)gte::sat_mac0(sy64, flag());
    push_sxy(static_cast<s32>(sx64 >> 16), static_cast<s32>(sy64 >> 16));

    // Step 5: depth-cue coefficient → MAC0, IR0.
    // For RTPT, this step is only performed for the LAST vertex (V2);
    // skipping it for V0/V1 prevents contaminating IR0_SAT/MAC0 flags.
    if (last) {
        const s64 mac0_raw =
            static_cast<s64>(N) * static_cast<s64>(dqa()) + static_cast<s64>(dqb());
        set_mac0(mac0_raw);
        set_ir0(static_cast<s32>(mac0_raw >> gte::FP_SHIFT));
    }
}

// ── GTE::dump_gte_state ───────────────────────────────────────────────────────
// Prints all 64 GTE registers in a format compatible with DuckStation's GTE
// debug panel, enabling direct line-by-line comparison.
//
// Data register names follow the PSX hardware manual:
//   D00 VXY0, D01 VZ0, D02 VXY1, D03 VZ1, D04 VXY2, D05 VZ2
//   D06 RGBC, D07 OTZ,  D08 IR0 … D11 IR3
//   D12 SXY0 … D15 SXYP, D16 SZ0 … D19 SZ3
//   D20 RGB0 … D23 RES1, D24 MAC0 … D27 MAC3
//   D28 IRGB, D29 ORGB,  D30 LZCS, D31 LZCR
// Control register names:
//   C00 RT1112 … C04 RT33, C05 TRX … C07 TRZ
//   C08 L1112  … C12 L33,  C13 RBK … C15 BBK
//   C16 LR1R2  … C20 LB3,  C21 RFC … C23 BFC
//   C24 OFX,  C25 OFY,  C26 H,   C27 DQA
//   C28 DQB,  C29 ZSF3, C30 ZSF4, C31 FLAG
//
// read_data() is used so that computed registers (ORGB, LZCR) show the value
// software would actually receive, matching the debugger output.
void GTE::dump_gte_state(u32 pc, std::FILE* out) const noexcept {
    static constexpr std::array<const char*, 32> kD = {
        "VXY0 ","VZ0  ","VXY1 ","VZ1  ","VXY2 ","VZ2  ","RGBC ","OTZ  ",
        "IR0  ","IR1  ","IR2  ","IR3  ","SXY0 ","SXY1 ","SXY2 ","SXYP ",
        "SZ0  ","SZ1  ","SZ2  ","SZ3  ","RGB0 ","RGB1 ","RGB2 ","RES1 ",
        "MAC0 ","MAC1 ","MAC2 ","MAC3 ","IRGB ","ORGB ","LZCS ","LZCR ",
    };
    static constexpr std::array<const char*, 32> kC = {
        "RT1112","RT1321","RT2223","RT3132","RT33  ",
        "TRX   ","TRY   ","TRZ   ",
        "L1112 ","L1321 ","L2223 ","L3132 ","L33   ",
        "RBK   ","GBK   ","BBK   ",
        "LR1R2 ","LR3G1 ","LG2G3 ","LB1B2 ","LB3   ",
        "RFC   ","GFC   ","BFC   ",
        "OFX   ","OFY   ","H     ","DQA   ","DQB   ",
        "ZSF3  ","ZSF4  ","FLAG  ",
    };

    std::fprintf(out, "GTE @ PC 0x%08X\n", pc);
    std::fprintf(out, "── CP2 Data ──────────────────────────────────────────────────\n");
    for (u32 i = 0; i < 32u; i += 2) {
        std::fprintf(out, "  D%02u %s= %08X    D%02u %s= %08X\n",
                     i,   kD[i],   read_data(i),
                     i+1, kD[i+1], read_data(i+1));
    }
    std::fprintf(out, "── CP2 Control ───────────────────────────────────────────────\n");
    for (u32 i = 0; i < 32u; i += 2) {
        std::fprintf(out, "  C%02u %s= %08X    C%02u %s= %08X\n",
                     i,   kC[i],   read_ctrl(i),
                     i+1, kC[i+1], read_ctrl(i+1));
    }
    // Decode active FLAG bits so they don't have to be looked up manually.
    const u32 f = read_ctrl(31);
    if (f) {
        std::fprintf(out, "  FLAG bits:");
        if (f & Flag::MAC1_POS) std::fprintf(out, " MAC1+");
        if (f & Flag::MAC1_NEG) std::fprintf(out, " MAC1-");
        if (f & Flag::MAC2_POS) std::fprintf(out, " MAC2+");
        if (f & Flag::MAC2_NEG) std::fprintf(out, " MAC2-");
        if (f & Flag::MAC3_POS) std::fprintf(out, " MAC3+");
        if (f & Flag::MAC3_NEG) std::fprintf(out, " MAC3-");
        if (f & Flag::IR1_SAT)  std::fprintf(out, " IR1_SAT");
        if (f & Flag::IR2_SAT)  std::fprintf(out, " IR2_SAT");
        if (f & Flag::IR3_SAT)  std::fprintf(out, " IR3_SAT");
        if (f & Flag::RGB_R_SAT) std::fprintf(out, " RGB_R");
        if (f & Flag::RGB_G_SAT) std::fprintf(out, " RGB_G");
        if (f & Flag::RGB_B_SAT) std::fprintf(out, " RGB_B");
        if (f & Flag::MAC0_POS) std::fprintf(out, " MAC0+");
        if (f & Flag::MAC0_NEG) std::fprintf(out, " MAC0-");
        if (f & Flag::SX2_SAT)  std::fprintf(out, " SX2");
        if (f & Flag::SY2_SAT)  std::fprintf(out, " SY2");
        if (f & Flag::SZ3_SAT)  std::fprintf(out, " SZ3");
        if (f & Flag::DIV_OVF)  std::fprintf(out, " DIV_OVF");
        if (f & (1u << 31))     std::fprintf(out, " ERROR");
        std::fprintf(out, "\n");
    }
}

// ── GTE lighting pipeline helpers ────────────────────────────────────────────
//
// The PSX lighting pipeline consists of up to four sequential steps:
//
//   Step 1 — diffuse intensity:    [MAC1,2,3] = (LLM * V[vi])        SAR (sf*12)
//                                  [IR1,2,3]  = [MAC1,2,3]            (lm clamp)
//   Step 2 — colour matrix + BK:   [MAC1,2,3] = (BK*1000h + LCM*IR)  SAR (sf*12)
//                                  [IR1,2,3]  = [MAC1,2,3]            (lm clamp)
//   Step 3 — source-colour scale:  [MAC1,2,3] = [R*IR1, G*IR2, B*IR3] SAR (sf*12)
//   Step 4 — far-colour blend:     [MAC1,2,3] = ((FC - MAC)*IR0 + MAC*1000h) SAR (sf*12)
//   Final  — push colour FIFO:     push_rgb(MAC1>>4, MAC2>>4, MAC3>>4)
//                                  [IR1,2,3] = [MAC1,2,3]

// ── compute_ncs_core: steps 1 and 2 ──────────────────────────────────────────
void GTE::compute_ncs_core(u32 vi, gte::Cmd c) noexcept {
    // Step 1: diffuse intensity — LLM × V[vi], no translation.
    // Hardware uses intermediate 44-bit overflow check + sign-extension between
    // dot-product terms (DuckStation: SignExtendMACResult between pair accumulations).
    const gte::Mat3  L = lm();
    const gte::Vec16 V = vec(static_cast<int>(vi));
    for (u32 i = 0; i < 3u; ++i) {
        const u32 n = i + 1u;
        const gte::fp44 acc =
            mac_se(gte::fp16_mul(L[i][0], V[0])
                 + gte::fp16_mul(L[i][1], V[1]), n)
            + gte::fp16_mul(L[i][2], V[2]);
        set_mac(n, acc, c.sf());
        set_ir (n, mac(n), c.lm());
    }

    // Step 2: colour matrix × diffuse IR + background colour.
    // Translation version: two nested flag-check+SE calls (DuckStation pattern).
    // IR1/2/3 are captured before the loop so each row uses the step-1 IR outputs,
    // not the IR values updated by previous rows of this step.
    const gte::Mat3  C = lcm();
    const gte::Vec32 B = bk();
    const gte::fp16 ir1 = ir(1u), ir2 = ir(2u), ir3 = ir(3u);
    for (u32 i = 0; i < 3u; ++i) {
        const u32 n = i + 1u;
        const gte::fp44 acc =
            mac_se(mac_se(gte::tr_to_fp44(B[i])
                        + gte::fp16_mul(C[i][0], ir1), n)
                 + gte::fp16_mul(C[i][1], ir2), n)
            + gte::fp16_mul(C[i][2], ir3);
        set_mac(n, acc, c.sf());
        set_ir (n, mac(n), c.lm());
    }
}

// ── compute_ncs: steps 1 + 2 + colour push ───────────────────────────────────
void GTE::compute_ncs(u32 vi, gte::Cmd c) noexcept {
    compute_ncs_core(vi, c);
    push_rgb(mac(1u) >> 4, mac(2u) >> 4, mac(3u) >> 4);
}

// ── compute_nccs: steps 1 + 2 + 3 + colour push ──────────────────────────────
void GTE::compute_nccs(u32 vi, gte::Cmd c) noexcept {
    compute_ncs_core(vi, c);

    // Step 3: source-colour scale — [R*IR1,G*IR2,B*IR3] SHL 4 SAR (sf*12).
    set_mac(1u, gte::fp16_mul(static_cast<gte::fp16>(rgbc_r()), ir(1u)) << 4, c.sf());
    set_mac(2u, gte::fp16_mul(static_cast<gte::fp16>(rgbc_g()), ir(2u)) << 4, c.sf());
    set_mac(3u, gte::fp16_mul(static_cast<gte::fp16>(rgbc_b()), ir(3u)) << 4, c.sf());
    set_ir(1u, mac(1u), c.lm());
    set_ir(2u, mac(2u), c.lm());
    set_ir(3u, mac(3u), c.lm());
    push_rgb(mac(1u) >> 4, mac(2u) >> 4, mac(3u) >> 4);
}

// ── compute_ncds: steps 1 + 2 + 3 + 4 + colour push ─────────────────────────
void GTE::compute_ncds(u32 vi, gte::Cmd c) noexcept {
    compute_ncs_core(vi, c);

    // Step 3: source-colour scale — (R*IR1)<<4 etc.
    // Hardware computes this as a raw value (no MAC truncation, no sf shift).
    // The result feeds directly into InterpolateColor as the "previous MAC".
    // DuckStation: in_MAC = (s32(R) * s32(IR)) << 4  (no TruncateAndSetMAC call).
    const gte::fp44 in_mac1 = static_cast<gte::fp44>(
        static_cast<s32>(rgbc_r()) * static_cast<s32>(ir(1u))) << 4;
    const gte::fp44 in_mac2 = static_cast<gte::fp44>(
        static_cast<s32>(rgbc_g()) * static_cast<s32>(ir(2u))) << 4;
    const gte::fp44 in_mac3 = static_cast<gte::fp44>(
        static_cast<s32>(rgbc_b()) * static_cast<s32>(ir(3u))) << 4;

    // Step 4: InterpolateColor — two-step far-colour blend.
    const gte::Vec32 F = fc();
    // Step 4a: (FC << 12) - in_MAC, IR with lm=false.
    set_mac(1u, static_cast<gte::fp44>(F[0]) * gte::fp44(0x1000) - in_mac1, c.sf());
    set_mac(2u, static_cast<gte::fp44>(F[1]) * gte::fp44(0x1000) - in_mac2, c.sf());
    set_mac(3u, static_cast<gte::fp44>(F[2]) * gte::fp44(0x1000) - in_mac3, c.sf());
    set_ir(1u, mac(1u), false);
    set_ir(2u, mac(2u), false);
    set_ir(3u, mac(3u), false);
    // Step 4b: IR * IR0 + in_MAC (pre-step-4 value), IR with cmd lm.
    set_mac(1u, static_cast<gte::fp44>(ir(1u)) * static_cast<gte::fp44>(ir0()) + in_mac1, c.sf());
    set_mac(2u, static_cast<gte::fp44>(ir(2u)) * static_cast<gte::fp44>(ir0()) + in_mac2, c.sf());
    set_mac(3u, static_cast<gte::fp44>(ir(3u)) * static_cast<gte::fp44>(ir0()) + in_mac3, c.sf());
    set_ir(1u, mac(1u), c.lm());
    set_ir(2u, mac(2u), c.lm());
    set_ir(3u, mac(3u), c.lm());
    push_rgb(mac(1u) >> 4, mac(2u) >> 4, mac(3u) >> 4);
}

// ── compute_dpcs: depth-cue a colour toward the far colour ───────────────────
// r, g, b are raw 8-bit colour values (0–255) from a colour register.
//
// Step 1:  [MAC1,2,3] = [R,G,B] << 16            (stored with shift=0)
// Then InterpolateColor([MAC1,2,3], sf, lm):
//   Step a: [MAC1,2,3] = (FC*0x1000 - in_MAC)    SAR (sf*12), IR with lm=false
//   Step b: [MAC1,2,3] = IR*IR0 + in_MAC          SAR (sf*12), IR with cmd lm
// push_rgb
void GTE::compute_dpcs(s32 r, s32 g, s32 b, gte::Cmd c) noexcept {
    // Step 1: MAC = Color << 16, stored with shift=0 (DuckStation: shift arg=0).
    set_mac(1u, static_cast<gte::fp44>(r) << 16, false);
    set_mac(2u, static_cast<gte::fp44>(g) << 16, false);
    set_mac(3u, static_cast<gte::fp44>(b) << 16, false);

    // Snapshot in_MAC — the original MAC values from step 1.
    // Used unchanged in both step a and step b of InterpolateColor.
    const s32 in_mac1 = mac(1u);
    const s32 in_mac2 = mac(2u);
    const s32 in_mac3 = mac(3u);

    // InterpolateColor step a: (FC * 0x1000) - in_MAC, IR with lm=false.
    const gte::Vec32 F = fc();
    set_mac(1u, static_cast<gte::fp44>(F[0]) * gte::fp44(0x1000) - static_cast<gte::fp44>(in_mac1), c.sf());
    set_mac(2u, static_cast<gte::fp44>(F[1]) * gte::fp44(0x1000) - static_cast<gte::fp44>(in_mac2), c.sf());
    set_mac(3u, static_cast<gte::fp44>(F[2]) * gte::fp44(0x1000) - static_cast<gte::fp44>(in_mac3), c.sf());
    set_ir(1u, mac(1u), false);
    set_ir(2u, mac(2u), false);
    set_ir(3u, mac(3u), false);

    // InterpolateColor step b: IR * IR0 + in_MAC (original), IR with cmd lm.
    set_mac(1u, static_cast<gte::fp44>(ir(1u)) * static_cast<gte::fp44>(ir0()) + static_cast<gte::fp44>(in_mac1), c.sf());
    set_mac(2u, static_cast<gte::fp44>(ir(2u)) * static_cast<gte::fp44>(ir0()) + static_cast<gte::fp44>(in_mac2), c.sf());
    set_mac(3u, static_cast<gte::fp44>(ir(3u)) * static_cast<gte::fp44>(ir0()) + static_cast<gte::fp44>(in_mac3), c.sf());
    set_ir(1u, mac(1u), c.lm());
    set_ir(2u, mac(2u), c.lm());
    set_ir(3u, mac(3u), c.lm());
    push_rgb(mac(1u) >> 4, mac(2u) >> 4, mac(3u) >> 4);
}

// ── GTE::execute ─────────────────────────────────────────────────────────────
// Dispatch table for the 6-bit function code embedded in the command word.
// pc is the address of the executing COP2 instruction (diagnostic only).
void GTE::execute(u32 cmd, u32 pc) noexcept {
    // Clear FLAG register at the start of every command (hardware behaviour).
    regs_.c[31] = 0u;

    const gte::Cmd c{cmd};

    switch (c.func()) {
    // ── Perspective transformation ─────────────────────────────────────────
    case 0x01: /* RTPS  — perspective transform of V0         */
        compute_rtps(0u, c);
        break;

    case 0x12: /* MVMVA — matrix-vector multiply+add           */
        compute_mvmva(c);
        break;

    case 0x30: /* RTPT  — perspective transform triple (V0,V1,V2) */
        // Each compute_rtps call sets MAC0 (depth-cue) and OTZ = A(MAC0>>12).
        // Depth-cue (MAC0/IR0) is only computed for the last vertex (V2).
        // V0 and V1 skip the depth-cue step to avoid contaminating MAC0/IR0_SAT.
        compute_rtps(0u, c, false);
        compute_rtps(1u, c, false);
        compute_rtps(2u, c, true);
        break;

    case 0x06: /* NCLIP — normal clip (winding-order cross product) */
        // MAC0 = SX0*(SY1-SY2) + SX1*(SY2-SY0) + SX2*(SY0-SY1)
        // Positive → front-facing; negative → back-facing.
        {
            const s64 m0 =
                  static_cast<s64>(sx(0u)) * (static_cast<s32>(sy(1u)) - static_cast<s32>(sy(2u)))
                + static_cast<s64>(sx(1u)) * (static_cast<s32>(sy(2u)) - static_cast<s32>(sy(0u)))
                + static_cast<s64>(sx(2u)) * (static_cast<s32>(sy(0u)) - static_cast<s32>(sy(1u)));
            set_mac0(m0);
        }
        break;

    case 0x2D: /* AVSZ3 — average of three screen-Z values (SZ1, SZ2, SZ3) */
        // MAC0 = ZSF3 * (SZ1 + SZ2 + SZ3)
        // OTZ  = full-precision m0 >> 12  (clamped to [0, 0xFFFF])
        {
            const s64 m0 = static_cast<s64>(zsf3())
                * (static_cast<s32>(sz(1u)) + static_cast<s32>(sz(2u)) + static_cast<s32>(sz(3u)));
            set_mac0(m0);
            set_otz(static_cast<s32>(m0 >> gte::FP_SHIFT));
        }
        break;

    case 0x2E: /* AVSZ4 — average of four screen-Z values (SZ0, SZ1, SZ2, SZ3) */
        // MAC0 = ZSF4 * (SZ0 + SZ1 + SZ2 + SZ3)
        // OTZ  = full-precision m0 >> 12  (clamped to [0, 0xFFFF])
        {
            const s64 m0 = static_cast<s64>(zsf4())
                * (static_cast<s32>(sz(0u)) + static_cast<s32>(sz(1u))
                +  static_cast<s32>(sz(2u)) + static_cast<s32>(sz(3u)));
            set_mac0(m0);
            set_otz(static_cast<s32>(m0 >> gte::FP_SHIFT));
        }
        break;

    // ── Sub-task A: simple arithmetic ──────────────────────────────────────────

    case 0x28: /* SQR — square IR1/2/3 → MAC1/2/3 → IR1/2/3 */
        for (u32 n = 1u; n <= 3u; ++n) {
            set_mac(n, gte::fp16_mul(ir(n), ir(n)), c.sf());
            set_ir (n, mac(n), c.lm());
        }
        break;

    case 0x0C: { /* OP — outer product of RT diagonal × IR */
        // RT diagonal: RT11=lo(c[0]), RT22=lo(c[2]), RT33=lo(c[4])
        const gte::fp16 r11 = lo16(regs_.c[0u]);
        const gte::fp16 r22 = lo16(regs_.c[2u]);
        const gte::fp16 r33 = lo16(regs_.c[4u]);
        set_mac(1u, gte::fp16_mul(ir(3u), r22) - gte::fp16_mul(ir(2u), r33), c.sf());
        set_mac(2u, gte::fp16_mul(ir(1u), r33) - gte::fp16_mul(ir(3u), r11), c.sf());
        set_mac(3u, gte::fp16_mul(ir(2u), r11) - gte::fp16_mul(ir(1u), r22), c.sf());
        set_ir(1u, mac(1u), c.lm());
        set_ir(2u, mac(2u), c.lm());
        set_ir(3u, mac(3u), c.lm());
        break;
    }

    // ── Sub-task B: normal-colour pipeline ─────────────────────────────────────

    case 0x1E: /* NCS  — normal colour single  (V0)       */
        compute_ncs(0u, c);
        break;

    case 0x20: /* NCT  — normal colour triple  (V0,V1,V2) */
        compute_ncs(0u, c);
        compute_ncs(1u, c);
        compute_ncs(2u, c);
        break;

    // ── Sub-task C: normal-colour-colour ───────────────────────────────────────

    case 0x1B: /* NCCS — normal colour colour single  (V0) */
        compute_nccs(0u, c);
        break;

    case 0x3F: /* NCCT — normal colour colour triple (V0,V1,V2) */
        compute_nccs(0u, c);
        compute_nccs(1u, c);
        compute_nccs(2u, c);
        break;

    case 0x1C: { /* CC — colour colour (step 2 + step 3, no normal multiply) */
        // Step 2: BK + LCM × current IR → MAC/IR.
        // Two nested flag-check+SE calls (translation + 3 products, DuckStation pattern).
        // IR captured before loop so all rows use the same IR inputs (not updated values).
        const gte::Mat3  C2 = lcm();
        const gte::Vec32 B  = bk();
        const gte::fp16  ir1 = ir(1u), ir2 = ir(2u), ir3 = ir(3u);
        for (u32 i = 0; i < 3u; ++i) {
            const u32 n = i + 1u;
            const gte::fp44 acc =
                mac_se(mac_se(gte::tr_to_fp44(B[i])
                            + gte::fp16_mul(C2[i][0], ir1), n)
                     + gte::fp16_mul(C2[i][1], ir2), n)
                + gte::fp16_mul(C2[i][2], ir3);
            set_mac(n, acc, c.sf());
            set_ir (n, mac(n), c.lm());
        }
        // Step 3: source-colour scale — [R*IR1,G*IR2,B*IR3] SHL 4 SAR (sf*12).
        set_mac(1u, gte::fp16_mul(static_cast<gte::fp16>(rgbc_r()), ir(1u)) << 4, c.sf());
        set_mac(2u, gte::fp16_mul(static_cast<gte::fp16>(rgbc_g()), ir(2u)) << 4, c.sf());
        set_mac(3u, gte::fp16_mul(static_cast<gte::fp16>(rgbc_b()), ir(3u)) << 4, c.sf());
        set_ir(1u, mac(1u), c.lm());
        set_ir(2u, mac(2u), c.lm());
        set_ir(3u, mac(3u), c.lm());
        push_rgb(mac(1u) >> 4, mac(2u) >> 4, mac(3u) >> 4);
        break;
    }

    // ── Sub-task D: depth-cue commands ─────────────────────────────────────────

    case 0x13: /* NCDS — normal colour depth single  (V0) */
        compute_ncds(0u, c);
        break;

    case 0x16: /* NCDT — normal colour depth triple (V0,V1,V2) */
        compute_ncds(0u, c);
        compute_ncds(1u, c);
        compute_ncds(2u, c);
        break;

    case 0x10: /* DPCS — depth-cue RGBC colour */
        compute_dpcs(static_cast<s32>(rgbc_r()),
                     static_cast<s32>(rgbc_g()),
                     static_cast<s32>(rgbc_b()), c);
        break;

    case 0x2A: { /* DPCT — depth-cue colour triple (RGB0, RGB1, RGB2) */
        // Snapshot all three colours before any push_rgb shifts the FIFO.
        const s32 r0 = static_cast<s32>(rgb_r(0u)), g0 = static_cast<s32>(rgb_g(0u)), b0 = static_cast<s32>(rgb_b(0u));
        const s32 r1 = static_cast<s32>(rgb_r(1u)), g1 = static_cast<s32>(rgb_g(1u)), b1 = static_cast<s32>(rgb_b(1u));
        const s32 r2 = static_cast<s32>(rgb_r(2u)), g2 = static_cast<s32>(rgb_g(2u)), b2 = static_cast<s32>(rgb_b(2u));
        compute_dpcs(r0, g0, b0, c);
        compute_dpcs(r1, g1, b1, c);
        compute_dpcs(r2, g2, b2, c);
        break;
    }

    case 0x29: { /* DCPL — depth-cue point light (step 3 + step 4, no normal) */
        // Step 3: source-colour scale — raw (R*IR)<<4, no MAC truncation.
        // Used directly as in_MAC for InterpolateColor (same as NCDS step 3).
        const gte::fp44 dcpl_mac1 = static_cast<gte::fp44>(
            static_cast<s32>(rgbc_r()) * static_cast<s32>(ir(1u))) << 4;
        const gte::fp44 dcpl_mac2 = static_cast<gte::fp44>(
            static_cast<s32>(rgbc_g()) * static_cast<s32>(ir(2u))) << 4;
        const gte::fp44 dcpl_mac3 = static_cast<gte::fp44>(
            static_cast<s32>(rgbc_b()) * static_cast<s32>(ir(3u))) << 4;
        // Step 4: InterpolateColor — two-step far-colour blend.
        {
            const gte::Vec32 F = fc();
            // Step 4a: (FC << 12) - in_MAC, IR with lm=false.
            set_mac(1u, static_cast<gte::fp44>(F[0]) * gte::fp44(0x1000) - dcpl_mac1, c.sf());
            set_mac(2u, static_cast<gte::fp44>(F[1]) * gte::fp44(0x1000) - dcpl_mac2, c.sf());
            set_mac(3u, static_cast<gte::fp44>(F[2]) * gte::fp44(0x1000) - dcpl_mac3, c.sf());
            set_ir(1u, mac(1u), false);
            set_ir(2u, mac(2u), false);
            set_ir(3u, mac(3u), false);
            // Step 4b: IR * IR0 + in_MAC (pre-step-4 value), IR with cmd lm.
            set_mac(1u, static_cast<gte::fp44>(ir(1u)) * static_cast<gte::fp44>(ir0()) + dcpl_mac1, c.sf());
            set_mac(2u, static_cast<gte::fp44>(ir(2u)) * static_cast<gte::fp44>(ir0()) + dcpl_mac2, c.sf());
            set_mac(3u, static_cast<gte::fp44>(ir(3u)) * static_cast<gte::fp44>(ir0()) + dcpl_mac3, c.sf());
        }
        set_ir(1u, mac(1u), c.lm());
        set_ir(2u, mac(2u), c.lm());
        set_ir(3u, mac(3u), c.lm());
        push_rgb(mac(1u) >> 4, mac(2u) >> 4, mac(3u) >> 4);
        break;
    }

    case 0x14: { /* CDP — colour depth point (step 2 + step 3 + step 4) */
        // Step 2: BK + LCM × current IR.
        // Two nested flag-check+SE calls (translation + 3 products, DuckStation pattern).
        // IR captured before loop so all rows use the same IR inputs (not updated values).
        const gte::Mat3  C2 = lcm();
        const gte::Vec32 B  = bk();
        const gte::fp16  ir1 = ir(1u), ir2 = ir(2u), ir3 = ir(3u);
        for (u32 i = 0; i < 3u; ++i) {
            const u32 n = i + 1u;
            const gte::fp44 acc =
                mac_se(mac_se(gte::tr_to_fp44(B[i])
                            + gte::fp16_mul(C2[i][0], ir1), n)
                     + gte::fp16_mul(C2[i][1], ir2), n)
                + gte::fp16_mul(C2[i][2], ir3);
            set_mac(n, acc, c.sf());
            set_ir (n, mac(n), c.lm());
        }
        // Step 3: source-colour scale — raw (R*IR)<<4, no MAC truncation.
        const gte::fp44 cdp_mac1 = static_cast<gte::fp44>(
            static_cast<s32>(rgbc_r()) * static_cast<s32>(ir(1u))) << 4;
        const gte::fp44 cdp_mac2 = static_cast<gte::fp44>(
            static_cast<s32>(rgbc_g()) * static_cast<s32>(ir(2u))) << 4;
        const gte::fp44 cdp_mac3 = static_cast<gte::fp44>(
            static_cast<s32>(rgbc_b()) * static_cast<s32>(ir(3u))) << 4;
        // Step 4: InterpolateColor — two-step far-colour blend.
        {
            const gte::Vec32 F = fc();
            // Step 4a: (FC << 12) - in_MAC, IR with lm=false.
            set_mac(1u, static_cast<gte::fp44>(F[0]) * gte::fp44(0x1000) - cdp_mac1, c.sf());
            set_mac(2u, static_cast<gte::fp44>(F[1]) * gte::fp44(0x1000) - cdp_mac2, c.sf());
            set_mac(3u, static_cast<gte::fp44>(F[2]) * gte::fp44(0x1000) - cdp_mac3, c.sf());
            set_ir(1u, mac(1u), false);
            set_ir(2u, mac(2u), false);
            set_ir(3u, mac(3u), false);
            // Step 4b: IR * IR0 + in_MAC (pre-step-4 value), IR with cmd lm.
            set_mac(1u, static_cast<gte::fp44>(ir(1u)) * static_cast<gte::fp44>(ir0()) + cdp_mac1, c.sf());
            set_mac(2u, static_cast<gte::fp44>(ir(2u)) * static_cast<gte::fp44>(ir0()) + cdp_mac2, c.sf());
            set_mac(3u, static_cast<gte::fp44>(ir(3u)) * static_cast<gte::fp44>(ir0()) + cdp_mac3, c.sf());
        }
        set_ir(1u, mac(1u), c.lm());
        set_ir(2u, mac(2u), c.lm());
        set_ir(3u, mac(3u), c.lm());
        push_rgb(mac(1u) >> 4, mac(2u) >> 4, mac(3u) >> 4);
        break;
    }

    // ── Sub-task E: interpolation commands ─────────────────────────────────────

    case 0x11: { /* INTPL — interpolate IR toward far colour by IR0 */
        // Step 1: in_MAC = IR << 12, stored with shift=0.
        set_mac(1u, static_cast<gte::fp44>(ir(1u)) << 12, false);
        set_mac(2u, static_cast<gte::fp44>(ir(2u)) << 12, false);
        set_mac(3u, static_cast<gte::fp44>(ir(3u)) << 12, false);
        // InterpolateColor using the same two-step formula.
        {
            const s32 in_mac1 = mac(1u), in_mac2 = mac(2u), in_mac3 = mac(3u);
            const gte::Vec32 F = fc();
            // Step a: (FC * 0x1000) - in_MAC, IR with lm=false.
            set_mac(1u, static_cast<gte::fp44>(F[0]) * gte::fp44(0x1000) - static_cast<gte::fp44>(in_mac1), c.sf());
            set_mac(2u, static_cast<gte::fp44>(F[1]) * gte::fp44(0x1000) - static_cast<gte::fp44>(in_mac2), c.sf());
            set_mac(3u, static_cast<gte::fp44>(F[2]) * gte::fp44(0x1000) - static_cast<gte::fp44>(in_mac3), c.sf());
            set_ir(1u, mac(1u), false);
            set_ir(2u, mac(2u), false);
            set_ir(3u, mac(3u), false);
            // Step b: IR * IR0 + in_MAC (original), IR with cmd lm.
            set_mac(1u, static_cast<gte::fp44>(ir(1u)) * static_cast<gte::fp44>(ir0()) + static_cast<gte::fp44>(in_mac1), c.sf());
            set_mac(2u, static_cast<gte::fp44>(ir(2u)) * static_cast<gte::fp44>(ir0()) + static_cast<gte::fp44>(in_mac2), c.sf());
            set_mac(3u, static_cast<gte::fp44>(ir(3u)) * static_cast<gte::fp44>(ir0()) + static_cast<gte::fp44>(in_mac3), c.sf());
            set_ir(1u, mac(1u), c.lm());
            set_ir(2u, mac(2u), c.lm());
            set_ir(3u, mac(3u), c.lm());
        }
        push_rgb(mac(1u) >> 4, mac(2u) >> 4, mac(3u) >> 4);
        break;
    }

    case 0x3D: { /* GPF — general-purpose interpolation: IR*IR0 */
        // [MAC1,2,3] = [IR1,2,3]*IR0  SAR (sf*12)
        for (u32 n = 1u; n <= 3u; ++n) {
            set_mac(n, gte::fp16_mul(ir(n), ir0()), c.sf());
            set_ir (n, mac(n), c.lm());
        }
        push_rgb(mac(1u) >> 4, mac(2u) >> 4, mac(3u) >> 4);
        break;
    }

    case 0x3E: { /* GPL — general-purpose interpolation + MAC: MAC + IR*IR0 */
        // [MAC1,2,3] = [MAC1,2,3] SHL (sf*12) + [IR1,2,3]*IR0  SAR (sf*12)
        // sf=1: stored MAC was >>12, shift it back up by 12 (tr_to_fp44)
        // sf=0: stored MAC IS the fp44 value already, just sign-extend
        for (u32 n = 1u; n <= 3u; ++n) {
            const gte::fp44 mac_fp44 = c.sf() ? gte::tr_to_fp44(mac(n))
                                               : static_cast<gte::fp44>(mac(n));
            const gte::fp44 acc = mac_fp44 + gte::fp16_mul(ir(n), ir0());
            set_mac(n, acc, c.sf());
            set_ir (n, mac(n), c.lm());
        }
        push_rgb(mac(1u) >> 4, mac(2u) >> 4, mac(3u) >> 4);
        break;
    }
    default:
        std::fprintf(stderr,
                     "[GTE] unknown command func=0x%02X cmd=0x%08X @ PC 0x%08X\n",
                     c.func(), cmd, pc);
        dump_gte_state(pc, stderr);
        break;
    }

    // Recompute error-summary bit after every command.
    update_flag();
}
