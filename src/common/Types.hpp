#pragma once

#include <cstdint>
#include <cstddef>

// ── Scalar type aliases ────────────────────────────────────────────────────────
using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

using s8  = std::int8_t;
using s16 = std::int16_t;
using s32 = std::int32_t;
using s64 = std::int64_t;

// ── PSX physical address space constants ──────────────────────────────────────
namespace PSX {

// The R3000A has a 32-bit virtual address space partitioned into regions.
// KSEG0 (0x8000_0000) and KSEG1 (0xA000_0000) both alias the same 512 MB of
// physical memory — KSEG1 is uncached.  Stripping the top 3 bits collapses all
// three user/kernel windows onto the same physical range.
//
//   virtual >> 29   0–3   KUSEG  – user, translated (TLB, not used on PSX)
//                   4     KSEG0  – kernel cached
//                   5     KSEG1  – kernel uncached
//                   6–7   KSEG2  – kernel, translated (rare on PSX)
//
inline constexpr u32 REGION_MASK[8] = {
    0xFFFF'FFFF, 0xFFFF'FFFF, 0xFFFF'FFFF, 0xFFFF'FFFF,  // KUSEG
    0x7FFF'FFFF,                                           // KSEG0
    0x1FFF'FFFF,                                           // KSEG1
    0xFFFF'FFFF, 0xFFFF'FFFF,                              // KSEG2
};

[[nodiscard]] constexpr u32 to_physical(u32 vaddr) noexcept {
    return vaddr & REGION_MASK[vaddr >> 29];
}

// Physical memory map ─────────────────────────────────────────────────────────
inline constexpr u32 RAM_BASE     = 0x0000'0000;
inline constexpr u32 RAM_SIZE     = 2u * 1024u * 1024u;   // 2 MiB

inline constexpr u32 EXP1_BASE    = 0x1F00'0000;
inline constexpr u32 EXP1_SIZE    = 8u * 1024u * 1024u;   // 8 MiB

inline constexpr u32 SCRATCH_BASE = 0x1F80'0000;
inline constexpr u32 SCRATCH_SIZE = 1024u;                 // 1 KiB scratchpad

inline constexpr u32 IO_BASE      = 0x1F80'1000;
inline constexpr u32 IO_SIZE      = 0x2000u;               // 8 KiB MMIO window

inline constexpr u32 EXP2_BASE    = 0x1F80'2000;
inline constexpr u32 EXP2_SIZE    = 0x2000u;

inline constexpr u32 BIOS_BASE    = 0x1FC0'0000;
inline constexpr u32 BIOS_SIZE    = 512u * 1024u;          // 512 KiB

inline constexpr u32 CACHE_CTL    = 0xFFFE'0000;           // cache-control register

// Reset vector (uncached BIOS entry point, KSEG1)
inline constexpr u32 RESET_VECTOR = 0xBFC0'0000;

} // namespace PSX
