#pragma once

#include <array>
#include <span>
#include <cstring>
#include <cassert>
#include "common/Types.hpp"

// ── RAM ───────────────────────────────────────────────────────────────────────
// 2 MiB of main system RAM.  On real hardware the 2 MiB window is mirrored
// four times across 0x0000_0000–0x007F_FFFF; the Bus is responsible for
// masking the address before calling into this class.
class Ram {
public:
    static constexpr u32 SIZE = PSX::RAM_SIZE;

    // Type-safe bulk access — used by the debugger / DMA
    [[nodiscard]] std::span<const u8> view() const noexcept { return data_; }
    [[nodiscard]] std::span<u8>       view()       noexcept { return data_; }

    // ── Bus interface ─────────────────────────────────────────────────────────
    // All reads/writes use std::memcpy to avoid strict-aliasing UB.
    // The Bus guarantees offset + sizeof(T) <= SIZE before calling.

    template<std::unsigned_integral T>
    [[nodiscard]] T read(u32 offset) const noexcept {
        assert(offset + sizeof(T) <= SIZE);
        T value{};
        std::memcpy(&value, data_.data() + offset, sizeof(T));
        return value;
    }

    template<std::unsigned_integral T>
    void write(u32 offset, T value) noexcept {
        assert(offset + sizeof(T) <= SIZE);
        std::memcpy(data_.data() + offset, &value, sizeof(T));
    }

private:
    // Value-initialised to zero — matches cold-boot state.
    std::array<u8, SIZE> data_{};
};
