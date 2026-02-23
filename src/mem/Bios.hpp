#pragma once

#include <array>
#include <span>
#include <filesystem>
#include <cstring>
#include <cassert>
#include "common/Types.hpp"

// ── BIOS ROM ──────────────────────────────────────────────────────────────────
// 512 KiB read-only image loaded from a dump file.
// Mapped at physical 0x1FC0_0000 (KSEG1 alias: 0xBFC0_0000 — reset vector).
class Bios {
public:
    static constexpr u32 SIZE = PSX::BIOS_SIZE;

    // Load a BIOS image from disk.  Returns false if the file cannot be read
    // or is the wrong size; the caller should treat that as a fatal error.
    [[nodiscard]] bool load(const std::filesystem::path& path);

    [[nodiscard]] std::span<const u8> view() const noexcept { return data_; }

    // Read-only — the BIOS ROM is not writable on real hardware.
    template<std::unsigned_integral T>
    [[nodiscard]] T read(u32 offset) const noexcept {
        assert(offset + sizeof(T) <= SIZE);
        T value{};
        std::memcpy(&value, data_.data() + offset, sizeof(T));
        return value;
    }

private:
    std::array<u8, SIZE> data_{};
};
