#include "Bios.hpp"

#include <fstream>
#include <stdexcept>

bool Bios::load(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return false;
    }

    const auto file_size = static_cast<std::size_t>(file.tellg());
    if (file_size != SIZE) {
        // BIOS images must be exactly 512 KiB.
        return false;
    }

    file.seekg(0);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    file.read(reinterpret_cast<char*>(data_.data()), SIZE);
    return file.good();
}
