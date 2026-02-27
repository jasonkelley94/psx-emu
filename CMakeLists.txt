cmake_minimum_required(VERSION 3.22)
project(psx_emu LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for clangd / IDE tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(SDL2 REQUIRED)

add_executable(psx_emu
    src/main.cpp
    src/mem/Ram.cpp
    src/mem/Bios.cpp
    src/gpu/GPU.cpp
    src/irq/IRQ.cpp
    src/cdrom/CDRom.cpp
    src/timers/Timers.cpp
    src/dma/DMA.cpp
    src/bus/Bus.cpp
    src/gte/GTE.cpp
    src/cpu/CPU.cpp
    src/display/Display.cpp
    src/spu/SPU.cpp
)

target_include_directories(psx_emu PRIVATE src)
target_link_libraries(psx_emu PRIVATE SDL2::SDL2)

target_compile_options(psx_emu PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
    $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>
    $<$<CONFIG:Debug>:-g3 -fno-omit-frame-pointer>
    $<$<CONFIG:Debug>:-fsanitize=address,undefined>
)

target_link_options(psx_emu PRIVATE
    $<$<CONFIG:Debug>:-fsanitize=address,undefined>
)

# ── convenience targets ────────────────────────────────────────────────────────
add_custom_target(run
    COMMAND psx_emu
    DEPENDS psx_emu
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running psx_emu"
)
