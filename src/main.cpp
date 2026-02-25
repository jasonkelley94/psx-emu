#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <memory>

#include "mem/Bios.hpp"
#include "bus/Bus.hpp"
#include "cpu/CPU.hpp"
#include "display/Display.hpp"

// ── VRAM PPM dump ─────────────────────────────────────────────────────────────
// Writes the GPU framebuffer as a binary PPM (P6) file.  The reference images
// shipped with ps1-tests releases can be compared against this output.
static void dump_vram_ppm(const GPU& gpu, const char* path) noexcept {
    FILE* f = std::fopen(path, "wb");
    if (!f) {
        std::fprintf(stderr, "[dump] cannot open '%s' for writing\n", path);
        return;
    }
    std::fprintf(f, "P6\n%u %u\n255\n",
                 static_cast<unsigned>(GPU::VRAM_W),
                 static_cast<unsigned>(GPU::VRAM_H));
    for (const u16 px : gpu.frame_buffer()) {
        // RGB555 → RGB888: scale each 5-bit channel to 8 bits.
        const u8 r = static_cast<u8>(((px >>  0u) & 0x1Fu) << 3u);
        const u8 g = static_cast<u8>(((px >>  5u) & 0x1Fu) << 3u);
        const u8 b = static_cast<u8>(((px >> 10u) & 0x1Fu) << 3u);
        std::fwrite(&r, 1u, 1u, f);
        std::fwrite(&g, 1u, 1u, f);
        std::fwrite(&b, 1u, 1u, f);
    }
    std::fclose(f);
    std::fprintf(stdout, "[dump] VRAM written to '%s'\n", path);
}

// ── Helpers ───────────────────────────────────────────────────────────────────
static bool ends_with(const char* s, const char* suffix) noexcept {
    const std::size_t slen   = std::strlen(s);
    const std::size_t suflen = std::strlen(suffix);
    return slen >= suflen && std::strcmp(s + slen - suflen, suffix) == 0;
}

static bool is_psexe(const char* path) noexcept {
    return ends_with(path, ".exe") || ends_with(path, ".EXE")
        || ends_with(path, ".psexe");
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    // ── Parse flags ───────────────────────────────────────────────────────────
    bool     headless   = false;
    uint64_t max_cycles = 0u;     // 0 = unlimited

    int positional = 0;            // index into argv of the first positional arg
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--headless") == 0) {
            headless = true;
        } else if (std::strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) {
            max_cycles = std::strtoull(argv[++i], nullptr, 10);
        } else if (positional == 0) {
            positional = i;        // first non-flag: bios or .exe
        }
    }

    if (positional == 0) {
        std::fprintf(stderr,
            "Usage: psx_emu [--headless] [--cycles N] <bios.bin|test.exe> [disc.bin|disc.iso|disc.cue]\n");
        return EXIT_FAILURE;
    }

    const char* first_arg = argv[positional];

    // ── Build the system and CPU ──────────────────────────────────────────────
    std::unique_ptr<Bus> bus;
    std::unique_ptr<CPU> cpu;

    if (is_psexe(first_arg)) {
        // ── Sideload mode: run a bare-metal PS-EXE without a BIOS ────────────
        // PSN00BSDK test executables are self-contained — they handle their own
        // hardware initialisation and do not require the PlayStation BIOS.
        bus = std::make_unique<Bus>(nullptr);
        cpu = std::make_unique<CPU>(*bus);

        const Bus::PsxExeInfo info = bus->sideload(first_arg);
        if (!info.ok) {
            std::fprintf(stderr, "Failed to load PS-EXE '%s'\n", first_arg);
            return EXIT_FAILURE;
        }

        cpu->set_pc(info.pc);
        cpu->set_reg(28u, info.gp);
        // ps1-tests set SP=0 in the EXE header; supply the conventional default.
        cpu->set_reg(29u, info.sp ? info.sp : 0x801F'FFF0u);

        // Default cycle budget for headless runs; interactive loop ignores it.
        if (headless && max_cycles == 0u) max_cycles = 20'000'000u;

    } else {
        // ── Normal mode: BIOS boot, optional disc image ───────────────────────
        auto bios = std::make_unique<Bios>();
        if (!bios->load(first_arg)) {
            std::fprintf(stderr, "Failed to load BIOS from '%s' (expected %u bytes)\n",
                         first_arg, Bios::SIZE);
            return EXIT_FAILURE;
        }
        std::fprintf(stdout, "BIOS loaded (%u KiB)\n", Bios::SIZE / 1024u);

        bus = std::make_unique<Bus>(std::move(bios));
        cpu = std::make_unique<CPU>(*bus);

        // Optional disc image (second positional argument).
        if (positional + 1 < argc) {
            const char* disc_path = argv[positional + 1];
            if (!is_psexe(disc_path)) {  // don't treat a .exe as a disc
                if (!bus->cdrom().load_disc(disc_path)) {
                    std::fprintf(stderr, "Warning: could not load disc '%s'\n", disc_path);
                }
            }
        }
    }

    // ── Headless run loop ─────────────────────────────────────────────────────
    if (headless) {
        uint64_t cycles  = 0u;
        u32      prev_pc = ~0u;
        int      same_pc = 0;

        u32 trace_prev_pc = 0;
        uint64_t trace_count = 0;
        while (max_cycles == 0u || cycles < max_cycles) {
            if (!cpu->step()) {
                std::fprintf(stderr, "CPU halted at PC=0x%08X\n", cpu->pc());
                break;
            }
            ++cycles;
            bus->tick();

            // PC frequency trace every 10M cycles
            if (cycles % 10000000u == 0u) {
                if (cpu->pc() == trace_prev_pc) {
                    ++trace_count;
                    if (trace_count <= 5u)
                        std::fprintf(stderr, "[TRACE] %lluM: PC=0x%08X (stuck)\n",
                                     static_cast<unsigned long long>(cycles / 1000000u), cpu->pc());
                } else {
                    trace_count = 0;
                    std::fprintf(stderr, "[TRACE] %lluM: PC=0x%08X\n",
                                 static_cast<unsigned long long>(cycles / 1000000u), cpu->pc());
                }
                trace_prev_pc = cpu->pc();
            }

            // Detect spin loop (j $pc with NOP delay slot → PC oscillates).
            if (cpu->pc() == prev_pc) {
                if (++same_pc >= 4) {
                    std::fprintf(stdout, "[HALT] PC=0x%08X after %llu cycles\n",
                                 cpu->pc(), static_cast<unsigned long long>(cycles));
                    break;
                }
            } else {
                same_pc = 0;
            }
            prev_pc = cpu->pc();
        }

        dump_vram_ppm(bus->gpu(), "vram.ppm");
        std::fprintf(stdout, "Done (%llu cycles). Final PC=0x%08X\n",
                     static_cast<unsigned long long>(cycles), cpu->pc());
        return EXIT_SUCCESS;
    }

    // ── Interactive SDL run loop ──────────────────────────────────────────────
    // Present the GPU framebuffer to the screen once per emulated VBlank (~60 Hz).
    Display display;

    static constexpr uint64_t kVBlankPeriod = 564'480u;
    uint64_t cycle_acc = 0u;

    while (display.poll_events()) {
        if (!cpu->step()) {
            std::fprintf(stderr, "CPU halted\n");
            break;
        }
        bus->tick();
        if (++cycle_acc >= kVBlankPeriod) {
            cycle_acc -= kVBlankPeriod;
            display.present(bus->gpu());
        }
    }

    std::fprintf(stdout, "Emulation stopped.\n");
    return EXIT_SUCCESS;
}
