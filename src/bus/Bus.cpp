#include "Bus.hpp"
#include <cstring>

Bus::Bus(std::unique_ptr<Bios> bios)
    : ram_   (std::make_unique<Ram>())
    , bios_  (std::move(bios))
    , gpu_   (std::make_unique<GPU>())
    , irq_   (std::make_unique<IRQ>())
    , cdrom_ (std::make_unique<CDRom>(*irq_))
    , timers_(std::make_unique<Timers>(*irq_, *gpu_))
    , spu_   (std::make_unique<SPU>(*irq_))
      // DMA constructed last — it holds non-owning references to the above.
    , dma_   (std::make_unique<DMA>(*ram_, *gpu_, *irq_, *cdrom_,
                                    spu_ram_.data(), spu_transfer_addr_))
{
    spu_->set_ram_ptr(spu_ram_.data(), &spu_transfer_addr_);
    mc_format();  // pre-load an empty formatted memory card
}

// ── I/O window offsets (relative to IO_BASE = 0x1F80_1000) ───────────────────
//
//   0x000 – 0x023   Memory control 1 (expansion timing — safe to ignore)
//   0x060           RAM size register
//   0x070           I_STAT  (interrupt status)
//   0x074           I_MASK  (interrupt mask)
//   0x080 – 0x0EF   DMA channels 0–6 (3 registers each, 0x10 bytes apart)
//   0x0F0           D_DPCR  (DMA priority)
//   0x0F4           D_DICR  (DMA interrupt control)
//   0x100 – 0x12C   Timers 0–2  (root counters)
//   0x810 – 0x814   GPU GP0 / GP1 + GPUSTAT

namespace {
    // CD-ROM register window: IO_BASE+0x800 … IO_BASE+0x803 (4 byte-wide regs)
    constexpr u32 CDROM_BASE_OFF = 0x800u;
    constexpr u32 CDROM_END_OFF  = 0x804u;

    // DMA register window: IO_BASE+0x080 … IO_BASE+0x0FF
    constexpr u32 DMA_BASE_OFF = 0x080u;
    constexpr u32 DMA_END_OFF  = 0x100u;

    // GPU register window
    constexpr u32 GPU_BASE_OFF = 0x810u;
    constexpr u32 GPU_END_OFF  = 0x818u;
}

// ── MMIO read (32-bit) ────────────────────────────────────────────────────────
u32 Bus::io_read32(u32 phys) const noexcept {
    const u32 off = phys - PSX::IO_BASE;

    // ── Interrupt controller ──────────────────────────────────────────────────
    if (off == 0x070) return irq_->read_stat();
    if (off == 0x074) return irq_->read_mask();

    // ── CD-ROM (byte-wide) — pack all 4 registers into a 32-bit word ─────────
    // In practice the CPU only does byte reads here; see io_read8 for the
    // correct single-register path (no side effects on registers not accessed).
    if (off >= CDROM_BASE_OFF && off < CDROM_END_OFF) {
        const u32 b0 = cdrom_->read(0u);
        const u32 b1 = cdrom_->read(1u);
        const u32 b2 = cdrom_->read(2u);
        const u32 b3 = cdrom_->read(3u);
        return b0 | (b1 << 8u) | (b2 << 16u) | (b3 << 24u);
    }

    // ── DMA ───────────────────────────────────────────────────────────────────
    if (off >= DMA_BASE_OFF && off < DMA_END_OFF) {
        return dma_->read(off - DMA_BASE_OFF);
    }

    // ── GPU ───────────────────────────────────────────────────────────────────
    if (off >= GPU_BASE_OFF && off < GPU_END_OFF) {
        return gpu_->read(off - GPU_BASE_OFF);
    }

    // ── Timers (root counters 0–2) ────────────────────────────────────────────
    if (off >= 0x100u && off < 0x130u) {
        return timers_->read(off - 0x100u);
    }

    // ── Joypad / SIO0 (0x040–0x04F) ─────────────────────────────────────────
    // SIO1 (0x050–0x05F): PSN00BSDK polls SIO_STAT(1) for TX-ready bits.
    if (off >= 0x050u && off < 0x060u) {
        return 0x0000'0005u;  // SIO1_STAT: TX ready, no RX
    }
    if (off >= 0x040u && off < 0x050u) {
        u32 val = 0u;
        switch (off & ~0x3u) {
        case 0x040u: {  // JOY_DATA — consume and clear RX FIFO
            val = joy_.rx_byte;
            joy_.rx_ready = false;
            break;
        }
        case 0x044u:  // JOY_STAT
            // bit 0 TXRDY1 = 1, bit 2 TXRDY2 = 1, bit 1 RXFIFO, bit 7 ACKINPUT
            // ACKINPUT = 0 (ACK active/low) when RX data ready, 1 when idle.
            val = 0x00000005u | (joy_.rx_ready ? 0x00000002u : 0x00000080u);
            break;
        default:
            val = 0u;  // JOY_MODE, JOY_CTRL, JOY_BAUD
            break;
        }
        // DEBUG: trace joypad reads
        { static int jr = 0;
          if (++jr <= 50)
              std::fprintf(stderr, "[JOY] read32 off=0x%03X val=0x%08X\n", off, val); }
        return val;
    }

    // ── Memory control registers — ignored, return 0 ─────────────────────────
    if (off < 0x024u || off == 0x060u) {
        return 0u;
    }

    // ── SPU (offsets 0xC00–0xE7F) ─────────────────────────────────────────────
    if (off >= 0xC00u && off < 0xE80u) {
        return spu_->read(off);
    }

    // ── MDEC (0x1F801820–0x1F801824) — status / control ──────────────────────
    // Status register (0x820): bit 30 = Command Busy (0=idle), bit 29 = DMA-In req,
    // bit 28 = DMA-Out req.  Return 0x00000000 so the BIOS sees MDEC as idle.
    if (off == 0x820u || off == 0x824u) return 0x00000000u;

    std::fprintf(stderr, "[Bus] unhandled I/O read32  off=0x%04X\n", off);
    return 0xFFFF'FFFFu;
}

// ── MMIO write (32-bit) ───────────────────────────────────────────────────────
void Bus::io_write32(u32 phys, u32 value) noexcept {
    const u32 off = phys - PSX::IO_BASE;

    // ── Interrupt controller ──────────────────────────────────────────────────
    if (off == 0x070) { irq_->write_stat(value); return; }
    if (off == 0x074) { irq_->write_mask(value); return; }

    // ── DMA ───────────────────────────────────────────────────────────────────
    if (off >= DMA_BASE_OFF && off < DMA_END_OFF) {
        dma_->write(off - DMA_BASE_OFF, value);
        // Advance timers by any CPU cycles yielded during chopped DMA bursts.
        if (const u32 c = dma_->take_cpu_credit(); c > 0u)
            tick(c);
        return;
    }

    // ── GPU ───────────────────────────────────────────────────────────────────
    if (off >= GPU_BASE_OFF && off < GPU_END_OFF) {
        gpu_->write(off - GPU_BASE_OFF, value);
        return;
    }

    // ── Timers (root counters 0–2) ────────────────────────────────────────────
    if (off >= 0x100u && off < 0x130u) {
        timers_->write(off - 0x100u, value);
        return;
    }

    // ── Memory control 1 (expansion timing) — safe to swallow ────────────────
    if (off < 0x024 || off == 0x060) {
        return;
    }

    // ── Joypad / SIO0 (0x040–0x04F) ─────────────────────────────────────────
    if (off >= 0x040u && off < 0x060u) {
        // DEBUG: trace joypad writes
        { static int jw = 0;
          if (++jw <= 50)
              std::fprintf(stderr, "[JOY] write32 off=0x%03X val=0x%08X\n", off, value); }
        if (off == 0x040u) joy_data_write(static_cast<u8>(value));  // JOY_DATA
        if (off == 0x04Au) {  // JOY_CTRL: bit 6 (reset) clears both state machines
            if (value & (1u << 6u)) {
                joy_.seq = 0u; joy_.rx_ready = false;
                mc_.seq = 0u; mc_active_ = false;
            }
        }
        return;  // other SIO0/SIO1 writes silently ignored
    }

    // ── SPU (0x1F80_1C00 – 0x1F80_1E7F) ────────────────────────────────────────
    if (off >= 0xC00u && off < 0xE80u) {
        spu_->write(off, value);
        return;
    }

    // ── Expansion region 2 (0x1F80_2000) — POST port, debug output ───────────
    if (off >= 0x1000u && off < 0x2000u) {
        return;
    }

    // ── MDEC (0x1F801820–0x1F801824) — command / control (silently swallow) ───
    if (off == 0x820u || off == 0x824u) return;

    std::fprintf(stderr, "[Bus] unhandled I/O write32 off=0x%04X val=0x%08X\n", off, value);
}

// ── MMIO write (16-bit) — used for halfword CPU writes to the I/O window ──────
// SPU and timer registers are all 16-bit; the BIOS and games use halfword
// stores to set them.  Route those here instead of logging a false error.
void Bus::io_write16(u32 phys, u16 value) noexcept {
    const u32 off = phys - PSX::IO_BASE;

    // ── Interrupt controller (16-bit writes) ────────────────────────────────
    if (off == 0x070u) { irq_->write_stat(static_cast<u32>(value)); return; }
    if (off == 0x074u) { irq_->write_mask(static_cast<u32>(value)); return; }

    // ── Timers (16-bit registers) ─────────────────────────────────────────────
    if (off >= 0x100u && off < 0x130u) {
        timers_->write(off - 0x100u, static_cast<u32>(value));
        return;
    }

    // ── Joypad / SIO0 (16-bit writes) ────────────────────────────────────────
    if (off >= 0x040u && off < 0x060u) {
        // DEBUG: trace joypad 16-bit writes
        { static int jw16 = 0;
          if (++jw16 <= 50)
              std::fprintf(stderr, "[JOY] write16 off=0x%03X val=0x%04X\n",
                           off, static_cast<unsigned>(value)); }
        if (off == 0x04Au) {  // JOY_CTRL halfword write
            if (value & (1u << 6u)) { joy_.seq = 0u; joy_.rx_ready = false; }
        }
        return;
    }

    // ── SPU (16-bit writes) ─────────────────────────────────────────────────────
    if (off >= 0xC00u && off < 0xE80u) {
        spu_->write(off, static_cast<u32>(value));
        return;
    }

    // ── Expansion region 2 ────────────────────────────────────────────────────
    if (off >= 0x1000u && off < 0x2000u) return;

    std::fprintf(stderr, "[Bus] unhandled I/O write16 off=0x%04X val=0x%04X\n",
                 off, static_cast<u32>(value));
}

// ── MMIO read (8-bit) — used for byte CPU reads to the I/O window ────────────
// Routes CD-ROM byte registers directly; falls back to the 32-bit read + lane
// extraction for everything else.
u8 Bus::io_read8(u32 phys) const noexcept {
    const u32 off = phys - PSX::IO_BASE;

    // ── CD-ROM (byte-wide registers) ──────────────────────────────────────────
    if (off >= CDROM_BASE_OFF && off < CDROM_END_OFF) {
        return cdrom_->read(off - CDROM_BASE_OFF);
    }

    // ── All other I/O: widen to 32 bits then extract the requested byte ───────
    const u32 word = io_read32(phys & ~0x3u);
    return static_cast<u8>(word >> ((off & 3u) * 8u));
}

// ── MMIO write (8-bit) — used for byte CPU writes to the I/O window ──────────
void Bus::io_write8(u32 phys, u8 value) noexcept {
    const u32 off = phys - PSX::IO_BASE;

    // ── CD-ROM (byte-wide registers) ──────────────────────────────────────────
    if (off >= CDROM_BASE_OFF && off < CDROM_END_OFF) {
        cdrom_->write(off - CDROM_BASE_OFF, value);
        return;
    }

    // ── Joypad / SIO ─────────────────────────────────────────────────────────
    // SIO0 (0x040–0x04F): JOY_DATA byte writes feed the exchange FSM.
    // SIO1 (0x050–0x05F): SIO_DATA(1) carries PSN00BSDK printf byte output.
    if (off >= 0x040u && off < 0x060u) {
        if (off == 0x040u) joy_data_write(value);              // JOY_DATA
        else if (off == 0x050u) tty_putchar(static_cast<char>(value));  // TTY
        return;
    }

    // ── SPU (8-bit writes) ──────────────────────────────────────────────────────
    if (off >= 0xC00u && off < 0xE80u) {
        spu_->write(off, static_cast<u32>(value));
        return;
    }

    // ── Expansion region 2 (POST port at 0x1F802041+) ────────────────────────
    // Some PSN00BSDK programs emit single debug bytes here.
    if (off >= 0x1000u && off < 0x2000u) {
        static char exp_buf[256]; static int exp_pos = 0;
        if (value == u8('\n') || exp_pos >= 254) {
            exp_buf[exp_pos] = '\0';
            std::fprintf(stdout, "[EXP2] %s\n", exp_buf);
            exp_pos = 0;
        } else if (value >= u8(0x20u)) {  // printable ASCII only
            exp_buf[exp_pos++] = static_cast<char>(value);
        }
        return;
    }

    std::fprintf(stderr, "[Bus] unhandled I/O write8  off=0x%04X val=0x%02X\n",
                 off, static_cast<u32>(value));
}

// ── SIO0 exchange state machine ───────────────────────────────────────────────
// joy_data_write() is the single entry point for every byte written to
// JOY_DATA.  The address byte (seq==0) selects the device:
//   0x01 → joypad (digital pad)
//   0x81 → memory card
// Subsequent bytes are routed to the appropriate sub-machine.
void Bus::joy_data_write(u8 b) noexcept {
    // ── Memory card in progress — route to its state machine ─────────────────
    if (mc_active_) {
        mc_data_write(b);
        return;
    }

    switch (joy_.seq) {
    case 0:  // Address byte — select device
        joy_.rx_byte = 0xFFu;
        if (b == 0x01u) {
            joy_.seq = 1u;          // joypad selected
        } else if (b == 0x81u) {
            mc_active_ = true;      // memory card selected
            mc_.seq    = 1u;        // mc_data_write() starts at seq 1 (command)
        }
        // Unknown address: stay idle, return 0xFF
        break;
    case 1:  // Command byte: 0x42 = poll → respond with digital-pad ID (0x41)
        joy_.rx_byte = (b == 0x42u) ? 0x41u : 0xFFu;
        joy_.seq     = 2u;
        break;
    case 2:  // TAP request byte → respond with payload-start token (0x5A)
        joy_.rx_byte = 0x5Au;
        joy_.seq     = 3u;
        break;
    case 3:  // First data byte → button state, low byte (active-low)
        joy_.rx_byte = static_cast<u8>(joy_.buttons);
        joy_.seq     = 4u;
        break;
    case 4:  // Second data byte → button state, high byte; end of exchange
        joy_.rx_byte = static_cast<u8>(joy_.buttons >> 8u);
        joy_.seq     = 0u;
        break;
    default:
        joy_.rx_byte = 0xFFu;
        joy_.seq     = 0u;
        break;
    }
    joy_.rx_ready = true;
    // Fire IRQ7 (Controller) to emulate the /ACK pulse driven by the real pad.
    irq_->set(IRQSource::Ctrl);
}

// ── Memory card exchange state machine ────────────────────────────────────────
// Read frame (0x52):
//   seq 1: cmd→FLAG  2:→5Ah  3:→5Dh  4:addr_hi→0  5:addr_lo→0
//   seq 6:→addr_hi   7:→addr_lo  8:→5Ch  9..136:→data[0..127]
//   seq 137:→cksum   138:→47h  → idle
// Write frame (0x57):
//   seq 1..7: same header as read
//   seq 8..135: data[0..127]←host  136: cksum←host  137:→5Ch  138:→47h  → idle
// Other commands: FLAG + 5Ah + 5Dh + a few 0x00 bytes then idle.
void Bus::mc_data_write(u8 b) noexcept {
    const u32 frame = (static_cast<u32>(mc_.addr_hi) << 8u) | mc_.addr_lo;
    const u32 base  = frame * 128u;

    switch (mc_.seq) {
    case 1:  // command byte
        mc_.cmd      = b;
        joy_.rx_byte = 0x00u;   // FLAG = 0x00: normal (no error, card initialized)
        mc_.seq++;
        break;
    case 2:  // → 0x5A  (ACK byte 1)
        joy_.rx_byte = 0x5Au;
        mc_.seq++;
        break;
    case 3:  // → 0x5D  (ACK byte 2)
        joy_.rx_byte = 0x5Du;
        mc_.seq++;
        break;
    case 4:  // frame address MSB
        mc_.addr_hi  = b;
        joy_.rx_byte = 0x00u;
        mc_.seq++;
        break;
    case 5:  // frame address LSB → validate and start checksum
        mc_.addr_lo  = b;
        mc_.cksum    = mc_.addr_hi ^ mc_.addr_lo;
        joy_.rx_byte = 0x00u;
        mc_.seq++;
        break;
    case 6:  // echo addr_hi
        joy_.rx_byte = mc_.addr_hi;
        mc_.seq++;
        break;
    case 7:  // echo addr_lo
        joy_.rx_byte = mc_.addr_lo;
        mc_.seq++;
        break;

    default:
        if (mc_.cmd == 0x52u) {
            // ── READ FRAME ────────────────────────────────────────────────────
            if (mc_.seq == 8u) {
                // Confirm address validity (5Ch = good, 04h = bad)
                joy_.rx_byte = (frame < 1024u) ? 0x5Cu : 0x04u;
                mc_.seq++;
            } else if (mc_.seq <= 136u) {
                // Data bytes 0..127 (seq 9..136)
                const u32 di  = mc_.seq - 9u;
                const u8  val = (base + di < McState::SIZE) ? mc_.ram[base + di] : 0x00u;
                mc_.cksum   ^= val;
                joy_.rx_byte = val;
                mc_.seq++;
            } else if (mc_.seq == 137u) {
                joy_.rx_byte = mc_.cksum;   // XOR checksum
                mc_.seq++;
            } else {
                joy_.rx_byte = 0x47u;       // 'G' = Good
                mc_.seq     = 0u;
                mc_active_  = false;
            }
        } else if (mc_.cmd == 0x57u) {
            // ── WRITE FRAME ───────────────────────────────────────────────────
            if (mc_.seq <= 135u) {
                // Data bytes 0..127 (seq 8..135)
                const u32 di = mc_.seq - 8u;
                if (base + di < McState::SIZE) {
                    mc_.ram[base + di] = b;
                    mc_.cksum         ^= b;
                }
                joy_.rx_byte = 0x00u;
                mc_.seq++;
            } else if (mc_.seq == 136u) {
                // Receive checksum from host (acknowledge, don't validate)
                joy_.rx_byte = 0x00u;
                mc_.seq++;
            } else if (mc_.seq == 137u) {
                joy_.rx_byte = 0x5Cu;   // confirm write OK
                mc_.seq++;
            } else {
                joy_.rx_byte = 0x47u;   // 'G' = Good
                mc_.seq     = 0u;
                mc_active_  = false;
            }
        } else {
            // ── Unknown command — short ack then idle ─────────────────────────
            joy_.rx_byte = 0x00u;
            if (mc_.seq >= 10u) {
                mc_.seq    = 0u;
                mc_active_ = false;
            } else {
                mc_.seq++;
            }
        }
        break;
    }

    joy_.rx_ready = true;
    irq_->set(IRQSource::Ctrl);
}

// ── Format the memory card with an empty PSX directory ────────────────────────
// Frame 0  : Manufacturer's block — magic "MC" + XOR checksum.
// Frames 1–15: Free directory entries (status=0xA0, next=0xFFFF, checksum=0xA0).
// All other frames are zero (available for data).
void Bus::mc_format() noexcept {
    mc_.ram.fill(0x00u);

    // Frame 0: Manufacturer's block "MC"
    mc_.ram[0]   = 0x4Du;  // 'M'
    mc_.ram[1]   = 0x43u;  // 'C'
    mc_.ram[127] = 0x4Du ^ 0x43u;  // XOR checksum of bytes 0–126 = 0x0E

    // Frames 1–15: Free directory entries
    for (u32 f = 1u; f <= 15u; ++f) {
        const u32 b = f * 128u;
        mc_.ram[b + 0]   = 0xA0u;  // block status: free
        mc_.ram[b + 8]   = 0xFFu;  // next block pointer hi: none
        mc_.ram[b + 9]   = 0xFFu;  // next block pointer lo: none
        // XOR of bytes 0–126: 0xA0 ^ 0xFF ^ 0xFF = 0xA0
        mc_.ram[b + 127] = 0xA0u;
    }
}

// ── TTY putchar ───────────────────────────────────────────────────────────────
// Accumulates characters into a line buffer; flushes with a [TTY] prefix on
// newline or when the buffer is nearly full.  Called from the SIO1 byte-write
// path and from the CPU's BIOS A-function intercept (A(0x3C/0x3E/0x3F)).
void Bus::tty_putchar(char c) noexcept {
    static char buf[256];
    static int  pos = 0;
    if (c == '\n' || pos >= 254) {
        buf[pos] = '\0';
        std::fprintf(stdout, "[TTY] %s\n", buf);
        std::fflush(stdout);
        pos = 0;
    } else {
        buf[pos++] = c;
    }
}

// ── PS-EXE sideloader ─────────────────────────────────────────────────────────
// Parses a standard PS-EXE header (2 KiB), copies the text section into RAM,
// and returns the initial PC / GP / SP values so the caller can initialise the
// CPU registers before entering the run loop.
//
// PS-EXE header layout (nocash/psx-spx — actual NOCASH offsets):
//   +0x000  "PS-X EXE\0\0\0\0\0\0\0\0"  magic (16 bytes)
//   +0x010  u32 pc0      initial PC
//   +0x014  u32 gp0      initial GP (r28)
//   +0x018  u32 t_addr   destination address in RAM
//   +0x01C  u32 t_size   text section byte count
//   +0x030  u32 s_addr   initial SP base  (0 = caller supplies default)
//   +0x034  u32 s_size   SP offset added to s_addr
// Text section begins at file offset 0x800 (immediately after the 2 KiB header).
Bus::PsxExeInfo Bus::sideload(const char* path) noexcept {
    PsxExeInfo info{};
    FILE* f = std::fopen(path, "rb");
    if (!f) {
        std::fprintf(stderr, "[PSX-EXE] cannot open '%s'\n", path);
        return info;
    }

    u8 hdr[0x800];
    if (std::fread(hdr, 1u, sizeof(hdr), f) != sizeof(hdr)) {
        std::fprintf(stderr, "[PSX-EXE] too short: '%s'\n", path);
        std::fclose(f); return info;
    }
    if (std::memcmp(hdr, "PS-X EXE", 8) != 0) {
        std::fprintf(stderr, "[PSX-EXE] bad magic: '%s'\n", path);
        std::fclose(f); return info;
    }

    // Helper: read a u32 from a given byte offset in the header.
    auto r32 = [&](u32 off) noexcept -> u32 {
        u32 v = 0u;
        std::memcpy(&v, hdr + off, 4u);
        return v;
    };

    // NOCASH PSX-SPX layout:
    //   0x010  u32  Initial PC
    //   0x014  u32  Initial GP/R28
    //   0x018  u32  Load address (destination in RAM)
    //   0x01C  u32  Text section size (code+data+padding)
    //   0x020  u32  Data section addr  (unused, always 0)
    //   0x024  u32  Data section size  (unused, always 0)
    //   0x030  u32  Initial SP base
    //   0x034  u32  Initial SP offset
    const u32 t_addr = r32(0x018u) & 0x001F'FFFFu;  // strip KSEG0/KSEG1 prefix
    const u32 t_size = r32(0x01Cu);

    // Copy text section word by word into RAM.
    for (u32 i = 0u; i < t_size; i += 4u) {
        u32 word = 0u;
        if (std::fread(&word, 4u, 1u, f) != 1u) break;
        ram_->write<u32>((t_addr + i) & (Ram::SIZE - 1u), word);
    }
    std::fclose(f);

    info.pc = r32(0x010u);
    info.gp = r32(0x014u);
    const u32 sp_base = r32(0x030u);
    const u32 sp_off  = r32(0x034u);
    info.sp = (sp_base != 0u) ? (sp_base + sp_off) : 0u;
    info.ok = true;

    // ── Sideload bootstrap stubs ──────────────────────────────────────────────
    // PSN00BSDK uses BIOS A/B/C function dispatch vectors even in "bare-metal"
    // mode.  We install minimal stubs in low RAM so these calls return cleanly.
    //
    // The exception handler is extended to dispatch IRQs to a PSN00BSDK-
    // registered ISR.  PSN00BSDK calls B(0x19)=SetCustomExitFromException(ptr)
    // which stores a struct {fn, ?, ?, callback_table} at a known address.
    // When an IRQ fires our handler reads the struct, sets up $s0/$s1 for the
    // PSN00BSDK irq_dispatch loop, saves $s0/$s1/$ra/EPC for B(0x17)'s
    // restoration, then jumps into the dispatcher.  B(0x17)=ReturnFromException
    // restores the saved state and returns to EPC via rfe.
    //
    // Memory layout installed at load time (all physical addresses):
    //
    //   0x0000'0080  Exception vector (BEV=0): J → extended handler at 0x120
    //   0x0000'00A0  BIOS A stub: jr $ra
    //   0x0000'00B0  BIOS B stub: J → B dispatcher at 0x190
    //   0x0000'00C0  BIOS C stub: jr $ra
    //
    //   0x0000'0100  BIOS kernel variables:
    //                  +0x00  saved $s0     (written by IRQ exception handler)
    //                  +0x04  saved $s1
    //                  +0x08  Process*      (fixed = 0x0400; getCurrentThread reads here)
    //                  +0x0C  saved EPC     (IRQ path; read by B(0x17))
    //                  +0x10  ISR struct ptr (written by B(0x19))
    //                  +0x14  saved $ra     (IRQ path; read by B(0x17))
    //
    //   0x0000'01E4  Return-from-exception stub (8 instructions):
    //                  reads returnPC from Thread struct, returns via rfe+jr
    //
    //   0x0000'0300  hookUnresolvedExceptionHandler slot (A0[0x40]):
    //                  written at runtime by the test's hookUnresolvedExceptionHandler()
    //
    //   0x0000'0400  Process struct: { Thread* thread = 0x0404 }
    //   0x0000'0404  Thread struct  (0xC0 bytes, zero-initialised)
    //
    //   0x0000'0120  Extended exception handler (27 instructions)
    //   0x0000'0190  B-function dispatcher (21 instructions)
    //
    // Registers used ($k0=26, $k1=27 are kernel-use; safe in exception context):
    //   jr $ra       = 0x03E0'0008    jr $k0   = 0x0340'0008
    //   mfc0 $k0,$13 = 0x401A'6800    (Cause)
    //   mfc0 $k0,$14 = 0x401A'7000    (EPC)    mfc0 $k1,$14 = 0x401B'7000
    //   rfe          = 0x4200'0010    nop      = 0x0000'0000

    auto wram = [&](u32 phys, u32 val) noexcept {
        ram_->write<u32>(phys & (Ram::SIZE - 1u), val);
    };

    // ── Exception vector at 0x80 ──────────────────────────────────────────────
    // J 0x80000120: opcode=2, target = (0x120>>2) = 0x48  →  0x0800'0048
    wram(0x0000'0080u, 0x08000048u);   // j   0x80000120
    wram(0x0000'0084u, 0x00000000u);   // nop

    // ── BIOS A stub — default no-op return ────────────────────────────────────
    wram(0x0000'00A0u, 0x03E00008u);   // jr  $ra
    wram(0x0000'00A4u, 0x00000000u);   // nop

    // ── BIOS B stub — jump to full B dispatcher ───────────────────────────────
    // J 0x80000190: target = (0x190>>2) = 0x64  →  0x0800'0064
    wram(0x0000'00B0u, 0x08000064u);   // j   0x80000190
    wram(0x0000'00B4u, 0x00000000u);   // nop

    // ── BIOS C stub — default no-op return ───────────────────────────────────
    wram(0x0000'00C0u, 0x03E00008u);   // jr  $ra
    wram(0x0000'00C4u, 0x00000000u);   // nop

    // ── Extended exception handler at 0x0000_0120 ─────────────────────────────
    // Reads Cause.ExcCode; for IRQ (ExcCode==0) jumps to the full-context-save
    // trampoline at 0x0580 which saves ALL caller-saved registers before entering
    // the ISR.  For non-IRQ exceptions returns directly to EPC.
    //
    // Pseudo-code:
    //   k0 = (Cause >> 2) & 0x1F   // ExcCode
    //   if (k0 != 0) goto fault     // non-IRQ: straight return
    //   k1 = EPC                    // (delay slot: always computed)
    //   jr k1 / rfe                 // fault path returns here
    //  irq:
    //   k0 = mem[0x110]             // ISR struct ptr (B(0x19) registration)
    //   if (k0 == 0) goto no_isr   // nothing registered
    //   j 0x80000580               // → full-save trampoline (saves ALL caller-saved regs)
    //  no_isr:
    //   k0 = EPC; jr k0; rfe       // return to interrupted code
    wram(0x0000'0120u, 0x401A6800u);   // [0]  mfc0 $k0, $13  (Cause)
    wram(0x0000'0124u, 0x001AD082u);   // [1]  srl  $k0, $k0, 2
    wram(0x0000'0128u, 0x335A001Fu);   // [2]  andi $k0, $k0, 0x1F
    wram(0x0000'012Cu, 0x13400003u);   // [3]  beq  $k0, $zero, +3  (→ irq at 0x013C)
    wram(0x0000'0130u, 0x401B7000u);   // [4]  mfc0 $k1, $14  [delay slot]
    wram(0x0000'0134u, 0x03600008u);   // [5]  jr   $k1        (non-IRQ: return to EPC)
    wram(0x0000'0138u, 0x42000010u);   // [6]  rfe             [delay slot]
    // irq:
    wram(0x0000'013Cu, 0x3C1A0000u);   // [7]  lui  $k0, 0
    wram(0x0000'0140u, 0x8F5A0110u);   // [8]  lw   $k0, 0x110($k0)  (struct ptr)
    wram(0x0000'0144u, 0x00000000u);   // [9]  nop  (load delay)
    wram(0x0000'0148u, 0x13400003u);   // [10] beq  $k0, $zero, +3  (→ no_isr at 0x0158)
    wram(0x0000'014Cu, 0x00000000u);   // [11] nop  [delay slot]
    // ISR path: jump to full-context-save trampoline (saves $at,$v0,$v1,$a0-$a3,$t0-$t9)
    wram(0x0000'0150u, 0x08000160u);   // [12] j    0x80000580  (full-save trampoline)
    wram(0x0000'0154u, 0x00000000u);   // [13] nop  [delay slot]
    // no_isr: no ISR registered — return directly to EPC
    wram(0x0000'0158u, 0x401A7000u);   // [14] mfc0 $k0, $14  (EPC)
    wram(0x0000'015Cu, 0x00000000u);   // [15] nop  (COP0 latency)
    wram(0x0000'0160u, 0x03400008u);   // [16] jr   $k0
    wram(0x0000'0164u, 0x42000010u);   // [17] rfe  [delay slot]
    // 0x0168-0x0188: padding (unreachable with new flow)

    // ── B-function dispatcher at 0x0000_0190 ──────────────────────────────────
    // Dispatches on $t1 (the BIOS function number loaded before "jr $t2").
    //
    // B(0x17) ReturnFromException — restores $s0/$s1/$ra from save area, loads
    //   EPC, returns to EPC via rfe.  Called by the PSN00BSDK irq_dispatch
    //   epilogue.  NB: EPC is saved at 0x10C; $ra is saved at 0x114.
    // B(0x18) SetDefaultExitFromException — clears ISR registration (mem[0x110]=0).
    // B(0x19) SetCustomExitFromException(ptr) — stores a0 (struct ptr) in mem[0x110].
    // All others — jr $ra (harmless no-op).
    //
    // Layout:
    //   check 0x19 → b19  (branch offset +7 after delay slot)
    //   check 0x17 → b17  (branch offset +8 after delay slot)
    //   check 0x18 → b18  (branch offset +10 after delay slot)
    //   jr $ra             (default)
    //  b19: sw $a0, 0x110($zero); jr $ra
    //  b17: lw $s0,0x100; lw $s1,0x104; lw $k0,0x10C; lw $ra,0x114; jr $k0; rfe
    //  b18: sw $zero,0x110($zero); jr $ra
    //
    // Registers: $t1=9 (fn#), $at=1 (scratch), $a0=4 (arg to B(0x19))
    wram(0x0000'0190u, 0x24010019u);   // [0]  addiu $at, $zero, 0x19
    wram(0x0000'0194u, 0x10290007u);   // [1]  beq   $at, $t1, +7   (→ b19)
    wram(0x0000'0198u, 0x24010017u);   // [2]  addiu $at, $zero, 0x17  [delay]
    wram(0x0000'019Cu, 0x10290008u);   // [3]  beq   $at, $t1, +8   (→ b17)
    wram(0x0000'01A0u, 0x24010018u);   // [4]  addiu $at, $zero, 0x18  [delay]
    wram(0x0000'01A4u, 0x1029000Cu);   // [5]  beq   $at, $t1, +12  (→ b18)
    wram(0x0000'01A8u, 0x00000000u);   // [6]  nop   [delay]
    wram(0x0000'01ACu, 0x03E00008u);   // [7]  jr    $ra  (default: unknown B fn)
    wram(0x0000'01B0u, 0x00000000u);   // [8]  nop
    // b19: SetCustomExitFromException(ptr) — store a0 in ISR slot
    wram(0x0000'01B4u, 0xAC040110u);   // [9]  sw    $a0, 0x110($zero)
    wram(0x0000'01B8u, 0x03E00008u);   // [10] jr    $ra
    wram(0x0000'01BCu, 0x00000000u);   // [11] nop
    // b17: ReturnFromException — jump to full-context-restore routine at 0x80000650
    // That routine restores $at,$v0,$v1,$a0-$a3,$t0-$t9,$s0,$s1,$ra then returns
    // to the saved EPC via rfe, leaving the interrupted program's registers intact.
    wram(0x0000'01C0u, 0x08000194u);   // [12] j    0x80000650  (full-restore routine)
    wram(0x0000'01C4u, 0x00000000u);   // [13] nop  [delay slot]
    // [14]-[17] at 0x01C8-0x01D4: unreachable padding
    // b18: SetDefaultExitFromException — clear ISR registration
    wram(0x0000'01D8u, 0xAC000110u);   // [18] sw    $zero, 0x110($zero)
    wram(0x0000'01DCu, 0x03E00008u);   // [19] jr    $ra
    wram(0x0000'01E0u, 0x00000000u);   // [20] nop

    // ── Return-from-exception stub at 0x0000_00D0 ─────────────────────────────
    // Called via $ra = 0x8000_00D0 when handle_non_irq_exception() dispatches a
    // user handler.  After the handler returns (jr $ra → here), we:
    //   1. Restore $ra from Thread.registers.r[31] (the original $ra before the
    //      exception), so callers after the exception resume with the right link.
    //   2. Load Thread.registers.returnPC (set by the exception handler) into $k0.
    //   3. Jump to returnPC; rfe in the delay slot restores SR exception bits.
    //
    // Placed at 0x00D0 (after the C stub at 0x00C0–0x00C8) to avoid the BIOS
    // globals table which starts at 0x0200.  0x00D0–0x00F7 (10 instructions).
    //
    //   lw $k1, 0x108($zero)   ; k1 = Process*
    //   nop                    ; load delay
    //   lw $k1, 0x0($k1)       ; k1 = Thread*
    //   nop                    ; load delay
    //   lw $ra, 0x84($k1)      ; $ra = Thread.registers.r[31] (original $ra)
    //   nop                    ; load delay
    //   lw $k0, 0x88($k1)      ; k0 = Thread.registers.returnPC
    //   nop                    ; load delay
    //   jr   $k0               ; jump to returnPC
    //   rfe                    ; [delay slot] restore SR
    //
    // Encodings:
    //   lw $k1, 0x108($zero): rt=27, rs=0,  imm=0x108  → 0x8C1B0108
    //   lw $k1, 0x0($k1):     rt=27, rs=27, imm=0      → 0x8F7B0000
    //   lw $ra, 0x84($k1):    rt=31, rs=27, imm=0x84   → 0x8F7F0084
    //   lw $k0, 0x88($k1):    rt=26, rs=27, imm=0x88   → 0x8F7A0088
    //   jr $k0:                                          → 0x03400008
    //   rfe:                                             → 0x42000010
    wram(0x0000'00D0u, 0x8C1B0108u);   // lw   $k1, 0x108($zero)
    wram(0x0000'00D4u, 0x00000000u);   // nop
    wram(0x0000'00D8u, 0x8F7B0000u);   // lw   $k1, 0x0($k1)
    wram(0x0000'00DCu, 0x00000000u);   // nop
    wram(0x0000'00E0u, 0x8F7F0084u);   // lw   $ra,  0x84($k1)   ← restore orig $ra
    wram(0x0000'00E4u, 0x00000000u);   // nop
    wram(0x0000'00E8u, 0x8F7A0088u);   // lw   $k0, 0x88($k1)
    wram(0x0000'00ECu, 0x00000000u);   // nop
    wram(0x0000'00F0u, 0x03400008u);   // jr   $k0
    wram(0x0000'00F4u, 0x42000010u);   // rfe  [delay slot]

    // ── BIOS kernel-data area — Process/Thread struct chain ───────────────────
    // Initialise the data that getCurrentThread() in exception.cpp reads:
    //   Process** processes = (Process**)0x108;   // reads 4 bytes at 0x108
    //   return processes[0]->thread;              // follows pointer, reads thread
    //
    // We place a minimal Process struct at 0x0400 and Thread struct at 0x0404.
    // RAM[0x108] = 0x0400 (Process*)
    // RAM[0x400] = 0x0404 (Process.thread = Thread*)
    // Thread struct (0xC0 bytes from 0x0404) starts zero-initialised.
    //
    // NB: 0x108 previously held the IRQ-path's saved EPC.  That slot is now
    // at 0x10C (the [16] sw and B(0x17) [14] lw above were updated accordingly).
    wram(0x0000'0108u, 0x0000'0400u);  // RAM[0x108] = Process struct address
    wram(0x0000'0400u, 0x0000'0404u);  // Process.thread = Thread struct address
    // Thread struct at 0x0404 (0xC0 bytes) is already zero from RAM reset.

    // ── Fake C0 function table at 0x0000_0500 ────────────────────────────────
    // B(0x5B) = GetC0Table() returns 0x0500.  The table has 32 entries; each
    // entry is a pointer to our "jr $ra; nop" stub (0x000000A0) so that any
    // call into the C0 table is a harmless no-op.
    for (u32 i = 0; i < 32u; ++i) {
        wram(0x0000'0500u + i * 4u, 0x0000'00A0u);
    }

    // ── Full-context-save trampoline at 0x0000_0580 ───────────────────────────
    // Entered via j from exception handler when an IRQ fires and an ISR is
    // registered.  Saves ALL caller-saved registers ($at,$v0,$v1,$a0-$a3,$t0-$t9)
    // to 0x600-0x640, then saves $s0/$s1/EPC/$ra and enters the ISR.
    //
    // Save area layout (physical RAM):
    //   0x0600: $at  0x0604: $v0  0x0608: $v1
    //   0x060C: $a0  0x0610: $a1  0x0614: $a2  0x0618: $a3
    //   0x061C: $t0  0x0620: $t1  0x0624: $t2  0x0628: $t3
    //   0x062C: $t4  0x0630: $t5  0x0634: $t6  0x0638: $t7
    //   0x063C: $t8  0x0640: $t9
    //
    // On entry: $k0 = ISR struct pointer (set by the exception handler).
    wram(0x0000'0580u, 0xAC010600u);   // [0]  sw   $at, 0x600($zero)
    wram(0x0000'0584u, 0xAC020604u);   // [1]  sw   $v0, 0x604($zero)
    wram(0x0000'0588u, 0xAC030608u);   // [2]  sw   $v1, 0x608($zero)
    wram(0x0000'058Cu, 0xAC04060Cu);   // [3]  sw   $a0, 0x60C($zero)
    wram(0x0000'0590u, 0xAC050610u);   // [4]  sw   $a1, 0x610($zero)
    wram(0x0000'0594u, 0xAC060614u);   // [5]  sw   $a2, 0x614($zero)
    wram(0x0000'0598u, 0xAC070618u);   // [6]  sw   $a3, 0x618($zero)
    wram(0x0000'059Cu, 0xAC08061Cu);   // [7]  sw   $t0, 0x61C($zero)
    wram(0x0000'05A0u, 0xAC090620u);   // [8]  sw   $t1, 0x620($zero)
    wram(0x0000'05A4u, 0xAC0A0624u);   // [9]  sw   $t2, 0x624($zero)
    wram(0x0000'05A8u, 0xAC0B0628u);   // [10] sw   $t3, 0x628($zero)
    wram(0x0000'05ACu, 0xAC0C062Cu);   // [11] sw   $t4, 0x62C($zero)
    wram(0x0000'05B0u, 0xAC0D0630u);   // [12] sw   $t5, 0x630($zero)
    wram(0x0000'05B4u, 0xAC0E0634u);   // [13] sw   $t6, 0x634($zero)
    wram(0x0000'05B8u, 0xAC0F0638u);   // [14] sw   $t7, 0x638($zero)
    wram(0x0000'05BCu, 0xAC18063Cu);   // [15] sw   $t8, 0x63C($zero)
    wram(0x0000'05C0u, 0xAC190640u);   // [16] sw   $t9, 0x640($zero)
    wram(0x0000'05C4u, 0xAC100100u);   // [17] sw   $s0, 0x100($zero)
    wram(0x0000'05C8u, 0xAC110104u);   // [18] sw   $s1, 0x104($zero)
    wram(0x0000'05CCu, 0x401B7000u);   // [19] mfc0 $k1, $14  (EPC)
    wram(0x0000'05D0u, 0x00000000u);   // [20] nop  (COP0 latency)
    wram(0x0000'05D4u, 0xAC1B010Cu);   // [21] sw   $k1, 0x10C($zero)  (save EPC)
    wram(0x0000'05D8u, 0x8F5B000Cu);   // [22] lw   $k1, 0xC($k0)   (callback tbl; load delay ok — $k1 used 2 insns later)
    wram(0x0000'05DCu, 0x8F5A0000u);   // [23] lw   $k0, 0x0($k0)   (ISR fn addr; uses OLD $k0 ✓)
    wram(0x0000'05E0u, 0x03608025u);   // [24] or   $s0, $k1, $zero  ($s0=callback_tbl; $k1 valid here)
    wram(0x0000'05E4u, 0x24110000u);   // [25] addiu $s1, $zero, 0   ($s1=0; $k0 now valid)
    wram(0x0000'05E8u, 0x03400008u);   // [26] jr   $k0  → enter ISR
    wram(0x0000'05ECu, 0xAC1F0114u);   // [27] sw   $ra, 0x114($zero)  [delay: save $ra before ISR clobbers it]

    // ── Full-context-restore routine at 0x0000_0650 ───────────────────────────
    // Jumped to by B(0x17) (ReturnFromException).  Restores all caller-saved
    // registers from the save area then returns to the saved EPC via rfe,
    // leaving the interrupted program's register state completely intact.
    wram(0x0000'0650u, 0x8C010600u);   // [0]  lw   $at, 0x600($zero)
    wram(0x0000'0654u, 0x8C020604u);   // [1]  lw   $v0, 0x604($zero)
    wram(0x0000'0658u, 0x8C030608u);   // [2]  lw   $v1, 0x608($zero)
    wram(0x0000'065Cu, 0x8C04060Cu);   // [3]  lw   $a0, 0x60C($zero)
    wram(0x0000'0660u, 0x8C050610u);   // [4]  lw   $a1, 0x610($zero)
    wram(0x0000'0664u, 0x8C060614u);   // [5]  lw   $a2, 0x614($zero)
    wram(0x0000'0668u, 0x8C070618u);   // [6]  lw   $a3, 0x618($zero)
    wram(0x0000'066Cu, 0x8C08061Cu);   // [7]  lw   $t0, 0x61C($zero)
    wram(0x0000'0670u, 0x8C090620u);   // [8]  lw   $t1, 0x620($zero)
    wram(0x0000'0674u, 0x8C0A0624u);   // [9]  lw   $t2, 0x624($zero)
    wram(0x0000'0678u, 0x8C0B0628u);   // [10] lw   $t3, 0x628($zero)
    wram(0x0000'067Cu, 0x8C0C062Cu);   // [11] lw   $t4, 0x62C($zero)
    wram(0x0000'0680u, 0x8C0D0630u);   // [12] lw   $t5, 0x630($zero)
    wram(0x0000'0684u, 0x8C0E0634u);   // [13] lw   $t6, 0x634($zero)
    wram(0x0000'0688u, 0x8C0F0638u);   // [14] lw   $t7, 0x638($zero)
    wram(0x0000'068Cu, 0x8C18063Cu);   // [15] lw   $t8, 0x63C($zero)
    wram(0x0000'0690u, 0x8C190640u);   // [16] lw   $t9, 0x640($zero)
    wram(0x0000'0694u, 0x8C100100u);   // [17] lw   $s0, 0x100($zero)
    wram(0x0000'0698u, 0x8C110104u);   // [18] lw   $s1, 0x104($zero)
    wram(0x0000'069Cu, 0x8C1A010Cu);   // [19] lw   $k0, 0x10C($zero)  (EPC)
    wram(0x0000'06A0u, 0x8C1F0114u);   // [20] lw   $ra, 0x114($zero)  (fills $k0 load-delay slot)
    wram(0x0000'06A4u, 0x03400008u);   // [21] jr   $k0  ($k0 loaded 2 insns ago — valid)
    wram(0x0000'06A8u, 0x42000010u);   // [22] rfe  [delay slot — restores SR exception bits]

    std::fprintf(stdout, "[PSX-EXE] PC=0x%08X GP=0x%08X loaded '%s'\n",
                 info.pc, info.gp, path);
    return info;
}

// ── Bus::tick() ───────────────────────────────────────────────────────────────
// Called once per CPU instruction (≈1 system-clock cycle for our purposes).
// Generates HBlank begin/end and VBlank begin/end events for timer sync modes.
void Bus::tick(u32 cycles) noexcept {
    // ── HBlank timing ─────────────────────────────────────────────────────────
    hblank_cycles_ += cycles;
    while (hblank_cycles_ >= kHBlankPeriod) {
        hblank_cycles_ -= kHBlankPeriod;
        if (!in_hblank_) {
            in_hblank_     = true;
            hblank_active_ = 0;
            timers_->hblank_begin();
        }
    }
    if (in_hblank_) {
        hblank_active_ += cycles;
        if (hblank_active_ >= kHBlankDuration) {
            in_hblank_     = false;
            hblank_active_ = 0;
            timers_->hblank_end();
        }
    }

    // ── Root counters (sys clock, dotclock, sys/8) ────────────────────────────
    timers_->tick(cycles);

    // ── CD-ROM response delays ────────────────────────────────────────────────
    cdrom_->tick(cycles);

    // ── SPU tick ─────────────────────────────────────────────────────────────
    // SPU runs at 44.1kHz, not CPU speed. Tick at ~768 CPU cycles per sample.
    spu_cycles_ += cycles;
    if (spu_cycles_ >= 768u) {
        spu_cycles_ -= 768u;
        spu_->tick();
    }

    // ── VBlank timing ─────────────────────────────────────────────────────────
    vblank_cycles_ += cycles;
    if (vblank_cycles_ >= kVBlankPeriod) {
        vblank_cycles_ -= kVBlankPeriod;
        if (!in_vblank_) {
            in_vblank_     = true;
            vblank_active_ = 0;
            irq_->set(IRQSource::VBlank);
            timers_->vblank_begin();
            gpu_->toggle_field();
            spu_->mix(nullptr, SPU_SAMPLES_PER_FRAME);
        }
    }
    if (in_vblank_) {
        vblank_active_ += cycles;
        if (vblank_active_ >= kVBlankDuration) {
            in_vblank_     = false;
            vblank_active_ = 0;
            timers_->vblank_end();
        }
    }
}
