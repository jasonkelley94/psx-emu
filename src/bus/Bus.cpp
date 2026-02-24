#include "Bus.hpp"
#include <cstring>

Bus::Bus(std::unique_ptr<Bios> bios)
    : ram_   (std::make_unique<Ram>())
    , bios_  (std::move(bios))
    , gpu_   (std::make_unique<GPU>())
    , irq_   (std::make_unique<IRQ>())
    , cdrom_ (std::make_unique<CDRom>(*irq_))
    , timers_(std::make_unique<Timers>(*irq_))
      // DMA constructed last — it holds non-owning references to the above.
    , dma_   (std::make_unique<DMA>(*ram_, *gpu_, *irq_, *cdrom_))
{}

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

    // ── Joypad / SIO (0x040–0x05F) ───────────────────────────────────────────
    // Return TX-ready with no RX data and no ACK — controller not connected.
    // The BIOS times out waiting for /ACK and continues to the shell.
    // SIO1 (0x050–0x05F): PSN00BSDK polls SIO_STAT(1) for TX-ready bits.
    if (off >= 0x040u && off < 0x060u) {
        if (off >= 0x050u)                 return 0x0000'0005u;  // SIO1_STAT: TX ready
        if ((off & ~0x3u) == 0x044u)       return 0x0000'0005u;  // JOY_STAT
        return 0u;
    }

    // ── Memory control registers — ignored, return 0 ─────────────────────────
    if (off < 0x024u || off == 0x060u) {
        return 0u;
    }

    // ── SPU (offsets 0xC00–0xE7F) ─────────────────────────────────────────────
    // Backed by spu_regs_[] so that writes are preserved on read-back.
    // This lets code-in-io's testCodeInSPU write jr-$ra to 0x1F801C00, then
    // execute from that address and return without a bus error.
    // SPUSTAT (0xDAA): bit 10 (busy) and bits [5:0] match SPUCNT so BIOS init
    // loop terminates — these read as 0 until a real write sets them.
    if (off >= 0xC00u && off < 0xE80u) {
        const u32 idx = off - 0xC00u;
        u32 v = 0u;
        std::memcpy(&v, spu_regs_.data() + idx, sizeof(u32));
        return v;
    }

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

    // ── Joypad / SIO (0x040–0x05F) — swallow writes ──────────────────────────
    if (off >= 0x040u && off < 0x060u) {
        return;
    }

    // ── SPU (0x1F80_1C00 – 0x1F80_1E7F) — register file, preserves writes ───
    if (off >= 0xC00u && off < 0xE80u) {
        const u32 idx = off - 0xC00u;
        std::memcpy(spu_regs_.data() + idx, &value, sizeof(u32));
        return;
    }

    // ── Expansion region 2 (0x1F80_2000) — POST port, debug output ───────────
    if (off >= 0x1000u && off < 0x2000u) {
        return;
    }

    std::fprintf(stderr, "[Bus] unhandled I/O write32 off=0x%04X val=0x%08X\n", off, value);
}

// ── MMIO write (16-bit) — used for halfword CPU writes to the I/O window ──────
// SPU and timer registers are all 16-bit; the BIOS and games use halfword
// stores to set them.  Route those here instead of logging a false error.
void Bus::io_write16(u32 phys, u16 value) noexcept {
    const u32 off = phys - PSX::IO_BASE;

    // ── Timers (16-bit registers) ─────────────────────────────────────────────
    if (off >= 0x100u && off < 0x130u) {
        timers_->write(off - 0x100u, static_cast<u32>(value));
        return;
    }

    // ── Joypad / SIO ──────────────────────────────────────────────────────────
    if (off >= 0x040u && off < 0x060u) return;

    // ── SPU (16-bit writes) — store into register file ───────────────────────
    if (off >= 0xC00u && off < 0xE80u) {
        const u32 idx = off - 0xC00u;
        std::memcpy(spu_regs_.data() + idx, &value, sizeof(u16));
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
    // SIO_DATA(1) at offset 0x050 carries PSN00BSDK printf byte output.
    if (off >= 0x040u && off < 0x060u) {
        if (off == 0x050u) tty_putchar(static_cast<char>(value));
        return;
    }

    // ── SPU (8-bit writes) — store into register file ────────────────────────
    if (off >= 0xC00u && off < 0xE80u) {
        spu_regs_[off - 0xC00u] = value;
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
    // PSN00BSDK irq_dispatch loop, saves $s0/$s1/EPC for B(0x17)'s restoration,
    // then jumps into the dispatcher.  B(0x17)=ReturnFromException restores the
    // saved state and returns to EPC via rfe.
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
    // Reads Cause.ExcCode; for IRQ (ExcCode==0) saves $s0/$s1/EPC and jumps
    // to the registered ISR.  For non-IRQ exceptions returns directly to EPC.
    //
    // Pseudo-code:
    //   k0 = (Cause >> 2) & 0x1F   // ExcCode
    //   if (k0 != 0) goto fault     // non-IRQ: straight return
    //   k1 = EPC                    // (delay slot: always computed)
    //   jr k1 / rfe                 // fault path returns here
    //  irq:
    //   k0 = mem[0x110]             // ISR struct ptr (B(0x19) registration)
    //   if (k0 == 0) goto no_isr   // nothing registered
    //   mem[0x100] = s0             // save interrupted program's s0
    //   mem[0x104] = s1             //   and s1
    //   k1 = EPC; mem[0x108] = k1  // save EPC for B(0x17)
    //   k1 = k0[0xC]               // callback table  → s0
    //   k0 = k0[0x0]               // ISR function addr
    //   s0 = k1; s1 = 0            // set up loop regs for irq_dispatch
    //   jr k0                      // enter ISR (ISR calls B(0x17) to return)
    //  no_isr:
    //   k0 = EPC; jr k0; rfe       // return to interrupted code
    wram(0x0000'0120u, 0x401A6800u);   // [0]  mfc0 $k0, $13  (Cause)
    wram(0x0000'0124u, 0x001AD082u);   // [1]  srl  $k0, $k0, 2
    wram(0x0000'0128u, 0x335A001Fu);   // [2]  andi $k0, $k0, 0x1F
    wram(0x0000'012Cu, 0x13400003u);   // [3]  beq  $k0, $zero, +3  (→ irq)
    wram(0x0000'0130u, 0x401B7000u);   // [4]  mfc0 $k1, $14  [delay slot]
    wram(0x0000'0134u, 0x03600008u);   // [5]  jr   $k1        (non-IRQ: return to EPC)
    wram(0x0000'0138u, 0x42000010u);   // [6]  rfe             [delay slot]
    // irq:
    wram(0x0000'013Cu, 0x3C1A0000u);   // [7]  lui  $k0, 0
    wram(0x0000'0140u, 0x8F5A0110u);   // [8]  lw   $k0, 0x110($k0)  (struct ptr)
    wram(0x0000'0144u, 0x00000000u);   // [9]  nop  (load delay)
    wram(0x0000'0148u, 0x1340000Cu);   // [10] beq  $k0, $zero, +12  (→ no_isr)
    wram(0x0000'014Cu, 0x00000000u);   // [11] nop  [delay slot]
    wram(0x0000'0150u, 0xAC100100u);   // [12] sw   $s0, 0x100($zero)
    wram(0x0000'0154u, 0xAC110104u);   // [13] sw   $s1, 0x104($zero)
    wram(0x0000'0158u, 0x401B7000u);   // [14] mfc0 $k1, $14  (EPC)
    wram(0x0000'015Cu, 0x00000000u);   // [15] nop  (COP0 load latency)
    wram(0x0000'0160u, 0xAC1B010Cu);   // [16] sw   $k1, 0x10C($zero)  (EPC save; 0x108=Process*)
    wram(0x0000'0164u, 0x8F5B000Cu);   // [17] lw   $k1, 0xC($k0)   (callback tbl)
    wram(0x0000'0168u, 0x8F5A0000u);   // [18] lw   $k0, 0x0($k0)   (ISR fn addr)
    wram(0x0000'016Cu, 0x03608025u);   // [19] or   $s0, $k1, $zero  (s0=callback tbl)
    wram(0x0000'0170u, 0x24110000u);   // [20] addiu $s1, $zero, 0   (s1=0, start bit)
    wram(0x0000'0174u, 0x03400008u);   // [21] jr   $k0   → ISR
    wram(0x0000'0178u, 0x00000000u);   // [22] nop  [delay slot]
    // no_isr:
    wram(0x0000'017Cu, 0x401A7000u);   // [23] mfc0 $k0, $14  (EPC)
    wram(0x0000'0180u, 0x00000000u);   // [24] nop
    wram(0x0000'0184u, 0x03400008u);   // [25] jr   $k0
    wram(0x0000'0188u, 0x42000010u);   // [26] rfe  [delay slot]

    // ── B-function dispatcher at 0x0000_0190 ──────────────────────────────────
    // Dispatches on $t1 (the BIOS function number loaded before "jr $t2").
    //
    // B(0x17) ReturnFromException — restores $s0/$s1 from save area, loads EPC,
    //   returns to EPC via rfe.  Called by the PSN00BSDK irq_dispatch epilogue.
    //   NB: EPC is now saved at 0x10C (not 0x108; that slot holds Process*).
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
    //  b17: lw $s0,0x100; lw $s1,0x104; lw $k0,0x10C; nop; jr $k0; rfe
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
    // b17: ReturnFromException — restore state and return to EPC
    wram(0x0000'01C0u, 0x8C100100u);   // [12] lw    $s0, 0x100($zero)
    wram(0x0000'01C4u, 0x8C110104u);   // [13] lw    $s1, 0x104($zero)
    wram(0x0000'01C8u, 0x8C1A010Cu);   // [14] lw    $k0, 0x10C($zero)  (EPC save slot)
    wram(0x0000'01CCu, 0x00000000u);   // [15] nop   (load delay for $k0)
    wram(0x0000'01D0u, 0x03400008u);   // [16] jr    $k0
    wram(0x0000'01D4u, 0x42000010u);   // [17] rfe   [delay slot]
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
