#include "CDRom.hpp"
#include <cstring>
#include <cstdio>

// ── Status register ───────────────────────────────────────────────────────────
u8 CDRom::status() const noexcept {
    u8 s = 0x18u;  // PRMEMPT | PRMWRDY always asserted
    if (resp_pos_ < resp_len_) s |= 0x20u;   // RSLRRDY: response FIFO non-empty
    if (data_ready_)           s |= 0x40u;   // DRQSTS:  data FIFO non-empty
    return s;
}

// Approximate CDROM response delays in CPU cycles (@33.868 MHz).
// First response (INT3): ~50K cycles typical, 75K for Init/Reset.
// Second response delay from when s1 fires:
//   Init complete: ~400K   Seek complete: ~500K
//   Pause complete: ~900K  Stop complete: ~2M
// Sector read delays (1×/2× speed):
//   First INT1 from INT3: ~950K/520K   Per sector: ~448K/223K
static constexpr u32 kAck1    =  50'000u;
static constexpr u32 kAck1Init=  75'000u;
static constexpr u32 kAck1Read=  35'000u;
static constexpr u32 kD2Init  = 400'000u;
static constexpr u32 kD2Seek  = 500'000u;
static constexpr u32 kD2Pause = 900'000u;
static constexpr u32 kD2Stop  =2'000'000u;
static constexpr u32 kSect1_1x= 950'000u;   // first sector, 1×
static constexpr u32 kSectN_1x= 447'600u;   // per sector, 1×
static constexpr u32 kSect1_2x= 520'000u;   // first sector, 2×
static constexpr u32 kSectN_2x= 223'000u;   // per sector, 2×

// ── Multi-byte response + fire CDRom IRQ immediately ─────────────────────────
void CDRom::push_response_n(u8 int_type, const u8* data, u32 len) noexcept {
    resp_len_ = static_cast<u8>(len < 8u ? len : 8u);
    for (u32 i = 0u; i < static_cast<u32>(resp_len_); ++i)
        resp_[i] = data[i];
    resp_pos_ = 0u;
    irq_fl_   = int_type & 0x07u;
    // Only assert the hardware IRQ line when the INT type is enabled by irq_en_.
    // irq_fl_ holds the INT type NUMBER (1–5); irq_en_ bit N enables INT(N+1).
    if (irq_fl_ != 0u && (irq_en_ & (1u << (irq_fl_ - 1u))) != 0u)
        irq_.set(IRQSource::CDROM);
}

// ── Schedule first response after delay1 cycles ───────────────────────────────
void CDRom::sched(u8 type1, const u8* d1, u32 l1, u32 delay1) noexcept {
    s1_.len = static_cast<u8>(l1 < 8u ? l1 : 8u);
    for (u32 i = 0u; i < s1_.len; ++i) s1_.data[i] = d1[i];
    s1_.type  = type1;
    s1_.delay = delay1;
    s1_.active= true;
}

// ── Schedule second response; delay2 starts counting after s1 fires ──────────
void CDRom::sched2(u8 type2, const u8* d2, u32 l2, u32 delay2) noexcept {
    s2_.len = static_cast<u8>(l2 < 8u ? l2 : 8u);
    for (u32 i = 0u; i < s2_.len; ++i) s2_.data[i] = d2[i];
    s2_.type  = type2;
    s2_.delay = delay2;
    s2_.active= true;
}

// ── Timing tick: advance pending response countdowns ─────────────────────────
void CDRom::tick(u32 cycles) noexcept {
    // First response countdown.
    if (s1_.active) {
        if (cycles >= s1_.delay) {
            s1_.active = false;
            s1_.delay  = 0;
            push_response_n(s1_.type, s1_.data.data(), s1_.len);
            // After s1 fires, s2 delay begins.
        } else {
            s1_.delay -= cycles;
            return;  // s2 and reading don't progress until s1 fires
        }
    }

    // Second response countdown (starts after s1 fires).
    if (s2_.active) {
        if (cycles >= s2_.delay) {
            s2_.active = false;
            s2_.delay  = 0;
            // For ReadN: INT1 fires here (first sector).
            if (reading_) {
                read_sector();
                const u8 r = static_cast<u8>(cd_stat_ | 0x20u);
                push_response_n(1u, &r, 1u);
                rd_delay_  = rd_period_;   // schedule next sector
            } else {
                push_response_n(s2_.type, s2_.data.data(), s2_.len);
            }
        } else {
            s2_.delay -= cycles;
            return;  // rd_delay_ doesn't run while s2 is pending
        }
    }

    // Continuous sector delivery (INT1 stream after first sector).
    if (reading_ && rd_delay_ > 0) {
        if (cycles >= rd_delay_) {
            rd_delay_ = 0;
            if (read_sector()) {
                const u8 r = static_cast<u8>(cd_stat_ | 0x20u);
                push_response_n(1u, &r, 1u);
                rd_delay_ = rd_period_;
            } else {
                reading_ = false;
            }
        } else {
            rd_delay_ -= cycles;
        }
    }
}

// ── Load disc image ───────────────────────────────────────────────────────────
bool CDRom::load_disc(const char* path) noexcept {
    if (disc_) { std::fclose(disc_); disc_ = nullptr; }

    disc_ = std::fopen(path, "rb");
    if (!disc_) {
        std::fprintf(stderr, "[CDRom] cannot open '%s'\n", path);
        return false;
    }

    // Determine sector size from the file extension.
    const char* dot = std::strrchr(path, '.');
    disc_bin_ = dot &&
        (std::strcmp(dot, ".bin") == 0 || std::strcmp(dot, ".BIN") == 0);

    cd_stat_ = 0x02u;  // MOTOR_ON; disc present; no error; shell closed
    std::fprintf(stdout, "[CDRom] loaded '%s' (%s format)\n",
                 path, disc_bin_ ? "BIN/2352" : "ISO/2048");
    return true;
}

// ── Read 2048 bytes at seek_lba_ into data_buf_ ───────────────────────────────
bool CDRom::read_sector() noexcept {
    if (!disc_) {
        // No disc image: fill with zeroed dummy data so timing tests can proceed.
        data_buf_.fill(0u);
        data_pos_   = 0u;
        data_ready_ = true;
        seek_lba_  += 1u;
        return true;
    }

    // BIN: 2352 B/sector; user data begins 24 bytes into each sector.
    // ISO: 2048 B/sector; data is contiguous from the sector start.
    const long off = disc_bin_
        ? static_cast<long>(seek_lba_) * 2352L + 24L
        : static_cast<long>(seek_lba_) * 2048L;

    if (std::fseek(disc_, off, SEEK_SET) != 0) return false;
    const u32 got = static_cast<u32>(
        std::fread(data_buf_.data(), 1u, SECTOR_SIZE, disc_));
    if (got < SECTOR_SIZE) return false;

    data_pos_   = 0u;
    data_ready_ = true;
    seek_lba_  += 1u;
    return true;
}

// ── DMA ch3: drain one 32-bit word from the sector buffer ────────────────────
u32 CDRom::read_data_word() noexcept {
    if (data_pos_ + 4u > SECTOR_SIZE) return 0u;
    const u32 w =  static_cast<u32>(data_buf_[data_pos_    ])
                | (static_cast<u32>(data_buf_[data_pos_ + 1u]) <<  8u)
                | (static_cast<u32>(data_buf_[data_pos_ + 2u]) << 16u)
                | (static_cast<u32>(data_buf_[data_pos_ + 3u]) << 24u);
    data_pos_ += 4u;
    if (data_pos_ >= SECTOR_SIZE) data_ready_ = false;
    return w;
}

// ── Command dispatch ──────────────────────────────────────────────────────────
void CDRom::handle_command(u8 cmd) noexcept {
    // Snapshot current params then reset the FIFO for the next command.
    const u8 p0 = (param_len_ > 0u) ? params_[0] : 0u;
    const u8 p1 = (param_len_ > 1u) ? params_[1] : 0u;
    const u8 p2 = (param_len_ > 2u) ? params_[2] : 0u;
    param_len_ = 0u;

    switch (cmd) {
    // ── 0x01 GetStat ─────────────────────────────────────────────────────────
    case 0x01:
        sched1(3u, cd_stat_, kAck1);
        break;

    // ── 0x02 Setloc — save seek target (BCD MSF → LBA) ───────────────────────
    case 0x02:
        seek_lba_ = msf_to_lba(p0, p1, p2);
        sched1(3u, cd_stat_, kAck1);
        break;

    // ── 0x06 ReadN / 0x1B ReadS — read sector, then queue INT1 ───────────────
    case 0x06:
    case 0x1B: {
        reading_   = true;
        cd_stat_   = static_cast<u8>(cd_stat_ | 0x20u);  // READING flag
        // Determine per-sector period from mode byte (bit 7 = double speed).
        const bool double_speed = (mode_ & 0x80u) != 0u;
        rd_period_ = double_speed ? kSectN_2x : kSectN_1x;
        // s2 fires the first INT1 after s1 (using reading_ path in tick).
        sched1(3u, cd_stat_, kAck1Read);
        sched2(1u, nullptr, 0u, double_speed ? kSect1_2x : kSect1_1x);
        break;
    }

    // ── 0x08 Stop ────────────────────────────────────────────────────────────
    case 0x08:
        reading_ = false;
        cd_stat_ = 0x00u;
        sched1(3u, cd_stat_, kAck1);
        sched2(2u, &cd_stat_, 1u, kD2Stop);
        break;

    // ── 0x09 Pause ───────────────────────────────────────────────────────────
    case 0x09:
        reading_ = false;
        cd_stat_ = static_cast<u8>(cd_stat_ & 0xDFu);  // clear READING
        sched1(3u, cd_stat_, kAck1);
        sched2(2u, &cd_stat_, 1u, kD2Pause);
        break;

    // ── 0x0A Init — reset to known-good state ────────────────────────────────
    case 0x0A:
        reading_ = false;
        cd_stat_ = 0x02u;   // motor on, disc present (dummy if no image loaded)
        mode_    = 0x20u;
        sched1(3u, cd_stat_, kAck1Init);
        sched2(2u, &cd_stat_, 1u, kD2Init);
        break;

    // ── 0x0B Mute / 0x0C Demute — audio volume (no-op for data games) ────────
    case 0x0B:
    case 0x0C:
        sched1(3u, cd_stat_, kAck1);
        break;

    // ── 0x0E Setmode — store mode byte (speed, XA, data size…) ──────────────
    case 0x0E:
        mode_ = p0;
        sched1(3u, cd_stat_, kAck1);
        break;

    // ── 0x0F Getparam — return mode + 4 padding bytes ────────────────────────
    case 0x0F: {
        const u8 r[5] = { cd_stat_, mode_, 0x00u, 0x00u, 0x00u };
        sched(3u, r, 5u, kAck1);
        break;
    }

    // ── 0x13 GetTN — first and last track numbers (BCD) ──────────────────────
    case 0x13: {
        const u8 r[3] = { cd_stat_, 0x01u, 0x01u };  // single-track disc
        sched(3u, r, 3u, kAck1);
        break;
    }

    // ── 0x14 GetTD — track N start position (MSF) ────────────────────────────
    case 0x14: {
        // Track 1 always starts at MSF 00:02:00 on the PSX (150-frame lead-in).
        const u8 r[3] = { cd_stat_, 0x00u, 0x02u };
        sched(3u, r, 3u, kAck1);
        break;
    }

    // ── 0x15 SeekL / 0x16 SeekP — seek to seek_lba_, INT3 then INT2 ──────────
    case 0x15:
    case 0x16:
        sched1(3u, cd_stat_, kAck1);
        sched2(2u, &cd_stat_, 1u, kD2Seek);
        break;

    // ── 0x1A GetID — disc type + region identifier ────────────────────────────
    case 0x1A: {
        sched1(3u, cd_stat_, kAck1);
        // Licensed data disc, SCEA (North America) region marker.
        const u8 r2[8] = { 0x02u, 0x00u, 0x20u, 0x00u, 'S', 'C', 'E', 'A' };
        sched2(2u, r2, 8u, kD2Seek);
        break;
    }

    // ── Unknown command — INT5 error ──────────────────────────────────────────
    default: {
        std::fprintf(stderr, "[CDRom] unknown command 0x%02X\n",
                     static_cast<u32>(cmd));
        const u8 err[2] = { static_cast<u8>(cd_stat_ | 0x01u), 0x40u };
        sched(5u, err, 2u, kAck1);
        break;
    }
    }
}

// ── Read (byte-wide, const) ───────────────────────────────────────────────────
u8 CDRom::read(u32 off) const noexcept {
    switch (off & 3u) {
    case 0:                              // Status register (index-independent)
        return status();

    case 1:                              // Response FIFO
        if (resp_pos_ < resp_len_)
            return resp_[resp_pos_++];
        return 0x00u;

    case 2:                              // Data FIFO (normally drained by DMA)
        return 0x00u;

    case 3:
        if (index_ == 0) return irq_en_;
        // index 1 (and 2/3): IRQ flag register; top 3 bits always read 1.
        return (irq_fl_ & 0x1Fu) | 0xE0u;
    }
    return 0x00u;
}

// ── Write (byte-wide) ─────────────────────────────────────────────────────────
void CDRom::write(u32 off, u8 value) noexcept {
    switch (off & 3u) {
    case 0:                              // Index register
        index_ = value & 3u;
        break;

    case 1:
        if (index_ == 0)
            handle_command(value);       // Command register
        // index 1: sound map data write — ignored
        break;

    case 2:
        if (index_ == 0) {
            // Parameter FIFO — accumulate up to 8 bytes before the command.
            if (param_len_ < 8u) params_[param_len_++] = value;
        }
        // index 1: sound map coding info — ignored
        break;

    case 3:
        if (index_ == 0) {
            irq_en_ = value & 0x1Fu;     // IRQ enable mask
            // If a pending INT type just became enabled, assert the IRQ line now.
            if (irq_fl_ != 0u && (irq_en_ & (1u << (irq_fl_ - 1u))) != 0u)
                irq_.set(IRQSource::CDROM);
        } else if (index_ == 1) {
            // Write-1-to-clear IRQ flags.
            irq_fl_ = static_cast<u8>(irq_fl_ & static_cast<u8>(~(value & 0x1Fu)));
            // s2/reading responses are delivered by tick(), no immediate action needed.
        }
        break;
    }
}
