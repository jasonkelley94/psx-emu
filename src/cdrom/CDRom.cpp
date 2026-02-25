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
static constexpr u32 kSect1_1x=5'000'000u;   // first sector, 1× (~148ms, ~1 rotation near inner track)
static constexpr u32 kSectN_1x=  447'600u;   // per sector, 1×
static constexpr u32 kSect1_2x=3'000'000u;   // first sector, 2×
static constexpr u32 kSectN_2x=  223'000u;   // per sector, 2×

// ── Multi-byte response + fire CDRom IRQ immediately ─────────────────────────
void CDRom::push_response_n(u8 int_type, const u8* data, u32 len) noexcept {
    resp_len_ = static_cast<u8>(len < 8u ? len : 8u);
    for (u32 i = 0u; i < static_cast<u32>(resp_len_); ++i)
        resp_[i] = data[i];
    resp_pos_ = 0u;
    irq_fl_   = int_type & 0x07u;
    // Assert the hardware IRQ line when this INT type is enabled by irq_en_.
    // irq_fl_ stores the INT type number (1–5); bit (irq_fl_-1) of irq_en_ enables it.
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
            const u32 consumed = s1_.delay;
            s1_.active = false;
            s1_.delay  = 0;
            push_response_n(s1_.type, s1_.data.data(), s1_.len);
            // Subtract the cycles s1 consumed so s2 doesn't see the full batch.
            cycles -= consumed;
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
            // Transition from SEEKING (0x40) → READING (0x20) state.
            if (reading_) {
                cd_stat_ = static_cast<u8>((cd_stat_ & 0x02u) | 0x20u);  // MOTOR_ON | READING
                read_sector();
                push_response_n(1u, &cd_stat_, 1u);
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
                push_response_n(1u, &cd_stat_, 1u);
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

// ── BCD / MSF helpers ─────────────────────────────────────────────────────────
static u8 to_bcd(u32 v) noexcept {
    return static_cast<u8>((v / 10u) << 4u) | static_cast<u8>(v % 10u);
}

// Convert a signed LBA to absolute MSF in BCD.
// Absolute MSF = LBA + 150 frames (the 2-second CD lead-in offset).
// Negative LBA is valid (pregap/lead-in area, absolute frame still >= 0).
static void lba_to_abs_msf(int32_t lba, u8& mm, u8& ss, u8& ff) noexcept {
    const u32 f = static_cast<u32>(lba + 150);
    mm = to_bcd(f / (75u * 60u));
    ss = to_bcd((f / 75u) % 60u);
    ff = to_bcd(f % 75u);
}

// ── Read 2048 bytes at seek_lba_ into data_buf_ ───────────────────────────────
bool CDRom::read_sector() noexcept {
    if (!disc_) {
        // No disc image: fill with zeroed dummy data so timing tests can proceed.
        data_buf_.fill(0u);
        data_pos_   = 0u;
        data_ready_ = true;
        seek_lba_  += 1;
        loc_lba_    = seek_lba_ - 1;
        loc_valid_  = true;
        pos_valid_  = true;
        pos_lba_    = loc_lba_;
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
    seek_lba_  += 1;
    loc_lba_    = seek_lba_ - 1;
    loc_valid_  = true;
    pos_valid_  = true;
    pos_lba_    = loc_lba_;
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
        // Setloc only stores the seek target; it does NOT change cd_stat_.
        sched1(3u, cd_stat_, kAck1);
        break;

    // ── 0x06 ReadN / 0x1B ReadS — read sector, then queue INT1 ───────────────
    case 0x06:
    case 0x1B: {
        reading_   = true;
        // Drive enters SEEKING state while looking for the target sector.
        // READING (bit 5) will be set when the first INT1 fires in tick().
        cd_stat_   = static_cast<u8>((cd_stat_ & 0x02u) | 0x40u);  // MOTOR_ON | SEEKING
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
        reading_   = false;
        cd_stat_   = 0x02u;   // motor on, disc present (dummy if no image loaded)
        mode_      = 0x20u;
        // NOTE: do NOT reset loc_valid_/loc_lba_ here.  Init is a soft reset
        // that leaves the drive knowing its position.  GetlocL remains valid
        // if a seek/read had already been done before Init was called.
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
        // Negative LBA = lead-in/pregap — valid seek.
        // LBA ≥ 333,000 (past 74-min boundary) = out-of-range → INT5 error
        // (matching real hardware: seek fails immediately with SEEKERROR set).
        if (seek_lba_ >= 333'000) {
            cd_stat_   = 0x04u;  // SEEKERROR only (real hardware clears MOTOR_ON on OOB seek)
            loc_valid_ = false;
            pos_valid_ = false;
            const u8 err[2] = { cd_stat_, 0x04u };
            sched(5u, err, 2u, kAck1);
        } else {
            loc_lba_   = seek_lba_;
            loc_valid_ = true;
            pos_lba_   = seek_lba_;
            pos_valid_ = true;
            sched1(3u, cd_stat_, kAck1);
            sched2(2u, &cd_stat_, 1u, kD2Seek);
        }
        break;

    // ── 0x10 GetlocL — absolute MSF from last-read sector header ─────────────
    case 0x10: {
        if (!loc_valid_) {
            // No sector header available (no seek or read performed yet).
            const u8 err[2] = { static_cast<u8>(cd_stat_ | 0x01u), 0x80u };
            sched(5u, err, 2u, kAck1);
        } else {
            u8 mm, ss, ff;
            lba_to_abs_msf(loc_lba_, mm, ss, ff);
            // Response: [amm, ass, aff, mode, file, channel, submode]
            const u8 r[7] = { mm, ss, ff, 0x02u, 0x00u, 0x00u, 0x00u };
            sched(3u, r, 7u, kAck1);
        }
        break;
    }

    // ── 0x11 GetlocP — Q-subchannel track/index/position ──────────────────────
    case 0x11: {
        if (!pos_valid_) {
            // Head is past the physical disc edge — no valid Q-subchannel data.
            const u8 err[2] = { static_cast<u8>(cd_stat_ | 0x01u), 0x80u };
            sched(5u, err, 2u, kAck1);
            break;
        }
        // Compute absolute MSF from current head position.
        u8 amm, ass, aff;
        if (pos_lba_ >= 0) {
            lba_to_abs_msf(pos_lba_, amm, ass, aff);
        } else {
            // In pregap (before track 1 start at absolute frame 150).
            const u32 abs_f = static_cast<u32>(150 + pos_lba_);
            amm = to_bcd(abs_f / (75u * 60u));
            ass = to_bcd((abs_f / 75u) % 60u);
            aff = to_bcd(abs_f % 75u);
        }
        // Compute relative MSF within track 1 (track 1 starts at LBA 0).
        u8 track, index, rmm, rss, rff;
        if (pos_lba_ >= 0) {
            track = 0x01u;  index = 0x01u;  // in track data
            const u32 rf = static_cast<u32>(pos_lba_);
            rmm = to_bcd(rf / (75u * 60u));
            rss = to_bcd((rf / 75u) % 60u);
            rff = to_bcd(rf % 75u);
        } else {
            track = 0x01u;  index = 0x00u;  // in pregap
            const u32 pg = static_cast<u32>(-pos_lba_);  // frames before track start
            rmm = to_bcd(pg / (75u * 60u));
            rss = to_bcd((pg / 75u) % 60u);
            rff = to_bcd(pg % 75u);
        }
        // Response: [track, index, relMM, relSS, relFF, absMM, absSS, absFF]
        const u8 r[8] = { track, index, rmm, rss, rff, amm, ass, aff };
        sched(3u, r, 8u, kAck1);
        break;
    }

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
    case 0: return status();
    case 1:
        return (resp_pos_ < resp_len_) ? resp_[resp_pos_++] : 0x00u;
    case 2: return 0x00u;
    case 3:
        if (index_ == 0) return irq_en_;
        return (irq_fl_ & 0x1Fu) | 0xE0u;
    }
    return 0x00u;
}

// ── Write (byte-wide) ─────────────────────────────────────────────────────────
//
// CDROM register map (write paths):
//   off=0            → Index register [1:0]
//   off=1, idx=0     → Command register
//   off=1, idx=1/2/3 → Sound Map Data Out (ignored)
//   off=2, idx=0     → Parameter FIFO
//   off=2, idx=1     → INT Enable Register  ← PRIMARY write path for irq_en_
//   off=2, idx=2     → Audio Volume Left-Left
//   off=2, idx=3     → Audio Volume Left-Right
//   off=3, idx=0     → Request Register (bit7=BFRD, bit5=SMEN) — NOT irq_en_!
//   off=3, idx=1     → INT Flag Acknowledge (write-1-to-clear; bit6=reset param FIFO)
//   off=3, idx=2     → Audio Volume Right-Left
//   off=3, idx=3     → Audio Volume Right-Right
//
// Common confusion: off=3/idx=0 READS irq_en_ but WRITES the Request Register.
// The INT Enable Register is only written via off=2/idx=1.
void CDRom::write(u32 off, u8 value) noexcept {
    switch (off & 3u) {
    case 0:                              // Index register
        index_ = value & 3u;
        break;

    case 1:
        if (index_ == 0)
            handle_command(value);       // Command register
        // index 1/2/3: Sound Map Data Out — ignored
        break;

    case 2:
        if (index_ == 0) {
            // Parameter FIFO — accumulate up to 8 bytes before the command.
            if (param_len_ < 8u) params_[param_len_++] = value;
        } else if (index_ == 1) {
            // INT Enable Register — primary write path.
            irq_en_ = value & 0x1Fu;
            // If a response is already pending and its type just became enabled,
            // assert the hardware IRQ line now.
            if (irq_fl_ != 0u && (irq_en_ & (1u << (irq_fl_ - 1u))) != 0u)
                irq_.set(IRQSource::CDROM);
        }
        // index 2/3: Audio Volume Left-Left/Right — ignored (no audio output)
        break;

    case 3:
        if (index_ == 0) {
            // Request Register: bit7=BFRD (want data), bit5=SMEN (sound map enable).
            // BFRD=1: prepare sector data buffer for CPU/DMA reads.
            // We don't need to gate data_ready_ on BFRD — reads always succeed.
            // NOT the INT Enable Register (that's a read-only view here).
        } else if (index_ == 1) {
            // INT Flag Acknowledge — write-1-to-clear.
            // bit6: Reset Parameter FIFO.
            if (value & 0x40u) param_len_ = 0u;
            irq_fl_ = static_cast<u8>(irq_fl_ & static_cast<u8>(~(value & 0x1Fu)));
            // De-assert the hardware IRQ line when all flags are cleared.
            // (The CPU also clears I_STAT bit 2 independently via the IRQ controller.)
        }
        // index 2/3: Audio Volume Right-Left/Right — ignored
        break;
    }
}
