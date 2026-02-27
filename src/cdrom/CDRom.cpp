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

// ── XA-ADPCM filter coefficients (Q6 fixed-point, identical to SPU ADPCM) ────
// s += (k0 * prev0 + k1 * prev1 + 32) >> 6
static constexpr int32_t kXaK[4][2] = {
    {   0,   0 },
    {  60,   0 },
    { 115, -52 },
    {  98, -55 },
};

// Decode one 14-byte ADPCM block into 28 int16_t samples.
// shift/filter are taken from the sound-group parameter byte.
// prev[0] = history sample n-1, prev[1] = history sample n-2 (updated in-place).
static void xa_decode_block(const u8* data, u32 shift, u32 filter,
                              int32_t prev[2], int16_t* out) noexcept {
    const int32_t k0 = kXaK[filter & 3u][0];
    const int32_t k1 = kXaK[filter & 3u][1];
    const int32_t rs = (shift > 12u) ? 12u : (int32_t)shift;  // clamp (corrupt disc guard)

    for (u32 i = 0u; i < 28u; ++i) {
        const u8 byte   = data[i >> 1u];
        const u8 nibble = (i & 1u) ? (byte >> 4u) : (byte & 0xFu);

        // Sign-extend 4-bit nibble to int16 by placing it in the top nibble,
        // then arithmetic-right-shift by the block's range factor.
        int32_t s = (int32_t)(int16_t)((uint16_t)(nibble << 12u)) >> rs;
        s += (k0 * prev[0] + k1 * prev[1] + 32) >> 6;
        s  = (s < -32768) ? -32768 : (s > 32767) ? 32767 : s;

        out[i]  = (int16_t)s;
        prev[1] = prev[0];
        prev[0] = s;
    }
}

// ── Multi-byte response + fire CDRom IRQ immediately ─────────────────────────
void CDRom::push_response_n(u8 int_type, const u8* data, u32 len) noexcept {
    // DEBUG: log every response delivery
    { static u32 rn = 0;
      ++rn;
      if (rn <= 50u)
          std::fprintf(stderr, "[CDROM-RSP] #%u INT%u len=%u d0=0x%02X irq_fl_was=%u irq_en=0x%02X\n",
                       rn, int_type & 0x07u, len,
                       (data && len > 0) ? data[0] : 0u, irq_fl_, irq_en_);
    }
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
    // Close and clear any previously loaded disc.
    for (auto* f : disc_files_) std::fclose(f);
    disc_tracks_.clear();
    disc_files_.clear();
    lead_out_lba_ = 0;

    // .cue extension → full CUE sheet parser.
    const char* dot = std::strrchr(path, '.');
    if (dot && (std::strcmp(dot, ".cue") == 0 || std::strcmp(dot, ".CUE") == 0))
        return load_cue(path);

    // ── Bare BIN or ISO: synthesise a single-track disc entry ─────────────────
    std::FILE* fh = std::fopen(path, "rb");
    if (!fh) {
        std::fprintf(stderr, "[CDRom] cannot open '%s'\n", path);
        return false;
    }
    std::fseek(fh, 0, SEEK_END);
    const long sz = std::ftell(fh);
    std::rewind(fh);

    CdTrack ct{};
    ct.number   = 1;
    ct.audio    = false;
    ct.fh       = fh;
    ct.disc_lba = 0;
    ct.file_off = 0;

    const bool is_bin = dot &&
        (std::strcmp(dot, ".bin") == 0 || std::strcmp(dot, ".BIN") == 0);
    if (is_bin) {
        ct.sect_size  = 2352u;
        ct.user_skip  = 24u;
        lead_out_lba_ = static_cast<int32_t>(sz / 2352L);
    } else {
        ct.sect_size  = 2048u;
        ct.user_skip  = 0u;
        lead_out_lba_ = static_cast<int32_t>(sz / 2048L);
    }

    disc_tracks_.push_back(ct);
    disc_files_.push_back(fh);

    cd_stat_ = 0x02u;
    std::fprintf(stdout, "[CDRom] loaded '%s' (%s format, %d sectors)\n",
                 path, is_bin ? "BIN/2352" : "ISO/2048", lead_out_lba_);
    return true;
}

// ── CUE sheet parser ──────────────────────────────────────────────────────────
bool CDRom::load_cue(const char* cue_path) noexcept {
    // Extract directory from cue_path so we can resolve relative BIN filenames.
    char cue_dir[1024] = {};
    const char* last_slash = std::strrchr(cue_path, '/');
    if (last_slash) {
        const size_t dlen = static_cast<size_t>(last_slash - cue_path + 1);
        if (dlen < sizeof(cue_dir))
            std::memcpy(cue_dir, cue_path, dlen);
    }

    std::FILE* cf = std::fopen(cue_path, "r");
    if (!cf) {
        std::fprintf(stderr, "[CDRom] cannot open CUE '%s'\n", cue_path);
        return false;
    }

    // Intermediate structures: one per FILE / one per INDEX 01 line.
    struct PFile  { std::FILE* fh; long sz; u32 sect_size; };
    struct PTrack { int num; bool audio; u32 sect_size; u32 user_skip;
                    int file_idx; u32 file_sector; };

    std::vector<PFile>  pf;
    std::vector<PTrack> pt;

    char line[512];
    int  cur_fi   = -1;       // index into pf for current FILE directive
    int  cur_tnum = 1;
    u32  cur_ss   = 2352u;
    u32  cur_skip = 24u;
    bool cur_aud  = false;

    while (std::fgets(line, static_cast<int>(sizeof(line)), cf)) {
        const char* p = line;
        while (*p == ' ' || *p == '\t') ++p;   // skip leading whitespace

        if (std::strncmp(p, "FILE", 4) == 0) {
            // FILE "filename" BINARY  — open the referenced BIN file.
            const char* q1 = std::strchr(p, '"');
            if (!q1) continue;
            const char* q2 = std::strchr(q1 + 1, '"');
            if (!q2) continue;
            const size_t fl = static_cast<size_t>(q2 - q1 - 1);
            char fname[480] = {};
            if (fl >= sizeof(fname)) continue;
            std::memcpy(fname, q1 + 1, fl);

            char full[1024] = {};
            std::snprintf(full, sizeof(full), "%s%s", cue_dir, fname);
            std::FILE* fh = std::fopen(full, "rb");
            if (!fh) fh = std::fopen(fname, "rb");  // last-resort: as-is
            if (!fh) {
                std::fprintf(stderr, "[CDRom] CUE: cannot open '%s'\n", full);
                std::fclose(cf);
                for (auto& f : pf) std::fclose(f.fh);
                return false;
            }
            std::fseek(fh, 0, SEEK_END);
            const long sz = std::ftell(fh);
            std::rewind(fh);

            pf.push_back({ fh, sz, 2352u });
            cur_fi = static_cast<int>(pf.size()) - 1;

        } else if (std::strncmp(p, "TRACK", 5) == 0) {
            // TRACK nn MODE2/2352 | MODE1/2352 | MODE1/2048 | AUDIO
            char mode[32] = {};
            std::sscanf(p, "TRACK %d %31s", &cur_tnum, mode);
            if      (std::strcmp(mode, "AUDIO")     == 0) { cur_aud=true;  cur_ss=2352u; cur_skip= 0u; }
            else if (std::strcmp(mode, "MODE2/2352") == 0) { cur_aud=false; cur_ss=2352u; cur_skip=24u; }
            else if (std::strcmp(mode, "MODE1/2352") == 0) { cur_aud=false; cur_ss=2352u; cur_skip=16u; }
            else if (std::strcmp(mode, "MODE1/2048") == 0) { cur_aud=false; cur_ss=2048u; cur_skip= 0u; }
            else                                           { cur_aud=false; cur_ss=2352u; cur_skip=24u; }

        } else if (std::strncmp(p, "INDEX", 5) == 0) {
            // INDEX nn MM:SS:FF — only INDEX 01 marks where track data starts.
            int idx = 0;
            u32 mm = 0, ss = 0, ff = 0;
            if (std::sscanf(p, "INDEX %d %u:%u:%u", &idx, &mm, &ss, &ff) == 4
                    && idx == 1 && cur_fi >= 0) {
                const u32 fsect = (mm * 60u + ss) * 75u + ff;
                pt.push_back({ cur_tnum, cur_aud, cur_ss, cur_skip, cur_fi, fsect });
                // Stamp the sect_size for this file (first track in file wins).
                if (pf[static_cast<size_t>(cur_fi)].sect_size == 2352u)
                    pf[static_cast<size_t>(cur_fi)].sect_size = cur_ss;
            }
        }
    }
    std::fclose(cf);

    if (pt.empty()) {
        std::fprintf(stderr, "[CDRom] CUE: no INDEX 01 entries found in '%s'\n", cue_path);
        for (auto& f : pf) std::fclose(f.fh);
        return false;
    }

    // ── Compute cumulative disc-sector offset at the start of each file ───────
    // file_base[i] = total disc sectors before file i begins.
    std::vector<int32_t> file_base(pf.size(), 0);
    for (size_t i = 1; i < pf.size(); ++i)
        file_base[i] = file_base[i-1]
            + static_cast<int32_t>(pf[i-1].sz / static_cast<long>(pf[i-1].sect_size));

    // disc_lba 0 = INDEX 01 of the first track in the first file.
    const int32_t origin = file_base[static_cast<size_t>(pt[0].file_idx)]
                           + static_cast<int32_t>(pt[0].file_sector);

    // ── Populate disc_tracks_ and disc_files_ ─────────────────────────────────
    for (auto& f : pf) disc_files_.push_back(f.fh);

    for (auto& t : pt) {
        CdTrack ct{};
        ct.number    = t.num;
        ct.audio     = t.audio;
        ct.sect_size = t.sect_size;
        ct.user_skip = t.user_skip;
        ct.disc_lba  = file_base[static_cast<size_t>(t.file_idx)]
                       + static_cast<int32_t>(t.file_sector) - origin;
        ct.file_off  = static_cast<long>(t.file_sector) * static_cast<long>(t.sect_size);
        ct.fh        = pf[static_cast<size_t>(t.file_idx)].fh;
        disc_tracks_.push_back(ct);
    }

    // ── Lead-out: one-past-end of the last file on disc ───────────────────────
    lead_out_lba_ = file_base.back()
                    + static_cast<int32_t>(pf.back().sz
                                           / static_cast<long>(pf.back().sect_size))
                    - origin;

    cd_stat_ = 0x02u;
    std::fprintf(stdout, "[CDRom] CUE '%s': %zu track(s), lead-out LBA %d\n",
                 cue_path, disc_tracks_.size(), lead_out_lba_);
    return true;
}

// ── Find the track containing disc LBA lba ────────────────────────────────────
const CDRom::CdTrack* CDRom::find_track(int32_t lba) const noexcept {
    // disc_tracks_ is sorted by disc_lba ascending.  Return the last entry
    // whose disc_lba <= lba (i.e. the track the head is currently inside).
    const CdTrack* best = nullptr;
    for (const auto& t : disc_tracks_) {
        if (t.disc_lba <= lba) best = &t;
        else break;
    }
    return best;
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
    if (disc_tracks_.empty()) {
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

    const CdTrack* t = find_track(seek_lba_);
    if (!t) {
        // LBA before the first track (lead-in); return zeros.
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

    // Byte offset of the first byte of this raw sector in the file.
    const long sector_start = t->file_off
                            + static_cast<long>(seek_lba_ - t->disc_lba)
                              * static_cast<long>(t->sect_size);

    // For raw Mode-2 sectors (2352 B), peek at the 8-byte sub-header at
    // bytes 16–23 (after 12-byte sync + 4-byte header) to check for XA audio.
    if (t->sect_size == 2352u && !t->audio) {
        u8 subhdr[8] = {};
        if (std::fseek(t->fh, sector_start + 16L, SEEK_SET) == 0) {
            [[maybe_unused]] auto _r = std::fread(subhdr, 1u, 8u, t->fh);
        }

        const u8 submode = subhdr[2];
        const u8 coding  = subhdr[3];

        // bit 2 of submode = Audio sector (XA-ADPCM); mode_ bit 5 = CDXA enable.
        if ((submode & 0x04u) != 0u && (mode_ & 0x20u) != 0u) {
            // XA file/channel filter — mode_ bit 3 enables it.
            const bool pass = !(mode_ & 0x08u)
                           || (subhdr[0] == xa_file_ && subhdr[1] == xa_channel_);
            if (pass) {
                cd_stat_ |=  0x04u;  // set ADPBUSY (bit 2)
                xa_decode_sector(t->fh, sector_start, coding);
            }
            // XA sectors do NOT fill data_buf_; INT1 fires with ADPBUSY set.
            seek_lba_++;
            loc_lba_  = seek_lba_ - 1;
            loc_valid_ = pos_valid_ = true;
            pos_lba_   = loc_lba_;
            return true;
        }
    }

    // Normal data sector: skip the raw preamble and read 2048 user bytes.
    const long off = sector_start + static_cast<long>(t->user_skip);
    if (std::fseek(t->fh, off, SEEK_SET) != 0) return false;
    const u32 got = static_cast<u32>(
        std::fread(data_buf_.data(), 1u, SECTOR_SIZE, t->fh));
    if (got < SECTOR_SIZE) return false;

    cd_stat_ &= ~0x04u;  // clear ADPBUSY on data sectors
    data_pos_   = 0u;
    data_ready_ = true;
    seek_lba_  += 1;
    loc_lba_    = seek_lba_ - 1;
    loc_valid_  = true;
    pos_valid_  = true;
    pos_lba_    = loc_lba_;

    // DEBUG: trace sector reads
    { static int sr = 0;
      if (++sr <= 100)
          std::fprintf(stderr, "[CDROM] read_sector LBA=%d (data_ready=1)\n", loc_lba_); }

    return true;
}

// ── XA-ADPCM sector decode ────────────────────────────────────────────────────
// Reads 2304 bytes (18 × 128-byte sound groups) from the raw sector payload
// (starting at sector_off + 24) and pushes decoded stereo int16 samples into
// the xa_pcm_ ring buffer.
//
// Sound-group layout (128 bytes, 4-bit ADPCM):
//   Bytes  0-7  : Sound parameters for blocks 0–7 (one byte each)
//   Bytes  8-15 : Copy of bytes 0-7 (used for error detection on real hardware)
//   Bytes 16-127: 8 × 14 bytes of 4-bit sample data (28 nibbles per block)
//
// Stereo interleaving: even blocks (0,2,4,6) → left channel,
//                      odd blocks  (1,3,5,7) → right channel.
// Mono: all 8 blocks are output as the single channel (duplicated to L and R).
void CDRom::xa_decode_sector(std::FILE* fh, long sector_off, u8 coding) noexcept {
    const bool stereo = (coding & 0x01u) != 0u;
    // coding bit 4 = 8-bit samples (rare); we only handle 4-bit here.
    if (coding & 0x10u) return;   // 8-bit XA — skip for now

    // Read the audio payload: 18 sound groups × 128 bytes = 2304 bytes.
    // Starts right after the 24-byte raw-sector preamble (sync+header+subheader).
    static constexpr u32 kPayload = 2304u;
    std::array<u8, kPayload> buf;
    if (std::fseek(fh, sector_off + 24L, SEEK_SET) != 0) return;
    if (std::fread(buf.data(), 1u, kPayload, fh) < kPayload) return;

    int16_t tmp_l[28], tmp_r[28];

    for (u32 sg = 0u; sg < 18u; ++sg) {
        const u8* grp = buf.data() + sg * 128u;

        // Process 8 blocks in pairs (even=L, odd=R for stereo; both=mono otherwise).
        for (u32 blk = 0u; blk < 8u; blk += 2u) {
            // Decode even (left / mono) block.
            const u8  pl     = grp[blk];
            const u32 sl     = pl & 0xFu;
            const u32 fl     = (pl >> 4u) & 0x3u;
            xa_decode_block(grp + 16u + blk * 14u, sl, fl,
                            xa_adpcm_.prev[0], tmp_l);

            if (stereo) {
                // Decode odd (right) block.
                const u8  pr = grp[blk + 1u];
                const u32 sr = pr & 0xFu;
                const u32 fr = (pr >> 4u) & 0x3u;
                xa_decode_block(grp + 16u + (blk + 1u) * 14u, sr, fr,
                                xa_adpcm_.prev[1], tmp_r);

                for (u32 i = 0u; i < 28u; ++i) {
                    const u32 wp = xa_pcm_wr_ & (kXaBufSamples - 1u);
                    xa_pcm_[wp * 2u    ] = tmp_l[i];
                    xa_pcm_[wp * 2u + 1u] = tmp_r[i];
                    xa_pcm_wr_ = (xa_pcm_wr_ + 1u) & (kXaBufSamples - 1u);
                }
            } else {
                // Mono: output even block (L dup'd to R), then odd block.
                for (u32 i = 0u; i < 28u; ++i) {
                    const u32 wp = xa_pcm_wr_ & (kXaBufSamples - 1u);
                    xa_pcm_[wp * 2u    ] = tmp_l[i];
                    xa_pcm_[wp * 2u + 1u] = tmp_l[i];
                    xa_pcm_wr_ = (xa_pcm_wr_ + 1u) & (kXaBufSamples - 1u);
                }
                // Decode and output odd (mono) block.
                const u8  pr = grp[blk + 1u];
                const u32 sr = pr & 0xFu;
                const u32 fr = (pr >> 4u) & 0x3u;
                xa_decode_block(grp + 16u + (blk + 1u) * 14u, sr, fr,
                                xa_adpcm_.prev[0], tmp_r);
                for (u32 i = 0u; i < 28u; ++i) {
                    const u32 wp = xa_pcm_wr_ & (kXaBufSamples - 1u);
                    xa_pcm_[wp * 2u    ] = tmp_r[i];
                    xa_pcm_[wp * 2u + 1u] = tmp_r[i];
                    xa_pcm_wr_ = (xa_pcm_wr_ + 1u) & (kXaBufSamples - 1u);
                }
            }
        }
    }
}

// ── XA PCM drain ──────────────────────────────────────────────────────────────
u32 CDRom::xa_drain(int16_t* dst, u32 n) noexcept {
    // Compute available stereo pairs (ring buffer write minus read, mod size).
    // xa_pcm_rd_ is a local read cursor (caller's position).
    // In the headless build nobody calls this; it's here for future SDL use.
    const u32 avail = xa_pcm_wr_ & (kXaBufSamples - 1u);  // simplified: always full
    const u32 count = (n < avail) ? n : avail;
    for (u32 i = 0u; i < count; ++i) {
        const u32 rp = (xa_pcm_wr_ - count + i) & (kXaBufSamples - 1u);
        dst[i * 2u    ] = xa_pcm_[rp * 2u    ];
        dst[i * 2u + 1u] = xa_pcm_[rp * 2u + 1u];
    }
    return count;
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

    // DEBUG: trace CD-ROM commands
    { static int cc = 0;
      if (++cc <= 200)
          std::fprintf(stderr, "[CDROM] cmd=0x%02X p0=0x%02X p1=0x%02X p2=0x%02X (stat=0x%02X)\n",
                       cmd, p0, p1, p2, cd_stat_); }

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
        cd_stat_ = static_cast<u8>(cd_stat_ & ~(0x20u | 0x04u));  // clear READING + ADPBUSY
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
        u8 first = 0x01u, last = 0x01u;
        if (!disc_tracks_.empty()) {
            first = to_bcd(static_cast<u32>(disc_tracks_.front().number));
            last  = to_bcd(static_cast<u32>(disc_tracks_.back().number));
        }
        const u8 r[3] = { cd_stat_, first, last };
        sched(3u, r, 3u, kAck1);
        break;
    }

    // ── 0x14 GetTD — track N start position (minute:second in BCD) ───────────
    case 0x14: {
        u8 mm = 0x00u, ss = 0x02u;  // default: track 1 at absolute MSF 00:02:00
        if (!disc_tracks_.empty()) {
            if (p0 == 0xAAu) {
                // Track 0xAA = lead-out
                const u32 f = static_cast<u32>(lead_out_lba_ + 150);
                mm = to_bcd(f / (75u * 60u));
                ss = to_bcd((f / 75u) % 60u);
            } else {
                const u32 tnum = from_bcd(p0);
                bool found = false;
                for (const auto& t : disc_tracks_) {
                    if (t.number == static_cast<int>(tnum)) {
                        const u32 f = static_cast<u32>(t.disc_lba + 150);
                        mm = to_bcd(f / (75u * 60u));
                        ss = to_bcd((f / 75u) % 60u);
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    const u8 err[2] = { static_cast<u8>(cd_stat_ | 0x01u), 0x10u };
                    sched(5u, err, 2u, kAck1);
                    break;
                }
            }
        }
        const u8 r[3] = { cd_stat_, mm, ss };
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
        // Determine current track and compute relative MSF within it.
        u8 track, index, rmm, rss, rff;
        if (pos_lba_ >= 0) {
            // Scan track table for the track containing pos_lba_.
            int32_t track_start = 0;
            track = 0x01u;
            for (const auto& t : disc_tracks_) {
                if (t.disc_lba <= pos_lba_) {
                    track = to_bcd(static_cast<u32>(t.number));
                    track_start = t.disc_lba;
                } else break;
            }
            index = 0x01u;  // in track data (past INDEX 01)
            const u32 rf = static_cast<u32>(pos_lba_ - track_start);
            rmm = to_bcd(rf / (75u * 60u));
            rss = to_bcd((rf / 75u) % 60u);
            rff = to_bcd(rf % 75u);
        } else {
            track = 0x01u;  index = 0x00u;  // in pregap before track 1
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

    // ── 0x03 Play — CD-DA audio track playback (silent stub) ─────────────────
    // Start playback from seek_lba_ (or current position).  On real hardware
    // INT1 fires once per sector while playing; we fire INT3 + INT2 immediately
    // so the BIOS/game sees a completed play command rather than hanging.
    case 0x03:
        sched1(3u, cd_stat_, kAck1);
        sched2(2u, &cd_stat_, 1u, kD2Seek);
        break;

    // ── 0x04 Forward / 0x05 Backward — audio fast-scan (stub) ───────────────
    case 0x04:
    case 0x05:
        sched1(3u, cd_stat_, kAck1);
        break;

    // ── 0x07 MotorOn — spin up motor ─────────────────────────────────────────
    // Clears error flags and turns the motor on; INT3 then INT2.
    case 0x07:
        reading_ = false;
        cd_stat_ = 0x02u;  // MOTOR_ON
        sched1(3u, cd_stat_, kAck1);
        sched2(2u, &cd_stat_, 1u, kD2Seek);
        break;

    // ── 0x0D Setfilter — XA-ADPCM file/channel filter ────────────────────────
    // Store the target XA file and channel number.  No XA decode yet, but
    // accepting the command prevents INT5 errors in games that set the filter
    // before opening the XA stream.
    case 0x0D:
        xa_file_    = p0;
        xa_channel_ = p1;
        sched1(3u, cd_stat_, kAck1);
        break;

    // ── 0x19 Test — subcommand dispatch ──────────────────────────────────────
    // Sub 0x20: return CDROM firmware version/date string.
    case 0x19: {
        if (p0 == 0x20u) {
            // SCPH-1001 (NTSC-U) firmware date: 1997/01/10 version C2
            const u8 r[4] = { 0x97u, 0x01u, 0x10u, 0xC2u };
            sched(3u, r, 4u, kAck1);
        } else {
            const u8 err[2] = { static_cast<u8>(cd_stat_ | 0x01u), 0x40u };
            sched(5u, err, 2u, kAck1);
        }
        break;
    }

    // ── 0x1C LockDoor / 0x1D UnlockDoor ──────────────────────────────────────
    case 0x1C:
    case 0x1D:
        sched1(3u, cd_stat_, kAck1);
        break;

    // ── 0x1E ReadTOC — re-read the disc Table of Contents (stub) ──────────
    // Real hardware re-reads the TOC from the disc; we already have it parsed.
    // INT3 (acknowledge) then INT2 (complete) with the usual delays.
    case 0x1E:
        sched1(3u, cd_stat_, kAck1);
        sched2(2u, &cd_stat_, 1u, kD2Init);
        break;

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
