#pragma once

#include <array>
#include <cstdio>
#include <vector>
#include "common/Types.hpp"
#include "irq/IRQ.hpp"

// ── CD-ROM Controller ─────────────────────────────────────────────────────────
// Registers live at 0x1F80_1800 – 0x1F80_1803 (byte-wide).
//
//   0x1F801800  (R) Status register   (W) Index register [1:0]
//   0x1F801801  (R) Response FIFO     (W, idx 0) Command register
//   0x1F801802  (R) Data FIFO         (W, idx 0) Parameter FIFO
//   0x1F801803  (R, idx 0) IRQ Enable (W, idx 0) IRQ Enable
//               (R, idx 1) IRQ Flags  (W, idx 1) IRQ Flags (write-1-to-clear)
//
// Status byte (0x1F801800 read)
//   bit 2  ADPBUSY   XA-ADPCM busy
//   bit 3  PRMEMPT   Parameter FIFO empty     (1 = empty)
//   bit 4  PRMWRDY   Parameter FIFO not full  (1 = writable)
//   bit 5  RSLRRDY   Response FIFO not empty  (1 = data ready)
//   bit 6  DRQSTS    Data FIFO not empty
//   bit 7  BUSYSTS   Busy
//
// CD stat byte (returned in response FIFO)
//   bit 0  ERROR       an error occurred
//   bit 1  MOTOR_ON    spindle motor is on
//   bit 2  SEEKERROR   seek failed
//   bit 3  IDERROR     cannot detect disc (no disc)
//   bit 4  SHELL_OPEN  lid is open
//   bit 5  READING     reading data
//   bit 6  SEEKING     seeking
//   bit 7  PLAYING     playing audio
//
// Interrupt types in irq_fl_:
//   INT1  Data ready (sector in buffer)
//   INT2  Second completion response (Pause, Stop, Seek, …)
//   INT3  Acknowledge of first response (most commands)
//   INT4  Data end
//   INT5  Error
// ─────────────────────────────────────────────────────────────────────────────
class CDRom {
public:
    explicit CDRom(IRQ& irq) noexcept : irq_(irq) {}
    ~CDRom() noexcept { for (auto* f : disc_files_) std::fclose(f); }

    // ── Bus interface: byte-wide reads/writes at offsets 0–3 ─────────────────
    [[nodiscard]] u8 read (u32 off) const noexcept;   // const: resp_pos_ mutable
    void             write(u32 off, u8  value) noexcept;

    // ── Disc image ────────────────────────────────────────────────────────────
    // Load a .bin (2352 B/sector raw) or .iso (2048 B/sector) disc image.
    // Returns true on success; without a disc the controller reports lid-open.
    bool load_disc(const char* path) noexcept;

    // ── DMA ch3 interface ─────────────────────────────────────────────────────
    // Called by DMA channel 3 to drain the sector data buffer word by word.
    [[nodiscard]] u32 read_data_word() noexcept;

    // ── Timing tick ───────────────────────────────────────────────────────────
    // Called from Bus::tick() to advance pending CDROM response delays.
    void tick(u32 cycles) noexcept;

private:
    IRQ& irq_;

    u8 index_  = 0;       // current register bank [1:0]
    u8 irq_en_ = 0x1Fu;   // IRQ enable mask [4:0] — BIOS leaves all 5 INT types enabled
    u8 irq_fl_ = 0;       // pending INT type [2:0]  (0 = none)

    // ── CD stat / mode ────────────────────────────────────────────────────────
    u8 cd_stat_ = 0x02u;  // default: motor on, disc present
    u8 mode_    = 0x20u;  // Setmode byte (0x20 = double-speed)

    // ── XA-ADPCM filter (Setfilter 0x0D) ─────────────────────────────────────
    u8 xa_file_    = 0u;
    u8 xa_channel_ = 0u;

    // ── First response FIFO (up to 8 bytes) ───────────────────────────────────
    std::array<u8, 8> resp_{};
    u8            resp_len_     = 0;
    mutable u8    resp_pos_     = 0;   // mutable so read() can be const

    // ── Scheduled responses ───────────────────────────────────────────────────
    // Instead of firing immediately, responses go through a delay countdown.
    // s1_: first response (INT3/INT5); s2_: second response (INT2/INT1).
    // s2_ starts counting down after s1_ fires.
    struct Sched {
        bool active = false;
        u32  delay  = 0;
        u8   type   = 0;
        u8   len    = 0;
        std::array<u8, 8> data{};
    };
    Sched s1_{};   // first pending response
    Sched s2_{};   // second pending response (delay starts after s1 fires)

    // ── Continuous sector stream (ReadN/ReadS) ────────────────────────────────
    bool reading_   = false;   // true while drive is delivering sectors
    u32  rd_delay_  = 0;       // cycles until next INT1 fires
    u32  rd_period_ = 0;       // per-sector cycle count (speed-dependent)

    // ── Parameter FIFO (written before each command) ───────────────────────────
    std::array<u8, 8> params_{};
    u8 param_len_ = 0;

    // ── CUE/BIN multi-track disc ───────────────────────────────────────────────
    struct CdTrack {
        int        number    = 0;
        bool       audio     = false;
        u32        sect_size = 2352u;  // raw bytes per sector on disc
        u32        user_skip = 24u;    // bytes before 2048-byte user data block
        int32_t    disc_lba  = 0;      // disc LBA of INDEX 01 (0 = first user sector)
        long       file_off  = 0;      // byte offset in fh to INDEX 01 sector of this track
        std::FILE* fh        = nullptr; // non-owning; owned by disc_files_
    };
    std::vector<CdTrack>    disc_tracks_;         // sorted by disc_lba ascending
    std::vector<std::FILE*> disc_files_;          // all open file handles (owned)
    int32_t                 lead_out_lba_ = 0;    // disc LBA of lead-out (for GetTD 0xAA)

    int32_t    seek_lba_ = 0;        // target LBA saved by Setloc (signed: negative = lead-in)

    // ── Head position tracking (for GetlocL / GetlocP) ─────────────────────────
    int32_t loc_lba_   = 0;      // signed LBA of last-read/seeked sector (GetlocL)
    bool    loc_valid_ = false;  // loc_lba_ is valid (a seek/read has occurred)
    bool    pos_valid_ = true;   // Q-subchannel position valid (false only after OOB seek)
    int32_t pos_lba_   = -7;     // current head position: ≥0 = track data, <0 = pregap

    // ── Sector data buffer (for DMA ch3 readback) ─────────────────────────────
    static constexpr u32 SECTOR_SIZE = 2048u;
    std::array<u8, SECTOR_SIZE> data_buf_{};
    u32  data_pos_   = 0;          // byte cursor (advanced by read_data_word)
    bool data_ready_ = false;      // sector filled and INT1 pending/fired

    // ── Helpers ───────────────────────────────────────────────────────────────
    [[nodiscard]] u8 status() const noexcept;

    // Parse a .cue file and populate disc_tracks_ / disc_files_ / lead_out_lba_.
    bool load_cue(const char* path) noexcept;

    // Return the track that contains disc LBA lba (last track with disc_lba <= lba).
    [[nodiscard]] const CdTrack* find_track(int32_t lba) const noexcept;

    // Fire a response immediately: store in resp_[] and assert CDRom IRQ.
    void push_response_n(u8 int_type, const u8* data, u32 len) noexcept;

    // Schedule first response with d1 cycle delay.
    // Optionally schedule a second response with d2 cycle delay (from s1 fire).
    void sched(u8 type1, const u8* d1, u32 l1, u32 delay1) noexcept;
    void sched1(u8 type1, u8 byte1, u32 delay1) noexcept {
        sched(type1, &byte1, 1u, delay1);
    }
    void sched2(u8 type2, const u8* d2, u32 l2, u32 delay2) noexcept;

    // Read 2048 bytes at seek_lba_ from the disc image into data_buf_.
    // Advances seek_lba_ by 1 on success.
    bool read_sector() noexcept;

    void handle_command(u8 cmd) noexcept;

    // Decode one BCD byte to binary.
    static u32 from_bcd(u8 v) noexcept {
        return static_cast<u32>(v >> 4u) * 10u + static_cast<u32>(v & 0xFu);
    }

    // Convert MSF (three BCD bytes) to a disc LBA (signed).
    // LBA 0 = absolute MSF 00:02:00 (first user-data sector, 150 frames into disc).
    // Negative LBA means the position is in the lead-in / pregap (before track data).
    static int32_t msf_to_lba(u8 m, u8 s, u8 f) noexcept {
        return static_cast<int32_t>(
            (from_bcd(m) * 60u + from_bcd(s)) * 75u + from_bcd(f)) - 150;
    }
};
