#include "GPU.hpp"

// ── GP0 command word-count table ──────────────────────────────────────────────
// One entry per possible command byte (bits [31:24] of the first FIFO word).
// 0 = variable-length (poly-line); the terminator is detected at runtime
// when a vertex word has its top nibble == 0x5 (e.g. 0x55555555).
// ─────────────────────────────────────────────────────────────────────────────
namespace {
constexpr u8 kGP0Len[256] = {
    //  0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
        1,   1,   3,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,  // 00
        1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,  // 10
    //  tri  .    .    .  t-tri .    .    .   quad  .    .    .  t-quad .    .    .
        4,   4,   4,   4,   7,   7,   7,   7,   5,   5,   5,   5,   9,   9,   9,   9,  // 20
    // g-tri .    .    . gt-tri .    .    . g-quad .    .    . gt-quad .    .    .
        6,   6,   6,   6,   9,   9,   9,   9,   8,   8,   8,   8,  12,  12,  12,  12,  // 30
    // line  .    .    .    .    .    .    . poly-line (variable)
        3,   3,   3,   3,   3,   3,   3,   3,   0,   0,   0,   0,   0,   0,   0,   0,  // 40
    // g-line .    .    .    .    .    .   . g-polyline (variable)
        4,   4,   4,   4,   4,   4,   4,   4,   0,   0,   0,   0,   0,   0,   0,   0,  // 50
    // var-rect .    .    . tex-rect .    .    . 1x1-rect  .    .  1x1-tex  .    .
        3,   3,   3,   3,   4,   4,   4,   4,   2,   2,   2,   2,   3,   3,   3,   3,  // 60
    // 8x8  .    .    . 8x8-tex .    .    . 16x16 .    .    . 16x16-tex .    .
        2,   2,   2,   2,   3,   3,   3,   3,   2,   2,   2,   2,   3,   3,   3,   3,  // 70
    // VRAM-to-VRAM copy (all 0x80-0x9F)
        4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,  // 80
        4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,  // 90
    // CPU-to-VRAM header (all 0xA0-0xBF); data words follow outside the FIFO
        3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,  // A0
        3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,  // B0
    // VRAM-to-CPU header (all 0xC0-0xDF)
        3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,  // C0
        3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,  // D0
    // Environment commands (E0-FF): all 1 word
        1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,  // E0
        1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,  // F0
};
}  // namespace

// ── Bus interface ─────────────────────────────────────────────────────────────

u32 GPU::read(u32 offset) const noexcept {
    switch (offset) {
    case 0:
        // During a VRAM→CPU transfer, pack the next two pixels into GPUREAD.
        if (vtcr_active_) {
            const u16 lo = vram_read_pixel();
            const u16 hi = vram_read_pixel();
            gpuread_ = static_cast<u32>(lo) | (static_cast<u32>(hi) << 16u);
        }
        return gpuread_;
    case 4: return gpustat_;
    default: return 0;
    }
}

void GPU::write(u32 offset, u32 value) noexcept {
    switch (offset) {
    case 0: gp0(value); break;
    case 4: gp1(value); break;
    default: break;
    }
}

// ── GP0 — FIFO-based command decoder ─────────────────────────────────────────
//
// Phase 1: if a CPU→VRAM transfer is in progress, incoming words are pixel
//          data and bypass the FIFO entirely.
// Phase 2: otherwise, words are pushed into the FIFO.  Once enough words have
//          accumulated (per kGP0Len), gp0_execute() is called.
void GPU::gp0(u32 word) noexcept {
    // Phase 1 — CPU→VRAM data words (two 16bpp pixels per u32).
    if (ctvr_cy_ < ctvr_h_) {
        vram_write_pixel(static_cast<u16>(word));
        vram_write_pixel(static_cast<u16>(word >> 16u));
        return;
    }

    // Phase 2 — FIFO accumulation.
    if (fifo_len_ < FIFO_CAP) {
        fifo_[fifo_len_] = word;
    }
    ++fifo_len_;

    // On the first word, decode the expected command length.
    if (fifo_len_ == 1u) {
        fifo_need_ = kGP0Len[word >> 24u];
        if (fifo_need_ == 1u) {
            // Single-word command — execute immediately.
            gp0_execute();
            fifo_len_ = 0;
        }
        return;
    }

    // Variable-length poly-line: terminate on a vertex whose top nibble = 0x5.
    if (fifo_need_ == 0u) {
        if (fifo_len_ >= 3u && (word >> 28u) == 0x5u) {
            fifo_len_ = 0;
        }
        return;
    }

    // Fixed-length command — execute once all words are present.
    if (fifo_len_ >= fifo_need_) {
        gp0_execute();
        fifo_len_ = 0;
    }
}

// ── GP0 command executor ──────────────────────────────────────────────────────
void GPU::gp0_execute() noexcept {
    const u32 cmd   = fifo_[0] >> 24u;
    const u32 param = fifo_[0] & 0x00FF'FFFFu;

    switch (cmd) {

    // ── Misc ──────────────────────────────────────────────────────────────────
    case 0x00: break;   // NOP
    case 0x01: break;   // Clear texture cache

    case 0x1F:          // Interrupt request (IRQ1)
        gpustat_ |= (1u << 24u);
        break;

    // ── Fill rectangle ────────────────────────────────────────────────────────
    // fifo_[0]: cmd[31:24] + color 24bpp [23:0]
    // fifo_[1]: Y[24:16]   + X[9:0]
    // fifo_[2]: H[24:16]   + W[9:0]  (0 → 0x200 / 0x400)
    case 0x02: {
        const u32 r  = (param >>  0u) & 0xFFu;
        const u32 g  = (param >>  8u) & 0xFFu;
        const u32 b  = (param >> 16u) & 0xFFu;
        const u16 px = static_cast<u16>(
            ((r >> 3u) << 0u) | ((g >> 3u) << 5u) | ((b >> 3u) << 10u));

        const u32 x0 =  (fifo_[1] >>  0u) & 0x3FFu;
        const u32 y0 =  (fifo_[1] >> 16u) & 0x1FFu;
              u32 w  = ((fifo_[2] >>  0u) & 0x3FFu);
              u32 h  = ((fifo_[2] >> 16u) & 0x1FFu);
        if (w == 0u) w = 0x400u;
        if (h == 0u) h = 0x200u;

        for (u32 dy = 0; dy < h; ++dy) {
            const u32 vy = (y0 + dy) & (VRAM_H - 1u);
            for (u32 dx = 0; dx < w; ++dx) {
                const u32 vx = (x0 + dx) & (VRAM_W - 1u);
                vram_[vy * VRAM_W + vx] = px;
            }
        }
        break;
    }

    // ── VRAM-to-VRAM copy ─────────────────────────────────────────────────────
    // fifo_[1]: src Y[24:16] + src X[9:0]
    // fifo_[2]: dst Y[24:16] + dst X[9:0]
    // fifo_[3]: H[24:16]     + W[9:0]  (0 → 0x200 / 0x400)
    case 0x80: {
        const u32 sx =  (fifo_[1] >>  0u) & 0x3FFu;
        const u32 sy =  (fifo_[1] >> 16u) & 0x1FFu;
        const u32 dx =  (fifo_[2] >>  0u) & 0x3FFu;
        const u32 dy =  (fifo_[2] >> 16u) & 0x1FFu;
              u32 w  = ((fifo_[3] >>  0u) & 0x3FFu);
              u32 h  = ((fifo_[3] >> 16u) & 0x1FFu);
        if (w == 0u) w = 0x400u;
        if (h == 0u) h = 0x200u;

        for (u32 row = 0; row < h; ++row) {
            for (u32 col = 0; col < w; ++col) {
                const u32 svx = (sx + col) & (VRAM_W - 1u);
                const u32 svy = (sy + row) & (VRAM_H - 1u);
                const u32 dvx = (dx + col) & (VRAM_W - 1u);
                const u32 dvy = (dy + row) & (VRAM_H - 1u);
                vram_[dvy * VRAM_W + dvx] = vram_[svy * VRAM_W + svx];
            }
        }
        break;
    }

    // ── CPU→VRAM transfer — begin data phase ──────────────────────────────────
    // fifo_[1]: dst Y[24:16] + dst X[9:0]
    // fifo_[2]: H[24:16]     + W[9:0]  (0 → 0x200 / 0x400)
    // Subsequent GP0 words are pixel data (handled in Phase 1 of gp0()).
    case 0xA0: {
        ctvr_x_  =  (fifo_[1] >>  0u) & 0x3FFu;
        ctvr_y_  =  (fifo_[1] >> 16u) & 0x1FFu;
        ctvr_w_  = ((fifo_[2] >>  0u) & 0x3FFu);
        ctvr_h_  = ((fifo_[2] >> 16u) & 0x1FFu);
        ctvr_cx_ = 0;
        ctvr_cy_ = 0;
        if (ctvr_w_ == 0u) ctvr_w_ = 0x400u;
        if (ctvr_h_ == 0u) ctvr_h_ = 0x200u;
        break;
    }

    // ── VRAM→CPU transfer — set up readback cursor ────────────────────────────
    // fifo_[1]: src Y[24:16] + src X[9:0]
    // fifo_[2]: H[24:16]     + W[9:0]  (0 → 0x200 / 0x400)
    // Subsequent reads of GPUREAD (offset 0) return packed pixel pairs.
    case 0xC0: {
        vtcr_x_  =  (fifo_[1] >>  0u) & 0x3FFu;
        vtcr_y_  =  (fifo_[1] >> 16u) & 0x1FFu;
        vtcr_w_  = ((fifo_[2] >>  0u) & 0x3FFu);
        vtcr_h_  = ((fifo_[2] >> 16u) & 0x1FFu);
        vtcr_cx_ = 0;
        vtcr_cy_ = 0;
        if (vtcr_w_ == 0u) vtcr_w_ = 0x400u;
        if (vtcr_h_ == 0u) vtcr_h_ = 0x200u;
        vtcr_active_ = true;
        break;
    }

    // ── Environment commands (single-word; update GPUSTAT / stored regs) ──────
    case 0xE1: {
        // param[10:0] → GPUSTAT[10:0]
        // param[11]   → GPUSTAT[15] (texture disable) — only when allowed by GP1(09)
        constexpr u32 kClear = 0x7FFu | (1u << 15u);
        u32 new_stat = (gpustat_ & ~kClear) | (param & 0x7FFu) | kReadyMask;
        if (allow_texture_disable_) {
            new_stat = (new_stat & ~(1u << 15u)) | (((param >> 11u) & 1u) << 15u);
        }
        gpustat_ = new_stat;
        break;
    }
    case 0xE2: tex_window_   = param; break;
    case 0xE3: draw_area_tl_ = param; break;
    case 0xE4: draw_area_br_ = param; break;
    case 0xE5: draw_offset_  = param; break;

    case 0xE6:
        // param[0] → GPUSTAT[11] (set mask on draw)
        // param[1] → GPUSTAT[12] (check mask on draw)
        gpustat_ = (gpustat_ & ~(3u << 11u))
                 | ((param & 3u) << 11u)
                 | kReadyMask;
        break;

    default:
        // Drawing commands: dispatch by top 3 bits of the command byte.
        //   0x20–0x3F  (cmd >> 5 == 1)  polygons (triangles and quads)
        //   0x40–0x5F  (cmd >> 5 == 2)  lines and poly-lines
        //   0x60–0x7F  (cmd >> 5 == 3)  sprites / rectangles
        dispatch_draw(cmd);
        break;
    }
}

// ── VRAM pixel helpers ────────────────────────────────────────────────────────

void GPU::vram_write_pixel(u16 px) noexcept {
    if (ctvr_cy_ >= ctvr_h_) return;
    const u32 vx = (ctvr_x_ + ctvr_cx_) & (VRAM_W - 1u);
    const u32 vy = (ctvr_y_ + ctvr_cy_) & (VRAM_H - 1u);
    const u32 idx = vy * VRAM_W + vx;
    const bool check_mask = (gpustat_ >> 12u) & 1u;
    if (!(check_mask && (vram_[idx] & 0x8000u))) {
        const u16 mask_bit = ((gpustat_ >> 11u) & 1u) ? 0x8000u : 0u;
        vram_[idx] = px | mask_bit;
    }
    ++ctvr_cx_;
    if (ctvr_cx_ >= ctvr_w_) {
        ctvr_cx_ = 0;
        ++ctvr_cy_;
    }
}

u16 GPU::vram_read_pixel() const noexcept {
    if (!vtcr_active_) return 0u;
    const u32 vx = (vtcr_x_ + vtcr_cx_) & (VRAM_W - 1u);
    const u32 vy = (vtcr_y_ + vtcr_cy_) & (VRAM_H - 1u);
    const u16 px = vram_[vy * VRAM_W + vx];
    ++vtcr_cx_;
    if (vtcr_cx_ >= vtcr_w_) {
        vtcr_cx_ = 0;
        ++vtcr_cy_;
        if (vtcr_cy_ >= vtcr_h_) {
            vtcr_active_ = false;
        }
    }
    return px;
}

// ── Texture fetch ─────────────────────────────────────────────────────────────
//
// Applies texture window masking (GP0 E2), then indexes into VRAM using the
// texture page base from GPUSTAT[4:0] and the current color depth (GPUSTAT[8:7]).
//
// Depth 0 — 4-bit CLUT: each VRAM 16-bit word holds 4 palette indices.
// Depth 1 — 8-bit CLUT: each VRAM 16-bit word holds 2 palette indices.
// Depth 2+ — 15-bit raw: each VRAM 16-bit word is one texel (RGB555).
u16 GPU::fetch_texel(s32 u_raw, s32 v_raw) const noexcept {
    // Texture window (E2): mask selects which coordinate bits are replaced
    // by the offset bits.  Both fields are in units of 8 pixels.
    const u32 win_mask_x = ((tex_window_ >>  0u) & 0x1Fu) * 8u;
    const u32 win_mask_y = ((tex_window_ >>  5u) & 0x1Fu) * 8u;
    const u32 win_off_x  = ((tex_window_ >> 10u) & 0x1Fu) * 8u;
    const u32 win_off_y  = ((tex_window_ >> 15u) & 0x1Fu) * 8u;

    // Wrap u/v to 8-bit then apply window.
    const u32 u = static_cast<u32>(u_raw) & 0xFFu;
    const u32 v = static_cast<u32>(v_raw) & 0xFFu;
    const u32 ue = (u & ~win_mask_x) | (win_off_x & win_mask_x);
    const u32 ve = (v & ~win_mask_y) | (win_off_y & win_mask_y);

    // Texture page origin in VRAM.
    const u32 tx_base = (gpustat_ & 0xFu) * 64u;        // X: [3:0] × 64 pixels
    const u32 ty_base = ((gpustat_ >> 4u) & 1u) * 256u; // Y: bit4 × 256 pixels
    const u32 depth   = (gpustat_ >> 7u) & 3u;

    switch (depth) {
    case 0: {  // 4-bit CLUT — 4 indices per 16-bit word
        const u32 vx  = (tx_base + ue / 4u) & (VRAM_W - 1u);
        const u32 vy  = (ty_base + ve)       & (VRAM_H - 1u);
        const u16 raw = vram_[vy * VRAM_W + vx];
        const u32 idx = (raw >> ((ue & 3u) * 4u)) & 0xFu;
        return vram_[(clut_y_ & (VRAM_H - 1u)) * VRAM_W
                   + ((clut_x_ + idx) & (VRAM_W - 1u))];
    }
    case 1: {  // 8-bit CLUT — 2 indices per 16-bit word
        const u32 vx  = (tx_base + ue / 2u) & (VRAM_W - 1u);
        const u32 vy  = (ty_base + ve)       & (VRAM_H - 1u);
        const u16 raw = vram_[vy * VRAM_W + vx];
        const u32 idx = (raw >> ((ue & 1u) * 8u)) & 0xFFu;
        return vram_[(clut_y_ & (VRAM_H - 1u)) * VRAM_W
                   + ((clut_x_ + idx) & (VRAM_W - 1u))];
    }
    default: {  // 15-bit raw (depth 2 or 3)
        const u32 vx = (tx_base + ue) & (VRAM_W - 1u);
        const u32 vy = (ty_base + ve) & (VRAM_H - 1u);
        return vram_[vy * VRAM_W + vx];
    }
    }
}

// ── Coordinate helpers ────────────────────────────────────────────────────────

// Sign-extend 11-bit X coordinate from bits [10:0] of a vertex word.
s32 GPU::sx11(u32 word) noexcept {
    return static_cast<s32>((word & 0x7FFu) << 21u) >> 21;
}

// Sign-extend 11-bit Y coordinate from bits [26:16] of a vertex word.
s32 GPU::sy11(u32 word) noexcept {
    return static_cast<s32>(((word >> 16u) & 0x7FFu) << 21u) >> 21;
}

s32 GPU::da_x1() const noexcept { return static_cast<s32>( draw_area_tl_        & 0x3FFu); }
s32 GPU::da_y1() const noexcept { return static_cast<s32>((draw_area_tl_ >> 10u) & 0x1FFu); }
s32 GPU::da_x2() const noexcept { return static_cast<s32>( draw_area_br_        & 0x3FFu); }
s32 GPU::da_y2() const noexcept { return static_cast<s32>((draw_area_br_ >> 10u) & 0x1FFu); }

s32 GPU::off_x() const noexcept {
    return static_cast<s32>((draw_offset_        & 0x7FFu) << 21u) >> 21;
}
s32 GPU::off_y() const noexcept {
    return static_cast<s32>(((draw_offset_ >> 11u) & 0x7FFu) << 21u) >> 21;
}

// ── Rasterizer primitives ─────────────────────────────────────────────────────

// Apply one of the four PSX semi-transparency blend modes.
// mode is GPUSTAT[6:5]:  0: (B+F)/2  1: B+F  2: B-F  3: B+F/4
// back is the existing VRAM pixel (RGB555); fr/fg/fb are the front 5-bit channels.
// Returns the blended RGB555 (bit 15 = 0).
static u16 blend_rgb(u32 mode, u16 back, u32 fr, u32 fg, u32 fb) noexcept {
    const u32 br = (back >>  0u) & 0x1Fu;
    const u32 bg = (back >>  5u) & 0x1Fu;
    const u32 bb = (back >> 10u) & 0x1Fu;
    u32 rr, rg, rb;
    switch (mode) {
    case 0:  // (B + F) / 2
        rr = (br + fr) >> 1u;  rg = (bg + fg) >> 1u;  rb = (bb + fb) >> 1u;
        break;
    case 1:  // B + F, clamped to 31
        rr = std::min(31u, br + fr);
        rg = std::min(31u, bg + fg);
        rb = std::min(31u, bb + fb);
        break;
    case 2:  // B - F, clamped to 0
        rr = (br > fr) ? (br - fr) : 0u;
        rg = (bg > fg) ? (bg - fg) : 0u;
        rb = (bb > fb) ? (bb - fb) : 0u;
        break;
    default: // B + F/4, clamped to 31
        rr = std::min(31u, br + (fr >> 2u));
        rg = std::min(31u, bg + (fg >> 2u));
        rb = std::min(31u, bb + (fb >> 2u));
        break;
    }
    return static_cast<u16>(rr | (rg << 5u) | (rb << 10u));
}

// Write one RGB888 pixel (converted to RGB555) to VRAM, clipped to draw area.
// If prim_semi_ is set, applies the semi-transparency blend mode from GPUSTAT[6:5].
void GPU::put_pixel(s32 x, s32 y, u8 r, u8 g, u8 b) noexcept {
    if (x < da_x1() || x > da_x2() || y < da_y1() || y > da_y2()) return;
    if (x < 0 || x >= static_cast<s32>(VRAM_W)) return;
    if (y < 0 || y >= static_cast<s32>(VRAM_H)) return;
    const u32  idx        = static_cast<u32>(y) * VRAM_W + static_cast<u32>(x);
    const bool check_mask = (gpustat_ >> 12u) & 1u;
    if (check_mask && (vram_[idx] & 0x8000u)) return;
    const u16 mask_bit = ((gpustat_ >> 11u) & 1u) ? 0x8000u : 0u;
    const u32 fr = static_cast<u32>(r) >> 3u;
    const u32 fg = static_cast<u32>(g) >> 3u;
    const u32 fb = static_cast<u32>(b) >> 3u;
    u16 result;
    if (prim_semi_) {
        result = blend_rgb((gpustat_ >> 5u) & 3u, vram_[idx], fr, fg, fb);
    } else {
        result = static_cast<u16>(fr | (fg << 5u) | (fb << 10u));
    }
    vram_[idx] = result | mask_bit;
}

// Write one pre-built RGB555 texel to VRAM, clipped to draw area.
// Bit 15 of the texel is the per-pixel semi-transparency flag: if prim_semi_ is
// also set, this pixel is blended; otherwise it is drawn opaque.
// The stored VRAM bit 15 is always taken from GPUSTAT[11] (mask-on-draw).
void GPU::put_pixel(s32 x, s32 y, u16 rgb555) noexcept {
    if (x < da_x1() || x > da_x2() || y < da_y1() || y > da_y2()) return;
    if (x < 0 || x >= static_cast<s32>(VRAM_W)) return;
    if (y < 0 || y >= static_cast<s32>(VRAM_H)) return;
    const u32  idx        = static_cast<u32>(y) * VRAM_W + static_cast<u32>(x);
    const bool check_mask = (gpustat_ >> 12u) & 1u;
    if (check_mask && (vram_[idx] & 0x8000u)) return;
    const u16 mask_bit = ((gpustat_ >> 11u) & 1u) ? 0x8000u : 0u;
    u16 result;
    if (prim_semi_ && (rgb555 & 0x8000u)) {
        // Per-pixel semi-transparency: texel bit 15 set → apply blend.
        const u32 fr = (rgb555 >>  0u) & 0x1Fu;
        const u32 fg = (rgb555 >>  5u) & 0x1Fu;
        const u32 fb = (rgb555 >> 10u) & 0x1Fu;
        result = blend_rgb((gpustat_ >> 5u) & 3u, vram_[idx], fr, fg, fb);
    } else {
        result = rgb555 & 0x7FFFu;
    }
    vram_[idx] = result | mask_bit;
}

// Edge function: 2× signed area of the triangle (a, b, p).
static s32 edge_fn(s32 ax, s32 ay, s32 bx, s32 by, s32 px, s32 py) noexcept {
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

// Clamp a 32-bit value to [0, 255].
static u8 clamp_u8(s32 v) noexcept {
    return static_cast<u8>(v < 0 ? 0 : v > 255 ? 255 : v);
}

// Edge-function (half-space) rasterizer with Gouraud color / texture interpolation.
// Draw offset must already be incorporated into vertex coordinates before calling.
// textured=true: fetch texels via fetch_texel() and modulate with vertex color.
void GPU::raster_tri(Vertex v0, Vertex v1, Vertex v2, bool textured) noexcept {
    // 2× signed area — used both for orientation test and barycentric division.
    const s32 area2 = edge_fn(v0.x, v0.y, v1.x, v1.y, v2.x, v2.y);
    if (area2 == 0) return;  // degenerate (zero-area)

    // Bounding box clipped to the draw area.
    const s32 x_lo = std::max(std::min({v0.x, v1.x, v2.x}), da_x1());
    const s32 x_hi = std::min(std::max({v0.x, v1.x, v2.x}), da_x2());
    const s32 y_lo = std::max(std::min({v0.y, v1.y, v2.y}), da_y1());
    const s32 y_hi = std::min(std::max({v0.y, v1.y, v2.y}), da_y2());

    if (x_lo > x_hi || y_lo > y_hi) return;

    for (s32 py = y_lo; py <= y_hi; ++py) {
        for (s32 px = x_lo; px <= x_hi; ++px) {
            const s32 w0 = edge_fn(v1.x, v1.y, v2.x, v2.y, px, py);
            const s32 w1 = edge_fn(v2.x, v2.y, v0.x, v0.y, px, py);
            const s32 w2 = edge_fn(v0.x, v0.y, v1.x, v1.y, px, py);

            // All weights must have the same sign as area2 for the pixel to be inside.
            if (area2 > 0 ? (w0 < 0 || w1 < 0 || w2 < 0)
                          : (w0 > 0 || w1 > 0 || w2 > 0)) continue;

            if (textured) {
                // Barycentric UV interpolation.
                const s32 ui = (v0.u * w0 + v1.u * w1 + v2.u * w2) / area2;
                const s32 vi = (v0.v * w0 + v1.v * w1 + v2.v * w2) / area2;
                const u16 texel = fetch_texel(ui, vi);
                if (texel == 0u) continue;  // color-key transparent

                // Modulate: result = min(31, (texel_5bit × vertex_8bit) >> 7).
                // vertex_color 0x80 = 128 is the neutral "no tint" value.
                const s32 vr = clamp_u8((v0.r * w0 + v1.r * w1 + v2.r * w2) / area2);
                const s32 vg = clamp_u8((v0.g * w0 + v1.g * w1 + v2.g * w2) / area2);
                const s32 vb = clamp_u8((v0.b * w0 + v1.b * w1 + v2.b * w2) / area2);
                const s32 tr = (texel >>  0u) & 0x1Fu;
                const s32 tg = (texel >>  5u) & 0x1Fu;
                const s32 tb = (texel >> 10u) & 0x1Fu;
                const s32 fr = std::min(31, (tr * vr) >> 7);
                const s32 fg = std::min(31, (tg * vg) >> 7);
                const s32 fb = std::min(31, (tb * vb) >> 7);
                // Preserve texel bit 15 (per-pixel semi-transparency flag).
                put_pixel(px, py,
                          static_cast<u16>(fr | (fg << 5) | (fb << 10)
                                           | (texel & 0x8000u)));
            } else {
                // Barycentric color interpolation (flat or Gouraud).
                const u8 r = clamp_u8((v0.r * w0 + v1.r * w1 + v2.r * w2) / area2);
                const u8 g = clamp_u8((v0.g * w0 + v1.g * w1 + v2.g * w2) / area2);
                const u8 b = clamp_u8((v0.b * w0 + v1.b * w1 + v2.b * w2) / area2);
                put_pixel(px, py, r, g, b);
            }
        }
    }
}

// Axis-aligned filled rectangle, clipped to draw area.
// u_org/v_org: 8-bit UV at top-left before clipping; u_org == -1 → untextured.
void GPU::raster_rect(s32 x, s32 y, s32 w, s32 h,
                      u8 r, u8 g, u8 b,
                      s32 u_org, s32 v_org) noexcept {
    const bool textured = u_org >= 0;
    const s32 x0 = std::max(x,         da_x1());
    const s32 y0 = std::max(y,         da_y1());
    const s32 x1 = std::min(x + w - 1, da_x2());
    const s32 y1 = std::min(y + h - 1, da_y2());
    for (s32 ry = y0; ry <= y1; ++ry) {
        for (s32 rx = x0; rx <= x1; ++rx) {
            if (textured) {
                const s32 ui = (u_org + (rx - x)) & 0xFF;
                const s32 vi = (v_org + (ry - y)) & 0xFF;
                const u16 texel = fetch_texel(ui, vi);
                if (texel == 0u) continue;  // color-key transparent
                // Modulate texel with vertex color.
                const s32 tr = (texel >>  0u) & 0x1Fu;
                const s32 tg = (texel >>  5u) & 0x1Fu;
                const s32 tb = (texel >> 10u) & 0x1Fu;
                const s32 fr = std::min(31, (tr * static_cast<s32>(r)) >> 7);
                const s32 fg = std::min(31, (tg * static_cast<s32>(g)) >> 7);
                const s32 fb = std::min(31, (tb * static_cast<s32>(b)) >> 7);
                // Preserve texel bit 15 (per-pixel semi-transparency flag).
                put_pixel(rx, ry, static_cast<u16>(fr | (fg << 5) | (fb << 10)
                                                   | (texel & 0x8000u)));
            } else {
                put_pixel(rx, ry, r, g, b);
            }
        }
    }
}

// Bresenham line with Gouraud color interpolation.
void GPU::raster_line(Vertex v0, Vertex v1) noexcept {
    s32 dx = std::abs(v1.x - v0.x);
    s32 dy = std::abs(v1.y - v0.y);
    const s32 sx = (v1.x > v0.x) ? 1 : -1;
    const s32 sy = (v1.y > v0.y) ? 1 : -1;
    s32 err = dx - dy;
    const s32 steps = std::max(dx, dy);

    s32 cx = v0.x, cy = v0.y;
    for (s32 i = 0; i <= steps; ++i) {
        // Linearly interpolate color.
        const u8 r = (steps > 0) ? clamp_u8(v0.r + (v1.r - v0.r) * i / steps) : static_cast<u8>(v0.r);
        const u8 g = (steps > 0) ? clamp_u8(v0.g + (v1.g - v0.g) * i / steps) : static_cast<u8>(v0.g);
        const u8 b = (steps > 0) ? clamp_u8(v0.b + (v1.b - v0.b) * i / steps) : static_cast<u8>(v0.b);
        put_pixel(cx, cy, r, g, b);
        const s32 e2 = 2 * err;
        if (e2 > -dy) { err -= dy; cx += sx; }
        if (e2 <  dx) { err += dx; cy += sy; }
    }
}

// ── Drawing command dispatcher ────────────────────────────────────────────────
//
// Called from gp0_execute() for commands in 0x20–0x7F.
//
// Polygon command byte (0x20–0x3F) bit structure:
//   bit 4: 0=flat color, 1=Gouraud shading
//   bit 3: 0=triangle,   1=quad (4 vertices)
//   bit 2: 0=untextured, 1=textured
//   bit 1: 0=opaque,     1=semi-transparent  (blend not yet implemented)
//
// Flat textured triangle FIFO layout (7 words, stride=2):
//   [0]=cmd+color  [1]=v0_xy  [2]=clut|uv0  [3]=v1_xy  [4]=tpage|uv1
//   [5]=v2_xy      [6]=uv2
// Gouraud textured triangle (9 words, step=3):
//   [0]=c0  [1]=v0_xy  [2]=clut|uv0  [3]=c1  [4]=v1_xy  [5]=tpage|uv1
//   [6]=c2  [7]=v2_xy  [8]=uv2
//
// Line command byte (0x40–0x5F):
//   bit 4: 0=flat, 1=Gouraud
//   bit 3: 0=single segment, 1=poly-line (variable; FIFO terminated by 0x5...)
//
// Sprite/rect command byte (0x60–0x7F):
//   bits [4:3]: size  00=variable, 01=1×1, 10=8×8, 11=16×16
//   bit 2:      0=untextured, 1=textured
// Textured variable rect FIFO: [0]=cmd+color  [1]=xy  [2]=clut|uv  [3]=size
// Textured fixed  rect FIFO:   [0]=cmd+color  [1]=xy  [2]=clut|uv
void GPU::dispatch_draw(u32 cmd) noexcept {
    const s32 ox = off_x();
    const s32 oy = off_y();

    // Helper: decode a Vertex from a color word + coordinate word.
    // u and v are initialised to 0; callers set them for textured commands.
    auto make_v = [&](u32 color_word, u32 coord_word) -> Vertex {
        return Vertex{
            sx11(coord_word) + ox,
            sy11(coord_word) + oy,
            static_cast<s32>((color_word >>  0u) & 0xFFu),
            static_cast<s32>((color_word >>  8u) & 0xFFu),
            static_cast<s32>((color_word >> 16u) & 0xFFu),
            0, 0  // u, v — populated below for textured commands
        };
    };

    // Helper: update GPUSTAT texture page bits and clut_x_/clut_y_ from a
    // polygon's in-FIFO tpage/CLUT attribute words.
    // tpage_word: fifo word whose upper 16 bits are the texture-page attribute.
    // clut_word:  fifo word whose upper 16 bits are the CLUT attribute.
    auto apply_texattr = [&](u32 tpage_word, u32 clut_word) {
        const u32 tpage = tpage_word >> 16u;
        // Polygon texpage only updates GPUSTAT[8:0] (bits 9–10 are E1-only).
        constexpr u32 kClear = 0x1FFu | (1u << 15u);
        u32 ns = (gpustat_ & ~kClear) | (tpage & 0x1FFu) | kReadyMask;
        if (allow_texture_disable_)
            ns = (ns & ~(1u << 15u)) | (((tpage >> 11u) & 1u) << 15u);
        gpustat_ = ns;

        const u32 ca = clut_word >> 16u;
        clut_x_ = (ca & 0x3Fu) * 16u;
        clut_y_ = (ca >> 6u) & 0x1FFu;
    };

    // ── Polygons (0x20–0x3F) ─────────────────────────────────────────────────
    if (cmd >= 0x20u && cmd <= 0x3Fu) {
        const bool gouraud  = (cmd & 0x10u) != 0u;
        const bool quad     = (cmd & 0x08u) != 0u;
        const bool textured = (cmd & 0x04u) != 0u;
        prim_semi_          = (cmd & 0x02u) != 0u;

        if (!gouraud) {
            // Flat-shaded: one color for all vertices (from fifo_[0]).
            // Coordinate slot for vertex n: base + stride*n  (stride 2 if textured)
            // UV+CLUT/tpage slot for vertex n: base + stride*n + 1
            const u32 stride = textured ? 2u : 1u;
            const u32 base   = 1u;
            Vertex v0 = make_v(fifo_[0], fifo_[base + stride * 0u]);
            Vertex v1 = make_v(fifo_[0], fifo_[base + stride * 1u]);
            Vertex v2 = make_v(fifo_[0], fifo_[base + stride * 2u]);
            if (textured) {
                // fifo_[2] = CLUT|UV0,  fifo_[4] = TPAGE|UV1,  fifo_[6] = UV2
                apply_texattr(fifo_[base + stride * 1u + 1u],  // tpage at slot 4
                              fifo_[base + stride * 0u + 1u]); // clut  at slot 2
                v0.u = static_cast<s32>(fifo_[base + stride*0u + 1u] & 0xFFu);
                v0.v = static_cast<s32>((fifo_[base + stride*0u + 1u] >> 8u) & 0xFFu);
                v1.u = static_cast<s32>(fifo_[base + stride*1u + 1u] & 0xFFu);
                v1.v = static_cast<s32>((fifo_[base + stride*1u + 1u] >> 8u) & 0xFFu);
                v2.u = static_cast<s32>(fifo_[base + stride*2u + 1u] & 0xFFu);
                v2.v = static_cast<s32>((fifo_[base + stride*2u + 1u] >> 8u) & 0xFFu);
            }
            raster_tri(v0, v1, v2, textured);
            if (quad) {
                Vertex v3 = make_v(fifo_[0], fifo_[base + stride * 3u]);
                if (textured) {
                    v3.u = static_cast<s32>(fifo_[base + stride*3u + 1u] & 0xFFu);
                    v3.v = static_cast<s32>((fifo_[base + stride*3u + 1u] >> 8u) & 0xFFu);
                }
                raster_tri(v1, v2, v3, textured);
            }
        } else {
            // Gouraud-shaded: each vertex has its own color word.
            // Untextured: color/coord pairs (step=2).
            // Textured:   color/coord/uv triples (step=3).
            //   CLUT at fifo_[2], tpage at fifo_[step+2]=fifo_[5].
            const u32 step = textured ? 3u : 2u;
            Vertex v0 = make_v(fifo_[0u],        fifo_[1u]);
            Vertex v1 = make_v(fifo_[step],      fifo_[step + 1u]);
            Vertex v2 = make_v(fifo_[step * 2u], fifo_[step * 2u + 1u]);
            if (textured) {
                apply_texattr(fifo_[step + 2u],  // tpage at fifo_[5]
                              fifo_[2u]);         // clut  at fifo_[2]
                v0.u = static_cast<s32>(fifo_[2u] & 0xFFu);
                v0.v = static_cast<s32>((fifo_[2u] >> 8u) & 0xFFu);
                v1.u = static_cast<s32>(fifo_[step + 2u] & 0xFFu);
                v1.v = static_cast<s32>((fifo_[step + 2u] >> 8u) & 0xFFu);
                v2.u = static_cast<s32>(fifo_[step*2u + 2u] & 0xFFu);
                v2.v = static_cast<s32>((fifo_[step*2u + 2u] >> 8u) & 0xFFu);
            }
            raster_tri(v0, v1, v2, textured);
            if (quad) {
                Vertex v3 = make_v(fifo_[step * 3u], fifo_[step * 3u + 1u]);
                if (textured) {
                    v3.u = static_cast<s32>(fifo_[step*3u + 2u] & 0xFFu);
                    v3.v = static_cast<s32>((fifo_[step*3u + 2u] >> 8u) & 0xFFu);
                }
                raster_tri(v1, v2, v3, textured);
            }
        }
        return;
    }

    // ── Lines (0x40–0x5F) ────────────────────────────────────────────────────
    if (cmd >= 0x40u && cmd <= 0x5Fu) {
        const bool gouraud  = (cmd & 0x10u) != 0u;
        const bool polyline = (cmd & 0x08u) != 0u;
        prim_semi_          = (cmd & 0x02u) != 0u;

        if (!polyline) {
            // Single segment.
            if (!gouraud) {
                // fifo_: [cmd+col, v0, v1]
                const Vertex v0 = make_v(fifo_[0], fifo_[1]);
                const Vertex v1 = make_v(fifo_[0], fifo_[2]);
                raster_line(v0, v1);
            } else {
                // fifo_: [cmd+c0, v0, c1, v1]
                const Vertex v0 = make_v(fifo_[0], fifo_[1]);
                const Vertex v1 = make_v(fifo_[2], fifo_[3]);
                raster_line(v0, v1);
            }
        } else {
            // Poly-line: terminated by a 0x5... sentinel word.
            if (!gouraud) {
                // fifo_: [cmd+col, v0, v1, v2, ... , 0x5...]
                Vertex prev = make_v(fifo_[0], fifo_[1]);
                for (u32 i = 2u; i < FIFO_CAP; ++i) {
                    if ((fifo_[i] >> 28u) == 0x5u) break;
                    const Vertex cur = make_v(fifo_[0], fifo_[i]);
                    raster_line(prev, cur);
                    prev = cur;
                }
            } else {
                // fifo_: [cmd+c0, v0, c1, v1, c2, v2, ... , 0x5...]
                Vertex prev = make_v(fifo_[0], fifo_[1]);
                for (u32 i = 2u; i + 1u < FIFO_CAP; i += 2u) {
                    if ((fifo_[i] >> 28u) == 0x5u) break;
                    const Vertex cur = make_v(fifo_[i], fifo_[i + 1u]);
                    raster_line(prev, cur);
                    prev = cur;
                }
            }
        }
        return;
    }

    // ── Sprites / rectangles (0x60–0x7F) ─────────────────────────────────────
    if (cmd >= 0x60u && cmd <= 0x7Fu) {
        const u32 r = (fifo_[0] >>  0u) & 0xFFu;
        const u32 g = (fifo_[0] >>  8u) & 0xFFu;
        const u32 b = (fifo_[0] >> 16u) & 0xFFu;
        const s32 x = sx11(fifo_[1]) + ox;
        const s32 y = sy11(fifo_[1]) + oy;
        const bool textured = (cmd & 0x04u) != 0u;
        prim_semi_          = (cmd & 0x02u) != 0u;

        // Extract CLUT and UV origin for textured sprites.
        s32 u_org = -1, v_org = 0;
        if (textured) {
            const u32 ca = fifo_[2] >> 16u;
            clut_x_ = (ca & 0x3Fu) * 16u;
            clut_y_ = (ca >> 6u) & 0x1FFu;
            u_org = static_cast<s32>(fifo_[2] & 0xFFu);
            v_org = static_cast<s32>((fifo_[2] >> 8u) & 0xFFu);
        }

        // Size: bits [4:3] of cmd  — 00=variable, 01=1×1, 10=8×8, 11=16×16
        const u32 size_bits = (cmd >> 3u) & 3u;
        s32 w, h;
        if (size_bits == 0u) {
            // Variable: fifo_[2] is texcoord (textured) or size (untextured).
            const u32 size_word = textured ? fifo_[3] : fifo_[2];
            w = static_cast<s32>( size_word        & 0xFFFFu);
            h = static_cast<s32>((size_word >> 16u) & 0xFFFFu);
            if (w == 0) w = static_cast<s32>(VRAM_W);
            if (h == 0) h = static_cast<s32>(VRAM_H);
        } else {
            const s32 dim = (size_bits == 1u) ? 1 : (size_bits == 2u) ? 8 : 16;
            w = h = dim;
        }
        raster_rect(x, y, w, h,
                    static_cast<u8>(r), static_cast<u8>(g), static_cast<u8>(b),
                    u_org, v_org);
    }
}

// ── GP1 — display control ─────────────────────────────────────────────────────
void GPU::gp1(u32 value) noexcept {
    const u32 cmd   = (value >> 24u) & 0xFFu;
    const u32 param =  value & 0x00FF'FFFFu;

    switch (cmd) {
    case 0x00:          // Reset GPU
        gpustat_              = kReadyMask;
        gpuread_              = 0;
        tex_window_           = 0;
        draw_area_tl_         = 0;
        draw_area_br_         = 0;
        draw_offset_          = 0;
        fifo_len_             = 0;
        fifo_need_            = 1;
        ctvr_cy_              = ctvr_h_;   // deactivate CPU→VRAM (cy >= h)
        vtcr_active_          = false;
        allow_texture_disable_ = false;
        clut_x_               = 0;
        clut_y_               = 0;
        break;

    case 0x01:          // Reset command buffer
        fifo_len_  = 0;
        fifo_need_ = 1;
        break;

    case 0x02:          // Acknowledge GPU interrupt (clear GPUSTAT[24])
        gpustat_ &= ~(1u << 24u);
        break;

    case 0x03:          // Display enable  (param[0]: 0=enabled, 1=disabled)
        gpustat_ = (gpustat_ & ~(1u << 23u))
                 | ((param & 1u) << 23u);
        break;

    case 0x04:          // DMA direction (param[1:0] → GPUSTAT[30:29])
        gpustat_ = (gpustat_ & ~(3u << 29u))
                 | ((param & 3u) << 29u);
        update_dma_request();
        break;

    case 0x05:          // GP1(05) — Display start in VRAM
        disp_x_ =  param        & 0x3FEu;  // bits[9:1]×2, must be even
        disp_y_ = (param >> 10) & 0x1FFu;  // bits[18:10]
        break;
    case 0x06: break;   // Horizontal display range — width via GP1(08) mode bits
    case 0x07:          // GP1(07) — Vertical display range
        disp_v1_ =  param        & 0x3FFu;  // bits[9:0]:  first displayed line
        disp_v2_ = (param >> 10) & 0x3FFu;  // bits[19:10]: line after last
        break;

    case 0x08: {
        // Display mode — map param bits to GPUSTAT:
        //   param[1:0] → GPUSTAT[18:17]  Horizontal res 1
        //   param[2]   → GPUSTAT[19]     Vertical res
        //   param[3]   → GPUSTAT[20]     Video mode (NTSC/PAL)
        //   param[4]   → GPUSTAT[21]     Color depth
        //   param[5]   → GPUSTAT[22]     Vertical interlace
        //   param[6]   → GPUSTAT[16]     Horizontal res 2
        //   param[7]   → GPUSTAT[14]     Reverse flag
        constexpr u32 kClear = (1u << 14u) | (0x7Fu << 16u);   // bits 14,16-22
        gpustat_ = (gpustat_ & ~kClear)
                 | ((param & 0x3u) << 17u)
                 | (((param >>  2u) & 1u) << 19u)
                 | (((param >>  3u) & 1u) << 20u)
                 | (((param >>  4u) & 1u) << 21u)
                 | (((param >>  5u) & 1u) << 22u)
                 | (((param >>  6u) & 1u) << 16u)
                 | (((param >>  7u) & 1u) << 14u);
        // When vertical interlace is off, bit 13 must stay 1.
        if (!((gpustat_ >> 22u) & 1u))
            gpustat_ |= (1u << 13u);
        gpustat_ |= kReadyMask;
        update_dma_request();
        break;
    }

    case 0x09:          // Allow texture disable — stores the flag but does NOT
                        // directly modify GPUSTAT[15]; that bit is only changed
                        // by GP0(E1) bit 11 when this flag is true.
        allow_texture_disable_ = (param & 1u) != 0u;
        break;

    case 0x10:          // Get GPU info — return stored register via GPUREAD
        gpuread_ = get_gpu_info(param & 0xFFu);
        break;

    default: break;     // Silently ignore unknown GP1 commands
    }
}

// ── GPUSTAT[25] DMA request — derived from direction and ready signals ────────
void GPU::update_dma_request() noexcept {
    const u32 dir = (gpustat_ >> 29u) & 3u;
    u32 dreq = 0;
    switch (dir) {
    case 0: dreq = 0;                        break;  // Off
    case 1: dreq = (gpustat_ >> 26u) & 1u;  break;  // FIFO → cmd-ready
    case 2: dreq = (gpustat_ >> 28u) & 1u;  break;  // CPU→GPU → DMA-block-ready
    case 3: dreq = (gpustat_ >> 27u) & 1u;  break;  // GPU→CPU → VRAM-ready
    }
    gpustat_ = (gpustat_ & ~(1u << 25u)) | (dreq << 25u);
}

// ── GP1(10) — GPU info responses ──────────────────────────────────────────────
u32 GPU::get_gpu_info(u32 param) const noexcept {
    switch (param) {
    case 0x02: return tex_window_;    // Texture window
    case 0x03: return draw_area_tl_;  // Draw area top-left
    case 0x04: return draw_area_br_;  // Draw area bottom-right
    case 0x05: return draw_offset_;   // Draw offset
    case 0x07: return 2u;             // GPU type: 2 = late model (SCPH-5501+)
    case 0x08: return 0u;             // Root counter info (not implemented)
    default:   return 0u;
    }
}
