#pragma once

#include <algorithm>
#include <array>
#include "common/Types.hpp"

// ── GPU ────────────────────────────────────────────────────────────────────────
// Two MMIO registers in the I/O window:
//
//   0x1F80_1810  GP0 — draw commands / VRAM data  (write)
//                      GPUREAD response            (read)
//   0x1F80_1814  GP1 — display control commands   (write)
//                      GPUSTAT                    (read)
//
// GPUSTAT bit layout (selected fields):
//   [10:0]  Texture page / draw settings  (from GP0(E1))
//   [12:11] Mask bits                     (from GP0(E6))
//   [13]    Interlace field               (always 1 when interlace off)
//   [15]    Texture disable               (from GP0(E1) bit 11)
//   [18:16] Horizontal resolution         (from GP1(08))
//   [19]    Vertical resolution           (from GP1(08))
//   [20]    Video mode (NTSC=0 / PAL=1)   (from GP1(08))
//   [21]    Color depth                   (from GP1(08))
//   [22]    Vertical interlace            (from GP1(08))
//   [23]    Display disable               (from GP1(03))
//   [24]    IRQ1 pending                  (from GP0(1F))
//   [25]    DMA data request              (computed from direction)
//   [26]    Ready to receive cmd word     (always 1)
//   [27]    Ready to send VRAM to CPU     (always 1)
//   [28]    Ready to receive DMA block    (always 1)
//   [30:29] DMA direction                 (from GP1(04))
// ─────────────────────────────────────────────────────────────────────────────
class GPU {
public:
    // VRAM dimensions (16bpp, 1 MiB total)
    static constexpr u32 VRAM_W = 1024u;
    static constexpr u32 VRAM_H =  512u;

    // ── Bus interface ─────────────────────────────────────────────────────────
    [[nodiscard]] u32 read(u32 offset) const noexcept;
    void              write(u32 offset, u32 value) noexcept;

    // ── Display access ────────────────────────────────────────────────────────
    // Returns the raw 1024×512 RGB555 framebuffer for blitting to the screen.
    [[nodiscard]] const std::array<u16, VRAM_W * VRAM_H>& frame_buffer() const noexcept {
        return vram_;
    }

    // ── VBlank field toggle ───────────────────────────────────────────────────
    // Called by Bus::tick() at each VBlank.  Toggles GPUSTAT bit 31
    // (even/odd interlace field) so software polling this bit can detect frames.
    void toggle_field() noexcept {
        gpustat_ ^= (1u << 31u);
    }

    // ── Dotclock divisor ──────────────────────────────────────────────────────
    // Returns the integer number of system-clock cycles per one dot-clock tick,
    // derived from the horizontal resolution in GPUSTAT[18:16]:
    //   hres2=1           → 368px → divisor 7
    //   hres2=0, hres1=00 → 256px → divisor 10
    //   hres2=0, hres1=01 → 320px → divisor 8   (most common)
    //   hres2=0, hres1=10 → 512px → divisor 5
    //   hres2=0, hres1=11 → 640px → divisor 4
    [[nodiscard]] u32 dot_divisor() const noexcept {
        if ((gpustat_ >> 16u) & 1u) return 7u;   // hres2 = 368px
        switch ((gpustat_ >> 17u) & 3u) {
        case 0:  return 10u;   // 256px
        case 1:  return  8u;   // 320px
        case 2:  return  5u;   // 512px
        default: return  4u;   // 640px
        }
    }

private:
    // Bits 28/27/26 (ready signals) and 13 (interlace field when off) are
    // always forced high in this stub so spin-waits in the BIOS terminate.
    static constexpr u32 kReadyMask = (1u << 28u) | (1u << 27u)
                                    | (1u << 26u) | (1u << 13u);

    u32         gpustat_ = kReadyMask;
    mutable u32 gpuread_ = 0;  // mutable: written by const read() during VRAM→CPU

    // GP0 environment registers — stored so GP1(10) can return them.
    u32 tex_window_   = 0;   // GP0(E2)
    u32 draw_area_tl_ = 0;   // GP0(E3)
    u32 draw_area_br_ = 0;   // GP0(E4)
    u32 draw_offset_  = 0;   // GP0(E5)

    // GP1(09) "Allow Texture Disable" — when false, E1 bit 11 cannot set
    // GPUSTAT[15].  Disabling does NOT clear the bit (it stays until E1 clears it).
    bool allow_texture_disable_ = false;

    // CLUT origin in VRAM (updated from the CLUT attribute word in textured cmds).
    u32 clut_x_ = 0;   // X pixel coordinate in VRAM (= raw_field[5:0] * 16)
    u32 clut_y_ = 0;   // Y pixel coordinate in VRAM (= raw_field[14:6])

    // Set before any put_pixel calls for the current primitive.
    // Controls whether semi-transparency blending is applied.
    bool prim_semi_ = false;

    // ── VRAM ──────────────────────────────────────────────────────────────────
    // 1024 × 512 × 16bpp = 1 MiB.  Safe on the heap — GPU is always owned
    // by a std::unique_ptr<GPU> in Bus.
    std::array<u16, VRAM_W * VRAM_H> vram_{};

    // ── GP0 command FIFO ──────────────────────────────────────────────────────
    // Accumulates words until the current command is complete, then dispatches
    // via gp0_execute().  Variable-length poly-lines use fifo_need_=0 and are
    // terminated when a vertex word has its top nibble equal to 0x5.
    static constexpr u32 FIFO_CAP = 16u;
    std::array<u32, FIFO_CAP> fifo_{};
    u32 fifo_len_  = 0;
    u32 fifo_need_ = 1;  // words needed to complete current command (0=variable)

    // ── CPU→VRAM transfer state ───────────────────────────────────────────────
    // Active while ctvr_cy_ < ctvr_h_.  Incoming GP0 data words bypass the
    // FIFO and are written directly into VRAM two pixels at a time.
    u32 ctvr_x_  = 0, ctvr_y_  = 0;  // VRAM destination origin
    u32 ctvr_w_  = 0, ctvr_h_  = 0;  // transfer extents (pixels)
    u32 ctvr_cx_ = 0, ctvr_cy_ = 0;  // current column/row within transfer

    // ── VRAM→CPU transfer state ───────────────────────────────────────────────
    // Mutable because vram_read_pixel() advances the cursor from const read().
    mutable u32  vtcr_x_      = 0, vtcr_y_  = 0;
    mutable u32  vtcr_w_      = 0, vtcr_h_  = 0;
    mutable u32  vtcr_cx_     = 0, vtcr_cy_ = 0;
    mutable bool vtcr_active_ = false;

    // ── Rasterizer vertex ─────────────────────────────────────────────────────
    // x, y in VRAM coordinates (after draw-offset is applied).
    // r, g, b in [0, 255].  u, v are 8-bit texture coordinates (0–255).
    struct Vertex { s32 x, y, r, g, b, u, v; };

    // ── Private helpers: GP0 command processing ───────────────────────────────
    void              gp0(u32 value) noexcept;
    void              gp1(u32 value) noexcept;
    void              gp0_execute() noexcept;
    void              vram_write_pixel(u16 px) noexcept;
    [[nodiscard]] u16 vram_read_pixel() const noexcept;
    void              update_dma_request() noexcept;
    [[nodiscard]] u32 get_gpu_info(u32 param) const noexcept;

    // Fetch one texel from VRAM using current texture page (GPUSTAT) and CLUT.
    // u, v are 8-bit texture coordinates; wrapping and texture window applied here.
    [[nodiscard]] u16 fetch_texel(s32 u, s32 v) const noexcept;

    // ── Private helpers: coordinate accessors ─────────────────────────────────
    // Sign-extend 11-bit vertex coordinates packed in GP0 data words.
    static s32 sx11(u32 word) noexcept;   // bits [10:0]  → s32
    static s32 sy11(u32 word) noexcept;   // bits [26:16] → s32

    // Draw area bounds (from draw_area_tl_ / draw_area_br_).
    [[nodiscard]] s32 da_x1() const noexcept;
    [[nodiscard]] s32 da_y1() const noexcept;
    [[nodiscard]] s32 da_x2() const noexcept;
    [[nodiscard]] s32 da_y2() const noexcept;

    // Draw offset (11-bit signed, from draw_offset_).
    [[nodiscard]] s32 off_x() const noexcept;
    [[nodiscard]] s32 off_y() const noexcept;

    // ── Private helpers: rasterizer ───────────────────────────────────────────
    // Write one RGB888 pixel to VRAM, clipped to the current draw area.
    void put_pixel(s32 x, s32 y, u8 r, u8 g, u8 b) noexcept;
    // Write one pre-built RGB555 pixel (e.g. a modulated texel) to VRAM.
    void put_pixel(s32 x, s32 y, u16 rgb555) noexcept;

    // Edge-function (half-space) triangle fill with Gouraud/texture interpolation.
    // Flat color: pass identical RGB to all three vertices.
    // textured=true: barycentric UV interpolation; fetches texels via fetch_texel().
    void raster_tri(Vertex v0, Vertex v1, Vertex v2, bool textured = false) noexcept;

    // Axis-aligned filled rectangle, optionally textured.
    // u_org/v_org: 8-bit UV at the top-left corner; -1 = untextured (flat color).
    void raster_rect(s32 x, s32 y, s32 w, s32 h,
                     u8 r, u8 g, u8 b,
                     s32 u_org = -1, s32 v_org = 0) noexcept;

    // Bresenham line between two vertices (Gouraud color interpolation).
    void raster_line(Vertex v0, Vertex v1) noexcept;

    // Top-level drawing-command dispatcher (called from gp0_execute default).
    void dispatch_draw(u32 cmd) noexcept;
};
