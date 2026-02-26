#include "Display.hpp"
#include <algorithm>
#include <array>
#include <cstdio>

// ── Constants ─────────────────────────────────────────────────────────────────
// The display window is 2× the native VRAM size for legibility.
// SDL2 scales the 1024×512 texture up to 2048×1024 automatically.
namespace {
    constexpr int kTexW = static_cast<int>(GPU::VRAM_W);
    constexpr int kTexH = static_cast<int>(GPU::VRAM_H);
    constexpr int kWinW = kTexW * 2;
    constexpr int kWinH = kTexH * 2;
}

Display::Display() {
    // Force X11 driver explicitly for VMware compatibility
    SDL_setenv("SDL_VIDEODRIVER", "x11", 1);

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "[Display] SDL_Init failed: %s\n", SDL_GetError());
        return;
    }

    window_ = SDL_CreateWindow(
        "PSX Emulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        kWinW, kWinH,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!window_) {
        std::fprintf(stderr, "[Display] SDL_CreateWindow failed: %s\n", SDL_GetError());
        return;
    }

    // Use vsync-less renderer — we drive timing from the emulated VBlank.
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer_) {
        std::fprintf(stderr, "[Display] SDL_CreateRenderer failed: %s\n", SDL_GetError());
        return;
    }

    // Nearest-neighbor scaling for pixel-art sharpness.
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");

    texture_ = SDL_CreateTexture(
        renderer_,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        kTexW, kTexH);
    if (!texture_) {
        std::fprintf(stderr, "[Display] SDL_CreateTexture failed: %s\n", SDL_GetError());
    }
}

Display::~Display() {
    if (texture_)  SDL_DestroyTexture(texture_);
    if (renderer_) SDL_DestroyRenderer(renderer_);
    if (window_)   SDL_DestroyWindow(window_);
    SDL_Quit();
}

bool Display::poll_events() noexcept {
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        if (ev.type == SDL_QUIT) return false;
        if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) return false;
    }
    return true;
}

void Display::present(const GPU& gpu) noexcept {
    if (!texture_) return;

    // Lock the streaming texture to get a writable pixel buffer.
    void* pixels  = nullptr;
    int   pitch   = 0;
    if (SDL_LockTexture(texture_, nullptr, &pixels, &pitch) != 0) return;

    const auto& vram = gpu.frame_buffer();
    auto* dst = static_cast<uint32_t*>(pixels);
    const int row_words = pitch / static_cast<int>(sizeof(uint32_t));

    for (int y = 0; y < kTexH; ++y) {
        for (int x = 0; x < kTexW; ++x) {
            const uint16_t px = vram[static_cast<uint32_t>(y) * GPU::VRAM_W
                                    + static_cast<uint32_t>(x)];
            // RGB555 → ARGB8888.
            // Scale each 5-bit channel to 8 bits: x8 = (x5 << 3) | (x5 >> 2).
            const uint32_t r5 =  px        & 0x1Fu;
            const uint32_t g5 = (px >>  5u) & 0x1Fu;
            const uint32_t b5 = (px >> 10u) & 0x1Fu;
            const uint32_t r8 = (r5 << 3u) | (r5 >> 2u);
            const uint32_t g8 = (g5 << 3u) | (g5 >> 2u);
            const uint32_t b8 = (b5 << 3u) | (b5 >> 2u);
            dst[y * row_words + x] = 0xFF000000u | (b8 << 16u) | (g8 << 8u) | r8;
        }
    }

    SDL_UnlockTexture(texture_);

    // Crop to the GPU's configured active display area and scale to fill the window.
    const auto dw = static_cast<int>(std::max(gpu.disp_width(),  1u));
    const auto dh = static_cast<int>(std::max(gpu.disp_height(), 1u));
    const SDL_Rect src {
        static_cast<int>(gpu.disp_start_x()),
        static_cast<int>(gpu.disp_start_y()),
        dw, dh
    };
    // Scale to fill the full window using physical-pixel coordinates.
    // SDL_RenderSetLogicalSize is intentionally NOT used here — it activates a
    // logical coordinate system that conflicts with explicit dst rects and causes
    // a black window on SDL 2.30.0 when changed per-frame.
    int winW, winH;
    SDL_GetRendererOutputSize(renderer_, &winW, &winH);
    const SDL_Rect dstRect { 0, 0, winW, winH };
    SDL_RenderClear(renderer_);
    SDL_RenderCopy(renderer_, texture_, &src, &dstRect);
    SDL_RenderPresent(renderer_);
}
