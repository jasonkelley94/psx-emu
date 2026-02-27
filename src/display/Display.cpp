#include "Display.hpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>

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
    // Try to use X11 driver for better VMware compatibility
    // Must be set before SDL_Init
    if (std::getenv("SDL_VIDEODRIVER") == nullptr) {
        SDL_setenv("SDL_VIDEODRIVER", "x11", 0);
    }

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
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

    // Open the first available game controller (DualShock, Xbox, etc.).
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            controller_ = SDL_GameControllerOpen(i);
            if (controller_) {
                std::fprintf(stdout, "[Input] Controller: %s\n",
                             SDL_GameControllerName(controller_));
                break;
            }
        }
    }
}

Display::~Display() {
    if (controller_) SDL_GameControllerClose(controller_);
    if (texture_)    SDL_DestroyTexture(texture_);
    if (renderer_)   SDL_DestroyRenderer(renderer_);
    if (window_)     SDL_DestroyWindow(window_);
    SDL_Quit();
}

bool Display::poll_events() noexcept {
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        switch (ev.type) {
        case SDL_QUIT:
            return false;
        case SDL_KEYDOWN:
            if (ev.key.keysym.sym == SDLK_ESCAPE) return false;
            break;
        case SDL_CONTROLLERDEVICEADDED:
            if (!controller_) {
                controller_ = SDL_GameControllerOpen(ev.cdevice.which);
                if (controller_)
                    std::fprintf(stdout, "[Input] Controller connected: %s\n",
                                 SDL_GameControllerName(controller_));
            }
            break;
        case SDL_CONTROLLERDEVICEREMOVED:
            if (controller_ &&
                SDL_GameControllerFromInstanceID(ev.cdevice.which) == controller_) {
                std::fprintf(stdout, "[Input] Controller disconnected\n");
                SDL_GameControllerClose(controller_);
                controller_ = nullptr;
            }
            break;
        default:
            break;
        }
    }

    // ── Sample input state (polled, not event-driven) ───────────────────────
    // Active-low: 0 = pressed, 1 = released.  Keyboard and controller are
    // merged via bitwise AND so either device can press any button.
    // Helper: clear bit N in a u16 (marks button as pressed).
    auto clr = [](u16& w, unsigned bit) { w &= static_cast<u16>(~(1u << bit)); };

    u16 b = 0xFFFFu;

    // Keyboard: WASD + IJKL layout
    const u8* key = SDL_GetKeyboardState(nullptr);
    if (key[SDL_SCANCODE_RETURN])  clr(b, 3);   // Start
    if (key[SDL_SCANCODE_RSHIFT])  clr(b, 0);   // Select
    if (key[SDL_SCANCODE_W])       clr(b, 4);   // D-pad Up
    if (key[SDL_SCANCODE_S])       clr(b, 6);   // D-pad Down
    if (key[SDL_SCANCODE_A])       clr(b, 7);   // D-pad Left
    if (key[SDL_SCANCODE_D])       clr(b, 5);   // D-pad Right
    if (key[SDL_SCANCODE_K])       clr(b, 14);  // Cross  (X)
    if (key[SDL_SCANCODE_L])       clr(b, 13);  // Circle (O)
    if (key[SDL_SCANCODE_I])       clr(b, 12);  // Triangle
    if (key[SDL_SCANCODE_J])       clr(b, 15);  // Square
    if (key[SDL_SCANCODE_Q])       clr(b, 10);  // L1
    if (key[SDL_SCANCODE_E])       clr(b, 8);   // L2
    if (key[SDL_SCANCODE_U])       clr(b, 11);  // R1
    if (key[SDL_SCANCODE_O])       clr(b, 9);   // R2

    // Game controller (DualShock / Xbox / etc.)
    if (controller_) {
        auto pressed = [&](SDL_GameControllerButton btn) {
            return SDL_GameControllerGetButton(controller_, btn) != 0;
        };
        if (pressed(SDL_CONTROLLER_BUTTON_START))         clr(b, 3);
        if (pressed(SDL_CONTROLLER_BUTTON_BACK))          clr(b, 0);
        if (pressed(SDL_CONTROLLER_BUTTON_DPAD_UP))       clr(b, 4);
        if (pressed(SDL_CONTROLLER_BUTTON_DPAD_DOWN))     clr(b, 6);
        if (pressed(SDL_CONTROLLER_BUTTON_DPAD_LEFT))     clr(b, 7);
        if (pressed(SDL_CONTROLLER_BUTTON_DPAD_RIGHT))    clr(b, 5);
        if (pressed(SDL_CONTROLLER_BUTTON_A))             clr(b, 14);  // Cross
        if (pressed(SDL_CONTROLLER_BUTTON_B))             clr(b, 13);  // Circle
        if (pressed(SDL_CONTROLLER_BUTTON_Y))             clr(b, 12);  // Triangle
        if (pressed(SDL_CONTROLLER_BUTTON_X))             clr(b, 15);  // Square
        if (pressed(SDL_CONTROLLER_BUTTON_LEFTSHOULDER))  clr(b, 10);  // L1
        if (pressed(SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)) clr(b, 11);  // R1
        if (pressed(SDL_CONTROLLER_BUTTON_LEFTSTICK))     clr(b, 1);   // L3
        if (pressed(SDL_CONTROLLER_BUTTON_RIGHTSTICK))    clr(b, 2);   // R3

        // Analog triggers → digital: threshold at ~25% pull
        constexpr s16 kTriggerThreshold = 8000;
        if (SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_TRIGGERLEFT)  > kTriggerThreshold)
            clr(b, 8);   // L2
        if (SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) > kTriggerThreshold)
            clr(b, 9);   // R2
    }

    buttons_ = b;
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
            dst[y * row_words + x] = 0xFF000000u | (r8 << 16u) | (g8 << 8u) | b8;
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
