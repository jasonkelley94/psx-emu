#pragma once

#include <string_view>
#include <SDL2/SDL.h>
#include "gpu/GPU.hpp"

// ── Display ───────────────────────────────────────────────────────────────────
// Wraps an SDL2 window that presents the GPU VRAM on each VBlank.
//
// The native PSX VRAM is 1024×512 @ RGB555.  We scale the window ×2 for a
// more comfortable 2048×1024 display, but keep the texture at 1024×512 and
// let SDL2's renderer scale it up.
//
// Usage:
//   Display display;
//   while (display.poll_events()) {
//       cpu.step();
//       if (vblank_fired) display.present(bus.gpu());
//   }
// ─────────────────────────────────────────────────────────────────────────────
class Display {
public:
    Display();
    ~Display();

    // Process pending OS events.  Returns false when the user closes the window.
    bool poll_events() noexcept;

    // Blit the GPU VRAM to the screen.  Called once per VBlank (~60 Hz).
    void present(const GPU& gpu) noexcept;

private:
    SDL_Window*   window_   = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    // ARGB8888 streaming texture — uploaded every frame from the RGB555 VRAM.
    SDL_Texture*  texture_  = nullptr;
};
