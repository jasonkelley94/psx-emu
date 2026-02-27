#pragma once

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
//       bus.set_buttons(display.buttons());
//       cpu.step();
//       if (vblank_fired) display.present(bus.gpu());
//   }
// ─────────────────────────────────────────────────────────────────────────────
class Display {
public:
    Display();
    ~Display();

    // Process pending OS events and sample input state.
    // Returns false when the user closes the window.
    bool poll_events() noexcept;

    // Current joypad button state (active-low: 0 = pressed, 1 = released).
    [[nodiscard]] u16 buttons() const noexcept { return buttons_; }

    // Blit the GPU VRAM to the screen.  Called once per VBlank (~60 Hz).
    void present(const GPU& gpu) noexcept;

private:
    SDL_Window*         window_     = nullptr;
    SDL_Renderer*       renderer_   = nullptr;
    SDL_Texture*        texture_    = nullptr;
    SDL_GameController* controller_ = nullptr;
    u16                 buttons_    = 0xFFFFu;  // all released
};
