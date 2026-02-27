#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <SDL2/SDL.h>

#include "common/Types.hpp"

class IRQ;

constexpr u32 SPU_RAM_SIZE = 512 * 1024u;
constexpr int SPU_NUM_VOICES = 24;
constexpr int SPU_SAMPLE_RATE = 44100;
constexpr int SPU_SAMPLES_PER_FRAME = SPU_SAMPLE_RATE / 60;

class SPU {
public:
    explicit SPU(IRQ& irq);
    ~SPU();

    void reset() noexcept;

    u8* ram() noexcept { return ram_.data(); }

    u32 read(u32 offset) const noexcept;
    void write(u32 offset, u32 value) noexcept;

    void tick() noexcept;
    bool mix(s16* stereo_out, int samples) noexcept;

    void set_ram_ptr(u8* ram, u32* addr) noexcept {
        external_ram_ = ram;
        spu_addr_ptr_ = addr;
    }

private:
    struct Voice {
        u32 addr = 0;
        u32 loop_addr = 0;
        s16 vol_left = 0;
        s16 vol_right = 0;
        u16 pitch = 0;
        u16 adsr = 0;
        u8  adsr_mode = 0;
        s32 adsr_value = 0;
        u32 phase = 0;
        bool playing = false;
        bool key_on = false;
    };

    static constexpr int kAdsrShift = 16;

    void voice_key_on(int v) noexcept;
    void voice_key_off(int v) noexcept;
    s32 compute_adsr(int v, s32 old_val) noexcept;
    s16 interpolate_sample(int v) noexcept;

    IRQ& irq_;

    std::array<u8, SPU_RAM_SIZE> ram_{};
    u8* external_ram_ = nullptr;
    u32* spu_addr_ptr_ = nullptr;

    std::array<Voice, SPU_NUM_VOICES> voices_{};

    u16 ctrl_ = 0;
    u16 stat_ = 0;
    u16 reverb_ = 0;

    u32 irq_addr_ = 0;
    u16 irq_stat_ = 0;

    int volume_left_ = 0;
    int volume_right_ = 0;

    std::array<s16, SPU_SAMPLES_PER_FRAME * 2> mix_buffer_{};

    SDL_AudioDeviceID audio_dev_ = 0;
    bool audio_open_ = false;
};
