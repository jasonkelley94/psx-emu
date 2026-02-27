#include "spu/SPU.hpp"
#include "irq/IRQ.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace {
    constexpr int kAdsrRateTable[32] = {
        0,      4,      8,      16,     20,     24,     28,     32,
        40,     48,     56,     64,     80,     96,     112,    128,
        160,    192,    224,    256,    320,    384,    448,    512,
        640,    768,    896,    1024,   1280,   1536,   1792,   2048
    };

    constexpr int kExpDecayTable[32] = {
        0,      4,      8,      16,     20,     24,     28,     32,
        40,     48,     56,     64,     80,     96,     112,    128,
        160,    192,    224,    256,    320,    384,    448,    512,
        640,    768,    896,    1024,   1280,   1536,   1792,   2048
    };
}

SPU::SPU(IRQ& irq) : irq_(irq) {
    ram_.fill(0);

    if (SDL_Init(SDL_INIT_AUDIO) == 0) {
        SDL_AudioSpec want, have;
        SDL_zero(want);
        want.freq = SPU_SAMPLE_RATE;
        want.format = AUDIO_S16LSB;
        want.channels = 2;
        want.samples = 512;
        audio_dev_ = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
        if (audio_dev_ != 0) {
            audio_open_ = true;
            SDL_PauseAudioDevice(audio_dev_, 0);
        }
    }
}

SPU::~SPU() {
    if (audio_dev_ != 0) {
        SDL_CloseAudioDevice(audio_dev_);
    }
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

void SPU::reset() noexcept {
    ctrl_ = 0;
    stat_ = 0;
    reverb_ = 0;
    irq_addr_ = 0;
    irq_stat_ = 0;
    volume_left_ = 0;
    volume_right_ = 0;

    for (auto& v : voices_) {
        v = Voice{};
    }
}

u32 SPU::read(u32 offset) const noexcept {
    // Voice registers (0xC00–0xD7F): 24 voices × 16 bytes each
    if (offset >= 0xC00u && offset < 0xD80u) {
        return 0;
    }

    // Global SPU registers — 32-bit reads return two packed 16-bit registers.
    // The Bus reads 32 bits at word-aligned addresses and extracts halfwords.
    switch (offset) {
    // 0xDA8: lo=Data Transfer FIFO (0), hi=SPUCNT (ctrl_)
    case 0xDA8:
        return static_cast<u32>(ctrl_) << 16u;

    // 0xDAC: lo=Data Transfer Control (0), hi=SPUSTAT
    // SPUSTAT bits[5:0] must mirror SPUCNT bits[5:0] — the BIOS polls this.
    case 0xDAC: {
        const u16 spustat = (stat_ & 0xFFC0u) | (ctrl_ & 0x3Fu);
        return static_cast<u32>(spustat) << 16u;
    }

    default:
        return 0;
    }
}

void SPU::write(u32 offset, u32 value) noexcept {
    // Voice registers (0xC00–0xD7F): 24 voices × 16 bytes each
    if (offset >= 0xC00u && offset < 0xD80u) {
        int v = static_cast<int>((offset - 0xC00u) / 0x10);
        if (v >= 0 && v < SPU_NUM_VOICES) {
            auto& voice = voices_[static_cast<unsigned>(v)];
            switch ((offset - 0xC00u) % 0x10) {
            case 0x0:
                voice.vol_left = static_cast<s16>(static_cast<s8>(value & 0xFF));
                break;
            case 0x2:
                voice.vol_right = static_cast<s16>(static_cast<s8>((value >> 16) & 0xFF));
                break;
            case 0x4:
                voice.pitch = static_cast<u16>(value & 0x3FFF);
                break;
            case 0x8:
                voice.adsr = static_cast<u16>(value & 0xFFFF);
                break;
            case 0xC:
                voice.adsr_mode = static_cast<u8>((value >> 8) & 0xFF);
                break;
            }
        }
        return;
    }

    // Global SPU registers.  The Bus dispatches 16-bit writes with the raw
    // halfword-aligned offset, so we must handle both word and halfword offsets.
    switch (offset) {
    // SPUADDR (0x1F801DA6): set SPU RAM transfer byte address = value × 8
    case 0xDA6:
        if (spu_addr_ptr_)
            *spu_addr_ptr_ = static_cast<u32>(value & 0xFFFFu) * 8u;
        break;

    // SPUDATA (0x1F801DA8): PIO halfword write to SPU RAM, advance by 2
    case 0xDA8: {
        if (external_ram_ && spu_addr_ptr_) {
            const u32 sa = *spu_addr_ptr_ & (SPU_RAM_SIZE - 1u);
            external_ram_[sa]      = static_cast<u8>(value);
            external_ram_[sa + 1u] = static_cast<u8>(value >> 8u);
            *spu_addr_ptr_ = (*spu_addr_ptr_ + 2u) & (SPU_RAM_SIZE - 1u);
        }
        break;
    }

    // SPUCNT (0x1F801DAA): SPU control register
    case 0xDAA:
        ctrl_ = static_cast<u16>(value & 0xFFFFu);
        break;

    // Main volume left/right (0x1F801D80/0x1F801D82)
    case 0xD80:
        volume_left_ = static_cast<s16>(value & 0xFFFFu);
        break;
    case 0xD82:
        volume_right_ = static_cast<s16>(value & 0xFFFFu);
        break;

    default:
        break;  // other SPU registers — silently ignore
    }
}

void SPU::tick() noexcept {
    for (int v = 0; v < SPU_NUM_VOICES; ++v) {
        auto& voice = voices_[static_cast<unsigned>(v)];

        if (voice.key_on) {
            voice.key_on = false;
            voice.playing = true;
            voice.phase = 0;
            voice.adsr_value = 0;
        }

        if (!voice.playing) continue;

        voice.phase += voice.pitch;
        if ((voice.phase >> 14) >= 28) {
            voice.phase = (voice.loop_addr << 14);
            voice.playing = false;
        }
    }
}

bool SPU::mix(s16* stereo_out, int samples) noexcept {
    std::fill(mix_buffer_.begin(), mix_buffer_.end(), 0);

    for (int v = 0; v < SPU_NUM_VOICES; ++v) {
        auto& voice = voices_[static_cast<unsigned>(v)];
        if (!voice.playing) continue;

        for (int i = 0; i < samples * 2; i += 2) {
            s16 sample = interpolate_sample(v);

            int left = mix_buffer_[static_cast<unsigned>(i)] + (sample * voice.vol_left / 128);
            int right = mix_buffer_[static_cast<unsigned>(i + 1)] + (sample * voice.vol_right / 128);

            left = std::clamp(left, -32768, 32767);
            right = std::clamp(right, -32768, 32767);

            mix_buffer_[static_cast<unsigned>(i)] = static_cast<s16>(left);
            mix_buffer_[static_cast<unsigned>(i + 1)] = static_cast<s16>(right);
        }

        voice.phase += voice.pitch;
    }

    if (audio_open_ && audio_dev_ != 0) {
        SDL_QueueAudio(audio_dev_, mix_buffer_.data(), static_cast<unsigned int>(samples * 4));
    }

    if (stereo_out) {
        std::memcpy(stereo_out, mix_buffer_.data(), static_cast<size_t>(samples) * 4);
    }
    return true;
}

void SPU::voice_key_on(int v) noexcept {
    if (v >= 0 && v < SPU_NUM_VOICES) {
        voices_[static_cast<unsigned>(v)].key_on = true;
    }
}

void SPU::voice_key_off(int v) noexcept {
    if (v >= 0 && v < SPU_NUM_VOICES) {
        voices_[static_cast<unsigned>(v)].playing = false;
    }
}

s32 SPU::compute_adsr(int v, s32 old_val) noexcept {
    auto& voice = voices_[static_cast<unsigned>(v)];
    u8 mode = voice.adsr_mode;
    u16 adsr = voice.adsr;

    int attack_rate = kAdsrRateTable[(adsr >> 8) & 0x1F];
    int decay_rate = kAdsrRateTable[16 + ((adsr >> 4) & 0x0F)];
    int sustain_level = ((adsr >> 4) & 0x0F) * 0x100;
    int release_rate = kExpDecayTable[adsr & 0x1F];

    bool sustain_mode = (mode >> 6) & 1;
    bool release_mode = (mode >> 7) & 1;

    if (release_mode) {
        s32 new_val = old_val - (old_val * release_rate / 1024);
        if (new_val < 0) new_val = 0;
        return new_val;
    }

    if (sustain_mode) {
        if (old_val < sustain_level) {
            return std::min<s32>(old_val + attack_rate * 4, sustain_level);
        } else {
            return std::max<s32>(old_val - decay_rate, sustain_level);
        }
    }

    return old_val;
}

s16 SPU::interpolate_sample(int v) noexcept {
    auto& voice = voices_[static_cast<unsigned>(v)];
    const u8* ram = external_ram_ ? external_ram_ : ram_.data();

    u32 pos = voice.phase >> 14;
    if (pos >= SPU_RAM_SIZE - 1) return 0;

    u8 lo = ram[pos];
    u8 hi = ram[pos + 1];

    s16 sample = static_cast<s16>((static_cast<s8>(hi) << 8) | lo);

    u32 frac = voice.phase & 0x3FFF;
    s32 interpolated = static_cast<s32>(sample) * static_cast<s32>(frac) / 16384;

    return static_cast<s16>(std::clamp(interpolated, -32768, 32767));
}
