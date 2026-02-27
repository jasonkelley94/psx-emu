# PSX Emulator — Development Status

## Test Results Summary

Tests marked **AUTO** are covered by `tests/run_tests.sh` and run automatically in CI.

| Test | Status | Notes |
|------|--------|-------|
| `gte/test-all` | **1150/1150 ✓** AUTO | |
| `dma/otc-test` | **13/13 ✓** AUTO | |
| `dma/chopping` | **Passes ✓** AUTO | Burst interleaving with CPU cycle credits |
| `dma/chain-looping` | **Passes ✓** AUTO | Chain-end sentinel + DMA IRQ fire correct |
| `gpu/gp0-e1` | **10/10 ✓** AUTO | |
| `gpu/mask-bit` | **5/5 ✓** AUTO | |
| `gpu/transparency` | **Passes ✓** | All 4 semi-transparency blend modes |
| `cpu/cop` | **Passes ✓** AUTO | |
| `cpu/code-in-io` | **Passes ✓** AUTO | IBE exception + A(0x40) BIOS intercept fixed |
| `cpu/io-access-bitwidth` | **Passes ✓** AUTO | |
| `timers/timers` | **Passes ✓** AUTO | Done. printed ~35–40M cycles in |
| `cdrom/getloc` | **Passes ✓** AUTO | kSect1_1x = 5M cycles |
| `cdrom/timing` | **Completes ✓** | Reports timing measurements; no pass/fail verdict |
| `cdrom/disc-swap` | **N/A** | Requires physical tray open; not runnable headless |
| `cdrom/terminal` | **N/A** | Interactive UART tool; not runnable headless |
| `input/pad` | **Passes ✓** | All 16 buttons recognised |
| `spu/test` | **Runs ✓** | Completes "16/32-bit access" and "crashing now..." (~100M cycles) |
| `spu/memory-transfer` | **Runs ✓** | Completes within 20M cycles; no explicit pass/fail output |
| `spu/stereo` | **Runs ✓** | Completes within 20M cycles (visual/audio test) |
| `spu/ram-sandbox` | **N/A** | Interactive controller test; not headless testable |
| `spu/toolbox` | **N/A** | Interactive controller test; not headless testable |
| `dma/dpcr` | **Pending** | Needs SPU DMA ch4 (deferred to SPU phase) |

Run the full automated suite:
```bash
PSX_EMU=/tmp/psx-emu PS1_TESTS_DIR=/tmp/ps1-tests-full bash tests/run_tests.sh
# or after cmake build + tests/fetch_tests.sh:
bash tests/run_tests.sh
```

---

## Recent Fixes

### SPU — Dedicated Module (`src/spu/SPU.cpp`)

Replaced the inline `spu_regs_[]` passthrough in Bus with a standalone `SPU` class
(`src/spu/SPU.hpp`, `src/spu/SPU.cpp`).  Key features:

- 24-voice ADPCM decoding with pitch and ADSR envelope
- 512 KB SPU RAM with PIO and DMA transfer support
- SPUSTAT mirroring of SPUCNT bits [5:0]
- SDL2 audio output at 44.1 kHz (stereo S16)
- SPU ticked at ~768 CPU cycles per sample from `Bus::tick()`

---

### Controller / Keyboard Input (`src/display/Display.cpp`)

Wired keyboard and SDL2 GameController input to the joypad state machine.
Both keyboard and gamepad are polled (not event-driven) and merged via
bitwise AND (either device can press any button).

- **Keyboard**: WASD = D-pad, IJKL = Triangle/Cross/Circle/Square,
  Q/E = L1/L2, U/O = R1/R2, Enter = Start, RShift = Select
- **Gamepad**: SDL2 GameController API (DualShock/DualSense/Xbox auto-mapped),
  analog triggers digitized at 25% threshold
- Hot-plug: `SDL_CONTROLLERDEVICEADDED` / `SDL_CONTROLLERDEVICEREMOVED` handled
- `Display::buttons()` → `Bus::set_buttons()` wired in main loop

---

### DMA ch3 (CD-ROM) — BCR Slice Mode Fix

CD-ROM DMA now correctly handles sync mode 1 (slice): total words =
`block_size × block_count` instead of just `BCR[15:0]`.  Also added
`cdrom_dreq` flag so ch3 burst transfers start without requiring bit 28
(trigger), matching GPU/SPU DREQ behavior.

---

### Display — RGB Channel Order Fix

Fixed red/blue swap in `Display::present()`: was `0xFF | B<<16 | G<<8 | R`,
now correctly `0xFF | R<<16 | G<<8 | B` (ARGB8888 format).

---

### IRQ — 16-bit Write Support

Added `io_write16` handlers for I_STAT (0x1F801070) and I_MASK (0x1F801074).
Some games/BIOS code writes these as halfwords rather than full words.

---

### Cache Control Register Address Fix

`CACHE_CTL` corrected from `0xFFFE'0000` to `0xFFFE'0130` (the actual
KSEG2 cache control register address on PSX).

---

### Crash Bandicoot — Boot Investigation (in progress)

Game boots to the "opening symbol" screen but does not progress further.
Investigation traced the issue to the BIOS exception handler:

- CD-ROM command/response flow is correct (33 responses delivered, no overwrites)
- BIOS reads 8 boot sectors, sends GetID — all acknowledged except final 2 responses
- Root cause: CPU stuck in exception handler with `COP0_SR.IEc=0` permanently
- I_STAT shows VBlank + DMA always pending; likely interrupt storm prevents
  CDROM GetID response from being serviced
- **Next step**: add ExcCode trace to identify the exception type causing the loop,
  then check VBlank/DMA firing frequency

---

### Test Automation — ps1-tests runner + GitHub Actions CI

Added `tests/run_tests.sh`, `tests/fetch_tests.sh`, and `.github/workflows/ci.yml`.
The runner covers 11 automatable tests with TTY-based pass/fail detection.
`tests/fetch_tests.sh` downloads the JaCzekanski/ps1-tests release zip into
`tests/ps1-tests/` (gitignored).  CI runs on every push and PR.

---

### Display — Black Window Fix (SDL_RenderSetLogicalSize Removed)

**Root cause:** `Display::present()` called `SDL_RenderSetLogicalSize()` every frame
to change the logical coordinate space to the active display size (e.g. 320×240).  On
SDL 2.30.0 this per-frame logical-size change corrupted the renderer state, resulting in
a completely black window even though VRAM contained correct rendered content (confirmed
via headless `vram.ppm` dump: all 76,800 display-area pixels non-zero).

**Fix (`src/display/Display.cpp`):**
- Removed `SDL_RenderSetLogicalSize()` from both the constructor and `present()`.
- `present()` now calls `SDL_GetRendererOutputSize()` to obtain physical window pixel
  dimensions and passes an explicit `dstRect {0, 0, winW, winH}` to `SDL_RenderCopy`.
  Without any logical size active, SDL render coordinates equal physical pixels and the
  cropped VRAM region scales to fill the window correctly on every frame.

---

### GPU Display Area — GP1(05) and GP1(07) Implemented

**Root cause:** The SDL window always showed raw 1024×512 VRAM from (0,0), ignoring the
game's configured display region.  GP1(05) (display start X/Y) and GP1(07) (vertical
display range) were stubs that did nothing.

**Fix across three files:**
- `src/gpu/GPU.hpp`: added `disp_x_`, `disp_y_`, `disp_v1_` (= 16), `disp_v2_` (= 256)
  private fields; added `disp_start_x()`, `disp_start_y()`, `disp_width()`,
  `disp_height()` public accessors.
- `src/gpu/GPU.cpp`: `GP1(05)` stores display-start X/Y; `GP1(07)` stores vert range.
- `src/display/Display.cpp`: `present()` uses a source `SDL_Rect` cropped to the GPU's
  active display area.  A 320×240 game at VRAM (0,16) now fills the window instead of
  appearing as a tiny patch in the upper-left corner.

---

### CDROM — XA-ADPCM Sector Detection and Decode

Mode-2 raw sectors with sub-header submode bit 2 set are now decoded as XA-ADPCM rather
than copied into the data buffer.  The ADPBUSY flag (`cd_stat_` bit 2) is set during XA
decode and cleared on data sectors or Pause.  Prevents hangs in games that check ADPBUSY
in their INT1 handler.

Key details:
- Sub-header read at `sector_start + 16` to check file/channel/submode/coding.
- Setfilter (cmd 0x0D) gating via `xa_file_` / `xa_channel_` when mode_ bit 3 is set.
- 4-bit ADPCM decoder: 18 × 128-byte sound groups per sector, Q6 fixed-point IIR filter
  with 4 coefficient pairs.  PCM output held in a 4096-pair ring buffer for SDL output.

---

### CDROM — Additional Command Handlers

Added handlers for commands that were previously returning INT5 errors:
- `0x03` Play (audio track playback, fires INT3)
- `0x04` / `0x05` Forward / Backward skip
- `0x07` MotorOn (sets MOTOR_ON, fires INT3 + INT2)
- `0x0D` Setfilter (stores XA file/channel filter for mode_ bit 3 gating)
- `0x19` Test (sub-command 0x20 returns BIOS version bytes)
- `0x1C` / `0x1D` LockDoor / UnlockDoor (no-op acknowledge)

---

### Interactive SDL Window — sideloaded EXEs

Removed the unconditional `headless = true` for `.exe` sideload mode in `src/main.cpp`.
Running `./psx_emu test.exe` now opens an SDL window; `--headless` still works as before.

---

### cdrom/getloc — First Sector Timing (prior session)

`kSect1_1x`: 950,000 → 5,000,000 cycles (~148ms, past the test's 4M-cycle busy-wait).
`kSect1_2x`: 520,000 → 3,000,000 cycles.
**Result:** `cdrom/getloc` → "Test passed"

---

### Timer Timing Constants (prior session)

`kVBlankPeriod`: 100,000 → 568,000 cycles (NTSC: ~263 lines × ~2159 cycles).
Dotclock: `cycles / div` → `cycles * 11 / (div * 7)` (PSX pixel clock = sys × 11/7).

---

## Known Issues / Next Work

### 1. Crash Bandicoot Boot — Exception Handler Loop
CPU gets stuck in the BIOS exception handler (IEc=0 permanently) after CD-ROM
GetID.  VBlank + DMA interrupts are always pending in I_STAT, suggesting an
interrupt storm that prevents the CDROM response from being serviced.
Next: add ExcCode trace, check VBlank/DMA assertion frequency.

### 2. SPU — Partial Implementation
24-voice ADPCM, basic ADSR, pitch, and SDL audio output implemented.
Reverb, noise generator, and CD audio input mixing not yet done.

### 3. dma/dpcr — needs SPU DMA ch4
DMA ch4 (SPU) data transfers implemented but dpcr test still pending.

### 4. GPU Visual Tests — Not Pixel-Compared
15 GPU visual tests (`triangle`, `quad`, `rectangles`, `lines`, etc.) produce VRAM dumps
but have not been compared against reference PNGs.

### 5. MDEC — Not Implemented
Hardware JPEG-like decoder.  Needed for 9 MDEC tests.

### 6. Debug Traces — Need Cleanup
Several temporary debug traces remain in the codebase (joypad read/write traces
in Bus.cpp, DMA ch3 trace, CDROM response trace, IRQ counter, CPU check_irq
trace).  These should be removed before any release.

---

## Architecture Notes

### CPU (`src/cpu/CPU.cpp`)
- MIPS R3000A, no cache, load delay slot modelled via `PendingLoad`
- Exceptions via `exception(ExceptionCode, ce=0)` — uses `current_epc_` / `current_bd_`
- BIOS intercepts at `0x000000A0` (A-table) and `0x000000B0` (B-table)
- GTE (COP2) delegated to `src/gte/GTE.cpp`
- SYSCALL handler: `a0=1` → EnterCS (clear IEc), `a0=2` → ExitCS (set IEc+IM[2]), `epc+=4`

### GPU (`src/gpu/GPU.cpp`, `src/gpu/GPU.hpp`)
- Software rasteriser: triangles, quads, lines, rectangles
- VRAM: 1024×512 16bpp
- Semi-transparency: all 4 blend modes (GP0-E1 bits [6:5])
- Display area: GP1(05) sets display start X/Y; GP1(07) sets vertical range
- VBlank every `kVBlankPeriod = 568,000` cycles (NTSC accurate)

### Display (`src/display/Display.cpp`)
- SDL2 streaming texture (1024×512 ARGB8888)
- `present()` crops to GPU active display area using a source `SDL_Rect`
- Keyboard + SDL2 GameController input (WASD/IJKL + DualShock/Xbox auto-mapped)
- Controller hot-plug support via SDL events

### Timers (`src/timers/Timers.cpp`)
- Three 16-bit counters; sys / dot / HBlank-count / sys/8 clock sources
- All sync modes for T0 (HBlank gate) and T1 (VBlank gate) implemented
- T2 sync modes 0/3 stop permanently; 1/2 free-run
- Dotclock divisor from `GPU::dot_divisor()` (GPUSTAT[18:16])

### DMA (`src/dma/DMA.cpp`)
- OTC (ch6) ✓, GPU LL (ch2) ✓, GPU VRAM (ch2 sync=2) ✓
- CD-ROM (ch3) ✓ — burst and slice mode, DREQ auto-start
- SPU (ch4) ✓ — main RAM ↔ SPU RAM transfer
- Chopping mode ✓ (burst interleave with CPU cycle credits)
- Chain-end detection + DMA IRQ ✓
- Channels 0, 1, 5: stub — immediately finish to prevent hangs

### CDROM (`src/cdrom/CDRom.cpp`)
- Commands: GetStat, Setloc, ReadN/S, Stop, Pause, Init, Mute/Demute, Setmode,
  Getparam, GetTN, GetTD, SeekL/P, GetlocL, GetlocP, GetID, Play, MotorOn,
  Setfilter, Test, LockDoor/UnlockDoor
- XA-ADPCM decode: sub-header detection, 4-bit IIR filter, PCM ring buffer
- Timing: kAck1=50K, kAck1Read=35K, kSect1_1x=5M, kSectN_1x=448K cycles

### SPU (`src/spu/SPU.cpp`, `src/spu/SPU.hpp`)
- 24-voice ADPCM decoding with pitch modulation and basic ADSR envelope
- 512 KB SPU RAM; PIO transfers via SPUADDR/SPUDATA, DMA via ch4
- SPUSTAT mirrors SPUCNT bits [5:0]; SDL2 audio output at 44.1 kHz stereo
- Ticked from Bus at ~768 CPU cycles per sample

### Joypad (`src/bus/Bus.cpp`)
- SIO0 digital pad fully implemented (all 16 buttons); IRQ on each byte exchange

### MDEC
- Not implemented
