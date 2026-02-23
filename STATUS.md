# PSX Emulator — Development Status

## Test Results Summary

| Test | Status | Notes |
|------|--------|-------|
| `gte/test-all` | **1150/1150 ✓** | |
| `dma/otc-test` | **15/15 ✓** | |
| `gpu/gp0-e1` | **9/9 ✓** | |
| `gpu/mask-bit` | **5/5 ✓** | |
| `cpu/cop` | **2/2 ✓** | |
| `cpu/code-in-io` | **1/3** | `testCodeInRam` passes; scratchpad IBE exception missing |
| `timers/timers` | **Partial** | Runs through all sections but sync modes produce wrong values; see below |

---

## Recent Fixes

### Branch Delay Slot EPC Bug (FIXED)
The CPU was setting EPC (Exception Program Counter) to the wrong address when an interrupt
fired during a branch delay slot. The old code had `cop0_.epc += 4` in `check_irq()` as
a hack to "adjust" EPC from `pc_ - 4` to `pc_`, but this broke for delay-slot instructions
because `exception()` was already computing the correct branch address.

**Fix applied in:**
- `src/cpu/CPU.hpp` — added `current_epc_` and `current_bd_` pre-computed fields
- `src/cpu/CPU.cpp`:
  - All branch/jump handlers (`J`, `JAL`, `BEQ`, `BNE`, `BLEZ`, `BGTZ`, `JR`, `JALR`,
    `BLTZ`, `BGEZ`, `BLTZAL`, `BGEZAL`) now set `in_delay_slot_ = true`
  - `check_irq()`: pre-computes `current_epc_/current_bd_`, removes the `+= 4` hack
  - `step()`: pre-computes `current_epc_/current_bd_` for non-IRQ exceptions
  - `exception()`: uses `current_epc_` and `current_bd_` directly

**Impact:** Timer test no longer hangs in B(0x17) return-from-exception loop. The test
now runs through all sections to completion within the cycle budget.

---

## Known Issues / Next Work

### 1. Timer Sync Modes (Priority: High)
The timers test completes (no hang) but produces wrong values for sync modes. All four
sync modes show identical free-running counter values because sync mode logic is not
implemented in `src/timers/Timers.cpp`.

**What's needed:**
- `src/timers/Timers.hpp/cpp`: Add sync mode gating in `tick()` / `advance()`:
  - **Timer 0** (HBlank sync):
    - Mode 0: Pause counter during HBlank
    - Mode 1: Reset counter to 0 at HBlank start
    - Mode 2: Run during HBlank, pause outside
    - Mode 3: Pause until first HBlank, then free-run
  - **Timer 1** (VBlank sync):
    - Same pattern as Timer 0 but using VBlank signal
  - **Timer 2** (no H/V sync):
    - Mode 0: Stop counter permanently
    - Mode 1: Free run
    - Mode 2: Free run
    - Mode 3: Stop counter permanently
- `src/gpu/GPU.cpp`: Track HBlank/VBlank state and call `timers_.hblank_tick()` /
  `timers_.vblank_tick()` at correct cycle boundaries. VBlank IRQ must be fired.
- VBlank period: Reference shows ~112,000 sys cycles per frame (NTSC). Our current
  `kVBlankPeriod = 100,000` is too short.

**Reference expected output** (from `ps1tests/timers/psx.log`):
```
Timer2 sync modes (System clock):
  0 = Stop counter at current value:  0, 0, 0, 0, ...
  1 = Free Run:                     248, 371, 494, 617, ...
  2 = Free Run:                     248, 371, 494, 617, ...
  3 = Stop counter at current value:  0, 0, 0, 0, ...
Timer1 sync mode 3 with VBlank (Pause until VBlank, then free-run): all 0s until VBlank
Timer0 sync mode 0 (Pause during HBlank): values pause during blanking window
```

### 2. Scratchpad IBE Exception (cpu/code-in-io test — 2 tests failing)
Instruction fetch from scratchpad (`0x1F800000–0x1F8003FF`) must raise Bus Error
Instruction exception (ExceptionCode::IBE = 6).

**Fix needed in `src/cpu/CPU.cpp` `step()`:**
```cpp
// After fetching the PC, before decoding:
const u32 phys = pc_ & 0x1FFFFFFFu;  // strip KSEG bits
if (phys >= 0x1F800000u && phys < 0x1F800400u) {
    exception(ExceptionCode::IBE);
    return true;
}
```
Also add `IBE = 6` to the `ExceptionCode` enum in `src/cpu/CPU.hpp` (or wherever it's defined).

### 3. Frame Period / VBlank Timing
Current `kVBlankPeriod = 100,000` cycles. Real NTSC PSX ≈ `33,868,800 Hz / 60 fps ≈ 564,480` system cycles per frame. However the timer test "frame delay" reads ~112,000–112,500 on real hardware — this reflects the VISIBLE scanlines × cycles per line, not the total frame.

Real NTSC timing:
- Total scanlines: 263 (240 visible + 23 VBlank)
- Cycles per scanline: ~1,711 sys clocks (NTSC dotclock ÷ pixels per line)
- Total frame: ~263 × ~1,712 = ~450,256 sys clocks? No...

Actually from the test output, Timer1 HBlank clock gives ~263–264 HBlank pulses per "frame delay". With 263 lines × 1,711 cycles/line ≈ 450k cycles per full frame, but the test "frame delay" measures only 112k. The "frame delay" in the test waits for EXACTLY one VBlank IRQ, so the VBlank period is what matters.

The ps1-tests frame delay ~112,000 corresponds to the GPU's VBlank IRQ period. Our GPU fires VBlank every 100,000 cycles → too fast by ~12%.

### 4. Unrun Tests (47 remaining)
See `PLAN.md` (or the plan file) for full phase breakdown. Key unrun categories:
- **CPU**: access-time, io-access-bitwidth
- **DMA**: chain-looping, chopping, dpcr
- **GPU**: 16 visual tests (triangle, quad, etc.)
- **GTE**: gte-fuzz
- **Timers**: timer-dump
- **CDROM**: disc-swap, getloc, terminal, timing
- **SPU**: memory-transfer, ram-sandbox, stereo, test, toolbox
- **MDEC**: 9 tests
- **Input**: pad

---

## Architecture Notes

### CPU (`src/cpu/CPU.cpp`, `src/cpu/CPU.hpp`)
- MIPS R3000A, no cache, load delay slot modeled via `PendingLoad`
- Exceptions via `exception(ExceptionCode, ce=0)` — uses `current_epc_` / `current_bd_`
- BIOS intercepts at `0x000000A0` (A-table) and `0x000000B0` (B-table)
- GTE (COP2) delegated to `src/gte/GTE.cpp`

### GPU (`src/gpu/GPU.cpp`)
- Software rasterizer: triangles, quads, lines, rectangles
- VRAM: 1024×512 16bpp
- VBlank fired every `kVBlankPeriod` cycles (currently 100,000 — needs correction)
- HBlank fired every `kHBlankPeriod` cycles (currently 380 — close to correct)

### Timers (`src/timers/Timers.cpp`)
- Three 16-bit counters; clock source handled; target/overflow IRQs work
- Sync modes NOT implemented (all modes behave as free-run)

### DMA (`src/dma/DMA.cpp`)
- OTC (ch6) ✓, GPU LL (ch2) ✓, GPU VRAM (ch2 sync=2) ✓
- Channels 0,1,4,5 stub — immediately finish to prevent hangs

### CDROM (`src/cdrom/CDRom.cpp`)
- Basic commands implemented (GetStat, Setloc, ReadN/S, Stop, Pause, Init, Setmode, etc.)
- Async response timing: immediate (may fail timing tests)

### SPU
- Stub only (reads return 0, writes ignored)

### MDEC
- Not implemented

### Joypad (SIO0)
- Not implemented (reads return 0, writes ignored)
