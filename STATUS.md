# PSX Emulator — Development Status

## Test Results Summary

| Test | Status | Notes |
|------|--------|-------|
| `gte/test-all` | **1150/1150 ✓** | |
| `dma/otc-test` | **15/15 ✓** | |
| `gpu/gp0-e1` | **9/9 ✓** | |
| `gpu/mask-bit` | **5/5 ✓** | |
| `cpu/cop` | **2/2 ✓** | |
| `cpu/code-in-io` | **3/3 ✓** | IBE exception + A(0x40) BIOS intercept fixed |
| `timers/timers` | **Passes ✓** | All checks pass; "Done." printed within 20M cycles |
| `dma/chopping` | **Passes ✓** | Burst interleaving with CPU cycle credits |
| `dma/chain-looping` | **Passes ✓** | Chain-end sentinel + DMA IRQ fire correct |
| `gpu/transparency` | **Passes ✓** | All 4 semi-transparency blend modes |
| `cdrom/timing` | **Stuck ❌** | psxcd init hangs; see CDROM section below |

---

## Recent Fixes (this session)

### DMA Chopping Mode (Task E — FIXED)
`dma/chopping.exe` reported 25 cycles regardless of block size.

**Root cause:** The entire transfer ran in one shot — CPU got zero cycles during DMA.

**Fix in `src/dma/DMA.cpp`:**
- CHCR bits [19:16] = chop_dma_window (words per burst), [22:20] = chop_cpu_window.
- Added `chop_credits_` counter to accumulate CPU-cycle debt between bursts.
- `DMA::tick()` drains at most one burst worth of words per call, then credits
  `chop_cpu_window << chop_dma_window` cycles back to the CPU counter.
- Verified: timing now scales linearly with block size.

---

### DMA Chain-End Detection (Task B — FIXED)
`dma/chain-looping.exe` showed `finished=false, irq=false` for both test cases.

**Root cause:** CH2 linked-list handler didn't fire the DMA2 IRQ or clear the busy
bit when it read the `0x00FFFFFF` end-of-list sentinel.

**Fix in `src/dma/DMA.cpp` (CH2 LL path):**
- On sentinel detection: set CHCR bit 24 cleared (transfer complete), call
  `irq_.set(IRQSource::DMA2)`, break the transfer loop.

---

### GPU Semi-Transparency Blend Modes (Task C — FIXED)
`gpu/transparency.exe` produced incorrect blended output.

**Fix in `src/gpu/GPU.cpp` (`put_pixel()`):**
- Reads existing VRAM pixel and applies blend when primitive has semi-transparency
  enabled (opcode bit) and the texel is not the colour-key (0x0000).
- All 4 PSX blend modes implemented (GP0-E1 bits [6:5]):
  - 0: `(src + dst) / 2`
  - 1: `src + dst`  (clamped to 31 per channel)
  - 2: `src - dst`  (clamped to 0)
  - 3: `src + dst/4`

---

### CDROM irq_en_ Gating (Task D — Partial)
**Changes applied in `src/cdrom/CDRom.cpp` / `src/cdrom/CDRom.hpp`:**

- `push_response_n()` now only calls `irq_.set(CDROM)` when the fired INT type is
  enabled by `irq_en_`: `(irq_en_ & (1u << (irq_fl_ - 1u))) != 0`.
  Previously it fired unconditionally, causing spurious CPU exceptions during
  direct-register-polling sections (like `cdrom/timing`'s disc-read timing loop
  which sets `irq_en_=0` before polling).
- `write()` at reg3/index0 now re-asserts the IRQ if `irq_fl_` is pending when
  `irq_en_` is written.
- `irq_en_` initialised to `0x1Fu` (all 5 INT types enabled — matches BIOS default).
- All debug `fprintf` prints and `total_cmds` counter removed.

**Remaining issue:** `cdrom/timing.exe` still hangs at PC=0x80011610 after psxcd
`Init Ok!` is never printed. The test runs in an infinite VBlank IRQ loop without
CDROM interrupts completing. Root cause still under investigation — likely the psxcd
initialization path has a deeper interaction with the IRQ enable/clear sequence that
isn't yet modelled correctly.

**TODO:**
- Trace the exact IRQ-enable sequence the psxcd library uses during `psxcd_init()`.
- Verify that `write(reg3, index=1, val)` (write-1-to-clear) also re-checks whether
  to de-assert the CPU IRQ line after clearing flags.
- Consider whether the CDROM IRQ acknowledgement path in `Bus.cpp` needs to clear
  `I_STAT` bit 2 when `irq_fl_` is cleared (currently only the CPU ISR does it).

---

## Earlier Fixes

### Branch Delay Slot EPC Bug (FIXED)
The CPU was setting EPC to the wrong address when an interrupt fired during a branch
delay slot.

**Fix in `src/cpu/CPU.hpp` / `src/cpu/CPU.cpp`:**
- Added `current_epc_` / `current_bd_` pre-computed fields; all branch/jump handlers
  set `in_delay_slot_ = true`; `check_irq()` / `exception()` use these directly.

### Scratchpad IBE Exception (FIXED — cpu/code-in-io 3/3)
Instruction fetch from scratchpad (`0x1F800000–0x1F8003FF`) now raises Bus Error
Instruction exception (ExceptionCode::IBE = 6).

### A(0x40) hookUnresolvedExceptionHandler (FIXED)
BIOS A-function 0x40 now intercepted and silently no-ops.

### Timer Sync Modes (FIXED — commit `888af9d`)
All four sync modes for Timer 0/1 implemented; HBlank/VBlank edge events modelled.

### `timers/timers` Hang — `$ra` clobber in sideload stubs (FIXED)
Exception handler now saves/restores `$ra` around `irq_dispatch` (Bus.cpp sideload).

---

## Known Issues / Next Work

### 1. CDROM timing (`cdrom/timing`) — Stuck
See CDROM section above. psxcd init doesn't complete; disc-read timing measurements
never execute. The irq_en_ gating fix is logically correct; the init sequence hang
needs further tracing.

### 2. SPU — Not Implemented
24 voices, ADSR, pitch conversion, DMA CH4. Largest remaining task (~16–32h).

### 3. MDEC — Not Implemented

### 4. Other unrun CDROM tests
`cdrom/getloc`, `cdrom/disc-swap`, `cdrom/terminal` — likely need the timing fix to
complete first.

---

## Architecture Notes

### CPU (`src/cpu/CPU.cpp`, `src/cpu/CPU.hpp`)
- MIPS R3000A, no cache, load delay slot modelled via `PendingLoad`
- Exceptions via `exception(ExceptionCode, ce=0)` — uses `current_epc_` / `current_bd_`
- BIOS intercepts at `0x000000A0` (A-table) and `0x000000B0` (B-table)
- GTE (COP2) delegated to `src/gte/GTE.cpp`
- SYSCALL handler: `a0=1` → EnterCS (clear IEc), `a0=2` → ExitCS (set IEc+IM[2]), `epc+=4`

### GPU (`src/gpu/GPU.cpp`)
- Software rasteriser: triangles, quads, lines, rectangles
- VRAM: 1024×512 16bpp
- Semi-transparency: all 4 blend modes (GP0-E1 bits [6:5])
- VBlank every `kVBlankPeriod = 100,000` cycles (real NTSC ≈ 564,480 — kept fast for tests)

### Timers (`src/timers/Timers.cpp`)
- Three 16-bit counters; sys / dot / HBlank-count / sys/8 clock sources
- All sync modes for T0 (HBlank gate) and T1 (VBlank gate) implemented
- T2 sync modes 0/3 stop permanently; 1/2 free-run

### DMA (`src/dma/DMA.cpp`)
- OTC (ch6) ✓, GPU LL (ch2) ✓, GPU VRAM (ch2 sync=2) ✓
- Chopping mode ✓ (burst interleave with CPU cycle credits)
- Chain-end detection + DMA IRQ ✓
- Channels 0, 1, 4, 5: stub — immediately finish to prevent hangs

### CDROM (`src/cdrom/CDRom.cpp`)
- Commands: GetStat, Setloc, ReadN/S, Stop, Pause, Init, Mute/Demute, Setmode,
  Getparam, GetTN, GetTD, SeekL/P, GetID
- Async response timing via `Sched` (s1_ / s2_) + continuous sector stream
- `irq_en_` gating: CDROM only asserts CPU IRQ when INT type is enabled (bit-correct)
- **Known issue:** psxcd init sequence still hangs — see TODO above

### SPU
- Stub only (reads return 0, writes ignored)

### MDEC
- Not implemented

### Joypad (SIO0)
- Digital pad fully implemented (all 16 buttons); IRQ on each byte exchange
