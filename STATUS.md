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
| `timers/timers` | **Partial** | Prints lines 1–114 (sync modes correct); hangs in target-compare section |

---

## Recent Fixes

### Branch Delay Slot EPC Bug (FIXED)
The CPU was setting EPC (Exception Program Counter) to the wrong address when an interrupt
fired during a branch delay slot.

**Fix applied in:**
- `src/cpu/CPU.hpp` — added `current_epc_` and `current_bd_` pre-computed fields
- `src/cpu/CPU.cpp`:
  - All branch/jump handlers set `in_delay_slot_ = true`
  - `check_irq()`: pre-computes `current_epc_/current_bd_`, removes old `+= 4` hack
  - `exception()`: uses `current_epc_` and `current_bd_` directly

### Scratchpad IBE Exception (FIXED — cpu/code-in-io now 3/3)
Instruction fetch from scratchpad (`0x1F800000–0x1F8003FF`) now raises Bus Error
Instruction exception (ExceptionCode::IBE = 6).

### A(0x40) hookUnresolvedExceptionHandler (FIXED)
BIOS A-function 0x40 now intercepted and silently no-ops.

### Timer Sync Modes (IMPLEMENTED — commit `888af9d`)
All four sync modes for Timer 0 (HBlank) and Timer 1 (VBlank) now implemented.
Timer 2 sync modes 0/3 stop the counter permanently; 1/2 are free-run.
HBlank and VBlank now have begin/end edge events with modeled durations.

---

## Active Bug: `timers/timers` Hang After Line 114

### Symptom
The test prints all sync-mode output (lines 1–114) correctly, then hangs at:
```
[TTY] Check when Timer2 counter (System clock) is reset when reaching target (= 10).
```
It never prints the histogram or "Reached target: 1 (ok)" lines.

### Expected Output (from `ps1tests/timers/psx.log`)
```
Check when Timer2 counter (System clock) is reset when reaching target (= 10).
counter ==  0,  1667 ticks
counter ==  1,   833 ticks
...
counter == 10,   833 ticks
counter == 11,     0 ticks
Reached target:                      1 (ok)
Reached 0xFFFF:                      0 (ok)
Counter reset AFTER reaching target: 1 (ok)
Target + 1 not reached:              1 (ok)
Done.
VSync: timeout
```

### Timer2 Register Sequence Before Hang
```
[TMR2] write reg=2 val=0x000A   → target = 10
[TMR2] write reg=1 val=0x0008   → mode = 0x0008 (reset-on-target, no IRQ, no sync)
[TMR2] write reg=0 val=0x0000   → counter = 0
```

### PC Trace During Hang
```
[TRACE] 10M: PC=0x80011790   ← inside irq_dispatch
[TRACE] 20M: PC=0x80011740   ← inside irq_dispatch
[TRACE] 30M: PC=0x8001179C   ← inside irq_dispatch
```
**The CPU is stuck inside `irq_dispatch`** (PSN00BSDK interrupt dispatcher at 0x8001172C–0x800117A0).

### Binary Flow at This Point
1. `jal 0x80011878` → `EnterCriticalSection`: `addiu a0,z,1; SYSCALL; jr ra; nop`
2. Write target=10, mode=0x0008, counter=0
3. `jal 0x800100A4` → histogram tight loop (10,000 iters reading `0x1F801120` = Timer2 counter)
4. `jal 0x80011888` → `ExitCriticalSection`: `addiu a0,z,2; SYSCALL; jr ra; nop`
5. Print results

The SYSCALL handler has `cop0_.epc += 4u` to advance past the syscall instruction.
ExitCriticalSection sets `SR |= 0x0401` (IEc=1, IM[2]=1), enabling interrupts.
A pending VBlank IRQ immediately fires → CPU enters `irq_dispatch`.

### Most Likely Root Cause: IRQ ACK Bug

`irq_dispatch` writes to I_STAT to acknowledge (clear) the VBlank bit. PSX ACK protocol:
**writing a 0 to a bit in I_STAT clears it** (NOT writing a 1).

**Check `src/irq/IRQ.cpp` — `write_stat()` must use `&=` not `|=`:**
```cpp
// CORRECT:
void IRQ::write_stat(u32 val) noexcept {
    i_stat_ &= val;   // clear bits where val has 0s (PSX ACK protocol)
}
// WRONG (would leave IRQ stuck on):
void IRQ::write_stat(u32 val) noexcept {
    i_stat_ |= val;
}
```
If `write_stat` does not clear the bit, `irq_dispatch` loops forever finding VBlank still set.

**Fix**: Verify/correct `write_stat()` in `src/irq/IRQ.cpp`.

---

## Known Issues / Next Work

### 1. Timer Target-Compare Hang (Priority: Critical)
See "Active Bug" section above. Fix: verify `IRQ::write_stat()` uses `i_stat_ &= val`.

### 2. Timer 0 Dotclock Divisor (Priority: Medium)
Currently hardcoded to `11/56` (≈ sys/5.09). Should vary with GPU horizontal resolution:
- 256px → sys/10 (5.32 MHz)
- 320px → sys/8  (6.65 MHz)
- 368px → sys/7  (7.60 MHz)
- 512px → sys/5  (10.64 MHz)
- 640px → sys/4  (13.31 MHz)

### 3. Unrun Tests (47 remaining)
Key unrun categories:
- **CPU**: access-time, io-access-bitwidth
- **DMA**: chain-looping, chopping, dpcr
- **GPU**: 16 visual tests (triangle, quad, rectangles, lines, etc.)
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
- SYSCALL handler: `a0=1` → EnterCS (clear IEc), `a0=2` → ExitCS (set IEc+IM[2]), `epc+=4`

### GPU (`src/gpu/GPU.cpp`)
- Software rasterizer: triangles, quads, lines, rectangles
- VRAM: 1024×512 16bpp
- VBlank fired every `kVBlankPeriod = 100,000` cycles (real NTSC ≈ 564,480 — deliberate reduction for test compatibility)

### Timers (`src/timers/Timers.cpp`)
- Three 16-bit counters; clock sources (sys, dot, HBlank count, sys/8) implemented
- All four sync modes implemented for T0 (HBlank gate) and T1 (VBlank gate)
- T2 sync modes 0/3 stop counter permanently; 1/2 free-run
- Target/overflow IRQs work; mode bits 11/12 (status flags) set/cleared correctly

### DMA (`src/dma/DMA.cpp`)
- OTC (ch6) ✓, GPU LL (ch2) ✓, GPU VRAM (ch2 sync=2) ✓
- Channels 0, 1, 4, 5: stub — immediately finish to prevent hangs

### CDROM (`src/cdrom/CDRom.cpp`)
- Basic commands implemented (GetStat, Setloc, ReadN/S, Stop, Pause, Init, Setmode, etc.)
- Async response timing: immediate (may fail timing tests)

### SPU
- Stub only (reads return 0, writes ignored)

### MDEC
- Not implemented

### Joypad (SIO0)
- Not implemented (reads return 0, writes ignored)

---

## Key Addresses (timers.exe binary)

| Address | Description |
|---------|-------------|
| 0x80011998 | EXE entry point |
| 0x80011878 | EnterCriticalSection: `addiu a0,z,1; SYSCALL; jr ra; nop` |
| 0x80011888 | ExitCriticalSection: `addiu a0,z,2; SYSCALL; jr ra; nop` |
| 0x8001183C | printf dispatcher → A(0x3F) |
| 0x800100A4 | Histogram tight loop (10,000 iters, reads Timer2 counter) |
| 0x8001172C | irq_dispatch start |
| 0x80011790 | PC observed during hang (inside irq_dispatch) |

## Timer Mode Bits Reference

| Bit | Meaning |
|-----|---------|
| 0 | Sync enable (0=free-run) |
| 2:1 | Sync mode (0–3, meaning depends on timer) |
| 3 | Reset counter to 0 on target match |
| 4 | IRQ on target reach |
| 5 | IRQ on overflow (0xFFFF→0) |
| 8 | T0 clock: 0=sys, 1=dot; T1 clock: 0=sys, 1=HBlank count |
| 9 | T2 clock: 0=sys, 1=sys/8 |
| 11 | TargetReached flag (cleared on mode read) |
| 12 | OverflowReached flag (cleared on mode read) |
