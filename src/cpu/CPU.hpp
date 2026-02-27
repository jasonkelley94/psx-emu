#pragma once

#include <array>
#include <stdexcept>
#include <string_view>
#include <format>
#include "common/Types.hpp"
#include "cpu/Instruction.hpp"
#include "gte/GTE.hpp"

// Forward declaration — CPU holds a reference, not ownership.
class Bus;

// ── COP0 register file ────────────────────────────────────────────────────────
struct COP0 {
    u32 badvaddr = 0;   // r8  — faulting virtual address
    u32 sr       = (1u << 22);   // r12 — Status Register (BEV=1 after reset)
    u32 cause    = 0;   // r13 — Cause of last exception
    u32 epc      = 0;   // r14 — Exception Program Counter

    // ── SR bit helpers ────────────────────────────────────────────────────────
    // IEc (bit 0): current interrupt enable
    [[nodiscard]] bool ie_current()  const noexcept { return sr & 1u; }
    // KUc (bit 1): 0 = kernel mode
    [[nodiscard]] bool kernel_mode() const noexcept { return !(sr & 2u); }
    // ISc (bit 16): isolate cache — loads/stores hit cache, not RAM
    [[nodiscard]] bool cache_isolated() const noexcept { return sr & (1u << 16); }

    // Push the KU/IE 3-level stack on exception entry.
    // Bits [5:0] = {KUo,IEo, KUp,IEp, KUc,IEc}; shift left by 2.
    void push_exception_state() noexcept {
        sr = (sr & ~0x3Fu) | ((sr & 0x0Fu) << 2);
    }

    // Pop (RFE instruction) — shift right by 2, preserve bits [5:4].
    void pop_exception_state() noexcept {
        sr = (sr & ~0x0Fu) | ((sr >> 2) & 0x0Fu);
    }
};

// ── Pending load (load-delay slot) ───────────────────────────────────────────
// On the R3000A the result of a load is not visible to the *immediately*
// following instruction (one-cycle load interlock in hardware; software must
// insert a NOP or independent instruction between a load and its use).
// We model this with a one-entry "pending write" applied at the top of step().
struct PendingLoad {
    u32 reg   = 0;
    u32 value = 0;
};

// ── CPU (MIPS R3000A) ─────────────────────────────────────────────────────────
class CPU {
public:
    explicit CPU(Bus& bus);

    // Advance the CPU by one instruction.  Returns false on unrecoverable
    // error (e.g. double-fault), true otherwise.
    bool step();

    // ── Inspection (debugger / test harness) ──────────────────────────────────
    [[nodiscard]] u32        reg(u32 idx) const noexcept { return gpr_[idx]; }
    [[nodiscard]] u32        pc()         const noexcept { return pc_; }
    [[nodiscard]] u32        hi()         const noexcept { return hi_; }
    [[nodiscard]] u32        lo()         const noexcept { return lo_; }
    [[nodiscard]] const COP0& cop0()      const noexcept { return cop0_; }
    [[nodiscard]] GTE&       gte()              noexcept { return gte_; }

    // ── Sideload initialisation (PS-EXE direct boot) ──────────────────────────
    void set_pc (u32 addr) noexcept { pc_ = addr; next_pc_ = addr + 4u; }
    void set_reg(u32 idx, u32 val) noexcept { if (idx != 0u) gpr_[idx] = val; }
    void set_cop0_sr(u32 val) noexcept { cop0_.sr = val; }

private:
    // ── Register file ─────────────────────────────────────────────────────────
    // gpr_[0] is always kept 0 (writes are silently discarded via set_reg).
    std::array<u32, 32> gpr_{};

    u32  pc_      = PSX::RESET_VECTOR;  // current instruction address
    u32  next_pc_ = PSX::RESET_VECTOR + 4;
    u32  hi_      = 0;
    u32  lo_      = 0;

    // True when the instruction at pc_ (about to execute next step) is in a
    // branch delay slot.  Set by execute() for every branch/jump instruction,
    // regardless of whether the branch is taken.  Reset at the start of each
    // step() after being consumed by check_irq() / the IBE check.
    bool in_delay_slot_ = false;

    // EPC and BD flag pre-computed for the instruction currently being
    // processed.  Set by check_irq() (pre-advance) and step() (post-advance,
    // pre-execute).  exception() reads these instead of using the fragile
    // pc_ ± 4 / pc_ ± 8 formulas that break for non-sequential branch targets.
    u32  current_epc_ = 0;
    bool current_bd_  = false;

    COP0        cop0_{};
    GTE         gte_{};
    PendingLoad pending_load_{};

    Bus& bus_;

    // ── Helpers ───────────────────────────────────────────────────────────────
    // (set_reg is in the public section — it enforces r0=0 and is also used
    //  internally by the execute loop.)

    // Commit the pending load *before* executing the next instruction.
    // This implements the one-cycle load delay slot.
    void commit_load() noexcept {
        set_reg(pending_load_.reg, pending_load_.value);
        pending_load_ = {};
    }

    // Schedule a future load (result available after the next instruction).
    void defer_load(u32 reg, u32 value) noexcept {
        pending_load_ = {reg, value};
    }

    // Trigger a branch: set next_pc_ to target; delay-slot instruction runs.
    void branch(u32 target) noexcept {
        next_pc_ = target;
    }

    // Raise an exception, redirect PC to the exception handler vector.
    // ce: Coprocessor number for CpU exceptions (Cause[29:28]); 0 for others.
    void exception(ExceptionCode code, u32 ce = 0) noexcept;

    // For non-IRQ exceptions: look up the user handler registered via
    // hookUnresolvedExceptionHandler (RAM[0x300]), save key registers to the
    // Thread struct (found via RAM[0x108]→Process→Thread), and redirect PC to
    // the handler with the return-from-exception stub address in $ra.
    void handle_non_irq_exception() noexcept;

    // Check for a pending hardware interrupt and take it if SR enables it.
    // Called at the top of every step(), before the instruction fetch.
    void check_irq() noexcept;

    // ── BIOS function intercepts ──────────────────────────────────────────────
    // When PC == 0x0000_00A0 the CPU has been dispatched to the BIOS A-table.
    // $t1 carries the function number; we handle printf/puts/putchar here so
    // that sideloaded test EXEs get TTY output without a real BIOS ROM.
    void bios_a_call(u32 fn) noexcept;
    bool bios_b_call(u32 fn) noexcept;  // returns true if handled (skip dispatcher)
    void bios_printf() noexcept;  // implementation of A(0x3F) printf

    // ── Instruction group handlers ────────────────────────────────────────────
    void execute(Instruction instr);

    void op_special(Instruction instr);  // opcode 0x00
    void op_regimm (Instruction instr);  // opcode 0x01
    void op_cop0   (Instruction instr);  // opcode 0x10
    void op_cop2   (Instruction instr);  // opcode 0x12 — GTE

    // Loads
    void op_lb (Instruction instr);
    void op_lbu(Instruction instr);
    void op_lh (Instruction instr);
    void op_lhu(Instruction instr);
    void op_lw (Instruction instr);
    void op_lwl(Instruction instr);
    void op_lwr(Instruction instr);

    // Stores
    void op_sb (Instruction instr);
    void op_sh (Instruction instr);
    void op_sw (Instruction instr);
    void op_swl(Instruction instr);
    void op_swr(Instruction instr);
};
