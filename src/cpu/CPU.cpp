#include "CPU.hpp"
#include "bus/Bus.hpp"

#include <cstdio>
#include <cstring>
#include <bit>

CPU::CPU(Bus& bus) : bus_(bus) {}

// ── check_irq() ───────────────────────────────────────────────────────────────
// Reflects the PSX external interrupt line into COP0.Cause.IP[2] (bit 10)
// and takes an interrupt exception if the CPU has it unmasked and enabled.
//
// The external interrupt line is asserted whenever (I_STAT & I_MASK) != 0.
// All PSX hardware interrupts (VBlank, DMA, timers, …) share this single
// external line — the software reads I_STAT to distinguish sources.
//
// COP0.SR bit layout relevant here:
//   Bit  0    IEc — current interrupt enable (0 = disabled)
//   Bits[15:8] IM — hardware/software interrupt masks; IM[2] = bit 10 = IRQ0
//
// COP0.Cause bit layout:
//   Bits[15:8] IP — interrupt pending; IP[2] = bit 10 = hardware IRQ line
//   Bits[6:2]  ExcCode — exception code (set to 0 for Int)
//
void CPU::check_irq() noexcept {
    // Mirror the external IRQ line into Cause.IP[2].
    if (bus_.irq_pending()) {
        cop0_.cause |=  (1u << 10);
    } else {
        cop0_.cause &= ~(1u << 10);
    }

    // Take the interrupt if all three conditions hold:
    //   1. IEc (global interrupt enable) is set
    //   2. The corresponding IM bit is set  (IM[2], bit 10)
    //   3. The corresponding IP bit is set  (IP[2], bit 10)
    // We check IM & IP with an 8-bit mask covering bits [15:8].
    if (cop0_.ie_current() && (cop0_.sr & cop0_.cause & 0xFF00u)) {
        // pc_ is the instruction that would have executed if no IRQ.
        // in_delay_slot_ (set by the previous execute()) tells us if pc_ is
        // in a branch delay slot: if so EPC = branch address = pc_ - 4, BD=1.
        current_epc_ = in_delay_slot_ ? (pc_ - 4u) : pc_;
        current_bd_  = in_delay_slot_;
        in_delay_slot_ = false;  // entering exception vector — not a delay slot
        exception(ExceptionCode::Int);
    }
}

// ── step() ────────────────────────────────────────────────────────────────────
// One full fetch–decode–execute cycle.
//
// Pipeline model:
//   1. Check for a pending hardware interrupt (before every fetch).
//   2. Commit any pending load from the previous cycle (load-delay slot).
//   3. Fetch the instruction at pc_.
//   4. Advance pc_ → next_pc_, set next_pc_ += 4 (default sequential).
//   5. Execute the instruction (which may update next_pc_ for branches).
//
// Branch delay slot:
//   A branch writes its target into next_pc_.  The instruction *at* the old
//   next_pc_ (the delay slot) still executes on the following step() before
//   the redirect takes effect — because we advance pc_ from next_pc_ before
//   calling execute(), not after.
bool CPU::step() {
    // 1. Hardware interrupt check — level sensitive, polled every instruction.
    check_irq();

    // 2. Commit the previous cycle's deferred load.
    commit_load();

    // ── BIOS A-function intercept ─────────────────────────────────────────────
    // When the code does "addiu $t2, $zero, 0xA0; jr $t2; addiu $t1, $zero, N"
    // PC lands on 0x0000_00A0.  Instead of executing our stub (jr $ra) we call
    // the C++ implementation directly so sideloaded EXEs get TTY output.
    if (pc_ == 0x0000'00A0u) {
        bios_a_call(gpr_[9]);  // $t1 = function number
        pc_      = gpr_[31];   // jr $ra — return to the original caller
        next_pc_ = pc_ + 4u;
        in_delay_slot_ = false;
        return true;
    }

    // ── BIOS B-function intercept ─────────────────────────────────────────────
    // When the code does "addiu $t2, $zero, 0xB0; jr $t2; addiu $t1, $zero, N"
    // PC lands on 0x0000_00B0.  We handle selected B functions needed by tests.
    // B(0x17/0x18/0x19) fall through to our RAM-patched B dispatcher.
    if (pc_ == 0x0000'00B0u) {
        const u32 fn = gpr_[9];  // $t1 = function number
        if (fn != 0x17u && fn != 0x18u && fn != 0x19u) {
            if (bios_b_call(fn)) {
                pc_      = gpr_[31];
                next_pc_ = pc_ + 4u;
                in_delay_slot_ = false;
                return true;
            }
        }
    }

    // 3. Fetch.
    const u32 pc_now = pc_;
    const Instruction instr{bus_.read<u32>(pc_now)};

    // 4. Advance program counter.
    pc_      = next_pc_;
    next_pc_ = pc_ + 4;

    // Pre-compute EPC/BD for this instruction (pc_now).
    // in_delay_slot_ was set by the PREVIOUS step's execute() if that
    // instruction was a branch/jump — making pc_now the delay slot.
    // If pc_now is a delay slot: EPC = branch address = pc_now - 4, BD = 1.
    // Otherwise:                 EPC = pc_now (the faulting instruction).
    current_epc_ = in_delay_slot_ ? (pc_now - 4u) : pc_now;
    current_bd_  = in_delay_slot_;
    // Reset now; execute() will set in_delay_slot_ = true if this instruction
    // is a branch/jump (so the NEXT step knows its fetch is a delay slot).
    in_delay_slot_ = false;

    // IBE check for instruction fetch from non-executable regions.
    //
    // On real PSX most non-RAM/ROM addresses bus-error on instruction fetch,
    // but a few IO regions (DMA registers, SPU voice registers) respond normally
    // to bus reads and can hold executable code (the cpu/code-in-io test writes
    // "jr $ra; nop" there and expects clean execution).  Those ranges are
    // excluded from the IBE check so the bus read proceeds normally.
    //
    // Executable / no-IBE regions (physical):
    //   0x00000000 – 0x001FFFFF  RAM
    //   0x1F801080 – 0x1F8010FF  DMA channels (MADR/BCR/CHCR) + DPCR/DICR
    //   0x1F801C00 – 0x1F801E7F  SPU voice + control registers
    //   0x1FC00000 – 0x1FFFFFFF  BIOS ROM
    {
        const u32 phys = pc_now & 0x1FFF'FFFFu;
        const bool is_ram  = phys < 0x0020'0000u;
        const bool is_dma  = phys >= 0x1F80'1080u && phys < 0x1F80'1100u;
        const bool is_spu  = phys >= 0x1F80'1C00u && phys < 0x1F80'1E80u;
        const bool is_bios = phys >= 0x1FC0'0000u;
        if (!is_ram && !is_dma && !is_spu && !is_bios) {
            cop0_.badvaddr = pc_now;
            exception(ExceptionCode::IBE);
            return true;
        }
    }

    // 5. Execute.
    execute(instr);

    return true;
}

// ── exception() ───────────────────────────────────────────────────────────────
// R3000A exception entry sequence (see MIPS architecture manual §5.3):
//
//   1. Write ExcCode into Cause[6:2].
//   2. Preserve Cause.IP[15:8] — the interrupt-pending latches are driven
//      continuously by external hardware and must not be cleared here.
//      Zeroing IP would cause check_irq() to immediately lose any pending
//      interrupt that fired just before the exception was taken.
//   3. Clear Cause.BD[31], then set it only when current_bd_ is true.
//   4. EPC = current_epc_, which was pre-computed by check_irq() (for IRQs)
//      or step() (for execute-time exceptions) using the correct pc_now and
//      in_delay_slot_ values — avoiding the fragile pc_ ± 8 formula.
//   5. Push the KU/IE 2-bit stack in SR so the handler runs in kernel mode
//      with interrupts disabled.
void CPU::exception(ExceptionCode code, u32 ce) noexcept {
    // Exception vectors — selected by SR.BEV (bit 22).
    const bool bev    = cop0_.sr & (1u << 22);
    const u32  vector = bev ? 0xBFC0'0180u : 0x8000'0080u;

    // Update Cause: preserve IP[15:8], clear BD and ExcCode, then set ExcCode.
    // For CpU exceptions, also set CE[29:28] = coprocessor number.
    constexpr u32 kIPMask = 0x0000'FF00u;   // bits [15:8] — keep
    cop0_.cause = (cop0_.cause & kIPMask)
                | (static_cast<u32>(code) << 2)
                | (ce << 28);

    // EPC and BD are pre-computed in check_irq() / step() so this function
    // does not need to know how many times pc_ has been advanced.
    cop0_.epc = current_epc_;
    if (current_bd_) {
        cop0_.cause |= (1u << 31);    // BD bit — exception is in a branch delay slot
    }

    cop0_.push_exception_state();

    pc_      = vector;
    next_pc_ = vector + 4;

    // For "real" unrecoverable exceptions, invoke the user handler registered
    // via hookUnresolvedExceptionHandler().  On real PSX the BIOS dispatches
    // Int (IRQ) and Syscall through their own handlers; the unresolved handler
    // only fires for bus errors, reserved-instruction, overflow, etc.
    // Breakpoint (Bp) is also handled separately (not sent to unresolved handler).
    if (code != ExceptionCode::Int    &&
        code != ExceptionCode::Syscall &&
        code != ExceptionCode::Bp) {
        handle_non_irq_exception();
    }
}

// ── handle_non_irq_exception() ────────────────────────────────────────────────
// Called from exception() for non-IRQ exceptions (IBE, RI, Ov, CpU, …).
//
// hookUnresolvedExceptionHandler(fn) [exception.cpp] writes fn to RAM[0x300]
// (globals table A0, index 0x40 = byte offset 0x100 from A0 base 0x200).
//
// getCurrentThread() [exception.cpp] reads:
//   Process** processes = (Process**)0x108;
//   return processes[0]->thread;
// i.e.  Thread* = *(u32*)( *(u32*)0x108 )
//
// Our sideload stub pre-initialises:
//   RAM[0x108] = 0x0400   (Process struct address)
//   RAM[0x400] = 0x0404   (Process.thread → Thread struct address)
//   Thread struct at 0x0404, size 0xC0 (initially zero)
//
// Thread struct layout (exception.hpp):
//   +0x00  flags
//   +0x04  flags2
//   +0x08  registers (Registers)
//     .r[0..31]  +0x08..+0x87  (r[31]/$ra at Thread+0x84)
//     .returnPC  +0x88          (← set to EPC; handler modifies for IBE)
//     .hi        +0x8C
//     .lo        +0x90
//     .sr        +0x94
//     .cause     +0x98
//   +0x9C  unknown[0]  (exception-thrown flag, set by user handler)
//   +0xA0  unknown[1]  (exception type,         set by user handler)
//
// After saving registers we set $ra = 0x8000_00D0 (the return-from-exception
// stub installed by sideload) and jump PC to fn.  The stub reads returnPC from
// the Thread struct and returns via rfe + jr.
void CPU::handle_non_irq_exception() noexcept {
    // Read user handler address from RAM[0x300].
    const u32 fn = bus_.read<u32>(0x0300u);
    if (fn == 0u) return;   // No handler registered — stay at exception vector.

    // Walk the kernel-data chain to the Thread struct.
    const u32 proc_ptr   = bus_.read<u32>(0x0108u);   // RAM[0x108] = Process*
    if (proc_ptr == 0u) return;
    const u32 thread_ptr = bus_.read<u32>(proc_ptr);   // RAM[Process*+0] = Thread*
    if (thread_ptr == 0u) return;


    // Save key registers into the Thread struct so the user handler can read
    // them via getCurrentThread() and set returnPC for the correct return target.
    bus_.write<u32>(thread_ptr + 0x84u, gpr_[31]);      // r[31] = $ra (interrupt)
    bus_.write<u32>(thread_ptr + 0x88u, cop0_.epc);     // returnPC = EPC (initial)
    bus_.write<u32>(thread_ptr + 0x98u, cop0_.cause);   // cause register

    // Point $ra at the return-from-exception stub (KSEG0 0x8000_00D0).
    // When the handler does jr $ra it lands in the stub, which reads
    // returnPC from the Thread struct and resumes via rfe + jr.
    gpr_[31] = 0x8000'00D0u;

    // Redirect execution to the user exception handler.
    pc_      = fn;
    next_pc_ = fn + 4u;
}

// ── execute() — primary dispatch ─────────────────────────────────────────────
void CPU::execute(Instruction i) {
    switch (i.opcode()) {
    // ── SPECIAL (R-type) ──────────────────────────────────────────────────────
    case Op::SPECIAL: op_special(i); break;

    // ── REGIMM ───────────────────────────────────────────────────────────────
    case Op::REGIMM:  op_regimm(i);  break;

    // ── Jumps ─────────────────────────────────────────────────────────────────
    case Op::J: {
        // J target26: PC = PC[31:28] | (target26 << 2)
        const u32 target = (pc_ & 0xF000'0000u) | (i.target26() << 2);
        next_pc_ = target;
        in_delay_slot_ = true;  // next instruction is the delay slot
        break;
    }
    case Op::JAL: {
        const u32 target = (pc_ & 0xF000'0000u) | (i.target26() << 2);
        set_reg(31, next_pc_); // return address = PC after delay slot
        next_pc_ = target;
        in_delay_slot_ = true;  // next instruction is the delay slot
        break;
    }

    // ── Branches ─────────────────────────────────────────────────────────────
    case Op::BEQ:
        if (gpr_[i.rs()] == gpr_[i.rt()])
            branch(pc_ + static_cast<u32>(i.simm16() << 2));
        in_delay_slot_ = true;  // delay slot always executes (taken or not)
        break;
    case Op::BNE:
        if (gpr_[i.rs()] != gpr_[i.rt()])
            branch(pc_ + static_cast<u32>(i.simm16() << 2));
        in_delay_slot_ = true;
        break;
    case Op::BLEZ:
        if (static_cast<s32>(gpr_[i.rs()]) <= 0)
            branch(pc_ + static_cast<u32>(i.simm16() << 2));
        in_delay_slot_ = true;
        break;
    case Op::BGTZ:
        if (static_cast<s32>(gpr_[i.rs()]) > 0)
            branch(pc_ + static_cast<u32>(i.simm16() << 2));
        in_delay_slot_ = true;
        break;

    // ── Immediate arithmetic / logic ──────────────────────────────────────────
    case Op::ADDI: {
        // Signed addition — raises Overflow exception on overflow.
        const s32 a = static_cast<s32>(gpr_[i.rs()]);
        const s32 b = i.simm16();
        const s32 result = a + b;
        // Overflow: signs of both operands match but differ from result.
        if (((a ^ result) & (b ^ result)) < 0) {
            exception(ExceptionCode::Ov);
        } else {
            set_reg(i.rt(), static_cast<u32>(result));
        }
        break;
    }
    case Op::ADDIU:
        // ADDIU does NOT raise overflow — despite the sign-extension.
        set_reg(i.rt(), gpr_[i.rs()] + static_cast<u32>(i.simm16()));
        break;
    case Op::SLTI:
        set_reg(i.rt(), static_cast<s32>(gpr_[i.rs()]) < i.simm16() ? 1u : 0u);
        break;
    case Op::SLTIU:
        // imm is sign-extended then compared unsigned.
        set_reg(i.rt(), gpr_[i.rs()] < static_cast<u32>(i.simm16()) ? 1u : 0u);
        break;
    case Op::ANDI:
        set_reg(i.rt(), gpr_[i.rs()] & i.uimm16());
        break;
    case Op::ORI:
        set_reg(i.rt(), gpr_[i.rs()] | i.uimm16());
        break;
    case Op::XORI:
        set_reg(i.rt(), gpr_[i.rs()] ^ i.uimm16());
        break;
    case Op::LUI:
        set_reg(i.rt(), i.uimm16() << 16);
        break;

    // ── Coprocessors ─────────────────────────────────────────────────────────
    case Op::COP0: op_cop0(i); break;
    case Op::COP1: // FPU (not present on PSX)
        if (!(cop0_.sr & (1u << 29))) exception(ExceptionCode::CpU, 1);
        // CU1=1: silently ignore (NOP)
        break;
    case Op::COP2: op_cop2(i); break;
    case Op::COP3: // not present on PSX
        if (!(cop0_.sr & (1u << 31))) exception(ExceptionCode::CpU, 3);
        // CU3=1: silently ignore (NOP)
        break;

    // ── Load/Store to coprocessor registers ──────────────────────────────────
    case Op::LWC0:
        if (!(cop0_.sr & (1u << 28))) exception(ExceptionCode::CpU, 0);
        break; // CU0=1: NOP (COP0 has no loadable data register)
    case Op::LWC1:
        if (!(cop0_.sr & (1u << 29))) exception(ExceptionCode::CpU, 1);
        break; // CU1=1: NOP (FPU not present)
    case Op::LWC2:
        if (!(cop0_.sr & (1u << 30))) exception(ExceptionCode::CpU, 2);
        // CU2=1: GTE load — NOP for now (MTC2/CTC2 paths cover most use cases)
        break;
    case Op::LWC3:
        if (!(cop0_.sr & (1u << 31))) exception(ExceptionCode::CpU, 3);
        break; // CU3=1: NOP
    case Op::SWC0:
        if (!(cop0_.sr & (1u << 28))) exception(ExceptionCode::CpU, 0);
        break; // CU0=1: NOP
    case Op::SWC1:
        if (!(cop0_.sr & (1u << 29))) exception(ExceptionCode::CpU, 1);
        break; // CU1=1: NOP
    case Op::SWC2:
        if (!(cop0_.sr & (1u << 30))) exception(ExceptionCode::CpU, 2);
        // CU2=1: GTE store — NOP for now
        break;
    case Op::SWC3:
        if (!(cop0_.sr & (1u << 31))) exception(ExceptionCode::CpU, 3);
        break; // CU3=1: NOP

    // ── Loads ─────────────────────────────────────────────────────────────────
    case Op::LB:  op_lb (i); break;
    case Op::LBU: op_lbu(i); break;
    case Op::LH:  op_lh (i); break;
    case Op::LHU: op_lhu(i); break;
    case Op::LW:  op_lw (i); break;
    case Op::LWL: op_lwl(i); break;
    case Op::LWR: op_lwr(i); break;

    // ── Stores ────────────────────────────────────────────────────────────────
    case Op::SB:  op_sb (i); break;
    case Op::SH:  op_sh (i); break;
    case Op::SW:  op_sw (i); break;
    case Op::SWL: op_swl(i); break;
    case Op::SWR: op_swr(i); break;

    default:
        std::fprintf(stderr, "[CPU] Unknown opcode 0x%02X at PC=0x%08X\n",
                     i.opcode(), pc_ - 4);
        exception(ExceptionCode::RI);
        break;
    }
}

// ── SPECIAL (opcode 0x00) ─────────────────────────────────────────────────────
void CPU::op_special(Instruction i) {
    switch (i.funct()) {
    // ── Shifts ────────────────────────────────────────────────────────────────
    case Funct::SLL:  set_reg(i.rd(), gpr_[i.rt()] << i.shamt());    break;
    case Funct::SRL:  set_reg(i.rd(), gpr_[i.rt()] >> i.shamt());    break;
    case Funct::SRA:  set_reg(i.rd(), static_cast<u32>(
                          static_cast<s32>(gpr_[i.rt()]) >> static_cast<int>(i.shamt())));
                      break;
    case Funct::SLLV: set_reg(i.rd(), gpr_[i.rt()] << (gpr_[i.rs()] & 0x1Fu)); break;
    case Funct::SRLV: set_reg(i.rd(), gpr_[i.rt()] >> (gpr_[i.rs()] & 0x1Fu)); break;
    case Funct::SRAV: set_reg(i.rd(), static_cast<u32>(
                          static_cast<s32>(gpr_[i.rt()]) >> (static_cast<int>(gpr_[i.rs()]) & 0x1F)));
                      break;

    // ── Jumps ─────────────────────────────────────────────────────────────────
    case Funct::JR:
        next_pc_ = gpr_[i.rs()];
        in_delay_slot_ = true;  // next instruction is the delay slot
        break;
    case Funct::JALR:
        set_reg(i.rd(), next_pc_); // save return address (after delay slot)
        next_pc_ = gpr_[i.rs()];
        in_delay_slot_ = true;  // next instruction is the delay slot
        break;

    // ── System calls ──────────────────────────────────────────────────────────
    case Funct::SYSCALL: {
        // PSN00BSDK uses SYSCALL a0=1 / a0=2 as EnterCriticalSection /
        // ExitCriticalSection.  We implement these by manipulating COP0.SR
        // *before* calling exception() so that push_exception_state() captures
        // the right IEc bit in IEp, and RFE restores it.
        const u32 a0 = gpr_[4u];
        if (a0 == 1u) {
            // EnterCriticalSection: clear IEc so interrupts stay off on return.
            cop0_.sr &= ~1u;
        } else if (a0 == 2u) {
            // ExitCriticalSection: set IEc (bit 0) and IM[2] (bit 10) so the
            // hardware interrupt line is unmasked and enabled on return.
            cop0_.sr |= 0x0401u;
        }
        exception(ExceptionCode::Syscall);
        // exception() sets EPC = current_epc_ = pc_now (the SYSCALL address).
        // Advance EPC past it so the sideload exception handler returns to
        // the instruction after the SYSCALL rather than re-triggering it.
        cop0_.epc += 4u;
        break;
    }
    case Funct::BREAK:   exception(ExceptionCode::Bp);      break;

    // ── HI/LO moves ───────────────────────────────────────────────────────────
    case Funct::MFHI: set_reg(i.rd(), hi_); break;
    case Funct::MTHI: hi_ = gpr_[i.rs()];   break;
    case Funct::MFLO: set_reg(i.rd(), lo_); break;
    case Funct::MTLO: lo_ = gpr_[i.rs()];   break;

    // ── Multiply / Divide ─────────────────────────────────────────────────────
    case Funct::MULT: {
        const s64 result = static_cast<s64>(static_cast<s32>(gpr_[i.rs()]))
                         * static_cast<s64>(static_cast<s32>(gpr_[i.rt()]));
        lo_ = static_cast<u32>(result);
        hi_ = static_cast<u32>(static_cast<u64>(result) >> 32);
        break;
    }
    case Funct::MULTU: {
        const u64 result = static_cast<u64>(gpr_[i.rs()]) * static_cast<u64>(gpr_[i.rt()]);
        lo_ = static_cast<u32>(result);
        hi_ = static_cast<u32>(result >> 32);
        break;
    }
    case Funct::DIV: {
        const s32 n = static_cast<s32>(gpr_[i.rs()]);
        const s32 d = static_cast<s32>(gpr_[i.rt()]);
        if (d == 0) {
            lo_ = (n < 0) ? 1u : 0xFFFF'FFFFu;
            hi_ = static_cast<u32>(n);
        } else if (n == std::numeric_limits<s32>::min() && d == -1) {
            lo_ = 0x8000'0000u;
            hi_ = 0u;
        } else {
            lo_ = static_cast<u32>(n / d);
            hi_ = static_cast<u32>(n % d);
        }
        break;
    }
    case Funct::DIVU: {
        const u32 n = gpr_[i.rs()], d = gpr_[i.rt()];
        if (d == 0) { lo_ = 0xFFFF'FFFFu; hi_ = n; }
        else        { lo_ = n / d;         hi_ = n % d; }
        break;
    }

    // ── ALU (register operands) ───────────────────────────────────────────────
    case Funct::ADD: {
        const s32 a = static_cast<s32>(gpr_[i.rs()]);
        const s32 b = static_cast<s32>(gpr_[i.rt()]);
        const s32 r = a + b;
        if (((a ^ r) & (b ^ r)) < 0) exception(ExceptionCode::Ov);
        else set_reg(i.rd(), static_cast<u32>(r));
        break;
    }
    case Funct::ADDU: set_reg(i.rd(), gpr_[i.rs()] + gpr_[i.rt()]); break;
    case Funct::SUB: {
        const s32 a = static_cast<s32>(gpr_[i.rs()]);
        const s32 b = static_cast<s32>(gpr_[i.rt()]);
        const s32 r = a - b;
        if (((a ^ b) & (a ^ r)) < 0) exception(ExceptionCode::Ov);
        else set_reg(i.rd(), static_cast<u32>(r));
        break;
    }
    case Funct::SUBU: set_reg(i.rd(), gpr_[i.rs()] - gpr_[i.rt()]); break;
    case Funct::AND:  set_reg(i.rd(), gpr_[i.rs()] & gpr_[i.rt()]); break;
    case Funct::OR:   set_reg(i.rd(), gpr_[i.rs()] | gpr_[i.rt()]); break;
    case Funct::XOR:  set_reg(i.rd(), gpr_[i.rs()] ^ gpr_[i.rt()]); break;
    case Funct::NOR:  set_reg(i.rd(), ~(gpr_[i.rs()] | gpr_[i.rt()])); break;
    case Funct::SLT:
        set_reg(i.rd(), static_cast<s32>(gpr_[i.rs()]) < static_cast<s32>(gpr_[i.rt()]) ? 1u : 0u);
        break;
    case Funct::SLTU:
        set_reg(i.rd(), gpr_[i.rs()] < gpr_[i.rt()] ? 1u : 0u);
        break;

    default:
        std::fprintf(stderr, "[CPU] Unknown SPECIAL funct 0x%02X at PC=0x%08X\n",
                     i.funct(), pc_ - 4);
        exception(ExceptionCode::RI);
        break;
    }
}

// ── REGIMM (opcode 0x01) ──────────────────────────────────────────────────────
void CPU::op_regimm(Instruction i) {
    const s32 rs = static_cast<s32>(gpr_[i.rs()]);
    const u32 target = pc_ + static_cast<u32>(i.simm16() << 2);

    switch (i.rt()) {
    case RegImm::BLTZ:
        if (rs < 0)  branch(target);
        in_delay_slot_ = true;  // delay slot always executes
        break;
    case RegImm::BGEZ:
        if (rs >= 0) branch(target);
        in_delay_slot_ = true;
        break;
    case RegImm::BLTZAL:
        set_reg(31, next_pc_);
        if (rs < 0)  branch(target);
        in_delay_slot_ = true;
        break;
    case RegImm::BGEZAL:
        set_reg(31, next_pc_);
        if (rs >= 0) branch(target);
        in_delay_slot_ = true;
        break;
    default:
        std::fprintf(stderr, "[CPU] Unknown REGIMM rt=0x%02X at PC=0x%08X\n",
                     i.rt(), pc_ - 4);
        exception(ExceptionCode::RI);
        break;
    }
}

// ── COP0 (opcode 0x10) ────────────────────────────────────────────────────────
void CPU::op_cop0(Instruction i) {
    switch (i.cop_op()) {
    case CopOp::MF:  // MFC0 rt, rd — move COP0[rd] → GPR[rt]
        switch (i.rd()) {
        case 8:  defer_load(i.rt(), cop0_.badvaddr); break;
        case 12: defer_load(i.rt(), cop0_.sr);       break;
        case 13: defer_load(i.rt(), cop0_.cause);    break;
        case 14: defer_load(i.rt(), cop0_.epc);      break;
        default:
            std::fprintf(stderr, "[COP0] MFC0 from unimplemented register %u\n", i.rd());
            defer_load(i.rt(), 0);
            break;
        }
        break;

    case CopOp::MT:  // MTC0 rt, rd — move GPR[rt] → COP0[rd]
        switch (i.rd()) {
        case 3: case 5: case 6: case 7: case 9: case 11:
            // Breakpoint registers — ignored in this skeleton
            break;
        case 12: cop0_.sr    = gpr_[i.rt()]; break;
        case 13: cop0_.cause = gpr_[i.rt()]; break;
        default:
            std::fprintf(stderr, "[COP0] MTC0 to unimplemented register %u\n", i.rd());
            break;
        }
        break;

    default:
        // funct == 0x10 → RFE (Return From Exception)
        if ((i.raw & 0x3Fu) == 0x10u) {
            cop0_.pop_exception_state();
        }
        // Unknown COP0 sub-opcodes are silently ignored (NOP) on PSX.
        break;
    }
}

// ── COP2 / GTE (opcode 0x12) ─────────────────────────────────────────────────
//
// COP2 instruction encoding:
//
//   [31:26] = 0x12  (COP2 primary opcode)
//   [25]    = 1     → GTE command: bits [24:0] are the 25-bit command word
//   [25]    = 0     → Register move; [25:21] selects the operation:
//                       0x00 MFC2  rt ← CP2d[rd]   (load-delayed like LW)
//                       0x02 CFC2  rt ← CP2c[rd]
//                       0x04 MTC2  CP2d[rd] ← rt
//                       0x06 CTC2  CP2c[rd] ← rt
//
// The GTE does not use the standard MIPS load-delay slot for MFC2/CFC2; on
// real hardware there is a two-instruction stall requirement between a GTE
// command and the subsequent MFC2 that reads its result.  Games insert the
// necessary NOPs, so for emulation we model MFC2/CFC2 as immediate (no
// pending-load deferral) which is correct for well-formed code.
void CPU::op_cop2(Instruction i) {
    // COP2 (GTE) is unusable when SR.CU2 (bit 30) is clear.
    if (!(cop0_.sr & (1u << 30))) {
        exception(ExceptionCode::CpU, 2);
        return;
    }

    // Bit 25 set → GTE command word
    if (i.raw & (1u << 25)) {
        gte_.execute(i.raw & 0x01FF'FFFFu, pc_);
        return;
    }

    switch (i.cop_op()) {
    case 0x00:  // MFC2 — move from CP2 data register to GPR
        set_reg(i.rt(), gte_.read_data(i.rd()));
        break;
    case 0x02:  // CFC2 — move from CP2 control register to GPR
        set_reg(i.rt(), gte_.read_ctrl(i.rd()));
        break;
    case 0x04:  // MTC2 — move from GPR to CP2 data register
        gte_.write_data(i.rd(), gpr_[i.rt()]);
        break;
    case 0x06:  // CTC2 — move from GPR to CP2 control register
        gte_.write_ctrl(i.rd(), gpr_[i.rt()]);
        break;
    default:
        std::fprintf(stderr, "[COP2] unknown sub-opcode 0x%02X at PC=0x%08X\n",
                     i.cop_op(), pc_ - 4);
        exception(ExceptionCode::RI);
        break;
    }
}

// ── Load instructions ─────────────────────────────────────────────────────────
// All loads go through defer_load() to implement the one-cycle delay slot.

void CPU::op_lb(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    const u32 val  = static_cast<u32>(static_cast<s8>(bus_.read<u8>(addr)));
    defer_load(i.rt(), val);
}

void CPU::op_lbu(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    defer_load(i.rt(), bus_.read<u8>(addr));
}

void CPU::op_lh(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    if (addr & 1u) { exception(ExceptionCode::AdEL); return; }
    const u32 val = static_cast<u32>(static_cast<s16>(bus_.read<u16>(addr)));
    defer_load(i.rt(), val);
}

void CPU::op_lhu(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    if (addr & 1u) { exception(ExceptionCode::AdEL); return; }
    defer_load(i.rt(), bus_.read<u16>(addr));
}

void CPU::op_lw(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    if (addr & 3u) { exception(ExceptionCode::AdEL); return; }
    defer_load(i.rt(), bus_.read<u32>(addr));
}

// LWL: load word left — merges the most-significant bytes of an unaligned word.
// Addr points to the most-significant byte of the target word.
void CPU::op_lwl(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr      = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    const u32 aligned   = addr & ~3u;
    const u32 word      = bus_.read<u32>(aligned);
    const u32 current   = pending_load_.reg == i.rt() ? pending_load_.value : gpr_[i.rt()];
    static constexpr u32 mask[4]  = {0x00FF'FFFFu, 0x0000'FFFFu, 0x0000'00FFu, 0x0000'0000u};
    static constexpr u32 shift[4] = {24, 16, 8, 0};
    const u32 lane = addr & 3u;
    defer_load(i.rt(), (current & mask[lane]) | (word << shift[lane]));
}

// LWR: load word right — merges the least-significant bytes of an unaligned word.
void CPU::op_lwr(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr    = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    const u32 aligned = addr & ~3u;
    const u32 word    = bus_.read<u32>(aligned);
    const u32 current = pending_load_.reg == i.rt() ? pending_load_.value : gpr_[i.rt()];
    static constexpr u32 mask[4]  = {0x0000'0000u, 0xFF00'0000u, 0xFFFF'0000u, 0xFFFF'FF00u};
    static constexpr u32 shift[4] = {0, 8, 16, 24};
    const u32 lane = addr & 3u;
    defer_load(i.rt(), (current & mask[lane]) | (word >> shift[lane]));
}

// ── Store instructions ────────────────────────────────────────────────────────

void CPU::op_sb(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    bus_.write<u8>(addr, static_cast<u8>(gpr_[i.rt()]));
}

void CPU::op_sh(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    if (addr & 1u) { exception(ExceptionCode::AdES); return; }
    bus_.write<u16>(addr, static_cast<u16>(gpr_[i.rt()]));
}

void CPU::op_sw(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    if (addr & 3u) { exception(ExceptionCode::AdES); return; }
    bus_.write<u32>(addr, gpr_[i.rt()]);
}

// SWL: store word left (unaligned store, most-significant bytes).
void CPU::op_swl(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr    = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    const u32 aligned = addr & ~3u;
    const u32 cur     = bus_.read<u32>(aligned);
    const u32 val     = gpr_[i.rt()];
    static constexpr u32 mask[4]  = {0xFFFF'FF00u, 0xFFFF'0000u, 0xFF00'0000u, 0x0000'0000u};
    static constexpr u32 shift[4] = {24, 16, 8, 0};
    const u32 lane = addr & 3u;
    bus_.write<u32>(aligned, (cur & mask[lane]) | (val >> shift[lane]));
}

// SWR: store word right (unaligned store, least-significant bytes).
void CPU::op_swr(Instruction i) {
    if (cop0_.cache_isolated()) return;
    const u32 addr    = gpr_[i.rs()] + static_cast<u32>(i.simm16());
    const u32 aligned = addr & ~3u;
    const u32 cur     = bus_.read<u32>(aligned);
    const u32 val     = gpr_[i.rt()];
    static constexpr u32 mask[4]  = {0x0000'0000u, 0x0000'00FFu, 0x0000'FFFFu, 0x00FF'FFFFu};
    static constexpr u32 shift[4] = {0, 8, 16, 24};
    const u32 lane = addr & 3u;
    bus_.write<u32>(aligned, (cur & mask[lane]) | (val << shift[lane]));
}

// ── BIOS B-function intercept implementations ─────────────────────────────────
//
// Called from step() when PC == 0x000000B0 and fn != 0x17/0x18/0x19.
// Returns true if the function was handled (step() will do jr $ra).
// Returns false to fall through to the RAM-patched B dispatcher.
//
// B functions used by ps1tests:
//   B(0x12) = SetRCnt(rcnt, target, mode)   — configure a root counter
//   B(0x13) = GetRCnt(rcnt)                 — read root counter value
//   B(0x14) = StartRCnt(rcnt)               — enable root counter IRQ
//   B(0x15) = StopRCnt(rcnt)                — disable root counter IRQ
//
// Root counter register map (base = 0x1F801100):
//   Timer 0: base + 0x00 (counter), +0x04 (mode), +0x08 (target)
//   Timer 1: base + 0x10, +0x14, +0x18
//   Timer 2: base + 0x20, +0x24, +0x28
bool CPU::bios_b_call(u32 fn) noexcept {
    // Timer register helper: IO_BASE + 0x100 = 0x1F801100.
    static constexpr u32 kTimerBase = 0x1F801100u;
    auto rcnt_base = [](u32 n) { return kTimerBase + n * 0x10u; };

    switch (fn) {
    case 0x12u: {  // SetRCnt(rcnt, target, mode) — $a0=rcnt, $a1=target, $a2=mode
        const u32 n      = gpr_[4] & 3u;
        const u32 target = gpr_[5] & 0xFFFFu;
        const u32 mode   = gpr_[6] & 0xFFFFu;
        bus_.write<u32>(rcnt_base(n) + 0x8u, target);  // target register
        bus_.write<u32>(rcnt_base(n) + 0x4u, mode);    // mode (resets counter)
        gpr_[2] = 0;
        return true;
    }
    case 0x13u: {  // GetRCnt(rcnt) — $a0=rcnt, returns counter value
        const u32 n = gpr_[4] & 3u;
        gpr_[2] = bus_.read<u32>(rcnt_base(n));
        return true;
    }
    case 0x14u: {  // StartRCnt(rcnt) — enable IRQ for timer n in I_MASK
        // IRQ sources: TMR0=bit4, TMR1=bit5, TMR2=bit6 in I_MASK (0x1F801074)
        static constexpr u32 kIMask = 0x1F801074u;
        const u32 n    = gpr_[4] & 3u;
        const u32 ibit = 1u << (4u + n);
        bus_.write<u32>(kIMask, bus_.read<u32>(kIMask) | ibit);
        gpr_[2] = 1;
        return true;
    }
    case 0x15u: {  // StopRCnt(rcnt) — disable IRQ for timer n in I_MASK
        static constexpr u32 kIMask = 0x1F801074u;
        const u32 n    = gpr_[4] & 3u;
        const u32 ibit = 1u << (4u + n);
        bus_.write<u32>(kIMask, bus_.read<u32>(kIMask) & ~ibit);
        gpr_[2] = 0;
        return true;
    }
    case 0x5Bu:  // GetC0Table() — return pointer to fake C0 function table
        // The C0 table is pre-filled with no-op stubs in sideload().
        gpr_[2] = 0x0000'0500u;
        return true;

    default:
        // Unknown B function — log and return 0 (null/error).
        std::fprintf(stderr, "[BIOS B] unhandled B(0x%02X) called from 0x%08X\n",
                     fn, gpr_[31]);
        gpr_[2] = 0u;
        return true;
    }
}

// ── BIOS A-function intercept implementations ─────────────────────────────────

// Dispatch table for the most common BIOS A functions used by PSN00BSDK tests.
//   A(0x3C) = putchar(c)          → output one character
//   A(0x3E) = puts(s)             → output null-terminated string + newline
//   A(0x3F) = printf(fmt, ...)    → formatted output
// All other function numbers are silently no-ops (the existing jr-$ra stub
// in RAM still handles the return path correctly).
void CPU::bios_a_call(u32 fn) noexcept {
    switch (fn) {
    case 0x3Cu: {  // putchar(c) — $a0 = character
        bus_.tty_putchar(static_cast<char>(gpr_[4]));
        gpr_[2] = gpr_[4];  // return value = the character
        break;
    }
    case 0x3Eu: {  // puts(s) — $a0 = pointer to null-terminated string
        for (u32 p = gpr_[4]; ; ++p) {
            const char c = static_cast<char>(bus_.read<u8>(p));
            if (!c) break;
            bus_.tty_putchar(c);
        }
        bus_.tty_putchar('\n');
        gpr_[2] = 0u;
        break;
    }
    case 0x3Fu:  // printf(fmt, ...) — $a0 = format string, $a1-$a3 + stack = args
        bios_printf();
        break;
    default:
        break;  // unknown function — silently ignore, return value undefined
    }
}

// Minimal printf implementation for BIOS A(0x3F).
//
// MIPS O32 calling convention for printf:
//   $a0 (gpr_[4])  = format string pointer
//   $a1 (gpr_[5])  = 1st vararg
//   $a2 (gpr_[6])  = 2nd vararg
//   $a3 (gpr_[7])  = 3rd vararg
//   sp+0x10        = 4th vararg
//   sp+0x14        = 5th vararg
//   …
//
// For %s specifiers the argument is a MIPS pointer; the string is read from
// MIPS memory.  All other specifiers treat the arg as a 32-bit integer.
void CPU::bios_printf() noexcept {
    // Read format string from MIPS memory (capped at 511 chars).
    char fmt[512] = {};
    for (u32 i = 0u, p = gpr_[4]; i < sizeof(fmt) - 1u; ++i) {
        if (!(fmt[i] = static_cast<char>(bus_.read<u8>(p + i)))) break;
    }

    // Argument iterator — mirrors the O32 calling convention.
    int argn = 0;
    auto get_arg = [&]() -> u32 {
        u32 v;
        if      (argn == 0) v = gpr_[5];
        else if (argn == 1) v = gpr_[6];
        else if (argn == 2) v = gpr_[7];
        else                v = bus_.read<u32>(gpr_[29] + 0x10u + u32(argn - 3) * 4u);
        ++argn;
        return v;
    };

    // Walk the format string, forwarding each specifier to snprintf so that
    // width, precision, and flag characters are handled correctly.
    for (const char* f = fmt; *f; ) {
        if (*f != '%') { bus_.tty_putchar(*f++); continue; }

        // Capture everything from '%' up to and including the conversion char.
        const char* spec_start = f++;
        while (*f && !std::strchr("diouxXeEfgGcs%", *f)) ++f;
        const char conv = *f ? *f++ : '\0';

        // Build a NUL-terminated copy of the specifier (e.g. "%-14s", "%04x").
        // Also normalize: move any flags (- + space # 0) that appear after width
        // digits back to just after '%'.  Real PSX code uses "%-14s" but some test
        // binaries emit "%14-s" / "%2-d" which confuses standard snprintf wrappers.
        char spec[32] = {};
        {
            char flags[8]  = {}; int fi = 0;
            char width[8]  = {}; int wi = 0;
            char prec[16]  = {}; int pi = 0;
            char lenmod[4] = {}; int li = 0;
            for (const char* p = spec_start + 1; p < f - 1; ++p) {
                const char c = *p;
                if (c == '-' || c == '+' || c == ' ' || c == '#' || c == '0') {
                    if (fi < 7) flags[fi++] = c;
                } else if ((c >= '1' && c <= '9') || c == '*') {
                    if (pi == 0 && wi < 7) width[wi++] = c;
                    else if (pi > 0 && pi < 14) prec[pi++] = c;
                } else if (c == '.') {
                    if (pi == 0 && pi < 14) prec[pi++] = '.';
                } else if (c == 'h' || c == 'l' || c == 'L' ||
                           c == 'z' || c == 't' || c == 'j') {
                    if (li < 3) lenmod[li++] = c;
                }
            }
            char* out = spec;
            *out++ = '%';
            std::memcpy(out, flags,  fi); out += fi;
            std::memcpy(out, width,  wi); out += wi;
            std::memcpy(out, prec,   pi); out += pi;
            std::memcpy(out, lenmod, li); out += li;
            *out++ = conv;
            *out   = '\0';
        }

        if (conv == '%') { bus_.tty_putchar('%'); continue; }

        char out[256] = {};
        if (conv == 's') {
            // %s — read the string from MIPS memory at the arg address.
            char sbuf[256] = {};
            const u32 addr = get_arg();
            if (addr) {
                for (u32 j = 0u; j < sizeof(sbuf) - 1u; ++j) {
                    if (!(sbuf[j] = static_cast<char>(bus_.read<u8>(addr + j)))) break;
                }
            }
            std::snprintf(out, sizeof(out), spec, sbuf);
        } else if (conv == 'c') {
            std::snprintf(out, sizeof(out), spec, static_cast<int>(get_arg()));
        } else if (conv == 'd' || conv == 'i') {
            std::snprintf(out, sizeof(out), spec, static_cast<int>(get_arg()));
        } else {
            // u, x, X, o, e, f, g — treat as unsigned 32-bit.
            std::snprintf(out, sizeof(out), spec, get_arg());
        }

        for (const char* op = out; *op; ++op) bus_.tty_putchar(*op);
    }

    gpr_[2] = 1u;  // non-zero return (number of chars written — approximate)
}
