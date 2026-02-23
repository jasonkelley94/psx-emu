#pragma once

#include "common/Types.hpp"

// ── MIPS R3000A instruction word ──────────────────────────────────────────────
//
// Three encoding formats:
//
//  R-type  [31:26] op=0   [25:21] rs   [20:16] rt   [15:11] rd
//          [10:6]  shamt  [5:0]   funct
//
//  I-type  [31:26] op     [25:21] rs   [20:16] rt   [15:0]  imm16
//
//  J-type  [31:26] op     [25:0]  target26
//
struct Instruction {
    u32 raw;

    // ── Field extractors ──────────────────────────────────────────────────────
    [[nodiscard]] constexpr u32 opcode() const noexcept { return raw >> 26; }
    [[nodiscard]] constexpr u32 rs()     const noexcept { return (raw >> 21) & 0x1Fu; }
    [[nodiscard]] constexpr u32 rt()     const noexcept { return (raw >> 16) & 0x1Fu; }
    [[nodiscard]] constexpr u32 rd()     const noexcept { return (raw >> 11) & 0x1Fu; }
    [[nodiscard]] constexpr u32 shamt()  const noexcept { return (raw >>  6) & 0x1Fu; }
    [[nodiscard]] constexpr u32 funct()  const noexcept { return raw & 0x3Fu; }

    // Zero-extended 16-bit immediate
    [[nodiscard]] constexpr u32 uimm16() const noexcept { return raw & 0xFFFFu; }

    // Sign-extended 16-bit immediate (cast through s16 preserves sign)
    [[nodiscard]] constexpr s32 simm16() const noexcept {
        return static_cast<s32>(static_cast<s16>(raw & 0xFFFFu));
    }

    // 26-bit jump target (shifted left 2 by the CPU, or'd with PC[31:28])
    [[nodiscard]] constexpr u32 target26() const noexcept { return raw & 0x3FF'FFFFu; }

    // COP sub-opcode (bits [25:21]) — used for MFC0/MTC0 etc.
    [[nodiscard]] constexpr u32 cop_op() const noexcept { return (raw >> 21) & 0x1Fu; }
};

// ── Primary opcode table ──────────────────────────────────────────────────────
namespace Op {
    inline constexpr u32 SPECIAL = 0x00;  // dispatch on funct
    inline constexpr u32 REGIMM  = 0x01;  // dispatch on rt
    inline constexpr u32 J       = 0x02;
    inline constexpr u32 JAL     = 0x03;
    inline constexpr u32 BEQ     = 0x04;
    inline constexpr u32 BNE     = 0x05;
    inline constexpr u32 BLEZ    = 0x06;
    inline constexpr u32 BGTZ    = 0x07;
    inline constexpr u32 ADDI    = 0x08;
    inline constexpr u32 ADDIU   = 0x09;
    inline constexpr u32 SLTI    = 0x0A;
    inline constexpr u32 SLTIU   = 0x0B;
    inline constexpr u32 ANDI    = 0x0C;
    inline constexpr u32 ORI     = 0x0D;
    inline constexpr u32 XORI    = 0x0E;
    inline constexpr u32 LUI     = 0x0F;
    inline constexpr u32 COP0    = 0x10;
    inline constexpr u32 COP1    = 0x11;  // FPU (not present on PSX)
    inline constexpr u32 COP2    = 0x12;  // GTE
    inline constexpr u32 COP3    = 0x13;  // not present on PSX
    inline constexpr u32 LB      = 0x20;
    inline constexpr u32 LH      = 0x21;
    inline constexpr u32 LWL     = 0x22;
    inline constexpr u32 LW      = 0x23;
    inline constexpr u32 LBU     = 0x24;
    inline constexpr u32 LHU     = 0x25;
    inline constexpr u32 LWR     = 0x26;
    inline constexpr u32 SB      = 0x28;
    inline constexpr u32 SH      = 0x29;
    inline constexpr u32 SWL     = 0x2A;
    inline constexpr u32 SW      = 0x2B;
    inline constexpr u32 SWR     = 0x2E;
    inline constexpr u32 LWC0    = 0x30;  // Load Word to COP0
    inline constexpr u32 LWC1    = 0x31;  // Load Word to COP1
    inline constexpr u32 LWC2    = 0x32;  // Load Word to GTE
    inline constexpr u32 LWC3    = 0x33;  // Load Word to COP3
    inline constexpr u32 SWC0    = 0x38;  // Store Word from COP0
    inline constexpr u32 SWC1    = 0x39;  // Store Word from COP1
    inline constexpr u32 SWC2    = 0x3A;  // Store Word from GTE
    inline constexpr u32 SWC3    = 0x3B;  // Store Word from COP3
} // namespace Op

// ── SPECIAL funct codes ───────────────────────────────────────────────────────
namespace Funct {
    inline constexpr u32 SLL     = 0x00;
    inline constexpr u32 SRL     = 0x02;
    inline constexpr u32 SRA     = 0x03;
    inline constexpr u32 SLLV    = 0x04;
    inline constexpr u32 SRLV    = 0x06;
    inline constexpr u32 SRAV    = 0x07;
    inline constexpr u32 JR      = 0x08;
    inline constexpr u32 JALR    = 0x09;
    inline constexpr u32 SYSCALL = 0x0C;
    inline constexpr u32 BREAK   = 0x0D;
    inline constexpr u32 MFHI    = 0x10;
    inline constexpr u32 MTHI    = 0x11;
    inline constexpr u32 MFLO    = 0x12;
    inline constexpr u32 MTLO    = 0x13;
    inline constexpr u32 MULT    = 0x18;
    inline constexpr u32 MULTU   = 0x19;
    inline constexpr u32 DIV     = 0x1A;
    inline constexpr u32 DIVU    = 0x1B;
    inline constexpr u32 ADD     = 0x20;
    inline constexpr u32 ADDU    = 0x21;
    inline constexpr u32 SUB     = 0x22;
    inline constexpr u32 SUBU    = 0x23;
    inline constexpr u32 AND     = 0x24;
    inline constexpr u32 OR      = 0x25;
    inline constexpr u32 XOR     = 0x26;
    inline constexpr u32 NOR     = 0x27;
    inline constexpr u32 SLT     = 0x2A;
    inline constexpr u32 SLTU    = 0x2B;
} // namespace Funct

// ── REGIMM rt codes ───────────────────────────────────────────────────────────
namespace RegImm {
    inline constexpr u32 BLTZ    = 0x00;
    inline constexpr u32 BGEZ    = 0x01;
    inline constexpr u32 BLTZAL  = 0x10;
    inline constexpr u32 BGEZAL  = 0x11;
} // namespace RegImm

// ── COP0 sub-opcodes ──────────────────────────────────────────────────────────
namespace CopOp {
    inline constexpr u32 MF = 0x00;  // Move From COP
    inline constexpr u32 MT = 0x04;  // Move To COP
    inline constexpr u32 RFE= 0x10;  // funct=0x10: RFE
} // namespace CopOp

// ── Exception codes (Cause[6:2]) ──────────────────────────────────────────────
enum class ExceptionCode : u32 {
    Int      = 0x00,  // Hardware interrupt
    AdEL     = 0x04,  // Address error (load / instruction fetch)
    AdES     = 0x05,  // Address error (store)
    IBE      = 0x06,  // Bus error (instruction fetch)
    DBE      = 0x07,  // Bus error (data load/store)
    Syscall  = 0x08,
    Bp       = 0x09,  // Breakpoint
    RI       = 0x0A,  // Reserved instruction
    CpU      = 0x0B,  // Coprocessor unusable
    Ov       = 0x0C,  // Arithmetic overflow
};
