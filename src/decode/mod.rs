//! Instruction decode: a raw 32-bit word + PC + register file → a [`Uop`].
//!
//! The raw-field extractors mirror the C `inst_t` bit-field union; [`decode`]
//! reproduces `Core_decode` as one `match` on the opcode, reading the
//! `rs1`/`rs2` operands eagerly (as the C did) so Execute never touches the
//! register file. No architectural state is mutated.

mod uop;

pub use uop::{AluSrc1, AluSrc2, CsrOp, CsrSource, MemOp, RdSource, SystemKind, Uop};

use crate::alu::{AluOp, BranchType};
use crate::bus::Width;

// --- Opcodes (the RV32I subset we implement) -------------------------------
const OP: u32 = 0x33;
const OP_IMM: u32 = 0x13;
const LOAD: u32 = 0x03;
const STORE: u32 = 0x23;
const BRANCH: u32 = 0x63;
const JAL: u32 = 0x6F;
const JALR: u32 = 0x67;
const AUIPC: u32 = 0x17;
const LUI: u32 = 0x37;
const SYSTEM: u32 = 0x73;
const MISC_MEM: u32 = 0x0F;

// --- SYSTEM funct12 (when funct3 == 0) -------------------------------------
const FUNCT12_ECALL: u32 = 0x000;
const FUNCT12_EBREAK: u32 = 0x001;
const FUNCT12_MRET: u32 = 0x302;
const FUNCT12_WFI: u32 = 0x105;

// --- Raw bit-field extractors ----------------------------------------------
#[inline]
fn opcode(raw: u32) -> u32 {
    raw & 0x7F
}
#[inline]
fn rd(raw: u32) -> u8 {
    ((raw >> 7) & 0x1F) as u8
}
#[inline]
fn funct3(raw: u32) -> u32 {
    (raw >> 12) & 0x7
}
#[inline]
fn rs1(raw: u32) -> u8 {
    ((raw >> 15) & 0x1F) as u8
}
#[inline]
fn rs2(raw: u32) -> u8 {
    ((raw >> 20) & 0x1F) as u8
}
#[inline]
fn funct7(raw: u32) -> u32 {
    (raw >> 25) & 0x7F
}
/// SYSTEM `funct12` / Zicsr CSR address — the I-type immediate field.
#[inline]
fn funct12(raw: u32) -> u32 {
    (raw >> 20) & 0xFFF
}

// --- Immediate decoders (sign-extended, returned as bit patterns) ----------
#[inline]
fn imm_i(raw: u32) -> u32 {
    ((raw as i32) >> 20) as u32
}
#[inline]
fn imm_s(raw: u32) -> u32 {
    let v = ((raw >> 25) << 5) | ((raw >> 7) & 0x1F);
    (((v << 20) as i32) >> 20) as u32 // sign-extend 12 bits
}
#[inline]
fn imm_b(raw: u32) -> u32 {
    let v = (((raw >> 31) & 1) << 12)
        | (((raw >> 7) & 1) << 11)
        | (((raw >> 25) & 0x3F) << 5)
        | (((raw >> 8) & 0xF) << 1);
    (((v << 19) as i32) >> 19) as u32 // sign-extend 13 bits
}
#[inline]
fn imm_u(raw: u32) -> u32 {
    raw & 0xFFFF_F000
}
#[inline]
fn imm_j(raw: u32) -> u32 {
    let v = (((raw >> 31) & 1) << 20)
        | (((raw >> 12) & 0xFF) << 12)
        | (((raw >> 20) & 1) << 11)
        | (((raw >> 21) & 0x3FF) << 1);
    (((v << 11) as i32) >> 11) as u32 // sign-extend 21 bits
}

/// Width + sign-extension for a `LOAD`/`STORE` `funct3` (shared mapping).
fn mem_width(f3: u32) -> Width {
    match f3 {
        0 | 4 => Width::Byte, // LB / LBU / SB
        1 | 5 => Width::Half, // LH / LHU / SH
        _ => Width::Word,     // LW / SW (and the unused codes, as in C)
    }
}

/// ALU op for an `OP` / `OP_IMM` instruction. `func7` distinguishes
/// `ADD`/`SUB` and `SRL`/`SRA`; for `OP_IMM`, `SUB` is impossible (`ADDI`
/// always adds) and `SRAI` carries `func7 == 0x20` in its immediate.
fn arith_alu_op(is_reg_reg: bool, f3: u32, f7: u32) -> AluOp {
    match f3 {
        0 => {
            if is_reg_reg && f7 != 0 {
                AluOp::Sub
            } else {
                AluOp::Add
            }
        }
        1 => AluOp::Sll,
        2 => AluOp::Slt,
        3 => AluOp::Sltu,
        4 => AluOp::Xor,
        5 => {
            if f7 != 0 {
                AluOp::Sra
            } else {
                AluOp::Srl
            }
        }
        6 => AluOp::Or,
        _ => AluOp::And, // f3 == 7
    }
}

/// Branch condition for a `BRANCH` `funct3`. `None` for the two reserved codes
/// (2, 3) — the decoder marks those illegal (the C version left them as a
/// stray unconditional jump; this port closes that hole).
fn branch_cond(f3: u32) -> Option<BranchType> {
    match f3 {
        0 => Some(BranchType::Eq),
        1 => Some(BranchType::Neq),
        4 => Some(BranchType::Lt),
        5 => Some(BranchType::Ge),
        6 => Some(BranchType::Ltu),
        7 => Some(BranchType::Geu),
        _ => None,
    }
}

/// Decode `raw` (the instruction at `pc`) against register file `gpr`.
pub fn decode(raw: u32, pc: u32, gpr: &[u32; 32]) -> Uop {
    let mut u = Uop::blank(pc);
    let (rs1, rs2, rd, f3, f7) = (rs1(raw), rs2(raw), rd(raw), funct3(raw), funct7(raw));
    let read = |i: u8| gpr[i as usize];

    match opcode(raw) {
        OP => {
            u.rs1 = rs1;
            u.rs2 = rs2;
            u.rd = rd;
            u.rs1_val = read(rs1);
            u.rs2_val = read(rs2);
            u.alu_src1 = AluSrc1::Rs1;
            u.alu_src2 = AluSrc2::Rs2;
            u.alu_op = arith_alu_op(true, f3, f7);
            u.rd_source = RdSource::AluResult;
        }
        OP_IMM => {
            u.rs1 = rs1;
            u.rd = rd;
            u.rs1_val = read(rs1);
            u.imm = imm_i(raw);
            u.alu_src1 = AluSrc1::Rs1;
            u.alu_src2 = AluSrc2::Imm;
            u.alu_op = arith_alu_op(false, f3, f7);
            u.rd_source = RdSource::AluResult;
        }
        LOAD => {
            u.rs1 = rs1;
            u.rd = rd;
            u.rs1_val = read(rs1);
            u.imm = imm_i(raw);
            u.alu_src1 = AluSrc1::Rs1;
            u.alu_src2 = AluSrc2::Imm;
            u.alu_op = AluOp::Add;
            u.rd_source = RdSource::MemLoad;
            u.mem = Some(MemOp {
                store: false,
                width: mem_width(f3),
                load_signext: f3 != 4 && f3 != 5, // not LBU / LHU
            });
        }
        STORE => {
            u.rs1 = rs1;
            u.rs2 = rs2;
            u.rs1_val = read(rs1);
            u.rs2_val = read(rs2);
            u.imm = imm_s(raw);
            u.alu_src1 = AluSrc1::Rs1;
            u.alu_src2 = AluSrc2::Imm;
            u.alu_op = AluOp::Add;
            u.rd_source = RdSource::Skip;
            u.mem = Some(MemOp {
                store: true,
                width: mem_width(f3),
                load_signext: false,
            });
        }
        BRANCH => {
            u.rs1 = rs1;
            u.rs2 = rs2;
            u.rs1_val = read(rs1);
            u.rs2_val = read(rs2);
            u.imm = imm_b(raw);
            u.alu_src1 = AluSrc1::Pc;
            u.alu_src2 = AluSrc2::Imm;
            u.alu_op = AluOp::Add;
            u.rd_source = RdSource::Skip;
            match branch_cond(f3) {
                Some(bt) => u.branch = Some(bt),
                None => u.illegal = true,
            }
        }
        JAL => {
            u.rd = rd;
            u.imm = imm_j(raw);
            u.alu_src1 = AluSrc1::Pc;
            u.alu_src2 = AluSrc2::Imm;
            u.alu_op = AluOp::Add;
            u.branch = Some(BranchType::JumpAnyway);
            u.rd_source = RdSource::PcPlus4;
        }
        JALR => {
            u.rs1 = rs1;
            u.rd = rd;
            u.rs1_val = read(rs1);
            u.imm = imm_i(raw);
            u.alu_src1 = AluSrc1::Rs1;
            u.alu_src2 = AluSrc2::Imm;
            u.alu_op = AluOp::Add;
            u.branch = Some(BranchType::JumpAnyway);
            u.rd_source = RdSource::PcPlus4;
        }
        AUIPC => {
            u.rd = rd;
            u.imm = imm_u(raw);
            u.alu_src1 = AluSrc1::Pc;
            u.alu_src2 = AluSrc2::Imm;
            u.alu_op = AluOp::Add;
            u.rd_source = RdSource::AluResult;
        }
        LUI => {
            u.rd = rd;
            u.imm = imm_u(raw);
            u.alu_src1 = AluSrc1::Zero;
            u.alu_src2 = AluSrc2::Imm;
            u.alu_op = AluOp::Add;
            u.rd_source = RdSource::AluResult;
        }
        MISC_MEM => {
            // FENCE / FENCE.TSO / PAUSE → NOP. FENCE.I (func3 == 1) is Zifencei,
            // out of scope → illegal.
            u.rd_source = RdSource::Skip;
            if f3 != 0 {
                u.illegal = true;
            }
        }
        SYSTEM => decode_system(&mut u, raw, rs1, rd, f3, read),
        _ => u.illegal = true,
    }

    u
}

/// SYSTEM-opcode sub-decode: trap-control ops (`func3 == 0`) discriminated by
/// `funct12`, or the Zicsr family.
fn decode_system(u: &mut Uop, raw: u32, rs1: u8, rd: u8, f3: u32, read: impl Fn(u8) -> u32) {
    u.rd_source = RdSource::Skip;
    match f3 {
        0 => {
            // ECALL / EBREAK / MRET / WFI — rs1 and rd must be 0.
            if rs1 != 0 || rd != 0 {
                u.illegal = true;
                return;
            }
            u.system = match funct12(raw) {
                FUNCT12_ECALL => SystemKind::Ecall,
                FUNCT12_EBREAK => SystemKind::Ebreak,
                FUNCT12_MRET => SystemKind::Mret,
                FUNCT12_WFI => SystemKind::Wfi,
                _ => {
                    u.illegal = true;
                    return;
                }
            };
        }
        4 => u.illegal = true, // reserved
        _ => {
            // Zicsr. func3 ∈ {1,2,3} are register variants; {5,6,7} are imm5.
            let addr = funct12(raw) as u16;
            let is_imm = f3 >= 5;
            let op = match f3 {
                1 | 5 => CsrOp::Rw,
                2 | 6 => CsrOp::Rs,
                _ => CsrOp::Rc, // 3 | 7
            };
            let source = if is_imm {
                CsrSource::Imm(rs1 as u32) // uimm5 lives in the rs1 slot
            } else {
                CsrSource::Reg(read(rs1))
            };
            // CSRRW always writes; CSRRS/CSRRC write only when source reg != x0.
            let writes = matches!(op, CsrOp::Rw) || rs1 != 0;
            u.rd = rd;
            u.system = SystemKind::Csr {
                op,
                addr,
                source,
                writes,
            };
        }
    }
}
