//! Decode sanity checks — the Rust port of the C `tests/test_decode.c`, plus
//! coverage for the illegal/FENCE paths the C test did not exercise.
//!
//! "Decoding" here means: take a raw 32-bit word and confirm the resulting
//! [`Uop`] control signals match what the RV32I spec says about the example.
//! The Core is never run.

use iss_core::alu::{AluOp, BranchType};
use iss_core::bus::Width;
use iss_core::decode::{decode, AluSrc1, AluSrc2, MemOp, RdSource, SystemKind, Uop};

/// Decode with a zeroed register file at PC 0 (the field-extraction cases never
/// depend on register contents).
fn dec(raw: u32) -> Uop {
    decode(raw, 0, &[0u32; 32])
}

#[test]
fn r_type_add() {
    // add x5, x6, x7  -> 0x007302B3
    let u = dec(0x0073_02B3);
    assert_eq!(u.rd, 5);
    assert_eq!(u.rs1, 6);
    assert_eq!(u.rs2, 7);
    assert_eq!(u.alu_op, AluOp::Add);
    assert_eq!(u.alu_src1, AluSrc1::Rs1);
    assert_eq!(u.alu_src2, AluSrc2::Rs2);
    assert_eq!(u.rd_source, RdSource::AluResult);
    assert!(!u.illegal);
}

#[test]
fn r_type_sub() {
    // sub x5, x6, x7  -> 0x407302B3 (func7 = 0x20)
    let u = dec(0x4073_02B3);
    assert_eq!(u.alu_op, AluOp::Sub);
    assert_eq!(u.rd, 5);
}

#[test]
fn i_type_addi_positive() {
    // addi x10, x0, 5  -> 0x00500513
    let u = dec(0x0050_0513);
    assert_eq!(u.rd, 10);
    assert_eq!(u.rs1, 0);
    assert_eq!(u.alu_op, AluOp::Add);
    assert_eq!(u.alu_src2, AluSrc2::Imm);
    assert_eq!(u.imm, 5);
}

#[test]
fn i_type_addi_negative() {
    // addi x10, x10, -1  -> 0xFFF50513 ; imm sign-extends to -1.
    let u = dec(0xFFF5_0513);
    assert_eq!(u.imm, 0xFFFF_FFFF); // -1 as a bit pattern
    assert_eq!(u.imm as i32, -1);
}

#[test]
fn s_type_sw() {
    // sw x10, 0(x11)  -> 0x00A5A023
    let u = dec(0x00A5_A023);
    assert_eq!(u.rs1, 11);
    assert_eq!(u.rs2, 10);
    assert_eq!(u.imm, 0);
    assert_eq!(
        u.mem,
        Some(MemOp {
            store: true,
            width: Width::Word,
            load_signext: false
        })
    );
    assert_eq!(u.rd_source, RdSource::Skip);
}

#[test]
fn b_type_bne_negative() {
    // bne x10, x0, -4  -> 0xFE051EE3
    let u = dec(0xFE05_1EE3);
    assert_eq!(u.rs1, 10);
    assert_eq!(u.rs2, 0);
    assert_eq!(u.branch, Some(BranchType::Neq));
    assert_eq!(u.imm as i32, -4);
    assert_eq!(u.alu_src1, AluSrc1::Pc);
}

#[test]
fn u_type_lui() {
    // lui x11, 0x10000  -> 0x100005B7
    let u = dec(0x1000_05B7);
    assert_eq!(u.rd, 11);
    assert_eq!(u.imm, 0x1000_0000); // imm[31:12] << 12
    assert_eq!(u.alu_src1, AluSrc1::Zero);
    assert_eq!(u.rd_source, RdSource::AluResult);
}

#[test]
fn j_type_jal_self() {
    // jal x0, 0  -> 0x0000006F
    let u = dec(0x0000_006F);
    assert_eq!(u.rd, 0);
    assert_eq!(u.imm, 0);
    assert_eq!(u.branch, Some(BranchType::JumpAnyway));
    assert_eq!(u.rd_source, RdSource::PcPlus4);
}

// --- paths the C test did not cover ---------------------------------------

#[test]
fn loads_set_width_and_signedness() {
    // lb  x1, 0(x2) -> 0x00010083 (signed byte)
    let lb = dec(0x0001_0083);
    assert_eq!(
        lb.mem,
        Some(MemOp {
            store: false,
            width: Width::Byte,
            load_signext: true
        })
    );
    // lbu x1, 0(x2) -> 0x00014083 (unsigned byte)
    let lbu = dec(0x0001_4083);
    assert!(!lbu.mem.unwrap().load_signext);
    assert_eq!(lbu.mem.unwrap().width, Width::Byte);
}

#[test]
fn fence_is_nop_fencei_is_illegal() {
    // fence (func3 = 0) -> 0x0FF0000F : NOP, not illegal.
    let fence = dec(0x0FF0_000F);
    assert!(!fence.illegal);
    assert_eq!(fence.rd_source, RdSource::Skip);
    // fence.i (func3 = 1) -> 0x0000100F : illegal (Zifencei, out of scope).
    let fencei = dec(0x0000_100F);
    assert!(fencei.illegal);
}

#[test]
fn system_ops_decode() {
    assert_eq!(dec(0x0000_0073).system, SystemKind::Ecall); // ecall
    assert_eq!(dec(0x0010_0073).system, SystemKind::Ebreak); // ebreak
    assert_eq!(dec(0x3020_0073).system, SystemKind::Mret); // mret
    assert_eq!(dec(0x1050_0073).system, SystemKind::Wfi); // wfi
                                                          // ecall with nonzero rs1/rd is illegal.
    assert!(dec(0x0010_8073).illegal);
}

#[test]
fn csrrw_decodes_with_address_and_write() {
    // csrrw x0, mstatus, x1 -> 0x30009073 (func3 = 1, csr = 0x300, rs1 = 1)
    let u = dec(0x3000_9073);
    match u.system {
        SystemKind::Csr { addr, writes, .. } => {
            assert_eq!(addr, 0x300);
            assert!(writes); // CSRRW always writes
        }
        other => panic!("expected Csr, got {other:?}"),
    }
}

#[test]
fn unknown_opcode_is_illegal() {
    // opcode 0x0B is not in the RV32I subset.
    assert!(dec(0x0000_000B).illegal);
    // reserved branch func3 = 2 is illegal.
    assert!(dec(0x0000_2063).illegal);
}
