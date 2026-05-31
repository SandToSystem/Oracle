//! End-to-end Core tests: small programs assembled into a [`RamBus`] and run
//! via [`Core::run_until_halt`], asserting on the emitted [`CommitPacket`]
//! stream. Locks down the trap/barrier/halt semantics ported from the C Core.

use iss_core::csr::addr as csr;
use iss_core::{CommitPacket, Core, HaltKind, RamBus};

// --- tiny RV32I assembler --------------------------------------------------

fn i_type(op: u32, f3: u32, rd: u32, rs1: u32, imm: i32) -> u32 {
    ((imm as u32 & 0xFFF) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op
}
fn s_type(op: u32, f3: u32, rs1: u32, rs2: u32, imm: i32) -> u32 {
    let imm = imm as u32 & 0xFFF;
    ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | ((imm & 0x1F) << 7) | op
}
fn r_type(op: u32, f3: u32, f7: u32, rd: u32, rs1: u32, rs2: u32) -> u32 {
    (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | op
}

fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(0x13, 0, rd, rs1, imm)
}
fn add(rd: u32, rs1: u32, rs2: u32) -> u32 {
    r_type(0x33, 0, 0, rd, rs1, rs2)
}
fn lw(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(0x03, 2, rd, rs1, imm)
}
/// `sw rs2, imm(rs1)`
fn sw(rs1: u32, rs2: u32, imm: i32) -> u32 {
    s_type(0x23, 2, rs1, rs2, imm)
}
fn jalr(rd: u32, rs1: u32, imm: i32) -> u32 {
    i_type(0x67, 0, rd, rs1, imm)
}
fn csrrw(rd: u32, csr_addr: u16, rs1: u32) -> u32 {
    i_type(0x73, 1, rd, rs1, csr_addr as i32)
}
const ECALL: u32 = 0x0000_0073;
const EBREAK: u32 = 0x0010_0073;
const MRET: u32 = 0x3020_0073;

/// Build a Core over a 4 KiB RAM preloaded with `program` at address 0.
fn core_with(program: &[u32]) -> Core<RamBus> {
    let mut bus = RamBus::new(0, 0x1000);
    bus.load_program(0, program);
    Core::new(bus)
}

/// Run to halt, collecting every packet.
fn run(core: &mut Core<RamBus>) -> (Vec<CommitPacket>, HaltKind) {
    let mut log = Vec::new();
    let halt = core
        .run_until_halt_max(1000, |p| log.push(*p))
        .expect("program should halt within 1000 steps");
    (log, halt.kind)
}

// --- tests -----------------------------------------------------------------

#[test]
fn ebreak_voluntary_exit_is_a0_low_byte() {
    let mut core = core_with(&[EBREAK]);
    core.state_mut().gpr[10] = 0x1234_002A; // a0 = x10; only low byte matters
    let halt = core.run_until_halt(|_| {});
    assert_eq!(halt.kind, HaltKind::Voluntary { a0: 0x2A });
    assert_eq!(halt.exit_code, 42);
}

#[test]
fn addi_then_ebreak_writes_register_and_advances_pc() {
    let mut core = core_with(&[addi(1, 0, 5), EBREAK]);
    let (log, kind) = run(&mut core);
    // First packet: x1 <- 5 at pc 0.
    assert_eq!(log[0].pc, 0);
    assert_eq!(log[0].rd, 1);
    assert_eq!(log[0].rd_value, 5);
    assert_eq!(core.state().gpr[1], 5);
    // Second packet halts at pc 4.
    assert_eq!(log[1].pc, 4);
    assert_eq!(kind, HaltKind::Voluntary { a0: 0 });
}

#[test]
fn store_then_load_roundtrips_through_bus() {
    // x1 = 0xABCD; sw x1, 0x40(x0); lw x2, 0x40(x0); ebreak
    let mut core = core_with(&[addi(1, 0, 0x5BC), sw(0, 1, 0x40), lw(2, 0, 0x40), EBREAK]);
    let (log, _) = run(&mut core);
    // The store packet records the architectural triple.
    let store = log[1].store.expect("step 1 should store");
    assert_eq!(store.addr, 0x40);
    assert_eq!(store.data, 0x5BC);
    assert_eq!(store.strb, 0xF);
    assert_eq!(core.state().gpr[2], 0x5BC);
}

#[test]
fn ecall_enters_trap_to_mtvec() {
    let mut core = core_with(&[ECALL, /* pad */ 0]);
    core.state_mut().csrs.write(csr::MTVEC, 0x80);
    core.bus_mut().load_word(0x80, EBREAK); // handler halts cleanly
    let (log, _) = run(&mut core);

    let trap = log[0].trap.expect("ecall should trap");
    assert_eq!(trap.cause, 11); // M-mode environment call
    assert_eq!(trap.mepc, 0); // PC of the un-retired ECALL
    assert_eq!(core.state().pc, 0x80); // handler entry (but already halted)
                                       // Snapshot reflects the trap-entry CSR writes.
    assert_eq!(log[0].csr_snapshot.mcause, 11);
    assert_eq!(log[0].csr_snapshot.mepc, 0);
}

#[test]
fn mret_restores_mie_and_returns_to_mepc() {
    let mut core = core_with(&[MRET]);
    core.state_mut().csrs.write(csr::MEPC, 0x10);
    core.state_mut().csrs.write(csr::MSTATUS, 0x80); // MPIE = 1, MIE = 0
    core.bus_mut().load_word(0x10, EBREAK);
    let (_log, kind) = run(&mut core);
    assert_eq!(kind, HaltKind::Voluntary { a0: 0 });
    // After MRET: MIE restored from MPIE.
    assert!(core.state().csrs.mie_enabled());
}

#[test]
fn async_interrupt_beats_staged_ecall_with_priority_mei() {
    let mut core = core_with(&[ECALL]);
    core.state_mut().csrs.write(csr::MTVEC, 0x80);
    core.state_mut().csrs.write(csr::MSTATUS, 0x8); // MIE = 1
    core.state_mut().csrs.write(csr::MIE, 0x888); // MEIE | MTIE | MSIE
    core.csrs_mut().set_wires(true, true, true); // all three pending
    core.bus_mut().load_word(0x80, EBREAK);
    let (log, _) = run(&mut core);

    let trap = log[0].trap.expect("interrupt should trap");
    // MEI wins (cause 11) with the interrupt bit set; it preempts the ECALL.
    assert_eq!(trap.cause, 0x8000_000B);
}

#[test]
fn interrupt_priority_msi_over_mti() {
    let mut core = core_with(&[addi(1, 0, 1)]); // any non-trapping instruction
    core.state_mut().csrs.write(csr::MTVEC, 0x80);
    core.state_mut().csrs.write(csr::MSTATUS, 0x8);
    core.state_mut().csrs.write(csr::MIE, 0x888);
    core.csrs_mut().set_wires(true, true, false); // MSI + MTI, no MEI
    core.bus_mut().load_word(0x80, EBREAK);
    let (log, _) = run(&mut core);
    assert_eq!(log[0].trap.unwrap().cause, 0x8000_0003); // MSI (3) > MTI (7)
}

#[test]
fn double_trap_on_handler_prologue_halts_135() {
    // ECALL at 0 traps to 0x80; the handler's first instruction is ECALL again.
    let mut core = core_with(&[ECALL]);
    core.state_mut().csrs.write(csr::MTVEC, 0x80);
    core.bus_mut().load_word(0x80, ECALL);
    let (log, kind) = run(&mut core);
    assert!(log[0].trap.is_some()); // first ECALL trapped
    assert_eq!(kind, HaltKind::DoubleTrap);
    assert_eq!(log[1].halt.unwrap().exit_code, 135);
}

#[test]
fn load_bus_error_halts_132_during_execute() {
    let mut core = core_with(&[lw(1, 2, 0)]);
    core.state_mut().gpr[2] = 0x9000; // outside the 4 KiB RAM
    let halt = core.run_until_halt(|_| {});
    assert_eq!(halt.kind, HaltKind::BusErrorLd);
    assert_eq!(halt.exit_code, 132);
}

#[test]
fn store_bus_error_halts_133_and_pins_pc() {
    let mut core = core_with(&[sw(2, 1, 0)]);
    core.state_mut().gpr[2] = 0x9000; // store target outside RAM
    let halt = core.run_until_halt(|_| {});
    assert_eq!(halt.kind, HaltKind::BusErrorSt);
    assert_eq!(halt.exit_code, 133);
    // PC update is skipped on a store-drain fault.
    assert_eq!(core.state().pc, 0);
}

#[test]
fn misaligned_fetch_load_store_all_halt_134() {
    // Misaligned PC (fetch).
    let mut c1 = core_with(&[add(0, 0, 0)]);
    c1.state_mut().pc = 0x2;
    assert_eq!(c1.run_until_halt(|_| {}).kind, HaltKind::MisalignPc);

    // Misaligned load: lw x1, 1(x0).
    let mut c2 = core_with(&[lw(1, 0, 1)]);
    assert_eq!(c2.run_until_halt(|_| {}).kind, HaltKind::MisalignLd);

    // Misaligned store: sw x1, 2(x0).
    let mut c3 = core_with(&[sw(0, 1, 2)]);
    assert_eq!(c3.run_until_halt(|_| {}).kind, HaltKind::MisalignSt);

    // Misaligned jump target via JALR: jalr x0, 2(x0) → target 2.
    let mut c4 = core_with(&[jalr(0, 0, 2)]);
    assert_eq!(c4.run_until_halt(|_| {}).kind, HaltKind::MisalignPc);
}

#[test]
fn csrrw_reads_old_writes_new() {
    // mscratch starts 0; csrrw x5, mscratch, x1 with x1 = 0xDEAD.
    let mut core = core_with(&[addi(1, 0, 0x6AD), csrrw(5, csr::MSCRATCH, 1), EBREAK]);
    let (log, _) = run(&mut core);
    // csrrw packet: x5 <- old mscratch (0).
    assert_eq!(log[1].rd, 5);
    assert_eq!(log[1].rd_value, 0);
    // New mscratch value is the x1 contents.
    assert_eq!(core.state().csrs.read(csr::MSCRATCH), Some(0x6AD));
}

#[test]
fn x0_writes_are_dropped() {
    // addi x0, x0, 5 must not change x0; packet reports no write.
    let mut core = core_with(&[addi(0, 0, 5), EBREAK]);
    let (log, _) = run(&mut core);
    assert_eq!(log[0].rd, 0);
    assert_eq!(core.state().gpr[0], 0);
}

#[test]
fn unmapped_fetch_halts_131() {
    // Empty program region: fetch at 0 reads 0x00000000 which decodes to an
    // illegal instruction; instead point PC outside RAM to force an IF bus error.
    let mut core = core_with(&[EBREAK]);
    core.state_mut().pc = 0x8000; // outside the 4 KiB RAM
    let halt = core.run_until_halt(|_| {});
    assert_eq!(halt.kind, HaltKind::BusErrorIf);
    assert_eq!(halt.exit_code, 131);
}
