//! Execute — reads architectural state, computes results, and produces a
//! [`Staging`] describing the intended commit. **No architectural state is
//! mutated here**; that is Commit's job (in [`core`](crate::core)).
//!
//! Loads still read the bus during Execute (there is no separate MEM stage in
//! the ISS), but stores only *stage* a [`StoreReq`] — the actual bus write
//! happens at Commit. Fault conditions (illegal, load bus error, misalignment)
//! short-circuit by returning `Err(HaltKind)`; the barrier in `core` never sees
//! a halting step.

use crate::arch::ArchState;
use crate::bus::{AxiResp, Bus, Width, WidthAlign};
use crate::decode::{AluSrc1, AluSrc2, CsrSource, MemOp, RdSource, SystemKind, Uop};
use crate::packet::HaltKind;

/// The retirement/trap action Execute requests of the barrier.
///
/// [`Action::SyncTrap`] is a *request*, not a commit action — the barrier
/// consumes it and routes to trap entry; Commit only ever sees
/// `Normal`/`Zicsr`/`Mret`.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Action {
    /// Plain retirement: optional GPR write + `PC ← new_pc`.
    Normal,
    /// Zicsr: GPR write of the old value + optional CSR write of the new value.
    Zicsr {
        /// CSR address to write.
        addr: u16,
        /// The new CSR value.
        write_value: u32,
        /// Whether the CSR is actually written.
        writes: bool,
    },
    /// `MRET`: restore `mstatus` from MPIE, `PC ← mepc`. No GPR write.
    Mret,
    /// Synchronous trap requested by Execute (`ECALL`). Consumed by the barrier.
    SyncTrap {
        /// Trap cause (11 for `ECALL`).
        cause: u32,
        /// Trap value (0 under the simplified model).
        tval: u32,
    },
}

/// A staged store — the 1-entry store buffer, drained to the bus at Commit.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct StoreReq {
    /// Target address.
    pub addr: u32,
    /// Data to write.
    pub data: u32,
    /// Byte-enable strobe.
    pub strb: u8,
    /// Access width.
    pub width: Width,
}

/// Everything Commit needs to retire the instruction (or that the barrier needs
/// to route it to a trap).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Staging {
    /// The commit/trap action.
    pub action: Action,
    /// PC target for `Normal`/`Zicsr`/`Mret` (unused for `SyncTrap`).
    pub new_pc: u32,
    /// GPR write target (`0` ⇒ no write).
    pub rd: u8,
    /// GPR write value.
    pub rd_value: u32,
    /// A staged store, if any.
    pub store: Option<StoreReq>,
}

impl Staging {
    /// The default staging for the instruction at `pc`: plain retirement,
    /// `new_pc = pc + 4`, no GPR write, no store.
    fn normal(pc: u32) -> Self {
        Staging {
            action: Action::Normal,
            new_pc: pc.wrapping_add(4),
            rd: 0,
            rd_value: 0,
            store: None,
        }
    }
}

/// Execute one decoded uop against `state` (read-only) using `bus` for loads.
///
/// Returns the [`Staging`] for Commit, or `Err(HaltKind)` for a fault that
/// halts the simulation during Execute.
pub fn execute(state: &ArchState, uop: &Uop, bus: &mut impl Bus) -> Result<Staging, HaltKind> {
    if uop.illegal {
        return Err(HaltKind::Illegal);
    }
    if uop.system != SystemKind::None {
        return execute_system(state, uop);
    }

    let mut stg = Staging::normal(uop.pc);

    // --- ALU --------------------------------------------------------------
    let op1 = match uop.alu_src1 {
        AluSrc1::Rs1 => uop.rs1_val,
        AluSrc1::Pc => uop.pc,
        AluSrc1::Zero => 0,
    };
    let op2 = match uop.alu_src2 {
        AluSrc2::Rs2 => uop.rs2_val,
        AluSrc2::Imm => uop.imm,
    };
    let alu_result = uop.alu_op.apply(op1, op2);

    // --- Memory (load reads now; store stages for Commit) -----------------
    let mut mem_load = 0u32;
    if let Some(mem) = uop.mem {
        mem_load = execute_mem(uop, mem, alu_result, &mut stg, bus)?;
    }

    // --- Branch / jump ----------------------------------------------------
    if let Some(bt) = uop.branch {
        let mut target = alu_result;
        if bt == crate::alu::BranchType::JumpAnyway {
            target &= !1; // clear bit 0 (JALR semantics; harmless for JAL)
        }
        if bt.taken(uop.rs1_val, uop.rs2_val) {
            if !Width::Word.is_aligned(target) {
                return Err(HaltKind::MisalignPc);
            }
            stg.new_pc = target;
        }
    }

    // --- Stage the rd write ----------------------------------------------
    let rd_value = match uop.rd_source {
        RdSource::PcPlus4 => uop.pc.wrapping_add(4),
        RdSource::AluResult => alu_result,
        RdSource::MemLoad => mem_load,
        RdSource::Skip => 0,
    };
    if uop.rd_source != RdSource::Skip && uop.rd != 0 {
        stg.rd = uop.rd;
        stg.rd_value = rd_value;
    }

    Ok(stg)
}

/// Resolve a memory op: check alignment, then either stage a store or read a
/// load. Returns the (sign-extended) load value; 0 for stores.
fn execute_mem(
    uop: &Uop,
    mem: MemOp,
    addr: u32,
    stg: &mut Staging,
    bus: &mut impl Bus,
) -> Result<u32, HaltKind> {
    if !mem.width.is_aligned(addr) {
        return Err(if mem.store {
            HaltKind::MisalignSt
        } else {
            HaltKind::MisalignLd
        });
    }

    if mem.store {
        stg.store = Some(StoreReq {
            addr,
            data: uop.rs2_val,
            strb: mem.width.full_strb(),
            width: mem.width,
        });
        Ok(0)
    } else {
        let raw = bus
            .read(addr, mem.width)
            .map_err(|_: AxiResp| HaltKind::BusErrorLd)?;
        Ok(sign_extend_load(raw, mem))
    }
}

/// Sign- or zero-extend a freshly loaded value per the load's width/signedness.
fn sign_extend_load(raw: u32, mem: MemOp) -> u32 {
    if mem.load_signext && mem.width != Width::Word {
        match mem.width {
            Width::Byte => ((raw & 0xFF) as i8) as i32 as u32,
            Width::Half => ((raw & 0xFFFF) as i16) as i32 as u32,
            Width::Word => raw,
        }
    } else {
        raw
    }
}

/// Execute a SYSTEM-class instruction — stages `SyncTrap`/`Mret`/`Zicsr`/NOP, or
/// halts on `EBREAK` / an illegal CSR access.
fn execute_system(state: &ArchState, uop: &Uop) -> Result<Staging, HaltKind> {
    let mut stg = Staging::normal(uop.pc);
    match uop.system {
        SystemKind::Ecall => {
            stg.action = Action::SyncTrap { cause: 11, tval: 0 };
        }
        SystemKind::Ebreak => {
            // Voluntary halt; exit = a0[7:0] (x10).
            return Err(HaltKind::Voluntary {
                a0: (state.read_gpr(10) & 0xFF) as u8,
            });
        }
        SystemKind::Mret => {
            stg.action = Action::Mret;
            stg.new_pc = state.csrs.mepc();
        }
        SystemKind::Wfi => {
            // NOP — default staging is correct.
        }
        SystemKind::Csr {
            op,
            addr,
            source,
            writes,
        } => {
            let old = state.csrs.read(addr).ok_or(HaltKind::Illegal)?; // unrecognised CSR
            if writes && !crate::csr::CsrFile::addr_writable(addr) {
                return Err(HaltKind::Illegal); // write to read-only CSR
            }
            let src = source_value(source);
            let new_val = match op {
                crate::decode::CsrOp::Rw => src,
                crate::decode::CsrOp::Rs => old | src,
                crate::decode::CsrOp::Rc => old & !src,
            };
            stg.action = Action::Zicsr {
                addr,
                write_value: new_val,
                writes,
            };
            stg.rd = uop.rd;
            stg.rd_value = old;
        }
        SystemKind::None => unreachable!("execute_system called with SystemKind::None"),
    }
    Ok(stg)
}

#[inline]
fn source_value(source: CsrSource) -> u32 {
    source.value()
}
