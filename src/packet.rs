//! [`CommitPacket`] — the architectural footprint of one [`Core::step`].
//!
//! The verification protocol: every step (including halt events) emits one
//! packet so a cross-verifier can diff the architectural effect before either
//! side exits. Where the C `commit_packet_t` used parallel
//! `halt_observed`/`trap_taken`/`store_valid` booleans, this port uses
//! `Option`s so an absent effect is simply `None`.
//!
//! [`Core::step`]: crate::Core::step

use crate::csr::CsrFile;

/// Why a step terminated the simulation.
///
/// Each variant maps to a process exit code (the C `halt_kind_exit_code`); the
/// involuntary kinds use the fixed 130–135 range, while a voluntary `EBREAK`
/// carries `a0 & 0xFF`.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum HaltKind {
    /// `EBREAK`; exit code = `a0`.
    Voluntary {
        /// The low byte of `a0` at the retiring `EBREAK`.
        a0: u8,
    },
    /// Decoder reject or CSR access violation; exit 130.
    Illegal,
    /// Instruction-fetch bus error (SLVERR/DECERR); exit 131.
    BusErrorIf,
    /// Load bus error; exit 132.
    BusErrorLd,
    /// Store bus error; exit 133.
    BusErrorSt,
    /// `JALR`/branch target misaligned; exit 134.
    MisalignPc,
    /// Load address misaligned; exit 134.
    MisalignLd,
    /// Store address misaligned; exit 134.
    MisalignSt,
    /// Async interrupt during the ECALL handler prologue; exit 135.
    DoubleTrap,
}

impl HaltKind {
    /// The process exit code this halt maps to (matches C `halt_kind_exit_code`).
    pub fn exit_code(self) -> u8 {
        match self {
            HaltKind::Voluntary { a0 } => a0,
            HaltKind::Illegal => 130,
            HaltKind::BusErrorIf => 131,
            HaltKind::BusErrorLd => 132,
            HaltKind::BusErrorSt => 133,
            HaltKind::MisalignPc | HaltKind::MisalignLd | HaltKind::MisalignSt => 134,
            HaltKind::DoubleTrap => 135,
        }
    }

    /// A short diagnostic tag (mirrors the C `[FATAL]` log labels).
    pub fn tag(self) -> &'static str {
        match self {
            HaltKind::Voluntary { .. } => "EBREAK voluntary",
            HaltKind::Illegal => "illegal instruction",
            HaltKind::BusErrorIf => "IF bus error",
            HaltKind::BusErrorLd => "LD bus error",
            HaltKind::BusErrorSt => "ST bus error",
            HaltKind::MisalignPc => "misaligned PC",
            HaltKind::MisalignLd => "misaligned load",
            HaltKind::MisalignSt => "misaligned store",
            HaltKind::DoubleTrap => "double trap",
        }
    }
}

/// A halt event: the kind plus its resolved exit code.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct HaltEvent {
    /// The halt classification.
    pub kind: HaltKind,
    /// The process exit code (`kind.exit_code()`).
    pub exit_code: u8,
}

impl HaltEvent {
    /// Build the event for `kind`, computing its exit code.
    pub fn new(kind: HaltKind) -> Self {
        HaltEvent {
            kind,
            exit_code: kind.exit_code(),
        }
    }
}

/// A synchronous trap entry (ECALL, or an async interrupt under the simplified
/// model). Carries the CSR values the trap-entry microsequence wrote.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct TrapEvent {
    /// `mcause` (interrupt bit set for async).
    pub cause: u32,
    /// `mepc` (the un-retired instruction's PC).
    pub mepc: u32,
    /// `mtval` (0 under the simplified model).
    pub mtval: u32,
}

/// An architectural store effect (the C `store_*` triple).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct StoreObserved {
    /// Target address.
    pub addr: u32,
    /// Data written (low `width` bytes meaningful).
    pub data: u32,
    /// Byte-enable strobe.
    pub strb: u8,
}

/// Post-step snapshot of the 7 storage CSRs (the C inline-snapshot, so a diff
/// never races a separate accessor).
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub struct CsrSnapshot {
    /// `mstatus` (with MPP synthesised to `0b11`).
    pub mstatus: u32,
    /// `mtvec`.
    pub mtvec: u32,
    /// `mepc`.
    pub mepc: u32,
    /// `mcause`.
    pub mcause: u32,
    /// `mtval`.
    pub mtval: u32,
    /// `mscratch`.
    pub mscratch: u32,
    /// `mie`.
    pub mie: u32,
}

impl CsrSnapshot {
    /// Capture the current CSR storage.
    pub fn capture(csrs: &CsrFile) -> Self {
        CsrSnapshot {
            mstatus: csrs.mstatus_snapshot(),
            mtvec: csrs.mtvec_raw(),
            mepc: csrs.mepc(),
            mcause: csrs.mcause_raw(),
            mtval: csrs.mtval_raw(),
            mscratch: csrs.mscratch_raw(),
            mie: csrs.mie_raw(),
        }
    }
}

/// The architectural footprint of one step. A field's `Option` is `None` when
/// that effect did not occur.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
pub struct CommitPacket {
    /// PC of the instruction this step processed.
    pub pc: u32,
    /// GPR write target (`0` ⇒ no architectural write happened).
    pub rd: u8,
    /// The value written to `rd` (meaningful only when `rd != 0`).
    pub rd_value: u32,
    /// A store effect, if one retired this step.
    pub store: Option<StoreObserved>,
    /// A halt event, if this step terminated the simulation. The caller **must**
    /// stop stepping when this is `Some`.
    pub halt: Option<HaltEvent>,
    /// A trap entry, if this step took a synchronous/async trap.
    pub trap: Option<TrapEvent>,
    /// Post-step CSR snapshot.
    pub csr_snapshot: CsrSnapshot,
}

impl CommitPacket {
    /// A pending packet for the instruction at `pc` — no effects recorded yet.
    pub(crate) fn pending(pc: u32) -> Self {
        CommitPacket {
            pc,
            ..Default::default()
        }
    }
}
