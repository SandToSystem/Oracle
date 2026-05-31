//! The decoded micro-op [`Uop`] and its control-signal enums — the intermediate
//! form passed from Decode to Execute.
//!
//! Where the C `uop_t` used parallel boolean flags (`is_mem`, `is_branch`,
//! `csr_source_is_imm`), this port folds them into `Option`s and data-carrying
//! enums so illegal combinations are unrepresentable.

use crate::alu::{AluOp, BranchType};
use crate::bus::Width;

/// First ALU operand select.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum AluSrc1 {
    /// `rs1` value
    Rs1,
    /// the instruction's PC (for `AUIPC`, branches, `JAL`)
    Pc,
    /// constant 0 (for `LUI`)
    Zero,
}

/// Second ALU operand select.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum AluSrc2 {
    /// `rs2` value
    Rs2,
    /// the sign-extended immediate
    Imm,
}

/// Where the value written to `rd` comes from.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum RdSource {
    /// `pc + 4` (`JAL` / `JALR` link)
    PcPlus4,
    /// the ALU result
    AluResult,
    /// the value loaded from memory
    MemLoad,
    /// no `rd` write
    Skip,
}

/// A memory access — replaces the C `is_mem` / `is_store` / `load_signext` /
/// `mem_length` quartet. Present (`Some`) only for `LOAD`/`STORE`.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct MemOp {
    /// `true` for a store, `false` for a load.
    pub store: bool,
    /// Access width (1 / 2 / 4 bytes).
    pub width: Width,
    /// Sign-extend the loaded value (`LB`/`LH`, not `LBU`/`LHU`/`LW`). Ignored
    /// for stores.
    pub load_signext: bool,
}

/// Atomic Zicsr operation kind, re-exported shape of [`crate::csr::CsrOp`] for
/// the decoded form.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum CsrOp {
    /// `CSRRW` / `CSRRWI`
    Rw,
    /// `CSRRS` / `CSRRSI`
    Rs,
    /// `CSRRC` / `CSRRCI`
    Rc,
}

impl From<CsrOp> for crate::csr::CsrOp {
    fn from(op: CsrOp) -> Self {
        match op {
            CsrOp::Rw => crate::csr::CsrOp::Rw,
            CsrOp::Rs => crate::csr::CsrOp::Rs,
            CsrOp::Rc => crate::csr::CsrOp::Rc,
        }
    }
}

/// The source operand of a Zicsr op — folds the C `csr_source_is_imm` flag into
/// the type so the two cases can't be confused.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum CsrSource {
    /// `rs1` register value (`CSRRW`/`CSRRS`/`CSRRC`).
    Reg(u32),
    /// Zero-extended 5-bit immediate (`CSRRWI`/`CSRRSI`/`CSRRCI`).
    Imm(u32),
}

impl CsrSource {
    /// The numeric source value, regardless of register/immediate origin.
    pub fn value(self) -> u32 {
        match self {
            CsrSource::Reg(v) | CsrSource::Imm(v) => v,
        }
    }
}

/// SYSTEM-family discriminator. [`SystemKind::None`] means the instruction is
/// not a SYSTEM op and the rest of the [`Uop`] applies normally.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum SystemKind {
    /// Not a SYSTEM instruction.
    None,
    /// `ECALL` — synchronous trap (cause 11).
    Ecall,
    /// `EBREAK` — voluntary halt (exit = `a0 & 0xFF`).
    Ebreak,
    /// `MRET` — trap return.
    Mret,
    /// `WFI` — implemented as a NOP.
    Wfi,
    /// A Zicsr read-modify-write.
    Csr {
        /// The atomic operation.
        op: CsrOp,
        /// 12-bit CSR address.
        addr: u16,
        /// The source operand (register value or imm5).
        source: CsrSource,
        /// Whether the instruction writes the CSR (spec §9.1).
        writes: bool,
    },
}

/// A decoded micro-op: everything Execute needs, with no architectural mutation
/// performed during decode.
///
/// Built by [`crate::decode::decode`]. Defaults to an illegal NOP shell so the
/// decoder only sets the fields each format actually uses.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct Uop {
    /// PC of this instruction.
    pub pc: u32,

    /// Register indices (0 means "unused / x0").
    pub rs1: u8,
    /// See [`rs1`](Self::rs1).
    pub rs2: u8,
    /// Destination register index (0 ⇒ no write).
    pub rd: u8,
    /// Captured `rs1` value (read at decode time).
    pub rs1_val: u32,
    /// Captured `rs2` value (read at decode time).
    pub rs2_val: u32,

    /// Sign-extended immediate (as a bit pattern).
    pub imm: u32,

    /// First ALU operand select.
    pub alu_src1: AluSrc1,
    /// Second ALU operand select.
    pub alu_src2: AluSrc2,
    /// ALU operation.
    pub alu_op: AluOp,

    /// `Some` for `LOAD`/`STORE`.
    pub mem: Option<MemOp>,
    /// `Some` for branches and jumps.
    pub branch: Option<BranchType>,
    /// Where `rd`'s value comes from.
    pub rd_source: RdSource,
    /// SYSTEM / Zicsr discriminator.
    pub system: SystemKind,

    /// The decoder rejected this instruction; Execute turns it into an illegal
    /// halt.
    pub illegal: bool,
}

impl Uop {
    /// A blank uop for `pc`: illegal NOP shell. The decoder overrides fields.
    pub(crate) fn blank(pc: u32) -> Self {
        Uop {
            pc,
            rs1: 0,
            rs2: 0,
            rd: 0,
            rs1_val: 0,
            rs2_val: 0,
            imm: 0,
            alu_src1: AluSrc1::Rs1,
            alu_src2: AluSrc2::Rs2,
            alu_op: AluOp::Add,
            mem: None,
            branch: None,
            rd_source: RdSource::Skip,
            system: SystemKind::None,
            illegal: false,
        }
    }
}
