//! [`ArchState`] — the full programmer-visible architectural snapshot.
//!
//! Per the RISC-V spec the CSR file is part of architectural state (trap entry
//! and `MRET` mutate it just like ALU ops mutate the GPRs), so it lives here
//! rather than as a separate Core member — [`Core::state`](crate::Core::state) /
//! [`set_state`](crate::Core::set_state) therefore capture and restore it
//! automatically. Direct port of `arch.h`.

use crate::csr::CsrFile;

/// PC + the 32 integer GPRs + the M-mode CSR file.
///
/// `x0` is architecturally hardwired to zero. The array stores it like any
/// other register; the invariant is enforced at the single GPR-write site in
/// the Core (writes with `rd == 0` are dropped) and re-asserted by
/// [`write_gpr`](Self::write_gpr).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ArchState {
    /// Program counter.
    pub pc: u32,
    /// `x0`–`x31`. `gpr[0]` is always 0.
    pub gpr: [u32; 32],
    /// M-mode CSR file (7 storage CSRs + wire inputs).
    pub csrs: CsrFile,
}

impl ArchState {
    /// Reset state: `pc = 0`, all GPRs 0, CSR file in reset.
    pub fn new() -> Self {
        Self::default()
    }

    /// Read `gpr[index]`. `index` must be `< 32`.
    pub fn read_gpr(&self, index: u8) -> u32 {
        self.gpr[index as usize]
    }

    /// Write `value` to `gpr[index]`, honouring the `x0`-hardwired-zero rule:
    /// a write to `index == 0` is a no-op. This is the *only* place GPR writes
    /// should flow through so the invariant cannot drift.
    pub fn write_gpr(&mut self, index: u8, value: u32) {
        if index != 0 {
            self.gpr[index as usize] = value;
        }
    }
}
