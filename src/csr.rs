//! [`CsrFile`] — the RV32I_Zicsr Machine-mode CSR file.
//!
//! Holds the 7 storage-bearing CSRs; the four read-only-zero information CSRs
//! and `misa` are computed inline, and `mip` is wire-only — the file mirrors
//! three external wires (`msip` from the CLINT, `mtip` from the CLINT, `meip`
//! from the IRQ aggregator) and synthesises the `mip` value on read. The
//! current privilege mode is structurally always M, so the privilege check is
//! omitted. Direct Rust port of `csr_file.{h,c}`.

// --- mstatus bit positions -------------------------------------------------
const MSTATUS_MIE: u32 = 1 << 3;
const MSTATUS_MPIE: u32 = 1 << 7;
/// MPP is always `0b11` (M-mode) — synthesised on read, never stored.
const MSTATUS_MPP_M: u32 = 0x3 << 11;
/// Bits software may modify in `mstatus` (MIE + MPIE; MPP stays 11, rest WPRI).
const MASK_MSTATUS: u32 = MSTATUS_MIE | MSTATUS_MPIE;

// --- mie / mip bit positions ----------------------------------------------
const MIP_MSIP_BIT: u32 = 3;
const MIP_MTIP_BIT: u32 = 7;
const MIP_MEIP_BIT: u32 = 11;
const MASK_MIE: u32 = (1 << MIP_MSIP_BIT) | (1 << MIP_MTIP_BIT) | (1 << MIP_MEIP_BIT);

/// `BASE[31:2]`; MODE forced to `0b00` (Direct). Also the `mepc` low-2-clear mask.
const MASK_MTVEC: u32 = 0xFFFF_FFFC;
const MASK_MEPC: u32 = 0xFFFF_FFFC;

/// `misa`: MXL = `01` (RV32) at bits [31:30], extension bit `I` (bit 8) set.
const MISA_VALUE: u32 = 0x4000_0100;

/// The 13 recognised CSR addresses.
pub mod addr {
    pub const MSTATUS: u16 = 0x300;
    pub const MISA: u16 = 0x301;
    pub const MIE: u16 = 0x304;
    pub const MTVEC: u16 = 0x305;
    pub const MSCRATCH: u16 = 0x340;
    pub const MEPC: u16 = 0x341;
    pub const MCAUSE: u16 = 0x342;
    pub const MTVAL: u16 = 0x343;
    pub const MIP: u16 = 0x344;
    pub const MVENDORID: u16 = 0xF11;
    pub const MARCHID: u16 = 0xF12;
    pub const MIMPID: u16 = 0xF13;
    pub const MHARTID: u16 = 0xF14;
}

/// Atomic Zicsr operation kind (`CSRRW` / `CSRRS` / `CSRRC` and their `*I`
/// variants).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum CsrOp {
    /// `CSRRW` / `CSRRWI` — replace.
    Rw,
    /// `CSRRS` / `CSRRSI` — set bits.
    Rs,
    /// `CSRRC` / `CSRRCI` — clear bits.
    Rc,
}

/// The M-mode CSR file: 7 storage CSRs plus the three interrupt-pending wires.
///
/// The wires are *not* architectural in the spec sense (they are external
/// inputs sampled when reading `mip`) but are bundled here to keep the access
/// primitives in one place; the standalone runner re-drives them every step.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CsrFile {
    /// Only MIE (bit 3) and MPIE (bit 7) are stored; MPP is fixed at `0b11`.
    mstatus: u32,
    /// Only MSIE (3), MTIE (7), MEIE (11) are writable.
    mie: u32,
    /// `BASE[31:2]` only; MODE forced to `0b00` (Direct).
    mtvec: u32,
    /// Full 32-bit R/W scratch register.
    mscratch: u32,
    /// `[31:2]` only; `[1:0]` forced to 0 (IALIGN = 32).
    mepc: u32,
    /// Full 32-bit (interrupt bit + cause code).
    mcause: u32,
    /// Full 32-bit; always written 0 under the simplified trap model.
    mtval: u32,

    msip_wire: bool,
    mtip_wire: bool,
    meip_wire: bool,
}

impl CsrFile {
    /// A CSR file in reset state (all storage zero, wires low).
    pub fn new() -> Self {
        Self::default()
    }

    // --- wire inputs -------------------------------------------------------

    /// Drive the three external interrupt-pending wires (`mip` bits 3/7/11).
    pub fn set_wires(&mut self, msip: bool, mtip: bool, meip: bool) {
        self.msip_wire = msip;
        self.mtip_wire = mtip;
        self.meip_wire = meip;
    }

    /// The synthesised `mip` value (only the three wire bits can be set).
    pub fn mip(&self) -> u32 {
        let mut v = 0;
        if self.msip_wire {
            v |= 1 << MIP_MSIP_BIT;
        }
        if self.mtip_wire {
            v |= 1 << MIP_MTIP_BIT;
        }
        if self.meip_wire {
            v |= 1 << MIP_MEIP_BIT;
        }
        v
    }

    /// `mie` (already masked to the three enable bits).
    pub fn mie(&self) -> u32 {
        self.mie & MASK_MIE
    }

    /// `true` when global interrupts are enabled (`mstatus.MIE`).
    pub fn mie_enabled(&self) -> bool {
        self.mstatus & MSTATUS_MIE != 0
    }

    /// The `BASE` field of `mtvec` (bits `[31:2]`, low 2 bits cleared).
    pub fn mtvec_base(&self) -> u32 {
        self.mtvec & MASK_MTVEC
    }

    /// `mepc` (masked to the architectural `[31:2]`).
    pub fn mepc(&self) -> u32 {
        self.mepc & MASK_MEPC
    }

    // --- address predicates ------------------------------------------------

    /// `true` if `addr` is one of the 13 CSRs this file recognises.
    pub fn addr_recognised(addr: u16) -> bool {
        use addr::*;
        matches!(
            addr,
            MSTATUS
                | MISA
                | MIE
                | MTVEC
                | MSCRATCH
                | MEPC
                | MCAUSE
                | MTVAL
                | MIP
                | MVENDORID
                | MARCHID
                | MIMPID
                | MHARTID
        )
    }

    /// `true` if a write to `addr` would *not* raise an illegal-instruction
    /// violation. Per spec §9.1, `addr[11:10] == 0b11` marks a read-only CSR.
    /// `misa`/`mip` writes are silently ignored but the address is still
    /// "writable" for this predicate.
    pub fn addr_writable(addr: u16) -> bool {
        Self::addr_recognised(addr) && (addr >> 10) & 0x3 != 0x3
    }

    // --- plain read / write ------------------------------------------------

    /// Side-effect-free read of the synthesised CSR view. `None` on an
    /// unrecognised address.
    pub fn read(&self, addr: u16) -> Option<u32> {
        use addr::*;
        Some(match addr {
            MSTATUS => (self.mstatus & MASK_MSTATUS) | MSTATUS_MPP_M,
            MISA => MISA_VALUE,
            MIE => self.mie & MASK_MIE,
            MTVEC => self.mtvec & MASK_MTVEC,
            MSCRATCH => self.mscratch,
            MEPC => self.mepc & MASK_MEPC,
            MCAUSE => self.mcause,
            MTVAL => self.mtval,
            MIP => self.mip(),
            MVENDORID | MARCHID | MIMPID | MHARTID => 0,
            _ => return None,
        })
    }

    /// Apply the per-CSR write mask. Returns `false` if the address is
    /// unrecognised or architecturally read-only (the caller must not have
    /// staged the write). `misa`/`mip` writes are silently ignored (`true`,
    /// no mutation).
    pub fn write(&mut self, addr: u16, value: u32) -> bool {
        use addr::*;
        match addr {
            MSTATUS => self.mstatus = value & MASK_MSTATUS,
            MISA => {} // writes silently ignored (spec §3.1.1)
            MIE => self.mie = value & MASK_MIE,
            MTVEC => self.mtvec = value & MASK_MTVEC,
            MSCRATCH => self.mscratch = value,
            MEPC => self.mepc = value & MASK_MEPC,
            MCAUSE => self.mcause = value,
            MTVAL => self.mtval = value,
            MIP => {} // all bits read-only (wire-driven); writes ignored
            // Architecturally read-only: a write here is illegal.
            MVENDORID | MARCHID | MIMPID | MHARTID => return false,
            _ => return false,
        }
        true
    }

    /// Atomic Zicsr read-modify-write.
    ///
    /// `source` is the rs1 value (register variants) or the zero-extended imm5
    /// (`*I` variants); `writes` is whether the instruction performs a write
    /// (RW always; RS/RC only when `source != 0` — the caller computes this).
    /// Returns the old (pre-modify) value on success, or `None` if the CSR is
    /// unrecognised or the write targets a read-only CSR (map to an illegal
    /// halt). Mutates `self` in place — the Core's pipelined Execute/Commit
    /// split instead stages via [`read`](Self::read) / [`write`](Self::write).
    pub fn access(&mut self, addr: u16, op: CsrOp, source: u32, writes: bool) -> Option<u32> {
        let old = self.read(addr)?;
        if writes {
            let new = match op {
                CsrOp::Rw => source,
                CsrOp::Rs => old | source,
                CsrOp::Rc => old & !source,
            };
            if !self.write(addr, new) {
                return None;
            }
        }
        Some(old)
    }

    // --- trap microsequences ----------------------------------------------

    /// Trap-entry microsequence (privileged-arch-plan §4.3):
    /// `mepc ← epc`, `mcause ← cause`, `mtval ← tval`, `MPIE ← MIE`, `MIE ← 0`.
    /// The caller sets `PC = mtvec.BASE` separately.
    pub fn trap_entry(&mut self, cause: u32, epc: u32, tval: u32) {
        self.mepc = epc & MASK_MEPC;
        self.mcause = cause;
        self.mtval = tval;

        let old_mie = self.mstatus & MSTATUS_MIE != 0;
        self.mstatus &= !(MSTATUS_MIE | MSTATUS_MPIE);
        if old_mie {
            self.mstatus |= MSTATUS_MPIE;
        }
        // MIE cleared above; MPP is structurally always 11 (not stored).
    }

    /// `MRET` microsequence (privileged-arch-plan §4.4): `MIE ← MPIE`,
    /// `MPIE ← 1`. The caller sets `PC = mepc` separately.
    pub fn mret(&mut self) {
        let old_mpie = self.mstatus & MSTATUS_MPIE != 0;
        self.mstatus &= !(MSTATUS_MIE | MSTATUS_MPIE);
        self.mstatus |= MSTATUS_MPIE; // MPIE ← 1 per spec §3.3.2
        if old_mpie {
            self.mstatus |= MSTATUS_MIE;
        }
    }

    // --- raw storage accessors (for the commit-packet snapshot) -----------

    /// `mstatus` as the post-step snapshot sees it (stored bits + synthesised
    /// MPP).
    pub fn mstatus_snapshot(&self) -> u32 {
        (self.mstatus & MASK_MSTATUS) | MSTATUS_MPP_M
    }
    /// Raw stored `mtvec`.
    pub fn mtvec_raw(&self) -> u32 {
        self.mtvec
    }
    /// Raw stored `mcause`.
    pub fn mcause_raw(&self) -> u32 {
        self.mcause
    }
    /// Raw stored `mtval`.
    pub fn mtval_raw(&self) -> u32 {
        self.mtval
    }
    /// Raw stored `mscratch`.
    pub fn mscratch_raw(&self) -> u32 {
        self.mscratch
    }
    /// Raw stored `mie`.
    pub fn mie_raw(&self) -> u32 {
        self.mie
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mstatus_synthesises_mpp() {
        let csr = CsrFile::new();
        // MPP = 0b11 even at reset.
        assert_eq!(csr.read(addr::MSTATUS), Some(MSTATUS_MPP_M));
    }

    #[test]
    fn mip_wire_synthesis() {
        let mut csr = CsrFile::new();
        csr.set_wires(true, false, true);
        assert_eq!(csr.read(addr::MIP), Some((1 << 3) | (1 << 11)));
    }

    #[test]
    fn addr_writable_marks_info_csrs_readonly() {
        assert!(CsrFile::addr_writable(addr::MSTATUS));
        assert!(CsrFile::addr_writable(addr::MIP)); // silent, but "writable"
        assert!(!CsrFile::addr_writable(addr::MHARTID)); // addr[11:10] == 0b11
        assert!(!CsrFile::addr_writable(0x999)); // unrecognised
    }

    #[test]
    fn misa_and_mip_writes_silently_ignored() {
        let mut csr = CsrFile::new();
        assert!(csr.write(addr::MISA, 0xFFFF_FFFF));
        assert_eq!(csr.read(addr::MISA), Some(MISA_VALUE));
        assert!(csr.write(addr::MIP, 0xFFFF_FFFF));
        assert_eq!(csr.read(addr::MIP), Some(0));
    }

    #[test]
    fn write_to_readonly_info_csr_rejected() {
        let mut csr = CsrFile::new();
        assert!(!csr.write(addr::MHARTID, 1));
    }

    #[test]
    fn atomic_access_rw_rs_rc() {
        let mut csr = CsrFile::new();
        // RW writes 0xF into mscratch, returns old (0).
        assert_eq!(csr.access(addr::MSCRATCH, CsrOp::Rw, 0xF, true), Some(0));
        // RS sets bit 4, returns old 0xF.
        assert_eq!(csr.access(addr::MSCRATCH, CsrOp::Rs, 0x10, true), Some(0xF));
        assert_eq!(csr.read(addr::MSCRATCH), Some(0x1F));
        // RC clears bit 0, returns old 0x1F.
        assert_eq!(csr.access(addr::MSCRATCH, CsrOp::Rc, 0x1, true), Some(0x1F));
        assert_eq!(csr.read(addr::MSCRATCH), Some(0x1E));
    }

    #[test]
    fn atomic_access_unknown_addr_is_none() {
        let mut csr = CsrFile::new();
        assert_eq!(csr.access(0x999, CsrOp::Rw, 0, true), None);
    }

    #[test]
    fn trap_entry_then_mret_round_trips_mie() {
        let mut csr = CsrFile::new();
        // Enable MIE.
        csr.write(addr::MSTATUS, MSTATUS_MIE);
        csr.trap_entry(11, 0x100, 0);
        // After trap: MIE cleared, MPIE set, mepc/mcause recorded.
        assert!(!csr.mie_enabled());
        assert_eq!(csr.mepc(), 0x100);
        assert_eq!(csr.read(addr::MCAUSE), Some(11));
        // mret restores MIE.
        csr.mret();
        assert!(csr.mie_enabled());
    }

    #[test]
    fn mtvec_mode_bits_forced_zero() {
        let mut csr = CsrFile::new();
        csr.write(addr::MTVEC, 0x8000_0003); // request vectored mode
        assert_eq!(csr.mtvec_base(), 0x8000_0000);
    }
}
