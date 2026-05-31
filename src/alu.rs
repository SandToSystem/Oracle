//! The ALU and branch-condition evaluators — pure functions over `u32`.
//!
//! [`AluOp::apply`] is a single exhaustive `match`; a closure table would be
//! less readable here, so the "closures where they pay off" budget is spent in
//! [`Core::run_until_halt`](crate::Core::run_until_halt) instead. All shifts
//! mask the amount to 5 bits and all arithmetic wraps, matching C's defined
//! behaviour and avoiding debug-mode overflow panics.

/// The ten RV32I integer ALU operations.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum AluOp {
    /// `a + b`
    Add,
    /// `a << (b & 0x1F)`
    Sll,
    /// signed `a < b` → 1/0
    Slt,
    /// unsigned `a < b` → 1/0
    Sltu,
    /// `a ^ b`
    Xor,
    /// logical `a >> (b & 0x1F)`
    Srl,
    /// `a | b`
    Or,
    /// `a & b`
    And,
    /// `a - b`
    Sub,
    /// arithmetic `a >> (b & 0x1F)`
    Sra,
}

impl AluOp {
    /// Compute `op(a, b)`. Shift amounts are masked to 5 bits per spec §2.4.
    pub fn apply(self, a: u32, b: u32) -> u32 {
        let sh = b & 0x1F;
        match self {
            AluOp::Add => a.wrapping_add(b),
            AluOp::Sub => a.wrapping_sub(b),
            AluOp::Sll => a.wrapping_shl(sh),
            AluOp::Srl => a.wrapping_shr(sh),
            AluOp::Sra => ((a as i32) >> sh) as u32,
            AluOp::Slt => ((a as i32) < (b as i32)) as u32,
            AluOp::Sltu => (a < b) as u32,
            AluOp::Xor => a ^ b,
            AluOp::Or => a | b,
            AluOp::And => a & b,
        }
    }
}

/// Branch condition. `JumpAnyway` covers the unconditional `JAL`/`JALR`.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum BranchType {
    /// Always taken (`JAL` / `JALR`).
    JumpAnyway,
    /// `BEQ`
    Eq,
    /// `BNE`
    Neq,
    /// `BLT` (signed)
    Lt,
    /// `BLTU` (unsigned)
    Ltu,
    /// `BGE` (signed)
    Ge,
    /// `BGEU` (unsigned)
    Geu,
}

impl BranchType {
    /// Evaluate the condition against the two source-register values.
    pub fn taken(self, rs1: u32, rs2: u32) -> bool {
        match self {
            BranchType::JumpAnyway => true,
            BranchType::Eq => rs1 == rs2,
            BranchType::Neq => rs1 != rs2,
            BranchType::Lt => (rs1 as i32) < (rs2 as i32),
            BranchType::Ge => (rs1 as i32) >= (rs2 as i32),
            BranchType::Ltu => rs1 < rs2,
            BranchType::Geu => rs1 >= rs2,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shifts_mask_amount_and_dont_panic() {
        // Shift amount 32 masks to 0 → identity (would panic with plain `<<`).
        assert_eq!(AluOp::Sll.apply(1, 32), 1);
        assert_eq!(AluOp::Srl.apply(0x8000_0000, 32), 0x8000_0000);
        assert_eq!(AluOp::Sra.apply(0x8000_0000, 1), 0xC000_0000);
    }

    #[test]
    fn add_sub_wrap() {
        assert_eq!(AluOp::Add.apply(0xFFFF_FFFF, 1), 0);
        assert_eq!(AluOp::Sub.apply(0, 1), 0xFFFF_FFFF);
    }

    #[test]
    fn signed_vs_unsigned_compare() {
        assert_eq!(AluOp::Slt.apply(0xFFFF_FFFF, 1), 1); // -1 < 1
        assert_eq!(AluOp::Sltu.apply(0xFFFF_FFFF, 1), 0); // big < 1 = false
    }

    #[test]
    fn branch_conditions() {
        assert!(BranchType::Eq.taken(5, 5));
        assert!(BranchType::Lt.taken(0xFFFF_FFFF, 0)); // -1 < 0
        assert!(!BranchType::Ltu.taken(0xFFFF_FFFF, 0));
        assert!(BranchType::JumpAnyway.taken(1, 2));
    }
}
