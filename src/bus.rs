//! The [`Bus`] abstraction the Core talks to, plus value types shared across
//! every bus access ([`AxiResp`], [`Width`]) and a flat-memory [`RamBus`].
//!
//! Mirrors the C `MemoryMap` entry points and the `axi_resp_e` vocabulary from
//! `devices/common`. The C API used `bool`/out-parameters and a `size_t width`;
//! here the result is a [`Result`] and the width is an enum so an illegal width
//! is unrepresentable.

/// AXI4-Lite response codes (`xRESP`), 2-bit wire encoding.
///
/// The discriminants match the AXI wire values verbatim so a future DPI bridge
/// is a pass-through cast. [`AxiResp::ExOkay`] is defined for completeness but
/// is forbidden by AXI4-Lite (no exclusive access).
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum AxiResp {
    /// `2'b00` — normal access success.
    Okay = 0,
    /// `2'b01` — forbidden in AXI4-Lite; present only for wire completeness.
    ExOkay = 1,
    /// `2'b10` — subordinate-level error (e.g. write to a read-only register).
    SlvErr = 2,
    /// `2'b11` — decode error; raised by the fabric for an unmapped address.
    DecErr = 3,
}

impl AxiResp {
    /// Human-readable tag for diagnostics (mirrors C `axi_resp_name`).
    pub fn name(self) -> &'static str {
        match self {
            AxiResp::Okay => "OKAY",
            AxiResp::ExOkay => "EXOKAY",
            AxiResp::SlvErr => "SLVERR",
            AxiResp::DecErr => "DECERR",
        }
    }
}

/// Access width in bytes. RV32I memory accesses are 1, 2, or 4 bytes wide.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Width {
    /// 1 byte (`LB`/`LBU`/`SB`).
    Byte = 1,
    /// 2 bytes (`LH`/`LHU`/`SH`).
    Half = 2,
    /// 4 bytes (`LW`/`SW` and every instruction fetch).
    Word = 4,
}

impl Width {
    /// The width as a byte count (1 / 2 / 4).
    pub const fn bytes(self) -> u32 {
        self as u32
    }

    /// The full AXI `WSTRB` byte-enable for a packed store of this width
    /// (`0x1` / `0x3` / `0xF`). Per the L1 contract the strobe masks the low
    /// `width` bytes of the value, not lanes within the 32-bit word.
    pub const fn full_strb(self) -> u8 {
        match self {
            Width::Byte => 0x1,
            Width::Half => 0x3,
            Width::Word => 0xF,
        }
    }

    /// `true` when `addr` is naturally aligned for this width (spec §2.6).
    pub const fn is_aligned(self, addr: u32) -> bool {
        addr & (self.bytes() - 1) == 0
    }
}

/// A memory-mapped bus addressed by *system* address.
///
/// The Core only ever calls [`read`](Bus::read) / [`write`](Bus::write); it has
/// no other dependency on the SoC topology. Implementations return
/// [`AxiResp::DecErr`] for an unmapped address and propagate a device's
/// [`AxiResp::SlvErr`] for a subordinate-level error.
pub trait Bus {
    /// Read `width` bytes at `addr`, returning the zero-extended word.
    fn read(&mut self, addr: u32, width: Width) -> Result<u32, AxiResp>;

    /// Write the low `width` bytes of `value` at `addr` under the `strb`
    /// byte-enable (bit *i* enables byte *i*). For [`Width::Word`] `strb` must
    /// be `0xF` per the AXI4-Lite minimal subset.
    fn write(&mut self, addr: u32, width: Width, value: u32, strb: u8) -> Result<(), AxiResp>;
}

/// Blanket forward so `Core<&mut dyn Bus>` (or any `&mut B`) works — the `dyn`
/// escape hatch for callers who need dynamic dispatch over a generic `Core`.
impl<B: Bus + ?Sized> Bus for &mut B {
    fn read(&mut self, addr: u32, width: Width) -> Result<u32, AxiResp> {
        (**self).read(addr, width)
    }
    fn write(&mut self, addr: u32, width: Width, value: u32, strb: u8) -> Result<(), AxiResp> {
        (**self).write(addr, width, value, strb)
    }
}

/// A flat little-endian RAM covering `[base, base + len)`. The canonical bus
/// for tests: every in-range access is [`AxiResp::Okay`]; anything outside the
/// region (or straddling its end) is [`AxiResp::DecErr`].
#[derive(Clone, Debug)]
pub struct RamBus {
    base: u32,
    mem: Vec<u8>,
}

impl RamBus {
    /// A zero-filled region of `len` bytes based at `base`.
    pub fn new(base: u32, len: usize) -> Self {
        RamBus {
            base,
            mem: vec![0; len],
        }
    }

    /// A region initialised from `bytes` (its length becomes the region size).
    pub fn from_bytes(base: u32, bytes: Vec<u8>) -> Self {
        RamBus { base, mem: bytes }
    }

    /// Load a little-endian 32-bit `word` at system `addr` (panics if out of
    /// range — a setup-time helper, not a bus access).
    pub fn load_word(&mut self, addr: u32, word: u32) {
        let off = (addr - self.base) as usize;
        self.mem[off..off + 4].copy_from_slice(&word.to_le_bytes());
    }

    /// Load a sequence of little-endian words starting at `addr` — handy for
    /// staging a small program image.
    pub fn load_program(&mut self, addr: u32, words: &[u32]) {
        for (i, &w) in words.iter().enumerate() {
            self.load_word(addr + (i as u32) * 4, w);
        }
    }

    /// Resolve a system address+width to an in-bounds local byte range.
    fn range(&self, addr: u32, width: Width) -> Option<core::ops::Range<usize>> {
        let off = addr.checked_sub(self.base)? as usize;
        let end = off.checked_add(width.bytes() as usize)?;
        (end <= self.mem.len()).then_some(off..end)
    }
}

impl Bus for RamBus {
    fn read(&mut self, addr: u32, width: Width) -> Result<u32, AxiResp> {
        let r = self.range(addr, width).ok_or(AxiResp::DecErr)?;
        let mut buf = [0u8; 4];
        buf[..r.len()].copy_from_slice(&self.mem[r]);
        Ok(u32::from_le_bytes(buf))
    }

    fn write(&mut self, addr: u32, width: Width, value: u32, strb: u8) -> Result<(), AxiResp> {
        let r = self.range(addr, width).ok_or(AxiResp::DecErr)?;
        let bytes = value.to_le_bytes();
        for (i, slot) in self.mem[r].iter_mut().enumerate() {
            if strb & (1 << i) != 0 {
                *slot = bytes[i];
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn width_helpers() {
        assert_eq!(Width::Word.bytes(), 4);
        assert_eq!(Width::Half.full_strb(), 0x3);
        assert!(Width::Word.is_aligned(0x40));
        assert!(!Width::Word.is_aligned(0x42));
        assert!(Width::Byte.is_aligned(0x3));
    }

    #[test]
    fn rambus_roundtrip_little_endian() {
        let mut bus = RamBus::new(0x8000_0000, 0x100);
        bus.write(0x8000_0000, Width::Word, 0xDEAD_BEEF, 0xF)
            .unwrap();
        assert_eq!(bus.read(0x8000_0000, Width::Word), Ok(0xDEAD_BEEF));
        // Byte 0 is the least-significant byte.
        assert_eq!(bus.read(0x8000_0000, Width::Byte), Ok(0xEF));
        assert_eq!(bus.read(0x8000_0001, Width::Byte), Ok(0xBE));
    }

    #[test]
    fn rambus_partial_strobe() {
        let mut bus = RamBus::new(0, 0x10);
        bus.write(0, Width::Word, 0xFFFF_FFFF, 0xF).unwrap();
        // Only byte 0 enabled.
        bus.write(0, Width::Byte, 0x00, 0x1).unwrap();
        assert_eq!(bus.read(0, Width::Word), Ok(0xFFFF_FF00));
    }

    #[test]
    fn rambus_out_of_range_is_decerr() {
        let mut bus = RamBus::new(0x1000, 0x10);
        assert_eq!(bus.read(0x0, Width::Word), Err(AxiResp::DecErr));
        assert_eq!(bus.read(0x100C, Width::Word), Ok(0)); // last fully in-range word
        assert_eq!(bus.read(0x100D, Width::Word), Err(AxiResp::DecErr)); // straddles end
        assert_eq!(bus.read(0x100F, Width::Byte), Ok(0)); // last in-range byte
    }
}
