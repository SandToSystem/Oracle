//! The [`Bus`] abstraction the Core talks to and a flat-memory [`RamBus`].
//!
//! The AXI vocabulary ([`AxiResp`], [`Slverr`], [`Width`]) is **owned by the
//! Hermes submodule** and re-exported here — ISS used to define its own
//! divergent copies; now there is a single definition. The [`Bus`] trait and
//! [`RamBus`] are genuinely CPU-side (Hermes has no equivalent) and stay here.
//!
//! The Core talks to the bus through [`Bus::read`] / [`Bus::write`]; the
//! [`impl Bus for hermes::MemoryMap`](Bus) bridge at the bottom lets a `Core`
//! run on the full Hermes SoC fabric, while [`RamBus`] keeps the flat
//! fast-path for instruction-level tests.

pub use hermes::{AxiResp, Slverr, Width};

/// Natural-alignment test for a [`Width`].
///
/// ISS needs this in fetch / branch-target / load-store alignment checks, but
/// Hermes's shared [`Width`] does not carry it. Provided as an ISS-local
/// extension trait so the Hermes submodule stays the untouched source of truth.
pub trait WidthAlign {
    /// `true` when `addr` is naturally aligned for this width (spec §2.6).
    fn is_aligned(&self, addr: u32) -> bool;
}

impl WidthAlign for Width {
    fn is_aligned(&self, addr: u32) -> bool {
        addr & (self.bytes() - 1) == 0
    }
}

/// A memory-mapped bus addressed by *system* address.
///
/// The Core only ever calls [`read`](Bus::read) / [`write`](Bus::write); it has
/// no other dependency on the SoC topology. Implementations return
/// [`AxiResp::Decerr`] for an unmapped address and propagate a device's
/// [`AxiResp::Slverr`] for a subordinate-level error.
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
/// region (or straddling its end) is [`AxiResp::Decerr`].
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
        let r = self.range(addr, width).ok_or(AxiResp::Decerr)?;
        Ok(hermes::le_load(&self.mem[r]))
    }

    fn write(&mut self, addr: u32, width: Width, value: u32, strb: u8) -> Result<(), AxiResp> {
        let r = self.range(addr, width).ok_or(AxiResp::Decerr)?;
        hermes::le_store(&mut self.mem[r], value, strb);
        Ok(())
    }
}

/// Bridge ISS's CPU-side [`Bus`] onto Hermes's address-decoding fabric, so a
/// `Core<MemoryMap>` runs against the full SoC. `MemoryMap`'s own `read`/`write`
/// take `&self` (interior mutability via `Rc<RefCell<_>>`); the trait wants
/// `&mut self`, which trivially satisfies it. Fully-qualified calls avoid
/// recursing into the trait method.
impl Bus for hermes::MemoryMap {
    fn read(&mut self, addr: u32, width: Width) -> Result<u32, AxiResp> {
        hermes::MemoryMap::read(self, addr, width)
    }

    fn write(&mut self, addr: u32, width: Width, value: u32, strb: u8) -> Result<(), AxiResp> {
        hermes::MemoryMap::write(self, addr, width, value, strb)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn width_is_aligned() {
        // `bytes`/`full_strb` are tested in Hermes; here we cover the ISS-local
        // `WidthAlign` extension (brought in by `use super::*`).
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
        assert_eq!(bus.read(0x0, Width::Word), Err(AxiResp::Decerr));
        assert_eq!(bus.read(0x100C, Width::Word), Ok(0)); // last fully in-range word
        assert_eq!(bus.read(0x100D, Width::Word), Err(AxiResp::Decerr)); // straddles end
        assert_eq!(bus.read(0x100F, Width::Byte), Ok(0)); // last in-range byte
    }
}
