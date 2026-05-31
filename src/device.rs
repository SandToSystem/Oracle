//! Device-dispatch bus — the Rust port of the C `MemoryMap` / `MmioDevice`
//! pair.
//!
//! [`MmioDevice`] is the trait a memory-mapped subordinate implements, using a
//! *device-relative* offset (the C vtable took `addr - unit.base`). [`DeviceBus`]
//! owns a set of `(range, device)` units and implements [`Bus`] by locating the
//! covering unit and forwarding the access with the base subtracted. Overlap is
//! rejected at registration ([`DeviceBus::add`]) so decode errors surface during
//! construction, and an unmapped address yields [`AxiResp::DecErr`].

use core::ops::Range;

use crate::bus::{AxiResp, Bus, Width};

/// A memory-mapped subordinate, addressed by an offset relative to its base.
///
/// Subordinates never return [`AxiResp::DecErr`] — that is fabric-only and is
/// synthesised by [`DeviceBus`]. A subordinate returns [`AxiResp::SlvErr`] for a
/// register-level error (write to a read-only register, bad width/strobe, or an
/// offset outside any defined sub-region).
pub trait MmioDevice {
    /// Read `width` bytes at device-relative `offset`.
    fn read(&mut self, offset: u32, width: Width) -> Result<u32, AxiResp>;

    /// Write the low `width` bytes of `value` at `offset` under `strb`.
    fn write(&mut self, offset: u32, width: Width, value: u32, strb: u8) -> Result<(), AxiResp>;
}

/// Why a [`DeviceBus::add`] registration was rejected.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AddError {
    /// `size == 0`, or `base + size` overflows the 32-bit address space.
    BadRange,
    /// The requested range overlaps an already-registered unit.
    Overlap,
}

struct Unit {
    range: Range<u32>,
    device: Box<dyn MmioDevice>,
}

/// A registry of [`MmioDevice`]s dispatched by address range — the C
/// `MemoryMap`. Non-owning in C; here the bus owns its boxed devices.
#[derive(Default)]
pub struct DeviceBus {
    units: Vec<Unit>,
}

impl DeviceBus {
    /// An empty registry.
    pub fn new() -> Self {
        DeviceBus { units: Vec::new() }
    }

    /// Register `device` over `[base, base + size)`.
    ///
    /// Returns [`AddError::BadRange`] for a zero or overflowing range and
    /// [`AddError::Overlap`] if it intersects an existing unit. Registration
    /// order does not affect dispatch.
    pub fn add(
        &mut self,
        base: u32,
        size: u32,
        device: Box<dyn MmioDevice>,
    ) -> Result<(), AddError> {
        let end = base.checked_add(size).ok_or(AddError::BadRange)?;
        if size == 0 {
            return Err(AddError::BadRange);
        }
        let new = base..end;
        if self.units.iter().any(|u| ranges_overlap(&u.range, &new)) {
            return Err(AddError::Overlap);
        }
        self.units.push(Unit { range: new, device });
        Ok(())
    }

    /// Locate the unit owning `[addr, addr + width)`, if any. An access that
    /// starts inside a unit but extends past its end is treated as unmapped
    /// (no straddling across the fabric boundary).
    fn unit_for(&mut self, addr: u32, width: Width) -> Option<&mut Unit> {
        let end = addr.checked_add(width.bytes())?;
        self.units
            .iter_mut()
            .find(|u| u.range.start <= addr && end <= u.range.end)
    }
}

impl Bus for DeviceBus {
    fn read(&mut self, addr: u32, width: Width) -> Result<u32, AxiResp> {
        match self.unit_for(addr, width) {
            Some(u) => {
                let offset = addr - u.range.start;
                u.device.read(offset, width)
            }
            None => Err(AxiResp::DecErr),
        }
    }

    fn write(&mut self, addr: u32, width: Width, value: u32, strb: u8) -> Result<(), AxiResp> {
        match self.unit_for(addr, width) {
            Some(u) => {
                let offset = addr - u.range.start;
                u.device.write(offset, width, value, strb)
            }
            None => Err(AxiResp::DecErr),
        }
    }
}

/// Half-open `[start, end)` intersection test.
fn ranges_overlap(a: &Range<u32>, b: &Range<u32>) -> bool {
    a.start < b.end && b.start < a.end
}

/// A flat little-endian RAM as an [`MmioDevice`] — the simplest subordinate,
/// for composing memory regions into a [`DeviceBus`].
pub struct Ram {
    mem: Vec<u8>,
}

impl Ram {
    /// A zero-filled RAM of `len` bytes.
    pub fn new(len: usize) -> Self {
        Ram { mem: vec![0; len] }
    }

    /// A RAM initialised from `bytes`.
    pub fn from_bytes(bytes: Vec<u8>) -> Self {
        Ram { mem: bytes }
    }

    fn range(&self, offset: u32, width: Width) -> Option<Range<usize>> {
        let off = offset as usize;
        let end = off.checked_add(width.bytes() as usize)?;
        (end <= self.mem.len()).then_some(off..end)
    }
}

impl MmioDevice for Ram {
    fn read(&mut self, offset: u32, width: Width) -> Result<u32, AxiResp> {
        let r = self.range(offset, width).ok_or(AxiResp::SlvErr)?;
        let mut buf = [0u8; 4];
        buf[..r.len()].copy_from_slice(&self.mem[r]);
        Ok(u32::from_le_bytes(buf))
    }

    fn write(&mut self, offset: u32, width: Width, value: u32, strb: u8) -> Result<(), AxiResp> {
        let r = self.range(offset, width).ok_or(AxiResp::SlvErr)?;
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

    /// A write-only register that returns SLVERR on read — exercises device
    /// response passthrough.
    struct WriteOnly;
    impl MmioDevice for WriteOnly {
        fn read(&mut self, _o: u32, _w: Width) -> Result<u32, AxiResp> {
            Err(AxiResp::SlvErr)
        }
        fn write(&mut self, _o: u32, _w: Width, _v: u32, _s: u8) -> Result<(), AxiResp> {
            Ok(())
        }
    }

    #[test]
    fn dispatch_forwards_relative_offset() {
        let mut bus = DeviceBus::new();
        bus.add(0x4000_0000, 0x1000, Box::new(Ram::new(0x1000)))
            .unwrap();
        bus.write(0x4000_0010, Width::Word, 0x1234_5678, 0xF)
            .unwrap();
        // Same word read back through the device.
        assert_eq!(bus.read(0x4000_0010, Width::Word), Ok(0x1234_5678));
    }

    #[test]
    fn unmapped_is_decerr() {
        let mut bus = DeviceBus::new();
        bus.add(0x4000_0000, 0x10, Box::new(Ram::new(0x10)))
            .unwrap();
        assert_eq!(bus.read(0x5000_0000, Width::Word), Err(AxiResp::DecErr));
        // Starts in-range but straddles the end → unmapped.
        assert_eq!(bus.read(0x4000_000E, Width::Word), Err(AxiResp::DecErr));
    }

    #[test]
    fn overlap_rejected_at_registration() {
        let mut bus = DeviceBus::new();
        bus.add(0x1000, 0x100, Box::new(Ram::new(0x100))).unwrap();
        assert_eq!(
            bus.add(0x1080, 0x100, Box::new(Ram::new(0x100))),
            Err(AddError::Overlap)
        );
        assert_eq!(
            bus.add(0x2000, 0, Box::new(Ram::new(0))),
            Err(AddError::BadRange)
        );
        // Adjacent (non-overlapping) is fine.
        assert!(bus.add(0x1100, 0x100, Box::new(Ram::new(0x100))).is_ok());
    }

    #[test]
    fn device_slverr_passes_through() {
        let mut bus = DeviceBus::new();
        bus.add(0x0, 0x10, Box::new(WriteOnly)).unwrap();
        assert_eq!(bus.read(0x0, Width::Word), Err(AxiResp::SlvErr));
        assert_eq!(bus.write(0x0, Width::Word, 0, 0xF), Ok(()));
    }
}
