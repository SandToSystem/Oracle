//! ELF program loader — parse an ELF32 RISC-V executable and write its
//! loadable segments into a [`Bus`].
//!
//! A Rust port of the C `elf_loader.c` from `comporg-labs/elf_loader`, with two
//! deliberate changes over the original:
//!
//! 1. **Parsing** uses the [`elf`] crate rather than hand-rolled `<elf.h>`
//!    struct reads — endian-aware and validated, and it exposes the symbol table
//!    a future riscv-tests harness needs for `tohost`.
//! 2. **Generic over [`Bus`]** instead of hard-wired to a `MemoryMap`. The same
//!    loader stages programs into [`RamBus`](crate::RamBus) for fast tests or
//!    into the Hermes `MemoryMap` + `Dram` fabric. The C's hard-coded
//!    `e_entry == 0x8000_0000` Model-B check is dropped; the entry point is
//!    returned for the caller to validate against its own memory map.
//!
//! Like the C original, the loader copies `[p_offset, p_offset + p_filesz)` for
//! each `PT_LOAD` segment and does **not** zero the `p_memsz - p_filesz` tail —
//! clearing `.bss` is crt0's job.

use std::path::Path;

use elf::abi::{EM_RISCV, ET_EXEC, PT_LOAD};
use elf::endian::AnyEndian;
use elf::file::Class;
use elf::ElfBytes;

use crate::bus::{AxiResp, Bus, Width};

/// Everything that can go wrong loading an ELF.
///
/// Hand-written `Display`/`Error` impls (no `thiserror`), matching the house
/// style of Hermes's `DramError` / `MapError`.
#[derive(Debug)]
pub enum ElfError {
    /// The `elf` crate could not parse the image (bad magic, truncation, a
    /// segment whose data falls outside the buffer, …).
    Parse(elf::ParseError),
    /// Reading the file from disk failed (path-based [`load_elf`] only).
    Io(std::io::Error),
    /// `EI_CLASS` was not `ELFCLASS32`.
    NotElf32,
    /// `e_machine` was not `EM_RISCV`.
    NotRiscv,
    /// `e_type` was not `ET_EXEC`.
    NotExecutable,
    /// A loadable segment had `p_paddr != p_vaddr` (no identity mapping).
    SegmentPaddrMismatch { vaddr: u32, paddr: u32 },
    /// A segment write was rejected by the bus — the address is unmapped
    /// (`Decerr`) or the device refused it (`Slverr`). The `-EFAULT` analogue.
    Bus { addr: u32, resp: AxiResp },
}

impl std::fmt::Display for ElfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ElfError::Parse(e) => write!(f, "ELF parse error: {e}"),
            ElfError::Io(e) => write!(f, "ELF read error: {e}"),
            ElfError::NotElf32 => f.write_str("not an ELFCLASS32 object"),
            ElfError::NotRiscv => f.write_str("e_machine is not EM_RISCV"),
            ElfError::NotExecutable => f.write_str("e_type is not ET_EXEC"),
            ElfError::SegmentPaddrMismatch { vaddr, paddr } => {
                write!(f, "segment p_paddr {paddr:#010x} != p_vaddr {vaddr:#010x}")
            }
            ElfError::Bus { addr, resp } => {
                write!(f, "bus rejected segment write at {addr:#010x}: {resp:?}")
            }
        }
    }
}

impl std::error::Error for ElfError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ElfError::Parse(e) => Some(e),
            ElfError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<elf::ParseError> for ElfError {
    fn from(e: elf::ParseError) -> Self {
        ElfError::Parse(e)
    }
}

impl From<std::io::Error> for ElfError {
    fn from(e: std::io::Error) -> Self {
        ElfError::Io(e)
    }
}

/// Parse an ELF image and write its loadable segments into `bus`.
///
/// On success returns the ELF entry point (`e_entry`). On failure `bus` may have
/// been partially written; callers should not rely on its contents.
///
/// This is the testable core; [`load_elf`] is the thin file-path wrapper.
pub fn load_elf_bytes<B: Bus>(bus: &mut B, image: &[u8]) -> Result<u32, ElfError> {
    let file = ElfBytes::<AnyEndian>::minimal_parse(image)?;

    if file.ehdr.class != Class::ELF32 {
        return Err(ElfError::NotElf32);
    }
    if file.ehdr.e_machine != EM_RISCV {
        return Err(ElfError::NotRiscv);
    }
    if file.ehdr.e_type != ET_EXEC {
        return Err(ElfError::NotExecutable);
    }

    if let Some(segments) = file.segments() {
        for phdr in segments {
            if phdr.p_type != PT_LOAD || phdr.p_filesz == 0 {
                continue;
            }
            if phdr.p_paddr != phdr.p_vaddr {
                return Err(ElfError::SegmentPaddrMismatch {
                    vaddr: phdr.p_vaddr as u32,
                    paddr: phdr.p_paddr as u32,
                });
            }
            let data = file.segment_data(&phdr)?;
            write_segment(bus, phdr.p_paddr as u32, data)?;
        }
    }

    Ok(file.ehdr.e_entry as u32)
}

/// Read an ELF from `path` and load it into `bus` via [`load_elf_bytes`].
pub fn load_elf<B: Bus>(bus: &mut B, path: impl AsRef<Path>) -> Result<u32, ElfError> {
    let image = std::fs::read(path)?;
    load_elf_bytes(bus, &image)
}

/// Copy `data` into `bus` starting at system address `addr`.
///
/// Three phases — leading bytes until 4-byte aligned, then aligned words, then
/// trailing bytes — so an unaligned segment start or a length that is not a
/// multiple of 4 still satisfies a device's "word writes must be naturally
/// aligned and full-strobe" rule (e.g. Hermes `Dram`).
///
/// Strobe note: unlike the C original (which uses `1 << (addr & 3)` for byte
/// lanes because its `MemoryMap_write` indexes within a word), the Rust
/// [`Bus`]/`le_store` path slices exactly the accessed bytes — so a byte write
/// uses `strb = 0x1` with the byte in `value`'s low lane.
fn write_segment<B: Bus>(bus: &mut B, addr: u32, data: &[u8]) -> Result<(), ElfError> {
    let mut i: usize = 0;

    let put_byte = |bus: &mut B, off: usize| -> Result<(), ElfError> {
        let a = addr + off as u32;
        bus.write(a, Width::Byte, data[off] as u32, 0x1)
            .map_err(|resp| ElfError::Bus { addr: a, resp })
    };

    // Leading bytes until the destination address is 4-byte aligned.
    while i < data.len() && (addr + i as u32) & 0x3 != 0 {
        put_byte(bus, i)?;
        i += 1;
    }

    // Aligned 32-bit words.
    while i + 4 <= data.len() {
        let word = u32::from_le_bytes([data[i], data[i + 1], data[i + 2], data[i + 3]]);
        let a = addr + i as u32;
        bus.write(a, Width::Word, word, 0xF)
            .map_err(|resp| ElfError::Bus { addr: a, resp })?;
        i += 4;
    }

    // Trailing bytes.
    while i < data.len() {
        put_byte(bus, i)?;
        i += 1;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RamBus;

    const BASE: u32 = 0x8000_0000;

    /// The 4-instruction payload used by the synthetic ELF (contents arbitrary).
    const PROG: [u32; 4] = [
        0x0050_0513, // addi x10, x0, 5
        0xFFF5_0513, // addi x10, x10, -1
        0xFE05_1EE3, // bne  x10, x0, -4
        0x0000_006F, // jal  x0, 0
    ];

    /// Build a minimal single-`PT_LOAD` ELF32 image: ehdr (52B) + phdr (32B) +
    /// `payload`, with entry and segment vaddr/paddr at `entry`. Mirrors the C
    /// `build_minimal_elf` in `test_elf_loader.c`.
    fn synth_elf(entry: u32, payload: &[u8]) -> Vec<u8> {
        const EHSZ: usize = 52;
        const PHSZ: usize = 32;
        let off_payload = EHSZ + PHSZ;
        let mut buf = vec![0u8; off_payload + payload.len()];

        // --- Elf32_Ehdr ---
        buf[0..4].copy_from_slice(&[0x7F, b'E', b'L', b'F']);
        buf[4] = 1; // EI_CLASS = ELFCLASS32
        buf[5] = 1; // EI_DATA  = ELFDATA2LSB
        buf[6] = 1; // EI_VERSION = EV_CURRENT
        let w16 =
            |buf: &mut [u8], o: usize, v: u16| buf[o..o + 2].copy_from_slice(&v.to_le_bytes());
        let w32 =
            |buf: &mut [u8], o: usize, v: u32| buf[o..o + 4].copy_from_slice(&v.to_le_bytes());
        w16(&mut buf, 16, 2); // e_type = ET_EXEC
        w16(&mut buf, 18, 243); // e_machine = EM_RISCV
        w32(&mut buf, 20, 1); // e_version
        w32(&mut buf, 24, entry); // e_entry
        w32(&mut buf, 28, EHSZ as u32); // e_phoff
        w16(&mut buf, 40, EHSZ as u16); // e_ehsize
        w16(&mut buf, 42, PHSZ as u16); // e_phentsize
        w16(&mut buf, 44, 1); // e_phnum

        // --- Elf32_Phdr at EHSZ ---
        let p = EHSZ;
        w32(&mut buf, p, PT_LOAD); // p_type
        w32(&mut buf, p + 4, off_payload as u32); // p_offset
        w32(&mut buf, p + 8, entry); // p_vaddr
        w32(&mut buf, p + 12, entry); // p_paddr
        w32(&mut buf, p + 16, payload.len() as u32); // p_filesz
        w32(&mut buf, p + 20, payload.len() as u32); // p_memsz
        w32(&mut buf, p + 24, 0x5); // p_flags = R|X
        w32(&mut buf, p + 28, 4); // p_align

        buf[off_payload..].copy_from_slice(payload);
        buf
    }

    fn prog_bytes() -> Vec<u8> {
        PROG.iter().flat_map(|w| w.to_le_bytes()).collect()
    }

    #[test]
    fn loads_minimal_elf_into_rambus() {
        let elf = synth_elf(BASE, &prog_bytes());
        let mut bus = RamBus::new(BASE, 0x1000);

        let entry = load_elf_bytes(&mut bus, &elf).unwrap();

        assert_eq!(entry, BASE);
        assert_eq!(bus.read(BASE, Width::Word), Ok(PROG[0]));
        assert_eq!(bus.read(BASE + 12, Width::Word), Ok(PROG[3]));
    }

    #[test]
    fn bad_magic_is_rejected() {
        let mut elf = synth_elf(BASE, &prog_bytes());
        elf[0] = 0x00; // corrupt the magic
        let mut bus = RamBus::new(BASE, 0x1000);
        assert!(matches!(
            load_elf_bytes(&mut bus, &elf),
            Err(ElfError::Parse(_))
        ));
    }

    #[test]
    fn wrong_machine_is_rejected() {
        let mut elf = synth_elf(BASE, &prog_bytes());
        // e_machine at offset 18 -> something other than EM_RISCV (243).
        elf[18..20].copy_from_slice(&3u16.to_le_bytes()); // EM_386
        let mut bus = RamBus::new(BASE, 0x1000);
        assert!(matches!(
            load_elf_bytes(&mut bus, &elf),
            Err(ElfError::NotRiscv)
        ));
    }

    #[test]
    fn unaligned_start_and_odd_length_round_trip() {
        // Segment starts 1 byte into the word and is 6 bytes long: exercises
        // the leading-byte, single-word, and trailing-byte phases together.
        let payload: [u8; 6] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
        let start = BASE + 1;
        let elf = synth_elf(start, &payload);
        let mut bus = RamBus::new(BASE, 0x1000);

        let entry = load_elf_bytes(&mut bus, &elf).unwrap();
        assert_eq!(entry, start);
        for (i, &b) in payload.iter().enumerate() {
            assert_eq!(bus.read(start + i as u32, Width::Byte), Ok(b as u32));
        }
        // Bytes outside the segment stay zero.
        assert_eq!(bus.read(BASE, Width::Byte), Ok(0));
        assert_eq!(bus.read(start + 6, Width::Byte), Ok(0));
    }

    #[test]
    fn segment_outside_bus_is_bus_error() {
        // Bus covers only [BASE, BASE+8); a 16-byte segment overruns it.
        let elf = synth_elf(BASE, &prog_bytes());
        let mut bus = RamBus::new(BASE, 8);
        assert!(matches!(
            load_elf_bytes(&mut bus, &elf),
            Err(ElfError::Bus { .. })
        ));
    }
}
