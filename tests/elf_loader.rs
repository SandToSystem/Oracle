//! End-to-end ELF loader tests over the real Hermes fabric.
//!
//! A synthetic ELF32 RISC-V executable is loaded into a `MemoryMap` + `Dram`
//! at `0x8000_0000`, then the `Core`'s PC is seeded with the returned entry and
//! run to completion — proving the loader parses, maps, and *executes* a real
//! ELF image. A second test exercises the file-path entry point.

use std::cell::RefCell;
use std::rc::Rc;

use iss_core::{load_elf, load_elf_bytes, Core, Dram, DramModel, MemoryMap, MmioDevice, Width};

const BASE: u32 = 0x8000_0000;
const DRAM_SIZE: u32 = 0x1000;

/// ```text
///   addi a0, x0, 0x2A   ; a0 = 42
///   ebreak              ; voluntary halt, exit = a0 & 0xFF = 42
/// ```
const PROGRAM: [u32; 2] = [
    0x02A0_0513, // addi a0, x0, 0x2A
    0x0010_0073, // ebreak
];

/// Build a minimal single-`PT_LOAD` ELF32 image: ehdr (52B) + phdr (32B) +
/// `payload`, entry and segment vaddr/paddr at `entry`.
fn synth_elf(entry: u32, payload: &[u8]) -> Vec<u8> {
    const EHSZ: usize = 52;
    const PHSZ: usize = 32;
    let off_payload = EHSZ + PHSZ;
    let mut buf = vec![0u8; off_payload + payload.len()];

    buf[0..4].copy_from_slice(&[0x7F, b'E', b'L', b'F']);
    buf[4] = 1; // ELFCLASS32
    buf[5] = 1; // ELFDATA2LSB
    buf[6] = 1; // EV_CURRENT
    let w16 = |buf: &mut [u8], o: usize, v: u16| buf[o..o + 2].copy_from_slice(&v.to_le_bytes());
    let w32 = |buf: &mut [u8], o: usize, v: u32| buf[o..o + 4].copy_from_slice(&v.to_le_bytes());
    w16(&mut buf, 16, 2); // ET_EXEC
    w16(&mut buf, 18, 243); // EM_RISCV
    w32(&mut buf, 20, 1); // e_version
    w32(&mut buf, 24, entry); // e_entry
    w32(&mut buf, 28, EHSZ as u32); // e_phoff
    w16(&mut buf, 40, EHSZ as u16); // e_ehsize
    w16(&mut buf, 42, PHSZ as u16); // e_phentsize
    w16(&mut buf, 44, 1); // e_phnum

    let p = EHSZ;
    w32(&mut buf, p, 1); // PT_LOAD
    w32(&mut buf, p + 4, off_payload as u32); // p_offset
    w32(&mut buf, p + 8, entry); // p_vaddr
    w32(&mut buf, p + 12, entry); // p_paddr
    w32(&mut buf, p + 16, payload.len() as u32); // p_filesz
    w32(&mut buf, p + 20, payload.len() as u32); // p_memsz
    w32(&mut buf, p + 24, 0x5); // R|X
    w32(&mut buf, p + 28, 4); // p_align

    buf[off_payload..].copy_from_slice(payload);
    buf
}

fn prog_bytes() -> Vec<u8> {
    PROGRAM.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// A `MemoryMap` whose only device is a `DRAM` at `BASE`, with a shared handle
/// for post-load inspection.
fn dram_map() -> (MemoryMap, Rc<RefCell<Dram>>) {
    let dram = Rc::new(RefCell::new(
        Dram::new(DRAM_SIZE, DramModel::Ideal).unwrap(),
    ));
    let mut map = MemoryMap::new();
    map.add_device(BASE, DRAM_SIZE, dram.clone()).unwrap();
    (map, dram)
}

#[test]
fn loads_into_dram_fabric_and_executes() {
    let elf = synth_elf(BASE, &prog_bytes());
    let (mut map, dram) = dram_map();

    let entry = load_elf_bytes(&mut map, &elf).unwrap();
    assert_eq!(entry, BASE);

    // The segment bytes landed in DRAM, observed through the device handle.
    assert_eq!(dram.borrow().read(0, Width::Word), Ok(PROGRAM[0]));
    assert_eq!(dram.borrow().read(4, Width::Word), Ok(PROGRAM[1]));

    // Seed PC with the entry point and run the loaded program to its EBREAK.
    let mut core = Core::new(map);
    core.state_mut().pc = entry;
    let halt = core.run_until_halt(|_pkt| {});
    assert_eq!(halt.exit_code, 42); // a0 & 0xFF
}

#[test]
fn loads_from_file_path() {
    let elf = synth_elf(BASE, &prog_bytes());

    // Unique temp path without a dev-dependency.
    let path = std::env::temp_dir().join(format!("iss_elf_test_{}.elf", std::process::id()));
    std::fs::write(&path, &elf).unwrap();

    let (mut map, _dram) = dram_map();
    let result = load_elf(&mut map, &path);
    let _ = std::fs::remove_file(&path); // best-effort cleanup before asserting

    let entry = result.unwrap();
    assert_eq!(entry, BASE);

    let mut core = Core::new(map);
    core.state_mut().pc = entry;
    let halt = core.run_until_halt(|_pkt| {});
    assert_eq!(halt.exit_code, 42);
}
