//! Bridge integration: a `Core` driven over Hermes's `MemoryMap` fabric.
//!
//! This is the end-to-end proof that the bus/device/AXI reconciliation actually
//! composes — ISS no longer redefines its own `MmioDevice`/`MemoryMap`; it runs
//! the *same* `Core` against Hermes's `Dram` behind `impl Bus for MemoryMap`.
//! A tiny program is staged into DRAM, fetched, and exercises a store + load
//! round-trip through the real fabric before halting via `EBREAK`.

use std::cell::RefCell;
use std::rc::Rc;

use iss_core::{Core, Dram, DramModel, MemoryMap, MmioDevice, Width};

/// DRAM base for this test. The reset PC is 0, so the program lives at the
/// bottom of DRAM and its data scratch sits well above it.
const DRAM_BASE: u32 = 0x0000_0000;
const DRAM_SIZE: u32 = 0x1000;
const DATA_OFF: u32 = 0x100;

/// ```text
///   addi a1, x0, 0x100   ; a1 = data scratch address
///   addi a0, x0, 0x2A    ; a0 = 42
///   sw   a0, 0(a1)       ; mem[0x100] = 42        (store through the fabric)
///   lw   a2, 0(a1)       ; a2 = mem[0x100]        (load back through the fabric)
///   ebreak               ; voluntary halt, exit = a0 & 0xFF = 42
/// ```
const PROGRAM: [u32; 5] = [
    0x1000_0593, // addi a1, x0, 0x100
    0x02A0_0513, // addi a0, x0, 0x2A
    0x00A5_A023, // sw   a0, 0(a1)
    0x0005_A603, // lw   a2, 0(a1)
    0x0010_0073, // ebreak
];

/// Build a `MemoryMap` whose only device is a DRAM staged with `program`,
/// returning both the map and a shared handle for post-run inspection.
fn map_with_program(program: &[u32]) -> (MemoryMap, Rc<RefCell<Dram>>) {
    let mut dram = Dram::new(DRAM_SIZE, DramModel::Ideal).unwrap();
    for (i, word) in program.iter().enumerate() {
        let off = i * 4;
        dram.backing_mut()[off..off + 4].copy_from_slice(&word.to_le_bytes());
    }

    let dram = Rc::new(RefCell::new(dram));
    let mut map = MemoryMap::new();
    map.add_device(DRAM_BASE, DRAM_SIZE, dram.clone()).unwrap();
    (map, dram)
}

#[test]
fn core_runs_a_program_on_hermes_memory_map() {
    let (map, dram) = map_with_program(&PROGRAM);
    let mut core = Core::new(map);

    let halt = core.run_until_halt(|_pkt| {});

    // EBREAK exit code is the low byte of a0 (= 42).
    assert_eq!(halt.exit_code, 42);
    // The load wrote a2 (x12) back from the fabric.
    assert_eq!(core.state().gpr[12], 42);
    // And the store really landed in DRAM, observed through the device handle.
    assert_eq!(dram.borrow().read(DATA_OFF, Width::Word), Ok(42));
}

#[test]
fn unmapped_fetch_through_the_fabric_halts() {
    // A MemoryMap with DRAM only at a high base leaves the reset PC (0) unmapped,
    // so the first fetch is a fabric decode error → BusErrorIf halt (exit 131).
    let mut dram = Dram::new(DRAM_SIZE, DramModel::Ideal).unwrap();
    dram.backing_mut()[0..4].copy_from_slice(&0x0010_0073u32.to_le_bytes()); // ebreak
    let dram = Rc::new(RefCell::new(dram));
    let mut map = MemoryMap::new();
    map.add_device(0x8000_0000, DRAM_SIZE, dram).unwrap();

    let mut core = Core::new(map);
    let halt = core.run_until_halt(|_pkt| {});
    assert_eq!(halt.exit_code, 131);
}
