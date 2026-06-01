# iss-core — RV32I_Zicsr ISS Core (Rust)

A pure functional model of one RISC-V **RV32I_Zicsr Machine-mode hart**, rewritten
in idiomatic Rust from the C `libiss` in `comporg-labs/iss`. One `Core::step()`
runs the canonical pipeline

```text
Fetch → Decode → Execute → [barrier] → Commit
```

and returns a `CommitPacket` describing the architectural footprint of that step
— a normal retirement (GPR write and/or store), a synchronous trap entry
(`ECALL`), or a halt event (voluntary `EBREAK` or an involuntary fault). The
barrier *between* Execute and Commit is the single trap insertion point: an
asynchronous interrupt or a staged sync trap routes to trap entry and bypasses
Commit entirely.

The Core is generic over a `Bus`; it knows nothing about concrete devices.

## Layout

The device/fabric layer (`MmioDevice`, the address-decoding `MemoryMap`, and
concrete devices such as `Dram`) plus the shared AXI vocabulary (`AxiResp`,
`Slverr`, `Width`, `le_load`/`le_store`) live in the **Hermes** submodule and are
re-exported here, rather than redefined. ISS keeps only the CPU-side seam.

```
src/
├── lib.rs        crate docs + re-exports (Hermes device layer re-exported here)
├── bus.rs        Bus trait, RamBus (flat memory), WidthAlign ext,
│                 impl Bus for hermes::MemoryMap; AxiResp/Slverr/Width re-exported
├── csr.rs        CsrFile — M-mode CSR file (7 storage CSRs + mip wires)
├── arch.rs       ArchState (PC + 32 GPRs + CSR file)
├── alu.rs        AluOp / BranchType evaluators
├── decode/       raw bit-field extractors + control-signal enums + decode()
├── execute.rs    Action / Staging — Execute produces, never mutates state
├── loader.rs     load_elf / load_elf_bytes — parse an ELF32 RISC-V exe and
│                 stage its PT_LOAD segments into any Bus (port of elf_loader.c)
├── packet.rs     CommitPacket / HaltKind / TrapEvent (the verify protocol)
└── core.rs       Core<B: Bus> — step / barrier / commit / trap / run loop
tests/
├── decode.rs           per-format decode checks (port of test_decode.c)
├── core_integration.rs programs run on RamBus, asserting the packet stream
├── hermes_bridge.rs    a Core driven over Hermes's MemoryMap + Dram fabric
└── elf_loader.rs       load an ELF into the DRAM fabric, then run it to halt
```

## Public API

```rust
use iss_core::{Core, RamBus, Bus, Width};

let mut bus = RamBus::new(0, 0x1000);
bus.write(0, Width::Word, 0x0010_0073, 0xF).unwrap(); // ebreak

let mut core = Core::new(bus);
core.state_mut().gpr[10] = 0x2A;                       // a0 = 42

// Observe every retired step; the closure is the cross-verify hook.
let halt = core.run_until_halt(|_pkt| {});
assert_eq!(halt.exit_code, 42);                        // EBREAK exit = a0 & 0xFF
```

Key entry points:

| Item | Purpose |
|---|---|
| `Core::new(bus)` / `Core::with_state(bus, state)` | construct a hart |
| `Core::step() -> CommitPacket` | one architectural step |
| `Core::run_until_halt(\|pkt\| …) -> HaltEvent` | step to halt with a per-step callback |
| `Core::run_until_halt_max(n, …)` | bounded variant (`Err(n)` if it never halts) |
| `Core::state()` / `state_mut()` / `set_state()` | read/seed architectural state |
| `Core::csrs_mut().set_wires(msip, mtip, meip)` | drive the three `mip` interrupt wires |
| `Bus` | the memory abstraction (`RamBus`, or Hermes's `MemoryMap` of `MmioDevice`s) |

## Loading ELF programs

Instead of hand-staging word arrays, parse a real ELF32 RISC-V executable and
write its `PT_LOAD` segments into any [`Bus`] (`RamBus` or the Hermes
`MemoryMap` fabric). The entry point is returned for the caller to seed `pc`.

```rust
use iss_core::{load_elf_bytes, Core, RamBus, Bus, Width};

let mut bus = RamBus::new(0x8000_0000, 0x1000);
let entry = load_elf_bytes(&mut bus, &elf_image)?;   // elf_image: &[u8]

let mut core = Core::new(bus);
core.state_mut().pc = entry;                          // start at e_entry
let halt = core.run_until_halt(|_pkt| {});
# Ok::<(), iss_core::ElfError>(())
```

`load_elf(&mut bus, path)` is the file-path wrapper. Both validate
`ELFCLASS32` / `EM_RISCV` / `ET_EXEC` and return [`ElfError`] on a bad image or
a segment that lands outside the bus. The loader does **not** zero
`p_memsz - p_filesz` (that is crt0's `.bss` job).

## Halt exit codes

| Kind | Exit |
|---|---|
| `Voluntary` (`EBREAK`) | `a0 & 0xFF` |
| `Illegal` | 130 |
| `BusErrorIf` / `Ld` / `St` | 131 / 132 / 133 |
| `MisalignPc` / `Ld` / `St` | 134 |
| `DoubleTrap` | 135 |

## Rust features showcased

- **Traits + generics:** `Core<B: Bus>` is monomorphised over the bus (zero
  dispatch on the hot path); a blanket `impl Bus for &mut B` keeps a `dyn`
  escape hatch. The same `Core` runs on `RamBus` for fast tests or on Hermes's
  `MemoryMap` fabric (`MmioDevice`s by address range) via `impl Bus for MemoryMap`.
- **Rich enums + pattern matching:** control signals (`AluOp`, `SystemKind`,
  `CsrSource`, `Action`, `HaltKind`) carry their data so illegal states are
  unrepresentable — e.g. `CsrSource::Imm(u32)` folds away a separate boolean.
- **`Option` over flags:** `Uop.mem: Option<MemOp>` / `branch: Option<BranchType>`
  and `CommitPacket.{store,halt,trap}: Option<_>` replace the C boolean pairs.
- **Closures:** `run_until_halt(FnMut(&CommitPacket))` models the C "emit a
  packet every step so the caller can diff before exit" contract.
- **OOP-style encapsulation:** each subsystem owns its invariants behind methods
  (`CsrFile`, `ArchState::write_gpr` for the `x0` hardwire, etc.).

## Build / test

```bash
cargo build
cargo test                          # unit + decode + integration + doctest + riscv-tests
cargo clippy --all-targets -- -D warnings
cargo fmt --check
```

## riscv-tests

The official RISC-V ISA suite runs end-to-end on the ISS. The `riscv-tests/`
submodule supplies the inputs:

- `riscv-tests/isa/rv32ui/*.S` — the upstream test bodies, unmodified.
- `riscv-tests/env/` — its own env submodule (the SandToSystem fork of
  `riscv-test-env`), carrying our `snake/` shim (`riscv_test.h` + `link.ld`) that
  signals pass/fail with **`EBREAK`** (`a0` = exit code, `0` = pass) instead of
  spike-style `tohost` polling — matching the ISS `HaltKind::Voluntary` halt.

At build time `build.rs` cross-compiles the allow-listed tests
(`-march=rv32i_zicsr -mabi=ilp32`, linked at DRAM base `0x8000_0000`) against the
`snake` env and generates one `#[test]` per ELF for `tests/riscv_tests.rs`, which
loads each image via [`load_elf`], seeds `pc = e_entry`, runs to halt, and asserts
`exit_code == 0` (a failure decodes to the failing test-case number).

Currently wired: 40 `rv32ui-p-*` (all RV32I integer; `fence_i`/`ma_data` excluded
as out-of-ISA) and 3 `rv32mi-p-*` (`csr` / `scall` / `mcsr` — the ECALL-only trap
subset). The harness is best-effort: if the RISC-V toolchain
(`riscv64-unknown-elf-gcc`, found on `PATH` or `/opt/riscv/bin`) or the submodules
are absent, the build still succeeds and the suite runs zero tests.

```bash
git submodule update --init --recursive   # fetch riscv-tests (+ its nested env)
cargo test --test riscv_tests              # run just the ISA suite
```
