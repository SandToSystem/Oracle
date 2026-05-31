//! # iss-core — RV32I_Zicsr instruction-set-simulator Core
//!
//! A pure functional model of one RISC-V **RV32I_Zicsr Machine-mode hart**,
//! rewritten in idiomatic Rust from the C `libiss` in `comporg-labs/iss`.
//!
//! One [`Core::step`] runs the canonical pipeline
//!
//! ```text
//! Fetch → Decode → Execute → [barrier] → Commit
//! ```
//!
//! and returns a [`CommitPacket`] describing the architectural footprint of
//! that step — a normal retirement (GPR write and/or store), a synchronous
//! trap entry (ECALL), or a halt event (voluntary `EBREAK` or an involuntary
//! fault). The barrier *between* Execute and Commit is the single trap
//! insertion point: an asynchronous interrupt or a staged sync trap routes to
//! trap entry and bypasses Commit entirely.
//!
//! The Core is generic over a [`Bus`] ([`Core<B: Bus>`](Core)); it knows nothing
//! about concrete devices. Two bus implementations ship with the crate:
//! [`RamBus`] (a flat little-endian memory, ideal for tests) and [`DeviceBus`]
//! (an address-range dispatcher composed of [`MmioDevice`] trait objects,
//! porting the C `MemoryMap`/`MmioDevice` pair).
//!
//! ## Quick start
//!
//! ```
//! use iss_core::{Core, RamBus, Bus, Width};
//!
//! // `ebreak` (0x00100073) at address 0 with a0 (x10) = 0x2A → exit code 42.
//! let mut bus = RamBus::new(0, 0x1000);
//! bus.write(0, Width::Word, 0x0010_0073, 0xF).unwrap();
//!
//! let mut core = Core::new(bus);
//! core.state_mut().gpr[10] = 0x2A;
//!
//! let halt = core.run_until_halt(|_pkt| { /* observe each retired step */ });
//! assert_eq!(halt.exit_code, 42);
//! ```
//!
//! ## Module map
//! - [`bus`] — the [`Bus`] trait, [`AxiResp`], [`Width`], and [`RamBus`].
//! - [`device`] — [`MmioDevice`] trait + [`DeviceBus`] range dispatcher.
//! - [`csr`] — the M-mode [`CsrFile`].
//! - [`arch`] — [`ArchState`] (PC + 32 GPRs + CSR file).
//! - [`decode`] — instruction decode into a [`Uop`](decode::Uop).
//! - [`packet`] — the [`CommitPacket`] verification protocol.

pub mod alu;
pub mod arch;
pub mod bus;
pub mod core;
pub mod csr;
pub mod decode;
pub mod device;
pub mod execute;
pub mod packet;

pub use arch::ArchState;
pub use bus::{AxiResp, Bus, RamBus, Width};
pub use core::Core;
pub use csr::CsrFile;
pub use decode::{decode, Uop};
pub use device::{AddError, DeviceBus, MmioDevice, Ram};
pub use packet::{CommitPacket, HaltEvent, HaltKind, StoreObserved, TrapEvent};
