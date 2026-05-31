//! [`Core`] — the RV32I_Zicsr functional CPU model and its step FSM.
//!
//! One [`Core::step`] runs `Fetch → Decode → Execute → [barrier] → Commit`. The
//! barrier *between* Execute and Commit is the single trap insertion point:
//!
//! - **(a)** if an async interrupt is pending (`MIE ∧ (mip & mie) ≠ 0`), take an
//!   async trap (priority MEI > MSI > MTI) — this wins over any staged sync
//!   `ECALL`;
//! - **(b)** else if Execute staged a sync trap (`ECALL`), take it;
//! - **(c)** else Commit applies the staging.
//!
//! A trap **bypasses Commit entirely**: all staged effects (GPR/CSR writes,
//! `MRET`, the store buffer, the PC advance) are discarded and only the
//! trap-entry microsequence runs (`mepc ← current PC`, `PC ← mtvec.BASE`). Every
//! step emits a [`CommitPacket`], including halt events, so a cross-verifier can
//! diff the architectural effect before either side stops. Direct port of
//! `core.{h,c}`.

use crate::arch::ArchState;
use crate::bus::{Bus, Width, WidthAlign};
use crate::csr::CsrFile;
use crate::decode::decode;
use crate::execute::{execute, Action, Staging};
use crate::packet::{CommitPacket, CsrSnapshot, HaltEvent, HaltKind, StoreObserved, TrapEvent};

/// A single RV32I_Zicsr hart wired to a [`Bus`].
///
/// Generic over the bus so the hot [`step`](Self::step) path is monomorphised
/// (zero dynamic dispatch); use `Core<&mut dyn Bus>` via the blanket impl in
/// [`crate::bus`] when dynamic dispatch is needed.
pub struct Core<B: Bus> {
    state: ArchState,
    bus: B,
    /// Whether the previous step entered a trap — drives double-trap detection.
    last_step_entered_trap: bool,
}

impl<B: Bus> Core<B> {
    /// Construct a Core wired to `bus`, in reset state (PC 0, GPRs 0, CSRs reset).
    pub fn new(bus: B) -> Self {
        Core {
            state: ArchState::new(),
            bus,
            last_step_entered_trap: false,
        }
    }

    /// Construct a Core wired to `bus` with a pre-seeded architectural `state`.
    pub fn with_state(bus: B, state: ArchState) -> Self {
        Core {
            state,
            bus,
            last_step_entered_trap: false,
        }
    }

    // --- accessors --------------------------------------------------------

    /// Borrow the architectural state (PC + GPRs + CSRs).
    pub fn state(&self) -> &ArchState {
        &self.state
    }

    /// Mutably borrow the architectural state — for test setup / cross-verify.
    pub fn state_mut(&mut self) -> &mut ArchState {
        &mut self.state
    }

    /// Replace the architectural state wholesale (the C `Core_set_arch_state`).
    pub fn set_state(&mut self, state: ArchState) {
        self.state = state;
    }

    /// Borrow the CSR file (e.g. for an external CLINT/IRQ-AGG to drive the
    /// three `mip` wires via [`CsrFile::set_wires`]).
    pub fn csrs_mut(&mut self) -> &mut CsrFile {
        &mut self.state.csrs
    }

    /// Borrow the underlying bus.
    pub fn bus(&self) -> &B {
        &self.bus
    }

    /// Mutably borrow the underlying bus.
    pub fn bus_mut(&mut self) -> &mut B {
        &mut self.bus
    }

    // --- the step FSM -----------------------------------------------------

    /// Execute one architectural step, returning the [`CommitPacket`] that
    /// describes the outcome (normal retirement, trap entry, or halt). The
    /// caller **must** stop stepping when `packet.halt` is `Some`.
    pub fn step(&mut self) -> CommitPacket {
        let pc = self.state.pc;
        let mut pkt = CommitPacket::pending(pc);

        // Double-trap predicate: the previous step trapped AND this step starts
        // at the handler's first instruction (mtvec.BASE).
        let entering_handler = self.last_step_entered_trap && pc == self.state.csrs.mtvec_base();

        match self.fetch_decode_execute(pc) {
            Err(halt) => set_halt(&mut pkt, halt),
            Ok(stg) => self.barrier(pc, stg, entering_handler, &mut pkt),
        }

        // Remember trap state for the next step's double-trap check, then
        // snapshot the post-step CSRs into the packet.
        self.last_step_entered_trap = pkt.trap.is_some();
        pkt.csr_snapshot = CsrSnapshot::capture(&self.state.csrs);
        pkt
    }

    /// Phases 1–3: fetch the word, decode it, execute it. Any fault in these
    /// phases short-circuits to a halt.
    fn fetch_decode_execute(&mut self, pc: u32) -> Result<Staging, HaltKind> {
        let raw = self.fetch(pc)?;
        let uop = decode(raw, pc, &self.state.gpr);
        execute(&self.state, &uop, &mut self.bus)
    }

    /// Phase 1: fetch the 4-byte instruction word at `pc`.
    fn fetch(&mut self, pc: u32) -> Result<u32, HaltKind> {
        if !Width::Word.is_aligned(pc) {
            return Err(HaltKind::MisalignPc);
        }
        self.bus
            .read(pc, Width::Word)
            .map_err(|_| HaltKind::BusErrorIf)
    }

    /// Phase 4: the barrier. Chooses trap entry vs Commit (never both) and
    /// detects double-traps.
    fn barrier(&mut self, pc: u32, stg: Staging, entering_handler: bool, pkt: &mut CommitPacket) {
        // (a) async interrupt wins over a staged sync trap.
        let mut trap_cause = None;
        if self.state.csrs.mie_enabled() {
            let async_cause = self.async_priority();
            if async_cause != 0 {
                trap_cause = Some((1 << 31) | async_cause); // set interrupt bit
            }
        }
        // (b) else a staged sync ECALL.
        if trap_cause.is_none() {
            if let Action::SyncTrap { cause, .. } = stg.action {
                trap_cause = Some(cause);
            }
        }

        match trap_cause {
            Some(cause) => {
                // Double-trap: any in-bound trap on the handler's prologue.
                if entering_handler {
                    set_halt(pkt, HaltKind::DoubleTrap);
                    return;
                }
                self.trap_entry(cause, pc, 0, pkt);
            }
            // (c) commit.
            None => self.commit(stg, pkt),
        }
    }

    /// Phase 5a — Commit: apply staging to architectural state atomically. The
    /// only place a successful-retirement path mutates architectural state.
    fn commit(&mut self, stg: Staging, pkt: &mut CommitPacket) {
        // (1) GPR write (x0 drop handled by write_gpr; packet reports the
        // write only when rd != 0, matching the C convention).
        if stg.rd != 0 {
            self.state.write_gpr(stg.rd, stg.rd_value);
            pkt.rd = stg.rd;
            pkt.rd_value = stg.rd_value;
        }

        // (2) CSR mutation.
        match stg.action {
            Action::Normal => {}
            Action::Zicsr {
                addr,
                write_value,
                writes,
            } => {
                if writes {
                    let ok = self.state.csrs.write(addr, write_value);
                    debug_assert!(
                        ok,
                        "staged Zicsr write rejected — execute should have validated"
                    );
                }
            }
            Action::Mret => self.state.csrs.mret(),
            Action::SyncTrap { .. } => {
                unreachable!("SyncTrap must be handled by the barrier, not Commit")
            }
        }

        // (3) Drain the 1-entry store buffer; a bus error here halts and skips
        // the PC update so the halt packet's PC stays pinned at this instruction.
        if let Some(store) = stg.store {
            match self
                .bus
                .write(store.addr, store.width, store.data, store.strb)
            {
                Ok(()) => {
                    pkt.store = Some(StoreObserved {
                        addr: store.addr,
                        data: store.data,
                        strb: store.strb,
                    });
                }
                Err(_) => {
                    set_halt(pkt, HaltKind::BusErrorSt);
                    return; // skip (4)
                }
            }
        }

        // (4) PC update.
        self.state.pc = stg.new_pc;
    }

    /// Phase 5b — Trap entry: the alternative to Commit. Discards all staged
    /// retirement effects; only the trap-entry microsequence applies.
    fn trap_entry(&mut self, cause: u32, epc: u32, tval: u32, pkt: &mut CommitPacket) {
        self.state.csrs.trap_entry(cause, epc, tval);
        self.state.pc = self.state.csrs.mtvec_base();
        pkt.trap = Some(TrapEvent {
            cause,
            mepc: self.state.csrs.mepc(),
            mtval: self.state.csrs.mtval_raw(),
        });
    }

    /// Priority-encode the highest-priority pending+enabled async source. Spec
    /// cause codes: 11 (MEI) > 3 (MSI) > 7 (MTI); 0 if none.
    fn async_priority(&self) -> u32 {
        let pending = self.state.csrs.mip() & self.state.csrs.mie();
        if pending & (1 << 11) != 0 {
            11
        } else if pending & (1 << 3) != 0 {
            3
        } else if pending & (1 << 7) != 0 {
            7
        } else {
            0
        }
    }

    // --- run loop ---------------------------------------------------------

    /// Step until a halt event, invoking `on_step` with every emitted packet
    /// (the C "emit a packet every step so the caller can diff before exit"
    /// contract), and return the terminating [`HaltEvent`].
    ///
    /// This never returns if the program never halts — see
    /// [`run_until_halt_max`](Self::run_until_halt_max) for a bounded variant.
    pub fn run_until_halt<F: FnMut(&CommitPacket)>(&mut self, mut on_step: F) -> HaltEvent {
        loop {
            let pkt = self.step();
            on_step(&pkt);
            if let Some(halt) = pkt.halt {
                return halt;
            }
        }
    }

    /// Like [`run_until_halt`](Self::run_until_halt) but bounded to `max_steps`.
    /// Returns `Ok(halt)` if the program halted within the budget, or
    /// `Err(steps_run)` if the budget was exhausted first.
    pub fn run_until_halt_max<F: FnMut(&CommitPacket)>(
        &mut self,
        max_steps: usize,
        mut on_step: F,
    ) -> Result<HaltEvent, usize> {
        for n in 0..max_steps {
            let pkt = self.step();
            on_step(&pkt);
            if let Some(halt) = pkt.halt {
                return Ok(halt);
            }
            let _ = n;
        }
        Err(max_steps)
    }
}

/// Record a halt event into the packet (resolving its exit code).
fn set_halt(pkt: &mut CommitPacket, kind: HaltKind) {
    pkt.halt = Some(HaltEvent::new(kind));
}
