//! `run-eos` — interactively run an Eos ELF on a full Snake SoC.
//!
//! Wires the UART to the real terminal — RX from the keyboard
//! ([`TerminalBackend`], raw + non-blocking, restored on exit), TX to stdout —
//! then single-steps the core while ticking the SoC each step so received bytes
//! reach the guest. This is what makes the input demos playable:
//!
//! ```text
//!   cargo build -p runtime --release --manifest-path Eos/Cargo.toml --bin snake
//!   cargo run --bin run-eos -- Eos/target/riscv32i-unknown-none-elf/release/snake
//! ```
//!
//! Any Eos binary works (e.g. `hello`, `sorts`); only the input demos actually
//! use the keyboard. The process exits with the guest's halt code (`a0`).

use std::env;
use std::io::{self, Write};
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant};

use hermes::terminal::TerminalBackend;
use iss_core::{load_elf, Core, Soc, SocBuilder, Width};

/// Wraps a writer and expands `\n` into `\r\n`.
///
/// `TerminalBackend` puts the terminal in raw mode, which disables the kernel's
/// `ONLCR` newline translation, so the guest's bare line feeds would otherwise
/// stair-step (drop a line without returning to column 0). Re-adding the
/// carriage return on the way out restores normal newline behaviour for the
/// UART TX stream without the guest having to know about terminals.
struct CrlfWriter<W: Write>(W);

impl<W: Write> Write for CrlfWriter<W> {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut tmp = Vec::with_capacity(buf.len() + 4);
        for &b in buf {
            if b == b'\n' {
                tmp.push(b'\r');
            }
            tmp.push(b);
        }
        self.0.write_all(&tmp)?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        self.0.flush()
    }
}

/// UART TX register address (Snake SoC). A store here is the guest emitting a
/// byte — used to tell "rendering" apart from "spin-waiting for input".
const UART_TX: u32 = 0x1000_1000;
/// Steps without any UART TX after which the guest is assumed to be blocked in
/// `getchar()`; we then poll the keyboard gently instead of busy-spinning.
const IDLE_STEPS: u32 = 5_000;
/// How often to resync `mtime` to the wall clock (in steps). Frequent enough
/// for sub-millisecond timer resolution, rare enough to be free for compute.
const MTIME_SYNC_STEPS: u32 = 1_024;
/// CLINT `mtime` low/high register addresses on the Snake SoC.
const CLINT_MTIME_LO: u32 = 0x0200_0000 + 0xBFF8;
const CLINT_MTIME_HI: u32 = 0x0200_0000 + 0xBFFC;
/// Hard bound so a runaway guest can't spin forever.
const MAX_STEPS: u64 = 5_000_000_000;

fn main() -> ExitCode {
    let mut args = env::args().skip(1);
    let Some(path) = args.next() else {
        eprintln!("usage: run-eos <eos-elf>");
        eprintln!("  e.g. run-eos Eos/target/riscv32i-unknown-none-elf/release/snake");
        return ExitCode::from(2);
    };

    // RX from the real terminal; TX to stdout with raw-mode-safe newlines.
    let soc: Soc = SocBuilder::new()
        .uart_rx(Box::new(TerminalBackend::new()))
        .uart_tx(Box::new(CrlfWriter(io::stdout())))
        .build()
        .expect("failed to wire SoC");
    // Keep the ticker and the CLINT/IRQ handles: the ticker shares the device
    // handles with `map`, and the handles let us bridge the interrupt wires into
    // the core (UART RX is delivered as a machine-external interrupt).
    let Soc {
        mut map,
        ticker,
        clint,
        irq,
        ..
    } = soc;

    let entry = match load_elf(&mut map, &path) {
        Ok(entry) => entry,
        Err(e) => {
            eprintln!("run-eos: failed to load {path}: {e}");
            return ExitCode::from(1);
        }
    };

    let mut core = Core::new(map);
    core.state_mut().pc = entry;

    let start = Instant::now();
    let mut exit_code: u8 = 0;
    let mut idle: u32 = 0;
    let mut since_sync: u32 = 0;
    let mut halted = false;
    for _ in 0..MAX_STEPS {
        let pkt = core.step();
        if let Some(h) = pkt.halt {
            exit_code = h.exit_code;
            halted = true;
            break;
        }

        // Tie `mtime` to the host wall clock so timer-based guests (snake_rt)
        // run in real time on any machine. Compute-only guests never read
        // `mtime`, so this just costs an occasional clock read for them.
        since_sync += 1;
        if since_sync >= MTIME_SYNC_STEPS {
            since_sync = 0;
            let us = start.elapsed().as_micros() as u64;
            let _ = core
                .bus_mut()
                .write(CLINT_MTIME_LO, Width::Word, us as u32, 0xF);
            let _ = core
                .bus_mut()
                .write(CLINT_MTIME_HI, Width::Word, (us >> 32) as u32, 0xF);
        }

        // A UART TX means the guest is producing output (running, rendering);
        // a long TX-less stretch means it is parked waiting for input.
        if pkt.store.map(|s| s.addr) == Some(UART_TX) {
            idle = 0;
        } else {
            idle += 1;
            if idle >= IDLE_STEPS {
                idle = 0;
                thread::sleep(Duration::from_millis(2));
            }
        }

        ticker.tick_all(); // latch the next keyboard byte once RX drains
                           // Bridge the device interrupt wires into the core so a received byte is
                           // delivered to the guest as a machine-external interrupt.
        core.csrs_mut().set_wires(
            clint.borrow().msip(),
            clint.borrow().mtip(),
            irq.borrow().meip(),
        );
    }

    // Drop the SoC (core + ticker) so TerminalBackend restores the terminal
    // *before* we print the exit line.
    drop(core);
    drop(ticker);

    if halted {
        eprintln!("\n[run-eos] guest halted, exit code {exit_code}");
        ExitCode::from(exit_code)
    } else {
        eprintln!("\n[run-eos] step bound reached without halt");
        ExitCode::from(1)
    }
}
