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
use std::time::Duration;

use hermes::terminal::TerminalBackend;
use iss_core::{load_elf, Core, Soc, SocBuilder};

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
    // Keep the ticker: it shares the UART handle with `map`, so ticking it
    // advances the device the core reads through the bus.
    let Soc { mut map, ticker, .. } = soc;

    let entry = match load_elf(&mut map, &path) {
        Ok(entry) => entry,
        Err(e) => {
            eprintln!("run-eos: failed to load {path}: {e}");
            return ExitCode::from(1);
        }
    };

    let mut core = Core::new(map);
    core.state_mut().pc = entry;

    let mut exit_code: u8 = 0;
    let mut idle: u32 = 0;
    let mut halted = false;
    for _ in 0..MAX_STEPS {
        let pkt = core.step();
        if let Some(h) = pkt.halt {
            exit_code = h.exit_code;
            halted = true;
            break;
        }

        // A UART TX means the guest is producing output (running, rendering);
        // a long TX-less stretch means it is parked in getchar() waiting on us.
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
