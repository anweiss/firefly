//! Minimal Pro DJ Link receiver — Beat packets (port 50001) and CDJ
//! Status packets (port 50002).
//!
//! We deliberately do NOT use `prodjlink-rs` here: it pulls in a tokio
//! multi-thread runtime, `if-addrs`, and other deps that need patching
//! to cross-compile to ESP-IDF. The on-device coordinator only needs
//! two packet types for the beat-clock state machine, and they're
//! straightforward to parse inline.
//!
//! Packet layout (Beat / CDJ Status share the same envelope):
//! - bytes 0..10 : magic `Qspt1WmJOL` (0x51, 0x73, 0x70, 0x74, 0x31,
//!                                     0x57, 0x6D, 0x4A, 0x4F, 0x4C)
//! - byte 10     : packet kind (0x28=Beat, 0x0A=CDJ status)
//! - bytes 11..31: device name (ASCIIZ, 20 bytes)
//! - byte 31     : sub-type / length-class (Beat=0x01)
//! - byte 32     : reporting device number (1..=6)
//!
//! For Beat packets (kind 0x28):
//! - bytes 84..86 : tempo (big-endian u16, BPM × 100)
//! - byte  92     : beat_within_bar (1..=4)
//!
//! For CDJ Status packets (kind 0x0A):
//! - byte 0x7B (123): play state byte. Bit 0x40 set = "playing" (forward).

use crate::beat_state::{Beat, CdjStatus};
use anyhow::{anyhow, Result};
use log::{debug, info, warn};
use std::net::UdpSocket;
use std::sync::mpsc::Sender;
use std::time::Duration;

const MAGIC: [u8; 10] = [0x51, 0x73, 0x70, 0x74, 0x31, 0x57, 0x6D, 0x4A, 0x4F, 0x4C];
const PORT_BEAT: u16 = 50001;
const PORT_STATUS: u16 = 50002;

const KIND_BEAT: u8 = 0x28;
const KIND_CDJ_STATUS: u8 = 0x0A;

/// Events emitted by the DJ Link receiver into the main loop.
#[derive(Debug, Clone, Copy)]
pub enum DjLinkEvent {
    Beat(Beat),
    Status(CdjStatus),
}

fn parse_beat(buf: &[u8]) -> Option<Beat> {
    if buf.len() < 96 || buf[0..10] != MAGIC || buf[10] != KIND_BEAT {
        return None;
    }
    let device_number = buf[33];
    let beat_within_bar = buf[92];
    let tempo_x100 = u16::from_be_bytes([buf[84], buf[85]]);
    let tempo = tempo_x100 as f64 / 100.0;
    Some(Beat {
        device_number,
        beat_within_bar,
        effective_tempo_bpm: tempo,
        // The deck does report next_beat / next_bar millisecond offsets
        // earlier in the packet, but the coordinator's PLL recomputes
        // them from the smoothed beat timestamp + tempo. We only fall
        // back to these when tempo is unknown, which doesn't happen
        // here. Leave None.
        next_beat_ms: None,
        next_bar_ms: None,
    })
}

fn parse_status(buf: &[u8]) -> Option<CdjStatus> {
    if buf.len() < 0x80 || buf[0..10] != MAGIC || buf[10] != KIND_CDJ_STATUS {
        return None;
    }
    let device_number = buf[33];
    let play_byte = buf[0x7B];
    let is_playing = (play_byte & 0x40) != 0;
    Some(CdjStatus {
        device_number,
        is_playing,
    })
}

/// Run the DJ Link receive loop on the current thread. Spawns nothing;
/// the caller is expected to put us on a dedicated thread.
pub fn run(tx: Sender<DjLinkEvent>) -> Result<()> {
    let beat_sock = UdpSocket::bind(("0.0.0.0", PORT_BEAT))
        .map_err(|e| anyhow!("bind beat port {}: {}", PORT_BEAT, e))?;
    let status_sock = UdpSocket::bind(("0.0.0.0", PORT_STATUS))
        .map_err(|e| anyhow!("bind status port {}: {}", PORT_STATUS, e))?;
    beat_sock.set_read_timeout(Some(Duration::from_millis(50)))?;
    status_sock.set_read_timeout(Some(Duration::from_millis(50)))?;

    info!(
        "DJ Link: listening on UDP {} (beat) and {} (status)",
        PORT_BEAT, PORT_STATUS
    );

    let mut buf = [0u8; 1500];
    loop {
        // Drain Beat socket (don't block more than the read timeout).
        match beat_sock.recv_from(&mut buf) {
            Ok((n, _src)) => {
                if let Some(beat) = parse_beat(&buf[..n]) {
                    debug!(
                        "DJ Link beat: dev={} bib={} bpm={:.2}",
                        beat.device_number, beat.beat_within_bar, beat.effective_tempo_bpm
                    );
                    let _ = tx.send(DjLinkEvent::Beat(beat));
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => warn!("DJ Link beat recv error: {}", e),
        }

        match status_sock.recv_from(&mut buf) {
            Ok((n, _src)) => {
                if let Some(status) = parse_status(&buf[..n]) {
                    let _ = tx.send(DjLinkEvent::Status(status));
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => warn!("DJ Link status recv error: {}", e),
        }
    }
}
