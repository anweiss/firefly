use std::collections::HashMap;
use std::io::Write;
use std::net::Ipv4Addr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use ableton_link_rs::link::{BasicLink, SessionState};
use clap::Parser;
use crc::{Crc, CRC_8_MAXIM_DOW};
use prodjlink_rs::{BeatEvent, ProDjLink};
use tracing::{debug, error, info, warn};

// ── Protocol v2 constants ───────────────────────────────────────────

const SYNC: [u8; 2] = [0xBE, 0xA7];
const VERSION: u8 = 0x02;
const TOTAL_LEN: u8 = 36;
const PACKET_SIZE: usize = 36;
const CRC8: Crc<u8> = Crc::<u8>::new(&CRC_8_MAXIM_DOW);

// Flag bits
const FLAG_PLAYING: u8 = 1 << 0;
const FLAG_CDJ_ACTIVE: u8 = 1 << 1;

/// If no beat arrives within this window, fall back to Link-only mode.
const CDJ_TIMEOUT: Duration = Duration::from_secs(2);

/// Minimum BPM difference before we propagate a tempo change to Link.
const TEMPO_EPSILON: f64 = 0.05;

// ── CLI ─────────────────────────────────────────────────────────────

#[derive(Parser, Debug)]
#[command(name = "firefly-coordinator")]
#[command(about = "DJ Link + Ableton Link → ESP-NOW bridge for Firefly wristbands")]
struct Args {
    /// Serial port path (e.g. /dev/cu.usbmodem1101)
    #[arg(short, long)]
    port: String,

    /// Serial baud rate
    #[arg(short, long, default_value_t = 115200)]
    baud: u32,

    /// Initial BPM (used when no Link peers or CDJs are connected)
    #[arg(long, default_value_t = 120.0)]
    bpm: f64,

    /// Beats per bar (quantum)
    #[arg(short, long, default_value_t = 4.0)]
    quantum: f64,

    /// Broadcast rate in Hz
    #[arg(long, default_value_t = 20)]
    rate: u32,

    /// Network interface IP for DJ Link (e.g. 192.168.1.145).
    /// If omitted, auto-detects the first non-loopback interface.
    #[arg(short, long)]
    interface: Option<Ipv4Addr>,

    /// Virtual CDJ device number on the DJ Link network (1–6)
    #[arg(short, long, default_value_t = 5)]
    device_number: u8,

    /// Disable DJ Link integration (Link-only mode)
    #[arg(long, default_value_t = false)]
    no_djlink: bool,
}

// ── Packet serialization (v2) ───────────────────────────────────────

fn build_packet(
    send_time_us: i64,
    next_downbeat_us: i64,
    tempo_bpm_x100: u16,
    beat_in_bar: u8,
    flags: u8,
    next_beat_us: i64,
    on_air_mask: u8,
    master_device: u8,
) -> [u8; PACKET_SIZE] {
    let mut buf = [0u8; PACKET_SIZE];

    // Header
    buf[0] = SYNC[0];
    buf[1] = SYNC[1];
    buf[2] = VERSION;
    buf[3] = TOTAL_LEN;

    // v1 fields — field-by-field little-endian
    buf[4..12].copy_from_slice(&send_time_us.to_le_bytes());
    buf[12..20].copy_from_slice(&next_downbeat_us.to_le_bytes());
    buf[20..22].copy_from_slice(&tempo_bpm_x100.to_le_bytes());
    buf[22] = beat_in_bar;
    buf[23] = flags;

    // v2 fields
    buf[24..32].copy_from_slice(&next_beat_us.to_le_bytes());
    buf[32] = on_air_mask;
    buf[33] = master_device;
    buf[34] = 0; // reserved

    // CRC over bytes [2..35)
    buf[35] = CRC8.checksum(&buf[2..35]);

    buf
}

// ── Compute next downbeat time from Link ────────────────────────────

fn next_downbeat_time(session: &SessionState, now: chrono::Duration, quantum: f64) -> chrono::Duration {
    let phase = session.phase_at_time(now, quantum);
    let epsilon = 0.001;
    let beats_until_downbeat = if phase <= epsilon {
        quantum
    } else {
        quantum - phase
    };

    let current_beat = session.beat_at_time(now, quantum);
    let next_db_beat = current_beat + beats_until_downbeat;
    session.time_at_beat(next_db_beat, quantum)
}

// ── On-air bitmask helper ───────────────────────────────────────────

fn on_air_to_mask(channels: &HashMap<u8, bool>) -> u8 {
    let mut mask: u8 = 0;
    for (&ch, &active) in channels {
        if active && ch >= 1 && ch <= 8 {
            mask |= 1 << (ch - 1);
        }
    }
    mask
}

// ── Main ────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    let args = Args::parse();
    let running = Arc::new(AtomicBool::new(true));

    // Ctrl-C handler
    {
        let running = running.clone();
        ctrlc::set_handler(move || {
            running.store(false, Ordering::SeqCst);
        })?;
    }

    // Open serial port
    info!(port = %args.port, baud = args.baud, "opening serial port");
    let mut port = serialport::new(&args.port, args.baud)
        .timeout(Duration::from_millis(100))
        .open()?;
    port.clear(serialport::ClearBuffer::All)?;

    // Initialize Ableton Link
    info!(bpm = args.bpm, quantum = args.quantum, "starting Ableton Link");
    let mut link = BasicLink::new(args.bpm).await;
    link.enable().await;

    link.set_num_peers_callback(|count| {
        info!(peers = count, "Link peer count changed");
    });
    link.set_tempo_callback(|bpm| {
        info!(bpm = format!("{:.2}", bpm), "Link tempo changed");
    });

    // Initialize DJ Link (optional)
    let pdl = if !args.no_djlink {
        info!(
            device_number = args.device_number,
            interface = ?args.interface,
            "starting DJ Link"
        );
        let mut builder = ProDjLink::builder()
            .device_number(args.device_number);
        if let Some(ip) = args.interface {
            builder = builder.interface_address(ip);
        }
        match builder.build().await {
            Ok(pdl) => {
                pdl.virtual_cdj().set_auto_negotiate(true).await;
                pdl.virtual_cdj().set_sending_status(true).await;
                info!("DJ Link active — device #{}", args.device_number);
                Some(pdl)
            }
            Err(e) => {
                warn!(error = %e, "DJ Link init failed — running Link-only");
                None
            }
        }
    } else {
        info!("DJ Link disabled — running Link-only");
        None
    };

    // Subscribe to DJ Link events
    let mut beats_rx = pdl.as_ref().map(|p| p.subscribe_beats());
    let mut on_air_rx = pdl.as_ref().map(|p| p.subscribe_on_air());

    // Beat source state
    let mut last_cdj_beat_time = Instant::now() - CDJ_TIMEOUT;
    let mut cdj_beat_in_bar: u8 = 0;
    let mut cdj_next_beat_us: i64 = 0;
    let mut cdj_next_bar_us: i64 = 0;
    let mut cdj_bpm: f64 = 0.0;
    let mut cdj_playing = false;
    let mut master_device: u8 = 0;
    let mut channels_on_air: HashMap<u8, bool> = HashMap::new();

    let mut last_beat_in_bar: u8 = u8::MAX;
    let mut bar_counter: u8 = 0;

    info!("coordinator running — broadcasting v2 packets at {}Hz", args.rate);

    let interval = Duration::from_micros(1_000_000 / args.rate as u64);
    let quantum = args.quantum;

    loop {
        if !running.load(Ordering::SeqCst) {
            break;
        }

        // ── Drain DJ Link events (non-blocking) ────────────────────
        if let Some(ref mut rx) = beats_rx {
            while let Ok(event) = rx.try_recv() {
                if let BeatEvent::NewBeat(beat) = event {
                    // Only use beats from the master player
                    let is_master = pdl.as_ref()
                        .map(|p| p.virtual_cdj().tempo_master().master_device())
                        .flatten()
                        .map_or(false, |d| d == beat.device_number);

                    if is_master {
                        last_cdj_beat_time = Instant::now();
                        cdj_bpm = beat.effective_tempo();
                        // beat_within_bar is 1-based from CDJ, convert to 0-based
                        cdj_beat_in_bar = if beat.beat_within_bar > 0 {
                            beat.beat_within_bar - 1
                        } else {
                            0
                        };
                        cdj_playing = true;
                        master_device = beat.device_number.0;

                        // Use CDJ's own timing predictions (ms from now)
                        let now_us = link.clock().micros()
                            .num_microseconds()
                            .unwrap_or(0);

                        cdj_next_beat_us = beat.next_beat
                            .map(|ms| now_us + (ms as i64) * 1000)
                            .unwrap_or(0);

                        cdj_next_bar_us = beat.next_bar
                            .map(|ms| now_us + (ms as i64) * 1000)
                            .unwrap_or(0);

                        // Bridge CDJ tempo → Link
                        let link_bpm = link.capture_app_session_state().tempo();
                        if (link_bpm - cdj_bpm).abs() > TEMPO_EPSILON {
                            let time = link.clock().micros();
                            let mut session = link.capture_app_session_state();
                            session.set_tempo(cdj_bpm, time);
                            link.commit_app_session_state(session).await;
                            debug!(cdj_bpm, "bridged CDJ tempo → Link");
                        }

                        // Bridge play state → Link
                        let link_playing = link.capture_app_session_state().is_playing();
                        if !link_playing {
                            let time = link.clock().micros();
                            let mut session = link.capture_app_session_state();
                            session.set_is_playing(true, time);
                            link.commit_app_session_state(session).await;
                        }
                    }
                }
            }
        }

        if let Some(ref mut rx) = on_air_rx {
            while let Ok(on_air) = rx.try_recv() {
                channels_on_air = on_air.channels.iter().map(|(&k, &v)| (k, v)).collect();
                let active: Vec<u8> = channels_on_air.iter()
                    .filter(|(_, v)| **v)
                    .map(|(&k, _)| k)
                    .collect();
                debug!(on_air = ?active, "channels on-air updated");
            }
        }

        // ── CDJ timeout detection ──────────────────────────────────
        if cdj_playing && last_cdj_beat_time.elapsed() > CDJ_TIMEOUT {
            cdj_playing = false;
            info!("CDJ beat timeout — falling back to Link-only");
            let time = link.clock().micros();
            let mut session = link.capture_app_session_state();
            session.set_is_playing(false, time);
            link.commit_app_session_state(session).await;
        }

        // ── Build packet ───────────────────────────────────────────
        let now = link.clock().micros();
        let session = link.capture_app_session_state();
        let cdj_active = cdj_playing && last_cdj_beat_time.elapsed() < CDJ_TIMEOUT;

        let (tempo, beat_in_bar, next_db_us, next_bt_us, is_playing) = if cdj_active {
            // CDJ is authoritative
            (cdj_bpm, cdj_beat_in_bar, cdj_next_bar_us, cdj_next_beat_us, true)
        } else {
            // Link-only fallback
            let phase = session.phase_at_time(now, quantum);
            let beat = phase.floor() as u8;
            let next_db = next_downbeat_time(&session, now, quantum);
            let next_db_us = next_db.num_microseconds().unwrap_or(0);
            // Approximate next beat from Link
            let beats_until_next = 1.0 - (phase - phase.floor());
            let current_beat = session.beat_at_time(now, quantum);
            let next_beat_time = session.time_at_beat(current_beat + beats_until_next, quantum);
            let next_bt_us = next_beat_time.num_microseconds().unwrap_or(0);
            (session.tempo(), beat, next_db_us, next_bt_us, session.is_playing())
        };

        // Track bar transitions
        if beat_in_bar == 0 && last_beat_in_bar != 0 && last_beat_in_bar != u8::MAX {
            bar_counter = bar_counter.wrapping_add(1);
        }
        last_beat_in_bar = beat_in_bar;

        let send_time_us = now.num_microseconds().expect("coordinator clock overflow");

        let mut flags: u8 = 0;
        if is_playing { flags |= FLAG_PLAYING; }
        if cdj_active { flags |= FLAG_CDJ_ACTIVE; }

        let on_air_mask = on_air_to_mask(&channels_on_air);
        let master_dev = if cdj_active { master_device } else { 0 };

        let packet = build_packet(
            send_time_us,
            next_db_us,
            (tempo * 100.0) as u16,
            beat_in_bar,
            flags,
            next_bt_us,
            on_air_mask,
            master_dev,
        );

        match port.write_all(&packet) {
            Ok(()) => {
                debug!(
                    tempo = format!("{:.2}", tempo),
                    beat_in_bar,
                    bar = bar_counter,
                    cdj_active,
                    master = master_dev,
                    "sent v2 packet"
                );
            }
            Err(e) => {
                warn!(error = %e, "serial write failed — dongle disconnected?");
                tokio::time::sleep(Duration::from_secs(1)).await;
                match serialport::new(&args.port, args.baud)
                    .timeout(Duration::from_millis(100))
                    .open()
                {
                    Ok(new_port) => {
                        port = new_port;
                        let _ = port.clear(serialport::ClearBuffer::All);
                        info!("serial port reconnected");
                    }
                    Err(e) => {
                        error!(error = %e, "serial reconnect failed");
                    }
                }
            }
        }

        tokio::time::sleep(interval).await;
    }

    info!("shutting down");
    link.disable().await;
    if let Some(pdl) = pdl {
        pdl.shutdown();
    }
    Ok(())
}

// ── Tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_packet_v2_header() {
        let pkt = build_packet(1000, 2000, 12000, 0, FLAG_PLAYING, 1500, 0b0000_0101, 3);
        assert_eq!(pkt[0], 0xBE);
        assert_eq!(pkt[1], 0xA7);
        assert_eq!(pkt[2], 0x02);
        assert_eq!(pkt[3], 36);
    }

    #[test]
    fn build_packet_v2_fields_round_trip() {
        let send_time: i64 = 123_456_789;
        let next_db: i64 = 234_567_890;
        let next_bt: i64 = 150_000_000;
        let pkt = build_packet(send_time, next_db, 12630, 2, FLAG_PLAYING | FLAG_CDJ_ACTIVE, next_bt, 0b0011, 1);

        // Parse back
        let parsed_send = i64::from_le_bytes(pkt[4..12].try_into().unwrap());
        let parsed_db = i64::from_le_bytes(pkt[12..20].try_into().unwrap());
        let parsed_tempo = u16::from_le_bytes(pkt[20..22].try_into().unwrap());
        let parsed_beat = pkt[22];
        let parsed_flags = pkt[23];
        let parsed_next_bt = i64::from_le_bytes(pkt[24..32].try_into().unwrap());
        let parsed_on_air = pkt[32];
        let parsed_master = pkt[33];

        assert_eq!(parsed_send, send_time);
        assert_eq!(parsed_db, next_db);
        assert_eq!(parsed_tempo, 12630);
        assert_eq!(parsed_beat, 2);
        assert_eq!(parsed_flags, FLAG_PLAYING | FLAG_CDJ_ACTIVE);
        assert_eq!(parsed_next_bt, next_bt);
        assert_eq!(parsed_on_air, 0b0011);
        assert_eq!(parsed_master, 1);
    }

    #[test]
    fn build_packet_v2_crc_validates() {
        let pkt = build_packet(0, 0, 12000, 0, 0, 0, 0, 0);
        let expected_crc = CRC8.checksum(&pkt[2..35]);
        assert_eq!(pkt[35], expected_crc);
    }

    #[test]
    fn on_air_mask_channels() {
        let mut channels = HashMap::new();
        channels.insert(1, true);
        channels.insert(2, false);
        channels.insert(3, true);
        channels.insert(4, false);
        assert_eq!(on_air_to_mask(&channels), 0b0000_0101); // ch1 + ch3
    }

    #[test]
    fn on_air_mask_empty() {
        let channels = HashMap::new();
        assert_eq!(on_air_to_mask(&channels), 0);
    }

    #[test]
    fn on_air_mask_all_channels() {
        let mut channels = HashMap::new();
        for ch in 1..=4 {
            channels.insert(ch, true);
        }
        assert_eq!(on_air_to_mask(&channels), 0b0000_1111);
    }
}
