#[cfg(test)]
mod firmware_sim;

use std::collections::HashMap;
use std::io::Write;
use std::net::Ipv4Addr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use ableton_link_rs::link::{BasicLink, SessionState};
use clap::Parser;
use crc::{Algorithm, Crc};
use prodjlink_rs::{BeatEvent, ProDjLink};
use tracing::{debug, error, info, warn};

// ── Protocol v2 constants ───────────────────────────────────────────

const SYNC: [u8; 2] = [0xBE, 0xA7];
const VERSION: u8 = 0x02;
const TOTAL_LEN: u8 = 36;
const PACKET_SIZE: usize = 36;
// Non-reflected CRC-8 with poly 0x31 matching the firmware's firefly_crc8().
// NOT CRC-8/MAXIM-DOW (which is reflected). Check value 0xA2 for "123456789".
const CRC8_FIREFLY: Algorithm<u8> = Algorithm {
    width: 8,
    poly: 0x31,
    init: 0x00,
    refin: false,
    refout: false,
    xorout: 0x00,
    check: 0xA2,
    residue: 0x00,
};
const CRC8: Crc<u8> = Crc::<u8>::new(&CRC8_FIREFLY);

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

// ── Beat source state machine ───────────────────────────────────────

/// Extracted CDJ beat-processing state, testable without real hardware.
struct BeatSourceState {
    last_cdj_beat_time: Instant,
    cdj_beat_in_bar: u8,
    cdj_next_beat_us: i64,
    cdj_next_bar_us: i64,
    cdj_bpm: f64,
    cdj_playing: bool,
    master_device: u8,
    channels_on_air: HashMap<u8, bool>,
    last_beat_in_bar: u8,
    bar_counter: u8,
}

impl BeatSourceState {
    fn new() -> Self {
        Self {
            last_cdj_beat_time: Instant::now() - CDJ_TIMEOUT,
            cdj_beat_in_bar: 0,
            cdj_next_beat_us: 0,
            cdj_next_bar_us: 0,
            cdj_bpm: 0.0,
            cdj_playing: false,
            master_device: 0,
            channels_on_air: HashMap::new(),
            last_beat_in_bar: u8::MAX,
            bar_counter: 0,
        }
    }

    /// Update CDJ state from a master beat. The caller must filter to
    /// master-only beats before calling this.
    fn process_master_beat(
        &mut self,
        beat: &prodjlink_rs::Beat,
        now: Instant,
        link_clock_us: i64,
    ) {
        self.last_cdj_beat_time = now;
        self.cdj_bpm = beat.effective_tempo();
        self.cdj_beat_in_bar = if beat.beat_within_bar > 0 {
            beat.beat_within_bar - 1
        } else {
            0
        };
        self.cdj_playing = true;
        self.master_device = beat.device_number.0;

        self.cdj_next_beat_us = beat
            .next_beat
            .map(|ms| link_clock_us + (ms as i64) * 1000)
            .unwrap_or(0);

        self.cdj_next_bar_us = beat
            .next_bar
            .map(|ms| link_clock_us + (ms as i64) * 1000)
            .unwrap_or(0);
    }

    /// Returns `true` if CDJ timed out and state transitioned to not-playing.
    fn check_cdj_timeout(&mut self, now: Instant) -> bool {
        if self.cdj_playing && (now - self.last_cdj_beat_time) > CDJ_TIMEOUT {
            self.cdj_playing = false;
            true
        } else {
            false
        }
    }

    /// Whether the CDJ is currently the timing authority.
    fn is_cdj_active(&self, now: Instant) -> bool {
        self.cdj_playing && (now - self.last_cdj_beat_time) < CDJ_TIMEOUT
    }

    fn on_air_mask(&self) -> u8 {
        on_air_to_mask(&self.channels_on_air)
    }

    fn update_on_air(&mut self, channels: HashMap<u8, bool>) {
        self.channels_on_air = channels;
    }
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
    let mut state = BeatSourceState::new();

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
                        let now_instant = Instant::now();
                        let now_us = link.clock().micros()
                            .num_microseconds()
                            .unwrap_or(0);

                        state.process_master_beat(&beat, now_instant, now_us);

                        // Bridge CDJ tempo → Link
                        let link_bpm = link.capture_app_session_state().tempo();
                        if (link_bpm - state.cdj_bpm).abs() > TEMPO_EPSILON {
                            let time = link.clock().micros();
                            let mut session = link.capture_app_session_state();
                            session.set_tempo(state.cdj_bpm, time);
                            link.commit_app_session_state(session).await;
                            debug!(cdj_bpm = state.cdj_bpm, "bridged CDJ tempo → Link");
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
                let channels: HashMap<u8, bool> = on_air.channels.iter().map(|(&k, &v)| (k, v)).collect();
                let active: Vec<u8> = channels.iter()
                    .filter(|(_, v)| **v)
                    .map(|(&k, _)| k)
                    .collect();
                state.update_on_air(channels);
                debug!(on_air = ?active, "channels on-air updated");
            }
        }

        // ── CDJ timeout detection ──────────────────────────────────
        if state.check_cdj_timeout(Instant::now()) {
            info!("CDJ beat timeout — falling back to Link-only");
            let time = link.clock().micros();
            let mut session = link.capture_app_session_state();
            session.set_is_playing(false, time);
            link.commit_app_session_state(session).await;
        }

        // ── Build packet ───────────────────────────────────────────
        let now = link.clock().micros();
        let session = link.capture_app_session_state();
        let cdj_active = state.is_cdj_active(Instant::now());

        let (tempo, beat_in_bar, next_db_us, next_bt_us, is_playing) = if cdj_active {
            // CDJ is authoritative
            (state.cdj_bpm, state.cdj_beat_in_bar, state.cdj_next_bar_us, state.cdj_next_beat_us, true)
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
        if beat_in_bar == 0 && state.last_beat_in_bar != 0 && state.last_beat_in_bar != u8::MAX {
            state.bar_counter = state.bar_counter.wrapping_add(1);
        }
        state.last_beat_in_bar = beat_in_bar;

        let send_time_us = now.num_microseconds().expect("coordinator clock overflow");

        let mut flags: u8 = 0;
        if is_playing { flags |= FLAG_PLAYING; }
        if cdj_active { flags |= FLAG_CDJ_ACTIVE; }

        let on_air_mask = state.on_air_mask();
        let master_dev = if cdj_active { state.master_device } else { 0 };

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
                    bar = state.bar_counter,
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
    fn crc8_golden_vectors() {
        // Standard check string — must produce 0xA2 (non-reflected poly 0x31)
        assert_eq!(CRC8.checksum(b"123456789"), 0xA2);
        // All zeros
        assert_eq!(CRC8.checksum(&[0u8; 10]), 0x00);
        // Sample v2 packet payload: version(0x02) + total_len(36) + 31 zero bytes
        let mut payload = [0u8; 33];
        payload[0] = 0x02;
        payload[1] = 36;
        let crc = CRC8.checksum(&payload);
        // Deterministic — just verify it's stable and non-trivial for non-zero input
        assert_ne!(crc, 0x00);
        assert_eq!(crc, CRC8.checksum(&payload)); // idempotent
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

    // ── BeatSourceState tests ──────────────────────────────────────

    use prodjlink_rs::{Bpm, DeviceNumber, DeviceType, Pitch};
    use prodjlink_rs::protocol::beat::Beat;

    fn make_test_beat(device: u8, bpm: f64, beat_within_bar: u8) -> Beat {
        Beat {
            name: "CDJ-TEST".to_string(),
            device_number: DeviceNumber(device),
            device_type: DeviceType::Cdj,
            bpm: Bpm(bpm),
            pitch: Pitch(0x100000), // normal speed
            next_beat: Some(500),
            second_beat: None,
            next_bar: Some(2000),
            fourth_beat: None,
            second_bar: None,
            eighth_beat: None,
            beat_within_bar,
            timestamp: Instant::now(),
        }
    }

    #[test]
    fn process_master_beat_updates_all_fields() {
        let mut state = BeatSourceState::new();
        let beat = make_test_beat(2, 128.0, 3);
        let now = Instant::now();
        let link_clock_us: i64 = 1_000_000;

        state.process_master_beat(&beat, now, link_clock_us);

        assert_eq!(state.last_cdj_beat_time, now);
        assert!((state.cdj_bpm - 128.0).abs() < 0.01);
        assert_eq!(state.cdj_beat_in_bar, 2); // 3 (1-based) → 2 (0-based)
        assert!(state.cdj_playing);
        assert_eq!(state.master_device, 2);
        // next_beat=500ms → link_clock + 500*1000 = 1_500_000
        assert_eq!(state.cdj_next_beat_us, 1_500_000);
        // next_bar=2000ms → link_clock + 2000*1000 = 3_000_000
        assert_eq!(state.cdj_next_bar_us, 3_000_000);
    }

    #[test]
    fn check_cdj_timeout_transitions_after_2s() {
        let mut state = BeatSourceState::new();
        let past = Instant::now() - Duration::from_secs(3);
        state.cdj_playing = true;
        state.last_cdj_beat_time = past;

        assert!(state.check_cdj_timeout(Instant::now()));
        assert!(!state.cdj_playing);
    }

    #[test]
    fn check_cdj_timeout_no_transition_before_2s() {
        let mut state = BeatSourceState::new();
        state.cdj_playing = true;
        state.last_cdj_beat_time = Instant::now();

        assert!(!state.check_cdj_timeout(Instant::now()));
        assert!(state.cdj_playing);
    }

    #[test]
    fn is_cdj_active_reflects_timeout() {
        let mut state = BeatSourceState::new();
        let now = Instant::now();

        // Not playing → not active
        assert!(!state.is_cdj_active(now));

        // Playing with recent beat → active
        state.cdj_playing = true;
        state.last_cdj_beat_time = now;
        assert!(state.is_cdj_active(now));

        // Playing but beat too old → not active
        state.last_cdj_beat_time = now - Duration::from_secs(3);
        assert!(!state.is_cdj_active(now));
    }

    #[test]
    fn on_air_mask_via_state() {
        let mut state = BeatSourceState::new();
        let mut channels = HashMap::new();
        channels.insert(1, true);
        channels.insert(2, false);
        channels.insert(4, true);
        state.update_on_air(channels);

        assert_eq!(state.on_air_mask(), 0b0000_1001); // ch1 + ch4
    }

    // ── End-to-end integration tests ───────────────────────────────
    //
    // These exercise the full pipeline: CDJ Beat → BeatSourceState →
    // build_packet → DongleSim (framing + CRC) → WristbandSim (parse
    // + clock offset + flash scheduling). No real hardware required.

    use crate::firmware_sim::{DongleSim, WristbandSim};

    /// Helper: process a beat through the full pipeline and return the
    /// wristband's state after receiving the packet.
    fn pipeline(
        beat: &Beat,
        on_air: Option<HashMap<u8, bool>>,
    ) -> (BeatSourceState, [u8; PACKET_SIZE], WristbandSim) {
        let mut state = BeatSourceState::new();
        let now = Instant::now();
        let link_clock_us: i64 = 10_000_000; // 10s into the session

        state.process_master_beat(beat, now, link_clock_us);
        if let Some(channels) = on_air {
            state.update_on_air(channels);
        }

        let cdj_active = state.is_cdj_active(now);
        let mut flags: u8 = 0;
        if cdj_active { flags |= FLAG_PLAYING | FLAG_CDJ_ACTIVE; }

        let packet = build_packet(
            link_clock_us,
            state.cdj_next_bar_us,
            (state.cdj_bpm * 100.0) as u16,
            state.cdj_beat_in_bar,
            flags,
            state.cdj_next_beat_us,
            state.on_air_mask(),
            if cdj_active { state.master_device } else { 0 },
        );

        // Dongle: validate framing + CRC
        let mut dongle = DongleSim::new();
        let forwarded = dongle.feed_bytes(&packet);
        assert_eq!(forwarded.len(), 1, "dongle must forward the packet");
        assert_eq!(dongle.crc_errors, 0, "coordinator CRC must match firmware CRC");
        assert_eq!(dongle.version_errors, 0);

        // Wristband: parse + clock offset
        let mut wristband = WristbandSim::new();
        // Simulate wristband local clock = coordinator clock - 5ms transport delay
        let wb_local_now = link_clock_us - 5_000;
        wristband.receive_packet(&forwarded[0], wb_local_now);

        (state, packet, wristband)
    }

    #[test]
    fn e2e_cdj_beat_flows_to_wristband() {
        let beat = make_test_beat(1, 126.3, 1); // CDJ-3000 P1 at 126.3 BPM, beat 1
        let (state, _pkt, wb) = pipeline(&beat, None);

        // Coordinator state
        assert!(state.cdj_playing);
        assert_eq!(state.master_device, 1);
        assert!((state.cdj_bpm - 126.3).abs() < 0.01);

        // Wristband received the packet
        assert_eq!(wb.packets_received, 1);
        assert!(wb.is_playing());
        assert!(wb.is_cdj_active());
        assert_eq!(wb.master_device, 1);
        assert_eq!(wb.tempo_x100, 12630);
        assert_eq!(wb.beat_in_bar, 0); // 1 (1-based) → 0 (0-based)
    }

    #[test]
    fn e2e_downbeat_detection() {
        // beat_within_bar=1 is the downbeat (1-based from CDJ)
        let beat = make_test_beat(2, 128.0, 1);
        let (_state, _pkt, wb) = pipeline(&beat, None);

        assert_eq!(wb.beat_in_bar, 0); // 0-based downbeat
        assert!(wb.is_next_downbeat());
    }

    #[test]
    fn e2e_non_downbeat() {
        let beat = make_test_beat(2, 128.0, 3);
        let (_state, _pkt, wb) = pipeline(&beat, None);

        assert_eq!(wb.beat_in_bar, 2); // 3 (1-based) → 2 (0-based)
        // beat_in_bar != 0 and next_beat_us != next_downbeat_us
        assert!(!wb.is_next_downbeat() || wb.next_beat_us == wb.next_downbeat_us);
    }

    #[test]
    fn e2e_on_air_mask_propagated() {
        let beat = make_test_beat(1, 120.0, 1);
        let mut channels = HashMap::new();
        channels.insert(1, true); // ch1 on-air
        channels.insert(2, true); // ch2 on-air
        channels.insert(3, false);
        channels.insert(4, false);

        let (_state, _pkt, wb) = pipeline(&beat, Some(channels));

        assert_eq!(wb.on_air_mask, 0b0000_0011); // ch1 + ch2
    }

    #[test]
    fn e2e_master_device_propagated() {
        let beat = make_test_beat(3, 140.0, 2); // device 3 is master
        let (_state, _pkt, wb) = pipeline(&beat, None);

        assert_eq!(wb.master_device, 3);
    }

    #[test]
    fn e2e_next_beat_timing_propagated() {
        let beat = make_test_beat(1, 126.0, 2);
        let (_state, _pkt, wb) = pipeline(&beat, None);

        // CDJ says next_beat=500ms from now → link_clock(10_000_000) + 500*1000
        assert_eq!(wb.next_beat_us, 10_500_000);
        // CDJ says next_bar=2000ms from now → link_clock(10_000_000) + 2000*1000
        assert_eq!(wb.next_downbeat_us, 12_000_000);
    }

    #[test]
    fn e2e_wristband_schedules_flash() {
        let beat = make_test_beat(1, 128.0, 2);
        let (_state, _pkt, wb) = pipeline(&beat, None);

        let flash = wb.next_flash_local_us().expect("should schedule a flash");
        // Flash should be in the future relative to wristband's local clock
        // wb_local_now was 10_000_000 - 5_000 = 9_995_000
        // next_beat_us is 10_500_000, clock_offset = 10_000_000 - 9_995_000 = 5000
        // local flash = 10_500_000 - 5000 = 10_495_000
        assert!(flash > 9_995_000, "flash should be in the future");
    }

    #[test]
    fn e2e_cdj_timeout_clears_cdj_active() {
        let beat = make_test_beat(1, 126.0, 1);
        let mut state = BeatSourceState::new();
        let past = Instant::now() - Duration::from_secs(3);

        state.process_master_beat(&beat, past, 10_000_000);

        // 3 seconds later: CDJ should have timed out
        let now = Instant::now();
        assert!(state.check_cdj_timeout(now));
        assert!(!state.cdj_playing);
        assert!(!state.is_cdj_active(now));

        // Build a Link-only packet (no CDJ data)
        let flags: u8 = 0; // not playing, not cdj_active
        let packet = build_packet(10_000_000, 0, 12000, 0, flags, 0, 0, 0);

        let mut dongle = DongleSim::new();
        let forwarded = dongle.feed_bytes(&packet);
        assert_eq!(forwarded.len(), 1);

        let mut wb = WristbandSim::new();
        wb.receive_packet(&forwarded[0], 9_995_000);

        assert!(!wb.is_playing());
        assert!(!wb.is_cdj_active());
        assert_eq!(wb.master_device, 0);
    }

    #[test]
    fn e2e_multi_beat_sequence() {
        // Simulate 4 beats (one full bar) flowing through the pipeline
        let mut state = BeatSourceState::new();
        let mut dongle = DongleSim::new();
        let mut wristband = WristbandSim::new();
        let base_time = Instant::now();
        let beat_interval_ms: u64 = 469; // ~128 BPM

        for i in 0..4u8 {
            let beat_within_bar = i + 1; // 1-based
            let beat = make_test_beat(1, 128.0, beat_within_bar);
            let now = base_time + Duration::from_millis(i as u64 * beat_interval_ms);
            let link_us = 10_000_000 + (i as i64) * (beat_interval_ms as i64) * 1000;

            state.process_master_beat(&beat, now, link_us);

            let cdj_active = state.is_cdj_active(now);
            let mut flags: u8 = 0;
            if cdj_active { flags |= FLAG_PLAYING | FLAG_CDJ_ACTIVE; }

            let packet = build_packet(
                link_us,
                state.cdj_next_bar_us,
                (state.cdj_bpm * 100.0) as u16,
                state.cdj_beat_in_bar,
                flags,
                state.cdj_next_beat_us,
                0, 0,
            );

            let forwarded = dongle.feed_bytes(&packet);
            assert_eq!(forwarded.len(), 1, "beat {} must forward", i);

            let wb_local = link_us - 5_000;
            wristband.receive_packet(&forwarded[0], wb_local);

            assert_eq!(wristband.beat_in_bar, i); // 0-based
            assert!(wristband.is_playing());
            assert!(wristband.is_cdj_active());
        }

        assert_eq!(dongle.packets_forwarded, 4);
        assert_eq!(wristband.packets_received, 4);
        assert!(wristband.offset_initialized);
    }

    #[test]
    fn e2e_dongle_rejects_corrupted_then_accepts_good() {
        let beat = make_test_beat(1, 120.0, 1);
        let mut state = BeatSourceState::new();
        let now = Instant::now();
        state.process_master_beat(&beat, now, 10_000_000);

        let packet = build_packet(
            10_000_000, state.cdj_next_bar_us,
            12000, 0, FLAG_PLAYING | FLAG_CDJ_ACTIVE,
            state.cdj_next_beat_us, 0, 1,
        );

        // Corrupt the first packet
        let mut bad = packet;
        bad[20] ^= 0xFF; // corrupt tempo field

        let mut dongle = DongleSim::new();

        // Feed corrupted packet — should be rejected (CRC mismatch)
        let results = dongle.feed_bytes(&bad);
        assert!(results.is_empty());
        assert_eq!(dongle.crc_errors, 1);

        // Feed good packet — should pass
        let results = dongle.feed_bytes(&packet);
        assert_eq!(results.len(), 1);
        assert_eq!(dongle.packets_forwarded, 1);
    }

    #[test]
    fn e2e_crc_cross_validation() {
        // Verify that the coordinator's CRC (crc crate with custom algo)
        // matches the firmware's CRC (manual bit-banging) for a real packet.
        let packet = build_packet(
            99_999_999, 199_999_999, 12630, 2,
            FLAG_PLAYING | FLAG_CDJ_ACTIVE,
            149_999_999, 0b0000_0101, 3,
        );

        // Coordinator CRC
        let coord_crc = packet[35];

        // Firmware CRC (via DongleSim which uses the manual implementation)
        let mut dongle = DongleSim::new();
        let forwarded = dongle.feed_bytes(&packet);
        assert_eq!(forwarded.len(), 1, "CRC must match — coordinator and firmware agree");
        assert_eq!(dongle.crc_errors, 0, "zero CRC errors confirms cross-compatibility");

        // Also verify the CRC byte is non-trivial
        assert_ne!(coord_crc, 0x00, "CRC should be non-trivial for this payload");
    }
}
