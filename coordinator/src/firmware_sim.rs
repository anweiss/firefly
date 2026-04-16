//! Firmware simulation for integration testing.
//!
//! Provides Rust implementations of the dongle serial framing state machine
//! and wristband packet parsing + clock offset EMA, so we can verify the
//! coordinator's output without real ESP32 hardware.

const SYNC_0: u8 = 0xBE;
const SYNC_1: u8 = 0xA7;
const VERSION: u8 = 0x02;
const PACKET_SIZE: usize = 36;

const FLAG_PLAYING: u8 = 1 << 0;
const FLAG_CDJ_ACTIVE: u8 = 1 << 1;

/// Non-reflected CRC-8, poly 0x31, matching protocol.h's firefly_crc8().
fn firefly_crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x00;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ── Dongle simulator ────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RxState {
    WaitSync0,
    WaitSync1,
    ReadPayload,
}

/// Simulates the dongle's serial framing state machine from dongle.ino.
///
/// Feed bytes through `feed_byte` / `feed_bytes` to receive validated packets
/// exactly as the real firmware would forward them over ESP-NOW.
pub struct DongleSim {
    rx_state: RxState,
    rx_buf: [u8; PACKET_SIZE],
    rx_idx: usize,
    pub packets_forwarded: u32,
    pub crc_errors: u32,
    pub version_errors: u32,
}

impl DongleSim {
    pub fn new() -> Self {
        Self {
            rx_state: RxState::WaitSync0,
            rx_buf: [0u8; PACKET_SIZE],
            rx_idx: 0,
            packets_forwarded: 0,
            crc_errors: 0,
            version_errors: 0,
        }
    }

    /// Feed a single byte (simulating serial reception).
    /// Returns `Some(packet)` when a valid packet is assembled and CRC checks out.
    pub fn feed_byte(&mut self, b: u8) -> Option<[u8; PACKET_SIZE]> {
        match self.rx_state {
            RxState::WaitSync0 => {
                if b == SYNC_0 {
                    self.rx_buf[0] = b;
                    self.rx_state = RxState::WaitSync1;
                }
                None
            }
            RxState::WaitSync1 => {
                if b == SYNC_1 {
                    self.rx_buf[1] = b;
                    self.rx_idx = 2;
                    self.rx_state = RxState::ReadPayload;
                } else {
                    // False sync — check if this byte is a new SYNC_0
                    if b == SYNC_0 {
                        self.rx_buf[0] = b;
                        self.rx_state = RxState::WaitSync1;
                    } else {
                        self.rx_state = RxState::WaitSync0;
                    }
                }
                None
            }
            RxState::ReadPayload => {
                self.rx_buf[self.rx_idx] = b;
                self.rx_idx += 1;

                if self.rx_idx >= PACKET_SIZE {
                    let result = self.validate_packet();
                    self.rx_state = RxState::WaitSync0;
                    self.rx_idx = 0;
                    result
                } else {
                    None
                }
            }
        }
    }

    /// Feed a slice of bytes. Returns all valid packets found.
    pub fn feed_bytes(&mut self, data: &[u8]) -> Vec<[u8; PACKET_SIZE]> {
        let mut packets = Vec::new();
        for &b in data {
            if let Some(pkt) = self.feed_byte(b) {
                packets.push(pkt);
            }
        }
        packets
    }

    fn validate_packet(&mut self) -> Option<[u8; PACKET_SIZE]> {
        // Check version
        if self.rx_buf[2] != VERSION {
            self.version_errors += 1;
            return None;
        }

        // CRC over bytes [2..35)
        let expected_crc = firefly_crc8(&self.rx_buf[2..35]);
        if self.rx_buf[PACKET_SIZE - 1] != expected_crc {
            self.crc_errors += 1;
            return None;
        }

        self.packets_forwarded += 1;
        Some(self.rx_buf)
    }
}

// ── Wristband simulator ─────────────────────────────────────────────

const EMA_ALPHA: f64 = 0.1;

/// Simulates the wristband's packet parsing and clock offset EMA
/// from wristband.ino.
pub struct WristbandSim {
    pub clock_offset_us: i64,
    pub offset_initialized: bool,
    pub next_downbeat_us: i64,
    pub next_beat_us: i64,
    pub tempo_x100: u16,
    pub beat_in_bar: u8,
    pub flags: u8,
    pub on_air_mask: u8,
    pub master_device: u8,
    pub packets_received: u32,
}

impl WristbandSim {
    pub fn new() -> Self {
        Self {
            clock_offset_us: 0,
            offset_initialized: false,
            next_downbeat_us: 0,
            next_beat_us: 0,
            tempo_x100: 0,
            beat_in_bar: 0,
            flags: 0,
            on_air_mask: 0,
            master_device: 0,
            packets_received: 0,
        }
    }

    /// Process a validated 36-byte packet (as received from the dongle).
    /// `local_now_us` simulates `esp_timer_get_time()`.
    pub fn receive_packet(&mut self, packet: &[u8; PACKET_SIZE], local_now_us: i64) {
        // Validate sync, version, CRC (same checks as wristband on_receive)
        if packet[0] != SYNC_0 || packet[1] != SYNC_1 {
            return;
        }
        if packet[2] != VERSION {
            return;
        }
        let expected_crc = firefly_crc8(&packet[2..35]);
        if packet[PACKET_SIZE - 1] != expected_crc {
            return;
        }

        // Parse fields (little-endian)
        let send_time_us = i64::from_le_bytes(packet[4..12].try_into().unwrap());
        let next_downbeat_us = i64::from_le_bytes(packet[12..20].try_into().unwrap());
        let tempo_bpm_x100 = u16::from_le_bytes(packet[20..22].try_into().unwrap());
        let beat_in_bar = packet[22];
        let flags = packet[23];
        let next_beat_us = i64::from_le_bytes(packet[24..32].try_into().unwrap());
        let on_air_mask = packet[32];
        let master_device = packet[33];

        // Clock offset EMA
        let measured_offset = send_time_us - local_now_us;
        if !self.offset_initialized {
            self.clock_offset_us = measured_offset;
            self.offset_initialized = true;
        } else {
            self.clock_offset_us = (EMA_ALPHA * measured_offset as f64
                + (1.0 - EMA_ALPHA) * self.clock_offset_us as f64)
                as i64;
        }

        // Update shared state
        self.next_downbeat_us = next_downbeat_us;
        self.next_beat_us = next_beat_us;
        self.tempo_x100 = tempo_bpm_x100;
        self.beat_in_bar = beat_in_bar;
        self.flags = flags;
        self.on_air_mask = on_air_mask;
        self.master_device = master_device;
        self.packets_received += 1;
    }

    /// Convert a coordinator-domain timestamp to the local clock domain.
    pub fn to_local_time(&self, coordinator_us: i64) -> i64 {
        coordinator_us - self.clock_offset_us
    }

    pub fn is_playing(&self) -> bool {
        self.flags & FLAG_PLAYING != 0
    }

    pub fn is_cdj_active(&self) -> bool {
        self.flags & FLAG_CDJ_ACTIVE != 0
    }

    /// Returns the scheduled flash time in local microseconds, or `None` if
    /// not playing or no target is known.
    pub fn next_flash_local_us(&self) -> Option<i64> {
        if !self.is_playing() {
            return None;
        }
        // Prefer next_beat_us; fall back to next_downbeat_us
        let target_us = if self.next_beat_us != 0 {
            self.next_beat_us
        } else if self.next_downbeat_us != 0 {
            self.next_downbeat_us
        } else {
            return None;
        };
        Some(self.to_local_time(target_us))
    }

    /// Returns `true` if the next flash corresponds to a downbeat.
    pub fn is_next_downbeat(&self) -> bool {
        self.beat_in_bar == 0
            || (self.next_beat_us != 0 && self.next_beat_us == self.next_downbeat_us)
    }
}

// ── Tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper: build a valid v2 packet ─────────────────────────────

    fn make_packet(
        send_time_us: i64,
        next_downbeat_us: i64,
        tempo_x100: u16,
        beat_in_bar: u8,
        flags: u8,
        next_beat_us: i64,
        on_air_mask: u8,
        master_device: u8,
    ) -> [u8; PACKET_SIZE] {
        let mut buf = [0u8; PACKET_SIZE];
        buf[0] = SYNC_0;
        buf[1] = SYNC_1;
        buf[2] = VERSION;
        buf[3] = PACKET_SIZE as u8;
        buf[4..12].copy_from_slice(&send_time_us.to_le_bytes());
        buf[12..20].copy_from_slice(&next_downbeat_us.to_le_bytes());
        buf[20..22].copy_from_slice(&tempo_x100.to_le_bytes());
        buf[22] = beat_in_bar;
        buf[23] = flags;
        buf[24..32].copy_from_slice(&next_beat_us.to_le_bytes());
        buf[32] = on_air_mask;
        buf[33] = master_device;
        buf[34] = 0; // reserved
        buf[35] = firefly_crc8(&buf[2..35]);
        buf
    }

    fn make_default_packet() -> [u8; PACKET_SIZE] {
        make_packet(1_000_000, 2_000_000, 12000, 0, FLAG_PLAYING, 1_500_000, 0, 0)
    }

    // ── CRC golden vectors ──────────────────────────────────────────

    #[test]
    fn crc8_check_string() {
        assert_eq!(firefly_crc8(b"123456789"), 0xA2);
    }

    #[test]
    fn crc8_all_zeros() {
        assert_eq!(firefly_crc8(&[0u8; 10]), 0x00);
    }

    #[test]
    fn crc8_all_ones() {
        let crc = firefly_crc8(&[0xFF; 10]);
        // Deterministic non-trivial value
        assert_eq!(crc, firefly_crc8(&[0xFF; 10]));
        assert_ne!(crc, 0x00);
    }

    // ── Dongle tests ────────────────────────────────────────────────

    #[test]
    fn valid_packet_passes_through() {
        let mut dongle = DongleSim::new();
        let pkt = make_default_packet();
        let results = dongle.feed_bytes(&pkt);
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], pkt);
        assert_eq!(dongle.packets_forwarded, 1);
        assert_eq!(dongle.crc_errors, 0);
        assert_eq!(dongle.version_errors, 0);
    }

    #[test]
    fn bad_crc_rejected() {
        let mut dongle = DongleSim::new();
        let mut pkt = make_default_packet();
        pkt[35] ^= 0xFF; // corrupt CRC
        let results = dongle.feed_bytes(&pkt);
        assert!(results.is_empty());
        assert_eq!(dongle.crc_errors, 1);
        assert_eq!(dongle.packets_forwarded, 0);
    }

    #[test]
    fn bad_version_rejected() {
        let mut dongle = DongleSim::new();
        let mut pkt = make_default_packet();
        pkt[2] = 0x01; // wrong version
        // Recompute CRC for the corrupted version so we test version check, not CRC
        pkt[35] = firefly_crc8(&pkt[2..35]);
        let results = dongle.feed_bytes(&pkt);
        assert!(results.is_empty());
        assert_eq!(dongle.version_errors, 1);
        assert_eq!(dongle.crc_errors, 0);
    }

    #[test]
    fn garbage_before_sync_handled() {
        let mut dongle = DongleSim::new();
        let pkt = make_default_packet();
        let mut data = vec![0x00, 0x11, 0x22, 0x33, 0x44];
        data.extend_from_slice(&pkt);
        let results = dongle.feed_bytes(&data);
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], pkt);
    }

    #[test]
    fn false_sync_byte_handled() {
        let mut dongle = DongleSim::new();
        let pkt = make_default_packet();
        // 0xBE followed by non-0xA7, then the real packet
        let mut data = vec![SYNC_0, 0x00];
        data.extend_from_slice(&pkt);
        let results = dongle.feed_bytes(&data);
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], pkt);
    }

    #[test]
    fn back_to_back_packets() {
        let mut dongle = DongleSim::new();
        let pkt1 = make_packet(100, 200, 12000, 0, FLAG_PLAYING, 150, 0, 0);
        let pkt2 = make_packet(300, 400, 12800, 1, FLAG_PLAYING | FLAG_CDJ_ACTIVE, 350, 0b11, 2);
        let mut data = Vec::new();
        data.extend_from_slice(&pkt1);
        data.extend_from_slice(&pkt2);
        let results = dongle.feed_bytes(&data);
        assert_eq!(results.len(), 2);
        assert_eq!(results[0], pkt1);
        assert_eq!(results[1], pkt2);
        assert_eq!(dongle.packets_forwarded, 2);
    }

    #[test]
    fn partial_packet_then_valid_packet() {
        let mut dongle = DongleSim::new();
        let pkt = make_default_packet();
        // Feed first half
        let results1 = dongle.feed_bytes(&pkt[..18]);
        assert!(results1.is_empty());
        // Feed second half
        let results2 = dongle.feed_bytes(&pkt[18..]);
        assert_eq!(results2.len(), 1);
        assert_eq!(results2[0], pkt);
    }

    // ── Wristband tests ─────────────────────────────────────────────

    #[test]
    fn parse_all_fields_correctly() {
        let mut wb = WristbandSim::new();
        let pkt = make_packet(
            1_000_000,   // send_time
            2_000_000,   // next_downbeat
            12800,       // 128.00 BPM
            2,           // beat_in_bar
            FLAG_PLAYING | FLAG_CDJ_ACTIVE,
            1_500_000,   // next_beat
            0b0000_0101, // on_air: ch1 + ch3
            3,           // master_device
        );
        wb.receive_packet(&pkt, 999_000); // local time slightly behind

        assert_eq!(wb.next_downbeat_us, 2_000_000);
        assert_eq!(wb.next_beat_us, 1_500_000);
        assert_eq!(wb.tempo_x100, 12800);
        assert_eq!(wb.beat_in_bar, 2);
        assert!(wb.is_playing());
        assert!(wb.is_cdj_active());
        assert_eq!(wb.on_air_mask, 0b0000_0101);
        assert_eq!(wb.master_device, 3);
        assert_eq!(wb.packets_received, 1);
    }

    #[test]
    fn clock_offset_ema_first_packet() {
        let mut wb = WristbandSim::new();
        let send_time = 10_000_000_i64;
        let local_now = 9_990_000_i64;
        let pkt = make_packet(send_time, 0, 12000, 0, FLAG_PLAYING, 0, 0, 0);
        wb.receive_packet(&pkt, local_now);

        assert!(wb.offset_initialized);
        assert_eq!(wb.clock_offset_us, send_time - local_now); // 10_000
    }

    #[test]
    fn clock_offset_ema_converges() {
        let mut wb = WristbandSim::new();
        let true_offset: i64 = 50_000; // coordinator is 50ms ahead

        for i in 0..20 {
            let local_now = 1_000_000 + i * 50_000_i64;
            let send_time = local_now + true_offset;
            let pkt = make_packet(send_time, 0, 12000, 0, FLAG_PLAYING, 0, 0, 0);
            wb.receive_packet(&pkt, local_now);
        }

        // After 20 packets with constant offset, EMA should be very close
        let error = (wb.clock_offset_us - true_offset).abs();
        assert!(
            error < 500,
            "EMA should converge to true offset; error={error} us"
        );
    }

    #[test]
    fn clock_offset_ema_with_jitter() {
        let mut wb = WristbandSim::new();
        let true_offset: i64 = 100_000;
        // Jitter pattern: ±2000 µs
        let jitter = [
            1500_i64, -800, 2000, -1200, 500, -1800, 1000, -600, 1900, -1500, 700, -900, 1600,
            -400, 1100, -1700, 800, -1000, 1400, -500, 900, -1300, 1800, -200, 600, -1600, 1200,
            -700, 1500, -1100,
        ];

        for (i, &j) in jitter.iter().enumerate() {
            let local_now = 1_000_000 + (i as i64) * 50_000;
            let send_time = local_now + true_offset + j;
            let pkt = make_packet(send_time, 0, 12000, 0, FLAG_PLAYING, 0, 0, 0);
            wb.receive_packet(&pkt, local_now);
        }

        // With EMA smoothing the jitter should be bounded
        let error = (wb.clock_offset_us - true_offset).abs();
        assert!(
            error < 3000,
            "EMA with jitter should stay bounded; error={error} us"
        );
    }

    #[test]
    fn next_flash_uses_next_beat_when_available() {
        let mut wb = WristbandSim::new();
        let pkt = make_packet(
            1_000_000, // send_time
            2_000_000, // next_downbeat
            12000,
            1,            // beat_in_bar (not downbeat)
            FLAG_PLAYING, // playing
            1_500_000,    // next_beat (non-zero → preferred)
            0,
            0,
        );
        wb.receive_packet(&pkt, 1_000_000);

        let flash = wb.next_flash_local_us().expect("should have flash time");
        let expected = wb.to_local_time(1_500_000);
        assert_eq!(flash, expected);
    }

    #[test]
    fn next_flash_falls_back_to_downbeat_when_no_beat() {
        let mut wb = WristbandSim::new();
        let pkt = make_packet(
            1_000_000, // send_time
            2_000_000, // next_downbeat
            12000,
            0,            // beat_in_bar
            FLAG_PLAYING, // playing
            0,            // next_beat = 0 → unknown
            0,
            0,
        );
        wb.receive_packet(&pkt, 1_000_000);

        let flash = wb.next_flash_local_us().expect("should fall back to downbeat");
        let expected = wb.to_local_time(2_000_000);
        assert_eq!(flash, expected);
    }

    #[test]
    fn not_playing_returns_no_flash() {
        let mut wb = WristbandSim::new();
        let pkt = make_packet(
            1_000_000,
            2_000_000,
            12000,
            0,
            0, // not playing
            1_500_000,
            0,
            0,
        );
        wb.receive_packet(&pkt, 1_000_000);
        assert!(wb.next_flash_local_us().is_none());
    }
}
