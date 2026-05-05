//! Wire protocol v2 — must stay in lockstep with `coordinator/src/main.rs`
//! and `shared/protocol.h`. If you change anything here, also update those.

use crc::{Algorithm, Crc};

pub const SYNC: [u8; 2] = [0xBE, 0xA7];
pub const VERSION: u8 = 0x02;
pub const TOTAL_LEN: u8 = 36;
pub const PACKET_SIZE: usize = 36;

/// Non-reflected CRC-8 with poly 0x31 matching the firmware's `firefly_crc8()`.
/// NOT CRC-8/MAXIM-DOW (which is reflected). Check value 0xA2 for "123456789".
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
pub const FLAG_PLAYING: u8 = 1 << 0;
pub const FLAG_CDJ_ACTIVE: u8 = 1 << 1;

#[allow(clippy::too_many_arguments)]
pub fn build_packet(
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_check_value() {
        assert_eq!(CRC8.checksum(b"123456789"), 0xA2);
    }

    #[test]
    fn build_packet_layout() {
        let pkt = build_packet(0x1122_3344_5566_7788, 0, 12000, 2, FLAG_PLAYING, 0, 0, 0);
        assert_eq!(pkt[0], 0xBE);
        assert_eq!(pkt[1], 0xA7);
        assert_eq!(pkt[2], VERSION);
        assert_eq!(pkt[3], TOTAL_LEN);
        assert_eq!(pkt[22], 2);
        assert_eq!(pkt[23], FLAG_PLAYING);
        assert_eq!(pkt[35], CRC8.checksum(&pkt[2..35]));
    }
}
