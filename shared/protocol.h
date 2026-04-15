/**
 * Firefly Protocol v2 — Shared packet definition
 *
 * Wire format (33 bytes total, little-endian):
 *   [0..1]   sync              0xBE 0xA7
 *   [2]      version           0x02
 *   [3]      total_len         33
 *   [4..11]  send_time_us      i64  coordinator clock at packet creation
 *   [12..19] next_downbeat_us  i64  coordinator clock time of next downbeat
 *   [20..21] tempo_bpm_x100    u16  BPM × 100 (e.g. 12000 = 120.00 BPM)
 *   [22]     beat_in_bar       u8   current beat phase (0–3 for 4/4)
 *   [23]     flags             u8   bit 0 = is_playing, bit 1 = cdj_active
 *   [24..31] next_beat_us      i64  coordinator clock time of next beat
 *   [32]     on_air_mask       u8   bitmask: bit N = channel (N+1) on-air
 *   [33]     master_device     u8   DJ Link device number of master (0 = none)
 *   [34]     reserved          u8   reserved for future use
 *   [35]     crc8              u8   CRC-8/MAXIM over bytes [2..35)
 *
 * Sentinels:
 *   next_beat_us = 0      → unknown / not available
 *   master_device = 0     → no DJ Link master detected
 *   next_downbeat_us = 0  → unknown
 */

#pragma once

#include <stdint.h>

#define FIREFLY_SYNC_0        0xBE
#define FIREFLY_SYNC_1        0xA7
#define FIREFLY_VERSION       0x02
#define FIREFLY_TOTAL_LEN     36
#define FIREFLY_PACKET_SIZE   36

// Flag bits
#define FIREFLY_FLAG_PLAYING    (1 << 0)
#define FIREFLY_FLAG_CDJ_ACTIVE (1 << 1)

typedef struct __attribute__((packed)) {
    uint8_t  sync[2];
    uint8_t  version;
    uint8_t  total_len;
    int64_t  send_time_us;
    int64_t  next_downbeat_us;
    uint16_t tempo_bpm_x100;
    uint8_t  beat_in_bar;
    uint8_t  flags;
    int64_t  next_beat_us;
    uint8_t  on_air_mask;
    uint8_t  master_device;
    uint8_t  reserved;
    uint8_t  crc8;
} firefly_packet_t;

// CRC-8/MAXIM (polynomial 0x31, init 0x00, no reflect, no xor-out)
static inline uint8_t firefly_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
