#pragma once
#include "protocol.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define WRISTBAND_EMA_ALPHA 0.1f

typedef struct {
    int64_t  clock_offset_us;
    bool     offset_initialized;
    int64_t  next_downbeat_us;
    int64_t  next_beat_us;
    uint16_t tempo_x100;
    uint8_t  beat_in_bar;
    uint8_t  flags;
    uint8_t  on_air_mask;
    uint8_t  master_device;
    uint32_t packets_received;
} wristband_state_t;

static inline void wristband_state_init(wristband_state_t *s) {
    memset(s, 0, sizeof(*s));
}

// Process a validated packet. local_now_us is the wristband's local clock.
// Returns true if packet was accepted.
static inline bool wristband_process_packet(wristband_state_t *s,
                                            const uint8_t *data,
                                            int len,
                                            int64_t local_now_us) {
    if (len != FIREFLY_PACKET_SIZE) return false;
    if (data[0] != FIREFLY_SYNC_0 || data[1] != FIREFLY_SYNC_1) return false;
    if (data[2] != FIREFLY_VERSION) return false;

    uint8_t expected_crc = firefly_crc8(&data[2], FIREFLY_PACKET_SIZE - 3);
    if (data[FIREFLY_PACKET_SIZE - 1] != expected_crc) return false;

    int64_t send_time_us;
    int64_t next_downbeat_us;
    uint16_t tempo_bpm_x100;
    int64_t next_beat_us;

    memcpy(&send_time_us,     &data[4],  sizeof(int64_t));
    memcpy(&next_downbeat_us, &data[12], sizeof(int64_t));
    memcpy(&tempo_bpm_x100,   &data[20], sizeof(uint16_t));
    memcpy(&next_beat_us,     &data[24], sizeof(int64_t));

    // Clock offset EMA
    int64_t measured_offset = send_time_us - local_now_us;
    if (!s->offset_initialized) {
        s->clock_offset_us = measured_offset;
        s->offset_initialized = true;
    } else {
        s->clock_offset_us = (int64_t)(
            WRISTBAND_EMA_ALPHA * (float)measured_offset +
            (1.0f - WRISTBAND_EMA_ALPHA) * (float)s->clock_offset_us
        );
    }

    s->next_downbeat_us = next_downbeat_us;
    s->next_beat_us     = next_beat_us;
    s->tempo_x100       = tempo_bpm_x100;
    s->beat_in_bar      = data[22];
    s->flags            = data[23];
    s->on_air_mask      = data[32];
    s->master_device    = data[33];
    s->packets_received++;
    return true;
}

// Convert coordinator timestamp to local time domain
static inline int64_t wristband_to_local(const wristband_state_t *s, int64_t coord_us) {
    return coord_us - s->clock_offset_us;
}

static inline bool wristband_is_playing(const wristband_state_t *s) {
    return (s->flags & FIREFLY_FLAG_PLAYING) != 0;
}

static inline bool wristband_is_cdj_active(const wristband_state_t *s) {
    return (s->flags & FIREFLY_FLAG_CDJ_ACTIVE) != 0;
}
