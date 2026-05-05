/**
 * Firefly beat-clock helper.
 *
 * Both the dongle and the wristband display the current beat-in-bar on
 * their OLED. They refresh independently (~2 Hz), so they cannot rely
 * on the snapshot field `beat_in_bar` from the most recently received
 * packet — that snapshot is always one OLED-refresh-period stale and
 * the two devices drift relative to each other.
 *
 * Instead, both devices compute the *current* beat-in-bar by
 * extrapolating from the last received packet using its `next_beat_us`
 * field and tempo. Given the same coordinator-clock "now", the same
 * `next_beat_us`, the same `latest_beat_in_bar`, and the same tempo,
 * the two devices produce identical results.
 *
 * Inputs:
 *   coord_now_us         — current time in coordinator clock (microseconds)
 *   next_beat_us         — coordinator-clock time of the next beat boundary
 *                          (from the most recent packet); 0 = unknown
 *   latest_beat_in_bar   — beat-in-bar value from the most recent packet
 *                          (the beat that was current at packet send_time)
 *   tempo_x100           — tempo from packet, in BPM × 100
 *   quantum              — beats per bar (typically 4)
 *
 * Returns the current beat-in-bar (0..quantum-1), or `latest_beat_in_bar`
 * if extrapolation isn't possible.
 */

#pragma once

#include <stdint.h>

static inline uint8_t firefly_current_beat_in_bar(
    int64_t  coord_now_us,
    int64_t  next_beat_us,
    uint8_t  latest_beat_in_bar,
    uint16_t tempo_x100,
    uint8_t  quantum)
{
    if (quantum == 0) {
        return latest_beat_in_bar;
    }
    if (next_beat_us == 0 || tempo_x100 == 0) {
        return (uint8_t)(latest_beat_in_bar % quantum);
    }
    // We are still in the beat that was current at packet send_time.
    if (coord_now_us < next_beat_us) {
        return (uint8_t)(latest_beat_in_bar % quantum);
    }
    // 60_000_000 us / (tempo_x100 / 100) == 6_000_000_000 / tempo_x100
    int64_t beat_period_us = (int64_t)6000000000LL / (int64_t)tempo_x100;
    if (beat_period_us <= 0) {
        return (uint8_t)(latest_beat_in_bar % quantum);
    }
    int64_t advance = 1 + (coord_now_us - next_beat_us) / beat_period_us;
    int64_t beat = ((int64_t)latest_beat_in_bar + advance) % (int64_t)quantum;
    if (beat < 0) beat += quantum;
    return (uint8_t)beat;
}
