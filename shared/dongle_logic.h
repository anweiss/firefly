#pragma once
#include "protocol.h"
#include <stdbool.h>
#include <string.h>

typedef enum {
    DONGLE_WAIT_SYNC_0,
    DONGLE_WAIT_SYNC_1,
    DONGLE_READ_PAYLOAD,
} dongle_rx_state_t;

typedef struct {
    dongle_rx_state_t rx_state;
    uint8_t rx_buf[FIREFLY_PACKET_SIZE];
    uint8_t rx_idx;
    uint32_t packets_forwarded;
    uint32_t crc_errors;
    uint32_t version_errors;
} dongle_framer_t;

static inline void dongle_framer_init(dongle_framer_t *f) {
    f->rx_state = DONGLE_WAIT_SYNC_0;
    f->rx_idx = 0;
    f->packets_forwarded = 0;
    f->crc_errors = 0;
    f->version_errors = 0;
    memset(f->rx_buf, 0, sizeof(f->rx_buf));
}

// Feed one byte. Returns true if a valid packet is now in f->rx_buf.
static inline bool dongle_framer_feed(dongle_framer_t *f, uint8_t b) {
    switch (f->rx_state) {
        case DONGLE_WAIT_SYNC_0:
            if (b == FIREFLY_SYNC_0) {
                f->rx_buf[0] = b;
                f->rx_state = DONGLE_WAIT_SYNC_1;
            }
            return false;

        case DONGLE_WAIT_SYNC_1:
            if (b == FIREFLY_SYNC_1) {
                f->rx_buf[1] = b;
                f->rx_idx = 2;
                f->rx_state = DONGLE_READ_PAYLOAD;
            } else {
                if (b == FIREFLY_SYNC_0) {
                    f->rx_buf[0] = b;
                    f->rx_state = DONGLE_WAIT_SYNC_1;
                } else {
                    f->rx_state = DONGLE_WAIT_SYNC_0;
                }
            }
            return false;

        case DONGLE_READ_PAYLOAD:
            f->rx_buf[f->rx_idx++] = b;

            if (f->rx_idx >= FIREFLY_PACKET_SIZE) {
                bool valid = false;

                if (f->rx_buf[2] != FIREFLY_VERSION) {
                    f->version_errors++;
                } else {
                    uint8_t expected_crc = firefly_crc8(&f->rx_buf[2], FIREFLY_PACKET_SIZE - 3);
                    if (f->rx_buf[FIREFLY_PACKET_SIZE - 1] == expected_crc) {
                        f->packets_forwarded++;
                        valid = true;
                    } else {
                        f->crc_errors++;
                    }
                }

                f->rx_state = DONGLE_WAIT_SYNC_0;
                f->rx_idx = 0;
                return valid;
            }
            return false;
    }
    return false;
}
