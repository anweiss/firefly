/**
 * Firefly Dongle — USB-Serial → ESP-NOW bridge
 *
 * Runs on XIAO ESP32-C3. Reads Firefly v2 packets from USB serial,
 * validates framing + CRC, broadcasts over ESP-NOW.
 *
 * Flash via Arduino IDE with board: "XIAO_ESP32C3"
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "../shared/protocol.h"

// ESP-NOW broadcast address
static const uint8_t BROADCAST_ADDR[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ESP-NOW WiFi channel — must match wristbands
#define ESPNOW_CHANNEL 1

// Serial framing state machine
enum RxState {
    WAIT_SYNC_0,
    WAIT_SYNC_1,
    READ_PAYLOAD,
};

static RxState rx_state = WAIT_SYNC_0;
static uint8_t rx_buf[FIREFLY_PACKET_SIZE];
static uint8_t rx_idx = 0;

// Stats
static uint32_t packets_forwarded = 0;
static uint32_t crc_errors = 0;
static uint32_t version_errors = 0;
static uint32_t last_status_ms = 0;

// Timeout: reset framing if partial packet stalls
static uint32_t last_byte_ms = 0;
#define FRAME_TIMEOUT_MS 50

// ── ESP-NOW callbacks ───────────────────────────────────────────────

void on_send(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("ESP-NOW send failed");
    }
}

// ── Setup ───────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("Firefly Dongle v0.2 (protocol v2)");

    // Init WiFi in station mode (no association — ESP-NOW only)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Lock to a fixed channel
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED");
        while (1) { delay(1000); }
    }

    esp_now_register_send_cb(on_send);

    // Add broadcast peer
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add broadcast peer");
        while (1) { delay(1000); }
    }

    Serial.println("ESP-NOW ready — waiting for v2 packets");
    last_byte_ms = millis();
}

// ── Main loop ───────────────────────────────────────────────────────

void loop() {
    // Timeout partial frames
    if (rx_state != WAIT_SYNC_0 && (millis() - last_byte_ms) > FRAME_TIMEOUT_MS) {
        rx_state = WAIT_SYNC_0;
        rx_idx = 0;
    }

    while (Serial.available()) {
        uint8_t b = Serial.read();
        last_byte_ms = millis();

        switch (rx_state) {
            case WAIT_SYNC_0:
                if (b == FIREFLY_SYNC_0) {
                    rx_buf[0] = b;
                    rx_state = WAIT_SYNC_1;
                }
                break;

            case WAIT_SYNC_1:
                if (b == FIREFLY_SYNC_1) {
                    rx_buf[1] = b;
                    rx_idx = 2;
                    rx_state = READ_PAYLOAD;
                } else {
                    // False sync — check if this byte is a new SYNC_0
                    rx_state = (b == FIREFLY_SYNC_0) ? WAIT_SYNC_1 : WAIT_SYNC_0;
                    if (b == FIREFLY_SYNC_0) rx_buf[0] = b;
                }
                break;

            case READ_PAYLOAD:
                rx_buf[rx_idx++] = b;

                if (rx_idx >= FIREFLY_PACKET_SIZE) {
                    // Full packet received — validate version
                    if (rx_buf[2] != FIREFLY_VERSION) {
                        version_errors++;
                        rx_state = WAIT_SYNC_0;
                        rx_idx = 0;
                        break;
                    }

                    // Validate CRC: over bytes [2..35)
                    uint8_t expected_crc = firefly_crc8(&rx_buf[2], FIREFLY_PACKET_SIZE - 3);

                    if (rx_buf[FIREFLY_PACKET_SIZE - 1] == expected_crc) {
                        // Forward entire packet over ESP-NOW
                        esp_now_send(BROADCAST_ADDR, rx_buf, FIREFLY_PACKET_SIZE);
                        packets_forwarded++;
                    } else {
                        crc_errors++;
                    }

                    rx_state = WAIT_SYNC_0;
                    rx_idx = 0;
                }
                break;
        }
    }

    // Periodic status (every 5 seconds)
    if (millis() - last_status_ms > 5000) {
        last_status_ms = millis();
        Serial.printf("fwd: %lu  crc_err: %lu  ver_err: %lu\n",
            packets_forwarded, crc_errors, version_errors);
    }
}
