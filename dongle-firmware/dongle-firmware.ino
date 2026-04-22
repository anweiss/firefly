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
#include "../shared/dongle_logic.h"

// ESP-NOW broadcast address
static const uint8_t BROADCAST_ADDR[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ESP-NOW WiFi channel — must match wristbands.
// Ch 11 chosen to avoid congestion from common APs on ch 1/6.
#define ESPNOW_CHANNEL 11

// Serial framing state machine (extracted to shared/dongle_logic.h)
static dongle_framer_t framer;

static uint32_t last_status_ms = 0;

// Timeout: reset framing if partial packet stalls
static uint32_t last_byte_ms = 0;
#define FRAME_TIMEOUT_MS 50

// ── ESP-NOW callbacks ───────────────────────────────────────────────

static uint32_t send_ok = 0;
static uint32_t send_fail = 0;

void on_send(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        send_fail++;
    } else {
        send_ok++;
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
    delay(100);

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
    peer.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add broadcast peer");
        while (1) { delay(1000); }
    }

    dongle_framer_init(&framer);

    Serial.println("ESP-NOW ready — waiting for v2 packets");

    // Diagnostic: print MAC and channel
    uint8_t mac[6];
    WiFi.macAddress(mac);
    uint8_t primary; wifi_second_chan_t second;
    esp_wifi_get_channel(&primary, &second);
    Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X  channel: %d\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], primary);

    last_byte_ms = millis();
}

// ── Main loop ───────────────────────────────────────────────────────

void loop() {
    // Timeout partial frames
    if (framer.rx_state != DONGLE_WAIT_SYNC_0 && (millis() - last_byte_ms) > FRAME_TIMEOUT_MS) {
        framer.rx_state = DONGLE_WAIT_SYNC_0;
        framer.rx_idx = 0;
    }

    while (Serial.available()) {
        uint8_t b = Serial.read();
        last_byte_ms = millis();

        if (dongle_framer_feed(&framer, b)) {
            // Valid packet — forward over ESP-NOW
            esp_now_send(BROADCAST_ADDR, framer.rx_buf, FIREFLY_PACKET_SIZE);
        }
    }

    // Periodic status (every 5 seconds)
    if (millis() - last_status_ms > 5000) {
        last_status_ms = millis();
        uint8_t pri; wifi_second_chan_t sec;
        esp_wifi_get_channel(&pri, &sec);
        Serial.printf("fwd: %lu  send_ok: %lu  send_fail: %lu  crc_err: %lu  ver_err: %lu  ch: %d\n",
            framer.packets_forwarded, send_ok, send_fail, framer.crc_errors, framer.version_errors, pri);
    }
}
