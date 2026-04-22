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
#include "../shared/oled_display.h"

// ESP-NOW broadcast address
static const uint8_t BROADCAST_ADDR[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ESP-NOW WiFi channel — must match wristbands.
// Ch 11 chosen to avoid congestion from common APs on ch 1/6.
#define ESPNOW_CHANNEL 11

// Serial framing state machine (extracted to shared/dongle_logic.h)
static dongle_framer_t framer;

static uint32_t last_status_ms = 0;
static uint32_t last_oled_ms   = 0;

// Latest BPM from most recently forwarded packet (parsed after frame success)
static uint16_t latest_tempo_x100 = 0;
static uint8_t  latest_beat_in_bar = 0;
static uint32_t last_fwd_seen = 0;

static oled_display_t oled;

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

    // OLED on expansion board (optional — continues if not present)
    bool oled_ok = firefly_oled_begin(&oled);
    if (oled_ok) {
        firefly_oled_clear(&oled);
        firefly_oled_header(&oled, "Firefly DNG");
        firefly_oled_kv(&oled, 0, "ch", String(ESPNOW_CHANNEL).c_str());
        firefly_oled_kv(&oled, 1, "st", "waiting");
        firefly_oled_flush(&oled);
    }

    Serial.println("ESP-NOW ready — waiting for v2 packets");
    Serial.printf("OLED: %s\n", oled_ok ? "ok" : "absent");

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

            // Peek at tempo + beat_in_bar for OLED display
            memcpy(&latest_tempo_x100, &framer.rx_buf[20], sizeof(uint16_t));
            latest_beat_in_bar = framer.rx_buf[22];
            last_fwd_seen = millis();
        }
    }

    // Refresh OLED ~2Hz — minimize time spent blocking in I2C transfer
    if (millis() - last_oled_ms > 500) {
        last_oled_ms = millis();
        if (oled.present) {
            char line[24];
            firefly_oled_clear(&oled);
            firefly_oled_header(&oled, "Firefly DNG");

            float bpm = latest_tempo_x100 / 100.0f;
            bool live = (millis() - last_fwd_seen) < 1000 && last_fwd_seen != 0;
            // Keep under 128px (21 chars @ 6px default font)
            snprintf(line, sizeof(line), "%5.1f %s b%u",
                     bpm, live ? "LIVE" : "idle",
                     (unsigned)(latest_beat_in_bar + 1));
            firefly_oled_kv(&oled, 0, "bpm", line);

            snprintf(line, sizeof(line), "%lu", (unsigned long)framer.packets_forwarded);
            firefly_oled_kv(&oled, 1, "fwd", line);

            snprintf(line, sizeof(line), "%lu/%lu",
                     (unsigned long)send_ok, (unsigned long)send_fail);
            firefly_oled_kv(&oled, 2, "tx ", line);

            uint8_t pri; wifi_second_chan_t sec;
            esp_wifi_get_channel(&pri, &sec);
            snprintf(line, sizeof(line), "%u err c:%lu v:%lu",
                     pri,
                     (unsigned long)framer.crc_errors,
                     (unsigned long)framer.version_errors);
            firefly_oled_kv(&oled, 3, "ch ", line);

            firefly_oled_flush(&oled);
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
