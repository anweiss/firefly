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
#include "../shared/beat_clock.h"

#include <esp_timer.h>

// ESP-NOW broadcast address
static const uint8_t BROADCAST_ADDR[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ESP-NOW WiFi channel — must match wristbands.
// Channel 6 chosen as a mid-spectrum compromise: avoids ch 1/11 which
// are typically saturated by neighbour APs and ch 11 had visible
// reception failures through walls in testing.
#define ESPNOW_CHANNEL 6

// Serial framing state machine (extracted to shared/dongle_logic.h)
static dongle_framer_t framer;

static uint32_t last_status_ms = 0;
static uint32_t last_oled_ms   = 0;

// Latest BPM from most recently forwarded packet (parsed after frame success)
static uint16_t latest_tempo_x100 = 0;
static uint8_t  latest_beat_in_bar = 0;
static int64_t  latest_send_time_us = 0;   // coordinator clock
static int64_t  latest_next_beat_us = 0;   // coordinator clock
static int64_t  latest_packet_local_us = 0; // local esp_timer at packet arrival
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

// ── Unicast peer registry (auto-paired wristbands) ─────────────────
//
// Wristbands periodically broadcast a tiny "hello" frame announcing
// their MAC. We add each new MAC as a unicast ESP-NOW peer so we can
// forward data via 802.11-acked unicast (with retry) instead of
// broadcast (no ack, no retry, unreliable through walls and even more
// unreliable on ESP32-C3 when the wristband is on a non-host USB
// power source — the C3's USB-Serial/JTAG peripheral can interfere
// with broadcast RX dispatching).
#define MAX_PEERS 8
static uint8_t  peer_macs[MAX_PEERS][6];
static uint8_t  peer_count = 0;
static uint32_t hellos_rx = 0;
static uint32_t any_rx = 0;

static bool peer_known(const uint8_t *mac) {
    for (uint8_t i = 0; i < peer_count; i++) {
        if (memcmp(peer_macs[i], mac, 6) == 0) return true;
    }
    return false;
}

static void add_unicast_peer(const uint8_t *mac) {
    if (peer_count >= MAX_PEERS) return;
    if (peer_known(mac)) return;
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.channel = ESPNOW_CHANNEL;
    p.encrypt = false;
    p.ifidx = WIFI_IF_STA;
    if (esp_now_add_peer(&p) == ESP_OK) {
        memcpy(peer_macs[peer_count], mac, 6);
        peer_count++;
        // Pin per-peer rate to 11b 1Mbps for max sensitivity.
        esp_now_rate_config_t rc = {};
        rc.phymode = WIFI_PHY_MODE_11B;
        rc.rate = WIFI_PHY_RATE_1M_L;
        rc.ersu = false;
        rc.dcm = false;
        esp_now_set_peer_rate_config(mac, &rc);
    }
}

void IRAM_ATTR on_dongle_recv(const esp_now_recv_info_t *info,
                              const uint8_t *data, int len) {
    any_rx++;
    if (len == FIREFLY_HELLO_SIZE &&
        data[0] == FIREFLY_HELLO_SYNC_0 &&
        data[1] == FIREFLY_HELLO_SYNC_1) {
        hellos_rx++;
        // The hello carries the wristband MAC starting at byte 2,
        // but info->src_addr is the same value and is already in
        // a convenient 6-byte buffer.
        add_unicast_peer(info->src_addr);
    }
}

// ── Setup ───────────────────────────────────────────────────────────

static void oled_task(void *);
static TaskHandle_t oled_task_handle = nullptr;

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("Firefly Dongle v0.2 (protocol v2)");

    setCpuFrequencyMhz(160);
    btStop();

    // Init WiFi in station mode (no association — ESP-NOW only)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    delay(100);

    // Standard 802.11 b/g/n. LR mode caused bursty packet delivery
    // patterns that broke beat-flash timing; stock b/g/n at max TX
    // power + IPEX antennas gives plenty of range.
    esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    // Crank TX power to the ESP32-C3 datasheet maximum (≈19.5 dBm).
    esp_wifi_set_max_tx_power(78);

    // Permissive country so all channels + max TX are allowed.
    wifi_country_t country = {};
    memcpy(country.cc, "01", 2);
    country.schan = 1;
    country.nchan = 13;
    country.policy = WIFI_COUNTRY_POLICY_MANUAL;
    esp_wifi_set_country(&country);

    // Lock to a fixed channel
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED");
        while (1) { delay(1000); }
    }

    esp_now_register_send_cb(on_send);
    esp_now_register_recv_cb(on_dongle_recv);

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

    // Pin broadcast TX to 11b @ 1 Mbps for max receiver sensitivity
    // and predictable beat-rate timing. Unicast peers added at runtime
    // by the hello handler get the same rate config.
    esp_now_rate_config_t rate_cfg = {};
    rate_cfg.phymode = WIFI_PHY_MODE_11B;
    rate_cfg.rate = WIFI_PHY_RATE_1M_L;
    rate_cfg.ersu = false;
    rate_cfg.dcm = false;
    esp_now_set_peer_rate_config(BROADCAST_ADDR, &rate_cfg);

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

    // Start the OLED refresh task so I2C work cannot block the main
    // loop's USB→ESP-NOW forwarding path.
    if (oled.present) {
        xTaskCreate(oled_task, "oled", 4096, nullptr, 1, &oled_task_handle);
    }
}

// ── OLED background task ───────────────────────────────────────────
//
// OLED I2C writes take 20–40ms — far too long to run inline in the
// main loop, where they would block USB serial reads and delay
// beat-edge ESP-NOW forwarding by a variable amount, producing a
// visible "gallop" downstream at the wristband. Pinning the OLED to a
// dedicated FreeRTOS task lets the scheduler preempt it whenever USB
// or ESP-NOW work is pending, so forwarding latency stays bounded and
// constant.
static void oled_task(void *) {
    static uint8_t last_displayed_beat = 0xFF;
    for (;;) {
        if (oled.present) {
            uint8_t cur_beat = latest_beat_in_bar;
            bool live = (millis() - last_fwd_seen) < 1000 && last_fwd_seen != 0;

            char line[24];
            firefly_oled_clear(&oled);
            firefly_oled_header(&oled, "Firefly DNG");

            float bpm = latest_tempo_x100 / 100.0f;
            snprintf(line, sizeof(line), "%5.1f %s b%u",
                     bpm, live ? "LIVE" : "idle",
                     (unsigned)(cur_beat + 1));
            firefly_oled_kv(&oled, 0, "bpm", line);

            snprintf(line, sizeof(line), "%lu", (unsigned long)framer.packets_forwarded);
            firefly_oled_kv(&oled, 1, "fwd", line);

            snprintf(line, sizeof(line), "%lu/%lu",
                     (unsigned long)send_ok, (unsigned long)send_fail);
            firefly_oled_kv(&oled, 2, "tx ", line);

            uint8_t pri; wifi_second_chan_t sec;
            esp_wifi_get_channel(&pri, &sec);
            snprintf(line, sizeof(line), "%u p:%u h:%lu",
                     pri,
                     (unsigned)peer_count,
                     (unsigned long)hellos_rx);
            firefly_oled_kv(&oled, 3, "ch ", line);

            firefly_oled_flush(&oled);
            last_displayed_beat = cur_beat;
            (void)last_displayed_beat;
            last_oled_ms = millis();
        }
        // Sleep until a packet arrives (notification from loop()) or
        // 250 ms elapses for periodic refresh of static fields.
        // Notification-driven wakeup means the OLED counter advances
        // within ≈1 ms of the new beat_in_bar landing, matching the
        // wristband's LED-from-callback latency.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
    }
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
            // Re-stamp packet timestamps to the dongle's local clock
            // immediately before TX. This eliminates USB CDC jitter
            // from the wristband's clock-offset calculation: the
            // wristband only ever sees dongle_clock vs wristband_clock
            // (which has just sub-ms ESP-NOW air-time variance),
            // never the highly variable USB CDC + Serial buffering
            // latency between coordinator and dongle.
            //
            // We preserve the *deltas* between send_time and the
            // future next_beat / next_downbeat times (those are what
            // the wristband actually schedules against), and re-base
            // them onto the dongle's `esp_timer_get_time()` clock.
            int64_t coord_send, coord_next_beat, coord_next_db;
            memcpy(&coord_send,      &framer.rx_buf[4],  sizeof(int64_t));
            memcpy(&coord_next_db,   &framer.rx_buf[12], sizeof(int64_t));
            memcpy(&coord_next_beat, &framer.rx_buf[24], sizeof(int64_t));

            int64_t dongle_now = esp_timer_get_time();
            int64_t delta_beat = (coord_next_beat == 0) ? 0 : (coord_next_beat - coord_send);
            int64_t delta_db   = (coord_next_db   == 0) ? 0 : (coord_next_db   - coord_send);

            int64_t dongle_send_time     = dongle_now;
            int64_t dongle_next_beat_us  = (coord_next_beat == 0) ? 0 : (dongle_now + delta_beat);
            int64_t dongle_next_db_us    = (coord_next_db   == 0) ? 0 : (dongle_now + delta_db);

            memcpy(&framer.rx_buf[4],  &dongle_send_time,    sizeof(int64_t));
            memcpy(&framer.rx_buf[12], &dongle_next_db_us,   sizeof(int64_t));
            memcpy(&framer.rx_buf[24], &dongle_next_beat_us, sizeof(int64_t));

            // Recompute CRC over bytes [2..35) since we mutated the
            // payload.
            framer.rx_buf[FIREFLY_PACKET_SIZE - 1] =
                firefly_crc8(&framer.rx_buf[2], FIREFLY_PACKET_SIZE - 3);

            esp_now_send(BROADCAST_ADDR, framer.rx_buf, FIREFLY_PACKET_SIZE);
            // Also send unicast to each paired wristband. Unicast has
            // 802.11 ACK + retry at the MAC layer, so this carries the
            // load when broadcast can't be reliably received (e.g. C3
            // wristband on alt power, where broadcast RX dispatch is
            // blocked by USB-Serial/JTAG-vs-WiFi interference).
            for (uint8_t i = 0; i < peer_count; i++) {
                esp_now_send(peer_macs[i], framer.rx_buf, FIREFLY_PACKET_SIZE);
            }

            uint8_t prev_bib = latest_beat_in_bar;
            memcpy(&latest_tempo_x100, &framer.rx_buf[20], sizeof(uint16_t));
            latest_beat_in_bar = framer.rx_buf[22];
            latest_send_time_us = dongle_send_time;
            latest_next_beat_us = dongle_next_beat_us;
            latest_packet_local_us = dongle_now;
            last_fwd_seen = millis();

            // Wake the OLED task on every beat-in-bar change so the
            // counter redraws in lockstep with the wristband's LED.
            if (oled_task_handle && latest_beat_in_bar != prev_bib) {
                xTaskNotifyGive(oled_task_handle);
            }
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
