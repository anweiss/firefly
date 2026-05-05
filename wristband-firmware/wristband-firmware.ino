/**
 * Firefly Wristband — ESP-NOW beat-synced LED driver (protocol v2)
 *
 * Runs on XIAO ESP32-C3. Receives beat timing packets from the
 * coordinator dongle over ESP-NOW, flashes WS2813 LEDs on every beat
 * with downbeat accents.
 *
 * Flash via Arduino IDE with board: "XIAO_ESP32C3"
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <FastLED.h>
#include "../shared/protocol.h"
#include "../shared/oled_display.h"
#include "../shared/beat_clock.h"

// ── Hardware config ─────────────────────────────────────────────────

#define LED_PIN       D0          // GPIO connected to WS2813 data-in
#define NUM_LEDS      10          // Grove RGB LED Stick has 10 LEDs
#define LED_TYPE      WS2813
#define COLOR_ORDER   GRB
#define MAX_BRIGHTNESS 255        // full brightness — very bright, very clear

// ESP-NOW channel — must match dongle.
// Channel 6 chosen as a mid-spectrum compromise: avoids ch 1/11 which
// are typically saturated by neighbour APs and ch 11 had visible
// reception failures through walls in testing.
#define ESPNOW_CHANNEL 11

// ── LED colors ──────────────────────────────────────────────────────

#define COLOR_DOWNBEAT  CRGB(255, 80, 0)    // warm orange for downbeats
#define COLOR_BEAT      CRGB(0, 120, 255)   // cool blue for other beats
#define COLOR_CDJ_BEAT  CRGB(80, 255, 80)   // green when CDJ is driving
#define COLOR_OFF       CRGB(0, 0, 0)

// ── Flash timing ────────────────────────────────────────────────────

#define FLASH_DURATION_MS  40     // LED on-time per beat (sharp on→off, no fade)

// ── Clock offset tracking (EMA) ─────────────────────────────────────

static volatile int64_t clock_offset_us = 0;    // coordinator_time - local_time
static volatile bool    offset_initialized = false;
static const float      EMA_ALPHA = 0.1f;

// ── Shared state (written in ISR context, read in loop) ─────────────

static volatile int64_t  latest_next_downbeat_us = 0;
static volatile int64_t  latest_next_beat_us = 0;
static volatile uint16_t latest_tempo_x100 = 0;
static volatile uint8_t  latest_beat_in_bar = 0;
static volatile uint8_t  latest_flags = 0;
static volatile uint8_t  latest_on_air_mask = 0;
static volatile uint8_t  latest_master_device = 0;
static volatile uint32_t packets_received = 0;
static volatile uint32_t last_packet_local_us = 0;
static volatile uint32_t last_packet_ms = 0;

// ── LED state ───────────────────────────────────────────────────────

static CRGB leds[NUM_LEDS];

// Flash state machine (declared early because the ESP-NOW receive
// callback fires the LED directly for minimum-latency beat response).
enum flash_state_t { FLASH_IDLE, FLASH_ON };
static volatile flash_state_t flash_state = FLASH_IDLE;
static volatile uint32_t flash_start_ms = 0;
static uint32_t last_led_show_ms = 0;
static CRGB     flash_color = CRGB::Black;

// Reactive-flash tracking (also driven from the receive callback).
static volatile uint8_t  last_fired_beat_in_bar = 0xFF;
static volatile uint32_t last_fired_packet_count = 0;
static volatile uint8_t  pinned_beat_in_bar = 0;
static volatile int64_t  pinned_beat_until_local_us = 0;

// Inter-flash debounce. Real musical tempos top out around 220 BPM
// (~273 ms/beat), so any two flashes closer than 150 ms apart must be
// caused by upstream glitches — e.g. the coordinator restarting and
// burst-replaying packets with stale timestamps while it re-syncs to
// DJ Link. Without this guard, a burst of N stale packets each
// schedules a near-immediate flash, and the LED appears to alternate
// colors with no OFF transition in between (because each new flash
// fires before flash_tick has elapsed FLASH_DURATION_MS).
#define MIN_INTER_FLASH_MS 150
static volatile uint32_t last_flash_fire_ms = 0;

// Scheduled-flash mechanism. The coordinator's broadcast tick (5 ms at
// 200 Hz) means a bib-change-on-arrival flash trigger has up to ~5 ms
// of quantization jitter relative to the true musical beat boundary.
// Instead, we read the precise `next_beat_us` timestamp out of every
// packet, translate it into the local clock domain, and fire the LED
// at that exact moment from a tight poll loop. Sub-millisecond
// accuracy regardless of broadcast tick rate.
static volatile int64_t  scheduled_flash_at_local_us = 0;  // 0 = none scheduled
static volatile uint8_t  scheduled_flash_bib = 0;          // bib value at the scheduled beat
static volatile bool     scheduled_flash_is_immediate = false;

// ── OLED state ──────────────────────────────────────────────────────

static oled_display_t oled;
static uint32_t last_oled_ms = 0;
static TaskHandle_t oled_task_handle = nullptr;

// ── Pairing/hello state ─────────────────────────────────────────────
//
// When ESP-NOW broadcast RX is unreliable on this platform (notably
// the ESP32-C3 USB Serial/JTAG-vs-WiFi interaction, where broadcast
// frames can fail to be dispatched to the recv callback when there is
// no enumerated USB host), we fall back to unicast: the wristband
// announces itself with a tiny "hello" frame the dongle uses to add it
// as a unicast peer, and the dongle then forwards data to that peer
// directly. Unicast ESP-NOW has 802.11 ACK + auto-retry, vastly more
// reliable than broadcast which has neither.
static volatile uint32_t hellos_sent = 0;
static volatile uint32_t hellos_send_ok = 0;
static volatile uint32_t hellos_send_fail = 0;
static uint32_t last_hello_ms = 0;

// ── Helpers ─────────────────────────────────────────────────────────

static int64_t local_time_us() {
    return (int64_t)esp_timer_get_time();
}

// Convert coordinator timestamp to local clock domain
static int64_t to_local_time(int64_t coordinator_us) {
    return coordinator_us - clock_offset_us;
}

// ── ESP-NOW send callback ──────────────────────────────────────────
static void on_send(const esp_now_send_info_t *, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) hellos_send_ok++;
    else hellos_send_fail++;
}

// Build + transmit an 8-byte broadcast hello frame announcing this
// wristband's MAC. The dongle listens for these and adds the sender
// as a unicast peer.
static void send_hello() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    uint8_t frame[FIREFLY_HELLO_SIZE];
    frame[0] = FIREFLY_HELLO_SYNC_0;
    frame[1] = FIREFLY_HELLO_SYNC_1;
    memcpy(&frame[2], mac, 6);
    static const uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    if (esp_now_send(bcast, frame, sizeof(frame)) == ESP_OK) {
        hellos_sent++;
    }
}

// ── ESP-NOW receive callback (runs in WiFi task context) ────────────

// Track ALL frames arriving (any length) — separate from packets_received which filters
static volatile uint32_t any_frames = 0;

void IRAM_ATTR on_receive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    any_frames++;
    if (len != FIREFLY_PACKET_SIZE) return;

    // Validate sync bytes
    if (data[0] != FIREFLY_SYNC_0 || data[1] != FIREFLY_SYNC_1) return;

    // Validate version
    if (data[2] != FIREFLY_VERSION) return;

    // Validate CRC: over bytes [2..35)
    uint8_t expected_crc = firefly_crc8(&data[2], FIREFLY_PACKET_SIZE - 3);
    if (data[FIREFLY_PACKET_SIZE - 1] != expected_crc) return;

    // Parse fields (little-endian)
    int64_t send_time_us;
    int64_t next_downbeat_us;
    uint16_t tempo_bpm_x100;
    int64_t next_beat_us;

    memcpy(&send_time_us,      &data[4],  sizeof(int64_t));
    memcpy(&next_downbeat_us,  &data[12], sizeof(int64_t));
    memcpy(&tempo_bpm_x100,    &data[20], sizeof(uint16_t));
    memcpy(&next_beat_us,      &data[24], sizeof(int64_t));

    uint8_t beat_in_bar    = data[22];
    uint8_t flags          = data[23];
    uint8_t on_air_mask    = data[32];
    uint8_t master_device  = data[33];

    // ── Clock offset update (EMA filter) ────────────────────────────
    int64_t local_now = local_time_us();
    int64_t measured_offset = send_time_us - local_now;

    if (!offset_initialized) {
        clock_offset_us = measured_offset;
        offset_initialized = true;
    } else {
        clock_offset_us = (int64_t)(
            EMA_ALPHA * (float)measured_offset +
            (1.0f - EMA_ALPHA) * (float)clock_offset_us
        );
    }

    // ── Update shared state ─────────────────────────────────────────
    latest_next_downbeat_us = next_downbeat_us;
    latest_next_beat_us     = next_beat_us;
    latest_tempo_x100       = tempo_bpm_x100;
    latest_beat_in_bar      = beat_in_bar;
    latest_flags            = flags;
    latest_on_air_mask      = on_air_mask;
    latest_master_device    = master_device;
    last_packet_local_us    = (uint32_t)(local_now & 0xFFFFFFFF);
    last_packet_ms          = millis();
    packets_received++;

    // ── Beat flash trigger ──────────────────────────────────────────
    // Two paths:
    //  1. Schedule a flash at the precise `next_beat_us` time (in
    //     local clock domain). This eliminates the 0–5 ms quantization
    //     error that comes from waiting for a bib-change-on-arrival.
    //  2. Bib-change fallback for safety: if a packet carries a new
    //     bib but the corresponding next_beat_us has already passed
    //     (catch-up after a stall, or coordinator emitted an
    //     immediate-fire packet with next_beat_us≈send_time_us), fire
    //     right now.
    bool is_playing = (flags & FIREFLY_FLAG_PLAYING) != 0;
    if (is_playing) {
        if (last_fired_beat_in_bar == 0xFF) {
            // First packet after boot/resume — adopt without firing.
            last_fired_beat_in_bar = beat_in_bar;
            last_fired_packet_count = packets_received;
        }

        // Schedule the *next* beat (the one whose timestamp is in the
        // packet's next_beat_us field). The bib at that beat will be
        // (current bib + 1) % 4.
        int64_t flash_at_local = to_local_time(next_beat_us);
        uint8_t next_bib = (uint8_t)((beat_in_bar + 1) % 4);
        // Only update the schedule if the new target is meaningfully
        // different — avoids re-arming for the same beat on every
        // packet (which would be harmless but wasteful).
        int64_t now_local = local_now;
        // Sanity-bound: must be within the next ~2 beats. Otherwise
        // the timestamp is stale or corrupted.
        if (flash_at_local > now_local - 2000 &&
            flash_at_local < now_local + 1500000) {
            scheduled_flash_at_local_us = flash_at_local;
            scheduled_flash_bib = next_bib;
            scheduled_flash_is_immediate = (flash_at_local <= now_local + 2000);
        }

        // Bib-change-on-arrival fallback. Catches the case where the
        // coordinator advanced bib (tick filler or play/cue press
        // immediate-fire) without a future next_beat_us — fire now.
        // Guard: only fire if the bib advanced forward by 1. After the
        // scheduled flash fires (last_fired = target_bib), a stale
        // packet built pre-promotion may carry the old bib; treating
        // that as a new beat would cause a double-fire on the prior
        // beat. Forward-by-1 is the only legitimate transition.
        if (beat_in_bar != last_fired_beat_in_bar) {
            uint8_t expected_next = (uint8_t)((last_fired_beat_in_bar + 1) % 4);
            bool forward_by_one = (last_fired_beat_in_bar == 0xFF) ||
                                  (beat_in_bar == expected_next);
            // If we've already scheduled a near-future flash for this
            // bib, let the schedule handle it; don't double-fire.
            bool scheduled_for_this_bib =
                (scheduled_flash_at_local_us > now_local) &&
                (scheduled_flash_bib == beat_in_bar);
            if (forward_by_one && !scheduled_for_this_bib &&
                (millis() - last_flash_fire_ms) >= MIN_INTER_FLASH_MS) {
                bool cdj_active = (flags & FIREFLY_FLAG_CDJ_ACTIVE) != 0;
                bool is_downbeat = (beat_in_bar == 0);

                CRGB color;
                if (is_downbeat) {
                    color = COLOR_DOWNBEAT;
                } else if (cdj_active) {
                    color = COLOR_CDJ_BEAT;
                } else {
                    color = COLOR_BEAT;
                }

                flash_color = color;
                flash_start_ms = millis();
                flash_state = FLASH_ON;
                fill_solid(leds, NUM_LEDS, color);
                FastLED.show();
                last_led_show_ms = millis();
                last_flash_fire_ms = millis();

                pinned_beat_in_bar = beat_in_bar;
                pinned_beat_until_local_us = local_now + 250000;
            }

            // Only latch when forward-by-one — a stale-packet bib
            // (running behind our scheduled fire) must NOT clobber
            // the latch, otherwise the next forward transition gets
            // misclassified as backward.
            if (forward_by_one) {
                last_fired_beat_in_bar = beat_in_bar;
                last_fired_packet_count = packets_received;
            }
        }
    } else {
        // Not playing — clear any pending scheduled flash so we don't
        // fire after pause.
        scheduled_flash_at_local_us = 0;
    }

    // Wake the OLED task to redraw the beat counter in lockstep with
    // the LED. The callback runs on the WiFi task (not an ISR), so
    // the regular xTaskNotifyGive is correct.
    if (oled_task_handle) {
        xTaskNotifyGive(oled_task_handle);
    }
}

// ── LED animation (non-blocking) ────────────────────────────────────
//
// Previous implementation did delay(80) + fade_out() with delay(8) per
// step = ~160ms of blocking time per beat with FastLED IRQ-disabled
// sections — starved ESP-NOW RX on single-core C3 causing periodic
// freezes. Refactored to a state machine driven by millis().

#define FADE_DURATION_MS  0       // no fade — LED snaps off right after beat
#define LED_MIN_SHOW_MS   12      // (unused with no-fade path; kept for future)

static inline void flash_leds(CRGB color) {
    fill_solid(leds, NUM_LEDS, color);
    FastLED.show();
    last_led_show_ms = millis();
}

// Advance the flash state machine. Call every loop iteration.
// Sharp on→off behavior: LEDs go full-bright on beat, snap off after
// FLASH_DURATION_MS. No fade — keeps each beat crisp and unambiguous.
static void flash_tick() {
    if (flash_state == FLASH_IDLE) return;

    uint32_t elapsed = millis() - flash_start_ms;

    if (elapsed >= FLASH_DURATION_MS) {
        fill_solid(leds, NUM_LEDS, COLOR_OFF);
        FastLED.show();
        last_led_show_ms = millis();
        flash_state = FLASH_IDLE;
    }
}

// ── Setup ───────────────────────────────────────────────────────────

static void oled_task(void *);

void setup() {
    Serial.begin(115200);
    // Make CDC writes non-blocking — when there is no USB host,
    // a stalled TX would otherwise stall setup() up to 1s per call.
    // Only available on the HWCDC class (ESP32 Arduino core 3.x with
    // USB CDC on boot), which is what `Serial` is on the XIAO C3.
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);
#endif
    Serial.println("Firefly Wristband v0.2 (protocol v2)");

    // Pin CPU at 160 MHz (max for C3) so WiFi/ESP-NOW timing is stable
    // across power sources. Default DFS can drop to 80 MHz under no-load.
    setCpuFrequencyMhz(160);

    // Init LEDs
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS)
        .setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(MAX_BRIGHTNESS);
    fill_solid(leds, NUM_LEDS, COLOR_OFF);
    FastLED.show();

    // Startup flash — confirms LEDs work
    flash_leds(CRGB::Green);
    delay(200);
    fill_solid(leds, NUM_LEDS, COLOR_OFF);
    FastLED.show();

    // Free the radio from BT coexistence scheduler — we only use WiFi.
    btStop();

    // Init WiFi for ESP-NOW (no association)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);
    delay(100);  // give the radio time to settle before we configure it
    esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
    // Enable standard 802.11 b/g/n. Long-range mode (WIFI_PROTOCOL_LR)
    // was tested but caused irregular packet arrival patterns that
    // broke the beat-flash logic; with the IPEX antennas installed,
    // stock b/g/n at max TX power gives plenty of range for our use.
    esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    // Crank TX power to the ESP32-C3 datasheet maximum (≈19.5 dBm).
    // Default is ~8.5 dBm, so this is ~10× the radiated power.
    esp_err_t pwr_err = esp_wifi_set_max_tx_power(78);
    // Set a permissive country so channel 6 + max TX power are allowed
    // regardless of region defaults baked into the chip.
    wifi_country_t country = {};
    memcpy(country.cc, "01", 2);
    country.schan = 1;
    country.nchan = 13;
    country.policy = WIFI_COUNTRY_POLICY_MANUAL;
    esp_wifi_set_country(&country);
    esp_err_t ch_err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Init ESP-NOW
    esp_err_t now_err = esp_now_init();
    if (now_err != ESP_OK) {
        Serial.printf("ESP-NOW init FAILED: %d\n", now_err);
        flash_leds(CRGB::Red);
        while (1) { delay(1000); }
    }

    esp_now_register_recv_cb(on_receive);
    esp_now_register_send_cb(on_send);

    // Register broadcast peer (needed for RX on ESP32-C3 in some Arduino core versions)
    const uint8_t BROADCAST_ADDR[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_err_t peer_err = esp_now_add_peer(&peer);

    // Pin ESP-NOW broadcast TX rate to the most robust 802.11
    // modulation — 11b at 1 Mbps with long preamble. ~7 dB better
    // receiver sensitivity than higher rates and standard 802.11
    // (not Espressif-proprietary like LR), so packets arrive on
    // predictable beat-rate timing.
    esp_now_rate_config_t rate_cfg = {};
    rate_cfg.phymode = WIFI_PHY_MODE_11B;
    rate_cfg.rate = WIFI_PHY_RATE_1M_L;
    rate_cfg.ersu = false;
    rate_cfg.dcm = false;
    esp_now_set_peer_rate_config(BROADCAST_ADDR, &rate_cfg);

    Serial.println("ESP-NOW ready — waiting for beats");
    Serial.printf("init codes: ps=%d pwr=%d ch=%d now=%d peer=%d\n",
        ps_err, pwr_err, ch_err, now_err, peer_err);

    // Diagnostic: print MAC and channel
    uint8_t mac[6];
    WiFi.macAddress(mac);
    uint8_t primary; wifi_second_chan_t second;
    esp_wifi_get_channel(&primary, &second);
    Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X  channel: %d\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], primary);

    // OLED on expansion board (optional)
    bool oled_ok = firefly_oled_begin(&oled);
    if (oled_ok) {
        char chbuf[8];
        snprintf(chbuf, sizeof(chbuf), "%d", primary);
        char macbuf[16];
        snprintf(macbuf, sizeof(macbuf), "%02X%02X%02X",
                 mac[3], mac[4], mac[5]);
        firefly_oled_clear(&oled);
        firefly_oled_header(&oled, "Firefly WB");
        firefly_oled_kv(&oled, 0, "ch ", chbuf);
        firefly_oled_kv(&oled, 1, "st ", "waiting");
        firefly_oled_kv(&oled, 2, "mac", macbuf);
        firefly_oled_flush(&oled);
    }
    Serial.printf("OLED: %s\n", oled_ok ? "ok" : "absent");

    // OLED on its own FreeRTOS task — keeps I2C blocking off the
    // main loop and out of the WiFi/ESP-NOW receive callback.
    if (oled_ok) {
        xTaskCreate(oled_task, "oled", 4096, nullptr, 1, &oled_task_handle);
    }
}

// ── Main loop ───────────────────────────────────────────────────────

// ── Main loop ───────────────────────────────────────────────────────
//
// The reactive beat-flash trigger now lives in the ESP-NOW receive
// callback (see `on_receive` above), so the loop is responsible only
// for advancing the flash OFF transition, the idle indicator, and OLED
// refresh — none of which are timing-critical for the LED beat.

static uint32_t last_status_ms = 0;

// Stream is "live" when a packet arrived within the recent past. When
// the stream goes quiet we stop extrapolating beats — both the flash
// scheduler and the OLED beat counter freeze instead of free-running
// off the last-known timing.
#define STREAM_TIMEOUT_MS  500U

static inline bool stream_is_live() {
    return packets_received > 0 &&
           (millis() - last_packet_ms) < STREAM_TIMEOUT_MS;
}

// ── OLED background task ───────────────────────────────────────────
//
// Runs OLED I2C writes (~20–40ms per refresh) on a dedicated FreeRTOS
// task so they cannot delay the WiFi-task-driven LED flash callback or
// the main loop's housekeeping. Galloping beat cadence was traced to
// OLED I2C blocking the main loop in lockstep with the beat-edge
// packet arrivals.
static void oled_task(void *) {
    static uint8_t last_displayed_beat = 0xFF;
    for (;;) {
        if (oled.present) {
            bool live = stream_is_live();
            bool is_playing_now = (latest_flags & FIREFLY_FLAG_PLAYING) != 0;
            uint8_t cur_beat = latest_beat_in_bar;

            char line[24];
            firefly_oled_clear(&oled);
            firefly_oled_header(&oled, "Firefly WB");

            float tempo = latest_tempo_x100 / 100.0f;
            bool cdj_active = (latest_flags & FIREFLY_FLAG_CDJ_ACTIVE) != 0;
            const char *src = !live ? "idle"
                              : (cdj_active ? "CDJ " : (is_playing_now ? "LINK" : "idle"));
            snprintf(line, sizeof(line), "%5.1f %s b%u", tempo, src,
                     (unsigned)(cur_beat + 1));
            firefly_oled_kv(&oled, 0, "bpm", line);

            snprintf(line, sizeof(line), "%lu/%lu",
                     (unsigned long)packets_received, (unsigned long)any_frames);
            firefly_oled_kv(&oled, 1, "rx ", line);

            long off_ms = (long)(clock_offset_us / 1000);
            snprintf(line, sizeof(line), "%ld ms", off_ms);
            firefly_oled_kv(&oled, 2, "off", line);

            snprintf(line, sizeof(line), "any %lu  h %lu",
                     (unsigned long)any_frames,
                     (unsigned long)hellos_send_ok);
            firefly_oled_kv(&oled, 3, "rf ", line);

            firefly_oled_flush(&oled);
            last_displayed_beat = cur_beat;
            (void)last_displayed_beat;
            last_oled_ms = millis();
        }
        // Sleep until the receive callback notifies us of a new
        // packet, or 250 ms elapses for periodic refresh of static
        // fields. Notification-driven wakeup keeps the OLED beat
        // counter advancing in lockstep with the LED flash (which
        // also fires from the receive callback).
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
    }
}

void loop() {
    bool live = stream_is_live();

    // Periodic hello broadcast for unicast pairing with the dongle.
    // Sends every 1s when no data has been received yet (dongle hasn't
    // paired us) or stream has gone idle (dongle may have rebooted).
    // Once data is flowing live, drops to a slower keepalive (5s).
    {
        uint32_t now_ms = millis();
        uint32_t interval = live ? 5000 : 1000;
        if (now_ms - last_hello_ms >= interval) {
            send_hello();
            last_hello_ms = now_ms;
        }
    }

    // If the stream has gone quiet, clear the latch so the first beat
    // after re-acquisition adopts cleanly without firing.
    if (!live) {
        last_fired_beat_in_bar = 0xFF;
        scheduled_flash_at_local_us = 0;
    }

    // ── Scheduled beat flash (sub-ms accurate) ──────────────────────
    // Polled here at full loop rate (no delay in loop body → ~µs
    // resolution). Fires the LED at the precise next_beat_us moment
    // received in the most-recent packet, eliminating the broadcast-
    // tick quantization jitter that bib-change-on-arrival had.
    if (scheduled_flash_at_local_us != 0) {
        int64_t now_local = local_time_us();
        if (now_local >= scheduled_flash_at_local_us) {
            uint8_t target_bib = scheduled_flash_bib;
            int64_t fire_local_now = now_local;
            scheduled_flash_at_local_us = 0;  // consume one-shot

            // Avoid double-firing if the bib-change fallback already
            // fired for this same beat. Also debounce against
            // unrealistically rapid flashes.
            if (target_bib != last_fired_beat_in_bar &&
                (millis() - last_flash_fire_ms) >= MIN_INTER_FLASH_MS) {
                uint8_t cur_flags = latest_flags;
                bool cdj_active = (cur_flags & FIREFLY_FLAG_CDJ_ACTIVE) != 0;
                bool is_downbeat = (target_bib == 0);

                CRGB color;
                if (is_downbeat) {
                    color = COLOR_DOWNBEAT;
                } else if (cdj_active) {
                    color = COLOR_CDJ_BEAT;
                } else {
                    color = COLOR_BEAT;
                }

                flash_color = color;
                flash_start_ms = millis();
                flash_state = FLASH_ON;
                fill_solid(leds, NUM_LEDS, color);
                FastLED.show();
                last_led_show_ms = millis();
                last_flash_fire_ms = millis();

                pinned_beat_in_bar = target_bib;
                pinned_beat_until_local_us = fire_local_now + 250000;

                // Latch the bib we just fired so the next packet's
                // bib-change fallback sees a match (no double-fire)
                // and the OLED counter advances in step.
                last_fired_beat_in_bar = target_bib;
                latest_beat_in_bar = target_bib;
                if (oled_task_handle) {
                    xTaskNotifyGive(oled_task_handle);
                }
            }
        }
    }

    // Advance LED off-transition state machine (non-blocking).
    flash_tick();

    // ── Idle indicator ──────────────────────────────────────────────
    // Dim pulse if no packets for >3 seconds. Skip while a flash is
    // animating to avoid stomping on the beat color.
    static uint32_t last_idle_ms = 0;
    if (flash_state == FLASH_IDLE &&
        (packets_received == 0 ||
        (millis() - (last_packet_local_us / 1000)) > 3000) &&
        (millis() - last_idle_ms) >= 100) {
        last_idle_ms = millis();
        uint8_t brightness = (uint8_t)(20 + 10 * sin(millis() / 500.0));
        fill_solid(leds, NUM_LEDS, CRGB(0, 0, brightness));
        FastLED.show();
    }

    // ── Periodic status (serial debug) ──────────────────────────────
    if (millis() - last_status_ms > 5000) {
        last_status_ms = millis();
        float tempo = latest_tempo_x100 / 100.0f;
        bool cdj_active = (latest_flags & FIREFLY_FLAG_CDJ_ACTIVE) != 0;
        Serial.printf(
            "pkts: %lu  any: %lu  offset: %lld us  tempo: %.1f  beat: %d  "
            "playing: %s  cdj: %s  master: %d  air: 0x%02X\n",
            packets_received,
            any_frames,
            clock_offset_us,
            tempo,
            latest_beat_in_bar,
            (latest_flags & FIREFLY_FLAG_PLAYING) ? "yes" : "no",
            cdj_active ? "yes" : "no",
            latest_master_device,
            latest_on_air_mask
        );
    }

    delay(1);
}
