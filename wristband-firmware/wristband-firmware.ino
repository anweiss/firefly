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

// ── Hardware config ─────────────────────────────────────────────────

#define LED_PIN       D0          // GPIO connected to WS2813 data-in
#define NUM_LEDS      10          // Grove RGB LED Stick has 10 LEDs
#define LED_TYPE      WS2813
#define COLOR_ORDER   GRB
#define MAX_BRIGHTNESS 50         // 20-30% of 255 for battery life

// ESP-NOW channel — must match dongle.
// Ch 11 chosen to avoid congestion from common APs on ch 1/6.
#define ESPNOW_CHANNEL 11

// ── LED colors ──────────────────────────────────────────────────────

#define COLOR_DOWNBEAT  CRGB(255, 80, 0)    // warm orange for downbeats
#define COLOR_BEAT      CRGB(0, 120, 255)   // cool blue for other beats
#define COLOR_CDJ_BEAT  CRGB(80, 255, 80)   // green when CDJ is driving
#define COLOR_OFF       CRGB(0, 0, 0)

// ── Flash timing ────────────────────────────────────────────────────

#define FLASH_DURATION_MS  80     // LED on-time per beat
#define FADE_STEPS         10     // smooth fade-out steps

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

// ── LED state ───────────────────────────────────────────────────────

static CRGB leds[NUM_LEDS];

// ── OLED state ──────────────────────────────────────────────────────

static oled_display_t oled;
static uint32_t last_oled_ms = 0;

// ── Helpers ─────────────────────────────────────────────────────────

static int64_t local_time_us() {
    return (int64_t)esp_timer_get_time();
}

// Convert coordinator timestamp to local clock domain
static int64_t to_local_time(int64_t coordinator_us) {
    return coordinator_us - clock_offset_us;
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
    packets_received++;
}

// ── LED animation (non-blocking) ────────────────────────────────────
//
// Previous implementation did delay(80) + fade_out() with delay(8) per
// step = ~160ms of blocking time per beat with FastLED IRQ-disabled
// sections — starved ESP-NOW RX on single-core C3 causing periodic
// freezes. Refactored to a state machine driven by millis().

#define FADE_DURATION_MS  80      // fade-out wall clock time
#define LED_MIN_SHOW_MS   12      // throttle FastLED.show() during fade

static enum { FLASH_IDLE, FLASH_ON, FLASH_FADING } flash_state = FLASH_IDLE;
static uint32_t flash_start_ms = 0;
static uint32_t last_led_show_ms = 0;
static CRGB     flash_color = CRGB::Black;

static inline void flash_leds(CRGB color) {
    fill_solid(leds, NUM_LEDS, color);
    FastLED.show();
    last_led_show_ms = millis();
}

// Advance the flash state machine. Call every loop iteration.
static void flash_tick() {
    if (flash_state == FLASH_IDLE) return;

    uint32_t elapsed = millis() - flash_start_ms;

    if (flash_state == FLASH_ON) {
        if (elapsed >= FLASH_DURATION_MS) {
            flash_state = FLASH_FADING;
        }
        return;
    }

    // FLASH_FADING
    uint32_t fade_elapsed = elapsed - FLASH_DURATION_MS;
    if (fade_elapsed >= FADE_DURATION_MS) {
        fill_solid(leds, NUM_LEDS, COLOR_OFF);
        FastLED.show();
        last_led_show_ms = millis();
        flash_state = FLASH_IDLE;
        return;
    }

    // Throttle show() calls to avoid excessive bit-banging
    if (millis() - last_led_show_ms < LED_MIN_SHOW_MS) return;

    uint8_t scale = (uint8_t)(255 - (255 * fade_elapsed / FADE_DURATION_MS));
    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = flash_color;
        leds[i].nscale8(scale);
    }
    FastLED.show();
    last_led_show_ms = millis();
}

// ── Setup ───────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println("Firefly Wristband v0.2 (protocol v2)");

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

    // Init WiFi for ESP-NOW (no association)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);
    esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_err_t ch_err = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Init ESP-NOW
    esp_err_t now_err = esp_now_init();
    if (now_err != ESP_OK) {
        Serial.printf("ESP-NOW init FAILED: %d\n", now_err);
        flash_leds(CRGB::Red);
        while (1) { delay(1000); }
    }

    esp_now_register_recv_cb(on_receive);

    // Register broadcast peer (needed for RX on ESP32-C3 in some Arduino core versions)
    const uint8_t BROADCAST_ADDR[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_err_t peer_err = esp_now_add_peer(&peer);

    Serial.println("ESP-NOW ready — waiting for beats");
    Serial.printf("init codes: ps=%d ch=%d now=%d peer=%d\n",
        ps_err, ch_err, now_err, peer_err);

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
        firefly_oled_clear(&oled);
        firefly_oled_header(&oled, "Firefly WB");
        firefly_oled_kv(&oled, 0, "ch ", String(ESPNOW_CHANNEL).c_str());
        firefly_oled_kv(&oled, 1, "st ", "waiting");
        firefly_oled_flush(&oled);
    }
    Serial.printf("OLED: %s\n", oled_ok ? "ok" : "absent");
}

// ── Main loop ───────────────────────────────────────────────────────

// State tracking
static bool     waiting_for_beat = true;
static int64_t  scheduled_flash_local_us = 0;
static bool     scheduled_is_downbeat = false;
static uint32_t last_status_ms = 0;

void loop() {
    int64_t now_us = local_time_us();

    // ── Check for new beat data ─────────────────────────────────────
    if (offset_initialized) {
        bool is_playing = (latest_flags & FIREFLY_FLAG_PLAYING) != 0;

        if (is_playing) {
            // Prefer next_beat_us for per-beat flashing (v2 field)
            // Falls back to next_downbeat_us if next_beat_us is 0 (unknown)
            int64_t target_us = (latest_next_beat_us != 0)
                ? latest_next_beat_us
                : latest_next_downbeat_us;

            if (target_us != 0) {
                int64_t local_target = to_local_time(target_us);

                // Only schedule if this is a future event we haven't scheduled yet
                if (local_target > now_us && local_target != scheduled_flash_local_us) {
                    scheduled_flash_local_us = local_target;
                    // Determine if this is a downbeat
                    // beat_in_bar==0 and using downbeat time, OR beat_in_bar will wrap to 0
                    scheduled_is_downbeat = (latest_beat_in_bar == 0) ||
                        (target_us == latest_next_downbeat_us);
                    waiting_for_beat = true;
                }
            }
        }
    }

    // ── Fire scheduled flash ────────────────────────────────────────
    if (waiting_for_beat && scheduled_flash_local_us != 0) {
        int64_t time_until_flash = scheduled_flash_local_us - now_us;

        if (time_until_flash <= 0) {
            bool cdj_active = (latest_flags & FIREFLY_FLAG_CDJ_ACTIVE) != 0;

            // Color: downbeat accent, CDJ tint, or regular beat
            CRGB color;
            if (scheduled_is_downbeat) {
                color = COLOR_DOWNBEAT;
            } else if (cdj_active) {
                color = COLOR_CDJ_BEAT;
            } else {
                color = COLOR_BEAT;
            }

            flash_color = color;
            flash_start_ms = millis();
            flash_state = FLASH_ON;
            flash_leds(color);

            waiting_for_beat = false;
            scheduled_flash_local_us = 0;
        }
    }

    // Advance LED fade state machine (non-blocking)
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

    // ── OLED refresh ~2Hz ────────────────────────────────────────────
    // Kept at 2Hz (500ms) to minimize time spent blocking in I2C transfer.
    if (oled.present && millis() - last_oled_ms > 500) {
        last_oled_ms = millis();
        char line[24];
        firefly_oled_clear(&oled);
        firefly_oled_header(&oled, "Firefly WB");

        float tempo = latest_tempo_x100 / 100.0f;
        bool is_playing = (latest_flags & FIREFLY_FLAG_PLAYING) != 0;
        bool cdj_active = (latest_flags & FIREFLY_FLAG_CDJ_ACTIVE) != 0;
        const char *src = cdj_active ? "CDJ " : (is_playing ? "LINK" : "idle");
        snprintf(line, sizeof(line), "%5.1f %s b%u", tempo, src,
                 (unsigned)(latest_beat_in_bar + 1));
        firefly_oled_kv(&oled, 0, "bpm", line);

        snprintf(line, sizeof(line), "%lu", (unsigned long)packets_received);
        firefly_oled_kv(&oled, 1, "rx ", line);

        // Offset in ms for readability
        long off_ms = (long)(clock_offset_us / 1000);
        snprintf(line, sizeof(line), "%ld ms", off_ms);
        firefly_oled_kv(&oled, 2, "off", line);

        snprintf(line, sizeof(line), "m:%u air:%02X",
                 latest_master_device, latest_on_air_mask);
        firefly_oled_kv(&oled, 3, "mt ", line);

        firefly_oled_flush(&oled);
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

    // Small yield to avoid tight-looping
    delay(1);
}
