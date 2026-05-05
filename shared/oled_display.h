/**
 * Firefly OLED display helper — Seeed XIAO expansion board SSD1306
 *
 * 128×64 OLED @ I²C 0x3C on Wire (SDA=D4/GPIO6, SCL=D5/GPIO7 on C3).
 *
 * Usage:
 *   oled_display_t oled;
 *   firefly_oled_begin(&oled);   // returns false if no display found
 *   firefly_oled_header(&oled, "Firefly DNG");
 *   firefly_oled_kv(&oled, 1, "fwd", buf);
 *   firefly_oled_flush(&oled);
 */

#ifndef FIREFLY_OLED_DISPLAY_H
#define FIREFLY_OLED_DISPLAY_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define FIREFLY_OLED_W      128
#define FIREFLY_OLED_H      64
#define FIREFLY_OLED_ADDR   0x3C

typedef struct {
    Adafruit_SSD1306 *dev;
    bool present;
} oled_display_t;

static inline bool firefly_oled_begin(oled_display_t *o) {
    Wire.begin();
    o->dev = new Adafruit_SSD1306(FIREFLY_OLED_W, FIREFLY_OLED_H, &Wire, -1);
    o->present = o->dev->begin(SSD1306_SWITCHCAPVCC, FIREFLY_OLED_ADDR);
    if (!o->present) return false;
    // Bump to 400kHz AFTER begin() so initialization runs at the safe
    // 100kHz default (some SSD1306 modules ACK at 0x3C but corrupt their
    // command sequence at 400kHz during init). 400kHz is fine for
    // subsequent frame data: drops 128x64 transfer from ~105ms to ~26ms,
    // freeing the C3 to service ESP-NOW RX between refreshes.
    Wire.setClock(400000);
    o->dev->clearDisplay();
    o->dev->setTextColor(SSD1306_WHITE);
    o->dev->setTextSize(1);
    o->dev->setTextWrap(false);
    o->dev->setCursor(0, 0);
    o->dev->display();
    return true;
}

static inline void firefly_oled_clear(oled_display_t *o) {
    if (!o->present) return;
    o->dev->clearDisplay();
}

// Inverted header row with title
static inline void firefly_oled_header(oled_display_t *o, const char *title) {
    if (!o->present) return;
    o->dev->fillRect(0, 0, FIREFLY_OLED_W, 10, SSD1306_WHITE);
    o->dev->setTextColor(SSD1306_BLACK);
    o->dev->setCursor(2, 1);
    o->dev->print(title);
    o->dev->setTextColor(SSD1306_WHITE);
}

// Row: 0 = just below header, increments by 10px
static inline void firefly_oled_kv(oled_display_t *o, uint8_t row,
                                   const char *key, const char *val) {
    if (!o->present) return;
    int y = 12 + row * 10;
    o->dev->setCursor(0, y);
    o->dev->print(key);
    o->dev->print(": ");
    o->dev->print(val);
}

static inline void firefly_oled_flush(oled_display_t *o) {
    if (!o->present) return;
    o->dev->display();
}

#endif // FIREFLY_OLED_DISPLAY_H
