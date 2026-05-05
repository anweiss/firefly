/**
 * Firefly battery-sense helpers — shared between dongle, wristband, and
 * firefly-fw (Rust).
 *
 * Hardware
 * --------
 * The Seeed XIAO Expansion Board v1.0 wires the JST-PH 2.0 battery
 * connector to BAT+ on the XIAO module (through the on-board charge
 * controller). The XIAO ESP32-C3 has *no* factory voltage divider from
 * BAT to an ADC pin, so a single-cell LiPo (3.0–4.2 V) cannot be read
 * directly — its voltage exceeds the 3.3 V ADC reference at full
 * charge.
 *
 * To enable battery monitoring, solder a 100 kΩ : 100 kΩ resistor
 * divider from BAT (top of the JST-PH connector) to A2 / D2 / GPIO4
 * (any free ADC1 channel works — A2 is unused on both the dongle and
 * wristband expansion-board layouts):
 *
 *      BAT+ ──[100 kΩ]──┬──[100 kΩ]── GND
 *                       │
 *                       └──── A2 / D2 / GPIO4 (ADC1_CH4)
 *
 * The divider halves the battery voltage so 4.2 V → 2.1 V at the ADC,
 * within DB_11 attenuation range (~0–3.3 V). 100 kΩ legs draw 21 µA at
 * full charge → ~21 µA × 24 h × 30 d ≈ 15 mAh/month standby drain on
 * the 500 mAh LiPo, negligible vs. radio current.
 *
 * If the divider is not wired (pin floating), `firefly_battery_percent`
 * returns -1 and consumers should display "—" instead of 0%.
 *
 * Voltage → percent
 * -----------------
 * LiPo discharge is non-linear; below 3.7 V the cell empties quickly.
 * We use a piecewise-linear approximation tuned for typical 1S 4.2 V
 * cells under light load (matches a stock iPhone-style curve closely
 * enough for a status row):
 *
 *     >= 4.20 V  → 100 %
 *        4.10 V  →  90 %
 *        4.00 V  →  80 %
 *        3.90 V  →  65 %
 *        3.80 V  →  45 %
 *        3.70 V  →  25 %
 *        3.60 V  →  10 %
 *        3.50 V  →   5 %
 *     <= 3.30 V  →   0 %
 */

#ifndef FIREFLY_BATTERY_SENSE_H
#define FIREFLY_BATTERY_SENSE_H

#include <stdint.h>

#define FIREFLY_VBAT_DIVIDER_NUM   2   /* divider ratio numerator   */
#define FIREFLY_VBAT_DIVIDER_DEN   1   /* divider ratio denominator */

/* Plausibility window (battery side, after multiplying by divider). */
#define FIREFLY_VBAT_MIN_MV        2500
#define FIREFLY_VBAT_MAX_MV        4400

/**
 * Map a battery voltage (in mV) to a 0–100 percent estimate using the
 * piecewise curve documented above. Returns -1 if the value is outside
 * the plausible LiPo window (likely the divider isn't wired and the
 * ADC is floating).
 */
static inline int8_t firefly_battery_percent(int32_t vbat_mv) {
    if (vbat_mv < FIREFLY_VBAT_MIN_MV || vbat_mv > FIREFLY_VBAT_MAX_MV) {
        return -1;
    }
    /* Piecewise-linear LiPo curve (mV, %). */
    static const int16_t curve_mv[] = {
        3300, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200
    };
    static const int8_t  curve_pc[] = {
           0,    5,   10,   25,   45,   65,   80,   90,  100
    };
    const int n = (int)(sizeof(curve_mv) / sizeof(curve_mv[0]));
    if (vbat_mv >= curve_mv[n - 1]) return 100;
    if (vbat_mv <= curve_mv[0])     return 0;
    for (int i = 1; i < n; i++) {
        if (vbat_mv <= curve_mv[i]) {
            int32_t span_mv = curve_mv[i] - curve_mv[i - 1];
            int32_t span_pc = curve_pc[i] - curve_pc[i - 1];
            int32_t off_mv  = vbat_mv      - curve_mv[i - 1];
            return (int8_t)(curve_pc[i - 1] + (off_mv * span_pc) / span_mv);
        }
    }
    return -1;
}

#endif /* FIREFLY_BATTERY_SENSE_H */
