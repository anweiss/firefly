# Firefly — Copilot Instructions

## Project Overview

Beat-synced LED wristbands: Pioneer DJ Link → Rust coordinator → USB serial → ESP32 dongle (ESP-NOW) → ESP32 wristbands (LEDs).

## Repository Layout

| Path | Language | Build tool |
|---|---|---|
| `coordinator/` | Rust | `cargo` |
| `dongle-firmware/` | Arduino C++ | `arduino-cli` (board: `esp32:esp32:XIAO_ESP32C3`) |
| `wristband-firmware/` | Arduino C++ | `arduino-cli` (board: `esp32:esp32:XIAO_ESP32C3`) |
| `shared/protocol.h` | C header | Included by all three components |

## Validation Requirements

**After any change to coordinator, firmware, or shared/protocol.h, run ALL of the following:**

```bash
# 1. Coordinator tests (Rust)
cd coordinator && cargo test

# 2. Dongle firmware compilation
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 dongle-firmware/

# 3. Wristband firmware compilation
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 wristband-firmware/
```

All three must pass with zero errors before committing.

## Arduino Conventions

- `.ino` filename **must** match the directory name (e.g. `dongle-firmware/dongle-firmware.ino`)
- ESP32 Arduino core version: **3.x** — API signatures differ from 2.x (e.g. `esp_now_send_cb_t` uses `esp_now_send_info_t*`, not `uint8_t*`)
- FastLED library required for wristband firmware

## Protocol

- Wire protocol v2: 36-byte packets, little-endian
- CRC-8: **non-reflected** poly 0x31, init 0x00 (NOT CRC-8/MAXIM-DOW which is reflected)
- Check value for "123456789": `0xA2`
- CRC covers bytes [2..35)
- Both Rust (`crc` crate with custom `CRC8_FIREFLY` algorithm) and C (`firefly_crc8()` in protocol.h) must produce identical checksums

## Cross-Component Impact

Changes to `shared/protocol.h` affect all three components — always recompile and test everything.

Changes to packet field offsets, sizes, or CRC must be validated against the firmware simulators in `coordinator/src/firmware_sim.rs` (DongleSim + WristbandSim).
