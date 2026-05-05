# Firefly 🔥

Beat-synced LED wristbands powered by [Pioneer Pro DJ Link](https://github.com/anweiss/prodjlink-rs), [Ableton Link](https://www.ableton.com/en/link/), and ESP-NOW.

Two deployment modes:

* **All-in-one (recommended)** — `firefly-fw/` runs on a single XIAO ESP32-C3 that joins your Wi-Fi, listens for DJ Link directly, and broadcasts ESP-NOW to the wristbands. No host laptop needed at the gig.
* **Host coordinator (legacy / Link-bridge)** — `coordinator/` (Rust on Mac) + `dongle-firmware/` (USB-serial bridge XIAO). Required if you want Ableton Link peer participation, since `ableton-link-rs` doesn't cross-compile to RISC-V yet.

## Architecture

```
                              ALL-IN-ONE PATH (firefly-fw)
┌──────────┐  DJ Link UDP   ┌──────────────────────────┐  ESP-NOW   ┌────────────┐
│ CDJ-3000 │───────────────►│  XIAO ESP32-C3           │─ ─ ─ ─ ─ ─►│ Wristband  │
│ DJM-A9   │   (Wi-Fi LAN)  │  Rust + esp-idf-svc      │            │ (ESP32-C3) │ ×N
└──────────┘                │  100 Hz broadcast + OLED │            └────────────┘
                            └──────────────────────────┘

                              HOST COORDINATOR PATH (legacy)
┌──────────┐  DJ Link  ┌──────────────────┐  USB Serial  ┌────────────┐  ESP-NOW ┌────────────┐
│ CDJ-3000 │──────────►│  Coordinator     │─────────────►│   Dongle   │─ ─ ─ ─ ─►│ Wristband  │
│ DJM-A9   │           │  (Mac, Rust)     │              │ (ESP32-C3) │          │ (ESP32-C3) │ ×N
└──────────┘     ┌────►│  prodjlink-rs +  │              └────────────┘          └────────────┘
                 │     │  ableton-link-rs │
┌──────────┐  Link     └──────────────────┘
│ Ableton  │─────┘
└──────────┘
```

When CDJs are playing, the coordinator/firmware uses **CDJ beats as the authoritative timing source**. If no CDJ master is detected (2 s timeout), it falls back to an internal clock (host coordinator additionally bridges to a Link session in this state).

## Components

| Directory | Language | Description |
|---|---|---|
| `firefly-fw/` | Rust (esp-idf-svc) | **All-in-one firmware** for XIAO ESP32-C3 — Wi-Fi STA + DJ Link UDP + ESP-NOW broadcaster + SSD1306 OLED status |
| `coordinator/` | Rust | Legacy host-side: joins DJ Link + Link sessions, computes beat timing, writes serial packets |
| `dongle-firmware/` | Arduino C++ | Legacy USB-Serial → ESP-NOW bridge (XIAO ESP32-C3) with optional OLED |
| `wristband-firmware/` | Arduino C++ | Receives ESP-NOW, flashes LEDs on beat (XIAO ESP32-C3) |
| `shared/` | C headers | Wire protocol v2 + extracted state machines (testable without hardware) |
| `tests/` | C++ | Native host tests (`g++`) |

`shared/` headers:

| File | Purpose |
|---|---|
| `protocol.h` | 36-byte wire packet definition + `firefly_crc8()` |
| `beat_clock.h` | Pre-flash scheduling + EMA clock offset helpers (shared between firmware paths) |
| `dongle_logic.h` | Dongle serial-framing state machine |
| `wristband_logic.h` | Wristband packet parsing + clock offset EMA |
| `oled_display.h` | SSD1306 helper layout shared between dongle + firefly-fw |

## Wire Protocol v2

36-byte packet, little-endian:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0–1   | 2 | `sync` | `0xBE 0xA7` |
| 2     | 1 | `version` | `0x02` |
| 3     | 1 | `total_len` | `36` |
| 4–11  | 8 | `send_time_us` | Coordinator clock at send (i64) |
| 12–19 | 8 | `next_downbeat_us` | Coordinator time of next downbeat (i64) |
| 20–21 | 2 | `tempo_bpm_x100` | BPM × 100 (u16) |
| 22    | 1 | `beat_in_bar` | Current phase beat (0–3 for 4/4) |
| 23    | 1 | `flags` | bit 0 = `is_playing`, bit 1 = `cdj_active` |
| 24–31 | 8 | `next_beat_us` | Coordinator time of next beat (i64, 0 = unknown) |
| 32    | 1 | `on_air_mask` | Bitmask: bit N = channel (N+1) on-air |
| 33    | 1 | `master_device` | DJ Link device number of master (0 = none) |
| 34    | 1 | `reserved` | Reserved for future use |
| 35    | 1 | `crc8` | CRC-8 (poly 0x31, init 0x00, **non-reflected**) over bytes [2..35) |

⚠️ The CRC is **not** CRC-8/MAXIM-DOW (which is reflected). The check value over `"123456789"` is `0xA2`.

## ESP-NOW Topology

* **Wi-Fi channel: 11** (must match across firefly-fw / dongle / wristbands)
* **PHY rate: 11b @ 1 Mbps** (`WIFI_PHY_RATE_1M_L`) — Long Range mode caused bursty arrival that broke beat-flash timing
* **Broadcast rate: 100 Hz** (firefly-fw) — 200 Hz saturated the C3's small ESP-NOW TX queue under Wi-Fi STA contention
* **Hardware**: solder the XIAO ESP32-C3's IPEX U.FL external antenna — the PCB antenna detunes badly without a USB-cable counterpoise

## Quick Start

### All-in-one (firefly-fw)

See [`firefly-fw/README.md`](firefly-fw/README.md) for full setup. TL;DR:

```bash
cd firefly-fw
# One-time toolchain
cargo install espup ldproxy espflash --locked
espup install
. $HOME/export-esp.sh

# Build + flash (Wi-Fi creds baked in at compile time)
WIFI_SSID="your-ssid" WIFI_PASS="your-password" cargo run --release
```

### Host coordinator (legacy)

```bash
cd coordinator
cargo build --release

# With CDJ/mixer on the network:
cargo run --release -- --port /dev/cu.usbmodem1101 --interface 192.168.1.145

# Auto-detect serial port (tries /dev/cu.usbmodem* first):
cargo run --release -- --interface 192.168.1.145

# Link-only mode (no DJ Link):
cargo run --release -- --no-djlink --bpm 120
```

### Dongle & Wristband (Arduino)

```bash
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 dongle-firmware/
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 wristband-firmware/
arduino-cli upload --fqbn esp32:esp32:XIAO_ESP32C3 -p /dev/cu.usbmodem1101 wristband-firmware/
```

Requirements:
* ESP32 Arduino core **3.x** (API signatures differ from 2.x — e.g. `esp_now_send_cb_t` takes `esp_now_send_info_t*` not `uint8_t*`)
* FastLED library (wristband)

## Hardware

* 1× Seeed XIAO ESP32-C3 per role (firefly-fw OR dongle, plus N wristbands) — **with IPEX U.FL antenna soldered**
* 1× Grove RGB LED Stick (WS2813) per wristband
* Optional: 0.96″ SSD1306 OLED (I²C @ 0x3C, SDA=GPIO6, SCL=GPIO7) on firefly-fw / dongle / wristband
* 1× 3.7 V LiPo with JST-PH 2.0 connector per wristband
* USB-C data cables

## Clock Synchronization

Wristbands track the offset between the broadcaster's clock and their local clock using an exponential moving average (EMA, α=0.1). Packets carry "the next beat will happen at time T" rather than "beat now" — ESP-NOW jitter is absorbed into scheduling lead time so all wristbands flash within microseconds of each other. PLL anchor in `tick_predicted_beats` lets the wristband interpolate smoothly across short ESP-NOW outages (e.g. the occasional ~500 ms burst when Wi-Fi STA preempts the radio for beacon recovery).

## CI

GitHub Actions runs on every push / PR (see `.github/workflows/`):

* `coordinator` — rustfmt, clippy (`-D warnings`), `cargo test`
* `firefly-fw` — rustfmt, clippy (`-D warnings`) cross-compiled to `riscv32imc-esp-espidf` via [`esp-rs/xtensa-toolchain`](https://github.com/esp-rs/xtensa-toolchain) (also serves as the build check)
* Arduino firmware — `arduino-cli compile` for both `dongle-firmware/` and `wristband-firmware/`
* `tests/test_firmware.cpp` — host-native C++ tests for `shared/` logic
* `cargo audit` — both Rust crates, weekly schedule + on PR
