# Firefly — Copilot Instructions

## Project Overview

Beat-synced LED wristbands. Two deployment paths share the same wire
protocol and wristband firmware:

1. **All-in-one (firefly-fw)**: XIAO ESP32-C3 → Wi-Fi STA → DJ Link UDP →
   ESP-NOW broadcast → wristbands. Single device, no host laptop.
2. **Host coordinator (legacy)**: Mac coordinator → USB serial → ESP32
   dongle → ESP-NOW → wristbands. Required for Ableton Link peer
   participation.

## Repository Layout

| Path | Language | Build tool |
|---|---|---|
| `firefly-fw/` | Rust (esp-idf-svc, std-on-IDF) | `cargo` (target `riscv32imc-esp-espidf` via espup + ldproxy) |
| `coordinator/` | Rust | `cargo` |
| `dongle-firmware/` | Arduino C++ | `arduino-cli` (board: `esp32:esp32:XIAO_ESP32C3`) |
| `wristband-firmware/` | Arduino C++ | `arduino-cli` (board: `esp32:esp32:XIAO_ESP32C3`) |
| `shared/protocol.h` | C header | Wire packet definition + `firefly_crc8()` — included by all C/C++ components |
| `shared/beat_clock.h` | C header | Pre-flash scheduling + EMA clock-offset helpers shared between firmware paths |
| `shared/dongle_logic.h` | C header | Dongle serial framing state machine (testable without ESP32) |
| `shared/wristband_logic.h` | C header | Wristband packet parsing + clock offset EMA (testable without ESP32) |
| `shared/oled_display.h` | C header | SSD1306 layout shared between dongle + firefly-fw |
| `tests/` | C++ | Native host tests (`g++`, no hardware needed) |

## Validation Requirements

**After any change to firefly-fw, coordinator, firmware, or
shared/*.h, run the relevant subset of:**

```bash
# 1. Coordinator (Rust)
cd coordinator && cargo fmt -- --check && cargo clippy --all-targets -- -D warnings && cargo test

# 2. firefly-fw (Rust, cross-compile to esp32c3)
cd firefly-fw && cargo fmt -- --check
WIFI_SSID="ci" WIFI_PASS="ci" cargo clippy --release --all-targets -- -D warnings

# 3. Dongle firmware compilation
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 dongle-firmware/

# 4. Wristband firmware compilation
arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 wristband-firmware/

# 5. Firmware native tests (C++)
cd tests && g++ -std=c++17 -Wall -Wextra -Werror -I ../shared -o test_firmware test_firmware.cpp && ./test_firmware
```

CI runs all of these on every push and PR — see `.github/workflows/ci.yml`.

## Arduino Conventions

- `.ino` filename **must** match the directory name (e.g. `dongle-firmware/dongle-firmware.ino`)
- ESP32 Arduino core version: **3.x** — API signatures differ from 2.x
  (e.g. `esp_now_send_cb_t` uses `esp_now_send_info_t*`, not `uint8_t*`)
- FastLED library required for wristband firmware
- `Serial` on the XIAO C3 is `HWCDC` (USB CDC on boot), not the legacy
  `HardwareSerial`. HWCDC-only methods like `setTxTimeoutMs()` should
  be guarded with `#if ARDUINO_USB_CDC_ON_BOOT` so VS Code IntelliSense
  (which reads core 2.x headers) doesn't squiggle them.

## firefly-fw Conventions

- Toolchain: nightly + rust-src, target `riscv32imc-esp-espidf`. Pinned
  in `firefly-fw/rust-toolchain.toml`.
- Wi-Fi credentials are baked in via `env!()` at build time. Always
  pass `WIFI_SSID` + `WIFI_PASS` env vars to `cargo build` (CI uses
  placeholders).
- The build is a full ESP-IDF C SDK build (esp-idf-sys / `.embuild/`).
  Cold builds are ~3–5 min; **never `cargo clean`** unless you have
  to — the checkout is ~1 GB.
- Don't mark intentionally-unused public API as warnings: use
  `#[allow(dead_code)]` rather than deleting it. CI clippy is
  `-D warnings`.

## ESP-NOW Topology

- **Channel: 11** across all components (`ESPNOW_CHANNEL` in dongle +
  wristband firmware; firefly-fw inherits from its Wi-Fi STA channel,
  so the AP must be pinned to ch 11).
- **PHY rate: 11b @ 1 Mbps** (`WIFI_PHY_RATE_1M_L`). LR mode caused
  bursty arrival that broke beat-flash timing — do not enable it.
- **Broadcast rate**: 200 Hz (host coordinator) / 100 Hz (firefly-fw).
  100 Hz is the highest the C3 can sustain alongside Wi-Fi STA
  without ESP-NOW TX queue saturation (`ESP_ERR_ESPNOW_NO_MEM`).

## Wire Protocol

- Wire protocol v2: **36-byte packets, little-endian**
- CRC-8: **non-reflected** poly 0x31, init 0x00 (NOT CRC-8/MAXIM-DOW
  which is reflected)
- Check value for "123456789": `0xA2`
- CRC covers bytes [2..35)
- All three implementations (Rust `crc` crate with custom
  `CRC8_FIREFLY` algorithm, C `firefly_crc8()` in protocol.h, Rust
  hand-port in `firefly-fw/src/protocol.rs`) must produce identical
  checksums

## Cross-Component Impact

- Changes to `shared/protocol.h` affect coordinator, dongle, wristband,
  AND firefly-fw (`src/protocol.rs` is a hand-port). Recompile + test
  all four.
- Changes to packet field offsets, sizes, or CRC must be validated
  against the firmware simulators in `coordinator/src/firmware_sim.rs`
  (DongleSim + WristbandSim) AND mirrored in `firefly-fw/src/protocol.rs`.

## Documentation

Three doc surfaces must stay in sync with the code. **Update the
relevant ones whenever you change behaviour, architecture, build
process, or deployment story** — not just when adding features:

| File | Scope | Update when… |
|---|---|---|
| `README.md` (root) | High-level overview, both deployment paths, Quick Start, ESP-NOW topology | Components added/removed, build commands change, ESP-NOW parameters change, CI surface changes |
| `firefly-fw/README.md` | firefly-fw module map, toolchain, build/flash, operational notes (100 Hz, antenna, PHY rate), gaps vs host coordinator | Any firefly-fw module added/removed, env-var contract changes, broadcast rate / PHY / channel changes, new "what's not yet ported" item resolved |
| `.github/copilot-instructions.md` | This file — conventions, validation matrix, gotchas | New convention discovered, new gotcha encountered, validation steps change |

There is also a separate **`firefly-docs` repo** at
`~/Development/anweiss/firefly-docs` (https://github.com/anweiss/firefly-docs)
covering architecture, protocol spec, project history, and planning.
**When you make a non-trivial change here, also update firefly-docs
in the same logical change** — typically `architecture.md` (system
design), `protocol-v2.md` (wire format / rates / framing), and
`project-history.md` (append a milestone for major changes). It runs
markdownlint-cli2 in CI; verify locally with
`cd ~/Development/anweiss/firefly-docs && npx -y markdownlint-cli2 '**/*.md'`
before pushing.

## Git Conventions

- Conventional commits — title format validated by
  `.github/workflows/conventional-commits.yml`.
- Allowed types: `feat`, `fix`, `docs`, `style`, `refactor`, `perf`,
  `test`, `build`, `chore`, `ci`.
- Always include the `Co-authored-by: Copilot
  <223556219+Copilot@users.noreply.github.com>` trailer.
