# firefly-fw

All-in-one Firefly firmware for the **XIAO ESP32-C3**: replaces the
host-side `coordinator` binary plus the USB-serial `dongle-firmware`
bridge with a single Rust device on the Wi-Fi network.

## Pipeline

```
   Pioneer DJ Link (UDP 50001/50002)        ESP-NOW broadcast (channel 11)
              │                                     ▲
              ▼                                     │
   ┌─────────────────────────────────┐              │
   │     XIAO ESP32-C3               │              │
   │  ─────────────────────────────  │              │
   │  Wi-Fi STA  →  djlink::run()    │              │
   │                  │              │              │
   │                  ▼              │              │
   │     BeatSourceState (PLL)       │              │
   │                  │              │              │
   │                  ▼              │              │
   │     build_packet (v2, 36 B) ────┼──────────────┘
   │                                 │
   │  100 Hz periodic broadcast      │
   │  + SSD1306 OLED status thread   │
   └─────────────────────────────────┘
```

Module layout (`src/`):

| File | Role |
|---|---|
| `main.rs` | Bootstraps Wi-Fi + DJ Link + broadcaster + display threads, runs the 100 Hz broadcast loop with internal-clock fallback |
| `wifi.rs` | STA scan-and-connect; default modem-sleep PS leaves natural radio-pause windows for the ESP-NOW TX queue to drain |
| `djlink.rs` | UDP listeners on 50001 (Beat) + 50002 (Status); parses tempo and beat-in-bar from the magic `Qspt1WmJOL` envelope |
| `espnow.rs` | Broadcast peer + hello-frame ingest (auto-pairs wristbands by MAC), 11b @ 1 Mbps PHY rate, rate-limited NO_MEM warning |
| `beat_state.rs` | CDJ beat ingest + 2 s timeout state machine + PLL-anchored beat prediction |
| `protocol.rs` | Hand-port of the v2 wire packet from `coordinator/src/main.rs` (kept byte-for-byte identical) |
| `display.rs` | SSD1306 OLED on I²C0 (SDA=GPIO6, SCL=GPIO7), dedicated thread so I²C latency cannot gallop the broadcast loop |

## Toolchain setup

This project uses **Rust on ESP-IDF (std)**, not bare-metal. The Espressif
toolchain ships RISC-V LLVM and ldproxy.

```bash
# 1. Install espup, the Espressif toolchain manager
cargo install espup --locked
espup install                       # installs Rust nightly + LLVM toolchains

# 2. Source env vars on every shell (or add to your shell rc)
. $HOME/export-esp.sh

# 3. Install ldproxy + espflash
cargo install ldproxy espflash --locked
```

Reference: <https://docs.esp-rs.org/book/installation/index.html>

## Build & flash

```bash
# Wi-Fi credentials are baked in at compile time (env! in src/wifi.rs).
# The placeholders in .cargo/config.toml ARE NOT USED if these env vars
# are set. Override per-shell or per-command:
WIFI_SSID="your-ssid" WIFI_PASS="your-password" \
    cargo build --release

# Flash + open serial monitor (espflash discovers the C3 over USB-CDC)
WIFI_SSID="your-ssid" WIFI_PASS="your-password" \
    cargo run --release
```

Cold release builds take ~3–5 minutes the first time (esp-idf-sys
bootstraps ESP-IDF v5.2.2 via cmake + ninja into `.embuild/`). Warm
incremental rebuilds are ~5 s. Don't `cargo clean` casually — the
checkout is ~1 GB.

## Network requirements

- The XIAO and your CDJs **must be on the same Wi-Fi LAN** (or at least
  same broadcast domain). Pro DJ Link uses UDP broadcast on ports
  50001 / 50002.
- Most pro DJ setups use a wired Ethernet network. To bridge, attach a
  cheap travel router with DHCP and a Wi-Fi AP, plug the CDJs into its
  LAN ports, and join the XIAO to its SSID.
- ESP-NOW operates on the channel the Wi-Fi STA is currently on. **Pin
  your AP to channel 11** to match the wristband firmware's
  `ESPNOW_CHANNEL`.

## Operational notes

- **Broadcast rate is 100 Hz**, not 200 Hz. The single-radio C3 cannot
  sustain 200 Hz × ~3 retransmits without saturating the IDF ESP-NOW
  TX queue (`ESP_ERR_ESPNOW_NO_MEM`) when Wi-Fi STA contention spikes.
  10 ms phase granularity is well below human flicker-fusion for
  beat-synced LEDs; the wristband's PLL anchor smooths over the
  occasional ~500 ms dropout caused by Wi-Fi background scans / beacon
  recovery.
- **PHY rate is 11b @ 1 Mbps** (`WIFI_PHY_RATE_1M_L`). Long Range mode
  caused bursty arrival that broke beat-flash timing — do not enable it.
- **Solder the IPEX U.FL external antenna** on the XIAO. The PCB
  antenna detunes badly without a USB-cable counterpoise and is
  unreliable on battery / charger power.

## What's not yet ported from the host coordinator

- **Ableton Link peer participation** — `ableton-link-rs` wraps a C++
  library that does not cross-compile to RISC-V. Without Link, the
  firmware emits a not-playing stream when no CDJs are active (instead
  of bridging to a Link session). For most live setups with CDJs this
  is functionally identical. If you need Link, use the host `coordinator/`
  + `dongle-firmware/` path.
- **stdin tempo override** — irrelevant on a headless device.
- **Tempo-master election** — the host coordinator queries
  `pdl.virtual_cdj().tempo_master().master_device()`. firefly-fw uses
  simpler "first deck heard wins" master selection. Sufficient for
  single-deck tests; multi-deck may need a Pro DJ Link master-election
  rewrite.
- **`channels_on_air`** — populated by Pro DJ Link MixerStatus packets
  on the host; not yet parsed here. `on_air_mask` will always be 0.

## CI

GitHub Actions runs `cargo fmt -- --check` and `cargo clippy --release
--all-targets -- -D warnings` (cross-compiled to `riscv32imc-esp-espidf`)
on every push and PR via `esp-rs/xtensa-toolchain` — see the root
[`.github/workflows/ci.yml`](../.github/workflows/ci.yml) for the
firefly-fw jobs.
