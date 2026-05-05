# firefly-fw

All-in-one Firefly coordinator + ESP-NOW broadcaster firmware for the
**XIAO ESP32-C3**. Replaces the host-side `coordinator` binary plus the
USB-serial `dongle-firmware` bridge with a single device on the Wi-Fi
network.

## Pipeline

```
   Pioneer DJ Link (UDP 50001/50002)        ESP-NOW (broadcast)
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
   │  200 Hz periodic broadcast      │
   └─────────────────────────────────┘
```

## Toolchain setup

This project uses **Rust on ESP-IDF (std)**, not bare-metal. You need
the Espressif Rust toolchain.

```bash
# 1. Install espup, the Espressif toolchain manager
cargo install espup --locked
espup install                       # installs nightly + Xtensa/RV32 LLVM

# 2. Source the env vars (or use $HOME/export-esp.sh on every shell)
. $HOME/export-esp.sh

# 3. Install ldproxy + espflash
cargo install ldproxy espflash --locked
```

Reference: <https://docs.esp-rs.org/book/installation/index.html>

## Build & flash

```bash
# Provide Wi-Fi credentials at compile time. The defaults in
# .cargo/config.toml are placeholders.
WIFI_SSID="your-ssid" WIFI_PASS="your-password" \
    cargo build --release

# Flash + open serial monitor (espflash discovers the C3 over USB-CDC)
WIFI_SSID="your-ssid" WIFI_PASS="your-password" \
    cargo run --release
```

## Network requirements

- The XIAO and your CDJs **must be on the same Wi-Fi LAN** (or at least
  same broadcast domain). Pro DJ Link uses UDP broadcast on ports
  50001/50002.
- Most pro DJ setups use a wired Ethernet network. To bridge, attach a
  cheap travel router with DHCP and a Wi-Fi AP, plug the CDJs into its
  LAN ports, and join the XIAO to its SSID.
- ESP-NOW operates on the channel the Wi-Fi STA is currently on. If the
  AP roams the XIAO between channels, ESP-NOW connectivity to the
  wristbands will break. Pin your AP to **channel 11** to match the
  wristband firmware (`ESPNOW_CHANNEL`).

## What's not yet ported from the host coordinator

- **Ableton Link peer participation** — `ableton-link-rs` wraps a C++
  library that does not cross-compile to RISC-V. Without Link, the
  firmware emits a not-playing stream when no CDJs are active (instead
  of bridging to a Link session). For most live setups with CDJs this
  is functionally identical.
- **stdin tempo override** — irrelevant on a headless device; the host
  coordinator's stdin path is gone.
- **Tempo-master election** — host coordinator queries
  `pdl.virtual_cdj().tempo_master().master_device()`. This firmware
  uses simpler "first deck heard wins" master selection. Sufficient for
  single-deck tests; multi-deck may need a Pro DJ Link master-election
  rewrite (the protocol is well documented).
- **`channels_on_air`** — populated by Pro DJ Link MixerStatus packets.
  Not yet parsed here; `on_air_mask` will always be 0.

## Verifying the protocol against the host coordinator

The wire-protocol module (`src/protocol.rs`) is a hand-port of
`coordinator/src/main.rs`. To make sure they stay in sync:

```bash
# Run host coordinator's protocol tests
cd ../coordinator && cargo test

# Run the firmware's protocol tests (host build, ignores ESP-IDF)
cd ../firefly-fw && cargo test --target $(rustc -vV | sed -n 's/host: //p') protocol
```

(The second command works only if you remove ESP-IDF from the default
`build.target`. Easiest is to just run `cargo test --lib` after
factoring `protocol.rs` into a no_std-able crate — TODO.)
