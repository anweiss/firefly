# Firefly 🔥

Beat-synced LED wristbands powered by [Pioneer Pro DJ Link](https://github.com/anweiss/prodjlink-rs), [Ableton Link](https://www.ableton.com/en/link/), and ESP-NOW.

## Architecture

```
┌──────────┐  DJ Link  ┌─────────────────┐  USB Serial  ┌────────────┐  ESP-NOW  ┌────────────┐
│ CDJ-3000 │──────────►│  Coordinator     │─────────────►│   Dongle   │─ ─ ─ ─ ─►│ Wristband  │
│ DJM-A9   │           │  (Mac, Rust)     │              │  (ESP32-C3)│          │ (ESP32-C3) │ ×N
└──────────┘     ┌────►│  prodjlink-rs +  │              └────────────┘          └────────────┘
                 │     │  ableton-link-rs │
┌──────────┐  Link     └─────────────────┘
│ Ableton  │─────┘
│ / other  │
└──────────┘
```

**Signal flow**: CDJ beats (or Ableton Link) → Rust coordinator on Mac → USB serial → ESP32 dongle → ESP-NOW broadcast → wristband LEDs flash on beat.

When CDJs are playing, the coordinator uses **CDJ beats as the authoritative timing source** and bridges tempo to Link. If no CDJ master is detected (2s timeout), it falls back to **Link-only mode**.

## Components

| Directory | Language | Description |
|---|---|---|
| `coordinator/` | Rust | Joins DJ Link + Link sessions, computes beat timing, writes serial packets |
| `dongle-firmware/` | Arduino C++ | USB-Serial → ESP-NOW bridge (XIAO ESP32-C3) |
| `wristband-firmware/` | Arduino C++ | Receives ESP-NOW, flashes LEDs on beat (XIAO ESP32-C3) |
| `shared/` | C header | Wire protocol v2 definition (packet format, CRC-8) |

## DJ Link Features

The coordinator uses [prodjlink-rs](https://github.com/anweiss/prodjlink-rs) to:
- **Discover CDJs/mixers** on the network and join as a virtual player
- **Track the tempo master** and use its beat timing directly
- **Receive on-air status** from the mixer (carried in packets for future effects)
- **Participate in master handoff** (Baroque dance protocol with auto-negotiate)
- **Detect play/stop** via beat timeout (2s window)

## Wire Protocol v2

36-byte packet, little-endian:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0–1 | 2 | `sync` | `0xBE 0xA7` |
| 2 | 1 | `version` | `0x02` |
| 3 | 1 | `total_len` | `36` |
| 4–11 | 8 | `send_time_us` | Coordinator clock at send (i64) |
| 12–19 | 8 | `next_downbeat_us` | Coordinator time of next downbeat (i64) |
| 20–21 | 2 | `tempo_bpm_x100` | BPM × 100 (u16) |
| 22 | 1 | `beat_in_bar` | Current phase beat (0–3 for 4/4) |
| 23 | 1 | `flags` | bit 0 = is_playing, bit 1 = cdj_active |
| 24–31 | 8 | `next_beat_us` | Coordinator time of next beat (i64, 0 = unknown) |
| 32 | 1 | `on_air_mask` | Bitmask: bit N = channel (N+1) on-air |
| 33 | 1 | `master_device` | DJ Link device number of master (0 = none) |
| 34 | 1 | `reserved` | Reserved for future use |
| 35 | 1 | `crc8` | CRC-8/MAXIM over bytes [2..35) |

## Quick Start

### Coordinator (Mac)

```bash
cd coordinator
cargo build --release

# With CDJ/mixer on the network:
cargo run --release -- --port /dev/cu.usbmodem1101 --interface 192.168.1.145

# Link-only mode (no DJ Link):
cargo run --release -- --port /dev/cu.usbmodem1101 --no-djlink --bpm 120
```

### Dongle & Wristband

Flash via Arduino IDE:
1. Install ESP32 board support (`https://espressif.github.io/arduino-esp32/package_esp32_index.json`)
2. Install FastLED library (wristband only)
3. Select board: **XIAO_ESP32C3**
4. Flash `dongle-firmware/dongle.ino` to the USB-connected XIAO
5. Flash `wristband-firmware/wristband.ino` to each wristband XIAO

## Hardware (Stage 1 Prototype)

- 2× Seeed XIAO ESP32-C3 (pre-soldered headers)
- 1× Grove RGB LED Stick (WS2813)
- 1× USB-C data cable
- 1× 3.7V LiPo with JST-PH 2.0 connector (wristband only)

## Clock Synchronization

Wristbands track the offset between the coordinator's clock and their local clock using an exponential moving average (EMA, α=0.1). The coordinator sends "beat will happen at time T" rather than "beat now" — this absorbs ESP-NOW jitter into scheduling lead time, so all wristbands flash within microseconds of each other.
