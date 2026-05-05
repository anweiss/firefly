//! Firefly all-in-one firmware.
//!
//! Replaces the host-side coordinator + USB-serial dongle bridge with
//! a single XIAO ESP32-C3 device that:
//!   1. Joins Wi-Fi STA so it can hear Pro DJ Link UDP traffic.
//!   2. Parses DJ Link Beat (port 50001) and CDJ Status (port 50002)
//!      packets, runs the same beat-clock fusion logic the host
//!      coordinator uses.
//!   3. Builds the v2 wire packet (`shared/protocol.h`) and broadcasts
//!      it via ESP-NOW at 200 Hz to the wristbands.
//!
//! Out of scope (vs. host coordinator):
//!   - Ableton Link peer participation (C++ Link library does not
//!     cross-compile to RISC-V). The Link-only fallback path (used
//!     when no CDJs are visible) is replaced here by an internal
//!     fixed-tempo clock derived from the last known CDJ tempo.

mod beat_state;
mod display;
mod djlink;
mod espnow;
mod protocol;
mod wifi;

use anyhow::Result;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::{debug, info, warn};
use std::sync::atomic::Ordering;
use std::sync::mpsc::{channel, TryRecvError};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use beat_state::{BeatSourceState, CdjPlayTransition};
use display::LiveState;
use djlink::DjLinkEvent;
use espnow::Broadcaster;
use protocol::{build_packet, FLAG_CDJ_ACTIVE, FLAG_PLAYING};

/// Broadcast rate. The host coordinator defaults to 200 Hz, but on the
/// ESP32-C3 with a single radio shared between Wi-Fi STA + ESP-NOW the
/// IDF TX queue saturates at 200 Hz × ~3 retransmits → ~600 pkts/s,
/// emitting `ESP_ERR_ESPNOW_NO_MEM` once the queue backs up after a
/// minute of runtime. 100 Hz still gives 10 ms phase granularity (well
/// below human flicker-fusion for beat-synced LEDs) while leaving the
/// queue plenty of headroom to drain between beacons.
const BROADCAST_HZ: u64 = 100;
const BROADCAST_PERIOD: Duration = Duration::from_micros(1_000_000 / BROADCAST_HZ);

/// Beats per bar. Mirrors `--quantum 4.0`.
const QUANTUM: u8 = 4;

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches
    // to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("firefly-fw v{} booting", env!("CARGO_PKG_VERSION"));

    let peripherals = Peripherals::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    // Take I²C0 + the OLED pins now, before `peripherals` is partially
    // moved into Wi-Fi. SDA = D4 (GPIO6), SCL = D5 (GPIO7) on the
    // XIAO C3 expansion board (see shared/oled_display.h).
    let i2c0 = peripherals.i2c0;
    let sda: esp_idf_svc::hal::gpio::AnyIOPin = peripherals.pins.gpio6.into();
    let scl: esp_idf_svc::hal::gpio::AnyIOPin = peripherals.pins.gpio7.into();

    // Wi-Fi must be up before EspNow::take() (ESP-NOW rides the same
    // radio interface). On failure we still proceed — without Wi-Fi
    // there's no DJ Link, but ESP-NOW alone can broadcast a free-running
    // clock for testing.
    let _wifi = match wifi::connect(peripherals.modem, sysloop, nvs) {
        Ok(w) => w,
        Err(e) => {
            warn!(
                "Wi-Fi connect failed: {}. Halting — ESP-NOW requires Wi-Fi init.",
                e
            );
            // Sleep forever rather than panic; lets the user see the log.
            loop {
                thread::sleep(Duration::from_secs(60));
            }
        }
    };

    let broadcaster = Broadcaster::new()?;
    let stats = broadcaster.stats();

    // OLED display task. Mirrors the dongle firmware's dedicated
    // FreeRTOS task — runs on its own thread so I²C latency cannot
    // gallop the 200 Hz broadcast loop. Silently no-ops if no display.
    let live_state = Arc::new(LiveState::default());
    if let Err(e) = display::spawn(i2c0, sda, scl, live_state.clone(), stats.clone()) {
        warn!("OLED: failed to spawn display thread: {:?}", e);
    }

    // DJ Link receive thread → main loop via mpsc.
    let (dj_tx, dj_rx) = channel::<DjLinkEvent>();
    thread::Builder::new()
        .name("djlink".into())
        .stack_size(8192)
        .spawn(move || {
            if let Err(e) = djlink::run(dj_tx) {
                warn!("DJ Link receiver exited: {}", e);
            }
        })?;

    // The "now" timestamp the host coordinator gets from
    // `link.clock().micros()` is replaced here by `boot_micros()` —
    // monotonic since boot, in microseconds. The wristband already only
    // uses these timestamps as deltas so the absolute origin doesn't
    // matter.
    let boot = Instant::now();
    let boot_micros = || -> i64 {
        Instant::now()
            .saturating_duration_since(boot)
            .as_micros()
            .try_into()
            .unwrap_or(i64::MAX)
    };

    let mut state = BeatSourceState::new();
    let mut next_tick = Instant::now();

    info!(
        "firefly-fw running — broadcasting v2 packets at {} Hz",
        BROADCAST_HZ
    );

    loop {
        // ── Drain DJ Link events ────────────────────────────────────
        loop {
            match dj_rx.try_recv() {
                Ok(DjLinkEvent::Beat(beat)) => {
                    let now_instant = Instant::now();
                    let now_us = boot_micros();
                    // Filter to first deck heard. We don't yet have a
                    // tempo-master election — the first deck that
                    // reports beats becomes master, identical to how
                    // the dongle firmware behaves with a single deck.
                    if state.master_device == 0 || beat.device_number == state.master_device {
                        state.process_master_beat(&beat, now_instant, now_us);
                    }
                }
                Ok(DjLinkEvent::Status(status)) => match state.process_cdj_status(&status) {
                    CdjPlayTransition::Paused => {
                        info!("CDJ master paused/cued");
                    }
                    CdjPlayTransition::PlayStarted => {
                        info!("CDJ master play/cue pressed — firing immediate flash");
                    }
                    CdjPlayTransition::None => {}
                },
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    warn!("DJ Link channel disconnected");
                    break;
                }
            }
        }

        // ── CDJ timeout detection ──────────────────────────────────
        if state.check_cdj_timeout(Instant::now()) {
            info!("CDJ beat timeout — falling back to internal clock");
        }

        // ── Build packet ───────────────────────────────────────────
        let now_us = boot_micros();
        state.tick_predicted_beats(now_us);
        let cdj_active = state.is_cdj_active(Instant::now());

        let (tempo, beat_in_bar, next_db_us, next_bt_us, is_playing) = if cdj_active {
            (
                state.cdj_bpm,
                state.cdj_beat_in_bar,
                state.cdj_next_bar_us,
                state.cdj_next_beat_us,
                true,
            )
        } else {
            // No CDJ + no Ableton Link on-device. Emit a not-playing
            // stream at 120 BPM so the wristband shows the idle
            // breathing animation.
            (120.0_f64, 0u8, 0i64, 0i64, false)
        };

        if beat_in_bar == 0 && state.last_beat_in_bar != 0 && state.last_beat_in_bar != u8::MAX {
            state.bar_counter = state.bar_counter.wrapping_add(1);
        }
        state.last_beat_in_bar = beat_in_bar;

        let mut flags: u8 = 0;
        if is_playing {
            flags |= FLAG_PLAYING;
        }
        if cdj_active {
            flags |= FLAG_CDJ_ACTIVE;
        }

        let mut next_bt_us = next_bt_us;
        if state.pending_play_flash {
            next_bt_us = now_us;
            flags |= FLAG_PLAYING;
            state.pending_play_flash = false;
        }

        let on_air_mask = state.on_air_mask();
        let master_dev = if cdj_active { state.master_device } else { 0 };

        let packet = build_packet(
            now_us,
            next_db_us,
            (tempo * 100.0) as u16,
            beat_in_bar,
            flags,
            next_bt_us,
            on_air_mask,
            master_dev,
        );

        if let Err(e) = broadcaster.send(&packet) {
            debug!("ESP-NOW send error: {:?}", e);
        }

        // Push live values to the OLED thread.
        live_state
            .tempo_x100
            .store((tempo * 100.0) as u16, Ordering::Relaxed);
        let prev_bib = live_state.beat_in_bar.swap(beat_in_bar, Ordering::Relaxed);
        if cdj_active {
            live_state.last_fwd_ms.store(
                Instant::now().saturating_duration_since(boot).as_millis() as u32,
                Ordering::Relaxed,
            );
        }
        // Wake the OLED thread on every beat-in-bar change so the
        // beat counter advances within ~1 ms of the new edge — same
        // behaviour as the dongle firmware's xTaskNotifyGive.
        if beat_in_bar != prev_bib {
            live_state.kick.store(true, Ordering::Relaxed);
        }

        // Fixed-rate scheduling — independent of loop body cost.
        next_tick += BROADCAST_PERIOD;
        let now = Instant::now();
        if next_tick > now {
            thread::sleep(next_tick - now);
        } else {
            // We've fallen behind — resync to "now" instead of
            // bursting to catch up (which would tighten beat edges
            // visibly on the wristband). Always yield at least 1ms
            // so the IDLE task isn't starved (watchdog feeder).
            next_tick = now + BROADCAST_PERIOD;
            thread::sleep(Duration::from_millis(1));
        }

        // Keep `_quantum` referenced — it'll matter once Link-fallback
        // gets reintroduced.
        let _ = QUANTUM;
    }
}
