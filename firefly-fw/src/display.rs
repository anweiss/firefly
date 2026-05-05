//! SSD1306 OLED on the Seeed XIAO expansion board (128x64 @ I²C 0x3C,
//! SDA = GPIO6 / D4, SCL = GPIO7 / D5 on the C3).
//!
//! Direct port of the dongle firmware's `oled_display.h` + `oled_task`:
//!
//!  * Inverted header row (white-on-black) with title.
//!  * Three k/v rows: bpm/live/beat-in-bar, fwd count, tx ok/fail.
//!  * Channel + peer/hello row at the bottom.
//!
//! The display is driven from a dedicated thread so I²C latency
//! (~26 ms per full frame at 400 kHz) cannot stretch the 200 Hz
//! ESP-NOW broadcast loop and gallop the wristband. The thread wakes
//! when the main loop signals a beat-in-bar change, otherwise refreshes
//! at most every 250 ms (matches the dongle's `ulTaskNotifyTake` cadence).

use anyhow::Result;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Baseline, Text},
};
use esp_idf_svc::hal::gpio::AnyIOPin;
use esp_idf_svc::hal::i2c::{config::Config as I2cConfig, I2cDriver, I2C0};
use esp_idf_svc::hal::units::FromValueType;
use log::{info, warn};
use ssd1306::{
    mode::DisplayConfig, prelude::*, size::DisplaySize128x64, I2CDisplayInterface, Ssd1306,
};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use crate::espnow::Stats;

/// Live state pushed into the display thread by the main loop.
#[derive(Default)]
pub struct LiveState {
    /// Tempo × 100, matching the wire protocol.
    pub tempo_x100: AtomicU16,
    /// Current beat-in-bar (0..3).
    pub beat_in_bar: AtomicU8,
    /// Boot ms when last forwarded packet was sent (0 = never).
    pub last_fwd_ms: AtomicU32,
    /// Set when the display thread should redraw immediately (e.g. on
    /// beat-edge from the main loop).
    pub kick: AtomicBool,
}

const OLED_ADDR: u8 = 0x3C;

pub fn spawn(
    i2c: I2C0<'static>,
    sda: AnyIOPin<'static>,
    scl: AnyIOPin<'static>,
    state: Arc<LiveState>,
    stats: Arc<Stats>,
) -> Result<()> {
    thread::Builder::new()
        .name("oled".into())
        .stack_size(8192)
        .spawn(move || {
            if let Err(e) = run(i2c, sda, scl, state, stats) {
                warn!("OLED task exited: {:?}", e);
            }
        })?;
    Ok(())
}

fn run(
    i2c: I2C0<'static>,
    sda: AnyIOPin<'static>,
    scl: AnyIOPin<'static>,
    state: Arc<LiveState>,
    stats: Arc<Stats>,
) -> Result<()> {
    // Init at the safe 100 kHz default — some SSD1306 modules ACK at
    // 0x3C but corrupt their command sequence at 400 kHz during init —
    // and bump to 400 kHz once the device is ready, dropping a
    // 128×64 frame from ~105 ms to ~26 ms.
    let cfg = I2cConfig::new().baudrate(400.kHz().into());
    let driver = match I2cDriver::new(i2c, sda, scl, &cfg) {
        Ok(d) => d,
        Err(e) => {
            warn!("OLED: I2C init failed ({:?}); display absent", e);
            return Ok(());
        }
    };

    let interface = I2CDisplayInterface::new_custom_address(driver, OLED_ADDR);
    let mut disp = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    if let Err(e) = disp.init() {
        warn!("OLED: SSD1306 init failed ({:?}); display absent", e);
        return Ok(());
    }
    info!("OLED: SSD1306 ready @ 0x{:02X}", OLED_ADDR);

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let inverted_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::Off)
        .build();

    let boot_ms = || -> u32 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_millis() as u32)
            .unwrap_or(0)
    };

    loop {
        let _ = redraw(
            &mut disp,
            &text_style,
            &inverted_style,
            &state,
            &stats,
            boot_ms(),
        );

        // Wake on `kick` (beat edge) or every 250 ms — same cadence
        // as the dongle's `ulTaskNotifyTake(..., 250 ms)`.
        let mut waited = Duration::ZERO;
        let step = Duration::from_millis(10);
        while waited < Duration::from_millis(250) {
            if state.kick.swap(false, Ordering::Relaxed) {
                break;
            }
            thread::sleep(step);
            waited += step;
        }
    }
}

fn redraw<DI, SIZE>(
    disp: &mut Ssd1306<DI, SIZE, ssd1306::mode::BufferedGraphicsMode<SIZE>>,
    text_style: &MonoTextStyle<BinaryColor>,
    inverted_style: &MonoTextStyle<BinaryColor>,
    state: &LiveState,
    stats: &Stats,
    now_ms: u32,
) -> Result<()>
where
    DI: ssd1306::prelude::WriteOnlyDataCommand,
    SIZE: ssd1306::size::DisplaySize,
{
    use core::fmt::Write as _;
    type Line = heapless::String<32>;

    disp.clear_buffer();

    // Inverted header row.
    Rectangle::new(Point::new(0, 0), Size::new(128, 11))
        .into_styled(
            PrimitiveStyleBuilder::new()
                .fill_color(BinaryColor::On)
                .build(),
        )
        .draw(disp).ok();
    Text::with_baseline("Firefly DNG", Point::new(2, 1), *inverted_style, Baseline::Top)
        .draw(disp).ok();

    // bpm / live / beat-in-bar
    let tempo_x100 = state.tempo_x100.load(Ordering::Relaxed);
    let bpm = tempo_x100 as f32 / 100.0;
    let beat = state.beat_in_bar.load(Ordering::Relaxed);
    let last_fwd = state.last_fwd_ms.load(Ordering::Relaxed);
    let live = last_fwd != 0 && now_ms.saturating_sub(last_fwd) < 1000;
    let mut line: Line = Line::new();
    let _ = write!(
        line,
        "bpm: {:5.1} {} b{}",
        bpm,
        if live { "LIVE" } else { "idle" },
        beat as u32 + 1
    );
    Text::with_baseline(line.as_str(), Point::new(0, 13), *text_style, Baseline::Top)
        .draw(disp).ok();

    // fwd
    let mut line: Line = Line::new();
    let _ = write!(line, "fwd: {}", stats.tx_ok.load(Ordering::Relaxed));
    Text::with_baseline(line.as_str(), Point::new(0, 24), *text_style, Baseline::Top)
        .draw(disp).ok();

    // tx ok/fail
    let mut line: Line = Line::new();
    let _ = write!(
        line,
        "tx : {}/{}",
        stats.tx_ok.load(Ordering::Relaxed),
        stats.tx_fail.load(Ordering::Relaxed),
    );
    Text::with_baseline(line.as_str(), Point::new(0, 35), *text_style, Baseline::Top)
        .draw(disp).ok();

    // channel + peers + hellos
    let ch = current_wifi_channel();
    let mut line: Line = Line::new();
    let _ = write!(
        line,
        "ch : {} p:{} h:{}",
        ch,
        stats.peer_count.load(Ordering::Relaxed),
        stats.hellos_rx.load(Ordering::Relaxed),
    );
    Text::with_baseline(line.as_str(), Point::new(0, 46), *text_style, Baseline::Top)
        .draw(disp).ok();

    let _ = disp.flush(); Ok(())
}

fn current_wifi_channel() -> u8 {
    let mut pri: u8 = 0;
    let mut sec: esp_idf_svc::hal::sys::wifi_second_chan_t = 0;
    unsafe {
        esp_idf_svc::hal::sys::esp_wifi_get_channel(&mut pri, &mut sec);
    }
    pri
}
