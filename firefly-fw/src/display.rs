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
use core::sync::atomic::{AtomicBool, AtomicI8, AtomicI32, AtomicU16, AtomicU32, AtomicU8, Ordering};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::{Baseline, Text},
};
use esp_idf_svc::hal::adc::attenuation::DB_12;
use esp_idf_svc::hal::adc::oneshot::config::{AdcChannelConfig, Calibration};
use esp_idf_svc::hal::adc::oneshot::{AdcChannelDriver, AdcDriver};
use esp_idf_svc::hal::adc::ADC1;
use esp_idf_svc::hal::gpio::{AnyIOPin, Gpio4};
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
    /// Smoothed battery voltage (mV) at the cell, after dividing.
    /// 0 = uninitialised.
    pub vbat_mv: AtomicI32,
    /// Battery percentage (0..=100) or -1 if no divider wired.
    pub vbat_percent: AtomicI8,
}

const OLED_ADDR: u8 = 0x3C;

/// 100k:100k voltage divider from BAT to GPIO4 (A2 / D2 on the XIAO
/// expansion board). See `shared/battery_sense.h` for wiring.
const VBAT_DIVIDER_NUM: i32 = 2;
const VBAT_DIVIDER_DEN: i32 = 1;

/// Plausibility window — readings outside this range are treated as
/// "no divider wired" and the display shows "--".
const VBAT_MIN_MV: i32 = 2500;
const VBAT_MAX_MV: i32 = 4400;

pub fn spawn(
    i2c: I2C0<'static>,
    sda: AnyIOPin<'static>,
    scl: AnyIOPin<'static>,
    adc1: ADC1<'static>,
    vbat_pin: Gpio4<'static>,
    state: Arc<LiveState>,
    stats: Arc<Stats>,
) -> Result<()> {
    thread::Builder::new()
        .name("oled".into())
        .stack_size(8192)
        .spawn(move || {
            if let Err(e) = run(i2c, sda, scl, adc1, vbat_pin, state, stats) {
                warn!("OLED task exited: {:?}", e);
            }
        })?;
    Ok(())
}

fn run(
    i2c: I2C0<'static>,
    sda: AnyIOPin<'static>,
    scl: AnyIOPin<'static>,
    adc1: ADC1<'static>,
    vbat_pin: Gpio4<'static>,
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
    // Match the wristband / dongle Adafruit_SSD1306 default contrast
    // (0xCF for SWITCHCAPVCC). The ssd1306 crate's default is much
    // dimmer (NORMAL = 0x5F).
    if let Err(e) = disp.set_brightness(Brightness::BRIGHTEST) {
        warn!("OLED: set_brightness failed ({:?})", e);
    }
    info!("OLED: SSD1306 ready @ 0x{:02X}", OLED_ADDR);

    // ADC oneshot driver for battery sense. If it fails to initialise
    // (e.g. ADC peripheral already taken) we fall back to "no battery"
    // mode rather than aborting the whole display task.
    let adc = AdcDriver::new(adc1).ok();
    let mut chan = adc.as_ref().and_then(|a| {
        let cfg = AdcChannelConfig {
            attenuation: DB_12,
            calibration: Calibration::Curve,
            ..Default::default()
        };
        AdcChannelDriver::new(a, vbat_pin, &cfg).ok()
    });
    if chan.is_none() {
        warn!("OLED: ADC init failed; battery row will show '--'");
    }

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
        // Sample battery on every refresh tick (cheap, <100 µs).
        if let (Some(adc), Some(chan)) = (adc.as_ref(), chan.as_mut()) {
            if let Ok(adc_mv) = adc.read(chan) {
                let bat_mv = adc_mv as i32 * VBAT_DIVIDER_NUM / VBAT_DIVIDER_DEN;
                let prev = state.vbat_mv.load(Ordering::Relaxed);
                let smoothed = if prev == 0 {
                    bat_mv
                } else {
                    (prev * 7 + bat_mv) / 8
                };
                state.vbat_mv.store(smoothed, Ordering::Relaxed);
                state
                    .vbat_percent
                    .store(lipo_percent(smoothed), Ordering::Relaxed);
            }
        }

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

/// Piecewise-linear LiPo voltage→percent mapping. Mirrors
/// `firefly_battery_percent` in `shared/battery_sense.h`. Returns -1
/// when the voltage is outside the plausible 1S LiPo window (likely
/// the divider isn't wired and the ADC is floating).
fn lipo_percent(vbat_mv: i32) -> i8 {
    if !(VBAT_MIN_MV..=VBAT_MAX_MV).contains(&vbat_mv) {
        return -1;
    }
    const CURVE: &[(i32, i8)] = &[
        (3300, 0),
        (3500, 5),
        (3600, 10),
        (3700, 25),
        (3800, 45),
        (3900, 65),
        (4000, 80),
        (4100, 90),
        (4200, 100),
    ];
    if vbat_mv >= CURVE[CURVE.len() - 1].0 {
        return 100;
    }
    if vbat_mv <= CURVE[0].0 {
        return 0;
    }
    for w in CURVE.windows(2) {
        let (lo_mv, lo_pc) = w[0];
        let (hi_mv, hi_pc) = w[1];
        if vbat_mv <= hi_mv {
            let span_mv = hi_mv - lo_mv;
            let span_pc = (hi_pc - lo_pc) as i32;
            let off_mv = vbat_mv - lo_mv;
            return (lo_pc as i32 + off_mv * span_pc / span_mv) as i8;
        }
    }
    -1
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
        .draw(disp)
        .ok();
    Text::with_baseline(
        "Firefly DNG",
        Point::new(2, 1),
        *inverted_style,
        Baseline::Top,
    )
    .draw(disp)
    .ok();

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
        .draw(disp)
        .ok();

    // tx ok/fail
    let mut line: Line = Line::new();
    let _ = write!(
        line,
        "tx : {}/{}",
        stats.tx_ok.load(Ordering::Relaxed),
        stats.tx_fail.load(Ordering::Relaxed),
    );
    Text::with_baseline(line.as_str(), Point::new(0, 24), *text_style, Baseline::Top)
        .draw(disp)
        .ok();

    // channel + peers + hellos + wifi state. Wifi probed live via
    // esp_wifi_sta_get_ap_info — ESP_OK = connected (rssi available),
    // ESP_ERR_WIFI_NOT_CONNECT = scanning/associating, anything else
    // (incl. NOT_INIT/NOT_STARTED) treated as down.
    let ch = current_wifi_channel();
    let mut line: Line = Line::new();
    match wifi_link_status() {
        WifiLink::Connected(rssi) => {
            let _ = write!(line, "ch:{} W:{}dB", ch, rssi);
        }
        WifiLink::Connecting => {
            let _ = write!(line, "ch:{} W:..", ch);
        }
        WifiLink::Down => {
            let _ = write!(line, "ch:{} W:NO", ch);
        }
    }
    let _ = write!(
        line,
        " p:{}",
        stats.peer_count.load(Ordering::Relaxed),
    );
    Text::with_baseline(line.as_str(), Point::new(0, 35), *text_style, Baseline::Top)
        .draw(disp)
        .ok();

    // battery — "--" when divider not wired (vbat_percent stays at -1
    // because the floating ADC reads outside the LiPo plausibility
    // window).
    let pct = state.vbat_percent.load(Ordering::Relaxed);
    let mv = state.vbat_mv.load(Ordering::Relaxed);
    let mut line: Line = Line::new();
    if pct >= 0 {
        let _ = write!(line, "bat: {}% ({} mV)", pct, mv);
    } else {
        let _ = write!(line, "bat: --");
    }
    Text::with_baseline(line.as_str(), Point::new(0, 46), *text_style, Baseline::Top)
        .draw(disp)
        .ok();

    let _ = disp.flush();
    Ok(())
}

fn current_wifi_channel() -> u8 {
    let mut pri: u8 = 0;
    let mut sec: esp_idf_svc::hal::sys::wifi_second_chan_t = 0;
    unsafe {
        esp_idf_svc::hal::sys::esp_wifi_get_channel(&mut pri, &mut sec);
    }
    pri
}

enum WifiLink {
    Connected(i8),
    Connecting,
    Down,
}

fn wifi_link_status() -> WifiLink {
    use esp_idf_svc::hal::sys::{
        esp_wifi_sta_get_ap_info, wifi_ap_record_t, ESP_ERR_WIFI_NOT_CONNECT, ESP_OK,
    };
    let mut info: wifi_ap_record_t = unsafe { core::mem::zeroed() };
    let err = unsafe { esp_wifi_sta_get_ap_info(&mut info) };
    if err == ESP_OK {
        WifiLink::Connected(info.rssi)
    } else if err == ESP_ERR_WIFI_NOT_CONNECT {
        WifiLink::Connecting
    } else {
        WifiLink::Down
    }
}
