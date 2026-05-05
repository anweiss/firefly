//! Beat-clock fusion state machine — ported from
//! `coordinator/src/main.rs::BeatSourceState`.
//!
//! IMPORTANT: this is a hand-port. Any change here must be mirrored to
//! `coordinator/src/main.rs` (and vice versa) so the host coordinator
//! and on-device coordinator behave identically. The host coordinator
//! has the canonical test suite — run `cargo test -p firefly-coordinator`
//! after any algorithmic change.

use core::time::Duration;
use std::collections::HashMap;
use std::time::Instant;

/// If no beat arrives within this window, fall back to Link-only mode.
pub const CDJ_TIMEOUT: Duration = Duration::from_secs(2);

/// Minimum BPM difference before we propagate a tempo change.
#[allow(dead_code)]
pub const TEMPO_EPSILON: f64 = 0.05;

/// Decoded Pro DJ Link Beat packet (subset that the coordinator uses).
/// We define this locally rather than depending on `prodjlink-rs` so the
/// firmware's beat-clock module stays portable.
#[derive(Debug, Clone, Copy)]
pub struct Beat {
    pub device_number: u8,
    /// 1..=4 in Pro DJ Link wire format (0 means "unknown").
    pub beat_within_bar: u8,
    /// BPM × 100 (already adjusted for the deck's pitch).
    pub effective_tempo_bpm: f64,
    /// Distance in ms to the next beat, if reported by the deck.
    pub next_beat_ms: Option<u32>,
    /// Distance in ms to the next bar, if reported by the deck.
    pub next_bar_ms: Option<u32>,
}

impl Beat {
    pub fn effective_tempo(&self) -> f64 {
        self.effective_tempo_bpm
    }
}

/// CDJ status update — subset used for play/pause detection.
#[derive(Debug, Clone, Copy)]
pub struct CdjStatus {
    pub device_number: u8,
    pub is_playing: bool,
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CdjPlayTransition {
    None,
    Paused,
    PlayStarted,
}

pub struct BeatSourceState {
    pub last_cdj_beat_time: Instant,
    pub cdj_beat_in_bar: u8,
    pub cdj_next_beat_us: i64,
    pub cdj_next_bar_us: i64,
    pub cdj_bpm: f64,
    pub cdj_playing: bool,
    pub master_device: u8,
    pub channels_on_air: HashMap<u8, bool>,
    pub last_beat_in_bar: u8,
    pub bar_counter: u8,
    last_smoothed_beat_us: i64,
    have_smoothed_beat: bool,
    pending_beat_in_bar: Option<u8>,
    pending_beat_at_us: i64,
    pub pending_play_flash: bool,
}

impl Default for BeatSourceState {
    fn default() -> Self {
        Self::new()
    }
}

impl BeatSourceState {
    pub fn new() -> Self {
        Self {
            last_cdj_beat_time: Instant::now()
                .checked_sub(CDJ_TIMEOUT)
                .unwrap_or_else(Instant::now),
            cdj_beat_in_bar: 0,
            cdj_next_beat_us: 0,
            cdj_next_bar_us: 0,
            cdj_bpm: 0.0,
            cdj_playing: false,
            master_device: 0,
            channels_on_air: HashMap::new(),
            last_beat_in_bar: u8::MAX,
            bar_counter: 0,
            last_smoothed_beat_us: 0,
            have_smoothed_beat: false,
            pending_beat_in_bar: None,
            pending_beat_at_us: 0,
            pending_play_flash: false,
        }
    }

    pub fn process_master_beat(&mut self, beat: &Beat, now: Instant, link_clock_us: i64) {
        self.last_cdj_beat_time = now;
        self.cdj_bpm = beat.effective_tempo();
        let target_beat_in_bar = if beat.beat_within_bar > 0 {
            beat.beat_within_bar - 1
        } else {
            0
        };
        self.cdj_playing = true;
        self.master_device = beat.device_number;

        let beat_period_us = if self.cdj_bpm > 0.0 {
            (60_000_000.0_f64 / self.cdj_bpm).round() as i64
        } else {
            0
        };

        let smoothed_beat_us = if self.have_smoothed_beat && beat_period_us > 0 {
            let expected = self.last_smoothed_beat_us + beat_period_us;
            let error = link_clock_us - expected;
            if error.abs() > beat_period_us / 2 {
                link_clock_us
            } else {
                expected + error / 8
            }
        } else {
            link_clock_us
        };
        self.last_smoothed_beat_us = smoothed_beat_us;
        self.have_smoothed_beat = true;

        self.pending_beat_in_bar = Some(target_beat_in_bar);
        self.pending_beat_at_us = smoothed_beat_us;

        if beat_period_us > 0 {
            self.cdj_next_beat_us = smoothed_beat_us + beat_period_us;
            let beats_to_bar = if target_beat_in_bar == 0 {
                4
            } else {
                4 - target_beat_in_bar as i64
            };
            self.cdj_next_bar_us = smoothed_beat_us + beats_to_bar * beat_period_us;
        } else {
            self.cdj_next_beat_us = beat
                .next_beat_ms
                .map(|ms| link_clock_us + (ms as i64) * 1000)
                .unwrap_or(0);
            self.cdj_next_bar_us = beat
                .next_bar_ms
                .map(|ms| link_clock_us + (ms as i64) * 1000)
                .unwrap_or(0);
        }
    }

    pub fn process_cdj_status(&mut self, status: &CdjStatus) -> CdjPlayTransition {
        let now_playing = status.is_playing;
        if self.master_device == 0 {
            if now_playing {
                self.master_device = status.device_number;
                self.cdj_playing = true;
                self.pending_play_flash = true;
                return CdjPlayTransition::PlayStarted;
            }
            return CdjPlayTransition::None;
        }
        if status.device_number != self.master_device {
            return CdjPlayTransition::None;
        }
        if self.cdj_playing && !now_playing {
            self.cdj_playing = false;
            self.have_smoothed_beat = false;
            self.pending_beat_in_bar = None;
            self.pending_play_flash = false;
            return CdjPlayTransition::Paused;
        }
        if !self.cdj_playing && now_playing {
            self.cdj_playing = true;
            self.pending_play_flash = true;
            return CdjPlayTransition::PlayStarted;
        }
        CdjPlayTransition::None
    }

    pub fn check_cdj_timeout(&mut self, now: Instant) -> bool {
        if self.cdj_playing && now.saturating_duration_since(self.last_cdj_beat_time) > CDJ_TIMEOUT
        {
            self.cdj_playing = false;
            self.have_smoothed_beat = false;
            self.pending_beat_in_bar = None;
            true
        } else {
            false
        }
    }

    pub fn is_cdj_active(&self, now: Instant) -> bool {
        self.cdj_playing && now.saturating_duration_since(self.last_cdj_beat_time) < CDJ_TIMEOUT
    }

    pub fn tick_predicted_beats(&mut self, link_clock_us: i64) -> bool {
        if !self.cdj_playing || self.cdj_bpm <= 0.0 {
            return false;
        }
        let beat_period_us = (60_000_000.0_f64 / self.cdj_bpm).round() as i64;
        if beat_period_us <= 0 {
            return false;
        }
        let mut advanced = false;

        if let Some(target) = self.pending_beat_in_bar {
            if link_clock_us >= self.pending_beat_at_us {
                self.cdj_beat_in_bar = target;
                self.pending_beat_in_bar = None;
                advanced = true;
            }
        }

        let mut guard = 0;
        while self.cdj_next_beat_us > 0 && link_clock_us >= self.cdj_next_beat_us && guard < 8 {
            self.cdj_beat_in_bar = (self.cdj_beat_in_bar + 1) % 4;
            let just_advanced_at = self.cdj_next_beat_us;
            self.cdj_next_beat_us += beat_period_us;
            let beats_to_bar = if self.cdj_beat_in_bar == 0 {
                4
            } else {
                4 - self.cdj_beat_in_bar as i64
            };
            self.cdj_next_bar_us = just_advanced_at + beats_to_bar * beat_period_us;
            guard += 1;
            advanced = true;
        }
        advanced
    }

    pub fn on_air_mask(&self) -> u8 {
        let mut mask: u8 = 0;
        for (&ch, &active) in &self.channels_on_air {
            if active && (1..=8).contains(&ch) {
                mask |= 1 << (ch - 1);
            }
        }
        mask
    }

    #[allow(dead_code)]
    pub fn update_on_air(&mut self, channels: HashMap<u8, bool>) {
        self.channels_on_air = channels;
    }
}
