//! ESP-NOW broadcaster. Mirrors the Arduino dongle firmware's behaviour:
//! - broadcast peer at FF:FF:FF:FF:FF:FF
//! - 11b @ 1Mbps rate (per stored memory `espnow rate` — LR mode caused
//!   bursty arrival that breaks beat-flash timing)
//! - channel 11 (per stored memory `espnow channel`)
//! - ingest 8-byte hello frames from wristbands and add them as unicast
//!   peers (max 8) so we can also unicast to known devices

use anyhow::Result;
use esp_idf_svc::espnow::{EspNow, PeerInfo, BROADCAST};
use esp_idf_svc::hal::sys::wifi_phy_rate_t_WIFI_PHY_RATE_1M_L;
use heapless::Vec;
use log::{debug, info, warn};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

/// Hello-frame sync bytes from `shared/protocol.h`.
const HELLO_SYNC: [u8; 2] = [0xBE, 0xA8];
const HELLO_SIZE: usize = 8;

const MAX_UNICAST_PEERS: usize = 8;

#[derive(Default)]
pub struct Stats {
    pub tx_ok: AtomicU32,
    pub tx_fail: AtomicU32,
    pub hellos_rx: AtomicU32,
    pub peer_count: AtomicU32,
}

pub struct Broadcaster {
    espnow: EspNow<'static>,
    peers: Arc<Mutex<Vec<[u8; 6], MAX_UNICAST_PEERS>>>,
    stats: Arc<Stats>,
}

impl Broadcaster {
    pub fn new() -> Result<Self> {
        let espnow = EspNow::take()?;

        // Add broadcast peer
        let mut peer = PeerInfo::default();
        peer.peer_addr.copy_from_slice(&BROADCAST);
        peer.channel = 0; // 0 = current channel
        peer.encrypt = false;
        // Ensure the broadcast peer uses the same low-rate (long-link-budget)
        // phy as the wristbands.
        unsafe {
            esp_idf_svc::hal::sys::esp_wifi_config_espnow_rate(
                esp_idf_svc::hal::sys::wifi_interface_t_WIFI_IF_STA,
                wifi_phy_rate_t_WIFI_PHY_RATE_1M_L,
            );
        }
        espnow.add_peer(peer)?;

        let peers = Arc::new(Mutex::new(Vec::new()));
        let stats = Arc::new(Stats::default());

        // Hello-frame ingest: wristbands send 8-byte hello packets every
        // 1 s when idle / 5 s when live. Capture the source MAC and add
        // as unicast peer.
        let peers_cb = peers.clone();
        let stats_cb = stats.clone();
        espnow.register_recv_cb(
            move |info: &esp_idf_svc::espnow::ReceiveInfo, data: &[u8]| {
                let mac = &info.src_addr;
                if data.len() == HELLO_SIZE && data[0] == HELLO_SYNC[0] && data[1] == HELLO_SYNC[1]
                {
                    stats_cb.hellos_rx.fetch_add(1, Ordering::Relaxed);
                    let addr: [u8; 6] = **mac;
                    if let Ok(mut peers) = peers_cb.lock() {
                        if !peers.contains(&addr) && peers.push(addr).is_ok() {
                            stats_cb
                                .peer_count
                                .store(peers.len() as u32, Ordering::Relaxed);
                            info!(
                            "ESP-NOW: paired wristband {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
                        );
                        }
                    }
                }
            },
        )?;

        Ok(Self {
            espnow,
            peers,
            stats,
        })
    }

    pub fn stats(&self) -> Arc<Stats> {
        self.stats.clone()
    }

    #[allow(dead_code)]
    pub fn peer_count(&self) -> usize {
        self.peers.lock().map(|p| p.len()).unwrap_or(0)
    }

    /// Add a unicast peer to the underlying ESP-NOW peer table. Called
    /// lazily as new wristbands hello us.
    fn ensure_unicast_peer(&self, addr: &[u8; 6]) {
        let mut peer = PeerInfo::default();
        peer.peer_addr.copy_from_slice(addr);
        peer.channel = 0;
        peer.encrypt = false;
        let _ = self.espnow.add_peer(peer);
    }

    /// Broadcast a packet, plus unicast to every known wristband peer
    /// for redundancy (matches dongle firmware).
    pub fn send(&self, packet: &[u8]) -> Result<()> {
        match self.espnow.send(BROADCAST, packet) {
            Ok(_) => {
                self.stats.tx_ok.fetch_add(1, Ordering::Relaxed);
            }
            Err(e) => {
                let n = self.stats.tx_fail.fetch_add(1, Ordering::Relaxed);
                // Rate-limit: ESP_ERR_ESPNOW_NO_MEM bursts during Wi-Fi
                // STA background activity are benign (next 10 ms tick
                // supersedes the dropped frame). Log every 256th drop
                // so the operator still sees if it goes pathological.
                if n.is_power_of_two() {
                    warn!(
                        "ESP-NOW broadcast send failed: {:?} (total fails: {})",
                        e,
                        n + 1
                    );
                }
            }
        }

        if let Ok(peers) = self.peers.lock() {
            for addr in peers.iter() {
                // Lazy peer-table population: ESP-NOW requires peer
                // entries for unicast — register on first send.
                self.ensure_unicast_peer(addr);
                if let Err(e) = self.espnow.send(*addr, packet) {
                    debug!("ESP-NOW unicast send failed to {:?}: {:?}", addr, e);
                }
            }
        }
        Ok(())
    }
}
