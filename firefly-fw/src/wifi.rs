//! Wi-Fi station bringup. Joins the SSID baked in via env vars
//! `WIFI_SSID` / `WIFI_PASS`. Uses the same Wi-Fi channel as ESP-NOW
//! (channel 11 — see stored memory `espnow channel`); the AP must be
//! reachable on a channel matching `ESPNOW_CHANNEL` at runtime, since
//! ESP-NOW can only operate on the channel the radio is tuned to.

use anyhow::{anyhow, Result};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use heapless::String as HString;
use log::info;

const SSID: &str = env!("WIFI_SSID");
const PASS: &str = env!("WIFI_PASS");

/// Bring up Wi-Fi STA, block until associated and DHCP-bound.
pub fn connect(
    modem: Modem<'static>,
    sysloop: EspSystemEventLoop,
    nvs: EspDefaultNvsPartition,
) -> Result<BlockingWifi<EspWifi<'static>>> {
    let mut wifi = BlockingWifi::wrap(EspWifi::new(modem, sysloop.clone(), Some(nvs))?, sysloop)?;

    let mut ssid: HString<32> = HString::new();
    ssid.push_str(SSID).map_err(|_| anyhow!("SSID too long"))?;
    let mut password: HString<64> = HString::new();
    password
        .push_str(PASS)
        .map_err(|_| anyhow!("password too long"))?;

    wifi.start()?;
    info!("Wi-Fi: scanning for SSID='{}'", SSID);

    // Scan first so we can pick up the AP's actual auth method,
    // channel, and BSSID. Some routers run WPA2/WPA3 mixed mode and
    // refuse association if we hard-code WPA2Personal.
    let scan = wifi.scan().unwrap_or_default();
    info!("Wi-Fi: scan returned {} APs", scan.len());
    for ap in &scan {
        info!(
            "  ssid='{}' ch={} rssi={} auth={:?}",
            ap.ssid, ap.channel, ap.signal_strength, ap.auth_method
        );
    }
    let found = scan.iter().find(|a| a.ssid.as_str() == SSID);
    let (auth, channel, bssid) = match found {
        Some(ap) => {
            info!(
                "Wi-Fi: found AP — channel={} rssi={} auth={:?} bssid={:02X?}",
                ap.channel, ap.signal_strength, ap.auth_method, ap.bssid
            );
            (
                ap.auth_method.unwrap_or(AuthMethod::WPA2Personal),
                Some(ap.channel),
                Some(ap.bssid),
            )
        }
        None => {
            log::warn!(
                "Wi-Fi: SSID '{}' not in scan results — proceeding with default WPA2",
                SSID
            );
            (
                if PASS.is_empty() {
                    AuthMethod::None
                } else {
                    AuthMethod::WPA2Personal
                },
                None,
                None,
            )
        }
    };

    let mut cfg = ClientConfiguration {
        ssid,
        password,
        auth_method: auth,
        ..Default::default()
    };
    if let Some(ch) = channel {
        cfg.channel = Some(ch);
    }
    if let Some(b) = bssid {
        cfg.bssid = Some(b);
    }
    wifi.set_configuration(&Configuration::Client(cfg))?;

    wifi.connect()?;
    info!("Wi-Fi: associated, waiting for IP");
    wifi.wait_netif_up()?;

    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;
    info!("Wi-Fi: up — {:?}", ip_info);

    Ok(wifi)
}
