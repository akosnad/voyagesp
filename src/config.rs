use alloc::vec;
use embassy_sync::once_lock::OnceLock;
use hass_types::*;

static CONFIG: OnceLock<SystemConfig> = OnceLock::new();

#[derive(Debug, Clone)]
pub struct SystemConfig {
    pub device_tracker: DeviceTracker,
}
impl SystemConfig {
    pub fn get() -> &'static Self {
        CONFIG.get_or_init(|| Self {
            device_tracker: include!(concat!(env!("OUT_DIR"), "/device_tracker_config.rs")),
        })
    }
}
