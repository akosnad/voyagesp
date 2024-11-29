use alloc::vec;
use embassy_sync::once_lock::OnceLock;
use hass_types::*;

static CONFIG: OnceLock<SystemConfig> = OnceLock::new();

#[derive(Debug, Clone)]
pub struct SystemConfig {
    pub device_tracker: DeviceTracker,
    pub ignition_sense_sensor: BinarySensor,
}
impl SystemConfig {
    pub fn get() -> &'static Self {
        CONFIG.get_or_init(|| Self {
            device_tracker: include!(concat!(env!("OUT_DIR"), "/device_tracker_config.rs")),
            ignition_sense_sensor: include!(concat!(
                env!("OUT_DIR"),
                "/ignition_sense_sensor_config.rs"
            )),
        })
    }
}
