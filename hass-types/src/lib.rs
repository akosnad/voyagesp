#![no_std]

use alloc::{format, string::String};
use heapless::Vec;
use serde::{Deserialize, Serialize};

extern crate alloc;

mod binary_sensor;
mod button;
mod device_tracker;
mod sensor;

pub use binary_sensor::{BinarySensor, BinarySensorDeviceClass};
pub use button::{Button, ButtonDeviceClass};
pub use device_tracker::{DeviceTracker, DeviceTrackerAttributes};
pub use sensor::{Sensor, SensorDeviceClass, SensorStateClass};

pub const DISCOVERY_PREFIX: &str = "homeassistant";

pub trait Discoverable {
    fn discovery_topic(&self) -> Topic;
}

pub trait Publishable {
    fn state_topic(&self) -> Topic;
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, Default)]
pub enum AvailabilityMode {
    #[serde(rename = "all")]
    All,
    #[serde(rename = "any")]
    Any,
    #[default]
    #[serde(rename = "latest")]
    Latest,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Availability {
    pub payload_available: Option<String>,
    pub payload_not_available: Option<String>,
    pub topic: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Device {
    pub configuration_url: Option<String>,
    pub hw_version: Option<String>,
    pub sw_version: Option<String>,
    pub name: Option<String>,
    pub identifiers: Vec<String, 5>,
    pub model: Option<String>,
    pub model_id: Option<String>,
    pub via_device: Option<String>,
}

#[derive(Debug, Clone, Deserialize, Serialize, Default)]
pub struct UniqueId(pub String);

#[derive(Debug, Clone, Deserialize, Serialize, Default)]
pub struct Topic(pub String);

#[derive(Debug, Clone, Deserialize, Serialize)]
#[allow(non_camel_case_types)]
pub enum EntityCategory {
    config,
    diagnostic,
}
