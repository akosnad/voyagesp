#![no_std]

use alloc::{format, string::String};
use heapless::Vec;
use serde::{Deserialize, Serialize};

extern crate alloc;

pub const DISCOVERY_PREFIX: &str = "homeassistant";

pub trait Discoverable {
    fn discovery_topic(&self) -> Topic;
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
    pub hw_version: Option<String>,
    pub sw_version: Option<String>,
    pub name: Option<String>,
    pub identifiers: Vec<String, 5>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct UniqueId(pub String);

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Topic(pub String);

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct DeviceTracker {
    /// Only first availability is used by us, the rest are ignored.
    pub availability: Vec<Availability, 5>,
    pub availability_mode: Option<AvailabilityMode>,
    pub device: Option<Device>,
    pub unique_id: UniqueId,
    pub name: String,
    pub json_attributes_topic: Topic,
}
impl Discoverable for DeviceTracker {
    fn discovery_topic(&self) -> Topic {
        Topic(format!(
            "{}/device_tracker/{}/config",
            DISCOVERY_PREFIX, self.unique_id.0
        ))
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct DeviceTrackerAttributes {
    pub longitude: f64,
    pub latitude: f64,
    pub gps_accuracy: Option<f64>,
}
