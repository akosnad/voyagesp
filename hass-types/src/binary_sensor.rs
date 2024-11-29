use alloc::string::ToString;

use super::*;

#[derive(Debug, Clone, Deserialize, Serialize)]
#[allow(non_camel_case_types)]
pub enum BinarySensorDeviceClass {
    // source: https://developers.home-assistant.io/docs/core/entity/binary-sensor/#available-device-classes
    battery,
    battery_charging,
    co,
    cold,
    connectivity,
    door,
    garage_door,
    gas,
    heat,
    light,
    lock,
    moisture,
    motion,
    moving,
    occupancy,
    opening,
    plug,
    power,
    presence,
    problem,
    running,
    safety,
    smoke,
    sound,
    tamper,
    update,
    vibration,
    window,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BinarySensor {
    /// Only first availability is used by us, the rest are ignored.
    pub availability: Vec<Availability, 5>,
    pub availability_mode: Option<AvailabilityMode>,
    pub device: Option<Device>,
    pub unique_id: UniqueId,
    pub name: String,
    pub device_class: Option<BinarySensorDeviceClass>,
    pub state_topic: Topic,
    pub json_attributes_topic: Option<Topic>,
    pub enabled_by_default: Option<bool>,
    pub off_delay: Option<u32>,
    #[serde(default = "default_payload_on")]
    pub payload_on: Option<String>,
    #[serde(default = "default_payload_off")]
    pub payload_off: Option<String>,
    pub entity_category: Option<EntityCategory>,
}
impl Default for BinarySensor {
    fn default() -> Self {
        Self {
            availability: Default::default(),
            availability_mode: Default::default(),
            device: Default::default(),
            unique_id: Default::default(),
            name: Default::default(),
            device_class: Default::default(),
            state_topic: Default::default(),
            json_attributes_topic: Default::default(),
            enabled_by_default: Default::default(),
            off_delay: Default::default(),
            payload_on: default_payload_on(),
            payload_off: default_payload_off(),
            entity_category: Default::default(),
        }
    }
}

fn default_payload_on() -> Option<String> {
    Some("true".to_string())
}
fn default_payload_off() -> Option<String> {
    Some("false".to_string())
}

impl Discoverable for BinarySensor {
    fn discovery_topic(&self) -> Topic {
        Topic(format!(
            "{}/binary_sensor/{}/config",
            DISCOVERY_PREFIX, self.unique_id.0
        ))
    }
}

impl Publishable for BinarySensor {
    fn state_topic(&self) -> Topic {
        self.state_topic.clone()
    }
}
