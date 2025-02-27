use super::*;

#[derive(Debug, Clone, Deserialize, Serialize)]
#[allow(non_camel_case_types)]
pub enum ButtonDeviceClass {
    // source: https://developers.home-assistant.io/docs/core/entity/button#available-device-classes
    identify,
    restart,
    update,
}

#[derive(Debug, Clone, Deserialize, Serialize, Default)]
pub struct Button {
    /// Only first availability is used by us, the rest are ignored.
    pub availability: Vec<Availability, 5>,
    pub availability_mode: Option<AvailabilityMode>,
    pub device: Option<Device>,
    pub unique_id: UniqueId,
    pub name: String,
    pub device_class: Option<ButtonDeviceClass>,
    pub command_topic: Topic,
    pub payload_press: Option<String>,
    pub entity_category: Option<EntityCategory>,
}

impl Discoverable for Button {
    fn discovery_topic(&self) -> Topic {
        Topic(format!(
            "{}/button/{}/config",
            DISCOVERY_PREFIX, self.unique_id.0
        ))
    }
}

impl Publishable for Button {
    fn state_topic(&self) -> Topic {
        self.command_topic.clone()
    }
}
