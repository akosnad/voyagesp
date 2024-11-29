use super::*;

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

impl Publishable for DeviceTracker {
    fn state_topic(&self) -> Topic {
        self.json_attributes_topic.clone()
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct DeviceTrackerAttributes {
    pub longitude: f64,
    pub latitude: f64,
    pub gps_accuracy: Option<f64>,
}
