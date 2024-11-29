use super::*;

#[derive(Debug, Clone, Deserialize, Serialize)]
#[allow(non_camel_case_types)]
pub enum SensorDeviceClass {
    // source: https://developers.home-assistant.io/docs/core/entity/sensor/#available-device-classes
    apparent_power,
    aqi,
    atmospheric_pressure,
    battery,
    blood_glucose_concentration,
    co2,
    co,
    conductivity,
    current,
    data_rate,
    data_size,
    date,
    distance,
    duration,
    energy,
    energy_storage,
    r#enum,
    frequency,
    gas,
    humidity,
    illuminance,
    irradiance,
    moisture,
    monetary,
    nitrogen_dioxide,
    nitrogen_monoxide,
    nitrous_oxide,
    ozone,
    ph,
    pm1,
    pm25,
    pm10,
    power,
    power_factor,
    precipitation,
    precipitation_intensity,
    pressure,
    reactive_power,
    signal_strength,
    sound_pressure,
    speed,
    sulphur_dioxide,
    temperature,
    timestamp,
    volatile_organic_compounds,
    volatile_organic_compounds_parts,
    voltage,
    volume,
    volume_flow_rate,
    volume_storage,
    water,
    weight,
    wind_speed,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorStateClass {
    // source: https://developers.home-assistant.io/docs/core/entity/sensor/#available-state-classes
    Measurement,
    Total,
    TotalIncreasing,
}

#[derive(Debug, Clone, Deserialize, Serialize, Default)]
pub struct Sensor {
    /// Only first availability is used by us, the rest are ignored.
    pub availability: Vec<Availability, 5>,
    pub availability_mode: Option<AvailabilityMode>,
    pub device: Option<Device>,
    pub unique_id: UniqueId,
    pub name: String,
    pub device_class: Option<SensorDeviceClass>,
    pub state_class: Option<SensorStateClass>,
    pub state_topic: Topic,
    pub json_attributes_topic: Option<Topic>,
    pub entity_category: Option<EntityCategory>,
    pub suggested_display_precision: Option<u8>,
    pub unit_of_measurement: Option<String>,
}

impl Discoverable for Sensor {
    fn discovery_topic(&self) -> Topic {
        Topic(format!(
            "{}/sensor/{}/config",
            DISCOVERY_PREFIX, self.unique_id.0
        ))
    }
}

impl Publishable for Sensor {
    fn state_topic(&self) -> Topic {
        self.state_topic.clone()
    }
}
