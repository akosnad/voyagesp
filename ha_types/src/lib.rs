use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HAEntity {
    pub name: String,
    pub variant: HAEntityVariant,
    pub unique_id: String,
    pub state_topic: String,
    pub icon: Option<String>,
    #[serde(skip_deserializing)]
    pub availability: Option<HADeviceAvailability>,
    pub device: Option<HADevice>,
    pub device_class: Option<String>,
    pub entity_category: Option<String>,
    pub gpio_pin: Option<u8>,
    pub command_topic: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HAEntityOut {
    pub name: String,
    pub unique_id: String,
    pub state_topic: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub icon: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub availability: Option<HADeviceAvailabilityOut>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub device: Option<HADeviceOut>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub device_class: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub entity_category: Option<String>,

    #[serde(skip_serializing_if = "Option::is_none")]
    pub code_arm_required: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub code_disarm_required: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub code_trigger_required: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub command_topic: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub supported_features: Option<Vec<String>>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub enum HAEntityVariant {
    binary_sensor,
    sensor,
    alarm_control_panel,
}
impl std::fmt::Display for HAEntityVariant {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HAEntityVariant::binary_sensor => write!(f, "binary_sensor"),
            HAEntityVariant::sensor => write!(f, "sensor"),
            HAEntityVariant::alarm_control_panel => write!(f, "alarm_control_panel"),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HADeviceAvailability {
    pub payload_available: Option<String>,
    pub payload_not_available: Option<String>,
    pub topic: String,
    pub value_template: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HADeviceAvailabilityOut {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub payload_available: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub payload_not_available: Option<String>,
    pub topic: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value_template: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HADevice {
    pub configuration_url: Option<String>,
    pub hw_version: Option<String>,
    pub identifiers: Option<Vec<String>>,
    pub manufacturer: Option<String>,
    pub model: Option<String>,
    pub name: Option<String>,
    pub serial_number: Option<String>,
    pub suggested_area: Option<String>,
    pub sw_version: Option<String>,
    pub via_device: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HADeviceOut {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub configuration_url: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hw_version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub identifiers: Option<Vec<String>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub manufacturer: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub serial_number: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub suggested_area: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sw_version: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub via_device: Option<String>,
}

impl From<HAEntity> for HAEntityOut {
    fn from(entity: HAEntity) -> Self {
        if entity.variant == HAEntityVariant::alarm_control_panel {
            HAEntityOut {
                name: entity.name,
                unique_id: entity.unique_id,
                state_topic: entity.state_topic,
                command_topic: entity.command_topic,
                icon: entity.icon,
                availability: entity.availability.map(|a| a.into()),
                device: entity.device.map(|d| d.into()),
                device_class: entity.device_class,
                entity_category: entity.entity_category,
                code_arm_required: Some(false),
                code_disarm_required: Some(false),
                code_trigger_required: Some(false),
                supported_features: Some(vec!["arm_away".to_string(), "trigger".to_string()]),
            }
        } else {
            HAEntityOut {
                name: entity.name,
                unique_id: entity.unique_id,
                state_topic: entity.state_topic,
                command_topic: None,
                icon: entity.icon,
                availability: entity.availability.map(|a| a.into()),
                device: entity.device.map(|d| d.into()),
                device_class: entity.device_class,
                entity_category: entity.entity_category,
                code_arm_required: None,
                code_disarm_required: None,
                code_trigger_required: None,
                supported_features: None,
            }
        }
    }
}

impl From<HADeviceAvailability> for HADeviceAvailabilityOut {
    fn from(availability: HADeviceAvailability) -> Self {
        HADeviceAvailabilityOut {
            payload_available: availability.payload_available,
            payload_not_available: availability.payload_not_available,
            topic: availability.topic,
            value_template: availability.value_template,
        }
    }
}

impl From<HADevice> for HADeviceOut {
    fn from(device: HADevice) -> Self {
        HADeviceOut {
            configuration_url: device.configuration_url,
            hw_version: device.hw_version,
            identifiers: device.identifiers,
            manufacturer: device.manufacturer,
            model: device.model,
            name: device.name,
            serial_number: device.serial_number,
            suggested_area: device.suggested_area,
            sw_version: device.sw_version,
            via_device: device.via_device,
        }
    }
}
