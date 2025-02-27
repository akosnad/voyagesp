use alloc::{borrow::ToOwned as _, format};
use embassy_sync::once_lock::OnceLock;
use hass_types::{BinarySensor, Button, Sensor, Topic};

use super::Client;

static DIAGNOSTIC_ENTITIES: OnceLock<DiagnosticEntities> = OnceLock::new();
pub struct DiagnosticEntities {
    pub battery_charging: BinarySensor,
    pub battery_voltage: Sensor,
    pub battery_charge_current: Sensor,
    pub ext_voltage: Sensor,
    pub ext_current: Sensor,
    pub int_temperature: Sensor,
    pub satellite_count: Sensor,
    pub reboot_button: Button,
}
impl DiagnosticEntities {
    pub fn get() -> &'static Self {
        DIAGNOSTIC_ENTITIES.get_or_init(|| {
            const TOPIC_PREFIX: &str = env!("DIAGNOSTIC_ENTITIES_TOPIC_PREFIX");
            let config = &crate::config::SystemConfig::get();
            let availability = config.ignition_sense_sensor.availability.to_owned();
            let device = config.ignition_sense_sensor.device.clone();

            let gen_sensor = |name: &'static str,
                              display_name: &'static str,
                              device_class: Option<hass_types::SensorDeviceClass>,
                              state_class: Option<hass_types::SensorStateClass>,
                              precision: Option<u8>,
                              unit_of_measurement: Option<&'static str>|
             -> Sensor {
                Sensor {
                    availability: availability.clone(),
                    device: device.clone(),
                    unique_id: hass_types::UniqueId(format!("{}_{}", TOPIC_PREFIX, name)),
                    name: display_name.into(),
                    device_class,
                    state_class,
                    state_topic: Topic(format!("{}/{}", TOPIC_PREFIX, name)),
                    entity_category: Some(hass_types::EntityCategory::diagnostic),
                    suggested_display_precision: precision,
                    unit_of_measurement: unit_of_measurement.map(Into::into),
                    ..Default::default()
                }
            };

            let gen_binary_sensor = |name: &'static str,
                                     display_name: &'static str,
                                     device_class: Option<hass_types::BinarySensorDeviceClass>|
             -> BinarySensor {
                BinarySensor {
                    availability: availability.clone(),
                    device: device.clone(),
                    unique_id: hass_types::UniqueId(format!("{}_{}", TOPIC_PREFIX, name)),
                    name: display_name.into(),
                    device_class,
                    state_topic: Topic(format!("{}/{}", TOPIC_PREFIX, name)),
                    entity_category: Some(hass_types::EntityCategory::diagnostic),
                    ..Default::default()
                }
            };

            Self {
                battery_charging: gen_binary_sensor(
                    "battery_charging",
                    "Battery Charging",
                    Some(hass_types::BinarySensorDeviceClass::battery_charging),
                ),
                battery_voltage: gen_sensor(
                    "battery_voltage",
                    "Battery Voltage",
                    Some(hass_types::SensorDeviceClass::voltage),
                    Some(hass_types::SensorStateClass::Measurement),
                    Some(3),
                    Some("V"),
                ),
                battery_charge_current: gen_sensor(
                    "battery_charge_current",
                    "Battery Charge Current",
                    Some(hass_types::SensorDeviceClass::current),
                    Some(hass_types::SensorStateClass::Measurement),
                    Some(4),
                    Some("A"),
                ),
                ext_voltage: gen_sensor(
                    "ext_voltage",
                    "External Voltage",
                    Some(hass_types::SensorDeviceClass::voltage),
                    Some(hass_types::SensorStateClass::Measurement),
                    Some(3),
                    Some("V"),
                ),
                ext_current: gen_sensor(
                    "ext_current",
                    "External Current",
                    Some(hass_types::SensorDeviceClass::current),
                    Some(hass_types::SensorStateClass::Measurement),
                    Some(4),
                    Some("A"),
                ),
                int_temperature: gen_sensor(
                    "int_temperature",
                    "Internal Temperature",
                    Some(hass_types::SensorDeviceClass::temperature),
                    Some(hass_types::SensorStateClass::Measurement),
                    Some(1),
                    Some("°C"),
                ),
                satellite_count: gen_sensor(
                    "satellite_count",
                    "GPS Satellite Count",
                    Some(hass_types::SensorDeviceClass::signal_strength),
                    Some(hass_types::SensorStateClass::Measurement),
                    Some(0),
                    None,
                ),
                reboot_button: Button {
                    availability: availability.clone(),
                    device: device.clone(),
                    unique_id: hass_types::UniqueId(format!("{}_{}", TOPIC_PREFIX, "reboot")),
                    name: "Reboot software".into(),
                    device_class: Some(hass_types::ButtonDeviceClass::restart),
                    command_topic: Topic(format!("{}/{}", TOPIC_PREFIX, "reboot")),
                    entity_category: Some(hass_types::EntityCategory::diagnostic),
                    ..Default::default()
                },
            }
        })
    }

    pub async fn send_discovery_configs(&self, client: &mut Client<'_, '_>) -> anyhow::Result<()> {
        // TODO: macro this
        client.send_discovery_config(&self.battery_charging).await?;
        client.send_discovery_config(&self.battery_voltage).await?;
        client
            .send_discovery_config(&self.battery_charge_current)
            .await?;
        client.send_discovery_config(&self.ext_voltage).await?;
        client.send_discovery_config(&self.ext_current).await?;
        client.send_discovery_config(&self.int_temperature).await?;
        client.send_discovery_config(&self.satellite_count).await?;
        client.send_discovery_config(&self.reboot_button).await?;
        Ok(())
    }

    pub async fn subscribe_to_actions(&self, client: &mut Client<'_, '_>) -> anyhow::Result<()> {
        client
            .subscribe_to_topic(&self.reboot_button.command_topic.0)
            .await?;
        Ok(())
    }
}
