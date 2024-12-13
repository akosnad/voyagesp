use alloc::borrow::ToOwned;
use alloc::format;
use anyhow::anyhow;
use core::str::FromStr;
use embassy_futures::select::{select3, Either3};
use embassy_net::{driver::Driver, tcp::TcpSocket, Ipv4Address, Stack};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, once_lock::OnceLock,
};
use embassy_time::{Duration, TimeoutError, Timer, WithTimeout};
use esp_hal::rng::Rng;
use hass_types::{BinarySensor, DeviceTrackerAttributes, Discoverable, Publishable, Sensor, Topic};
use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::publish_packet::QualityOfService,
};
use serde::Serialize;

use crate::{gps::GpsCoords, PsuData};

const BUF_SIZE: usize = 2048;
const MAX_PROPERTIES: usize = 10;
const EVENT_QUEUE_SIZE: usize = 5;
const SOCKET_TIMEOUT: Duration = Duration::from_secs(35);
const MQTT_KEEPALIVE_INTERVAL: usize = 30;

type WifiStack =
    &'static embassy_net::Stack<esp_wifi::wifi::WifiDevice<'static, esp_wifi::wifi::WifiStaDevice>>;
type ModemStack = &'static embassy_net::Stack<&'static mut embassy_net_ppp::Device<'static>>;

#[derive(Debug)]
pub enum PublishEvent {
    DeviceTracker(GpsCoords),
    Ignition(bool),
    Psu(PsuData),
}

type Endpoint = (Ipv4Address, u16);

struct Connection<'s> {
    socket: TcpSocket<'s>,
    endpoint: Endpoint,
    is_wifi: bool,
}

static DIAGNOSTIC_ENTITIES: OnceLock<DiagnosticEntities> = OnceLock::new();
struct DiagnosticEntities {
    battery_charging: BinarySensor,
    battery_voltage: Sensor,
    battery_charge_current: Sensor,
    ext_voltage: Sensor,
    ext_current: Sensor,
    int_temperature: Sensor,
    satellite_count: Sensor,
}
impl DiagnosticEntities {
    pub fn get() -> &'static Self {
        DIAGNOSTIC_ENTITIES.get_or_init(|| {
            const TOPIC_PREFIX: &str = env!("DIAGNOSTIC_ENTITIES_TOPIC_PREFIX");
            let config = &crate::config::SystemConfig::get();
            let availability = config.device_tracker.availability.to_owned();
            let device = config.device_tracker.device.clone();

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
            }
        })
    }
}

pub struct Mqtt {
    wifi_stack: WifiStack,
    modem_stack: ModemStack,
    config: ClientConfig<'static, MAX_PROPERTIES, esp_hal::rng::Rng>,
    pub_queue: Channel<CriticalSectionRawMutex, PublishEvent, EVENT_QUEUE_SIZE>,
}

impl Mqtt {
    pub fn new(wifi_stack: WifiStack, modem_stack: ModemStack, rng: esp_hal::rng::Rng) -> Self {
        let system_config = crate::config::SystemConfig::get();

        let mut config =
            ClientConfig::new(rust_mqtt::client::client_config::MqttVersion::MQTTv5, rng);
        config.add_client_id(&system_config.device_tracker.unique_id.0);
        config.add_username(env!("MQTT_USER"));
        config.add_password(env!("MQTT_PASSWORD"));
        if let Some(availability) = system_config.device_tracker.availability.first() {
            let payload_offline = availability
                .payload_not_available
                .as_deref()
                .unwrap_or("offline");
            config.add_will(&availability.topic, payload_offline.as_bytes(), true);
        }
        config.keep_alive = MQTT_KEEPALIVE_INTERVAL as u16;
        config.max_packet_size = (BUF_SIZE as u32) - 1;

        Self {
            wifi_stack,
            modem_stack,
            config,
            pub_queue: Channel::new(),
        }
    }

    async fn setup_tcp_socket<'a>(
        &self,
        rx: &'a mut [u8],
        tx: &'a mut [u8],
    ) -> anyhow::Result<Connection<'a>> {
        let port = u16::from_str(env!("MQTT_PORT")).expect("invalid MQTT port");

        if self.wifi_stack.is_link_up() && self.wifi_stack.is_config_up() {
            let ip = Ipv4Address::from_str(env!("MQTT_WIFI_IP"))
                .expect("invalid IP address for MQTT WiFi endpoint");

            log::debug!("Using MQTT endpoint: {ip}:{port}");

            Ok(Connection {
                socket: TcpSocket::new(self.wifi_stack, rx, tx),
                endpoint: (ip, port),
                is_wifi: true,
            })
        } else if self.modem_stack.is_link_up() && self.modem_stack.is_config_up() {
            let ip = match get_host_ip(env!("MQTT_MODEM_HOST"), self.modem_stack).await {
                Ok(ip) => ip,
                Err(e) => anyhow::bail!("Failed to get IP address for MQTT modem endpoint: {e:?}"),
            };
            log::debug!("Using MQTT endpoint: {ip}:{port}");

            Ok(Connection {
                socket: TcpSocket::new(self.modem_stack, rx, tx),
                endpoint: (ip, port),
                is_wifi: false,
            })
        } else {
            anyhow::bail!("No network connection available");
        }
    }

    /// MQTT stack task
    ///
    /// Should be only called once
    pub async fn run(&self) -> ! {
        'socket_retry: loop {
            Timer::after(Duration::from_secs(5)).await;

            let mut mqtt_rx = [0u8; 128];
            let mut mqtt_tx = [0u8; 128];
            let Connection {
                mut socket,
                endpoint,
                is_wifi: connection_is_wifi,
            } = match self.setup_tcp_socket(&mut mqtt_rx, &mut mqtt_tx).await {
                Ok(res) => res,
                Err(e) => {
                    log::error!("Failed to setup MQTT socket: {:?}", e);
                    continue 'socket_retry;
                }
            };
            socket.set_timeout(Some(SOCKET_TIMEOUT));

            if let Err(e) = socket.connect(endpoint).await {
                log::error!("Failed to connect to MQTT broker: {:?}", e);
                continue 'socket_retry;
            }
            log::info!("MQTT socket connected");

            let mut client_tx = [0u8; BUF_SIZE];
            let mut client_rx = [0u8; BUF_SIZE];
            let mut client = MqttClient::<_, MAX_PROPERTIES, _>::new(
                &mut socket,
                &mut client_tx,
                BUF_SIZE,
                &mut client_rx,
                BUF_SIZE,
                self.config.clone(),
            );

            if let Err(e) = client.connect_to_broker().await {
                log::error!("Failed to connect to MQTT broker: {:?}", e);
                continue 'socket_retry;
            }
            log::info!("MQTT broker connected");

            if let Err(e) = self.init_entities(&mut client).await {
                log::error!("Failed to init MQTT entities: {:?}", e);
                continue 'socket_retry;
            }
            log::info!("MQTT entities initialized");

            'event_loop: loop {
                let event = self.pub_queue.ready_to_receive();
                let receive = client.receive_message();
                let timeout = Timer::after(Duration::from_secs(5));
                match select3(event, receive, timeout).await {
                    Either3::First(_) => {
                        let event = self.pub_queue.receive().await;
                        match self
                            .handle_publish_event(&mut client, event)
                            .with_timeout(Duration::from_secs(30))
                            .await
                        {
                            Ok(Ok(_)) => {}
                            Ok(Err(e)) => {
                                log::error!("Failed to handle event: {:?}", e);
                                break 'event_loop;
                            }
                            Err(TimeoutError) => {
                                log::error!("Handling event timed out");
                                break 'event_loop;
                            }
                        }
                    }
                    Either3::Second(Ok((topic, _payload))) => {
                        log::info!("Received message on topic: {}", topic);
                    }
                    Either3::Second(Err(e)) => {
                        log::error!("Failed to receive MQTT message: {:?}", e);
                        break 'event_loop;
                    }
                    Either3::Third(_) => {}
                }
                match client.send_ping().await {
                    Ok(_) => {}
                    Err(e) => {
                        log::error!("Failed to send MQTT ping: {:?}", e);
                        break 'event_loop;
                    }
                }
                if self.wifi_stack.is_link_up()
                    && self.wifi_stack.is_config_up()
                    && !connection_is_wifi
                {
                    log::info!("WiFi seems to be up, switching MQTT connection from modem to WiFi");
                    break 'event_loop;
                }
            }
        }
    }

    async fn init_entities(
        &self,
        client: &mut MqttClient<'_, &mut TcpSocket<'_>, MAX_PROPERTIES, Rng>,
    ) -> anyhow::Result<()> {
        let crate::config::SystemConfig {
            ref device_tracker,
            ref ignition_sense_sensor,
        } = &crate::config::SystemConfig::get();

        // send birth message
        if let Some(device_tracker_availability) = device_tracker.availability.first() {
            let online = device_tracker_availability
                .payload_available
                .as_deref()
                .unwrap_or("online");
            log::debug!(
                "Sending online ({}) message for {}",
                online,
                device_tracker_availability.topic
            );
            client
                .send_message(
                    device_tracker_availability.topic.as_str(),
                    online.as_bytes(),
                    QualityOfService::QoS0,
                    true,
                )
                .await
                .map_err(|e| anyhow!("Failed to send MQTT message: {e:?}"))?;
        }

        self.send_discovery_config(client, device_tracker).await?;
        self.send_discovery_config(client, ignition_sense_sensor)
            .await?;

        let DiagnosticEntities {
            ref battery_charging,
            ref battery_voltage,
            ref battery_charge_current,
            ref ext_voltage,
            ref ext_current,
            ref int_temperature,
            ref satellite_count,
        } = DiagnosticEntities::get();

        self.send_discovery_config(client, battery_charging).await?;
        self.send_discovery_config(client, battery_voltage).await?;
        self.send_discovery_config(client, battery_charge_current)
            .await?;
        self.send_discovery_config(client, ext_voltage).await?;
        self.send_discovery_config(client, ext_current).await?;
        self.send_discovery_config(client, int_temperature).await?;
        self.send_discovery_config(client, satellite_count).await?;

        Ok(())
    }

    async fn send_discovery_config(
        &self,
        client: &mut MqttClient<'_, &mut TcpSocket<'_>, MAX_PROPERTIES, Rng>,
        entity: &(impl Discoverable + Serialize),
    ) -> anyhow::Result<()> {
        let config = {
            let value = serde_json::to_value(entity)
                .map_err(|e| anyhow!("Failed to serialize entity: {e:?}"))?;
            serde_json::to_string(&SkipNulls(&value))
                .map_err(|e| anyhow!("Failed to serialize entity: {e:?}"))?
        };

        let discovery_topic = entity.discovery_topic();
        let discovery_topic = discovery_topic.0.as_str();

        log::debug!(
            "Sending entity discovery config for {}: {}",
            discovery_topic,
            config
        );

        client
            .send_message(
                discovery_topic,
                config.as_bytes(),
                QualityOfService::QoS0,
                true,
            )
            .await
            .map_err(|e| anyhow!("Failed to send MQTT message: {e:?}"))?;

        Ok(())
    }

    async fn handle_publish_event(
        &self,
        client: &mut MqttClient<'_, &mut TcpSocket<'_>, MAX_PROPERTIES, Rng>,
        event: PublishEvent,
    ) -> anyhow::Result<()> {
        log::debug!("Handling event: {:?}", event);
        match event {
            PublishEvent::DeviceTracker(data) => {
                if data.is_fixed {
                    let attrs: DeviceTrackerAttributes = data.into();
                    self.publish_entity_state(
                        client,
                        &crate::config::SystemConfig::get().device_tracker,
                        attrs,
                    )
                    .await?;
                }

                self.publish_entity_state(
                    client,
                    &DiagnosticEntities::get().satellite_count,
                    data.satellite_count,
                )
                .await?;
            }
            PublishEvent::Ignition(state) => {
                self.publish_entity_state(
                    client,
                    &crate::config::SystemConfig::get().ignition_sense_sensor,
                    state,
                )
                .await?;
            }
            PublishEvent::Psu(psu_state) => {
                self.publish_psu_state(client, psu_state).await?;
            }
        }
        Ok(())
    }

    pub async fn publish(&self, event: PublishEvent) {
        self.pub_queue.send(event).await;
    }

    async fn publish_psu_state(
        &self,
        client: &mut MqttClient<'_, &mut TcpSocket<'_>, MAX_PROPERTIES, Rng>,
        psu_state: PsuData,
    ) -> anyhow::Result<()> {
        let PsuData {
            ref battery_charging,
            ref battery_voltage,
            ref battery_charge_current,
            ref ext_voltage,
            ref ext_current,
            ref temperature,
        } = psu_state;
        let entities = DiagnosticEntities::get();

        self.publish_entity_state(client, &entities.battery_charging, battery_charging)
            .await?;
        self.publish_entity_state(client, &entities.battery_voltage, battery_voltage)
            .await?;
        self.publish_entity_state(
            client,
            &entities.battery_charge_current,
            battery_charge_current,
        )
        .await?;
        self.publish_entity_state(client, &entities.ext_voltage, ext_voltage)
            .await?;
        self.publish_entity_state(client, &entities.ext_current, ext_current)
            .await?;
        self.publish_entity_state(client, &entities.int_temperature, temperature)
            .await?;

        Ok(())
    }

    async fn publish_entity_state(
        &self,
        client: &mut MqttClient<'_, &mut TcpSocket<'_>, MAX_PROPERTIES, Rng>,
        entity: &impl Publishable,
        state: impl Serialize,
    ) -> anyhow::Result<()> {
        let state_value = serde_json::to_value(state)
            .map_err(|e| anyhow!("Failed to serialize event data: {e:?}"))?;
        match serde_json::to_string(&SkipNulls(&state_value)) {
            Ok(state_str) => client
                .send_message(
                    entity.state_topic().0.as_str(),
                    state_str.as_bytes(),
                    QualityOfService::QoS0,
                    false,
                )
                .await
                .map_err(|e| anyhow!("Failed to send MQTT message: {e:?}")),
            Err(e) => anyhow::bail!("Failed to serialize event data: {e:?}"),
        }
    }
}

#[derive(Debug)]
struct SkipNulls<'v>(&'v serde_json::Value);
impl<'v> Serialize for SkipNulls<'v> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer + ?Sized,
    {
        use serde::ser::{SerializeMap, SerializeSeq};

        match &self.0 {
            serde_json::Value::Object(map) => {
                let map = map.iter().filter(|(_, v)| !v.is_null());
                let mut ser = serializer.serialize_map(None)?;
                for (k, v) in map {
                    ser.serialize_entry(k, &SkipNulls(v))?;
                }
                ser.end()
            }
            serde_json::Value::Array(arr) => {
                let arr = arr.iter().filter(|v| !v.is_null());
                let mut ser = serializer.serialize_seq(None)?;
                for v in arr {
                    ser.serialize_element(&SkipNulls(v))?;
                }
                ser.end()
            }
            _ => self.0.serialize(serializer),
        }
    }
}

async fn get_host_ip<D: Driver>(host: &str, stack: &Stack<D>) -> anyhow::Result<Ipv4Address> {
    stack
        .dns_query(host, smoltcp::wire::DnsQueryType::A)
        .await
        .map_err(|e| anyhow!("Failed to query DNS for {}: {:?}", host, e))
        .and_then(|res| {
            res.first()
                .map(|ip| Ok(Ipv4Address::from_bytes(ip.as_bytes())))
                .unwrap_or_else(|| Err(anyhow!("No IP address found for {}", host)))
        })
}
