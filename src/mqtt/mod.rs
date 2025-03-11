use super::lib;
use alloc::format;
use anyhow::anyhow;
use core::str::FromStr;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_net::{driver::Driver, tcp::TcpSocket, Ipv4Address, Stack};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, TimeoutError, Timer, WithTimeout};
use esp_hal::rng::Rng;
use hass_types::{DeviceTrackerAttributes, Discoverable, Publishable};
use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::publish_packet::QualityOfService,
};
use serde::Serialize;

use crate::{gps::GpsCoords, PsuData};

mod diagnostic_entities;
use diagnostic_entities::DiagnosticEntities;

const BUF_SIZE: usize = 2048;
const MAX_PROPERTIES: usize = 10;
const EVENT_QUEUE_SIZE: usize = 5;
const SOCKET_TIMEOUT: Duration = Duration::from_secs(35);
const MQTT_KEEPALIVE_INTERVAL: usize = 30;
const OPERATION_TIMEOUT: Duration = Duration::from_secs(15);

type WifiStack =
    &'static embassy_net::Stack<esp_wifi::wifi::WifiDevice<'static, esp_wifi::wifi::WifiStaDevice>>;
type ModemStack = &'static embassy_net::Stack<&'static mut embassy_net_ppp::Device<'static>>;
type ClientInner<'c, 's> = MqttClient<'c, TcpSocket<'s>, MAX_PROPERTIES, Rng>;

#[derive(Debug)]
pub enum PublishEvent {
    DeviceTracker(GpsCoords),
    Ignition(bool),
    Psu(PsuData),
}

impl PublishEvent {
    async fn handle(self, client: &mut Client<'_, '_>) -> anyhow::Result<()> {
        log::debug!("Handling event: {:?}", self);
        match self {
            PublishEvent::DeviceTracker(data) => {
                if data.is_fixed {
                    let attrs: DeviceTrackerAttributes = data.into();
                    client
                        .publish_entity_state(
                            &crate::config::SystemConfig::get().device_tracker,
                            attrs,
                        )
                        .await?;
                }

                client
                    .publish_entity_state(
                        &DiagnosticEntities::get().satellite_count,
                        data.satellite_count,
                    )
                    .await?;
            }
            PublishEvent::Ignition(state) => {
                client
                    .publish_entity_state(
                        &crate::config::SystemConfig::get().ignition_sense_sensor,
                        state,
                    )
                    .await?;
            }
            PublishEvent::Psu(psu_state) => {
                client.handle_psu_event(psu_state).await?;
            }
        }
        Ok(())
    }
}

#[derive(Debug)]
enum RemoteAction {
    Reboot,
}

impl TryFrom<(&str, &[u8])> for RemoteAction {
    type Error = ();
    fn try_from((topic, _payload): (&str, &[u8])) -> Result<Self, ()> {
        const TOPIC_PREFIX: &str = env!("DIAGNOSTIC_ENTITIES_TOPIC_PREFIX");
        let inner_topic = topic
            .strip_prefix(format!("{TOPIC_PREFIX}/").as_str())
            .ok_or(())?;

        match inner_topic {
            "reboot" => Ok(RemoteAction::Reboot),
            _ => Err(()),
        }
    }
}

impl RemoteAction {
    async fn handle(self) -> anyhow::Result<()> {
        log::debug!("Handling remote action: {:?}", self);
        match self {
            RemoteAction::Reboot => {
                esp_hal::reset::software_reset();
                unreachable!();
            }
        }
    }
}

struct Client<'c, 's> {
    inner: ClientInner<'c, 's>,
}
impl<'c, 'sb, 's> Client<'c, 's> {
    fn new(mqtt_client: ClientInner<'c, 's>) -> Self {
        Self { inner: mqtt_client }
    }

    async fn receive_message(
        &mut self,
    ) -> Result<(&str, &[u8]), rust_mqtt::packet::v5::reason_codes::ReasonCode> {
        self.inner.receive_message().await
    }

    async fn send_ping(&mut self) -> Result<(), rust_mqtt::packet::v5::reason_codes::ReasonCode> {
        self.inner.send_ping().await
    }

    async fn init_entities(&mut self, allow_actions: bool) -> anyhow::Result<()> {
        self.send_birth_message().await?;

        let crate::config::SystemConfig {
            ref device_tracker,
            ref ignition_sense_sensor,
        } = &crate::config::SystemConfig::get();

        self.send_discovery_config(device_tracker).await?;
        self.send_discovery_config(ignition_sense_sensor).await?;

        let diagnostic_entities = DiagnosticEntities::get();
        diagnostic_entities.send_discovery_configs(self).await?;
        if allow_actions {
            diagnostic_entities.subscribe_to_actions(self).await?;
        }
        Ok(())
    }

    async fn send_birth_message(&mut self) -> anyhow::Result<()> {
        let crate::config::SystemConfig {
            ref device_tracker,
            ref ignition_sense_sensor,
        } = &crate::config::SystemConfig::get();

        if let Some(availability) = ignition_sense_sensor
            .availability
            .first()
            .or(device_tracker.availability.first())
        {
            let online = availability
                .payload_available
                .as_deref()
                .unwrap_or("online");
            self.inner
                .send_message(
                    availability.topic.as_str(),
                    online.as_bytes(),
                    QualityOfService::QoS0,
                    true,
                )
                .with_timeout(OPERATION_TIMEOUT)
                .await
                .map_err(|_| anyhow!("Failed to send MQTT message: timeout"))?
                .map_err(|e| anyhow!("Failed to send MQTT message: {e:?}"))?;
        }
        Ok(())
    }

    async fn send_discovery_config(
        &mut self,
        entity: &(impl Discoverable + Serialize),
    ) -> anyhow::Result<()> {
        let config = {
            let value = serde_json::to_value(entity)
                .map_err(|e| anyhow!("Failed to serialize entity: {e:?}"))?;
            serde_json::to_string(&lib::SkipNulls(&value))
                .map_err(|e| anyhow!("Failed to serialize entity: {e:?}"))?
        };

        let discovery_topic = entity.discovery_topic();
        let discovery_topic = discovery_topic.0.as_str();

        self.inner
            .send_message(
                discovery_topic,
                config.as_bytes(),
                QualityOfService::QoS0,
                true,
            )
            .with_timeout(OPERATION_TIMEOUT)
            .await
            .map_err(|_| anyhow!("Failed to send MQTT message: timeout"))?
            .map_err(|e| anyhow!("Failed to send MQTT message: {e:?}"))?;

        Ok(())
    }

    async fn subscribe_to_topic(&mut self, topic: &str) -> anyhow::Result<()> {
        self.inner
            .subscribe_to_topic(topic)
            .await
            .map_err(|e| anyhow!("Failed to subscribe to topic {topic}: {e:?}"))
    }

    async fn handle_psu_event(&mut self, psu_state: PsuData) -> anyhow::Result<()> {
        let PsuData {
            ref battery_charging,
            ref battery_voltage,
            ref battery_charge_current,
            ref ext_voltage,
            ref ext_current,
            ref temperature,
        } = psu_state;
        let entities = DiagnosticEntities::get();

        self.publish_entity_state(&entities.battery_charging, battery_charging)
            .await?;
        self.publish_entity_state(&entities.battery_voltage, battery_voltage)
            .await?;
        self.publish_entity_state(&entities.battery_charge_current, battery_charge_current)
            .await?;
        self.publish_entity_state(&entities.ext_voltage, ext_voltage)
            .await?;
        self.publish_entity_state(&entities.ext_current, ext_current)
            .await?;
        self.publish_entity_state(&entities.int_temperature, temperature)
            .await?;

        Ok(())
    }

    async fn publish_entity_state(
        &mut self,
        entity: &impl Publishable,
        state: impl Serialize,
    ) -> anyhow::Result<()> {
        let state_value = serde_json::to_value(state)
            .map_err(|e| anyhow!("Failed to serialize event data: {e:?}"))?;
        match serde_json::to_string(&lib::SkipNulls(&state_value)) {
            Ok(state_str) => self
                .inner
                .send_message(
                    entity.state_topic().0.as_str(),
                    state_str.as_bytes(),
                    QualityOfService::QoS0,
                    entity.retain(),
                )
                .with_timeout(OPERATION_TIMEOUT)
                .await
                .map_err(|_| anyhow!("Failed to send MQTT message: timeout"))?
                .map_err(|e| anyhow!("Failed to send MQTT message: {e:?}")),
            Err(e) => anyhow::bail!("Failed to serialize event data: {e:?}"),
        }
    }
}

type Endpoint = (Ipv4Address, u16);

#[derive(Debug)]
enum ConnectionMedium {
    Wifi,
    Modem,
}

struct Connection<'s> {
    socket: TcpSocket<'s>,
    endpoint: Endpoint,
    medium: ConnectionMedium,
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
        if let Some(availability) = system_config
            .ignition_sense_sensor
            .availability
            .first()
            .or(system_config.device_tracker.availability.first())
        {
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

        match select(
            self.wifi_stack.wait_config_up(),
            self.modem_stack.wait_config_up(),
        )
        .await
        {
            // WiFi
            Either::First(_) => {
                let ip = Ipv4Address::from_str(env!("MQTT_WIFI_IP"))
                    .expect("invalid IP address for MQTT WiFi endpoint");

                log::debug!("MQTT: Using endpoint: {ip}:{port}");

                Ok(Connection {
                    socket: TcpSocket::new(self.wifi_stack, rx, tx),
                    endpoint: (ip, port),
                    medium: ConnectionMedium::Wifi,
                })
            }
            // Modem
            Either::Second(_) => {
                let ip = match get_host_ip(env!("MQTT_MODEM_HOST"), self.modem_stack).await {
                    Ok(ip) => ip,
                    Err(e) => {
                        anyhow::bail!("Failed to get IP address for MQTT modem endpoint: {e:?}")
                    }
                };
                log::debug!("MQTT: Using endpoint: {ip}:{port}");

                Ok(Connection {
                    socket: TcpSocket::new(self.modem_stack, rx, tx),
                    endpoint: (ip, port),
                    medium: ConnectionMedium::Modem,
                })
            }
        }
    }

    async fn setup<'c, 's>(
        &self,
        tcp_rx: &'s mut [u8],
        tcp_tx: &'s mut [u8],
        mqtt_rx: &'c mut [u8],
        mqtt_tx: &'c mut [u8],
    ) -> anyhow::Result<(ClientInner<'c, 's>, ConnectionMedium)> {
        let Connection {
            mut socket,
            endpoint,
            medium,
        } = match self.setup_tcp_socket(tcp_rx, tcp_tx).await {
            Ok(res) => res,
            Err(e) => {
                anyhow::bail!("Failed to set up socket: {:?}", e);
            }
        };
        socket.set_timeout(Some(SOCKET_TIMEOUT));

        match socket
            .connect(endpoint)
            .with_timeout(OPERATION_TIMEOUT)
            .await
        {
            Ok(Ok(_)) => {}
            Ok(Err(e)) => {
                anyhow::bail!("Failed to connect to TCP endpoint: {:?}", e);
            }
            Err(TimeoutError) => {
                anyhow::bail!("Failed to connect to TCP endpoint: timeout");
            }
        }
        log::info!("MQTT: socket connected over {medium:?}");

        let mut client = MqttClient::<_, MAX_PROPERTIES, _>::new(
            socket,
            mqtt_tx,
            BUF_SIZE,
            mqtt_rx,
            BUF_SIZE,
            self.config.clone(),
        );

        match client
            .connect_to_broker()
            .with_timeout(OPERATION_TIMEOUT)
            .await
        {
            Ok(Ok(_)) => {}
            Err(TimeoutError) => {
                anyhow::bail!("Failed to connect to broker: timeout");
            }
            Ok(Err(e)) => {
                anyhow::bail!("Failed to connect to broker: {:?}", e);
            }
        }
        log::info!("MQTT: broker connected");

        Ok((client, medium))
    }

    /// MQTT stack task
    ///
    /// Should be only called once
    pub async fn run(&self) -> ! {
        'socket_retry: loop {
            log::debug!("MQTT: Starting stack...");

            let mut mqtt_rx = [0u8; 128];
            let mut mqtt_tx = [0u8; 128];
            let mut client_tx = [0u8; BUF_SIZE];
            let mut client_rx = [0u8; BUF_SIZE];

            let result = self
                .setup(&mut mqtt_rx, &mut mqtt_tx, &mut client_rx, &mut client_tx)
                .await;
            match result {
                Ok((client, medium)) => {
                    let mut client = Client::new(client);
                    if let Err(e) = self.eventloop(&mut client, medium).await {
                        log::error!("MQTT: event loop terminated: {:?}", e);
                    }
                }
                Err(e) => {
                    log::error!("MQTT: Failed to set up stack: {:?}", e);
                    Timer::after_secs(5).await;
                    continue 'socket_retry;
                }
            }
        }
    }

    async fn eventloop(
        &self,
        client: &mut Client<'_, '_>,
        connection_medium: ConnectionMedium,
    ) -> anyhow::Result<()> {
        let allow_actions = matches!(connection_medium, ConnectionMedium::Wifi);
        client.init_entities(allow_actions).await?;
        log::info!("MQTT: entities initialized");

        loop {
            let event = self.pub_queue.ready_to_receive();
            let receive = client.receive_message();
            let timeout = Timer::after(Duration::from_secs(5));
            match select3(event, receive, timeout).await {
                Either3::First(_) => {
                    let event = self.pub_queue.receive().await;
                    event
                        .handle(client)
                        .await
                        .map_err(|e| anyhow!("Failed to handle event: {e:?}"))?;
                }
                Either3::Second(Ok((topic, payload))) => {
                    log::info!("MQTT: Received message on topic: {}", topic);
                    if let Ok(action) = RemoteAction::try_from((topic, payload)) {
                        action
                            .handle()
                            .await
                            .map_err(|e| anyhow!("Failed to handle remote event: {e:?}"))?;
                    }
                }
                Either3::Second(Err(e)) => {
                    anyhow::bail!("Failed to receive message: {:?}", e);
                }
                Either3::Third(_) => {}
            }

            match client.send_ping().with_timeout(OPERATION_TIMEOUT).await {
                Ok(Ok(_)) => {}
                Ok(Err(e)) => {
                    anyhow::bail!("Failed to send ping: {:?}", e);
                }
                Err(TimeoutError) => {
                    anyhow::bail!("Failed to send ping: timeout");
                }
            }

            if self.wifi_stack.is_link_up()
                && self.wifi_stack.is_config_up()
                && !matches!(connection_medium, ConnectionMedium::Wifi)
            {
                log::info!("MQTT: WiFi seems to be up, reconnecting...");
                break Ok(());
            }
        }
    }

    pub async fn publish(&self, event: PublishEvent) {
        self.pub_queue.send(event).await;
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
