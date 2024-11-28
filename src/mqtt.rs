use core::str::FromStr;

use anyhow::anyhow;
use embassy_futures::select::{select3, Either3};
use embassy_net::{tcp::TcpSocket, Ipv4Address};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use esp_hal::rng::Rng;
use hass_types::{DeviceTrackerAttributes, Discoverable};
use rust_mqtt::{
    client::{client::MqttClient, client_config::ClientConfig},
    packet::v5::publish_packet::QualityOfService,
};
use serde::Serialize;

const BUF_SIZE: usize = 2048;
const MAX_PROPERTIES: usize = 10;
const EVENT_QUEUE_SIZE: usize = 5;

type WifiStack =
    &'static embassy_net::Stack<esp_wifi::wifi::WifiDevice<'static, esp_wifi::wifi::WifiStaDevice>>;
type ModemStack = &'static embassy_net::Stack<&'static mut embassy_net_ppp::Device<'static>>;

#[derive(Debug)]
pub enum Event {
    DeviceTrackerStateChange(DeviceTrackerAttributes),
}

pub struct Mqtt {
    wifi_stack: WifiStack,
    modem_stack: ModemStack,
    config: ClientConfig<'static, MAX_PROPERTIES, esp_hal::rng::Rng>,
    event_queue: Channel<CriticalSectionRawMutex, Event, EVENT_QUEUE_SIZE>,
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
        config.keep_alive = 15;
        config.max_packet_size = (BUF_SIZE as u32) - 1;

        Self {
            wifi_stack,
            modem_stack,
            config,
            event_queue: Channel::new(),
        }
    }

    /// MQTT stack task
    ///
    /// Should be only called once
    pub async fn run(&self) -> ! {
        let endpoint = (
            Ipv4Address::from_str(env!("MQTT_WIFI_IP"))
                .expect("invalid IP address for MQTT WiFi endpoint"),
            u16::from_str(env!("MQTT_PORT")).expect("invalid MQTT port"),
        );
        'socket_retry: loop {
            Timer::after(Duration::from_secs(5)).await;

            let mut mqtt_rx = [0u8; 128];
            let mut mqtt_tx = [0u8; 128];
            let mut mqtt_sock = TcpSocket::new(self.wifi_stack, &mut mqtt_rx, &mut mqtt_tx);
            mqtt_sock.set_timeout(Some(Duration::from_secs(10)));

            if let Err(e) = mqtt_sock.connect(endpoint).await {
                log::error!("Failed to connect to MQTT broker: {:?}", e);
                continue 'socket_retry;
            }
            log::info!("MQTT socket connected");

            let mut client_tx = [0u8; BUF_SIZE];
            let mut client_rx = [0u8; BUF_SIZE];
            let mut client = MqttClient::<_, MAX_PROPERTIES, _>::new(
                &mut mqtt_sock,
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
                let event = self.event_queue.ready_to_receive();
                let receive = client.receive_message();
                let timeout = Timer::after(Duration::from_secs(5));
                match select3(event, receive, timeout).await {
                    Either3::First(_) => {
                        let event = self.event_queue.receive().await;
                        match self.handle_event(&mut client, event).await {
                            Ok(_) => {}
                            Err(e) => {
                                log::error!("Failed to handle event: {:?}", e);
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
                    Either3::Third(_) => match client.send_ping().await {
                        Ok(_) => {}
                        Err(e) => {
                            log::error!("Failed to send MQTT ping: {:?}", e);
                            break 'event_loop;
                        }
                    },
                }
            }
        }
    }

    async fn init_entities(
        &self,
        client: &mut MqttClient<'_, &mut TcpSocket<'_>, MAX_PROPERTIES, Rng>,
    ) -> anyhow::Result<()> {
        let device_tracker = &crate::config::SystemConfig::get().device_tracker;

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
                    QualityOfService::QoS1,
                    true,
                )
                .await
                .map_err(|e| anyhow!("Failed to send MQTT message: {e:?}"))?;
        }

        let device_tracker_value = serde_json::to_value(device_tracker)
            .map_err(|e| anyhow!("Failed to serialize device tracker: {e:?}"))?;
        let device_tracker_str = serde_json::to_string(&SkipNulls(device_tracker_value))
            .map_err(|e| anyhow!("Failed to serialize device tracker: {e:?}"))?;
        log::debug!(
            "Sending device tracker discovery config: {}",
            device_tracker_str
        );
        client
            .send_message(
                device_tracker.discovery_topic().0.as_str(),
                device_tracker_str.as_bytes(),
                QualityOfService::QoS1,
                true,
            )
            .await
            .map_err(|e| anyhow!("Failed to send MQTT message: {e:?}"))?;

        Ok(())
    }

    async fn handle_event(
        &self,
        client: &mut MqttClient<'_, &mut TcpSocket<'_>, MAX_PROPERTIES, Rng>,
        event: Event,
    ) -> anyhow::Result<()> {
        log::debug!("Handling event: {:?}", event);
        match event {
            Event::DeviceTrackerStateChange(data) => {
                let data_value = serde_json::to_value(data)
                    .map_err(|e| anyhow!("Failed to serialize event data: {e:?}"))?;
                match serde_json::to_string(&SkipNulls(data_value)) {
                    Ok(data_str) => {
                        let device_tracker = &crate::config::SystemConfig::get().device_tracker;
                        client
                            .send_message(
                                device_tracker.json_attributes_topic.0.as_str(),
                                data_str.as_bytes(),
                                QualityOfService::QoS2,
                                true,
                            )
                            .await
                            .map_err(|e| anyhow!("Failed to send MQTT message: {e:?}"))?;
                    }
                    Err(e) => anyhow::bail!("Failed to serialize event data: {e:?}"),
                }
            }
        }
        Ok(())
    }

    pub async fn send_event(&self, event: Event) {
        self.event_queue.send(event).await;
    }
}

#[derive(Debug)]
struct SkipNulls(serde_json::Value);
impl Serialize for SkipNulls {
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
                    ser.serialize_entry(k, &SkipNulls(v.clone()))?;
                }
                ser.end()
            }
            serde_json::Value::Array(arr) => {
                let arr = arr.iter().filter(|v| !v.is_null());
                let mut ser = serializer.serialize_seq(None)?;
                for v in arr {
                    ser.serialize_element(&SkipNulls(v.clone()))?;
                }
                ser.end()
            }
            _ => self.0.serialize(serializer),
        }
    }
}
