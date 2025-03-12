use hass_types::{BinarySensor, DeviceTracker};
use std::str::FromStr;

#[derive(serde::Deserialize)]
struct Config {
    powersave_delay_seconds: usize,
    wifi_ssid: String,
    wifi_password: String,
    mqtt_user: String,
    mqtt_password: String,
    mqtt_wifi_ip: String,
    mqtt_modem_host: String,
    mqtt_port: usize,
    apn: String,
    diagnostic_entities_topic_prefix: String,
    device_tracker_config: DeviceTracker,
    ignition_sense_sensor_config: BinarySensor,
    ota_topic: String,
}

impl Config {
    fn check(&self) {
        assert!(!self.wifi_ssid.is_empty(), "wifi_ssid is empty");
        assert!(!self.wifi_password.is_empty(), "wifi_password is empty");
        assert!(!self.mqtt_user.is_empty(), "mqtt_user is empty");
        assert!(!self.mqtt_password.is_empty(), "mqtt_password is empty");
        assert!(!self.mqtt_wifi_ip.is_empty(), "mqtt_wifi_ip is empty");
        assert!(!self.mqtt_modem_host.is_empty(), "mqtt_modem_host is empty");
        assert!(!self.apn.is_empty(), "apn is empty");
        assert!(self.mqtt_port > 0, "mqtt_port is invalid");
        assert!(self.mqtt_port < 65536, "mqtt_port is invalid");
        std::net::Ipv4Addr::from_str(&self.mqtt_wifi_ip).expect("mqtt_wifi_ip is invalid");
        assert!(
            !self.ignition_sense_sensor_config.availability.is_empty(),
            "ignition_sense_sensor_config.availability should contain at least one element"
        );
        assert!(!self.ota_topic.is_empty(), "ota topic is empty");
    }

    fn export_vars(&self) {
        println!(
            "cargo:rustc-env=POWERSAVE_DELAY_SECONDS={}",
            self.powersave_delay_seconds
        );
        println!("cargo:rustc-env=WIFI_SSID={}", self.wifi_ssid);
        println!("cargo:rustc-env=WIFI_PASSWORD={}", self.wifi_password);
        println!("cargo:rustc-env=MQTT_USER={}", self.mqtt_user);
        println!("cargo:rustc-env=MQTT_PASSWORD={}", self.mqtt_password);
        println!("cargo:rustc-env=MQTT_WIFI_IP={}", self.mqtt_wifi_ip);
        println!("cargo:rustc-env=MQTT_MODEM_HOST={}", self.mqtt_modem_host);
        println!("cargo:rustc-env=MQTT_PORT={}", self.mqtt_port);
        println!("cargo:rustc-env=APN={}", self.apn);
        println!(
            "cargo:rustc-env=DIAGNOSTIC_ENTITIES_TOPIC_PREFIX={}",
            self.diagnostic_entities_topic_prefix
        );
        println!("cargo:rustc-env=OTA_TOPIC={}", self.ota_topic);
    }
}

fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlinkall.x");

    println!("cargo:rerun-if-changed=config.yml");
    let config = {
        let config_string = std::fs::read_to_string("config.yml").expect("config.yml not found");
        serde_yaml::from_str::<Config>(&config_string).expect("config.yml is not valid")
    };
    config.check();
    config.export_vars();

    uneval::to_out_dir(config.device_tracker_config, "device_tracker_config.rs")
        .expect("Failed to write device_tracker_config.rs");
    uneval::to_out_dir(
        config.ignition_sense_sensor_config,
        "ignition_sense_sensor_config.rs",
    )
    .expect("Failed to write ignition_sense_sensor_config.rs");
}
