use ha_types::HAEntity;
use serde::Deserialize;

#[derive(Deserialize)]
struct Config {
    mqtt_endpoint: String,
    entities: Vec<HAEntity>,
    availability_topic: String,
    ota_topic: String,
}

impl Config {
    fn verify(&self) -> anyhow::Result<()> {
        if self.mqtt_endpoint.is_empty() {
            anyhow::bail!("mqtt endpoint cannot be empty");
        }
        if !self.mqtt_endpoint.starts_with("mqtt://") {
            anyhow::bail!(
                "mqtt endpoint must start with \"mqtt://\". no other protocols are supported yet."
            );
        }

        for entity in self.entities.iter() {
            if entity.name.is_empty() {
                anyhow::bail!("entity name cannot be empty");
            }
            if entity.unique_id.is_empty() {
                anyhow::bail!("entity unique_id cannot be empty");
            }
            if entity.state_topic.is_empty() {
                anyhow::bail!("entity state_topic cannot be empty");
            }
        }
        Ok(())
    }
}

macro_rules! config_entry_to_env {
    ($config:ident, $env:ident, $entry:ident) => {
        println!("cargo:rustc-env={}={}", stringify!($env), $config.$entry);
    };
}

fn main() {
    embuild::espidf::sysenv::output();

    println!("cargo:rerun-if-changed=config.yml");

    let config_file = std::fs::read_to_string("config.yml").expect("config.yml not found");
    let config: Config = serde_yaml::from_str(&config_file).expect("config.yml is not valid yaml");
    config.verify().expect("config.yml validation failed");

    config_entry_to_env!(config, ESP_MQTT_ENDPOINT, mqtt_endpoint);
    config_entry_to_env!(config, ESP_AVAILABILITY_TOPIC, availability_topic);
    config_entry_to_env!(config, ESP_OTA_TOPIC, ota_topic);

    uneval::to_out_dir(config.entities, "entities.rs").expect("Failed to write entities.rs");
}
