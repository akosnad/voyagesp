use hass_types::DeviceTracker;

#[derive(serde::Deserialize)]
struct Config {
    wifi_ssid: String,
    wifi_password: String,
    apn: String,
    device_tracker_config: DeviceTracker,
}

impl Config {
    fn export_vars(&self) {
        println!("cargo:rustc-env=WIFI_SSID={}", self.wifi_ssid);
        println!("cargo:rustc-env=WIFI_PASSWORD={}", self.wifi_password);
        println!("cargo:rustc-env=APN={}", self.apn);
    }
}

fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlinkall.x");
    println!("cargo:rustc-link-arg-bins=-Trom_functions.x");

    println!("cargo:rerun-if-changed=config.yml");
    let config = {
        let config_string = std::fs::read_to_string("config.yml").expect("config.yml not found");
        serde_yaml::from_str::<Config>(&config_string).expect("config.yml is not valid")
    };
    config.export_vars();

    uneval::to_out_dir(config.device_tracker_config, "device_tracker_config.rs")
        .expect("Failed to write device_tracker_config.rs");
}
