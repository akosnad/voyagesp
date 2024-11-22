#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]

use alloc::boxed::Box;
use core::str::FromStr;
use embassy_executor::{task, Spawner};
use embassy_net::{tcp::TcpSocket, ConfigV4, Ipv4Address, StackResources, StaticConfigV4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, Io, Output},
    prelude::*,
    rng::Rng,
    uart,
};
use esp_hal_embassy::main;
use esp_wifi::wifi::{
    utils::create_network_interface, ClientConfiguration, Configuration, WifiController,
    WifiDevice, WifiEvent, WifiStaDevice, WifiState,
};
use log::info;
use rust_mqtt::{client::client::MqttClient, utils::rng_generator::CountingRng};
use static_cell::make_static;

extern crate alloc;

mod gps;
mod modem;

use gps::Gps;

#[export_name = "custom_halt"]
pub fn custom_halt() -> ! {
    loop {
        esp_hal::reset::software_reset();
    }
}

const GPS_BAUD: u32 = 9600;

#[main]
async fn main_task(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger(log::LevelFilter::Debug);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // PSU
    let psu_i2c =
        esp_hal::i2c::I2c::new(peripherals.I2C0, io.pins.gpio21, io.pins.gpio22, 100.kHz());
    let mut psu = axp192::Axp192::new(psu_i2c);

    psu.set_dcdc1_on(false).unwrap();
    psu.set_ldo2_on(false).unwrap();
    psu.set_ldo3_on(false).unwrap();
    psu.set_dcdc2_on(false).unwrap();
    psu.set_exten_on(false).unwrap();

    // WIFI
    let mut rng = Rng::new(peripherals.RNG);
    let wifi_init = esp_wifi::init(
        esp_wifi::EspWifiInitFor::Wifi,
        timg0.timer1,
        rng,
        peripherals.RADIO_CLK,
    )
    .expect("Failed to initialize WiFi clocks");
    let mut socket_storage: [smoltcp::iface::SocketStorage; 3] = Default::default();
    let (_wifi_interface, wifi_device, wifi_controller, _wifi_sockets) = create_network_interface(
        &wifi_init,
        peripherals.WIFI,
        WifiStaDevice,
        &mut socket_storage,
    )
    .expect("Failed to create WiFi network interface");

    let stack_resources: &mut StackResources<3> = Box::leak(Box::new(StackResources::new()));
    let wifi_stack = make_static!(embassy_net::Stack::new(
        wifi_device,
        embassy_net::Config::dhcpv4(Default::default()),
        stack_resources,
        1234u64,
    ));

    spawner
        .spawn(wifi_connection(wifi_controller))
        .expect("Failed to spawn WiFi connection task");
    spawner
        .spawn(wifi_net_task(wifi_stack))
        .expect("Failed to spawn WiFi task");

    // GPS
    let gps_uart = esp_hal::uart::Uart::new_async_with_config(
        peripherals.UART1,
        uart::config::Config {
            baudrate: GPS_BAUD,
            data_bits: uart::config::DataBits::DataBits8,
            parity: uart::config::Parity::ParityNone,
            stop_bits: uart::config::StopBits::STOP1,
            rx_timeout: Some(50),
            ..Default::default()
        },
        io.pins.gpio13,
        io.pins.gpio15,
    )
    .expect("Failed to initialize GPS UART");

    let gps = {
        let gps = gps::Gps::new(gps_uart)
            .await
            .expect("Failed to initialize GPS");
        make_static!(gps)
    };
    spawner
        .spawn(gps_task(gps))
        .expect("Failed to spawn GPS task");

    // MODEM
    let modem_dtr = Output::new(io.pins.gpio32, esp_hal::gpio::Level::Low);
    let modem_ri = Input::new(io.pins.gpio33, esp_hal::gpio::Pull::None);

    let modem_pwrkey = Output::new(io.pins.gpio4, esp_hal::gpio::Level::Low);
    let modem_power_on = Output::new(io.pins.gpio25, esp_hal::gpio::Level::Low);

    const MODEM_BAUD: u32 = 115_200;
    let modem_uart = esp_hal::uart::Uart::new_async_with_config(
        peripherals.UART2,
        uart::config::Config {
            baudrate: MODEM_BAUD,
            data_bits: uart::config::DataBits::DataBits8,
            parity: uart::config::Parity::ParityNone,
            stop_bits: uart::config::StopBits::STOP1,
            rx_timeout: Some(50),
            ..Default::default()
        },
        io.pins.gpio26,
        io.pins.gpio27,
    )
    .expect("Failed to initialize modem UART");

    let config_chan = {
        let chan: embassy_sync::channel::Channel<CriticalSectionRawMutex, StaticConfigV4, 1> =
            embassy_sync::channel::Channel::new();
        let boxed = Box::new(chan);
        Box::leak(boxed)
    };
    let (modem, modem_ppp) = {
        let modem = modem::Modem::new(
            &spawner,
            modem_uart,
            modem_dtr,
            modem_ri,
            modem_pwrkey,
            modem_power_on,
            config_chan.sender(),
        )
        .await
        .expect("Failed to initialize modem");
        make_static!(modem)
    };
    spawner
        .spawn(modem_task(modem))
        .expect("Failed to spawn modem task");

    let stack_resources = {
        let resources: StackResources<3> = StackResources::new();
        let boxed = Box::new(resources);
        Box::leak(boxed)
    };
    let seed = {
        let mut buf = [0u8; 8];
        rng.read(&mut buf);
        u64::from_le_bytes(buf)
    };

    let dummy_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: embassy_net::Ipv4Cidr::new(Ipv4Address::new(0, 0, 0, 0), 0),
        gateway: Some(Ipv4Address::new(0, 0, 0, 0)),
        dns_servers: heapless::Vec::new(),
    });
    let modem_stack = {
        let stack = embassy_net::Stack::new(modem_ppp, dummy_config, stack_resources, seed);
        let boxed = Box::new(stack);
        Box::leak(boxed)
    };

    spawner
        .spawn(modem_net(modem_stack))
        .expect("Failed to spawn modem network task");
    spawner
        .spawn(modem_stack_config_setter(
            modem_stack,
            config_chan.receiver(),
        ))
        .expect("Failed to spawn modem stack config setter");

    // MQTT
    let mut mqtt_rx = [0u8; 128];
    let mut mqtt_tx = [0u8; 128];
    let mut mqtt_sock = TcpSocket::new(wifi_stack, &mut mqtt_rx, &mut mqtt_tx);
    mqtt_sock.set_timeout(Some(Duration::from_secs(10)));
    let endpoint = (Ipv4Address::new(10, 20, 0, 1), 1883);
    loop {
        Timer::after(Duration::from_secs(5)).await;
        if let Err(e) = mqtt_sock.connect(endpoint).await {
            log::error!("Failed to connect to MQTT broker: {:?}", e);
            continue;
        }
        info!("MQTT socket connected");

        let mut config = rust_mqtt::client::client_config::ClientConfig::new(
            rust_mqtt::client::client_config::MqttVersion::MQTTv5,
            CountingRng(20000),
        );
        config.add_client_id("voyagesp");
        const MQTT_BUF_SIZE: usize = 128;
        config.max_packet_size = (MQTT_BUF_SIZE as u32) - 1;
        let mut client_tx = [0u8; MQTT_BUF_SIZE];
        let mut client_rx = [0u8; MQTT_BUF_SIZE];
        let mut client = MqttClient::<_, 5, _>::new(
            &mut mqtt_sock,
            &mut client_tx,
            MQTT_BUF_SIZE,
            &mut client_rx,
            MQTT_BUF_SIZE,
            config,
        );

        if let Err(e) = client.connect_to_broker().await {
            log::error!("Failed to connect to MQTT broker: {:?}", e);
            continue;
        }

        loop {
            let gps_data = {
                let raw = match gps.get_coords().await {
                    Some(data) => data,
                    None => {
                        Timer::after(Duration::from_secs(2)).await;
                        continue;
                    }
                };
                match serde_json::to_string(&raw) {
                    Ok(data) => data,
                    Err(e) => {
                        log::error!("Failed to serialize GPS data: {:?}", e);
                        continue;
                    }
                }
            };
            let result = client
                .send_message(
                    "voyagesp",
                    gps_data.as_bytes(),
                    rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS0,
                    false,
                )
                .await;
            if let Err(e) = result {
                log::error!("Failed to send message: {:?}", e);
                break;
            }
            Timer::after(Duration::from_secs(2)).await;
        }
    }
}

#[task]
async fn modem_stack_config_setter(
    stack: &'static embassy_net::Stack<&'static mut embassy_net_ppp::Device<'static>>,
    config_chan: embassy_sync::channel::Receiver<
        'static,
        CriticalSectionRawMutex,
        StaticConfigV4,
        1,
    >,
) {
    loop {
        let config = config_chan.receive().await;
        log::info!("Got modem ipv4 config: {:?}", config);
        stack.set_config_v4(ConfigV4::Static(config));
    }
}

#[task]
async fn modem_net(
    stack: &'static embassy_net::Stack<&'static mut embassy_net_ppp::Device<'static>>,
) {
    stack.run().await;
}

#[task]
async fn wifi_connection(mut controller: WifiController<'static>) -> ! {
    loop {
        if esp_wifi::wifi::get_wifi_state() == WifiState::StaConnected {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            log::info!("WiFi disconnected");
            Timer::after(Duration::from_millis(5000)).await;
        }

        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: heapless::String::from_str(env!("WIFI_SSID"))
                    .expect("Failed to create SSID string"),
                password: heapless::String::from_str(env!("WIFI_PASSWORD"))
                    .expect("Failed to create password string"),
                ..Default::default()
            });
            controller
                .set_configuration(&client_config)
                .expect("Failed to set WiFi configuration");
            log::info!("Starting WiFi...");
            controller
                .start()
                .await
                .expect("Failed to start WiFi controller");
            log::info!("Started WiFi");
        }

        log::info!("Connecting to WiFi...");
        match controller.connect().await {
            Ok(_) => log::info!("Wifi connected"),
            Err(e) => {
                log::error!("Failed to connect to WiFi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[task]
async fn wifi_net_task(stack: &'static embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await;
}

#[task]
async fn gps_task(gps: &'static Gps<GPS_BAUD>) {
    gps.run().await;
}

#[task]
async fn modem_task(modem: &'static modem::Modem) {
    modem.run().await;
}
