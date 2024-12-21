#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(impl_trait_in_assoc_type)]

use alloc::{boxed::Box, sync::Arc};
use core::{str::FromStr, sync::atomic::AtomicBool};
use embassy_executor::{task, Spawner};
use embassy_net::{ConfigV4, Ipv4Address, StackResources, StaticConfigV4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, Io, Output},
    prelude::*,
    rng::Rng,
    uart, Blocking,
};
use esp_hal_embassy::main;
use esp_wifi::wifi::{
    utils::create_network_interface, ClientConfiguration, Configuration, WifiController,
    WifiDevice, WifiEvent, WifiStaDevice, WifiState,
};
use mqtt::PublishEvent;
use static_cell::make_static;

extern crate alloc;

mod config;
mod gps;
mod modem;
mod mqtt;

const GPS_BAUD: u32 = 9600;
const MODEM_BAUD: u32 = 115_200;

const SYSTEM_EVENT_QUEUE_SIZE: usize = 5;
const HEAP_SIZE: usize = 64 * 1024;

const PSU_DATA_INTERVAL: Duration = Duration::from_secs(15);
const GPS_DATA_INTERVAL: Duration = Duration::from_secs(5);
const IGNITION_SENSE_DEBOUNCE: Duration = Duration::from_secs(1);

type SystemEventSender = embassy_sync::channel::Sender<
    'static,
    CriticalSectionRawMutex,
    SystemEvent,
    SYSTEM_EVENT_QUEUE_SIZE,
>;
type SystemEventReceiver = embassy_sync::channel::Receiver<
    'static,
    CriticalSectionRawMutex,
    SystemEvent,
    SYSTEM_EVENT_QUEUE_SIZE,
>;

type Psu = axp192::Axp192<esp_hal::i2c::I2c<'static, esp_hal::peripherals::I2C0, Blocking>>;
type Gps = gps::Gps<GPS_BAUD>;

#[derive(Debug)]
pub enum SystemEvent {
    IgnitionStateChange(bool),
    GpsData(gps::GpsCoords),
    PsuData(PsuData),
}

#[derive(Debug)]
pub struct PsuData {
    pub battery_charging: bool,
    pub battery_voltage: f32,
    pub battery_charge_current: f32,
    pub ext_voltage: f32,
    pub ext_current: f32,
    pub temperature: f32,
}

#[export_name = "custom_halt"]
pub fn custom_halt() -> ! {
    loop {
        esp_hal::reset::software_reset();
    }
}

#[main]
async fn main_task(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    esp_alloc::heap_allocator!(HEAP_SIZE);

    esp_println::logger::init_logger(log::LevelFilter::Debug);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Ignition sense
    let ignition_sense = Input::new(io.pins.gpio14, esp_hal::gpio::Pull::Down);
    let event_channel = {
        let chan: embassy_sync::channel::Channel<
            CriticalSectionRawMutex,
            SystemEvent,
            SYSTEM_EVENT_QUEUE_SIZE,
        > = embassy_sync::channel::Channel::new();
        let boxed = Box::new(chan);
        Box::leak(boxed)
    };

    let ignition_state = Arc::new(AtomicBool::new(false));
    spawner
        .spawn(ignition_sense_task(
            ignition_sense,
            event_channel.sender(),
            ignition_state.clone(),
        ))
        .expect("Failed to spawn ignition sense task");

    // PSU
    let psu_i2c =
        esp_hal::i2c::I2c::new(peripherals.I2C0, io.pins.gpio21, io.pins.gpio22, 100.kHz());
    let psu: &mut Psu = make_static!(axp192::Axp192::new(psu_i2c));

    psu.set_dcdc1_on(false).unwrap();
    psu.set_ldo2_on(false).unwrap();
    psu.set_ldo3_on(false).unwrap();
    psu.set_dcdc2_on(false).unwrap();
    psu.set_exten_on(false).unwrap();
    psu.set_acin_voltage_adc_enable(true).unwrap();
    psu.set_acin_current_adc_enable(true).unwrap();
    psu.set_battery_voltage_adc_enable(true).unwrap();
    psu.set_battery_current_adc_enable(true).unwrap();

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

    let gps: &mut Gps = {
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
    let mqtt = make_static!(mqtt::Mqtt::new(wifi_stack, modem_stack, rng));
    spawner
        .spawn(mqtt_task(mqtt))
        .expect("Failed to spawn MQTT task");

    // System tasks
    spawner
        .spawn(mqtt_publisher_task(event_channel.receiver(), mqtt))
        .expect("Failed to spawn system data sender");
    spawner
        .spawn(gps_data_fetcher(
            gps,
            event_channel.sender(),
            ignition_state,
        ))
        .expect("Failed to spawn GPS data fetcher");
    spawner
        .spawn(psu_state_task(psu, event_channel.sender()))
        .expect("Failed to spawn PSU task");
}

#[task]
async fn gps_data_fetcher(
    gps: &'static Gps,
    event_sender: SystemEventSender,
    ignition_state: Arc<AtomicBool>,
) -> ! {
    loop {
        if ignition_state.load(core::sync::atomic::Ordering::SeqCst) {
            if let Some(gps_coords) = gps.get_coords().await {
                event_sender.send(SystemEvent::GpsData(gps_coords)).await;
            } else {
                log::warn!("Tried to get GPS data before initialization");
            }
        } else {
            log::debug!("GPS data fetcher: Ignition off, skipping GPS data fetch");
        }
        Timer::after(GPS_DATA_INTERVAL).await;
    }
}

#[task]
async fn psu_state_task(psu: &'static mut Psu, event_sender: SystemEventSender) -> ! {
    loop {
        let data = PsuData {
            battery_charging: psu.get_charging().unwrap(),
            battery_voltage: psu.get_battery_voltage().unwrap(),
            battery_charge_current: psu.get_battery_charge_current().unwrap(),
            ext_voltage: psu.get_acin_voltage().unwrap(),
            ext_current: psu.get_acin_current().unwrap(),
            temperature: psu.get_internal_temperature().unwrap(),
        };
        event_sender.send(SystemEvent::PsuData(data)).await;

        Timer::after(PSU_DATA_INTERVAL).await;
    }
}

#[task]
async fn mqtt_publisher_task(event_receiver: SystemEventReceiver, mqtt: &'static mqtt::Mqtt) -> ! {
    loop {
        match event_receiver.receive().await {
            SystemEvent::IgnitionStateChange(ignition_state) => {
                mqtt.publish(PublishEvent::Ignition(ignition_state)).await;
            }
            SystemEvent::GpsData(gps_data) => {
                mqtt.publish(PublishEvent::DeviceTracker(gps_data)).await;
            }
            SystemEvent::PsuData(psu_state) => {
                mqtt.publish(PublishEvent::Psu(psu_state)).await;
            }
        }
    }
}

#[task]
async fn ignition_sense_task(
    sense: Input<'static>,
    event_sender: SystemEventSender,
    ignition_state: Arc<AtomicBool>,
) -> ! {
    let mut last_level = None;
    loop {
        let level = sense.get_level();
        if last_level
            .map(|last_level| level != last_level)
            .unwrap_or(true)
        {
            let level = level == esp_hal::gpio::Level::High;
            log::info!("Ignition sense: {:?}", level);
            ignition_state.store(level, core::sync::atomic::Ordering::SeqCst);
            event_sender
                .send(SystemEvent::IgnitionStateChange(level))
                .await;
        }
        last_level = Some(level);
        Timer::after(IGNITION_SENSE_DEBOUNCE).await;
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
                log::warn!("Failed to connect to WiFi: {:?}", e);
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
async fn gps_task(gps: &'static Gps) {
    gps.run().await;
}

#[task]
async fn modem_task(modem: &'static modem::Modem) {
    modem.run().await;
}

#[task]
async fn mqtt_task(mqtt: &'static mqtt::Mqtt) {
    mqtt.run().await;
}
