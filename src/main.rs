#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(impl_trait_in_assoc_type)]

use alloc::sync::Arc;
use core::{str::FromStr, sync::atomic::AtomicBool};
use embassy_executor::{task, Spawner};
use embassy_futures::yield_now;
use embassy_net::{ConfigV4, Ipv4Address, Runner, Stack, StackResources, StaticConfigV4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, Output},
    rng::Rng,
    uart, Blocking,
};
use esp_hal_embassy::main;
use esp_wifi::{
    wifi::{
        ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice,
        WifiState,
    },
    EspWifiController,
};
use mqtt::PublishEvent;
use static_cell::StaticCell;
pub use voyagesp as lib;

extern crate alloc;

mod config;
mod gps;
mod modem;
mod mqtt;
mod ota;

const GPS_BAUD: u32 = 9600;
const MODEM_BAUD: u32 = 115_200;

const SYSTEM_EVENT_QUEUE_SIZE: usize = 5;
const HEAP_SIZE: usize = 68 * 1024;

const PSU_DATA_INTERVAL: Duration = Duration::from_secs(15);
const PSU_DATA_IDLE_INTERVAL: Duration = Duration::from_secs(60);
const GPS_DATA_INTERVAL: Duration = Duration::from_secs(5);
const IGNITION_SENSE_DEBOUNCE: Duration = Duration::from_secs(2);
const POWERSAVE_DELAY: Duration = Duration::from_secs(
    match u64::from_str_radix(env!("POWERSAVE_DELAY_SECONDS"), 10) {
        Ok(d) => d,
        Err(_) => panic!("Invalid POWERSAVE_DELAY_SECONDS"),
    },
);

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

type Psu = axp192::Axp192<esp_hal::i2c::master::I2c<'static, Blocking>>;
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

    //TODO: actually check if OTA is valid, instead of blindly accepting
    let mut flash = esp_storage::FlashStorage::new();
    esp_ota_nostd::ota_accept(&mut flash).expect("failed to mark OTA valid");
    log::info!("[OTA] marked current boot as valid");

    // Ignition sense
    let ignition_sense = Input::new(peripherals.GPIO14, esp_hal::gpio::Pull::Down);
    let event_channel = {
        static CELL: StaticCell<
            embassy_sync::channel::Channel<
                CriticalSectionRawMutex,
                SystemEvent,
                SYSTEM_EVENT_QUEUE_SIZE,
            >,
        > = StaticCell::new();
        CELL.init(embassy_sync::channel::Channel::new())
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
        esp_hal::i2c::master::I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
            .expect("failed to init i2c")
            .with_sda(peripherals.GPIO21)
            .with_scl(peripherals.GPIO22);
    let psu = {
        static CELL: StaticCell<Psu> = StaticCell::new();
        CELL.init(axp192::Axp192::new(psu_i2c))
    };

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
    let wifi_init = {
        static CELL: StaticCell<EspWifiController<'static>> = StaticCell::new();
        CELL.init(
            esp_wifi::init(timg0.timer1, rng, peripherals.RADIO_CLK)
                .expect("Failed to initialize WiFi clocks"),
        )
    };
    let (wifi_device, wifi_controller) = esp_wifi::wifi::new_with_config::<WifiStaDevice>(
        wifi_init,
        peripherals.WIFI,
        esp_wifi::wifi::ClientConfiguration {
            ssid: heapless::String::from_str(env!("WIFI_SSID"))
                .expect("Failed to create SSID string"),
            password: heapless::String::from_str(env!("WIFI_PASSWORD"))
                .expect("Failed to create password string"),
            ..Default::default()
        },
    )
    .expect("Failed to create WiFi network interface");

    let stack_resources = {
        static CELL: StaticCell<StackResources<3>> = StaticCell::new();
        CELL.init(StackResources::new())
    };
    let (wifi_stack, wifi_runner) = {
        static CELL: StaticCell<(Stack, Runner<WifiDevice<'static, WifiStaDevice>>)> =
            StaticCell::new();
        CELL.init(embassy_net::new(
            wifi_device,
            embassy_net::Config::dhcpv4(Default::default()),
            stack_resources,
            1234u64,
        ))
    };

    spawner
        .spawn(wifi_connection(wifi_controller))
        .expect("Failed to spawn WiFi connection task");
    spawner
        .spawn(wifi_net_task(wifi_runner))
        .expect("Failed to spawn WiFi task");

    // GPS
    let gps_uart = esp_hal::uart::Uart::new(
        peripherals.UART1,
        uart::Config::default()
            .with_baudrate(GPS_BAUD)
            .with_data_bits(uart::DataBits::_8)
            .with_parity(uart::Parity::None)
            .with_stop_bits(uart::StopBits::_1)
            .with_rx_timeout(50),
    )
    .expect("Failed to initialize GPS UART")
    .with_rx(peripherals.GPIO13)
    .with_tx(peripherals.GPIO15)
    .into_async();

    let gps = {
        static CELL: StaticCell<Gps> = StaticCell::new();
        CELL.init(
            gps::Gps::new(gps_uart)
                .await
                .expect("Failed to initialize GPS"),
        )
    };
    spawner
        .spawn(gps_task(gps))
        .expect("Failed to spawn GPS task");

    // MODEM
    let modem_dtr = Output::new(peripherals.GPIO32, esp_hal::gpio::Level::Low);
    let modem_ri = Input::new(peripherals.GPIO33, esp_hal::gpio::Pull::None);

    let modem_pwrkey = Output::new(peripherals.GPIO4, esp_hal::gpio::Level::Low);
    let modem_power_on = Output::new(peripherals.GPIO25, esp_hal::gpio::Level::Low);

    let modem_uart = esp_hal::uart::Uart::new(
        peripherals.UART2,
        uart::Config::default()
            .with_baudrate(MODEM_BAUD)
            .with_data_bits(uart::DataBits::_8)
            .with_parity(uart::Parity::None)
            .with_stop_bits(uart::StopBits::_1)
            .with_rx_timeout(50),
    )
    .expect("Failed to initialize modem UART")
    .with_rx(peripherals.GPIO26)
    .with_tx(peripherals.GPIO27)
    .into_async();

    let config_chan = {
        static CELL: StaticCell<
            embassy_sync::channel::Channel<CriticalSectionRawMutex, StaticConfigV4, 1>,
        > = StaticCell::new();
        CELL.init(embassy_sync::channel::Channel::new())
    };
    let (modem, modem_ppp) = {
        static CELL: StaticCell<(modem::Modem, embassy_net_ppp::Device<'static>)> =
            StaticCell::new();
        CELL.init(
            modem::Modem::new(
                &spawner,
                modem_uart,
                modem_dtr,
                modem_ri,
                modem_pwrkey,
                modem_power_on,
                config_chan.sender(),
            )
            .await
            .expect("Failed to initialize modem"),
        )
    };
    spawner
        .spawn(modem_task(modem))
        .expect("Failed to spawn modem task");

    let stack_resources = {
        static CELL: StaticCell<StackResources<3>> = StaticCell::new();
        CELL.init(StackResources::new())
    };
    let seed = {
        let mut buf = [0u8; 8];
        rng.read(&mut buf);
        u64::from_le_bytes(buf)
    };

    let (modem_stack, modem_runner) = {
        // this is a hack around the non-exhaustive struct `embassy_net::Config`
        let dummy_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
            address: embassy_net::Ipv4Cidr::new(Ipv4Address::new(0, 0, 0, 0), 0),
            gateway: Some(Ipv4Address::new(0, 0, 0, 0)),
            dns_servers: heapless::Vec::new(),
        });
        let (stack, runner) = embassy_net::new(modem_ppp, dummy_config, stack_resources, seed);
        // here we set the actual config; this is the only way without violating the non-exhaustive
        // initialization of `embassy_net::Config`
        stack.set_config_v4(ConfigV4::None);

        static CELL: StaticCell<(
            Stack,
            Runner<'static, &mut embassy_net_ppp::Device<'static>>,
        )> = StaticCell::new();
        CELL.init((stack, runner))
    };

    spawner
        .spawn(modem_net(modem_runner))
        .expect("Failed to spawn modem network task");
    spawner
        .spawn(modem_stack_config_setter(
            modem_stack,
            config_chan.receiver(),
        ))
        .expect("Failed to spawn modem stack config setter");

    // MQTT
    let mqtt = {
        static CELL: StaticCell<mqtt::Mqtt> = StaticCell::new();
        CELL.init(mqtt::Mqtt::new(
            wifi_stack.clone(),
            modem_stack.clone(),
            rng,
        ))
    };
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
            ignition_state.clone(),
        ))
        .expect("Failed to spawn GPS data fetcher");
    spawner
        .spawn(psu_state_task(
            psu,
            event_channel.sender(),
            ignition_state.clone(),
        ))
        .expect("Failed to spawn PSU task");
    spawner
        .spawn(modem_power_task(modem, ignition_state, modem_stack))
        .expect("Failed to spawn modem power task");
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
        }
        Timer::after(GPS_DATA_INTERVAL).await;
    }
}

fn get_psu_data(psu: &mut Psu) -> anyhow::Result<PsuData> {
    Ok(PsuData {
        battery_charging: psu.get_charging().map_err(|e| anyhow::anyhow!("{e:?}"))?,
        battery_voltage: psu
            .get_battery_voltage()
            .map_err(|e| anyhow::anyhow!("{e:?}"))?,
        battery_charge_current: psu
            .get_battery_charge_current()
            .map_err(|e| anyhow::anyhow!("{e:?}"))?,
        ext_voltage: psu
            .get_acin_voltage()
            .map_err(|e| anyhow::anyhow!("{e:?}"))?,
        ext_current: psu
            .get_acin_current()
            .map_err(|e| anyhow::anyhow!("{e:?}"))?,
        temperature: psu
            .get_internal_temperature()
            .map_err(|e| anyhow::anyhow!("{e:?}"))?,
    })
}

#[task]
async fn psu_state_task(
    psu: &'static mut Psu,
    event_sender: SystemEventSender,
    ignition_state: Arc<AtomicBool>,
) -> ! {
    loop {
        match get_psu_data(psu) {
            Ok(data) => event_sender.send(SystemEvent::PsuData(data)).await,
            Err(e) => log::error!("Failed to get PSU data: {:?}", e),
        };

        if ignition_state.load(core::sync::atomic::Ordering::SeqCst) {
            Timer::after(PSU_DATA_INTERVAL).await;
        } else {
            Timer::after(PSU_DATA_IDLE_INTERVAL).await;
        }
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
        let level = sense.level();
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
    stack: &'static embassy_net::Stack<'static>,
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
    runner: &'static mut embassy_net::Runner<
        'static,
        &'static mut embassy_net_ppp::Device<'static>,
    >,
) {
    runner.run().await;
}

#[task]
async fn wifi_connection(mut controller: WifiController<'static>) -> ! {
    loop {
        if esp_wifi::wifi::wifi_state() == WifiState::StaConnected {
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
            controller.start().expect("Failed to start WiFi controller");
            log::info!("Started WiFi");
        }

        match controller.connect_async().await {
            Ok(_) => log::info!("Wifi connected"),
            Err(_) => Timer::after(Duration::from_millis(5000)).await,
        }
    }
}

#[task]
async fn wifi_net_task(
    runner: &'static mut embassy_net::Runner<'static, WifiDevice<'static, WifiStaDevice>>,
) {
    runner.run().await;
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

#[task]
async fn modem_power_task(
    modem: &'static modem::Modem,
    ignition_state: Arc<AtomicBool>,
    modem_stack: &'static embassy_net::Stack<'static>,
) {
    let mut ignition_off_start: Option<Instant> = None;
    loop {
        if ignition_state.load(core::sync::atomic::Ordering::SeqCst) {
            ignition_off_start = None;
            if !modem.is_powered_up().await {
                if let Err(e) = modem.power_up().await {
                    log::error!("Failed to power up modem: {:?}", e);
                }
            }
        } else {
            match ignition_off_start {
                Some(ignition_off_start) => {
                    if ignition_off_start.elapsed() > POWERSAVE_DELAY && modem.is_powered_up().await
                    {
                        if modem_stack.config_v4().is_some() {
                            modem_stack.set_config_v4(ConfigV4::None);
                        }
                        if let Err(e) = modem.power_down().await {
                            log::error!("Failed to power down modem: {:?}", e);
                        }
                    }
                }
                None => ignition_off_start = Some(Instant::now()),
            }
        }
        // FIXME: this is a busy loop, the nice soulution would be to poll ignition state with futures
        yield_now().await;
    }
}
