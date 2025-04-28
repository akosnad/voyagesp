#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]
#![feature(impl_trait_in_assoc_type)]
#![feature(const_int_from_str)]

use alloc::{boxed::Box, sync::Arc};
use core::{str::FromStr, sync::atomic::AtomicBool};
use embassy_executor::{task, Spawner};
use embassy_futures::{
    select::{self, select},
    yield_now,
};
use embassy_net::{ConfigV4, Ipv4Address, StackResources, StaticConfigV4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write;
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, Io, Output},
    peripherals::{UART0, UART2},
    prelude::*,
    rng::Rng,
    uart::{self, UartRx, UartTx},
    Async, Blocking,
};
use esp_hal_embassy::main;
use esp_wifi::wifi::{
    utils::create_network_interface, ClientConfiguration, Configuration, WifiController,
    WifiDevice, WifiEvent, WifiStaDevice, WifiState,
};
use log::info;
use mqtt::PublishEvent;
use static_cell::make_static;
pub use voyagesp as lib;

extern crate alloc;

mod config;
mod gps;
mod modem;
mod mqtt;

const GPS_BAUD: u32 = 9600;
const MODEM_BAUD: u32 = 115_200;

const SYSTEM_EVENT_QUEUE_SIZE: usize = 5;
const HEAP_SIZE: usize = 56 * 1024;

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

    // esp_println::logger::init_logger(log::LevelFilter::Debug);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // MODEM
    let mut modem_dtr = Output::new(io.pins.gpio32, esp_hal::gpio::Level::Low);
    let modem_ri = Input::new(io.pins.gpio33, esp_hal::gpio::Pull::None);

    let mut modem_pwrkey = Output::new(io.pins.gpio4, esp_hal::gpio::Level::Low);
    let mut modem_power_on = Output::new(io.pins.gpio25, esp_hal::gpio::Level::Low);

    let (modem_rx, modem_tx) = esp_hal::uart::Uart::new_async_with_config(
        peripherals.UART2,
        uart::config::Config {
            baudrate: MODEM_BAUD,
            data_bits: uart::config::DataBits::DataBits8,
            parity: uart::config::Parity::ParityNone,
            stop_bits: uart::config::StopBits::STOP1,
            ..Default::default()
        },
        io.pins.gpio26,
        io.pins.gpio27,
    )
    .expect("Failed to initialize modem UART")
    .split();

    let (usb_rx, usb_tx) = esp_hal::uart::Uart::new_async_with_config(
        peripherals.UART0,
        uart::config::Config {
            baudrate: 115200,
            data_bits: uart::config::DataBits::DataBits8,
            parity: uart::config::Parity::ParityNone,
            stop_bits: uart::config::StopBits::STOP1,
            ..Default::default()
        },
        io.pins.gpio3,
        io.pins.gpio1,
    )
    .expect("failed to init usb uart")
    .split();

    // init modem
    modem_dtr.set_low();

    modem_power_on.set_low();
    Timer::after(Duration::from_millis(100)).await;

    modem_power_on.set_high();
    modem_pwrkey.set_high();
    Timer::after(Duration::from_millis(100)).await;
    modem_pwrkey.set_low();
    Timer::after(Duration::from_millis(1_000)).await;
    modem_pwrkey.set_high();
    Timer::after(Duration::from_millis(6_000)).await;

    spawner.spawn(usb_to_modem(usb_rx, modem_tx)).unwrap();
    spawner.spawn(modem_to_usb(modem_rx, usb_tx)).unwrap();
}

#[task]
async fn usb_to_modem(
    mut usb_rx: UartRx<'static, UART0, Async>,
    mut modem_tx: UartTx<'static, UART2, Async>,
) {
    loop {
        use embedded_io_async::Read as _;
        use embedded_io_async::Write as _;

        let mut buf = [0u8; 512];
        let Ok(len) = usb_rx.read(&mut buf).await else {
            continue;
        };
        modem_tx.write_all(&buf[..len]).await.ok();
        modem_tx.flush_async().await.ok();
    }
}

#[task]
async fn modem_to_usb(
    mut modem_rx: UartRx<'static, UART2, Async>,
    mut usb_tx: UartTx<'static, UART0, Async>,
) {
    loop {
        use embedded_io_async::Read as _;
        use embedded_io_async::Write as _;

        let mut buf = [0u8; 512];
        let Ok(len) = modem_rx.read(&mut buf).await else {
            continue;
        };
        usb_tx.write_all(&buf[..len]).await.ok();
        usb_tx.flush_async().await.ok();
    }
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

        match controller.connect().await {
            Ok(_) => log::info!("Wifi connected"),
            Err(_) => Timer::after(Duration::from_millis(5000)).await,
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

#[task]
async fn modem_power_task(
    modem: &'static modem::Modem,
    ignition_state: Arc<AtomicBool>,
    modem_stack: &'static embassy_net::Stack<&'static mut embassy_net_ppp::Device<'static>>,
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
