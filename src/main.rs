#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]

use alloc::boxed::Box;
use anyhow::anyhow;
use core::str::FromStr;
use embassy_executor::{task, Spawner};
use embassy_net::StackResources;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel};
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read as _, ReadReady as _, Write as _};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Io, Output},
    prelude::*,
    reset::software_reset,
    rng::Rng,
    uart, Async,
};
use esp_hal_embassy::main;
use esp_wifi::{
    wifi::{
        utils::create_network_interface, ClientConfiguration, Configuration, WifiController,
        WifiDevice, WifiEvent, WifiStaDevice, WifiState,
    },
    EspWifiInitFor,
};
use log::info;
use smoltcp::iface::SocketStorage;
use static_cell::make_static;

extern crate alloc;

mod gps;

use gps::Gps;

#[export_name = "custom_halt"]
pub fn custom_halt() -> ! {
    loop {
        software_reset();
    }
}

#[main]
async fn main_task(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger_from_env();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let psu_i2c =
        esp_hal::i2c::I2c::new(peripherals.I2C0, io.pins.gpio21, io.pins.gpio22, 100.kHz());
    let mut psu = axp192::Axp192::new(psu_i2c);

    psu.set_dcdc1_on(false).unwrap();
    psu.set_ldo2_on(false).unwrap();
    psu.set_ldo3_on(false).unwrap();
    psu.set_dcdc2_on(false).unwrap();
    psu.set_exten_on(false).unwrap();

    let wifi_init = esp_wifi::init(
        EspWifiInitFor::Wifi,
        timg0.timer1,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .expect("Failed to initialize WiFi clocks");
    let mut socket_storage: [SocketStorage; 3] = Default::default();
    let (_wifi_interface, wifi_device, wifi_controller, _wifi_sockets) = create_network_interface(
        &wifi_init,
        peripherals.WIFI,
        WifiStaDevice,
        &mut socket_storage,
    )
    .expect("Failed to create WiFi network interface");

    let stack_resources: &mut StackResources<3> = Box::leak(Box::new(StackResources::new()));
    let stack = make_static!(embassy_net::Stack::new(
        wifi_device,
        embassy_net::Config::dhcpv4(Default::default()),
        stack_resources,
        1234u64,
    ));

    spawner
        .spawn(wifi_connection(wifi_controller))
        .expect("Failed to spawn WiFi connection task");
    spawner
        .spawn(wifi_net_task(stack))
        .expect("Failed to spawn WiFi task");

    //log::info!("Waiting for WiFi link...");
    //loop {
    //    if stack.is_link_up() {
    //        log::info!("WiFi link up");
    //        break;
    //    }
    //    Timer::after(Duration::from_millis(500)).await;
    //}

    //log::info!("Waiting for IP...");
    //loop {
    //    if let Some(config) = stack.config_v4() {
    //        log::info!("Got IP: {}", config.address);
    //        break;
    //    };
    //    Timer::after(Duration::from_millis(500)).await;
    //}

    const GPS_BAUD: u32 = 9600;
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
        io.pins.gpio34,
        io.pins.gpio32,
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

    let status_chan = channel::Channel::<NoopRawMutex, StatusEvent, 3>::new();

    const MODEM_BAUD: u32 = 115_200;
    let mut modem_uart = esp_hal::uart::Uart::new_async_with_config(
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

    let mut modem_pwrkey = Output::new(io.pins.gpio4, esp_hal::gpio::Level::Low);
    let mut modem_power_on = Output::new(io.pins.gpio25, esp_hal::gpio::Level::Low);

    modem_init(&mut modem_uart, &mut modem_pwrkey, &mut modem_power_on)
        .await
        .expect("Failed to initialize modem");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        info!("gps coords: {:?}", gps.get_coords().await);
    }
}

enum StatusEvent {}

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
                ssid: heapless::String::from_str("Gaia").expect("Failed to create SSID string"),
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
async fn gps_task(gps: &'static Gps<9600>) {
    gps.run().await;
}

async fn modem_init<T, PK, PO>(
    uart: &mut esp_hal::uart::Uart<'_, T, Async>,
    pwrkey: &mut esp_hal::gpio::Output<'_, PK>,
    power_on: &mut esp_hal::gpio::Output<'_, PO>,
) -> anyhow::Result<()>
where
    T: esp_hal::uart::Instance,
    PK: esp_hal::gpio::OutputPin,
    PO: esp_hal::gpio::OutputPin,
{
    info!("Resetting modem...");

    power_on.set_high();
    pwrkey.set_high();
    Timer::after(Duration::from_millis(100)).await;
    pwrkey.set_low();
    Timer::after(Duration::from_millis(1_000)).await;
    pwrkey.set_high();

    Timer::after(Duration::from_millis(8_000)).await;

    info!("Modem powered on, initializing...");

    uart.flush_tx()
        .map_err(|e| anyhow!("Failed to flush modem TX: {:?}", e))?;

    uart.write(b"AT\r\n")
        .await
        .map_err(|e| anyhow!("Failed to write AT command: {:?}", e))?;
    modem_expect_ack(uart).await?;

    Ok(())
}

async fn modem_expect_ack<T>(uart: &mut esp_hal::uart::Uart<'_, T, Async>) -> anyhow::Result<()>
where
    T: esp_hal::uart::Instance,
{
    const MAX_ITERATIONS: usize = 100;
    let mut i = 0;

    loop {
        if i >= MAX_ITERATIONS {
            anyhow::bail!("Timed out while initializing modem");
        }

        let ready = uart
            .read_ready()
            .map_err(|e| anyhow!("Failed to check UART read ready: {:?}", e))?;
        if !ready {
            Timer::after(Duration::from_millis(10)).await;
            i += 1;
            continue;
        }

        let mut buf = [0u8; 256];
        let len = uart
            .read(&mut buf)
            .await
            .map_err(|e| anyhow!("Failed to read modem response: {:?}", e))?;

        let response = core::str::from_utf8(&buf[..len])
            .map_err(|e| anyhow!("Failed to parse modem response: {:?}", e))?;
        log::info!("Modem response: {}", response);

        if response.contains("OK") {
            return Ok(());
        }
    }
}
