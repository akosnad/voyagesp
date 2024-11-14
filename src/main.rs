#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]

use alloc::{boxed::Box, sync::Arc};
use core::str::FromStr;
use embassy_executor::{task, Spawner};
use embassy_net::StackResources;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read as _, Write as _};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, Io, Output},
    peripherals::{UART0, UART2},
    prelude::*,
    reset::software_reset,
    rng::Rng,
    uart::{self, UartRx, UartTx},
    Async,
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
mod modem;

use gps::Gps;

#[export_name = "custom_halt"]
pub fn custom_halt() -> ! {
    loop {
        //software_reset();
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

    // let status_chan = channel::Channel::<NoopRawMutex, StatusEvent, 3>::new();

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

    //let mut modem_interface = modem::ModemInterface {
    //    uart: modem_uart,
    //    dtr: modem_dtr,
    //    ri: modem_ri,
    //    pwrkey: modem_pwrkey,
    //    power_on: modem_power_on,
    //};
    //modem::modem_init(&mut modem_interface).await.expect("Failed to initialize modem");

    //let mut usb_uart = esp_hal::uart::Uart::new_async_with_config(
    //    peripherals.UART0,
    //    uart::config::Config {
    //        baudrate: 115_200,
    //        data_bits: uart::config::DataBits::DataBits8,
    //        parity: uart::config::Parity::ParityNone,
    //        stop_bits: uart::config::StopBits::STOP1,
    //        rx_timeout: Some(50),
    //        ..Default::default()
    //    },
    //    io.pins.gpio3,
    //    io.pins.gpio1,
    //).expect("Failed to initialize USB UART");

    //usb_uart.write_all(b"Hello, world!\r\n").await.expect("Failed to write to USB UART");

    //let (usb_rx, usb_tx) = usb_uart.split();
    //let (modem_rx, modem_tx) = modem_interface.uart.split();

    //let usb_tx: UsbTx = Arc::new(Mutex::new(usb_tx));

    //spawner.spawn(usb_reader(usb_rx, modem_tx, usb_tx.clone())).expect("Failed to spawn USB reader task");
    //spawner.spawn(modem_reader(modem_rx, usb_tx)).expect("Failed to spawn modem reader task");

    let modem = {
        let modem = modem::Modem::new(
            &spawner,
            modem_uart,
            modem_dtr,
            modem_ri,
            modem_pwrkey,
            modem_power_on,
        )
        .await
        .expect("Failed to initialize modem");
        make_static!(modem)
    };
    spawner
        .spawn(modem_task(modem))
        .expect("Failed to spawn modem task");

    loop {
        Timer::after(Duration::from_secs(5)).await;
        //if let Err(e) = modem_uart.write(b"AT+CSQ\r\n").await {
        //    log::error!("Failed to write AT command: {:?}", e);
        //}
        //modem_ri.wait_for_high().await;
        //let mut buf = [0u8; 512];
        //match modem_uart.read(&mut buf).await {
        //    Ok(len) => {
        //        if let Ok(response) = core::str::from_utf8(&buf[..len]) {
        //            log::info!("Modem response: {}", response);
        //        } else {
        //            log::error!("Failed to parse modem response");
        //        }
        //    }
        //    Err(e) => {
        //        log::error!("GPS Read failed: {:?}", e);
        //        continue;
        //    }
        //};
        info!("gps coords: {:?}", gps.get_coords().await);
    }
}

type UsbTx = Arc<Mutex<CriticalSectionRawMutex, UartTx<'static, UART0, Async>>>;

#[task]
async fn usb_reader(
    mut usb_rx: UartRx<'static, UART0, Async>,
    mut modem_tx: UartTx<'static, UART2, Async>,
    usb_tx: UsbTx,
) -> ! {
    loop {
        let mut buf = [0u8; 512];
        let len = usb_rx
            .read(&mut buf)
            .await
            .expect("Failed to read from USB UART");
        modem_tx
            .write_all(&buf[..len])
            .await
            .expect("Failed to write to modem UART");
        usb_tx
            .lock()
            .await
            .write_all(&buf[..len])
            .await
            .expect("Failed to write to USB UART");
    }
}

#[task]
async fn modem_reader(mut modem_rx: UartRx<'static, UART2, Async>, usb_tx: UsbTx) -> ! {
    loop {
        let mut buf = [0u8; 1024];
        match modem_rx.read(&mut buf).await {
            Ok(len) => {
                usb_tx
                    .lock()
                    .await
                    .write_all(&buf[..len])
                    .await
                    .expect("Failed to write to USB UART");
            }
            Err(e) => {
                log::error!("Modem read failed: {:?}", e);
            }
        }
    }
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
