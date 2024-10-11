#![no_std]
#![no_main]

use alloc::vec::Vec;
use anyhow::anyhow;
use embedded_io::{Read, ReadReady, Write};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    uart,
};
use log::info;

extern crate alloc;
use core::mem::MaybeUninit;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);
    init_heap();

    esp_println::logger::init_logger_from_env();

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0, &clocks);
    let _init = esp_wifi::initialize(
        esp_wifi::EspWifiInitFor::Wifi,
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .expect("Failed to initialize WiFi");
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let psu_i2c = esp_hal::i2c::I2C::new(
        peripherals.I2C0,
        io.pins.gpio21,
        io.pins.gpio22,
        100.kHz(),
        &clocks,
    );
    let mut psu = axp192::Axp192::new(psu_i2c);

    psu.set_dcdc1_on(false).unwrap();
    psu.set_ldo2_on(false).unwrap();
    psu.set_ldo3_on(false).unwrap();
    psu.set_dcdc2_on(false).unwrap();
    psu.set_exten_on(false).unwrap();

    const GPS_BAUD: u32 = 9600;
    let mut gps_uart = esp_hal::uart::Uart::new_with_config(
        peripherals.UART1,
        uart::config::Config {
            baudrate: GPS_BAUD,
            data_bits: uart::config::DataBits::DataBits8,
            parity: uart::config::Parity::ParityNone,
            stop_bits: uart::config::StopBits::STOP1,
            rx_timeout: Some(50),
            ..Default::default()
        },
        &clocks,
        io.pins.gpio5,
        io.pins.gpio36,
    )
    .expect("Failed to initialize GPS UART");

    gps_init::<_, _, GPS_BAUD>(&mut gps_uart, &delay).expect("Failed to initialize GPS");

    const MODEM_BAUD: u32 = 115_200;
    let mut modem_uart = esp_hal::uart::Uart::new_with_config(
        peripherals.UART2,
        uart::config::Config {
            baudrate: MODEM_BAUD,
            data_bits: uart::config::DataBits::DataBits8,
            parity: uart::config::Parity::ParityNone,
            stop_bits: uart::config::StopBits::STOP1,
            rx_timeout: Some(50),
            ..Default::default()
        },
        &clocks,
        io.pins.gpio27,
        io.pins.gpio26,
    )
    .expect("Failed to initialize modem UART");

    let mut modem_pwrkey = Output::new(io.pins.gpio4, Level::Low);
    let mut modem_power_on = Output::new(io.pins.gpio25, Level::Low);

    modem_init(
        &mut modem_uart,
        &mut modem_pwrkey,
        &mut modem_power_on,
        &delay,
    )
    .expect("Failed to initialize modem");

    let gps_buf = Vec::<u8>::new();
    let mut gps_parser = ublox::Parser::new(gps_buf);

    loop {
        let mut gps_data = [0u8; 64];
        let len = gps_uart.read(&mut gps_data).unwrap();
        let mut it = gps_parser.consume(&gps_data[..len]);
        loop {
            match it.next() {
                Some(Ok(packet)) => {
                    info!("GPS Packet: {:#?}", packet);
                }
                Some(Err(e)) => {
                    info!("GPS Error: {:?}", e);
                }
                None => {
                    // done processing current gps_data
                    break;
                }
            }
        }
    }
}

fn gps_init<T, M, const BAUD: u32>(
    gps_uart: &mut esp_hal::uart::Uart<T, M>,
    delay: &esp_hal::delay::Delay,
) -> anyhow::Result<()>
where
    T: esp_hal::uart::Instance,
    M: esp_hal::Mode,
{
    let gps_reset_packet = ublox::CfgRstBuilder {
        nav_bbr_mask: ublox::NavBbrMask::all(),
        reset_mode: ublox::ResetMode::ControlledSoftwareReset,
        reserved1: 0,
    }
    .into_packet_bytes();
    info!("Resetting GPS...");
    gps_uart
        .write_all(&gps_reset_packet)
        .map_err(|e| anyhow::anyhow!("Failed to write GPS reset packet: {:?}", e))?;

    // wait for GPS to reset
    delay.delay_millis(1000u32);

    let gps_init_packet = ublox::CfgPrtUartBuilder {
        portid: ublox::UartPortId::Uart1,
        reserved0: 0,
        tx_ready: 0,
        mode: ublox::UartMode::new(
            ublox::DataBits::Eight,
            ublox::Parity::None,
            ublox::StopBits::One,
        ),
        baud_rate: BAUD,
        in_proto_mask: ublox::InProtoMask::all(),
        out_proto_mask: ublox::OutProtoMask::UBLOX,
        flags: 0,
        reserved5: 0,
    }
    .into_packet_bytes();
    info!("Initializing GPS...");
    gps_uart
        .write_all(&gps_init_packet)
        .map_err(|e| anyhow!("Failed to write GPS init packet: {:?}", e))?;
    gps_expect_ack(gps_uart, delay).map_err(|e| anyhow!("Failed to receive GPS ACK: {:?}", e))?;

    //info!("Changing GPS baud rate to {}", GPS_TARGET_BAUD);
    //gps_uart.change_baud(GPS_TARGET_BAUD, ClockSource::Apb, &clocks);

    let nav5_init_packet = ublox::CfgNav5Builder {
        mask: ublox::CfgNav5Params::DYN,
        dyn_model: ublox::CfgNav5DynModel::Automotive,
        ..Default::default()
    }
    .into_packet_bytes();
    info!("Setting GPS to automotive mode");
    gps_uart
        .write_all(&nav5_init_packet)
        .map_err(|e| anyhow!("Failed to write NAV5 init packet: {:?}", e))?;
    gps_expect_ack(gps_uart, delay).map_err(|_| anyhow!("Failed to receive GPS ACK"))?;

    let gps_rate_init = ublox::CfgRateBuilder {
        measure_rate_ms: 500,
        nav_rate: 1,
        time_ref: ublox::AlignmentToReferenceTime::Utc,
    }
    .into_packet_bytes();
    info!("Setting GPS rate to 500ms");
    gps_uart
        .write_all(&gps_rate_init)
        .map_err(|e| anyhow!("Failed to write GPS rate init packet: {:?}", e))?;
    gps_expect_ack(gps_uart, delay).map_err(|_| anyhow!("Failed to receive GPS ACK"))?;

    let nav_pvt_rate_init =
        ublox::CfgMsgSinglePortBuilder::set_rate_for::<ublox::NavPvt>(1).into_packet_bytes();
    info!("Setting NAV-PVT rate to 1 every measurement cycle");
    gps_uart
        .write_all(&nav_pvt_rate_init)
        .map_err(|e| anyhow!("Failed to write NAV-PVT rate init packet: {:?}", e))?;
    gps_expect_ack(gps_uart, delay).map_err(|_| anyhow!("Failed to receive GPS ACK"))?;

    info!("GPS initialized");
    Ok(())
}

fn gps_expect_ack<T, M>(
    uart: &mut esp_hal::uart::Uart<T, M>,
    delay: &esp_hal::delay::Delay,
) -> anyhow::Result<()>
where
    T: esp_hal::uart::Instance,
    M: esp_hal::Mode,
{
    const BUF_SIZE: usize = 64;
    const MAX_ITERATIONS: usize = 100_000;
    let mut parser = ublox::Parser::new(Vec::new());

    let mut i = 0;
    loop {
        if i >= MAX_ITERATIONS {
            anyhow::bail!("Waiting for ACK timed out");
        }

        let ready = uart
            .read_ready()
            .map_err(|e| anyhow!("Failed to check UART read ready: {:?}", e))?;
        if !ready {
            delay.delay_millis(10u32);
            i += 1;
            continue;
        }

        let mut buf: [u8; BUF_SIZE] = [0; BUF_SIZE];
        let len = uart.read(&mut buf).unwrap();
        let mut it = parser.consume(&buf[..len]);
        loop {
            match it.next() {
                Some(Ok(ublox::PacketRef::AckAck(_))) | Some(Ok(ublox::PacketRef::MgaAck(_))) => {
                    return Ok(())
                }
                Some(Ok(ublox::PacketRef::AckNak(_))) => anyhow::bail!("Received NAK from GPS"),
                Some(Err(e)) => anyhow::bail!("GPS Error: {:?}", e),
                None => {
                    // done processing packet
                    // not a valid ACK/NACK, continue to outer waiting loop
                    i = 0;
                    break;
                }
                _ => continue,
            }
        }
    }
}

fn modem_init<T, M, PK, PO>(
    uart: &mut esp_hal::uart::Uart<T, M>,
    pwrkey: &mut esp_hal::gpio::Output<PK>,
    power_on: &mut esp_hal::gpio::Output<PO>,
    delay: &esp_hal::delay::Delay,
) -> anyhow::Result<()>
where
    T: esp_hal::uart::Instance,
    M: esp_hal::Mode,
    PK: esp_hal::gpio::OutputPin,
    PO: esp_hal::gpio::OutputPin,
{
    info!("Resetting modem...");

    power_on.set_high();
    pwrkey.set_high();
    delay.delay_millis(100);
    pwrkey.set_low();
    delay.delay_millis(1_000);
    pwrkey.set_high();

    delay.delay_millis(8_000);

    info!("Modem powered on, initializing...");

    uart.flush_tx()
        .map_err(|e| anyhow!("Failed to flush modem TX: {:?}", e))?;

    write!(uart, "AT\r\n").map_err(|e| anyhow!("Failed to write AT command: {:?}", e))?;
    modem_expect_ack(uart, delay)?;

    Ok(())
}

fn modem_expect_ack<T, M>(
    uart: &mut esp_hal::uart::Uart<T, M>,
    delay: &esp_hal::delay::Delay,
) -> anyhow::Result<()>
where
    T: esp_hal::uart::Instance,
    M: esp_hal::Mode,
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
            delay.delay_millis(10);
            i += 1;
            continue;
        }

        let mut buf = [0u8; 256];
        let len = uart
            .read(&mut buf)
            .map_err(|e| anyhow!("Failed to read modem response: {:?}", e))?;

        let response = core::str::from_utf8(&buf[..len])
            .map_err(|e| anyhow!("Failed to parse modem response: {:?}", e))?;
        log::info!("Modem response: {}", response);

        if response.contains("OK") {
            return Ok(());
        }
    }
}
