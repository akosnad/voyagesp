use std::thread::JoinHandle;

use esp_idf_hal::{
    cpu::Core,
    ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver},
    peripherals::Peripherals,
    prelude::*,
    task::thread::ThreadSpawnConfiguration,
    gpio::AnyIOPin,
};

use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    nvs::EspDefaultNvsPartition,
    timer::EspTaskTimerService,
};
use esp_idf_sys::esp_restart;
use log::{error, info};

/// Helper which spawns a task with a name
fn spawn_task(
    task: impl FnOnce() + Send + 'static,
    task_name: &'static str,
    pin_to_core: Option<Core>,
) -> anyhow::Result<JoinHandle<()>> {
    info!("spawning task: {}", task_name);

    ThreadSpawnConfiguration {
        name: Some(task_name.as_bytes()),
        pin_to_core,
        ..Default::default()
    }
    .set()?;

    let handle = std::thread::Builder::new().stack_size(8192).spawn(task)?;

    info!("spawned task: {}", task_name);

    Ok(handle)
}

#[allow(unreachable_code)]
fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    #[cfg(feature = "simulation")]
    {
        return simulation();
    }

    let peripherals = Peripherals::take()?;
    let pins = peripherals.pins;
    let sysloop = EspSystemEventLoop::take()?;
    let timer = EspTaskTimerService::new()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let led = {
        let timer = LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &TimerConfig::default().frequency(25.kHz().into()),
        )?;
        let led = LedcDriver::new(peripherals.ledc.channel0, timer, pins.gpio2)?;
        Box::leak(Box::new(led))
    };
    led.set_duty(0)?;

    let psu_i2c = esp_idf_svc::hal::i2c::I2cDriver::new(
        peripherals.i2c0,
        pins.gpio21,
        pins.gpio22,
        &esp_idf_svc::hal::i2c::config::Config::default(),
    )?;
    let mut psu = axp192::Axp192::new(psu_i2c);

    const TARGET_BAUDRATE: u32 = 115_200;
    let gps_uart = esp_idf_svc::hal::uart::UartDriver::new(
        peripherals.uart1,
        pins.gpio5,
        pins.gpio36,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &esp_idf_svc::hal::uart::config::Config {
            baudrate: 9600.Hz(), // initial rate is 9600 after power loss
            ..Default::default()
        },
    )?;
    let gps_init_packet: [u8; 28] = ublox::CfgPrtUartBuilder {
        portid: ublox::UartPortId::Uart1,
        reserved0: 0,
        tx_ready: 0,
        mode: ublox::UartMode::new(ublox::DataBits::Eight, ublox::Parity::None, ublox::StopBits::One),
        baud_rate: TARGET_BAUDRATE,
        in_proto_mask: ublox::InProtoMask::all(),
        out_proto_mask: ublox::OutProtoMask::UBLOX,
        flags: 0,
        reserved5: 0,
    }.into_packet_bytes();
    gps_uart.write(&gps_init_packet)?;
    let nav5_init_packet = ublox::CfgNav5Builder {
        mask: ublox::CfgNav5Params::DYN,
        dyn_model: ublox::CfgNav5DynModel::Automotive,
        ..Default::default()
    }.into_packet_bytes();
    gps_uart.write(&nav5_init_packet)?;
    gps_uart.wait_tx_done(1000)?;
    gps_uart.change_baudrate(TARGET_BAUDRATE.Hz())?;

    let gps_rate_init = ublox::CfgRateBuilder {
        measure_rate_ms: 500,
        nav_rate: 1,
        time_ref: ublox::AlignmentToReferenceTime::Utc,
    }.into_packet_bytes();
    gps_uart.write(&gps_rate_init)?;
    let nav_pvt_rate_init = ublox::CfgMsgSinglePortBuilder::set_rate_for::<ublox::NavPvt>(1).into_packet_bytes();
    gps_uart.write(&nav_pvt_rate_init)?;

    let mut gps_parser = ublox::Parser::default();

    loop {
        let batt_voltage = psu.get_battery_voltage()?;
        let charging = psu.get_charging()?;
        //info!("Battery voltage: {}V, Charging: {}", batt_voltage, charging);
        let vbus_voltage = psu.get_vbus_voltage()?;
        let vbus_current = psu.get_vbus_current()?;
        //info!("Vbus voltage: {}V, Vbus current: {}A", vbus_voltage, vbus_current);

        let mut buf = [0u8; 512];
        let len = gps_uart.read(&mut buf, 50)?;
        let mut it = gps_parser.consume(&buf[..len]);
        loop {
            match it.next() {
                Some(Ok(packet)) => {
                    info!("GPS packet: {:#?}", packet);
                }
                Some(Err(e)) => {
                    error!("GPS parser error: {:?}", e);
                }
                None => {
                    // done processing current buf
                    break;
                }
            }
        }
    }

    //let tasks: Vec<JoinHandle<()>> = Vec::new();

    return Ok(());

    // Wait for tasks to exit
    //for task in tasks {
    //    task.join().unwrap();
    //}

    //error!("All tasks have exited, restarting...");

    //unsafe {
    //    esp_restart();
    //}
}

