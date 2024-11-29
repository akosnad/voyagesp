use alloc::{sync::Arc, vec::Vec};
use anyhow::{anyhow, Result};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read as _, ReadReady as _, Write as _};
use esp_hal::{peripherals::UART1, uart::Uart, Async};
use hass_types::DeviceTrackerAttributes;
use log::{info, trace};
use ublox::{GpsFix, PacketRef};

#[derive(Default, Debug, Clone, Copy, serde::Serialize)]
pub struct GpsCoords {
    pub lat: f64,
    pub lon: f64,
    pub height: f64,
    pub horiz_accuracy: u32,
    pub satellite_count: u8,
}

impl From<ublox::NavPvtRef<'_>> for GpsCoords {
    fn from(nav: ublox::NavPvtRef<'_>) -> Self {
        Self {
            lat: nav.lat_degrees(),
            lon: nav.lon_degrees(),
            height: nav.height_meters(),
            horiz_accuracy: nav.horiz_accuracy(),
            satellite_count: nav.num_satellites(),
        }
    }
}

impl From<GpsCoords> for DeviceTrackerAttributes {
    fn from(coords: GpsCoords) -> Self {
        Self {
            longitude: coords.lon,
            latitude: coords.lat,
            // NAV-PVT reports in millimeters, convert to meters
            gps_accuracy: Some(coords.horiz_accuracy as f64 / 1000.),
        }
    }
}

#[derive(Debug, Default)]
enum GpsState {
    #[default]
    Uninitialized,
    Initializing,
    Ready,
}

type GpsUart = Uart<'static, UART1, Async>;

pub struct Gps<const BAUD: u32> {
    state: Arc<Mutex<CriticalSectionRawMutex, GpsState>>,
    uart: Arc<Mutex<CriticalSectionRawMutex, GpsUart>>,
    coords: Arc<Mutex<CriticalSectionRawMutex, Option<GpsCoords>>>,
}

impl<const BAUD: u32> Gps<BAUD> {
    pub async fn new(uart: GpsUart) -> Result<Self> {
        Ok(Self {
            state: Default::default(),
            uart: Arc::new(Mutex::new(uart)),
            coords: Arc::new(Mutex::new(Default::default())),
        })
    }

    pub async fn get_coords(&self) -> Option<GpsCoords> {
        match *self.state.lock().await {
            GpsState::Ready => *self.coords.lock().await,
            _ => None,
        }
    }

    /// Gps stack task
    /// Should be only called once
    ///
    /// # Panics
    /// Panics if called more than once
    pub async fn run(&self) -> ! {
        {
            let mut state = self.state.lock().await;
            match *state {
                GpsState::Uninitialized => {
                    let mut uart = self.uart.lock().await;
                    *state = GpsState::Initializing;
                    while let Err(e) = uart_init::<_, BAUD>(&mut uart).await {
                        log::error!("GPS initialization failed: {:?}", e);
                    }
                    *state = GpsState::Ready;
                }
                GpsState::Initializing | GpsState::Ready => panic!("GPS task already running"),
            }
        }

        let mut parser = {
            let buf = Vec::<u8>::new();
            ublox::Parser::new(buf)
        };
        let mut uart = self.uart.lock().await;

        loop {
            let mut uart_buf = [0u8; 512];
            let len = {
                let result = uart.read(&mut uart_buf).await;
                match result {
                    Ok(len) => len,
                    Err(e) => {
                        log::error!("GPS Read failed: {:?}", e);
                        continue;
                    }
                }
            };
            let mut it = parser.consume(&uart_buf[..len]);
            loop {
                match it.next() {
                    Some(Ok(PacketRef::NavPvt(nav))) => {
                        trace!("Positional data: {:?}", nav);
                        let mut data = self.coords.lock().await;
                        *data = {
                            match nav.fix_type() {
                                GpsFix::Fix2D | GpsFix::Fix3D => Some(nav.into()),
                                _ => None,
                            }
                        };
                    }
                    Some(Ok(packet)) => {
                        trace!("Received packet: {:?}", packet);
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
}

/// Initialize GPS hardware
///  
/// [reference](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)
async fn uart_init<T, const BAUD: u32>(
    uart: &mut esp_hal::uart::Uart<'_, T, Async>,
) -> anyhow::Result<()>
where
    T: esp_hal::uart::Instance,
{
    let gps_reset_packet = ublox::CfgRstBuilder {
        nav_bbr_mask: ublox::NavBbrMask::empty(), // don't clear Battery Backed RAM
        reset_mode: ublox::ResetMode::ControlledSoftwareReset,
        reserved1: 0,
    }
    .into_packet_bytes();
    info!("Resetting GPS...");
    uart_command(uart, &gps_reset_packet, true, false).await?;

    // wait for GPS to boot up
    Timer::after(Duration::from_millis(6000)).await;

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
    uart_command(uart, &gps_init_packet, false, true).await?;

    //info!("Changing GPS baud rate to {}", GPS_TARGET_BAUD);
    //uart.change_baud(GPS_TARGET_BAUD, ClockSource::Apb, &clocks);

    let nav5_init_packet = ublox::CfgNav5Builder {
        mask: ublox::CfgNav5Params::DYN,
        dyn_model: ublox::CfgNav5DynModel::Automotive,
        ..Default::default()
    }
    .into_packet_bytes();
    info!("Setting GPS to automotive mode");
    uart_command(uart, &nav5_init_packet, false, true).await?;

    let gps_rate_init = ublox::CfgRateBuilder {
        measure_rate_ms: 1000,
        nav_rate: 1,
        time_ref: ublox::AlignmentToReferenceTime::Utc,
    }
    .into_packet_bytes();
    info!("Setting GPS rate to 1s");
    uart_command(uart, &gps_rate_init, false, true).await?;

    let nav_pvt_rate_init =
        ublox::CfgMsgSinglePortBuilder::set_rate_for::<ublox::NavPvt>(1).into_packet_bytes();
    info!("Setting GPS NAV-PVT rate to 1 every measurement cycle");
    uart_command(uart, &nav_pvt_rate_init, false, true).await?;

    info!("GPS initialized");
    Ok(())
}

async fn uart_command<T>(
    uart: &mut esp_hal::uart::Uart<'_, T, Async>,
    cmd: &[u8],
    flush: bool,
    expect_ack: bool,
) -> anyhow::Result<()>
where
    T: esp_hal::uart::Instance,
{
    uart.write_all(cmd)
        .await
        .map_err(|e| anyhow!("Failed to write GPS command: {:?}", e))?;

    if flush {
        uart.flush()
            .await
            .map_err(|e| anyhow!("Failed to flush GPS UART: {:?}", e))?;
    }

    if expect_ack {
        uart_expect_ack(uart)
            .await
            .map_err(|e| anyhow!("Failed to receive GPS ACK: {:?}", e))?;
    }

    Ok(())
}

async fn uart_expect_ack<T>(uart: &mut esp_hal::uart::Uart<'_, T, Async>) -> anyhow::Result<()>
where
    T: esp_hal::uart::Instance,
{
    const BUF_SIZE: usize = 128;
    const MAX_ITERATIONS: usize = 500;
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
            Timer::after(Duration::from_millis(10)).await;
            i += 1;
            continue;
        }

        let mut buf: [u8; BUF_SIZE] = [0; BUF_SIZE];
        let Ok(len) = uart.read(&mut buf).await else {
            i += 1;
            continue;
        };
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
