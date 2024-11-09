use alloc::{sync::Arc, vec::Vec};
use anyhow::{anyhow, Result};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read as _, ReadReady as _, Write as _};
use esp_hal::{peripherals::UART1, uart::Uart, Async};
use log::{info, trace};
use ublox::{GpsFix, PacketRef};

#[derive(Default, Debug, Clone, Copy, serde::Serialize)]
pub struct GpsCoords {
    lat: f64,
    lon: f64,
    height: f64,
    horiz_accuracy: u32,
}

impl From<ublox::NavPvtRef<'_>> for GpsCoords {
    fn from(nav: ublox::NavPvtRef<'_>) -> Self {
        Self {
            lat: nav.lat_degrees(),
            lon: nav.lon_degrees(),
            height: nav.height_meters(),
            horiz_accuracy: nav.horiz_accuracy(),
        }
    }
}

type GpsUart = Uart<'static, UART1, Async>;

pub struct Gps<const BAUD: u32> {
    uart: Arc<Mutex<CriticalSectionRawMutex, GpsUart>>,
    coords: Arc<Mutex<CriticalSectionRawMutex, Option<GpsCoords>>>,
}

impl<const BAUD: u32> Gps<BAUD> {
    pub async fn new(mut uart: Uart<'static, UART1, Async>) -> Result<Self> {
        uart_init::<_, BAUD>(&mut uart).await?;

        Ok(Self {
            uart: Arc::new(Mutex::new(uart)),
            coords: Arc::new(Mutex::new(Default::default())),
        })
    }

    pub async fn get_coords(&self) -> Option<GpsCoords> {
        *self.coords.lock().await
    }

    pub async fn run(&self) -> ! {
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

async fn uart_init<T, const BAUD: u32>(
    uart: &mut esp_hal::uart::Uart<'_, T, Async>,
) -> anyhow::Result<()>
where
    T: esp_hal::uart::Instance,
{
    let gps_reset_packet = ublox::CfgRstBuilder {
        nav_bbr_mask: ublox::NavBbrMask::all(),
        reset_mode: ublox::ResetMode::ControlledSoftwareReset,
        reserved1: 0,
    }
    .into_packet_bytes();
    info!("Resetting GPS...");
    uart.write_all(&gps_reset_packet)
        .await
        .map_err(|e| anyhow::anyhow!("Failed to write GPS reset packet: {:?}", e))?;
    uart.flush().await.expect("Failed to flush GPS writer");
    uart.flush_tx().expect("Failed to flush GPS TX");

    // wait for GPS to reset
    Timer::after(Duration::from_millis(1000)).await;

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
    uart.write_all(&gps_init_packet)
        .await
        .map_err(|e| anyhow!("Failed to write GPS init packet: {:?}", e))?;
    log::trace!("written init packet");
    uart_expect_ack(uart)
        .await
        .map_err(|e| anyhow!("Failed to receive GPS ACK: {:?}", e))?;
    log::trace!("received ack");

    //info!("Changing GPS baud rate to {}", GPS_TARGET_BAUD);
    //uart.change_baud(GPS_TARGET_BAUD, ClockSource::Apb, &clocks);

    let nav5_init_packet = ublox::CfgNav5Builder {
        mask: ublox::CfgNav5Params::DYN,
        dyn_model: ublox::CfgNav5DynModel::Automotive,
        ..Default::default()
    }
    .into_packet_bytes();
    info!("Setting GPS to automotive mode");
    uart.write_all(&nav5_init_packet)
        .await
        .map_err(|e| anyhow!("Failed to write NAV5 init packet: {:?}", e))?;
    uart_expect_ack(uart)
        .await
        .map_err(|_| anyhow!("Failed to receive GPS ACK"))?;

    let gps_rate_init = ublox::CfgRateBuilder {
        measure_rate_ms: 500,
        nav_rate: 1,
        time_ref: ublox::AlignmentToReferenceTime::Utc,
    }
    .into_packet_bytes();
    info!("Setting GPS rate to 500ms");
    uart.write_all(&gps_rate_init)
        .await
        .map_err(|e| anyhow!("Failed to write GPS rate init packet: {:?}", e))?;
    uart_expect_ack(uart)
        .await
        .map_err(|_| anyhow!("Failed to receive GPS ACK"))?;

    let nav_pvt_rate_init =
        ublox::CfgMsgSinglePortBuilder::set_rate_for::<ublox::NavPvt>(1).into_packet_bytes();
    info!("Setting NAV-PVT rate to 1 every measurement cycle");
    uart.write_all(&nav_pvt_rate_init)
        .await
        .map_err(|e| anyhow!("Failed to write NAV-PVT rate init packet: {:?}", e))?;
    uart_expect_ack(uart)
        .await
        .map_err(|_| anyhow!("Failed to receive GPS ACK"))?;

    info!("GPS initialized");
    Ok(())
}

async fn uart_expect_ack<T>(uart: &mut esp_hal::uart::Uart<'_, T, Async>) -> anyhow::Result<()>
where
    T: esp_hal::uart::Instance,
{
    const BUF_SIZE: usize = 128;
    const MAX_ITERATIONS: usize = 1000;
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
