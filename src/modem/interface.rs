use atat::asynch::AtatClient as _;
use atat::{
    nom::FindSubstring, AtatIngress as _, DefaultDigester, Ingress, ResponseSlot, UrcChannel,
};
use static_cell::make_static;

use super::*;

pub const INGRESS_BUF_SIZE: usize = 64;
pub const URC_CAPACITY: usize = 8;
pub const URC_SUBSCRIBERS: usize = 3;
const COMMAND_PIPE_SIZE: usize = 16;
const DATA_PIPE_SIZE: usize = 32;

pub static URC_CHANNEL: UrcChannel<Urc, URC_CAPACITY, URC_SUBSCRIBERS> = UrcChannel::new();

pub type ModemUart = Uart<'static, UART2, Async>;
type CommandPipe = embassy_sync::pipe::Pipe<CriticalSectionRawMutex, COMMAND_PIPE_SIZE>;
type CommandRx = embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, COMMAND_PIPE_SIZE>;
type CommandTx = embassy_sync::pipe::Writer<'static, CriticalSectionRawMutex, COMMAND_PIPE_SIZE>;

pub type DataPipe = embassy_sync::pipe::Pipe<CriticalSectionRawMutex, DATA_PIPE_SIZE>;
pub type DataRx = embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, DATA_PIPE_SIZE>;
pub type DataTx = embassy_sync::pipe::Writer<'static, CriticalSectionRawMutex, DATA_PIPE_SIZE>;

type CommandClient = atat::asynch::Client<'static, CommandTx, INGRESS_BUF_SIZE>;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
enum Mode {
    #[default]
    Command,
    Data,
}

struct Pins {
    pub dtr: Output<'static, AnyPin>,
    pub pwrkey: Output<'static, AnyPin>,
    pub power_on: Output<'static, AnyPin>,
}

pub struct ModemInterface {
    pins: Arc<Mutex<CriticalSectionRawMutex, Pins>>,
    cmd_client: Arc<Mutex<CriticalSectionRawMutex, CommandClient>>,
    mode: Arc<Mutex<CriticalSectionRawMutex, Mode>>,
}

impl ModemInterface {
    pub async fn new(
        spawner: &embassy_executor::Spawner,
        dtr: Output<'static, AnyPin>,
        ri: Input<'static, AnyPin>,
        pwrkey: Output<'static, AnyPin>,
        power_on: Output<'static, AnyPin>,
        uart: ModemUart,
    ) -> anyhow::Result<(Self, DataTx, DataRx)> {
        let (cmd_tx_reader, cmd_tx_writer) = {
            let cmd_tx = make_static!(CommandPipe::new());
            cmd_tx.split()
        };
        let (cmd_rx_reader, cmd_rx_writer) = {
            let cmd_rx = make_static!(CommandPipe::new());
            cmd_rx.split()
        };

        let (data_tx_reader, data_tx_writer) = {
            let data_tx = make_static!(DataPipe::new());
            data_tx.split()
        };
        let (data_rx_reader, data_rx_writer) = {
            let data_rx = make_static!(DataPipe::new());
            data_rx.split()
        };

        static INGRESS_BUF: StaticCell<[u8; INGRESS_BUF_SIZE]> = StaticCell::new();
        let mut ingress = Ingress::new(
            DefaultDigester::<Urc>::new(),
            INGRESS_BUF.init([0; INGRESS_BUF_SIZE]),
            &RES_SLOT,
            &URC_CHANNEL,
        );
        ingress.clear();
        spawner
            .spawn(cmd_ingress_task(ingress, cmd_rx_reader))
            .map_err(|e| anyhow!("Failed to spawn ingress task: {e:?}"))?;

        static RES_SLOT: ResponseSlot<INGRESS_BUF_SIZE> = ResponseSlot::new();
        static CLIENT_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
        let cmd_client = CommandClient::new(
            cmd_tx_writer,
            &RES_SLOT,
            CLIENT_BUF.init([0u8; 1024]),
            atat::Config::default(),
        );

        spawner
            .spawn(ri_task(ri))
            .map_err(|e| anyhow!("Failed to spawn RI task: {e:?}"))?;

        let mode: Arc<Mutex<CriticalSectionRawMutex, Mode>> = Default::default();
        spawner
            .spawn(io_task(
                uart,
                mode.clone(),
                cmd_tx_reader,
                cmd_rx_writer,
                data_tx_reader,
                data_rx_writer,
            ))
            .map_err(|e| anyhow!("Failed to spawn IO task: {e:?}"))?;

        Ok((
            Self {
                pins: Arc::new(Mutex::new(Pins {
                    dtr,
                    pwrkey,
                    power_on,
                })),
                cmd_client: Arc::new(Mutex::new(cmd_client)),
                mode,
            },
            data_tx_writer,
            data_rx_reader,
        ))
    }

    /// Initialize Modem hardware
    ///
    /// [reference](https://microchip.ua/simcom/2G/SIM800%20Series_AT%20Command%20Manual_V1.12.pdf)
    pub async fn init_hardware(&self) -> anyhow::Result<()> {
        let mut cmd_client = self.cmd_client.lock().await;
        let mut pins = self.pins.lock().await;

        info!("Powering on modem...");

        pins.dtr.set_low();

        pins.power_on.set_low();
        Timer::after(Duration::from_millis(100)).await;

        pins.power_on.set_high();
        pins.pwrkey.set_high();
        Timer::after(Duration::from_millis(100)).await;
        pins.pwrkey.set_low();
        Timer::after(Duration::from_millis(1_000)).await;
        pins.pwrkey.set_high();

        info!("Modem powered on, initializing...");
        Timer::after(Duration::from_millis(5_000)).await;

        cmd_client
            .send(&lib::at::AT)
            .await
            .map_err(|e| anyhow::anyhow!("Failed to send AT command: {:?}", e))?;

        cmd_client
            .send(&lib::at::general::EnableGetLocalTimestamp { enable: 1 })
            .await
            .map_err(|e| anyhow::anyhow!("Failed to enable local timestamp: {:?}", e))?;

        cmd_client
            .send(&lib::at::general::DTRControl { dtr: 1 })
            .await
            .map_err(|e| anyhow::anyhow!("Failed to set DTR handling: {:?}", e))?;

        info!("Modem hardware initialized");
        Ok(())
    }

    pub async fn data_mode(&self) -> anyhow::Result<()> {
        let mut mode = self.mode.lock().await;
        match *mode {
            Mode::Data => Ok(()),
            Mode::Command => {
                let mut pins = self.pins.lock().await;
                let mut cmd_client = self.cmd_client.lock().await;

                pins.dtr.set_high();
                cmd_client
                    .send(&lib::at::general::EnterDataMode)
                    .await
                    .map_err(|e| anyhow!("Failed to send ATO command: {:?}", e))?;
                *mode = Mode::Data;
                log::info!("Modem entered data mode");
                Ok(())
            }
        }
    }

    pub async fn command_mode(&self) -> anyhow::Result<()> {
        let mut mode = self.mode.lock().await;
        match *mode {
            Mode::Command => Ok(()),
            Mode::Data => {
                let mut pins = self.pins.lock().await;
                pins.dtr.set_low();
                log::info!("Modem entered command mode");
                *mode = Mode::Command;

                embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
                pins.dtr.set_high();

                Ok(())
            }
        }
    }

    pub async fn command<Cmd: atat::AtatCmd>(&self, cmd: &Cmd) -> anyhow::Result<Cmd::Response> {
        let mut cmd_client = self.cmd_client.lock().await;
        cmd_client
            .send(cmd)
            .await
            .map_err(|e| anyhow!("Failed to send command: {:?}", e))
    }
}

#[task]
async fn cmd_ingress_task(
    mut ingress: Ingress<
        'static,
        DefaultDigester<Urc>,
        Urc,
        INGRESS_BUF_SIZE,
        URC_CAPACITY,
        URC_SUBSCRIBERS,
    >,
    mut reader: CommandRx,
) {
    ingress.read_from(&mut reader).await;
}

#[task]
async fn ri_task(mut ri: Input<'static, AnyPin>) {
    loop {
        ri.wait_for_any_edge().await;
        let state = match ri.is_high() {
            true => "HIGH",
            false => "LOW",
        };
        log::debug!("MODEM RI {}", state);
    }
}

#[task]
async fn io_task(
    mut uart: ModemUart,
    mode_mutex: Arc<Mutex<CriticalSectionRawMutex, Mode>>,
    cmd_tx_reader: CommandRx,
    mut cmd_rx_writer: CommandTx,
    data_tx_reader: DataRx,
    mut data_rx_writer: DataTx,
) -> ! {
    use embassy_futures::select::Either;
    use embedded_io_async::{Read, Write};
    loop {
        let mut mode = mode_mutex.lock().await;
        match *mode {
            Mode::Command => {
                let mut uart_buf: [u8; 64] = [0; 64];
                let mut cmd_buf: [u8; COMMAND_PIPE_SIZE] = [0; COMMAND_PIPE_SIZE];
                let result = embassy_futures::select::select(
                    cmd_tx_reader.read(&mut cmd_buf),
                    uart.read(&mut uart_buf),
                )
                .await;
                match result {
                    // CMD TX
                    Either::First(len) => {
                        if len == 0 {
                            continue;
                        }

                        uart.write_all(&cmd_buf[..len])
                            .await
                            .map_err(|e| log::error!("UART write error: {:?}", e))
                            .ok();
                    }
                    // UART RX
                    Either::Second(Ok(len)) => {
                        if len == 0 {
                            continue;
                        }

                        if let Some(len) = detect_data_mode_switch(&uart_buf[..len]) {
                            log::info!("MODEM Data mode switch detected");
                            cmd_rx_writer
                                .write_all(&uart_buf[..len])
                                .await
                                .map_err(|e| log::error!("Pipe write error: {:?}", e))
                                .ok();
                            *mode = Mode::Data;
                        } else {
                            cmd_rx_writer
                                .write_all(&uart_buf[..len])
                                .await
                                .map_err(|e| log::error!("Pipe write error: {:?}", e))
                                .ok();
                        }
                    }
                    // UART RX Error
                    Either::Second(Err(e)) => log::error!("UART read error: {:?}", e),
                }
            }
            Mode::Data => {
                let mut uart_buf: [u8; 64] = [0; 64];
                let mut data_buf: [u8; DATA_PIPE_SIZE] = [0; DATA_PIPE_SIZE];
                let result = embassy_futures::select::select(
                    data_tx_reader.read(&mut data_buf),
                    uart.read(&mut uart_buf),
                )
                .await;
                match result {
                    // DATA TX
                    Either::First(len) => {
                        if len == 0 {
                            continue;
                        }

                        uart.write_all(&data_buf[..len])
                            .await
                            .map_err(|e| log::error!("UART write error: {:?}", e))
                            .ok();
                    }
                    // UART RX
                    Either::Second(Ok(len)) => {
                        if len == 0 {
                            continue;
                        }

                        data_rx_writer
                            .write_all(&uart_buf[..len])
                            .await
                            .map_err(|e| log::error!("Pipe write error: {:?}", e))
                            .ok();
                    }
                    Either::Second(Err(e)) => log::error!("UART read error: {:?}", e),
                }
            }
        }
    }
}

fn detect_data_mode_switch(buf: &[u8]) -> Option<usize> {
    let string = core::str::from_utf8(buf).ok()?;
    const CONNECT: &str = "CONNECT\r\n";
    string.find_substring(CONNECT).map(|i| i + CONNECT.len())
}
