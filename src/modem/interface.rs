use atat::asynch::AtatClient as _;
use atat::{
    nom::FindSubstring, AtatIngress as _, DefaultDigester, Ingress, ResponseSlot, UrcChannel,
};
use embassy_sync::signal::Signal;
use embassy_time::Instant;
use static_cell::make_static;

use super::*;

pub const INGRESS_BUF_SIZE: usize = 64;
pub const URC_CAPACITY: usize = 8;
pub const URC_SUBSCRIBERS: usize = 3;
const COMMAND_PIPE_SIZE: usize = 64;
const DATA_PIPE_SIZE: usize = 2048;
const UART_CMD_BUF_SIZE: usize = COMMAND_PIPE_SIZE;
const UART_DATA_BUF_SIZE: usize = DATA_PIPE_SIZE;

pub static URC_CHANNEL: UrcChannel<Urc, URC_CAPACITY, URC_SUBSCRIBERS> = UrcChannel::new();

pub type ModemUart = Uart<'static, UART2, Async>;
type CommandPipe = embassy_sync::pipe::Pipe<CriticalSectionRawMutex, COMMAND_PIPE_SIZE>;
type CommandRx = embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, COMMAND_PIPE_SIZE>;
type CommandTx = embassy_sync::pipe::Writer<'static, CriticalSectionRawMutex, COMMAND_PIPE_SIZE>;

pub type DataPipe = embassy_sync::pipe::Pipe<CriticalSectionRawMutex, DATA_PIPE_SIZE>;
pub type DataRx = embassy_sync::pipe::Reader<'static, CriticalSectionRawMutex, DATA_PIPE_SIZE>;
pub type DataTx = embassy_sync::pipe::Writer<'static, CriticalSectionRawMutex, DATA_PIPE_SIZE>;

type CommandClient = atat::asynch::Client<'static, CommandTx, INGRESS_BUF_SIZE>;

const CONNECT: &str = "CONNECT\r\n";
const OK: &[u8] = b"OK\r\n";

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
    mode_signal: Arc<Signal<CriticalSectionRawMutex, Mode>>,
    mode_result_signal: Arc<Signal<CriticalSectionRawMutex, anyhow::Result<()>>>,
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

        let mode_signal = Arc::new(Signal::new());
        let mode_result_signal = Arc::new(Signal::new());
        let pins = Arc::new(Mutex::new(Pins {
            dtr,
            pwrkey,
            power_on,
        }));
        spawner
            .spawn(io_task(
                uart,
                mode_signal.clone(),
                mode_result_signal.clone(),
                cmd_tx_reader,
                cmd_rx_writer,
                data_tx_reader,
                data_rx_writer,
                pins.clone(),
                ri,
            ))
            .map_err(|e| anyhow!("Failed to spawn IO task: {e:?}"))?;

        Ok((
            Self {
                pins,
                cmd_client: Arc::new(Mutex::new(cmd_client)),
                mode_signal,
                mode_result_signal,
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
        self.mode_result_signal.reset();
        self.mode_signal.signal(Mode::Data);
        self.mode_result_signal.wait().await
    }

    pub async fn command_mode(&self) -> anyhow::Result<()> {
        self.mode_result_signal.reset();
        self.mode_signal.signal(Mode::Command);
        self.mode_result_signal.wait().await
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
async fn io_task(
    mut uart: ModemUart,
    mode_signal: Arc<Signal<CriticalSectionRawMutex, Mode>>,
    mode_result_signal: Arc<Signal<CriticalSectionRawMutex, anyhow::Result<()>>>,
    cmd_tx_reader: CommandRx,
    mut cmd_rx_writer: CommandTx,
    data_tx_reader: DataRx,
    mut data_rx_writer: DataTx,
    pins: Arc<Mutex<CriticalSectionRawMutex, Pins>>,
    mut ri: Input<'static, AnyPin>,
) -> ! {
    use embassy_futures::select::{Either3, Either4};
    use embedded_io_async::{Read, Write};
    let mut mode = Mode::Command;
    let mut ri_triggered_at: Option<Instant> = None;
    loop {
        match mode {
            Mode::Command => {
                if let Some(at) = ri_triggered_at.take() {
                    if at.elapsed() > Duration::from_secs(5) {
                        log::debug!("MODEM: ring indicator triggered switch timeout, going back to data mode");
                        let mut pins = pins.lock().await;
                        let mut buf = [0; UART_CMD_BUF_SIZE];
                        if let Err(e) = enter_data_mode(
                            &mut pins,
                            &mut uart,
                            &mut buf,
                            &mut cmd_rx_writer,
                            &mut mode,
                        )
                        .await
                        {
                            log::error!("MODEM: Failed to enter data mode: {:?}", e);
                        };
                    }
                }
                let mut uart_buf: [u8; UART_CMD_BUF_SIZE] = [0; UART_CMD_BUF_SIZE];
                let mut cmd_buf: [u8; COMMAND_PIPE_SIZE] = [0; COMMAND_PIPE_SIZE];
                let result = embassy_futures::select::select3(
                    cmd_tx_reader.read(&mut cmd_buf),
                    uart.read(&mut uart_buf),
                    mode_signal.wait(),
                )
                .await;
                match result {
                    // CMD TX
                    Either3::First(len) => {
                        if len == 0 {
                            continue;
                        }

                        uart.write_all(&cmd_buf[..len])
                            .await
                            .map_err(|e| log::error!("UART write error: {:?}", e))
                            .ok();
                    }
                    // UART RX
                    Either3::Second(Ok(len)) => {
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
                            mode = Mode::Data;
                        } else {
                            cmd_rx_writer
                                .write_all(&uart_buf[..len])
                                .await
                                .map_err(|e| log::error!("Pipe write error: {:?}", e))
                                .ok();
                        }
                    }
                    // UART RX Error
                    Either3::Second(Err(e)) => log::error!("UART read error: {:?}", e),
                    Either3::Third(Mode::Command) => {
                        mode_signal.reset();
                        mode_result_signal.signal(Ok(()));
                    }
                    Either3::Third(Mode::Data) => {
                        mode_signal.reset();
                        let mut pins = pins.lock().await;
                        let result = enter_data_mode(
                            &mut pins,
                            &mut uart,
                            &mut uart_buf,
                            &mut cmd_rx_writer,
                            &mut mode,
                        )
                        .await;
                        mode_result_signal.signal(result);
                    }
                }
            }
            Mode::Data => {
                let mut uart_buf: [u8; UART_DATA_BUF_SIZE] = [0; UART_DATA_BUF_SIZE];
                let mut data_buf: [u8; DATA_PIPE_SIZE] = [0; DATA_PIPE_SIZE];
                let result = embassy_futures::select::select4(
                    data_tx_reader.read(&mut data_buf),
                    uart.read(&mut uart_buf),
                    mode_signal.wait(),
                    ri.wait_for_low(),
                )
                .await;
                match result {
                    // DATA TX
                    Either4::First(len) => {
                        if len == 0 {
                            continue;
                        }

                        uart.write_all(&data_buf[..len])
                            .await
                            .map_err(|e| log::error!("UART write error: {:?}", e))
                            .ok();
                    }
                    // UART RX
                    Either4::Second(Ok(len)) => {
                        if len == 0 {
                            continue;
                        }

                        data_rx_writer
                            .write_all(&uart_buf[..len])
                            .await
                            .map_err(|e| log::error!("Pipe write error: {:?}", e))
                            .ok();
                    }
                    Either4::Second(Err(e)) => log::error!("UART read error: {:?}", e),
                    Either4::Third(Mode::Data) => {
                        mode_signal.reset();
                        mode_result_signal.signal(Ok(()));
                    }
                    Either4::Third(Mode::Command) => {
                        mode_signal.reset();
                        let mut pins = pins.lock().await;
                        let result = enter_command_mode(
                            &mut pins,
                            &mut uart,
                            &mut uart_buf,
                            &mut data_rx_writer,
                            &mut mode,
                        )
                        .await;
                        mode_result_signal.signal(result);
                    }
                    // RI pin low
                    Either4::Fourth(_) => {
                        log::info!("MODEM: ring indicator triggered, entering command mode");
                        ri_triggered_at = Some(Instant::now());
                        ri.wait_for_high().await;
                        let mut pins = pins.lock().await;
                        if let Err(e) = enter_command_mode(
                            &mut pins,
                            &mut uart,
                            &mut uart_buf,
                            &mut data_rx_writer,
                            &mut mode,
                        )
                        .await
                        {
                            log::error!("Failed to enter command mode: {:?}", e)
                        };
                    }
                }
            }
        }
    }
}

fn detect_data_mode_switch(buf: &[u8]) -> Option<usize> {
    let string = core::str::from_utf8(buf).ok()?;
    string.find_substring(CONNECT).map(|i| i + CONNECT.len())
}

fn detect_command_mode_switch(buf: &[u8]) -> Option<usize> {
    for (i, window) in buf.windows(OK.len()).enumerate() {
        if window == OK {
            return Some(i);
        }
    }
    None
}

async fn enter_command_mode(
    pins: &mut Pins,
    uart: &mut ModemUart,
    uart_buf: &mut [u8],
    data_rx_writer: &mut DataTx,
    mode: &mut Mode,
) -> anyhow::Result<()> {
    use embedded_io_async::{Read, Write};

    log::debug!("MODEM: Entering command mode...");

    pins.dtr.set_low();
    embassy_time::Timer::after_millis(1200).await;
    pins.dtr.set_high();

    let mut attemts_left = 10;
    while attemts_left > 0 {
        attemts_left -= 1;
        uart.read(uart_buf)
            .await
            .map_err(|e| anyhow!("UART read error: {:?}", e))?;
        if let Some(len) = detect_command_mode_switch(uart_buf) {
            log::info!("Modem entered command mode");
            *mode = Mode::Command;

            // flush remaining data to consumer, if any
            data_rx_writer
                .write_all(&uart_buf[..len])
                .await
                .map_err(|e| anyhow!("Pipe write error: {:?}", e))?;
            return Ok(());
        }
    }
    anyhow::bail!("Failed to detect command mode switch");
}

async fn enter_data_mode(
    pins: &mut Pins,
    uart: &mut ModemUart,
    uart_buf: &mut [u8],
    cmd_rx_writer: &mut CommandTx,
    mode: &mut Mode,
) -> anyhow::Result<()> {
    use embedded_io_async::{Read, Write};

    pins.dtr.set_high();
    uart.write_all(b"ATO\r\n")
        .await
        .map_err(|e| anyhow!("UART write error: {:?}", e))?;

    let mut attempts_left = 10;
    while attempts_left > 0 {
        attempts_left -= 1;
        uart.read(uart_buf)
            .await
            .map_err(|e| anyhow!("UART read error: {:?}", e))?;
        if let Some(len) = detect_data_mode_switch(uart_buf) {
            log::info!("Modem entered data mode");
            *mode = Mode::Data;
            // flush remaining command rx before our ATO command, if any
            cmd_rx_writer
                .write_all(&uart_buf[..len - CONNECT.len()])
                .await
                .map_err(|e| anyhow!("Pipe write error: {:?}", e))?;
            return Ok(());
        }
    }
    anyhow::bail!("Failed to detect data mode switch");
}
