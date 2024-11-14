use alloc::sync::Arc;
use anyhow::anyhow;
use atat::{asynch::AtatClient, AtatIngress, DefaultDigester, Ingress, ResponseSlot, UrcChannel};
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer};
use esp_hal::{
    gpio::{AnyPin, Input, Output},
    peripherals::UART2,
    uart::{Uart, UartRx, UartTx},
    Async,
};
use log::info;
use static_cell::StaticCell;

mod common;

type ModemUart = Uart<'static, UART2, Async>;
type ModemRx = UartRx<'static, UART2, Async>;
type ModemTx = UartTx<'static, UART2, Async>;
type ModemClient = atat::asynch::Client<'static, ModemTx, INGRESS_BUF_SIZE>;

#[derive(Debug, Default)]
enum ModemState {
    #[default]
    Uninitialized,
    Initializing,
    HardwareReady,
}

pub struct ModemInterface {
    pub dtr: Output<'static, AnyPin>,
    pub ri: Input<'static, AnyPin>,
    pub pwrkey: Output<'static, AnyPin>,
    pub power_on: Output<'static, AnyPin>,
    client: ModemClient,
}

const INGRESS_BUF_SIZE: usize = 1024;
const URC_CAPACITY: usize = 512;
const URC_SUBSCRIBERS: usize = 3;

static INGRESS_BUF: StaticCell<[u8; INGRESS_BUF_SIZE]> = StaticCell::new();
static RES_SLOT: ResponseSlot<INGRESS_BUF_SIZE> = ResponseSlot::new();
static URC_CHANNEL: UrcChannel<common::Urc, URC_CAPACITY, URC_SUBSCRIBERS> = UrcChannel::new();
static CLIENT_BUF: StaticCell<[u8; 1024]> = StaticCell::new();

pub struct Modem {
    interface: Arc<Mutex<CriticalSectionRawMutex, ModemInterface>>,
    state: Arc<Mutex<CriticalSectionRawMutex, ModemState>>,
}

impl Modem {
    pub async fn new(
        spawner: &embassy_executor::Spawner,
        uart: ModemUart,
        dtr: Output<'static, AnyPin>,
        ri: Input<'static, AnyPin>,
        pwrkey: Output<'static, AnyPin>,
        power_on: Output<'static, AnyPin>,
    ) -> anyhow::Result<Self> {
        let (reader, writer) = uart.split();
        init_ingress(spawner, reader).await?;

        let client = ModemClient::new(
            writer,
            &RES_SLOT,
            CLIENT_BUF.init([0u8; 1024]),
            atat::Config::default(),
        );

        Ok(Self {
            interface: Arc::new(Mutex::new(ModemInterface {
                dtr,
                ri,
                pwrkey,
                power_on,
                client,
            })),
            state: Default::default(),
        })
    }

    /// Modem stack task
    /// Should be only called once
    ///
    /// # Panics
    /// Panics if called more than once
    pub async fn run(&self) -> ! {
        {
            let mut state = self.state.lock().await;
            match *state {
                ModemState::Uninitialized => {
                    let mut interface = self.interface.lock().await;
                    *state = ModemState::Initializing;
                    while let Err(e) = modem_init(&mut interface).await {
                        log::error!("Failed to initialize modem: {:?}", e);
                    }
                    *state = ModemState::HardwareReady;
                }
                _ => panic!("Modem task already running"),
            }
        }
        let mut int = self.interface.lock().await;

        let at = int.client.send(&common::AT).await;
        log::info!("Initial AT ping response: {:?}", at);

        self.dump_info(&mut int)
            .await
            .expect("Failed to dump modem info");

        let mut urc_subcriber = URC_CHANNEL
            .subscribe()
            .expect("Failed to subscribe to URC channel");
        loop {
            let urc = urc_subcriber.next_message().await;
            log::info!("Received URC: {:?}", urc);
        }
    }

    async fn dump_info(&self, int: &mut ModemInterface) -> anyhow::Result<()> {
        let manufacturer = int
            .client
            .send(&common::general::GetManufacturerInfo)
            .await
            .map_err(|e| anyhow!("Failed to get manufacturer info: {e:?}"))?;
        let parsed = core::str::from_utf8(&manufacturer.id)
            .map_err(|e| anyhow!("Failed to parse manufacturer info: {e:?}"))?;
        log::info!("Modem Manufacturer: {}", parsed);

        Ok(())
    }
}

/// Initialize Modem hardware
///
/// [reference](https://microchip.ua/simcom/2G/SIM800%20Series_AT%20Command%20Manual_V1.12.pdf)
pub async fn modem_init(int: &mut ModemInterface) -> anyhow::Result<()> {
    info!("Powering on modem...");

    int.power_on.set_low();
    Timer::after(Duration::from_millis(100)).await;

    int.power_on.set_high();
    int.pwrkey.set_high();
    Timer::after(Duration::from_millis(100)).await;
    int.pwrkey.set_low();
    Timer::after(Duration::from_millis(1_000)).await;
    int.pwrkey.set_high();

    info!("Modem powered on, initializing...");
    Timer::after(Duration::from_millis(5_000)).await;

    int.client
        .send(&common::AT)
        .await
        .map_err(|e| anyhow::anyhow!("Failed to send AT command: {:?}", e))?;

    info!("Modem hardware initialized");
    Ok(())
}

async fn init_ingress(
    spawner: &embassy_executor::Spawner,
    reader: UartRx<'static, UART2, Async>,
) -> anyhow::Result<()> {
    let mut ingress = Ingress::new(
        DefaultDigester::<common::Urc>::new(),
        INGRESS_BUF.init([0; INGRESS_BUF_SIZE]),
        &RES_SLOT,
        &URC_CHANNEL,
    );

    ingress.clear();

    spawner
        .spawn(ingress_task(ingress, reader))
        .expect("Failed to spawn ingress task");

    Ok(())
}

#[task]
async fn ingress_task(
    mut ingress: Ingress<
        'static,
        DefaultDigester<common::Urc>,
        common::Urc,
        INGRESS_BUF_SIZE,
        URC_CAPACITY,
        URC_SUBSCRIBERS,
    >,
    mut reader: UartRx<'static, UART2, Async>,
) {
    ingress.read_from(&mut reader).await;
}
