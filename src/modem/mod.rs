use core::str::FromStr;

use alloc::sync::Arc;
use anyhow::anyhow;
use atat::{asynch::AtatClient, AtatIngress, DefaultDigester, Ingress, ResponseSlot, UrcChannel};
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, TimeoutError, Timer, WithTimeout};
use esp_hal::{
    gpio::{AnyPin, Input, Output},
    peripherals::UART2,
    uart::{Uart, UartRx, UartTx},
    Async,
};
use log::info;
use static_cell::StaticCell;

use crate::modem::common::general::urc::PinStatus;

mod common;

type ModemUart = Uart<'static, UART2, Async>;
type ModemRx = UartRx<'static, UART2, Async>;
type ModemTx = UartTx<'static, UART2, Async>;
type ModemClient = atat::asynch::Client<'static, ModemTx, INGRESS_BUF_SIZE>;

#[derive(Debug, Clone, Default)]
enum ModemState {
    #[default]
    Uninitialized,
    Initializing,
    HardwareReady,
    SimReady,
    SimOnline,
    GprsOnline,
}

pub struct ModemInterface {
    pub dtr: Output<'static, AnyPin>,
    pub ri: Input<'static, AnyPin>,
    pub pwrkey: Output<'static, AnyPin>,
    pub power_on: Output<'static, AnyPin>,
    client: ModemClient,
}

const INGRESS_BUF_SIZE: usize = 1024;
const URC_CAPACITY: usize = 1024;
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

        let mut urc_subcriber = URC_CHANNEL
            .subscribe()
            .expect("Failed to subscribe to URC channel");

        int.client
            .send(&common::general::EnableGetLocalTimestamp { enable: 1 })
            .await
            .expect("Failed to enable local timestamp reporting");

        self.dump_info(&mut int)
            .await
            .map_err(|e| log::error!("Failed to dump info: {:?}", e))
            .ok();

        loop {
            let future = urc_subcriber
                .next_message()
                .with_timeout(Duration::from_secs(3));
            if let Ok(embassy_sync::pubsub::WaitResult::Message(urc)) = future.await {
                self.handle_urc(&mut int, urc)
                    .await
                    .map_err(|e| log::error!("Failed to handle URC: {:?}", e))
                    .ok();
            }

            // self.dump_status(&mut int)
            //     .await
            //     .map_err(|e| log::error!("Failed to dump status: {:?}", e)).ok();

            {
                let mut state_guard = self.state.lock().await;
                let pending_state = state_guard.clone();

                if let Ok(next_state) = self.handle_state_change(&mut int, pending_state).await {
                    *state_guard = next_state;
                }
            }
        }
    }

    async fn handle_urc(&self, int: &mut ModemInterface, urc: common::Urc) -> anyhow::Result<()> {
        match urc {
            common::Urc::IndicatorEvent(e) => {
                log::info!("Indicator event: {:?}", e);
                let mut state = self.state.lock().await;
                if let ModemState::SimReady = *state {
                    if e.rdy == 0 {
                        *state = ModemState::SimOnline;
                    }
                }
            }
            common::Urc::PinStatus(PinStatus { status }) => {
                log::info!("PIN status: {}", status);
                match status.as_str() {
                    "READY" => {
                        info!("SIM card ready");
                        let mut state = self.state.lock().await;
                        if let ModemState::HardwareReady = *state {
                            *state = ModemState::SimReady;
                        }
                    }
                    "NOT READY" => info!("SIM card not ready"),
                    "NOT INSERTED" => info!("SIM card not inserted"),
                    _ => anyhow::bail!("Unknown PIN status: {}", status),
                }
            }
            urc => log::info!("Received URC: {:?}", urc),
        }

        Ok(())
    }

    async fn handle_state_change(
        &self,
        int: &mut ModemInterface,
        current_state: ModemState,
    ) -> anyhow::Result<ModemState> {
        match current_state {
            ModemState::SimOnline => {
                int.client
                    .send(&common::cip::SetPDPContext {
                        cid: 1,
                        pdp_type: heapless::String::from_str("IP").unwrap(),
                        apn: heapless::String::from_str(env!("APN"))
                            .map_err(|_| anyhow!("Failed to create APN string"))?,
                    })
                    .await
                    .map_err(|e| anyhow!("Failed to set PDP context: {:?}", e))?;

                int.client
                    .send(&common::cip::AttachToGprsService { attach: 1 })
                    .await
                    .map_err(|e| anyhow!("Failed to attach to GPRS service: {:?}", e))?;

                int.client
                    .send(&common::cip::StartDataConnection {
                        cid: 1,
                        l2p: heapless::String::from_str("PPP").unwrap(),
                    })
                    .await
                    .map_err(|e| anyhow!("Failed to bring up wireless connection: {:?}", e))?;

                Ok(ModemState::GprsOnline)
            }
            _ => Ok(current_state),
        }
    }

    async fn dump_info(&self, int: &mut ModemInterface) -> anyhow::Result<()> {
        let manufacturer = int
            .client
            .send(&common::general::GetManufacturerInfo)
            .await
            .map_err(|e| anyhow!("Failed to get manufacturer info: {e:?}"))?;
        log::info!("Modem Manufacturer: {}", manufacturer.id);

        let model = int
            .client
            .send(&common::general::GetModelInfo)
            .await
            .map_err(|e| anyhow!("Failed to get model info: {e:?}"))?;
        log::info!("Modem model: {}", model.id);

        Ok(())
    }

    async fn dump_status(&self, int: &mut ModemInterface) -> anyhow::Result<()> {
        match int.client.send(&common::general::GetSignalQuality).await {
            Ok(report) => log::info!("Signal quality: {:?}", report),
            Err(e) => log::error!("Failed to get signal quality: {e:?}"),
        }

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
