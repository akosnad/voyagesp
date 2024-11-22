use core::str::FromStr;

use alloc::sync::Arc;
use anyhow::anyhow;
use atat::{asynch::AtatClient, AtatIngress, DefaultDigester, Ingress, ResponseSlot, UrcChannel};
use embassy_executor::task;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Timer, WithTimeout};
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
mod interface;

use interface::ModemInterface;

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

pub struct Modem {
    interface: Arc<Mutex<CriticalSectionRawMutex, ModemInterface>>,
    state: Arc<Mutex<CriticalSectionRawMutex, ModemState>>,
}

impl Modem {
    pub async fn new(
        spawner: &embassy_executor::Spawner,
        uart: interface::ModemUart,
        dtr: Output<'static, AnyPin>,
        ri: Input<'static, AnyPin>,
        pwrkey: Output<'static, AnyPin>,
        power_on: Output<'static, AnyPin>,
    ) -> anyhow::Result<Self> {
        let interface = ModemInterface::new(spawner, dtr, ri, pwrkey, power_on, uart).await?;

        Ok(Self {
            interface: Arc::new(Mutex::new(interface)),
            state: Default::default(),
        })
    }

    /// Modem stack task
    /// Should be only called once
    ///
    /// # Panics
    /// Panics if called more than once
    pub async fn run(&self) -> ! {
        let mut urc_subcriber = interface::URC_CHANNEL
            .subscribe()
            .expect("Failed to subscribe to URC channel");

        {
            let mut state = self.state.lock().await;
            match *state {
                ModemState::Uninitialized => {
                    let interface = self.interface.lock().await;
                    *state = ModemState::Initializing;
                    while let Err(e) = interface.init_hardware().await {
                        log::error!("Failed to initialize modem: {:?}", e);
                    }
                    *state = ModemState::HardwareReady;
                }
                _ => panic!("Modem task already running"),
            }
        }
        let mut int = self.interface.lock().await;

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
                int.command(&common::cip::SetPDPContext {
                    cid: 1,
                    pdp_type: heapless::String::from_str("IP").unwrap(),
                    apn: heapless::String::from_str(env!("APN"))
                        .map_err(|_| anyhow!("Failed to create APN string"))?,
                })
                .await
                .map_err(|e| anyhow!("Failed to set PDP context: {:?}", e))?;

                int.command(&common::cip::AttachToGprsService { attach: 1 })
                    .await
                    .map_err(|e| anyhow!("Failed to attach to GPRS service: {:?}", e))?;

                int.command(&common::cip::StartDataConnection {
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
            .command(&common::general::GetManufacturerInfo)
            .await
            .map_err(|e| anyhow!("Failed to get manufacturer info: {e:?}"))?;
        log::info!("Modem Manufacturer: {}", manufacturer.id);

        let model = int
            .command(&common::general::GetModelInfo)
            .await
            .map_err(|e| anyhow!("Failed to get model info: {e:?}"))?;
        log::info!("Modem model: {}", model.id);

        Ok(())
    }

    async fn dump_status(&self, int: &mut ModemInterface) -> anyhow::Result<()> {
        match int.command(&common::general::GetSignalQuality).await {
            Ok(report) => log::info!("Signal quality: {:?}", report),
            Err(e) => log::error!("Failed to get signal quality: {e:?}"),
        }

        Ok(())
    }
}
