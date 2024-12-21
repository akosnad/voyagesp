use alloc::sync::Arc;
use anyhow::anyhow;
use core::str::FromStr;
use embassy_executor::task;
use embassy_net::{Ipv4Address, StaticConfigV4};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::WithTimeout as _;
use embassy_time::{Duration, Timer};
use esp_hal::{
    gpio::{AnyPin, Input, Output},
    peripherals::UART2,
    uart::Uart,
    Async,
};
use log::info;
use static_cell::{make_static, StaticCell};

use voyagesp as lib;

use lib::at::{general::urc::PinStatus, Urc};

mod interface;

use interface::ModemInterface;

#[derive(Debug, Clone, Default, PartialEq)]
enum ModemState {
    #[default]
    Uninitialized,
    Initializing,
    HardwareReady,
    SimReady,
    SimOnline,
    GprsOnline,
    PPPFailed,
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
        config_sender: embassy_sync::channel::Sender<
            'static,
            CriticalSectionRawMutex,
            StaticConfigV4,
            1,
        >,
    ) -> anyhow::Result<(Self, embassy_net_ppp::Device<'static>)> {
        let (interface, data_tx, data_rx) =
            ModemInterface::new(spawner, dtr, ri, pwrkey, power_on, uart).await?;
        let state: Arc<Mutex<CriticalSectionRawMutex, ModemState>> = Default::default();

        let ppp_state = make_static!(embassy_net_ppp::State::new());
        let (ppp_device, ppp_runner) = embassy_net_ppp::new::<16, 16>(ppp_state);

        spawner
            .spawn(ppp_task(
                data_tx,
                data_rx,
                ppp_runner,
                config_sender,
                state.clone(),
            ))
            .map_err(|e| anyhow!("Failed to spawn PPP task: {:?}", e))?;

        Ok((
            Self {
                interface: Arc::new(Mutex::new(interface)),
                state,
            },
            ppp_device,
        ))
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

        'main: loop {
            let future = urc_subcriber
                .next_message()
                .with_timeout(Duration::from_secs(3));
            if let Ok(embassy_sync::pubsub::WaitResult::Message(urc)) = future.await {
                self.handle_urc(&mut int, urc)
                    .await
                    .map_err(|e| log::error!("Failed to handle URC: {:?}", e))
                    .ok();
            }

            {
                let mut state_guard = self.state.lock().await;
                let pending_state = state_guard.clone();

                match self.handle_state_change(&mut int, pending_state).await {
                    Ok(next_state) => *state_guard = next_state,
                    Err(e) => {
                        log::error!("Failed to handle state change: {:?}", e);
                        *state_guard = ModemState::SimReady;
                        continue 'main;
                    }
                }
            }
        }
    }

    async fn handle_urc(&self, _int: &mut ModemInterface, urc: Urc) -> anyhow::Result<()> {
        match urc {
            Urc::IndicatorEvent(e) => {
                log::info!("Indicator event: {:?}", e);
                let mut state = self.state.lock().await;
                if let ModemState::SimReady = *state {
                    if e.rdy == 0 {
                        *state = ModemState::SimOnline;
                    }
                }
            }
            Urc::PinStatus(PinStatus { status }) => {
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
            ModemState::SimReady => {
                int.command_mode().await?;
                let sig = int
                    .command(&lib::at::general::GetSignalQuality)
                    .await
                    .map_err(|e| anyhow!("Failed to get signal quality: {e:?}"))?;
                if sig.rssi > 0 && sig.ber > 0 {
                    log::info!("Modem signal acquired");
                    Ok(ModemState::SimOnline)
                } else {
                    Ok(current_state)
                }
            }
            ModemState::SimOnline => {
                // Wait for network registration
                Timer::after(Duration::from_secs(5)).await;
                int.command_mode().await?;

                int.command(&lib::at::cip::SetPDPContext {
                    cid: 1,
                    pdp_type: heapless::String::from_str("IP").unwrap(),
                    apn: heapless::String::from_str(env!("APN"))
                        .map_err(|_| anyhow!("Failed to create APN string"))?,
                })
                .await
                .map_err(|e| anyhow!("Failed to set PDP context: {:?}", e))?;

                int.command(&lib::at::cip::AttachToGprsService { attach: 1 })
                    .await
                    .map_err(|e| anyhow!("Failed to attach to GPRS service: {:?}", e))?;

                int.command(&lib::at::cip::StartDataConnection {
                    cid: 1,
                    l2p: heapless::String::from_str("PPP").unwrap(),
                })
                .await
                .map_err(|e| anyhow!("Failed to bring up wireless connection: {:?}", e))?;

                Ok(ModemState::GprsOnline)
            }
            ModemState::PPPFailed => {
                int.command_mode().await?;

                let sig = int
                    .command(&lib::at::general::GetSignalQuality)
                    .await
                    .map_err(|e| anyhow!("Failed to get signal quality: {e:?}"))?;
                if sig.rssi == 0 && sig.ber == 0 {
                    log::warn!("Modem signal lost");
                    return Ok(ModemState::SimReady);
                }
                Ok(ModemState::SimOnline)
            }
            _ => Ok(current_state),
        }
    }

    async fn dump_info(&self, int: &mut ModemInterface) -> anyhow::Result<()> {
        let manufacturer = int
            .command(&lib::at::general::GetManufacturerInfo)
            .await
            .map_err(|e| anyhow!("Failed to get manufacturer info: {e:?}"))?;
        log::info!("Modem Manufacturer: {}", manufacturer.id);

        let model = int
            .command(&lib::at::general::GetModelInfo)
            .await
            .map_err(|e| anyhow!("Failed to get model info: {e:?}"))?;
        log::info!("Modem model: {}", model.id);

        Ok(())
    }
}

#[task]
async fn ppp_task(
    data_tx: interface::DataTx,
    mut data_rx: interface::DataRx,
    mut runner: embassy_net_ppp::Runner<'static>,
    config_sender: embassy_sync::channel::Sender<
        'static,
        CriticalSectionRawMutex,
        StaticConfigV4,
        1,
    >,
    modem_state: Arc<Mutex<CriticalSectionRawMutex, ModemState>>,
) {
    let on_ipv4_up = |ipv4_status: embassy_net_ppp::Ipv4Status| {
        let Some(address) = ipv4_status.address else {
            log::warn!("No address set in ipv4 modem PPP status");
            return;
        };
        let Some(gateway) = ipv4_status.peer_address else {
            log::warn!("No gateway set in ipv4 modem PPP status");
            return;
        };
        let dns_servers: heapless::Vec<Ipv4Address, 3> = if ipv4_status.dns_servers.is_empty() {
            // fallback to cloudflare dns if none specified from ppp
            heapless::Vec::from_slice(&[Ipv4Address::new(1, 1, 1, 1), Ipv4Address::new(1, 0, 0, 1)])
                .unwrap()
        } else {
            ipv4_status
                .dns_servers
                .iter()
                .flatten()
                .map(|addr| Ipv4Address::from_bytes(&addr.0))
                .collect()
        };

        let config = StaticConfigV4 {
            address: embassy_net::Ipv4Cidr::new(
                Ipv4Address::from_bytes(&address.0),
                0, // no network, since we are point-to-point
            ),
            gateway: Some(Ipv4Address::from_bytes(&gateway.0)),
            dns_servers,
        };

        config_sender
            .try_send(config)
            .map_err(|e| anyhow!("Failed to send modem ipv4 config: {:?}", e))
            .ok();
    };

    loop {
        let rw = UnifiedDataRxTx::new(&mut data_rx, &data_tx);
        let config = embassy_net_ppp::Config {
            username: b"",
            password: b"",
        };
        if let Err(e) = runner.run(rw, config, on_ipv4_up).await {
            log::error!("PPP link exited with error: {:?}", e);
            let mut state = modem_state.lock().await;
            if *state == ModemState::GprsOnline {
                *state = ModemState::PPPFailed;
            }
        }
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}

/// Wrapper around DataRx and DataTx to implement embedded_io_async::BufRead and embedded_io_async::Write
struct UnifiedDataRxTx<'a> {
    rx: &'a mut interface::DataRx,
    tx: &'a interface::DataTx,
}
impl<'a> UnifiedDataRxTx<'a> {
    pub fn new(rx: &'a mut interface::DataRx, tx: &'a interface::DataTx) -> Self {
        Self { rx, tx }
    }
}
impl<'a> embedded_io_async::ErrorType for UnifiedDataRxTx<'a> {
    type Error = embedded_io_async::ErrorKind;
}
impl<'a> embedded_io_async::BufRead for UnifiedDataRxTx<'a> {
    async fn fill_buf(&mut self) -> Result<&[u8], Self::Error> {
        Ok(self.rx.fill_buf().await)
    }

    fn consume(&mut self, amt: usize) {
        self.rx.consume(amt)
    }
}
impl<'a> embedded_io_async::Write for UnifiedDataRxTx<'a> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(self.tx.write(buf).await)
    }
}
