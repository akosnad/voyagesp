pub mod cip;
pub mod general;

use atat::atat_derive::{AtatCmd, AtatResp, AtatUrc};

#[derive(Clone, Debug, AtatResp)]
pub struct EmptyResponse;

#[derive(Clone, AtatCmd)]
#[at_cmd("", EmptyResponse, timeout_ms = 1000)]
pub struct AT;

#[derive(Clone, Debug, AtatUrc)]
pub enum Urc {
    #[at_urc("RDY")]
    Ready,
    #[at_urc("+CPIN")]
    PinStatus(general::urc::PinStatus),
    #[at_urc("+CFUN")]
    Functionality(general::urc::Functionality),
    #[at_urc("Call Ready")]
    CallReady,
    #[at_urc("SMS Ready")]
    SmsReady,
    #[at_urc("*PSUTTZ")]
    LocalTimestamp(general::urc::LocalTimestamp),
    #[at_urc("DST")]
    DaylightSavingTime(general::urc::DaylightSavingTime),
    #[at_urc("+CIEV")]
    IndicatorEvent(general::urc::IndicatorEvent),

    #[at_urc("STATUS")]
    CipStatus(cip::responses::CipStatus),
}
