pub mod general;

use atat::atat_derive::{AtatCmd, AtatResp, AtatUrc};

#[derive(Clone, Debug, AtatResp)]
pub struct EmptyResponse;

#[derive(Clone, AtatCmd)]
#[at_cmd("", EmptyResponse, timeout_ms = 1000)]
pub struct AT;

#[derive(Clone, Debug, AtatUrc)]
pub enum Urc {
    #[at_urc("Call Ready")]
    CallReady,
    #[at_urc("SMS Ready")]
    SmsReady,
}
