pub mod responses;
pub mod urc;

use super::EmptyResponse;
use atat::atat_derive::AtatCmd;
use responses::*;

#[derive(Clone, AtatCmd)]
#[at_cmd("+CGMI", Manufacturer)]
pub struct GetManufacturerInfo;

#[derive(Clone, AtatCmd)]
#[at_cmd("+CGMM", Model)]
pub struct GetModelInfo;

#[derive(Clone, AtatCmd)]
#[at_cmd("+CLTS", EmptyResponse)]
pub struct EnableGetLocalTimestamp {
    pub enable: u8,
}

#[derive(Clone, AtatCmd)]
#[at_cmd("+CSQ", SignalQuality)]
pub struct GetSignalQuality;
