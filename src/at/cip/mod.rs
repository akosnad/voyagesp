use super::EmptyResponse;
use atat::atat_derive::AtatCmd;

pub mod responses;
use responses::*;

#[derive(Clone, Debug, AtatCmd)]
#[at_cmd("+CIPSTATUS", CipStatus)]
pub struct GetCipStatus;

#[derive(Clone, Debug, AtatCmd)]
#[at_cmd("+CGDCONT", EmptyResponse)]
pub struct SetPDPContext {
    pub cid: u8,
    /// Only "IP" supported
    pub pdp_type: heapless::String<2>,
    pub apn: heapless::String<32>,
}

#[derive(Clone, Debug, AtatCmd)]
#[at_cmd("+CGATT", EmptyResponse)]
pub struct AttachToGprsService {
    /// 0 - Detach, 1 - Attach
    pub attach: u8,
}

#[derive(Clone, Debug, AtatCmd)]
#[at_cmd("+CGDATA", EmptyResponse)]
pub struct StartDataConnection {
    /// Only "PPP" supported
    pub l2p: heapless::String<3>,
    pub cid: u8,
}
