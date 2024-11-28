pub mod responses;
pub mod urc;

use super::EmptyResponse;
use atat::{atat_derive::AtatCmd, AtatCmd};
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

#[derive(Clone)]
pub struct DTRControl {
    /// 0 - ignore DTR
    /// 1 - ON->OFF: change to command mode without disconnecting the call
    /// 2 - ON->OFF: change to command mode and disconnect the call (During DTR=off auto-answer is
    ///   off)
    pub dtr: u8,
}
impl AtatCmd for DTRControl {
    type Response = EmptyResponse;

    const MAX_LEN: usize = 10;

    fn write(&self, mut buf: &mut [u8]) -> usize {
        use embedded_io::Write;
        let buf_len = buf.len();
        assert!(buf_len >= Self::MAX_LEN);
        write!(buf, "AT&D{}\r\n", self.dtr).ok();
        buf_len - buf.len()
    }

    fn parse(
        &self,
        resp: Result<&[u8], atat::InternalError>,
    ) -> Result<Self::Response, atat::Error> {
        match resp {
            Ok(_) => Ok(EmptyResponse),
            Err(_) => Err(atat::Error::Parse),
        }
    }
}

#[derive(Clone, AtatCmd, PartialEq)]
#[at_cmd("O", EmptyResponse)]
pub struct EnterDataMode;
