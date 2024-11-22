use alloc::string::String;
use atat::atat_derive::AtatResp;

#[derive(Clone, Debug, AtatResp)]
pub struct Manufacturer {
    pub id: String,
}

#[derive(Clone, Debug, AtatResp)]
pub struct Model {
    pub id: String,
}

#[derive(Clone, Debug, AtatResp)]
pub struct SignalQuality {
    pub rssi: u8,
    pub ber: u8,
}
