use alloc::string::String;
use atat::{atat_derive::AtatResp, AtatResp};

#[derive(Clone, Debug, AtatResp)]
pub struct NewMessageIndication {
    pub data: String,
}

// *PSUTTZ: 2024,11,20,20,4,48,\"+4\",0
#[derive(Clone, Debug, AtatResp)]
pub struct LocalTimestamp {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub min: u8,
    pub sec: u8,
    pub time_zone: String,
    pub dst: u8,
}

// DST: 0
#[derive(Clone, Debug, AtatResp)]
pub struct DaylightSavingTime {
    pub dst: u8,
}

// +CIEV: 10,\"21630\",\"Telekom HU\",\"THU\", 0, 0
#[derive(Clone, Debug, AtatResp)]
pub struct IndicatorEvent {
    pub fun: u8,
    pub n: String,
    pub oper: String,
    pub act: String,
    pub rdy: u8,
    pub urc: u8,
}

// +CFUN: 1
#[derive(Clone, Debug, AtatResp)]
pub struct Functionality {
    pub fun: u8,
}

#[derive(Clone, Debug, AtatResp)]
pub struct PinStatus {
    pub status: String,
}
