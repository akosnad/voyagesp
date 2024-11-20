use alloc::string::String;
use atat::atat_derive::AtatResp;

#[derive(Clone, Debug, AtatResp)]
pub struct CipStatus {
    pub status: String,
}
