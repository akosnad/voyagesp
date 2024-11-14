use atat::atat_derive::AtatResp;
use atat::heapless::String;

#[derive(Clone, Debug, AtatResp)]
pub struct Manufacturer {
    pub id: atat::heapless_bytes::Bytes<64>,
}
