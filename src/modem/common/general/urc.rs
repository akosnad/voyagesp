use atat::atat_derive::AtatResp;
use atat::heapless::String;

#[derive(Clone, Debug, AtatResp)]
pub struct NewMessageIndication {
    pub data: String<64>,
}
