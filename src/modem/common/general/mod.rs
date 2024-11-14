pub mod responses;
pub mod urc;

use atat::atat_derive::AtatCmd;
use responses::*;

#[derive(Clone, AtatCmd)]
#[at_cmd("+CGMI", Manufacturer)]
pub struct GetManufacturerInfo;
