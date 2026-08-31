use core::cmp::Ordering;

use defmt::info;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    priority_channel::{Min, PriorityChannel},
};
use heapless::Vec;

pub static CAN_CMD_DISPATCH_CHANNEL: PriorityChannel<CriticalSectionRawMutex, CanCmd, Min, 8> =
    PriorityChannel::new();

#[derive(PartialEq, PartialOrd, Eq, Ord, defmt::Format)]
pub enum CanCmdKind {
    // The priority of a message is determined by its discriminant
    // higher on this list == a lower discriminant == a higher priority
    WriteReg,
    ReadReg,
}

#[derive(defmt::Format)]
pub struct CanCmd {
    pub kind: CanCmdKind,
    pub buf: Vec<u8, 8>,
}

impl CanCmd {
    pub fn new(kind: CanCmdKind, buf: Vec<u8, 8>) -> Self {
        Self { kind, buf }
    }
}

// We want Ord for the PriorityChannel so that some messages can be handled at a higher priority
// but we only want it to compare the kind of command for ordering, not the data
impl Ord for CanCmd {
    fn cmp(&self, other: &Self) -> Ordering {
        self.kind.cmp(&other.kind)
    }
}

impl PartialOrd for CanCmd {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for CanCmd {
    fn eq(&self, other: &Self) -> bool {
        self.kind == other.kind
    }
}

impl Eq for CanCmd {}

#[embassy_executor::task]
pub async fn handle_cmds() {
    loop {
        let incoming_cmd = CAN_CMD_DISPATCH_CHANNEL.receive().await;

        match incoming_cmd.kind {
            CanCmdKind::WriteReg => {
                info!("Write CMD: {}", incoming_cmd);
            }
            CanCmdKind::ReadReg => {
                info!("Read CMD: {}", incoming_cmd);
            }
        }
    }
}
