/*
* Can module
* Reads messages coming in on the bus and forwards them to the command handler.
* Also declares node_id to bus on startup, and holds CAN related constants and utils.
*/

use defmt::{debug, error, info, unwrap};
use embassy_stm32::{
    Peri, bind_interrupts,
    can::{
        self, BufferedCanReceiver, BufferedCanSender, CanConfigurator, Frame, RxBuf, TxBuf,
        frame::Header,
    },
    peripherals::{FDCAN1, PA11, PA12},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embedded_can::{Id, StandardId};
use heapless::Vec;
use static_cell::StaticCell;

use crate::cmd::{CAN_CMD_DISPATCH_CHANNEL, CanCmd, CanCmdKind};

bind_interrupts!(struct Irqs {
    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

pub const CAN_BUF_SIZE: usize = 64;
pub static CAN_RX_BUF: StaticCell<embassy_stm32::can::RxBuf<CAN_BUF_SIZE>> = StaticCell::new();
pub static CAN_TX_BUF: StaticCell<embassy_stm32::can::TxBuf<CAN_BUF_SIZE>> = StaticCell::new();

const NODE_ID_MASK: u16 = 0x7F;

const CLAIM_NODE_MSG_ID: u16 = 0x180;
const CMD_MSG_ID: u16 = 0x200;

pub static CAN_TX_CHANNEL: Channel<CriticalSectionRawMutex, Frame, 8> = Channel::new();

#[embassy_executor::task]
pub async fn handle_can_tx(mut can_writer: BufferedCanSender, self_node_id: u8) {
    // Declare this board's node_id to the bus
    let claim_node_id_msg = create_claim_node_id_frame(self_node_id);
    can_writer.write(claim_node_id_msg).await;

    loop {
        let tx_frame = CAN_TX_CHANNEL.receive().await;
        can_writer.write(tx_frame).await;
    }
}

#[embassy_executor::task]
pub async fn handle_can_rx(can_reader: BufferedCanReceiver, self_node_id: u8) {
    info!("CAN rx task loop start");
    loop {
        let envelope = match can_reader.receive().await {
            Ok(envelope) => envelope,
            Err(err) => {
                error!("Error in frame: {}", err);
                continue;
            }
        };

        let incoming_msg_id = match envelope.frame.header().id() {
            Id::Standard(id) => id,
            Id::Extended(_) => {
                error!("Unexpected extended id");
                continue;
            }
        };

        let frame = envelope.frame;
        let raw_id = incoming_msg_id.as_raw();
        let base_id = raw_id & !NODE_ID_MASK;
        let recipient_node_id = raw_id & NODE_ID_MASK;

        if recipient_node_id != self_node_id as u16 {
            // msg is not intended for this board,
            // or claimed node id does not conflict
            continue;
        }

        if base_id == CLAIM_NODE_MSG_ID {
            // Another node has already claimed this id
            panic!("Conflicting node id detected - shutting down");
        }

        log_can_msg(raw_id, &frame);

        let data_buf = match Vec::from_slice(frame.data()) {
            Ok(buf) => buf,
            Err(e) => {
                error!("Could not create buffer for CAN frame data: {}", e);
                continue;
            }
        };

        match base_id {
            CMD_MSG_ID => {
                let cmd = CanCmd::new(CanCmdKind::WriteReg, data_buf);
                CAN_CMD_DISPATCH_CHANNEL.send(cmd).await;
            }
            _ => {
                error!("Unhandled CAN message type");
            }
        }
    }
}

pub fn get_can_reader_and_writer(
    can: Peri<'static, FDCAN1>,
    can_rx_pin: Peri<'static, PA11>,
    can_tx_pin: Peri<'static, PA12>,
) -> (BufferedCanReceiver, BufferedCanSender) {
    let mut can_conf = CanConfigurator::new(can, can_rx_pin, can_tx_pin, Irqs);
    can_conf.set_bitrate(1_000_000);
    let can = can_conf.into_normal_mode();
    let can_buffered = can.buffered(
        CAN_TX_BUF.init(TxBuf::<CAN_BUF_SIZE>::new()),
        CAN_RX_BUF.init(RxBuf::<CAN_BUF_SIZE>::new()),
    );

    (can_buffered.reader(), can_buffered.writer())
}

fn create_claim_node_id_frame(node_id: u8) -> Frame {
    let msg = [0; 8];
    let claim_node_header_id = unwrap!(StandardId::new(CLAIM_NODE_MSG_ID + node_id as u16));
    let header: Header = Header::new(Id::Standard(claim_node_header_id), 8, false);
    can::frame::Frame::new(header, &msg).unwrap()
}

fn log_can_msg(id: u16, frame: &Frame) {
    debug!(
        "CAN message received -- id: 0x{:02x} data: {:02x}",
        id,
        frame.data()[0..frame.header().len() as usize],
    )
}
