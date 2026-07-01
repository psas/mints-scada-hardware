#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
// use embassy_stm32::peripherals::CAN1;
// use embassy_stm32::peripherals::pb8;
use embassy_stm32::{
    bind_interrupts,
    can::{
        Can, Fifo, Frame, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler,
        StandardId, TxInterruptHandler, filter::Mask32,
    },
    peripherals::CAN,
};
use embassy_time::Instant;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct CanIrqs {
    CEC_CAN =>
    Rx0InterruptHandler<CAN>,
    Rx1InterruptHandler<CAN>,
    SceInterruptHandler<CAN>,
    TxInterruptHandler<CAN>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

    let mut p = embassy_stm32::init(Default::default());

    info!("initializing CAN driver");
    let mut can = Can::new(p.CAN, p.PA11, p.PA12, CanIrqs);
    can.modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

    info!("configuring CAN");
    can.modify_config()
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        .set_bitrate(1_000_000);

    info!("enabling CAN");
    can.enable().await;

    let mut i: u8 = 0;
    loop {
        let tx_frame = Frame::new_data(unwrap!(StandardId::new(i as _)), &[i]).unwrap();
        can.write(&tx_frame).await;
        info!("Waiting....");

        match can.read().await {
            Ok(envelope) => {
                info!("envelope is defined");
                info!("loopback frame {}", envelope.frame.data());
            }
            Err(e) => {
                info!("{}", e)
            }
        }

        i = i.wrapping_add(1);
    }
}
