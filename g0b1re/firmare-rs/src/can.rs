use defmt::{error, info};
use embassy_stm32::{
    Peri, bind_interrupts,
    can::{self, frame::Header},
    peripherals::{FDCAN1, PA11, PA12},
};
use embassy_time::Timer;
bind_interrupts!(struct Irqs {
    TIM16_FDCAN_IT0 => can::IT0InterruptHandler<FDCAN1>;
    TIM17_FDCAN_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

#[embassy_executor::task]
pub async fn handle_can(
    fdcan1: Peri<'static, FDCAN1>,
    pa11: Peri<'static, PA11>,
    pa12: Peri<'static, PA12>,
) {
    let mut can = can::CanConfigurator::new(fdcan1, pa11, pa12, Irqs);

    can.set_bitrate(1_000_000);

    let mut can = can.start(can::OperatingMode::NormalOperationMode);

    let mut i: u8 = 0;
    let mut last_read_ts = embassy_time::Instant::now();

    loop {
        let msg = [i; 8];
        let id = embedded_can::Id::Standard(embedded_can::StandardId::new(0x123).unwrap());
        let header: Header = Header::new(id, 8, false);
        let frame = can::frame::Frame::new(header, &msg).unwrap();
        info!("Writing frame");

        _ = can.write(&frame).await;

        match can.read().await {
            Ok(envelope) => {
                let (ts, rx_frame) = (envelope.ts, envelope.frame);
                let delta = (ts - last_read_ts).as_millis();
                last_read_ts = ts;
                info!(
                    "Rx: {} {:02x} {}ms",
                    rx_frame.header().len(),
                    rx_frame.data()[0..rx_frame.header().len() as usize],
                    delta,
                )
            }
            Err(_err) => error!("Error in frame"),
        }

        Timer::after_millis(250).await;

        i += 1;
    }
}
